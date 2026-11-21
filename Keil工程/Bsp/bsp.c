#include "bsp.h"
#include "Delay.h"

/* 串口发送和电池 ADC 读取的轮询保护。
 * 正常情况下标志位很快置位；超出软件循环上限时跳过本次操作或沿用旧值，避免主循环卡在等待中。
 */
#define BSP_TXE_GUARD           20000u              // 等待串口发送寄存器空的最大轮询次数
#define BSP_EOC_GUARD           20000u              // 等待 ADC 转换完成的最大轮询次数

/* 工程未引入 IWDG 标准外设驱动，直接用寄存器地址访问独立看门狗。 */
#define IWDG_BASE_ADDR          0x40003000UL
#define IWDG_KR                 (*(volatile uint16_t *)(IWDG_BASE_ADDR + 0x00UL))  // 键寄存器，用于解锁、重装及启动
#define IWDG_PR                 (*(volatile uint16_t *)(IWDG_BASE_ADDR + 0x04UL))  // 预分频寄存器
#define IWDG_RLR                (*(volatile uint16_t *)(IWDG_BASE_ADDR + 0x08UL))  // 重装值寄存器
#define IWDG_SR                 (*(volatile uint16_t *)(IWDG_BASE_ADDR + 0x0CUL))  // 预分频与重装更新状态寄存器

/* 调试暂停时允许冻结独立看门狗，避免停在断点上触发复位。 */
#define DBGMCU_CR_ADDR          0xE0042004UL

static volatile uint32_t g_tick_5ms = 0;            // 由 TIM1 更新的全局 5 ms 节拍
static uint16_t g_enc_last_left = 0;                // 上次采样保存的左编码器计数
static uint16_t g_enc_last_right = 0;               // 上次采样保存的右编码器计数
static volatile uint8_t g_stage = 0;                // 当前执行路径的诊断阶段标记
static uint8_t  g_reset_cause = 0;                  // 本次上电时采样的复位原因位图

/* GPIO 初始化助手 */
/**
  * 函    数：用统一本板速度配置一组 GPIO
  * 参    数：port：GPIO 端口，如 GPIOA
  *           pins：要配置的引脚位掩码
  *           mode：输入、推挽输出等 GPIO 模式
  * 返 回 值：无
  */
static void GPIO_Config(GPIO_TypeDef *port, uint16_t pins, GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef gpio;

    gpio.GPIO_Pin = pins;                           // 端口内可一次配置多个引脚
    gpio.GPIO_Mode = mode;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;             // 输出模式使用 50 MHz 档
    GPIO_Init(port, &gpio);
}

/* 指示灯与蜂鸣器 */
/**
  * 函    数：打开或关闭外接状态 LED
  * 参    数：on：0 关闭，非 0 开启
  * 返 回 值：无
  */
void BSP_Led(uint8_t on)
{
    GPIO_WriteBit(LED_PORT, LED_PIN, on ? Bit_SET : Bit_RESET);  // 外置状态灯为高电平点亮
}

/**
  * 函    数：打开或关闭板载状态 LED
  * 参    数：on：0 关闭，非 0 开启
  * 返 回 值：无
  */
void BSP_LedOnboard(uint8_t on)
{
    /* PC13 的板载 LED 低电平点亮。 */
    GPIO_WriteBit(LED_ONBOARD_PORT, LED_ONBOARD_PIN, on ? Bit_RESET : Bit_SET);
}

/**
  * 函    数：打开或关闭有源蜂鸣器
  * 参    数：on：0 关闭，非 0 开启
  * 返 回 值：无
  */
void BSP_Buzzer(uint8_t on)
{
    /* 当前接法下 PB0 为低电平鸣叫，高电平静音。 */
    GPIO_WriteBit(BUZZER_PORT, BUZZER_PIN, on ? Bit_RESET : Bit_SET);
}

/* 低电平有效的按键输入 */
/**
  * 函    数：读取启停键原始状态
  * 参    数：无
  * 返 回 值：1 表示按下，0 表示松开
  */
uint8_t BSP_KeyStart(void)
{
    return GPIO_ReadInputDataBit(KEY_START_PORT, KEY_START_PIN) == Bit_RESET;
}

/**
  * 函    数：读取速度键原始状态
  * 参    数：无
  * 返 回 值：1 表示按下，0 表示松开
  */
uint8_t BSP_KeySpeed(void)
{
    return GPIO_ReadInputDataBit(KEY_SPEED_PORT, KEY_SPEED_PIN) == Bit_RESET;
}

/**
  * 函    数：读取模式键原始状态
  * 参    数：无
  * 返 回 值：1 表示按下，0 表示松开
  */
uint8_t BSP_KeyMode(void)
{
    return GPIO_ReadInputDataBit(KEY_MODE_PORT, KEY_MODE_PIN) == Bit_RESET;
}

/* 灰度模块改用 USART1，原并行 OUT 引脚按下拉输入闲置。
 * 模块 OUT 电平可能接近 4 V，原排线应保持断开。
 */
/**
  * 函    数：将原八路灰度并行输入配置为下拉闲置
  * 参    数：无
  * 返 回 值：无
  */
static void BSP_GrayPinsIdle(void)
{
    GPIO_Config(GPIOA, GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_15,
                GPIO_Mode_IPD);
    GPIO_Config(GPIOB, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_14,
                GPIO_Mode_IPD);
}

/* TB6612 双路电机驱动 */
/**
  * 函    数：根据有符号命令设置单个 H 桥的两路方向引脚
  * 参    数：in1_port：方向输入 1 的 GPIO 端口
  *           in1_pin：方向输入 1 的引脚掩码
  *           in2_port：方向输入 2 的 GPIO 端口
  *           in2_pin：方向输入 2 的引脚掩码
  *           duty：带符号的 PWM 比较值，正前进、负后退、0 制动
  *           invert：非 0 时反相前进与后退的方向信号
  * 返 回 值：无，PWM 比较值由 BSP_MotorSet 写入
  */
static void Motor_One(GPIO_TypeDef *in1_port, uint16_t in1_pin,
                      GPIO_TypeDef *in2_port, uint16_t in2_pin,
                      int16_t duty, uint8_t invert)
{
    uint8_t forward;

    if (duty > 0)       { forward = 1; }
    else if (duty < 0)  { forward = 0; duty = (int16_t)(-duty); }
    else
    {
        /* 零命令将方向引脚同时置高，使内轮在单轮转向时短路制动。 */
        GPIO_WriteBit(in1_port, in1_pin, Bit_SET);
        GPIO_WriteBit(in2_port, in2_pin, Bit_SET);
        return;
    }

    if (invert) { forward = (uint8_t)!forward; }    // 根据接线标定反相正、反转定义

    /* 方向引脚一高一低，控制 H 桥导通方向。 */
    if (forward)
    {
        GPIO_WriteBit(in1_port, in1_pin, Bit_SET);
        GPIO_WriteBit(in2_port, in2_pin, Bit_RESET);
    }
    else
    {
        GPIO_WriteBit(in1_port, in1_pin, Bit_RESET);
        GPIO_WriteBit(in2_port, in2_pin, Bit_SET);
    }
}

/**
  * 函    数：设置双轮电机方向和 PWM 比较值
  * 参    数：left：左轮带符号 PWM 值，超出范围限幅至 ±PWM_MAX
  *           right：右轮带符号 PWM 值，含义同左轮
  * 返 回 值：无
  */
void BSP_MotorSet(int16_t left, int16_t right)
{
    /* 左、右命令单独限幅。方向由符号确定，占空比使用绝对值。 */
    if (left > PWM_MAX)   { left = PWM_MAX; }
    if (left < -PWM_MAX)  { left = -PWM_MAX; }
    if (right > PWM_MAX)  { right = PWM_MAX; }
    if (right < -PWM_MAX) { right = -PWM_MAX; }

    Motor_One(MOTOR_LEFT_AIN1_PORT, MOTOR_LEFT_AIN1_PIN,
              MOTOR_LEFT_AIN2_PORT, MOTOR_LEFT_AIN2_PIN,
              left, MOTOR_LEFT_INVERT);

    Motor_One(MOTOR_RIGHT_BIN1_PORT, MOTOR_RIGHT_BIN1_PIN,
              MOTOR_RIGHT_BIN2_PORT, MOTOR_RIGHT_BIN2_PIN,
              right, MOTOR_RIGHT_INVERT);

    TIM_SetCompare3(TIM2, (left  >= 0) ? (uint16_t)left  : (uint16_t)(-left));  // 通过 TIM2 通道 3 驱动左轮 PWM
    TIM_SetCompare4(TIM2, (right >= 0) ? (uint16_t)right : (uint16_t)(-right));  // 通过 TIM2 通道 4 驱动右轮 PWM
}

/**
  * 函    数：对左右电机施加短路制动
  * 参    数：无
  * 返 回 值：无
  */
void BSP_MotorBrake(void)
{
    /* 将两侧 IN1、IN2 都设为高电平，并清零 PWM。 */
    GPIO_WriteBit(MOTOR_LEFT_AIN1_PORT,  MOTOR_LEFT_AIN1_PIN,  Bit_SET);
    GPIO_WriteBit(MOTOR_LEFT_AIN2_PORT,  MOTOR_LEFT_AIN2_PIN,  Bit_SET);
    GPIO_WriteBit(MOTOR_RIGHT_BIN1_PORT, MOTOR_RIGHT_BIN1_PIN, Bit_SET);
    GPIO_WriteBit(MOTOR_RIGHT_BIN2_PORT, MOTOR_RIGHT_BIN2_PIN, Bit_SET);
    TIM_SetCompare3(TIM2, 0);
    TIM_SetCompare4(TIM2, 0);
}

/**
  * 函    数：清除双轮 PWM 并将各路方向引脚拉低
  * 参    数：无
  * 返 回 值：无
  */
void BSP_MotorCoast(void)
{
    /* 将两个方向引脚组合清零，再将比较值设为 0。 */
    GPIO_WriteBit(MOTOR_LEFT_AIN1_PORT,  MOTOR_LEFT_AIN1_PIN,  Bit_RESET);
    GPIO_WriteBit(MOTOR_LEFT_AIN2_PORT,  MOTOR_LEFT_AIN2_PIN,  Bit_RESET);
    GPIO_WriteBit(MOTOR_RIGHT_BIN1_PORT, MOTOR_RIGHT_BIN1_PIN, Bit_RESET);
    GPIO_WriteBit(MOTOR_RIGHT_BIN2_PORT, MOTOR_RIGHT_BIN2_PIN, Bit_RESET);
    TIM_SetCompare3(TIM2, 0);
    TIM_SetCompare4(TIM2, 0);
}

/* 双轮正交编码器 */
/**
  * 函    数：读取自上次调用以来的左右轮有符号计数差
  * 参    数：left：写入左轮计数差的指针
  *           right：写入右轮计数差的指针
  * 返 回 值：无，前进方向按标定后为正
  */
void BSP_EncoderRead(int16_t *left, int16_t *right)
{
    uint16_t now_left  = (uint16_t)TIM_GetCounter(TIM3);  // 左轮编码器连接 TIM3
    uint16_t now_right = (uint16_t)TIM_GetCounter(TIM4);  // 右轮编码器连接 TIM4
    /* 用 16 位有符号差分处理计数器回绕，每周期实际变化应小于半个计数范围。 */
    int16_t d_left  = (int16_t)(now_left  - g_enc_last_left);
    int16_t d_right = (int16_t)(now_right - g_enc_last_right);

    g_enc_last_left  = now_left;                    // 更新下一次差分的参考计数
    g_enc_last_right = now_right;

    *left  = (int16_t)(d_left  * ENCODER_LEFT_SIGN);  // 应用左轮编码器方向因子
    *right = (int16_t)(d_right * ENCODER_RIGHT_SIGN);  // 应用右轮编码器方向因子
}

/**
  * 函    数：清零双轮硬件编码器计数及软件差分基准
  * 参    数：无
  * 返 回 值：无
  */
void BSP_EncoderReset(void)
{
    TIM_SetCounter(TIM3, 0);
    TIM_SetCounter(TIM4, 0);
    g_enc_last_left = 0;
    g_enc_last_right = 0;
}

/* 电池电压 ADC 采样 */
/**
  * 函    数：对电池分压输入做 8 次 ADC 采样并平均
  * 参    数：无
  * 返 回 值：0～4095 的 ADC 原码，等待超时时返回上次成功值
  */
uint16_t BSP_AdcReadBattery(void)
{
    static uint16_t last = 0;
    uint32_t sum = 0;
    uint8_t i;

    /* 连续做 8 次转换并求均值，以减轻电机 PWM 和地线扰动造成的显示跳动。 */
    for (i = 0; i < 8; i++)
    {
        uint32_t guard = BSP_EOC_GUARD;

        ADC_SoftwareStartConvCmd(ADC1, ENABLE);     // 由软件触发一次 ADC1 转换
        while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)  // 轮询本次转换完成标志
        {
            /* 电压或时钟异常时转换可能无法完成。等待到限后沿用上次成功采样值。 */
            if (--guard == 0u)
            {
                return last;
            }
        }
        sum += ADC_GetConversionValue(ADC1);        // 累加本次转换结果
    }

    last = (uint16_t)(sum / 8);                     // 保存 8 次转换平均值，供下次异常时回用
    return last;
}

/* 调试串口发送，与灰度控制命令共用 USART1 */
/**
  * 函    数：等待 USART1 发送寄存器可写，再写入一个字节
  * 参    数：c：待发送字符或任意 8 位协议字节
  * 返 回 值：无，等待超时会锁定后续发送直至重启
  */
void BSP_UartSendChar(char c)
{
    static uint8_t tx_stuck = 0;                    // 记录一次发送超时，避免对每个后续字节重复长等待
    uint32_t guard = BSP_TXE_GUARD;

    /* TXE 长时间不置位时放弃，并锁定本次通电期间的后续发送。 */
    if (tx_stuck)
    {
        return;
    }

    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)  // TXE 表示数据寄存器空，可继续写下一字节
    {
        if (--guard == 0u)
        {
            tx_stuck = 1;
            return;
        }
    }
    USART_SendData(USART1, (uint16_t)(uint8_t)c);   // 按无符号 8 位数据写入串口数据寄存器
}

/**
  * 函    数：逐字节发送以空字符结尾的字符串
  * 参    数：s：待发送字符串的首地址
  * 返 回 值：无
  */
void BSP_UartSendString(const char *s)
{
    while (*s)
    {
        BSP_UartSendChar(*s++);                     // 发送当前位置的字符后将指针移到下一字符
    }
}

/**
  * 函    数：将有符号整数转为十进制文本输出
  * 参    数：v：要输出的整数，负值取反时须在 int32_t 可表示范围内
  * 返 回 值：无
  */
void BSP_UartSendInt(int32_t v)
{
    char buf[12];
    uint8_t i = 0;
    uint8_t neg = 0;
    uint32_t u;

    if (v < 0) { neg = 1; u = (uint32_t)(-v); }     // 保存符号，用无符号量拆分各位
    else       { u = (uint32_t)v; }

    if (u == 0)
    {
        buf[i++] = '0';
    }
    else
    {
        while (u > 0 && i < sizeof(buf))
        {
            buf[i++] = (char)('0' + (u % 10));      // 从最低位开始提取十进制数字
            u /= 10;
        }
    }

    if (neg) { BSP_UartSendChar('-'); }
    while (i > 0)
    {
        BSP_UartSendChar(buf[--i]);                 // 逆序输出缓存中的数字，恢复通常显示顺序
    }
}

/* 全局控制时基 */
/**
  * 函    数：推进一个全局控制节拍
  * 参    数：无
  * 返 回 值：无
  */
void BSP_Tick5ms(void)
{
    g_tick_5ms++;                                   // 由 TIM1 更新中断每 5 ms 累加一次
}

/**
  * 函    数：获取当前全局控制节拍
  * 参    数：无
  * 返 回 值：以 5 ms 为单位的无符号节拍计数
  */
uint32_t BSP_GetTick5ms(void)
{
    return g_tick_5ms;
}

/* 独立看门狗及故障阶段标记 */
/**
  * 函    数：设置独立看门狗的预分频和重装值并启动
  * 参    数：timeout_ms：名义超时，单位 ms，0 表示禁用
  * 返 回 值：无
  */
void BSP_WdgInit(uint16_t timeout_ms)
{
    uint32_t reload;

    if (timeout_ms == 0u) { return; }               // 配置为 0 时不启动独立看门狗

    /* 先限制毫秒参数，再根据名义 LSI 频率折算为 12 位重装值。 */
    if (timeout_ms < 50u)    { timeout_ms = 50u; }
    if (timeout_ms > 26000u) { timeout_ms = 26000u; }

    /* 按 LSI 约 40 kHz 和 64 分频计算，每个计数约 1.6 ms。 */
    reload = (uint32_t)timeout_ms * 625u / 1000u;
    if (reload < 10u)    { reload = 10u; }
    if (reload > 0x0FFFu) { reload = 0x0FFFu; }

    *(volatile uint32_t *)DBGMCU_CR_ADDR |= (1u << 8);  // 内核被调试器暂停时冻结独立看门狗

    IWDG_KR = 0x5555u;                              // 解锁预分频和重装寄存器
    IWDG_PR = 4u;                                   // PR 为 4 对应 64 分频
    IWDG_RLR = (uint16_t)reload;                    // 写入计算并限幅后的重装值
    {
        uint32_t guard = 200000u;                   // 有限次等待寄存器更新完成
        while (((IWDG_SR & 0x03u) != 0u) && (--guard != 0u))
        {
        }
    }
    IWDG_KR = 0xAAAAu;                              // 将重装值装入计数器
    IWDG_KR = 0xCCCCu;                              // 启动独立看门狗
}

/**
  * 函    数：重装独立看门狗计数器以维持运行
  * 参    数：无
  * 返 回 值：无
  */
void BSP_WdgFeed(void)
{
    IWDG_KR = 0xAAAAu;
}

/**
  * 函    数：获取本次启动的复位来源快照
  * 参    数：无
  * 返 回 值：复位原因位图，位定义见 bsp.h
  */
uint8_t BSP_ResetCause(void)
{
    return g_reset_cause;
}

/**
  * 函    数：设置故障日志使用的当前代码阶段
  * 参    数：stage：BSP_STAGE_* 阶段编号
  * 返 回 值：无
  */
void BSP_Stage(uint8_t stage)
{
    g_stage = stage;
}

/**
  * 函    数：获取最近一次写入的执行阶段
  * 参    数：无
  * 返 回 值：BSP_STAGE_* 阶段编号
  */
uint8_t BSP_GetStage(void)
{
    return g_stage;
}

/**
  * 函    数：将多个 RCC 复位标志压缩成位图并清除硬件标志
  * 参    数：无
  * 返 回 值：无
  */
static void BSP_ResetCauseSample(void)
{
    uint8_t c = 0;

    /* 一次启动可能读取到多个复位原因，逐项置位保留。 */
    if (RCC_GetFlagStatus(RCC_FLAG_PORRST)  != RESET) { c |= 0x01u; }
    if (RCC_GetFlagStatus(RCC_FLAG_PINRST)  != RESET) { c |= 0x02u; }
    if (RCC_GetFlagStatus(RCC_FLAG_SFTRST)  != RESET) { c |= 0x04u; }
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) { c |= 0x08u; }
    if (RCC_GetFlagStatus(RCC_FLAG_WWDGRST) != RESET) { c |= 0x10u; }
    if (RCC_GetFlagStatus(RCC_FLAG_LPWRRST) != RESET) { c |= 0x20u; }

    g_reset_cause = c;                              // 保存本次复位来源供启动日志读取
    RCC_ClearFlag();                                // 清除 RCC 复位标志，避免沿用到下次启动
}

/**
  * 函    数：将两路 LED 引脚初始化为输出并熄灭
  * 参    数：无
  * 返 回 值：无
  */
static void BSP_LedInit(void)
{
    GPIO_Config(LED_PORT, LED_PIN, GPIO_Mode_Out_PP);
    GPIO_Config(LED_ONBOARD_PORT, LED_ONBOARD_PIN, GPIO_Mode_Out_PP);
    BSP_Led(0);
    BSP_LedOnboard(0);
}

/**
  * 函    数：将蜂鸣器引脚初始化为输出并关闭
  * 参    数：无
  * 返 回 值：无
  */
static void BSP_BuzzerInit(void)
{
    GPIO_Config(BUZZER_PORT, BUZZER_PIN, GPIO_Mode_Out_PP);
    BSP_Buzzer(0);
}

/**
  * 函    数：将三枚按键配置为内部上拉输入
  * 参    数：无
  * 返 回 值：无
  */
static void BSP_KeyInit(void)
{
    GPIO_Config(KEY_START_PORT, KEY_START_PIN, GPIO_Mode_IPU);
    GPIO_Config(KEY_SPEED_PORT, KEY_SPEED_PIN, GPIO_Mode_IPU);
    GPIO_Config(KEY_MODE_PORT,  KEY_MODE_PIN,  GPIO_Mode_IPU);
}

/**
  * 函    数：配置 TB6612 方向口及 TIM2 双路 PWM
  * 参    数：无
  * 返 回 值：无
  */
static void BSP_MotorInit(void)
{
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef oc;
    GPIO_InitTypeDef gpio;

    /* 将左右 H 桥方向引脚配置为推挽输出。 */
    GPIO_Config(MOTOR_LEFT_AIN1_PORT,
                MOTOR_LEFT_AIN1_PIN | MOTOR_LEFT_AIN2_PIN, GPIO_Mode_Out_PP);
    GPIO_Config(MOTOR_RIGHT_BIN1_PORT,
                MOTOR_RIGHT_BIN1_PIN | MOTOR_RIGHT_BIN2_PIN, GPIO_Mode_Out_PP);

    /* PA2 为 TIM2_CH3 左轮 PWM，PA3 为 TIM2_CH4 右轮 PWM。 */
    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;               // PWM 引脚配置为定时器复用推挽
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    tb.TIM_Prescaler = 0;                           // 定时器不分频，直接采用定时器时钟
    tb.TIM_Period = PWM_MAX;                        // 设定计数周期；72 MHz 下得到 20 kHz PWM
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &tb);

    oc.TIM_OCMode = TIM_OCMode_PWM1;                // PWM1 模式，比较值决定有效脉宽
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_Pulse = 0;                               // 初始占空比为 0
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    /* 两个通道使用相同的 PWM 模式。开启预装载，使比较值在更新事件处生效。 */
    TIM_OC3Init(TIM2, &oc);
    TIM_OC4Init(TIM2, &oc);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);                          // 启用 TIM2 计数

    BSP_MotorCoast();                               // 初始化结束时清除驱动输出
}

/**
  * 函    数：配置左右轮的 TIM3、TIM4 正交解码
  * 参    数：无
  * 返 回 值：无
  */
static void BSP_EncoderInit(void)
{
    TIM_TimeBaseInitTypeDef tb;
    TIM_ICInitTypeDef ic;
    GPIO_InitTypeDef gpio;

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_IPU;                 // 编码器 A/B 输入使用上拉
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);                        // PA6、PA7 接左轮编码器
    GPIO_Init(GPIOB, &gpio);                        // PB6、PB7 接右轮编码器

    tb.TIM_Prescaler = 0;
    tb.TIM_Period = 65535;                          // 16 位计数范围，允许自由回绕
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM3, &tb);
    TIM_TimeBaseInit(TIM4, &tb);

    /* TI12 模式对两路信号联合计数，计数方向由 A/B 相位决定。 */
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    ic.TIM_Channel = TIM_Channel_1;
    ic.TIM_ICPolarity = TIM_ICPolarity_Rising;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    ic.TIM_ICFilter = 6;                            // 使用输入数字滤波降低毛刺干扰
    TIM_ICInit(TIM3, &ic);
    TIM_ICInit(TIM4, &ic);

    ic.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM3, &ic);
    TIM_ICInit(TIM4, &ic);

    /* 将两路计数从零开始，并使能定时器。 */
    TIM_SetCounter(TIM3, 0);
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

/**
  * 函    数：配置 ADC1 通道 9 的单次电池电压采样并执行校准
  * 参    数：无
  * 返 回 值：无
  */
static void BSP_AdcInit(void)
{
    ADC_InitTypeDef adc;

    GPIO_Config(GPIOB, GPIO_Pin_1, GPIO_Mode_AIN);  // PB1 对应 ADC1 通道 9

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);               // PCLK2 六分频作为 ADC 时钟
    ADC_DeInit(ADC1);

    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;           // 每次软件触发只采集一次
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &adc);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_55Cycles5);  // 选择通道 9，采样时间为 55.5 个 ADC 时钟周期
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1) == SET)  // 等待复位校准完成
    {
    }
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1) == SET)   // 等待 ADC 校准完成，启动阶段在此轮询
    {
    }
}

/**
  * 函    数：初始化灰度与调试共用的 USART1
  * 参    数：无
  * 返 回 值：无
  */
static void BSP_UartInit(void)
{
    USART_InitTypeDef uart;
    GPIO_InitTypeDef gpio;

    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;               // PA9 复用推挽输出，为 STM32 TX
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IPU;                 // PA10 配置为上拉输入，为 STM32 RX
    GPIO_Init(GPIOA, &gpio);

    uart.USART_BaudRate = GRAY_UART_BAUD;           // 灰度模块和调试共用 115200 波特率
    /* 配置 8N1，无硬件流控，同时开启收发。 */
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &uart);
    USART_Cmd(USART1, ENABLE);                      // 开启 USART1，接收中断在 Gray_Init 中配置
}

/**
  * 函    数：配置 TIM1 为 200 Hz 更新中断，等待显式启动
  * 参    数：无
  * 返 回 值：无
  */
static void BSP_TickInit(void)
{
    TIM_TimeBaseInitTypeDef tb;
    NVIC_InitTypeDef nvic;

    tb.TIM_Prescaler = 7199;                        // 72 MHz 经 7200 分频后为 10 kHz
    tb.TIM_Period = 49;                             // 每 50 个计数更新一次，频率 200 Hz
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &tb);

    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);     // 先清初始化留下的更新标志
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    nvic.NVIC_IRQChannel = TIM1_UP_IRQn;
    /* TIM1 抢占优先级为 1，低于 USART1 接收中断。
     * 控制周期内的软件 I²C 读数约耗 1.5 ms。允许 USART1 抢占，可减轻接收数据被覆盖。
     */
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

/**
  * 函    数：清零 TIM1 计数并启动 5 ms 控制时基
  * 参    数：无
  * 返 回 值：无
  */
void BSP_TickStart(void)
{
    TIM_SetCounter(TIM1, 0);
    TIM_Cmd(TIM1, ENABLE);
}

/**
  * 函    数：初始化 STM32 板级时钟、引脚与外设
  * 参    数：无
  * 返 回 值：无
  */
void BSP_Init(void)
{
    BSP_Stage(BSP_STAGE_BOOT);
    BSP_ResetCauseSample();                         // 要在任何代码清除 RCC 标志前采样复位来源

    /* 打开 GPIO、AFIO、ADC、USART 和各定时器所需的 APB 时钟。 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_ADC1 | RCC_APB2Periph_USART1 |
                           RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 |
                           RCC_APB1Periph_TIM4, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  // NVIC 分组 2，分配抢占和响应优先级

    /* 关闭 JTAG 释放 PB3、PB4、PA15，保留 PA13/PA14 的 SWD。 */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    Delay_Init();                                   // 开启延时所需的 DWT 周期计数器

    BSP_LedInit();
    BSP_BuzzerInit();
    BSP_KeyInit();
    BSP_GrayPinsIdle();                             // 将不再使用的八路灰度并行引脚设为下拉
    BSP_MotorInit();
    BSP_EncoderInit();                              // 启动左右编码器计数
    BSP_AdcInit();                                  // 初始化并校准电池电压 ADC
    BSP_UartInit();
    BSP_TickInit();                                 // 仅配置 TIM1，等主程序完成初始化后启动
}
