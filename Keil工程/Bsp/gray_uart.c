#include "stm32f10x.h"
#include "gray_uart.h"
#include "app_config.h"
#include "bsp.h"

/* CH32V006 八路灰度模块的串口简化协议驱动。
 * 请求格式为 A5 CMD ARGS，响应为 5A CMD STATUS PAYLOAD。
 * 推送模式 0x02 的一帧为 0x5B 加 8 个小端 16 位字段，总共 17 字节，每个数值有效 12 位。
 * 寄存器 0x15 设为 0x02 后，模块约每 10 ms 推送一帧。模块复位会丢失此设置，Gray_Poll 会在超时后重发命令。
 */

/* 串口协议常量 */
#define GRAY_REQ_HEAD           0xA5u               // STM32 发往灰度模块的请求帧头
#define GRAY_PUSH_HEAD          0x5Bu               // 灰度模块主动推送数据的帧头

#define GRAY_CMD_READ_REG       0x02u
#define GRAY_CMD_WRITE_REG      0x03u               // 写寄存器的命令字
#define GRAY_REG_PUSH_MODE      0x15u               // 串口推送模式的寄存器编号
#define GRAY_PUSH_MODE_GRAY8    0x02u               // 选择主动推送八路灰度原值

#define GRAY_FRAME_LEN          17u                 // 1 字节帧头加 8 路各 2 字节

/* 串口接收环形缓冲区为 256 字节，保留一个空槽区分满和空。
 * USART1 写入，5 ms 控制任务读取；缓冲满时忽略新字节。
 */
#define GRAY_RING_SIZE          256u
#define GRAY_RING_MASK          (GRAY_RING_SIZE - 1u)  // 由于长度为 256，掩码可实现索引回绕

static volatile uint8_t  s_rx[GRAY_RING_SIZE];
static volatile uint8_t  s_rx_head = 0u;            // 下一个可写位置，由接收中断推进
static volatile uint8_t  s_rx_tail = 0u;            // 下一个可读位置，由 Gray_Task 推进

/* 当前组帧进度 */
static uint8_t  s_frame[GRAY_FRAME_LEN];
static uint8_t  s_frame_len = 0u;

/* 最近一次有效帧及其统计 */
static uint16_t s_raw[GRAY_CHANNELS];               // 最新有效帧的八路原始 ADC 值
static uint8_t  s_bits = 0u;                        // 阈值化后的灰度位图，离线时保留
static uint8_t  s_online = 0u;
static volatile uint16_t s_frames = 0u;             // 接受的帧数，饱和于 uint16_t 上限
static volatile uint8_t  s_errors = 0u;             // 高字节非法而拒收的帧数，饱和于 uint8_t 上限
static volatile uint32_t s_last_frame_tick = 0u;    // 最近有效帧到达时的 5 ms 节拍

static uint32_t s_last_cmd_tick = 0u;
static uint8_t  s_cmd_pending = 1u;
static uint8_t  s_cmd_announced = 0u;
static uint8_t  s_link_announced = 0u;
static uint32_t s_announce_tick = 0u;

/* 位图构造与组帧 */
/**
  * 函    数：用阈值把最新的八路 ADC 值转为黑白位图
  * 参    数：无
  * 返 回 值：位值 1 表示黑，位序可由 GRAY_REVERSE 翻转
  */
static uint8_t gray_build_bits(void)
{
    uint8_t i;
    uint8_t bits = 0u;

    for (i = 0u; i < GRAY_CHANNELS; i++)
    {
        uint8_t black;

#if GRAY_BLACK_IS_HIGH
        black = (s_raw[i] > (uint16_t)GRAY_BLACK_ADC) ? 1u : 0u;  // 默认高于阈值判黑
#else
        black = (s_raw[i] <= (uint16_t)GRAY_BLACK_ADC) ? 1u : 0u;  // 低值判黑的替代极性
#endif

        if (black)
        {
            bits |= (uint8_t)(1u << i);             // 将该通道标为黑色
        }
    }

/* 可按安装方向翻转通道位序。 */
#if GRAY_REVERSE
    {
        uint8_t r = 0u;
        for (i = 0u; i < GRAY_CHANNELS; i++)
        {
            if (bits & (uint8_t)(1u << i))
            {
                r |= (uint8_t)(1u << (GRAY_CHANNELS - 1u - i));
            }
        }
        bits = r;
    }
#endif

    return bits;
}

/**
  * 函    数：校验一帧数据并更新八路读数与链路状态
  * 参    数：无
  * 返 回 值：无，非法帧只增加坏帧统计
  */
static void gray_accept_frame(void)
{
    uint8_t i;

    /* 12 位原始值的高字节只允许 0x00～0x0F。
     * 若任一路超出范围，丢弃整帧并继续寻找帧头。
     */
    for (i = 0u; i < GRAY_CHANNELS; i++)
    {
        if ((s_frame[2u + 2u * i] & 0xF0u) != 0u)
        {
            if (s_errors < 0xFFu) { s_errors++; }   // 坏帧计数采用饱和处理
            return;
        }
    }

    for (i = 0u; i < GRAY_CHANNELS; i++)
    {
        /* 将低字节和高字节按小端顺序合并。 */
        s_raw[i] = (uint16_t)s_frame[1u + 2u * i] |
                   ((uint16_t)s_frame[2u + 2u * i] << 8);
    }

    s_bits = gray_build_bits();                     // 将通过校验的全部读数转换成黑白位图
    s_online = 1u;                                  // 只有接收有效帧才重新判定在线
    s_last_frame_tick = BSP_GetTick5ms();           // 记录本帧到达节拍，用于离线超时

    if (s_frames < 0xFFFFu)
    {
        s_frames++;                                 // 有效帧计数饱和后保持最大值
    }
}

/**
  * 函    数：将一个串口字节输入定长灰度帧解析状态机
  * 参    数：b：串口接收的一个字节
  * 返 回 值：无
  */
static void gray_parse_byte(uint8_t b)
{
    if (s_frame_len == 0u)                          // 帧长为 0 表示正在查找帧头
    {
        if (b == GRAY_PUSH_HEAD)                    // 只接受 0x5B 作为推送帧开始
        {
            s_frame[0] = b;
            s_frame_len = 1u;
        }
        return;
    }

    s_frame[s_frame_len] = b;                       // 帧头找到后，后续字节依次进入当前帧
    s_frame_len++;

    if (s_frame_len >= GRAY_FRAME_LEN)
    {
        s_frame_len = 0u;                           // 完整帧交付后恢复寻找帧头的状态
        gray_accept_frame();
    }
}

/**
  * 函    数：通过 USART1 请求灰度模块推送八路原始数据
  * 参    数：无
  * 返 回 值：无
  */
static void gray_send_push_cmd(void)
{
    /* 发送 A5 03 15 02，将寄存器 0x15 写成 0x02。 */
    BSP_UartSendChar((char)GRAY_REQ_HEAD);
    BSP_UartSendChar((char)GRAY_CMD_WRITE_REG);
    BSP_UartSendChar((char)GRAY_REG_PUSH_MODE);
    BSP_UartSendChar((char)GRAY_PUSH_MODE_GRAY8);
}

/* 灰度模块公开接口 */
/**
  * 函    数：清空灰度驱动状态并配置 USART1 接收中断
  * 参    数：无
  * 返 回 值：无
  */
void Gray_Init(void)
{
    NVIC_InitTypeDef nvic;

    /* 初始化接收读写位置、组帧进度、状态及计数。 */
    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_frame_len = 0u;
    s_bits = 0u;
    s_online = 0u;
    s_frames = 0u;
    s_errors = 0u;
    s_cmd_pending = 1u;                             // 标记首个推送命令尚未发送
    s_cmd_announced = 0u;
    s_link_announced = 0u;
    s_announce_tick = 0u;

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  // 接收数据寄存器非空时触发中断

    /* USART1 的抢占优先级高于 200 Hz 的 TIM1 控制中断。
     * 串口中断只将字节写入缓冲区，可在软件 I²C 访问期间及时接收。
     */
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    /* 将上次命令节拍预置为一个重试周期之前，使首次 Gray_Poll 能立即发命令。 */
    s_last_cmd_tick = BSP_GetTick5ms() - (uint32_t)(GRAY_LINK_RETRY_MS / 5);
}

/**
  * 函    数：排空 USART1 接收队列、解析数据并检查链路超时
  * 参    数：无
  * 返 回 值：无，由 5 ms 控制循环调用
  */
void Gray_Task(void)
{
    uint32_t age;

    while (s_rx_tail != s_rx_head)
    {
        uint8_t b = s_rx[s_rx_tail];                // 读取缓冲区中最旧的未解析字节
        s_rx_tail = (uint8_t)((s_rx_tail + 1u) & GRAY_RING_MASK);  // 先释放已取出的槽位，再向解析器送入字节
        gray_parse_byte(b);
    }

    age = (uint32_t)BSP_GetTick5ms() - s_last_frame_tick;  // 用无符号节拍差计算帧龄，允许时基回绕
    if (age > (uint32_t)(GRAY_LINK_TIMEOUT_MS / 5))
    {
        s_online = 0u;                              // 超时只撤销在线标志，缓存原始值与位图留待诊断
    }
}

/**
  * 函    数：在主循环中维护灰度重连并提示链路恢复
  * 参    数：无
  * 返 回 值：无
  */
void Gray_Poll(void)
{
    uint32_t now = BSP_GetTick5ms();
    uint32_t since_cmd = now - s_last_cmd_tick;
    uint32_t since_frame = now - s_last_frame_tick;

    /* 每次连通或重连只提示一行，并限制提示输出间隔。 */
    if (s_online != 0u)
    {
        if ((s_link_announced == 0u) && ((now - s_announce_tick) > 200u))
        {
            s_link_announced = 1u;
            s_announce_tick = now;
            BSP_UartSendString("\r\nGRAY link ok, g=");
            BSP_UartSendInt((int32_t)s_bits);
            BSP_UartSendString("\r\n");
        }
    }
    else
    {
        s_link_announced = 0u;                      // 离线后清除提示标记，使下一次恢复时能够再次打印
    }

    if (since_cmd < (uint32_t)(GRAY_LINK_RETRY_MS / 5))  // 限制两次重配置命令之间的最短间隔
    {
        return;
    }

    if ((s_cmd_pending == 0u) && (since_frame <= (uint32_t)(GRAY_LINK_TIMEOUT_MS / 5)))  // 首命令已发且近期有有效帧时，无需重发
    {
        return;
    }

    if (s_cmd_announced == 0u)
    {
        /* 后续 4 个二进制字节同样会出现在调试串口输出中，先打印文本说明。 */
        s_cmd_announced = 1u;
        BSP_UartSendString("\r\nGRAY push-mode cmd A5 03 15 02\r\n");
    }

    s_cmd_pending = 0u;                             // 首次发送的待处理状态清零
    s_last_cmd_tick = now;                          // 更新重试节拍基准
    gray_send_push_cmd();
}

/**
  * 函    数：查询灰度模块最近是否发来有效帧
  * 参    数：无
  * 返 回 值：1 为在线，0 为离线
  */
uint8_t Gray_IsOnline(void)
{
    return s_online;
}

/**
  * 函    数：获取最后一次有效灰度帧的阈值位图
  * 参    数：无
  * 返 回 值：八路位图，1 表示黑，离线时仍返回缓存
  */
uint8_t Gray_GetBits(void)
{
    return s_bits;
}

/**
  * 函    数：读取指定探头的最近原始 ADC 计数
  * 参    数：index：探头索引，0～GRAY_CHANNELS-1
  * 返 回 值：12 位原始值，索引越界时返回 0
  */
uint16_t Gray_GetRaw(uint8_t index)
{
    if (index >= GRAY_CHANNELS)
    {
        return 0u;
    }
    return s_raw[index];
}

/**
  * 函    数：获取本次启动以来接受的有效灰度帧数
  * 参    数：无
  * 返 回 值：饱和计数，最大为 65535
  */
uint16_t Gray_GetFrames(void)
{
    return s_frames;
}

/**
  * 函    数：获取本次启动以来因高字节非法拒收的帧数
  * 参    数：无
  * 返 回 值：饱和计数，最大为 255
  */
uint8_t Gray_GetErrors(void)
{
    return s_errors;
}

/* USART1 接收中断 */
/**
  * 函    数：将收到的一个字节加入灰度环形缓冲区
  * 参    数：无
  * 返 回 值：无，缓冲区已满时丢弃当前字节
  */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t b = (uint8_t)USART_ReceiveData(USART1);  // 读取 USART1 数据寄存器中的字节
        uint8_t next = (uint8_t)((s_rx_head + 1u) & GRAY_RING_MASK);  // 计算推进后的位置，掩码实现回绕

        if (next != s_rx_tail)                      // 下一个位置等于读指针时，队列已满
        {
            s_rx[s_rx_head] = b;                    // 先写数据，再发布新的写入位置
            s_rx_head = next;
        }
    }
}
