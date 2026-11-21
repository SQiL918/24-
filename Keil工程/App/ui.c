#include "stm32f10x.h"
#include "ui.h"
#include "app_config.h"
#include "bsp.h"
#include "control.h"
#include "path.h"
#include "imu.h"
#include "OLED.h"
#include "OLED_CN.h"

static volatile uint16_t g_speed_set = 600;         // 基础速度默认 600 mm/s，供控制中断读取
static uint8_t g_mode = 1;                          // 待启动的任务模式，默认 Q1

static uint32_t g_last_ui_ms = 0;                   // 最近一次屏幕刷新时的 5 ms 节拍数
static uint32_t g_last_key_ms = 0;                  // 最近一次按键扫描时的 5 ms 节拍数

static uint8_t g_db_start_state = 0;
static uint8_t g_db_speed_state = 0;
static uint8_t g_db_mode_state = 0;
static uint8_t g_db_start_cnt = 0;
static uint8_t g_db_speed_cnt = 0;
static uint8_t g_db_mode_cnt = 0;

static const uint16_t g_pat_start[] = { 20, 0 };    // 起跑亮灯并鸣叫 100 ms，0 为结束标记
static const uint16_t g_pat_point[] = { 10, 10, 10, 0 };  // 到点连续两次亮灯并鸣叫，每次 50 ms，中间停 50 ms
static const uint16_t g_pat_stop[]  = { 100, 0 };   // 停车亮灯并鸣叫 500 ms
static const uint16_t *g_pat = 0;
static uint8_t  g_pat_index = 0;
static uint16_t g_pat_timer = 0;

/**
  * 函    数：对单个按键消抖并检测一次稳定按下事件
  * 参    数：raw：当前原始按键状态，1 为按下
  *           state：保存最近原始电平的指针
  *           count：保存连续稳定扫描次数的指针
  * 返 回 值：1 表示本次确认按下，0 表示无新的按下事件
  */
static uint8_t key_debounce(uint8_t raw, uint8_t *state, uint8_t *count)
{
    if (raw != *state)
    {
        *state = raw;
        *count = 0;                                 // 电平变化，重新累计稳定扫描次数
    }
    else if (*count < 3)
    {
        (*count)++;
    }

    if ((*count == 3) && raw)                       // 原始状态连续稳定 3 次且为按下时产生事件
    {
        *count = 4;                                 // 置为已触发，长按时不重复产生事件
        return 1;
    }

    return 0;
}

/* 蜂鸣器、外接 LED（PB2）和板载 LED（PC13）按同一声光序列工作。
 * 各引脚的物理电平极性由板级接口处理。
 */
/**
  * 函    数：同步控制蜂鸣器、外接 LED 和板载 LED
  * 参    数：on：0 关闭，非 0 开启
  * 返 回 值：无
  */
static void ui_indicator(uint8_t on)
{
    BSP_Buzzer(on);
    BSP_Led(on);
    BSP_LedOnboard(on);
}

/**
  * 函    数：初始化 OLED 静态界面并关闭声光输出
  * 参    数：无
  * 返 回 值：无
  */
void UI_Init(void)
{
    OLED_Init();
    OLED_Clear();

    /* 将四个 16×16 点阵居中显示在屏幕前两页。 */
    OLED_ShowChinese16(32, 0, CN_DI);
    OLED_ShowChinese16(48, 0, CN_SHI);
    OLED_ShowChinese16(64, 0, CN_LIU);
    OLED_ShowChinese16(80, 0, CN_DUI);

    OLED_ShowString(2, 1, "L:     R:");             // 第二行的 L、R 标签对应左右轮速
    OLED_ShowString(3, 1, "V:     M:Q");            // 第三行显示设定速度和模式编号
    OLED_ShowString(4, 1, "Y:     B:  . V");        // 第四行显示相对航向和电池电压

    ui_indicator(0);
}

/**
  * 函    数：根据路径事件开始对应的声光序列
  * 参    数：event：起跑、到点或停车事件
  * 返 回 值：无
  */
void UI_Announce(PathEvent event)
{
    if (event == PATH_EVENT_START)
    {
        g_pat = g_pat_start;
    }
    else if (event == PATH_EVENT_POINT)
    {
        g_pat = g_pat_point;
    }
    else if (event == PATH_EVENT_STOP)
    {
        g_pat = g_pat_stop;
    }
    else
    {
        return;
    }

    g_pat_index = 0;                                // 从序列的第一个阶段开始
    g_pat_timer = g_pat[0];                         // 取出首阶段剩余的 5 ms 节拍数

    ui_indicator(1);                                // 序列从亮灯和鸣叫开始
}

/**
  * 函    数：推进声光序列的 5 ms 定时
  * 参    数：无
  * 返 回 值：无
  */
void UI_Tick5ms(void)
{
    uint16_t d;

    if (g_pat == 0)                                 // 空指针表示当前没有活动的提示序列
    {
        return;
    }

    if (g_pat_timer > 0)
    {
        g_pat_timer--;                              // 每个定时中断将本阶段时长减去一个节拍
    }

    if (g_pat_timer > 0)
    {
        return;
    }

    g_pat_index++;
    d = g_pat[g_pat_index];

    if (d == 0)                                     // 零时长表示整段声光提示已结束
    {
        g_pat = 0;
        ui_indicator(0);
        return;
    }

    g_pat_timer = d;

    if ((g_pat_index & 1u) == 0)                    // 偶数阶段输出，奇数阶段间隔
    {
        ui_indicator(1);
    }
    else
    {
        ui_indicator(0);
    }
}

/**
  * 函    数：获取当前选择的基础速度
  * 参    数：无
  * 返 回 值：基础速度设定，单位 mm/s
  */
uint16_t UI_GetSpeedSet(void)
{
    return g_speed_set;
}

/**
  * 函    数：更新 OLED 的轮速、模式、航向和电压
  * 参    数：无
  * 返 回 值：无
  */
static void ui_show_numbers(void)
{
    int32_t left = (int32_t)Control_GetLeftSpeed();
    int32_t right = (int32_t)Control_GetRightSpeed();
    int32_t yaw = (int32_t)Imu_GetYaw();
    uint16_t raw_adc = BSP_AdcReadBattery();
    uint32_t adc_mv = (uint32_t)raw_adc * 3300UL / 4095UL;  // 以 3.3 V 为基准，将 ADC 原码换算为引脚电压
    uint32_t bat_mv = adc_mv * VBAT_ADC_MULT;       // 应用本车标定倍率得到电池电压，单位 mV

    /* OLED 轮速字段固定为 4 位无符号数，先限制显示范围。 */
    if (left < 0)   { left = 0; }
    if (left > 9999) { left = 9999; }
    if (right < 0)  { right = 0; }
    if (right > 9999) { right = 9999; }

    /* 将累计航向折返到 ±180°，仅影响显示。 */
    while (yaw > 180)  { yaw -= 360; }
    while (yaw < -180) { yaw += 360; }

    OLED_ShowNum(2, 3, (uint32_t)left, 4);
    OLED_ShowNum(2, 10, (uint32_t)right, 4);

    OLED_ShowNum(3, 3, (uint32_t)g_speed_set, 4);
    OLED_ShowNum(3, 11, (uint32_t)g_mode, 1);

    if (Imu_IsReady())
    {
        OLED_ShowSignedNum(4, 3, yaw, 4);
    }
    else
    {
        OLED_ShowString(4, 3, "Err  ");             // IMU 未就绪时显示错误标记
    }

    OLED_ShowNum(4, 10, bat_mv / 1000UL, 2);        // 写入电压整数部分
    OLED_ShowNum(4, 13, (bat_mv % 1000UL) / 100UL, 1);  // 只显示小数点后第一位
}

/**
  * 函    数：在主循环扫描按键并按周期刷新 OLED
  * 参    数：无
  * 返 回 值：无
  */
void UI_Update(void)
{
    uint32_t now = BSP_GetTick5ms();

    /* 按键扫描间隔为 4 个节拍，即 20 ms。 */
    if ((now - g_last_key_ms) >= 4)
    {
        uint8_t pressed;

        g_last_key_ms = now;

        pressed = key_debounce(BSP_KeyStart(), &g_db_start_state, &g_db_start_cnt);  // 读取启停按键的稳定按下事件
        if (pressed)
        {
            if (Path_IsRunning())
            {
                Path_Stop();                        // 取消正在执行的路径
                BSP_MotorBrake();                   // 手动停车时直接输出电机制动
            }
            else
            {
                Control_Reset();                    // 起跑前重置里程、轮速及 PI 状态
                Imu_ResetYaw();                     // 将本次起跑方向设为航向零点
                Path_Start(g_mode);                 // 用当前选定模式启动任务
            }
        }

        pressed = key_debounce(BSP_KeySpeed(), &g_db_speed_state, &g_db_speed_cnt);
        if (pressed)
        {
            if (g_speed_set >= SPEED_SET_MAX) { g_speed_set = SPEED_SET_MIN; }  // 速度达到最高档后循环到最低档
            else                              { g_speed_set += SPEED_SET_STEP; }  // 每次按下提高一个速度档位
        }

        pressed = key_debounce(BSP_KeyMode(), &g_db_mode_state, &g_db_mode_cnt);
        if (pressed && !Path_IsRunning())           // 只允许在未运行时切换待启模式
        {
            g_mode++;
            if (g_mode > 4) { g_mode = 1; }         // 模式编号按 Q1～Q4 循环
            if (g_mode != Path_GetMode())
            {
                /* 所选模式在下次启动时由 Path_Start 采用 */
            }
        }
    }

    /* OLED 按 200 ms 周期刷新，约 5 Hz。 */
    if ((now - g_last_ui_ms) >= (UI_REFRESH_MS / 5))
    {
        g_last_ui_ms = now;
        ui_show_numbers();
    }
}
