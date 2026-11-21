#include "stm32f10x.h"
#include "debug.h"
#include "app_config.h"
#include "bsp.h"
#include "control.h"
#include "path.h"
#include "imu.h"
#include "gray_uart.h"

/**
  * 函    数：开机输出固件版本、复位原因和灰度串口协议摘要
  * 参    数：无
  * 返 回 值：无
  */
void Debug_Init(void)
{
    uint8_t rst = BSP_ResetCause();                 // 读取 BSP 在启动时保存的复位原因位图

    BSP_UartSendString("\r\nBOOT 16th team STM32F103 ");
    BSP_UartSendString(FW_TAG);                     // 输出固件版本标识
    BSP_UartSendString("\r\nRST flags=");
    BSP_UartSendInt((int32_t)rst);
    /* 根据复位原因位图附加上电、引脚、软件、看门狗和低功耗标志。 */
    if (rst & 0x01u) { BSP_UartSendString(" POR"); }
    if (rst & 0x02u) { BSP_UartSendString(" PIN"); }
    if (rst & 0x04u) { BSP_UartSendString(" SW"); }
    if (rst & 0x08u) { BSP_UartSendString(" IWDG"); }
    if (rst & 0x10u) { BSP_UartSendString(" WWDG"); }
    if (rst & 0x20u) { BSP_UartSendString(" LPWR"); }
    BSP_UartSendString("\r\n");
    if (rst & 0x08u)
    {
        /* IWDG 表示独立看门狗曾触发复位，日志保留此次事件供排查。 */
        BSP_UartSendString("WARNING: previous run was reset by the watchdog\r\n");
    }
    BSP_UartSendString("GRAY uart 115200, push mode 0x02 (5B + 8 x 12 bit)\r\n");
}

/**
  * 函    数：周期性输出运行遥测，并发送到达的弧段统计
  * 参    数：无
  * 返 回 值：无
  */
void Debug_Update(void)
{
    static uint32_t last_ms = 0;                    // 上一次发送遥测的 5 ms 节拍数
    uint32_t now = BSP_GetTick5ms();
    uint16_t raw_adc;
    uint32_t adc_mv;
    uint32_t bat_mv;
    uint8_t i;

    if ((now - last_ms) < 20)                       // 至少间隔 20 个节拍，约 100 ms
    {
        return;
    }
    last_ms = now;

    raw_adc = BSP_AdcReadBattery();
    adc_mv = (uint32_t)raw_adc * 3300UL / 4095UL;   // 把 12 位 ADC 原码换算为 PB1 上的毫伏值
    bat_mv = adc_mv * VBAT_ADC_MULT;                // 用标定倍率估算电池端电压

    BSP_UartSendString("t=");
    BSP_UartSendInt((int32_t)(now * 5));            // 5 ms 节拍换算为毫秒
    BSP_UartSendString(" run=");
    BSP_UartSendInt(Path_IsRunning());
    BSP_UartSendString(" mode=");
    BSP_UartSendInt(Path_GetMode());
    BSP_UartSendString(" seg=");
    BSP_UartSendInt(Path_GetSegment());
    BSP_UartSendString(" d=");
    BSP_UartSendInt((int32_t)Control_GetDistance());  // 累计里程，单位 mm
    BSP_UartSendString(" yaw=");
    BSP_UartSendInt((int32_t)Imu_GetYaw());         // 相对航向，单位 °
    BSP_UartSendString(" gz=");
    BSP_UartSendInt((int32_t)(Imu_GetGyroZ() * 10.0f));  // gz 以 0.1 °/s 为一个计数单位
    BSP_UartSendString(" g=");
    BSP_UartSendInt(Control_GetGray());             // g 为本周期使用的灰度位图
    BSP_UartSendString(" gl=");
    BSP_UartSendInt(Gray_IsOnline());               // gl 为灰度链路在线标志
    BSP_UartSendString(" gf=");
    BSP_UartSendInt((int32_t)Gray_GetFrames());     // gf 为已接受灰度帧数，达到上限后饱和
    BSP_UartSendString(" gx=");
    BSP_UartSendInt((int32_t)Gray_GetErrors());     // gx 为高字节校验拒收的帧数
    BSP_UartSendString(" e=");
    BSP_UartSendInt((int32_t)(Path_GetLineError() * 100.0f));  // e 为归一化灰度误差放大 100 倍后的整数
    BSP_UartSendString(" yr=");
    BSP_UartSendInt((int32_t)Path_GetYawRef());     // yr 为当前航向参考，单位 °
    BSP_UartSendString(" gc=");
    BSP_UartSendInt((int32_t)Path_GetGrayCorr());   // gc 为灰度航向修正，单位 °
    BSP_UartSendString(" lt=");
    BSP_UartSendInt((int32_t)Path_GetLineTheta());  // lt 为循迹航向偏置，单位 °
    BSP_UartSendString(" lr=");
    BSP_UartSendInt((int32_t)Path_GetLineRate());   // lr 为灰度给出的目标角速度修正，单位 °/s
    BSP_UartSendString(" lm=");
    BSP_UartSendInt(Path_GetArcMode());             // lm 为当前是否选择灰度循迹模式
    BSP_UartSendString(" Rr=");
    BSP_UartSendInt((int32_t)Path_GetArcRadius());  // Rr 为估算弧线半径，单位 mm
    BSP_UartSendString(" L=");
    BSP_UartSendInt((int32_t)Control_GetLeftSpeed());
    BSP_UartSendString(" R=");
    BSP_UartSendInt((int32_t)Control_GetRightSpeed());
    BSP_UartSendString(" bat=");
    BSP_UartSendInt((int32_t)(bat_mv / 1000UL));
    BSP_UartSendString(".");
    BSP_UartSendInt((int32_t)((bat_mv % 1000UL) / 100UL));
    BSP_UartSendString("V");

    /* 输出八路原始 ADC 计数。通常白底为几百，黑线为几千，可用来检查单个通道是否异常。 */
    BSP_UartSendString(" a=");
    for (i = 0; i < GRAY_CHANNELS; i++)
    {
        if (i > 0u)
        {
            BSP_UartSendChar(',');
        }
        BSP_UartSendInt((int32_t)Gray_GetRaw(i));
    }

    BSP_UartSendString("\r\n");

    /* 单独输出弧段统计，便于赛后检查每段距离、转角和丢线情况。 */
    {
        ArcReport rep;
        if (Path_TakeArcReport(&rep))               // 取出待发送的一段弧线统计，同时清除待取标志
        {
            BSP_UartSendString("ARC d=");
            BSP_UartSendInt(rep.dist_mm);
            BSP_UartSendString(" yaw=");
            BSP_UartSendInt(rep.yaw_deg);
            BSP_UartSendString(" Rr=");
            BSP_UartSendInt(rep.radius_mm);
            BSP_UartSendString(" lm=");
            BSP_UartSendInt(rep.mode);
            BSP_UartSendString(" why=");
            BSP_UartSendInt(rep.why);               // 输出弧段结束原因编码
            BSP_UartSendString(" e=");
            BSP_UartSendInt(rep.e_min);
            BSP_UartSendString("..");
            BSP_UartSendInt(rep.e_max);
            BSP_UartSendString(" gc=");
            BSP_UartSendInt(rep.gc_min);
            BSP_UartSendString("..");
            BSP_UartSendInt(rep.gc_max);
            BSP_UartSendString(" g=");
            BSP_UartSendInt(rep.g_min);
            BSP_UartSendString("..");
            BSP_UartSendInt(rep.g_max);
            BSP_UartSendString(" lost=");
            BSP_UartSendInt(rep.lost);              // 输出弧段中累计无方向信息的控制周期数
            BSP_UartSendString("\r\n");
        }
    }
}
