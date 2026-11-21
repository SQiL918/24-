#include "stm32f10x.h"
#include "bsp.h"
#include "imu.h"
#include "control.h"
#include "path.h"
#include "ui.h"
#include "debug.h"
#include "gray_uart.h"

/**
  * 函    数：初始化小车固件并运行前台任务
  * 参    数：无
  * 返 回 值：正常运行时常驻主循环，无返回
  */
int main(void)
{
    BSP_Init();                                     // 初始化 GPIO、定时器、串口及 ADC 等板级外设

    /* 板级初始化后开启独立看门狗，覆盖后续上电校准及运行阶段。
     * IMU 初始化和校准中的正常长等待会主动喂狗，运行后由主循环喂狗。
     */
    BSP_WdgInit(IWDG_TIMEOUT_MS);

    BSP_Stage(BSP_STAGE_INIT);                      // 标记初始化阶段，供异常日志定位

    Control_Init();                                 // 清零轮速和里程，保持电机短路制动
    Path_Init();                                    // 将路径状态机设为待机
    Gray_Init();                                    // 启用灰度串口接收中断

    BSP_UartSendString("\r\nIMU calibration, keep the car still...\r\n");
    Imu_Init();                                     // 配置 MPU6050 并检查唤醒结果
    Imu_Calibrate();                                // 静止采集陀螺仪零偏
    BSP_UartSendString("IMU calibration done\r\n");

    UI_Init();                                      // 初始化 OLED 和声光提示
    Debug_Init();                                   // 输出固件版本及复位原因

    BSP_TickStart();                                // 启动 TIM1，使 5 ms 控制循环开始运行

    while (1)
    {
        BSP_Stage(BSP_STAGE_UI);
        UI_Update();                                // 按节拍扫描按键并更新 OLED
        BSP_Stage(BSP_STAGE_DEBUG);
        Debug_Update();                             // 按节拍发送遥测和弧段汇总
        BSP_Stage(BSP_STAGE_GRAY);
        Gray_Poll();                                // 维护灰度链路，按需请求模块恢复推送
        BSP_WdgFeed();                              // 前台任务完成一轮后重装看门狗
    }
}
