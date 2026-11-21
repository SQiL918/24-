#ifndef __BSP_H
#define __BSP_H

#include "stm32f10x.h"
#include "app_config.h"

/* 板级外设初始化与 5 ms 时基。 */
void BSP_Init(void);
void BSP_TickStart(void);
void BSP_Tick5ms(void);
uint32_t BSP_GetTick5ms(void);

/* 稳定性与故障诊断接口 */
/* 独立看门狗。前台长期未喂狗时触发复位。 */
void     BSP_WdgInit(uint16_t timeout_ms);
void     BSP_WdgFeed(void);

/* 本次启动的复位来源，在清除 RCC 标志前采样。
 * bit0 上电，bit1 复位引脚，bit2 软件复位，bit3 IWDG，bit4 WWDG，bit5 低功耗复位。
 */
uint8_t  BSP_ResetCause(void);

/* 故障处理会打印最近写入的执行阶段，供串口定位卡顿位置。 */
void     BSP_Stage(uint8_t stage);
uint8_t  BSP_GetStage(void);

#define BSP_STAGE_BOOT      1u
#define BSP_STAGE_INIT      2u
#define BSP_STAGE_UI        3u
#define BSP_STAGE_DEBUG     4u
#define BSP_STAGE_GRAY      5u
/* 以下阶段在 200 Hz 控制中断中写入。 */
#define BSP_STAGE_CONTROL   6u                      // 编码器采样与灰度数据解析
#define BSP_STAGE_IMU       7u                      // MPU6050 软件 I²C 读取与航向更新
#define BSP_STAGE_PATH      8u                      // 路径状态机
#define BSP_STAGE_OUTPUT    9u                      // 速度环、电机 PWM 与制动

/* 指示灯与蜂鸣器，调用方使用逻辑开关状态。 */
void BSP_Led(uint8_t on);
void BSP_Buzzer(uint8_t on);
void BSP_LedOnboard(uint8_t on);

/* 按键为低电平有效，读取接口返回 1 表示按下。 */
uint8_t BSP_KeyStart(void);
uint8_t BSP_KeySpeed(void);
uint8_t BSP_KeyMode(void);

/* 八路灰度由串口采集，接口见 gray_uart.h。常规位序 bit0 对应通道 1，位值为 1 表示黑。 */

/* 电机 PWM 使用带符号比较值，范围为 -PWM_MAX～+PWM_MAX。 */
void BSP_MotorSet(int16_t left, int16_t right);
void BSP_MotorBrake(void);
void BSP_MotorCoast(void);

/* 编码器返回自上次读取以来的有符号计数差。 */
void BSP_EncoderRead(int16_t *left, int16_t *right);
void BSP_EncoderReset(void);

/* 电池 ADC 返回 12 位原码，范围 0～4095。 */
uint16_t BSP_AdcReadBattery(void);

/* 调试串口为 USART1，115200 8N1，与灰度模块共用发送引脚。 */
void BSP_UartSendChar(char c);
void BSP_UartSendString(const char *s);
void BSP_UartSendInt(int32_t v);

#endif
