#ifndef __GRAY_UART_H
#define __GRAY_UART_H

#include "stm32f10x.h"

/* CH32V006 八路灰度串口接口。
 * PA10 接模块 TX，PA9 同时发送到模块 RX 和调试 CH340 RX。
 * 启动后发送 A5 03 15 02。模块通常每 10 ms 推送 17 字节，链路断开后会重发配置。
 * 调试文本使用 ASCII，字节范围低于 0x80。
 */

#define GRAY_CHANNELS           8u

void     Gray_Init(void);                           // 初始化接收中断，并准备发送推送配置
void     Gray_Task(void);                           // 从 5 ms 控制周期调用，排空并解析接收缓冲
void     Gray_Poll(void);                           // 从主循环调用，维护链路并按需重发配置

uint8_t  Gray_IsOnline(void);                       // 近期有有效帧时返回 1
uint8_t  Gray_GetBits(void);                        // 默认 bit0 对应第 1 路，位值 1 表示黑，离线时保留上一帧
uint16_t Gray_GetRaw(uint8_t index);                // 读取某一路 12 位原始值，index 从 0 开始
uint16_t Gray_GetFrames(void);                      // 有效帧计数，达到 65535 后饱和
uint8_t  Gray_GetErrors(void);                      // 值域校验拒收的帧数，达到 255 后饱和

#endif
