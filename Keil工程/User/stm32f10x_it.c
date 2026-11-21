/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* 异常处理接口和故障诊断所需的板级驱动。 */
#include "stm32f10x_it.h"
#include "bsp.h"

/* 故障处理先输出异常类型、阶段和内核故障寄存器，再按不同次数闪烁板载 LED。
 * 看门狗已启动时，停止喂狗会在超时后复位，启动日志会标记 IWDG。
 */
/**
  * 函    数：将异常类型、当前执行阶段和故障寄存器打印到串口
  * 参    数：name：表示异常类型的以空字符结尾字符串
  * 返 回 值：无
  */
static void Fault_Report(const char *name)
{
    BSP_UartSendString("\r\nFAULT ");
    BSP_UartSendString(name);                       // 输出 HARD、MEM、BUS 或 USAGE 标记
    BSP_UartSendString(" stage=");
    BSP_UartSendInt((int32_t)BSP_GetStage());       // 读取应用最后一次记录的执行阶段
    BSP_UartSendString(" CFSR=");
    BSP_UartSendInt((int32_t)(SCB->CFSR & 0x7FFFFFFFu));  // 输出可配置故障状态寄存器，最高位清零
    BSP_UartSendString(" HFSR=");
    BSP_UartSendInt((int32_t)(SCB->HFSR & 0x7FFFFFFFu));  // 输出硬故障状态寄存器，最高位清零
    BSP_UartSendString(" MMFAR=");
    BSP_UartSendInt((int32_t)(SCB->MMFAR & 0x7FFFFFFFu));  // 输出内存管理故障地址寄存器，最高位清零
    BSP_UartSendString(" BFAR=");
    BSP_UartSendInt((int32_t)(SCB->BFAR & 0x7FFFFFFFu));  // 输出总线故障地址寄存器，最高位清零
    BSP_UartSendString("\r\n");
}

/**
  * 函    数：循环闪烁板载 LED 显示故障类型
  * 参    数：blinks：每组闪烁次数
  * 返 回 值：不返回，等待看门狗或外部复位
  */
static void Fault_Hold(uint8_t blinks)
{
    volatile uint32_t d;                            // 使用 volatile 保留忙等待循环

    for (;;)
    {
        uint8_t i;

        for (i = 0; i < blinks; i++)
        {
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);  // PC13 拉低点亮板载 LED
            for (d = 0; d < 1500000u; d++) { }
            GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);  // PC13 拉高熄灭 LED
            for (d = 0; d < 1500000u; d++) { }
        }
        for (d = 0; d < 3000000u; d++) { }          // 每组闪烁之间等待较长时段
    }
}

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */


/* Cortex-M3 内核异常和系统中断入口。 */

/**
  * 函    数：响应不可屏蔽中断，当前处理函数为空
  * 参    数：无
  * 返 回 值：无
  */
void NMI_Handler(void)
{
}

/**
  * 函    数：报告硬故障，并以每组三次闪灯指示
  * 参    数：无
  * 返 回 值：不返回
  */
void HardFault_Handler(void)
{
  Fault_Report("HARD");
  Fault_Hold(3);                                    // 三次闪灯代表硬故障
}

/**
  * 函    数：报告内存管理故障，并以每组四次闪灯指示
  * 参    数：无
  * 返 回 值：不返回
  */
void MemManage_Handler(void)
{
  Fault_Report("MEM");
  Fault_Hold(4);                                    // 四次闪灯代表内存管理故障
}

/**
  * 函    数：报告总线故障，并以每组五次闪灯指示
  * 参    数：无
  * 返 回 值：不返回
  */
void BusFault_Handler(void)
{
  Fault_Report("BUS");
  Fault_Hold(5);                                    // 五次闪灯代表总线故障
}

/**
  * 函    数：报告用法故障，并以每组六次闪灯指示
  * 参    数：无
  * 返 回 值：不返回
  */
void UsageFault_Handler(void)
{
  Fault_Report("USAGE");
  Fault_Hold(6);                                    // 六次闪灯代表用法故障
}

/**
  * 函    数：响应 SVC 系统调用异常，当前处理函数为空
  * 参    数：无
  * 返 回 值：无
  */
void SVC_Handler(void)
{
}

/**
  * 函    数：响应调试监视异常，当前处理函数为空
  * 参    数：无
  * 返 回 值：无
  */
void DebugMon_Handler(void)
{
}

/**
  * 函    数：响应 PendSV 异常，当前处理函数为空
  * 参    数：无
  * 返 回 值：无
  */
void PendSV_Handler(void)
{
}

/**
  * 函    数：响应 SysTick 中断，当前控制时基由 TIM1 提供
  * 参    数：无
  * 返 回 值：无
  */
void SysTick_Handler(void)
{
}

/* 外设中断入口名称由启动文件向量表决定。
 * 实际的 TIM1 更新中断在 control.c，USART1 接收中断在 gray_uart.c。
 */

/* 外设中断模板占位。新增外设时应使用启动文件定义的真实 IRQHandler 名称。 */

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
