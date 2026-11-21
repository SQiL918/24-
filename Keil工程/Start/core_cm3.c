/**************************************************************************//**
 * @file     core_cm3.c
 * @brief    CMSIS Cortex-M3 Core Peripheral Access Layer Source File
 * @version  V1.30
 * @date     30. October 2009
 *
 * @note
 * Copyright (C) 2009 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M 
 * processor based microcontrollers.  This file can be freely distributed 
 * within development tools that are supporting such ARM based processors. 
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/

#include <stdint.h>

/* define compiler specific symbols */
#if defined ( __CC_ARM   )
  #define __ASM            __asm                                      /*!< asm keyword for ARM Compiler          */
  #define __INLINE         __inline                                   /*!< inline keyword for ARM Compiler       */

#elif defined ( __ICCARM__ )
  #define __ASM           __asm                                       /*!< asm keyword for IAR Compiler          */
  #define __INLINE        inline                                      /*!< inline keyword for IAR Compiler. Only avaiable in High optimization mode! */

#elif defined   (  __GNUC__  )
  #define __ASM            __asm                                      /*!< asm keyword for GNU Compiler          */
  #define __INLINE         inline                                     /*!< inline keyword for GNU Compiler       */

#elif defined   (  __TASKING__  )
  #define __ASM            __asm                                      /*!< asm keyword for TASKING Compiler      */
  #define __INLINE         inline                                     /*!< inline keyword for TASKING Compiler   */

#endif


/* ###################  Compiler specific Intrinsics  ########################### */

#if defined ( __CC_ARM   ) /*------------------RealView Compiler -----------------*/
/* ARM armcc specific functions */

/**
 * @brief  Return the Process Stack Pointer
 *
 * @return ProcessStackPointer
 *
 * Return the actual process stack pointer
 */
/**
  * 函    数：读取线程可使用的进程栈指针 PSP
  * 参    数：无
  * 返 回 值：PSP 当前值
  */
__ASM uint32_t __get_PSP(void)
{
  mrs r0, psp
  bx lr
}

/**
 * @brief  Set the Process Stack Pointer
 *
 * @param  topOfProcStack  Process Stack Pointer
 *
 * Assign the value ProcessStackPointer to the MSP 
 * (process stack pointer) Cortex processor register
 */
/**
  * 函    数：写入进程栈指针 PSP
  * 参    数：topOfProcStack：要写入 PSP 的进程栈指针
  * 返 回 值：无
  */
__ASM void __set_PSP(uint32_t topOfProcStack)
{
  msr psp, r0
  bx lr
}

/**
 * @brief  Return the Main Stack Pointer
 *
 * @return Main Stack Pointer
 *
 * Return the current value of the MSP (main stack pointer)
 * Cortex processor register
 */
/**
  * 函    数：读取主栈指针 MSP
  * 参    数：无
  * 返 回 值：MSP 当前值
  */
__ASM uint32_t __get_MSP(void)
{
  mrs r0, msp
  bx lr
}

/**
 * @brief  Set the Main Stack Pointer
 *
 * @param  topOfMainStack  Main Stack Pointer
 *
 * Assign the value mainStackPointer to the MSP 
 * (main stack pointer) Cortex processor register
 */
/**
  * 函    数：写入主栈指针 MSP
  * 参    数：mainStackPointer：要写入 MSP 的主栈指针
  * 返 回 值：无
  */
__ASM void __set_MSP(uint32_t mainStackPointer)
{
  msr msp, r0
  bx lr
}

/**
 * @brief  Reverse byte order in unsigned short value
 *
 * @param   value  value to reverse
 * @return         reversed value
 *
 * Reverse byte order in unsigned short value
 */
/**
  * 函    数：在两个半字内部各自交换高低字节
  * 参    数：value：要在半字内部交换字节的值
  * 返 回 值：以 32 位类型返回字节交换结果
  */
__ASM uint32_t __REV16(uint16_t value)
{
  rev16 r0, r0
  bx lr
}

/**
 * @brief  Reverse byte order in signed short value with sign extension to integer
 *
 * @param   value  value to reverse
 * @return         reversed value
 *
 * Reverse byte order in signed short value with sign extension to integer
 */
/**
  * 函    数：交换低 16 位的两个字节并做符号扩展
  * 参    数：value：要交换字节并扩展的有符号 16 位值
  * 返 回 值：符号扩展后的 32 位结果
  */
__ASM int32_t __REVSH(int16_t value)
{
  revsh r0, r0
  bx lr
}


#if (__ARMCC_VERSION < 400000)

/**
 * @brief  Remove the exclusive lock created by ldrex
 *
 * Removes the exclusive lock which is created by ldrex.
 */
/**
  * 函    数：清除内核本地独占访问标记
  * 参    数：无
  * 返 回 值：无
  */
__ASM void __CLREX(void)
{
  clrex
}

/**
 * @brief  Return the Base Priority value
 *
 * @return BasePriority
 *
 * Return the content of the base priority register
 */
/**
  * 函    数：读取可屏蔽异常的优先级屏蔽阈值
  * 参    数：无
  * 返 回 值：BASEPRI 当前值，0 表示不设阈值
  */
__ASM uint32_t  __get_BASEPRI(void)
{
  mrs r0, basepri
  bx lr
}

/**
 * @brief  Set the Base Priority value
 *
 * @param  basePri  BasePriority
 *
 * Set the base priority register
 */
/**
  * 函    数：设置可屏蔽异常的优先级屏蔽阈值
  * 参    数：basePri：写入 BASEPRI 的优先级门限，0 表示关闭该屏蔽
  * 返 回 值：无
  */
__ASM void __set_BASEPRI(uint32_t basePri)
{
  msr basepri, r0
  bx lr
}

/**
 * @brief  Return the Priority Mask value
 *
 * @return PriMask
 *
 * Return state of the priority mask bit from the priority mask register
 */
/**
  * 函    数：读取可配置优先级异常的全局屏蔽位
  * 参    数：无
  * 返 回 值：PRIMASK 当前值
  */
__ASM uint32_t __get_PRIMASK(void)
{
  mrs r0, primask
  bx lr
}

/**
 * @brief  Set the Priority Mask value
 *
 * @param  priMask  PriMask
 *
 * Set the priority mask bit in the priority mask register
 */
/**
  * 函    数：写入可配置优先级异常的全局屏蔽位
  * 参    数：priMask：0 解除 PRIMASK 屏蔽，1 使能屏蔽
  * 返 回 值：无
  */
__ASM void __set_PRIMASK(uint32_t priMask)
{
  msr primask, r0
  bx lr
}

/**
 * @brief  Return the Fault Mask value
 *
 * @return FaultMask
 *
 * Return the content of the fault mask register
 */
/**
  * 函    数：读取 FAULTMASK 故障屏蔽位
  * 参    数：无
  * 返 回 值：FAULTMASK 当前值
  */
__ASM uint32_t  __get_FAULTMASK(void)
{
  mrs r0, faultmask
  bx lr
}

/**
 * @brief  Set the Fault Mask value
 *
 * @param  faultMask  faultMask value
 *
 * Set the fault mask register
 */
/**
  * 函    数：设置 FAULTMASK 故障屏蔽位
  * 参    数：faultMask：0 解除 FAULTMASK 屏蔽，1 使能屏蔽
  * 返 回 值：无
  */
__ASM void __set_FAULTMASK(uint32_t faultMask)
{
  msr faultmask, r0
  bx lr
}

/**
 * @brief  Return the Control Register value
 * 
 * @return Control value
 *
 * Return the content of the control register
 */
/**
  * 函    数：读取线程模式的栈选择和特权控制寄存器
  * 参    数：无
  * 返 回 值：CONTROL 当前值
  */
__ASM uint32_t __get_CONTROL(void)
{
  mrs r0, control
  bx lr
}

/**
 * @brief  Set the Control Register value
 *
 * @param  control  Control value
 *
 * Set the control register
 */
/**
  * 函    数：设置线程模式的栈选择和特权控制寄存器
  * 参    数：control：要写入 CONTROL 的值，包括线程模式的栈选择和特权位
  * 返 回 值：无
  */
__ASM void __set_CONTROL(uint32_t control)
{
  msr control, r0
  bx lr
}

#endif /* __ARMCC_VERSION  */ 



#elif (defined (__ICCARM__)) /*------------------ ICC Compiler -------------------*/
/* IAR iccarm specific functions */
#pragma diag_suppress=Pe940

/**
 * @brief  Return the Process Stack Pointer
 *
 * @return ProcessStackPointer
 *
 * Return the actual process stack pointer
 */
/**
  * 函    数：读取线程可使用的进程栈指针 PSP
  * 参    数：无
  * 返 回 值：PSP 当前值
  */
uint32_t __get_PSP(void)
{
  __ASM("mrs r0, psp");
  __ASM("bx lr");
}

/**
 * @brief  Set the Process Stack Pointer
 *
 * @param  topOfProcStack  Process Stack Pointer
 *
 * Assign the value ProcessStackPointer to the MSP 
 * (process stack pointer) Cortex processor register
 */
/**
  * 函    数：写入进程栈指针 PSP
  * 参    数：topOfProcStack：要写入 PSP 的进程栈指针
  * 返 回 值：无
  */
void __set_PSP(uint32_t topOfProcStack)
{
  __ASM("msr psp, r0");
  __ASM("bx lr");
}

/**
 * @brief  Return the Main Stack Pointer
 *
 * @return Main Stack Pointer
 *
 * Return the current value of the MSP (main stack pointer)
 * Cortex processor register
 */
/**
  * 函    数：读取主栈指针 MSP
  * 参    数：无
  * 返 回 值：MSP 当前值
  */
uint32_t __get_MSP(void)
{
  __ASM("mrs r0, msp");
  __ASM("bx lr");
}

/**
 * @brief  Set the Main Stack Pointer
 *
 * @param  topOfMainStack  Main Stack Pointer
 *
 * Assign the value mainStackPointer to the MSP 
 * (main stack pointer) Cortex processor register
 */
/**
  * 函    数：写入主栈指针 MSP
  * 参    数：topOfMainStack：要写入 MSP 的主栈指针
  * 返 回 值：无
  */
void __set_MSP(uint32_t topOfMainStack)
{
  __ASM("msr msp, r0");
  __ASM("bx lr");
}

/**
 * @brief  Reverse byte order in unsigned short value
 *
 * @param  value  value to reverse
 * @return        reversed value
 *
 * Reverse byte order in unsigned short value
 */
/**
  * 函    数：在两个半字内部各自交换高低字节
  * 参    数：value：要在半字内部交换字节的值
  * 返 回 值：以 32 位类型返回字节交换结果
  */
uint32_t __REV16(uint16_t value)
{
  __ASM("rev16 r0, r0");
  __ASM("bx lr");
}

/**
 * @brief  Reverse bit order of value
 *
 * @param  value  value to reverse
 * @return        reversed value
 *
 * Reverse bit order of value
 */
/**
  * 函    数：反转 32 位值的位顺序
  * 参    数：value：要反转位顺序的 32 位值
  * 返 回 值：位顺序反转后的 32 位值
  */
uint32_t __RBIT(uint32_t value)
{
  __ASM("rbit r0, r0");
  __ASM("bx lr");
}

/**
 * @brief  LDR Exclusive (8 bit)
 *
 * @param  *addr  address pointer
 * @return        value of (*address)
 *
 * Exclusive LDR command for 8 bit values)
 */
/**
  * 函    数：以独占访问方式读取一个 8 位值
  * 参    数：addr：访问地址，须按访问宽度对齐
  * 返 回 值：从指定地址读出的 8 位值
  */
uint8_t __LDREXB(uint8_t *addr)
{
  __ASM("ldrexb r0, [r0]");
  __ASM("bx lr"); 
}

/**
 * @brief  LDR Exclusive (16 bit)
 *
 * @param  *addr  address pointer
 * @return        value of (*address)
 *
 * Exclusive LDR command for 16 bit values
 */
/**
  * 函    数：以独占访问方式读取一个 16 位值
  * 参    数：addr：访问地址，须按访问宽度对齐
  * 返 回 值：从指定地址读出的 16 位值
  */
uint16_t __LDREXH(uint16_t *addr)
{
  __ASM("ldrexh r0, [r0]");
  __ASM("bx lr");
}

/**
 * @brief  LDR Exclusive (32 bit)
 *
 * @param  *addr  address pointer
 * @return        value of (*address)
 *
 * Exclusive LDR command for 32 bit values
 */
/**
  * 函    数：以独占访问方式读取一个 32 位值
  * 参    数：addr：访问地址，须按访问宽度对齐
  * 返 回 值：从指定地址读出的 32 位值
  */
uint32_t __LDREXW(uint32_t *addr)
{
  __ASM("ldrex r0, [r0]");
  __ASM("bx lr");
}

/**
 * @brief  STR Exclusive (8 bit)
 *
 * @param  value  value to store
 * @param  *addr  address pointer
 * @return        successful / failed
 *
 * Exclusive STR command for 8 bit values
 */
/**
  * 函    数：独占写入一个 8 位值
  * 参    数：value：要写入的 8 位值
  *           addr：访问地址，须按访问宽度对齐
  * 返 回 值：0 表示写入成功，1 表示独占条件失效
  */
uint32_t __STREXB(uint8_t value, uint8_t *addr)
{
  __ASM("strexb r0, r0, [r1]");
  __ASM("bx lr");
}

/**
 * @brief  STR Exclusive (16 bit)
 *
 * @param  value  value to store
 * @param  *addr  address pointer
 * @return        successful / failed
 *
 * Exclusive STR command for 16 bit values
 */
/**
  * 函    数：独占写入一个 16 位值
  * 参    数：value：要写入的 16 位值
  *           addr：访问地址，须按访问宽度对齐
  * 返 回 值：0 表示写入成功，1 表示独占条件失效
  */
uint32_t __STREXH(uint16_t value, uint16_t *addr)
{
  __ASM("strexh r0, r0, [r1]");
  __ASM("bx lr");
}

/**
 * @brief  STR Exclusive (32 bit)
 *
 * @param  value  value to store
 * @param  *addr  address pointer
 * @return        successful / failed
 *
 * Exclusive STR command for 32 bit values
 */
/**
  * 函    数：独占写入一个 32 位值
  * 参    数：value：要写入的 32 位值
  *           addr：访问地址，须按访问宽度对齐
  * 返 回 值：0 表示写入成功，1 表示独占条件失效
  */
uint32_t __STREXW(uint32_t value, uint32_t *addr)
{
  __ASM("strex r0, r0, [r1]");
  __ASM("bx lr");
}

#pragma diag_default=Pe940


#elif (defined (__GNUC__)) /*------------------ GNU Compiler ---------------------*/
/* GNU gcc specific functions */

/**
 * @brief  Return the Process Stack Pointer
 *
 * @return ProcessStackPointer
 *
 * Return the actual process stack pointer
 */
uint32_t __get_PSP(void) __attribute__( ( naked ) );
/**
  * 函    数：读取线程可使用的进程栈指针 PSP
  * 参    数：无
  * 返 回 值：PSP 当前值
  */
uint32_t __get_PSP(void)
{
  uint32_t result=0;

  __ASM volatile ("MRS %0, psp\n\t" 
                  "MOV r0, %0 \n\t"
                  "BX  lr     \n\t"  : "=r" (result) );
  return(result);
}

/**
 * @brief  Set the Process Stack Pointer
 *
 * @param  topOfProcStack  Process Stack Pointer
 *
 * Assign the value ProcessStackPointer to the MSP 
 * (process stack pointer) Cortex processor register
 */
void __set_PSP(uint32_t topOfProcStack) __attribute__( ( naked ) );
/**
  * 函    数：写入进程栈指针 PSP
  * 参    数：topOfProcStack：要写入 PSP 的进程栈指针
  * 返 回 值：无
  */
void __set_PSP(uint32_t topOfProcStack)
{
  __ASM volatile ("MSR psp, %0\n\t"
                  "BX  lr     \n\t" : : "r" (topOfProcStack) );
}

/**
 * @brief  Return the Main Stack Pointer
 *
 * @return Main Stack Pointer
 *
 * Return the current value of the MSP (main stack pointer)
 * Cortex processor register
 */
uint32_t __get_MSP(void) __attribute__( ( naked ) );
/**
  * 函    数：读取主栈指针 MSP
  * 参    数：无
  * 返 回 值：MSP 当前值
  */
uint32_t __get_MSP(void)
{
  uint32_t result=0;

  __ASM volatile ("MRS %0, msp\n\t" 
                  "MOV r0, %0 \n\t"
                  "BX  lr     \n\t"  : "=r" (result) );
  return(result);
}

/**
 * @brief  Set the Main Stack Pointer
 *
 * @param  topOfMainStack  Main Stack Pointer
 *
 * Assign the value mainStackPointer to the MSP 
 * (main stack pointer) Cortex processor register
 */
void __set_MSP(uint32_t topOfMainStack) __attribute__( ( naked ) );
/**
  * 函    数：写入主栈指针 MSP
  * 参    数：topOfMainStack：要写入 MSP 的主栈指针
  * 返 回 值：无
  */
void __set_MSP(uint32_t topOfMainStack)
{
  __ASM volatile ("MSR msp, %0\n\t"
                  "BX  lr     \n\t" : : "r" (topOfMainStack) );
}

/**
 * @brief  Return the Base Priority value
 *
 * @return BasePriority
 *
 * Return the content of the base priority register
 */
/**
  * 函    数：读取可屏蔽异常的优先级屏蔽阈值
  * 参    数：无
  * 返 回 值：BASEPRI 当前值，0 表示不设阈值
  */
uint32_t __get_BASEPRI(void)
{
  uint32_t result=0;
  
  __ASM volatile ("MRS %0, basepri_max" : "=r" (result) );
  return(result);
}

/**
 * @brief  Set the Base Priority value
 *
 * @param  basePri  BasePriority
 *
 * Set the base priority register
 */
/**
  * 函    数：设置可屏蔽异常的优先级屏蔽阈值
  * 参    数：value：写入 BASEPRI 的优先级门限，0 表示关闭该屏蔽
  * 返 回 值：无
  */
void __set_BASEPRI(uint32_t value)
{
  __ASM volatile ("MSR basepri, %0" : : "r" (value) );
}

/**
 * @brief  Return the Priority Mask value
 *
 * @return PriMask
 *
 * Return state of the priority mask bit from the priority mask register
 */
/**
  * 函    数：读取可配置优先级异常的全局屏蔽位
  * 参    数：无
  * 返 回 值：PRIMASK 当前值
  */
uint32_t __get_PRIMASK(void)
{
  uint32_t result=0;

  __ASM volatile ("MRS %0, primask" : "=r" (result) );
  return(result);
}

/**
 * @brief  Set the Priority Mask value
 *
 * @param  priMask  PriMask
 *
 * Set the priority mask bit in the priority mask register
 */
/**
  * 函    数：写入可配置优先级异常的全局屏蔽位
  * 参    数：priMask：0 解除 PRIMASK 屏蔽，1 使能屏蔽
  * 返 回 值：无
  */
void __set_PRIMASK(uint32_t priMask)
{
  __ASM volatile ("MSR primask, %0" : : "r" (priMask) );
}

/**
 * @brief  Return the Fault Mask value
 *
 * @return FaultMask
 *
 * Return the content of the fault mask register
 */
/**
  * 函    数：读取 FAULTMASK 故障屏蔽位
  * 参    数：无
  * 返 回 值：FAULTMASK 当前值
  */
uint32_t __get_FAULTMASK(void)
{
  uint32_t result=0;
  
  __ASM volatile ("MRS %0, faultmask" : "=r" (result) );
  return(result);
}

/**
 * @brief  Set the Fault Mask value
 *
 * @param  faultMask  faultMask value
 *
 * Set the fault mask register
 */
/**
  * 函    数：设置 FAULTMASK 故障屏蔽位
  * 参    数：faultMask：0 解除 FAULTMASK 屏蔽，1 使能屏蔽
  * 返 回 值：无
  */
void __set_FAULTMASK(uint32_t faultMask)
{
  __ASM volatile ("MSR faultmask, %0" : : "r" (faultMask) );
}

/**
 * @brief  Return the Control Register value
* 
*  @return Control value
 *
 * Return the content of the control register
 */
/**
  * 函    数：读取线程模式的栈选择和特权控制寄存器
  * 参    数：无
  * 返 回 值：CONTROL 当前值
  */
uint32_t __get_CONTROL(void)
{
  uint32_t result=0;

  __ASM volatile ("MRS %0, control" : "=r" (result) );
  return(result);
}

/**
 * @brief  Set the Control Register value
 *
 * @param  control  Control value
 *
 * Set the control register
 */
/**
  * 函    数：设置线程模式的栈选择和特权控制寄存器
  * 参    数：control：要写入 CONTROL 的值，包括线程模式的栈选择和特权位
  * 返 回 值：无
  */
void __set_CONTROL(uint32_t control)
{
  __ASM volatile ("MSR control, %0" : : "r" (control) );
}


/**
 * @brief  Reverse byte order in integer value
 *
 * @param  value  value to reverse
 * @return        reversed value
 *
 * Reverse byte order in integer value
 */
/**
  * 函    数：反转 32 位值中四个字节的位置
  * 参    数：value：要交换字节顺序的 32 位值
  * 返 回 值：字节序反转后的 32 位无符号值
  */
uint32_t __REV(uint32_t value)
{
  uint32_t result=0;
  
  __ASM volatile ("rev %0, %1" : "=r" (result) : "r" (value) );
  return(result);
}

/**
 * @brief  Reverse byte order in unsigned short value
 *
 * @param  value  value to reverse
 * @return        reversed value
 *
 * Reverse byte order in unsigned short value
 */
/**
  * 函    数：在两个半字内部各自交换高低字节
  * 参    数：value：要在半字内部交换字节的值
  * 返 回 值：以 32 位类型返回字节交换结果
  */
uint32_t __REV16(uint16_t value)
{
  uint32_t result=0;
  
  __ASM volatile ("rev16 %0, %1" : "=r" (result) : "r" (value) );
  return(result);
}

/**
 * @brief  Reverse byte order in signed short value with sign extension to integer
 *
 * @param  value  value to reverse
 * @return        reversed value
 *
 * Reverse byte order in signed short value with sign extension to integer
 */
/**
  * 函    数：交换低 16 位的两个字节并做符号扩展
  * 参    数：value：要交换字节并扩展的有符号 16 位值
  * 返 回 值：符号扩展后的 32 位结果
  */
int32_t __REVSH(int16_t value)
{
  uint32_t result=0;
  
  __ASM volatile ("revsh %0, %1" : "=r" (result) : "r" (value) );
  return(result);
}

/**
 * @brief  Reverse bit order of value
 *
 * @param  value  value to reverse
 * @return        reversed value
 *
 * Reverse bit order of value
 */
/**
  * 函    数：反转 32 位值的位顺序
  * 参    数：value：要反转位顺序的 32 位值
  * 返 回 值：位顺序反转后的 32 位值
  */
uint32_t __RBIT(uint32_t value)
{
  uint32_t result=0;
  
   __ASM volatile ("rbit %0, %1" : "=r" (result) : "r" (value) );
   return(result);
}

/**
 * @brief  LDR Exclusive (8 bit)
 *
 * @param  *addr  address pointer
 * @return        value of (*address)
 *
 * Exclusive LDR command for 8 bit value
 */
/**
  * 函    数：以独占访问方式读取一个 8 位值
  * 参    数：addr：访问地址，须按访问宽度对齐
  * 返 回 值：从指定地址读出的 8 位值
  */
uint8_t __LDREXB(uint8_t *addr)
{
    uint8_t result=0;
  
   __ASM volatile ("ldrexb %0, [%1]" : "=r" (result) : "r" (addr) );
   return(result);
}

/**
 * @brief  LDR Exclusive (16 bit)
 *
 * @param  *addr  address pointer
 * @return        value of (*address)
 *
 * Exclusive LDR command for 16 bit values
 */
/**
  * 函    数：以独占访问方式读取一个 16 位值
  * 参    数：addr：访问地址，须按访问宽度对齐
  * 返 回 值：从指定地址读出的 16 位值
  */
uint16_t __LDREXH(uint16_t *addr)
{
    uint16_t result=0;
  
   __ASM volatile ("ldrexh %0, [%1]" : "=r" (result) : "r" (addr) );
   return(result);
}

/**
 * @brief  LDR Exclusive (32 bit)
 *
 * @param  *addr  address pointer
 * @return        value of (*address)
 *
 * Exclusive LDR command for 32 bit values
 */
/**
  * 函    数：以独占访问方式读取一个 32 位值
  * 参    数：addr：访问地址，须按访问宽度对齐
  * 返 回 值：从指定地址读出的 32 位值
  */
uint32_t __LDREXW(uint32_t *addr)
{
    uint32_t result=0;
  
   __ASM volatile ("ldrex %0, [%1]" : "=r" (result) : "r" (addr) );
   return(result);
}

/**
 * @brief  STR Exclusive (8 bit)
 *
 * @param  value  value to store
 * @param  *addr  address pointer
 * @return        successful / failed
 *
 * Exclusive STR command for 8 bit values
 */
/**
  * 函    数：独占写入一个 8 位值
  * 参    数：value：要写入的 8 位值
  *           addr：访问地址，须按访问宽度对齐
  * 返 回 值：0 表示写入成功，1 表示独占条件失效
  */
uint32_t __STREXB(uint8_t value, uint8_t *addr)
{
   uint32_t result=0;
  
   __ASM volatile ("strexb %0, %2, [%1]" : "=r" (result) : "r" (addr), "r" (value) );
   return(result);
}

/**
 * @brief  STR Exclusive (16 bit)
 *
 * @param  value  value to store
 * @param  *addr  address pointer
 * @return        successful / failed
 *
 * Exclusive STR command for 16 bit values
 */
/**
  * 函    数：独占写入一个 16 位值
  * 参    数：value：要写入的 16 位值
  *           addr：访问地址，须按访问宽度对齐
  * 返 回 值：0 表示写入成功，1 表示独占条件失效
  */
uint32_t __STREXH(uint16_t value, uint16_t *addr)
{
   uint32_t result=0;
  
   __ASM volatile ("strexh %0, %2, [%1]" : "=r" (result) : "r" (addr), "r" (value) );
   return(result);
}

/**
 * @brief  STR Exclusive (32 bit)
 *
 * @param  value  value to store
 * @param  *addr  address pointer
 * @return        successful / failed
 *
 * Exclusive STR command for 32 bit values
 */
/**
  * 函    数：独占写入一个 32 位值
  * 参    数：value：要写入的 32 位值
  *           addr：访问地址，须按访问宽度对齐
  * 返 回 值：0 表示写入成功，1 表示独占条件失效
  */
uint32_t __STREXW(uint32_t value, uint32_t *addr)
{
   uint32_t result=0;
  
   __ASM volatile ("strex %0, %2, [%1]" : "=r" (result) : "r" (addr), "r" (value) );
   return(result);
}


#elif (defined (__TASKING__)) /*------------------ TASKING Compiler ---------------------*/
/* TASKING carm specific functions */

/*
 * The CMSIS functions have been implemented as intrinsics in the compiler.
 * Please use "carm -?i" to get an up to date list of all instrinsics,
 * Including the CMSIS ones.
 */

#endif
