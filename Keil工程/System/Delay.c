#include "stm32f10x.h"
#include "Delay.h"

/* DWT 周期计数延时。工程所带的 CMSIS 未提供 DWT 结构体，下面直接使用寄存器地址。
 * 计数器独立于 SysTick，可在本工程的中断内调用。延时期间 CPU 忙等待。
 */
#define DEMCR_ADDR      0xE000EDFCUL
#define DWT_CTRL_ADDR   0xE0001000UL
#define DWT_CYCCNT_ADDR 0xE0001004UL
#define TRCENA_BIT      (1UL << 24)
#define CYCCNTENA_BIT   (1UL << 0)

/**
  * 函    数：使能 DWT 内核周期计数器
  * 参    数：无
  * 返 回 值：无
  */
void Delay_Init(void)
{
    *(volatile uint32_t *)DEMCR_ADDR |= TRCENA_BIT;  // 打开内核跟踪模块访问
    *(volatile uint32_t *)DWT_CYCCNT_ADDR = 0;      // 将周期计数清零
    *(volatile uint32_t *)DWT_CTRL_ADDR |= CYCCNTENA_BIT;  // 使能 CYCCNT 自动累加
}

/**
  * 函    数：按 DWT 周期差忙等待指定微秒
  * 参    数：us：延时时长，单位 μs，换算的周期数须能用 uint32_t 表示
  * 返 回 值：无
  */
void Delay_us(uint32_t us)
{
    uint32_t start;
    uint32_t ticks;

    if (us == 0)
    {
        return;
    }

    start = *(volatile uint32_t *)DWT_CYCCNT_ADDR;  // 记录本次延时的起始计数
    ticks = us * (SystemCoreClock / 1000000U);      // 按内核时钟将微秒换算为周期数

    while ((*(volatile uint32_t *)DWT_CYCCNT_ADDR - start) < ticks)  // 用无符号计数差判断到期，允许计数器回绕
    {
        /* 轮询周期计数器，直至达到目标 */
    }
}

/**
  * 函    数：按毫秒分段执行阻塞延时
  * 参    数：ms：延时时长，单位 ms
  * 返 回 值：无
  */
void Delay_ms(uint32_t ms)
{
    while (ms--)
    {
        Delay_us(1000);                             // 单次等待 1 ms，累计达到目标时长
    }
}

/**
  * 函    数：按秒分段执行阻塞延时
  * 参    数：s：延时时长，单位 s
  * 返 回 值：无
  */
void Delay_s(uint32_t s)
{
    while (s--)
    {
        Delay_ms(1000);                             // 单次等待 1 s
    }
}
