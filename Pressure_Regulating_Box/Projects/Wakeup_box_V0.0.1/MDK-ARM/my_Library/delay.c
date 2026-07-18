#include "delay.h"

#include "gd32l23x.h"
#include <stdio.h>
#include <stddef.h>

// ==================== SysTick 毫秒计数器 ====================
// 由 trea_time_isr() 每 1ms 递增（systick_config 配置为 HCLK/1000 中断）.
// 深休眠期间 SysTick 停止，s_ms_counter 不递增；唤醒后继续累加.
// FreeRTOS 迁移: 替换为 xTaskGetTickCount().
static volatile uint32_t s_ms_counter = 0;

// ==================== 软件看门狗 ====================
// 设计: 在每 1ms 的 SysTick 回调中累加计数, 超过超时阈值则 NVIC_SystemReset().
//   - 仅在唤醒后的"活跃处理阶段"使能 (swdt_enable), 进入深休眠前关闭 (swdt_disable).
//   - 深休眠期间 SysTick 停摆, 计数自然冻结, 不会误复位, 不影响低功耗.
//   - swdt_feed() 在长耗时操作的关键节点调用以复位计数.
static volatile uint32_t s_swdt_counter    = 0;   // 距上次喂狗的毫秒数
static volatile uint32_t s_swdt_timeout_ms = 0;   // 超时阈值 (ms)
static volatile uint8_t  s_swdt_enabled    = 0;   // 1=已使能

void swdt_enable(uint32_t timeout_ms)
{
    s_swdt_timeout_ms = timeout_ms;
    s_swdt_counter    = 0;
    s_swdt_enabled    = (timeout_ms != 0) ? 1 : 0;
}

void swdt_disable(void)
{
    s_swdt_enabled = 0;
    s_swdt_counter = 0;
}

void swdt_feed(void)
{
    s_swdt_counter = 0;
}

// ==================== TIMER1 @ 1MHz 自由运行计数器 ====================
// 仅供 delay_us() 使用; 1 tick = 1μs.  GD32L233 TIMER1 为 16位, 最大 65535μs.
// 深休眠后 APB1 时钟停止, TIMER1 停止计数; 唤醒后必须重新调用此函数.
void timer1_delay_init(void)
{
    timer_parameter_struct timer_initpara;

    SystemCoreClockUpdate(); // 确保 SystemCoreClock 反映唤醒后的实际时钟
    rcu_periph_clock_enable(RCU_TIMER1);
    timer_deinit(TIMER1);

    timer_initpara.prescaler         = (uint16_t)(SystemCoreClock / 1000000U) - 1U; // 计数频率 1MHz
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 0xFFFFU; // 16位最大值, 自由运行
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;

    timer_init(TIMER1, &timer_initpara);
    timer_enable(TIMER1);
}

// SysTick ISR 回调: 每 1ms 递增 s_ms_counter.
// FreeRTOS 迁移: 此函数可留空, trea_millis 改为 xTaskGetTickCount().
void trea_time_isr(void)
{
    s_ms_counter++;

    // 软件看门狗: 仅在使能时计时, 超时复位 MCU
    if(s_swdt_enabled)
    {
        if(++s_swdt_counter >= s_swdt_timeout_ms)
        {
            s_swdt_enabled = 0;   // 防止复位前重复进入
            NVIC_SystemReset();
        }
    }
}

// my_systick_config: mysystick_delay.c 未加入工程, 此处提供空实现.
// SysTick 已由 systick_config() 以 HCLK/1ms 正确配置, 无需额外操作.
void my_systick_config(void)
{
}

// ==================== trea_millis ====================
// 返回自启动以来的毫秒数(不含深休眠时间), 溢出周期 ~49.7天.
uint32_t trea_millis(void)
{
    return s_ms_counter;
}

// ==================== delay_us ====================
// 微秒忙等, 基于 TIMER1 计数器差值(1 tick = 1μs).
// TIMER1 为 16位, uint16_t 减法天然处理单次绕回; 单次调用不得超过 65535μs.
void delay_us(uint32_t counts)
{
    if(counts == 0) return;
    uint16_t start = (uint16_t)TIMER_CNT(TIMER1);
    while((uint16_t)(TIMER_CNT(TIMER1) - start) < (uint16_t)counts);
}

// ==================== trea_delay_ms ====================
// 毫秒忙等, 基于 TIMER1 硬件计数器(每 ms 调用一次 delay_us).
// 不依赖 SysTick ISR; 允许中断正常运行.
// FreeRTOS 迁移替换为 vTaskDelay(pdMS_TO_TICKS(counts)).
void trea_delay_ms(uint32_t counts)
{
    while(counts--)
        delay_us(1000);
}