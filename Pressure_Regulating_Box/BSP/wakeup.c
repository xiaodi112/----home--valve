/**
 * @file wakeup.c
 * @brief 深休眠入口与唤醒后系统重初始化 (BSP 电源/唤醒管理)
 */

#include "wakeup.h"
#include "usart.h"
#include "mygpio.h"
#include "systick.h"
#include "delay.h"
#include "gd32l23x_libopt.h"

/* 唤醒源标志 (定义于 BSP 层, 供休眠等待使用) */
extern volatile uint8_t rtc_alarm_flag;      /* BSP/tim.c */
extern volatile uint8_t button_wakeup_flag;  /* BSP/mygpio.c */
extern volatile uint8_t ri_wakeup_flag;      /* BSP/mygpio.c */

static void system_clock_reconfig(void);

void enter_deepsleep_mode(const PmSleepCfg_t *cfg)
{
    uint8_t lpuart_enabled = cfg ? cfg->lpuart_enabled : 0;
    uint8_t lcd_active     = cfg ? cfg->lcd_active : 0;
    uint8_t bt_on          = cfg ? cfg->bt_on : 0;

    counter0 = 0x00;
    rtc_alarm_flag = 0;
    button_wakeup_flag = 0;
    ri_wakeup_flag = 0;

    // 软件看门狗: 进入深休眠前必须关闭 (深休眠期间 SysTick 停摆, 否则唤醒后计数错乱)
    swdt_disable();

    // 配置未使用GPIO 为 ANALOG 以极致降低漏电
    gpio_enter_low_power(lpuart_enabled, lcd_active, bt_on);

    rcu_periph_clock_enable(RCU_PMU);
    pmu_to_deepsleepmode(PMU_LDNPDSP_LOWDRIVE, WFI_CMD, PMU_DEEPSLEEP);

    // 等待任意唤醒源
    while((0x00 == counter0) && (0 == rtc_alarm_flag) && (0 == button_wakeup_flag) && (0 == ri_wakeup_flag));
}

void system_wakeup_reinit(void)
{
    // 恢复时钟 + 滴答定时器 + TIMER1（必须最先执行）
    system_clock_reconfig();
    systick_config();
    my_systick_config();
    timer1_delay_init(); // 深休眠后 APB1 停摆, 必须重新启动 TIMER1 计数器

    // 无论哪种唤醒源，都必须关闭LPUART唤醒模式和中断
    lpuart_wakeup_disable(LPUART0);
    lpuart_interrupt_disable(LPUART0, LPUART_INT_WU);
    exti_flag_clear(EXTI_28);

    // 关闭RI引脚EXTI (防止唤醒后RI持续低电平重复触发)
    ri_exti_disable();

    // LPUART唤醒时：消费触发唤醒的字符, 深休眠期间DMA未运行, 字节在RDATA中
    if(counter0 != 0x00)
    {
        if(RESET != lpuart_flag_get(LPUART0, LPUART_FLAG_RBNE))
            lpuart_data_receive(LPUART0);
        lpuart_receive_config(LPUART0, LPUART_RECEIVE_ENABLE);
        while(RESET == lpuart_flag_get(LPUART0, LPUART_FLAG_TC));
        lpuart_disable(LPUART0);
    }
}

/* ===================== 时钟配置 ===================== */
static void system_clock_reconfig(void)
{
    RCU_CTL |= RCU_CTL_IRC16MEN;
    while(rcu_osci_stab_wait(RCU_IRC16M) != SUCCESS);

    rcu_ahb_clock_config(RCU_AHB_CKSYS_DIV1);
    rcu_apb1_clock_config(RCU_APB1_CKAHB_DIV1);
    rcu_apb2_clock_config(RCU_APB2_CKAHB_DIV1);

    RCU_CTL &= ~RCU_CTL_PLLEN;
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_IRC16M;
}
