#include "mygpio.h"



void gpio_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOF);

    /* 先写 ODR 再切输出模式, 避免模式切换瞬间的电平跳变:
     *   PB5(SW_IIC 电子开关): 默认 LOW (ODR 复位值=0), 由采集路径按需 set/reset 控制
     *   PA0(4G_DTR): 默认 HIGH (允许 EC800E 休眠)
     *   PF1(V4G_EN 4G 电源): 默认 LOW (4G 默认断电)
     */
    gpio_bit_set(GPIOA, GPIO_PIN_0);

    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_5);

    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_0);

    gpio_mode_set(GPIOF, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOF, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_1);
}

// ===================== PA12 按键唤醒 =====================
volatile uint8_t button_wakeup_flag = 0;

// 深休眠前调用: PA12输入+上拉 → EXTI12下降沿 → NVIC使能
void button_exti_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_SYSCFG);

    // PA12: 输入 + 内部上拉 (空闲HIGH, 按下LOW)
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_12);

    // EXTI12 → PA12, 下降沿触发
    syscfg_exti_line_config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN12);
    exti_init(EXTI_12, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_flag_clear(EXTI_12);

    // NVIC使能 EXTI10_15 共享中断
    nvic_irq_enable(EXTI10_15_IRQn, 2);

    button_wakeup_flag = 0;
}

// 唤醒后调用: 禁用EXTI12 + 清标志, 防止抖动重复触发
void button_exti_disable(void)
{
    exti_interrupt_disable(EXTI_12);
    exti_flag_clear(EXTI_12);
    nvic_irq_disable(EXTI10_15_IRQn);
}

// 读取PA12电平: 1=按下(LOW), 0=松开(HIGH)
uint8_t button_is_pressed(void)
{
    return (gpio_input_bit_get(GPIOA, GPIO_PIN_12) == RESET) ? 1 : 0;
}

// ===================== 极低功耗休眠配置 =====================
// 将休眠期间不需要的GPIO配置为 Analog Input (不带上下拉)，彻底杜绝引脚倒灌和悬空漏电
void gpio_enter_low_power(uint8_t lpuart_enabled)
{
    /* 1. 确保所有GPIO时钟开启以允许配置其寄存器 */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOF);

    /* 2. GPIOA 配置
     *    排除 PA0 (4G_DTR 输出, 休眠期间需保持 HIGH 允许 EC800E 休眠)
     *    排除 PA12 (按键 EXTI, 外部上拉)
     *    PA2/PA3 是 UART1 TX/RX (非 LPUART0), 休眠期间无需保留, 直接设为模拟
     */
    uint32_t gpioa_analog_pins = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | 
                                 GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | 
                                 GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_15;
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, gpioa_analog_pins);

    /* 3. GPIOB 配置
     *    排除 PB5 (SW_IIC 电子开关, 需保持 Output LOW 锁定关断状态)
     */
    uint32_t gpiob_analog_pins = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                                 GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7 |
                                 GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
                                 GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    gpio_mode_set(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, gpiob_analog_pins);

    /* 4. GPIOC 配置
     *    排除 PC14 和 PC15 (外部低速晶振 LXTAL，用于 RTC 保持, 绝对不能动!)
     */
    uint32_t gpioc_analog_pins = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | 
                                 GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | 
                                 GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | 
                                 GPIO_PIN_12 | GPIO_PIN_13;
    gpio_mode_set(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, gpioc_analog_pins);

    /* 5. GPIOD 配置
     *    PD8 = LPUART0_TX, PD9 = LPUART0_RX
     *    LPUART 启用时排除 PD8/PD9 保持 AF (用于 LPUART 唤醒), 否则一并设为模拟
     */
    uint32_t gpiod_analog_pins = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                                 GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                                 GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                                 GPIO_PIN_14 | GPIO_PIN_15;
    if (!lpuart_enabled)
    {
        gpiod_analog_pins |= (GPIO_PIN_8 | GPIO_PIN_9);  /* PD8=LPUART_TX, PD9=LPUART_RX */
    }
    gpio_mode_set(GPIOD, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, gpiod_analog_pins);

    /* 6. GPIOF 配置
     *    排除 PF1 (V4G_EN 4G 电源, 需保持 Output LOW 维持 4G 断电状态)
     */
    uint32_t gpiof_analog_pins = GPIO_PIN_0;
    gpio_mode_set(GPIOF, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, gpiof_analog_pins);
}

// ===================== 控制 4G 模组 DTR 引脚 (PA0) =====================
// level: 1 = HIGH (允许模块休眠); 0 = LOW (强制唤醒)
void gpio_set_4g_dtr(uint8_t level)
{
    if(level)
    {
        gpio_bit_set(GPIOA, GPIO_PIN_0);
    }
    else
    {
        gpio_bit_reset(GPIOA, GPIO_PIN_0);
    }
}