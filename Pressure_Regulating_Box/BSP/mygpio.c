#include "mygpio.h"
#include "power.h"
#include "output.h"
#include "motor.h"



void gpio_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOF);

    /* 电源引脚 (V4G_EN/PWRKEY/BLE_EN/CH4/DTR): BSP/power.c 统一管理 */
    power_gpio_init();
    /* 蜂鸣器/LED 输出引脚: BSP/output.c 统一管理 */
    output_gpio_init();

#if MOTOR_ENABLED
    /* PA11/PA12 限位开关: 恢复输入+上拉 (深休眠 gpio_enter_low_power 会转 Analog) */
    motor_limit_init();
    /* PA5 (MOTOR_SLEEP): 电机使能, 默认 LOW (电机禁用) */
    gpio_mode_set(MOTOR_SLEEP_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, MOTOR_SLEEP_PIN);
    gpio_output_options_set(MOTOR_SLEEP_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, MOTOR_SLEEP_PIN);
#endif
}

// ===================== PD0(Btn1)/PD1(Btn2) 按键唤醒 =====================
volatile uint8_t button_wakeup_flag = 0;

// 深休眠前调用: PD0输入+上拉 → EXTI0下降沿 → NVIC使能
void button_exti_init(void)
{
#if BUTTON_ENABLED
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_SYSCFG);

    // PD0(Btn1)/PD1(Btn2): 输入, 内部上拉 (空闲HIGH, 按下LOW)
    gpio_mode_set(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_0);

    // EXTI0 → PD0(Btn1), 下降沿触发唤醒
    syscfg_exti_line_config(EXTI_SOURCE_GPIOD, EXTI_SOURCE_PIN0);
    exti_init(EXTI_0, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_flag_clear(EXTI_0);

    // NVIC使能 EXTI0 独立中断
    nvic_irq_enable(EXTI0_IRQn, 2);

    button_wakeup_flag = 0;
#endif
}

// 唤醒后调用: 禁用EXTI0 + 清标志, 防止抖动重复触发
void button_exti_disable(void)
{
#if BUTTON_ENABLED
    exti_interrupt_disable(EXTI_0);
    exti_flag_clear(EXTI_0);
    nvic_irq_disable(EXTI0_IRQn);
#endif
}

// 读取PD0(Btn1)电平: 1=按下(LOW), 0=松开(HIGH)
uint8_t button_is_pressed(void)
{
#if BUTTON_ENABLED
    return (gpio_input_bit_get(GPIOD, GPIO_PIN_0) == RESET) ? 1 : 0;
#else
    return 0;
#endif
}

// ===================== 极低功耗休眠配置 =====================
// 将休眠期间不需要的GPIO配置为 Analog Input (不带上下拉)，彻底杜绝引脚倒灌和悬空漏电
void gpio_enter_low_power(uint8_t lpuart_enabled, uint8_t lcd_active, uint8_t bt_on)
{
    /* 1. 确保所有GPIO时钟开启以允许配置其寄存器 */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_GPIOF);

    /* 2. GPIOA 配置
     *    排除 PA0 (4G DTR) / PA1 (4G RI) — 4G在线时 DTR 保持输出HIGH, RI 保持INPUT(EXTI)
     *    排除 PA8/PA9/PA10 (SLCD COM0/COM1/COM2, LCD开启时保持AF)
     */
    uint32_t gpioa_analog_pins = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
                                 GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                                 GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
    if(!lpuart_enabled)
        gpioa_analog_pins |= (GPIO_PIN_0 | GPIO_PIN_1);  /* PA0/PA1: 4G断电时转Analog */
    if(!lcd_active)
        gpioa_analog_pins |= (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);  /* PA8-10: SLCD COM0-2 */
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, gpioa_analog_pins);

    /* 3. GPIOB 配置
     *    排除 PB0/PB1 (SLCD SEG5/6), PB9 (SLCD COM3), PB10-PB15 (SLCD SEG10-15) — LCD开启时
     *    排除 PB3/PB4 (UART4_TX/RX) — 蓝牙开启时保持AF
     */
    uint32_t gpiob_analog_pins = GPIO_PIN_2 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
    if(!bt_on)
        gpiob_analog_pins |= (GPIO_PIN_3 | GPIO_PIN_4);  /* 蓝牙关闭时PB3/PB4转Analog */
    if(!lcd_active)
        gpiob_analog_pins |= (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_9 |
                              GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                              GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);  /* SLCD SEG+COM3 */
    gpio_mode_set(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, gpiob_analog_pins);

    /* 4. GPIOC 配置
     *    排除 PC0 (V4G_EN) — 4G在线时保持输出HIGH, 维持模块供电 (条件, 见下)
     *    排除 PC6-PC11 (SLCD SEG24-29, LCD开启时保持AF)
     *    排除 PC12 (SW_BT 输出)
     *    排除 PC14/PC15 (LXTAL, RTC保持, 绝对不动!)
     */
    uint32_t gpioc_analog_pins = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                                 GPIO_PIN_4 | GPIO_PIN_5 | 
                                 GPIO_PIN_13;
    if(!lpuart_enabled)
        gpioc_analog_pins |= GPIO_PIN_0;  /* PC0 (V4G_EN): 4G断电时转Analog */
    if(!lcd_active)
    {
        gpioc_analog_pins |= (GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 |
                              GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11);  /* SLCD SEG24-29 */
        rcu_periph_clock_disable(RCU_SLCD);  /* 关闭SLCD外设时钟 */
    }
    gpio_mode_set(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, gpioc_analog_pins);

    /* 5. GPIOD 配置
     *    PD0(Btn1): 按键唤醒 EXTI0, 保持 INPUT (外部上拉)
     *    PD1(Btn2): 保持 INPUT (外部上拉, 防悬空)
     *    PD4/PD5(IIC1): 中压传感器 I2C 总线, 采集后已转 Analog
     *    PD8=LPUART0_TX, PD9=LPUART0_RX: LPUART启用时保留 AF
     */
    uint32_t gpiod_analog_pins = GPIO_PIN_3 | GPIO_PIN_4 |
                                 GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                                 GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                                 GPIO_PIN_14 | GPIO_PIN_15;
#if !BUTTON_ENABLED
    gpiod_analog_pins |= (GPIO_PIN_0 | GPIO_PIN_1);  /* 无按键: PD0/PD1转Analog省电 */
#endif
    if(!bt_on)
        gpiod_analog_pins |= GPIO_PIN_2;  /* BLE关闭时PD2转Analog, BLE开启时保持INPUT */
    if (!lpuart_enabled)
    {
        gpiod_analog_pins |= (GPIO_PIN_8 | GPIO_PIN_9);  /* PD8=LPUART_TX, PD9=LPUART_RX */
    }
    gpio_mode_set(GPIOD, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, gpiod_analog_pins);

    /* 6. GPIOF 配置
     *    PF1 未使用, 全部设为模拟
     */
    uint32_t gpiof_analog_pins = GPIO_PIN_0 | GPIO_PIN_1;
    gpio_mode_set(GPIOF, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, gpiof_analog_pins);
}

// ===================== PD2 蓝牙连接状态引脚 =====================
// BLE模块(VG6328A-MS)连接成功后输出LOW, 未连接时HIGH
void ble_link_gpio_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOD);
    gpio_mode_set(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_2);
}

uint8_t ble_link_is_connected(void)
{
    return (gpio_input_bit_get(GPIOD, GPIO_PIN_2) == RESET) ? 1 : 0;
}

// ===================== PD2 蓝牙连接 EXTI 唤醒 =====================
volatile uint8_t ble_connect_wakeup_flag = 0;

// BLE开启时深休眠前调用: PD2 EXTI2 下降沿唤醒
void ble_exti_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_SYSCFG);

    // PD2: 输入, 内部上拉 (空闲HIGH, 连接LOW)
    gpio_mode_set(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_2);

    // EXTI2 → PD2, 下降沿触发
    syscfg_exti_line_config(EXTI_SOURCE_GPIOD, EXTI_SOURCE_PIN2);
    exti_init(EXTI_2, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_flag_clear(EXTI_2);

    // GD32L23x: EXTI2独立中断
    nvic_irq_enable(EXTI2_IRQn, 2);

    ble_connect_wakeup_flag = 0;
}

void ble_exti_disable(void)
{
    exti_interrupt_disable(EXTI_2);
    exti_flag_clear(EXTI_2);
    nvic_irq_disable(EXTI2_IRQn);
}

// ===================== PA1 4G RI引脚 EXTI 唤醒 =====================
// EC800E 收到服务器下行数据时 RI 脚产生低电平脉冲(~120ms)
// PA1 下降沿中断唤醒 MCU, 与 LPUART 起始位唤醒互为冗余
volatile uint8_t ri_wakeup_flag = 0;

void ri_exti_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_SYSCFG);

    // PA1: 输入, 内部上拉 (RI空闲HIGH, 有数据时LOW)
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_1);

    // EXTI1 → PA1, 下降沿触发
    syscfg_exti_line_config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN1);
    exti_init(EXTI_1, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_flag_clear(EXTI_1);

    nvic_irq_enable(EXTI1_IRQn, 1);

    ri_wakeup_flag = 0;
}

void ri_exti_disable(void)
{
    exti_interrupt_disable(EXTI_1);
    exti_flag_clear(EXTI_1);
    nvic_irq_disable(EXTI1_IRQn);
}
