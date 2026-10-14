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
    /* 到位检测引脚: 按板级方式分派 (双限位 PA11/PA12 或 堵转电流 PB0) */
    motor_pos_init();
    /* PA5 (MOTOR_SLEEP): 电机使能, 默认 LOW (电机禁用) */
    gpio_mode_set(MOTOR_SLEEP_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, MOTOR_SLEEP_PIN);
    gpio_output_options_set(MOTOR_SLEEP_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, MOTOR_SLEEP_PIN);
#endif
}

// ===================== PA4(Btn1) 按键唤醒 =====================
volatile uint8_t button_wakeup_flag = 0;

// 深休眠前调用: PA4输入+上拉 → EXTI4下降沿 → NVIC使能
void button_exti_init(void)
{
#if BUTTON_ENABLED
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_SYSCFG);

    // PA4(Btn1): 输入, 内部上拉 (空闲HIGH, 按下LOW)
    gpio_mode_set(BUTTON_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BUTTON_PIN);

    // EXTI4 → PA4(Btn1), 下降沿触发唤醒
    syscfg_exti_line_config(BUTTON_EXTI_SOURCE, BUTTON_EXTI_SOURCE_PIN);
    exti_init(BUTTON_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_flag_clear(BUTTON_EXTI_LINE);

    // NVIC使能 EXTI4 独立中断
    nvic_irq_enable(BUTTON_EXTI_IRQn, 2);

    button_wakeup_flag = 0;
#endif
}

// 唤醒后调用: 禁用EXTI4 + 清标志, 防止抖动重复触发
void button_exti_disable(void)
{
#if BUTTON_ENABLED
    exti_interrupt_disable(BUTTON_EXTI_LINE);
    exti_flag_clear(BUTTON_EXTI_LINE);
    nvic_irq_disable(BUTTON_EXTI_IRQn);
#endif
}

// 读取PA4(Btn1)电平: 1=按下(LOW), 0=松开(HIGH)
uint8_t button_is_pressed(void)
{
#if BUTTON_ENABLED
    return (gpio_input_bit_get(BUTTON_PORT, BUTTON_PIN) == RESET) ? 1 : 0;
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

    (void)lcd_active;   /* 本板无 LCD (LCD_ENABLED=0), 参数保留以兼容 PmSleepCfg_t */

    /* 2. GPIOA 配置
     *    保持: PA0(4G DTR, 输出电平保持) / PA1(4G RI, EXTI)
     *          PA4(Btn1 唤醒 EXTI4, 输入上拉)
     *          PA5(M_SLP 电机使能, 输出低=禁用)
     *          PA10-PA12(三路 LED: 低电平点亮, 休眠期间保持短亮电平)
     *    Analog: PA2/PA3(CH4 未启用) / PA6/PA7(电机 PWM) / PA8/PA9 / PA15(空闲)
     */
    uint32_t gpioa_analog_pins = GPIO_PIN_2 | GPIO_PIN_3 |
                                 GPIO_PIN_6 | GPIO_PIN_7 |
                                 GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_15;
    if(!lpuart_enabled)
        gpioa_analog_pins |= (GPIO_PIN_0 | GPIO_PIN_1);  /* 4G断电: DTR/RI 转Analog */
#if !BUTTON_ENABLED
    gpioa_analog_pins |= BUTTON_PIN;                     /* 无按键: PA4 转Analog */
#endif
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, gpioa_analog_pins);

    /* PA5: 电机驱动使能, 保持输出并拉低 (禁用驱动, 防漏电) */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_5);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_5);
    gpio_bit_reset(GPIOA, GPIO_PIN_5);

    /* PA10-PA12: 三路 LED 保持推挽输出, 不改变 ODR (深睡期间由引脚电平维持短亮) */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  OUT_LED_CHECK_PIN | OUT_LED_VALVE_PIN | OUT_LED_BATT_PIN);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
                            OUT_LED_CHECK_PIN | OUT_LED_VALVE_PIN | OUT_LED_BATT_PIN);

#if BUTTON_ENABLED
    /* PA4 按键唤醒: 保持输入+内部上拉 (按下=LOW) */
    gpio_mode_set(BUTTON_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BUTTON_PIN);
#endif

    /* 3. GPIOB 配置
     *    Analog: PB0(电机电流采样输入) / PB1/PB2/PB5(空闲) / PB6/PB7(EEPROM IIC)
     *            PB9/PB10/PB12/PB15(空闲) / PB11(电压采集基准, 采样前重新拉低)
     *    保持:   PB8(蜂鸣器, 输出低) / PB13(V4G_EN, 保持输出电平)
     *            PB14(4G_PWRKEY, 输入高阻释放态)
     */
    uint32_t gpiob_analog_pins = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
                                 GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                                 GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
                                 GPIO_PIN_12 | GPIO_PIN_15;
    if(!bt_on)
        gpiob_analog_pins |= (GPIO_PIN_3 | GPIO_PIN_4);  /* 蓝牙关闭时PB3/PB4转Analog */
    gpio_mode_set(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, gpiob_analog_pins);

    /* PB8 蜂鸣器: 保持输出低 (停响) */
    gpio_mode_set(OUT_BUZZER_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, OUT_BUZZER_PIN);
    gpio_output_options_set(OUT_BUZZER_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, OUT_BUZZER_PIN);
    gpio_bit_reset(OUT_BUZZER_PORT, OUT_BUZZER_PIN);

    /* PB13 V4G_EN: 供电=推挽输出高, 断电=推挽输出低。
     * 深睡期间保持推挽输出与对应电平 (4G 在线时模块持续供电)。 */
    power_4g_apply_state();

    /* PB14 4G_PWRKEY: 保持输入高阻(释放态) */
    gpio_mode_set(PWR_4G_KEY_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, PWR_4G_KEY_PIN);

    /* 4. GPIOC 配置
     *    Analog: PC0/PC1(IIC2 低压传感器, 采集后已释放) / PC2(AD_BAT)
     *            PC4/PC5(调试串口) / PC6-PC11(空闲, 原 SLCD SEG)
     *    保持:   PC3(传感器电源, 输出低) / PC12(SW_BT 输出电平) / PC13(CH4 电源, 输出低)
     *    绝不动: PC14/PC15 (LXTAL, RTC 时钟)
     */
    uint32_t gpioc_analog_pins = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
                                 GPIO_PIN_4 | GPIO_PIN_5 |
                                 GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 |
                                 GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    gpio_mode_set(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, gpioc_analog_pins);

    power_sensor_set(0);    /* PC3: 传感器电源断电 */

    /* PC12 SW_BT: 保持推挽输出(电平不变: 蓝牙开HIGH / 关LOW) */
    gpio_mode_set(PWR_BLE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_BLE_PIN);
    gpio_output_options_set(PWR_BLE_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, PWR_BLE_PIN);

    /* PC13 CH4 电源: 保持输出低 (本产品无 CH4 硬件) */
    gpio_mode_set(PWR_CH4_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_CH4_PIN);
    gpio_output_options_set(PWR_CH4_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, PWR_CH4_PIN);
    gpio_bit_reset(PWR_CH4_PORT, PWR_CH4_PIN);

    /* 5. GPIOD 配置
     *    PD0/PD1(空闲, 按键已移至 PA4) / PD3/PD6/PD7/PD10-PD15 空闲
     *    PD4/PD5(IIC1 中压传感器, 采集后已转 Analog)
     *    PD8/PD9(LPUART0): LPUART启用时保留 AF
     *    PD2(BLE 连接检测): 蓝牙开启时保持 INPUT
     */
    uint32_t gpiod_analog_pins = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 |
                                 GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
                                 GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                                 GPIO_PIN_14 | GPIO_PIN_15;
    if(!bt_on)
        gpiod_analog_pins |= GPIO_PIN_2;  /* BLE关闭时PD2转Analog, BLE开启时保持INPUT */
    if (!lpuart_enabled)
    {
        gpiod_analog_pins |= (GPIO_PIN_8 | GPIO_PIN_9);  /* PD8=LPUART_TX, PD9=LPUART_RX */
    }
    gpio_mode_set(GPIOD, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, gpiod_analog_pins);

    /* 6. GPIOF 配置
     *    PF0/PF1 未使用, 全部设为模拟
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
