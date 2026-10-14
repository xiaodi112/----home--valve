#include "output.h"

/* 三路 LED 最近一次设定值 (1=亮)。
 * gpio_init() 在运行期会被重复调用 (sys_init / 唤醒恢复 / 4G 补读前), 若每次重配
 * 引脚都一律熄灭, 会把"按状态长亮"和"三灯同步提示"打断 (最坏情况是配完就进深睡,
 * 中途没有刷新点, 灯会一直灭到下次唤醒)。这里记录并在重配后回放, 与硬件 ODR
 * 在重配后失效的行为对齐; 上电初值 0 = 全灭, 冷启动行为与旧版一致。 */
static uint8_t s_led_check_on = 0;
static uint8_t s_led_valve_on = 0;
static uint8_t s_led_batt_on  = 0;

void output_gpio_init(void)
{
    output_buzzer_init();
    output_led_init();
}

void output_buzzer_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_mode_set(OUT_BUZZER_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, OUT_BUZZER_PIN);
    gpio_output_options_set(OUT_BUZZER_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, OUT_BUZZER_PIN);
    gpio_bit_reset(OUT_BUZZER_PORT, OUT_BUZZER_PIN);   /* 默认关闭 */
}

void output_led_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_mode_set(OUT_LED_CHECK_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                  OUT_LED_CHECK_PIN | OUT_LED_VALVE_PIN | OUT_LED_BATT_PIN);
    gpio_output_options_set(OUT_LED_CHECK_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
                            OUT_LED_CHECK_PIN | OUT_LED_VALVE_PIN | OUT_LED_BATT_PIN);
    /* 回放最近设定值 (冷启动为全灭) */
    output_led_check_set(s_led_check_on);
    output_led_valve_set(s_led_valve_on);
    output_led_batt_set(s_led_batt_on);
}

void output_buzzer_set(uint8_t on)
{
    if(on) gpio_bit_set(OUT_BUZZER_PORT, OUT_BUZZER_PIN);
    else   gpio_bit_reset(OUT_BUZZER_PORT, OUT_BUZZER_PIN);
}

/* 低电平点亮: on=1 → 引脚拉低 */
void output_led_check_set(uint8_t on)
{
    s_led_check_on = on ? 1u : 0u;
    if(on) gpio_bit_reset(OUT_LED_CHECK_PORT, OUT_LED_CHECK_PIN);
    else   gpio_bit_set(OUT_LED_CHECK_PORT, OUT_LED_CHECK_PIN);
}

void output_led_valve_set(uint8_t on)
{
    s_led_valve_on = on ? 1u : 0u;
    if(on) gpio_bit_reset(OUT_LED_VALVE_PORT, OUT_LED_VALVE_PIN);
    else   gpio_bit_set(OUT_LED_VALVE_PORT, OUT_LED_VALVE_PIN);
}

void output_led_batt_set(uint8_t on)
{
    s_led_batt_on = on ? 1u : 0u;
    if(on) gpio_bit_reset(OUT_LED_BATT_PORT, OUT_LED_BATT_PIN);
    else   gpio_bit_set(OUT_LED_BATT_PORT, OUT_LED_BATT_PIN);
}

void output_led_all_off(void)
{
    output_led_check_set(0);
    output_led_valve_set(0);
    output_led_batt_set(0);
}

void output_led_all_on(void)
{
    output_led_check_set(1);
    output_led_valve_set(1);
    output_led_batt_set(1);
}
