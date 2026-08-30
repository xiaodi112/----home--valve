#include "output.h"

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
    gpio_mode_set(OUT_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, OUT_LED_PIN);
    gpio_output_options_set(OUT_LED_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, OUT_LED_PIN);
    gpio_bit_set(OUT_LED_PORT, OUT_LED_PIN);   /* 默认熄灭 */
}

void output_buzzer_set(uint8_t on)
{
    if(on) gpio_bit_set(OUT_BUZZER_PORT, OUT_BUZZER_PIN);
    else   gpio_bit_reset(OUT_BUZZER_PORT, OUT_BUZZER_PIN);
}

void output_led_set(uint8_t on)
{
    if(on) gpio_bit_reset(OUT_LED_PORT, OUT_LED_PIN);
    else   gpio_bit_set(OUT_LED_PORT, OUT_LED_PIN);
}

uint8_t output_led_is_on(void)
{
    return (gpio_input_bit_get(OUT_LED_PORT, OUT_LED_PIN) == RESET) ? 1 : 0;
}
