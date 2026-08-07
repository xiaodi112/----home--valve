#include "led.h"
#include "delay.h"

void led_init(void)
{
    /* RCU_GPIOA 由 gpio_init() 已使能，此处直接配置引脚 */
    gpio_mode_set(LED1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED1_PIN);
    gpio_output_options_set(LED1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, LED1_PIN);
    gpio_bit_set(LED1_PORT, LED1_PIN);    /* 默认息灭 */
}

void led_on(void)
{
    gpio_bit_reset(LED1_PORT, LED1_PIN);
}

void led_off(void)
{
    gpio_bit_set(LED1_PORT, LED1_PIN);
}

void led_toggle(void)
{
    if(gpio_input_bit_get(LED1_PORT, LED1_PIN) == SET)
        gpio_bit_reset(LED1_PORT, LED1_PIN);
    else
        gpio_bit_set(LED1_PORT, LED1_PIN);
}

void led_blink(uint8_t times, uint16_t period_ms)
{
    uint16_t half = period_ms / 2;
    for(uint8_t i = 0; i < times; i++)
    {
        led_on();
        trea_delay_ms(half);
        led_off();
    }
}
