#include "led.h"
#include "output.h"
#include "delay.h"

void led_init(void)
{
    output_led_init();
}

void led_on(void)
{
    output_led_set(1);
}

void led_off(void)
{
    output_led_set(0);
}

void led_toggle(void)
{
    output_led_set(!output_led_is_on());
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
