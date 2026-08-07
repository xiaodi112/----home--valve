#include "buzzer.h"
#include "delay.h"

void buzzer_init(void)
{
    /* RCU_GPIOB 由 gpio_init() 已使能，此处直接配置引脚 */
    gpio_mode_set(BUZZER_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BUZZER_PIN);
    gpio_output_options_set(BUZZER_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, BUZZER_PIN);
    gpio_bit_reset(BUZZER_PORT, BUZZER_PIN);    /* 默认关闭 */
}

void buzzer_on(void)
{
    gpio_bit_set(BUZZER_PORT, BUZZER_PIN);
}

void buzzer_off(void)
{
    gpio_bit_reset(BUZZER_PORT, BUZZER_PIN);
}

void buzzer_beep(uint8_t times, uint16_t on_ms)
{
    for(uint8_t i = 0; i < times; i++)
    {
        buzzer_on();
        trea_delay_ms(on_ms);
        buzzer_off();
        if(i < times - 1)
            trea_delay_ms(200);
    }
}
