#include "buzzer.h"
#include "output.h"
#include "delay.h"

void buzzer_init(void)
{
    output_buzzer_init();
}

void buzzer_on(void)
{
    output_buzzer_set(1);
}

void buzzer_off(void)
{
    output_buzzer_set(0);
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
