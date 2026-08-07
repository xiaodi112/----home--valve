#ifndef __MOTOR_H
#define __MOTOR_H

#include "gd32l23x.h"

void pwm_init(uint32_t freq, float duty_ch0, float duty_ch1);
void pwm_set_freq(uint32_t freq);
void pwm_set_duty(uint8_t ch, float duty);

#endif


