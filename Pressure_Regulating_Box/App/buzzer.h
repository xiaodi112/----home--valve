#ifndef BUZZER_H
#define BUZZER_H

#include "gd32l23x.h"
#include <stdint.h>

/* BUZZER: PB8, HIGH=响, LOW=停 */
#define BUZZER_PORT  GPIOB
#define BUZZER_PIN   GPIO_PIN_8

/* GPIO时钟由 gpio_init() 统一使能，buzzer_init() 仅配置引脚模式 */
void    buzzer_init(void);
void    buzzer_on(void);
void    buzzer_off(void);
/* 蜂鸣 times 次, on_ms=鸣响时长, off_ms=间隔时长 */
void buzzer_beep(uint8_t times, uint16_t on_ms);

#endif
