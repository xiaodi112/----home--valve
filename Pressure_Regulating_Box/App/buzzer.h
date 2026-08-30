#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

/* 引脚配置在 BSP/output.c (output_buzzer_init, gpio_init 时统一调用) */
void    buzzer_init(void);
void    buzzer_on(void);
void    buzzer_off(void);
/* 蜂鸣 times 次, on_ms=鸣响时长, off_ms=间隔时长 */
void buzzer_beep(uint8_t times, uint16_t on_ms);

#endif
