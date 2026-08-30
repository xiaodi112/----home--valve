#ifndef LED_H
#define LED_H

#include <stdint.h>

/* LED1: PA15, LOW=亮, HIGH=灭 (引脚配置在 BSP/output.c) */
void    led_init(void);
void    led_on(void);
void    led_off(void);
void    led_toggle(void);
/* 闪烁 times 次, period_ms 为完整亮灭周期(亮/灭各半) */
void    led_blink(uint8_t times, uint16_t period_ms);

#endif
