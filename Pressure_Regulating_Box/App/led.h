#ifndef LED_H
#define LED_H

#include "gd32l23x.h"
#include <stdint.h>

/* LED1: PA15, HIGH=亮, LOW=灭 */
#define LED1_PORT   GPIOA
#define LED1_PIN    GPIO_PIN_15

/* GPIO时钟由 gpio_init() 统一使能，led_init() 仅配置引脚模式 */
void    led_init(void);
void    led_on(void);
void    led_off(void);
void    led_toggle(void);
/* 闪烁 times 次, period_ms 为完整亮灭周期(亮/灭各半) */
void    led_blink(uint8_t times, uint16_t period_ms);

#endif
