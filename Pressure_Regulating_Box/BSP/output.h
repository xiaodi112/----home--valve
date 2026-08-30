#ifndef OUTPUT_H
#define OUTPUT_H

#include "gd32l23x.h"

/* ==================== 板级输出引脚映射 ==================== */
#define OUT_BUZZER_PORT   GPIOB      /* BUZZER: HIGH=响, LOW=停 */
#define OUT_BUZZER_PIN    GPIO_PIN_8
#define OUT_LED_PORT      GPIOA      /* LED1: LOW=亮, HIGH=灭 */
#define OUT_LED_PIN       GPIO_PIN_15

/* 初始化全部输出引脚(默认关闭): 蜂鸣器 LOW, LED 熄灭(HIGH) */
void output_gpio_init(void);
void output_buzzer_init(void);
void output_led_init(void);

/* 蜂鸣器: 1=响, 0=停 */
void output_buzzer_set(uint8_t on);
/* LED: 1=亮, 0=灭 */
void output_led_set(uint8_t on);
uint8_t output_led_is_on(void);

#endif /* OUTPUT_H */
