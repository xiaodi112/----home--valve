#ifndef OUTPUT_H
#define OUTPUT_H

#include "gd32l23x.h"

/* ==================== 板级输出引脚映射 ==================== */
#define OUT_BUZZER_PORT   GPIOB      /* BUZZER: HIGH=响, LOW=停 */
#define OUT_BUZZER_PIN    GPIO_PIN_8

/* 三路状态灯: 低电平点亮 (LOW=亮, HIGH=灭) */
#define OUT_LED_CHECK_PORT  GPIOA    /* LED1 检修: 阀门故障时亮 */
#define OUT_LED_CHECK_PIN   GPIO_PIN_12
#define OUT_LED_VALVE_PORT  GPIOA    /* LED2 阀关: 阀门处于关闭位时亮 */
#define OUT_LED_VALVE_PIN   GPIO_PIN_11
#define OUT_LED_BATT_PORT   GPIOA    /* LED3 换电池: 电压低于 spupower.low 时亮 */
#define OUT_LED_BATT_PIN    GPIO_PIN_10

/* 初始化全部输出引脚(默认关闭): 蜂鸣器 LOW, LED 熄灭(HIGH) */
void output_gpio_init(void);
void output_buzzer_init(void);
void output_led_init(void);          /* 初始化三路 LED, 默认熄灭 */

/* 蜂鸣器: 1=响, 0=停 */
void output_buzzer_set(uint8_t on);

/* 三路 LED: 1=亮(引脚拉低), 0=灭 */
void output_led_check_set(uint8_t on);
void output_led_valve_set(uint8_t on);
void output_led_batt_set(uint8_t on);
/* 三路 LED 全灭 */
void output_led_all_off(void);
/* 三路 LED 全亮 */
void output_led_all_on(void);
#endif /* OUTPUT_H */
