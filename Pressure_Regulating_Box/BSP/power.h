#ifndef POWER_H
#define POWER_H

#include "gd32l23x.h"

/* ==================== 板级电源引脚映射 ==================== */
#define PWR_4G_PORT       GPIOB      /* V4G_EN: 4G 模块电源 (HIGH=供电) */
#define PWR_4G_PIN        GPIO_PIN_5
#define PWR_BLE_PORT      GPIOC      /* SW_BT: 蓝牙模块电源 (HIGH=供电) */
#define PWR_BLE_PIN       GPIO_PIN_12
#define PWR_SENSOR_PORT   GPIOC      /* SENSOR_PWR: 传感器模块电源/上拉 (HIGH=供电) */
#define PWR_SENSOR_PIN    GPIO_PIN_3
#define PWR_CH4_PORT      GPIOC      /* SW_Sensor: CH4 (HIGH=ON) */
#define PWR_CH4_PIN       GPIO_PIN_13
#define PWR_4G_DTR_PORT   GPIOA      /* 4G_DTR: 模块休眠控制 (HIGH=允许休眠) */
#define PWR_4G_DTR_PIN    GPIO_PIN_0

/* 传感器电源开关: 1=PC3 供电+上拉(当前默认, 传感器上电); 0=传感器常供电板型 */
#define SENSOR_PWR_ENABLED         1
/* PC3 上电稳定延时(毫秒) */
#define SENSOR_PWRUP_DELAY_MS      100

/* 初始化全部电源控制引脚(默认全部断电/LOW, DTR 默认 HIGH 允许休眠) */
void power_gpio_init(void);

/* 4G 模块电源: 1=供电, 0=断电 */
void power_4g_set(uint8_t on);
uint8_t power_4g_is_on(void);

/* 蓝牙模块电源: 1=供电, 0=断电 */
void power_ble_set(uint8_t on);

/* 传感器模块电源: 1=供电, 0=断电 */
void power_sensor_set(uint8_t on);

/* 4G DTR: 1=HIGH(允许模块休眠), 0=LOW(强制唤醒) */
void power_4g_dtr_set(uint8_t level);

/* CH4: 1=, 0= */
void power_ch4_set(uint8_t on);
uint8_t power_ch4_is_on(void);

#endif /* POWER_H */
