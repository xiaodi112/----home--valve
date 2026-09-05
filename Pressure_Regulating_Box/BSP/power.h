#ifndef POWER_H
#define POWER_H

#include "gd32l23x.h"

/* ==================== 板级电源引脚映射 ==================== */
#define PWR_4G_PORT       GPIOC      /* V4G_EN: 4G 模块总电源 (HIGH=供电) */
#define PWR_4G_PIN        GPIO_PIN_0
#define PWR_4G_KEY_PORT   GPIOC      /* 4G_PWRKEY: 经 NPN 低侧开关, MCU 高电平=模块 PWRKEY 被拉低 */
#define PWR_4G_KEY_PIN    GPIO_PIN_1
#define PWR_BLE_PORT      GPIOC      /* SW_BT: 蓝牙模块电源 (HIGH=供电) */
#define PWR_BLE_PIN       GPIO_PIN_12
#define PWR_CH4_PORT      GPIOC      /* SW_Sensor: CH4 (HIGH=ON) */
#define PWR_CH4_PIN       GPIO_PIN_13
#define PWR_4G_DTR_PORT   GPIOA      /* 4G_DTR: 模块休眠控制 (HIGH=允许休眠) */
#define PWR_4G_DTR_PIN    GPIO_PIN_0

/* 注意: PC3 已改作电压采集基准 (见 adc_bat.c), 归 ADC 模块管理, 不再在此定义 */

/* 初始化全部电源控制引脚: V4G_EN/蓝牙/CH4 默认断电(LOW), DTR 默认 HIGH,
 * PWRKEY(PC1) 默认高阻释放态(输入), 脉冲由 EC 驱动临时切换输出 */
void power_gpio_init(void);

/* 4G 模块电源: 1=供电, 0=断电 */
void power_4g_set(uint8_t on);
uint8_t power_4g_is_on(void);

/* 蓝牙模块电源: 1=供电, 0=断电 */
void power_ble_set(uint8_t on);

/* 4G DTR: 1=HIGH(允许模块休眠), 0=LOW(强制唤醒) */
void power_4g_dtr_set(uint8_t level);

/* CH4: 1=, 0= */
void power_ch4_set(uint8_t on);
uint8_t power_ch4_is_on(void);

#endif /* POWER_H */
