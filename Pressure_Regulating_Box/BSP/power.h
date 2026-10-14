#ifndef POWER_H
#define POWER_H

#include "gd32l23x.h"

/* ==================== 板级电源引脚映射 ====================
 * 换板只改本表: 引脚全部收口在 BSP/power.c 与本头文件, 驱动/应用层不含裸引脚
 */
#include <stdint.h>

#define PWR_4G_PORT       GPIOB      /* V4G_EN: 4G 模块总电源使能 (高电平=使能, 低电平=关闭) */
#define PWR_4G_PIN        GPIO_PIN_13
#define PWR_4G_KEY_PORT   GPIOB      /* 4G_PWRKEY: 经 NPN 低侧开关, MCU 高电平=模块 PWRKEY 被拉低 */
#define PWR_4G_KEY_PIN    GPIO_PIN_14
#define PWR_BLE_PORT      GPIOC      /* SW_BT: 蓝牙模块电源 (HIGH=供电) */
#define PWR_BLE_PIN       GPIO_PIN_12
#define PWR_CH4_PORT      GPIOC      /* SW_Sensor: CH4 (HIGH=ON) */
#define PWR_CH4_PIN       GPIO_PIN_13
#define PWR_SENSOR_PORT   GPIOC      /* 传感器 IIC 电源 (HIGH=供电, 采集前上电/采集后断电) */
#define PWR_SENSOR_PIN    GPIO_PIN_3
#define PWR_4G_DTR_PORT   GPIOA      /* 4G_DTR: 模块休眠控制 (HIGH=允许休眠) */
#define PWR_4G_DTR_PIN    GPIO_PIN_0

/* 注意: 电压采集基准/使能脚已挪到 PB11, 归 ADC 模块管理 (见 BSP/adc_bat.c) */

/* 初始化全部电源控制引脚: V4G_EN/蓝牙/CH4 默认断电(LOW), DTR 默认 HIGH,
 * PWRKEY(PC1) 默认高阻释放态(输入), 脉冲由 EC 驱动临时切换输出 */
void power_gpio_init(void);

/* 4G 模块电源 (推挽输出驱动规则):
 *   1=供电 → 引脚推挽输出高 (电源使能)
 *   0=断电 → 引脚推挽输出低 (关闭电源)
 * 注: 引脚在深睡眠期间保持推挽输出与对应电平 (由 gpio_enter_low_power 重新施加),
 *     因此"4G 在线"时模块在整个深睡期间持续供电。
 *     复位后引脚为输入, 固件上电即按本模块状态施加电平 (默认断电 → 输出低)。 */
void power_4g_set(uint8_t on);
/* 4G 供电状态 (软件真源, 不依赖引脚回读): 1=供电, 0=断电 */
uint8_t power_4g_is_on(void);
/* 按当前供电状态重新施加引脚电气状态 (深睡配置/重入 sys_init 后调用) */
void power_4g_apply_state(void);

/* 蓝牙模块电源: 1=供电, 0=断电 */
void power_ble_set(uint8_t on);
uint8_t power_ble_is_on(void);

/* 蓝牙模块电源状态 (0=断电, 1=供电): 与 power_ble_set() 双向同步, 供状态机/日志读取 */
extern uint8_t g_bt_on;

/* 4G DTR: 1=HIGH(允许模块休眠), 0=LOW(强制唤醒) */
void power_4g_dtr_set(uint8_t level);

/* CH4: 1=, 0= */
void power_ch4_set(uint8_t on);
uint8_t power_ch4_is_on(void);

/* 传感器 IIC 电源 (PC3): 1=供电, 0=断电 */
void power_sensor_set(uint8_t on);
uint8_t power_sensor_is_on(void);

#endif /* POWER_H */
