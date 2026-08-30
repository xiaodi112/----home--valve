#ifndef MYGPIO__H
#define MYGPIO__H

#include "gd32l23x.h"

/* 按键功能开关 (板级): 1=有按键功能, 0=无按键 (PD0 -> Analog) */
#define BUTTON_ENABLED                1

void gpio_init(void);

// ===================== PD0(Btn1)/PD1(Btn2) 按键唤醒 =====================
// 按键接地(按下=LOW), 内部上拉, EXTI0下降沿唤醒
extern volatile uint8_t button_wakeup_flag;
void button_exti_init(void);      // 深休眠前调用: 配置PD0/PD1+EXTI0
void button_exti_disable(void);   // 唤醒后调用: 禁用EXTI0防止重复触发
uint8_t button_is_pressed(void);  // 读取PD0(Btn1): 1=按下(LOW), 0=松开(HIGH)

// 进入极低功耗休眠配置 (关闭所有外设和未使用GPIO)
// lcd_active/bt_on 由调用方传入, 避免 BSP 依赖业务全局
void gpio_enter_low_power(uint8_t lpuart_enabled, uint8_t lcd_active, uint8_t bt_on);

// ===================== PD2 蓝牙连接状态引脚 =====================
// BLE模块连接成功后输出LOW, 未连接时为HIGH (内部上拉)
void ble_link_gpio_init(void);      // 配置PD2为输入+上拉
uint8_t ble_link_is_connected(void); // 1=已连接(LOW), 0=未连接(HIGH)

// ===================== PD2 蓝牙连接EXTI唤醒 =====================
// BLE开启后深休眠前调用: PD2下降沿EXTI唤醒 (手机连接时PD2→LOW)
extern volatile uint8_t ble_connect_wakeup_flag;
void ble_exti_init(void);            // 使能PD2 EXTI2下降沿唤醒
void ble_exti_disable(void);         // 禁用PD2 EXTI2

// ===================== PA1 4G RI引脚EXTI唤醒 =====================
// 4G模块收到服务器数据时RI脚拉低, PA1下降沿EXTI唤醒MCU (与LPUART互为冗余)
extern volatile uint8_t ri_wakeup_flag;
void ri_exti_init(void);             // 深休眠前调用: PA1输入上拉+EXTI1下降沿
void ri_exti_disable(void);          // 唤醒后调用: 禁用EXTI1

#endif
