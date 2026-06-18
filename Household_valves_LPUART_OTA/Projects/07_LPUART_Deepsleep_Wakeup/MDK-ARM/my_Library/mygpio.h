#ifndef MYGPIO__H
#define MYGPIO__H

#include "gd32l23x.h"

void gpio_init(void);

// ===================== PA12 按键唤醒 =====================
// 按键接地(按下=LOW), 内部上拉, EXTI12下降沿唤醒
extern volatile uint8_t button_wakeup_flag;
void button_exti_init(void);      // 深休眠前调用: 配置PA12+EXTI12
void button_exti_disable(void);   // 唤醒后调用: 禁用EXTI12防止重复触发
uint8_t button_is_pressed(void);  // 读取PA12: 1=按下(LOW), 0=松开(HIGH)

// 进入极低功耗休眠配置 (关闭所有外设和未使用GPIO)
void gpio_enter_low_power(uint8_t lpuart_enabled);

// 控制 4G 模组 DTR 引脚 (PA0): 1=HIGH (允许休眠), 0=LOW (强制唤醒)
void gpio_set_4g_dtr(uint8_t level);

#endif
