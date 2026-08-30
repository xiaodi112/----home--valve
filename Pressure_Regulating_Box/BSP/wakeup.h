#ifndef WAKEUP_H
#define WAKEUP_H

#include "gd32l23x.h"

/* 深休眠配置: 由调用方(main/App)传入当前外设状态, BSP 不依赖业务全局 */
typedef struct {
    uint8_t lpuart_enabled;  /* 1=LPUART 起始位唤醒(4G long模式) */
    uint8_t lcd_active;      /* 1=LCD 显示中(SLCD 引脚保持 AF) */
    uint8_t bt_on;           /* 1=蓝牙开启(UART4/BLE 引脚保持) */
} PmSleepCfg_t;

/* 进入深休眠 (支持 LPUART/RTC闹钟/按键/4G RI 四种唤醒源) */
void enter_deepsleep_mode(const PmSleepCfg_t *cfg);
/* 唤醒后系统重初始化 (时钟/滴答/TIMER1/LPUART/EXTI) */
void system_wakeup_reinit(void);

#endif /* WAKEUP_H */
