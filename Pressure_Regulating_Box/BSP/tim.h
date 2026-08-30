#ifndef TIM_H
#define TIM_H

#include <stdint.h>
#include <stdio.h>
#include "gd32l23x.h"

// 时钟源选择 (IRC32K=内部RC, 无需外部晶振; LXTAL=外部32.768kHz晶振)
#define RTC_CLOCK_SOURCE_LXTAL
//#define RTC_CLOCK_SOURCE_IRC32K

#define BKP_VALUE    0x32F0

// ==============================
// RTC 基础功能
// ==============================
void rtc_init_all(void);
void rtc_pre_config_only(void);
void rtc_show_time(void);

// 从AT+CCLK?应答解析时间并设置RTC
// 输入格式: "26/04/17,09:37:11+32" (自动+8小时)
// 返回: 1=成功, 0=解析失败
int rtc_set_from_cclk(const char *cclk_str);

// ==============================
// RTC 闹钟唤醒功能（低功耗定时唤醒）
// ==============================
void rtc_alarm_wakeup_init(void);
void rtc_set_next_alarm(uint16_t interval_minutes);
void rtc_set_next_alarm_sec(uint8_t seconds);
void rtc_get_time_str(char *buf, uint8_t buf_size);

// ==============================
// 当前日期时间结构体 (十进制值, 供定时/日期逻辑复用)
// ==============================
typedef struct {
    uint8_t year;    /* 0-99 (如 26 = 2026) */
    uint8_t month;   /* 1-12 */
    uint8_t day;     /* 1-31 */
    uint8_t hour;    /* 0-23 */
    uint8_t minute;  /* 0-59 */
    uint8_t second;  /* 0-59 */
} RtcDateTime_t;

// 获取当前RTC日期时间 (BCD转十进制)
void rtc_get_datetime(RtcDateTime_t *dt);

// 获取当前RTC时间的HHMMSS整数值 (如 13:29:00 → 132900)
uint32_t rtc_get_hhmmss(void);

// 设置RTC闹钟到绝对时间 (HHMMSS格式, 如 134700 = 13:47:00)
// 返回距离目标时间的秒数 (0=已过/立即触发)
uint32_t rtc_set_alarm_at_hhmmss(uint32_t target_hhmmss);

// RTC闹钟唤醒标志（中断中置1）
extern volatile uint8_t rtc_alarm_flag;

// BCD <-> DEC 工具
uint8_t bcd2dec(uint8_t bcd);
uint8_t dec2bcd(uint8_t dec);

#endif










