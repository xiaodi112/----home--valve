#ifndef TREA_TIME_H
#define TREA_TIME_H

#include <stdint.h>
#include <stdio.h>

// 时间结构体（你可以直接用）
typedef struct {
    uint16_t year;    // 年 20xx
    uint8_t  month;   // 月
    uint8_t  day;     // 日
    uint8_t  hour;    // 时
    uint8_t  min;     // 分
    uint8_t  sec;     // 秒
    uint8_t  week;    // 星期
} SysTime_TypeDef;

void timer1_delay_init(void);
void trea_time_isr(void);
uint32_t trea_millis(void);
void trea_delay_ms(uint32_t ms);
void delay_us(uint32_t us);


void my_systick_config(void);
/*ms*/
void my_systick_delay_ms(uint32_t counts);
/*us*/
void my_systick_delay_us(uint32_t counts);

#endif