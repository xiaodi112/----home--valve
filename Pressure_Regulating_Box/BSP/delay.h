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

// ==================== 软件看门狗 ====================
// 仅在唤醒后的活跃处理阶段使能; 进入深休眠前必须 swdt_disable().
#define SWDT_TIMEOUT_MS  180000U   // 180s: 覆盖最长合法活跃段(4G冷启+连接+上传+下行等待)
void swdt_enable(uint32_t timeout_ms);   // 使能并复位计数 (timeout_ms=0 等同 disable)
void swdt_disable(void);                 // 关闭并清零 (深休眠前调用)
void swdt_feed(void);                    // 喂狗: 复位计数


void my_systick_config(void);
/*ms*/
void my_systick_delay_ms(uint32_t counts);
/*us*/
void my_systick_delay_us(uint32_t counts);

#endif