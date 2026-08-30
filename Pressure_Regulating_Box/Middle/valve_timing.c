#include "valve_timing.h"
#include <string.h>

/* 取字符串第 pos 起的两位十进制数 (0-99); 调用方保证 10 位数字 */
static uint8_t vt_field(const char *t, uint8_t pos)
{
    return (uint8_t)((t[pos] - '0') * 10 + (t[pos + 1] - '0'));
}

uint8_t valve_timing_time_valid(const char *t)
{
    uint8_t i;
    uint8_t month, day, hour, minute;

    if(t == NULL) return 0;

    /* 必须为 10 位数字 */
    for(i = 0; i < 10; i++)
    {
        if(t[i] < '0' || t[i] > '9') return 0;
    }

    /* 范围校验 (00 通配仅允许年/月/日, 时分必须真实) */
    month  = vt_field(t, 2);
    day    = vt_field(t, 4);
    hour   = vt_field(t, 6);
    minute = vt_field(t, 8);

    if(month != 0 && (month < 1 || month > 12)) return 0;
    if(day   != 0 && (day   < 1 || day   > 31)) return 0;
    if(hour  > 23) return 0;
    if(minute > 59) return 0;

    return 1;
}

uint8_t valve_timing_time_match(const char *t, const RtcDateTime_t *now, int32_t tol_sec)
{
    int32_t tgt_sec, now_sec, diff;
    uint8_t y, mo, d, h, mi;

    if(t == NULL || now == NULL) return 0;
    if(!valve_timing_time_valid(t)) return 0;

    y  = vt_field(t, 0);
    mo = vt_field(t, 2);
    d  = vt_field(t, 4);
    h  = vt_field(t, 6);
    mi = vt_field(t, 8);

    /* 日期通配: 非00段必须等于当前值 */
    if(y  != 0 && y  != now->year)   return 0;
    if(mo != 0 && mo != now->month)  return 0;
    if(d  != 0 && d  != now->day)    return 0;

    /* 时分容差: 目标=目标分钟第0秒, 当前=当日秒, diff∈[0,tol_sec] 命中 */
    tgt_sec = ((int32_t)h * 60 + mi) * 60;
    now_sec = ((int32_t)now->hour * 60 + now->minute) * 60 + now->second;

    diff = now_sec - tgt_sec;
    if(diff >= 0 && diff <= tol_sec) return 1;

    return 0;
}