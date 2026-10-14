/**
 * @file config_policy.c
 * @brief 配置策略/纯逻辑: 周期转换、异常关阀策略、窗口/采集决策
 * @details 无EEPROM/串口依赖, 仅读取运行时配置结构体与RTC时间
 */
#include "config_policy.h"
#include "eeprom_config.h"
#include "calib_mode.h"
#include "tim.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/* ==================== dot pattern (24-hour bitmap) helpers ==================== */
void window_dottime_clear(uint8_t *mask)
{
    if(!mask) return;
    memset(mask, 0, 3);
}

void window_dottime_set(uint8_t *mask, uint8_t hour)
{
    if(!mask || hour > 23) return;
    mask[hour >> 3] |= (uint8_t)(1u << (hour & 7));
}

uint8_t window_dottime_hit(const uint8_t *mask, uint8_t hour)
{
    if(!mask || hour > 23) return 0;
    return (mask[hour >> 3] & (uint8_t)(1u << (hour & 7))) ? 1 : 0;
}

uint8_t window_dottime_first(const uint8_t *mask)
{
    uint8_t i;
    if(!mask) return 0xFF;
    for(i = 0; i < 24; i++)
        if(window_dottime_hit(mask, i)) return i;
    return 0xFF;
}

void window_dottime_from_str(const char *str, uint8_t *mask)
{
    char tmp[96];
    char *tok;
    if(!str || !mask) return;
    window_dottime_clear(mask);
    strncpy(tmp, str, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = 0;
    tok = strtok(tmp, ", ");
    while(tok)
    {
        int v = atoi(tok);
        if(v >= 0 && v <= 23) window_dottime_set(mask, (uint8_t)v);
        tok = strtok(NULL, ", ");
    }
}

void window_dottime_to_str(const uint8_t *mask, char *buf, uint16_t buf_size)
{
    uint8_t i;
    uint8_t first = 1;
    if(!buf || buf_size == 0) return;
    buf[0] = 0;
    if(!mask) return;
    for(i = 0; i < 24; i++)
    {
        if(window_dottime_hit(mask, i))
        {
            char hh[8];
            int n = snprintf(hh, sizeof(hh), "%s%02u", first ? "" : ",", (unsigned)i);
            if((int)strlen(buf) + n < (int)buf_size) strcat(buf, hh);
            first = 0;
        }
    }
}

// ==================== 周期类型工具函数 ====================
CycleType_t cycle_type_from_str(const char *str)
{
    if(strcmp(str, "sec") == 0)    return CYCLE_SEC;
    if(strcmp(str, "minute") == 0) return CYCLE_MINUTE;
    if(strcmp(str, "hour") == 0)   return CYCLE_HOUR;
    if(strcmp(str, "day") == 0)    return CYCLE_DAY;
    if(strcmp(str, "week") == 0)   return CYCLE_WEEK;
    return CYCLE_MINUTE;  // 默认
}

const char* cycle_type_to_str(CycleType_t type)
{
    switch(type)
    {
        case CYCLE_SEC:    return "sec";
        case CYCLE_MINUTE: return "minute";
        case CYCLE_HOUR:   return "hour";
        case CYCLE_DAY:    return "day";
        case CYCLE_WEEK:   return "week";
        default:           return "minute";
    }
}

uint32_t cycle_to_seconds(const CycleConfig_t *cfg)
{
    uint32_t base = (cfg->once > 0) ? cfg->once : 1;
    switch(cfg->type)
    {
        case CYCLE_SEC:    return base;
        case CYCLE_MINUTE: return base * 60;
        case CYCLE_HOUR:   return base * 3600;
        case CYCLE_DAY:    return base * 86400;
        case CYCLE_WEEK:   return base * 604800;
        default:           return base * 60;
    }
}

/* 单位对应的 once 上限: 保证 once*单位 <= 65535 分钟 (g_interval.*_min 为 uint16),
 * 同时避免 cycle_to_seconds 的 uint32 乘法回绕 (week 原上限约 7101)。 */
uint32_t cycle_max_once(CycleType_t type)
{
    switch(type)
    {
        case CYCLE_SEC:    return 65535u;   /* 65535 s  = 18.2 h   */
        case CYCLE_MINUTE: return 65535u;   /* 65535 min= 45.5 天  */
        case CYCLE_HOUR:   return 1092u;    /* 1092 h   = 65520 min */
        case CYCLE_DAY:    return 45u;      /* 45 天    = 64800 min */
        case CYCLE_WEEK:   return 6u;       /* 6 周     = 60480 min */
        default:           return 65535u;
    }
}

/* 周期越界钳制; 返回 1=发生过修正(调用方可打印日志) */
uint8_t cycle_clamp(CycleConfig_t *cfg)
{
    uint32_t max_once;

    if(cfg == NULL) return 0;
    max_once = cycle_max_once(cfg->type);

    if(cfg->once == 0u)
    {
        cfg->once = 1u;
        return 1u;
    }
    if((uint32_t)cfg->once > max_once)
    {
        cfg->once = (uint16_t)max_once;
        return 1u;
    }
    return 0u;
}

/* ==================== 绝对时间轴 (跨天周期调度) ====================
 * 以 1999-12-27(周一) 为第 0 天, 使:
 *   1 day  → 每天 00:00 (与旧"按当天取模"行为一致)
 *   N day  → 每 N 天 00:00
 *   1 week → 每周一 00:00
 * RTC 年份仅 00~99 (2000~2099), 闰年按 %4 判定 (2100 不在可表示范围内) */
#define SCHED_EPOCH_DOW_OFFSET   5u    /* 2000-01-01 为周六, +5 天对齐到周一 */

static const uint16_t s_sched_mdays[12] =
    {0u, 31u, 59u, 90u, 120u, 151u, 181u, 212u, 243u, 273u, 304u, 334u};

/* 自 1999-12-27(周一) 起的天数 */
uint32_t sched_day_index(uint8_t year, uint8_t month, uint8_t day)
{
    uint32_t days = 0;
    uint8_t y;

    if(month < 1u)  month = 1u;
    if(month > 12u) month = 12u;
    if(day < 1u)    day = 1u;
    if(day > 31u)   day = 31u;
    if(year > 99u)  year = 99u;     /* RTC 仅表示 00~99; 野值会让累加越界致 abs_sec 溢出 */

    for(y = 0; y < year; y++)
        days += ((y % 4u) == 0u) ? 366u : 365u;        /* 2000 ~ (2000+year-1) */
    days += s_sched_mdays[month - 1u];
    if(month > 2u && ((year % 4u) == 0u))
        days += 1u;                                     /* 当年闰年的 2/29 */
    days += (uint32_t)(day - 1u);

    return days + SCHED_EPOCH_DOW_OFFSET;
}

uint32_t sched_abs_min_of(uint8_t year, uint8_t month, uint8_t day,
                          uint8_t hour, uint8_t minute)
{
    return sched_day_index(year, month, day) * 1440u
         + (uint32_t)hour * 60u + (uint32_t)minute;
}

uint32_t sched_abs_min(void)
{
    RtcDateTime_t dt;
    rtc_get_datetime(&dt);
    return sched_abs_min_of(dt.year, dt.month, dt.day, dt.hour, dt.minute);
}

uint32_t sched_abs_sec(void)
{
    RtcDateTime_t dt;
    rtc_get_datetime(&dt);
    return sched_day_index(dt.year, dt.month, dt.day) * 86400u
         + ((uint32_t)dt.hour * 60u + (uint32_t)dt.minute) * 60u + (uint32_t)dt.second;
}

// ==================== 异常关阀工具函数 ====================
AbnormalClose_t abnormal_from_str(const char *str)
{
    /* 平台 pv* 协议; 旧值/未知值一律按关闭处理 */
    if(strcmp(str, "pvhigh") == 0)             return ABNORMAL_PV_HIGH;
    if(strcmp(str, "pvlow") == 0)              return ABNORMAL_PV_LOW;
    if(strcmp(str, "pvmutation") == 0)         return ABNORMAL_PV_MUTATION;
    if(strcmp(str, "pvhighorlow") == 0)        return ABNORMAL_PV_HIGH_OR_LOW;
    if(strcmp(str, "pvhighormutation") == 0)   return ABNORMAL_PV_HIGH_OR_MUTATION;
    if(strcmp(str, "pvlowormutation") == 0)    return ABNORMAL_PV_LOW_OR_MUTATION;
    if(strcmp(str, "all") == 0)                return ABNORMAL_PV_ALL;
    return ABNORMAL_NONE;
}

const char* abnormal_to_str(AbnormalClose_t ac)
{
    switch(ac)
    {
        case ABNORMAL_PV_HIGH:             return "pvhigh";
        case ABNORMAL_PV_LOW:              return "pvlow";
        case ABNORMAL_PV_MUTATION:         return "pvmutation";
        case ABNORMAL_PV_HIGH_OR_LOW:      return "pvhighorlow";
        case ABNORMAL_PV_HIGH_OR_MUTATION: return "pvhighormutation";
        case ABNORMAL_PV_LOW_OR_MUTATION:  return "pvlowormutation";
        case ABNORMAL_PV_ALL:              return "all";
        default:                           return "none";
    }
}

int abnormal_should_close_valve(AbnormalClose_t policy, ThresholdResult_t result)
{
    // 电压报警 VPROTECT/VOLT_HIGH/VOLT_LOW: 只上报不动阀
    // pv* 协议仅匹配极值档(超高/超低)与突变; 普通档/温度不参与关阀
    int is_vhigh = (result == THRESHOLD_VERY_HIGH);
    int is_vlow  = (result == THRESHOLD_VERY_LOW);
    int is_mut   = (result == THRESHOLD_MUTATION);

    switch(policy)
    {
        case ABNORMAL_PV_HIGH:             return is_vhigh;
        case ABNORMAL_PV_LOW:              return is_vlow;
        case ABNORMAL_PV_MUTATION:         return is_mut;
        case ABNORMAL_PV_HIGH_OR_LOW:      return (is_vhigh || is_vlow);
        case ABNORMAL_PV_HIGH_OR_MUTATION: return (is_vhigh || is_mut);
        case ABNORMAL_PV_LOW_OR_MUTATION:  return (is_vlow || is_mut);
        case ABNORMAL_PV_ALL:              return (is_vhigh || is_vlow || is_mut);
        default:                           return 0;
    }
}

// ==================== 判断当前RTC时间是否在活动窗口 ====================
/* 按指定时刻(HHMMSS)判断是否在活动窗口内 (dot模式视为全天有效) */
int config_is_active_window_at(uint32_t hhmmss)
{
    uint32_t start, stop;

    // dot模式不使用窗口概念, 由do_collect按整点判断
    if(strcmp(g_window.pattern, "dot") == 0) return 1;

    // long/short: 基于HHMMSS窗口判断
    start = (uint32_t)atol(g_window.start);
    stop  = (uint32_t)atol(g_window.stop);

    // 未配置窗口时(start==0 && stop==0), 全天有效
    if(start == 0 && stop == 0) return 1;

    // stop 加 2 分钟容差(HHMMSS +200), 覆盖 RTC 闹钟偏移导致的边界踩空
    {
        uint32_t stop_m = stop % 10000;       // MMSS 部分
        uint32_t stop_h = stop / 10000;       // HH 部分
        stop_m += 200;                        // +2分钟
        if((stop_m / 100) % 100 >= 60)        // MM>=60 进位
        { stop_m -= 6000; stop_h++; }
        if(stop_h >= 24) stop_h -= 24;
        stop = stop_h * 10000 + stop_m;
    }

    if(start < stop)
        return (hhmmss >= start && hhmmss <= stop);
    else
        return (hhmmss >= start || hhmmss <= stop);  // 跨午夜
}

int config_is_active_window(void)
{
    return config_is_active_window_at(rtc_get_hhmmss());
}

// ==================== 智能决策当前采集周期是否需要采样电池电压 ====================
// 仅在首次、尾次（上报点）或低电压保护模式下开启采样，其余中间采集点保持关闭以最大化省电。extern uint8_t voltage_low_power;
uint8_t config_need_adc_sampling(void)
{
    // 1. 低电压保护状态下：必须每次都采样，以便及时检测电池更换和电压恢复，退出低电压状态
if(voltage_low_power)
        return 1;

    // 2. dot（定点模式）：一天上报一次，首次和尾次检查
if(strcmp(g_window.pattern, "dot") == 0)
    {
        uint32_t now_hhmmss = rtc_get_hhmmss();
        uint8_t cur_hour = (uint8_t)(now_hhmmss / 10000);
        uint8_t cur_min = (uint8_t)((now_hhmmss / 100) % 100);

        uint8_t is_last_col = (window_dottime_hit(g_window.dottime_mask, cur_hour) && cur_min == 0);
        uint16_t eff_col = (calib_is_active() ? CALIB_SAMPLE_MIN : g_interval.collect_interval_min);
        uint8_t is_first_col = (window_dottime_hit(g_window.dottime_mask, cur_hour) && cur_min == eff_col);

        if (is_last_col || is_first_col)
            return 1;

        return 0;
    }

    // 3. long（长连接）和 short（短连接）模式：按分钟整网格计算首次和尾次
if(strcmp(g_window.pattern, "long") == 0 || strcmp(g_window.pattern, "short") == 0)
    {
        uint32_t now_hhmmss = rtc_get_hhmmss();
        uint16_t cur_total_min = (uint16_t)(now_hhmmss / 10000) * 60
                               + (uint16_t)((now_hhmmss / 100) % 100);

        uint16_t upload_interval = g_interval.upload_interval_min;
        uint16_t collect_interval = (calib_is_active() ? CALIB_SAMPLE_MIN : g_interval.collect_interval_min);

        if (upload_interval == 0 || collect_interval == 0)
            return 1;

        uint16_t rem = cur_total_min % upload_interval;
        uint8_t is_last_col = (rem == 0);
        uint8_t is_first_col = (rem == (collect_interval % upload_interval));

        if (is_last_col || is_first_col)
            return 1;

        return 0;
    }

    return 1; // 其它模式（安全兜底）：都采样
}
