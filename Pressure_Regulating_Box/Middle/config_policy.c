/**
 * @file config_policy.c
 * @brief 配置策略/纯逻辑: 周期转换、异常关阀策略、窗口/采集决策
 * @details 无EEPROM/串口依赖, 仅读取运行时配置结构体与RTC时间
 */
#include "config_policy.h"
#include "eeprom_config.h"
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

// ==================== 异常关阀工具函数 ====================
AbnormalClose_t abnormal_from_str(const char *str)
{
    if(strcmp(str, "high") == 0)           return ABNORMAL_HIGH;
    if(strcmp(str, "low") == 0)            return ABNORMAL_LOW;
    if(strcmp(str, "mutation") == 0)       return ABNORMAL_MUTATION;
    if(strcmp(str, "highorlow") == 0)      return ABNORMAL_HIGH_OR_LOW;
    if(strcmp(str, "highormutation") == 0) return ABNORMAL_HIGH_OR_MUTATION;
    if(strcmp(str, "lowormutation") == 0)  return ABNORMAL_LOW_OR_MUTATION;
    if(strcmp(str, "all") == 0)            return ABNORMAL_ALL;
    if(strcmp(str, "temph") == 0)          return ABNORMAL_TEMP_HIGH;
    if(strcmp(str, "templ") == 0)          return ABNORMAL_TEMP_LOW;
    if(strcmp(str, "jgclose") == 0)        return ABNORMAL_JGCLOSE;
    /* 注: 电压报警 (voltage_high/low) 不参与策略, 只上报不动阀 */
    return ABNORMAL_NONE;
}

const char* abnormal_to_str(AbnormalClose_t ac)
{
    switch(ac)
    {
        case ABNORMAL_HIGH:             return "high";
        case ABNORMAL_LOW:              return "low";
        case ABNORMAL_MUTATION:         return "mutation";
        case ABNORMAL_HIGH_OR_LOW:      return "highorlow";
        case ABNORMAL_HIGH_OR_MUTATION: return "highormutation";
        case ABNORMAL_LOW_OR_MUTATION:  return "lowormutation";
        case ABNORMAL_ALL:              return "all";
        case ABNORMAL_TEMP_HIGH:        return "temph";
        case ABNORMAL_TEMP_LOW:         return "templ";
        case ABNORMAL_JGCLOSE:          return "jgclose";
        default:                        return "none";
    }
}

int abnormal_should_close_valve(AbnormalClose_t policy, ThresholdResult_t result)
{
    // 电压报警 VPROTECT/VOLT_HIGH/VOLT_LOW: 只上报不动阀
    // 单档/组合档严格匹配普通档; 仅 ABNORMAL_ALL 额外包含压力超高/超低
    int is_high  = (result == THRESHOLD_OVER_LIMIT);
    int is_low   = (result == THRESHOLD_UNDER_LIMIT);
    int is_mut   = (result == THRESHOLD_MUTATION);
    int is_vhigh = (result == THRESHOLD_VERY_HIGH);
    int is_vlow  = (result == THRESHOLD_VERY_LOW);
    int is_temph = (result == THRESHOLD_TEMP_HIGH);
    int is_templ = (result == THRESHOLD_TEMP_LOW);

    switch(policy)
    {
        case ABNORMAL_HIGH:             return is_high;
        case ABNORMAL_LOW:              return is_low;
        case ABNORMAL_MUTATION:         return is_mut;
        case ABNORMAL_HIGH_OR_LOW:      return (is_high || is_low);
        case ABNORMAL_HIGH_OR_MUTATION: return (is_high || is_mut);
        case ABNORMAL_LOW_OR_MUTATION:  return (is_low || is_mut);
        case ABNORMAL_ALL:              return (is_high || is_low || is_mut || is_vhigh || is_vlow);
        case ABNORMAL_TEMP_HIGH:        return is_temph;
        case ABNORMAL_TEMP_LOW:         return is_templ;
        default:                        return 0;
    }
}

// ==================== 判断当前RTC时间是否在活动窗口 ====================
int config_is_active_window(void)
{
    uint32_t now, start, stop;

    // dot模式不使用窗口概念, 由do_collect按整点判断
if(strcmp(g_window.pattern, "dot") == 0) return 1;

    // long/short: 基于HHMMSS窗口判断
now   = rtc_get_hhmmss();
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
        return (now >= start && now <= stop);
    else
        return (now >= start || now <= stop);  // 跨午夜
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
        uint8_t is_first_col = (window_dottime_hit(g_window.dottime_mask, cur_hour) && cur_min == g_interval.collect_interval_min);

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
        uint16_t collect_interval = g_interval.collect_interval_min;

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
