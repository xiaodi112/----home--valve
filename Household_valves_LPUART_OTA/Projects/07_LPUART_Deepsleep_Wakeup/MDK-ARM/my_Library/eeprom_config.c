/**
 * @file eeprom_config.c
 * @brief AT24C16 EEPROM 配置存储 + Debug串口配置命令解析
 *
 * 支持的Debug串口命令:
 *   间隔配置: {collect:0.5,upload:1.0}
 *   MQTT配置: {clientId:xxx,username:xxx,passwd:xxx,port:1883}
 */

#include "eeprom_config.h"
#include "AT24c16.h"
#include "usart.h"
#include "delay.h"
#include "app_config.h"
#include "threshold.h"
#include "tim.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// ==================== 全局运行时配置 ====================
AppInterval_t              g_interval;
MqttConfig_t               g_mqtt;
WindowConfig_t             g_window;
CycleConfig_t              g_collect_cycle;
CycleConfig_t              g_report_cycle;
char                       g_abnormal_close[16];
uint8_t                    g_valve_last_order;
uint8_t                    g_valve_alarm_closed = 0;    // 报警自动关阀标志 (EEPROM持久化, OTA重启后可恢复)
uint8_t                    g_valve_pre_alarm_order = 0; // 关阀前阀门状态 (EEPROM持久化)
uint8_t                    g_real_enabled   = 0;   // 实时采集开关 (默认关闭)
uint8_t                    g_real_frequency = 5;   // 实时采集频率 (默认5秒)
uint16_t                   g_keepalive_sec  = APP_MQTT_KEEPALIVE;
TempThresholdConfig_t      g_tthreshold;            // 温度阈值
ValveDelayConfig_t         g_vdelay;                // 阀门动作延时
VoltagePowerConfig_t       g_spupower;              // 电压检测
PressureThresholdConfig_t  g_pthreshold;            // 压力阈值

// ==================== 模块共享 配置命令行缓冲 ====================
// config_enter_setup_mode (上电配置阶段) 与 config_check_cmd (运行期主循环) 互斥调用
// 不并发, 不嵌套, 可共享同一份 512B 缓冲 节省 512B RAM
static char s_cfg_line_buf[512];

// ==================== 内部工具 ====================

// 从EEPROM读float
static float ee_read_float(uint16_t addr)
{
    float val = 0;
    AT24C16_Read(addr, (uint8_t *)&val, sizeof(float));
    return val;
}

// 写float到EEPROM
static void ee_write_float(uint16_t addr, float val)
{
    AT24C16_Write(addr, (const uint8_t *)&val, sizeof(float));
}

// 从EEPROM读字符串
static void ee_read_string(uint16_t addr, char *buf, uint16_t max_len)
{
    AT24C16_Read(addr, (uint8_t *)buf, max_len);
    buf[max_len - 1] = '\0';  // 确保终止
}

// 写字符串到EEPROM（含终止符）
static void ee_write_string(uint16_t addr, const char *str, uint16_t max_len)
{
    uint16_t len = (uint16_t)strlen(str) + 1;
    if(len > max_len) len = max_len;
    AT24C16_Write(addr, (const uint8_t *)str, len);
}

// 从字符串中提取key:value对的value
// 输入: src = "clientId:abc,username:def", key = "clientId"
// 输出: out = "abc", 返回1成功
// 注: 必须带单词边界 — 前一字符必须是 { , : 或字符串起始,
//     防止 "pmax" 被 "ppmax:5000" 内部子串误匹配
//     (同样规避 high/vhigh, low/vlow, pmin/ppmin 等前缀冲突)
static int extract_field(const char *src, const char *key, char *out, uint16_t out_size)
{
    char search_key[32];
    snprintf(search_key, sizeof(search_key), "%s:", key);
    size_t key_len = strlen(search_key);

    const char *p = src;
    while((p = strstr(p, search_key)) != NULL)
    {
        // 单词边界检查: p前面的字符必须是分隔符或字符串起点
        if(p == src || *(p - 1) == '{' || *(p - 1) == ',' || *(p - 1) == ':' || *(p - 1) == ' ')
        {
            break;  // 命中真正的字段
        }
        p += 1;  // 跳过当前位置继续搜索
    }
    if(!p) return 0;

    p += key_len;
    uint16_t i = 0;
    while(*p && *p != ',' && *p != '}' && i < out_size - 1)
    {
        out[i++] = *p++;
    }
    out[i] = '\0';
    return (i > 0) ? 1 : 0;
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
        default:                        return "none";
    }
}

int abnormal_should_close_valve(AbnormalClose_t policy, ThresholdResult_t result)
{
    // 电压保护始终触发关阀 (VLP 是电池保护, 与策略无关)
    if(result == THRESHOLD_VPROTECT) return 1;

    // 电压报警 VOLT_HIGH/LOW: 只上报不动阀 (架构设计)
    // 只有压力和温度报警需要自动关阀
    int is_high  = (result == THRESHOLD_OVER_LIMIT || result == THRESHOLD_VERY_HIGH);
    int is_low   = (result == THRESHOLD_UNDER_LIMIT || result == THRESHOLD_VERY_LOW);
    int is_mut   = (result == THRESHOLD_MUTATION);
    int is_temph = (result == THRESHOLD_TEMP_HIGH || result == THRESHOLD_TEMP_VERY_HIGH);
    int is_templ = (result == THRESHOLD_TEMP_LOW  || result == THRESHOLD_TEMP_VERY_LOW);

    switch(policy)
    {
        case ABNORMAL_HIGH:             return is_high;
        case ABNORMAL_LOW:              return is_low;
        case ABNORMAL_MUTATION:         return is_mut;
        case ABNORMAL_HIGH_OR_LOW:      return (is_high || is_low);
        case ABNORMAL_HIGH_OR_MUTATION: return (is_high || is_mut);
        case ABNORMAL_LOW_OR_MUTATION:  return (is_low || is_mut);
        case ABNORMAL_ALL:              return (is_high || is_low || is_mut);
        case ABNORMAL_TEMP_HIGH:        return is_temph;
        case ABNORMAL_TEMP_LOW:         return is_templ;
        default:                        return 0;
    }
}

// ==================== 重新计算间隔 ====================
void config_recalc_interval(void)
{
    if(g_interval.collect_interval_hours < 0.01f)
        g_interval.collect_interval_hours = 0.01f;
    if(g_interval.upload_interval_hours < g_interval.collect_interval_hours)
        g_interval.upload_interval_hours = g_interval.collect_interval_hours;

    g_interval.collect_interval_min = (uint16_t)(g_interval.collect_interval_hours * 60.0f);
    g_interval.upload_interval_min  = (uint16_t)(g_interval.upload_interval_hours * 60.0f);

    if(g_interval.collect_interval_min < 1)
        g_interval.collect_interval_min = 1;
    if(g_interval.upload_interval_min < g_interval.collect_interval_min)
        g_interval.upload_interval_min = g_interval.collect_interval_min;

    g_interval.upload_every_n = g_interval.upload_interval_min / g_interval.collect_interval_min;
    if(g_interval.upload_every_n < 1)
        g_interval.upload_every_n = 1;
}

// ==================== 从EEPROM加载全部配置 ====================
void config_load_all(void)
{
    uint8_t magic = 0;

    // --- 加载间隔配置 ---
    AT24C16_Read(EE_ADDR_INTERVAL_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_INTERVAL)
    {
        g_interval.collect_interval_hours = ee_read_float(EE_ADDR_COLLECT_HOURS);
        g_interval.upload_interval_hours  = ee_read_float(EE_ADDR_UPLOAD_HOURS);
        printf_debug("[CFG] EEPROM interval: collect=%.2fh upload=%.2fh\r\n",
                     g_interval.collect_interval_hours, g_interval.upload_interval_hours);
    }
    else
    {
        g_interval.collect_interval_hours = APP_DEFAULT_COLLECT_HOURS;
        g_interval.upload_interval_hours  = APP_DEFAULT_UPLOAD_HOURS;
        printf_debug("[CFG] No interval, using defaults: collect=%.3fh upload=%.3fh\r\n",
                     g_interval.collect_interval_hours, g_interval.upload_interval_hours);
        config_save_interval();
    }
    config_recalc_interval();

    // --- 加载MQTT配置 ---
    AT24C16_Read(EE_ADDR_MQTT_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_MQTT)
    {
        ee_read_string(EE_ADDR_MQTT_CLIENT_ID, g_mqtt.client_id, MQTT_FIELD_MAX);
        ee_read_string(EE_ADDR_MQTT_USERNAME,  g_mqtt.username,  MQTT_FIELD_MAX);
        ee_read_string(EE_ADDR_MQTT_PASSWORD,  g_mqtt.password,  MQTT_FIELD_MAX);
        ee_read_string(EE_ADDR_MQTT_SERVER,    g_mqtt.server,    MQTT_FIELD_MAX);
        ee_read_string(EE_ADDR_MQTT_TOPIC_UP,  g_mqtt.topic_up,  MQTT_TOPIC_MAX);
        AT24C16_Read(EE_ADDR_MQTT_PORT, (uint8_t *)&g_mqtt.port, 2);
        ee_read_string(EE_ADDR_MQTT_SERVER2,   g_mqtt.server2,   MQTT_FIELD_MAX);
        AT24C16_Read(EE_ADDR_MQTT_PORT2, (uint8_t *)&g_mqtt.port2, 2);
        if(g_mqtt.port2 == 0 || g_mqtt.port2 == 0xFFFF) g_mqtt.port2 = g_mqtt.port;
        printf_debug("[CFG] EEPROM MQTT loaded: %s@%s:%d (s2=%s:%d)\r\n",
                     g_mqtt.client_id, g_mqtt.server, g_mqtt.port,
                     g_mqtt.server2[0] ? g_mqtt.server2 : "(none)", g_mqtt.port2);
    }
    else
    {
        // 使用编译时默认值 (MQTT 需用户下发 clientId/server, 故不 save)
        strncpy(g_mqtt.client_id, APP_DEFAULT_MQTT_CLIENT_ID, MQTT_FIELD_MAX - 1);
        strncpy(g_mqtt.username,  APP_DEFAULT_MQTT_USER,      MQTT_FIELD_MAX - 1);
        strncpy(g_mqtt.password,  APP_DEFAULT_MQTT_PASS,      MQTT_FIELD_MAX - 1);
        strncpy(g_mqtt.server,    APP_DEFAULT_MQTT_SERVER,    MQTT_FIELD_MAX - 1);
        strncpy(g_mqtt.topic_up,  APP_DEFAULT_MQTT_TOPIC,     MQTT_TOPIC_MAX - 1);
        g_mqtt.port = APP_DEFAULT_MQTT_PORT;
        strncpy(g_mqtt.server2,   APP_DEFAULT_MQTT_SERVER2,   MQTT_FIELD_MAX - 1);
        g_mqtt.port2 = APP_DEFAULT_MQTT_PORT2;
        printf_debug("[CFG] No MQTT, using defaults: %s@%s:%d (s2=%s:%d) [waiting setup]\r\n",
                     g_mqtt.client_id, g_mqtt.server, g_mqtt.port,
                     g_mqtt.server2[0] ? g_mqtt.server2 : "(none)", g_mqtt.port2);
    }

    // --- 加载压力阈值(vpthreshold) + 心跳配置 ---
    AT24C16_Read(EE_ADDR_VPTH_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_VPTH)
    {
        AT24C16_Read(EE_ADDR_VPTH_ENABLED, &g_pthreshold.enabled, 1);
        g_pthreshold.ppmax    = ee_read_float(EE_ADDR_VPTH_PPMAX);
        g_pthreshold.pmax     = ee_read_float(EE_ADDR_VPTH_PMAX);
        g_pthreshold.ppmin    = ee_read_float(EE_ADDR_VPTH_PPMIN);
        g_pthreshold.pmin     = ee_read_float(EE_ADDR_VPTH_PMIN);
        g_pthreshold.mutation = ee_read_float(EE_ADDR_VPTH_MUTATION);
        g_pthreshold.shock    = ee_read_float(EE_ADDR_VPTH_SHOCK);
        AT24C16_Read(EE_ADDR_KEEPALIVE_SEC, (uint8_t *)&g_keepalive_sec, 2);
        if(g_pthreshold.enabled > 1) g_pthreshold.enabled = 0;
        if(g_keepalive_sec == 0 || g_keepalive_sec == 0xFFFF) g_keepalive_sec = APP_MQTT_KEEPALIVE;
        printf_debug("[CFG] VPTH: en=%d ppmax=%.1f pmax=%.1f ppmin=%.1f pmin=%.1f mut=%.1f shock=%.1f ka=%d\r\n",
                     g_pthreshold.enabled, g_pthreshold.ppmax, g_pthreshold.pmax,
                     g_pthreshold.ppmin, g_pthreshold.pmin,
                     g_pthreshold.mutation, g_pthreshold.shock, g_keepalive_sec);
    }
    else
    {
        // 首次启动: 清零默认 (switch=0, 阈值=0)
        g_pthreshold.enabled  = DEF_VP_SWITCH;
        g_pthreshold.ppmax    = DEF_VP_PPMAX;
        g_pthreshold.pmax     = DEF_VP_PMAX;
        g_pthreshold.ppmin    = DEF_VP_PPMIN;
        g_pthreshold.pmin     = DEF_VP_PMIN;
        g_pthreshold.mutation = DEF_VP_MUTATION;
        g_pthreshold.shock    = DEF_VP_SHOCK;
        g_keepalive_sec       = APP_MQTT_KEEPALIVE;
        printf_debug("[CFG] No VPTH, using defaults: en=%d ppmax=%.1f pmax=%.1f ppmin=%.1f pmin=%.1f mut=%.1f shock=%.1f\r\n",
                     g_pthreshold.enabled, g_pthreshold.ppmax, g_pthreshold.pmax,
                     g_pthreshold.ppmin, g_pthreshold.pmin,
                     g_pthreshold.mutation, g_pthreshold.shock);
        config_save_vpthreshold();
    }

    // --- 加载温度阈值(Tthreshold) ---
    AT24C16_Read(EE_ADDR_TTH_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_TTH)
    {
        AT24C16_Read(EE_ADDR_TTH_ENABLED, &g_tthreshold.enabled, 1);
        g_tthreshold.high  = ee_read_float(EE_ADDR_TTH_HIGH);
        g_tthreshold.low   = ee_read_float(EE_ADDR_TTH_LOW);
        g_tthreshold.vhigh = ee_read_float(EE_ADDR_TTH_VHIGH);
        g_tthreshold.vlow  = ee_read_float(EE_ADDR_TTH_VLOW);
        if(g_tthreshold.enabled > 1) g_tthreshold.enabled = 0;
        printf_debug("[CFG] TTH: en=%d high=%.1f low=%.1f vhigh=%.1f vlow=%.1f\r\n",
                     g_tthreshold.enabled, g_tthreshold.high, g_tthreshold.low,
                     g_tthreshold.vhigh, g_tthreshold.vlow);
    }
    else
    {
        g_tthreshold.enabled = DEF_TT_SWITCH;
        g_tthreshold.high    = DEF_TT_HIGH;
        g_tthreshold.low     = DEF_TT_LOW;
        g_tthreshold.vhigh   = DEF_TT_VHIGH;
        g_tthreshold.vlow    = DEF_TT_VLOW;
        printf_debug("[CFG] No TTH, using defaults: en=%d high=%.1f low=%.1f vhigh=%.1f vlow=%.1f\r\n",
                     g_tthreshold.enabled, g_tthreshold.high, g_tthreshold.low,
                     g_tthreshold.vhigh, g_tthreshold.vlow);
        config_save_tthreshold();
    }

    // --- 加载电压检测(spupower) ---
    AT24C16_Read(EE_ADDR_SPUP_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_SPUP)
    {
        AT24C16_Read(EE_ADDR_SPUP_ENABLED, &g_spupower.enabled, 1);
        g_spupower.high    = ee_read_float(EE_ADDR_SPUP_HIGH);
        g_spupower.low     = ee_read_float(EE_ADDR_SPUP_LOW);
        g_spupower.protect = ee_read_float(EE_ADDR_SPUP_PROTECT);
        if(g_spupower.enabled > 1) g_spupower.enabled = 0;
        printf_debug("[CFG] SPUP: en=%d high=%.2f low=%.2f protect=%.2f\r\n",
                     g_spupower.enabled, g_spupower.high, g_spupower.low, g_spupower.protect);
    }
    else
    {
        g_spupower.enabled = DEF_SP_SWITCH;
        g_spupower.high    = DEF_SP_HIGH;
        g_spupower.low     = DEF_SP_LOW;
        g_spupower.protect = DEF_SP_PROTECT;
        printf_debug("[CFG] No SPUP, using defaults: en=%d high=%.2f low=%.2f protect=%.2f\r\n",
                     g_spupower.enabled, g_spupower.high,
                     g_spupower.low, g_spupower.protect);
        config_save_spupower();
    }

    // --- 加载阀门动作延时(valve_delay) ---
    AT24C16_Read(EE_ADDR_VDLY_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_VDLY)
    {
        AT24C16_Read(EE_ADDR_VDLY_ENABLED, &g_vdelay.enabled, 1);
        AT24C16_Read(EE_ADDR_VDLY_DELAY_MS, (uint8_t *)&g_vdelay.delay_ms, 2);
        if(g_vdelay.enabled > 1) g_vdelay.enabled = 0;
        printf_debug("[CFG] VDLY: en=%d delay_ms=%d\r\n",
                     g_vdelay.enabled, g_vdelay.delay_ms);
    }
    else
    {
        g_vdelay.enabled  = DEF_VD_SWITCH;
        g_vdelay.delay_ms = DEF_VD_TIME_MS;
        printf_debug("[CFG] No VDLY, using defaults: en=%d delay_ms=%d\r\n",
                     g_vdelay.enabled, g_vdelay.delay_ms);
        config_save_vdelay();
    }

    // --- 加载扩展配置 (window/cycle/abnormal/valve) ---
    AT24C16_Read(EE_ADDR_EXT_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_EXT)
    {
        ee_read_string(EE_ADDR_WINDOW_PATTERN, g_window.pattern, 8);
        AT24C16_Read(EE_ADDR_WINDOW_END_RESULT, &g_window.end_result, 1);
        ee_read_string(EE_ADDR_WINDOW_START_STR, g_window.start, 8);
        ee_read_string(EE_ADDR_WINDOW_STOP_STR, g_window.stop, 8);
        AT24C16_Read(EE_ADDR_WINDOW_DOTTIME, &g_window.dottime, 1);
        AT24C16_Read(EE_ADDR_COLLECT_ONCE, (uint8_t *)&g_collect_cycle.once, 2);
        { uint8_t t = 0; AT24C16_Read(EE_ADDR_COLLECT_TYPE, &t, 1); g_collect_cycle.type = (CycleType_t)t; }
        AT24C16_Read(EE_ADDR_REPORT_ONCE, (uint8_t *)&g_report_cycle.once, 2);
        { uint8_t t = 0; AT24C16_Read(EE_ADDR_REPORT_TYPE, &t, 1); g_report_cycle.type = (CycleType_t)t; }
        ee_read_string(EE_ADDR_ABNORMAL_CLOSE, g_abnormal_close, 16);
        AT24C16_Read(EE_ADDR_VALVE_LAST_ORDER, &g_valve_last_order, 1);
        AT24C16_Read(EE_ADDR_REAL_ENABLED, &g_real_enabled, 1);
        AT24C16_Read(EE_ADDR_REAL_FREQUENCY, &g_real_frequency, 1);
        AT24C16_Read(EE_ADDR_VALVE_ALARM_CLOSED,    &g_valve_alarm_closed,    1);
        AT24C16_Read(EE_ADDR_VALVE_PRE_ALARM_ORDER, &g_valve_pre_alarm_order, 1);
        if(g_real_enabled > 1)   g_real_enabled = 0;
        if(g_real_frequency < 1 || g_real_frequency > 10) g_real_frequency = 5;
        // 防护: 未写过的字节是 0xFF, 视为 0
        if(g_valve_alarm_closed    > 1) g_valve_alarm_closed    = 0;
        if(g_valve_pre_alarm_order > 1) g_valve_pre_alarm_order = 0;
        printf_debug("[CFG] EXT: win=%s er=%d start=%s stop=%s dot=%d\r\n",
                     g_window.pattern, g_window.end_result,
                     g_window.start, g_window.stop, g_window.dottime);
        printf_debug("[CFG] EXT: collect=%d*%s report=%d*%s abnormal=%s valve=%d\r\n",
                     g_collect_cycle.once, cycle_type_to_str(g_collect_cycle.type),
                     g_report_cycle.once, cycle_type_to_str(g_report_cycle.type),
                     g_abnormal_close, g_valve_last_order);
        printf_debug("[CFG] EXT: real_enabled=%d real_frequency=%ds\r\n",
                     g_real_enabled, g_real_frequency);
        // ext存在时, 用cycle配置覆盖g_interval的周期(保证两套数据一致)
        g_interval.collect_interval_hours = (float)cycle_to_seconds(&g_collect_cycle) / 3600.0f;
        g_interval.upload_interval_hours  = (float)cycle_to_seconds(&g_report_cycle)  / 3600.0f;
        config_recalc_interval();
    }
    else
    {
        strncpy(g_window.pattern, APP_DEFAULT_WINDOW_PATTERN, 7); g_window.pattern[7] = '\0';
        g_window.end_result  = APP_DEFAULT_WINDOW_END_RESULT;
        strncpy(g_window.start, APP_DEFAULT_WINDOW_START, 7); g_window.start[7] = '\0';
        strncpy(g_window.stop,  APP_DEFAULT_WINDOW_STOP,  7); g_window.stop[7]  = '\0';
        g_window.dottime     = APP_DEFAULT_WINDOW_DOTTIME;
        g_collect_cycle.once = APP_DEFAULT_COLLECT_ONCE;
        g_collect_cycle.type = APP_DEFAULT_COLLECT_TYPE;
        g_report_cycle.once  = APP_DEFAULT_REPORT_ONCE;
        g_report_cycle.type  = APP_DEFAULT_REPORT_TYPE;
        strncpy(g_abnormal_close, APP_DEFAULT_ABNORMAL_CLOSE, 15); g_abnormal_close[15] = '\0';
        g_valve_last_order   = APP_DEFAULT_VALVE_LAST_ORDER;
        g_real_enabled       = APP_DEFAULT_REAL_ENABLED;
        g_real_frequency     = APP_DEFAULT_REAL_FREQUENCY;
        g_valve_alarm_closed = 0;
        g_valve_pre_alarm_order = 0;
        // ext 不存在时同步 g_interval (保证两套数据一致)
        g_interval.collect_interval_hours = (float)cycle_to_seconds(&g_collect_cycle) / 3600.0f;
        g_interval.upload_interval_hours  = (float)cycle_to_seconds(&g_report_cycle)  / 3600.0f;
        config_recalc_interval();
        printf_debug("[CFG] No EXT, using defaults: win=%s er=%d start=%s stop=%s dot=%d collect=%d*%s report=%d*%s abnormal=%s valve=%d real=%d/%ds\r\n",
                     g_window.pattern, g_window.end_result,
                     g_window.start, g_window.stop, g_window.dottime,
                     g_collect_cycle.once, cycle_type_to_str(g_collect_cycle.type),
                     g_report_cycle.once,  cycle_type_to_str(g_report_cycle.type),
                     g_abnormal_close, g_valve_last_order,
                     g_real_enabled, g_real_frequency);
        config_save_ext();
    }
}

// ==================== 保存间隔配置到EEPROM ====================
void config_save_interval(void)
{
    uint8_t magic = EE_MAGIC_INTERVAL;
    AT24C16_WriteByte(EE_ADDR_INTERVAL_MAGIC, magic);
    ee_write_float(EE_ADDR_COLLECT_HOURS, g_interval.collect_interval_hours);
    ee_write_float(EE_ADDR_UPLOAD_HOURS,  g_interval.upload_interval_hours);
    config_recalc_interval();
    printf_debug("[CFG] Interval saved: collect=%.2fh upload=%.2fh\r\n",
                 g_interval.collect_interval_hours, g_interval.upload_interval_hours);
}

// ==================== 保存扩展配置到EEPROM ====================
void config_save_ext(void)
{
    uint8_t magic = EE_MAGIC_EXT;
    AT24C16_WriteByte(EE_ADDR_EXT_MAGIC, magic);
    ee_write_string(EE_ADDR_WINDOW_PATTERN, g_window.pattern, 8);
    AT24C16_WriteByte(EE_ADDR_WINDOW_END_RESULT, g_window.end_result);
    ee_write_string(EE_ADDR_WINDOW_START_STR, g_window.start, 8);
    ee_write_string(EE_ADDR_WINDOW_STOP_STR, g_window.stop, 8);
    AT24C16_WriteByte(EE_ADDR_WINDOW_DOTTIME, g_window.dottime);
    AT24C16_Write(EE_ADDR_COLLECT_ONCE, (const uint8_t *)&g_collect_cycle.once, 2);
    { uint8_t t = (uint8_t)g_collect_cycle.type; AT24C16_WriteByte(EE_ADDR_COLLECT_TYPE, t); }
    AT24C16_Write(EE_ADDR_REPORT_ONCE, (const uint8_t *)&g_report_cycle.once, 2);
    { uint8_t t = (uint8_t)g_report_cycle.type; AT24C16_WriteByte(EE_ADDR_REPORT_TYPE, t); }
    ee_write_string(EE_ADDR_ABNORMAL_CLOSE, g_abnormal_close, 16);
    AT24C16_WriteByte(EE_ADDR_VALVE_LAST_ORDER, g_valve_last_order);
    AT24C16_WriteByte(EE_ADDR_REAL_ENABLED, g_real_enabled);
    AT24C16_WriteByte(EE_ADDR_REAL_FREQUENCY, g_real_frequency);
    AT24C16_WriteByte(EE_ADDR_VALVE_ALARM_CLOSED,    g_valve_alarm_closed);
    AT24C16_WriteByte(EE_ADDR_VALVE_PRE_ALARM_ORDER, g_valve_pre_alarm_order);
    printf_debug("[CFG] EXT saved\r\n");
}

// ==================== 保存MQTT配置到EEPROM ====================
void config_save_mqtt(void)
{
    uint8_t magic = EE_MAGIC_MQTT;
    AT24C16_WriteByte(EE_ADDR_MQTT_MAGIC, magic);
    ee_write_string(EE_ADDR_MQTT_CLIENT_ID, g_mqtt.client_id, MQTT_FIELD_MAX);
    ee_write_string(EE_ADDR_MQTT_USERNAME,  g_mqtt.username,  MQTT_FIELD_MAX);
    ee_write_string(EE_ADDR_MQTT_PASSWORD,  g_mqtt.password,  MQTT_FIELD_MAX);
    ee_write_string(EE_ADDR_MQTT_SERVER,    g_mqtt.server,    MQTT_FIELD_MAX);
    ee_write_string(EE_ADDR_MQTT_TOPIC_UP,  g_mqtt.topic_up,  MQTT_TOPIC_MAX);
    AT24C16_Write(EE_ADDR_MQTT_PORT, (const uint8_t *)&g_mqtt.port, 2);
    ee_write_string(EE_ADDR_MQTT_SERVER2,   g_mqtt.server2,   MQTT_FIELD_MAX);
    AT24C16_Write(EE_ADDR_MQTT_PORT2, (const uint8_t *)&g_mqtt.port2, 2);
    printf_debug("[CFG] MQTT saved: %s@%s:%d (s2=%s:%d)\r\n",
                 g_mqtt.client_id, g_mqtt.server, g_mqtt.port,
                 g_mqtt.server2[0] ? g_mqtt.server2 : "(none)", g_mqtt.port2);
}

// ==================== 保存压力阈值(vpthreshold) + 心跳 ====================
// 写入顺序: 数据 → 最后 magic, 原子提交语义 — 掉电时若 magic 未写则下次启动用默认值
void config_save_vpthreshold(void)
{
    AT24C16_WriteByte(EE_ADDR_VPTH_ENABLED, g_pthreshold.enabled);
    ee_write_float(EE_ADDR_VPTH_PPMAX,    g_pthreshold.ppmax);
    ee_write_float(EE_ADDR_VPTH_PMAX,     g_pthreshold.pmax);
    ee_write_float(EE_ADDR_VPTH_PPMIN,    g_pthreshold.ppmin);
    ee_write_float(EE_ADDR_VPTH_PMIN,     g_pthreshold.pmin);
    ee_write_float(EE_ADDR_VPTH_MUTATION, g_pthreshold.mutation);
    ee_write_float(EE_ADDR_VPTH_SHOCK,    g_pthreshold.shock);
    AT24C16_Write(EE_ADDR_KEEPALIVE_SEC, (const uint8_t *)&g_keepalive_sec, 2);
    AT24C16_WriteByte(EE_ADDR_VPTH_MAGIC, EE_MAGIC_VPTH);  // 最后写, 标记区数据有效
    printf_debug("[CFG] VPTH saved: en=%d ppmax=%.1f pmax=%.1f ppmin=%.1f pmin=%.1f mut=%.1f shock=%.1f ka=%d\r\n",
                 g_pthreshold.enabled, g_pthreshold.ppmax, g_pthreshold.pmax,
                 g_pthreshold.ppmin, g_pthreshold.pmin,
                 g_pthreshold.mutation, g_pthreshold.shock, g_keepalive_sec);
}

// ==================== 保存温度阈值(Tthreshold) ====================
// 写入顺序: 数据 → 最后 magic, 原子提交语义
void config_save_tthreshold(void)
{
    AT24C16_WriteByte(EE_ADDR_TTH_ENABLED, g_tthreshold.enabled);
    ee_write_float(EE_ADDR_TTH_HIGH,  g_tthreshold.high);
    ee_write_float(EE_ADDR_TTH_LOW,   g_tthreshold.low);
    ee_write_float(EE_ADDR_TTH_VHIGH, g_tthreshold.vhigh);
    ee_write_float(EE_ADDR_TTH_VLOW,  g_tthreshold.vlow);
    AT24C16_WriteByte(EE_ADDR_TTH_MAGIC, EE_MAGIC_TTH);  // 最后写
    printf_debug("[CFG] TTH saved: en=%d high=%.1f low=%.1f vhigh=%.1f vlow=%.1f\r\n",
                 g_tthreshold.enabled, g_tthreshold.high, g_tthreshold.low,
                 g_tthreshold.vhigh, g_tthreshold.vlow);
}

// ==================== 保存电压检测(spupower) ====================
// 写入顺序: 数据 → 最后 magic, 原子提交语义
void config_save_spupower(void)
{
    AT24C16_WriteByte(EE_ADDR_SPUP_ENABLED, g_spupower.enabled);
    ee_write_float(EE_ADDR_SPUP_HIGH,    g_spupower.high);
    ee_write_float(EE_ADDR_SPUP_LOW,     g_spupower.low);
    ee_write_float(EE_ADDR_SPUP_PROTECT, g_spupower.protect);
    AT24C16_WriteByte(EE_ADDR_SPUP_MAGIC, EE_MAGIC_SPUP);  // 最后写
    printf_debug("[CFG] SPUP saved: en=%d high=%.2f low=%.2f protect=%.2f\r\n",
                 g_spupower.enabled, g_spupower.high, g_spupower.low, g_spupower.protect);
}

// ==================== 保存阀门动作延时(valve_delay) ====================
// 写入顺序: 数据 → 最后 magic, 原子提交语义
// 强制 clamp delay_ms 到 [0, VDLY_DELAY_MS_MAX] 保护电机不堵转过长
void config_save_vdelay(void)
{
    if(g_vdelay.delay_ms > VDLY_DELAY_MS_MAX) g_vdelay.delay_ms = VDLY_DELAY_MS_MAX;
    AT24C16_WriteByte(EE_ADDR_VDLY_ENABLED, g_vdelay.enabled);
    AT24C16_Write(EE_ADDR_VDLY_DELAY_MS, (const uint8_t *)&g_vdelay.delay_ms, 2);
    AT24C16_WriteByte(EE_ADDR_VDLY_MAGIC, EE_MAGIC_VDLY);  // 最后写
    printf_debug("[CFG] VDLY saved: en=%d delay_ms=%d\r\n",
                 g_vdelay.enabled, g_vdelay.delay_ms);
}

// ==================== 获取心跳值 (防无效值) ====================
uint16_t config_get_keepalive(void)
{
    if(g_keepalive_sec < 30)
        return APP_MQTT_KEEPALIVE;
    return g_keepalive_sec;
}

// ==================== Topic自动推导 ====================
// 从 clientId "<前缀>&<设备ID>&<产品ID>&<序号>" 按&分隔提取
// 前缀可变(S/E/...), 取seg[1]=设备ID, seg[2]=产品ID
// 生成 topic_up = "/<产品ID>/<设备ID>/property/post"
static void derive_topic_from_client_id(void)
{
    const char *p = g_mqtt.client_id;
    const char *seg[4] = {0};   // &分隔的段
    int seg_count = 0;
    seg[seg_count++] = p;

    while(*p && seg_count < 4)
    {
        if(*p == '&')
        {
            seg[seg_count++] = p + 1;
        }
        p++;
    }

    // seg[0]=前缀(S/E/...), seg[1]=设备ID, seg[2]=产品ID, seg[3]=序号
    if(seg_count >= 3)
    {
        // 提取设备ID (seg[1]到下一个&)
        char device_id[MQTT_FIELD_MAX] = {0};
        char product_id[MQTT_FIELD_MAX] = {0};
        uint16_t i;

        for(i = 0; seg[1][i] && seg[1][i] != '&' && i < MQTT_FIELD_MAX - 1; i++)
            device_id[i] = seg[1][i];
        device_id[i] = '\0';

        for(i = 0; seg[2][i] && seg[2][i] != '&' && i < MQTT_FIELD_MAX - 1; i++)
            product_id[i] = seg[2][i];
        product_id[i] = '\0';

        snprintf(g_mqtt.topic_up, MQTT_TOPIC_MAX, "/%s/%s/property/post",
                 product_id, device_id);
        printf_debug("[CFG] Topic derived: %s\r\n", g_mqtt.topic_up);
    }
    else
    {
        printf_debug("[CFG] Cannot derive topic from clientId\r\n");
    }
}

// ==================== 解析MQTT命令 ====================
// 格式: {clientId:<前缀>&<设备ID>&<产品ID>&<序号>,username:FastBee,passwd:xxx,IP:x.x.x.x,port:1883}
static int parse_mqtt_line(const char *cmd)
{
    if(!strstr(cmd, "clientId:")) return 0;

    char val_buf[MQTT_FIELD_MAX];

    if(extract_field(cmd, "clientId", val_buf, sizeof(val_buf)))
        strncpy(g_mqtt.client_id, val_buf, MQTT_FIELD_MAX - 1);
    if(extract_field(cmd, "username", val_buf, sizeof(val_buf)))
        strncpy(g_mqtt.username, val_buf, MQTT_FIELD_MAX - 1);
    if(extract_field(cmd, "passwd", val_buf, sizeof(val_buf)))
        strncpy(g_mqtt.password, val_buf, MQTT_FIELD_MAX - 1);
    if(extract_field(cmd, "IP", val_buf, sizeof(val_buf)))
        strncpy(g_mqtt.server, val_buf, MQTT_FIELD_MAX - 1);
    if(extract_field(cmd, "port", val_buf, sizeof(val_buf)))
        g_mqtt.port = (uint16_t)atoi(val_buf);

    // 自动从 clientId 推导 topic
    derive_topic_from_client_id();

    config_save_mqtt();
    return 1;
}

// ==================== 解析采集周期命令 ====================
// 格式: {id:collect_interval,value:0.5}
static int parse_collect_line(const char *cmd)
{
    if(!strstr(cmd, "id:collect_interval")) return 0;

    char val_buf[16];
    if(extract_field(cmd, "value", val_buf, sizeof(val_buf)))
    {
        g_interval.collect_interval_hours = (float)atof(val_buf);
        config_save_interval();
        printf_debug("[CFG] Collect interval: %.2fh\r\n", g_interval.collect_interval_hours);
        return 1;
    }
    return 0;
}

// ==================== 解析心跳命令 ==
// 格式: {id:keepalive,value:300}
static int parse_keepalive_line(const char *cmd)
{
    if(!strstr(cmd, "id:keepalive")) return 0;

    char val_buf[16];
    if(extract_field(cmd, "value", val_buf, sizeof(val_buf)))
    {
        uint16_t ka = (uint16_t)atoi(val_buf);
        if(ka < 30) ka = 30;
        g_keepalive_sec = ka;
        config_save_vpthreshold();
        printf_debug("[CFG] Keepalive: %d s\r\n", g_keepalive_sec);
        return 1;
    }
    return 0;
}

// ==================== 解析压力阈值 (vpthreshold) ====================
// 格式: {id:vpthreshold,value:switch:1,ppmax:5000,pmax:1000,ppmin:-5000,pmin:-1000,mutation:1000,shock:100}
static int parse_vpthreshold_line(const char *cmd)
{
    if(!strstr(cmd, "id:vpthreshold")) return 0;

    char val_buf[24];
    int changed = 0;

    if(extract_field(cmd, "switch",   val_buf, sizeof(val_buf)))
    { g_pthreshold.enabled  = (uint8_t)atoi(val_buf); if(g_pthreshold.enabled > 1) g_pthreshold.enabled = 0; changed = 1; }
    if(extract_field(cmd, "ppmax",    val_buf, sizeof(val_buf)))
    { g_pthreshold.ppmax    = (float)atof(val_buf); changed = 1; }
    if(extract_field(cmd, "pmax",     val_buf, sizeof(val_buf)))
    { g_pthreshold.pmax     = (float)atof(val_buf); changed = 1; }
    if(extract_field(cmd, "ppmin",    val_buf, sizeof(val_buf)))
    { g_pthreshold.ppmin    = (float)atof(val_buf); changed = 1; }
    if(extract_field(cmd, "pmin",     val_buf, sizeof(val_buf)))
    { g_pthreshold.pmin     = (float)atof(val_buf); changed = 1; }
    if(extract_field(cmd, "mutation", val_buf, sizeof(val_buf)))
    { g_pthreshold.mutation = (float)atof(val_buf); changed = 1; }
    if(extract_field(cmd, "shock",    val_buf, sizeof(val_buf)))
    { g_pthreshold.shock    = (float)atof(val_buf); changed = 1; }

    if(changed) config_save_vpthreshold();
    return 1;
}

// ==================== 解析温度阈值 (Tthreshold) ====================
// 格式: {id:Tthreshold,value:switch:1,high:50,low:-50,vhigh:100,vlow:-100}
static int parse_tthreshold_line(const char *cmd)
{
    if(!strstr(cmd, "id:Tthreshold")) return 0;

    char val_buf[24];
    int changed = 0;

    if(extract_field(cmd, "switch", val_buf, sizeof(val_buf)))
    { g_tthreshold.enabled = (uint8_t)atoi(val_buf); if(g_tthreshold.enabled > 1) g_tthreshold.enabled = 0; changed = 1; }
    if(extract_field(cmd, "high",   val_buf, sizeof(val_buf)))
    { g_tthreshold.high  = (float)atof(val_buf); changed = 1; }
    if(extract_field(cmd, "low",    val_buf, sizeof(val_buf)))
    { g_tthreshold.low   = (float)atof(val_buf); changed = 1; }
    if(extract_field(cmd, "vhigh",  val_buf, sizeof(val_buf)))
    { g_tthreshold.vhigh = (float)atof(val_buf); changed = 1; }
    if(extract_field(cmd, "vlow",   val_buf, sizeof(val_buf)))
    { g_tthreshold.vlow  = (float)atof(val_buf); changed = 1; }

    if(changed) config_save_tthreshold();
    return 1;
}

// ==================== 解析电压检测 (spupower) ====================
// 格式: {id:spupower,value:switch:1,high:3.6,low:3.0,protect:3.2}
static int parse_spupower_line(const char *cmd)
{
    if(!strstr(cmd, "id:spupower")) return 0;

    char val_buf[24];
    int changed = 0;

    if(extract_field(cmd, "switch",  val_buf, sizeof(val_buf)))
    { g_spupower.enabled = (uint8_t)atoi(val_buf); if(g_spupower.enabled > 1) g_spupower.enabled = 0; changed = 1; }
    if(extract_field(cmd, "high",    val_buf, sizeof(val_buf)))
    { g_spupower.high    = (float)atof(val_buf); changed = 1; }
    if(extract_field(cmd, "low",     val_buf, sizeof(val_buf)))
    { g_spupower.low     = (float)atof(val_buf); changed = 1; }
    if(extract_field(cmd, "protect", val_buf, sizeof(val_buf)))
    { g_spupower.protect = (float)atof(val_buf); changed = 1; }

    if(changed) config_save_spupower();
    return 1;
}

// ==================== 解析阀门动作延时 (delay) ====================
// 格式: {id:delay,value:switch:1,time:500}
// time 范围 0~VDLY_DELAY_MS_MAX (默认 5000ms), 超出在 config_save_vdelay 内 clamp
static int parse_vdelay_line(const char *cmd)
{
    if(!strstr(cmd, "id:delay")) return 0;

    char val_buf[24];
    int changed = 0;

    if(extract_field(cmd, "switch", val_buf, sizeof(val_buf)))
    { g_vdelay.enabled = (uint8_t)atoi(val_buf); if(g_vdelay.enabled > 1) g_vdelay.enabled = 0; changed = 1; }
    if(extract_field(cmd, "time",   val_buf, sizeof(val_buf)))
    {
        uint32_t v = (uint32_t)atoi(val_buf);
        if(v > VDLY_DELAY_MS_MAX) v = VDLY_DELAY_MS_MAX;
        g_vdelay.delay_ms = (uint16_t)v;
        changed = 1;
    }

    if(changed) config_save_vdelay();
    return 1;
}

// ==================== 统一配置命令解析(公开接口) ====================
// 返回: 1=有效配置, 0=无效/无数据, -1=reset(恢复出厂)
int config_parse_and_apply(const char *cmd)
{
    if(strstr(cmd, "reset"))
    {
        printf_debug("[CFG] Factory reset, rebooting...\r\n");
        trea_delay_ms(50);
        // 清除EEPROM所有Magic → 重启后自动进入MQTT等待
        AT24C16_WriteByte(EE_ADDR_INTERVAL_MAGIC, 0xFF);
        AT24C16_WriteByte(EE_ADDR_MQTT_MAGIC,     0xFF);
        AT24C16_WriteByte(EE_ADDR_EXT_MAGIC,      0xFF);
        AT24C16_WriteByte(EE_ADDR_VPTH_MAGIC,     0xFF);
        AT24C16_WriteByte(EE_ADDR_TTH_MAGIC,      0xFF);
        AT24C16_WriteByte(EE_ADDR_SPUP_MAGIC,     0xFF);
        AT24C16_WriteByte(EE_ADDR_VDLY_MAGIC,     0xFF);
        NVIC_SystemReset();
        return -1;  // 不会到达
    }
    if(parse_mqtt_line(cmd))         return 1;
    if(parse_collect_line(cmd))      return 1;
    if(parse_keepalive_line(cmd))    return 1;
    if(parse_vpthreshold_line(cmd))  return 1;
    if(parse_tthreshold_line(cmd))   return 1;
    if(parse_spupower_line(cmd))     return 1;
    if(parse_vdelay_line(cmd))       return 1;

    // ---- 异常关阀策略 ----
    // 格式: {id:abnormal,value:temph}
    if(strstr(cmd, "id:abnormal"))
    {
        char val_buf[16];
        if(extract_field(cmd, "value", val_buf, sizeof(val_buf)))
        {
            strncpy(g_abnormal_close, val_buf, 15);
            config_save_ext();
            printf_debug("[CFG] abnormal_close -> %s\r\n", g_abnormal_close);
        }
        else
            printf_debug("[CFG] Abnormal: %s\r\n", g_abnormal_close);
        return 1;
    }

    // ---- 采集周期(新格式) ----
    // 格式: {id:collect_cycle,once:22,type:minute}
    if(strstr(cmd, "id:collect_cycle"))
    {
        char val_buf[16];
        int got = 0;
        if(extract_field(cmd, "once", val_buf, sizeof(val_buf)))
        { g_collect_cycle.once = (uint16_t)atoi(val_buf); got = 1; }
        if(extract_field(cmd, "type", val_buf, sizeof(val_buf)))
        { g_collect_cycle.type = cycle_type_from_str(val_buf); got = 1; }
        if(got)
        {
            g_interval.collect_interval_hours = (float)cycle_to_seconds(&g_collect_cycle) / 3600.0f;
            config_save_interval();
            config_save_ext();
            config_recalc_interval();
        }
        printf_debug("[CFG] collect_cycle: %d*%s (%.2fh)\r\n",
                     g_collect_cycle.once, cycle_type_to_str(g_collect_cycle.type),
                     g_interval.collect_interval_hours);
        return 1;
    }

    // ---- 上报周期(新格式) ----
    // 格式: {id:report_cycle,once:12,type:minute}
    if(strstr(cmd, "id:report_cycle"))
    {
        char val_buf[16];
        int got = 0;
        if(extract_field(cmd, "once", val_buf, sizeof(val_buf)))
        { g_report_cycle.once = (uint16_t)atoi(val_buf); got = 1; }
        if(extract_field(cmd, "type", val_buf, sizeof(val_buf)))
        { g_report_cycle.type = cycle_type_from_str(val_buf); got = 1; }
        if(got)
        {
            g_interval.upload_interval_hours = (float)cycle_to_seconds(&g_report_cycle) / 3600.0f;
            config_save_interval();
            config_save_ext();
            config_recalc_interval();
        }
        printf_debug("[CFG] report_cycle: %d*%s (%.2fh)\r\n",
                     g_report_cycle.once, cycle_type_to_str(g_report_cycle.type),
                     g_interval.upload_interval_hours);
        return 1;
    }

    // ---- 窗口模式(新格式) ----
    // 格式: {id:window_pattern,value:short,start:080000,stop:200000,end_result:1,dottime:3}
    if(strstr(cmd, "id:window_pattern"))
    {
        char val_buf[16];
        int got = 0;
        if(extract_field(cmd, "value", val_buf, sizeof(val_buf)))
        { strncpy(g_window.pattern, val_buf, 7); got = 1; }
        if(extract_field(cmd, "start", val_buf, sizeof(val_buf)))
        { strncpy(g_window.start, val_buf, 7); got = 1; }
        if(extract_field(cmd, "stop", val_buf, sizeof(val_buf)))
        { strncpy(g_window.stop, val_buf, 7); got = 1; }
        if(extract_field(cmd, "end_result", val_buf, sizeof(val_buf)))
        { g_window.end_result = (uint8_t)atoi(val_buf); got = 1; }
        if(extract_field(cmd, "dottime", val_buf, sizeof(val_buf)))
        { g_window.dottime = (uint8_t)atoi(val_buf); got = 1; }
        if(got) config_save_ext();
        printf_debug("[CFG] window: %s er=%d s=%s e=%s dot=%d\r\n",
                     g_window.pattern, g_window.end_result,
                     g_window.start, g_window.stop, g_window.dottime);
        return 1;
    }

    // ---- 简单查询命令 (不需要main.c上下文) ----
    if(strcmp(cmd, "config") == 0)
    { config_print_current(); return 1; }
    if(strcmp(cmd, "vpthreshold_get") == 0)
    {
        printf_debug("  VPTH en=%d ppmax=%.1f pmax=%.1f ppmin=%.1f pmin=%.1f mut=%.1f shock=%.1f ka=%d\r\n",
                     g_pthreshold.enabled, g_pthreshold.ppmax, g_pthreshold.pmax,
                     g_pthreshold.ppmin, g_pthreshold.pmin,
                     g_pthreshold.mutation, g_pthreshold.shock, g_keepalive_sec);
        return 1;
    }
    if(strcmp(cmd, "Tthreshold_get") == 0)
    {
        printf_debug("  TTH  en=%d high=%.1f low=%.1f vhigh=%.1f vlow=%.1f\r\n",
                     g_tthreshold.enabled, g_tthreshold.high, g_tthreshold.low,
                     g_tthreshold.vhigh, g_tthreshold.vlow);
        return 1;
    }
    if(strcmp(cmd, "spupower_get") == 0)
    {
        printf_debug("  SPUP en=%d high=%.2f low=%.2f protect=%.2f\r\n",
                     g_spupower.enabled, g_spupower.high, g_spupower.low, g_spupower.protect);
        return 1;
    }
    if(strcmp(cmd, "delay_get") == 0)
    {
        printf_debug("  VDLY en=%d delay_ms=%d\r\n", g_vdelay.enabled, g_vdelay.delay_ms);
        return 1;
    }
    if(strcmp(cmd, "valve_status") == 0)
    {
        printf_debug("  valve_last_order=%d\r\n", g_valve_last_order);
        return 1;
    }
    if(strcmp(cmd, "ip") == 0)
    {
        printf_debug("  IP1: %s:%d  IP2: %s:%d\r\n",
                     g_mqtt.server, g_mqtt.port,
                     g_mqtt.server2[0] ? g_mqtt.server2 : "(none)", g_mqtt.port2);
        return 1;
    }

    // 返回0: 未匹配, 由调用方(main.c)继续处理需要硬件上下文的命令
    return 0;
}

// ==================== 上电配置检查 ====================
// 仅当MQTT未配置时阻塞等待(30秒), 期间接受任意配置命令(任意顺序)
// MQTT以外的参数都有默认值, 超时后用默认值启动
void config_enter_setup_mode(uint8_t force_reconfig)
{
    // 共享 s_cfg_line_buf (模块级); 与 config_check_cmd 互斥调用
    uint8_t mqtt_valid = 0;

    (void)force_reconfig;  // 不再需要: reset已改为清magic+重启

    // 检查MQTT是否已配置
    {
        uint8_t m = 0;
        AT24C16_Read(EE_ADDR_MQTT_MAGIC, &m, 1);
        mqtt_valid = (m == EE_MAGIC_MQTT) ? 1 : 0;
    }

    if(mqtt_valid)
    {
        printf_debug("[CFG] MQTT valid, ready\r\n");
        return;
    }

    // MQTT未配置: 进入配置窗口
    printf_debug("\r\n[CFG] === MQTT not configured, waiting (30s) ===\r\n");
    printf_debug("  MQTT: {clientId:xxx,username:xxx,passwd:xxx,IP:x.x.x.x,port:1883}\r\n");
    printf_debug("  Also accepts: collect_cycle / report_cycle / keepalive / vpthreshold / Tthreshold / spupower / delay / window_pattern\r\n");

    uint32_t start = trea_millis();
    uint32_t last_hint = 0;

    while((trea_millis() - start) < 30000)
    {
        // 每15秒提示
        uint32_t elapsed = (trea_millis() - start) / 1000;
        if(elapsed - last_hint >= 15)
        {
            last_hint = elapsed;
            printf_debug("[CFG] Waiting for MQTT... (%ds/30s)\r\n", (int)elapsed);
        }

        uint16_t len = debug_rx_read_line(s_cfg_line_buf, sizeof(s_cfg_line_buf), 1000);
        if(len == 0) continue;

        printf_debug("[CFG] Recv: %s\r\n", s_cfg_line_buf);
        config_parse_and_apply(s_cfg_line_buf);

        // 检查MQTT是否已配置
        {
            uint8_t m = 0;
            AT24C16_Read(EE_ADDR_MQTT_MAGIC, &m, 1);
            if(m == EE_MAGIC_MQTT)
            {
                printf_debug("[CFG] MQTT configured! Waiting 5s for optional params...\r\n");
                // 短窗口接收可选参数
                uint32_t opt_start = trea_millis();
                while((trea_millis() - opt_start) < 5000)
                {
                    len = debug_rx_read_line(s_cfg_line_buf, sizeof(s_cfg_line_buf), 1000);
                    if(len > 0)
                    {
                        printf_debug("[CFG] Recv: %s\r\n", s_cfg_line_buf);
                        config_parse_and_apply(s_cfg_line_buf);
                    }
                }
                config_print_current();
                printf_debug("[CFG] === Setup complete ===\r\n");
                return;
            }
        }
    }

    // 30秒超时, MQTT仍未配置
    printf_debug("[CFG] TIMEOUT: MQTT not configured, entering deep sleep...\r\n");
    trea_delay_ms(50);
    enter_deepsleep_mode(0);
}

// ==================== 打印当前配置 ====================
void config_print_current(void)
{
    printf_debug("\r\n--- Current Config ---\r\n");
    printf_debug("  MQTT: %s@%s:%d\r\n", g_mqtt.client_id, g_mqtt.server, g_mqtt.port);
    printf_debug("  User: %s\r\n", g_mqtt.username);
    printf_debug("  Topic: %s\r\n", g_mqtt.topic_up);
    printf_debug("  Keepalive: %d s\r\n", g_keepalive_sec);
    printf_debug("  Collect: %.2fh (%dmin) [%d*%s]\r\n",
                 g_interval.collect_interval_hours, g_interval.collect_interval_min,
                 g_collect_cycle.once, cycle_type_to_str(g_collect_cycle.type));
    printf_debug("  Upload:  %.2fh (%dmin) [%d*%s] every=%d\r\n",
                 g_interval.upload_interval_hours, g_interval.upload_interval_min,
                 g_report_cycle.once, cycle_type_to_str(g_report_cycle.type),
                 g_interval.upload_every_n);
    printf_debug("  Window: pattern=%s start=%s stop=%s er=%d dot=%d\r\n",
                 g_window.pattern, g_window.start, g_window.stop,
                 g_window.end_result, g_window.dottime);
    printf_debug("  Abnormal: %s\r\n", g_abnormal_close);
    printf_debug("  Valve: last_order=%d\r\n", g_valve_last_order);
    printf_debug("  VPTH en=%d ppmax=%.1f pmax=%.1f ppmin=%.1f pmin=%.1f mut=%.1f shock=%.1f\r\n",
                 g_pthreshold.enabled, g_pthreshold.ppmax, g_pthreshold.pmax,
                 g_pthreshold.ppmin, g_pthreshold.pmin,
                 g_pthreshold.mutation, g_pthreshold.shock);
    printf_debug("  TTH  en=%d high=%.1f low=%.1f vhigh=%.1f vlow=%.1f\r\n",
                 g_tthreshold.enabled, g_tthreshold.high, g_tthreshold.low,
                 g_tthreshold.vhigh, g_tthreshold.vlow);
    printf_debug("  SPUP en=%d high=%.2f low=%.2f protect=%.2f\r\n",
                 g_spupower.enabled, g_spupower.high, g_spupower.low, g_spupower.protect);
    printf_debug("  VDLY en=%d delay_ms=%d\r\n", g_vdelay.enabled, g_vdelay.delay_ms);
    printf_debug("  Realtime: enabled=%d frequency=%ds\r\n",
                 g_real_enabled, g_real_frequency);
    printf_debug("----------------------\r\n");
}

// ==================== 非阻塞检查debug串口(主循环每轮调用) ====================
void config_check_cmd(void)
{
    // 共享 s_cfg_line_buf (模块级); 与 config_enter_setup_mode 互斥调用
    uint16_t len;

    while((len = debug_rx_read_line(s_cfg_line_buf, sizeof(s_cfg_line_buf), 0)) > 0)
    {
        printf_debug("[CFG] Recv: %s\r\n", s_cfg_line_buf);
        config_parse_and_apply(s_cfg_line_buf);
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

    // 未配置窗口时间(start==0 && stop==0), 全天有效
    if(start == 0 && stop == 0) return 1;

    if(start < stop)
        return (now >= start && now < stop);
    else
        return (now >= start || now < stop);  // 跨午夜
}

// ==================== 智能决策当前采集周期是否需要采样电池电压 ====================
// 仅在首次、尾次（上报点）或低电压保护模式下开启采样，其余中间采集点保持关闭以最大化省电。
extern uint8_t voltage_low_power;
uint8_t config_need_adc_sampling(void)
{
    // 1. 低电压保护状态下：必须每次都采样，以便及时检测电池更换和电压恢复，退出低电压状态
    if(voltage_low_power)
        return 1;

    // 2. dot（定点模式）：一天上报一次，首次和尾次检测
    if(strcmp(g_window.pattern, "dot") == 0)
    {
        uint32_t now_hhmmss = rtc_get_hhmmss();
        uint8_t cur_hour = (uint8_t)(now_hhmmss / 10000);
        uint8_t cur_min = (uint8_t)((now_hhmmss / 100) % 100);

        uint8_t is_last_col = (cur_hour == g_window.dottime && cur_min == 0);
        uint8_t is_first_col = (cur_hour == g_window.dottime && cur_min == g_interval.collect_interval_min);

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

    return 1; // 其它模式（安全兜底）：都采
}
