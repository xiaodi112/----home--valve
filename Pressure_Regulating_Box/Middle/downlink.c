/**
 * @file downlink.c
 * @brief MQTT下行指令处理模块 (优化版)
 * @details 负责接收处理云平台下发的MQTT消息, 执行对应指令并上报结果
 */

#include "downlink.h"
#include "report.h"
#include "ec800e.h"
#include "eeprom_config.h"
#include "valve_timing.h"
#include "app_config.h"
#include "threshold.h"
#include "sensor_record.h"
#include "PT_304_35.h"
#include "main.h"
#include "adc_bat.h"
#include "ota_app.h"
#include "valve.h"
#include "usart.h"
#include "delay.h"
#include "tim.h"
#include "json_builder.h"
#include "gd32l23x.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>

// ==================== 跨模块外部变量 (定义在main.c) ====================
extern uint8_t fourG_user_off;
extern uint8_t g_need_adc_sampling;
extern volatile uint8_t g_ch4_request;

// ==================== 模块内静态缓冲区 ====================
static char json_buf[1024];
static char ts_buf[32];
static JsonBuilder jb;

// ==================== 内部: JSON字段提取 ====================
static int extract_json_field(const char *buf, const char *field, char *out, uint16_t out_size)
{
    char *p;
    char *q;
    char *r;
    uint16_t len;

    out[0] = '\0';
    p = strstr(buf, field);
    if(!p) return 0;
    q = strchr(p + strlen(field), '"');
    if(!q) return 0;
    q++;
    r = strchr(q, '"');
    if(!r) return 0;
    len = (uint16_t)(r - q);
    if(len >= out_size) len = out_size - 1;
    memcpy(out, q, len);
    out[len] = '\0';
    return (len > 0) ? 1 : 0;
}

// 提取 "key","value":"xxx" 格式字段值 (平台 set 指令格式)
static int extract_key_value(const char *buf, const char *key, char *out, uint16_t out_size)
{
    char pat[64];
    char *p;
    char *q;
    uint16_t len;

    out[0] = '\0';
    snprintf(pat, sizeof(pat), "\"%s\",\"value\"", key);
    p = strstr(buf, pat);
    if(!p) return 0;
    p += strlen(pat);
    if(*p == ':') p++;
    if(*p != '"') return 0;
    p++;
    q = strchr(p, '"');
    if(!q) return 0;
    len = (uint16_t)(q - p);
    if(len >= out_size) len = out_size - 1;
    memcpy(out, p, len);
    out[len] = '\0';
    return (len > 0) ? 1 : 0;
}

// ==================== commandStr 映射表 ====================
typedef struct {
    const char *cmd_str;     // commandStr 值
    const char *id;          // 映射到 id_val
    const char *value;       // 固定值, NULL 表示从 raw_buf 提取
    const char *extract;     // 提取字段名 (value 为 NULL 时使用)
} CmdMap;

static const CmdMap s_cmd_map[] = {
    {"valve_open",        "valve",           "open",  NULL},
    {"valve_close",       "valve",           "close", NULL},
    {"get_pressure",      "get_pressure",    "1",     NULL},
    {"get_pressure1",     "get_pressure",    "1",     NULL},
    {"get_sim",           "get_sim",         "1",     NULL},
    {"get_ip",            "get_ip",          "1",     NULL},
    {"sensor_status",     "sensor_status",   "1",     NULL},
    {"restart",           "restart",         "1",     NULL},
    {"restart_off",       "restart_off",     "1",     NULL},
    {"4Gpower_off",       "4Gpower_off",     "1",     NULL},
    {"temp_get1",         "temp_status",     "1",     NULL},
    {"temp_get2",         "temp_status2",    "1",     NULL},
    {"LY06_get",          "LY06_get",        "1",     NULL},
    {"gas_get",           "gas_get",         "1",     NULL},
    {"laser_get",         "laser_get",       "",      NULL},
    {"laser_set",         "laser_set",       NULL,    "laser_switch"},
    {"msh_get",           "msh_get",         "",      NULL},
    {"msh_set",           "msh_set",         NULL,    "msh_threshold"},
    {"get_pressure2",     "get_pressure2",   "1",     NULL},
    {"threshold_get1",    "threshold",       "",      NULL},
    {"collectcycle_get",  "collect_cycle",   "",      NULL},
    {"report_get",        "report_cycle",    "",      NULL},
    {"window_get",        "window_pattern",  "",      NULL},
    {"abnormal_get",      "abnormal_close",  "",      NULL},
    {"abnormal2_get",     "abnormal2_close", "",      NULL},
    {"abnormal3_get",     "abnormal3_close", "",      NULL},
    {"first_ip_get",      "get_ip",          "1",     NULL},
    {"scend_ip_get",      "get_scend_ip",    "1",     NULL},
    {"valve_last_get",    "valve_last_order","1",     NULL},
    {"real_get",          "real_get",        "",      NULL},
    {"user_ip_get",       "user_ip_get",     "1",     NULL},
    {"MQTT_get",          "user_mqtt_get",   "1",     NULL},
    // 带参数提取的命令
    {"collectspeed_set",  "collect_cycle",   NULL,    "collect_once"},
    {"report_set",        "report_cycle",    NULL,    "report_once"},
    {"window_set",        "window_pattern",  NULL,    "window_pattern"},
    {"abnormal_set",      "abnormal_close",  NULL,    "abnormal_close"},
    {"abnormal2_set",     "abnormal2_close", NULL,    "abnormal2_close"},
    {"abnormal3_set",     "abnormal3_close", NULL,    "abnormal3_close"},
    {"first_ip_set",      "first_ip_set",    NULL,    "first_ip"},
    {"scend_ip_set",      "scend_ip_set",    NULL,    "scend_ip"},
    {"real_set",          "real_set",        NULL,    "real_enabled"},
    {"user_ip_set",       "user_ip_set",     NULL,    "user_ip"},
    {"MQTT_set",          "user_mqtt_set",   "1",     NULL},
    // 阈值 set/get
    {"vpthreshold_set1",  "vpthreshold",     "1",     NULL},
    {"vpthreshold_get1",  "vpthreshold",     "",      NULL},
    {"Tthreshold_set",    "Tthreshold",      "1",     NULL},
    {"Tthreshold_get",    "Tthreshold",      "",      NULL},
    {"spupower_set",      "spupower",        "1",     NULL},
    {"spupower_get",      "spupower",        "",      NULL},
    {"delay_set",         "delay",           "1",     NULL},
    {"delay_get",         "delay",           "",      NULL},
    {"vpthreshold_set2",  "vpthreshold2",    "1",     NULL},
    {"vpthreshold_get2",  "vpthreshold2",    "",      NULL},
    {"Tthreshold_set2",   "Tthreshold2",     "1",     NULL},
    {"Tthreshold_get2",   "Tthreshold2",     "",      NULL},
    {"lcd_switch",        "lcd_switch",      NULL,    "lcd_switch"},
    // 阀门及定时开关
    {"valve_get",         "valve_status",    "1",     NULL},
    {"closeoropen_get",   "closeoropen",     "",      NULL},
    {"closeoropen_set",   "closeoropen",     "1",     NULL},
    // 表号
    {"table_get",         "table",           "",      NULL},
    {"table_set",         "table",           NULL,    "table"},
};
#define CMD_MAP_COUNT (sizeof(s_cmd_map) / sizeof(s_cmd_map[0]))

// ==================== 通用辅助函数 ====================

// 采集所有传感器
static void collect_all_sensors(void)
{
    PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);
}

// 双重JSON字段提取: 先查短格式 "key", 再查长格式 "key","value"
static int extract_field_dual(const char *raw_buf, const char *key_short,
                              const char *key_long, char *out, uint16_t out_size)
{
    char key1[48];
    char pat[64];
    snprintf(key1, sizeof(key1), "\"%s\"", key_short);
    if(extract_json_field(raw_buf, key1, out, out_size))
        return 1;
    snprintf(pat, sizeof(pat), "\"%s\",\"value\"", key_long);
    return extract_json_field(raw_buf, pat, out, out_size);
}

// 解析 "IP.port" 字符串 (按最后一个'.'分割)
static int parse_ip_port(const char *str, char *ip_out, uint16_t *port_out)
{
    char buf[MQTT_FIELD_MAX];
    char *dot;
    strncpy(buf, str, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    dot = strrchr(buf, '.');
    if(!dot) return 0;
    *dot = '\0';
    *port_out = (uint16_t)atoi(dot + 1);
    strncpy(ip_out, buf, MQTT_FIELD_MAX - 1);
    ip_out[MQTT_FIELD_MAX - 1] = '\0';
    return 1;
}

// 发布JSON并延时+反馈
static void publish_json_df(int fb_code)
{
    jb_end(&jb);
    EC801E_PublishRawJson(json_buf);
    trea_delay_ms(300);
    report_publish_feedback(fb_code);
}

// 仅发布JSON (无延时无反馈)
static void publish_json_only(void)
{
    jb_end(&jb);
    EC801E_PublishRawJson(json_buf);
}

// ==================== 阈值表驱动处理 ====================
typedef enum { FT_U8_BOOL, FT_FLOAT, FT_U16_CLAMP } ThrFieldType;

typedef struct {
    const char  *name;          // 字段名后缀 (如 "switch", "ppmax")
    uint16_t     offset;        // 结构体内偏移量
    ThrFieldType type;
    uint16_t     clamp_max;     // FT_U16_CLAMP时的最大值
} ThrField;

typedef struct {
    const char     *prefix;     // 字段前缀 (如 "vpthreshold")
    void           *config;     // 配置结构体指针
    const ThrField *fields;
    uint8_t         field_count;
    void          (*save_fn)(void);
    uint8_t         use_float1; // 1=jb_add_float1, 0=jb_add_float
    uint8_t         has_guard;  // 1=需要cmd_val非空守卫
    uint8_t         fb_set;     // set成功反馈码
    uint8_t         fb_get;     // get反馈码
} ThrGroup;

// 各阈值字段表
static const ThrField vp_fields[] = {
    {"switch",  offsetof(PressureThresholdConfig_t, enabled),  FT_U8_BOOL,   0},
    {"ppmax",   offsetof(PressureThresholdConfig_t, ppmax),     FT_FLOAT,     0},
    {"pmax",    offsetof(PressureThresholdConfig_t, pmax),      FT_FLOAT,     0},
    {"ppmin",   offsetof(PressureThresholdConfig_t, ppmin),     FT_FLOAT,     0},
    {"pmin",    offsetof(PressureThresholdConfig_t, pmin),      FT_FLOAT,     0},
    {"mutation",offsetof(PressureThresholdConfig_t, mutation), FT_FLOAT,     0},
    {"shock",   offsetof(PressureThresholdConfig_t, shock),     FT_FLOAT,     0},
};

static const ThrField tt_fields[] = {
    {"switch", offsetof(TempThresholdConfig_t, enabled), FT_U8_BOOL, 0},
    {"high",   offsetof(TempThresholdConfig_t, high),    FT_FLOAT,   0},
    {"low",    offsetof(TempThresholdConfig_t, low),     FT_FLOAT,   0},
    {"vhigh",  offsetof(TempThresholdConfig_t, vhigh),   FT_FLOAT,   0},
    {"vlow",   offsetof(TempThresholdConfig_t, vlow),    FT_FLOAT,   0},
};

static const ThrField sp_fields[] = {
    {"switch",  offsetof(VoltagePowerConfig_t, enabled), FT_U8_BOOL, 0},
    {"high",    offsetof(VoltagePowerConfig_t, high),    FT_FLOAT,   0},
    {"low",     offsetof(VoltagePowerConfig_t, low),     FT_FLOAT,   0},
    {"protect", offsetof(VoltagePowerConfig_t, protect), FT_FLOAT,   0},
};

static const ThrField vd_fields[] = {
    {"switch", offsetof(ValveDelayConfig_t, enabled),  FT_U8_BOOL,   0},
    {"time",   offsetof(ValveDelayConfig_t, delay_ms), FT_U16_CLAMP, VDLY_DELAY_MS_MAX},
};

// 阈值组定义表
static const ThrGroup thr_groups[] = {
    {"vpthreshold",  &g_pthreshold,  vp_fields, 7, config_save_vpthreshold,  0, 0, 8, 7},
    {"Tthreshold",   &g_tthreshold,  tt_fields, 5, config_save_tthreshold,  0, 0, 6, 5},
    {"spupower",     &g_spupower,    sp_fields, 4, config_save_spupower,    0, 0, 16, 15},
    {"delay",        &g_vdelay,      vd_fields, 2, config_save_vdelay,      0, 0, 20, 19},
    {"vpthreshold2", &g_pthreshold2, vp_fields, 7, config_save_vpthreshold2, 1, 1, 8, 7},
    {"Tthreshold2",  &g_tthreshold2, tt_fields, 5, config_save_tthreshold2, 1, 1, 6, 5},
};
#define THR_GROUP_COUNT (sizeof(thr_groups) / sizeof(thr_groups[0]))

// 发布阈值组当前值
static void publish_threshold_group(const ThrGroup *grp)
{
    uint8_t i;
    rtc_get_time_str(ts_buf, sizeof(ts_buf));
    jb_begin(&jb, json_buf, sizeof(json_buf));
    for(i = 0; i < grp->field_count; i++)
    {
        const ThrField *f = &grp->fields[i];
        char key[48];
        uint8_t *p = (uint8_t*)grp->config + f->offset;
        snprintf(key, sizeof(key), "%s_%s", grp->prefix, f->name);
        switch(f->type)
        {
            case FT_U8_BOOL:
                jb_add_int(&jb, key, *(uint8_t*)p, ts_buf);
                break;
            case FT_U16_CLAMP:
                jb_add_int(&jb, key, *(uint16_t*)p, ts_buf);
                break;
            case FT_FLOAT:
                if(grp->use_float1)
                    jb_add_float1(&jb, key, *(float*)p, ts_buf);
                else
                    jb_add_float(&jb, key, *(float*)p, ts_buf);
                break;
        }
    }
    jb_end(&jb);
    EC801E_PublishRawJson(json_buf);
}

// 处理阈值 set/get
static void process_threshold(const ThrGroup *grp, const char *cmd_val, const char *raw_buf)
{
    char tv[16];
    int changed = 0;
    int do_set = !grp->has_guard || (cmd_val[0] != '\0');

    if(do_set)
    {
        uint8_t i;
        for(i = 0; i < grp->field_count; i++)
        {
            const ThrField *f = &grp->fields[i];
            char key[48];
            uint8_t *p = (uint8_t*)grp->config + f->offset;
            snprintf(key, sizeof(key), "%s_%s", grp->prefix, f->name);
            if(extract_key_value(raw_buf, key, tv, sizeof(tv)))
            {
                switch(f->type)
                {
                    case FT_U8_BOOL:
                        *p = (uint8_t)atoi(tv);
                        if(*p > 1) *p = 0;
                        break;
                    case FT_FLOAT:
                        *(float*)p = (float)atof(tv);
                        break;
                    case FT_U16_CLAMP:
                    {
                        uint32_t v = (uint32_t)atoi(tv);
                        if(v > f->clamp_max) v = f->clamp_max;
                        *(uint16_t*)p = (uint16_t)v;
                        break;
                    }
                }
                changed = 1;
            }
        }
        if(changed || grp->has_guard)
        {
            grp->save_fn();
            printf_debug("[SM] %s updated\r\n", grp->prefix);
        }
    }

    if(EC801E_EnsureConnected() == EC_OK)
    {
        publish_threshold_group(grp);
        trea_delay_ms(300);
        if(grp->has_guard)
            report_publish_feedback(do_set ? grp->fb_set : grp->fb_get);
        else
            report_publish_feedback(changed ? grp->fb_set : grp->fb_get);
    }
}




// ==================== 指令处理函数 (每个分支一个handler, 表驱动) ====================
static void h_valve(const char *id_val, const char *cmd_val, const char *raw_buf)
{
// ---- 阀门控制 ----
        const char *result = valve_execute(cmd_val, 1);
        printf_debug("[SM] Valve result: %s\r\n", result);

        if(g_valve_alarm_closed)
        {
            g_valve_alarm_closed = 0;
            g_valve_pre_alarm_order = g_valve_last_order;
            config_save_ext();
            printf_debug("[SM] Valve user override: cleared alarm_closed flag\r\n");
        }
        if(g_ch4_alarm_closed)
        {
            g_ch4_alarm_closed = 0;
            config_save_ext();   // 持久化清除关阀标志, 防止重启后仍处于告警闭锁
            printf_debug("[SM] Valve user override: cleared ch4_alarm_closed flag\r\n");
        }
        g_alarm_recover_reported = 0;   // 手动操作视为新一轮, 恢复上报标志复位

        if(EC801E_EnsureConnected() == EC_OK)
        {
            EC801E_PublishValveStatus(result, g_valve_last_order);
            trea_delay_ms(300);
            report_publish_feedback(23);
        }
        else
            printf_debug("[SM] Valve publish skipped: MQTT not connected\r\n");
}

static void h_sensor_read(const char *id_val, const char *cmd_val, const char *raw_buf)
{
// ---- 即时采集+阈值校验+压力状态上报 ----
        PressureStatus_t ps;
        const char *ps_str;

        collect_all_sensors();
        {
            float v_now = adc_bat_read_voltage();
            g_sensor.voltage = (v_now >= 0.0f) ? v_now : g_sensor.last_normal_voltage;
        }
        printf_debug("[SM] sensor_read: P=%.2f T=%.2f V=%.2f err=%d\r\n",
                     g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage, g_sensor.raw.err);

        if(EC801E_EnsureConnected() == EC_OK)
            report_publish_sensor_status();

        ps = threshold_get_pressure_status(g_sensor.raw.pressure, g_sensor.last_pressure_collect);
        ps_str = threshold_pressure_status_str(ps);
        g_sensor.last_pressure_collect = g_sensor.raw.pressure;

        if(EC801E_EnsureConnected() == EC_OK)
        {
            EC801E_PublishSensorData(&g_sensor.raw, g_sensor.voltage);
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_str(&jb, "pressure_status", ps_str, ts_buf);
            publish_json_only();
            trea_delay_ms(300);
            report_publish_feedback(30);   /* 压力状态获取成功 */
        }

        // 阈值告警 + 异常自动关阀 (进气端 abnormal_close, 出气端 abnormal2_close)
        {
            ThresholdResult_t tr = threshold_check(g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage, g_sensor.last_pressure_collect, g_need_adc_sampling, 0);
            ThresholdResult_t tr2 = THRESHOLD_OK;
            if(g_sensor_count >= 2 && g_sensor.raw2.err == PT304D_OK)
                tr2 = threshold_check2(g_sensor.raw2.pressure, g_sensor.raw2.temperature, g_sensor.last_pressure_collect2, 0);
            if(tr != THRESHOLD_OK || tr2 != THRESHOLD_OK)
            {
                ThresholdResult_t tr_eff = (tr != THRESHOLD_OK) ? tr : tr2;
                printf_debug("[SM] sensor_read ALARM: %s\r\n", threshold_alarm_type_str(tr_eff));
                if(EC801E_EnsureConnected() == EC_OK)
                {
                    report_publish_alarm_batch(g_sensor.last_pressure_collect, g_sensor.last_pressure_collect2);
                    if(tr == THRESHOLD_VPROTECT || tr == THRESHOLD_VOLT_HIGH || tr == THRESHOLD_VOLT_LOW)
                        report_publish_ly06(1, g_sensor.voltage);
                }

                const char *vr1 = alarm_try_close_valve(tr);
                const char *vr2 = alarm_try_close_valve2(tr2);
                const char *vr = vr1 ? vr1 : vr2;
                if(vr && EC801E_EnsureConnected() == EC_OK)
                    EC801E_PublishValveStatus(vr, g_valve_last_order);
            }
        }
}

static void h_collect_cycle(const char *id_val, const char *cmd_val, const char *raw_buf)
{
if(cmd_val[0] != '\0')
        {
            char type_buf[16] = {0};
            g_collect_cycle.once = (uint16_t)atoi(cmd_val);
            extract_field_dual(raw_buf, "type", "collect_type", type_buf, sizeof(type_buf));
            if(type_buf[0]) g_collect_cycle.type = cycle_type_from_str(type_buf);
            g_interval.collect_interval_hours = (float)cycle_to_seconds(&g_collect_cycle) / 3600.0f;
            config_save_interval();
            config_save_ext();
            config_recalc_interval();
            printf_debug("[SM] collect_cycle -> %d*%s\r\n",
                         g_collect_cycle.once, cycle_type_to_str(g_collect_cycle.type));
        }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_int(&jb, "collect_once", g_collect_cycle.once, ts_buf);
            jb_add_str(&jb, "collect_type", cycle_type_to_str(g_collect_cycle.type), ts_buf);
            publish_json_df(cmd_val[0] != '\0' ? 12 : 11);
        }
}

static void h_report_cycle(const char *id_val, const char *cmd_val, const char *raw_buf)
{
if(cmd_val[0] != '\0')
        {
            char type_buf[16] = {0};
            g_report_cycle.once = (uint16_t)atoi(cmd_val);
            extract_field_dual(raw_buf, "type", "report_type", type_buf, sizeof(type_buf));
            if(type_buf[0]) g_report_cycle.type = cycle_type_from_str(type_buf);
            g_interval.upload_interval_hours = (float)cycle_to_seconds(&g_report_cycle) / 3600.0f;
            config_save_interval();
            config_save_ext();
            config_recalc_interval();
            printf_debug("[SM] report_cycle -> %d*%s\r\n",
                         g_report_cycle.once, cycle_type_to_str(g_report_cycle.type));
        }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_int(&jb, "report_once", g_report_cycle.once, ts_buf);
            jb_add_str(&jb, "report_type", cycle_type_to_str(g_report_cycle.type), ts_buf);
            publish_json_df(cmd_val[0] != '\0' ? 10 : 9);
        }
}

static void h_vpthreshold(const char *id_val, const char *cmd_val, const char *raw_buf)
{
process_threshold(&thr_groups[0], cmd_val, raw_buf);
}

static void h_tthreshold(const char *id_val, const char *cmd_val, const char *raw_buf)
{
process_threshold(&thr_groups[1], cmd_val, raw_buf);
}

static void h_spupower(const char *id_val, const char *cmd_val, const char *raw_buf)
{
process_threshold(&thr_groups[2], cmd_val, raw_buf);
}

static void h_gas_get(const char *id_val, const char *cmd_val, const char *raw_buf)
{
#if LASER_CH4_ENABLED
    /* 置位CH4采集请求: do_downlink 检测到后进入 STATE_CH4 执行采集+上报 */
    g_ch4_request = 1;
    printf_debug("[SM] gas_get: CH4 collect requested\r\n");
#else
    printf_debug("[SM] gas_get: laser sensor not supported\r\n");
#endif
}

/* 激光甲烷定时采集配置上报: laser_switch + laser_time (协议示例格式)
 * TODO: 平台反馈码待定, 后续在 set/get 成功后调用 report_publish_feedback(fb_code) */
#if LASER_CH4_ENABLED
static void laser_publish_config(void)
{
    char time_str[96] = {0};
    char sw_str[4] = {0};
    window_dottime_to_str(g_laser_time_mask, time_str, sizeof(time_str));
    snprintf(sw_str, sizeof(sw_str), "%d", g_laser_switch);
    rtc_get_time_str(ts_buf, sizeof(ts_buf));
    jb_begin(&jb, json_buf, sizeof(json_buf));
    jb_add_str(&jb, "laser_switch", sw_str, ts_buf);
    jb_add_str(&jb, "laser_time", time_str, ts_buf);
    publish_json_only();
}

static void h_laser_get(const char *id_val, const char *cmd_val, const char *raw_buf)
{
    printf_debug("[SM] laser_get: switch=%d time=%s\r\n",
                 g_laser_switch, (window_dottime_first(g_laser_time_mask) == 0xFF) ? "(none)" : "mask");
    if(EC801E_EnsureConnected() == EC_OK)
    {
        laser_publish_config();
        trea_delay_ms(300);
        report_publish_feedback(33);   /* 激光时间点读取成功 */
    }
}

static void h_laser_set(const char *id_val, const char *cmd_val, const char *raw_buf)
{
    char tv[96] = {0};
    if(extract_key_value(raw_buf, "laser_switch", tv, sizeof(tv)))
    {
        g_laser_switch = (uint8_t)atoi(tv);
        if(g_laser_switch > 1) g_laser_switch = 0;
    }
    if(extract_key_value(raw_buf, "laser_time", tv, sizeof(tv)))
        window_dottime_from_str(tv, g_laser_time_mask);
    config_save_laser();
    printf_debug("[SM] laser_set: switch=%d time=%s\r\n",
                 g_laser_switch, tv);
    if(EC801E_EnsureConnected() == EC_OK)
    {
        laser_publish_config();
        trea_delay_ms(300);
        report_publish_feedback(34);   /* 激光时间点配置成功 */
    }
}
#else
static void h_laser_get(const char *id_val, const char *cmd_val, const char *raw_buf)
{
    printf_debug("[SM] laser_get: laser sensor not supported\r\n");
}

static void h_laser_set(const char *id_val, const char *cmd_val, const char *raw_buf)
{
    printf_debug("[SM] laser_set: laser sensor not supported\r\n");
}
#endif

/* 激光甲烷门限值 (msh_threshold) 获取与配置
 * 反馈码: 35=门限值获取成功, 36=门限值配置成功 */
#if LASER_CH4_ENABLED
static void msh_publish_config(void)
{
    rtc_get_time_str(ts_buf, sizeof(ts_buf));
    jb_begin(&jb, json_buf, sizeof(json_buf));
    jb_add_float(&jb, "msh_threshold", g_ch4_threshold.threshold, ts_buf);
    publish_json_only();
}

static void h_msh_get(const char *id_val, const char *cmd_val, const char *raw_buf)
{
    printf_debug("[SM] msh_get: threshold=%.2f\r\n", g_ch4_threshold.threshold);
    if(EC801E_EnsureConnected() == EC_OK)
    {
        msh_publish_config();
        trea_delay_ms(300);
        report_publish_feedback(35);   /* 甲烷门限值获取成功 */
    }
}

static void h_msh_set(const char *id_val, const char *cmd_val, const char *raw_buf)
{
    char tv[16] = {0};
    if(extract_key_value(raw_buf, "msh_threshold", tv, sizeof(tv)))
    {
        float th = (float)atof(tv);
        /* 非法值(NaN/越界 0~999)回退默认 0.5 */
        if(!(th >= DEF_CH4_THRESHOLD_MIN && th <= DEF_CH4_THRESHOLD_MAX))
            th = DEF_CH4_THRESHOLD;
        g_ch4_threshold.threshold = th;
        config_save_ch4_threshold();
    }
    printf_debug("[SM] msh_set: threshold=%.2f\r\n", g_ch4_threshold.threshold);
    if(EC801E_EnsureConnected() == EC_OK)
    {
        msh_publish_config();
        trea_delay_ms(300);
        report_publish_feedback(36);   /* 甲烷门限值配置成功 */
    }
}
#else
static void h_msh_get(const char *id_val, const char *cmd_val, const char *raw_buf)
{
    printf_debug("[SM] msh_get: laser sensor not supported\r\n");
}

static void h_msh_set(const char *id_val, const char *cmd_val, const char *raw_buf)
{
    printf_debug("[SM] msh_set: laser sensor not supported\r\n");
}
#endif

static void h_threshold(const char *id_val, const char *cmd_val, const char *raw_buf)
{
// 批量阈值查询: 依次发布压力/温度/电压阈值
        if(EC801E_EnsureConnected() == EC_OK)
        {
            publish_threshold_group(&thr_groups[0]);
            trea_delay_ms(300);
            publish_threshold_group(&thr_groups[1]);
            trea_delay_ms(300);
            publish_threshold_group(&thr_groups[2]);
            trea_delay_ms(300);
            report_publish_feedback(5);
        }
}

static void h_delay(const char *id_val, const char *cmd_val, const char *raw_buf)
{
process_threshold(&thr_groups[3], cmd_val, raw_buf);
}

static void h_window_pattern(const char *id_val, const char *cmd_val, const char *raw_buf)
{
if(cmd_val[0] != '\0')
        {
            char tmp[16] = {0};
            strncpy(g_window.pattern, cmd_val, 7); g_window.pattern[7] = '\0';
            if(extract_field_dual(raw_buf, "end_result", "window_end_result", tmp, sizeof(tmp)))
                g_window.end_result = (uint8_t)atoi(tmp);
            if(extract_field_dual(raw_buf, "start", "window_start", tmp, sizeof(tmp)))
            { strncpy(g_window.start, tmp, 7); g_window.start[7] = '\0'; }
            if(extract_field_dual(raw_buf, "stop", "window_stop", tmp, sizeof(tmp)))
            { strncpy(g_window.stop, tmp, 7); g_window.stop[7] = '\0'; }
            if(extract_field_dual(raw_buf, "dottime", "window_dottime", tmp, sizeof(tmp)))
                window_dottime_from_str(tmp, g_window.dottime_mask);
            config_save_ext();
            char dot_str4[64]; window_dottime_to_str(g_window.dottime_mask, dot_str4, sizeof(dot_str4));
            printf_debug("[SM] window: %s er=%d s=%s e=%s dot=%s\r\n",
                         g_window.pattern, g_window.end_result, g_window.start, g_window.stop, dot_str4);
        }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_str(&jb, "window_pattern", g_window.pattern, ts_buf);
            jb_add_int(&jb, "window_end_result", g_window.end_result, ts_buf);
            jb_add_str(&jb, "window_start", g_window.start, ts_buf);
            jb_add_str(&jb, "window_stop", g_window.stop, ts_buf);
            char dot_str2[64]; window_dottime_to_str(g_window.dottime_mask, dot_str2, sizeof(dot_str2));
            jb_add_str(&jb, "window_dottime", dot_str2, ts_buf);
            publish_json_df(cmd_val[0] != '\0' ? 4 : 3);
        }
}

static void h_abnormal_close(const char *id_val, const char *cmd_val, const char *raw_buf)
{
// ---- 异常自动关阀设置 (cmd_val非空=设置, 空=查询) ----
        if(cmd_val[0] != '\0')
        {
            strncpy(g_abnormal_close, cmd_val, 15); g_abnormal_close[15] = '\0';
            config_save_ext();
            printf_debug("[SM] abnormal_close -> %s\r\n", g_abnormal_close);
        }
        if(EC801E_EnsureConnected() == EC_OK)
            EC801E_PublishResponse("abnormal_close", g_abnormal_close);
        report_publish_feedback(cmd_val[0] != '\0' ? 29 : 28);
}

/* 出气端异常自动关阀 (abnormal2_close pv* 策略) */
static void h_abnormal2_close(const char *id_val, const char *cmd_val, const char *raw_buf)
{
        if(cmd_val[0] != '\0')
        {
            strncpy(g_abnormal2_close, cmd_val, 15); g_abnormal2_close[15] = '\0';
            config_save_abn23();
            printf_debug("[SM] abnormal2_close -> %s\r\n", g_abnormal2_close);
        }
        if(EC801E_EnsureConnected() == EC_OK)
            EC801E_PublishResponse("abnormal2_close", g_abnormal2_close);
        report_publish_feedback(cmd_val[0] != '\0' ? 29 : 28);
}

/* 激光甲烷异常自动关阀 (abnormal3_close: 0=关 1=开) */
static void h_abnormal3_close(const char *id_val, const char *cmd_val, const char *raw_buf)
{
        if(cmd_val[0] != '\0')
        {
            g_abnormal3_close = (uint8_t)atoi(cmd_val);
            if(g_abnormal3_close > 1) g_abnormal3_close = 0;
            config_save_abn23();
            printf_debug("[SM] abnormal3_close -> %d\r\n", g_abnormal3_close);
        }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            char v[4];
            snprintf(v, sizeof(v), "%d", g_abnormal3_close);
            EC801E_PublishResponse("abnormal3_close", v);
        }
        report_publish_feedback(cmd_val[0] != '\0' ? 29 : 28);
}

static void h_valve_last_order(const char *id_val, const char *cmd_val, const char *raw_buf)
{
char v[4];
        snprintf(v, sizeof(v), "%d", g_valve_last_order);
        if(EC801E_EnsureConnected() == EC_OK)
            EC801E_PublishResponse("valve_last_order", v);
}

static void h_keepalive(const char *id_val, const char *cmd_val, const char *raw_buf)
{
uint16_t ka = (uint16_t)atoi(cmd_val);
        if(ka >= 30)
        {
            g_keepalive_sec = ka;
            config_save_vpthreshold();
            char at_cmd[64];
            snprintf(at_cmd, sizeof(at_cmd), "AT+QMTCFG=\"keepalive\",0,%d\r\n", ka);
            EC801E_Wakeup();
            printf_cat1("%s", at_cmd);
            trea_delay_ms(500);
            printf_debug("[SM] keepalive -> %d s\r\n", ka);
        }
}

static void h_get_sim(const char *id_val, const char *cmd_val, const char *raw_buf)
{
char iccid[32] = {0}, csq[8] = {0}, imei_buf[24] = {0};
        char rsrp_buf[8], sinr_buf[8], cereg_buf[4] = {0};
        uint8_t cereg = 0;
        report_query_signal_info(csq, sizeof(csq), rsrp_buf, sizeof(rsrp_buf), sinr_buf, sizeof(sinr_buf));
        EC801E_QueryCEREG(&cereg);
        snprintf(cereg_buf, sizeof(cereg_buf), "%d", cereg);
        EC801E_QueryICCID(iccid, sizeof(iccid));
        EC801E_QueryIMEI(imei_buf, sizeof(imei_buf));
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_str(&jb, "csq", csq, ts_buf);
            jb_add_str(&jb, "rsrp", rsrp_buf, ts_buf);
            jb_add_str(&jb, "snr", sinr_buf, ts_buf);
            jb_add_str(&jb, "cereg", cereg_buf, ts_buf);
            jb_add_str(&jb, "soft-version", APP_FIRMWARE_VERSION, ts_buf);
            jb_add_str(&jb, "ICCID", iccid, ts_buf);
            jb_add_str(&jb, "IMEI", imei_buf, ts_buf);
            publish_json_only();
        }
}

static void h_get_ip(const char *id_val, const char *cmd_val, const char *raw_buf)
{
report_publish_ip_field("first_ip", g_mqtt.server, g_mqtt.port, 1);
}

static void h_get_scend_ip(const char *id_val, const char *cmd_val, const char *raw_buf)
{
report_publish_ip_field("scend_ip", g_mqtt.server2, g_mqtt.port2, 1);
}

static void h_first_ip_set(const char *id_val, const char *cmd_val, const char *raw_buf)
{
char ip_buf[MQTT_FIELD_MAX];
        uint16_t port;
        if(parse_ip_port(cmd_val, ip_buf, &port))
        {
            strncpy(g_mqtt.server, ip_buf, MQTT_FIELD_MAX - 1);
            g_mqtt.server[MQTT_FIELD_MAX - 1] = '\0';
            g_mqtt.port = port;
            config_save_mqtt();
            printf_debug("[SM] Primary IP set: %s:%d\r\n", g_mqtt.server, g_mqtt.port);
            EC801E_DisconnectMQTT();
            trea_delay_ms(500);
            EC801E_ConnectMQTT();
            EC801E_SubscribeTopic();
            if(EC801E_EnsureConnected() == EC_OK)
            {
                EC801E_PublishResponse("first_ip_set", cmd_val);
                trea_delay_ms(300);
                report_publish_feedback(2);
            }
        }
        else
        {
            if(EC801E_EnsureConnected() == EC_OK)
                report_publish_feedback(24);
        }
}

static void h_scend_ip_set(const char *id_val, const char *cmd_val, const char *raw_buf)
{
char ip_buf[MQTT_FIELD_MAX];
        uint16_t port;
        if(parse_ip_port(cmd_val, ip_buf, &port))
        {
            strncpy(g_mqtt.server2, ip_buf, MQTT_FIELD_MAX - 1);
            g_mqtt.server2[MQTT_FIELD_MAX - 1] = '\0';
            g_mqtt.port2 = port;
            config_save_mqtt();
            printf_debug("[SM] Secondary IP set: %s:%d\r\n", g_mqtt.server2, g_mqtt.port2);
            if(EC801E_EnsureConnected() == EC_OK)
            {
                EC801E_PublishResponse("scend_ip_set", cmd_val);
                trea_delay_ms(300);
                report_publish_feedback(2);
            }
        }
        else
        {
            if(EC801E_EnsureConnected() == EC_OK)
                report_publish_feedback(24);
        }
}

static void h_sensor_status(const char *id_val, const char *cmd_val, const char *raw_buf)
{
collect_all_sensors();
        {
            uint8_t cur_online = (g_sensor.raw.err == PT304D_OK) ? 1 : 0;
            if(cur_online != g_sensor.prev_online)
                g_sensor.prev_online = cur_online;
        }
        printf_debug("[SM] sensor_status queried\r\n");
        if(EC801E_EnsureConnected() == EC_OK)
            report_publish_sensor_status();
}

static void h_temp_status(const char *id_val, const char *cmd_val, const char *raw_buf)
{
collect_all_sensors();
        TempStatus_t ts = threshold_get_temp_status(g_sensor.raw.temperature);
        const char *ts_str = threshold_temp_status_str(ts);
        printf_debug("[SM] temp_status: T=%.2f -> %s\r\n", g_sensor.raw.temperature, ts_str);
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_str(&jb, "temp_status", ts_str, ts_buf);
            jb_add_float(&jb, "temp", g_sensor.raw.temperature, ts_buf);
            publish_json_only();
            trea_delay_ms(300);
            report_publish_feedback(31);   /* 温度状态获取成功 */
        }
}

static void h_ly06_get(const char *id_val, const char *cmd_val, const char *raw_buf)
{
char val_buf[8];
        float v = adc_bat_read_voltage();
        int volt_alarm = threshold_voltage_alarm(v);
        snprintf(val_buf, sizeof(val_buf), "%d", volt_alarm);
        printf_debug("[SM] LY06_get: V=%.2f -> %d\r\n", v, volt_alarm);
        if(EC801E_EnsureConnected() == EC_OK)
        {
            char v_buf[8];
            snprintf(v_buf, sizeof(v_buf), "%.2f", v);
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_str(&jb, "LY06", val_buf, ts_buf);
            jb_add_str(&jb, "voltage", v_buf, ts_buf);
            publish_json_only();
        }
}

//static void h_restart(const char *id_val, const char *cmd_val, const char *raw_buf)
//{
//printf_debug("[SM] Restart commanded\r\n");
//        if(EC801E_EnsureConnected() == EC_OK)
//            EC801E_PublishResponse("restart", "0");
//        trea_delay_ms(200);
//        NVIC_SystemReset();
//}

static void h_restart_off(const char *id_val, const char *cmd_val, const char *raw_buf)
{
printf_debug("[SM] Restart_off: closing valve then reset\r\n");
//        valve_execute("close", 1);
        if(EC801E_EnsureConnected() == EC_OK)
            EC801E_PublishResponse("restart", "0");
        trea_delay_ms(50);
        NVIC_SystemReset();
}

//static void h_4gpower_off(const char *id_val, const char *cmd_val, const char *raw_buf)
//{
//printf_debug("[SM] 4G power off\r\n");
//        report_4g_power_state(0);
//        trea_delay_ms(200);
//        EC801E_PowerOff();
//        fourG_user_off = 1;
//}

static void h_real_set(const char *id_val, const char *cmd_val, const char *raw_buf)
{
if(strcmp(id_val, "real_set") == 0 && cmd_val[0] != '\0')
        {
            g_real_enabled = (uint8_t)atoi(cmd_val);
            if(g_real_enabled > 1) g_real_enabled = 0;
            char freq_buf[8] = {0};
            extract_key_value(raw_buf, "real_frequency", freq_buf, sizeof(freq_buf));
            if(freq_buf[0])
            {
                g_real_frequency = (uint8_t)atoi(freq_buf);
                if(g_real_frequency < 1 || g_real_frequency > 10)
                    g_real_frequency = 5;
            }
            config_save_ext();
            threshold_alarm_reset_rt();   /* 切换实时模式: 清实时防重复状态 */
            printf_debug("[SM] real_set: enabled=%d frequency=%ds\r\n", g_real_enabled, g_real_frequency);
        }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_int(&jb, "real_enabled", g_real_enabled, ts_buf);
            jb_add_int(&jb, "real_frequency", g_real_frequency, ts_buf);
            publish_json_df(strcmp(id_val, "real_set") == 0 ? 14 : 13);
        }
}

static void h_lcd_switch(const char *id_val, const char *cmd_val, const char *raw_buf)
{
if(cmd_val[0] != '\0')
        {
            g_lcd_enabled = (uint8_t)atoi(cmd_val);
            if(g_lcd_enabled > 1) g_lcd_enabled = 1;
            if(!g_lcd_enabled) lcd_shutdown();
            config_save_ext();
            printf_debug("[SM] lcd_switch: %d (saved)\r\n", g_lcd_enabled);
        }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_int(&jb, "lcd_switch", g_lcd_enabled, ts_buf);
            publish_json_only();
        }
}

static void h_vpthreshold2(const char *id_val, const char *cmd_val, const char *raw_buf)
{
process_threshold(&thr_groups[4], cmd_val, raw_buf);
}

static void h_tthreshold2(const char *id_val, const char *cmd_val, const char *raw_buf)
{
process_threshold(&thr_groups[5], cmd_val, raw_buf);
}

static void h_temp_status2(const char *id_val, const char *cmd_val, const char *raw_buf)
{
collect_all_sensors();
        TempStatus_t ts2 = threshold_get_temp_status2(g_sensor.raw2.temperature);
        const char *ts2_str = threshold_temp_status_str(ts2);
        printf_debug("[SM] temp_status2: T2=%.2f -> %s\r\n", g_sensor.raw2.temperature, ts2_str);
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_str(&jb, "temp2_status", ts2_str, ts_buf);
            jb_add_float(&jb, "temp2", g_sensor.raw2.temperature, ts_buf);
            publish_json_only();
            trea_delay_ms(300);
            report_publish_feedback(31);   /* 温度状态获取成功 */
        }
}

static void h_get_pressure2(const char *id_val, const char *cmd_val, const char *raw_buf)
{
collect_all_sensors();
        PressureStatus_t ps2 = threshold_get_pressure_status2(g_sensor.raw2.pressure, -1.0f);
        const char *ps2_str = threshold_pressure_status_str(ps2);
        printf_debug("[SM] get_pressure2: P2=%.2f -> %s\r\n", g_sensor.raw2.pressure, ps2_str);
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_str(&jb, "pressure2_status", ps2_str, ts_buf);
            jb_add_float(&jb, "pressure2", g_sensor.raw2.pressure, ts_buf);
            jb_add_float(&jb, "temp2", g_sensor.raw2.temperature, ts_buf);
            publish_json_only();
            trea_delay_ms(300);
            report_publish_feedback(30);   /* 压力状态获取成功 */
        }
}

static void h_user_ip_get(const char *id_val, const char *cmd_val, const char *raw_buf)
{
report_publish_ip_field("user_ip", g_user_mqtt.server, g_user_mqtt.port, 1);
}

static void h_user_ip_set(const char *id_val, const char *cmd_val, const char *raw_buf)
{
char ip_buf[MQTT_FIELD_MAX];
        uint16_t port;
        if(parse_ip_port(cmd_val, ip_buf, &port))
        {
            strncpy(g_user_mqtt.server, ip_buf, MQTT_FIELD_MAX - 1);
            g_user_mqtt.server[MQTT_FIELD_MAX - 1] = '\0';
            g_user_mqtt.port = port;
            // topic为空时自动用client_id拼接
            if(g_user_mqtt.topic_up[0] == '\0' && g_user_mqtt.client_id[0])
                snprintf(g_user_mqtt.topic_up, MQTT_TOPIC_MAX, "/155/%s/property/post", g_user_mqtt.client_id);
            config_save_user_mqtt();
            printf_debug("[SM] User IP set: %s:%d\r\n", g_user_mqtt.server, g_user_mqtt.port);
            if(EC801E_EnsureConnected() == EC_OK)
            {
                EC801E_PublishResponse("user_ip_set", cmd_val);
                trea_delay_ms(300);
                report_publish_feedback(2);
            }
        }
        else
        {
            if(EC801E_EnsureConnected() == EC_OK)
                report_publish_feedback(24);
        }
}

static void h_user_mqtt_get(const char *id_val, const char *cmd_val, const char *raw_buf)
{
if(strcmp(id_val, "user_mqtt_set") == 0)
        {
            char tv[MQTT_TOPIC_MAX];
            uint8_t topic_explicit = 0;
            if(extract_key_value(raw_buf, "MQTT_username", tv, sizeof(tv)))
                strncpy(g_user_mqtt.username, tv, MQTT_FIELD_MAX - 1);
            if(extract_key_value(raw_buf, "MQTT_password", tv, sizeof(tv)))
                strncpy(g_user_mqtt.password, tv, MQTT_FIELD_MAX - 1);
            if(extract_key_value(raw_buf, "MQTT_subject", tv, sizeof(tv)))
            {
                strncpy(g_user_mqtt.topic_up, tv, MQTT_TOPIC_MAX - 1);
                topic_explicit = 1;
            }
            if(extract_key_value(raw_buf, "MQTT_client_id", tv, sizeof(tv)))
            {
                strncpy(g_user_mqtt.client_id, tv, MQTT_FIELD_MAX - 1);
                // 下发未显式指定topic且当前topic为空时, 自动用client_id拼接
                if(!topic_explicit && g_user_mqtt.topic_up[0] == '\0' && g_user_mqtt.server[0])
                    snprintf(g_user_mqtt.topic_up, MQTT_TOPIC_MAX, "/155/%s/property/post", g_user_mqtt.client_id);
            }
            config_save_user_mqtt();
            printf_debug("[SM] User MQTT set: user=%s client=%s topic=%s\r\n",
                         g_user_mqtt.username, g_user_mqtt.client_id, g_user_mqtt.topic_up);
        }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_str(&jb, "MQTT_username", g_user_mqtt.username, ts_buf);
            jb_add_str(&jb, "MQTT_password", g_user_mqtt.password, ts_buf);
            jb_add_str(&jb, "MQTT_client_id", g_user_mqtt.client_id, ts_buf);
            jb_add_str(&jb, "MQTT_subject", g_user_mqtt.topic_up, ts_buf);
            publish_json_df(strcmp(id_val, "user_mqtt_set") == 0 ? 18 : 17);
        }
}

static void h_valve_status(const char *id_val, const char *cmd_val, const char *raw_buf)
{
const char *vs = valve_get_status();
        printf_debug("[SM] valve_get: %s\r\n", vs);
        if(EC801E_EnsureConnected() == EC_OK)
        {
            EC801E_PublishResponse("valve_control", vs);
            trea_delay_ms(300);
            report_publish_feedback(25);
        }
}

static void h_closeoropen(const char *id_val, const char *cmd_val, const char *raw_buf)
{
// ---- 定时开关阀 set/get (YYMMDDHHMM, 开/关开关独立) ----
        if(cmd_val[0] != '\0')
        {
            char tv[16];
            char new_open[11], new_close[11];
            uint8_t new_on  = g_valve_timing.on_switch;
            uint8_t new_off = g_valve_timing.off_switch;
            uint8_t changed = 0;

            if(extract_key_value(raw_buf, "timing_onswitch", tv, sizeof(tv)))
            { new_on = (uint8_t)atoi(tv); if(new_on > 1) new_on = 0; changed = 1; }
            if(extract_key_value(raw_buf, "timing_offswitch", tv, sizeof(tv)))
            { new_off = (uint8_t)atoi(tv); if(new_off > 1) new_off = 0; changed = 1; }

            strncpy(new_open,  g_valve_timing.open_time,  10); new_open[10]  = 0;
            strncpy(new_close, g_valve_timing.close_time, 10); new_close[10] = 0;

            if(extract_key_value(raw_buf, "timing_open", tv, sizeof(tv)))
            {
                strncpy(new_open, tv, 10); new_open[10] = 0;
                if(valve_timing_time_valid(new_open)) changed = 1;
                else
                {
                    strncpy(new_open, g_valve_timing.open_time, 10); new_open[10] = 0;
                    printf_debug("[SM] closeoropen: invalid timing_open=%s rejected\r\n", tv);
                }
            }
            if(extract_key_value(raw_buf, "timing_close", tv, sizeof(tv)))
            {
                strncpy(new_close, tv, 10); new_close[10] = 0;
                if(valve_timing_time_valid(new_close)) changed = 1;
                else
                {
                    strncpy(new_close, g_valve_timing.close_time, 10); new_close[10] = 0;
                    printf_debug("[SM] closeoropen: invalid timing_close=%s rejected\r\n", tv);
                }
            }

            if(changed)
            {
                g_valve_timing.on_switch  = new_on;
                g_valve_timing.off_switch = new_off;
                strncpy(g_valve_timing.open_time,  new_open,  10); g_valve_timing.open_time[10]  = 0;
                strncpy(g_valve_timing.close_time, new_close, 10); g_valve_timing.close_time[10] = 0;
                config_save_valve_timing();
                timing_open_done_today  = 0;
                timing_close_done_today = 0;
            }
            printf_debug("[SM] closeoropen_set: on=%d off=%d open=%s close=%s (flags reset)\r\n",
                         g_valve_timing.on_switch, g_valve_timing.off_switch,
                         g_valve_timing.open_time, g_valve_timing.close_time);
        }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            char v_str[4];
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            snprintf(v_str, sizeof(v_str), "%d", g_valve_timing.on_switch);
            jb_add_str(&jb, "timing_onswitch", v_str, ts_buf);
            snprintf(v_str, sizeof(v_str), "%d", g_valve_timing.off_switch);
            jb_add_str(&jb, "timing_offswitch", v_str, ts_buf);
            jb_add_str(&jb, "timing_open", g_valve_timing.open_time, ts_buf);
            jb_add_str(&jb, "timing_close", g_valve_timing.close_time, ts_buf);
            publish_json_df(cmd_val[0] != '\0' ? 22 : 21);
        }
}

static void h_table(const char *id_val, const char *cmd_val, const char *raw_buf)
{
// ---- 表号 get/set ----
        if(cmd_val[0] != '\0')
        {
            // SET: 替换 client_id 中的第2段(设备ID/表号)
            char new_id[MQTT_FIELD_MAX];
            const char *src = g_mqtt.client_id;
            const char *amp1 = strchr(src, '&');
            if(amp1)
            {
                const char *amp2 = strchr(amp1 + 1, '&');
                if(amp2)
                {
                    uint16_t prefix_len = (uint16_t)(amp1 - src + 1);
                    memcpy(new_id, src, prefix_len);
                    new_id[prefix_len] = '\0';
                    strncat(new_id, cmd_val, MQTT_FIELD_MAX - prefix_len - 1);
                    strncat(new_id, amp2, MQTT_FIELD_MAX - strlen(new_id) - 1);
                    new_id[MQTT_FIELD_MAX - 1] = '\0';
                    strncpy(g_mqtt.client_id, new_id, MQTT_FIELD_MAX - 1);
                    g_mqtt.client_id[MQTT_FIELD_MAX - 1] = '\0';
                    derive_topic_from_client_id();
                    config_save_mqtt();
                    printf_debug("[SM] table set: %s\r\n", g_mqtt.client_id);
                }
            }
        }
        // 上报当前表号 (读取client_id第2段)
        if(EC801E_EnsureConnected() == EC_OK)
        {
            char table_val[MQTT_FIELD_MAX] = {0};
            const char *a1 = strchr(g_mqtt.client_id, '&');
            if(a1)
            {
                const char *a2 = strchr(a1 + 1, '&');
                uint16_t len = a2 ? (uint16_t)(a2 - a1 - 1) : (uint16_t)strlen(a1 + 1);
                if(len >= MQTT_FIELD_MAX) len = MQTT_FIELD_MAX - 1;
                memcpy(table_val, a1 + 1, len);
                table_val[len] = '\0';
            }
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_str(&jb, "table", table_val, ts_buf);
            publish_json_df(cmd_val[0] != '\0' ? 27 : 26);
        }
}

/* ==================== 指令分发表 ==================== */
typedef struct {
    const char *id;                                              /* 指令 id (格式A) 或 CmdMap 映射后的 id */
    void (*handler)(const char *id_val, const char *cmd_val, const char *raw_buf);
} CmdHandler_t;

static const CmdHandler_t s_cmd_handlers[] = {
    { "valve", h_valve },
    { "sensor_read", h_sensor_read },
    { "get_pressure", h_sensor_read },
    { "collect_cycle", h_collect_cycle },
    { "report_cycle", h_report_cycle },
    { "vpthreshold", h_vpthreshold },
    { "Tthreshold", h_tthreshold },
    { "spupower", h_spupower },
    { "threshold", h_threshold },
    { "delay", h_delay },
    { "window_pattern", h_window_pattern },
    { "abnormal_close", h_abnormal_close },
    { "abnormal2_close", h_abnormal2_close },
    { "abnormal3_close", h_abnormal3_close },
    { "valve_last_order", h_valve_last_order },
    { "keepalive", h_keepalive },
    { "get_sim", h_get_sim },
    { "get_ip", h_get_ip },
    { "get_scend_ip", h_get_scend_ip },
    { "first_ip_set", h_first_ip_set },
    { "scend_ip_set", h_scend_ip_set },
    { "sensor_status", h_sensor_status },
    { "temp_status", h_temp_status },
    { "LY06_get", h_ly06_get },
    { "restart", h_restart_off },
    { "restart_off", h_restart_off },
//    { "4Gpower_off", h_4gpower_off },
    { "real_set", h_real_set },
    { "real_get", h_real_set },
    { "lcd_switch", h_lcd_switch },
    { "vpthreshold2", h_vpthreshold2 },
    { "Tthreshold2", h_tthreshold2 },
    { "temp_status2", h_temp_status2 },
    { "get_pressure2", h_get_pressure2 },
    { "gas_get", h_gas_get },
    { "laser_get", h_laser_get },
    { "laser_set", h_laser_set },
    { "msh_get", h_msh_get },
    { "msh_set", h_msh_set },
    { "user_ip_get", h_user_ip_get },
    { "user_ip_set", h_user_ip_set },
    { "user_mqtt_get", h_user_mqtt_get },
    { "user_mqtt_set", h_user_mqtt_get },
    { "valve_status", h_valve_status },
    { "closeoropen", h_closeoropen },
    { "table", h_table },
};
#define CMD_HANDLER_COUNT (sizeof(s_cmd_handlers) / sizeof(s_cmd_handlers[0]))

/* 处理单条下行指令 (表驱动分发) */
static void process_downlink_cmd(const char *id_val, const char *cmd_val, const char *raw_buf)
{
    uint8_t i;
    for(i = 0; i < CMD_HANDLER_COUNT; i++)
    {
        if(strcmp(id_val, s_cmd_handlers[i].id) == 0)
        {
            s_cmd_handlers[i].handler(id_val, cmd_val, raw_buf);
            return;
        }
    }
    printf_debug("[SM] Unknown cmd: %s\r\n", id_val);
}
// ==================== 排空处理所有MQTT下行消息 ====================
/* ==================== 下行报文解析 ==================== */
typedef enum {
    CMD_KIND_NORMAL = 0,
    CMD_KIND_OTA
} CmdKind_t;

typedef struct {
    char id[32];
    char value[192];
} CmdRequest_t;

/* 解析单条下行报文: OTA检测 + 格式B(commandStr映射) + 格式A(id/value) 归一 */
static CmdKind_t parse_cmd(const char *raw, CmdRequest_t *req)
{
    char cs[64] = {0};

    req->id[0] = '\0';
    req->value[0] = '\0';

    /* OTA 升级指令 */
    if(strstr(raw, "downloadUrl") != NULL)
        return CMD_KIND_OTA;

    /* 格式B: commandStr 查映射表 */
    if(extract_json_field(raw, "\"commandStr\"", cs, sizeof(cs)))
    {
        uint8_t i;
        uint8_t found = 0;
        for(i = 0; i < CMD_MAP_COUNT; i++)
        {
            if(strcmp(cs, s_cmd_map[i].cmd_str) == 0)
            {
                strcpy(req->id, s_cmd_map[i].id);
                if(s_cmd_map[i].value != NULL)
                    strcpy(req->value, s_cmd_map[i].value);
                else
                    extract_key_value(raw, s_cmd_map[i].extract, req->value, sizeof(req->value));
                found = 1;
                break;
            }
        }
        if(!found)
        {
            strncpy(req->id, cs, sizeof(req->id) - 1);
            req->id[sizeof(req->id) - 1] = '\0';
        }
        return CMD_KIND_NORMAL;
    }

    /* 格式A: 直接取 id / value */
    extract_json_field(raw, "\"id\"", req->id, sizeof(req->id));
    extract_json_field(raw, "\"value\"", req->value, sizeof(req->value));
    return CMD_KIND_NORMAL;
}

// ==================== 排空处理所有MQTT下行消息 ====================
uint8_t drain_pending_downlink(void)
{
    static char dl_buf[1024];
    CmdRequest_t req;
    uint8_t msg_count = 0;

    while(msg_count < 5)
    {
        if(EC801E_ReadDownlink(dl_buf, sizeof(dl_buf)) != EC_OK)
            break;

        msg_count++;

        if(parse_cmd(dl_buf, &req) == CMD_KIND_OTA)
        {
            printf_debug("[SM] OTA upgrade cmd detected\r\n");
            ota_app_handle_upgrade_cmd(dl_buf);
            continue;
        }

        printf_debug("[SM] CMD id=%s value=%s\r\n", req.id, req.value);
        process_downlink_cmd(req.id, req.value, dl_buf);
    }

    return msg_count;
}
