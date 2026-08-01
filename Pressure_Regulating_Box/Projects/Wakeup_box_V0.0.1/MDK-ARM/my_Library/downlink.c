/**
 * @file downlink.c
 * @brief MQTT下发指令处理模块 (优化版)
 * @details 负责接收并解析平台下发的MQTT消息, 执行对应指令并上报结果
 */

#include "downlink.h"
#include "ec800e.h"
#include "eeprom_config.h"
#include "app_config.h"
#include "threshold.h"
#include "sensor_buffer.h"
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

// ==================== 跨模块变量访问 (定义在main.c) ====================
extern uint8_t fourG_user_off;
extern ThresholdResult_t last_realtime_alarm;
extern uint8_t g_need_adc_sampling;

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

// ==================== commandStr 查表映射 ====================
typedef struct {
    const char *cmd_str;     // commandStr 值
    const char *id;          // 映射到 id_val
    const char *value;       // 固定值, NULL 表示需从 raw_buf 提取
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
    {"get_pressure2",     "get_pressure2",   "1",     NULL},
    {"threshold_get1",    "threshold",       "",      NULL},
    {"collectcycle_get",  "collect_cycle",   "",      NULL},
    {"report_get",        "report_cycle",    "",      NULL},
    {"window_get",        "window_pattern",  "",      NULL},
    {"abnormal_get",      "abnormal_close",  "",      NULL},
    {"first_ip_get",      "get_ip",          "1",     NULL},
    {"scend_ip_get",      "get_scend_ip",    "1",     NULL},
    {"valve_last_get",    "valve_last_order","1",     NULL},
    {"real_get",          "real_get",        "",      NULL},
    {"user_ip_get",       "user_ip_get",     "1",     NULL},
    {"MQTT_get",          "user_mqtt_get",   "1",     NULL},
    // 带参数提取的命令
    {"collectspeed_set",  "collect_cycle",   NULL,    "collect_once\",\"value\""},
    {"report_set",        "report_cycle",    NULL,    "report_once\",\"value\""},
    {"window_set",        "window_pattern",  NULL,    "window_pattern\",\"value\""},
    {"abnormal_set",      "abnormal_close",  NULL,    "abnormal_close\",\"value\""},
    {"first_ip_set",      "first_ip_set",    NULL,    "first_ip\",\"value\""},
    {"scend_ip_set",      "scend_ip_set",    NULL,    "scend_ip\",\"value\""},
    {"real_set",          "real_set",        NULL,    "real_enabled\",\"value\""},
    {"user_ip_set",       "user_ip_set",     NULL,    "user_ip\",\"value\""},
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
    {"lcd_switch",        "lcd_switch",      NULL,    "lcd_switch\",\"value\""},
    {"sensor_count_set",  "sensor_count_set",NULL,    "sensor_count\",\"value\""},
    // 阀门定时开关
    {"valve_get",         "valve_status",    "1",     NULL},
    {"closeoropen_get",   "closeoropen",     "",      NULL},
    {"closeoropen_set",   "closeoropen",     "1",     NULL},
};
#define CMD_MAP_COUNT (sizeof(s_cmd_map) / sizeof(s_cmd_map[0]))

// ==================== 传感器状态上报 (公共函数, 仅rtu在线状态) ====================
void publish_sensor_status(void)
{
    static char s_json[360];
    static char s_ts[32];
    JsonBuilder jb;

    const char *have    = (g_sensor.raw.err != PT304D_ERR_I2C) ? "1" : "0";
    const char *rstatus = (g_sensor.raw.err == PT304D_OK) ? "1" : "0";

    rtc_get_time_str(s_ts, sizeof(s_ts));
    jb_begin(&jb, s_json, sizeof(s_json));
    jb_add_str(&jb, "rtu_have", have, s_ts);
    jb_add_str(&jb, "rtu_status", rstatus, s_ts);

    if(g_sensor_count >= 2)
    {
        const char *have2    = (g_sensor.raw2.err != PT304D_ERR_I2C) ? "1" : "0";
        const char *rstatus2 = (g_sensor.raw2.err == PT304D_OK) ? "1" : "0";
        jb_add_str(&jb, "rtu2_have", have2, s_ts);
        jb_add_str(&jb, "rtu2_status", rstatus2, s_ts);
    }

    jb_end(&jb);
    EC801E_PublishRawJson(s_json);
}

// ==================== 上报汇总 (历史数据上报完成后调用) ====================
// 合并传感器在线状态 + 信号 + 压力温度状态 + 阀门 + IP + 异常关阀 + 电压
void publish_upload_summary(void)
{
    static char s_json[800];
    static char s_ts[32];
    static char csq_buf[8];
    static char v_buf[8];
    static char va_buf[4];
	static volatile char user_state = 0;
    JsonBuilder jb;
	
	
	// 1. 传感器在线状态
	publish_sensor_status();
    rtc_get_time_str(s_ts, sizeof(s_ts));
    jb_begin(&jb, s_json, sizeof(s_json));

    // 2. 信号强度
    csq_buf[0] = '\0';
    EC801E_QueryCSQ(csq_buf, sizeof(csq_buf));
    jb_add_str(&jb, "csq", csq_buf, s_ts);

    // 4. 压力+温度状态
    {
        PressureStatus_t ps = threshold_get_pressure_status(g_sensor.raw.pressure, g_sensor.last_pressure_collect);
        jb_add_str(&jb, "pressure_status", threshold_pressure_status_str(ps), s_ts);
        TempStatus_t ts = threshold_get_temp_status(g_sensor.raw.temperature);
        jb_add_str(&jb, "temp_status", threshold_temp_status_str(ts), s_ts);

        if(g_sensor_count >= 2 && g_sensor.raw2.err == PT304D_OK)
        {
            PressureStatus_t ps2 = threshold_get_pressure_status2(g_sensor.raw2.pressure, g_sensor.last_pressure_collect2);
            jb_add_str(&jb, "pressure2_status", threshold_pressure_status_str(ps2), s_ts);
            TempStatus_t ts2 = threshold_get_temp_status2(g_sensor.raw2.temperature);
            jb_add_str(&jb, "temp2_status", threshold_temp_status_str(ts2), s_ts);
        }
    }
	jb_end(&jb);
    EC801E_PublishRawJson(s_json);
	
		// 7. 用户端mqtt配置
	if(!user_state)
	{
		user_state = 1;
		jb_begin(&jb, s_json, sizeof(s_json));
		jb_add_str(&jb, "MQTT_username", g_user_mqtt.username, s_ts);
		jb_add_str(&jb, "MQTT_password", g_user_mqtt.password, s_ts);
		jb_add_str(&jb, "MQTT_client_id", g_user_mqtt.client_id, s_ts);
		jb_add_str(&jb, "MQTT_subject", g_user_mqtt.topic_up, s_ts);

		jb_add_str(&jb, "window_pattern", g_window.pattern, s_ts);
		jb_add_int(&jb, "window_end_result", g_window.end_result, s_ts);
		jb_add_str(&jb, "window_start", g_window.start, s_ts);
		jb_add_str(&jb, "window_stop", g_window.stop, s_ts);
		jb_add_int(&jb, "window_dottime", g_window.dottime, s_ts);
		
		jb_end(&jb);
		EC801E_PublishRawJson(s_json);
	}
	
	jb_begin(&jb, s_json, sizeof(s_json));
	 // 5. 阀门状态
    {
		const char *valve_status = valve_get_status();
        jb_add_str(&jb, "valve_control", valve_status, s_ts);
    }
	
    // 6. 用户端IP / 主IP / 副IP
	{
		jb_add_str(&jb, "user_ip", g_user_mqtt.server, s_ts);
		jb_add_str(&jb, "first_ip", g_mqtt.server, s_ts);
		jb_add_str(&jb, "scend_ip", g_mqtt.server2, s_ts);
	}


	
    // 8. 异常自动关阀
    jb_add_str(&jb, "abnormal_close", g_abnormal_close, s_ts);

    // 9. 电压数值 + 电压状态
    {
        float bat_v = adc_bat_read_voltage();
        if(bat_v >= 0.0f)
        {
            g_sensor.voltage = bat_v;
            g_sensor.last_normal_voltage = bat_v;
        }
        snprintf(v_buf, sizeof(v_buf), "%.2f", g_sensor.voltage);
        jb_add_str(&jb, "voltage", v_buf, s_ts);

        int volt_alarm = threshold_voltage_alarm(g_sensor.voltage);
        snprintf(va_buf, sizeof(va_buf), "%d", volt_alarm);
        jb_add_str(&jb, "LY06", va_buf, s_ts);
    }

    jb_end(&jb);
    EC801E_PublishRawJson(s_json);
    printf_debug("[SM] upload summary:\r\n CSQ=%s V=%s\r\n", csq_buf, v_buf);
}

// ==================== 指令反馈上报 ====================
static void publish_feedback(int code)
{
    static char fb_json[128];
    static char fb_ts[32];
    rtc_get_time_str(fb_ts, sizeof(fb_ts));
    snprintf(fb_json, sizeof(fb_json),
        "[{\"id\":\"feedback\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"}]", code, fb_ts);
    EC801E_PublishRawJson(fb_json);
}

// ==================== 处理单条下发指令 ====================
static void process_downlink_cmd(const char *id_val, const char *cmd_val, const char *raw_buf)
{
    static char json_buf[1024];
    static char ts_buf[32];
    JsonBuilder jb;

    if(strcmp(id_val, "valve") == 0)
    {
        // ---- 阀门控制 ----
        const char *result = valve_execute(cmd_val, 1);
        printf_debug("[SM] Valve result: %s\r\n", result);

        // 用户主动操作清除报警恢复标志, 避免后续传感器恢复时卷回用户动作
        if(g_valve_alarm_closed)
        {
            g_valve_alarm_closed = 0;
            g_valve_pre_alarm_order = g_valve_last_order;
            config_save_ext();
            printf_debug("[SM] Valve user override: cleared alarm_closed flag\r\n");
        }

        if(EC801E_EnsureConnected() == EC_OK)
        {
            EC801E_PublishValveStatus(result, g_valve_last_order);
            trea_delay_ms(300);
            publish_feedback(23);
        }
        else
            printf_debug("[SM] Valve publish skipped: MQTT not connected\r\n");
    }
    else if(strcmp(id_val, "sensor_read") == 0 || strcmp(id_val, "get_pressure") == 0)
    {
        // ---- 立即采集+阈值校验+压力状态上报 ----
        PressureStatus_t ps;
        const char *ps_str;

        PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);
        {
            float v_now = adc_bat_read_voltage();
            g_sensor.voltage = (v_now >= 0.0f) ? v_now : g_sensor.last_normal_voltage;
        }
        printf_debug("[SM] sensor_read: P=%.2f T=%.2f V=%.2f err=%d\r\n",
                     g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage, g_sensor.raw.err);

        // 传感器状态上报 (不修改消抖状态, 留给 do_collect/do_realtime_check 的 sensor_debounce_update)
        if(EC801E_EnsureConnected() == EC_OK)
            publish_sensor_status();

        // 压力状态判断 + 上报 (更新基线防止连续 sensor_read 突变误判)
        ps = threshold_get_pressure_status(g_sensor.raw.pressure, g_sensor.last_pressure_collect);
        ps_str = threshold_pressure_status_str(ps);
        g_sensor.last_pressure_collect = g_sensor.raw.pressure;

        if(EC801E_EnsureConnected() == EC_OK)
        {
            EC801E_PublishSensorData(&g_sensor.raw, g_sensor.voltage);
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_str(&jb, "pressure_status", ps_str, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
        }

        // 温度状态判断 + 上报
//        {
//            TempStatus_t t_s = threshold_get_temp_status(g_sensor.raw.temperature);
//            const char *t_str = threshold_temp_status_str(t_s);
//            if(EC801E_EnsureConnected() == EC_OK)
//            {
//                rtc_get_time_str(ts_buf, sizeof(ts_buf));
//                jb_begin(&jb, json_buf, sizeof(json_buf));
//                jb_add_str(&jb, "temp_status", t_str, ts_buf);
//                jb_end(&jb);
//                EC801E_PublishRawJson(json_buf);
//            }
//        }

        // 阈值检查 + 异常自动关阀 (复用公共模块)
        {
            ThresholdResult_t tr = threshold_check(g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage, g_sensor.last_pressure_collect, g_need_adc_sampling, 0);
            if(tr != THRESHOLD_OK)
            {
                printf_debug("[SM] sensor_read ALARM: %s\r\n", threshold_alarm_type_str(tr));
                if(EC801E_EnsureConnected() == EC_OK)
                    EC801E_PublishAlarmStatus(tr, g_sensor.voltage);

                const char *vr = alarm_try_close_valve(tr);
                if(vr && EC801E_EnsureConnected() == EC_OK)
                    EC801E_PublishValveStatus(vr, g_valve_last_order);
            }
        }
    }
    else if(strcmp(id_val, "collect_cycle") == 0)
    {
        if(cmd_val[0] != '\0')
        {
            char type_buf[16] = {0};
            g_collect_cycle.once = (uint16_t)atoi(cmd_val);
            extract_json_field(raw_buf, "\"type\"", type_buf, sizeof(type_buf));
            if(!type_buf[0]) extract_json_field(raw_buf, "collect_type\",\"value\"", type_buf, sizeof(type_buf));
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
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            publish_feedback(cmd_val[0] != '\0' ? 12 : 11);
        }
    }
    else if(strcmp(id_val, "report_cycle") == 0)
    {
        if(cmd_val[0] != '\0')
        {
            char type_buf[16] = {0};
            g_report_cycle.once = (uint16_t)atoi(cmd_val);
            extract_json_field(raw_buf, "\"type\"", type_buf, sizeof(type_buf));
            if(!type_buf[0]) extract_json_field(raw_buf, "report_type\",\"value\"", type_buf, sizeof(type_buf));
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
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            publish_feedback(cmd_val[0] != '\0' ? 10 : 9);
        }
    }
    else if(strcmp(id_val, "vpthreshold") == 0)
    {
        char tv[16];
        int changed = 0;
        if(extract_json_field(raw_buf, "vpthreshold_switch\",\"value\"", tv, sizeof(tv)))
        { g_pthreshold.enabled = (uint8_t)atoi(tv); if(g_pthreshold.enabled > 1) g_pthreshold.enabled = 0; changed = 1; }
        if(extract_json_field(raw_buf, "vpthreshold_ppmax\",\"value\"", tv, sizeof(tv)))
        { g_pthreshold.ppmax = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "vpthreshold_pmax\",\"value\"", tv, sizeof(tv)))
        { g_pthreshold.pmax = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "vpthreshold_ppmin\",\"value\"", tv, sizeof(tv)))
        { g_pthreshold.ppmin = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "vpthreshold_pmin\",\"value\"", tv, sizeof(tv)))
        { g_pthreshold.pmin = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "vpthreshold_mutation\",\"value\"", tv, sizeof(tv)))
        { g_pthreshold.mutation = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "vpthreshold_shock\",\"value\"", tv, sizeof(tv)))
        { g_pthreshold.shock = (float)atof(tv); changed = 1; }
        if(changed) { config_save_vpthreshold(); printf_debug("[SM] vpthreshold updated\r\n"); }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_int(&jb, "vpthreshold_switch", g_pthreshold.enabled, ts_buf);
            jb_add_float(&jb, "vpthreshold_ppmax", g_pthreshold.ppmax, ts_buf);
            jb_add_float(&jb, "vpthreshold_pmax", g_pthreshold.pmax, ts_buf);
            jb_add_float(&jb, "vpthreshold_ppmin", g_pthreshold.ppmin, ts_buf);
            jb_add_float(&jb, "vpthreshold_pmin", g_pthreshold.pmin, ts_buf);
            jb_add_float(&jb, "vpthreshold_mutation", g_pthreshold.mutation, ts_buf);
            jb_add_float(&jb, "vpthreshold_shock", g_pthreshold.shock, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            publish_feedback(changed ? 8 : 7);
        }
    }
    else if(strcmp(id_val, "Tthreshold") == 0)
    {
        char tv[16];
        int changed = 0;
        if(extract_json_field(raw_buf, "Tthreshold_switch\",\"value\"", tv, sizeof(tv)))
        { g_tthreshold.enabled = (uint8_t)atoi(tv); if(g_tthreshold.enabled > 1) g_tthreshold.enabled = 0; changed = 1; }
        if(extract_json_field(raw_buf, "Tthreshold_high\",\"value\"", tv, sizeof(tv)))
        { g_tthreshold.high = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "Tthreshold_low\",\"value\"", tv, sizeof(tv)))
        { g_tthreshold.low = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "Tthreshold_vhigh\",\"value\"", tv, sizeof(tv)))
        { g_tthreshold.vhigh = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "Tthreshold_vlow\",\"value\"", tv, sizeof(tv)))
        { g_tthreshold.vlow = (float)atof(tv); changed = 1; }
        if(changed) { config_save_tthreshold(); printf_debug("[SM] Tthreshold updated\r\n"); }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_int(&jb, "Tthreshold_switch", g_tthreshold.enabled, ts_buf);
            jb_add_float(&jb, "Tthreshold_high", g_tthreshold.high, ts_buf);
            jb_add_float(&jb, "Tthreshold_low", g_tthreshold.low, ts_buf);
            jb_add_float(&jb, "Tthreshold_vhigh", g_tthreshold.vhigh, ts_buf);
            jb_add_float(&jb, "Tthreshold_vlow", g_tthreshold.vlow, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            publish_feedback(changed ? 6 : 5);
        }
    }
    else if(strcmp(id_val, "spupower") == 0)
    {
        char tv[16];
        int changed = 0;
        if(extract_json_field(raw_buf, "spupower_switch\",\"value\"", tv, sizeof(tv)))
        { g_spupower.enabled = (uint8_t)atoi(tv); if(g_spupower.enabled > 1) g_spupower.enabled = 0; changed = 1; }
        if(extract_json_field(raw_buf, "spupower_high\",\"value\"", tv, sizeof(tv)))
        { g_spupower.high = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "spupower_low\",\"value\"", tv, sizeof(tv)))
        { g_spupower.low = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "spupower_protect\",\"value\"", tv, sizeof(tv)))
        { g_spupower.protect = (float)atof(tv); changed = 1; }
        if(changed) { config_save_spupower(); printf_debug("[SM] spupower updated\r\n"); }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_int(&jb, "spupower_switch", g_spupower.enabled, ts_buf);
            jb_add_float(&jb, "spupower_high", g_spupower.high, ts_buf);
            jb_add_float(&jb, "spupower_low", g_spupower.low, ts_buf);
            jb_add_float(&jb, "spupower_protect", g_spupower.protect, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            publish_feedback(changed ? 16 : 15);
        }
    }
    else if(strcmp(id_val, "threshold") == 0)
    {
        // 兼容旧版 threshold_get1: 依次回复三组
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_int(&jb, "vpthreshold_switch", g_pthreshold.enabled, ts_buf);
            jb_add_float(&jb, "vpthreshold_ppmax", g_pthreshold.ppmax, ts_buf);
            jb_add_float(&jb, "vpthreshold_pmax", g_pthreshold.pmax, ts_buf);
            jb_add_float(&jb, "vpthreshold_ppmin", g_pthreshold.ppmin, ts_buf);
            jb_add_float(&jb, "vpthreshold_pmin", g_pthreshold.pmin, ts_buf);
            jb_add_float(&jb, "vpthreshold_mutation", g_pthreshold.mutation, ts_buf);
            jb_add_float(&jb, "vpthreshold_shock", g_pthreshold.shock, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_int(&jb, "Tthreshold_switch", g_tthreshold.enabled, ts_buf);
            jb_add_float(&jb, "Tthreshold_high", g_tthreshold.high, ts_buf);
            jb_add_float(&jb, "Tthreshold_low", g_tthreshold.low, ts_buf);
            jb_add_float(&jb, "Tthreshold_vhigh", g_tthreshold.vhigh, ts_buf);
            jb_add_float(&jb, "Tthreshold_vlow", g_tthreshold.vlow, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_int(&jb, "spupower_switch", g_spupower.enabled, ts_buf);
            jb_add_float(&jb, "spupower_high", g_spupower.high, ts_buf);
            jb_add_float(&jb, "spupower_low", g_spupower.low, ts_buf);
            jb_add_float(&jb, "spupower_protect", g_spupower.protect, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            publish_feedback(5);
        }
    }
    else if(strcmp(id_val, "delay") == 0)
    {
        char tv[16];
        int changed = 0;
        if(extract_json_field(raw_buf, "delay_switch\",\"value\"", tv, sizeof(tv)))
        { g_vdelay.enabled = (uint8_t)atoi(tv); if(g_vdelay.enabled > 1) g_vdelay.enabled = 0; changed = 1; }
        if(extract_json_field(raw_buf, "delay_time\",\"value\"", tv, sizeof(tv)))
        {
            uint32_t v = (uint32_t)atoi(tv);
            if(v > VDLY_DELAY_MS_MAX) v = VDLY_DELAY_MS_MAX;
            g_vdelay.delay_ms = (uint16_t)v;
            changed = 1;
        }
        if(changed) { config_save_vdelay(); printf_debug("[SM] delay updated: en=%d ms=%d\r\n", g_vdelay.enabled, g_vdelay.delay_ms); }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_int(&jb, "delay_switch", g_vdelay.enabled, ts_buf);
            jb_add_int(&jb, "delay_time", g_vdelay.delay_ms, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            publish_feedback(changed ? 20 : 19);
        }
    }
    else if(strcmp(id_val, "window_pattern") == 0)
    {
        if(cmd_val[0] != '\0')
        {
            strncpy(g_window.pattern, cmd_val, 7); g_window.pattern[7] = '\0';
            char tmp[16] = {0};
            extract_json_field(raw_buf, "\"end_result\"", tmp, sizeof(tmp));
            if(!tmp[0]) extract_json_field(raw_buf, "window_end_result\",\"value\"", tmp, sizeof(tmp));
            if(tmp[0]) g_window.end_result = (uint8_t)atoi(tmp);
            tmp[0] = '\0';
            extract_json_field(raw_buf, "\"start\"", tmp, sizeof(tmp));
            if(!tmp[0]) extract_json_field(raw_buf, "window_start\",\"value\"", tmp, sizeof(tmp));
            if(tmp[0]) { strncpy(g_window.start, tmp, 7); g_window.start[7] = '\0'; }
            tmp[0] = '\0';
            extract_json_field(raw_buf, "\"stop\"", tmp, sizeof(tmp));
            if(!tmp[0]) extract_json_field(raw_buf, "window_stop\",\"value\"", tmp, sizeof(tmp));
            if(tmp[0]) { strncpy(g_window.stop, tmp, 7); g_window.stop[7] = '\0'; }
            tmp[0] = '\0';
            extract_json_field(raw_buf, "\"dottime\"", tmp, sizeof(tmp));
            if(!tmp[0]) extract_json_field(raw_buf, "window_dottime\",\"value\"", tmp, sizeof(tmp));
            if(tmp[0]) g_window.dottime = (uint8_t)atoi(tmp);
            config_save_ext();
            printf_debug("[SM] window: %s er=%d s=%s e=%s dot=%d\r\n",
                         g_window.pattern, g_window.end_result, g_window.start, g_window.stop, g_window.dottime);
        }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_str(&jb, "window_pattern", g_window.pattern, ts_buf);
            jb_add_int(&jb, "window_end_result", g_window.end_result, ts_buf);
            jb_add_str(&jb, "window_start", g_window.start, ts_buf);
            jb_add_str(&jb, "window_stop", g_window.stop, ts_buf);
            jb_add_int(&jb, "window_dottime", g_window.dottime, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            publish_feedback(cmd_val[0] != '\0' ? 4 : 3);
        }
    }
    else if(strcmp(id_val, "abnormal_close") == 0)
    {
        // ---- 异常自动关阀策略 (cmd_val非空=设置, 空=查询) ----
        if(cmd_val[0] != '\0')
        {
            strncpy(g_abnormal_close, cmd_val, 15); g_abnormal_close[15] = '\0';
            config_save_ext();
            printf_debug("[SM] abnormal_close -> %s\r\n", g_abnormal_close);
        }
        if(EC801E_EnsureConnected() == EC_OK)
            EC801E_PublishResponse("abnormal_close", g_abnormal_close);
    }
    else if(strcmp(id_val, "valve_last_order") == 0)
    {
        // ---- 查询/上报最后阀门指令 ----
        char v[4];
        snprintf(v, sizeof(v), "%d", g_valve_last_order);
        if(EC801E_EnsureConnected() == EC_OK)
            EC801E_PublishResponse("valve_last_order", v);
    }
    else if(strcmp(id_val, "keepalive") == 0)
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
    else if(strcmp(id_val, "get_sim") == 0)
    {
        char iccid[32] = {0}, csq[8] = {0}, imei_buf[24] = {0};
        EC801E_QueryCSQ(csq, sizeof(csq));
        EC801E_QueryICCID(iccid, sizeof(iccid));
        EC801E_QueryIMEI(imei_buf, sizeof(imei_buf));
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_str(&jb, "csq", csq, ts_buf);
            jb_add_str(&jb, "soft-version", APP_FIRMWARE_VERSION, ts_buf);
            jb_add_str(&jb, "ICCID", iccid, ts_buf);
            jb_add_str(&jb, "IMEI", imei_buf, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "get_ip") == 0)
    {
        if(EC801E_EnsureConnected() == EC_OK)
        {
            char ip_str[48];
            snprintf(ip_str, sizeof(ip_str), "%s.%d", g_mqtt.server, g_mqtt.port);
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_str(&jb, "first_ip", ip_str, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            publish_feedback(1);
        }
    }
    else if(strcmp(id_val, "get_scend_ip") == 0)
    {
        if(EC801E_EnsureConnected() == EC_OK)
        {
            char ip_str[48];
            snprintf(ip_str, sizeof(ip_str), "%s.%d",
                     g_mqtt.server2[0] ? g_mqtt.server2 : "0.0.0.0", g_mqtt.port2);
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_str(&jb, "scend_ip", ip_str, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            publish_feedback(1);
        }
    }
    else if(strcmp(id_val, "first_ip_set") == 0)
    {
        char ip_buf[MQTT_FIELD_MAX];
        strncpy(ip_buf, cmd_val, sizeof(ip_buf) - 1);
        ip_buf[sizeof(ip_buf) - 1] = '\0';
        char *last_dot = strrchr(ip_buf, '.');
        if(last_dot)
        {
            *last_dot = '\0';
            g_mqtt.port = (uint16_t)atoi(last_dot + 1);
            strncpy(g_mqtt.server, ip_buf, MQTT_FIELD_MAX - 1);
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
                publish_feedback(2);
            }
        }
        else
        {
            if(EC801E_EnsureConnected() == EC_OK)
                publish_feedback(24);
        }
    }
    else if(strcmp(id_val, "scend_ip_set") == 0)
    {
        char ip_buf[MQTT_FIELD_MAX];
        strncpy(ip_buf, cmd_val, sizeof(ip_buf) - 1);
        ip_buf[sizeof(ip_buf) - 1] = '\0';
        char *last_dot = strrchr(ip_buf, '.');
        if(last_dot)
        {
            *last_dot = '\0';
            g_mqtt.port2 = (uint16_t)atoi(last_dot + 1);
            strncpy(g_mqtt.server2, ip_buf, MQTT_FIELD_MAX - 1);
            config_save_mqtt();
            printf_debug("[SM] Secondary IP set: %s:%d\r\n", g_mqtt.server2, g_mqtt.port2);
            if(EC801E_EnsureConnected() == EC_OK)
            {
                EC801E_PublishResponse("scend_ip_set", cmd_val);
                trea_delay_ms(300);
                publish_feedback(2);
            }
        }
        else
        {
            if(EC801E_EnsureConnected() == EC_OK)
                publish_feedback(24);
        }
    }
    else if(strcmp(id_val, "sensor_status") == 0)
    {
        PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);
        {
            uint8_t cur_online = (g_sensor.raw.err == PT304D_OK) ? 1 : 0;
            if(cur_online != g_sensor.prev_online)
                g_sensor.prev_online = cur_online;
        }
        printf_debug("[SM] sensor_status queried\r\n");
        if(EC801E_EnsureConnected() == EC_OK)
            publish_sensor_status();
    }
    else if(strcmp(id_val, "temp_status") == 0)
    {
        PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);
        TempStatus_t ts = threshold_get_temp_status(g_sensor.raw.temperature);
        const char *ts_str = threshold_temp_status_str(ts);
        printf_debug("[SM] temp_status: T=%.2f -> %s\r\n", g_sensor.raw.temperature, ts_str);
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_str(&jb, "temp_status", ts_str, ts_buf);
            jb_add_float(&jb, "temp", g_sensor.raw.temperature, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "LY06_get") == 0)
    {
        // ---- 本端服务器查询电压报警状态, 上报 LY06 (-1=未知/0=正常/1=报警) ----
        // 实时读取 ADC; <0 表示无效 → threshold_voltage_alarm 返回 -1 (未知)
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
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "restart") == 0)
    {
        printf_debug("[SM] Restart commanded\r\n");
        if(EC801E_EnsureConnected() == EC_OK)
            EC801E_PublishResponse("restart", "0");
        trea_delay_ms(200);
        NVIC_SystemReset();
    }
    else if(strcmp(id_val, "restart_off") == 0)
    {
        printf_debug("[SM] Restart_off: closing valve then reset\r\n");
        valve_execute("close", 1);
        if(EC801E_EnsureConnected() == EC_OK)
            EC801E_PublishResponse("restart_off", "0");
        trea_delay_ms(200);
        NVIC_SystemReset();
    }
    else if(strcmp(id_val, "4Gpower_off") == 0)
    {
        printf_debug("[SM] 4G power off\r\n");
        report_4g_power_state(0);
        trea_delay_ms(200);
        EC801E_PowerOff();
        fourG_user_off = 1;
    }
    else if(strcmp(id_val, "real_set") == 0 || strcmp(id_val, "real_get") == 0)
    {
        if(strcmp(id_val, "real_set") == 0 && cmd_val[0] != '\0')
        {
            g_real_enabled = (uint8_t)atoi(cmd_val);
            if(g_real_enabled > 1) g_real_enabled = 0;
            char freq_buf[8] = {0};
            extract_json_field(raw_buf, "real_frequency\",\"value\"", freq_buf, sizeof(freq_buf));
            if(freq_buf[0])
            {
                g_real_frequency = (uint8_t)atoi(freq_buf);
                if(g_real_frequency < 1 || g_real_frequency > 10)
                    g_real_frequency = 5;
            }
            config_save_ext();
            last_realtime_alarm = THRESHOLD_OK;
            printf_debug("[SM] real_set: enabled=%d frequency=%ds\r\n", g_real_enabled, g_real_frequency);
        }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_int(&jb, "real_enabled", g_real_enabled, ts_buf);
            jb_add_int(&jb, "real_frequency", g_real_frequency, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            publish_feedback(strcmp(id_val, "real_set") == 0 ? 14 : 13);
        }
    }
    else if(strcmp(id_val, "lcd_switch") == 0)
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
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "sensor_count_set") == 0)
    {
        if(cmd_val[0] != '\0')
        {
            uint8_t v = (uint8_t)atoi(cmd_val);
            if(v >= 1 && v <= 2) g_sensor_count = v;
            config_save_ext();
            printf_debug("[SM] sensor_count_set: %d\r\n", g_sensor_count);
        }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_int(&jb, "sensor_count", g_sensor_count, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "vpthreshold2") == 0)
    {
        if(cmd_val[0] != '\0')
        {
            char tv[16];
            if(extract_json_field(raw_buf, "vpthreshold2_switch\",\"value\"", tv, sizeof(tv)))
            { g_pthreshold2.enabled = (uint8_t)atoi(tv); if(g_pthreshold2.enabled > 1) g_pthreshold2.enabled = 0; }
            if(extract_json_field(raw_buf, "vpthreshold2_ppmax\",\"value\"", tv, sizeof(tv)))
                g_pthreshold2.ppmax = (float)atof(tv);
            if(extract_json_field(raw_buf, "vpthreshold2_pmax\",\"value\"", tv, sizeof(tv)))
                g_pthreshold2.pmax = (float)atof(tv);
            if(extract_json_field(raw_buf, "vpthreshold2_ppmin\",\"value\"", tv, sizeof(tv)))
                g_pthreshold2.ppmin = (float)atof(tv);
            if(extract_json_field(raw_buf, "vpthreshold2_pmin\",\"value\"", tv, sizeof(tv)))
                g_pthreshold2.pmin = (float)atof(tv);
            if(extract_json_field(raw_buf, "vpthreshold2_mutation\",\"value\"", tv, sizeof(tv)))
                g_pthreshold2.mutation = (float)atof(tv);
            if(extract_json_field(raw_buf, "vpthreshold2_shock\",\"value\"", tv, sizeof(tv)))
                g_pthreshold2.shock = (float)atof(tv);
            config_save_vpthreshold2();
            printf_debug("[SM] vpthreshold2 updated\r\n");
        }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_int(&jb, "vpthreshold2_switch", g_pthreshold2.enabled, ts_buf);
            jb_add_float1(&jb, "vpthreshold2_ppmax", g_pthreshold2.ppmax, ts_buf);
            jb_add_float1(&jb, "vpthreshold2_pmax", g_pthreshold2.pmax, ts_buf);
            jb_add_float1(&jb, "vpthreshold2_ppmin", g_pthreshold2.ppmin, ts_buf);
            jb_add_float1(&jb, "vpthreshold2_pmin", g_pthreshold2.pmin, ts_buf);
            jb_add_float1(&jb, "vpthreshold2_mutation", g_pthreshold2.mutation, ts_buf);
            jb_add_float1(&jb, "vpthreshold2_shock", g_pthreshold2.shock, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            publish_feedback(cmd_val[0] != '\0' ? 8 : 7);
        }
    }
    else if(strcmp(id_val, "Tthreshold2") == 0)
    {
        if(cmd_val[0] != '\0')
        {
            char tv[16];
            if(extract_json_field(raw_buf, "Tthreshold2_switch\",\"value\"", tv, sizeof(tv)))
            { g_tthreshold2.enabled = (uint8_t)atoi(tv); if(g_tthreshold2.enabled > 1) g_tthreshold2.enabled = 0; }
            if(extract_json_field(raw_buf, "Tthreshold2_high\",\"value\"", tv, sizeof(tv)))
                g_tthreshold2.high = (float)atof(tv);
            if(extract_json_field(raw_buf, "Tthreshold2_low\",\"value\"", tv, sizeof(tv)))
                g_tthreshold2.low = (float)atof(tv);
            if(extract_json_field(raw_buf, "Tthreshold2_vhigh\",\"value\"", tv, sizeof(tv)))
                g_tthreshold2.vhigh = (float)atof(tv);
            if(extract_json_field(raw_buf, "Tthreshold2_vlow\",\"value\"", tv, sizeof(tv)))
                g_tthreshold2.vlow = (float)atof(tv);
            config_save_tthreshold2();
            printf_debug("[SM] Tthreshold2 updated\r\n");
        }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_int(&jb, "Tthreshold2_switch", g_tthreshold2.enabled, ts_buf);
            jb_add_float1(&jb, "Tthreshold2_high", g_tthreshold2.high, ts_buf);
            jb_add_float1(&jb, "Tthreshold2_low", g_tthreshold2.low, ts_buf);
            jb_add_float1(&jb, "Tthreshold2_vhigh", g_tthreshold2.vhigh, ts_buf);
            jb_add_float1(&jb, "Tthreshold2_vlow", g_tthreshold2.vlow, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            publish_feedback(cmd_val[0] != '\0' ? 6 : 5);
        }
    }
    else if(strcmp(id_val, "temp_status2") == 0)
    {
        PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);
        TempStatus_t ts2 = threshold_get_temp_status2(g_sensor.raw2.temperature);
        const char *ts2_str = threshold_temp_status_str(ts2);
        printf_debug("[SM] temp_status2: T2=%.2f -> %s\r\n", g_sensor.raw2.temperature, ts2_str);
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_str(&jb, "temp2_status", ts2_str, ts_buf);
            jb_add_float(&jb, "temp2", g_sensor.raw2.temperature, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "get_pressure2") == 0)
    {
        PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);
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
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "user_ip_get") == 0)
    {
        if(EC801E_EnsureConnected() == EC_OK)
        {
            char ip_str[48];
            snprintf(ip_str, sizeof(ip_str), "%s.%d",
                     g_user_mqtt.server[0] ? g_user_mqtt.server : "0.0.0.0", g_user_mqtt.port);
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            jb_add_str(&jb, "user_ip", ip_str, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            publish_feedback(1);
        }
    }
    else if(strcmp(id_val, "user_ip_set") == 0)
    {
        char ip_buf[MQTT_FIELD_MAX];
        strncpy(ip_buf, cmd_val, sizeof(ip_buf) - 1);
        ip_buf[sizeof(ip_buf) - 1] = '\0';
        char *last_dot = strrchr(ip_buf, '.');
        if(last_dot)
        {
            *last_dot = '\0';
            g_user_mqtt.port = (uint16_t)atoi(last_dot + 1);
            strncpy(g_user_mqtt.server, ip_buf, MQTT_FIELD_MAX - 1);
            // topic为空时自动从client_id拼接, 已有自定义topic则保留不覆盖
            if(g_user_mqtt.topic_up[0] == '\0' && g_user_mqtt.client_id[0])
                snprintf(g_user_mqtt.topic_up, MQTT_TOPIC_MAX, "/155/%s/property/post", g_user_mqtt.client_id);
            config_save_user_mqtt();
            printf_debug("[SM] User IP set: %s:%d\r\n", g_user_mqtt.server, g_user_mqtt.port);
            if(EC801E_EnsureConnected() == EC_OK)
            {
                EC801E_PublishResponse("user_ip_set", cmd_val);
                trea_delay_ms(300);
                publish_feedback(2);
            }
        }
        else
        {
            if(EC801E_EnsureConnected() == EC_OK)
                publish_feedback(24);
        }
    }
    else if(strcmp(id_val, "user_mqtt_get") == 0 || strcmp(id_val, "user_mqtt_set") == 0)
    {
        if(strcmp(id_val, "user_mqtt_set") == 0)
        {
            char tv[MQTT_TOPIC_MAX];
            uint8_t topic_explicit = 0;
            if(extract_json_field(raw_buf, "MQTT_username\",\"value\"", tv, sizeof(tv)))
                strncpy(g_user_mqtt.username, tv, MQTT_FIELD_MAX - 1);
            if(extract_json_field(raw_buf, "MQTT_password\",\"value\"", tv, sizeof(tv)))
                strncpy(g_user_mqtt.password, tv, MQTT_FIELD_MAX - 1);
            if(extract_json_field(raw_buf, "MQTT_subject\",\"value\"", tv, sizeof(tv)))
            {
                strncpy(g_user_mqtt.topic_up, tv, MQTT_TOPIC_MAX - 1);
                topic_explicit = 1;
            }
            if(extract_json_field(raw_buf, "MQTT_client_id\",\"value\"", tv, sizeof(tv)))
            {
                strncpy(g_user_mqtt.client_id, tv, MQTT_FIELD_MAX - 1);
                // 下发未显式指定topic且当前topic为空时, 自动从client_id拼接
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
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            publish_feedback(strcmp(id_val, "user_mqtt_set") == 0 ? 18 : 17);
        }
    }
    else if(strcmp(id_val, "valve_status") == 0)
    {
        // ---- 查询阀门实时状态 (不执行动作, 仅读限位开关) ----
        const char *vs = valve_get_status();
        printf_debug("[SM] valve_get: %s\r\n", vs);
        if(EC801E_EnsureConnected() == EC_OK)
            EC801E_PublishResponse("valve_control", vs);
    }
    else if(strcmp(id_val, "closeoropen") == 0)
    {
        // ---- 定时开关阀 set/get ----
        if(cmd_val[0] != '\0')
        {
            // closeoropen_set: 解析 commandParms
            char tv[16];
            if(extract_json_field(raw_buf, "timing_switch\",\"value\"", tv, sizeof(tv)))
            { g_valve_timing.enabled = (uint8_t)atoi(tv); if(g_valve_timing.enabled > 1) g_valve_timing.enabled = 0; }
            if(extract_json_field(raw_buf, "timing_open\",\"value\"", tv, sizeof(tv)))
                g_valve_timing.open_time = (uint32_t)atol(tv);
            if(extract_json_field(raw_buf, "timing_close\",\"value\"", tv, sizeof(tv)))
                g_valve_timing.close_time = (uint32_t)atol(tv);
            config_save_valve_timing();
            timing_open_done_today  = 0;
            timing_close_done_today = 0;
            printf_debug("[SM] closeoropen_set: en=%d open=%06lu close=%06lu (flags reset)\r\n",
                         g_valve_timing.enabled, g_valve_timing.open_time, g_valve_timing.close_time);
        }
        // 上报当前配置
        if(EC801E_EnsureConnected() == EC_OK)
        {
            char v_str[12];
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            jb_begin(&jb, json_buf, sizeof(json_buf));
            snprintf(v_str, sizeof(v_str), "%d", g_valve_timing.enabled);
            jb_add_str(&jb, "timing_switch", v_str, ts_buf);
            snprintf(v_str, sizeof(v_str), "%06lu", g_valve_timing.open_time);
            jb_add_str(&jb, "timing_open", v_str, ts_buf);
            snprintf(v_str, sizeof(v_str), "%06lu", g_valve_timing.close_time);
            jb_add_str(&jb, "timing_close", v_str, ts_buf);
            jb_end(&jb);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            publish_feedback(cmd_val[0] != '\0' ? 22 : 21);
        }
    }
    else
    {
        printf_debug("[SM] Unknown cmd: %s\r\n", id_val);
    }
}

// ==================== 排空并处理缓存的MQTT下发消息 ====================
uint8_t drain_pending_downlink(void)
{
    static char dl_buf[1024];
    char id_val[32];
    char cmd_val[192];
    uint8_t msg_count = 0;
    uint8_t i;

    while(msg_count < 5)
    {
        id_val[0] = '\0';
        cmd_val[0] = '\0';

        if(EC801E_ReadDownlink(dl_buf, sizeof(dl_buf)) != EC_OK)
            break;

        msg_count++;

        // OTA 升级检测
        if(strstr(dl_buf, "downloadUrl") != NULL)
        {
            printf_debug("[SM] OTA upgrade cmd detected\r\n");
            ota_app_handle_upgrade_cmd(dl_buf);
            continue;
        }

        // 解析 commandStr (格式B) 或 id/value (格式A)
        {
            char cs[64] = {0};
            extract_json_field(dl_buf, "\"commandStr\"", cs, sizeof(cs));

            if(cs[0] != '\0')
            {
                // 查表映射
                uint8_t found = 0;
                for(i = 0; i < CMD_MAP_COUNT; i++)
                {
                    if(strcmp(cs, s_cmd_map[i].cmd_str) == 0)
                    {
                        strcpy(id_val, s_cmd_map[i].id);
                        if(s_cmd_map[i].value != NULL)
                            strcpy(cmd_val, s_cmd_map[i].value);
                        else
                            extract_json_field(dl_buf, s_cmd_map[i].extract, cmd_val, sizeof(cmd_val));
                        found = 1;
                        break;
                    }
                }
                if(!found)
                {
                    strncpy(id_val, cs, sizeof(id_val) - 1);
                    id_val[sizeof(id_val) - 1] = '\0';
                }
            }
            else
            {
                // 格式A: 直接取 id / value
                extract_json_field(dl_buf, "\"id\"", id_val, sizeof(id_val));
                extract_json_field(dl_buf, "\"value\"", cmd_val, sizeof(cmd_val));
            }
        }
        printf_debug("[SM] CMD id=%s value=%s\r\n", id_val, cmd_val);
        process_downlink_cmd(id_val, cmd_val, dl_buf);

    }  // while(msg_count < 5)

    return msg_count;
}
