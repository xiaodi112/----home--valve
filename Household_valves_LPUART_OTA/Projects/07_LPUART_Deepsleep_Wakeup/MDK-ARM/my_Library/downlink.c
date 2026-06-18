/**
 * @file downlink.c
 * @brief MQTT下发指令处理模块
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
#include "gd32l23x.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// ==================== 跨模块变量访问 (定义在main.c) ====================
extern uint8_t fourG_user_off;
extern ThresholdResult_t last_realtime_alarm;
extern uint8_t g_need_adc_sampling;

// ==================== 内部: JSON字段提取 ====================
// 返回 1=找到字段且 out 非空, 0=未找到
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
    q++;  // skip opening quote
    r = strchr(q, '"');
    if(!r) return 0;
    len = (uint16_t)(r - q);
    if(len >= out_size) len = out_size - 1;
    memcpy(out, q, len);
    out[len] = '\0';
    return (len > 0) ? 1 : 0;
}

// ==================== 处理单条下发指令 ====================
static void process_downlink_cmd(const char *id_val, const char *cmd_val, const char *raw_buf)
{
    static char json_buf[1024];
    static char ts_buf[32];

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
            EC801E_PublishResponse("valve_control", result);
            // 上报阀门最新状态
            char v_order[4];
            snprintf(v_order, sizeof(v_order), "%d", g_valve_last_order);
            EC801E_PublishResponse("valve_last_order", v_order);
        }
        else
            printf_debug("[SM] Valve publish skipped: MQTT not connected\r\n");
    }
    else if(strcmp(id_val, "sensor_read") == 0 || strcmp(id_val, "get_pressure") == 0)
    {
        // ---- 立即采集+阈值校验+压力状态上报 ----
        PressureStatus_t ps;
        const char *ps_str;

        PT304D_CollectAll(&g_sensor.raw2, &g_sensor.raw);
        g_sensor.voltage = adc_bat_read_voltage();
        printf_debug("[SM] sensor_read: P=%.2f T=%.2f V=%.2f err=%d\r\n",
                     g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage, g_sensor.raw.err);

        // 传感器状态更新 + 上报
        {
            uint8_t cur_online = (g_sensor.raw.err == PT304D_OK) ? 1 : 0;
            const char *have   = (g_sensor.raw.err != PT304D_ERR_I2C) ? "1" : "0";
            const char *rstatus = (g_sensor.raw.err == PT304D_OK) ? "1" : "0";   // I2C 采集是否成功

            if(cur_online != g_sensor.prev_online)
            {
                g_sensor.prev_online = cur_online;
                printf_debug("[SM] sensor_read: STATE CHANGED -> %s\r\n",
                             cur_online ? "ONLINE" : "OFFLINE");
            }

            // 每次平台查询都上报传感器状态
            if(EC801E_EnsureConnected() == EC_OK)
            {
                rtc_get_time_str(ts_buf, sizeof(ts_buf));
                snprintf(json_buf, sizeof(json_buf),
                    "[{\"id\":\"rtu_have\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
                    "{\"id\":\"rtu_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]",
                    have, ts_buf, rstatus, ts_buf);
                EC801E_PublishRawJson(json_buf);
            }
        }

        // 压力状态判断 + 上报
        ps = threshold_get_pressure_status(g_sensor.raw.pressure, g_sensor.last_pressure_collect);
        ps_str = threshold_pressure_status_str(ps);

        if(EC801E_EnsureConnected() == EC_OK)
        {
            // 上报传感器数据
            EC801E_PublishSensorData(&g_sensor.raw, g_sensor.voltage);
            // 上报压力状态
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            snprintf(json_buf, sizeof(json_buf),
                "[{\"id\":\"pressure_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]", ps_str, ts_buf);
            EC801E_PublishRawJson(json_buf);
        }

        // 温度状态判断 + 上报
        {
            TempStatus_t t_s = threshold_get_temp_status(g_sensor.raw.temperature);
            const char *t_str = threshold_temp_status_str(t_s);
            if(EC801E_EnsureConnected() == EC_OK)
            {
                rtc_get_time_str(ts_buf, sizeof(ts_buf));
                snprintf(json_buf, sizeof(json_buf),
                    "[{\"id\":\"temp_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]", t_str, ts_buf);
                EC801E_PublishRawJson(json_buf);
            }
        }

        // 阈值检查 + 异常自动关阀
        {
            ThresholdResult_t tr = threshold_check(g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage, g_sensor.last_pressure_collect, g_need_adc_sampling);
            if(tr != THRESHOLD_OK)
            {
                // 上报报警
                const char *alarm_type = "unknown";
                switch(tr)
                {
                    case THRESHOLD_VPROTECT:        alarm_type = "voltage_protect";    break;
                    case THRESHOLD_VERY_LOW:        alarm_type = "pressure_verylow";   break;
                    case THRESHOLD_VERY_HIGH:       alarm_type = "pressure_veryhigh";  break;
                    case THRESHOLD_OVER_LIMIT:      alarm_type = "pressure_high";      break;
                    case THRESHOLD_UNDER_LIMIT:     alarm_type = "pressure_low";       break;
                    case THRESHOLD_MUTATION:        alarm_type = "pressure_mutation";  break;
                    case THRESHOLD_TEMP_HIGH:       alarm_type = "temperature_high";   break;
                    case THRESHOLD_TEMP_LOW:        alarm_type = "temperature_low";    break;
                    case THRESHOLD_TEMP_VERY_HIGH:  alarm_type = "temperature_veryhigh"; break;
                    case THRESHOLD_TEMP_VERY_LOW:   alarm_type = "temperature_verylow";  break;
                    case THRESHOLD_VOLT_HIGH:       alarm_type = "voltage_high";       break;
                    case THRESHOLD_VOLT_LOW:        alarm_type = "voltage_low";        break;
                    default: break;
                }
                printf_debug("[SM] sensor_read ALARM: %s\r\n", alarm_type);
                if(EC801E_EnsureConnected() == EC_OK)
                    EC801E_PublishAlarm(alarm_type, g_sensor.raw.pressure, g_sensor.voltage);

                // 守卫: !g_valve_alarm_closed 避免二次/多次异常覆盖 pre_alarm_order
                AbnormalClose_t policy = abnormal_from_str(g_abnormal_close);
                if(abnormal_should_close_valve(policy, tr) && !g_valve_alarm_closed)
                {
                    g_valve_pre_alarm_order = g_valve_last_order;
                    printf_debug("[SM] Auto-close valve (policy=%s, pre=%d)\r\n",
                                 g_abnormal_close, g_valve_pre_alarm_order);
                    const char *vr = valve_execute("close", 0);
                    g_valve_alarm_closed = 1;
                    config_save_ext();   // 持久化恢复标志
                    if(EC801E_EnsureConnected() == EC_OK)
                    {
                        EC801E_PublishResponse("valve_control", vr);
                        char v_order[4];
                        snprintf(v_order, sizeof(v_order), "%d", g_valve_last_order);
                        EC801E_PublishResponse("valve_last_order", v_order);
                    }
                }
            }
        }
    }
    else if(strcmp(id_val, "collect_cycle") == 0)
    {
        // ---- 采集周期 (cmd_val非空=设置, 空=查询) ----
        if(cmd_val[0] != '\0')
        {
            char type_buf[16] = {0};
            g_collect_cycle.once = (uint16_t)atoi(cmd_val);
            // 兼容两种格式: "type" (格式A) 或 "collect_type","value" (commandParms)
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
        // 上报当前配置
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            snprintf(json_buf, sizeof(json_buf),
                "[{\"id\":\"collect_once\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"collect_type\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]",
                g_collect_cycle.once, ts_buf, cycle_type_to_str(g_collect_cycle.type), ts_buf);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "report_cycle") == 0)
    {
        // ---- 上报周期 (cmd_val非空=设置, 空=查询) ----
        if(cmd_val[0] != '\0')
        {
            char type_buf[16] = {0};
            g_report_cycle.once = (uint16_t)atoi(cmd_val);
            // 兼容两种格式: "type" (格式A) 或 "report_type","value" (commandParms)
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
            snprintf(json_buf, sizeof(json_buf),
                "[{\"id\":\"report_once\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"report_type\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]",
                g_report_cycle.once, ts_buf, cycle_type_to_str(g_report_cycle.type), ts_buf);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "vpthreshold") == 0)
    {
        // ---- 压力阈值 (vpthreshold_set1/get1) ----
        // raw_buf 含 vpthreshold_switch / ppmax / pmax / ppmin / pmin / mutation / shock
        char tv[16];
        int changed = 0;
        if(extract_json_field(raw_buf, "vpthreshold_switch\",\"value\"", tv, sizeof(tv)))
        { g_pthreshold.enabled  = (uint8_t)atoi(tv); if(g_pthreshold.enabled > 1) g_pthreshold.enabled = 0; changed = 1; }
        if(extract_json_field(raw_buf, "vpthreshold_ppmax\",\"value\"", tv, sizeof(tv)))
        { g_pthreshold.ppmax    = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "vpthreshold_pmax\",\"value\"", tv, sizeof(tv)))
        { g_pthreshold.pmax     = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "vpthreshold_ppmin\",\"value\"", tv, sizeof(tv)))
        { g_pthreshold.ppmin    = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "vpthreshold_pmin\",\"value\"", tv, sizeof(tv)))
        { g_pthreshold.pmin     = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "vpthreshold_mutation\",\"value\"", tv, sizeof(tv)))
        { g_pthreshold.mutation = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "vpthreshold_shock\",\"value\"", tv, sizeof(tv)))
        { g_pthreshold.shock    = (float)atof(tv); changed = 1; }
        if(changed)
        {
            config_save_vpthreshold();
            printf_debug("[SM] vpthreshold updated\r\n");
        }
        // 上报全字段
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            snprintf(json_buf, sizeof(json_buf),
                "[{\"id\":\"vpthreshold_switch\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"vpthreshold_ppmax\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"vpthreshold_pmax\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"vpthreshold_ppmin\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"vpthreshold_pmin\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"vpthreshold_mutation\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"vpthreshold_shock\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"}]",
                g_pthreshold.enabled, ts_buf,
                g_pthreshold.ppmax, ts_buf, g_pthreshold.pmax, ts_buf,
                g_pthreshold.ppmin, ts_buf, g_pthreshold.pmin, ts_buf,
                g_pthreshold.mutation, ts_buf, g_pthreshold.shock, ts_buf);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "Tthreshold") == 0)
    {
        // ---- 温度阈值 (Tthreshold_set/get) ----
        char tv[16];
        int changed = 0;
        if(extract_json_field(raw_buf, "Tthreshold_switch\",\"value\"", tv, sizeof(tv)))
        { g_tthreshold.enabled = (uint8_t)atoi(tv); if(g_tthreshold.enabled > 1) g_tthreshold.enabled = 0; changed = 1; }
        if(extract_json_field(raw_buf, "Tthreshold_high\",\"value\"", tv, sizeof(tv)))
        { g_tthreshold.high  = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "Tthreshold_low\",\"value\"", tv, sizeof(tv)))
        { g_tthreshold.low   = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "Tthreshold_vhigh\",\"value\"", tv, sizeof(tv)))
        { g_tthreshold.vhigh = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "Tthreshold_vlow\",\"value\"", tv, sizeof(tv)))
        { g_tthreshold.vlow  = (float)atof(tv); changed = 1; }
        if(changed)
        {
            config_save_tthreshold();
            printf_debug("[SM] Tthreshold updated\r\n");
        }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            snprintf(json_buf, sizeof(json_buf),
                "[{\"id\":\"Tthreshold_switch\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"Tthreshold_high\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"Tthreshold_low\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"Tthreshold_vhigh\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"Tthreshold_vlow\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"}]",
                g_tthreshold.enabled, ts_buf,
                g_tthreshold.high, ts_buf, g_tthreshold.low, ts_buf,
                g_tthreshold.vhigh, ts_buf, g_tthreshold.vlow, ts_buf);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "spupower") == 0)
    {
        // ---- 电压检测 (spupower_set/get) ----
        char tv[16];
        int changed = 0;
        if(extract_json_field(raw_buf, "spupower_switch\",\"value\"", tv, sizeof(tv)))
        { g_spupower.enabled = (uint8_t)atoi(tv); if(g_spupower.enabled > 1) g_spupower.enabled = 0; changed = 1; }
        if(extract_json_field(raw_buf, "spupower_high\",\"value\"", tv, sizeof(tv)))
        { g_spupower.high    = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "spupower_low\",\"value\"", tv, sizeof(tv)))
        { g_spupower.low     = (float)atof(tv); changed = 1; }
        if(extract_json_field(raw_buf, "spupower_protect\",\"value\"", tv, sizeof(tv)))
        { g_spupower.protect = (float)atof(tv); changed = 1; }
        if(changed)
        {
            config_save_spupower();
            printf_debug("[SM] spupower updated\r\n");
        }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            snprintf(json_buf, sizeof(json_buf),
                "[{\"id\":\"spupower_switch\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"spupower_high\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"spupower_low\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"spupower_protect\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"}]",
                g_spupower.enabled, ts_buf,
                g_spupower.high, ts_buf, g_spupower.low, ts_buf,
                g_spupower.protect, ts_buf);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "threshold") == 0)
    {
        // ---- 兼容旧版 threshold_get1: 依次回复三组阈值配置 ----
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            snprintf(json_buf, sizeof(json_buf),
                "[{\"id\":\"vpthreshold_switch\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"vpthreshold_ppmax\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"vpthreshold_pmax\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"vpthreshold_ppmin\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"vpthreshold_pmin\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"vpthreshold_mutation\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"vpthreshold_shock\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"}]",
                g_pthreshold.enabled, ts_buf,
                g_pthreshold.ppmax, ts_buf, g_pthreshold.pmax, ts_buf,
                g_pthreshold.ppmin, ts_buf, g_pthreshold.pmin, ts_buf,
                g_pthreshold.mutation, ts_buf, g_pthreshold.shock, ts_buf);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            snprintf(json_buf, sizeof(json_buf),
                "[{\"id\":\"Tthreshold_switch\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"Tthreshold_high\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"Tthreshold_low\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"Tthreshold_vhigh\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"Tthreshold_vlow\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"}]",
                g_tthreshold.enabled, ts_buf,
                g_tthreshold.high, ts_buf, g_tthreshold.low, ts_buf,
                g_tthreshold.vhigh, ts_buf, g_tthreshold.vlow, ts_buf);
            EC801E_PublishRawJson(json_buf);
            trea_delay_ms(300);
            snprintf(json_buf, sizeof(json_buf),
                "[{\"id\":\"spupower_switch\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"spupower_high\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"spupower_low\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"spupower_protect\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"}]",
                g_spupower.enabled, ts_buf,
                g_spupower.high, ts_buf, g_spupower.low, ts_buf,
                g_spupower.protect, ts_buf);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "delay") == 0)
    {
        // ---- 阀门动作延时 (delay_set/get) ----
        char tv[16];
        int changed = 0;
        if(extract_json_field(raw_buf, "delay_switch\",\"value\"", tv, sizeof(tv)))
        { g_vdelay.enabled = (uint8_t)atoi(tv); if(g_vdelay.enabled > 1) g_vdelay.enabled = 0; changed = 1; }
        if(extract_json_field(raw_buf, "delay_time\",\"value\"", tv, sizeof(tv)))
        {
            uint32_t v = (uint32_t)atoi(tv);
            if(v > VDLY_DELAY_MS_MAX) v = VDLY_DELAY_MS_MAX;  // clamp 上限 5000ms
            g_vdelay.delay_ms = (uint16_t)v;
            changed = 1;
        }
        if(changed)
        {
            config_save_vdelay();
            printf_debug("[SM] delay updated: en=%d delay_ms=%d\r\n", g_vdelay.enabled, g_vdelay.delay_ms);
        }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            snprintf(json_buf, sizeof(json_buf),
                "[{\"id\":\"delay_switch\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"delay_time\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"}]",
                g_vdelay.enabled, ts_buf, g_vdelay.delay_ms, ts_buf);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "window_pattern") == 0)
    {
        // ---- 窗口模式 (cmd_val非空=设置, 空=查询) ----
        if(cmd_val[0] != '\0')
        {
            strncpy(g_window.pattern, cmd_val, 7); g_window.pattern[7] = '\0';
            char tmp[16] = {0};
            // 兼容两种格式: "end_result" (格式A) 或 "window_end_result","value" (commandParms)
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
                         g_window.pattern, g_window.end_result,
                         g_window.start, g_window.stop, g_window.dottime);
        }
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            snprintf(json_buf, sizeof(json_buf),
                "[{\"id\":\"window_pattern\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"window_end_result\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"window_start\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"window_stop\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"window_dottime\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"}]",
                g_window.pattern, ts_buf, g_window.end_result, ts_buf,
                g_window.start, ts_buf, g_window.stop, ts_buf, g_window.dottime, ts_buf);
            EC801E_PublishRawJson(json_buf);
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
        // ---- SIM卡信息查询 ----
        char iccid[32] = {0};
        char csq[8] = {0};
        char imei_buf[24] = {0};
        EC801E_QueryCSQ(csq, sizeof(csq));
        EC801E_QueryICCID(iccid, sizeof(iccid));
        EC801E_QueryIMEI(imei_buf, sizeof(imei_buf));
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            snprintf(json_buf, sizeof(json_buf),
                "[{\"id\":\"csq\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"soft-version\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"ICCID\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"IMEI\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]",
                csq, ts_buf, APP_FIRMWARE_VERSION, ts_buf, iccid, ts_buf, imei_buf, ts_buf);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "get_ip") == 0)
    {
        // ---- 主IP上报 (value格式: IP.port) ----
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            snprintf(json_buf, sizeof(json_buf),
                "[{\"id\":\"first_ip\",\"value\":\"%s.%d\",\"ts\":\"%s\",\"remark\":\"\"}]",
                g_mqtt.server, g_mqtt.port, ts_buf);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "get_scend_ip") == 0)
    {
        // ---- 副IP上报 (value格式: IP.port) ----
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            snprintf(json_buf, sizeof(json_buf),
                "[{\"id\":\"scend_ip\",\"value\":\"%s.%d\",\"ts\":\"%s\",\"remark\":\"\"}]",
                g_mqtt.server2[0] ? g_mqtt.server2 : "0.0.0.0", g_mqtt.port2, ts_buf);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "first_ip_set") == 0)
    {
        // ---- 设置主IP (value格式: IP.port, 如 120.78.220.171.1883) ----
        // 最后一个'.'后为port
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
            // 断开重连使新IP生效
            EC801E_DisconnectMQTT();
            trea_delay_ms(500);
            EC801E_ConnectMQTT();
            EC801E_SubscribeTopic();
            if(EC801E_EnsureConnected() == EC_OK)
                EC801E_PublishResponse("first_ip_set", cmd_val);
        }
    }
    else if(strcmp(id_val, "scend_ip_set") == 0)
    {
        // ---- 设置副IP (value格式: IP.port) ----
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
                EC801E_PublishResponse("scend_ip_set", cmd_val);
        }
    }
    else if(strcmp(id_val, "sensor_status") == 0)
    {
        // ---- 传感器在线状态查询: 上报 rtu_have + rtu_status ----
        PT304D_CollectAll(&g_sensor.raw2, &g_sensor.raw);
        const char *have   = (g_sensor.raw.err != PT304D_ERR_I2C) ? "1" : "0";
        const char *rstatus = (g_sensor.raw.err == PT304D_OK) ? "1" : "0";   // I2C 采集是否成功
        // 同步传感器状态
        {
            uint8_t cur_online = (g_sensor.raw.err == PT304D_OK) ? 1 : 0;
            if(cur_online != g_sensor.prev_online)
                g_sensor.prev_online = cur_online;
        }
        printf_debug("[SM] sensor_status: err=%d have=%s rtu_status=%s\r\n", g_sensor.raw.err, have, rstatus);
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            snprintf(json_buf, sizeof(json_buf),
                "[{\"id\":\"rtu_have\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"rtu_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]",
                have, ts_buf, rstatus, ts_buf);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "temp_status") == 0)
    {
        // ---- 温度状态查询 (temp_get1) ----
        PT304D_CollectAll(&g_sensor.raw2, &g_sensor.raw);
        TempStatus_t ts = threshold_get_temp_status(g_sensor.raw.temperature);
        const char *ts_str = threshold_temp_status_str(ts);
        printf_debug("[SM] temp_status: T=%.2f -> %s\r\n", g_sensor.raw.temperature, ts_str);
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            snprintf(json_buf, sizeof(json_buf),
                "[{\"id\":\"temp_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"temp\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"}]",
                ts_str, ts_buf, g_sensor.raw.temperature, ts_buf);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "restart") == 0)
    {
        // ---- 系统重启 (先上报0=重启中, 重启后上报1=开机成功) ----
        printf_debug("[SM] Restart commanded\r\n");
        if(EC801E_EnsureConnected() == EC_OK)
            EC801E_PublishResponse("restart", "0");
        trea_delay_ms(200);
        NVIC_SystemReset();
    }
    else if(strcmp(id_val, "4Gpower_off") == 0)
    {
        // ---- 4G模块断电 (先上报确认再断电) ----
        printf_debug("[SM] 4G power off\r\n");
        report_4g_power_state(0);
        trea_delay_ms(200);
        EC801E_PowerOff();
        fourG_user_off = 1;  // 标记用户主动关机(防止do_wakeup盲目恢复PD9)
    }
    else if(strcmp(id_val, "real_set") == 0)
    {
        // ---- 实时采集设置 ----
        // cmd_val = real_enabled value, raw_buf 中还有 real_frequency
        if(cmd_val[0] != '\0')
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
            last_realtime_alarm = THRESHOLD_OK;  // 重置防重复标记
            printf_debug("[SM] real_set: enabled=%d frequency=%ds\r\n",
                         g_real_enabled, g_real_frequency);
        }
        // 上报当前配置
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            snprintf(json_buf, sizeof(json_buf),
                "[{\"id\":\"real_enabled\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"real_frequency\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"}]",
                g_real_enabled, ts_buf, g_real_frequency, ts_buf);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else if(strcmp(id_val, "real_get") == 0)
    {
        // ---- 实时采集查询 ----
        if(EC801E_EnsureConnected() == EC_OK)
        {
            rtc_get_time_str(ts_buf, sizeof(ts_buf));
            snprintf(json_buf, sizeof(json_buf),
                "[{\"id\":\"real_enabled\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"},"
                "{\"id\":\"real_frequency\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"}]",
                g_real_enabled, ts_buf, g_real_frequency, ts_buf);
            EC801E_PublishRawJson(json_buf);
        }
    }
    else
    {
        printf_debug("[SM] Unknown cmd: %s\r\n", id_val);
    }
}

// ==================== 排空并处理缓存的MQTT下发消息 ====================
// 返回处理的消息数 (供do_downlink和do_upload共用)
uint8_t drain_pending_downlink(void)
{
    static char dl_buf[1024];
    char id_val[32];
    char cmd_val[192];
    uint8_t msg_count = 0;

    while(msg_count < 5)
    {
        id_val[0] = '\0';
        cmd_val[0] = '\0';

        if(EC801E_ReadDownlink(dl_buf, sizeof(dl_buf)) != EC_OK)
            break;

        msg_count++;

        // === FastBee 升级下发识别 (upgrade/set) ===
        // payload: {"version":N,"downloadUrl":"http://..."}
        // 该消息不进入 process_downlink_cmd, 由 ota_app 处理 (含上报+重启)
        if(strstr(dl_buf, "downloadUrl") != NULL)
        {
            printf_debug("[SM] OTA upgrade cmd detected\r\n");
            ota_app_handle_upgrade_cmd(dl_buf);
            // ota_app_handle_upgrade_cmd 失败时返回, 成功则 NVIC_SystemReset 不返回
            continue;
        }

        // 解析FastBee JSON: 兼容两种下发格式
        //   格式A (直接id/value): [{"id":"valve","value":"open","type":"..."}]
        //   格式B (commandStr±commandParms): {"commandStr":"valve_open","value":""}
        //   注意: commandParms格式也含"id"字段, 必须先检查commandStr
        {
            char cs[64] = {0};
            extract_json_field(dl_buf, "\"commandStr\"", cs, sizeof(cs));
            if(strcmp(cs, "valve_open") == 0)
            { strcpy(id_val, "valve"); strcpy(cmd_val, "open"); }
            else if(strcmp(cs, "valve_close") == 0)
            { strcpy(id_val, "valve"); strcpy(cmd_val, "close"); }
            else if(strcmp(cs, "get_pressure") == 0)
            { strcpy(id_val, "get_pressure"); strcpy(cmd_val, "1"); }
            else if(strcmp(cs, "get_sim") == 0)
            { strcpy(id_val, "get_sim"); strcpy(cmd_val, "1"); }
            else if(strcmp(cs, "get_ip") == 0)
            { strcpy(id_val, "get_ip"); strcpy(cmd_val, "1"); }
            else if(strcmp(cs, "sensor_status") == 0)
            { strcpy(id_val, "sensor_status"); strcpy(cmd_val, "1"); }
            else if(strcmp(cs, "restart") == 0)
            { strcpy(id_val, "restart"); strcpy(cmd_val, "1"); }
            else if(strcmp(cs, "4Gpower_off") == 0)
            { strcpy(id_val, "4Gpower_off"); strcpy(cmd_val, "1"); }
            else if(strcmp(cs, "temp_get1") == 0)
            { strcpy(id_val, "temp_status"); strcpy(cmd_val, "1"); }
            else if(strcmp(cs, "threshold_get1") == 0)
            { strcpy(id_val, "threshold"); cmd_val[0] = '\0'; }
            else if(strcmp(cs, "collectcycle_get") == 0)
            { strcpy(id_val, "collect_cycle"); cmd_val[0] = '\0'; }
            else if(strcmp(cs, "report_get") == 0)
            { strcpy(id_val, "report_cycle"); cmd_val[0] = '\0'; }
            else if(strcmp(cs, "window_get") == 0)
            { strcpy(id_val, "window_pattern"); cmd_val[0] = '\0'; }
            else if(strcmp(cs, "abnormal_get") == 0)
            { strcpy(id_val, "abnormal_close"); cmd_val[0] = '\0'; }
            else if(strcmp(cs, "collectspeed_set") == 0)
            {
                strcpy(id_val, "collect_cycle");
                extract_json_field(dl_buf, "collect_once\",\"value\"", cmd_val, sizeof(cmd_val));
            }
            else if(strcmp(cs, "report_set") == 0)
            {
                strcpy(id_val, "report_cycle");
                extract_json_field(dl_buf, "report_once\",\"value\"", cmd_val, sizeof(cmd_val));
            }
            else if(strcmp(cs, "window_set") == 0)
            {
                strcpy(id_val, "window_pattern");
                extract_json_field(dl_buf, "window_pattern\",\"value\"", cmd_val, sizeof(cmd_val));
            }
            else if(strcmp(cs, "abnormal_set") == 0)
            {
                strcpy(id_val, "abnormal_close");
                extract_json_field(dl_buf, "abnormal_close\",\"value\"", cmd_val, sizeof(cmd_val));
            }
            else if(strcmp(cs, "restart_off") == 0)
            { strcpy(id_val, "restart"); strcpy(cmd_val, "1"); }
            else if(strcmp(cs, "first_ip_get") == 0)
            { strcpy(id_val, "get_ip"); strcpy(cmd_val, "1"); }
            else if(strcmp(cs, "scend_ip_get") == 0)
            { strcpy(id_val, "get_scend_ip"); strcpy(cmd_val, "1"); }
            else if(strcmp(cs, "first_ip_set") == 0)
            {
                strcpy(id_val, "first_ip_set");
                extract_json_field(dl_buf, "first_ip\",\"value\"", cmd_val, sizeof(cmd_val));
            }
            else if(strcmp(cs, "scend_ip_set") == 0)
            {
                strcpy(id_val, "scend_ip_set");
                extract_json_field(dl_buf, "scend_ip\",\"value\"", cmd_val, sizeof(cmd_val));
            }
            else if(strcmp(cs, "valve_last_get") == 0)
            { strcpy(id_val, "valve_last_order"); strcpy(cmd_val, "1"); }
            else if(strcmp(cs, "real_get") == 0)
            { strcpy(id_val, "real_get"); cmd_val[0] = '\0'; }
            else if(strcmp(cs, "real_set") == 0)
            {
                strcpy(id_val, "real_set");
                extract_json_field(dl_buf, "real_enabled\",\"value\"", cmd_val, sizeof(cmd_val));
            }
            // ---- 新四组阈值 set/get ----
            else if(strcmp(cs, "vpthreshold_set1") == 0)
            { strcpy(id_val, "vpthreshold"); strcpy(cmd_val, "1"); }
            else if(strcmp(cs, "vpthreshold_get1") == 0)
            { strcpy(id_val, "vpthreshold"); cmd_val[0] = '\0'; }
            else if(strcmp(cs, "Tthreshold_set") == 0)
            { strcpy(id_val, "Tthreshold"); strcpy(cmd_val, "1"); }
            else if(strcmp(cs, "Tthreshold_get") == 0)
            { strcpy(id_val, "Tthreshold"); cmd_val[0] = '\0'; }
            else if(strcmp(cs, "spupower_set") == 0)
            { strcpy(id_val, "spupower"); strcpy(cmd_val, "1"); }
            else if(strcmp(cs, "spupower_get") == 0)
            { strcpy(id_val, "spupower"); cmd_val[0] = '\0'; }
            else if(strcmp(cs, "delay_set") == 0)
            { strcpy(id_val, "delay"); strcpy(cmd_val, "1"); }
            else if(strcmp(cs, "delay_get") == 0)
            { strcpy(id_val, "delay"); cmd_val[0] = '\0'; }
            else if(cs[0] != '\0')
            { strncpy(id_val, cs, sizeof(id_val) - 1); id_val[sizeof(id_val) - 1] = 0; }
            else
            {
                extract_json_field(dl_buf, "\"id\"", id_val, sizeof(id_val));
                extract_json_field(dl_buf, "\"value\"", cmd_val, sizeof(cmd_val));
            }
        }
        printf_debug("[SM] CMD id=%s value=%s\r\n", id_val, cmd_val);
        process_downlink_cmd(id_val, cmd_val, dl_buf);

    }  // while(msg_count < 5)

    return msg_count;
}
