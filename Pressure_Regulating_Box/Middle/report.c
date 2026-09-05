/**
 * @file report.c
 * @brief 统一上报模块: 平台 JSON 上报封装
 * @details 统一采用 id/value/ts 格式上报, 供 main/downlink 调用,
 *          避免相同字段上报到多处格式不一致
 */

#include "report.h"
#include "ec800e.h"
#include "eeprom_config.h"
#include "threshold.h"
#include "PT_304_35.h"
#include "main.h"
#include "adc_bat.h"
#include "valve.h"
#include "tim.h"
#include "json_builder.h"
#include "usart.h"
#include "delay.h"
#include <string.h>
#include <stdio.h>

/* ==================== 模块级静态缓冲 (上报相关函数共用) ==================== */
static char        s_rpt_buf[1024];
static char        s_rpt_ts[32];
static JsonBuilder s_rpt_jb;

/* 单字段状态上报: [{"id":..,"value":..,"ts":..,"remark":""}] */
void report_publish_field(const char *id, const char *value)
{
    rtc_get_time_str(s_rpt_ts, sizeof(s_rpt_ts));
    snprintf(s_rpt_buf, sizeof(s_rpt_buf),
        "[{\"id\":\"%s\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]", id, value, s_rpt_ts);
    EC801E_PublishRawJson(s_rpt_buf);
}

/* 指令反馈: feedback=code */
void report_publish_feedback(int code)
{
    rtc_get_time_str(s_rpt_ts, sizeof(s_rpt_ts));
    snprintf(s_rpt_buf, sizeof(s_rpt_buf),
        "[{\"id\":\"feedback\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"}]", code, s_rpt_ts);
    EC801E_PublishRawJson(s_rpt_buf);
}

/* IP 字段上报 (IP.port 格式) + 可选 feedback */
void report_publish_ip_field(const char *field_name, const char *server,
                             uint16_t port, int fb_code)
{
    if(EC801E_EnsureConnected() == EC_OK)
    {
        char ip_str[48];
        snprintf(ip_str, sizeof(ip_str), "%s.%d", server[0] ? server : "0.0.0.0", port);
        rtc_get_time_str(s_rpt_ts, sizeof(s_rpt_ts));
        jb_begin(&s_rpt_jb, s_rpt_buf, sizeof(s_rpt_buf));
        jb_add_str(&s_rpt_jb, field_name, ip_str, s_rpt_ts);
        jb_end(&s_rpt_jb);
        EC801E_PublishRawJson(s_rpt_buf);
        if(fb_code >= 0)
        {
            trea_delay_ms(300);
            report_publish_feedback(fb_code);
        }
    }
}

/* 查询信号信息 (CSQ + RSRP + SINR) */
void report_query_signal_info(char *csq, uint8_t csq_size,
                              char *rsrp, uint8_t rsrp_size,
                              char *sinr, uint8_t sinr_size)
{
    int r = 0, s = 0;
    csq[0] = '\0';
    EC801E_QueryCSQ(csq, csq_size);
    rsrp[0] = '\0';
    sinr[0] = '\0';
    if(EC801E_QueryQCSQ(&r, &s) == EC_OK)
    {
        snprintf(rsrp, rsrp_size, "%d", r);
        snprintf(sinr, sinr_size, "%d", s);
    }
}

/* 上报传感器在线状态 (rtu_have/rtu_status 各 2) */
void report_publish_sensor_status(void)
{
    const char *have    = (g_sensor.raw.err != PT304D_ERR_I2C) ? "1" : "0";
    const char *rstatus = (g_sensor.raw.err == PT304D_OK) ? "1" : "0";

    rtc_get_time_str(s_rpt_ts, sizeof(s_rpt_ts));
    jb_begin(&s_rpt_jb, s_rpt_buf, sizeof(s_rpt_buf));
    jb_add_str(&s_rpt_jb, "rtu_have", have, s_rpt_ts);
    jb_add_str(&s_rpt_jb, "rtu_status", rstatus, s_rpt_ts);

    if(g_sensor_count >= 2)
    {
        const char *have2    = (g_sensor.raw2.err != PT304D_ERR_I2C) ? "1" : "0";
        const char *rstatus2 = (g_sensor.raw2.err == PT304D_OK) ? "1" : "0";
        jb_add_str(&s_rpt_jb, "rtu2_have", have2, s_rpt_ts);
        jb_add_str(&s_rpt_jb, "rtu2_status", rstatus2, s_rpt_ts);
    }

    jb_end(&s_rpt_jb);
    EC801E_PublishRawJson(s_rpt_buf);
}

/* 状态批量上报(实测值+状态同一条消息): 报警/正常/恢复共用
 * 双传感器: temp/pressure/temp_status/pressure_status + temp2/pressure2/temp2_status/pressure2_status
 * 单传感器: 仅前四字段 */
static void publish_status_batch(float last_p1, float last_p2)
{
    rtc_get_time_str(s_rpt_ts, sizeof(s_rpt_ts));
    jb_begin(&s_rpt_jb, s_rpt_buf, sizeof(s_rpt_buf));

    jb_add_float(&s_rpt_jb, "temp", g_sensor.raw.temperature, s_rpt_ts);
    jb_add_float(&s_rpt_jb, "pressure", g_sensor.raw.pressure, s_rpt_ts);
    /* 进气端传感器未采到数据: 状态上报 temp_err/pressure_err */
    if(g_sensor.raw.err != PT304D_OK)
    {
        jb_add_str(&s_rpt_jb, "temp_status", "temp_err", s_rpt_ts);
        jb_add_str(&s_rpt_jb, "pressure_status", "pressure_err", s_rpt_ts);
    }
    else
    {
        jb_add_str(&s_rpt_jb, "temp_status",
                   threshold_temp_status_str(threshold_get_temp_status(g_sensor.raw.temperature)), s_rpt_ts);
        jb_add_str(&s_rpt_jb, "pressure_status",
                   threshold_pressure_status_str(threshold_get_pressure_status(g_sensor.raw.pressure, last_p1)), s_rpt_ts);
    }

    if(g_sensor_count >= 2)
    {
        jb_add_float(&s_rpt_jb, "temp2", g_sensor.raw2.temperature, s_rpt_ts);
        jb_add_float(&s_rpt_jb, "pressure2", g_sensor.raw2.pressure, s_rpt_ts);
        /* 出气端传感器未采到数据: 状态上报 temp_err/pressure_err */
        if(g_sensor.raw2.err != PT304D_OK)
        {
            jb_add_str(&s_rpt_jb, "temp2_status", "temp_err", s_rpt_ts);
            jb_add_str(&s_rpt_jb, "pressure2_status", "pressure_err", s_rpt_ts);
        }
        else
        {
            jb_add_str(&s_rpt_jb, "temp2_status",
                       threshold_temp_status_str(threshold_get_temp_status2(g_sensor.raw2.temperature)), s_rpt_ts);
            jb_add_str(&s_rpt_jb, "pressure2_status",
                       threshold_pressure_status_str(threshold_get_pressure_status2(g_sensor.raw2.pressure, last_p2)), s_rpt_ts);
        }
    }

    jb_end(&s_rpt_jb);
//    printf_debug("[RPT] ALARM batch: %s\r\n", s_rpt_buf);
    EC801E_PublishRawJson(s_rpt_buf);
}

/* 报警批量上报 (实测值 + 状态) */
void report_publish_alarm_batch(float last_p1, float last_p2)
{
    publish_status_batch(last_p1, last_p2);
}

/* 正常/恢复状态批量上报 (与报警同格式: 实测值 + 状态) */
void report_publish_status_all(float last_p1, float last_p2)
{
    publish_status_batch(last_p1, last_p2);
}

/* 电压报警/状态上报 (LY06+voltage, 单条消息, 温度/压力状态分开) */
void report_publish_ly06(uint8_t alarm, float voltage)
{
    rtc_get_time_str(s_rpt_ts, sizeof(s_rpt_ts));
    snprintf(s_rpt_buf, sizeof(s_rpt_buf),
        "[{\"id\":\"LY06\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"},"
         "{\"id\":\"voltage\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"}]",
        alarm ? 1 : 0, s_rpt_ts, voltage, s_rpt_ts);
    printf_debug("[RPT] LY06 status: alarm=%d V=%.2f\r\n", alarm ? 1 : 0, voltage);
    EC801E_PublishRawJson(s_rpt_buf);
}

/* CH4 status report: gas/mstate/mtemp/methane */
#if LASER_CH4_ENABLED
void report_publish_ch4_status(const CH4_Data_t *data, CH4_AlarmLevel_t level)
{
    char mstate_buf[4];
    char methane_buf[8];

    rtc_get_time_str(s_rpt_ts, sizeof(s_rpt_ts));
    snprintf(mstate_buf, sizeof(mstate_buf), "%02d", data->status);
    snprintf(methane_buf, sizeof(methane_buf), "%.2f", data->concentration);

    jb_begin(&s_rpt_jb, s_rpt_buf, sizeof(s_rpt_buf));
    jb_add_int(&s_rpt_jb, "gas", (int)level, s_rpt_ts);
    jb_add_str(&s_rpt_jb, "mstate", mstate_buf, s_rpt_ts);
    jb_add_float(&s_rpt_jb, "mtemp", data->temperature, s_rpt_ts);
    jb_add_str(&s_rpt_jb, "methane", methane_buf, s_rpt_ts);
    jb_end(&s_rpt_jb);

    printf_debug("[RPT] CH4: gas=%d mstate=%s mtemp=%.2f methane=%s\r\n",
                 (int)level, mstate_buf, data->temperature, methane_buf);
    EC801E_PublishRawJson(s_rpt_buf);
}
#endif /* LASER_CH4_ENABLED */

/* ==================== 汇总上报 (历史数据上报后调用) ==================== */
void report_publish_upload_summary(void)
{
    static char csq_buf[8];
    static char v_buf[8];
    static char va_buf[4];
    static volatile char user_state = 0;
    char rsrp_buf[8], sinr_buf[8];

    // 1. 上报传感器在线状态
    report_publish_sensor_status();
    rtc_get_time_str(s_rpt_ts, sizeof(s_rpt_ts));
    jb_begin(&s_rpt_jb, s_rpt_buf, sizeof(s_rpt_buf));

    // 2. 信号强度
    report_query_signal_info(csq_buf, sizeof(csq_buf), rsrp_buf, sizeof(rsrp_buf), sinr_buf, sizeof(sinr_buf));
    jb_add_str(&s_rpt_jb, "csq", csq_buf, s_rpt_ts);
    jb_add_str(&s_rpt_jb, "rsrp", rsrp_buf, s_rpt_ts);
    jb_add_str(&s_rpt_jb, "snr", sinr_buf, s_rpt_ts);

    // 4. 压力+温度状态 (传感器未采到数据 → temp_err/pressure_err)
    {
        if(g_sensor.raw.err != PT304D_OK)
        {
            jb_add_str(&s_rpt_jb, "pressure_status", "pressure_err", s_rpt_ts);
            jb_add_str(&s_rpt_jb, "temp_status", "temp_err", s_rpt_ts);
        }
        else
        {
            PressureStatus_t ps = threshold_get_pressure_status(g_sensor.raw.pressure, g_sensor.last_pressure_collect);
            jb_add_str(&s_rpt_jb, "pressure_status", threshold_pressure_status_str(ps), s_rpt_ts);
            TempStatus_t ts = threshold_get_temp_status(g_sensor.raw.temperature);
            jb_add_str(&s_rpt_jb, "temp_status", threshold_temp_status_str(ts), s_rpt_ts);
        }

        if(g_sensor_count >= 2)
        {
            if(g_sensor.raw2.err != PT304D_OK)
            {
                jb_add_str(&s_rpt_jb, "pressure2_status", "pressure_err", s_rpt_ts);
                jb_add_str(&s_rpt_jb, "temp2_status", "temp_err", s_rpt_ts);
            }
            else
            {
                PressureStatus_t ps2 = threshold_get_pressure_status2(g_sensor.raw2.pressure, g_sensor.last_pressure_collect2);
                jb_add_str(&s_rpt_jb, "pressure2_status", threshold_pressure_status_str(ps2), s_rpt_ts);
                TempStatus_t ts2 = threshold_get_temp_status2(g_sensor.raw2.temperature);
                jb_add_str(&s_rpt_jb, "temp2_status", threshold_temp_status_str(ts2), s_rpt_ts);
            }
        }
    }
    jb_end(&s_rpt_jb);
    EC801E_PublishRawJson(s_rpt_buf);

    // 7. 用户mqtt信息
    if(!user_state)
    {
        user_state = 1;
        jb_begin(&s_rpt_jb, s_rpt_buf, sizeof(s_rpt_buf));
        jb_add_str(&s_rpt_jb, "MQTT_username", g_user_mqtt.username, s_rpt_ts);
        jb_add_str(&s_rpt_jb, "MQTT_password", g_user_mqtt.password, s_rpt_ts);
        jb_add_str(&s_rpt_jb, "MQTT_client_id", g_user_mqtt.client_id, s_rpt_ts);
        jb_add_str(&s_rpt_jb, "MQTT_subject", g_user_mqtt.topic_up, s_rpt_ts);

        jb_add_str(&s_rpt_jb, "window_pattern", g_window.pattern, s_rpt_ts);
        jb_add_int(&s_rpt_jb, "window_end_result", g_window.end_result, s_rpt_ts);
        jb_add_str(&s_rpt_jb, "window_start", g_window.start, s_rpt_ts);
        jb_add_str(&s_rpt_jb, "window_stop", g_window.stop, s_rpt_ts);
        char dot_str3[64]; window_dottime_to_str(g_window.dottime_mask, dot_str3, sizeof(dot_str3));
        jb_add_str(&s_rpt_jb, "window_dottime", dot_str3, s_rpt_ts);

        jb_end(&s_rpt_jb);
        EC801E_PublishRawJson(s_rpt_buf);
    }

    jb_begin(&s_rpt_jb, s_rpt_buf, sizeof(s_rpt_buf));
    // 5. 阀门状态
    {
        const char *valve_status = valve_get_status();
        jb_add_str(&s_rpt_jb, "valve_control", valve_status, s_rpt_ts);
    }

    // 6. 用户IP / 主IP / 副IP
    {
        char ip_str[48];

        snprintf(ip_str, sizeof(ip_str), "%s.%d",
                     g_user_mqtt.server[0] ? g_user_mqtt.server : "0.0.0.0", g_user_mqtt.port);
        jb_add_str(&s_rpt_jb, "user_ip", ip_str, s_rpt_ts);

        snprintf(ip_str, sizeof(ip_str), "%s.%d", g_mqtt.server, g_mqtt.port);
        jb_add_str(&s_rpt_jb, "first_ip", ip_str, s_rpt_ts);

        snprintf(ip_str, sizeof(ip_str), "%s.%d",
                     g_mqtt.server2[0] ? g_mqtt.server2 : "0.0.0.0", g_mqtt.port2);
        jb_add_str(&s_rpt_jb, "scend_ip", ip_str, s_rpt_ts);
		jb_end(&s_rpt_jb);
		EC801E_PublishRawJson(s_rpt_buf);
    }
	
    // 9. 电池电压 + 电压状态
	jb_begin(&s_rpt_jb, s_rpt_buf, sizeof(s_rpt_buf));
    {
        float bat_v = adc_bat_read_voltage();
        if(bat_v >= 0.0f)
        {
            g_sensor.voltage = bat_v;
            g_sensor.last_normal_voltage = bat_v;
        }
        snprintf(v_buf, sizeof(v_buf), "%.2f", g_sensor.voltage);
        jb_add_str(&s_rpt_jb, "voltage", v_buf, s_rpt_ts);

        int volt_alarm = threshold_voltage_alarm(g_sensor.voltage);
        snprintf(va_buf, sizeof(va_buf), "%d", volt_alarm);
        jb_add_str(&s_rpt_jb, "LY06", va_buf, s_rpt_ts);
    }

    jb_end(&s_rpt_jb);
    EC801E_PublishRawJson(s_rpt_buf);
    printf_debug("[SM] upload summary:\r\n CSQ=%s V=%s\r\n", csq_buf, v_buf);
}
