/**
 * @file config_store.c
 * @brief 配置持久化: EEPROM 读写/加载/保存, 所有配置全局变量定义于此
 * @details 与 eeprom_config.c (CLI解析) / config_policy.c (纯策略) 解耦
 */
#include "config_store.h"
#include "eeprom_config.h"
#include "config_policy.h"
#include "app_config.h"
#include "valve_timing.h"
#include "motor.h"        /* MOTOR_CS_TH_DEFAULT / 电流阈值换算 */
#include "usart.h"
#include "delay.h"
#include "threshold.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ==================== 配置全局变量 (编译时默认值) ==================== */
AppInterval_t g_interval = {
    APP_DEFAULT_COLLECT_HOURS,
    APP_DEFAULT_UPLOAD_HOURS,
    0, 0, 0
};
float g_report_delay_min = 0.0f;   /* 周期上报后延分钟 (0~60, 支持小数) */

MqttConfig_t g_mqtt = {
    APP_DEFAULT_MQTT_CLIENT_ID,
    APP_DEFAULT_MQTT_USER,
    APP_DEFAULT_MQTT_PASS,
    APP_DEFAULT_MQTT_SERVER,
    APP_DEFAULT_MQTT_SERVER2,
    APP_DEFAULT_MQTT_TOPIC,
    APP_DEFAULT_MQTT_PORT,
    APP_DEFAULT_MQTT_PORT2
};

UserMqttConfig_t g_user_mqtt = { {0}, 0, {0}, {0}, {0}, {0} };

WindowConfig_t g_window = {
    APP_DEFAULT_WINDOW_PATTERN,
    APP_DEFAULT_WINDOW_END_RESULT,
    APP_DEFAULT_WINDOW_START,
    APP_DEFAULT_WINDOW_STOP,
    {0}
};

CycleConfig_t g_collect_cycle = { APP_DEFAULT_COLLECT_ONCE, APP_DEFAULT_COLLECT_TYPE };
CycleConfig_t g_report_cycle  = { APP_DEFAULT_REPORT_ONCE,  APP_DEFAULT_REPORT_TYPE  };

char g_abnormal_close[16] = APP_DEFAULT_ABNORMAL_CLOSE;
char g_abnormal2_close[16] = APP_DEFAULT_ABNORMAL_CLOSE;   /* 出气端 pv* 策略 */
uint8_t g_abnormal3_close = 0;                             /* 激光甲烷异常关阀 0=关 1=开 */
uint8_t g_valve_last_order = APP_DEFAULT_VALVE_LAST_ORDER;
uint8_t g_valve_alarm_closed = 0;
uint8_t g_valve_pre_alarm_order = 0;
uint8_t g_ch4_alarm_closed = 0;     // 激光甲烷报警关阀标志 (EEPROM持久化, 重启后保持, 平台 valve_open 清除)
uint8_t g_real_enabled = APP_DEFAULT_REAL_ENABLED;
uint8_t g_real_frequency = APP_DEFAULT_REAL_FREQUENCY;
uint8_t g_lcd_enabled = APP_DEFAULT_LCD_ENABLED;
uint8_t g_sensor_count = APP_DEFAULT_SENSOR_COUNT;
uint16_t g_keepalive_sec = APP_MQTT_KEEPALIVE;
uint32_t g_valve_timeout_ms = DEF_VALVE_TIMEOUT_MS;   /* 阀门限位到位等待超时(ms) */
uint16_t g_motor_cs_th = 0;            /* 堵转判定电流阈值(ADC原始值); 启动时由 config_load_all 决定:
                                        * EEPROM 有效值(1~4095) → 用它; 其它情况 → MOTOR_CS_TH_DEFAULT */
uint8_t  g_valve_pos_valid = 0;        /* 阀门位置可信标志: 动作前置0, 到位后置1 (脏标志法) */
uint8_t  g_valve_pos_state = 0;        /* 阀门物理位置(0=关到底,1=开到底); 与"用户最后指令"解耦 */

TempThresholdConfig_t g_tthreshold = {
    DEF_TT_SWITCH, DEF_TT_HIGH, DEF_TT_LOW, DEF_TT_VHIGH, DEF_TT_VLOW
};
TempThresholdConfig_t g_tthreshold2 = {
    DEF_TT2_SWITCH, DEF_TT2_HIGH, DEF_TT2_LOW, DEF_TT2_VHIGH, DEF_TT2_VLOW
};
ValveDelayConfig_t g_vdelay = { DEF_VD_SWITCH, DEF_VD_TIME_MS };
VoltagePowerConfig_t g_spupower = {
    DEF_SP_SWITCH, DEF_SP_HIGH, DEF_SP_LOW, DEF_SP_PROTECT
};
PressureThresholdConfig_t g_pthreshold = {
    DEF_VP_SWITCH, DEF_VP_PPMAX, DEF_VP_PMAX, DEF_VP_PPMIN, DEF_VP_PMIN,
    DEF_VP_MUTATION, DEF_VP_SHOCK
};
PressureThresholdConfig_t g_pthreshold2 = {
    DEF_VP2_SWITCH, DEF_VP2_PPMAX, DEF_VP2_PMAX, DEF_VP2_PPMIN, DEF_VP2_PMIN,
    DEF_VP2_MUTATION, DEF_VP2_SHOCK
};
ValveTimingConfig_t g_valve_timing = {
    DEF_VT_ON_SWITCH, DEF_VT_OFF_SWITCH,
    DEF_VT_OPEN_TIME, DEF_VT_CLOSE_TIME
};

CH4_ThresholdConfig_t g_ch4_threshold = {
    DEF_CH4_ENABLED, DEF_CH4_WARMUP_SEC, DEF_CH4_THRESHOLD
};

uint8_t g_laser_switch = DEF_LASER_SWITCH;
uint8_t g_laser_time_mask[3] = { 0x00, 0x01, 0x10 };  /* 默认 08:00,20:00 (bit8,bit20) */

/* ==================== EEPROM 读写辅助 ==================== */
static float ee_read_float(uint16_t addr)
{
    float v = 0.0f;
    EE_READ(addr, (uint8_t *)&v, sizeof(v));
    return v;
}

static void ee_write_float(uint16_t addr, float v)
{
    EE_WRITE(addr, (uint8_t *)&v, sizeof(v));
}

static void ee_read_string(uint16_t addr, char *buf, uint16_t max_len)
{
    if(max_len == 0) return;
    EE_READ(addr, (uint8_t *)buf, max_len);
    buf[max_len - 1] = '\0';
}

static void ee_write_string(uint16_t addr, const char *str, uint16_t max_len)
{
    uint16_t len = (uint16_t)strlen(str) + 1;
    if(len > max_len) len = max_len;
    EE_WRITE(addr, (const uint8_t *)str, len);
}

/* ==================== 周期重算 ==================== */
void config_recalc_interval(void)
{
    if(g_interval.collect_interval_hours < 0.01f)
        g_interval.collect_interval_hours = 0.01f;
    if(g_interval.upload_interval_hours < g_interval.collect_interval_hours)
        g_interval.upload_interval_hours = g_interval.collect_interval_hours;

    g_interval.collect_interval_min = (uint16_t)(g_interval.collect_interval_hours * 60.0f);
    g_interval.upload_interval_min  = (uint16_t)(g_interval.upload_interval_hours  * 60.0f);
    if(g_interval.collect_interval_min == 0)
        g_interval.collect_interval_min = 1;
    if(g_interval.collect_interval_min > g_interval.upload_interval_min)
        g_interval.upload_interval_min = g_interval.collect_interval_min;

    /* 跨天周期: 上报周期必须是采集周期的整数倍, 否则"上报点必然也是采集点"不成立,
     * 且上报判定只在采集点执行 → 目标不落在采集点会被 120s 宽限判为"消费为 done"而漏报 */
    {
        uint16_t cm = g_interval.collect_interval_min;
        uint16_t um = g_interval.upload_interval_min;
        if(cm > 0u && um > 0u && (um % cm) != 0u)
        {
            uint32_t n = (uint32_t)(um / cm) + 1u;
            uint32_t v = n * (uint32_t)cm;
            if(v > 65535u)
                v = ((uint32_t)65535u / (uint32_t)cm) * (uint32_t)cm;
            g_interval.upload_interval_min   = (uint16_t)v;
            g_interval.upload_interval_hours = (float)v / 60.0f;
            printf_debug("[CFG] upload not multiple of collect: -> %umin\r\n", (unsigned)v);
        }
    }

    g_interval.upload_every_n = (uint16_t)(g_interval.upload_interval_min / g_interval.collect_interval_min);
    if(g_interval.upload_every_n == 0)
        g_interval.upload_every_n = 1;
}

/* ==================== 上报周期防重锁 (持久化) ==================== */
/* 记录最后一次成功上报的"上报周期序号", 保证"一个上报周期只上报一次"(跨重启保持);
 * 与运行期标志位 s_upload_pending / s_last_upload_idx 配合使用 */
void config_save_upload_idx(uint32_t idx)
{
    EE_WRITE_BYTE(EE_ADDR_UPL_MAGIC, EE_MAGIC_UPL);
    EE_WRITE(EE_ADDR_UPL_DONE_IDX, (const uint8_t *)&idx, 4);
}

uint32_t config_load_upload_idx(void)
{
    uint8_t  magic = 0;
    uint32_t idx   = 0xFFFFFFFFu;

    EE_READ(EE_ADDR_UPL_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_UPL)
    {
        EE_READ(EE_ADDR_UPL_DONE_IDX, (uint8_t *)&idx, 4);
        if(idx == 0u) idx = 0xFFFFFFFFu;   /* 无效值按"未上报"处理 */
    }
    return idx;
}

/* ==================== 区块1: 采集/上报间隔 ==================== */
void config_save_interval(void)
{
    EE_WRITE_BYTE(EE_ADDR_INTERVAL_MAGIC, EE_MAGIC_INTERVAL);
    ee_write_float(EE_ADDR_COLLECT_HOURS, g_interval.collect_interval_hours);
    ee_write_float(EE_ADDR_UPLOAD_HOURS,  g_interval.upload_interval_hours);
    config_recalc_interval();
    printf_debug("[CFG] Interval saved: collect=%.2fh upload=%.2fh\r\n",
                 g_interval.collect_interval_hours, g_interval.upload_interval_hours);
}

/* ==================== 区块2: MQTT ==================== */
void config_save_mqtt(void)
{
    EE_WRITE_BYTE(EE_ADDR_MQTT_MAGIC, EE_MAGIC_MQTT);
    ee_write_string(EE_ADDR_MQTT_CLIENT_ID, g_mqtt.client_id, MQTT_FIELD_MAX);
    ee_write_string(EE_ADDR_MQTT_USERNAME,  g_mqtt.username,  MQTT_FIELD_MAX);
    ee_write_string(EE_ADDR_MQTT_PASSWORD,  g_mqtt.password,  MQTT_FIELD_MAX);
    ee_write_string(EE_ADDR_MQTT_SERVER,    g_mqtt.server,    MQTT_FIELD_MAX);
    ee_write_string(EE_ADDR_MQTT_TOPIC_UP,  g_mqtt.topic_up,  MQTT_TOPIC_MAX);
    EE_WRITE(EE_ADDR_MQTT_PORT, (const uint8_t *)&g_mqtt.port, 2);
    ee_write_string(EE_ADDR_MQTT_SERVER2,   g_mqtt.server2,   MQTT_FIELD_MAX);
    EE_WRITE(EE_ADDR_MQTT_PORT2, (const uint8_t *)&g_mqtt.port2, 2);
    printf_debug("[CFG] MQTT saved: %s@%s:%d (s2=%s:%d)\r\n",
                 g_mqtt.client_id, g_mqtt.server, g_mqtt.port,
                 g_mqtt.server2[0] ? g_mqtt.server2 : "(none)", g_mqtt.port2);
}

/* 由 clientId 推导 topic_up
 * clientId 格式: <前缀>&<设备ID>&<产品ID>&<序号>
 * topic 格式:   /<产品ID>/<设备ID>/property/post */
void derive_topic_from_client_id(void)
{
    const char *p = g_mqtt.client_id;
    const char *segs[4] = { NULL, NULL, NULL, NULL };
    int idx = 1;
    char product_id[64] = {0};
    char device_id[64] = {0};
    uint8_t i;

    while(*p && idx <= 3)
    {
        if(*p == '&')
        {
            segs[idx] = p + 1;
            idx++;
        }
        p++;
    }
    if(idx < 3)
    {
        printf_debug("[CFG] Cannot derive topic from clientId\r\n");
        return;
    }

    /* segs[1] = 设备ID, segs[2] = 产品ID */
    i = 0;
    p = segs[1];
    while(*p && *p != '&' && i < 63) device_id[i++] = *p++;
    device_id[i] = '\0';

    i = 0;
    p = segs[2];
    while(*p && *p != '&' && i < 63) product_id[i++] = *p++;
    product_id[i] = '\0';

    snprintf(g_mqtt.topic_up, MQTT_TOPIC_MAX, "/%s/%s/property/post",
             product_id, device_id);
    printf_debug("[CFG] Topic derived: %s\r\n", g_mqtt.topic_up);
}

/* ==================== 区块3: 压力阈值 vpthreshold ==================== */
void config_save_vpthreshold(void)
{
    EE_WRITE_BYTE(EE_ADDR_VPTH_ENABLED, g_pthreshold.enabled);
    ee_write_float(EE_ADDR_VPTH_PPMAX,    g_pthreshold.ppmax);
    ee_write_float(EE_ADDR_VPTH_PMAX,     g_pthreshold.pmax);
    ee_write_float(EE_ADDR_VPTH_PPMIN,    g_pthreshold.ppmin);
    ee_write_float(EE_ADDR_VPTH_PMIN,     g_pthreshold.pmin);
    ee_write_float(EE_ADDR_VPTH_MUTATION, g_pthreshold.mutation);
    ee_write_float(EE_ADDR_VPTH_SHOCK,    g_pthreshold.shock);
    EE_WRITE(EE_ADDR_KEEPALIVE_SEC, (const uint8_t *)&g_keepalive_sec, 2);
    EE_WRITE_BYTE(EE_ADDR_VPTH_MAGIC, EE_MAGIC_VPTH);
    printf_debug("[CFG] VPTH saved: en=%d ppmax=%.1f pmax=%.1f ppmin=%.1f pmin=%.1f mut=%.1f shock=%.1f ka=%d\r\n",
                 g_pthreshold.enabled, g_pthreshold.ppmax, g_pthreshold.pmax,
                 g_pthreshold.ppmin, g_pthreshold.pmin,
                 g_pthreshold.mutation, g_pthreshold.shock, g_keepalive_sec);
}

/* ==================== 区块4: 温度阈值 Tthreshold ==================== */
void config_save_tthreshold(void)
{
    EE_WRITE_BYTE(EE_ADDR_TTH_ENABLED, g_tthreshold.enabled);
    ee_write_float(EE_ADDR_TTH_HIGH,  g_tthreshold.high);
    ee_write_float(EE_ADDR_TTH_LOW,   g_tthreshold.low);
    ee_write_float(EE_ADDR_TTH_VHIGH, g_tthreshold.vhigh);
    ee_write_float(EE_ADDR_TTH_VLOW,  g_tthreshold.vlow);
    EE_WRITE_BYTE(EE_ADDR_TTH_MAGIC, EE_MAGIC_TTH);
    printf_debug("[CFG] TTH saved: en=%d high=%.1f low=%.1f vhigh=%.1f vlow=%.1f\r\n",
                 g_tthreshold.enabled, g_tthreshold.high, g_tthreshold.low,
                 g_tthreshold.vhigh, g_tthreshold.vlow);
}

/* ==================== 区块5: 电压检测 spupower ==================== */
void config_save_spupower(void)
{
    EE_WRITE_BYTE(EE_ADDR_SPUP_ENABLED, g_spupower.enabled);
    ee_write_float(EE_ADDR_SPUP_HIGH,    g_spupower.high);
    ee_write_float(EE_ADDR_SPUP_LOW,     g_spupower.low);
    ee_write_float(EE_ADDR_SPUP_PROTECT, g_spupower.protect);
    EE_WRITE_BYTE(EE_ADDR_SPUP_MAGIC, EE_MAGIC_SPUP);
    printf_debug("[CFG] SPUP saved: en=%d high=%.2f low=%.2f protect=%.2f\r\n",
                 g_spupower.enabled, g_spupower.high, g_spupower.low, g_spupower.protect);
}

/* ==================== 区块7: 阀门到位延时 valve_delay ==================== */
void config_save_vdelay(void)
{
    if(g_vdelay.delay_ms > VDLY_DELAY_MS_MAX)
        g_vdelay.delay_ms = VDLY_DELAY_MS_MAX;
    EE_WRITE_BYTE(EE_ADDR_VDLY_ENABLED, g_vdelay.enabled);
    EE_WRITE(EE_ADDR_VDLY_DELAY_MS, (const uint8_t *)&g_vdelay.delay_ms, 2);
    EE_WRITE(EE_ADDR_VALVE_TIMEOUT, (const uint8_t *)&g_valve_timeout_ms, 4);
    EE_WRITE_BYTE(EE_ADDR_VDLY_MAGIC, EE_MAGIC_VDLY);
    printf_debug("[CFG] VDLY saved: en=%d delay_ms=%d\r\n",
                 g_vdelay.enabled, g_vdelay.delay_ms);
}

/* ==================== 区块8: 第二路压力阈值 vpthreshold2 ==================== */
void config_save_vpthreshold2(void)
{
    EE_WRITE_BYTE(EE_ADDR_VPTH2_ENABLED, g_pthreshold2.enabled);
    ee_write_float(EE_ADDR_VPTH2_PPMAX,    g_pthreshold2.ppmax);
    ee_write_float(EE_ADDR_VPTH2_PMAX,     g_pthreshold2.pmax);
    ee_write_float(EE_ADDR_VPTH2_PPMIN,    g_pthreshold2.ppmin);
    ee_write_float(EE_ADDR_VPTH2_PMIN,     g_pthreshold2.pmin);
    ee_write_float(EE_ADDR_VPTH2_MUTATION, g_pthreshold2.mutation);
    ee_write_float(EE_ADDR_VPTH2_SHOCK,    g_pthreshold2.shock);
    EE_WRITE_BYTE(EE_ADDR_VPTH2_MAGIC, EE_MAGIC_VPTH2);
    printf_debug("[CFG] VPTH2 saved: en=%d ppmax=%.1f pmax=%.1f ppmin=%.1f pmin=%.1f mut=%.1f shock=%.1f\r\n",
                 g_pthreshold2.enabled, g_pthreshold2.ppmax, g_pthreshold2.pmax,
                 g_pthreshold2.ppmin, g_pthreshold2.pmin,
                 g_pthreshold2.mutation, g_pthreshold2.shock);
}

/* ==================== 区块9: 第二路温度阈值 Tthreshold2 ==================== */
void config_save_tthreshold2(void)
{
    EE_WRITE_BYTE(EE_ADDR_TTH2_ENABLED, g_tthreshold2.enabled);
    ee_write_float(EE_ADDR_TTH2_HIGH,  g_tthreshold2.high);
    ee_write_float(EE_ADDR_TTH2_LOW,   g_tthreshold2.low);
    ee_write_float(EE_ADDR_TTH2_VHIGH, g_tthreshold2.vhigh);
    ee_write_float(EE_ADDR_TTH2_VLOW,  g_tthreshold2.vlow);
    EE_WRITE_BYTE(EE_ADDR_TTH2_MAGIC, EE_MAGIC_TTH2);
    printf_debug("[CFG] TTH2 saved: en=%d high=%.1f low=%.1f vhigh=%.1f vlow=%.1f\r\n",
                 g_tthreshold2.enabled, g_tthreshold2.high, g_tthreshold2.low,
                 g_tthreshold2.vhigh, g_tthreshold2.vlow);
}

/* ==================== 区块10: 用户端MQTT ==================== */
void config_save_user_mqtt(void)
{
    EE_WRITE_BYTE(EE_ADDR_UMQTT_MAGIC, EE_MAGIC_UMQTT);
    ee_write_string(EE_ADDR_UMQTT_SERVER,   g_user_mqtt.server,    MQTT_FIELD_MAX);
    EE_WRITE(EE_ADDR_UMQTT_PORT, (const uint8_t *)&g_user_mqtt.port, 2);
    ee_write_string(EE_ADDR_UMQTT_USERNAME, g_user_mqtt.username,  MQTT_FIELD_MAX);
    ee_write_string(EE_ADDR_UMQTT_PASSWORD, g_user_mqtt.password,  MQTT_FIELD_MAX);
    ee_write_string(EE_ADDR_UMQTT_CLIENT_ID,g_user_mqtt.client_id, MQTT_FIELD_MAX);
    ee_write_string(EE_ADDR_UMQTT_TOPIC_UP, g_user_mqtt.topic_up,  MQTT_TOPIC_MAX);
    printf_debug("[CFG] UMQTT saved: %s@%s:%d topic=%s\r\n",
                 g_user_mqtt.client_id, g_user_mqtt.server,
                 g_user_mqtt.port, g_user_mqtt.topic_up);
}

/* ==================== 区块11: 定时开关阀 ==================== */
void config_save_valve_timing(void)
{
    EE_WRITE_BYTE(EE_ADDR_VTIMING_ON_SWITCH,  g_valve_timing.on_switch);
    EE_WRITE_BYTE(EE_ADDR_VTIMING_OFF_SWITCH, g_valve_timing.off_switch);
    ee_write_string(EE_ADDR_VTIMING_OPEN,  g_valve_timing.open_time,  11);
    ee_write_string(EE_ADDR_VTIMING_CLOSE, g_valve_timing.close_time, 11);
    EE_WRITE_BYTE(EE_ADDR_VTIMING_MAGIC, EE_MAGIC_VTIMING);
    printf_debug("[CFG] VTIMING saved: on=%d off=%d open=%s close=%s\r\n",
                 g_valve_timing.on_switch, g_valve_timing.off_switch,
                 g_valve_timing.open_time, g_valve_timing.close_time);
}

/* ==================== 区块12: CH4 甲烷门限 ==================== */
void config_save_ch4_threshold(void)
{
    EE_WRITE_BYTE(EE_ADDR_CH4_ENABLED, g_ch4_threshold.enabled);
    EE_WRITE_BYTE(EE_ADDR_CH4_WARMUP_SEC, g_ch4_threshold.warmup_sec);
    ee_write_float(EE_ADDR_CH4_THRESHOLD, g_ch4_threshold.threshold);
    EE_WRITE_BYTE(EE_ADDR_CH4_MAGIC, EE_MAGIC_CH4);
    printf_debug("[CFG] CH4 saved: en=%d warmup=%ds threshold=%.2f\r\n",
                 g_ch4_threshold.enabled, g_ch4_threshold.warmup_sec,
                 g_ch4_threshold.threshold);
}

/* ==================== 区块13: 激光甲烷定时采集 ==================== */
void config_save_laser(void)
{
    char tstr[96] = {0};
    EE_WRITE_BYTE(EE_ADDR_LASER_MAGIC, EE_MAGIC_LASER);
    EE_WRITE_BYTE(EE_ADDR_LASER_SWITCH, g_laser_switch);
    EE_WRITE(EE_ADDR_LASER_TIME, g_laser_time_mask, 3);
    window_dottime_to_str(g_laser_time_mask, tstr, sizeof(tstr));
    printf_debug("[CFG] LASER saved: switch=%d time=%s\r\n", g_laser_switch, tstr);
}

/* ==================== 区块14: 出气端/激光异常关阀 ==================== */
void config_save_abn23(void)
{
    EE_WRITE_BYTE(EE_ADDR_ABN2_MAGIC, EE_MAGIC_ABN23);
    ee_write_string(EE_ADDR_ABNORMAL2_CLOSE, g_abnormal2_close, 16);
    EE_WRITE_BYTE(EE_ADDR_ABNORMAL3_CLOSE, g_abnormal3_close);
    printf_debug("[CFG] ABN23 saved: abn2=%s abn3=%d\r\n",
                 g_abnormal2_close, g_abnormal3_close);
}

/* ==================== 区块6: 扩展配置 ==================== */
void config_save_ext(void)
{
    EE_WRITE_BYTE(EE_ADDR_EXT_MAGIC, EE_MAGIC_EXT);
    ee_write_string(EE_ADDR_WINDOW_PATTERN, g_window.pattern, 8);
    EE_WRITE_BYTE(EE_ADDR_WINDOW_END_RESULT, g_window.end_result);
    ee_write_string(EE_ADDR_WINDOW_START_STR, g_window.start, 8);
    ee_write_string(EE_ADDR_WINDOW_STOP_STR,  g_window.stop,  8);
    EE_WRITE(EE_ADDR_WINDOW_DOTTIME_MASK, g_window.dottime_mask, 3);
    EE_WRITE_BYTE(EE_ADDR_WINDOW_DOTTIME, window_dottime_first(g_window.dottime_mask));
    EE_WRITE(EE_ADDR_COLLECT_ONCE, (const uint8_t *)&g_collect_cycle.once, 2);
    EE_WRITE_BYTE(EE_ADDR_COLLECT_TYPE, (uint8_t)g_collect_cycle.type);
    EE_WRITE(EE_ADDR_REPORT_ONCE, (const uint8_t *)&g_report_cycle.once, 2);
    EE_WRITE_BYTE(EE_ADDR_REPORT_TYPE, (uint8_t)g_report_cycle.type);
    ee_write_string(EE_ADDR_ABNORMAL_CLOSE, g_abnormal_close, 16);
    EE_WRITE_BYTE(EE_ADDR_VALVE_LAST_ORDER, g_valve_last_order);
    EE_WRITE_BYTE(EE_ADDR_REAL_ENABLED, g_real_enabled);
    EE_WRITE_BYTE(EE_ADDR_REAL_FREQUENCY, g_real_frequency);
    EE_WRITE_BYTE(EE_ADDR_VALVE_ALARM_CLOSED, g_valve_alarm_closed);
    EE_WRITE_BYTE(EE_ADDR_VALVE_PRE_ALARM_ORDER, g_valve_pre_alarm_order);
    EE_WRITE_BYTE(EE_ADDR_CH4_ALARM_CLOSED, g_ch4_alarm_closed);
    ee_write_float(EE_ADDR_REPORT_DELAY, g_report_delay_min);
    EE_WRITE_BYTE(EE_ADDR_LCD_ENABLED, g_lcd_enabled);
    EE_WRITE_BYTE(EE_ADDR_VLP_FLAG, voltage_low_power);
    EE_WRITE(EE_ADDR_MOTOR_CS_TH, (const uint8_t *)&g_motor_cs_th, 2);
    EE_WRITE_BYTE(EE_ADDR_VALVE_POS_VALID, g_valve_pos_valid);
    EE_WRITE_BYTE(EE_ADDR_VALVE_POS_STATE, g_valve_pos_state);
    printf_debug("[CFG] EXT saved\r\n");
}

/* ---------- 单字段写入: 避免整块 EXT 重写(延长 EEPROM 寿命/缩短写入时间) ---------- */
void config_save_motor_cs_th(void)
{
    EE_WRITE(EE_ADDR_MOTOR_CS_TH, (const uint8_t *)&g_motor_cs_th, 2);
}

void config_save_valve_pos_valid(void)
{
    EE_WRITE_BYTE(EE_ADDR_VALVE_POS_VALID, g_valve_pos_valid);
}

/* ==================== 心跳 ==================== */
uint16_t config_get_keepalive(void)
{
    return (g_keepalive_sec >= 30) ? g_keepalive_sec : (uint16_t)APP_MQTT_KEEPALIVE;
}

/* ==================== 全部配置加载 (上电调用) ==================== */
void config_load_all(void)
{
    uint8_t magic = 0xFF;
    char dot_str[64] = {0};

    /* ---------- 区块1: 采集/上报间隔 ---------- */
    EE_READ(EE_ADDR_INTERVAL_MAGIC, &magic, 1);
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

    /* ---------- 区块2: MQTT ---------- */
    EE_READ(EE_ADDR_MQTT_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_MQTT)
    {
        ee_read_string(EE_ADDR_MQTT_CLIENT_ID, g_mqtt.client_id, MQTT_FIELD_MAX);
        ee_read_string(EE_ADDR_MQTT_USERNAME,  g_mqtt.username,  MQTT_FIELD_MAX);
        ee_read_string(EE_ADDR_MQTT_PASSWORD,  g_mqtt.password,  MQTT_FIELD_MAX);
        ee_read_string(EE_ADDR_MQTT_SERVER,    g_mqtt.server,    MQTT_FIELD_MAX);
        ee_read_string(EE_ADDR_MQTT_TOPIC_UP,  g_mqtt.topic_up,  MQTT_TOPIC_MAX);
        EE_READ(EE_ADDR_MQTT_PORT, (uint8_t *)&g_mqtt.port, 2);
        ee_read_string(EE_ADDR_MQTT_SERVER2,   g_mqtt.server2,   MQTT_FIELD_MAX);
        EE_READ(EE_ADDR_MQTT_PORT2, (uint8_t *)&g_mqtt.port2, 2);
        if(g_mqtt.port2 == 0 || g_mqtt.port2 == 0xFFFF)
            g_mqtt.port2 = g_mqtt.port;
        printf_debug("[CFG] EEPROM MQTT loaded: %s@%s:%d (s2=%s:%d)\r\n",
                     g_mqtt.client_id, g_mqtt.server, g_mqtt.port,
                     g_mqtt.server2[0] ? g_mqtt.server2 : "(none)", g_mqtt.port2);
    }
    else
    {
        strncpy(g_mqtt.client_id, APP_DEFAULT_MQTT_CLIENT_ID, MQTT_FIELD_MAX - 1);
        strncpy(g_mqtt.username,  APP_DEFAULT_MQTT_USER,      MQTT_FIELD_MAX - 1);
        strncpy(g_mqtt.password,  APP_DEFAULT_MQTT_PASS,      MQTT_FIELD_MAX - 1);
        strncpy(g_mqtt.server,    APP_DEFAULT_MQTT_SERVER,    MQTT_FIELD_MAX - 1);
        strncpy(g_mqtt.server2,   APP_DEFAULT_MQTT_SERVER2,   MQTT_FIELD_MAX - 1);
        strncpy(g_mqtt.topic_up,  APP_DEFAULT_MQTT_TOPIC,     MQTT_TOPIC_MAX - 1);
        g_mqtt.port  = APP_DEFAULT_MQTT_PORT;
        g_mqtt.port2 = APP_DEFAULT_MQTT_PORT2;
        printf_debug("[CFG] No MQTT, using defaults: %s@%s:%d (s2=%s:%d) [waiting setup]\r\n",
                     g_mqtt.client_id, g_mqtt.server, g_mqtt.port,
                     g_mqtt.server2[0] ? g_mqtt.server2 : "(none)", g_mqtt.port2);
        delay_us(6000);
    }

    /* ---------- 区块3: 压力阈值 vpthreshold ---------- */
    EE_READ(EE_ADDR_VPTH_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_VPTH)
    {
        EE_READ(EE_ADDR_VPTH_ENABLED, &g_pthreshold.enabled, 1);
        g_pthreshold.ppmax    = ee_read_float(EE_ADDR_VPTH_PPMAX);
        g_pthreshold.pmax     = ee_read_float(EE_ADDR_VPTH_PMAX);
        g_pthreshold.ppmin    = ee_read_float(EE_ADDR_VPTH_PPMIN);
        g_pthreshold.pmin     = ee_read_float(EE_ADDR_VPTH_PMIN);
        g_pthreshold.mutation = ee_read_float(EE_ADDR_VPTH_MUTATION);
        g_pthreshold.shock    = ee_read_float(EE_ADDR_VPTH_SHOCK);
        EE_READ(EE_ADDR_KEEPALIVE_SEC, (uint8_t *)&g_keepalive_sec, 2);
        if(g_pthreshold.enabled >= 2) g_pthreshold.enabled = 0;
        if(g_keepalive_sec == 0 || g_keepalive_sec == 0xFFFF)
            g_keepalive_sec = APP_MQTT_KEEPALIVE;
        printf_debug("[CFG] VPTH: en=%d ppmax=%.1f pmax=%.1f ppmin=%.1f pmin=%.1f mut=%.1f shock=%.1f ka=%d\r\n",
                     g_pthreshold.enabled, g_pthreshold.ppmax, g_pthreshold.pmax,
                     g_pthreshold.ppmin, g_pthreshold.pmin,
                     g_pthreshold.mutation, g_pthreshold.shock, g_keepalive_sec);
    }
    else
    {
        g_pthreshold.enabled = DEF_VP_SWITCH;
        g_pthreshold.ppmax   = DEF_VP_PPMAX;
        g_pthreshold.pmax    = DEF_VP_PMAX;
        g_pthreshold.ppmin   = DEF_VP_PPMIN;
        g_pthreshold.pmin    = DEF_VP_PMIN;
        g_pthreshold.mutation= DEF_VP_MUTATION;
        g_pthreshold.shock   = DEF_VP_SHOCK;
        g_keepalive_sec      = APP_MQTT_KEEPALIVE;
        printf_debug("[CFG] No VPTH, using defaults: en=%d ppmax=%.1f pmax=%.1f ppmin=%.1f pmin=%.1f mut=%.1f shock=%.1f\r\n",
                     g_pthreshold.enabled, g_pthreshold.ppmax, g_pthreshold.pmax,
                     g_pthreshold.ppmin, g_pthreshold.pmin,
                     g_pthreshold.mutation, g_pthreshold.shock);
        config_save_vpthreshold();
    }

    /* ---------- 区块4: 温度阈值 Tthreshold ---------- */
    EE_READ(EE_ADDR_TTH_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_TTH)
    {
        EE_READ(EE_ADDR_TTH_ENABLED, &g_tthreshold.enabled, 1);
        g_tthreshold.high  = ee_read_float(EE_ADDR_TTH_HIGH);
        g_tthreshold.low   = ee_read_float(EE_ADDR_TTH_LOW);
        g_tthreshold.vhigh = ee_read_float(EE_ADDR_TTH_VHIGH);
        g_tthreshold.vlow  = ee_read_float(EE_ADDR_TTH_VLOW);
        if(g_tthreshold.enabled >= 2) g_tthreshold.enabled = 0;
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

    /* ---------- 区块5: 电压检测 spupower ---------- */
    EE_READ(EE_ADDR_SPUP_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_SPUP)
    {
        EE_READ(EE_ADDR_SPUP_ENABLED, &g_spupower.enabled, 1);
        g_spupower.high    = ee_read_float(EE_ADDR_SPUP_HIGH);
        g_spupower.low     = ee_read_float(EE_ADDR_SPUP_LOW);
        g_spupower.protect = ee_read_float(EE_ADDR_SPUP_PROTECT);
        if(g_spupower.enabled >= 2) g_spupower.enabled = 0;
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
                     g_spupower.enabled, g_spupower.high, g_spupower.low, g_spupower.protect);
        config_save_spupower();
    }

    /* ---------- 区块7: 阀门到位延时 ---------- */
    EE_READ(EE_ADDR_VDLY_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_VDLY)
    {
        EE_READ(EE_ADDR_VDLY_ENABLED, &g_vdelay.enabled, 1);
        EE_READ(EE_ADDR_VDLY_DELAY_MS, (uint8_t *)&g_vdelay.delay_ms, 2);
        EE_READ(EE_ADDR_VALVE_TIMEOUT, (uint8_t *)&g_valve_timeout_ms, 4);
        /* 合法性: 0/0xFFFFFFFF → 默认; 越界 → 钳位边界 */
        if(g_valve_timeout_ms == 0 || g_valve_timeout_ms == 0xFFFFFFFFu)
            g_valve_timeout_ms = DEF_VALVE_TIMEOUT_MS;
        if(g_valve_timeout_ms < VALVE_TIMEOUT_MS_MIN) g_valve_timeout_ms = VALVE_TIMEOUT_MS_MIN;
        if(g_valve_timeout_ms > VALVE_TIMEOUT_MS_MAX) g_valve_timeout_ms = VALVE_TIMEOUT_MS_MAX;
        if(g_vdelay.enabled >= 2) g_vdelay.enabled = 0;
        printf_debug("[CFG] VDLY: en=%d delay_ms=%d\r\n",
                     g_vdelay.enabled, g_vdelay.delay_ms);
    }
    else
    {
        g_vdelay.enabled = DEF_VD_SWITCH;
        g_vdelay.delay_ms = DEF_VD_TIME_MS;
        g_valve_timeout_ms = DEF_VALVE_TIMEOUT_MS;
        printf_debug("[CFG] No VDLY, using defaults: en=%d delay_ms=%d\r\n",
                     g_vdelay.enabled, g_vdelay.delay_ms);
        config_save_vdelay();
    }

    /* ---------- 区块8: 第二路压力阈值 ---------- */
    EE_READ(EE_ADDR_VPTH2_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_VPTH2)
    {
        EE_READ(EE_ADDR_VPTH2_ENABLED, &g_pthreshold2.enabled, 1);
        g_pthreshold2.ppmax    = ee_read_float(EE_ADDR_VPTH2_PPMAX);
        g_pthreshold2.pmax     = ee_read_float(EE_ADDR_VPTH2_PMAX);
        g_pthreshold2.ppmin    = ee_read_float(EE_ADDR_VPTH2_PPMIN);
        g_pthreshold2.pmin     = ee_read_float(EE_ADDR_VPTH2_PMIN);
        g_pthreshold2.mutation = ee_read_float(EE_ADDR_VPTH2_MUTATION);
        g_pthreshold2.shock    = ee_read_float(EE_ADDR_VPTH2_SHOCK);
        if(g_pthreshold2.enabled >= 2) g_pthreshold2.enabled = 0;
        printf_debug("[CFG] VPTH2: en=%d ppmax=%.1f pmax=%.1f ppmin=%.1f pmin=%.1f mut=%.1f shock=%.1f\r\n",
                     g_pthreshold2.enabled, g_pthreshold2.ppmax, g_pthreshold2.pmax,
                     g_pthreshold2.ppmin, g_pthreshold2.pmin,
                     g_pthreshold2.mutation, g_pthreshold2.shock);
    }
    else
    {
        g_pthreshold2.enabled = DEF_VP2_SWITCH;
        g_pthreshold2.ppmax   = DEF_VP2_PPMAX;
        g_pthreshold2.pmax    = DEF_VP2_PMAX;
        g_pthreshold2.ppmin   = DEF_VP2_PPMIN;
        g_pthreshold2.pmin    = DEF_VP2_PMIN;
        g_pthreshold2.mutation= DEF_VP2_MUTATION;
        g_pthreshold2.shock   = DEF_VP2_SHOCK;
        printf_debug("[CFG] No VPTH2, using defaults\r\n");
        config_save_vpthreshold2();
    }

    /* ---------- 区块9: 第二路温度阈值 ---------- */
    EE_READ(EE_ADDR_TTH2_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_TTH2)
    {
        EE_READ(EE_ADDR_TTH2_ENABLED, &g_tthreshold2.enabled, 1);
        g_tthreshold2.high  = ee_read_float(EE_ADDR_TTH2_HIGH);
        g_tthreshold2.low   = ee_read_float(EE_ADDR_TTH2_LOW);
        g_tthreshold2.vhigh = ee_read_float(EE_ADDR_TTH2_VHIGH);
        g_tthreshold2.vlow  = ee_read_float(EE_ADDR_TTH2_VLOW);
        if(g_tthreshold2.enabled >= 2) g_tthreshold2.enabled = 0;
        printf_debug("[CFG] TTH2: en=%d high=%.1f low=%.1f vhigh=%.1f vlow=%.1f\r\n",
                     g_tthreshold2.enabled, g_tthreshold2.high, g_tthreshold2.low,
                     g_tthreshold2.vhigh, g_tthreshold2.vlow);
    }
    else
    {
        g_tthreshold2.enabled = DEF_TT2_SWITCH;
        g_tthreshold2.high    = DEF_TT2_HIGH;
        g_tthreshold2.low     = DEF_TT2_LOW;
        g_tthreshold2.vhigh   = DEF_TT2_VHIGH;
        g_tthreshold2.vlow    = DEF_TT2_VLOW;
        printf_debug("[CFG] No TTH2, using defaults\r\n");
        config_save_tthreshold2();
    }

    /* ---------- 区块10: 用户端MQTT ---------- */
    EE_READ(EE_ADDR_UMQTT_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_UMQTT)
    {
        ee_read_string(EE_ADDR_UMQTT_SERVER,    g_user_mqtt.server,    MQTT_FIELD_MAX);
        EE_READ(EE_ADDR_UMQTT_PORT, (uint8_t *)&g_user_mqtt.port, 2);
        ee_read_string(EE_ADDR_UMQTT_USERNAME,  g_user_mqtt.username,  MQTT_FIELD_MAX);
        ee_read_string(EE_ADDR_UMQTT_PASSWORD,  g_user_mqtt.password,  MQTT_FIELD_MAX);
        ee_read_string(EE_ADDR_UMQTT_CLIENT_ID, g_user_mqtt.client_id, MQTT_FIELD_MAX);
        ee_read_string(EE_ADDR_UMQTT_TOPIC_UP,  g_user_mqtt.topic_up,  MQTT_TOPIC_MAX);
        if(g_user_mqtt.port == 0 || g_user_mqtt.port == 0xFFFF)
            g_user_mqtt.port = 13000;
        /* topic 为空时由 client_id 推导 */
        if(g_user_mqtt.topic_up[0] == '\0' || g_user_mqtt.topic_up[0] == 0xFF)
        {
            if(g_user_mqtt.client_id[0] != '\0')
                snprintf(g_user_mqtt.topic_up, MQTT_TOPIC_MAX, "/155/%s/property/post", g_user_mqtt.client_id);
            else
                g_user_mqtt.topic_up[0] = '\0';
        }
        printf_debug("[CFG] UMQTT: %s@%s:%d topic=%s\r\n",
                     g_user_mqtt.client_id[0] ? g_user_mqtt.client_id : "(none)",
                     g_user_mqtt.server[0] ? g_user_mqtt.server : "(none)",
                     g_user_mqtt.port, g_user_mqtt.topic_up);
    }
    else
    {
        memset(&g_user_mqtt, 0, sizeof(g_user_mqtt));
        printf_debug("[CFG] No UMQTT config (user platform disabled)\r\n");
    }

    /* ---------- 区块11: 定时开关阀 ---------- */
    EE_READ(EE_ADDR_VTIMING_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_VTIMING)
    {
        EE_READ(EE_ADDR_VTIMING_ON_SWITCH, &g_valve_timing.on_switch, 1);
        EE_READ(EE_ADDR_VTIMING_OFF_SWITCH, &g_valve_timing.off_switch, 1);
        ee_read_string(EE_ADDR_VTIMING_OPEN,  g_valve_timing.open_time,  11);
        ee_read_string(EE_ADDR_VTIMING_CLOSE, g_valve_timing.close_time, 11);
        if(g_valve_timing.on_switch >= 2)  g_valve_timing.on_switch = 0;
        if(g_valve_timing.off_switch >= 2) g_valve_timing.off_switch = 0;
        if(!valve_timing_time_valid(g_valve_timing.open_time))
        {
            strncpy(g_valve_timing.open_time, DEF_VT_OPEN_TIME, 10);
            g_valve_timing.open_time[10] = '\0';
        }
        if(!valve_timing_time_valid(g_valve_timing.close_time))
        {
            strncpy(g_valve_timing.close_time, DEF_VT_CLOSE_TIME, 10);
            g_valve_timing.close_time[10] = '\0';
        }
        printf_debug("[CFG] VTIMING: on=%d off=%d open=%s close=%s\r\n",
                     g_valve_timing.on_switch, g_valve_timing.off_switch,
                     g_valve_timing.open_time, g_valve_timing.close_time);
    }
    else
    {
        g_valve_timing.on_switch = DEF_VT_ON_SWITCH;
        g_valve_timing.off_switch = DEF_VT_OFF_SWITCH;
        strncpy(g_valve_timing.open_time, DEF_VT_OPEN_TIME, 10);
        g_valve_timing.open_time[10] = '\0';
        strncpy(g_valve_timing.close_time, DEF_VT_CLOSE_TIME, 10);
        g_valve_timing.close_time[10] = '\0';
        printf_debug("[CFG] No VTIMING config (disabled, default 00:00 daily)\r\n");
    }

    /* ---------- 区块6: 扩展配置 ---------- */
    EE_READ(EE_ADDR_EXT_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_EXT)
    {
        uint8_t legacy_dot = 0xFF;
        ee_read_string(EE_ADDR_WINDOW_PATTERN, g_window.pattern, 8);
        EE_READ(EE_ADDR_WINDOW_END_RESULT, &g_window.end_result, 1);
        ee_read_string(EE_ADDR_WINDOW_START_STR, g_window.start, 8);
        ee_read_string(EE_ADDR_WINDOW_STOP_STR,  g_window.stop,  8);
        EE_READ(EE_ADDR_WINDOW_DOTTIME_MASK, g_window.dottime_mask, 3);
        /* 兼容旧版单字节 dottime: 新位图为全 0xFF 时读取旧字段 */
        if(g_window.dottime_mask[0] == 0xFF && g_window.dottime_mask[1] == 0xFF &&
           g_window.dottime_mask[2] == 0xFF)
        {
            EE_READ(EE_ADDR_WINDOW_DOTTIME, &legacy_dot, 1);
            window_dottime_clear(g_window.dottime_mask);
            if(legacy_dot <= 23)
                window_dottime_set(g_window.dottime_mask, legacy_dot);
        }
        EE_READ(EE_ADDR_COLLECT_ONCE, (uint8_t *)&g_collect_cycle.once, 2);
        EE_READ(EE_ADDR_COLLECT_TYPE, (uint8_t *)&g_collect_cycle.type, 1);
        EE_READ(EE_ADDR_REPORT_ONCE,  (uint8_t *)&g_report_cycle.once,  2);
        EE_READ(EE_ADDR_REPORT_TYPE,  (uint8_t *)&g_report_cycle.type,  1);
        /* 周期越界钳制 (防 EEPROM 脏数据/旧版写入导致分钟域溢出) */
        if(cycle_clamp(&g_collect_cycle))
            printf_debug("[CFG] collect cycle clamped to %u*%s\r\n",
                         g_collect_cycle.once, cycle_type_to_str(g_collect_cycle.type));
        if(cycle_clamp(&g_report_cycle))
            printf_debug("[CFG] report cycle clamped to %u*%s\r\n",
                         g_report_cycle.once, cycle_type_to_str(g_report_cycle.type));
        ee_read_string(EE_ADDR_ABNORMAL_CLOSE, g_abnormal_close, 16);
        EE_READ(EE_ADDR_VALVE_LAST_ORDER, &g_valve_last_order, 1);
        EE_READ(EE_ADDR_REAL_ENABLED, &g_real_enabled, 1);
        EE_READ(EE_ADDR_REAL_FREQUENCY, &g_real_frequency, 1);
        EE_READ(EE_ADDR_VALVE_ALARM_CLOSED, &g_valve_alarm_closed, 1);
        EE_READ(EE_ADDR_VALVE_PRE_ALARM_ORDER, &g_valve_pre_alarm_order, 1);
        EE_READ(EE_ADDR_CH4_ALARM_CLOSED, &g_ch4_alarm_closed, 1);

        /* 堵转电流阈值 + 阀门位置可信标志 (新板堵转方案) */
        {
            uint16_t cs_th = 0xFFFF;
            EE_READ(EE_ADDR_MOTOR_CS_TH, (uint8_t *)&cs_th, 2);
            /* 有效范围 1~4095 用 EEPROM; 0(未标定)/0xFFFF(从未写过)/越界 → 用编译期出厂默认。
             * 注: 需要"关闭堵转判定"时, 把 MOTOR_CS_TH_MA_DEFAULT 设为 0 重新编译即可。 */
            if(cs_th >= 1u && cs_th <= 4095u)
                g_motor_cs_th = cs_th;
            else
                g_motor_cs_th = MOTOR_CS_TH_DEFAULT;
            EE_READ(EE_ADDR_VALVE_POS_VALID, &g_valve_pos_valid, 1);
            if(g_valve_pos_valid >= 2) g_valve_pos_valid = 0;
            EE_READ(EE_ADDR_VALVE_POS_STATE, &g_valve_pos_state, 1);
            if(g_valve_pos_state >= 2)
            {
                /* 未写过(0xFF)/非法值: 视为"位置未知", 不冒充"阀关",
                 * 由下一次阀门动作先回零重建位置 (安全侧处理) */
                g_valve_pos_state = 0;
                g_valve_pos_valid = 0;
            }
            printf_debug("[CFG] motor: cs_th=%u(%lumA) pos_valid=%d\r\n",
                         g_motor_cs_th, (unsigned long)MOTOR_CS_RAW_TO_MA(g_motor_cs_th),
                         g_valve_pos_valid);
        }
        g_report_delay_min = ee_read_float(EE_ADDR_REPORT_DELAY);
        /* 合法性: NaN/越界(0~60)回退 0 */
        if(!(g_report_delay_min >= 0.0f && g_report_delay_min <= 60.0f))
            g_report_delay_min = 0.0f;
        EE_READ(EE_ADDR_LCD_ENABLED, &g_lcd_enabled, 1);
        EE_READ(EE_ADDR_VLP_FLAG, &voltage_low_power, 1);

        /* 合理性修正 */
        if(g_real_enabled >= 2)   g_real_enabled = 0;
        if(g_real_frequency == 0 || g_real_frequency >= 11) g_real_frequency = APP_DEFAULT_REAL_FREQUENCY;
        if(g_lcd_enabled >= 2)    g_lcd_enabled = 1;
        if(voltage_low_power >= 2) voltage_low_power = 0;
        if(g_ch4_alarm_closed >= 2) g_ch4_alarm_closed = 0;

        window_dottime_to_str(g_window.dottime_mask, dot_str, sizeof(dot_str));
        printf_debug("[CFG] EXT: win=%s er=%d start=%s stop=%s dot=%s\r\n",
                     g_window.pattern, g_window.end_result,
                     g_window.start, g_window.stop, dot_str);
        printf_debug("[CFG] EXT: collect=%d*%s report=%d*%s abnormal=%s valve=%d\r\n",
                     g_collect_cycle.once, cycle_type_to_str(g_collect_cycle.type),
                     g_report_cycle.once, cycle_type_to_str(g_report_cycle.type),
                     g_abnormal_close, g_valve_last_order);
        printf_debug("[CFG] EXT: real_enabled=%d real_frequency=%ds lcd=%d sensor_count=%d\r\n",
                     g_real_enabled, g_real_frequency, g_lcd_enabled, g_sensor_count);

        g_interval.collect_interval_hours = (float)cycle_to_seconds(&g_collect_cycle) / 3600.0f;
        g_interval.upload_interval_hours  = (float)cycle_to_seconds(&g_report_cycle)  / 3600.0f;
        config_recalc_interval();
    }
    else
    {
        strncpy(g_window.pattern, APP_DEFAULT_WINDOW_PATTERN, 7);
        g_window.pattern[7] = '\0';
        g_window.end_result = APP_DEFAULT_WINDOW_END_RESULT;
        strncpy(g_window.start, APP_DEFAULT_WINDOW_START, 7);
        g_window.start[7] = '\0';
        strncpy(g_window.stop, APP_DEFAULT_WINDOW_STOP, 7);
        g_window.stop[7] = '\0';
        window_dottime_clear(g_window.dottime_mask);
        window_dottime_set(g_window.dottime_mask, 0);

        g_collect_cycle.once = APP_DEFAULT_COLLECT_ONCE;
        g_collect_cycle.type = APP_DEFAULT_COLLECT_TYPE;
        g_report_cycle.once  = APP_DEFAULT_REPORT_ONCE;
        g_report_cycle.type  = APP_DEFAULT_REPORT_TYPE;
        memset(g_abnormal_close, 0, sizeof(g_abnormal_close));
        strncpy(g_abnormal_close, APP_DEFAULT_ABNORMAL_CLOSE, sizeof(g_abnormal_close) - 1);
        g_valve_last_order = APP_DEFAULT_VALVE_LAST_ORDER;
        g_real_enabled = APP_DEFAULT_REAL_ENABLED;
        g_real_frequency = APP_DEFAULT_REAL_FREQUENCY;
        g_lcd_enabled = APP_DEFAULT_LCD_ENABLED;
        g_sensor_count = APP_DEFAULT_SENSOR_COUNT;
        /* 堵转方案: EEPROM 无有效 EXT 块时也套用编译期出厂阈值, 否则首启会"未标定"无法动作 */
        g_motor_cs_th = MOTOR_CS_TH_DEFAULT;
        g_valve_pos_valid = 0;
        g_valve_pos_state = 0;
        g_valve_alarm_closed = 0;
        g_valve_pre_alarm_order = 0;
        g_ch4_alarm_closed = 0;
        g_report_delay_min = 0.0f;
        voltage_low_power = 0;

        g_interval.collect_interval_hours = (float)cycle_to_seconds(&g_collect_cycle) / 3600.0f;
        g_interval.upload_interval_hours  = (float)cycle_to_seconds(&g_report_cycle)  / 3600.0f;
        config_recalc_interval();

        window_dottime_to_str(g_window.dottime_mask, dot_str, sizeof(dot_str));
        printf_debug("[CFG] No EXT, using defaults: win=%s er=%d start=%s stop=%s dot=%s collect=%d*%s report=%d*%s abnormal=%s valve=%d real=%d/%ds\r\n",
                     g_window.pattern, g_window.end_result,
                     g_window.start, g_window.stop, dot_str,
                     g_collect_cycle.once, cycle_type_to_str(g_collect_cycle.type),
                     g_report_cycle.once, cycle_type_to_str(g_report_cycle.type),
                     g_abnormal_close, g_valve_last_order,
                     g_real_enabled, g_real_frequency);
        config_save_ext();
    }

    /* ---------- 区块12: CH4 甲烷门限 ---------- */
    EE_READ(EE_ADDR_CH4_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_CH4)
    {
        uint8_t stored_en = 0xFF;
        uint8_t stored_warm = 0xFF;
        EE_READ(EE_ADDR_CH4_ENABLED, &stored_en, 1);
        EE_READ(EE_ADDR_CH4_WARMUP_SEC, &stored_warm, 1);
        g_ch4_threshold.enabled = (stored_en > 1) ? 0 : stored_en;
        g_ch4_threshold.warmup_sec = stored_warm;
        g_ch4_threshold.threshold = ee_read_float(EE_ADDR_CH4_THRESHOLD);

        /* 门限合法性: NaN/越界(0~999) → 回退默认 0.5 并回写 */
        if(!(g_ch4_threshold.threshold >= DEF_CH4_THRESHOLD_MIN &&
             g_ch4_threshold.threshold <= DEF_CH4_THRESHOLD_MAX))
        {
            g_ch4_threshold.threshold = DEF_CH4_THRESHOLD;
            config_save_ch4_threshold();
        }
        if(g_ch4_threshold.warmup_sec == 0 || g_ch4_threshold.warmup_sec == 0xFF)
            g_ch4_threshold.warmup_sec = DEF_CH4_WARMUP_SEC;

        printf_debug("[CFG] CH4: en=%d warmup=%ds threshold=%.2f\r\n",
                     g_ch4_threshold.enabled, g_ch4_threshold.warmup_sec,
                     g_ch4_threshold.threshold);
    }
    else
    {
        g_ch4_threshold.enabled = DEF_CH4_ENABLED;
        g_ch4_threshold.warmup_sec = DEF_CH4_WARMUP_SEC;
        g_ch4_threshold.threshold = DEF_CH4_THRESHOLD;
        printf_debug("[CFG] No CH4, using defaults\r\n");
        config_save_ch4_threshold();
    }

    /* ---------- 区块13: 激光甲烷定时采集 ---------- */
    EE_READ(EE_ADDR_LASER_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_LASER)
    {
        EE_READ(EE_ADDR_LASER_SWITCH, &g_laser_switch, 1);
        EE_READ(EE_ADDR_LASER_TIME, g_laser_time_mask, 3);
        if(g_laser_switch >= 2) g_laser_switch = 0;
        window_dottime_to_str(g_laser_time_mask, dot_str, sizeof(dot_str));
        printf_debug("[CFG] LASER: switch=%d time=%s\r\n", g_laser_switch, dot_str);
    }
    else
    {
        g_laser_switch = DEF_LASER_SWITCH;
        window_dottime_from_str(DEF_LASER_TIME_DEFAULT, g_laser_time_mask);
        printf_debug("[CFG] No LASER config, using defaults\r\n");
        config_save_laser();
    }

    /* ---------- 区块14: 出气端/激光异常关阀 ---------- */
    EE_READ(EE_ADDR_ABN2_MAGIC, &magic, 1);
    if(magic == EE_MAGIC_ABN23)
    {
        ee_read_string(EE_ADDR_ABNORMAL2_CLOSE, g_abnormal2_close, 16);
        EE_READ(EE_ADDR_ABNORMAL3_CLOSE, &g_abnormal3_close, 1);
        if(g_abnormal3_close > 1) g_abnormal3_close = 0;
        printf_debug("[CFG] ABN23: abn2=%s abn3=%d\r\n",
                     g_abnormal2_close, g_abnormal3_close);
    }
    else
    {
        strncpy(g_abnormal2_close, APP_DEFAULT_ABNORMAL_CLOSE, sizeof(g_abnormal2_close) - 1);
        g_abnormal2_close[sizeof(g_abnormal2_close) - 1] = '\0';
        g_abnormal3_close = 0;
        printf_debug("[CFG] No ABN23, using defaults\r\n");
        config_save_abn23();
    }
}
