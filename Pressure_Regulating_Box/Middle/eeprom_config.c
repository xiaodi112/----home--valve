/**
 * @file eeprom_config.c
 * @brief 配置命令行解析 (Debug串口 CLI + 上电配置等待)
 * @details 持久化见 config_store.c, 纯策略见 config_policy.c
 */
#include "eeprom_config.h"
#include "config_store.h"
#include "config_policy.h"
#include "app_config.h"
#include "calib_mode.h"
#include "zero_cal.h"
#include "valve_timing.h"
#include "wakeup.h"
#include "usart.h"
#include "delay.h"
#include "threshold.h"
#include "tim.h"
#include "PT_304_35.h"
#include "main.h"
#include "eeprom_history.h"
#include "leak_store.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
// ==================== 模块共享 配置命令行缓冲 ====================
// config_enter_setup_mode (上电配置阶段) 与 config_check_cmd (运行期主循环) 互斥调用
// 不并发 不嵌套 可共享同一块 512B 缓冲 节省 512B RAM
static char s_cfg_line_buf[512];
// 从字符串中提取key:value对的value
// 输入: src = "clientId:abc,username:def", key = "clientId"
// 输出: out = "abc", 返回1成功
// 匹配必须带单词边界: 前一字符必须是 { , : 或字符串起始,
//     防止 "pmax" 误匹配 "ppmax:5000" 内部子串//     (同样规避 high/vhigh, low/vlow, pmin/ppmin 等前缀冲突)
static int extract_field(const char *src, const char *key, char *out, uint16_t out_size)
{
    char search_key[32];
    snprintf(search_key, sizeof(search_key), "%s:", key);
    size_t key_len = strlen(search_key);

    const char *p = src;
    while((p = strstr(p, search_key)) != NULL)
    {
        // 单词边界检查: p前面的字符必须是分隔符或字符串起始
if(p == src || *(p - 1) == '{' || *(p - 1) == ',' || *(p - 1) == ':' || *(p - 1) == ' ')
        {
            break;  // 命中真正的关键词
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

// ==================== 解析用户端MQTT命令 ====================
// 格式: {umqtt_server:x.x.x.x,umqtt_port:13000,umqtt_user:xxx,umqtt_pass:xxx,umqtt_client:xxx,umqtt_topic:/155/xxx/property/post}
// 所有字段均可选, 仅更新提供的字段; 必须至少包含 umqtt_server / umqtt_client / umqtt_topic 来触发匹配
static int parse_user_mqtt_line(const char *cmd)
{
    if(!strstr(cmd, "umqtt_")) return 0;

    char val_buf[MQTT_TOPIC_MAX];
    int changed = 0;

    if(extract_field(cmd, "umqtt_server", val_buf, MQTT_FIELD_MAX))
    { strncpy(g_user_mqtt.server, val_buf, MQTT_FIELD_MAX - 1); changed = 1; }
    if(extract_field(cmd, "umqtt_port", val_buf, sizeof(val_buf)))
    { g_user_mqtt.port = (uint16_t)atoi(val_buf); changed = 1; }
    if(extract_field(cmd, "umqtt_user", val_buf, MQTT_FIELD_MAX))
    { strncpy(g_user_mqtt.username, val_buf, MQTT_FIELD_MAX - 1); changed = 1; }
    if(extract_field(cmd, "umqtt_pass", val_buf, MQTT_FIELD_MAX))
    { strncpy(g_user_mqtt.password, val_buf, MQTT_FIELD_MAX - 1); changed = 1; }
    if(extract_field(cmd, "umqtt_client", val_buf, MQTT_FIELD_MAX))
    { strncpy(g_user_mqtt.client_id, val_buf, MQTT_FIELD_MAX - 1); changed = 1; }
    if(extract_field(cmd, "umqtt_topic", val_buf, MQTT_TOPIC_MAX))
    { strncpy(g_user_mqtt.topic_up, val_buf, MQTT_TOPIC_MAX - 1); changed = 1; }

    // 若未显式指定topic但有client_id且topic为空, 自动回退拼接
if(g_user_mqtt.topic_up[0] == '\0' && g_user_mqtt.client_id[0])
        snprintf(g_user_mqtt.topic_up, MQTT_TOPIC_MAX, "/155/%s/property/post", g_user_mqtt.client_id);

    if(changed) config_save_user_mqtt();
    printf_debug("[CFG] UMQTT: %s@%s:%d topic=%s\r\n",
                 g_user_mqtt.client_id, g_user_mqtt.server, g_user_mqtt.port, g_user_mqtt.topic_up);
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

// ==================== 解析压力阈值(vpthreshold) ====================
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

// ==================== 解析温度阈值(Tthreshold) ====================
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

// ==================== 解析电压检测(spupower) ====================
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

// ==================== 解析阀门动作延时(delay) ====================
// 格式: {id:delay,value:switch:1,time:500}
// time 范围 0~VDLY_DELAY_MS_MAX (默认 5000ms), 超出后 config_save_vdelay 内 clamp
static int parse_vdelay_line(const char *cmd)
{
    /* 严格边界匹配: 避免吞掉 {id:delay_report,...} (delay_report 含子串 id:delay) */
    if(!strstr(cmd, "id:delay,") && !strstr(cmd, "id:delay}")) return 0;

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
        // 清除EEPROM所有Magic, 重启后自动进入MQTT等待
EE_WRITE_BYTE(EE_ADDR_INTERVAL_MAGIC, 0xFF);
        EE_WRITE_BYTE(EE_ADDR_MQTT_MAGIC,     0xFF);
        EE_WRITE_BYTE(EE_ADDR_EXT_MAGIC,      0xFF);
        EE_WRITE_BYTE(EE_ADDR_VPTH_MAGIC,     0xFF);
        EE_WRITE_BYTE(EE_ADDR_TTH_MAGIC,      0xFF);
        EE_WRITE_BYTE(EE_ADDR_SPUP_MAGIC,     0xFF);
        EE_WRITE_BYTE(EE_ADDR_VDLY_MAGIC,     0xFF);
        EE_WRITE_BYTE(EE_ADDR_VPTH2_MAGIC,    0xFF);
        EE_WRITE_BYTE(EE_ADDR_TTH2_MAGIC,     0xFF);
        EE_WRITE_BYTE(EE_ADDR_UMQTT_MAGIC,    0xFF);
        EE_WRITE_BYTE(EE_ADDR_VTIMING_MAGIC,  0xFF);
        EE_WRITE_BYTE(EE_ADDR_CH4_MAGIC,      0xFF);
        EE_WRITE_BYTE(EE_ADDR_LASER_MAGIC,    0xFF);
        EE_WRITE_BYTE(EE_ADDR_ABN2_MAGIC,     0xFF);
        EE_WRITE_BYTE(EE_ADDR_UPL_MAGIC,      0xFF);   /* 上报防重锁: 恢复出厂后重新起算 */
        calib_erase_state();    /* reset 同时清除已安装锁定/零点, 恢复未安装校零状态 */
        zero_cal_erase();       /* reset 同时清除新传感器的大气基准(零点), 重启后重新采集 */
        NVIC_SystemReset();
        return -1;  // 不会到达
        }
    if(parse_mqtt_line(cmd))         return 1;
    if(parse_user_mqtt_line(cmd))    return 1;
    if(parse_collect_line(cmd))      return 1;
    if(parse_keepalive_line(cmd))    return 1;
    if(parse_vpthreshold_line(cmd))  return 1;
    if(parse_tthreshold_line(cmd))   return 1;
    if(parse_spupower_line(cmd))     return 1;

    // ---- 周期上报后延(分钟, 支持小数) ----
    // 格式: {id:delay_report,value:1.5}  (1.5 = 后延1分30秒; 范围 0~60)
    // 注意: 必须放在 parse_vdelay_line 之前, 且 vdelay 已用严格边界匹配
    if(strstr(cmd, "id:delay_report"))
    {
        char val_buf[16];
        if(extract_field(cmd, "value", val_buf, sizeof(val_buf)))
        {
            float d = (float)atof(val_buf);
            if(!(d >= 0.0f && d <= 60.0f))   /* NaN/越界回退 0 */
                d = 0.0f;
            g_report_delay_min = d;
            config_save_ext();
            upload_schedule_reset();   /* 后延变更: 重置延迟上报锁存 */
            printf_debug("[CFG] delay_report -> %.1f min\r\n", g_report_delay_min);
        }
        else
            printf_debug("[CFG] delay_report: %.1f min\r\n", g_report_delay_min);
        return 1;
    }

    // ---- 阀门限位到位等待超时(ms) ----
    // 格式: {id:valve_timeout,value:40000}  (范围 1000~120000, 默认40000)
    if(strstr(cmd, "id:valve_timeout"))
    {
        char val_buf[16];
        if(extract_field(cmd, "value", val_buf, sizeof(val_buf)))
        {
            uint32_t v = (uint32_t)atol(val_buf);
            if(v < VALVE_TIMEOUT_MS_MIN) v = VALVE_TIMEOUT_MS_MIN;
            if(v > VALVE_TIMEOUT_MS_MAX) v = VALVE_TIMEOUT_MS_MAX;
            g_valve_timeout_ms = v;
            config_save_vdelay();
            printf_debug("[CFG] valve_timeout -> %u ms\r\n", g_valve_timeout_ms);
        }
        else
            printf_debug("[CFG] valve_timeout: %u ms\r\n", g_valve_timeout_ms);
        return 1;
    }

    if(parse_vdelay_line(cmd))       return 1;

    // ---- 异常关阀策略 ----
    // 格式: {id:abnormal,value:temph}
if(strstr(cmd, "id:abnormal"))
    {
        char val_buf[96];
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
        char val_buf[96];
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
            upload_schedule_reset();   /* 上报周期变更: 重置延迟上报锁存 */
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
        char val_buf[96];
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
            upload_schedule_reset();   /* 周期变更: 重置网格防重锁, 严格等到下一目标 */
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
        char val_buf[96];
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
        { window_dottime_from_str(val_buf, g_window.dottime_mask); got = 1; }
        if(got) config_save_ext();
    char dot_str[64]; window_dottime_to_str(g_window.dottime_mask, dot_str, sizeof(dot_str));
        printf_debug("[CFG] window: %s er=%d s=%s e=%s dot=%s\r\n",
                     g_window.pattern, g_window.end_result,
                     g_window.start, g_window.stop, dot_str);
        return 1;
    }

    // ---- 简单查询命令(不需要main.c上下文) ----
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
    if(strcmp(cmd, "umqtt_get") == 0)
    {
        printf_debug("  UMQTT server=%s port=%d user=%s client=%s topic=%s\r\n",
                     g_user_mqtt.server[0] ? g_user_mqtt.server : "(none)",
                     g_user_mqtt.port,
                     g_user_mqtt.username[0] ? g_user_mqtt.username : "(none)",
                     g_user_mqtt.client_id[0] ? g_user_mqtt.client_id : "(none)",
                     g_user_mqtt.topic_up[0] ? g_user_mqtt.topic_up : "(none)");
        return 1;
    }
    if(strncmp(cmd, "lcd_switch=", 11) == 0)
    {
        g_lcd_enabled = (uint8_t)atoi(cmd + 11);
        if(g_lcd_enabled > 1) g_lcd_enabled = 1;
        printf_debug("  lcd_enabled=%d\r\n", g_lcd_enabled);
        return 1;
    }
    // ---- 实时采集开关+频率 ----
    // 格式: real_set=1,5  (开启,5秒)  real_set=0  (关闭)  real_set=1  (开启,频率不变)
    if(strncmp(cmd, "real_set=", 9) == 0)
    {
        const char *p = cmd + 9;
        g_real_enabled = (uint8_t)atoi(p);
        if(g_real_enabled > 1) g_real_enabled = 0;
        const char *comma = strchr(p, ',');
        if(comma)
        {
            uint8_t freq = (uint8_t)atoi(comma + 1);
            if(freq >= 1 && freq <= 10)
                g_real_frequency = freq;
        }
        config_save_ext();
        printf_debug("  real_enabled=%d real_frequency=%ds\r\n", g_real_enabled, g_real_frequency);
        return 1;
    }
	// ---- 副IP修改(只保存,不重连) ----
	// 格式: scend_ip:IP.port  (最后一个点后为端口)
	if(strncmp(cmd, "scend_ip:", 9) == 0)
	{
		char ip_buf[MQTT_FIELD_MAX];
		strncpy(ip_buf, cmd + 9, sizeof(ip_buf) - 1);
		ip_buf[sizeof(ip_buf) - 1] = '\0';
		char *last_dot = strrchr(ip_buf, '.');
		if(last_dot)
		{
			*last_dot = '\0';
			g_mqtt.port2 = (uint16_t)atoi(last_dot + 1);
			strncpy(g_mqtt.server2, ip_buf, MQTT_FIELD_MAX - 1);
			config_save_mqtt();
			printf_debug("  Secondary IP set: %s:%d\r\n", g_mqtt.server2, g_mqtt.port2);
		}
		else
			printf_debug("  Invalid format, use: scend_ip:IP.port\r\n");
		return 1;
	}

    // 返回0: 未匹配, 由调用方(main.c)继续处理需要硬件上下文的命令
return 0;
}

// ==================== 上电配置检查 ====================
// 仅当MQTT未配置时阻塞等待(30s), 期间接受任意配置命令(任意顺序)
// MQTT以外的参数都有默认值, 超时后用默认值启动
void config_enter_setup_mode(uint8_t force_reconfig)
{
    // 共享 s_cfg_line_buf (模块级; 与 config_check_cmd 互斥调用
uint8_t mqtt_valid = 0;

    (void)force_reconfig;  // 不再需要, reset已改为清magic+重启

    // 检查MQTT是否已配置
	{
    uint8_t m = 0;
        EE_READ(EE_ADDR_MQTT_MAGIC, &m, 1);
        mqtt_valid = (m == EE_MAGIC_MQTT) ? 1 : 0;
    }

    if(mqtt_valid)
    {
        printf_debug("[CFG] MQTT valid, ready\r\n");
        return;
    }

    // MQTT未配置, 进入配置窗口
    printf_debug("\r\n[CFG] === MQTT not configured, waiting (30s) ===\r\n");
    printf_debug("  MQTT: {clientId:xxx,username:xxx,passwd:xxx,IP:x.x.x.x,port:1883}\r\n");
    printf_debug("  UMQTT: {umqtt_server:x.x.x.x,umqtt_port:13000,umqtt_user:xx,umqtt_pass:xx,umqtt_client:xx,umqtt_topic:/xx/xx}\r\n");
    printf_debug("  Also accepts: collect_cycle / report_cycle / keepalive / vpthreshold / Tthreshold / spupower / delay / window_pattern\r\n");

    uint32_t start = trea_millis();
    uint32_t last_hint = 0;

    while((trea_millis() - start) < 30000)
    {
        // 每5秒提示
    int elapsed = (trea_millis() - start) / 1000;
        if(elapsed - last_hint >= 15)
        {
            last_hint = elapsed;
            printf_debug("[CFG] Waiting for MQTT... (%ds/30s)\r\n", elapsed);
        }

        uint16_t len = debug_rx_read_line(s_cfg_line_buf, sizeof(s_cfg_line_buf), 1000);
        if(len == 0) continue;

        printf_debug("[CFG] Recv: %s\r\n", s_cfg_line_buf);
        config_parse_and_apply(s_cfg_line_buf);

        // 检查MQTT是否已配置        {
	uint8_t m = 0;
            EE_READ(EE_ADDR_MQTT_MAGIC, &m, 1);
            if(m == EE_MAGIC_MQTT)
            {
                printf_debug("[CFG] MQTT configured! Waiting 5s for optional params...\r\n");
                // 短窗口接收可选参数
    uint32_t opt_start = trea_millis();
                while((trea_millis() - opt_start) < 10000)
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
    // 30秒超时, MQTT仍未配置
    printf_debug("[CFG] TIMEOUT: MQTT not configured, entering deep sleep...\r\n");
    trea_delay_ms(50);
    {
        PmSleepCfg_t sc = { 0, 0, 0 };   /* 首次配置阶段: LCD/蓝牙均未开启 */
        enter_deepsleep_mode(&sc);
    }
}


// ==================== 存储布局版本 / 迁移 ====================
/* 上电检查 EEPROM 分区布局版本:
 *   读到当前版本 → 返回 0 (正常启动, 不动数据)
 *   读到其它值   → 判定为旧布局(或首次烧写设备), 主动格式化历史区与测漏区后写入当前版本,
 *                  返回 1。这样"历史区/测漏区搬家"不会把旧分区的残留当成有效数据。
 * 调用时机: config_load_all() + ee_hist_init() + leak_store_init() 之后,
 *           任何历史/测漏读写之前 (main() 内)。 */
uint8_t eeprom_layout_check_and_migrate(void)
{
    uint8_t magic = 0xFF;
    uint8_t ver = 0xFF;

    EE_READ(EE_ADDR_LAYOUT_MAGIC, &magic, 1);
    EE_READ(EE_ADDR_LAYOUT_VER, &ver, 1);

    if(magic == EE_MAGIC_LAYOUT && ver == EE_LAYOUT_VER_CURRENT)
    {
        printf_debug("[CFG] storage layout ver=%d OK\r\n", (int)ver);
        return 0;
    }

    printf_debug("[CFG] storage layout changed (magic=0x%02X ver=%d -> %d), migrate...\r\n",
                 (unsigned)magic, (int)ver, EE_LAYOUT_VER_CURRENT);

    ee_hist_clear();        /* 历史区: 写管理头(count=0), 旧记录作废 */
    leak_store_clear();     /* 测漏区: 逻辑清零 */

    EE_WRITE_BYTE(EE_ADDR_LAYOUT_MAGIC, EE_MAGIC_LAYOUT);
    EE_WRITE_BYTE(EE_ADDR_LAYOUT_VER, (uint8_t)EE_LAYOUT_VER_CURRENT);

    printf_debug("[CFG] storage layout migrated to ver=%d\r\n", EE_LAYOUT_VER_CURRENT);
    return 1;
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
    printf_debug("  Delay_report: %.1f min\r\n", g_report_delay_min);
    char dot_str[64]; window_dottime_to_str(g_window.dottime_mask, dot_str, sizeof(dot_str));
    printf_debug("  Window: pattern=%s start=%s stop=%s er=%d dot=%s\r\n",
                 g_window.pattern, g_window.start, g_window.stop,
                 g_window.end_result, dot_str);
    printf_debug("  Abnormal: %s\r\n", g_abnormal_close);
    printf_debug("  Abnormal2(outlet): %s   \r\nAbnormal3(laser): %d\r\n",
                 g_abnormal2_close, g_abnormal3_close);
    printf_debug("  Valve: last_order=%d\r\n", g_valve_last_order);
    printf_debug("  Sensor: count=%d type=PT304D-4B S1(out)=%.1fkPa S2(in)=%.1fkPa temp=%.0f~%.0fC\r\n",
                 g_sensor_count,
                 PT304D_S1_FS / 1000.0f, PT304D_S2_FS / 1000.0f,
                 PT304D_TEMP_MIN, PT304D_TEMP_MAX);
    printf_debug("  VPTH en=%d ppmax=%.1f pmax=%.1f ppmin=%.1f pmin=%.1f mut=%.1f shock=%.1f\r\n",
                 g_pthreshold.enabled, g_pthreshold.ppmax, g_pthreshold.pmax,
                 g_pthreshold.ppmin, g_pthreshold.pmin,
                 g_pthreshold.mutation, g_pthreshold.shock);
    printf_debug("  TTH  en=%d high=%.1f low=%.1f vhigh=%.1f vlow=%.1f\r\n",
                 g_tthreshold.enabled, g_tthreshold.high, g_tthreshold.low,
                 g_tthreshold.vhigh, g_tthreshold.vlow);
    printf_debug("  VPTH2 en=%d ppmax=%.1f pmax=%.1f ppmin=%.1f pmin=%.1f mut=%.1f shock=%.1f\r\n",
                 g_pthreshold2.enabled, g_pthreshold2.ppmax, g_pthreshold2.pmax,
                 g_pthreshold2.ppmin, g_pthreshold2.pmin,
                 g_pthreshold2.mutation, g_pthreshold2.shock);
    printf_debug("  TTH2  en=%d high=%.1f low=%.1f vhigh=%.1f vlow=%.1f\r\n",
                 g_tthreshold2.enabled, g_tthreshold2.high, g_tthreshold2.low,
                 g_tthreshold2.vhigh, g_tthreshold2.vlow);
    printf_debug("  SPUP en=%d high=%.2f low=%.2f protect=%.2f\r\n",
                 g_spupower.enabled, g_spupower.high, g_spupower.low, g_spupower.protect);
    printf_debug("  VDLY en=%d delay_ms=%d\r\n", g_vdelay.enabled, g_vdelay.delay_ms);
    printf_debug("  Valve_timeout: %u ms\r\n", g_valve_timeout_ms);
    printf_debug("  Realtime: enabled=%d frequency=%ds\r\n",
                 g_real_enabled, g_real_frequency);
    printf_debug("  CH4  en=%d warmup=%ds threshold=%.2f\r\n",
                 g_ch4_threshold.enabled, g_ch4_threshold.warmup_sec,
                 g_ch4_threshold.threshold);
    {
        char laser_str[64];
        window_dottime_to_str(g_laser_time_mask, laser_str, sizeof(laser_str));
        printf_debug("  LASER switch=%d time=%s\r\n", g_laser_switch, laser_str);
    }
    printf_debug("  UMQTT: %s@%s:%d topic=%s\r\n",
                 g_user_mqtt.client_id[0] ? g_user_mqtt.client_id : "(none)",
                 g_user_mqtt.server[0] ? g_user_mqtt.server : "(none)",
                 g_user_mqtt.port,
                 g_user_mqtt.topic_up[0] ? g_user_mqtt.topic_up : "(none)");
    printf_debug("----------------------\r\n");
}

// ==================== 非阻塞检查debug串口(主循环每轮调用) ====================
void config_check_cmd(void)
{
    // 共享 s_cfg_line_buf (模块级; 与 config_enter_setup_mode 互斥调用
uint16_t len;

    while((len = debug_rx_read_line(s_cfg_line_buf, sizeof(s_cfg_line_buf), 0)) > 0)
    {
        printf_debug("[CFG] Recv: %s\r\n", s_cfg_line_buf);
        config_parse_and_apply(s_cfg_line_buf);
    }
}
