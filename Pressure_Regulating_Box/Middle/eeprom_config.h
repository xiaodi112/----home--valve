#ifndef EEPROM_CONFIG_H
#define EEPROM_CONFIG_H

#include <stdint.h>
#include "app_config.h"
#include "threshold.h"

/*
 * ============================================================
 *  M24M01 EEPROM 配置存储 (128K 字节)
 * ============================================================
 *
 *  区域 1 — 间隔配置 (Magic 0xA5)
 *    0x000: Magic
 *    0x001-0x004: collect_interval_hours (float)
 *    0x005-0x008: upload_interval_hours  (float)
 *
 *  区域 2 — MQTT 配置 (Magic 0x5A)
 *    0x020: Magic
 *    0x021-0x060: ClientID (64B)
 *    0x061-0x0A0: Username (64B)
 *    0x0A1-0x0E0: Password (64B)
 *    0x0E1-0x0E2: Port (uint16)
 *    0x0E3-0x122: Server IP (64B)
 *    0x123-0x1A2: Topic上报 (128B)
 *
 *  区域 3 — 压力阈值 vpthreshold (Magic 0xC0)
 *    0x1B0: Magic
 *    0x1B1: enabled (1B)
 *    0x1B2-0x1B5: ppmax    (float)
 *    0x1B6-0x1B9: pmax     (float)
 *    0x1BA-0x1BD: ppmin    (float)
 *    0x1BE-0x1C1: pmin     (float)
 *    0x1C2-0x1C5: mutation (float)
 *    0x1C6-0x1C9: shock    (float)
 *    0x1CA-0x1CB: keepalive_sec (uint16, MQTT心跳)
 *
 *  区域 4 — 温度阈值 Tthreshold (Magic 0xC1)
 *    0x1D0: Magic
 *    0x1D1: enabled (1B)
 *    0x1D2-0x1D5: high  (float)
 *    0x1D6-0x1D9: low   (float)
 *    0x1DA-0x1DD: vhigh (float)
 *    0x1DE-0x1E1: vlow  (float)
 *
 *  区域 5 — 电压检测 spupower (Magic 0xC2)
 *    0x1F0: Magic
 *    0x1F1: enabled (1B)
 *    0x1F2-0x1F5: high    (float)
 *    0x1F6-0x1F9: low     (float)
 *    0x1FA-0x1FD: protect (float, VLP触发)
 *
 *  区域 6 — 扩展配置 (Magic 0xB7)
 *    0x200: Magic
 *    0x201-0x208: window_pattern (8B)
 *    0x209: window_end_result (1B)
 *    0x20A-0x211: window_start (8B)
 *    0x212-0x219: window_stop  (8B)
 *    0x21A: window_dottime (1B)
 *    0x21B-0x21C: collect_once (uint16)
 *    0x21D: collect_type (1B)
 *    0x21E-0x21F: report_once (uint16)
 *    0x220: report_type (1B)
 *    0x221-0x230: abnormal_close (16B string)
 *    0x231: valve_last_order (1B)
 *    0x240-0x27F: Server2 副IP (64B)
 *    0x280-0x281: Port2 (uint16)
 *    0x282: real_enabled (1B)
 *    0x283: real_frequency (1B)
 *    0x284: valve_alarm_closed (1B, 报警自动关阀标志, OTA/重启后仍能恢复)
 *    0x285: valve_pre_alarm_order (1B, 关阀前阀门状态, 用于恢复)
 *
 *  区域 7 — 阀门动作延时 valve_delay (Magic 0xC3)
 *    0x290: Magic
 *    0x291: enabled (1B)
 *    0x292-0x293: delay_ms (uint16)
 */

// ==================== EEPROM 地址定义 ====================
// 区域 1: 间隔配置
#define EE_ADDR_INTERVAL_MAGIC    0x000
#define EE_ADDR_COLLECT_HOURS     0x001
#define EE_ADDR_UPLOAD_HOURS      0x005

// 区域 2: MQTT 配置
#define EE_ADDR_MQTT_MAGIC        0x020
#define EE_ADDR_MQTT_CLIENT_ID    0x021
#define EE_ADDR_MQTT_USERNAME     0x061
#define EE_ADDR_MQTT_PASSWORD     0x0A1
#define EE_ADDR_MQTT_PORT         0x0E1
#define EE_ADDR_MQTT_SERVER       0x0E3
#define EE_ADDR_MQTT_TOPIC_UP     0x123

// 区域 3: 压力阈值 (vpthreshold)
#define EE_ADDR_VPTH_MAGIC        0x1B0
#define EE_ADDR_VPTH_ENABLED      0x1B1
#define EE_ADDR_VPTH_PPMAX        0x1B2
#define EE_ADDR_VPTH_PMAX         0x1B6
#define EE_ADDR_VPTH_PPMIN        0x1BA
#define EE_ADDR_VPTH_PMIN         0x1BE
#define EE_ADDR_VPTH_MUTATION     0x1C2
#define EE_ADDR_VPTH_SHOCK        0x1C6
#define EE_ADDR_KEEPALIVE_SEC     0x1CA   /* 2B uint16 MQTT心跳 (与压力区同 magic) */

// 区域 4: 温度阈值 (Tthreshold)
#define EE_ADDR_TTH_MAGIC         0x1D0
#define EE_ADDR_TTH_ENABLED       0x1D1
#define EE_ADDR_TTH_HIGH          0x1D2
#define EE_ADDR_TTH_LOW           0x1D6
#define EE_ADDR_TTH_VHIGH         0x1DA
#define EE_ADDR_TTH_VLOW          0x1DE

// 区域 5: 电压检测 (spupower)
#define EE_ADDR_SPUP_MAGIC        0x1F0
#define EE_ADDR_SPUP_ENABLED      0x1F1
#define EE_ADDR_SPUP_HIGH         0x1F2
#define EE_ADDR_SPUP_LOW          0x1F6
#define EE_ADDR_SPUP_PROTECT      0x1FA

// 区域 6: 扩展配置
#define EE_ADDR_EXT_MAGIC         0x200
#define EE_ADDR_WINDOW_PATTERN    0x201
#define EE_ADDR_WINDOW_END_RESULT 0x209
#define EE_ADDR_WINDOW_START_STR  0x20A
#define EE_ADDR_WINDOW_STOP_STR   0x212
#define EE_ADDR_WINDOW_DOTTIME    0x21A
#define EE_ADDR_COLLECT_ONCE      0x21B
#define EE_ADDR_COLLECT_TYPE      0x21D
#define EE_ADDR_REPORT_ONCE       0x21E
#define EE_ADDR_REPORT_TYPE       0x220
#define EE_ADDR_ABNORMAL_CLOSE    0x221
#define EE_ADDR_VALVE_LAST_ORDER  0x231
#define EE_ADDR_MQTT_SERVER2      0x240
#define EE_ADDR_MQTT_PORT2        0x280
#define EE_ADDR_REAL_ENABLED      0x282
#define EE_ADDR_REAL_FREQUENCY    0x283
#define EE_ADDR_VALVE_ALARM_CLOSED 0x284
#define EE_ADDR_VALVE_PRE_ALARM_ORDER 0x285
#define EE_ADDR_LCD_ENABLED       0x286
#define EE_ADDR_SENSOR_COUNT      0x287
#define EE_ADDR_VLP_FLAG          0x288

// 区域 7: 阀门动作延时 (valve_delay)
#define EE_ADDR_VDLY_MAGIC        0x290
#define EE_ADDR_VDLY_ENABLED      0x291
#define EE_ADDR_VDLY_DELAY_MS     0x292

// 区域 8: 出气端压力阈值 (vpthreshold2, Magic 0xC4)
//   0x2A0: Magic
//   0x2A1: enabled (1B)
//   0x2A2-0x2A5: ppmax   (float)
//   0x2A6-0x2A9: pmax    (float)
//   0x2AA-0x2AD: ppmin   (float)
//   0x2AE-0x2B1: pmin    (float)
//   0x2B2-0x2B5: mutation (float)
//   0x2B6-0x2B9: shock   (float)
#define EE_ADDR_VPTH2_MAGIC       0x2A0
#define EE_ADDR_VPTH2_ENABLED     0x2A1
#define EE_ADDR_VPTH2_PPMAX       0x2A2
#define EE_ADDR_VPTH2_PMAX        0x2A6
#define EE_ADDR_VPTH2_PPMIN       0x2AA
#define EE_ADDR_VPTH2_PMIN        0x2AE
#define EE_ADDR_VPTH2_MUTATION    0x2B2
#define EE_ADDR_VPTH2_SHOCK       0x2B6

// 区域 9: 出气端温度阈值 (Tthreshold2, Magic 0xC5)
//   0x2C0: Magic
//   0x2C1: enabled (1B)
//   0x2C2-0x2C5: high  (float)
//   0x2C6-0x2C9: low   (float)
//   0x2CA-0x2CD: vhigh (float)
//   0x2CE-0x2D1: vlow  (float)
#define EE_ADDR_TTH2_MAGIC        0x2C0
#define EE_ADDR_TTH2_ENABLED      0x2C1
#define EE_ADDR_TTH2_HIGH         0x2C2
#define EE_ADDR_TTH2_LOW          0x2C6
#define EE_ADDR_TTH2_VHIGH        0x2CA
#define EE_ADDR_TTH2_VLOW         0x2CE

// 区域 10: 用户端MQTT配置 (UserMqtt, Magic 0xC6)
//   0x300: Magic
//   0x301-0x340: server  (64B)
//   0x341-0x342: port    (uint16)
//   0x343-0x382: username (64B)
//   0x383-0x3C2: password (64B)
//   0x3C3-0x402: client_id (64B)
//   0x403-0x482: topic_up (128B)
#define EE_ADDR_UMQTT_MAGIC       0x300
#define EE_ADDR_UMQTT_SERVER      0x301
#define EE_ADDR_UMQTT_PORT        0x341
#define EE_ADDR_UMQTT_USERNAME    0x343
#define EE_ADDR_UMQTT_PASSWORD    0x383
#define EE_ADDR_UMQTT_CLIENT_ID   0x3C3
#define EE_ADDR_UMQTT_TOPIC_UP    0x403

// 区域 11: 定时开关阀 (ValveTiming, Magic 0xC7)
//   0x490: Magic
//   0x491: timing_switch (uint8)
//   0x492-0x495: open_time  (uint32, HHMMSS)
//   0x496-0x499: close_time (uint32, HHMMSS)
#define EE_ADDR_VTIMING_MAGIC     0x490
#define EE_ADDR_VTIMING_SWITCH    0x491
#define EE_ADDR_VTIMING_OPEN      0x492
#define EE_ADDR_VTIMING_CLOSE     0x496

// ==================== Magic 值 ====================
#define EE_MAGIC_INTERVAL         0xA5
#define EE_MAGIC_MQTT             0x5A
#define EE_MAGIC_EXT              0xB7
#define EE_MAGIC_VPTH             0xC0
#define EE_MAGIC_TTH              0xC1
#define EE_MAGIC_SPUP             0xC2
#define EE_MAGIC_VDLY             0xC3
#define EE_MAGIC_VPTH2            0xC4
#define EE_MAGIC_TTH2             0xC5
#define EE_MAGIC_UMQTT            0xC6
#define EE_MAGIC_VTIMING          0xC7

// ==================== 首次开机默认值 (清零策略, 用户后续填充) ====================
// 说明: 按需求默认 switch=0 (报警关), 阈值=0; 本人可修改这些宏提供用户预设
// 温度阈值 (Tthreshold) — 进气端(第一路)
#define DEF_TT_SWITCH        0
#define DEF_TT_HIGH          0.0f
#define DEF_TT_LOW           0.0f
#define DEF_TT_VHIGH         0.0f
#define DEF_TT_VLOW          0.0f

// 温度阈值 (Tthreshold2) — 出气端(第二路)
#define DEF_TT2_SWITCH       0
#define DEF_TT2_HIGH         0.0f
#define DEF_TT2_LOW          0.0f
#define DEF_TT2_VHIGH        0.0f
#define DEF_TT2_VLOW         0.0f

// 阀门动作延时 (valve_delay)
#define DEF_VD_SWITCH        0
#define DEF_VD_TIME_MS       0
// 安全上限: 阀门到位后保持 PWM 最大允许时间 (ms)
// 超出此值会让电机长时间堵转, 可能损坏电机或耗尽电池
#define VDLY_DELAY_MS_MAX    5000

// 电压检测 (spupower)
#define DEF_SP_SWITCH        1
#define DEF_SP_HIGH          7.2f
#define DEF_SP_LOW           3.0f
#define DEF_SP_PROTECT       3.2f

// 压力阈值 (vpthreshold) — 进气端(第一路)
#define DEF_VP_SWITCH        0
#define DEF_VP_PPMAX         10000.0f
#define DEF_VP_PMAX          5000.0f
#define DEF_VP_PPMIN         -200.0f
#define DEF_VP_PMIN          -100.0f
#define DEF_VP_MUTATION      1000.0f
#define DEF_VP_SHOCK         100.0f

// 压力阈值 (vpthreshold2) — 出气端(第二路)
#define DEF_VP2_SWITCH       0
#define DEF_VP2_PPMAX        10000.0f
#define DEF_VP2_PMAX         5000.0f
#define DEF_VP2_PPMIN        -200.0f
#define DEF_VP2_PMIN         -100.0f
#define DEF_VP2_MUTATION     1000.0f
#define DEF_VP2_SHOCK        100.0f

// 字段最大长度
#define MQTT_FIELD_MAX            64
#define MQTT_TOPIC_MAX            128

// ==================== 运行时配置结构体 ====================

typedef struct {
    float collect_interval_hours;   // 采集周期（小时）
    float upload_interval_hours;    // 上报周期（小时）
    uint16_t collect_interval_min;  // 自动计算: 分钟
    uint16_t upload_interval_min;   // 自动计算: 分钟
    uint16_t upload_every_n;        // 自动计算: 每N次唤醒上报
} AppInterval_t;

typedef struct {
    char client_id[MQTT_FIELD_MAX];
    char username[MQTT_FIELD_MAX];
    char password[MQTT_FIELD_MAX];
    char server[MQTT_FIELD_MAX];        // 主IP
    char server2[MQTT_FIELD_MAX];       // 副IP (为空则不启用)
    char topic_up[MQTT_TOPIC_MAX];
    uint16_t port;                      // 主端口
    uint16_t port2;                     // 副端口
} MqttConfig_t;

// 温度阈值 (Tthreshold_set/get)
typedef struct {
    uint8_t enabled;            // 0=关闭温度报警, 1=开启
    float   high;               // 温度上限
    float   low;                // 温度下限
    float   vhigh;              // 温度极高限
    float   vlow;               // 温度极低限
} TempThresholdConfig_t;

// 阀门动作延时 (delay_set/get)
typedef struct {
    uint8_t  enabled;           // 0=不延时, 1=到位后保持原动作 delay_ms
    uint16_t delay_ms;          // 保持时间 (ms)
} ValveDelayConfig_t;

// 定时开关阀 (closeoropen_set/get)
typedef struct {
    uint8_t  enabled;           // timing_switch: 0=关, 1=开
    uint32_t open_time;         // 开阀时间 HHMMSS (如 123400)
    uint32_t close_time;        // 关阀时间 HHMMSS (如 134700)
} ValveTimingConfig_t;

// 电压检测 (spupower_set/get)
typedef struct {
    uint8_t enabled;            // 0=关闭电压检测(包括 VLP), 1=开启
    float   high;               // 电压上限
    float   low;                // 电压下限
    float   protect;            // 电压保护阈值 (触发 VLP 省电)
} VoltagePowerConfig_t;

// 压力阈值 (vpthreshold_set1/get1)
typedef struct {
    uint8_t enabled;            // 0=关闭压力报警, 1=开启
    float   ppmax;              // 压力极高限
    float   pmax;               // 压力上限
    float   ppmin;              // 压力极低限
    float   pmin;               // 压力下限
    float   mutation;           // 压力突变阈值 (|Δp|, 不受 shock 影响)
    float   shock;              // 静态阈值滞回带 (仅作用于 pmax/pmin/ppmax/ppmin)
} PressureThresholdConfig_t;

// 用户端MQTT配置 (第二连接, id=1)
typedef struct {
    char server[MQTT_FIELD_MAX];        // 用户端服务器IP/域名
    uint16_t port;                      // 用户端端口
    char username[MQTT_FIELD_MAX];      // 用户端用户名
    char password[MQTT_FIELD_MAX];      // 用户端密码
    char client_id[MQTT_FIELD_MAX];     // 用户端client_id (同时作为deviceKey)
    char topic_up[MQTT_TOPIC_MAX];      // 用户端上报topic
} UserMqttConfig_t;

// 全局运行时配置（其他文件通过extern引用）
extern AppInterval_t              g_interval;
extern MqttConfig_t               g_mqtt;
extern UserMqttConfig_t           g_user_mqtt;            // 用户端MQTT配置
extern WindowConfig_t             g_window;
extern CycleConfig_t              g_collect_cycle;
extern CycleConfig_t              g_report_cycle;
extern char                       g_abnormal_close[16];
extern uint8_t                    g_valve_last_order;
extern uint8_t                    g_valve_alarm_closed;    // 报警自动关阀标志 (EEPROM持久化, OTA重启后可恢复)
extern uint8_t                    g_valve_pre_alarm_order; // 关阀前阀门状态 (EEPROM持久化)
extern uint8_t                    g_real_enabled;         // 实时采集开关 (0=关, 1=开, EEPROM持久化)
extern uint8_t                    g_real_frequency;       // 实时采集频率 (1-10秒)
extern uint8_t                    g_lcd_enabled;          // LCD显示开关 (0=关, 1=开, EEPROM持久化)
extern uint8_t                    g_lcd_active;           // LCD当前是否在显示 (运行态, 延时关闭用)
extern uint8_t                    g_sensor_count;         // 传感器数量 (1=单路, 2=双路, EEPROM持久化)
extern uint16_t                   g_keepalive_sec;        // MQTT心跳(秒)
extern TempThresholdConfig_t      g_tthreshold;           // 进气端温度阈值
extern TempThresholdConfig_t      g_tthreshold2;          // 出气端温度阈值
extern ValveDelayConfig_t         g_vdelay;               // 阀门动作延时
extern VoltagePowerConfig_t       g_spupower;             // 电压检测
extern PressureThresholdConfig_t  g_pthreshold;           // 进气端压力阈值
extern PressureThresholdConfig_t  g_pthreshold2;          // 出气端压力阈值
extern ValveTimingConfig_t        g_valve_timing;         // 定时开关阀配置

// ==================== 接口函数 ====================

// 从EEPROM加载配置（上电调用）
void config_load_all(void);

// 保存间隔配置到EEPROM
void config_save_interval(void);

// 保存MQTT配置到EEPROM
void config_save_mqtt(void);

// 从client_id推导topic_up
void derive_topic_from_client_id(void);

// 保存扩展配置到EEPROM (window/cycle/abnormal/valve_last_order)
void config_save_ext(void);

// 保存压力阈值区 (含 keepalive)
void config_save_vpthreshold(void);

// 保存温度阈值区
void config_save_tthreshold(void);

// 保存电压检测区
void config_save_spupower(void);

// 保存阀门延时区
void config_save_vdelay(void);

// 保存出气端压力阈值区
void config_save_vpthreshold2(void);

// 保存出气端温度阈值区
void config_save_tthreshold2(void);

// 保存用户端MQTT配置
void config_save_user_mqtt(void);

// 保存定时开关阀配置
void config_save_valve_timing(void);

// 获取心跳值(防无效)
uint16_t config_get_keepalive(void);

// 周期类型字符串 <-> 枚举 转换
CycleType_t cycle_type_from_str(const char *str);
const char* cycle_type_to_str(CycleType_t type);

// 将周期配置转换为秒数
uint32_t cycle_to_seconds(const CycleConfig_t *cfg);

// 异常关阀字符串 <-> 枚举
AbnormalClose_t abnormal_from_str(const char *str);
const char* abnormal_to_str(AbnormalClose_t ac);

// 检查阈值结果是否匹配异常关阀策略 (1=应自动关阀, 0=不关)
int abnormal_should_close_valve(AbnormalClose_t policy, ThresholdResult_t result);

// 上电配置检查: MQTT未配置时阻塞等待30秒, 其余用默认值
void config_enter_setup_mode(uint8_t force_reconfig);

// 打印当前配置到debug串口
void config_print_current(void);

// 重新计算间隔的分钟数和唤醒次数
void config_recalc_interval(void);

// 统一配置命令解析(上电等待+运行时复用)
// 返回: 1=有效配置, 0=无效, -1=reset(不会返回,直接重启)
int config_parse_and_apply(const char *cmd);

// 非阻塞检查debug串口(主循环每轮调用, ISR置位后立即响应)
void config_check_cmd(void);

// 判断当前RTC时间是否在活动窗口内
// 返回: 1=活动(应上报), 0=非活动(仅采集)
int config_is_active_window(void);

// 低电压省电模式标志 (定义于 main.c)
extern uint8_t voltage_low_power;

// 智能决策当前采集周期是否需要采样电池电压
// 仅在首次、尾次或低电压保护模式下开启，其余中间周期关闭以省电
uint8_t config_need_adc_sampling(void);

#endif /* EEPROM_CONFIG_H */
