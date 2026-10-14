#ifndef EEPROM_CONFIG_H
#define EEPROM_CONFIG_H

#include <stdint.h>
#include "app_config.h"
#include "threshold.h"

/*
 * ============================================================
 *  M24M01 EEPROM 存储布局 (128K 容量)
 * ============================================================
 *
 *  区块 1: 采集上报间隔 (Magic 0xA5)
 *    0x000: Magic
 *    0x001-0x004: collect_interval_hours (float)
 *    0x005-0x008: upload_interval_hours  (float)
 *
 *  区块 2: MQTT 配置 (Magic 0x5A)
 *    0x020: Magic
 *    0x021-0x060: ClientID (64B)
 *    0x061-0x0A0: Username (64B)
 *    0x0A1-0x0E0: Password (64B)
 *    0x0E1-0x0E2: Port (uint16)
 *    0x0E3-0x122: Server IP (64B)
 *    0x123-0x1A2: Topic上报 (128B)
 *
 *  区块 3: 压力阈值 vpthreshold (Magic 0xC0)
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
 *  区块 4: 温度阈值 Tthreshold (Magic 0xC1)
 *    0x1D0: Magic
 *    0x1D1: enabled (1B)
 *    0x1D2-0x1D5: high  (float)
 *    0x1D6-0x1D9: low   (float)
 *    0x1DA-0x1DD: vhigh (float)
 *    0x1DE-0x1E1: vlow  (float)
 *
 *  区块 5: 电压检测 spupower (Magic 0xC2)
 *    0x1F0: Magic
 *    0x1F1: enabled (1B)
 *    0x1F2-0x1F5: high    (float)
 *    0x1F6-0x1F9: low     (float)
 *    0x1FA-0x1FD: protect (float, VLP保护)
 *
 *  区块 6: 扩展配置 (Magic 0xB7)
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
 *    0x284: valve_alarm_closed (1B, 报警自动关阀标志, OTA/重启后自动恢复)
 *    0x285: valve_pre_alarm_order (1B, 关阀前阀门状态, 恢复用)
 *    0x286: lcd_enabled (1B)
 *    0x287: sensor_count (1B)
 *    0x288: vlp_flag (1B)
 *    0x289: ch4_alarm_closed (1B, 激光甲烷报警关阀标志, 重启后保持, 平台 valve_open 清除)
 *    0x28A-0x28D: report_delay_min (float, 周期上报后延分钟, 0~60, 支持小数如1.5)
 *    0x232-0x233: motor_cs_th (uint16, 堵转判定电流阈值 ADC 原始值; 0=未标定)
 *    0x234: valve_pos_valid (1B, 阀门位置可信标志: 动作前置0, 到位后置1)
 *
 *  区块 7: 阀门到位延时 valve_delay (Magic 0xC3)
 *    0x290: Magic
 *    0x291: enabled (1B)
 *    0x292-0x293: delay_ms (uint16)
 *    0x294-0x297: valve_timeout_ms (uint32, 阀门限位到位等待超时, 默认40000)
 */

// ==================== EEPROM 地址定义 ====================
// 区块 1: 采集上报间隔
#define EE_ADDR_INTERVAL_MAGIC    0x000
#define EE_ADDR_COLLECT_HOURS     0x001
#define EE_ADDR_UPLOAD_HOURS      0x005

// 区块 2: MQTT 配置
#define EE_ADDR_MQTT_MAGIC        0x020
#define EE_ADDR_MQTT_CLIENT_ID    0x021
#define EE_ADDR_MQTT_USERNAME     0x061
#define EE_ADDR_MQTT_PASSWORD     0x0A1
#define EE_ADDR_MQTT_PORT         0x0E1
#define EE_ADDR_MQTT_SERVER       0x0E3
#define EE_ADDR_MQTT_TOPIC_UP     0x123

// 区块 3: 压力阈值 (vpthreshold)
#define EE_ADDR_VPTH_MAGIC        0x1B0
#define EE_ADDR_VPTH_ENABLED      0x1B1
#define EE_ADDR_VPTH_PPMAX        0x1B2
#define EE_ADDR_VPTH_PMAX         0x1B6
#define EE_ADDR_VPTH_PPMIN        0x1BA
#define EE_ADDR_VPTH_PMIN         0x1BE
#define EE_ADDR_VPTH_MUTATION     0x1C2
#define EE_ADDR_VPTH_SHOCK        0x1C6
#define EE_ADDR_KEEPALIVE_SEC     0x1CA   /* 2B uint16 MQTT心跳 (与压力阈值同一 magic) */

// 区块 4: 温度阈值 (Tthreshold)
#define EE_ADDR_TTH_MAGIC         0x1D0
#define EE_ADDR_TTH_ENABLED       0x1D1
#define EE_ADDR_TTH_HIGH          0x1D2
#define EE_ADDR_TTH_LOW           0x1D6
#define EE_ADDR_TTH_VHIGH         0x1DA
#define EE_ADDR_TTH_VLOW          0x1DE

// 区块 5: 电压检测 (spupower)
#define EE_ADDR_SPUP_MAGIC        0x1F0
#define EE_ADDR_SPUP_ENABLED      0x1F1
#define EE_ADDR_SPUP_HIGH         0x1F2
#define EE_ADDR_SPUP_LOW          0x1F6
#define EE_ADDR_SPUP_PROTECT      0x1FA

// 区块 6: 扩展配置
#define EE_ADDR_EXT_MAGIC         0x200
#define EE_ADDR_WINDOW_PATTERN    0x201
#define EE_ADDR_WINDOW_END_RESULT 0x209
#define EE_ADDR_WINDOW_START_STR  0x20A
#define EE_ADDR_WINDOW_STOP_STR   0x212
#define EE_ADDR_WINDOW_DOTTIME    0x21A
#define EE_ADDR_WINDOW_DOTTIME_MASK 0x2E0   /* dot 24h bitmap (3B); 0x21A keeps legacy first point */
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
#define EE_ADDR_MOTOR_CS_TH       0x232   /* 2B uint16: 堵转电流阈值(ADC原始值, 0=未标定) */
#define EE_ADDR_VALVE_POS_VALID   0x234   /* 1B: 阀门位置可信标志(脏标志法, 回零后置1) */
#define EE_ADDR_VALVE_POS_STATE   0x235   /* 1B: 阀门物理位置(0=关到底, 1=开到底; 配合 POS_VALID) */
#define EE_ADDR_VALVE_ALARM_CLOSED 0x284
#define EE_ADDR_VALVE_PRE_ALARM_ORDER 0x285
#define EE_ADDR_CH4_ALARM_CLOSED  0x289
#define EE_ADDR_LCD_ENABLED       0x286
#define EE_ADDR_SENSOR_COUNT      0x287
#define EE_ADDR_VLP_FLAG          0x288
#define EE_ADDR_REPORT_DELAY      0x28A
#define EE_ADDR_VALVE_TIMEOUT     0x294

// 区块 7: 阀门到位延时 (valve_delay)
#define EE_ADDR_VDLY_MAGIC        0x290
#define EE_ADDR_VDLY_ENABLED      0x291
#define EE_ADDR_VDLY_DELAY_MS     0x292

// 区块 8: 第二路压力阈值 (vpthreshold2, Magic 0xC4)
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

// 区块 9: 第二路温度阈值 (Tthreshold2, Magic 0xC5)
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

// 区块 10: 用户端MQTT配置 (UserMqtt, Magic 0xC6)
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

// 区块 11: 定时开关阀 (ValveTiming, Magic 0xC8) 存 YYMMDDHHMM
//   0x490: Magic
//   0x491: on_switch  (uint8)  定时开阀开关
//   0x492: off_switch (uint8)  定时关阀开关
//   0x493-0x49D: open_time  (char[11], YYMMDDHHMM)
//   0x49E-0x4A8: close_time (char[11], YYMMDDHHMM)
#define EE_ADDR_VTIMING_MAGIC       0x490
#define EE_ADDR_VTIMING_ON_SWITCH   0x491
#define EE_ADDR_VTIMING_OFF_SWITCH  0x492
#define EE_ADDR_VTIMING_OPEN        0x493
#define EE_ADDR_VTIMING_CLOSE       0x49E

// 12: CH4 (Magic 0xC9)
//   0x4B0: Magic
//   0x4B1: enabled (1B, 仅存储兼容)
//   0x4B2: warmup_sec (1B)
//   0x4B3-0x4B6: threshold (float, 甲烷门限 %VOL; 浓度>=门限→报警)
//   0x4B7-0x4BE: 作废保留 (旧 alarm/critical, 不再读写)
#define EE_ADDR_CH4_MAGIC        0x4B0
#define EE_ADDR_CH4_ENABLED      0x4B1
#define EE_ADDR_CH4_WARMUP_SEC  0x4B2
#define EE_ADDR_CH4_THRESHOLD   0x4B3

// 区块 13: 激光甲烷定时采集 (LaserSchedule, Magic 0xCA)
//   0x4C0: Magic
//   0x4C1: laser_switch (uint8) 0=关 1=开 (仅管理定时读取时间点)
//   0x4C2-0x4C4: laser_time 24h位图 (bit0=0点..bit23=23点)
#define EE_ADDR_LASER_MAGIC       0x4C0
#define EE_ADDR_LASER_SWITCH      0x4C1
#define EE_ADDR_LASER_TIME        0x4C2
#define EE_MAGIC_LASER            0xCA

// 区块 14: 出气端/激光异常关阀 (Magic 0xCB)
//   0x4D0: Magic
//   0x4D1-0x4E0: abnormal2_close (16B string, 出气端 pv* 策略)
//   0x4E1: abnormal3_close (1B, 激光甲烷异常关阀 0=关 1=开)
#define EE_ADDR_ABN2_MAGIC        0x4D0
#define EE_ADDR_ABNORMAL2_CLOSE   0x4D1
#define EE_ADDR_ABNORMAL3_CLOSE   0x4E1
#define EE_MAGIC_ABN23            0xCB

// 区块 15: 存储布局版本 (Magic 0xCC)
//   0x4F0: Magic (0xCC)
//   0x4F1: layout_ver (1B) 当前 = EE_LAYOUT_VER_CURRENT
// 作用: 记录当前 EEPROM 分区布局版本。上电时若读到非当前版本(老固件升级上来),
//       主动格式化受影响的区(历史区/测漏区)后写回版本号, 使迁移行为确定,
//       不依赖"新地址恰好命中旧 magic"的概率。
#define EE_ADDR_LAYOUT_MAGIC      0x4F0
#define EE_ADDR_LAYOUT_VER        0x4F1

/* 区块 16: 上报周期防重锁 (Magic 0xCD, 跨重启保持)
 *   0x500: Magic
 *   0x501-0x504: last_upload_idx (uint32) = 绝对分钟 / 上报周期分钟 (整点网格序号)
 * 记录"最后一次成功上报的上报周期序号", 保证"一个上报周期只上报一次"(跨重启保持);
 * 旧设备无此 magic → 读回 0xFFFFFFFF(未上报), 兼容。
 * 注: 本工程 0x4F0/0x4F1 已被区块 15(存储布局版本, magic 0xCC) 占用, 故从 0x500 起。 */
#define EE_ADDR_UPL_MAGIC         0x500   /* 1B  magic */
#define EE_ADDR_UPL_DONE_IDX      0x501   /* 4B  uint32 上报周期序号 */

// ==================== Magic ====================值 ====================
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
#define EE_MAGIC_VTIMING          0xC8
#define EE_MAGIC_CH4              0xC9
#define EE_MAGIC_LAYOUT           0xCC
#define EE_MAGIC_UPL              0xCD

/* 当前存储布局版本: 1=旧布局(测漏0x1000/600条 + 历史0x5B00/3367条)
 *                      2=新布局(测漏0x1000/2047条 + 历史0x11000/1919条)
 * 布局再次调整时该值 +1 即可, 设备自动迁移 */
#define EE_LAYOUT_VER_CURRENT     2


// 定时开关阀 (ValveTiming, YYMMDDHHMM)
#define DEF_VT_ON_SWITCH    0               // 定时开阀开关默认: 关
#define DEF_VT_OFF_SWITCH   0               // 定时关阀开关默认: 关
#define DEF_VT_OPEN_TIME    "0000000000"    // 开阀时间默认: 每天 00:00 (全部通配即每天)
#define DEF_VT_CLOSE_TIME   "0000000000"    // 关阀时间默认: 每天 00:00// ==================== 阈值默认值 (默认 关闭 阈值0; 用户配置后生效) ====================
// 说明: 阈值默认 switch=0 (功能关闭), 阈值=0; 用户配置后才生效
// 温度阈值 (Tthreshold) 为 第一路(进气端)
#define DEF_TT_SWITCH        0
#define DEF_TT_HIGH          0.0f
#define DEF_TT_LOW           0.0f
#define DEF_TT_VHIGH         0.0f
#define DEF_TT_VLOW          0.0f

// 温度阈值 (Tthreshold2) 为 第二路(出气端)
#define DEF_TT2_SWITCH       0
#define DEF_TT2_HIGH         0.0f
#define DEF_TT2_LOW          0.0f
#define DEF_TT2_VHIGH        0.0f
#define DEF_TT2_VLOW         0.0f

// 阀门到位延时 (valve_delay)
#define DEF_VD_SWITCH        0
#define DEF_VD_TIME_MS       0
// 安全上限: 阀门到位保持 PWM 驱动的最长时间5000(ms)
// 防止误配置超长延时, 保护电机与阀门
#define VDLY_DELAY_MS_MAX    0

/* 阀门限位到位等待超时 (ms): 可配置 1~120s (看门狗180s, 留余量) */
#define DEF_VALVE_TIMEOUT_MS   6000
#define VALVE_TIMEOUT_MS_MIN   1000
#define VALVE_TIMEOUT_MS_MAX   120000

// 电压检测 (spupower)
#define DEF_SP_SWITCH        0
#define DEF_SP_HIGH          0.0f
#define DEF_SP_LOW           0.0f
#define DEF_SP_PROTECT       0.0f

// 压力阈值 (vpthreshold) 为 第一路(进气端,单路传感器依旧使用此
#if APP_DEFAULT_SENSOR_COUNT == 1
#define DEF_VP_SWITCH        1
#define DEF_VP_PPMAX         0.0f
#define DEF_VP_PMAX          0.0f
#define DEF_VP_PPMIN         0.0f
#define DEF_VP_PMIN          0.0f
#define DEF_VP_MUTATION      0.0f
#define DEF_VP_SHOCK         100.0f
#elif APP_DEFAULT_SENSOR_COUNT == 2
#define DEF_VP_SWITCH        1
#define DEF_VP_PPMAX         0.0f
#define DEF_VP_PMAX          0.0f
#define DEF_VP_PPMIN         0.0f
#define DEF_VP_PMIN          0.0f
#define DEF_VP_MUTATION      0.0f
#define DEF_VP_SHOCK         3000.0f
#endif

// 压力阈值 (vpthreshold2) 为 第二路(出气端)
#define DEF_VP2_SWITCH       1
#define DEF_VP2_PPMAX        0.0f
#define DEF_VP2_PMAX         0.0f
#define DEF_VP2_PPMIN        0.0f
#define DEF_VP2_PMIN         0.0f
#define DEF_VP2_MUTATION     0.0f
#define DEF_VP2_SHOCK        100.0f

/* CH4 enabled 字段仅作 EEPROM 存储兼容/打印, 不再参与任何逻辑;
 * 激光传感器功能由编译宏 LASER_CH4_ENABLED (app_config.h) 决定 */
#define DEF_CH4_ENABLED        0
#define DEF_CH4_WARMUP_SEC    20
#define DEF_CH4_THRESHOLD     0.5f
#define DEF_CH4_THRESHOLD_MIN 0.0f
#define DEF_CH4_THRESHOLD_MAX 999.0f

/* 激光甲烷定时采集配置 (laser_switch 仅管理读取时间点, 不控制 gas_get) */
#define DEF_LASER_SWITCH      	1
#define DEF_LASER_TIME_DEFAULT "08,20"   /* 出厂默认读取时间点: 08:00, 20:00 (平台下发后以平台为准) */
extern uint8_t  g_laser_switch;        /* 0=关闭定时读取, 1=开启定时读取 */
extern uint8_t  g_laser_time_mask[3];  /* 24h位图: bit0=0点..bit23=23点 */

// CH4
typedef struct {
    uint8_t enabled;        /* 存储兼容, 不参与逻辑 */
    uint8_t warmup_sec;     /* 暖机时间(秒) */
    float   threshold;      /* 甲烷门限 %VOL: 浓度>=门限 → 报警(1) */
} CH4_ThresholdConfig_t;

extern CH4_ThresholdConfig_t      g_ch4_threshold;



// 扩展字段最大长度
#define MQTT_FIELD_MAX            64
#define MQTT_TOPIC_MAX            128

// ==================== 配置结构体定义 ====================

typedef struct {
    float collect_interval_hours;   // 采集间隔(小时)
    float upload_interval_hours;    // 上报间隔(小时)
    uint16_t collect_interval_min;  // 采集间隔: 分钟
    uint16_t upload_interval_min;   // 上报间隔: 分钟
    uint16_t upload_every_n;        // 上报间隔: 每N次上报
} AppInterval_t;

typedef struct {
    char client_id[MQTT_FIELD_MAX];
    char username[MQTT_FIELD_MAX];
    char password[MQTT_FIELD_MAX];
    char server[MQTT_FIELD_MAX];        // 主IP
    char server2[MQTT_FIELD_MAX];       // 副IP (为故障切换)
    char topic_up[MQTT_TOPIC_MAX];
    uint16_t port;                      // 主端口
    uint16_t port2;                     // 副端口
} MqttConfig_t;

// 温度阈值 (Tthreshold_set/get)
typedef struct {
    uint8_t enabled;            // 0=关闭温度检测, 1=开启
    float   high;               // 温度上限
    float   low;                // 温度下限
    float   vhigh;              // 温度极高阈值
    float   vlow;               // 温度极低阈值
} TempThresholdConfig_t;

// 阀门到位延时 (delay_set/get)
typedef struct {
    uint8_t  enabled;           // 0=无延时, 1=到位后保持原动作 delay_ms
    uint16_t delay_ms;          // 到位延时 (ms)
} ValveDelayConfig_t;

// 定时开关阀 (closeoropen_set/get)
typedef struct {
    uint8_t  on_switch;         // 定时开阀开关: 0=关, 1=开
    uint8_t  off_switch;        // 定时关阀开关: 0=关, 1=开
    char     open_time[11];     // 开阀时间 YYMMDDHHMM (全0=每天00:00)
    char     close_time[11];    // 关阀时间 YYMMDDHHMM
} ValveTimingConfig_t;


// 电压检测 (spupower_set/get)
typedef struct {
    uint8_t enabled;            // 0=关闭电压检测(含 VLP), 1=开启
    float   high;               // 电压上限
    float   low;                // 电压下限
    float   protect;            // 电压保护值 (触发 VLP 省电)
} VoltagePowerConfig_t;

// 压力阈值 (vpthreshold_set1/get1)
typedef struct {
    uint8_t enabled;            // 0=关闭压力阈值, 1=开启
    float   ppmax;              // 压力极高阈值
    float   pmax;               // 压力上限
    float   ppmin;              // 压力极低阈值
    float   pmin;               // 压力下限
    float   mutation;           // 压力突变阈值 (|Δp|, 不受 shock 影响)
    float   shock;              // 静态阈值滞回带 (作用于 pmax/pmin/ppmax/ppmin)
} PressureThresholdConfig_t;

// 用户端MQTT配置 (第二路通道, id=1)
typedef struct {
    char server[MQTT_FIELD_MAX];        // 用户端服务器IP/域名
    uint16_t port;                      // 用户端端口
    char username[MQTT_FIELD_MAX];      // 用户端用户名
    char password[MQTT_FIELD_MAX];      // 用户端密码
    char client_id[MQTT_FIELD_MAX];     // 用户端client_id (同时用作deviceKey)
    char topic_up[MQTT_TOPIC_MAX];      // 用户端上报topic
} UserMqttConfig_t;

// 全部配置结构体通过extern声明 (定义在config_store.c)
extern AppInterval_t              g_interval;
extern float                      g_report_delay_min;   // 周期上报后延分钟 (0~60, 支持小数)
extern MqttConfig_t               g_mqtt;
extern UserMqttConfig_t           g_user_mqtt;            // 用户端MQTT配置
extern WindowConfig_t             g_window;
extern CycleConfig_t              g_collect_cycle;
extern CycleConfig_t              g_report_cycle;
extern char                       g_abnormal_close[16];
extern char                       g_abnormal2_close[16];   // 出气端异常关阀策略 (pv*)
extern uint8_t                    g_abnormal3_close;       // 激光甲烷异常关阀 0=关 1=开
extern uint8_t                    g_valve_last_order;
extern uint8_t                    g_valve_alarm_closed;    // 报警自动关阀标志 (EEPROM持久化, OTA重启后自动恢复)
extern uint8_t                    g_valve_pre_alarm_order; // 关阀前阀门状态 (EEPROM持久化)
extern uint8_t                    g_ch4_alarm_closed;      // 激光甲烷报警关阀标志 (EEPROM持久化, 平台 valve_open 清除)
extern uint8_t                    g_real_enabled;         // 实时采集开关 (0=关, 1=开, EEPROM持久化)
extern uint8_t                    g_real_frequency;       // 实时采集频率 (1-10秒)
extern uint8_t                    g_lcd_enabled;          // LCD显示开关 (0=关, 1=开, EEPROM持久化)
extern uint8_t                    g_sensor_count;         // 传感器数量 (1=单路, 2=双路, EEPROM持久化)
extern uint16_t                   g_keepalive_sec;        // MQTT心跳(秒)
extern uint32_t                   g_valve_timeout_ms;     // 阀门限位到位等待超时(ms), 默认40000
extern uint16_t                   g_motor_cs_th;          // 堵转判定电流阈值(ADC原始值; 0=未标定, 判定关闭)
extern uint8_t                    g_valve_pos_valid;      // 阀门位置可信标志(1=可信; 0=中途掉电/未回零)
/* 阀门物理位置(0=关到底, 1=开到底): 与"用户最后指令" g_valve_last_order 解耦,
 * 平台指令/定时阀/异常关阀/回零 等所有确认到位的动作都会更新它 */
extern uint8_t                    g_valve_pos_state;
extern TempThresholdConfig_t      g_tthreshold;           // 第一路温度阈值
extern TempThresholdConfig_t      g_tthreshold2;          // 第二路温度阈值
extern ValveDelayConfig_t         g_vdelay;               // 阀门到位延时
extern VoltagePowerConfig_t       g_spupower;             // 电压检测
extern PressureThresholdConfig_t  g_pthreshold;           // 第一路压力阈值
extern PressureThresholdConfig_t  g_pthreshold2;          // 第二路压力阈值
extern ValveTimingConfig_t        g_valve_timing;



// ==================== 接口函数 ====================

// 从EEPROM加载全部配置(上电调用)
void config_load_all(void);

// 保存采集间隔到EEPROM
void config_save_interval(void);

// 保存MQTT配置到EEPROM
void config_save_mqtt(void);

// 由client_id推导topic_up
void derive_topic_from_client_id(void);

// 保存扩展配置到EEPROM (window/cycle/abnormal/valve_last_order)
void config_save_ext(void);

/* 上报周期防重锁 (持久化, 区块16): 上报成功后提交"上报周期序号"; 读回无效时返回 0xFFFFFFFF */
void     config_save_upload_idx(uint32_t idx);
uint32_t config_load_upload_idx(void);

// 保存压力阈值配置 (含 keepalive)
void config_save_vpthreshold(void);

// 保存温度阈值配置
void config_save_tthreshold(void);

// 保存电压检测配置
void config_save_spupower(void);

// 保存阀门延时配置
void config_save_vdelay(void);

// 保存第二路压力阈值到EEPROM
void config_save_vpthreshold2(void);

// 保存第二路温度阈值到EEPROM
void config_save_tthreshold2(void);

// 保存用户端MQTT配置
void config_save_user_mqtt(void);

// 保存定时开关阀配置
void config_save_valve_timing(void);

void config_save_ch4_threshold(void);

// 保存激光甲烷定时采集配置 (laser_switch + laser_time 位图)
void config_save_laser(void);

// 保存出气端/激光异常关阀配置 (abnormal2_close + abnormal3_close)
void config_save_abn23(void);

/* 单字段写入 (堵转方案): 电流阈值 / 阀门位置可信标志
 * 注: 动作频繁, 用单字段写避免整块 EXT 重写, 延长 EEPROM 寿命 */
void config_save_motor_cs_th(void);
void config_save_valve_pos_valid(void);

// 获取心跳值(含默认值)
uint16_t config_get_keepalive(void);

// 周期字符串 <-> 枚举 转换
CycleType_t cycle_type_from_str(const char *str);
const char* cycle_type_to_str(CycleType_t type);

// 将周期配置转换为秒数
uint32_t cycle_to_seconds(const CycleConfig_t *cfg);

/* 周期上限/越界钳制 (保证 minutes 域不溢出, 支持跨天周期)
 * 注: 与 config_policy.h 同源声明; downlink.c 经由本头文件取用 */
uint32_t cycle_max_once(CycleType_t type);
uint8_t  cycle_clamp(CycleConfig_t *cfg);

// 异常关阀策略 <-> 枚举
AbnormalClose_t abnormal_from_str(const char *str);
const char* abnormal_to_str(AbnormalClose_t ac);

// 判断异常结果是否匹配策略 (1=应自动关阀, 0=不关阀)
int abnormal_should_close_valve(AbnormalClose_t policy, ThresholdResult_t result);

// 首次配置模式: MQTT未配置时阻塞等待30秒, 超时后用默认值
void config_enter_setup_mode(uint8_t force_reconfig);

// 打印当前配置到debug串口
void config_print_current(void);

// 根据周期配置重算间隔
void config_recalc_interval(void);

/* ==================== 存储布局版本 / 迁移 ====================
 * 上电调用(config_load_all + ee_hist_init + leak_store_init 之后, 任何历史/测漏读写之前):
 *   读到当前布局版本 → 直接返回 0
 *   否则(老固件升级/首次烧写设备) → 格式化历史区与测漏区, 写入当前版本, 返回 1
 * 保障: 布局变更后不会把旧分区的残留数据当成有效历史/测漏数据。 */
uint8_t eeprom_layout_check_and_migrate(void);

// 统一配置解析函数(可处理部分命令)
// 返回: 1=有效命令, 0=无效, -1=reset(恢复默认并重启)
int config_parse_and_apply(const char *cmd);

// 主循环轮询debug串口命令(每次一行, ISR置位标志后响应)
void config_check_cmd(void);

// 判断当前RTC时间是否在活动窗口
// 返回: 1=活动(应上报), 0=非活动(仅采集)
int config_is_active_window(void);
// 按指定时刻(HHMMSS)判断是否在活动窗口 (后延上报用网格时刻判断)
int config_is_active_window_at(uint32_t hhmmss);

// 低电压省电模式标志 (定义于 main.c)
extern uint8_t voltage_low_power;

// 判断当前采集周期是否需要采样电池电压
// 仅在首次、尾次或低电压保护模式下开启采样, 其余中间采集点保持关闭以省电
uint8_t config_need_adc_sampling(void);


/* ==================== dot pattern (24-hour bitmap) helpers ==================== */
void window_dottime_clear(uint8_t *mask);
void window_dottime_set(uint8_t *mask, uint8_t hour);
uint8_t window_dottime_hit(const uint8_t *mask, uint8_t hour);
uint8_t window_dottime_first(const uint8_t *mask);
void window_dottime_from_str(const char *str, uint8_t *mask);
void window_dottime_to_str(const uint8_t *mask, char *buf, uint16_t buf_size);
#endif /* EEPROM_CONFIG_H */
