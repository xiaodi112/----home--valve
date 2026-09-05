#ifndef MAIN_H
#define MAIN_H

#include "stdint.h"
#include "PT_304_35.h"
#include "ota_info.h"
#include "threshold.h"

#define APP_FW_VERSION_NUM     1

/* ==================== State Machine ==================== */
typedef enum {
    STATE_INIT = 0,
    STATE_SLEEP,
    STATE_WAKEUP,
    STATE_COLLECT,
    STATE_UPLOAD,
    STATE_DOWNLINK,
    STATE_ALARM,
    STATE_BLE_OTA,
    STATE_CH4
} AppState_t;

extern AppState_t app_state;

/* CH4 request flag (set by downlink gas_get, checked by do_downlink) */
extern volatile uint8_t g_ch4_request;
extern uint8_t g_alarm_recover_reported; /* 报警关阀恢复"已上报一次"标志 (RAM) */
typedef struct {
    PT304D_Data raw;
    PT304D_Data raw2;
    float       voltage;
    float       last_pressure_collect;
    float       last_pressure_realtime;
    float       last_pressure_collect2;
    float       last_pressure_realtime2;
    float       last_normal_voltage;
    uint8_t     prev_online;
    uint8_t     offline_cnt;
    uint8_t     online_cnt;
    uint8_t     prev_online2;
    uint8_t     offline_cnt2;
    uint8_t     online_cnt2;
} SensorContext_t;

extern SensorContext_t g_sensor;
extern uint8_t g_lcd_active;   // LCD当前是否显示 (运行状态, 区别于配置项; main.c 维护)

// 供其他模块使用 (main.c 实现, downlink.c 等模块调用)
void lcd_shutdown(void);

const char* alarm_try_close_valve_all(const char *policy_str, ThresholdResult_t result);

const char* alarm_try_close_valve(ThresholdResult_t result);
const char* alarm_try_close_valve2(ThresholdResult_t result);   /* 出气端 (abnormal2_close) */
const char* ch4_alarm_try_close_valve(void);
void threshold_alarm_reset_rt(void);
void upload_schedule_reset(void);   /* 上报周期/后延配置变更后调用, 重置延迟上报锁存 */

#endif
