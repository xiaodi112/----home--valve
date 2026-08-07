#ifndef MAIN_H
#define MAIN_H

#include "stdint.h"
#include "PT_304_35.h"
#include "ota_info.h"
#include "threshold.h"

#define APP_FW_VERSION_NUM     1
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

// 公共功能函数 (main.c 定义, downlink.c 等模块调用)
void lcd_shutdown(void);
const char* alarm_try_close_valve(ThresholdResult_t result);

#endif
