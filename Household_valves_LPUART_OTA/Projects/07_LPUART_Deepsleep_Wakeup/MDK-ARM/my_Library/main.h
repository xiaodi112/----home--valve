#ifndef MAIN_H
#define MAIN_H

#include "stdint.h"
#include "PT_304_35.h"
#include "ota_info.h"      /* Flash 分区 + OTA 状态码 (BL 与 App 共享) */

/* 当前 App 固件版本号 (整数, 与 ota_info.new_fw_version 比较)
 * 注意: 须与 app_config.h 中 APP_FIRMWARE_VERSION 保持一致 */
#define APP_FW_VERSION_NUM      1

/* ==================== 传感器上下文 (统一管理所有传感器相关状态) ==================== */
typedef struct {
    PT304D_Data raw;                   // PT304D #2 数据 (PB6/PB7 独立总线)
    PT304D_Data raw2;                  // PT304D #1 数据 (PC0/PC1 与 AT24C16 共用总线)
    float       voltage;              // 电池电压 (V)
    float       last_pressure_collect; // 周期采集突变检测基准 (基于 raw, PT304D #2)
    float       last_pressure_realtime;// 实时采集突变检测基准 (基于 raw, PT304D #2)
    float       last_normal_voltage;   // 缓存电压 (ADC关闭时复用)
    uint8_t     prev_online;           // 上次传感器在线状态 (1=在线, 基于 PT304D #2)
} SensorContext_t;

extern SensorContext_t g_sensor;

#endif