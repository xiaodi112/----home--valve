#ifndef THRESHOLD_H
#define THRESHOLD_H

#include <stdint.h>
#include "app_config.h"

/**
 * @file threshold.h
 * @brief 阈值检查模块 — 压力/温度/电压/突变多组检查
 *
 * 配置存储在EEPROM (由 eeprom_config 管理):
 *   - g_pthreshold (压力阈值, 含 shock 滞回带)
 *   - g_tthreshold (温度阈值)
 *   - g_spupower   (电压检测, 含 VLP 保护)
 *
 * shock 滞回规则 (仅作用于压力静态阈值):
 *   pressure > pmax + shock  → over_limit
 *   pressure < pmin - shock  → under_limit
 *   pressure > ppmax + shock → very_high
 *   pressure < ppmin - shock → very_low
 */

// 阈值检查结果
typedef enum {
    THRESHOLD_OK = 0,           // 正常
    THRESHOLD_OVER_LIMIT,       // 压力超上限 (> pmax + shock)
    THRESHOLD_UNDER_LIMIT,      // 压力低于下限 (< pmin - shock)
    THRESHOLD_MUTATION,         // 压力突变 (|Δp| > mutation)
    THRESHOLD_VERY_LOW,         // 压力极低 (< ppmin - shock)
    THRESHOLD_VERY_HIGH,        // 压力极高 (> ppmax + shock)
    THRESHOLD_VPROTECT,         // 电压保护 (< spupower.protect, 进入 VLP)
    THRESHOLD_TEMP_HIGH,        // 温度过高 (> high)
    THRESHOLD_TEMP_LOW,         // 温度过低 (< low)
    THRESHOLD_TEMP_VERY_HIGH,   // 温度极高 (> vhigh)
    THRESHOLD_TEMP_VERY_LOW,    // 温度极低 (< vlow)
    THRESHOLD_VOLT_HIGH,        // 电压过高 (> spupower.high)
    THRESHOLD_VOLT_LOW          // 电压过低 (< spupower.low)
} ThresholdResult_t;

// ==================== 接口函数 ====================

// 阈值检查: 依据 g_pthreshold/g_tthreshold/g_spupower 三组配置进行全面检查
// last_pressure: 上次采集的压力值 (用于突变检测, 首次传-1忽略)
// 优先级: VPROTECT > PRESSURE_VERY > PRESSURE > TEMP_VERY > TEMP > VOLT > MUTATION
ThresholdResult_t threshold_check(float pressure, float temperature, float voltage, float last_pressure, uint8_t check_voltage);

// 获取压力状态枚举 (用于 get_pressure 上报)
// switch=0 时返回 PRESSURE_NORMAL
PressureStatus_t threshold_get_pressure_status(float pressure, float last_pressure);

// 获取压力状态字符串 (用于JSON上报)
const char* threshold_pressure_status_str(PressureStatus_t status);

// 获取温度状态枚举 (用于 temp_get1 上报)
// switch=0 时返回 TEMP_NORMAL
TempStatus_t threshold_get_temp_status(float temperature);

// 获取温度状态字符串 (用于JSON上报)
const char* threshold_temp_status_str(TempStatus_t status);

#endif /* THRESHOLD_H */
