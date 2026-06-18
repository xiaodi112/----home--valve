/**
 * @file threshold.c
 * @brief 阈值检查模块 — 依据 g_pthreshold/g_tthreshold/g_spupower 三组配置
 *
 * 数据存储由 eeprom_config 管理, 本模块只负责检查逻辑.
 * shock 滞回带仅作用于压力静态阈值 (ppmax/pmax/ppmin/pmin), 不作用于 mutation.
 */

#include "threshold.h"
#include "eeprom_config.h"
#include "app_config.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// ==================== 阈值检查 ====================
// 优先级:
//   1. spupower.protect (VLP, 跟随 switch 开关)
//   2. vpthreshold ppmax/ppmin + shock (压力极限)
//   3. vpthreshold pmax/pmin + shock   (压力上下限)
//   4. Tthreshold vhigh/vlow            (温度极限)
//   5. Tthreshold high/low              (温度上下限)
//   6. spupower high/low                (电压上下限)
//   7. vpthreshold mutation             (压力突变, 不受 shock 影响)
ThresholdResult_t threshold_check(float pressure, float temperature, float voltage, float last_pressure, uint8_t check_voltage)
{
    // ---- 1. 电压保护(VLP) — 跟随 spupower.switch ----
    if(check_voltage && g_spupower.enabled && voltage < g_spupower.protect)
        return THRESHOLD_VPROTECT;

    // ---- 2/3. 压力检查 (含 shock 滞回带) ----
    if(g_pthreshold.enabled)
    {
        float shock = g_pthreshold.shock;
        // 压力极限 (ppmin/ppmax)
        if(pressure > g_pthreshold.ppmax + shock)
            return THRESHOLD_VERY_HIGH;
        if(pressure < g_pthreshold.ppmin - shock)
            return THRESHOLD_VERY_LOW;
        // 普通上下限
        if(pressure > g_pthreshold.pmax + shock)
            return THRESHOLD_OVER_LIMIT;
        if(pressure < g_pthreshold.pmin - shock)
            return THRESHOLD_UNDER_LIMIT;
    }

    // ---- 4/5. 温度检查 ----
    if(g_tthreshold.enabled)
    {
        if(temperature > g_tthreshold.vhigh)
            return THRESHOLD_TEMP_VERY_HIGH;
        if(temperature < g_tthreshold.vlow)
            return THRESHOLD_TEMP_VERY_LOW;
        if(temperature > g_tthreshold.high)
            return THRESHOLD_TEMP_HIGH;
        if(temperature < g_tthreshold.low)
            return THRESHOLD_TEMP_LOW;
    }

    // ---- 6. 电压上下限 (spupower.switch=1 时) ----
    if(check_voltage && g_spupower.enabled)
    {
        if(voltage > g_spupower.high)
            return THRESHOLD_VOLT_HIGH;
        if(voltage < g_spupower.low)
            return THRESHOLD_VOLT_LOW;
    }

    // ---- 7. 压力突变 (last_pressure < 0 表示首次采集,跳过; 不应用 shock) ----
    if(g_pthreshold.enabled && last_pressure >= 0)
    {
        float delta = pressure - last_pressure;
        if(delta < 0) delta = -delta;
        if(delta > g_pthreshold.mutation)
            return THRESHOLD_MUTATION;
    }

    return THRESHOLD_OK;
}

// ==================== 压力状态查询 ====================
// switch=0 → PRESSURE_NORMAL; shock 滞回应用于所有静态边界
PressureStatus_t threshold_get_pressure_status(float pressure, float last_pressure)
{
    if(!g_pthreshold.enabled)
        return PRESSURE_NORMAL;

    float shock = g_pthreshold.shock;

    if(pressure > g_pthreshold.ppmax + shock)
        return PRESSURE_VERYHIGH;
    if(pressure < g_pthreshold.ppmin - shock)
        return PRESSURE_VERYLOW;
    if(pressure > g_pthreshold.pmax + shock)
        return PRESSURE_HIGH;
    if(pressure < g_pthreshold.pmin - shock)
        return PRESSURE_LOW;

    if(last_pressure >= 0)
    {
        float delta = pressure - last_pressure;
        if(delta < 0) delta = -delta;
        if(delta > g_pthreshold.mutation)
            return PRESSURE_MUTATION;
    }
    return PRESSURE_NORMAL;
}

const char* threshold_pressure_status_str(PressureStatus_t status)
{
    switch(status)
    {
        case PRESSURE_LOW:       return "pressure_low";
        case PRESSURE_HIGH:      return "pressure_high";
        case PRESSURE_VERYLOW:   return "pressure_verylow";
        case PRESSURE_VERYHIGH:  return "pressure_veryhigh";
        case PRESSURE_MUTATION:  return "pressure_mutation";
        default:                 return "pressure_normal";
    }
}

// ==================== 温度状态查询 ====================
// switch=0 → TEMP_NORMAL; vhigh/vlow 优先于 high/low
TempStatus_t threshold_get_temp_status(float temperature)
{
    if(!g_tthreshold.enabled)
        return TEMP_NORMAL;

    if(temperature > g_tthreshold.vhigh)
        return TEMP_VERY_HIGH;
    if(temperature < g_tthreshold.vlow)
        return TEMP_VERY_LOW;
    if(temperature > g_tthreshold.high)
        return TEMP_HIGH;
    if(temperature < g_tthreshold.low)
        return TEMP_LOW;
    return TEMP_NORMAL;
}

const char* threshold_temp_status_str(TempStatus_t status)
{
    switch(status)
    {
        case TEMP_HIGH:       return "temp_high";
        case TEMP_LOW:        return "temp_low";
        case TEMP_VERY_HIGH:  return "temp_very_high";
        case TEMP_VERY_LOW:   return "temp_very_low";
        default:              return "temp_normal";
    }
}

/* EEPROM 加载/保存/解析逻辑已迁移到 eeprom_config.c
 * 心跳值统一使用 config_get_keepalive(); 阈值字段统一通过 g_pthreshold/g_tthreshold/g_spupower 访问
 */
