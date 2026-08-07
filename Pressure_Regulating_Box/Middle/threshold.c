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
//   1. spupower.protect (VPROTECT, 仅报警; VLP模式由 main.c 门控)
//   2. vpthreshold ppmax/ppmin + shock (压力极限)
//   3. vpthreshold pmax/pmin + shock   (压力上下限)
//   4. Tthreshold vhigh/vlow            (温度极限)
//   5. Tthreshold high/low              (温度上下限)
//   6. spupower high/low                (电压上下限)
//   7. vpthreshold mutation             (压力突变, 不受 shock 影响)
ThresholdResult_t threshold_check(float pressure, float temperature, float voltage, float last_pressure, uint8_t check_voltage, uint8_t skip_mutation)
{
    // ---- 1. 电压保护(VPROTECT) — spupower.enabled 关闭时跳过, 不触发报警/VLP ----
    // voltage >= 0 守卫: ADC 超时返回 -1.0, 不可参与电压判定, 否则误触发 VPROTECT
    // 进入滞回: 已在VLP时用 protect+0.15 判断(由do_collect处理退出), 未在VLP时用 protect-0.1 防振荡
    if(g_spupower.enabled && check_voltage && voltage >= 0.0f && voltage < g_spupower.protect - 0.1f)
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

    // ---- 6. 电压上下限 (spupower.enabled 关闭时跳过, 不触发报警)
    if(g_spupower.enabled && check_voltage && voltage >= 0.0f)
    {
        if(voltage > g_spupower.high)
            return THRESHOLD_VOLT_HIGH;
        if(voltage < g_spupower.low)
            return THRESHOLD_VOLT_LOW;
    }

    // ---- 7. 压力突变 (last_pressure < 0 表示首次采集,跳过; 不应用 shock) ----
    if(!skip_mutation && g_pthreshold.enabled && last_pressure >= 0)
    {
        float delta = pressure - last_pressure;
        if(delta < 0) delta = -delta;
        if(delta > g_pthreshold.mutation)
            return THRESHOLD_MUTATION;
    }

    return THRESHOLD_OK;
}

// ==================== 报警类型字符串 ====================
// 将 ThresholdResult_t 映射为上报用报警类型字符串 (供 do_alarm/do_realtime_check/downlink 共用)
const char* threshold_alarm_type_str(ThresholdResult_t result)
{
    switch(result)
    {
        case THRESHOLD_VPROTECT:        return "voltage_protect";
        case THRESHOLD_VERY_LOW:        return "pressure_verylow";
        case THRESHOLD_VERY_HIGH:       return "pressure_veryhigh";
        case THRESHOLD_OVER_LIMIT:      return "pressure_high";
        case THRESHOLD_UNDER_LIMIT:     return "pressure_low";
        case THRESHOLD_MUTATION:        return "pressure_mutation";
        case THRESHOLD_TEMP_HIGH:       return "temperature_high";
        case THRESHOLD_TEMP_LOW:        return "temperature_low";
        case THRESHOLD_TEMP_VERY_HIGH:  return "temperature_veryhigh";
        case THRESHOLD_TEMP_VERY_LOW:   return "temperature_verylow";
        case THRESHOLD_VOLT_HIGH:       return "voltage_high";
        case THRESHOLD_VOLT_LOW:        return "voltage_low";
        default:                        return "unknown";
    }
}

// ==================== 电压报警状态 (LY06 语义, 单一真源) ====================
// -1=未知(ADC无效 voltage<0), 0=正常, 1=报警
// 报警条件: voltage > high || voltage < low || voltage < protect
int threshold_voltage_alarm(float voltage)
{
//    if(voltage < 0.0f || !g_spupower.enabled)
	if(voltage < 0.0f)
        return -1;
    if(voltage > g_spupower.high || voltage < g_spupower.low || voltage < g_spupower.protect)
        return 1;
    return 0;
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

// ==================== 出气端(第二路)阈值检查 ====================
// 基于 g_pthreshold2/g_tthreshold2, 不含电压检查 (电压为全局共享)
ThresholdResult_t threshold_check2(float pressure, float temperature, float last_pressure, uint8_t skip_mutation)
{
    // ---- 压力检查 (含 shock 滞回带) ----
    if(g_pthreshold2.enabled)
    {
        float shock = g_pthreshold2.shock;
        if(pressure > g_pthreshold2.ppmax + shock)
            return THRESHOLD_VERY_HIGH;
        if(pressure < g_pthreshold2.ppmin - shock)
            return THRESHOLD_VERY_LOW;
        if(pressure > g_pthreshold2.pmax + shock)
            return THRESHOLD_OVER_LIMIT;
        if(pressure < g_pthreshold2.pmin - shock)
            return THRESHOLD_UNDER_LIMIT;
    }

    // ---- 温度检查 ----
    if(g_tthreshold2.enabled)
    {
        if(temperature > g_tthreshold2.vhigh)
            return THRESHOLD_TEMP_VERY_HIGH;
        if(temperature < g_tthreshold2.vlow)
            return THRESHOLD_TEMP_VERY_LOW;
        if(temperature > g_tthreshold2.high)
            return THRESHOLD_TEMP_HIGH;
        if(temperature < g_tthreshold2.low)
            return THRESHOLD_TEMP_LOW;
    }

    // ---- 压力突变 (last_pressure < 0 表示首次采集,跳过) ----
    if(!skip_mutation && g_pthreshold2.enabled && last_pressure >= 0)
    {
        float delta = pressure - last_pressure;
        if(delta < 0) delta = -delta;
        if(delta > g_pthreshold2.mutation)
            return THRESHOLD_MUTATION;
    }

    return THRESHOLD_OK;
}

// ==================== 出气端(第二路)压力状态查询 ====================
// 基于 g_pthreshold2 配置, 逻辑同 threshold_get_pressure_status
PressureStatus_t threshold_get_pressure_status2(float pressure, float last_pressure)
{
    if(!g_pthreshold2.enabled)
        return PRESSURE_NORMAL;

    float shock = g_pthreshold2.shock;

    if(pressure > g_pthreshold2.ppmax + shock)
        return PRESSURE_VERYHIGH;
    if(pressure < g_pthreshold2.ppmin - shock)
        return PRESSURE_VERYLOW;
    if(pressure > g_pthreshold2.pmax + shock)
        return PRESSURE_HIGH;
    if(pressure < g_pthreshold2.pmin - shock)
        return PRESSURE_LOW;

    if(last_pressure >= 0)
    {
        float delta = pressure - last_pressure;
        if(delta < 0) delta = -delta;
        if(delta > g_pthreshold2.mutation)
            return PRESSURE_MUTATION;
    }
    return PRESSURE_NORMAL;
}

// ==================== 出气端(第二路)温度状态查询 ====================
// 基于 g_tthreshold2 配置, 逻辑同 threshold_get_temp_status
TempStatus_t threshold_get_temp_status2(float temperature)
{
    if(!g_tthreshold2.enabled)
        return TEMP_NORMAL;

    if(temperature > g_tthreshold2.vhigh)
        return TEMP_VERY_HIGH;
    if(temperature < g_tthreshold2.vlow)
        return TEMP_VERY_LOW;
    if(temperature > g_tthreshold2.high)
        return TEMP_HIGH;
    if(temperature < g_tthreshold2.low)
        return TEMP_LOW;
    return TEMP_NORMAL;
}

/* EEPROM 加载/保存/解析逻辑已迁移到 eeprom_config.c
 * 心跳值统一使用 config_get_keepalive(); 阈值字段统一通过 g_pthreshold/g_tthreshold/g_spupower 访问
 */
