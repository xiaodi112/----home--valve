/**
 * @file threshold.c
 * @brief 阈值检查模块 — 依据 g_pthreshold/g_tthreshold/g_spupower 三组配置
 *
 * 数据存储由 eeprom_config 管理, 本模块只负责检查逻辑.
 * 压力静态阈值(ppmax/pmax/ppmin/pmin)采用 ±shock 双边迟滞: 进入用 T+s, 退出用 T-s,
 * 带内保持上一次档位; mutation(压力突变)不参与迟滞.
 */

#include "threshold.h"
#include "eeprom_config.h"
#include "app_config.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* 阈值值 == 0.00 视为该单项关闭(不产生报警), 平台/debug串口配置均适用;
 * shock 滞回带是修正量, 不参与"0=关闭" */
#define THR_ACTIVE(v)  ((v) != 0.0f)

/* ==================== 压力静态阈值双边迟滞 (±shock) ====================
 * 规则(以高侧为例, T=阈值, s=shock):
 *   进入报警: p > T + s      退出报警: p < T - s
 *   带内(T-s ~ T+s): 保持上一次档位(不来回翻转) —— 这才是真正的迟滞
 * 低侧对称: 进入 p < T - s, 退出 p > T + s
 * 档位: 0=正常, 1=普通档(pmax/pmin), 2=极限档(ppmax/ppmin)
 * 状态为纯 RAM(上电清零); 阈值/shock/switch 变化时按配置快照自动复位,
 * 避免"改了阈值状态还锁在报警"。判定与上报共用同一状态, 保证两侧一致。 */
typedef struct {
    float   ppmax, pmax, ppmin, pmin, shock;
    uint8_t enabled;
} PressHystCfg_t;

static uint8_t        s_hi_level[2] = {0, 0};   /* [0]=进气端 [1]=出气端; 0=正常 1=高 2=超高 */
static uint8_t        s_lo_level[2] = {0, 0};   /* 0=正常 1=低 2=超低 */
static PressHystCfg_t s_hyst_cfg[2];

/* 配置变化(或首次使用)时复位迟滞状态 */
static void press_hyst_sync_cfg(uint8_t ch)
{
    const PressureThresholdConfig_t *cfg  = ch ? &g_pthreshold2 : &g_pthreshold;
    PressHystCfg_t                  *snap = &s_hyst_cfg[ch];

    if(snap->ppmax != cfg->ppmax || snap->pmax != cfg->pmax ||
       snap->ppmin != cfg->ppmin || snap->pmin != cfg->pmin ||
       snap->shock != cfg->shock || snap->enabled != cfg->enabled)
    {
        snap->ppmax = cfg->ppmax; snap->pmax = cfg->pmax;
        snap->ppmin = cfg->ppmin; snap->pmin = cfg->pmin;
        snap->shock = cfg->shock; snap->enabled = cfg->enabled;
        s_hi_level[ch] = 0;
        s_lo_level[ch] = 0;
    }
}

/* 更新某一通道的压力档位(双边迟滞), 判定与上报共用这一份状态 */
static void press_hyst_update(uint8_t ch, float p)
{
    const PressureThresholdConfig_t *cfg = ch ? &g_pthreshold2 : &g_pthreshold;
    float   s;
    uint8_t hi, lo;

    press_hyst_sync_cfg(ch);           /* 配置变更时状态已被复位 */

    s  = cfg->shock;
    hi = s_hi_level[ch];
    lo = s_lo_level[ch];

    if(!cfg->enabled)
    {
        s_hi_level[ch] = 0;
        s_lo_level[ch] = 0;
        return;
    }

    /* 高侧: 进入 > T+s; 已在该档则 > T-s 保持 */
    if(THR_ACTIVE(cfg->ppmax) && (p > cfg->ppmax + s || (hi == 2 && p > cfg->ppmax - s)))
        hi = 2;
    else if(THR_ACTIVE(cfg->pmax) && (p > cfg->pmax + s || (hi >= 1 && p > cfg->pmax - s)))
        hi = 1;
    else
        hi = 0;

    /* 低侧: 进入 < T-s; 已在该档则 < T+s 保持 */
    if(THR_ACTIVE(cfg->ppmin) && (p < cfg->ppmin - s || (lo == 2 && p < cfg->ppmin + s)))
        lo = 2;
    else if(THR_ACTIVE(cfg->pmin) && (p < cfg->pmin - s || (lo >= 1 && p < cfg->pmin + s)))
        lo = 1;
    else
        lo = 0;

    s_hi_level[ch] = hi;
    s_lo_level[ch] = lo;
}

// ==================== 阈值检查 ====================
// 优先级:
//   1. spupower.protect (VPROTECT, 仅报警; VLP模式由 main.c 门控)
//   2. vpthreshold ppmax/ppmin  (压力极限, 双边迟滞 ±shock)
//   3. vpthreshold pmax/pmin    (压力上下限, 双边迟滞 ±shock)
//   4. Tthreshold vhigh/vlow            (温度极限)
//   5. Tthreshold high/low              (温度上下限)
//   6. spupower high/low                (电压上下限)
//   7. vpthreshold mutation             (压力突变, 不受 shock 影响)
ThresholdResult_t threshold_check(float pressure, float temperature, float voltage, float last_pressure, uint8_t check_voltage, uint8_t skip_mutation)
{
    // ---- 1. 电压保护(VPROTECT) — spupower.enabled 关闭时跳过, 不触发报警/VLP ----
    // voltage >= 0 守卫: ADC 超时返回 -1.0, 不可参与电压判定, 否则误触发 VPROTECT
    // 进入滞回: 已在VLP时用 protect+0.15 判断(由do_collect处理退出), 未在VLP时用 protect-0.1 防振荡
    if(g_spupower.enabled && check_voltage && voltage >= 0.0f &&
       THR_ACTIVE(g_spupower.protect) && voltage < g_spupower.protect - 0.1f)
        return THRESHOLD_VPROTECT;

    // ---- 2/3. 压力检查 (双边迟滞: 进入 ±shock, 退出 ∓shock, 带内保持档位) ----
    press_hyst_update(0, pressure);
    if(g_pthreshold.enabled)
    {
        if(s_hi_level[0] == 2) return THRESHOLD_VERY_HIGH;
        if(s_lo_level[0] == 2) return THRESHOLD_VERY_LOW;
        if(s_hi_level[0] == 1) return THRESHOLD_OVER_LIMIT;
        if(s_lo_level[0] == 1) return THRESHOLD_UNDER_LIMIT;
    }

    // ---- 4/5. 温度检查 ----
    if(g_tthreshold.enabled)
    {
        if(THR_ACTIVE(g_tthreshold.vhigh) && temperature > g_tthreshold.vhigh)
            return THRESHOLD_TEMP_VERY_HIGH;
        if(THR_ACTIVE(g_tthreshold.vlow) && temperature < g_tthreshold.vlow)
            return THRESHOLD_TEMP_VERY_LOW;
        if(THR_ACTIVE(g_tthreshold.high) && temperature > g_tthreshold.high)
            return THRESHOLD_TEMP_HIGH;
        if(THR_ACTIVE(g_tthreshold.low) && temperature < g_tthreshold.low)
            return THRESHOLD_TEMP_LOW;
    }

    // ---- 6. 电压上下限 (spupower.enabled 关闭时跳过, 不触发报警)
    if(g_spupower.enabled && check_voltage && voltage >= 0.0f)
    {
        if(THR_ACTIVE(g_spupower.high) && voltage > g_spupower.high)
            return THRESHOLD_VOLT_HIGH;
        if(THR_ACTIVE(g_spupower.low) && voltage < g_spupower.low)
            return THRESHOLD_VOLT_LOW;
    }

    // ---- 7. 压力突变 (last_pressure < 0 表示首次采集,跳过; 不应用 shock) ----
    if(!skip_mutation && g_pthreshold.enabled && last_pressure >= 0)
    {
        float delta = pressure - last_pressure;
        if(delta < 0) delta = -delta;
        if(THR_ACTIVE(g_pthreshold.mutation) && delta > g_pthreshold.mutation)
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
    if((THR_ACTIVE(g_spupower.high) && voltage > g_spupower.high) ||
       (THR_ACTIVE(g_spupower.low)  && voltage < g_spupower.low) ||
       (THR_ACTIVE(g_spupower.protect) && voltage < g_spupower.protect))
        return 1;
    return 0;
}

// ==================== 压力状态查询 ====================
// switch=0 → PRESSURE_NORMAL; 静态边界使用双边迟滞(±shock, 带内保持档位)
PressureStatus_t threshold_get_pressure_status(float pressure, float last_pressure)
{
    press_hyst_update(0, pressure);

    if(!g_pthreshold.enabled)
        return PRESSURE_NORMAL;

    if(s_hi_level[0] == 2) return PRESSURE_VERYHIGH;
    if(s_lo_level[0] == 2) return PRESSURE_VERYLOW;
    if(s_hi_level[0] == 1) return PRESSURE_HIGH;
    if(s_lo_level[0] == 1) return PRESSURE_LOW;

    if(last_pressure >= 0)
    {
        float delta = pressure - last_pressure;
        if(delta < 0) delta = -delta;
        if(THR_ACTIVE(g_pthreshold.mutation) && delta > g_pthreshold.mutation)
            return PRESSURE_MUTATION;
    }
    return PRESSURE_NORMAL;
}

const char* threshold_pressure_status_str(PressureStatus_t status)
{
    switch(status)
    {
        case PRESSURE_LOW:       return "pressure-low";
        case PRESSURE_HIGH:      return "pressure-high";
        case PRESSURE_VERYLOW:   return "pressure-verylow";
        case PRESSURE_VERYHIGH:  return "pressure-veryhigh";
        case PRESSURE_MUTATION:  return "pressure-mutation";
        default:                 return "pressure-normal";
    }
}

// ==================== 温度状态查询 ====================
// switch=0 → TEMP_NORMAL; vhigh/vlow 优先于 high/low
TempStatus_t threshold_get_temp_status(float temperature)
{
    if(!g_tthreshold.enabled)
        return TEMP_NORMAL;

    if(THR_ACTIVE(g_tthreshold.vhigh) && temperature > g_tthreshold.vhigh)
        return TEMP_VERY_HIGH;
    if(THR_ACTIVE(g_tthreshold.vlow) && temperature < g_tthreshold.vlow)
        return TEMP_VERY_LOW;
    if(THR_ACTIVE(g_tthreshold.high) && temperature > g_tthreshold.high)
        return TEMP_HIGH;
    if(THR_ACTIVE(g_tthreshold.low) && temperature < g_tthreshold.low)
        return TEMP_LOW;
    return TEMP_NORMAL;
}

const char* threshold_temp_status_str(TempStatus_t status)
{
    switch(status)
    {
        case TEMP_HIGH:       return "temp-high";
        case TEMP_LOW:        return "temp-low";
        case TEMP_VERY_HIGH:  return "temp-very-high";
        case TEMP_VERY_LOW:   return "temp-very-low";
        default:              return "temp-normal";
    }
}

// ==================== 出气端(第二路)阈值检查 ====================
// 基于 g_pthreshold2/g_tthreshold2, 不含电压检查 (电压为全局共享)
ThresholdResult_t threshold_check2(float pressure, float temperature, float last_pressure, uint8_t skip_mutation)
{
    // ---- 压力检查 (双边迟滞: 进入 ±shock, 退出 ∓shock, 带内保持档位) ----
    press_hyst_update(1, pressure);
    if(g_pthreshold2.enabled)
    {
        if(s_hi_level[1] == 2) return THRESHOLD_VERY_HIGH;
        if(s_lo_level[1] == 2) return THRESHOLD_VERY_LOW;
        if(s_hi_level[1] == 1) return THRESHOLD_OVER_LIMIT;
        if(s_lo_level[1] == 1) return THRESHOLD_UNDER_LIMIT;
    }

    // ---- 温度检查 ----
    if(g_tthreshold2.enabled)
    {
        if(THR_ACTIVE(g_tthreshold2.vhigh) && temperature > g_tthreshold2.vhigh)
            return THRESHOLD_TEMP_VERY_HIGH;
        if(THR_ACTIVE(g_tthreshold2.vlow) && temperature < g_tthreshold2.vlow)
            return THRESHOLD_TEMP_VERY_LOW;
        if(THR_ACTIVE(g_tthreshold2.high) && temperature > g_tthreshold2.high)
            return THRESHOLD_TEMP_HIGH;
        if(THR_ACTIVE(g_tthreshold2.low) && temperature < g_tthreshold2.low)
            return THRESHOLD_TEMP_LOW;
    }

    // ---- 压力突变 (last_pressure < 0 表示首次采集,跳过) ----
    if(!skip_mutation && g_pthreshold2.enabled && last_pressure >= 0)
    {
        float delta = pressure - last_pressure;
        if(delta < 0) delta = -delta;
        if(THR_ACTIVE(g_pthreshold2.mutation) && delta > g_pthreshold2.mutation)
            return THRESHOLD_MUTATION;
    }

    return THRESHOLD_OK;
}

// ==================== 出气端(第二路)压力状态查询 ====================
// 基于 g_pthreshold2 配置, 逻辑同 threshold_get_pressure_status
PressureStatus_t threshold_get_pressure_status2(float pressure, float last_pressure)
{
    press_hyst_update(1, pressure);

    if(!g_pthreshold2.enabled)
        return PRESSURE_NORMAL;

    if(s_hi_level[1] == 2) return PRESSURE_VERYHIGH;
    if(s_lo_level[1] == 2) return PRESSURE_VERYLOW;
    if(s_hi_level[1] == 1) return PRESSURE_HIGH;
    if(s_lo_level[1] == 1) return PRESSURE_LOW;

    if(last_pressure >= 0)
    {
        float delta = pressure - last_pressure;
        if(delta < 0) delta = -delta;
        if(THR_ACTIVE(g_pthreshold2.mutation) && delta > g_pthreshold2.mutation)
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

    if(THR_ACTIVE(g_tthreshold2.vhigh) && temperature > g_tthreshold2.vhigh)
        return TEMP_VERY_HIGH;
    if(THR_ACTIVE(g_tthreshold2.vlow) && temperature < g_tthreshold2.vlow)
        return TEMP_VERY_LOW;
    if(THR_ACTIVE(g_tthreshold2.high) && temperature > g_tthreshold2.high)
        return TEMP_HIGH;
    if(THR_ACTIVE(g_tthreshold2.low) && temperature < g_tthreshold2.low)
        return TEMP_LOW;
    return TEMP_NORMAL;
}

/* EEPROM 加载/保存/解析逻辑已迁移到 eeprom_config.c
 * 心跳值统一使用 config_get_keepalive(); 阈值字段统一通过 g_pthreshold/g_tthreshold/g_spupower 访问
 */
