#ifndef CONFIG_POLICY_H
#define CONFIG_POLICY_H

#include <stdint.h>
#include "eeprom_config.h"

/* ==================== 配置策略/纯逻辑 (无EEPROM/串口依赖) ==================== */

/* dot pattern (24-hour bitmap) */
void window_dottime_clear(uint8_t *mask);
void window_dottime_set(uint8_t *mask, uint8_t hour);
uint8_t window_dottime_hit(const uint8_t *mask, uint8_t hour);
uint8_t window_dottime_first(const uint8_t *mask);
void window_dottime_from_str(const char *str, uint8_t *mask);
void window_dottime_to_str(const uint8_t *mask, char *buf, uint16_t buf_size);

/* 周期类型 */
CycleType_t cycle_type_from_str(const char *str);
const char* cycle_type_to_str(CycleType_t type);
uint32_t cycle_to_seconds(const CycleConfig_t *cfg);

/* 周期上限/越界钳制 (保证 minutes 域不溢出, 支持跨天周期) */
uint32_t cycle_max_once(CycleType_t type);   /* 单位对应的 once 上限(分钟域不溢出) */
uint8_t  cycle_clamp(CycleConfig_t *cfg);    /* 越界钳制; 返回 1=被修正过 */

/* ==================== 绝对时间轴 (跨天周期调度) ====================
 * 以 1999-12-27(周一) 为第 0 天, 使:
 *   1 day  → 每天 00:00
 *   N day  → 每 N 天 00:00 (按 epoch 整除对齐)
 *   1 week → 每周一 00:00
 * 供采集网格判定/采集闹钟/上报周期序号使用, 避免"当日秒取模"在 >1 天周期上退化。 */
uint32_t sched_day_index(uint8_t year, uint8_t month, uint8_t day);
uint32_t sched_abs_min_of(uint8_t year, uint8_t month, uint8_t day,
                          uint8_t hour, uint8_t minute);
uint32_t sched_abs_min(void);                /* 当前绝对分钟 */
uint32_t sched_abs_sec(void);                /* 当前绝对秒 */

/* 异常关阀策略 */
AbnormalClose_t abnormal_from_str(const char *str);
const char* abnormal_to_str(AbnormalClose_t ac);
int abnormal_should_close_valve(AbnormalClose_t policy, ThresholdResult_t result);

/* 窗口/采集决策 */
int config_is_active_window(void);
int config_is_active_window_at(uint32_t hhmmss);
uint8_t config_need_adc_sampling(void);

#endif /* CONFIG_POLICY_H */
