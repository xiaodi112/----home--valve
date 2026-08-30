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

/* 异常关阀策略 */
AbnormalClose_t abnormal_from_str(const char *str);
const char* abnormal_to_str(AbnormalClose_t ac);
int abnormal_should_close_valve(AbnormalClose_t policy, ThresholdResult_t result);

/* 窗口/采集决策 */
int config_is_active_window(void);
uint8_t config_need_adc_sampling(void);

#endif /* CONFIG_POLICY_H */
