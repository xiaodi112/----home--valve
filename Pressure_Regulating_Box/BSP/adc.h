#ifndef ADC_H
#define ADC_H

#include "gd32l23x.h"
#include "gd32l23x_libopt.h"

/* ADC服务 */
#define ADC_SVC_VREF_MV          3000u
#define ADC_SVC_RAW_INVALID      0xFFFFu

/* 内部参考电压 VREFINT */
#define ADC_SVC_VREFINT_CHANNEL  ADC_CHANNEL_17
#define ADC_SVC_VREFINT_MV       1200u

/* 初始化 ADC + 校准 */
void adc_svc_init(void);

/* 读取指定 ADC 通道原始值 */
uint16_t adc_svc_read_raw(uint8_t channel);

/* 读取内部 VREFINT 原始值 */
uint16_t adc_svc_read_vrefint(void);

/* 原始值 → 毫伏，按固定3.0V参考计算 */
uint32_t adc_svc_raw_to_mv(uint16_t raw);

#endif /* ADC_H */