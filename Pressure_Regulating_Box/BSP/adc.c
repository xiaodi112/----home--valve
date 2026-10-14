/**
 * @file adc.c
 * @brief ADC服务: 多通道共用同一个ADC (电池电压 + 电机电流 + VREFINT)
 *
 * 只有 adc_svc_init() 会初始化/校准 ADC;
 * 之后每次读取只重配 routine channel。
 */

#include "adc.h"

#define ADC_SVC_TIMEOUT     100000u


void adc_svc_init(void)
{
    rcu_periph_clock_enable(RCU_ADC);

    /* ADC时钟 = APB2/6 */
    rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);

    adc_deinit();

    adc_special_function_config(ADC_CONTINUOUS_MODE, DISABLE);
    adc_special_function_config(ADC_SCAN_MODE, DISABLE);
    adc_data_alignment_config(ADC_DATAALIGN_RIGHT);

    /* 规则组长度固定1，通道在每次读取前设置 */
    adc_channel_length_config(ADC_ROUTINE_CHANNEL, 1);

    adc_routine_channel_config(0,
                               ADC_CHANNEL_12,
                               ADC_SAMPLETIME_239POINT5);

    /* 开启内部通道，包括VREFINT */
    adc_channel_16_to_19(ADC_INTERNAL_CHANNEL_SWITCH, ENABLE);

    /* 软件触发 */
    adc_external_trigger_source_config(
        ADC_ROUTINE_CHANNEL,
        ADC_EXTTRIG_ROUTINE_NONE
    );

    adc_external_trigger_config(
        ADC_ROUTINE_CHANNEL,
        ENABLE
    );

    adc_enable();
    adc_calibration_enable();
}


uint16_t adc_svc_read_raw(uint8_t channel)
{
    volatile uint32_t timeout = 0;

    /* 切换采样通道 */
    adc_routine_channel_config(
        0,
        channel,
        ADC_SAMPLETIME_239POINT5
    );

    adc_flag_clear(ADC_FLAG_EOC);

    adc_software_trigger_enable(ADC_ROUTINE_CHANNEL);

    while(!adc_flag_get(ADC_FLAG_EOC))
    {
        if(++timeout > ADC_SVC_TIMEOUT)
        {
            return ADC_SVC_RAW_INVALID;
        }
    }

    adc_flag_clear(ADC_FLAG_EOC);

    return adc_routine_data_read();
}


uint16_t adc_svc_read_vrefint(void)
{
    return adc_svc_read_raw(ADC_SVC_VREFINT_CHANNEL);
}


uint32_t adc_svc_raw_to_mv(uint16_t raw)
{
    if(raw == ADC_SVC_RAW_INVALID)
        return 0u;

    return ((uint32_t)raw * ADC_SVC_VREF_MV) / 4095u;
}