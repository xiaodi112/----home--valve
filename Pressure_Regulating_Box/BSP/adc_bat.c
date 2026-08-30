#include "adc_bat.h"

// ADC参考电压 3.0V, 12位分辨率
#define ADC_VREF        3.0f
#define ADC_RESOLUTION  4095.0f

// 分压比: R3=2M, R5=2M → V_BAT = V_ADC * (R3+R5)/R5 = V_ADC * 2
#define DIVIDER_RATIO   2.0f

void adc_bat_init(void)
{
    // 使能时钟
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_ADC);

    // ADC时钟 = APB2/6
    rcu_adc_clock_config(RCU_ADCCK_APB2_DIV6);

    // PC2 配置为模拟输入 (AD_BAT, ADC_CHANNEL_12)
    gpio_mode_set(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_2);

    // ADC复位 + 基本配置
    adc_deinit();
    adc_special_function_config(ADC_CONTINUOUS_MODE, DISABLE);
    adc_special_function_config(ADC_SCAN_MODE, DISABLE);
    adc_data_alignment_config(ADC_DATAALIGN_RIGHT);

    // 单通道: channel 12 (PC2=AD_BAT), 采样时间 239.5 cycles (稳定)
    adc_channel_length_config(ADC_ROUTINE_CHANNEL, 1);
    adc_routine_channel_config(0, ADC_CHANNEL_12, ADC_SAMPLETIME_239POINT5);

    // 软件触发
    adc_external_trigger_source_config(ADC_ROUTINE_CHANNEL, ADC_EXTTRIG_ROUTINE_NONE);
    adc_external_trigger_config(ADC_ROUTINE_CHANNEL, ENABLE);

    // 使能ADC + 校准
    adc_enable();
    adc_calibration_enable();
}

float adc_bat_read_voltage(void)
{
    uint16_t adc_raw;
    float v_adc, v_bat;
    volatile uint32_t timeout = 0;

    // 启动转换
    adc_software_trigger_enable(ADC_ROUTINE_CHANNEL);

    // 等待转换完成 (超时保护, 避免ADC硬件异常时死循环)
    while(!adc_flag_get(ADC_FLAG_EOC))
    {
        if(++timeout > 100000U)
        {
            return -1.0f;  // ADC转换超时, 返回错误值
        }
    }
    adc_flag_clear(ADC_FLAG_EOC);

    adc_raw = adc_routine_data_read();

    // V_ADC = raw * 3.0 / 4095
    v_adc = (float)adc_raw * ADC_VREF / ADC_RESOLUTION;

    // V_BAT = V_ADC * 2 (分压比)
    v_bat = v_adc * DIVIDER_RATIO;

    return v_bat;
}
