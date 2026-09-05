#include "adc_bat.h"

// ADC参考电压 3.0V, 12位分辨率
#define ADC_VREF        3.0f
#define ADC_RESOLUTION  4095.0f

// 分压比: R3=2M, R5=2M → V_BAT = V_ADC * (R3+R5)/R5 = V_ADC * 2
#define DIVIDER_RATIO   2.0f

/* PC3: 电压采集基准/使能脚. 采样前必须输出低(作为分压低端), 深睡可转Analog */
#define ADC_REF_PORT    GPIOC
#define ADC_REF_PIN     GPIO_PIN_3

/* PC3 配置为推挽输出并拉低. 每次采样前调用: 兼容最小唤醒路径(深睡后 PC3 为 Analog,
 * 不依赖 power_gpio_init 的调用顺序) */
static void adc_bat_ref_low(void)
{
    rcu_periph_clock_enable(RCU_GPIOC);
    gpio_mode_set(ADC_REF_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, ADC_REF_PIN);
    gpio_output_options_set(ADC_REF_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, ADC_REF_PIN);
    gpio_bit_reset(ADC_REF_PORT, ADC_REF_PIN);
}

void adc_bat_init(void)
{
    // 使能时钟
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_ADC);

    // 先拉低采集基准 (PC3), 再配置/使能 ADC
    adc_bat_ref_low();

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

    // 确保采集基准已拉低 (深睡唤醒的最小路径可能未调用 adc_bat_init)
    adc_bat_ref_low();

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
