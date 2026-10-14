#include "adc_bat.h"
#include "adc.h"

/* ADC 12位分辨率 */
#define ADC_RESOLUTION     4095.0f

/* VREFINT典型值 */
#define ADC_VREFINT        1.20f

/* 分压比: R3=2M, R5=2M */
#define DIVIDER_RATIO      2.0f

/* PB11: 电压采集基准/使能脚 */
#define ADC_REF_PORT       GPIOB
#define ADC_REF_PIN        GPIO_PIN_11

/* PC2: AD_BAT = ADC_CHANNEL_12 */
#define ADC_BAT_PORT       GPIOC
#define ADC_BAT_PIN        GPIO_PIN_2


/* PB11配置为推挽输出并拉低，接通电池分压 */
static void adc_bat_ref_low(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);

    gpio_mode_set(ADC_REF_PORT,
                  GPIO_MODE_OUTPUT,
                  GPIO_PUPD_NONE,
                  ADC_REF_PIN);

    gpio_output_options_set(ADC_REF_PORT,
                            GPIO_OTYPE_PP,
                            GPIO_OSPEED_2MHZ,
                            ADC_REF_PIN);

    gpio_bit_reset(ADC_REF_PORT, ADC_REF_PIN);
}


/* 关闭电池分压 */
static void adc_bat_ref_disable(void)
{
    gpio_mode_set(ADC_REF_PORT,
                  GPIO_MODE_ANALOG,
                  GPIO_PUPD_NONE,
                  ADC_REF_PIN);
}


void adc_bat_init(void)
{
    /* 使能时钟 */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);

    /* 先拉低采集基准(PB11) */
    adc_bat_ref_low();

    /* PC2配置为模拟输入 */
    gpio_mode_set(GPIOC,
                  GPIO_MODE_ANALOG,
                  GPIO_PUPD_NONE,
                  ADC_BAT_PIN);

    /* ADC初始化/校准 */
    adc_svc_init();
}


float adc_bat_read_voltage(void)
{
    uint16_t adc_bat;
    uint16_t adc_vrefint;
    float v_bat;

    /* 确保采集基准已拉低 */
    adc_bat_ref_low();

    /* 读取AD_BAT */
    adc_bat = adc_svc_read_raw(ADC_CHANNEL_12);

    if(adc_bat == ADC_SVC_RAW_INVALID)
    {
        adc_bat_ref_disable();
        return -1.0f;
    }

    /* 读取内部VREFINT */
    adc_vrefint = adc_svc_read_vrefint();

    if(adc_vrefint == ADC_SVC_RAW_INVALID ||
       adc_vrefint == 0)
    {
        adc_bat_ref_disable();
        return -1.0f;
    }

    /*
     * V_BAT =
     * ADC_BAT / ADC_VREFINT × VREFINT × 分压比
     */
    v_bat = ((float)adc_bat / (float)adc_vrefint)
            * ADC_VREFINT
            * DIVIDER_RATIO;

    /* 关闭分压 */
    adc_bat_ref_disable();

    return v_bat;
}