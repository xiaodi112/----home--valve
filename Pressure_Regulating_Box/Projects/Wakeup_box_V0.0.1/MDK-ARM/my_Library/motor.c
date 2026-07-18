#include "motor.h"

static uint32_t timer_clk;
static uint32_t pwm_freq;
static uint16_t arr, psc;

/* 自动计算PSC和ARR */
static void pwm_compute(uint32_t freq)
{
    uint32_t clk = timer_clk;
    uint32_t prescaler;
    uint32_t period;

    for(prescaler = 0; prescaler < 65535; prescaler++) {
        period = clk / (freq * (prescaler + 1));

        if(period <= 65535 && period >= 100) {
            break;
        }
    }

    // Safety clamp: prevent period=0 causing arr underflow
    if(period == 0) period = 100;

    psc = prescaler;
    arr = period - 1;
}

/* 初始化PWM */
void pwm_init(uint32_t freq, float duty_ch0, float duty_ch1)
{
    SystemCoreClockUpdate();
    timer_clk = SystemCoreClock;
    pwm_freq = freq;

    /* 时钟使能 */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_TIMER2);

    /* GPIO复用 */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_6 | GPIO_PIN_7);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6 | GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);

    /* 计算参数 */
    pwm_compute(freq);

    timer_deinit(TIMER2);

    timer_parameter_struct timer_initpara;

    timer_initpara.prescaler         = psc;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = arr;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;

    timer_init(TIMER2, &timer_initpara);

    /* PWM参数（通用定时器版本） */
    timer_oc_parameter_struct ocpara;

    ocpara.outputstate  = TIMER_CCX_ENABLE;
    ocpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;

    /* CH0 → PA6 */
    timer_channel_output_config(TIMER2, TIMER_CH_0, &ocpara);
    timer_channel_output_mode_config(TIMER2, TIMER_CH_0, TIMER_OC_MODE_PWM0);
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_0, duty_ch0 * (arr + 1));
    timer_channel_output_shadow_config(TIMER2, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    /* CH1 → PA7 */
    timer_channel_output_config(TIMER2, TIMER_CH_1, &ocpara);
    timer_channel_output_mode_config(TIMER2, TIMER_CH_1, TIMER_OC_MODE_PWM0);
    timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_1, duty_ch1 * (arr + 1));
    timer_channel_output_shadow_config(TIMER2, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);

    /* 启动定时器 */
    timer_enable(TIMER2);
}

/* 修改频率 */
void pwm_set_freq(uint32_t freq)
{
    pwm_freq = freq;

    pwm_compute(freq);

    timer_prescaler_config(TIMER2, psc, TIMER_PSC_RELOAD_NOW);
    timer_autoreload_value_config(TIMER2, arr);
}

/* 修改占空比（0.0~1.0） */
void pwm_set_duty(uint8_t ch, float duty)
{
    uint16_t pulse = duty * (arr + 1);

    if(ch == 0) {
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_0, pulse);
    } else if(ch == 1) {
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_1, pulse);
    }
}