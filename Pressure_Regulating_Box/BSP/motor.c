#include "motor.h"
#include "adc.h"

#if MOTOR_ENABLED

static uint32_t timer_clk;
static uint32_t pwm_freq;
static uint16_t arr, psc;

/* 自动计算PSC和ARR */
static void pwm_compute(uint32_t freq)
{
    uint32_t clk = timer_clk;
    uint32_t prescaler = 0;
    uint32_t period;

    /* 非法输入保护: 频率为0或过高时回退到安全默认值 */
    if(freq == 0 || freq > 1000000) freq = 160000;

    for(prescaler = 0; prescaler < 65535; prescaler++) {
        period = clk / (freq * (prescaler + 1));

        if(period <= 65535 && period >= 100) {
            break;
        }

        /* 频率过高: period 随分频增大只会更小, 提前退出避免无效计算 */
        if(period < 100 && prescaler == 0) {
            break;
        }
    }

    /* Safety clamp: 防止 period 越界导致 arr 下溢/溢出 */
    if(period < 100 || period > 65535) {
        period = 100;
        prescaler = 0;
    }

    psc = prescaler;
    arr = period - 1;
}

/* 初始化PWM */
static void pwm_init(uint32_t freq, float duty_ch0, float duty_ch1)
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
static void pwm_set_freq(uint32_t freq)
{
    pwm_freq = freq;

    pwm_compute(freq);

    timer_prescaler_config(TIMER2, psc, TIMER_PSC_RELOAD_NOW);
    timer_autoreload_value_config(TIMER2, arr);
}

/* 修改占空比（0.0~1.0） */
static void pwm_set_duty(uint8_t ch, float duty)
{
    uint16_t pulse;

    /* 占空比越界钳制, 防止 pulse 溢出 */
    if(duty < 0.0f) duty = 0.0f;
    if(duty > 1.0f) duty = 1.0f;
    pulse = (uint16_t)(duty * (arr + 1));

    if(ch == 0) {
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_0, pulse);
    } else if(ch == 1) {
        timer_channel_output_pulse_value_config(TIMER2, TIMER_CH_1, pulse);
    }
}

/* ==================== 到位检测硬件初始化 (按板级方式分派) ==================== */
void motor_pos_init(void)
{
#if VALVE_POS_SENSE == VALVE_POS_SENSE_LIMIT
    motor_limit_init();
#else
    motor_current_init();
#endif
}

#if VALVE_POS_SENSE == VALVE_POS_SENSE_LIMIT
/* ==================== 限位开关 (旧板) ==================== */
void motor_limit_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);

    /* PA11: 限位1(开阀到位), PA12: 限位2(关阀到位), 内部上拉（低电平=到位） */
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_11 | GPIO_PIN_12);
}

/* PA11: K1-S 开阀到位（低电平=到位） */
uint8_t motor_limit1_reached(void)
{
    return (gpio_input_bit_get(GPIOA, GPIO_PIN_11) == RESET) ? 1 : 0;
}

/* PA12: K2-S 关阀到位（低电平=到位） */
uint8_t motor_limit2_reached(void)
{
    return (gpio_input_bit_get(GPIOA, GPIO_PIN_12) == RESET) ? 1 : 0;
}
#else
/* ==================== 堵转电流采样 (本板: PB0 = ADC_IN8, 1Ω 采样电阻) ==================== */
void motor_current_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);

    /* PB0 模拟输入: 由 adc_svc_read_raw() 在读取时切换通道 */
    gpio_mode_set(MOTOR_CS_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, MOTOR_CS_PIN);

    /* ADC 初始化兜底: 最小唤醒/按键快检路径可能未调用 adc_bat_init(),
     * 此处在采样前保证 ADC 已使能并校准 (adc_svc_init 幂等) */
    adc_svc_init();
}

uint16_t motor_current_raw(void)
{
    return adc_svc_read_raw(MOTOR_CS_ADC_CH);
}

uint32_t motor_current_ma(void)
{
    uint16_t raw = motor_current_raw();

    if(raw == ADC_SVC_RAW_INVALID)
        return 0xFFFFFFFFu;

    /* I(mA) = V(mV) / R(Ω) ; V(mV) = raw * VREF / 4095 */
    if(MOTOR_CS_R_SHUNT <= 0.0f)
        return 0xFFFFFFFFu;
    return (uint32_t)(((float)adc_svc_raw_to_mv(raw)) / MOTOR_CS_R_SHUNT);
}
#endif /* VALVE_POS_SENSE */

/* ==================== 电机初始化 ==================== */
void motor_init(uint8_t enabled)
{
    if(!enabled) return;

    motor_pos_init();

#if MOTOR_DRV_MODE == 1
    /* PWM 驱动: PA6/PA7 复用 TIMER2 CH0/CH1 */
    pwm_init(160000, 0.0f, 0.0f);
#else
    /* GPIO 直驱: PA6/PA7 推挽输出 */
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6 | GPIO_PIN_7);
#endif
}

/* ==================== 方向驱动(语义接口) ==================== */
void motor_drive_open(void)
{
    motor_sleep_enable();   /* M_SLP(PA5) 拉高: 使能电机驱动 */
#if MOTOR_DRV_MODE == 1
    pwm_set_duty(0, 0.0f);
    pwm_set_duty(1, 0.9f);
#else
    gpio_bit_reset(GPIOA, GPIO_PIN_6);
    gpio_bit_set(GPIOA, GPIO_PIN_7);
#endif
}

void motor_drive_close(void)
{
    motor_sleep_enable();   /* M_SLP(PA5) 拉高: 使能电机驱动 */
#if MOTOR_DRV_MODE == 1
    pwm_set_duty(0, 0.9f);
    pwm_set_duty(1, 0.0f);
#else
    gpio_bit_set(GPIOA, GPIO_PIN_6);
    gpio_bit_reset(GPIOA, GPIO_PIN_7);
#endif
}

void motor_stop(void)
{
#if MOTOR_DRV_MODE == 1
    pwm_set_duty(0, 0.0f);
    pwm_set_duty(1, 0.0f);
#else
    gpio_bit_reset(GPIOA, GPIO_PIN_6);
    gpio_bit_reset(GPIOA, GPIO_PIN_7);
#endif
    motor_sleep_disable();   /* 停止后 M_SLP 拉低 */
}

/* ==================== 电机使能 M_SLEEP (PA5) ==================== */
void motor_sleep_enable(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_mode_set(MOTOR_SLEEP_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, MOTOR_SLEEP_PIN);
    gpio_output_options_set(MOTOR_SLEEP_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, MOTOR_SLEEP_PIN);
    gpio_bit_set(MOTOR_SLEEP_PORT, MOTOR_SLEEP_PIN);
}

void motor_sleep_disable(void)
{
    gpio_bit_reset(MOTOR_SLEEP_PORT, MOTOR_SLEEP_PIN);
}

#endif /* MOTOR_ENABLED */
