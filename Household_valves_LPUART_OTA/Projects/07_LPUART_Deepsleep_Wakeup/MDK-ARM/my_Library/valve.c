#include "valve.h"
#include "motor.h"
#include "eeprom_config.h"
#include "delay.h"
#include "usart.h"
#include <string.h>

// ==================== 电机 GPIO 初始化 ====================
void motor_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOC);

    /* PC0: 电机使能，推挽输出，默认低（关闭） */
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_0);
    gpio_bit_reset(GPIOC, GPIO_PIN_0);

    /* PC1/PC2: 电机到位信号输入，内部上拉（低电平=到位） */
    gpio_mode_set(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_1 | GPIO_PIN_2);
}

// ==================== 限位开关读取 ====================
uint8_t motor_limit1_reached(void)
{
    return (gpio_input_bit_get(GPIOC, GPIO_PIN_1) == RESET) ? 1 : 0;
}

uint8_t motor_limit2_reached(void)
{
    return (gpio_input_bit_get(GPIOC, GPIO_PIN_2) == RESET) ? 1 : 0;
}

// ==================== 阀门动作执行 (含valve_err检测) ====================
const char* valve_execute(const char *action, uint8_t update_last_order)
{
    uint32_t t;
    uint8_t target_limit, wrong_limit;

    gpio_bit_set(GPIOC, GPIO_PIN_0);    // 电机使能 ON
    pwm_init(160000, 0.0f, 0.0f);

    if(strcmp(action, "open") == 0 || strcmp(action, "1") == 0)
    {
        printf_debug("[SM] Valve OPEN\r\n");
        pwm_set_duty(0, 0.0f);
        pwm_set_duty(1, 0.9f);
        t = trea_millis();
        while(!motor_limit1_reached() && (trea_millis() - t) < 20000);

        // 到位后保持原动作 (delay_switch=1 时), 用于压紧阀芯
        // 安全 clamp: 即使 g_vdelay.delay_ms 被外部误修改, 也不超过 VDLY_DELAY_MS_MAX
        if(motor_limit1_reached() && g_vdelay.enabled && g_vdelay.delay_ms > 0)
        {
            uint16_t hold_ms = g_vdelay.delay_ms > VDLY_DELAY_MS_MAX ? VDLY_DELAY_MS_MAX : g_vdelay.delay_ms;
            printf_debug("[SM] Valve OPEN limit reached, hold %dms\r\n", hold_ms);
            trea_delay_ms(hold_ms);
        }

        pwm_set_duty(0, 0.0f);
        pwm_set_duty(1, 0.0f);

        target_limit = motor_limit1_reached();
        wrong_limit  = motor_limit2_reached();

        gpio_bit_reset(GPIOC, GPIO_PIN_0);

        // valve_err: 两个限位同时触发, 或目标未到位且反向限位也未到位(机械卡死)
        if(target_limit && wrong_limit)
            return "valve_err";
        if(target_limit)
        {
            if(update_last_order) { g_valve_last_order = 1; config_save_ext(); }
            return "valve_on";
        }
        // 超时: 检查是否反向限位仍触发(机械故障)
        if(wrong_limit)
            return "valve_err";
        return "valve_timeout";
    }
    else if(strcmp(action, "close") == 0 || strcmp(action, "0") == 0)
    {
        printf_debug("[SM] Valve CLOSE\r\n");
        pwm_set_duty(0, 0.9f);
        pwm_set_duty(1, 0.0f);
        t = trea_millis();
        while(!motor_limit2_reached() && (trea_millis() - t) < 20000);

        // 到位后保持原动作 (delay_switch=1 时), 用于压紧阀芯
        // 安全 clamp 与 OPEN 分支一致
        if(motor_limit2_reached() && g_vdelay.enabled && g_vdelay.delay_ms > 0)
        {
            uint16_t hold_ms = g_vdelay.delay_ms > VDLY_DELAY_MS_MAX ? VDLY_DELAY_MS_MAX : g_vdelay.delay_ms;
            printf_debug("[SM] Valve CLOSE limit reached, hold %dms\r\n", hold_ms);
            trea_delay_ms(hold_ms);
        }

        pwm_set_duty(0, 0.0f);
        pwm_set_duty(1, 0.0f);

        target_limit = motor_limit2_reached();
        wrong_limit  = motor_limit1_reached();

        gpio_bit_reset(GPIOC, GPIO_PIN_0);

        if(target_limit && wrong_limit)
            return "valve_err";
        if(target_limit)
        {
            if(update_last_order) { g_valve_last_order = 0; config_save_ext(); }
            return "valve_off";
        }
        if(wrong_limit)
            return "valve_err";
        return "valve_timeout";
    }

    gpio_bit_reset(GPIOC, GPIO_PIN_0);
    return "valve_err";
}
