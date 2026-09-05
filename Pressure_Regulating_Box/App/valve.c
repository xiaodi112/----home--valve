#include "valve.h"
#include "motor.h"
#include "eeprom_config.h"
#include "delay.h"
#include "usart.h"
#include "buzzer.h"
#include <string.h>

// ==================== 阀门动作执行 (含valve_err检测) ====================
#if MOTOR_ENABLED
const char* valve_execute(const char *action, uint8_t update_last_order)
{
    uint32_t t;
    uint8_t target_limit, wrong_limit;

    if(strcmp(action, "open") == 0 || strcmp(action, "1") == 0)
    {
        printf_debug("[SM] Valve OPEN\r\n");
        motor_drive_open();
        t = trea_millis();
        while(!motor_limit1_reached() && (trea_millis() - t) < g_valve_timeout_ms);

        // 到位后保持原动作 (delay_switch=1 时), 用于压紧阀芯
        // 安全 clamp: 即使 g_vdelay.delay_ms 被外部误修改, 也不超过 VDLY_DELAY_MS_MAX
        if(motor_limit1_reached() && g_vdelay.enabled && g_vdelay.delay_ms > 0)
        {
            uint16_t hold_ms = g_vdelay.delay_ms > VDLY_DELAY_MS_MAX ? VDLY_DELAY_MS_MAX : g_vdelay.delay_ms;
            printf_debug("[SM] Valve OPEN limit reached, hold %dms\r\n", hold_ms);
            trea_delay_ms(hold_ms);
        }

        motor_stop();
        target_limit = motor_limit1_reached();
        wrong_limit  = motor_limit2_reached();

        // valve_err: 两个限位同时触发, 或目标未到位且反向限位也未到位(机械卡死)
        if(target_limit && wrong_limit)
            return "valve_err";
        if(target_limit)
        {
            if(update_last_order) { g_valve_last_order = 1; config_save_ext(); }
            buzzer_init();
            buzzer_beep(1, 500);   /* 开阀到位提示 */
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
        motor_drive_close();
        t = trea_millis();
        while(!motor_limit2_reached() && (trea_millis() - t) < g_valve_timeout_ms);

        // 到位后保持原动作 (delay_switch=1 时), 用于压紧阀芯
        // 安全 clamp 与 OPEN 分支一致
        if(motor_limit2_reached() && g_vdelay.enabled && g_vdelay.delay_ms > 0)
        {
            uint16_t hold_ms = g_vdelay.delay_ms > VDLY_DELAY_MS_MAX ? VDLY_DELAY_MS_MAX : g_vdelay.delay_ms;
            printf_debug("[SM] Valve CLOSE limit reached, hold %dms\r\n", hold_ms);
            trea_delay_ms(hold_ms);
        }

        motor_stop();
        target_limit = motor_limit2_reached();
        wrong_limit  = motor_limit1_reached();

        if(target_limit && wrong_limit)
            return "valve_err";
        if(target_limit)
        {
            if(update_last_order) { g_valve_last_order = 0; config_save_ext(); }
            buzzer_init();
            buzzer_beep(1, 500);   /* 关阀到位提示 */
            return "valve_off";
        }
        if(wrong_limit)
            return "valve_err";
        return "valve_timeout";
    }

    motor_stop();
    return "valve_err";
}

// ==================== 阀门实时状态查询 (不动作) ====================
const char* valve_get_status(void)
{
    uint8_t l1 = motor_limit1_reached();  // 开阀到位
    uint8_t l2 = motor_limit2_reached();  // 关阀到位

    if(l1 && l2)  return "valve_err";
    if(l1)        return "valve_on";
    if(l2)        return "valve_off";
    return "valve_timeout";
}

#else /* !MOTOR_ENABLED: 无电机硬件, 阀门动作不可用 */
const char* valve_execute(const char *action, uint8_t update_last_order)
{
    (void)action;
    (void)update_last_order;
    printf_debug("[VALVE] Motor not present (MOTOR_ENABLED=0), skip\r\n");
    return "valve_err";
}

const char* valve_get_status(void)
{
    return "valve_err";
}
#endif /* MOTOR_ENABLED */
