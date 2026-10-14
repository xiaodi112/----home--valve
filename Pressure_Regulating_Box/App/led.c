/**
 * @file led.c
 * @brief 三路状态灯服务 (检修/阀关/换电池), 低电平点亮, 长亮显示
 *        另含"三灯同步提示"(按键有效反馈), 见文件末尾 led_notice_* 与 led.h
 *
 * 层次: 应用层状态指示. 只调用 BSP(output_led_*) 与既有状态源
 *       (valve_get_status / g_valve_pos_valid / g_valve_pos_state / g_spupower),
 *       不直接操作寄存器与引脚。
 */

#include "led.h"
#include "output.h"
#include "app_config.h"
#include "motor.h"        /* VALVE_POS_SENSE: 到位判定方式 */
#include "eeprom_config.h"
#include "main.h"
#include "valve.h"
#include "delay.h"
#include "tim.h"
#include <string.h>

/* 三灯同步提示状态 (当日秒, 跨天安全) */
static uint8_t  s_notice_active = 0;
static uint32_t s_notice_deadline = 0;

static uint32_t led_now_sec(void)
{
    uint32_t t = rtc_get_hhmmss();
    return (t / 10000u) * 3600u + ((t / 100u) % 100u) * 60u + (t % 100u);
}

void led_init(void)
{
    output_led_init();
}

void led_all_off(void)
{
    output_led_all_off();
}

void led_all_on(void)
{
    output_led_all_on();
}

void led_status_refresh(void)
{
    uint8_t check_on = 0;
    uint8_t valve_on = 0;
    uint8_t batt_on  = 0;

    /* 三灯同步提示期间, 三灯全亮是唯一显示内容: 必须在这里短路, 否则下面按状态
     * 算出的结果会把提示中的某几路重新熄灭(例如阀开时 LED2 本该是灭的)。 */
    if(s_notice_active)
    {
        led_all_on();
        return;
    }

#if MOTOR_ENABLED
    /* 检修灯: 阀门故障 / 位置未确认 */
    {
        const char *vs = valve_get_status();
#if VALVE_POS_SENSE == VALVE_POS_SENSE_STALL
        if(strcmp(vs, "valve-timeout") == 0)      /* 位置未确认(未回零/中途掉电/动作失败) */
            check_on = 1;
#else
        if(strcmp(vs, "valve-err") == 0)
            check_on = 1;
#endif
    }
#endif

    /* 阀关灯: 位置可信且物理位置为"关"。故障/位置未确认时以检修灯优先 (不点灯) */
    if(!check_on)
    {
        if(g_valve_pos_valid && g_valve_pos_state == 0)
            valve_on = 1;
    }

    /* 换电池灯: 电压低于低压门限 (门限为 0 表示未配置) */
    if(g_spupower.enabled && g_spupower.low > 0.0f &&
       g_sensor.voltage >= 0.0f && g_sensor.voltage < g_spupower.low)
        batt_on = 1;

    output_led_check_set(check_on);
    output_led_valve_set(valve_on);
    output_led_batt_set(batt_on);
}

void led_fail_hold(void)
{
    uint16_t i;

    led_all_on();
    for(i = 0; i < (LED_FAIL_HOLD_MS / 100u); i++)
    {
        trea_delay_ms(100);
        swdt_feed();          /* 5 秒提示期间保持喂狗 */
    }
    led_status_refresh();     /* 恢复"按状态长亮" */
}

/* ==================== 三灯同步提示 ("按键有效"反馈) ==================== */
void led_notice_show(void)
{
    /* 不判阀门状态: 按键反馈与阀门位置无关, 任何状态下都给同一视觉反馈 */
    s_notice_deadline = (led_now_sec() + (LED_NOTICE_MS / 1000u)) % 86400u;
    s_notice_active = 1;                  /* 提示期内再次短按: 重新计时 */
    led_all_on();                         /* 立即点亮三灯 */
}

uint8_t led_notice_active(void)
{
    return s_notice_active;
}

int32_t led_notice_left_sec(void)
{
    int32_t d = (int32_t)s_notice_deadline - (int32_t)led_now_sec();

    if(d >= 0)  return d;
    if(d >= -3600) return 0;              /* 短时已过期: 立即处理, 不跨天 */
    return d + 86400;
}

void led_notice_end(void)
{
    s_notice_active = 0;
    led_status_refresh();                 /* 回到"按状态长亮" */
}

void led_notice_expired_check(void)
{
    if(!s_notice_active)
        return;

    /* 只按截止时间结束: 提示期内阀门动作(位置变为不可信)不改变提示时长,
     * 否则"短按看到三灯 5 秒"会随阀门动作时有时无 */
    if(led_notice_left_sec() <= 0)
        led_notice_end();
}

uint8_t led_notice_alarm_hhmmss(uint32_t *out_hhmmss)
{
    if(!s_notice_active || out_hhmmss == NULL)
        return 0;

    *out_hhmmss = (s_notice_deadline / 3600u) * 10000u
                + ((s_notice_deadline / 60u) % 60u) * 100u
                + (s_notice_deadline % 60u);
    return 1;
}
