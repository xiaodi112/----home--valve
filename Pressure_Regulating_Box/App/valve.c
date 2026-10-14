/**
 * @file valve.c
 * @brief 阀门动作控制 (到位判定按板级方式二选一, 见 BSP/motor.h: VALVE_POS_SENSE)
 *
 * 堵转电流方案 (本板):
 *   电机无限位开关, 到位只能由"堵转电流"判断: 驱动方向由指令决定, 电流超过阈值
 *   并连续确认后即认为已到该方向终点。由此带来两条设计约束:
 *     1) 位置不可读, 只能靠 EEPROM 记忆 → 用"位置可信脏标志"(g_valve_pos_valid):
 *        动作前先置 0(中途掉电即知不可信), 到位后置 1; 不可信时先回零(关阀到底)。
 *     2) 中段机械卡阻与到位不可区分, 统一按到位处理。
 */

#include "valve.h"
#include "motor.h"
#include "adc.h"
#include "eeprom_config.h"
#include "config_store.h"
#include "delay.h"
#include "usart.h"
#include "buzzer.h"
#include <string.h>

#if MOTOR_ENABLED

#if VALVE_POS_SENSE == VALVE_POS_SENSE_STALL

/* ==================== 位置可信标志 (EEPROM 脏标志) ==================== */
static void valve_pos_mark_dirty(void)
{
    if(g_valve_pos_valid)
    {
        g_valve_pos_valid = 0;
        config_save_valve_pos_valid();   /* 单字节写: 动作前落盘, 掉电即知不可信 */
    }
}

/* 记录物理位置 (与"用户最后指令" g_valve_last_order 解耦):
 * 平台指令/定时阀/异常关阀/回零 —— 任何确认到位的动作都调用本函数。
 * 持久化由调用方统一用 config_save_ext() 完成 (一次写 pos_valid + pos_state)。 */
static void valve_pos_set(uint8_t open_state)
{
    g_valve_pos_state = open_state ? 1u : 0u;
    g_valve_pos_valid = 1;
}

/* ==================== 驱动到堵转 ====================
 * open_dir: 1=开阀方向, 0=关阀方向
 * 返回: 1=已确认到位(堵转), 0=超时未确认
 * 说明: 阈值 g_motor_cs_th == 0 表示未标定, 此时不做堵转判定(只驱动到超时)。 */
static uint8_t valve_drive_until_stall(uint8_t open_dir)
{
    uint32_t t0 = trea_millis();
    uint32_t timeout_ms;
    uint16_t over_cnt = 0;
    uint8_t  stall = 0;

    /* 阈值未标定: 无判据, 缩短驱动时限避免电机长时间堵转发热 */
    if(g_motor_cs_th == 0)
    {
        timeout_ms = MOTOR_CS_UNCAL_MAX_MS;
        printf_debug("[VALVE] WARNING: current threshold not calibrated, "
                     "stall detection OFF (max %ums)\r\n", MOTOR_CS_UNCAL_MAX_MS);
    }
    else
    {
        timeout_ms = g_valve_timeout_ms;
    }

    if(open_dir) motor_drive_open();
    else         motor_drive_close();

    while((trea_millis() - t0) < timeout_ms)
    {
        trea_delay_ms(MOTOR_CS_SAMPLE_MS);
        swdt_feed();                        /* 动作期间持续喂狗(最长 g_valve_timeout_ms) */

        if(g_motor_cs_th == 0)
            continue;                       /* 阈值未标定: 关闭堵转判定 */

        if((trea_millis() - t0) < MOTOR_CS_BLANK_MS)
            continue;                       /* 启动屏蔽期: 避开启动浪涌 */

        {
            uint16_t raw = motor_current_raw();

            if(raw != ADC_SVC_RAW_INVALID && raw >= g_motor_cs_th)
            {
                if(++over_cnt >= MOTOR_CS_CONFIRM_N)
                {
                    stall = 1;
                    printf_debug("[VALVE] stall detected: raw=%u (%lumA) th=%u at %lums\r\n",
                                 raw, (unsigned long)MOTOR_CS_RAW_TO_MA(raw),
                                 g_motor_cs_th, (unsigned long)(trea_millis() - t0));
                    break;
                }
            }
            else
            {
                over_cnt = 0;               /* 抗 PWM 纹波: 需连续超阈 */
            }
        }
    }

    /* 到位后压紧保持 (可选): 继续通电 hold_ms, 上限受 VDLY_DELAY_MS_MAX 约束 */
    if(stall && g_vdelay.enabled && g_vdelay.delay_ms > 0)
    {
        uint16_t hold = (g_vdelay.delay_ms > VDLY_DELAY_MS_MAX) ? VDLY_DELAY_MS_MAX
                                                               : g_vdelay.delay_ms;
        printf_debug("[VALVE] limit reached, hold %dms\r\n", hold);
        trea_delay_ms(hold);
        swdt_feed();
    }

    motor_stop();
    return stall;
}

/* ==================== 回零: 朝关阀方向驱动到堵转 ====================
 * 建立已知起点"已关到底"; 仅位置不可信时执行 (方案B)。返回 1=成功 */
uint8_t valve_home(void)
{
    printf_debug("[VALVE] homing (drive close until stall)...\r\n");

    if(g_motor_cs_th == 0)
    {
        printf_debug("[VALVE] homing skipped: current threshold not calibrated\r\n");
        return 0;
    }

    motor_pos_init();
    if(!valve_drive_until_stall(0))
    {
        printf_debug("[VALVE] homing FAILED (no stall detected)\r\n");
        return 0;
    }

    /* 回零属于"设备自动动作", 不改变用户的最后指令 g_valve_last_order,
     * 只记录物理位置(已关到底) */
    valve_pos_set(0);
    config_save_ext();                      /* 保存 pos_valid + pos_state */
    printf_debug("[VALVE] homed: valve closed, position valid\r\n");
    return 1;
}

/* ==================== 阀门动作执行 ==================== */
const char* valve_execute(const char *action, uint8_t update_last_order)
{
    uint8_t open_dir;
    uint8_t stall;

    if(strcmp(action, "open") == 0 || strcmp(action, "1") == 0)
        open_dir = 1;
    else if(strcmp(action, "close") == 0 || strcmp(action, "0") == 0)
        open_dir = 0;
    else
    {
        motor_stop();
        return "valve-err";
    }

    motor_pos_init();       /* 深休眠后 PB0 为 Analog, 驱动/采样前恢复 */

    /* 位置不可信(中途掉电/首次上电): 先回零建立已知起点 */
    if(!g_valve_pos_valid)
    {
        if(!valve_home())
        {
            printf_debug("[VALVE] abort: position unknown and homing failed\r\n");
            return "valve-err";
        }
    }

    /* 已在目标位置(比对物理位置): 不做无谓堵转, 但仍按需更新"用户最后指令" */
    if(g_valve_pos_state == (open_dir ? 1u : 0u))
    {
        printf_debug("[VALVE] already %s (pos_state=%d), skip drive\r\n",
                     open_dir ? "open" : "close", g_valve_pos_state);
        if(update_last_order && g_valve_last_order != (open_dir ? 1u : 0u))
        {
            g_valve_last_order = open_dir ? 1u : 0u;
            config_save_ext();
        }
        return open_dir ? "valve-on" : "valve-off";
    }

    printf_debug("[VALVE] %s driving (th=%u blank=%ums)\r\n",
                 open_dir ? "OPEN" : "CLOSE", g_motor_cs_th, MOTOR_CS_BLANK_MS);

    valve_pos_mark_dirty();                 /* 动作前落盘"位置不可信" */
    stall = valve_drive_until_stall(open_dir);

    if(stall)
    {
        valve_pos_set(open_dir);            /* 物理位置: 任何动作到位都记录 */
        if(update_last_order)
        {
            g_valve_last_order = open_dir ? 1u : 0u;   /* 仅"用户/平台指令"才改写最后指令 */
        }
        config_save_ext();                  /* 一次落盘: pos_valid + pos_state (+last_order) */
        buzzer_init();
        buzzer_beep(1, 500);                /* 到位提示 */
        return open_dir ? "valve-on" : "valve-off";
    }

    /* 全程未检测到堵转: 位置仍不可信, 不更新 last_order */
    printf_debug("[VALVE] %s NOT confirmed (no stall) -> timeout\r\n",
                 open_dir ? "OPEN" : "CLOSE");
    return "valve-timeout";
}

/* ==================== 阀门状态查询 (堵转方案: 返回持久化状态) ==================== */
const char* valve_get_status(void)
{
    if(!g_valve_pos_valid)
        return "valve-timeout";             /* 位置未确认(未回零/中途掉电) */
    return g_valve_pos_state ? "valve-on" : "valve-off";   /* 物理位置, 与用户指令无关 */
}

#else /* VALVE_POS_SENSE_STALL: 旧板双限位方案 (原实现原样保留) */

/* ==================== 阀门动作执行 (双限位: 含 valve-err 检测) ==================== */
const char* valve_execute(const char *action, uint8_t update_last_order)
{
    uint32_t t;
    uint8_t target_limit, wrong_limit;

    if(strcmp(action, "open") == 0 || strcmp(action, "1") == 0)
    {
        printf_debug("[SM] Valve OPEN\r\n");
        motor_drive_open();
        t = trea_millis();
        while(!motor_limit1_reached() && (trea_millis() - t) < g_valve_timeout_ms)
            swdt_feed();

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

        if(target_limit && wrong_limit)
            return "valve-err";
        if(target_limit)
        {
            if(update_last_order) { g_valve_last_order = 1; config_save_ext(); }
            buzzer_init();
            buzzer_beep(1, 500);   /* 开阀到位提示 */
            return "valve-on";
        }
        if(wrong_limit)
            return "valve-err";
        return "valve-timeout";
    }
    else if(strcmp(action, "close") == 0 || strcmp(action, "0") == 0)
    {
        printf_debug("[SM] Valve CLOSE\r\n");
        motor_drive_close();
        t = trea_millis();
        while(!motor_limit2_reached() && (trea_millis() - t) < g_valve_timeout_ms)
            swdt_feed();

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
            return "valve-err";
        if(target_limit)
        {
            if(update_last_order) { g_valve_last_order = 0; config_save_ext(); }
            buzzer_init();
            buzzer_beep(1, 500);   /* 关阀到位提示 */
            return "valve-off";
        }
        if(wrong_limit)
            return "valve-err";
        return "valve-timeout";
    }

    motor_stop();
    return "valve-err";
}

/* ==================== 阀门实时状态查询 (双限位) ==================== */
const char* valve_get_status(void)
{
    uint8_t l1 = motor_limit1_reached();  // 开阀到位
    uint8_t l2 = motor_limit2_reached();  // 关阀到位

    if(l1 && l2)  return "valve-err";
    if(l1)        return "valve-on";
    if(l2)        return "valve-off";
    return "valve-timeout";
}

/* 双限位方案无需回零 (位置可直接读取) */
uint8_t valve_home(void)
{
    return 1;
}

#endif /* VALVE_POS_SENSE */

#else /* !MOTOR_ENABLED: 无电机硬件, 阀门动作不可用 */

const char* valve_execute(const char *action, uint8_t update_last_order)
{
    (void)action;
    (void)update_last_order;
    printf_debug("[VALVE] Motor not present (MOTOR_ENABLED=0), skip\r\n");
    return "valve-err";
}

const char* valve_get_status(void)
{
    return "valve-err";
}

uint8_t valve_home(void)
{
    return 0;
}

#endif /* MOTOR_ENABLED */
