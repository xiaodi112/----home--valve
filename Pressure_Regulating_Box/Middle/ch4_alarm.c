/**
 * @file ch4_alarm.c
 * @brief CH4 alarm logic + independent state machine (do_ch4)
 * @details Triggered by gas_get downlink -> g_ch4_request -> STATE_CH4
 */

#include "ch4_alarm.h"
#include "eeprom_config.h"
#include "report.h"
#include "main.h"
#include "usart.h"
#include "ec800e.h"
#include "delay.h"
#include "mygpio.h"
#include "buzzer.h"
#include <stdio.h>
#include <string.h>

#if LASER_CH4_ENABLED

/* ==================== Alarm Check ==================== */
CH4_AlarmLevel_t ch4_check_alarm(const CH4_Data_t *data)
{
    /* 两状态报警: 浓度 >= 门限 → 1(报警), 否则 0(正常)
     * enabled 字段不再参与判断
     * (功能存在性由编译宏 LASER_CH4_ENABLED 决定) */
    if(!data->data_valid)
        return CH4_ALARM_NORMAL;

    float conc = data->concentration;
    return (conc >= g_ch4_threshold.threshold) ? CH4_ALARM_ALARM : CH4_ALARM_NORMAL;
}

/* ==================== Independent State Machine ==================== */
void do_ch4(void)
{
    static CH4_InternalState_t ch4_state = CH4_STATE_IDLE;
    static uint32_t warmup_start_ms = 0;
    static CH4_Data_t ch4_data;
    static uint8_t s_ch4_4g_ok = 0;   // 本次采集上报 4G 连接是否成功 (POWER_OFF 据此决定是否恢复QSCLK)

    switch(ch4_state)
    {
        case CH4_STATE_IDLE:
            /* gas_get/定时触发均无条件采集, 不再检查 enabled */
            ch4_state = CH4_STATE_POWER_ON;
            break;

        case CH4_STATE_POWER_ON:
            ch4_power_on();
            ch4_set_warmup_time((uint32_t)g_ch4_threshold.warmup_sec * 1000);
            warmup_start_ms = trea_millis();
            printf_debug("[CH4] power on, warmup %ds\r\n", g_ch4_threshold.warmup_sec);
            ch4_state = CH4_STATE_WARMING;
            break;

        case CH4_STATE_WARMING:
            swdt_enable(SWDT_TIMEOUT_MS);
            if((trea_millis() - warmup_start_ms) >= (uint32_t)g_ch4_threshold.warmup_sec * 1000) {
                printf_debug("[CH4] warmup done\r\n");
                ch4_state = CH4_STATE_COLLECT;
            }
            break;

        case CH4_STATE_COLLECT:
        {
            CH4_Status_t ret = ch4_collect(&ch4_data);
            if(ret == CH4_OK) {
                printf_debug("[CH4] collect OK: conc=%.2f temp=%.1f status=%02X\r\n",
                             ch4_data.concentration, ch4_data.temperature, ch4_data.status);
                ch4_state = CH4_STATE_REPORT;
            } else {
                printf_debug("[CH4] collect fail: %d\r\n", ret);
                ch4_state = CH4_STATE_POWER_OFF;
            }
            break;
        }

        case CH4_STATE_REPORT:
        {
            CH4_AlarmLevel_t level = ch4_check_alarm(&ch4_data);
            /* 激光甲烷异常自动关阀 (jgclose 策略): 报警即关阀, 保持关闭直到平台 valve_open */
            if(level == CH4_ALARM_ALARM)
                ch4_alarm_try_close_valve();
            /* 暖机/采集期间 4G 已进入 QSCLK 休眠, 上报前先恢复 4G LPUART 再唤醒并确认 MQTT 在线
             * (纯RTC最小唤醒进入CH4时, 4G串口未初始化, 必须先 com_lpuart_init 否则所有AT rx(0)) */
            com_lpuart_init();
            trea_delay_ms(100);
            gpio_init();   /* 最小唤醒后 PA0(DTR) 为 Analog, 恢复输出后才能拉低DTR唤醒4G */
            EC801E_Wakeup();
            if(EC801E_EnsureConnected() == EC_OK)
            {
                s_ch4_4g_ok = 1;
                report_publish_ch4_status(&ch4_data, level);
                trea_delay_ms(300);
                report_publish_feedback(32);   /* 激光甲烷状态获取成功 */
				if(level == CH4_ALARM_ALARM)
					// 报警提示音
					buzzer_beep(4, 100);
            }
            else
            {
                /* 4G 通信失败: 断电并标记失败, 避免"假在线"与无效重试 (与 do_upload 一致) */
                s_ch4_4g_ok = 0;
                EC801E_PowerOff();
                printf_debug("[CH4] report: 4G connect failed, power off\r\n");
            }
            printf_debug("[CH4] report: gas=%d\r\n", (int)level);
            ch4_state = CH4_STATE_POWER_OFF;
            break;
        }

        case CH4_STATE_POWER_OFF:
            ch4_power_off();
            if(s_ch4_4g_ok)
                EC801E_SetPSM(1);   /* long模式: 上报成功后恢复QSCLK轻休眠 */
            s_ch4_4g_ok = 0;
            printf_debug("[CH4] power off\r\n");
            ch4_state = CH4_STATE_IDLE;
            app_state = STATE_SLEEP;
            break;

        default:
            ch4_state = CH4_STATE_IDLE;
            app_state = STATE_SLEEP;
            break;
    }
}

#endif /* LASER_CH4_ENABLED */
