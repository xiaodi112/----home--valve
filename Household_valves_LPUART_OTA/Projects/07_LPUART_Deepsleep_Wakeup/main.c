/**
 * @file main.c
 * @brief 家用阀门物联网控制器 — 状态机驱动低功耗主程序
 *
 * 架构: MQTT持久连接 + MCU深休眠 + LPUART唤醒接收下发
 *
 * 状态机 (7个状态):
 *   STATE_INIT     → 上电初始化+MQTT连接+订阅+首次上报 → STATE_SLEEP
 *   STATE_SLEEP    → 设闹钟+深休眠(模块QSCLK保持MQTT) → STATE_WAKEUP
 *   STATE_WAKEUP   → 恢复外设+判断唤醒源:
 *                     RTC闹钟 → STATE_COLLECT
 *                     LPUART  → STATE_DOWNLINK
 *   STATE_COLLECT  → 采集+阈值检查+存入缓冲区 → STATE_UPLOAD/ALARM/SLEEP
 *   STATE_UPLOAD   → 批量逐条上报+清空缓冲区 → STATE_SLEEP
 *   STATE_DOWNLINK → 读取+处理MQTT下发指令+立即上报结果 → STATE_SLEEP
 *   STATE_ALARM    → 5次重采确认+报警上报 → STATE_SLEEP
 */

#include "gd32l23x.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "systick.h"
#include "usart.h"
#include "delay.h"
#include "ec800e.h"
#include "mygpio.h"
#include "tim.h"
#include "PT_304_35.h"
#include "valve.h"
#include "downlink.h"
#include "app_config.h"
#include "eeprom_config.h"
#include "AT24c16.h"
#include "main.h"
#include "adc_bat.h"
#include "i2c_sensor.h"
#include "sensor_buffer.h"
#include "threshold.h"
#include "ota_app.h"
#include "slcd_seg.h"

// ==================== 状态定义 ====================
typedef enum {
    STATE_INIT = 0,
    STATE_SLEEP,
    STATE_WAKEUP,
    STATE_COLLECT,
    STATE_UPLOAD,
    STATE_DOWNLINK,
    STATE_ALARM
} AppState_t;

// ==================== 全局变量 ====================
SensorContext_t g_sensor = { .last_pressure_collect = -1.0f, .last_pressure_realtime = -1.0f, .prev_online = 1 };

static volatile uint16_t wakeup_count = 0;
static AppState_t app_state = STATE_INIT;
static uint8_t dot_uploaded_this_hour = 0;  // dot模式: 防止同一小时重复上报
// valve_alarm_closed / valve_pre_alarm_order 已移到 g_* 全局 (EEPROM 持久化, OTA 重启后可恢复), 见 eeprom_config.h
uint8_t voltage_low_power = 0;              // 低电压省电模式: 仅8:00/18:00上报
uint8_t fourG_user_off = 0;                 // 用户主动关机4G标志(阻止do_wakeup盲目恢复PD9)
static uint8_t vlp_uploaded_this_hour = 0;  // 低电压模式: 防止同一小时重复上报
ThresholdResult_t last_realtime_alarm = THRESHOLD_OK;          // 实时模式防重复报警
static uint8_t g_is_minimal_wakeup = 0;            // 标志：是否为纯周期传感器采集的最简唤醒状态
uint8_t g_need_adc_sampling = 1;                   // 标志：当前周期是否需要进行电池电压采样
static uint16_t last_grid_collect_min = 0xFFFF;    // 标志：上一次执行周期采集的分钟戳，防止在同一采集整点多次触发

// ==================== 模块共享 JSON 上报缓冲 ====================
// 多处 pressure_status / temp_status / rtu_have 上报顺序使用, 共用一组 RAM, 节省 ~1.7KB
// 每个 publish 块完整执行 (生成时间戳→snprintf→PublishRawJson) 后下一块再用, 不嵌套
static char s_status_json[256];   // ps_json / ts_json / rtu_json 公用 (最大 256)
static char s_status_ts[32];      // ps_ts / ts_ts / rtu_ts / rt_ts 公用

// ==================== 硬件恢复 ====================
static void sys_init(void)
{
    /* 重定位中断向量表到 App 区起址 (Bootloader 跳转后必须设置, Cortex-M23 支持 VTOR) */
    SCB->VTOR = OTA_APP_ADDR;

    systick_config();
    my_systick_config();
    timer1_delay_init();
    trea_uart_init();

    gpio_init();

    // PD9(4G电源) 立即恢复: gpio_init 后 ODR=0 即 PD9=LOW
    // 若 4G 唤醒前在线(long模式+窗口内), 必须在此处第一时间拉高, 否则 1s+ 延时会导致 4G 复位
    if(!voltage_low_power && !fourG_user_off &&
       strcmp(g_window.pattern, "long") == 0 &&
       (config_is_active_window() || g_window.end_result == 1))
        gpio_bit_set(GPIOD, GPIO_PIN_9);

    motor_init();
    IIC_Init();
	IIC_GPIO_Init1();
    adc_bat_init();
    // PD9 (4G电源) 状态由上方条件逻辑控制，此处不做覆盖
    // PC0 (电机使能) 由 valve_execute() 按需控制，此处不提前开启
    // PD4 (I2C 电子开关) 默认保持 LOW; 由各采集点按需 set/reset

    debug_rx_init();   // USART0 RX中断缓冲初始化
    trea_delay_ms(1000);
}

// ==================== 前向声明 ====================
static void ensure_full_init(void);

// ==================== 实时路径: 配置唤醒源+深休眠 ====================
static void realtime_enter_sleep(void)
{
    uint8_t lpuart_enabled = 0;

    gpio_bit_reset(GPIOD, GPIO_PIN_4);
    rtc_alarm_wakeup_init();
    rtc_set_next_alarm_sec(g_real_frequency);

    // 4G在线时使能LPUART唤醒 (接收平台下发)
    if(!voltage_low_power && !fourG_user_off &&
       strcmp(g_window.pattern, "long") == 0 &&
       (config_is_active_window() || g_window.end_result == 1))
    {
        com_lpuart_init();
        lpuart_wakeup_init();
        lpuart_enabled = 1;
    }

    button_exti_init();
    lpuart_dma_rx_disable();
    enter_deepsleep_mode(lpuart_enabled);
}

// ==================== 实时采集最小化唤醒路径 ====================
// 仅初始化 SysTick + I2C + PD4, 采集传感器, 判断异常
// 异常时立即重采确认, 确认后启4G上报; 正常则直接回睡
// ==================== ==================== ====================
static void do_realtime_check(void)
{
    ThresholdResult_t result;
    TempStatus_t ts;
    uint8_t sensor_err;
    uint8_t still_abnormal = 1;
    uint8_t i;

    // === 最小初始化 (时钟恢复) ===
    system_wakeup_reinit();

    // === 采集 (仅传感器, 不读ADC; PB5/IIC 由 CollectAll 内部管理) ===
    PT304D_CollectAll(&g_sensor.raw2, &g_sensor.raw);

    // === 传感器在线/离线状态变化检测 ===
    {
        uint8_t cur_online = (g_sensor.raw.err == PT304D_OK) ? 1 : 0;
        if(cur_online != g_sensor.prev_online)
        {
            g_sensor.prev_online = cur_online;
            const char *have   = (g_sensor.raw.err != PT304D_ERR_I2C) ? "1" : "0";

            ensure_full_init();
            trea_uart_init();
            {
                rtc_get_time_str(s_status_ts, sizeof(s_status_ts));
                printf_debug("[RT] %s SENSOR STATE CHANGED: %s (err=%d) freq=%ds\r\n",
                             s_status_ts, cur_online ? "ONLINE" : "OFFLINE", g_sensor.raw.err, g_real_frequency);
            }

            // 立即4G上报
            {
                int was_online = (gpio_input_bit_get(GPIOD, GPIO_PIN_9) == SET);
                com_lpuart_init();
                trea_delay_ms(100);
                if(!was_online)
                {
                    EC801E_PowerOn();
                    EC801E_InitAndConnect();
                    EC801E_SetPSM(0);
                    trea_delay_ms(300);
                    report_4g_power_state(1);
                }
                else
                    EC801E_Wakeup();

                if(EC801E_EnsureConnected() == EC_OK)
                {
                    rtc_get_time_str(s_status_ts, sizeof(s_status_ts));
                    snprintf(s_status_json, sizeof(s_status_json),
                        "[{\"id\":\"rtu_have\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
                        "{\"id\":\"rtu_status\",\"value\":\"1\",\"ts\":\"%s\",\"remark\":\"\"}]",
                        have, s_status_ts, s_status_ts);
                    EC801E_PublishRawJson(s_status_json);
                    printf_debug("[RT] Sensor reported: have=%s\r\n", have);
                }

                if(was_online)
                    EC801E_SetPSM(1);
                else
                {
                    report_4g_power_state(0);
                    EC801E_PowerOff();
                }
            }

            // 离线时跳过阈值检查, 直接回睡
            if(!cur_online)
            {
                g_sensor.last_pressure_realtime = g_sensor.raw.pressure;
                realtime_enter_sleep();
                return;
            }
        }
        else if(!cur_online)
        {
            // 持续离线, 不重复上报, 直接回睡
            realtime_enter_sleep();
            return;
        }
    }

    // === 三重异常检测 ===
    result     = threshold_check(g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.last_normal_voltage, g_sensor.last_pressure_realtime, 0);
    ts         = threshold_get_temp_status(g_sensor.raw.temperature);
    sensor_err = (g_sensor.raw.err != PT304D_OK);

    // === 正常 → 检查报警恢复 → 更新压力 → 直接回睡 ===
    if(result == THRESHOLD_OK && ts == TEMP_NORMAL && !sensor_err)
    {
        // 报警恢复: 之前自动关阀, 现传感器正常 → 恢复阀门到关阀前状态
        // (与 do_collect 末尾恢复逻辑一致, 实时模式无需等待下一采集周期)
        if(g_valve_alarm_closed && !voltage_low_power)
        {
            const char *restore_action = (g_valve_pre_alarm_order == 1) ? "open" : "close";
            int rt_was_online = (gpio_input_bit_get(GPIOD, GPIO_PIN_9) == SET);

            ensure_full_init();
            trea_uart_init();
            printf_debug("[RT] ALARM recovered, restoring valve to %s (pre=%d)\r\n",
                         restore_action, g_valve_pre_alarm_order);

            IIC_Init();    // EEPROM I2C (valve_execute 保存 valve_last_order)
            motor_init();  // 阀门 PWM+GPIO
            const char *vr = valve_execute(restore_action, 0);
            g_valve_alarm_closed = 0;
            config_save_ext();   // 持久化清除恢复标志

            // 上报阀门恢复
            com_lpuart_init();
            trea_delay_ms(100);
            gpio_init();    // 恢复 PA11(DTR) 为 OUTPUT (gpio_enter_low_power 将其置为 ANALOG)
            int rt_ec_ok;
            if(rt_was_online)
            {
                EC801E_Wakeup();
                rt_ec_ok = (EC801E_EnsureConnected() == EC_OK);
            }
            else
            {
                EC801E_PowerOn();
                rt_ec_ok = (EC801E_InitAndConnect() == EC_OK);
                if(rt_ec_ok)
                {
                    EC801E_SetPSM(0);
                    trea_delay_ms(300);
                    report_4g_power_state(1);
                }
            }
            if(rt_ec_ok)
            {
                EC801E_PublishResponse("valve_control", vr);
                char v_order[4];
                snprintf(v_order, sizeof(v_order), "%d", g_valve_last_order);
                EC801E_PublishResponse("valve_last_order", v_order);

                EC801E_PublishAlarm("normal", g_sensor.raw.pressure, g_sensor.last_normal_voltage);

                {
                    PressureStatus_t ps = threshold_get_pressure_status(g_sensor.raw.pressure, g_sensor.last_pressure_realtime);
                    const char *ps_str = threshold_pressure_status_str(ps);
                    rtc_get_time_str(s_status_ts, sizeof(s_status_ts));
                    snprintf(s_status_json, sizeof(s_status_json),
                        "[{\"id\":\"pressure_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]", ps_str, s_status_ts);
                    EC801E_PublishRawJson(s_status_json);
                }

                {
                    const char *ts_str = threshold_temp_status_str(ts);
                    rtc_get_time_str(s_status_ts, sizeof(s_status_ts));
                    snprintf(s_status_json, sizeof(s_status_json),
                        "[{\"id\":\"temp_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]", ts_str, s_status_ts);
                    EC801E_PublishRawJson(s_status_json);
                }
            }
            if(rt_was_online)
                EC801E_SetPSM(1);
            else
            {
                report_4g_power_state(0);
                EC801E_PowerOff();
            }
        }

        g_sensor.last_pressure_realtime = g_sensor.raw.pressure;
        if(last_realtime_alarm != THRESHOLD_OK)
            last_realtime_alarm = THRESHOLD_OK;   // 恢复正常, 清除防重复标记
        realtime_enter_sleep();
        return;
    }

    // === 异常 → 防重复: 同类异常不重复上报 ===
    if(result == last_realtime_alarm && result != THRESHOLD_OK)
    {
        g_sensor.last_pressure_realtime = g_sensor.raw.pressure;
        realtime_enter_sleep();
        return;
    }

    // === 异常 → 立即重采确认 (3次) ===
    for(i = 0; i < APP_ALARM_RECHECK_COUNT; i++)
    {
        trea_delay_ms(APP_ALARM_RECHECK_DELAY_MS);
        PT304D_CollectAll(&g_sensor.raw2, &g_sensor.raw);

        result     = threshold_check(g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.last_normal_voltage, g_sensor.last_pressure_realtime, 0);
        ts         = threshold_get_temp_status(g_sensor.raw.temperature);
        sensor_err = (g_sensor.raw.err != PT304D_OK);

        if(result == THRESHOLD_OK && ts == TEMP_NORMAL && !sensor_err)
        {
            still_abnormal = 0;
            break;
        }
    }

    if(!still_abnormal)
    {
        // 偶发抖动 → 回睡
        g_sensor.last_pressure_realtime = g_sensor.raw.pressure;
        realtime_enter_sleep();
        return;
    }

    // === 确认异常 → 记录防重复标记 ===
    last_realtime_alarm = result;

    // === 判断4G状态, 选择快速/冷启动 ===
    {
        int was_online = (gpio_input_bit_get(GPIOD, GPIO_PIN_9) == SET);
        const char *alarm_type = "unknown";

        // 判断报警类型 (同do_alarm逻辑)
        switch(result)
        {
            case THRESHOLD_VPROTECT:        alarm_type = "voltage_protect";    break;
            case THRESHOLD_VERY_LOW:        alarm_type = "pressure_verylow";   break;
            case THRESHOLD_VERY_HIGH:       alarm_type = "pressure_veryhigh";  break;
            case THRESHOLD_OVER_LIMIT:      alarm_type = "pressure_high";      break;
            case THRESHOLD_UNDER_LIMIT:     alarm_type = "pressure_low";       break;
            case THRESHOLD_MUTATION:        alarm_type = "pressure_mutation";  break;
            case THRESHOLD_TEMP_HIGH:       alarm_type = "temperature_high";   break;
            case THRESHOLD_TEMP_LOW:        alarm_type = "temperature_low";    break;
            case THRESHOLD_TEMP_VERY_HIGH:  alarm_type = "temperature_veryhigh"; break;
            case THRESHOLD_TEMP_VERY_LOW:   alarm_type = "temperature_verylow";  break;
            case THRESHOLD_VOLT_HIGH:       alarm_type = "voltage_high";       break;
            case THRESHOLD_VOLT_LOW:        alarm_type = "voltage_low";        break;
            default: break;
        }
        // 传感器错误特殊处理
        if(sensor_err && result == THRESHOLD_OK)
            alarm_type = "sensor_offline";

        // 恢复 PA11(DTR) 为 OUTPUT (gpio_enter_low_power 将其置为 ANALOG)
        gpio_init();
        // 初始化Debug串口用于日志
        trea_uart_init();
        {
            rtc_get_time_str(s_status_ts, sizeof(s_status_ts));
            printf_debug("[RT] %s ALARM: %s P=%.2f T=%.2f freq=%ds\r\n",
                         s_status_ts, alarm_type, g_sensor.raw.pressure, g_sensor.raw.temperature, g_real_frequency);
        }

        // 启动4G上报
        com_lpuart_init();
        trea_delay_ms(100);
        int ec_ok;
        if(was_online)
        {
            EC801E_Wakeup();
            ec_ok = (EC801E_EnsureConnected() == EC_OK);
        }
        else
        {
            EC801E_PowerOn();
            ec_ok = (EC801E_InitAndConnect() == EC_OK);
            if(ec_ok)
            {
                EC801E_SetPSM(0);
                trea_delay_ms(200);
                report_4g_power_state(1);
            }
        }

        if(ec_ok)
        {
            // 上报报警
            EC801E_PublishAlarm(alarm_type, g_sensor.raw.pressure, g_sensor.last_normal_voltage);

            // 上报压力状态
            {
                PressureStatus_t ps = threshold_get_pressure_status(g_sensor.raw.pressure, g_sensor.last_pressure_realtime);
                const char *ps_str = threshold_pressure_status_str(ps);
                rtc_get_time_str(s_status_ts, sizeof(s_status_ts));
                snprintf(s_status_json, sizeof(s_status_json),
                    "[{\"id\":\"pressure_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]", ps_str, s_status_ts);
                EC801E_PublishRawJson(s_status_json);
            }

            // 上报温度状态
            {
                const char *ts_str = threshold_temp_status_str(ts);
                rtc_get_time_str(s_status_ts, sizeof(s_status_ts));
                snprintf(s_status_json, sizeof(s_status_json),
                    "[{\"id\":\"temp_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]", ts_str, s_status_ts);
                EC801E_PublishRawJson(s_status_json);
            }

            // 传感器离线上报
            if(sensor_err)
            {
                const char *rt_have = (g_sensor.raw.err != PT304D_ERR_I2C) ? "1" : "0";

                rtc_get_time_str(s_status_ts, sizeof(s_status_ts));
                snprintf(s_status_json, sizeof(s_status_json),
                    "[{\"id\":\"rtu_have\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
                    "{\"id\":\"rtu_status\",\"value\":\"1\",\"ts\":\"%s\",\"remark\":\"\"}]",
                    rt_have, s_status_ts, s_status_ts);
                EC801E_PublishRawJson(s_status_json);
            }

            // 异常关阀策略 (同do_alarm)
            // 守卫: !g_valve_alarm_closed 避免二次/多次异常覆盖 pre_alarm_order, 防止恢复时阀门不还原
            {
                AbnormalClose_t policy = abnormal_from_str(g_abnormal_close);
                if(abnormal_should_close_valve(policy, result) && !g_valve_alarm_closed)
                {
                    g_valve_pre_alarm_order = g_valve_last_order;
                    printf_debug("[RT] Auto-close valve (policy=%s, pre=%d)\r\n",
                                 g_abnormal_close, g_valve_pre_alarm_order);
                    IIC_Init();    // EEPROM I2C (valve_execute保存valve_last_order)
                    motor_init();  // 阀门需要PWM+GPIO
                    const char *vr = valve_execute("close", 0);
                    g_valve_alarm_closed = 1;
                    config_save_ext();   // 持久化恢复标志
                    EC801E_PublishResponse("valve_control", vr);
                    {
                        char v_order[4];
                        snprintf(v_order, sizeof(v_order), "%d", g_valve_last_order);
                        EC801E_PublishResponse("valve_last_order", v_order);
                    }
                }
            }
        }

        // 恢复4G状态
        if(was_online)
            EC801E_SetPSM(1);
        else
        {
            report_4g_power_state(0);
            EC801E_PowerOff();
        }
    }

    // === 回睡 ===
    g_sensor.last_pressure_realtime = g_sensor.raw.pressure;
    realtime_enter_sleep();
}

// ==================== 进入休眠 ====================
static void do_sleep(void)
{
    int fourG_keep_on = 0;

    // 关闭外设降功耗
    gpio_bit_reset(GPIOC, GPIO_PIN_0);
    gpio_bit_reset(GPIOD, GPIO_PIN_4);

	
	gpio_bit_reset(GPIOA, GPIO_PIN_2);
	gpio_bit_reset(GPIOA, GPIO_PIN_3);
	gpio_bit_reset(GPIOC, GPIO_PIN_4);
	gpio_bit_reset(GPIOC, GPIO_PIN_5);
	gpio_bit_reset(GPIOB, GPIO_PIN_8);
	gpio_bit_reset(GPIOB, GPIO_PIN_9);
	gpio_bit_reset(GPIOB, GPIO_PIN_6);
	gpio_bit_reset(GPIOB, GPIO_PIN_7);
	
	gpio_bit_reset(GPIOA, GPIO_PIN_5);
	gpio_bit_reset(GPIOA, GPIO_PIN_6);
	gpio_bit_reset(GPIOA, GPIO_PIN_7);

    // 4G电源管理: 根据window pattern决定PD9状态
    // 低电压省电模式: 强制4G断电
    if(voltage_low_power)
    {
        gpio_bit_reset(GPIOD, GPIO_PIN_9);
    }
    else if(strcmp(g_window.pattern, "long") == 0)
    {
        // long模式: 窗口内或end_result=1时保持4G (用户主动关机除外)
        if(!fourG_user_off && (config_is_active_window() || g_window.end_result == 1))
            fourG_keep_on = 1;
        else
            gpio_bit_reset(GPIOD, GPIO_PIN_9);  // long+er=0+窗口外/user_off: 4G断电
    }
    else
    {
        // short/dot: 4G始终断电(由do_upload在上报完后已PowerOff)
        gpio_bit_reset(GPIOD, GPIO_PIN_9);
    }

    // 设置RTC闹钟（紧贴深休眠前，防止闹钟时间已过）
    rtc_alarm_wakeup_init();
    if(g_real_enabled)
        rtc_set_next_alarm_sec(g_real_frequency);
    else
        rtc_set_next_alarm(g_interval.collect_interval_min);

    // LPUART唤醒: 仅4G有电时才有意义(可接收下发URC)
    if(fourG_keep_on)
    {
        com_lpuart_init();
        lpuart_wakeup_init();
    }

    // PA12按键唤醒: 始终启用(用于手动开机4G)
    button_exti_init();

    // 深休眠前禁用DMA(深休眠期间DMA时钟被门控, 需清理状态)
    lpuart_dma_rx_disable();

    if(g_real_enabled)
        printf_debug("[SM] SLEEP %ds-RT (buf=%d 4G=%s)\r\n",
                     g_real_frequency, sensor_buf_count(),
                     fourG_keep_on ? "ON" : "OFF");
    else
        printf_debug("[SM] SLEEP %dmin (buf=%d 4G=%s)\r\n",
                     g_interval.collect_interval_min, sensor_buf_count(),
                     fourG_keep_on ? "ON" : "OFF");
    trea_delay_ms(20);

    enter_deepsleep_mode(fourG_keep_on);

    // 实时采集循环: RTC秒级唤醒 → 检查是否到采集网格 → 否则最小采集+回睡
    while(g_real_enabled && rtc_alarm_flag && !counter0 && !button_wakeup_flag)
    {
        // 最小时钟恢复 → 读RTC判断是否到达正常采集网格
        system_wakeup_reinit();
        rtc_register_sync_wait();

        {
            uint32_t now_hhmmss = rtc_get_hhmmss();
            uint16_t cur_min = (uint16_t)(now_hhmmss / 10000) * 60
                             + (uint16_t)((now_hhmmss / 100) % 100);
            uint8_t cur_sec = (uint8_t)(now_hhmmss % 100);

            // 到达正常采集网格 (整分钟±5s容差, 且当前分钟未执行过采集) → 退出循环, 走完整STATE_WAKEUP
            if(g_interval.collect_interval_min > 0 &&
               (cur_min % g_interval.collect_interval_min) == 0 &&
               cur_sec < 6 &&
               cur_min != last_grid_collect_min)
                break;
        }

        // 非采集网格 → 最小化实时采集 (内部: 采集→判断→[可能上报]→设闹钟→深休眠→唤醒后返回)
        do_realtime_check();
        // do_realtime_check 返回后, 新的唤醒标志已置位, while再次判断
    }

    // 唤醒后立即转到 WAKEUP 状态
    app_state = STATE_WAKEUP;
}

// ==================== 按键单按（短按）处理 ====================
// 当用户短按 PA12（小于4秒释放）时调用。打印配置、当前实时传感器数据，并连接 4G 平台完整上报当时的状态。
static void do_button_short_press(void)
{
    static char s_short_json[256];
    static char s_short_ts[32];

    printf_debug("\r\n==================================================\r\n");
    printf_debug("[SM] BUTTON SHORT PRESS: Starting manual report & config dump\r\n");

    // 1. 打印当前系统配置
    config_print_current();
    
    // 2. 立即采集当前实时传感器数据并打印
    printf_debug("[SM] Sampling live sensor data...\r\n");
    PT304D_CollectAll(&g_sensor.raw2, &g_sensor.raw);
    
    g_sensor.voltage = adc_bat_read_voltage();
    g_sensor.last_normal_voltage = g_sensor.voltage;
    
    printf_debug("--- Live Data ---\r\n");
    printf_debug("  Pressure:    %.2f Pa\r\n", g_sensor.raw.pressure);
    printf_debug("  Temperature: %.2f C\r\n", g_sensor.raw.temperature);
    printf_debug("  Voltage:     %.2f V\r\n", g_sensor.voltage);
    printf_debug("  Sensor Err:  %d\r\n", g_sensor.raw.err);
    printf_debug("==================================================\r\n\r\n");

    // 3. 4G 连接检查与自适应快速上报
    int was_online = (gpio_input_bit_get(GPIOD, GPIO_PIN_9) == SET);
    
    com_lpuart_init();
    trea_delay_ms(100);

    if(was_online)
    {
        printf_debug("[SM] 4G module already powered ON, checking MQTT connection...\r\n");
        // 如果原本已经通电，我们只需要快速确保 MQTT 已经就绪
        if(EC801E_EnsureConnected() != EC_OK)
        {
            printf_debug("[SM] 4G was ON but MQTT offline, re-initializing connection...\r\n");
            EC801E_InitAndConnect();
        }
    }
    else
    {
        printf_debug("[SM] 4G module is OFF, performing full cold power-on & connection...\r\n");
        fourG_user_off = 0;                  // 清除用户手动关机标志以允许开机
        EC801E_PowerOn();
        EC801E_InitAndConnect();
    }

    if(EC801E_EnsureConnected() == EC_OK)
    {
        rtc_get_time_str(s_short_ts, sizeof(s_short_ts));
        
        // A. 上报 4G 开机状态
        report_4g_power_state(1);
        trea_delay_ms(700);

        // B. 上报传感器数据
        EC801E_PublishSensorData(&g_sensor.raw, g_sensor.voltage);
        trea_delay_ms(700);

        // C. 上报传感器在线状态 (rtu_have, rtu_status)
        const char *have    = (g_sensor.raw.err != PT304D_ERR_I2C) ? "1" : "0";
        const char *rstatus = (g_sensor.raw.err == PT304D_OK) ? "1" : "0";
        snprintf(s_short_json, sizeof(s_short_json),
            "[{\"id\":\"rtu_have\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"rtu_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]",
            have, s_short_ts, rstatus, s_short_ts);
        EC801E_PublishRawJson(s_short_json);
        trea_delay_ms(700);

        // D. 上报压力状态
        PressureStatus_t ps = threshold_get_pressure_status(g_sensor.raw.pressure, g_sensor.last_pressure_collect);
        const char *ps_str = threshold_pressure_status_str(ps);
        snprintf(s_short_json, sizeof(s_short_json),
            "[{\"id\":\"pressure_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]", ps_str, s_short_ts);
        EC801E_PublishRawJson(s_short_json);
        trea_delay_ms(700);

        // E. 上报温度状态
        TempStatus_t t_s = threshold_get_temp_status(g_sensor.raw.temperature);
        const char *t_str = threshold_temp_status_str(t_s);
        snprintf(s_short_json, sizeof(s_short_json),
            "[{\"id\":\"temp_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]", t_str, s_short_ts);
        EC801E_PublishRawJson(s_short_json);
        trea_delay_ms(700);

        // F. 上报最后阀门指令
        char v_order[4];
        snprintf(v_order, sizeof(v_order), "%d", g_valve_last_order);
        EC801E_PublishResponse("valve_last_order", v_order);
        trea_delay_ms(700);

        printf_debug("[SM] BUTTON SHORT PRESS: Manual status reporting success!\r\n");
    }
    else
    {
        printf_debug("[SM] BUTTON SHORT PRESS: manual report FAILED (MQTT connection failed)\r\n");
    }

    // 4. 等待按键松手，防止释放抖动再次触发
    while(button_is_pressed())
        trea_delay_ms(100);
    trea_delay_ms(50);

    // 5. 收尾：等待 5s 接收平台下行指令
    printf_debug("[SM] BUTTON: waiting 5s for downlink...\r\n");
    trea_delay_ms(5000);
    {
        uint8_t dl_count = drain_pending_downlink();
        if(dl_count > 0)
            printf_debug("[SM] BUTTON: processed %d downlink msg(s)\r\n", dl_count);
        else
            printf_debug("[SM] BUTTON: no downlink within 5s\r\n");
    }

    // 6. 按照原本状态关机回睡
    if(was_online)
    {
        EC801E_SetPSM(1);
        printf_debug("[SM] BUTTON: keep 4G online and enable PSM (was_online=1)\r\n");
    }
    else
    {
        report_4g_power_state(0);
        EC801E_PowerOff();
        fourG_user_off = 1; // 恢复手动关机状态
        printf_debug("[SM] BUTTON: restored 4G to powered off state (was_online=0)\r\n");
    }
}

// ==================== 唤醒恢复 ====================
static void do_wakeup(void)
{
    // 判断是否为纯周期采集定时唤醒 (非按键且非LPUART活动)
    if(button_wakeup_flag == 0 && counter0 == 0)
    {
        g_is_minimal_wakeup = 1;
        g_need_adc_sampling = config_need_adc_sampling();
        
        system_wakeup_reinit();
        
        // 极简采集初始化: Systick、Debug 串口发送; PB5(SW_IIC)/IIC 引脚由 PT304D_CollectAll 内部初始化
        systick_config();
        my_systick_config();
        trea_uart_init();
        
        if(g_need_adc_sampling)
        {
            adc_bat_init();
        }
        
        button_exti_disable();
        
        rtc_show_time();
        printf_debug("[SM] PURE RTC WAKEUP: minimal init (ADC=%d)\r\n", g_need_adc_sampling);
    }
    else
    {
        g_is_minimal_wakeup = 0;
        g_need_adc_sampling = 1;
        
        system_wakeup_reinit();
        sys_init();
        
        // 禁用按键EXTI(唤醒后立即禁止抨动重复触发)
        button_exti_disable();
        
        // PD9(4G电源)恢复: gpio_init默认PD9=LOW
        // 低电压省电模式: 不恢复4G
        // 用户主动关机: 不恢复4G(等待按键重新开机)
        // long模式: 窗口内或end_result=1时恢复4G供电
        if(!voltage_low_power && !fourG_user_off &&
           strcmp(g_window.pattern, "long") == 0 &&
           (config_is_active_window() || g_window.end_result == 1))
            gpio_bit_set(GPIOD, GPIO_PIN_9);
        // 其余情况(VLP/user_off/short/dot/long+er=0+窗口外): PD9保持LOW
        
        rtc_show_time();
        printf_debug("\r\n");
    }

    // === 按键唤醒: 4秒长按确认后开机4G ===
    if(button_wakeup_flag)
    {
        printf_debug("[SM] WAKEUP: BUTTON\r\n");
        button_wakeup_flag = 0;

        // 消抖延时: 等20ms等机械前沿抖动结束 (20ms更为灵敏，契合快速单击)
        trea_delay_ms(20);

        // 确认当前按键状态
        if(!button_is_pressed())
        {
            // 唤醒了但20ms内已松开，说明是极速短按，而非 false trigger!
            printf_debug("[SM] BUTTON: released fast (<20ms), short press detected\r\n");
            do_button_short_press();
            app_state = STATE_SLEEP;
            return;
        }

        // 4秒长按确认: 每100ms检查一次, 共40次
        // 不论 PD9 当前状态, 长按 4s 都将触发完整的上电+连接平台流程
        {
            uint8_t i;
            for(i = 0; i < 40; i++)
            {
                trea_delay_ms(100);
                if(!button_is_pressed())
                {
                    printf_debug("[SM] BUTTON: released at %dms, short press detected\r\n", (i + 1) * 100 + 20);
                    do_button_short_press();
                    app_state = STATE_SLEEP;
                    return;
                }
            }
        }

        // 确认长按→PD9 拉高 + 4G 开机 + 连接平台
        printf_debug("[SM] BUTTON: 4s confirmed, powering on 4G\r\n");
        fourG_user_off = 0;  // 清除用户关机标志
        com_lpuart_init();
        trea_delay_ms(100);
        EC801E_PowerOn();         // PD9 拉高 (idempotent, 已 HIGH 时不影响)
        EC801E_InitAndConnect();

        // 验证4G模块AT响应 + 上报开机状态
        if(EC801E_EnsureConnected() == EC_OK)
        {
            report_4g_power_state(1);
            printf_debug("[SM] BUTTON: 4G power on OK, reported\r\n");
        }
        else
        {
            printf_debug("[SM] BUTTON: 4G power on FAILED (no AT response)\r\n");
        }

        // 等待松手, 防止释放抖动触发下次唤醒
        while(button_is_pressed())
            trea_delay_ms(100);
        trea_delay_ms(50);

        // === 统一等待 20s 接收下行指令 (所有模式) ===
        printf_debug("[SM] BUTTON: %s mode, waiting 20s for downlink...\r\n",
                     g_window.pattern);
        trea_delay_ms(20000);
        {
            uint8_t dl_count = drain_pending_downlink();
            if(dl_count > 0)
                printf_debug("[SM] BUTTON: processed %d downlink msg(s)\r\n", dl_count);
            else
                printf_debug("[SM] BUTTON: no downlink within 20s\r\n");
        }

        // === 模式相关收尾 ===
        if(strcmp(g_window.pattern, "long") == 0)
        {
            // long模式: 保持4G在线 (do_sleep 内 fourG_keep_on 路径接管)
            EC801E_SetPSM(1);
            printf_debug("[SM] BUTTON: long mode, keep 4G online (PSM)\r\n");
        }
        else
        {
            // short/dot模式: 断电回睡
            report_4g_power_state(0);
            EC801E_PowerOff();
            printf_debug("[SM] BUTTON: %s mode, 4G powered off\r\n", g_window.pattern);
        }

        app_state = STATE_SLEEP;
        return;
    }

    // debug配置已由主循环统一处理

    if(counter0)
    {
        printf_debug("[SM] WAKEUP: LPUART (downlink)\r\n");
        app_state = STATE_DOWNLINK;
    }
    else if(rtc_alarm_flag)
    {
        printf_debug("[SM] WAKEUP: RTC\r\n");
        app_state = STATE_COLLECT;
    }
    else
    {
        printf_debug("[SM] WAKEUP: unknown\r\n");
        app_state = STATE_COLLECT;
    }
}

// ==================== 确保完整初始化 ====================
static void ensure_full_init(void)
{
    if(g_is_minimal_wakeup)
    {
        printf_debug("[SM] Escalating to FULL INIT (need 4G/Motor/EEPROM)\r\n");
        sys_init();
        g_is_minimal_wakeup = 0;
        g_need_adc_sampling = 1;
    }
}

// ==================== 采集传感器 + 阈值检查 ====================
static void do_collect(void)
{
    SensorRecord_t record;
    ThresholdResult_t result;
    uint8_t vlp_just_exited = 0;

    // 记录本次正常周期采集的分钟戳，防止秒级高频实时监测循环在同一分钟内多次唤醒采集
    {
        uint32_t now_hhmmss = rtc_get_hhmmss();
        last_grid_collect_min = (uint16_t)(now_hhmmss / 10000) * 60
                              + (uint16_t)((now_hhmmss / 100) % 100);
    }

    PT304D_CollectAll(&g_sensor.raw2, &g_sensor.raw);
    
    if(g_need_adc_sampling)
    {
        g_sensor.voltage = adc_bat_read_voltage();
        g_sensor.last_normal_voltage = g_sensor.voltage;
    }
    else
    {
        g_sensor.voltage = g_sensor.last_normal_voltage;
    }

    printf_debug("[SM] COLLECT (buf=%d) V=%.2f P=%.2f T=%.2f err=%d (ADC=%d)\r\n",
                 sensor_buf_count() + 1,
                 g_sensor.voltage, g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.raw.err, g_need_adc_sampling);

    // ========== 传感器状态变化检测 (状态改变时立即上报) ==========
    {
        uint8_t cur_online = (g_sensor.raw.err == PT304D_OK) ? 1 : 0;

        if(cur_online != g_sensor.prev_online)
        {
            // 状态发生变化 → 立即上报 (需要完整系统硬件支持)
            ensure_full_init();
            g_sensor.prev_online = cur_online;

            // rtu_have: I2C是否有应答 (0=没有传感器, 1=有传感器)
            // rtu_status: I2C 采集是否成功 (PT304D_OK=1, 其余=0)
            const char *have   = (g_sensor.raw.err != PT304D_ERR_I2C) ? "1" : "0";
            const char *status = (g_sensor.raw.err == PT304D_OK) ? "1" : "0";

            printf_debug("[SM] SENSOR STATE CHANGED: %s (err=%d)\r\n",
                         cur_online ? "ONLINE" : "OFFLINE", g_sensor.raw.err);

            // 立即4G上报
            {
                int sensor_coldstart = 0;
                com_lpuart_init();
                trea_delay_ms(100);

                if(voltage_low_power ||
                   strcmp(g_window.pattern, "short") == 0 || strcmp(g_window.pattern, "dot") == 0)
                {
                    sensor_coldstart = 1;
                    EC801E_PowerOn();
                    EC801E_InitAndConnect();
                    EC801E_SetPSM(0);
                    trea_delay_ms(300);
                    report_4g_power_state(1);
                }

                if(EC801E_EnsureConnected() == EC_OK)
                {
                    rtc_get_time_str(s_status_ts, sizeof(s_status_ts));
                    snprintf(s_status_json, sizeof(s_status_json),
                        "[{\"id\":\"rtu_have\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
                        "{\"id\":\"rtu_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]",
                        have, s_status_ts, status, s_status_ts);
                    EC801E_PublishRawJson(s_status_json);
                    printf_debug("[SM] Sensor reported: have=%s status=%s\r\n", have, status);
                }

                if(sensor_coldstart)
                {
                    report_4g_power_state(0);
                    EC801E_PowerOff();
                }
                else
                    EC801E_SetPSM(1);
            }

            // 离线时跳过阈值检查和数据缓存, 直接回休眠
            if(!cur_online)
            {
                app_state = STATE_SLEEP;
                return;
            }
        }
        else if(!cur_online)
        {
            // 状态未变但仍离线 → 不重复上报, 跳过采集直接回休眠
            printf_debug("[SM] SENSOR still OFFLINE, skip\r\n");
            app_state = STATE_SLEEP;
            return;
        }
    }

    // 阈值检查
    result = threshold_check(g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage, g_sensor.last_pressure_collect, g_need_adc_sampling);

    if(result != THRESHOLD_OK)
    {
        // 低电压省电模式: 已进入过VPROTECT报警, 后续VPROTECT不再重复报警
        if(voltage_low_power && result == THRESHOLD_VPROTECT)
        {
            printf_debug("[SM] VLP: still low voltage, skip alarm\r\n");
            // 不进入ALARM, 继续走低电压省电调度
        }
        else
        {
            // 首次VPROTECT会进入ALARM(关阀+报警), 同时标记低电压模式
            if(result == THRESHOLD_VPROTECT && !voltage_low_power)
            {
                voltage_low_power = 1;
                printf_debug("[SM] VLP: entering low voltage power-save mode\r\n");
            }
            printf_debug("[SM] THRESHOLD ABNORMAL (%d), entering ALARM\r\n", result);
            ensure_full_init();
            app_state = STATE_ALARM;
            return;
        }
    }

    // 低电压恢复: 以下任一条件触发退出 VLP
    //   (a) 电压回到 vprotect 以上 (正常恢复)
    //   (b) spupower.switch 被用户关闭 (不再执行电压检测, 不能残留 VLP 状态锁死)
    if(voltage_low_power && (!g_spupower.enabled || g_sensor.voltage >= g_spupower.protect + 0.15f))
    {
        voltage_low_power = 0;
        vlp_uploaded_this_hour = 0;
        vlp_just_exited = 1;     // 标记: 本次 collect 内 VLP 已退出, 需补 normal 上报
        if(!g_spupower.enabled)
            printf_debug("[SM] VLP: spupower disabled by user, exiting low power mode\r\n");
        else
            printf_debug("[SM] VLP: voltage recovered (%.2fV >= %.2fV), exiting low power mode\r\n",
                         g_sensor.voltage, g_spupower.protect);
    }

    // 恢复正常上报: (a) 之前报警自动关阀 → 还原阀门 + alarm:normal
    //               (b) VLP 刚退出 → alarm:normal (即使没触发关阀, 也通知平台脱离低压保护)
    // 共享 4G 启停, 避免重复 cold-start
    if((g_valve_alarm_closed || vlp_just_exited) && !voltage_low_power)
    {
        ensure_full_init();
        const char *restore_action = NULL;
        const char *vr = NULL;

        // (a) 阀门恢复 (仅当之前自动关阀时执行)
        if(g_valve_alarm_closed)
        {
            restore_action = (g_valve_pre_alarm_order == 1) ? "open" : "close";
            printf_debug("[SM] ALARM recovered, restoring valve to %s (pre=%d)\r\n",
                         restore_action, g_valve_pre_alarm_order);
            vr = valve_execute(restore_action, 0);
            g_valve_alarm_closed = 0;
            config_save_ext();   // 持久化: 清除恢复标志
        }
        else if(vlp_just_exited)
        {
            printf_debug("[SM] VLP recovered, publishing normal status\r\n");
        }

        // 4G 启停 + 对称恢复上报 (alarm:normal + pressure_status + temp_status)
        {
            int recovery_coldstart = 0;
            com_lpuart_init();
            trea_delay_ms(100);
            if(strcmp(g_window.pattern, "short") == 0 || strcmp(g_window.pattern, "dot") == 0)
            {
                recovery_coldstart = 1;
                EC801E_PowerOn();
                EC801E_InitAndConnect();
                EC801E_SetPSM(0);
                trea_delay_ms(300);
                report_4g_power_state(1);
            }
            if(EC801E_EnsureConnected() == EC_OK)
            {
                // 阀门字段 (条件)
                if(vr != NULL)
                {
                    EC801E_PublishResponse("valve_control", vr);
                    char v_order[4];
                    snprintf(v_order, sizeof(v_order), "%d", g_valve_last_order);
                    EC801E_PublishResponse("valve_last_order", v_order);
                }

                // alarm:normal + 当前 P/V (与告警 payload 对称)
                EC801E_PublishAlarm("normal", g_sensor.raw.pressure, g_sensor.voltage);

                // pressure_status
                {
                    PressureStatus_t ps = threshold_get_pressure_status(g_sensor.raw.pressure, g_sensor.last_pressure_collect);
                    const char *ps_str = threshold_pressure_status_str(ps);
                    rtc_get_time_str(s_status_ts, sizeof(s_status_ts));
                    snprintf(s_status_json, sizeof(s_status_json),
                        "[{\"id\":\"pressure_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]", ps_str, s_status_ts);
                    EC801E_PublishRawJson(s_status_json);
                }

                // temp_status
                {
                    TempStatus_t tps = threshold_get_temp_status(g_sensor.raw.temperature);
                    const char *ts_str = threshold_temp_status_str(tps);
                    rtc_get_time_str(s_status_ts, sizeof(s_status_ts));
                    snprintf(s_status_json, sizeof(s_status_json),
                        "[{\"id\":\"temp_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]", ts_str, s_status_ts);
                    EC801E_PublishRawJson(s_status_json);
                }
            }
            if(recovery_coldstart)
            {
                report_4g_power_state(0);
                EC801E_PowerOff();
            }
            else
                EC801E_SetPSM(1);
        }
    }

    // 正常: 存入缓冲区
    record.pressure = g_sensor.raw.pressure;
    record.temperature = g_sensor.raw.temperature;
    record.voltage = g_sensor.voltage;
    rtc_get_time_str(record.timestamp, sizeof(record.timestamp));
    record.flags = 0;
    memset(record.reserved, 0, sizeof(record.reserved));

    sensor_buf_push(&record);
    g_sensor.last_pressure_collect = g_sensor.raw.pressure;
    wakeup_count++;

    // === 低电压省电模式: 覆盖正常上报调度, 仅8:00/18:00上报 ===
    if(voltage_low_power)
    {
        uint32_t now_hhmmss = rtc_get_hhmmss();
        uint8_t current_hour = (uint8_t)(now_hhmmss / 10000);
        if((current_hour == 8 || current_hour == 18) && !vlp_uploaded_this_hour)
        {
            printf_debug("[SM] VLP trigger: hour=%d, uploading\r\n", current_hour);
            vlp_uploaded_this_hour = 1;
            ensure_full_init();
            app_state = STATE_UPLOAD;
        }
        else
        {
            if(current_hour != 8 && current_hour != 18)
                vlp_uploaded_this_hour = 0;
            app_state = STATE_SLEEP;
        }
        return;
    }

    // 判断是否需要上报 (正常模式, 基于时间对齐)
    if(strcmp(g_window.pattern, "dot") == 0)
    {
        // dot模式: 当前整点小时==dottime 且 本小时未上报 → 触发上报
        uint32_t now_hhmmss = rtc_get_hhmmss();
        uint8_t current_hour = (uint8_t)(now_hhmmss / 10000);
        if(current_hour == g_window.dottime && !dot_uploaded_this_hour)
        {
            printf_debug("[SM] DOT trigger: hour=%d == dottime=%d\r\n",
                         current_hour, g_window.dottime);
            dot_uploaded_this_hour = 1;
            ensure_full_init();
            app_state = STATE_UPLOAD;
        }
        else
        {
            // 跨小时后重置标志
            if(current_hour != g_window.dottime)
                dot_uploaded_this_hour = 0;
            app_state = STATE_SLEEP;
        }
    }
    else
    {
        // long/short: 基于时间网格判断是否到达上报整点
        uint32_t now_hhmmss = rtc_get_hhmmss();
        uint16_t cur_total_min = (uint16_t)(now_hhmmss / 10000) * 60
                               + (uint16_t)((now_hhmmss / 100) % 100);
        if(g_interval.upload_interval_min > 0 &&
           (cur_total_min % g_interval.upload_interval_min) == 0)
        {
            printf_debug("[SM] UPLOAD trigger: %02d:%02d on %dmin grid\r\n",
                         cur_total_min / 60, cur_total_min % 60,
                         g_interval.upload_interval_min);
            ensure_full_init();
            app_state = STATE_UPLOAD;
        }
        else
            app_state = STATE_SLEEP;
    }
}

// ==================== 报警确认 ====================
static void do_alarm(void)
{
    uint8_t i;
    uint8_t still_abnormal = 1;
    SensorRecord_t record;
    ThresholdResult_t result;
    const char *alarm_type = "unknown";

    ensure_full_init();

    printf_debug("[SM] ALARM: rechecking %d times...\r\n", APP_ALARM_RECHECK_COUNT);

    for(i = 0; i < APP_ALARM_RECHECK_COUNT; i++)
    {
        trea_delay_ms(APP_ALARM_RECHECK_DELAY_MS);

        PT304D_CollectAll(&g_sensor.raw2, &g_sensor.raw);
        g_sensor.voltage = adc_bat_read_voltage();

        result = threshold_check(g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage, g_sensor.last_pressure_collect, 1);
        printf_debug("[SM] ALARM recheck %d/%d: P=%.2f T=%.2f V=%.2f result=%d\r\n",
                     i + 1, APP_ALARM_RECHECK_COUNT, g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage, result);

        if(result == THRESHOLD_OK)
        {
            // 恢复正常: 偶发抖动
            printf_debug("[SM] ALARM: recovered at recheck %d, false alarm\r\n", i + 1);
            still_abnormal = 0;
            break;
        }
    }

    // 构造记录
    record.pressure = g_sensor.raw.pressure;
    record.temperature = g_sensor.raw.temperature;
    record.voltage = g_sensor.voltage;
    rtc_get_time_str(record.timestamp, sizeof(record.timestamp));
    memset(record.reserved, 0, sizeof(record.reserved));

    if(still_abnormal)
    {
        // 确认报警
        record.flags = SENSOR_FLAG_ALARM;

        // 判断报警类型 (基于threshold_check返回的result, 与阈值检查优先级一致)
        switch(result)
        {
            case THRESHOLD_VPROTECT:        alarm_type = "voltage_protect";    break;
            case THRESHOLD_VERY_LOW:        alarm_type = "pressure_verylow";   break;
            case THRESHOLD_VERY_HIGH:       alarm_type = "pressure_veryhigh";  break;
            case THRESHOLD_OVER_LIMIT:      alarm_type = "pressure_high";      break;
            case THRESHOLD_UNDER_LIMIT:     alarm_type = "pressure_low";       break;
            case THRESHOLD_MUTATION:        alarm_type = "pressure_mutation";  break;
            case THRESHOLD_TEMP_HIGH:       alarm_type = "temperature_high";   break;
            case THRESHOLD_TEMP_LOW:        alarm_type = "temperature_low";    break;
            case THRESHOLD_TEMP_VERY_HIGH:  alarm_type = "temperature_veryhigh"; break;
            case THRESHOLD_TEMP_VERY_LOW:   alarm_type = "temperature_verylow";  break;
            case THRESHOLD_VOLT_HIGH:       alarm_type = "voltage_high";       break;
            case THRESHOLD_VOLT_LOW:        alarm_type = "voltage_low";        break;
            default: alarm_type = "unknown"; break;
        }

        printf_debug("[SM] *** ALARM CONFIRMED: %s ***\r\n", alarm_type);

        // Debug串口打印报警
        printf_debug("[SM] ALARM: P=%.2f T=%.2f V=%.2f @%s\r\n",
                     g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage, record.timestamp);

        // 立即4G上报报警 + 压力状态 + 主动关阀
        {
            int alarm_coldstart = 0;
            int ec_ok;
            com_lpuart_init();
            trea_delay_ms(100);

            // short/dot/VLP模式: 4G断电状态, 需冷启动
            if(voltage_low_power ||
               strcmp(g_window.pattern, "short") == 0 || strcmp(g_window.pattern, "dot") == 0)
            {
                alarm_coldstart = 1;
                EC801E_PowerOn();
                ec_ok = (EC801E_InitAndConnect() == EC_OK);
                if(ec_ok)
                {
                    EC801E_SetPSM(0);
                    trea_delay_ms(300);
                    report_4g_power_state(1);
                }
            }
            else
            {
                ec_ok = (EC801E_EnsureConnected() == EC_OK);
            }

            if(ec_ok)
            {
                // 上报报警
                EC801E_PublishAlarm(alarm_type, g_sensor.raw.pressure, g_sensor.voltage);

                // 上报压力状态 (主动判断, 不依赖查询)
                {
                    PressureStatus_t ps = threshold_get_pressure_status(g_sensor.raw.pressure, g_sensor.last_pressure_collect);
                    const char *ps_str = threshold_pressure_status_str(ps);
                    rtc_get_time_str(s_status_ts, sizeof(s_status_ts));
                    snprintf(s_status_json, sizeof(s_status_json),
                        "[{\"id\":\"pressure_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]", ps_str, s_status_ts);
                    EC801E_PublishRawJson(s_status_json);
                }

                // 上报温度状态 (异常时立即上报, 不等待查询)
                {
                    TempStatus_t ts = threshold_get_temp_status(g_sensor.raw.temperature);
                    const char *ts_str = threshold_temp_status_str(ts);
                    rtc_get_time_str(s_status_ts, sizeof(s_status_ts));
                    snprintf(s_status_json, sizeof(s_status_json),
                        "[{\"id\":\"temp_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]", ts_str, s_status_ts);
                    EC801E_PublishRawJson(s_status_json);
                }

                // 主动异常关阀 (根据策略自动执行, 不依赖平台查询)
                // 守卫: !g_valve_alarm_closed 避免二次/多次异常覆盖 pre_alarm_order 与重复 PWM 耗电
                {
                    AbnormalClose_t policy = abnormal_from_str(g_abnormal_close);
                    if(abnormal_should_close_valve(policy, result) && !g_valve_alarm_closed)
                    {
                        // 首次异常: 保存关阀前状态, 后续重复异常跳过整段
                        g_valve_pre_alarm_order = g_valve_last_order;
                        printf_debug("[SM] ALARM auto-close valve (policy=%s, pre=%d)\r\n",
                                     g_abnormal_close, g_valve_pre_alarm_order);
                        const char *vr = valve_execute("close", 0);
                        g_valve_alarm_closed = 1;
                        config_save_ext();   // 持久化恢复标志
                        EC801E_PublishResponse("valve_control", vr);
                        {
                            char v_order[4];
                            snprintf(v_order, sizeof(v_order), "%d", g_valve_last_order);
                            EC801E_PublishResponse("valve_last_order", v_order);
                        }
                    }
                }
            }

            if(alarm_coldstart)
            {
                report_4g_power_state(0);
                EC801E_PowerOff();
            }
            else
            {
                trea_delay_ms(500);
                EC801E_SetPSM(1);
            }
        }
    }
    else
    {
        // 偶发抖动, 正常记录
        record.flags = 0;
    }

    // 存入缓冲区
    sensor_buf_push(&record);
    g_sensor.last_pressure_collect = g_sensor.raw.pressure;
    wakeup_count++;

    // 回到休眠 (不在此处触发上报周期判断, 下次正常采集时再判断)
    app_state = STATE_SLEEP;
}

// ==================== 数据上报 (批量逐条, 三模式) ====================
static void do_upload(void)
{
    uint16_t n, i;
    const SensorRecord_t *rec;
    int is_coldstart = 0;

    // === 窗口检查 (dot/VLP模式由do_collect已确认时间点, 直接通过) ===
    if(!voltage_low_power && strcmp(g_window.pattern, "dot") != 0)
    {
        if(!config_is_active_window())
        {
            printf_debug("[SM] UPLOAD: outside window (%s), skip\r\n", g_window.pattern);
            app_state = STATE_SLEEP;
            return;
        }
    }

    printf_debug("[SM] UPLOAD: %d records (pattern=%s%s)\r\n",
                 sensor_buf_count(), g_window.pattern,
                 voltage_low_power ? " VLP" : "");

    // === 建立4G连接 ===
    com_lpuart_init();
    trea_delay_ms(100);

    // 低电压省电模式: 强制走冷启动路径
    if(voltage_low_power || strcmp(g_window.pattern, "long") != 0)
    {
        // short/dot/VLP: 4G断电状态, 需完整开机
        is_coldstart = 1;
        EC801E_PowerOn();
        if(EC801E_InitAndConnect() != EC_OK)
        {
            printf_debug("[SM] UPLOAD: cold start failed, power off\r\n");
            EC801E_PowerOff();
            app_state = STATE_SLEEP;
            return;
        }
        EC801E_SetPSM(0);
        trea_delay_ms(300);
        report_4g_power_state(1);
    }
    else
    {
        // long模式(非VLP): 4G应已连接(QSCLK), 直接确认
        if(EC801E_EnsureConnected() != EC_OK)
        {
            printf_debug("[SM] UPLOAD: connect failed, skip\r\n");
            trea_delay_ms(500);
            EC801E_SetPSM(1);
            app_state = STATE_SLEEP;
            return;
        }
    }

    // === 逐条上传并确认弹出 (方案 B) ===
    while(sensor_buf_count() > 0)
    {
        rec = sensor_buf_get(0);  // 永远获取当前最老的一条记录
        if(rec)
        {
            if(EC801E_PublishSingleRecord(rec) == EC_OK)
            {
                // 发送成功，从缓冲区弹出该条记录
                sensor_buf_pop();

                // 若还有后续数据，延时 400ms 隔离
                if(sensor_buf_count() > 0)
                    trea_delay_ms(400);
            }
            else
            {
                // 物理发送失败，立刻终止循环，保护未发送的数据
                printf_debug("[SM] UPLOAD: Publish record failed, stop remaining uploads\r\n");
                break;
            }
        }
        else
        {
            // 防御保护，防止空指针卡死
            sensor_buf_pop();
        }
    }

    // 上报传感器在线状态 (每次上报周期均上报)
    trea_delay_ms(500);
    {
        const char *have    = (g_sensor.raw.err != PT304D_ERR_I2C) ? "1" : "0";
        const char *rstatus = (g_sensor.raw.err == PT304D_OK) ? "1" : "0";   // I2C 采集是否成功

        rtc_get_time_str(s_status_ts, sizeof(s_status_ts));
        snprintf(s_status_json, sizeof(s_status_json),
            "[{\"id\":\"rtu_have\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"rtu_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]",
            have, s_status_ts, rstatus, s_status_ts);
        EC801E_PublishRawJson(s_status_json);
        printf_debug("[SM] rtu_have=%s rtu_status=%s\r\n", have, rstatus);
    }

    // 上传流程结束，复位唤醒周期统计计数
    wakeup_count = 0;
    printf_debug("[SM] UPLOAD finished, remaining records in buffer: %d\r\n", sensor_buf_count());

    // === RTC校准: 利用4G在线从网络同步时间 ===
    {
        trea_delay_ms(200);
        EC_Status_t sync_ret = EC801E_SyncTime();
        if(sync_ret == EC_OK)
            printf_debug("[SM] RTC calibrated from network\r\n");
        else
            printf_debug("[SM] RTC calibration failed (non-critical)\r\n");
    }

    // === 模式特定收尾 ===
    if(is_coldstart)
    {
        // 冷启动模式(VLP/short/dot): 等待30s接收平台下发指令
        printf_debug("[SM] Waiting 30s for downlink...\r\n");
        trea_delay_ms(30000);
        {
            uint8_t dl_count = drain_pending_downlink();
            if(dl_count > 0)
                printf_debug("[SM] Processed %d downlink msg(s)\r\n", dl_count);
        }
        report_4g_power_state(0);
        EC801E_PowerOff();
    }
    else
    {
        // long: 重新启用QSCLK休眠
        trea_delay_ms(500);
        EC801E_SetPSM(1);
    }

    app_state = STATE_SLEEP;
}



// ==================== 处理MQTT下发指令 (★最高优先级) ====================
static void do_downlink(void)
{
    uint8_t msg_count;

    printf_debug("[SM] DOWNLINK\r\n");
    com_lpuart_init();
    trea_delay_ms(100);

    msg_count = drain_pending_downlink();

    if(msg_count == 0)
    {
        printf_debug("[SM] No downlink, check MQTT...\r\n");
        if(EC801E_EnsureConnected() == EC_OK)
            printf_debug("[SM] MQTT OK\r\n");
    }
    else
    {
        printf_debug("[SM] Processed %d msg(s)\r\n", msg_count);
    }

    // 重新启用模块休眠
    trea_delay_ms(500);
    EC801E_SetPSM(1);

    app_state = STATE_SLEEP;
}

// ==================== Debug串口命令(需要硬件上下文) ====================
static void debug_check_cmd(void)
{
    static char line_buf[512];
    uint16_t len;

    while((len = debug_rx_read_line(line_buf, sizeof(line_buf), 0)) > 0)
    {
        printf_debug("[DBG] %s\r\n", line_buf);

        // ---- 需要main.c硬件上下文的命令 ----
        if(strcmp(line_buf, "status") == 0)
        {
            const char *state_names[] = {"INIT","SLEEP","WAKEUP","COLLECT","UPLOAD","DOWNLINK","ALARM"};
            printf_debug("  state=%s wakeup=%d buf=%d\r\n",
                         (app_state <= STATE_ALARM) ? state_names[app_state] : "?",
                         wakeup_count, sensor_buf_count());
            printf_debug("  P=%.2f T=%.2f V=%.2f last_Pc=%.2f last_Pr=%.2f\r\n",
                         g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage, g_sensor.last_pressure_collect, g_sensor.last_pressure_realtime);
            printf_debug("  valve=%d abnormal=%s\r\n", g_valve_last_order, g_abnormal_close);
            printf_debug("  PT304D err=%d stat=0x%02X\r\n", g_sensor.raw.err, g_sensor.raw.stat_raw);
            continue;
        }
        if(strcmp(line_buf, "sensor") == 0)
        {
            PT304D_CollectAll(&g_sensor.raw2, &g_sensor.raw);
            g_sensor.voltage = adc_bat_read_voltage();
            printf_debug("  err=%d stat=0x%02X P=%.2f T=%.2f V=%.2f\r\n",
                         g_sensor.raw.err, g_sensor.raw.stat_raw, g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage);
            if(g_sensor.raw.err != PT304D_OK)
                printf_debug("  press_raw=0x%06X temp_raw=0x%06X\r\n", g_sensor.raw.press_raw, g_sensor.raw.temp_raw);
            continue;
        }
        if(strcmp(line_buf, "sensor_status") == 0)
        {
            PT304D_CollectAll(&g_sensor.raw2, &g_sensor.raw);
            const char *have   = (g_sensor.raw.err != PT304D_ERR_I2C) ? "1" : "0";
            const char *status = (g_sensor.raw.err == PT304D_OK)       ? "1" : "0";
            printf_debug("  have=%s status=%s err=%d stat=0x%02X\r\n",
                         have, status, g_sensor.raw.err, g_sensor.raw.stat_raw);
            continue;
        }
        if(strcmp(line_buf, "temp_status") == 0)
        {
            PT304D_CollectAll(&g_sensor.raw2, &g_sensor.raw);
            TempStatus_t ts = threshold_get_temp_status(g_sensor.raw.temperature);
            printf_debug("  T=%.2f -> %s\r\n", g_sensor.raw.temperature, threshold_temp_status_str(ts));
            continue;
        }
        if(strcmp(line_buf, "pressure_status") == 0)
        {
            PT304D_CollectAll(&g_sensor.raw2, &g_sensor.raw);
            PressureStatus_t ps = threshold_get_pressure_status(g_sensor.raw.pressure, g_sensor.last_pressure_collect);
            printf_debug("  P=%.2f -> %s\r\n", g_sensor.raw.pressure, threshold_pressure_status_str(ps));
            continue;
        }
        if(strcmp(line_buf, "sim") == 0)
        {
            char csq[8] = {0}, iccid[32] = {0}, imei[24] = {0};
            EC801E_Wakeup();
            EC801E_QueryCSQ(csq, sizeof(csq));
            EC801E_QueryICCID(iccid, sizeof(iccid));
            EC801E_QueryIMEI(imei, sizeof(imei));
            printf_debug("  CSQ=%s ICCID=%s IMEI=%s\r\n", csq, iccid, imei);
            continue;
        }
        if(strcmp(line_buf, "restart") == 0)
        {
            printf_debug("[DBG] Restarting...\r\n");
            trea_delay_ms(50);
            NVIC_SystemReset();
        }
        if(strncmp(line_buf, "valve:", 6) == 0)
        {
            const char *action = line_buf + 6;  // "open" 或 "close"
            const char *result = valve_execute(action, 1);
            // 用户主动操作清除报警恢复标志
            if(g_valve_alarm_closed)
            {
                g_valve_alarm_closed = 0;
                g_valve_pre_alarm_order = g_valve_last_order;
                config_save_ext();
                printf_debug("  cleared alarm_closed flag (user override)\r\n");
            }
            printf_debug("  valve(%s) -> %s  last_order=%d\r\n", action, result, g_valve_last_order);
            continue;
        }
        if(strncmp(line_buf, "first_ip:", 9) == 0)
        {
            // 格式: first_ip:120.78.220.171.1883
            char ip_buf[MQTT_FIELD_MAX];
            strncpy(ip_buf, line_buf + 9, sizeof(ip_buf) - 1);
            ip_buf[sizeof(ip_buf) - 1] = '\0';
            char *last_dot = strrchr(ip_buf, '.');
            if(last_dot)
            {
                *last_dot = '\0';
                g_mqtt.port = (uint16_t)atoi(last_dot + 1);
                strncpy(g_mqtt.server, ip_buf, MQTT_FIELD_MAX - 1);
                config_save_mqtt();
                printf_debug("  Primary IP set: %s:%d, reconnecting...\r\n", g_mqtt.server, g_mqtt.port);
                EC801E_DisconnectMQTT();
                trea_delay_ms(500);
                EC801E_ConnectMQTT();
                EC801E_SubscribeTopic();
            }
            else
                printf_debug("  Invalid format, use: first_ip:IP.port\r\n");
            continue;
        }
        if(strncmp(line_buf, "scend_ip:", 9) == 0)
        {
            // 格式: scend_ip:120.78.220.171.1883
            char ip_buf[MQTT_FIELD_MAX];
            strncpy(ip_buf, line_buf + 9, sizeof(ip_buf) - 1);
            ip_buf[sizeof(ip_buf) - 1] = '\0';
            char *last_dot = strrchr(ip_buf, '.');
            if(last_dot)
            {
                *last_dot = '\0';
                g_mqtt.port2 = (uint16_t)atoi(last_dot + 1);
                strncpy(g_mqtt.server2, ip_buf, MQTT_FIELD_MAX - 1);
                config_save_mqtt();
                printf_debug("  Secondary IP set: %s:%d\r\n", g_mqtt.server2, g_mqtt.port2);
            }
            else
                printf_debug("  Invalid format, use: scend_ip:IP.port\r\n");
            continue;
        }
        if(strcmp(line_buf, "4Gpower_off") == 0)
        {
            printf_debug("  4G power off\r\n");
            report_4g_power_state(0);
            trea_delay_ms(200);
            EC801E_PowerOff();
            fourG_user_off = 1;
            continue;
        }
        if(strcmp(line_buf, "4Gpower_on") == 0)
        {
            printf_debug("  4G power on\r\n");
            fourG_user_off = 0;
            com_lpuart_init();
            trea_delay_ms(100);
            EC801E_PowerOn();
            EC801E_InitAndConnect();
            if(EC801E_EnsureConnected() == EC_OK)
            {
                report_4g_power_state(1);
                printf_debug("  4G power on OK, reported\r\n");
            }
            else
                printf_debug("  4G power on FAILED\r\n");
            EC801E_SetPSM(1);
            continue;
        }
        if(strcmp(line_buf, "mqtt_reconnect") == 0)
        {
            printf_debug("  MQTT reconnecting...\r\n");
            EC801E_Wakeup();
            EC801E_DisconnectMQTT();
            trea_delay_ms(500);
            if(EC801E_ConnectMQTT() == EC_OK)
            {
                EC801E_SubscribeTopic();
                printf_debug("  MQTT reconnected OK\r\n");
            }
            else
                printf_debug("  MQTT reconnect FAILED\r\n");
            continue;
        }
        if(strcmp(line_buf, "vlp") == 0)
        {
            printf_debug("  voltage_low_power=%d vlp_uploaded=%d valve_alarm_closed=%d pre=%d\r\n",
                         voltage_low_power, vlp_uploaded_this_hour,
                         g_valve_alarm_closed, g_valve_pre_alarm_order);
            printf_debug("  spupower en=%d vprotect=%.2f\r\n",
                         g_spupower.enabled, g_spupower.protect);
            continue;
        }
        if(strcmp(line_buf, "buf") == 0)
        {
            printf_debug("  sensor_buf: %d records\r\n", sensor_buf_count());
            continue;
        }
        if(strcmp(line_buf, "time") == 0)
        {
            rtc_show_time();
            continue;
        }

        // ---- 不需要硬件上下文的命令, 交给eeprom_config.c处理 ----
        if(config_parse_and_apply(line_buf) == 0)
            printf_debug("[DBG] Unknown: %s\r\n", line_buf);
    }
}

// ==================== 主函数 ====================
int main(void)
{
    SensorRecord_t init_record;

    // ==================== STATE_INIT ====================
    sys_init();
    com_lpuart_init();
    lpuart_wakeup_init();

    config_load_all();
    config_enter_setup_mode(0);
    config_print_current();

    rtc_init_all();
    rtc_alarm_wakeup_init();
    PT304D_Init(PT304D_SENSOR_2);
    sensor_buf_init();

    printf_debug("\r\n[OTA] yes%s \r\n", APP_FIRMWARE_VERSION);
    printf_debug("\r\n========================================\r\n");
    printf_debug("  Household Valve Controller v%s\r\n", APP_FIRMWARE_VERSION);
    printf_debug("  Collect: %dmin  Upload: %dmin (grid-aligned)\r\n",
                 g_interval.collect_interval_min,
                 g_interval.upload_interval_min);
    printf_debug("  Window: pattern=%s start=%s stop=%s er=%d dot=%d\r\n",
                 g_window.pattern, g_window.start, g_window.stop,
                 g_window.end_result, g_window.dottime);
    printf_debug("  VPTH en=%d ppmax=%.1f pmax=%.1f ppmin=%.1f pmin=%.1f mut=%.1f shock=%.1f\r\n",
                 g_pthreshold.enabled, g_pthreshold.ppmax, g_pthreshold.pmax,
                 g_pthreshold.ppmin, g_pthreshold.pmin,
                 g_pthreshold.mutation, g_pthreshold.shock);
    printf_debug("  TTH  en=%d high=%.1f low=%.1f vhigh=%.1f vlow=%.1f\r\n",
                 g_tthreshold.enabled, g_tthreshold.high, g_tthreshold.low,
                 g_tthreshold.vhigh, g_tthreshold.vlow);
    printf_debug("  SPUP en=%d high=%.2f low=%.2f protect=%.2f\r\n",
                 g_spupower.enabled, g_spupower.high, g_spupower.low, g_spupower.protect);
    printf_debug("  VDLY en=%d delay_ms=%d\r\n", g_vdelay.enabled, g_vdelay.delay_ms);
    printf_debug("  Keepalive: %d s\r\n", config_get_keepalive());
    printf_debug("========================================\r\n");

    // 4G开机 + 注网 + MQTT连接 + keepalive + 时间同步 + QSCLK
    EC801E_PowerOn();
    EC801E_InitAndConnect();
    // InitAndConnect末尾启用了QSCLK休眠, 这里先禁用以确保后续AT查询成功
    EC801E_SetPSM(0);
    trea_delay_ms(300);
    report_4g_power_state(1);

    // === OTA 启动回执: 若 InfoBlock 显示上次升级 SUCCESS/FAILED, 上报 mqttUpgradeReply ===
    ota_app_report_boot_status();

    // 首次采集+存入缓冲+上报
    PT304D_CollectAll(&g_sensor.raw2, &g_sensor.raw);
    trea_delay_ms(500);
    g_sensor.voltage = adc_bat_read_voltage();

    init_record.pressure = g_sensor.raw.pressure;
    init_record.temperature = g_sensor.raw.temperature;
    init_record.voltage = g_sensor.voltage;
    rtc_get_time_str(init_record.timestamp, sizeof(init_record.timestamp));
    init_record.flags = 0;
    memset(init_record.reserved, 0, sizeof(init_record.reserved));
    sensor_buf_push(&init_record);
    g_sensor.last_pressure_collect  = g_sensor.raw.pressure;
    g_sensor.last_pressure_realtime = g_sensor.raw.pressure;

    printf_debug("[SM] INIT V=%.2f P=%.2f T=%.2f\r\n", g_sensor.voltage, g_sensor.raw.pressure, g_sensor.raw.temperature);

    // ==================== 启动批量上报 ====================
    // 上报传感器数据
    EC801E_PublishSensorData(&g_sensor.raw, g_sensor.voltage);
    trea_delay_ms(500);

    // 批 1: restart=1 + valve_last_order
    {
        char v_order[4];
        snprintf(v_order, sizeof(v_order), "%d", g_valve_last_order);
        static char boot_json[256];
        static char boot_ts[32];
        rtc_get_time_str(boot_ts, sizeof(boot_ts));
        snprintf(boot_json, sizeof(boot_json),
            "[{\"id\":\"restart\",\"value\":\"1\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"valve_last_order\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]",
            boot_ts, v_order, boot_ts);
        EC801E_PublishRawJson(boot_json);
    }
    trea_delay_ms(500);

    // 批 2: 设备信息 (csq + soft-version + ICCID + IMEI)
    {
        static char info_json[512];
        char csq[8] = {0}, iccid[32] = {0}, imei[24] = {0};
        EC801E_Wakeup();
        EC801E_QueryCSQ(csq, sizeof(csq));
        trea_delay_ms(200);
        EC801E_QueryICCID(iccid, sizeof(iccid));
        trea_delay_ms(200);
        EC801E_QueryIMEI(imei, sizeof(imei));
        static char info_ts[32];
        rtc_get_time_str(info_ts, sizeof(info_ts));
        snprintf(info_json, sizeof(info_json),
            "[{\"id\":\"csq\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"soft-version\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"ICCID\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"IMEI\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]",
            csq, info_ts, APP_FIRMWARE_VERSION, info_ts, iccid, info_ts, imei, info_ts);
        EC801E_PublishRawJson(info_json);
        printf_debug("[SM] Device info: CSQ=%s ICCID=%s IMEI=%s\r\n", csq, iccid, imei);
    }
    trea_delay_ms(500);

    // 批 3: 传感器在线状态
    {
        const char *have   = (g_sensor.raw.err != PT304D_ERR_I2C) ? "1" : "0";
        const char *status = (g_sensor.raw.err == PT304D_OK) ? "1" : "0";   // I2C 采集是否成功
        rtc_get_time_str(s_status_ts, sizeof(s_status_ts));
        snprintf(s_status_json, sizeof(s_status_json),
            "[{\"id\":\"rtu_have\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"rtu_status\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]",
            have, s_status_ts, status, s_status_ts);
        EC801E_PublishRawJson(s_status_json);
        g_sensor.prev_online = (g_sensor.raw.err == PT304D_OK) ? 1 : 0;
    }
    trea_delay_ms(500);

    // 批 4: 压力阈值 vpthreshold (与平台同步当前配置)
    {
        static char vpth_json[640];
        static char vpth_ts[32];
        rtc_get_time_str(vpth_ts, sizeof(vpth_ts));
        snprintf(vpth_json, sizeof(vpth_json),
            "[{\"id\":\"vpthreshold_switch\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"vpthreshold_ppmax\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"vpthreshold_pmax\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"vpthreshold_ppmin\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"vpthreshold_pmin\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"vpthreshold_mutation\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"vpthreshold_shock\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"}]",
            g_pthreshold.enabled, vpth_ts,
            g_pthreshold.ppmax, vpth_ts, g_pthreshold.pmax, vpth_ts,
            g_pthreshold.ppmin, vpth_ts, g_pthreshold.pmin, vpth_ts,
            g_pthreshold.mutation, vpth_ts, g_pthreshold.shock, vpth_ts);
        EC801E_PublishRawJson(vpth_json);
    }
    trea_delay_ms(500);

    // 批 5: 温度阈值 Tthreshold
    {
        static char tth_json[480];
        static char tth_ts[32];
        rtc_get_time_str(tth_ts, sizeof(tth_ts));
        snprintf(tth_json, sizeof(tth_json),
            "[{\"id\":\"Tthreshold_switch\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"Tthreshold_high\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"Tthreshold_low\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"Tthreshold_vhigh\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"Tthreshold_vlow\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"}]",
            g_tthreshold.enabled, tth_ts,
            g_tthreshold.high, tth_ts, g_tthreshold.low, tth_ts,
            g_tthreshold.vhigh, tth_ts, g_tthreshold.vlow, tth_ts);
        EC801E_PublishRawJson(tth_json);
    }
    trea_delay_ms(500);

    // 批 6: 电压检测 spupower
    {
        static char sp_json[384];
        static char sp_ts[32];
        rtc_get_time_str(sp_ts, sizeof(sp_ts));
        snprintf(sp_json, sizeof(sp_json),
            "[{\"id\":\"spupower_switch\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"spupower_high\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"spupower_low\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"spupower_protect\",\"value\":\"%.2f\",\"ts\":\"%s\",\"remark\":\"\"}]",
            g_spupower.enabled, sp_ts,
            g_spupower.high, sp_ts, g_spupower.low, sp_ts,
            g_spupower.protect, sp_ts);
        EC801E_PublishRawJson(sp_json);
    }
    trea_delay_ms(500);

    // 批 7: 阀门动作延时 valve_delay
    {
        static char vd_json[256];
        static char vd_ts[32];
        rtc_get_time_str(vd_ts, sizeof(vd_ts));
        snprintf(vd_json, sizeof(vd_json),
            "[{\"id\":\"delay_switch\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"delay_time\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"}]",
            g_vdelay.enabled, vd_ts, g_vdelay.delay_ms, vd_ts);
        EC801E_PublishRawJson(vd_json);
    }
    trea_delay_ms(500);

    EC801E_SetPSM(1);
    wakeup_count = 0;

    // 初始化完成，进入休眠
    app_state = STATE_SLEEP;

    // ==================== 状态机主循环 ====================
    while(1)
    {
        // ISR置位后立即响应debug串口配置命令
        if(debug_rx_line_ready())
            debug_check_cmd();

        switch(app_state)
        {
            case STATE_SLEEP:    do_sleep();    break;
            case STATE_WAKEUP:   do_wakeup();   break;
            case STATE_COLLECT:  do_collect();  break;
            case STATE_UPLOAD:   do_upload();   break;
            case STATE_DOWNLINK: do_downlink(); break;
            case STATE_ALARM:    do_alarm();    break;
            default:             app_state = STATE_SLEEP; break;
        }
    }
}
