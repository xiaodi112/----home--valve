/**
 * @file main.c
 * @brief 调压箱物联网控制器 + 状态机驱动低功耗主程序
 *
 * 架构: MQTT持久连接 + MCU深休眠 + LPUART唤醒接收下发
 *
 * 状态机 (9个状态):
 *   STATE_INIT     → 上电初始化 → MQTT连接+订阅+首次上报 → STATE_SLEEP
 *   STATE_SLEEP    → 设闹钟 → 深休眠 → 模块QSCLK保持MQTT) → STATE_WAKEUP
 *   STATE_WAKEUP   → 恢复外设+判断唤醒源
 *                     RTC闹钟 → STATE_COLLECT
 *                     LPUART  → STATE_DOWNLINK
 *   STATE_COLLECT  → 采集+阈值检查 → 存入缓冲区 → STATE_UPLOAD/ALARM/SLEEP
 *   STATE_UPLOAD   → 批量逐条上报+清空缓冲区 → STATE_SLEEP
 *   STATE_DOWNLINK → 读取+处理MQTT下发指令+立即上报结果 → STATE_SLEEP
 *   STATE_ALARM    → 5次重采确认 → 报警上报 → STATE_SLEEP
 *   STATE_BLE_OTA  → BLE蓝牙固件升级(等待连接→接收数据→写Flash→提交)
 *   STATE_CH4  	→ 激光甲烷传感器采集: 上电→预热(20s)→采集→阈值判断→上报→断电。
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
#include "power.h"
#include "wakeup.h"
#include "tim.h"
#include "valve_timing.h"
#include "PT_304_35.h"
#include "valve.h"
#include "downlink.h"
#include "report.h"
#include "app_config.h"
#include "eeprom_config.h"
#include "m24m01.h"
#include "main.h"
#include "adc_bat.h"
#include "soft_i2c.h"
#include "motor.h"
#include "sensor_record.h"
#include "threshold.h"
#include "ota_app.h"
#include "slcd_seg.h"
#include "lcd_ui.h"
#include "led.h"
#include "buzzer.h"
#include "ble_ota.h"
#include "json_builder.h"
#include "eeprom_history.h"
#include "ch4_alarm.h"

// ==================== 状态机 ====================
/* AppState_t moved to main.h */

// ==================== 全局变量 ====================
SensorContext_t g_sensor = { .last_pressure_collect = -1.0f, .last_pressure_realtime = -1.0f, .last_pressure_collect2 = -1.0f, .last_pressure_realtime2 = -1.0f, .prev_online = 1, .offline_cnt = 0, .online_cnt = 0, .prev_online2 = 1, .offline_cnt2 = 0, .online_cnt2 = 0, .last_normal_voltage = 3.6f };

#define SENSOR_OFFLINE_DEBOUNCE  3   // 连续N次离线才确认状态变化
#define SENSOR_ONLINE_DEBOUNCE   2   // 连续N次在线才确认状态变化(比离线快)
#define	one_by_one_delay		 900	// 逐条上报间隔延时

static volatile uint16_t wakeup_count = 0;
AppState_t app_state = STATE_INIT;
volatile uint8_t g_ch4_request = 0;
uint8_t g_alarm_recover_reported = 0;   // 报警关阀恢复"已上报一次"标志 (RAM, 防重复上报)
static uint8_t dot_uploaded_mask[3] = {0};
static uint16_t dot_uploaded_day = 0xFFFF;  // RTC day of uploaded bitmap (cross-day reset)  // dot pattern: uploaded-hour bitmap
// valve_alarm_closed / valve_pre_alarm_order / ch4_alarm_closed 定义为 g_* 全局 (EEPROM 持久化, OTA 升级后自动恢复), 见 eeprom_config.h
uint8_t voltage_low_power = 0;              // 低电压省电模式; 8:00/18:00上报
uint8_t fourG_user_off = 0;                 // 用户手动关闭4G标志(防止do_wakeup盲目恢复PD9)
static uint8_t fourG_init_failed = 0;       // 4G注册失败标志(防止do_sleep保持4G在线, 下次上报重新尝试)
static uint8_t fourG_weak_signal = 0;       // 弱信号断电标志(long模式按CSQ判断, 平时保持断电)
#define CSQ_WEAK_THRESHOLD  8               // CSQ<该值则断电省电(8约-97dBm)
static uint8_t vlp_uploaded_this_hour = 0;  // 低电压模式: 防止同一小时重复上报
/* 阈值报警状态: 双路报警结果 + 双路温度状态 (防止压力报警叠加温度变化) */
typedef struct {
    ThresholdResult_t r1;      /* 通道1 报警结果 (压力/温度/突变按优先级) */
    ThresholdResult_t r2;      /* 通道2 报警结果 */
    TempStatus_t      temp1;   /* 通道1 温度状态 (单独, 不叠加压力报警等级) */
    TempStatus_t      temp2;   /* 通道2 温度状态 */
} ThresholdAlarmState_t;

static ThresholdAlarmState_t s_last_rt_state      = { THRESHOLD_OK, THRESHOLD_OK, TEMP_NORMAL, TEMP_NORMAL };
static ThresholdAlarmState_t s_last_collect_state = { THRESHOLD_OK, THRESHOLD_OK, TEMP_NORMAL, TEMP_NORMAL };

static void alarm_state_fill(ThresholdAlarmState_t *st, ThresholdResult_t r1, ThresholdResult_t r2)
{
    st->r1 = r1;
    st->r2 = r2;
    st->temp1 = threshold_get_temp_status(g_sensor.raw.temperature);
    st->temp2 = (g_sensor_count >= 2 && g_sensor.raw2.err == PT304D_OK)
              ? threshold_get_temp_status2(g_sensor.raw2.temperature) : TEMP_NORMAL;
}

static uint8_t alarm_state_is_normal(const ThresholdAlarmState_t *st)
{
    return (st->r1 == THRESHOLD_OK && st->r2 == THRESHOLD_OK &&
            st->temp1 == TEMP_NORMAL && st->temp2 == TEMP_NORMAL) ? 1 : 0;
}

static void alarm_state_reset(ThresholdAlarmState_t *st)
{
    st->r1 = THRESHOLD_OK;
    st->r2 = THRESHOLD_OK;
    st->temp1 = TEMP_NORMAL;
    st->temp2 = TEMP_NORMAL;
}

/* 实时模式防重复状态重置 (real_set 切换实时模式时调用) */
void threshold_alarm_reset_rt(void)
{
    alarm_state_reset(&s_last_rt_state);
}
static uint32_t g_ble_on_hhmmss = 0;                          // BLE开启时的RTC时间 (HHMMSS), 超过60s未连接超时自动关闭
static uint8_t g_is_minimal_wakeup = 0;            // 标志是否为最小唤醒(纯RTC采集或按键快检状态)
uint8_t g_need_adc_sampling = 1;                   // 标志当前唤醒是否需要执行电池电压采样
static uint16_t last_grid_collect_min = 0xFFFF;    // 标志最后一次执行网格采集的分钟序号, 防止同一采集分钟重复上报
#if LASER_CH4_ENABLED
static uint8_t s_laser_done_hour = 0xFF;           // 激光定时采集: 本小时已触发标志(跨小时自动恢复)
#endif

// ==================== 定时开关阀 当日执行标志 ====================
uint8_t timing_open_done_today  = 0;   // 当日已执行开阀
uint8_t timing_close_done_today = 0;   // 当日已执行关阀
static uint8_t timing_last_reset_hour  = 0xFF;// 上次重置标志的小时(防止重复)
volatile uint8_t g_timing_pending = 0;   /* timing valve pending (set by RTC ISR) */
volatile uint8_t g_timing_alarm_set = 0; /* current RTC alarm is a timing-valve alarm */

// ==================== LCD 超时关闭 ====================
uint8_t  g_lcd_active     = 0;      // LCD当前是否正在显示(运行状态, 区别于产品配置g_lcd_enabled)
static uint32_t g_lcd_on_hhmmss = 0; // LCD开启时的RTC时间 (HHMMSS格式)
#define LCD_DISPLAY_TIMEOUT_SEC  60  // LCD无操作显示超时

static uint32_t hhmmss_to_seconds(uint32_t hhmmss)
{
    uint8_t h = (uint8_t)(hhmmss / 10000);
    uint8_t m = (uint8_t)((hhmmss / 100) % 100);
    uint8_t s = (uint8_t)(hhmmss % 100);
    return (uint32_t)h * 3600 + (uint32_t)m * 60 + s;
}
static void lcd_activate(void)
{
    if(!g_lcd_enabled) return;
    if(!g_lcd_active)
    {
        slcd_seg_configuration();
        lcd_ui_init();
    }
    g_lcd_active = 1;
    g_lcd_on_hhmmss = rtc_get_hhmmss();
}

void lcd_shutdown(void)
{
    if(!g_lcd_active) return;
    slcd_shutdown();   /* 关闭SLCD并释放引脚 (硬件关屏由BSP实现) */
    g_lcd_active = 0;
}

static void lcd_check_timeout(void)
{
#if LCD_LOCK_ENABLED
    /* LCD锁定模式: 取消60s超时, 改用lcd_shutdown关闭 */
    if(!g_lcd_active) return;
    uint32_t now_sec = hhmmss_to_seconds(rtc_get_hhmmss());
    uint32_t on_sec  = hhmmss_to_seconds(g_lcd_on_hhmmss);
    uint32_t elapsed;
    if(now_sec >= on_sec)
        elapsed = now_sec - on_sec;
    else
        elapsed = (86400 - on_sec) + now_sec;  // 跨过午夜
    if(elapsed >= LCD_DISPLAY_TIMEOUT_SEC)
        lcd_shutdown();
#else
    /* LCD常亮模式: 不做超时, 保持常开 */
    (void)0;
#endif
}

// 传感器在线/离线消抖状态机 (进气端raw + 出气端raw2 共用; do_collect/do_realtime_check 复用)
// 更新 offline_cnt/offline_cnt2; 消抖通过后更新 prev_online/prev_online2.
// 返回: 1 = 发生经消抖确认的状态变化 0 = 无变化
static uint8_t sensor_debounce_update(uint8_t cur_online, uint8_t cur_online2)
{
    uint8_t state_changed = 0;

    // --- 进气端(raw, PT304D #2) ---
    if(cur_online && !g_sensor.prev_online)
    {
        g_sensor.offline_cnt = 0;
        g_sensor.online_cnt++;
        if(g_sensor.online_cnt >= SENSOR_ONLINE_DEBOUNCE)
            state_changed = 1;
    }
    else if(!cur_online && g_sensor.prev_online)
    {
        g_sensor.online_cnt = 0;
        g_sensor.offline_cnt++;
        if(g_sensor.offline_cnt >= SENSOR_OFFLINE_DEBOUNCE)
            state_changed = 1;
    }
    else
    {
        g_sensor.offline_cnt = cur_online ? 0 : g_sensor.offline_cnt;
        g_sensor.online_cnt  = cur_online ? g_sensor.online_cnt : 0;
    }

    // --- 出气端(raw2, PT304D #1) --- // 双传感器时才参与消抖
    if(g_sensor_count >= 2)
    {
        if(cur_online2 && !g_sensor.prev_online2)
        {
            g_sensor.offline_cnt2 = 0;
            g_sensor.online_cnt2++;
            if(g_sensor.online_cnt2 >= SENSOR_ONLINE_DEBOUNCE)
                state_changed = 1;
        }
        else if(!cur_online2 && g_sensor.prev_online2)
        {
            g_sensor.online_cnt2 = 0;
            g_sensor.offline_cnt2++;
            if(g_sensor.offline_cnt2 >= SENSOR_OFFLINE_DEBOUNCE)
                state_changed = 1;
        }
        else
        {
            g_sensor.offline_cnt2 = cur_online2 ? 0 : g_sensor.offline_cnt2;
            g_sensor.online_cnt2  = cur_online2 ? g_sensor.online_cnt2 : 0;
        }
    }

    if(state_changed)
    {
        // 同时更新两个通道的 prev 状态
        if(cur_online != g_sensor.prev_online &&
           (cur_online  ? (g_sensor.online_cnt  >= SENSOR_ONLINE_DEBOUNCE)
                        : (g_sensor.offline_cnt >= SENSOR_OFFLINE_DEBOUNCE)))
        {
            g_sensor.prev_online = cur_online;
            g_sensor.offline_cnt = 0;
            g_sensor.online_cnt  = 0;
        }
        if(g_sensor_count >= 2 &&
           cur_online2 != g_sensor.prev_online2 &&
           (cur_online2 ? (g_sensor.online_cnt2  >= SENSOR_ONLINE_DEBOUNCE)
                        : (g_sensor.offline_cnt2 >= SENSOR_OFFLINE_DEBOUNCE)))
        {
            g_sensor.prev_online2 = cur_online2;
            g_sensor.offline_cnt2 = 0;
            g_sensor.online_cnt2  = 0;
        }
    }
    return state_changed;
}

// ==================== 硬件恢复 ====================
static void sys_init(void)
{
    /* 重定位中断向量表到 App 运行地址 (Bootloader 跳转后必须设置, Cortex-M23 支持 VTOR) */
    SCB->VTOR = OTA_APP_ADDR;


    systick_config();
    my_systick_config();
    timer1_delay_init();
    trea_uart_init();

    gpio_init();

    // PB5(V4G_EN 4G电源) 默认低电平: gpio_init 后 ODR=0 即 PB5=LOW
    // 若4G 需保持在线(long模式+活动窗口), 则在此处补一次上电, 避免 1s+ 的延时导致4G 复位
    // 弱信号模式: 4G已断电, 无需恢复PB5
    if(!voltage_low_power && !fourG_user_off && !fourG_init_failed && !fourG_weak_signal &&
       strcmp(g_window.pattern, "long") == 0 &&
       (config_is_active_window() || g_window.end_result == 1))
        power_4g_set(1);

    motor_init(MOTOR_ENABLED);
    soft_i2c_sensors_init();
    adc_bat_init();
    // PB5 (V4G_EN 4G电源) 状态由上层业务逻辑控制，此处不再重复操作
    // PA5 (M_SLP 电机使能) 由 valve_execute() 内部控制，此处不做提前处理
    // PD4 (I2C 传感器接口) 默认保持 LOW; 由采集点按需 set/reset

    debug_rx_init();   // USART0 RX中断缓冲初始化
}

// ==================== 前向声明 ====================
static void ensure_full_init(void);

// ==================== LCD信号强度更新 (4G在线时调用, 同时上报给平台) ====================
static void lcd_update_csq(void)
{
    char csq_buf[8] = {0};
	EC801E_PowerOn();
	EC801E_Wakeup();
	trea_delay_ms(120);
    if(EC801E_QueryCSQ(csq_buf, sizeof(csq_buf)) == EC_OK)
    {
        uint8_t csq_val = (uint8_t)atoi(csq_buf);
        if(g_lcd_active) lcd_ui_update_signal(csq_val);

        // 上报CSQ信号给平台
        report_publish_field("csq", csq_buf);
    }

    // 附加信息: LTE信号质量 + 网络注册状态
    {
        int rsrp = 0, sinr = 0;
        uint8_t cereg = 0;
        EC801E_QueryQCSQ(&rsrp, &sinr);
        EC801E_QueryCEREG(&cereg);
        printf_debug("[SM] Signal: CSQ=%s RSRP=%d SINR=%d CEREG=%d\r\n",
                     csq_buf, rsrp, sinr, cereg);
    }
}

// ==================== 4G 电源管理模块: 上电/唤醒 + 恢复 ====================
// 确保4G模块通道可用: 已通电则直接唤醒, 未通电则开机并初始化
// 返回: 1=连接成功, 0=失败; *p_was_online: 之前4G是否在线(供fourG_restore)
static int fourG_ensure_ready(int *p_was_online)
{
    int was_online = power_4g_is_on();
    *p_was_online = was_online;

    com_lpuart_init();
    trea_delay_ms(100);
    gpio_init();

    if(was_online)
    {
        EC801E_Wakeup();
        return (EC801E_EnsureConnected() == EC_OK);
    }
    else
    {
        fourG_user_off = 0;
        EC801E_PowerOn();
        int ok = (EC801E_InitAndConnect() == EC_OK);
        if(ok)
        {
            fourG_init_failed = 0;
            EC801E_SetPSM(0);
            trea_delay_ms(300);
            report_4g_power_state(1);
        }
        return ok;
    }
}

// 恢复4G到休眠状态:
//   success=0 (连接失败): 无论之前是否在线, 统一断电 (不再尝试PowerOn)
//   success=1 且之前在线: SetPSM(1) 恢复轻休眠保持在线
//   success=1 且之前断电: 上报下线后断电 (临时连接用完即关)
static void fourG_restore(int was_online, int success)
{
    if(!success)
    {
        EC801E_PowerOff();
        return;
    }
    if(was_online)
    {
        // long模式: 根据信号强度决定是否保持在线
        if(strcmp(g_window.pattern, "long") == 0)
        {
            char csq_buf[8] = {0};
            if(EC801E_QueryCSQ(csq_buf, sizeof(csq_buf)) == EC_OK)
            {
                uint8_t csq_val = (uint8_t)atoi(csq_buf);
                if(csq_val < CSQ_WEAK_THRESHOLD || csq_val == 99)
                {
                    printf_debug("[SM] CSQ=%d weak, power off 4G\r\n", csq_val);
                    fourG_weak_signal = 1;
                    report_4g_power_state(0);
                    EC801E_PowerOff();
                    return;
                }
                else if(fourG_weak_signal)
                {
                    printf_debug("[SM] CSQ=%d recovered, restoring long mode\r\n", csq_val);
                    fourG_weak_signal = 0;
                }
            }
        }
        EC801E_SetPSM(1);
    }
    else
    {
        // 之前断电(冷启动): 判断是否为弱信号恢复路径
        if(fourG_weak_signal && strcmp(g_window.pattern, "long") == 0)
        {
            // 弱信号断电后重新检查信号是否恢复
            char csq_buf[8] = {0};
            if(EC801E_QueryCSQ(csq_buf, sizeof(csq_buf)) == EC_OK)
            {
                uint8_t csq_val = (uint8_t)atoi(csq_buf);
                if(csq_val >= CSQ_WEAK_THRESHOLD && csq_val != 99)
                {
                    printf_debug("[SM] CSQ=%d recovered (was weak), keep 4G online\r\n", csq_val);
                    fourG_weak_signal = 0;
                    EC801E_SetPSM(1);
                    return;
                }
            }
            // 否则: 断电
            printf_debug("[SM] Still weak signal, power off\r\n");
        }
        report_4g_power_state(0);
        EC801E_PowerOff();
    }
}

// ==================== 弱信号检测+决策 (do_upload/do_downlink 收尾专用) ====================
// 上报/下行处理完后调用, 决定是否让4G去休眠
// 返回: 1=信号OK(调用方应SetPSM), 0=信号弱(内部已断电)
static int fourG_signal_check_and_decide(void)
{
    if(strcmp(g_window.pattern, "long") != 0)
        return 1;  // 非long模式不适用该逻辑

    char csq_buf[8] = {0};
    if(EC801E_QueryCSQ(csq_buf, sizeof(csq_buf)) != EC_OK)
    {
        // 查询失败按弱信号处理
        printf_debug("[SM] CSQ query failed, power off 4G\r\n");
        fourG_weak_signal = 1;
        report_4g_power_state(0);
        EC801E_PowerOff();
        return 0;
    }

    uint8_t csq_val = (uint8_t)atoi(csq_buf);
    if(csq_val < CSQ_WEAK_THRESHOLD || csq_val == 99)

    {

        printf_debug("[SM] CSQ=%d < %d, 4G power off (weak signal)\r\n", csq_val, CSQ_WEAK_THRESHOLD);

        fourG_weak_signal = 1;
        report_4g_power_state(0);
        EC801E_PowerOff();
        return 0;
    }

    // 信号恢复
    if(fourG_weak_signal)
    {
        printf_debug("[SM] CSQ=%d, signal recovered\r\n", csq_val);
        fourG_weak_signal = 0;
    }
    return 1;
}

// ==================== 电压采样模块 ====================

// 统一ADC电压采样: 读取→有效性判断→写入g_sensor.voltage
// update_baseline: 1=同时更新last_normal_voltage(正常采集), 0=仅更新voltage(报警重采等)
// fallback: ADC无效时的回退值 (通常为g_sensor.last_normal_voltage, 首次启动用3.6f)
static void sensor_sample_voltage(uint8_t update_baseline, float fallback)
{
    float v_now = adc_bat_read_voltage();
    if(v_now >= 0.0f)
    {
        g_sensor.voltage = v_now;
        if(update_baseline)
            g_sensor.last_normal_voltage = v_now;
    }
    else
        g_sensor.voltage = fallback;
}

// 异常自动关阀: 根据策略判断是否关阀, 不依赖4G通信状态
// 返回: 动作执行结果字符串(已关阀), NULL(未关阀)
// 安全说明: 此函数内部初始化IIC/电机, 确保任何调用点的均可安全执行
const char* alarm_try_close_valve(ThresholdResult_t result)
{
    AbnormalClose_t policy = abnormal_from_str(g_abnormal_close);
    /* 仅按异常关阀策略判定; 电压类报警(VPROTECT/VOLT_HIGH/VOLT_LOW)只上报不动阀 */
    if(abnormal_should_close_valve(policy, result) && !g_valve_alarm_closed)
    {
        g_valve_pre_alarm_order = g_valve_last_order;
        printf_debug("[SM] Auto-close valve (policy=%s, result=%d, pre=%d)\r\n",
                     g_abnormal_close, result, g_valve_pre_alarm_order);
    soft_i2c_sensors_init();
        motor_init(MOTOR_ENABLED);
        const char *vr = valve_execute("close", 0);
        g_valve_alarm_closed = 1;
        g_alarm_recover_reported = 0;   // 新一轮报警: 恢复上报标志复位
        config_save_ext();
        return vr;
    }
    return NULL;
}

// 激光甲烷异常自动关阀 (策略 jgclose): 报警即关阀, 关闭后保持
// 直到平台 valve_open 手动打开 (不参与压力/温度报警的自动恢复链)
// 关阀标志 EEPROM 持久化: 掉电/重启后保持关闭状态, 不会被下一次采集误判为"未关阀"
// 返回: 动作执行结果字符串(已关阀), NULL(策略未启用/已关闭)
const char* ch4_alarm_try_close_valve(void)
{
    if(abnormal_from_str(g_abnormal_close) != ABNORMAL_JGCLOSE)
        return NULL;
    if(g_ch4_alarm_closed)
        return NULL;

    g_ch4_alarm_closed = 1;
    g_alarm_recover_reported = 0;   // 新一轮报警: 恢复上报标志复位
    printf_debug("[SM] CH4 auto-close valve (policy=jgclose)\r\n");
    soft_i2c_sensors_init();
    motor_init(MOTOR_ENABLED);
    const char *vr = valve_execute("close", 0);
    config_save_ext();   // 关阀动作结束后再持久化标志: 中途掉电则重启后重新关阀, 避免半开状态被当作已关
    return vr;
}

// 用户端MQTT上报 (id=1, LY格式双通道): 连接、发送、断开 (约1分钟耗时)
static void user_mqtt_try_publish(void)
{
    if(!EC801E_UserMqtt_IsConfigured())
        return;

    trea_delay_ms(500);
    if(EC801E_UserMqtt_Connect() != EC_OK)
    {
        printf_debug("[SM] UserMqtt: connect failed, retry once...\r\n");
        trea_delay_ms(1000);
        if(EC801E_UserMqtt_Connect() != EC_OK)
        {
            printf_debug("[SM] UserMqtt: retry failed, skip publish\r\n");
            return;
        }
    }

    trea_delay_ms(300);
    if(EC801E_UserMqtt_PublishSensorData(&g_sensor.raw, &g_sensor.raw2, g_sensor.voltage) != EC_OK)
        printf_debug("[SM] UserMqtt: publish sensor data failed\r\n");
    else
        printf_debug("[SM] UserMqtt: publish OK\r\n");

    trea_delay_ms(500);
    EC801E_UserMqtt_Disconnect();
}

// ==================== 状态上报汇总模块 ====================
// 上报实测值+状态 (temp/pressure/temp_status/pressure_status [+第二路])
// last_p1/last_p2: 上次压力值(用于突变判断)
static void publish_all_status(float last_p1, float last_p2)
{
    report_publish_status_all(last_p1, last_p2);
}

// ==================== 定时阀: 执行+上报(最高优先级) ====================
// action: "open" 或 "close"
// 执行阀门动作, 通过4G上报 valve_control + valve_last_order, 恢复PSM
static void vtiming_execute_and_report(const char *action);   /* forward */
/* timing valve immediate check: run during upload/blocking as fallback;
 * executes when within +-30s window and not yet done today */
/* 日期+1 (2000-2099, 4年一闰) */
static void vtiming_next_day(RtcDateTime_t *dt)
{
    static const uint8_t mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    uint8_t maxd = mdays[dt->month - 1];

    if(dt->month == 2 && (dt->year % 4) == 0) maxd = 29;

    dt->day++;
    if(dt->day > maxd)
    {
        dt->day = 1;
        dt->month++;
        if(dt->month > 12)
        {
            dt->month = 1;
            dt->year++;
            if(dt->year > 99) dt->year = 0;   /* 2000年后回卷 */
        }
    }
}

/* 计算距离下一个匹配事件的时间 (YYMMDDHHMM, 支持通配); 无匹配返回 0xFFFFFFFF */
static uint32_t vtiming_next_wait(const char *t, const RtcDateTime_t *now)
{
    RtcDateTime_t cand;
    uint16_t days;
    uint32_t tgt_sec, now_sec;
    uint8_t y, mo, d, h, mi;

    if(!valve_timing_time_valid(t)) return 0xFFFFFFFF;

    y  = (uint8_t)((t[0] - '0') * 10 + (t[1] - '0'));
    mo = (uint8_t)((t[2] - '0') * 10 + (t[3] - '0'));
    d  = (uint8_t)((t[4] - '0') * 10 + (t[5] - '0'));
    h  = (uint8_t)((t[6] - '0') * 10 + (t[7] - '0'));
    mi = (uint8_t)((t[8] - '0') * 10 + (t[9] - '0'));

    tgt_sec = ((uint32_t)h * 60 + mi) * 60;
    now_sec = ((uint32_t)now->hour * 60 + now->minute) * 60 + now->second;

    cand = *now;
    for(days = 0; days <= 366; days++)
    {
        if((y == 0 || y == cand.year) &&
           (mo == 0 || mo == cand.month) &&
           (d == 0 || d == cand.day))
        {
            /* 目标已过 则从明天开始算 */
            if(days == 0 && tgt_sec <= now_sec) { }
            else
                return (uint32_t)days * 86400 + tgt_sec - now_sec;
        }
        vtiming_next_day(&cand);
    }
    return 0xFFFFFFFF;
}

/* timing valve immediate check: run during upload/blocking as fallback;
 * executes when date matches (wildcard) and within tolerance */
static void vtiming_check_now(void)
{
    RtcDateTime_t dt;

    /* 消费定时阀挂起事件: RTC ISR 置位的 g_timing_pending 必须在此清零,
     * 否则 EC_SendCmd 永久返回 EC_PREEMPT (4G 通信失效, do_upload 死循环) */
    g_timing_pending = 0;

    if(!g_valve_timing.on_switch && !g_valve_timing.off_switch) return;

    rtc_get_datetime(&dt);

    /* 跨天重置当日执行标志 */
    if(dt.hour == 0 && timing_last_reset_hour != 0)
    {
        timing_open_done_today  = 0;
        timing_close_done_today = 0;
        timing_last_reset_hour  = 0;
    }
    else if(dt.hour != 0)
        timing_last_reset_hour = dt.hour;

    /* 开阀: 定时开 + 当日未执行 + 时间匹配(±30s) */
    if(!timing_open_done_today && g_valve_timing.on_switch &&
       valve_timing_time_match(g_valve_timing.open_time, &dt, 30))
    {
        vtiming_execute_and_report("open");
        timing_open_done_today = 1;
    }

    /* 关阀 */
    if(!timing_close_done_today && g_valve_timing.off_switch &&
       valve_timing_time_match(g_valve_timing.close_time, &dt, 30))
    {
        vtiming_execute_and_report("close");
        timing_close_done_today = 1;
    }
}

static void vtiming_execute_and_report(const char *action)
{
    g_timing_pending = 0;   /* 执行前消费挂起事件, 确保 4G 上报不被 EC_PREEMPT 拦截 */
    /* 报警自动关阀/激光甲烷关阀未恢复前禁止自动开阀 (工业安全) */
    if(strcmp(action, "open") == 0 && (g_ch4_alarm_closed || g_valve_alarm_closed))
    {
        printf_debug("[SM] VTIMING: open skipped (alarm latch active)\r\n");
        return;
    }
    ensure_full_init();
    soft_i2c_sensors_init();
    motor_init(MOTOR_ENABLED);
    const char *vr = valve_execute(action, 1);
    printf_debug("[SM] VTIMING: %s result=%s\r\n", action, vr);

    int was_online;
    int ec_ok = fourG_ensure_ready(&was_online);
    if(ec_ok)
        EC801E_PublishValveStatus(vr, g_valve_last_order);
    fourG_restore(was_online, ec_ok);
}

// ==================== 实时路径: 按用户选择进入休眠 ====================
static void realtime_enter_sleep(void)
{
    uint8_t lpuart_enabled = 0;

    sensor_pwr_off();   /* PC3 拉低: 传感器断电 */
    rtc_alarm_wakeup_init();
    rtc_set_next_alarm_sec(g_real_frequency);
    g_timing_alarm_set = 0;   /* realtime alarm, timing handled per-cycle */

    // 4G在线时启用LPUART唤醒 + RI中断 (可接收平台下发, 弱信号断电时禁用)
    if(!voltage_low_power && !fourG_user_off && !fourG_init_failed && !fourG_weak_signal &&
       strcmp(g_window.pattern, "long") == 0 &&
       (config_is_active_window() || g_window.end_result == 1))
    {
        com_lpuart_init();
        lpuart_wakeup_init();
        ri_exti_init();
        lpuart_enabled = 1;
    }

    button_exti_init();
    if(g_bt_on) ble_exti_init();
    lpuart_dma_rx_disable();
    {
        PmSleepCfg_t sc = { lpuart_enabled, g_lcd_active, g_bt_on };
        enter_deepsleep_mode(&sc);
    }
}

// ==================== 实时采集最小唤醒路径 ====================
// 最小化初始化 SysTick + I2C + PD4, 采集判断异常
// 异常时重采确认, 确认后通过4G上报; 正常时直接回睡
// ==================== ==================== ====================
static void do_realtime_check(void)
{
    ThresholdResult_t result;
    ThresholdResult_t r1, r2;
    uint8_t sensor_err;
    uint8_t still_abnormal = 1;
    uint8_t i;

    // === 最小初始化 (时钟恢复) ===
    system_wakeup_reinit();

    // 软件看门狗: 实时采集路径由 do_sleep 循环直接调用(非主循环入口), 需自行使能
    // SysTick 由 system_wakeup_reinit 重新启动; 休眠时 enter_deepsleep_mode 会关闭
    swdt_enable(SWDT_TIMEOUT_MS);

    // === 采集 (双通道, 不采ADC; PD4/IIC 由 CollectAll 内部处理) ===
    PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);

    // === LCD超时检查 + 采集后刷新显示 ===
    lcd_check_timeout();
#if LCD_LOCK_ENABLED
    if(g_lcd_active) lcd_ui_refresh(&g_sensor.raw, &g_sensor.raw2, g_sensor.voltage);
#else
    /* LCD常亮模式: 每个采集周期自动翻一页 */
    if(g_lcd_active) { lcd_ui_refresh(&g_sensor.raw, &g_sensor.raw2, g_sensor.voltage); lcd_ui_next_page(); }
#endif

    // === 传感器在线/离线状态变化检测(带消抖, 双路在线/离线) ===
    {
        uint8_t cur_online  = (g_sensor.raw.err  == PT304D_OK) ? 1 : 0;
        uint8_t cur_online2 = (g_sensor.raw2.err == PT304D_OK) ? 1 : 0;
        uint8_t state_changed = sensor_debounce_update(cur_online, cur_online2);

        if(state_changed)
        {
            ensure_full_init();
            trea_uart_init();
            {
                char rt_ts[32];
                rtc_get_time_str(rt_ts, sizeof(rt_ts));
                printf_debug("[RT] %s SENSOR STATE CHANGED: S2=%s(err=%d) S1=%s(err=%d) freq=%ds\r\n",
                             rt_ts,
                             cur_online  ? "ON" : "OFF", g_sensor.raw.err,
                             cur_online2 ? "ON" : "OFF", g_sensor.raw2.err,
                             g_real_frequency);
            }

            // 启动4G上报
            {
                int was_online;
                int ec_ok = fourG_ensure_ready(&was_online);
                if(ec_ok)
                {
                    report_publish_sensor_status();
                    printf_debug("[RT] Sensor state reported\r\n");
                }
                fourG_restore(was_online, ec_ok);
            }

            // 传感器离线时不参与阈值检查, 直接回睡
            if(!cur_online)
            {
                g_sensor.last_pressure_realtime = g_sensor.raw.pressure;
                g_sensor.last_pressure_realtime2 = g_sensor.raw2.pressure;
                realtime_enter_sleep();
                return;
            }
        }
        else if(!cur_online)
        {
            // 传感器在线(异常期间), 不重复上报, 直接回睡
            realtime_enter_sleep();
            return;
        }
    }

    // === 阈值异常检查 ===
    // 实时路径不采 ADC: 电压占位-1.0f (配合 check_voltage=0 + threshold_check 跳过) 表示"未采样电压"
    // skip_mutation=1: 恢复判断时跳过突变项, 防止压力回正常时瞬间变化触发突变报警
    r1 = threshold_check(g_sensor.raw.pressure, g_sensor.raw.temperature, -1.0f, g_sensor.last_pressure_realtime, 0, 1);
    r2 = THRESHOLD_OK;
    if(g_sensor_count >= 2 && g_sensor.raw2.err == PT304D_OK)
        r2 = threshold_check2(g_sensor.raw2.pressure, g_sensor.raw2.temperature, g_sensor.last_pressure_realtime2, 1);
    result = (r1 != THRESHOLD_OK) ? r1 : r2;   /* 有效报警: 通道1优先 (关阀/上报/恢复) */
    sensor_err = (g_sensor.raw.err != PT304D_OK);

    // === 正常 → 检查恢复上报 → 恢复报警状态 → 直接回睡 ===
    if(result == THRESHOLD_OK && !sensor_err)
    {
        // 报警关阀后恢复正常: 只上报一次, 不自动操作阀门 (工业安全)
        if(g_valve_alarm_closed && !g_ch4_alarm_closed && !voltage_low_power)
        {
            if(!g_alarm_recover_reported)
            {
                g_alarm_recover_reported = 1;
                ensure_full_init();
                int was_online;
                int ec_ok = fourG_ensure_ready(&was_online);
                if(ec_ok)
                {
                    EC801E_PublishValveStatus(valve_get_status(), g_valve_last_order);
                    report_publish_ly06(0, g_sensor.last_normal_voltage);
                    publish_all_status(g_sensor.last_pressure_realtime, g_sensor.last_pressure_realtime2);
                }
                fourG_restore(was_online, ec_ok);
            }
        }
        else if(!alarm_state_is_normal(&s_last_rt_state))
        {
            // 之前有报警但未关阀(策略为none时), 现恢复为正常, 上报恢复状态
            ensure_full_init();
            int was_online;
            int ec_ok = fourG_ensure_ready(&was_online);
            if(ec_ok)
            {
                report_publish_ly06(0, g_sensor.last_normal_voltage);
                publish_all_status(g_sensor.last_pressure_realtime, g_sensor.last_pressure_realtime2);
            }
            fourG_restore(was_online, ec_ok);
        }

        if(!alarm_state_is_normal(&s_last_rt_state))
            alarm_state_reset(&s_last_rt_state);   // 恢复完成, 防止重复上报

        // 动态阈值再次检查, 包含突变项 (skip_mutation=0)
        // 恢复上报结束后, 此处再确认是否触发新的突变异常
        r1 = threshold_check(g_sensor.raw.pressure, g_sensor.raw.temperature, -1.0f, g_sensor.last_pressure_realtime, 0, 0);
        r2 = THRESHOLD_OK;
        if(g_sensor_count >= 2 && g_sensor.raw2.err == PT304D_OK)
            r2 = threshold_check2(g_sensor.raw2.pressure, g_sensor.raw2.temperature, g_sensor.last_pressure_realtime2, 0);
        result = (r1 != THRESHOLD_OK) ? r1 : r2;

        g_sensor.last_pressure_realtime = g_sensor.raw.pressure;
        g_sensor.last_pressure_realtime2 = g_sensor.raw2.pressure;

        if(result == THRESHOLD_OK)
        {
            realtime_enter_sleep();
            return;
        }
        // 突变异常 → 继续走下方异常确认流程
    }

    // === 异常 → 去重: 温度/压力状态与上次一致不重复 (温度单独, 不叠加压力等级) ===
    {
        ThresholdAlarmState_t st;
        alarm_state_fill(&st, r1, r2);
        if((r1 != THRESHOLD_OK || r2 != THRESHOLD_OK) &&
           memcmp(&st, &s_last_rt_state, sizeof(st)) == 0)
        {
            g_sensor.last_pressure_realtime = g_sensor.raw.pressure;
            g_sensor.last_pressure_realtime2 = g_sensor.raw2.pressure;
            realtime_enter_sleep();
            return;
        }
    }

    // === 异常 → 走重采确认 (3次) ===
    for(i = 0; i < APP_ALARM_RECHECK_COUNT; i++)
    {
        trea_delay_ms(APP_ALARM_RECHECK_DELAY_MS);
        PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);

        r1 = threshold_check(g_sensor.raw.pressure, g_sensor.raw.temperature, -1.0f, g_sensor.last_pressure_realtime, 0, 0);
        r2 = THRESHOLD_OK;
        if(g_sensor_count >= 2 && g_sensor.raw2.err == PT304D_OK)
            r2 = threshold_check2(g_sensor.raw2.pressure, g_sensor.raw2.temperature, g_sensor.last_pressure_realtime2, 0);
        result = (r1 != THRESHOLD_OK) ? r1 : r2;
        sensor_err = (g_sensor.raw.err != PT304D_OK);

        if(result == THRESHOLD_OK && !sensor_err)
        {
            still_abnormal = 0;
            break;
        }
    }

    if(!still_abnormal)
    {
        // 偶发异常 → 回睡
        g_sensor.last_pressure_realtime = g_sensor.raw.pressure;
        g_sensor.last_pressure_realtime2 = g_sensor.raw2.pressure;
        realtime_enter_sleep();
        return;
    }

    // === 确认异常 → 记录报警状态用于去重上报 (含双路温度, 压力不叠加温度变化) ===
    alarm_state_fill(&s_last_rt_state, r1, r2);

    // === 异常关阀 (安全动作, 不依赖4G通信) ===
    {
        const char *alarm_type = threshold_alarm_type_str(result);
        if(sensor_err && result == THRESHOLD_OK)
            alarm_type = "sensor_offline";

        ensure_full_init();
        trea_uart_init();
        printf_debug("[RT] ALARM: %s P=%.2f T=%.2f freq=%ds\r\n",
                     alarm_type, g_sensor.raw.pressure, g_sensor.raw.temperature, g_real_frequency);

        const char *vr = alarm_try_close_valve(result);

        // === 4G上报 (报警状态+关阀结果) ===
        int was_online;
        int ec_ok = fourG_ensure_ready(&was_online);
        if(ec_ok)
        {
            if(r1 != THRESHOLD_OK || r2 != THRESHOLD_OK)
            {
                /* 一条消息包含全部通道的温度+压力状态 */
                report_publish_alarm_batch(g_sensor.last_pressure_realtime, g_sensor.last_pressure_realtime2);
                /* 低压报警再补一条 (电压B) */
                if(r1 == THRESHOLD_VPROTECT || r1 == THRESHOLD_VOLT_HIGH || r1 == THRESHOLD_VOLT_LOW)
                    report_publish_ly06(1, g_sensor.last_normal_voltage);
            }
            if(sensor_err)
                report_publish_sensor_status();
            if(vr)
                EC801E_PublishValveStatus(vr, g_valve_last_order);

            // 用户端MQTT同步上报传感器数据
            user_mqtt_try_publish();
        }
        fourG_restore(was_online, ec_ok);
    }

    // === 回睡 ===
    g_sensor.last_pressure_realtime = g_sensor.raw.pressure;
    g_sensor.last_pressure_realtime2 = g_sensor.raw2.pressure;
    realtime_enter_sleep();
}

// ==================== 休眠流程 ====================
static void do_sleep(void)
{
    int fourG_keep_on = 0;


    motor_sleep_disable();   /* M_SLP(PA5) 拉低: 电机禁用 */
    sensor_pwr_off();   /* PC3 拉低: 传感器断电 */

    gpio_bit_reset(GPIOA, GPIO_PIN_2);
    gpio_bit_reset(GPIOA, GPIO_PIN_3);
    gpio_bit_reset(GPIOC, GPIO_PIN_4);
    gpio_bit_reset(GPIOC, GPIO_PIN_5);
    gpio_bit_reset(GPIOB, GPIO_PIN_8);
    // PB9 = SLCD COM3, 保持关闭
    // PB6/PB7(I2C0 SCL/SDA): 常供电总线, 由 gpio_enter_low_power 转 Analog
    // 避免休眠期间总线上残留异常 I2C 信号导致功耗异常

#if MOTOR_DRV_MODE == 0
    gpio_bit_reset(GPIOA, GPIO_PIN_6);
    gpio_bit_reset(GPIOA, GPIO_PIN_7);
#endif

    // 4G电源控制: 根据window pattern决定PB5(V4G_EN)状态
    // 低电压省电模式: 强制4G断电
    if(voltage_low_power)
    {
        power_4g_set(0);
    }
    else if(strcmp(g_window.pattern, "long") == 0)
    {
        // long模式: 活动窗口或end_result=1时保持4G (用户手动关闭/弱信号除外)
        if(!fourG_user_off && !fourG_init_failed && !fourG_weak_signal && (config_is_active_window() || g_window.end_result == 1))
            fourG_keep_on = 1;
        else
            power_4g_set(0);  // long+er=0+非窗口: user_off/weak_signal: 4G断电
    }
    else
    {
        // short/dot: 4G始终断电(由do_upload上报时临时PowerOn后PowerOff)
        power_4g_set(0);
    }

#if LCD_LOCK_ENABLED
    // === LCD 超时关闭: LCD处于显示时, 先短睡, 60秒超时后再关闭 ===
    // 实时模式(1-10s频率): do_realtime_check + lcd_check_timeout() 会处理, 此处跳过
    if(g_lcd_active && !g_real_enabled)
    {
        uint32_t now_sec = hhmmss_to_seconds(rtc_get_hhmmss());
        uint32_t on_sec  = hhmmss_to_seconds(g_lcd_on_hhmmss);
        uint32_t elapsed = (now_sec >= on_sec) ? (now_sec - on_sec) : (86400 - on_sec + now_sec);

		
        if(elapsed < LCD_DISPLAY_TIMEOUT_SEC)
        {
			
            uint32_t remain = LCD_DISPLAY_TIMEOUT_SEC - elapsed;
            if(remain < 1) remain = 1;

            rtc_alarm_wakeup_init();
            rtc_set_next_alarm_sec((uint8_t)remain);
            g_timing_alarm_set = 0;   /* LCD timeout sleep: normal alarm */
            button_exti_init();
            if(g_bt_on) ble_exti_init();
            if(fourG_keep_on) { com_lpuart_init(); lpuart_wakeup_init(); ri_exti_init(); }
            lpuart_dma_rx_disable();

            printf_debug("[SM] LCD timeout sleep %ds\r\n", (int)remain);
            trea_delay_ms(20);
            {
                PmSleepCfg_t sc = { (uint8_t)fourG_keep_on, g_lcd_active, g_bt_on };
                enter_deepsleep_mode(&sc);
            }

            // 唤醒: 按键/LPUART/RI/BLE事件 → 直接进入 STATE_WAKEUP
            if(button_wakeup_flag || counter0 || ri_wakeup_flag || ble_connect_wakeup_flag)
            {
                app_state = STATE_WAKEUP;
                return;
            }
        }
        // RTC 超时唤醒后继续: 关闭 LCD
        lcd_shutdown();
        printf_debug("[SM] LCD off after timeout\r\n");
    }
#endif /* LCD_LOCK_ENABLED */

    // 重设RTC闹钟: 计算好下次采集时刻, 防止超时唤醒错过窗口
    rtc_alarm_wakeup_init();

    // 定时阀门: 计算最近一个未执行的阀门事件距当前的时间
    // 定时阀门: 计算最近一个未执行的阀门事件距当前的时间
    {
        uint8_t use_valve_alarm = 0;
        uint32_t valve_wait_sec = 0xFFFFFFFF;

        if((g_valve_timing.on_switch || g_valve_timing.off_switch) && !g_real_enabled)
        {
            RtcDateTime_t dt;
            rtc_get_datetime(&dt);

            if(!timing_open_done_today && g_valve_timing.on_switch)
            {
                uint32_t w = vtiming_next_wait(g_valve_timing.open_time, &dt);
                if(w < valve_wait_sec) valve_wait_sec = w;
            }
            if(!timing_close_done_today && g_valve_timing.off_switch)
            {
                uint32_t w = vtiming_next_wait(g_valve_timing.close_time, &dt);
                if(w < valve_wait_sec) valve_wait_sec = w;
            }

            // 阀门事件更早: 可等待到目标时间, 直接设定秒级闹钟
            uint32_t collect_sec = (uint32_t)g_interval.collect_interval_min * 60;
            if(valve_wait_sec < collect_sec && valve_wait_sec < 0xFFFFFFFF)
            {
                // 计算目标绝对时间(可能跨天), 换算为HHMMSS
                uint32_t t_abs = ((uint32_t)dt.hour * 60 + dt.minute) * 60 + dt.second + valve_wait_sec;
                uint32_t target_hhmmss;
                t_abs %= 86400;
                target_hhmmss = (t_abs / 3600) * 10000 + ((t_abs / 60) % 60) * 100 + (t_abs % 60);

                rtc_set_alarm_at_hhmmss(target_hhmmss);
                g_timing_alarm_set = 1;   /* timing valve alarm */
                use_valve_alarm = 1;
            }
        }

        if(!use_valve_alarm)
        {
            if(g_real_enabled)
                rtc_set_next_alarm_sec(g_real_frequency);
            else
                rtc_set_next_alarm(g_interval.collect_interval_min);
            g_timing_alarm_set = 0;   /* normal collect alarm */
        }
    }

    // LPUART唤醒 + RI中断: 4G在线时允许接收(可接收下行URC)
    if(fourG_keep_on)
    {
        com_lpuart_init();
        lpuart_wakeup_init();
        ri_exti_init();
    }

    // PA12按键唤醒: 始终使能(可手动唤醒4G)
    button_exti_init();

    // BLE连接事件未触发(与按键同时到达), 回到WAKEUP处理
    if(ble_connect_wakeup_flag)
    {
        app_state = STATE_WAKEUP;
        return;
    }

    // BLE超时: 超过60s未连接超时, 未超时则使用PD2 EXTI等待
    if(g_bt_on)
    {
        uint32_t now_sec = hhmmss_to_seconds(rtc_get_hhmmss());
        uint32_t on_sec  = hhmmss_to_seconds(g_ble_on_hhmmss);
        uint32_t elapsed = (now_sec >= on_sec) ? (now_sec - on_sec) : (86400 - on_sec + now_sec);
        if(elapsed >= 60)
        {
            printf_debug("[SM] BLE timeout (%ds), auto power off\r\n", (int)elapsed);
            g_bt_on = 0;
            power_ble_set(0);
            if(g_lcd_active) slcd_seg_icon_display(SLCD_ICON_LY, RESET);
        }
        else
        {
            ble_exti_init();
        }
    }

    // 关闭接收DMA(休眠期间DMA时钟被关闭, 唤醒后重新配置)
    lpuart_dma_rx_disable();

    if(g_real_enabled)
        printf_debug("[SM] SLEEP %ds-RT (buf=%d 4G=%s BLE=%d)\r\n",
                     g_real_frequency, ee_hist_count(),
                     fourG_keep_on ? "ON" : "OFF", g_bt_on);
    else
        printf_debug("[SM] SLEEP %dmin (buf=%d 4G=%s BLE=%d)\r\n",
                     g_interval.collect_interval_min, ee_hist_count(),
                     fourG_keep_on ? "ON" : "OFF", g_bt_on);
    trea_delay_ms(20);

    {
        PmSleepCfg_t sc = { (uint8_t)fourG_keep_on, g_lcd_active, g_bt_on };
        enter_deepsleep_mode(&sc);
    }

    // 实时采集循环: RTC秒级唤醒 → 判断是否到采集分钟 → 否则最小实时采集→睡
    while(g_real_enabled && rtc_alarm_flag && !counter0 && !ri_wakeup_flag && !button_wakeup_flag && !ble_connect_wakeup_flag)
    {
        // 最小时钟恢复 → 查RTC中断是否到达网格采集时刻
        system_wakeup_reinit();
        rtc_register_sync_wait();

        {
            uint32_t now_hhmmss = rtc_get_hhmmss();
            uint16_t cur_min = (uint16_t)(now_hhmmss / 10000) * 60
                             + (uint16_t)((now_hhmmss / 100) % 100);
            uint8_t cur_sec = (uint8_t)(now_hhmmss % 100);

            // 网格采集时刻到 (分钟+5s容差, 且当前分钟未执行过采集 → 退出循环, 转入STATE_WAKEUP
            if(g_interval.collect_interval_min > 0 &&
               (cur_min % g_interval.collect_interval_min) == 0 &&
               cur_sec < 6 &&
               cur_min != last_grid_collect_min)
                break;

            // ---- 实时模式下的定时阀门检查 ----
            // ---- 实时模式下的定时阀门检查 ----
            if(g_valve_timing.on_switch || g_valve_timing.off_switch)
            {
                RtcDateTime_t dt;
                rtc_get_datetime(&dt);

                // 跨天重置当日执行标志
                if(dt.hour == 0 && timing_last_reset_hour != 0)
                {
                    timing_open_done_today  = 0;
                    timing_close_done_today = 0;
                    timing_last_reset_hour  = 0;
                }
                else if(dt.hour != 0)
                    timing_last_reset_hour = dt.hour;

                // 检查开阀 (容差5秒, 实时模式频率高于30s)
                if(!timing_open_done_today && g_valve_timing.on_switch &&
                   valve_timing_time_match(g_valve_timing.open_time, &dt, 5))
                {
                    printf_debug("[SM] VTIMING-RT: OPEN at %02u:%02u\r\n", dt.hour, dt.minute);
                    vtiming_execute_and_report("open");
                    timing_open_done_today = 1;
                }
                // 检查关阀 (容差5秒)
                if(!timing_close_done_today && g_valve_timing.off_switch &&
                   valve_timing_time_match(g_valve_timing.close_time, &dt, 5))
                {
                    printf_debug("[SM] VTIMING-RT: CLOSE at %02u:%02u\r\n", dt.hour, dt.minute);
                    vtiming_execute_and_report("close");
                    timing_close_done_today = 1;
                }
            }

#if LASER_CH4_ENABLED
            // ---- 实时模式下的激光甲烷定时采集检查 ----
            // 命中时间点 → 置 g_ch4_request 并退出实时循环;
            // 由主循环在网格采集/整点上传完成后执行 STATE_CH4 (不抢占采集)
            if(g_laser_switch &&
               ((uint8_t)(now_hhmmss / 10000)) != s_laser_done_hour &&
               window_dottime_hit(g_laser_time_mask, (uint8_t)(now_hhmmss / 10000)))
            {
                printf_debug("[SM] LASER schedule trigger at %02u:00\r\n", (uint8_t)(now_hhmmss / 10000));
                g_ch4_request = 1;
                break;
            }
#endif
        }

        // 非采集分钟 → 最小实时采集 (内部: 采集→判断→[异常上报]→上电→进入睡眠→唤醒后返回)
        do_realtime_check();
        // do_realtime_check 返回后, 新的唤醒标志已被复位, while再次判断
    }

    // 唤醒后统一转入 WAKEUP 状态
    app_state = STATE_WAKEUP;
}

// ==================== 按键短按流程处理模块 ====================
// 按键短按 PA12小于1秒释放时触发：打印配置、采集实时数据并上报，同时等待 4G 平台下发返回状态
static void do_button_short_press(void)
{
    printf_debug("\r\n==================================================\r\n");
    printf_debug("[SM] BUTTON SHORT PRESS: Starting manual report & config dump\r\n");

    // 1. 打印当前系统配置
    config_print_current();
    
    // 2. 采集当前实时传感器数据并打印
    printf_debug("[SM] Sampling live sensor data...\r\n");
    PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);
    
    sensor_sample_voltage(1, g_sensor.last_normal_voltage);
    
    printf_debug("--- Live Data ---\r\n");
    printf_debug("  S2(inlet):  P=%.2f Pa  T=%.2f C  err=%d\r\n", g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.raw.err);
    printf_debug("  S1(outlet): P=%.2f Pa  T=%.2f C  err=%d\r\n", g_sensor.raw2.pressure, g_sensor.raw2.temperature, g_sensor.raw2.err);
    printf_debug("  Voltage:    %.2f V\r\n", g_sensor.voltage);
    printf_debug("==================================================\r\n\r\n");

    // 3. 4G上报 (使用通用模块)
    int was_online;
    int ec_ok = fourG_ensure_ready(&was_online);

    if(ec_ok)
    {
        lcd_update_csq();

        // A. 上报传感器实时数据
        EC801E_PublishSensorData(&g_sensor.raw, g_sensor.voltage);
        trea_delay_ms(700);

        // B. 上报传感器在线状态
        report_publish_sensor_status();
        trea_delay_ms(700);

        // C. 上报压力+温度状态 (使用通用模块)
        publish_all_status(g_sensor.last_pressure_collect, g_sensor.last_pressure_collect2);
        trea_delay_ms(700);

        // D. 上报阀门状态指令
        {
            char v_order[4];
            snprintf(v_order, sizeof(v_order), "%d", g_valve_last_order);
            EC801E_PublishResponse("valve_last_order", v_order);
        }
        trea_delay_ms(700);

        // E. 用户端MQTT上报
        user_mqtt_try_publish();

        printf_debug("[SM] BUTTON SHORT PRESS: Manual status reporting success!\r\n");
    }
    else
    {
        printf_debug("[SM] BUTTON SHORT PRESS: manual report FAILED (MQTT connection failed)\r\n");
    }

    // 4. 等待按键释放, 防止释放抖动再次触发
    while(button_is_pressed())
        trea_delay_ms(100);
    trea_delay_ms(50);

    // 5. 结尾再等待5s 接收平台下发指令
    printf_debug("[SM] BUTTON: waiting 5s for downlink...\r\n");
    trea_delay_ms(5000);
    {
        uint8_t dl_count = drain_pending_downlink();
        if(dl_count > 0)
            printf_debug("[SM] BUTTON: processed %d downlink msg(s)\r\n", dl_count);
        else
            printf_debug("[SM] BUTTON: no downlink within 5s\r\n");
    }

    // 6. 恢复4G状态 (使用通用模块)
    fourG_restore(was_online, ec_ok);
}

// ==================== 唤醒恢复 ====================
static void do_wakeup(void)
{
    // 判断是否为纯RTC网格采集唤醒 (非按键且非LPUART且非RI且非BLE唤醒)
    if(button_wakeup_flag == 0 && counter0 == 0 && ri_wakeup_flag == 0 && ble_connect_wakeup_flag == 0)
    {
        g_is_minimal_wakeup = 1;
        g_need_adc_sampling = 0;
        
        system_wakeup_reinit();
        
        // 最小采集初始化: Systick和Debug 串口未启动, 仅初始化 I2C 以及 PC3 电源由 PT304D_CollectAll 内部处理
        systick_config();
//        my_systick_config();
        trea_uart_init();
        
        if(g_need_adc_sampling)
        {
            adc_bat_init();
        }
        
        button_exti_disable();
        
        rtc_show_time();
        printf_debug("[SM] PURE RTC WAKEUP: minimal init (ADC=%d)\r\n", g_need_adc_sampling);
    }
    else if(button_wakeup_flag && counter0 == 0 && ri_wakeup_flag == 0)
    {
        // === 按键唤醒 (快速检测路径: 最小初始化, 不执行 sys_init() 约1s 延时 ===
        // GPIO 状态在休眠期间保持不变, PD0 作为按键输入, button_is_pressed() 可直接使用
        // 需要完整功能时由 ensure_full_init() 升级处理
        g_is_minimal_wakeup = 1;
        g_need_adc_sampling = 1;
        
        system_wakeup_reinit();
        trea_uart_init();
		buzzer_init();
        button_exti_disable();
        
        rtc_show_time();
        printf_debug("[SM] WAKEUP: BUTTON (fast detect)\r\n");
    }
    else
    {
        // === LPUART/RI 唤醒或恢复会话: 完整初始化 ===
        g_is_minimal_wakeup = 0;
        g_need_adc_sampling = 1;
        
        system_wakeup_reinit();
        sys_init();
        
        // 禁用按键EXTI(唤醒后立即禁用防止抖动重复触发)
        button_exti_disable();
        
        // PB5(V4G_EN 4G电源)恢复: gpio_init默认PB5=LOW
        // 低电压省电模式: 不恢复4G
        // 用户手动关闭: 不恢复4G(等待用户再次打开)
        // 弱信号模式: 不恢复4G(下次上报时再尝试)
        // long模式: 活动窗口或end_result=1时恢复4G供电
        if(!voltage_low_power && !fourG_user_off && !fourG_init_failed && !fourG_weak_signal &&
           strcmp(g_window.pattern, "long") == 0 &&
           (config_is_active_window() || g_window.end_result == 1))
            power_4g_set(1);
        // 其他情况(VLP/user_off/weak_signal/short/dot/long+er=0+非窗口): PB5保持LOW
        
        rtc_show_time();
        printf_debug("\r\n");
    }

    // === 按键唤醒: 检测按键 + 启动4G电源 ===
    if(button_wakeup_flag)
    {
        printf_debug("[SM] WAKEUP: BUTTON\r\n");

        // 按键唤醒恢复LCD显示 (唤醒后关闭需重新初始化)
        uint8_t lcd_was_off = !g_lcd_active;
        lcd_activate();
        if(g_lcd_active)
            lcd_ui_refresh(&g_sensor.raw, &g_sensor.raw2, g_sensor.voltage);

        button_wakeup_flag = 0;

        // LCD从关闭恢复: 首次按键唤醒点亮屏幕, 完成UI初始化(翻页)
        if(lcd_was_off && g_lcd_active)
        {
            buzzer_beep(1, 200);
            printf_debug("[SM] BUTTON: LCD activated (consume press)\r\n");
            app_state = STATE_SLEEP;
            return;
        }

        // 按键抖动: 20ms等待消除前抖动
        trea_delay_ms(20);

        // 确认当前按键状态
        uint8_t is_short_press = 0;

        if(!button_is_pressed())
        {
            // 20ms内已松开 → 短按
            is_short_press = 1;
        }
        else
        {
            // 按键仍按住, 检查是否长按4s)
            uint8_t i;
            for(i = 0; i < 40; i++)
            {
                trea_delay_ms(100);
                if(!button_is_pressed())
                {
                    // 4s内松开 → 短按
                    is_short_press = 1;
                    break;
                }
            }
        }

        if(is_short_press)
        {
            // === 短按: 统计连续按键次数 (500ms窗口内) ===
            uint8_t press_count = 1;
            uint16_t wait_ms;

            // 等待按键释放 (消除抖动)
            while(button_is_pressed())
                trea_delay_ms(20);
            trea_delay_ms(30);  // 按键稳定

            // 500ms窗口内继续检测下一次按键
            while(press_count < 4)
            {
                uint8_t got_press = 0;
                for(wait_ms = 0; wait_ms < 500; wait_ms += 20)
                {
                    trea_delay_ms(20);
                    if(button_is_pressed())
                    {
                        got_press = 1;
                        break;
                    }
                }

                if(!got_press)
                    break;  // 500ms内无新按键, 结束统计

                press_count++;
                // 等待本次按键释放
                while(button_is_pressed())
                    trea_delay_ms(20);
                trea_delay_ms(30);  // 按键稳定
            }

            printf_debug("[SM] BUTTON: %d press(es) detected\r\n", press_count);

            switch(press_count)
            {
                
                case 1:
                    // 1次短按: LCD页切换 + 按键确认
                    buzzer_beep(1, 200);
                    if(g_lcd_active) lcd_ui_next_page();
                        printf_debug("[SM] BUTTON: LCD page -> %d\r\n", g_lcd_page);
                    break;

                case 2:
                    // 2次短按: 手动上报 (完整初始化)
                    ensure_full_init();
                    do_button_short_press();
                    break;

                case 3:
                    // 3次短按: 蓝牙开关切换 (控制: 开/关, 不进入BLE_OTA)
                    if(g_lcd_active) lcd_ui_toggle_bluetooth();
                    buzzer_beep(1, 200);
                    printf_debug("[SM] BUTTON: Bluetooth %s\r\n", g_bt_on ? "ON" : "OFF");
                    if(g_bt_on)
                    {
                        ble_link_gpio_init();           /* PD2输入+上拉 */
                        ble_set_name_from_device_id();  /* AT指令设置BLE名称 */
                        g_ble_on_hhmmss = rtc_get_hhmmss(); /* 记录开启时间, 用于60s超时 */
                        printf_debug("[SM] BLE powered on, waiting connect via EXTI\r\n");
                    }
                    else
                    {
                        ble_exti_disable();
                        ble_uart_deinit();
                        printf_debug("[SM] BLE powered off\r\n");
                    }
                    break;
				case 4:
                    // 4次短按: 打开4G
                    fourG_user_off = 0;
                    fourG_weak_signal = 0;  // 清除弱信号标志
                    /* 4x short press: really power on 4G and init the module.
                     * Old code only cleared flags; PB5(V4G_EN) stayed LOW so
                     * do_sleep treated 4G as online while it was powered off. */
                    ensure_full_init();
                    com_lpuart_init();
                    trea_delay_ms(100);
                    EC801E_PowerOn();
                    if(EC801E_InitAndConnect() == EC_OK)
                    {
                        fourG_init_failed = 0;
                        report_4g_power_state(1);
                        lcd_update_csq();
                        buzzer_beep(2, 200);
                        printf_debug("[SM] BUTTON: 4G powered on OK\r\n");
                    }
                    else
                    {
                        printf_debug("[SM] BUTTON: 4G power on FAILED (no AT response)\r\n");
                    }
                    break;

                default:
                    break;
            }

            app_state = STATE_SLEEP;
            return;
        }

        // === 长按4s确认 → 4G电源开启(强制打开) ===
        ensure_full_init();
        printf_debug("[SM] BUTTON: 4s confirmed, powering on 4G\r\n");
        fourG_user_off = 0;  // 清除用户关闭标志
        fourG_weak_signal = 0;  // 手动打开同时清除弱信号标志
        com_lpuart_init();
        trea_delay_ms(100);
        EC801E_PowerOn();         // PD9 拉高 (idempotent, 已HIGH 时无影响)
        EC801E_InitAndConnect();

        // 验证4G模块AT响应 + 上报电源状态
        if(EC801E_EnsureConnected() == EC_OK)
        {
            fourG_init_failed = 0;  // 手动连接成功, 清除失败标志
            report_4g_power_state(1);
            lcd_update_csq();
            buzzer_beep(2, 200);
            printf_debug("[SM] BUTTON: 4G power on OK, reported\r\n");
        }
        else
        {
            printf_debug("[SM] BUTTON: 4G power on FAILED (no AT response)\r\n");
        }

        // 等待按键释放, 防止释放抖动再次触发下次唤醒
        while(button_is_pressed())
            trea_delay_ms(100);
        trea_delay_ms(50);

        // === 统一等待 20s 接收平台下发指令 (强制模式) ===
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

        // === 模式化收尾 ===
        if(strcmp(g_window.pattern, "long") == 0)
        {
            // long模式: 保持4G在线 (do_sleep 中 fourG_keep_on 路径已接管)
            EC801E_SetPSM(1);
            printf_debug("[SM] BUTTON: long mode, keep 4G online (PSM)\r\n");
        }
        else
        {
            // short/dot模式: 断电后休眠
            report_4g_power_state(0);
            EC801E_PowerOff();
            printf_debug("[SM] BUTTON: %s mode, 4G powered off\r\n", g_window.pattern);
        }

        app_state = STATE_SLEEP;
        return;
    }

    // debug串口命令由主循环统一处理

    // === BLE连接唤醒: PD2 EXTI下降沿 (手机连接BLE模块) ===
    if(ble_connect_wakeup_flag)
    {
        ble_connect_wakeup_flag = 0;
        ble_exti_disable();

        if(g_bt_on && ble_link_is_connected())
        {
            printf_debug("[SM] WAKEUP: BLE connected (PD2=LOW), entering OTA mode\r\n");
            ensure_full_init();
            ble_uart_init();
            ble_uart_rx_flush();
            ble_ota_init();
            g_ble_ota.state = BLE_OTA_IDLE;
            g_ble_ota.last_rx_ms = trea_millis();
            app_state = STATE_BLE_OTA;
            return;
        }
        else
        {
            printf_debug("[SM] WAKEUP: BLE EXTI but not connected (glitch?)\r\n");
        }
    }

    if(counter0 || ri_wakeup_flag)
    {
        printf_debug("[SM] WAKEUP: %s (downlink)\r\n", counter0 ? "LPUART" : "RI");
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

// ==================== 确认完整初始化 ====================
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

// ==================== 上报调度判断 (do_collect / do_alarm 共用) ====================
// 根据当前模式和时间判断是否需要批量上报历史数据, 设置 app_state
static void check_upload_schedule(void)
{
    // === 低电压省电模式: 不执行常规上报, 8:00/18:00上报 ===
    if(voltage_low_power)
    {
        uint32_t now_hhmmss = rtc_get_hhmmss();
        uint8_t current_hour = (uint8_t)(now_hhmmss / 10000);
        if((current_hour == 8 || current_hour == 18) && !vlp_uploaded_this_hour)
        {
            vlp_uploaded_this_hour = 1;
            if(ee_hist_count() > 0)
            {
                printf_debug("[SM] VLP trigger: hour=%d, uploading %d records\r\n", current_hour, ee_hist_count());
                ensure_full_init();
                app_state = STATE_UPLOAD;
            }
            else
            {
                printf_debug("[SM] VLP trigger: hour=%d, buf empty, skip 4G\r\n", current_hour);
                app_state = STATE_SLEEP;
            }
        }
        else
        {
            if(current_hour != 8 && current_hour != 18)
                vlp_uploaded_this_hour = 0;
            app_state = STATE_SLEEP;
        }
        return;
    }

    // 判断是否需要上报(网格模式, 对齐时间网格)
    if(strcmp(g_window.pattern, "dot") == 0)
    {
        // dot pattern: any configured hour in 24h bitmap triggers upload once per day
        uint32_t now_hhmmss = rtc_get_hhmmss();
        uint8_t current_hour = (uint8_t)(now_hhmmss / 10000);
        rtc_parameter_struct tp;
        rtc_current_time_get(&tp);
        uint16_t day_key = (uint16_t)(((tp.year & 0xFF) << 8) | ((tp.month & 0xFF) << 4) | (tp.date & 0xFF));
        if(day_key != dot_uploaded_day)
        {
            memset(dot_uploaded_mask, 0, sizeof(dot_uploaded_mask));
            dot_uploaded_day = day_key;
        }
        if(window_dottime_hit(g_window.dottime_mask, current_hour) &&
           !window_dottime_hit(dot_uploaded_mask, current_hour))
        {
            printf_debug("[SM] DOT trigger: hour=%d (mask hit)\r\n", current_hour);
            window_dottime_set(dot_uploaded_mask, current_hour);
            ensure_full_init();
            app_state = STATE_UPLOAD;
        }
        else
        {
            app_state = STATE_SLEEP;
        }
    }
    else
    {
        // long/short: 按上报间隔判断是否到达上报时刻
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

// ==================== 采集传感器 + 阈值检查 ====================
static void do_collect(void)
{
    SensorRecord_t record;
    ThresholdResult_t result;
    ThresholdResult_t r1, r2;
    uint8_t dup_skip = 0;
    uint8_t vlp_just_exited = 0;

    // ========== 定时开关阀检查 ==========
    // ========== 定时开关阀检查 ==========
    if(g_valve_timing.on_switch || g_valve_timing.off_switch)
    {
        RtcDateTime_t dt;
        rtc_get_datetime(&dt);
        uint8_t cur_hour = dt.hour;

        // 跨天重置当日执行标志 (00:00~00:59, 若未重置过)
        if(cur_hour == 0 && timing_last_reset_hour != 0)
        {
            timing_open_done_today  = 0;
            timing_close_done_today = 0;
            timing_last_reset_hour  = 0;
            printf_debug("[SM] VTIMING: daily flags reset\r\n");
        }
        else if(cur_hour != 0)
            timing_last_reset_hour = cur_hour;

        // 检查开阀时间 (容差30秒)
        if(!timing_open_done_today && g_valve_timing.on_switch &&
           valve_timing_time_match(g_valve_timing.open_time, &dt, 30))
        {
            printf_debug("[SM] VTIMING: OPEN triggered at %02u:%02u\r\n", dt.hour, dt.minute);
            vtiming_execute_and_report("open");
            timing_open_done_today = 1;
        }

        // 检查关阀时间 (容差30秒)
        if(!timing_close_done_today && g_valve_timing.off_switch &&
           valve_timing_time_match(g_valve_timing.close_time, &dt, 30))
        {
            printf_debug("[SM] VTIMING: CLOSE triggered at %02u:%02u\r\n", dt.hour, dt.minute);
            vtiming_execute_and_report("close");
            timing_close_done_today = 1;
        }
    }

    // 记录本次网格采集的分钟序号, 防止与秒级频率实时唤醒循环在同一分钟重复巡检
    {
        uint32_t now_hhmmss = rtc_get_hhmmss();
        last_grid_collect_min = (uint16_t)(now_hhmmss / 10000) * 60
                              + (uint16_t)((now_hhmmss / 100) % 100);
    }

    PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);
    g_sensor.voltage = g_sensor.last_normal_voltage;

    printf_debug("[SM] COLLECT (buf=%d) V=%.2f S2:P=%.2f T=%.2f err=%d S1:P=%.2f T=%.2f err=%d (ADC=%d)\r\n",
                 ee_hist_count() + 1, g_sensor.voltage,
                 g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.raw.err,
                 g_sensor.raw2.pressure, g_sensor.raw2.temperature, g_sensor.raw2.err,
                 g_need_adc_sampling);

    // 实时模式关闭时, 采集周期结束后 LCD 刷新 (实时模式常亮时由 do_realtime_check 刷新)
#if LCD_LOCK_ENABLED
    if(g_lcd_active && !g_real_enabled)
        lcd_ui_refresh(&g_sensor.raw, &g_sensor.raw2, g_sensor.voltage);
#else
    /* LCD常亮模式: 每个采集周期自动翻一页 */
    if(g_lcd_active && !g_real_enabled)
    { lcd_ui_refresh(&g_sensor.raw, &g_sensor.raw2, g_sensor.voltage); lcd_ui_next_page(); }
#endif

    // ========== 传感器在线/离线状态变化检测(带消抖, 双路在线/离线) ==========
    {
        uint8_t cur_online  = (g_sensor.raw.err  == PT304D_OK) ? 1 : 0;
        uint8_t cur_online2 = (g_sensor.raw2.err == PT304D_OK) ? 1 : 0;
        uint8_t state_changed = sensor_debounce_update(cur_online, cur_online2);

        if(state_changed)
        {
            ensure_full_init();

            printf_debug("[SM] SENSOR STATE CHANGED: S2=%s(err=%d) S1=%s(err=%d) debounce=%d\r\n",
                         cur_online  ? "ON" : "OFF", g_sensor.raw.err,
                         cur_online2 ? "ON" : "OFF", g_sensor.raw2.err,
                         SENSOR_OFFLINE_DEBOUNCE);

            // 启动4G上报
            {
                int was_online;
                int ec_ok = fourG_ensure_ready(&was_online);
                if(ec_ok)
                {
                    report_publish_sensor_status();
                    printf_debug("[SM] Sensor state reported\r\n");
                }
                fourG_restore(was_online, ec_ok);
            }

            // 传感器离线时不参与阈值检查, 直接回睡
            if(!cur_online)
            {
                app_state = STATE_SLEEP;
                return;
            }
        }
        else if(!cur_online)
        {
            // 传感器在线(异常期间), 不重复上报, 直接回睡
            printf_debug("[SM] SENSOR still OFFLINE (cnt=%d/%d cnt2=%d/%d), skip\r\n",
                         g_sensor.offline_cnt, SENSOR_OFFLINE_DEBOUNCE,
                         g_sensor.offline_cnt2, SENSOR_OFFLINE_DEBOUNCE);
            app_state = STATE_SLEEP;
            return;
        }
    }

    // 阈值检查 (双通道全项检查: 压力/温度/突变)
    r1 = threshold_check(g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage, g_sensor.last_pressure_collect, 0, 0);
    r2 = THRESHOLD_OK;
    if(g_sensor_count >= 2 && g_sensor.raw2.err == PT304D_OK)
        r2 = threshold_check2(g_sensor.raw2.pressure, g_sensor.raw2.temperature, g_sensor.last_pressure_collect2, 0);
    result = (r1 != THRESHOLD_OK) ? r1 : r2;

    // 去重: 温度/压力状态与上次一致且报警 → 不重复触发ALARM/上报 (只记录入库)
    {
        ThresholdAlarmState_t st;
        alarm_state_fill(&st, r1, r2);
        if(result != THRESHOLD_OK && memcmp(&st, &s_last_collect_state, sizeof(st)) == 0)
        {
            dup_skip = 1;
            printf_debug("[SM] THRESHOLD SAME AS LAST, skip alarm repeat\r\n");
        }
    }

    if(!dup_skip && result != THRESHOLD_OK)
    {
        // 低电压省电模式: 已进入VPROTECT保护, 不重复VPROTECT报警上报
        if(voltage_low_power && result == THRESHOLD_VPROTECT)
        {
            printf_debug("[SM] VLP: still low voltage, skip alarm\r\n");
            // 不进入ALARM, 保持低电压省电状态
        }
        else
        {
            // 首次VPROTECT报警进ALARM(关阀+上报), 同时标记低电压模式
            if(result == THRESHOLD_VPROTECT && !voltage_low_power)
            {
                voltage_low_power = 1;
                config_save_ext();
                printf_debug("[SM] VLP: entering low voltage power-save mode\r\n");
            }
            printf_debug("[SM] THRESHOLD ABNORMAL (%d), entering ALARM\r\n", result);
            ensure_full_init();
            app_state = STATE_ALARM;
            return;
        }
    }

    // 低电压恢复: 电压恢复到正常时退出VLP
    //   (a) 电压回到 vprotect 以上 (正常恢复)
    //   (b) spupower.switch 被用户关闭(不再执行电压检查, 不能残留 VLP 状态锁定)
    if(!dup_skip && result == THRESHOLD_OK)
        alarm_state_reset(&s_last_collect_state);

    if(voltage_low_power && (!g_spupower.enabled || g_sensor.voltage >= g_spupower.protect + 0.15f))
    {
        voltage_low_power = 0;
        config_save_ext();
        vlp_uploaded_this_hour = 0;
        vlp_just_exited = 1;     // 标记: 本次 collect 时 VLP 刚退出, 走 normal 上报
        if(!g_spupower.enabled)
            printf_debug("[SM] VLP: spupower disabled by user, exiting low power mode\r\n");
        else
            printf_debug("[SM] VLP: voltage recovered (%.2fV >= %.2fV), exiting low power mode\r\n",
                         g_sensor.voltage, g_spupower.protect);
    }

    // 恢复状态上报: (a) 报警自动关阀后恢复正常 → 只上报一次"正常+阀门保持关闭", 不自动操作阀门
    //               (b) VLP 刚退出 → alarm:normal (即使未发生关阀, 也通知平台低电压恢复)
    // 不依赖 4G 是否在线, 避免重复 cold-start
    if(!dup_skip && !voltage_low_power &&
       ((g_valve_alarm_closed && !g_ch4_alarm_closed && !g_alarm_recover_reported) || vlp_just_exited))
    {
        ensure_full_init();
        const char *vr = NULL;

        // (a) 报警关阀后恢复正常: 只上报一次, 不自动操作阀门 (工业安全)
        if(g_valve_alarm_closed && !g_ch4_alarm_closed)
        {
            g_alarm_recover_reported = 1;
            vr = valve_get_status();   /* 上报真实阀门状态(保持关闭) */
            printf_debug("[SM] ALARM recovered, valve stays closed (report only)\r\n");
        }
        else if(vlp_just_exited)
        {
            printf_debug("[SM] VLP recovered, publishing normal status\r\n");
        }

        // 4G 临时连接 + 对称恢复上报
        {
            int was_online;
            int ec_ok = fourG_ensure_ready(&was_online);
            if(ec_ok)
            {
                if(vr != NULL)
                    EC801E_PublishValveStatus(vr, g_valve_last_order);
                report_publish_ly06(0, g_sensor.voltage);
                publish_all_status(g_sensor.last_pressure_collect, g_sensor.last_pressure_collect2);
            }
            fourG_restore(was_online, ec_ok);
        }
    }

    // 记录: 存入缓冲区
    record.pressure = g_sensor.raw.pressure;
    record.temperature = g_sensor.raw.temperature;
    record.pressure2 = g_sensor.raw2.pressure;
    record.temperature2 = g_sensor.raw2.temperature;
    record.voltage = g_sensor.voltage;
    rtc_get_time_str(record.timestamp, sizeof(record.timestamp));
    record.flags = 0;
    memset(record.reserved, 0, sizeof(record.reserved));

    ee_hist_push(record.pressure, record.temperature,
                 record.pressure2, record.temperature2,
                 record.voltage, record.flags);
    g_sensor.last_pressure_collect = g_sensor.raw.pressure;
    g_sensor.last_pressure_collect2 = g_sensor.raw2.pressure;
    wakeup_count++;

    // 报警确认
    check_upload_schedule();
}

// ==================== 报警确认 ====================
static void do_alarm(void)
{
    uint8_t i;
    uint8_t still_abnormal = 1;
    SensorRecord_t record;
    ThresholdResult_t result;
    ThresholdResult_t r1, r2;
    const char *alarm_type = "unknown";

    ensure_full_init();

    printf_debug("[SM] ALARM: rechecking %d times...\r\n", APP_ALARM_RECHECK_COUNT);

    for(i = 0; i < APP_ALARM_RECHECK_COUNT; i++)
    {
        trea_delay_ms(APP_ALARM_RECHECK_DELAY_MS);

        PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);
        sensor_sample_voltage(0, g_sensor.last_normal_voltage);

        r1 = threshold_check(g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage, g_sensor.last_pressure_collect, 1, 0);
        r2 = THRESHOLD_OK;
        if(g_sensor_count >= 2 && g_sensor.raw2.err == PT304D_OK)
            r2 = threshold_check2(g_sensor.raw2.pressure, g_sensor.raw2.temperature, g_sensor.last_pressure_collect2, 0);
        result = (r1 != THRESHOLD_OK) ? r1 : r2;
        printf_debug("[SM] ALARM recheck %d/%d: P=%.2f T=%.2f V=%.2f P2=%.2f T2=%.2f result=%d\r\n",
                     i + 1, APP_ALARM_RECHECK_COUNT, g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage,
                     g_sensor.raw2.pressure, g_sensor.raw2.temperature, result);

        if(result == THRESHOLD_OK)
        {
            // 恢复判断: 正常状态
            printf_debug("[SM] ALARM: recovered at recheck %d, false alarm\r\n", i + 1);
            still_abnormal = 0;
            break;
        }
    }

    // 报警记录
    record.pressure = g_sensor.raw.pressure;
    record.temperature = g_sensor.raw.temperature;
    record.pressure2 = g_sensor.raw2.pressure;
    record.temperature2 = g_sensor.raw2.temperature;
    record.voltage = g_sensor.voltage;
    rtc_get_time_str(record.timestamp, sizeof(record.timestamp));
    memset(record.reserved, 0, sizeof(record.reserved));

    if(still_abnormal)
    {
        // 确认报警
        record.flags = SENSOR_FLAG_ALARM;
        /* 记录确认后的报警状态用于去重上报 (含双路温度) */
        alarm_state_fill(&s_last_collect_state, r1, r2);

        // 判断报警类型 (与 do_realtime_check/downlink 共用 threshold_alarm_type_str)
        alarm_type = threshold_alarm_type_str(result);

        printf_debug("[SM] *** ALARM CONFIRMED: %s ***\r\n", alarm_type);

        // Debug串口打印报警信息
        printf_debug("[SM] ALARM: P=%.2f T=%.2f V=%.2f @%s\r\n",
                     g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage, record.timestamp);

        // 异常关阀 (安全动作, 不依赖4G通信)
        const char *vr = alarm_try_close_valve(result);

        // 4G上报 (报警状态+关阀结果)
        {
            int was_online;
            int ec_ok = fourG_ensure_ready(&was_online);
            if(ec_ok)
            {
                if(r1 != THRESHOLD_OK || r2 != THRESHOLD_OK)
                {
                    /* 一条消息包含全部通道的温度+压力状态 */
                    report_publish_alarm_batch(g_sensor.last_pressure_collect, g_sensor.last_pressure_collect2);
                    /* 低压报警再补一条 (电压B) */
                    if(r1 == THRESHOLD_VPROTECT || r1 == THRESHOLD_VOLT_HIGH || r1 == THRESHOLD_VOLT_LOW)
                        report_publish_ly06(1, g_sensor.voltage);
                }
                if(vr)
                    EC801E_PublishValveStatus(vr, g_valve_last_order);

                // 用户端MQTT同步上报传感器数据
                user_mqtt_try_publish();
            }
            fourG_restore(was_online, ec_ok);
        }
    }
    else
    {
        // 偶发报警, 不记录报警
        record.flags = 0;
    }

    // 存入EEPROM历史记录
    ee_hist_push(record.pressure, record.temperature,
                 record.pressure2, record.temperature2,
                 record.voltage, record.flags);
    g_sensor.last_pressure_collect = g_sensor.raw.pressure;
    g_sensor.last_pressure_collect2 = g_sensor.raw2.pressure;
    wakeup_count++;

    // 上报调度判断 (若报警上报后仍到上报时刻, 历史数据再补上报)
    check_upload_schedule();
}

// ==================== 批量上报 (历史数据, 各模式) ====================
static void do_upload(void)
{
    uint16_t n, i;
    int is_coldstart = 0;

    // === 窗口检查(dot/VLP模式由do_collect已确认时间点, 直接通过) ===
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
                 ee_hist_count(), g_window.pattern,
                 voltage_low_power ? " VLP" : "");

    // === 确保4G模块连接 ===
    com_lpuart_init();
    trea_delay_ms(100);

    // 低电压省电模式 / 弱信号模式: 强制走冷启动路径
    if(voltage_low_power || strcmp(g_window.pattern, "long") != 0 || fourG_weak_signal)
    {
        // short/dot/VLP/weak_signal: 4G断电状态, 重新开机连接
        is_coldstart = 1;
        EC801E_PowerOn();
        if(EC801E_InitAndConnect() != EC_OK)
        {
            printf_debug("[SM] UPLOAD: cold start failed, power off\r\n");
            EC801E_PowerOff();
            app_state = STATE_SLEEP;
            return;
        }
        fourG_init_failed = 0;  // 连接成功, 清除失败标志
        EC801E_SetPSM(0);
        trea_delay_ms(300);
        report_4g_power_state(1);
    }
    else
    {
        // long模式(无VLP, 无弱信号): 4G应保持在线(QSCLK), 直接确认
        if(EC801E_EnsureConnected() != EC_OK)
        {
            printf_debug("[SM] UPLOAD: connect failed, power off\r\n");
            EC801E_PowerOff();
            app_state = STATE_SLEEP;
            return;
        }
        fourG_init_failed = 0;  // EnsureConnected 成功, 清除失败标志
    }

    // 4G连接成功后才更新LCD信号强度
    lcd_update_csq();

    // === 用户端MQTT: 预连接 (历史记录同步上报, 约1分钟耗时) ===
    uint8_t user_mqtt_on = 0;
    if(EC801E_UserMqtt_IsConfigured())
    {
        trea_delay_ms(500);
        if(EC801E_UserMqtt_Connect() == EC_OK)
        {
            user_mqtt_on = 1;
        }
        else
        {
            printf_debug("[SM] UserMqtt: 1st connect failed, retry...\r\n");
            trea_delay_ms(1000);
            user_mqtt_on = (EC801E_UserMqtt_Connect() == EC_OK);
            if(!user_mqtt_on)
                printf_debug("[SM] UserMqtt: 2nd connect failed, skip user mqtt\r\n");
        }
    }

    // === 批量上传 (双通道: 平台id=0 + 用户端id=1, 从EEPROM读取) ===
    while(ee_hist_count() > 0)
    {
        vtiming_check_now();   /* timing valve preempts upload */
        EeHistRecord_t ee_rec;
        SensorRecord_t rec;
        uint8_t rd_ret;

        swdt_feed();
        rd_ret = ee_hist_read(0, &ee_rec);
        if(rd_ret == 0)
        {
            ee_hist_to_sensor_record(&ee_rec, &rec);

            EC_Status_t pub_ret = EC801E_PublishSingleRecord(&rec);
            if(pub_ret == EC_PREEMPT)
            {
                vtiming_check_now();   /* run timing valve, then resend current record */
                continue;
            }
            if(pub_ret == EC_OK)
            {
                if(user_mqtt_on)
                {
                    trea_delay_ms(300);
                    if(EC801E_UserMqtt_PublishRecord(&rec) != EC_OK)
                    {
                        printf_debug("[SM] UserMqtt: publish failed, try reconnect\r\n");
                        EC801E_UserMqtt_Disconnect();
                        trea_delay_ms(500);
                        if(EC801E_UserMqtt_Connect() == EC_OK)
                        {
                            trea_delay_ms(300);
                            EC801E_UserMqtt_PublishRecord(&rec);
                        }
                        else
                        {
                            printf_debug("[SM] UserMqtt: reconnect failed, disable\r\n");
                            user_mqtt_on = 0;
                        }
                    }
                }
                ee_hist_pop();

                if(ee_hist_count() > 0)
                    trea_delay_ms(one_by_one_delay);
            }
            else
            {
                printf_debug("[SM] UPLOAD: Publish record failed, stop remaining uploads\r\n");
                break;
            }
        }
        else
        {
            // CRC校验失败或读取异常, 跳过损坏记录
            printf_debug("[SM] UPLOAD: EEPROM read error (ret=%d), skip record\r\n", rd_ret);
            ee_hist_pop();
        }
    }

    // === 用户端MQTT: 断开 ===
    if(user_mqtt_on)
    {
        trea_delay_ms(500);
        EC801E_UserMqtt_Disconnect();
    }

    // 上报汇总状态 (含传感器状态+信号+压力温度状态+阀门+IP+电压)
    trea_delay_ms(200);
    report_publish_upload_summary();
    printf_debug("[SM] upload summary published\r\n");

    // 上次上传过程结束点位清零重新开始统计计数
    wakeup_count = 0;
    printf_debug("[SM] UPLOAD finished, remaining records in buffer: %d\r\n", ee_hist_count());

    // === RTC校准: 通过4G网络时间同步本地时钟 ===
    {
        trea_delay_ms(200);
        EC_Status_t sync_ret = EC801E_SyncTime();
        if(sync_ret == EC_OK)
            printf_debug("[SM] RTC calibrated from network\r\n");
        else
            printf_debug("[SM] RTC calibration failed (non-critical)\r\n");
    }

    // === 模式化收尾 ===
    if(is_coldstart)
    {
        // 冷启动模式(VLP/short/dot/weak_signal): 等待20s接收平台下发指令
        printf_debug("[SM] Waiting 20s for downlink...\r\n");
        trea_delay_ms(20000);
        {
            uint8_t dl_count = drain_pending_downlink();
            if(dl_count > 0)
                printf_debug("[SM] Processed %d downlink msg(s)\r\n", dl_count);
        }

        // long模式弱信号恢复路径: 检查信号是否恢复, 决定是否保持在线
        if(fourG_weak_signal && strcmp(g_window.pattern, "long") == 0)
        {
            if(fourG_signal_check_and_decide())
            {
                // 信号恢复, 保持4G在线(long模式正常行为)
                EC801E_SetPSM(1);
                printf_debug("[SM] UPLOAD: signal OK, keep 4G online\r\n");
            }
            // else: 内部已断电
        }
        else
        {
            report_4g_power_state(0);
            EC801E_PowerOff();
        }
    }
    else
    {
        // long(无弱信号): 按信号强弱决定去留
        trea_delay_ms(500);
        if(fourG_signal_check_and_decide())
            EC801E_SetPSM(1);
        // else: 信号弱, 内部已断电
    }

    app_state = STATE_SLEEP;
}



// ==================== 处理MQTT下行指令 (下行最高优先级) ====================
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

    /* gas_get 指令: 优先进入 CH4 采集状态机。
     * 放在 SetPSM/弱信号判断之前: 此时 4G 刚被 QMTRECV 唤醒且 MQTT 在线,
     * 直接进入 CH4 可保证采集完成后的上报通道可用; 若先 SetPSM(1), 暖机
     * 30s 后模块已 QSCLK 休眠, 上报首条 AT 会丢失。 */
    if(g_ch4_request) {
        g_ch4_request = 0;
        app_state = STATE_CH4;
        return;
    }

    // 如果信号OK则保持在线 (弱信号断电时已内部处理)
    trea_delay_ms(500);
    if(fourG_signal_check_and_decide())
        EC801E_SetPSM(1);
    // else: 信号弱, 内部已断电

    app_state = STATE_SLEEP;
}

// ==================== BLE OTA 状态处理 ====================
static void do_ble_ota(void)
{
    /* 第一阶段: 等待手机连接BLE (PD2=LOW) */
    if(g_ble_ota.state == BLE_OTA_WAIT_CONNECT)
    {
        if(ble_link_is_connected())
        {
            /* 手机已连接BLE模块 -> 初始化UART开始处理OTA指令 */
            printf_debug("[SM] BLE connected (PD2=LOW), init UART4\r\n");
            ble_uart_init();
            ble_uart_rx_flush();
            g_ble_ota.state = BLE_OTA_IDLE;
            g_ble_ota.last_rx_ms = trea_millis();  /* 重置超时计时 */
            trea_delay_ms(500);
            printf_debug("[SM] BLE connected (PD2=LOW), init UART4---OK\r\n");
        }
        else
        {
            /* 尚未连接, 检查超时 (ble_ota_process内部处理) */
            ble_ota_process();
        }
    }
    else
    {
        /* 第二阶段: 数据接收, 处理OTA指令 */
        ble_ota_process();

        if(g_ble_ota.state == BLE_OTA_DOWNLOADING)
        {
            ota_wdt_feed();
        }
    }
	
#if BUTTON_ENABLED
    /* 按键退出: 3次按键关闭蓝牙 (需3次按键不松手) */
    if(button_is_pressed())
    {
        uint8_t ble_press_count = 1;
        uint16_t ble_wait;

        while(button_is_pressed()) trea_delay_ms(20);
        trea_delay_ms(30);

        while(ble_press_count < 3)
        {
            uint8_t ble_got = 0;
            for(ble_wait = 0; ble_wait < 500; ble_wait += 20)
            {
                trea_delay_ms(20);
                if(button_is_pressed()) { ble_got = 1; break; }
            }
            if(!ble_got) break;
            ble_press_count++;
            while(button_is_pressed()) trea_delay_ms(20);
            trea_delay_ms(30);
        }

        if(ble_press_count >= 3)
        {
            printf_debug("[SM] BLE_OTA: 3-press detected, turning off BLE\r\n");
            g_bt_on = 0;
            power_ble_set(0);
            if(g_lcd_active) slcd_seg_icon_display(SLCD_ICON_LY, RESET);
        }
    }
#endif

    /* 退出条件: OTA完成/超时, 关闭蓝牙并返回 */
    if(g_ble_ota.state == BLE_OTA_ERROR || !g_bt_on)
    {
        printf_debug("[SM] BLE_OTA exit: state=%d, bt=%d\r\n", g_ble_ota.state, g_bt_on);
        ble_uart_deinit();
        if(g_bt_on)
        {
            lcd_ui_toggle_bluetooth();
        }
        app_state = STATE_SLEEP;
    }
    /* 注: OTA_COMPLETE不会走到这里, handle_end()已调用NVIC_SystemReset() */
}

// ==================== Debug串口命令处理(需要硬件上下文) ====================
static void debug_check_cmd(void)
{
    static char line_buf[512];
    uint16_t len;

    while((len = debug_rx_read_line(line_buf, sizeof(line_buf), 0)) > 0)
    {
        printf_debug("[DBG] %s\r\n", line_buf);

        // ---- commands requiring main.c hardware context ----
        if(strcmp(line_buf, "status") == 0)
        {
            const char *state_names[] = {"INIT","SLEEP","WAKEUP","COLLECT","UPLOAD","DOWNLINK","ALARM","BLE_OTA"};
            printf_debug("  state=%s wakeup=%d buf=%d\r\n",
                         (app_state <= STATE_BLE_OTA) ? state_names[app_state] : "?",
                         wakeup_count, ee_hist_count());
            printf_debug("  P=%.2f T=%.2f V=%.2f last_Pc=%.2f last_Pr=%.2f\r\n",
                         g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage, g_sensor.last_pressure_collect, g_sensor.last_pressure_realtime);
            printf_debug("  valve=%d abnormal=%s\r\n", g_valve_last_order, g_abnormal_close);
            printf_debug("  4G: user_off=%d init_fail=%d weak_sig=%d\r\n", fourG_user_off, fourG_init_failed, fourG_weak_signal);
            printf_debug("  BLE: on=%d\r\n", g_bt_on);
            printf_debug("  PT304D err=%d stat=0x%02X\r\n", g_sensor.raw.err, g_sensor.raw.stat_raw);
            continue;
        }
        if(strcmp(line_buf, "sensor") == 0)
        {
            PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);
            g_sensor.voltage = adc_bat_read_voltage();
            printf_debug("  err=%d stat=0x%02X P=%.2f T=%.2f V=%.2f\r\n",
                         g_sensor.raw.err, g_sensor.raw.stat_raw, g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage);
            if(g_sensor.raw.err != PT304D_OK)
                printf_debug("  press_raw=0x%06X temp_raw=0x%06X\r\n", g_sensor.raw.press_raw, g_sensor.raw.temp_raw);
            continue;
        }
        if(strcmp(line_buf, "sensor_status") == 0)
        {
            PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);
            const char *have   = (g_sensor.raw.err != PT304D_ERR_I2C) ? "1" : "0";
            const char *status = (g_sensor.raw.err == PT304D_OK)       ? "1" : "0";
            printf_debug("  have=%s status=%s err=%d stat=0x%02X\r\n",
                         have, status, g_sensor.raw.err, g_sensor.raw.stat_raw);
            continue;
        }
        if(strcmp(line_buf, "temp_status") == 0)
        {
            PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);
            TempStatus_t ts = threshold_get_temp_status(g_sensor.raw.temperature);
            printf_debug("  T=%.2f -> %s\r\n", g_sensor.raw.temperature, threshold_temp_status_str(ts));
            continue;
        }
        if(strcmp(line_buf, "pressure_status") == 0)
        {
            PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);
            PressureStatus_t ps = threshold_get_pressure_status(g_sensor.raw.pressure, g_sensor.last_pressure_collect);
            printf_debug("  P=%.2f -> %s\r\n", g_sensor.raw.pressure, threshold_pressure_status_str(ps));
            continue;
        }
        if(strcmp(line_buf, "sim") == 0)
        {
            char csq[8] = {0}, iccid[32] = {0}, imei[24] = {0};
            int rsrp = 0, sinr = 0;
            uint8_t cereg = 0;
            EC801E_Wakeup();
            EC801E_QueryCSQ(csq, sizeof(csq));
            EC801E_QueryQCSQ(&rsrp, &sinr);
            EC801E_QueryCEREG(&cereg);
            EC801E_QueryICCID(iccid, sizeof(iccid));
            EC801E_QueryIMEI(imei, sizeof(imei));
            printf_debug("  CSQ=%s RSRP=%d SINR=%d CEREG=%d\r\n", csq, rsrp, sinr, cereg);
            printf_debug("  ICCID=%s IMEI=%s\r\n", iccid, imei);
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
            const char *action = line_buf + 6;  // "open" or "close"
            const char *result = valve_execute(action, 1);
            if(g_valve_alarm_closed)
            {
                g_valve_alarm_closed = 0;
                g_valve_pre_alarm_order = g_valve_last_order;
                config_save_ext();
                printf_debug("  cleared alarm_closed flag (user override)\r\n");
            }
            g_ch4_alarm_closed = 0;
            g_alarm_recover_reported = 0;
            printf_debug("  valve(%s) -> %s  last_order=%d\r\n", action, result, g_valve_last_order);
            continue;
        }
        if(strncmp(line_buf, "first_ip:", 9) == 0)
        {
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
            fourG_weak_signal = 0;
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
            printf_debug("  ee_hist: %d records\r\n", ee_hist_count());
            continue;
        }
        if(strcmp(line_buf, "time") == 0)
        {
            rtc_show_time();
            continue;
        }

        // ---- commands without hardware context, delegated to eeprom_config.c ----
        if(config_parse_and_apply(line_buf) == 0)
            printf_debug("[DBG] Unknown: %s\r\n", line_buf);
    }
}

#if LASER_CH4_ENABLED
// ==================== 激光甲烷定时采集调度 ====================
// laser_switch 仅管理读取时间点: 当前小时命中 laser_time 位图且本小时未执行过则触发 STATE_CH4
static void laser_schedule_check(void)
{
    RtcDateTime_t dt;

    if(!g_laser_switch) return;

    rtc_get_datetime(&dt);
    if(dt.hour != s_laser_done_hour && window_dottime_hit(g_laser_time_mask, dt.hour))
    {
        s_laser_done_hour = dt.hour;
        printf_debug("[SM] LASER schedule trigger at %02u:00\r\n", dt.hour);
        /* 仅置请求标志, 由主循环在 app_state==SLEEP 时转入 STATE_CH4,
         * 确保网格采集/整点上传先完成, 不再抢占 STATE_COLLECT */
        g_ch4_request = 1;
    }
}
#endif /* LASER_CH4_ENABLED */

// ==================== 4G连接主动巡检 (一直在线模式, 每30分钟) ====================
// long模式且(活动窗口或end_result=1)时4G应保持在线; 每30分钟主动确认与平台MQTT连接,
// 掉线自动重连; 重连失败断电并置失败标志, 等下次上报冷启动 (与 do_upload 一致)
static void check_4g_connection_supervision(void)
{
    static uint16_t s_last_supervise_min = 0xFFFF;
    uint32_t now_hhmmss;
    uint16_t now_min;

    /* 非"一直在线"模式: 不巡检, 保持现状 */
    if(voltage_low_power || fourG_user_off || fourG_init_failed || fourG_weak_signal ||
       strcmp(g_window.pattern, "long") != 0 ||
       !(config_is_active_window() || g_window.end_result == 1))
        return;

    now_hhmmss = rtc_get_hhmmss();
    now_min = (uint16_t)((now_hhmmss / 10000) * 60 + (now_hhmmss / 100) % 100);

    if(s_last_supervise_min == 0xFFFF)
    {
        s_last_supervise_min = now_min;   /* 上电后满30分钟再首次巡检 */
        return;
    }
    if((uint16_t)(now_min - s_last_supervise_min) < FOURG_SUPERVISE_INTERVAL_MIN)
        return;

    s_last_supervise_min = now_min;
    printf_debug("[SM] 4G supervision: checking MQTT connection...\r\n");

    if(EC801E_EnsureConnected() != EC_OK)
    {
        printf_debug("[SM] 4G supervision: reconnect failed, power off (next upload cold start)\r\n");
        EC801E_PowerOff();
        fourG_init_failed = 1;
        return;
    }

    fourG_init_failed = 0;
    EC801E_SetPSM(1);   /* 恢复QSCLK轻休眠, 保持在线 */
}

// ==================== 主函数 ====================
int main(void)
{
    SensorRecord_t init_record;		//首次采集记录

    // ==================== STATE_INIT ====================
    sys_init();
	// ==================== 首次采集上报 ====================
	PT304D_Init(PT304D_SENSOR_1);
    PT304D_Init(PT304D_SENSOR_2);
    // 软件看门狗: SysTick 由 sys_init 启动, 可保护后续的 4G 连接/上报等耗时操作
    swdt_enable(SWDT_TIMEOUT_MS);
    com_lpuart_init();
    lpuart_wakeup_init();
	printf_debug("\r\n[OTA] yes%s \r\n", APP_FIRMWARE_VERSION);
	
	// 4G上电 + 注册 + MQTT连接 + keepalive + 时间同步 + QSCLK
    EC801E_PowerOn();
	
    config_load_all();
    config_enter_setup_mode(0);
    config_print_current();

    rtc_init_all();
    rtc_alarm_wakeup_init();
	
	
	
    ee_hist_init();
	
	// SLCD初始化(必须在rtc_init_all之后, LXTAL已启动)
    if(g_lcd_enabled) { slcd_seg_configuration(); lcd_ui_init(); g_lcd_active = 1; g_lcd_on_hhmmss = rtc_get_hhmmss(); }
	
	// 首次采集+存入缓冲区+上报
    PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);
	// 首次采集后刷新LCD显示
    sensor_sample_voltage(1, g_sensor.last_normal_voltage);
    if(g_lcd_active) lcd_ui_refresh(&g_sensor.raw, &g_sensor.raw2, g_sensor.voltage);

    
	
    printf_debug("\r\n[OTA] yes%s \r\n", APP_FIRMWARE_VERSION);
	
    printf_debug("\r\n========================================\r\n");
    printf_debug("  Household Valve Controller v%s\r\n", APP_FIRMWARE_VERSION);
    printf_debug("  Collect: %dmin  Upload: %dmin (grid-aligned)\r\n",
                 g_interval.collect_interval_min,
                 g_interval.upload_interval_min);
    char dot_str5[64]; window_dottime_to_str(g_window.dottime_mask, dot_str5, sizeof(dot_str5));
    printf_debug("  Window: pattern=%s start=%s stop=%s er=%d dot=%s\r\n",
                 g_window.pattern, g_window.start, g_window.stop,
                 g_window.end_result, dot_str5);
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

    // 4G连接前检查debug串口是否有待处理命令
    if(debug_rx_line_ready())
            debug_check_cmd();

	// 4G上电 + 注册 + MQTT连接 + keepalive + 时间同步 + QSCLK
    uint8_t init_4g_ok = (EC801E_InitAndConnect() == EC_OK);

    if(init_4g_ok)
    {

        // 4G连接成功后, 再检查debug串口命令
        if(debug_rx_line_ready())
            debug_check_cmd();

        // 用户端MQTT连接 (id=1, 与本地服务器同一阶段建立)
        if(EC801E_UserMqtt_IsConfigured())
            EC801E_UserMqtt_Connect();
        report_4g_power_state(1);

        // 4G连接后更新LCD信号强度图标
        lcd_update_csq();

        // 连接成功提示音
        buzzer_beep(1, 200);
    }
    else
    {
        printf_debug("[SM] INIT: 4G connect failed, skip MQTT reports\r\n");
        fourG_init_failed = 1;
    }

    // OTA启动前debug串口命令
    config_check_cmd();

    // === OTA 启动回执: 按 InfoBlock 显示上次升级 SUCCESS/FAILED, 上报 mqttUpgradeReply ===
    ota_app_report_boot_status();

    
    trea_delay_ms(50);

    init_record.pressure = g_sensor.raw.pressure;
    init_record.temperature = g_sensor.raw.temperature;
    init_record.pressure2 = g_sensor.raw2.pressure;
    init_record.temperature2 = g_sensor.raw2.temperature;
    init_record.voltage = g_sensor.voltage;
    rtc_get_time_str(init_record.timestamp, sizeof(init_record.timestamp));
    init_record.flags = 0;
    memset(init_record.reserved, 0, sizeof(init_record.reserved));
    ee_hist_push(init_record.pressure, init_record.temperature,
                 init_record.pressure2, init_record.temperature2,
                 init_record.voltage, init_record.flags);
    g_sensor.last_pressure_collect  = g_sensor.raw.pressure;
    g_sensor.last_pressure_realtime = g_sensor.raw.pressure;
    g_sensor.last_pressure_collect2  = g_sensor.raw2.pressure;
    g_sensor.last_pressure_realtime2 = g_sensor.raw2.pressure;

    printf_debug("[SM] INIT V=%.2f S2:P=%.2f T=%.2f err=%d S1:P=%.2f T=%.2f err=%d\r\n",
                 g_sensor.voltage,
                 g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.raw.err,
                 g_sensor.raw2.pressure, g_sensor.raw2.temperature, g_sensor.raw2.err);

    

    // 将首次采集结果更新 prev_online (不论4G是否成功, 防止实时模式误判状态变化)
    g_sensor.prev_online  = (g_sensor.raw.err  == PT304D_OK) ? 1 : 0;
    g_sensor.prev_online2 = (g_sensor.raw2.err == PT304D_OK) ? 1 : 0;

    // ==================== 首次批量上报 (当4G连接成功时) ====================
    if(!init_4g_ok) goto init_skip_report;

    EC801E_PublishSensorData(&g_sensor.raw, g_sensor.voltage);
    trea_delay_ms(500);

    // 第1: restart=1 + valve_last_order
    {
        char v_order[4];
        snprintf(v_order, sizeof(v_order), "%d", g_valve_last_order);
        static char boot_json[100];
        static char boot_ts[32];
        rtc_get_time_str(boot_ts, sizeof(boot_ts));
        snprintf(boot_json, sizeof(boot_json),
            "[{\"id\":\"restart\",\"value\":\"1\",\"ts\":\"%s\",\"remark\":\"\"}]",
            boot_ts);
        EC801E_PublishRawJson(boot_json);
    }
    trea_delay_ms(300);

    // 第2: 设备信息 (csq + rsrp + sinr + cereg + soft-version + ICCID + IMEI)
    {
        static char info_json[530];
        char csq[8] = {0}, iccid[32] = {0}, imei[24] = {0};
        char rsrp_str[8] = {0}, sinr_str[8] = {0}, cereg_str[4] = {0};
        int rsrp = 0, sinr = 0;
        uint8_t cereg = 0;
        EC801E_Wakeup();
        EC801E_QueryCSQ(csq, sizeof(csq));
        if(EC801E_QueryQCSQ(&rsrp, &sinr) == EC_OK)
        {
            snprintf(rsrp_str, sizeof(rsrp_str), "%d", rsrp);
            snprintf(sinr_str, sizeof(sinr_str), "%d", sinr);
        }
        EC801E_QueryCEREG(&cereg);
        snprintf(cereg_str, sizeof(cereg_str), "%d", cereg);
        trea_delay_ms(200);
        EC801E_QueryICCID(iccid, sizeof(iccid));
        trea_delay_ms(200);
        EC801E_QueryIMEI(imei, sizeof(imei));
        static char info_ts[32];
        rtc_get_time_str(info_ts, sizeof(info_ts));
        snprintf(info_json, sizeof(info_json),
            "[{\"id\":\"csq\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"rsrp\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"snr\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"soft-version\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"ICCID\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"IMEI\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]",
            csq, info_ts, rsrp_str, info_ts, sinr_str, info_ts, 
            APP_FIRMWARE_VERSION, info_ts, iccid, info_ts, imei, info_ts);
        EC801E_PublishRawJson(info_json);
        printf_debug("[SM] Device info:\r\n CSQ=%s RSRP=%s SINR=%s CEREG=%s\r\n ICCID=%s\r\n IMEI=%s\r\n",
                     csq, rsrp_str, sinr_str, cereg_str, iccid, imei);
    }

    // 第3: 传感器状态(在线+离线)字段
    {
//        report_publish_sensor_status();   // rtu_have/status 各 2 (传感器状态字段上报)
		report_publish_upload_summary();
        g_sensor.prev_online  = (g_sensor.raw.err  == PT304D_OK) ? 1 : 0;
        g_sensor.prev_online2 = (g_sensor.raw2.err == PT304D_OK) ? 1 : 0;
    }


    // === 用户端MQTT上报 (id=1, 与传感器数据同一阶段建立) ===
    if(EC801E_UserMqtt_IsConfigured())
    {
        trea_delay_ms(300);
        EC801E_UserMqtt_PublishSensorData(&g_sensor.raw, &g_sensor.raw2, g_sensor.voltage);
        trea_delay_ms(500);
        EC801E_UserMqtt_Disconnect();
    }

	// 连接成功提示音
    buzzer_beep(2, 200);
init_skip_report:
    if(init_4g_ok)
    {
        // 首次上报后也检查信号: 弱信号直接断电, 不再无效重试
        if(!fourG_signal_check_and_decide())
            printf_debug("[SM] INIT: weak signal, 4G powered off\r\n");
        else
            EC801E_SetPSM(1);
    }
    else
        EC801E_PowerOff();
    wakeup_count = 0;

    // INIT结束: 若 spupower 关闭或电压已恢复, 则清除退出时的 VLP 状态
    if(voltage_low_power && (!g_spupower.enabled || g_sensor.voltage >= g_spupower.protect + 0.15f))
    {
        voltage_low_power = 0;
        config_save_ext();
        printf_debug("[SM] VLP cleared on boot (spup_en=%d V=%.2f)\r\n", g_spupower.enabled, g_sensor.voltage);
    }

    // 初始化完成, 进入睡眠
    app_state = STATE_SLEEP;

    // ==================== 主状态机循环 ====================
    while(1)
    {
        // ISR置位标志后执行对应debug串口命令处理
        if(debug_rx_line_ready())
            debug_check_cmd();

        // 软件看门狗: 每个状态分支前重新使能并复位计时器(活跃阶段均被保护)
        // 注意: 进入 enter_deepsleep_mode() 内部会 swdt_disable() 关闭, 休眠期间不计时
        swdt_enable(SWDT_TIMEOUT_MS);

        vtiming_check_now();   /* timing valve highest priority */
#if LASER_CH4_ENABLED
        laser_schedule_check();  /* 激光甲烷定时采集: 置 g_ch4_request */
        if(g_ch4_request && app_state == STATE_SLEEP)
        {
            g_ch4_request = 0;
            app_state = STATE_CH4;
        }
#endif
        check_4g_connection_supervision();  /* 一直在线模式: 每30分钟巡检4G/平台连接 */
        switch(app_state)
        {
            case STATE_SLEEP:    do_sleep();    break;
            case STATE_WAKEUP:   do_wakeup();   break;
            case STATE_COLLECT:  do_collect();  break;
            case STATE_UPLOAD:   do_upload();   break;
            case STATE_DOWNLINK: do_downlink(); break;
            case STATE_ALARM:    do_alarm();    break;
            case STATE_BLE_OTA:  do_ble_ota();  break;
            case STATE_CH4:
#if LASER_CH4_ENABLED
                do_ch4();
#else
                app_state = STATE_SLEEP;
#endif
                break;
            default:             app_state = STATE_SLEEP; break;
        }
    }
}
