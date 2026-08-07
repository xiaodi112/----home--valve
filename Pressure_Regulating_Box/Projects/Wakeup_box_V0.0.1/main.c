/**
 * @file main.c
 * @brief 家用阀门物联网控制�?�?状态机驱动低功耗主程序
 *
 * 架构: MQTT持久连接 + MCU深休�?+ LPUART唤醒接收下发
 *
 * 状态机 (7个状�?:
 *   STATE_INIT     �?上电初始�?MQTT连接+订阅+首次上报 �?STATE_SLEEP
 *   STATE_SLEEP    �?设闹�?深休�?模块QSCLK保持MQTT) �?STATE_WAKEUP
 *   STATE_WAKEUP   �?恢复外设+判断唤醒�?
 *                     RTC闹钟 �?STATE_COLLECT
 *                     LPUART  �?STATE_DOWNLINK
 *   STATE_COLLECT  �?采集+阈值检�?存入缓冲�?�?STATE_UPLOAD/ALARM/SLEEP
 *   STATE_UPLOAD   �?批量逐条上报+清空缓冲�?�?STATE_SLEEP
 *   STATE_DOWNLINK �?读取+处理MQTT下发指令+立即上报结果 �?STATE_SLEEP
 *   STATE_ALARM    �?5次重采确�?报警上报 �?STATE_SLEEP
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
#include "m24m01.h"
#include "main.h"
#include "adc_bat.h"
#include "i2c_sensor.h"
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

// ==================== 状态定�?====================
typedef enum {
    STATE_INIT = 0,
    STATE_SLEEP,
    STATE_WAKEUP,
    STATE_COLLECT,
    STATE_UPLOAD,
    STATE_DOWNLINK,
    STATE_ALARM,
    STATE_BLE_OTA
} AppState_t;

// ==================== 全局变量 ====================
SensorContext_t g_sensor = { .last_pressure_collect = -1.0f, .last_pressure_realtime = -1.0f, .last_pressure_collect2 = -1.0f, .last_pressure_realtime2 = -1.0f, .prev_online = 1, .offline_cnt = 0, .online_cnt = 0, .prev_online2 = 1, .offline_cnt2 = 0, .online_cnt2 = 0, .last_normal_voltage = 3.6f };

#define SENSOR_OFFLINE_DEBOUNCE  3   // 连续N次离线才确认状态变�?
#define SENSOR_ONLINE_DEBOUNCE   2   // 连续N次上线才确认状态变�?(防接触不良抖�?
#define	one_by_one_delay		 900	//批量逐条延时

static volatile uint16_t wakeup_count = 0;
static AppState_t app_state = STATE_INIT;
static uint8_t dot_uploaded_mask[3] = {0};
static uint16_t dot_uploaded_day = 0xFFFF;  // RTC day of uploaded bitmap (cross-day reset)  // dot pattern: uploaded-hour bitmap
// valve_alarm_closed / valve_pre_alarm_order 已移�?g_* 全局 (EEPROM 持久�? OTA 重启后可恢复), �?eeprom_config.h
uint8_t voltage_low_power = 0;              // 低电压省电模�? �?:00/18:00上报
uint8_t fourG_user_off = 0;                 // 用户主动关机4G标志(阻止do_wakeup盲目恢复PD9)
static uint8_t fourG_init_failed = 0;       // 4G注网失败标志(阻止do_sleep保持4G供电, 下次上报周期重试)
static uint8_t fourG_weak_signal = 0;       // 弱信号断电标志(long模式下CSQ过低, 暂时按需冷启动)
#define CSQ_WEAK_THRESHOLD  8               // CSQ<此值触发断电省电(8≈-97dBm)
static uint8_t vlp_uploaded_this_hour = 0;  // 低电压模�? 防止同一小时重复上报
ThresholdResult_t last_realtime_alarm = THRESHOLD_OK;          // 实时模式防重复报�?
static uint32_t g_ble_on_hhmmss = 0;                          // BLE开启时的RTC时间 (HHMMSS), 用于60s未连接超时自动关闭�?
static uint8_t g_is_minimal_wakeup = 0;            // 标志：是否为纯周期传感器采集的最简唤醒状�?
uint8_t g_need_adc_sampling = 1;                   // 标志：当前周期是否需要进行电池电压采�?
static uint16_t last_grid_collect_min = 0xFFFF;    // 标志：上一次执行周期采集的分钟戳，防止在同一采集整点多次触发

// ==================== 定时开关阀 当日执行标志 ====================
uint8_t timing_open_done_today  = 0;   // 今日已执行开阀
uint8_t timing_close_done_today = 0;   // 今日已执行关阀
static uint8_t timing_last_reset_hour  = 0xFF;// 上次重置标志的小时(防重复)
volatile uint8_t g_timing_pending = 0;   /* timing valve pending (set by RTC ISR) */
volatile uint8_t g_timing_alarm_set = 0; /* current RTC alarm is a timing-valve alarm */

// ==================== LCD 延时关闭 ====================
uint8_t  g_lcd_active     = 0;      // LCD当前是否在显�?(运行�? 区别于产品开关g_lcd_enabled)
static uint32_t g_lcd_on_hhmmss = 0; // LCD被点亮时的RTC时间 (HHMMSS格式)
#define LCD_DISPLAY_TIMEOUT_SEC  60  // LCD持续显示秒数

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
    slcd_disable();
//    rcu_periph_clock_disable(RCU_SLCD);

    // 将SLCD全部COM/SEG引脚设为模拟输入, 与无LCD产品状态完全一�?
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
                  GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);                   /* COM0-2 */
    gpio_mode_set(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
                  GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_9 |                    /* SEG5,6 + COM3 */
                  GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |                 /* SEG10-12 */
                  GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);                 /* SEG13-15 */
    gpio_mode_set(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE,
                  GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 |                    /* SEG24-26 */
                  GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11);                  /* SEG27-29 */

    g_lcd_active = 0;
}

static void lcd_check_timeout(void)
{
    if(!g_lcd_active) return;
    uint32_t now_sec = hhmmss_to_seconds(rtc_get_hhmmss());
    uint32_t on_sec  = hhmmss_to_seconds(g_lcd_on_hhmmss);
    uint32_t elapsed;
    if(now_sec >= on_sec)
        elapsed = now_sec - on_sec;
    else
        elapsed = (86400 - on_sec) + now_sec;  // 跨午�?
    if(elapsed >= LCD_DISPLAY_TIMEOUT_SEC)
        lcd_shutdown();
}

// ==================== 模块共享 JSON 上报缓冲 ====================
// 多处 pressure_status / temp_status / rtu_have 上报顺序使用, 共用一�?RAM, 节省 ~1.7KB
// 每个 publish 块完整执�?(生成时间戳→snprintf→PublishRawJson) 后下一块再�? 不嵌�?
static char s_status_json[384];   // ps_json / ts_json / rtu_json 公用 (4字段rtu�?80字节)
static char s_status_ts[32];      // ps_ts / ts_ts / rtu_ts / rt_ts 公用

// 单字段状态上�? 生成时间�?�?构建 [{"id","value","ts","remark"}] �?PublishRawJson
// �?csq / pressure_status / temp_status 等单字段上报共用
static void publish_status_field(const char *id, const char *value)
{
    rtc_get_time_str(s_status_ts, sizeof(s_status_ts));
    snprintf(s_status_json, sizeof(s_status_json),
        "[{\"id\":\"%s\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]", id, value, s_status_ts);
    EC801E_PublishRawJson(s_status_json);
}

// 传感器在�?离线消抖状态机 (进气�?raw + 出气�?raw2 共用; do_collect/do_realtime_check 复用)
// 更新 offline_cnt/offline_cnt2; 消抖通过后更�?prev_online/prev_online2.
// 返回: 1 = 发生经消抖确认的状态变�? 0 = 无变�?
static uint8_t sensor_debounce_update(uint8_t cur_online, uint8_t cur_online2)
{
    uint8_t state_changed = 0;

    // --- 进气�?(raw, PT304D #2) ---
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

    // --- 出气�?(raw2, PT304D #1) --- 仅双传感器时参与消抖
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
        // 仅在消抖通过后更�?prev 状�?
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
    /* 重定位中断向量表�?App 区起址 (Bootloader 跳转后必须设�? Cortex-M23 支持 VTOR) */
    SCB->VTOR = OTA_APP_ADDR;

    systick_config();
    my_systick_config();
    timer1_delay_init();
    trea_uart_init();

    gpio_init();

    // PB5(V4G_EN 4G电源) 立即恢复: gpio_init 后 ODR=0 → PB5=LOW
    // 若4G 唤醒前在线(long模式+窗口内), 必须在此处第一时间拉高, 否则 1s+ 延时会导致4G 复位
    // 弱信号模式: 4G已断电, 不恢复PB5
    if(!voltage_low_power && !fourG_user_off && !fourG_init_failed && !fourG_weak_signal &&
       strcmp(g_window.pattern, "long") == 0 &&
       (config_is_active_window() || g_window.end_result == 1))
        gpio_bit_set(GPIOB, GPIO_PIN_5);

    motor_init();
    IIC_Init();
	IIC_GPIO_Init1();
    adc_bat_init();
    // PB5 (V4G_EN 4G电源) 状态由上方条件逻辑控制，此处不做覆�?
    // PA5 (M_SLP 电机使能) �?valve_execute() 按需控制，此处不提前开�?
    // PD4 (I2C 电子开�? 默认保持 LOW; 由各采集点按需 set/reset

    debug_rx_init();   // USART0 RX中断缓冲初始�?
}

// ==================== 前向声明 ====================
static void ensure_full_init(void);

// ==================== LCD信号强度更新 (4G在线时调�? 同时上报本端平台) ====================
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

        // 上报CSQ到本端平台
        publish_status_field("csq", csq_buf);
    }

    // 附加诊断: LTE信号质量 + 网络注册状态
    {
        int rsrp = 0, sinr = 0;
        uint8_t cereg = 0;
        EC801E_QueryQCSQ(&rsrp, &sinr);
        EC801E_QueryCEREG(&cereg);
        printf_debug("[SM] Signal: CSQ=%s RSRP=%d SINR=%d CEREG=%d\r\n",
                     csq_buf, rsrp, sinr, cereg);
    }
}

// ==================== 4G 公共模块: 唤醒/连接 + 恢复 ====================
// 确保4G模块可通信: 已通电则唤醒, 未通电则开机连接
// 返回: 1=连接成功, 0=失败; *p_was_online: 调用前4G是否在线(用于fourG_restore)
static int fourG_ensure_ready(int *p_was_online)
{
    int was_online = (gpio_input_bit_get(GPIOB, GPIO_PIN_5) == SET);
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

// 恢复4G到操作后状态:
//   success=0 (连接失败): 无论之前是否在线, 统一断电 (下周期重新PowerOn)
//   success=1 且之前在线: SetPSM(1) 恢复长连接休眠
//   success=1 且之前离线: 上报下线后断电 (临时开机已完成任务)
static void fourG_restore(int was_online, int success)
{
    if(!success)
    {
        EC801E_PowerOff();
        return;
    }
    if(was_online)
    {
        // long模式: 检查信号强度决定是否保持在线
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
        // 之前离线(冷启动): 检查是否为弱信号恢复场景
        if(fourG_weak_signal && strcmp(g_window.pattern, "long") == 0)
        {
            // 弱信号冷启动后检查信号是否恢复
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
            // 仍弱: 断电
            printf_debug("[SM] Still weak signal, power off\r\n");
        }
        report_4g_power_state(0);
        EC801E_PowerOff();
    }
}

// ==================== 弱信号检查+决策 (do_upload/do_downlink 收尾专用) ====================
// 上报/下行处理完毕后调用, 决定4G去留
// 返回: 1=信号OK(调用者应SetPSM), 0=信号弱(内部已断电)
static int fourG_signal_check_and_decide(void)
{
    if(strcmp(g_window.pattern, "long") != 0)
        return 1;  // 非long模式不参与此逻辑

    char csq_buf[8] = {0};
    if(EC801E_QueryCSQ(csq_buf, sizeof(csq_buf)) != EC_OK)
    {
        // 查询失败视为弱信号
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

// ==================== 公共功能模块 ====================

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

// 异常自动关阀: 根据策略决定是否关阀, 独立于4G通信状态
// 返回: 阀门执行结果字符串(已关阀), NULL(未关阀)
// 安全设计: 此函数内部初始化IIC/电机, 确保任何上下文均可安全调用
const char* alarm_try_close_valve(ThresholdResult_t result)
{
    AbnormalClose_t policy = abnormal_from_str(g_abnormal_close);
    if((abnormal_should_close_valve(policy, result) || result == THRESHOLD_VPROTECT) && !g_valve_alarm_closed)
    {
        g_valve_pre_alarm_order = g_valve_last_order;
        printf_debug("[SM] Auto-close valve (policy=%s, result=%d, pre=%d)\r\n",
                     g_abnormal_close, result, g_valve_pre_alarm_order);
        IIC_Init();
        motor_init();
        const char *vr = valve_execute("close", 0);
        g_valve_alarm_closed = 1;
        config_save_ext();
        return vr;
    }
    return NULL;
}

// 用户端MQTT上报 (id=1, LY格式双通道): 连接→发送→断开 (含1次重试)
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

// ==================== 状态上报公共模块 ====================
// 上报进气端+出气端的 pressure_status / temp_status
// last_p1/last_p2: 上次压力值(用于突变检测)
static void publish_all_status(float last_p1, float last_p2)
{
    PressureStatus_t ps = threshold_get_pressure_status(g_sensor.raw.pressure, last_p1);
    publish_status_field("pressure_status", threshold_pressure_status_str(ps));

    TempStatus_t ts = threshold_get_temp_status(g_sensor.raw.temperature);
    publish_status_field("temp_status", threshold_temp_status_str(ts));

    if(g_sensor_count >= 2 && g_sensor.raw2.err == PT304D_OK)
    {
        PressureStatus_t ps2 = threshold_get_pressure_status2(g_sensor.raw2.pressure, last_p2);
        publish_status_field("pressure2_status", threshold_pressure_status_str(ps2));
        TempStatus_t ts2 = threshold_get_temp_status2(g_sensor.raw2.temperature);
        publish_status_field("temp2_status", threshold_temp_status_str(ts2));
    }
}

// ==================== 定时阀门: 执行+上报(公共函数) ====================
// action: "open" 或 "close"
// 执行阀门动作, 唤醒4G上报 valve_control + valve_last_order, 恢复PSM
static void vtiming_execute_and_report(const char *action);   /* forward */
/* timing valve immediate check: run during upload/blocking as fallback;
 * executes when within +-30s window and not yet done today */
static void vtiming_check_now(void)
{
    uint32_t now_hhmmss;
    uint32_t now_sec;
    int32_t diff;
    uint8_t cur_hour;

    if(!g_valve_timing.enabled) return;

    now_hhmmss = rtc_get_hhmmss();
    now_sec = hhmmss_to_seconds(now_hhmmss);
    cur_hour = (uint8_t)(now_hhmmss / 10000);

    if(cur_hour == 0 && timing_last_reset_hour != 0)
    {
        timing_open_done_today  = 0;
        timing_close_done_today = 0;
        timing_last_reset_hour  = 0;
    }
    else if(cur_hour != 0)
        timing_last_reset_hour = cur_hour;

    if(!timing_open_done_today && g_valve_timing.open_time > 0)
    {
        diff = (int32_t)now_sec - (int32_t)hhmmss_to_seconds(g_valve_timing.open_time);
        if(diff >= 0 && diff <= 30)
        {
            vtiming_execute_and_report("open");
            timing_open_done_today = 1;
        }
    }
    if(!timing_close_done_today && g_valve_timing.close_time > 0)
    {
        diff = (int32_t)now_sec - (int32_t)hhmmss_to_seconds(g_valve_timing.close_time);
        if(diff >= 0 && diff <= 30)
        {
            vtiming_execute_and_report("close");
            timing_close_done_today = 1;
        }
    }
}

static void vtiming_execute_and_report(const char *action)
{
    ensure_full_init();
    IIC_Init();
    motor_init();
    const char *vr = valve_execute(action, 1);
    printf_debug("[SM] VTIMING: %s result=%s\r\n", action, vr);

    int was_online;
    int ec_ok = fourG_ensure_ready(&was_online);
    if(ec_ok)
        EC801E_PublishValveStatus(vr, g_valve_last_order);
    fourG_restore(was_online, ec_ok);
}

// ==================== 实时路径: 配置唤醒�?深休�?====================
static void realtime_enter_sleep(void)
{
    uint8_t lpuart_enabled = 0;

    gpio_bit_reset(GPIOD, GPIO_PIN_4);
    rtc_alarm_wakeup_init();
    rtc_set_next_alarm_sec(g_real_frequency);
    g_timing_alarm_set = 0;   /* realtime alarm, timing handled per-cycle */

    // 4G在线时使能LPUART唤醒 + RI唤醒 (接收平台下发, 弱信号断电时不启用)
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
    enter_deepsleep_mode(lpuart_enabled);
}

// ==================== 实时采集最小化唤醒路径 ====================
// 仅初始化 SysTick + I2C + PD4, 采集传感�? 判断异常
// 异常时立即重采确�? 确认后启4G上报; 正常则直接回�?
// ==================== ==================== ====================
static void do_realtime_check(void)
{
    ThresholdResult_t result;
    uint8_t sensor_err;
    uint8_t still_abnormal = 1;
    uint8_t i;

    // === 最小初始化 (时钟恢复) ===
    system_wakeup_reinit();

    // 软件看门�? 实时采集路径�?do_sleep 循环直接调用(不经主循环分�?, 需独立使能
    // SysTick 已由 system_wakeup_reinit 重新配置; 回睡�?enter_deepsleep_mode 会关�?
    swdt_enable(SWDT_TIMEOUT_MS);

    // === 采集 (仅传感器, 不读ADC; PD4/IIC �?CollectAll 内部管理) ===
    PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);

    // === LCD超时检�?+ 采样周期刷新显示 ===
    lcd_check_timeout();
    if(g_lcd_active) lcd_ui_refresh(&g_sensor.raw, &g_sensor.raw2, g_sensor.voltage);

    // === 传感器在�?离线状态变化检�?(带消�? 进气�?出气�? ===
    {
        uint8_t cur_online  = (g_sensor.raw.err  == PT304D_OK) ? 1 : 0;
        uint8_t cur_online2 = (g_sensor.raw2.err == PT304D_OK) ? 1 : 0;
        uint8_t state_changed = sensor_debounce_update(cur_online, cur_online2);

        if(state_changed)
        {
            ensure_full_init();
            trea_uart_init();
            {
                rtc_get_time_str(s_status_ts, sizeof(s_status_ts));
                printf_debug("[RT] %s SENSOR STATE CHANGED: S2=%s(err=%d) S1=%s(err=%d) freq=%ds\r\n",
                             s_status_ts,
                             cur_online  ? "ON" : "OFF", g_sensor.raw.err,
                             cur_online2 ? "ON" : "OFF", g_sensor.raw2.err,
                             g_real_frequency);
            }

            // 立即4G上报
            {
                int was_online;
                int ec_ok = fourG_ensure_ready(&was_online);
                if(ec_ok)
                {
                    publish_sensor_status();
                    printf_debug("[RT] Sensor state reported\r\n");
                }
                fourG_restore(was_online, ec_ok);
            }

            // 进气端离线时跳过阈值检�? 直接回睡
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
            // 持续离线(含消抖期�?, 不重复上�? 直接回睡
            realtime_enter_sleep();
            return;
        }
    }

    // === 三重异常检�?===
    // 实时路径不采 ADC: 传哨�?-1.0f (配合 check_voltage=0 + threshold_check 守卫) 明确"未采集电�?
    // skip_mutation=1: 恢复判定时跳过突变检测, 避免压力回归正常时的变化量触发虚假突变报警
    result     = threshold_check(g_sensor.raw.pressure, g_sensor.raw.temperature, -1.0f, g_sensor.last_pressure_realtime, 0, 1);
    if(result == THRESHOLD_OK && g_sensor_count >= 2 && g_sensor.raw2.err == PT304D_OK)
        result = threshold_check2(g_sensor.raw2.pressure, g_sensor.raw2.temperature, g_sensor.last_pressure_realtime2, 1);
    sensor_err = (g_sensor.raw.err != PT304D_OK);

    // === 正常 �?检查报警恢�?�?更新压力 �?直接回睡 ===
    if(result == THRESHOLD_OK && !sensor_err)
    {
        // 报警恢复: 之前自动关阀, 现传感器正常 �?恢复阀门到关阀前状�?
        // (�?do_collect 末尾恢复逻辑一�? 实时模式无需等待下一采集周期)
        if(g_valve_alarm_closed && !voltage_low_power)
        {
            const char *restore_action = (g_valve_pre_alarm_order == 1) ? "open" : "close";
            ensure_full_init();
            trea_uart_init();
            printf_debug("[RT] ALARM recovered, restoring valve to %s (pre=%d)\r\n",
                         restore_action, g_valve_pre_alarm_order);

            IIC_Init();
            motor_init();
            const char *vr = valve_execute(restore_action, 0);
            g_valve_alarm_closed = 0;
            config_save_ext();

            int was_online;
            int ec_ok = fourG_ensure_ready(&was_online);
            if(ec_ok)
            {
                EC801E_PublishValveStatus(vr, g_valve_last_order);
                EC801E_PublishAlarmStatus(THRESHOLD_OK, g_sensor.last_normal_voltage);
                publish_all_status(g_sensor.last_pressure_realtime, g_sensor.last_pressure_realtime2);
            }
            fourG_restore(was_online, ec_ok);
        }
        else if(last_realtime_alarm != THRESHOLD_OK)
        {
            // 之前有报警但未关阀(策略为none等), 现恢复正常, 需上报恢复状态
            ensure_full_init();
            int was_online;
            int ec_ok = fourG_ensure_ready(&was_online);
            if(ec_ok)
            {
                EC801E_PublishAlarmStatus(THRESHOLD_OK, g_sensor.last_normal_voltage);
                publish_all_status(g_sensor.last_pressure_realtime, g_sensor.last_pressure_realtime2);
            }
            fourG_restore(was_online, ec_ok);
        }

        if(last_realtime_alarm != THRESHOLD_OK)
            last_realtime_alarm = THRESHOLD_OK;   // 恢复正常, 清除防重复标�?

        // 静态限值正常后, 补充突变检测 (skip_mutation=0)
        // 恢复上报已完成, 此处仅检测是否存在新的纯突变异常
        result = threshold_check(g_sensor.raw.pressure, g_sensor.raw.temperature, -1.0f, g_sensor.last_pressure_realtime, 0, 0);
        if(result == THRESHOLD_OK && g_sensor_count >= 2 && g_sensor.raw2.err == PT304D_OK)
            result = threshold_check2(g_sensor.raw2.pressure, g_sensor.raw2.temperature, g_sensor.last_pressure_realtime2, 0);

        g_sensor.last_pressure_realtime = g_sensor.raw.pressure;
        g_sensor.last_pressure_realtime2 = g_sensor.raw2.pressure;

        if(result == THRESHOLD_OK)
        {
            realtime_enter_sleep();
            return;
        }
        // 突变异常 → 继续往下走异常确认流程
    }

    // === 异常 → 防重复: 同类异常不重复上报 ===
    if(result == last_realtime_alarm && result != THRESHOLD_OK)
    {
        g_sensor.last_pressure_realtime = g_sensor.raw.pressure;
        g_sensor.last_pressure_realtime2 = g_sensor.raw2.pressure;
        realtime_enter_sleep();
        return;
    }

    // === 异常 �?立即重采确认 (3�? ===
    for(i = 0; i < APP_ALARM_RECHECK_COUNT; i++)
    {
        trea_delay_ms(APP_ALARM_RECHECK_DELAY_MS);
        PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);

        result     = threshold_check(g_sensor.raw.pressure, g_sensor.raw.temperature, -1.0f, g_sensor.last_pressure_realtime, 0, 0);
        if(result == THRESHOLD_OK && g_sensor_count >= 2 && g_sensor.raw2.err == PT304D_OK)
            result = threshold_check2(g_sensor.raw2.pressure, g_sensor.raw2.temperature, g_sensor.last_pressure_realtime2, 0);
        sensor_err = (g_sensor.raw.err != PT304D_OK);

        if(result == THRESHOLD_OK && !sensor_err)
        {
            still_abnormal = 0;
            break;
        }
    }

    if(!still_abnormal)
    {
        // 偶发抖动 �?回睡
        g_sensor.last_pressure_realtime = g_sensor.raw.pressure;
        g_sensor.last_pressure_realtime2 = g_sensor.raw2.pressure;
        realtime_enter_sleep();
        return;
    }

    // === 确认异常 �?记录防重复标�?===
    last_realtime_alarm = result;

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
            EC801E_PublishAlarmStatus(result, g_sensor.last_normal_voltage);
            if(sensor_err)
                publish_sensor_status();
            if(vr)
                EC801E_PublishValveStatus(vr, g_valve_last_order);

            // 用户端MQTT同步上报报警数据
            user_mqtt_try_publish();
        }
        fourG_restore(was_online, ec_ok);
    }

    // === 回睡 ===
    g_sensor.last_pressure_realtime = g_sensor.raw.pressure;
    g_sensor.last_pressure_realtime2 = g_sensor.raw2.pressure;
    realtime_enter_sleep();
}

// ==================== 进入休眠 ====================
static void do_sleep(void)
{
    int fourG_keep_on = 0;


    gpio_bit_reset(GPIOA, GPIO_PIN_5);
    gpio_bit_reset(GPIOD, GPIO_PIN_4);

    gpio_bit_reset(GPIOA, GPIO_PIN_2);
    gpio_bit_reset(GPIOA, GPIO_PIN_3);
    gpio_bit_reset(GPIOC, GPIO_PIN_4);
    gpio_bit_reset(GPIOC, GPIO_PIN_5);
    gpio_bit_reset(GPIOB, GPIO_PIN_8);
    // PB9 = SLCD COM3, 不关�?
    // PB6/PB7(I2C总线2 SCL/SDA): 不主动拉低, 由 gpio_enter_low_power 切 Analog
    // 避免在传感器断电过程中产生异常 I2C 信号导致下次上电后总线锁死

    gpio_bit_reset(GPIOA, GPIO_PIN_6);
    gpio_bit_reset(GPIOA, GPIO_PIN_7);

    // 4G电源管理: 根据window pattern决定PB5(V4G_EN)状�?
    // 低电压省电模�? 强制4G断电
    if(voltage_low_power)
    {
        gpio_bit_reset(GPIOB, GPIO_PIN_5);
    }
    else if(strcmp(g_window.pattern, "long") == 0)
    {
        // long模式: 窗口内或end_result=1时保�?G (用户主动关机/弱信号除外)
        if(!fourG_user_off && !fourG_init_failed && !fourG_weak_signal && (config_is_active_window() || g_window.end_result == 1))
            fourG_keep_on = 1;
        else
            gpio_bit_reset(GPIOB, GPIO_PIN_5);  // long+er=0+窗口�?user_off/weak_signal: 4G断电
    }
    else
    {
        // short/dot: 4G始终断电(由do_upload在上报完后已PowerOff)
        gpio_bit_reset(GPIOB, GPIO_PIN_5);
    }

    // === LCD 延时关闭: LCD仍在显示�? 先短休眠�?0秒超时再关闭 ===
    // 实时模式(1-10s唤醒)�?do_realtime_check �?lcd_check_timeout() 自然处理, 此处跳过
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
            enter_deepsleep_mode(fourG_keep_on);

            // 唤醒: 按键或LPUART或RI或BLE连接 → 直接进入 STATE_WAKEUP
            if(button_wakeup_flag || counter0 || ri_wakeup_flag || ble_connect_wakeup_flag)
            {
                app_state = STATE_WAKEUP;
                return;
            }
        }
        // RTC 超时或已过期: 关闭 LCD
        lcd_shutdown();
        printf_debug("[SM] LCD off after timeout\r\n");
    }

    // 设置RTC闹钟（紧贴深休眠前，防止闹钟时间已过�?
    rtc_alarm_wakeup_init();

    // 定时阀门: 计算最近一个未执行的阀门事件距当前的秒数
    {
        uint8_t use_valve_alarm = 0;
        uint32_t valve_wait_sec = 0xFFFFFFFF;

        if(g_valve_timing.enabled && !g_real_enabled)
        {
            uint32_t now_sec = hhmmss_to_seconds(rtc_get_hhmmss());
            if(!timing_open_done_today)
            {
                uint32_t t = hhmmss_to_seconds(g_valve_timing.open_time);
                uint32_t w = (t > now_sec) ? (t - now_sec) : (86400 - now_sec + t);
                if(w < valve_wait_sec) valve_wait_sec = w;
            }
            if(!timing_close_done_today)
            {
                uint32_t t = hhmmss_to_seconds(g_valve_timing.close_time);
                uint32_t w = (t > now_sec) ? (t - now_sec) : (86400 - now_sec + t);
                if(w < valve_wait_sec) valve_wait_sec = w;
            }

            // 与正常采集间隔比较, 取更早的
            uint32_t collect_sec = (uint32_t)g_interval.collect_interval_min * 60;
            if(valve_wait_sec < collect_sec && valve_wait_sec < 0xFFFFFFFF)
            {
                // 阀门事件更早: 直接设定绝对时间闹钟
                uint32_t target;
                if(!timing_open_done_today)
                {
                    uint32_t t_open = hhmmss_to_seconds(g_valve_timing.open_time);
                    uint32_t w_open = (t_open > now_sec) ? (t_open - now_sec) : (86400 - now_sec + t_open);
                    if(!timing_close_done_today)
                    {
                        uint32_t t_close = hhmmss_to_seconds(g_valve_timing.close_time);
                        uint32_t w_close = (t_close > now_sec) ? (t_close - now_sec) : (86400 - now_sec + t_close);
                        target = (w_open <= w_close) ? g_valve_timing.open_time : g_valve_timing.close_time;
                    }
                    else
                        target = g_valve_timing.open_time;
                }
                else
                    target = g_valve_timing.close_time;

                rtc_set_alarm_at_hhmmss(target);
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

    // LPUART唤醒 + RI唤醒: 4G有电时才有意义(可接收下发URC)
    if(fourG_keep_on)
    {
        com_lpuart_init();
        lpuart_wakeup_init();
        ri_exti_init();
    }

    // PA12按键唤醒: 始终启用(用于手动开�?G)
    button_exti_init();

    // BLE连接事件未处理(与按键同时唤醒), 回到WAKEUP处理
    if(ble_connect_wakeup_flag)
    {
        app_state = STATE_WAKEUP;
        return;
    }

    // BLE开启时: 检查60s未连接超时, 未超时则使能PD2 EXTI唤醒
    if(g_bt_on)
    {
        uint32_t now_sec = hhmmss_to_seconds(rtc_get_hhmmss());
        uint32_t on_sec  = hhmmss_to_seconds(g_ble_on_hhmmss);
        uint32_t elapsed = (now_sec >= on_sec) ? (now_sec - on_sec) : (86400 - on_sec + now_sec);
        if(elapsed >= 60)
        {
            printf_debug("[SM] BLE timeout (%ds), auto power off\r\n", (int)elapsed);
            g_bt_on = 0;
            gpio_bit_reset(GPIOC, GPIO_PIN_12);
            if(g_lcd_active) slcd_seg_icon_display(SLCD_ICON_LY, RESET);
        }
        else
        {
            ble_exti_init();
        }
    }

    // 深休眠前禁用DMA(深休眠期间DMA时钟被门�? 需清理状�?
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

    enter_deepsleep_mode(fourG_keep_on);

    // 实时采集循环: RTC秒级唤醒 → 检查是否到采集网格 → 否则最小采集回睡
    while(g_real_enabled && rtc_alarm_flag && !counter0 && !ri_wakeup_flag && !button_wakeup_flag && !ble_connect_wakeup_flag)
    {
        // 最小时钟恢�?�?读RTC判断是否到达正常采集网格
        system_wakeup_reinit();
        rtc_register_sync_wait();

        {
            uint32_t now_hhmmss = rtc_get_hhmmss();
            uint16_t cur_min = (uint16_t)(now_hhmmss / 10000) * 60
                             + (uint16_t)((now_hhmmss / 100) % 100);
            uint8_t cur_sec = (uint8_t)(now_hhmmss % 100);

            // 到达正常采集网格 (整分钟�?s容差, 且当前分钟未执行过采�? �?退出循�? 走完整STATE_WAKEUP
            if(g_interval.collect_interval_min > 0 &&
               (cur_min % g_interval.collect_interval_min) == 0 &&
               cur_sec < 6 &&
               cur_min != last_grid_collect_min)
                break;

            // ---- 实时模式下的定时阀门检查 ----
            if(g_valve_timing.enabled)
            {
                uint32_t now_sec = hhmmss_to_seconds(now_hhmmss);
                uint8_t cur_hour = (uint8_t)(now_hhmmss / 10000);

                // 午夜重置当日执行标志
                if(cur_hour == 0 && timing_last_reset_hour != 0)
                {
                    timing_open_done_today  = 0;
                    timing_close_done_today = 0;
                    timing_last_reset_hour  = 0;
                }
                else if(cur_hour != 0)
                    timing_last_reset_hour = cur_hour;

                // 检查开阀 (容差±5秒, 实时模式频率高无需30s)
                if(!timing_open_done_today && g_valve_timing.open_time > 0)
                {
                    uint32_t tgt = hhmmss_to_seconds(g_valve_timing.open_time);
                    int32_t diff = (int32_t)now_sec - (int32_t)tgt;
                    if(diff >= 0 && diff <= 5)
                    {
                        printf_debug("[SM] VTIMING-RT: OPEN at %06lu\r\n", now_hhmmss);
                        vtiming_execute_and_report("open");
                        timing_open_done_today = 1;
                    }
                }
                // 检查关阀 (容差±5秒)
                if(!timing_close_done_today && g_valve_timing.close_time > 0)
                {
                    uint32_t tgt = hhmmss_to_seconds(g_valve_timing.close_time);
                    int32_t diff = (int32_t)now_sec - (int32_t)tgt;
                    if(diff >= 0 && diff <= 5)
                    {
                        printf_debug("[SM] VTIMING-RT: CLOSE at %06lu\r\n", now_hhmmss);
                        vtiming_execute_and_report("close");
                        timing_close_done_today = 1;
                    }
                }
            }
        }

        // 非采集网�?�?最小化实时采集 (内部: 采集→判断→[可能上报]→设闹钟→深休眠→唤醒后返回)
        do_realtime_check();
        // do_realtime_check 返回�? 新的唤醒标志已置�? while再次判断
    }

    // 唤醒后立即转�?WAKEUP 状�?
    app_state = STATE_WAKEUP;
}

// ==================== 按键单按（短按）处理 ====================
// 当用户短�?PA12（小�?秒释放）时调用。打印配置、当前实时传感器数据，并连接 4G 平台完整上报当时的状态�?
static void do_button_short_press(void)
{
    printf_debug("\r\n==================================================\r\n");
    printf_debug("[SM] BUTTON SHORT PRESS: Starting manual report & config dump\r\n");

    // 1. 打印当前系统配置
    config_print_current();
    
    // 2. 立即采集当前实时传感器数据并打印
    printf_debug("[SM] Sampling live sensor data...\r\n");
    PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);
    
    sensor_sample_voltage(1, g_sensor.last_normal_voltage);
    
    printf_debug("--- Live Data ---\r\n");
    printf_debug("  S2(inlet):  P=%.2f Pa  T=%.2f C  err=%d\r\n", g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.raw.err);
    printf_debug("  S1(outlet): P=%.2f Pa  T=%.2f C  err=%d\r\n", g_sensor.raw2.pressure, g_sensor.raw2.temperature, g_sensor.raw2.err);
    printf_debug("  Voltage:    %.2f V\r\n", g_sensor.voltage);
    printf_debug("==================================================\r\n\r\n");

    // 3. 4G连接 (复用公共模块)
    int was_online;
    int ec_ok = fourG_ensure_ready(&was_online);

    if(ec_ok)
    {
        lcd_update_csq();

        // A. 上报传感器数据
        EC801E_PublishSensorData(&g_sensor.raw, g_sensor.voltage);
        trea_delay_ms(700);

        // B. 上报传感器在线状态
        publish_sensor_status();
        trea_delay_ms(700);

        // C. 上报压力+温度状态 (复用公共模块)
        publish_all_status(g_sensor.last_pressure_collect, g_sensor.last_pressure_collect2);
        trea_delay_ms(700);

        // D. 上报最后阀门指令
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

    // 4. 等待按键松手，防止释放抖动再次触发
    while(button_is_pressed())
        trea_delay_ms(100);
    trea_delay_ms(50);

    // 5. 收尾：等待5s 接收平台下行指令
    printf_debug("[SM] BUTTON: waiting 5s for downlink...\r\n");
    trea_delay_ms(5000);
    {
        uint8_t dl_count = drain_pending_downlink();
        if(dl_count > 0)
            printf_debug("[SM] BUTTON: processed %d downlink msg(s)\r\n", dl_count);
        else
            printf_debug("[SM] BUTTON: no downlink within 5s\r\n");
    }

    // 6. 恢复4G状态 (复用公共模块)
    fourG_restore(was_online, ec_ok);
}

// ==================== 唤醒恢复 ====================
static void do_wakeup(void)
{
    // 判断是否为纯周期采集定时唤醒 (非按键且非LPUART且非RI且非BLE连接)
    if(button_wakeup_flag == 0 && counter0 == 0 && ri_wakeup_flag == 0 && ble_connect_wakeup_flag == 0)
    {
        g_is_minimal_wakeup = 1;
        g_need_adc_sampling = 0;
        
        system_wakeup_reinit();
        
        // 极简采集初始�? Systick、Debug 串口发�? PD4(SW_IIC)/IIC 引脚�?PT304D_CollectAll 内部初始�?
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
        // === 按键唤醒 (快速检测路�?: 最小初始化, 跳过 sys_init() �?1s 延时 ===
        // GPIO 状态在深休眠期间保�? PD0 仍为输入上拉, button_is_pressed() 可直接使�?
        // 需要完整外设时�?ensure_full_init() 按需升级
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
        // === LPUART/RI 唤醒或混合唤醒: 完整初始化 ===
        g_is_minimal_wakeup = 0;
        g_need_adc_sampling = 1;
        
        system_wakeup_reinit();
        sys_init();
        
        // 禁用按键EXTI(唤醒后立即禁止抨动重复触�?
        button_exti_disable();
        
        // PB5(V4G_EN 4G电源)恢复: gpio_init默认PB5=LOW
        // 低电压省电模�? 不恢�?G
        // 用户主动关机: 不恢�?G(等待按键重新开�?
        // 弱信号模式: 不恢�?G(下次上报冷启动)
        // long模式: 窗口内或end_result=1时恢�?G供电
        if(!voltage_low_power && !fourG_user_off && !fourG_init_failed && !fourG_weak_signal &&
           strcmp(g_window.pattern, "long") == 0 &&
           (config_is_active_window() || g_window.end_result == 1))
            gpio_bit_set(GPIOB, GPIO_PIN_5);
        // 其余情况(VLP/user_off/weak_signal/short/dot/long+er=0+窗口�?: PB5保持LOW
        
        rtc_show_time();
        printf_debug("\r\n");
    }

    // === 按键唤醒: 连按检�?+ 长按4G开�?===
    if(button_wakeup_flag)
    {
        printf_debug("[SM] WAKEUP: BUTTON\r\n");

        // 按键唤醒恢复LCD显示 (若已关闭则重新初始化)
        uint8_t lcd_was_off = !g_lcd_active;
        lcd_activate();
        if(g_lcd_active)
            lcd_ui_refresh(&g_sensor.raw, &g_sensor.raw2, g_sensor.voltage);

        button_wakeup_flag = 0;

        // LCD从关闭恢�? 本次按键仅激活屏�? 不做UI操作(翻页�?
        if(lcd_was_off && g_lcd_active)
        {
            buzzer_beep(1, 200);
            printf_debug("[SM] BUTTON: LCD activated (consume press)\r\n");
            app_state = STATE_SLEEP;
            return;
        }

        // 消抖延时: �?0ms等机械前沿抖动结�?
        trea_delay_ms(20);

        // 确认当前按键状�?
        uint8_t is_short_press = 0;

        if(!button_is_pressed())
        {
            // 20ms内已松开 �?短按
            is_short_press = 1;
        }
        else
        {
            // 按键仍持�? 检查是否长�?4s)
            uint8_t i;
            for(i = 0; i < 40; i++)
            {
                trea_delay_ms(100);
                if(!button_is_pressed())
                {
                    // 4s内松开 �?短按
                    is_short_press = 1;
                    break;
                }
            }
        }

        if(is_short_press)
        {
            // === 短按: 进入连按计数 (500ms窗口) ===
            uint8_t press_count = 1;
            uint16_t wait_ms;

            // 等待松手 (如果还按着)
            while(button_is_pressed())
                trea_delay_ms(20);
            trea_delay_ms(30);  // 松手消抖

            // 500ms窗口内检测后续按�?
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
                    break;  // 500ms内无新按�? 结束计数

                press_count++;
                // 等待本次按键松手
                while(button_is_pressed())
                    trea_delay_ms(20);
                trea_delay_ms(30);  // 松手消抖
            }

            printf_debug("[SM] BUTTON: %d press(es) detected\r\n", press_count);

            switch(press_count)
            {
                
                case 1:
                    // 1次短按? LCD页面切换 + 蜂鸣确认
                    buzzer_beep(1, 200);
                    if(g_lcd_active) lcd_ui_next_page();
                        printf_debug("[SM] BUTTON: LCD page -> %d\r\n", g_lcd_page);
                    break;

                case 2:
                    // 2次短按? 数据上报 (需完整外设)
                    ensure_full_init();
                    do_button_short_press();
                    break;

                case 3:
                    // 3次短按: 蓝牙开关切换 (非阻塞: 仅上/断电, 不进入BLE_OTA)
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
                    // 4次短按，4G开
                    fourG_user_off = 0;
                    fourG_weak_signal = 0;  // 清除弱信号标志
					printf_debug("4G_user_on,Press the button twice in quick succession");
                    break;

                default:
                    break;
            }

            app_state = STATE_SLEEP;
            return;
        }

        // === 长按4s确认 �?4G开机联�?(需完整外设) ===
        ensure_full_init();
        printf_debug("[SM] BUTTON: 4s confirmed, powering on 4G\r\n");
        fourG_user_off = 0;  // 清除用户关机标志
        fourG_weak_signal = 0;  // 手动开机清除弱信号标志
        com_lpuart_init();
        trea_delay_ms(100);
        EC801E_PowerOn();         // PD9 拉高 (idempotent, �?HIGH 时不影响)
        EC801E_InitAndConnect();

        // 验证4G模块AT响应 + 上报开机状�?
        if(EC801E_EnsureConnected() == EC_OK)
        {
            fourG_init_failed = 0;  // 手动开机成功, 清除失败标志
            report_4g_power_state(1);
            lcd_update_csq();
            buzzer_beep(2, 200);
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

        // === 统一等待 20s 接收下行指令 (所有模�? ===
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
            // long模式: 保持4G在线 (do_sleep �?fourG_keep_on 路径接管)
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

// ==================== 确保完整初始化?====================
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
// 根据当前模式和时间判断是否需要批量上报历史数�? 设置 app_state
static void check_upload_schedule(void)
{
    // === 低电压省电模�? 覆盖正常上报调度, �?:00/18:00上报 ===
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

    // 判断是否需要上�?(正常模式, 基于时间对齐)
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

// ==================== 采集传感�?+ 阈值检�?====================
static void do_collect(void)
{
    SensorRecord_t record;
    ThresholdResult_t result;
    uint8_t vlp_just_exited = 0;

    // ========== 定时开关阀检查 ==========
    if(g_valve_timing.enabled)
    {
        uint32_t now_hhmmss = rtc_get_hhmmss();
        uint32_t now_sec = hhmmss_to_seconds(now_hhmmss);
        uint8_t cur_hour = (uint8_t)(now_hhmmss / 10000);

        // 午夜重置当日执行标志 (00:00~00:59, 且未重置过)
        if(cur_hour == 0 && timing_last_reset_hour != 0)
        {
            timing_open_done_today  = 0;
            timing_close_done_today = 0;
            timing_last_reset_hour  = 0;
            printf_debug("[SM] VTIMING: daily flags reset\r\n");
        }
        else if(cur_hour != 0)
            timing_last_reset_hour = cur_hour;

        // 检查开阀时间 (容差±30秒)
        if(!timing_open_done_today && g_valve_timing.open_time > 0)
        {
            uint32_t tgt_sec = hhmmss_to_seconds(g_valve_timing.open_time);
            int32_t diff = (int32_t)now_sec - (int32_t)tgt_sec;
            if(diff >= 0 && diff <= 30)
            {
                printf_debug("[SM] VTIMING: OPEN triggered at %06lu\r\n", now_hhmmss);
                vtiming_execute_and_report("open");
                timing_open_done_today = 1;
            }
        }

        // 检查关阀时间 (容差±30秒)
        if(!timing_close_done_today && g_valve_timing.close_time > 0)
        {
            uint32_t tgt_sec = hhmmss_to_seconds(g_valve_timing.close_time);
            int32_t diff = (int32_t)now_sec - (int32_t)tgt_sec;
            if(diff >= 0 && diff <= 30)
            {
                printf_debug("[SM] VTIMING: CLOSE triggered at %06lu\r\n", now_hhmmss);
                vtiming_execute_and_report("close");
                timing_close_done_today = 1;
            }
        }
    }

    // 记录本次正常周期采集的分钟戳，防止秒级高频实时监测循环在同一分钟内多次唤醒采�?
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

    // 实时模式关闭�? 采集周期兼作 LCD 刷新 (实时模式开启时�?do_realtime_check 刷新)
    if(g_lcd_active && !g_real_enabled)
        lcd_ui_refresh(&g_sensor.raw, &g_sensor.raw2, g_sensor.voltage);

    // ========== 传感器状态变化检�?(带消�? 进气�?出气�? ==========
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

            // 立即4G上报
            {
                int was_online;
                int ec_ok = fourG_ensure_ready(&was_online);
                if(ec_ok)
                {
                    publish_sensor_status();
                    printf_debug("[SM] Sensor state reported\r\n");
                }
                fourG_restore(was_online, ec_ok);
            }

            // 进气端离线时跳过阈值检查和数据缓存, 直接回休�?
            if(!cur_online)
            {
                app_state = STATE_SLEEP;
                return;
            }
        }
        else if(!cur_online)
        {
            // 持续离线(含消抖期�? �?不重复上�? 跳过采集直接回休�?
            printf_debug("[SM] SENSOR still OFFLINE (cnt=%d/%d cnt2=%d/%d), skip\r\n",
                         g_sensor.offline_cnt, SENSOR_OFFLINE_DEBOUNCE,
                         g_sensor.offline_cnt2, SENSOR_OFFLINE_DEBOUNCE);
            app_state = STATE_SLEEP;
            return;
        }
    }

    // 阈值检�?
    result = threshold_check(g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage, g_sensor.last_pressure_collect, 0, 0);
    // 第二路阈值检查 (仅在双传感器且第二路在线时)
    if(result == THRESHOLD_OK && g_sensor_count >= 2 && g_sensor.raw2.err == PT304D_OK)
        result = threshold_check2(g_sensor.raw2.pressure, g_sensor.raw2.temperature, g_sensor.last_pressure_collect2, 0);

    if(result != THRESHOLD_OK)
    {
        // 低电压省电模�? 已进入过VPROTECT报警, 后续VPROTECT不再重复报警
        if(voltage_low_power && result == THRESHOLD_VPROTECT)
        {
            printf_debug("[SM] VLP: still low voltage, skip alarm\r\n");
            // 不进入ALARM, 继续走低电压省电调度
        }
        else
        {
            // 首次VPROTECT会进入ALARM(关阀+报警), 同时标记低电压模�?
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

    // 低电压恢�? 以下任一条件触发退�?VLP
    //   (a) 电压回到 vprotect 以上 (正常恢复)
    //   (b) spupower.switch 被用户关�?(不再执行电压检�? 不能残留 VLP 状态锁�?
    if(voltage_low_power && (!g_spupower.enabled || g_sensor.voltage >= g_spupower.protect + 0.15f))
    {
        voltage_low_power = 0;
        config_save_ext();
        vlp_uploaded_this_hour = 0;
        vlp_just_exited = 1;     // 标记: 本次 collect �?VLP 已退�? 需�?normal 上报
        if(!g_spupower.enabled)
            printf_debug("[SM] VLP: spupower disabled by user, exiting low power mode\r\n");
        else
            printf_debug("[SM] VLP: voltage recovered (%.2fV >= %.2fV), exiting low power mode\r\n",
                         g_sensor.voltage, g_spupower.protect);
    }

    // 恢复正常上报: (a) 之前报警自动关阀 �?还原阀�?+ alarm:normal
    //               (b) VLP 刚退�?�?alarm:normal (即使没触发关阀, 也通知平台脱离低压保护)
    // 共享 4G 启停, 避免重复 cold-start
    if((g_valve_alarm_closed || vlp_just_exited) && !voltage_low_power)
    {
        ensure_full_init();
        const char *restore_action = NULL;
        const char *vr = NULL;

        // (a) 阀门恢�?(仅当之前自动关阀时执�?
        if(g_valve_alarm_closed)
        {
            restore_action = (g_valve_pre_alarm_order == 1) ? "open" : "close";
            printf_debug("[SM] ALARM recovered, restoring valve to %s (pre=%d)\r\n",
                         restore_action, g_valve_pre_alarm_order);
            vr = valve_execute(restore_action, 0);
            g_valve_alarm_closed = 0;
            config_save_ext();   // 持久�? 清除恢复标志
        }
        else if(vlp_just_exited)
        {
            printf_debug("[SM] VLP recovered, publishing normal status\r\n");
        }

        // 4G 启停 + 对称恢复上报
        {
            int was_online;
            int ec_ok = fourG_ensure_ready(&was_online);
            if(ec_ok)
            {
                if(vr != NULL)
                {
                    EC801E_PublishValveStatus(vr, g_valve_last_order);
                }
                EC801E_PublishAlarmStatus(THRESHOLD_OK, g_sensor.voltage);
                publish_all_status(g_sensor.last_pressure_collect, g_sensor.last_pressure_collect2);
            }
            fourG_restore(was_online, ec_ok);
        }
    }

    // 正常: 存入缓冲�?
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

    // 上报调度判断
    check_upload_schedule();
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

        PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);
        sensor_sample_voltage(0, g_sensor.last_normal_voltage);

        result = threshold_check(g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage, g_sensor.last_pressure_collect, 1, 0);
        if(result == THRESHOLD_OK && g_sensor_count >= 2 && g_sensor.raw2.err == PT304D_OK)
            result = threshold_check2(g_sensor.raw2.pressure, g_sensor.raw2.temperature, g_sensor.last_pressure_collect2, 0);
        printf_debug("[SM] ALARM recheck %d/%d: P=%.2f T=%.2f V=%.2f P2=%.2f T2=%.2f result=%d\r\n",
                     i + 1, APP_ALARM_RECHECK_COUNT, g_sensor.raw.pressure, g_sensor.raw.temperature, g_sensor.voltage,
                     g_sensor.raw2.pressure, g_sensor.raw2.temperature, result);

        if(result == THRESHOLD_OK)
        {
            // 恢复正常: 偶发抖动
            printf_debug("[SM] ALARM: recovered at recheck %d, false alarm\r\n", i + 1);
            still_abnormal = 0;
            break;
        }
    }

    // 构造记�?
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

        // 判断报警类型 (�?do_realtime_check/downlink 共用 threshold_alarm_type_str)
        alarm_type = threshold_alarm_type_str(result);

        printf_debug("[SM] *** ALARM CONFIRMED: %s ***\r\n", alarm_type);

        // Debug串口打印报警
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
                EC801E_PublishAlarmStatus(result, g_sensor.voltage);
                if(vr)
                    EC801E_PublishValveStatus(vr, g_valve_last_order);

                // 用户端MQTT同步上报报警数据
                user_mqtt_try_publish();
            }
            fourG_restore(was_online, ec_ok);
        }
    }
    else
    {
        // 偶发抖动, 正常记录
        record.flags = 0;
    }

    // 存入EEPROM历史缓冲
    ee_hist_push(record.pressure, record.temperature,
                 record.pressure2, record.temperature2,
                 record.voltage, record.flags);
    g_sensor.last_pressure_collect = g_sensor.raw.pressure;
    g_sensor.last_pressure_collect2 = g_sensor.raw2.pressure;
    wakeup_count++;

    // 上报调度判断 (报警与上报周期重合时, 历史数据仍需批量上报)
    check_upload_schedule();
}

// ==================== 数据上报 (批量逐条, 三模式) ====================
static void do_upload(void)
{
    uint16_t n, i;
    int is_coldstart = 0;

    // === 窗口检�?(dot/VLP模式由do_collect已确认时间点, 直接通过) ===
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

    // === 建立4G连接 ===
    com_lpuart_init();
    trea_delay_ms(100);

    // 低电压省电模式 / 弱信号模式: 强制走冷启动路径
    if(voltage_low_power || strcmp(g_window.pattern, "long") != 0 || fourG_weak_signal)
    {
        // short/dot/VLP/weak_signal: 4G断电状态, 需完整开机
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
        // long模式(非VLP, 非弱信号): 4G应已连接(QSCLK), 直接确认
        if(EC801E_EnsureConnected() != EC_OK)
        {
            printf_debug("[SM] UPLOAD: connect failed, power off\r\n");
            EC801E_PowerOff();
            app_state = STATE_SLEEP;
            return;
        }
        fourG_init_failed = 0;  // EnsureConnected 成功, 清除失败标志
    }

    // 4G连接成功后更新LCD信号强度
    lcd_update_csq();

    // === 用户端MQTT: 预连接 (历史数据同步上报, 含1次重试) ===
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

    // === 逐条上传 (双通道: 主平台id=0 + 用户端id=1, 从EEPROM读取) ===
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
            // CRC校验失败或读取错误, 跳过损坏记录
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

    // 上报汇总状态 (传感器在线+信号+压力温度状态+阀门+IP+电压)
    trea_delay_ms(200);
    publish_upload_summary();
    printf_debug("[SM] upload summary published\r\n");

    // 上传流程结束，复位唤醒周期统计计�?
    wakeup_count = 0;
    printf_debug("[SM] UPLOAD finished, remaining records in buffer: %d\r\n", ee_hist_count());

    // === RTC校准: 利用4G在线从网络同步时�?===
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
        // 冷启动模式(VLP/short/dot/weak_signal): 等待20s接收平台下发指令
        printf_debug("[SM] Waiting 20s for downlink...\r\n");
        trea_delay_ms(20000);
        {
            uint8_t dl_count = drain_pending_downlink();
            if(dl_count > 0)
                printf_debug("[SM] Processed %d downlink msg(s)\r\n", dl_count);
        }

        // long模式弱信号冷启动: 检查信号是否恢复, 决定是否保持在线
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
        // long(非弱信号): 检查信号决定去留
        trea_delay_ms(500);
        if(fourG_signal_check_and_decide())
            EC801E_SetPSM(1);
        // else: 信号弱, 内部已断电
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

    // 重新启用模块休眠 (检查信号决定是否保持在线)
    trea_delay_ms(500);
    if(fourG_signal_check_and_decide())
        EC801E_SetPSM(1);
    // else: 信号弱, 内部已断电

    app_state = STATE_SLEEP;
}

// ==================== BLE OTA 状态处�?====================
static void do_ble_ota(void)
{
    /* 第一阶段: 等待手机连接BLE (PD2=LOW) */
    if(g_ble_ota.state == BLE_OTA_WAIT_CONNECT)
    {
        if(ble_link_is_connected())
        {
            /* 手机已连接BLE模块 -> 初始化UART开始接收OTA指令 */
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
            /* 还未连接, 仅检查超时 (ble_ota_process内部处理) */
            ble_ota_process();
        }
    }
    else
    {
        /* 第二阶段: 已连接, 正常OTA流程 */
        ble_ota_process();

        if(g_ble_ota.state == BLE_OTA_DOWNLOADING)
        {
            ota_wdt_feed();
        }
    }
	
    /* 按键检测: 3连按关闭蓝牙 (与3连按开启对称) */
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
            gpio_bit_reset(GPIOC, GPIO_PIN_12);
            if(g_lcd_active) slcd_seg_icon_display(SLCD_ICON_LY, RESET);
        }
    }

    /* 退出条件: OTA错误/超时, 或蓝牙被关闭 */
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
    /* 注: OTA_COMPLETE不会到达这里, handle_end()已调用NVIC_SystemReset() */
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
            const char *action = line_buf + 6;  // "open" �?"close"
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
            fourG_weak_signal = 0;  // 手动开机清除弱信号标志
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

        // ---- 不需要硬件上下文的命�? 交给eeprom_config.c处理 ----
        if(config_parse_and_apply(line_buf) == 0)
            printf_debug("[DBG] Unknown: %s\r\n", line_buf);
    }
}

// ==================== 主函�?====================
int main(void)
{
    SensorRecord_t init_record;		//单条传感器记�?

    // ==================== STATE_INIT ====================
    sys_init();
	
	PT304D_Init(PT304D_SENSOR_1);
    PT304D_Init(PT304D_SENSOR_2);
    // 软件看门�? SysTick 已在 sys_init 内启�? 启用以保护后�?4G 联网/上报等阻塞流�?
    swdt_enable(SWDT_TIMEOUT_MS);
    com_lpuart_init();
    lpuart_wakeup_init();
	printf_debug("\r\n[OTA] yes%s \r\n", APP_FIRMWARE_VERSION);
	
	// 4G开�?+ 注网 + MQTT连接 + keepalive + 时间同步 + QSCLK
    EC801E_PowerOn();
	
    config_load_all();
    config_enter_setup_mode(0);
    config_print_current();

    rtc_init_all();
    rtc_alarm_wakeup_init();
	
	
	
    ee_hist_init();

    // SLCD初始�?(必须在rtc_init_all之后, LXTAL已配�?
    if(g_lcd_enabled) { slcd_seg_configuration(); lcd_ui_init(); g_lcd_active = 1; g_lcd_on_hhmmss = rtc_get_hhmmss(); }

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

    // 4G开机前检查debug串口是否有待处理命令
    config_check_cmd();

    // 4G开�?+ 注网 + MQTT连接 + keepalive + 时间同步 + QSCLK
    uint8_t init_4g_ok = (EC801E_InitAndConnect() == EC_OK);

    if(init_4g_ok)
    {
//        // InitAndConnect末尾启用了QSCLK休眠, 这里先禁用以确保后续AT查询成功
//        trea_delay_ms(300);

        // 4G连接完成, 检查debug串口命令
        config_check_cmd();

        // 用户端MQTT连接 (id=1, 与本地服务器同阶段建立)
        if(EC801E_UserMqtt_IsConfigured())
            EC801E_UserMqtt_Connect();
        report_4g_power_state(1);

        // 4G连接后更新LCD信号强度图标
        lcd_update_csq();

        // 联网成功蜂鸣提示
        buzzer_beep(2, 200);
    }
    else
    {
        printf_debug("[SM] INIT: 4G connect failed, skip MQTT reports\r\n");
        fourG_init_failed = 1;
    }

    // OTA前检查debug串口命令
    config_check_cmd();

    // === OTA 启动回执: �?InfoBlock 显示上次升级 SUCCESS/FAILED, 上报 mqttUpgradeReply ===
    ota_app_report_boot_status();

    // 首次采集+存入缓冲+上报
	trea_delay_ms(50);
    PT304D_CollectAll((g_sensor_count >= 2) ? &g_sensor.raw2 : NULL, &g_sensor.raw);
	// 首次采集后刷新LCD显示
    sensor_sample_voltage(1, g_sensor.last_normal_voltage);
    if(g_lcd_active) lcd_ui_refresh(&g_sensor.raw, &g_sensor.raw2, g_sensor.voltage);
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

    

    // 根据首次采集结果修正 prev_online (无论4G是否成功, 防止实时唤醒误判状态变化)
    g_sensor.prev_online  = (g_sensor.raw.err  == PT304D_OK) ? 1 : 0;
    g_sensor.prev_online2 = (g_sensor.raw2.err == PT304D_OK) ? 1 : 0;

    // ==================== 启动批量上报 (仅4G连接成功时) ====================
    if(!init_4g_ok) goto init_skip_report;

    EC801E_PublishSensorData(&g_sensor.raw, g_sensor.voltage);
    trea_delay_ms(500);

    // �?1: restart=1 + valve_last_order
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
    trea_delay_ms(500);

    // 包2: 设备信息 (csq + rsrp + sinr + cereg + soft-version + ICCID + IMEI)
    {
        static char info_json[512];
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
						"{\"id\":\"csq_csq\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"csq_rsrp\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"csq_sinr\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"soft-version\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"ICCID\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"},"
            "{\"id\":\"IMEI\",\"value\":\"%s\",\"ts\":\"%s\",\"remark\":\"\"}]",
            csq, info_ts, csq, info_ts, rsrp_str, info_ts, sinr_str, info_ts, 
            APP_FIRMWARE_VERSION, info_ts, iccid, info_ts, imei, info_ts);
        EC801E_PublishRawJson(info_json);
        printf_debug("[SM] Device info:\r\n CSQ=%s RSRP=%s SINR=%s CEREG=%s\r\n ICCID=%s\r\n IMEI=%s\r\n",
                     csq, rsrp_str, sinr_str, cereg_str, iccid, imei);
    }
    trea_delay_ms(500);

    // �?3: 传感器在线状�?(进气+出气�?字段)
    {
//        publish_sensor_status();   // rtu_have/status × 2 (与状态机各处共用)
		publish_upload_summary();
        g_sensor.prev_online  = (g_sensor.raw.err  == PT304D_OK) ? 1 : 0;
        g_sensor.prev_online2 = (g_sensor.raw2.err == PT304D_OK) ? 1 : 0;
    }
    trea_delay_ms(500);


    // === 用户端MQTT发布 (id=1, 连接已在启动阶段建立) ===
    if(EC801E_UserMqtt_IsConfigured())
    {
        trea_delay_ms(300);
        EC801E_UserMqtt_PublishSensorData(&g_sensor.raw, &g_sensor.raw2, g_sensor.voltage);
        trea_delay_ms(500);
        EC801E_UserMqtt_Disconnect();
    }

init_skip_report:
    if(init_4g_ok)
    {
        // 首次启动也检查信号: 弱信号直接断电, 避免无效心跳
        if(!fourG_signal_check_and_decide())
            printf_debug("[SM] INIT: weak signal, 4G powered off\r\n");
        else
            EC801E_SetPSM(1);
    }
    else
        EC801E_PowerOff();
    wakeup_count = 0;

    // INIT结束: 如果 spupower 关闭或电压已恢复, 立即退出残留的 VLP 状态
    if(voltage_low_power && (!g_spupower.enabled || g_sensor.voltage >= g_spupower.protect + 0.15f))
    {
        voltage_low_power = 0;
        config_save_ext();
        printf_debug("[SM] VLP cleared on boot (spup_en=%d V=%.2f)\r\n", g_spupower.enabled, g_sensor.voltage);
    }

    // 初始化完成，进入休眠
    app_state = STATE_SLEEP;

    // ==================== 状态机主循�?====================
    while(1)
    {
        // ISR置位后立即响应debug串口配置命令
        if(debug_rx_line_ready())
            debug_check_cmd();

        // 软件看门�? 每次状态分发前重新使能并复位计�?(活跃处理阶段保护)
        // 深休眠由 enter_deepsleep_mode() 内部 swdt_disable() 关闭, 休眠期不计时
        swdt_enable(SWDT_TIMEOUT_MS);

        vtiming_check_now();   /* timing valve highest priority */
        switch(app_state)
        {
            case STATE_SLEEP:    do_sleep();    break;
            case STATE_WAKEUP:   do_wakeup();   break;
            case STATE_COLLECT:  do_collect();  break;
            case STATE_UPLOAD:   do_upload();   break;
            case STATE_DOWNLINK: do_downlink(); break;
            case STATE_ALARM:    do_alarm();    break;
            case STATE_BLE_OTA:  do_ble_ota();  break;
            default:             app_state = STATE_SLEEP; break;
        }
    }
}
