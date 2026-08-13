#include "lcd_ui.h"
#include "slcd_seg.h"
#include "mygpio.h"
#include "tim.h"
#include "eeprom_config.h"
#include "gd32l23x.h"
#include "gd32l23x_rtc.h"

// ==================== 全局状态 ====================
uint8_t g_lcd_page = 0;
uint8_t g_bt_on = 0;
uint8_t g_last_csq = 99;

// ==================== 内部缓存 ====================
#if !LCD_SINGLE_SENSOR
static float s_inlet_pressure = 0.0f;
static float s_inlet_temp = 0.0f;
static PT304D_Status s_inlet_err = PT304D_ERR_I2C;
#endif
static float s_outlet_pressure = 0.0f;
static float s_outlet_temp = 0.0f;
static float s_voltage = 0.0f;
static PT304D_Status s_outlet_err = PT304D_ERR_I2C;
static uint32_t s_page_change_sec = 0;

static uint32_t hhmmss_to_total_sec(uint32_t hhmmss)
{
    uint32_t h = hhmmss / 10000;
    uint32_t m = (hhmmss / 100) % 100;
    uint32_t s = hhmmss % 100;
    return h * 3600 + m * 60 + s;
}

// ==================== 内部函数声明 ====================
static void lcd_ui_display_page(void);
static void lcd_ui_update_common_icons(void);

// ==================== 通用浮点显示 (4位自适应, 支持负数, 无闪屏) ====================
// decimals: ival 的隐含小数位数
//   压力: Pa→KPa, scale=1, decimals=3 (1Pa=0.001KPa)
//   温度: °C, scale=100, decimals=2 (乘100后为0.01°C)
//   电压: V, scale=100, decimals=2
static void lcd_ui_display_value(float value, uint16_t scale, uint8_t decimals)
{
    char buf[8];
    uint8_t neg = 0;
    uint32_t ival;

    if(value < 0.0f) {
        neg = 1;
        value = -value;
    }

    ival = (uint32_t)(value * scale + 0.5f);

    // 根据 decimals 计算各格式的阈值边界
    // decimals=3: th[0]=10000, th[1]=100000, th[2]=1000000
    // decimals=2: th[0]=1000,  th[1]=10000,  th[2]=100000
    uint32_t th0, th1, th2;
    if(decimals == 3) { th0 = 10000; th1 = 100000; th2 = 1000000; }
    else              { th0 = 1000;  th1 = 10000;  th2 = 100000;  }

    if(neg) {
        if(ival < th0) {
            // -D.DDD (dec=3) 或 -D.DD (dec=2)
            buf[0] = '-';
            if(decimals == 3) {
                buf[1] = '0' + (ival / 1000);
                buf[2] = '.';
                buf[3] = '0' + ((ival / 100) % 10);
                buf[4] = '0' + ((ival / 10) % 10);
                buf[5] = '0' + (ival % 10);
                buf[6] = '\0';
            } else {
                buf[1] = '0' + (ival / 100);
                buf[2] = '.';
                buf[3] = '0' + ((ival / 10) % 10);
                buf[4] = '0' + (ival % 10);
                buf[5] = '\0';
            }
        } else if(ival < th1) {
            // -DD.DD (dec=3) 或 -DD.D (dec=2)
            uint32_t v = (ival + 5) / 10;
            if(v > 9999) v = 9999;
            buf[0] = '-';
            if(decimals == 3) {
                buf[1] = '0' + (v / 1000);
                buf[2] = '0' + ((v / 100) % 10);
                buf[3] = '.';
                buf[4] = '0' + ((v / 10) % 10);
                buf[5] = '0' + (v % 10);
                buf[6] = '\0';
            } else {
                if(v > 999) v = 999;
                buf[1] = '0' + (v / 100);
                buf[2] = '0' + ((v / 10) % 10);
                buf[3] = '.';
                buf[4] = '0' + (v % 10);
                buf[5] = '\0';
            }
        } else if(ival < th2) {
            // -DDD.D (dec=3) 或 -DDD (dec=2)
            uint32_t v = (ival + 50) / 100;
            if(v > 9999) v = 9999;
            buf[0] = '-';
            if(decimals == 3) {
                buf[1] = '0' + (v / 1000);
                buf[2] = '0' + ((v / 100) % 10);
                buf[3] = '0' + ((v / 10) % 10);
                buf[4] = '.';
                buf[5] = '0' + (v % 10);
                buf[6] = '\0';
            } else {
                if(v > 999) v = 999;
                buf[1] = '0' + (v / 100);
                buf[2] = '0' + ((v / 10) % 10);
                buf[3] = '0' + (v % 10);
                buf[4] = '\0';
            }
        } else {
            // -DDDD
            uint32_t v = (decimals == 3) ? (ival + 500) / 1000 : (ival + 50) / 100;
            if(v > 9999) v = 9999;
            buf[0] = '-';
            buf[1] = '0' + (v / 1000);
            buf[2] = '0' + ((v / 100) % 10);
            buf[3] = '0' + ((v / 10) % 10);
            buf[4] = '0' + (v % 10);
            buf[5] = '\0';
        }
    } else {
        if(ival < th0) {
            // D.DDD (dec=3) 或 D.DD (dec=2)
            if(decimals == 3) {
                buf[0] = '0' + (ival / 1000);
                buf[1] = '.';
                buf[2] = '0' + ((ival / 100) % 10);
                buf[3] = '0' + ((ival / 10) % 10);
                buf[4] = '0' + (ival % 10);
                buf[5] = '\0';
            } else {
                buf[0] = '0' + (ival / 100);
                buf[1] = '.';
                buf[2] = '0' + ((ival / 10) % 10);
                buf[3] = '0' + (ival % 10);
                buf[4] = '\0';
            }
        } else if(ival < th1) {
            // DD.DD (dec=3) 或 DD.D (dec=2)
            uint32_t v = (ival + 5) / 10;
            if(decimals == 3) {
                buf[0] = '0' + (v / 1000);
                buf[1] = '0' + ((v / 100) % 10);
                buf[2] = '.';
                buf[3] = '0' + ((v / 10) % 10);
                buf[4] = '0' + (v % 10);
                buf[5] = '\0';
            } else {
                if(v > 999) v = 999;
                buf[0] = '0' + (v / 100);
                buf[1] = '0' + ((v / 10) % 10);
                buf[2] = '.';
                buf[3] = '0' + (v % 10);
                buf[4] = '\0';
            }
        } else if(ival < th2) {
            // DDD.D (dec=3) 或 DDD (dec=2)
            uint32_t v = (ival + 50) / 100;
            if(decimals == 3) {
                buf[0] = '0' + (v / 1000);
                buf[1] = '0' + ((v / 100) % 10);
                buf[2] = '0' + ((v / 10) % 10);
                buf[3] = '.';
                buf[4] = '0' + (v % 10);
                buf[5] = '\0';
            } else {
                if(v > 999) v = 999;
                buf[0] = '0' + (v / 100);
                buf[1] = '0' + ((v / 10) % 10);
                buf[2] = '0' + (v % 10);
                buf[3] = '\0';
            }
        } else {
            // DDDD
            uint32_t v = (decimals == 3) ? (ival + 500) / 1000 : (ival + 50) / 100;
            if(v > 9999) v = 9999;
            buf[0] = '0' + (v / 1000);
            buf[1] = '0' + ((v / 100) % 10);
            buf[2] = '0' + ((v / 10) % 10);
            buf[3] = '0' + (v % 10);
            buf[4] = '\0';
        }
    }

    slcd_seg_string_atomic_display(buf);
}

// 压力显示: Pa→KPa, scale=1, 3位小数
static inline void lcd_ui_display_pressure_kpa(float pa) { lcd_ui_display_value(pa, 1, 3); }
// 温度显示: °C, scale=100, 2位小数
static inline void lcd_ui_display_temp(float c)          { lcd_ui_display_value(c, 100, 2); }

// ==================== 初始化 ====================
void lcd_ui_init(void)
{
#if LCD_SINGLE_SENSOR
    g_lcd_page = LCD_PAGE_OUTLET_PRESSURE;
#else
    g_lcd_page = LCD_PAGE_INLET_PRESSURE;
#endif
//    g_last_csq = 99;

    slcd_seg_clear_all();
    slcd_seg_icon_clear_all();

    lcd_ui_display_page();

}

// ==================== 刷新当前页面 (采集后调用) ====================
void lcd_ui_refresh(PT304D_Data *inlet, PT304D_Data *outlet, float voltage)
{
    uint8_t need_redraw = 0;

    // 空闲超时20s自动回默认页
#if LCD_SINGLE_SENSOR
    if(g_lcd_page != LCD_PAGE_OUTLET_PRESSURE)
#else
    if(g_lcd_page != LCD_PAGE_INLET_PRESSURE)
#endif
    {
        uint32_t now_sec = hhmmss_to_total_sec(rtc_get_hhmmss());
        int32_t elapsed = (int32_t)(now_sec - s_page_change_sec);
        if(elapsed < 0) elapsed += 86400;
        if(elapsed >= 20)
        {
#if LCD_SINGLE_SENSOR
            g_lcd_page = LCD_PAGE_OUTLET_PRESSURE;
#else
            g_lcd_page = LCD_PAGE_INLET_PRESSURE;
#endif
            lcd_ui_display_page();
            return;
        }
    }

    /* LCD常亮模式(LCD_LOCK_ENABLED=0): 常亮不超时, 页面由采集周期每次刷新后自动进行翻页 */

    // 缓存数据并检测变化
#if LCD_SINGLE_SENSOR
    if(inlet != NULL)
    {
        if(s_outlet_pressure != inlet->pressure ||
           s_outlet_temp    != inlet->temperature ||
           s_outlet_err     != inlet->err)
        {
            s_outlet_pressure = inlet->pressure;
            s_outlet_temp = inlet->temperature;
            s_outlet_err = inlet->err;
            if(g_lcd_page == LCD_PAGE_OUTLET_PRESSURE || g_lcd_page == LCD_PAGE_OUTLET_TEMP)
                need_redraw = 1;
        }
    }
#else
    if(inlet != NULL)
    {
        if(s_inlet_pressure != inlet->pressure ||
           s_inlet_temp    != inlet->temperature ||
           s_inlet_err     != inlet->err)
        {
            s_inlet_pressure = inlet->pressure;
            s_inlet_temp = inlet->temperature;
            s_inlet_err = inlet->err;
            if(g_lcd_page == LCD_PAGE_INLET_PRESSURE || g_lcd_page == LCD_PAGE_INLET_TEMP)
                need_redraw = 1;
        }
    }
    if(outlet != NULL)
    {
        if(s_outlet_pressure != outlet->pressure ||
           s_outlet_temp    != outlet->temperature ||
           s_outlet_err     != outlet->err)
        {
            s_outlet_pressure = outlet->pressure;
            s_outlet_temp = outlet->temperature;
            s_outlet_err = outlet->err;
            if(g_lcd_page == LCD_PAGE_OUTLET_PRESSURE || g_lcd_page == LCD_PAGE_OUTLET_TEMP)
                need_redraw = 1;
        }
    }
#endif
    if(s_voltage != voltage)
    {
        s_voltage = voltage;
        if(g_lcd_page == LCD_PAGE_BATTERY_VOLTAGE)
            need_redraw = 1;
    }

    // 时间页面每次都刷新
    if(g_lcd_page >= LCD_PAGE_TIME_YEAR)
        need_redraw = 1;

    if(need_redraw)
        lcd_ui_display_page();
}

// ==================== 切换下一页 ====================
void lcd_ui_next_page(void)
{
    g_lcd_page = (g_lcd_page + 1) % LCD_PAGE_COUNT;
    s_page_change_sec = hhmmss_to_total_sec(rtc_get_hhmmss());
    lcd_ui_display_page();
}

// ==================== 更新信号强度图标 ====================
void lcd_ui_update_signal(uint8_t csq)
{
    g_last_csq = csq;

    if(csq == 0 || csq == 99)
    {
        slcd_seg_icon_display(SLCD_ICON_S7, RESET);
        slcd_seg_icon_display(SLCD_ICON_S8, RESET);
        slcd_seg_icon_display(SLCD_ICON_S9, RESET);
        slcd_seg_icon_display(SLCD_ICON_S10, RESET);
    }
    else
    {
        slcd_seg_icon_display(SLCD_ICON_S7, SET);
        slcd_seg_icon_display(SLCD_ICON_S8, SET);
        slcd_seg_icon_display(SLCD_ICON_S9, (csq >= 15) ? SET : RESET);
        slcd_seg_icon_display(SLCD_ICON_S10, (csq >= 25) ? SET : RESET);
    }
}

// ==================== 更新阀门图标 ====================
void lcd_ui_update_valve(uint8_t valve_on)
{
    slcd_seg_icon_display(SLCD_ICON_S11, valve_on ? SET : RESET);
}

// ==================== 更新电池图标 ====================
void lcd_ui_update_battery(float voltage)
{
    slcd_seg_icon_display(SLCD_ICON_S1, SET);
    slcd_seg_icon_display(SLCD_ICON_S2, SET);
    slcd_seg_icon_display(SLCD_ICON_S3, (voltage >= 3.3f) ? SET : RESET);
}

// ==================== 蓝牙开关切换 ====================
void lcd_ui_toggle_bluetooth(void)
{
    g_bt_on ^= 1;
    if(g_bt_on) {
        gpio_bit_set(GPIOC, GPIO_PIN_12);
    } else {
        gpio_bit_reset(GPIOC, GPIO_PIN_12);
    }
    slcd_seg_icon_display(SLCD_ICON_LY, g_bt_on ? SET : RESET);
}

// ==================== 时间页辅助: 显示 XX-XX ====================
static void lcd_ui_display_time_pair(uint8_t hi, uint8_t lo)
{
    slcd_seg_clear_digits();
    slcd_seg_digit_display(hi / 10, 1);
    slcd_seg_digit_display(hi % 10, 2);
    slcd_seg_code_display(0x08, 3);  // '-'
    slcd_seg_digit_display(lo / 10, 4);
    slcd_seg_digit_display(lo % 10, 5);
}

// ==================== 内部: 显示当前页面 ====================
static void lcd_ui_display_page(void)
{
    slcd_seg_batch_begin();

    switch(g_lcd_page)
    {
#if !LCD_SINGLE_SENSOR
        case LCD_PAGE_INLET_PRESSURE:
            if(s_inlet_err != PT304D_OK) {
                slcd_seg_string_atomic_display("-EEr-");
				slcd_seg_icon_display(SLCD_ICON_S13,SET);
            } else {
                lcd_ui_display_pressure_kpa(s_inlet_pressure);
            }
            slcd_seg_icon_display(SLCD_ICON_S5, SET);
            if(s_inlet_err == PT304D_OK) slcd_seg_icon_display(SLCD_ICON_S13, SET);
            break;

        case LCD_PAGE_INLET_TEMP:
            if(s_inlet_err != PT304D_OK) {
                slcd_seg_string_atomic_display("-EEr-");
				slcd_seg_icon_display(SLCD_ICON_S12,SET);
            } else {
                lcd_ui_display_temp(s_inlet_temp);
            }
            slcd_seg_icon_display(SLCD_ICON_S5, SET);
            if(s_inlet_err == PT304D_OK) slcd_seg_icon_display(SLCD_ICON_S12, SET);
            break;

#endif
        case LCD_PAGE_OUTLET_PRESSURE:
            if(s_outlet_err != PT304D_OK) {
                slcd_seg_string_atomic_display("-EEr-");
				slcd_seg_icon_display(SLCD_ICON_S13,SET);
            } else {
                lcd_ui_display_pressure_kpa(s_outlet_pressure);
            }
            slcd_seg_icon_display(SLCD_ICON_S6, SET);
            if(s_outlet_err == PT304D_OK) slcd_seg_icon_display(SLCD_ICON_S13, SET);
            break;

        case LCD_PAGE_OUTLET_TEMP:
            if(s_outlet_err != PT304D_OK) {
                slcd_seg_string_atomic_display("-EEr-");
				slcd_seg_icon_display(SLCD_ICON_S12,SET);
            } else {
                lcd_ui_display_temp(s_outlet_temp);
            }
            slcd_seg_icon_display(SLCD_ICON_S6, SET);
            if(s_outlet_err == PT304D_OK) slcd_seg_icon_display(SLCD_ICON_S12, SET);
            break;

        case LCD_PAGE_BATTERY_VOLTAGE:
            lcd_ui_display_value(s_voltage, 100, 2);
            break;

        case LCD_PAGE_TIME_YEAR:
        {
            rtc_parameter_struct tp;
            char ybuf[6];
            rtc_current_time_get(&tp);
            uint8_t year = bcd2dec((uint8_t)(tp.year & 0xFF));
            ybuf[0] = '2';
            ybuf[1] = '0';
            ybuf[2] = '0' + (year / 10);
            ybuf[3] = '0' + (year % 10);
            ybuf[4] = '\0';
            slcd_seg_string_atomic_display(ybuf);
            break;
        }

        case LCD_PAGE_TIME_MONTHDAY:
        {
            rtc_parameter_struct tp;
            rtc_current_time_get(&tp);
            lcd_ui_display_time_pair(bcd2dec((uint8_t)(tp.month & 0xFF)),
                                     bcd2dec((uint8_t)(tp.date & 0xFF)));
            break;
        }

        case LCD_PAGE_TIME_HOURMIN:
        {
            rtc_parameter_struct tp;
            rtc_current_time_get(&tp);
            lcd_ui_display_time_pair(bcd2dec((uint8_t)(tp.hour & 0xFF)),
                                     bcd2dec((uint8_t)(tp.minute & 0xFF)));
            break;
        }

        default:
            break;
    }

    lcd_ui_update_common_icons();

    slcd_seg_batch_end();
}

// ==================== 内部: 恢复公共图标 ====================
static void lcd_ui_update_common_icons(void)
{
	// 电池
    lcd_ui_update_battery(s_voltage);
	// 信号
    lcd_ui_update_signal(g_last_csq);
	// 阀门
    lcd_ui_update_valve(g_valve_last_order);
    // 蓝牙
    slcd_seg_icon_display(SLCD_ICON_LY, g_bt_on ? SET : RESET);
}

