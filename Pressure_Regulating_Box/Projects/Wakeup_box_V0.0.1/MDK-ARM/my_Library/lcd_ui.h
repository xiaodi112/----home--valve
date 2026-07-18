#ifndef LCD_UI_H
#define LCD_UI_H

#include <stdint.h>
#include "PT_304_35.h"

// ==================== LCD 页面定义 ====================
typedef enum {
    LCD_PAGE_INLET_PRESSURE = 0,   // 进口压力 (KPa)
    LCD_PAGE_INLET_TEMP,           // 进口温度 (℃)
    LCD_PAGE_OUTLET_PRESSURE,      // 出口压力 (KPa)
    LCD_PAGE_OUTLET_TEMP,          // 出口温度 (℃)
    LCD_PAGE_BATTERY_VOLTAGE,      // 电池电压 (V)
    LCD_PAGE_TIME_YEAR,            // 时间-年 (2026)
    LCD_PAGE_TIME_MONTHDAY,        // 时间-月日 (06-24)
    LCD_PAGE_TIME_HOURMIN,         // 时间-时分 (16-17)
    LCD_PAGE_COUNT                  // 页面总数
} LCD_Page_t;

// ==================== 外部状态变量 ====================
extern uint8_t g_lcd_page;         // 当前LCD页面索引
extern uint8_t g_bt_on;            // 蓝牙状态 (0=关, 1=开)
extern uint8_t g_last_csq;         // 最近一次CSQ信号值 (0~31, 99=未知)

// ==================== LCD UI 接口 ====================

// 初始化LCD显示 (启动时调用, 在slcd_seg_configuration之后)
void lcd_ui_init(void);

// 刷新当前页面数据 (采集后调用)
void lcd_ui_refresh(PT304D_Data *inlet, PT304D_Data *outlet, float voltage);

// 切换到下一页
void lcd_ui_next_page(void);

// 更新信号强度图标 (4G在线查询CSQ后调用)
void lcd_ui_update_signal(uint8_t csq);

// 更新阀门图标 (阀门动作后调用)
void lcd_ui_update_valve(uint8_t valve_on);

// 更新蓝牙图标和电源
void lcd_ui_toggle_bluetooth(void);

// 更新电池图标 (根据电压)
void lcd_ui_update_battery(float voltage);

#endif /* LCD_UI_H */
