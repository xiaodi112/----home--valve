#ifndef VALVE_H
#define VALVE_H

#include <stdint.h>
#include "gd32l23x.h"

// ==================== 阀门动作执行 ====================
// action: "open"/"1" 开阀, "close"/"0" 关阀
// update_last_order: 1=更新g_valve_last_order并持久化, 0=不更新
// 返回: "valve-on"/"valve-off"/"valve-timeout"/"valve-err"
const char* valve_execute(const char *action, uint8_t update_last_order);

// ==================== 阀门实时状态查询 ====================
// 堵转方案: 位置不可读, 返回持久化状态 (g_valve_pos_valid=0 时返回 valve-timeout 表示未确认)
// 双限位方案: 通过限位开关读取当前状态
// 返回: "valve-on"/"valve-off"/"valve-timeout"/"valve-err"
const char* valve_get_status(void);

// ==================== 回零 (仅堵转方案) ====================
// 朝关阀方向驱动到堵转, 建立已知起点"已关到底", 并置位置可信标志
// 返回: 1=成功(已关到底), 0=失败(阈值未标定/未检测到堵转; 双限位方案恒返回1)
uint8_t valve_home(void);

// ==================== 定时开关阀 当日执行标志 ====================
extern uint8_t timing_open_done_today;
extern uint8_t timing_close_done_today;

#endif /* VALVE_H */
