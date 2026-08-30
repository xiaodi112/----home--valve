#ifndef VALVE_H
#define VALVE_H

#include <stdint.h>
#include "gd32l23x.h"

// ==================== 阀门动作执行 ====================
// action: "open"/"1" 开阀, "close"/"0" 关阀
// update_last_order: 1=更新g_valve_last_order并持久化, 0=不更新
// 返回: "valve_on"/"valve_off"/"valve_timeout"/"valve_err"
const char* valve_execute(const char *action, uint8_t update_last_order);

// ==================== 阀门实时状态查询 ====================
// 不执行动作, 仅通过限位开关读取当前状态
// 返回: "valve_on"/"valve_off"/"valve_timeout"/"valve_err"
const char* valve_get_status(void);

// ==================== 定时开关阀 当日执行标志 ====================
extern uint8_t timing_open_done_today;
extern uint8_t timing_close_done_today;

#endif /* VALVE_H */
