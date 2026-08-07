#ifndef VALVE_H
#define VALVE_H

#include <stdint.h>
#include "gd32l23x.h"

// ==================== 电机 GPIO 初始化 ====================
// PA5: M_SLP 电机驱动使能输出(默认LOW关闭)
// PA11: K1-S 开阀到位输入(内部上拉, 低电平=到位)
// PA12: K2-S 关阀到位输入(内部上拉, 低电平=到位, 兼用作按键唤醒)
// ==================== 电机功能开关 ====================
// 0=关闭(默认, 产品不带电机), 1=开启
extern uint8_t g_motor_enabled;

void motor_init(void);

// ==================== 限位开关读取 ====================
// 返回 1=到位(低电平), 0=未到位
uint8_t motor_limit1_reached(void);
uint8_t motor_limit2_reached(void);

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
