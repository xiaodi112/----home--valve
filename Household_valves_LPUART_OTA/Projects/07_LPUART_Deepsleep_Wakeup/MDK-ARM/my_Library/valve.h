#ifndef VALVE_H
#define VALVE_H

#include <stdint.h>
#include "gd32l23x.h"

// ==================== 电机 GPIO 初始化 ====================
// PC0: 电机使能输出(默认LOW关闭)
// PC1/PC2: 到位信号输入(内部上拉, 低电平=到位)
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

#endif /* VALVE_H */
