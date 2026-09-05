#ifndef __MOTOR_H
#define __MOTOR_H

#include "gd32l23x.h"
#include "app_config.h"

/* ==================== 板级配置 (电机) ==================== */
/* 电机使能 M_SLEEP (PA5): 驱动前拉高, 停止/休眠拉低 */
#define MOTOR_SLEEP_PORT           GPIOA
#define MOTOR_SLEEP_PIN            GPIO_PIN_5
/* 驱动方式: 0=GPIO电平驱动(PA6/PA7); 1=PWM驱动(TIMER2 CH0/CH1, 160kHz) */
#define MOTOR_DRV_MODE             1

#if MOTOR_ENABLED
/* 限位引脚初始化: PA11/PA12 恢复为输入+上拉 (深休眠 gpio_enter_low_power 会转 Analog,
 * 动作或读取限位前必须先调用) */
void motor_limit_init(void);

/* ==================== 电机初始化 ==================== */
/* enabled: 1=产品带电机, 初始化限位输入+驱动输出; 0=无电机, 跳过硬件配置 */
void motor_init(uint8_t enabled);

/* ==================== 限位开关读取 ==================== */
/* 返回 1=到位(低电平), 0=未到位 */
uint8_t motor_limit1_reached(void);   /* 开阀到位 PA11 */
uint8_t motor_limit2_reached(void);   /* 关阀到位 PA12 */

/* 电机使能 M_SLEEP (PA5):
 *   motor_sleep_enable()  : 驱动前调用, PA5 拉高
 *   motor_sleep_disable() : 停止/休眠调用, PA5 拉低 */
void motor_sleep_enable(void);
void motor_sleep_disable(void);

/* ==================== 方向驱动(语义接口, 内部封装 PWM/GPIO 差异) ==================== */
void motor_drive_open(void);     /* 使能+开阀方向 */
void motor_drive_close(void);    /* 使能+关阀方向 */
void motor_stop(void);           /* 停止驱动并拉低使能 */
#else
/* 无电机硬件: 提供空实现, 调用点无需 #if 保护 */
static inline void motor_limit_init(void)           { }
static inline void motor_init(uint8_t enabled)      { (void)enabled; }
static inline uint8_t motor_limit1_reached(void)    { return 0; }
static inline uint8_t motor_limit2_reached(void)    { return 0; }
static inline void motor_sleep_enable(void)         { }
static inline void motor_sleep_disable(void)        { }
static inline void motor_drive_open(void)           { }
static inline void motor_drive_close(void)          { }
static inline void motor_stop(void)                 { }
#endif /* MOTOR_ENABLED */

#endif
