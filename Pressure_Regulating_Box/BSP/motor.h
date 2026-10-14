#ifndef __MOTOR_H
#define __MOTOR_H

#include "gd32l23x.h"
#include "app_config.h"
#include "adc.h"        /* ADC_SVC_VREF_MV */

/* ==================== 板级配置 (电机) ==================== */
/* 电机使能 M_SLEEP (PA5): 驱动前拉高, 停止/休眠拉低 */
#define MOTOR_SLEEP_PORT           GPIOA
#define MOTOR_SLEEP_PIN            GPIO_PIN_5
/* 驱动方式: 0=GPIO电平驱动(PA6/PA7); 1=PWM驱动(TIMER2 CH0/CH1, 160kHz) */
#define MOTOR_DRV_MODE             1

/* ==================== 阀门"到位"检测方式 (板级编译开关) ====================
 * VALVE_POS_SENSE_LIMIT = 0: 双限位开关 (旧板: PA11=开阀到位 / PA12=关阀到位, 低电平到位)
 * VALVE_POS_SENSE_STALL = 1: 堵转电流   (本板: PB0 = ADC_IN8 模拟电流, 无机械限位开关)
 *
 * 本板 PA11/PA12 已改作 LED2(阀关)/LED1(检修), 因此只能用堵转方式;
 * 旧的限位逻辑保留编译开关以备旧板复用, 但两种方式互斥。
 */
#define VALVE_POS_SENSE_LIMIT      0
#define VALVE_POS_SENSE_STALL      1
#define VALVE_POS_SENSE            VALVE_POS_SENSE_STALL

/* ---------- 堵转电流采样 (仅 VALVE_POS_SENSE_STALL 有效) ---------- */
#define MOTOR_CS_PORT              GPIOB
#define MOTOR_CS_PIN               GPIO_PIN_0
#define MOTOR_CS_ADC_CH            ADC_CHANNEL_8   /* PB0 = ADC_IN8 */
#define MOTOR_CS_R_SHUNT           1.0f            /* 采样电阻(Ω): 1Ω → 1V/A (须与下面 mΩ 宏一致) */
#define MOTOR_CS_R_MILLIOHM        1000u           /* 采样电阻(毫欧): 1000 = 1.0Ω */

/* 电流(mA) <-> ADC 原始值 换算 (12 位 ADC, 参考 ADC_SVC_VREF_MV) */
#define MOTOR_CS_MA_TO_RAW(ma)     ((uint16_t)(((uint32_t)(ma) * 4095u * MOTOR_CS_R_MILLIOHM) /      \
                                               ((uint32_t)ADC_SVC_VREF_MV * 1000u)))
#define MOTOR_CS_RAW_TO_MA(raw)    ((uint32_t)(((uint32_t)(raw) * (uint32_t)ADC_SVC_VREF_MV * 1000u) / \
                                               (4095u * MOTOR_CS_R_MILLIOHM)))

/* ---------- 堵转阈值 (编译期出厂默认 + EEPROM 运行时可覆盖) ----------
 * MOTOR_CS_TH_MA_DEFAULT: 出厂默认阈值(mA)。EEPROM 该字段从未写过(0xFFFF)时生效;
 *                         一旦用 motor_cs_th:<raw> 写过, 以 EEPROM 为准(写 0 = 关闭判定)。
 * 换算参考: 1Ω + 3.0V 基准 → raw ≈ mA × 1.365, 即 50mA ≈ raw 68。
 */
#define MOTOR_CS_TH_MA_DEFAULT     50u
#define MOTOR_CS_TH_DEFAULT        MOTOR_CS_MA_TO_RAW(MOTOR_CS_TH_MA_DEFAULT)

/* 堵转判定时间参数 (阈值由 EEPROM 配置, 现场标定后生效) */
#define MOTOR_CS_BLANK_MS          300u    /* 启动屏蔽期: 避开启动浪涌误判 */
#define MOTOR_CS_SAMPLE_MS         10u     /* 电流采样周期 */
#define MOTOR_CS_CONFIRM_N         5u      /* 连续超阈次数确认到位(抗 PWM 纹波) */
#define MOTOR_CS_CAL_MAX_MS        4000u   /* 标定驱动的最长持续时间 */
/* 阈值未标定时的驱动时限: 避免无判据时电机长时间堵转发热 */
#define MOTOR_CS_UNCAL_MAX_MS      5000u

#if MOTOR_ENABLED
/* 到位检测硬件初始化 (按 VALVE_POS_SENSE 分派; 深休眠后引脚为 Analog, 动作/读取前必须调用) */
void motor_pos_init(void);

#if VALVE_POS_SENSE == VALVE_POS_SENSE_LIMIT
void motor_limit_init(void);          /* PA11/PA12: 输入+上拉 (低电平=到位) */
uint8_t motor_limit1_reached(void);   /* 开阀到位 PA11 */
uint8_t motor_limit2_reached(void);   /* 关阀到位 PA12 */
#else
void     motor_current_init(void);    /* PB0 配置为模拟输入 */
uint16_t motor_current_raw(void);     /* ADC 原始值; ADC_SVC_RAW_INVALID(0xFFFF)=读失败 */
uint32_t motor_current_ma(void);      /* 换算电流(mA); 0xFFFFFFFF=读失败 */
#endif

/* ==================== 电机初始化 ==================== */
/* enabled: 1=产品带电机, 初始化限位输入+驱动输出; 0=无电机, 跳过硬件配置 */
void motor_init(uint8_t enabled);

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
static inline void motor_pos_init(void)             { }
static inline void motor_current_init(void)         { }
static inline uint16_t motor_current_raw(void)      { return 0xFFFFu; }
static inline uint32_t motor_current_ma(void)       { return 0xFFFFFFFFu; }
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
