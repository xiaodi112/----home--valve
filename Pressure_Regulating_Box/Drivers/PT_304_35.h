#ifndef PT_304_35__H
#define PT_304_35__H

#include "gd32l23x.h"
#include "app_config.h"
#include "soft_i2c.h"
#include "delay.h"
#include "stddef.h"

/*****************************************************************************
 * Driver protocol selection (compile-time switch):
 * - Enable one: both sensors use the same protocol
 * - Enable both 4BYTE+7BYTE: each sensor selects via PT304D_S1_DRIVER / S2_DRIVER
 * - PT304D_DRV_SENSOR: new register-based calibrated sensor (addr 0x76/0x77)
 *****************************************************************************/
//#define PT304D_DRV_7BYTE         // 7-byte protocol: send 0xAA cmd + read 7 bytes
//#define PT304D_DRV_4BYTE         // 4-byte protocol: direct read 4 bytes (addr=0x28)
#define PT304D_DRV_SENSOR          // New sensor: register map + calib coefficients (0x76/0x77)

/*****************************************************************************
 * Per-sensor driver binding (effective only when both 4BYTE and 7BYTE are enabled)
 * value: 7 = 7-byte protocol, 4 = 4-byte protocol
 *****************************************************************************/
#define PT304D_S1_DRIVER   4
#define PT304D_S2_DRIVER   4

/*****************************************************************************
 * Per-sensor full scale (unit: Pa)  —— only used by 4BYTE/7BYTE
 *****************************************************************************/
#define PT304D_S1_FS       40000.0f    // S1 full scale (Pa): 40kPa
#define PT304D_S2_FS       600000.0f   // S2 full scale (Pa): 600kPa

/************************** Dual sensor index **************************/
#define PT304D_SENSOR_1    0   // 阀后/出气 (IIC2/PC0/PC1)  → addr 0x77
#define PT304D_SENSOR_2    1   // 阀前/进气 (IIC1/PD4/PD5)  → addr 0x76

/************************** New sensor addresses **************************/
#ifdef PT304D_DRV_SENSOR
#define PT304D_S1_I2C_ADDR     0x77    // 阀后出气端
//#define PT304D_S2_I2C_ADDR     0x76    // 阀前进气端
#define PT304D_S2_I2C_ADDR     0x77    // 阀前进气端
#endif

/************************** 7-byte protocol **************************/
#ifdef PT304D_DRV_7BYTE
#define PT304D_7B_ADDR         0x00
#define PT304D_7B_MEAS_CMD     0xAA
#define PT304D_7B_READ_BYTES   7
#define PT304D_7B_MEAS_DELAY   15
#define PT304D_7B_ADDR_REG     0x02
#define PT304D_7B_24BIT_MAX    0xFFFFFF
#define PT304D_7B_TEMP_MIN     -40.0f
#define PT304D_7B_TEMP_MAX     125.0f
#endif

/************************** 4-byte protocol **************************/
#ifdef PT304D_DRV_4BYTE
#define PT304D_4B_ADDR         0x28
#define PT304D_4B_READ_BYTES   4
#define PT304D_4B_ZERO_ADC     1638
#define PT304D_4B_SPAN_ADC     13110
#define PT304D_4B_TEMP_SCALE   2047
#endif

/************************** Legacy aliases **************************/
#define PT304D_I2C_ADDR_7BIT   0x00
#define PT304D_MEAS_CMD        0xAA
#define PT304D_READ_BYTES      7
#define PT304D_MEAS_DELAY_MS   15
#define PT304D_ADDR_REG        0x02
#define PT304D_PRESS_FS_35     40000.0f
#define PT304D_PRESS_FS_600    600000.0f
#define PT304D_24BIT_MAX       0xFFFFFF
#define PT304D_TEMP_MIN        -40.0f
#define PT304D_TEMP_MAX        125.0f

/************************** Legacy protocol selector **************************/
#if defined(PT304D_DRV_7BYTE) && defined(PT304D_DRV_4BYTE)
  #define PT304D_IS_4BYTE(idx) \
      (((idx) == PT304D_SENSOR_1) ? (PT304D_S1_DRIVER == 4) : (PT304D_S2_DRIVER == 4))
#elif defined(PT304D_DRV_4BYTE)
  #define PT304D_IS_4BYTE(idx)  1
#else
  #define PT304D_IS_4BYTE(idx)  0
#endif

/************************** Status bit definitions (7-byte) **************************/
#define PT304D_STAT_BIT7       (1 << 7)
#define PT304D_STAT_POWERED    (1 << 6)
#define PT304D_STAT_BUSY       (1 << 5)
#define PT304D_STAT_MODE_BIT4  (1 << 4)
#define PT304D_STAT_MODE_BIT3  (1 << 3)
#define PT304D_STAT_MEM_ERR    (1 << 2)
#define PT304D_STAT_CONN_ERR   (1 << 1)
#define PT304D_STAT_MATH_SAT   (1 << 0)
#define PT304D_STAT_MODE_MASK  (PT304D_STAT_MODE_BIT4 | PT304D_STAT_MODE_BIT3)
#define PT304D_MODE_LOW_POWER  0b10

/************************** Sensor status codes **************************/
typedef enum {
    PT304D_OK          = 0,
    PT304D_ERR_I2C     = 1,
    PT304D_ERR_BUSY    = 2,
    PT304D_ERR_MODE    = 3,
    PT304D_ERR_POWER   = 4,
    PT304D_ERR_NULLPTR = 5,
    PT304D_ERR_DATA    = 6
} PT304D_Status;

/************************** Sensor data struct **************************/
typedef struct {
    uint8_t  stat_raw;
    uint32_t press_raw;
    uint32_t temp_raw;
    float    pressure;      // Pa
    float    temperature;   // °C
    PT304D_Status err;
} PT304D_Data;

/************************** Public interfaces **************************/
PT304D_Status PT304D_Init(uint8_t sensor_idx);
PT304D_Status PT304D_SendMeasCmd(uint8_t sensor_idx);
PT304D_Status PT304D_ReadMeasData(uint8_t sensor_idx, PT304D_Data *p_data);
PT304D_Status PT304D_Collect(uint8_t sensor_idx, PT304D_Data *p_data);
PT304D_Status PT304D_SetI2CAddr(uint8_t sensor_idx, uint8_t new_7bit_addr);

void  PT304D_SetZeroOffset(uint8_t sensor_idx, float offset_pa);
float PT304D_GetZeroOffset(uint8_t sensor_idx);

void PT304D_CollectAll(PT304D_Data *p_s1, PT304D_Data *p_s2);

/* ==================== 静态测漏: 保持供电采集 ====================
 * 常规采集每次上/断电; 静态测漏任务期间要求传感器不断电(读数稳定/避免50ms上电等待),
 * 用法: 任务开始 PT304D_PowerHold(1) → 循环 PT304D_CollectAllHeld() → 任务结束 PT304D_PowerHold(0)
 * PT304D_PowerHold(1) 幂等: 供电已被其它路径断开时会重新上电并等待稳定。 */
void PT304D_PowerHold(uint8_t on);
void PT304D_CollectAllHeld(PT304D_Data *p_s1, PT304D_Data *p_s2);

#endif
