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
 * - Enable both: each sensor selects its protocol via PT304D_S1_DRIVER / S2_DRIVER
 * (per-sensor protocol is compile-time only)
 *****************************************************************************/
//#define PT304D_DRV_7BYTE         // 7-byte protocol: send 0xAA cmd + read 7 bytes (addr=0x00, 24bit)
#define PT304D_DRV_4BYTE       // 4-byte protocol: direct read 4 bytes       (addr=0x28, 14bit)

/*****************************************************************************
 * Per-sensor driver binding (effective only when both drivers are enabled)
 * value: 7 = 7-byte protocol, 4 = 4-byte protocol
 *****************************************************************************/
#define PT304D_S1_DRIVER   4     // S1 (outlet, IIC2/PC0/PC1, 双传感器旧板)
#define PT304D_S2_DRIVER   4     // S2 (inlet,  IIC1/PD4/PD5)

/*****************************************************************************
 * Per-sensor full scale (unit: Pa)
 * (per-sensor full scale is compile-time only)
 *****************************************************************************/
#define PT304D_S1_FS       40000.0f    // S1 full scale (Pa): 40kPa
#define PT304D_S2_FS       600000.0f   // S2 full scale (Pa): 600kPa
//#define PT304D_S2_FS       35000.0f    // S2 full scale (Pa): 35kPa (single-sensor variant)

/************************** Dual sensor index **************************/
#define PT304D_SENSOR_1    0   // low-pressure/outlet sensor: IIC2 (PC0/PC1) — 仅双传感器旧板, 新板不可用
#define PT304D_SENSOR_2    1   // mid-pressure/inlet  sensor: IIC1 (PD4/PD5)

/************************** Sensor power (always-powered variant, see soft_i2c.h) **************************/
/* I2C 常供电: sensor_pwr_on/off 为空实现; PC3 已改作电压采集基准, 归 adc_bat 管理 */

/************************** 7-byte protocol (0xAA command mode) **************************/
#ifdef PT304D_DRV_7BYTE
#define PT304D_7B_ADDR         0x00    // 7bit I2C address
#define PT304D_7B_MEAS_CMD     0xAA    // measurement command
#define PT304D_7B_READ_BYTES   7       // response bytes (1 status + 3 pressure + 3 temp)
#define PT304D_7B_MEAS_DELAY   15      // measurement delay (ms, datasheet min ~10ms)
#define PT304D_7B_ADDR_REG     0x02    // address-change register
#define PT304D_7B_24BIT_MAX    0xFFFFFF// 24-bit raw full scale
#define PT304D_7B_TEMP_MIN     -40.0f  // temperature range min (C)
#define PT304D_7B_TEMP_MAX     125.0f  // temperature range max (C)
#endif

/************************** 4-byte protocol (direct read) **************************/
#ifdef PT304D_DRV_4BYTE
#define PT304D_4B_ADDR         0x28    // 7bit I2C address
#define PT304D_4B_READ_BYTES   4       // response bytes (status + pressure 14bit + temp 11bit)
#define PT304D_4B_ZERO_ADC     1638    // pressure zero ADC (10%)
#define PT304D_4B_SPAN_ADC     13110   // pressure effective span (90%-10% = 14746-1638)
#define PT304D_4B_TEMP_SCALE   2047    // temperature 11bit scale
#endif

/************************** Legacy aliases (kept for compatibility) **************************/
#define PT304D_I2C_ADDR_7BIT   0x00
#define PT304D_MEAS_CMD        0xAA
#define PT304D_READ_BYTES      7
#define PT304D_MEAS_DELAY_MS   15
#define PT304D_ADDR_REG        0x02
#define PT304D_PRESS_FS_35     35000.0f
#define PT304D_PRESS_FS_600    600000.0f
#define PT304D_24BIT_MAX       0xFFFFFF
#define PT304D_TEMP_MIN        -40.0f
#define PT304D_TEMP_MAX        125.0f

/************************** Legacy protocol selector (runtime dispatch in PT_304_35.c) **************************/
#if defined(PT304D_DRV_7BYTE) && defined(PT304D_DRV_4BYTE)
  #define PT304D_IS_4BYTE(idx) \
      (((idx) == PT304D_SENSOR_1) ? (PT304D_S1_DRIVER == 4) : (PT304D_S2_DRIVER == 4))
#elif defined(PT304D_DRV_4BYTE)
  #define PT304D_IS_4BYTE(idx)  1
#else
  #define PT304D_IS_4BYTE(idx)  0
#endif

/************************** Status bit definitions (7-byte protocol, also matches 4-byte) Bit7~Bit0 **************************/
// Datasheet status bits: Bit7(reserved=0), Bit6(Powered?), Bit5(Busy?), Bit4~Bit3(Mode), Bit2(MemErr), Bit1(ConnErr), Bit0(MathSat)
#define PT304D_STAT_BIT7       (1 << 7)  // reserved, expected 0
#define PT304D_STAT_POWERED    (1 << 6)  // Bit6 Powered?: 1=powered, 0=abnormal
#define PT304D_STAT_BUSY       (1 << 5)  // Bit5 Busy?: 1=busy/not ready, 0=ready
#define PT304D_STAT_MODE_BIT4  (1 << 4)  // Bit4 mode bit high
#define PT304D_STAT_MODE_BIT3  (1 << 3)  // Bit3 mode bit low
#define PT304D_STAT_MEM_ERR    (1 << 2)  // Bit2 Memory Error?
#define PT304D_STAT_CONN_ERR   (1 << 1)  // Bit1 Connection Check Fault?
#define PT304D_STAT_MATH_SAT   (1 << 0)  // Bit0 Math Saturation?
// Mode bits combination: low-power mode requires 0b10 (Bit4=1, Bit3=0)
#define PT304D_STAT_MODE_MASK  (PT304D_STAT_MODE_BIT4 | PT304D_STAT_MODE_BIT3)
#define PT304D_MODE_LOW_POWER  0b10     // low-power mode standard value (required by sensor)

/************************** Sensor status codes (standard response: 0=valid) **************************/
typedef enum {
    PT304D_OK          = 0,  // valid data
    PT304D_ERR_I2C     = 1,  // I2C communication failure (no ACK / bus fault)
    PT304D_ERR_BUSY    = 2,  // Bit5=1: sensor busy, data not ready
    PT304D_ERR_MODE    = 3,  // Bit4~3 != 0b10: wrong power mode
    PT304D_ERR_POWER   = 4,  // Bit6=0: sensor not powered
    PT304D_ERR_NULLPTR = 5,  // null pointer parameter
    PT304D_ERR_DATA    = 6   // data out of expected range
} PT304D_Status;

/************************** Sensor data struct (raw + converted values) **************************/
typedef struct {
    uint8_t  stat_raw;    // status byte raw value (first byte, Bit7~Bit0)
    uint32_t press_raw;   // pressure raw: 7byte=24bit, 4byte=14bit
    uint32_t temp_raw;    // temperature raw: 7byte=24bit, 4byte=11bit
    float    pressure;    // converted pressure (Pa)
    float    temperature; // converted temperature (C)
    PT304D_Status err;    // sensor status code for fault location
} PT304D_Data;

/************************** Public interfaces (used by upper layers) **************************/
// sensor_idx: PT304D_SENSOR_1 = low-pressure (IIC2/PC0/PC1), PT304D_SENSOR_2 = mid-pressure (IIC1/PD4/PD5)
PT304D_Status PT304D_Init(uint8_t sensor_idx);
PT304D_Status PT304D_SendMeasCmd(uint8_t sensor_idx);
PT304D_Status PT304D_ReadMeasData(uint8_t sensor_idx, PT304D_Data *p_data);
PT304D_Status PT304D_Collect(uint8_t sensor_idx, PT304D_Data *p_data);   // includes PC3 power on/off
PT304D_Status PT304D_SetI2CAddr(uint8_t sensor_idx, uint8_t new_7bit_addr);

// Collect both sensors in one power cycle (PC3 power on/off once, saves current)
// p_s1: PT304D_SENSOR_1 result pointer (low-pressure/IIC2)
// p_s2: PT304D_SENSOR_2 result pointer (mid-pressure/IIC1)
// Passing NULL skips the corresponding sensor
void PT304D_CollectAll(PT304D_Data *p_s1, PT304D_Data *p_s2);

#endif
