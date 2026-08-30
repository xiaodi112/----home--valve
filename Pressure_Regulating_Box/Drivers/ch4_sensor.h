#ifndef CH4_SENSOR_H
#define CH4_SENSOR_H

#include <stdint.h>

/* ==================== CH4 Sensor Driver (independent, portable) ====================
 * 传感器型号选择 (编译宏):
 *   CH4_SENSOR_MODEL_A = 1: 型号A 帧 "A+000.00 +27.4 00 60\r\n" (22B)
 *       帧头'A'(1) 浓度(7) 空格 温度(5) 空格 故障码(2) 空格 异或(2) \r\n, 无压力
 *   CH4_SENSOR_MODEL_A = 0: 原型号 帧 "+000.00 +21.4 1001.01 00 28\r\n" (29B)
 *       5字段: 浓度/温度/压力/状态码/异或, 含压力
 * 型号A 校验规则(已用真实数据验证): XOR over bytes[0..17] (含帧头, 即校验码前所有字符)
 */
#define CH4_SENSOR_MODEL_A   1

/* ==================== 帧格式参考 (原型号) ====================
 * Frame format: "+000.00 +21.4 1001.01 00 28\r\n" (29 bytes)
 *   field1: concentration (%VOL)
 *   field2: temperature (degC)
 *   field3: pressure (mbar)
 *   field4: status code (hex, 2 digits)
 *   field5: XOR checksum (hex, 2 digits) over bytes[0..24]
 */

typedef struct {
    float   concentration;   /* %VOL  -> methane */
    float   temperature;    /* degC  -> mtemp   */
    float   pressure;        /* mbar  -> internal */
    uint8_t status;         /* hex   -> mstate  */
    uint8_t data_valid;     /* 1 = valid frame  */
} CH4_Data_t;

typedef enum {
    CH4_OK = 0,
    CH4_ERR_TIMEOUT,
    CH4_ERR_CHECKSUM,
    CH4_ERR_FORMAT,
    CH4_ERR_NO_DATA,
    CH4_ERR_NULLPTR
} CH4_Status_t;

/* power + uart */
void ch4_power_on(void);    /* PC13=HIGH + UART1 init */
void ch4_power_off(void);   /* UART1 deinit + PC13=LOW */

/* warmup management */
void    ch4_set_warmup_time(uint32_t ms);
uint8_t ch4_is_warmup_done(void);

/* data collection (blocking, with 2s timeout) */
CH4_Status_t ch4_collect(CH4_Data_t *data);

#endif /* CH4_SENSOR_H */
