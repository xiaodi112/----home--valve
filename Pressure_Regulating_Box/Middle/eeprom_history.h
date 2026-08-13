#ifndef EEPROM_HISTORY_H
#define EEPROM_HISTORY_H

#include <stdint.h>
#include "app_config.h"

/**
 * @file eeprom_history.h
 * @brief EEPROM 历史数据环形缓冲存储 (M24M01 / AT24C64 兼容)
 *
 * 地址规划:
 *   配置区: 0x000 ~ 0x7FF (2KB, 含870B预留)
 *   历史区: 0x800 ~ 0x1FFF (6144B)
 *     管理头: 0x800 ~ 0x81F (32B, 1页)
 *     数据区: 0x820 ~ 0x1FFF (6112B = 191条 x 32B)
 *
 * 每条记录 32 字节 = 1页, 写入无需跨页, 效率最优.
 * 环形覆盖: 写满后自动覆盖最旧记录.
 */

// ==================== 地址定义 ====================
/* ==================== EEPROM partition (per chip) ====================
 * AT24C64 (8KB) : config 0x000~0x7FF (2KB) + history 0x800~0x1FFF (6KB/191 rec)
 * M24M01 (128KB): config 0x000~0xFFF (4KB) + static-leak reserved 0x1000~0x5AFF
 *                 (600 rec x 32B) + history 0x5B00~0x1FFFF (~3367 rec)
 */
#define EE_HIST_RECORD_SIZE   32       /* record size = page size */

#if defined(EEPROM_DRV_AT24C64)
#define EE_HIST_BASE          0x0800
#define EE_HIST_HEADER_ADDR   0x0800
#define EE_HIST_DATA_ADDR     0x0820
#define EE_HIST_END_ADDR      0x1FFF
#define EE_HIST_MAX_RECORDS   191
#else   /* M24M01: 128KB full range */
#define EE_HIST_BASE          0x5B00   /* after config(4KB)+sideleak(600x32B) */
#define EE_HIST_HEADER_ADDR   0x5B00
#define EE_HIST_DATA_ADDR     0x5B20
#define EE_HIST_END_ADDR      0x1FFFF
#define EE_HIST_MAX_RECORDS   3367     /* (0x1FFFF-0x5B20+1)/32 */
#endif

/* Static leak reserved region (M24M01 only; feature not implemented yet) */
#if defined(EEPROM_DRV_M24M01)
#define EE_SIDELEAK_BASE      0x1000   /* after 4KB config */
#define EE_SIDELEAK_RECORDS   600
#define EE_SIDELEAK_SIZE      (EE_SIDELEAK_RECORDS * EE_HIST_RECORD_SIZE)
#endif


// ==================== 管理头 Magic ====================
#define EE_HIST_MAGIC         0xDA7A   // "DATA" 谐音

// ==================== 紧凑型历史记录 (32字节, 页对齐) ====================
typedef struct {
    float    pressure;       // 进气端压力 (Pa)          [4B]
    float    temperature;    // 进气端温度 (C)           [4B]
    float    pressure2;      // 出气端压力 (Pa)          [4B]
    float    temperature2;   // 出气端温度 (C)           [4B]
    float    voltage;        // 电池电压 (V)             [4B]
    uint8_t  ts_bcd[6];     // 时间戳 BCD: YY MM DD HH MM SS [6B]
    uint8_t  flags;          // 标志位 (SENSOR_FLAG_xxx) [1B]
    uint8_t  crc8;           // CRC8校验 (前31字节)      [1B]
    uint8_t  reserved[4];    // 预留扩展                 [4B]
} EeHistRecord_t;            // 合计: 32B

// ==================== 接口函数 ====================

/**
 * 初始化历史缓冲区 (上电调用)
 * 读取管理头, 校验magic; 无效则格式化.
 */
void ee_hist_init(void);

/**
 * 写入一条历史记录 (自动获取当前RTC时间作为时间戳)
 * @param pressure   进气端压力 (Pa)
 * @param temperature 进气端温度 (C)
 * @param pressure2  出气端压力 (Pa)
 * @param temperature2 出气端温度 (C)
 * @param voltage    电池电压 (V)
 * @param flags      标志位
 */
void ee_hist_push(float pressure, float temperature,
                  float pressure2, float temperature2,
                  float voltage, uint8_t flags);

/**
 * 读取第 index 条记录 (0=最旧, count-1=最新)
 * @param index  记录索引
 * @param out    输出缓冲
 * @return 0=成功, 1=索引越界, 2=CRC校验失败
 */
uint8_t ee_hist_read(uint16_t index, EeHistRecord_t *out);

/**
 * 获取当前有效记录数
 */
uint16_t ee_hist_count(void);

/**
 * 弹出最旧一条记录 (tail前进, count--, 保存管理头)
 * 用于逐条上报后删除
 */
void ee_hist_pop(void);

/**
 * 清空所有历史记录 (管理头复位, 不擦除数据页)
 */
void ee_hist_clear(void);

/**
 * 将 BCD 时间戳转为字符串 "20YY-MM-DD HH:MM:SS"
 * @param ts_bcd  6字节BCD时间 [YY,MM,DD,HH,MM,SS]
 * @param buf     输出缓冲 (至少20字节)
 */
void ee_hist_ts_to_str(const uint8_t *ts_bcd, char *buf);

/**
 * 将历史记录转为 SensorRecord_t 格式 (兼容现有上报接口)
 * @param hist   EEPROM历史记录
 * @param out    输出 SensorRecord_t (需包含 sensor_buffer.h)
 */
void ee_hist_to_sensor_record(const EeHistRecord_t *hist, void *out);

#endif /* EEPROM_HISTORY_H */
