#ifndef SENSOR_BUFFER_H
#define SENSOR_BUFFER_H

#include <stdint.h>

/**
 * @file sensor_buffer.h
 * @brief RAM环形缓冲区 — 存储传感器采集记录
 *
 * 深休眠时SRAM保持供电,数据不丢失。
 * 上电时通过magic校验判断是否需要清零。
 */

// 缓冲区容量
#define SENSOR_BUF_MAX_RECORDS   128

// 记录标志位
#define SENSOR_FLAG_ALARM        0x01   // 报警记录

// 单条传感器记录 (44字节)
typedef struct {
    float pressure;             // PT304D #2 压力 (Pa) — PB6/PB7 独立总线
    float temperature;          // PT304D #2 温度 (℃)
    float pressure2;            // PT304D #1 压力 (Pa) — PC0/PC1 共用总线
    float temperature2;         // PT304D #1 温度 (℃)
    float voltage;              // 电池电压 (V)
    char  timestamp[20];        // "20xx-xx-xx xx:xx:xx"
    uint8_t flags;              // SENSOR_FLAG_xxx
    uint8_t reserved[3];        // 对齐到44字节
} SensorRecord_t;

// ==================== 接口函数 ====================

// 初始化缓冲区 (上电时调用, magic校验, 无效则清零)
void sensor_buf_init(void);

// 写入一条记录 (满则覆盖最早的)
void sensor_buf_push(const SensorRecord_t *record);

// 读取第i条记录 (0 ~ count-1)
const SensorRecord_t* sensor_buf_get(uint16_t index);

// 弹出（删除）最老的一条记录
void sensor_buf_pop(void);

// 当前记录数
uint16_t sensor_buf_count(void);

// 清空缓冲区 (上报完成后调用)
void sensor_buf_clear(void);

#endif /* SENSOR_BUFFER_H */
