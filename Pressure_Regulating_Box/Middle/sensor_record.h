#ifndef SENSOR_RECORD_H
#define SENSOR_RECORD_H

#include <stdint.h>

/**
 * @file sensor_record.h
 * @brief 传感器记录结构定义 (EEPROM 历史 / 上报接口共用)
 */

/* 记录标志位 */
#define SENSOR_FLAG_ALARM        0x01   /* 报警记录 */

/* 单条传感器记录 (44 字节) */
typedef struct {
    float pressure;             /* PT304D #2 压力 (Pa) - PB6/PB7 独立总线 */
    float temperature;          /* PT304D #2 温度 (C) */
    float pressure2;            /* PT304D #1 压力 (Pa) - PC0/PC1 共用总线 */
    float temperature2;         /* PT304D #1 温度 (C) */
    float voltage;              /* 电池电压 (V) */
    char  timestamp[20];        /* "20xx-xx-xx xx:xx:xx" */
    uint8_t flags;              /* SENSOR_FLAG_xxx */
    uint8_t reserved[3];        /* 对齐到 4 字节 */
} SensorRecord_t;

#endif /* SENSOR_RECORD_H */
