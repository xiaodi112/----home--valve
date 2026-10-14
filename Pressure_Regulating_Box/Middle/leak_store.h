#ifndef LEAK_STORE_H
#define LEAK_STORE_H

#include <stdint.h>
#include "app_config.h"
#include "eeprom_history.h"

/**
 * @file leak_store.h
 * @brief 静态测漏数据暂存区 (M24M01 独占分区, 与历史环形区完全独立)
 *
 * 分区 (地址定义见 eeprom_history.h, 仅 M24M01 存在):
 *   管理头 32B  @0x1000
 *   数据区     2047 条 x 32B @0x1020~0x10FFF
 *
 * 定位: 单次测漏任务的"整批数据暂存区", 不做环形覆盖、不做断点续传。
 *   上电/任务开始 → 清零; 上报流程结束 → 清零(不论成败)。
 *   管理头只在 3 个时刻各写一次(开始/采集完成/清除), 不随采样更新, 避免单页磨损。
 *
 * 记录 32B 与历史记录同构: 一页一次写入, 不跨页; 自带 CRC8 校验。
 */

#define LEAK_STORE_MAGIC        0x5EA1u
#define LEAK_STORE_STATE_IDLE   0       /* 空闲(空区) */
#define LEAK_STORE_STATE_COLLECT 1      /* 采集中 */
#define LEAK_STORE_STATE_UPLOAD 2       /* 采集完成, 待上报/上报中 */

#if defined(EEPROM_DRV_M24M01)
#define LEAK_STORE_SUPPORTED    1
#define LEAK_STORE_MAX_RECORDS  EE_SIDELEAK_RECORDS      /* 2047 */
#define LEAK_RECORD_SIZE        EE_HIST_RECORD_SIZE      /* 32 */
#else
#define LEAK_STORE_SUPPORTED    0                        /* AT24C64: 无测漏分区 */
#define LEAK_STORE_MAX_RECORDS  0
#define LEAK_RECORD_SIZE        32
#endif

/* 任务号最长约 33 字符(LEAK+14位时间+12位表号+3位), 头里只留尾部做调试痕迹 */
#define LEAK_TASK_NUMBER_MAX    40
#define LEAK_HEAD_TAIL_LEN      22

/* 管理头 (32B, 页对齐) */
typedef struct {
    uint16_t magic;                       /* LEAK_STORE_MAGIC */
    uint16_t state;                       /* LEAK_STORE_STATE_xxx */
    uint16_t count;                       /* 已写入有效记录数 (0~2047) */
    uint16_t task_crc16;                  /* taskNumber 的 CRC16 */
    char     task_tail[LEAK_HEAD_TAIL_LEN];/* taskNumber 尾部 (调试用) */
    uint8_t  crc8;                        /* 前31字节 CRC8 */
    uint8_t  reserved;
} LeakStoreHeader_t;                      /* 合计 32B */

/* 单条测漏记录 (32B, 与历史记录同构) */
typedef struct {
    float    pressure;      /* 进气端压力 (Pa) */
    float    temperature;   /* 进气端温度 (C)  */
    float    pressure2;     /* 出气端压力 (Pa) */
    float    temperature2;  /* 出气端温度 (C)  */
    uint8_t  ts_bcd[6];     /* 时间戳 BCD: YY MM DD HH MM SS */
    uint8_t  flags;         /* 保留 */
    uint8_t  crc8;          /* 前31字节 CRC8 */
    uint8_t  reserved[8];   /* 预留扩展 */
} LeakRecord_t;                           /* 合计 32B */

/* ==================== 接口 ==================== */

/**
 * 上电初始化: 读管理头并打印状态; 无论是否残留有效数据一律清零
 * (本功能不做断点续传, 掉电中途的残留数据不再使用)
 */
void leak_store_init(void);

/**
 * 任务开始: 写入管理头 (state=采集中, count=0, 记录 taskNumber 校验)
 * @param taskNumber 任务序号字符串 (可为 NULL)
 * @return 0=成功, 1=失败
 */
uint8_t leak_store_begin(const char *taskNumber);

/**
 * 采集结束: 把管理头更新为 state=待上报 并落盘条数 (掉电取证用)
 * @return 0=成功, 1=失败
 */
uint8_t leak_store_commit(uint16_t count);

/**
 * 逻辑清零 (不擦除数据页, 只写管理头): 上报流程结束后调用, 不论成败
 * @return 0=成功, 1=失败
 */
uint8_t leak_store_clear(void);

/**
 * 追加一条记录 (时间戳取当前 RTC)
 * @return 0=成功, 1=满/未初始化/写失败
 */
uint8_t leak_store_push(float pressure, float temperature,
                        float pressure2, float temperature2);

/** 当前暂存记录数 (RAM 计数, 由 push 维护) */
uint16_t leak_store_count(void);

/**
 * 读取第 index 条 (0=最新写入顺序的第1条)
 * @return 0=成功, 1=索引越界, 2=CRC校验失败
 */
uint8_t leak_store_read(uint16_t index, LeakRecord_t *out);

/** BCD 时间戳转 "20YY-MM-DD HH:MM:SS" (buf 至少 20 字节) */
void leak_store_ts_to_str(const uint8_t *ts_bcd, char *buf);

#endif /* LEAK_STORE_H */
