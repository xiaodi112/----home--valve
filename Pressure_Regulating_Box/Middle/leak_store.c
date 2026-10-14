/**
 * @file leak_store.c
 * @brief 静态测漏数据暂存区实现 (M24M01 独占分区)
 *
 * 布局: 管理头 32B @0x1000 + 数据 2047x32B @0x1020~0x10FFF
 * 特点: 每条 32B 恰好一页写入(不跨页), 自带 CRC8; 无环形覆盖, 无断点续传。
 */

#include "leak_store.h"
#include "app_config.h"
#include "usart.h"
#include "tim.h"
#include <string.h>
#include <stdio.h>

#if defined(EEPROM_DRV_M24M01)

#include "m24m01.h"
#define LEAK_EE_READ(addr, buf, len)    M24M01_Read((addr), (buf), (len))
#define LEAK_EE_WRITE(addr, buf, len)   M24M01_Write((addr), (buf), (len))

/* ==================== 模块状态 ==================== */
static LeakStoreHeader_t s_hdr;
static uint16_t s_count = 0;      /* RAM 计数 (push 维护, 不依赖EEPROM) */
static uint8_t  s_inited = 0;

/* ==================== CRC8 (多项式 0x07, 与历史记录一致) ==================== */
static uint8_t crc8_calc(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    uint16_t i;
    uint8_t j;
    for(i = 0; i < len; i++)
    {
        crc ^= data[i];
        for(j = 0; j < 8; j++)
        {
            if(crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x07);
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* ==================== CRC16-CCITT (任务号校验/调试一致性) ==================== */
static uint16_t crc16_calc(const char *str)
{
    uint16_t crc = 0xFFFF;
    if(str == NULL) return 0;
    while(*str)
    {
        crc ^= (uint16_t)((uint8_t)*str++) << 8;
        for(uint8_t i = 0; i < 8; i++)
        {
            if(crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ==================== 内部: 管理头读写 ==================== */
static uint8_t header_save(void)
{
    s_hdr.crc8 = crc8_calc((const uint8_t *)&s_hdr, sizeof(LeakStoreHeader_t) - 1);
    return (LEAK_EE_WRITE(EE_SIDELEAK_HEADER_ADDR,
                          (const uint8_t *)&s_hdr,
                          sizeof(LeakStoreHeader_t)) == 0) ? 0 : 1;
}

static void header_default(uint16_t state, uint16_t count)
{
    memset(&s_hdr, 0, sizeof(s_hdr));
    s_hdr.magic  = LEAK_STORE_MAGIC;
    s_hdr.state  = state;
    s_hdr.count  = count;
}

/* ==================== 内部: 记录地址 ==================== */
static uint32_t record_addr(uint16_t index)
{
    return (uint32_t)EE_SIDELEAK_DATA_ADDR + ((uint32_t)index * (uint32_t)LEAK_RECORD_SIZE);
}

/* ==================== 内部: 取当前 BCD 时间戳 ==================== */
static void get_bcd_timestamp(uint8_t *ts_bcd)
{
    rtc_parameter_struct tp;
    rtc_current_time_get(&tp);
    ts_bcd[0] = (uint8_t)(tp.year   & 0xFF);  /* YY (BCD) */
    ts_bcd[1] = (uint8_t)(tp.month  & 0xFF);  /* MM */
    ts_bcd[2] = (uint8_t)(tp.date   & 0xFF);  /* DD */
    ts_bcd[3] = (uint8_t)(tp.hour   & 0xFF);  /* HH */
    ts_bcd[4] = (uint8_t)(tp.minute & 0xFF);  /* MM */
    ts_bcd[5] = (uint8_t)(tp.second & 0xFF);  /* SS */
}

/* ==================== 公开接口 ==================== */

void leak_store_init(void)
{
    uint8_t rd_ok = (LEAK_EE_READ(EE_SIDELEAK_HEADER_ADDR,
                                  (uint8_t *)&s_hdr,
                                  sizeof(LeakStoreHeader_t)) == 0) ? 1 : 0;

    if(rd_ok && s_hdr.magic == LEAK_STORE_MAGIC &&
       s_hdr.crc8 == crc8_calc((const uint8_t *)&s_hdr, sizeof(LeakStoreHeader_t) - 1))
    {
        if(s_hdr.state != LEAK_STORE_STATE_IDLE || s_hdr.count != 0)
            printf_debug("[LEAK] power-on leftover (state=%d count=%d) -> clear\r\n",
                         s_hdr.state, s_hdr.count);
    }
    else
    {
        printf_debug("[LEAK] store header invalid -> format\r\n");
    }

    /* 不做断点续传: 上电一律清零 */
    leak_store_clear();
    printf_debug("[LEAK] store ready, capacity=%d rec @0x%05X~0x%05X\r\n",
                 LEAK_STORE_MAX_RECORDS, EE_SIDELEAK_DATA_ADDR, EE_SIDELEAK_END_ADDR);
    s_inited = 1;
}

uint8_t leak_store_clear(void)
{
    header_default(LEAK_STORE_STATE_IDLE, 0);
    s_count = 0;
    s_inited = 1;
    return header_save();
}

uint8_t leak_store_begin(const char *taskNumber)
{
    uint8_t i;
    uint16_t len = 0;

    header_default(LEAK_STORE_STATE_COLLECT, 0);

    if(taskNumber != NULL)
    {
        s_hdr.task_crc16 = crc16_calc(taskNumber);
        len = (uint16_t)strlen(taskNumber);
        if(len > LEAK_HEAD_TAIL_LEN) len = LEAK_HEAD_TAIL_LEN;
        /* 只留尾部: 任务号前段是固定前缀+时间, 尾部含表号与随机码, 更具区分度 */
        memcpy(s_hdr.task_tail,
               taskNumber + (strlen(taskNumber) - len),
               len);
    }

    s_count = 0;
    s_inited = 1;

    for(i = 0; i < 3; i++)   /* 页写重试: 掉电/写失败重试, 最多3次 */
    {
        if(header_save() == 0) return 0;
    }
    printf_debug("[LEAK] begin header write FAILED\r\n");
    return 1;
}

uint8_t leak_store_commit(uint16_t count)
{
    s_hdr.state = LEAK_STORE_STATE_UPLOAD;
    s_hdr.count = count;
    return header_save();
}

uint8_t leak_store_push(float pressure, float temperature,
                        float pressure2, float temperature2)
{
    LeakRecord_t rec;

    if(!s_inited) return 1;
    if(s_count >= LEAK_STORE_MAX_RECORDS) return 1;   /* 满: 由调用方保证不超限 */

    rec.pressure    = pressure;
    rec.temperature = temperature;
    rec.pressure2   = pressure2;
    rec.temperature2 = temperature2;
    get_bcd_timestamp(rec.ts_bcd);
    rec.flags = 0;
    memset(rec.reserved, 0, sizeof(rec.reserved));

    rec.crc8 = 0;
    rec.crc8 = crc8_calc((const uint8_t *)&rec, LEAK_RECORD_SIZE - 1);

    if(LEAK_EE_WRITE(record_addr(s_count), (const uint8_t *)&rec, LEAK_RECORD_SIZE) != 0)
    {
        printf_debug("[LEAK] record write fail @idx=%d\r\n", s_count);
        return 1;
    }

    s_count++;
    return 0;
}

uint16_t leak_store_count(void)
{
    return s_count;
}

uint8_t leak_store_read(uint16_t index, LeakRecord_t *out)
{
    uint8_t saved_crc;
    uint8_t calc_crc;

    if(out == NULL) return 1;
    if(index >= s_count) return 1;

    if(LEAK_EE_READ(record_addr(index), (uint8_t *)out, LEAK_RECORD_SIZE) != 0)
        return 2;

    saved_crc = out->crc8;
    out->crc8 = 0;
    calc_crc  = crc8_calc((const uint8_t *)out, LEAK_RECORD_SIZE - 1);
    out->crc8 = saved_crc;

    return (saved_crc == calc_crc) ? 0 : 2;
}

void leak_store_ts_to_str(const uint8_t *ts_bcd, char *buf)
{
    snprintf(buf, 20, "20%02x-%02x-%02x %02x:%02x:%02x",
             ts_bcd[0], ts_bcd[1], ts_bcd[2],
             ts_bcd[3], ts_bcd[4], ts_bcd[5]);
}

#else  /* AT24C64: 无测漏分区, 提供空实现保证工程可编译 */

void leak_store_init(void)
{
    printf_debug("[LEAK] not supported on AT24C64\r\n");
}
uint8_t leak_store_clear(void)                            { return 1; }
uint8_t leak_store_begin(const char *taskNumber)          { (void)taskNumber; return 1; }
uint8_t leak_store_commit(uint16_t count)                 { (void)count; return 1; }
uint8_t leak_store_push(float p, float t, float p2, float t2)
{
    (void)p; (void)t; (void)p2; (void)t2;
    return 1;
}
uint16_t leak_store_count(void)                           { return 0; }
uint8_t leak_store_read(uint16_t index, LeakRecord_t *out)
{
    (void)index; (void)out;
    return 1;
}
void leak_store_ts_to_str(const uint8_t *ts_bcd, char *buf)
{
    (void)ts_bcd;
    if(buf) buf[0] = '\0';
}

#endif /* EEPROM_DRV_M24M01 */

/* ==================== 编译期布局断言 ==================== */
/* 记录/管理头必须恰好 32B: 否则一次写入跨页, 页写语义被破坏 */
typedef char leak_record_size_check[(sizeof(LeakRecord_t) == 32) ? 1 : -1];
typedef char leak_header_size_check[(sizeof(LeakStoreHeader_t) == 32) ? 1 : -1];
