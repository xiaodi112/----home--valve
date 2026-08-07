/**
 * @file eeprom_history.c
 * @brief EEPROM 历史数据环形缓冲实现
 *
 * 存储区: 0x800 ~ 0x1FFF (6144B)
 *   管理头 (32B @0x800): magic(2) + head(2) + tail(2) + count(2) + crc8(1) + reserved(23)
 *   数据区 (6112B @0x820): 191条 x 32B, 每条恰好一页写入
 *
 * 环形覆盖策略: 写满后 head 自动前进, 覆盖最旧记录.
 */

#include "eeprom_history.h"
/* EEPROM chip driver selection (macro in app_config.h): M24M01 / AT24C64 */
#if defined(EEPROM_DRV_AT24C64)
#include "at24c64.h"
#define EE_READ(addr, buf, len)       AT24C64_Read((addr), (buf), (len))
#define EE_WRITE(addr, buf, len)      AT24C64_Write((addr), (buf), (len))
#define EE_WRITE_BYTE(addr, data)     AT24C64_WriteByte((addr), (data))
#else
#include "m24m01.h"
#define EE_READ(addr, buf, len)       M24M01_Read((addr), (buf), (len))
#define EE_WRITE(addr, buf, len)      M24M01_Write((addr), (buf), (len))
#define EE_WRITE_BYTE(addr, data)     M24M01_WriteByte((addr), (data))
#endif
#include "tim.h"
#include "sensor_buffer.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

// ==================== 管理头结构 (32字节, 与页大小对齐) ====================
typedef struct {
    uint16_t magic;          // EE_HIST_MAGIC
    uint16_t head;           // 写指针 (下一个写入的slot索引, 0~190)
    uint16_t tail;           // 读指针 (最旧记录的slot索引)
    uint16_t count;          // 有效记录数 (0~191)
    uint8_t  crc8;           // 管理头前8字节的CRC8
    uint8_t  reserved[23];   // 填充到32字节
} EeHistHeader_t;

// ==================== RAM 缓存管理头 (避免频繁读EEPROM) ====================
static EeHistHeader_t s_hdr;
static uint8_t s_inited = 0;

// ==================== CRC8 (多项式 0x07, 初值 0x00) ====================
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
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// ==================== 内部: 写管理头到EEPROM ====================
static void header_save(void)
{
    s_hdr.crc8 = crc8_calc((const uint8_t *)&s_hdr, 8);
    EE_WRITE(EE_HIST_HEADER_ADDR, (const uint8_t *)&s_hdr, sizeof(EeHistHeader_t));
}

// ==================== 内部: 从EEPROM读管理头 ====================
static uint8_t header_load(void)
{
    EE_READ(EE_HIST_HEADER_ADDR, (uint8_t *)&s_hdr, sizeof(EeHistHeader_t));

    // 校验magic
    if(s_hdr.magic != EE_HIST_MAGIC)
        return 1;

    // 校验CRC
    uint8_t expected_crc = crc8_calc((const uint8_t *)&s_hdr, 8);
    if(s_hdr.crc8 != expected_crc)
        return 2;

    // 范围检查
    if(s_hdr.head >= EE_HIST_MAX_RECORDS ||
       s_hdr.tail >= EE_HIST_MAX_RECORDS ||
       s_hdr.count > EE_HIST_MAX_RECORDS)
        return 3;

    return 0;
}

// ==================== 内部: 格式化(清空)历史区 ====================
static void history_format(void)
{
    memset(&s_hdr, 0, sizeof(s_hdr));
    s_hdr.magic = EE_HIST_MAGIC;
    s_hdr.head = 0;
    s_hdr.tail = 0;
    s_hdr.count = 0;
    header_save();
}

// ==================== 内部: 获取BCD时间戳 ====================
static void get_bcd_timestamp(uint8_t *ts_bcd)
{
    rtc_parameter_struct tp;
    rtc_current_time_get(&tp);
    ts_bcd[0] = (uint8_t)(tp.year   & 0xFF);  // YY (BCD)
    ts_bcd[1] = (uint8_t)(tp.month  & 0xFF);  // MM (BCD)
    ts_bcd[2] = (uint8_t)(tp.date   & 0xFF);  // DD (BCD)
    ts_bcd[3] = (uint8_t)(tp.hour   & 0xFF);  // HH (BCD)
    ts_bcd[4] = (uint8_t)(tp.minute & 0xFF);  // MM (BCD)
    ts_bcd[5] = (uint8_t)(tp.second & 0xFF);  // SS (BCD)
}

// ==================== 内部: 计算记录EEPROM地址 ====================
static uint16_t record_addr(uint16_t slot)
{
    return EE_HIST_DATA_ADDR + (uint16_t)(slot * EE_HIST_RECORD_SIZE);
}

// ==================== 公开接口 ====================

void ee_hist_init(void)
{
    // 每次上电清空历史区, 从第一条开始记录
    // (EEPROM作为唯一存储, 上电即为新周期起点)
    printf_debug("[EE_HIST] Power-on reset, formatting...\r\n");
    history_format();
    s_inited = 1;
}

void ee_hist_push(float pressure, float temperature,
                  float pressure2, float temperature2,
                  float voltage, uint8_t flags)
{
    EeHistRecord_t rec;

    if(!s_inited) return;

    // 填充记录
    rec.pressure = pressure;
    rec.temperature = temperature;
    rec.pressure2 = pressure2;
    rec.temperature2 = temperature2;
    rec.voltage = voltage;
    get_bcd_timestamp(rec.ts_bcd);
    rec.flags = flags;
    memset(rec.reserved, 0, sizeof(rec.reserved));

    // 计算CRC8 (前31字节)
    rec.crc8 = 0;
    rec.crc8 = crc8_calc((const uint8_t *)&rec, EE_HIST_RECORD_SIZE - 1);

    // 写入当前slot (恰好32字节 = 1页写入)
    uint16_t addr = record_addr(s_hdr.head);
    EE_WRITE(addr, (const uint8_t *)&rec, EE_HIST_RECORD_SIZE);

    // 更新环形指针
    s_hdr.head = (s_hdr.head + 1) % EE_HIST_MAX_RECORDS;
    if(s_hdr.count < EE_HIST_MAX_RECORDS)
    {
        s_hdr.count++;
    }
    else
    {
        // 满了, tail跟随前进 (覆盖最旧)
        s_hdr.tail = (s_hdr.tail + 1) % EE_HIST_MAX_RECORDS;
    }

    // 保存管理头
    header_save();
}

uint8_t ee_hist_read(uint16_t index, EeHistRecord_t *out)
{
    if(!s_inited || !out) return 1;
    if(index >= s_hdr.count) return 1;

    // 计算实际slot
    uint16_t slot = (s_hdr.tail + index) % EE_HIST_MAX_RECORDS;
    uint16_t addr = record_addr(slot);

    EE_READ(addr, (uint8_t *)out, EE_HIST_RECORD_SIZE);

    // CRC校验
    uint8_t saved_crc = out->crc8;
    out->crc8 = 0;
    uint8_t calc_crc = crc8_calc((const uint8_t *)out, EE_HIST_RECORD_SIZE - 1);
    out->crc8 = saved_crc;

    if(saved_crc != calc_crc)
        return 2;  // CRC失败

    return 0;
}

uint16_t ee_hist_count(void)
{
    return s_inited ? s_hdr.count : 0;
}

void ee_hist_pop(void)
{
    if(!s_inited || s_hdr.count == 0) return;

    s_hdr.tail = (s_hdr.tail + 1) % EE_HIST_MAX_RECORDS;
    s_hdr.count--;
    header_save();
}

void ee_hist_clear(void)
{
    if(!s_inited) return;
    history_format();
    printf_debug("[EE_HIST] Cleared\r\n");
}

void ee_hist_ts_to_str(const uint8_t *ts_bcd, char *buf)
{
    snprintf(buf, 20, "20%02x-%02x-%02x %02x:%02x:%02x",
             ts_bcd[0], ts_bcd[1], ts_bcd[2],
             ts_bcd[3], ts_bcd[4], ts_bcd[5]);
}

void ee_hist_to_sensor_record(const EeHistRecord_t *hist, void *out)
{
    SensorRecord_t *rec = (SensorRecord_t *)out;
    if(!hist || !rec) return;

    rec->pressure = hist->pressure;
    rec->temperature = hist->temperature;
    rec->pressure2 = hist->pressure2;
    rec->temperature2 = hist->temperature2;
    rec->voltage = hist->voltage;
    rec->flags = hist->flags;
    memset(rec->reserved, 0, sizeof(rec->reserved));

    // BCD时间戳转字符串
    ee_hist_ts_to_str(hist->ts_bcd, rec->timestamp);
}
