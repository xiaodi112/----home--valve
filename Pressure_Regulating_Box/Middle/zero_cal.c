/**
 * @file zero_cal.c
 * @brief 零点(大气基准)管理: 采集/校验/双槽存储/上电只读应用
 *
 * 层次: 中间层服务. 只调用驱动(PT304D_ReadMeasData / PT304D_SetZeroOffset)、
 *       EEPROM 读写宏(config_store.h)与 BSP(soft_i2c/delay/tim/usart)。
 */

#include "zero_cal.h"
#include "app_config.h"
#include "config_store.h"      /* EE_READ / EE_WRITE (按 EEPROM 芯片选择) */
#include "PT_304_35.h"
#include "soft_i2c.h"
#include "delay.h"
#include "tim.h"
#include "usart.h"
#include "main.h"              /* g_sensor: 重置压力变化基准 */
#include <string.h>
#include <stddef.h>

#define ZERO_CAL_MAGIC        0xE1u
#define ZERO_CAL_VERSION      1u
#define ZERO_CAL_FLAG_VALID   0x01u

/* 双槽地址: A/B 分处不同 256B 页, 避免同页写入相互影响 */
#define EE_ADDR_ZERO_SLOT_A   0x0E00u
#define EE_ADDR_ZERO_SLOT_B   0x0F00u

typedef struct {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  flags;          /* bit0: 记录有效 */
    uint16_t seq;            /* 递增序号, 大者优先 */
    int32_t  zero_pa;        /* 基准值(Pa) */
    uint16_t sample_cnt;     /* 参与平均的采样次数(追溯用) */
    uint8_t  ts[6];          /* BCD: YY MM DD HH MM SS */
    uint8_t  reserved[3];
    uint8_t  crc8;
} ZeroCalRec_t;              /* 18 字节 */

static int32_t s_zero_pa    = ZERO_CAL_DEFAULT_PA;
static uint8_t s_zero_valid = 0;   /* 1 = 来自 EEPROM 的有效基准 */

/* ==================== CRC8 (poly 0x07) ==================== */
static uint8_t zc_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    uint16_t i;
    uint8_t bit;

    for(i = 0; i < len; i++)
    {
        crc ^= data[i];
        for(bit = 0; bit < 8; bit++)
        {
            if(crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x07u);
            else           crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static uint8_t zc_rec_valid(const ZeroCalRec_t *r)
{
    uint8_t calc = zc_crc8((const uint8_t *)r, (uint16_t)offsetof(ZeroCalRec_t, crc8));

    return (r->magic   == ZERO_CAL_MAGIC &&
            r->version == ZERO_CAL_VERSION &&
            (r->flags & ZERO_CAL_FLAG_VALID) != 0 &&
            r->crc8 == calc) ? 1 : 0;
}

/* ==================== 应用基准到驱动 ==================== */
static void zc_apply(int32_t pa)
{
    s_zero_pa = pa;
    PT304D_SetZeroOffset(ZERO_CAL_SENSOR_IDX, (float)pa);
}

int32_t zero_cal_get(void)      { return s_zero_pa; }
uint8_t zero_cal_is_valid(void) { return s_zero_valid; }

/* ==================== 上电: 读双槽 → 应用 (只读不改) ==================== */
void zero_cal_load_apply(void)
{
    ZeroCalRec_t a, b;
    const ZeroCalRec_t *best = NULL;

    memset(&a, 0, sizeof(a));
    memset(&b, 0, sizeof(b));

    EE_READ(EE_ADDR_ZERO_SLOT_A, (uint8_t *)&a, sizeof(a));
    EE_READ(EE_ADDR_ZERO_SLOT_B, (uint8_t *)&b, sizeof(b));

    if(zc_rec_valid(&a)) best = &a;
    if(zc_rec_valid(&b) && (best == NULL || (int16_t)(b.seq - best->seq) > 0))
        best = &b;

    if(best != NULL)
    {
        zc_apply(best->zero_pa);
        s_zero_valid = 1;
        printf_debug("[ZERO] baseline loaded: %ldPa (seq=%u cnt=%u)\r\n",
                     (long)best->zero_pa, best->seq, best->sample_cnt);
    }
    else
    {
        /* 无有效基准: 先用编译期默认值保证测量可用, 由调用方决定是否自动采集 */
        zc_apply(ZERO_CAL_DEFAULT_PA);
        s_zero_valid = 0;
        printf_debug("[ZERO] no valid baseline -> default %ldPa (calibration needed)\r\n",
                     (long)ZERO_CAL_DEFAULT_PA);
    }
}

/* ==================== 清除基准 (恢复出厂 reset 调用) ==================== */
void zero_cal_erase(void)
{
    uint8_t ff[sizeof(ZeroCalRec_t)];

    memset(ff, 0xFF, sizeof(ff));
    EE_WRITE(EE_ADDR_ZERO_SLOT_A, ff, sizeof(ff));
    EE_WRITE(EE_ADDR_ZERO_SLOT_B, ff, sizeof(ff));

    s_zero_valid = 0;
    zc_apply(ZERO_CAL_DEFAULT_PA);      /* 本次会话回落到默认基准 */

    printf_debug("[ZERO] baseline erased (factory reset); default %ldPa, will re-calibrate on boot\r\n",
                 (long)ZERO_CAL_DEFAULT_PA);
}

/* ==================== 采集 + 校验 (5 次 x 500ms) ==================== */
uint8_t zero_cal_capture(int32_t *out_pa)
{
    uint8_t i;
    int32_t sum = 0;
    PT304D_Data d;
    float saved_off;
    uint8_t ok = 1;

    if(out_pa == NULL)
        return 0;

    /* 采样期间必须把驱动偏移清零: 否则读到的是"读数 - 当前基准"(≈0), 窗口判据永远不成立。
     * 本函数无副作用: 无论成功失败, 退出前都恢复原偏移, 由调用方决定何时应用新基准。 */
    saved_off = PT304D_GetZeroOffset(ZERO_CAL_SENSOR_IDX);
    PT304D_SetZeroOffset(ZERO_CAL_SENSOR_IDX, 0.0f);

    soft_i2c_sensors_init();     /* 传感器总线 GPIO (深睡后为 Analog, 需重新配置) */
    sensor_pwr_on();             /* PC3 上电 + 稳定延时 */

    for(i = 0; i < ZERO_CAL_SAMPLE_CNT; i++)
    {
        if(i > 0)
        {
            trea_delay_ms(ZERO_CAL_SAMPLE_MS);
            swdt_feed();
        }

        memset(&d, 0, sizeof(d));
        PT304D_ReadMeasData(ZERO_CAL_SENSOR_IDX, &d);

        if(d.err != PT304D_OK)
        {
            printf_debug("[ZERO] sample %u/%u FAILED (err=%d), abort\r\n",
                         i + 1, ZERO_CAL_SAMPLE_CNT, d.err);
            ok = 0;
            break;
        }

        if(d.pressure < (float)ZERO_CAL_WINDOW_MIN_PA ||
           d.pressure > (float)ZERO_CAL_WINDOW_MAX_PA)
        {
            printf_debug("[ZERO] sample %u/%u out of window: %.0fPa (need %d~%dPa), abort\r\n",
                         i + 1, ZERO_CAL_SAMPLE_CNT, (double)d.pressure,
                         ZERO_CAL_WINDOW_MIN_PA, ZERO_CAL_WINDOW_MAX_PA);
            ok = 0;
            break;
        }

        printf_debug("[ZERO] sample %u/%u: %.0fPa\r\n",
                     i + 1, ZERO_CAL_SAMPLE_CNT, (double)d.pressure);
        sum += (int32_t)d.pressure;
    }

    sensor_pwr_off();
    PT304D_SetZeroOffset(ZERO_CAL_SENSOR_IDX, saved_off);   /* 恢复原偏移(无副作用) */

    if(!ok)
        return 0;

    *out_pa = sum / (int32_t)ZERO_CAL_SAMPLE_CNT;
    return 1;
}

/* ==================== 写入双槽 + 回读校验 ==================== */
static uint8_t zc_save(int32_t pa)
{
    ZeroCalRec_t a, b, rec, rb;
    uint16_t seq = 1u;
    uint16_t addr = EE_ADDR_ZERO_SLOT_A;
    uint8_t oka, okb;
    RtcDateTime_t dt;

    memset(&a, 0, sizeof(a));
    memset(&b, 0, sizeof(b));
    EE_READ(EE_ADDR_ZERO_SLOT_A, (uint8_t *)&a, sizeof(a));
    EE_READ(EE_ADDR_ZERO_SLOT_B, (uint8_t *)&b, sizeof(b));
    oka = zc_rec_valid(&a);
    okb = zc_rec_valid(&b);

    /* 总是写入"较旧/无效"的槽, 保留另一槽的旧基准直到新记录校验通过 */
    if(oka && okb)
    {
        if((int16_t)(a.seq - b.seq) >= 0) { seq = (uint16_t)(a.seq + 1u); addr = EE_ADDR_ZERO_SLOT_B; }
        else                              { seq = (uint16_t)(b.seq + 1u); addr = EE_ADDR_ZERO_SLOT_A; }
    }
    else if(oka) { seq = (uint16_t)(a.seq + 1u); addr = EE_ADDR_ZERO_SLOT_B; }
    else if(okb) { seq = (uint16_t)(b.seq + 1u); addr = EE_ADDR_ZERO_SLOT_A; }

    memset(&rec, 0, sizeof(rec));
    rec.magic      = ZERO_CAL_MAGIC;
    rec.version    = ZERO_CAL_VERSION;
    rec.flags      = ZERO_CAL_FLAG_VALID;
    rec.seq        = seq;
    rec.zero_pa    = pa;
    rec.sample_cnt = ZERO_CAL_SAMPLE_CNT;

    rtc_get_datetime(&dt);
    rec.ts[0] = dec2bcd(dt.year);
    rec.ts[1] = dec2bcd(dt.month);
    rec.ts[2] = dec2bcd(dt.day);
    rec.ts[3] = dec2bcd(dt.hour);
    rec.ts[4] = dec2bcd(dt.minute);
    rec.ts[5] = dec2bcd(dt.second);

    rec.crc8 = zc_crc8((const uint8_t *)&rec, (uint16_t)offsetof(ZeroCalRec_t, crc8));

    EE_WRITE(addr, (uint8_t *)&rec, sizeof(rec));

    /* 写后回读校验 */
    memset(&rb, 0, sizeof(rb));
    EE_READ(addr, (uint8_t *)&rb, sizeof(rb));
    if(zc_rec_valid(&rb) && rb.seq == rec.seq && rb.zero_pa == rec.zero_pa)
        return 1;

    printf_debug("[ZERO] save verify FAILED at 0x%04X\r\n", addr);
    return 0;
}

/* ==================== 完整校准流程 ==================== */
uint8_t zero_cal_run(void)
{
    int32_t pa = 0;

    printf_debug("[ZERO] calibration start (sensor=%d, %u x %ums, window %d~%dPa)\r\n",
                 ZERO_CAL_SENSOR_IDX, ZERO_CAL_SAMPLE_CNT, ZERO_CAL_SAMPLE_MS,
                 ZERO_CAL_WINDOW_MIN_PA, ZERO_CAL_WINDOW_MAX_PA);

    if(!zero_cal_capture(&pa))
    {
        printf_debug("[ZERO] calibration ABORTED: sampling/window check failed, old baseline kept\r\n");
        return 0;
    }

    if(!zc_save(pa))
    {
        printf_debug("[ZERO] calibration ABORTED: eeprom write/verify failed, old baseline kept\r\n");
        return 0;
    }

    zc_apply(pa);
    s_zero_valid = 1;

    /* 零点变化后重置压力变化基准: 否则第一帧的 ΔP 等于新旧基准之差,
     * 可能被突变判据当成压力突变(若异常策略含 pvmutation 会误关阀)。
     * last_pressure < 0 表示"首次采集", 判据会跳过突变项。 */
    g_sensor.last_pressure_collect   = -1.0f;
    g_sensor.last_pressure_collect2  = -1.0f;
    g_sensor.last_pressure_realtime  = -1.0f;
    g_sensor.last_pressure_realtime2 = -1.0f;

    printf_debug("[ZERO] calibration OK: baseline=%ldPa saved\r\n", (long)pa);
    return 1;
}
