/**
 * @file calib_mode.c
 * @brief 未安装自动校零服务 (校零重做版)
 *
 * 未安装模式 = 正常模式 + 三个差异:
 *   采集节拍=CALIB_SAMPLE_MIN分钟 / LCD压力显示0 / 持续自动校零
 * 本模块只提供"状态+算法", 不接管采集调度/上报/下行/4G/睡眠.
 *
 * 编译开关: CALIB_MODE_ENABLED (Middle/app_config.h)
 */

#include "calib_mode.h"
#include "config_store.h"   /* EE_READ / EE_WRITE */
#include "PT_304_35.h"
#include "tim.h"
#include "usart.h"
#include <string.h>
#include <stddef.h>

#if CALIB_MODE_ENABLED

/* ==================== EEPROM 保护区 (双槽+seq+CRC) ==================== */
static CalibEeprom_t g_calib;

/* ==================== 校零会话状态 ==================== */
static uint8_t  s_active = 0;          /* 1=未安装校零激活 (LCD压力显示0, 节拍=宏) */
static uint8_t  s_warmup = 0;
static uint8_t  s_hi_cnt = 0;
static uint8_t  s_zero_frozen = 0;
static uint8_t  s_med_n = 0;
static float    s_med[CALIB_MEDIAN_CNT];
static int32_t  s_zero_pa = 0;         /* 当前零点 */
static uint8_t  s_zero_valid = 0;
static int32_t  s_applied_zero = 0;    /* 已写入PT304D偏移的值 */
static uint32_t s_samples = 0;

static uint8_t calib_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    uint16_t i;
    uint8_t bit;

    for(i = 0; i < len; i++)
    {
        crc ^= data[i];
        for(bit = 0; bit < 8; bit++)
        {
            if(crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x07u);
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static uint8_t calib_slot_valid(const CalibEeprom_t *r)
{
    uint8_t calc = calib_crc8((const uint8_t *)r, (uint16_t)offsetof(CalibEeprom_t, crc8));
    return (r->magic   == EE_MAGIC_CALIB &&
            r->version == EE_CALIB_VERSION &&
            r->installed <= 1 &&
            r->zero_valid <= 1 &&
            r->crc8 == calc) ? 1 : 0;
}

void calib_load_state(void)
{
    CalibEeprom_t a, b;
    uint8_t oka, okb;

    memset(&a, 0, sizeof(a));
    memset(&b, 0, sizeof(b));
    memset(&g_calib, 0, sizeof(g_calib));

    EE_READ(EE_ADDR_CALIB_SLOT_A, (uint8_t *)&a, sizeof(a));
    EE_READ(EE_ADDR_CALIB_SLOT_B, (uint8_t *)&b, sizeof(b));
    oka = calib_slot_valid(&a);
    okb = calib_slot_valid(&b);

    if(oka && okb)
        g_calib = (a.seq >= b.seq) ? a : b;
    else if(oka)
        g_calib = a;
    else if(okb)
        g_calib = b;

    g_calib.magic   = EE_MAGIC_CALIB;
    g_calib.version = EE_CALIB_VERSION;

    /* 未安装但已有冻结零点: 恢复会话零点(掉电/重启连续校零) */
    if(!g_calib.installed && g_calib.zero_valid)
    {
        s_zero_pa     = g_calib.zero_pa;
        s_zero_valid  = 1;
        s_zero_frozen = 1;
        s_samples     = g_calib.sample_cnt;
    }
}

void calib_erase_state(void)
{
    uint8_t ff[sizeof(CalibEeprom_t)];
    memset(ff, 0xFF, sizeof(ff));
    EE_WRITE(EE_ADDR_CALIB_SLOT_A, ff, sizeof(ff));
    EE_WRITE(EE_ADDR_CALIB_SLOT_B, ff, sizeof(ff));
    calib_load_state();
    s_active = 0; s_zero_valid = 0; s_zero_pa = 0; s_applied_zero = 0;
    s_warmup = 0; s_hi_cnt = 0; s_med_n = 0; s_samples = 0; s_zero_frozen = 0;
}

/* 写旧槽+seq+CRC, 读回校验; 返回1=落盘成功 */
static uint8_t calib_save_record(const CalibEeprom_t *rec)
{
    CalibEeprom_t a, b, rb;
    uint8_t oka, okb;
    uint16_t addr, next_seq;
    uint8_t ok = 0;

    memset(&a, 0, sizeof(a));
    memset(&b, 0, sizeof(b));
    EE_READ(EE_ADDR_CALIB_SLOT_A, (uint8_t *)&a, sizeof(a));
    EE_READ(EE_ADDR_CALIB_SLOT_B, (uint8_t *)&b, sizeof(b));
    oka = calib_slot_valid(&a);
    okb = calib_slot_valid(&b);

    if(!oka && !okb)          { addr = EE_ADDR_CALIB_SLOT_A; next_seq = 1; }
    else if(!oka)             { addr = EE_ADDR_CALIB_SLOT_A; next_seq = (uint16_t)(b.seq + 1u); }
    else if(!okb)             { addr = EE_ADDR_CALIB_SLOT_B; next_seq = (uint16_t)(a.seq + 1u); }
    else
    {
        addr = (a.seq <= b.seq) ? EE_ADDR_CALIB_SLOT_A : EE_ADDR_CALIB_SLOT_B;
        next_seq = (uint16_t)((a.seq >= b.seq ? a.seq : b.seq) + 1u);
    }

    g_calib = *rec;
    g_calib.magic   = EE_MAGIC_CALIB;
    g_calib.version = EE_CALIB_VERSION;
    g_calib.seq     = next_seq;
    g_calib.crc8    = calib_crc8((const uint8_t *)&g_calib, (uint16_t)offsetof(CalibEeprom_t, crc8));

    EE_WRITE(addr, (uint8_t *)&g_calib, sizeof(g_calib));

    memset(&rb, 0, sizeof(rb));
    EE_READ(addr, (uint8_t *)&rb, sizeof(rb));
    if(calib_slot_valid(&rb) &&
       rb.seq == g_calib.seq &&
       rb.installed == g_calib.installed &&
       rb.zero_valid == g_calib.zero_valid &&
       rb.zero_pa == g_calib.zero_pa)
    {
        ok = 1;
    }
    else
    {
        calib_load_state();     /* 落盘失败: 恢复旧状态供重试 */
    }
    return ok;
}

static uint8_t calib_persist(uint8_t installed)
{
    CalibEeprom_t rec;
    RtcDateTime_t dt;

    memset(&rec, 0, sizeof(rec));
    rec.magic      = EE_MAGIC_CALIB;
    rec.version    = EE_CALIB_VERSION;
    rec.installed  = installed;
    rec.zero_valid = installed ? (s_zero_valid ? 1u : 0u) : 1u;
    rec.zero_pa    = installed ? (s_zero_valid ? s_zero_pa : 0) : s_zero_pa;
    rec.sample_cnt = s_samples;
    rtc_get_datetime(&dt);
    rec.ts[0] = dec2bcd(dt.year);
    rec.ts[1] = dec2bcd(dt.month);
    rec.ts[2] = dec2bcd(dt.day);
    rec.ts[3] = dec2bcd(dt.hour);
    rec.ts[4] = dec2bcd(dt.minute);
    rec.ts[5] = dec2bcd(dt.second);
    return calib_save_record(&rec);
}

/* ==================== 状态查询/激活 ==================== */
uint8_t calib_is_installed(void) { return g_calib.installed; }
uint8_t calib_zero_valid(void)   { return s_zero_valid; }
int32_t calib_zero_pa(void)      { return s_zero_pa; }
uint8_t calib_is_active(void)    { return s_active; }
uint8_t calib_lcd_zero(void)     { return s_active; }

void calib_get_status(CalibStatus_t *st)
{
    if(!st) return;
    st->warm       = s_warmup;
    st->hi         = s_hi_cnt;
    st->med_n      = s_med_n;
    st->zero_valid = s_zero_valid;
    st->zero_pa    = s_zero_pa;
    st->samples    = s_samples;
}

void calib_set_active(uint8_t on)
{
    s_active = on ? 1u : 0u;
    if(s_active && !s_zero_valid && g_calib.zero_valid)
    {
        s_zero_pa     = g_calib.zero_pa;
        s_zero_valid  = 1;
        s_zero_frozen = 1;
        s_samples     = g_calib.sample_cnt;
    }
}

void calib_apply_zero_offsets(void)
{
    int32_t z;

    if(s_zero_valid)          z = s_zero_pa;
    else if(g_calib.zero_valid)
    {
        z = g_calib.zero_pa;
        s_zero_pa = z;
        s_zero_valid = 1;
    }
    else                      z = 0;

    PT304D_SetZeroOffset(PT304D_SENSOR_2, (float)z);
    s_applied_zero = z;
}

/* ==================== 零点算法 ==================== */
static float calib_median(void)
{
    float tmp[CALIB_MEDIAN_CNT];
    float t;
    uint8_t i, j;
    uint8_t n = s_med_n;

    if(n == 0) return 0.0f;
    if(n > CALIB_MEDIAN_CNT) n = CALIB_MEDIAN_CNT;
    for(i = 0; i < n; i++) tmp[i] = s_med[i];
    for(i = 0; i < n; i++)
        for(j = (uint8_t)(i + 1); j < n; j++)
            if(tmp[j] < tmp[i]) { t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }
    return tmp[n / 2];
}

float calib_feed_sample(float raw_pa)
{
    float delta = 0.0f;
    uint8_t i;

    if(!s_active)
        return 0.0f;

    if(s_warmup < CALIB_WARMUP_CNT)
    {
        s_warmup++;
        return 0.0f;                                /* 上电预热: 不参与判据 */
    }

    if(raw_pa > -CALIB_ZERO_WINDOW_PA && raw_pa < CALIB_ZERO_WINDOW_PA)
    {
        int32_t z;

        /* 零压窗口: 更新滑窗, 零点成熟立即生效 */
        s_hi_cnt = 0;
        s_zero_frozen = 0;
        if(s_med_n < CALIB_MEDIAN_CNT)
        {
            s_med[s_med_n++] = raw_pa;
        }
        else
        {
            for(i = 0; i < CALIB_MEDIAN_CNT - 1; i++)
                s_med[i] = s_med[i + 1];
            s_med[CALIB_MEDIAN_CNT - 1] = raw_pa;
        }
        if(s_samples < 0xFFFFFFF0u) s_samples++;

        if(s_med_n >= CALIB_MEDIAN_CNT)
        {
            z = (int32_t)calib_median();
            if(!s_zero_valid || z != s_zero_pa)
            {
                s_zero_pa    = z;
                s_zero_valid = 1;
                PT304D_SetZeroOffset(PT304D_SENSOR_2, (float)z);
                delta = (float)(z - s_applied_zero);   /* 当前样本同拍修正 */
                s_applied_zero = z;
                printf_debug("[CALIB] zero updated: %dPa (delta=%d)\r\n", (int)z, (int)delta);
            }
        }
        return delta;
    }

    /* 离开零压窗口: 冻结并落盘(掉电/重启安全) */
    if(s_zero_valid && !s_zero_frozen)
    {
        if(calib_persist(0u))
            s_zero_frozen = 1;
    }

    if(raw_pa >= CALIB_INSTALL_PA || raw_pa <= -CALIB_INSTALL_PA)
    {
        if(s_hi_cnt < 250u) s_hi_cnt++;
        if(s_hi_cnt >= CALIB_HIGH_CONFIRM_CNT)
        {
            /* 安装锁定: 有成熟零点用它; 从未校零(上电即带压)锁0偏移 */
            if(calib_persist(1u))
            {
                s_active = 0;
                printf_debug("[CALIB] INSTALLED & LOCKED (zero=%dPa valid=%d cnt=%lu)\r\n",
                             g_calib.zero_pa, g_calib.zero_valid,
                             (unsigned long)g_calib.sample_cnt);
            }
        }
    }
    else
    {
        s_hi_cnt = 0;                               /* 迟滞带: 不计连续高位 */
    }
    return 0.0f;
}

/* ==================== 平台强制置已安装 ==================== */
uint8_t calib_force_install(void)
{
    calib_load_state();
    if(g_calib.installed)
        return 1;                                   /* 幂等成功 */

    if(!s_zero_valid && g_calib.zero_valid)
    {
        s_zero_pa    = g_calib.zero_pa;
        s_zero_valid = 1;
    }
    if(calib_persist(1u))
    {
        calib_apply_zero_offsets();                 /* 立即生效 */
        s_active = 0;
        printf_debug("[CALIB] FORCE INSTALLED & LOCKED (zero=%dPa valid=%d)\r\n",
                     g_calib.zero_pa, g_calib.zero_valid);
        return 1;
    }
    return 0;
}

#else /* CALIB_MODE_ENABLED == 0 */

void calib_load_state(void) { }
void calib_erase_state(void) { }
uint8_t calib_is_installed(void) { return 0; }
uint8_t calib_zero_valid(void) { return 0; }
int32_t calib_zero_pa(void) { return 0; }
uint8_t calib_is_active(void) { return 0; }
void calib_set_active(uint8_t on) { (void)on; }
void calib_get_status(CalibStatus_t *st)
{
    if(st) memset(st, 0, sizeof(*st));
}
uint8_t calib_lcd_zero(void) { return 0; }
float calib_feed_sample(float raw_pa) { (void)raw_pa; return 0.0f; }
uint8_t calib_force_install(void) { return 0; }
void calib_apply_zero_offsets(void) { }

#endif /* CALIB_MODE_ENABLED */
