/**
 * @file ch4_sensor.c
 * @brief CH4 laser sensor driver - independent, portable, zero upper-layer deps
 * @details Frame: "+000.00 +21.4 1001.01 00 28\r\n" (29 bytes, XOR checksum)
 */

#include "ch4_sensor.h"
#include "power.h"
#include "usart.h"
#include "app_config.h"
#include "delay.h"
#include <stdlib.h>
#include <string.h>

#if LASER_CH4_ENABLED

/* 帧格式常量按传感器型号编译选择 (CH4_SENSOR_MODEL_A) */
#if CH4_SENSOR_MODEL_A
#define CH4_FRAME_LEN     22      /* 总帧长 (含帧头'A'与\r\n) */
#define CH4_PAYLOAD_LEN   21      /* 去掉帧头后的解析长度 */
#define CH4_XOR_LEN       18      /* 校验范围 bytes[0..17], 含帧头 */
#define CH4_TERM_CR_IDX   20      /* '\r' 所在索引 */
#define CH4_FIELD_COUNT   4       /* 字段数: 浓度/温度/故障码/异或 */
#define CH4_HEADER_BYTE   'A'
#else
#define CH4_FRAME_LEN     29
#define CH4_PAYLOAD_LEN   29
#define CH4_XOR_LEN       25      /* bytes[0..24] */
#define CH4_TERM_CR_IDX   27
#define CH4_FIELD_COUNT   5
#endif
#define CH4_COLLECT_TIMEOUT_MS  2000

static uint32_t s_warmup_ms = 20000;
static uint32_t s_warmup_start = 0;
static uint8_t  s_warmup_active = 0;

void ch4_power_on(void)
{
    /* 显式配置 PC13 为推挽输出再置位: 定时调度可能从最小唤醒直接进入 STATE_CH4,
     * 此时 PC13 仍为 ANALOG (gpio_enter_low_power 置位), 直接写 ODR 无效,
     * 会导致激光传感器供电缺失 */
    rcu_periph_clock_enable(RCU_GPIOC);
    gpio_mode_set(PWR_CH4_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_CH4_PIN);
    gpio_output_options_set(PWR_CH4_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, PWR_CH4_PIN);
    power_ch4_set(1);
    usart1_ch4_init();
    s_warmup_start = trea_millis();
    s_warmup_active = 1;
}

void ch4_power_off(void)
{
    usart1_ch4_deinit();
    power_ch4_set(0);
    s_warmup_active = 0;
}

void ch4_set_warmup_time(uint32_t ms)
{
    s_warmup_ms = ms;
}

uint8_t ch4_is_warmup_done(void)
{
    if(!s_warmup_active) return 0;
    if((trea_millis() - s_warmup_start) >= s_warmup_ms) {
        s_warmup_active = 0;
        return 1;
    }
    return 0;
}

/* parse a frame into CH4_Data_t (支持型号A与原型号, 见 CH4_SENSOR_MODEL_A) */
static CH4_Status_t parse_frame(const uint8_t *raw, uint16_t len, CH4_Data_t *data)
{
    if(data == NULL) return CH4_ERR_NULLPTR;
    if(len < CH4_FRAME_LEN) return CH4_ERR_FORMAT;

#if CH4_SENSOR_MODEL_A
    /* 帧头校验: 型号A 固定 'A' */
    if(raw[0] != CH4_HEADER_BYTE) return CH4_ERR_FORMAT;
#endif

    /* verify \r\n terminator */
    if(raw[CH4_TERM_CR_IDX] != '\r' || raw[CH4_TERM_CR_IDX + 1] != '\n') return CH4_ERR_FORMAT;

    /* make a local copy for strtok (need mutable buffer) */
    const uint8_t *src = raw;
#if CH4_SENSOR_MODEL_A
    src = raw + 1;   /* 跳过帧头, 避免 strtof("A+000.00") 解析为 0 */
#endif
    char buf[CH4_PAYLOAD_LEN + 1];
    memcpy(buf, src, CH4_PAYLOAD_LEN);
    buf[CH4_PAYLOAD_LEN] = '\0';

    /* XOR checksum over bytes[0..CH4_XOR_LEN-1] (含帧头与校验前所有字符) */
    uint8_t xor_val = 0;
    uint16_t i;
    for(i = 0; i < CH4_XOR_LEN; i++)
        xor_val ^= raw[i];

    /* parse space-separated tokens */
    char *tok[CH4_FIELD_COUNT];
    int n = 0;
    char *p = strtok(buf, " ");
    while(p && n < CH4_FIELD_COUNT) {
        tok[n++] = p;
        p = strtok(NULL, " \r\n");
    }
    if(n < CH4_FIELD_COUNT) return CH4_ERR_FORMAT;

    /* 最后字段 = checksum (hex) */
    uint8_t expected_xor = (uint8_t)strtol(tok[CH4_FIELD_COUNT - 1], NULL, 16);
    if(xor_val != expected_xor) return CH4_ERR_CHECKSUM;

    /* fill output */
    data->concentration = strtof(tok[0], NULL);
    data->temperature   = strtof(tok[1], NULL);
#if CH4_SENSOR_MODEL_A
    data->pressure      = 0.0f;                          /* 型号A 无压力字段 */
    data->status        = (uint8_t)strtol(tok[2], NULL, 16);   /* 故障码 */
#else
    data->pressure      = strtof(tok[2], NULL);
    data->status        = (uint8_t)strtol(tok[3], NULL, 16);
#endif
    data->data_valid    = 1;

    return CH4_OK;
}

CH4_Status_t ch4_collect(CH4_Data_t *data)
{
    if(data == NULL) return CH4_ERR_NULLPTR;
    memset(data, 0, sizeof(CH4_Data_t));
    ch4_clear_frame_ready();   /* 丢弃暖机期残留的噪声/旧帧, 等待新帧 */

    /* wait for IDLE frame ready (2s timeout) */
    uint32_t t0 = trea_millis();
    while(!ch4_get_frame_ready()) {
        if((trea_millis() - t0) >= CH4_COLLECT_TIMEOUT_MS)
            return CH4_ERR_TIMEOUT;
    }

    uint8_t *rx_buf = ch4_get_rx_buf();
    uint16_t rx_len = ch4_get_rx_len();
    ch4_clear_frame_ready();

    return parse_frame(rx_buf, rx_len, data);
}

#endif /* LASER_CH4_ENABLED */
