/**
 * @file    ble_ota.h
 * @brief   BLE OTA 蓝牙固件升级协议定义
 * @note    适配 VG6328A-MS BLE 5.1 透传模块 + GD32L233RC
 *          协议帧: SOF(0x5A)+VER(0xA5)+CMD+LEN(小端)+PAYLOAD+CRC16(小端)
 */
#ifndef BLE_OTA_H
#define BLE_OTA_H

#include <stdint.h>
#include <string.h>

/* ==================== 协议常量 ==================== */
#define BLE_OTA_SOF             0x5A
#define BLE_OTA_VER             0xA5
#define BLE_OTA_PAYLOAD_MAX     520     /* 最大负载长度 (4+512=516 数据帧, 留余量) */
#define BLE_OTA_IDLE_TIMEOUT    30000   /* 30s 无数据超时 (ms) */
#define BLE_OTA_CONNECT_TIMEOUT 60000   /* 60s 等待蓝牙连接超时 (ms) */

/* ==================== CMD 定义 ==================== */
/* 手机 → MCU */
#define CMD_OTA_START_REQ       0x01    /* 升级请求 */
#define CMD_OTA_DATA            0x02    /* 数据帧 */
#define CMD_OTA_END             0x03    /* 传输结束 */
#define CMD_OTA_ABORT           0x04    /* 中止升级 */

/* MCU → 手机 */
#define CMD_OTA_START_ACK       0x81    /* 同意升级 */
#define CMD_OTA_START_NACK      0x82    /* 拒绝升级 */
#define CMD_DATA_ACK            0x83    /* 数据帧确认 */
#define CMD_DATA_NACK           0x84    /* 数据帧拒绝 */

/* ==================== 拒绝原因 (START_NACK) ==================== */
#define REJECT_SIZE             0x01    /* 固件尺寸非法 */
#define REJECT_VERSION          0x02    /* 版本不高于当前 */
#define REJECT_FLASH_BUSY       0x03    /* Flash状态异常 */
#define REJECT_BATTERY          0x04    /* 电池电压不足 */

/* ==================== 错误码 (DATA_NACK) ==================== */
#define ERR_SESSION             0x01    /* session_id 不匹配 */
#define ERR_SEQ_GAP             0x02    /* 序列号不连续 */
#define ERR_CRC16               0x03    /* 帧CRC校验失败 */
#define ERR_FLASH_WRITE         0x04    /* Flash写入错误 */

/* ==================== OTA 状态机 ==================== */
typedef enum {
    BLE_OTA_WAIT_CONNECT = 0, /* 等待手机连接BLE (PD2=LOW) */
    BLE_OTA_IDLE,           /* 已连接, 等待 START_REQ */
    BLE_OTA_HANDSHAKE,      /* 握手中 */
    BLE_OTA_DOWNLOADING,    /* 数据接收中 */
    BLE_OTA_COMPLETE,       /* 传输完成 */
    BLE_OTA_ERROR           /* 错误状态 */
} BleOtaState_t;

/* ==================== OTA 控制结构体 ==================== */
typedef struct {
    BleOtaState_t state;
    uint16_t session_id;
    uint32_t total_size;        /* 固件总字节数 */
    uint32_t expected_crc32;    /* 手机下发的CRC32 */
    uint32_t new_version;       /* 新固件版本号 */
    uint32_t flash_addr;        /* 当前Flash写入地址 */
    uint16_t expected_seq;      /* 期望的下一个seq */
    uint32_t received_bytes;    /* 已接收字节数 */
    uint32_t word_buf;          /* 4字节拼装缓冲 */
    uint8_t  word_pos;          /* 当前拼装位置 (0-3) */
    uint32_t last_rx_ms;        /* 最后收到数据的时间戳 (trea_millis) */
} BleOtaCtrl_t;

/* ==================== 帧解析器 ==================== */
typedef enum {
    BLE_PARSE_IDLE = 0,     /* 等待 SOF (0x5A) */
    BLE_PARSE_SOF,          /* 收到 0x5A, 等待 VER (0xA5) */
    BLE_PARSE_HEADER,       /* 接收 CMD + LEN (3字节) */
    BLE_PARSE_PAYLOAD,      /* 接收负载 */
    BLE_PARSE_CRC           /* 接收 CRC16 */
} BleParseState_t;

typedef enum {
    FRAME_OK = 0,           /* 帧完整且CRC校验通过 */
    FRAME_INCOMPLETE,       /* 帧不完整, 继续接收 */
    FRAME_CRC_ERROR         /* CRC校验失败 */
} FrameResult_t;

typedef struct {
    BleParseState_t state;
    uint8_t  cmd;
    uint16_t payload_len;
    uint16_t payload_idx;
    uint8_t  payload[BLE_OTA_PAYLOAD_MAX];
    uint16_t crc_calc;
    uint8_t  header_idx;
    uint8_t  header_buf[3];
    uint8_t  crc_idx;
    uint8_t  crc_buf[2];
} BleOtaParser_t;

/* ==================== 全局变量 ==================== */
extern BleOtaCtrl_t g_ble_ota;

/* ==================== 函数声明 ==================== */

/* 初始化 / 主流程 */
void ble_ota_init(void);
void ble_ota_process(void);

/* 帧解析 */
FrameResult_t ble_ota_parse_byte(uint8_t ch, BleOtaParser_t *p);

/* 帧发送 */
void ble_ota_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len);
void ble_ota_send_start_ack(uint16_t sid, uint32_t cur_ver, uint32_t agreed_size, uint8_t flags);
void ble_ota_send_start_nack(uint16_t sid, uint8_t reason, uint32_t cur_ver);
void ble_ota_send_data_ack(uint16_t sid, uint16_t seq);
void ble_ota_send_data_nack(uint16_t sid, uint8_t error_code);

/* CRC */
uint16_t ble_ota_crc16_update(uint16_t crc, const uint8_t *data, uint16_t len);
uint16_t ble_ota_crc16_update_byte(uint16_t crc, uint8_t byte);
uint32_t ble_ota_crc32_compute(const uint8_t *data, uint32_t len);

/* BLE AT 配置 */
void ble_set_name_from_device_id(void);

#endif /* BLE_OTA_H */
