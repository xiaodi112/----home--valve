# 蓝牙OTA升级方案完整文档

> 适用模块：VG6328A-MS 主从一体 BLE 5.1 蓝牙透传模块
> 适用 MCU：GD32L233RC (Cortex-M23, 小端序)
> 字节序：全协议小端序 (Little-Endian)
> 版本：v1.0
> 日期：2026-07-13
> 适配工程：Pressure_Regulating_Box / Household_valves_LPUART_OTA

---

## 目录

1. [方案概述](#1-方案概述)
2. [协议帧结构](#2-协议帧结构)
3. [命令码定义](#3-命令码定义)
4. [握手流程详解](#4-握手流程详解)
5. [数据传输示例（两帧不同长度）](#5-数据传输示例两帧不同长度)
6. [升级提交流程](#6-升级提交流程)
7. [帧解析器实现](#7-帧解析器实现)
8. [与现有工程适配](#8-与现有工程适配)
9. [安全保障机制](#9-安全保障机制)
10. [代码结构建议](#10-代码结构建议)
11. [时序参数](#11-时序参数)

---

## 1. 方案概述

### 1.1 设计目标

- **握手成功才升级**：三次握手确保双方就绪
- **握手失败不升级**：任意环节失败保留旧程序
- **数据流小端序**：与GD32L233硬件一致，零开销
- **固定头尾帧**：SOF(0x5A)+VER(0xA5)+CMD+LEN+PAYLOAD+CRC16
- **长度不定**：LEN字段告知负载长度，最大520字节
- **CRC校验**：帧级CRC16 + 全量CRC32双重保障

### 1.2 系统拓扑

```
  Phone App          BLE GATT       VG6328A-MS        UART 115200      GD32L233RC
 ┌──────────┐     ┌──────────┐     ┌──────────┐     ┌──────────┐     ┌──────────┐
 │  BLE     │◄───▶│  BLE     │     │  UART    │◄───▶│  USART1  │     │  OTA     │
 │ Central  │     │  Server  │────▶│  透传模式 │     │  DMA RX  │────▶│ 接收+Flash│
 └──────────┘     └──────────┘     └──────────┘     └──────────┘     └─────┬────┘
                                                                           │
                                                                   ┌───────▼──────┐
                                                                   │  Bootloader  │
                                                                   │  (完全不变)   │
                                                                   └──────────────┘
```

### 1.3 升级流程总览

```
手机发起 ──▶ 三次握手 ──▶ 数据传输(多帧) ──▶ 升级提交 ──▶ 重启跳转

              │              │              │              │
              ▼              ▼              ▼              ▼
        START_REQ      OTA_DATA(xN)      OTA_END      NVIC_SystemReset
        ↓ ACK/NACK     ↓ DATA_ACK      ↓ CRC32校验
        握手失败        ↓ 写入Flash     ↓ 写InfoBlock
        保留旧程序      ↓ seq递增       ↓ 重启
                       ↓ 超时重传       ↓ Bootloader完成升级
```

---

## 2. 协议帧结构

### 2.1 帧格式

```
 Byte:   0      1      2      3      4    5..5+LEN-1   5+LEN  5+LEN+1
       ┌──────┬──────┬──────┬──────┬────┬────────────┬──────┬──────┐
       │ SOF  │ VER  │ CMD  │LEN_L │LEN_H│  PAYLOAD   │CRC_L │CRC_H │
       │ 0x5A │ 0xA5 │ 1B   │ 小端 │小端 │ LEN字节    │ 小端 │ 小端 │
       └──────┴──────┴──────┴──────┴────┴────────────┴──────┴──────┘
          ↔同步字←  ←──帧头──→  ←──负载──→  ←CRC16→
```

**帧总长度 = 7 + LEN 字节**

### 2.2 字段说明

| 偏移 | 字段 | 长度 | 端序 | 说明 |
|------|------|------|------|------|
| 0 | SOF | 1B | — | 固定起始字节 `0x5A` |
| 1 | VER | 1B | — | 协议版本 `0xA5` |
| 2 | CMD | 1B | — | 命令码 |
| 3 | LEN_L | 1B | 小端低字节 | 负载长度低8位 |
| 4 | LEN_H | 1B | 小端高字节 | 负载长度高8位 |
| 5..5+LEN-1 | PAYLOAD | LEN | 小端 | 负载数据 |
| 5+LEN | CRC_L | 1B | 小端低字节 | CRC16低8位 |
| 5+LEN+1 | CRC_H | 1B | 小端高字节 | CRC16高8位 |

### 2.3 CRC覆盖范围

CRC16 覆盖 **CMD + LEN_L + LEN_H + PAYLOAD**（即 Byte 2 ~ 5+LEN-1）

---

## 3. 命令码定义

### 3.1 手机 → MCU

| CMD | 名称 | 触发时机 | 负载长度 | 负载格式 |
|-----|------|----------|----------|----------|
| `0x01` | OTA_START_REQ | 发起升级 | 14B | session_id(2) + total_size(4) + crc32(4) + new_version(4) |
| `0x02` | OTA_DATA | 发送数据块 | 4+N | session_id(2) + seq(2) + data(N) |
| `0x03` | OTA_END | 全部发完 | 2B | session_id(2) |
| `0x04` | OTA_ABORT | 放弃升级 | 2B | session_id(2) |

### 3.2 MCU → 手机

| CMD | 名称 | 触发时机 | 负载长度 | 负载格式 |
|-----|------|----------|----------|----------|
| `0x81` | OTA_START_ACK | 同意升级 | 11B | session_id(2) + cur_version(4) + agreed_size(4) + flags(1) |
| `0x82` | OTA_START_NACK | 拒绝升级 | 7B | session_id(2) + reject_reason(1) + cur_version(4) |
| `0x83` | DATA_ACK | 数据帧成功 | 4B | session_id(2) + seq(2) |
| `0x84` | DATA_NACK | 数据帧失败 | 3B | session_id(2) + error_code(1) |

### 3.3 拒绝原因（OTA_START_NACK）

| 值 | 名称 | 含义 | 手机应做 |
|-----|------|------|----------|
| `0x01` | REJECT_VERSION | new_version ≤ cur_version | 提示已是最新 |
| `0x02` | REJECT_SIZE | total_size > 缓存容量 | 检查固件 |
| `0x03` | REJECT_FLASH_BUSY | 上次OTA未完成 | 等待/重启 |
| `0x04` | REJECT_BATTERY | 电池过低 | 提示充电 |
| `0x05` | REJECT_BUSY | MCU正忙 | 稍后重试 |
| `0x06` | REJECT_PARAM | 参数非法 | 检查文件 |

### 3.4 错误码（DATA_NACK）

| 值 | 名称 | 含义 |
|-----|------|------|
| `0x01` | ERR_CRC16 | 帧CRC16校验失败 |
| `0x02` | ERR_FLASH_WRITE | Flash写入失败 |
| `0x03` | ERR_SESSION | session_id不匹配 |
| `0x04` | ERR_SEQ_GAP | seq不连续 |
| `0x05` | ERR_STATE | 状态机错误 |

### 3.5 flags位掩码（OTA_START_ACK）

```
Bit 0: FLASH_OK   (1=Flash正常)
Bit 1: BATTERY_OK (1=电池充足)
Bit 2: INFO_OK    (1=InfoBlock状态为IDLE)
Bit 3~7: 保留
```

---

## 4. 握手流程详解

### 4.1 场景设定

| 参数 | 值 |
|------|-----|
| session_id | 0x1234（手机随机生成） |
| 新固件版本 | 3 |
| 新固件大小 | 768 字节 |
| 新固件CRC32 | 0x12345678 |
| MCU当前版本 | 2 |

### 4.2 ① 手机 → MCU：OTA_START_REQ

**帧总长度：21 字节**

| 字节 | 数据 | 字段 | 说明 |
|------|------|------|------|
| 0 | `0x5A` | SOF | 帧起始符 |
| 1 | `0xA5` | VER | 协议版本 |
| 2 | `0x01` | CMD | OTA_START_REQ |
| 3 | `0x0E` | LEN_L | 负载长度低字节 = 14 |
| 4 | `0x00` | LEN_H | 负载长度高字节 = 0 |
| **5** | `0x34` | **session_id_L** | **小端序：0x1234** |
| **6** | `0x12` | **session_id_H** | |
| **7** | `0x00` | **total_size_0** | **小端序：768** |
| **8** | `0x00` | **total_size_1** | |
| **9** | `0x03` | **total_size_2** | |
| **10** | `0x00` | **total_size_3** | |
| **11** | `0x78` | **crc32_0** | **小端序：0x12345678** |
| **12** | `0x56` | **crc32_1** | |
| **13** | `0x34` | **crc32_2** | |
| **14** | `0x12` | **crc32_3** | |
| **15** | `0x03` | **new_version_0** | **小端序：3** |
| **16** | `0x00` | **new_version_1** | |
| **17** | `0x00` | **new_version_2** | |
| **18** | `0x00` | **new_version_3** | |
| 19 | `XX` | CRC_L | CRC16低字节 |
| 20 | `XX` | CRC_H | CRC16高字节 |

**MCU解析（小端序零开销）：**

```c
uint8_t *p = parser.payload;
uint16_t session_id  = *(uint16_t*)(&p[0]);   // p[0]=0x34, p[1]=0x12 → 0x1234
uint32_t total_size  = *(uint32_t*)(&p[2]);   // p[2]=0x00, p[3]=0x00, p[4]=0x03, p[5]=0x00 → 768
uint32_t crc32       = *(uint32_t*)(&p[6]);   // p[6]=0x78, p[7]=0x56, p[8]=0x34, p[9]=0x12 → 0x12345678
uint32_t new_version = *(uint32_t*)(&p[10]);  // p[10]=0x03, p[11]=0x00, p[12]=0x00, p[13]=0x00 → 3
```

### 4.3 MCU决策链（全部通过才升级）

```c
int ble_ota_handle_start_req(uint16_t sid, uint32_t sz, uint32_t crc, uint32_t ver) {
    // 1. 参数合法性校验
    if (sz == 0 || sz > OTA_CACHE_SIZE) {
        ble_ota_send_start_nack(sid, REJECT_SIZE, APP_FW_VERSION_NUM); return -1;
    }
    
    // 2. 版本校验：新版本必须高于当前版本
    if (ver <= APP_FW_VERSION_NUM) {
        ble_ota_send_start_nack(sid, REJECT_VERSION, APP_FW_VERSION_NUM); return -1;
    }
    
    // 3. InfoBlock状态校验：上次OTA必须已完成
    const volatile OtaInfo_t *info = OTA_INFO_PTR;
    if (info->magic == OTA_MAGIC_VALID &&
        (info->status == OTA_STATUS_PENDING || info->status == OTA_STATUS_FLASHING)) {
        ble_ota_send_start_nack(sid, REJECT_FLASH_BUSY, APP_FW_VERSION_NUM); return -1;
    }
    
    // 4. 电池电压校验
    if (adc_bat_read() < 3.1f) {
        ble_ota_send_start_nack(sid, REJECT_BATTERY, APP_FW_VERSION_NUM); return -1;
    }
    
    // 5. 擦除缓存区
    if (ota_cache_erase() != 0) {
        ble_ota_send_start_nack(sid, REJECT_FLASH_BUSY, APP_FW_VERSION_NUM); return -1;
    }
    
    // 6. 全部通过 → 发送ACK
    uint8_t flags = 0x07;  // FLASH_OK | BATTERY_OK | INFO_OK
    ble_ota_send_start_ack(sid, APP_FW_VERSION_NUM, sz, flags);
    return 0;
}
```

### 4.4 ② MCU → 手机：OTA_START_ACK

**帧总长度：18 字节**

| 字节 | 数据 | 字段 | 说明 |
|------|------|------|------|
| 0 | `0x5A` | SOF | 帧起始符 |
| 1 | `0xA5` | VER | 协议版本 |
| 2 | `0x81` | CMD | OTA_START_ACK |
| 3 | `0x0B` | LEN_L | 负载长度低字节 = 11 |
| 4 | `0x00` | LEN_H | 负载长度高字节 = 0 |
| **5** | `0x34` | **session_id_L** | **小端序：0x1234** |
| **6** | `0x12` | **session_id_H** | |
| **7** | `0x02` | **cur_version_0** | **小端序：2** |
| **8** | `0x00` | **cur_version_1** | |
| **9** | `0x00` | **cur_version_2** | |
| **10** | `0x00` | **cur_version_3** | |
| **11** | `0x00` | **agreed_size_0** | **小端序：768** |
| **12** | `0x00` | **agreed_size_1** | |
| **13** | `0x03` | **agreed_size_2** | |
| **14** | `0x00` | **agreed_size_3** | |
| **15** | `0x07` | **flags** | **0x07 = FLASH_OK\|BATTERY_OK\|INFO_OK** |
| 16 | `XX` | CRC_L | CRC16低字节 |
| 17 | `XX` | CRC_H | CRC16高字节 |

### 4.5 ③ 手机验证并开始传输

| 验证项 | 期望值 | MCU回复 | 结果 |
|--------|--------|---------|------|
| session_id | 0x1234 | 0x1234 | ✓ |
| cur_version < new_version | 2 < 3 | 2 | ✓ |
| agreed_size | 768 | 768 | ✓ |
| flags | 0x07 | 0x07 | ✓ |

**验证全部通过 → 握手成功 → 开始发送数据**

---

## 5. 数据传输示例（两帧不同长度）

### 5.1 第一帧数据（256字节）

#### 手机 → MCU：OTA_DATA seq=0

**帧总长度：267 字节**

| 字节 | 数据 | 字段 | 说明 |
|------|------|------|------|
| 0 | `0x5A` | SOF | 帧起始符 |
| 1 | `0xA5` | VER | 协议版本 |
| 2 | `0x02` | CMD | OTA_DATA |
| 3 | `0x04` | LEN_L | 负载长度低字节 = 4 |
| 4 | `0x01` | LEN_H | 负载长度高字节 = 1 |
| **5** | `0x34` | **session_id_L** | **小端序：0x1234** |
| **6** | `0x12` | **session_id_H** | |
| **7** | `0x00` | **seq_L** | **小端序：0** |
| **8** | `0x00` | **seq_H** | |
| **9~264** | `AA~BB` | **data** | **256字节固件数据** |
| 265 | `XX` | CRC_L | CRC16低字节 |
| 266 | `XX` | CRC_H | CRC16高字节 |

**LEN字段解析：**

```c
uint8_t len_l = header_buf[1];  // 0x04
uint8_t len_h = header_buf[2];  // 0x01

uint16_t payload_len = len_l | (len_h << 8);
                     = 0x04  | (0x01 << 8)
                     = 0x104
                     = 260 ✓ （负载 = sid(2B) + seq(2B) + data(256B) = 260B）
```

**MCU处理：**

```c
case OTA_DATA: {
    uint16_t rx_sid = *(uint16_t*)(&parser.payload[0]);  // 0x1234
    uint16_t rx_seq = *(uint16_t*)(&parser.payload[2]);  // 0
    
    // 校验session_id和seq
    if (rx_sid != g_ota_sid) { ble_ota_send_data_nack(g_ota_sid, ERR_SESSION); continue; }
    if (rx_seq != g_ota_seq) { ble_ota_send_data_nack(g_ota_sid, ERR_SEQ_GAP); continue; }
    
    // 提取数据
    uint8_t *data = &parser.payload[4];
    uint16_t dlen = parser.payload_len - 4;  // 260 - 4 = 256
    
    // 逐字节拼word写入Flash
    for (uint16_t i = 0; i < dlen; i++) {
        ((uint8_t*)&g_word_buf)[g_word_pos++] = data[i];
        if (g_word_pos == 4) {
            fmc_word_program(g_ota_addr, g_word_buf);
            g_ota_addr += 4;
            g_word_buf = 0xFFFFFFFFU;
            g_word_pos = 0;
        }
    }
    
    g_ota_rcvd += dlen;  // 已接收: 256 字节
    g_ota_seq++;         // 下一帧期望: seq=1
    ble_ota_send_data_ack(g_ota_sid, rx_seq);
} break;
```

**Flash写入：**

| 项目 | 值 |
|------|-----|
| 地址范围 | 0x08022000 ~ 0x08022100 |
| 写入字节数 | 256 字节 |
| fmc_word_program 次数 | 64 次 |

#### MCU → 手机：DATA_ACK seq=0

**帧总长度：11 字节**

| 字节 | 数据 | 字段 | 说明 |
|------|------|------|------|
| 0 | `0x5A` | SOF | 帧起始符 |
| 1 | `0xA5` | VER | 协议版本 |
| 2 | `0x83` | CMD | DATA_ACK |
| 3 | `0x04` | LEN_L | 负载长度低字节 = 4 |
| 4 | `0x00` | LEN_H | 负载长度高字节 = 0 |
| **5** | `0x34` | **session_id_L** | **小端序：0x1234** |
| **6** | `0x12` | **session_id_H** | |
| **7** | `0x00` | **seq_L** | **小端序：0** |
| **8** | `0x00` | **seq_H** | |
| 9 | `XX` | CRC_L | CRC16低字节 |
| 10 | `XX` | CRC_H | CRC16高字节 |

---

### 5.2 第二帧数据（512字节）

#### 手机 → MCU：OTA_DATA seq=1

**帧总长度：523 字节**

| 字节 | 数据 | 字段 | 说明 |
|------|------|------|------|
| 0 | `0x5A` | SOF | 帧起始符 |
| 1 | `0xA5` | VER | 协议版本 |
| 2 | `0x02` | CMD | OTA_DATA |
| 3 | `0x04` | LEN_L | 负载长度低字节 = 4 |
| 4 | `0x02` | LEN_H | 负载长度高字节 = 2 |
| **5** | `0x34` | **session_id_L** | **小端序：0x1234** |
| **6** | `0x12` | **session_id_H** | |
| **7** | `0x01` | **seq_L** | **小端序：1** |
| **8** | `0x00` | **seq_H** | |
| **9~518** | `FF~EE` | **data** | **512字节固件数据** |
| 519 | `XX` | CRC_L | CRC16低字节 |
| 520 | `XX` | CRC_H | CRC16高字节 |

**LEN字段解析：**

```c
uint8_t len_l = header_buf[1];  // 0x04
uint8_t len_h = header_buf[2];  // 0x02

uint16_t payload_len = len_l | (len_h << 8);
                     = 0x04  | (0x02 << 8)
                     = 0x204
                     = 516 ✓ （负载 = sid(2B) + seq(2B) + data(512B) = 516B）
```

**MCU处理：**

```c
case OTA_DATA: {
    uint16_t rx_sid = *(uint16_t*)(&parser.payload[0]);  // 0x1234
    uint16_t rx_seq = *(uint16_t*)(&parser.payload[2]);  // 1
    
    // 校验session_id和seq
    if (rx_sid != g_ota_sid) { ble_ota_send_data_nack(g_ota_sid, ERR_SESSION); continue; }
    if (rx_seq != g_ota_seq) { ble_ota_send_data_nack(g_ota_sid, ERR_SEQ_GAP); continue; }
    
    // 提取数据
    uint8_t *data = &parser.payload[4];
    uint16_t dlen = parser.payload_len - 4;  // 516 - 4 = 512
    
    // 逐字节拼word写入Flash
    for (uint16_t i = 0; i < dlen; i++) {
        ((uint8_t*)&g_word_buf)[g_word_pos++] = data[i];
        if (g_word_pos == 4) {
            fmc_word_program(g_ota_addr, g_word_buf);
            g_ota_addr += 4;
            g_word_buf = 0xFFFFFFFFU;
            g_word_pos = 0;
        }
    }
    
    g_ota_rcvd += dlen;  // 已接收: 256 + 512 = 768 字节（全部完成！）
    g_ota_seq++;         // 下一帧期望: seq=2
    ble_ota_send_data_ack(g_ota_sid, rx_seq);
} break;
```

**Flash写入：**

| 项目 | 值 |
|------|-----|
| 地址范围 | 0x08022100 ~ 0x08022300 |
| 写入字节数 | 512 字节 |
| fmc_word_program 次数 | 128 次 |

#### MCU → 手机：DATA_ACK seq=1

**帧总长度：11 字节**

| 字节 | 数据 | 字段 | 说明 |
|------|------|------|------|
| 0 | `0x5A` | SOF | 帧起始符 |
| 1 | `0xA5` | VER | 协议版本 |
| 2 | `0x83` | CMD | DATA_ACK |
| 3 | `0x04` | LEN_L | 负载长度低字节 = 4 |
| 4 | `0x00` | LEN_H | 负载长度高字节 = 0 |
| **5** | `0x34` | **session_id_L** | **小端序：0x1234** |
| **6** | `0x12` | **session_id_H** | |
| **7** | `0x01` | **seq_L** | **小端序：1** |
| **8** | `0x00` | **seq_H** | |
| 9 | `XX` | CRC_L | CRC16低字节 |
| 10 | `XX` | CRC_H | CRC16高字节 |

---

### 5.3 两帧数据对比

| 维度 | 第一帧（seq=0） | 第二帧（seq=1） |
|------|----------------|----------------|
| 数据长度 | 256 字节 | 512 字节 |
| 负载长度 | 260 字节 | 516 字节 |
| LEN_L | `0x04` | `0x04` |
| LEN_H | `0x01` | `0x02` |
| LEN（小端） | `0x0104` = 260 | `0x0204` = 516 |
| 帧总长度 | 267 字节 | 523 字节 |
| seq值 | 0 | 1 |
| Flash写入次数 | 64 次 | 128 次 |
| Flash地址范围 | 0x08022000 ~ 0x08022100 | 0x08022100 ~ 0x08022300 |
| 累计接收 | 256 字节 | 768 字节（完成） |

---

## 6. 升级提交流程

### 6.1 手机 → MCU：OTA_END

**帧总长度：9 字节**

| 字节 | 数据 | 字段 | 说明 |
|------|------|------|------|
| 0 | `0x5A` | SOF | 帧起始符 |
| 1 | `0xA5` | VER | 协议版本 |
| 2 | `0x03` | CMD | OTA_END |
| 3 | `0x02` | LEN_L | 负载长度低字节 = 2 |
| 4 | `0x00` | LEN_H | 负载长度高字节 = 0 |
| **5** | `0x34` | **session_id_L** | **小端序：0x1234** |
| **6** | `0x12` | **session_id_H** | |
| 7 | `XX` | CRC_L | CRC16低字节 |
| 8 | `XX` | CRC_H | CRC16高字节 |

### 6.2 MCU处理

```c
case OTA_END: {
    uint16_t rx_sid = *(uint16_t*)(&parser.payload[0]);
    if (rx_sid != g_ota_sid) continue;
    
    // 1. 补齐最后一个word（不足4字节用0xFF填充）
    if (g_word_pos != 0) {
        while (g_word_pos < 4)
            ((uint8_t*)&g_word_buf)[g_word_pos++] = 0xFF;
        fmc_word_program(g_ota_addr, g_word_buf);
    }
    
    // 2. CRC32全量校验（关键！）
    uint32_t calc_crc = crc32_compute((uint8_t*)OTA_CACHE_ADDR, g_ota_size);
    if (calc_crc != g_ota_crc32) {
        ble_ota_send_data_nack(g_ota_sid, ERR_CRC16);  // 校验失败！不升级
        return -1;  // 继续运行旧程序
    }
    
    // 3. 校验成功！写InfoBlock
    OtaInfo_t info;
    memset(&info, 0xFF, sizeof(info));
    info.magic          = OTA_MAGIC_VALID;
    info.status         = OTA_STATUS_PENDING;
    info.new_fw_size    = g_ota_size;
    info.new_fw_version = g_ota_new_ver;
    info.retry_count    = 0;
    ota_info_write(&info);
    
    // 4. 发送ACK并重启
    ble_ota_send_data_ack(g_ota_sid, 0xFFFF);  // seq=0xFFFF表示END确认
    trea_delay_ms(100);
    NVIC_SystemReset();  // 进入Bootloader完成升级
} break;
```

### 6.3 MCU → 手机：DATA_ACK（END确认）

**帧总长度：11 字节**

| 字节 | 数据 | 字段 | 说明 |
|------|------|------|------|
| 0 | `0x5A` | SOF | 帧起始符 |
| 1 | `0xA5` | VER | 协议版本 |
| 2 | `0x83` | CMD | DATA_ACK |
| 3 | `0x04` | LEN_L | 负载长度低字节 = 4 |
| 4 | `0x00` | LEN_H | 负载长度高字节 = 0 |
| **5** | `0x34` | **session_id_L** | **小端序：0x1234** |
| **6** | `0x12` | **session_id_H** | |
| **7** | `0xFF` | **seq_L** | **小端序：0xFFFF（END确认）** |
| **8** | `0xFF` | **seq_H** | |
| 9 | `XX` | CRC_L | CRC16低字节 |
| 10 | `XX` | CRC_H | CRC16高字节 |

### 6.4 升级成功后的Bootloader流程

```
Bootloader启动:
  ┌─ 读InfoBlock
  │   ├─ magic != VALID → 直接跳App（正常启动）
  │   └─ magic == VALID → 继续
  │
  ├─ status == PENDING
  │   ├─ 擦除App区 (0x08005000 ~ 0x08021FFF)
  │   ├─ 拷贝缓存区 → App区 (0x08022000 → 0x08005000)
  │   ├─ 写InfoBlock(status=SUCCESS, cur_version=new_version)
  │   └─ 跳转App
  │
  └─ status == FLASHING（断电恢复）
      └─ 继续未完成的拷贝
```

---

## 7. 帧解析器实现

### 7.1 解析器状态机

```c
typedef enum {
    BLE_PARSE_IDLE,       // 空闲，等待SOF(0x5A)
    BLE_PARSE_SOF,        // 收到0x5A，等待VER(0xA5)
    BLE_PARSE_HEADER,     // 收到0xA5，接收CMD+LEN(3字节)
    BLE_PARSE_PAYLOAD,    // 接收负载数据
    BLE_PARSE_CRC         // 接收CRC16
} BleParseState_t;

typedef enum {
    FRAME_OK=0,           // 帧完整且CRC校验通过
    FRAME_INCOMPLETE,     // 帧不完整，继续接收
    FRAME_CRC_ERROR       // CRC校验失败
} FrameResult_t;

typedef struct {
    BleParseState_t state;
    uint8_t cmd;
    uint16_t payload_len;
    uint16_t payload_idx;
    uint8_t payload[BLE_OTA_PAYLOAD_MAX];
    uint16_t crc_calc;
    uint8_t header_idx;
    uint8_t header_buf[3];
    uint8_t crc_idx;
    uint8_t crc_buf[2];
} BleOtaParser_t;
```

### 7.2 逐字节解析函数

```c
FrameResult_t ble_ota_parse_byte(uint8_t ch, BleOtaParser_t *p) {
    switch (p->state) {
    case BLE_PARSE_IDLE:
        if (ch == 0x5A) p->state = BLE_PARSE_SOF;
        break;
    
    case BLE_PARSE_SOF:
        if (ch == 0xA5) {
            p->state = BLE_PARSE_HEADER;
            p->header_idx = 0;
            p->crc_calc = 0xFFFF;
        } else if (ch != 0x5A) {
            p->state = BLE_PARSE_IDLE;
        }
        break;
    
    case BLE_PARSE_HEADER:
        p->header_buf[p->header_idx++] = ch;
        if (p->header_idx == 3) {
            p->cmd = p->header_buf[0];
            p->payload_len = p->header_buf[1] | (p->header_buf[2] << 8);
            
            if (p->payload_len > BLE_OTA_PAYLOAD_MAX) {
                p->state = BLE_PARSE_IDLE;
                return FRAME_CRC_ERROR;
            }
            
            p->crc_calc = crc16_update(p->crc_calc, p->header_buf, 3);
            p->payload_idx = 0;
            p->state = (p->payload_len > 0) ? BLE_PARSE_PAYLOAD : BLE_PARSE_CRC;
        }
        break;
    
    case BLE_PARSE_PAYLOAD:
        p->payload[p->payload_idx++] = ch;
        p->crc_calc = crc16_update_byte(p->crc_calc, ch);
        if (p->payload_idx >= p->payload_len) {
            p->state = BLE_PARSE_CRC;
            p->crc_idx = 0;
        }
        break;
    
    case BLE_PARSE_CRC:
        p->crc_buf[p->crc_idx++] = ch;
        if (p->crc_idx == 2) {
            uint16_t crc_rcvd = p->crc_buf[0] | (p->crc_buf[1] << 8);
            p->state = BLE_PARSE_IDLE;
            return (crc_rcvd == p->crc_calc) ? FRAME_OK : FRAME_CRC_ERROR;
        }
        break;
    }
    return FRAME_INCOMPLETE;
}
```

### 7.3 帧发送函数

```c
void ble_ota_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len) {
    uint16_t crc = 0xFFFF;
    crc = crc16_update_byte(crc, cmd);
    crc = crc16_update_byte(crc, (uint8_t)(len & 0xFF));
    crc = crc16_update_byte(crc, (uint8_t)((len >> 8) & 0xFF));
    
    serial_write_byte(BLE_UART, 0x5A);
    serial_write_byte(BLE_UART, 0xA5);
    serial_write_byte(BLE_UART, cmd);
    serial_write_byte(BLE_UART, (uint8_t)(len & 0xFF));
    serial_write_byte(BLE_UART, (uint8_t)((len >> 8) & 0xFF));
    
    for (uint16_t i = 0; i < len; i++) {
        serial_write_byte(BLE_UART, payload[i]);
        crc = crc16_update_byte(crc, payload[i]);
    }
    
    serial_write_byte(BLE_UART, (uint8_t)(crc & 0xFF));
    serial_write_byte(BLE_UART, (uint8_t)((crc >> 8) & 0xFF));
}
```

---

## 8. 与现有工程适配

### 8.1 Flash分区（复用现有定义）

| 分区 | 地址 | 大小 | 用途 |
|------|------|------|------|
| Bootloader | 0x08000000 | 16 KB | 启动引导 |
| InfoBlock | 0x08004000 | 4 KB | OTA状态管理 |
| App运行区 | 0x08005000 | 116 KB | 当前固件 |
| 下载缓存区 | 0x08022000 | 120 KB | 新固件缓存 |

**复用文件：** [ota_info.h](file:///f:/GD32_library/t-y-x/3-now_one%20_ok%20_onchang/Pressure_Regulating_Box/Projects/Wakeup_box_V0.0.1/MDK-ARM/my_Library/ota_info.h)

### 8.2 InfoBlock状态（复用现有定义）

| 状态码 | 名称 | 含义 |
|--------|------|------|
| 0xFFFFFFFF | IDLE | 空闲 |
| 0x55AA55AA | PENDING | 待刷写 |
| 0xA5A5A5A5 | FLASHING | 刷写中 |
| 0x5A5A5A5A | SUCCESS | 升级成功 |
| 0xDEAD0001 | FAILED | 升级失败 |

### 8.3 复用的现有函数

| 函数 | 来源文件 | 用途 |
|------|----------|------|
| `ota_info_write()` | ota_app.c | 写入InfoBlock |
| `ota_info_clear()` | ota_app.c | 清除InfoBlock |
| `ota_cache_erase()` | ota_app.c | 擦除缓存区 |
| `ota_wdt_enable()` | ota_app.c | 启用独立看门狗 |
| `ota_wdt_feed()` | ota_app.c | 喂狗 |
| `OTA_INFO_PTR` | ota_info.h | 静态访问InfoBlock |

### 8.4 新增函数

| 函数 | 用途 |
|------|------|
| `ble_ota_download()` | BLE OTA主循环 |
| `ble_ota_handle_start_req()` | 处理升级请求 |
| `ble_ota_send_start_ack()` | 发送同意应答 |
| `ble_ota_send_start_nack()` | 发送拒绝应答 |
| `ble_ota_send_data_ack()` | 发送数据帧确认 |
| `ble_ota_send_data_nack()` | 发送数据帧拒绝 |
| `ble_ota_parse_byte()` | 逐字节解析帧 |
| `ble_ota_send_frame()` | 发送帧 |

---

## 9. 安全保障机制

### 9.1 双重校验

| 校验层级 | 算法 | 覆盖范围 | 作用 |
|----------|------|----------|------|
| 帧级校验 | CRC16/CCITT | CMD + LEN + PAYLOAD | 防止单帧传输错误 |
| 全量校验 | CRC32/MPEG-2 | 整个固件 | 防止累积错误和篡改 |

### 9.2 版本保护

```c
// 新版本必须严格高于当前版本
if (new_version <= APP_FW_VERSION_NUM) {
    ble_ota_send_start_nack(sid, REJECT_VERSION, APP_FW_VERSION_NUM);
    return -1;  // 不升级，保留旧程序
}
```

### 9.3 Flash状态保护

```c
// 上次OTA必须已完成
if (info->status == OTA_STATUS_PENDING || info->status == OTA_STATUS_FLASHING) {
    ble_ota_send_start_nack(sid, REJECT_FLASH_BUSY, APP_FW_VERSION_NUM);
    return -1;  // 不升级，等待Bootloader处理
}
```

### 9.4 电池保护

```c
// 电池电压必须足够
if (adc_bat_read() < 3.1f) {
    ble_ota_send_start_nack(sid, REJECT_BATTERY, APP_FW_VERSION_NUM);
    return -1;  // 不升级，防止中途掉电
}
```

### 9.5 看门狗保护

```c
void ble_ota_download(void) {
    ota_wdt_enable();  // 启用独立看门狗（~26秒超时）
    
    while (1) {
        ota_wdt_feed();  // 每帧处理前喂狗
        // ... 帧处理 ...
    }
}
```

### 9.6 session_id防串扰

```c
// 每帧都校验session_id
if (rx_sid != g_ota_sid) {
    ble_ota_send_data_nack(g_ota_sid, ERR_SESSION);
    continue;  // 丢弃不属于当前会话的帧
}
```

---

## 10. 代码结构建议

```
my_Library/
├── ble_ota.c          # BLE OTA主流程（新增）
├── ble_ota.h          # BLE OTA头文件（新增）
├── ble_ota_parser.c   # 帧解析器（新增）
├── ble_ota_parser.h   # 帧解析器头文件（新增）
├── crc.c              # CRC16/CRC32实现（新增或复用）
├── crc.h              # CRC头文件（新增）
├── ota_app.c          # 现有4G OTA逻辑（不变）
├── ota_app.h          # 现有OTA头文件（不变）
├── ota_info.h         # Flash分区和状态定义（不变）
└── usart.c            # 串口驱动（需添加蓝牙UART支持）
```

### 主循环集成

```c
// 在 main.c 的状态机中添加
#define STATE_BLE_OTA   10

// 在事件分发中添加
case STATE_BLE_OTA:
    ble_ota_download();
    // 升级成功会在OTA_END中NVIC_SystemReset()
    // 升级失败则返回，继续运行旧程序
    app_state = STATE_IDLE;
    break;
```

---

## 11. 时序参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 握手ACK等待 | 5000 ms | 手机发START_REQ后等ACK/NACK |
| DATA帧ACK等待 | 2000 ms | 手机发DATA后等ACK |
| END ACK等待 | 5000 ms | 含CRC32 + InfoBlock写入 |
| MCU空闲超时 | 30000 ms | 连续无数据视为中断 |
| 最大重试 | 3 次 | 超过后发ABORT |
| 推荐数据块 | 512 字节 | 8帧 = 1 Flash页 |
| 最大负载长度 | 520 字节 | 防止缓冲区溢出 |

---

## 12. 完整时序图

```
手机                                              MCU
  │                                                 │
  │  ① OTA_START_REQ (sid=0x1234, ver=3, 768B)     │
  │ ──────────────────────────────────────────────► │ 决策链: ✓ ✓ ✓ ✓ ✓
  │                                                 │ 擦除缓存区
  │  ② OTA_START_ACK (cur=2, agreed=768, flags=7)  │
  │ ◄────────────────────────────────────────────── │
  │                                                 │
  │  握手成功！开始发送数据                          │
  │                                                 │
  │  ③ OTA_DATA (seq=0, 256B)                       │
  │ ──────────────────────────────────────────────► │ LEN=0x104=260
  │                                                 │ 写入Flash: 0x08022000~0x08022100
  │  ④ DATA_ACK (seq=0)                             │
  │ ◄────────────────────────────────────────────── │
  │                                                 │
  │  ⑤ OTA_DATA (seq=1, 512B)                       │
  │ ──────────────────────────────────────────────► │ LEN=0x204=516
  │                                                 │ 写入Flash: 0x08022100~0x08022300
  │  ⑥ DATA_ACK (seq=1)                             │
  │ ◄────────────────────────────────────────────── │
  │                                                 │
  │  全部768字节发送完成！                           │
  │                                                 │
  │  ⑦ OTA_END                                      │
  │ ──────────────────────────────────────────────► │ CRC32校验: 0x12345678 ✓
  │                                                 │ 写InfoBlock(status=PENDING)
  │  ⑧ DATA_ACK (seq=0xFFFF)                        │
  │ ◄────────────────────────────────────────────── │
  │                                                 │ NVIC_SystemReset()
  │                                                 │ Bootloader: 读InfoBlock → 擦App → 拷贝 → 跳转
  │                                                 │ 运行新固件 v3
```

---

## 附录：CRC算法实现

### CRC16/CCITT

```c
uint16_t crc16_update(uint16_t crc, const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)(data[i] << 8);
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
    }
    return crc;
}

uint16_t crc16_update_byte(uint16_t crc, uint8_t byte) {
    crc ^= (uint16_t)(byte << 8);
    for (uint8_t j = 0; j < 8; j++)
        crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
    return crc;
}
```

### CRC32/MPEG-2

```c
uint32_t crc32_compute(const uint8_t *data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++)
        crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ data[i]) & 0xFF];
    return crc;
}
```

---

> 文档结束