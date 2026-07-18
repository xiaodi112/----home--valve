# BLE 透传 OTA 固件升级协议规范

> 适用模块：VG6328A-MS 主从一体 BLE 5.1 蓝牙透传模块
> 适用 MCU：GD32L233RC (Cortex-M23, 小端序)
> 字节序：全协议小端序 (Little-Endian)
> 版本：v3.0  (增加双向握手)
> 日期：2026-06-17

---

## 1. 设计总览

### 1.1 为什么选小端序

| # | 原因 | 说明 |
|---|---|---|
| 1 | MCU 原生小端 | GD32L233 (Cortex-M23) 内存按小端存放，协议与硬件一致则零开销 |
| 2 | Flash 写入零开销 | .bin 文件的字节顺序 = ARM 内存中的样子 = Flash 中的样子，全程不翻转 |
| 3 | 代码极简 | `*(uint16_t*)ptr` 直接读出，无需移位拼接 |
| 4 | 与现有代码一致 | 当前 `ec800e.c` 逐字节拼 word 就是小端模式，可完全复用 |

### 1.2 双向握手设计

4G HTTP OTA 是单向的：服务器有固件就发，MCU 被动接收。BLE OTA 不同——手机和 MCU 是直连的，双方可以对等协商。

本协议采用三次握手：

```
① 手机 ──▶ MCU    "我要升级，版本2，大小110592，CRC32=xxx"
② MCU  ──▶ 手机   "同意！我当前版本1，接收110592字节"  或  "拒绝！原因：版本已是最新"
③ 手机 ──▶ MCU   发送第一帧数据（隐含确认：我收到了你的应答）
```

握手确保：
- MCU 有能力接收（Flash 正常、缓存够、电量足）
- MCU 有必要升级（new_version > cur_version）
- 手机确认 MCU 已就绪才开始传输
- session_id 防串扰（BLE 重连后可区分新旧会话）

### 1.3 系统拓扑

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

---

## 2. 帧协议规范

### 2.1 帧结构

```
 Byte:  0       1       2       3     4        5  ...  5+LEN-1   5+LEN  5+LEN+1
      ┌───────┬───────┬───────┬───────┬──────┬──────────────┬───────┬───────┐
      │  SOF  │  VER  │  CMD  │  LEN (uint16 LE) │   PAYLOAD    │ CRC16 (LE) │
      │ 0x5A  │ 0xA5  │  1B   │  LB    HB  │    LEN bytes   │  LB    HB  │
      └───────┴───────┴───────┴───────┴──────┴──────────────┴───────┴───────┘
        └ sync word ┘ └─ frame header ─┘ └── payload ──┘ └─ CRC16 ─┘
```

帧总长度 = LEN + 7 字节。LEN 为小端序 uint16，低字节在前（Byte 3），高字节在后（Byte 4）。
CRC16 覆盖范围：CMD + LEN_L + LEN_H + PAYLOAD（即 Byte 2 ~ 5+LEN-1）。

| 偏移 | 字段 | 长度 | 端序 | 说明 |
|---|---|---|---|---|
| 0 | SOF | 1 | — | 固定 `0x5A` |
| 1 | VER | 1 | — | 协议版本 `0xA5` |
| 2 | CMD | 1 | — | 命令码 |
| 3 | LEN_L | 1 | 小端低字节 | 负载长度低 8 位 |
| 4 | LEN_H | 1 | 小端高字节 | 负载长度高 8 位 |
| 5..5+LEN-1 | PAYLOAD | LEN | 小端 | 负载数据 |
| 5+LEN | CRC_L | 1 | 小端低字节 | CRC16 低 8 位 |
| 5+LEN+1 | CRC_H | 1 | 小端高字节 | CRC16 高 8 位 |

### 2.2 命令码

#### 手机 → MCU

| CMD | 名称 | 触发时机 | 负载 |
|---|---|---|---|
| `0x01` | OTA_START_REQ | 发起升级 | session_id(2B) + total_size(4B) + crc32(4B) + new_version(4B) = 14B |
| `0x02` | OTA_DATA | 发送数据块 | session_id(2B) + seq(2B) + chunk(N) |
| `0x03` | OTA_END | 全部发完 | session_id(2B) |
| `0x04` | OTA_ABORT | 放弃升级 | session_id(2B) |

#### MCU → 手机

| CMD | 名称 | 触发时机 | 负载 |
|---|---|---|---|
| `0x81` | OTA_START_ACK | 同意升级 | session_id(2B) + cur_version(4B) + agreed_size(4B) + flags(1B) = 11B |
| `0x82` | OTA_START_NACK | 拒绝升级 | session_id(2B) + reject_reason(1B) + cur_version(4B) = 7B |
| `0x83` | DATA_ACK | 数据帧成功 | session_id(2B) + seq(2B) = 4B |
| `0x84` | DATA_NACK | 数据帧失败 | session_id(2B) + error_code(1B) = 3B |

### 2.3 拒绝原因（OTA_START_NACK）

| 值 | 名称 | 含义 | 手机应做 |
|---|---|---|---|
| `0x01` | REJECT_VERSION | new_version ≤ cur_version | 提示已是最新 |
| `0x02` | REJECT_SIZE | total_size > 缓存容量 | 检查固件 |
| `0x03` | REJECT_FLASH_BUSY | 上次 OTA 未完成 | 等待/重启 |
| `0x04` | REJECT_BATTERY | 电池过低 | 提示充电 |
| `0x05` | REJECT_BUSY | MCU 正忙 | 稍后重试 |
| `0x06` | REJECT_PARAM | 参数非法 | 检查文件 |

### 2.4 错误码（DATA_NACK）

| 值 | 名称 | 含义 |
|---|---|---|
| `0x01` | ERR_CRC16 | 帧 CRC16 校验失败 |
| `0x02` | ERR_FLASH_WRITE | Flash 写入失败 |
| `0x03` | ERR_SESSION | session_id 不匹配 |
| `0x04` | ERR_SEQ_GAP | seq 不连续 |
| `0x05` | ERR_STATE | 状态机错误 |

### 2.5 flags 位掩码（OTA_START_ACK）

```
Bit 0: FLASH_OK   (1=Flash 正常)
Bit 1: BATTERY_OK (1=电池充足)
Bit 2: INFO_OK    (1=InfoBlock 状态为 IDLE)
Bit 3~7: 保留
```
---

## 3. 各帧详细说明

### 3.1 OTA_START_REQ — 升级请求（手机 → MCU）

负载 14 字节（小端序）：

| 偏移 | 0-1 | 2-5 | 6-9 | 10-13 |
|---|---|---|---|---|
| 字段 | session_id | total_size | crc32 | new_version |
| 类型 | uint16 LE | uint32 LE | uint32 LE | uint32 LE |

**手机端组帧（Kotlin）：**

\\\kotlin
fun buildStartReq(sid: Int, size: Int, crc: Int, ver: Int): ByteArray {
    val p = ByteArray(14)
    p[0] = (sid and 0xFF).toByte(); p[1] = ((sid shr 8) and 0xFF).toByte()
    p[2] = (size and 0xFF).toByte(); p[3] = ((size shr 8) and 0xFF).toByte()
    p[4] = ((size shr 16) and 0xFF).toByte(); p[5] = ((size shr 24) and 0xFF).toByte()
    p[6] = (crc and 0xFF).toByte(); p[7] = ((crc shr 8) and 0xFF).toByte()
    p[8] = ((crc shr 16) and 0xFF).toByte(); p[9] = ((crc shr 24) and 0xFF).toByte()
    p[10] = (ver and 0xFF).toByte(); p[11] = ((ver shr 8) and 0xFF).toByte()
    p[12] = ((ver shr 16) and 0xFF).toByte(); p[13] = ((ver shr 24) and 0xFF).toByte()
    return BleOtaProtocol.buildFrame(0x01.toByte(), p)
}
\\\

**MCU 端解帧（C）：**

\\\c
uint16_t session_id  = *(uint16_t *)(&parser.payload[0]);  // 小端零开销
uint32_t total_size  = *(uint32_t *)(&parser.payload[2]);
uint32_t crc32       = *(uint32_t *)(&parser.payload[6]);
uint32_t new_version = *(uint32_t *)(&parser.payload[10]);
\\\

---

### 3.2 OTA_START_ACK — 同意升级（MCU → 手机）

负载 11 字节（小端序）：

| 偏移 | 0-1 | 2-5 | 6-9 | 10 |
|---|---|---|---|---|
| 字段 | session_id | cur_version | agreed_size | flags |
| 类型 | uint16 LE | uint32 LE | uint32 LE | uint8 |

session_id 回显手机发来的值，cur_version 是 MCU 当前运行的固件版本，手机应验证 cur_version < new_version。

**MCU 端发送（C）：**

\\\c
void ble_ota_send_start_ack(uint16_t sid, uint32_t cur_ver,
                            uint32_t agreed, uint8_t flags) {
    uint8_t p[11];
    p[0] = (uint8_t)(sid & 0xFF);
    p[1] = (uint8_t)((sid >> 8) & 0xFF);
    p[2] = (uint8_t)(cur_ver & 0xFF);
    p[3] = (uint8_t)((cur_ver >> 8) & 0xFF);
    p[4] = (uint8_t)((cur_ver >> 16) & 0xFF);
    p[5] = (uint8_t)((cur_ver >> 24) & 0xFF);
    p[6] = (uint8_t)(agreed & 0xFF);
    p[7] = (uint8_t)((agreed >> 8) & 0xFF);
    p[8] = (uint8_t)((agreed >> 16) & 0xFF);
    p[9] = (uint8_t)((agreed >> 24) & 0xFF);
    p[10] = flags;
    ble_ota_send_frame(0x81, p, 11);
}
\\\

**手机端验证 ACK（Kotlin）：**

\\\kotlin
data class StartAck(val sessionId: Int, val curVersion: Int,
                    val agreedSize: Int, val flags: Int)

fun verifyHandshake(mySid: Int, mySize: Int, myVer: Int, ack: StartAck): Result<Unit> {
    if (ack.sessionId != mySid)
        return Result.failure(Exception("Session ID mismatch"))
    if (ack.curVersion >= myVer)
        return Result.failure(Exception("MCU already at v"))
    if (ack.agreedSize != mySize)
        return Result.failure(Exception("Size mismatch"))
    if ((ack.flags and 0x01) == 0)
        return Result.failure(Exception("MCU Flash not ready"))
    return Result.success(Unit)
}
\\\

---

### 3.3 OTA_START_NACK — 拒绝升级（MCU → 手机）

负载 7 字节：

| 偏移 | 0-1 | 2 | 3-6 |
|---|---|---|---|
| 字段 | session_id | reject_reason | cur_version |
| 类型 | uint16 LE | uint8 | uint32 LE |

**MCU 端完整决策链（C）：**

\\\c
int ble_ota_handle_start_req(uint16_t sid, uint32_t sz,
                              uint32_t crc, uint32_t ver) {
    // (1) 参数合法性
    if (sz == 0 || sz > OTA_CACHE_SIZE) {
        ble_ota_send_start_nack(sid, REJECT_SIZE, CUR_VER); return -1;
    }
    if (crc == 0) {
        ble_ota_send_start_nack(sid, REJECT_PARAM, CUR_VER); return -1;
    }
    // (2) 版本检查
    if (ver <= CUR_VER) {
        ble_ota_send_start_nack(sid, REJECT_VERSION, CUR_VER); return -1;
    }
    // (3) InfoBlock 状态 (上次OTA未完成?)
    const volatile OtaInfo_t *info = OTA_INFO_PTR;
    if (info->magic == OTA_MAGIC_VALID &&
        (info->status == OTA_STATUS_PENDING ||
         info->status == OTA_STATUS_FLASHING)) {
        ble_ota_send_start_nack(sid, REJECT_FLASH_BUSY, CUR_VER); return -1;
    }
    // (4) 电池检查
    float batt = adc_bat_read();
    if (batt < 3.1f) {
        ble_ota_send_start_nack(sid, REJECT_BATTERY, CUR_VER); return -1;
    }
    uint8_t flags = 0x04;  // INFO_OK
    if (batt >= 3.5f) flags |= 0x02;  // BATTERY_OK
    flags |= 0x01;  // FLASH_OK

    // (5) 擦除缓存区
    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
    uint32_t pages = (sz + OTA_FLASH_PAGE_SIZE - 1) / OTA_FLASH_PAGE_SIZE;
    for (uint32_t i = 0, addr = OTA_CACHE_ADDR; i < pages; i++, addr += 4096) {
        if (fmc_page_erase(addr) != FMC_READY) {
            fmc_lock();
            ble_ota_send_start_nack(sid, REJECT_FLASH_BUSY, CUR_VER);
            return -1;
        }
    }
    fmc_lock();

    // (6) 全部通过 → ACK
    ble_ota_send_start_ack(sid, CUR_VER, sz, flags);
    return 0;
}
\\\

---

### 3.4 OTA_DATA — 数据帧（手机 → MCU）

| 偏移 | 0-1 | 2-3 | 4..N+3 |
|---|---|---|---|
| 字段 | session_id | seq | data |
| 类型 | uint16 LE | uint16 LE | 原始字节 |

推荐 data 大小 512 字节，此时 LEN = 4 + 512 = 516 = 0x0204。

\\\kotlin
fun buildDataFrame(sid: Int, seq: Int, chunk: ByteArray): ByteArray {
    val p = ByteArray(4 + chunk.size)
    p[0] = (sid and 0xFF).toByte(); p[1] = ((sid shr 8) and 0xFF).toByte()
    p[2] = (seq and 0xFF).toByte(); p[3] = ((seq shr 8) and 0xFF).toByte()
    System.arraycopy(chunk, 0, p, 4, chunk.size)
    return BleOtaProtocol.buildFrame(0x02.toByte(), p)
}
\\\

MCU 端需校验 session_id 和 seq 后再写入 Flash：

\\\c
uint16_t rx_sid = *(uint16_t *)(&parser.payload[0]);
uint16_t rx_seq = *(uint16_t *)(&parser.payload[2]);
if (rx_sid != g_ota_session_id) {
    ble_ota_send_data_nack(g_ota_session_id, ERR_SESSION); return;
}
if (rx_seq != g_ota_expected_seq) {
    ble_ota_send_data_nack(g_ota_session_id, ERR_SEQ_GAP); return;
}
// 逐字节拼 word → Flash ...
g_ota_expected_seq++;
ble_ota_send_data_ack(g_ota_session_id, rx_seq);
\\\

---

### 3.5 DATA_ACK / DATA_NACK（MCU → 手机）

DATA_ACK 负载 = session_id(2B) + seq(2B) = 4B
DATA_NACK 负载 = session_id(2B) + error_code(1B) = 3B

### 3.6 OTA_END / OTA_ABORT（手机 → MCU）

负载 = session_id(2B)。MCU 收到 OTA_END 后：CRC32 全量校验 → 写 InfoBlock → 回复 DATA_ACK(sid, seq=0xFFFF) → 重启。


---

## 4. 组帧 / 解帧通用函数

### 4.1 手机端（Kotlin）

\\\kotlin
object BleOtaProtocol {
    const val SOF = 0x5A.toByte(); const val VER = 0xA5.toByte()
    const val CMD_START_REQ = 0x01.toByte(); const val CMD_DATA   = 0x02.toByte()
    const val CMD_END       = 0x03.toByte(); const val CMD_ABORT  = 0x04.toByte()
    const val CMD_START_ACK = 0x81.toByte(); const val CMD_START_NACK = 0x82.toByte()
    const val CMD_DATA_ACK  = 0x83.toByte(); const val CMD_DATA_NACK  = 0x84.toByte()

    fun buildFrame(cmd: Byte, payload: ByteArray): ByteArray {
        val len = payload.size
        val frame = ByteArray(5 + len + 2)
        frame[0] = SOF; frame[1] = VER; frame[2] = cmd
        frame[3] = (len and 0xFF).toByte()
        frame[4] = ((len shr 8) and 0xFF).toByte()
        System.arraycopy(payload, 0, frame, 5, len)
        // CRC over CMD + LEN + PAYLOAD
        val crcData = ByteArray(3 + len)
        crcData[0] = cmd; crcData[1] = frame[3]; crcData[2] = frame[4]
        System.arraycopy(payload, 0, crcData, 3, len)
        val crc = crc16(crcData)
        frame[5 + len] = (crc and 0xFF).toByte()
        frame[5 + len + 1] = ((crc shr 8) and 0xFF).toByte()
        return frame
    }
}
\\\

### 4.2 MCU 端帧解析器（C）

\\\c
#define BLE_OTA_PAYLOAD_MAX  520
typedef enum { BLE_PARSE_IDLE, BLE_PARSE_SOF, BLE_PARSE_HEADER,
               BLE_PARSE_PAYLOAD, BLE_PARSE_CRC } BleParseState_t;
typedef enum { FRAME_OK=0, FRAME_INCOMPLETE, FRAME_CRC_ERROR } FrameResult_t;

typedef struct {
    BleParseState_t state;  uint8_t cmd;  uint16_t payload_len;
    uint16_t payload_idx;   uint8_t payload[BLE_OTA_PAYLOAD_MAX];
    uint16_t crc_calc;      uint8_t header_idx;
    uint8_t header_buf[3];  uint8_t crc_idx;  uint8_t crc_buf[2];
} BleOtaParser_t;

FrameResult_t ble_ota_parse_byte(uint8_t ch, BleOtaParser_t *p) {
    switch (p->state) {
    case BLE_PARSE_IDLE:
        if (ch == 0x5A) p->state = BLE_PARSE_SOF; break;
    case BLE_PARSE_SOF:
        if (ch == 0xA5) {
            p->state = BLE_PARSE_HEADER; p->header_idx = 0; p->crc_calc = 0xFFFF;
        } else if (ch != 0x5A) p->state = BLE_PARSE_IDLE;
        break;
    case BLE_PARSE_HEADER:
        p->header_buf[p->header_idx++] = ch;
        if (p->header_idx == 3) {
            p->cmd = p->header_buf[0];
            p->payload_len = p->header_buf[1] | (p->header_buf[2] << 8);
            if (p->payload_len > BLE_OTA_PAYLOAD_MAX) {
                p->state = BLE_PARSE_IDLE; return FRAME_CRC_ERROR;
            }
            p->crc_calc = crc16_update(p->crc_calc, p->header_buf, 3);
            p->payload_idx = 0;
            p->state = (p->payload_len > 0) ? BLE_PARSE_PAYLOAD : BLE_PARSE_CRC;
        } break;
    case BLE_PARSE_PAYLOAD:
        p->payload[p->payload_idx++] = ch;
        p->crc_calc = crc16_update_byte(p->crc_calc, ch);
        if (p->payload_idx >= p->payload_len) {
            p->state = BLE_PARSE_CRC; p->crc_idx = 0;
        } break;
    case BLE_PARSE_CRC:
        p->crc_buf[p->crc_idx++] = ch;
        if (p->crc_idx == 2) {
            uint16_t crc_rcvd = p->crc_buf[0] | (p->crc_buf[1] << 8);
            p->state = BLE_PARSE_IDLE;
            return (crc_rcvd == p->crc_calc) ? FRAME_OK : FRAME_CRC_ERROR;
        } break;
    }
    return FRAME_INCOMPLETE;
}
\\\

### 4.3 MCU 端发送帧（C）

\\\c
void ble_ota_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len) {
    uint16_t crc = 0xFFFF;
    crc = crc16_update_byte(crc, cmd);
    crc = crc16_update_byte(crc, (uint8_t)(len & 0xFF));
    crc = crc16_update_byte(crc, (uint8_t)((len >> 8) & 0xFF));
    serial_write_byte(BLE_UART, 0x5A);  serial_write_byte(BLE_UART, 0xA5);
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
\\\

---

## 5. MCU 端 OTA 主循环（C）

\\\c
static uint16_t g_ota_sid = 0, g_ota_seq = 0;
static uint32_t g_ota_size = 0, g_ota_crc32 = 0, g_ota_addr = 0, g_ota_rcvd = 0;
static uint32_t g_ota_new_ver = 0;
static uint32_t g_word_buf = 0xFFFFFFFFU; static uint8_t g_word_pos = 0;

int ble_ota_download(void) {
    BleOtaParser_t parser = {0};
    uint8_t ch;
    uint32_t last = trea_millis();

    while (1) {
        if ((trea_millis() - last) > 30000) return -1;
        if (!trea_uart_recv_byte_timeout(BLE_UART, &ch, 100)) continue;
        last = trea_millis();

        FrameResult_t r = ble_ota_parse_byte(ch, &parser);
        if (r == FRAME_CRC_ERROR) {
            ble_ota_send_data_nack(g_ota_sid, ERR_CRC16); continue;
        }
        if (r != FRAME_OK) continue;

        switch (parser.cmd) {
        case 0x01: {  // OTA_START_REQ
            uint16_t sid = *(uint16_t*)(&parser.payload[0]);
            uint32_t sz  = *(uint32_t*)(&parser.payload[2]);
            uint32_t crc = *(uint32_t*)(&parser.payload[6]);
            uint32_t ver = *(uint32_t*)(&parser.payload[10]);
            if (ble_ota_handle_start_req(sid, sz, crc, ver) == 0) {
                g_ota_sid = sid; g_ota_size = sz;
                g_ota_crc32 = crc; g_ota_addr = OTA_CACHE_ADDR;
                g_ota_rcvd = 0; g_ota_seq = 0;
                g_ota_new_ver = ver;
                g_word_buf = 0xFFFFFFFFU; g_word_pos = 0;
                fmc_unlock();  // 握手中擦除后已lock，数据帧写Flash前需重新unlock
                fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
            }
        } break;

        case 0x02: {  // OTA_DATA
            uint16_t sid = *(uint16_t*)(&parser.payload[0]);
            uint16_t seq = *(uint16_t*)(&parser.payload[2]);
            if (sid != g_ota_sid) {
                ble_ota_send_data_nack(g_ota_sid, ERR_SESSION); continue;
            }
            if (seq != g_ota_seq) {
                ble_ota_send_data_nack(g_ota_sid, ERR_SEQ_GAP); continue;
            }
            uint8_t *data = &parser.payload[4];
            uint16_t dlen = parser.payload_len - 4;
            ota_wdt_feed();
            for (uint16_t i = 0; i < dlen; i++) {
                ((uint8_t*)&g_word_buf)[g_word_pos++] = data[i];
                if (g_word_pos == 4) {
                    if (fmc_word_program(g_ota_addr, g_word_buf) != FMC_READY) {
                        ble_ota_send_data_nack(g_ota_sid, ERR_FLASH_WRITE);
                        return -1;
                    }
                    g_ota_addr += 4; g_word_buf = 0xFFFFFFFFU; g_word_pos = 0;
                }
            }
            g_ota_rcvd += dlen; g_ota_seq++;
            ble_ota_send_data_ack(g_ota_sid, seq);
        } break;

        case 0x03: {  // OTA_END
            uint16_t sid = *(uint16_t*)(&parser.payload[0]);
            if (sid != g_ota_sid) continue;
            if (g_word_pos != 0) {
                while (g_word_pos < 4)
                    ((uint8_t*)&g_word_buf)[g_word_pos++] = 0xFF;
                fmc_word_program(g_ota_addr, g_word_buf);
            }
            if (crc32_compute((uint8_t*)OTA_CACHE_ADDR, g_ota_size) != g_ota_crc32) {
                ble_ota_send_data_nack(g_ota_sid, ERR_CRC16); return -1;
            }
            ble_ota_send_data_ack(g_ota_sid, 0xFFFF);
            trea_delay_ms(100);

            OtaInfo_t info;
            memset(&info, 0xFF, sizeof(info));
            info.magic = OTA_MAGIC_VALID;
            info.status = OTA_STATUS_PENDING;
            info.new_fw_size = g_ota_size;
            info.new_fw_version = g_ota_new_ver;
            info.retry_count = 0;
            fmc_lock();  // 数据传输完成，锁定Flash
            ota_info_write(&info);
            NVIC_SystemReset();
        } break;

        case 0x04: {  // OTA_ABORT
            if (*(uint16_t*)(&parser.payload[0]) != g_ota_sid) continue;
            ble_ota_send_data_ack(g_ota_sid, 0xFFFF); return -1;
        }
        }
    }
}
\\\

---

## 6. 握手拒绝场景

### 6.1 版本不高于当前

手机发 START_REQ(ver=2)，MCU 当前 v2 → 回复 START_NACK(REJECT_VERSION, cur=2)
手机: "MCU 已是 v2，无需升级" → 终止

### 6.2 Flash 状态异常

MCU 读 InfoBlock: status=PENDING（上次 OTA 未完成）
→ 回复 START_NACK(REJECT_FLASH_BUSY)
手机: "上次 OTA 未完成，请等待 MCU Bootloader 处理后重试"

### 6.3 电池过低

MCU 电池 2.9V < 3.1V → 回复 START_NACK(REJECT_BATTERY)
手机: "电池电量不足，请充电后再升级"


---

## 7. 手机端完整流程（Kotlin）

\\\kotlin
class BleOtaManager(private val ble: BleGattClient) {
    private var sessionId = 0

    suspend fun performOta(fw: ByteArray, ver: Int,
                           onProgress: (Int) -> Unit): Result<Unit> =
        withContext(Dispatchers.IO) {

        val totalSize = fw.size; val crc32 = Crc32Mpeg2.compute(fw)
        sessionId = Random.nextInt(1, 0xFFFF)

        // ═══ Phase 1: 三次握手 ═══
        ble.write(BleOtaProtocol.buildFrame(0x01.toByte(),
            buildStartReqPayload(sessionId, totalSize, crc32, ver)))

        val resp = waitFrame(5000)
            ?: return@withContext Result.failure(Exception("Handshake timeout"))

        when (resp.cmd) {
            0x81.toByte() -> {  // OTA_START_ACK
                val sid = (resp.payload[0].toInt() and 0xFF) or
                          ((resp.payload[1].toInt() and 0xFF) shl 8)
                val cur = (resp.payload[2].toInt() and 0xFF) or
                          ((resp.payload[3].toInt() and 0xFF) shl 8) or
                          ((resp.payload[4].toInt() and 0xFF) shl 16) or
                          ((resp.payload[5].toInt() and 0xFF) shl 24)
                val agreed = (resp.payload[6].toInt() and 0xFF) or
                             ((resp.payload[7].toInt() and 0xFF) shl 8) or
                             ((resp.payload[8].toInt() and 0xFF) shl 16) or
                             ((resp.payload[9].toInt() and 0xFF) shl 24)
                val flags = resp.payload[10].toInt() and 0xFF

                if (sid != sessionId)
                    return@withContext Result.failure(Exception("SID mismatch"))
                if (cur >= ver)
                    return@withContext Result.failure(Exception("Already v"))
                if (agreed != totalSize)
                    return@withContext Result.failure(Exception("Size mismatch"))
                if ((flags and 0x01) == 0)
                    return@withContext Result.failure(Exception("Flash not ready"))
                Log.i("OTA", "Handshake OK: v -> v")
            }
            0x82.toByte() -> {  // OTA_START_NACK
                val reason = resp.payload[2].toInt() and 0xFF
                val cur = (resp.payload[3].toInt() and 0xFF) or
                          ((resp.payload[4].toInt() and 0xFF) shl 8) or
                          ((resp.payload[5].toInt() and 0xFF) shl 16) or
                          ((resp.payload[6].toInt() and 0xFF) shl 24)
                val msg = mapOf(1 to "Already v",
                    2 to "Too large", 3 to "Flash busy",
                    4 to "Low battery", 5 to "Busy")
                return@withContext Result.failure(
                    Exception("Rejected: "))
            }
            else -> return@withContext Result.failure(
                Exception("Unexpected: "))
        }
        // ③ 握手完成

        // ═══ Phase 2: 数据传输 ═══
        var offset = 0; var seq = 0
        while (offset < totalSize) {
            val end = minOf(offset + 512, totalSize)
            val chunk = fw.copyOfRange(offset, end)
            val sent = sendChunkRetry(seq, chunk)
            if (!sent) {
                ble.write(BleOtaProtocol.buildFrame(0x04.toByte(),
                    byteArrayOf((sessionId and 0xFF).toByte(),
                                ((sessionId shr 8) and 0xFF).toByte())))
                return@withContext Result.failure(Exception("Failed at seq="))
            }
            offset = end; seq++; onProgress(offset * 100 / totalSize)
        }

        // ═══ Phase 3: 提交 ═══
        ble.write(BleOtaProtocol.buildFrame(0x03.toByte(),
            byteArrayOf((sessionId and 0xFF).toByte(),
                        ((sessionId shr 8) and 0xFF).toByte())))
        val finalAck = waitFrame(5000)
        return@withContext if (finalAck?.cmd == 0x83.toByte())
            Result.success(Unit)  // MCU 将重启
        else Result.failure(Exception("OTA_END not acked"))
    }

    private fun sendChunkRetry(seq: Int, chunk: ByteArray): Boolean {
        val frame = buildDataFrame(sessionId, seq, chunk)
        repeat(3) {
            ble.write(frame)
            val ack = waitFrame(2000)
            if (ack?.cmd == 0x83.toByte()) {
                val ackSid = (ack.payload[0].toInt() and 0xFF) or
                             ((ack.payload[1].toInt() and 0xFF) shl 8)
                val ackSeq = (ack.payload[2].toInt() and 0xFF) or
                             ((ack.payload[3].toInt() and 0xFF) shl 8)
                if (ackSid == sessionId && ackSeq == seq) return true
            }
        }
        return false
    }

    private fun buildStartReqPayload(sid: Int, sz: Int, crc: Int, ver: Int) =
        ByteArray(14).apply {
            this[0]=(sid and 0xFF).toByte(); this[1]=((sid shr 8) and 0xFF).toByte()
            this[2]=(sz and 0xFF).toByte(); this[3]=((sz shr 8) and 0xFF).toByte()
            this[4]=((sz shr 16) and 0xFF).toByte(); this[5]=((sz shr 24) and 0xFF).toByte()
            this[6]=(crc and 0xFF).toByte(); this[7]=((crc shr 8) and 0xFF).toByte()
            this[8]=((crc shr 16) and 0xFF).toByte(); this[9]=((crc shr 24) and 0xFF).toByte()
            this[10]=(ver and 0xFF).toByte(); this[11]=((ver shr 8) and 0xFF).toByte()
            this[12]=((ver shr 16) and 0xFF).toByte(); this[13]=((ver shr 24) and 0xFF).toByte()
        }

    private fun buildDataFrame(sid: Int, seq: Int, chunk: ByteArray): ByteArray {
        val p = ByteArray(4 + chunk.size)
        p[0]=(sid and 0xFF).toByte(); p[1]=((sid shr 8) and 0xFF).toByte()
        p[2]=(seq and 0xFF).toByte(); p[3]=((seq shr 8) and 0xFF).toByte()
        System.arraycopy(chunk, 0, p, 4, chunk.size)
        return BleOtaProtocol.buildFrame(0x02.toByte(), p)
    }
}
\\\

---

## 8. CRC 算法

### CRC16/CCITT（帧级校验，覆盖 CMD+LEN+PAYLOAD）

\\\c
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
\\\

### CRC32/MPEG-2（整包校验，多项式 0x04C11DB7，初始 0xFFFFFFFF，无最终异或）

\\\c
uint32_t crc32_compute(const uint8_t *data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++)
        crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ data[i]) & 0xFF];
    return crc;
}
\\\

---

## 9. 时序参数

| 参数 | 值 | 说明 |
|---|---|---|
| 握手 ACK 等待 | 5000 ms | 手机发 START_REQ 后等 ACK/NACK |
| DATA 帧 ACK 等待 | 2000 ms | 手机发 DATA 后等 ACK |
| END ACK 等待 | 5000 ms | 含 CRC32 + InfoBlock 写入 |
| MCU 空闲超时 | 30000 ms | 连续无数据视为中断 |
| 最大重试 | 3 次 | 超过后发 ABORT |
| 数据块 | 512 字节 | 8 帧 = 1 Flash 页 |


---

## 10. 帧同步机制

`
IDLE ──(0x5A)──▶ SOF ──(0xA5)──▶ HEADER ──(3B)──▶ PAYLOAD ──(LEN字节)──▶ CRC ──(通过)──▶ 交付帧
  ▲                │                  │               │                    │
  │                ▼ 非0xA5非0x5A     │               │                    ▼ CRC失败
  └────────────────┘                  │               │                回到 IDLE
  ▲  非0x5A                           │               │
  └───────────────────────────────────┘               │
  ▲  LEN > 520                                       │
  └──────────────────────────────────────────────────┘
`

---

## 11. 帧示例（完整 HEX）

### OTA_START_REQ (sid=0xA37F, size=110592, crc32=0xB8C1E3F4, ver=2)

`
5A A5 01  0E 00           ← SOF VER CMD LEN=14
7F A3                     ← session_id (LE)
00 B0 01 00               ← total_size=110592 (LE)
F4 E3 C1 B8               ← crc32 (LE)
02 00 00 00               ← new_version=2 (LE)
XX XX                     ← CRC16
`

### OTA_START_ACK (sid=0xA37F, cur=1, agreed=110592, flags=0x07)

`
5A A5 81  0B 00           ← SOF VER CMD LEN=11
7F A3                     ← session_id (回显)
01 00 00 00               ← cur_version=1 (LE)
00 B0 01 00               ← agreed_size=110592 (LE)
07                        ← flags=0x07 (FLASH_OK|BAT_OK|INFO_OK)
XX XX                     ← CRC16
`

### OTA_START_NACK (sid=0xA37F, reason=REJECT_VERSION, cur=2)

`
5A A5 82  07 00           ← SOF VER CMD LEN=7
7F A3                     ← session_id
01                        ← reason=0x01 (REJECT_VERSION)
02 00 00 00               ← cur_version=2 (LE)
XX XX                     ← CRC16
`

### OTA_DATA (sid=0xA37F, seq=0, 512B)

`
5A A5 02  04 02           ← SOF VER CMD LEN=516 (0x0204 LE)
7F A3                     ← session_id
00 00                     ← seq=0 (LE)
XX...XX (512B)            ← 固件数据
XX XX                     ← CRC16
`

### DATA_ACK (sid=0xA37F, seq=0)

`
5A A5 83  04 00           ← SOF VER CMD LEN=4
7F A3                     ← session_id
00 00                     ← seq=0
XX XX                     ← CRC16
`

### DATA_NACK (sid=0xA37F, err=ERR_CRC16)

`
5A A5 84  03 00           ← SOF VER CMD LEN=3
7F A3                     ← session_id
01                        ← error=0x01
XX XX                     ← CRC16
`

### OTA_END (sid=0xA37F)

`
5A A5 03  02 00           ← SOF VER CMD LEN=2
7F A3                     ← session_id
XX XX                     ← CRC16
`

---

## 12. 完整传输实例：握手 + 5 帧数据端到端

### 12.1 握手阶段

`
① 手机→MCU  OTA_START_REQ(sid=0xA37F, size=2560, crc32=0xB8C1E3F4, ver=2)
   HEX: 5A A5 01 0E 00  7F A3  00 0A 00 00  F4 E3 C1 B8  02 00 00 00  XX XX

② MCU 检查链:
   size=2560 ≤ 128KB ✓, ver=2 > cur(1) ✓, InfoBlock=IDLE ✓, 电池3.7V → flags=0x07 ✓
   擦除 fmc_page_erase(0x08020000) ✓

③ MCU→手机  OTA_START_ACK(sid=0xA37F, cur=1, agreed=2560, flags=0x07)
   HEX: 5A A5 81 0B 00  7F A3  01 00 00 00  00 0A 00 00  07  XX XX

④ 手机验证: session_id匹配 ✓, cur(1)<new(2) ✓, agreed=2560 ✓, flags=0x07正常 ✓
   → 握手成功！开始发送数据
`

### 12.2 数据传输

`
seq=0: 手机→ 5A A5 02 04 02  7F A3 00 00  [512B .bin前512字节]  XX XX
       MCU: 128次 fmc_word_program → 0x08020000~0x080201FF
       MCU→ 5A A5 83 04 00  7F A3 00 00  XX XX  (DATA_ACK)

seq=1: → 0x08020200~0x080203FF  →  DATA_ACK(sid, seq=1)
seq=2: → 0x08020400~0x080205FF  →  DATA_ACK(sid, seq=2)
seq=3: → 0x08020600~0x080207FF  →  DATA_ACK(sid, seq=3)
seq=4: → 0x08020800~0x080209FF  →  DATA_ACK(sid, seq=4)
`

### 12.3 提交 & 重启

`
手机→ 5A A5 03 02 00  7F A3  XX XX  (OTA_END)

MCU:  crc32(0x08020000,2560)=0xB8C1E3F4 ✓
      写 InfoBlock(0x08004000): magic=VALID, status=PENDING, size=2560, ver=2
MCU→ 5A A5 83 04 00  7F A3  FF FF  XX XX  (DATA_ACK, seq=0xFFFF="END已确认")

trea_delay_ms(100) → NVIC_SystemReset()
→ Bootloader: 读InfoBlock → 擦App → 拷贝2560B → 跳新固件
`

### 12.4 数据统计

`
手机发送: 21(START_REQ) + 521×5(DATA) + 9(END)    = 2635 字节
MCU 回复: 18(START_ACK) + 11×5(DATA_ACK) + 11(ACK) =   84 字节
有效固件:                                                  2560 字节
协议开销: (2635 - 2560) / 2560 = 2.93%
`

### 12.5 完整时序图

`
手机                                              MCU
  │                                                 │
  │  ① OTA_START_REQ (sid=0xA37F, ver=2, 2560B)    │
  │ ──────────────────────────────────────────────▶ │ 检查链 → 擦缓存
  │  ② OTA_START_ACK (cur=1, agreed=2560, flags=7) │
  │ ◀────────────────────────────────────────────── │
  │  ③ 验证通过，握手完成                             │
  │  ④ OTA_DATA(seq=0, 512B)                        │
  │ ──────────────────────────────────────────────▶ │ 逐word写Flash
  │  ⑤ DATA_ACK(seq=0)                               │
  │ ◀────────────────────────────────────────────── │
  │        ... seq=1~4 同上 ...                       │
  │  ⑥ OTA_END                                      │
  │ ──────────────────────────────────────────────▶ │ CRC32→InfoBlock
  │  ⑦ DATA_ACK(seq=0xFFFF)                          │
  │ ◀────────────────────────────────────────────── │ 重启
  │                                                 │ Bootloader→App
`

---

## 13. 小端序优势总结

| 操作 | 大端协议 | 小端协议 |
|---|---|---|
| MCU 读 session_id | (p[0]<<8) \| p[1] | *(uint16_t*)p 零开销 |
| MCU 读 seq | (p[0]<<8) \| p[1] | *(uint16_t*)p |
| MCU 读 total_size | (p[0]<<24)\|(p[1]<<16)\|(p[2]<<8)\|p[3] | *(uint32_t*)p |
| MCU 读 crc32/version | 同上 | *(uint32_t*)p |
| Flash 写入 | 每word需翻转字节序 | 直接写入，零翻转 |
| 手机端成本 | 需显式拆字节 | 需显式拆字节（无区别） |

---

> 文档结束
