/**
 * @file    ble_ota.c
 * @brief   BLE OTA 蓝牙固件升级协议实现
 * @note    适配 VG6328A-MS BLE 5.1 透传模块 + GD32L233RC
 *          协议帧: SOF(0x5A)+VER(0xA5)+CMD+LEN(小端)+PAYLOAD+CRC16(小端)
 *          升级提交: 写InfoBlock(PENDING) + NVIC_SystemReset → Bootloader拷贝
 */

#include "gd32l23x.h"
#include "ble_ota.h"
#include "ota_app.h"
#include "ota_info.h"
#include "usart.h"
#include "delay.h"
#include "main.h"
#include "mygpio.h"
#include <stdio.h>

/*===========================================================================
 * 全局变量
 *===========================================================================*/
BleOtaCtrl_t g_ble_ota;
static BleOtaParser_t g_parser;

/* CRC32查找表 (MPEG-2多项式: 0x04C11DB7) */
static const uint32_t crc32_table[256] = {
    0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B, 0x1A864DB2, 0x1E475005,
    0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61, 0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD,
    0x4C11DB70, 0x48D0C6C7, 0x4593E01E, 0x4152FDA9, 0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75,
    0x6A1936C8, 0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3, 0x709F7B7A, 0x745E66CD,
    0x9823B6E0, 0x9CE2AB57, 0x91A18D8E, 0x95609039, 0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5,
    0xBE2B5B58, 0xBAEA46EF, 0xB7A96036, 0xB3687D81, 0xAD2F2D84, 0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D,
    0xD4326D90, 0xD0F37027, 0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB, 0xCEB42022, 0xCA753D95,
    0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1, 0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D,
    0x34867077, 0x30476DC0, 0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C, 0x2E003DC5, 0x2AC12072,
    0x128E9DCF, 0x164F8078, 0x1B0CA6A1, 0x1FCDBB16, 0x018AEB13, 0x054BF6A4, 0x0808D07D, 0x0CC9CDCA,
    0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE, 0x6B93DDDB, 0x6F52C06C, 0x6211E6B5, 0x66D0FB02,
    0x5E9F46BF, 0x5A5E5B08, 0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D, 0x40D816BA,
    0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E, 0xBFA1B04B, 0xBB60ADFC, 0xB6238B25, 0xB2E29692,
    0x8AAD2B2F, 0x8E6C3698, 0x832F1041, 0x87EE0DF6, 0x99A95DF3, 0x9D684044, 0x902B669D, 0x94EA7B2A,
    0xE0B41DE7, 0xE4750050, 0xE9362689, 0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2,
    0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683, 0xD1799B34, 0xDC3ABDED, 0xD8FBA05A,
    0x690CE0EE, 0x6DCDFD59, 0x608EDB80, 0x644FC637, 0x7A089632, 0x7EC98B85, 0x738AAD5C, 0x774BB0EB,
    0x4F040D56, 0x4BC510E1, 0x46863638, 0x42472B8F, 0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53,
    0x251D3B9E, 0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5, 0x3F9B762C, 0x3B5A6B9B,
    0x0315D626, 0x07D4CB91, 0x0A97ED48, 0x0E56F0FF, 0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623,
    0xF12F560E, 0xF5EE4BB9, 0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2, 0xE6EA3D65, 0xEBA91BBC, 0xEF68060B,
    0xD727BBB6, 0xD3E6A601, 0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD, 0xCDA1F604, 0xC960EBB3,
    0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7, 0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B,
    0x9B3660C6, 0x9FF77D71, 0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD, 0x81B02D74, 0x857130C3,
    0x5D8A9099, 0x594B8D2E, 0x5408ABF7, 0x50C9B640, 0x4E8EE645, 0x4A4FFBF2, 0x470CDD2B, 0x43CDC09C,
    0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8, 0x68860BFD, 0x6C47164A, 0x61043093, 0x65C52D24,
    0x119B4BE9, 0x155A565E, 0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B, 0x0FDC1BEC,
    0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088, 0x2497D08D, 0x2056CD3A, 0x2D15EBE3, 0x29D4F654,
    0xC5A92679, 0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0, 0xD6AD50A5, 0xD26C4D12, 0xDF2F6BCB, 0xDBEE767C,
    0xE3A1CBC1, 0xE760D676, 0xEA23F0AF, 0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4,
    0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5, 0x9E7D9662, 0x933EB0BB, 0x97FFAD0C,
    0xAFB010B1, 0xAB710D06, 0xA6322BDF, 0xA2F33668, 0xBCB4666D, 0xB8757BDA, 0xB5365D03, 0xB1F740B4
};

/*===========================================================================
 * ble_uart_send_byte - 蓝牙串口发送单字节
 *===========================================================================*/
static void ble_uart_send_byte(uint8_t ch)
{
    usart_data_transmit(UART4, ch);
    while(RESET == usart_flag_get(UART4, USART_FLAG_TBE));
}

/*===========================================================================
 * CRC16/CCITT (多项式: 0x1021, 初始值: 0xFFFF)
 *===========================================================================*/
uint16_t ble_ota_crc16_update(uint16_t crc, const uint8_t *data, uint16_t len)
{
    uint16_t i;
    while(len--)
    {
        crc ^= (uint16_t)(*data++ << 8);
        for(i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
    }
    return crc;
}

uint16_t ble_ota_crc16_update_byte(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)(byte << 8);
    for(uint8_t i = 0; i < 8; i++)
        crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
    return crc;
}

/*===========================================================================
 * CRC32/MPEG-2 (多项式: 0x04C11DB7, 初始值: 0xFFFFFFFF)
 *===========================================================================*/
uint32_t ble_ota_crc32_compute(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    while(len--)
        crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ *data++) & 0xFF];
    return crc;
}

/*===========================================================================
 * ble_ota_send_frame - 发送完整协议帧
 *===========================================================================*/
void ble_ota_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;

    /* CRC覆盖: CMD + LEN_L + LEN_H + PAYLOAD */
    crc = ble_ota_crc16_update_byte(crc, cmd);
    crc = ble_ota_crc16_update_byte(crc, (uint8_t)(len & 0xFF));
    crc = ble_ota_crc16_update_byte(crc, (uint8_t)((len >> 8) & 0xFF));

    /* 发送帧头 */
    ble_uart_send_byte(BLE_OTA_SOF);
    ble_uart_send_byte(BLE_OTA_VER);
    ble_uart_send_byte(cmd);
    ble_uart_send_byte((uint8_t)(len & 0xFF));
    ble_uart_send_byte((uint8_t)((len >> 8) & 0xFF));

    /* 发送负载 */
    for(i = 0; i < len; i++)
    {
        ble_uart_send_byte(payload[i]);
        crc = ble_ota_crc16_update_byte(crc, payload[i]);
    }

    /* 发送CRC16 (小端) */
    ble_uart_send_byte((uint8_t)(crc & 0xFF));
    ble_uart_send_byte((uint8_t)((crc >> 8) & 0xFF));
}

/*===========================================================================
 * 发送 OTA_START_ACK (同意升级)
 * 负载: session_id(2) + cur_version(4) + agreed_size(4) + flags(1) = 11B
 *===========================================================================*/
void ble_ota_send_start_ack(uint16_t sid, uint32_t cur_ver, uint32_t agreed_size, uint8_t flags)
{
    uint8_t buf[11];
    *(uint16_t *)(&buf[0]) = sid;
    *(uint32_t *)(&buf[2]) = cur_ver;
    *(uint32_t *)(&buf[6]) = agreed_size;
    buf[10] = flags;
    ble_ota_send_frame(CMD_OTA_START_ACK, buf, 11);
}

/*===========================================================================
 * 发送 OTA_START_NACK (拒绝升级)
 * 负载: session_id(2) + reject_reason(1) + cur_version(4) = 7B
 *===========================================================================*/
void ble_ota_send_start_nack(uint16_t sid, uint8_t reason, uint32_t cur_ver)
{
    uint8_t buf[7];
    *(uint16_t *)(&buf[0]) = sid;
    buf[2] = reason;
    *(uint32_t *)(&buf[3]) = cur_ver;
    ble_ota_send_frame(CMD_OTA_START_NACK, buf, 7);
}

/*===========================================================================
 * 发送 DATA_ACK (数据帧确认)
 * 负载: session_id(2) + seq(2) = 4B
 *===========================================================================*/
void ble_ota_send_data_ack(uint16_t sid, uint16_t seq)
{
    uint8_t buf[4];
    *(uint16_t *)(&buf[0]) = sid;
    *(uint16_t *)(&buf[2]) = seq;
    ble_ota_send_frame(CMD_DATA_ACK, buf, 4);
}

/*===========================================================================
 * 发送 DATA_NACK (数据帧拒绝)
 * 负载: session_id(2) + error_code(1) = 3B
 *===========================================================================*/
void ble_ota_send_data_nack(uint16_t sid, uint8_t error_code)
{
    uint8_t buf[3];
    *(uint16_t *)(&buf[0]) = sid;
    buf[2] = error_code;
    ble_ota_send_frame(CMD_DATA_NACK, buf, 3);
}

/*===========================================================================
 * ble_ota_parse_byte - 逐字节帧解析器
 *===========================================================================*/
FrameResult_t ble_ota_parse_byte(uint8_t ch, BleOtaParser_t *p)
{
    switch(p->state)
    {
    case BLE_PARSE_IDLE:
        if(ch == BLE_OTA_SOF)
            p->state = BLE_PARSE_SOF;
        break;

    case BLE_PARSE_SOF:
        if(ch == BLE_OTA_VER)
        {
            p->state = BLE_PARSE_HEADER;
            p->header_idx = 0;
            p->crc_calc = 0xFFFF;
        }
        else if(ch != BLE_OTA_SOF)
        {
            p->state = BLE_PARSE_IDLE;
        }
        /* 如果是0x5A则保持在SOF状态 */
        break;

    case BLE_PARSE_HEADER:
        p->header_buf[p->header_idx++] = ch;
        if(p->header_idx >= 3)
        {
            p->cmd = p->header_buf[0];
            p->payload_len = p->header_buf[1] | ((uint16_t)p->header_buf[2] << 8);

            if(p->payload_len > BLE_OTA_PAYLOAD_MAX)
            {
                /* 负载长度超限，丢弃此帧 */
                p->state = BLE_PARSE_IDLE;
                return FRAME_CRC_ERROR;
            }

            /* CRC覆盖: CMD + LEN_L + LEN_H */
            p->crc_calc = ble_ota_crc16_update(p->crc_calc, p->header_buf, 3);
            p->payload_idx = 0;

            if(p->payload_len > 0)
                p->state = BLE_PARSE_PAYLOAD;
            else
                p->state = BLE_PARSE_CRC;
        }
        break;

    case BLE_PARSE_PAYLOAD:
        p->payload[p->payload_idx++] = ch;
        p->crc_calc = ble_ota_crc16_update_byte(p->crc_calc, ch);
        if(p->payload_idx >= p->payload_len)
        {
            p->state = BLE_PARSE_CRC;
            p->crc_idx = 0;
        }
        break;

    case BLE_PARSE_CRC:
        p->crc_buf[p->crc_idx++] = ch;
        if(p->crc_idx >= 2)
        {
            uint16_t crc_rcvd = p->crc_buf[0] | ((uint16_t)p->crc_buf[1] << 8);
            p->state = BLE_PARSE_IDLE;
            return (crc_rcvd == p->crc_calc) ? FRAME_OK : FRAME_CRC_ERROR;
        }
        break;
    }
    return FRAME_INCOMPLETE;
}

/*===========================================================================
 * ble_ota_init - OTA状态初始化
 *===========================================================================*/
void ble_ota_init(void)
{
    memset(&g_ble_ota, 0, sizeof(g_ble_ota));
    memset(&g_parser, 0, sizeof(g_parser));
    g_ble_ota.state = BLE_OTA_WAIT_CONNECT;
    g_ble_ota.last_rx_ms = trea_millis();
    g_parser.state = BLE_PARSE_IDLE;
}

/*===========================================================================
 * handle_start_req - 处理 OTA_START_REQ
 *===========================================================================*/
static void handle_start_req(BleOtaParser_t *p)
{
    uint16_t sid;
    uint32_t total_size, crc32, new_version;

    /* 解析负载 (小端序) */
    sid        = *(uint16_t *)(&p->payload[0]);
    total_size = *(uint32_t *)(&p->payload[2]);
    crc32      = *(uint32_t *)(&p->payload[6]);
    new_version= *(uint32_t *)(&p->payload[10]);

    printf_debug("[BLE_OTA] START_REQ: sid=0x%04X, size=%u, ver=%u, crc=0x%08X\r\n",
                 sid, (unsigned)total_size, (unsigned)new_version, (unsigned)crc32);

    /* 1. 参数合法性校验 */
    if(total_size == 0 || total_size > OTA_CACHE_SIZE)
    {
        printf_debug("[BLE_OTA] REJECT: size invalid (%u > %u)\r\n",
                     (unsigned)total_size, (unsigned)OTA_CACHE_SIZE);
        ble_ota_send_start_nack(sid, REJECT_SIZE, APP_FW_VERSION_NUM);
        return;
    }

    /* 2. 版本校验: 新版本必须高于当前版本 */
    if(new_version <= APP_FW_VERSION_NUM)
    {
        printf_debug("[BLE_OTA] REJECT: version not higher (%u <= %u)\r\n",
                     (unsigned)new_version, (unsigned)APP_FW_VERSION_NUM);
        ble_ota_send_start_nack(sid, REJECT_VERSION, APP_FW_VERSION_NUM);
        return;
    }

    /* 3. 检查InfoBlock状态: 上次OTA必须已完成 */
    {
        const volatile OtaInfo_t *info = OTA_INFO_PTR;
        if(info->status == OTA_STATUS_PENDING || info->status == OTA_STATUS_FLASHING)
        {
            printf_debug("[BLE_OTA] REJECT: flash busy (status=0x%08X)\r\n",
                         (unsigned)info->status);
            ble_ota_send_start_nack(sid, REJECT_FLASH_BUSY, APP_FW_VERSION_NUM);
            return;
        }
    }

    /* 4. 擦除缓存区 */
    printf_debug("[BLE_OTA] Erasing cache 0x%08X (%u KB)...\r\n",
                 (unsigned)OTA_CACHE_ADDR, (unsigned)(OTA_CACHE_SIZE / 1024));
    {
        uint32_t addr;
        uint32_t pages = OTA_CACHE_SIZE / OTA_FLASH_PAGE_SIZE;
        uint32_t i;
        fmc_unlock();
        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
        for(i = 0, addr = OTA_CACHE_ADDR; i < pages; i++, addr += OTA_FLASH_PAGE_SIZE)
        {
            ota_wdt_feed();
            fmc_page_erase(addr);
            fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
        }
        /* 保持Flash解锁 (OTA数据写入期间) */
    }
    printf_debug("[BLE_OTA] Cache erased OK\r\n");

    /* 5. 全部通过 → 初始化OTA状态 */
    g_ble_ota.session_id     = sid;
    g_ble_ota.total_size     = total_size;
    g_ble_ota.expected_crc32 = crc32;
    g_ble_ota.new_version    = new_version;
    g_ble_ota.flash_addr     = OTA_CACHE_ADDR;
    g_ble_ota.expected_seq   = 0;
    g_ble_ota.received_bytes = 0;
    g_ble_ota.word_buf       = 0xFFFFFFFF;
    g_ble_ota.word_pos       = 0;
    g_ble_ota.last_rx_ms     = trea_millis();
    g_ble_ota.state          = BLE_OTA_DOWNLOADING;

    /* 发送ACK */
    uint8_t flags = 0x07;  /* FLASH_OK | BATTERY_OK | INFO_OK */
    ble_ota_send_start_ack(sid, APP_FW_VERSION_NUM, total_size, flags);
    printf_debug("[BLE_OTA] START_ACK sent, downloading...\r\n");
}

/*===========================================================================
 * handle_data - 处理 OTA_DATA
 *===========================================================================*/
static void handle_data(BleOtaParser_t *p)
{
    uint16_t rx_sid, rx_seq, dlen;
    uint8_t *data;
    uint16_t i;

    if(g_ble_ota.state != BLE_OTA_DOWNLOADING)
    {
        printf_debug("[BLE_OTA] DATA in wrong state(%d)\r\n", g_ble_ota.state);
        return;
    }

    /* 解析负载头 */
    rx_sid = *(uint16_t *)(&p->payload[0]);
    rx_seq = *(uint16_t *)(&p->payload[2]);

    /* 校验session_id */
    if(rx_sid != g_ble_ota.session_id)
    {
        printf_debug("[BLE_OTA] DATA session mismatch: 0x%04X != 0x%04X\r\n",
                     rx_sid, g_ble_ota.session_id);
        ble_ota_send_data_nack(g_ble_ota.session_id, ERR_SESSION);
        return;
    }

    /* 校验seq连续性 */
    if(rx_seq != g_ble_ota.expected_seq)
    {
        printf_debug("[BLE_OTA] DATA seq gap: %u != %u\r\n", rx_seq, g_ble_ota.expected_seq);
        ble_ota_send_data_nack(g_ble_ota.session_id, ERR_SEQ_GAP);
        return;
    }

    /* 提取数据 */
    data = &p->payload[4];
    dlen = p->payload_len - 4;  /* 减去 sid(2) + seq(2) */

    /* 逐字节拼word写入Flash */
    for(i = 0; i < dlen; i++)
    {
        ((uint8_t *)&g_ble_ota.word_buf)[g_ble_ota.word_pos++] = data[i];
        if(g_ble_ota.word_pos >= 4)
        {
            fmc_word_program(g_ble_ota.flash_addr, g_ble_ota.word_buf);
            fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
            g_ble_ota.flash_addr += 4;
            g_ble_ota.word_buf = 0xFFFFFFFF;
            g_ble_ota.word_pos = 0;
        }
    }

    g_ble_ota.received_bytes += dlen;
    g_ble_ota.expected_seq++;
    g_ble_ota.last_rx_ms = trea_millis();

    /* 发送ACK */
    ble_ota_send_data_ack(g_ble_ota.session_id, rx_seq);

    /* 进度打印 (每10%打印一次) */
    {
        static uint8_t last_pct = 0xFF;
        uint8_t pct = (uint8_t)((uint64_t)g_ble_ota.received_bytes * 100 / g_ble_ota.total_size);
        if(pct != last_pct && (pct % 10 == 0))
        {
            last_pct = pct;
            printf_debug("[BLE_OTA] Progress: %u%% (%u/%u)\r\n",
                         pct, (unsigned)g_ble_ota.received_bytes, (unsigned)g_ble_ota.total_size);
        }
    }
}

/*===========================================================================
 * handle_end - 处理 OTA_END
 * 使用 Bootloader 路径: 写InfoBlock(PENDING) → NVIC_SystemReset
 *===========================================================================*/
static void handle_end(BleOtaParser_t *p)
{
    uint16_t rx_sid;
    uint32_t calc_crc;

    if(g_ble_ota.state != BLE_OTA_DOWNLOADING)
    {
        printf_debug("[BLE_OTA] END in wrong state(%d)\r\n", g_ble_ota.state);
        return;
    }

    rx_sid = *(uint16_t *)(&p->payload[0]);
    if(rx_sid != g_ble_ota.session_id)
    {
        printf_debug("[BLE_OTA] END session mismatch\r\n");
        return;
    }

    printf_debug("[BLE_OTA] OTA_END received, total=%u bytes\r\n",
                 (unsigned)g_ble_ota.received_bytes);

    /* 1. 补齐最后一个word (不足4字节用0xFF填充) */
    if(g_ble_ota.word_pos != 0)
    {
        while(g_ble_ota.word_pos < 4)
            ((uint8_t *)&g_ble_ota.word_buf)[g_ble_ota.word_pos++] = 0xFF;
        fmc_word_program(g_ble_ota.flash_addr, g_ble_ota.word_buf);
        fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
    }

    /* 2. 加锁Flash */
    fmc_lock();

    /* 3. CRC32全量校验 */
    printf_debug("[BLE_OTA] Computing CRC32...\r\n");
    ota_wdt_feed();
    calc_crc = ble_ota_crc32_compute((uint8_t *)OTA_CACHE_ADDR, g_ble_ota.total_size);
    printf_debug("[BLE_OTA] Expected: 0x%08X, Calculated: 0x%08X\r\n",
                 (unsigned)g_ble_ota.expected_crc32, (unsigned)calc_crc);

    if(calc_crc != g_ble_ota.expected_crc32)
    {
        printf_debug("[BLE_OTA] CRC32 MISMATCH! Upgrade FAILED!\r\n");
        ble_ota_send_data_nack(g_ble_ota.session_id, ERR_FLASH_WRITE);
        g_ble_ota.state = BLE_OTA_ERROR;
        return;
    }

    /* 4. CRC32校验通过 → 写InfoBlock (Bootloader路径) */
    printf_debug("[BLE_OTA] CRC32 OK! Writing InfoBlock...\r\n");
    {
        OtaInfo_t info;
        memset(&info, 0xFF, sizeof(info));
        info.magic          = OTA_MAGIC_VALID;
        info.status         = OTA_STATUS_PENDING;
        info.new_fw_size    = g_ble_ota.total_size;
        info.new_fw_version = g_ble_ota.new_version;
        info.retry_count    = 0;

        if(ota_info_write(&info) != 0)
        {
            printf_debug("[BLE_OTA] InfoBlock write FAILED!\r\n");
            ble_ota_send_data_nack(g_ble_ota.session_id, ERR_FLASH_WRITE);
            g_ble_ota.state = BLE_OTA_ERROR;
            return;
        }
    }

    /* 5. 发送END确认 (seq=0xFFFF表示END确认) */
    ble_ota_send_data_ack(g_ble_ota.session_id, 0xFFFF);
    g_ble_ota.state = BLE_OTA_COMPLETE;

    printf_debug("[BLE_OTA] Upgrade committed! Resetting in 100ms...\r\n");

    /* 6. 延时后重启 → Bootloader 读InfoBlock → 擦App → 拷贝 → 跳转新固件 */
    trea_delay_ms(100);
    NVIC_SystemReset();
}

/*===========================================================================
 * handle_abort - 处理 OTA_ABORT
 *===========================================================================*/
static void handle_abort(BleOtaParser_t *p)
{
    uint16_t rx_sid = *(uint16_t *)(&p->payload[0]);
    printf_debug("[BLE_OTA] ABORT received, sid=0x%04X\r\n", rx_sid);

    if(g_ble_ota.state == BLE_OTA_DOWNLOADING)
    {
        fmc_lock();
    }
    g_ble_ota.state = BLE_OTA_IDLE;
}

/*===========================================================================
 * ble_ota_process - OTA主处理循环
 * 在蓝牙开启期间由主循环周期性调用
 * 从DMA环形缓冲区读取数据，逐字节解析到一帧完成
 *===========================================================================*/
void ble_ota_process(void)
{
    FrameResult_t result;
    uint8_t ch;
    uint8_t frame_done = 0;
    uint16_t byte_count = 0;

    /* 从DMA环形缓冲区读取数据 */
    while(!frame_done && ble_uart_rx_available() > 0)
    {
        ch = ble_uart_rx_read();
        byte_count++;

        result = ble_ota_parse_byte(ch, &g_parser);

        if(result == FRAME_OK)
        {
            frame_done = 1;

            /* 重置空闲计时 */
            g_ble_ota.last_rx_ms = trea_millis();

            /* 帧解析完成且CRC校验通过 */
            switch(g_parser.cmd)
            {
            case CMD_OTA_START_REQ:
                handle_start_req(&g_parser);
                break;

            case CMD_OTA_DATA:
                handle_data(&g_parser);
                break;

            case CMD_OTA_END:
                handle_end(&g_parser);
                break;

            case CMD_OTA_ABORT:
                handle_abort(&g_parser);
                break;

            default:
                printf_debug("[BLE_OTA] Unknown CMD: 0x%02X\r\n", g_parser.cmd);
                break;
            }
        }
        else if(result == FRAME_CRC_ERROR)
        {
            frame_done = 1;
            printf_debug("[BLE_OTA] Frame CRC error, CMD=0x%02X\r\n", g_parser.cmd);
            /* 如果正在下载中，发送NACK */
            if(g_ble_ota.state == BLE_OTA_DOWNLOADING)
            {
                ble_ota_send_data_nack(g_ble_ota.session_id, ERR_CRC16);
            }
        }

        /* 防止单次处理太多字节阻塞主循环 */
        if(byte_count >= 600)
        {
            break;
        }
    }

    /* 超时检测: 使用 trea_millis() 时间戳 */
    if(g_ble_ota.state == BLE_OTA_WAIT_CONNECT)
    {
        /* 等待手机连接BLE: 60s超时 */
        if((trea_millis() - g_ble_ota.last_rx_ms) > BLE_OTA_CONNECT_TIMEOUT)
        {
            printf_debug("[BLE_OTA] Connect timeout! No BLE connection for %u ms\r\n",
                         BLE_OTA_CONNECT_TIMEOUT);
            g_ble_ota.state = BLE_OTA_ERROR;
        }
    }
    else if(g_ble_ota.state == BLE_OTA_IDLE)
    {
        /* 已连接但等待START_REQ: 30s超时 */
        if((trea_millis() - g_ble_ota.last_rx_ms) > BLE_OTA_IDLE_TIMEOUT)
        {
            printf_debug("[BLE_OTA] Idle timeout! No START_REQ for %u ms\r\n",
                         BLE_OTA_IDLE_TIMEOUT);
            g_ble_ota.state = BLE_OTA_ERROR;
        }
    }
    else if(g_ble_ota.state == BLE_OTA_DOWNLOADING)
    {
        /* 数据传输中: 30s无数据超时 */
        if((trea_millis() - g_ble_ota.last_rx_ms) > BLE_OTA_IDLE_TIMEOUT)
        {
            printf_debug("[BLE_OTA] Timeout! No data for %u ms\r\n", BLE_OTA_IDLE_TIMEOUT);
            fmc_lock();
            g_ble_ota.state = BLE_OTA_ERROR;
        }
    }
}
