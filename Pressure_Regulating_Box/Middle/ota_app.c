/**
 * @file  ota_app.c
 * @brief App 端 OTA 流程实现
 */
#include "ota_app.h"
#include "ec800e.h"
#include "eeprom_config.h"
#include "adc_bat.h"
#include "usart.h"
#include "delay.h"
#include "tim.h"
#include "main.h"
#include "gd32l23x.h"
#include "gd32l23x_fwdgt.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define OTA_CSQ_MIN  10   			// OTA最低信号要求 (CSQ<10 拒绝升级)
#define OTA_vol_protection  2.8     // OTA最低电压要求 (value<OTA_vol_protection 拒绝升级)

// OTA upgrade 状态上报值 (id:upgrade)
#define OTA_ST_UPGRADING     0   // 进行升级
#define OTA_ST_SUCCESS       1   // 升级成功
#define OTA_ST_FAILED        2   // 升级失败
#define OTA_ST_REJECT_CSQ    3   // 拒绝升级（信号）
#define OTA_ST_REJECT_VOL    4   // 拒绝升级（电压）
#define OTA_ST_VERIFY_OK     5   // 校验成功
#define OTA_ST_VERIFY_FAIL   6   // 校验失败
#define OTA_ST_DOWNLOADING   7   // 下载中
#define OTA_ST_REBOOT        8   // 重启
#define OTA_ST_VER_LOW       9   // 固件版本低

// 内部helper: 上报 upgrade 状态到数据topic
static void ota_report_status(uint8_t status)
{
    static char s_ota_ts[32];
    static char s_ota_json[128];
    rtc_get_time_str(s_ota_ts, sizeof(s_ota_ts));
    snprintf(s_ota_json, sizeof(s_ota_json),
             "[{\"id\":\"upgrade\",\"value\":\"%d\",\"ts\":\"%s\",\"remark\":\"\"}]",
             status, s_ota_ts);
    EC801E_PublishRawJson(s_ota_json);
    trea_delay_ms(300);
}

/* ==================== InfoBlock 读写 (1 KB 整页) ==================== */

/* 用 SRAM 缓冲拼装一页, 一次性写入 (保证幂等) */
int ota_info_write(const OtaInfo_t *info)
{
    static uint32_t buf[OTA_FLASH_PAGE_SIZE / 4];
    uint32_t i;
    uint32_t addr;
    fmc_state_enum st;

    /* 1. 准备一页内容: 头部 sizeof(OtaInfo_t), 尾部 0xFF */
    printf_debug("[OTA] info_write: memset...\r\n");
    memset(buf, 0xFF, sizeof(buf));
    memcpy(buf, info, sizeof(OtaInfo_t));

    /* 2. 解锁 + 擦除 + 写入 */
    printf_debug("[OTA] info_write: fmc_unlock...\r\n");
    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);

    printf_debug("[OTA] info_write: erasing 0x%08X...\r\n", (unsigned int)OTA_INFO_ADDR);
    st = fmc_page_erase(OTA_INFO_ADDR);
    if(st != FMC_READY)
    {
        fmc_lock();
        printf_debug("[OTA] info erase fail (%d)\r\n", (int)st);
        return -1;
    }
    printf_debug("[OTA] info_write: erase OK, programming...\r\n");

    addr = OTA_INFO_ADDR;
    for(i = 0; i < (OTA_FLASH_PAGE_SIZE / 4); i++)
    {
        st = fmc_word_program(addr, buf[i]);
        if(st != FMC_READY)
        {
            fmc_lock();
            printf_debug("[OTA] info prog fail @0x%08X (%d)\r\n",
                         (unsigned int)addr, (int)st);
            return -1;
        }
        addr += 4;
    }
    fmc_lock();
    printf_debug("[OTA] info_write: done OK\r\n");
    return 0;
}

/* 清除 InfoBlock (擦除整页, 等价于 status=IDLE, magic=0xFFFFFFFF) */
static void ota_info_clear(void)
{
    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);
    fmc_page_erase(OTA_INFO_ADDR);
    fmc_lock();
}

/* ==================== 看门狗控制 (仅在 OTA 升级期间启用) ==================== */
void ota_wdt_enable(void)
{
    printf_debug("[WDT] Enabling Independent Watchdog for OTA (approx 26.2s timeout)\r\n");
    fwdgt_write_enable();
    fwdgt_config(4095, FWDGT_PSC_DIV256);
    fwdgt_counter_reload();
    fwdgt_enable();
}

void ota_wdt_feed(void)
{
    fwdgt_counter_reload();
}

/* ==================== 缓存区擦除 ==================== */

static int ota_cache_erase(void)
{
    uint32_t addr;
    fmc_state_enum st;
    uint32_t pages = OTA_CACHE_SIZE / OTA_FLASH_PAGE_SIZE;
    uint32_t i;

    fmc_unlock();
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_WPERR | FMC_FLAG_PGERR);

    addr = OTA_CACHE_ADDR;
    for(i = 0; i < pages; i++)
    {
        ota_wdt_feed();  /* 喂狗防止擦写耗时导致重启 */
        st = fmc_page_erase(addr);
        if(st != FMC_READY)
        {
            fmc_lock();
            printf_debug("[OTA] cache erase fail @0x%08X (%d)\r\n",
                         (unsigned int)addr, (int)st);
            return -1;
        }
        addr += OTA_FLASH_PAGE_SIZE;
    }
    fmc_lock();
    printf_debug("[OTA] cache erased (%d pages)\r\n", (int)pages);
    return 0;
}

/* ==================== JSON 字段提取 (与 main.c 同款轻量解析) ==================== */
/* 提取字符串值: "key":"value" */
static void ota_extract_field(const char *buf, const char *key, char *out, uint16_t out_size)
{
    char *p;
    char *q;
    char *r;
    uint16_t len;

    out[0] = '\0';
    p = strstr(buf, key);
    if(!p) return;
    q = strchr(p + strlen(key), '"');
    if(!q) return;
    q++;
    r = strchr(q, '"');
    if(!r) return;
    len = (uint16_t)(r - q);
    if(len >= out_size) len = out_size - 1;
    memcpy(out, q, len);
    out[len] = '\0';
}

/* 提取数字值: "key":123  (无引号包裹) */
static uint32_t ota_extract_int(const char *buf, const char *key)
{
    const char *p;
    const char *q;

    p = strstr(buf, key);
    if(!p) return 0;
    q = p + strlen(key);
    /* 跳过 ':' 和空格 */
    while(*q == ':' || *q == ' ') q++;
    /* 兼容带引号的数字: "version":"2" */
    if(*q == '"') q++;
    return (uint32_t)atoi(q);
}

/* 从 url 提取 host (跳过 http:// 或 https://, 取到第一个 '/') */
static int ota_parse_url(const char *url, char *host, uint16_t host_size,
                         const char **path_out)
{
    const char *p;
    const char *slash;
    uint16_t hlen;

    if(!url) return -1;

    if(strncmp(url, "http://", 7) == 0)
        p = url + 7;
    else if(strncmp(url, "https://", 8) == 0)
        p = url + 8;
    else
        p = url;

    slash = strchr(p, '/');
    if(!slash)
    {
        /* 无路径, 整段当 host, path = "/" */
        hlen = (uint16_t)strlen(p);
        if(hlen >= host_size) hlen = host_size - 1;
        memcpy(host, p, hlen);
        host[hlen] = '\0';
        *path_out = "/";
        return 0;
    }

    hlen = (uint16_t)(slash - p);
    if(hlen >= host_size) hlen = host_size - 1;
    memcpy(host, p, hlen);
    host[hlen] = '\0';
    *path_out = slash;
    return 0;
}

/* ==================== 升级主流程 ==================== */
void ota_app_handle_upgrade_cmd(const char *payload)
{
    static char url_buf[256];
    uint32_t new_version;
    uint32_t actual_size = 0;
    OtaInfo_t info;

    printf_debug("[OTA] upgrade cmd: %s\r\n", payload);

    /* 1. 解析 JSON */
    ota_extract_field(payload, "\"downloadUrl\"", url_buf, sizeof(url_buf));
    new_version = ota_extract_int(payload, "\"version\"");

    if(url_buf[0] == '\0')
    {
        printf_debug("[OTA] no downloadUrl\r\n");
        ota_report_status(OTA_ST_FAILED);
        EC801E_PublishUpgradeReply(OTA_REPLY_FAILED);
        return;
    }

    /* 1.5 前置条件检查: 电压+信号, 不满足则拒绝升级 */
    {
        // 电压检查: 低电压模式或实时电压低于保护值
        float bat_v = adc_bat_read_voltage();
        if(voltage_low_power || (bat_v < OTA_vol_protection))
        {
            printf_debug("[OTA] reject: low voltage (%.2fV, protect=%.2f)\r\n",
                         bat_v, OTA_vol_protection);
            ota_report_status(OTA_ST_REJECT_VOL);
            EC801E_PublishUpgradeReply(OTA_REPLY_FAILED);
            return;
        }

        // 信号检查: CSQ过低则拒绝(下载大概率失败)
        {
            char csq_buf[8] = {0};
            if(EC801E_QueryCSQ(csq_buf, sizeof(csq_buf)) == EC_OK)
            {
                uint8_t csq_val = (uint8_t)atoi(csq_buf);
                if(csq_val < OTA_CSQ_MIN || csq_val == 99)
                {
                    printf_debug("[OTA] reject: weak signal (CSQ=%d)\r\n", csq_val);
                    ota_report_status(OTA_ST_REJECT_CSQ);
                    EC801E_PublishUpgradeReply(OTA_REPLY_FAILED);
                    return;
                }
            }
        }
    }

    /* 2. 立即上报 REPLY=2 (升级中) */
    EC801E_PublishUpgradeReply(OTA_REPLY_REPLY);
    trea_delay_ms(200);

    /* 3. 版本检查: 新版本必须严格高于当前版本, 否则拒绝 */
    if(new_version == 0 || new_version <= APP_FW_VERSION_NUM)
    {
        printf_debug("[OTA] reject: new=%u cur=%u (need higher)\r\n",
                     (unsigned int)new_version, (unsigned int)APP_FW_VERSION_NUM);
        ota_report_status(OTA_ST_VER_LOW);
        EC801E_PublishUpgradeReply(OTA_REPLY_FAILED);
        return;
    }

    /* 版本校验通过，上报 upgrade=0 (进行升级) */
    ota_report_status(OTA_ST_UPGRADING);

    /* 启动看门狗保护OTA过程 */
    ota_wdt_enable();

    /* 4. 保持 4G 在线 (取消 QSCLK) */
    EC801E_SetPSM(0);
    trea_delay_ms(300);

    /* 5. 擦除缓存区 */
    if(ota_cache_erase() != 0)
    {
        ota_report_status(OTA_ST_FAILED);
        EC801E_PublishUpgradeReply(OTA_REPLY_FAILED);
        EC801E_SetPSM(1);
        trea_delay_ms(1000);
        NVIC_SystemReset();
        return;
    }

    /* 6. HTTP 下载 → 缓存区 */
    ota_report_status(OTA_ST_DOWNLOADING);
    {
        char host[64];
        const char *path;
        if(ota_parse_url(url_buf, host, sizeof(host), &path) != 0)
        {
            printf_debug("[OTA] parse url fail\r\n");
            ota_report_status(OTA_ST_FAILED);
            EC801E_PublishUpgradeReply(OTA_REPLY_FAILED);
            EC801E_SetPSM(1);
            trea_delay_ms(1000);
            NVIC_SystemReset();
            return;
        }
        if(EC801E_HttpDownloadToFlash(url_buf,
                                      OTA_CACHE_ADDR,
                                      OTA_CACHE_SIZE,
                                      &actual_size) != EC_OK)
        {
            printf_debug("[OTA] download fail (got %u bytes)\r\n",
                         (unsigned int)actual_size);
            ota_report_status(OTA_ST_FAILED);
            EC801E_PublishUpgradeReply(OTA_REPLY_FAILED);
            EC801E_SetPSM(1);
            trea_delay_ms(1000);
            NVIC_SystemReset();
            return;
        }
    }

    /* 7. 固件校验 */
    if(actual_size == 0 || actual_size > OTA_APP_SIZE)
    {
        printf_debug("[OTA] bad size: %u (max %u)\r\n",
                     (unsigned int)actual_size, (unsigned int)OTA_APP_SIZE);
        ota_report_status(OTA_ST_VERIFY_FAIL);
        EC801E_PublishUpgradeReply(OTA_REPLY_FAILED);
        EC801E_SetPSM(1);
        trea_delay_ms(1000);
        NVIC_SystemReset();
        return;
    }
    printf_debug("[OTA] downloaded %u bytes, verify OK\r\n", (unsigned int)actual_size);
    ota_report_status(OTA_ST_VERIFY_OK);

    /* 7. 写 InfoBlock: PENDING + size + version */
    printf_debug("[OTA] preparing InfoBlock...\r\n");
    memset(&info, 0xFF, sizeof(info));
    info.magic          = OTA_MAGIC_VALID;
    info.status         = OTA_STATUS_PENDING;
    info.new_fw_size    = actual_size;
    info.new_fw_version = new_version;
    info.retry_count    = 0;
    /* cur_version 由 BL 升级成功后写入 (此处保持 0xFFFFFFFF) */

    if(ota_info_write(&info) != 0)
    {
        printf_debug("[OTA] info write fail\r\n");
        ota_report_status(OTA_ST_FAILED);
        EC801E_PublishUpgradeReply(OTA_REPLY_FAILED);
        EC801E_SetPSM(1);
        trea_delay_ms(1000);
        NVIC_SystemReset();
        return;
    }
    printf_debug("[OTA] InfoBlock written, restarting...\r\n");

    /* 8. 重启 → 进入 Bootloader 完成擦写 */
    ota_report_status(OTA_ST_REBOOT);
    trea_delay_ms(500);
    NVIC_SystemReset();
}

/* ==================== 启动回执 ==================== */
void ota_app_report_boot_status(void)
{
    const volatile OtaInfo_t *info = OTA_INFO_PTR;

    if(info->magic != OTA_MAGIC_VALID)
        return;

    switch(info->status)
    {
        case OTA_STATUS_SUCCESS:
            printf_debug("[OTA] boot SUCCESS report (ver %u → %u)\r\n",
                         (unsigned int)info->cur_version,
                         (unsigned int)info->new_fw_version);
            EC801E_PublishUpgradeReply(OTA_REPLY_SUCCESS);
            trea_delay_ms(500);
            ota_report_status(OTA_ST_SUCCESS);
            ota_info_clear();
            break;

        case OTA_STATUS_FAILED:
            printf_debug("[OTA] boot FAILED report (retry=%u)\r\n",
                         (unsigned int)info->retry_count);
            EC801E_PublishUpgradeReply(OTA_REPLY_FAILED);
            trea_delay_ms(300);
            ota_report_status(OTA_ST_FAILED);
            ota_info_clear();
            break;

        default:
            /* PENDING/FLASHING/IDLE 都不主动上报 */
            break;
    }
}

/* ==================== STOP 上报 ==================== */
void ota_app_report_stop(void)
{
    EC801E_PublishUpgradeReply(OTA_REPLY_STOP);
}
