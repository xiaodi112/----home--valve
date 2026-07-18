/**
 * @file  ota_info.h
 * @brief Bootloader 与 App 共享的 OTA 元数据 / Flash 分区定义
 *
 * Flash 256 KB 分区 (GD32L233RC, 4KB/page, 64 pages):
 *   0x08000000 - 0x08003FFF   16 KB   Bootloader   (pages 0-3)
 *   0x08004000 - 0x08004FFF    4 KB   OTA InfoBlock (page 4)
 *   0x08005000 - 0x08021FFF  116 KB   App 运行区   (pages 5-33)
 *   0x08022000 - 0x0803FFFF  120 KB   下载缓存区   (pages 34-63)
 *
 * 升级状态机 (与 FastBee 平台 mqttUpgradeReply 对齐):
 *   0 AWAIT   等待升级
 *   1 SEND    已发送
 *   2 REPLY   升级中  (App 收到 upgrade/set 后立即上报)
 *   3 SUCCESS 升级成功(新 App 启动后上报)
 *   4 FAILED  升级失败(下载失败/写 Flash 失败/重试超限)
 *   5 STOP    停止   (低电压等主动放弃)
 *   404 UNKNOWN
 */
#ifndef OTA_INFO_H
#define OTA_INFO_H

#include <stdint.h>

/* ==================== Flash 分区 ==================== */
#define OTA_FLASH_BASE              0x08000000U
#define OTA_FLASH_PAGE_SIZE         0x00001000U   /* 4 KB (GD32L233 实际页大小) */
#define OTA_FLASH_TOTAL_SIZE        0x00040000U   /* 256 KB */

#define OTA_BOOT_ADDR               0x08000000U
#define OTA_BOOT_SIZE               0x00004000U   /* 16 KB */

#define OTA_INFO_ADDR               0x08004000U
#define OTA_INFO_SIZE               0x00001000U   /* 4 KB (page 4) */

#define OTA_APP_ADDR                0x08005000U
#define OTA_APP_SIZE                0x0001D000U   /* 116 KB (pages 5-33) */
#define OTA_APP_END                 (OTA_APP_ADDR + OTA_APP_SIZE - 1U)

#define OTA_CACHE_ADDR              0x08022000U
#define OTA_CACHE_SIZE              0x0001E000U   /* 120 KB (pages 34-63) */
#define OTA_CACHE_END               (OTA_CACHE_ADDR + OTA_CACHE_SIZE - 1U)

/* ==================== Magic / 状态码 ==================== */
#define OTA_MAGIC_VALID             0xAABB1122U   /* InfoBlock 有效标记 */

#define OTA_STATUS_IDLE             0xFFFFFFFFU   /* 空闲 (擦除态) */
#define OTA_STATUS_PENDING          0x55AA55AAU   /* App 已下载完, 待 BL 刷写 */
#define OTA_STATUS_FLASHING         0xA5A5A5A5U   /* BL 正在拷贝 (断电恢复用) */
#define OTA_STATUS_SUCCESS          0x5A5A5A5AU   /* 升级成功 (待 App 上报) */
#define OTA_STATUS_FAILED           0xDEAD0001U   /* 升级失败 (待 App 上报) */

/* 重试上限: 超过则放弃, 标记 FAILED */
#define OTA_MAX_RETRY               3

/* ==================== FastBee 升级回执码 ==================== */
typedef enum {
    OTA_REPLY_AWAIT   = 0,
    OTA_REPLY_SEND    = 1,
    OTA_REPLY_REPLY   = 2,    /* 升级中 */
    OTA_REPLY_SUCCESS = 3,
    OTA_REPLY_FAILED  = 4,
    OTA_REPLY_STOP    = 5,
    OTA_REPLY_UNKNOWN = 404
} OtaReplyCode_t;

/* ==================== InfoBlock 结构 (4 KB 页, 头部用) ==================== */
typedef struct {
    uint32_t magic;             /* 0x00: OTA_MAGIC_VALID */
    uint32_t status;            /* 0x04: OTA_STATUS_xxx */
    uint32_t new_fw_size;       /* 0x08: 缓存区中新固件字节数 */
    uint32_t new_fw_version;    /* 0x0C: 平台下发的 version */
    uint32_t retry_count;       /* 0x10: 升级重试次数 */
    uint32_t cur_version;       /* 0x14: 当前 App 版本 (BL 升级成功后写入) */
    uint32_t reserved[58];      /* 0x18 - 0xFF, 保持为 0xFF */
} OtaInfo_t;

#define OTA_INFO_HEADER_BYTES       (sizeof(OtaInfo_t))  /* 实际占用, 远小于 1KB 页 */

/* 静态访问宏 (BL 与 App 都直接读 Flash) */
#define OTA_INFO_PTR                ((const volatile OtaInfo_t *)OTA_INFO_ADDR)

#endif /* OTA_INFO_H */
