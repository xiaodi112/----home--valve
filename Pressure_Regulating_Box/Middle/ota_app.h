/**
 * @file  ota_app.h
 * @brief App 端 OTA: 解析 upgrade/set 下发, HTTP 下载到缓存区, 写 InfoBlock, 重启
 */
#ifndef OTA_APP_H
#define OTA_APP_H

#include <stdint.h>
#include "ota_info.h"

/* 解析 upgrade/set 下发的 JSON payload, 触发完整升级流程
 * payload 形如:
 *   {"version":2,"downloadUrl":"http://host/path/firmware.bin"}
 *
 * 流程:
 *   1. 解析 version + downloadUrl
 *   2. 立即上报 mqttUpgradeReply=2 (REPLY 升级中)
 *   3. 校验 version > APP_FW_VERSION_NUM, 否则上报 FAILED 退出
 *   4. EC801E_SetPSM(0) 保持 4G 在线
 *   5. 擦除 OTA_CACHE 区
 *   6. 通过 EC801E_HttpDownloadToFlash() 下载 bin 到 OTA_CACHE_ADDR
 *   7. 写 OTA InfoBlock (status=PENDING, size, version)
 *   8. NVIC_SystemReset() (重启进入 Bootloader)
 *
 * 失败路径: 上报 FAILED, 不写 InfoBlock, 不重启 (App 继续运行旧固件)
 */
void ota_app_handle_upgrade_cmd(const char *payload);

/* 写入 OTA InfoBlock (BLE OTA 和 4G OTA 共用) */
int ota_info_write(const OtaInfo_t *info);

/* App 启动时回执升级结果 (在 EC801E 联网+MQTT 就绪后调用)
 *   读 InfoBlock:
 *     status=SUCCESS → 上报 mqttUpgradeReply=3, 清 status (cur_version 保留)
 *     status=FAILED  → 上报 mqttUpgradeReply=4, 清 status, retry=0
 *     其他 → 不处理
 */
void ota_app_report_boot_status(void);

/* 用户主动放弃升级 (如低电压保护): 上报 STOP=5 */
void ota_app_report_stop(void);

/* OTA 专用看门狗控制 */
void ota_wdt_enable(void);
void ota_wdt_feed(void);

#endif /* OTA_APP_H */
