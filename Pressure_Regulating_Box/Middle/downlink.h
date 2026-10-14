#ifndef DOWNLINK_H
#define DOWNLINK_H

#include <stdint.h>

// ==================== MQTT下发消息处理 ====================
// 排空并处理缓存的MQTT下发消息, 返回处理的消息数
// 供 do_downlink / do_upload / do_wakeup / do_button_short_press 调用
uint8_t drain_pending_downlink(void);

// 处理一条已读取出来的下行 payload (解析+分发+回执)
// 常规指令链路复用入口: 供静态测漏任务在任务期间轮询到非测漏指令时调用
void downlink_handle_raw(const char *raw);

// 上报相关接口已移至 report.h (report_publish_*)

#endif /* DOWNLINK_H */
