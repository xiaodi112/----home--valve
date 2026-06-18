#ifndef DOWNLINK_H
#define DOWNLINK_H

#include <stdint.h>

// ==================== MQTT下发消息处理 ====================
// 排空并处理缓存的MQTT下发消息, 返回处理的消息数
// 供 do_downlink / do_upload / do_wakeup / do_button_short_press 调用
uint8_t drain_pending_downlink(void);

#endif /* DOWNLINK_H */
