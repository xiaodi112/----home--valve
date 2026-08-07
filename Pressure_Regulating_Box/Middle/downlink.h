#ifndef DOWNLINK_H
#define DOWNLINK_H

#include <stdint.h>

// ==================== MQTT下发消息处理 ====================
// 排空并处理缓存的MQTT下发消息, 返回处理的消息数
// 供 do_downlink / do_upload / do_wakeup / do_button_short_press 调用
uint8_t drain_pending_downlink(void);

// 上报传感器在线状态 (rtu_have/status × 2), 供 main.c 中状态变更上报
void publish_sensor_status(void);

// 上报汇总 (历史数据上报完成后调用): 传感器在线+信号+压力温度状态+阀门+IP+电压
void publish_upload_summary(void);

#endif /* DOWNLINK_H */
