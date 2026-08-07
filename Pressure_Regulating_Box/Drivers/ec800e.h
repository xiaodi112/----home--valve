#ifndef EC801E_H
#define EC801E_H

#include <stdio.h>
#include <stdint.h>
#include "gd32l23x.h"
#include "PT_304_35.h"
#include "sensor_buffer.h"
#include "threshold.h"

// ==================== 状态码 ====================
typedef enum {
    EC_OK = 0,
    EC_ERROR,
    EC_TIMEOUT
} EC_Status_t;

// ==================== 硬件引脚 ====================
#define EC_PWRKEY_PORT       GPIOB
#define EC_PWRKEY_PIN        GPIO_PIN_5

// ==================== 接口函数 ====================

void EC801E_PowerOn(void);
void EC801E_PowerOff(void);

EC_Status_t EC801E_InitNetwork(void);
EC_Status_t EC801E_ConnectMQTT(void);
EC_Status_t EC801E_DisconnectMQTT(void);
EC_Status_t EC801E_PublishSensorData(PT304D_Data *data, float voltage);
EC_Status_t EC801E_FullUpload(PT304D_Data *data, float voltage);
void EC801E_SetPSM(uint8_t mode);  // 0=关闭, 1=QSCLK轻睡眠, 3=PSM深度省电

// 从4G模块获取网络时间 (AT+CCLK?)
// 成功后自动调用 rtc_set_from_cclk() 设置RTC
// 返回: EC_OK=成功, EC_ERROR=失败
EC_Status_t EC801E_SyncTime(void);

// ==================== 持久连接 + 下发 ====================

// 首次初始化: 网络+MQTT连接+订阅+QSCLK (上电调用一次)
EC_Status_t EC801E_InitAndConnect(void);

// 订阅下发topic (从topic_up推导: /post → /get)
EC_Status_t EC801E_SubscribeTopic(void);

// 确保MQTT已连接 (断线自动重连)
EC_Status_t EC801E_EnsureConnected(void);

// 读取MQTT下发消息 (LPUART唤醒后调用)
// payload_buf: 输出JSON payload, buf_size: 缓冲区大小
// 返回: EC_OK=收到下发, EC_TIMEOUT=无数据
EC_Status_t EC801E_ReadDownlink(char *payload_buf, uint16_t buf_size);

// 唤醒模块 (QSCLK模式下首字节可能丢失,先发AT)
void EC801E_Wakeup(void);

// 发布单条历史记录 (用于批量上报)
EC_Status_t EC801E_PublishSingleRecord(const SensorRecord_t *record);

// 发布报警状态 (统一格式: id+value, 确认异常后立即调用)
EC_Status_t EC801E_PublishAlarmStatus(ThresholdResult_t result, float voltage);

// 发布下发响应结果 (单字段)
EC_Status_t EC801E_PublishResponse(const char *id, const char *value);

// 发布阀门状态 (合并 valve_control + valve_last_order 为一条JSON)
EC_Status_t EC801E_PublishValveStatus(const char *valve_result, int last_order);

// 发布任意已组装好的JSON字符串 (用于多字段回传: 阈值/窗口/周期等)
EC_Status_t EC801E_PublishRawJson(const char *json_str);

// 查询模块信息
EC_Status_t EC801E_QueryCSQ(char *out, uint16_t size);      // 信号强度 (CSQ 0~31)
EC_Status_t EC801E_QueryQCSQ(int *out_rsrp, int *out_sinr); // LTE信号质量 (RSRP dBm, SINR dB)
EC_Status_t EC801E_QueryCEREG(uint8_t *out_stat);           // 网络注册状态 (1=本地,5=漫游)
EC_Status_t EC801E_QueryICCID(char *out, uint16_t size);    // SIM卡号
EC_Status_t EC801E_QueryIMEI(char *out, uint16_t size);     // 模块IMEI

// ==================== OTA 升级 (FastBee 协议) ====================
// 订阅升级下发 topic: /<productId>/<deviceMac>/upgrade/set (从 topic_up 推导)
EC_Status_t EC801E_SubscribeUpgradeTopic(void);

// 上报升级状态到 /<productId>/<deviceMac>/upgrade/reply
// payload: {"mqttUpgradeReply":<code>}
// code 取自 OtaReplyCode_t (0..5/404)
EC_Status_t EC801E_PublishUpgradeReply(int reply_code);

// HTTP 下载固件到 Flash 缓存区
//   url           - 完整 URL (http://host/path/file.bin)
//   flash_addr    - 目标 Flash 起址 (OTA_CACHE_ADDR)
//   max_size      - 缓存区容量上限 (OTA_CACHE_SIZE)
//   actual_size   - 实际下载字节数 (出参)
// 返回: EC_OK 成功; EC_ERROR/EC_TIMEOUT 失败
EC_Status_t EC801E_HttpDownloadToFlash(const char *url,
                                       uint32_t flash_addr,
                                       uint32_t max_size,
                                       uint32_t *actual_size);

// 上报4G开关状态: on=1上线, on=0下线
void report_4g_power_state(uint8_t on);

// ==================== 用户端MQTT (id=1, 双实例) ====================

// 连接用户端MQTT服务器 (id=1)
// 前提: 网络已激活 (CGACT), g_user_mqtt 已配置
// 返回: EC_OK=连接成功, EC_ERROR=失败
EC_Status_t EC801E_UserMqtt_Connect(void);

// 断开用户端MQTT连接 (id=1)
EC_Status_t EC801E_UserMqtt_Disconnect(void);

// 发布用户端传感器数据 (LY格式JSON, 发布到id=1)
// 自动构建 {"param":[...]} 格式
EC_Status_t EC801E_UserMqtt_PublishSensorData(PT304D_Data *inlet, PT304D_Data *outlet, float voltage);

// 发布单条历史记录到用户端MQTT (LY格式, id=1)
// 使用 record 自身的 timestamp, 适用于批量历史上报
EC_Status_t EC801E_UserMqtt_PublishRecord(const SensorRecord_t *record);

// 查询用户端MQTT连接状态 (id=1)
// 返回: 1=已连接, 0=未连接/未配置
uint8_t EC801E_UserMqtt_IsConfigured(void);

#endif
