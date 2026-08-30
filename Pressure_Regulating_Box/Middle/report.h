#ifndef REPORT_H
#define REPORT_H

#include <stdint.h>
#include "threshold.h"
#include "ch4_alarm.h"

/* ==================== 统一上报模块 (平台 JSON 上报封装) ==================== */
/* 统一采用 id/value/ts JSON 格式上报的通用模块, 供 main/downlink/驱动调用 */

/* 单字段状态上报: [{"id":..,"value":..,"ts":..,"remark":""}] */
void report_publish_field(const char *id, const char *value);

/* 上报传感器在线状态 (rtu_have/rtu_status 各 2) */
void report_publish_sensor_status(void);

/* 报警批量上报: 一条消息包含全部通道的温度+压力状态
 * (实测值+状态同消息: temp/pressure/temp_status/pressure_status; 双传感器再加 temp2/pressure2/...) */
void report_publish_alarm_batch(float last_p1, float last_p2);

/* 正常/恢复状态批量上报 (与报警同格式: 实测值 + 状态) */
void report_publish_status_all(float last_p1, float last_p2);

/* 电压报警/状态上报 (LY06+voltage, 单条消息, 温度/压力状态分开) */
void report_publish_ly06(uint8_t alarm, float voltage);

/* 汇总上报 (历史数据上报后调用): 在线状态+信号+压力温度+阀门+IP+电压+用户MQTT */
void report_publish_upload_summary(void);

/* 指令反馈: feedback=code */
void report_publish_feedback(int code);

/* CH4 status report: gas/mstate/mtemp/methane */
void report_publish_ch4_status(const CH4_Data_t *data, CH4_AlarmLevel_t level);

/* IP 字段上报 (IP.port 格式) + 可选 feedback */
void report_publish_ip_field(const char *field_name, const char *server, uint16_t port, int fb_code);

/* 查询信号信息 (CSQ + RSRP + SINR) */
void report_query_signal_info(char *csq, uint8_t csq_size,
                              char *rsrp, uint8_t rsrp_size,
                              char *sinr, uint8_t sinr_size);

#endif /* REPORT_H */
