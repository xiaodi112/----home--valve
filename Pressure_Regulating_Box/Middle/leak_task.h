#ifndef LEAK_TASK_H
#define LEAK_TASK_H

#include <stdint.h>

/**
 * @file leak_task.h
 * @brief 静态测漏任务 (平台 leak/set 下发 → 采集 → 分包上报 → 清除)
 *
 * 协议 (详见 Docs/设备测漏mqtt.md):
 *   下发  /<productId>/<deviceNum>/leak/set
 *     type=1 任务: {"type":1,"taskNumber":..,"samplingFreq":秒,"collectionDuration":秒}
 *     type=2 阀门: {"type":2,"taskNumber":..,"valveCommand":1开/2关}
 *   上报  /<productId>/<deviceNum>/leak/reply (topic 镜像自下发 topic)
 *     taskStatus=1 已接收 / 2 数据包 / 3 任务完成 / 5 阀门回执
 *
 * 设计约束:
 *   - 独立链路: 不复用/不修改原有 function/get 指令表, 不干扰常规采集/上报/报警
 *   - 采集严格按平台数量: t=0 起每 samplingFreq 秒 1 条, N = floor(D/f)+1
 *   - 超限任务(条数>暂存区容量)不采集、不回执(平台侧超时), 仅调试口告警
 *   - 采完再发: 采集阶段全部结束才进入上报阶段; 每 5 条一包, 包间 100ms
 *   - 上报流程结束后整块丢弃数据(不论成败), 不做断点续传
 *   - 任务运行中收到阀门指令: 入队不打断采集, 任务结束后按序补执行并回执
 */

/**
 * 下行消息分流: topic 属于本功能时处理并返回 1; 否则返回 0 (交回原指令链路)
 * 由 drain_pending_downlink() 在解析前调用
 */
uint8_t leak_task_on_msg(const char *topic, const char *payload);

/** 是否有已接收待执行的任务 (主循环据此切换到 STATE_LEAK) */
uint8_t leak_task_pending(void);

/** 任务主体: 采集 → 分包上报 → 收尾清除 → 补执行阀门指令 (阻塞执行) */
void leak_task_run(void);

#endif /* LEAK_TASK_H */
