/**
 * @file leak_task.c
 * @brief 静态测漏任务实现 (平台 leak/set → 采集 → 分包上报 → 清除)
 *
 * 分层: Middle 层业务模块。向下只用驱动接口(ec800e/PT_304_35/valve/leak_store),
 *       不直接操作寄存器; 与常规采集/上报/报警链路完全解耦。
 *
 * 执行时序 (严格顺序):
 *   1) 收到 type=1 并校验参数 → 回 taskStatus=1
 *   2) 采集阶段: t=0 起每 samplingFreq 秒采 1 条, 逐条写入测漏暂存区
 *      (传感器保持供电、MCU 不休眠、4G 保持 QSCLK 在线)
 *   3) 上报阶段: 从暂存区顺序读, 每 5 条一包发到镜像 reply topic, 包间 100ms
 *   4) 收尾: 回 taskStatus=3 → 清空整个暂存区(不论成败) → 补执行阀门指令 → 回睡
 */

#include "leak_task.h"
#include "leak_store.h"
#include "ec800e.h"
#include "downlink.h"
#include "app_config.h"
#include "eeprom_config.h"
#include "eeprom_history.h"
#include "PT_304_35.h"
#include "valve.h"
#include "main.h"
#include "usart.h"
#include "delay.h"
#include "tim.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

/* ==================== 可调参数 ==================== */
#define LEAK_PACKET_RECORDS        5u        /* 每包数据条数 */
#define LEAK_PACKET_GAP_MS         100u      /* 包间最小间隔(ms); 实际=max(此值, AT往返) */
#define LEAK_WAIT_SLICE_MS         200u      /* 采集等待分片: 喂狗/轮询粒度 */
#define LEAK_POLL_INTERVAL_S       10u       /* 采集期间下行轮询间隔(s) */
#define LEAK_VALVE_QUEUE_DEPTH     4u        /* 任务期间阀门指令缓存深度 */
#define LEAK_RESAMPLE_RETRY        1u        /* 单条采样失败重试次数 */
/* failReason 取值口径: 0=字符串(成功""/失败"1"/"2", 与协议示例一致)
 *                     1=数字  (成功0 / 失败1 / 失败2) */
#define LEAK_FAILREASON_AS_CODE    0

/* ==================== topic 归属 ==================== */
#define LEAK_TOPIC_NONE            0u
#define LEAK_TOPIC_LEAK            1u   /* /.../leak/set    → /.../leak/reply  */
#define LEAK_TOPIC_MONITOR         2u   /* /.../monitor/get → /.../monitor/post */

#define LEAK_SEC_PER_DAY           86400u

/* ==================== 任务上下文 ==================== */
typedef struct {
    uint8_t  pending;                       /* 1=任务已接收待执行 */
    uint8_t  topic_kind;                    /* 回复 topic 归属 */
    char     taskNumber[LEAK_TASK_NUMBER_MAX];
    uint16_t samplingFreq;                  /* s */
    uint16_t collectionDuration;            /* s */
    uint16_t totalSamples;                  /* floor(D/f)+1 (已校验 <= 暂存区容量) */
} LeakTaskCtx_t;

typedef struct {
    uint8_t  used;
    uint8_t  valveCommand;                  /* 1=开阀 2=关阀 */
    uint8_t  topic_kind;
    char     taskNumber[LEAK_TASK_NUMBER_MAX];
} LeakValveCmd_t;

/* ==================== 模块静态数据 (主循环单线程调用, 无需加锁) ==================== */
static LeakTaskCtx_t s_task;
static LeakValveCmd_t s_valve_q[LEAK_VALVE_QUEUE_DEPTH];

/* 采样数据: 进气端(s_dev1) / 出气端(s_dev2)
 * 用模块私有缓冲, 不写 main.c 的 g_sensor, 避免干扰常规采集/实时模式 */
static PT304D_Data s_dev1;
static PT304D_Data s_dev2;
/* 单条采样失败时沿用上一条有效值: 保证上报条数与平台期望严格一致 */
static float   s_hold[4];
static uint8_t s_hold_valid = 0;

/* 上报 JSON 缓冲: 5 条 x (时间20B + 4字段) 约 700B, 留足余量 */
static char     s_json[1024];
static uint16_t s_json_pos = 0;
static uint8_t  s_json_full = 0;

/* 下行轮询缓冲 (静态分配, 避免深调用栈占用) */
static char s_rx_topic[MQTT_TOPIC_MAX];
static char s_rx_payload[1024];     /* 与 drain_pending_downlink 的 dl_buf 等长: 常规指令可能较长(OTA URL等) */

/* ==================== JSON 取值 (本功能专用最小实现) ==================== */
/* 取字符串字段: {"key":"value"} */
static int json_get_str(const char *buf, const char *key, char *out, uint16_t out_size)
{
    char pat[40];
    const char *p;
    const char *q;
    uint16_t len;

    if(buf == NULL || out == NULL || out_size == 0) return 0;
    out[0] = '\0';

    snprintf(pat, sizeof(pat), "\"%s\"", key);
    p = strstr(buf, pat);
    if(p == NULL) return 0;
    p += strlen(pat);
    while(*p == ' ' || *p == '\t' || *p == ':') p++;   /* 跳过冒号与空白 */
    if(*p != '"') return 0;
    p++;
    q = strchr(p, '"');
    if(q == NULL) return 0;

    len = (uint16_t)(q - p);
    if(len >= out_size) len = (uint16_t)(out_size - 1);
    memcpy(out, p, len);
    out[len] = '\0';
    return (len > 0) ? 1 : 0;
}

/* 取整数字段: {"key":123} */
static int json_get_long(const char *buf, const char *key, long *out)
{
    char pat[40];
    const char *p;

    if(buf == NULL || out == NULL) return 0;
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    p = strstr(buf, pat);
    if(p == NULL) return 0;
    p += strlen(pat);
    while(*p == ' ' || *p == '\t' || *p == ':') p++;
    if(*p != '-' && (*p < '0' || *p > '9')) return 0;

    *out = strtol(p, NULL, 10);
    return 1;
}

/* ==================== JSON 组装 (位置式追加 + 溢出保护) ==================== */
static void jb_reset(void)
{
    s_json_pos = 0;
    s_json_full = 0;
    s_json[0] = '\0';
}

static void jb_put(const char *fmt, ...)
{
    va_list ap;
    int n;
    uint16_t room;

    if(s_json_full) return;
    if(s_json_pos + 1 >= sizeof(s_json)) { s_json_full = 1; return; }

    room = (uint16_t)(sizeof(s_json) - 1 - s_json_pos);
    va_start(ap, fmt);
    n = vsnprintf(s_json + s_json_pos, room, fmt, ap);
    va_end(ap);

    if(n < 0) { s_json_full = 1; return; }
    if((uint16_t)n >= room)              /* 截断: 标记失败, 不发出半截报文 */
    {
        s_json_pos = (uint16_t)(sizeof(s_json) - 1);
        s_json_full = 1;
        return;
    }
    s_json_pos = (uint16_t)(s_json_pos + (uint16_t)n);
}

/* ==================== topic 处理 ==================== */
static uint8_t leak_topic_kind(const char *topic)
{
    if(topic == NULL) return LEAK_TOPIC_NONE;
    if(strstr(topic, "/leak/") != NULL)    return LEAK_TOPIC_LEAK;
    if(strstr(topic, "/monitor/") != NULL) return LEAK_TOPIC_MONITOR;
    return LEAK_TOPIC_NONE;
}

/* 回复 topic 镜像下发 topic: leak/set→leak/reply, monitor/get→monitor/post */
static void leak_reply_topic(uint8_t kind, char *out, uint16_t out_size)
{
    if(kind == LEAK_TOPIC_MONITOR)
        EC801E_DeriveTopic("/monitor/post", out, out_size);
    else
        EC801E_DeriveTopic("/leak/reply", out, out_size);
}

/* ==================== 回执发送 ==================== */
/* 发布带优先级的报文:
 * 产品策略规定"定时开关阀"最高优先级 —— 4G 发送被其抢占时先执行定时阀再重发一次;
 * 同时必须消费 g_timing_pending, 否则该标志残留会让后续所有 AT 指令返回 EC_PREEMPT。 */
static EC_Status_t leak_publish_prio(const char *topic, const char *json)
{
    EC_Status_t pr = EC801E_PublishOnTopic(topic, json);

    if(pr == EC_PREEMPT)
    {
        printf_debug("[LEAK] publish preempted by timing valve -> run & resend\r\n");
        vtiming_check_now();
        swdt_feed();
        pr = EC801E_PublishOnTopic(topic, json);
    }
    return pr;
}

static void leak_send_status(uint8_t kind, const char *taskNumber, uint16_t taskStatus)
{
    char topic[MQTT_TOPIC_MAX];

    leak_reply_topic(kind, topic, sizeof(topic));
    jb_reset();
    jb_put("{\"taskStatus\":%u,\"taskNumber\":\"%s\"}",
           (unsigned)taskStatus, taskNumber);
    if(s_json_full) { printf_debug("[LEAK] status json overflow\r\n"); return; }

    leak_publish_prio(topic, s_json);
    printf_debug("[LEAK] reply taskStatus=%u task=%s\r\n",
                 (unsigned)taskStatus, taskNumber);
}

/* 阀门回执: fail_code 0=成功, 1=动作超时未到位, 2=阀门异常 */
static void leak_send_valve_reply(uint8_t kind, const char *taskNumber,
                                  uint8_t cmd_status, uint8_t fail_code)
{
    char topic[MQTT_TOPIC_MAX];
    char fail_str[4];

    leak_reply_topic(kind, topic, sizeof(topic));
    jb_reset();

#if LEAK_FAILREASON_AS_CODE
    jb_put("{\"taskStatus\":5,\"taskNumber\":\"%s\",\"valveCmdStatus\":%u,\"failReason\":%u}",
           taskNumber, (unsigned)cmd_status, (unsigned)fail_code);
#else
    if(cmd_status == 1) fail_str[0] = '\0';
    else                snprintf(fail_str, sizeof(fail_str), "%u", (unsigned)fail_code);
    jb_put("{\"taskStatus\":5,\"taskNumber\":\"%s\",\"valveCmdStatus\":%u,\"failReason\":\"%s\"}",
           taskNumber, (unsigned)cmd_status, fail_str);
#endif

    if(s_json_full) { printf_debug("[LEAK] valve json overflow\r\n"); return; }
    leak_publish_prio(topic, s_json);
    printf_debug("[LEAK] valve reply status=%u fail=%u task=%s\r\n",
                 (unsigned)cmd_status, (unsigned)fail_code, taskNumber);
}

/* ==================== 阀门动作 (与平台 valve 指令同语义) ==================== */
static void leak_exec_valve_cmd(uint8_t kind, const char *taskNumber, uint8_t valveCommand)
{
    const char *result;
    const char *action;
    uint8_t cmd_status;
    uint8_t fail_code;

    if(valveCommand == 2)      action = "close";
    else if(valveCommand == 1) action = "open";
    else
    {
        printf_debug("[LEAK] bad valveCommand=%u\r\n", (unsigned)valveCommand);
        leak_send_valve_reply(kind, taskNumber, 2, 2);
        return;
    }

    result = valve_execute(action, 1);      /* 内部含 motor_pos_init + 掉电位置校验 */

    if(strcmp(result, "valve-timeout") == 0)  { cmd_status = 2; fail_code = 1; }
    else if(strcmp(result, "valve-err") == 0) { cmd_status = 2; fail_code = 2; }
    else                                      { cmd_status = 1; fail_code = 0; }

    /* 平台/用户手动动阀 = 覆盖报警闭锁 (与既有 valve 指令路径保持一致) */
    if(g_valve_alarm_closed)
    {
        g_valve_alarm_closed = 0;
        g_valve_pre_alarm_order = g_valve_pos_state;
        config_save_ext();
    }
    if(g_ch4_alarm_closed)
    {
        g_ch4_alarm_closed = 0;
        config_save_ext();
    }
    g_alarm_recover_reported = 0;

    leak_send_valve_reply(kind, taskNumber, cmd_status, fail_code);
}

/* ==================== 阀门指令队列 (任务期间缓存, 结束后补执行) ==================== */
static void valve_queue_push(const char *taskNumber, uint8_t valveCommand, uint8_t kind)
{
    uint8_t i;

    if(valveCommand != 1 && valveCommand != 2)
    {
        printf_debug("[LEAK] bad valveCommand=%u, ignored\r\n", (unsigned)valveCommand);
        return;
    }

    for(i = 0; i < LEAK_VALVE_QUEUE_DEPTH; i++)      /* 找空位 */
    {
        if(!s_valve_q[i].used) break;
    }
    if(i >= LEAK_VALVE_QUEUE_DEPTH)                  /* 满: 丢弃最旧, 保留最近指令 */
    {
        printf_debug("[LEAK] valve queue full -> drop oldest\r\n");
        for(i = 1; i < LEAK_VALVE_QUEUE_DEPTH; i++) s_valve_q[i - 1] = s_valve_q[i];
        i = LEAK_VALVE_QUEUE_DEPTH - 1;
    }

    memset(&s_valve_q[i], 0, sizeof(s_valve_q[i]));
    s_valve_q[i].used = 1;
    s_valve_q[i].valveCommand = valveCommand;
    s_valve_q[i].topic_kind = kind;
    if(taskNumber != NULL)
        strncpy(s_valve_q[i].taskNumber, taskNumber, LEAK_TASK_NUMBER_MAX - 1);

    printf_debug("[LEAK] valve cmd queued (cmd=%u) - defer until task end\r\n",
                 (unsigned)valveCommand);
}

static void valve_queue_run_all(void)
{
    uint8_t i;

    for(i = 0; i < LEAK_VALVE_QUEUE_DEPTH; i++)
    {
        if(!s_valve_q[i].used) continue;
        printf_debug("[LEAK] deferred valve cmd %u execute\r\n",
                     (unsigned)s_valve_q[i].valveCommand);
        leak_exec_valve_cmd(s_valve_q[i].topic_kind,
                            s_valve_q[i].taskNumber,
                            s_valve_q[i].valveCommand);
        s_valve_q[i].used = 0;
        swdt_feed();
    }
}

/* ==================== 下行处理 ==================== */
/* allow_queue=0: 非任务运行期, 阀门指令直接执行
 * allow_queue=1: 任务运行期, type=2 入队(不打断采集), type=1 忽略 */
static void leak_handle_payload(uint8_t kind, const char *payload, uint8_t allow_queue)
{
    long ltype = 0;
    long lval = 0;
    char taskNumber[LEAK_TASK_NUMBER_MAX];
    uint8_t type;

    if(payload == NULL) return;
    if(!json_get_long(payload, "type", &ltype)) return;   /* 非本协议格式: 交给原链路 */
    type = (uint8_t)ltype;

    if(!json_get_str(payload, "taskNumber", taskNumber, sizeof(taskNumber)))
        taskNumber[0] = '\0';

    if(type == 2)
    {
        if(!json_get_long(payload, "valveCommand", &lval))
        {
            printf_debug("[LEAK] type=2 without valveCommand\r\n");
            return;
        }
        if(allow_queue) valve_queue_push(taskNumber, (uint8_t)lval, kind);
        else            leak_exec_valve_cmd(kind, taskNumber, (uint8_t)lval);
        return;
    }

    if(type != 1) return;                                 /* 未定义类型: 忽略 */

    /* ---- type=1: 任务下发 ---- */
    if(s_task.pending)
    {
        printf_debug("[LEAK] task busy, new task ignored (%s)\r\n", taskNumber);
        return;
    }

#if LEAK_STORE_SUPPORTED
    {
        long lfreq = 0;
        long ldur  = 0;
        uint32_t total;

        if(!json_get_long(payload, "samplingFreq", &lfreq) ||
           !json_get_long(payload, "collectionDuration", &ldur))
        {
            printf_debug("[LEAK] task missing params, ignored\r\n");
            return;
        }
        if(lfreq <= 0 || ldur <= 0 || taskNumber[0] == '\0')
        {
            printf_debug("[LEAK] task invalid params, ignored\r\n");
            return;
        }

        /* 采集条数 = floor(D/f) + 1 (t=0 立即采第1条) */
        total = (uint32_t)ldur / (uint32_t)lfreq + 1u;
        if(total > LEAK_STORE_MAX_RECORDS)
        {
            /* 超限: 不采集、不回执(平台侧超时), 仅调试口告警 */
            printf_debug("[LEAK] task OVER CAPACITY: need %u > %d, NO REPLY\r\n",
                         (unsigned)total, LEAK_STORE_MAX_RECORDS);
            return;
        }

        memset(&s_task, 0, sizeof(s_task));
        s_task.topic_kind = kind;
        strncpy(s_task.taskNumber, taskNumber, LEAK_TASK_NUMBER_MAX - 1);
        s_task.samplingFreq = (uint16_t)lfreq;
        s_task.collectionDuration = (uint16_t)ldur;
        s_task.totalSamples = (uint16_t)total;
        s_task.pending = 1;

        printf_debug("[LEAK] task accepted: freq=%us dur=%us samples=%u\r\n",
                     (unsigned)s_task.samplingFreq, (unsigned)s_task.collectionDuration,
                     (unsigned)s_task.totalSamples);
    }
#else
    printf_debug("[LEAK] task ignored: not supported by this build\r\n");
#endif
}

/* 读取一条下行消息: 属于本功能→按归属处理; 否则交回原指令链路 */
static void leak_poll_downlink(uint8_t allow_queue)
{
    uint8_t flags[5];
    uint8_t i;

    /* 槽位化: 先查 5 个槽位标志, 再逐槽读取(与 drain_pending_downlink 同一策略) */
    if(EC801E_QueryRecvSlots(flags) != EC_OK)
        return;

    for(i = 0; i < 5; i++)
    {
        uint8_t kind;

        if(!flags[i]) continue;

        if(EC801E_ReadDownlinkById(i, s_rx_topic, sizeof(s_rx_topic),
                                   s_rx_payload, sizeof(s_rx_payload)) != EC_OK)
            continue;

        kind = leak_topic_kind(s_rx_topic);
        if(kind != LEAK_TOPIC_NONE)
        {
            leak_handle_payload(kind, s_rx_payload, allow_queue);
        }
        else
        {
            /* 测漏期间不接受 OTA 升级(HTTP 下载大阻塞会毁掉采样节奏), 其余指令保持原行为 */
            if(strstr(s_rx_payload, "downloadUrl") != NULL)
                printf_debug("[LEAK] OTA cmd ignored during leak task\r\n");
            else
                downlink_handle_raw(s_rx_payload);
        }
    }
}

/* ==================== RTC 秒对齐 ==================== */
static uint32_t leak_sec_of_day(void)
{
    uint32_t hhmmss = rtc_get_hhmmss();
    return (hhmmss / 10000u) * 3600u
         + ((hhmmss / 100u) % 100u) * 60u
         + (hhmmss % 100u);
}

static uint32_t leak_elapsed(uint32_t t0)
{
    uint32_t now = leak_sec_of_day();
    return (now >= t0) ? (now - t0) : (now + LEAK_SEC_PER_DAY - t0);
}

/* ==================== 单条采样 ==================== */
static uint8_t leak_sample_once(void)
{
    uint8_t ok;

    PT304D_PowerHold(1);            /* 幂等: 若被外部断电则重新上电并稳定等待 */
    PT304D_CollectAllHeld((g_sensor_count >= 2) ? &s_dev2 : NULL, &s_dev1);

    ok = (s_dev1.err == PT304D_OK) ? 1 : 0;
    if(g_sensor_count >= 2)
        ok = (uint8_t)(ok && (s_dev2.err == PT304D_OK));
    return ok;
}

static void leak_push_sample(void)
{
    uint8_t attempt;
    uint8_t ok = 0;

    for(attempt = 0; attempt <= LEAK_RESAMPLE_RETRY; attempt++)
    {
        ok = leak_sample_once();
        if(ok) break;
        printf_debug("[LEAK] sample err (S2=%d S1=%d) retry %d\r\n",
                     s_dev1.err, s_dev2.err, attempt);
    }

    if(ok)
    {
        s_hold[0] = s_dev1.pressure;
        s_hold[1] = s_dev1.temperature;
        s_hold[2] = (g_sensor_count >= 2) ? s_dev2.pressure    : 0.0f;
        s_hold[3] = (g_sensor_count >= 2) ? s_dev2.temperature : 0.0f;
        s_hold_valid = 1;
    }
    else
    {
        /* 读失败: 沿用上一条有效值, 保证上报条数与平台期望严格一致 */
        if(!s_hold_valid)
        {
            s_hold[0] = 0.0f; s_hold[1] = 0.0f; s_hold[2] = 0.0f; s_hold[3] = 0.0f;
        }
        printf_debug("[LEAK] sample failed -> hold previous value\r\n");
    }

    if(leak_store_push(s_hold[0], s_hold[1], s_hold[2], s_hold[3]) != 0)
        printf_debug("[LEAK] store push failed at idx=%u\r\n", (unsigned)leak_store_count());
}

/* ==================== 数据分包上报 ==================== */
static uint8_t leak_publish_pack(uint8_t kind, const char *taskNumber,
                                 uint16_t first, uint16_t count)
{
    char topic[MQTT_TOPIC_MAX];
    char ts[20];
    LeakRecord_t rec;
    uint16_t i;

    leak_reply_topic(kind, topic, sizeof(topic));
    jb_reset();
    jb_put("{\"taskStatus\":2,\"taskNumber\":\"%s\",\"taskData\":[", taskNumber);

    for(i = 0; i < count; i++)
    {
        if(leak_store_read((uint16_t)(first + i), &rec) != 0)
        {
            printf_debug("[LEAK] record read failed idx=%u\r\n", (unsigned)(first + i));
            return 1;
        }
        leak_store_ts_to_str(rec.ts_bcd, ts);

        if(g_sensor_count >= 2)
        {
            jb_put("%s{\"collationTime\":\"%s\",\"temp\":%.2f,\"pressure\":%.2f,"
                   "\"temp2\":%.2f,\"pressure2\":%.2f}",
                   (i == 0) ? "" : ",",
                   ts, rec.temperature, rec.pressure,
                   rec.temperature2, rec.pressure2);
        }
        else
        {
            jb_put("%s{\"collationTime\":\"%s\",\"temp\":%.2f,\"pressure\":%.2f}",
                   (i == 0) ? "" : ",",
                   ts, rec.temperature, rec.pressure);
        }

        if(s_json_full)
        {
            printf_debug("[LEAK] packet json overflow (first=%u n=%u)\r\n",
                         (unsigned)first, (unsigned)count);
            return 1;
        }
    }
    jb_put("]}");
    if(s_json_full)
    {
        printf_debug("[LEAK] packet json overflow (tail)\r\n");
        return 1;
    }

    /* 模组已在上报阶段开始前唤醒并保持唤醒(见 leak_task_execute), 此处直接发;
     * 单包失败时确认连接后重发一次 */
    if(leak_publish_prio(topic, s_json) != EC_OK)
    {
        printf_debug("[LEAK] pack failed, reconnect & retry (first=%u)\r\n",
                     (unsigned)first);
        if(EC801E_EnsureConnected() != EC_OK ||
           leak_publish_prio(topic, s_json) != EC_OK)
        {
            printf_debug("[LEAK] pack lost (first=%u n=%u)\r\n",
                         (unsigned)first, (unsigned)count);
            return 1;
        }
    }

    printf_debug("[LEAK] pack sent: idx %u..%u (%u rec, %u B)\r\n",
                 (unsigned)first, (unsigned)(first + count - 1u),
                 (unsigned)count, (unsigned)strlen(s_json));
    return 0;
}

/* ==================== 任务主体 ==================== */
static void leak_task_execute(void)
{
    uint32_t t0;
    uint32_t task_ms = 0;
    uint32_t next_poll_ms = (uint32_t)LEAK_POLL_INTERVAL_S * 1000u;
    uint16_t total = s_task.totalSamples;
    uint16_t k;
    uint16_t first;
    uint16_t count;
    uint16_t stored;

    printf_debug("[LEAK] === task start: %s (freq=%us dur=%us n=%u) ===\r\n",
                 s_task.taskNumber, (unsigned)s_task.samplingFreq,
                 (unsigned)s_task.collectionDuration, (unsigned)total);

    /* 任务期间 4G 必须在线: 确认连接(异常时冷启动重连一次) */
    if(EC801E_EnsureConnected() != EC_OK)
    {
        printf_debug("[LEAK] MQTT not ready, cold start...\r\n");
        (void)EC801E_InitAndConnect();
    }

    /* --- 阶段1: 已接收回执 --- */
    leak_send_status(s_task.topic_kind, s_task.taskNumber, 1);

    /* --- 阶段2: 采集 (传感器保持供电, MCU 不休眠) --- */
    leak_store_begin(s_task.taskNumber);
    PT304D_PowerHold(1);                    /* 任务期间不断电 */
    t0 = leak_sec_of_day();

    for(k = 0; k < total; k++)
    {
        uint32_t due = (uint32_t)k * (uint32_t)s_task.samplingFreq;

        /* RTC 秒对齐等待: 分片延时, 每片喂狗, 周期轮询下行 */
        while(leak_elapsed(t0) < due)
        {
            trea_delay_ms(LEAK_WAIT_SLICE_MS);
            task_ms += LEAK_WAIT_SLICE_MS;
            swdt_feed();
            if(task_ms >= next_poll_ms)
            {
                next_poll_ms += (uint32_t)LEAK_POLL_INTERVAL_S * 1000u;
                vtiming_check_now();        /* 定时开关阀最高优先级 + 消费挂起标志 */
                leak_poll_downlink(1);      /* type=2 入队, 不打断采集 */
                /* 轮询会调用 EC801E_Wakeup()(DTR 拉低=强制唤醒), 这里立即恢复 QSCLK:
                 * 采集期间模组只在轮询时短暂唤醒, 其余时间保持轻休眠省电 */
                EC801E_SetPSM(1);
            }
        }

        leak_push_sample();
        swdt_feed();
        printf_debug("[LEAK] sample %u/%u, stored=%u\r\n",
                     (unsigned)(k + 1u), (unsigned)total, (unsigned)leak_store_count());
    }

    stored = leak_store_count();
    leak_store_commit(stored);
    printf_debug("[LEAK] collect done: %u rec, start upload\r\n", (unsigned)stored);

    /* --- 阶段3: 上报 (每 5 条一包, 包间 100ms) --- */
    /* 唤醒模组并保持唤醒(DTR 低)直到整批发完: 避免每包 160ms 唤醒开销与首字节丢失 */
    EC801E_Wakeup();
    if(EC801E_EnsureConnected() != EC_OK)
        printf_debug("[LEAK] upload: MQTT ensure failed, per-pack retry enabled\r\n");

    for(first = 0; first < stored; first = (uint16_t)(first + LEAK_PACKET_RECORDS))
    {
        count = (uint16_t)(stored - first);
        if(count > LEAK_PACKET_RECORDS) count = (uint16_t)LEAK_PACKET_RECORDS;

        (void)leak_publish_pack(s_task.topic_kind, s_task.taskNumber, first, count);
        swdt_feed();

        if((uint16_t)(first + count) < stored)
            trea_delay_ms(LEAK_PACKET_GAP_MS);
    }

    /* --- 阶段4: 收尾 --- */
    leak_send_status(s_task.topic_kind, s_task.taskNumber, 3);
    leak_store_clear();                     /* 不论成败整块清除 */
    valve_queue_run_all();                  /* 任务期间缓存的阀门指令补执行 */
    PT304D_PowerHold(0);                    /* 传感器恢复断电 */
    EC801E_SetPSM(1);                       /* 4G 恢复轻休眠保持在线 */
    printf_debug("[LEAK] === task finished ===\r\n");
}

/* ==================== 对外接口 ==================== */
uint8_t leak_task_on_msg(const char *topic, const char *payload)
{
    uint8_t kind = leak_topic_kind(topic);

    if(kind == LEAK_TOPIC_NONE) return 0;               /* 非本功能 topic */

    leak_handle_payload(kind, payload, s_task.pending);
    return 1;
}

uint8_t leak_task_pending(void)
{
    return s_task.pending;
}

void leak_task_run(void)
{
    if(!s_task.pending)
    {
        app_state = STATE_SLEEP;
        return;
    }

    leak_task_execute();

    s_task.pending = 0;
    memset(&s_task, 0, sizeof(s_task));
    s_hold_valid = 0;
    app_state = STATE_SLEEP;
}
