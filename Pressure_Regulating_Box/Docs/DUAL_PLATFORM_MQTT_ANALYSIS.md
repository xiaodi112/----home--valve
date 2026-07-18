# 双平�?MQTT 上报功能 �?可行性分析与实施方案

> **文档日期**: 2025-06-22  
> **状�?*: 分析阶段 (未修改代�?  
> **目标**: 在保持现有主/�?IP 故障切换不变的前提下，新增一个客�?IP 建立第二个独�?MQTT 连接，实现双平台并行上报

---

## 一、现有架构分�?
### 1.1 当前 MQTT 连接模型

```
主IP (server)  ──�?                  ├──�?同一�?MQTT Client (index=0) ──�?同一套凭�?副IP (server2) ──�?    �?串行故障切换, 非并行连�?```

- **�?�?IP 是同一平台的容�?*：共�?client index `0`、共用凭�?(`client_id`/`username`/`password`)、共�?topic
- 同一时刻只连接一�?IP，主 IP 连接失败才切换到�?IP

### 1.2 当前 AT 命令中的 Client Index

所�?MQTT AT 命令均硬编码使用 **client index `0`**�?
| AT 命令 | 代码位置 | 用�?|
|---------|---------|------|
| `AT+QMTOPEN=0,...` | `ec800e.c:286` | 打开 MQTT 连接 |
| `AT+QMTCONN=0,...` | `ec800e.c:299` | 登录 MQTT 服务�?|
| `AT+QMTCONN?` �?`+QMTCONN: 0,3` | `ec800e.c:311,571` | 查询连接状�?|
| `AT+QMTSUB=0,...` | `ec800e.c:489,815` | 订阅下发/升级 topic |
| `AT+QMTPUBEX=0,...` | `ec800e.c:657,686,709,729,839` | 发布数据 |
| `AT+QMTRECV=0` | `ec800e.c:521` | 读取下发消息 |
| `AT+QMTCFG=...,0,...` | `ec800e.c:602,608` | keepalive/recv mode |
| `AT+QMTDISC=0` / `AT+QMTCLOSE=0` | `ec800e.c:279,281` | 断开/关闭连接 |

### 1.3 当前配置结构�?(`MqttConfig_t`)

```c
// eeprom_config.h
typedef struct {
    char client_id[64];
    char username[64];
    char password[64];
    char server[64];         // 主IP
    char server2[64];        // 副IP (故障切换)
    char topic_up[128];
    uint16_t port;           // 主端�?    uint16_t port2;          // 副端�?} MqttConfig_t;
```

运行时全局变量：`g_mqtt` (单实�?

### 1.4 当前�?�?IP 切换逻辑

```c
// ec800e.c �?EC801E_ConnectMQTT()
EC_Status_t EC801E_ConnectMQTT(void)
{
    // 1. 先尝试主IP
    if(mqtt_connect_to(g_mqtt.server, g_mqtt.port) == EC_OK)
        return EC_OK;

    // 2. 主IP失败, 尝试副IP (如果已配�?
    if(g_mqtt.server2[0] != '\0')
    {
        if(mqtt_connect_to(g_mqtt.server2, g_mqtt.port2) == EC_OK)
            return EC_OK;
    }
    return EC_ERROR;
}
```

---

## 二、EC800E �?MQTT 客户端能�?
EC800E/EC801E 模块支持 **最�?6 个并�?MQTT 客户�?* (client index `0` ~ `5`)�?
每个客户端完全独立，可同时：
- 连接不同服务�?端口
- 使用不同凭证 (ClientID / Username / Password)
- 订阅不同 Topic
- 发布到不�?Topic

AT 命令格式统一通过第一个参�?`<client_idx>` 区分�?
```
AT+QMTOPEN=<idx>,<server>,<port>
AT+QMTCONN=<idx>,<clientid>,<user>,<pass>
AT+QMTSUB=<idx>,<msgid>,<topic>,<qos>
AT+QMTPUBEX=<idx>,<msgid>,<qos>,<retain>,<topic>,<len>
AT+QMTRECV=<idx>
AT+QMTCFG=<param>,<idx>,...
```

**结论：硬件和模块完全支持同时维护两个独立 MQTT 连接，当�?index `1` 完全空闲�?*

---

## 三、双平台目标架构

```
┌───────────────── 自有平台 (FastBee) ──────────────────�?�? Client Index: 0                                       �?�? 服务�? server / server2 (主副故障切换, 逻辑不变)       �?�? 凭证: client_id / username / password                  �?�? 上报 Topic: topic_up                                   �?�? 订阅 Topic: /function/get + /upgrade/set               �?�? 功能: 数据上报 + 下发控制 + OTA升级                     �?└────────────────────────────────────────────────────────�?
┌───────────────── 客户平台 (新增) ─────────────────────�?�? Client Index: 1                                       �?�? 服务�? server_b (�?IP, 无需�?IP)                    �?�? 凭证: client_id_b / username_b / password_b            �?�? 上报 Topic: topic_up_b                                 �?�? 订阅 Topic: (可�? 建议初期不订�?                      �?�? 功能: 数据上报 (只读镜像)                               �?└────────────────────────────────────────────────────────�?```

### 设计原则

1. **Client 0 (自有平台) 为主**，连�?上报失败不受 Client 1 影响
2. **Client 1 (客户平台) 为辅**，连接失败静默跳过，不阻塞主流程
3. **Client 1 仅上报数�?*，初期不处理下发，避免双平台控制冲突
4. **OTA 升级仅绑�?Client 0**，Client 1 不订�?upgrade topic

---

## 四、EEPROM 存储方案

### 4.1 当前 EEPROM 使用情况 (M24M01, 2048 字节)

| 区域 | 地址范围 | Magic | 大小 | 内容 |
|------|---------|-------|------|------|
| 1 �?间隔配置 | 0x000 - 0x01F | 0xA5 | 32 B | 采集/上报间隔 |
| 2 �?MQTT 配置 | 0x020 - 0x1AF | 0x5A | 400 B | ClientID/凭证/服务�?Topic |
| 3 �?压力阈�?| 0x1B0 - 0x1CF | 0xC0 | 32 B | 上下�?突变/滞回/心跳 |
| 4 �?温度阈�?| 0x1D0 - 0x1EF | 0xC1 | 32 B | 高温/低温/极�?|
| 5 �?电压检�?| 0x1F0 - 0x1FF | 0xC2 | 16 B | 高压/低压/保护 |
| 6 �?扩展配置 | 0x200 - 0x28F | 0xB7 | 144 B | 窗口/周期/阀�?副IP |
| 7 �?阀门延�?| 0x290 - 0x29F | 0xC3 | 16 B | 延时开�?时间 |
| **已用合计** | **0x000 - 0x29F** | �?| **672 B** | �?|
| **空闲** | **0x2A0 - 0x7FF** | �?| **1376 B** | **可用** |

### 4.2 新增区域 8 �?客户 MQTT 配置

建议起始地址 `0x2A0`，Magic `0xD0`�?
| 偏移 | 字段 | 大小 | 说明 |
|------|------|------|------|
| 0x2A0 | Magic | 1 B | 0xD0 = 有效 |
| 0x2A1 | enabled | 1 B | 0 = 禁用客户平台, 1 = 启用 |
| 0x2A2 | client_id_b | 64 B | 客户 MQTT ClientID |
| 0x2E2 | username_b | 64 B | 客户 MQTT 用户�?|
| 0x322 | password_b | 64 B | 客户 MQTT 密码 |
| 0x362 | server_b | 64 B | 客户 MQTT 服务�?IP |
| 0x3A2 | port_b | 2 B | 客户 MQTT 端口 |
| 0x3A4 | topic_up_b | 128 B | 客户上报 Topic |
| **合计** | �?| **388 B** | 截止 0x423 |

�?完全在空闲区域内，不影响现有配置�?
---

## 五、代码修改清�?
### 5.1 eeprom_config.h �?新增定义

```c
// 区域 8: 客户 MQTT 配置
#define EE_ADDR_MQTTB_MAGIC       0x2A0
#define EE_ADDR_MQTTB_ENABLED     0x2A1
#define EE_ADDR_MQTTB_CLIENT_ID   0x2A2
#define EE_ADDR_MQTTB_USERNAME    0x2E2
#define EE_ADDR_MQTTB_PASSWORD    0x322
#define EE_ADDR_MQTTB_SERVER      0x362
#define EE_ADDR_MQTTB_PORT        0x3A2
#define EE_ADDR_MQTTB_TOPIC_UP    0x3A4

#define EE_MAGIC_MQTTB            0xD0

// 运行时配�?typedef struct {
    uint8_t  enabled;               // 0=禁用, 1=启用
    char     client_id[MQTT_FIELD_MAX];
    char     username[MQTT_FIELD_MAX];
    char     password[MQTT_FIELD_MAX];
    char     server[MQTT_FIELD_MAX];
    char     topic_up[MQTT_TOPIC_MAX];
    uint16_t port;
} MqttConfigB_t;

extern MqttConfigB_t g_mqtt_b;
```

### 5.2 eeprom_config.c �?新增加载/保存

| 函数 | 说明 |
|------|------|
| `config_load_all()` | 追加加载区域 8 (客户 MQTT) |
| `config_save_mqtt_b()` | 新增：保存客�?MQTT 配置�?EEPROM |
| `config_parse_and_apply()` | 追加串口命令解析 (�?`set mqtt_b ...`) |
| `config_print_current()` | 追加打印客户 MQTT 配置 |

### 5.3 ec800e.c �?需修改/新增的函�?
#### 参数化改�?
�?`mqtt_connect_to()` 改为支持传入 client index�?
```c
// 改造前 (硬编�?index 0):
static EC_Status_t mqtt_connect_to(const char *server, uint16_t port);

// 改造后 (参数�?:
static EC_Status_t mqtt_connect_to_idx(uint8_t idx,
                                        const char *server, uint16_t port,
                                        const char *client_id,
                                        const char *username,
                                        const char *password);
```

#### 新增函数

| 函数 | Client Index | 说明 |
|------|-------------|------|
| `EC801E_ConnectMQTT_B()` | 1 | 连接客户平台 (�?server_b, 无副IP切换) |
| `EC801E_EnsureConnected_B()` | 1 | 检�?重连 client 1 |
| `EC801E_PublishSingleRecord_B()` | 1 | 客户平台数据上报 |
| `EC801E_PublishRawJson_B()` | 1 | 客户平台通用 JSON 上报 |
| `EC801E_PublishAlarm_B()` | 1 | 客户平台报警上报 |
| `EC801E_DisconnectMQTT_B()` | 1 | 断开客户连接 |

#### 修改函数

| 函数 | 修改内容 |
|------|---------|
| `EC801E_InitAndConnect()` | client 0 成功�? 追加 client 1 连接 (�?`g_mqtt_b.enabled`) |
| `EC801E_EnsureConnected()` | 仅检�?client 0 (不变), 但新�?`_B` 版本 |

### 5.4 main.c �?修改位置

| 状�?函数 | 修改内容 |
|-----------|---------|
| `STATE_INIT` | `InitAndConnect` 已内�?client 1 连接, 无需额外�?|
| `do_upload()` | 每条记录 publish �?client 0 �? 追加 publish �?client 1 |
| `do_realtime_check()` | 异常上报追加 client 1 publish |
| `do_sleep()` | LPUART/QSCLK 对所�?client 统一生效, 无需修改 |
| 报警上报 | 所�?`EC801E_PublishAlarm()` 调用点追�?`_B` 版本 |

### 5.5 downlink.c (可�? 初期不改)

- 客户平台初期 **不订阅下�?topic**，不需要修改下发处�?- 后续如需支持客户下发，再新增 `EC801E_ReadDownlink_B()` + 消息路由

### 5.6 app_config.h �?新增默认�?
```c
/* ---------- 客户 MQTT 默认�?---------- */
#define APP_DEFAULT_MQTTB_ENABLED     0           /* 默认禁用 */
#define APP_DEFAULT_MQTTB_CLIENT_ID   ""
#define APP_DEFAULT_MQTTB_SERVER      ""
#define APP_DEFAULT_MQTTB_PORT        1883
#define APP_DEFAULT_MQTTB_USER        ""
#define APP_DEFAULT_MQTTB_PASS        ""
#define APP_DEFAULT_MQTTB_TOPIC       ""
```

---

## 六、上报流程对�?
### 现有流程 (单平�?

```
do_upload()
  ├─ 窗口检�?  ├─ 建立4G连接 (PowerOn / EnsureConnected)
  ├─ while(缓冲区有数据)
  �?   └─ EC801E_PublishSingleRecord(rec)    // client 0
  ├─ 上报传感器状�?  ├─ RTC校准
  └─ 模式收尾 (等待下发 / PSM)
```

### 改造后流程 (双平�?

```
do_upload()
  ├─ 窗口检�?  ├─ 建立4G连接 (PowerOn / EnsureConnected)
  ├─ if(g_mqtt_b.enabled) EC801E_EnsureConnected_B()   // �?新增
  ├─ while(缓冲区有数据)
  �?   ├─ EC801E_PublishSingleRecord(rec)               // client 0 (自有平台)
  �?   └─ if(g_mqtt_b.enabled)
  �?        EC801E_PublishSingleRecord_B(rec)            // �?新增: client 1 (客户平台)
  ├─ 上报传感器状�?(双平�?
  ├─ RTC校准
  └─ 模式收尾
```

---

## 七、技术风险与应对

| # | 风险 | 影响 | 应对措施 |
|---|------|------|---------|
| 1 | **连接耗时增加** | cold start �?6-10 �?(QMTOPEN+QMTCONN) | client 1 连接超时设短 (10s); 失败不阻�?client 0 |
| 2 | **双�?keepalive** | QSCLK 下模块唤醒频率翻�? 功耗增�?| client 1 keepalive 设大 (�?600s); 或仅 short/dot 模式�?client 1 |
| 3 | **publish 耗时翻�?* | 每条记录发两�? 上报窗口延长 | 可接�?(每条 ~500ms); 批量上报场景注意总时�?|
| 4 | **client 1 服务器不可达** | 拖慢上报流程 | 设置独立失败标志 `mqtt_b_connected`, 失败后本周期跳过 |
| 5 | **双平台控制冲�?* | 两个平台同时下发阀门控�?| 初期 client 1 不订阅下�?topic, 仅上�?|
| 6 | **OTA 安全** | 客户平台不应触发 OTA | client 1 不订�?`/upgrade/set` |
| 7 | **RAM 增加** | 新增 `MqttConfigB_t` ~460 字节 | 32KB SRAM 完全够用 |
| 8 | **EEPROM 空间** | 新增区域 8 �?388 字节 | 剩余 1376 字节, 足够 |

---

## 八、串口配置命令规�?
### 新增命令

| 命令 | 参数 | 说明 |
|------|------|------|
| `set mqtt_b_enabled` | `0` / `1` | 启用/禁用客户平台 |
| `set mqtt_b_server` | `<IP>` | 客户 MQTT 服务�?IP |
| `set mqtt_b_port` | `<端口>` | 客户 MQTT 端口 |
| `set mqtt_b_clientid` | `<字符�?` | 客户 MQTT ClientID |
| `set mqtt_b_user` | `<字符�?` | 客户 MQTT 用户�?|
| `set mqtt_b_pass` | `<字符�?` | 客户 MQTT 密码 |
| `set mqtt_b_topic` | `<Topic路径>` | 客户上报 Topic |
| `get mqtt_b` | �?| 查看客户 MQTT 当前配置 |

### MQTT 下发配置 (可�?

后续可通过自有平台 (FastBee) MQTT 下发 JSON 远程配置客户 MQTT 参数�?
```json
[{"id":"mqtt_b_server","value":"192.168.1.100"},
 {"id":"mqtt_b_port","value":"1883"},
 {"id":"mqtt_b_clientid","value":"customer_device_001"},
 {"id":"mqtt_b_user","value":"admin"},
 {"id":"mqtt_b_pass","value":"password123"},
 {"id":"mqtt_b_topic","value":"/customer/device001/data"},
 {"id":"mqtt_b_enabled","value":"1"}]
```

---

## 九、实施步骤建�?
### Phase 1: 基础双连�?(最小可�?

1. `eeprom_config.h` �?新增 `MqttConfigB_t` 结构�?+ EEPROM 地址�?2. `eeprom_config.c` �?实现加载/保存 + 串口配置命令
3. `app_config.h` �?新增客户 MQTT 默认�?4. `ec800e.c` �?参数�?`mqtt_connect_to_idx()`，新�?`_B` 系列函数
5. `ec800e.h` �?新增函数声明
6. `main.c` �?`do_upload()` 追加 client 1 发布

### Phase 2: 完善容错

7. 新增 `mqtt_b_connected` 标志，client 1 失败时自动跳�?8. client 1 连接失败计数，连�?N 次失败后暂停，下个上报周期重�?9. 串口 `get mqtt_b` 打印客户平台连接状�?
### Phase 3: 可选扩�?
10. 客户平台下发订阅 (如需双向通信)
11. 客户平台独立 keepalive 配置
12. 自有平台远程配置客户 MQTT 参数 (通过 FastBee 下发)

---

## 十、文件变更汇�?
| 文件 | 改动类型 | 改动量估�?|
|------|---------|-----------|
| `app_config.h` | 新增默认值宏 | +10 �?|
| `eeprom_config.h` | 新增结构�?+ 地址�?+ extern | +30 �?|
| `eeprom_config.c` | 新增加载/保存/命令解析 | +80 �?|
| `ec800e.h` | 新增 `_B` 系列函数声明 | +15 �?|
| `ec800e.c` | 参数�?+ 新增 `_B` 函数 | +120 �?|
| `main.c` | `do_upload` / 报警 / 实时检查追�?client 1 | +40 �?|
| `downlink.c` | (Phase 3, 可�? | +30 �?|
| **合计** | �?| **~250 行新�?修改** |

---

## 十一、客户平台协议规�?(中燃 SCADA)

### 11.1 已确认连接参�?
| 参数 | �?|
|------|-----|
| **Client Index** | `1` (EC800E 第二�?MQTT) |
| **Server** | `da.95007.com` (域名, EC800E 自动 DNS 解析) |
| **Port** | `13000` |
| **ClientID** | `D54GRG29T5HB0` |
| **Username** | `hhdl` |
| **Password** | `EQ@hhdl2024nhjkb31` |
| **Topic** | `/api/v1/D54GRG29T5HB0/mqtt/dataPub` |
| **deviceKey** | `D54GRG29T5HB0` (= deviceIdentity, 同一�? |
| **下发订阅** | 初期不订�? 仅上�?|

### 11.2 AT 命令序列

```
AT+QMTCFG="keepalive",1,600
AT+QMTCFG="recv/mode",1,0,1
AT+QMTOPEN=1,"da.95007.com",13000
�?+QMTOPEN: 1,0
AT+QMTCONN=1,"D54GRG29T5HB0","hhdl","EQ@hhdl2024nhjkb31"
�?+QMTCONN: 1,0,0
AT+QMTPUBEX=1,0,0,0,"/api/v1/D54GRG29T5HB0/mqtt/dataPub",<len>
�?> <payload>
�?+QMTPUBEX: 1,0,0
```

### 11.3 JSON 上报格式

```json
{
  "param": [
    {"deviceKey":"D54GRG29T5HB0","key":"LY01","value":"120.50","ts":"2025-06-22 10:00:00"},
    {"deviceKey":"D54GRG29T5HB0","key":"LY02","value":"25.30","ts":"2025-06-22 10:00:00"},
    {"deviceKey":"D54GRG29T5HB0","key":"LY03","value":"3.60","ts":"2025-06-22 10:00:00"},
    {"deviceKey":"D54GRG29T5HB0","key":"LY04","value":"0","ts":"2025-06-22 10:00:00"},
    {"deviceKey":"D54GRG29T5HB0","key":"LY05","value":"22","ts":"2025-06-22 10:00:00"},
    {"deviceKey":"D54GRG29T5HB0","key":"LY06","value":"0","ts":"2025-06-22 10:00:00"},
    {"deviceKey":"D54GRG29T5HB0","key":"LY07","value":"360","ts":"2025-06-22 10:00:00"},
    {"deviceKey":"D54GRG29T5HB0","key":"LY08","value":"98.20","ts":"2025-06-22 10:00:00"},
    {"deviceKey":"D54GRG29T5HB0","key":"LY11","value":"24.80","ts":"2025-06-22 10:00:00"}
  ]
}
```

- 单条 payload �?**810 字节** (9 个变�?
- `s_payload` 需�?512 扩到 **1024 字节**

### 11.4 LY 变量映射�?
| Key | 单位 | 说明 | SensorRecord_t 字段 | 缓存方式 |
|-----|------|------|---------------------|---------|
| LY01 | kPa | 进口压力 | `pressure2` (PT304D #1, PC0/PC1) | �?历史缓存 |
| LY02 | �?| 进口温度 | `temperature2` (PT304D #1) | �?历史缓存 |
| LY03 | V | 电池电压 | `voltage` | �?历史缓存 |
| LY04 | �?| 压力探头故障 (0:正常; �?:故障) | �?| �?实时: `g_sensor.raw.err` |
| LY05 | �?| 信号强度 | �?| �?实时: `AT+CSQ` |
| LY06 | �?| 电池电压报警 (-1:未知; 0:正常; 1:报警) | �?| �?实时: voltage vs 阈�?|
| LY07 | �?| 传输周期 | �?| �?配置�? `g_interval` |
| LY08 | kPa | 出口压力 | `pressure` (PT304D #2, PB6/PB7) | �?历史缓存 |
| LY11 | �?| 出口温度 | `temperature` (PT304D #2) | �?历史缓存 |

> LY09 (出口压力2)、LY10 (进口压力2) �?当前硬件无对应传感器, 不上�?
### 11.5 上报规则

| 规则 | 要求 | 实现方案 |
|------|------|---------|
| 固定周期上传 | 默认 6 小时传输一�?| 沿用现有 `g_interval.upload_interval_min` |
| 历史数据 | 一次传�?6 条间�?1 小时的记�?| 沿用现有 `sensor_buf` 环形缓冲�?|
| 报警立即上报 | 阈值超限立即发�?| 沿用现有报警即发逻辑 |
| 单包 < 500 �?| 禁止分包 | 逐条发�?(每条 9 �?param), 远小�?500 �?|
| 帧间�?�?1s | 数据帧之间至�?1 �?| client 1 publish �?`delay(1000)` |

---

## 十二、数据缓存策�?
### 12.1 核心思路: 只缓存原始数�? 上报时实时格式化

```
采集阶段: SensorRecord_t 只存 5 �?float + timestamp �?44 字节/�?                �?上报阶段: 从缓存读历史�?+ 实时获取诊断�?+ 固定 JSON 模板 snprintf
                �?          共享 s_payload[1024] 格式化缓冲区, 逐条发�?```

### 12.2 SensorRecord_t 现状

```c
// sensor_buffer.h �?已有完整字段 (44 字节)
typedef struct {
    float pressure;       // PT304D #2 (PB6/PB7) �?LY08 出口压力
    float temperature;    // PT304D #2            �?LY11 出口温度
    float pressure2;      // PT304D #1 (PC0/PC1) �?LY01 进口压力
    float temperature2;   // PT304D #1            �?LY02 进口温度
    float voltage;        //                      �?LY03 电池电压
    char  timestamp[20];  //                      �?ts 字段
    uint8_t flags;
    uint8_t reserved[3];
} SensorRecord_t;         // 44 字节/�? 128 条环形缓�?```

�?**结构体无需修改** �?`pressure2` / `temperature2` 字段已存�?
### 12.3 待修�? push 代码未填充传感器2

当前 `main.c` �?3 �?push 点只填了 3 个�?

```c
// main.c 当前代码 (3 处相同模�?
record.pressure = g_sensor.raw.pressure;
record.temperature = g_sensor.raw.temperature;
record.voltage = g_sensor.voltage;
// ⚠️ 缺少:
// record.pressure2 = g_sensor.raw2.pressure;     // PT304D #1
// record.temperature2 = g_sensor.raw2.temperature; // PT304D #1
```

需在以�?3 处补充赋�?
1. `main.c:1116` �?正常采集存缓�?2. `main.c:1224` �?报警采集存缓�?3. `main.c:1783` �?首次启动存缓�?
每处�?**+2 �?*, 改动极小�?
### 12.4 内存开销

| 项目 | 大小 | 说明 |
|------|------|------|
| SensorRecord_t | 0 字节增量 | 字段已存�? 无需扩展 |
| `s_payload` 扩展 | +512 字节 | �?512B 扩到 1024B |
| `MqttConfigB_t` | ~460 字节 | 客户 MQTT 运行时配�?|
| **合计 RAM 增量** | **~1 KB** | 32KB SRAM 完全够用 |

---

## 十三、EEPROM 存储最终方�?(�?device_key)

| 偏移 | 字段 | 大小 | 本设备�?|
|------|------|------|---------|
| 0x2A0 | Magic | 1 B | `0xD0` |
| 0x2A1 | enabled | 1 B | `1` |
| 0x2A2 | client_id_b | 64 B | `"D54GRG29T5HB0"` |
| 0x2E2 | username_b | 64 B | `"hhdl"` |
| 0x322 | password_b | 64 B | `"EQ@hhdl2024nhjkb31"` |
| 0x362 | server_b | 64 B | `"da.95007.com"` |
| 0x3A2 | port_b | 2 B | `13000` |
| 0x3A4 | topic_up_b | 128 B | `"/api/v1/D54GRG29T5HB0/mqtt/dataPub"` |
| 0x424 | device_key | 64 B | `"D54GRG29T5HB0"` |
| **合计** | �?| **453 B** | 截止 0x463, 空闲区仍�?923B |

---

## 十四、最终代码改动量

| 文件 | 改动内容 | 行数 |
|------|---------|------|
| `app_config.h` | 新增客户 MQTT 默认值宏 | +10 �?|
| `eeprom_config.h` | 新增 `MqttConfigB_t` + 地址�?+ device_key | +30 �?|
| `eeprom_config.c` | 加载/保存客户配置 + 串口命令 | +70 �?|
| `ec800e.h` | 新增 `_B` 系列函数声明 | +8 �?|
| `ec800e.c` | 参数化连�?+ 客户 publish (�?LY01-LY11 JSON) | +100 �?|
| `ec800e.c` | `s_payload[512]` �?`s_payload[1024]` | �?1 �?|
| `main.c` | 3 �?push 补填 `pressure2`/`temperature2` | +6 �?|
| `main.c` | `do_upload()` 追加 client 1 上报循环 (�?s 帧间�? | +20 �?|
| `main.c` | 报警上报追加 client 1 | +5 �?|
| **合计** | �?| **~250 �?* |
