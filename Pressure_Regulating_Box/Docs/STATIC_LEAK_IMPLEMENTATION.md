# 静态测漏功能实现说明

> 适用范围: `Pressure_Regulating_Box` 工程 (GD32L233R + M24M01 128KB)
> 相关模块: `Middle/leak_task.c` / `Middle/leak_store.c` / `Drivers/ec800e.c` / `Drivers/PT_304_35.c`

## 1. 功能概述

平台通过 MQTT 下发测漏任务, 设备按指定频率采集压力/温度, 数据落 M24M01 专用分区,
采集结束后按 5 条一包上报, 最后回执任务完成并清空数据。

- 独立链路: 不复用也不修改 `function/get` 指令表, 不影响常规采集/上报/报警/OTA
- 任务期间: 传感器保持供电、MCU 不休眠、4G 保持在线 (QSCLK)
- 采完再发: 采集阶段结束后才进入上报阶段
- 无断点续传: 上电与任务结束都清空暂存区, 不管上报是否成功

## 2. 协议

### 2.1 下发 `/155/{表号}/leak/set`

```json
// type=1 任务下发
{"type":1,"taskNumber":"LEAK20260818161943202607280001999","samplingFreq":120,"collectionDuration":900}
// type=2 阀门指令
{"type":2,"taskNumber":"LEAK20260818161943202607280001999","valveCommand":1}
```

| 字段 | 含义 |
|---|---|
| `type` | 1=测漏任务 2=阀门指令 |
| `taskNumber` | 任务序号 (最长约 33 字符) |
| `samplingFreq` | 采集频率, 单位秒 |
| `collectionDuration` | 采集时长, 单位秒 |
| `valveCommand` | 1=开阀 2=关阀 |

### 2.2 上报 `/155/{表号}/leak/reply` (topic 镜像自下发 topic)

```json
{"taskStatus":1,"taskNumber":"..."}                                  // 已接收
{"taskStatus":2,"taskNumber":"...","taskData":[ ... ]}               // 数据包(每包5条)
{"taskStatus":3,"taskNumber":"..."}                                  // 任务完成
{"taskStatus":5,"taskNumber":"...","valveCmdStatus":1,"failReason":""} // 阀门回执
```

数据条目 (双路传感器):

```json
{"collationTime":"2026-09-12 13:00:00","temp":20.11,"pressure":13.33,"temp2":20.11,"pressure2":13.33}
```

单路传感器时只输出 `collationTime` / `temp` / `pressure`。

| 项 | 口径 |
|---|---|
| 字段名 | 与产品物模型一致: `temp` / `pressure` / `temp2` / `pressure2` |
| `collationTime` | 字符串, `yyyy-MM-dd HH:mm:ss` (非 ISO 时区格式) |
| 数值 | JSON 数字, `%.2f`; 压力单位 Pa (不要复用用户端 MQTT 的 kPa 组包) |
| `valveCmdStatus` | 1=成功 2=失败 |
| `failReason` | 字符串: 成功 `""`, 失败 `"1"`=动作超时未到位 / `"2"`=阀门异常 |

`failReason` 若平台要求数字口径, 把 `leak_task.c` 中 `LEAK_FAILREASON_AS_CODE` 改为 `1` 即可 (成功=0)。

### 2.3 状态机口径

| 场景 | 行为 |
|---|---|
| 任务条数 `floor(D/f)+1` > 暂存区容量 | **不采集、不回执**(平台侧超时), 仅调试串口告警 |
| 参数非法 (freq/dur<=0, taskNumber 为空) | 同上, 静默忽略 + 日志 |
| 任务运行中再收到 type=1 | 忽略 + 日志 |
| 任务运行中收到 type=2 | 入队(深度 4, 满则丢最旧), **不打断采集**; 任务结束后按序补执行并逐条回执 |
| 非任务运行期收到 type=2 | 立即执行并回执 |

## 3. topic 与订阅

| # | msgid | topic | 说明 |
|---|---|---|---|
| 1 | 1 | `/{productId}/{deviceNum}/function/get` | 常规指令 (原有) |
| 2 | 2 | `/{productId}/{deviceNum}/upgrade/set` | OTA (原有) |
| 3 | 3 | `/{productId}/{deviceNum}/leak/set` | 静态测漏 (新增, QoS0) |

- 全部主题由上报主题 `topic_up` 派生 (`EC801E_DeriveTopic`: 把 `/property/post` 换成目标后缀)
- 统一入口 `EC801E_SubscribeAll()`: 上电首次连接、`first_ip_set` 改IP、串口 `first_ip:`/`mqtt_reconnect` 均调用它,
  顺带修复了"改IP后 OTA 主题不重订阅"的旧问题
- 回复 topic 镜像下发 topic: `leak/set → leak/reply`; 若平台改用 `monitor/get`, 代码已按 `monitor/post` 兼容,
  只需在 `EC801E_SubscribeAll()` 中追加一条订阅

## 4. EEPROM 布局 (M24M01, 128KB)

| 区块 | 地址 | 大小 | 内容 |
|---|---|---|---|
| 配置区 | `0x00000~0x00FFF` | 4KB | 既有配置块 (实占 1085B) + 布局版本 `0x4F0` |
| 静态测漏区 | `0x01000~0x10FFF` | 64KB | 32B 管理头 + **2047 条 × 32B** |
| 历史环形区 | `0x11000~0x1FFFF` | 60KB | 32B 管理头 + **1919 条 × 32B** |

- 测漏管理头只在 3 个时刻各写一次 (任务开始 / 采集完成 / 清除), 不随采样更新, 避免单页磨损
- 测漏每条 32B 恰好一次页写 (驱动按 32B 分页), 带 CRC8 自校验
- AT24C64 版本无测漏分区, `leak_store.c` 提供空实现, 工程仍可编译

### 4.1 布局版本与迁移 (关键)

历史区地址由 `0x5B00` 变更为 `0x11000`。新地址在旧布局里是历史数据页,
直接读会以 1/65536(再叠加 CRC 1/256) 的概率"命中"旧 magic, 属于概率性误判。

因此新增配置块 (Magic `0xCC`):

| 地址 | 内容 |
|---|---|
| `0x4F0` | Magic = 0xCC |
| `0x4F1` | `layout_ver` (当前 = 2) |

上电流程 (`main()` 内 `ee_hist_init()` / `leak_store_init()` 之后调用
`eeprom_layout_check_and_migrate()`):

```
读 0x4F0/0x4F1
├─ magic==0xCC 且 ver==2 → 正常启动, 不动数据
└─ 其它情况(老固件升级上来/首次烧写)
      → ee_hist_clear()  (历史区格式化)
      → leak_store_clear()
      → 写入 magic + ver=2
```

以后布局再次调整: 把 `EE_LAYOUT_VER_CURRENT` +1 即可, 迁移自动完成。

> 说明: 本次升级到该版本, 设备里旧的历史缓存会被格式化一次 (一次性)。

### 4.2 容量核算

采集条数 `N = floor(collectionDuration / samplingFreq) + 1` (t=0 立即采第 1 条)

| 参数 | 条数 | 结论 |
|---|---|---|
| 900s / 120s | 8 | ✅ |
| 600s / 2s | 301 | ✅ |
| 900s / 2s | 451 | ✅ |
| 900s / 1s | 901 | ✅ |
| 1023s / 2s | 512 | ✅ |
| 2046s / 1s | 2047 | ✅ (刚好用满) |
| 3600s / 1s | 3601 | ❌ 不采集/不回执 |

## 5. 代码结构

| 文件 | 角色 |
|---|---|
| `Middle/leak_task.c/.h` (新增) | 业务层: 下行解析(type1/2)、参数校验、任务状态机、JSON 组包、阀门队列 |
| `Middle/leak_store.c/.h` (新增) | 存储层: 测漏分区管理头/记录读写、CRC、清零 |
| `Drivers/ec800e.c/.h` | 新增 `EC801E_DeriveTopic` / `SubscribeLeakTopic` / `SubscribeAll` / `ReadDownlinkEx` / `PublishOnTopic`; 原 `ReadDownlink`、`PublishRawJson` 变成薄封装 |
| `Drivers/PT_304_35.c/.h` | 新增 `PT304D_PowerHold()` / `PT304D_CollectAllHeld()` (任务期间传感器不断电) |
| `Middle/downlink.c` | `drain_pending_downlink()` 按 topic 分流; 新增 `downlink_handle_raw()` 供测漏期间复用常规指令链路 |
| `Middle/eeprom_history.h` | 历史区地址/条数宏 (布局 B) |
| `Middle/eeprom_config.c/.h` | 布局版本块 + `eeprom_layout_check_and_migrate()` |
| `Middle/main.h` / `main.c` | `STATE_LEAK` 状态、主循环接入、`vtiming_check_now()` 对外暴露、订阅收口 |

数据流:

```
leak/set ──► drain_pending_downlink ──► leak_task_on_msg ──► 任务上下文
                                              │
主循环 STATE_LEAK ──► leak_task_run ──► 采集(leak_store_push) ──► 分包(EC801E_PublishOnTopic) ──► leak/reply
```

## 6. 任务时序

```
接收   回 taskStatus=1 ; 确认 4G 在线(必要时冷启动) ; 写管理头(state=采集中)
采集   PT304D_PowerHold(1) 保持供电
       t=0 起每 samplingFreq 秒采 1 条 (RTC 秒对齐, 不累积漂移)
       每条: 采集 → 写 EEPROM(32B 页写)
       每 10s: vtiming_check_now() + 轮询下行(type=2 入队; 非测漏指令交回原链路; OTA 忽略)
上报   管理头 state=待上报(含条数) ; 唤醒模组并保持唤醒
       每 5 条一包, 包间 100ms ; 单包失败: 重连后重发一次, 仍失败则记日志继续
收尾   回 taskStatus=3 → 清空暂存区 → 补执行阀门指令 → 传感器断电 → 4G 回 QSCLK → 回 STATE_SLEEP
```

耗时预算 (实测参数: 16MHz 内核、软件 I2C 半周期 150 → 约 10kHz、EEPROM 写周期 5ms):

- 单条记录 EEPROM 写 ≈ 30~36ms; 单条样本含传感器读取 ≈ 50~100ms (双路)
- 上报阶段: 每包约 0.4~0.8s (AT 往返为主, 100ms 只是下限)

| 规模 | 条数 | 包数 | 采集期 | 上报期估计 |
|---|---|---|---|---|
| 900s@120s | 8 | 2 | 900s | <2s |
| 600s@2s | 301 | 61 | 600s | 25~50s |
| 900s@1s | 901 | 181 | 900s | 72~145s |
| 2046s@1s | 2047 | 410 | 2046s | 165~330s |

> 平台侧超时判定需按 `collectionDuration + 上报耗时` 设计。

## 7. 可调参数 (宏)

| 宏 (leak_task.c) | 默认 | 说明 |
|---|---|---|
| `LEAK_PACKET_RECORDS` | 5 | 每包条数 |
| `LEAK_PACKET_GAP_MS` | 100 | 包间最小间隔 |
| `LEAK_POLL_INTERVAL_S` | 10 | 采集期间下行轮询间隔 |
| `LEAK_VALVE_QUEUE_DEPTH` | 4 | 任务期间阀门指令队列深度 |
| `LEAK_RESAMPLE_RETRY` | 1 | 单条采样失败重试次数 |
| `LEAK_FAILREASON_AS_CODE` | 0 | failReason 字符串/数字口径 |
| `LEAK_WAIT_SLICE_MS` | 200 | 采集等待分片(喂狗粒度) |

## 8. 验证清单

| # | 用例 | 判据 |
|---|---|---|
| 1 | 双路 900s@120s | 8 条、2 包、回执 1→2→3、字段/时间格式正确 |
| 2 | 单路 900s@120s | 仅 `temp`/`pressure` |
| 3 | 2046s@1s | 2047 条, 不越界到历史区 |
| 4 | 3600s@1s | 不采集、不回执, 调试口告警 |
| 5 | type=2 开/关阀 | 回 `taskStatus=5`, `valveCmdStatus`/`failReason` 正确 |
| 6 | 任务中收到 type=2 | 采集不中断, 任务结束后补执行并回执 |
| 7 | 任务中平台下发 `get_pressure` | 仍正常回执 |
| 8 | 断网(关服务器) | 采集照常, 上报失败不卡死, 结束清除并回睡 |
| 9 | 任务中途掉电 | 重新上电后测漏区被清, 常规功能正常 |
| 10 | 老固件升级首启 | 打印 `storage layout changed`, 历史被格式化一次, 版本写入 |
| 11 | `first_ip_set` 回归 | 改IP后 3 个主题都重订阅成功(OTA 可用) |
| 12 | 看门狗 | 全程(含 120s 间隔) 不复位 |

## 9. 待与平台确认

1. `samplingFreq` 最小值 / `collectionDuration` 最大值 (决定超限门槛; 当前容量 2047 条)
2. 900s/120s 的期望条数 (当前口径 8 条), 以及尾包不足 5 条是否需要特殊处理 (当前按实际条数发)
3. 任务运行中收到的阀门指令, 允许延迟到任务结束后才回 `taskStatus=5` 吗
   (最坏延迟 = 剩余采集时间 + 上报时间)
4. `failReason` 成功时用空串还是数字 0
