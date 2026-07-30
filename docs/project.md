# 家用阀门物联网控制器 — 项目说明文档

> **MCU**: GD32L233RC (ARM Cortex-M23, 256KB Flash, 32KB SRAM)  
> **固件版本**: v3  
> **IDE**: Keil MDK-ARM  
> **工程路径**: `Pressure_Regulating_Box/Projects/Wakeup_box_V0.0.1/`

---

## 目录

1. [系统概述](#1-系统概述)
2. [硬件引脚分配](#2-硬件引脚分配)
3. [Flash 分区与 OTA 升级方案](#3-flash-分区与-ota-升级方案)
4. [状态机架构](#4-状态机架构)
5. [通信模块](#5-通信模块)
6. [功能特性](#6-功能特性)
7. [MQTT 下发指令列表](#7-mqtt-下发指令列表)
8. [Debug 串口调试命令](#8-debug-串口调试命令)
9. [配置管理 (EEPROM)](#9-配置管理-eeprom)
10. [BLE 蓝牙模块](#10-ble-蓝牙模块)
11. [低功耗设计](#11-低功耗设计)
12. [文件结构与模块职责](#12-文件结构与模块职责)
13. [首次烧录配置流程](#13-首次烧录配置流程)
14. [常见问题排查](#14-常见问题排查)

---

## 1. 系统概述

本项目为家用调压箱物联网控制器固件，核心功能包括：

- **压力/温度传感器采集** — 双路 PT304D I2C 传感器（进气端 + 出气端）
- **电磁阀控制** — 电机驱动球阀，支持远程/本地开关阀
- **4G MQTT 上报** — 通过 EC801E 4G Cat.1 模组进行 MQTT 持久连接
- **阈值告警 + 自动关阀** — 压力超限/突变/温度异常/电压保护自动关阀
- **深度休眠 + RTC/LPUART 唤醒** — 极低功耗设计，电池供电可用多年
- **OTA 空中升级** — 支持 4G HTTP 下载固件 + BLE 近场升级两种方式
- **段码 LCD 显示** — 实时压力/温度显示，信号/蓝牙/阀门状态图标
- **多种工作模式** — long(长连接) / short(窗口) / dot(定点) 三种上报策略

---

## 2. 硬件引脚分配

### 2.1 UART 串口

| 外设 | 引脚 | 功能 | 波特率 |
|------|------|------|--------|
| USART0 | PA2(TX), PA3(RX) | Debug 调试串口 | 115200 |
| LPUART0 | PD8(TX), PD9(RX) | 4G 模组通信 | 115200 |
| UART4 | PB3(TX), PB4(RX) | BLE 蓝牙模组 | 115200 |

### 2.2 GPIO 控制

| 引脚 | 功能 | 说明 |
|------|------|------|
| PA0 | 4G_DTR | HIGH=允许EC801E休眠, LOW=强制唤醒 |
| PA5 | M_SLP | 电机驱动使能(LOW=关闭) |
| PA11 | K1-S | 开阀到位限位开关(LOW=到位) |
| PA12 | K2-S | 关阀到位限位开关(LOW=到位) |
| PA15 | LED1 | 指示灯 |
| PB0/PB1 | SLCD SEG5/6 | 段码LCD |
| PB5 | V4G_EN | 4G电源使能(HIGH=上电) |
| PB8 | BUZZER | 蜂鸣器(HIGH=响) |
| PB9 | SLCD COM3 | 段码LCD |
| PB10-PB15 | SLCD SEG10-15 | 段码LCD |
| PC6-PC11 | SLCD SEG24-29 | 段码LCD |
| PC12 | SW_BT | 蓝牙电源开关(HIGH=上电) |
| PC14/PC15 | LXTAL | 32.768kHz外部晶振(RTC/SLCD) |
| PD0 | Btn1 | 按键(外部上拉, EXTI0唤醒) |
| PD2 | BLE_LINK | BLE连接状态(LOW=已连接) |
| PD4 | SW_IIC | 传感器I2C电子开关(HIGH=上电) |

### 2.3 I2C 总线

| 通道 | 引脚 | 设备 |
|------|------|------|
| I2C0 (软件) | — | PT304D 传感器 #1 (出气端) |
| I2C1 (软件) | — | PT304D 传感器 #2 (进气端) |
| I2C (软件) | — | M24M01 EEPROM (128KB) |

---

## 3. Flash 分区与 OTA 升级方案

### 3.1 Flash 布局 (256KB, 4KB/page)

```
0x08000000 - 0x08003FFF   16 KB   Bootloader   (pages 0-3)
0x08004000 - 0x08004FFF    4 KB   OTA InfoBlock (page 4)
0x08005000 - 0x08021FFF  116 KB   App 运行区   (pages 5-33)
0x08022000 - 0x0803FFFF  120 KB   下载缓存区   (pages 34-63)
```

### 3.2 4G OTA 升级流程

1. 平台下发 `downloadUrl` + `version` JSON
2. App 上报 `mqttUpgradeReply=2` (升级中)
3. 版本校验 — `new_version > APP_FW_VERSION_NUM` 才继续
4. 启用独立看门狗 (FWDGT, ~26s 超时)
5. 擦除缓存区 (pages 34-63)
6. HTTP 分块下载固件到缓存区，每块喂狗
7. 写 InfoBlock: `magic=0xAABB1122, status=PENDING(0x55AA55AA), size, version`
8. `NVIC_SystemReset()` → Bootloader 检测 PENDING → 拷贝缓存→App区 → 写 SUCCESS
9. 新 App 启动，读 InfoBlock 上报 `mqttUpgradeReply=3` (成功)

### 3.3 BLE OTA 升级流程

- **模块**: VG6328A-MS BLE 5.1 透传模块
- **入口**: 3次短按按键 → STATE_BLE_OTA
- **协议帧格式**: `SOF(0x5A) + VER(0xA5) + CMD(1B) + LEN(2B,LE) + PAYLOAD + CRC16(2B,LE)`
- **命令码**: START_REQ(0x01), DATA(0x02), END(0x03), ABORT(0x04)
- **传输**: 手机APP通过BLE透传发送固件分包，MCU 写入缓存区，完成后同4G路径提交
- **超时**: 连接等待60s, 空闲/传输30s无数据自动退出

---

## 4. 状态机架构

```
STATE_INIT → STATE_SLEEP → STATE_WAKEUP ─┬→ STATE_COLLECT → STATE_UPLOAD
                  ↑                       ├→ STATE_DOWNLINK              │
                  │                       ├→ STATE_ALARM                 │
                  │                       └→ STATE_BLE_OTA               │
                  └─────────────────────────────────────────────────────┘
```

| 状态 | 功能 |
|------|------|
| **STATE_INIT** | 上电初始化: 外设→EEPROM加载→4G联网→MQTT连接→首次采集上报 |
| **STATE_SLEEP** | 配置RTC闹钟→深休眠 (MCU + 4G QSCLK 省电) |
| **STATE_WAKEUP** | 恢复外设→判断唤醒源(RTC/LPUART/按键) |
| **STATE_COLLECT** | 采集传感器→阈值检查→存入环形缓冲→判断是否上报 |
| **STATE_UPLOAD** | 批量逐条上报历史数据→清空缓冲 |
| **STATE_DOWNLINK** | 读取+处理MQTT下发指令→立即上报结果 |
| **STATE_ALARM** | 异常重采确认(3次)→报警上报→自动关阀 |
| **STATE_BLE_OTA** | BLE蓝牙固件升级(等待连接→接收数据→写Flash→提交) |

### 4.1 实时采集模式

当 `real_enabled=1` 时，MCU 以 `real_frequency` (1-10秒) 为间隔执行最小化唤醒:
- 仅初始化 SysTick + I2C，不启 4G
- 采集 → 阈值检查 → 正常直接回睡
- 异常 → 启 4G → 3次重采确认 → 上报告警 → 自动关阀

---

## 5. 通信模块

### 5.1 4G 模组 (EC801E)

- **驱动文件**: `ec800e.c / ec800e.h`
- **接口**: LPUART0 (PD8/PD9), DMA 环形缓冲接收
- **MQTT 协议**: 连接 FastBee 物联网平台
- **省电**: QSCLK=1 轻睡眠(保持MQTT) / QSCLK=2 深睡眠
- **DTR 控制**: PA0 控制模组休眠/唤醒

### 5.2 MQTT Topic 格式

```
上报: /<product_id>/<device_id>/property/post
订阅: /<product_id>/<device_id>/function/get
OTA:  /<product_id>/<device_id>/upgrade/set
      /<product_id>/<device_id>/upgrade/reply
```

**Client ID 格式**: `S&<device_id>&<product_id>&<seq>`  
**示例**: `S&D54GRG29T5HB0&149&1`

### 5.3 用户端 MQTT (可选第二路服务器)

- 支持配置独立的 server/port/username/password/client_id/topic
- MQTT id=1 (与主平台 id=0 并行)
- 传感器数据以 LY 格式 JSON 发布

---

## 6. 功能特性

### 6.1 压力/温度阈值告警

支持双路独立阈值配置:

| 参数 | 说明 |
|------|------|
| ppmax / ppmin | 极高/极低压力阈值 |
| pmax / pmin | 高/低压力阈值 |
| mutation | 突变检测阈值 |
| shock | 迟滞回差 |
| high / low | 温度高/低报警 |
| vhigh / vlow | 温度极高/极低报警 |

**告警类型**: VERY_HIGH, VERY_LOW, OVER_LIMIT, UNDER_LIMIT, MUTATION, TEMP_HIGH, TEMP_LOW, TEMP_VERY_HIGH, TEMP_VERY_LOW, VOLTAGE_HIGH, VOLTAGE_LOW, VPROTECT

### 6.2 异常自动关阀策略

通过 `abnormal_close` 参数配置，支持:

| 策略值 | 含义 |
|--------|------|
| none | 不自动关阀 |
| high | 压力超上限关阀 |
| low | 压力超下限关阀 |
| mutation | 压力突变关阀 |
| highorlow | 超上限或超下限 |
| highormutation | 超上限或突变 |
| lowormutation | 超下限或突变 |
| all | 全部压力异常 |
| temph | 温度过高关阀 |
| templ | 温度过低关阀 |

> **电压保护 (VPROTECT)**: 当电压低于 `spupower.protect` 时，无条件强制关阀，独立于上述策略。

### 6.3 电磁阀控制

- **驱动**: 电机球阀 (PA5=驱动使能, PA11/PA12=限位开关)
- **到位检测**: 限位开关低电平 = 动作到位
- **延时功能**: `valve_delay` 可配置动作延时 (0-65535ms)
- **状态持久化**: `g_valve_last_order` 存入 EEPROM，掉电/OTA 重启后恢复

### 6.4 窗口模式

| 模式 | 说明 |
|------|------|
| **long** | 4G长连接，MCU休眠期间模组保持MQTT(QSCLK)，支持LPUART唤醒接收下发 |
| **short** | 定时活动窗口(start~stop)，窗口内4G在线，窗口外断电 |
| **dot** | 定点上报，每天指定整点(dottime)上报一次 |

### 6.5 传感器在线/离线检测

- 消抖机制: 连续3次离线确认 / 连续2次上线确认
- 状态变化立即上报 (不等待上报周期)
- 离线时跳过阈值检查

### 6.6 电压检测与低电量保护

- ADC 采集电池电压
- `spupower.high/low`: 电压报警阈值 (仅上报)
- `spupower.protect`: 极低电压保护 → 强制关阀 + 进入低功耗模式 (6:00/18:00 上报)

### 6.7 LCD 段码显示

- 4×COM, 多×SEG 段码驱动
- 显示内容: 压力值(KPa)、温度值(℃)、信号强度、蓝牙图标、阀门状态
- 按键点亮后 60 秒自动关闭
- 可通过 `lcd_switch` 命令远程禁用

### 6.8 蜂鸣器提示

- 联网成功: 2声短鸣
- 按键操作: 声音反馈

---

## 7. MQTT 下发指令列表

### 7.1 查询类指令 (commandStr)

| commandStr | 功能 | 上报字段 |
|------------|------|----------|
| `get_pressure` / `get_pressure1` | 查询进气端压力 | pressure, pressure_status, temp_status |
| `get_pressure2` | 查询出气端压力 | pressure2, pressure2_status, temp2 |
| `temp_get1` | 查询进气端温度 | temp_status, temp |
| `temp_get2` | 查询出气端温度 | temp2_status, temp2 |
| `sensor_status` | 查询传感器在线状态 | rtu_have/status × 2 |
| `get_sim` | 查询SIM/信号信息 | csq, soft-version, ICCID, IMEI |
| `get_ip` / `first_ip_get` | 查询主服务器IP | first_ip |
| `scend_ip_get` | 查询副服务器IP | scend_ip |
| `LY06_get` | 查询电压报警状态 | LY06, voltage |
| `valve_last_get` | 查询阀门状态 | valve_last_order |
| `threshold_get1` | 查询所有阈值 | vpthreshold + Tthreshold + spupower |
| `vpthreshold_get1` | 查询压力阈值(通道1) | vpthreshold_* |
| `vpthreshold_get2` | 查询压力阈值(通道2) | vpthreshold2_* |
| `Tthreshold_get` | 查询温度阈值(通道1) | Tthreshold_* |
| `Tthreshold_get2` | 查询温度阈值(通道2) | Tthreshold2_* |
| `collectcycle_get` | 查询采集周期 | collect_once, collect_type |
| `report_get` | 查询上报周期 | report_once, report_type |
| `window_get` | 查询窗口模式 | window_pattern/start/stop/end_result/dottime |
| `abnormal_get` | 查询异常关阀策略 | abnormal_close |
| `real_get` | 查询实时模式状态 | real_enabled, real_frequency |
| `user_ip_get` | 查询用户端服务器 | user_ip |
| `MQTT_get` | 查询用户端MQTT参数 | MQTT_username/password/client_id/subject |

### 7.2 设置类指令 (commandStr)

| commandStr | 功能 | 参数格式 |
|------------|------|----------|
| `valve_open` | 开阀 | — |
| `valve_close` | 关阀 | — |
| `collectspeed_set` | 设置采集周期 | collect_once + type |
| `report_set` | 设置上报周期 | report_once + type |
| `window_set` | 设置窗口模式 | window_pattern + start/stop/end_result/dottime |
| `abnormal_set` | 设置异常策略 | abnormal_close |
| `first_ip_set` | 设置主IP | "IP.port" |
| `scend_ip_set` | 设置副IP | "IP.port" |
| `vpthreshold_set1` | 设置压力阈值(通道1) | vpthreshold_switch/ppmax/pmax/ppmin/pmin/mutation/shock |
| `vpthreshold_set2` | 设置压力阈值(通道2) | vpthreshold2_* |
| `Tthreshold_set` | 设置温度阈值(通道1) | Tthreshold_switch/high/low/vhigh/vlow |
| `Tthreshold_set2` | 设置温度阈值(通道2) | Tthreshold2_* |
| `spupower_set` | 设置电压检测 | spupower_switch/high/low/protect |
| `delay_set` | 设置阀门延时 | delay_switch/delay_time |
| `real_set` | 设置实时模式 | real_enabled + real_frequency |
| `lcd_switch` | 设置LCD开关 | 0=禁用, 1=启用 |
| `sensor_count_set` | 设置传感器数量 | 1=单路, 2=双路 |
| `user_ip_set` | 设置用户端IP | "IP.port" |
| `MQTT_set` | 设置用户端MQTT | MQTT_username/password/subject/client_id |
| `restart` | 远程重启 | — |
| `restart_off` | 关阀后重启 | — |
| `4Gpower_off` | 关闭4G | — |

---

## 8. Debug 串口调试命令

通过 **USART0 (115200,8N1)** 输入命令，换行符 `\r\n` 结尾。

### 8.1 实时查询命令

| 命令 | 功能 |
|------|------|
| `status` | 查看状态机状态、唤醒计数、缓冲区条数、传感器数据、阀门状态 |
| `sensor` | 立即采集并显示压力/温度/电压/错误码 |
| `sensor_status` | 查询传感器在线/状态 |
| `temp_status` | 查询温度状态等级 |
| `pressure_status` | 查询压力状态等级 |
| `sim` | 查询CSQ/ICCID/IMEI |
| `time` | 显示当前RTC时间 |
| `buf` | 查看缓冲区数据条数 |
| `vlp` | 查看低电量保护状态 |
| `config` | 打印当前所有配置 |
| `vpthreshold_get` | 打印压力阈值(通道1) |
| `Tthreshold_get` | 打印温度阈值 |
| `spupower_get` | 打印电压检测配置 |
| `delay_get` | 打印阀门延时配置 |
| `valve_status` | 打印阀门最后指令 |
| `ip` | 打印主IP/副IP |
| `umqtt_get` | 打印用户端MQTT配置 |

### 8.2 控制命令

| 命令 | 功能 |
|------|------|
| `valve:open` | 开阀 |
| `valve:close` | 关阀 |
| `4Gpower_off` | 关闭4G |
| `4Gpower_on` | 开启4G并重连 |
| `mqtt_reconnect` | 断开并重连MQTT |
| `restart` | MCU软件复位 |
| `reset` | 恢复出厂设置 (清除EEPROM所有Magic, 重启) |

### 8.3 配置命令

**MQTT 配置** (JSON格式):
```
{clientId:S&D54GRG29T5HB0&149&1,username:FastBee,passwd:P41TQ4X1Y0C57IW0,IP:120.78.220.171,port:1883}
```

**用户端 MQTT 配置**:
```
{umqtt_server:192.168.1.100,umqtt_port:13000,umqtt_user:admin,umqtt_pass:123,umqtt_client:device01,umqtt_topic:/custom/topic}
```

**IP 修改**:
```
first_ip:120.78.220.171.1883
scend_ip:192.168.1.200.1884
```

**采集/上报周期**:
```
{id:collect_cycle,once:10,type:minute}
{id:report_cycle,once:1,type:hour}
```
> type 可选: `sec` / `minute` / `hour` / `day` / `week`

**窗口模式**:
```
{id:window_pattern,value:short,start:080000,stop:200000,end_result:1,dottime:3}
```

**异常关阀策略**:
```
{id:abnormal,value:highorlow}
```

**压力阈值** (通道1):
```
{id:vpthreshold,switch:1,ppmax:500.0,pmax:400.0,ppmin:50.0,pmin:100.0,mutation:200.0,shock:10.0}
```

**温度阈值**:
```
{id:Tthreshold,switch:1,high:60.0,low:-10.0,vhigh:80.0,vlow:-20.0}
```

**电压检测**:
```
{id:spupower,switch:1,high:4.2,low:3.3,protect:3.0}
```

**阀门延时**:
```
{id:delay,switch:1,delay_ms:500}
```

**MQTT心跳**:
```
{id:keepalive,value:300}
```

**产品开关**:
```
lcd_switch=1
sensor_count=2
```

---

## 9. 配置管理 (EEPROM)

### 9.1 存储芯片

**M24M01** — 128KB I2C EEPROM, 页写入

### 9.2 存储区域

| 区域 | Magic | 地址 | 内容 |
|------|-------|------|------|
| 间隔配置 | 0xA5 | 0x000 | collect_interval, upload_interval |
| MQTT | 0x5A | 0x020 | client_id, username, password, server, port, topic |
| 压力阈值1 | 0xC0 | 0x1B0 | vpthreshold (进气端) + keepalive |
| 温度阈值1 | 0xC1 | 0x1D0 | Tthreshold (进气端) |
| 电压检测 | 0xC2 | 0x1F0 | spupower |
| 扩展配置 | 0xB7 | 0x200 | window/cycle/abnormal/valve/real/副IP等 |
| 阀门延时 | 0xC3 | 0x290 | delay enabled + ms |
| 压力阈值2 | 0xC4 | 0x2A0 | vpthreshold2 (出气端) |
| 温度阈值2 | 0xC5 | 0x2C0 | Tthreshold2 (出气端) |
| 用户端MQTT | 0xC6 | 0x300 | user mqtt server/port/user/pass/client/topic |

### 9.3 加载逻辑

1. 上电读取 Magic 字节
2. Magic 有效 → 从 EEPROM 加载
3. Magic 无效 → 使用 `app_config.h` 中的编译时默认值
4. MQTT 区 Magic 无效 → 进入 30s 配置等待模式

---

## 10. BLE 蓝牙模块

### 10.1 硬件

- **模块**: VG6328A-MS BLE 5.1 透传
- **串口**: UART4 (PB3/PB4), 115200, DMA环形缓冲 1024B
- **电源**: PC12 控制 (HIGH=上电)
- **连接检测**: PD2 (LOW=手机已连接)

### 10.2 BLE 名称配置

通过 AT 指令在每次进入 BLE OTA 模式时自动设置:
- **名称格式**: `YLY_<设备ID后3位>`
- **示例**: 设备ID = `D54GRG29T5HB0` → BLE名 = `YLY_HB0`
- **AT 命令序列**:
  1. `AT+ENAT\r\n` → 进入 AT 模式 (等待 OK)
  2. `AT+LENAYLY_HB0\r\n` → 设置名称 (等待 OK)
  3. `AT+REST\r\n` → 模块复位生效

### 10.3 BLE OTA 操作步骤

1. **按键 3 次短按** → 蓝牙开启 + 进入 `STATE_BLE_OTA`
2. 自动设置 BLE 名称 (AT指令)
3. 等待手机连接 (PD2=LOW, 60s 超时)
4. UART4 初始化 → 接收 OTA 数据帧
5. 手机 APP 发送固件 → MCU 写入缓存区
6. 完成后写 InfoBlock + 重启
7. 再次按 3 次 → 蓝牙关闭

---

## 11. 低功耗设计

### 11.1 深休眠 (DeepSleep)

- MCU 进入 GD32L233 DeepSleep 模式
- 所有未用 GPIO 设为模拟输入 (杜绝漏电)
- LXTAL 保持运行 (RTC + SLCD)
- 唤醒源: RTC闹钟 / LPUART 字节 / 按键 EXTI

### 11.2 4G 省电

- **QSCLK=1**: 模组轻睡眠, 保持 MQTT 连接, DTR HIGH 允许休眠
- **唤醒**: DTR LOW → 模组醒来 → 可收发 AT/MQTT

### 11.3 软件看门狗

- SysTick 计时 + 超时检测 (非硬件 WDT)
- 深休眠前禁用, 唤醒后重启
- 防止 4G 通信阻塞死锁

---

## 12. 文件结构与模块职责

```
MDK-ARM/my_Library/
├── main.c/h            — 主状态机、全局变量、debug命令(硬件相关)
├── app_config.h        — 编译时默认配置、枚举定义
├── eeprom_config.c/h   — EEPROM读写、配置解析、串口配置命令
├── ec800e.c/h          — 4G EC801E 驱动: AT命令、MQTT、HTTP下载
├── downlink.c/h        — MQTT下发指令解析与执行
├── usart.c/h           — USART0/LPUART0/UART4 初始化、DMA、中断
├── ble_ota.c/h         — BLE OTA协议解析、AT名称配置
├── ota_app.c/h         — 4G OTA流程: 下载→缓存→InfoBlock→重启
├── ota_info.h          — Flash分区定义、InfoBlock结构
├── threshold.c/h       — 阈值检查(双通道)、状态判断
├── sensor_buffer.c/h   — 传感器数据环形缓冲 (SRAM)
├── PT_304_35.c/h       — PT304D I2C传感器驱动
├── valve.c/h           — 电磁阀控制(电机驱动+限位检测)
├── mygpio.c/h          — GPIO初始化、按键EXTI、低功耗配置、BLE连接检测
├── tim.c/h             — RTC初始化/闹钟/时间获取
├── delay.c/h           — 延时函数 (trea_delay_ms, trea_millis)
├── lcd_ui.c/h          — LCD UI层 (数值显示/图标/蓝牙/信号)
├── slcd_seg.c/h        — SLCD底层驱动 (段码映射)
├── adc_bat.c/h         — 电池电压ADC采集
├── i2c_sensor.c/h      — I2C软件驱动 (传感器)
├── i2c_eeprom.c/h      — I2C软件驱动 (EEPROM)
├── m24m01.c/h          — M24M01 EEPROM读写接口
├── json_builder.c/h    — 轻量JSON构建器 (上报用)
├── led.c/h             — LED控制
├── buzzer.c/h          — 蜂鸣器控制
└── motor.c/h           — 电机底层驱动
```

---

## 13. 首次烧录配置流程

### 13.1 烧录

1. 通过 SWD 烧录 Bootloader (0x08000000)
2. 烧录 App 固件 (0x08005000)

### 13.2 首次上电

1. MCU 启动，检测 EEPROM MQTT Magic = 0xFF (未配置)
2. 进入 **配置等待模式** (30s 超时)
3. 通过 Debug 串口发送 MQTT 配置:
   ```
   {clientId:S&DEVICE_ID&PRODUCT_ID&1,username:FastBee,passwd:YOUR_PASS,IP:120.78.220.171,port:1883}
   ```
4. 收到后立即保存，继续接受 5s 可选参数
5. 超时后以默认值启动剩余模块
6. 4G 开机 → 注网 → MQTT 连接 → 首次数据上报

### 13.3 恢复出厂

Debug 串口输入 `reset` → 清除所有 EEPROM Magic → MCU 重启 → 重新进入配置等待模式

---

## 14. 常见问题排查

### Q: 设备无法联网

1. `sim` 命令查看 CSQ (>10 正常), ICCID (非空=SIM正常)
2. `ip` 确认服务器IP/端口正确
3. `mqtt_reconnect` 尝试重连
4. `4Gpower_on` 手动开启4G

### Q: 传感器数据异常

1. `sensor` 命令查看原始数据和错误码
2. err=0 正常, err=1 I2C通信失败, err=2 数据越界
3. 检查 PD4 (I2C电子开关) 是否正常

### Q: 阀门不动作

1. `valve:open` / `valve:close` 测试
2. 检查 PA5 (电机使能), PA11/PA12 (限位开关)
3. `delay_get` 确认延时配置

### Q: OTA 升级失败

1. 4G OTA: 检查 URL 可达性, 确认 version > 当前版本
2. BLE OTA: 确认 PD2=LOW (手机已连接), 检查帧 CRC
3. Debug 输出 `[OTA]` 前缀日志定位具体失败步骤

### Q: 功耗偏高

1. `vlp` 查看低电量状态
2. 确认非 long 模式下 4G 确实断电 (PB5=LOW)
3. 确认 BLE 未意外开启 (PC12=LOW)
4. 确认 LCD 超时关闭正常

### Q: 如何修改采集/上报周期

```
{id:collect_cycle,once:5,type:minute}
{id:report_cycle,once:30,type:minute}
```
或通过平台下发 `collectspeed_set` / `report_set` 命令。

---

## 附录: 默认配置汇总

| 参数 | 默认值 |
|------|--------|
| 采集周期 | 10 分钟 |
| 上报周期 | 1 小时 |
| 窗口模式 | long |
| 实时采集 | 启用, 5秒 |
| 异常策略 | highorlow |
| LCD | 启用 |
| 传感器数 | 2 (双路) |
| MQTT心跳 | 300秒 |
| 阈值 | 全部禁用 (switch=0) |
| 阀门延时 | 禁用 |
| 固件版本号 | 3 |

---

*文档更新日期: 2025-07*
