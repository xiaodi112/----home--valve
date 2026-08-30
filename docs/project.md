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
| USART0 | PC4(TX), PC5(RX) | Debug 调试串口 | 115200 |
| USART1 | PA2(TX), PA3(RX) | CH4 激光甲烷传感器 | 115200 |
| LPUART0 | PD8(TX), PD9(RX) | 4G 模组通信 | 115200 |
| UART4 | PB3(TX), PB4(RX) | BLE 蓝牙模组 | 115200 |

### 2.2 GPIO 控制

| 引脚 | 功能 | 说明 |
|------|------|------|
| PA0 | 4G_DTR | HIGH=允许EC801E休眠, LOW=强制唤醒 |
| PA1 | 4G_RI | 4G 唤醒输入, EXTI1 下降沿(与LPUART互为冗余) |
| PA5 | M_SLP | 电机驱动使能(LOW=关闭); 当前 `MOTOR_ENABLED=0` 未使用 |
| PA6/PA7 | 电机方向 | GPIO 直驱 或 TIMER2 CH0/CH1 PWM(`MOTOR_DRV_MODE`); 当前未使用 |
| PA11 | K1-S | 开阀到位限位开关(LOW=到位); 仅 `MOTOR_ENABLED=1` 时配置 |
| PA12 | K2-S | 关阀到位限位开关(LOW=到位); 同上 |
| PA15 | LED1 | 指示灯 |
| PB0/PB1 | SLCD SEG5/6 | 段码LCD |
| PB5 | V4G_EN | 4G电源使能(HIGH=上电) |
| PB8 | BUZZER | 蜂鸣器(HIGH=响) |
| PB9 | SLCD COM3 | 段码LCD |
| PB10-PB15 | SLCD SEG10-15 | 段码LCD |
| PC3 | SENSOR_PWR | 压力/温度传感器供电+上拉开关(HIGH=上电); `SENSOR_PWR_ENABLED=1` 时启用 |
| PC6-PC11 | SLCD SEG24-29 | 段码LCD |
| PC12 | SW_BT | 蓝牙电源开关(HIGH=上电) |
| PC13 | CH4_PWR | 激光甲烷传感器电源开关(HIGH=上电); `LASER_CH4_ENABLED=1` 时启用 |
| PC14/PC15 | LXTAL | 32.768kHz外部晶振(RTC/SLCD) |
| PD0 | Btn1 | 按键(外部上拉, EXTI0唤醒); `BUTTON_ENABLED=1` 时启用 |
| PD2 | BLE_LINK | BLE连接状态(LOW=已连接) |

### 2.3 I2C 总线 (三组独立软件 I2C)

| 通道 | 引脚 | 设备 | 说明 |
|------|------|------|------|
| IIC0 | PB6(SCL), PB7(SDA) | EEPROM (AT24C64 8KB / M24M01 128KB) | 常供电+常上拉,与传感器电源独立 |
| IIC1 | PD4(SCL), PD5(SDA) | PT304D 传感器 S2 (中压/进气端) | 由 PC3 供电+上拉 |
| IIC2 | PC0(SCL), PC1(SDA) | PT304D 传感器 S1 (低压/出气端) | 由 PC3 供电+上拉 |

> 三组总线相互独立,采集后传感器总线(PD4/PD5、PC0/PC1)转 Analog 杜绝漏电;EEPROM 总线(PB6/PB7)由驱动按需初始化(开漏+死锁恢复)。

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
                  │                       ├→ STATE_BLE_OTA               │
                  │                       └→ STATE_CH4                   │
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
| **STATE_CH4** | 激光甲烷传感器采集: 上电→预热(20s)→采集→阈值判断→上报→断电。由 `g_ch4_request`(平台 `gas_get` 触发)或 `laser_time` 位图命中整点时,在主循环 `STATE_SLEEP` 分支转入,不抢占网格采集/整点上报 |

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

### 6.9 激光甲烷 (CH4) 传感器

> 由编译宏 `LASER_CH4_ENABLED=1` 启用(当前默认启用);`=0` 时相关代码不编译,平台下发 `gas_get/laser_get/laser_set/msh_get/msh_set` 时直接忽略。

| 项 | 说明 |
|------|------|
| **硬件** | 激光甲烷传感器,USART1 (PA2/PA3, 115200),电源开关 PC13 (`CH4_PWR`) |
| **驱动** | `Drivers/ch4_sensor.c` — 独立、可移植、零上层依赖 |
| **业务编排** | `Middle/ch4_alarm.c` — 状态机(`do_ch4()` 入口),阈值判断 |
| **帧格式(型号A)** | `A+000.00 +27.4 00 60\r\n` (22B),XOR 校验 bytes[0..17] |
| **帧格式(原型号)** | `+000.00 +21.4 1001.01 00 28\r\n` (29B),5 字段含压力 |
| **预热** | 默认 20s (`DEF_CH4_WARMUP_SEC`),可通过 `msh_set` 调整 |
| **采集超时** | 2s (`CH4_COLLECT_TIMEOUT_MS`) |
| **阈值** | `g_ch4_threshold.threshold` (%VOL),浓度 ≥ 门限 → 报警(`CH4_ALARM_ALARM=1`) |
| **触发方式** | ① 平台 `gas_get` 即时触发(`g_ch4_request` 置位 → 主循环转入 `STATE_CH4`);② `laser_time` 24h 位图命中整点定时触发(`laser_switch=1` 时启用) |
| **不抢占** | 网格采集/整点上报优先,`STATE_CH4` 仅在 `STATE_SLEEP` 分支被转入 |

### 6.10 定时开关阀

> 由 `closeoropen_set` / `closeoropen_get` 平台指令配置,持久化于 EEPROM 区块 11 (ValveTiming, Magic 0xC8)。

| 字段 | 说明 |
|------|------|
| `on_switch` / `off_switch` | 定时开阀/关阀开关 (0=关, 1=开) |
| `open_time` / `close_time` | YYMMDDHHMM 格式,全 0=每天 00:00(通配) |

匹配逻辑在 `Middle/valve_timing.c`(纯逻辑,无硬件依赖)。

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
| `valve_get` | 查询阀门最后指令 | valve_last_order |
| `closeoropen_get` | 查询定时开关阀配置 | on_switch/off_switch/open_time/close_time |
| `table_get` | 查询表号 | table |
| `gas_get` | 触发激光甲烷即时采集 | methane/mtemp/mstate (+ alarm) |
| `laser_get` | 查询激光定时采集配置 | laser_switch, laser_time |
| `msh_get` | 查询甲烷阈值配置 | warmup_sec, threshold |

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
| `closeoropen_set` | 设置定时开关阀 | on_switch/off_switch/open_time/close_time |
| `table_set` | 设置表号 | table |
| `laser_set` | 设置激光定时采集 | laser_switch + laser_time(24h 位图) |
| `msh_set` | 设置甲烷阈值 | warmup_sec + threshold(%VOL) |
| `restart` | 远程重启 | — |
| `restart_off` | 关阀后重启 | — |
| `4Gpower_off` | 关闭4G | — |

---

## 8. Debug 串口调试命令

### 8.0 串口参数与调用入口

| 项 | 说明 |
|------|------|
| **物理串口** | USART0,引脚 PC4(TX)/PC5(RX),115200 波特率,8N1 |
| **换行符** | 命令以 `\r\n`(回车+换行)结尾,常见串口工具默认即此格式 |
| **运行期入口** | `debug_check_cmd()`(main.c,主循环每轮非阻塞调用) |
| **上电配置入口** | `config_enter_setup_mode()`(eeprom_config.c,MQTT 未配置时阻塞 30s) |
| **分发架构** | 需硬件上下文的命令(status/sensor/sim/valve 等)在 main.c 处理;其余命令委托 `config_parse_and_apply()`(eeprom_config.c)统一解析。main.c 未匹配时回传 0,再由 eeprom_config.c 处理;两者都未匹配则打印 `[DBG] Unknown:` |
| **缓冲区** | 两处各持一块 512B 静态缓冲(line_buf / s_cfg_line_buf),互斥调用不并发 |

> 使用方法:用 USB-TTL 串口工具连接 PC4/PC5(注意交叉:工具 RX 接板子 PC4 TX),打开串口终端(如 SecureCRT/PUTTY/串口调试助手),波特率 115200,输入命令回车即可。运行期任意时刻可输入;上电 30s 配置窗口内也可输入。

### 8.1 实时查询命令(只读,不改配置)

#### 8.1.1 系统总览

**`status`** — 打印状态机、唤醒计数、缓冲区、传感器、阀门、4G/BLE 综合状态。

输出示例:
```
[DBG] status
  state=SLEEP wakeup=12 buf=3
  P=101.35 T=25.40 V=3.95 last_Pc=101.30 last_Pr=101.32
  valve=1 abnormal=highorlow
  4G: user_off=0 init_fail=0 weak_sig=0
  BLE: on=0
  PT304D err=0 stat=0x02
```

字段解释:
- `state`:当前主状态机状态(INIT/SLEEP/WAKEUP/COLLECT/UPLOAD/DOWNLINK/ALARM/BLE_OTA)
- `wakeup`:累计唤醒次数(深睡唤醒计数)
- `buf`:EEPROM 历史数据缓冲区条数(待上报)
- `P/T/V`:最近一次压力(kPa)/温度(℃)/电池电压(V)
- `last_Pc/last_Pr`:上次采集压力/上次实时压力(用于突变判断)
- `valve`:阀门最后指令(1=开,0=关)
- `4G user_off/init_fail/weak_sig`:4G 用户关断/初始化失败/弱信号标志
- `PT304D err/stat`:传感器错误码与状态寄存器原始值

使用场景:故障排查首选命令,一眼判断设备处于什么状态、传感器是否正常、4G 是否在线。

#### 8.1.2 传感器采集与诊断

**`sensor`** — 立即触发一次 PT304D 采集 + 电池 ADC,打印原始值。err≠0 时额外打印寄存器原始值。

输出示例(正常):
```
[DBG] sensor
  err=0 stat=0x02 P=101.35 T=25.40 V=3.95
```

输出示例(I2C 故障):
```
[DBG] sensor
  err=3 stat=0x00 P=0.00 T=0.00 V=3.95
  press_raw=0x000000 temp_raw=0x000000
```

> `err=3` 表示 `PT304D_ERR_I2C`(I2C 通信失败),用于排查传感器接线或 PC3 供电。

**`sensor_status`** — 采集后只打印在线/状态布尔值,适合脚本化批量检测。

输出示例:
```
[DBG] sensor_status
  have=1 status=1 err=0 stat=0x02
```

- `have`:传感器是否在线(err≠I2C 错误=1)
- `status`:数据是否有效(err=OK=1)

**`temp_status`** — 采集后打印温度所处阈值等级。

输出示例:
```
[DBG] temp_status
  T=25.40 -> NORMAL
```

等级:`NORMAL` / `HIGH` / `LOW` / `VERY_HIGH` / `VERY_LOW`(由 `threshold.c` 根据 Tthreshold 配置判断)。

**`pressure_status`** — 采集后打印压力所处阈值等级。

输出示例:
```
[DBG] pressure_status
  P=101.35 -> NORMAL
```

等级:`NORMAL` / `HIGH` / `LOW` / `VERY_HIGH` / `VERY_LOW` / `MUTATION`(突变)。

#### 8.1.3 4G/SIM 信息

**`sim`** — 唤醒 4G 模组,查询信号与卡号信息。

输出示例:
```
[DBG] sim
  CSQ=25 RSRP=-85 SINR=12 CEREG=1
  ICCID=89860412345678901234 IMEI=861234567890123
```

字段解释:
- `CSQ`:信号质量(0-31,≥15 为良好)
- `RSRP/SINR`:LTE 参考信号功率/信噪比(dBm/dB)
- `CEREG`:网络注册状态(1=已注册)
- `ICCID/IMEI`:SIM 卡号/模组串号

使用场景:排查 4G 不在线、信号弱、SIM 卡未识别。

#### 8.1.4 时间与缓冲区

**`time`** — 显示当前 RTC 时间(用于校准时钟或排查闹钟不准)。

输出示例:
```
[DBG] time
  2026-08-28 14:30:25
```

**`buf`** — 查看 EEPROM 历史数据缓冲区条数(离线积攒的待上报记录)。

输出示例:
```
[DBG] buf
  ee_hist: 3 records
```

> 条数过多说明上报不成功,可结合 `status` 排查 4G 状态。

**`vlp`** — 查看低电压保护状态(用于排查电池异常关阀)。

输出示例:
```
[DBG] vlp
  voltage_low_power=0 vlp_uploaded=0 valve_alarm_closed=0 pre=1
  spupower en=1 vprotect=3.00
```

字段解释:
- `voltage_low_power`:当前是否处于低电压省电模式
- `vlp_uploaded`:本小时是否已上报过低电压事件
- `valve_alarm_closed`:是否因报警自动关阀
- `pre`:关阀前阀门状态(恢复用)
- `spupower en/vprotect`:电压检测开关与保护阈值

#### 8.1.5 配置查询

**`config`** — 打印当前所有配置(MQTT/周期/窗口/阈值/延时/实时采集/用户端MQTT),用于全量核对。

**`vpthreshold_get`** — 打印压力阈值通道1(进气端)。

输出示例:
```
  VPTH en=1 ppmax=500.0 pmax=400.0 ppmin=50.0 pmin=100.0 mut=200.0 shock=10.0 ka=300
```

**`Tthreshold_get`** — 打印温度阈值。
```
  TTH  en=1 high=60.0 low=-10.0 vhigh=80.0 vlow=-20.0
```

**`spupower_get`** — 打印电压检测配置。
```
  SPUP en=1 high=4.20 low=3.30 protect=3.00
```

**`delay_get`** — 打印阀门延时配置。
```
  VDLY en=1 delay_ms=500
```

**`valve_status`** — 打印阀门最后指令(1=开,0=关)。
```
  valve_last_order=1
```

**`ip`** — 打印主 IP / 副 IP。
```
  IP1: 120.78.220.171:1883  IP2: 192.168.1.200:1884
```

**`umqtt_get`** — 打印用户端 MQTT 配置。
```
  UMQTT server=192.168.1.100 port=13000 user=admin client=device01 topic=/custom/topic
```

### 8.2 控制命令(改变设备行为)

#### 8.2.1 阀门控制

**`valve:open`** / **`valve:close`** — 执行开阀/关阀动作,并清除报警自动关阀标志(用户手动覆盖)。

输出示例:
```
[DBG] valve:open
  cleared alarm_closed flag (user override)
  valve(open) -> OK  last_order=1
```

> 注意:需 `MOTOR_ENABLED=1` 才有实际电机动作;当前产品 `MOTOR_ENABLED=0` 时只更新 `g_valve_last_order` 状态,无物理动作。

#### 8.2.2 4G 电源控制

**`4Gpower_off`** — 关闭 4G 模组电源(PB5 拉低),上报状态后置 `fourG_user_off=1`,主循环不再尝试联网。
```
[DBG] 4Gpower_off
  4G power off
```

**`4Gpower_on`** — 重新上电并初始化 4G,成功后上报状态并设 PSM。
```
[DBG] 4Gpower_on
  4G power on
  4G power on OK, reported
```

> 失败时打印 `4G power on FAILED`,可用 `sim` 命令进一步排查。

#### 8.2.3 MQTT 与复位

**`mqtt_reconnect`** — 断开 MQTT 并重连,重新订阅主题。
```
[DBG] mqtt_reconnect
  MQTT reconnecting...
  MQTT reconnected OK
```

**`restart`** — MCU 软件复位(`NVIC_SystemReset`),不改变 EEPROM 配置。
```
[DBG] restart
[DBG] Restarting...
```

**`reset`** — 恢复出厂设置:清除 EEPROM 所有 Magic 字节后重启,重启后因 MQTT Magic 无效自动进入 30s 配置等待模式。
```
[DBG] reset
[CFG] Factory reset, rebooting...
```

> 区别:`restart` 只重启不擦配置;`reset` 清空所有持久化配置(阈值/周期/MQTT 全部恢复默认)。

#### 8.2.4 IP 修改(立即重连)

**`first_ip:IP.port`** — 修改主 MQTT 服务器,格式用 `.` 分隔端口(最后一个点后为端口)。

举例:
```
first_ip:120.78.220.171.1883
```
行为:解析出 IP=120.78.220.171、port=1883,保存到 EEPROM,断开 MQTT 后重连+重新订阅。输出:
```
  Primary IP set: 120.78.220.171:1883, reconnecting...
```

**`scend_ip:IP.port`** — 修改副 IP(故障切换用),格式同上,只保存不重连。
```
scend_ip:192.168.1.200.1884
  Secondary IP set: 192.168.1.200:1884
```

> 格式错误(找不到最后一个点)时提示 `Invalid format, use: first_ip:IP.port`。

### 8.3 配置命令(JSON/key-value 格式,自动持久化到 EEPROM)

> 所有配置命令解析后立即调用 `config_save_*` 写入 EEPROM,重启不丢失。字段均为可选,只更新提供的字段。

#### 8.3.1 MQTT 主服务器配置

格式:
```
{clientId:<前缀>&<设备ID>&<产品ID>&<序号>,username:<用户名>,passwd:<密码>,IP:<服务器IP>,port:<端口>}
```

举例:
```
{clientId:S&D54GRG29T5HB0&149&1,username:FastBee,passwd:P41TQ4X1Y0C57IW0,IP:120.78.220.171,port:1883}
```

字段解释:
- `clientId`:FastBee 平台设备三元组,格式 `前缀&设备ID&产品ID&序号`,程序自动从中推导出订阅/上报 topic
- `username`/`passwd`:MQTT 登录凭据
- `IP`/`port`:MQTT broker 地址

使用方法:上电 30s 配置窗口内必须配置此项,否则超时后进入深睡;配置后自动进入 10s 可选参数窗口。

#### 8.3.2 用户端 MQTT 配置

格式(字段均可选):
```
{umqtt_server:<IP>,umqtt_port:<端口>,umqtt_user:<用户>,umqtt_pass:<密码>,umqtt_client:<客户端ID>,umqtt_topic:<上报主题>}
```

举例:
```
{umqtt_server:192.168.1.100,umqtt_port:13000,umqtt_user:admin,umqtt_pass:123,umqtt_client:device01,umqtt_topic:/custom/topic}
```

说明:用于将数据同时转发到用户自建 MQTT 服务器。未显式指定 topic 但有 client_id 时,自动回退拼接为 `/155/<client_id>/property/post`。

#### 8.3.3 采集/上报周期

格式:
```
{id:collect_cycle,once:<数量>,type:<单位>}
{id:report_cycle,once:<数量>,type:<单位>}
```

举例:
```
{id:collect_cycle,once:10,type:minute}   → 每 10 分钟采集一次
{id:report_cycle,once:1,type:hour}        → 每 1 小时上报一次
```

`type` 可选值:`sec` / `minute` / `hour` / `day` / `week`。

> 内部统一换算为小时数存入 `collect_interval_hours` / `upload_interval_hours`,并重算 RTC 闹钟间隔。

#### 8.3.4 窗口模式

格式:
```
{id:window_pattern,value:<模式>,start:<HHMMSS>,stop:<HHMMSS>,end_result:<0|1>,dottime:<小时列表>}
```

举例:
```
{id:window_pattern,value:short,start:080000,stop:200000,end_result:1,dottime:3}
```

字段解释:
- `value`:窗口模式 `long`(全时在线)/ `short`(时段在线)/ `dot`(定点采集)
- `start`/`stop`:short 模式的起止时间(HHMMSS)
- `end_result`:窗口结束时是否强制上报(1=是)
- `dottime`:dot 模式的采集小时(0-23),多个用逗号分隔(如 `3,8,20`)

#### 8.3.5 异常关阀策略

格式:
```
{id:abnormal,value:<策略>}
```

举例:
```
{id:abnormal,value:highorlow}
```

可选值:`none`(不关阀)/ `high`(超高关阀)/ `low`(超低关阀)/ `highorlow`(超限关阀)。

#### 8.3.6 阈值配置

**压力阈值通道1**(进气端):
```
{id:vpthreshold,switch:1,ppmax:500.0,pmax:400.0,ppmin:50.0,pmin:100.0,mutation:200.0,shock:10.0}
```

字段解释:
- `switch`:阈值检测开关(1=启用,0=禁用)
- `ppmax`/`pmax`:超高压力告警阈值/严重阈值
- `ppmin`/`pmin`:超低压力告警阈值/严重阈值
- `mutation`:突变检测阈值(相邻采集差值)
- `shock`:冲击(瞬时突变)阈值

**温度阈值**:
```
{id:Tthreshold,switch:1,high:60.0,low:-10.0,vhigh:80.0,vlow:-20.0}
```

**电压检测**:
```
{id:spupower,switch:1,high:4.2,low:3.3,protect:3.0}
```

- `high`/`low`:电压告警上下限,`protect`:低电压保护阈值(触发 VLP 省电模式)。

**阀门延时**(开/关阀动作前的延时):
```
{id:delay,switch:1,delay_ms:500}
```

> `delay_ms` 上限 5000ms,超出自动钳制。

**MQTT 心跳**:
```
{id:keepalive,value:300}
```

> 心跳下限 30s,低于此值自动提升为 30s。

#### 8.3.7 实时采集开关

格式:
```
real_set=<开关>[,<频率>]
```

举例:
```
real_set=1,5    → 开启实时采集,每 5 秒一次
real_set=0      → 关闭实时采集
real_set=1      → 开启,频率保持不变
```

说明:实时采集开启后,RTC 闹钟缩短为 1-10 秒,每次唤醒只采集+阈值判断不上报,用于压力波动监测。频率范围 1-10 秒。

#### 8.3.8 LCD 开关

格式:
```
lcd_switch=<0|1>
```

举例:
```
lcd_switch=1    → 开启 LCD 显示
lcd_switch=0    → 关闭 LCD(省电)
```

> 持久化到 EEPROM(`g_lcd_enabled`),重启后保持。与 `LCD_LOCK_ENABLED` 编译宏配合:宏=1 时 LCD 超时关屏,宏=0 时常亮。

---

## 9. 配置管理 (EEPROM)

### 9.1 存储芯片

EEPROM 芯片由 `app_config.h` 编译宏二选一(当前启用 AT24C64):
- `EEPROM_DRV_AT24C64` — AT24C64 (8KB) **当前默认**
- `EEPROM_DRV_M24M01` — M24M01 (128KB)

均走 IIC0 (PB6/PB7) 独立总线,常供电、常上拉,与传感器电源(PC3)无关。

### 9.2 存储区域

| 区域 | Magic | 地址 | 内容 |
|------|-------|------|------|
| 间隔配置 | 0xA5 | 0x000 | collect_interval, upload_interval |
| MQTT | 0x5A | 0x020 | client_id, username, password, server, port, topic |
| 压力阈值1 | 0xC0 | 0x1B0 | vpthreshold (进气端) + keepalive |
| 温度阈值1 | 0xC1 | 0x1D0 | Tthreshold (进气端) |
| 电压检测 | 0xC2 | 0x1F0 | spupower |
| 扩展配置 | 0xB7 | 0x200 | window/cycle/abnormal/valve/real/副IP/lcd/sensor_count/vlp_flag 等(见 §9.3) |
| 阀门延时 | 0xC3 | 0x290 | delay enabled + ms |
| 压力阈值2 | 0xC4 | 0x2A0 | vpthreshold2 (出气端) |
| 温度阈值2 | 0xC5 | 0x2C0 | Tthreshold2 (出气端) |
| 用户端MQTT | 0xC6 | 0x300 | user mqtt server/port/user/pass/client/topic |
| 定时开关阀 | 0xC8 | 0x490 | on_switch/off_switch/open_time/close_time (YYMMDDHHMM) |
| CH4 阈值 | 0xC9 | 0x4B0 | warmup_sec + threshold(%VOL); enabled 字段仅作存储兼容 |
| 激光定时采集 | 0xCA | 0x4C0 | laser_switch + laser_time(24h 位图 3B) |

### 9.3 扩展配置区(0x200)新增字段

| 地址 | 字段 | 说明 |
|------|------|------|
| 0x240-0x27F | MQTT_SERVER2 / PORT2 | 副IP 故障切换 |
| 0x282 / 0x283 | real_enabled / real_frequency | 实时采集开关与频率 |
| 0x284 | valve_alarm_closed | 报警自动关阀标志(OTA/重启后自动恢复) |
| 0x285 | valve_pre_alarm_order | 关阀前阀门状态(恢复用) |
| 0x286 | lcd_enabled | LCD 显示开关(EEPROM 持久化) |
| 0x287 | sensor_count | 传感器数量 (1=单路, 2=双路) |
| 0x288 | vlp_flag | 低电压省电模式标志 |
| 0x2E0 | window_dottime_mask | dot 模式 24h 位图(3B,覆盖 0x21A 旧单点) |

### 9.4 加载逻辑

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

工程按四层分层(App / Middle / BSP / Drivers),依赖方向严格向下,详见 `docs/工程架构整理.md`。

```
Pressure_Regulating_Box/
├── App/                          业务层(不碰寄存器/外设)
│   ├── valve.c/h                 阀门业务: 开/关/状态查询、到位超时、状态持久化
│   ├── lcd_ui.c/h                LCD 页面/图标、蓝牙开关(走 power_ble_set)
│   ├── buzzer.c/h                蜂鸣器提示音(调 output API)
│   └── led.c/h                  LED 指示(调 output API)
│
├── Middle/                       中间层(业务编排、协议、配置、上报)
│   ├── main.h                    主模块接口: g_sensor、lcd_shutdown、alarm_try_close_valve
│   ├── app_config.h              编译期业务默认值、枚举定义
│   ├── downlink.c/h              下行指令: parse_cmd + s_cmd_handlers 表驱动分发(44 项)
│   ├── report.c/h                统一上报唯一出口(report_publish_*)
│   ├── eeprom_config.h           配置门面头: EEPROM 地址/Magic/结构体/extern/接口
│   ├── config_store.c/h          配置持久化: EE 读写、config_load_all、config_save_*
│   ├── config_policy.c/h         配置纯策略: dottime 位图、周期/异常转换
│   ├── eeprom_config.c           配置 CLI: 串口命令解析、上电配置等待、配置打印
│   ├── threshold.c/h             阈值判断(双通道,纯业务)
│   ├── valve_timing.c/h         定时开关阀时间匹配(纯逻辑)
│   ├── ch4_alarm.c/h             激光甲烷状态机(do_ch4 入口)+ 阈值判断
│   ├── json_builder.c/h         轻量 JSON 构建器
│   ├── ota_app.c/h               4G OTA 流程: 下载→缓存→InfoBlock→重启
│   ├── eeprom_history.c/h        历史数据存取
│   ├── sensor_record.h          传感器数据结构定义
│   └── ota_info.h                Flash 分区定义、InfoBlock 结构
│
├── BSP/                          板级驱动(自包含、零业务依赖)
│   ├── power.c/h                 电源管理: 4G_EN/BLE_EN/SENSOR_PWR/CH4_PWR/DTR
│   ├── output.c/h                蜂鸣器/LED 输出
│   ├── motor.c/h                 电机: 限位/方向/使能/引脚宏(MOTOR_ENABLED=0 时空实现)
│   ├── wakeup.c/h                深睡入口/唤醒重初始化
│   ├── mygpio.c/h                GPIO 初始化、按键 EXTI、低功耗引脚归位
│   ├── usart.c/h                 USART0/USART1(CH4)/LPUART0/UART4: 初始化、DMA、打印
│   ├── tim.c/h                   RTC 时间/闹钟
│   ├── delay.c/h                 延时 + 软件看门狗
│   ├── systick.c/h               系统滴答
│   ├── adc_bat.c/h               电池电压 ADC
│   └── slcd_seg.c/h              段码 LCD 底层 + 关屏释放引脚
│
├── Drivers/                      外设驱动
│   ├── soft_i2c.c/h              软件 I2C(三组独立总线 IIC0/1/2)
│   ├── ec800e.c/h                4G AT/MQTT/HTTP(仍混业务,待主题 B 拆分)
│   ├── ble_ota.c/h               BLE OTA 协议(仍含 YLY_<ID> 名称配置业务)
│   ├── PT_304_35.c/h             PT304D 压力/温度传感器驱动
│   ├── ch4_sensor.c/h            激光甲烷传感器驱动(独立、可移植)
│   ├── at24c64.c/h               AT24C64 EEPROM 芯片驱动(当前默认)
│   └── m24m01.c/h                M24M01 EEPROM 芯片驱动(备选)
│
├── Projects/Wakeup_box_V0.0.1/   顶层
│   ├── main.c                    主状态机编排、启动初始化、休眠/唤醒、4G 策略、调试命令
│   ├── gd32l23x_it.c             中断服务(仅置唤醒标志)
│   └── gd32l23x_libopt.h         库选项配置
│
└── Bootloader/boot_main.c        启动引导 + OTA 搬运
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

| 参数 | 默认值 | 来源 |
|------|--------|------|
| 采集周期 | 10 分钟 | `APP_DEFAULT_COLLECT_*` |
| 上报周期 | 1 小时 | `APP_DEFAULT_REPORT_*` |
| 窗口模式 | long, end_result=1 | `APP_DEFAULT_WINDOW_*` |
| 实时采集 | 启用, 5秒 | `APP_DEFAULT_REAL_*` |
| 异常策略 | none | `APP_DEFAULT_ABNORMAL_CLOSE` |
| LCD | 启用, LCD_LOCK=0(常亮+自动翻页) | `APP_DEFAULT_LCD_ENABLED` / `LCD_LOCK_ENABLED` |
| 传感器数 | 2 (双路) | `APP_DEFAULT_SENSOR_COUNT` |
| MQTT心跳 | 300秒 | `APP_MQTT_KEEPALIVE` |
| 阈值 | 全部禁用 (switch=0) | `DEF_VP_*` / `DEF_TT_*` / `DEF_SP_*` |
| 阀门延时 | 禁用 | `DEF_VD_SWITCH` |
| 激光甲烷 | 启用 (`LASER_CH4_ENABLED=1`), 预热 20s, 阈值 0.5%VOL, 定时 08:00/20:00 | `DEF_CH4_*` / `DEF_LASER_*` |
| 电机 | 禁用 (`MOTOR_ENABLED=0`) | `app_config.h` |
| EEPROM 芯片 | AT24C64 (8KB) | `EEPROM_DRV_AT24C64` |
| 固件版本号 | 1 | `APP_FIRMWARE_VERSION` / `APP_FW_VERSION_NUM` |

---

*文档更新日期: 2026-08-28(基于当前代码逐项核对)*
