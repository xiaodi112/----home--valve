# 家用阀门物联网控制�?�?工程说明文档

> **MCU**: GD32L233RC (Cortex-M23, 256KB Flash, 32KB SRAM)  
> **开发环�?*: MDK-ARM (Keil)  
> **固件版本**: V2  
> **文档日期**: 2026-06-30

---

## 一、系统概�?
本项目是一款基�?GD32L233RC 的家用阀门物联网控制器，支持 **双路压力/温度传感器采�?*�?*电动阀门控�?*�?*4G (EC800E) 双平�?MQTT 上云**�?*LPUART 深休眠唤�?*�?*秒级实时监测**�?*OTA 空中升级** 等功能。系统采�?**状态机驱动 + 深休眠低功耗架�?*，通过 RTC 闹钟 / LPUART 接收 / 按键三种方式唤醒�?
### 核心特性一�?
| 特�?| 说明 |
|------|------|
| 双路传感�?| PT304D ×2 (进气端中�?+ 出气端低�?，可配置为单�?|
| 双平�?MQTT | 主平�?(id=0, FastBee) + 用户平台 (id=1, 可�? 双连接上�?|
| 秒级实时监测 | `g_real_enabled` 开启后，休眠期内按 1~10s 频率唤醒采集并即时判异常 |
| 低功耗调�?| 深休�?+ 三类唤醒�?+ 窗口模式 (long/short/dot) |
| 异常自动关阀 | �?`abnormal_close` 策略触发，OTA/重启后可持久化恢�?|
| 低电压保�?(VLP) | 电压低于保护值强制关阀 + 降频上报 (�?8:00/18:00) |
| OTA 升级 | HTTP 下载 + Bootloader 搬运 + 升级回执 |
| 产品复用配置 | LCD 显示开�?`g_lcd_enabled` + 传感器数�?`g_sensor_count`，运行时可改 |
| 4G 模块短路保护 | InitAndConnect 连续 3 轮失败后�?`s_module_dead`，跳过所�?4G 操作直至重新上电 |

### 状态机�? 个状态）

| 状�?| 功能描述 |
|------|---------|
| `STATE_INIT` | 上电初始�?�?MQTT 连接 (�?用户平台) �?订阅 �?首次采集上报 |
| `STATE_SLEEP` | 设置 RTC 闹钟 �?配置低功�?GPIO �?进入深休眠；实时模式下内嵌秒级采集循�?|
| `STATE_WAKEUP` | 恢复外设 �?判断唤醒�?RTC/LPUART/按键)，按键支持连�?长按 |
| `STATE_COLLECT` | 传感器采�?�?阈值检�?�?存入 RAM 环形缓冲�?�?判断是否上报 |
| `STATE_UPLOAD` | 批量逐条 MQTT 上报 �?用户平台上报 �?清空缓冲�?|
| `STATE_DOWNLINK` | 读取并处�?MQTT 下发指令 �?立即上报结果 |
| `STATE_ALARM` | 多次重采确认 �?报警上报 �?异常自动关阀 |

> **传感器命名注�?*：代码中 `g_sensor.raw` 对应 **PT304D #2** (PB6/PB7 独立总线, 进气�?中压)，`g_sensor.raw2` 对应 **PT304D #1** (PC0/PC1 �?EEPROM 共用总线, 出气�?低压)。即 `raw` �?传感�?，`raw2` �?传感�?，命名与物理编号相反，阅读代码时需特别留意�?
---

## 二、引脚分配总表

### 2.1 GPIOA

| 引脚 | 功能 | 方向/模式 | 说明 |
|------|------|----------|------|
| PA0 | 4G_DTR | 推挽输出 | EC800E DTR 控制引脚。HIGH = 允许模块休眠, LOW = 强制唤醒 |
| PA5 | M_SLP | 推挽输出 | 电机驱动使能 (DRV8837)。HIGH = 使能, LOW = 休眠关闭 |
| PA6 | TIMER2_CH0 (PWM) | AF1 复用 | 电机 PWM 通道 0 (关阀方向) |
| PA7 | TIMER2_CH1 (PWM) | AF1 复用 | 电机 PWM 通道 1 (开阀方向) |
| PA8 | SLCD_COM0 | AF3 复用 | 段码液晶 COM0 |
| PA9 | SLCD_COM1 | AF3 复用 | 段码液晶 COM1 |
| PA10 | SLCD_COM2 | AF3 复用 | 段码液晶 COM2 |
| PA11 | K1-S (限位1) | 输入上拉 | 开阀到位检测，低电�?= 到位 |
| PA12 | K2-S (限位2) | 输入上拉 | 关阀到位检测，低电�?= 到位 |
| PA15 | LED1 | 推挽输出 | 状态指示灯。HIGH = �? LOW = �?|

### 2.2 GPIOB

| 引脚 | 功能 | 方向/模式 | 说明 |
|------|------|----------|------|
| PB0 | SLCD_SEG5 | AF3 复用 | 段码液晶 SEG5 |
| PB1 | SLCD_SEG6 | AF3 复用 | 段码液晶 SEG6 |
| PB6 | I2C_SCL (传感�?) | 开漏输出上�?| 软件 I2C 时钟�?�?PT304D #2 独立总线 |
| PB7 | I2C_SDA (传感�?) | 开漏输出上�?| 软件 I2C 数据�?�?PT304D #2 独立总线 |
| PB8 | BUZZER | 推挽输出 | 蜂鸣器控制。HIGH = �? LOW = �?|
| PB9 | SLCD_COM3 | AF3 复用 | 段码液晶 COM3 |
| PB10 | SLCD_SEG10 | AF3 复用 | 段码液晶 SEG10 |
| PB11 | SLCD_SEG11 | AF3 复用 | 段码液晶 SEG11 |
| PB12 | SLCD_SEG12 | AF3 复用 | 段码液晶 SEG12 |
| PB13 | SLCD_SEG13 | AF3 复用 | 段码液晶 SEG13 |
| PB14 | SLCD_SEG14 | AF3 复用 | 段码液晶 SEG14 |
| PB15 | SLCD_SEG15 | AF3 复用 | 段码液晶 SEG15 |

### 2.3 GPIOC

| 引脚 | 功能 | 方向/模式 | 说明 |
|------|------|----------|------|
| PC0 | I2C_SCL (传感�?/EEPROM) | 开漏输出上�?| 软件 I2C 时钟�?�?PT304D #1 + M24M01 共用总线 |
| PC1 | I2C_SDA (传感�?/EEPROM) | 开漏输出上�?| 软件 I2C 数据�?�?PT304D #1 + M24M01 共用总线 |
| PC2 | AD_BAT (ADC_CH12) | 模拟输入 | 电池电压检测。分压比 2:1 (R3=2MΩ, R5=2MΩ) |
| PC4 | USART0_TX | AF7 复用上拉 | 调试串口发�?(115200 bps) |
| PC5 | USART0_RX | AF7 复用上拉 | 调试串口接收 (115200 bps) |
| PC6 | SLCD_SEG24 | AF3 复用 | 段码液晶 SEG24 |
| PC7 | SLCD_SEG25 | AF3 复用 | 段码液晶 SEG25 |
| PC8 | SLCD_SEG26 | AF3 复用 | 段码液晶 SEG26 |
| PC9 | SLCD_SEG27 | AF3 复用 | 段码液晶 SEG27 |
| PC10 | SLCD_SEG28 | AF3 复用 | 段码液晶 SEG28 |
| PC11 | SLCD_SEG29 | AF3 复用 | 段码液晶 SEG29 |
| PC12 | SW_BT | 推挽输出 | 蓝牙模块电源开关。HIGH = 上电, LOW = 断电 (默认断电) |
| PC14 | LXTAL_IN | 晶振 | 外部 32.768kHz 低速晶�?(RTC 时钟�? |
| PC15 | LXTAL_OUT | 晶振 | 外部 32.768kHz 低速晶�?|

### 2.4 GPIOD

| 引脚 | 功能 | 方向/模式 | 说明 |
|------|------|----------|------|
| PD0 | Btn1 (按键1) | 输入 (内部上拉) | 用户按键，按�?= LOW。EXTI0 下降沿唤醒深休眠 |
| PD1 | Btn2 (按键2) | 输入 (内部上拉) | 预留按键，防悬空 |
| PD4 | SW_IIC | 推挽输出 | I2C 传感�?EEPROM 电子开关。HIGH = 上电, LOW = 断电 (节能) |
| PD6 | SLCD_VLCD | 模拟输入 | SLCD 外部电压�?|
| PD8 | LPUART0_TX | AF8 复用上拉 | 低功耗串口发�?�?4G 模块 (115200 bps) |
| PD9 | LPUART0_RX | AF8 复用上拉 | 低功耗串口接�?�?4G 模块 (115200 bps)，支持深休眠唤醒 |

### 2.5 GPIOF

| 引脚 | 功能 | 方向/模式 | 说明 |
|------|------|----------|------|
| PB5 | V4G_EN | 推挽输出 | 4G 模块 (EC800E) 总电源控制。HIGH = 上电, LOW = 断电 (默认断电) |

---

## 三、功能模块详�?
### 3.1 串口通信模块 (`usart.c / usart.h`)

| 外设 | 引脚 | 波特�?| 用�?|
|------|------|--------|------|
| USART0 | PC4(TX) / PC5(RX) | 115200 | 调试串口 �?日志输出 + 在线配置 |
| LPUART0 | PD8(TX) / PD9(RX) | 115200 | 4G 通信串口 �?AT 指令 + MQTT 数据 |

**LPUART0 特�?*:
- **DMA 循环接收** (DMA_CH1)�?024 字节环形缓冲区，零中断高吞吐
- **IDLE 帧检�?*：中断检测帧结束，支�?OTA 整帧接收
- **深休眠唤�?*：起始位唤醒模式 (EXTI28)，可在深休眠中接�?MQTT 下发

**USART0 特�?*:
- RBNE 中断接收�?56 字节环形缓冲
- 支持行级读取 (带超�?，用于调试命令解�?
### 3.2 4G 通信模块 (`ec800e.c / ec800e.h`)

- **模块型号**: 移远 EC800E (CAT1)
- **电源控制**: PB5 (V4G_EN) 控制总电源开�?- **DTR 控制**: PA0 控制模块休眠/唤醒
- **通信方式**: LPUART0 AT 指令
- **省电模式**: 支持 QSCLK 轻睡�?/ PSM 深度省电

**核心功能**:
- MQTT 双服务器连接 (�?�?IP 自动切换)
- 传感器数据上�?(JSON 格式)
- MQTT 下发消息订阅与处�?- OTA 升级固件 HTTP 下载
- 网络时间同步 (AT+CCLK? �?RTC)
- 信号强度/ICCID/IMEI 查询

**双平�?MQTT 架构**:

| 平台 | 连接 ID | 配置来源 | 说明 |
|------|---------|---------|------|
| 主平�?| `id=0` | `g_mqtt` (区域2) | FastBee 平台，常驻连接，接收下发指令 + 全量数据上报 |
| 用户平台 | `id=1` | `g_user_mqtt` (区域10) | 可选第二平台，`EC801E_UserMqtt_IsConfigured()` 判空，仅上报传感器数�?|

- 用户平台�?`STATE_INIT` �?`STATE_UPLOAD` 末尾通过 `EC801E_UserMqtt_PublishSensorData()` 上报，发布后立即 `Disconnect` 释放连接，节省模块资源�?- 主平台主/�?IP 故障切换：`server`/`server2` + `port`/`port2`，副 IP 为空时禁用切换�?
**模块短路保护 (`s_module_dead`)**:
- `EC801E_InitAndConnect()` 连续 `APP_EC_MAX_RETRY`(=3) 轮重试全部失败后置位 `s_module_dead`�?- 置位后，所�?4G 操作 (`EnsureConnected/PublishXxx/QueryCSQ/SetPSM/...`) 立即返回 `EC_ERROR` 短路跳过，避免在无网络环境下反复阻塞拖垮功耗与喂狗�?- �?`EC801E_PowerOn()` 清除该标�?(新电源周期重新尝�?�?
### 3.3 压力/温度传感器模�?(`PT_304_35.c / PT_304_35.h`)

- **传感器型�?*: PT304D 数字压力/温度传感�?- **通信协议**: 软件模拟 I2C
- **双传感器配置**:

| 传感�?| I2C 总线 | 引脚 | 备注 |
|--------|---------|------|------|
| PT304D #1 | PC0(SCL) / PC1(SDA) | `i2c_sensor.c` | �?M24M01 共用总线 |
| PT304D #2 | PB6(SCL) / PB7(SDA) | `i2c_eeprom.c` | 独立总线 |

- **统一电源开�?*: PD4 (SW_IIC)，采集前上电，完成后断电
- **采集流程**: PD4 上电 �?等待 50ms �?发送测量命�?(0xAA) �?等待 15ms �?读取 7 字节数据
- **数据内容**: 压力 (24bit) + 温度 (24bit) + 状态字
- **量程**: 35kPa / 600kPa 可选，温度 -40�?~ +125�?
**I2C 总线健壮性措�?*:
- **ACK 超时轮询**: `IIC_Wait_Ack` 循环检�?SDA 最�?1000 次再判定 NACK (双总线一�?
- **死锁恢复**: `IIC_Init` / `IIC_GPIO_Init1` 初始化时发�?9 �?SCL 脉冲，释放从机占用的 SDA
- **在线/离线去抖**: 连续 `SENSOR_OFFLINE_DEBOUNCE`(=3) 次采集失败才判定离线，上线立即恢�?
**统一采集接口 `PT304D_CollectAll(p_s1, p_s2)`**:
- 一次性采集两路传感器，PD4 仅上/断电一次，提高效率。任一指针�?`NULL` 即跳过对应传感器�?- **传感器数量可配置**：所有调用点统一�?`PT304D_CollectAll((g_sensor_count >= 2) ? &raw2 : NULL, &raw)`。当 `g_sensor_count == 1` 时跳过出气端 (PT304D #1) 采集，`raw2` 数据保持零值，单路产品可省去第二路传感器�?
### 3.4 EEPROM 存储模块 (`M24M01.c / M24M01.h / i2c_sensor.c`)

- **芯片型号**: M24M01 (16Kbit = 2048 字节)
- **通信方式**: 软件 I2C (PC0/PC1)，与 PT304D #1 共用总线
- **页大�?*: 16 字节，写周期 5ms
- **上电总线恢复**: `M24M01_pwr_on()` 中调�?`IIC_Init()` 确保 GPIO 重配 + 9-SCL 死锁恢复
- **存储分区**:

| 区域 | 地址范围 | Magic | 内容 |
|------|---------|-------|------|
| 1 间隔配置 | 0x000-0x01F | 0xA5 | 采集/上报间隔 (小时) |
| 2 主MQTT配置 | 0x020-0x1AF | 0x5A | ClientID/用户�?密码/主服务器/端口/Topic |
| 3 进气端压力阈�?| 0x1B0-0x1CF | 0xC0 | 上下�?极高极低/突变/滞回(shock)/心跳 |
| 4 进气端温度阈�?| 0x1D0-0x1EF | 0xC1 | 高温/低温/极高/极低 |
| 5 电压检�?spupower) | 0x1F0-0x1FF | 0xC2 | 高压/低压/保护电压(VLP) |
| 6 扩展配置(EXT) | 0x200-0x28F | 0xB7 | 窗口/采集上报周期/异常关阀/阀门状�?副IP端口/实时开�?频率/报警关阀标志/**LCD开�?0x286)/传感器数�?0x287)** |
| 7 阀门延�?valve_delay) | 0x290-0x29F | 0xC3 | 延时开�?延时时间(ms) |
| 8 出气端压力阈�?| 0x2A0-0x2BF | 0xC4 | 出气端上下限/极高极低/突变/滞回 |
| 9 出气端温度阈�?| 0x2C0-0x2DF | 0xC5 | 出气端高�?低温/极高/极低 |
| 10 用户端MQTT配置 | 0x300-0x482 | 0xC6 | 用户平台服务�?端口/用户�?密码/ClientID/Topic |

> **EXT �?(0x200) 关键字段**：`window_pattern/start/stop/dottime`、`collect_once+type`、`report_once+type`、`abnormal_close`、`valve_last_order`、`server2+port2`(副IP)、`real_enabled+real_frequency`、`valve_alarm_closed`(报警关阀标志)、`valve_pre_alarm_order`(关阀前状�?、`lcd_enabled`(0x286)、`sensor_count`(0x287)。各 1B 字段�?0xFF 防护，无效时回落默认值�?
### 3.5 电机驱动与阀门控制模�?(`motor.c / motor.h / valve.c / valve.h`)

- **驱动芯片**: 推测 DRV8837 或类�?H 桥驱�?- **PWM 频率**: 160kHz (TIMER2)
- **占空�?*: 90% 工作

| 引脚 | 功能 |
|------|------|
| PA5 | M_SLP �?电机驱动使能 (推挽输出) |
| PA6 | TIMER2_CH0 �?PWM 通道 0 (关阀方向) |
| PA7 | TIMER2_CH1 �?PWM 通道 1 (开阀方向) |
| PA11 | K1-S �?开阀到位限位开�?(低电平到�? |
| PA12 | K2-S �?关阀到位限位开�?(低电平到�? |

**阀门控制逻辑**:
- 开阀: CH1=90%, CH0=0% �?等待 PA11(K1-S) 到位 �?到位后可延时保持 (压紧阀�?
- 关阀: CH0=90%, CH1=0% �?等待 PA12(K2-S) 到位 �?到位后可延时保持
- 超时保护: 20 秒动作超�?- 故障检�? 双限位同时触�?/ 反向限位触发 �?�?`valve_err`

### 3.6 电池电压检测模�?(`adc_bat.c / adc_bat.h`)

- **ADC 通道**: ADC_CHANNEL_12 (PC2)
- **分压电路**: VIN3V6 �?R3(2MΩ) �?AD_BAT(PC2) �?R5(2MΩ) �?GND
- **计算公式**: `V_BAT = ADC_raw × (3.3 / 4095) × 2`
- **采样时间**: 239.5 cycles (高精�?
- **低电压保�?*: 电压低于 `spupower.protect` 时触�?VLP 模式，强制关阀

### 3.7 段码液晶模块 (`slcd_seg.c / slcd_seg.h / lcd_ui.c / lcd_ui.h`)

- **显示�?*: XD-H2074A 段码液晶
- **驱动方式**: GD32L233 内置 SLCD 控制器，1/4 duty
- **时钟�?*: IRC32K (内部 32K RC 振荡�?

**引脚占用** (�?19 引脚):
- **COM**: PA8(COM0), PA9(COM1), PA10(COM2), PB9(COM3)
- **SEG**: PB0(SEG5), PB1(SEG6), PB10~PB15(SEG10~SEG15), PC6~PC11(SEG24~SEG29)
- **外部电压**: PD6 (模拟输入)

**显示功能**:
- 5 位数�?字符显示 (含字�?A-Y 和符�?-/_)
- 整数/浮点�?时间显示
- 图标显示: 充电、电�?(3 �?、信�?(3 �?、蓝牙、阀门、进�?出口、MPa/KPa/�?%/PPM/LEL 单位
- 小数点独立控�?(3 个位�?

**防闪屏架�?* (影子缓冲�?:
- `s_shadow[4]` RAM 缓冲区持有完�?SLCD_DATA0~3 镜像，所有写操作修改影子而非硬件寄存�?- `slcd_seg_batch_begin()` / `slcd_seg_batch_end()` 批量模式，整页刷新仅触发 **1 �?* `slcd_data_update_request()`
- `slcd_seg_clear_digits()` 仅清除数字段�?+ 小数�?+ 页面图标�?*保留电池/信号/阀�?蓝牙常驻图标不闪�?*
- `slcd_seg_flush()` 唯一写入硬件寄存器的通道，确保原子更�?
**LCD UI �?* (`lcd_ui.c/h`):
- 多页面管�? 进口压力/出口压力/进口温度/出口温度/电压/时间(�?月日/时分)
- 条件重绘: 缓存上次显示值，仅数据变化时触发页面刷新
- 公共图标统一管理: 电池格数/信号格数/阀门状�?蓝牙开�?- 按键翻页 + 自动回首页超�?
**LCD 显示开�?(`g_lcd_enabled`, 产品复用)**:
- 默认 1 (�?LCD 产品)；置 0 时为无屏产品，跳�?`slcd_seg_configuration()`/`lcd_ui_init()` 硬件初始化与所有刷新调用，进一步省电�?- `main.c` 中所�?LCD 调用 (init/refresh/update_signal/next_page/toggle_bluetooth) 均以 `if(g_lcd_enabled)` 守卫�?- 运行时可改：串口 `lcd_switch=0/1` �?MQTT 下发 `lcd_switch`，即时生效并�?EEPROM (EXT �?0x286)�?
### 3.8 LED 指示灯模�?(`led.c / led.h`)

- **引脚**: PA15
- **驱动方式**: 推挽输出，HIGH = 亮，LOW = �?- **功能**: `led_on()` / `led_off()` / `led_toggle()` / `led_blink(次数, 周期ms)`

### 3.9 蜂鸣器模�?(`buzzer.c / buzzer.h`)

- **引脚**: PB8
- **驱动方式**: 推挽输出，HIGH = 响，LOW = �?- **功能**: `buzzer_on()` / `buzzer_off()` / `buzzer_beep(次数, 鸣响ms, 间隔ms)`

### 3.10 按键模块 (`mygpio.c / mygpio.h`)

| 引脚 | 功能 | 说明 |
|------|------|------|
| PD0 | Btn1 | 外部上拉，按�?= LOW，EXTI0 下降沿唤醒深休眠 |
| PD1 | Btn2 | 外部上拉，预留按�?|

### 3.11 RTC 实时时钟模块 (`tim.c / tim.h`)

- **时钟�?*: LXTAL (外部 32.768kHz 晶振, PC14/PC15)
- **时间同步**: 4G 模块 AT+CCLK? �?`rtc_set_from_cclk()` 自动同步 (UTC+8)
- **闹钟唤醒**: 支持分钟/秒级定时唤醒深休�?(EXTI17)
- **BKP 寄存�?*: 用于标记 RTC 是否已配�?(Magic = 0x32F0)

### 3.12 定时器与延时模块 (`delay.c / delay.h / systick.c`)

| 定时�?| 用�?|
|--------|------|
| SysTick | 系统滴答 �?`trea_millis()` 毫秒计时 + `trea_delay_ms()` 阻塞延时 |
| TIMER1 | 微秒级延�?`delay_us()` |
| TIMER2 | 电机 PWM 输出 (PA6/PA7) |

### 3.13 OTA 空中升级模块 (`ota_app.c / ota_app.h / ota_info.h`)

**Flash 分区** (256KB):

| 区域 | 地址范围 | 大小 | 说明 |
|------|---------|------|------|
| Bootloader | 0x08000000 - 0x08003FFF | 16 KB | 引导加载程序 |
| OTA InfoBlock | 0x08004000 - 0x08004FFF | 4 KB | 升级状�?版本/固件大小 |
| App 运行�?| 0x08005000 - 0x08021FFF | 116 KB | 应用程序 (当前运行) |
| 下载缓存�?| 0x08022000 - 0x0803FFFF | 120 KB | OTA 固件下载暂存 |

**升级流程**: 收到 `upgrade/set` �?上报升级�?�?HTTP 下载 bin �?�?InfoBlock �?系统复位 �?Bootloader 搬运 �?App 启动上报成功

### 3.14 MQTT 下发处理模块 (`downlink.c / downlink.h`)

- 排空并处理缓存的 MQTT 下发消息 (`drain_pending_downlink()`)，逐条解析平台下发�?`commandStr` + `value`�?- 通过 `s_cmd_map[]` 命令映射表统一分发，部分命令带 JSON 字段提取 (如阈�?IP)�?
**支持的下发命�?* (主要):

| 命令 | 功能 |
|------|------|
| `valve_open` / `valve_close` | 远程开/关阀 |
| `get_pressure` / `get_pressure2` | 查询进气�?出气端压力状�?|
| `temp_get1` / `temp_get2` | 查询进气�?出气端温度状�?|
| `sensor_status` | 查询传感器在线状�?(rtu_have/status ×2) |
| `LY06_get` | 查询电压报警状�?|
| `collectspeed_set` / `collectcycle_get` | 设置/查询采集周期 |
| `report_set` / `report_get` | 设置/查询上报周期 |
| `window_set` / `window_get` | 设置/查询窗口模式 |
| `abnormal_set` / `abnormal_get` | 设置/查询异常关阀策略 |
| `vpthreshold_set1/get1` / `set2/get2` | 进气�?出气端压力阈�?|
| `Tthreshold_set/get` / `set2/get2` | 进气�?出气端温度阈�?|
| `spupower_set/get` | 电压检测配�?|
| `delay_set/get` | 阀门动作延�?|
| `real_set/get` | 实时采集开�?+ 频率 |
| `first_ip_set/get` / `scend_ip_set/get` | �?�?IP 配置 |
| `user_ip_set/get` / `MQTT_set/get` | 用户平台 IP / MQTT 配置 |
| `lcd_switch` | LCD 显示开�?(0/1) |
| `sensor_count_set` | 传感器数�?(1/2) |
| `restart` / `4Gpower_off` | 远程重启 / 4G 断电 |

**串口调试命令** (USART0，部�?:
`sensor` / `sensor_status` / `temp_status` / `pressure_status` (即采即查)、`vlp` / `buf` / `time` / `ip`、`4Gpower_on/off`、`mqtt_reconnect`、`valve open/close`、`first_ip:IP.port`、`lcd_switch=0/1`、`sensor_count=1/2`、`reset`(恢复出厂)�?
### 3.15 阈值检查模�?(`threshold.c / threshold.h`)

**检查项及优先级** (从高到低):
1. **VPROTECT** �?电压保护 (低于保护�?�?强制关阀，进�?VLP)
2. **PRESSURE_VERY** �?压力极高/极低 (`> ppmax+shock` / `< ppmin-shock`)
3. **PRESSURE** �?压力超上/下限 (`> pmax+shock` / `< pmin-shock`)
4. **TEMP_VERY** �?温度极高/极低
5. **TEMP** �?温度过高/过低
6. **VOLT** �?电压过高/过低
7. **MUTATION** �?压力突变 (`|Δp| > mutation`，不�?shock 影响)

**`ThresholdResult_t` 枚举** (`threshold_check` 返回):
`THRESHOLD_OK / OVER_LIMIT / UNDER_LIMIT / MUTATION / VERY_LOW / VERY_HIGH / VPROTECT / TEMP_HIGH / TEMP_LOW / TEMP_VERY_HIGH / TEMP_VERY_LOW / VOLT_HIGH / VOLT_LOW`

- 支持 **shock 滞回�?* 防止压力静态阈值边缘抖动误�?(仅作用于 pmax/pmin/ppmax/ppmin)�?- **LY06 电压报警** (`threshold_voltage_alarm`): 单一真源，返�?`-1`(未知/ADC无效/检测关�? / `0`(正常) / `1`(报警)，供 `ec800e.c` 上报�?`downlink.c` �?`LY06_get` 查询共用�?- **双通道阈�?*: 进气端用 `g_pthreshold`/`g_tthreshold`，出气端�?`g_pthreshold2`/`g_tthreshold2`，分别由 `threshold_get_pressure_status2()`/`threshold_get_temp_status2()` 提供状态�?- **异常自动关阀策略** (`AbnormalClose_t`): `none/high/low/mutation/highorlow/highormutation/lowormutation/all/temph/templ`。电压报警仅上报不动阀；电压保�?(VPROTECT) 独立强制关阀�?
### 3.16 传感器数据缓冲模�?(`sensor_buffer.c / sensor_buffer.h`)

- **RAM 环形缓冲�?*，深休眠�?SRAM 保持供电，数据不丢失
- 容量: 128 条记�?(满则覆盖最老记�?
- 单条记录 44 字节: 双路压力/温度 + 电池电压 + 时间�?+ 标志�?
---

## 四、低功耗设�?
### 4.1 深休眠模�?
- **模式**: PMU_DEEPSLEEP (低驱动、低功耗稳压器)
- **唤醒�?*:

| 唤醒�?| 触发方式 | 标志变量 |
|--------|---------|---------|
| LPUART0 | 起始位唤�?(EXTI28) | `counter0` |
| RTC 闹钟 | ALARM0 中断 (EXTI17) | `rtc_alarm_flag` |
| 按键 Btn1 | PD0 EXTI0 下降�?| `button_wakeup_flag` |

### 4.2 GPIO 低功耗配�?
休眠前调�?`gpio_enter_low_power()` 将未使用 GPIO 切为 **模拟输入 (Analog)**，彻底杜绝引脚倒灌和悬空漏电。保持以下引脚状�?

| 引脚 | 休眠保持 | 原因 |
|------|---------|------|
| PA0 (DTR) | OUTPUT HIGH | 允许 EC800E 进入休眠 |
| PA5 (M_SLP) | OUTPUT LOW | 电机驱动休眠, 防漏�?|
| PA15 (LED1) | OUTPUT LOW | LED 息灭 |
| PB8 (BUZZER) | OUTPUT LOW | 蜂鸣器关�?|
| PC12 (SW_BT) | OUTPUT LOW | 蓝牙断电 |
| PC14/PC15 | LXTAL | RTC 晶振，绝不修�?|
| PD0/PD1 | INPUT | 按键唤醒 (外部上拉) |
| PD4 (SW_IIC) | OUTPUT LOW | 传感器电子开关断�?|
| PD8/PD9 | AF (可�? | LPUART 启用时保�?|
| PB5 (V4G_EN) | OUTPUT LOW | 4G 模块断电 |

### 4.3 窗口模式

配置结构�?`WindowConfig_t`：`pattern`(模式)、`start/stop`(窗口 HHMMSS)、`end_result`、`dottime`(整点小时)�?
| 模式 | 4G 休眠行为 | 上报时机 | 说明 |
|------|------------|---------|------|
| `long` | 窗口内或 `end_result=1` �?4G 保持供电 (QSCLK 轻睡�?，可接收 MQTT 下发；`end_result=0` 且窗口外�?4G 断电 | 按采�?上报周期上报 | 长连接模�?�?支持 LPUART 唤醒接收平台下发 |
| `short` | 休眠�?4G **始终断电**，仅在上报周期到达且处于活动窗口 (`start`~`stop`) 内时才开机联网上�?| 窗口内按周期上报 | 短连接模�?�?窗口外仅采集不联网，节省功�?|
| `dot` | 休眠�?4G **始终断电**，不使用窗口时间段概�?| 每天�?`dottime` 指定的整�?(0-23) 上报一�?| 定点模式 �?最省电，一天仅联网一�?|

### 4.4 秒级实时监测模式

�?`g_real_enabled`(开�? + `g_real_frequency`(1~10s) 控制，开启后�?`do_sleep()` 内嵌一个秒级采集循环，实现高频异常监测而不必每次走完整唤醒流程�?
**工作流程**:
1. `do_sleep()` 配置 **RTC 秒级闹钟** (`rtc_set_next_alarm_sec`) 而非分钟闹钟，进入深休眠�?2. 唤醒后进�?`while(g_real_enabled ...)` 循环，先 `system_wakeup_reinit()` + �?RTC�?   - **到达正常采集网格** (整分�?±5s 且本分钟未采�? �?退出循环，走完�?`STATE_WAKEUP`�?   - **非采集网�?* �?调用 `do_realtime_check()` 最小化采集后立即回睡�?3. `do_realtime_check()` 仅初始化 SysTick + I2C + PD4，采集传感器并判异常�?   - **正常** �?检查报警恢�?�?更新基准压力 �?回睡�?   - **异常** �?防重�?(同类异常不重复上�? �?立即重采确认 3 �?�?确认后启 4G 上报报警 + 按策略关阀 �?回睡�?4. `last_grid_collect_min` 记录本分钟已采集，防止秒级循环在同一分钟内重复触发正常采集�?
> 实时模式�?4G 是否使能 LPUART 唤醒由窗口模式决�?(�?`long` + 在线时启�?，异常上报采用「快速唤�?(在线) / 冷启�?(离线)」两条路径，上报完按原状态恢�?PSM 或断电�?
### 4.5 按键功能 (PD0 Btn1)

深休眠下 PD0 下降�?(EXTI0) 唤醒，`do_wakeup()` 区分短按连击与长按：

| 操作 | 功能 |
|------|------|
| 单击 (1 短按) | LCD 翻页 (`lcd_ui_next_page`) |
| 双击 (2 短按) | 手动数据上报 + 配置打印 (`do_button_short_press`) |
| 三击 (3 短按) | 蓝牙开�?(`lcd_ui_toggle_bluetooth`, PC12 SW_BT) |
| 长按 4s | 4G 开机联�?�?等待 20s 接收下发 �?按窗口模式收�?|

> 短按连击�?500ms 窗口内计数；长按需持续 4s 确认。按键唤醒走「快速检测路径」，跳过 `sys_init()` �?1s 延时，需完整外设时由 `ensure_full_init()` 按需升级�?
---

## 五、外设资源占用汇�?
| 外设 | 用�?|
|------|------|
| USART0 | 调试串口 (PC4/PC5) |
| LPUART0 | 4G 通信串口 (PD8/PD9) + DMA_CH1 接收 |
| TIMER2 | 电机 PWM (PA6/PA7) |
| TIMER1 | 微秒级延�?|
| SysTick | 系统滴答计时 |
| ADC (CH12) | 电池电压检�?(PC2) |
| RTC | 实时时钟 + 闹钟唤醒 |
| SLCD | 段码液晶显示 (19 引脚) |
| EXTI0 | 按键唤醒 (PD0) |
| EXTI17 | RTC 闹钟唤醒 |
| EXTI28 | LPUART 深休眠唤�?|
| DMA_CH1 | LPUART0 RX 循环接收 |
| I2C (软件 #1) | PC0/PC1 �?PT304D 传感�? + M24M01 EEPROM (共用) |
| I2C (软件 #2) | PB6/PB7 �?PT304D 传感�? (独立) |
| Flash | OTA 分区 (Bootloader 16K + Info 4K + App 116K + Cache 120K) |
| BKP 寄存�?| RTC 配置标记 |

---

## 六、引脚使用一览图

```
GPIOA:  PA0[DTR]  PA5[M_SLP]  PA6[PWM0]  PA7[PWM1]  PA8[COM0]  PA9[COM1]  PA10[COM2]
        PA11[K1-S]  PA12[K2-S]  PA15[LED1]

GPIOB:  PB0[SEG5]  PB1[SEG6]  PB6[I2C_SCL2]  PB7[I2C_SDA2]  PB8[BUZZER]
        PB9[COM3]  PB10[SEG10]  PB11[SEG11]  PB12[SEG12]  PB13[SEG13]  PB14[SEG14]  PB15[SEG15]

GPIOC:  PC0[I2C_SCL1]  PC1[I2C_SDA1]  PC2[AD_BAT]  PC4[DBG_TX]  PC5[DBG_RX]
        PC6[SEG24]  PC7[SEG25]  PC8[SEG26]  PC9[SEG27]  PC10[SEG28]  PC11[SEG29]
        PC12[SW_BT]  PC14[LXTAL_IN]  PC15[LXTAL_OUT]

GPIOD:  PD0[Btn1]  PD1[Btn2]  PD4[SW_IIC]  PD6[SLCD_VLCD]  PD8[LP_TX]  PD9[LP_RX]

GPIOB:  ... PB5[V4G_EN] ...
```

---

## 七、软件模块文件清�?
| 文件 | 功能 |
|------|------|
| `main.c` | 主程序，状态机驱动 |
| `mygpio.c/h` | GPIO 初始�?+ 低功耗配�?+ DTR/按键控制 |
| `usart.c/h` | USART0/LPUART0 初始�?+ DMA 接收 + 深休眠唤�?|
| `ec800e.c/h` | EC800E 4G 模块驱动 (AT 指令 + MQTT + HTTP) |
| `PT_304_35.c/h` | PT304D 双传感器采集驱动 |
| `i2c_sensor.c/h` | 软件 I2C #1 (PC0/PC1) |
| `i2c_eeprom.c/h` | 软件 I2C #2 (PB6/PB7) |
| `M24M01.c/h` | M24M01 EEPROM 读写驱动 |你进行修�?| `eeprom_config.c/h` | EEPROM 配置数据管理 (10 个存储区�? + 串口命令解析 |
| `motor.c/h` | TIMER2 PWM 电机驱动 |
| `valve.c/h` | 阀门动作控�?(含限位检�?+ 故障判断) |
| `adc_bat.c/h` | ADC 电池电压采集 |
| `slcd_seg.c/h` | 段码液晶底层驱动 (XD-H2074A) + 影子缓冲区防闪屏 |
| `lcd_ui.c/h` | LCD UI �?�?多页面管�?+ 条件重绘 + 公共图标 |
| `json_builder.c/h` | 轻量 JSON 构建工具 |
| `led.c/h` | LED 指示灯控�?|
| `buzzer.c/h` | 蜂鸣器控�?|
| `tim.c/h` | RTC 初始�?+ 闹钟唤醒 + 时间同步 |
| `delay.c/h` | TIMER1 延时 + SysTick 毫秒计时 |
| `systick.c/h` | SysTick 配置 |
| `threshold.c/h` | 压力/温度/电压阈值检�?|
| `sensor_buffer.c/h` | RAM 环形缓冲�?(128 条记�? |
| `downlink.c/h` | MQTT 下发消息处理 |
| `ota_app.c/h` | App �?OTA 升级逻辑 |
| `ota_info.h` | OTA Flash 分区 + 状态码定义 (BL/App 共享) |
| `app_config.h` | 编译时默认参�?+ 枚举/结构体定�?(�?LCD/传感器数量默认�? |
| `main.h` | 传感器上下文结构�?+ 固件版本�?(APP_FW_VERSION_NUM=2) |
| `gd32l23x_it.c` | 中断服务函数汇�?|

---

## 八、运行时可配置参数汇�?
以下参数均可通过 **调试串口 (USART0)** �?**MQTT 下发** 修改，并持久化到 EEPROM，重�?OTA 后仍生效�?
| 参数 | 全局变量 | 默认 | 范围/取�?| 说明 |
|------|---------|------|---------|------|
| 采集周期 | `g_collect_cycle` | 10 min | once + sec/min/hour/day | 采集闹钟间隔 |
| 上报周期 | `g_report_cycle` | 1 h | once + sec/min/hour/day/week | 上报间隔 |
| 窗口模式 | `g_window` | long | long/short/dot | 4G 供电调度策略 |
| 异常关阀策略 | `g_abnormal_close` | highorlow | 10 种策�?| �?`AbnormalClose_t` |
| 实时采集开�?| `g_real_enabled` | 1 | 0/1 | 秒级监测开�?|
| 实时采集频率 | `g_real_frequency` | 5 | 1~10 s | 秒级采集间隔 |
| LCD 显示开�?| `g_lcd_enabled` | 1 | 0/1 | �?无屏产品复用 (`lcd_switch`) |
| 传感器数�?| `g_sensor_count` | 2 | 1/2 | 单路/双路产品复用 (`sensor_count_set`) |
| 进气端压力阈�?| `g_pthreshold` | switch=0 | �?| 上下�?极限/突变/滞回 |
| 出气端压力阈�?| `g_pthreshold2` | switch=0 | �?| 同上 (第二�? |
| 进气端温度阈�?| `g_tthreshold` | switch=0 | �?| �?�?极高/极低 |
| 出气端温度阈�?| `g_tthreshold2` | switch=0 | �?| 同上 (第二�? |
| 电压检�?| `g_spupower` | switch=0 | �?| 高压/低压/保护�?VLP) |
| 阀门动作延�?| `g_vdelay` | switch=0 | �?| 到位后保持时�?|
| �?�?IP | `g_mqtt.server/server2` | �?app_config | �?| 主平台服务器 |
| 用户平台 MQTT | `g_user_mqtt` | 未配�?| �?| 可选第二平�?|
| 阀门最后状�?| `g_valve_last_order` | 0 | 0/1 | �?开，持久化 |

> 上电时若 EEPROM 无有�?MQTT 配置，`config_enter_setup_mode()` 阻塞等待 30s 接受任意顺序的配置命令；超时后用默认值启动�?