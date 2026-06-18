# GD32L233RCT6 引脚分配总表

芯片型号：**GD32L233RCT6**（LQFP-64）  
项目：Household Valves LPUART OTA

---

## 功能模块索引

| 模块 | 引脚 |
|------|------|
| 4G 模组 (EC801E) | PF1, PA0, PA1, PD8, PD9 |
| 传感器 I2C 总线（共享） | PC0, PC1, PB5 |
| 传感器 I2C 总线（独立） | PB6, PB7 |
| 电池 ADC | PC2 |
| 电机驱动 | PA5, PA6, PA7 |
| 串口1 RS485 | PA2, PA3 |
| 蓝牙 (BT) | PB3, PB4, PD2, PC12 |
| 调试串口 | PC4, PC5 |
| 按键唤醒 | PA11, PA12 |
| LED / 蜂鸣器 | PD0, PD5 |
| SW 电源控制 | PD4 |
| LCD（段码）| PA8~PA10, PB0~PB2, PB10~PB15, PC6~PC12, PC10, PC11 |
| RTC 晶振 | PC14, PC15 |
| SWD 调试 | PA13, PA14 |

---

## 详细引脚表

### 1. 4G 模组（EC801E / EC800E）

| GPIO | 方向 | 功能 | 说明 |
|------|------|------|------|
| **PF1** | OUT | V4G_EN | 4G 电源控制（HIGH=上电，LOW=断电）|
| **PA0** | OUT | 4G_DTR | 控制 4G 模块休眠（HIGH=允许睡眠，LOW=强制唤醒）|
| **PA1** | IN  | 4G_RI  | 4G 模块振铃指示输入 |
| **PD8** | OUT (AF) | LPUART0_TX | 向 4G 模块发送 AT 指令（LP_TXD）|
| **PD9** | IN  (AF) | LPUART0_RX | 接收 4G 模块响应，支持深睡唤醒（LP_RXD）|

> **深睡策略**：PF1 保持 OUTPUT LOW（4G 断电）；PD8/PD9 在 `lpuart_enabled=1` 时保持 AF，否则设为模拟。

---

### 2. 压力传感器 PT304D

#### 传感器1（35 kPa）+ AT24C16 EEPROM — 共用 I2C 总线

| GPIO | 方向 | 功能 | 说明 |
|------|------|------|------|
| **PC0** | OUT (OD) | I2C2_SCL | 共享 SW I2C 时钟线，连接传感器1 + EEPROM |
| **PC1** | I/O (OD) | I2C2_SDA | 共享 SW I2C 数据线 |
| **PB5** | OUT | SW_IIC    | I2C 总线电源开关（HIGH=上电，LOW=断电，深睡保持 LOW）|

#### 传感器2（600 kPa）— 独立 I2C 总线

| GPIO | 方向 | 功能 | 说明 |
|------|------|------|------|
| **PB6** | OUT (OD) | I2C1_SCL | 独立 SW I2C 时钟线（SCL1，22Ω 串联）|
| **PB7** | I/O (OD) | I2C1_SDA | 独立 SW I2C 数据线（SDA1，22Ω 串联）|

> 两路 I2C 总线均为软件模拟 I2C，通过 `PT304D_CollectAll` 统一采集。

---

### 3. 电池电压 ADC

| GPIO | 功能 | ADC 通道 | 说明 |
|------|------|----------|------|
| **PC2** | AD_BAT | ADC_CHANNEL_12 | 分压采样（R3=2MΩ, R5=2MΩ，分压比=2）|

> 转换公式：`V_BAT = (raw / 4095.0) × 3.3V × 2`

---

### 4. 电机驱动

| GPIO | 方向 | 功能 | 说明 |
|------|------|------|------|
| **PA5** | OUT | M_SLP | 电机驱动器使能/睡眠（HIGH=使能，LOW=睡眠）|
| **PA6** | OUT (PWM) | M_IN1 | 电机方向 / PWM 输入 1 |
| **PA7** | OUT (PWM) | M_IN2 | 电机方向 / PWM 输入 2 |

---

### 5. 串口通信

| GPIO | 方向 | 外设 | 网络名 | 说明 |
|------|------|------|--------|------|
| **PA2** | OUT (AF) | USART1_TX | S1_RX | RS485 / Serial1 发送（33Ω 串联）|
| **PA3** | IN  (AF) | USART1_RX | S1_TX | RS485 / Serial1 接收（33Ω 串联）|
| **PC4** | OUT (AF) | USART0_TX | Debug_RX | 调试串口发送（33Ω 串联）|
| **PC5** | IN  (AF) | USART0_RX | Debug_TX | 调试串口接收（33Ω 串联）|
| **PB3** | OUT (AF) | UART4_TX | BT_TXD | 蓝牙模块发送（33Ω 串联）|
| **PB4** | IN  (AF) | UART4_RX | BT_RXD | 蓝牙模块接收（33Ω 串联）|

---

### 6. 蓝牙模块

| GPIO | 方向 | 功能 | 说明 |
|------|------|------|------|
| **PB3** | OUT (AF) | UART4_TX / BT_TXD | 见上表 |
| **PB4** | IN  (AF) | UART4_RX / BT_RXD | 见上表 |
| **PD2** | I/O | BT_IO  | 蓝牙状态/控制 IO |
| **PC12**| OUT | SW_BT  | 蓝牙模块电源开关 |

---

### 7. 按键 & 指示

| GPIO | 方向 | 功能 | 说明 |
|------|------|------|------|
| **PA11** | IN (上拉) | Btn1 | 按键 1 输入 |
| **PA12** | IN (上拉) | Btn2 | 按键 2，EXTI12 下降沿触发深睡唤醒 |
| **PD0**  | OUT | LED1 | 指示灯 |
| **PD5**  | OUT | BUZZER | 蜂鸣器 |
| **PD4**  | OUT | SW_Uart1 | UART1 RS485 发送使能 |

---

### 8. LCD（段码驱动）

| 类型 | GPIO |
|------|------|
| COM（公共极）| PA8 (COM0), PA9 (COM1), PA10 (COM2), PB9 (COM3) |
| SEG（段）    | PB0(L5), PB1(L6), PB2(L7), PB10(L7), PB11(L8) |
|              | PB12(L9), PB13(L10), PB14(L11), PB15(L12) |
|              | PC6(L13), PC7(L14), PC8(L15), PC9(L16) |
|              | PC10(L17), PC11(L18) |

---

### 9. 系统 / 时钟 / 调试

| GPIO | 功能 | 说明 |
|------|------|------|
| **PC14** | OSC32IN  | 32.768 kHz LXTAL 晶振输入（RTC，**禁止改动**）|
| **PC15** | OSC32OUT | 32.768 kHz LXTAL 晶振输出（**禁止改动**）|
| **PF0**  | OSCIN    | 主晶振输入 |
| **PF1**  | OSCOUT / V4G_EN | 复用为 4G 电源控制输出 |
| **PA13** | SWDIO    | SWD 调试数据 |
| **PA14** | SWCLK    | SWD 调试时钟 |

---

## 深睡引脚状态一览

深睡前调用 `gpio_enter_low_power(lpuart_enabled)` 后各引脚状态：

| GPIO | 深睡状态 | 原因 |
|------|----------|------|
| PA0  | 保持 OUTPUT HIGH | 4G DTR，维持 EC801E 休眠 |
| PB5  | 保持 OUTPUT LOW  | SW_IIC，I2C 总线断电 |
| PF1  | 保持 OUTPUT LOW  | V4G_EN，4G 模块断电 |
| PC14/PC15 | 保持 AF（LXTAL）| RTC 继续走时 |
| PA12 | 保持 INPUT 上拉 + EXTI12 | 按键唤醒 |
| PD8/PD9 | `lpuart_enabled=1` 保持 AF；`=0` 设为模拟 | LPUART0 唤醒 |
| 其余引脚 | 模拟输入（无上下拉）| 最低漏电 |

---

## 修订记录

| 日期 | 修改内容 |
|------|----------|
| 2026-06-17 | PF1 确认为 V4G_EN（原误为 PD9）；PC2 确认为 AD_BAT（原误为 PA5）；PD8=LPUART_TX, PD9=LPUART_RX 逻辑修正；PA0 确认为 4G_DTR（原注释误标 PA11）|
