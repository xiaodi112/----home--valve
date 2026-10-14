#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>
/* ==================== EEPROM chip select (compile-time, exactly one) ====================
 *   EEPROM_DRV_M24M01  : M24M01 (128KB, default)
 *   EEPROM_DRV_AT24C64 : AT24C64 (8KB)
 */
#define EEPROM_DRV_M24M01
//#define EEPROM_DRV_AT24C64

#if defined(EEPROM_DRV_M24M01) && defined(EEPROM_DRV_AT24C64)
  #error "EEPROM: pick exactly one chip (M24M01 / AT24C64)"
#elif !defined(EEPROM_DRV_M24M01) && !defined(EEPROM_DRV_AT24C64)
  #error "EEPROM: no chip selected in app_config.h"
#endif

/*
 * ============================================================
 *  编译时默认配置 — 当EEPROM无有效数据时使用
 *  运行时实际使用 g_interval / g_mqtt 全局变量（从EEPROM加载）
 *  用户可通过Debug串口在线修改并保存到EEPROM
 * ============================================================
 */

/* ---------- 固件版本 ---------- */
#define APP_FIRMWARE_VERSION          "1"

/* ---------- 间隔默认值（EEPROM无数据时使用） ----------
 * 注意: 必须与下面 APP_DEFAULT_COLLECT/REPORT_ONCE+TYPE 数学等价!
 *   COLLECT_HOURS = ONCE * (sec_per_unit) / 3600
 * 当前: 10*60/3600 = 0.1667h (10min) / 1*3600/3600 = 1h
 * EXT 区有效时, g_interval 由 cycle 推导, 两者必须保持同步。
 */
#define APP_DEFAULT_COLLECT_HOURS     (20.0f / 60.0f)   /* 20 min */
#define APP_DEFAULT_UPLOAD_HOURS      (120.0f / 60.0f)   /* 2 h */

/* ---------- MQTT默认值（EEPROM无数据时使用） ---------- */
#define APP_DEFAULT_MQTT_CLIENT_ID    "S&D54GRG29T5HB0&149&1"
#define APP_DEFAULT_MQTT_SERVER       "120.78.220.171"  /* 主IP */
#define APP_DEFAULT_MQTT_PORT         1883              /* 主端口 */
#define APP_DEFAULT_MQTT_SERVER2      "120.78.220.171"  /* 副IP (主IP故障切换), 留空字符串"" 表示禁用副IP */
#define APP_DEFAULT_MQTT_PORT2        1883              /* 副端口 */
#define APP_DEFAULT_MQTT_USER         "FastBee"
#define APP_DEFAULT_MQTT_PASS         "P41TQ4X1Y0C57IW0"
#define APP_DEFAULT_MQTT_TOPIC        "/149/D54GRG29T5HB0/property/post"

/* ---------- 固定参数（不通过串口修改） ---------- */
#define APP_EC_MAX_RETRY              1
#define APP_AT_TIMEOUT_SHORT          3000		/*返回状态*/
#define APP_AT_TIMEOUT_PUBEX          3000      /* QMTPUBEX等待'>'提示(QSCLK=2唤醒后模块需2-3s) */
#define APP_AT_TIMEOUT_LONG           10000
#define APP_AT_TIMEOUT_MQTT           15000
#define APP_MQTT_KEEPALIVE            300    /* MQTT心跳(秒), 模块自动发PINGREQ */

/* 一直在线模式(long+活动窗口或end_result=1)下, 4G与平台连接主动巡检间隔(分钟) */
#define FOURG_SUPERVISE_INTERVAL_MIN  30

/* ---------- 窗口配置默认值 ---------- */
#define APP_DEFAULT_WINDOW_PATTERN    "long"    /* long=长连接 short=短连接 dot=定点 */
#define APP_DEFAULT_WINDOW_END_RESULT 1         /* 0=窗口结束4G断电 1=保持在线 (long模式) */
#define APP_DEFAULT_WINDOW_DOTTIME    0         /* dot模式整点小时(0-23) */
#define APP_DEFAULT_WINDOW_START      "000000" /* short模式: 活动窗口开始 HHMMSS */
#define APP_DEFAULT_WINDOW_STOP       "000000" /* short模式: 活动窗口结束 HHMMSS */

/* ---------- 采集/上报周期默认值 (与 APP_DEFAULT_COLLECT/UPLOAD_HOURS 等价, 单位粒度更细) ---------- */
#define APP_DEFAULT_COLLECT_ONCE      20        /* 采集周期数值 */
#define APP_DEFAULT_COLLECT_TYPE      CYCLE_MINUTE /* 采集周期单位 */
#define APP_DEFAULT_REPORT_ONCE       2         /* 上报周期数值 */
#define APP_DEFAULT_REPORT_TYPE       CYCLE_HOUR /* 上报周期单位 */

/* ---------- 阀门最后状态默认值 ---------- */
#define APP_DEFAULT_VALVE_LAST_ORDER  0         /* 0=关, 1=开 (上电时若 EEPROM 无数据则使用) */



/*
	BUTTON_ENABLED 已移至 BSP/mygpio.h (板级按键配置):
	BUTTON_ENABLED=1 + LCD_LOCK_ENABLED=1 — 有按键 + LCD 受控（当前产品）
	BUTTON_ENABLED=1 + LCD_LOCK_ENABLED=0 — 有按键 + LCD 长亮轮显
	BUTTON_ENABLED=0 + LCD_LOCK_ENABLED=0 — 无按键 + LCD 长亮轮显
	BUTTON_ENABLED=0 + LCD_LOCK_ENABLED=1 — 无按键 + LCD 受控（不合理组合）
*/
/* ---------- LCD 显示总开关 (产品选配, 编译期) ----------
 * 1 = 带段码 LCD 产品: 编译 BSP/slcd_seg.c + App/lcd_ui.c 并启用全部显示逻辑
 * 0 = 无 LCD 产品: SLCD 驱动/UI/超时/校零短亮 整块不参与编译, 相关调用变空操作
 * 注: 无 LCD 板上 PA10/PB0/PB11/PB12/PB13/PB14 已改作 LED3/电机电流/ADC基准/空闲/4G,
 *     与 SLCD 引脚复用冲突, 因此本板必须为 0。 */
#define LCD_ENABLED                   0

/* ---------- LCD显示模式编译开关 (仅 LCD_ENABLED=1 时有效) ---------- */
#define LCD_LOCK_ENABLED              0         /* 1=LCD按键锁定(60s超时关屏/20s回默认页); 0=LCD常亮+采集周期自动翻页 */

#define LCD_SINGLE_SENSOR             1         /* 1=6页界面显示（6,一个传感器); 0=8页界面显示(8,2个传感器) */

/* ---------- 激光甲烷传感器硬件选配 (编译开关) ----------
 * 1=硬件带激光甲烷传感器: 编译 CH4 采集驱动/状态机/gas_get/laser_get/laser_set
 * 0=无此硬件: 相关代码不编译, 平台下发 laser/gas 命令时直接忽略(仅打印调试日志)
 */
#define LASER_CH4_ENABLED             0

/* ---------- 电机(阀门执行器)硬件选配 (编译开关) ----------
 * 1=硬件带电机: 编译电机驱动/阀门动作(删除运行时门控, 直接生效)
 * 0=无电机: 电机代码整块剔除, 节省 RAM/Flash, 阀门指令返回 valve-err
 */
#define MOTOR_ENABLED                 1

/* ---------- 蓝牙(含 BLE OTA)功能开关 ----------
 * 1 = 带蓝牙: 编译 BLE OTA 状态机 / UART4 / BLE 电源控制
 * 0 = 本产品不使用蓝牙: 上述全部停用; PB3/PB4(UART4)、PC12(SW_BT)、PD2(连接检测)
 *     在深睡时转为模拟以省电 (引脚可另行复用) */
#define BLE_ENABLED                   0

/* ---------- 零点(大气基准)标定参数 ----------
 * 新传感器为绝压式, 空载读数≈当地大气压; 以"大气压"为零点基准,
 * 后续测量值 = 传感器读数 - 基准 (表压)。
 * 安全约束: 上电只在 EEPROM 无有效基准时自动采集一次; 其余情况只读不改;
 *           用户可用按键手势显式重校 (见 ZERO_CAL_HOLD_MS / CONFIRM_*) */
#define ZERO_CAL_SENSOR_IDX           1        /* 传感器索引: 1 = PT304D_SENSOR_2 (中压/进气端, 0x76) */
#define ZERO_CAL_SAMPLE_CNT           5        /* 采样次数 */
#define ZERO_CAL_SAMPLE_MS            500      /* 采样间隔(ms) */
#define ZERO_CAL_WINDOW_MIN_PA        60000    /* 大气窗口下限(Pa) */
#define ZERO_CAL_WINDOW_MAX_PA        120000   /* 大气窗口上限(Pa) */
#define ZERO_CAL_DEFAULT_PA           98555    /* 默认基准(Pa): EEPROM 无有效基准时使用 */
#define ZERO_CAL_HOLD_MS              8000     /* 校准手势: 长按判定(ms) */
#define ZERO_CAL_CONFIRM_MS           3000     /* 校准手势: 确认窗口(ms), 从满 8 秒起算 */
#define ZERO_CAL_CONFIRM_PRESSES      2        /* 校准手势: 确认所需短按次数 */
#define LED_FAIL_HOLD_MS              5000     /* 校准失败: 三灯全亮提示时长(ms) */
#define LED_NOTICE_MS                 5000     /* 按键有效: 三灯全亮提示时长(ms) */
#define LED_BOOT_NOTICE_MS            1000     /* 开机提示: 三灯全亮时长(ms) */

/* ---------- 未安装校零前置模式 (编译期软件开关) ----------
 * 1=启用: 无已安装标志时开机先进入校零前置模式
 *        (仅5分钟采样校零+安装判定 与 每天10:00一次占位实时上报)
 * 0=禁用: 恢复旧行为(不校零, 直接进入正常状态机) */
#define CALIB_MODE_ENABLED            0

/* ---------- 产品复用配置默认值 ----------
 * 注意: 必须定义在下面 IIC2 引脚映射之前 —— IIC2 是否存在由
 *       APP_DEFAULT_SENSOR_COUNT 决定, 而 #if 只会看到此前已定义的宏。 */
#define APP_DEFAULT_LCD_ENABLED       0         /* 0=无LCD产品(本板), 1=有LCD产品 */
#define APP_DEFAULT_SENSOR_COUNT      1         /* 1=单路传感器(进气端), 2=双路传感器(本板双路) */

/* ---------- 三组独立 I2C 总线引脚映射 ---------- */
#define IIC0_SCL_PORT  GPIOB    /* IIC0: EEPROM      SCL */
#define IIC0_SCL_PIN   GPIO_PIN_6
#define IIC0_SDA_PORT  GPIOB    /* IIC0: EEPROM    SDA */
#define IIC0_SDA_PIN   GPIO_PIN_7
#define IIC1_SCL_PORT  GPIOD    /* IIC1: 进气端(中压)传感器 SCL (S2) */
#define IIC1_SCL_PIN   GPIO_PIN_4
#define IIC1_SDA_PORT  GPIOD    /* IIC1: 进气端(中压)传感器 SDA (S2) */
#define IIC1_SDA_PIN   GPIO_PIN_5

#if APP_DEFAULT_SENSOR_COUNT == 2
#define IIC2_SCL_PORT  GPIOC    /* IIC2: 出气端(低压)传感器 SCL (S1) */
#define IIC2_SCL_PIN   GPIO_PIN_0
#define IIC2_SDA_PORT  GPIOC    /* IIC2: 出气端(低压)传感器 SDA (S1) */
#define IIC2_SDA_PIN   GPIO_PIN_1
#endif /*  */

/* ---------- 软件 I2C 总线速率 (半周期延时计数, 越小越快) ---------- */
#define SOFT_I2C_HALF_PERIOD       150

/* 注: 传感器 IIC 电源由 PC3 开关控制 (BSP/power.c: PWR_SENSOR_PORT/PIN);
 *     电压采集基准/使能脚已挪到 PB11 (BSP/adc_bat.c) */
/* 电机引脚 (MOTOR_SLEEP_PORT/PIN, MOTOR_DRV_MODE) 已移至 BSP/motor.h */

#define APP_DEFAULT_REAL_ENABLED      1         /* 0=禁用, 1=启用秒级采集 */
#define APP_DEFAULT_REAL_FREQUENCY    5         /* 秒数 (1-10) */

/* ---------- 阈值默认值 ---------- */
/* 注: 阈值结构已拆分为 4 组独立配置, 默认值定义在 eeprom_config.h:
 *   DEF_VP_*  压力阈值 (vpthreshold, magic 0xC0)
 *   DEF_TT_*  温度阈值 (Tthreshold,  magic 0xC1)
 *   DEF_SP_*  电压检测 (spupower,    magic 0xC2)
 *   DEF_VD_*  阀门延时 (valve_delay, magic 0xC3)
 * 全部默认 0 + switch=0 禁用, 用户后续配置后才生效。
 */

/* ---------- 异常自动关阀策略默认值 ---------- */
#define APP_DEFAULT_ABNORMAL_CLOSE    "none"  /* 默认: 关闭 */

/* ---------- 报警确认参数 ---------- */
#define APP_ALARM_RECHECK_COUNT       3         /* 异常重采次数 */
#define APP_ALARM_RECHECK_DELAY_MS    500       /* 重采间隔 (ms) */

/* ---------- 采集/上报周期类型枚举 ---------- */
typedef enum {
    CYCLE_SEC = 0,
    CYCLE_MINUTE,
    CYCLE_HOUR,
    CYCLE_DAY,
    CYCLE_WEEK        /* 仅上报周期使用 */
} CycleType_t;

/* ---------- 窗口配置结构体 ---------- */
typedef struct {
    char     pattern[8];       /* "long"/"short"/"dot" */
    uint8_t  end_result;       /* long模式: 0=窗口结束4G断电, 1=窗口外4G保持可收下发 */
    char     start[8];         /* HHMMSS格式, 如"132900" (short模式) */
    char     stop[8];          /* HHMMSS格式, 如"140700" (short模式) */
    uint8_t  dottime_mask[3];  /* dot pattern: 24-hour bitmap, bit0=0h..bit23=23h, e.g. "08,12,14,18,20" */
} WindowConfig_t;

/* ---------- 采集/上报周期配置结构体 ---------- */
typedef struct {
    uint16_t   once;           /* 数值, 如22 */
    CycleType_t type;          /* 单位: sec/minute/hour/day/week */
} CycleConfig_t;

/* ---------- 异常自动关阀策略枚举 ---------- */
/* ---------- 异常自动关阀策略枚举 (平台协议 pv* 极值档) ----------
 * 进气端 abnormal_close / 出气端 abnormal2_close 共用:
 *   pvhigh=压力超高(VERY_HIGH), pvlow=压力超低(VERY_LOW), pvmutation=压力骤变,
 *   pvhighorlow / pvhighormutation / pvlowormutation / all(三档全含) / none(关闭)
 * 注: 普通档(pmax/pmin)与温度不再参与关阀; 激光由 abnormal3_close(0/1) 独立控制;
 *     电压报警(含VPROTECT)只上报, 不关阀。 */
typedef enum {
    ABNORMAL_NONE = 0,
    ABNORMAL_PV_HIGH,             /* 压力超高关阀 (pvhigh)    = VERY_HIGH */
    ABNORMAL_PV_LOW,              /* 压力超低关阀 (pvlow)     = VERY_LOW */
    ABNORMAL_PV_MUTATION,         /* 压力骤变关阀 (pvmutation)= MUTATION */
    ABNORMAL_PV_HIGH_OR_LOW,      /* 超高或超低 (pvhighorlow) */
    ABNORMAL_PV_HIGH_OR_MUTATION, /* 超高或骤变 (pvhighormutation) */
    ABNORMAL_PV_LOW_OR_MUTATION,  /* 超低或骤变 (pvlowormutation) */
    ABNORMAL_PV_ALL               /* 超高/超低/骤变 (all) */
} AbnormalClose_t;

/* ---------- 温度状态枚举(上报用) ---------- */
typedef enum {
    TEMP_NORMAL = 0,
    TEMP_HIGH,                 /* > Tthreshold.high  */
    TEMP_LOW,                  /* < Tthreshold.low   */
    TEMP_VERY_HIGH,            /* > Tthreshold.vhigh */
    TEMP_VERY_LOW              /* < Tthreshold.vlow  */
} TempStatus_t;

/* ---------- 压力状态枚举(上报用) ---------- */
typedef enum {
    PRESSURE_NORMAL = 0,
    PRESSURE_LOW,              /* < pmin */
    PRESSURE_HIGH,             /* > pmax */
    PRESSURE_VERYLOW,          /* < ppmin */
    PRESSURE_VERYHIGH,         /* > ppmax */
    PRESSURE_MUTATION          /* 突变 */
} PressureStatus_t;

/* ---------- 阀门控制结果枚举 ---------- */
typedef enum {
    VALVE_ON = 0,              /* 开到位 */
    VALVE_OFF,                 /* 关到位 */
    VALVE_TIMEOUT,             /* 动作超时 */
    VALVE_ERR                  /* 阀门故障 */
} ValveResult_t;

#endif /* APP_CONFIG_H */
