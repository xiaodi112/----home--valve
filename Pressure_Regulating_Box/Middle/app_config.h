#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>
/* ==================== EEPROM chip select (compile-time, exactly one) ====================
 *   EEPROM_DRV_M24M01  : M24M01 (128KB, default)
 *   EEPROM_DRV_AT24C64 : AT24C64 (8KB)
 */
//#define EEPROM_DRV_M24M01
#define EEPROM_DRV_AT24C64

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
#define APP_DEFAULT_COLLECT_HOURS     (20.0f / 60.0f)   /* 10 min */
#define APP_DEFAULT_UPLOAD_HOURS      (120.0f / 60.0f)   /* 1 h */

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
#define APP_DEFAULT_WINDOW_START      "080000" /* short模式: 活动窗口开始 HHMMSS */
#define APP_DEFAULT_WINDOW_STOP       "180000" /* short模式: 活动窗口结束 HHMMSS */

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
/* ---------- LCD显示模式编译开关 ---------- */
#define LCD_LOCK_ENABLED              0         /* 1=LCD按键锁定(60s超时关屏/20s回默认页); 0=LCD常亮+采集周期自动翻页 */

#define LCD_SINGLE_SENSOR             0         /* 1=6页界面显示（6,一个传感器); 0=8页界面显示(8,2个传感器) */

/* ---------- 激光甲烷传感器硬件选配 (编译开关) ----------
 * 1=硬件带激光甲烷传感器: 编译 CH4 采集驱动/状态机/gas_get/laser_get/laser_set
 * 0=无此硬件: 相关代码不编译, 平台下发 laser/gas 命令时直接忽略(仅打印调试日志)
 */
#define LASER_CH4_ENABLED             1

/* ---------- 电机(阀门执行器)硬件选配 (编译开关) ----------
 * 1=硬件带电机: 编译电机驱动/阀门动作(删除运行时门控, 直接生效)
 * 0=无电机: 电机代码整块剔除, 节省 RAM/Flash, 阀门指令返回 valve_err
 */
#define MOTOR_ENABLED                 1

/* ---------- 三组独立 I2C 总线引脚映射 ---------- */
#define IIC0_SCL_PORT  GPIOB    /* IIC0: EEPROM    SCL */
#define IIC0_SCL_PIN   GPIO_PIN_6
#define IIC0_SDA_PORT  GPIOB    /* IIC0: EEPROM    SDA */
#define IIC0_SDA_PIN   GPIO_PIN_7
#define IIC1_SCL_PORT  GPIOD    /* IIC1: 中压传感器 SCL S2*/
#define IIC1_SCL_PIN   GPIO_PIN_4
#define IIC1_SDA_PORT  GPIOD    /* IIC1: 中压传感器 SDA S2*/
#define IIC1_SDA_PIN   GPIO_PIN_5
#define IIC2_SCL_PORT  GPIOC    /* IIC2: 低压传感器 SCL S1*/
#define IIC2_SCL_PIN   GPIO_PIN_0
#define IIC2_SDA_PORT  GPIOC    /* IIC2: 低压传感器 SDA S1*/
#define IIC2_SDA_PIN   GPIO_PIN_1

/* ---------- 软件 I2C 总线速率 (半周期延时计数, 越小越快) ---------- */
#define SOFT_I2C_HALF_PERIOD       150

/* 传感器供电 (SENSOR_PWR_ENABLED/SENSOR_PWRUP_DELAY_MS) 已移至 BSP/power.h */
/* 电机引脚 (MOTOR_SLEEP_PORT/PIN, MOTOR_DRV_MODE) 已移至 BSP/motor.h */

#define APP_DEFAULT_REAL_ENABLED      1         /* 0=禁用, 1=启用秒级采集 */
#define APP_DEFAULT_REAL_FREQUENCY    5         /* 秒数 (1-10) */

/* ---------- 产品复用配置默认值 ---------- */
#define APP_DEFAULT_LCD_ENABLED       0         /* 0=无LCD产品, 1=有LCD产品 */
#define APP_DEFAULT_SENSOR_COUNT      2         /* 1=单路传感器(进气端), 2=双路传感器 */

/* ---------- 阈值默认值 ---------- */
/* 注: 阈值结构已拆分为 4 组独立配置, 默认值定义在 eeprom_config.h:
 *   DEF_VP_*  压力阈值 (vpthreshold, magic 0xC0)
 *   DEF_TT_*  温度阈值 (Tthreshold,  magic 0xC1)
 *   DEF_SP_*  电压检测 (spupower,    magic 0xC2)
 *   DEF_VD_*  阀门延时 (valve_delay, magic 0xC3)
 * 全部默认 0 + switch=0 禁用, 用户后续配置后才生效。
 */

/* ---------- 异常自动关阀策略默认值 ---------- */
#define APP_DEFAULT_ABNORMAL_CLOSE    "none"  /* 默认: 超上限或超下限自动关阀 */

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
typedef enum {
    ABNORMAL_NONE = 0,         /* 不自动关阀 */
    ABNORMAL_HIGH,             /* 压力超上限 */
    ABNORMAL_LOW,              /* 压力超下限 */
    ABNORMAL_MUTATION,         /* 压力突变 */
    ABNORMAL_HIGH_OR_LOW,      /* 超上限或超下限 */
    ABNORMAL_HIGH_OR_MUTATION, /* 超上限或突变 */
    ABNORMAL_LOW_OR_MUTATION,  /* 超下限或突变 */
    ABNORMAL_ALL,              /* 全部 (压力超高/超低/突变) */
    ABNORMAL_TEMP_HIGH,        /* 温度过高关阀 (temph)  含 TEMP_HIGH + TEMP_VERY_HIGH */
    ABNORMAL_TEMP_LOW,         /* 温度过低关阀 (templ)  含 TEMP_LOW  + TEMP_VERY_LOW */
    ABNORMAL_JGCLOSE           /* 激光甲烷报警关阀 (jgclose): CH4浓度>=门限(报警)即关阀, 平台 valve_open 恢复 */
    /* 注: 电压报警 (voltage_high/voltage_low) 只上报, 不动阀。
     *     电压保护 (voltage_protect, VPROTECT) 也只上报/进入VLP省电, 不关阀。
     */
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
