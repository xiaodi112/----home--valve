#ifndef CALIB_MODE_H
#define CALIB_MODE_H

#include <stdint.h>

/* ==================== 未安装自动校零 (校零服务) ====================
 * 未安装模式 = 正常模式 + 三个差异:
 *   1) 采集节拍 = CALIB_SAMPLE_MIN 分钟 (覆盖配置的 collect_cycle)
 *   2) LCD 压力显示 0 (其它页不变)
 *   3) 持续自动校零: 零点成熟即时生效, 离窗冻结, 自动/强制锁定后转正常
 * 其余(上报/下行/报警/阀门/4G策略)全部与正常模式一致.
 * ========================================================================== */

/* 采集节拍(分钟, 宏固定) */
#define CALIB_SAMPLE_MIN         1u

/* 校零判定参数 */
#define CALIB_ZERO_WINDOW_PA     900.0f   /* 零压窗口: ±900 Pa */
#define CALIB_INSTALL_PA         1000.0f  /* 安装判定阈值: ≥1000 Pa */
#define CALIB_WARMUP_CNT         2u       /* 上电预热样本数 */
#define CALIB_HIGH_CONFIRM_CNT   2u       /* 连续高位确认次数 */
#define CALIB_MEDIAN_CNT         5u       /* 零点滑窗长度 */

/* ==================== 校零/已安装保护区 (EEPROM) ====================
 * 普通恢复出厂(配置区)不清此保护区; 回退仅全片擦除/工程通道.
 * Slot A: 0x500 / Slot B: 0x520, 双槽 + 递增 seq + CRC8.
 */
#define EE_MAGIC_CALIB           0xD1
#define EE_CALIB_VERSION         1
#define EE_ADDR_CALIB_SLOT_A     0x500
#define EE_ADDR_CALIB_SLOT_B     0x520

typedef struct {
    uint8_t  magic;
    uint8_t  version;
    uint8_t  installed;    /* 1=已安装单向锁定 */
    uint8_t  zero_valid;   /* 1=算法零点有效; 0=0偏移 */
    uint16_t seq;
    int32_t  zero_pa;      /* 校正压力 = 原始压力 - zero_pa */
    uint32_t sample_cnt;
    uint8_t  ts[6];        /* BCD: YY MM DD HH MM SS */
    uint8_t  reserved[7];
    uint8_t  crc8;
} CalibEeprom_t;

/* ---- 服务接口 (供 main/downlink/lcd_ui 调用) ---- */

/* 开机读取双槽记录; 无有效记录时默认 未安装/无零点 */
void calib_load_state(void);
/* 清除保护区 (恢复出厂 reset 命令调用) */
void calib_erase_state(void);

uint8_t calib_is_installed(void);
uint8_t calib_zero_valid(void);
int32_t calib_zero_pa(void);

/* 未安装校零是否激活 (main 开机按 installed 标志设置) */
uint8_t calib_is_active(void);
void    calib_set_active(uint8_t on);

/* 校零诊断状态 (日志用) */
typedef struct {
    uint8_t  warm;
    uint8_t  hi;
    uint8_t  med_n;
    uint8_t  zero_valid;
    int32_t  zero_pa;
    uint32_t samples;
} CalibStatus_t;
void calib_get_status(CalibStatus_t *st);

/* LCD 压力显示是否强制0 (未安装校零期) */
uint8_t calib_lcd_zero(void);

/* 采集样本喂入: 零点成熟立即生效.
 * 返回"本样本需额外扣除的修正量"-" (新零点 - 上次已应用零点), main 用它修正当前读数 */
float calib_feed_sample(float raw_pa);

/* 平台强制置已安装 (install_status_set=1): 幂等, 成功返回1 */
uint8_t calib_force_install(void);

/* 将当前/锁定零点应用到 PT304D 转换出口 */
void calib_apply_zero_offsets(void);

#endif /* CALIB_MODE_H */
