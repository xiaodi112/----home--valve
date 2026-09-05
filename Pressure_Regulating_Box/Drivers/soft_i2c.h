#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include "gd32l23x.h"
#include "app_config.h"

/* =====================================================================
 * 软件 I2C 总线驱动 (GPIO 模拟, 三组独立实例)
 *
 * 硬件:
 *   IIC0: EEPROM        SCL=PB6 SDA=PB7   (常供电, 常上拉)
 *   IIC1: 中压传感器    SCL=PD4 SDA=PD5   (常供电, 常上拉)
 *   IIC2: 低压传感器    SCL=PC0 SDA=PC1   (仅双传感器旧板; 新板 PC0/PC1 为 4G, 不可用)
 *
 * 引脚映射收敛在 app_config.h, 换板只改宏, 本驱动代码零改动。
 * ===================================================================== */

/* 总线描述符: 一条 GPIO 模拟 I2C 总线 = 一组 SCL/SDA 引脚 */
typedef struct {
    uint32_t rcu;          /* GPIO 时钟使能位 (RCU_GPIOx) */
    uint32_t scl_port;     /* SCL 端口 */
    uint32_t scl_pin;      /* SCL 引脚 */
    uint32_t sda_port;     /* SDA 端口 */
    uint32_t sda_pin;      /* SDA 引脚 */
    uint16_t half_period;  /* 半周期延时计数(空循环), 值越大速率越慢 */
} SoftI2C_t;

/* 三组总线实例 (定义于 soft_i2c.c) */
extern const SoftI2C_t g_iic0;   /* IIC0: EEPROM       (PB6/PB7) */
extern const SoftI2C_t g_iic1;   /* IIC1: 中压传感器   (PD4/PD5) */
extern const SoftI2C_t g_iic2;   /* IIC2: 低压传感器 (PC0/PC1, 双传感器旧板) */

/* 底层引脚操作宏 (寄存器直操, 单指令置位/清零, 效率最高) */
#define I2C_SCL_H(bus)   GPIO_BOP((bus)->scl_port) = (bus)->scl_pin
#define I2C_SCL_L(bus)   GPIO_BC((bus)->scl_port)  = (bus)->scl_pin
#define I2C_SDA_H(bus)   GPIO_BOP((bus)->sda_port) = (bus)->sda_pin
#define I2C_SDA_L(bus)   GPIO_BC((bus)->sda_port)  = (bus)->sda_pin
#define I2C_SDA_READ(bus)  ((GPIO_ISTAT((bus)->sda_port) & (bus)->sda_pin) != 0)

/* I2C 读写方向位 (兼容旧宏名) */
#ifndef IIC_WR
#define IIC_WR    0   /* 写操作 */
#define IIC_RD    1   /* 读操作 */
#endif

/* ---- 接口函数 ---- */
void    soft_i2c_init(const SoftI2C_t *bus);                     /* 开漏配置+总线死锁恢复+Stop */
void    soft_i2c_start(const SoftI2C_t *bus);
void    soft_i2c_stop(const SoftI2C_t *bus);
uint8_t soft_i2c_write_byte(const SoftI2C_t *bus, uint8_t data); /* 写1字节并读ACK: 0=ACK, 1=NACK */
uint8_t soft_i2c_read_byte(const SoftI2C_t *bus, uint8_t ack);   /* 读1字节, ack=1发送ACK */
uint8_t soft_i2c_check(const SoftI2C_t *bus, uint8_t addr);      /* 探测设备(7bit地址): 0=存在 */

/* ---- 传感器总线管理 ----
 * I2C 常供电板型: sensor_pwr_on/off 为空实现 (兼容旧调用点)。
 * PC3 已改作电压采集基准, 由 adc_bat 模块管理, 严禁在此驱动 PC3。 */
void soft_i2c_sensors_init(void);   /* 初始化 IIC1+IIC2 总线 GPIO (开漏释放) */
void sensor_pwr_on(void);           /* 空实现: 常供电板型 */
void sensor_pwr_off(void);          /* 空实现: 常供电板型 */

#endif /* SOFT_I2C_H */
