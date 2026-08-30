#include "soft_i2c.h"
#include "power.h"
#include "delay.h"

/* ==================== 三组总线实例 ==================== */
/* half_period 统一取 SOFT_I2C_HALF_PERIOD (app_config.h), 默认 150 与原驱动一致;
 * 需要提速时改小该宏即可, 不必动驱动代码。 */
const SoftI2C_t g_iic0 = { RCU_GPIOB, IIC0_SCL_PORT, IIC0_SCL_PIN, IIC0_SDA_PORT, IIC0_SDA_PIN, SOFT_I2C_HALF_PERIOD };
const SoftI2C_t g_iic1 = { RCU_GPIOD, IIC1_SCL_PORT, IIC1_SCL_PIN, IIC1_SDA_PORT, IIC1_SDA_PIN, SOFT_I2C_HALF_PERIOD };
const SoftI2C_t g_iic2 = { RCU_GPIOC, IIC2_SCL_PORT, IIC2_SCL_PIN, IIC2_SDA_PORT, IIC2_SDA_PIN, SOFT_I2C_HALF_PERIOD };

/* 半周期延时: 空循环, 编译后每条约1~2周期 */
static void i2c_delay(uint16_t n)
{
    volatile uint16_t i;
    for(i = 0; i < n; i++);
}

/* 开漏输出 + 内部上拉, 总线死锁恢复(9个SCL脉冲), 结束时释放总线 */
void soft_i2c_init(const SoftI2C_t *bus)
{
    uint8_t i;

    rcu_periph_clock_enable(bus->rcu);

    gpio_mode_set(bus->scl_port, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, bus->scl_pin);
    gpio_mode_set(bus->sda_port, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, bus->sda_pin);
    gpio_output_options_set(bus->scl_port, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, bus->scl_pin);
    gpio_output_options_set(bus->sda_port, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, bus->sda_pin);

    I2C_SCL_H(bus);
    I2C_SDA_H(bus);
    i2c_delay(bus->half_period);

    /* 总线死锁恢复: 若 SDA 被从机拉低, 发送9个SCL脉冲释放总线 */
    if(I2C_SDA_READ(bus) == 0)
    {
        for(i = 0; i < 9; i++)
        {
            I2C_SCL_L(bus);
            i2c_delay(bus->half_period);
            I2C_SCL_H(bus);
            i2c_delay(bus->half_period);
            if(I2C_SDA_READ(bus) != 0) break;
        }
    }

    soft_i2c_stop(bus);
}

void soft_i2c_start(const SoftI2C_t *bus)
{
    I2C_SDA_H(bus);
    I2C_SCL_H(bus);
    i2c_delay(bus->half_period);
    I2C_SDA_L(bus);
    i2c_delay(bus->half_period);
    I2C_SCL_L(bus);
    i2c_delay(bus->half_period);
}

void soft_i2c_stop(const SoftI2C_t *bus)
{
    I2C_SCL_L(bus);
    I2C_SDA_L(bus);
    i2c_delay(bus->half_period);
    I2C_SCL_H(bus);
    i2c_delay(bus->half_period);
    I2C_SDA_H(bus);
    i2c_delay(bus->half_period);
}

/* 写1字节并在第9个时钟读 ACK: 返回 0=ACK(设备响应), 1=NACK */
uint8_t soft_i2c_write_byte(const SoftI2C_t *bus, uint8_t data)
{
    int8_t i;
    uint8_t ack;

    for(i = 7; i >= 0; i--)
    {
        I2C_SCL_L(bus);
        if(data & (1u << i)) I2C_SDA_H(bus);
        else                 I2C_SDA_L(bus);
        i2c_delay(bus->half_period);
        I2C_SCL_H(bus);
        i2c_delay(bus->half_period);
    }

    /* 第9个时钟: SDA 释放, 读从机 ACK (低电平=ACK) */
    I2C_SCL_L(bus);
    I2C_SDA_H(bus);
    i2c_delay(bus->half_period);
    I2C_SCL_H(bus);
    i2c_delay(bus->half_period);
    ack = (I2C_SDA_READ(bus) != 0) ? 1 : 0;
    I2C_SCL_L(bus);
    i2c_delay(bus->half_period);

    return ack;
}

/* 读1字节; ack=1 时第9个时钟发 ACK(继续读), ack=0 发 NACK(最后字节) */
uint8_t soft_i2c_read_byte(const SoftI2C_t *bus, uint8_t ack)
{
    int8_t i;
    uint8_t value = 0;

    I2C_SDA_H(bus);
    for(i = 7; i >= 0; i--)
    {
        I2C_SCL_L(bus);
        i2c_delay(bus->half_period);
        I2C_SCL_H(bus);
        i2c_delay(bus->half_period);
        if(I2C_SDA_READ(bus)) value |= (1u << i);
    }

    /* 第9个时钟: 发送 ACK/NACK */
    I2C_SCL_L(bus);
    if(ack) I2C_SDA_L(bus);
    else    I2C_SDA_H(bus);
    i2c_delay(bus->half_period);
    I2C_SCL_H(bus);
    i2c_delay(bus->half_period);
    I2C_SCL_L(bus);
    I2C_SDA_H(bus);
    i2c_delay(bus->half_period);

    return value;
}

/* 探测 7bit 地址设备: 返回 0=存在(ACK), 1=不存在 */
uint8_t soft_i2c_check(const SoftI2C_t *bus, uint8_t addr)
{
    uint8_t ack;

    soft_i2c_start(bus);
    ack = soft_i2c_write_byte(bus, (uint8_t)((addr << 1) | 0x00));
    soft_i2c_stop(bus);
    return ack;
}

/* 初始化两条传感器总线 GPIO (开漏释放); 采集前调用, 避免传感器上电瞬间看到毛刺 */
void soft_i2c_sensors_init(void)
{
    soft_i2c_init(&g_iic1);
    soft_i2c_init(&g_iic2);
}

/* ==================== 传感器电源/上拉控制 (PC3) ==================== */
#if SENSOR_PWR_ENABLED
void sensor_pwr_on(void)
{
    /* 显式配置 PC3 为推挽输出再置位: 最小唤醒路径不调用 gpio_init,
     * 休眠时 gpio_enter_low_power 已把 PC3 置为 ANALOG, 直接写 ODR 无效,
     * 会导致传感器供电/上拉缺失、I2C 读到全 0 (历史数据异常) */
    rcu_periph_clock_enable(RCU_GPIOC);
    gpio_mode_set(PWR_SENSOR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_SENSOR_PIN);
    gpio_output_options_set(PWR_SENSOR_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, PWR_SENSOR_PIN);
    power_sensor_set(1);
    trea_delay_ms(SENSOR_PWRUP_DELAY_MS);   /* 传感器上电稳定 */
}

void sensor_pwr_off(void)
{
    power_sensor_set(0);
}
#else
/* 传感器常供电板型: 电源控制为空实现 */
void sensor_pwr_on(void)  { }
void sensor_pwr_off(void) { }
#endif
