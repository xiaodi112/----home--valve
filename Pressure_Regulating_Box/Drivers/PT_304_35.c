#include "PT_304_35.h"
#include "soft_i2c.h"

/* ==================== 传感器总线映射与电源管理 ==================== */
/* S1=低压/出气 → IIC2 (PC0/PC1), S2=中压/进气 → IIC1 (PD4/PD5)
 * 两路传感器均由 PC3 供电+上拉 (见 soft_i2c.h / app_config.h) */
static const SoftI2C_t *pt304d_bus_of(uint8_t sensor_idx)
{
    return (sensor_idx == PT304D_SENSOR_1) ? &g_iic2 : &g_iic1;
}

/* 传感器断电后: IIC1/IIC2 引脚转 Analog, 彻底杜绝悬空漏电; 下次采集时 soft_i2c_init 重新配置 */
static void pt304d_bus_release(void)
{
    gpio_mode_set(IIC1_SCL_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, IIC1_SCL_PIN | IIC1_SDA_PIN);
    gpio_mode_set(IIC2_SCL_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, IIC2_SCL_PIN | IIC2_SDA_PIN);
}

/* 采集统一上电: 先开漏释放总线(防毛刺), 再 PC3 供电(含上电稳定延时) */
static void pt304d_power_on(void)
{
    soft_i2c_sensors_init();
    sensor_pwr_on();
}

/* 采集统一断电: PC3 拉低 + 总线转 Analog */
static void pt304d_power_off(void)
{
    sensor_pwr_off();
    pt304d_bus_release();
}

/************************** 量程辅助: 根据 sensor_idx 返回满量程 (Pa) **************************/

static float pt304d_get_fs(uint8_t sensor_idx)
{
    return (sensor_idx == PT304D_SENSOR_1) ? PT304D_S1_FS : PT304D_S2_FS;
}

/*===========================================================================
 *                    7 字节协议实现 (0xAA 指令型)
 *  编译条件: PT304D_DRV_7BYTE 已定义
 *=========================================================================*/
#ifdef PT304D_DRV_7BYTE

static PT304D_Status pt304d_7b_check_stat(uint8_t stat)
{
    if ((stat & PT304D_STAT_BUSY) != 0)
        return PT304D_ERR_BUSY;
    uint8_t mode_val = (stat & PT304D_STAT_MODE_MASK) >> 3;
    if (mode_val != PT304D_MODE_LOW_POWER)
        return PT304D_ERR_MODE;
    if ((stat & PT304D_STAT_POWERED) == 0)
        return PT304D_ERR_POWER;
    return PT304D_OK;
}

static float pt304d_7b_conv_press(uint32_t raw, float fs)
{
    return ((raw / (float)PT304D_7B_24BIT_MAX) - 0.1f) / (0.9f - 0.1f) * fs;
}

static float pt304d_7b_conv_temp(uint32_t raw)
{
    return PT304D_7B_TEMP_MIN + (raw / (float)PT304D_7B_24BIT_MAX) * (PT304D_7B_TEMP_MAX - PT304D_7B_TEMP_MIN);
}

static PT304D_Status pt304d_7b_send_meas(uint8_t sensor_idx)
{
    const SoftI2C_t *bus = pt304d_bus_of(sensor_idx);

    soft_i2c_start(bus);
    if(soft_i2c_write_byte(bus, (uint8_t)((PT304D_7B_ADDR << 1) | IIC_WR)) != 0) { soft_i2c_stop(bus); return PT304D_ERR_I2C; }
    if(soft_i2c_write_byte(bus, PT304D_7B_MEAS_CMD) != 0)                         { soft_i2c_stop(bus); return PT304D_ERR_I2C; }
    soft_i2c_stop(bus);
    return PT304D_OK;
}

static PT304D_Status pt304d_7b_read(uint8_t sensor_idx, PT304D_Data *p_data)
{
    const SoftI2C_t *bus = pt304d_bus_of(sensor_idx);
    uint8_t data_buf[PT304D_7B_READ_BYTES] = {0};
    uint8_t i;
    PT304D_Status ret;

    soft_i2c_start(bus);
    if(soft_i2c_write_byte(bus, (uint8_t)((PT304D_7B_ADDR << 1) | IIC_RD)) != 0)
    {
        soft_i2c_stop(bus);
        p_data->err = PT304D_ERR_I2C;
        return PT304D_ERR_I2C;
    }
    for(i = 0; i < PT304D_7B_READ_BYTES; i++)
        data_buf[i] = soft_i2c_read_byte(bus, (i < (PT304D_7B_READ_BYTES - 1)) ? 1 : 0);
    soft_i2c_stop(bus);

    p_data->stat_raw = data_buf[0];
    ret = pt304d_7b_check_stat(data_buf[0]);
    if (ret != PT304D_OK) { p_data->err = ret; return ret; }

    p_data->press_raw = ((uint32_t)data_buf[1] << 16) | ((uint32_t)data_buf[2] << 8) | data_buf[3];
    p_data->temp_raw  = ((uint32_t)data_buf[4] << 16) | ((uint32_t)data_buf[5] << 8) | data_buf[6];
    p_data->pressure    = pt304d_7b_conv_press(p_data->press_raw, pt304d_get_fs(sensor_idx));
    p_data->temperature = pt304d_7b_conv_temp(p_data->temp_raw);
    p_data->err = PT304D_OK;
    return PT304D_OK;
}

#endif /* PT304D_DRV_7BYTE */

/*===========================================================================
 *                    4 字节协议实现 (直读型, addr=0x28)
 *  编译条件: PT304D_DRV_4BYTE 已定义
 *=========================================================================*/
#ifdef PT304D_DRV_4BYTE

static PT304D_Status pt304d_4b_read(uint8_t sensor_idx, PT304D_Data *p_data)
{
    const SoftI2C_t *bus = pt304d_bus_of(sensor_idx);
    uint8_t buf[PT304D_4B_READ_BYTES];
    uint8_t i;

    trea_delay_ms(50);
    soft_i2c_start(bus);
    if(soft_i2c_write_byte(bus, (uint8_t)((PT304D_4B_ADDR << 1) | IIC_RD)) != 0)
    {
        soft_i2c_stop(bus);
        p_data->err = PT304D_ERR_I2C;
        return PT304D_ERR_I2C;
    }
    for(i = 0; i < PT304D_4B_READ_BYTES; i++)
        buf[i] = soft_i2c_read_byte(bus, (i < (PT304D_4B_READ_BYTES - 1)) ? 1 : 0);
    soft_i2c_stop(bus);

    /* 状态解析: Bit7:6 = status, 00=正常 */
    p_data->stat_raw = buf[0];
    uint8_t status = (buf[0] >> 6) & 0x03;
    if (status != 0)
    {
        p_data->err = PT304D_ERR_BUSY;
        return PT304D_ERR_BUSY;
    }

    /* 压力 14bit → Pa */
    uint16_t p_raw = ((uint16_t)(buf[0] & 0x3F) << 8) | buf[1];
    p_data->press_raw = (uint32_t)p_raw;
    float fs_kpa = pt304d_get_fs(sensor_idx) / 1000.0f;  /* Pa → kPa */
    if (p_raw <= PT304D_4B_ZERO_ADC)
        p_data->pressure = 0.0f;
    else
        p_data->pressure = (float)(p_raw - PT304D_4B_ZERO_ADC) * fs_kpa / (float)PT304D_4B_SPAN_ADC * 1000.0f;

    /* 温度 11bit → ℃ */
    uint16_t t_raw = ((uint16_t)buf[2] << 3) | ((buf[3] >> 5) & 0x07);
    p_data->temp_raw = (uint32_t)t_raw;
    p_data->temperature = 200.0f * (float)t_raw / (float)PT304D_4B_TEMP_SCALE - 50.0f;

    p_data->err = PT304D_OK;
    return PT304D_OK;
}

#endif /* PT304D_DRV_4BYTE */

/*===========================================================================
 *                    公共外部接口实现 (根据宏自动分派)
 *=========================================================================*/

PT304D_Status PT304D_Init(uint8_t sensor_idx)
{
    soft_i2c_init(pt304d_bus_of(sensor_idx));
    return PT304D_OK;
}

PT304D_Status PT304D_SendMeasCmd(uint8_t sensor_idx)
{
#ifdef PT304D_DRV_7BYTE
    if (!PT304D_IS_4BYTE(sensor_idx))
        return pt304d_7b_send_meas(sensor_idx);
#endif
#ifdef PT304D_DRV_4BYTE
    if (PT304D_IS_4BYTE(sensor_idx))
        return PT304D_OK;  /* 4字节协议无需发送测量指令 */
#endif
    return PT304D_OK;
}

PT304D_Status PT304D_ReadMeasData(uint8_t sensor_idx, PT304D_Data *p_data)
{
    if (p_data == NULL) return PT304D_ERR_NULLPTR;

#if defined(PT304D_DRV_7BYTE) && defined(PT304D_DRV_4BYTE)
    /* 双驱动模式: 运行时按通道选择 */
    if (PT304D_IS_4BYTE(sensor_idx))
        return pt304d_4b_read(sensor_idx, p_data);
    else
        return pt304d_7b_read(sensor_idx, p_data);
#elif defined(PT304D_DRV_4BYTE)
    return pt304d_4b_read(sensor_idx, p_data);
#else
    return pt304d_7b_read(sensor_idx, p_data);
#endif
}

PT304D_Status PT304D_Collect(uint8_t sensor_idx, PT304D_Data *p_data)
{
    PT304D_Status ret;

    if (p_data == NULL) return PT304D_ERR_NULLPTR;

    pt304d_power_on();

    ret = PT304D_SendMeasCmd(sensor_idx);
    if (ret != PT304D_OK) { p_data->err = ret; pt304d_power_off(); return ret; }

#ifdef PT304D_DRV_7BYTE
    if (!PT304D_IS_4BYTE(sensor_idx))
        trea_delay_ms(PT304D_7B_MEAS_DELAY);
#endif

    ret = PT304D_ReadMeasData(sensor_idx, p_data);

    pt304d_power_off();
    return ret;
}

/* 一次性采集两路传感器, PC3 仅上/断电一次, 效率更高
 * p_s1 = 低压/出气 (IIC2/PC0/PC1), p_s2 = 中压/进气 (IIC1/PD4/PD5) */
void PT304D_CollectAll(PT304D_Data *p_s1, PT304D_Data *p_s2)
{
    /* 先初始化总线 GPIO(开漏释放) 再 PC3 供电: 防止传感器上电瞬间看到毛刺 */
    pt304d_power_on();

    if (p_s1 != NULL)
    {
        if (PT304D_SendMeasCmd(PT304D_SENSOR_1) == PT304D_OK)
        {
#ifdef PT304D_DRV_7BYTE
            if (!PT304D_IS_4BYTE(PT304D_SENSOR_1))
                trea_delay_ms(PT304D_7B_MEAS_DELAY);
#endif
            PT304D_ReadMeasData(PT304D_SENSOR_1, p_s1);
        }
        else
        {
            p_s1->err = PT304D_ERR_I2C;
        }
    }

    if (p_s2 != NULL)
    {
        if (PT304D_SendMeasCmd(PT304D_SENSOR_2) == PT304D_OK)
        {
#ifdef PT304D_DRV_7BYTE
            if (!PT304D_IS_4BYTE(PT304D_SENSOR_2))
                trea_delay_ms(PT304D_7B_MEAS_DELAY);
#endif
            PT304D_ReadMeasData(PT304D_SENSOR_2, p_s2);
        }
        else
        {
            p_s2->err = PT304D_ERR_I2C;
        }
    }

    pt304d_power_off();
}

PT304D_Status PT304D_SetI2CAddr(uint8_t sensor_idx, uint8_t new_7bit_addr)
{
    const SoftI2C_t *bus = pt304d_bus_of(sensor_idx);

    if (new_7bit_addr > 0x7F) return PT304D_ERR_DATA;

    soft_i2c_start(bus);
    if(soft_i2c_write_byte(bus, (uint8_t)((PT304D_I2C_ADDR_7BIT << 1) | IIC_WR)) != 0) { soft_i2c_stop(bus); return PT304D_ERR_I2C; }
    if(soft_i2c_write_byte(bus, PT304D_ADDR_REG) != 0)                                { soft_i2c_stop(bus); return PT304D_ERR_I2C; }
    if(soft_i2c_write_byte(bus, new_7bit_addr) != 0)                                  { soft_i2c_stop(bus); return PT304D_ERR_I2C; }
    soft_i2c_stop(bus);
    return PT304D_OK;
}
