#include "PT_304_35.h"
#include "soft_i2c.h"

/* 每路传感器零点偏移(Pa): 校正压力 = 转换压力 - offset (默认0=出厂) */
static float s_pt304d_zero_offset[2] = { 0.0f, 0.0f };

/* 传感器供电保持标志: 静态测漏任务期间由 PT304D_PowerHold 管理, 避免每次采集断电 */
static uint8_t s_pt304d_pwr_held = 0;

void PT304D_SetZeroOffset(uint8_t sensor_idx, float offset_pa)
{
    if (sensor_idx <= 1)
        s_pt304d_zero_offset[sensor_idx] = offset_pa;
}

float PT304D_GetZeroOffset(uint8_t sensor_idx)
{
    return (sensor_idx <= 1) ? s_pt304d_zero_offset[sensor_idx] : 0.0f;
}

/* ==================== 传感器总线映射 ==================== */
static const SoftI2C_t *pt304d_bus_of(uint8_t sensor_idx)
{
    return (sensor_idx == PT304D_SENSOR_1) ? &g_iic2 : &g_iic1;
}

/* 采集结束后: IIC1/IIC2 引脚转 Analog */
static void pt304d_bus_release(void)
{
    gpio_mode_set(IIC1_SCL_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, IIC1_SCL_PIN | IIC1_SDA_PIN);
#if APP_DEFAULT_SENSOR_COUNT == 2
    gpio_mode_set(IIC2_SCL_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, IIC2_SCL_PIN | IIC2_SDA_PIN);
#endif
}

static void pt304d_power_on(void)
{
    soft_i2c_sensors_init();
    sensor_pwr_on();
    s_pt304d_pwr_held = 1;
}

static void pt304d_power_off(void)
{
    pt304d_bus_release();   /* 先释放总线(转Analog), 再断传感器电源: 避免向未上电器件倒灌电流 */
    sensor_pwr_off();
    s_pt304d_pwr_held = 0;
}

static float pt304d_get_fs(uint8_t sensor_idx)
{
    return (sensor_idx == PT304D_SENSOR_1) ? PT304D_S1_FS : PT304D_S2_FS;
}

/*===========================================================================
 *                    新传感器驱动 (PT304D_DRV_SENSOR)
 *  寄存器式校准系数传感器 (阀前0x76 / 阀后0x77)
 *=========================================================================*/
#ifdef PT304D_DRV_SENSOR

#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2
#define CONTINUOUS_P_AND_T      3
#define PRESSURE_SENSOR         0
#define TEMPERATURE_SENSOR      1

typedef struct {
    int16_t c0, c1;
    int32_t c00, c10;
    int16_t c01, c11, c20, c21, c30;
} Sensor_calib_param_t;

typedef struct {
    Sensor_calib_param_t calib_param;
    int32_t i32rawPressure;
    int32_t i32rawTemperature;
    int32_t i32kP;
    int32_t i32kT;
    uint8_t inited;
} Sensor_dev_t;

static Sensor_dev_t s_sensor_dev[2];

/* 地址映射: S1(阀后出气)=0x77, S2(阀前进气)=0x76 */
static uint8_t sensor_addr_of(uint8_t sensor_idx)
{
    return (sensor_idx == PT304D_SENSOR_1) ? PT304D_S1_I2C_ADDR : PT304D_S2_I2C_ADDR;
}

/* ---------- 底层 I2C 读写（带 ACK 检测） ---------- */
static uint8_t sensor_write_reg(const SoftI2C_t *bus, uint8_t hw_adr, uint8_t reg, uint8_t val)
{
    soft_i2c_start(bus);
    if (soft_i2c_write_byte(bus, (uint8_t)(hw_adr << 1)) != 0) { soft_i2c_stop(bus); return 1; }
    if (soft_i2c_write_byte(bus, reg) != 0)                     { soft_i2c_stop(bus); return 1; }
    if (soft_i2c_write_byte(bus, val) != 0)                     { soft_i2c_stop(bus); return 1; }
    soft_i2c_stop(bus);
    return 0;
}

static uint8_t sensor_read_reg(const SoftI2C_t *bus, uint8_t hw_adr, uint8_t reg, uint8_t *pval)
{
    soft_i2c_start(bus);
    if (soft_i2c_write_byte(bus, (uint8_t)(hw_adr << 1)) != 0) { soft_i2c_stop(bus); return 1; }
    if (soft_i2c_write_byte(bus, reg) != 0)                     { soft_i2c_stop(bus); return 1; }
    soft_i2c_start(bus);   /* repeated start */
    if (soft_i2c_write_byte(bus, (uint8_t)((hw_adr << 1) | 1)) != 0) { soft_i2c_stop(bus); return 1; }
    *pval = soft_i2c_read_byte(bus, 0);  /* NACK */
    soft_i2c_stop(bus);
    return 0;
}

/* ---------- 校准系数读取 ---------- */
static PT304D_Status sensor_get_calib_param(uint8_t sensor_idx)
{
    const SoftI2C_t *bus = pt304d_bus_of(sensor_idx);
    uint8_t hw_adr = sensor_addr_of(sensor_idx);
    Sensor_dev_t *dev = &s_sensor_dev[sensor_idx];
    uint8_t h, m, l;

    if (sensor_read_reg(bus, hw_adr, 0x10, &h)) return PT304D_ERR_I2C;
    if (sensor_read_reg(bus, hw_adr, 0x11, &l)) return PT304D_ERR_I2C;
    dev->calib_param.c0 = (int16_t)((h << 4) | (l >> 4));
    if (dev->calib_param.c0 & 0x0800) dev->calib_param.c0 |= 0xF000;

    if (sensor_read_reg(bus, hw_adr, 0x11, &h)) return PT304D_ERR_I2C;
    if (sensor_read_reg(bus, hw_adr, 0x12, &l)) return PT304D_ERR_I2C;
    dev->calib_param.c1 = (int16_t)(((h & 0x0F) << 8) | l);
    if (dev->calib_param.c1 & 0x0800) dev->calib_param.c1 |= 0xF000;

    if (sensor_read_reg(bus, hw_adr, 0x13, &h)) return PT304D_ERR_I2C;
    if (sensor_read_reg(bus, hw_adr, 0x14, &m)) return PT304D_ERR_I2C;
    if (sensor_read_reg(bus, hw_adr, 0x15, &l)) return PT304D_ERR_I2C;
    dev->calib_param.c00 = (int32_t)((h << 12) | (m << 4) | (l >> 4));
    if (dev->calib_param.c00 & 0x080000) dev->calib_param.c00 |= 0xFFF00000;

    if (sensor_read_reg(bus, hw_adr, 0x15, &h)) return PT304D_ERR_I2C;
    if (sensor_read_reg(bus, hw_adr, 0x16, &m)) return PT304D_ERR_I2C;
    if (sensor_read_reg(bus, hw_adr, 0x17, &l)) return PT304D_ERR_I2C;
    dev->calib_param.c10 = (int32_t)(((h & 0x0F) << 16) | (m << 8) | l);
    if (dev->calib_param.c10 & 0x080000) dev->calib_param.c10 |= 0xFFF00000;

    if (sensor_read_reg(bus, hw_adr, 0x18, &h)) return PT304D_ERR_I2C;
    if (sensor_read_reg(bus, hw_adr, 0x19, &l)) return PT304D_ERR_I2C;
    dev->calib_param.c01 = (int16_t)((h << 8) | l);

    if (sensor_read_reg(bus, hw_adr, 0x1A, &h)) return PT304D_ERR_I2C;
    if (sensor_read_reg(bus, hw_adr, 0x1B, &l)) return PT304D_ERR_I2C;
    dev->calib_param.c11 = (int16_t)((h << 8) | l);

    if (sensor_read_reg(bus, hw_adr, 0x1C, &h)) return PT304D_ERR_I2C;
    if (sensor_read_reg(bus, hw_adr, 0x1D, &l)) return PT304D_ERR_I2C;
    dev->calib_param.c20 = (int16_t)((h << 8) | l);

    if (sensor_read_reg(bus, hw_adr, 0x1E, &h)) return PT304D_ERR_I2C;
    if (sensor_read_reg(bus, hw_adr, 0x1F, &l)) return PT304D_ERR_I2C;
    dev->calib_param.c21 = (int16_t)((h << 8) | l);

    if (sensor_read_reg(bus, hw_adr, 0x20, &h)) return PT304D_ERR_I2C;
    if (sensor_read_reg(bus, hw_adr, 0x21, &l)) return PT304D_ERR_I2C;
    dev->calib_param.c30 = (int16_t)((h << 8) | l);

    return PT304D_OK;
}

/* ---------- 过采样 / 采样率设置 (默认 OS=1, Rate=1) ---------- */
static PT304D_Status sensor_rateset(uint8_t sensor_idx, uint8_t iSensor, uint8_t u8OverSmpl, uint8_t u8SmplRate)
{
    const SoftI2C_t *bus = pt304d_bus_of(sensor_idx);
    uint8_t hw_adr = sensor_addr_of(sensor_idx);
    Sensor_dev_t *dev = &s_sensor_dev[sensor_idx];
    uint8_t reg = 0;
    int32_t i32kPkT = 524288;   /* OS=1 default */

    switch (u8SmplRate) {
        case 2:  reg |= (1 << 4); break;
        case 4:  reg |= (2 << 4); break;
        case 8:  reg |= (3 << 4); break;
        case 16: reg |= (4 << 4); break;
        case 32: reg |= (5 << 4); break;
        case 64: reg |= (6 << 4); break;
        case 128:reg |= (7 << 4); break;
        default: break;
    }

    switch (u8OverSmpl) {
        case 2:  reg |= 1; i32kPkT = 1572864;  break;
        case 4:  reg |= 2; i32kPkT = 3670016;  break;
        case 8:  reg |= 3; i32kPkT = 7864320;  break;
        case 16: reg |= 4; i32kPkT = 253952;   break;
        case 32: reg |= 5; i32kPkT = 516096;   break;
        case 64: reg |= 6; i32kPkT = 1040384;  break;
        case 128:reg |= 7; i32kPkT = 2088960;  break;
        default: break;
    }

    if (iSensor == PRESSURE_SENSOR) {
        dev->i32kP = i32kPkT;
        if (sensor_write_reg(bus, hw_adr, 0x06, reg)) return PT304D_ERR_I2C;
        uint8_t tmp;
        if (sensor_read_reg(bus, hw_adr, 0x09, &tmp)) return PT304D_ERR_I2C;
        if (u8OverSmpl > 8)
            sensor_write_reg(bus, hw_adr, 0x09, tmp | 0x04);
        else
            sensor_write_reg(bus, hw_adr, 0x09, tmp & (~0x04));
    } else {
        dev->i32kT = i32kPkT;
        if (sensor_write_reg(bus, hw_adr, 0x07, reg | 0x80)) return PT304D_ERR_I2C;  /* MEMS temp */
        uint8_t tmp;
        if (sensor_read_reg(bus, hw_adr, 0x09, &tmp)) return PT304D_ERR_I2C;
        if (u8OverSmpl > 8)
            sensor_write_reg(bus, hw_adr, 0x09, tmp | 0x08);
        else
            sensor_write_reg(bus, hw_adr, 0x09, tmp & (~0x08));
    }
    return PT304D_OK;
}

/* ---------- 初始化（读取校准 + 设置默认采样参数） ---------- */
static PT304D_Status sensor_init_dev(uint8_t sensor_idx)
{
    Sensor_dev_t *dev = &s_sensor_dev[sensor_idx];
    if (dev->inited) return PT304D_OK;

    trea_delay_ms(40);

    if (sensor_get_calib_param(sensor_idx) != PT304D_OK)
        return PT304D_ERR_I2C;

    if (sensor_rateset(sensor_idx, PRESSURE_SENSOR, 1, 1) != PT304D_OK)
        return PT304D_ERR_I2C;
    trea_delay_ms(15);

    if (sensor_rateset(sensor_idx, TEMPERATURE_SENSOR, 1, 1) != PT304D_OK)
        return PT304D_ERR_I2C;
    trea_delay_ms(5);

    dev->inited = 1;
    return PT304D_OK;
}

/* ---------- 启动单次测量 ---------- */
static PT304D_Status sensor_start_pressure(uint8_t sensor_idx)
{
    return sensor_write_reg(pt304d_bus_of(sensor_idx), sensor_addr_of(sensor_idx), 0x08, 0x01)
           ? PT304D_ERR_I2C : PT304D_OK;
}

static PT304D_Status sensor_start_temperature(uint8_t sensor_idx)
{
    return sensor_write_reg(pt304d_bus_of(sensor_idx), sensor_addr_of(sensor_idx), 0x08, 0x02)
           ? PT304D_ERR_I2C : PT304D_OK;
}

/* ---------- 读取原始值 ---------- */
static PT304D_Status sensor_get_raw_pressure(uint8_t sensor_idx)
{
    const SoftI2C_t *bus = pt304d_bus_of(sensor_idx);
    uint8_t hw_adr = sensor_addr_of(sensor_idx);
    Sensor_dev_t *dev = &s_sensor_dev[sensor_idx];
    uint8_t h, m, l;

    if (sensor_read_reg(bus, hw_adr, 0x00, &h)) return PT304D_ERR_I2C;
    if (sensor_read_reg(bus, hw_adr, 0x01, &m)) return PT304D_ERR_I2C;
    if (sensor_read_reg(bus, hw_adr, 0x02, &l)) return PT304D_ERR_I2C;

    dev->i32rawPressure = (int32_t)((h << 16) | (m << 8) | l);
    if (dev->i32rawPressure & 0x800000)
        dev->i32rawPressure |= 0xFF000000;
    return PT304D_OK;
}

static PT304D_Status sensor_get_raw_temp(uint8_t sensor_idx)
{
    const SoftI2C_t *bus = pt304d_bus_of(sensor_idx);
    uint8_t hw_adr = sensor_addr_of(sensor_idx);
    Sensor_dev_t *dev = &s_sensor_dev[sensor_idx];
    uint8_t h, m, l;

    if (sensor_read_reg(bus, hw_adr, 0x03, &h)) return PT304D_ERR_I2C;
    if (sensor_read_reg(bus, hw_adr, 0x04, &m)) return PT304D_ERR_I2C;
    if (sensor_read_reg(bus, hw_adr, 0x05, &l)) return PT304D_ERR_I2C;

    dev->i32rawTemperature = (int32_t)((h << 16) | (m << 8) | l);
    if (dev->i32rawTemperature & 0x800000)
        dev->i32rawTemperature |= 0xFF000000;
    return PT304D_OK;
}

/* ---------- 温度/压力补偿计算 ---------- */
static float sensor_calc_temperature(Sensor_dev_t *dev)
{
    float fTsc = dev->i32rawTemperature / (float)dev->i32kT;
    return dev->calib_param.c0 * 0.5f + dev->calib_param.c1 * fTsc;
}

static float sensor_calc_pressure(Sensor_dev_t *dev)
{
    float fTsc = dev->i32rawTemperature / (float)dev->i32kT;
    float fPsc = dev->i32rawPressure / (float)dev->i32kP;
    float qua2 = dev->calib_param.c10 + fPsc * (dev->calib_param.c20 + fPsc * dev->calib_param.c30);
    float qua3 = fTsc * fPsc * (dev->calib_param.c11 + fPsc * dev->calib_param.c21);
    return dev->calib_param.c00 + fPsc * qua2 + fTsc * dev->calib_param.c01 + qua3;
}

/* ---------- 完整单次采集实现 ---------- */
static PT304D_Status sensor_read(uint8_t sensor_idx, PT304D_Data *p_data)
{
    PT304D_Status ret;
    Sensor_dev_t *dev = &s_sensor_dev[sensor_idx];

    /* 懒初始化 */
    ret = sensor_init_dev(sensor_idx);
    if (ret != PT304D_OK) {
        p_data->err = ret;
        return ret;
    }

    /* 测量压力 */
    ret = sensor_start_pressure(sensor_idx);
    if (ret != PT304D_OK) { p_data->err = ret; return ret; }
    trea_delay_ms(20);   /* OS=1 足够，可按需调整 */

    ret = sensor_get_raw_pressure(sensor_idx);
    if (ret != PT304D_OK) { p_data->err = ret; return ret; }

    /* 测量温度 */
    ret = sensor_start_temperature(sensor_idx);
    if (ret != PT304D_OK) { p_data->err = ret; return ret; }
    trea_delay_ms(20);

    ret = sensor_get_raw_temp(sensor_idx);
    if (ret != PT304D_OK) { p_data->err = ret; return ret; }

    /* 计算物理量 */
    p_data->press_raw   = (uint32_t)dev->i32rawPressure;
    p_data->temp_raw    = (uint32_t)dev->i32rawTemperature;
    p_data->temperature = sensor_calc_temperature(dev);
    p_data->pressure    = sensor_calc_pressure(dev);
    p_data->pressure   -= s_pt304d_zero_offset[sensor_idx];
    p_data->stat_raw    = 0;
    p_data->err         = PT304D_OK;

    return PT304D_OK;
}

#endif /* PT304D_DRV_SENSOR */

/*===========================================================================
 *                    7 字节协议实现 (保留)
 *=========================================================================*/
#ifdef PT304D_DRV_7BYTE
/* ... 原有 7BYTE 代码保持不变 ... */
static PT304D_Status pt304d_7b_check_stat(uint8_t stat)
{
    if ((stat & PT304D_STAT_BUSY) != 0) return PT304D_ERR_BUSY;
    uint8_t mode_val = (stat & PT304D_STAT_MODE_MASK) >> 3;
    if (mode_val != PT304D_MODE_LOW_POWER) return PT304D_ERR_MODE;
    if ((stat & PT304D_STAT_POWERED) == 0) return PT304D_ERR_POWER;
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
    if (soft_i2c_write_byte(bus, (uint8_t)((PT304D_7B_ADDR << 1) | IIC_WR)) != 0) { soft_i2c_stop(bus); return PT304D_ERR_I2C; }
    if (soft_i2c_write_byte(bus, PT304D_7B_MEAS_CMD) != 0)                         { soft_i2c_stop(bus); return PT304D_ERR_I2C; }
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
    if (soft_i2c_write_byte(bus, (uint8_t)((PT304D_7B_ADDR << 1) | IIC_RD)) != 0) {
        soft_i2c_stop(bus);
        p_data->err = PT304D_ERR_I2C;
        return PT304D_ERR_I2C;
    }
    for (i = 0; i < PT304D_7B_READ_BYTES; i++)
        data_buf[i] = soft_i2c_read_byte(bus, (i < (PT304D_7B_READ_BYTES - 1)) ? 1 : 0);
    soft_i2c_stop(bus);

    p_data->stat_raw = data_buf[0];
    ret = pt304d_7b_check_stat(data_buf[0]);
    if (ret != PT304D_OK) { p_data->err = ret; return ret; }

    p_data->press_raw = ((uint32_t)data_buf[1] << 16) | ((uint32_t)data_buf[2] << 8) | data_buf[3];
    p_data->temp_raw  = ((uint32_t)data_buf[4] << 16) | ((uint32_t)data_buf[5] << 8) | data_buf[6];
    p_data->pressure    = pt304d_7b_conv_press(p_data->press_raw, pt304d_get_fs(sensor_idx));
    p_data->temperature = pt304d_7b_conv_temp(p_data->temp_raw);
    p_data->pressure   -= s_pt304d_zero_offset[sensor_idx];
    p_data->err = PT304D_OK;
    return PT304D_OK;
}
#endif /* PT304D_DRV_7BYTE */

/*===========================================================================
 *                    4 字节协议实现 (保留)
 *=========================================================================*/
#ifdef PT304D_DRV_4BYTE
static PT304D_Status pt304d_4b_read(uint8_t sensor_idx, PT304D_Data *p_data)
{
    const SoftI2C_t *bus = pt304d_bus_of(sensor_idx);
    uint8_t buf[PT304D_4B_READ_BYTES];
    uint8_t i;

    trea_delay_ms(50);
    soft_i2c_start(bus);
    if (soft_i2c_write_byte(bus, (uint8_t)((PT304D_4B_ADDR << 1) | IIC_RD)) != 0) {
        soft_i2c_stop(bus);
        p_data->err = PT304D_ERR_I2C;
        return PT304D_ERR_I2C;
    }
    for (i = 0; i < PT304D_4B_READ_BYTES; i++)
        buf[i] = soft_i2c_read_byte(bus, (i < (PT304D_4B_READ_BYTES - 1)) ? 1 : 0);
    soft_i2c_stop(bus);

    p_data->stat_raw = buf[0];
    uint8_t status = (buf[0] >> 6) & 0x03;
    if (status != 0) {
        p_data->err = PT304D_ERR_BUSY;
        return PT304D_ERR_BUSY;
    }

    uint16_t p_raw = ((uint16_t)(buf[0] & 0x3F) << 8) | buf[1];
    p_data->press_raw = (uint32_t)p_raw;
    float fs_kpa = pt304d_get_fs(sensor_idx) / 1000.0f;
    if (p_raw <= PT304D_4B_ZERO_ADC)
        p_data->pressure = 0.0f;
    else
        p_data->pressure = (float)(p_raw - PT304D_4B_ZERO_ADC) * fs_kpa / (float)PT304D_4B_SPAN_ADC * 1000.0f;

    uint16_t t_raw = ((uint16_t)buf[2] << 3) | ((buf[3] >> 5) & 0x07);
    p_data->temp_raw = (uint32_t)t_raw;
    p_data->temperature = 200.0f * (float)t_raw / (float)PT304D_4B_TEMP_SCALE - 50.0f;

    p_data->pressure -= s_pt304d_zero_offset[sensor_idx];
    p_data->err = PT304D_OK;
    return PT304D_OK;
}
#endif /* PT304D_DRV_4BYTE */

/*===========================================================================
 *                    公共外部接口实现
 *=========================================================================*/

PT304D_Status PT304D_Init(uint8_t sensor_idx)
{
    soft_i2c_init(pt304d_bus_of(sensor_idx));
#ifdef PT304D_DRV_SENSOR
    /* 新传感器在首次 Collect 时懒初始化，这里只保证总线可用 */
#endif
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
        return PT304D_OK;
#endif
#ifdef PT304D_DRV_SENSOR
    return PT304D_OK;   /* 新传感器在 Read 内部启动测量 */
#endif
    return PT304D_OK;
}

PT304D_Status PT304D_ReadMeasData(uint8_t sensor_idx, PT304D_Data *p_data)
{
    if (p_data == NULL) return PT304D_ERR_NULLPTR;

#if defined(PT304D_DRV_SENSOR)
    return sensor_read(sensor_idx, p_data);
#elif defined(PT304D_DRV_7BYTE) && defined(PT304D_DRV_4BYTE)
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

void PT304D_CollectAll(PT304D_Data *p_s1, PT304D_Data *p_s2)
{
    pt304d_power_on();

    if (p_s1 != NULL) {
        if (PT304D_SendMeasCmd(PT304D_SENSOR_1) == PT304D_OK) {
#ifdef PT304D_DRV_7BYTE
            if (!PT304D_IS_4BYTE(PT304D_SENSOR_1))
                trea_delay_ms(PT304D_7B_MEAS_DELAY);
#endif
            PT304D_ReadMeasData(PT304D_SENSOR_1, p_s1);
        } else {
            p_s1->err = PT304D_ERR_I2C;
        }
    }

    if (p_s2 != NULL) {
        if (PT304D_SendMeasCmd(PT304D_SENSOR_2) == PT304D_OK) {
#ifdef PT304D_DRV_7BYTE
            if (!PT304D_IS_4BYTE(PT304D_SENSOR_2))
                trea_delay_ms(PT304D_7B_MEAS_DELAY);
#endif
            PT304D_ReadMeasData(PT304D_SENSOR_2, p_s2);
        } else {
            p_s2->err = PT304D_ERR_I2C;
        }
    }

    pt304d_power_off();
}

/* ==================== 静态测漏: 保持供电采集 ====================
 * 常规采集(PT304D_CollectAll)每次上电/断电; 静态测漏任务采样频率高且要求读数稳定,
 * 因此任务期间由 PT304D_PowerHold(1) 保持传感器供电, 用本函数读数据(不切电源)。
 * PT304D_PowerHold(1) 是幂等的: 若供电被其它路径断开, 会重新上电并等待稳定。 */
void PT304D_PowerHold(uint8_t on)
{
    if(on)
    {
        if(!s_pt304d_pwr_held)
            pt304d_power_on();      /* soft_i2c_sensors_init + sensor_pwr_on(含稳定等待) */
    }
    else
    {
        if(s_pt304d_pwr_held)
            pt304d_power_off();
    }
}

void PT304D_CollectAllHeld(PT304D_Data *p_s1, PT304D_Data *p_s2)
{
    /* 与 PT304D_CollectAll 相同, 但不做上/断电: 供电由 PT304D_PowerHold 管理 */
    if (p_s1 != NULL) {
        if (PT304D_SendMeasCmd(PT304D_SENSOR_1) == PT304D_OK) {
#ifdef PT304D_DRV_7BYTE
            if (!PT304D_IS_4BYTE(PT304D_SENSOR_1))
                trea_delay_ms(PT304D_7B_MEAS_DELAY);
#endif
            PT304D_ReadMeasData(PT304D_SENSOR_1, p_s1);
        } else {
            p_s1->err = PT304D_ERR_I2C;
        }
    }

    if (p_s2 != NULL) {
        if (PT304D_SendMeasCmd(PT304D_SENSOR_2) == PT304D_OK) {
#ifdef PT304D_DRV_7BYTE
            if (!PT304D_IS_4BYTE(PT304D_SENSOR_2))
                trea_delay_ms(PT304D_7B_MEAS_DELAY);
#endif
            PT304D_ReadMeasData(PT304D_SENSOR_2, p_s2);
        } else {
            p_s2->err = PT304D_ERR_I2C;
        }
    }
}

PT304D_Status PT304D_SetI2CAddr(uint8_t sensor_idx, uint8_t new_7bit_addr)
{
    /* 新传感器通常不支持软件改地址，或需按具体手册实现；此处保留空实现兼容 */
#ifdef PT304D_DRV_SENSOR
    (void)sensor_idx;
    (void)new_7bit_addr;
    return PT304D_ERR_DATA;
#else
    const SoftI2C_t *bus = pt304d_bus_of(sensor_idx);
    if (new_7bit_addr > 0x7F) return PT304D_ERR_DATA;

    soft_i2c_start(bus);
    if (soft_i2c_write_byte(bus, (uint8_t)((PT304D_I2C_ADDR_7BIT << 1) | IIC_WR)) != 0) { soft_i2c_stop(bus); return PT304D_ERR_I2C; }
    if (soft_i2c_write_byte(bus, PT304D_ADDR_REG) != 0)                                { soft_i2c_stop(bus); return PT304D_ERR_I2C; }
    if (soft_i2c_write_byte(bus, new_7bit_addr) != 0)                                  { soft_i2c_stop(bus); return PT304D_ERR_I2C; }
    soft_i2c_stop(bus);
    return PT304D_OK;
#endif
}
