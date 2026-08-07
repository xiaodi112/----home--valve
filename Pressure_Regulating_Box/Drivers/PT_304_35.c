#include "PT_304_35.h"
#include "i2c_eeprom.h"
#include "i2c_sensor.h"

/************************** PD4 SW_IIC 内部辅助 **************************/
static void pt304d_sw_iic_on(void)
{
    rcu_periph_clock_enable(RCU_GPIOD);
    gpio_mode_set(PT304D_SW_IIC_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PT304D_SW_IIC_PIN);
    gpio_output_options_set(PT304D_SW_IIC_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, PT304D_SW_IIC_PIN);
    gpio_bit_set(PT304D_SW_IIC_PORT, PT304D_SW_IIC_PIN);
    trea_delay_ms(PT304D_PWRUP_DELAY_MS);
}

static void pt304d_sw_iic_off(void)
{
    gpio_bit_reset(PT304D_SW_IIC_PORT, PT304D_SW_IIC_PIN);
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
    if (sensor_idx == PT304D_SENSOR_1)
    {
        IIC_Start();
        IIC_Send_Byte((PT304D_7B_ADDR << 1) | IIC_WR);
        if (IIC_Wait_Ack(0) != 0) { IIC_Stop(); return PT304D_ERR_I2C; }
        IIC_Send_Byte(PT304D_7B_MEAS_CMD);
        if (IIC_Wait_Ack(0) != 0) { IIC_Stop(); return PT304D_ERR_I2C; }
        IIC_Stop();
    }
    else
    {
        IIC_Start1();
        IIC_Send_Byte1((PT304D_7B_ADDR << 1) | IIC_WR);
        if (IIC_Wait_Ack1(0) != 0) { IIC_Stop1(); return PT304D_ERR_I2C; }
        IIC_Send_Byte1(PT304D_7B_MEAS_CMD);
        if (IIC_Wait_Ack1(0) != 0) { IIC_Stop1(); return PT304D_ERR_I2C; }
        IIC_Stop1();
    }
    return PT304D_OK;
}

static PT304D_Status pt304d_7b_read(uint8_t sensor_idx, PT304D_Data *p_data)
{
    uint8_t data_buf[PT304D_7B_READ_BYTES] = {0};
    uint8_t i;
    PT304D_Status ret;

    if (sensor_idx == PT304D_SENSOR_1)
    {
        IIC_Start();
        IIC_Send_Byte((PT304D_7B_ADDR << 1) | IIC_RD);
        if (IIC_Wait_Ack(0) != 0)
        {
            IIC_Stop();
            p_data->err = PT304D_ERR_I2C;
            return PT304D_ERR_I2C;
        }
        for (i = 0; i < PT304D_7B_READ_BYTES; i++)
            data_buf[i] = IIC_Read_Byte(i < (PT304D_7B_READ_BYTES - 1) ? 1 : 0);
        IIC_Stop();
    }
    else
    {
        IIC_Start1();
        IIC_Send_Byte1((PT304D_7B_ADDR << 1) | IIC_RD);
        if (IIC_Wait_Ack1(0) != 0)
        {
            IIC_Stop1();
            p_data->err = PT304D_ERR_I2C;
            return PT304D_ERR_I2C;
        }
        for (i = 0; i < PT304D_7B_READ_BYTES; i++)
            data_buf[i] = IIC_Read_Byte1(i < (PT304D_7B_READ_BYTES - 1) ? 1 : 0);
        IIC_Stop1();
    }

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
    uint8_t buf[PT304D_4B_READ_BYTES];
    uint8_t i;

	trea_delay_ms(50);
    if (sensor_idx == PT304D_SENSOR_1)
    {
        IIC_Start();
        IIC_Send_Byte((PT304D_4B_ADDR << 1) | IIC_RD);
        if (IIC_Wait_Ack(0) != 0)
        {
            IIC_Stop();
            p_data->err = PT304D_ERR_I2C;
            return PT304D_ERR_I2C;
        }
//		//低功耗模式
//		trea_delay_ms(50);
//		IIC_Start();
//        IIC_Send_Byte((PT304D_4B_ADDR << 1) | IIC_RD);
//        if (IIC_Wait_Ack(0) != 0)
//        {
//            IIC_Stop();
//            p_data->err = PT304D_ERR_I2C;
//            return PT304D_ERR_I2C;
//        }
        for (i = 0; i < PT304D_4B_READ_BYTES; i++)
            buf[i] = IIC_Read_Byte(i < (PT304D_4B_READ_BYTES - 1) ? 1 : 0);
        IIC_Stop();
    }
    else
    {
        IIC_Start1();
        IIC_Send_Byte1((PT304D_4B_ADDR << 1) | IIC_RD);
        if (IIC_Wait_Ack1(0) != 0)
        {
            IIC_Stop1();
            p_data->err = PT304D_ERR_I2C;
            return PT304D_ERR_I2C;
        }
        for (i = 0; i < PT304D_4B_READ_BYTES; i++)
            buf[i] = IIC_Read_Byte1(i < (PT304D_4B_READ_BYTES - 1) ? 1 : 0);
        IIC_Stop1();
    }

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
    if (sensor_idx == PT304D_SENSOR_1)
        IIC_Init();
    else
        IIC_GPIO_Init1();
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

    pt304d_sw_iic_on();
    PT304D_Init(sensor_idx);

    ret = PT304D_SendMeasCmd(sensor_idx);
    if (ret != PT304D_OK) { p_data->err = ret; pt304d_sw_iic_off(); return ret; }

#ifdef PT304D_DRV_7BYTE
    if (!PT304D_IS_4BYTE(sensor_idx))
        trea_delay_ms(PT304D_7B_MEAS_DELAY);
#endif

    ret = PT304D_ReadMeasData(sensor_idx, p_data);

    pt304d_sw_iic_off();
    return ret;
}

// 一次性采集两路传感器，PD4 仅上/断电一次，效率更高   s2进气（中压）   s1出气（低压）
void PT304D_CollectAll(PT304D_Data *p_s1, PT304D_Data *p_s2)
{
    // Init I2C GPIO before power-on: open-drain HIGH (bus idle)
    // Prevents sensor seeing glitches from Analog-mode residue after deep sleep
    if(p_s1 != NULL) IIC_Init();
    if(p_s2 != NULL) IIC_GPIO_Init1();

    pt304d_sw_iic_on();

    // --- 传感器1: PC0/PC1 (与M24M01 共用总线) ---
    if (p_s1 != NULL)
    {
        IIC_Init();
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

    // --- 传感器2: PB6/PB7 (独立总线) ---
    if (p_s2 != NULL)
    {
        IIC_GPIO_Init1();
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

    pt304d_sw_iic_off();
}

PT304D_Status PT304D_SetI2CAddr(uint8_t sensor_idx, uint8_t new_7bit_addr)
{
    if (new_7bit_addr > 0x7F) return PT304D_ERR_DATA;

    if (sensor_idx == PT304D_SENSOR_1)
    {
        IIC_Start();
        IIC_Send_Byte((PT304D_I2C_ADDR_7BIT << 1) | IIC_WR);
        if (IIC_Wait_Ack(0) != 0) { IIC_Stop(); return PT304D_ERR_I2C; }
        IIC_Send_Byte(PT304D_ADDR_REG);
        if (IIC_Wait_Ack(0) != 0) { IIC_Stop(); return PT304D_ERR_I2C; }
        IIC_Send_Byte(new_7bit_addr);
        if (IIC_Wait_Ack(0) != 0) { IIC_Stop(); return PT304D_ERR_I2C; }
        IIC_Stop();
    }
    else
    {
        IIC_Start1();
        IIC_Send_Byte1((PT304D_I2C_ADDR_7BIT << 1) | IIC_WR);
        if (IIC_Wait_Ack1(0) != 0) { IIC_Stop1(); return PT304D_ERR_I2C; }
        IIC_Send_Byte1(PT304D_ADDR_REG);
        if (IIC_Wait_Ack1(0) != 0) { IIC_Stop1(); return PT304D_ERR_I2C; }
        IIC_Send_Byte1(new_7bit_addr);
        if (IIC_Wait_Ack1(0) != 0) { IIC_Stop1(); return PT304D_ERR_I2C; }
        IIC_Stop1();
    }
    return PT304D_OK;
}






