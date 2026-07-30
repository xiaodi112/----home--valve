#include "PT_304_35.h"
#include "i2c_eeprom.h"
#include "i2c_sensor.h"

/************************** 静态工具函数声明（驱动内部调用，隐藏实现细节） **************************/
static PT304D_Status PT304D_CheckStat(uint8_t stat);
static float PT304D_ConvPress(uint32_t raw, float fs);
static float PT304D_ConvTemp(uint32_t raw);

/************************** 静态工具函数实现（1:1匹配手册规则�?**************************/
/**
 * @brief  校验传感器返回的状态字，判断数据是否有效（完全匹配手册�?�?
 * @param  stat: 传感器返回的原始状态字（第1字节，Bit7~Bit0�?
 * @retval PT304D_Status: 校验结果，仅返回OK时数据有�?
 */
static PT304D_Status PT304D_CheckStat(uint8_t stat)
{
    // 1. 优先检查Bit5（Busy?）：1=忙，命令未执行完，后续数据完全无效（手册核心要求�?
    if ((stat & PT304D_STAT_BUSY) != 0)
    {
        return PT304D_ERR_BUSY;
    }

    // 2. 检查Bit4~3（Mode）：必须�?b10（低功耗模式），否则模式异常（手册核心要求�?
    uint8_t mode_val = (stat & PT304D_STAT_MODE_MASK) >> 3; // 右移3位，将Bit4~3移至最�?�?
    if (mode_val != PT304D_MODE_LOW_POWER)
    {
        return PT304D_ERR_MODE;
    }

    // 3. 检查Bit6（Powered?）：0=供电异常，传感器未正常上电（手册定义�?
    if ((stat & PT304D_STAT_POWERED) == 0)
    {
        return PT304D_ERR_POWER;
    }

    // 所有手册要求的核心位校验通过，数据有�?
    return PT304D_OK;
}

/**
 * @brief  �?4位原始压力数据转换为实际压力值（严格遵循手册线性换算公式）
 * @param  raw: 压力24位无符号原始数据�?~0xFFFFFF，手册定义）
 * @retval float: 换算后压力值，单位Pa
 * @note   手册公式：压�?= (原始�?÷ 最大�? - 有效量程下限占比] ÷ 
			(有效量程上限占比 - 有效量程下限占比) × 传感器满量程
 */
static float PT304D_ConvPress(uint32_t raw, float fs)
{
    return ((raw / (float)PT304D_24BIT_MAX) - 0.1f) / (0.9f - 0.1f) * fs;
}

/**
 * @brief  �?4位原始温度数据转换为实际温度值（严格遵循手册线性换算公式）
 * @param  raw: 温度24位无符号原始数据�?~0xFFFFFF，手册定义）
 * @retval float: 换算后温度值，单位�?
 * @note   手册公式：温�?= -40 + (原始�?0xFFFFFF) * (125 - (-40)) = -40 + 原始�?0xFFFFFF * 165
 */
static float PT304D_ConvTemp(uint32_t raw)
{
    return PT304D_TEMP_MIN + (raw / (float)PT304D_24BIT_MAX) * (PT304D_TEMP_MAX - PT304D_TEMP_MIN);
}

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

/************************** 外部函数实现（双路分派，严格匹配手册通信时序�?**************************/
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
    if (sensor_idx == PT304D_SENSOR_1)
    {
        IIC_Start();
        IIC_Send_Byte((PT304D_I2C_ADDR_7BIT << 1) | IIC_WR);
        if (IIC_Wait_Ack(0) != 0) { IIC_Stop(); return PT304D_ERR_I2C; }
        IIC_Send_Byte(PT304D_MEAS_CMD);
        if (IIC_Wait_Ack(0) != 0) { IIC_Stop(); return PT304D_ERR_I2C; }
        IIC_Stop();
    }
    else
    {
        IIC_Start1();
        IIC_Send_Byte1((PT304D_I2C_ADDR_7BIT << 1) | IIC_WR);
        if (IIC_Wait_Ack1() != 0) { IIC_Stop1(); return PT304D_ERR_I2C; }
        IIC_Send_Byte1(PT304D_MEAS_CMD);
        if (IIC_Wait_Ack1() != 0) { IIC_Stop1(); return PT304D_ERR_I2C; }
        IIC_Stop1();
    }
    return PT304D_OK;
}

PT304D_Status PT304D_ReadMeasData(uint8_t sensor_idx, PT304D_Data *p_data)
{
    uint8_t data_buf[PT304D_READ_BYTES] = {0};
    uint8_t i;
    PT304D_Status ret;

    if (p_data == NULL) return PT304D_ERR_NULLPTR;

    if (sensor_idx == PT304D_SENSOR_1)
    {
        IIC_Start();
        IIC_Send_Byte((PT304D_I2C_ADDR_7BIT << 1) | IIC_RD);
        if (IIC_Wait_Ack(0) != 0)
        {
            IIC_Stop();
            p_data->err = PT304D_ERR_I2C;
            return PT304D_ERR_I2C;
        }
        for (i = 0; i < PT304D_READ_BYTES; i++)
            data_buf[i] = IIC_Read_Byte(i < (PT304D_READ_BYTES - 1) ? 1 : 0);
        IIC_Stop();
    }
    else
    {
        IIC_Start1();
        IIC_Send_Byte1((PT304D_I2C_ADDR_7BIT << 1) | IIC_RD);
        if (IIC_Wait_Ack1() != 0)
        {
            IIC_Stop1();
            p_data->err = PT304D_ERR_I2C;
            return PT304D_ERR_I2C;
        }
        for (i = 0; i < PT304D_READ_BYTES; i++)
            data_buf[i] = IIC_Read_Byte1(i < (PT304D_READ_BYTES - 1) ? 1 : 0);
        IIC_Stop1();
    }

    p_data->stat_raw = data_buf[0];
    ret = PT304D_CheckStat(data_buf[0]);
    if (ret != PT304D_OK) { p_data->err = ret; return ret; }

    p_data->press_raw = ((uint32_t)data_buf[1] << 16) | ((uint32_t)data_buf[2] << 8) | data_buf[3];
    p_data->temp_raw  = ((uint32_t)data_buf[4] << 16) | ((uint32_t)data_buf[5] << 8) | data_buf[6];
    {
        float fs = (sensor_idx == PT304D_SENSOR_1) ? PT304D_PRESS_FS_35 : PT304D_PRESS_FS_600;
        p_data->pressure = PT304D_ConvPress(p_data->press_raw, fs);
    }
    p_data->temperature = PT304D_ConvTemp(p_data->temp_raw);
    p_data->err = PT304D_OK;
    return PT304D_OK;
}

PT304D_Status PT304D_Collect(uint8_t sensor_idx, PT304D_Data *p_data)
{
    PT304D_Status ret;

    if (p_data == NULL) return PT304D_ERR_NULLPTR;

    pt304d_sw_iic_on();
    PT304D_Init(sensor_idx);

    ret = PT304D_SendMeasCmd(sensor_idx);
    if (ret != PT304D_OK) { p_data->err = ret; pt304d_sw_iic_off(); return ret; }

    trea_delay_ms(PT304D_MEAS_DELAY_MS);
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

    // --- 传感�?: PC0/PC1 (�?M24M01 共用总线) ---
    if (p_s1 != NULL)
    {
        IIC_Init();
        if (PT304D_SendMeasCmd(PT304D_SENSOR_1) == PT304D_OK)
        {
            trea_delay_ms(PT304D_MEAS_DELAY_MS);
            PT304D_ReadMeasData(PT304D_SENSOR_1, p_s1);
        }
        else
        {
            p_s1->err = PT304D_ERR_I2C;
        }
    }

    // --- 传感�?: PB6/PB7 (独立总线) ---
    if (p_s2 != NULL)
    {
        IIC_GPIO_Init1();
        if (PT304D_SendMeasCmd(PT304D_SENSOR_2) == PT304D_OK)
        {
            trea_delay_ms(PT304D_MEAS_DELAY_MS);
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
        if (IIC_Wait_Ack1() != 0) { IIC_Stop1(); return PT304D_ERR_I2C; }
        IIC_Send_Byte1(PT304D_ADDR_REG);
        if (IIC_Wait_Ack1() != 0) { IIC_Stop1(); return PT304D_ERR_I2C; }
        IIC_Send_Byte1(new_7bit_addr);
        if (IIC_Wait_Ack1() != 0) { IIC_Stop1(); return PT304D_ERR_I2C; }
        IIC_Stop1();
    }
    return PT304D_OK;
}










