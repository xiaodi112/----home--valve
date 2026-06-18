#include "i2c_sensor.h"
#include "delay.h"

void IIC_Delay(void)
{
    volatile uint16_t i;
    for(i = 0; i < 100; i++);
}

void IIC_Init(void)
{
    RCC_IIC_ENABLE;
    gpio_mode_set(GPIO_PORT_IIC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, IIC_SCL_PIN | IIC_SDA_PIN);
    gpio_output_options_set(GPIO_PORT_IIC, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, IIC_SCL_PIN | IIC_SDA_PIN);
    IIC_SCL_H();
    IIC_SDA_H();
    IIC_Delay();
    IIC_Stop();
}

void IIC_Start(void)
{
    IIC_SDA_H();
    IIC_SCL_H();
    IIC_Delay();
    IIC_SDA_L();
    IIC_Delay();
    IIC_SCL_L();
    IIC_Delay();
}

void IIC_Stop(void)
{
    IIC_SCL_L();
    IIC_SDA_L();
    IIC_Delay();
    IIC_SCL_H();
    IIC_Delay();
    IIC_SDA_H();
    IIC_Delay();
}

void IIC_Send_Byte(uint8_t txd)
{
    int8_t i;

    for(i = 7; i >= 0; i--)
    {
        IIC_SCL_L();
        if(txd & BIT(i)) IIC_SDA_H();
        else IIC_SDA_L();
        IIC_Delay();
        IIC_SCL_H();
        IIC_Delay();
    }
    IIC_SCL_L();
    IIC_SDA_H();
    IIC_Delay();
}

uint8_t IIC_Wait_Ack(int16_t timeout)
{
    (void)timeout;

    IIC_SDA_H();
    IIC_Delay();
    IIC_SCL_H();
    IIC_Delay();

    if(IIC_SDA_READ() != 0)
    {
        IIC_SCL_L();
        return 1;
    }

    IIC_SCL_L();
    IIC_Delay();
    return 0;
}

uint8_t IIC_Read_Byte(uint8_t ack)
{
    int8_t i;
    uint8_t rxd = 0;

    IIC_SDA_H();

    for(i = 7; i >= 0; i--)
    {
        IIC_SCL_L();
        IIC_Delay();
        IIC_SCL_H();
        IIC_Delay();
        if(IIC_SDA_READ()) rxd |= BIT(i);
    }
    IIC_SCL_L();
    IIC_Delay();

    if(ack)
        IIC_SDA_L();
    else
        IIC_SDA_H();

    IIC_Delay();
    IIC_SCL_H();
    IIC_Delay();
    IIC_SCL_L();
    IIC_SDA_H();
    IIC_Delay();

    return rxd;
}
