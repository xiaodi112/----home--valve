#include "i2c_eeprom.h"
#include "gd32l23x.h"
#include "delay.h"

/* 配置IIC连接的GPIO引脚，用户仅需修改此处4行即可切换SCL/SDA引脚 */
#define GPIO_PORT_IIC     GPIOB                       /* GPIO端口 */
#define RCC_IIC_ENABLE    rcu_periph_clock_enable(RCU_GPIOB)  /* GPIO时钟使能 */
#define IIC_SCL_PIN       GPIO_PIN_6                  /* 连接SCL时钟线的GPIO */
#define IIC_SDA_PIN       GPIO_PIN_7                  /* 连接SDA数据线的GPIO */

/* 读写SCL和SDA的宏，优先使用GPIO库函数实现（兼容GD32标准库） */
#if 1   /* 方式1：使用GPIO库函数操作IO */
#define IIC_SCL_1()  gpio_bit_set(GPIO_PORT_IIC, IIC_SCL_PIN)        /* SCL = 1 */
#define IIC_SCL_0()  gpio_bit_reset(GPIO_PORT_IIC, IIC_SCL_PIN)      /* SCL = 0 */

#define IIC_SDA_1()  gpio_bit_set(GPIO_PORT_IIC, IIC_SDA_PIN)        /* SDA = 1 */
#define IIC_SDA_0()  gpio_bit_reset(GPIO_PORT_IIC, IIC_SDA_PIN)      /* SDA = 0 */

#define IIC_SDA_READ()  gpio_input_bit_get(GPIO_PORT_IIC, IIC_SDA_PIN) /* 读SDA引脚状态 */
#else   /* 方式2：直接操作寄存器（效率更高，可选） */
#define IIC_SCL_1()  GPIO_PORT_IIC->BSC = IIC_SCL_PIN                 /* SCL = 1 */
#define IIC_SCL_0()  GPIO_PORT_IIC->BCC = IIC_SCL_PIN                 /* SCL = 0 */

#define IIC_SDA_1()  GPIO_PORT_IIC->BSC = IIC_SDA_PIN                 /* SDA = 1 */
#define IIC_SDA_0()  GPIO_PORT_IIC->BCC = IIC_SDA_PIN                 /* SDA = 0 */

#define IIC_SDA_READ()  ((GPIO_PORT_IIC->ISTAT & IIC_SDA_PIN) != 0)   /* 读SDA引脚状态 */
#endif

/* IIC读写方向定义（增强代码可读性） */
#define IIC_WR    0   /* 写操作 */
#define IIC_RD    1   /* 读操作 */

static void IIC_Delay1(void)
{
	volatile uint8_t i;
    for(i = 0; i < 150; i++);
//   trea_delay_ms(3);
}

void IIC_Start1(void)
{
    IIC_SDA_1();
    IIC_SCL_1();
    IIC_Delay1();
    IIC_SDA_0();
    IIC_Delay1();
    IIC_SCL_0();
    IIC_Delay1();
}

void IIC_Stop1(void)
{
    
	IIC_SCL_0();
	IIC_SDA_0();
	IIC_Delay1();
    IIC_SCL_1();
    IIC_Delay1();
    IIC_SDA_1();
    IIC_Delay1();
}

void IIC_Send_Byte1(uint8_t _ucByte)
{
    int8_t i;

    for (i = 7; i >= 0; i--)
    {
//        if (_ucByte & 0x80)
//        {
//            IIC_SDA_1();
//        }
//        else
//        {
//            IIC_SDA_0();
//        }
		IIC_SCL_0();
		if(_ucByte & BIT(i)) IIC_SDA_1();
		else IIC_SDA_0();
        IIC_Delay1();
        IIC_SCL_1();
        IIC_Delay1();
	}
        IIC_SCL_0();
        IIC_SDA_1();
        IIC_Delay1();
}

uint8_t IIC_Read_Byte1(uint8_t ack)
{
    int8_t i;
    uint8_t value = 0;

    IIC_SDA_1();
    for (i = 7; i >= 0; i--)
    {
        IIC_SCL_0();
        IIC_Delay1();
        IIC_SCL_1();
        IIC_Delay1();
        if(IIC_SDA_READ()) value |= BIT(i);
    }

    // 第9个时钟: 发送ACK/NACK
    IIC_SCL_0();
    if(ack)
        IIC_SDA_0();    // ACK: SDA=0
    else
        IIC_SDA_1();    // NACK: SDA=1
    IIC_Delay1();
    IIC_SCL_1();        // 从机在此上升沿采样ACK
    IIC_Delay1();
    IIC_SCL_0();
    IIC_SDA_1();        // 释放SDA
    IIC_Delay1();

    return value;
}

uint8_t IIC_Wait_Ack1(uint16_t timeout)
{
    uint16_t wait = (timeout > 0) ? (uint16_t)timeout : 80;

    IIC_SDA_1();
    IIC_Delay1();
    IIC_SCL_1();
    IIC_Delay1();

    while(IIC_SDA_READ() != 0 && wait--)
    {
        IIC_Delay1();
    }

    if(IIC_SDA_READ() != 0)
    {
		IIC_SCL_1();
        return  1;
    }

    IIC_SCL_0();
    IIC_Delay1();
    return 0;
}

void IIC_Ack1(void)
{
    IIC_SDA_0();
    IIC_Delay1();
    IIC_SCL_1();
    IIC_Delay1();
    IIC_SCL_0();
    IIC_Delay1();
    IIC_SDA_1();
}

void IIC_NAck1(void)
{
    IIC_SDA_1();
    IIC_Delay1();
    IIC_SCL_1();
    IIC_Delay1();
    IIC_SCL_0();
    IIC_Delay1();
}

void IIC_GPIO_Init1(void)
{
    uint8_t i;
    RCC_IIC_ENABLE;
    gpio_mode_set(GPIO_PORT_IIC, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, IIC_SCL_PIN | IIC_SDA_PIN);
    gpio_output_options_set(GPIO_PORT_IIC, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, IIC_SCL_PIN | IIC_SDA_PIN);
    IIC_SCL_1();
    IIC_SDA_1();
    IIC_Delay1();

    /* 总线死锁恢复: 若 SDA 被从机拉低, 发送9个SCL脉冲释放总线 */
    if(IIC_SDA_READ() == 0)
    {
        for(i = 0; i < 9; i++)
        {
            IIC_SCL_0();
            IIC_Delay1();
            IIC_SCL_1();
            IIC_Delay1();
            if(IIC_SDA_READ() != 0) break;
        }
    }

    IIC_Stop1();
}

uint8_t IIC_CheckDevice1(uint8_t _Address)
{
    uint8_t ucAck = 1;

    IIC_GPIO_Init1();
    IIC_Start1();
    IIC_Send_Byte1((_Address << 1) | IIC_WR);
    ucAck = IIC_Wait_Ack1(0);
    IIC_Stop1();

    return ucAck;
}
