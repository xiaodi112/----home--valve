#ifndef I2C_EEPROM_H
#define I2C_EEPROM_H

#include "gd32l23x.h"  // GD32L233标准库头文件

#ifndef IIC_WR
#define IIC_WR    0     /* 写操作bit */
#define IIC_RD    1     /* 读操作bit */
#endif

void IIC_Start1(void);
void IIC_Stop1(void);
void IIC_Send_Byte1(uint8_t _ucByte);
uint8_t IIC_Read_Byte1(uint8_t ack);
uint8_t IIC_Wait_Ack1(uint16_t timeout);
void IIC_Ack1(void);
void IIC_NAck1(void);
uint8_t IIC_CheckDevice1(uint8_t _Address);
void IIC_GPIO_Init1(void);

#endif /* I2C_EEPROM_H */
