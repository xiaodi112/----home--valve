#ifndef PT_304_35__H
#define PT_304_35__H

#include "gd32l23x.h"
#include "i2c_eeprom.h"
#include "i2c_sensor.h"
#include "delay.h"
#include "stddef.h"

/************************** 双传感器索引定义 **************************/
#define PT304D_SENSOR_1    0   // 传感器1: PC0/PC1 总线 (与 AT24C16 共用), 使用 IIC_xxx()
#define PT304D_SENSOR_2    1   // 传感器2: PB6/PB7 总线 (独立),            使用 IIC_xxx1()

/************************** SW_IIC 统一电源开关 (PB5) **************************/
#define PT304D_SW_IIC_PORT   GPIOB
#define PT304D_SW_IIC_PIN    GPIO_PIN_5
#define PT304D_PWRUP_DELAY_MS  15     // PB5 上电后等待传感器稳定时间 (ms)

/************************** PT304D 协议核心配置（1:1匹配手册） **************************/
#define PT304D_I2C_ADDR_7BIT   0x00    // 传感器默认7位I2C地址（手册明确定义）
#define PT304D_MEAS_CMD        0xAA    // 测量触发指令（手册要求发送0xAA启动测量）
#define PT304D_READ_BYTES      7       // 测量应答数据长度：1字节状态+6字节数据（手册定义）
#define PT304D_MEAS_DELAY_MS   15      // 发送指令后等待时间（手册要求≥10ms）
#define PT304D_ADDR_REG        0x02    // 地址修改寄存器（手册定义，可修改7位I2C地址）
#define PT304D_PRESS_FS_35     35000.0f // 压力满量程(Pa)，根据实际传感器量程修改
#define PT304D_PRESS_FS_600    600000.0f // 压力满量程(Pa)，根据实际传感器量程修改
#define PT304D_24BIT_MAX       0xFFFFFF// 24位无符号数最大值（手册压力/温度原始数据上限）
#define PT304D_TEMP_MIN        -40.0f  // 温度量程下限（手册定义0对应-40℃）
#define PT304D_TEMP_MAX        125.0f  // 温度量程上限（手册定义0xFFFFFF对应125℃）

/************************** 状态字位定义（完全匹配手册表1：Bit7~Bit0） **************************/
// 手册明确状态字位序：Bit7(0)、Bit6(Powered?)、Bit5(Busy?)、Bit4~Bit3(Mode)、Bit2(MemErr)、Bit1(ConnErr)、Bit0(MathSat)
#define PT304D_STAT_BIT7       (1 << 7)  // 固定为0，手册定义
#define PT304D_STAT_POWERED    (1 << 6)  // Bit6：Powered? - 传感器供电状态（1=正常，0=异常）
#define PT304D_STAT_BUSY       (1 << 5)  // Bit5：Busy? - 命令执行状态（1=忙/未完成，0=完成/数据有效）【手册核心】
#define PT304D_STAT_MODE_BIT4  (1 << 4)  // Bit4：模式位高位 - 与Bit3组合为模式
#define PT304D_STAT_MODE_BIT3  (1 << 3)  // Bit3：模式位低位 - 与Bit4组合为模式【手册核心】
#define PT304D_STAT_MEM_ERR    (1 << 2)  // Bit2：Memory Error? - 内存错误（1=错误）
#define PT304D_STAT_CONN_ERR   (1 << 1)  // Bit1：Connection Check Fault? - 连接故障（1=故障）
#define PT304D_STAT_MATH_SAT   (1 << 0)  // Bit0：Math Saturation - 计算饱和（1=饱和）
// 模式位组合掩码与低功耗模式值（手册要求低功耗模式为0b10，即Bit4=1、Bit3=0）
#define PT304D_STAT_MODE_MASK  (PT304D_STAT_MODE_BIT4 | PT304D_STAT_MODE_BIT3)
#define PT304D_MODE_LOW_POWER  0b10     // 低功耗模式标准值（手册强制要求）

/************************** 驱动返回状态码（精准对应协议异常场景） **************************/
typedef enum {
    PT304D_OK          = 0,  // 操作成功，数据有效
    PT304D_ERR_I2C     = 1,  // I2C通信失败（无ACK、移位错误等）
    PT304D_ERR_BUSY    = 2,  // Bit5=1，传感器忙，命令未执行完，数据无效
    PT304D_ERR_MODE    = 3,  // Bit4~3≠0b10，非手册要求的低功耗模式
    PT304D_ERR_POWER   = 4,  // Bit6=0，传感器供电异常
    PT304D_ERR_NULLPTR = 5,  // 传入空指针，参数非法
    PT304D_ERR_DATA    = 6   // 数据解析异常（预留）
} PT304D_Status;

/************************** 传感器数据结构（包含原始/换算数据，便于调试/应用） **************************/
typedef struct {
    uint8_t  stat_raw;    // 状态字原始值（手册返回的第1字节，Bit7~Bit0）
    uint32_t press_raw;   // 压力24位原始数据（手册：SensorData<23:0>，高位在前）
    uint32_t temp_raw;    // 温度24位原始数据（手册：TempData<23:0>，高位在前）
    float    pressure;    // 换算后实际压力值（单位：Pa）
    float    temperature; // 换算后实际温度值（单位：℃）
    PT304D_Status err;    // 最新操作错误码，便于问题定位
} PT304D_Data;

/************************** 外部函数声明（分层设计，应用层直接调用） **************************/
// sensor_idx: PT304D_SENSOR_1 (PC0/PC1) 或 PT304D_SENSOR_2 (PB6/PB7)
PT304D_Status PT304D_Init(uint8_t sensor_idx);
PT304D_Status PT304D_SendMeasCmd(uint8_t sensor_idx);
PT304D_Status PT304D_ReadMeasData(uint8_t sensor_idx, PT304D_Data *p_data);
PT304D_Status PT304D_Collect(uint8_t sensor_idx, PT304D_Data *p_data);   // 含 PB5 上/断电
PT304D_Status PT304D_SetI2CAddr(uint8_t sensor_idx, uint8_t new_7bit_addr);

// 便捷接口：一次性采集两路传感器，统一管理 PB5 电源（省去重复上电）
// p_s1: PT304D_SENSOR_1 数据指针 (PC0/PC1)
// p_s2: PT304D_SENSOR_2 数据指针 (PB6/PB7)
// 任意指针传 NULL 则跳过对应传感器
void PT304D_CollectAll(PT304D_Data *p_s1, PT304D_Data *p_s2);

#endif



