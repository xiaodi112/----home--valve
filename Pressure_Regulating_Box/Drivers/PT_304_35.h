#ifndef PT_304_35__H
#define PT_304_35__H

#include "gd32l23x.h"
#include "i2c_eeprom.h"
#include "i2c_sensor.h"
#include "delay.h"
#include "stddef.h"

/*****************************************************************************
 * 驱动协议选择（编译宏开关）
 * 使用方法: 取消注释启用对应驱动, 注释则禁用
 * - 只开一个: 两路传感器都用同一协议
 * - 两个都开: 每路独立选择协议 (由 PT304D_S1_DRIVER / PT304D_S2_DRIVER 绑定)
 *****************************************************************************/
#define PT304D_DRV_7BYTE         // 7字节协议: 发0xAA指令+读7字节 (addr=0x00, 24bit)
//#define PT304D_DRV_4BYTE       // 4字节协议: 直接读4字节       (addr=0x28, 14bit)

/*****************************************************************************
 * 每路传感器驱动绑定（仅两个驱动都启用时有效）
 * 值: 7 = 使用7字节协议, 4 = 使用4字节协议
 *****************************************************************************/
#define PT304D_S1_DRIVER   7     // S1(出气端, PC0/PC1) 使用的驱动协议
#define PT304D_S2_DRIVER   7     // S2(进气端, PB6/PB7) 使用的驱动协议

/*****************************************************************************
 * 每路传感器量程配置 (单位: Pa)
 * 修改此处即可切换传感器量程, 无需改动其它代码
 *****************************************************************************/
#define PT304D_S1_FS       35000.0f    // S1 满量程 (Pa): 35kPa
//#define PT304D_S2_FS       600000.0f   // S2 满量程 (Pa): 600kPa
#define PT304D_S2_FS     35000.0f    // S2 满量程 (Pa): 35kPa (立管阀单路使用)

/************************** 双传感器索引定义 **************************/
#define PT304D_SENSOR_1    0   // 传感器1: PC0/PC1 总线 (与M24M01 共用), 使用 IIC_xxx()   出气端
#define PT304D_SENSOR_2    1   // 传感器2: PB6/PB7 总线 (独立),            使用 IIC_xxx1() 进气端（立管阀单路使用）

/************************** SW_IIC 统一电源开关(PD4) **************************/
#define PT304D_SW_IIC_PORT   GPIOD
#define PT304D_SW_IIC_PIN    GPIO_PIN_4
#define PT304D_PWRUP_DELAY_MS  150     // PD4 上电后等待传感器稳定时间 (ms)

/************************** 7字节协议参数 (0xAA指令型) **************************/
#ifdef PT304D_DRV_7BYTE
#define PT304D_7B_ADDR         0x00    // 7bit I2C地址
#define PT304D_7B_MEAS_CMD     0xAA    // 测量触发指令
#define PT304D_7B_READ_BYTES   7       // 应答数据长度 (1状态+3压力+3温度)
#define PT304D_7B_MEAS_DELAY   15      // 指令后等待时间 (ms, 手册≥10ms)
#define PT304D_7B_ADDR_REG     0x02    // 地址修改寄存器
#define PT304D_7B_24BIT_MAX    0xFFFFFF// 24位原始数据上限
#define PT304D_7B_TEMP_MIN     -40.0f  // 温度下限 ℃
#define PT304D_7B_TEMP_MAX     125.0f  // 温度上限 ℃
#endif

/************************** 4字节协议参数 (直读型) **************************/
#ifdef PT304D_DRV_4BYTE
#define PT304D_4B_ADDR         0x28    // 7bit I2C地址
#define PT304D_4B_READ_BYTES   4       // 应答数据长度 (状态+压力14bit+温度11bit)
#define PT304D_4B_ZERO_ADC     1638    // 压力零点ADC (10%)
#define PT304D_4B_SPAN_ADC     13110   // 压力有效跨度 (90%-10% = 14746-1638)
#define PT304D_4B_TEMP_SCALE   2047    // 温度11bit满量程
#endif

/************************** 兼容旧宏名 (保证其他文件不需修改) **************************/
#define PT304D_I2C_ADDR_7BIT   0x00
#define PT304D_MEAS_CMD        0xAA
#define PT304D_READ_BYTES      7
#define PT304D_MEAS_DELAY_MS   15
#define PT304D_ADDR_REG        0x02
#define PT304D_PRESS_FS_35     35000.0f
#define PT304D_PRESS_FS_600    600000.0f
#define PT304D_24BIT_MAX       0xFFFFFF
#define PT304D_TEMP_MIN        -40.0f
#define PT304D_TEMP_MAX        125.0f

/************************** 驱动分派辅助宏 (内部使用) **************************/
#if defined(PT304D_DRV_7BYTE) && defined(PT304D_DRV_4BYTE)
  /* 双驱动模式: 按通道绑定分派 */
  #define PT304D_IS_4BYTE(idx) \
      (((idx) == PT304D_SENSOR_1) ? (PT304D_S1_DRIVER == 4) : (PT304D_S2_DRIVER == 4))
#elif defined(PT304D_DRV_4BYTE)
  /* 仅4字节模式 */
  #define PT304D_IS_4BYTE(idx)  1
#else
  /* 仅7字节模式 (默认) */
  #define PT304D_IS_4BYTE(idx)  0
#endif

/************************** 状态字位定义（7字节协议, 完全匹配手册）：Bit7~Bit0 **************************/
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
    PT304D_ERR_MODE    = 3,  // Bit4~3非0b10，非手册要求的低功耗模式
    PT304D_ERR_POWER   = 4,  // Bit6=0，传感器供电异常
    PT304D_ERR_NULLPTR = 5,  // 传入空指针，参数非法
    PT304D_ERR_DATA    = 6   // 数据解析异常（预留）
} PT304D_Status;

/************************** 传感器数据结构（包含原始/换算数据，便于调试/应用） **************************/
typedef struct {
    uint8_t  stat_raw;    // 状态字原始值（手册返回的第1字节，Bit7~Bit0）
    uint32_t press_raw;   // 压力原始数据（7byte:24bit, 4byte:14bit）
    uint32_t temp_raw;    // 温度原始数据（7byte:24bit, 4byte:11bit）
    float    pressure;    // 换算后实际压力值（单位：Pa）
    float    temperature; // 换算后实际温度值（单位：℃）
    PT304D_Status err;    // 最新操作错误码，便于问题定位
} PT304D_Data;

/************************** 外部函数声明（分层设计，应用层直接调用） **************************/
// sensor_idx: PT304D_SENSOR_1 (PC0/PC1) 或 PT304D_SENSOR_2 (PB6/PB7)
PT304D_Status PT304D_Init(uint8_t sensor_idx);
PT304D_Status PT304D_SendMeasCmd(uint8_t sensor_idx);
PT304D_Status PT304D_ReadMeasData(uint8_t sensor_idx, PT304D_Data *p_data);
PT304D_Status PT304D_Collect(uint8_t sensor_idx, PT304D_Data *p_data);   // 含PD4 上/断电
PT304D_Status PT304D_SetI2CAddr(uint8_t sensor_idx, uint8_t new_7bit_addr);

// 便捷接口：一次性采集两路传感器，统一管理 PD4 电源（省去重复上电）
// p_s1: PT304D_SENSOR_1 数据指针 (PC0/PC1)
// p_s2: PT304D_SENSOR_2 数据指针 (PB6/PB7)
// 任意指针为NULL 则跳过对应传感器
void PT304D_CollectAll(PT304D_Data *p_s1, PT304D_Data *p_s2);

#endif
