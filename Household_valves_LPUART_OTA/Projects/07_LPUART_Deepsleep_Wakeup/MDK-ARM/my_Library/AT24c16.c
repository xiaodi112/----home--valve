#include "AT24c16.h"
#include "i2c_sensor.h"
#include "delay.h"
#include "usart.h"
#include "gd32l23x.h"
#include <string.h>

/* ===================== I2C 电子开关 PB5 (SW_IIC) 管理 =====================
 * PB5 = SW_IIC 电子开关 (TPS22918), 控制所有 I2C 设备电源
 * 默认 LOW 节能; 每次 EEPROM 读/写前自动 set + 10ms 上电稳定, 操作完 reset
 * 接口调用者无需关心 PB5 状态 (与传感器采集路径的 PB5 状态独立)
 */
static inline void at24c16_pwr_on(void)
{
    gpio_bit_set(GPIOB, GPIO_PIN_5);
    trea_delay_ms(10);   // EEPROM 上电稳定 (手册典型 ≤5ms, 留余量)
}

static inline void at24c16_pwr_off(void)
{
    gpio_bit_reset(GPIOB, GPIO_PIN_5);
}

/*
 * AT24C16 驱动（严格匹配规格书）
 *
 * 地址机制: 11位地址 = 高3位(P2P1P0)编入设备地址字节 + 低8位字地址
 *   设备地址字节: 1010 P2 P1 P0 R/W
 *   只发送1字节字地址（非双字节！）
 *
 * 页写规则: 每页16字节，页边界 = addr & 0xFFF0
 *   写入不可跨页，跨页时地址会回卷到页首
 *
 * 写周期: 每次写操作(字节写/页写)后需等待最多5ms
 */

/* ===================== ACK轮询等待写周期完成 ===================== */
uint8_t AT24C16_WaitReady(uint16_t addr, uint16_t timeout_ms)
{
    uint8_t dev_byte = AT24C16_DEV_ADDR(addr);
    uint16_t i;

    for(i = 0; i < timeout_ms; i++)
    {
        IIC_Start();
        IIC_Send_Byte(dev_byte);
        if(IIC_Wait_Ack(10) == 0)
        {
            IIC_Stop();
            return 0;
        }
        IIC_Stop();
        trea_delay_ms(1);
    }
    return 1;
}

/* ===================== 写单字节 ===================== */
uint8_t AT24C16_WriteByte(uint16_t addr, uint8_t data)
{
    uint8_t dev_byte  = AT24C16_DEV_ADDR(addr);
    uint8_t word_addr = AT24C16_WORD_ADDR(addr);
    uint8_t ret;

    if(addr >= AT24C16_TOTAL_SIZE) return 1;

    at24c16_pwr_on();

    IIC_Start();
    IIC_Send_Byte(dev_byte);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); at24c16_pwr_off(); return 2; }

    IIC_Send_Byte(word_addr);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); at24c16_pwr_off(); return 3; }

    IIC_Send_Byte(data);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); at24c16_pwr_off(); return 4; }

    IIC_Stop();

    ret = AT24C16_WaitReady(addr, AT24C16_WRITE_TIME + 1);
    at24c16_pwr_off();
    return ret;
}

/* ===================== 页写（不可跨页，调用者保证） ===================== */
uint8_t AT24C16_WritePage(uint16_t addr, const uint8_t *data, uint8_t len)
{
    uint8_t dev_byte  = AT24C16_DEV_ADDR(addr);
    uint8_t word_addr = AT24C16_WORD_ADDR(addr);
    uint8_t i;
    uint8_t ret;

    if(addr >= AT24C16_TOTAL_SIZE || len == 0 || len > AT24C16_PAGE_SIZE) return 1;

    at24c16_pwr_on();

    IIC_Start();
    IIC_Send_Byte(dev_byte);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); at24c16_pwr_off(); return 2; }

    IIC_Send_Byte(word_addr);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); at24c16_pwr_off(); return 3; }

    for(i = 0; i < len; i++)
    {
        IIC_Send_Byte(data[i]);
        if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); at24c16_pwr_off(); return 4; }
    }

    IIC_Stop();

    ret = AT24C16_WaitReady(addr, AT24C16_WRITE_TIME + 1);
    at24c16_pwr_off();
    return ret;
}

/* ===================== 任意长度写（自动分页） ===================== */
/* 注: WritePage 内部也会调 at24c16_pwr_on/off; 跨页多次写会重复起动 PD4 (10ms×N),
 *      但为保证 WritePage 独立可用且逻辑简洁, 不作合并优化; 配置保存不是频繁操作, 可接受 */
uint8_t AT24C16_Write(uint16_t addr, const uint8_t *data, uint16_t len)
{
    uint16_t offset = 0;
    uint8_t  ret;

    while(offset < len)
    {
        // 当前页剩余可写字节数
        uint8_t page_remain = AT24C16_PAGE_SIZE - ((addr + offset) % AT24C16_PAGE_SIZE);
        uint16_t left = len - offset;
        uint8_t  chunk = (left < page_remain) ? (uint8_t)left : page_remain;

        ret = AT24C16_WritePage(addr + offset, data + offset, chunk);
        if(ret != 0) return ret;

        offset += chunk;
    }
    return 0;
}

/* ===================== 随机读（任意地址、任意长度） ===================== */
uint8_t AT24C16_Read(uint16_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t dev_byte  = AT24C16_DEV_ADDR(addr);
    uint8_t word_addr = AT24C16_WORD_ADDR(addr);
    uint16_t i;

    if(addr >= AT24C16_TOTAL_SIZE || len == 0) return 1;

    at24c16_pwr_on();

    /* 第一步: Dummy Write — 设置读起始地址 */
    IIC_Start();
    IIC_Send_Byte(dev_byte | 0x00);          // 设备地址 + 写位
    if(IIC_Wait_Ack(1000) != 0) { IIC_Stop(); at24c16_pwr_off(); return 2; }

    IIC_Send_Byte(word_addr);                // 只发1字节字地址
    if(IIC_Wait_Ack(1000) != 0) { IIC_Stop(); at24c16_pwr_off(); return 3; }

    /* 第二步: 重复起始 + 读数据 */
    IIC_Start();
    IIC_Send_Byte(dev_byte | 0x01);          // 设备地址 + 读位
    if(IIC_Wait_Ack(1000) != 0) { IIC_Stop(); at24c16_pwr_off(); return 4; }

    for(i = 0; i < len; i++)
    {
        if(i == len - 1)
            buf[i] = IIC_Read_Byte(0);       // 最后1字节: NACK
        else
            buf[i] = IIC_Read_Byte(1);       // 其余: ACK（继续读）
    }

    IIC_Stop();
    at24c16_pwr_off();
    return 0;
}

/* ===================== 诊断测试 ===================== */
void AT24C16_Test(void)
{
    uint8_t ret, val;
    uint16_t test_addr;

    printf_debug("\r\n===== AT24C16 Diagnostic Start =====\r\n");

    // I2C初始化
    IIC_Init();

    // ---- 诊断A: SDA GPIO自检 ----
    printf_debug("[DiagA] SDA GPIO self-test (PB9):\r\n");
    IIC_SDA_H();
    IIC_Delay(); IIC_Delay(); IIC_Delay();
    printf_debug("  SDA=H -> read=%d (expect 1)\r\n", (int)IIC_SDA_READ());
    IIC_SDA_L();
    IIC_Delay(); IIC_Delay(); IIC_Delay();
    printf_debug("  SDA=L -> read=%d (expect 0)\r\n", (int)IIC_SDA_READ());
    IIC_SDA_H();
    IIC_SCL_H();

    // ---- 诊断B: I2C总线扫描 ----
    printf_debug("[DiagB] I2C bus scan (addr 0xA0~0xAE):\r\n");
    {
        uint8_t a;
        for(a = 0xA0; a <= 0xAE; a += 2)
        {
            IIC_Start();
            IIC_Send_Byte(a);
            ret = IIC_Wait_Ack(100);
            IIC_Stop();
            printf_debug("  0x%02X -> %s\r\n", a, (ret == 0) ? "ACK" : "NACK");
            IIC_Delay(); IIC_Delay();
        }
    }

    // ---- 诊断C: 读未写过的高地址（空片应为0xFF）----
    test_addr = 0x07F0;
    ret = AT24C16_Read(test_addr, &val, 1);
    printf_debug("[DiagC] Read virgin addr=0x%04X: ret=%d val=0x%02X (expect 0xFF)\r\n",
                 test_addr, ret, val);

    // ---- 诊断D: 单字节写+读详细步骤 ----
    test_addr = 0x0050;
    printf_debug("[DiagD] WriteByte addr=0x%04X data=0xA5\r\n", test_addr);
    ret = AT24C16_WriteByte(test_addr, 0xA5);
    printf_debug("  WriteByte ret=%d\r\n", ret);
    trea_delay_ms(10);  // 额外等待确保写完成

    val = 0x00;
    ret = AT24C16_Read(test_addr, &val, 1);
    printf_debug("  Read ret=%d val=0x%02X\r\n", ret, val);
    if(val == 0xA5)
        printf_debug("  -> PASS\r\n");
    else
        printf_debug("  -> FAIL (expect 0xA5, got 0x%02X)\r\n", val);

    // ---- 诊断E: 写0x55再读（不同数据排除巧合） ----
    printf_debug("[DiagE] WriteByte addr=0x%04X data=0x55\r\n", test_addr);
    ret = AT24C16_WriteByte(test_addr, 0x55);
    printf_debug("  WriteByte ret=%d\r\n", ret);
    trea_delay_ms(10);

    val = 0x00;
    ret = AT24C16_Read(test_addr, &val, 1);
    printf_debug("  Read ret=%d val=0x%02X\r\n", ret, val);
    if(val == 0x55)
        printf_debug("  -> PASS\r\n");
    else
        printf_debug("  -> FAIL (expect 0x55, got 0x%02X)\r\n", val);

    printf_debug("===== AT24C16 Diagnostic End =====\r\n\r\n");
}



