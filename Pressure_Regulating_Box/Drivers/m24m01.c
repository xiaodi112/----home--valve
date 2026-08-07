#include "m24m01.h"
#include "i2c_sensor.h"
#include "delay.h"
#include "usart.h"
#include "gd32l23x.h"
#include <string.h>

/* ===================== I2C 电子开�?PD4 (SW_IIC) 管理 =====================
 * PD4 = SW_IIC 电子开�?(TPS22918), 控制所�?I2C 设备电源
 * 默认 LOW 节能; 每次 EEPROM �?写前自动 set + 20ms 上电稳定, 操作�?reset
 * 接口调用者无需关心 PD4 状�?(与传感器采集路径�?PD4 状态独�?
 */
static uint8_t s_pwr_init_done = 0;

static inline void m24m01_pwr_on(void)
{
    rcu_periph_clock_enable(RCU_GPIOD);
    gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_4);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_4);

    if(!s_pwr_init_done) {
        s_pwr_init_done = 1;
        gpio_bit_reset(GPIOD, GPIO_PIN_4);
        trea_delay_ms(50);
    }

    gpio_bit_set(GPIOD, GPIO_PIN_4);
    trea_delay_ms(20);   // EEPROM 上电稳定 (手册典型 �?ms, 留余�?
    IIC_Init();          // 重新配置 PC0/PC1 GPIO + 总线死锁恢复 (与传感器共用总线)
}

static inline void m24m01_pwr_off(void)
{
    gpio_bit_reset(GPIOD, GPIO_PIN_4);
}

/*
 * M24M01-RMN6P 驱动（严格匹配规格书�?
 *
 * 地址机制: 17位地址 = A16编入设备地址字节bit1 + 2字节字地址(A15~A0)
 *   设备地址字节: 1010 E2 E1 A16 R/W  (E2=E1=0)
 *   发�?字节字地址: 先高(A15~A8) 再低(A7~A0)
 *
 * 页写规则: 每页256字节，页边界 = addr & 0x1FF00
 *   写入不可跨页，跨页时地址会回卷到页首
 *
 * 写周�? 每次写操�?字节�?页写)后需等待最�?ms
 */

/* ===================== ACK轮询等待写周期完�?===================== */
uint8_t M24M01_WaitReady(uint32_t addr, uint16_t timeout_ms)
{
    uint8_t dev_byte = M24M01_DEV_ADDR(addr);
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
uint8_t M24M01_WriteByte(uint32_t addr, uint8_t data)
{
    uint8_t dev_byte = M24M01_DEV_ADDR(addr);
    uint8_t addr_hi  = M24M01_ADDR_HI(addr);
    uint8_t addr_lo  = M24M01_ADDR_LO(addr);
    uint8_t ret;

    if(addr >= M24M01_TOTAL_SIZE) return 1;

    m24m01_pwr_on();

    IIC_Start();
    IIC_Send_Byte(dev_byte);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); m24m01_pwr_off(); return 2; }

    IIC_Send_Byte(addr_hi);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); m24m01_pwr_off(); return 3; }

    IIC_Send_Byte(addr_lo);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); m24m01_pwr_off(); return 4; }

    IIC_Send_Byte(data);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); m24m01_pwr_off(); return 5; }

    IIC_Stop();

    ret = M24M01_WaitReady(addr, M24M01_WRITE_TIME + 1);
    m24m01_pwr_off();
    return ret;
}

/* ===================== 页写（不可跨页，调用者保证） ===================== */
uint8_t M24M01_WritePage(uint32_t addr, const uint8_t *data, uint16_t len)
{
    uint8_t dev_byte = M24M01_DEV_ADDR(addr);
    uint8_t addr_hi  = M24M01_ADDR_HI(addr);
    uint8_t addr_lo  = M24M01_ADDR_LO(addr);
    uint16_t i;
    uint8_t ret;

    if(addr >= M24M01_TOTAL_SIZE || len == 0 || len > M24M01_PAGE_SIZE) return 1;

    m24m01_pwr_on();

    IIC_Start();
    IIC_Send_Byte(dev_byte);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); m24m01_pwr_off(); return 2; }

    IIC_Send_Byte(addr_hi);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); m24m01_pwr_off(); return 3; }

    IIC_Send_Byte(addr_lo);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); m24m01_pwr_off(); return 4; }

    for(i = 0; i < len; i++)
    {
        IIC_Send_Byte(data[i]);
        if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); m24m01_pwr_off(); return 5; }
    }

    IIC_Stop();

    ret = M24M01_WaitReady(addr, M24M01_WRITE_TIME + 1);
    m24m01_pwr_off();
    return ret;
}

/* ===================== 任意长度写（自动分页�?===================== */
uint8_t M24M01_Write(uint32_t addr, const uint8_t *data, uint16_t len)
{
    uint16_t offset = 0;
    uint8_t  ret;

    while(offset < len)
    {
        // 当前页剩余可写字节数 (页大�?56)
        uint16_t page_remain = M24M01_PAGE_SIZE - ((addr + offset) % M24M01_PAGE_SIZE);
        uint16_t left = len - offset;
        uint16_t chunk = (left < page_remain) ? left : page_remain;

        ret = M24M01_WritePage(addr + offset, data + offset, chunk);
        if(ret != 0) return ret;

        offset += chunk;
    }
    return 0;
}

/* ===================== 随机读（任意地址、任意长度） ===================== */
uint8_t M24M01_Read(uint32_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t dev_byte = M24M01_DEV_ADDR(addr);
    uint8_t addr_hi  = M24M01_ADDR_HI(addr);
    uint8_t addr_lo  = M24M01_ADDR_LO(addr);
    uint16_t i;

    if(addr >= M24M01_TOTAL_SIZE || len == 0) return 1;

    m24m01_pwr_on();

    /* 第一�? Dummy Write �?设置读起始地址 (2字节) */
    IIC_Start();
    IIC_Send_Byte(dev_byte | 0x00);          // 设备地址 + 写位
    if(IIC_Wait_Ack(1000) != 0) { IIC_Stop(); m24m01_pwr_off(); return 2; }

    IIC_Send_Byte(addr_hi);                  // 字地址高字�?A15~A8
    if(IIC_Wait_Ack(1000) != 0) { IIC_Stop(); m24m01_pwr_off(); return 3; }

    IIC_Send_Byte(addr_lo);                  // 字地址低字�?A7~A0
    if(IIC_Wait_Ack(1000) != 0) { IIC_Stop(); m24m01_pwr_off(); return 4; }

    /* 第二�? 重复起始 + 读数�?*/
    IIC_Start();
    IIC_Send_Byte(dev_byte | 0x01);          // 设备地址 + 读位
    if(IIC_Wait_Ack(1000) != 0) { IIC_Stop(); m24m01_pwr_off(); return 5; }

    for(i = 0; i < len; i++)
    {
        if(i == len - 1)
            buf[i] = IIC_Read_Byte(0);       // 最�?字节: NACK
        else
            buf[i] = IIC_Read_Byte(1);       // 其余: ACK（继续读�?
    }

    IIC_Stop();
    m24m01_pwr_off();
    return 0;
}

/* ===================== 诊断测试 ===================== */
void M24M01_Test(void)
{
    uint8_t ret, val;
    uint32_t test_addr;

    printf_debug("\r\n===== M24M01 Diagnostic Start =====\r\n");

    // I2C开关上�?+ I2C初始�?
    m24m01_pwr_on();

    // ---- 诊断A: SDA GPIO自检 ----
    printf_debug("[DiagA] SDA GPIO self-test:\r\n");
    IIC_SDA_H();
    IIC_Delay(); IIC_Delay(); IIC_Delay();
    printf_debug("  SDA=H -> read=%d (expect 1)\r\n", (int)IIC_SDA_READ());
    IIC_SDA_L();
    IIC_Delay(); IIC_Delay(); IIC_Delay();
    printf_debug("  SDA=L -> read=%d (expect 0)\r\n", (int)IIC_SDA_READ());
    IIC_SDA_H();
    IIC_SCL_H();

    // ---- 诊断B: I2C总线扫描 (M24M01 E2=E1=0: 只响�?0xA0/0xA2) ----
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

    // ---- 诊断C: 读未写过的高地址（空片应�?xFF�?---
    test_addr = 0x1FF0;
    ret = M24M01_Read(test_addr, &val, 1);
    printf_debug("[DiagC] Read virgin addr=0x%05lX: ret=%d val=0x%02X (expect 0xFF)\r\n",
                 (unsigned long)test_addr, ret, val);

    // ---- 诊断D: 单字节写+�?(低地址�? ----
    test_addr = 0x0050;
    printf_debug("[DiagD] WriteByte addr=0x%05lX data=0xA5\r\n", (unsigned long)test_addr);
    ret = M24M01_WriteByte(test_addr, 0xA5);
    printf_debug("  WriteByte ret=%d\r\n", ret);
    trea_delay_ms(10);

    val = 0x00;
    ret = M24M01_Read(test_addr, &val, 1);
    printf_debug("  Read ret=%d val=0x%02X\r\n", ret, val);
    if(val == 0xA5)
        printf_debug("  -> PASS\r\n");
    else
        printf_debug("  -> FAIL (expect 0xA5, got 0x%02X)\r\n", val);

    // ---- 诊断E: �?x55再读（不同数据排除巧合） ----
    printf_debug("[DiagE] WriteByte addr=0x%05lX data=0x55\r\n", (unsigned long)test_addr);
    ret = M24M01_WriteByte(test_addr, 0x55);
    printf_debug("  WriteByte ret=%d\r\n", ret);
    trea_delay_ms(10);

    val = 0x00;
    ret = M24M01_Read(test_addr, &val, 1);
    printf_debug("  Read ret=%d val=0x%02X\r\n", ret, val);
    if(val == 0x55)
        printf_debug("  -> PASS\r\n");
    else
        printf_debug("  -> FAIL (expect 0x55, got 0x%02X)\r\n", val);

    // ---- 诊断F: 高地址区写+�?(A16=1, 验证第二�?28KB可用) ----
    test_addr = 0x1F50;
    printf_debug("[DiagF] WriteByte addr=0x%05lX data=0x3C\r\n", (unsigned long)test_addr);
    ret = M24M01_WriteByte(test_addr, 0x3C);
    printf_debug("  WriteByte ret=%d\r\n", ret);
    trea_delay_ms(10);

    val = 0x00;
    ret = M24M01_Read(test_addr, &val, 1);
    printf_debug("  Read ret=%d val=0x%02X\r\n", ret, val);
    if(val == 0x3C)
        printf_debug("  -> PASS (upper 64KB OK)\r\n");
    else
        printf_debug("  -> FAIL (expect 0x3C, got 0x%02X)\r\n", val);

    m24m01_pwr_off();
    printf_debug("===== M24M01 Diagnostic End =====\r\n\r\n");
}



