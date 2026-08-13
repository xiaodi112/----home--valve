#include "at24c64.h"
#include "app_config.h"
#include "i2c_sensor.h"
#include "delay.h"
#include "usart.h"
#include "gd32l23x.h"
#include <string.h>

/* ===================== I2C 电子开关 PD4 (SW_IIC) 管理 =====================
 * PD4 = SW_IIC 电子开关 (TPS22918), 控制所有 I2C 设备电源
 * 默认 LOW 节能; 每次 EEPROM 读/写前自动 set + 20ms 上电稳定, 操作后 reset
 * 接口调用者无需关心 PD4 状态 (与传感器采集路径的 PD4 状态独立)
 */
#if IIC_SWITCH_ENABLED
static uint8_t s_pwr_init_done = 0;

#endif
static inline void at24c64_pwr_on(void)
{
#if IIC_SWITCH_ENABLED
    rcu_periph_clock_enable(RCU_GPIOD);
    gpio_mode_set(SW_IIC_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SW_IIC_GPIO_PIN);
    gpio_output_options_set(SW_IIC_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, SW_IIC_GPIO_PIN);

    if(!s_pwr_init_done) {
        s_pwr_init_done = 1;
        gpio_bit_reset(SW_IIC_GPIO_PORT, SW_IIC_GPIO_PIN);
        trea_delay_ms(50);
    }

    gpio_bit_set(SW_IIC_GPIO_PORT, SW_IIC_GPIO_PIN);
    trea_delay_ms(20);   /* EEPROM 上电稳定 (手册典型 1ms, 留余量) */
#endif
    IIC_Init();          /* 重新配置 PC0/PC1 GPIO + 总线死锁恢复 (与传感器共用总线) */
}

static inline void at24c64_pwr_off(void)
{
#if IIC_SWITCH_ENABLED
    gpio_bit_reset(SW_IIC_GPIO_PORT, SW_IIC_GPIO_PIN);
#endif
}

/*
 * AT24C64 驱动 (严格匹配规格书)
 *
 * 地址机制: 13 位地址 = 2 字节字地址 (高字节 A12~A8, 低字节 A7~A0)
 *   设备地址字节: 1010 A2 A1 A0 R/W  (A2/A1/A0 = 0)
 *   发送字地址: 先高字节 再低字节
 *
 * 页写规则: 每页 32 字节, 页边界 = addr & ~0x1F
 *   写入不可跨页, 跨页时地址回卷到页首
 *
 * 写周期: 每次写操作后需等待最长 5ms (ACK 轮询)
 */

/* ===================== ACK轮询等待写周期完成 ===================== */
uint8_t AT24C64_WaitReady(uint32_t addr, uint16_t timeout_ms)
{
    uint8_t dev_byte = AT24C64_BASE_ADDR;
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
uint8_t AT24C64_WriteByte(uint32_t addr, uint8_t data)
{
    uint8_t dev_byte = AT24C64_BASE_ADDR;
    uint8_t addr_hi  = AT24C64_ADDR_HI(addr);
    uint8_t addr_lo  = AT24C64_ADDR_LO(addr);
    uint8_t ret;

    if(addr >= AT24C64_TOTAL_SIZE) return 1;

    at24c64_pwr_on();

    IIC_Start();
    IIC_Send_Byte(dev_byte);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); at24c64_pwr_off(); return 2; }

    IIC_Send_Byte(addr_hi);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); at24c64_pwr_off(); return 3; }

    IIC_Send_Byte(addr_lo);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); at24c64_pwr_off(); return 4; }

    IIC_Send_Byte(data);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); at24c64_pwr_off(); return 5; }

    IIC_Stop();

    ret = AT24C64_WaitReady(addr, AT24C64_WRITE_TIME + 1);
    at24c64_pwr_off();
    return ret;
}

/* ===================== 页写 (不可跨页, 调用者保证) ===================== */
uint8_t AT24C64_WritePage(uint32_t addr, const uint8_t *data, uint16_t len)
{
    uint8_t dev_byte = AT24C64_BASE_ADDR;
    uint8_t addr_hi  = AT24C64_ADDR_HI(addr);
    uint8_t addr_lo  = AT24C64_ADDR_LO(addr);
    uint16_t i;
    uint8_t ret;

    if(addr >= AT24C64_TOTAL_SIZE || len == 0 || len > AT24C64_PAGE_SIZE) return 1;

    at24c64_pwr_on();

    IIC_Start();
    IIC_Send_Byte(dev_byte);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); at24c64_pwr_off(); return 2; }

    IIC_Send_Byte(addr_hi);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); at24c64_pwr_off(); return 3; }

    IIC_Send_Byte(addr_lo);
    if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); at24c64_pwr_off(); return 4; }

    for(i = 0; i < len; i++)
    {
        IIC_Send_Byte(data[i]);
        if(IIC_Wait_Ack(100) != 0) { IIC_Stop(); at24c64_pwr_off(); return 5; }
    }

    IIC_Stop();

    ret = AT24C64_WaitReady(addr, AT24C64_WRITE_TIME + 1);
    at24c64_pwr_off();
    return ret;
}

/* ===================== 任意长度写 (自动分页) ===================== */
uint8_t AT24C64_Write(uint32_t addr, const uint8_t *data, uint16_t len)
{
    uint16_t offset = 0;
    uint8_t  ret;

    while(offset < len)
    {
        /* 当前页剩余可写字节数 (页大小 32) */
        uint16_t page_remain = AT24C64_PAGE_SIZE - ((addr + offset) % AT24C64_PAGE_SIZE);
        uint16_t left = len - offset;
        uint16_t chunk = (left < page_remain) ? left : page_remain;

        ret = AT24C64_WritePage(addr + offset, data + offset, chunk);
        if(ret != 0) return ret;

        offset += chunk;
    }
    return 0;
}

/* ===================== 随机读 (任意地址、任意长度) ===================== */
uint8_t AT24C64_Read(uint32_t addr, uint8_t *buf, uint16_t len)
{
    uint8_t dev_byte = AT24C64_BASE_ADDR;
    uint8_t addr_hi  = AT24C64_ADDR_HI(addr);
    uint8_t addr_lo  = AT24C64_ADDR_LO(addr);
    uint16_t i;

    if(addr >= AT24C64_TOTAL_SIZE || len == 0) return 1;

    at24c64_pwr_on();

    /* 第一步: Dummy Write 设置读起始地址 (2 字节) */
    IIC_Start();
    IIC_Send_Byte(dev_byte | 0x00);          /* 设备地址 + 写位 */
    if(IIC_Wait_Ack(1000) != 0) { IIC_Stop(); at24c64_pwr_off(); return 2; }

    IIC_Send_Byte(addr_hi);                  /* 字地址高字节 A12~A8 */
    if(IIC_Wait_Ack(1000) != 0) { IIC_Stop(); at24c64_pwr_off(); return 3; }

    IIC_Send_Byte(addr_lo);                  /* 字地址低字节 A7~A0 */
    if(IIC_Wait_Ack(1000) != 0) { IIC_Stop(); at24c64_pwr_off(); return 4; }

    /* 第二步: 重复起始 + 读数据 */
    IIC_Start();
    IIC_Send_Byte(dev_byte | 0x01);          /* 设备地址 + 读位 */
    if(IIC_Wait_Ack(1000) != 0) { IIC_Stop(); at24c64_pwr_off(); return 5; }

    for(i = 0; i < len; i++)
    {
        if(i == len - 1)
            buf[i] = IIC_Read_Byte(0);       /* 最后字节: NACK */
        else
            buf[i] = IIC_Read_Byte(1);       /* 其余: ACK (继续读) */
    }

    IIC_Stop();
    at24c64_pwr_off();
    return 0;
}

/* ===================== 诊断测试 ===================== */
void AT24C64_Test(void)
{
    uint8_t ret, val;
    uint32_t test_addr;

    printf_debug("\r\n===== AT24C64 Diagnostic Start =====\r\n");

    at24c64_pwr_on();

    // 读取未写过的地址 (空片应为 0xFF)
    test_addr = 0x1FF0;
    ret = AT24C64_Read(test_addr, &val, 1);
    printf_debug("[DiagC] Read virgin addr=0x%04lX: ret=%d val=0x%02X (expect 0xFF)\r\n",
                 (unsigned long)test_addr, ret, val);

    // 单字节写 + 读 (低地址区)
    test_addr = 0x0050;
    printf_debug("[DiagD] WriteByte addr=0x%04lX data=0xA5\r\n", (unsigned long)test_addr);
    ret = AT24C64_WriteByte(test_addr, 0xA5);
    printf_debug("  WriteByte ret=%d\r\n", ret);
    trea_delay_ms(10);

    val = 0x00;
    ret = AT24C64_Read(test_addr, &val, 1);
    printf_debug("  Read ret=%d val=0x%02X\r\n", ret, val);
    if(val == 0xA5)
        printf_debug("  -> PASS\r\n");
    else
        printf_debug("  -> FAIL (expect 0xA5, got 0x%02X)\r\n", val);

    at24c64_pwr_off();
    printf_debug("===== AT24C64 Diagnostic End =====\r\n\r\n");
}
