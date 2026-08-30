#include "at24c64.h"
#include "app_config.h"
#include "soft_i2c.h"
#include "delay.h"
#include "usart.h"
#include "gd32l23x.h"
#include <string.h>

/* ===================== EEPROM 总线 (IIC0: PB6/PB7) =====================
 * 硬件: EEPROM 常供电、常上拉, 与传感器电源(PC3)无关。
 * 每次读/写前调用总线初始化(开漏配置+死锁恢复), 常供电无需断电。
 */
static inline void at24c64_bus_init(void)
{
    soft_i2c_init(&g_iic0);
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
        soft_i2c_start(&g_iic0);
        if(soft_i2c_write_byte(&g_iic0, dev_byte) == 0)
        {
            soft_i2c_stop(&g_iic0);
            return 0;
        }
        soft_i2c_stop(&g_iic0);
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

    at24c64_bus_init();

    soft_i2c_start(&g_iic0);
    if(soft_i2c_write_byte(&g_iic0, dev_byte) != 0) { soft_i2c_stop(&g_iic0); return 2; }
    if(soft_i2c_write_byte(&g_iic0, addr_hi) != 0)  { soft_i2c_stop(&g_iic0); return 3; }
    if(soft_i2c_write_byte(&g_iic0, addr_lo) != 0)  { soft_i2c_stop(&g_iic0); return 4; }
    if(soft_i2c_write_byte(&g_iic0, data) != 0)     { soft_i2c_stop(&g_iic0); return 5; }
    soft_i2c_stop(&g_iic0);

    ret = AT24C64_WaitReady(addr, AT24C64_WRITE_TIME + 1);
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

    at24c64_bus_init();

    soft_i2c_start(&g_iic0);
    if(soft_i2c_write_byte(&g_iic0, dev_byte) != 0) { soft_i2c_stop(&g_iic0); return 2; }
    if(soft_i2c_write_byte(&g_iic0, addr_hi) != 0)  { soft_i2c_stop(&g_iic0); return 3; }
    if(soft_i2c_write_byte(&g_iic0, addr_lo) != 0)  { soft_i2c_stop(&g_iic0); return 4; }

    for(i = 0; i < len; i++)
    {
        if(soft_i2c_write_byte(&g_iic0, data[i]) != 0) { soft_i2c_stop(&g_iic0); return 5; }
    }
    soft_i2c_stop(&g_iic0);

    ret = AT24C64_WaitReady(addr, AT24C64_WRITE_TIME + 1);
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

    at24c64_bus_init();

    /* 第一步: Dummy Write 设置读起始地址 (2 字节) */
    soft_i2c_start(&g_iic0);
    if(soft_i2c_write_byte(&g_iic0, dev_byte | 0x00) != 0) { soft_i2c_stop(&g_iic0); return 2; }
    if(soft_i2c_write_byte(&g_iic0, addr_hi) != 0)         { soft_i2c_stop(&g_iic0); return 3; }
    if(soft_i2c_write_byte(&g_iic0, addr_lo) != 0)         { soft_i2c_stop(&g_iic0); return 4; }

    /* 第二步: 重复起始 + 读数据 */
    soft_i2c_start(&g_iic0);
    if(soft_i2c_write_byte(&g_iic0, dev_byte | 0x01) != 0) { soft_i2c_stop(&g_iic0); return 5; }

    for(i = 0; i < len; i++)
    {
        if(i == len - 1)
            buf[i] = soft_i2c_read_byte(&g_iic0, 0);       /* 最后字节: NACK */
        else
            buf[i] = soft_i2c_read_byte(&g_iic0, 1);       /* 其余: ACK (继续读) */
    }

    soft_i2c_stop(&g_iic0);
    return 0;
}

/* ===================== 诊断测试 ===================== */
void AT24C64_Test(void)
{
    uint8_t ret, val;
    uint32_t test_addr;

    printf_debug("\r\n===== AT24C64 Diagnostic Start =====\r\n");

    at24c64_bus_init();

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

    printf_debug("===== AT24C64 Diagnostic End =====\r\n\r\n");
}