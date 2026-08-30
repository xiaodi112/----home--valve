#include "m24m01.h"
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
static inline void m24m01_bus_init(void)
{
    soft_i2c_init(&g_iic0);
}

/*
 * M24M01-RMN6P 驱动（严格匹配规格书）
 *
 * 地址机制: 17位地址 = A16编入设备地址字节bit1 + 2字节字地址(A15~A0)
 *   设备地址字节: 1010 E2 E1 A16 R/W  (E2=E1=0)
 *   发送2字节字地址: 先高(A15~A8) 再低(A7~A0)
 *
 * 页写规则: 每页256字节，页边界 = addr & 0x1FF00
 *   写入不可跨页，跨页时地址会回卷到页首
 *
 * 写周期: 每次写操作(字节写/页写)后需等待最长5ms
 */

/* ===================== ACK轮询等待写周期完成 ===================== */
uint8_t M24M01_WaitReady(uint32_t addr, uint16_t timeout_ms)
{
    uint8_t dev_byte = M24M01_DEV_ADDR(addr);
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
uint8_t M24M01_WriteByte(uint32_t addr, uint8_t data)
{
    uint8_t dev_byte = M24M01_DEV_ADDR(addr);
    uint8_t addr_hi  = M24M01_ADDR_HI(addr);
    uint8_t addr_lo  = M24M01_ADDR_LO(addr);
    uint8_t ret;

    if(addr >= M24M01_TOTAL_SIZE) return 1;

    m24m01_bus_init();

    soft_i2c_start(&g_iic0);
    if(soft_i2c_write_byte(&g_iic0, dev_byte) != 0) { soft_i2c_stop(&g_iic0); return 2; }
    if(soft_i2c_write_byte(&g_iic0, addr_hi) != 0)  { soft_i2c_stop(&g_iic0); return 3; }
    if(soft_i2c_write_byte(&g_iic0, addr_lo) != 0)  { soft_i2c_stop(&g_iic0); return 4; }
    if(soft_i2c_write_byte(&g_iic0, data) != 0)     { soft_i2c_stop(&g_iic0); return 5; }
    soft_i2c_stop(&g_iic0);

    ret = M24M01_WaitReady(addr, M24M01_WRITE_TIME + 1);
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

    m24m01_bus_init();

    soft_i2c_start(&g_iic0);
    if(soft_i2c_write_byte(&g_iic0, dev_byte) != 0) { soft_i2c_stop(&g_iic0); return 2; }
    if(soft_i2c_write_byte(&g_iic0, addr_hi) != 0)  { soft_i2c_stop(&g_iic0); return 3; }
    if(soft_i2c_write_byte(&g_iic0, addr_lo) != 0)  { soft_i2c_stop(&g_iic0); return 4; }

    for(i = 0; i < len; i++)
    {
        if(soft_i2c_write_byte(&g_iic0, data[i]) != 0) { soft_i2c_stop(&g_iic0); return 5; }
    }
    soft_i2c_stop(&g_iic0);

    ret = M24M01_WaitReady(addr, M24M01_WRITE_TIME + 1);
    return ret;
}

/* ===================== 任意长度写（自动分页） ===================== */
uint8_t M24M01_Write(uint32_t addr, const uint8_t *data, uint16_t len)
{
    uint16_t offset = 0;
    uint8_t  ret;

    while(offset < len)
    {
        // 当前页剩余可写字节数 (页大小256)
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

    m24m01_bus_init();

    /* 第一步: Dummy Write 设置读起始地址 (2字节) */
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
            buf[i] = soft_i2c_read_byte(&g_iic0, 0);       // 最后字节: NACK
        else
            buf[i] = soft_i2c_read_byte(&g_iic0, 1);       // 其余: ACK（继续读）
    }

    soft_i2c_stop(&g_iic0);
    return 0;
}

/* ===================== 诊断测试 ===================== */
void M24M01_Test(void)
{
    uint8_t ret, val;
    uint32_t test_addr;

    printf_debug("\r\n===== M24M01 Diagnostic Start =====\r\n");

    // IIC0 总线初始化
    m24m01_bus_init();

    // ---- 诊断A: SDA GPIO自检 ----
    printf_debug("[DiagA] SDA GPIO self-test:\r\n");
    I2C_SDA_H(&g_iic0);
    trea_delay_ms(1);
    printf_debug("  SDA=H -> read=%d (expect 1)\r\n", (int)I2C_SDA_READ(&g_iic0));
    I2C_SDA_L(&g_iic0);
    trea_delay_ms(1);
    printf_debug("  SDA=L -> read=%d (expect 0)\r\n", (int)I2C_SDA_READ(&g_iic0));
    I2C_SDA_H(&g_iic0);
    I2C_SCL_H(&g_iic0);

    // ---- 诊断B: I2C总线扫描 (M24M01 E2=E1=0: 只响应0xA0/0xA2) ----
    printf_debug("[DiagB] I2C bus scan (addr 0xA0~0xAE):\r\n");
    {
        uint8_t a;
        for(a = 0xA0; a <= 0xAE; a += 2)
        {
            ret = soft_i2c_check(&g_iic0, (uint8_t)(a >> 1));
            printf_debug("  0x%02X -> %s\r\n", a, (ret == 0) ? "ACK" : "NACK");
            trea_delay_ms(1);
        }
    }

    // ---- 诊断C: 读未写过的高地址（空片应为0xFF）----
    test_addr = 0x1FF0;
    ret = M24M01_Read(test_addr, &val, 1);
    printf_debug("[DiagC] Read virgin addr=0x%05lX: ret=%d val=0x%02X (expect 0xFF)\r\n",
                 (unsigned long)test_addr, ret, val);

    // ---- 诊断D: 单字节写+读(低地址区) ----
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

    // ---- 诊断E: 写0x55再读（不同数据排除巧合） ----
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

    // ---- 诊断F: 高地址区写+读(A16=1, 验证第二个128KB可用) ----
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

    printf_debug("===== M24M01 Diagnostic End =====\r\n\r\n");
}