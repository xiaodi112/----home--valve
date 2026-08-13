#ifndef AT24C64_H
#define AT24C64_H

#include "stdint.h"

/*
 * AT24C64 (64Kbit = 8K Bytes) 关键参数:
 * - 容量: 8K Bytes (0x0000 ~ 0x1FFF)
 * - 13 位地址: A12~A0 (2 字节字地址: 高字节 A12~A8, 低字节 A7~A0)
 * - 设备地址: 1010 A2 A1 A0 R/W (A2/A1/A0 接地 = 0xA0)
 * - 页大小: 32 字节 (页边界 = addr & ~0x1F)
 * - 写周期: 最长 5ms (ACK 轮询等待)
 */

#define AT24C64_BASE_ADDR   0xA0    /* 基础设备地址 1010_000_0 */
#define AT24C64_PAGE_SIZE   32      /* 页大小: 32 字节 */
#define AT24C64_TOTAL_SIZE  8192    /* 总容量: 8KB */
#define AT24C64_WRITE_TIME  5       /* 写周期: 5ms */

/* 13 位地址: 高 5 位 (A12~A8) 放入字地址高字节低 5 位 */
#define AT24C64_ADDR_HI(full_addr)  ((uint8_t)(((full_addr) >> 8) & 0x1F))
#define AT24C64_ADDR_LO(full_addr)  ((uint8_t)((full_addr) & 0xFF))

/* 写单字节 */
uint8_t AT24C64_WriteByte(uint32_t addr, uint8_t data);
/* 页写 (最多 32 字节, 不可跨页) */
uint8_t AT24C64_WritePage(uint32_t addr, const uint8_t *data, uint16_t len);
/* 任意长度写 (自动分页, 支持跨页) */
uint8_t AT24C64_Write(uint32_t addr, const uint8_t *data, uint16_t len);
/* 随机读 (任意地址, 任意长度) */
uint8_t AT24C64_Read(uint32_t addr, uint8_t *buf, uint16_t len);
/* ACK 轮询等待写周期完成 */
uint8_t AT24C64_WaitReady(uint32_t addr, uint16_t timeout_ms);
/* 读写诊断测试 (通过调试串口输出结果) */
void AT24C64_Test(void);

#endif /* AT24C64_H */
