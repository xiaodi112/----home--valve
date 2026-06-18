#ifndef AT24C16__H
#define AT24C16__H

#include "stdint.h"

/*
 * AT24C16 关键参数（规格书）
 * - 容量: 16Kbit = 2048 Bytes
 * - 组织: 128页 × 16字节/页
 * - 11位地址: 高3位(P2P1P0)编入设备地址字节, 低8位为字地址
 * - 设备地址字节: 1010 P2 P1 P0 R/W
 * - 页写大小: 16字节
 * - 写周期: 最大5ms
 */

#define AT24C16_BASE_ADDR   0xA0    // 基础设备地址 1010_000_0
#define AT24C16_PAGE_SIZE   16      // 页大小16字节
#define AT24C16_TOTAL_SIZE  2048    // 总容量2048字节
#define AT24C16_WRITE_TIME  5       // 写周期 5ms

// 根据11位地址计算设备地址字节（高3位编入设备地址）
#define AT24C16_DEV_ADDR(full_addr)  (AT24C16_BASE_ADDR | (((full_addr) >> 7) & 0x0E))
// 取低8位字地址
#define AT24C16_WORD_ADDR(full_addr) ((uint8_t)((full_addr) & 0xFF))

// 写单字节（支持0~2047全地址范围）
uint8_t AT24C16_WriteByte(uint16_t addr, uint8_t data);
// 页写（最多16字节，不可跨页）
uint8_t AT24C16_WritePage(uint16_t addr, const uint8_t *data, uint8_t len);
// 任意长度写（自动分页，支持跨页）
uint8_t AT24C16_Write(uint16_t addr, const uint8_t *data, uint16_t len);
// 随机读（任意地址，任意长度）
uint8_t AT24C16_Read(uint16_t addr, uint8_t *buf, uint16_t len);
// ACK轮询等待写周期完成
uint8_t AT24C16_WaitReady(uint16_t addr, uint16_t timeout_ms);
// 读写测试（通过调试串口输出结果）
void AT24C16_Test(void);

#endif
