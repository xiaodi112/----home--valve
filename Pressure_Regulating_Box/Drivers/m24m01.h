#ifndef M24M01__H
#define M24M01__H

#include "stdint.h"

/*
 * M24M01-RMN6P 关键参数（规格书�?
 * - 容量: 1Mbit = 128K Bytes (131072)
 * - 组织: 512�?× 256字节/�?
 * - 17位地址: A16编入设备地址字节, A15~A0�?字节字地址
 * - 设备地址字节: 1010 E2 E1 A16 R/W  (E2=E1=0 接地)
 * - 页写大小: 256字节
 * - 写周�? 最�?ms
 */

#define M24M01_BASE_ADDR   0xA0    // 基础设备地址 1010_00_0_0 (E2=0,E1=0)
#define M24M01_PAGE_SIZE   32     // 页大�?56字节
#define M24M01_TOTAL_SIZE  131072  // 总容�?28KB
#define M24M01_WRITE_TIME  5       // 写周�?5ms

// 根据17位地址计算设备地址字节（A16编入bit1�?
#define M24M01_DEV_ADDR(full_addr)  (M24M01_BASE_ADDR | ((((full_addr) >> 16) & 0x01) << 1))
// 取高字节字地址 A15~A8
#define M24M01_ADDR_HI(full_addr)   ((uint8_t)(((full_addr) >> 8) & 0xFF))
// 取低字节字地址 A7~A0
#define M24M01_ADDR_LO(full_addr)   ((uint8_t)((full_addr) & 0xFF))

// 写单字节
uint8_t M24M01_WriteByte(uint32_t addr, uint8_t data);
// 页写（最�?56字节，不可跨页）
uint8_t M24M01_WritePage(uint32_t addr, const uint8_t *data, uint16_t len);
// 任意长度写（自动分页，支持跨页）
uint8_t M24M01_Write(uint32_t addr, const uint8_t *data, uint16_t len);
// 随机读（任意地址，任意长度）
uint8_t M24M01_Read(uint32_t addr, uint8_t *buf, uint16_t len);
// ACK轮询等待写周期完�?
uint8_t M24M01_WaitReady(uint32_t addr, uint16_t timeout_ms);
// 读写测试（通过调试串口输出结果�?
void M24M01_Test(void);

#endif
