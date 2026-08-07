#ifndef USART_H
#define USART_H

#include "gd32l23x.h"
#include "gd32l23x_libopt.h"
#include <stdbool.h>
#include <stdint.h>

int trea_uart_recv_byte_timeout(uint32_t uart, uint8_t* ch, uint32_t timeout_ms);
void trea_uart_init(void);
void lpuart_cat_uart_init(void);
int serial_write(uint32_t uart, const uint8_t *data, uint16_t len);
int lpuart_wait_for(const char *expect, uint32_t timeout_ms);
int printf_debug(const char* fmt, ...);
int printf_cat1(const char* fmt, ...);

// ===================== 【封装1】LPUART唤醒初始化 =====================
void lpuart_wakeup_init(void);
// ===================== 【封装2】进入深休眠 =====================
void enter_deepsleep_mode(uint8_t lpuart_enabled);
// ===================== 【封装3】唤醒后系统重新初始化 =====================
void system_wakeup_reinit(void);
// ===================== LPUART初始化 =====================
void com_lpuart_init(void);
extern __IO uint8_t counter0;  // LPUART唤醒标记

// ===================== LPUART0 DMA接收 =====================
void lpuart_dma_rx_init(void);                                     // DMA循环接收初始化(DMA_CH1)
void lpuart_dma_rx_disable(void);                                  // 禁用DMA接收(深休眠前)
void lpuart_rx_irq_handler(void);                                  // IDLE+错误中断服务(在LPUART_IRQHandler中调用)
void lpuart_rx_flush(void);                                        // 清空DMA接收缓冲(读指针跟到写指针)
void lpuart_rx_recover(void);                                      // TX后防御性恢复(清错误标志)
// IDLE帧级接收(OTA用)
extern volatile uint8_t  lpuart_frame_ready;                       // 帧完成标志(ISR置位)
extern volatile uint16_t lpuart_frame_end_pos;                     // 帧结束DMA位置
uint16_t lpuart_wait_frame(uint32_t timeout_ms);                   // 等待帧完成, 返回帧长度, 0=超时
uint16_t lpuart_read_frame(uint8_t *dst, uint16_t max_len);        // 拷贝帧数据到用户缓冲

// ===================== USART0 中断接收缓冲 =====================
void debug_rx_init(void);                                          // 初始化USART0 RX中断
void debug_rx_irq_handler(void);                                   // 中断服务(在USART0_IRQHandler中调用)
uint16_t debug_rx_read_line(char *buf, uint16_t buf_size, uint32_t timeout_ms); // 读一行(带超时), 返回长度, 0=超时
int debug_rx_has_data(void);                                       // 缓冲区是否有数据
int debug_rx_line_ready(void);                                     // ISR置位: 有完整行待处理

// ===================== UART4 蓝牙 DMA接收 =====================
void ble_uart_init(void);                                          // UART4初始化(PB3=TX,PB4=RX,115200,DMA)
void ble_uart_deinit(void);                                        // UART4反初始化(关闭蓝牙时调用)
void ble_uart_rx_irq_handler(void);                                // UART4 IDLE中断服务(在UART4_IRQHandler中调用)
uint16_t ble_uart_rx_available(void);                              // DMA缓冲区可读字节数
uint8_t  ble_uart_rx_read(void);                                   // 读取一字节(主循环调用)
void ble_uart_rx_flush(void);                                      // 清空DMA接收缓冲

#endif
