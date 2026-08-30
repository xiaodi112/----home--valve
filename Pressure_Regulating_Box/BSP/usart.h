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

// ===================== 封装好1号LPUART唤醒初始化 =====================
void lpuart_wakeup_init(void);
// 唤醒后统一由 BSP/wakeup.c (wakeup.h) 重新初始化
// ===================== LPUART初始化 =====================
void com_lpuart_init(void);
extern __IO uint8_t counter0;  // LPUART唤醒标志

// ===================== LPUART0 DMA接收 =====================
void lpuart_dma_rx_init(void);                                     // DMA循环接收初始化(DMA_CH1)
void lpuart_dma_rx_disable(void);                                  // 关闭DMA接收(进入休眠前)
void lpuart_rx_irq_handler(void);                                  // IDLE+错误中断处理(由LPUART_IRQHandler调用)
void lpuart_rx_flush(void);                                        // 清空DMA接收缓冲(丢弃残留数据)
void lpuart_rx_recover(void);                                      // TX后防御性恢复(清除错误标志)
// IDLE帧接收(OTA用)
extern volatile uint8_t  lpuart_frame_ready;                       // 帧就绪标志(ISR置位)
extern volatile uint16_t lpuart_frame_end_pos;                     // 帧结束DMA位置
uint16_t lpuart_wait_frame(uint32_t timeout_ms);                   // 等待帧就绪, 返回帧长度, 0=超时
uint16_t lpuart_read_frame(uint8_t *dst, uint16_t max_len);        // 拷贝帧数据到用户缓冲

// ===================== USART0 调试串口接收 =====================
void debug_rx_init(void);                                          // 初始化USART0 RX中断
void debug_rx_irq_handler(void);                                   // 中断处理(由USART0_IRQHandler调用)
uint16_t debug_rx_read_line(char *buf, uint16_t buf_size, uint32_t timeout_ms); // 读取一行(带超时), 失败返回0
int debug_rx_has_data(void);                                       // 是否有缓冲数据
int debug_rx_line_ready(void);                                     // ISR置位: 是否有完整行

// ===================== UART4 蓝牙 DMA接收 =====================
void ble_uart_init(void);                                          // UART4初始化(PB3=TX,PB4=RX,115200,DMA)
void ble_uart_deinit(void);                                        // UART4关闭初始化(关外设与GPIO)
void ble_uart_rx_irq_handler(void);                                // UART4 IDLE中断处理(由UART4_IRQHandler调用)
uint16_t ble_uart_rx_available(void);                              // DMA缓冲中可读字节数
uint8_t  ble_uart_rx_read(void);                                   // 读取一个字节(环形缓冲)
void ble_uart_rx_flush(void);                                      // 清空DMA接收缓冲

// ===================== USART1 (CH4 sensor) =====================
void usart1_ch4_init(void);        // PA2=TX, PA3=RX, AF7, 115200, 8N1, IDLE IRQ
void usart1_ch4_deinit(void);      // deinit + GPIO analog
void usart1_ch4_irq_handler(void); // IDLE IRQ handler
uint8_t *ch4_get_rx_buf(void);
uint16_t ch4_get_rx_len(void);
uint8_t  ch4_get_frame_ready(void);
void     ch4_clear_frame_ready(void);

#endif
