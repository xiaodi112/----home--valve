#include "usart.h"
#include "mygpio.h"
#include "app_config.h"

#include "delay.h"
#include "systick.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define TREA_UART_DEBUG USART0
#define TREA_UART_CAT1  LPUART0

static void trea_uart_send_byte(uint32_t uart, uint8_t ch)
{
    if(uart == LPUART0)
    {
        lpuart_data_transmit(LPUART0, ch);
        while(RESET == lpuart_flag_get(LPUART0, LPUART_FLAG_TBE));
    }
    else
    {
        usart_data_transmit(uart, ch);
        while(RESET == usart_flag_get(uart, USART_FLAG_TBE));
    }
}

// ===================== LPUART0 DMA接收缓冲 =====================
// DMA循环模式接收串口数据, 供AT指令响应解析
#define LPUART_DMA_BUF_SIZE  1024
#define LPUART_DMA_CHANNEL   DMA_CH1

static uint8_t  lpuart_dma_buf[LPUART_DMA_BUF_SIZE];
static volatile uint16_t lpuart_dma_read_pos = 0;  // 软件读取位置

// IDLE帧中断: 用于OTA帧接收
volatile uint8_t  lpuart_frame_ready = 0;     // 帧已就绪(IDLE中断置位)
volatile uint16_t lpuart_frame_end_pos = 0;   // 帧结束DMA写位置

// 获取DMA当前写位置(DMA正在写入的数据位置)
static uint16_t lpuart_dma_write_pos(void)
{
    return (uint16_t)(LPUART_DMA_BUF_SIZE - dma_transfer_number_get(LPUART_DMA_CHANNEL));
}

// 初始化LPUART0 DMA接收 (DMA_CH1, 循环模式)
void lpuart_dma_rx_init(void)
{
    dma_parameter_struct dma_init_struct;

    rcu_periph_clock_enable(RCU_DMA);

    dma_deinit(LPUART_DMA_CHANNEL);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.request      = DMA_REQUEST_LPUART_RX;
    dma_init_struct.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.periph_addr  = (uint32_t)&LPUART_RDATA(LPUART0);
    dma_init_struct.memory_addr  = (uint32_t)lpuart_dma_buf;
    dma_init_struct.number       = LPUART_DMA_BUF_SIZE;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.priority     = DMA_PRIORITY_HIGH;
    dma_init(LPUART_DMA_CHANNEL, &dma_init_struct);

    // 循环模式: 写满末尾后自动回卷到头部
    dma_circulation_enable(LPUART_DMA_CHANNEL);
    dma_channel_enable(LPUART_DMA_CHANNEL);

    // LPUART使能DMA接收请求
    lpuart_dma_receive_config(LPUART0, LPUART_RECEIVE_DMA_ENABLE);

    lpuart_dma_read_pos = 0;
}

// 关闭DMA接收(进入休眠前调用)
void lpuart_dma_rx_disable(void)
{
    dma_channel_disable(LPUART_DMA_CHANNEL);
    lpuart_dma_receive_config(LPUART0, LPUART_RECEIVE_DMA_DISABLE);
}

// LPUART IRQ处理: IDLE帧中断+ 错误清除
void lpuart_rx_irq_handler(void)
{
    // IDLE帧中断: 一帧接收完成置位
    if(SET == lpuart_interrupt_flag_get(LPUART0, LPUART_INT_FLAG_IDLE))
    {
        lpuart_flag_clear(LPUART0, LPUART_FLAG_IDLE);
        lpuart_frame_end_pos = lpuart_dma_write_pos();
        lpuart_frame_ready = 1;
    }
    // 清除错误标志: 一次性清除全部错误(ORE/FE/NE/PE),
    // 防止未处理的中断源(如 Noise Error)导致中断风暴
    LPUART_INTC(LPUART0) = LPUART_INTC_PEC | LPUART_INTC_FEC
                          | LPUART_INTC_NEC | LPUART_INTC_OREC;
}

// 清空DMA接收缓冲(丢弃残留数据)
void lpuart_rx_flush(void)
{
    lpuart_dma_read_pos = lpuart_dma_write_pos();
    lpuart_frame_ready = 0;
    // 清除溢出标志
    if(SET == lpuart_flag_get(LPUART0, LPUART_FLAG_ORERR))
        lpuart_flag_clear(LPUART0, LPUART_FLAG_ORERR);
    if(SET == lpuart_flag_get(LPUART0, LPUART_FLAG_FERR))
        lpuart_flag_clear(LPUART0, LPUART_FLAG_FERR);
    // 清IDLE标志(防止重复触发)
    if(SET == lpuart_flag_get(LPUART0, LPUART_FLAG_IDLE))
        lpuart_flag_clear(LPUART0, LPUART_FLAG_IDLE);
}

// 等待一帧数据(IDLE中断), 返回帧长度; 0=超时
uint16_t lpuart_wait_frame(uint32_t timeout_ms)
{
    uint32_t start = trea_millis();
    uint16_t rp;

    /* Critical section: prevent IDLE ISR from setting frame_ready between read and clear */
    nvic_irq_disable(LPUART_IRQn);
    rp = lpuart_dma_read_pos;
    lpuart_frame_ready = 0;
    nvic_irq_enable(LPUART_IRQn, 1);

    while((trea_millis() - start) < timeout_ms)
    {
        if(lpuart_frame_ready)
        {
            uint16_t ep = lpuart_frame_end_pos;
            uint16_t len = (ep >= rp) ? (ep - rp) : (LPUART_DMA_BUF_SIZE - rp + ep);
            return len;
        }
    }
    return 0;
}

// 从DMA接收缓冲拷贝数据到用户缓冲, 按帧读取, 更新read_pos)
uint16_t lpuart_read_frame(uint8_t *dst, uint16_t max_len)
{
    uint16_t wp, rp, len;

    /* Critical section: snapshot wp/rp atomically to prevent IDLE ISR race */
    nvic_irq_disable(LPUART_IRQn);
    wp = lpuart_frame_end_pos;
    rp = lpuart_dma_read_pos;
    len = (wp >= rp) ? (wp - rp) : (LPUART_DMA_BUF_SIZE - rp + wp);
    nvic_irq_enable(LPUART_IRQn, 1);

    if(len > max_len) len = max_len;

    for(uint16_t i = 0; i < len; i++)
    {
        dst[i] = lpuart_dma_buf[rp];
        rp = (rp + 1) % LPUART_DMA_BUF_SIZE;
    }
    lpuart_dma_read_pos = rp;
    lpuart_frame_ready = 0;
    return len;
}

int trea_uart_recv_byte_timeout(uint32_t uart, uint8_t* ch, uint32_t timeout_ms)
{
    uint32_t start = trea_millis();
    if(uart == LPUART0)
    {
         // 对LPUART0 专用路径走 DMA 缓冲读取
        while((trea_millis() - start) < timeout_ms) {
            uint16_t wp = lpuart_dma_write_pos();
            if(lpuart_dma_read_pos != wp) {
                *ch = lpuart_dma_buf[lpuart_dma_read_pos];
                lpuart_dma_read_pos = (lpuart_dma_read_pos + 1) % LPUART_DMA_BUF_SIZE;
                return 1;
            }
        }
        return 0;
    }
    // 其他UART: 轮询硬件接收寄存器
    while((trea_millis() - start) < timeout_ms) {
        if(RESET != usart_flag_get(uart, USART_FLAG_RBNE)) {
            *ch = (uint8_t)usart_data_receive(uart);
            return 1;
        }
    }
    return 0;
}

void trea_uart_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_USART0);

    gpio_af_set(GPIOC, GPIO_AF_7, GPIO_PIN_4);
    gpio_af_set(GPIOC, GPIO_AF_7, GPIO_PIN_5);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_4);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_4);
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_5);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_5);

    usart_deinit(TREA_UART_DEBUG);
    usart_baudrate_set(TREA_UART_DEBUG, 115200U);
    usart_word_length_set(TREA_UART_DEBUG, USART_WL_8BIT);
    usart_stop_bit_set(TREA_UART_DEBUG, USART_STB_1BIT);
    usart_parity_config(TREA_UART_DEBUG, USART_PM_NONE);
    usart_receive_config(TREA_UART_DEBUG, USART_RECEIVE_ENABLE);
    usart_transmit_config(TREA_UART_DEBUG, USART_TRANSMIT_ENABLE);
    usart_enable(TREA_UART_DEBUG);
}


int serial_write(uint32_t uart, const uint8_t *data, uint16_t len)
{
    if(data == NULL || len == 0) return 0;
    for(uint16_t i = 0; i < len; i++) {
        trea_uart_send_byte(uart, data[i]);
    }
    return (int)len;
}

void lpuart_rx_recover(void)
{
    // TX完成后防御性恢复: 清除错误标志(DMA模式不可用中断使能时调用)
    if(SET == lpuart_flag_get(LPUART0, LPUART_FLAG_ORERR))
        lpuart_flag_clear(LPUART0, LPUART_FLAG_ORERR);
    if(SET == lpuart_flag_get(LPUART0, LPUART_FLAG_FERR))
        lpuart_flag_clear(LPUART0, LPUART_FLAG_FERR);
}

int lpuart_wait_for(const char *expect, uint32_t timeout_ms)
{
    if(expect == NULL || expect[0] == 0) return 0;
    uint32_t start = trea_millis();
    char rx[256];
    uint16_t n = 0;
    rx[0] = 0;

    while((trea_millis() - start) < timeout_ms) {
        uint8_t ch = 0;
        if(trea_uart_recv_byte_timeout(TREA_UART_CAT1, &ch, 30)) {
            if(n < (uint16_t)(sizeof(rx) - 1)) {
                if(ch == '\r' || ch == '\n' || (ch >= 0x20U && ch <= 0x7EU)) {
                    rx[n++] = (char)ch;
                } else {
                    rx[n++] = '.';
                }
                rx[n] = 0;
            }
            if(strstr(rx, expect) != NULL) return 1;
        }
    }
    printf_debug("[EC] rx(%d): %s\r\n", n, rx);
    return 0;
}

int printf_debug(const char* fmt, ...)
{
    static char buf[512];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if(n < 0) return n;
    if(n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
    serial_write(TREA_UART_DEBUG, (const uint8_t*)buf, (uint16_t)n);
    return n;
}
int printf_cat1(const char* fmt, ...)
{
    static char buf[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if(n < 0) return n;
    if(n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
    serial_write(TREA_UART_CAT1, (const uint8_t*)buf, (uint16_t)n);
	memset(buf,0,sizeof(buf));
    return n;
}


// ===================== 封装好的LPUART唤醒初始化 =====================
void lpuart_wakeup_init(void)
{
    // 时钟源在 com_lpuart_init() 中已配置为 IRC16MDIV, 波特率与唤醒一致

    // 使能唤醒中断 + EXTI中断
    nvic_irq_enable(LPUART_WKUP_IRQn, 0);
    exti_init(EXTI_28, EXTI_INTERRUPT, EXTI_TRIG_RISING);

    // 起始位唤醒模式
    lpuart_wakeup_mode_config(LPUART0, LPUART_WUM_STARTB);
    lpuart_enable(LPUART0);

    while(RESET == lpuart_flag_get(LPUART0, LPUART_FLAG_REA));
    while(SET == lpuart_flag_get(LPUART0, LPUART_FLAG_BSY));

    lpuart_wakeup_enable(LPUART0);
    lpuart_interrupt_enable(LPUART0, LPUART_INT_WU);
}

// ===================== LPUART初始化 =====================
void com_lpuart_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_LPUART);

    gpio_af_set(GPIOD, GPIO_AF_8, GPIO_PIN_8);   /* PD8 = LPUART0_TX 即 LP_TXD 连 4G模块 */
    gpio_af_set(GPIOD, GPIO_AF_8, GPIO_PIN_9);   /* PD9 = LPUART0_RX 即 LP_RXD 连 4G模块 */

    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_8);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_8);
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_9);

    lpuart_deinit(LPUART0);
    // 时钟源选择在 UE=0 (deinit完成) 且 baudrate_set 之前配置,
    // 确保 BRR 按 IRC16M 时钟计算正确, 规避"UE=1时禁止切换时钟"的硬件约束
    rcu_lpuart_clock_config(RCU_LPUARTSRC_IRC16MDIV);
    lpuart_word_length_set(LPUART0, LPUART_WL_8BIT);
    lpuart_stop_bit_set(LPUART0, LPUART_STB_1BIT);
    lpuart_parity_config(LPUART0, LPUART_PM_NONE);
    lpuart_baudrate_set(LPUART0, 115200U);
    lpuart_receive_config(LPUART0, LPUART_RECEIVE_ENABLE);
    lpuart_transmit_config(LPUART0, LPUART_TRANSMIT_ENABLE);
    lpuart_enable(LPUART0);

    // DMA循环接收初始化: 避免RBNE中断, 减少中断消耗)
    lpuart_dma_rx_init();
    // IDLE中断: 帧接收完成标志(OTA等帧协议)
    // 注: 未使能 LPUART_INT_ERR(EIE), DMA模式下不可用中断使能/恢复方式处理
    //     EIE 会使 NE(Noise Error) 触发未完全处理的中断, 可能导致中断风暴
    lpuart_interrupt_enable(LPUART0, LPUART_INT_IDLE);
    nvic_irq_enable(LPUART_IRQn, 1);
}

// ===================== USART0 调试串口接收缓冲 =====================
#define DEBUG_RX_BUF_SIZE  256
static volatile char    drx_buf[DEBUG_RX_BUF_SIZE];
static volatile uint16_t drx_head = 0;   // 写位置 (中断更新)
static volatile uint16_t drx_tail = 0;   // 读位置 (主循环更新)
static volatile uint8_t  drx_line_ready = 0; // 整行就绪标志

void debug_rx_init(void)
{
    // 使能USART0 RBNE中断
    usart_interrupt_enable(USART0, USART_INT_RBNE);
    nvic_irq_enable(USART0_IRQn, 2);
    drx_head = 0;
    drx_tail = 0;
    drx_line_ready = 0;
}

void debug_rx_irq_handler(void)
{
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE))
    {
        char ch = (char)usart_data_receive(USART0);
        uint16_t next = (drx_head + 1) % DEBUG_RX_BUF_SIZE;
        if(next != drx_tail)  // 缓冲未满
        {
            drx_buf[drx_head] = ch;
            drx_head = next;
            if(ch == '\r' || ch == '\n')
                drx_line_ready = 1;
        }
    }
}

int debug_rx_has_data(void)
{
    return (drx_head != drx_tail) ? 1 : 0;
}

int debug_rx_line_ready(void)
{
    return drx_line_ready ? 1 : 0;
}

uint16_t debug_rx_read_line(char *buf, uint16_t buf_size, uint32_t timeout_ms)
{
    uint16_t idx = 0;

    // 等待整行就绪
    uint32_t t0 = trea_millis();
    while(!drx_line_ready)
    {
        if((trea_millis() - t0) >= timeout_ms)
        {
            buf[0] = '\0';
            return 0;
        }
    }

    // 从环形缓冲拷贝一行, 失败时回滚读指针防止数据丢失
    uint16_t saved_tail = drx_tail;

    while(drx_tail != drx_head && idx < buf_size - 1)
    {
        char ch = drx_buf[drx_tail];
        drx_tail = (drx_tail + 1) % DEBUG_RX_BUF_SIZE;

        if(ch == '\r' || ch == '\n')
        {
            if(idx > 0)
            {
                buf[idx] = '\0';
                // 跳过多余换行 \r\n
                while(drx_tail != drx_head)
                {
                    char peek = drx_buf[drx_tail];
                    if(peek == '\r' || peek == '\n')
                        drx_tail = (drx_tail + 1) % DEBUG_RX_BUF_SIZE;
                    else
                        break;
                }
                // 重新检查缓冲中是否有新行
                drx_line_ready = 0;
                {
                    uint16_t t = drx_tail;
                    while(t != drx_head)
                    {
                        if(drx_buf[t] == '\r' || drx_buf[t] == '\n')
                        { drx_line_ready = 1; break; }
                        t = (t + 1) % DEBUG_RX_BUF_SIZE;
                    }
                }
                return idx;
            }
            saved_tail = drx_tail;  // 无整行时回滚读指针到保存值
            continue;  // 跳过空行 \r\n
        }
        buf[idx++] = ch;
    }

    // 没有完整行(缺少结尾\r\n)时回滚读指针, 等待下次数据
    drx_tail = saved_tail;
    drx_line_ready = 0;
    buf[0] = '\0';
    return 0;
}

// ===================== UART4 蓝牙 DMA接收 =====================
// DMA循环模式 + IDLE中断, 类似 LPUART0 DMA 机制
#define BLE_DMA_BUF_SIZE     1024
#define BLE_DMA_CHANNEL      DMA_CH2

static uint8_t  ble_dma_buf[BLE_DMA_BUF_SIZE];
static volatile uint16_t ble_dma_read_pos = 0;

// 获取DMA当前写位置
static uint16_t ble_dma_write_pos(void)
{
    return (uint16_t)(BLE_DMA_BUF_SIZE - dma_transfer_number_get(BLE_DMA_CHANNEL));
}

// UART4初始化: PB3(TX)/PB4(RX), AF8, 115200, DMA循环接收 + IDLE中断
void ble_uart_init(void)
{
	
    dma_parameter_struct dma_init_struct;

    /* GPIO + UART4 时钟 */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_UART4);
    rcu_periph_clock_enable(RCU_DMA);

    /* PB3 = UART4_TX, PB4 = UART4_RX, AF8 */
    gpio_af_set(GPIOB, GPIO_AF_8, GPIO_PIN_3);
    gpio_af_set(GPIOB, GPIO_AF_8, GPIO_PIN_4);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_3);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_3);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_4);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_4);

    /* UART4 基本配置 */
    usart_deinit(UART4);
    usart_baudrate_set(UART4, 115200U);
    usart_word_length_set(UART4, USART_WL_8BIT);
    usart_stop_bit_set(UART4, USART_STB_1BIT);
    usart_parity_config(UART4, USART_PM_NONE);
    usart_receive_config(UART4, USART_RECEIVE_ENABLE);
    usart_transmit_config(UART4, USART_TRANSMIT_ENABLE);
    usart_enable(UART4);

    /* DMA 循环接收初始化 */
    dma_deinit(BLE_DMA_CHANNEL);
    dma_struct_para_init(&dma_init_struct);
    dma_init_struct.request      = DMA_REQUEST_UART4_RX;
    dma_init_struct.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.periph_addr  = (uint32_t)&USART_RDATA(UART4);
    dma_init_struct.memory_addr  = (uint32_t)ble_dma_buf;
    dma_init_struct.number       = BLE_DMA_BUF_SIZE;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.priority     = DMA_PRIORITY_HIGH;
    dma_init(BLE_DMA_CHANNEL, &dma_init_struct);

    dma_circulation_enable(BLE_DMA_CHANNEL);
    dma_channel_enable(BLE_DMA_CHANNEL);

    /* UART4 使能DMA接收 */
    usart_dma_receive_config(UART4, USART_RECEIVE_DMA_ENABLE);

    /* IDLE中断: 帧接收完成标志*/
    usart_interrupt_enable(UART4, USART_INT_IDLE);
    nvic_irq_enable(UART4_IRQn, 1);
	
    /* 复位读位置指针 */
    ble_dma_read_pos = 0;
	
}

// UART4 关闭初始化 (关闭外设时钟和GPIO)
void ble_uart_deinit(void)
{
    /* 关闭中断 */
    usart_interrupt_disable(UART4, USART_INT_IDLE);
    nvic_irq_disable(UART4_IRQn);

    /* 关闭DMA */
    dma_channel_disable(BLE_DMA_CHANNEL);
    usart_dma_receive_config(UART4, USART_RECEIVE_DMA_DISABLE);

    /* 关闭UART4 */
    usart_disable(UART4);

    /* GPIO恢复为模拟输入 (省电) */
    gpio_mode_set(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_3 | GPIO_PIN_4);

    /* 关闭UART4时钟 */
    rcu_periph_clock_disable(RCU_UART4);

    ble_dma_read_pos = 0;
}

// UART4 IDLE中断处理 (由 UART4_IRQHandler 调用)
void ble_uart_rx_irq_handler(void)
{
    /* IDLE中断: 一帧接收完成*/
    if(RESET != usart_interrupt_flag_get(UART4, USART_INT_FLAG_IDLE))
    {
		usart_flag_clear(UART4,USART_FLAG_IDLE);
        /* 读STAT+DATA清除IDLE标志 */
        usart_data_receive(UART4);
    }
    /* 清除错误标志 (防中断风暴) */
    if(RESET != usart_flag_get(UART4, USART_FLAG_ORERR))
        usart_flag_clear(UART4, USART_FLAG_ORERR);
    if(RESET != usart_flag_get(UART4, USART_FLAG_FERR))
        usart_flag_clear(UART4, USART_FLAG_FERR);
}

// DMA接收缓冲中可读字节数
uint16_t ble_uart_rx_available(void)
{
    uint16_t wp = ble_dma_write_pos();
    if(wp >= ble_dma_read_pos)
        return wp - ble_dma_read_pos;
    else
        return BLE_DMA_BUF_SIZE - ble_dma_read_pos + wp;
}

// 读取一个字节
uint8_t ble_uart_rx_read(void)
{
    uint8_t ch = ble_dma_buf[ble_dma_read_pos];
    ble_dma_read_pos = (ble_dma_read_pos + 1) % BLE_DMA_BUF_SIZE;
    return ch;
}

// 清空DMA接收缓冲
void ble_uart_rx_flush(void)
{
    ble_dma_read_pos = ble_dma_write_pos();
    /* clear error flags */
    if(RESET != usart_flag_get(UART4, USART_FLAG_ORERR))
        usart_flag_clear(UART4, USART_FLAG_ORERR);
    if(RESET != usart_flag_get(UART4, USART_FLAG_FERR))
        usart_flag_clear(UART4, USART_FLAG_FERR);
    if(RESET != usart_flag_get(UART4, USART_FLAG_IDLE))
        usart_data_receive(UART4);  /* clear IDLE */
}

#if LASER_CH4_ENABLED
// ==================== USART1 (CH4 sensor) ====================
volatile uint8_t  s_ch4_rx_buf[64];
static volatile uint16_t s_ch4_rx_len;
static volatile uint8_t  s_ch4_frame_ready;

void usart1_ch4_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART1);

    /* PA2 = USART1_TX (AF7), PA3 = USART1_RX (AF7) */
    gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_2 | GPIO_PIN_3);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2 | GPIO_PIN_3);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

    usart_deinit(USART1);
    usart_baudrate_set(USART1, 115200U);
    usart_parity_config(USART1, USART_PM_NONE);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);

    usart_enable(USART1);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);

    /* 先复位接收状态, 再使能中断, 避免 ISR 使用陈旧计数 */
    s_ch4_rx_len = 0;
    s_ch4_frame_ready = 0;

    /* 逐字节接收中断 + IDLE 帧结束中断。
     * 此前只使能 IDLE: GD32L23x USART 无接收 FIFO 时 RDR 仅缓存 1 字节,
     * 29 字节帧在 IDLE 触发前几乎全部溢出, 且 IDLE 处理又先读掉最后一字节,
     * 导致永远收不到完整帧 (ch4 parse 必然失败)。*/
    usart_interrupt_enable(USART1, USART_INT_RBNE);
    usart_interrupt_enable(USART1, USART_INT_IDLE);
    nvic_irq_enable(USART1_IRQn, 1);
}

void usart1_ch4_deinit(void)
{
    usart_interrupt_disable(USART1, USART_INT_RBNE);
    usart_interrupt_disable(USART1, USART_INT_IDLE);
    nvic_irq_disable(USART1_IRQn);
    usart_disable(USART1);
    rcu_periph_clock_disable(RCU_USART1);

    /* PA2/PA3 -> analog for low power */
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_2 | GPIO_PIN_3);
}

void usart1_ch4_irq_handler(void)
{
    /* 逐字节接收: RBNE 中断逐字节入库 (此前仅 IDLE, 帧内字节全部丢失) */
    if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE))
    {
        while(RESET != usart_flag_get(USART1, USART_FLAG_RBNE))
        {
            uint8_t ch = (uint8_t)usart_data_receive(USART1);
            if(s_ch4_rx_len < sizeof(s_ch4_rx_buf))
                s_ch4_rx_buf[s_ch4_rx_len++] = ch;
        }
    }
    /* 帧结束: IDLE 置位。
     * 必须用 usart_flag_clear() 写 USART_INTC 清 IDLE 标志——仅读 RDR
     * 不会清 IDLE, 会导致 IDLE 中断无限重入、主循环饿死
     * (实测卡死约180s后被 SWDT 复位)。 */
    if(RESET != usart_flag_get(USART1, USART_FLAG_IDLE))
    {
        usart_flag_clear(USART1, USART_FLAG_IDLE);
        /* 清 IDLE 后再收残留字节, 防止漏掉最后一位 */
        while(RESET != usart_flag_get(USART1, USART_FLAG_RBNE))
        {
            uint8_t ch = (uint8_t)usart_data_receive(USART1);
            if(s_ch4_rx_len < sizeof(s_ch4_rx_buf))
                s_ch4_rx_buf[s_ch4_rx_len++] = ch;
        }
        s_ch4_frame_ready = 1;
    }
    /* 溢出/帧错误清除, 防中断风暴 */
    if(RESET != usart_flag_get(USART1, USART_FLAG_ORERR))
        usart_flag_clear(USART1, USART_FLAG_ORERR);
    if(RESET != usart_flag_get(USART1, USART_FLAG_FERR))
        usart_flag_clear(USART1, USART_FLAG_FERR);
}

/* accessors for ch4_sensor.c */
uint8_t *ch4_get_rx_buf(void)    { return (uint8_t *)s_ch4_rx_buf; }
uint16_t ch4_get_rx_len(void)   { return s_ch4_rx_len; }
uint8_t  ch4_get_frame_ready(void) { return s_ch4_frame_ready; }
void     ch4_clear_frame_ready(void) { s_ch4_frame_ready = 0; s_ch4_rx_len = 0; }

#endif /* LASER_CH4_ENABLED */
