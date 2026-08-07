#include "usart.h"
#include "mygpio.h"

#include "delay.h"
#include "systick.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static void system_clock_reconfig(void);

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
// DMA循环模式自动搬运, 软件跟踪读指�?
#define LPUART_DMA_BUF_SIZE  1024
#define LPUART_DMA_CHANNEL   DMA_CH1

static uint8_t  lpuart_dma_buf[LPUART_DMA_BUF_SIZE];
static volatile uint16_t lpuart_dma_read_pos = 0;  // 软件读取位置

// IDLE帧检�? 用于OTA整帧接收
volatile uint8_t  lpuart_frame_ready = 0;     // 帧完成标�?IDLE中断置位)
volatile uint16_t lpuart_frame_end_pos = 0;   // 帧结束DMA写位�?

// 获取DMA当前写入位置(DMA已写入字节数)
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

    // 循环模式: 写到末尾后自动回到开�?
    dma_circulation_enable(LPUART_DMA_CHANNEL);
    dma_channel_enable(LPUART_DMA_CHANNEL);

    // LPUART使能DMA接收请求
    lpuart_dma_receive_config(LPUART0, LPUART_RECEIVE_DMA_ENABLE);

    lpuart_dma_read_pos = 0;
}

// 禁用DMA接收(深休眠前调用)
void lpuart_dma_rx_disable(void)
{
    dma_channel_disable(LPUART_DMA_CHANNEL);
    lpuart_dma_receive_config(LPUART0, LPUART_RECEIVE_DMA_DISABLE);
}

// LPUART IRQ处理: IDLE帧检�?+ 错误清除
void lpuart_rx_irq_handler(void)
{
    // IDLE空闲检�? 一帧数据接收完�?
    if(SET == lpuart_interrupt_flag_get(LPUART0, LPUART_INT_FLAG_IDLE))
    {
        lpuart_flag_clear(LPUART0, LPUART_FLAG_IDLE);
        lpuart_frame_end_pos = lpuart_dma_write_pos();
        lpuart_frame_ready = 1;
    }
    // 防御性兜底: 一次性清除所有错误标志(ORE/FE/NE/PE),
    // 防止未处理的错误源(如 Noise Error)导致中断风暴
    LPUART_INTC(LPUART0) = LPUART_INTC_PEC | LPUART_INTC_FEC
                          | LPUART_INTC_NEC | LPUART_INTC_OREC;
}

// 清空DMA接收缓冲(读指针跟到写指针)
void lpuart_rx_flush(void)
{
    lpuart_dma_read_pos = lpuart_dma_write_pos();
    lpuart_frame_ready = 0;
    // 清除错误标志
    if(SET == lpuart_flag_get(LPUART0, LPUART_FLAG_ORERR))
        lpuart_flag_clear(LPUART0, LPUART_FLAG_ORERR);
    if(SET == lpuart_flag_get(LPUART0, LPUART_FLAG_FERR))
        lpuart_flag_clear(LPUART0, LPUART_FLAG_FERR);
    // 清除IDLE标志(防止残留触发)
    if(SET == lpuart_flag_get(LPUART0, LPUART_FLAG_IDLE))
        lpuart_flag_clear(LPUART0, LPUART_FLAG_IDLE);
}

// 等待一帧完�?IDLE检�?, 返回帧长�? 0=超时
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

// 从DMA缓冲拷贝数据到用户缓�?妄帧读取, 跟新read_pos)
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
         // 【LPUART0 专用路径：从 DMA 缓冲区读取】
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
    // 其他UART: 轮询硬件寄存�?
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
    // TX后防御性恢�? 清除错误标志(DMA模式无需重新使能中断)
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


// ===================== 【封�?】LPUART唤醒初始�?=====================
void lpuart_wakeup_init(void)
{
    // 时钟源已在 com_lpuart_init() 中配置为 IRC16MDIV, 此处不再重复设置

    // 唤醒中断 + EXTI配置
    nvic_irq_enable(LPUART_WKUP_IRQn, 0);
    exti_init(EXTI_28, EXTI_INTERRUPT, EXTI_TRIG_RISING);

    // 起始位唤醒
    lpuart_wakeup_mode_config(LPUART0, LPUART_WUM_STARTB);
    lpuart_enable(LPUART0);

    while(RESET == lpuart_flag_get(LPUART0, LPUART_FLAG_REA));
    while(SET == lpuart_flag_get(LPUART0, LPUART_FLAG_BSY));

    lpuart_wakeup_enable(LPUART0);
    lpuart_interrupt_enable(LPUART0, LPUART_INT_WU);
}

// ===================== 【封�?】进入深休眠 =====================
// 支持四种唤醒�? LPUART (counter0) / RTC闹钟 (rtc_alarm_flag) / 按键 (button_wakeup_flag) / 4G RI (ri_wakeup_flag)
void enter_deepsleep_mode(uint8_t lpuart_enabled)
{
    extern volatile uint8_t rtc_alarm_flag;
    extern volatile uint8_t button_wakeup_flag;
    extern volatile uint8_t ri_wakeup_flag;

    counter0 = 0x00;
    rtc_alarm_flag = 0;
    button_wakeup_flag = 0;
    ri_wakeup_flag = 0;

    // 软件看门狗: 进入深休眠前必须关闭 (深休眠期间 SysTick 停摆, 否则唤醒后计数错乱)
    // 所有进入深休眠的路径都经过本函数, 此处统一关闭可保证休眠期不被误复位
    swdt_disable();

    // 配置未使�?GPIO �?ANALOG 以极致降低漏�?
    gpio_enter_low_power(lpuart_enabled);

    rcu_periph_clock_enable(RCU_PMU);
    pmu_to_deepsleepmode(PMU_LDNPDSP_LOWDRIVE, WFI_CMD, PMU_DEEPSLEEP);

    // 等待任意唤醒�?
    while((0x00 == counter0) && (0 == rtc_alarm_flag) && (0 == button_wakeup_flag) && (0 == ri_wakeup_flag));
}

// ===================== 【封�?】唤醒后系统重新初始�?=====================
void system_wakeup_reinit(void)
{
    // 恢复时钟 + 滴答定时器 + TIMER1（必须最先执行）
    system_clock_reconfig();
    systick_config();
    my_systick_config();
    timer1_delay_init(); // 深休眠后 APB1 停摆, 必须重新启动 TIMER1 计数器

    // 无论哪种唤醒源，都必须关闭LPUART唤醒模式和中断 + RI EXTI
    // 否则LPUART唤醒中断持续触发，无法再次进入深休眠
    lpuart_wakeup_disable(LPUART0);
    lpuart_interrupt_disable(LPUART0, LPUART_INT_WU);
    exti_flag_clear(EXTI_28);

    // 关闭RI引脚EXTI (防止唤醒后RI持续低电平重复触发)
    ri_exti_disable();

    // LPUART唤醒时：消费触发唤醒的字�?深休眠期间DMA未运�? 字节在RDATA�?
    if(counter0 != 0x00)
    {
        if(RESET != lpuart_flag_get(LPUART0, LPUART_FLAG_RBNE))
            lpuart_data_receive(LPUART0);
        lpuart_receive_config(LPUART0, LPUART_RECEIVE_ENABLE);
        while(RESET == lpuart_flag_get(LPUART0, LPUART_FLAG_TC));
        lpuart_disable(LPUART0);
    }
}

// ===================== LPUART初始化=====================
void com_lpuart_init(void)
{
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_LPUART);

    gpio_af_set(GPIOD, GPIO_AF_8, GPIO_PIN_8);   /* PD8 = LPUART0_TX → LP_TXD → 4G模块 */
    gpio_af_set(GPIOD, GPIO_AF_8, GPIO_PIN_9);   /* PD9 = LPUART0_RX ← LP_RXD ← 4G模块 */

    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_8);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_8);
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_9);

    lpuart_deinit(LPUART0);
    // 时钟源必须在 UE=0 (deinit后) 且 baudrate_set 之前配置,
    // 确保 BRR 按 IRC16M 时钟正确计算, 且不违反"UE=1时禁止切换时钟"的硬件约束
    rcu_lpuart_clock_config(RCU_LPUARTSRC_IRC16MDIV);
    lpuart_word_length_set(LPUART0, LPUART_WL_8BIT);
    lpuart_stop_bit_set(LPUART0, LPUART_STB_1BIT);
    lpuart_parity_config(LPUART0, LPUART_PM_NONE);
    lpuart_baudrate_set(LPUART0, 115200U);
    lpuart_receive_config(LPUART0, LPUART_RECEIVE_ENABLE);
    lpuart_transmit_config(LPUART0, LPUART_TRANSMIT_ENABLE);
    lpuart_enable(LPUART0);

    // DMA循环接收初始�?替代RBNE中断, 零中断高吞吐)
    lpuart_dma_rx_init();
    // IDLE中断: 帧结束检测(OTA整帧接收)
    // 注: 不启用 LPUART_INT_ERR(EIE), DMA模式下错误由 flush/recover 显式清除,
    //     EIE 会使 NE(Noise Error) 等触发中断但 ISR 未必全部处理, 易引发中断风暴
    lpuart_interrupt_enable(LPUART0, LPUART_INT_IDLE);
    nvic_irq_enable(LPUART_IRQn, 1);
}

// ===================== USART0 中断接收环形缓冲 =====================
#define DEBUG_RX_BUF_SIZE  256
static volatile char    drx_buf[DEBUG_RX_BUF_SIZE];
static volatile uint16_t drx_head = 0;   // 写入位置 (中断�?
static volatile uint16_t drx_tail = 0;   // 读取位置 (主循环读)
static volatile uint8_t  drx_line_ready = 0; // 有完整行标记

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
        if(next != drx_tail)  // 缓冲区未�?
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

    // 等待完整行或超时
    uint32_t t0 = trea_millis();
    while(!drx_line_ready)
    {
        if((trea_millis() - t0) >= timeout_ms)
        {
            buf[0] = '\0';
            return 0;
        }
    }

    // 保存读指针，若未找到完整行则回滚，避免数据丢�?
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
                // 消费掉连续的 \r\n
                while(drx_tail != drx_head)
                {
                    char peek = drx_buf[drx_tail];
                    if(peek == '\r' || peek == '\n')
                        drx_tail = (drx_tail + 1) % DEBUG_RX_BUF_SIZE;
                    else
                        break;
                }
                // 检查是否还有更多行
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
            saved_tail = drx_tail;  // 空行已消费，更新保存�?
            continue;  // 跳过行首的空 \r\n
        }
        buf[idx++] = ch;
    }

    // 没找到完整行(缺少结尾\r\n)，回滚读指针，数据保留在缓冲�?
    drx_tail = saved_tail;
    drx_line_ready = 0;
    buf[0] = '\0';
    return 0;
}

// ===================== UART4 蓝牙 DMA接收 =====================
// DMA循环模式 + IDLE中断, 参照 LPUART0 DMA 方案
#define BLE_DMA_BUF_SIZE     1024
#define BLE_DMA_CHANNEL      DMA_CH2

static uint8_t  ble_dma_buf[BLE_DMA_BUF_SIZE];
static volatile uint16_t ble_dma_read_pos = 0;

// 获取DMA当前写入位置
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

    /* UART4 配置 */
    usart_deinit(UART4);
    usart_baudrate_set(UART4, 115200U);
    usart_word_length_set(UART4, USART_WL_8BIT);
    usart_stop_bit_set(UART4, USART_STB_1BIT);
    usart_parity_config(UART4, USART_PM_NONE);
    usart_receive_config(UART4, USART_RECEIVE_ENABLE);
    usart_transmit_config(UART4, USART_TRANSMIT_ENABLE);
    usart_enable(UART4);

    /* DMA 循环接收 */
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

    /* IDLE中断: 帧结束检测 */
    usart_interrupt_enable(UART4, USART_INT_IDLE);
    nvic_irq_enable(UART4_IRQn, 1);
	
    /* 复位读指针 */
    ble_dma_read_pos = 0;
	
}

// UART4 反初始化 (关闭蓝牙时调用)
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

    /* GPIO恢复为模拟输入 (低功耗) */
    gpio_mode_set(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_3 | GPIO_PIN_4);

    /* 关闭UART4时钟 */
    rcu_periph_clock_disable(RCU_UART4);

    ble_dma_read_pos = 0;
}

// UART4 IDLE中断处理 (在 UART4_IRQHandler 中调用)
void ble_uart_rx_irq_handler(void)
{
    /* IDLE检测: 一帧数据接收完毕 */
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

// DMA缓冲区可读字节数
uint16_t ble_uart_rx_available(void)
{
    uint16_t wp = ble_dma_write_pos();
    if(wp >= ble_dma_read_pos)
        return wp - ble_dma_read_pos;
    else
        return BLE_DMA_BUF_SIZE - ble_dma_read_pos + wp;
}

// 读取一字节
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
    /* 清除可能残留的错误标志 */
    if(RESET != usart_flag_get(UART4, USART_FLAG_ORERR))
        usart_flag_clear(UART4, USART_FLAG_ORERR);
    if(RESET != usart_flag_get(UART4, USART_FLAG_FERR))
        usart_flag_clear(UART4, USART_FLAG_FERR);
    if(RESET != usart_flag_get(UART4, USART_FLAG_IDLE))
        usart_data_receive(UART4);  /* 清除IDLE */
}

// ===================== 时钟配置 =====================
static void system_clock_reconfig(void)
{
    RCU_CTL |= RCU_CTL_IRC16MEN;
    while(rcu_osci_stab_wait(RCU_IRC16M) != SUCCESS);

    rcu_ahb_clock_config(RCU_AHB_CKSYS_DIV1);
    rcu_apb1_clock_config(RCU_APB1_CKAHB_DIV1);
    rcu_apb2_clock_config(RCU_APB2_CKAHB_DIV1);

    RCU_CTL &= ~RCU_CTL_PLLEN;
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_IRC16M;
}
