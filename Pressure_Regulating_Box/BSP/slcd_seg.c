/*!
    \file    slcd_seg.c
    \brief   source of the slcd segment driver

    \version 2025-08-21, V2.5.0, demo for GD32L23x
*/

/*
    Copyright (c) 2025, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "slcd_seg.h"
#include "gd32l23x.h"


/*!
    \brief      configure SLCD low-speed clock source
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void slcd_clock_source_config(void)
{
    /* enable PMU clock */
    rcu_periph_clock_enable(RCU_PMU);
    /* PMU backup domain write enable */
    pmu_backup_write_enable();

    /* reset backup domain before changing RTC/SLCD clock source */
    rcu_bkp_reset_enable();
    rcu_bkp_reset_disable();

    /* enable internal 32K low-speed RC oscillator */
    rcu_osci_on(RCU_IRC32K);
    /* wait for IRC32K stabilization flag */
    rcu_osci_stab_wait(RCU_IRC32K);
    /* configure RTC/SLCD clock source selection */
    rcu_rtc_clock_config(RCU_RTCSRC_IRC32K);

    /* enable RTC clock because SLCD clock is derived from the RTC clock source */
    rcu_periph_clock_enable(RCU_RTC);
}


/* digit SLCD DATA buffer */
uint8_t digit[4];

/* ====== 影子缓冲区 (消除闪屏) ====== */
static uint32_t s_shadow[4] = {0, 0, 0, 0};
static uint8_t  s_batch = 0;  /* 1 = 批量模式, 不自动刷新 */

/* 数字位掩码: 5个位置的 sa/sb SEG bit */
#define DIGIT_SA_MASK  ((1UL<<5)|(1UL<<10)|(1UL<<12)|(1UL<<14)|(1UL<<24))
#define DIGIT_SB_MASK  ((1UL<<6)|(1UL<<11)|(1UL<<13)|(1UL<<15)|(1UL<<25))
#define DIGIT_ALL_MASK (DIGIT_SA_MASK | DIGIT_SB_MASK)

/* 页面图标掩码 (进口S5/出口S6/℃S12/kPaS13 等切页时需清除的图标) */
/* S5:DATA0-bit26, S6:DATA0-bit27, S13:DATA0-bit29 */
#define PAGE_ICON_DATA0_MASK ((1UL<<26)|(1UL<<27)|(1UL<<29))
/* S12:DATA3-bit28 */
#define PAGE_ICON_DATA3_MASK (1UL<<28)

static void slcd_auto_flush(void)
{
    if(!s_batch) slcd_seg_flush();
}

/* table of the digit code for SLCD */
__I uint32_t numbertable[10] = {
    /*
     * XD-H2074A digit bit map:
     * bit0 = A, bit1 = B, bit2 = F, bit3 = G,
     * bit4 = E, bit5 = C, bit6 = D, bit7 = unused.
     */
    /* 0     1     2     3     4 */
    0x77, 0x22, 0x5B, 0x6B, 0x2E,
    /* 5     6     7     8     9 */
    0x6D, 0x7D, 0x23, 0x7F, 0x6F
};

/*
 * Display position (1..5) to SLCD SEG pair mapping.
 * Each digit uses two glass SEG lines. The mapping from glass SEG to SLCD SEG:
 *   glass SEG0->SLCD SEG6,  SEG1->SEG10, SEG2->SEG11, SEG3->SEG12,
 *   SEG4->SEG13, SEG5->SEG14, SEG6->SEG15, SEG7->SEG24,
 *   SEG8->SEG25, SEG9->SEG26, SEG10->SEG27, SEG11->SEG17,
 *   SEG12->SEG28, SEG13->SEG29
 *
 * Position 1: glass SEG0/SEG1 -> SLCD SEG5/SEG6
 * Position 2: glass SEG2/SEG3 -> SLCD SEG10/SEG11
 * Position 3: glass SEG4/SEG5 -> SLCD SEG12/SEG13
 * Position 4: glass SEG6/SEG7 -> SLCD SEG14/SEG15
 * Position 5: glass SEG8/SEG9 -> SLCD SEG24/SEG25
 */
static const uint8_t pos_seg_a[6] = {0U, 5U, 10U, 12U, 14U, 24U};
static const uint8_t pos_seg_b[6] = {0U, 6U, 11U, 13U, 15U, 25U};

typedef struct {
    uint8_t data_index;  /* SLCD DATA register index (0,1,2,3 -> glass COM1..COM4) */
    uint8_t seg;         /* SLCD SEG bit index */
} slcd_point_struct;

static void digit_to_code(uint8_t c);
static uint8_t slcd_seg_char_to_code(char ch);
static void slcd_clock_source_config(void);
static void slcd_gpio_config(void);
static void slcd_seg_digit_write(uint8_t ch, uint8_t position, slcd_display_enum type);
static void slcd_seg_digit8_write(uint8_t position);
static void slcd_seg_point_set(uint8_t data_index, uint8_t seg, FlagStatus state);

/*
 * Icon table: maps each icon to (SLCD DATA register index, SLCD SEG bit).
 *
 * SLCD DATA -> SLCD COM -> Glass COM (1/8 duty, COM4-COM7):
 *   DATA0 -> COM0 -> Glass COM1 (D, icons)
 *   DATA1 -> COM1 -> Glass COM2 (E, C)
 *   DATA2 -> COM2 -> Glass COM3 (F, G)
 *   DATA3 -> COM3 -> Glass COM4 (A, B)
 */
static const slcd_point_struct icon_table[] = {
    /* SLCD_ICON_T1  */ {0U, 6U},   /* Glass COM1 x SEG1 -> DATA4, seg10 */
    /* SLCD_ICON_P2  */ {0U, 11U},  /* Glass COM1 x SEG3 -> DATA4, seg12 */
    /* SLCD_ICON_P3  */ {0U, 13U},  /* Glass COM1 x SEG5 -> DATA4, seg14 */
    /* SLCD_ICON_P4  */ {0U, 15U},  /* Glass COM1 x SEG7 -> DATA4, seg24 */
    /* SLCD_ICON_S14 */ {0U, 25U},  /* Glass COM1 x SEG9 -> DATA4, seg26 */
    /* SLCD_ICON_S1  */ {3U, 26U},  /* Glass COM4 x SEG10 -> DATA7, seg27 */
    /* SLCD_ICON_S2  */ {2U, 26U},  /* Glass COM3 x SEG10 -> DATA6, seg27 */
    /* SLCD_ICON_S3  */ {1U, 26U},  /* Glass COM2 x SEG10 -> DATA5, seg27 */
    /* SLCD_ICON_S5  */ {0U, 26U},  /* Glass COM1 x SEG10 -> DATA4, seg27 */
    /* SLCD_ICON_S7  */ {3U, 27U},  /* Glass COM4 x SEG11 -> DATA7, seg17 */
    /* SLCD_ICON_LY  */ {2U, 27U},  /* Glass COM3 x SEG11 -> DATA6, seg17 */
    /* SLCD_ICON_S11 */ {1U, 27U},  /* Glass COM2 x SEG11 -> DATA5, seg17 */
    /* SLCD_ICON_S6  */ {0U, 27U},  /* Glass COM1 x SEG11 -> DATA4, seg17 */
    /* SLCD_ICON_S12 */ {3U, 28U},  /* Glass COM4 x SEG12 -> DATA7, seg28 */
    /* SLCD_ICON_S8  */ {2U, 28U},  /* Glass COM3 x SEG12 -> DATA6, seg28 */
    /* SLCD_ICON_S9  */ {1U, 28U},  /* Glass COM2 x SEG12 -> DATA5, seg28 */
    /* SLCD_ICON_S10 */ {0U, 28U},  /* Glass COM1 x SEG12 -> DATA4, seg28 */
    /* SLCD_ICON_T2  */ {3U, 29U},  /* Glass COM4 x SEG13 -> DATA7, seg29 */
    /* SLCD_ICON_T3  */ {2U, 29U},  /* Glass COM3 x SEG13 -> DATA6, seg29 */
    /* SLCD_ICON_T4  */ {1U, 29U},  /* Glass COM2 x SEG13 -> DATA5, seg29 */
    /* SLCD_ICON_S13 */ {0U, 29U}   /* Glass COM1 x SEG13 -> DATA4, seg29 */
};

/*!
    \brief      convert digit to SLCD code
    \param[in]  the digit to write
    \param[out] none
    \retval     none
*/
static void digit_to_code(uint8_t c)
{
    uint8_t ch = 0;

    /* the *c is a number */
    if(c < 10) {
        ch = numbertable[c];
    }

    digit[0] = (uint8_t)((ch) & 0x03);
    digit[1] = (uint8_t)((ch >> 2) & 0x03);
    digit[2] = (uint8_t)((ch >> 4) & 0x03);
    digit[3] = (uint8_t)((ch >> 6) & 0x03);
}
/*!
    \brief      convert ASCII character to XD-H2074A 7-segment code
    \param[in]  ch: character to display
    \param[out] none
    \retval     7-segment code, bit0=A bit1=B bit2=F bit3=G bit4=E bit5=C bit6=D
*/
static uint8_t slcd_seg_char_to_code(char ch)
{
    if((ch >= '0') && (ch <= '9')) {
        return (uint8_t)numbertable[ch - '0'];
    }

    if((ch >= 'a') && (ch <= 'z')) {
        ch = (char)(ch - 'a' + 'A');
    }

    switch(ch) {
    case 'A':
        return 0x3FU;
    case 'B':
        return 0x7CU;
    case 'C':
        return 0x55U;
    case 'D':
        return 0x7AU;
    case 'E':
        return 0x5DU;
    case 'F':
        return 0x1DU;
    case 'G':
        return 0x75U;
    case 'H':
        return 0x3EU;
    case 'I':
        return 0x22U;
    case 'J':
        return 0x72U;
    case 'L':
        return 0x54U;
    case 'N':
        return 0x38U;
    case 'O':
        return 0x77U;
    case 'P':
        return 0x1FU;
    case 'R':
        return 0x18U;
    case 'S':
        return 0x6DU;
    case 'T':
        return 0x5CU;
    case 'U':
        return 0x76U;
    case 'Y':
        return 0x6EU;
    case '-':
        return 0x08U;
    case '_':
        return 0x40U;
    case ' ':
    default:
        return 0x00U;
    }
}

/*!
    \brief      initialize the GPIO port of SLCD peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void slcd_gpio_config(void)
{
    /* SLCD pin mapping (new wiring, 1/8 duty using COM4-COM7):
     *   PA8 AF3->COM0, PA9 AF3->COM1, PA10 AF3->COM2
     *   PB1 AF3 -> SEG6,  PB10-PB15 AF3 -> SEG10-SEG15
     *   PC6  AF3 -> SEG24, PC7 AF3 -> SEG25, PC8 AF3 -> SEG26, PC9 AF3 -> SEG27
     *   
     *   PD2 unused, PD6 ANALOG (SLCD external voltage)
     *   PD6  ANALOG (SLCD external voltage source)
     */

    /* enable GPIO clocks */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);

    /* configure GPIOA: PA8(COM0), PA9(COM1), PA10(COM2) */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,
                            GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
    gpio_af_set(GPIOA, GPIO_AF_3,
                GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);

    /* configure GPIOB: PB0(SEG5), PB1(SEG6), PB9(COM3), PB10-PB15(SEG10-15) */
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_9 |
                  GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                  GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,
                            GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_9 |
                            GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                            GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    gpio_af_set(GPIOB, GPIO_AF_3,
                GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_9 |
                GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
                GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

    /* configure GPIOC: PC6-PC11 (SEG24-SEG29) */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 |
                  GPIO_PIN_10 | GPIO_PIN_11);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,
                            GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 |
                            GPIO_PIN_10 | GPIO_PIN_11);
    gpio_af_set(GPIOC, GPIO_AF_3,
                GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 |
                GPIO_PIN_10 | GPIO_PIN_11);

    /* configure PD6 as analog for SLCD external voltage source */
    gpio_mode_set(GPIOD, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_6);
}
/*!
    \brief      configure SLCD peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void slcd_seg_configuration(void)
{
    volatile uint16_t i;

    /* SLCD shares the RTC clock source (LXTAL already configured by rtc_pre_config).
     * Do NOT call slcd_clock_source_config() here because it resets the backup domain
     * which would destroy the RTC configuration.
     * Only ensure RTC clock is enabled (SLCD clock is derived from RTC clock source). */
    rcu_periph_clock_enable(RCU_PMU);
    pmu_backup_write_enable();
    rcu_periph_clock_enable(RCU_RTC);

    /* configure the SLCD GPIO pins */
    slcd_gpio_config();

    /* enable the clock of SLCD */
    rcu_periph_clock_enable(RCU_SLCD);
    /* wait 2 RTC clock to write SLCD register */
    for(i = 0; i < 1000; i++);

    slcd_deinit();
    /*
     * Clock: f_frame = f_IRC32K / (duty x prescaler x divider)
     * 1/4 duty: 32000 / (4 x 2 x 16) = 250 Hz (flicker-free)
     */
    slcd_clock_config(SLCD_PRESCALER_1, SLCD_DIVIDER_16);
    /* SLCD bias voltage select */
    slcd_bias_voltage_select(SLCD_BIAS_1_3);
    /* SLCD duty cycle select: 1/4 duty (COM0-COM3 active) */
    slcd_duty_select(SLCD_DUTY_1_4);
    /* Remap SEG28-SEG31 as segment outputs (default: reserved for COM4-COM7) */
    slcd_com_seg_remap(ENABLE);
    /* SLCD voltage source select */
    slcd_voltage_source_select(SLCD_VOLTAGE_EXTERNAL);
    /* SLCD dead time duration config */
    slcd_dead_time_config(SLCD_DEADTIME_PERIOD_0);
    /* enable the permanent high drive */
    slcd_high_drive_config(ENABLE);//DISABLE

    /* wait for SLCD CFG register synchronization */
    while(!slcd_flag_get(SLCD_FLAG_SYN));
    /* enable SLCD interface */
    slcd_enable();
    /* wait for SLCD controller on flag */
    while(!slcd_flag_get(SLCD_FLAG_ON));

    /* 初始化影子缓冲区: 同步硬件寄存器当前值
     * COM重映射: SLCD COM0→Glass COM4, COM1→COM3, COM2→COM2, COM3→COM1 */
    s_shadow[0] = SLCD_DATA3;
    s_shadow[1] = SLCD_DATA2;
    s_shadow[2] = SLCD_DATA1;
    s_shadow[3] = SLCD_DATA0;
}

void slcd_seg_flush(void)
{
    while(slcd_flag_get(SLCD_FLAG_UPR));
    /* COM重映射: Glass COM4<->COM1, COM3<->COM2
     * SLCD COM0→Glass COM4(shadow[3]), COM1→COM3(shadow[2]),
     * COM2→COM2(shadow[1]), COM3→COM1(shadow[0]) */
    SLCD_DATA0 = s_shadow[3];
    SLCD_DATA1 = s_shadow[2];
    SLCD_DATA2 = s_shadow[1];
    SLCD_DATA3 = s_shadow[0];
    slcd_data_update_request();
}

void slcd_seg_batch_begin(void)
{
    s_batch = 1;
}

void slcd_seg_batch_end(void)
{
    s_batch = 0;
    slcd_seg_flush();
}

void slcd_seg_clear_digits(void)
{
    /* 清除数字段码(sa+sb) + 小数点 + 页面切换图标, 保留常驻图标 */
    s_shadow[0] &= ~(DIGIT_ALL_MASK | PAGE_ICON_DATA0_MASK);
    s_shadow[1] &= ~DIGIT_ALL_MASK;
    s_shadow[2] &= ~DIGIT_ALL_MASK;
    s_shadow[3] &= ~(DIGIT_ALL_MASK | PAGE_ICON_DATA3_MASK);
}

/*!
    \brief      write one digit to the SLCD DATA register
    \param[in]  ch: the digit to write
    \param[in]  position: position in the SLCD of the digit to write
    \param[out] none
    \retval     none
*/
void slcd_seg_digit_display(uint8_t ch, uint8_t position)
{
    /* SLCD write a char to shadow */
    slcd_seg_digit_write(ch, position, INTEGER);

    slcd_auto_flush();
}

/*!
    \brief      write a integer(6 digits) to SLCD DATA register
    \param[in]  num: number to send to SLCD(0-999999)
    \param[out] none
    \retval     none
*/
void slcd_seg_number_display(uint32_t num)
{
    uint8_t length;
    uint8_t i = 0U, digit_value = 0U;

    slcd_seg_clear_digits();

    if(num >= 100000U) {
        length = 6U;
    } else if(num >= 10000U) {
        length = 5U;
    } else if(num >= 1000U) {
        length = 4U;
    } else if(num >= 100U) {
        length = 3U;
    } else if(num >= 10U) {
        length = 2U;
    } else {
        length = 1U;
    }

    while(i < length) {
        digit_value = (uint8_t)(num % 10U);
        slcd_seg_digit_write(digit_value, (uint8_t)(6U - i), INTEGER);
        i++;
        num /= 10U;
    }

    slcd_auto_flush();
}

/*!
    \brief      write a integer(max 5 digits) with fixed width to SLCD DATA register
    \param[in]  num: number to send to SLCD(0-99999)
    \param[in]  width: display width(1-5)
    \param[in]  leading_zero: 1 to show leading zeros, 0 to hide
    \param[out] none
    \retval     none
*/
void slcd_seg_number_width_display(uint32_t num, uint8_t width, uint8_t leading_zero)
{
    uint8_t i = 0U, digit_value = 0U;
    uint32_t divisor = 1U;

    if((width == 0U) || (width > 5U)) {
        return;
    }

    for(i = 1U; i < width; i++) {
        divisor *= 10U;
    }

    slcd_seg_clear_digits();

    for(i = 0U; i < width; i++) {
        digit_value = (uint8_t)((num / divisor) % 10U);
        if((digit_value == 0U) && (leading_zero == 0U) && (i < (width - 1U)) && (num < divisor)) {
        } else {
            slcd_seg_digit_write(digit_value, (uint8_t)(5U - width + i + 1U), INTEGER);
        }
        divisor /= 10U;
    }

    slcd_auto_flush();
}
/*!
    \brief      display raw 7-segment code at position 1..5
    \param[in]  code: bit0=A bit1=B bit2=F bit3=G bit4=E bit5=C bit6=D
    \param[in]  position: display position, 1 is leftmost, 5 is rightmost
    \param[out] none
    \retval     none
*/
void slcd_seg_code_display(uint8_t code, uint8_t position)
{
    uint8_t sa, sb;
    uint32_t mask;

    if((position < 1U) || (position > 5U)) {
        return;
    }

    sa = pos_seg_a[position];
    sb = pos_seg_b[position];
    mask = (uint32_t)((1UL << sa) | (1UL << sb));

    /* Clear both SEG bits on shadow registers */
    s_shadow[0] &= ~mask;
    s_shadow[1] &= ~mask;
    s_shadow[2] &= ~mask;
    s_shadow[3] &= ~mask;

    if(code & 0x01U) { s_shadow[3] |= (uint32_t)(1UL << sa); } /* A */
    if(code & 0x02U) { s_shadow[3] |= (uint32_t)(1UL << sb); } /* B */
    if(code & 0x04U) { s_shadow[2] |= (uint32_t)(1UL << sa); } /* F */
    if(code & 0x08U) { s_shadow[2] |= (uint32_t)(1UL << sb); } /* G */
    if(code & 0x10U) { s_shadow[1] |= (uint32_t)(1UL << sa); } /* E */
    if(code & 0x20U) { s_shadow[1] |= (uint32_t)(1UL << sb); } /* C */
    if(code & 0x40U) { s_shadow[0] |= (uint32_t)(1UL << sa); } /* D */

    slcd_auto_flush();
}

/*!
    \brief      turn decimal point after position 1..5 on or off
    \param[in]  position: display position before decimal point
    \param[in]  state: SET to turn on, RESET to turn off
    \param[out] none
    \retval     none
*/
void slcd_seg_decimal_display(uint8_t position, FlagStatus state)
{
    static const slcd_icon_enum point_icon[5] = {
        SLCD_ICON_T1, SLCD_ICON_P2, SLCD_ICON_P3, SLCD_ICON_P4, SLCD_ICON_S14
    };

    if((position == 0U) || (position > 5U)) {
        return;
    }

    slcd_seg_icon_display(point_icon[position - 1U], state);
}

/*!
    \brief      display a 5-position string
    \param[in]  str: supports 0-9, letters, '-', '_' and decimal point
    \param[out] none
    \retval     none
*/
void slcd_seg_string_display(const char *str)
{
    uint8_t position = 1U;
    uint8_t last_position = 0U;
    uint8_t was_batch = s_batch;
    char ch;

    if(str == 0) {
        return;
    }

    s_batch = 1;  /* 临时进入批量模式, 避免中间刷新 */
    slcd_seg_clear_digits();

    if(*str == '-') {
        slcd_seg_code_display(slcd_seg_char_to_code('-'), 1U);
        position = 2U;
        last_position = 1U;
        str++;
    } else if((*str >= '0') && (*str <= '9')) {
        position = 2U;
    }

    while((*str != '\0') && (position <= 5U)) {
        ch = *str++;

        if(ch == '.') {
            if(last_position != 0U) {
                slcd_seg_decimal_display(last_position, SET);
            }
            continue;
        }

        slcd_seg_code_display(slcd_seg_char_to_code(ch), position);
        last_position = position;
        position++;
    }

    s_batch = was_batch;
    slcd_auto_flush();
}

/*!
    \brief      display a 5-position string atomically (single update, no flicker)
    \param[in]  str: supports 0-9, letters, '-', '_' and decimal point
    \param[out] none
    \retval     none
*/
void slcd_seg_string_atomic_display(const char *str)
{
    uint8_t position = 1U;
    uint8_t last_position = 0U;
    uint8_t was_batch = s_batch;
    char ch;
    uint8_t code, sa, sb;
    uint32_t mask;

    if(str == 0) {
        return;
    }

    s_batch = 1;  /* 临时进入批量模式 */
    slcd_seg_clear_digits();

    if(*str == '-') {
        code = slcd_seg_char_to_code('-');
        sa = pos_seg_a[1U]; sb = pos_seg_b[1U];
        mask = (uint32_t)((1UL << sa) | (1UL << sb));
        s_shadow[0] &= ~mask; s_shadow[1] &= ~mask;
        s_shadow[2] &= ~mask; s_shadow[3] &= ~mask;
        if(code & 0x01U) { s_shadow[3] |= (uint32_t)(1UL << sa); }
        if(code & 0x02U) { s_shadow[3] |= (uint32_t)(1UL << sb); }
        if(code & 0x04U) { s_shadow[2] |= (uint32_t)(1UL << sa); }
        if(code & 0x08U) { s_shadow[2] |= (uint32_t)(1UL << sb); }
        if(code & 0x10U) { s_shadow[1] |= (uint32_t)(1UL << sa); }
        if(code & 0x20U) { s_shadow[1] |= (uint32_t)(1UL << sb); }
        if(code & 0x40U) { s_shadow[0] |= (uint32_t)(1UL << sa); }
        position = 2U;
        last_position = 1U;
        str++;
    } else if((*str >= '0') && (*str <= '9')) {
        position = 2U;
    }

    while((*str != '\0') && (position <= 5U)) {
        ch = *str++;

        if(ch == '.') {
            if(last_position >= 1U && last_position <= 5U) {
                slcd_seg_point_set(icon_table[last_position - 1U].data_index,
                                   icon_table[last_position - 1U].seg, SET);
            }
            continue;
        }

        code = slcd_seg_char_to_code(ch);
        sa = pos_seg_a[position]; sb = pos_seg_b[position];
        mask = (uint32_t)((1UL << sa) | (1UL << sb));
        s_shadow[0] &= ~mask; s_shadow[1] &= ~mask;
        s_shadow[2] &= ~mask; s_shadow[3] &= ~mask;
        if(code & 0x01U) { s_shadow[3] |= (uint32_t)(1UL << sa); }
        if(code & 0x02U) { s_shadow[3] |= (uint32_t)(1UL << sb); }
        if(code & 0x04U) { s_shadow[2] |= (uint32_t)(1UL << sa); }
        if(code & 0x08U) { s_shadow[2] |= (uint32_t)(1UL << sb); }
        if(code & 0x10U) { s_shadow[1] |= (uint32_t)(1UL << sa); }
        if(code & 0x20U) { s_shadow[1] |= (uint32_t)(1UL << sb); }
        if(code & 0x40U) { s_shadow[0] |= (uint32_t)(1UL << sa); }
        last_position = position;
        position++;
    }

    s_batch = was_batch;
    slcd_auto_flush();
}

/*!
    \brief      write a float number(6 digits which has 2 decimal) to SLCD DATA register
    \param[in]  num: number to send to SLCD
    \param[out] none
    \retval     none
*/
void slcd_seg_floatnumber_display(float num)
{
    uint8_t i = 0x00, length, ch[6];
    uint32_t temp;

    temp = (uint32_t)(num * 100);
    ch[5] = temp / 100000;
    ch[4] = (temp % 100000) / 10000;
    ch[3] = (temp % 10000) / 1000;
    ch[2] = (temp % 1000) / 100;
    ch[1] = (temp % 100) / 10;
    ch[0] = temp % 10;

    if(ch[5]) {
        length = 6;
    } else if(ch[4]) {
        length = 5;
    } else if(ch[3]) {
        length = 4;
    } else {
        length = 3;
    }

    slcd_seg_clear_digits();

    /* send the string character one by one to shadow */
    while(i < length) {
        slcd_seg_digit_write(ch[i], 6 - i, FLOAT);
        i++;
    }

    slcd_auto_flush();
}
/*!
    \brief      write a digit to SLCD DATA register
    \param[in]  ch: the digit to write
    \param[in]  position: position in the SLCD of the digit, which can be 1..5
    \param[in]  type: the type of the data
    \param[out] none
    \retval     none
*/
static void slcd_seg_digit_write(uint8_t ch, uint8_t position, slcd_display_enum type)
{
    uint8_t sa, sb;
    uint32_t mask;

    if((position < 1U) || (position > 5U)) {
        return;
    }

    if(ch == 8U) {
        slcd_seg_digit8_write(position);
        return;
    }

    /* convert digit to 7-segment code stored in digit[0..3] */
    digit_to_code(ch);

    sa = pos_seg_a[position];
    sb = pos_seg_b[position];
    mask = (uint32_t)((1UL << sa) | (1UL << sb));

    /* clear both SEG bits on shadow registers */
    s_shadow[0] &= ~mask;
    s_shadow[1] &= ~mask;
    s_shadow[2] &= ~mask;
    s_shadow[3] &= ~mask;

    /*
     * 7-segment -> (COM, SEG) mapping:
     *   digit[0] bit0=A, bit1=B -> Glass COM1 -> DATA4
     *   digit[1] bit0=F, bit1=G -> Glass COM2 -> DATA5
     *   digit[2] bit0=E, bit1=C -> Glass COM3 -> DATA6
     *   digit[3] bit0=D           -> Glass COM4 -> DATA7
     */
    if(digit[0] & 0x01U) { s_shadow[3] |= (uint32_t)(1UL << sa); } /* A */
    if(digit[0] & 0x02U) { s_shadow[3] |= (uint32_t)(1UL << sb); } /* B */
    if(digit[1] & 0x01U) { s_shadow[2] |= (uint32_t)(1UL << sa); } /* F */
    if(digit[1] & 0x02U) { s_shadow[2] |= (uint32_t)(1UL << sb); } /* G */
    if(digit[2] & 0x01U) { s_shadow[1] |= (uint32_t)(1UL << sa); } /* E */
    if(digit[2] & 0x02U) { s_shadow[1] |= (uint32_t)(1UL << sb); } /* C */
    if(digit[3] & 0x01U) { s_shadow[0] |= (uint32_t)(1UL << sa); } /* D */

    /* Float type: turn on S14 icon (glass SEG9 -> SLCD SEG25) on Glass COM1 */
    if((type == FLOAT) && (position == 2U)) {
        s_shadow[0] |= (uint32_t)(1UL << 25U);
    }
}

/*! 
    \brief      write digit 8 according to XD-H2074A real segment map
    \param[in]  position: position in the SLCD of the digit, which can be 1..5
    \param[out] none
    \retval     none
*/
static void slcd_seg_digit8_write(uint8_t position)
{
    uint8_t sa, sb;
    uint32_t mask;

    if((position < 1U) || (position > 5U)) {
        return;
    }

    sa = pos_seg_a[position];
    sb = pos_seg_b[position];
    mask = (uint32_t)((1UL << sa) | (1UL << sb));

    /* clear both SEG bits on shadow registers */
    s_shadow[0] &= ~mask;
    s_shadow[1] &= ~mask;
    s_shadow[2] &= ~mask;
    s_shadow[3] &= ~mask;

    /*
     * Digit 8 lights A/B/C/D/E/F/G:
     * Glass COM4(DATA3): A/B -> both SEG bits
     * Glass COM3(DATA2): F/G -> both SEG bits
     * Glass COM2(DATA1): E/C -> both SEG bits
     * Glass COM1(DATA0): D   -> first SEG bit only
     *              (second SEG bit = icon area, do not light)
     */
    s_shadow[3] |= (uint32_t)((1UL << sa) | (1UL << sb)); /* A, B */
    s_shadow[2] |= (uint32_t)((1UL << sa) | (1UL << sb)); /* F, G */
    s_shadow[1] |= (uint32_t)((1UL << sa) | (1UL << sb)); /* E, C */
    s_shadow[0] |= (uint32_t)(1UL << sa);                  /* D only */
}
/*!
    \brief      set one SLCD DATA/SEG point
    \param[in]  data_index: SLCD DATA register index, 0..7
    \param[in]  seg: SLCD segment index, 0..31
    \param[in]  state: SET to turn on, RESET to turn off
    \param[out] none
    \retval     none
*/
static void slcd_seg_point_set(uint8_t data_index, uint8_t seg, FlagStatus state)
{
    uint32_t mask;

    if((data_index > 7U) || (seg > 31U)) {
        return;
    }

    mask = (uint32_t)(1UL << seg);

    switch(data_index) {
    case 0: if(state == SET) { s_shadow[0] |= mask; } else { s_shadow[0] &= ~mask; }
        break;
    case 1: if(state == SET) { s_shadow[1] |= mask; } else { s_shadow[1] &= ~mask; }
        break;
    case 2: if(state == SET) { s_shadow[2] |= mask; } else { s_shadow[2] &= ~mask; }
        break;
    case 3:
        if(state == SET) { s_shadow[3] |= mask; } else { s_shadow[3] &= ~mask; }
        break;
    case 4:
        if(state == SET) { s_shadow[3] |= mask; } else { s_shadow[3] &= ~mask; }
        break;
    case 5:
        if(state == SET) { s_shadow[2] |= mask; } else { s_shadow[2] &= ~mask; }
        break;
    case 6: if(state == SET) { s_shadow[3] |= mask; } else { s_shadow[3] &= ~mask; }
        break;
    case 7:
        if(state == SET) { s_shadow[0] |= mask; } else { s_shadow[0] &= ~mask; }
        break;
    default:
        break;
    }
}

/*!
    \brief      turn one icon on or off
    \param[in]  icon: icon name
    \param[in]  state: SET to turn on, RESET to turn off
    \param[out] none
    \retval     none
*/
void slcd_seg_icon_display(slcd_icon_enum icon, FlagStatus state)
{
    if(icon >= (slcd_icon_enum)(sizeof(icon_table) / sizeof(icon_table[0]))) {
        return;
    }

    slcd_seg_point_set(icon_table[icon].data_index, icon_table[icon].seg, state);

    slcd_auto_flush();
}

/*!
    \brief      clear all icons, keep numeric digit area unchanged
    \param[in]  none
    \param[out] none
    \retval     none
*/
void slcd_seg_icon_clear_all(void)
{
    uint8_t i;

    for(i = 0U; i < (uint8_t)(sizeof(icon_table) / sizeof(icon_table[0])); i++) {
        slcd_seg_point_set(icon_table[i].data_index, icon_table[i].seg, RESET);
    }

    slcd_auto_flush();
}

/*!
    \brief      light one SLCD COM/SEG point for XD-H2074A mapping verification
    \param[in]  com: glass COM index (0=COM1, 1=COM2, 2=COM3, 3=COM4)
    \param[in]  seg: SLCD segment index, 0..31
    \param[out] none
    \retval     none
*/
void slcd_seg_point_display(uint8_t com, uint8_t seg)
{
    if((com > 3U) || (seg > 31U)) {
        return;
    }

    s_shadow[0] = 0; s_shadow[1] = 0;
    s_shadow[2] = 0; s_shadow[3] = 0;

    switch(com) {
    case 0: s_shadow[0] = (uint32_t)(1UL << seg);
        break;
    case 1: s_shadow[1] = (uint32_t)(1UL << seg);
        break;
    case 2: s_shadow[2] = (uint32_t)(1UL << seg);
        break;
    case 3: s_shadow[3] = (uint32_t)(1UL << seg);
        break;
    default:
        break;
    }

    slcd_seg_flush();
}

/*!
    \brief      clear data at a specific display position
    \param[in]  position: 1..5
    \param[out] none
    \retval     none
*/
void slcd_seg_digit_clear(uint8_t position)
{
    uint8_t sa, sb;
    uint32_t mask;

    if((position < 1U) || (position > 5U)) {
        return;
    }

    sa = pos_seg_a[position];
    sb = pos_seg_b[position];
    mask = (uint32_t)((1UL << sa) | (1UL << sb));

    s_shadow[0] &= ~mask;
    s_shadow[1] &= ~mask;
    s_shadow[2] &= ~mask;
    s_shadow[3] &= ~mask;
}

/*!
    \brief      clear all active SLCD DATA registers
    \param[in]  none
    \param[out] none
    \retval     none
*/
void slcd_seg_clear_all(void)
{
    s_shadow[0] = 0;
    s_shadow[1] = 0;
    s_shadow[2] = 0;
    s_shadow[3] = 0;
    slcd_auto_flush();
}































