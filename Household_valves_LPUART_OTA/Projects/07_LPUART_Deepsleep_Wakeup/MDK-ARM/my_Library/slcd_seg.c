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

    /* configure SLCD low-speed clock source */
    slcd_clock_source_config();
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
    /* SLCD voltage source select */
    slcd_voltage_source_select(SLCD_VOLTAGE_INTERNAL);
    /* SLCD dead time duration config */
    slcd_dead_time_config(SLCD_DEADTIME_PERIOD_0);
    /* enable the permanent high drive */
    slcd_high_drive_config(ENABLE);

    /* wait for SLCD CFG register synchronization */
    while(!slcd_flag_get(SLCD_FLAG_SYN));
    /* enable SLCD interface */
    slcd_enable();
    /* wait for SLCD controller on flag */
    while(!slcd_flag_get(SLCD_FLAG_ON));
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
    /* wait the last SLCD DATA update request finished */
    while(slcd_flag_get(SLCD_FLAG_UPR));

    /* SLCD write a char */
    slcd_seg_digit_write(ch, position, INTEGER);

    /* request SLCD DATA update */
    slcd_data_update_request();
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

    slcd_seg_clear_all();
    while(slcd_flag_get(SLCD_FLAG_UPR));

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

    slcd_data_update_request();
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

    slcd_seg_clear_all();
    while(slcd_flag_get(SLCD_FLAG_UPR));

    for(i = 0U; i < width; i++) {
        digit_value = (uint8_t)((num / divisor) % 10U);
        if((digit_value == 0U) && (leading_zero == 0U) && (i < (width - 1U)) && (num < divisor)) {
        } else {
            slcd_seg_digit_write(digit_value, (uint8_t)(5U - width + i + 1U), INTEGER);
        }
        divisor /= 10U;
    }

    slcd_data_update_request();
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

    while(slcd_flag_get(SLCD_FLAG_UPR));

    /* Clear both SEG bits on glass-connected COM registers */
    SLCD_DATA0 &= ~mask;
    SLCD_DATA1 &= ~mask;
    SLCD_DATA2 &= ~mask;
    SLCD_DATA3 &= ~mask;

    /*
     * Glass COM1(DATA4): A(sa) B(sb)
     * Glass COM2(DATA5): F(sa) G(sb)
     * Glass COM3(DATA6): E(sa) C(sb)
     * Glass COM4(DATA7): D(sa)   (sb is icon/unused)
     */
    if(code & 0x01U) { SLCD_DATA3 |= (uint32_t)(1UL << sa); } /* A */
    if(code & 0x02U) { SLCD_DATA3 |= (uint32_t)(1UL << sb); } /* B */
    if(code & 0x04U) { SLCD_DATA2 |= (uint32_t)(1UL << sa); } /* F */
    if(code & 0x08U) { SLCD_DATA2 |= (uint32_t)(1UL << sb); } /* G */
    if(code & 0x10U) { SLCD_DATA1 |= (uint32_t)(1UL << sa); } /* E */
    if(code & 0x20U) { SLCD_DATA1 |= (uint32_t)(1UL << sb); } /* C */
    if(code & 0x40U) { SLCD_DATA0 |= (uint32_t)(1UL << sa); } /* D */

    slcd_data_update_request();
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
    char ch;

    if(str == 0) {
        return;
    }

    slcd_seg_clear_all();
    while(slcd_flag_get(SLCD_FLAG_UPR));

    /*
     * Numeric display rule:
     * - position 1 is reserved for sign only.
     * - positive numeric values use positions 2..5, position 1 remains blank.
     * - negative numeric values display '-' at position 1, then use positions 2..5.
     * - non-numeric strings use positions 1..5 directly.
     */
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

    slcd_seg_clear_all();
    while(slcd_flag_get(SLCD_FLAG_UPR));

    /* send the string character one by one to SLCD */
    while(i < length) {
        /* display one digit on SLCD */
        slcd_seg_digit_write(ch[i], 6 - i, FLOAT);
        /* increment the digit counter */
        i++;
    }

    slcd_data_update_request();
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

    /* clear both SEG bits on all glass-connected COM registers */
    SLCD_DATA0 &= ~mask;
    SLCD_DATA1 &= ~mask;
    SLCD_DATA2 &= ~mask;
    SLCD_DATA3 &= ~mask;

    /*
     * 7-segment -> (COM, SEG) mapping:
     *   digit[0] bit0=A, bit1=B -> Glass COM1 -> DATA4
     *   digit[1] bit0=F, bit1=G -> Glass COM2 -> DATA5
     *   digit[2] bit0=E, bit1=C -> Glass COM3 -> DATA6
     *   digit[3] bit0=D           -> Glass COM4 -> DATA7
     */
    if(digit[0] & 0x01U) { SLCD_DATA3 |= (uint32_t)(1UL << sa); } /* A */
    if(digit[0] & 0x02U) { SLCD_DATA3 |= (uint32_t)(1UL << sb); } /* B */
    if(digit[1] & 0x01U) { SLCD_DATA2 |= (uint32_t)(1UL << sa); } /* F */
    if(digit[1] & 0x02U) { SLCD_DATA2 |= (uint32_t)(1UL << sb); } /* G */
    if(digit[2] & 0x01U) { SLCD_DATA1 |= (uint32_t)(1UL << sa); } /* E */
    if(digit[2] & 0x02U) { SLCD_DATA1 |= (uint32_t)(1UL << sb); } /* C */
    if(digit[3] & 0x01U) { SLCD_DATA0 |= (uint32_t)(1UL << sa); } /* D */

    /* Float type: turn on S14 icon (glass SEG9 -> SLCD SEG25) on Glass COM1 */
    if((type == FLOAT) && (position == 2U)) {
        SLCD_DATA0 |= (uint32_t)(1UL << 25U);
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

    /* clear both SEG bits on all glass-connected COM registers */
    SLCD_DATA0 &= ~mask;
    SLCD_DATA1 &= ~mask;
    SLCD_DATA2 &= ~mask;
    SLCD_DATA3 &= ~mask;

    /*
     * Digit 8 lights A/B/C/D/E/F/G:
     * Glass COM4(DATA3): A/B -> both SEG bits
     * Glass COM3(DATA2): F/G -> both SEG bits
     * Glass COM2(DATA1): E/C -> both SEG bits
     * Glass COM1(DATA0): D   -> first SEG bit only
     *              (second SEG bit = icon area, do not light)
     */
    SLCD_DATA3 |= (uint32_t)((1UL << sa) | (1UL << sb)); /* A, B */
    SLCD_DATA2 |= (uint32_t)((1UL << sa) | (1UL << sb)); /* F, G */
    SLCD_DATA1 |= (uint32_t)((1UL << sa) | (1UL << sb)); /* E, C */
    SLCD_DATA0 |= (uint32_t)(1UL << sa);                  /* D only */
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
    case 0: if(state == SET) { SLCD_DATA0 |= mask; } else { SLCD_DATA0 &= ~mask; }
        break;
    case 1: if(state == SET) { SLCD_DATA1 |= mask; } else { SLCD_DATA1 &= ~mask; }
        break;
    case 2: if(state == SET) { SLCD_DATA2 |= mask; } else { SLCD_DATA2 &= ~mask; }
        break;
    case 3:
        if(state == SET) { SLCD_DATA3 |= mask; } else { SLCD_DATA3 &= ~mask; }
        break;
    case 4:
        if(state == SET) { SLCD_DATA3 |= mask; } else { SLCD_DATA3 &= ~mask; }
        break;
    case 5:
        if(state == SET) { SLCD_DATA2 |= mask; } else { SLCD_DATA2 &= ~mask; }
        break;
    case 6: if(state == SET) { SLCD_DATA3 |= mask; } else { SLCD_DATA3 &= ~mask; }
        break;
    case 7:
        if(state == SET) { SLCD_DATA0 |= mask; } else { SLCD_DATA0 &= ~mask; }
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

    /* wait the last SLCD DATA update request finished */
    while(slcd_flag_get(SLCD_FLAG_UPR));

    slcd_seg_point_set(icon_table[icon].data_index, icon_table[icon].seg, state);

    /* request SLCD DATA update */
    slcd_data_update_request();
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

    /* wait the last SLCD DATA update request finished */
    while(slcd_flag_get(SLCD_FLAG_UPR));

    for(i = 0U; i < (uint8_t)(sizeof(icon_table) / sizeof(icon_table[0])); i++) {
        slcd_seg_point_set(icon_table[i].data_index, icon_table[i].seg, RESET);
    }

    /* request SLCD DATA update */
    slcd_data_update_request();
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

    /* wait the last SLCD DATA update request finished */
    while(slcd_flag_get(SLCD_FLAG_UPR));

    slcd_seg_clear_all();

    switch(com) {
    case 0: SLCD_DATA0 = (uint32_t)(1UL << seg);
        break;
    case 1: SLCD_DATA1 = (uint32_t)(1UL << seg);
        break;
    case 2: SLCD_DATA2 = (uint32_t)(1UL << seg);
        break;
    case 3: SLCD_DATA3 = (uint32_t)(1UL << seg);
        break;
    default:
        break;
    }

    /* request SLCD DATA update */
    slcd_data_update_request();
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

    SLCD_DATA0 &= ~mask;
    SLCD_DATA1 &= ~mask;
    SLCD_DATA2 &= ~mask;
    SLCD_DATA3 &= ~mask;
}

/*!
    \brief      clear all active SLCD DATA registers
    \param[in]  none
    \param[out] none
    \retval     none
*/
void slcd_seg_clear_all(void)
{
    SLCD_DATA0 = 0x0000;
    SLCD_DATA1 = 0x0000;
    SLCD_DATA2 = 0x0000;
    SLCD_DATA3 = 0x0000;
    SLCD_DATA3 = 0x0000;
}































