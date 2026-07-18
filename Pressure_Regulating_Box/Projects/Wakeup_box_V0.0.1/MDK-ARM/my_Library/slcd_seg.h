/*!
    \file    slcd_seg.h
    \brief   the header file of the slcd segment driver

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

#ifndef LCD_SEG_H
#define LCD_SEG_H

#include "gd32l23x.h"

typedef enum {
    INTEGER = 0,
    FLOAT = 1,
    TIME = 2
} slcd_display_enum;

typedef enum {
    SLCD_ICON_T1 = 0,			//充电显示
    SLCD_ICON_P2,				//小数点1
    SLCD_ICON_P3,				//小数点2
    SLCD_ICON_P4,				//小数点3
    SLCD_ICON_S14,				//MPa图标
    SLCD_ICON_S1,				//电量外框
    SLCD_ICON_S2,				//1格电
    SLCD_ICON_S3,				//2格电
    SLCD_ICON_S5,				//进口
    SLCD_ICON_S7,				//信号图标
    SLCD_ICON_LY,				//蓝牙图标
    SLCD_ICON_S11,				//阀门图标
    SLCD_ICON_S6,				//出口
    SLCD_ICON_S12,				//℃单位
    SLCD_ICON_S8,				//1格信号
    SLCD_ICON_S9,				//2格信号
    SLCD_ICON_S10,				//3格信号
    SLCD_ICON_T2,				//PPM单位
    SLCD_ICON_T3,				//%图标
    SLCD_ICON_T4,				//LEL单位
    SLCD_ICON_S13				//KPa单位
} slcd_icon_enum;

/* configure SLCD peripheral */
void slcd_seg_configuration(void);
/* display one digit at position 1..5 */
void slcd_seg_digit_display(uint8_t ch, uint8_t position);
/* display an integer, right aligned, max 5 digits */
void slcd_seg_number_display(uint32_t num);
/* display an integer with fixed width 1..5 */
void slcd_seg_number_width_display(uint32_t num, uint8_t width, uint8_t leading_zero);
/* display a 5-position string, supports 0-9, letters, '-', '_' and decimal point */
void slcd_seg_string_display(const char *str);
/* display a 5-position string atomically (single update, no flicker) */
void slcd_seg_string_atomic_display(const char *str);
/* display raw 7-segment code at position 1..5 */
void slcd_seg_code_display(uint8_t code, uint8_t position);
/* turn decimal point after position 1..5 on or off */
void slcd_seg_decimal_display(uint8_t position, FlagStatus state);
/* write a float number(6 digits which has 2 decimal) to SLCD DATA register */
void slcd_seg_floatnumber_display(float num);
/* turn one icon on or off */
void slcd_seg_icon_display(slcd_icon_enum icon, FlagStatus state);
/* clear all icons */
void slcd_seg_icon_clear_all(void);
/* light one SLCD COM/SEG point for XD-H2074A mapping verification */
void slcd_seg_point_display(uint8_t com, uint8_t seg);
/* clear data in the SLCD DATA register */
void slcd_seg_digit_clear(uint8_t position);
/* clear all the SLCD DATA register */
void slcd_seg_clear_all(void);

/* ====== 影子缓冲区批量模式 (消除闪屏) ====== */
/* 开始批量操作: 后续写入仅修改影子缓冲区, 不触发硬件刷新 */
void slcd_seg_batch_begin(void);
/* 结束批量操作: 将影子缓冲区写入硬件并触发一次刷新 */
void slcd_seg_batch_end(void);
/* 手动刷新: 将影子缓冲区写入硬件 (非批量模式下自动调用) */
void slcd_seg_flush(void);
/* 仅清除数字段码+小数点+页面图标, 保留电池/信号/阀门/蓝牙等常驻图标 */
void slcd_seg_clear_digits(void);

#endif /* LCD_SEG_H */
