/*!
    \file    gd32l23x_it.c
    \brief   interrupt service routines

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

#include "gd32l23x_it.h"
#include "gd32l233r_eval.h"
#include "gd32l23x_rtc.h"
#include "systick.h"
#include "delay.h"
#include "usart.h"
#include "mygpio.h"

__IO uint8_t counter0 = 0x00;
extern volatile uint8_t rtc_alarm_flag;
extern void led_spark(void);
extern FlagStatus g_transfer_complete;

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
    /* if NMI exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
    /* if SVC exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
    /* if PendSV exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
    delay_decrement();
    trea_time_isr();
}

/*!
    \brief      this function handles LPUART error interrupt (DMA mode: ORE/FERR clear)
*/
void LPUART_IRQHandler(void)
{
    lpuart_rx_irq_handler();
}

/*!
    \brief      this function handles LPUART wakeup from deepsleep interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void LPUART_WKUP_IRQHandler(void)
{
    if(SET == lpuart_interrupt_flag_get(LPUART0, LPUART_INT_FLAG_WU)) {
        lpuart_flag_clear(LPUART0, LPUART_FLAG_WU);
        counter0 = 0x01;
        exti_flag_clear(EXTI_28);
    }
}

/*!
    \brief      this function handles RTC Alarm interrupt request (EXTI17)
*/
void RTC_Alarm_IRQHandler(void)
{
    if(RESET != rtc_flag_get(RTC_FLAG_ALARM0)) {
        rtc_flag_clear(RTC_FLAG_ALARM0);
        exti_flag_clear(EXTI_17);
        rtc_alarm_flag = 1;
    }
}

/*!
    \brief      this function handles USART0 interrupt request (debug serial RX)
*/
void USART0_IRQHandler(void)
{
    debug_rx_irq_handler();
}

/*!
    \brief      this function handles EXTI0 interrupt request (PD0 Btn1 wakeup)
*/
void EXTI0_IRQHandler(void)
{
    if(RESET != exti_flag_get(EXTI_0)) {
        exti_flag_clear(EXTI_0);
        // 立即禁用EXTI0, 防止按键抖动期间重复进入ISR
        exti_interrupt_disable(EXTI_0);
        button_wakeup_flag = 1;
    }
}

/*!
    \brief      this function handles EXTI1 interrupt request (PA1 4G RI wakeup)
*/
void EXTI1_IRQHandler(void)
{
    if(RESET != exti_flag_get(EXTI_1)) {
        exti_flag_clear(EXTI_1);
        exti_interrupt_disable(EXTI_1);
        ri_wakeup_flag = 1;
    }
}

/*!
    \brief      this function handles EXTI2 interrupt request (PD2 BLE connect wakeup)
*/
void EXTI2_IRQHandler(void)
{
    if(RESET != exti_flag_get(EXTI_2)) {
        exti_flag_clear(EXTI_2);
        exti_interrupt_disable(EXTI_2);
        ble_connect_wakeup_flag = 1;
    }
}

/*!
    \brief      this function handles UART4 interrupt request (BLE OTA RX IDLE)
*/
void UART4_IRQHandler(void)
{
    ble_uart_rx_irq_handler();
}
