/******************************************************************************
 * @file     uart_transfer.h
 * @version  V1.00
 * @brief    General UART ISP slave header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __UART_TRANS_H__
#define __UART_TRANS_H__
#include <stdint.h>

/*-------------------------------------------------------------*/
/* Define maximum packet size */
#define MAX_PKT_SIZE            64

/*-------------------------------------------------------------*/

extern uint8_t g_au8uart_rcvbuf[];
extern uint8_t volatile g_u8bUartDataReady;
extern uint8_t volatile g_u8bufhead;

/*-------------------------------------------------------------*/
void UART_Init(void);
void UART6_IRQHandler(void);
void PutString(void);

#endif  /* __UART_TRANS_H__ */
