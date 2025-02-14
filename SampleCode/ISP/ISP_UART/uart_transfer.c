/**************************************************************************//**
 * @file     uart_transfer.c
 * @version  V1.00
 * @brief    General UART ISP slave Sample file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <string.h>
#include "NuMicro.h"
#include "uart_transfer.h"

__ALIGNED(4) uint8_t g_au8uart_rcvbuf[MAX_PKT_SIZE] = {0};

uint8_t volatile g_u8bUartDataReady = 0;
uint8_t volatile g_u8bufhead = 0;

/* please check "targetdev.h" for chip specific define option */

/*---------------------------------------------------------------------------------------------------------*/
/* INTSTS to handle UART Channel 0 interrupt event                                                         */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void UART0_IRQHandler(void)
{
    /*----- Determine interrupt source -----*/
    uint32_t u32IntSrc = ISP_UART_PORT->INTSTS;
    uint8_t  u8Data;

    /* RDA FIFO interrupt and RDA timeout interrupt */
    if (u32IntSrc & (UART_INTSTS_RXTOIF_Msk | UART_INTSTS_RDAIF_Msk))
    {
        /* Read data until RX FIFO is empty or data is over maximum packet size */
        while (((ISP_UART_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) && (g_u8bufhead < MAX_PKT_SIZE))
        {
            u8Data = (uint8_t)ISP_UART_PORT->DAT;
            g_au8uart_rcvbuf[g_u8bufhead++] = u8Data;
        }
    }

    /* Reset data buffer index */
    if (g_u8bufhead == MAX_PKT_SIZE)
    {
        g_u8bUartDataReady = TRUE;
        g_u8bufhead = 0;
    }
    else if (u32IntSrc & UART_INTSTS_RXTOIF_Msk)
    {
        g_u8bufhead = 0;
    }
}

extern __attribute__((aligned(4))) uint8_t g_au8ResponseBuff[128];
void PutString(void)
{
    uint32_t i;
    uint32_t u32TimeOutCount = SystemCoreClock;

    /* UART send response to master */
    for (i = 0; i < MAX_PKT_SIZE; i++)
    {
        /* Wait for TX not full */
        while ((ISP_UART_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk))
        {
            if (--u32TimeOutCount == 0) break; /* 1 second time-out */
        }

        /* UART send data */
        ISP_UART_PORT->DAT = g_au8ResponseBuff[i];
    }

    /* Reset RX FIFO to clear unexpected RX data */
    UART_RESET_RXFIFO(ISP_UART_PORT);
}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select UART function */
    ISP_UART_PORT->FUNCSEL = UART_FUNCSEL_UART;
    /* Set UART line configuration */
    ISP_UART_PORT->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    /* Set UART Rx and RTS trigger level */
    ISP_UART_PORT->FIFO = UART_FIFO_RFITL_14BYTES | UART_FIFO_RTSTRGLV_14BYTES;
    /* Set UART baud rate */
    ISP_UART_PORT->BAUD = (UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200));
    /* Set time-out interrupt comparator */
    ISP_UART_PORT->TOUT = (ISP_UART_PORT->TOUT & ~UART_TOUT_TOIC_Msk) | (0x40);
    NVIC_SetPriority(UART0_IRQn, 2);
    NVIC_EnableIRQ(UART0_IRQn);
    /* Enable tim-out counter, Rx tim-out interrupt and Rx ready interrupt */
    ISP_UART_PORT->INTEN = (UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_RDAIEN_Msk);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/