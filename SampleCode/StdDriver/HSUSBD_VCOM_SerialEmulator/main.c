/***************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to implement an USB virtual COM port device.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "vcom_serial.h"

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING g_LineCoding = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t gCtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* RX buffer size */

#define TX_FIFO_SIZE        16  /* TX Hardware FIFO size */


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#ifdef __ICCARM__
#pragma data_alignment=4
volatile uint8_t comRbuf[RXBUFSIZE];
volatile uint8_t comTbuf[TXBUFSIZE];
uint8_t gRxBuf[RXBUFSIZE] = {0};
uint8_t gUsbRxBuf[RXBUFSIZE] = {0};
#else
volatile uint8_t comRbuf[RXBUFSIZE] __attribute__((aligned(4)));
volatile uint8_t comTbuf[TXBUFSIZE]__attribute__((aligned(4)));
uint8_t gRxBuf[RXBUFSIZE] __attribute__((aligned(4))) = {0};
uint8_t gUsbRxBuf[RXBUFSIZE] __attribute__((aligned(4))) = {0};
#endif


volatile uint16_t comRbytes = 0;
volatile uint16_t comRhead = 0;
volatile uint16_t comRtail = 0;

volatile uint16_t comTbytes = 0;
volatile uint16_t comThead = 0;
volatile uint16_t comTtail = 0;

uint32_t gu32RxSize = 0;
uint32_t gu32TxSize = 0;

volatile int8_t gi8BulkOutReady = 0;

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Switch SCLK clock source to APLL0 and Enable APLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable all GPIO clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(GPIOC_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(GPIOF_MODULE);
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);
    CLK_EnableModuleClock(GPIOJ_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Enable HSOTG module clock */
    CLK_EnableModuleClock(HSOTG0_MODULE);

    /* Select HSOTG PHY Reference clock frequency which is from HXT */
    HSOTG_SET_PHY_REF_CLK(HSOTG_PHYCTL_FSEL_24_0M);

    /* Set HSUSB role to HSUSBD */
    SET_HSUSBDROLE();

    /* Enable HSUSB PHY */
    SYS_Enable_HSUSB_PHY();

    /* Enable HSUSBD peripheral clock */
    CLK_EnableModuleClock(HSUSBD0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART RXD and TXD */
    SetDebugUartMFP();
}

/* DEBUG_PORT Interrupt handler */
NVT_ITCM void DEBUG_PORT_IRQHandler(void)
{
    uint8_t bInChar;
    int32_t size;
    uint32_t u32IntStatus;

    u32IntStatus = DEBUG_PORT->INTSTS;

    if ((u32IntStatus & UART_INTSTS_RDAINT_Msk) || (u32IntStatus & UART_INTSTS_RXTOINT_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while ((!UART_GET_RX_EMPTY(DEBUG_PORT)))
        {
            /* Get the character from UART Buffer */
            bInChar = UART_READ(DEBUG_PORT);    /* Rx trigger level is 1 byte*/

            /* Check if buffer full */
            if (comRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                comRbuf[comRtail++] = bInChar;

                if (comRtail >= RXBUFSIZE)
                    comRtail = 0;

                comRbytes++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if (u32IntStatus & UART_INTSTS_THREINT_Msk)
    {

        if (comTbytes && (DEBUG_PORT->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            size = comTbytes;

            if (size >= TX_FIFO_SIZE)
            {
                size = TX_FIFO_SIZE;
            }

            while (size)
            {
                bInChar = comTbuf[comThead++];
                UART_WRITE(DEBUG_PORT, bInChar);

                if (comThead >= TXBUFSIZE)
                    comThead = 0;

                comTbytes--;
                size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            DEBUG_PORT->INTEN &= ~UART_INTEN_THREIEN_Msk;
        }
    }
}

void VCOM_TransferData(void)
{
    int32_t i, i32Len;

    /* Check whether USB is ready for next packet or not */
    if (gu32TxSize == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if (comRbytes)
        {
            i32Len = comRbytes;

            if (i32Len > EPA_MAX_PKT_SIZE)
                i32Len = EPA_MAX_PKT_SIZE;

            for (i = 0; i < i32Len; i++)
            {
                gRxBuf[i] = comRbuf[comRhead++];

                if (comRhead >= RXBUFSIZE)
                    comRhead = 0;
            }

            NVIC_DisableIRQ(UART0_IRQn);
            comRbytes -= i32Len;
            NVIC_EnableIRQ(UART0_IRQn);

            gu32TxSize = i32Len;

            for (i = 0; i < i32Len; i++)
                HSUSBD->EP[EPA].EPDAT_BYTE = gRxBuf[i];

            HSUSBD->EP[EPA].EPRSPCTL = HSUSBD_EP_RSPCTL_SHORTTXEN;    // packet end
            HSUSBD->EP[EPA].EPTXCNT = i32Len;
            HSUSBD_ENABLE_EP_INT(EPA, HSUSBD_EPINTEN_INTKIEN_Msk);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EPA_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            HSUSBD->EP[EPA].EPRSPCTL = HSUSBD_EP_RSPCTL_SHORTTXEN | HSUSBD_EP_RSPCTL_ZEROLEN;    // packet end
            HSUSBD->EP[EPA].EPTXCNT = 0;
            HSUSBD_ENABLE_EP_INT(EPA, HSUSBD_EPINTEN_INTKIEN_Msk);
        }
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if (gi8BulkOutReady && (gu32RxSize <= TXBUFSIZE - comTbytes))
    {
        for (i = 0; i < gu32RxSize; i++)
        {
            comTbuf[comTtail++] = gUsbRxBuf[i];

            if (comTtail >= TXBUFSIZE)
                comTtail = 0;
        }

        NVIC_DisableIRQ(DEBUG_PORT_IRQn);
        comTbytes += gu32RxSize;
        NVIC_EnableIRQ(DEBUG_PORT_IRQn);

        gu32RxSize = 0;
        gi8BulkOutReady = 0; /* Clear bulk out ready flag */
    }

    /* Process the software Tx FIFO */
    if (comTbytes)
    {
        /* Check if Tx is working */
        if ((DEBUG_PORT->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART_WRITE(DEBUG_PORT, comTbuf[comThead++]);

            if (comThead >= TXBUFSIZE)
                comThead = 0;

            comTbytes--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            DEBUG_PORT->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
}

int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(DEBUG_PORT, 115200);

    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(DEBUG_PORT, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));

    printf("NuMicro USB CDC VCOM\n");

    HSUSBD_Open(&gsHSInfo, VCOM_ClassRequest, NULL);

    /* Endpoint configuration */
    VCOM_Init();

    /* Enable HSUSBD interrupt */
    NVIC_EnableIRQ(HSUSBD_IRQn);

    /* Start transaction */
    while (1)
    {
        if (HSUSBD_IS_ATTACHED())
        {
            HSUSBD_Start();
            break;
        }
    }

    while (1)
    {
        VCOM_TransferData();
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
