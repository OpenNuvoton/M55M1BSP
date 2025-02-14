/**************************************************************************//**
 * @file    main.c
 * @version V1.0
 * @brief   Configure SPI as master mode and demonstrate how to communicate
 *          with an off-chip SPI slave device with FIFO mode.
 *          This sample code could work with SPI_SlaveFIFOMode sample code.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

//------------------------------------------------------------------------------
#define DATA_COUNT      16
#define TEST_PATTERN    0x00550000
#define SPI_CLK_FREQ    2000000

//------------------------------------------------------------------------------
/* Buffer for SPI0 data transfer with FIFO mode when DCache is disabled */
uint32_t g_au32SourceData[DATA_COUNT] = {0};
/* Buffer for SPI0 data transfer with FIFO mode when DCache is disabled */
uint32_t g_au32DestinationData[DATA_COUNT] = {0};
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

//------------------------------------------------------------------------------
NVT_ITCM void SPI0_IRQHandler(void)
{
    uint32_t u32RxDataCnt = 0;
    uint32_t u32TxDataCnt = 0;

    /* Check RX EMPTY flag */
    while (SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0) == 0)
    {
        u32RxDataCnt = g_u32RxDataCount++;
        /* Read RX FIFO */
        g_au32DestinationData[u32RxDataCnt] = SPI_READ_RX(SPI0);
    }

    /* Check TX FULL flag and TX data count */
    while ((SPI_GET_TX_FIFO_FULL_FLAG(SPI0) == 0) && (g_u32TxDataCount < DATA_COUNT))
    {
        u32TxDataCnt = g_u32TxDataCount++;
        /* Write to TX FIFO */
        SPI_WRITE_TX(SPI0, g_au32SourceData[u32TxDataCnt]);
    }

    if (g_u32TxDataCount >= DATA_COUNT)
        SPI_DisableInt(SPI0, SPI_FIFO_TXTH_INT_MASK); /* Disable TX FIFO threshold interrupt */

    /* Check the RX FIFO time-out interrupt flag */
    if (SPI_GetIntFlag(SPI0, SPI_FIFO_RXTO_INT_MASK))
    {
        /* If RX FIFO is not empty, read RX FIFO. */
        while (SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0) == 0)
        {
            u32RxDataCnt = g_u32RxDataCount++;
            g_au32DestinationData[u32RxDataCnt] = SPI_READ_RX(SPI0);
        }
    }
}

void SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL0 clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ, CLK_APLL0_SELECT);

    /* Switch SCLK clock source to PLL0 and divide 1 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);

    /* Set HCLK2 divide 2 */
    CLK_SET_HCLK2DIV(2);

    /* Set PCLKx divide 2 */
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Select PCLK0 as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_SPISEL_SPI0SEL_PCLK0, MODULE_NoMsk);

    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Setup SPI0 multi-function pins */
    /* PA.3 is SPI0_SS,   PA.2 is SPI0_CLK,
       PA.1 is SPI0_MISO, PA.0 is SPI0_MOSI*/
    SET_SPI0_SS_PA3();
    SET_SPI0_CLK_PA2();
    SET_SPI0_MOSI_PA0();
    SET_SPI0_MISO_PA1();
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2 MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 32, SPI_CLK_FREQ);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI0, SPI_SS, SPI_SS_ACTIVE_LOW);
}

/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    uint32_t u32DataCount;
    volatile int32_t i32TimeOutCount = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Init SPI */
    SPI_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                     SPI Master Mode Sample Code                      |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure SPI0 as a master.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for SPI0:\n");
    printf("    SPI0_SS(PA.3)\n    SPI0_CLK(PA.2)\n");
    printf("    SPI0_MISO(PA.1)\n    SPI0_MOSI(PA.0)\n\n");
    printf("SPI controller will enable FIFO mode and transfer %d data to a off-chip slave device.\n", DATA_COUNT);
    printf("In the meanwhile the SPI controller will receive %d data from the off-chip slave device.\n", DATA_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", DATA_COUNT);
    printf("The SPI master configuration is ready.\n");

    for (u32DataCount = 0; u32DataCount < DATA_COUNT; u32DataCount++)
    {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32DataCount] = TEST_PATTERN + u32DataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32DataCount] = 0;
    }

    printf("Before starting the data transfer, make sure the slave device is ready. Press any key to start the transfer.\n");
    getchar();
    printf("\n");

    /* Set TX FIFO threshold, enable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI_SetFIFO(SPI0, 2, 2);
    SPI_EnableInt(SPI0, SPI_FIFO_TXTH_INT_MASK | SPI_FIFO_RXTO_INT_MASK);

    g_u32TxDataCount = 0;
    g_u32RxDataCount = 0;

    NVIC_EnableIRQ(SPI0_IRQn);

    /* setup timeout */
    i32TimeOutCount = SystemCoreClock;

    /* Wait for transfer done */
    while (g_u32RxDataCount < DATA_COUNT)
    {
        if (--i32TimeOutCount <= 0)
        {
            printf("\nSomething is wrong, please check if pin connection is correct. \n");

            while (1);
        }
    }

    /* Print the received data */
    printf("Received data:\n");

    for (u32DataCount = 0; u32DataCount < DATA_COUNT; u32DataCount++)
    {
        printf("%d:\t0x%X\n", u32DataCount, g_au32DestinationData[u32DataCount]);
    }

    /* Disable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI_DisableInt(SPI0, SPI_FIFO_TXTH_INT_MASK | SPI_FIFO_RXTO_INT_MASK);
    NVIC_DisableIRQ(SPI0_IRQn);
    printf("The data transfer was done.\n");

    printf("\n\nExit SPI driver sample code.\n");

    /* Reset SPI0 */
    SPI_Close(SPI0);

    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
