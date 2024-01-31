/**************************************************************************//**
 * @file    main.c
 * @version V1.0
 * @brief   Configure LPSPI as Master mode and demonstrate how to communicate
 *          with an off-chip SPI slave device with FIFO mode.
 *          This sample code needs to work with LPSPI_SlaveFIFOMode sample code.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define DATA_COUNT          16
#define TEST_PATTERN        0x00550000
#define LPSPI_CLK_FREQ      2000000

uint32_t g_au32SourceData[DATA_COUNT];
uint32_t g_au32DestinationData[DATA_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

/* Function prototype declaration */
void SYS_Init(void);
void LPSPI_Init(void);

int main(void)
{
    uint32_t u32DataCount;
    uint32_t u32TimeOutCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Init LPSPI0 */
    LPSPI_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                     LPSPI Master Mode Sample Code                    |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure LPSPI0 as a master.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for LPSPI0:\n");
    printf("    LPSPI0_SS(PA.3)\n    LPSPI0_CLK(PA.2)\n");
    printf("    LPSPI0_MISO(PA.1)\n    LPSPI0_MOSI(PA.0)\n\n");
    printf("LPSPI controller will enable FIFO mode and transfer %d data to a off-chip slave device.\n", DATA_COUNT);
    printf("In the meanwhile the SPI controller will receive %d data from the off-chip slave device.\n", DATA_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", DATA_COUNT);
    printf("The LPSPI master configuration is ready.\n");

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
    LPSPI_SetFIFO(LPSPI0, 2, 2);
    LPSPI_EnableInt(LPSPI0, LPSPI_FIFO_TXTH_INT_MASK | LPSPI_FIFO_RXTO_INT_MASK);

    g_u32TxDataCount = 0;
    g_u32RxDataCount = 0;
    NVIC_EnableIRQ(LPSPI0_IRQn);

    /* setup timeout */
    u32TimeOutCount = SystemCoreClock;

    /* Wait for transfer done */
    while (g_u32RxDataCount < DATA_COUNT)
    {
        if (u32TimeOutCount == 0)
        {
            printf("\nSomething is wrong, please check if pin connection is correct. \n");

            while (1);
        }

        u32TimeOutCount--;
    }

    /* Print the received data */
    printf("Received data:\n");

    for (u32DataCount = 0; u32DataCount < DATA_COUNT; u32DataCount++)
    {
        printf("%d:\t0x%X\n", u32DataCount, g_au32DestinationData[u32DataCount]);
    }

    /* Disable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    LPSPI_DisableInt(LPSPI0, LPSPI_FIFO_TXTH_INT_MASK | LPSPI_FIFO_RXTO_INT_MASK);
    NVIC_DisableIRQ(LPSPI0_IRQn);
    printf("The data transfer was done.\n");

    printf("\n\nExit SPI driver sample code.\n");

    /* Reset LPSPI0 */
    LPSPI_Close(LPSPI0);

    while (1);
}

void SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL0 180MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL0_SELECT);

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

    /* Select HIRC as the clock source of LPSPI0 */
    CLK_SetModuleClock(LPSPI0_MODULE, CLK_LPSPISEL_LPSPI0SEL_HIRC, MODULE_NoMsk);

    /* Enable LPSPI0 peripheral clock */
    CLK_EnableModuleClock(LPSPI0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Setup LPSPI0 multi-function pins */
    /* PA.3 is LPSPI0_SS,   PA.2 is LPSPI0_CLK,
       PA.1 is LPSPI0_MISO, PA.0 is LPSPI0_MOSI*/
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA3MFP_Msk |
                                       SYS_GPA_MFP0_PA2MFP_Msk |
                                       SYS_GPA_MFP0_PA1MFP_Msk |
                                       SYS_GPA_MFP0_PA0MFP_Msk)) |
                    (SYS_GPA_MFP0_PA3MFP_LPSPI0_SS |
                     SYS_GPA_MFP0_PA2MFP_LPSPI0_CLK |
                     SYS_GPA_MFP0_PA1MFP_LPSPI0_MISO |
                     SYS_GPA_MFP0_PA0MFP_LPSPI0_MOSI);
}

void LPSPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init LPSPI0                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. LPSPI clock rate = 2 MHz */
    LPSPI_Open(LPSPI0, LPSPI_MASTER, LPSPI_MODE_0, 32, LPSPI_CLK_FREQ);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    LPSPI_EnableAutoSS(LPSPI0, LPSPI_SS, LPSPI_SS_ACTIVE_LOW);
}

NVT_ITCM void LPSPI0_IRQHandler(void)
{
    uint32_t u32RxDataCount = 0;

    /* Check RX EMPTY flag */
    while (LPSPI_GET_RX_FIFO_EMPTY_FLAG(LPSPI0) == 0)
    {
        /* Read RX FIFO */
        u32RxDataCount = g_u32RxDataCount++;
        g_au32DestinationData[u32RxDataCount] = LPSPI_READ_RX(LPSPI0);
    }

    /* Check TX FULL flag and TX data count */
    while ((LPSPI_GET_TX_FIFO_FULL_FLAG(LPSPI0) == 0) && (g_u32TxDataCount < DATA_COUNT))
    {
        /* Write to TX FIFO */
        LPSPI_WRITE_TX(LPSPI0, g_au32SourceData[g_u32TxDataCount++]);
    }

    if (g_u32TxDataCount >= DATA_COUNT)
        LPSPI_DisableInt(LPSPI0, LPSPI_FIFO_TXTH_INT_MASK); /* Disable TX FIFO threshold interrupt */

    /* Check the RX FIFO time-out interrupt flag */
    if (LPSPI_GetIntFlag(LPSPI0, LPSPI_FIFO_RXTO_INT_MASK))
    {
        /* If RX FIFO is not empty, read RX FIFO. */
        while (LPSPI_GET_RX_FIFO_EMPTY_FLAG(LPSPI0) == 0)
        {
            u32RxDataCount = g_u32RxDataCount++;
            g_au32DestinationData[u32RxDataCount] = LPSPI_READ_RX(LPSPI0);
        }
    }
}


/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

