/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Configure QSPI as Slave 3-wire mode and demonstrate how to communicate
 *          with an off-chip SPI Master device with FIFO mode.
 *          This sample code needs to work with SPI_MasterFIFOMode sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT                  (16)

static uint32_t g_au32SourceData[TEST_COUNT] = {0};
static uint32_t g_au32DestinationData[TEST_COUNT] = {0};
static volatile uint32_t g_u32TxDataCount = 0;
static volatile uint32_t g_u32RxDataCount = 0;

void SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL0 clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ, CLK_APLL0_SELECT);

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

    /* Select PCLK0 as the clock source of QSPI0 */
    CLK_SetModuleClock(QSPI0_MODULE, CLK_QSPISEL_QSPI0SEL_PCLK0, MODULE_NoMsk);

    /* Enable QSPI0 peripheral clock */
    CLK_EnableModuleClock(QSPI0_MODULE);

    /* Enable GPIO Module clock */
    CLK_EnableModuleClock(GPIOA_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Setup QSPI0 multi-function pins */
    SET_QSPI0_CLK_PA2();
    SET_QSPI0_MOSI0_PA0();
    SET_QSPI0_MISO0_PA1();

    /* Enable QSPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;
}

void QSPI_Init(void)
{

    /* Configure as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure QSPI0 as a low level active device. */
    QSPI_Open(QSPI0, QSPI_SLAVE, QSPI_MODE_0, 32, (uint32_t)NULL);

    /* Enable slave 3 wire mode */
    QSPI0->SSCTL |= QSPI_SSCTL_SLV3WIRE_Msk;
}

int main(void)
{
    uint32_t u32TxDataCount = 0, u32RxDataCount = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Init QSPI */
    QSPI_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|           QSPI0 Slave 3 Wire Mode Sample Code                        |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure QSPI0 as a slave.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for QSPI0:\n");
    printf("    QSPI0_CLK(PA2)\n    QSPI0_MISO0(PA1)\n    QSPI0_MOSI0(PA0)\n\n");
    printf("QSPI controller will enable FIFO mode and transfer %d data to an off-chip master device.\n", TEST_COUNT);
    printf("In the meanwhile the QSPI controller will receive %d data from the off-chip master device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);

    for (u32TxDataCount = 0; u32TxDataCount < TEST_COUNT; u32TxDataCount++)
    {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32TxDataCount] = 0x00AA0000 + u32TxDataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32TxDataCount] = 0;
    }

    u32TxDataCount = 0;
    u32RxDataCount = 0;
    printf("Press any key if the master device configuration is ready.\n");
    getchar();
    printf("\n");

    /* Set TX FIFO threshold and enable FIFO mode. */
    QSPI_SetFIFO(QSPI0, 4, 4);

    /* Access TX and RX FIFO */
    while (u32RxDataCount < TEST_COUNT)
    {
        /* Check TX FULL flag and TX data count */
        if ((QSPI_GET_TX_FIFO_FULL_FLAG(QSPI0) == 0) && (u32TxDataCount < TEST_COUNT))
            QSPI_WRITE_TX(QSPI0, g_au32SourceData[u32TxDataCount++]); /* Write to TX FIFO */

        /* Check RX EMPTY flag */
        while (QSPI_GET_RX_FIFO_EMPTY_FLAG(QSPI0) == 0)
            g_au32DestinationData[u32RxDataCount++] = QSPI_READ_RX(QSPI0); /* Read RX FIFO */
    }

    /* Print the received data */
    printf("Received data:\n");

    for (u32RxDataCount = 0; u32RxDataCount < TEST_COUNT; u32RxDataCount++)
    {
        printf("%d:\t0x%X\n", u32RxDataCount, g_au32DestinationData[u32RxDataCount]);
    }

    printf("The data transfer was done.\n");

    printf("\n\nExit QSPI driver sample code.\n");

    /* Reset QSPI0 */
    QSPI_Close(QSPI0);

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
