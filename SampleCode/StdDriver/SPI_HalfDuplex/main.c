/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate SPI half-duplex mode.
 *          Configure SPI0 as master mode and SPI1 as slave mode.
 *          Both SPI0 and SPI1 are half-duplex mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT              4

static uint32_t s_au32DestinationData[TEST_COUNT] = {0};
static uint32_t s_u32RxDataCount = 0;

/* Function prototype declaration */
void SYS_Init(void);
void SPI_Init(void);

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

    /* Enable SPI0 module clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Enable SPI1 module clock */
    CLK_EnableModuleClock(SPI1_MODULE);

    /* Select SPI0 module clock source as PCLK0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_SPISEL_SPI0SEL_PCLK0, MODULE_NoMsk);

    /* Select SPI1 module clock source as PCLK2 */
    CLK_SetModuleClock(SPI1_MODULE, CLK_SPISEL_SPI1SEL_PCLK2, MODULE_NoMsk);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Configure SPI0 related multi-function pins. GPA[3:0] : SPI0_SS, SPI0_CLK, SPI0_MOSI. */
    SET_SPI0_MOSI_PA0();
    SET_SPI0_CLK_PA2();
    SET_SPI0_SS_PA3();

    /* Configure SPI1 related multi-function pins. GPB[4:2] : SPI1_SS, SPI1_CLK, SPI1_MOSI. */
    SET_SPI1_MOSI_PB4();
    SET_SPI1_CLK_PB3();
    SET_SPI1_SS_PB2();
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 32, 2000000);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI0, SPI_SS, SPI_SS_ACTIVE_LOW);

    /* Configure SPI1 */
    /* Configure SPI1 as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI1 as a low level active device. SPI peripheral clock rate = f_PCLK0 */
    SPI_Open(SPI1, SPI_SLAVE, SPI_MODE_0, 32, (uint32_t)NULL);
}

int main(void)
{
    uint32_t u32DataCount, u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure debug uart: 115200, 8-bit word, no parity bit, 1 stop bit. */
    InitDebugUart();

    /* Init SPI */
    SPI_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                   SPI Half-duplex Mode Sample Code                   |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure SPI0 as a master and SPI1 as a slave.\n");
    printf("Set both SPI0 and SPI1 to half-duplex.\n");
    printf("Bit length of a transaction: 32\n");
    printf("Please connect below I/O connections for SPI0 and SPI1:\n");
    printf("    SPI0_SS(PA3)   <->   SPI1_SS(PB2)\n");
    printf("    SPI0_CLK(PA2)  <->   SPI1_CLK(PB3)\n");
    printf("    SPI0_MOSI(PA0) <->   SPI1_MOSI(PB4)\n\n");
    printf("After the transfer is done, the received data will be printed out.\n");

    /* Set slave SPI1 to half-duplex mode */
    SPI1->CTL |= SPI_CTL_HALFDPX_Msk;
    /* Enable half-duplex will produce TXFBCLR (SPIx_FIFOCTL[9]) and RXFBCLR (SPIx_FIFOCTL[8])*/
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (SPI1->STATUS & SPI_STATUS_TXRXRST_Msk)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for SPI time-out!\n");
            goto lexit;
        }
    }

    /* Set slave SPI1 data direction to output */
    SPI1->CTL |= SPI_CTL_DATDIR_Msk;

    /* Slave SPI1 prepare data to TX FIFO */
    SPI_WRITE_TX(SPI1, 0x55AA0000);
    SPI_WRITE_TX(SPI1, 0x55AA0001);
    SPI_WRITE_TX(SPI1, 0x55AA0002);
    SPI_WRITE_TX(SPI1, 0x55AA0003);

    /* Set master SPI0 to half-duplex mode */
    SPI0->CTL |= SPI_CTL_HALFDPX_Msk;
    /* Enable half-duplex will produce TXFBCLR (SPIx_FIFOCTL[9]) and RXFBCLR (SPIx_FIFOCTL[8])*/
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (SPI0->STATUS & SPI_STATUS_TXRXRST_Msk)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for SPI time-out!\n");
            goto lexit;
        }
    }

    /* Set master SPI0 data direction to input */
    SPI0->CTL &= ~SPI_CTL_DATDIR_Msk;

    /* Master SPI0 receive four data from slave SPI1 */
    for (s_u32RxDataCount = 0; s_u32RxDataCount < 4; s_u32RxDataCount++)
    {
        /* Master write TX for generating clock */
        SPI_WRITE_TX(SPI0, 0);

        /* Wait for Rx FIFO not empty */
        while (SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0)) {}

        /* Read data from RX register */
        s_au32DestinationData[s_u32RxDataCount] = SPI_READ_RX(SPI0);
    }

    /* Print the received data */
    printf("SPI0 Received data:\n");

    for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        printf("%d:\t0x%X\n", u32DataCount, s_au32DestinationData[u32DataCount]);
    }

    /* Reset slave RX related flags. */
    SPI1->FIFOCTL |= SPI_FIFOCTL_RXRST_Msk;
    /* Set master SPI0 data direction to output */
    SPI0->CTL |= SPI_CTL_DATDIR_Msk;
    /* Set slave SPI1 data direction to input */
    SPI1->CTL &= ~SPI_CTL_DATDIR_Msk;

    /* Master SPI0 prepare data to TX FIFO */
    SPI_WRITE_TX(SPI0, 0xAA550000);
    SPI_WRITE_TX(SPI0, 0xAA550001);
    SPI_WRITE_TX(SPI0, 0xAA550002);
    SPI_WRITE_TX(SPI0, 0xAA550003);

    /* Slave SPI1 receive four data from master SPI0 */
    for (s_u32RxDataCount = 0; s_u32RxDataCount < 4; s_u32RxDataCount++)
    {
        /* Wait for Rx FIFO not empty */
        while (SPI_GET_RX_FIFO_EMPTY_FLAG(SPI1)) {}

        /* Read data from RX register */
        s_au32DestinationData[s_u32RxDataCount] = SPI_READ_RX(SPI1);
    }

    /* Print the received data */
    printf("SPI1 Received data:\n");

    for (u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        printf("%d:\t0x%X\n", u32DataCount, s_au32DestinationData[u32DataCount]);
    }

    printf("The data transfer was done.\n");

lexit:

    printf("\n\nExit SPI driver sample code.\n");

    /* Reset SPI0 */
    SPI_Close(SPI0);
    /* Reset SPI1 */
    SPI_Close(SPI1);

    while (1);
}
