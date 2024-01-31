/*************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Configure USCI_SPI0 as slave mode and demonstrate how to communicate
 *          with an off-chip SPI master device.
 *          This sample code needs to work with USCI_SPI_MasterMode sample code.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

//------------------------------------------------------------------------------
#define TEST_COUNT  16

//------------------------------------------------------------------------------
uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];

//------------------------------------------------------------------------------
/* Function prototype declaration */
void SYS_Init(void);
void USCI_SPI_Init(void);

//------------------------------------------------------------------------------
int main()
{
    uint32_t u32TxDataCount, u32RxDataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Init USCI_SPI0 */
    USCI_SPI_Init();

    printf("\n\n");
    printf("+-----------------------------------------------------+\n");
    printf("|           USCI_SPI Slave Mode Sample Code           |\n");
    printf("+-----------------------------------------------------+\n");
    printf("\n");
    printf("Configure USCI_SPI0 as a slave.\n");
    printf("Bit length of a transaction: 16\n");
    printf("The I/O connection for USCI_SPI0:\n");
    printf("    USCI_SPI0_SS (PB.0)\n    USCI_SPI0_CLK (PA.11)\n");
    printf("    USCI_SPI0_MISO (PA.9)\n    USCI_SPI0_MOSI (PA.10)\n\n");
    printf("USCI_SPI controller will transfer %d data to a off-chip master device.\n", TEST_COUNT);
    printf("In the meanwhile the USCI_SPI controller will receive %d data from the off-chip master device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);

    for (u32TxDataCount = 0; u32TxDataCount < TEST_COUNT; u32TxDataCount++)
    {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32TxDataCount] = 0xAA00 + u32TxDataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32TxDataCount] = 0;
    }

    u32TxDataCount = 0;
    u32RxDataCount = 0;
    printf("Press any key if the master device configuration is ready.");
    getchar();
    printf("\n");

    /* Access TX and RX Buffer */
    while (u32RxDataCount < TEST_COUNT)
    {
        /* Check TX FULL flag and TX data count */
        if ((USPI_GET_TX_FULL_FLAG(USPI0) == 0) && (u32TxDataCount < TEST_COUNT))
            USPI_WRITE_TX(USPI0, g_au32SourceData[u32TxDataCount++]); /* Write to TX Buffer */

        /* Check RX EMPTY flag */
        if (USPI_GET_RX_EMPTY_FLAG(USPI0) == 0)
            g_au32DestinationData[u32RxDataCount++] = USPI_READ_RX(USPI0); /* Read RX Buffer */
    }

    /* Print the received data */
    printf("Received data:\n");

    for (u32RxDataCount = 0; u32RxDataCount < TEST_COUNT; u32RxDataCount++)
    {
        printf("%d:\t0x%X\n", u32RxDataCount, g_au32DestinationData[u32RxDataCount]);
    }

    printf("The data transfer was done.\n");

    printf("\n\nExit USCI_SPI driver sample code.\n");

    /* Close USCI_SPI0 */
    USPI_Close(USPI0);

    /* Lock protected registers */
    SYS_LockReg();

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

    CLK_EnableModuleClock(USCI0_MODULE);

    /* Enable GPIO Module Clock */
    CLK_EnableModuleClock(GPIOA_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set USCI0_SPI multi-function pins */
    SYS->GPA_MFP2 = SYS->GPA_MFP2 & ~(SYS_GPA_MFP2_PA9MFP_Msk | SYS_GPA_MFP2_PA10MFP_Msk | SYS_GPA_MFP2_PA11MFP_Msk);
    SYS->GPA_MFP2 = SYS->GPA_MFP2 | (SYS_GPA_MFP2_PA11MFP_USCI0_CLK | SYS_GPA_MFP2_PA10MFP_USCI0_DAT0 | SYS_GPA_MFP2_PA9MFP_USCI0_DAT1);
    SYS->GPB_MFP0 = SYS->GPB_MFP0 & ~(SYS_GPB_MFP0_PB0MFP_Msk);
    SYS->GPB_MFP0 = SYS->GPB_MFP0 | (SYS_GPB_MFP0_PB0MFP_USCI0_CTL0);

    /* USCI_SPI clock pin enable schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN0_Msk;

}

void USCI_SPI_Init(void)
{

    /* Configure USCI_SPI0 as a slave, USCI_SPI0 clock rate = f_PCLK0,
       clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    USPI_Open(USPI0, USPI_SLAVE, USPI_MODE_0, 16, 0);
    /* Configure USCI_SPI_SS pin as low-active. */
    USPI0->CTLIN0 = (USPI0->CTLIN0 & ~USPI_CTLIN0_ININV_Msk) | USPI_CTLIN0_ININV_Msk;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
