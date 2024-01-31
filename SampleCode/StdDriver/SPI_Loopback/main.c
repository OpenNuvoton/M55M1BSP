/******************************************************************************
 * @file    main.c
 * @version V1.00
 * @brief   Implement SPI Master loop back transfer.
 *          This sample code needs to connect MISO pin and MOSI pin together.
 *          It will compare the received data with transmitted data.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

#define TEST_CYCLE                  (10)
#define DATA_COUNT                  (64)
#define SPI_CLK_FREQ                (2000000)

#define SPI_TEST_PORT               SPI0

static uint32_t g_au32SourceData[DATA_COUNT] = {0};
static uint32_t g_au32DestinationData[DATA_COUNT] = {0};

/* Function prototype declaration */
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

    /* Select PCLK0 as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_SPISEL_SPI0SEL_PCLK0, MODULE_NoMsk);

    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Enable GPIO Module clock */
    CLK_EnableModuleClock(GPIOA_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Setup SPI0 multi-function pins */
    SYS->GPA_MFP0 |= (SYS_GPA_MFP0_PA0MFP_SPI0_MOSI |
                      SYS_GPA_MFP0_PA1MFP_SPI0_MISO |
                      SYS_GPA_MFP0_PA2MFP_SPI0_CLK  |
                      SYS_GPA_MFP0_PA3MFP_SPI0_SS);

    /* Enable SPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* Enable SPI0 I/O high slew rate */
    GPIO_SetSlewCtl(PA, 0x3F, GPIO_SLEWCTL_HIGH);

}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    SPI_Open(SPI_TEST_PORT, SPI_MASTER, SPI_MODE_0, 32, SPI_CLK_FREQ);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI_TEST_PORT, SPI_SS, SPI_SS_ACTIVE_LOW);
}

/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    uint32_t u32DataCount = 0, u32TestCycle = 0, u32Err = 0;
    uint32_t u32TimeOutCount = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init SPI */
    SPI_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("|                       SPI Driver Sample Code                       |\n");
    printf("+--------------------------------------------------------------------+\n");
    printf("\n");
    printf("\nThis sample code demonstrates SPI0 self loop back data transfer.\n");
    printf(" SPI0 configuration:\n");
    printf("     Master mode; data width 32 bits.\n");
    printf(" I/O connection:\n");
    printf("     SPI0_MOSI(PA.0) <--> SPI0_MISO(PA.1) \n");
    printf("\nSPI0 Loopback test \n\n");

    printf("Please hit any key to start test.\n\n");
    getchar();

    u32Err = 0;

    /* set the source data and clear the destination buffer */
    for (u32DataCount = 0; u32DataCount < DATA_COUNT; u32DataCount++)
    {
        g_au32SourceData[u32DataCount] = u32DataCount;
        g_au32DestinationData[u32DataCount] = 0;
    }

    for (u32TestCycle = 0; u32TestCycle < TEST_CYCLE; u32TestCycle++)
    {
        memset(g_au32DestinationData, 0, sizeof(g_au32DestinationData));
        u32DataCount = 0;

        putchar('.');


        while (1)
        {
            /* setup timeout */
            u32TimeOutCount = SystemCoreClock;

            /* Write to TX register */
            SPI_WRITE_TX(SPI_TEST_PORT, g_au32SourceData[u32DataCount]);

            /* Check SPI0 busy status */
            while (SPI_IS_BUSY(SPI_TEST_PORT))
            {
                if (u32TimeOutCount-- <= 0)
                {
                    printf("\nSomething is wrong, please check if pin connection is correct. \n");

                    while (1);
                }
            }

            /* Read received data */
            g_au32DestinationData[u32DataCount++] = SPI_READ_RX(SPI_TEST_PORT);

            if (u32DataCount >= DATA_COUNT)
                break;
        }

        /*  Check the received data */
        for (u32DataCount = 0; u32DataCount < DATA_COUNT; u32DataCount++)
        {
            if (g_au32DestinationData[u32DataCount] != g_au32SourceData[u32DataCount])
                u32Err = 1;
        }

        if (u32Err)
            break;
    }

    if (u32Err)
        printf(" [FAIL]\n\n");
    else
        printf(" [PASS]\n\n");

    /* Close SPI0 */
    SPI_Close(SPI_TEST_PORT);

    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
