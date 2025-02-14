/******************************************************************************
 * @file    main.c
 * @version V1.00
 * @brief   SPI read/write demo in PDMA mode.
 *          Connecting SPI MISO and MOSI pins.
 *          Both TX PDMA function and RX PDMA function will be enabled.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

//------------------------------------------------------------------------------
#define SPI_MASTER_TX_DMA_CH 0
#define SPI_MASTER_RX_DMA_CH 1
#define SPI_OPENED_CH   ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH))

#define DATA_COUNT      32
#define TEST_CYCLE      1000
#define TEST_PATTERN    0x55000000
#define SPI_CLK_FREQ    2000000

//------------------------------------------------------------------------------
/* Global variable declaration */
/* Buffer for SPI0 data transfer with PDMA */
#if (NVT_DCACHE_ON == 1)
/* DCache-line aligned buffer for SPI0 data transfer with PDMA */
uint32_t g_au32MasterToSlaveTestPattern[DCACHE_ALIGN_LINE_SIZE(DATA_COUNT)] __attribute__((aligned(DCACHE_LINE_SIZE))) = {0};
uint32_t g_au32MasterRxBuffer[DCACHE_ALIGN_LINE_SIZE(DATA_COUNT)] __attribute__((aligned(DCACHE_LINE_SIZE))) = {0};
#else
/* Buffer for SPI0 data transfer with PDMA when DCache is disabled */
uint32_t g_au32MasterToSlaveTestPattern[DATA_COUNT];
uint32_t g_au32MasterRxBuffer[DATA_COUNT];
#endif

//------------------------------------------------------------------------------
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

    /* Select PCLK0 as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_SPISEL_SPI0SEL_PCLK0, MODULE_NoMsk);

    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA0_MODULE);

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

void SpiLoopTest_WithPDMA(void)
{
    uint32_t u32DataCount, u32TestCycle;
    uint32_t u32RegValue, u32Abort;
    int32_t i32Err;
    uint32_t u32TimeOutCount;

    printf("\nSPI0 Loopback test with PDMA \n");

#if (NVT_DCACHE_ON == 1)
    /* If DCACHE is enabled, clean the data cache for the master to slave test pattern before using it */
    /* This is to ensure that the data written to the cache is actually written to the memory */
    SCB_CleanDCache_by_Addr((uint32_t *)&g_au32MasterToSlaveTestPattern, sizeof(g_au32MasterToSlaveTestPattern));
#endif

    /* Source data initiation */
    for (u32DataCount = 0; u32DataCount < DATA_COUNT; u32DataCount++)
    {
        g_au32MasterToSlaveTestPattern[u32DataCount] = TEST_PATTERN | (u32DataCount + 1);
    }

    /* Reset PDMA module */
    SYS_ResetModule(SYS_PDMA0RST);

    /* Enable PDMA channels */
    PDMA_Open(PDMA0, SPI_OPENED_CH);

    /*=======================================================================
      SPI master PDMA TX channel configuration:
      -----------------------------------------------------------------------
        Word length = 32 bits
        Transfer Count = DATA_COUNT
        Source = g_au32MasterToSlaveTestPattern
        Source Address = Incresing
        Destination = SPI0->TX
        Destination Address = Fixed
        Burst Type = Single Transfer
    =========================================================================*/
    /* Set transfer width (32 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA0, SPI_MASTER_TX_DMA_CH, PDMA_WIDTH_32, DATA_COUNT);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA0, SPI_MASTER_TX_DMA_CH, (uint32_t)g_au32MasterToSlaveTestPattern, PDMA_SAR_INC, (uint32_t)&SPI0->TX, PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, SPI_MASTER_TX_DMA_CH, PDMA_SPI0_TX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA0, SPI_MASTER_TX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA0->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    /*=======================================================================
      SPI master PDMA RX channel configuration:
      -----------------------------------------------------------------------
        Word length = 32 bits
        Transfer Count = DATA_COUNT
        Source = SPI0->RX
        Source Address = Fixed
        Destination = g_au32MasterRxBuffer
        Destination Address = Increasing
        Burst Type = Single Transfer
    =========================================================================*/
#if (NVT_DCACHE_ON == 1)
    /* If DCACHE is enabled, clean the data cache for the destination buffer before writing to it */
    /* This is to ensure that the data written to the cache is actually written to the memory */
    SCB_InvalidateDCache_by_Addr((uint32_t *)&g_au32MasterRxBuffer, sizeof(g_au32MasterRxBuffer));
#endif
    /* Set transfer width (32 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA0, SPI_MASTER_RX_DMA_CH, PDMA_WIDTH_32, DATA_COUNT);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA0, SPI_MASTER_RX_DMA_CH, (uint32_t)&SPI0->RX, PDMA_SAR_FIX, (uint32_t)g_au32MasterRxBuffer, PDMA_DAR_INC);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, SPI_MASTER_RX_DMA_CH, PDMA_SPI0_RX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA0, SPI_MASTER_RX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA0->DSCT[SPI_MASTER_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    /* Enable SPI master DMA function */
    SPI_TRIGGER_TX_PDMA(SPI0);
    SPI_TRIGGER_RX_PDMA(SPI0);

    i32Err = 0;

    for (u32TestCycle = 0; u32TestCycle < TEST_CYCLE; u32TestCycle++)
    {
        if ((u32TestCycle & 0x1FF) == 0)
            printf(".");

        /* setup timeout */
        u32TimeOutCount = SystemCoreClock;

        while (1)
        {
            /* Get interrupt status */
            u32RegValue = PDMA_GET_INT_STATUS(PDMA0);

            /* Check the PDMA transfer done interrupt flag */
            if (u32RegValue & PDMA_INTSTS_TDIF_Msk)
            {
                /* Check the PDMA transfer done flags */
                if ((PDMA_GET_TD_STS(PDMA0) & SPI_OPENED_CH) == SPI_OPENED_CH)
                {
                    /* Clear the PDMA transfer done flags */
                    PDMA_CLR_TD_FLAG(PDMA0, SPI_OPENED_CH);
                    /* Disable SPI master's PDMA transfer function */
                    SPI_DISABLE_TX_PDMA(SPI0);
                    SPI_DISABLE_RX_PDMA(SPI0);

#if (NVT_DCACHE_ON == 1)
                    /* If DCACHE is enabled, invalidate the data cache for the source buffer */
                    /* This is to ensure that the data written to the cache is actually written to the memory */
                    SCB_InvalidateDCache_by_Addr((uint32_t *)&g_au32MasterToSlaveTestPattern, sizeof(g_au32MasterToSlaveTestPattern));
#endif

                    /* Check the transfer data */
                    for (u32DataCount = 0; u32DataCount < DATA_COUNT; u32DataCount++)
                    {
                        if (g_au32MasterToSlaveTestPattern[u32DataCount] != g_au32MasterRxBuffer[u32DataCount])
                        {
                            i32Err = 1;
                            break;
                        }
                    }

                    if (u32TestCycle >= TEST_CYCLE)
                        break;

                    /* Source data initiation */
                    for (u32DataCount = 0; u32DataCount < DATA_COUNT; u32DataCount++)
                    {
                        g_au32MasterToSlaveTestPattern[u32DataCount]++;
                    }

                    /* Re-trigger */
                    /* Master PDMA TX channel configuration */
                    /* Set transfer width (32 bits) and transfer count */
                    PDMA_SetTransferCnt(PDMA0, SPI_MASTER_TX_DMA_CH, PDMA_WIDTH_32, DATA_COUNT);
                    /* Set request source; set basic mode. */
                    PDMA_SetTransferMode(PDMA0, SPI_MASTER_TX_DMA_CH, PDMA_SPI0_TX, FALSE, 0);

                    /* Master PDMA RX channel configuration */
                    /* Set transfer width (32 bits) and transfer count */
                    PDMA_SetTransferCnt(PDMA0, SPI_MASTER_RX_DMA_CH, PDMA_WIDTH_32, DATA_COUNT);
                    /* Set request source; set basic mode. */
                    PDMA_SetTransferMode(PDMA0, SPI_MASTER_RX_DMA_CH, PDMA_SPI0_RX, FALSE, 0);

                    /* Enable master's DMA transfer function */
                    SPI_TRIGGER_TX_PDMA(SPI0);
                    SPI_TRIGGER_RX_PDMA(SPI0);
                    break;
                }
            }

            /* Check the DMA transfer abort interrupt flag */
            if (u32RegValue & PDMA_INTSTS_ABTIF_Msk)
            {
                /* Get the target abort flag */
                u32Abort = PDMA_GET_ABORT_STS(PDMA0);
                /* Clear the target abort flag */
                PDMA_CLR_ABORT_FLAG(PDMA0, u32Abort);
                i32Err = 1;
                break;
            }

            /* Check the DMA time-out interrupt flag */
            if (u32RegValue & (PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk))
            {
                /* Clear the time-out flag */
                PDMA0->INTSTS = u32RegValue & (PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk);
                i32Err = 1;
                break;
            }

            if (u32TimeOutCount == 0)
            {
                printf("\nSomething is wrong, please check if pin connection is correct. \n");

                while (1);
            }

            u32TimeOutCount--;
        }

        if (i32Err)
            break;
    }

    /* Disable all PDMA channels */
    PDMA_Close(PDMA0);

    if (i32Err)
    {
        printf(" [FAIL]\n");
    }
    else
    {
        printf(" [PASS]\n");
    }

    return;
}

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Init SPI */
    SPI_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------------+\n");
    printf("|                  SPI + PDMA Sample Code                      |\n");
    printf("+--------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure SPI0 as a master.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for loopback test:\n");
    printf("    SPI0_MISO(PA.1) <--> SPI0_MOSI(PA.0)\n\n");
    printf("Please press any key to start transmission ...\n");
    getchar();
    printf("\n");

    SpiLoopTest_WithPDMA();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nExit SPI driver sample code.\n");

    /* Close SPI0 */
    SPI_Close(SPI0);

    while (1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
