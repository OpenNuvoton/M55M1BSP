/**************************************************************************//**
 * @file    main.c
 * @version V1.0
 * @brief   Demonstrate LPSPI data transfer with LPPDMA.
 *          LPSPI0 will be configured as Master mode and connect MISO_0 pin and MOSI_0 pin together.
 *          Both TX LPPDMA function and RX LPPDMA function will be enabled.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define LPSPI_MASTER_TX_DMA_CH 0
#define LPSPI_MASTER_RX_DMA_CH 1
#define LPSPI_OPENED_CH   ((1 << LPSPI_MASTER_TX_DMA_CH) | (1 << LPSPI_MASTER_RX_DMA_CH))

#define DATA_COUNT      32
#define TEST_CYCLE      0x10
#define TEST_PATTERN    0x55000000
#define LPSPI_CLK_FREQ  2000000

/* Function prototype declaration */
void SYS_Init(void);
void LPSPI_Init(void);
void LPSPILoopTest_WithLPPDMA(void);

/* Global variable declaration */
uint32_t g_au32MasterToSlaveTestPattern[DATA_COUNT] __attribute__((section(".lpSram")));
uint32_t g_au32MasterRxBuffer[DATA_COUNT] __attribute__((section(".lpSram")));

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Init LPSPI */
    LPSPI_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\n");
    printf("+--------------------------------------------------------------+\n");
    printf("|                  LPSPI + LPPDMA Sample Code                  |\n");
    printf("+--------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure LPSPI0 as a master.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for loopback test:\n");
    printf("    LPSPI0_MISO(PA.1) <--> LPSPI0_MOSI(PA.0)\n\n");
    printf("Please press any key to start transmission ...\n");
    getchar();
    printf("\n");

    LPSPILoopTest_WithLPPDMA();

    printf("\n\nExit SPI driver sample code.\n");

    /* Close LPSPI0 */
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

    /* Select PCLK1 as the clock source of LPSPI0 */
    CLK_SetModuleClock(LPSPI0_MODULE, CLK_LPSPISEL_LPSPI0SEL_HIRC, MODULE_NoMsk);

    /* Enable LPSPI0 peripheral clock */
    CLK_EnableModuleClock(LPSPI0_MODULE);

    /* Enable LPPDMA clock source */
    CLK_EnableModuleClock(LPPDMA0_MODULE);

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
    /* Init LPSPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. LPSPI clock rate = 2 MHz */
    LPSPI_Open(LPSPI0, LPSPI_MASTER, LPSPI_MODE_0, 32, LPSPI_CLK_FREQ);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    LPSPI_EnableAutoSS(LPSPI0, LPSPI_SS, LPSPI_SS_ACTIVE_LOW);
}

void LPSPILoopTest_WithLPPDMA(void)
{
    uint32_t u32DataCount, u32TestCycle;
    uint32_t u32RegValue, u32Abort;
    int32_t i32Err;
    uint32_t u32TimeOutCount;

    printf("\nLPSPI0 Loopback test with LPPDMA \n");

    /* Source data initiation */
    for (u32DataCount = 0; u32DataCount < DATA_COUNT; u32DataCount++)
    {
        g_au32MasterToSlaveTestPattern[u32DataCount] = TEST_PATTERN | (u32DataCount + 1);
    }

    /* Reset LPPDMA module */
    SYS_ResetModule(SYS_LPPDMA0RST);

    /* Enable LPPDMA channels */
    LPPDMA_Open(LPPDMA, LPSPI_OPENED_CH);

    /*=======================================================================
      LPSPI master LPPDMA TX channel configuration:
      -----------------------------------------------------------------------
        Word length = 32 bits
        Transfer Count = DATA_COUNT
        Source = g_au32MasterToSlaveTestPattern
        Source Address = Incresing
        Destination = LPSPI0->TX
        Destination Address = Fixed
        Burst Type = Single Transfer
    =========================================================================*/
    /* Set transfer width (32 bits) and transfer count */
    LPPDMA_SetTransferCnt(LPPDMA, LPSPI_MASTER_TX_DMA_CH, LPPDMA_WIDTH_32, DATA_COUNT);
    /* Set source/destination address and attributes */
    LPPDMA_SetTransferAddr(LPPDMA, LPSPI_MASTER_TX_DMA_CH, (uint32_t)g_au32MasterToSlaveTestPattern, LPPDMA_SAR_INC, (uint32_t)&LPSPI0->TX, LPPDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    LPPDMA_SetTransferMode(LPPDMA, LPSPI_MASTER_TX_DMA_CH, LPPDMA_LPSPI0_TX, FALSE, 0);
    /* Single request type. LPSPI only support LPPDMA single request type. */
    LPPDMA_SetBurstType(LPPDMA, LPSPI_MASTER_TX_DMA_CH, LPPDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    LPPDMA->LPDSCT[LPSPI_MASTER_TX_DMA_CH].CTL |= LPPDMA_DSCT_CTL_TBINTDIS_Msk;

    /*=======================================================================
      LPSPI master LPPDMA RX channel configuration:
      -----------------------------------------------------------------------
        Word length = 32 bits
        Transfer Count = DATA_COUNT
        Source = LPSPI0->RX
        Source Address = Fixed
        Destination = g_au32MasterRxBuffer
        Destination Address = Increasing
        Burst Type = Single Transfer
    =========================================================================*/
    /* Set transfer width (32 bits) and transfer count */
    LPPDMA_SetTransferCnt(LPPDMA, LPSPI_MASTER_RX_DMA_CH, LPPDMA_WIDTH_32, DATA_COUNT);
    /* Set source/destination address and attributes */
    LPPDMA_SetTransferAddr(LPPDMA, LPSPI_MASTER_RX_DMA_CH, (uint32_t)&LPSPI0->RX, LPPDMA_SAR_FIX, (uint32_t)g_au32MasterRxBuffer, LPPDMA_DAR_INC);
    /* Set request source; set basic mode. */
    LPPDMA_SetTransferMode(LPPDMA, LPSPI_MASTER_RX_DMA_CH, LPPDMA_LPSPI0_RX, FALSE, 0);
    /* Single request type. LPSPI only support LPPDMA single request type. */
    LPPDMA_SetBurstType(LPPDMA, LPSPI_MASTER_RX_DMA_CH, LPPDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    LPPDMA->LPDSCT[LPSPI_MASTER_RX_DMA_CH].CTL |= LPPDMA_DSCT_CTL_TBINTDIS_Msk;

    /* Enable LPSPI master DMA function */
    LPSPI_TRIGGER_TX_PDMA(LPSPI0);
    LPSPI_TRIGGER_RX_PDMA(LPSPI0);

    i32Err = 0;

    for (u32TestCycle = 0; u32TestCycle < TEST_CYCLE; u32TestCycle++)
    {
        putchar('.');

        /* setup timeout */
        u32TimeOutCount = SystemCoreClock;

        while (1)
        {
            /* Get interrupt status */
            u32RegValue = LPPDMA_GET_INT_STATUS(LPPDMA);

            /* Check the LPPDMA transfer done interrupt flag */
            if (u32RegValue & LPPDMA_INTSTS_TDIF_Msk)
            {
                /* Check the LPPDMA transfer done flags */
                if ((LPPDMA_GET_TD_STS(LPPDMA) & LPSPI_OPENED_CH) == LPSPI_OPENED_CH)
                {
                    /* Clear the LPPDMA transfer done flags */
                    LPPDMA_CLR_TD_FLAG(LPPDMA, LPSPI_OPENED_CH);
                    /* Disable LPSPI master's LPPDMA transfer function */
                    LPSPI_DISABLE_TX_PDMA(LPSPI0);
                    LPSPI_DISABLE_RX_PDMA(LPSPI0);

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
                    /* Master LPPDMA TX channel configuration */
                    /* Set transfer width (32 bits) and transfer count */
                    LPPDMA_SetTransferCnt(LPPDMA, LPSPI_MASTER_TX_DMA_CH, LPPDMA_WIDTH_32, DATA_COUNT);
                    /* Set request source; set basic mode. */
                    LPPDMA_SetTransferMode(LPPDMA, LPSPI_MASTER_TX_DMA_CH, LPPDMA_LPSPI0_TX, FALSE, 0);

                    /* Master LPPDMA RX channel configuration */
                    /* Set transfer width (32 bits) and transfer count */
                    LPPDMA_SetTransferCnt(LPPDMA, LPSPI_MASTER_RX_DMA_CH, LPPDMA_WIDTH_32, DATA_COUNT);
                    /* Set request source; set basic mode. */
                    LPPDMA_SetTransferMode(LPPDMA, LPSPI_MASTER_RX_DMA_CH, LPPDMA_LPSPI0_RX, FALSE, 0);

                    /* Enable master's LPPDMA transfer function */
                    LPSPI_TRIGGER_TX_PDMA(LPSPI0);
                    LPSPI_TRIGGER_RX_PDMA(LPSPI0);
                    break;
                }
            }

            /* Check the LPPDMA transfer abort interrupt flag */
            if (u32RegValue & LPPDMA_INTSTS_ABTIF_Msk)
            {
                /* Get the target abort flag */
                u32Abort = LPPDMA_GET_ABORT_STS(LPPDMA);
                /* Clear the target abort flag */
                LPPDMA_CLR_ABORT_FLAG(LPPDMA, u32Abort);
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

    /* Disable all LPPDMA channels */
    LPPDMA_Close(LPPDMA);

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

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

