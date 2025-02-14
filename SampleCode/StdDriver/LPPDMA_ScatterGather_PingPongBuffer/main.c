/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief
 *          Use LPPDMA to implement Ping-Pong buffer by scatter-gather mode(memory to memory).
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define LPPDMA_TEST_COUNT 50

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#if (NVT_DCACHE_ON == 1)
/* Base address and size of cache buffer must be DCACHE_LINE_SIZE byte aligned */
uint32_t g_au32SrcArray0[DCACHE_ALIGN_LINE_SIZE(4) / 4] __attribute__((aligned(DCACHE_LINE_SIZE)));
uint32_t g_au32SrcArray1[DCACHE_ALIGN_LINE_SIZE(4) / 4] __attribute__((aligned(DCACHE_LINE_SIZE)));
uint32_t g_au32DestArray[DCACHE_ALIGN_LINE_SIZE(4) / 4] __attribute__((aligned(DCACHE_LINE_SIZE)));
#else
__attribute__((aligned)) static uint32_t g_au32SrcArray0[1] = {0x55555555};
__attribute__((aligned)) static uint32_t g_au32SrcArray1[1] = {0xAAAAAAAA};
__attribute__((aligned)) static uint32_t g_au32DestArray[1];
#endif

static uint32_t volatile g_u32IsTestOver = 0;
static uint32_t volatile g_u32TransferredCount = 0;
static uint32_t g_u32DMAConfig = 0;
static uint32_t volatile g_u32DestVal = 0;

typedef struct dma_desc_t
{
    uint32_t u32Ctl;
    uint32_t u32Src;
    uint32_t u32Dest;
    uint32_t u32Offset;
} DMA_DESC_T;

/*
    The setting value of Descript Table is fixed, so it is placed in the non-cacheable section
    , eliminating the need for cache operations.
*/
NVT_NONCACHEABLE static DMA_DESC_T DMA_DESC[2]; /* Descriptor table */

/**
 * @brief       LPPDMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The LPPDMA default IRQ, declared in startup_M55M1.c.
 */
NVT_ITCM void LPPDMA_IRQHandler(void)
{
    uint32_t u32Status, u32DestVal;
#if (NVT_DCACHE_ON == 1)
    /*
       Invalidate the CPU Data cache after the DMA transfer.
       As the destination buffer may be used by the CPU, this guarantees up-to-date data when CPU access
    */
    SCB_InvalidateDCache_by_Addr(g_au32DestArray, sizeof(g_au32DestArray));
#endif  // (NVT_DCACHE_ON == 1)
    /* user must check Data validity */
    u32DestVal = g_au32DestArray[0];

    if (g_u32IsTestOver)
    {
        /* When test is over, clear the last transfer done flag */
        PDMA_CLR_TD_FLAG(LPPDMA, LPPDMA_TDSTS_TDIF2_Msk);
        printf("g_u32IsTestOver ...\n");
        return;
    }

    /* Check channel transfer done status */
    if (LPPDMA_GET_TD_STS(LPPDMA) == LPPDMA_TDSTS_TDIF2_Msk)
    {
        /* The  data is invalidate. Just wait for next transfer done and check again. */
        if (g_u32DestVal == u32DestVal)
        {
            printf(".");
            return;
        }

        g_u32DestVal = u32DestVal;
        printf("[%02d] 0x%08X\n", g_u32TransferredCount, g_au32DestArray[0]);
        /* When finished a descriptor table then g_u32TransferredCount increases 1 */
        g_u32TransferredCount++;

        /* Check if LPPDMA has finished LPPDMA_TEST_COUNT tasks */
        if (g_u32TransferredCount >= LPPDMA_TEST_COUNT)
        {
            /* Set LPPDMA into idle state by Descriptor table */
            DMA_DESC[0].u32Ctl &= ~LPPDMA_DSCT_CTL_OPMODE_Msk;
            DMA_DESC[1].u32Ctl &= ~LPPDMA_DSCT_CTL_OPMODE_Msk;
            g_u32IsTestOver = 1;
        }

        /* Clear transfer done flag of channel 2 */
        LPPDMA_CLR_TD_FLAG(LPPDMA, LPPDMA_TDSTS_TDIF2_Msk);
    }

    // CPU read interrupt flag register to wait write(clear) instruction completement.
    u32Status = LPPDMA_GET_INT_STATUS(LPPDMA);
    NVT_UNUSED(u32Status);
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to APLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);
    /* Use SystemCoreClockUpdate() to calculate and update SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Enable UART module clock */
    SetDebugUartCLK();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Enable LPPDMA clock source */
    CLK_EnableModuleClock(LPPDMA0_MODULE);
    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32TimeOutCnt;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-----------------------------------------------------------------------+ \n");
    printf("|     LPPDMA Driver Ping-Pong Buffer Sample Code (Scatter-gather)       | \n");
    printf("+-----------------------------------------------------------------------+ \n");
    /* This sample will transfer data by looped around two descriptor tables from two different source to the same destination buffer in sequence.
       And operation sequence will be table 1 -> table 2-> table 1 -> table 2 -> table 1 -> ... -> until LPPDMA configuration doesn't be reloaded. */
    /*--------------------------------------------------------------------------------------------------
      LPPDMA transfer configuration:

        Channel = 2
        Operation mode = scatter-gather mode
        First scatter-gather descriptor table = DMA_DESC[0]
        Request source = LPPDMA_MEM(memory to memory)

        Transmission flow:

                                            loop around
                                      LPPDMA_TEST_COUNT/2 times
           ------------------------                             -----------------------
          |                        | ------------------------> |                       |
          |  DMA_DESC[0]           |                           |  DMA_DESC[1]          |
          |  (Descriptor table 1)  |                           |  (Descriptor table 2) |
          |                        | <-----------------------  |                       |
           ------------------------                             -----------------------

        Note: The configuration of each table in LPSRAM need to be reloaded after transmission finished.
    --------------------------------------------------------------------------------------------------*/
    /* Open Channel 2 */
    LPPDMA_Open(LPPDMA, 1 << 2);
    /* Enable Scatter Gather mode, assign the first scatter-gather descriptor table is table 1,
       and set transfer mode as memory to memory */
    LPPDMA_SetTransferMode(LPPDMA, 2, LPPDMA_MEM, TRUE, (uint32_t)&DMA_DESC[0]);
    /* Scatter-Gather descriptor table configuration in LPSRAM */
    g_u32DMAConfig = \
                     (0 << LPPDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is 1 */ \
                     LPPDMA_WIDTH_32 |  /* Transfer width is 32 bits(one word) */ \
                     LPPDMA_SAR_FIX |   /* Source increment size is fixed(no increment) */ \
                     LPPDMA_DAR_FIX |   /* Destination increment size is fixed(no increment) */ \
                     LPPDMA_REQ_BURST | /* Transfer type is burst transfer type */ \
                     LPPDMA_BURST_1 |   /* Burst size is 128. No effect in single transfer type */ \
                     LPPDMA_OP_SCATTER; /* Operation mode is scatter-gather mode */
    /*-----------------------------------------------------------------------------------------------------------------------------------------------------------
       Note: LPPDMA_REQ_BURST is only supported in memory-to-memory transfer mode.
             LPPDMA transfer type should be set as LPPDMA_REQ_SINGLE in memory-to-peripheral and peripheral-to-memory transfer mode,
             then above code will be modified as follows:
             g_u32DMAConfig = (0 << LPPDMA_DSCT_CTL_TXCNT_Pos) | LPPDMA_WIDTH_32 | LPPDMA_SAR_FIX | LPPDMA_DAR_FIX | LPPDMA_BURST_1 | LPPDMA_REQ_SINGLE | LPPDMA_OP_SCATTER;
    -----------------------------------------------------------------------------------------------------------------------------------------------------------*/
    /*------------------------------------------------------------------------------------------------------
      Descriptor table 1 configuration:

             g_au32SrcArray0               transfer 1 times    g_au32DestArray
             ---------------------------   ----------------->  ---------------------------
            |            [0]            |                     |            [0]            |
             ---------------------------                       ---------------------------
             \                         /                       \                         /
                   32bits(one word)                                  32bits(one word)

        Operation mode = scatter-gather mode
        Next descriptor table = DMA_DESC[1](Descriptor table 2)
        transfer done and table empty interrupt = enable

        Transfer count = 1
        Transfer width = 32 bits(one word)
        Source address = g_au32SrcArray0
        Source address increment size = fixed address(no increment)
        Destination address = au8DestArray0
        Destination address increment size = fixed address(no increment)
        Transfer type = burst transfer

        Total transfer length = 1 * 32 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[0].u32Ctl = g_u32DMAConfig;
    /* Configure source address */
    DMA_DESC[0].u32Src = (uint32_t)g_au32SrcArray0; /* Ping-Pong buffer 1 */
    /* Configure destination address */
    DMA_DESC[0].u32Dest = (uint32_t)&g_au32DestArray[0];
    /* Configure next descriptor table address */
    DMA_DESC[0].u32Offset = (uint32_t)&DMA_DESC[1]; /* next operation table is table 2 */
    /*------------------------------------------------------------------------------------------------------
      Descriptor table 2 configuration:

             g_au32SrcArray1               transfer 1 times    g_au32DestArray
             ---------------------------   ----------------->  ---------------------------
            |            [0]            |                     |            [0]            |
             ---------------------------                       ---------------------------
             \                         /                       \                         /
                   32bits(one word)                                  32bits(one word)

        Operation mode = scatter-gather mode
        Next descriptor table = DMA_DESC[0](Descriptor table 1)
        transfer done and table empty interrupt = enable

        Transfer count = 1
        Transfer width = 32 bits(one word)
        Source address = g_au32SrcArray1
        Source address increment size = fixed address(no increment)
        Destination address = au8DestArray0
        Destination address increment size = fixed address(no increment)
        Transfer type = burst transfer

        Total transfer length = 1 * 32 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[1].u32Ctl = g_u32DMAConfig;
    /* Configure source address */
    DMA_DESC[1].u32Src = (uint32_t)g_au32SrcArray1; /* Ping-Pong buffer 2 */
    /* Configure destination address */
    DMA_DESC[1].u32Dest = (uint32_t)&g_au32DestArray[0];
    /* Configure next descriptor table address */
    DMA_DESC[1].u32Offset = (uint32_t)&DMA_DESC[0]; /* next operation table is table 1 */
    /* Enable transfer done interrupt */
    LPPDMA_EnableInt(LPPDMA, 2, LPPDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(LPPDMA_IRQn);
    g_u32IsTestOver = 0;
#if (NVT_DCACHE_ON == 1)
    g_au32SrcArray0[0] = 0x55555555;
    g_au32SrcArray1[0] = 0xAAAAAAAA;
    g_au32DestArray[0] = 0;
    /*
        Clean the CPU Data cache before starting the DMA transfer.
        This guarantees that the source buffer will be up to date before starting the transfer.
    */
    SCB_CleanDCache_by_Addr(g_au32SrcArray0, sizeof(g_au32SrcArray0));
    SCB_CleanDCache_by_Addr(g_au32SrcArray1, sizeof(g_au32SrcArray1));
    SCB_CleanDCache_by_Addr(g_au32DestArray, sizeof(g_au32DestArray));
#endif  // (NVT_DCACHE_ON == 1)
    /* Start LPPDMA operation */
    LPPDMA_Trigger(LPPDMA, 2);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (g_u32IsTestOver == 0)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for LPPDMA transfer done time-out!\n");
            break;
        }
    }

    if (g_u32IsTestOver == 1)
    {
        g_u32IsTestOver = 0;
        printf("test done...\n");
    }

    /* Close LPPDMA channel */
    LPPDMA_Close(LPPDMA);

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
