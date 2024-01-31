/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use LPPDMA channel 2 to transfer data from memory to memory by scatter-gather mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t LPPDMA_TEST_LENGTH = 64;

__attribute__((aligned)) static uint8_t au8SrcArray[256];
__attribute__((aligned)) static uint8_t au8DestArray0[256];
__attribute__((aligned)) static uint8_t au8DestArray1[256];

typedef struct dma_desc_t
{
    uint32_t u32Ctl;
    uint32_t u32Src;
    uint32_t u32Dest;
    uint32_t u32Offset;
} DMA_DESC_T;

static DMA_DESC_T DMA_DESC[2]; /* Descriptor table */

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
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);
    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Enable PLL0 180MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL0_SELECT);
    /* Switch SCLK clock source to PLL0 */
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
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Enable UART module clock */
    SetDebugUartCLK();
    /* Enable LPPDMA clock source */
    CLK_EnableModuleClock(LPPDMA0_MODULE);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t i, u32Src, u32Dst0, u32Dst1, u32TimeOutCnt;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-----------------------------------------------------------------------+ \n");
    printf("|    LPPDMA Memory to Memory Driver Sample Code (Scatter-gather)        | \n");
    printf("+-----------------------------------------------------------------------+ \n");
    u32Src = (uint32_t)au8SrcArray;
    u32Dst0 = (uint32_t)au8DestArray0;
    u32Dst1 = (uint32_t)au8DestArray1;

    for (i = 0; i < 0x100; i++)
    {
        au8SrcArray[i] = (uint8_t) i;
        au8DestArray0[i] = 0;
        au8DestArray1[i] = 0;
    }

    /* This sample will transfer data by finished two descriptor table in sequence.(descriptor table 1 -> descriptor table 2) */
    /*----------------------------------------------------------------------------------
      LPPDMA transfer configuration:

        Channel = 2
        Operation mode = scatter-gather mode
        First scatter-gather descriptor table = DMA_DESC[0]
        Request source = LPPDMA_MEM(memory to memory)

        Transmission flow:
           ------------------------      -----------------------
          |                        |    |                       |
          |  DMA_DESC[0]           | -> |  DMA_DESC[1]          | -> transfer done
          |  (Descriptor table 1)  |    |  (Descriptor table 2) |
          |                        |    |                       |
           ------------------------      -----------------------

    ----------------------------------------------------------------------------------*/
    /* Open Channel 2 */
    LPPDMA_Open(LPPDMA, 1 << 2);
    /* Enable Scatter Gather mode, assign the first scatter-gather descriptor table is table 1,
       and set transfer mode as memory to memory */
    LPPDMA_SetTransferMode(LPPDMA, 2, LPPDMA_MEM, 1, (uint32_t)&DMA_DESC[0]);
    /*------------------------------------------------------------------------------------------------------

                          au8SrcArray                         au8DestArray0
                          ---------------------------   -->   ---------------------------
                        /| [0]  | [1]  |  [2] |  [3] |       | [0]  | [1]  |  [2] |  [3] |\
                         |      |      |      |      |       |      |      |      |      |
      LPPDMA_TEST_LENGTH |            ...            |       |            ...            | LPPDMA_TEST_LENGTH
                         |      |      |      |      |       |      |      |      |      |
                        \| [60] | [61] | [62] | [63] |       | [60] | [61] | [62] | [63] |/
                          ---------------------------         ---------------------------
                          \                         /         \                         /
                                32bits(one word)                     32bits(one word)

      Descriptor table 1 configuration:

        Operation mode = scatter-gather mode
        Next descriptor table = DMA_DESC[1](Descriptor table 2)
        transfer done and table empty interrupt = disable

        Transfer count = LPPDMA_TEST_LENGTH
        Transfer width = 32 bits(one word)
        Source address = au8SrcArray
        Source address increment size = 32 bits(one word)
        Destination address = au8DestArray0
        Destination address increment size = 32 bits(one word)
        Transfer type = burst transfer

        Total transfer length = LPPDMA_TEST_LENGTH * 32 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[0].u32Ctl =
        ((LPPDMA_TEST_LENGTH - 1) << LPPDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is LPPDMA_TEST_LENGTH */ \
        LPPDMA_WIDTH_32 |   /* Transfer width is 32 bits(one word) */ \
        LPPDMA_SAR_INC |    /* Source increment size is 32 bits(one word) */ \
        LPPDMA_DAR_INC |    /* Destination increment size is 32 bits(one word) */ \
        LPPDMA_REQ_BURST |  /* Transfer type is burst transfer type */ \
        LPPDMA_BURST_128 |  /* Burst size is 128. No effect in single transfer type */ \
        LPPDMA_TBINTDIS_DISABLE |   /* Disable transfer done and table empty interrupt */ \
        LPPDMA_OP_SCATTER;  /* Operation mode is scatter-gather mode */
    /* Configure source address */
    DMA_DESC[0].u32Src = u32Src;
    /* Configure destination address */
    DMA_DESC[0].u32Dest = u32Dst0;
    /* Configure next descriptor table address */
    DMA_DESC[0].u32Offset = (uint32_t)&DMA_DESC[1]; /* next descriptor table is table 2 */
    /*------------------------------------------------------------------------------------------------------

                          au8DestArray0                       au8DestArray1
                          ---------------------------   -->   ---------------------------
                        /| [0]  | [1]  |  [2] |  [3] |       | [0]  | [1]  |  [2] |  [3] |\
                         |      |      |      |      |       |      |      |      |      |
      LPPDMA_TEST_LENGTH |            ...            |       |            ...            | LPPDMA_TEST_LENGTH
                         |      |      |      |      |       |      |      |      |      |
                        \| [60] | [61] | [62] | [63] |       | [60] | [61] | [62] | [63] |/
                          ---------------------------         ---------------------------
                          \                         /         \                         /
                                32bits(one word)                     32bits(one word)

      Descriptor table 2 configuration:

        Operation mode = basic mode
        transfer done and table empty interrupt = enable

        Transfer count = LPPDMA_TEST_LENGTH
        Transfer width = 32 bits(one word)
        Source address = au8DestArray0
        Source address increment size = 32 bits(one word)
        Destination address = au8DestArray1
        Destination address increment size = 32 bits(one word)
        Transfer type = burst transfer

        Total transfer length = LPPDMA_TEST_LENGTH * 32 bits
    ------------------------------------------------------------------------------------------------------*/
    DMA_DESC[1].u32Ctl =
        ((LPPDMA_TEST_LENGTH - 1) << LPPDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is LPPDMA_TEST_LENGTH */ \
        LPPDMA_WIDTH_32 |   /* Transfer width is 32 bits(one word) */ \
        LPPDMA_SAR_INC |    /* Source increment size is 32 bits(one word) */ \
        LPPDMA_DAR_INC |    /* Destination increment size is 32 bits(one word) */ \
        LPPDMA_REQ_BURST |  /* Transfer type is burst transfer type */ \
        LPPDMA_BURST_128 |  /* Burst size is 128. No effect in single transfer type */ \
        LPPDMA_OP_BASIC;    /* Operation mode is basic mode */
    DMA_DESC[1].u32Src = u32Dst0;
    DMA_DESC[1].u32Dest = u32Dst1;
    DMA_DESC[1].u32Offset = 0; /* No next operation table. No effect in basic mode */
    /* Generate a software request to trigger transfer with LPPDMA channel 2 */
    LPPDMA_Trigger(LPPDMA, 2);
    /* Waiting for transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (LPPDMA_IS_CH_BUSY(LPPDMA, 2))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for LPPDMA transfer done time-out!\n");
            break;
        }
    }

    printf("test done...\n");

    /* Compare data */
    for (i = 0; i < 0x100; i++)
    {
        if ((au8SrcArray[i] != au8DestArray0[i]) || (au8SrcArray[i] != au8DestArray1[i]))
        {
            printf(" - Compare data fail\n");
            break;
        }
    }

    /* Close LPPDMA channel */
    LPPDMA_Close(LPPDMA);

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
