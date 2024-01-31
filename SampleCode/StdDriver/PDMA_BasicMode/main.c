/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use PDMA0 channel 2 to transfer data from memory to memory.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PDMA_TEST_LENGTH 64

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
__attribute__((aligned)) static uint8_t au8SrcArray[256];
__attribute__((aligned)) static uint8_t au8DestArray[256];
static uint32_t volatile g_u32IsTestOver = 0;

NVT_ITCM void PDMA0_IRQHandler(void);

/**
 * @brief       PDMA0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PDMA0 default IRQ, declared in startup_M55M1.c.
 */
NVT_ITCM void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if (u32Status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        /* Check if channel 2 has abort error */
        if (PDMA_GET_ABORT_STS(PDMA0) & PDMA_ABTSTS_ABTIF2_Msk)
        {
            g_u32IsTestOver = 2;
        }

        /* Clear abort flag of channel 2 */
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if (u32Status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        /* Check transmission of channel 2 has been transfer done */
        if (PDMA_GET_TD_STS(PDMA0) & PDMA_TDSTS_TDIF2_Msk)
        {
            g_u32IsTestOver = 1;
        }

        /* Clear transfer done flag of channel 2 */
        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
    }

    // CPU read interrupt flag register to wait write(clear) instruction completement.
    u32Status = PDMA_GET_INT_STATUS(PDMA0);
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
    /* Enable PDMA0 clock source */
    CLK_EnableModuleClock(PDMA0_MODULE);
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
    uint32_t i, u32TimeOutCnt;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------------------+ \n");
    printf("|    PDMA Memory to Memory Driver Sample Code          | \n");
    printf("+------------------------------------------------------+ \n");
    /* Reset PDMA module */
    SYS_UnlockReg();
    SYS_ResetModule(SYS_PDMA0RST);
    SYS_LockReg();

    for (i = 0; i < 0x100; i++)
    {
        au8SrcArray[i] = (uint8_t) i;
        au8DestArray[i] = 0;
    }

    /*------------------------------------------------------------------------------------------------------

                         au8SrcArray                         au8DestArray
                         ---------------------------   -->   ---------------------------
                       /| [0]  | [1]  |  [2] |  [3] |       | [0]  | [1]  |  [2] |  [3] |\
                        |      |      |      |      |       |      |      |      |      |
       PDMA_TEST_LENGTH |            ...            |       |            ...            | PDMA_TEST_LENGTH
                        |      |      |      |      |       |      |      |      |      |
                       \| [60] | [61] | [62] | [63] |       | [60] | [61] | [62] | [63] |/
                         ---------------------------         ---------------------------
                         \                         /         \                         /
                               32bits(one word)                     32bits(one word)

      PDMA transfer configuration:

        Channel = 2
        Operation mode = basic mode
        Request source = PDMA_MEM(memory to memory)
        transfer done and table empty interrupt = enable

        Transfer count = PDMA_TEST_LENGTH
        Transfer width = 32 bits(one word)
        Source address = au8SrcArray
        Source address increment size = 32 bits(one word)
        Destination address = au8DestArray
        Destination address increment size = 32 bits(one word)
        Transfer type = burst transfer

        Total transfer length = PDMA_TEST_LENGTH * 32 bits
    ------------------------------------------------------------------------------------------------------*/
    /* Open Channel 2 */
    PDMA_Open(PDMA0, 1 << 2);
    /* Transfer count is PDMA_TEST_LENGTH, transfer width is 32 bits(one word) */
    PDMA_SetTransferCnt(PDMA0, 2, PDMA_WIDTH_32, PDMA_TEST_LENGTH);
    /* Set source address is au8SrcArray, destination address is au8DestArray, Source/Destination increment size is 32 bits(one word) */
    PDMA_SetTransferAddr(PDMA0, 2, (uint32_t)au8SrcArray, PDMA_SAR_INC, (uint32_t)au8DestArray, PDMA_DAR_INC);
    /* Request source is memory to memory */
    PDMA_SetTransferMode(PDMA0, 2, PDMA_MEM, FALSE, 0);
    /* Transfer type is burst transfer and burst size is 4 */
    PDMA_SetBurstType(PDMA0, 2, PDMA_REQ_BURST, PDMA_BURST_4);
    /* Enable interrupt */
    PDMA_EnableInt(PDMA0, 2, PDMA_INT_TRANS_DONE);
    /* Enable NVIC for PDMA */
    NVIC_EnableIRQ(PDMA0_IRQn);
    g_u32IsTestOver = 0;
    /* Generate a software request to trigger transfer with PDMA channel 2  */
    PDMA_Trigger(PDMA0, 2);
    /* Waiting for transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (g_u32IsTestOver == 0)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA transfer done time-out!\n");
            break;
        }
    }

    /* Check transfer result */
    if (g_u32IsTestOver == 1)
    {
        printf("test done...\n");

        /* Compare data */
        for (i = 0; i < 0x100; i++)
        {
            if (au8SrcArray[i] != au8DestArray[i])
            {
                printf(" - Compare data fail\n");
                break;
            }
        }
    }
    else if (g_u32IsTestOver == 2)
    {
        printf("target abort...\n");
    }

    /* Close channel 2 */
    PDMA_Close(PDMA0);

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
