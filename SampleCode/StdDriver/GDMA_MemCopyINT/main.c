/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use GDMA channel 0 to transfer data from memory to memory.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define GDMA_TEST_LENGTH 64

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#if (NVT_DCACHE_ON == 1)
    /* Base address and size of cache buffer must be DCACHE_LINE_SIZE byte aligned */
    uint8_t au8SrcArray[DCACHE_ALIGN_LINE_SIZE(GDMA_TEST_LENGTH)] __attribute__((aligned(DCACHE_LINE_SIZE)));
    uint8_t au8DestArray[DCACHE_ALIGN_LINE_SIZE(GDMA_TEST_LENGTH)] __attribute__((aligned(DCACHE_LINE_SIZE)));
#else
    __attribute__((aligned)) static uint8_t au8SrcArray[GDMA_TEST_LENGTH];
    __attribute__((aligned)) static uint8_t au8DestArray[GDMA_TEST_LENGTH];
#endif
static uint32_t volatile g_u32IsTestOver = 0;

/**
 * @brief       GDMA0 Channel 0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The GDMA0 Channel 0 default IRQ, declared in startup_M55M1.c.
 */
NVT_ITCM void GDMACH0_IRQHandler(void)
{
    union dma350_ch_status_t status = dma350_ch_get_status(GDMA_CH_DEV_S[0]);

    if (status.b.STAT_DONE)
    {
        GDMA_CH_DEV_S[0]->cfg.ch_base->CH_STATUS = DMA350_CH_STAT_DONE;
        g_u32IsTestOver = 1;
    }
    else
    {
        g_u32IsTestOver = 2;
    }

    status = dma350_ch_get_status(GDMA_CH_DEV_S[0]);
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
    /* Enable GDMA0 clock source */
    CLK_EnableModuleClock(GDMA0_MODULE);
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
    printf("+--------------------------------------------------------------+ \n");
    printf("|    GDMA Memory to Memory Driver Sample Code (Interrupt Mode) | \n");
    printf("+--------------------------------------------------------------+ \n");
    /* Reset GDMA module */
    SYS_UnlockReg();
    SYS_ResetModule(SYS_GDMA0RST);
    SYS_LockReg();

    for (i = 0; i < GDMA_TEST_LENGTH; i++)
    {
        au8SrcArray[i] = (uint8_t) i;
        au8DestArray[i] = 0;
    }

#if (NVT_DCACHE_ON == 1)
    /*
        Clean the CPU Data cache before starting the DMA transfer.
        This guarantees that the source buffer will be up to date before starting the transfer.
    */
    SCB_CleanDCache_by_Addr(au8SrcArray, sizeof(au8SrcArray));
    SCB_CleanDCache_by_Addr(au8DestArray, sizeof(au8DestArray));
#endif  // (NVT_DCACHE_ON == 1)
    /*------------------------------------------------------------------------------------------------------

                         au8SrcArray           au8DestArray
                         ------------    -->   -------------
                       /|    [0]     |         |    [0]     |\
                        |    [1]     |         |    [1]     |
       GDMA_TEST_LENGTH |    ...     |         |    ...     | GDMA_TEST_LENGTH
                        |    [62]    |         |    [62]    |
                       \|    [63]    |         |    [63]    |/
                         ------------           ------------
                         \          /           \          /
                           one byte               one byte

      GDMA transfer configuration:

        Channel = 0
        Operation mode = basic mode

        Transfer count = GDMA_TEST_LENGTH
        Transfer width = 8 bits(one byte)
        Source address = au8SrcArray
        Source address increment size = 8 bits(one byte)
        Destination address = au8DestArray
        Destination address increment size = 8 bits(one byte)

        Total transfer length = GDMA_TEST_LENGTH * 8 bits
    ------------------------------------------------------------------------------------------------------*/
    /* Initializes GDMA */
    dma350_init(&GDMA_DEV_S);
    /* Enable NVIC for GDMA CH0 */
    NVIC_EnableIRQ(GDMACH0_IRQn);
    g_u32IsTestOver = 0;
    dma350_memcpy(GDMA_CH_DEV_S[0],
                  au8SrcArray, au8DestArray,
                  GDMA_TEST_LENGTH,
                  DMA350_LIB_EXEC_IRQ);
    /* Waiting for transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (g_u32IsTestOver == 0)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for GDMA CH0 transfer done time-out!\n");
            break;
        }
    }

    /* Check transfer result */
    if (g_u32IsTestOver == 1)
    {
#if (NVT_DCACHE_ON == 1)
        /*
           Invalidate the CPU Data cache after the DMA transfer.
           As the destination buffer may be used by the CPU, this guarantees up-to-date data when CPU access
        */
        SCB_InvalidateDCache_by_Addr(au8DestArray, sizeof(au8DestArray));
#endif  // (NVT_DCACHE_ON == 1)
        printf("test done...\n");

        /* Compare data */
        for (i = 0; i < GDMA_TEST_LENGTH; i++)
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
        printf("unknown interrupt !!\n");
    }

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
