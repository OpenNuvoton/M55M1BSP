/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use GDMA channel 0 to do Command linking transfer.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define GDMA_TEST_LENGTH 32

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#if (NVT_DCACHE_ON == 1)
    /* Base address and size of cache buffer must be DCACHE_LINE_SIZE byte aligned */
    uint8_t au8SrcArray[DCACHE_ALIGN_LINE_SIZE(256)] __attribute__((aligned(DCACHE_LINE_SIZE)));
    uint8_t au8DestArray0[DCACHE_ALIGN_LINE_SIZE(256)] __attribute__((aligned(DCACHE_LINE_SIZE)));
    uint8_t au8DestArray1[DCACHE_ALIGN_LINE_SIZE(256)] __attribute__((aligned(DCACHE_LINE_SIZE)));
    uint32_t au32Commands[DCACHE_ALIGN_LINE_SIZE(256 * 4) / 4] __attribute__((aligned(DCACHE_LINE_SIZE)));
#else
    __attribute__((aligned)) static uint8_t au8SrcArray[256];
    __attribute__((aligned)) static uint8_t au8DestArray0[256];
    __attribute__((aligned)) static uint8_t au8DestArray1[256];
    __attribute__((aligned)) static uint32_t au32Commands[256];
#endif

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
    enum dma350_lib_error_t lib_err;
    struct dma350_cmdlink_gencfg_t cmdlink_cfg;
    uint32_t i;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-------------------------------------+ \n");
    printf("|    GDMA Command Link Sample         | \n");
    printf("+-------------------------------------+ \n");
    /* Reset GDMA module */
    SYS_UnlockReg();
    SYS_ResetModule(SYS_GDMA0RST);
    SYS_LockReg();

    for (i = 0; i < 0x100; i++)
    {
        au8SrcArray[i] = (uint8_t) i;
        au8DestArray0[i] = 0;
        au8DestArray1[i] = 0;
    }

    /* Initializes GDMA */
    dma350_init(&GDMA_DEV_S);
    dma350_cmdlink_init(&cmdlink_cfg);
    /* prepare extearnl command setting at SRAM */
    dma350_cmdlink_set_srcaddr32(&cmdlink_cfg, (uint32_t)au8DestArray0);
    dma350_cmdlink_set_desaddr32(&cmdlink_cfg, (uint32_t)au8DestArray1);
    dma350_cmdlink_set_xsize16(&cmdlink_cfg, (uint16_t)GDMA_TEST_LENGTH, (uint16_t)GDMA_TEST_LENGTH);
    dma350_cmdlink_set_linkaddr32(&cmdlink_cfg, 0); /* Link disabled at the last command */
    dma350_cmdlink_generate(&cmdlink_cfg, au32Commands, au32Commands + 0x20 - 1);
    /* Link to external command */
    dma350_ch_enable_linkaddr(GDMA_CH_DEV_S[0]);
    dma350_ch_set_linkaddr32(GDMA_CH_DEV_S[0], (uint32_t) au32Commands);
#if (NVT_DCACHE_ON == 1)
    /*
        Clean the CPU Data cache before starting the DMA transfer.
        This guarantees that the source buffer will be up to date before starting the transfer.
    */
    SCB_CleanDCache_by_Addr(au8SrcArray, sizeof(au8SrcArray));
    SCB_CleanDCache_by_Addr(au8DestArray0, sizeof(au8DestArray0));
    SCB_CleanDCache_by_Addr(au8DestArray1, sizeof(au8DestArray1));
    SCB_CleanDCache_by_Addr(au32Commands, sizeof(au32Commands));
#endif  // (NVT_DCACHE_ON == 1)
    /*
        The first command must be set directly in
        the channel registers to start the linked command chain.
    */
    lib_err = dma350_memcpy(GDMA_CH_DEV_S[0],
                            au8SrcArray,
                            au8DestArray0,
                            GDMA_TEST_LENGTH,
                            DMA350_LIB_EXEC_BLOCKING);

    if (DMA350_LIB_ERR_NONE == lib_err)
    {
#if (NVT_DCACHE_ON == 1)
        /*
           Invalidate the CPU Data cache after the DMA transfer.
           As the destination buffer may be used by the CPU, this guarantees up-to-date data when CPU access
        */
        SCB_InvalidateDCache_by_Addr(au8DestArray0, sizeof(au8DestArray0));
        SCB_InvalidateDCache_by_Addr(au8DestArray1, sizeof(au8DestArray1));
#endif  // (NVT_DCACHE_ON == 1)
        printf("GDMA transfer Done!\n");

        /* Compare data */
        for (i = 0; i < GDMA_TEST_LENGTH; i++)
        {
            if ((au8SrcArray[i] != au8DestArray0[i]) || (au8SrcArray[i] != au8DestArray1[i]))
            {
                printf(" - Compare data fail\n");
                break;
            }
        }
    }
    else
    {
        printf("GDMA transfer Fail!\n");
    }

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
