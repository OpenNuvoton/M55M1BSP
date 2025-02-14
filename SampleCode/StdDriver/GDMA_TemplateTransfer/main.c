/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use GDMA channel 0 to do template transfer.
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
    uint8_t au8SrcArray[DCACHE_ALIGN_LINE_SIZE(0x100)] __attribute__((aligned(DCACHE_LINE_SIZE)));
    uint8_t au8DestArray[DCACHE_ALIGN_LINE_SIZE(0x100)] __attribute__((aligned(DCACHE_LINE_SIZE)));
#else
    __attribute__((aligned)) static uint8_t au8SrcArray[0x100];
    __attribute__((aligned)) static uint8_t au8DestArray[0x100];
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
    uint32_t i;
    uint32_t u32SrcTmpl, u32DesTmpl, u32SrcTmplSize, u32DesTmplSize;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-------------------------------------+ \n");
    printf("|    GDMA Template Transfer Sample    | \n");
    printf("+-------------------------------------+ \n");
    /* Reset GDMA module */
    SYS_UnlockReg();
    SYS_ResetModule(SYS_GDMA0RST);
    SYS_LockReg();

    for (i = 0; i < 0x100; i++)
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
    u32SrcTmplSize + 1  |    ...     |         |    ...     | u32DesTmplSize + 1
                        |    [6]     |         |    [2]     |
                       \|    [7]     |         |    [3]     |/
                         ------------           ------------
                         \          /           \          /
                           one byte               one byte

      GDMA transfer configuration:

        Channel = 0
        Operation mode = basic mode

        Transfer count = GDMA_TEST_LENGTH
        Transfer width = 8 bits(one byte)
        Source address = au8SrcArray
        Source address increment size = 8 Bytes
        Source address Template = (BIT0 | BIT2 | BIT3 | BIT6)
        Destination address = au8DestArray
        Destination address increment size = 4 Bytes
        Destination address Template = (BIT0 | BIT1 | BIT2 | BIT3)

        Total transfer length = GDMA_TEST_LENGTH * 8 bits
    ------------------------------------------------------------------------------------------------------*/
    /* Initializes GDMA */
    dma350_init(&GDMA_DEV_S);
    /* Tempalte Configuration */
    u32SrcTmpl = (BIT0 | BIT2 | BIT3 | BIT6);
    u32DesTmpl = (BIT0 | BIT1 | BIT2 | BIT3);
    u32SrcTmplSize = 7;
    u32DesTmplSize = 3;
    dma350_ch_set_tmplt_src_size(GDMA_CH_DEV_S[0], u32SrcTmplSize);
    dma350_ch_set_tmplt_des_size(GDMA_CH_DEV_S[0], u32DesTmplSize);
    dma350_ch_set_tmplt_src(GDMA_CH_DEV_S[0], u32SrcTmpl);
    dma350_ch_set_tmplt_des(GDMA_CH_DEV_S[0], u32DesTmpl);
    lib_err = dma350_memcpy(GDMA_CH_DEV_S[0],
                            au8SrcArray,
                            au8DestArray,
                            GDMA_TEST_LENGTH,
                            DMA350_LIB_EXEC_BLOCKING);

    /* Check transfer result */
    if (lib_err == DMA350_LIB_ERR_NONE)
    {
#if (NVT_DCACHE_ON == 1)
        /*
           Invalidate the CPU Data cache after the DMA transfer.
           As the destination buffer may be used by the CPU, this guarantees up-to-date data when CPU access
        */
        SCB_InvalidateDCache_by_Addr(au8DestArray, sizeof(au8DestArray));
#endif  // (NVT_DCACHE_ON == 1)
        printf("Template Transfer\n");

        for (i = 0; i < GDMA_TEST_LENGTH; i++)
        {
            if ((i % 8) == 0)
            {
                printf("\n\t[%02d] ", i);
            }

            printf("%02d ", au8DestArray[i]);
        }

        printf("\nGDMA Transfer done\n");
    }
    else
    {
        printf("\nGDMA transfer error !!\n");
    }

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
