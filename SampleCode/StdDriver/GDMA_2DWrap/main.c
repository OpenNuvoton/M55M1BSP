/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use GDMA channel 0 to do WRAP for 2D.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define GDMA_SRC_XSIZE 4
#define GDMA_SRC_YSIZE 4
#define GDMA_DES_XSIZE 8
#define GDMA_DES_YSIZE 8

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#if (NVT_DCACHE_ON == 1)
    /* Base address and size of cache buffer must be DCACHE_LINE_SIZE byte aligned */
    uint8_t au8SrcArray[DCACHE_ALIGN_LINE_SIZE(GDMA_SRC_XSIZE * GDMA_SRC_YSIZE)] __attribute__((aligned(DCACHE_LINE_SIZE)));
    uint8_t au8DestArray[DCACHE_ALIGN_LINE_SIZE(GDMA_DES_XSIZE * GDMA_DES_YSIZE)] __attribute__((aligned(DCACHE_LINE_SIZE)));
#else
    __attribute__((aligned)) static uint8_t au8SrcArray[GDMA_SRC_XSIZE * GDMA_SRC_YSIZE];
    __attribute__((aligned)) static uint8_t au8DestArray[GDMA_DES_XSIZE * GDMA_DES_YSIZE];
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
    uint32_t i, j;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------+ \n");
    printf("|    GDMA WRAP for 2D Sample   | \n");
    printf("+------------------------------+ \n");
    /* Reset GDMA module */
    SYS_UnlockReg();
    SYS_ResetModule(SYS_GDMA0RST);
    SYS_LockReg();

    for (i = 0; i < GDMA_SRC_XSIZE * GDMA_SRC_YSIZE; i++)
    {
        au8SrcArray[i] = (uint8_t) i;
    }

    memset(au8DestArray, 0, GDMA_DES_XSIZE * GDMA_DES_YSIZE);
#if (NVT_DCACHE_ON == 1)
    /*
        Clean the CPU Data cache before starting the DMA transfer.
        This guarantees that the source buffer will be up to date before starting the transfer.
    */
    SCB_CleanDCache_by_Addr(au8SrcArray, sizeof(au8SrcArray));
    SCB_CleanDCache_by_Addr(au8DestArray, sizeof(au8DestArray));
#endif  // (NVT_DCACHE_ON == 1)
    /*------------------------------------------------------------------------------------------------------


                         au8SrcArray                           au8DestArray
                         ---------------------------   -->   -----------------------------
                       /| [0]  | [1]  |  [2] |  [3] |       | [0]  | [1]  ...  [6] |  [7] |\
                        |      |      |      |      |       |      |      |        |      |
         GDMA_SRC_YSIZE |             ...           |       |             ...             | GDMA_DES_YSIZE
                        |      |      |      |      |       |      |      |        |      |
                       \| [12] | [13] | [14] | [15] |       | [56] | [57] ... [62] | [63] |/
                         ---------------------------         -----------------------------
                         \                         /         \                           /
                               GDMA_SRC_XSIZE                       GDMA_DES_XSIZE

      GDMA transfer configuration:

        Channel = 0
        Trasnfer type  = 2D
        Transfer count = GDMA_DES_XSIZE * GDMA_DES_YSIZE
        Transfer width = 8 bits(one byte)
        Source address = au8SrcArray
        Source address increment size = GDMA_SRC_XSIZE bytes
        Destination address = au8DestArray
        Destination address increment size = GDMA_DES_XSIZE bytes

        Total transfer length = GDMA_DES_XSIZE * GDMA_DES_YSIZE * 8 bits
    ------------------------------------------------------------------------------------------------------*/
    /* Initializes GDMA */
    dma350_init(&GDMA_DEV_S);
    /* 2D Wrap */
    lib_err = dma350_draw_from_canvas(GDMA_CH_DEV_S[0],
                                      au8SrcArray, au8DestArray,
                                      GDMA_SRC_XSIZE, GDMA_SRC_YSIZE,
                                      GDMA_SRC_XSIZE,
                                      GDMA_DES_XSIZE, GDMA_DES_YSIZE,
                                      GDMA_DES_XSIZE,
                                      DMA350_CH_TRANSIZE_8BITS,
                                      DMA350_LIB_TRANSFORM_NONE,
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
        printf("2D Wrap\n");

        for (i = 0; i < GDMA_DES_YSIZE; i++)
        {
            printf("\n\t[%02d] ", i * GDMA_DES_XSIZE);

            for (j = 0; j < GDMA_DES_XSIZE; j++)
            {
                printf("%02d ", au8DestArray[i * GDMA_DES_XSIZE + j]);
            }
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
