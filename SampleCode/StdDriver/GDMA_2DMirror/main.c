/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use GDMA channel 0 to do 2D transfer (Mirror along X axis. Top to bottom).
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define GDMA_XSIZE 8
#define GDMA_YSIZE 8

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
__attribute__((aligned)) static uint8_t au8SrcArray[GDMA_XSIZE * GDMA_YSIZE];
__attribute__((aligned)) static uint8_t au8DestArray[GDMA_XSIZE * GDMA_YSIZE];

/* DMA350 driver structures */
static const struct dma350_dev_cfg_t GDMA_DEV_CFG_S =
{
    .dma_sec_cfg = (DMASECCFG_TypeDef *)(GDMA_S + 0x0UL),
    .dma_sec_ctrl = (DMASECCTRL_TypeDef *)(GDMA_S + 0x100UL),
    .dma_nsec_ctrl = (DMANSECCTRL_TypeDef *)(GDMA_S + 0x200UL),
    .dma_info = (DMAINFO_TypeDef *)(GDMA_S + 0xF00UL)
};

static struct dma350_dev_data_t GDMA_DEV_DATA_S =
{
    .state = 0
};

struct dma350_dev_t GDMA_DEV_S =
{
    &(GDMA_DEV_CFG_S),
    &(GDMA_DEV_DATA_S)
};

struct dma350_ch_dev_t GDMA_CH0_DEV_S =
{
    .cfg = {
        .ch_base = (DMACH_TypeDef *)(GDMA_S + 0x1000UL),
        .channel = 0
    },
    .data = {0}
};

struct dma350_ch_dev_t GDMA_CH1_DEV_S =
{
    .cfg = {
        .ch_base = (DMACH_TypeDef *)(GDMA_S + 0x1100UL),
        .channel = 1
    },
    .data = {0}
};

struct dma350_ch_dev_t *const GDMA_CH_DEV_S[] =
{
    &GDMA_CH0_DEV_S,
    &GDMA_CH1_DEV_S,
};

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
    /* Enable GDMA0 clock source */
    CLK_EnableModuleClock(GDMA0_MODULE);
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
    printf("+--------------------------------------+ \n");
    printf("|    GDMA Mirroring 2D Blocks Sample   | \n");
    printf("+--------------------------------------+ \n");
    /* Reset GDMA module */
    SYS_UnlockReg();
    SYS_ResetModule(SYS_GDMA0RST);
    SYS_LockReg();

    for (i = 0; i < GDMA_XSIZE * GDMA_YSIZE; i++)
    {
        au8SrcArray[i] = (uint8_t) i;
        au8DestArray[i] = 0;
    }

    /*------------------------------------------------------------------------------------------------------


                         au8SrcArray                           au8DestArray
                         -----------------------------   -->   -----------------------------
                       /| [0]  | [1]  ...  [6] |  [7] |       | [0]  | [1]  ...  [6] |  [7] |\
                        |      |      |        |      |       |      |      |        |      |
             GDMA_YSIZE |             ...             |       |             ...             | GDMA_YSIZE
                        |      |      |        |      |       |      |      |        |      |
                       \| [56] | [57] ... [62] | [63] |       | [56] | [57] ... [62] | [63] |/
                         -----------------------------         -----------------------------
                         \                           /         \                           /
                                  GDMA_XSIZE                            GDMA_XSIZE

      GDMA transfer configuration:

        Channel = 0
        Trasnfer type  = 2D
        Transfer count = GDMA_XSIZE * GDMA_YSIZE
        Transfer width = 8 bits(one byte)
        Source address = au8SrcArray
        Source address increment size = 8 bits(one byte)
        Destination address = au8DestArray
        Destination address increment size = 8 bits(one byte)

        Total transfer length = GDMA_XSIZ * GDMA_YSIZE * 8 bits
    ------------------------------------------------------------------------------------------------------*/
    /* Initializes GDMA */
    dma350_init(&GDMA_DEV_S);
    /* Mirror along X axis. Top to bottom */
    lib_err = dma350_2d_copy(GDMA_CH_DEV_S[0],
                             au8SrcArray, au8DestArray,
                             GDMA_XSIZE, GDMA_YSIZE,
                             DMA350_CH_TRANSIZE_8BITS,
                             DMA350_LIB_TRANSFORM_MIRROR_VER,
                             DMA350_LIB_EXEC_BLOCKING);

    /* Check transfer result */
    if (lib_err == DMA350_LIB_ERR_NONE)
    {
        printf("2D Mirror along X axis. Top to bottom\n");

        for (i = 0; i < GDMA_YSIZE; i++)
        {
            printf("\n\t[%02d] ", i * GDMA_XSIZE);

            for (j = 0; j < GDMA_XSIZE; j++)
            {
                printf("%02d ", au8DestArray[i * GDMA_XSIZE + j]);
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
