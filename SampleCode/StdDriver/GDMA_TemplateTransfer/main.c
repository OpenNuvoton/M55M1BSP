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
__attribute__((aligned)) static uint8_t au8SrcArray[0x100];
__attribute__((aligned)) static uint8_t au8DestArray[0x100];

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
