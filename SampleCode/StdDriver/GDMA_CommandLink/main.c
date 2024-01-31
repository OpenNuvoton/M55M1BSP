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
__attribute__((aligned)) static uint8_t au8SrcArray[256];
__attribute__((aligned)) static uint8_t au8DestArray0[256];
__attribute__((aligned)) static uint8_t au8DestArray1[256];
__attribute__((aligned)) static uint32_t au32Commands[256];

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
