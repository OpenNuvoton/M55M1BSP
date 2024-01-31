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
__attribute__((aligned)) static uint8_t au8SrcArray[GDMA_TEST_LENGTH];
__attribute__((aligned)) static uint8_t au8DestArray[GDMA_TEST_LENGTH];
static uint32_t volatile g_u32IsTestOver = 0;

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
