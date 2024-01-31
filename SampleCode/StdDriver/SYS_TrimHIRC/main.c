/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to use LXT to trim HIRC/HIRC48M.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void TrimHIRC(void);
void TrimHIRC48M(void);
void IRC_IRQHandler(void);
void SYS_Init(void);

/*--------------------------------------------------------------------------------------------------------*/
/*  IRCTrim IRQ Handler                                                                                   */
/*--------------------------------------------------------------------------------------------------------*/
NVT_ITCM void IRC_IRQHandler(void)
{
    if (SYS->TISTS12M & SYS_TISTS12M_TFAILIF_Msk)  /* Get Trim Failure Interrupt */
    {
        /* Display HIRC trim status */
        printf("HIRC Trim Failure Interrupt\n");

        /* Clear Trim Failure Interrupt */
        SYS->TISTS12M = SYS_TISTS12M_TFAILIF_Msk;
    }

    if (SYS->TISTS12M & SYS_TISTS12M_CLKERRIF_Msk)  /* Get LXT Clock Error Interrupt */
    {
        /* Display HIRC trim status */
        printf("LXT Clock Error Interrupt\n");

        /* Clear LXT Clock Error Interrupt */
        SYS->TISTS12M = SYS_TISTS12M_CLKERRIF_Msk;
    }

    if (SYS->TISTS48M & SYS_TISTS48M_TFAILIF_Msk)  /* Get Trim Failure Interrupt */
    {
        /* Display HIRC48M trim status */
        printf("HIRC48M Trim Failure Interrupt\n");

        /* Clear Trim Failure Interrupt */
        SYS->TISTS48M = SYS_TISTS48M_TFAILIF_Msk;
    }

    if (SYS->TISTS48M & SYS_TISTS48M_CLKERRIF_Msk)  /* Get LXT Clock Error Interrupt */
    {
        /* Display HIRC trim status */
        printf("LXT Clock Error Interrupt\n");

        /* Clear LXT Clock Error Interrupt */
        SYS->TISTS48M = SYS_TISTS48M_CLKERRIF_Msk;
    }

    /* CPU read interrupt flag register to wait write(clear) instruction completement */
    inp32(&SYS->TISTS12M);
    inp32(&SYS->TISTS48M);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

    /* Set PF multi-function pins for X32_OUT(PF.4) and X32_IN(PF.5) */
    SET_X32_OUT_PF4();
    SET_X32_IN_PF5();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable Internal RC 48MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRC48MEN_Msk);

    /* Waiting for Internal RC 48MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRC48MSTB_Msk);

    /* Enable external high speed clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for external high speed clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable external low speed clock */
    CLK_EnableXtalRC(CLK_SRCCTL_LXTEN_Msk);

    /* Waiting for external low speed clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Enable PLL0 180MHz clock and set all bus clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set PC multi-function pin for CLKO(PC.13) */
    SET_CLKO_PC13();

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    /* Enable Interrupt */
    NVIC_EnableIRQ(IRC_IRQn);

    printf("Press any key to start HIRC trim\n");

    getchar();

    /* Trim HIRC to 12MHz */
    TrimHIRC();

    printf("Press any key to start HIRC48M trim\n");

    getchar();

    /* Trim HIRC48M to 48MHz */
    TrimHIRC48M();

    /* Disable HIRC auto Trim */
    SYS->TCTL12M = SYS->TCTL12M & ~SYS_TCTL12M_FREQSEL_Msk;

    /* Disable HIRC48M auto Trim */
    SYS->TCTL48M = SYS->TCTL48M & ~SYS_TCTL48M_FREQSEL_Msk;

    printf("Test done, disable IRC trim.\n");

    while (1);
}

void TrimHIRC(void)
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable IRC Trim, set HIRC clock and enable interrupt */
    SYS->TIEN12M |= (SYS_TIEN12M_CLKEIEN_Msk | SYS_TIEN12M_TFAILIEN_Msk);
    SYS->TCTL12M = (SYS->TCTL12M & (~SYS_TCTL12M_FREQSEL_Msk)) | 0x1;

    CLK_SysTickDelay(2000); /* Waiting for HIRC Frequency Lock */

    /* Get HIRC Frequency Lock */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while ((SYS->TISTS12M & SYS_TISTS12M_FREQLOCK_Msk) == 0)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("HIRC Trim failed!\n");
            return;
        }
    }

    printf("HIRC Frequency Lock!\n");

    /* Clear Trim Lock */
    SYS->TISTS12M = SYS_TISTS12M_FREQLOCK_Msk;

    /* Enable CLKO and output frequency = HIRC */
    CLK_EnableCKO(CLK_CLKOSEL_CLKOSEL_HIRC, 1, CLK_CLKOCTL_DIV1EN_DIV_1);

    /* Lock protected registers */
    SYS_LockReg();
}

void TrimHIRC48M(void)
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable IRC Trim, set HIRC48M clock and enable interrupt */
    SYS->TIEN48M |= (SYS_TIEN48M_CLKEIEN_Msk | SYS_TIEN48M_TFAILIEN_Msk);
    SYS->TCTL48M = (SYS->TCTL48M & (~SYS_TCTL48M_FREQSEL_Msk)) | 0x1;

    CLK_SysTickDelay(2000); /* Waiting for HIRC Frequency Lock */

    /* Get HIRC Frequency Lock */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while ((SYS->TISTS48M & SYS_TISTS48M_FREQLOCK_Msk) == 0)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("HIRC48M Trim failed!\n");
            return;
        }
    }

    printf("HIRC48M Frequency Lock!\n");

    /* Clear Trim Lock */
    SYS->TISTS48M = SYS_TISTS48M_FREQLOCK_Msk;

    /* Enable CLKO and output frequency = HIRC48M */
    CLK_EnableCKO(CLK_CLKOSEL_CLKOSEL_HIRC48M, 1, CLK_CLKOCTL_DIV1EN_DIV_1);

    /* Lock protected registers */
    SYS_LockReg();
}
