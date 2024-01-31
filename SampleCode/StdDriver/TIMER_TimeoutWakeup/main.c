/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use timer to wake up system from Power-down mode periodically.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
/*
 * This sample uses internal RC as APLL0 clock source and UART to print messages.
 * Users may need to do extra system configuration according to their system design.
 *
 * Debug UART
 *   system_M55M1.c has three weak functions as below to configure debug UART port.
 *     SetDebugUartMFP, SetDebugUartCLK and InitDebugUart
 *   Users can re-implement these functions according to system design.
 */
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

static volatile uint32_t g_u32PDWK;

NVT_ITCM void TIMER0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    CLK_WaitModuleClockReady(TMR0_MODULE);//TESTCHIP_ONLY
    CLK_WaitModuleClockReady(DEBUG_PORT_MODULE);//TESTCHIP_ONLY

    // Clear wake up flag
    TIMER_ClearWakeupFlag(TIMER0);

    // Clear interrupt flag
    TIMER_ClearIntFlag(TIMER0);
    __DSB();
    __ISB();

    while (TIMER_GetWakeupFlag(TIMER0) || TIMER_GetIntFlag(TIMER0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for TIMER0 IntFlag time-out!\n");
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Power Wake-up IRQ Handler                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void PMC_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    g_u32PDWK = PMC_GetPMCWKSrc();
    CLK_WaitModuleClockReady(DEBUG_PORT_MODULE);//TESTCHIP_ONLY

    /* check power down wakeup flag */
    if (g_u32PDWK & PMC_INTSTS_PDWKIF_Msk)
    {
        PMC->INTSTS |= PMC_INTSTS_CLRWK_Msk;

        __DSB();
        __ISB();

        while (PMC_GetPMCWKSrc() & PMC_INTSTS_PDWKIF_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for PMC IntFlag time-out!\n");
            }
        }
    }
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Release GPIO hold status */
    PMC_RELEASE_GPIO();

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

    /* Set PF multi-function pins for X32_OUT(PF.4) and X32_IN(PF.5) */
    SET_X32_OUT_PF4();
    SET_X32_IN_PF5();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 180MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable LIRC*/
    CLK_EnableXtalRC(CLK_SRCCTL_LIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Select TIMER clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_LIRC, 0);
    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Set PC multi-function pin for CLKO(PC.13) */
    SET_CLKO_PC13();

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    int i = 0;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("System core clock = %d\n", SystemCoreClock);
    printf("Timer power down/wake up sample code\n");

    while (!UART_IS_TX_EMPTY(DEBUG_PORT));

    /* Output selected clock to CKO*/
    CLK_EnableCKO(CLK_CLKOSEL_CLKOSEL_SYSCLK, 3, CLK_CLKOCTL_DIV1EN_DIV_FREQSEL);

    /* Initial Timer0 to periodic mode with 1Hz, since system is fast (12MHz)
       and timer is slow (32KHz), and following function calls all modified timer's
       CTL register, so add extra delay between each function call and make sure the
       setting take effect */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);
    CLK_SysTickDelay(50);
    /* Enable timer wake up system */
    TIMER_EnableWakeup(TIMER0);
    CLK_SysTickDelay(50);
    /* Enable Timer0 interrupt */
    TIMER_EnableInt(TIMER0);
    CLK_SysTickDelay(50);
    NVIC_EnableIRQ(TIMER0_IRQn);
    NVIC_EnableIRQ(PMC_IRQn);
    /* Start Timer0 counting */
    TIMER_Start(TIMER0);
    CLK_SysTickDelay(50);
    /* Unlock protected registers */
    SYS_UnlockReg();
    PMC_ENABLE_INT();

    while (1)
    {
        printf("Enter Power-down !\n");
        g_u32PDWK = 0;

        while (!UART_IS_TX_EMPTY(DEBUG_PORT));

        PMC_PowerDown();

        while (!g_u32PDWK);

        printf("Wake %d\n", i++);
        CLK_SysTickDelay(1000000);
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
