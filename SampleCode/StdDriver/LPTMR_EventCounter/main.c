/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use LPTM0 pin to demonstrates LPTMR event counter function.
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

NVT_ITCM void LPTMR0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    LPTMR_ClearIntFlag(LPTMR0);
    printf("Count 1000 falling events! Test complete !\n");
    __DSB();
    __ISB();

    while (LPTMR_GetIntFlag(LPTMR0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for LPTMR0 IntFlag time-out!\n");
        }
    }
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

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
    /* Select LPTMR0 clock source */
    CLK_SetModuleClock(LPTMR0_MODULE, CLK_LPTMRSEL_LPTMR0SEL_HIRC, 0);

    /* Enable IP clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);

    /* Enable GPIO Port B clock */
    CLK_EnableModuleClock(GPIOB_MODULE);
    /* Set Timer cointer pin */
    SET_LPTM0_PB5();

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    int i;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("System core clock = %d\n", SystemCoreClock);
    printf("\nThis sample code use LPTM0 (PB.5) to count PB.4 input event\n");
    printf("Please connect PB.5 to PB.4, press any key to continue\n");
    getchar();

    PB4 = 1;    /* Set init state to high */
    GPIO_SetMode(PB, BIT4, GPIO_MODE_OUTPUT);

    /* Give a dummy target frequency here. Will over write prescale and compare value with macro */
    LPTMR_Open(LPTMR0, LPTMR_ONESHOT_MODE, 100);

    /* Update prescale and compare value to what we need in event counter mode. */
    LPTMR_SET_PRESCALE_VALUE(LPTMR0, 0);
    LPTMR_SET_CMP_VALUE(LPTMR0, 1000);
    /* Counter increase on falling edge */
    LPTMR_EnableEventCounter(LPTMR0, LPTMR_COUNTER_EVENT_FALLING);
    /* Start Timer 0 */
    LPTMR_Start(LPTMR0);
    /* Enable timer interrupt */
    LPTMR_EnableInt(LPTMR0);
    NVIC_EnableIRQ(LPTMR0_IRQn);

    for (i = 0; i < 1000; i++)
    {
        PB4 = 0;
        CLK_SysTickDelay(1);
        PB4 = 1;
        CLK_SysTickDelay(1);
    }

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
