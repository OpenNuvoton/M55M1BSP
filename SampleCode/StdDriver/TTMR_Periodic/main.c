/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use the TTMR periodic mode to generate timer interrupt every 1 second.
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

NVT_ITCM void TTMR0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    static uint32_t sec = 1;

    /* clear TTMR interrupt flag */
    TTMR_ClearIntFlag(TTMR0);
    printf("%d sec\n", sec++);
    __DSB();
    __ISB();

    while (TTMR_GetIntFlag(TTMR0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for TTMR0 IntFlag time-out!\n");
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
    /* Enable PLL0 220MHZ clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

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
    /* Select TTMR0 clock source */
    CLK_SetModuleClock(TTMR0_MODULE, CLK_TTMRSEL_TTMR0SEL_HIRC, 0);

    /* Enable TTMR0 clock */
    CLK_EnableModuleClock(TTMR0_MODULE);

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("System core clock = %d\n", SystemCoreClock);
    printf("\nThis sample code use TTMR to generate interrupt every 1 second \n");

    /* Set TTMR frequency to 1HZ */
    TTMR_Open(TTMR0, TTMR_PERIODIC_MODE, 1);

    /* Enable TTMR interrupt */
    TTMR_EnableInt(TTMR0);
    NVIC_EnableIRQ(TTMR0_IRQn);

    /* Start TTMR0 */
    TTMR_Start(TTMR0);

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
