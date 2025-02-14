/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use the LPTMR LPTM0_EXT pin to demonstrate timer free counting mode function.
 *          And displays the measured input frequency to UART console.
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
    static int cnt = 0;
    static uint32_t t0, t1;

    LPTMR_ClearCaptureIntFlag(LPTMR0);

    if (cnt == 0)
    {
        t0 = LPTMR_GetCaptureData(LPTMR0);
        cnt++;
    }
    else if (cnt == 1)
    {
        t1 = LPTMR_GetCaptureData(LPTMR0);
        cnt++;

        if (t0 >= t1)
        {
            /* over run, drop this data and do nothing */
        }
        else
        {
            /* LPTMR0 clock source = HIRC */
            printf("Input frequency is %luHz\n", (__HIRC) / (t1 - t0));
        }
    }
    else
    {
        cnt = 0;
    }

    __DSB();
    __ISB();
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
    /* Select LPTMR0 clock source */
    CLK_SetModuleClock(LPTMR0_MODULE, CLK_LPTMRSEL_LPTMR0SEL_HIRC, 0);

    /* Enable LPTMR0 clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);
    /* Set LPTMR0 capture pin */
    CLK_EnableModuleClock(GPIOA_MODULE);
    SET_LPTM0_EXT_PA11();

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
    printf("\nThis sample code demonstrate LPTMR0 free counting mode.\n");
    printf("Please connect input source with LPTMR0 capture pin PA.11, press any key to continue\n");
    getchar();

    /* Give a dummy target frequency here. Will over write capture resolution with macro */
    LPTMR_Open(LPTMR0, LPTMR_PERIODIC_MODE, 1000000);

    /* Update prescale to set proper resolution. */
    LPTMR_SET_PRESCALE_VALUE(LPTMR0, 0);

    /* Set compare value as large as possible, so don't need to worry about counter overrun too frequently. */
    LPTMR_SET_CMP_VALUE(LPTMR0, 0xFFFFFF);

    /* Configure LPTMR0 free counting mode, capture value on rising edge */
    LPTMR_EnableCapture(LPTMR0, LPTMR_CAPTURE_FREE_COUNTING_MODE, LPTMR_CAPTURE_EVENT_RISING);

    /* Start LPTMR0 */
    LPTMR_Start(LPTMR0);

    /* Enable timer interrupt */
    LPTMR_EnableCaptureInt(LPTMR0);
    NVIC_EnableIRQ(LPTMR0_IRQn);

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
