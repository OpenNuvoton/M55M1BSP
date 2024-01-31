/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show how to use the timer2 capture function to capture timer2 counter value.
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
/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t g_au32TMRINTCount = 0;

NVT_ITCM void TIMER2_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    if (TIMER_GetCaptureIntFlag(TIMER2) == 1)
    {
        /* Clear Timer2 capture trigger interrupt flag */
        TIMER_ClearCaptureIntFlag(TIMER2);

        g_au32TMRINTCount++;
    }

    __DSB();
    __ISB();

    while (TIMER_GetCaptureIntFlag(TIMER2))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for TIMER2 IntFlag time-out!\n");
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
    /* Enable HXT */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Select TIMER clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_HXT, 0);
    CLK_SetModuleClock(TMR2_MODULE, CLK_TMRSEL_TMR2SEL_HIRC, 0);
    CLK_SetModuleClock(TMR3_MODULE, CLK_TMRSEL_TMR3SEL_HXT, 0);

    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPIOG_MODULE);
    CLK_EnableModuleClock(GPIOF_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);

    /* Set multi-function pins for Timer0/Timer3 toggle-output pin and Timer2 event counter pin */
    SET_TM0_PG2();
    SET_TM2_PG4();
    SET_TM3_PF11();

    /* Set multi-function pin for Timer2 external capture pin */
    SET_TM2_EXT_PH2();
    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint32_t u32InitCount;
    uint32_t au32CAPValue[12], u32CAPDiff;
    uint32_t u32TimeOutCnt;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("System core clock = %d\n", SystemCoreClock);
    printf("+------------------------------------------+\n");
    printf("|    Timer2 Capture Counter Sample Code    |\n");
    printf("+------------------------------------------+\n\n");
    printf("# Timer0 Settings:\n");
    printf("    - Clock source is HXT\n");
    printf("    - Time-out frequency is 1000 Hz\n");
    printf("    - Toggle-output mode and frequency is 500 Hz\n");
    printf("# Timer3 Settings:\n");
    printf("    - Clock source is HXT\n");
    printf("    - Time-out frequency is 2 Hz\n");
    printf("    - Toggle-output mode and frequency is 1 Hz\n");
    printf("# Timer2 Settings:\n");
    printf("    - Clock source is HIRC              \n");
    printf("    - Continuous counting mode          \n");
    printf("    - Interrupt enable                  \n");
    printf("    - Compared value is 0xFFFFFF        \n");
    printf("    - Event counter mode enable         \n");
    printf("    - External capture mode enable      \n");
    printf("    - Capture trigger interrupt enable  \n");
    printf("# Connect TM0(PG.2) toggle-output pin to TM2(PG.4) event counter pin.\n");
    printf("# Connect TM3(PF.11) toggle-output pin to TM2_EXT(PH.2) external capture pin.\n\n");

    /* Enable Timer2 NVIC */
    NVIC_EnableIRQ(TIMER2_IRQn);

    /* Open Timer0 in toggle-output mode and toggle-output frequency is 500 Hz*/
    TIMER_Open(TIMER0, TIMER_TOGGLE_MODE, 1000);

    /* Open Timer3 in toggle-output mode and toggle-output frequency is 1 Hz */
    TIMER_Open(TIMER3, TIMER_TOGGLE_MODE, 2);

    /* Enable Timer2 event counter input and external capture function */
    TIMER_Open(TIMER2, TIMER_CONTINUOUS_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER2, 0);
    TIMER_SET_CMP_VALUE(TIMER2, 0xFFFFFF);
    TIMER_EnableEventCounter(TIMER2, TIMER_COUNTER_EVENT_FALLING);
    TIMER_EnableCapture(TIMER2, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_EVENT_FALLING);
    TIMER_EnableInt(TIMER2);
    TIMER_EnableCaptureInt(TIMER2);

    /* case 1. */
    printf("# Period between two falling edge captured event should be 500 counts.\n");

    /* Clear Timer2 interrupt counts to 0 */
    u32InitCount = g_au32TMRINTCount = 0;

    /* Start Timer0, Timer3 and Timer2 counting */
    TIMER_Start(TIMER0);
    TIMER_Start(TIMER3);
    TIMER_Start(TIMER2);

    /* Check Timer2 capture trigger interrupt counts */
    while (g_au32TMRINTCount < 10)
    {
        if (g_au32TMRINTCount != u32InitCount)
        {
            au32CAPValue[u32InitCount] = TIMER_GetCaptureData(TIMER2);

            if (u32InitCount ==  0)
            {
                printf("    [%2d]: %4d. (1st captured value)\n", g_au32TMRINTCount, au32CAPValue[u32InitCount]);

                if (au32CAPValue[u32InitCount] != 0)  // First capture event will reset counter value
                {
                    printf("*** FAIL ***\n");
                    goto lexit;
                }
            }
            else if (u32InitCount ==  1)
            {
                printf("    [%2d]: %4d. (2nd captured value) \n", g_au32TMRINTCount, au32CAPValue[u32InitCount]);

                if (au32CAPValue[u32InitCount] != 500)  // Second event gets two capture event duration counts directly
                {
                    printf("*** FAIL ***\n");
                    goto lexit;
                }
            }
            else
            {
                u32CAPDiff = au32CAPValue[u32InitCount] - au32CAPValue[u32InitCount - 1];
                printf("    [%2d]: %4d. Diff: %d.\n", g_au32TMRINTCount, au32CAPValue[u32InitCount], u32CAPDiff);

                if (u32CAPDiff != 500)
                {
                    printf("*** FAIL ***\n");
                    goto lexit;
                }
            }

            u32InitCount = g_au32TMRINTCount;
        }
    }

    printf("*** PASS ***\n\n");


    /* case 2. */
    TIMER_StopCapture(TIMER2);
    TIMER_Stop(TIMER2);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (TIMER_IS_ACTIVE(TIMER2))
        if (--u32TimeOutCnt == 0) break;

    TIMER_ClearIntFlag(TIMER2);
    TIMER_ClearCaptureIntFlag(TIMER2);
    /* Enable Timer2 event counter input and external capture function */
    TIMER_Open(TIMER2, TIMER_CONTINUOUS_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER2, 0);
    TIMER_SET_CMP_VALUE(TIMER2, 0xFFFFFF);
    TIMER_EnableEventCounter(TIMER2, TIMER_COUNTER_EVENT_FALLING);
    TIMER_EnableCapture(TIMER2, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_EVENT_GET_LOW_PERIOD);
    TIMER_EnableInt(TIMER2);
    TIMER_EnableCaptureInt(TIMER2);
    TIMER_Start(TIMER2);

    printf("# Get first low duration should be 250 counts.\n");
    printf("# And follows duration between two rising edge captured event should be 500 counts.\n");

    /* Clear Timer2 interrupt counts to 0 */
    u32InitCount = g_au32TMRINTCount = 0;

    /* Enable Timer2 event counter input and external capture function */
    TIMER2->CMP = 0xFFFFFF;
    TIMER2->CTL = TIMER_CTL_CNTEN_Msk | TIMER_CTL_INTEN_Msk | TIMER_CTL_EXTCNTEN_Msk | TIMER_CONTINUOUS_MODE;
    TIMER2->EXTCTL = TIMER_EXTCTL_CAPEN_Msk | TIMER_CAPTURE_FREE_COUNTING_MODE | TIMER_CAPTURE_EVENT_GET_LOW_PERIOD | TIMER_EXTCTL_CAPIEN_Msk;

    /* Check Timer2 capture trigger interrupt counts */
    while (g_au32TMRINTCount <= 10)
    {
        if (g_au32TMRINTCount != u32InitCount)
        {
            au32CAPValue[u32InitCount] = TIMER_GetCaptureData(TIMER2);

            if (u32InitCount ==  0)
            {
                printf("    [%2d]: %4d. (1st captured value)\n", g_au32TMRINTCount, au32CAPValue[u32InitCount]);

                if (au32CAPValue[u32InitCount] != 0)  // First capture event will reset counter value
                {
                    printf("*** FAIL ***\n");
                    goto lexit;
                }
            }
            else if (u32InitCount ==  1)
            {
                printf("    [%2d]: %4d. (2nd captured value)\n", g_au32TMRINTCount, au32CAPValue[u32InitCount]);

                if (au32CAPValue[u32InitCount] != 250)  // Get low duration counts directly
                {
                    printf("*** FAIL ***\n");
                    goto lexit;
                }
            }
            else
            {
                u32CAPDiff = au32CAPValue[u32InitCount] - au32CAPValue[u32InitCount - 1];
                printf("    [%2d]: %4d. Diff: %d.\n", g_au32TMRINTCount, au32CAPValue[u32InitCount], u32CAPDiff);

                if (u32CAPDiff != 500)
                {
                    printf("*** FAIL ***\n");
                    goto lexit;
                }
            }

            u32InitCount = g_au32TMRINTCount;
        }
    }

    printf("*** PASS ***\n");

lexit:

    /* Stop Timer0, Timer2 and Timer3 counting */
    TIMER_Stop(TIMER0);
    TIMER_Stop(TIMER2);
    TIMER_Stop(TIMER3);

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
