/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show how to use the LPTMR capture function to capture LPTMR counter value.
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

static volatile uint32_t g_au32LPTMR1INTCount = 0;

NVT_ITCM void LPTMR1_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    if (LPTMR_GetCaptureIntFlag(LPTMR1) == 1)
    {
        /* Clear LPTMR1 capture trigger interrupt flag */
        LPTMR_ClearCaptureIntFlag(LPTMR1);

        g_au32LPTMR1INTCount++;
    }

    __DSB();
    __ISB();

    while (LPTMR_GetCaptureIntFlag(LPTMR1))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for LPTMR1 IntFlag time-out!\n");
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
    /* Enable LPTMR and TIMER peripheral clock */
    CLK_SetModuleClock(LPTMR0_MODULE, CLK_LPTMRSEL_LPTMR0SEL_HIRC, 0);
    CLK_SetModuleClock(LPTMR1_MODULE, CLK_LPTMRSEL_LPTMR1SEL_HIRC, 0);
    CLK_SetModuleClock(TMR3_MODULE, CLK_TMRSEL_TMR3SEL_HIRC, 0);
    CLK_EnableModuleClock(LPTMR0_MODULE);
    CLK_EnableModuleClock(LPTMR1_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);
    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    /* Set PB multi-function pin for LPTMR1 external capture pin */
    SET_LPTM1_EXT_PA10();
    /* Set multi-function pins for LPTMR0 toggle-output pin and LPTMR1 event counter pin */
    SET_LPTM1_PB4();
    SET_LPTM0_PB5();
    /* Set multi-function pins for Timer3 toggle-output pin */
    SET_TM3_PB2();

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint32_t u32InitCount;
    uint32_t au32CAPValue[10], u32CAPDiff;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("System core clock = %d\n", SystemCoreClock);
    printf("+------------------------------------------+\n");
    printf("|    LPTMR1 Capture Counter Sample Code    |\n");
    printf("+------------------------------------------+\n\n");

    printf("# LPTMR0 Settings:\n");
    printf("    - Clock source is HIRC\n");
    printf("    - Time-out frequency is 1000 Hz\n");
    printf("    - Toggle-output mode and frequency is 500 Hz\n");
    printf("# Timer3 Settings:\n");
    printf("    - Clock source is HIRC\n");
    printf("    - Time-out frequency is 2 Hz\n");
    printf("    - Toggle-output mode and frequency is 1 Hz\n");
    printf("# LPTMR1 Settings:\n");
    printf("    - Clock source is HIRC              \n");
    printf("    - Continuous counting mode          \n");
    printf("    - Interrupt enable                  \n");
    printf("    - Compared value is 0xFFFFFF        \n");
    printf("    - Event counter mode enable         \n");
    printf("    - External capture mode enable      \n");
    printf("    - Capture trigger interrupt enable  \n");
    printf("# Connect LPTM0(PB.5) toggle-output pin to LPTM1(PB.4) event counter pin.\n");
    printf("# Connect TM3(PB.2)   toggle-output pin to LPTM1_EXT(PA.10) external capture pin.\n\n");

    /* Enable LPTMR1 NVIC */
    NVIC_EnableIRQ(LPTMR1_IRQn);

    /* Open LPTMR0 in toggle-output mode and toggle-output frequency is 500 Hz*/
    LPTMR_Open(LPTMR0, LPTMR_TOGGLE_MODE, 1000);

    /* Open Timer3 in toggle-output mode and toggle-output frequency is 1 Hz */
    TIMER_Open(TIMER3, TIMER_TOGGLE_MODE, 2);

    /* Enable LPTMR1 event counter input and external capture function */
    LPTMR_Open(LPTMR1, LPTMR_CONTINUOUS_MODE, 1);
    LPTMR_SET_PRESCALE_VALUE(LPTMR1, 0);
    LPTMR_SET_CMP_VALUE(LPTMR1, 0xFFFFFF);
    LPTMR_EnableEventCounter(LPTMR1, LPTMR_COUNTER_EVENT_FALLING);
    LPTMR_EnableCapture(LPTMR1, LPTMR_CAPTURE_FREE_COUNTING_MODE, LPTMR_CAPTURE_EVENT_FALLING);
    LPTMR_EnableInt(LPTMR1);
    LPTMR_EnableCaptureInt(LPTMR1);

    /* case 1. */
    printf("# Period between two FALLING EDGE captured event should be 500 counts.\n");

    /* Clear LPTMR1 interrupt counts to 0 */
    u32InitCount = g_au32LPTMR1INTCount = 0;

    /* Start LPTMR0, Timer3 and LPTMR1 counting */
    LPTMR_Start(LPTMR0);
    TIMER_Start(TIMER3);
    LPTMR_Start(LPTMR1);

    /* Check LPTMR1 capture trigger interrupt counts */
    while (g_au32LPTMR1INTCount <= 10)
    {
        if (g_au32LPTMR1INTCount != u32InitCount)
        {
            au32CAPValue[u32InitCount] = LPTMR_GetCaptureData(LPTMR1);

            if (u32InitCount ==  0)
            {
                printf("    [%2d]: %4d. (1st captured value)\n", g_au32LPTMR1INTCount, au32CAPValue[u32InitCount]);
            }
            else
            {
                u32CAPDiff = au32CAPValue[u32InitCount] - au32CAPValue[u32InitCount - 1];
                printf("    [%2d]: %4d. Diff: %d.\n", g_au32LPTMR1INTCount, au32CAPValue[u32InitCount], u32CAPDiff);

                if (u32CAPDiff != 500)
                {
                    printf("*** FAIL ***\n");

                    while (1);
                }
            }

            u32InitCount = g_au32LPTMR1INTCount;
        }
    }

    printf("*** PASS ***\n\n");

    /* case 2. */
    LPTMR_DisableCapture(LPTMR1);
    LPTMR_Stop(LPTMR1);

    while (LPTMR_IS_ACTIVE(LPTMR1));

    LPTMR_ClearIntFlag(LPTMR1);
    LPTMR_ClearCaptureIntFlag(LPTMR1);

    printf("# Period between two RISING EDGE captured event should be 500 counts.\n");

    /* Clear LPTMR1 interrupt counts to 0 */
    u32InitCount = g_au32LPTMR1INTCount = 0;

    /* Enable LPTMR1 event counter input and external capture function */
    LPTMR_Open(LPTMR1, LPTMR_CONTINUOUS_MODE, 1);
    LPTMR_SET_PRESCALE_VALUE(LPTMR1, 0);
    LPTMR_SET_CMP_VALUE(LPTMR1, 0xFFFFFF);
    LPTMR_EnableEventCounter(LPTMR1, LPTMR_COUNTER_EVENT_FALLING);
    LPTMR_EnableCapture(LPTMR1, LPTMR_CAPTURE_FREE_COUNTING_MODE, LPTMR_CAPTURE_EVENT_RISING);
    LPTMR_EnableInt(LPTMR1);
    LPTMR_EnableCaptureInt(LPTMR1);
    LPTMR_Start(LPTMR1);

    /* Check LPTMR1 capture trigger interrupt counts */
    while ((g_au32LPTMR1INTCount <= 10) && (u32InitCount < 10))
    {
        if (g_au32LPTMR1INTCount != u32InitCount)
        {
            au32CAPValue[u32InitCount] = LPTMR_GetCaptureData(LPTMR1);

            if (u32InitCount ==  0)
            {
                printf("    [%2d]: %4d. (1st captured value)\n", g_au32LPTMR1INTCount, au32CAPValue[u32InitCount]);
            }
            else
            {
                u32CAPDiff = au32CAPValue[u32InitCount] - au32CAPValue[u32InitCount - 1];
                printf("    [%2d]: %4d. Diff: %d.\n", g_au32LPTMR1INTCount, au32CAPValue[u32InitCount], u32CAPDiff);

                if (u32CAPDiff != 500)
                {
                    printf("*** FAIL ***\n");

                    while (1);
                }
            }

            u32InitCount = g_au32LPTMR1INTCount;
        }
    }

    /* Stop LPTMR0, LPTMR1 and Timer3 counting */
    LPTMR_Stop(LPTMR0);
    LPTMR_Stop(LPTMR1);
    TIMER_Stop(TIMER3);

    printf("*** PASS ***\n");

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
