/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate TIMER PWM accumulator interrupt to stop counting.
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
static volatile uint32_t gu32Period;

NVT_ITCM void TIMER0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    TPWM_ClearAccInt(TIMER0);

    printf("\nCheck if output toggles 11 times then stop toggles.\n");
    __DSB();
    __ISB();

    while (TPWM_GetAccInt(TIMER0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for TIMER0 IntFlag time-out!\n");
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
    /* Select TIMER clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_PCLK1, 0);
    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPIOB_MODULE);
    /* Set Timer0 PWM CH0(TM0) pin */
    SET_TM0_PB5();
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
    printf("+------------------------------------------------------------+\n");
    printf("|    Timer PWM Accumulator Inerrupt Stop Mode Sample Code    |\n");
    printf("+------------------------------------------------------------+\n\n");

    printf("  This sample code demonstrate Timer0 PWM accumulator stop mode.\n");
    printf("  When accumulator interrupt happens, the output of Timer0 PWM stops.\n");
    printf("  Since interrupt accumulator count is set to 10, the output toggles 11 times then stops.\n");
    printf("    - Timer0 PWM_CH0 on PB.5\n");

    printf("\n\nPress any key to start Timer0 PWM.\n");
    getchar();

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER0);

    /* Set PWM mode as independent mode*/
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER0);

    /* Set Timer0 PWM output frequency is 18000 Hz, duty 50% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER0, 18000, 50);

    /* Set PWM down count type */
    TPWM_SET_COUNTER_TYPE(TIMER0, TPWM_DOWN_COUNT);

    /* Enable output of PWM_CH0 */
    TPWM_ENABLE_OUTPUT(TIMER0, TPWM_CH0);

    /* Enable Timer0 PWM accumulator function, interrupt count 10, accumulator source select to zero point */
    TPWM_EnableAcc(TIMER0, 10, TPWM_IFA_ZERO_POINT);

    /* Enable Timer0 PWM accumulator interrupt */
    TPWM_EnableAccInt(TIMER0);

    /* Enable Timer0 PWM accumulator stop mode */
    TPWM_EnableAccStopMode(TIMER0);

    /* Enable Timer0 interrupt */
    NVIC_EnableIRQ(TIMER0_IRQn);

    /* Start Timer PWM counter */
    TPWM_START_COUNTER(TIMER0);

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
