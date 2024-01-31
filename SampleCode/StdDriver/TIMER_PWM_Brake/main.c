/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to use Timer0 PWM brake function.
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

NVT_ITCM void TIMER0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    printf("\nFault brake!\n");
    printf("Press any key to unlock brake state. (TIMER0 PWM output will toggle again)\n");
    getchar();

    // Clear brake interrupt flag
    SYS_UnlockReg();
    TIMER0->PWMBRKCTL = 0;
    TPWM_DisableFaultBrakeInt(TIMER0, TPWM_BRAKE_EDGE);
    TPWM_ClearFaultBrakeIntFlag(TIMER0, TPWM_BRAKE_EDGE);
    __DSB();
    __ISB();

    while (TPWM_GetFaultBrakeIntFlag(TIMER0, TPWM_BRAKE_EDGE))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for TPWM0 IntFlag time-out!\n");
        }
    }

    SYS_LockReg();
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
    /* Set Timer0 PWM output pins and EPWM1 brake pin 0 (TPWM_TM_BRAKE2),
       Timers share the same brake pins with EPWM */
    SET_TM0_PB5();
    SET_EPWM1_BRAKE0_PB7();
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
    printf("\nL->H state change on PB.7 will generate brake interrupt,\n");
    printf("and Timer0 PWM output on PB.5 will stop until brake state cleared.\n");

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER0);

    /* Set PWM mode as independent mode*/
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER0);

    /* Set PWM up count type */
    TPWM_SET_COUNTER_TYPE(TIMER0, TPWM_UP_COUNT);

    /* Enable output of PWM_CH0 */
    TPWM_ENABLE_OUTPUT(TIMER0, TPWM_CH0);

    /* Set Timer0 PWM output frequency is 18000 Hz, duty 50% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER0, 18000, 50);

    /* Start Timer PWM counter */
    TPWM_START_COUNTER(TIMER0);

    /* Enable brake and interrupt, PWM output stays at low after brake event */
    SYS_UnlockReg();
    TPWM_SET_BRAKE_PIN_SOURCE(TIMER0, TPWM_TM_BRAKE2);
    TPWM_EnableBrakePinDebounce(TIMER0, TPWM_TM_BRAKE2, 7, EPWM_NF_CLK_DIV_128);
    TPWM_EnableFaultBrake(TIMER0, TPWM_OUTPUT_LOW, TPWM_OUTPUT_LOW, TPWM_BRAKE_SOURCE_EDGE_BKPIN);
    TPWM_EnableFaultBrakeInt(TIMER0, TPWM_BRAKE_EDGE);
    TPWM_ClearFaultBrakeIntFlag(TIMER0, TPWM_BRAKE_EDGE);
    SYS_LockReg();

    NVIC_EnableIRQ(TIMER0_IRQn);

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
