/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate output different duty waveform in LPTMR0~LPTMR1 PWM.
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
    /* Select LPTMR clock source */
    CLK_SetModuleClock(LPTMR0_MODULE, CLK_LPTMRSEL_LPTMR0SEL_HIRC, 0);
    CLK_SetModuleClock(LPTMR1_MODULE, CLK_LPTMRSEL_LPTMR1SEL_HIRC, 0);
    /* Enable LPTMR module clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);
    CLK_EnableModuleClock(LPTMR1_MODULE);

    /* Set LPTMR0 PWM CH0(TM0) pin */
    CLK_EnableModuleClock(GPIOB_MODULE);
    SET_LPTM0_PB5();
    SET_LPTM1_PB4();

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
    printf("+---------------------------------------------------+\n");
    printf("|  Low Power TMR0~TMR1 PWM Output Duty Sample Code  |\n");
    printf("+---------------------------------------------------+\n\n");

    printf("# Low Power TMR0 PWM_CH0 output frequency is 18000 Hz and duty is 50%%.\n");
    printf("# Low Power TMR1 PWM_CH0 output frequency is 10000 Hz and duty is 10%%.\n");

    printf("# I/O configuration:\n");
    printf("    - Low Power TMR0 PWM_CH0 on PB.5\n");
    printf("    - Low Power TMR1 PWM_CH0 on PB.4\n");

    /* Change Low Power Timer to PWM counter mode */
    LPTPWM_ENABLE_PWM_MODE(LPTMR0);
    LPTPWM_ENABLE_PWM_MODE(LPTMR1);

    /* Enable output of PWM_CH0 */
    LPTPWM_ENABLE_OUTPUT(LPTMR0, TPWM_CH0);
    LPTPWM_ENABLE_OUTPUT(LPTMR1, TPWM_CH0);

    /* Set Low Power TMR0 PWM output frequency is 18000 Hz, duty 50% in up count type */
    if (LPTPWM_ConfigOutputFreqAndDuty(LPTMR0, 18000, 50) != 18000)
    {
        printf("Set the frequency different from the user\n");
    }

    /* Set Low Power TMR1 PWM output frequency is 10000 Hz, duty 10% in up count type */
    if (LPTPWM_ConfigOutputFreqAndDuty(LPTMR1, 10000, 10) != 10000)
    {
        printf("Set the frequency different from the user\n");
    }

    /* Start Low Power TMR PWM counter */
    TPWM_START_COUNTER(LPTMR0);
    TPWM_START_COUNTER(LPTMR1);

    printf("*** Check LPTMR0~LPTMR1 PWM_CH0 output waveform by oscilloscope ***\n");

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
