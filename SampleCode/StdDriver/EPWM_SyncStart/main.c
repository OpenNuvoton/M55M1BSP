/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to use EPWM counter synchronous start function.
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
    /* Select EPWM1 module clock source */
    CLK_SetModuleClock(EPWM0_MODULE, CLK_EPWMSEL_EPWM0SEL_PCLK0, 0);
    CLK_SetModuleClock(EPWM1_MODULE, CLK_EPWMSEL_EPWM1SEL_PCLK2, 0);

    /* Enable EPWM1 module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);
    CLK_EnableModuleClock(EPWM1_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(GPIOC_MODULE);
    /* Set multi-function pin for EPWM */
    SET_EPWM0_CH0_PE7();
    SET_EPWM0_CH1_PE6();
    SET_EPWM0_CH2_PE5();
    SET_EPWM0_CH3_PE4();
    SET_EPWM0_CH4_PE3();
    SET_EPWM0_CH5_PE2();

    SET_EPWM1_CH0_PC5();
    SET_EPWM1_CH1_PC4();
    SET_EPWM1_CH2_PC3();
    SET_EPWM1_CH3_PC2();
    SET_EPWM1_CH4_PC1();
    SET_EPWM1_CH5_PC0();

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

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                      EPWM SyncStart Sample Code                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with EPWM0 and EPWM1 channel 0~5 at the same time.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: EPWM0_CH0(PE.7), EPWM0_CH1(PE.6), EPWM0_CH2(PE.5), EPWM0_CH3(PE.4), EPWM0_CH4(PE.3), EPWM0_CH5(PE.2)\n");
    printf("                         EPWM1_CH0(PC.5), EPWM1_CH1(PC.4), EPWM1_CH2(PC.3), EPWM1_CH3(PC.2), EPWM1_CH4(PC.1), EPWM1_CH5(PC.0)\n");


    /* EPWM0 and EPWM1 channel 0~5 frequency and duty configuration are as follows */
    EPWM_ConfigOutputChannel(EPWM0, 0, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM0, 1, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM0, 2, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM0, 3, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM0, 4, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM0, 5, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM1, 0, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM1, 1, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM1, 2, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM1, 3, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM1, 4, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM1, 5, 1000, 50);

    /* Enable counter synchronous start function for EPWM0 and EPWM1 channel 0~5 */
    EPWM_ENABLE_TIMER_SYNC(EPWM0, 0x3F, EPWM_SSCTL_SSRC_EPWM0);
    EPWM_ENABLE_TIMER_SYNC(EPWM1, 0x3F, EPWM_SSCTL_SSRC_EPWM0);

    /* Enable output of EPWM0 and EPWM1 channel 0~5 */
    EPWM_EnableOutput(EPWM0, 0x3F);
    EPWM_EnableOutput(EPWM1, 0x3F);

    printf("Press any key to start.\n");
    getchar();

    /* Trigger EPWM counter synchronous start by EPWM0 */
    EPWM_TRIGGER_SYNC_START(EPWM0);

    printf("Done.\n");

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
