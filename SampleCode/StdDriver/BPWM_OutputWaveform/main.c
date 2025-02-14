/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to use BPWM counter output waveform.
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
    /* Enable PLL0 220MHZ clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();
    /* Select BPWM module clock source */
    CLK_SetModuleClock(BPWM0_MODULE, CLK_BPWMSEL_BPWM0SEL_PCLK0, 0);
    CLK_SetModuleClock(BPWM1_MODULE, CLK_BPWMSEL_BPWM1SEL_PCLK2, 0);
    /* Enable BPWM module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);
    CLK_EnableModuleClock(BPWM1_MODULE);
    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Set multi-function pin for BPWM */
    SET_BPWM0_CH0_PE2();
    SET_BPWM0_CH1_PE3();
    SET_BPWM0_CH2_PE4();
    SET_BPWM0_CH3_PE5();
    SET_BPWM0_CH4_PE6();
    SET_BPWM0_CH5_PE7();
    SET_BPWM1_CH0_PB11();
    SET_BPWM1_CH1_PB10();
    SET_BPWM1_CH2_PB9();
    SET_BPWM1_CH3_PB8();
    SET_BPWM1_CH4_PB7();
    SET_BPWM1_CH5_PB6();

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
    printf("|                  BPWM OutputWaveform Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with BPWM0 and BPWM1 channel 0~5.\n");
    printf("  I/O configuration:\n");
    printf("  BPWM0 channel 0: 180000 Hz, duty 90%%.\n");
    printf("  BPWM0 channel 1: 180000 Hz, duty 80%%.\n");
    printf("  BPWM0 channel 2: 180000 Hz, duty 75%%.\n");
    printf("  BPWM0 channel 3: 180000 Hz, duty 70%%.\n");
    printf("  BPWM0 channel 4: 180000 Hz, duty 60%%.\n");
    printf("  BPWM0 channel 5: 180000 Hz, duty 50%%.\n");
    printf("  BPWM1 channel 0:  60000 Hz, duty 50%%.\n");
    printf("  BPWM1 channel 1:  60000 Hz, duty 40%%.\n");
    printf("  BPWM1 channel 2:  60000 Hz, duty 30%%.\n");
    printf("  BPWM1 channel 3:  60000 Hz, duty 25%%.\n");
    printf("  BPWM1 channel 4:  60000 Hz, duty 20%%.\n");
    printf("  BPWM1 channel 5:  60000 Hz, duty 10%%.\n");
    printf("  waveform output pin: BPWM0_CH0(PE.2), BPWM0_CH1(PE.3), BPWM0_CH2(PE.4), BPWM0_CH3(PE.5), BPWM0_CH4(PE.6), BPWM0_CH5(PE.7)\n");
    printf("                       BPWM1_CH0(PB.11), BPWM1_CH1(PB.10), BPWM1_CH2(PB.9), BPWM1_CH3(PB.8), BPWM1_CH4(PB.7), BPWM1_CH5(PB.6)\n");

    /* BPWM0 and BPWM1 channel 0~5 frequency and duty configuration are as follows */
    /* Because of BPWM0 channel 0~5 share one period, so the period value of all channels need set the same. */
    BPWM_ConfigOutputChannel(BPWM0, 0, 180000, 90);
    BPWM_ConfigOutputChannel(BPWM0, 1, 180000, 80);
    BPWM_ConfigOutputChannel(BPWM0, 2, 180000, 75);
    BPWM_ConfigOutputChannel(BPWM0, 3, 180000, 70);
    BPWM_ConfigOutputChannel(BPWM0, 4, 180000, 60);
    BPWM_ConfigOutputChannel(BPWM0, 5, 180000, 50);
    /* Because of BPWM1 channel 0~5 share one period, so the period value of all channels need set the same. */
    BPWM_ConfigOutputChannel(BPWM1, 0, 60000, 50);
    BPWM_ConfigOutputChannel(BPWM1, 1, 60000, 40);
    BPWM_ConfigOutputChannel(BPWM1, 2, 60000, 30);
    BPWM_ConfigOutputChannel(BPWM1, 3, 60000, 25);
    BPWM_ConfigOutputChannel(BPWM1, 4, 60000, 20);
    BPWM_ConfigOutputChannel(BPWM1, 5, 60000, 10);

    /* Enable output of BPWM0 and BPWM1 channel 0~5 */
    BPWM_EnableOutput(BPWM0, 0x3F);
    BPWM_EnableOutput(BPWM1, 0x3F);

    /* Start BPWM0 counter */
    BPWM_Start(BPWM0, 0x3F);
    /* Start BPWM1 counter */
    BPWM_Start(BPWM1, 0x3F);

    /* Wait for user press any key to stop */
    printf("Press any key to stop.\n");
    getchar();

    /* Stop BPWM0 counter */
    BPWM_ForceStop(BPWM0, 0x3F);
    /* Stop BPWM1 counter */
    BPWM_ForceStop(BPWM1, 0x3F);

    printf("Done.");

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
