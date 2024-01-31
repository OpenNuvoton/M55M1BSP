/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Change duty cycle and period of output waveform by BPWM Double Buffer function.
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
/**
 * @brief       BPWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle BPWM0 interrupt event
 */
NVT_ITCM void BPWM0_IRQHandler(void)
{
    static uint32_t u32Toggle = 0;
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    // Clear channel 0 period interrupt flag
    BPWM_ClearPeriodIntFlag(BPWM0, 0);

    // Update BPWM0 channel 0 period and duty
    if (u32Toggle == 0)
    {
        BPWM_SET_CNR(BPWM0, 0, 399);
        BPWM_SET_CMR(BPWM0, 0, 300);
    }
    else
    {
        BPWM_SET_CNR(BPWM0, 0, 599);
        BPWM_SET_CMR(BPWM0, 0, 100);
    }

    u32Toggle ^= 1;
    __DSB();
    __ISB();

    while (BPWM_GetPeriodIntFlag(BPWM0, 0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for BPWM0 IntFlag time-out!\n");
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

    /* Select BPWM module clock source */
    CLK_SetModuleClock(BPWM0_MODULE, CLK_BPWMSEL_BPWM0SEL_PCLK0, 0);

    /* Enable BPWM module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);
    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPIOE_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Set multi-function pin for BPWM */
    SET_BPWM0_CH0_PE2();

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
    printf("|                    BPWM DoubleBuffer Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use BPWM0 channel 0 to output waveform\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: BPWM0 channel 0(PE.2)\n");
    printf("\nUse double buffer feature.\n");

    /*
        BPWM0 channel 0 waveform of this sample shown below(up counter type):

        |<-        CNR + 1  clk     ->|  CNR + 1 = 599 + 1 CLKs
        |<-  CMR clk ->|                 CMR = 100 CLKs
                                      |<-   CNR + 1  ->|  CNR + 1 = 399 + 1 CLKs
                                      |<-CMR->|           CMR = 300 CLKs
         _____                ___________
        | 100 |_____500______|   300     |_100_|     BPWM waveform
    */
    /*
      Configure BPWM0 channel 0 init period and duty.(up counter type)
      Period is PCLK / (prescaler * clock divider * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 100 MHz / (1 * 1 * (624 + 1)) =  160000 Hz
      Duty ratio = (312) / (624 + 1) = 50%
    */
    /* BPWM0 channel 0 frequency is 160000Hz, duty 50% */
    printf("\nSet 160000Hz frequency.real is %d\n", BPWM_ConfigOutputChannel(BPWM0, 0, 160000, 50));

    /* Enable output of BPWM0 channel 0 */
    BPWM_EnableOutput(BPWM0, BPWM_CH_0_MASK);

    /* Enable BPWM0 channel 0 period interrupt, use channel 0 to measure time. */
    BPWM_EnablePeriodInt(BPWM0, 0, 0);
    NVIC_EnableIRQ(BPWM0_IRQn);

    /* Start */
    BPWM_Start(BPWM0, BPWM_CH_0_MASK);

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
