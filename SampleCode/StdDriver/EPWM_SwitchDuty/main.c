/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Change duty cycle of output waveform by configured period.
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
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t CalNewDutyCMR(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32DutyCycle, uint32_t u32CycleResolution);

/**
 * @brief       Calculate the comparator value of new duty by configured period
 * @param       epwm                  The pointer of the specified EPWM module
 * @param       u32ChannelNum        EPWM channel number. Valid values are between 0~5
 * @param       u32DutyCycle         Target generator duty cycle percentage. Valid range are between 0 ~ u32CycleResolution.
 *                                   If u32CycleResolution is 100, and u32DutyCycle is 10 means 10%, 20 means 20% ...
 * @param       u32CycleResolution   Target generator duty cycle resolution. The value in general is 100.
 * @return      The compatator value by new duty cycle
 */
uint32_t CalNewDutyCMR(EPWM_T *epwm, uint32_t u32ChannelNum, uint32_t u32DutyCycle, uint32_t u32CycleResolution)
{
    return (u32DutyCycle * (EPWM_GET_CNR(epwm, u32ChannelNum) + 1) / u32CycleResolution);
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
    /* Select EPWM1 module clock source */
    CLK_SetModuleClock(EPWM0_MODULE, CLK_EPWMSEL_EPWM0SEL_PCLK0, 0);

    /* Enable EPWM1 module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPIOE_MODULE);
    /* Set multi-function pin for EPWM */
    SET_EPWM0_CH0_PE7();

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint8_t  u8Option;
    uint32_t u32NewDutyCycle = 0, u32NewCMR = 0;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+-----------------------------------------------------------------------------------+\n");
    printf("|                     EPWM SwitchDuty Sample Code                                   |\n");
    printf("+-----------------------------------------------------------------------------------+\n");
    printf("  This sample code will use EPWM0 channel 0 to output waveform, and switch duty cycle.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: EPWM0 channel 0(PE.7)\n");
    printf("\nOutput waveform is 1800Hz and it's duty is 50%%.\n");

    /* EPWM0 channel 0 frequency is 1800Hz, duty 50%, */
    EPWM_ConfigOutputChannel(EPWM0, 0, 1800, 50);

    /* Enable output of EPWM0 channel 0 */
    EPWM_EnableOutput(EPWM0, EPWM_CH_0_MASK);

    /* Start EPWM counter */
    EPWM_Start(EPWM0, EPWM_CH_0_MASK);

    while (1)
    {
        printf("\nSelect new duty: \n");
        printf("[1] 100%% \n");
        printf("[2] 75%% \n");
        printf("[3] 25%% \n");
        printf("[4] 0%% \n");
        printf("[Other] Exit \n");
        u8Option = (uint8_t)getchar();

        if (u8Option == '1')
        {
            u32NewDutyCycle = 100;
        }
        else if (u8Option == '2')
        {
            u32NewDutyCycle = 75;
        }
        else if (u8Option == '3')
        {
            u32NewDutyCycle = 25;
        }
        else if (u8Option == '4')
        {
            u32NewDutyCycle = 0;
        }
        else
        {
            printf("Exit\n");
            break;
        }

        /* Get new comparator value by call CalNewDutyCMR() */
        u32NewCMR = CalNewDutyCMR(EPWM0, 0, u32NewDutyCycle, 100);
        /* Set new comparator value to register */
        EPWM_SET_CMR(EPWM0, 0, u32NewCMR);
    }

    /* Stop EPWM counter */
    EPWM_Stop(EPWM0, EPWM_CH_0_MASK);
    /* Disable output of EPWM0 channel 0 */
    EPWM_DisableOutput(EPWM0, EPWM_CH_0_MASK);

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
