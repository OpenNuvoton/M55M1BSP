/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use BPWM0 channel 0 to capture the BPWM1 channel 0 waveform.
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

/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* au32Count[4] : Keep the internal counter value when input signal rising / falling    */
/*               happens                                                                */
/*                                                                                      */
/* time    A    B     C     D                                                           */
/*           ___   ___   ___   ___   ___   ___   ___   ___                              */
/*      ____|   |_|   |_|   |_|   |_|   |_|   |_|   |_|   |_____                        */
/* index              0 1   2 3                                                         */
/*                                                                                      */
/* The capture internal counter down count from 0x10000, and reload to 0x10000 after    */
/* input signal falling happens (Time B/C/D)                                            */
/*--------------------------------------------------------------------------------------*/
int32_t CalPeriodTime(BPWM_T *BPWM, uint32_t u32Ch)
{
    uint16_t au32Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;
    uint32_t u32TimeOutCnt;

    /* Clear Capture Falling Indicator (Time A) */
    BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH | BPWM_CAPTURE_INT_RISING_LATCH);

    /* Wait for Capture Falling Indicator */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 2)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for BPWM Capture Falling Indicator time-out!\n");
            return -1;
        }
    }

    /* Clear Capture Falling Indicator (Time B) */
    BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH);

    u32i = 0;

    while (u32i < 4)
    {
        /* Wait for Capture Falling Indicator */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 2)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM Capture Falling Indicator time-out!\n");
                return -1;
            }
        }

        /* Clear Capture Falling and Rising Indicator */
        BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH | BPWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Falling Latch Counter Data */
        au32Count[u32i++] = (uint16_t)BPWM_GET_CAPTURE_FALLING_DATA(BPWM, u32Ch);

        /* Wait for Capture Rising Indicator */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 1)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM Capture Rising Indicator time-out!\n");
                return -1;
            }
        }

        /* Clear Capture Rising Indicator */
        BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Rising Latch Counter Data */
        au32Count[u32i++] = (uint16_t)BPWM_GET_CAPTURE_RISING_DATA(BPWM, u32Ch);
    }

    u16RisingTime = au32Count[1];

    u16FallingTime = au32Count[0];

    u16HighPeriod = au32Count[1] - au32Count[2];

    u16LowPeriod = (uint16_t)(0x10000 - au32Count[3]);

    u16TotalPeriod = (uint16_t)(0x10000 - au32Count[2]);

    printf("\nBPWM generate: \nHigh Period=17999 ~ 18001, Low Period=41999 ~ 42001, Total Period=59999 ~ 60001\n");
    printf("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod);

    if ((u16HighPeriod < 17999) || (u16HighPeriod > 18001) || (u16LowPeriod < 41999) || (u16LowPeriod > 42001) || (u16TotalPeriod < 59999) || (u16TotalPeriod > 60001))
    {
        printf("Capture Test Fail!!\n");
        return -1;
    }
    else
    {
        printf("Capture Test Pass!!\n");
        return 0;
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
    SET_BPWM1_CH0_PB11();
    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint32_t u32TimeOutCnt;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                         BPWM Capture Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use BPWM0 channel 0 to capture\n  the signal from BPWM1 channel 0.\n");
    printf("  I/O configuration:\n");
    printf("    BPWM0_CH0(PE.2 BPWM0 channel 0) <--> BPWM1_CH0(PB.11 BPWM1 channel 0)\n\n");
    printf("Use BPWM0 Channel 0(PE.2) to capture the BPWM1 Channel 0(PB.11) Waveform\n");

    while (1)
    {
        printf("Press any key to start BPWM Capture Test\n");
        getchar();

        /*--------------------------------------------------------------------------------------*/
        /* Set the BPWM1 Channel 0 as BPWM output function.                                     */
        /*--------------------------------------------------------------------------------------*/

        /* Assume BPWM output frequency is 250Hz and duty ratio is 30%, user can calculate BPWM settings by follows.(up counter type)
           duty ratio = (CMR)/(CNR+1)
           cycle time = CNR+1
           High level = CMR
           BPWM clock source frequency = PLL/2 = 90000000
           (CNR+1) = PWM clock source frequency/prescaler/BPWM output frequency
                   = 90000000/6/250 = 60000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 59999
           duty ratio = 30% ==> (CMR)/(CNR+1) = 30%
           CMR = 18000
           Prescale value is 5 : prescaler= 6
        */

        /* Set BPWM1 channel 0 output configuration */
        printf("Set 250Hz, real is %d\n", BPWM_ConfigOutputChannel(BPWM1, 0, 250, 30));

        /* Enable BPWM Output path for BPWM1 channel 0 */
        BPWM_EnableOutput(BPWM1, BPWM_CH_0_MASK);

        /* Enable Timer for BPWM1 channel 0 */
        BPWM_Start(BPWM1, BPWM_CH_0_MASK);

        /*--------------------------------------------------------------------------------------*/
        /* Set the BPWM0 channel 0 for capture function                                         */
        /*--------------------------------------------------------------------------------------*/

        /* If input minimum frequency is 250Hz, user can calculate capture settings by follows.
           Capture clock source frequency = PLL = 100000000 in the sample code.
           (CNR+1) = Capture clock source frequency/prescaler/minimum input frequency
                   = 100000000/7/250 = 57142
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 0xFFFF
           (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)

           Capture unit time = 1/Capture clock source frequency/prescaler
           70 ns = 1/100000000/7
        */

        /* Set BPWM0 channel 0 capture configuration */
        BPWM_ConfigCaptureChannel(BPWM0, 0, 70, 0);
        BPWM_SET_PRESCALER(BPWM0, 0, (BPWM1)->CLKPSC);//PRESCALER same with BPWM1

        /* Enable Timer for BPWM0 channel 0 */
        BPWM_Start(BPWM0, BPWM_CH_0_MASK);

        /* Enable Capture Function for BPWM0 channel 0 */
        BPWM_EnableCapture(BPWM0, BPWM_CH_0_MASK);

        /* Enable falling capture reload */
        BPWM0->CAPCTL |= BPWM_CAPCTL_FCRLDEN0_Msk;

        /* Wait until BPWM0 channel 0 Timer start to count */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while ((BPWM0->CNT) == 0)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM channel 0 Timer start to count time-out!\n");
                goto lexit;
            }
        }

        /* Capture the Input Waveform Data */
        if (CalPeriodTime(BPWM0, 0) < 0)
            goto lexit;

        /*------------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM1 channel 0 (Recommended procedure method 1)                                                      */
        /* Set BPWM Timer loaded value(Period) as 0. When BPWM internal counter(CNT) reaches to 0, disable BPWM Timer */
        /*------------------------------------------------------------------------------------------------------------*/

        /* Set BPWM1 channel 0 loaded value as 0 */
        BPWM_Stop(BPWM1, BPWM_CH_0_MASK);

        /* Wait until BPWM1 channel 0 Timer Stop */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while ((BPWM1->CNT & BPWM_CNT_CNT_Msk) != 0)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM channel 0 Timer Stop time-out!\n");
                goto lexit;
            }
        }

        /* Disable Timer for BPWM1 channel 0 */
        BPWM_ForceStop(BPWM1, BPWM_CH_0_MASK);

        /* Disable BPWM Output path for BPWM1 channel 0 */
        BPWM_DisableOutput(BPWM1, BPWM_CH_0_MASK);

        /*------------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM0 channel 0 (Recommended procedure method 1)                                                      */
        /* Set BPWM Timer loaded value(Period) as 0. When BPWM internal counter(CNT) reaches to 0, disable BPWM Timer */
        /*------------------------------------------------------------------------------------------------------------*/

        /* Set loaded value as 0 for BPWM0 channel 0 */
        BPWM_Stop(BPWM0, BPWM_CH_0_MASK);

        /* Wait until BPWM0 channel 0 current counter reach to 0 */
        u32TimeOutCnt = SystemCoreClock;  /* 1 second time-out */

        while ((BPWM0->CNT & BPWM_CNT_CNT_Msk) != 0)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM channel 0 current counter reach to 0 time-out!\n");
                goto lexit;
            }
        }

        /* Disable Timer for BPWM0 channel 0 */
        BPWM_ForceStop(BPWM0, BPWM_CH_0_MASK);

        /* Disable Capture Function and Capture Input path for BPWM0 channel 0 */
        BPWM_DisableCapture(BPWM0, BPWM_CH_0_MASK);

        /* Clear Capture Interrupt flag for BPWM0 channel 0 */
        BPWM_ClearCaptureIntFlag(BPWM0, 0, BPWM_CAPTURE_INT_FALLING_LATCH);
    }

lexit:

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
