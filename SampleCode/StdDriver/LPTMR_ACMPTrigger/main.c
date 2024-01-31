/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use ACMP to trigger LPTMR0 counter reset mode.
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

NVT_ITCM void LPTMR0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    /* Clear LPTMR capture interrupt flag. */
    LPTMR_ClearCaptureIntFlag(LPTMR0);
    printf("ACMP triggered LPTMR0 reset while counter is at %d\n", LPTMR_GetCaptureData(LPTMR0));
    __DSB();
    __ISB();

    while (LPTMR_GetCaptureIntFlag(LPTMR0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for LPTMR0 IntFlag time-out!\n");
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
    /* Select LPTMR0 clock source */
    CLK_SetModuleClock(LPTMR0_MODULE, CLK_LPTMRSEL_LPTMR0SEL_HIRC, 0);

    /* Enable LPTMR0 clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);

    /* Enable ACMP clock */
    CLK_EnableModuleClock(ACMP01_MODULE);

    /* Enable GPIO Port B clock */
    CLK_EnableModuleClock(GPIOB_MODULE);
    /* Set PB4 multi-function pin for ACMP1 positive input pin */
    SET_ACMP1_P1_PB4();
    /* Disable digital input path of analog pin ACMP1_P1 to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT4);

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    int volatile i;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("System core clock = %d\n", SystemCoreClock);
    PB5 = 1;    /* Set init state to high */
    GPIO_SetMode(PB, BIT5, GPIO_MODE_OUTPUT);

    printf("\nThis sample code demonstrate ACMP trigger LPTMR0 counter reset mode.\n");
    printf("Please connect PB.5 with ACMP0 positive input pin PB.4,  press any key to continue\n");
    getchar();

    /* Give a dummy target frequency here. Will over write capture resolution with macro */
    LPTMR_Open(LPTMR0, LPTMR_PERIODIC_MODE, 1000000);

    /* Update prescale to set proper resolution. */
    LPTMR_SET_PRESCALE_VALUE(LPTMR0, 0);

    /* Set compare value as large as possible, so don't need to worry about counter overrun too frequently. */
    LPTMR_SET_CMP_VALUE(LPTMR0, 0xFFFFFF);

    /* Configure LPTMR0 counter reset mode */
    LPTMR_EnableCapture(LPTMR0, LPTMR_CAPTURE_COUNTER_RESET_MODE, LPTMR_CAPTURE_EVENT_RISING);

    /* Set capture source from Internal event ACMP1 */
    LPTMR_CaptureSelect(LPTMR0, LPTMR_CAPTURE_FROM_ACMP1);

    /* Start LPTMR0 */
    LPTMR_Start(LPTMR0);

    /* Configure ACMP1. Enable ACMP1 and select band-gap voltage as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_VBG, ACMP_CTL_HYSTERESIS_DISABLE);
    /* Select P1 as ACMP1 positive input channel */
    ACMP_SELECT_P(ACMP01, 1, ACMP_CTL_POSSEL_P1);
    /* Enable LPTMR0 interrupt */
    LPTMR_EnableCaptureInt(LPTMR0);
    NVIC_EnableIRQ(LPTMR0_IRQn);

    while (1)
    {
        PB5 = 0;
        CLK_SysTickDelay(10000);
        PB5 = 1;
        CLK_SysTickDelay(10000);
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
