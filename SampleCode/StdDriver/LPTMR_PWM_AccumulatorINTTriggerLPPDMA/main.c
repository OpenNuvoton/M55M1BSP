/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate Low Power TIMER PWM accumulator interrupt to trigger LPPDMA transfer.
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
static volatile uint32_t g_u32IsTestOver = 0;
static uint32_t u32UpdatedPeriod __attribute__((section(".lpSram")));

/**
 * @brief       LPPDMA IRQ Handler
 * @param       None
 * @return      None
 * @details     The Low Power PDMA default IRQ, declared in startup_M55M1.c.
 */
NVT_ITCM void LPPDMA_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    uint32_t u32Status = LPPDMA_GET_INT_STATUS(LPPDMA);

    if (u32Status & LPPDMA_INTSTS_ABTIF_Msk)       /* abort */
    {
        if (LPPDMA_GET_ABORT_STS(LPPDMA) & LPPDMA_ABTSTS_ABTIF0_Msk)
            g_u32IsTestOver = 2;

        LPPDMA_CLR_ABORT_FLAG(LPPDMA, LPPDMA_ABTSTS_ABTIF0_Msk);
    }
    else if (u32Status & LPPDMA_INTSTS_TDIF_Msk)  /* done */
    {
        if (LPPDMA_GET_TD_STS(LPPDMA) & LPPDMA_TDSTS_TDIF0_Msk)
            g_u32IsTestOver = 1;

        LPPDMA_CLR_TD_FLAG(LPPDMA, LPPDMA_TDSTS_TDIF0_Msk);
    }
    else
        printf("unknown interrupt %x !!\n", u32Status);

    __DSB();
    __ISB();

    while (LPPDMA_GET_INT_STATUS(LPPDMA))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for LPPDMA IntFlag time-out!\n");
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
    /* Select Low Power TIMER clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_LPTMRSEL_LPTMR0SEL_HIRC, 0);
    /* Enable Low Power TIMER module clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);

    /* Enable LPPDMA module clock */
    CLK_EnableModuleClock(LPPDMA0_MODULE);
    /* Enable LPSRAM module clock */
    CLK_EnableModuleClock(LPSRAM0_MODULE);

    /* Set Low Power Timer0 PWM CH0(TM0) pin */
    CLK_EnableModuleClock(GPIOB_MODULE);
    SET_LPTM0_PB5();

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint32_t u32InitPeriod, u32TimeOutCnt;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("System core clock = %d\n", SystemCoreClock);
    printf("+---------------------------------------------------------------------------+\n");
    printf("|    Low Power Timer PWM Accumulator Inerrupt Trigger LPPDMA Sample Code    |\n");
    printf("+---------------------------------------------------------------------------+\n\n");

    printf("  This sample code demonstrate Low Power Timer0 PWM accumulator interrupt trigger LPPDMA.\n");
    printf("  When accumulator interrupt happens, Low Power Timer0 PWM period will be updated to (Initial Period x 2) by LPPDMA.\n");
    printf("    - Timer0 PWM_CH0 on PB.5\n");
    printf("  Output frequency will be updated from 18kHz to 9kHz, and duty cycle from 50%% to 25%%.\n");

    printf("\n\nPress any key to start Low Power Timer0 PWM.\n\n");
    getchar();

    /* Change Timer to PWM counter mode */
    LPTPWM_ENABLE_PWM_MODE(LPTMR0);

    /* Set Low Power Timer0 PWM output frequency is 18000 Hz, duty 50% in up count type */
    LPTPWM_ConfigOutputFreqAndDuty(LPTMR0, 18000, 50);
    u32InitPeriod = LPTPWM_GET_PERIOD(LPTMR0);

    /* Enable output of Low Power Timer0 PWM_CH0 */
    LPTPWM_ENABLE_OUTPUT(LPTMR0, LPTPWM_CH0);

    /* Enable Low Power Timer0 PWM accumulator function, interrupt count 100, accumulator source select to PERIOD point */
    LPTPWM_EnableAcc(LPTMR0, 100, LPTPWM_IFA_PERIOD_POINT);

    /* Enable Low Power Timer0 PWM accumulator interrupt trigger LPPDMA */
    LPTPWM_EnableAccLPPDMA(LPTMR0);

    /*--------------------------------------------------------------------------------------*/
    /* Configure LPPDMA peripheral mode form memory to Low Power TIMER PWM                              */
    /*--------------------------------------------------------------------------------------*/
    /* Open LPPDMA Channel 0 */
    LPPDMA_Open(LPPDMA, BIT0);

    /* Transfer width is half word(16 bit) and transfer count is 1 */
    LPPDMA_SetTransferCnt(LPPDMA, 0, LPPDMA_WIDTH_16, 1);

    /* Set updated period vaule */
    u32UpdatedPeriod = ((u32InitPeriod + 1) * 2) - 1;

    /* Set source address as u32UpdatedPeriod(no increment) and destination address as Low Power Timer0 PWM period register(no increment) */
    LPPDMA_SetTransferAddr(LPPDMA, 0, (uint32_t)&u32UpdatedPeriod, LPPDMA_SAR_FIX, (uint32_t) & (LPTMR0->PWMPERIOD), LPPDMA_DAR_FIX);

    /* Select LPPDMA request source as LPPDMA_TMR0(Low Power Timer0 PWM accumulator interrupt) */
    LPPDMA_SetTransferMode(LPPDMA, 0, LPPDMA_LPTMR0, FALSE, 0);

    /* Set LPPDMA as single request type for Low Power Timer0 PWM */
    LPPDMA_SetBurstType(LPPDMA, 0, LPPDMA_REQ_SINGLE, LPPDMA_BURST_1);

    /* Enable LPPDMA interrupt */
    LPPDMA_EnableInt(LPPDMA, 0, LPPDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(LPPDMA_IRQn);

    g_u32IsTestOver = 0;

    /* Wait for LPPDMA transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    /* Start Low Power Timer0 PWM counter */
    LPTPWM_START_COUNTER(LPTMR0);

    while (g_u32IsTestOver != 1)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for LPPDMA transfer done time-out!\n");
            return -1;
        }
    }

    printf("Low Power Timer0 PWM period register is updated from %d to %d\n\n", u32InitPeriod, LPTPWM_GET_PERIOD(LPTMR0));

    printf("Press any key to stop Low Power Timer0 PWM.\n\n");
    getchar();

    /* Disable LPPDMA function */
    LPPDMA_Close(LPPDMA);

    /* Stop Low Power Timer0 PWM */
    LPTPWM_STOP_COUNTER(LPTMR0);

    /* Wait until Low Power Timer0 PWM Stop */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while ((LPTMR0->PWMCNT & LPTMR_PWMCNT_CNT_Msk) != 0)
        if (--u32TimeOutCnt == 0) break;

    if (u32TimeOutCnt == 0)
        printf("Wait for Low Power Timer PWM stop time-out!\n");
    else
        printf("Low Power Timer0 PWM has STOP.\n");

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
