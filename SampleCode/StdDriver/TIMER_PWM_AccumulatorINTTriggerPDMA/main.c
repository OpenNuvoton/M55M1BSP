/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate TIMER PWM accumulator interrupt to trigger PDMA transfer.
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

#if (NVT_DCACHE_ON == 1)
    /* Data size (g_au16Period) < one cache line size (32B) => Non-cacheable should be ok. */
    NVT_NONCACHEABLE __attribute__((aligned(4))) static uint32_t s_u32UpdatedPeriod;
#else
    __attribute__((aligned(4))) static uint32_t s_u32UpdatedPeriod;
#endif

/**
 * @brief       PDMA IRQ Handler
 * @param       None
 * @return      None
 * @details     The DMA default IRQ, declared in startup_m460.s.
 */
NVT_ITCM void PDMA0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if (u32Status & PDMA_INTSTS_ABTIF_Msk)       /* abort */
    {
        if (PDMA_GET_ABORT_STS(PDMA0) & PDMA_ABTSTS_ABTIF0_Msk)
            g_u32IsTestOver = 2;

        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF0_Msk);
    }
    else if (u32Status & PDMA_INTSTS_TDIF_Msk)  /* done */
    {
        if (PDMA_GET_TD_STS(PDMA0) & PDMA_TDSTS_TDIF0_Msk)
            g_u32IsTestOver = 1;

        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF0_Msk);
    }
    else
        printf("unknown interrupt %x !!\n", u32Status);

    __DSB();
    __ISB();

    while (PDMA_GET_INT_STATUS(PDMA0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA0 IntFlag time-out!\n");
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
    /* Enable PLL0 220MHZ clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

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

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPIOB_MODULE);
    /* Set Timer0 PWM CH0(TM0) pin */
    SET_TM0_PB5();
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
    printf("+---------------------------------------------------------------+\n");
    printf("|    Timer PWM Accumulator Inerrupt Trigger PDMA Sample Code    |\n");
    printf("+---------------------------------------------------------------+\n\n");

    printf("  This sample code demonstrate Timer0 PWM accumulator interrupt trigger PDMA.\n");
    printf("  When accumulator interrupt happens, Timer0 PWM period will be updated to (Initial Period x 2) by PDMA.\n");
    printf("    - Timer0 PWM_CH0 on PB.5\n");
    printf("  Output frequency will be updated from 18kHz to 9kHz, and duty cycle from 50%% to 25%%.\n");

    printf("\n\nPress any key to start Timer0 PWM.\n\n");
    getchar();

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER0);

    /* Set Timer0 PWM mode as independent mode */
    TPWM_ENABLE_INDEPENDENT_MODE(TIMER0);

    /* Set Timer0 PWM output frequency is 18000 Hz, duty 50% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER0, 18000, 50);
    u32InitPeriod = TPWM_GET_PERIOD(TIMER0);

    /* Set Timer0 PWM down count type */
    TPWM_SET_COUNTER_TYPE(TIMER0, TPWM_DOWN_COUNT);

    /* Enable output of Timer0 PWM_CH0 */
    TPWM_ENABLE_OUTPUT(TIMER0, TPWM_CH0);

    /* Enable Timer0 PWM accumulator function, interrupt count 10, accumulator source select to zero point */
    TPWM_EnableAcc(TIMER0, 100, TPWM_IFA_ZERO_POINT);

    /* Enable Timer0 PWM accumulator interrupt trigger PDMA */
    TPWM_EnableAccPDMA(TIMER0);

    /* Enable Timer0 PWM interrupt */
    NVIC_EnableIRQ(TIMER0_IRQn);

    /*--------------------------------------------------------------------------------------*/
    /* Configure PDMA peripheral mode form memory to TIMER PWM                              */
    /*--------------------------------------------------------------------------------------*/
    /* Open PDMA Channel 0 */
    PDMA_Open(PDMA0, BIT0);

    /* Transfer width is half word(16 bit) and transfer count is 1 */
    PDMA_SetTransferCnt(PDMA0, 0, PDMA_WIDTH_16, 1);

    /* Set updated period vaule */
    s_u32UpdatedPeriod = ((u32InitPeriod + 1) * 2) - 1;

    /* Set source address as s_u32UpdatedPeriod(no increment) and destination address as Timer0 PWM period register(no increment) */
    PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)&s_u32UpdatedPeriod, PDMA_SAR_FIX, (uint32_t) & (TIMER0->PWMPERIOD), PDMA_DAR_FIX);

    /* Select PDMA request source as PDMA_TMR0(Timer0 PWM accumulator interrupt) */
    PDMA_SetTransferMode(PDMA0, 0, PDMA_TMR0, FALSE, 0);

    /* Set PDMA as single request type for Timer0 PWM */
    PDMA_SetBurstType(PDMA0, 0, PDMA_REQ_SINGLE, PDMA_BURST_1);

    /* Enable PDMA interrupt */
    PDMA_EnableInt(PDMA0, 0, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);

    /* Start Timer0 PWM counter */
    TPWM_START_COUNTER(TIMER0);

    g_u32IsTestOver = 0;

    /* Wait for PDMA transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (g_u32IsTestOver != 1)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA transfer done time-out!\n");
            return -1;
        }
    }

    printf("Timer0 PWM period register is updated from %d to %d\n\n", u32InitPeriod, TPWM_GET_PERIOD(TIMER0));

    printf("Press any key to stop Timer0 PWM.\n\n");
    getchar();

    /* Disable PDMA function */
    PDMA_Close(PDMA0);
    NVIC_DisableIRQ(PDMA0_IRQn);

    /* Stop Timer0 PWM */
    TPWM_STOP_COUNTER(TIMER0);

    /* Wait until Timer0 PWM Stop */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while ((TIMER0->PWMCNT & TIMER_PWMCNT_CNT_Msk) != 0)
        if (--u32TimeOutCnt == 0) break;

    if (u32TimeOutCnt == 0)
        printf("Wait for Timer PWM stop time-out!\n");
    else
        printf("Timer0 PWM has STOP.\n");

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
