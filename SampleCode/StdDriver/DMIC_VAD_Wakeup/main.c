/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use DMIC_VAD to wake up system from Power-down mode periodically.
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

#define VAD_DETECT_SAMPLERATE      (8000)
#define VAD_STABLECNT      (100)

static volatile uint32_t g_u32PDWK;

void VAD_Start(void)
{
    DMIC_VAD_ENABLE(VAD0);
}

void VAD_Stop(void)
{
    DMIC_VAD_DISABLE(VAD0);
    NVIC_DisableIRQ(DMIC0VAD_IRQn);
}

void VAD_Init(void)
{
    DMIC_VAD_BIQ_T sBIQCoeff;

    // (2) Enable VAD and input its detect parameter.
    // Enable VAD down sample rate 48.
    DMIC_VAD_SET_DOWNSAMPLE(VAD0, DMIC_VAD_DOWNSAMPLE_48);
    // Set detect sample rate.
    printf("VAD SampleRate is %d\n", DMIC_VAD_SetSampleRate(VAD0, VAD_DETECT_SAMPLERATE));
    //VAD0->SINCCTL = 0x30210;
    //printf("VAD SampleRate is %d\n", DMIC_VAD_GetSampleRate(VAD0));
    // Writer BIQ coefficient(a0,a1,b0,b1,b2)
    sBIQCoeff.u16BIQCoeffA1 = 0xC174;//0xC249;//0xC2EC;//0xC715;
    sBIQCoeff.u16BIQCoeffA2 = 0x1F23;//0x1E01;//0x1D86;//0x19A0;
    sBIQCoeff.u16BIQCoeffB0 = 0x016B;//0x01F7;//0x0233;//0x1CA3;
    sBIQCoeff.u16BIQCoeffB1 = 0xFE0C;//0xFE12;//0xFE17;//0xC6BB;
    sBIQCoeff.u16BIQCoeffB2 = 0x008E;//0xFFF9;//0xFFB9;//0x1CA3;
    DMIC_VAD_SetBIQCoeff(VAD0, &sBIQCoeff);
    // Enable VAD BIQ filter.
    DMIC_VAD_ENABLE_BIQ(VAD0);
    // Set short term attack time.
    DMIC_VAD_SET_STAT(VAD0, DMIC_VAD_STAT_2MS);
    // Set long term attack time
    DMIC_VAD_SET_LTAT(VAD0, DMIC_VAD_LTAT_64MS);
    // Set short term power threshold.
    DMIC_VAD_SET_STTHRE(VAD0, DMIC_VAD_POWERTHRE_0DB);
    // Set long term power threshold.
    DMIC_VAD_SET_LTTHRE(VAD0, DMIC_VAD_POWERTHRE_M90DB);
    // Set deviation threshold.
    DMIC_VAD_SET_DEVTHRE(VAD0, DMIC_VAD_POWERTHRE_0DB);
    NVIC_DisableIRQ(DMIC0VAD_IRQn);
}

void VAD_WaitStable(uint32_t u32StableCount)
{
    while (u32StableCount--)
    {
        while ((DMIC_VAD_GET_DEV(VAD0) > DMIC_VAD_POWERTHRE_M90DB) || (DMIC_VAD_GET_STP(VAD0) > DMIC_VAD_POWERTHRE_M70DB)) {};

        while (DMIC_VAD_IS_ACTIVE(VAD0))
        {
            DMIC_VAD_CLR_ACTIVE(VAD0);
        }
    }
}

NVT_ITCM void DMIC0VAD_IRQHandler()
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    CLK_WaitModuleClockReady(DMIC0_MODULE);//TESTCHIP_ONLY
    CLK_WaitModuleClockReady(DEBUG_PORT_MODULE);//TESTCHIP_ONLY

    //printf(",DMIC_VAD_GET_STP(VAD0) %lx,DMIC_VAD_GET_DEV(VAD0) %lx\n", DMIC_VAD_GET_STP(VAD0),DMIC_VAD_GET_DEV(VAD0));
    DMIC_VAD_CLR_ACTIVE(VAD0);

    __DSB();
    __ISB();

    while (DMIC_VAD_IS_ACTIVE(VAD0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for VAD0 IntFlag time-out!\n");
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Power Wake-up IRQ Handler                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void PMC_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    g_u32PDWK = PMC_GetPMCWKSrc();
    CLK_WaitModuleClockReady(DEBUG_PORT_MODULE);//TESTCHIP_ONLY

    /* check power down wakeup flag */
    if (g_u32PDWK & PMC_INTSTS_PDWKIF_Msk)
    {
        PMC->INTSTS |= PMC_INTSTS_CLRWK_Msk;

        __DSB();
        __ISB();

        while (PMC_GetPMCWKSrc() & PMC_INTSTS_PDWKIF_Msk)
        {
            if (--u32TimeOutCnt == 0)
            {
                printf("Wait for PMC IntFlag time-out!\n");
            }
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    SYS_UnlockReg();
    PMC_ENABLE_INT();
    NVIC_EnableIRQ(PMC_IRQn);
    /* Switch SCLK clock source to HIRC */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(DEBUG_PORT);
    /* Set Power-down mode */
    PMC_SetPowerDownMode(PMC_NPD0, PMC_PLCTL_PLSEL_PL1);

    // Set short term power threshold.
    DMIC_VAD_SET_STTHRE(VAD0, DMIC_VAD_POWERTHRE_M60DB);
    // Set deviation threshold.
    DMIC_VAD_SET_DEVTHRE(VAD0, DMIC_VAD_POWERTHRE_M60DB);
    VAD_WaitStable(VAD_STABLECNT);
    NVIC_EnableIRQ(DMIC0VAD_IRQn);
    /* Enter to Power-down mode */
    PMC_PowerDown();
    /* Switch SCLK clock source to PLL0 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);
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

    // (1) Enable DMIC for VAD(Voice active detection)
    // Select DMIC_VAD CLK source from HIRC.
    CLK_SetModuleClock(DMIC0_MODULE, CLK_DMICSEL_DMIC0SEL_HIRC, 0UL);
    CLK_SetModuleClock(VAD0SEL_MODULE, CLK_DMICSEL_VAD0SEL_HIRC, 0UL);
    // Enable DMIC_VAD clock.
    CLK_EnableModuleClock(VAD0SEL_MODULE);
    // DMIC_VAD IPReset.
    SYS_ResetModule(SYS_DMIC0RST);
    CLK_EnableModuleClock(GPIOB_MODULE);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    CLK_EnableModuleClock(GPIOB_MODULE);
    SET_DMIC0_DAT_PB5();
    SET_DMIC0_CLKLP_PB6();
    SYS->GPB_MFOS = BIT5;
    PB5 = 1;
    SET_CLKO_PG15();
    CLK_EnableCKO(CLK_CLKOSEL_CLKOSEL_HXT, 0, 1);
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
    printf("DMIC_VAD power down/wake up sample code\n");
    printf("  !! Connect PB.4 <--> PB.6 !!\n\n");
    VAD_Init();
    VAD_Start();
    g_u32PDWK = 0;

    printf(" Press any key to Enter Power-down !\n");
    getchar();

    VAD_WaitStable(VAD_STABLECNT);
    printf("Enter Power-down !\n");
    PowerDownFunction();

    while (!g_u32PDWK) {};

    printf("Wake Up PASS\n");

    while (DMIC_VAD_IS_ACTIVE(VAD0))
    {
        DMIC_VAD_CLR_ACTIVE(VAD0);
    }

    VAD_Stop();

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
