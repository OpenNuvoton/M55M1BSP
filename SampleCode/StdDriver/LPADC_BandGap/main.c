/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Convert Band-gap (channel 31) and print conversion result.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32LpadcIntFlag;

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* LPADC interrupt handler                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void LPADC0_IRQHandler(void)
{
    g_u32LpadcIntFlag = 1;
    LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT); /* Clear the A/D interrupt flag */
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Switch SCLK clock source to APLL0 and Enable APLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* LPADC clock source is HIRC = 12MHz, set divider to 1, LPADC clock is 12 MHz */
    CLK_SetModuleClock(LPADC0_MODULE, CLK_LPADCSEL_LPADC0SEL_HIRC, CLK_LPADCDIV_LPADC0DIV(1));

    /* Enable LPADC module clock */
    CLK_EnableModuleClock(LPADC0_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();
    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/

    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

}

void LPADC_FunctionTest()
{
    int32_t  i32ConversionData;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                  LPADC for Band-gap test                             |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("+----------------------------------------------------------------------+\n");
    printf("|   LPADC clock source -> HIRC  = 12 MHz                              |\n");
    printf("|   LPADC clock divider          = 1                                   |\n");
    printf("|   LPADC clock                  = 12 MHz / 1 = 12 MHz                 |\n");
    printf("|   LPADC extended sampling time = 40                                  |\n");
    printf("|   LPADC conversion time = 20 + LPADC extended sampling time = 60     |\n");
    printf("|   LPADC conversion rate = 12 MHz / 60 = 200 kSPS                     |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Set input mode as single-end, Single mode, and select channel 31 (band-gap voltage) */
    LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_SINGLE_END, LPADC_ADCR_ADMD_SINGLE, BIT31);

    /* The maximum sampling rate will be 200 kSPS for Band-gap. */
    /* Set sample module extended sampling time to 40. */
    LPADC_SetExtendSampleTime(LPADC0, 0, 40);

    /* Clear the A/D interrupt flag for safe */
    LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT);

    /* Enable the sample module interrupt. */
    LPADC_EnableInt(LPADC0, LPADC_ADF_INT);
    NVIC_EnableIRQ(LPADC0_IRQn);

    /* Reset the LPADC interrupt indicator and trigger sample module to start A/D conversion */
    g_u32LpadcIntFlag = 0;
    LPADC_START_CONV(LPADC0);

    /* Wait LPADC conversion done */
    while (g_u32LpadcIntFlag == 0);

    /* Disable the A/D interrupt */
    LPADC_DisableInt(LPADC0, LPADC_ADF_INT);

    /* Get the conversion result of the channel 31 */
    i32ConversionData = LPADC_GET_CONVERSION_DATA(LPADC0, 31);

    printf("LPADC Conversion result of Band-gap: 0x%X (%d)\n", i32ConversionData, i32ConversionData);

    printf("Band-gap voltage is %dmV if Reference voltage is %.2f V\n", (3300 * i32ConversionData) / 4095, ((float)4095 / i32ConversionData) * (float)(1.2));

}

int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    /* Init Debug UART for printf */
    InitDebugUart();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* LPADC function test */
    LPADC_FunctionTest();

    /* Disable LPADC IP clock */
    CLK_DisableModuleClock(LPADC0_MODULE);
    /* Lock protected registers */
    SYS_LockReg();
    /* Disable LPADC Interrupt */
    NVIC_DisableIRQ(LPADC0_IRQn);

    printf("Exit LPADC sample code\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
