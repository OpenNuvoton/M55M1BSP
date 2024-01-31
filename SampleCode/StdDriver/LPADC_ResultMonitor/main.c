/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Monitor the conversion result of channel 1 by the digital compare function.
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
volatile uint32_t g_u32LpadcCmp0IntFlag;
volatile uint32_t g_u32LpadcCmp1IntFlag;

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* LPADC interrupt handler                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void LPADC0_IRQHandler(void)
{
    if (LPADC_GET_INT_FLAG(LPADC0, LPADC_CMP0_INT))
    {
        g_u32LpadcCmp0IntFlag = 1;
        LPADC_CLR_INT_FLAG(LPADC0, LPADC_CMP0_INT);    /* Clear the A/D compare flag 0 */
    }

    if (LPADC_GET_INT_FLAG(LPADC0, LPADC_CMP1_INT))
    {
        g_u32LpadcCmp1IntFlag = 1;
        LPADC_CLR_INT_FLAG(LPADC0, LPADC_CMP1_INT);    /* Clear the A/D compare flag 1 */
    }

    g_u32LpadcIntFlag = 1;
    LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable External RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HXTEN_Msk);

    /* Waiting for External RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch SCLK clock source to APLL0 and Enable APLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

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

    /* Set PB.1 to input mode */
    GPIO_SetMode(PB, BIT1, GPIO_MODE_INPUT);

    /* Configure the PB.1 LPADC analog input pins.  */
    SET_EADC0_CH1_PB1();

    /* Disable the PB.1 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT1);

}

void LPADC_FunctionTest()
{
    uint32_t u32ConversionData;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|          LPADC compare function (result monitor) sample code         |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\nIn this test, software will compare the conversion result of channel 1.\n");

    /* LPADC Calibration */
    LPADC_Calibration(LPADC0);

    /* Set input mode as single-end, Single mode, and select channel 1 */
    LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_SINGLE_END, LPADC_ADCR_ADMD_SINGLE, BIT1);

    /* Enable LPADC comparator 0. Compare condition: conversion result < 0x800; match Count=5 */
    printf("   Set the compare condition of comparator 0: channel 1 is less than 0x800; match count is 5.\n");
    LPADC_ENABLE_CMP0(LPADC0, 1, LPADC_ADCMPR_CMPCOND_LESS_THAN, 0x800, 5);

    /* Enable LPADC comparator 1. Compare condition: conversion result >= 0x800; match Count=5 */
    printf("   Set the compare condition of comparator 1: channel 1 is greater than or equal to 0x800; match count is 5.\n");
    LPADC_ENABLE_CMP1(LPADC0, 1, LPADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL, 0x800, 5);

    /* Clear the A/D interrupt flag for safe */
    LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT);

    /* Enable the sample module interrupt */
    LPADC_EnableInt(LPADC0, LPADC_ADF_INT);
    NVIC_EnableIRQ(LPADC0_IRQn);

    /* Clear the LPADC comparator 0 interrupt flag for safe */
    LPADC_CLR_INT_FLAG(LPADC0, LPADC_CMP0_INT);
    /* Enable LPADC comparator 0 interrupt */
    LPADC_EnableInt(LPADC0, LPADC_CMP0_INT);

    /* Clear the LPADC comparator 1 interrupt flag for safe */
    LPADC_CLR_INT_FLAG(LPADC0, LPADC_CMP1_INT);
    /* Enable LPADC comparator 1 interrupt */
    LPADC_EnableInt(LPADC0, LPADC_CMP1_INT);

    /* Reset the LPADC interrupt indicator and trigger sample module to start A/D conversion */
    g_u32LpadcIntFlag = 0;
    g_u32LpadcCmp0IntFlag = 0;
    g_u32LpadcCmp1IntFlag = 0;
    LPADC_START_CONV(LPADC0);

    /* Wait LPADC compare interrupt */
    while (1)
    {
        if (g_u32LpadcIntFlag == 1)
        {
            u32ConversionData = LPADC_GET_CONVERSION_DATA(LPADC0, 1);
            printf("Conversion result of channel 1: 0x%03X (%d)\n", u32ConversionData, u32ConversionData);

            if ((g_u32LpadcCmp0IntFlag == 1) || (g_u32LpadcCmp1IntFlag == 1))
                break;

            g_u32LpadcIntFlag = 0;
            LPADC_START_CONV(LPADC0);
        }
    }

    /* Disable LPADC comparator interrupt */
    LPADC_DisableInt(LPADC0, LPADC_CMP0_INT);
    LPADC_DisableInt(LPADC0, LPADC_CMP1_INT);
    /* Disable compare function */
    LPADC_DISABLE_CMP0(LPADC0);
    LPADC_DISABLE_CMP1(LPADC0);

    if (g_u32LpadcCmp0IntFlag == 1)
    {
        printf("Comparator 0 interrupt occurs.\nThe conversion result of channel 1 is less than 0x800\n");
    }
    else
    {
        printf("Comparator 1 interrupt occurs.\nThe conversion result of channel 1 is greater than or equal to 0x800\n");
    }
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

    /* Disable External Interrupt */
    NVIC_DisableIRQ(LPADC0_IRQn);

    printf("Exit LPADC sample code\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
