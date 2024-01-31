/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use ADINT interrupt to do the LPADC Single-cycle scan conversion.
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
/* LPADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void LPADC0_IRQHandler(void)
{
    LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT); /* Clear the A/D interrupt flag */

    g_u32LpadcIntFlag = 1;

    /*Confirm that the Flag has been cleared.*/
    M32(&LPADC0->ADSR0);
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

    /* Enable GPIOB module clock */
    CLK_EnableModuleClock(GPIOB_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/

    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Set PB.0 ~ PB.3 to input mode */
    GPIO_SetMode(PB, BIT0 | BIT1 | BIT2 | BIT3, GPIO_MODE_INPUT);

    /* Configure the GPB0 - GPB3 LPADC analog input pins.  */
    SET_LPADC0_CH0_PB0();
    SET_LPADC0_CH1_PB1();
    SET_LPADC0_CH2_PB2();
    SET_LPADC0_CH3_PB3();

    /* Disable the GPB0 - GPB3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0 | BIT1 | BIT2 | BIT3);

}

void LPADC_FunctionTest()
{
    uint8_t  u8Option, u32ChannelCount = 0;
    int32_t  i32ConversionData[8] = {0};

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      ADINT trigger mode test                         |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* LPADC Calibration */
    LPADC_Calibration(LPADC0);

    while (1)
    {
        printf("\n\nSelect input mode:\n");
        printf("  [1] Single end input (channel 0, 1, 2 and 3)\n");
        printf("  [2] Differential input (input channel pair 0 and 1)\n");
        printf("  Other keys: exit single-cycle scan mode test\n");
        u8Option = getchar();

        if (u8Option == '1')
        {
            /* Set input mode as single-end, Single-cycle scan mode, and select channel 0~3 */
            LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_SINGLE_END, LPADC_ADCR_ADMD_SINGLE_CYCLE, BIT0 | BIT1 | BIT2 | BIT3);

            /* Clear the A/D interrupt flag for safe */
            LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT);

            /* Enable the sample module interrupt */
            LPADC_EnableInt(LPADC0, LPADC_ADF_INT);
            NVIC_EnableIRQ(LPADC0_IRQn);

            /* Reset the LPADC indicator and trigger sample module to start A/D conversion */
            g_u32LpadcIntFlag = 0;
            LPADC_START_CONV(LPADC0);

            PMC_Idle();

            /* Wait conversion done */
            while (g_u32LpadcIntFlag == 0);

            /* Wait conversion data become valid */
            while (LPADC_IS_DATA_VALID(LPADC0, 0) == 0);

            while (LPADC_IS_DATA_VALID(LPADC0, 1) == 0);

            while (LPADC_IS_DATA_VALID(LPADC0, 2) == 0);

            while (LPADC_IS_DATA_VALID(LPADC0, 3) == 0);

            /* Get the conversion result of the sample module */
            for (u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount++)
                i32ConversionData[u32ChannelCount] = LPADC_GET_CONVERSION_DATA(LPADC0, u32ChannelCount);

            for (u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount++)
                printf("Conversion result of channel %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData[u32ChannelCount], i32ConversionData[u32ChannelCount]);
        }
        else if (u8Option == '2')
        {
            /* Set input mode as differential, Single-cycle scan mode, and select channel 0 and 2 */
            LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_DIFFERENTIAL, LPADC_ADCR_ADMD_SINGLE_CYCLE, BIT0 | BIT2);

            /* Clear the A/D interrupt flag for safe */
            LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT);

            /* Enable the sample module interrupt */
            LPADC_EnableInt(LPADC0, LPADC_ADF_INT);
            NVIC_EnableIRQ(LPADC0_IRQn);

            /* Reset the LPADC indicator and trigger sample module to start A/D conversion */
            g_u32LpadcIntFlag = 0;
            LPADC_START_CONV(LPADC0);

            PMC_Idle();

            /* Wait conversion done */
            while (g_u32LpadcIntFlag == 0);

            /* Wait conversion data become valid */
            while (LPADC_IS_DATA_VALID(LPADC0, 0) == 0);

            while (LPADC_IS_DATA_VALID(LPADC0, 2) == 0);

            /* Get the conversion result of the sample module */
            for (u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount += 2)
                i32ConversionData[u32ChannelCount] = LPADC_GET_CONVERSION_DATA(LPADC0, u32ChannelCount);

            for (u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount += 2)
                printf("Conversion result of channel %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData[u32ChannelCount], i32ConversionData[u32ChannelCount]);
        }
        else
            return ;
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
    /* Disable LPADC Interrupt */
    NVIC_DisableIRQ(LPADC0_IRQn);

    printf("Exit LPADC sample code\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
