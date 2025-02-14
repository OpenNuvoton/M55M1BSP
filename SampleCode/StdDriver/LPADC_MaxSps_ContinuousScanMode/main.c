/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to use PCLK4 as LPADC clock source to achieve maximum kSPS LPADC conversion rate.
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

    /* Switch SCLK clock source to APLL0 and Enable APLL0 200MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_200MHZ);

    /* PCLK4 source is APLL0 = 200MHz, set divider to 4, PCLK4 is 50 MHz */
    CLK_SET_PCLK4DIV(4);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* LPADC clock source is PCLK4 = 50MHz, set divider to 1, LPADC clock is 50 MHz */
    CLK_SetModuleClock(LPADC0_MODULE, CLK_LPADCSEL_LPADC0SEL_PCLK4, CLK_LPADCDIV_LPADC0DIV(1));

    /* Enable LPADC module clock */
    CLK_EnableModuleClock(LPADC0_MODULE);

    /* Enable GPIOB module clock */
    CLK_EnableModuleClock(GPIOB_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Debug UART clock setting*/
    SetDebugUartCLK();

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
    uint32_t u32ChannelCount;
    int32_t  i32ConversionData;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|           LPADC Maximum kSPS continuous scan mode sample code        |\n");
    printf("+----------------------------------------------------------------------+\n\n");

    printf("+----------------------------------------------------------------------+\n");
    printf("|   LPADC clock source -> PCLK4  =  50 MHz                             |\n");
    printf("|   LPADC clock divider          = 1                                   |\n");
    printf("|   LPADC clock                  = 50 MHz / 1 = 50 MHz                 |\n");
    printf("|   LPADC extended sampling time = 0                                   |\n");
    printf("|   LPADC conversion time = 25 + LPADC extended sampling time = 25     |\n");
    printf("|   LPADC conversion rate = 50 MHz / 25 = 2.0 MSPS                     |\n");
    printf("+----------------------------------------------------------------------+\n");

    while (1)
    {
        printf(" Press any key to start the continuous scan mode test\n");
        getchar();
        /* Set the LPADC operation mode as continuous scan, input mode as single-end and
           enable the analog input channel 0, 1, 2 and 3 */
        LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_SINGLE_END, LPADC_ADCR_ADMD_CONTINUOUS, BIT0 | BIT1 | BIT2 | BIT3);

        /* Clear the A/D interrupt flag for safe */
        LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT);

        /* Enable the sample module interrupt */
        LPADC_EnableInt(LPADC0, LPADC_ADF_INT);
        NVIC_EnableIRQ(LPADC0_IRQn);

        /* Reset the LPADC interrupt indicator and trigger sample module 0 to start A/D conversion */
        g_u32LpadcIntFlag = 0;
        LPADC_START_CONV(LPADC0);

        /* Wait LPADC interrupt (g_u32LpadcIntFlag will be set at LPADC0_IRQHandler function) */
        while (g_u32LpadcIntFlag == 0);

        /* Get the conversion result */
        for (u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount++)
        {
            i32ConversionData = LPADC_GET_CONVERSION_DATA(LPADC0, u32ChannelCount);
            printf("Conversion result of channel %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
        }

        printf("\n");

        /* Stop A/D conversion */
        LPADC_STOP_CONV(LPADC0);

        /* Disable the sample module interrupt */
        LPADC_DisableInt(LPADC0, LPADC_ADF_INT);
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
