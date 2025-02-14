/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use ADINT interrupt to do the EADC continuous scan conversion.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------------------------------*/
/* Define global variables and constants                                */
/*----------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag, g_u32COVNUMFlag = 0;

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void EADC00_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Switch SCLK clock source to APLL0 and Enable APLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Enable APLL1 200MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HXT, FREQ_200MHZ, CLK_APLL1_SELECT);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable EADC peripheral clock */
    CLK_SetModuleClock(EADC0_MODULE, CLK_EADCSEL_EADC0SEL_APLL1_DIV2, CLK_EADCDIV_EADC0DIV(10));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC0_MODULE);

    /* Enable GPB peripheral clock */
    CLK_EnableModuleClock(GPIOB_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();
    /* Set PB.0 - PB.3 to input mode */
    GPIO_SetMode(PB, BIT0 | BIT1 | BIT2 | BIT3, GPIO_MODE_INPUT);
    /* Configure the PB.0 - PB.3 ADC analog input pins. */
    SET_EADC0_CH0_PB0();
    SET_EADC0_CH1_PB1();
    SET_EADC0_CH2_PB2();
    SET_EADC0_CH3_PB3();
    /* Disable the PB.0 - PB.3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0 | BIT1 | BIT2 | BIT3);

    /* Set PB.8 - PB.9 to input mode */
    GPIO_SetMode(PB, BIT8 | BIT9, GPIO_MODE_INPUT);
    /* Configure the PB.8 - PB.9 ADC analog input pins. */
    SET_EADC0_CH8_PB8();
    SET_EADC0_CH9_PB9();
    /* Disable the PB.8 - PB.9 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT8 | BIT9);


}


void EADC_FunctionTest()
{
    uint8_t  u8Option, u32SAMPLECount = 0;
    int32_t  ai32ConversionData[8] = {0};

    /* The Maximum EADC clock frequency is 90 MHz.
     * Hence, we set PLL to 180 MHz, HCLK 180 MHz and PCLK0 to 90 MHz.
     * EADC clock source is from PCLK0/15 = 6 MHz.
     */
    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      ADINT trigger mode test                         |\n");
    printf("+----------------------------------------------------------------------+\n");

    while (1)
    {
        printf("\n\nSelect input mode:\n");
        printf("  [1] Single end input (channel 0, 1, 2 and 3)\n");
        printf("  [2] Differential input (channel pair 4: channel 8 and 9)\n");
        printf("  Other keys: exit continuous scan mode test\n");
        u8Option = getchar();

        if (u8Option == '1')
        {
            /* Set input mode as single-end and enable the A/D converter */
            EADC_Open(EADC0, EADC_CTL_DIFFEN_SINGLE_END);

            /* Configure the sample 4 module for analog input channel 0 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC0, 4, EADC_ADINT0_TRIGGER, 0);
            /* Configure the sample 5 module for analog input channel 1 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC0, 5, EADC_ADINT0_TRIGGER, 1);
            /* Configure the sample 6 module for analog input channel 2 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC0, 6, EADC_ADINT0_TRIGGER, 2);
            /* Configure the sample 7 module for analog input channel 3 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC0, 7, EADC_ADINT0_TRIGGER, 3);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

            /* Enable the sample module 7 interrupt */
            EADC_ENABLE_INT(EADC0, BIT0);
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT7);
            NVIC_EnableIRQ(EADC00_IRQn);

            /* Reset the ADC indicator and trigger sample module 7 to start A/D conversion */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            EADC_START_CONV(EADC0, BIT7);

            PMC_Idle();

            /* Disable the sample module 7 interrupt */
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT7);

            /* Wait conversion done */
            while (EADC_GET_DATA_VALID_FLAG(EADC0, (BIT7 | BIT6 | BIT5 | BIT4)) != (BIT7 | BIT6 | BIT5 | BIT4));

            /* Get the conversion result of the sample module */
            for (u32SAMPLECount = 0; u32SAMPLECount < 4; u32SAMPLECount++)
                ai32ConversionData[u32SAMPLECount] = EADC_GET_CONV_DATA(EADC0, (u32SAMPLECount + 4));

            printf("Conversion result of channel %d: 0x%X (%d)\n", 0, ai32ConversionData[0], ai32ConversionData[0]);
            printf("Conversion result of channel %d: 0x%X (%d)\n", 1, ai32ConversionData[1], ai32ConversionData[1]);
            printf("Conversion result of channel %d: 0x%X (%d)\n", 2, ai32ConversionData[2], ai32ConversionData[2]);
            printf("Conversion result of channel %d: 0x%X (%d)\n", 3, ai32ConversionData[3], ai32ConversionData[3]);

        }
        else if (u8Option == '2')
        {
            /* Set input mode as differential and enable the A/D converter */
            EADC_Open(EADC0, EADC_CTL_DIFFEN_DIFFERENTIAL);

            /* Configure the sample module 5 for analog input channel 8 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC0, 5, EADC_ADINT0_TRIGGER, 8);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

            /* Enable the sample module 5 interrupt */
            EADC_ENABLE_INT(EADC0, BIT0);
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT5);
            NVIC_EnableIRQ(EADC00_IRQn);

            /* Reset the ADC indicator and trigger sample module 5 to start A/D conversion */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            EADC_START_CONV(EADC0, BIT5);

            PMC_Idle();

            /* Disable the sample module 5 interrupt */
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT5);

            /* Wait conversion done */
            while (EADC_GET_DATA_VALID_FLAG(EADC0, BIT5) != BIT5);

            /* Get the conversion result of the sample module */
            ai32ConversionData[0] = EADC_GET_CONV_DATA(EADC0, 5);

            printf("Conversion result of channel pair 4 (channel 8/9): 0x%X (%d)\n", ai32ConversionData[0], ai32ConversionData[0]);

        }
        else
            return;
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

#if !(defined(DEBUG_ENABLE_SEMIHOST))
    /* Init Debug UART for printf */
    InitDebugUart();
#endif

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC0_MODULE);
    /* Lock protected registers */
    SYS_LockReg();
    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC00_IRQn);

    printf("Exit EADC sample code\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
