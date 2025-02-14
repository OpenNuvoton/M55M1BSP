/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to trigger LPADC by ACMP.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32LpadcIntFlag, g_u32COVNUMFlag = 0, g_u32ACMP1IntFlag = 0;

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* LPADC interrupt handler                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void LPADC0_IRQHandler(void)
{
    LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT); /* Clear the A/D interrupt flag */

    g_u32LpadcIntFlag = 1;

    g_u32COVNUMFlag++;
    /*Confirm that the Flag has been cleared.*/
    M32(&LPADC0->ADSR0);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ACMP interrupt handler                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void ACMP01_IRQHandler(void)
{
    /* Clear ACMP 1 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 1);
    g_u32ACMP1IntFlag = 1;

    /*Confirm that the Flag has been cleared.*/
    M32(&ACMP01->STATUS);
}

void SYS_Init(void)
{

    /* Switch SCLK clock source to APLL0 and Enable APLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* LPADC clock source is HIRC = 12MHz, set divider to 1, LPADC clock is 12 MHz */
    CLK_SetModuleClock(LPADC0_MODULE, CLK_LPADCSEL_LPADC0SEL_HIRC, CLK_LPADCDIV_LPADC0DIV(1));

    /* Enable LPADC peripheral clock */
    CLK_EnableModuleClock(LPADC0_MODULE);

    /* Enable ACMP01 peripheral clock */
    CLK_EnableModuleClock(ACMP01_MODULE);

    /* Enable GPA peripheral clock */
    CLK_EnableModuleClock(GPIOA_MODULE);

    /* Enable GPB peripheral clock */
    CLK_EnableModuleClock(GPIOB_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Set PA.10 to input mode */
    GPIO_SetMode(PA, BIT10, GPIO_MODE_INPUT);
    /* Set PB.0 - PB.1 to input mode */
    GPIO_SetMode(PB, BIT0 | BIT1, GPIO_MODE_INPUT);

    /* Configure the PB.0 - PB.1 LPADC analog input pins. */
    SET_LPADC0_CH0_PB0();
    SET_LPADC0_CH1_PB1();

    /* Set PA10 multi-function pin for ACMP1 positive input pin */
    SET_ACMP1_P0_PA10();
    /* Disable the PA10 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PA, BIT10);
    /* Disable the PB.0 - PB.1 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0 | BIT1);

}

void ACMP1_Init(void)
{
    /* Select P1 as ACMP positive input channel */
    ACMP_SELECT_P(ACMP01, 1, ACMP_CTL_POSSEL_P0);
    /* Configure ACMP1. Enable ACMP1 and select VBG output as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_VBG, ACMP_CTL_HYSTERESIS_DISABLE);
}

void LPADC_FunctionTest(void)
{
    uint8_t  u8Option;
    uint8_t  u8Index = 0;
    uint32_t u32COVNUMFlag = 0;
    int32_t  ai32ConversionData[6] = {0};

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                   LPADC trigger by Timer test                        |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel within 1 second.\n");

    while (1)
    {
        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 0 only)\n");
        printf("  [2] Differential input (channel pair 0 only)\n");
        printf("  Other keys: exit single mode test\n");
        u8Option = getchar();

        if (u8Option == '1')
        {
            /* Set input mode as single-end, Single mode, and select channel 0 */
            LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_SINGLE_END, LPADC_ADCR_ADMD_SINGLE, BIT0);

            /* Configure the sample module and enable ACMP1 trigger source */
            LPADC_EnableHWTrigger(LPADC0, LPADC_ACMP1_TRIGGER, 0);

            /* Clear the A/D interrupt flag for safe */
            LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT);
            /* Clear ACMP 1 interrupt flag */
            ACMP_CLR_INT_FLAG(ACMP01, 1);

            /* Enable the sample module interrupt */
            LPADC_EnableInt(LPADC0, LPADC_ADF_INT);
            /* Enable interrupt */
            ACMP_ENABLE_INT(ACMP01, 1);

            /* Enable ACMP01 interrupt */
            NVIC_EnableIRQ(ACMP01_IRQn);
            /* Enable LPADC0 interrupt */
            NVIC_EnableIRQ(LPADC0_IRQn);
            /* Reset the LPADC and ACMP indicator counter */
            g_u32LpadcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            g_u32ACMP1IntFlag = 0;

            printf("Conversion result of channel 0:\n");

            while (1)
            {
                /* Wait LPADC interrupt (g_u32LpadcIntFlag will be set at IRQ_Handler function) */
                /* Wait ACMP interrupt (g_u32ACMP1IntFlag will be set at IRQ_Handler function) */
                while (g_u32LpadcIntFlag == 0 || g_u32ACMP1IntFlag == 0) {};

                /* Reset the LPADC interrupt indicator */
                g_u32LpadcIntFlag = 0;

                /* Reset the ACMP1 interrupt indicator */
                g_u32ACMP1IntFlag = 0;

                /* Get the conversion result of LPADC channel 0 */
                u32COVNUMFlag = g_u32COVNUMFlag - 1;

                ai32ConversionData[u32COVNUMFlag] = LPADC_GET_CONVERSION_DATA(LPADC0, 0);

                if (g_u32COVNUMFlag >= 6)
                    break;

            }

            /* Disable the  ACMP1 interrupt */
            ACMP_DISABLE_INT(ACMP01, 1);
            /* Disable the sample module interrupt */
            LPADC_DisableInt(LPADC0, LPADC_ADF_INT);

            for (u8Index = 0; (u8Index) < 6; u8Index++)
                printf("                                0x%X (%d)\n", ai32ConversionData[u8Index], ai32ConversionData[u8Index]);
        }
        else if (u8Option == '2')
        {
            /* Set input mode as differential, Single mode, and select channel 0 */
            LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_DIFFERENTIAL, LPADC_ADCR_ADMD_SINGLE, BIT0);

            /* Configure the sample module and enable ACMP1 trigger source */
            LPADC_EnableHWTrigger(LPADC0, LPADC_ACMP1_TRIGGER, 0);

            /* Clear the A/D interrupt flag for safe */
            LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT);
            /* Clear ACMP 1 interrupt flag */
            ACMP_CLR_INT_FLAG(ACMP01, 1);

            /* Enable the sample module interrupt */
            LPADC_EnableInt(LPADC0, LPADC_ADF_INT);
            /* Enable interrupt */
            ACMP_ENABLE_INT(ACMP01, 1);

            /* Enable ACMP01 interrupt */
            NVIC_EnableIRQ(ACMP01_IRQn);
            /* Enable LPADC0 interrupt */
            NVIC_EnableIRQ(LPADC0_IRQn);
            /* Reset the LPADC and ACMP indicator counter */
            g_u32LpadcIntFlag = 0;
            g_u32COVNUMFlag = 0;

            printf("Conversion result of channel pair 0:\n");

            while (1)
            {

                /* Wait LPADC interrupt (g_u32LpadcIntFlag will be set at IRQ_Handler function) */
                /* Wait ACMP interrupt (g_u32ACMP1IntFlag will be set at IRQ_Handler function) */
                while (g_u32LpadcIntFlag == 0 || g_u32ACMP1IntFlag == 0) {};

                /* Reset the LPADC interrupt indicator */
                g_u32LpadcIntFlag = 0;

                /* Reset the ACMP1 interrupt indicator */
                g_u32ACMP1IntFlag = 0;

                /* Get the conversion result of the sample module 0 */
                u32COVNUMFlag = g_u32COVNUMFlag - 1;

                ai32ConversionData[u32COVNUMFlag] = LPADC_GET_CONVERSION_DATA(LPADC0, 0);

                if (g_u32COVNUMFlag >= 6)
                    break;

            }

            /* Disable the  ACMP1 interrupt */
            ACMP_DISABLE_INT(ACMP01, 1);
            /* Disable the sample module interrupt */
            LPADC_DisableInt(LPADC0, LPADC_ADF_INT);

            for (u8Index = 0; (u8Index) < 6; u8Index++)
                printf("                                0x%X (%d)\n", ai32ConversionData[u8Index], ai32ConversionData[u8Index]);
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

    /* Init ACMP1 for LPADC */
    ACMP1_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* LPADC function test */
    LPADC_FunctionTest();

    /* Disable ACMP01 IP clock */
    CLK_DisableModuleClock(ACMP01_MODULE);

    /* Disable LPADC IP clock */
    CLK_DisableModuleClock(LPADC0_MODULE);
    /* Lock protected registers */
    SYS_LockReg();
    /* Disable ACMP01 Interrupt */
    NVIC_DisableIRQ(ACMP01_IRQn);
    /* Disable LPADC Interrupt */
    NVIC_DisableIRQ(LPADC0_IRQn);

    printf("Exit LPADC sample code\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
