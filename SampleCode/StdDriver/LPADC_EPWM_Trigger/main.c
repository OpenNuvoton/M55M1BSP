/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to trigger LPADC by EPWM.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag, g_u32COVNUMFlag = 0;

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* LPADC interrupt handler                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void LPADC0_IRQHandler(void)
{
    LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT); /* Clear the A/D interrupt flag */

    g_u32AdcIntFlag = 1;
    g_u32COVNUMFlag++;

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

    /* Select EPWM0 module clock source as PCLK0 */
    CLK_SetModuleClock(EPWM0_MODULE, CLK_EPWMSEL_EPWM0SEL_PCLK0, 0);

    /* Enable EPWM0 module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);

    /* Enable GPIOB module clock */
    CLK_EnableModuleClock(GPIOB_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();
    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Set PB.0 - PB.1 to input mode */
    GPIO_SetMode(PB, BIT0 | BIT1, GPIO_MODE_INPUT);

    /* Configure the PB.0 - PB.1 LPADC analog input pins.  */
    SET_EADC0_CH0_PB0();
    SET_EADC0_CH1_PB1();

    /* Disable the PB.0 - PB.1 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0 | BIT1);


}

void EPWM0_Init()
{
    /* Set EPWM0 timer clock prescaler */
    EPWM_SET_PRESCALER(EPWM0, 0, 10);

    /* Set up counter type */
    EPWM0->CTL1 &= ~EPWM_CTL1_CNTTYPE0_Msk;

    /* Set EPWM0 timer duty */
    EPWM_SET_CMR(EPWM0, 0, 1000);

    /* Set EPWM0 timer period */
    EPWM_SET_CNR(EPWM0, 0, 2000);

    /* EPWM period point trigger ADC enable */
    EPWM_EnableADCTrigger(EPWM0, 0, EPWM_TRG_ADC_EVEN_PERIOD);

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    EPWM_SET_OUTPUT_LEVEL(EPWM0, BIT0, EPWM_OUTPUT_HIGH, EPWM_OUTPUT_LOW, EPWM_OUTPUT_NOTHING, EPWM_OUTPUT_NOTHING);

    /* Enable output of EPWM0 channel 0 */
    EPWM_EnableOutput(EPWM0, BIT0);
}


void LPADC_FunctionTest()
{
    uint8_t  u8Option;
    uint8_t  u8Index = 0;
    uint32_t u32COVNUMFlag = 0;
    int32_t  ai32ConversionData[6] = {0};

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      LPADC trigger by EPWM test                       |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel.\n");

    /* LPADC Calibration */
    LPADC_Calibration(LPADC0);

    while (1)
    {
        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 1 only)\n");
        printf("  [2] Differential input (channel pair 0 only)\n");
        printf("  Other keys: exit single mode test\n");
        u8Option = getchar();

        if (u8Option == '1')
        {
            /* Set input mode as single-end, Single mode, and select channel 0 */
            LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_SINGLE_END, LPADC_ADCR_ADMD_SINGLE, BIT1);

            /* Configure the sample module and enable EPWM0 trigger source */
            LPADC_EnableHWTrigger(LPADC0, LPADC_EPWM_TRIGGER, 0);

            /* Clear the A/D interrupt flag for safe */
            LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT);

            /* Enable the sample module interrupt */
            LPADC_EnableInt(LPADC0, LPADC_ADF_INT);
            NVIC_EnableIRQ(LPADC0_IRQn);

            printf("Conversion result of channel 1:\n");

            /* Reset the LPADC indicator and enable EPWM0 channel 0 counter */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;

            /* Enable EPWM0 channel 0 counter */
            EPWM_Start(EPWM0, BIT0); /* EPWM0 channel 0 counter start running. */

            while (1)
            {
                /* Wait LPADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
                while (g_u32AdcIntFlag == 0);

                /* Reset the LPADC interrupt indicator */
                g_u32AdcIntFlag = 0;

                /* Get the conversion result of the LPADC channel 1 */
                u32COVNUMFlag = g_u32COVNUMFlag - 1;
                ai32ConversionData[u32COVNUMFlag] = LPADC_GET_CONVERSION_DATA(LPADC0, 1);

                if (g_u32COVNUMFlag >= 6)
                    break;
            }

            /* Disable EPWM0 channel 0 counter */
            EPWM_ForceStop(EPWM0, BIT0); /* EPWM0 counter stop running. */

            for (u8Index = 0; (u8Index) < 6; u8Index++)
                printf("                                0x%X (%d)\n", ai32ConversionData[u8Index], ai32ConversionData[u8Index]);
        }
        else if (u8Option == '2')
        {
            /* Set input mode as differential, Single mode, and select channel 0 */
            LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_DIFFERENTIAL, LPADC_ADCR_ADMD_SINGLE, BIT0);

            /* Configure the sample module and enable EPWM0 trigger source */
            LPADC_EnableHWTrigger(LPADC0, LPADC_EPWM_TRIGGER, 0);

            /* Clear the A/D interrupt flag for safe */
            LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT);

            /* Enable the sample module interrupt */
            LPADC_EnableInt(LPADC0, LPADC_ADF_INT);
            NVIC_EnableIRQ(LPADC0_IRQn);

            printf("Conversion result of channel 0:\n");

            /* Reset the LPADC indicator and enable EPWM0 channel 0 counter */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;

            /* Enable EPWM0 channel 0 counter */
            EPWM_Start(EPWM0, BIT0); /* EPWM0 channel 0 counter start running. */

            while (1)
            {
                /* Wait LPADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
                while (g_u32AdcIntFlag == 0);

                /* Reset the LPADC interrupt indicator */
                g_u32AdcIntFlag = 0;

                /* Get the conversion result of the LPADC channel 2 */
                u32COVNUMFlag = g_u32COVNUMFlag - 1;
                ai32ConversionData[u32COVNUMFlag] = LPADC_GET_CONVERSION_DATA(LPADC0, 0);

                if (g_u32COVNUMFlag >= 6)
                    break;
            }

            /* Disable EPWM0 channel 0 counter */
            EPWM_ForceStop(EPWM0, BIT0); /* EPWM0 counter stop running. */

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

    /* Init EPWM for LPADC */
    EPWM0_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* LPADC function test */
    LPADC_FunctionTest();

    /* Disable LPADC IP clock */
    CLK_DisableModuleClock(LPADC0_MODULE);

    /* Disable EPWM0 IP clock */
    CLK_DisableModuleClock(EPWM0_MODULE);

    /* Lock protected registers */
    SYS_LockReg();

    /* Disable External Interrupt */
    NVIC_DisableIRQ(LPADC0_IRQn);

    printf("Exit LPADC sample code\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
