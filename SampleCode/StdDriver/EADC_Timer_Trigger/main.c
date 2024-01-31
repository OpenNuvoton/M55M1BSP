/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to trigger EADC by Timer.
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
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);/* Clear the A/D ADINT0 interrupt flag */

    /*Confirm that the Flag has been cleared.*/
    M32(&EADC0->STATUS2);

    g_u32AdcIntFlag = 1;
    g_u32COVNUMFlag++;
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

    /* Workaround(TESTCHIP_ONLY)  */
    /*If the ADC clock is divided, the conversion result value will deviate, so only the PCLK0 clock can be divided. */
    /* PCLK0 clock divider 15 */
    CLK_SET_PCLK0DIV(15);
    /* Enable EADC peripheral clock */
    CLK_SetModuleClock(EADC0_MODULE, CLK_EADCSEL_EADC0SEL_PCLK0, CLK_EADCDIV_EADC0DIV(1));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable GPIOB module clock */
    CLK_EnableModuleClock(GPIOB_MODULE);

    /* Select timer 0 module clock source as HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL_TMR0SEL_HIRC, 0);

    /* Enable Timer 0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Set PB.1 to input mode */
    GPIO_SetMode(PB, BIT1, GPIO_MODE_INPUT);
    /* Configure the PB.1 ADC analog input pins. */
    SET_EADC0_CH1_PB1();
    /* Disable the PB.1 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT1);

#if !defined(ALIGN_AF_PINS)
    /* Set PB.14 - PB.15 to input mode */
    GPIO_SetMode(PB, BIT14 | BIT15, GPIO_MODE_INPUT);
    /* Configure the PB.14 - PB.15 ADC analog input pins. */
    SET_EADC0_CH14_PB14();
    SET_EADC0_CH15_PB15();
    /* Disable the PB.14 - PB.15 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT14 | BIT15);
#else
    /* Set PB.8 - PB.9 to input mode */
    GPIO_SetMode(PB, BIT8 | BIT9, GPIO_MODE_INPUT);
    /* Configure the PB.8 - PB.9 ADC analog input pins. */
    SET_EADC0_CH8_PB8();
    SET_EADC0_CH9_PB9();
    /* Disable the PB.8 - PB.9 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT8 | BIT9);
#endif

}

void TIMER0_Init()
{
    /* Set timer0 periodic time-out period is 1s if timer clock is 12 MHz */

    TIMER_SetTriggerTarget(TIMER0, TIMER_TRG_TO_EADC);
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1);
}

void EADC_FunctionTest()
{
    uint8_t  u8Option;
    uint32_t u32COVNUMFlag = 0;
    int32_t  i32ConversionData[6] = {0};

    printf("\n");
    printf("+-------------------------------------------+\n");
    printf("|           Timer trigger mode test         |\n");
    printf("+-------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel.\n");

    while (1)
    {
        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 1 only)\n");
#if !defined(ALIGN_AF_PINS)
        printf("  [2] Differential input (channel pair 7: channel 14 and 15)\n");
#else
        printf("  [2] Differential input (channel pair 4: channel 8 and 9)\n");
#endif
        printf("  Other keys: exit single mode test\n");
        u8Option = getchar();

        if (u8Option == '1')
        {
            /* Set input mode as single-end and enable the A/D converter */
            EADC_Open(EADC0, EADC_CTL_DIFFEN_SINGLE_END);

            /* Configure the sample module 0 for analog input channel 1 and enable Timer0 trigger source */
            EADC_ConfigSampleModule(EADC0, 0, EADC_TIMER0_TRIGGER, 1);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

            /* Enable the sample module 0 interrupt.  */
            EADC_ENABLE_INT(EADC0, BIT0);
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT0);
            NVIC_EnableIRQ(EADC00_IRQn);

            printf("Conversion result of channel 1:\n");

            /* Reset the ADC indicator and enable Timer0 counter */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            TIMER_Start(TIMER0);

            while (1)
            {
                /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
                while (g_u32AdcIntFlag == 0);

                /* Reset the EADC interrupt indicator */
                g_u32AdcIntFlag = 0;


                /* Get the conversion result of the sample module 0 */
                u32COVNUMFlag = g_u32COVNUMFlag - 1;
                i32ConversionData[u32COVNUMFlag] = EADC_GET_CONV_DATA(EADC0, 0);
                printf("    0x%X (%d)\n", i32ConversionData[u32COVNUMFlag], i32ConversionData[u32COVNUMFlag]);

                if (g_u32COVNUMFlag > 5)
                    break;
            }

            /* Disable Timer0 counter */
            TIMER_Stop(TIMER0);

            /* Disable the ADINT0 interrupt */
            EADC_DISABLE_INT(EADC0, BIT0);
        }
        else if (u8Option == '2')
        {
            /* Set input mode as differential and enable the A/D converter */
            EADC_Open(EADC0, EADC_CTL_DIFFEN_DIFFERENTIAL);

#if !defined(ALIGN_AF_PINS)
            /* Configure the sample module 0 for analog input channel 14 and enable Timer0 trigger source */
            EADC_ConfigSampleModule(EADC0, 0, EADC_TIMER0_TRIGGER, 14);
#else
            /* Configure the sample module 0 for analog input channel 8 and enable Timer0 trigger source */
            EADC_ConfigSampleModule(EADC0, 0, EADC_TIMER0_TRIGGER, 8);
#endif

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

            /* Enable the sample module 0 interrupt.  */
            EADC_ENABLE_INT(EADC0, BIT0);
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT0);
            NVIC_EnableIRQ(EADC00_IRQn);

#if !defined(ALIGN_AF_PINS)
            printf("Conversion result of channel pair 7 (channel 14/15):\n");
#else
            printf("Conversion result of channel pair 4 (channel 8/9):\n");
#endif

            /* Reset the EADC indicator and enable Timer0 counter */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            TIMER_Start(TIMER0);

            while (1)
            {
                /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
                while (g_u32AdcIntFlag == 0);

                /* Reset the ADC interrupt indicator */
                g_u32AdcIntFlag = 0;

                /* Get the conversion result of the sample module 0 */
                u32COVNUMFlag = g_u32COVNUMFlag - 1;
                i32ConversionData[u32COVNUMFlag] = EADC_GET_CONV_DATA(EADC0, 0);
                printf("    0x%X (%d)\n", i32ConversionData[u32COVNUMFlag], i32ConversionData[u32COVNUMFlag]);

                if (g_u32COVNUMFlag > 5)
                    break;
            }

            /* Disable Timer0 counter */
            TIMER_Stop(TIMER0);

            /* Disable the ADINT0 interrupt */
            EADC_DISABLE_INT(EADC0, BIT0);
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

    /* Init Debug UART for printf */
    InitDebugUart();

    /* Init TIMER0 for EADC */
    TIMER0_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable Timer0 IP clock */
    CLK_DisableModuleClock(TMR0_MODULE);

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
