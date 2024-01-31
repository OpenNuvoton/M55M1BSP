/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to trigger multiple sample modules and got conversion results in order of priority.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------------------------------*/
/* Define global variables and constants                                */
/*----------------------------------------------------------------------*/
volatile uint32_t g_u32EadcInt0Flag, g_u32EadcInt1Flag,
         g_u32EadcInt2Flag, g_u32EadcInt3Flag;

uint32_t g_u32IntModule[4];    /* save the sample module number for ADINT0~3 */
volatile uint32_t g_u32IntSequence[4];  /* save the interrupt sequence for ADINT0~3 */
volatile uint32_t g_u32IntSequenceIndex;

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void EADC00_IRQHandler(void)
{
    g_u32EadcInt0Flag = 1;
    /* Clear the A/D ADINT0 interrupt flag */
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

    /* Save the interrupt sequence about ADINT0 */
    g_u32IntSequence[0] = g_u32IntSequenceIndex++;

    /*Confirm that the Flag has been cleared. */
    M32(&EADC0->STATUS2);
}

NVT_ITCM void EADC01_IRQHandler(void)
{
    g_u32EadcInt1Flag = 1;
    /* Clear the A/D ADINT1 interrupt flag */
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF1_Msk);

    /* Save the interrupt sequence about ADINT1 */
    g_u32IntSequence[1] = g_u32IntSequenceIndex++;

    /*Confirm that the Flag has been cleared. */
    M32(&EADC0->STATUS2);
}

NVT_ITCM void EADC02_IRQHandler(void)
{
    g_u32EadcInt2Flag = 1;
    /* Clear the A/D ADINT2 interrupt flag */
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF2_Msk);

    /* Save the interrupt sequence about ADINT2 */
    g_u32IntSequence[2] = g_u32IntSequenceIndex++;

    /*Confirm that the Flag has been cleared. */
    M32(&EADC0->STATUS2);
}

NVT_ITCM void EADC03_IRQHandler(void)
{
    g_u32EadcInt3Flag = 1;
    /* Clear the A/D ADINT3 interrupt flag */
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF3_Msk);

    /* Save the interrupt sequence about ADINT3 */
    g_u32IntSequence[3] = g_u32IntSequenceIndex++;

    /*Confirm that the Flag has been cleared. */
    M32(&EADC0->STATUS2);
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

    /* Workaround(Test Chip Only)  */
    /* If the ADC clock is divided, the conversion result value will deviate, so only the PCLK0 clock can be divided. */
    /* PCLK0 clock divider 15 */
    CLK_SET_PCLK0DIV(15);
    /* Enable EADC peripheral clock */
    CLK_SetModuleClock(EADC0_MODULE, CLK_EADCSEL_EADC0SEL_PCLK0, CLK_EADCDIV_EADC0DIV(1));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC0_MODULE);

    /* Enable GPIOB module clock */
    CLK_EnableModuleClock(GPIOB_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Debug UART clock setting*/
    SetDebugUartCLK();
    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Set PB.0, PB.6 ~ PB.9 to input mode */
    GPIO_SetMode(PB, BIT9 | BIT8 | BIT7 | BIT6, GPIO_MODE_INPUT);
    /* Configure the EADC analog input pins.  */
    SET_EADC0_CH6_PB6();
    SET_EADC0_CH7_PB7();
    SET_EADC0_CH8_PB8();
    SET_EADC0_CH9_PB9();
    /* Disable the GPB0, GPB6~9 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT9 | BIT8 | BIT7 | BIT6);

}

void EADC_FunctionTest()
{
    uint8_t  u8Option;
    int32_t  i32ConversionData, i;

    printf("\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|               EADC Pending Priority sample code               |\n");
    printf("+---------------------------------------------------------------+\n");

    /* Set the EADC and enable the A/D converter */
    EADC_Open(EADC0, (uint32_t)NULL);

    while (1)
    {
        printf("Select test items:\n");
        printf("  [1] Assign interrupt ADINT0~3 to Sample Module 0~3\n");
        printf("  [2] Assign interrupt ADINT3~0 to Sample Module 0~3\n");
        printf("  Other keys: exit EADC test\n");
        u8Option = getchar();

        if (u8Option == '1')
        {
            g_u32IntModule[0] = 0;  /* Assign ADINT0 to Sample module 0 */
            g_u32IntModule[1] = 1;  /* Assign ADINT1 to Sample module 1 */
            g_u32IntModule[2] = 2;  /* Assign ADINT2 to Sample module 2 */
            g_u32IntModule[3] = 3;  /* Assign ADINT3 to Sample module 3 */
        }
        else if (u8Option == '2')
        {
            g_u32IntModule[0] = 3;  /* Assign ADINT0 to Sample module 3 */
            g_u32IntModule[1] = 2;  /* Assign ADINT1 to Sample module 2 */
            g_u32IntModule[2] = 1;  /* Assign ADINT2 to Sample module 1 */
            g_u32IntModule[3] = 0;  /* Assign ADINT3 to Sample module 0 */
        }
        else
            break;  /* exit while loop */

        /* Configure the sample module for analog input channel and software trigger source. */
        EADC_ConfigSampleModule(EADC0, 0, EADC_SOFTWARE_TRIGGER, 6);
        EADC_ConfigSampleModule(EADC0, 1, EADC_SOFTWARE_TRIGGER, 7);
        EADC_ConfigSampleModule(EADC0, 2, EADC_SOFTWARE_TRIGGER, 8);
        EADC_ConfigSampleModule(EADC0, 3, EADC_SOFTWARE_TRIGGER, 9);

        /* Clear the A/D ADINTx interrupt flag for safe */
        EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk | EADC_STATUS2_ADIF1_Msk | EADC_STATUS2_ADIF2_Msk | EADC_STATUS2_ADIF3_Msk);

        /* Enable the sample module interrupt.  */
        EADC_ENABLE_INT(EADC0, BIT0 | BIT1 | BIT2 | BIT3);

        EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT0 << g_u32IntModule[0]);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 1, BIT0 << g_u32IntModule[1]);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 2, BIT0 << g_u32IntModule[2]);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 3, BIT0 << g_u32IntModule[3]);

        NVIC_EnableIRQ(EADC00_IRQn);
        NVIC_EnableIRQ(EADC01_IRQn);
        NVIC_EnableIRQ(EADC02_IRQn);
        NVIC_EnableIRQ(EADC03_IRQn);

        /* Reset the EADC interrupt indicator and trigger sample module to start A/D conversion */
        g_u32IntSequenceIndex = 0;
        g_u32EadcInt0Flag = 0;
        g_u32EadcInt1Flag = 0;
        g_u32EadcInt2Flag = 0;
        g_u32EadcInt3Flag = 0;

        /* Start EADC conversion for sample module 0 ~ 3 at the same time */
        EADC_START_CONV(EADC0, BIT0 | BIT1 | BIT2 | BIT3);

        /* Wait all EADC interrupt (g_u32EadcIntxFlag will be set at EADC_INTx_IRQHandler() function) */
        while ((g_u32EadcInt0Flag == 0) || (g_u32EadcInt1Flag == 0) ||
                (g_u32EadcInt2Flag == 0) || (g_u32EadcInt3Flag == 0));

        /* Get the conversion result of the sample module */
        printf("The ADINTx interrupt sequence is:\n");

        for (i = 0; i < 4; i++)
        {
            i32ConversionData = EADC_GET_CONV_DATA(EADC0, g_u32IntModule[i]);
            printf("ADINT%d: #%d, Module %d, Conversion result for channel %d: 0x%X (%d)\n", i, g_u32IntSequence[i], g_u32IntModule[i], g_u32IntModule[i] + 6, i32ConversionData, i32ConversionData);
        }

        printf("\n");

        /* Disable the ADINTx interrupt */
        EADC_DISABLE_INT(EADC0, BIT0 | BIT1 | BIT2 | BIT3);

        EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT0 << g_u32IntModule[0]);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, 1, BIT0 << g_u32IntModule[1]);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, 2, BIT0 << g_u32IntModule[2]);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, 3, BIT0 << g_u32IntModule[3]);

        NVIC_DisableIRQ(EADC00_IRQn);
        NVIC_DisableIRQ(EADC01_IRQn);
        NVIC_DisableIRQ(EADC02_IRQn);
        NVIC_DisableIRQ(EADC03_IRQn);
    }   /* End of while(1) */

    /* Disable the A/D converter */
    EADC_Close(EADC0);
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

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC0_MODULE);

    /* Reset EADC module */
    SYS_ResetModule(SYS_EADC0RST);
    /* Lock protected registers */
    SYS_LockReg();
    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC00_IRQn);
    NVIC_DisableIRQ(EADC01_IRQn);
    NVIC_DisableIRQ(EADC02_IRQn);
    NVIC_DisableIRQ(EADC03_IRQn);

    printf("Exit EADC sample code\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
