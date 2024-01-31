/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Convert VBAT/4 (Sample module 26) and print conversion result.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t g_u32AdcIntFlag;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void EADC_FunctionTest(void);
void SYS_Init(void);
NVT_ITCM void EADC00_IRQHandler(void);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void EADC00_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    /* Clear the A/D ADINT0 interrupt flag */
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);
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

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Set debug uart multi-function pins. */
    SetDebugUartMFP();

    /* Enable VBAT unity gain buffer */
    SYS->IVSCTL |= SYS_IVSCTL_VBATUGEN_Msk;
}


/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void)
{
    int32_t  i32ConversionData;
    uint32_t u32TimeOutCnt;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      VBAT/4 test                                     |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC0, EADC_CTL_DIFFEN_SINGLE_END);

    /* Set sample module 26 external sampling time to 0xFF */
    EADC_SetExtendSampleTime(EADC0, 26, 0xFF);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

    /* Enable sample module A/D ADINT0 interrupt. */
    EADC_ENABLE_INT(EADC0, BIT0);
    /* Enable sample module 26 interrupt. */
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT26);
    NVIC_EnableIRQ(EADC00_IRQn);

    /* Reset the ADC interrupt indicator and trigger sample module 26 to start A/D conversion */
    g_u32AdcIntFlag = 0;
    EADC_START_CONV(EADC0, BIT26);

    /* Wait EADC conversion done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (g_u32AdcIntFlag == 0)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for EADC conversion done time-out!\n");
            return;
        }
    }

    /* Disable the ADINT0 interrupt */
    EADC_DISABLE_INT(EADC0, BIT0);

    /* Get the conversion result of the sample module 26 */
    i32ConversionData = EADC_GET_CONV_DATA(EADC0, 26);
    printf("Conversion result of VBAT/4: 0x%X (%d)\n\n", i32ConversionData, i32ConversionData);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Reset EADC module */
    SYS_ResetModule(SYS_EADC0RST);

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC0_MODULE);
    /* Lock protected registers */
    SYS_LockReg();
    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC00_IRQn);

    printf("Exit EADC sample code\n");

    while (1);

}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
