/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to trigger LPADC by EPWM and transfer conversion data by LPPDMA.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32IsTestOver = 0;

/* M55M1: Because LPPDMA only can access LPSRAM,
   the g_i32ConversionData[] MUST be allocated at LPSRAM area 0x20310000 ~ 0x20311FFF (8 KB).
 */
volatile uint32_t g_i32ConversionData[7] __attribute__((section(".lpSram")));

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* LPPDMA interrupt handler                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void LPPDMA_IRQHandler(void)
{
    uint32_t status = LPPDMA_GET_INT_STATUS(LPPDMA);

    if (status & LPPDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        if (LPPDMA_GET_ABORT_STS(LPPDMA) & LPPDMA_ABTSTS_ABTIF1_Msk)
            g_u32IsTestOver = 2;

        LPPDMA_CLR_ABORT_FLAG(LPPDMA, LPPDMA_ABTSTS_ABTIF1_Msk);
    }
    else if (status & LPPDMA_INTSTS_TDIF_Msk)     /* done */
    {
        if (LPPDMA_GET_TD_STS(LPPDMA) & LPPDMA_TDSTS_TDIF1_Msk)
        {
            /* Disable EPWM0 channel 0 counter */
            EPWM_ForceStop(EPWM0, BIT0); /* EPWM0 counter stop running. */
            g_u32IsTestOver = 1;
        }

        LPPDMA_CLR_TD_FLAG(LPPDMA, LPPDMA_TDSTS_TDIF1_Msk);
    }
    else
        printf("unknown LPPDMA interrupt !!\n");

    /*Confirm that the Flag has been cleared.*/
    LPPDMA_GET_INT_STATUS(LPPDMA);
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

    /* Enable LPPDMA clock source */
    CLK_EnableModuleClock(LPPDMA0_MODULE);

    /* Enable LPSRAM clock source */
    /* LPPDMA only can access LPSRAM and cannot access normal SRAM. */
    CLK_EnableModuleClock(LPSRAM0_MODULE);

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
    EPWM_SET_CMR(EPWM0, 0, 108);

    /* Set EPWM0 timer period */
    EPWM_SET_CNR(EPWM0, 0, 216);

    /* EPWM period point trigger ADC enable */
    EPWM_EnableADCTrigger(EPWM0, 0, EPWM_TRG_ADC_EVEN_PERIOD);

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    EPWM_SET_OUTPUT_LEVEL(EPWM0, BIT0, EPWM_OUTPUT_HIGH, EPWM_OUTPUT_LOW, EPWM_OUTPUT_NOTHING, EPWM_OUTPUT_NOTHING);

    /* Enable output of EPWM0 channel 0 */
    EPWM_EnableOutput(EPWM0, BIT0);
}

void ReloadLPPDMA()
{
    /* transfer width is half word(16 bit) and transfer count is 6 */
    LPPDMA_SetTransferCnt(LPPDMA, 1, LPPDMA_WIDTH_32, 6);

    /* Select PDMA request source as LPADC RX */
    LPPDMA_SetTransferMode(LPPDMA, 1, LPPDMA_LPADC0_RX, FALSE, 0);
}

void LPPDMA_Init()
{
    /* Configure LPPDMA peripheral mode form LPADC to memory */
    /* Open LPPDMA Channel 1 */
    LPPDMA_Open(LPPDMA, BIT1);

    ReloadLPPDMA();

    /* Set source address as LPADC LPPDMA Current Transfer Data register (no increment) and destination address as g_i32ConversionData array (increment) */
    LPPDMA_SetTransferAddr(LPPDMA, 1, (uint32_t) & (LPADC0->ADPDMA), LPPDMA_SAR_FIX, (uint32_t)g_i32ConversionData, LPPDMA_DAR_INC);

    LPPDMA_SetBurstType(LPPDMA, 1, LPPDMA_REQ_SINGLE, LPPDMA_BURST_1);

    LPPDMA_CLR_TD_FLAG(LPPDMA, LPPDMA_TDSTS_TDIF1_Msk);
    LPPDMA_EnableInt(LPPDMA, 1, LPPDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(LPPDMA_IRQn);
}

void LPADC_FunctionTest()
{
    uint8_t  u8Option;
    uint32_t i;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|   EPWM trigger mode and transfer LPADC conversion data by LPPDMA test |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel.\n");

    /* Calibration LPADC */
    LPADC_Calibration(LPADC0);

    while (1)
    {
        /* reload LPPDMA configuration for next transmission */
        ReloadLPPDMA();

        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 1 only)\n");
        printf("  [2] Differential input (channel pair 0 only (channel 0 and 1))\n");
        printf("  Other keys: exit single mode test\n");
        u8Option = getchar();

        if (u8Option == '1')
        {
            /* Set input mode as single-end, Single mode, and select channel 1 */
            LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_SINGLE_END, LPADC_ADCR_ADMD_SINGLE, BIT1);

            /* Configure the sample module and enable EPWM0 trigger source */
            LPADC_EnableHWTrigger(LPADC0, LPADC_EPWM_TRIGGER, 0);

            /* LPADC enable LPPDMA transfer */
            LPADC_ENABLE_LPPDMA(LPADC0);

            printf("Conversion result of channel 1:\n");

            /* Enable EPWM0 channel 0 counter */
            EPWM_Start(EPWM0, BIT0); /* EPWM0 channel 0 counter start running. */

            /* Wait LPPDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
            while (g_u32IsTestOver == 0);

            g_u32IsTestOver = 0;

            for (i = 0; (i) < 6; i++)
            {
                printf("                                0x%X,", g_i32ConversionData[i]);
                printf("(%d)\n", (g_i32ConversionData[i] & 0xFFF));
            }
        }
        else if (u8Option == '2')
        {
            /* Set input mode as differential, Single mode, and select channel 0 */
            LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_DIFFERENTIAL, LPADC_ADCR_ADMD_SINGLE, BIT0);

            /* Configure the sample module and enable EPWM0 trigger source */
            LPADC_EnableHWTrigger(LPADC0, LPADC_EPWM_TRIGGER, 0);

            /* LPADC enable LPPDMA transfer */
            LPADC_ENABLE_LPPDMA(LPADC0);

            printf("Conversion result of channel 0:\n");

            /* Enable EPWM0 channel 0 counter */
            EPWM_Start(EPWM0, BIT0); /* EPWM0 channel 0 counter start running. */

            /* Wait LPPDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
            while (g_u32IsTestOver == 0);

            g_u32IsTestOver = 0;

            /* Disable EPWM0 channel 0 counter */
            EPWM_ForceStop(EPWM0, BIT0); /* EPWM0 counter stop running. */

            for (i = 0; (i) < 6; i++)
            {
                printf("                                0x%X,", g_i32ConversionData[i]);
                printf("(%d)\n", (g_i32ConversionData[i] & 0xFFF));
            }
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

    /* Init LPPDMA for LPADC */
    LPPDMA_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* LPADC function test */
    LPADC_FunctionTest();

    /* Disable LPADC IP clock */
    CLK_DisableModuleClock(LPADC0_MODULE);

    /* Disable PWM0 IP clock */
    CLK_DisableModuleClock(EPWM0_MODULE);

    /* Disable LPPDMA clock source */
    CLK_DisableModuleClock(LPPDMA0_MODULE);

    /* Lock protected registers */
    SYS_LockReg();

    /* Disable LPPDMA Interrupt */
    NVIC_DisableIRQ(LPPDMA_IRQn);

    printf("Exit LPADC sample code\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
