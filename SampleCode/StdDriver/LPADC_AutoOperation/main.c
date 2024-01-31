/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to enable LPADC Auto-operation mdoe
 *           to convert when chip enters power-down mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*--------------------------------------------------------------------------*/
/* Define global variables and constants                                    */
/*--------------------------------------------------------------------------*/
volatile uint32_t g_u32LPPdmaIntFlag = 0;
volatile uint32_t g_u32WakeupCount = 0;

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/* M55M1: Because LPPDMA only can access LPSRAM,
   the g_i32ConversionData[] MUST be allocated at LPSRAM area 0x20310000 ~ 0x20311FFF (8 KB).
 */
volatile uint32_t g_i32ConversionData[10] __attribute__((section(".lpSram")));

/*---------------------------------------------------------------------------------------------------------*/
/* LPPDMA interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void LPPDMA_IRQHandler(void)
{
    uint32_t status = LPPDMA_GET_INT_STATUS(LPPDMA);

    if (status & LPPDMA_INTSTS_WKF_Msk)         /* wakeup */
    {
        LPPDMA->INTSTS = LPPDMA_INTSTS_WKF_Msk;
    }

    if (status & LPPDMA_INTSTS_ABTIF_Msk)       /* abort */
    {
        if (LPPDMA_GET_ABORT_STS(LPPDMA) & LPPDMA_ABTSTS_ABTIF1_Msk)
            g_u32LPPdmaIntFlag = 2;

        LPPDMA_CLR_ABORT_FLAG(LPPDMA, LPPDMA_ABTSTS_ABTIF1_Msk);
    }
    else if (status & LPPDMA_INTSTS_TDIF_Msk)   /* done */
    {
        if (LPPDMA_GET_TD_STS(LPPDMA) & LPPDMA_TDSTS_TDIF1_Msk)
            g_u32LPPdmaIntFlag = 1;

        LPPDMA_CLR_TD_FLAG(LPPDMA, LPPDMA_TDSTS_TDIF1_Msk);
    }
    else
        printf("LPPDMA_IRQHandler: unknown LPPDMA interrupt !!\n");

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

    /* Select LPTMR0 module clock source as HIRC */
    CLK_SetModuleClock(LPTMR0_MODULE, CLK_LPTMRSEL_LPTMR0SEL_HIRC, 0);

    /* Enable LPTMR0 module clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);

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

    /* Set PH multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();;

    /* Set PB.1 to input mode */
    GPIO_SetMode(PB, BIT1, GPIO_MODE_INPUT);
    /* Configure the PB.1 LPADC analog input pins.  */
    SET_LPADC0_CH1_PB1();
    /* Disable the PB.1 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT1);

    /* Set LPTMR0 PWM mode output pins */
    SET_LPTM0_PB5();

    /* Clock output HCLK to PC13 */
    SET_CLKO_PC13();

    CLK_EnableCKO(CLK_CLKOSEL_CLKOSEL_HIRC, 0, 1);

}

void LPTPWM_Init(void)
{
    uint32_t freq;

    LPTPWM_ENABLE_PWM_MODE(LPTMR0);
    LPTPWM_ENABLE_OUTPUT(LPTMR0, LPTPWM_CH0);

    /* Set LPTMR0 PWM output frequency is 1 Hz, duty 50% in up count type */
    freq = LPTPWM_ConfigOutputFreqAndDuty(LPTMR0, 1, 50);

    if (freq != 1)
    {
        printf("LPTPWM: Set the frequency %d different from the user !\n", freq);
    }

    /* LPTMR0 PWM mode trigger all Low Power IPs but LPPDMA */
    LPTPWM_EnableTrigger(LPTMR0, LPTMR_PWMTRGCTL_TRGEN_Msk, LPTPWM_TRIGGER_AT_PERIOD_POINT);

    /* Enable LPTMR clock in power-down mode */
    LPTMR_EnableWakeup(LPTMR0);
}

void LPPDMA_Init()
{
    uint32_t lppdma_ch = 1;

    /* Configure LPPDMA peripheral mode form LPADC to LPSRAM */
    /* Open Channel 1 */
    LPPDMA_Open(LPPDMA, BIT0 << lppdma_ch);

    /* Transfer count is 10, transfer width is 32 bits */
    LPPDMA_SetTransferCnt(LPPDMA, lppdma_ch, LPPDMA_WIDTH_32, 10);

    /* Set source address is LPADC0->ADPDMA, destination address is g_i32ConversionData[] on LPSRAM */
    LPPDMA_SetTransferAddr(LPPDMA, lppdma_ch, (uint32_t) & (LPADC0->ADPDMA), LPPDMA_SAR_FIX, (uint32_t)g_i32ConversionData, LPPDMA_DAR_INC);
    /* Request source is LPADC */
    LPPDMA_SetTransferMode(LPPDMA, lppdma_ch, LPPDMA_LPADC0_RX, FALSE, 0);

    /* Transfer type is burst transfer and burst size is 1 */
    LPPDMA_SetBurstType(LPPDMA, lppdma_ch, LPPDMA_REQ_SINGLE, LPPDMA_BURST_1);

    /* Enable interrupt */
    LPPDMA_CLR_TD_FLAG(LPPDMA, LPPDMA_TDSTS_TDIF0_Msk << lppdma_ch);
    LPPDMA_EnableInt(LPPDMA, lppdma_ch, LPPDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(LPPDMA_IRQn);
}

void LPADC_Init(void)
{
    /* LPADC Calibration */
    LPADC_Calibration(LPADC0);

    /* Set input mode as single-end, Single-cycle scan mode, and select channel 1 */
    LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_SINGLE_END, LPADC_ADCR_ADMD_SINGLE, BIT1);

    /* Enable LPADC Auto-operation and set it can be triggered by LPTMR0 */
    LPADC_SelectAutoOperationMode(LPADC0, LPADC_AUTOCTL_TRIGSEL_LPTMR0);

    /* Set LPADC to trigger LPPDMA */
    LPADC_ENABLE_LPPDMA(LPADC0);
}

void AutoOperation_FunctionTest()
{
    uint32_t i;

    g_u32WakeupCount = 0;

    SYS_UnlockReg();
    /* Switch SCLK clock source to HIRC and divide 1 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);
    SYS_LockReg();

    /* Start LPTMR counting */
    LPTPWM_START_COUNTER(LPTMR0);

    while (1)
    {
        LPPDMA_Init();

        printf("Power down and wait LPPDMA wake up CPU ...\n\n");
        UART_WAIT_TX_EMPTY(DEBUG_PORT);

        /* Clear wake-up status flag */
        PMC->INTSTS = PMC_INTSTS_CLRWK_Msk;
        g_u32LPPdmaIntFlag = 0;

        SYS_UnlockReg();

        PMC_SetPowerDownMode(PMC_NPD0, PMC_PLCTL_PLSEL_PL0);
        PMC_PowerDown();

        SYS_LockReg();

        printf("Wakeup %d times !!\n", ++g_u32WakeupCount);

        /* Waiting for LPPDMA transfer done. g_u32LPPdmaIntFlag is set by LPPDMA interrupt handler */
        while (g_u32LPPdmaIntFlag == 0);

        /* Check transfer result */
        if (g_u32LPPdmaIntFlag == 1)
            printf("LPPDMA trasnfer LPADC register done.\n");
        else if (g_u32LPPdmaIntFlag == 2)
            printf("LPPDMA trasnfer LPADC register abort...\n");

        printf("Only the first 10 values are valid LPADC conversion results.\n");

        for (i = 0; i < 10; i++)
        {
            printf("    [%2d]: %08X.\n", i, g_i32ConversionData[i]);
        }
    }   /* end of while(1) */
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

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nSystem clock rate: %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------+\n");
    printf("|   LPADC Auto-operation mode sample code   |\n");
    printf("+-------------------------------------------+\n");
    printf("    * Initial LPTMR0, LPPDMA, and LPADC to Auto-operation mode.\n");
    printf("    * CPU enter power-down mode.\n");
    printf("    * LPTMR0 PWM mode output 1 Hz 50%% duty to LPTM0 (PB5).\n");
    printf("          and trigger LPADC0 channel 1 (PB1) at period point.\n");
    printf("    * LPADC0 trigger LPPDMA to move data from LPADC0 to LPSRAM.\n");
    printf("    * LPPDMA wakeup CPU after 10 data be moved.\n");
    printf("    * CPU can access LPSRAM to get 10 LPADC conversion result.\n");
    printf("    *** Check the HCLK output on PC13 can monitor power-down period.\n");

    LPTPWM_Init();

    LPADC_Init();

    LPPDMA_Init();

    AutoOperation_FunctionTest();

    printf("Exit LPADC Auto-operation sample code\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
