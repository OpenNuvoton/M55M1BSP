/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate LPTPWM Auto-operation with LPPDMA when chip enters power-down mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
/*
 * This sample uses internal RC as APLL0 clock source and UART to print messages.
 * Users may need to do extra system configuration according to their system design.
 *
 * Debug UART
 *   system_M55M1.c has three weak functions as below to configure debug UART port.
 *     SetDebugUartMFP, SetDebugUartCLK and InitDebugUart
 *   Users can re-implement these functions according to system design.
 */
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#define LPTMR_LPPDMA_CH      1
#define DATA_COUNT           10

#if (NVT_DCACHE_ON == 1)
    static uint32_t s_au32CAPValue[DCACHE_ALIGN_LINE_SIZE(DATA_COUNT)] __attribute__((aligned(DCACHE_LINE_SIZE), section(".lpSram")));
#else
    static uint32_t s_au32CAPValue[DATA_COUNT] __attribute__((section(".lpSram")));
#endif

static volatile uint32_t s_u32IsTestOver = 0;

NVT_ITCM void LPPDMA_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    uint32_t status = LPPDMA_GET_INT_STATUS(LPPDMA);

    if (status & LPPDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        /* Check if channel 1 has abort error */
        if (LPPDMA_GET_ABORT_STS(LPPDMA) & (LPPDMA_ABTSTS_ABTIF0_Msk << LPTMR_LPPDMA_CH))
            s_u32IsTestOver = 2;

        /* Clear abort flag of channel 1 */
        LPPDMA_CLR_ABORT_FLAG(LPPDMA, (LPPDMA_ABTSTS_ABTIF0_Msk << LPTMR_LPPDMA_CH));
    }
    else if (status & LPPDMA_INTSTS_TDIF_Msk)     /* done */
    {
        /* Check transmission of channel 1 has been transfer done */
        if (LPPDMA_GET_TD_STS(LPPDMA) & (LPPDMA_TDSTS_TDIF0_Msk << LPTMR_LPPDMA_CH))
            s_u32IsTestOver = 1;

        /* Clear transfer done flag of channel 1 */
        LPPDMA_CLR_TD_FLAG(LPPDMA, (LPPDMA_TDSTS_TDIF0_Msk << LPTMR_LPPDMA_CH));
    }
    else
        printf("unknown interrupt %x!!\n", status);

    if (status & LPPDMA_INTSTS_WKF_Msk)     /* wake up */
    {
        printf("wake up !!\n");

        /* Clear wake up flag */
        LPPDMA->INTSTS = status;
    }

    __DSB();
    __ISB();

    while (LPPDMA_GET_INT_STATUS(LPPDMA))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for LPPDMA IntFlag time-out!\n");
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  LPTMR Function                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void LPTMR_PWM_Trigger_Init(void)
{
    /* Change Low Power Timer to PWM counter mode */
    LPTPWM_ENABLE_PWM_MODE(LPTMR0);

    /* Set Low Power TMR0 PWM output frequency is 1000 Hz, duty 50% in up count type */
    if (LPTPWM_ConfigOutputFreqAndDuty(LPTMR0, 1000, 50) != 1000)
    {
        printf("Set the frequency different from the user\n");
    }

    /* Enable output of LPTPWM_CH0 */
    LPTPWM_ENABLE_OUTPUT(LPTMR0, TPWM_CH0);
    /* Enable LPTPWM0 to trigger LPPDMA */
    LPTPWM_EnableTrigger(LPTMR0, LPTPWM_TRG_TO_LPPDMA, LPTPWM_TRIGGER_AT_PERIOD_POINT);
    /* Enable LPTMR Power-down engine clock */
    LPTMR_EnablePDCLK(LPTMR0);
}

void LPPDMA_Init(void)
{
    /* Reset PDMA module */
    SYS_ResetModule(SYS_LPPDMA0RST);

#if (NVT_DCACHE_ON == 1)
    /*
        Clean the CPU Data cache before starting the DMA transfer.
        This guarantees that the source buffer will be up to date before starting the transfer.
    */
    SCB_CleanDCache_by_Addr(s_au32CAPValue, sizeof(s_au32CAPValue));
#endif  // (NVT_DCACHE_ON == 1)

    /* Enable PDMA channels */
    LPPDMA_Open(LPPDMA, 1 << LPTMR_LPPDMA_CH);

    /*=======================================================================
      LPPDMA channel configuration:
      -----------------------------------------------------------------------
        Word length = 32 bits
        Transfer Count = DATA_COUNT
        Source = s_au32CAPValue
        Source Address = Fixed
        Destination = &LPTMR0->PWMCMPDAT
        Destination Address = Fixed
        Burst Type = Single Transfer
    =========================================================================*/
    /* Set transfer width (32 bits) and transfer count */
    LPPDMA_SetTransferCnt(LPPDMA, LPTMR_LPPDMA_CH, LPPDMA_WIDTH_32, DATA_COUNT);
    /* Set source/destination address and attributes */
    LPPDMA_SetTransferAddr(LPPDMA, LPTMR_LPPDMA_CH, (uint32_t)s_au32CAPValue, LPPDMA_SAR_FIX, (uint32_t)&LPTMR0->PWMCMPDAT, LPPDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    LPPDMA_SetTransferMode(LPPDMA, LPTMR_LPPDMA_CH, LPPDMA_LPTMR0, FALSE, 0);
    /* Single request type. LPTMR only support LPPDMA single request type. */
    LPPDMA_SetBurstType(LPPDMA, LPTMR_LPPDMA_CH, LPPDMA_REQ_SINGLE, 0);
    /* Enable Channel Transfer done interrupt */
    LPPDMA_EnableInt(LPPDMA, LPTMR_LPPDMA_CH, LPPDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(LPPDMA_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    SYS_UnlockReg();
    /* Switch SCLK to HIRC when power down */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_HIRC, CLK_APLLCTL_APLLSRC_HIRC, 0);
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(DEBUG_PORT);
    //MIRC and HIRC enable in power down mode
    PMC_DISABLE_AOCKPD();
    /* Set Power-down mode */
    PMC_SetPowerDownMode(PMC_NPD0, PMC_PLCTL_PLSEL_PL1);
    /* Enter to Power-down mode */
    PMC_PowerDown();
    /* Enable PLL0 220MHZ clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);
    SYS_LockReg();
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 220MHZ clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* enable LPSRAM clock */
    CLK_EnableModuleClock(LPSRAM0_MODULE);
    /* Enable LPPDMA Clock */
    CLK_EnableModuleClock(LPPDMA0_MODULE);
    /* Select LPTMR0 module clock source from HIRC */
    CLK_SetModuleClock(LPTMR0_MODULE, CLK_LPTMRSEL_LPTMR0SEL_HIRC, MODULE_NoMsk);
    /* Enable LPTMR 0 module clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);
    /* Set LPTMR0 PWM CH0(LPTM0) pin */
    CLK_EnableModuleClock(GPIOB_MODULE);
    SET_LPTM0_PB5();
    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("System core clock = %d\n", SystemCoreClock);
    printf("+-------------------------------------------------------------+\n");
    printf("|         LPTPWM Auto Operation Mode Sample Code              |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("\n");
    printf(" 1. Initialize LPTPWM0 and LPPDMA. \n");
    printf(" 2. Let system enter Powerr Mode. \n");
    printf(" 3. LPTPWM0 will trigger LPPDMA to do Set PWMCMPDAT data transfer. After finishing data transfer, \n");
    printf("    system will be woken up and the set data can be checked. \n");

    LPTMR_PWM_Trigger_Init();
    s_u32IsTestOver = 0;
    s_au32CAPValue[0] = 30;
    LPPDMA_Init();

    /* Start Low Power Timer PWM counter */
    LPTPWM_EnableCounter(LPTMR0);
    PowerDownFunction();
    NVIC_DisableIRQ(LPPDMA_IRQn);
    LPPDMA_DisableInt(LPPDMA, LPTMR_LPPDMA_CH, LPPDMA_INT_TRANS_DONE);
    LPTPWM_DisableCounter(LPTMR0);

    /* Waiting for LPPDMA transfer done.  g_u32IsTestOver is set by LPPDMA interrupt handler */
    while (s_u32IsTestOver == 0) {};

#if (NVT_DCACHE_ON == 1)
    /*
       Invalidate the CPU Data cache after the DMA transfer.
       As the destination buffer may be used by the CPU, this guarantees up-to-date data when CPU access
    */
    SCB_InvalidateDCache_by_Addr(s_au32CAPValue, sizeof(s_au32CAPValue));

#endif  // (NVT_DCACHE_ON == 1)

    /* Check transfer result */
    if (s_u32IsTestOver == 1)
        printf("LPPDMA trasnfer LPTPWM done.\n");
    else if (s_u32IsTestOver == 2)
        printf("LPPDMA trasnfer LPTPWM abort...\n");

    if (s_au32CAPValue[0] == LPTMR0->PWMCMPDAT)
        printf("LPTPWM Auto Operation PASS.\n");
    else
        printf("LPTPWM Auto Operation FAIL.\n");

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
