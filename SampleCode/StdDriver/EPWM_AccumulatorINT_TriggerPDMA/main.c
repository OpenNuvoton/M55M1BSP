/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate EPWM accumulator interrupt trigger PDMA.
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

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static uint16_t g_au16Period[2] = {31999, 15999};
static volatile uint32_t g_u32IsTestOver = 0;

/**
 * @brief       PDMA IRQ Handler
 * @param       None
 * @return      None
 * @details     ISR to handle PDMA interrupt event
 */
NVT_ITCM void PDMA0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if (u32Status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        if (PDMA_GET_ABORT_STS(PDMA0) & PDMA_ABTSTS_ABTIF0_Msk)
            g_u32IsTestOver = 2;

        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF0_Msk);
    }
    else if (u32Status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        if (PDMA_GET_TD_STS(PDMA0) & PDMA_TDSTS_TDIF0_Msk)
            g_u32IsTestOver = 1;

        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF0_Msk);
    }
    else
        printf("unknown interrupt %x !!\n", u32Status);

    __DSB();
    __ISB();

    while (PDMA_GET_INT_STATUS(PDMA0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA IntFlag time-out!\n");
        }
    }
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 180MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ);

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
    /* Select EPWM1 module clock source */
    CLK_SetModuleClock(EPWM1_MODULE, CLK_EPWMSEL_EPWM1SEL_PCLK2, 0);

    /* Enable EPWM1 module clock */
    CLK_EnableModuleClock(EPWM1_MODULE);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPIOC_MODULE);
    /* Set multi-function pin for EPWM */
    SET_EPWM1_CH0_PC5();
    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint32_t u32NewCNR = 0;
    uint32_t u32TimeOutCnt = 0;
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|              EPWM AccumulatorINT TriggerPDMA Sample Code               |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code demonstrate EPWM1 channel 0 accumulator interrupt trigger PDMA.\n");
    printf("  When accumulator interrupt happens, EPWM1 channel 0 period register will be updated \n");
    printf("  to g_u32Count array content, 31999(0x7CFF), by PDMA.\n");

    printf("\n\nPress any key to start EPWM1 channel 0.\n");
    getchar();

    /*--------------------------------------------------------------------------------------*/
    /* Set the EPWM1 Channel 0 as EPWM output function.                                     */
    /*--------------------------------------------------------------------------------------*/

    /* Set EPWM1 channel 0 output configuration */
    EPWM_ConfigOutputChannel(EPWM1, 0, 300, 30);

    /* Enable EPWM Output path for EPWM1 channel 0 */
    EPWM_EnableOutput(EPWM1, EPWM_CH_0_MASK);

    /* Enable EPWM1 channel 0 accumulator, interrupt count 50, accumulator source select to zero point */
    EPWM_EnableAcc(EPWM1, 0, 50, EPWM_IFA_ZERO_POINT);

    /* Enable EPWM1 channel 0 accumulator interrupt */
    EPWM_EnableAccInt(EPWM1, 0);

    /* Enable EPWM1 channel 0 accumulator interrupt trigger PDMA */
    EPWM_EnableAccPDMA(EPWM1, 0);

    /* Enable Timer for EPWM1 channel 0 */
    EPWM_Start(EPWM1, EPWM_CH_0_MASK);

    /*--------------------------------------------------------------------------------------*/
    /* Configure PDMA peripheral mode form memory to EPWM                                   */
    /*--------------------------------------------------------------------------------------*/
    /* Open Channel 0 */
    PDMA_Open(PDMA0, BIT0);

    /* Transfer width is half word(16 bit) and transfer count is 1 */
    PDMA_SetTransferCnt(PDMA0, 0, PDMA_WIDTH_16, 1);

    /* Set source address as g_u32Count array(increment) and destination address as EPWM1 channel 0 period register(no increment) */
    PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)&g_au16Period[0], PDMA_SAR_INC, (uint32_t) & (EPWM1->PERIOD[0]), PDMA_DAR_FIX);

    /* Select PDMA request source as EPWM1_CH0_TX(EPWM1 channel 0 accumulator interrupt) */
    PDMA_SetTransferMode(PDMA0, 0, PDMA_EPWM1_CH0_TX, FALSE, 0);

    /* Set PDMA as single request type for EPWM */
    PDMA_SetBurstType(PDMA0, 0, PDMA_REQ_SINGLE, PDMA_BURST_1);

    PDMA_EnableInt(PDMA0, 0, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);

    g_u32IsTestOver = 0;

    /* Wait for PDMA transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (g_u32IsTestOver != 1)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA transfer done time-out!\n");
            goto lexit;
        }
    }

    u32NewCNR = EPWM_GET_CNR(EPWM1, 0);
    printf("\n\nEPWM1 channel0 period register is updated to %d(0x%x)\n", u32NewCNR, u32NewCNR);
    printf("Press any key to stop EPWM1 channel 0.\n");
    getchar();

    /* Set EPWM1 channel 0 loaded value as 0 */
    EPWM_Stop(EPWM1, EPWM_CH_0_MASK);

    /* Wait until EPWM1 channel 0 Timer Stop */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while ((EPWM1->CNT[0] & EPWM_CNT0_CNT_Msk) != 0)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for EPWM stop time-out!\n");
            break;
        }
    }

lexit:

    /* Disable Timer for EPWM1 channel 0 */
    EPWM_ForceStop(EPWM1, EPWM_CH_0_MASK);

    /* Disable EPWM Output path for EPWM1 channel 0 */
    EPWM_DisableOutput(EPWM1, EPWM_CH_0_MASK);

    /* Disable PDMA NVIC */
    NVIC_DisableIRQ(PDMA0_IRQn);

    /* Close PDMA */
    PDMA_Close(PDMA0);

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
