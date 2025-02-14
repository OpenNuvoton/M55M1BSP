/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to use EPWM brake function.
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

/**
 * @brief       EPWM0 Brake0 IRQ Handler
 * @param       None
 * @return      None
 * @details     ISR to handle EPWM0 Brake0 interrupt event
 */
NVT_ITCM void BRAKE0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    printf("\nFault brake!\n");
    printf("Press any key to unlock brake state. (EPWM0 channel 0 output will toggle again)\n");
    getchar();

    SYS_UnlockReg();
    /* Enable brake and interrupt */
    (EPWM0)->BRKCTL[0] = 0;
    EPWM_DisableFaultBrakeInt(EPWM0, EPWM_FB_EDGE);
    /* Clear brake interrupt flag */
    EPWM_ClearFaultBrakeIntFlag(EPWM0, EPWM_FB_EDGE);
    __DSB();
    __ISB();

    while (EPWM_GetFaultBrakeIntFlag(EPWM0, EPWM_FB_EDGE))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for EPWM0 IntFlag time-out!\n");
        }
    }

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
    /* Select EPWM1 module clock source */
    CLK_SetModuleClock(EPWM0_MODULE, CLK_EPWMSEL_EPWM0SEL_PCLK0, 0);

    /* Enable EPWM1 module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(GPIOD_MODULE);
    /* Set multi-function pin for EPWM */
    SET_EPWM0_CH0_PE7();

    /* Set multi-function pin for EPWM brake pin */
    SET_EPWM0_BRAKE0_PE8();
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
    printf("\nConnet PE.8 (EPWM0 brake pin 0) to PD.5.\n");
    printf("It will generate brake interrupt and EPWM0 channel 0 output PE.7 stop toggling.\n");

    GPIO_SetMode(PD, BIT5, GPIO_MODE_OUTPUT);
    PD5 = 0;

    /* EPWM0 frequency is 100Hz, duty 30%, */
    EPWM_ConfigOutputChannel(EPWM0, 0, 100, 30);

    /* Enable output of all EPWM channels */
    EPWM_EnableOutput(EPWM0, 0x3F);

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable brake and interrupt */
    EPWM_EnableFaultBrake(EPWM0, EPWM_CH_0_MASK, 1, EPWM_FB_EDGE_BKP0);
    EPWM_EnableFaultBrakeInt(EPWM0, EPWM_FB_EDGE);
    /* Enable brake noise filter : brake pin 0, filter count=7, filter clock=HCLK/128 */
    EPWM_EnableBrakeNoiseFilter(EPWM0, 0, 7, EPWM_NF_CLK_DIV_128);
    /* Clear brake interrupt flag */
    EPWM_ClearFaultBrakeIntFlag(EPWM0, EPWM_FB_EDGE);
    SYS_LockReg();

    NVIC_EnableIRQ(BRAKE0_IRQn);

    /* Start */
    EPWM_Start(EPWM0, EPWM_CH_0_MASK);

    printf("\nPress any key to generate a brake event\n");
    getchar();
    PD5 = 1;

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
