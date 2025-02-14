/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Implement TTMR counting in periodic mode.
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
/*----------------------------------------------------------------------*/
/* Global Interface Variables Declarations                              */
/*----------------------------------------------------------------------*/
static volatile uint32_t g_au32TTMRINTCount[2] = {0};

NVT_ITCM void TTMR0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    if (TTMR_GetIntFlag(TTMR0) == 1)
    {
        /* Clear TTMR0 time-out interrupt flag */
        TTMR_ClearIntFlag(TTMR0);

        g_au32TTMRINTCount[0]++;
    }

    __DSB();
    __ISB();

    while (TTMR_GetIntFlag(TTMR0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for TTMR0 IntFlag time-out!\n");
        }
    }
}

NVT_ITCM void TTMR1_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    if (TTMR_GetIntFlag(TTMR1) == 1)
    {
        /* Clear TTMR1 time-out interrupt flag */
        TTMR_ClearIntFlag(TTMR1);

        g_au32TTMRINTCount[1]++;
    }

    __DSB();
    __ISB();

    while (TTMR_GetIntFlag(TTMR1))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for TTMR1 IntFlag time-out!\n");
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
    /* Select LPTMR clock source */
    CLK_SetModuleClock(TTMR0_MODULE, CLK_TTMRSEL_TTMR0SEL_PCLK4, 0);
    CLK_SetModuleClock(TTMR1_MODULE, CLK_TTMRSEL_TTMR1SEL_HIRC, 0);

    /* Enable LPTMR clock */
    CLK_EnableModuleClock(TTMR0_MODULE);
    CLK_EnableModuleClock(TTMR1_MODULE);

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint32_t u32InitCount, au32Counts[4];

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("System core clock = %d\n", SystemCoreClock);
    printf("+--------------------------------------------+\n");
    printf("|    TTMR Periodic Interrupt Sample Code     |\n");
    printf("+--------------------------------------------+\n\n");

    printf("# TTMR0 Settings:\n");
    printf("    - Clock source is PCLK4     \n");
    printf("    - Time-out frequency is 1 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# TTMR1 Settings:\n");
    printf("    - Clock source is HIRC      \n");
    printf("    - Time-out frequency is 2 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# Check TTMR0 ~ TTMR1 interrupt counts are reasonable or not.\n\n");

    /* Open TTMR0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    TTMR_Open(TTMR0, TTMR_PERIODIC_MODE, 1);
    TTMR_EnableInt(TTMR0);

    /* Open TTMR1 in periodic mode, enable interrupt and 2 interrupt ticks per second */
    TTMR_Open(TTMR1, TTMR_PERIODIC_MODE, 2);
    TTMR_EnableInt(TTMR1);

    /* Enable TTMR0 ~ TTMR1 NVIC */
    NVIC_EnableIRQ(TTMR0_IRQn);
    NVIC_EnableIRQ(TTMR1_IRQn);

    /* Clear TTMR0 ~ TTMR1 interrupt counts to 0 */
    g_au32TTMRINTCount[0] = g_au32TTMRINTCount[1] = 0;
    u32InitCount = g_au32TTMRINTCount[0];

    /* Start TTMR0 ~ TTMR1 counting */
    TTMR_Start(TTMR0);
    TTMR_Start(TTMR1);

    /* Check TTMR0 ~ TTMR1 interrupt counts */
    printf("# TTMR interrupt counts :\n");

    while (u32InitCount < 20)
    {
        if (g_au32TTMRINTCount[0] != u32InitCount)
        {
            au32Counts[0] = g_au32TTMRINTCount[0];
            au32Counts[1] = g_au32TTMRINTCount[1];
            printf("    TTMR0:%3d    TTMR1:%3d\n",
                   au32Counts[0], au32Counts[1]);
            u32InitCount = g_au32TTMRINTCount[0];

            if ((au32Counts[1] > (au32Counts[0] * 2 + 1)) || (au32Counts[1] < (au32Counts[0] * 2 - 1)))
            {
                printf("*** FAIL ***\n");

                while (1);
            }
        }
    }

    printf("*** PASS ***\n");

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
