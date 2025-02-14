/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Implement LPTMR counting in periodic mode.
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
static volatile uint32_t g_au32LPTMRINTCount[2] = {0};

NVT_ITCM void LPTMR0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    if (LPTMR_GetIntFlag(LPTMR0) == 1)
    {
        /* Clear LPTMR0 time-out interrupt flag */
        LPTMR_ClearIntFlag(LPTMR0);

        g_au32LPTMRINTCount[0]++;
    }

    __DSB();
    __ISB();

    while (LPTMR_GetIntFlag(LPTMR0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for LPTMR0 IntFlag time-out!\n");
        }
    }
}

NVT_ITCM void LPTMR1_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    if (LPTMR_GetIntFlag(LPTMR1) == 1)
    {
        /* Clear LPTMR1 time-out interrupt flag */
        LPTMR_ClearIntFlag(LPTMR1);

        g_au32LPTMRINTCount[1]++;
    }

    __DSB();
    __ISB();

    while (LPTMR_GetIntFlag(LPTMR1))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for LPTMR1 IntFlag time-out!\n");
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
    CLK_SetModuleClock(LPTMR0_MODULE, CLK_LPTMRSEL_LPTMR0SEL_HIRC, 0);
    CLK_SetModuleClock(LPTMR1_MODULE, CLK_LPTMRSEL_LPTMR1SEL_PCLK4, 0);

    /* Enable LPTMR clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);
    CLK_EnableModuleClock(LPTMR1_MODULE);

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
    printf("|    LPTMR Periodic Interrupt Sample Code    |\n");
    printf("+--------------------------------------------+\n\n");

    printf("# LPTMR0 Settings:\n");
    printf("    - Clock source is HIRC       \n");
    printf("    - Time-out frequency is 1 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# LPTMR1 Settings:\n");
    printf("    - Clock source is PCLK4      \n");
    printf("    - Time-out frequency is 2 Hz\n");
    printf("    - Periodic mode             \n");
    printf("    - Interrupt enable          \n");
    printf("# Check LPTMR0 ~ LPTMR1 interrupt counts are reasonable or not.\n\n");

    /* Open LPTMR0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    LPTMR_Open(LPTMR0, LPTMR_PERIODIC_MODE, 1);
    LPTMR_EnableInt(LPTMR0);

    /* Open LPTMR1 in periodic mode, enable interrupt and 2 interrupt ticks per second */
    LPTMR_Open(LPTMR1, LPTMR_PERIODIC_MODE, 2);
    LPTMR_EnableInt(LPTMR1);

    /* Enable LPTMR0 ~ LPTMR1 NVIC */
    NVIC_EnableIRQ(LPTMR0_IRQn);
    NVIC_EnableIRQ(LPTMR1_IRQn);

    /* Clear LPTMR0 ~ LPTMR1 interrupt counts to 0 */
    g_au32LPTMRINTCount[0] = g_au32LPTMRINTCount[1] = 0;
    u32InitCount = g_au32LPTMRINTCount[0];

    /* Start LPTMR0 ~ LPTMR1 counting */
    LPTMR_Start(LPTMR0);
    LPTMR_Start(LPTMR1);

    /* Check LPTMR0 ~ LPTMR1 interrupt counts */
    printf("# LPTMR interrupt counts :\n");

    while (u32InitCount < 20)
    {
        if (g_au32LPTMRINTCount[0] != u32InitCount)
        {
            au32Counts[0] = g_au32LPTMRINTCount[0];
            au32Counts[1] = g_au32LPTMRINTCount[1];
            printf("    LPTMR0:%3d    LPTMR1:%3d\n",
                   au32Counts[0], au32Counts[1]);
            u32InitCount = g_au32LPTMRINTCount[0];

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
