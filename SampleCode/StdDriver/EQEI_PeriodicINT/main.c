/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief
 *           Show the usage of EQEI UINT timer function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_au32TMRINTCount[2] = {0};

/**
 * @brief       IRQ Handler for EQEI0 Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The EQEI0_IRQHandler is default IRQ of EQEI0, declared in startup_M55M1.c.
 */
NVT_ITCM void EQEI0_IRQHandler(void)
{
    uint32_t intflag;

    if (EQEI_GET_INT_FLAG(EQEI0, EQEI_STATUS_UTIEF_Msk))    /* EQEI Unit Timer Event flag */
    {
        EQEI_CLR_INT_FLAG(EQEI0, EQEI_STATUS_UTIEF_Msk);
        printf("Unit TImer0 INT!\n\n");
        g_au32TMRINTCount[0]++;
    }

    /* make sure that interrupt flag has been cleared. */
    intflag = EQEI0->STATUS;
    NVT_UNUSED(intflag);

}
/**
 * @brief       IRQ Handler for EQEI1 Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The EQEI1_IRQHandler is default IRQ of EQEI1, declared in startup_M55M1.c.
 */
NVT_ITCM void EQEI1_IRQHandler(void)
{
    uint32_t intflag;

    if (EQEI_GET_INT_FLAG(EQEI1, EQEI_STATUS_UTIEF_Msk))    /* EQEI Unit Timer Event flag */
    {
        EQEI_CLR_INT_FLAG(EQEI1, EQEI_STATUS_UTIEF_Msk);
        printf("Unit TImer1 INT!\n\n");
        g_au32TMRINTCount[1]++;
    }

    /* make sure that interrupt flag has been cleared. */
    intflag = EQEI1->STATUS;
    NVT_UNUSED(intflag);

}
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch SCLK clock source to PLL0 and Enable PLL0 72MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_72MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable module clock */
    CLK_EnableModuleClock(EQEI0_MODULE);
    CLK_EnableModuleClock(EQEI1_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
}
/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32InitCount, au32Counts[2];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init DeubgUART for printf */
    InitDebugUart();

    printf("CPU @ %uHz\n", SystemCoreClock);
    printf("PLCK0 @ %uHz\n", CLK_GetPCLK0Freq());
    printf("PLCK2 @ %uHz\n\n", CLK_GetPCLK2Freq());
    printf("+-----------------------------------------+\n");
    printf("|     EQEI Unit Timer Sample Code         |\n");
    printf("+-----------------------------------------+\n\n");
    printf("# EQEI0 Unit Timer Settings:\n");
    printf("    - Clock source is PCLK0       \n");
    printf("    - Compare Value frequency is 1 Hz\n");
    printf("    - Interrupt enable          \n");
    printf("# EQEI1 Unit Timer Settings:\n");
    printf("    - Clock source is PCLK2      \n");
    printf("    - Compare Value frequency is 2 Hz\n");
    printf("    - Interrupt enable          \n");

    /* Set Unit Timer compare value */
    EQEI_SET_UINT_TIMER_CMP_VALUE(EQEI0, CLK_GetPCLK0Freq());
    EQEI_SET_UINT_TIMER_CMP_VALUE(EQEI1, (CLK_GetPCLK2Freq() / 2));

    /* Enable EQEI interrupt */
    EQEI_EnableUintTimerINT(EQEI0);
    EQEI_EnableUintTimerINT(EQEI1);
    NVIC_EnableIRQ(EQEI0_IRQn);
    NVIC_EnableIRQ(EQEI1_IRQn);


    /* Clear Unit Timer0 ~ Timer1 interrupt counts to 0 */
    g_au32TMRINTCount[0] = g_au32TMRINTCount[1] = 0;
    u32InitCount = g_au32TMRINTCount[0];

    /* Start EQEI Unit TImer */
    EQEI_StartUintTimer(EQEI0);
    EQEI_StartUintTimer(EQEI1);

    /* Check EQEI Unit Timer0 ~ Timer1 interrupt counts */
    printf("# EQEI Unit Timer interrupt counts :\n");

    while (u32InitCount < 20)
    {
        if (g_au32TMRINTCount[0] != u32InitCount)
        {
            au32Counts[0] = g_au32TMRINTCount[0];
            au32Counts[1] = g_au32TMRINTCount[1];
            printf("    Unit Timer0:%3u    Unit Timer1:%3u\n", au32Counts[0], au32Counts[1]);
            u32InitCount = g_au32TMRINTCount[0];

            if ((au32Counts[1] > (au32Counts[0] * 2 + 1)) || (au32Counts[1] < (au32Counts[0] * 2 - 1)))
            {
                printf("*** FAIL ***\n");

                while (1);
            }
        }
    }

    printf("*** PASS ***\n");

    while (1);

}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
