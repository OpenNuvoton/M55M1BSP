/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show how to continue executing code after wake-up from SPD Power-down mode by SRAM data retention function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

extern uint32_t g_u32ReturnAddr, g_bReturnToAddr;
void SPDMode_WakeupAndReturn(uint32_t bReturn);

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Release GPIO hold status */
    PMC_RELEASE_GPIO();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Waiting for Internal RC 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable PLL0 220MHz clock and set all bus clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}



int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Unlock protected registers */
    SYS_UnlockReg();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("System core clock = %d\n", SystemCoreClock);

    printf("+-----------------------------------------------------------------+\n");
    printf("|         SPD power-down wake-up and return sample code           |\n");
    printf("+-----------------------------------------------------------------+\n");
    printf("|[0]        SYS_ResetChip.                                        |\n");
    printf("|[Others]   SPDMode_WakeupAndReturn.                              |\n");
    printf("+-----------------------------------------------------------------+\n");

    if (getchar() == '0')
    {
        uint32_t u32TimeOutCnt = SystemCoreClock;

        g_u32ReturnAddr   = 0;
        g_bReturnToAddr = FALSE;

        printf("SYS_ResetChip\n");

        /* Check if all the debug messages are finished */
        UART_WAIT_TX_EMPTY(DEBUG_PORT)

        if (--u32TimeOutCnt == 0) break;

        SYS_ResetChip();
    }
    else
    {
        printf("SPDMode_WakeupAndReturn\n");
        SPDMode_WakeupAndReturn(FALSE);
    }

    /* Clear all wake-up flag */
    PMC->INTSTS |= PMC_INTSTS_CLRWK_Msk;

    printf("Done.\n");

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
