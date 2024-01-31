/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show how to continue executing code after wake-up form SPD Power-down mode by VTOR function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

uint32_t SRAM_LoadCodeAndRun(void);

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

    /* Enable PLL0 180MHz clock and set all bus clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

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
    uint32_t u32TimeOutCnt;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Unlock protected registers */
    SYS_UnlockReg();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\nSystem core clock = %d\n", SystemCoreClock);

    printf("+-----------------------------------------------------------------+\n");
    printf("|         SPD power-down wake-up and return sample code           |\n");
    printf("+-----------------------------------------------------------------+\n");
    printf("|[0]      Noraml SPD mode wake-up.                                |\n");
    printf("|[Others] SPD mode wake-up from VTOR                              |\n");
    printf("+-----------------------------------------------------------------+\n");

    if (getchar() == '0')
    {
        /* Excute normal SPD power-down wake-up */
    }
    else
    {
        /* Load user image to FLASH */
        SRAM_LoadCodeAndRun();

        /* Set VTOR to wake-up form FLASH */
        SYS->VTORSET = 0x00180000;
    }

    /* Select power-down mode and power level */
    PMC_SetPowerDownMode(PMC_SPD0, PMC_PLCTL_PLSEL_PL1);

    /* Enable wake-up timer and set wake-up Time-out Interval */
    PMC_EnableSTMR(PMC_STMRWKCTL_STMRIS_65536);

    printf("Enter to SPD Power-down mode...\n");

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(DEBUG_PORT)

    if (--u32TimeOutCnt == 0) break;

    /* Enter to power-down */
    PMC_PowerDown();

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
