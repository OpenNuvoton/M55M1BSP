/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show how to execute code after wake-up form SPD Power-down mode by VTOR function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

void SPDMode_WakeupAndReturn(uint32_t bReturn);

#define CONTINUE_ADDRESS    0x00100000
#define SPECIFIC_ADDRESS    0x00110000

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
    printf("\nSystem core clock = %d\n", SystemCoreClock);

    printf("+-----------------------------------------------------------------+\n");
    printf("|         SPD power-down wake-up by VTOR sample code              |\n");
    printf("+-----------------------------------------------------------------+\n");
    printf("|[0]      SPD mode wake-up to continue execution                  |\n");
    printf("|[Others] SPD mode wake up to execute code from specific address  |\n");
    printf("+-----------------------------------------------------------------+\n");

    if (getchar() == '0')
    {
        /* Set the VTOR to continue execution */
        SYS->VTORSET = CONTINUE_ADDRESS;

        SPDMode_WakeupAndReturn(FALSE);

        printf("Test done.\n");
    }
    else
    {
        /* Set VTOR to wake-up form specific address */
        SYS->VTORSET = SPECIFIC_ADDRESS;

        SPDMode_WakeupAndReturn(FALSE);
    }

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
