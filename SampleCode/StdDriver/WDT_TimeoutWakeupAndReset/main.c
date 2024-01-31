/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Implement WDT0 time-out interrupt event to wake up system and generate
 *           time-out reset system event while WDT time-out reset delay period expired.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/

volatile uint32_t g_u32WDTINTCounts;
volatile uint8_t g_u8IsWDTWakeupINT;

int IsDebugFifoEmpty(void)
{
    return ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) != 0U);
}

/**
 * @brief       IRQ Handler for WDT Interrupt
 *
 * @param       None
 *
 * @return      None
 */
NVT_ITCM void WDT0_IRQHandler(void)
{
    uint32_t intflag;

    // TESTCHIP_ONLY
    CLK_WaitModuleClockReady(WDT0_MODULE);

    if (g_u32WDTINTCounts < 10)
        WDT_RESET_COUNTER(WDT0);

    if (WDT_GET_TIMEOUT_INT_FLAG(WDT0) == 1)
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG(WDT0);

        g_u32WDTINTCounts++;
    }

    if (WDT_GET_TIMEOUT_WAKEUP_FLAG(WDT0) == 1)
    {
        /* Clear WDT time-out wake-up flag */
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG(WDT0);

        g_u8IsWDTWakeupINT = 1;
    }

    /* make sure that interrupt flag has been cleared. */
    intflag = WDT0->STATUS;
    NVT_UNUSED(intflag);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);
    /* Enable Internal RC 32KHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_LIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Waiting for Low speed Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);


    /* Switch SCLK clock source to PLL0 and Enable PLL0 180MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set module clock*/
    CLK_SetModuleClock(WDT0_MODULE, CLK_WDTSEL_WDT0SEL_LIRC, 0);

    /* Enable module clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);
    CLK_EnableModuleClock(WDT0_MODULE);

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set PC multi-function pins for GPIO*/
    SET_GPIO_PA0();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init DeubgUART for printf */
    InitDebugUart();

    printf("\n\nCPU @ %u Hz\n", SystemCoreClock);
    printf("+----------------------------------------+\n");
    printf("|    WDT Time-out Wake-up Sample Code    |\n");
    printf("+----------------------------------------+\n\n");

    /* To check if system has been reset by WDT time-out reset or not */
    if (WDT_GET_RESET_FLAG(WDT0) == 1)
    {
        WDT_CLEAR_RESET_FLAG(WDT0);
        printf("*** System has been reset by WDT time-out event ***\n\n");

        while (1);
    }

    printf("# WDT Settings:\n");
    printf("    - Clock source is LIRC(32K Hz)          \n");
    printf("    - Time-out interval is 2^14 * WDT clock \n");
    printf("      (around 0.512 ~ 0.515 s)              \n");
    printf("    - Interrupt enable                      \n");
    printf("    - Wake-up function enable               \n");
    printf("    - Reset function enable                 \n");
    printf("    - Reset delay period is 18 * WDT clock  \n");
    printf("# System will generate a WDT time-out interrupt event after 0.512 ~ 0.515 s, \n");
    printf("    and will be wake up from power-down mode also.\n");
    printf("    (Use PA.0 high/low period time to check WDT time-out interval)\n");
    printf("# When WDT interrupt counts large than 10, \n");
    printf("    we will not reset WDT counter value and system will be reset immediately by WDT time-out reset signal.\n");

    /* Use PA.0 to check time-out period time */
    GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT);

    PA0 = 1;

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT0_IRQn);

    /* Because of all bits can be written in WDT Control Register are write-protected;
       To program it needs to disable register protection first. */
    SYS_UnlockReg();

    /* Configure WDT settings and start WDT counting */
    g_u32WDTINTCounts = g_u8IsWDTWakeupINT = 0;

    WDT_Open(WDT0, WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_18CLK, TRUE, TRUE);

    /* Enable WDT interrupt function */
    WDT_EnableInt(WDT0);

    while (1)
    {
        /* System enter to Power-down */
        /* To program PWRCTL register, it needs to disable register protection first. */
        SYS_UnlockReg();
        PMC_SetPowerDownMode(PMC_NPD0, PMC_PLCTL_PLSEL_PL1);
        printf("\nSystem enter to power-down mode ...\n");

        /* To check if all the debug messages are finished */
        while (IsDebugFifoEmpty() == 0);

        PMC_PowerDown();

        /* Check if WDT time-out interrupt and wake-up occurred or not */
        while (g_u8IsWDTWakeupINT == 0);

        g_u8IsWDTWakeupINT = 0;
        PA0 ^= 1;

        printf("System has been waken up done. WDT interrupt counts: %u.\n\n", g_u32WDTINTCounts);
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

