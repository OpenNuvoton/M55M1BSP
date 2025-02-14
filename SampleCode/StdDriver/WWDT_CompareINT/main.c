/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to reload the WWDT0/1 counter value.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint8_t g_u8WWDTINTCounts;

#define WWDT_PORT   (0)
#if (WWDT_PORT != 0)
    #define WWDT            WWDT1
    #define WWDT_IRQHandler WWDT1_IRQHandler
#else
    #define WWDT            WWDT0
    #define WWDT_IRQHandler WWDT0_IRQHandler
#endif

/**
 * @brief       IRQ Handler for WWDT0/1 Interrupt
 * @param       None
 * @return      None
 * @details     The WDT_IRQHandler is default IRQ of WWDT
 */
NVT_ITCM void WWDT_IRQHandler(void)
{
    if (WWDT_GET_INT_FLAG(WWDT) == 1)
    {
        /* Clear WWDT compare match interrupt flag */
        WWDT_CLEAR_INT_FLAG(WWDT);

        PA0 ^= 1;

        g_u8WWDTINTCounts++;

        if (g_u8WWDTINTCounts < 10)
        {
            /* To reload the WWDT counter value to 0x3F */
            WWDT_RELOAD_COUNTER(WWDT);
        }

        printf("WWDT compare match interrupt occurred. (%d)\n", g_u8WWDTINTCounts);
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);
    CLK_EnableXtalRC(CLK_SRCCTL_LIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Switch SCLK clock source to PLL0 and Enable PLL0 220MHz clock */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set module clock*/
#if (WWDT_PORT != 0)
    CLK_SetModuleClock(WWDT1_MODULE, CLK_WWDTSEL_WWDT1SEL_LIRC, 0);
#else
    CLK_SetModuleClock(WWDT0_MODULE, CLK_WWDTSEL_WWDT0SEL_LIRC, 0);
#endif

    /* Enable module clock */
    CLK_EnableModuleClock(GPIOA_MODULE);
    CLK_EnableModuleClock(GPIOB_MODULE);

#if (WWDT_PORT != 0)
    CLK_EnableModuleClock(WWDT1_MODULE);
#else
    CLK_EnableModuleClock(WWDT0_MODULE);
#endif

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
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    double dPeriodTime;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init DeubgUART for printf */
    InitDebugUart();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %u Hz\n", SystemCoreClock);
    printf("+------------------------------------------------+\n");
    printf("|    WWDT Compare Match Interrupt Sample Code    |\n");
    printf("+------------------------------------------------+\n\n");

    /* To check if system has been reset by WWDT time-out reset or not */
    if (WWDT_GET_RESET_FLAG(WWDT) == 1)
    {
        printf("*** System has been reset by WWDT time-out reset event. [WWDT_CTL: 0x%08X] ***\n\n", WWDT->CTL);
        WWDT_CLEAR_RESET_FLAG(WWDT);

        while (1) {}
    }

    dPeriodTime = ((((double)(1000000) / (double)__LIRC) * 1024) * 32) / 1000;

    printf("# WWDT Settings: \n");
    printf("    - Clock source is LIRC (32KHz)      \n");
    printf("    - WWDT counter prescale period is 1024, \n");
    printf("        and max WWDT time-out period is 1024 * (64 * WWDT_CLK)\n");
    printf("    - Interrupt enable                      \n");
    printf("    - Window Compare value is 32            \n");
    printf("# System will generate first WWDT compare match interrupt event after %.2f ms.\n", (dPeriodTime));
    printf("    1.) use PA.0 high/low period to check WWDT compare match interrupt period time\n");
    printf("    2.) reload WWDT counter value to avoid WWDT time-out reset system occurred\n");
    printf("        when interrupt counts less than 11.\n");
    printf("    3.) do not reload WWDT counter value to generate WWDT time-out reset system event\n");
    printf("        when interrupt counts large than 10.\n\n");

    /* Use PA.0 to check WWDT compare match interrupt period time */
    GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT);
    PA0 = 1;

    /* Enable WWDT NVIC */
#if (WWDT_PORT != 0)
    NVIC_EnableIRQ(WWDT1_IRQn);
#else
    NVIC_EnableIRQ(WWDT0_IRQn);
#endif

    g_u8WWDTINTCounts = 0;

    /*
        Max time-out period is 1024*(64*WWDT_CLK);
        WWDT compare value is 32;
        Enable WWDT compare match interrupt;
    */
    /* Note: WWDT_CTL register can be written only once after chip is powered on or reset */
    WWDT_Open(WWDT, WWDT_PRESCALER_1024, 32, TRUE);

    printf("[WWDT_CTL: 0x%08X]\n\n", WWDT->CTL);

    while (1) {}
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
