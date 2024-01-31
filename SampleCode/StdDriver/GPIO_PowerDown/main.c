/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show how to wake up system from Power-down mode by GPIO interrupt.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/* GPIO can support NPD0 ~ NDP1 power-down mode */
#define TEST_POWER_DOWN_MODE    PMC_NPD0

/*------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode        */
/*------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(DEBUG_PORT);
    /* Set Power-down mode */
    PMC_SetPowerDownMode(TEST_POWER_DOWN_MODE, PMC_PLCTL_PLSEL_PL1);
    /* Enter to Power-down mode */
    PMC_PowerDown();
}

NVT_ITCM void GPB_IRQHandler(void)
{
    volatile uint32_t temp;
    CLK_EnableModuleClock(GPIOB_MODULE);

    /* To check if PB.3 interrupt occurred */
    if (GPIO_GET_INT_FLAG(PB, BIT3))
    {
        GPIO_CLR_INT_FLAG(PB, BIT3);
        printf("PB.3 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        temp = PB->INTSRC;
        PB->INTSRC = temp;
        printf("Un-expected interrupts.\n");
    }
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);
    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
    /* Enable PLL0 180MHz clock */
    CLK_EnableAPLL(CLK_APLLCTL_APLLSRC_HIRC, FREQ_180MHZ, CLK_APLL0_SELECT);
    /* Switch SCLK clock source to PLL0 */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_APLL0);
    /* Set HCLK2 divide 2 */
    CLK_SET_HCLK2DIV(2);
    /* Set PCLKx divide 2 */
    CLK_SET_PCLK0DIV(2);
    CLK_SET_PCLK1DIV(2);
    CLK_SET_PCLK2DIV(2);
    CLK_SET_PCLK3DIV(2);
    CLK_SET_PCLK4DIV(2);
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
    /* Enable UART module clock */
    SetDebugUartCLK();
    /* Enable GPIO Port B clock */
    CLK_EnableModuleClock(GPIOB_MODULE);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Enable Internal low speed RC clock */
    CLK_EnableXtalRC(CLK_SRCCTL_LIRCEN_Msk);
    /* Waiting for Internal low speed RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);
    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();
#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------+\n");
    printf("|    GPIO Power-Down and Wake-up by PB.3 Sample Code    |\n");
    printf("+-------------------------------------------------------+\n\n");
    /* Configure PB.3 as Input mode and enable interrupt by rising edge trigger */
    GPIO_SetMode(PB, BIT3, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 3, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPB_IRQn);
    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(PB, GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PB, BIT3);
    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Waiting for PB.3 rising-edge interrupt event */
    while (1)
    {
        printf("press any key to continue ...\n");
        getchar();
        /* Enter to Power-down mode */
        printf("Enter to Power-Down. Wait PB.3 rising edge interrupt to wake up ...\n");
        PowerDownFunction();
        printf("System waken-up done.\n\n");
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
