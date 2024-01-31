/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to wake up system form DPD Power-down mode by different wakeup sources.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void PowerDownFunction(void);
void WakeUpPinFunction(uint32_t u32PDMode, uint32_t u32EdgeType);
void WakeUpTimerFunction(uint32_t u32PDMode, uint32_t u32Interval);
void CheckPowerSource(void);
void SYS_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(DEBUG_PORT)

    if (--u32TimeOutCnt == 0) break;

    /* Enter to Power-down mode */
    PMC_PowerDown();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by Wake-up pin                         */
/*---------------------------------------------------------------------------------------------------------*/
void WakeUpPinFunction(uint32_t u32PDMode, uint32_t u32EdgeType)
{
    printf("Enter to DPD Power-down mode......\n");

    /* Select Power-down mode */
    PMC_SetPowerDownMode(u32PDMode, PMC_PLCTL_PLSEL_PL0);

    /* Set Wake-up pin trigger type at Deep Power down mode */
    PMC_EnableWKPIN(u32EdgeType);

    /* Enter to Power-down mode and wait for wake-up reset happen */
    PowerDownFunction();
}

/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by Wake-up Timer                         */
/*-----------------------------------------------------------------------------------------------------------*/
void WakeUpTimerFunction(uint32_t u32PDMode, uint32_t u32Interval)
{
    printf("Enter to DPD Power-down mode......\n");

    /* Select Power-down mode */
    PMC_SetPowerDownMode(u32PDMode, PMC_PLCTL_PLSEL_PL0);

    /* Enable wake-up timer and set wake-up Time-out Interval */
    PMC_EnableSTMR(u32Interval);

    /* Enter to Power-down mode and wait for wake-up reset */
    PowerDownFunction();
}

/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for Check Power Manager Status                                                                  */
/*-----------------------------------------------------------------------------------------------------------*/
void CheckPowerSource(void)
{
    uint32_t u32RegRstsrc;
    u32RegRstsrc = PMC_GetPMCWKSrc();

    printf("Wake-up source 0x%x\n", u32RegRstsrc);

    if ((u32RegRstsrc & PMC_INTSTS_PIN0WKIF_Msk) != 0)
        printf("Wake-up source is Wake-up Pin.\n");

    if ((u32RegRstsrc & PMC_INTSTS_STMRWKIF_Msk) != 0)
        printf("Wake-up source is Standby Wake-up Timer.\n");

    /* Clear all wake-up flag */
    PMC->INTSTS |= PMC_INTSTS_CLRWK_Msk;
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Release GPIO hold status */
    PMC_RELEASE_GPIO();

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

    /* Set PF multi-function pins for X32_OUT(PF.4) and X32_IN(PF.5) */
    SET_X32_OUT_PF4();
    SET_X32_IN_PF5();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Waiting for Internal RC 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable Internal RC 48MHz clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRC48MEN_Msk);

    /* Enable Internal low speed RC clock */
    CLK_EnableXtalRC(CLK_SRCCTL_LIRCEN_Msk);

    /* Waiting for Internal low speed RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Switch SCLK clock source to HIRC */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_HIRC);

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

    /* Set PC multi-function pin for CLKO(PC.13) */
    SET_CLKO_PC13();

    /* Enable GPIO F module clock */
    CLK_EnableModuleClock(GPIOF_MODULE);

    /* Configure GPIO as input mode */
    GPIO_SetMode(PF, BIT6, GPIO_MODE_INPUT);

    /* Due to PF.6 is used as the PIN wake up source. */
    CLK_EnableModuleClock(RTC0_MODULE);
    RTC->LXTCTL = (RTC->LXTCTL & ~RTC_GPIOCTL0_OPMODE1_Msk) | (1 << RTC_GPIOCTL0_OPMODE1_Pos);

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint8_t u8Item;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /* Get power manager wake up source */
    CheckPowerSource();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    CLK_EnableCKO(CLK_CLKOSEL_CLKOSEL_SYSCLK, 3, CLK_CLKOCTL_DIV1EN_DIV_FREQSEL);

    printf("+----------------------------------------------------------------+\n");
    printf("|    DPD Power-down Mode and Wake-up Sample Code.                |\n");
    printf("|    Please Select Wake up source.                               |\n");
    printf("+----------------------------------------------------------------+\n");
    printf("|[1] DPD Wake-up Pin(PF.6) trigger type is rising edge.          |\n");
    printf("|[2] DPD Wake-up TIMER time-out interval is 4096 LIRC clocks.    |\n");
    printf("+----------------------------------------------------------------+\n");
    u8Item = (uint8_t)getchar();

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Disable wake-up timer */
    PMC_DisableSTMR();

    switch (u8Item)
    {
        case '1':
            printf("[1]\n");
            WakeUpPinFunction(PMC_DPD0, PMC_WKPIN4_RISING);
            break;

        case '2':
            printf("[2]\n");
            WakeUpTimerFunction(PMC_DPD0, PMC_STMRWKCTL_STMRIS_4096);
            break;

        default:
            break;
    }

    /* Lock protected registers */
    SYS_LockReg();

    while (1);
}
