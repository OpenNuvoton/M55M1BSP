/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to set different core voltage and main voltage regulator type.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


static volatile uint8_t s_u8IsINTEvent = 0;

void WDT0_IRQHandler(void);
void PowerDownFunction(void);
int32_t pi(void);
void CheckSystemWork(void);
void SYS_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  WDT0 IRQ Handler                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void WDT0_IRQHandler(void)
{
    if (WDT_GET_TIMEOUT_INT_FLAG(WDT0))
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG(WDT0);
    }

    if (WDT_GET_TIMEOUT_WAKEUP_FLAG(WDT0))
    {
        printf("WDT Wake-up!!!\n");

        /* Clear WDT time-out wake-up flag */
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG(WDT0);
    }

    s_u8IsINTEvent = 1;

    /* CPU read interrupt flag register to wait write(clear) instruction completement */
    inp32(&WDT0->STATUS);
}

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
/*  Simple calculation test function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define PI_NUM  256
static int32_t s_ai32f[PI_NUM + 1];
static uint32_t s_au32piTbl[19] =
{
    3141,
    5926,
    5358,
    9793,
    2384,
    6264,
    3383,
    2795,
    288,
    4197,
    1693,
    9937,
    5105,
    8209,
    7494,
    4592,
    3078,
    1640,
    6284
};

static int32_t s_ai32piResult[19];

int32_t pi(void)
{
    int32_t i, i32Err;
    int32_t a = 10000, b = 0, c = PI_NUM, d = 0, e = 0, g = 0;

    for (; b - c;)
        s_ai32f[b++] = a / 5;

    i = 0;

    for (; (void)(d = 0), g = c * 2; c -= 14, s_ai32piResult[i++] = e + d / a, e = d % a)
    {
        if (i == 19)
            break;

        for (b = c; (void)(d += s_ai32f[b] * a), (void)(s_ai32f[b] = d % --g), (void)(d /= g--), --b; d *= b);
    }

    i32Err = 0;

    for (i = 0; i < 19; i++)
    {
        if (s_au32piTbl[i] != (uint32_t)s_ai32piResult[i])
            i32Err = -1;
    }

    return i32Err;
}

void CheckSystemWork(void)
{
    if (pi())
    {
        printf("[FAIL]\n");
    }
    else
    {
        printf("[OK]\n");
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

    /* Set PF multi-function pins for X32_OUT(PF.4) and X32_IN(PF.5) */
    SET_X32_OUT_PF4();
    SET_X32_IN_PF5();

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

    /* Enable WDT0 module clock */
    CLK_EnableModuleClock(WDT0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TimeOutCnt;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+---------------------------------------+\n");
    printf("|         Power Mode Sample Code        |\n");
    printf("+---------------------------------------+\n");

    /* Unlock protected registers before setting power level and main voltage regulator type */
    SYS_UnlockReg();

    /* Set HCLK clock as MIRC */
    CLK_SetSCLK(CLK_SCLKSEL_SCLKSEL_MIRC);

    /* Set power level to 1.1V */
    printf("Set power level to 1.1V...");
    PMC_SetPowerLevel(PMC_PLCTL_PLSEL_PL1);

    /* Check system work */
    CheckSystemWork();

    /* Set power level to 1.15V */
    printf("Set power level to 1.15V..");
    PMC_SetPowerLevel(PMC_PLCTL_PLSEL_PL0);

    /* Set core clock as 200MHz from PLL */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_200MHZ);

    /* Check system work */
    CheckSystemWork();

    /* Set core clock as 180MHz from PLL */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HXT, FREQ_180MHZ);

    /* Set power level to 1.1V */
    printf("Set power level to 1.1V...");
    PMC_SetPowerLevel(PMC_PLCTL_PLSEL_PL1);

    /* Check system work */
    CheckSystemWork();

    /* Set main voltage regulator type to DCDC mode */
    printf("Set main voltage regulator type to DCDC mode..");

    if (PMC_SetPowerRegulator(PMC_VRCTL_MVRS_DCDC) != PMC_OK)
        printf("[no inductor connect]\n");
    else
        CheckSystemWork();      /* Check system work */

    /* Set main voltage regulator type to LDO mode */
    printf("Set main voltage regulator type to LDO mode...");
    PMC_SetPowerRegulator(PMC_VRCTL_MVRS_LDO);

    /* Check system work */
    CheckSystemWork();

    /* Enter to Power-down Mode and wake-up by WDT in LDO mode */
    printf("Press any key to entering power-down mode\n");
    getchar();

    /* Enable WDT0 NVIC */
    NVIC_EnableIRQ(WDT0_IRQn);

    /* Configure WDT settings and start WDT counting */
    WDT_Open(WDT0, WDT_TIMEOUT_2POW16, (uint32_t)NULL, FALSE, TRUE);

    /* Enable WDT0 interrupt function */
    WDT_EnableInt(WDT0);

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Check if WDT time-out interrupt and wake-up occurred or not */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (s_u8IsINTEvent == 0)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for WDT interrupt time-out!\n");
            break;
        }
    }

    printf("Check system work...");

    /* Check system work */
    CheckSystemWork();

    printf("Sample code end.\n");

    while (1);

}
