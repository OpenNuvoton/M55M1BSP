/******************************************************************************
 * @file     main.c
 * @version  V3.01
 * @brief    Demo how to use XOM library in dual bank.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "xomlib.h"

/* XOM limitation : After return from XOM region function, need to delay one cycle */
/* The marco XOM_CALL is using for avoid XOM limitation. */
#define XOM_CALL(pfunc, ret, ...)      { ret = pfunc(__VA_ARGS__); __NOP(); }

int32_t SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*------------------------------------------------------------------------*/
    /* Init System Clock                                                      */
    /*------------------------------------------------------------------------*/
    /* Enable PLL0 220MHz clock from HIRC and switch SCLK clock source to PLL0 */
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                */
    /*------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();

    return 0;
}

int32_t main(void)
{
    uint32_t u32Data = 0;
    int32_t  ai32NumArray[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    InitDebugUart();
    /*
        This sample code is used to show how to call XOM libary in dual bank.

        The XOM libary is build by XOMLib project.
        Users need to add include path of xomlib.h and add object file xomlib.lib(Keil)/xomlib.a(IAR)/libXOMLib.a(GCC)
        to using XOM library built by XOMLib project.
    */

    printf("+----------------------------------------+\n");
    printf("|  Demo how to use XOM library in Bank%d  |\n", DL_BANK);
    printf("+----------------------------------------+\n");

    XOM_CALL(XOM_Add, u32Data, 100, (200 + DL_BANK));
    printf(" 100 + %d = %d\n", (200 + DL_BANK), u32Data);

    XOM_CALL(XOM_Sub, u32Data, 500, (100 + DL_BANK));
    printf(" 500 - %d = %d\n", (100 + DL_BANK), u32Data);

    XOM_CALL(XOM_Mul, u32Data, 200, (100 + DL_BANK));
    printf(" 200 * %d = %d\n", (100 + DL_BANK), u32Data);

    XOM_CALL(XOM_Div, u32Data, 1000, ((DL_BANK == 0) ? 250 : 500));
    printf("1000 / %d = %d\n", ((DL_BANK == 0) ? 250 : 500), u32Data);

    u32Data = XOM_Sum(ai32NumArray, sizeof(ai32NumArray) / sizeof(ai32NumArray[0]));
    printf("Sum of ai32NumArray = %d\n", u32Data);

    printf("Done\n");

    while (1);
}

/*** (C) COPYRIGHT 2026 Nuvoton Technology Corp. ***/
