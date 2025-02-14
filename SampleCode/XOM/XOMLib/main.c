/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to config/erase XOM region.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"
#include "xomapi.h"

#define XOM_START       0x00104000
#define XOM_SIZE        0x00002000

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();
    CLK_EnableModuleClock(ISP0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

int32_t main(void)
{
    char cUserSel;
    int32_t i, r;
    int32_t ai32NumArray[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    uint32_t u32XOMBaseAddr;

#if defined (__ICCARM__)
    extern uint32_t __region_XOM_start__;
    u32XOMBaseAddr = (uint32_t)&__region_XOM_start__;
#elif defined(__ARMCC_VERSION)
    extern uint32_t Image$$XOM0_ROM$$Base;

    u32XOMBaseAddr = (uint32_t)&Image$$XOM0_ROM$$Base;
#else
    extern uint32_t __region_XOM_start__;

    u32XOMBaseAddr = (uint32_t)&__region_XOM_start__;
#endif

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /*
        This sample code is used to show how to build an XOM libary.

        The location of XOM region is defined by linker file:
          XOMLib_Code.scatter(Keil)/XOMLib_Code.icf(IAR)/XOMLib_Code.ld(GCC/VSCode)
        Exported XOM library header file is .\lib\xomlib.h
        Exported XOM functions are implemented in xom.c

        This project is only used to build code for XOM region and test its functions.
        To enable XOM region, please use "NuMicro ICP Programming Tool" or "[0] Set XOM".

        Example flow:
        1. Build XOMLib_Code and test XOM functions.
        2. There are two methods to set and enable XOM region.
           * Execute "[0] Set XOM" in XOMLib_Code.
           * Use defined XOM_START and XOM_SIZE in
               XOMLib_Code.scatter(Keil)/XOMLib_Code.icf(IAR)/XOMLib_Code.ld(GCC/VSCode).
             Then open "NuMicro ICP Programming Tool" to set and enable XOM region.
        3. Test XOM function with XOM enabled again.
        4. Review xomlib.c and .\lib\xomlib.h to check all XOM function pointers are
           included correctly (Check function address in "[1] Test XOM" output).
           Copy XOMLib address definitions to xomlib.c manually.
        5. Build XOMLib project to generate xomlib.lib(Keil)/xomlib.a(IAR)/libXOMLib.a(GCC/VSCode).
           It includes function pointers for XOM.
           The XOM library (xomlib.lib, xomlib.a and libXOMLib.a) and header (xomlib.h)
           are located at lib directory.
        7. Pass xomlib.lib(Keil)/xomlib.a(IAR)/libXOMLib.a(GCC/VSCode) and xomlib.h to the users
           who will call these functions in XOM.
    */

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|      FMC XOM Library Build Example     |\n");
    printf("+----------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function and enable APROM active*/
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();
    FMC_ENABLE_CFG_UPDATE();

    /* Read XOM Status */
    printf("XOM Status = 0x%X\n", FMC->XOMSTS);
    printf("[0] Set  XOM\n");
    printf("[1] Test XOM\n");

    cUserSel = (char)getchar();

    if (cUserSel == '0')
    {
        printf("Config XOM0\n");
        printf("  Base address: 0x%08X, page count: %d\n", (uint32_t)XOM_START, (uint32_t)(XOM_SIZE / FMC_FLASH_PAGE_SIZE));
        UART_WAIT_TX_EMPTY(DEBUG_PORT);

        FMC_ConfigXOM(0, XOM_START, (XOM_SIZE / FMC_FLASH_PAGE_SIZE));
        SYS_ResetChip();

        while (1) ;
    }

    printf("Copy below XOM libraray address definitions to xomlib.c to build XOMLib.\n");
    printf("/*------------------------------*/\n");
    printf("/*  XOMLib address definitions  */\n");
    printf("/*------------------------------*/\n");
    printf("#define XOM_ADD_ADDR    0x%08X\n", (uint32_t)XOM_Add);
    printf("#define XOM_SUB_ADDR    0x%08X\n", (uint32_t)XOM_Sub);
    printf("#define XOM_MUL_ADDR    0x%08X\n", (uint32_t)XOM_Mul);
    printf("#define XOM_DIV_ADDR    0x%08X\n", (uint32_t)XOM_Div);
    printf("#define XOM_SUM_ADDR    0x%08X\n", (uint32_t)XOM_Sum);

    /* Run XOM function */
    printf("\n");
    printf("Check XOM execution\n");
    printf("  [XOM_Add]  100 + 200 = %d\n", XOM_Add(100, 200));
    printf("  [XOM_Sub]  500 - 100 = %d\n", XOM_Sub(500, 100));
    printf("  [XOM_Mul]  200 * 100 = %d\n", XOM_Mul(200, 100));
    printf("  [XOM_Div] 1000 / 250 = %d\n", XOM_Div(1000, 250));
    printf("  [XOM_Sum] Sum of ai32NumArray = %d\n", XOM_Sum(ai32NumArray, sizeof(ai32NumArray) / sizeof(ai32NumArray[0])));

    for (i = 0; i < 1000; i++)
    {
        r = XOM_Add(500, 700);

        if (r != 1200)
        {
            printf("XOM ADD fail. It should be 1200 but %d\n", r);
            goto lexit;
        }
    }

    printf("\n");

    if (FMC->XOMSTS == 0x1)
        printf("Check CPU access XOM region all 0xFFFFFFFF.\n");
    else
        printf("Check CPU access XOM region not 0xFFFFFFFF.\n");

    for (i = 0; i < 16; i++)
    {
        printf("  [0x%08X] = 0x%08X\n", u32XOMBaseAddr + i * 4, M32(u32XOMBaseAddr + i * 4));
    }

lexit:
    printf("Done.\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
