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
    int32_t ai32NumArray[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /*
        This sample code is used to show how to build an XOM libary.

        The location of XOM region is defined by linker file: xom_scatter.scf(Keil)/xom.icf(IAR)
        The API header file is xomapi.h
        The XOM functions are implemented in xom.c

        This project is only used to build code for XOM region and test its functions.
        To enable XOM region, please use "NuMicro ICP Programming Tool".

        Example flow:
        1. Build XOMLib_Code and test XOM functions
        2. Use defined XOM_START and XOM_SIZE in xom.scatter or xom.icf and
           open "NuMicro ICP Programming Tool" to set and enable XOM region.
        3. Test XOM function with XOM enabled again.
        4. Review xomlib.c and .\lib\xomlib.h to check all XOM function pointers are
           included correctly (Check funtion address in .map file).
        5. Build final XOMLib_Code. XOMAddr.exe will be executed to update
           function pointer addresses after built.
        6. Build XOMLib project to generate xomlib.lib(Keil)/xomlib.a(IAR).
           It includes function pointers for XOM.
           The library (xomlib.lib or xomlib.a) and header (xomlib.h) is located at lib directory.
        7. Pass xomlib.lib(Keil)/xomlib.a(IAR) & xomlib.h to the users
           who will call the funcitons in XOM.
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
        FMC_ConfigXOM(0, XOM_START, (XOM_SIZE / FMC_FLASH_PAGE_SIZE));
        SYS_ResetChip();

        while (1) ;
    }

    /* Run XOM function */
    printf("\n");
    printf("Check XOM execution\n");
    printf("  [XOM_Add]  100 + 200 = %d\n", XOM_Add(100, 200));
    printf("  [XOM_Sub]  500 - 100 = %d\n", XOM_Sub(500, 100));
    printf("  [XOM_Mul]  200 * 100 = %d\n", XOM_Mul(200, 100));
    printf("  [XOM_Div] 1000 / 250 = %d\n", XOM_Div(1000, 250));
    printf("  [XOM_Sum] 1 + 2 +..+ 10 = %d\n", XOM_Sum(ai32NumArray, sizeof(ai32NumArray) / sizeof(ai32NumArray[0])));

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
        printf("  [%04x] = 0x%08x\n", 0x00104000 + i * 4, M32(0x00104000 + i * 4));
    }

lexit:
    printf("Done.\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
