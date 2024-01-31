/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    An example of using FMC driver to set up and erase XOM regions.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"

/* Need to place xom_add.o in specified XOM region in linker script */
#define XOMR0_Base    (FMC_APROM_BASE + 0x10000)

extern int32_t Lib_XOM_ADD(uint32_t a, uint32_t b);

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable PLL0 180MHz clock from HIRC and switch SCLK clock source to PLL0 */
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

uint32_t CheckAllOne(uint32_t u32StartAddr, uint32_t u32ByteSize)
{
    uint32_t u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < (u32StartAddr + u32ByteSize); u32Addr += 4)
    {
        if (inp32(u32Addr) != 0xFFFFFFFF)
            return READ_ALLONE_NOT;
    }

    return READ_ALLONE_YES;
}

int main()
{
    uint32_t    u32Status;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    /*
     *   This sample code is used to show how to use StdDriver API to enable/erase XOM.
     */
    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|  FMC XOM config & erase  Sample Code   |\n");
    printf("+----------------------------------------+\n");

    SYS_UnlockReg();            /* Unlock protected registers */
    FMC_Open();                 /* Enable FMC ISP function    */
    FMC_ENABLE_AP_UPDATE();     /* Enable APROM update        */
    FMC_ENABLE_CFG_UPDATE();    /* Enable APROM update        */

    /* Read User Configuration */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_USER_CONFIG_0));

    if ((FMC_GetXOMState(XOMR0) == 0) &&
            (CheckAllOne(XOMR0_Base, FMC_FLASH_PAGE_SIZE) == READ_ALLONE_YES))
    {
        printf("XOM0 region erased. No program code in XOM0.\n");
        printf("Demo completed. Please re-program flash if you want to run again.\n");

        while (1);
    }

    printf("XOM Status = 0x%X\n", FMC->XOMSTS);
    printf("Any key to continue...\n");
    getchar();

    /* Config XOMR0 */
    if (FMC_GetXOMState(XOMR0) == 0)
    {
        u32Status = FMC_ConfigXOM(XOMR0, XOMR0_Base, 1);

        if (u32Status)
            printf("XOMR0 Config fail...\n");
        else
            printf("XOMR0 Config OK...\n");

        printf("\nAny key to reset chip to enable XOM regions...\n");
        getchar();

        /* Reset chip to enable XOM region. */
        SYS_ResetChip();

        while (1) {};
    }

    /* Run XOM function */
    printf("\n");
    printf(" 100 + 200 = %d\n", Lib_XOM_ADD(100, 200));

    printf("\nXOMR0 active success....\n");
    printf("\nAny key to Erase XOM...\n");
    getchar();

    if (FMC_GetXOMState(XOMR0) == 1)
    {
        /* Erase XOMR0 region */
        if (FMC_EraseXOM(XOMR0) == 0)
            printf("Erase XOMR0....OK\n");
        else
            printf("Erase XOMR0....Fail\n");
    }

    printf("Done.\n");
    printf("Please press reset to check if XOM0 empty.\n");

    while (1);
}
