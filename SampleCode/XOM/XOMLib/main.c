/******************************************************************************
 * @file     main.c
 * @version  V3.01
 * @brief    Show how to build and provide the XOM library and
 *           configure/erase XOM regions for dual-bank operation.
 *           To ensure XOM functions work after bank remap,
 *           XOM regions must be configured at the corresponding offset in each bank.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "xomapi.h"

/* LOADER, XOM and XOM_LIB_DEMO regions should not overlap. */
#define LOADER_BASE         (FMC_APROM_BASE)
#define LOADER_SIZE         (FMC_FLASH_PAGE_SIZE * 2)
#define XOM_LIB_DEMO_BASE   (FMC_APROM_BASE + 0x8000)

/* Need to place xom.o in specified XOM region in linker script */
#define XOMR_PAGE_CNT       1
#define XOMR0_BASE          (LOADER_BASE + LOADER_SIZE)
/* Calculate the corresponding XOM offset in another bank */
#if (XOMR0_BASE > FMC_APROM_BANK1_BASE)
    #define XOMR1_BASE      ((XOMR0_BASE) - FMC_APROM_BANK_SIZE)
#else
    #define XOMR1_BASE      ((XOMR0_BASE) + FMC_APROM_BANK_SIZE)
#endif

int32_t SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*------------------------------------------------------------------------*/
    /* Init System Clock                                                      */
    /*------------------------------------------------------------------------*/
    CLK_SetBusClock(CLK_SCLKSEL_SCLKSEL_APLL0, CLK_APLLCTL_APLLSRC_HIRC, FREQ_220MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();
    CLK_EnableModuleClock(ISP0_MODULE);

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
    char        cUserSel;
    int32_t     i, i32Sum;
    int32_t     ai32NumArray[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    uint32_t    u32Status, u32Addr, u32Offset, u32ActBank, u32NewBank;
    uint32_t    u32Loader0ChkSum, u32Loader1ChkSum;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    /*
        This sample code is used to show how to build an XOM library.
        Sample code memory layout:
               Bank0                  Bank1
        --------------------   --------------------
        |      LOADER      |   |      LOADER      |
        |------------------|   |------------------|
        |     XOM0 Lib     |   |     XOM1 Lib     |
        |------------------|   |------------------|
        | XOMLibDemo_Bank0 |   | XOMLibDemo_Bank1 |
        --------------------   --------------------

        The location of XOM region is defined by linker file:
          XOMLib_Code.scatter(Keil)/XOMLib_Code.icf(IAR)/XOMLib_Code.ld(GCC/VSCode)
        Exported XOM library header file is .\lib\xomlib.h
        Exported XOM functions are implemented in xom.c

        This project is only used to build code for XOM region and test its functions.
        To enable XOM regions, please use "[0] Set XOM".

        Example flow:
        1. Build XOMLib_Code and test XOM functions.
        2. Execute "[0] Set XOM" in XOMLib_Code to set and enable XOM regions in dual bank.
        3. Test XOM function with XOM enabled again.
        4. Review xomlib.c and .\lib\xomlib.h to check all XOM function pointers are
           included correctly (Check function address in "[1] Test XOM" output).
           Copy XOMLib address definitions to xomlib.c manually.
        5. Build XOMLib project to generate xomlib.lib(Keil)/xomlib.a(IAR)/libXOMLib.a(GCC/VSCode).
           It includes function pointers for XOM.
           The XOM library (xomlib.lib, xomlib.a and libXOMLib.a) and header (xomlib.h)
           are located at lib directory.
        6. Pass xomlib.lib(Keil)/xomlib.a(IAR)/libXOMLib.a(GCC/VSCode) and xomlib.h to the users
           who will call these functions in XOM (e.g. XOMLibDemo).
        7. Build XOMLibDemo_Bank0 and XOMLibDemo_Bank1 and download to each bank
           with "NuMicro ICP Programming Tool".
    */

    printf("\n\n");
    printf("+-------------------------------------------+\n");
    printf("|  FMC Dual Bank XOM Library Build Example  |\n");
    printf("+-------------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function and enable APROM active*/
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();
    FMC_ENABLE_CFG_UPDATE();

    u32ActBank = FMC_GetBankIdx();
    printf("Bank%d is active.\n", u32ActBank);
    /* XOR with 1 toggles bank index between 0 and 1. */
    u32NewBank = u32ActBank ^ 1;

    u32Loader0ChkSum = FMC_GetChkSum(LOADER_BASE, LOADER_SIZE);
    u32Loader1ChkSum = FMC_GetChkSum(LOADER_BASE + FMC_APROM_BANK_SIZE, LOADER_SIZE);
    printf("Bank0 Loader checksum: 0x%08X.\nBank1 Loader checksum: 0x%08X.\n\n", u32Loader0ChkSum, u32Loader1ChkSum);

    /*
        Keep loader contents consistent in both banks.
        This sample only clones loader from Bank0 to Bank1 when mismatch is detected.
    */
    if ((u32ActBank == 0) && (u32Loader0ChkSum != u32Loader1ChkSum))
    {
        printf("Create Bank%d Loader... \n",  u32NewBank);

        /* Erase loader region in the other bank */
        for (u32Addr = LOADER_BASE; u32Addr < (LOADER_BASE + LOADER_SIZE); u32Addr += FMC_FLASH_PAGE_SIZE)
        {
            FMC_Erase(u32Addr + (FMC_APROM_BANK_SIZE * u32NewBank));
        }

        /* Create loader in the other bank */
        for (u32Addr = LOADER_BASE; u32Addr < (LOADER_BASE + LOADER_SIZE); u32Addr += 4)
        {
            FMC_Write(u32Addr + (FMC_APROM_BANK_SIZE * u32NewBank), FMC_Read(u32Addr + (FMC_APROM_BANK_SIZE * u32ActBank)));
        }

        printf("Create Bank%d Loader completed. \n\n", u32NewBank);
    }

    /* Read XOM Status */
    printf("XOM Status = 0x%X\n", FMC->XOMSTS);
    printf("[0] Set  XOM\n");
    printf("[1] Test XOM\n");
    printf("[2] Execute XOMLibDemo\n");

    cUserSel = (char)getchar();
    printf("Got %c\n", cUserSel);

    if (cUserSel == '0')
    {
        /* Option 0: Configure XOM in both banks. */
        /* Configure XOM1 at the corresponding offset in the other bank */
        if (FMC_GetXOMState(XOMR1) == 0)
        {
            /* Copy XOM code from XOM0 to XOM1 */
            for (u32Offset = 0; u32Offset < (XOMR_PAGE_CNT * FMC_FLASH_PAGE_SIZE); u32Offset += 4)
            {
                if ((u32Offset % FMC_FLASH_PAGE_SIZE) == 0)
                {
                    if (FMC_Erase(XOMR1_BASE + u32Offset) != FMC_OK)
                    {
                        printf("Failed to write 0x%08X !\n", (uint32_t)(XOMR1_BASE + u32Offset));

                        while (1) {};
                    }
                }

                if (FMC_Write(XOMR1_BASE + u32Offset, FMC_Read(XOMR0_BASE + u32Offset)) != FMC_OK)
                {
                    printf("Failed to write 0x%08X !\n", (uint32_t)(XOMR1_BASE + u32Offset));

                    while (1) {};
                }
            }

            printf("Config XOM1\n");
            printf("  Physical base address: 0x%08X, page count: %d\n", (uint32_t)XOMR1_BASE, (uint32_t)XOMR_PAGE_CNT);
            u32Status = FMC_ConfigXOM(XOMR1, XOMR1_BASE, XOMR_PAGE_CNT);

            if (u32Status)
            {
                printf("  XOM1 Config fail !\n");
            }
            else
            {
                printf("  XOM1 Config OK.\n");
            }
        }
        else
        {
            printf("[Info] XOM1 is already set. Please check if XOM1 contains correct XOM code.\n");
        }

        if (FMC_GetXOMState(XOMR0) == 0)
        {
            printf("Config XOM0\n");
            printf("  Physical base address: 0x%08X, page count: %d\n", (uint32_t)XOMR0_BASE, (uint32_t)XOMR_PAGE_CNT);
            u32Status = FMC_ConfigXOM(XOMR0, XOMR0_BASE, XOMR_PAGE_CNT);

            if (u32Status)
            {
                printf("  XOM0 Config fail !\n");
            }
            else
            {
                printf("  XOM0 Config OK.\n");
            }
        }
        else
        {
            printf("[Info] XOM0 is already set. Please check if XOM0 contains correct XOM code.\n");
        }

        UART_WAIT_TX_EMPTY(DEBUG_PORT);
        /* XOM setting is latched after reset; reboot to run with the new state. */
        SYS_ResetChip();

        while (1) {}
    }
    else if (cUserSel == '1')
    {
        /* Option 1: Validate XOM execution and print exported function addresses. */

        for (u32ActBank = 0; u32ActBank <= 1; u32ActBank++)
        {
            /* Remap target bank to the same virtual APROM window before testing. */
            if (FMC_RemapBank(u32ActBank) != FMC_OK)
            {
                printf("\n* Remap to Bank%d failed !\n", u32ActBank);

                /* Fatal in sample flow: stop here for debugging. */
                while (1) {};
            }
            else
            {
                u32ActBank = FMC_GetBankIdx();
                printf("\n* Bank%d is active.\n", u32ActBank);
            }

            /* Run XOM function */
            printf("Check XOM execution\n");

            /* If XOM is not enabled and region looks empty, skip this bank. */
            if ((FMC_GetXOMState(u32ActBank) == 0) && (M32(XOMR0_BASE) == 0xFFFFFFFF))
            {
                printf("XOM%d is not set. Skip execution check.\n", u32ActBank);
                continue;
            }

            printf("  [XOM_Add]  100 + 200 = %d\n", XOM_Add(100, 200));
            printf("  [XOM_Sub]  500 - 100 = %d\n", XOM_Sub(500, 100));
            printf("  [XOM_Mul]  200 * 100 = %d\n", XOM_Mul(200, 100));
            printf("  [XOM_Div] 1000 / 250 = %d\n", XOM_Div(1000, 250));
            printf("  [XOM_Sum] Sum of ai32NumArray = %d\n", XOM_Sum(ai32NumArray, sizeof(ai32NumArray) / sizeof(ai32NumArray[0])));

            for (i = 0; i < 1000; i++)
            {
                i32Sum = XOM_Add(500, 700);

                if (i32Sum != 1200)
                {
                    printf("XOM ADD fail. It should be 1200 but %d\n", i32Sum);
                    goto lexit;
                }
            }

            printf("\n");

            if (FMC->XOMSTS == 0x0U)
            {
                printf("Check CPU access XOM region as normal (not 0xFFFFFFFF).\n");
            }
            else
            {
                printf("Check CPU access XOM region all 0xFFFFFFFF.\n");
            }

            for (i = 0; i < 16; i++)
            {
                if ((i % 4) == 0)
                {
                    printf("\n  [0x%08X] ", (uint32_t)(XOMR0_BASE + i * 4));
                }

                printf("0x%08X ", M32(XOMR0_BASE + i * 4));
            }

            printf("\n");

            for (i = 0; i < 16; i++)
            {
                if ((i % 4) == 0)
                {
                    printf("\n  [0x%08X] ", (uint32_t)(XOMR0_BASE + (i * 4) + FMC_APROM_BANK_SIZE));
                }

                printf("0x%08X ", M32((XOMR0_BASE + (i * 4) + FMC_APROM_BANK_SIZE)));
            }

            printf("\n");
        }

        printf("\nCopy below XOM library address definitions to xomlib.c to build XOMLib.\n");
        printf("/*------------------------------*/\n");
        printf("/*  XOMLib address definitions  */\n");
        printf("/*------------------------------*/\n");
        printf("#define XOM_ADD_ADDR    0x%08X\n", (uint32_t)XOM_Add);
        printf("#define XOM_SUB_ADDR    0x%08X\n", (uint32_t)XOM_Sub);
        printf("#define XOM_MUL_ADDR    0x%08X\n", (uint32_t)XOM_Mul);
        printf("#define XOM_DIV_ADDR    0x%08X\n", (uint32_t)XOM_Div);
        printf("#define XOM_SUM_ADDR    0x%08X\n", (uint32_t)XOM_Sum);

    }
    else
    {
        /* Option 2: Jump to XOMLibDemo image after bank remap. */
        if ((FMC_Read(XOM_LIB_DEMO_BASE) == 0xFFFFFFFF) || (FMC_Read(XOM_LIB_DEMO_BASE + FMC_APROM_BANK_SIZE) == 0xFFFFFFFF))
        {
            printf("Please build XOMLibDemo_Bank0 and XOMLibDemo_Bank1 then download to each bank.\n");

            while (1) {}
        }

        printf("\nExecute XOMLibDemo on [0] Bank0 [1] Bank1 ?\n");
        u32NewBank = getchar();
        u32NewBank = (u32NewBank == '0') ? 0 : 1;

        if (FMC_RemapBank(u32NewBank) != FMC_OK)
        {
            printf("\n* Remap to Bank%d failed !\n", u32NewBank);

            while (1) {};
        }
        else
        {
            printf("\n* Bank%d is active.\n", FMC_GetBankIdx());
        }

        UART_WAIT_TX_EMPTY(DEBUG_PORT);
        /* Point vector table to app base in the currently remapped bank. */
        FMC_SetVectorPageAddr(XOM_LIB_DEMO_BASE);
        SYS_ResetCPU();
    }

lexit:
    printf("Done.\n");

    while (1) {}
}

/*** (C) COPYRIGHT 2026 Nuvoton Technology Corp. ***/
