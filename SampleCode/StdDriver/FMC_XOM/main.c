/******************************************************************************
 * @file     main.c
 * @version  V3.01
 * @brief    An example of using FMC APIs to configure/erase XOM regions for
 *           dual-bank operation. To ensure XOM functions work after bank remap,
 *           XOM regions must be configured at the corresponding offset in each bank.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include "NuMicro.h"

#define LOADER_BASE         (FMC_APROM_BASE)
#define LOADER_SIZE         (FMC_FLASH_PAGE_SIZE * 8)

/* Need to place xom_add.o in specified XOM region in linker script */
#define XOMR_PAGE_CNT       1
#define XOMR0_BASE          (LOADER_BASE + LOADER_SIZE)
/* Calculate the corresponding XOM offset in another bank */
#if (XOMR0_BASE > FMC_APROM_BANK1_BASE)
    #define XOMR1_BASE      ((XOMR0_BASE) - FMC_APROM_BANK_SIZE)
#else
    #define XOMR1_BASE      ((XOMR0_BASE) + FMC_APROM_BANK_SIZE)
#endif

extern int32_t Lib_XOM_ADD(uint32_t a, uint32_t b);

int32_t SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
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

int main(void)
{
    uint32_t    u32Status, u32Addr, u32Offset, u32ActBank, u32NewBank;
    uint32_t    u32Loader0ChkSum, u32Loader1ChkSum;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    /*
     *   This sample code is used to show how to use FMC APIs to enable/erase XOM.
     */
    printf("\n\n");
    printf("+------------------------------------------------+\n");
    printf("|  FMC Dual Bank XOM Config & Erase Sample Code  |\n");
    printf("+------------------------------------------------+\n");

    SYS_UnlockReg();            /* Unlock protected registers */
    FMC_Open();                 /* Enable FMC ISP function    */
    FMC_ENABLE_AP_UPDATE();     /* Enable APROM update        */
    FMC_ENABLE_CFG_UPDATE();    /* Enable Config update       */

    u32ActBank = FMC_GetBankIdx();
    printf("Bank%d is active.\n", u32ActBank);
    u32NewBank = u32ActBank ^ 1;

    u32Loader0ChkSum = FMC_GetChkSum(LOADER_BASE, LOADER_SIZE);
    u32Loader1ChkSum = FMC_GetChkSum(LOADER_BASE + FMC_APROM_BANK_SIZE, LOADER_SIZE);
    printf("Bank0 Loader checksum: 0x%08X.\nBank1 Loader checksum: 0x%08X.\n\n", u32Loader0ChkSum, u32Loader1ChkSum);

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

    if ((FMC_GetXOMState(XOMR0) == 0) &&
            (FMC_CheckAllOne(XOMR0_BASE, (XOMR_PAGE_CNT * FMC_FLASH_PAGE_SIZE)) == READ_ALLONE_YES))
    {
        printf("XOM0 region erased. No program code in XOM0.\n");

        if ((FMC_GetXOMState(XOMR1) == 0) &&
                (FMC_CheckAllOne(XOMR1_BASE, (XOMR_PAGE_CNT * FMC_FLASH_PAGE_SIZE)) == READ_ALLONE_YES))
        {
            printf("XOM1 region erased. No program code in XOM1.\n");
            printf("Demo completed.\nPlease re-program flash if you want to run again.\n");

            while (1);
        }
    }

    printf("XOM Status = 0x%X\n", FMC->XOMSTS);
    printf("Press any key to continue ...\n");
    getchar();

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

        u32Status = FMC_ConfigXOM(XOMR1, XOMR1_BASE, XOMR_PAGE_CNT);

        if (u32Status)
            printf("XOM1 Config fail !\n");
        else
            printf("XOM1 Config OK.\n");
    }
    else
    {
        // Check CPU data access in XOM region should read all 0xFFFFFFFF
        printf("Check XOM1 [0x%08X ~ 0x%08X] all 0xFFFFFFFF.\n", (uint32_t)XOMR1_BASE, (uint32_t)(XOMR1_BASE + (XOMR_PAGE_CNT * FMC_FLASH_PAGE_SIZE)));

        for (u32Addr = XOMR1_BASE; u32Addr < (XOMR1_BASE + (XOMR_PAGE_CNT * FMC_FLASH_PAGE_SIZE)); u32Addr += 4)
        {
            if (M32(u32Addr) != 0xFFFFFFFF)
            {
                printf("  Read 0x%08X not 0xFFFFFFFF but 0x%08X !\n", u32Addr, M32(u32Addr));
                break;
            }
        }
    }

    /* Config XOM0 */
    if (FMC_GetXOMState(XOMR0) == 0)
    {
        u32Status = FMC_ConfigXOM(XOMR0, XOMR0_BASE, XOMR_PAGE_CNT);

        if (u32Status)
            printf("XOM0 Config fail !\n");
        else
            printf("XOM0 Config OK.\n");
    }
    else
    {
        // Check CPU data access in XOM region should read all 0xFFFFFFFF
        printf("Check XOM0 [0x%08X ~ 0x%08X] all 0xFFFFFFFF.\n", (uint32_t)XOMR0_BASE, (uint32_t)(XOMR0_BASE + (XOMR_PAGE_CNT * FMC_FLASH_PAGE_SIZE)));

        for (u32Addr = XOMR0_BASE; u32Addr < (XOMR0_BASE + (XOMR_PAGE_CNT * FMC_FLASH_PAGE_SIZE)); u32Addr += 4)
        {
            if (M32(u32Addr) != 0xFFFFFFFF)
            {
                printf("  Read 0x%08X not 0xFFFFFFFF but 0x%08X !\n", u32Addr, M32(u32Addr));
                break;
            }
        }
    }

    /* Reset chip to enable XOM region. */
    if ((FMC_GetXOMState(XOMR0) == 0) || (FMC_GetXOMState(XOMR1) == 0))
    {
        printf("\nPress any key to reset chip to enable XOM0 and XOM1 region ...\n");
        getchar();
        /* Reset chip to enable XOM region. */
        SYS_ResetChip();

        while (1) {};
    }

    printf("\n* Bank%d is active.\n", FMC_GetBankIdx());
    /* Run XOM function in XOM0 */
    printf("Lib_XOM_ADD: 0x%08X\n", (uint32_t)Lib_XOM_ADD);
    printf("  100 + 200 = %d\n", Lib_XOM_ADD(100, 200));
    printf("XOMR0 active success.\n");

    if (FMC_RemapBank(u32NewBank) != FMC_OK)
    {
        printf("\n* Remap to Bank%d failed !\n", u32NewBank);

        while (1) {};
    }
    else
    {
        printf("\n* Bank%d is active.\n", FMC_GetBankIdx());
    }

    /* Run XOM function in XOM1 */
    printf("Lib_XOM_ADD: 0x%08X\n", (uint32_t)Lib_XOM_ADD);
    printf("  123 + 234 = %d\n", Lib_XOM_ADD(123, 234));
    printf("XOMR1 active success.\n");

    printf("\nPress any key to erase XOM0 and XOM1 ...\n");
    getchar();

    if (FMC_GetXOMState(XOMR0) == 1)
    {
        /* Erase XOM0 region */
        if (FMC_EraseXOM(XOMR0) == 0)
            printf("Erase XOM0 ... OK.\n");
        else
            printf("Erase XOM0 ... Fail !\n");
    }

    if (FMC_GetXOMState(XOMR1) == 1)
    {
        /* Erase XOM1 region */
        if (FMC_EraseXOM(XOMR1) == 0)
            printf("Erase XOM1 ... OK.\n");
        else
            printf("Erase XOM1 ... Fail !\n");
    }

    printf("Done.\n");
    printf("Please reset chip to check if XOM0 and XOM1 are empty.\n");

    while (1) {};
}

/*** (C) COPYRIGHT 2026 Nuvoton Technology Corp. ***/
