/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Show FMC read Flash IDs, erase, read, and write functions
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#define APROM_TEST_BASE             (FMC_APROM_BASE + 0x20000)                      /* APROM test start address */
#define APROM_TEST_END              (APROM_TEST_BASE + FMC_FLASH_PAGE_SIZE * 8)     /* APROM test end address   */
#define TEST_PATTERN                0x5A5A5A5A                                      /* Test pattern             */

void SYS_Init(void)
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}


int32_t FillDataPattern(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t u32Addr;                  /* flash address */

    /* Fill flash range from u32StartAddr to u32EndAddr with data word u32Pattern. */
    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        if (FMC_Write(u32Addr, u32Pattern) != 0)          /* Program flash */
        {
            printf("FMC_Write address 0x%x failed!\n", u32Addr);
            return -1;
        }
    }

    return 0;
}

int32_t  VerifyData(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;               /* Flash address */
    uint32_t    u32data;               /* Flash data    */

    /* Verify if each data word from flash u32StartAddr to u32EndAddr be u32Pattern.  */
    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        u32data = FMC_Read(u32Addr);   /* Read a flash word from address u32Addr. */

        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_Read address 0x%x failed!\n", u32Addr);
            return -1;
        }

        if (u32data != u32Pattern)     /* Verify if data matched. */
        {
            printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32data, u32Pattern);
            return -1;                 /* Data verify failed */
        }
    }

    return 0;                          /* Data verify OK */
}

int32_t  FlashTest(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;               /* Flash address */

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("    Flash test address: 0x%x    \r", u32Addr);       /* Information message */

        if (FMC_Erase(u32Addr) != 0)            /* Erase page */
        {
            printf("FMC_Erase address 0x%x failed!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all 0xFFFFFFFF */
        if (VerifyData(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);   /* Error message */
            return -1;                 /* Erase verify failed */
        }

        /* Write test pattern to fill the whole page. */
        if (FillDataPattern(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) != 0)
            return -1;

        /* Verify if page contents are all equal to test pattern */
        if (VerifyData(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("\nData verify failed!\n ");                      /* Error message */
            return -1;                 /* Program verify failed */
        }

        /* Erase page */
        if (FMC_Erase(u32Addr) != 0)
        {
            printf("FMC_Erase address 0x%x failed!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all 0xFFFFFFFF after erased. */
        if (VerifyData(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);   /* Error message */
            return -1;                 /* Erase verify failed */
        }
    }

    printf("\r    Flash Test Passed.          \n");                  /* Information message */
    return 0;      /* Flash test passed */
}

int main(void)
{
    uint32_t    i, u32Data;            /* variables */

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("+------------------------------------------+\n");
    printf("|     M55M1 FMC Read/Write Sample Code     |\n");
    printf("+------------------------------------------+\n");
    printf("FMC->ISPCTL: 0x%08X\n", FMC->ISPCTL);
    SYS_UnlockReg();    /* Unlock protected registers */
    FMC_Open();         /* Enable FMC ISP function */

    /* Get booting source (APROM/LDROM) */
    printf("  Boot Mode ............................. ");

    if (FMC_GetBootSource() == 0)
        printf("[APROM]\n");           /* Information message */
    else
    {
        printf("[LDROM]\n");           /* Information message */
        printf("  WARNING: This driver sample code must execute in APROM !\n");
        goto lexit;                    /* Failed to get boot source */
    }

    u32Data = FMC_ReadCID();           /* Get company ID, should be 0xDA. */

    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadCID failed!\n");
        goto lexit;
    }

    printf("  Company ID ............................ [0x%08x]\n", u32Data);   /* Information message */

    u32Data = FMC_ReadPID();           /* Get product ID. */

    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadPID failed!\n");
        goto lexit;
    }

    printf("  Product ID ............................ [0x%08x]\n", u32Data);   /* Information message */

    for (i = 0; i < 3; i++)            /* Get 96-bits UID. */
    {
        u32Data = FMC_ReadUID(i);

        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_ReadUID %d failed!\n", i);
            goto lexit;
        }

        printf("  Unique ID %d ........................... [0x%08x]\n", i, u32Data);  /* Information message */
    }

    for (i = 0; i < 4; i++)            /* Get 4 words UCID. */
    {
        u32Data = FMC_ReadUCID(i);

        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_ReadUCID %d failed!\n", i);
            goto lexit;
        }

        printf("  Unique Customer ID %d .................. [0x%08x]\n", i, u32Data);  /* Information message */
    }

    /* Read User Configuration CONFIG0 */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));

    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE) failed!\n");
        goto lexit;
    }

    /* Read User Configuration CONFIG1 */
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE + 4));

    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE+4) failed!\n");
        goto lexit;
    }

    printf("\n\nLDROM test =>\n");     /* Information message */

    FMC_ENABLE_LD_UPDATE();            /* Enable LDROM update. */

    /* Execute flash program/verify test on LDROM. */
    if (FlashTest(FMC_LDROM_BASE, FMC_LDROM_END, TEST_PATTERN) < 0)
    {
        printf("\n\nLDROM test failed!\n");        /* error message */
        goto lexit;                    /* LDROM test failed. Program aborted. */
    }

    FMC_DISABLE_LD_UPDATE();           /* Disable LDROM update. */

    printf("\n\nAPROM test =>\n");     /* Information message */

    FMC_ENABLE_AP_UPDATE();            /* Enable APROM update. */

    /* Execute flash program/verify test on APROM. */
    if (FlashTest(APROM_TEST_BASE, APROM_TEST_BASE + FMC_FLASH_PAGE_SIZE * 8, TEST_PATTERN) < 0)
    {
        printf("\n\nAPROM test failed!\n");        /* Information message */
        goto lexit;                                /* LDROM test failed. Program aborted. */
    }

    FMC_DISABLE_AP_UPDATE();           /* Disable APROM update. */

lexit:
    FMC_Close();                       /* Disable FMC ISP function */
    SYS_LockReg();                     /* Lock protected registers */

    printf("\nEnd of FMC Read/Write Sample Code\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
