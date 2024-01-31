/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Dual bank firmware update loader sample
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"
#include "NuDB_common.h"

static void SYS_Init(void)
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

    /* Enable module clock */
    CLK_EnableModuleClock(ISP0_MODULE);
    CLK_EnableModuleClock(CRC0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();
}

// [Begin] TESTCHIP_ONLY - This function should be removed in M55M1.
uint32_t FMC_GetChkSum(uint32_t u32StartAddr, uint32_t u32ByteSize)
{
    uint32_t u32CRC32Checksum = 0xFFFFFFFF;

    /* Configure CRC controller for CRC-CRC32 mode */
    CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFFul, CRC_CPU_WDATA_32);
    CRC_SET_DMA_SADDR(CRC, u32StartAddr);
    CRC_SET_DMACNT_WORD(CRC, u32ByteSize / 4);
    CRC_DMA_START(CRC);

    while ((CRC_GET_STATUS(CRC) & CRC_DMASTS_FINISH_Msk) == 0)
        ;

    CRC->DMASTS = CRC_DMASTS_FINISH_Msk;
    u32CRC32Checksum = CRC_GetChecksum();

    return u32CRC32Checksum;
}
// [End] TESTCHIP_ONLY

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
    uint8_t  u8GetCh;
    uint32_t u32Addr, u32ExecBank, u32UpdateBank;
    uint32_t u32Loader0ChkSum, u32Loader1ChkSum;
    uint32_t u32App0ChkSum, u32App1ChkSum;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("+-------------------------------------------+\n");
    printf("| M55M1 FMC Dual Bank FW Update Sample Code |\n");
    printf("+-------------------------------------------+\n");

    /* Disable register write-protection function */
    SYS_UnlockReg();
    /* Enable ISP and APROM update */
    FMC_ENABLE_ISP();
    FMC_ENABLE_AP_UPDATE();

    while (1)
    {
        printf("\n\n");
        printf("+------------------------+\n");
        printf("|  Boot from 0x%08X  |\n", FMC_GetVECMAP());
        printf("+------------------------+\n");

        u32ExecBank = (uint32_t)((FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) >> FMC_ISPSTS_FBS_Pos);
        printf("\n BANK%d Loader processing \n\n", u32ExecBank);
        u32UpdateBank = u32ExecBank ^ 1;

        u32Loader0ChkSum = FMC_GetChkSum(LOADER_BASE, LOADER_SIZE);
        u32Loader1ChkSum = FMC_GetChkSum(FMC_APROM_BANK_SIZE + LOADER_BASE, LOADER_SIZE);
        printf(" Loader0 checksum: 0x%08x \n Loader1 checksum: 0x%08x\n", u32Loader0ChkSum, u32Loader1ChkSum);

        u32App0ChkSum = FMC_GetChkSum(APP_BASE, APP_SIZE);
        u32App1ChkSum = FMC_GetChkSum(FMC_APROM_BANK_SIZE + APP_BASE, APP_SIZE);
        printf(" App0 checksum: 0x%08x \n App1 checksum: 0x%08x\n", u32App0ChkSum, u32App1ChkSum);

        if ((u32ExecBank == 0) && (u32Loader0ChkSum != u32Loader1ChkSum))
        {
            printf("\n Create BANK%d Loader... \n",  u32UpdateBank);

            /* Erase loader region */
            for (u32Addr = LOADER_BASE; u32Addr < (LOADER_BASE + LOADER_SIZE); u32Addr += FMC_FLASH_PAGE_SIZE)
            {
                FMC_Erase(FMC_APROM_BANK_SIZE * u32UpdateBank + u32Addr);
            }

            /* Create loader in the other bank */
            for (u32Addr = LOADER_BASE; u32Addr < (LOADER_BASE + LOADER_SIZE); u32Addr += 4)
            {
                FMC_Write(FMC_APROM_BANK_SIZE * u32UpdateBank + u32Addr, FMC_Read((FMC_APROM_BANK_SIZE * u32ExecBank) + u32Addr));
            }

            printf(" Create Bank%d Loader completed. \n", (u32ExecBank ^ 1));
        }

        if ((u32ExecBank == 0) && ((CheckAllOne((FMC_APROM_BANK_SIZE + APP_BASE), APP_SIZE)) == READ_ALLONE_YES))
        {
            printf("\n Create BANK%d App... \n", u32UpdateBank);

            /* Erase app region */
            for (u32Addr = APP_BASE; u32Addr < (APP_BASE + APP_SIZE); u32Addr += FMC_FLASH_PAGE_SIZE)
            {
                FMC_Erase(FMC_APROM_BANK_SIZE * u32UpdateBank + u32Addr);
            }

            /* Create app in the other bank(just for test)*/
            for (u32Addr = APP_BASE; u32Addr < (APP_BASE + APP_SIZE); u32Addr += 4)
            {
                FMC_Write(FMC_APROM_BANK_SIZE * u32UpdateBank + u32Addr, FMC_Read((FMC_APROM_BANK_SIZE * u32ExecBank) + u32Addr));
            }

            printf(" Create Bank%d App completed. \n", u32UpdateBank);
        }

        printf("\n Execute BANK%d APP ? [y/n]\n", u32ExecBank);
        u8GetCh = (uint8_t)getchar();

        if (u8GetCh == 'y')
        {
            /* Remap to App */
            FMC_SetVectorPageAddr(APP_BASE);
            SYS_ResetCPU();
        }
        else
        {
            printf("\n Remap to BANK%d Loader ? [y/n]\n", u32UpdateBank);
            u8GetCh = (uint8_t)getchar();

            if (u8GetCh == 'y')
            {
                /* Remap Bank */
                printf("\n BANK%d Loader before remap \n", u32ExecBank);
                FMC_RemapBank(u32ExecBank ^ 1);
                printf("\n Remap completed.\n");
            }
            else
            {
                printf("\n Continue to execute BANK%d Loader ? [y/n]\n ", u32ExecBank);
            }
        }
    }
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
