/***************************************************************************//**
 * @file     fmc_user.c
 * @version  V1.00
 * @brief    Simplified FMC driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "fmc_user.h"

int32_t g_FMC_i32ErrCode = 0;
#define FMC_BLOCK_SIZE           (FMC_FLASH_PAGE_SIZE * 4UL)

int FMC_Proc(uint32_t u32Cmd, uint32_t u32AddrStart, uint32_t u32AddrEnd, uint32_t *pu32Data)
{
    uint32_t u32Addr, Reg;
    uint32_t u32TimeOutCount = SystemCoreClock;

    for (u32Addr = u32AddrStart; u32Addr < u32AddrEnd;)
    {
        FMC->ISPCMD = u32Cmd;
        FMC->ISPADDR = u32Addr;

        if (u32Cmd == FMC_ISPCMD_PROGRAM)
        {
            FMC->ISPDAT = *pu32Data;
            pu32Data++;     /* Prevent Null pointer addition error in cppcheck */
        }

        FMC->ISPTRG = 0x1;
        __ISB();

        /* Wait ISP cmd complete */
        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
        {
            if (--u32TimeOutCount == 0) /* 1 second time-out */
                return -1;
        }

        Reg = FMC->ISPCTL;

        if (Reg & FMC_ISPCTL_ISPFF_Msk)
        {
            FMC->ISPCTL = Reg;
            return -1;
        }

        if (u32Cmd == FMC_ISPCMD_READ)
        {
            *pu32Data = FMC->ISPDAT;
            pu32Data++;     /* Prevent Null pointer addition error in cppcheck */
        }

        if (u32Cmd == FMC_ISPCMD_PAGE_ERASE)
        {
            u32Addr += FMC_FLASH_PAGE_SIZE;
        }
        else
        {
            u32Addr += 4;
        }
    }

    return 0;
}

/**
 * @brief      Program 32-bit data into specified address of flash
 *
 * @param[in]  u32Addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @param[in]  u32Data  32-bit Data to program
 *
 * @details    To program word data into Flash include APROM, LDROM, Data Flash, and CONFIG.
 *             The corresponding functions in CONFIG are listed in FMC section of TRM.
 *
 * @note
 *             Please make sure that Register Write-Protection Function has been disabled
 *             before using this function. User can check the status of
 *             Register Write-Protection Function with SYS_IsRegLocked().
 */
int FMC_Write_User(uint32_t u32Addr, uint32_t u32Data)
{
    return FMC_Proc(FMC_ISPCMD_PROGRAM, u32Addr, u32Addr + 4, &u32Data);
}

/**
 * @brief       Read 32-bit Data from specified address of flash
 *
 * @param[in]   u32Addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 *
 * @return      The data of specified address
 *
 * @details     To read word data from Flash include APROM, LDROM, Data Flash, and CONFIG.
 *
 * @note
 *              Please make sure that Register Write-Protection Function has been disabled
 *              before using this function. User can check the status of
 *              Register Write-Protection Function with SYS_IsRegLocked().
 */
int FMC_Read_User(uint32_t u32Addr, uint32_t *pu32Data)
{
    return FMC_Proc(FMC_ISPCMD_READ, u32Addr, u32Addr + 4, pu32Data);
}

/**
 * @brief      Flash page erase
 *
 * @param[in]  u32Addr  Flash address including APROM, LDROM, Data Flash, and CONFIG
 *
 * @details    To do flash page erase. The target address could be APROM, LDROM, Data Flash, or CONFIG.
 *             The page size is 512 bytes.
 *
 * @note
 *             Please make sure that Register Write-Protection Function has been disabled
 *             before using this function. User can check the status of
 *             Register Write-Protection Function with SYS_IsRegLocked().
 */
int FMC_Erase_User(uint32_t u32Addr)
{
    return FMC_Proc(FMC_ISPCMD_PAGE_ERASE, u32Addr, u32Addr + 4, 0);
}

void ReadData(uint32_t u32AddrStart, uint32_t u32AddrEnd, uint32_t *pu32Data)    // Read data from flash
{
    FMC_Proc(FMC_ISPCMD_READ, u32AddrStart, u32AddrEnd, pu32Data);
    return;
}

void WriteData(uint32_t u32AddrStart, uint32_t u32AddrEnd, uint32_t *pu32Data)  // Write data into flash
{
    FMC_Proc(FMC_ISPCMD_PROGRAM, u32AddrStart, u32AddrEnd, pu32Data);
    return;
}


int EraseAP(uint32_t u32AddrStart, uint32_t size)
{
    uint32_t u32Addr, u32Cmd, u32Size;
    uint32_t u32TimeOutCount = FMC_TIMEOUT_ERASE;
    u32Addr = u32AddrStart;

    while (size > 0)
    {
        if ((size >= FMC_APROM_BANK_SIZE) && !(u32Addr & (FMC_APROM_BANK_SIZE - 1)))
        {
            u32Cmd = FMC_ISPCMD_BANK_ERASE;
            u32Size = FMC_APROM_BANK_SIZE;
        }
        else
        {
            u32Cmd = FMC_ISPCMD_PAGE_ERASE;
            u32Size = FMC_FLASH_PAGE_SIZE;
        }

        FMC->ISPCMD = u32Cmd;
        FMC->ISPADDR = u32Addr;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        __ISB();

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)   /* Wait for ISP command done. */
        {
            if (--u32TimeOutCount == 0)
                return -1;
        }

        if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
        {
            FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
            return -1;
        }

        u32Addr += u32Size;
        size -= u32Size;
    }

    return 0;
}

void UpdateConfig(uint32_t *pu32Data, uint32_t *pu32Resp)
{
    uint32_t i;

    FMC_ENABLE_CFG_UPDATE();

    for (i = 0; i < FMC_CONFIG_CNT; i++)
    {
        FMC_Proc(FMC_ISPCMD_CFG_ERASE, Config0 + (i * 4), Config0 + ((i + 1) * 4), 0);
        FMC_Proc(FMC_ISPCMD_PROGRAM,   Config0 + (i * 4), Config0 + ((i + 1) * 4), pu32Data + i);
    }

    if (pu32Resp)
    {
        /* Read back to response */
        FMC_Proc(FMC_ISPCMD_READ, Config0, Config0 + (FMC_CONFIG_CNT * 4), pu32Resp);
    }

    FMC_DISABLE_CFG_UPDATE();
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
