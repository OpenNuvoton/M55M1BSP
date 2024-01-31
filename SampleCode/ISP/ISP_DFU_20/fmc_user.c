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

#define FMC_BLOCK_SIZE           (FMC_FLASH_PAGE_SIZE * 4UL)

int FMC_Proc(uint32_t u32Cmd, uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t *pu32DataBuf);

int FMC_Proc(uint32_t u32Cmd, uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t *pu32DataBuf)
{
    uint32_t u32Addr, Reg;
    uint32_t u32TimeOutCount;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        FMC->ISPADDR = u32Addr;

        if ((u32Addr & (FMC_FLASH_PAGE_SIZE - 1)) == 0 && u32Cmd == FMC_ISPCMD_PROGRAM)
        {
            if ((u32Addr & FMC_CONFIG_BASE) != FMC_CONFIG_BASE)
            {
                FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
                FMC->ISPTRG = 0x1;

                u32TimeOutCount = SystemCoreClock; /* 1 second time-out */

                while (FMC->ISPTRG & 0x1)
                {
                    if (--u32TimeOutCount == 0)
                        return -1;
                }
            }
        }

        FMC->ISPCMD = u32Cmd;

        if (u32Cmd == FMC_ISPCMD_PROGRAM)
        {
            FMC->ISPDAT = *pu32DataBuf;
            pu32DataBuf++;     /* Prevent Null pointer addition error in cppcheck */
        }

        FMC->ISPTRG = 0x1;
        //__ISB();

        /* Wait ISP cmd complete */
        u32TimeOutCount = SystemCoreClock; /* 1 second time-out */

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
        {
            if (--u32TimeOutCount == 0)
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
            *pu32DataBuf = FMC->ISPDAT;
            pu32DataBuf++;     /* Prevent Null pointer addition error in cppcheck */
        }
    }

    return 0;
}

/**
 * @brief       Read 32-bit Data from specified address of flash
 *
 * @param[in]   u32addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 *
 * @return      The pu32DataBuf of specified address
 *
 * @details     To read word pu32DataBuf from Flash include APROM, LDROM, Data Flash, and CONFIG.
 *
 * @note
 *              Please make sure that Register Write-Protection Function has been disabled
 *              before using this function.
 */
int FMC_Read_User(uint32_t u32Addr, uint32_t *pu32DataBuf)
{
    return FMC_Proc(FMC_ISPCMD_READ, u32Addr, u32Addr + 4, pu32DataBuf);
}

/**
 * @brief      Flash page erase
 *
 * @param[in]  u32addr  Flash address including APROM, LDROM, Data Flash, and CONFIG
 *
 * @details    To do flash page erase. The target address could be APROM, LDROM, Data Flash, or CONFIG.
 *             The page u32ByteSize is 512 bytes.
 *
 * @note
 *             Please make sure that Register Write-Protection Function has been disabled
 *             before using this function.
 */
int FMC_Erase_User(uint32_t u32Addr)
{
    return FMC_Proc(FMC_ISPCMD_PAGE_ERASE, u32Addr, u32Addr + 4, 0);
}

void ReadData(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t *pu32DataBuf)    // Read pu32DataBuf from flash
{
    FMC_Proc(FMC_ISPCMD_READ, u32StartAddr, u32EndAddr, pu32DataBuf);
    return;
}

void WriteData(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t *pu32DataBuf)  // Write pu32DataBuf into flash
{
    FMC_Proc(FMC_ISPCMD_PROGRAM, u32StartAddr, u32EndAddr, pu32DataBuf);
    return;
}


int EraseAP(uint32_t u32StartAddr, uint32_t u32ByteSize)
{
    uint32_t u32Addr, u32Cmd, u32Size;
    int32_t i32Size;
    uint32_t u32TimeOutCount = FMC_TIMEOUT_ERASE;

    u32Addr = u32StartAddr;
    i32Size = (int32_t)u32ByteSize;

    while (i32Size > 0)
    {
        if ((u32ByteSize >= FMC_APROM_BANK_SIZE) && !(u32Addr & (FMC_APROM_BANK_SIZE - 1)))
        {
            u32Cmd  = FMC_ISPCMD_BANK_ERASE;
            u32Size = FMC_APROM_BANK_SIZE;
        }
        else
        {
            u32Cmd  = FMC_ISPCMD_PAGE_ERASE;
            u32Size = FMC_FLASH_PAGE_SIZE;
        }

        FMC->ISPCMD  = u32Cmd;
        FMC->ISPADDR = u32Addr;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;
        __ISB();

        while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)  /* Wait for ISP command done. */
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
        u32ByteSize -= u32Size;
        i32Size = (int32_t)u32ByteSize;
    }

    return 0;
}

void UpdateConfig(uint32_t *pu32DataBuf, uint32_t *res)
{
    uint32_t u32Size = (FMC_CONFIG_CNT * 4);

    FMC_ENABLE_CFG_UPDATE();
    FMC_Proc(FMC_ISPCMD_CFG_ERASE, Config0, Config0 + 8, 0);
    FMC_Proc(FMC_ISPCMD_PROGRAM, Config0, Config0 + u32Size, pu32DataBuf);

    if (res)
    {
        FMC_Proc(FMC_ISPCMD_READ, Config0, Config0 + u32Size, res);
    }

    FMC_DISABLE_CFG_UPDATE();
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
