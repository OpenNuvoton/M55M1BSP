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

int FMC_Proc(uint32_t u32Cmd, uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t *pu32DataBuf);

int FMC_Proc(uint32_t u32Cmd, uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t *pu32DataBuf)
{
    uint32_t u32Addr, Reg;
    uint32_t u32TimeOutCount;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; pu32DataBuf++, u32Addr += 4)
    {
        FMC->ISPADDR = u32Addr;

        if ((u32Addr & (FMC_FLASH_PAGE_SIZE - 1)) == 0 && u32Cmd == FMC_ISPCMD_PROGRAM)
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

        FMC->ISPCMD = u32Cmd;

        if (u32Cmd == FMC_ISPCMD_PROGRAM)
        {
            FMC->ISPDAT = *pu32DataBuf;
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

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
