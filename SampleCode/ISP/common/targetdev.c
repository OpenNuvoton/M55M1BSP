/***************************************************************************//**
 * @file     targetdev.c
 * @version  V1.00
 * @brief    ISP support function source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "targetdev.h"
#include "isp_user.h"

uint32_t g_u32ApromSize;

void UserMemCopy(uint8_t pu8Dest[], uint8_t pu8Src[], uint32_t u32Size)
{
    uint32_t i = 0ul;

    while (u32Size--)
    {
        pu8Dest[i] = pu8Src[i];
        __DSB();
        i++;
    }
}

/* Supports maximum 2MB (APROM) */
uint32_t GetApromSize(void)
{
    /* The smallest of APROM u32Size is FMC_FLASH_PAGE_SIZE. */
    uint32_t u32Size = FMC_FLASH_PAGE_SIZE, u32Data;
    int result;

    do
    {
        result = FMC_Read_User(FMC_APROM_BASE + u32Size, &u32Data);

        if (result < 0)
        {
            return u32Size;
        }
        else
        {
            u32Size *= 2;
        }
    } while (1);
}

#if 0   /* M55M1 did not support Data Flash */
/* Data Flash is shared with APROM.
   The size and start address are defined in CONFIG1. */
void GetDataFlashInfo(uint32_t *pu32Addr, uint32_t *pu32Size)
{
    uint32_t u32Data;
    *pu32Size = 0;
    FMC_Read_User(Config0, &u32Data);

    if ((u32Data & 0x01) == 0)   /* DFEN enable */
    {
        FMC_Read_User(Config1, &u32Data);

        /* Filter the reserved bits in CONFIG1 */
        u32Data &= 0x000FFFFF;

        if (u32Data > g_u32ApromSize || u32Data & (FMC_FLASH_PAGE_SIZE - 1))   /* Avoid config1 value from error */
        {
            u32Data = g_u32ApromSize;
        }

        *pu32Addr = u32Data;
        *pu32Size = g_u32ApromSize - u32Data;
    }
    else
    {
        *pu32Addr = g_u32ApromSize;
        *pu32Size = 0;
    }
}
#endif

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

