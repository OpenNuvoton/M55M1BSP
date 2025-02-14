/***************************************************************************//**
 * @file     isp_user.c
 * @version  V1.00
 * @brief    ISP Command source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "targetdev.h"
#include "isp_user.h"

__ALIGNED(4) uint8_t g_au8ResponseBuff[128];
__ALIGNED(4) static uint8_t g_au8ApromBuf[FMC_FLASH_PAGE_SIZE];
uint32_t g_u32UpdateApromCmd;
uint32_t g_u32DataFlashAddr = 0,
         g_u32DataFlashSize = 0;

static uint16_t Checksum(unsigned char *buf, int len)
{
    int i;
    uint16_t c;

    for (c = 0, i = 0 ; i < len; i++)
    {
        c += buf[i];
    }

    return (c);
}

static uint16_t CalCheckSum(uint32_t u32start, uint32_t u32len)
{
    uint32_t u32i;
    uint16_t u16lcksum = 0;

    for (u32i = 0; u32i < u32len; u32i += FMC_FLASH_PAGE_SIZE)
    {
        ReadData(u32start + u32i, u32start + u32i + FMC_FLASH_PAGE_SIZE, (uint32_t *)g_au8ApromBuf);

        if (u32len - u32i >= FMC_FLASH_PAGE_SIZE)
        {
            u16lcksum += Checksum(g_au8ApromBuf, FMC_FLASH_PAGE_SIZE);
        }
        else
        {
            u16lcksum += Checksum(g_au8ApromBuf, u32len - u32i);
        }
    }

    return u16lcksum;
}

int ParseCmd(uint8_t *pu8Buffer, uint8_t u8len)
{
    static uint32_t u32StartAddress, u32StartAddress_bak, u32TotalLen, u32TotalLen_bak, u32LastDataLen, u32PackNo = 1;
    uint32_t u32PageAddress;
    uint8_t *pu8Response;
    uint16_t u16Lcksum;
    uint32_t u32Lcmd, u32srclen, u32i, u32Lock;
    uint32_t *pu32Config;
    uint8_t *pu8Src;
    static uint32_t u32Gcmd;

    pu8Response = g_au8ResponseBuff;
    pu8Src = pu8Buffer;
    u32srclen = u8len;
    u32Lcmd = inpw((uint32_t)pu8Src);
    outpw((uint32_t)(pu8Response + 4), 0);
    pu8Src += 8;
    u32srclen -= 8;
    //ReadData(Config0, Config0 + 16, (uint32_t *)(uint32_t)(pu8Response + 8)); /* Read config */
    ReadData(Config0, Config0 + (FMC_CONFIG_CNT * 4), (uint32_t *)(uint32_t)(pu8Response + 8)); /* Read config */
    pu32Config = (uint32_t *)(pu8Response + 8);
    u32Lock = (((pu32Config[11] & 0xFF) != 0x5A) || ((pu32Config[13] & 0xFF) != 0x5A));

    if (u32Lcmd == CMD_SYNC_PACKNO)
    {
        u32PackNo = inpw((uint32_t)pu8Src);
    }

    if ((u32Lcmd) && (u32Lcmd != CMD_RESEND_PACKET))
    {
        u32Gcmd = u32Lcmd;
    }

    if (u32Lcmd == CMD_GET_FWVER)
    {
        pu8Response[8] = FW_VERSION; /* version 2.3 */
    }
    else if (u32Lcmd == CMD_GET_DEVICEID)
    {
        outpw((uint32_t)(pu8Response + 8), SYS->PDID);
        goto out;
    }
    else if (u32Lcmd == CMD_RUN_APROM || u32Lcmd == CMD_RUN_LDROM || u32Lcmd == CMD_RESET)
    {
        /* Clear POR and Reset Pin reset flag */
        SYS_ClearResetSrc(SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);

        /* Set BS */
        if (u32Lcmd == CMD_RUN_APROM)
        {
            u32i = (FMC->ISPCTL & 0xFFFFFFFC);
        }
        else if (u32Lcmd == CMD_RUN_LDROM)
        {
            u32i = (FMC->ISPCTL & 0xFFFFFFFC);
            u32i |= 0x00000002;
        }
        else
        {
            u32i = (FMC->ISPCTL & 0xFFFFFFFE); /* ISP disable */
        }

        outpw(&FMC->ISPCTL, u32i);
        outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

        /* Trap the CPU */
        while (1);
    }
    else if (u32Lcmd == CMD_CONNECT)
    {
        u32PackNo = 1;
        goto out;
    }
    else if ((u32Lcmd == CMD_UPDATE_APROM) || (u32Lcmd == CMD_ERASE_ALL))
    {
        //EraseAP(FMC_APROM_BASE, (g_u32ApromSize < g_u32DataFlashAddr) ? g_u32ApromSize : g_u32DataFlashAddr);
        EraseAP(FMC_APROM_BASE, g_u32ApromSize);

#if 0   // M55M1 did not support data flash

        if (u32Lcmd == CMD_ERASE_ALL)   /* Erase data flash */
        {
            EraseAP(g_u32DataFlashAddr, g_u32DataFlashSize);
            *(uint32_t *)(pu8Response + 8) = u32regcnf0 | 0x02;
            UpdateConfig((uint32_t *)(pu8Response + 8), NULL);
        }

#endif

        g_u32UpdateApromCmd = TRUE;
    }
    else if (u32Lcmd == CMD_GET_FLASHMODE)
    {
        /* Return 1: APROM, 2: LDROM */
        outpw(pu8Response + 8, (FMC->ISPCTL & 0x2) ? 2 : 1);
    }

    if ((u32Lcmd == CMD_UPDATE_APROM) || (u32Lcmd == CMD_UPDATE_DATAFLASH))
    {
        if (u32Lcmd == CMD_UPDATE_DATAFLASH)
        {
            u32StartAddress = g_u32DataFlashAddr;

            if (g_u32DataFlashSize)
            {
                EraseAP(g_u32DataFlashAddr, g_u32DataFlashSize);
            }
            else
            {
                goto out;
            }
        }
        else
        {
            u32StartAddress = 0;
        }

        u32TotalLen = inpw(pu8Src + 4);
        pu8Src += 8;
        u32srclen -= 8;
        u32StartAddress_bak = u32StartAddress;
        u32TotalLen_bak = u32TotalLen;
    }
    else if (u32Lcmd == CMD_UPDATE_CONFIG)
    {
        if ((u32Lock) && (!g_u32UpdateApromCmd))   /* security lock */
        {
            goto out;
        }

        UpdateConfig((uint32_t *)(uint32_t)pu8Src, (uint32_t *)(uint32_t)(pu8Response + 8));
        //GetDataFlashInfo(&g_u32DataFlashAddr, &g_u32DataFlashSize);
        goto out;
    }
    else if (u32Lcmd == CMD_RESEND_PACKET)     /* for APROM and Data flash only */
    {
        u32StartAddress -= u32LastDataLen;
        u32TotalLen += u32LastDataLen;
        u32PageAddress = u32StartAddress & (0x100000 - FMC_FLASH_PAGE_SIZE);

        if (u32PageAddress >= Config0)
        {
            goto out;
        }

        ReadData(u32PageAddress, u32StartAddress, (uint32_t *)g_au8ApromBuf);
        FMC_Erase_User(u32PageAddress);
        WriteData(u32PageAddress, u32StartAddress, (uint32_t *)g_au8ApromBuf);

        if ((u32StartAddress % FMC_FLASH_PAGE_SIZE) >= (FMC_FLASH_PAGE_SIZE - u32LastDataLen))
        {
            FMC_Erase_User(u32PageAddress + FMC_FLASH_PAGE_SIZE);
        }

        goto out;
    }

    if ((u32Gcmd == CMD_UPDATE_APROM) || (u32Gcmd == CMD_UPDATE_DATAFLASH))
    {
        if (u32TotalLen < u32srclen)
        {
            u32srclen = u32TotalLen; /* Prevent last package from over writing */
        }

        u32TotalLen -= u32srclen;
        WriteData(FMC_APROM_BASE + u32StartAddress, FMC_APROM_BASE + u32StartAddress + u32srclen, (uint32_t *)(uint32_t)pu8Src);
        memset(pu8Src, 0, u32srclen);
        ReadData(FMC_APROM_BASE + u32StartAddress, FMC_APROM_BASE + u32StartAddress + u32srclen, (uint32_t *)(uint32_t)pu8Src);
        u32StartAddress += u32srclen;
        u32LastDataLen = u32srclen;

        if (u32TotalLen == 0)
        {
            u16Lcksum = CalCheckSum(FMC_APROM_BASE + u32StartAddress_bak, u32TotalLen_bak);
            outps(pu8Response + 8, u16Lcksum);
        }
    }

out:
    u16Lcksum = Checksum(pu8Buffer, u8len);
    outps((uint32_t)pu8Response, u16Lcksum);
    ++u32PackNo;
    outpw((uint32_t)(pu8Response + 4), u32PackNo);
    u32PackNo++;
    return 0;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
