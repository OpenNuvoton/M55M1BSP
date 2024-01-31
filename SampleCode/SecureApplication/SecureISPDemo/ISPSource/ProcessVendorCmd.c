/**************************************************************************//**
 * @file     ProcessVendorCmd.c
 * @version  V3.00
 * @brief    Process vendor command.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "CommandHandler.h"

int32_t Exec_VendorFunction(uint32_t *pu32Buf, uint32_t u32Len);

/*
    Provides the below vendor functions for USBH_SecureISP or PC VendorCmdSample Tool:
            Command ID      Description             Usage of Host
        1.  0x1000          Get Chip IDs            [Cmd ID]
        2.  0x2000          Read flash data         [Cmd ID, Addr, Size]
        3.  0x3000          Program flash           [Cmd ID, Addr, Size, Data0, Data1...]
*/
int32_t Exec_VendorFunction(uint32_t *pu32Buf, uint32_t u32Len)
{
    uint32_t        i, u32Addr, u32Size, u32Return;
    CMD_PACKET_T    cmd;

    NVT_UNUSED(u32Len);
    memcpy(&cmd, pu32Buf, sizeof(cmd));

    u32Len = cmd.au32Data[0]; // received byte counts

    if (cmd.au32Data[1] == 0x1000) // return IDs
    {
        /* allocate response cmd data */
        memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
        u32Return = 4 * 8;

        cmd.au32Data[2] = SYS->PDID;
        cmd.au32Data[3] = 0x11110000;
        cmd.au32Data[4] = 0x22220000;
        cmd.au32Data[5] = 0x33330000;
        cmd.au32Data[6] = 0x44440000;
        cmd.au32Data[7] = 0x55550000;
        cmd.au32Data[8] = 0x66660000;
        cmd.au32Data[9] = 0x77770000;
    }
    else if (cmd.au32Data[1] == 0x2000) // read flash
    {
        u32Addr = cmd.au32Data[2];
        u32Size = cmd.au32Data[3];

        /* allocate response cmd data */
        memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
        u32Return = u32Size;

        for (i = 0; i < (u32Size / 4); i++)
            cmd.au32Data[2 + i] = FMC_Read(u32Addr + (i * 4));
    }
    else if (cmd.au32Data[1] == 0x3000) // write flash
    {
        u32Addr = cmd.au32Data[2];
        u32Size = cmd.au32Data[3];

        FMC_ENABLE_ISP();
        FMC_ENABLE_AP_UPDATE();

        for (i = 0; i < (u32Size / 4); i++)
            FMC_Write(u32Addr + (i * 4), cmd.au32Data[4 + i]);

        FMC_DISABLE_AP_UPDATE();

        /* allocate response cmd data */
        memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
        u32Return = 0;
    }
    else
    {
        return 0;
    }

    cmd.au32Data[0] = STS_OK;
    cmd.au32Data[1] = u32Return;    // valid vendor data length
    cmd.u16Len = (uint16_t)(4 + 4 + u32Return); // cmd length

    memcpy(pu32Buf, &cmd, sizeof(cmd));
    return cmd.u16Len;
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
