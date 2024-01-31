/******************************************************************************
 * @file     sdglue.c
 * @version  V1.00
 * @brief    SD glue functions for FATFS.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"
#include "diskio.h"     /* FatFs lower layer API */
#include "ff.h"     /* FatFs lower layer API */

static FATFS  _FatfsVolSd0;
static FATFS  _FatfsVolSd1;

static TCHAR  _Path[3];

int32_t SDH_Open_Disk(SDH_T *sdh, uint32_t u32CardDetSrc)
{
    SDH_Open(sdh, u32CardDetSrc);

    if (SDH_Probe(sdh))
    {
        printf("SD initial fail!!\n");
        return SDH_ERR_FAIL;
    }

    _Path[1] = ':';
    _Path[2] = 0;

    if (sdh == SDH0)
    {
        _Path[0] = '0';
        f_mount(&_FatfsVolSd0, _Path, 1);
    }
    else
    {
        _Path[0] = '1';
        f_mount(&_FatfsVolSd1, _Path, 1);
    }

    return SDH_OK;
}

void SDH_Close_Disk(SDH_T *sdh)
{
    if (sdh == SDH0)
    {
        memset(&SD0, 0, sizeof(SDH_INFO_T));
        f_mount(NULL, _Path, 1);
        memset(&_FatfsVolSd0, 0, sizeof(FATFS));
    }
    else
    {
        memset(&SD1, 0, sizeof(SDH_INFO_T));
        f_mount(NULL, _Path, 1);
        memset(&_FatfsVolSd1, 0, sizeof(FATFS));
    }
}
