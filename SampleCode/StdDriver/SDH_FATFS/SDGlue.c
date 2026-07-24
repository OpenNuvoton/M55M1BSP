/**************************************************************************//**
 * @file     SDGlue.c
 * @version  V1.00
 * @brief    SD glue functions for FATFS
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"
#include "diskio.h"     /* FatFs lower layer API */
#include "ff.h"     /* FatFs lower layer API */

FATFS  _FatfsVolSd0;
FATFS  _FatfsVolSd1;

static TCHAR  _Path[3];

static FATFS *SDH_GetFatFsVolume(SDH_T *sdh)
{
    return (sdh == SDH0) ? &_FatfsVolSd0 : &_FatfsVolSd1;
}

static void SDH_GetDrivePath(SDH_T *sdh, TCHAR *pachPath)
{
    pachPath[0] = (sdh == SDH0) ? '0' : '1';
    pachPath[1] = ':';
    pachPath[2] = 0;
}

int32_t SDH_Open_Disk(SDH_T *sdh, uint32_t u32CardDetSrc)
{
    FATFS *pFatFs = SDH_GetFatFsVolume(sdh);

    SDH_GetDrivePath(sdh, _Path);
    f_mount(NULL, _Path, 1);
    (void)memset(pFatFs, 0, sizeof(FATFS));

    SDH_Open(sdh, u32CardDetSrc);

    if (SDH_Probe(sdh))
    {
        printf("SD initial fail!!\n");
        return -1;
    }

    if (f_mount(pFatFs, _Path, 1) != FR_OK)
    {
        printf("f_mount fail!!\n");
        return -1;
    }

    return 0;
}

void SDH_Close_Disk(SDH_T *sdh)
{
    FATFS *pFatFs = SDH_GetFatFsVolume(sdh);
    SDH_INFO_T *pSD = (sdh == SDH0) ? &SD0 : &SD1;

    SDH_GetDrivePath(sdh, _Path);
    f_mount(NULL, _Path, 1);
    (void)memset(pFatFs, 0, sizeof(FATFS));
    (void)memset(pSD, 0, sizeof(SDH_INFO_T));
    SDH_Close(sdh);
}
