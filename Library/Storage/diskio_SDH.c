/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "NuMicro.h"
#include "diskio.h"     /* FatFs lower layer API */
#include "ff.h"

//--------------------------------------------------------------------------------------------------
/* Definitions of physical drive number for each media */
#define SDH0_DRIVE      0        /* for SD0          */
#define SDH1_DRIVE      1        /* for SD1          */
#define EMMC_DRIVE      2        /* for eMMC/NAND    */
#define USBH_DRIVE_0    3        /* USB Mass Storage */
#define USBH_DRIVE_1    4        /* USB Mass Storage */
#define USBH_DRIVE_2    5        /* USB Mass Storage */
#define USBH_DRIVE_3    6        /* USB Mass Storage */
#define USBH_DRIVE_4    7        /* USB Mass Storage */

/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize(BYTE pdrv)        /* Physical drive number (0..) */
{
    SDH_T *sdh = (pdrv == SDH0_DRIVE) ? SDH0 : (pdrv == SDH1_DRIVE) ? SDH1 : NULL;

    if (sdh == NULL)
    {
        return RES_ERROR;
    }

    return (SDH_GET_CARD_CAPACITY(sdh) == 0) ? RES_ERROR : RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status(BYTE pdrv)        /* Physical drive number (0..) */
{
    return disk_initialize(pdrv);
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
DRESULT disk_read(
    BYTE pdrv,      /* Physical drive number (0..) */
    BYTE *buff,     /* Data buffer to store read data */
    LBA_t sector,   /* Sector address (LBA) */
    UINT count      /* Number of sectors to read (1..128) */
)
{
    SDH_T *sdh = (pdrv == SDH0_DRIVE) ? SDH0 : (pdrv == SDH1_DRIVE) ? SDH1 : NULL;
    DRESULT  ret = RES_PARERR;

    //printf("disk_read - drv:%d, sec:%d, cnt:%d, buff:0x%x\n", pdrv, sector, count, (uint32_t)buff);

    if ((count > 0) && sdh && (buff != NULL))
    {
        uint32_t drv_ret = SDH_Read(sdh, (uint8_t *)buff, sector, count);

        ret = (drv_ret == Successful) ? RES_OK : RES_ERROR;
    }

    return ret;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/
DRESULT disk_write(
    BYTE pdrv,          /* Physical drive number (0..) */
    const BYTE *buff,   /* Data to be written */
    LBA_t sector,       /* Start sector in LBA */
    UINT count          /* Number of sectors to write (1..128) */
)
{
    SDH_T *sdh = (pdrv == SDH0_DRIVE) ? SDH0 : (pdrv == SDH1_DRIVE) ? SDH1 : NULL;

    DRESULT  ret = RES_PARERR;

    //printf("disk_write - drv:%d, sec:%d, cnt:%d, buff:0x%x\n", pdrv, sector, count, (uint32_t)buff);

    if ((count > 0) && sdh && (buff != NULL))
    {
        uint32_t drv_ret = SDH_Write(sdh, (uint8_t *)buff, sector, count);

        ret = (drv_ret == Successful) ? RES_OK : RES_ERROR;
    }

    return ret;
}

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
DRESULT disk_ioctl(
    BYTE pdrv,      /* Physical drive number (0..) */
    BYTE cmd,       /* Control code */
    void *buff      /* Buffer to send/receive control data */
)
{
    SDH_T *sdh = (pdrv == SDH0_DRIVE) ? SDH0 : (pdrv == SDH1_DRIVE) ? SDH1 : NULL;
    DRESULT res = RES_PARERR;

    if (sdh)
    {
        SDH_INFO_T *sdh_info = (sdh == SDH0) ? &SD0 : (sdh == SDH1) ? &SD1 : NULL;

        switch (cmd)
        {
            case CTRL_SYNC:
                res = RES_OK;
                break;

            case GET_SECTOR_COUNT:
                *(DWORD *)buff = sdh_info->totalSectorN;
                res = RES_OK;

                break;

            case GET_SECTOR_SIZE:
                *(WORD *)buff = sdh_info->sectorSize;
                res = RES_OK;
                break;

            default:
                res = RES_PARERR;
                break;
        }
    }

    return res;
}
