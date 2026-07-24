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
#include <stddef.h>

#include "NuMicro.h"
#include "ff.h"
#include "diskio.h"     /* FatFs lower layer API */

//--------------------------------------------------------------------------------------------------
/* Definitions of physical drive number for each media */
#define SDH0_DRIVE                  0        /* for SD0          */
#define SDH1_DRIVE                  1        /* for SD1          */
#define EMMC_DRIVE                  2        /* for eMMC/NAND    */
#define USBH_DRIVE_0                3        /* USB Mass Storage */
#define USBH_DRIVE_1                4        /* USB Mass Storage */
#define USBH_DRIVE_2                5        /* USB Mass Storage */
#define USBH_DRIVE_3                6        /* USB Mass Storage */
#define USBH_DRIVE_4                7        /* USB Mass Storage */
#define DISKIO_MAX_TRANSFER_SECTORS 128U

//------------------------------------------------------------------------------
extern DSTATUS disk_initialize(BYTE pdrv);
extern DSTATUS disk_status(BYTE pdrv);
extern DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count);
extern DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count);
extern DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff);

//------------------------------------------------------------------------------
// Get SDH object pointer and SDH info pointer by physical drive number
//------------------------------------------------------------------------------
static SDH_T *diskio_get_sdh(BYTE pdrv)
{
    SDH_T *sdh;

    switch (pdrv)
    {
        case (BYTE)SDH0_DRIVE:
            sdh = SDH0;
            break;

        case (BYTE)SDH1_DRIVE:
            sdh = SDH1;
            break;

        default:
            sdh = (SDH_T *)NULL;
            break;
    }

    return sdh;
}

static SDH_INFO_T *diskio_get_sdh_info(BYTE pdrv)
{
    SDH_INFO_T *sdh_info;

    switch (pdrv)
    {
        case (BYTE)SDH0_DRIVE:
            sdh_info = &SD0;
            break;

        case (BYTE)SDH1_DRIVE:
            sdh_info = &SD1;
            break;

        default:
            sdh_info = (SDH_INFO_T *)NULL;
            break;
    }

    return sdh_info;
}

/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/*-----------------------------------------------------------------------*/
/* cppcheck-suppress misra-c2012-8.6 ; DEVIATION: FatFs diskio glue; alternate impl in diskio_USBHost.c, only one linked per project */
DSTATUS disk_initialize(BYTE pdrv)         /* Physical drive number (0..) */
{
    SDH_T const *sdh = diskio_get_sdh(pdrv);
    SDH_INFO_T const *sdh_info = diskio_get_sdh_info(pdrv);

    if ((sdh == (SDH_T *)NULL) || (sdh_info == (SDH_INFO_T *)NULL))
    {
        return STA_NOINIT;
    }

    if ((SDH_GET_CARD_CAPACITY(sdh) == (uint32_t)0U) ||
            (sdh_info->totalSectorN == 0U) ||
            (sdh_info->sectorSize == 0L))
    {
        return STA_NOINIT;
    }

    return 0U;
}

/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/
/* cppcheck-suppress misra-c2012-8.6 ; DEVIATION: FatFs diskio glue; alternate impl in diskio_USBHost.c, only one linked per project */
DSTATUS disk_status(BYTE pdrv)        /* Physical drive number (0..) */
{
    return disk_initialize(pdrv);
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
/* cppcheck-suppress misra-c2012-8.6 ; DEVIATION: FatFs diskio glue; alternate impl in diskio_USBHost.c, only one linked per project */
DRESULT disk_read(
    BYTE pdrv,      /* Physical drive number (0..) */
    BYTE *buff,     /* Data buffer to store read data */
    LBA_t sector,   /* Sector address (LBA) */
    UINT count      /* Number of sectors to read (1..128) */
)
{
    SDH_T *sdh = diskio_get_sdh(pdrv);
    SDH_INFO_T const *sdh_info = diskio_get_sdh_info(pdrv);
    DRESULT  ret = RES_PARERR;

    //printf("disk_read - drv:%d, sec:%d, cnt:%d, buff:0x%x\n", pdrv, sector, count, (uint32_t)buff);

    if ((sdh == (SDH_T *)NULL) ||
            (sdh_info == (SDH_INFO_T const *)NULL) ||
            (buff == (BYTE *)NULL) ||
            (count == 0U) ||
            (count > DISKIO_MAX_TRANSFER_SECTORS))
    {
        return RES_PARERR;
    }

    if (disk_initialize(pdrv) != 0U)
    {
        return RES_NOTRDY;
    }

    if ((sector >= (LBA_t)sdh_info->totalSectorN) ||
            ((LBA_t)count > ((LBA_t)sdh_info->totalSectorN - sector)))
    {
        return RES_PARERR;
    }

    if ((count > 0U) && sdh)
    {
        uint32_t drv_ret = SDH_Read(sdh, (uint8_t *)buff, sector, count);

        ret = (drv_ret == Successful) ? RES_OK : RES_ERROR;
    }

    return ret;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/
/* cppcheck-suppress misra-c2012-8.6 ; DEVIATION: FatFs diskio glue; alternate impl in diskio_USBHost.c, only one linked per project */
DRESULT disk_write(
    BYTE pdrv,          /* Physical drive number (0..) */
    const BYTE *buff,   /* Data to be written */
    LBA_t sector,       /* Start sector in LBA */
    UINT count          /* Number of sectors to write (1..128) */
)
{
    SDH_T *sdh = diskio_get_sdh(pdrv);
    SDH_INFO_T *sdh_info = diskio_get_sdh_info(pdrv);

    DRESULT  ret = RES_PARERR;

    //printf("disk_write - drv:%d, sec:%d, cnt:%d, buff:0x%x\n", pdrv, sector, count, (uint32_t)buff);

    if ((sdh == (SDH_T *)NULL) ||
            (sdh_info == (SDH_INFO_T *)NULL) ||
            (buff == (BYTE *)NULL) ||
            (count == 0U) ||
            (count > DISKIO_MAX_TRANSFER_SECTORS))
    {
        return RES_PARERR;
    }

    if (disk_initialize(pdrv) != 0U)
    {
        return RES_NOTRDY;
    }

    if ((sector >= (LBA_t)sdh_info->totalSectorN) ||
            ((LBA_t)count > ((LBA_t)sdh_info->totalSectorN - sector)))
    {
        return RES_PARERR;
    }

    if ((count > 0U) && sdh)
    {
        uint32_t drv_ret = SDH_Write(sdh, (uint8_t *)buff, sector, count);

        ret = (drv_ret == Successful) ? RES_OK : RES_ERROR;
    }

    return ret;
}

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
/* cppcheck-suppress misra-c2012-8.6 ; DEVIATION: FatFs diskio glue; alternate impl in diskio_USBHost.c, only one linked per project */
DRESULT disk_ioctl(
    BYTE pdrv,      /* Physical drive number (0..) */
    BYTE cmd,       /* Control code */
    void *buff      /* Buffer to send/receive control data */
)
{
    SDH_T const *sdh = diskio_get_sdh(pdrv);
    SDH_INFO_T const *sdh_info = diskio_get_sdh_info(pdrv);
    DRESULT res = RES_PARERR;

    if ((sdh == (SDH_T *)NULL) ||
            (sdh_info == (SDH_INFO_T *)NULL))
    {
        return RES_PARERR;
    }

    if (disk_initialize(pdrv) != 0U)
    {
        return RES_NOTRDY;
    }

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

        case GET_BLOCK_SIZE:
            *(DWORD *)buff = 1U;
            res = RES_OK;
            break;

        default:
            res = RES_PARERR;
            break;
    }

    return res;
}
