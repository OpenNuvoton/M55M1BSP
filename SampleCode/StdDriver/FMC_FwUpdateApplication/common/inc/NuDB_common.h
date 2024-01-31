/**************************************************************************//**
 * @file     NuDB_common.h
 * @version  V0.00
 * @brief    NuDB common header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __NUDB_COMMON_H__
#define __NUDB_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define LOADER_BASE         (FMC_APROM_BASE)
#define LOADER_SIZE         (FMC_FLASH_PAGE_SIZE * 2)
#define APP_BASE            (FMC_APROM_BASE + LOADER_SIZE)
#define APP_SIZE            (FMC_FLASH_PAGE_SIZE * 4)

#define FW_CRC_BASE         (FMC_APROM_BANK0_END - FMC_FLASH_PAGE_SIZE)
#define NEW_FW_CRC_BASE     (FW_CRC_BASE + 0x4)
#define BACKUP_FW_CRC_BASE  (FW_CRC_BASE + 0x8)

#ifdef __cplusplus
}
#endif

#endif /* __NUDB_COMMON_H__ */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
