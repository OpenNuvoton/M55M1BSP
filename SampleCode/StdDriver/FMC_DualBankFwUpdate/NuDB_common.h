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

#define LOADER_BASE     FMC_APROM_BASE
#define LOADER_SIZE     0x10000
#define APP_BASE        (FMC_APROM_BASE + LOADER_SIZE)
#define APP_SIZE        0x10000

#ifdef __cplusplus
extern "C"
{
#endif


#ifdef __cplusplus
}
#endif

#endif /* __NUDB_COMMON_H__ */

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
