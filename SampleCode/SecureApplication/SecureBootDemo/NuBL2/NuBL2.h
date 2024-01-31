/**************************************************************************//**
 * @file     NuBL2.h
 * @version  V3.00
 * @brief    NuBL2 header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "../image.h"

#define NUBL2_FW_IMG_SIZE           (0x00040000)                        // 256 KB
#define NUBL32_FW_IMG_BASE          (FMC_APROM_BASE + 0x00040000ul)     // 256 KB
#define NUBL32_FW_IMG_SIZE          (0x00040000)                        // 256 KB
#define NUBL33_FW_IMG_BASE          FMC_NON_SECURE_BASE                 // Non-secure flash base address
#define NUBL33_FW_IMG_SIZE          (0x00040000)                        // 256 KB

#ifdef __cplusplus
extern "C"
{
#endif

int32_t VerifyNuBL3x(uint32_t u32ImgBaseAddr, uint32_t u32ImgByteSize, uint32_t *pu32ImgStartAddr);

#ifdef __cplusplus
extern "C"
}
#endif

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
