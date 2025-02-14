/**************************************************************************//**
 * @file     NuBL32.h
 * @version  V3.00
 * @brief    NuBL32 header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

extern const uint32_t g_InitialFWInfo[]; // A global variable to store NuBL32 FWINFO address, declared in FwInfo.c

#ifdef __cplusplus
extern "C"
{
#endif

/*----------------------------------------------------------------------------
  Non-secure Callable Functions from Secure Region
 *----------------------------------------------------------------------------*/
__NONSECURE_ENTRY uint32_t GetSystemCoreClock(void);

#ifdef __cplusplus
extern "C"
}
#endif

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
