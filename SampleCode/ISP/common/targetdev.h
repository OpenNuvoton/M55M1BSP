/***************************************************************************//**
 * @file     targetdev.h
 * @version  V1.00
 * @brief    ISP support function header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __TARGET_H__
#define __TARGET_H__

#include "NuMicro.h"
#include "isp_user.h"

#define PLL_CLOCK       FREQ_180MHZ
#define I2C_ADDR        0x60
#define DETECT_PIN      PB12

#ifdef __cplusplus
extern "C"
{
#endif

void GetDataFlashInfo(uint32_t *addr, uint32_t *size);
uint32_t GetApromSize(void);

#ifdef __cplusplus
}
#endif

#endif // __TARGET_H__
