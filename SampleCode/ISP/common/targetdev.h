/***************************************************************************//**
 * @file     targetdev.h
 * @version  V1.00
 * @brief    ISP support function header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __TARGETDEV_H__
#define __TARGETDEV_H__

#include "NuMicro.h"

#define PLL_CLOCK       FREQ_220MHZ
#define I2C_ADDR        0x60
#define DETECT_PIN      PI11    // NuMaker BTN_0

#ifdef __cplusplus
extern "C"
{
#endif

extern uint32_t g_u32ApromSize;

void GetDataFlashInfo(uint32_t *addr, uint32_t *size);
uint32_t GetApromSize(void);

#ifdef __cplusplus
}
#endif

#endif // __TARGETDEV_H__
