/**************************************************************************//**
 * @file     gdma.h
 * @version  V1.00
 * @brief    GDMA driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __GDMA_H__
#define __GDMA_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "dma350_lib.h"
#include "dma350_drv.h"
#include "dma350_ch_drv.h"

extern struct dma350_dev_t GDMA_DEV_S;
extern struct dma350_ch_dev_t *const GDMA_CH_DEV_S[];

#ifdef __cplusplus
}
#endif

#endif //__GDMA_H__
