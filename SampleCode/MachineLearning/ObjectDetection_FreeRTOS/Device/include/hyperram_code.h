/**************************************************************************//**
 * @file        hyperflash_code.h
 * @version     V3.00
 * @brief       HyperFlash device driver
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __HYPER_RAM_CODE_H__
#define __HYPER_RAM_CODE_H__

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------------
#define BUFF_SIZE                   0x200

//------------------------------------------------------------------------------
void HyperRAM_Erase(SPIM_T *spim, uint32_t u32StartAddr, uint32_t u32EraseSize);
void HyperRAM_Init(SPIM_T *spim);
void HyperRAM_PinConfig(SPIM_T *spim);

#ifdef __cplusplus
}
#endif

#endif  /* __HYPER_RAM_CODE_H__ */

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
