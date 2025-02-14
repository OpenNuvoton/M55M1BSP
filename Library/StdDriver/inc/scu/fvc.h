/**************************************************************************//**
 * @file     fvc.h
 * @version  V1.00
 * @brief    Firmware Version Counter driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __FVC_H__
#define __FVC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup FVC_Driver FVC Driver
  @{
*/

/** @addtogroup FVC_EXPORTED_CONSTANTS FVC Exported Constants
  @{
*/

#define FVC_WVCODE   (0x7710 << FVC_CTL_WVCODE_Pos)  /*!< The key code for FVC_CTL write. */

/** @} end of group FVC_EXPORTED_CONSTANTS */


/** @addtogroup FVC_EXPORTED_FUNCTIONS FVC Exported Functions
  @{
*/

int32_t FVC_Open(void);
int32_t FVC_EnableMonotone(void);
int32_t FVC_SetNVC(uint32_t u32NvcIdx, uint32_t u32Cnt);
int32_t FVC_GetNVC(uint32_t u32NvcIdx);


/** @} end of group FVC_EXPORTED_FUNCTIONS */
/** @} end of group FVC_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __FVC_H__ */
