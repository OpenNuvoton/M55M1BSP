/**************************************************************************//**
 * @file    ACI_SinCos.h
 * @version V1.00
 * @brief   Helper function for Nuvoton sin/cos ACI feature
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __ACI_SINCOS_H__
#define __ACI_SINCOS_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief      This function return the sine of fRadians
  * @param[in]  fRadians
  * @return     sinf of fRadians
  */
float sinf_aci(float fRadians);

/**
  * @brief      This function return the cosine of fRadians
  * @param[in]  fRadians
  * @return     cosf of fRadians
  */
float cosf_aci(float fRadians);

#ifdef __cplusplus
}
#endif

#endif


