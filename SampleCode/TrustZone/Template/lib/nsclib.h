/**************************************************************************//**
 * @file     nsclib.h
 * @version  V1.00
 * @brief    M55M1 Non-secure callable library header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __NSCLIB_H__
#define __NSCLIB_H__

/* typedef for Non-secure callback function */
typedef __NONSECURE_CALL int32_t (*PFN_NON_SECURE_FUNC)(uint32_t);

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------------------------------------------------
  Non-secure Callable Functions from Secure Region
 *----------------------------------------------------------------------------*/
__NONSECURE_ENTRY int32_t SecureFunc(void);
__NONSECURE_ENTRY int32_t Secure_LED_On(uint32_t u32Ticks);
__NONSECURE_ENTRY int32_t Secure_LED_Off(uint32_t u32Ticks);
__NONSECURE_ENTRY int32_t Secure_LED_On_Callback(PFN_NON_SECURE_FUNC pfnCallback);
__NONSECURE_ENTRY int32_t Secure_LED_Off_Callback(PFN_NON_SECURE_FUNC pfnCallback);

#ifdef __cplusplus
}
#endif

#endif  // __NSCLIB_H__
