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
__NONSECURE_ENTRY uint32_t GetSystemCoreClock(void);

#ifdef __cplusplus
}
#endif

#endif  // __NSCLIB_H__
