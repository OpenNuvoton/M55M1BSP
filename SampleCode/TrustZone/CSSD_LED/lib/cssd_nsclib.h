/**************************************************************************//**
 * @file     cssd_nsclib.h
 * @version  V1.00
 * @brief    M55M1 Collaborative Secure Software Development Library header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __CSSD_NSCLIB_H__
#define __CSSD_NSCLIB_H__

/* typedef for Non-secure callback function */
typedef __NONSECURE_CALL int32_t (*PFN_NON_SECURE_FUNC)(uint32_t);

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------------------------------------------------
  Non-secure Callable Functions from Secure Region
 *----------------------------------------------------------------------------*/
__NONSECURE_ENTRY int32_t Secure_PA11_LED_On(uint32_t num);
__NONSECURE_ENTRY int32_t Secure_PA11_LED_Off(uint32_t num);
__NONSECURE_ENTRY int32_t Secure_PA12_LED_On(uint32_t num);
__NONSECURE_ENTRY int32_t Secure_PA12_LED_Off(uint32_t num);
__NONSECURE_ENTRY int32_t Secure_PA13_LED_On(uint32_t num);
__NONSECURE_ENTRY int32_t Secure_PA13_LED_Off(uint32_t num);
__NONSECURE_ENTRY uint32_t GetSystemCoreClock(void);

#ifdef __cplusplus
}
#endif

#endif  // __CSSD_NSCLIB_H__
