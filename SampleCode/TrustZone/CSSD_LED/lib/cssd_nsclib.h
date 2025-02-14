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

#define LED_ON      FALSE
#define LED_OFF     TRUE

/* typedef for Non-secure callback function */
typedef __NONSECURE_CALL int32_t (*PFN_NON_SECURE_FUNC)(uint32_t);

#ifdef __cplusplus
extern "C" {
#endif

/*----------------------------------------------------------------------------
  Non-secure Callable Functions from Secure Region
 *----------------------------------------------------------------------------*/
__NONSECURE_ENTRY int32_t Secure_LED1(uint32_t u32Num, uint32_t bOn);
__NONSECURE_ENTRY int32_t Secure_LED2(uint32_t u32Num, uint32_t bOn);
__NONSECURE_ENTRY int32_t Secure_LED3(uint32_t u32Num, uint32_t bOn);

__NONSECURE_ENTRY uint32_t GetSystemCoreClock(void);

#ifdef __cplusplus
}
#endif

#endif  // __CSSD_NSCLIB_H__
