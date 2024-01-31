/**************************************************************************//**
 * @file     trng.h
 * @version  V1.00
 * @brief    TRNG driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __TRNG_H__
#define __TRNG_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup TRNG_Driver TRNG Driver
  @{
*/


/** @addtogroup TRNG_EXPORTED_CONSTANTS TRNG Exported Constants
  @{
*/

/*----------------------------------------------------------------------------------------------*/
/*  Macros                                                                                      */
/*----------------------------------------------------------------------------------------------*/
#define TRNG_CTL_KATSEL_INSTANTIATION_TESTING      0x0 /*!< TRNG CTR_DRBG Known Answer Test Selection: Instantiation*/
#define TRNG_CTL_KATSEL_RESEED_TESTING             0x1 /*!< TRNG CTR_DRBG Known Answer Test Selection: Reseed*/
#define TRNG_CTL_KATSEL_GENERATION_TESTING         0x2 /*!< TRNG CTR_DRBG Known Answer Test Selection: Generation*/

#define TRNG_CTL_MODE_OUTPUT_ENTROPY               0x00 /*!< TRNG Random Bit Generator Output Selection: entropy (32-bits)*/
#define TRNG_CTL_MODE_OUTPUT_NRBG                  0x01 /*!< TRNG Random Bit Generator Output Selection: NRBG (128-bits)*/
#define TRNG_CTL_MODE_OUTPUT_DRBG                  0x02 /*!< TRNG Random Bit Generator Output Selection: DRBG (128-bits)*/
/** @} end of group TRNG_EXPORTED_CONSTANTS */


/** @addtogroup TRNG_EXPORTED_MACROS TRNG Exported Macros
  @{
*/

/**
  * @brief This macro enables TRNG interrupt.
  * @param trng     Specified trng module
  * @return None
  * \hideinitializer
  */
#define TRNG_ENABLE_INT(trng)       ((trng)->CTL |= TRNG_CTL_DVIEN_Msk)

/**
  * @brief This macro disables TRNG interrupt.
  * @param trng     Specified trng module
  * @return None
  * \hideinitializer
  */
#define TRNG_DISABLE_INT(trng)      ((trng)->CTL &= ~TRNG_CTL_DVIEN_Msk)

/**
  * @brief This macro gets TRNG interrupt flag.
  * @param trng     Specified trng module
  * @return TRNG interrupt flag.
  * \hideinitializer
  */
#define TRNG_GET_INT_FLAG(trng)     ((trng)->STS & TRNG_STS_DVIF_Msk)


/** @} end of group TRNG_EXPORTED_MACROS */

/** @addtogroup TRNG_EXPORTED_FUNCTIONS TRNG Exported Functions
  @{
*/
/*----------------------------------------------------------------------------------------------*/
/*  Functions                                                                                   */
/*----------------------------------------------------------------------------------------------*/
int32_t TRNG_Open(void);
int32_t TRNG_GenWord(uint32_t *u32RndNum);
int32_t TRNG_GenBignum(uint8_t u8BigNum[], int32_t i32Len);
int32_t TRNG_GenBignumHex(char cBigNumHex[], int32_t i32Len);
/** @} end of group TRNG_EXPORTED_FUNCTIONS */
/** @} end of group TRNG_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif  /* __TRNG_H__ */
