/**************************************************************************//**
 * @file     otfc.c
 * @version  V1.00
 * @brief    OTFC driver source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup OTFC_Driver OTFC Driver
  @{
*/

/** @addtogroup OTFC_EXPORTED_FUNCTIONS OTFC Exported Functions
  @{
*/
static int32_t otfc_wait_busy(OTFC_T *otfc, uint32_t u32PR)
{
    volatile int32_t i32Timeout = OTFC_TIMEOUT;

    /* Wait Protection Region 0 ~ 3 not Busy*/
    while (OTFC_GET_BUSY(otfc, u32PR) == 1)
    {
        if (i32Timeout-- <= 0)
        {
            return OTFC_ERR_TIMEOUT;
        }
    }

    return OTFC_OK;
}

/**
 * @brief   Set Scramble Number
 *
 * @param otfc  is the base address of OTFC module.
 * @param[in] u32PR Protection Region.
 *                  - \ref OTFC_PR_0
 *                  - \ref OTFC_PR_1
 *                  - \ref OTFC_PR_2
 *                  - \ref OTFC_PR_3
 * @param[in] u32Scramble   if the Protection Region Scramble key
 * @return int32_t
 */
int32_t OTFC_SetScrambleNum(OTFC_T *otfc, uint32_t u32PR, uint32_t u32Scramble)
{
    /* Wait Protection Region 0 ~ 3 not Busy*/
    if (otfc_wait_busy(otfc, u32PR) != OTFC_OK)
    {
        return OTFC_ERR_TIMEOUT;
    }

    /* Set Protection Region Scramble Number */
    OTFC_SET_SCRAMBLE(otfc, u32PR, u32Scramble);

    return OTFC_OK;
}

/**
 * @brief Set nonce number
 *
 * @param otfc  is the base address of OTFC module.
 * @param[in] u32PR Protection Region.
 *                  - \ref OTFC_PR_0
 *                  - \ref OTFC_PR_1
 *                  - \ref OTFC_PR_2
 *                  - \ref OTFC_PR_3
 * @param[in] u32Nonce0 is the Protection Region Nonce Word 0 Key
 * @param[in] u32Nonce1 is the Protection Region Nonce Word 1 Key
 * @param[in] u32Nonce2 is the Protection Region Nonce Word 2 Key
 * @return int32_t
 */
int32_t OTFC_SetNonceNum(OTFC_T *otfc, uint32_t u32PR,
                         uint32_t u32Nonce0, uint32_t u32Nonce1, uint32_t u32Nonce2)
{
    /* Wait Protection Region 0 ~ 3 not Busy*/
    if (otfc_wait_busy(otfc, u32PR) != OTFC_OK)
    {
        return OTFC_ERR_TIMEOUT;
    }

    /* Set Protection Region Nonce0 ~ 2 Number */

    OTFC_SET_NONCE0(otfc, u32PR, u32Nonce0);
    OTFC_SET_NONCE1(otfc, u32PR, u32Nonce1);
    OTFC_SET_NONCE2(otfc, u32PR, u32Nonce2);

    return OTFC_OK;
}

/**
 * @brief   Enable OTFC Protection Region 0 ~ 3 and Use Key Source From Key store.
 * @param[in] otfc  is the base address of OTFC module.
 * @param[in] u32PR Protection Region.
 *                  - \ref OTFC_PR_0
 *                  - \ref OTFC_PR_1
 *                  - \ref OTFC_PR_2
 *                  - \ref OTFC_PR_3
 * @param[in] u32SAddr  is the protection region start address
 * @param[in] u32PRSize is the protection region size
 * @param[in] u32KeyNum is the KEY Store of KEY number
 * @param[in] u32KeySrc is the Key Source form Key Store SRAM or OTP
 *                      - OTFC_KS_SRC_SRAM
 *                      - OTFC_KS_SRC_OTP
 * @note      Keystore key number only sets 128 bits to keys 0 ~ 3,
 *            the user must manually set the scrambling and nonce number.
 */
int32_t OTFC_SetKeyFromKeyStore(OTFC_T *otfc, uint32_t u32PR,
                                uint32_t u32SAddr, uint32_t u32PRSize,
                                uint32_t u32KeyNum, uint32_t u32KeySrc)
{
    /* Wait Protection Region 0 ~ 3 not Busy*/
    if (otfc_wait_busy(otfc, u32PR) != OTFC_OK)
    {
        return OTFC_ERR_TIMEOUT;
    }

    /* Reset Protection Region 0 ~ 3 */
    OTFC_RESET_PR(otfc, u32PR);

    /* Set Protection Region Start and End Address */
    OTFC_SET_START_ADDR(otfc, u32PR, u32SAddr);
    OTFC_SET_END_ADDR(otfc, u32PR, u32SAddr + u32PRSize);

    /* Clear protect region setting */
    OTFC_CLEAR_KSCTRL(otfc, u32PR);

    /* Set Key Store key number */
    if (u32KeySrc == OTFC_KS_SRC_SRAM)
    {
        OTFC_ENABLE_KS_SRAM(otfc, u32PR, u32KeyNum);
    }
    else if (u32KeySrc == OTFC_KS_SRC_OTP)
    {
        OTFC_ENABLE_KS_OTP(otfc, u32PR, u32KeyNum);
    }

    return OTFC_OK;
}

/**
 * @brief Use Key table set OTFC protection Region 0 ~ 3 key Source to register.
 * @param[in] otfc
 * @param[in] pau32KeyTable  OTFC key table.
 * @param[in] u32PR          Protection Region.
 *                           - \ref OTFC_PR_0
 *                           - \ref OTFC_PR_1
 *                           - \ref OTFC_PR_2
 *                           - \ref OTFC_PR_3
 * @param[in] u32SAddr       Protection region start address.
 * @param[in] u32PRSize      Protection region Size.
 */
int32_t OTFC_SetKeyFromKeyReg(OTFC_T *otfc, uint32_t *pau32KeyTable, uint32_t u32PR,
                              uint32_t u32SAddr, uint32_t u32PRSize)
{
    /* Wait Protection Region 0 ~ 3 not Busy*/
    if (otfc_wait_busy(otfc, u32PR) != OTFC_OK)
    {
        return OTFC_ERR_TIMEOUT;
    }

    /* Reset Protection Region 0 ~ 3 Setting*/
    OTFC_RESET_PR(otfc, u32PR);

    OTFC_CLEAR_KSCTRL(otfc, u32PR);

    /* Protection Region 0 ~ 3 key is read from registers OTFC_PRx_KEYx */
    OTFC_SET_START_ADDR(otfc, u32PR, u32SAddr);
    OTFC_SET_END_ADDR(otfc, u32PR, u32SAddr + u32PRSize);
    OTFC_SET_KEY0(otfc, u32PR, pau32KeyTable[0]);
    OTFC_SET_KEY1(otfc, u32PR, pau32KeyTable[1]);
    OTFC_SET_KEY2(otfc, u32PR, pau32KeyTable[2]);
    OTFC_SET_KEY3(otfc, u32PR, pau32KeyTable[3]);
    OTFC_SET_SCRAMBLE(otfc, u32PR, pau32KeyTable[4]);
    OTFC_SET_NONCE0(otfc, u32PR, pau32KeyTable[5]);
    OTFC_SET_NONCE1(otfc, u32PR, pau32KeyTable[6]);
    OTFC_SET_NONCE2(otfc, u32PR, pau32KeyTable[7]);

    return OTFC_OK;
}

/** @} end of group OTFC_EXPORTED_FUNCTIONS */
/** @} end of group OTFC_Driver */
/** @} end of group Standard_Driver */
