/**************************************************************************//**
 * @file    otfc.h
 * @version V1.00
 * @brief   OTFC driver header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#ifndef __OTFC_H__
#define __OTFC_H__

#include <stdint.h>

/*----------------------------------------------------------------------------*/
/* Include related headers                                                    */
/*----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
    @{
*/

/** @addtogroup OTFC_Driver OTFC Driver
    @{
*/

/** @addtogroup OTFC_EXPORTED_CONSTANTS OTFC Exported Constants
    @{
*/

/* OTFC Wait State Timeout Counter. */
#define OTFC_TIMEOUT                    SystemCoreClock /*!< OTFC time-out counter (1 second time-out) */

/* OTFC Define Error Code */
#define OTFC_OK                         ( 0L)   /*!< OTFC operation OK */
#define OTFC_ERR_FAIL                   (-1L)   /*!< OTFC operation failed */
#define OTFC_ERR_TIMEOUT                (-2L)   /*!< OTFC operation abort due to timeout error */

/*----------------------------------------------------------------------------*/
/* OTFC_CTL constant definitions                                              */
/*----------------------------------------------------------------------------*/
#define OTFC_CTL_PR_Pos                 (8)     /*!< OTFC CTL Protection Region BIT SHIFT */

#define OTFC_CTL_RST_Pos                (1)
#define OTFC_CTL_SWAP_Pos               (2)
#define OTFC_CTL_INT_Pos                (7)

#define OTFC_STS_KSERR_Pos(pr)          (((pr) * OTFC_CTL_PR_Pos) + 5)
#define OTFC_STS_KSERR_Msk(pr)          (0x1ul << OTFC_STS_KSERR_Pos(pr))

#define OTFC_STS_DFAE_Pos(pr)           (((pr) * OTFC_CTL_PR_Pos) + 6)
#define OTFC_STS_DFAE_Mak(pr)           (0x1ul << OTFC_STS_DFAE_Pos(pr))

#define OTFC_STS_IF_Pos(pr)             (((pr) * OTFC_CTL_PR_Pos) + 7)
#define OTFC_STS_IF_Msk(pr)             (0x1ul << OTFC_STS_IF_Pos(pr))

#define OTFC_PR_0                       (0)     /*!< OTFC Protection Region 0 */
#define OTFC_PR_1                       (1)     /*!< OTFC Protection Region 1 */
#define OTFC_PR_2                       (2)     /*!< OTFC Protection Region 2 */
#define OTFC_PR_3                       (3)     /*!< OTFC Protection Region 3 */

#define OTFC_KEY_SRC_REG                (0x00)    /*!< OTFC Protection Region key source from register */
#define OTFC_KEY_SRC_KS                 (0x01)    /*!< OTFC Protection Region key source from key store */

#define OTFC_KS_SRC_SRAM                (0x00)    /*!< Key is read from the SRAM of Key Store. */
#define OTFC_KS_SRC_OTP                 (0x02)    /*!< Key is read from the OTP of Key Store. */

/**
  * @brief  Enable OTFC Protection Region.
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * \hideinitializer
  */
#define OTFC_ENABLE_PR(otfc, pr)    (otfc->CTL |= (0x1ul << ((pr) * OTFC_CTL_PR_Pos)))

/**
 * @brief   Disable Protection Region
 * @param[in] otfc  The base address of OTFC module.
 * @param[in] pr    Protection Region.
 *                  - \ref OTFC_PR_0
 *                  - \ref OTFC_PR_1
 *                  - \ref OTFC_PR_2
 *                  - \ref OTFC_PR_3
 *
 */
#define OTFC_DISABLE_PR(otfc, pr)   (otfc->CTL &= ~(0x1ul << ((pr) * OTFC_CTL_PR_Pos)))

/**
  * @brief  Reset OTFC Protection Region.
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * \hideinitializer
  */
#define OTFC_RESET_PR(otfc, pr) \
    (otfc->CTL |= (0x1ul << (((pr) * OTFC_CTL_PR_Pos) + OTFC_CTL_RST_Pos)))

/**
 * @brief  Enable Swap Key, Scramble and Nonce.
 * @param[in] otfc  The base address of OTFC module.
 * @param[in] pr    Protection Region.
 *                  - \ref OTFC_PR_0
 *                  - \ref OTFC_PR_1
 *                  - \ref OTFC_PR_2
 *                  - \ref OTFC_PR_3
 * \hideinitializer
 */
#define OTFC_ENABLE_KEYSWAP(otfc, pr)  \
    (otfc->CTL |= (0x1ul << (((pr) *OTFC_CTL_PR_Pos) + OTFC_CTL_SWAP_Pos)))

/**
  * @brief  Disable Swap Key, Scramble and Nonce.
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * \hideinitializer
  */
#define OTFC_DISABLE_KEYSWAP(otfc, pr) \
    (otfc->CTL &= ~(0x1ul << (((pr) * OTFC_CTL_PR_Pos) + OTFC_CTL_SWAP_Pos)))

/**
  * @brief  Enable Interrupt.
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * \hideinitializer
  */
#define OTFC_ENABLE_INT(otfc, pr)   \
    (otfc->CTL |= (0x1ul << (((pr) * OTFC_CTL_PR_Pos) + OTFC_CTL_INT_Pos)))

/**
  * @brief  Disable Interrupt.
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * \hideinitializer
  */
#define OTFC_DISABLE_INT(otfc, pr)  \
    (otfc->CTL &= ~(0x1ul << (((pr) * OTFC_CTL_PR_Pos) + OTFC_CTL_INT_Pos)))

/**
  * @brief  Get Busy Status. (Read Only)
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * \hideinitializer
  */
#define OTFC_GET_BUSY(otfc, pr)   \
    ((otfc->STS & (0x1ul << ((pr) * OTFC_CTL_PR_Pos))) >> ((pr) * OTFC_CTL_PR_Pos))

/**
  * @brief  Get Access Key Store Error Flag. (Read Only)
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * \hideinitializer
  */
#define OTFC_GET_KSERR(otfc, pr)    \
    ((otfc->STS & (OTFC_STS_KSERR_Msk(pr))) >> (OTFC_STS_KSERR_Pos(pr)))

/**
* @brief    Get Differential Fault Attack Error Flag.
* @param[in] otfc   The base address of OTFC module.
* @param[in] pr     Protection Region.
*                   - \ref OTFC_PR_0
*                   - \ref OTFC_PR_1
*                   - \ref OTFC_PR_2
*                   - \ref OTFC_PR_3
* \hideinitializer
*/
#define OTFC_GET_DFAERR(otfc, pr)   \
    ((otfc->STS & (OTFC_STS_DFAE_Mak(pr))) >> (OTFC_STS_DFAE_Pos(pr)))

/**
  * @brief  Get Interrupt Flag.
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * \hideinitializer
  */
#define OTFC_GET_IF(otfc, pr)   \
    ((otfc->STS & (OTFC_STS_IF_Msk(pr))) >> (OTFC_STS_IF_Msk(pr)))

/**
  * @brief  Set Protection Start Address.
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * @param[in] saddr is the protection start address.
  * @note   The start address should be located at 128-bit boundary.
  *         In other words, bit 3, 2, 1 and 0 of SADDR are ignored.
  * \hideinitializer
  */
#define OTFC_SET_START_ADDR(otfc, pr, saddr)    (otfc->PR[(pr)].SADDR = ((saddr) & ~0xF))

/**
  * @brief  Set Protrction End Address.
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * @param[in] eaddr is the protection end address.
  * @note   The end address should be located at 128-bit boundary.
  *         In other words, bit 3, 2, 1 and 0 of EADDR are ignored.
  * \hideinitializer
  */
#define OTFC_SET_END_ADDR(otfc, pr, eaddr)  (otfc->PR[(pr)].EADDR = ((eaddr) & ~0xF))

/**
  * @brief  Clear Key Store Control Setting.
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * \hideinitializer
  */
#define OTFC_CLEAR_KSCTRL(otfc, pr) \
    (otfc->PR[(pr)].KSCTL &= ~(0xFFul << OTFC_PR_KSCTL_NUM_Pos))

/**
  * @brief  Read the key source from the register.
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * \hideinitializer
  */
#define OTFC_ENABLE_REG_KEY(otfc, pr)   \
    (otfc->PR[(pr)].KSCTL &= ~(OTFC_PR_KSCTL_NUM_Msk | OTFC_PR_KSCTL_RSRC_Msk))

/**
  * @brief  Read the key source from the key store in SRAM.
  * @param[in] otfc     The base address of OTFC module.
  * @param[in] pr       Protection Region.
  *                     - \ref OTFC_PR_0
  *                     - \ref OTFC_PR_1
  *                     - \ref OTFC_PR_2
  *                     - \ref OTFC_PR_3
  * @param[in] key_num  key number of key store.
  * \hideinitializer
  */
#define OTFC_ENABLE_KS_SRAM(otfc, pr, key_num)                                    \
    (otfc->PR[(pr)].KSCTL = (otfc->PR[(pr)].KSCTL & ~(OTFC_PR_KSCTL_NUM_Msk)) |   \
                            (((key_num) << OTFC_PR_KSCTL_NUM_Pos) | OTFC_PR_KSCTL_RSRC_Msk))

/**
  * @brief  Read the key source from the key store in OTP.
  * @param[in] otfc     The base address of OTFC module.
  * @param[in] pr       Protection Region.
  *                     - \ref OTFC_PR_0
  *                     - \ref OTFC_PR_1
  *                     - \ref OTFC_PR_2
  *                     - \ref OTFC_PR_3
  * @param[in] key_num  key number of key store.
  * \hideinitializer
  */
#define OTFC_ENABLE_KS_OTP(otfc, pr, key_num)                                        \
    (otfc->PR[(pr)].KSCTL = (otfc->PR[(pr)].KSCTL & ~(OTFC_PR_KSCTL_NUM_Msk)) |      \
                            (((key_num) << OTFC_PR_KSCTL_NUM_Pos) |                  \
                             (OTFC_KS_SRC_OTP << OTFC_PR_KSCTL_RSSRC_Pos) |          \
                             OTFC_PR_KSCTL_RSRC_Msk))

/**
  * @brief  The KEY keeps the security key for AES .
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * @param[in] key  The KEY keeps the security key for AES.
  * \hideinitializer
  */
#define OTFC_SET_KEY0(otfc, pr, key)    (otfc->PR[(pr)].KEY0 = (key))

/**
  * @brief  The KEY keeps the security key for AES.
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * @param[in] key  The KEY keeps the security key for AES.
  * \hideinitializer
  */
#define OTFC_SET_KEY1(otfc, pr, key)    (otfc->PR[(pr)].KEY1 = (key))

/**
  * @brief  The KEY keeps the security key for AES.
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * @param[in] key The KEY keeps the security key for AES.
  * \hideinitializer
  */
#define OTFC_SET_KEY2(otfc, pr, key)    (otfc->PR[(pr)].KEY2 = (key))

/**
  * @brief  The KEY keeps the security key for AES.
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * @param[in] key  The KEY keeps the security key for AES.
  * \hideinitializer
  */
#define OTFC_SET_KEY3(otfc, pr, key)    (otfc->PR[(pr)].KEY3 = (key))

/**
  * @brief  The scramble key for address cipher.
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * @param[in] key  The KEY keeps the security key for AES.
  * \hideinitializer
  */
#define OTFC_SET_SCRAMBLE(otfc, pr, key)    (otfc->PR[(pr)].SCRAMBLE = (key))

/**
  * @brief  Three nonces are for the address cipher function in OTFC.
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * @param[in] key The KEY keeps the security key for AES.
  * \hideinitializer
  */
#define OTFC_SET_NONCE0(otfc, pr, key)  (otfc->PR[(pr)].NONCE0 = (key))

/**
  * @brief  Three nonces are for the address cipher function in OTFC.
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * @param[in] key The KEY keeps the security key for AES.
  * \hideinitializer
  */
#define OTFC_SET_NONCE1(otfc, pr, key)  (otfc->PR[(pr)].NONCE1 = (key))

/**
  * @brief  Three nonces are for the address cipher function in OTFC.
  * @param[in] otfc The base address of OTFC module.
  * @param[in] pr   Protection Region.
  *                 - \ref OTFC_PR_0
  *                 - \ref OTFC_PR_1
  *                 - \ref OTFC_PR_2
  *                 - \ref OTFC_PR_3
  * @param[in] key The KEY keeps the security key for AES.
  * \hideinitializer
  */
#define OTFC_SET_NONCE2(otfc, pr, key)  (otfc->PR[(pr)].NONCE2 = (key))

/** @} end of group OTFC_EXPORTED_CONSTANTS */

/** @addtogroup OTFC_EXPORTED_FUNCTIONS OTFC Exported Functions
  @{
*/
int32_t OTFC_SetKeyFromKeyReg(OTFC_T *otfc,
                              uint32_t *pau32KeyTable,
                              uint32_t u32PR,
                              uint32_t u32SAddr,
                              uint32_t u32PRSize);

int32_t OTFC_SetKeyFromKeyStore(OTFC_T *otfc, uint32_t u32PR,
                                uint32_t u32SAddr, uint32_t u32PRSize,
                                uint32_t u32KeyNum, uint32_t u32KeySrc);

int32_t OTFC_SetScrambleNum(OTFC_T *otfc, uint32_t u32PR, uint32_t u32Scramble);
int32_t OTFC_SetNonceNum(OTFC_T *otfc, uint32_t u32PR, uint32_t u32Nonce0, uint32_t u32Nonce1, uint32_t u32Nonce2);

/** @} end of group OTFC_EXPORTED_FUNCTIONS */
/** @} end of group OTFC_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __OTFC_H__ */
