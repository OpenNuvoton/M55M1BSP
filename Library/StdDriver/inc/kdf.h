/**************************************************************************//**
 * @file     kdf.h
 * @version  V1.00
 * @brief    KDF driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __KDF_H__
#define __KDF_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
    @{
*/

/** @addtogroup KDF_Driver KDF Driver
    @{
*/

/** @addtogroup KDF_EXPORTED_CONSTANTS KDF Exported Constants
    @{
*/
#define KDF_KEYOUT_TO_REG       (0x0UL << KDF_KSCTL_WDST_Pos)   /*!< Output key to register                */
#define KDF_KEYOUT_TO_KS        (0x1UL << KDF_KSCTL_WDST_Pos)   /*!< Output key to Key Store               */
#define KDF_KS_OWNER_AES        (0x0UL << KDF_KSCTL_OWNER_Pos)  /*!< Only for AES engine use               */
#define KDF_KS_OWNER_HMAC       (0x1UL << KDF_KSCTL_OWNER_Pos)  /*!< Only for HMAC engine use              */
#define KDF_KS_OWNER_ECC        (0x4UL << KDF_KSCTL_OWNER_Pos)  /*!< Only for ECC engine use               */
#define KDF_KS_OWNER_CPU        (0x5UL << KDF_KSCTL_OWNER_Pos)  /*!< Only for CPU use                      */
#define KDF_KS_PRIV             (0x1UL << KDF_KSCTL_PRIV_Pos)   /*!< Privilege key                         */
#define KDF_KS_NON_PRIV         (0x0UL << KDF_KSCTL_PRIV_Pos)   /*!< Non-privilege key                     */
#define KDF_KS_SECURE           (0x1UL << KDF_KSCTL_TRUST_Pos)  /*!< Secure key                            */
#define KDF_KS_NON_SECURE       (0x0UL << KDF_KSCTL_TRUST_Pos)  /*!< Non-Secure key                        */

#define KDF_KS_KEYSIZE_128     (0x0UL)                          /*!< Key size 128 bits                     */
#define KDF_KS_KEYSIZE_163     (0x1UL)                          /*!< Key size 163 bits                     */
#define KDF_KS_KEYSIZE_192     (0x2UL)                          /*!< Key size 192 bits                     */
#define KDF_KS_KEYSIZE_224     (0x3UL)                          /*!< Key size 224 bits                     */
#define KDF_KS_KEYSIZE_233     (0x4UL)                          /*!< Key size 233 bits                     */
#define KDF_KS_KEYSIZE_255     (0x5UL)                          /*!< Key size 255 bits                     */
#define KDF_KS_KEYSIZE_256     (0x6UL)                          /*!< Key size 256 bits                     */
#define KDF_KS_KEYSIZE_283     (0x7UL)                          /*!< Key size 283 bits                     */
#define KDF_KS_KEYSIZE_384     (0x8UL)                          /*!< Key size 384 bits                     */
#define KDF_KS_KEYSIZE_409     (0x9UL)                          /*!< Key size 409 bits                     */
#define KDF_KS_KEYSIZE_512     (0xAUL)                          /*!< Key size 512 bits                     */
#define KDF_KS_KEYSIZE_521     (0xBUL)                          /*!< Key size 521 bits                     */
#define KDF_KS_KEYSIZE_571     (0xCUL)                          /*!< Key size 571 bits                     */

#define KDF_KEYIN_FROM_REG      (0UL << KDF_CTL_KEYINSEL_Pos)   /*!< KDF key input from register           */
#define KDF_KEYIN_FROM_NVM      (1UL << KDF_CTL_KEYINSEL_Pos)   /*!< KDF key input from NVM (KS OTP Key15) */
#define KDF_SALT_FROM_REG       (0UL << KDF_CTL_SALTSEL_Pos)    /*!< KDF salt from register                */
#define KDF_SALT_FROM_RANDOM    (1UL << KDF_CTL_SALTSEL_Pos)    /*!< KDF salt from random value            */
#define KDF_LABEL_FROM_REG      (0UL << KDF_CTL_LBSEL_Pos)      /*!< KDF label from register               */
#define KDF_LABEL_FROM_RANDOM   (1UL << KDF_CTL_LBSEL_Pos)      /*!< KDF label from random value           */
#define KDF_CONTEXT_FROM_REG    (0UL << KDF_CTL_CTXTSEL_Pos)    /*!< KDF context from register             */
#define KDF_CONTEXT_FROM_RANDOM (1UL << KDF_CTL_CTXTSEL_Pos)    /*!< KDF context from random value         */

#define KDF_TIMEOUT             SystemCoreClock                 /*!< 1 second time-out \hideinitializer    */

typedef enum
{
    eKDF_ERRCODE_SUCCESS        =  0,
    eKDF_ERRCODE_FAIL           = -1,                           /*!< KDF operation failed                  */
    eKDF_ERRCODE_TIMEOUT        = -2,                           /*!< KDF operation timeout error           */
    eKDF_ERRCODE_BUSY           = -3,                           /*!< KDF is in busy state                  */
    eKDF_ERRCODE_INVALID_PARAM  = -4,                           /*!< Invalid parameters                    */
} E_KDF_ERRCODE;

typedef enum
{
    eKDF_MODE_HKDF  = (0UL << KDF_CTL_MODE_Pos),                /*!< HKDF (RFC 5869)                       */
    eKDF_MODE_KBKDF = (1UL << KDF_CTL_MODE_Pos),                /*!< KBKDF (NIST SP 800-108)               */
} E_KDF_MODE;

/** @} end of group KDF_EXPORTED_CONSTANTS */

extern int32_t  g_KDF_i32ErrCode;                               /*!< KDF global error code                 */

/** @addtogroup KDF_EXPORTED_FUNCTIONS KDF Exported Functions
    @{
*/

void KDF_SetKeyInput(const uint8_t pu8KeyInput[], uint32_t u32ByteCnt);
void KDF_SetSalt(const uint8_t pu8Salt    [], uint32_t u32ByteCnt);
void KDF_SetLabel(const uint8_t pu8Label   [], uint32_t u32ByteCnt);
void KDF_SetContext(const uint8_t pu8Context [], uint32_t u32ByteCnt);
int32_t KDF_GetKeyBitSize(uint32_t u32KeySizeOpt);
int32_t KDF_DeriveKey(E_KDF_MODE eMode, uint32_t u32DeriveKeyParam, uint32_t u32KeyBitSize, uint32_t *pu32KeyOut);
int32_t KDF_DeriveKeyToKS(KS_MEM_Type eMemType, E_KDF_MODE eMode, uint32_t u32DeriveKeyParam, uint32_t u32KeySizeSel, uint32_t u32KeyMeta);

/** @} end of group KDF_EXPORTED_FUNCTIONS */
/** @} end of group KDF_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __KDF_H__ */
