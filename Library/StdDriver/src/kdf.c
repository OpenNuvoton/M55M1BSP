/**************************************************************************//**
 * @file     kdf.c
 * @version  V1.00
 * @brief    Key Derivation Function driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <string.h>
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
    @{
*/

/** @addtogroup KDF_Driver KDF Driver
    @{
*/

int32_t  g_KDF_i32ErrCode = eKDF_ERRCODE_SUCCESS;   /*!< KDF global error code */

/** @addtogroup KDF_EXPORTED_FUNCTIONS KDF Exported Functions
    @{
*/

/**
  * @brief      Get key bit size
  * @param[in]  u32KeySizeSel       Output key size selection. It could be:
  *                                 \ref KDF_KS_KEYSIZE_128
  *                                 \ref KDF_KS_KEYSIZE_163
  *                                 \ref KDF_KS_KEYSIZE_192
  *                                 \ref KDF_KS_KEYSIZE_224
  *                                 \ref KDF_KS_KEYSIZE_233
  *                                 \ref KDF_KS_KEYSIZE_255
  *                                 \ref KDF_KS_KEYSIZE_256
  *                                 \ref KDF_KS_KEYSIZE_283
  *                                 \ref KDF_KS_KEYSIZE_384
  *                                 \ref KDF_KS_KEYSIZE_409
  *                                 \ref KDF_KS_KEYSIZE_512
  *                                 \ref KDF_KS_KEYSIZE_521
  *                                 \ref KDF_KS_KEYSIZE_571
  * @return     eKDF_ERRCODE_INVALID_PARAM  Invalid key size selection
  *             key bit size
  * @details    This function is used to get key size selection.
  */
int32_t KDF_GetKeyBitSize(uint32_t u32KeySizeSel)
{
    uint32_t au32KeyBitLenTbl[] = { 128, 163, 192, 224, 233, 255, 256, 283, 384, 409, 512, 521, 571 };

    if (u32KeySizeSel >= (sizeof(au32KeyBitLenTbl) / sizeof(au32KeyBitLenTbl[0])))
        return eKDF_ERRCODE_INVALID_PARAM;

    return au32KeyBitLenTbl[u32KeySizeSel];
}

/**
  * @brief      Set key input
  * @param[in]  pu8KeyInput     Byte buffer contain key input
  * @param[in]  u32ByteCnt      Specify byte count of pu8InputKey and its value must <= 32
  * @details    This function is used to set key input.
  */
void KDF_SetKeyInput(const uint8_t pu8KeyInput[], uint32_t u32ByteCnt)
{
    if (u32ByteCnt > sizeof(KDF->KEYIN))
        u32ByteCnt = sizeof(KDF->KEYIN);

    memcpy((void *)KDF->KEYIN, (void *)pu8KeyInput, u32ByteCnt);
}

/**
  * @brief      Set key salt
  * @param[in]  pu8Salt         Byte buffer contain key salt
  * @param[in]  u32ByteCnt      Specify byte count of pu8Salt and its value must <= 32
  * @details    This function is used to set key salt.
  */
void KDF_SetSalt(const uint8_t pu8Salt[], uint32_t u32ByteCnt)
{
    if (u32ByteCnt > sizeof(KDF->SALT))
        u32ByteCnt = sizeof(KDF->SALT);

    memcpy((void *)KDF->SALT, (void *)pu8Salt, u32ByteCnt);
}

/**
  * @brief      Set key label
  * @param[in]  pu8Label        Byte buffer contain key label
  * @param[in]  u32ByteCnt      Specify byte count of pu8Label and its value must <= 12
  * @details    This function is used to set key label.
  */
void KDF_SetLabel(const uint8_t pu8Label[], uint32_t u32ByteCnt)
{
    if (u32ByteCnt > sizeof(KDF->LABEL))
        u32ByteCnt = sizeof(KDF->LABEL);

    memcpy((void *)KDF->LABEL, (void *)pu8Label, u32ByteCnt);
}

/**
  * @brief      Set key context
  * @param[in]  pu8Context      Byte buffer contain key context
  * @param[in]  u32ByteCnt      Specify byte count of pu8Ctxt and its value must <= 16
  * @details    This function is used to set key context.
  */
void KDF_SetContext(const uint8_t pu8Context[], uint32_t u32ByteCnt)
{
    if (u32ByteCnt > sizeof(KDF->CTXT))
        u32ByteCnt = sizeof(KDF->CTXT);

    memcpy((void *)KDF->CTXT, (void *)pu8Context, u32ByteCnt);
}

/**
  * @brief        Derive key
  *
  * @param[in]    eMode                 The pointer of the specified UART module.
  *                                     - \ref E_KDF_MODE
  *                                            eKDF_MODE_HKDF : HKDF  (RFC 5869)
  *                                            eKDF_MODE_KBKDF: KBKDF (NIST SP 800-108)
  * @param[in]    u32DeriveKeyParam     To specify keyin, salt, label and context selection.
  *                                     If KDF_xxx_FROM_REG is set, users can use these functions -
  *                                         KDF_SetKeyInput, KDF_SetSalt, KDF_SetLabel and KDF_SetCtxt
  *                                     to write key parameters before deriving key
  *                                     It could be the combine of
  *                                     \ref KDF_KEYIN_FROM_REG   / \ref KDF_KEYIN_FROM_NVM (KS OTP Key15)
  *                                     \ref KDF_SALT_FROM_REG    / \ref KDF_SALT_FROM_RANDOM
  *                                     \ref KDF_LABEL_FROM_REG   / \ref KDF_LABEL_FROM_RANDOM
  *                                     \ref KDF_CONTEXT_FROM_REG / \ref KDF_CONTEXT_FROM_RANDOM
  * @param[in]    u32KeyBitSize         To specify output key bit length and its maximum value must <= 65280
  * @param[out]   pu32KeyOut            Buffer to store output key and buffer size must >= u32KeyBitSize and word alignment
  *
  * @return       eKDF_ERRCODE_SUCCESS: Success
  *               Others: Fail
  * @details      Derive key with specified key parameters and key length
  */
int32_t KDF_DeriveKey(E_KDF_MODE eMode, uint32_t u32DeriveKeyParam, uint32_t u32KeyBitSize, uint32_t *pu32KeyOut)
{
    int32_t  i;
    uint32_t u32Idx = 0;
    int32_t  i32LeftKeyBitSize = u32KeyBitSize;
    uint32_t u32TimeOutCount, u32ByteCnt;

    KDF->CTL   = eMode | u32DeriveKeyParam;
    KDF->KLEN  = u32KeyBitSize;
    KDF->KSCTL = KDF_KEYOUT_TO_REG;

    while (i32LeftKeyBitSize > 0)
    {
        if (i32LeftKeyBitSize == u32KeyBitSize)
            KDF->CTL |= KDF_CTL_START_Msk;  /* Trigger to start key derive operation */
        else
            KDF->CTL |= KDF_CTL_NEXT_Msk;   /* Trigger to derive next partial key output */

        /* Wait until KDF HMAC engine become idle */
        u32TimeOutCount = KDF_TIMEOUT;

        while (KDF->STS & KDF_STS_HMACBUSY_Msk)
        {
            if (--u32TimeOutCount == 0)
            {
                return eKDF_ERRCODE_TIMEOUT;
            }
        }

        /* Check status */
        if (KDF->STS & (KDF_STS_NEXTERR_Msk | KDF_STS_KSERR_Msk))
            return eKDF_ERRCODE_FAIL;

        /* Store output key to pu32KeyOut */
        u32ByteCnt = (i32LeftKeyBitSize / 8) + ((i32LeftKeyBitSize % 8) > 0);

        /* Maximum byte count of derived key is 32 bytes per derivation */
        if (u32ByteCnt > 32)
            u32ByteCnt = 32;

        for (i = 0; i < u32ByteCnt; i += 4)
        {
            pu32KeyOut[(u32Idx + i) / 4] = KDF->KEYOUT[i / 4];
        }

        u32Idx += i;
        i32LeftKeyBitSize -= (u32ByteCnt * 8);
    }

    /* Stop KDF derive opeation */
    KDF->CTL |= KDF_CTL_FINISH_Msk;
    /* Wait until KDF become idle */
    u32TimeOutCount = KDF_TIMEOUT;

    while (KDF->STS & (KDF_STS_BUSY_Msk | KDF_STS_HMACBUSY_Msk))
    {
        if (--u32TimeOutCount == 0)
        {
            return eKDF_ERRCODE_TIMEOUT;
        }
    }

    return eKDF_ERRCODE_SUCCESS;
}

/**
  * @brief        Derive key to Key Store
  *
  * @param[in]    eMemType           The memory type. It could be:
  *                                  \ref KS_SRAM
  *                                  \ref KS_FLASH
  * @param[in]    eMode              The operation mode. It could be:
  *                                  \ref eKDF_MODE_HKDF
  *                                  \ref eKDF_MODE_KBKDF
  * @param[in]    u32DeriveKeyParam  To specify keyin, salt, label and context selection.
  *                                  If KDF_xxx_FROM_REG is set, users can use these functions -
  *                                    KDF_SetKeyInput, KDF_SetSalt, KDF_SetLabel and KDF_SetCtxt
  *                                  to write key parameters before deriving key
  *                                  \ref KDF_KEYIN_FROM_REG   / \ref KDF_KEYIN_FROM_NVM (KS OTP Key15)
  *                                  \ref KDF_SALT_FROM_REG    / \ref KDF_SALT_FROM_RANDOM
  *                                  \ref KDF_LABEL_FROM_REG   / \ref KDF_LABEL_FROM_RANDOM
  *                                  \ref KDF_CONTEXT_FROM_REG / \ref KDF_CONTEXT_FROM_RANDOM
  * @param[in]    u32KeySizeSel      Output key size selection. It could be:
  *                                  \ref KDF_KS_KEYSIZE_128
  *                                  \ref KDF_KS_KEYSIZE_163
  *                                  \ref KDF_KS_KEYSIZE_192
  *                                  \ref KDF_KS_KEYSIZE_224
  *                                  \ref KDF_KS_KEYSIZE_233
  *                                  \ref KDF_KS_KEYSIZE_255
  *                                  \ref KDF_KS_KEYSIZE_256
  *                                  \ref KDF_KS_KEYSIZE_283
  *                                  \ref KDF_KS_KEYSIZE_384
  *                                  \ref KDF_KS_KEYSIZE_409
  *                                  \ref KDF_KS_KEYSIZE_512
  *                                  \ref KDF_KS_KEYSIZE_521
  *                                  \ref KDF_KS_KEYSIZE_571
  * @param[in]    u32KeyMeta         The metadata of the key. It could be the combine of
  *                                  \ref KDF_KS_OWNER_AES / \ref KDF_KS_OWNER_HMAC / \ref KDF_KS_OWNER_ECC / \ref KDF_KS_OWNER_CPU
  *                                  \ref KDF_KS_NON_PRIV   / \ref KDF_KS_PRIV
  *                                  \ref KDF_KS_NON_SECURE / \ref KDF_KS_SECURE
  * @return       Index of the key. Failed when index < 0.
  * @details      Derive key to Key Store with specified key parameters and key length
  */
int32_t KDF_DeriveKeyToKS(KS_MEM_Type eMemType, E_KDF_MODE eMode, uint32_t u32DeriveKeyParam, uint32_t u32KeySizeSel, uint32_t u32KeyMeta)
{
    uint32_t u32TimeOutCount;

    KDF->CTL    = eMode | u32DeriveKeyParam;
    KDF->KSCTL  = (eMemType << KDF_KSCTL_WSDST_Pos) | KDF_KEYOUT_TO_KS | u32KeyMeta;
    KDF->KSSIZE = u32KeySizeSel;
    KDF->CTL    = KDF->CTL | KDF_CTL_START_Msk;

    /* Waiting for busy */
    u32TimeOutCount = KDF_TIMEOUT;

    while (KDF->STS & (KDF_STS_BUSY_Msk | KDF_STS_HMACBUSY_Msk))
    {
        if (--u32TimeOutCount == 0)
        {
            return eKDF_ERRCODE_TIMEOUT;
        }
    }

    if (KDF->STS & (KDF_STS_NEXTERR_Msk | KDF_STS_KSERR_Msk))
        return eKDF_ERRCODE_FAIL;

    return (KDF->KSSTS & KDF_KSSTS_NUM_Msk);
}

/** @} end of group KDF_EXPORTED_FUNCTIONS */
/** @} end of group KDF_Driver */
/** @} end of group Standard_Driver */
