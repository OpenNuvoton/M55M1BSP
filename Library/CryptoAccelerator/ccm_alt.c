/*
 *  NIST SP800-38C compliant CCM implementation
 *
 *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
 *  Copyright (c) 2022, Nuvoton Technology Corporation
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  This file is part of mbed TLS (https://tls.mbed.org)
 */

/*
 * Definition of CCM:
 * http://csrc.nist.gov/publications/nistpubs/800-38C/SP800-38C_updated-July20_2007.pdf
 * RFC 3610 "Counter with CBC-MAC (CCM)"
 *
 * Related:
 * RFC 5116 "An Interface and Algorithms for Authenticated Encryption"
 */

#include "common.h"

#if defined(MBEDTLS_CCM_C)

#include "mbedtls/ccm.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/error.h"
#include <string.h>

#if defined(MBEDTLS_SELF_TEST) && defined(MBEDTLS_AES_C)
    #if defined(MBEDTLS_PLATFORM_C)
        #include "mbedtls/platform.h"
    #else
        #include <stdio.h>
        #define mbedtls_printf printf
    #endif /* MBEDTLS_PLATFORM_C */
#endif /* MBEDTLS_SELF_TEST && MBEDTLS_AES_C */

#if defined(MBEDTLS_CCM_ALT)

#define CCM_VALIDATE_RET( cond ) \
    MBEDTLS_INTERNAL_VALIDATE_RET( cond, MBEDTLS_ERR_CCM_BAD_INPUT )
#define CCM_VALIDATE( cond ) \
    MBEDTLS_INTERNAL_VALIDATE( cond )

#define CCM_ENCRYPT 1
#define CCM_DECRYPT 0




#define START       CRPT_AES_CTL_START_Msk
#define DMAEN       CRPT_AES_CTL_DMAEN_Msk
#define DMALAST     CRPT_AES_CTL_DMALAST_Msk
#define DMACC       CRPT_AES_CTL_DMACSCAD_Msk
#define START       CRPT_AES_CTL_START_Msk
#define FBIN        CRPT_AES_CTL_FBIN_Msk
#define FBOUT       CRPT_AES_CTL_FBOUT_Msk

#define GCM_MODE    (AES_MODE_GCM << CRPT_AES_CTL_OPMODE_Pos)
#define GHASH_MODE  (AES_MODE_GHASH << CRPT_AES_CTL_OPMODE_Pos)
#define CTR_MODE    (AES_MODE_CTR << CRPT_AES_CTL_OPMODE_Pos)




int32_t ToBigEndian(uint8_t *pbuf, uint32_t u32Size)
{
    uint32_t i;
    uint8_t u8Tmp;
    uint32_t u32Tmp;

    /* pbuf must be word alignment */
    if ((uint32_t)pbuf & 0x3)
    {
        printf("The buffer must be 32-bit alignment.");
        return -1;
    }

    while (u32Size >= 4)
    {
        u8Tmp = *pbuf;
        *(pbuf) = *(pbuf + 3);
        *(pbuf + 3) = u8Tmp;

        u8Tmp = *(pbuf + 1);
        *(pbuf + 1) = *(pbuf + 2);
        *(pbuf + 2) = u8Tmp;

        u32Size -= 4;
        pbuf += 4;
    }

    if (u32Size > 0)
    {
        u32Tmp = 0;

        for (i = 0; i < u32Size; i++)
        {
            u32Tmp |= *(pbuf + i) << (24 - i * 8);
        }

        *((uint32_t *)pbuf) = u32Tmp;
    }

    return 0;
}


/*
    CCM input format must be block alignment. The block size is 16 bytes.

    ----------------------------------------------------------------------
     Block B0
          Formatting of the Control Information and the Nonce
    ----------------------------------------------------------------------
    First block B_0:
    0        .. 0        flags
    1        .. iv_len   nonce (aka iv)
    iv_len+1 .. 15       length


    flags:
    With flags as (bits):
    7        0
    6        add present?
    5 .. 3   (t - 2) / 2
    2 .. 0   q - 1

*/

int32_t CCMPacker(const uint8_t *iv, uint32_t ivlen, const uint8_t *A, uint32_t alen, const uint8_t *P, uint32_t plen, uint8_t *pbuf, uint32_t *psize, uint32_t tlen, int32_t enc)
{
    uint32_t i, j;
    uint32_t alen_aligned, plen_aligned;
    uint32_t u32Offset = 0;
    uint8_t u8Tmp;
    uint32_t q;


    /* Flags in B0
    *With flags as(bits) :
        7        0
        6        add present ?
        5 .. 3   (t - 2) / 2
        2 .. 0   q - 1, q = 15 - nlen
    */

    q = 15 - ivlen;
    u8Tmp = (q - 1) | ((tlen - 2) / 2 << 3) | ((alen > 0) ? 0x40 : 0);
    pbuf[0] = u8Tmp;            // flags

    for (i = 0; i < ivlen; i++) // N
        pbuf[i + 1] = iv[i];

    for (i = ivlen + 1, j = q - 1; i < 16; i++, j--)   // Q
    {
        if (j >= 4)
            pbuf[i] = 0;
        else
        {
            pbuf[i] = (plen >> j * 8) & 0xfful;
        }
    }

    u32Offset = 16;

    /* Formatting addition data */
    /* alen. It is limited to be smaller than 2^16-2^8 */
    if (alen > 0)
    {
        pbuf[u32Offset] = (alen >> 8) & 0xfful;
        pbuf[u32Offset + 1] = alen & 0xfful;

        for (i = 0; i < alen; i++)
            pbuf[u32Offset + i + 2] = A[i];

        alen_aligned = ((alen + 2 + 15) / 16) * 16;

        for (i = u32Offset + 2 + alen; i < alen_aligned; i++)
        {
            pbuf[i] = (enc) ? 0 : 0xff; // padding zero or 0xff
        }

        u32Offset += alen_aligned;
    }

    /* Formatting payload */
    if (plen > 0)
    {
        plen_aligned = ((plen + 15) / 16) * 16;

        for (i = 0; i < plen; i++)
        {
            pbuf[u32Offset + i] = P[i];
        }

        for (; i < plen_aligned; i++)
        {
            pbuf[u32Offset + i] = 0; // padding zero
        }

        u32Offset += plen_aligned;
    }


    /* Formatting Ctr0 */
    pbuf[u32Offset] = q - 1; // Flags

    for (i = 0; i < ivlen; i++) // N
    {
        pbuf[u32Offset + 1 + i] = iv[i];
    }

    for (; i < 16; i++)
    {
        pbuf[u32Offset + 1 + i] = 0; // padding zero to block alignment
    }

    *psize = u32Offset;

    return 0;
}


static int32_t _CCM(mbedtls_ccm_context *ctx, int32_t enc, const uint8_t *iv, uint32_t ivlen, const uint8_t *A, uint32_t alen, const uint8_t *P, uint32_t plen, uint8_t *buf, uint8_t *tag,
                    uint32_t tlen)
{
    uint32_t size, plen_aligned;
    int32_t timeout = 0x1000000;
    uint32_t *pu32;
    uint32_t key[8], i;

    if (ivlen > 16)
        return -1;

    for (i = 0; i < 8; i++)
    {
        key[i] = CRPT->AES_KEY[i];
    }

    SYS->IPRST0 = SYS_IPRST0_CRPTRST_Msk;
    SYS->IPRST0 = 0;

    for (i = 0; i < 8; i++)
    {
        CRPT->AES_KEY[i] = key[i];
    }


    AES_ENABLE_INT(CRPT);

    /* Prepare the blocked buffer for GCM */
    memset(ctx->ccm_buf, 0, MAX_CCM_BUF);
    CCMPacker(iv, ivlen, A, alen, P, plen, ctx->ccm_buf, &size, tlen, enc);

    ToBigEndian(ctx->ccm_buf, size + 16);

    plen_aligned = (plen & 0xful) ? ((plen + 15) / 16) * 16 : plen;

    if (ctx->keySize == 16)
    {
        CRPT->AES_CTL = (enc << CRPT_AES_CTL_ENCRPT_Pos) |
                        (AES_MODE_CCM << CRPT_AES_CTL_OPMODE_Pos) |
                        (AES_KEY_SIZE_128 << CRPT_AES_CTL_KEYSZ_Pos) |
                        (AES_OUT_SWAP << CRPT_AES_CTL_OUTSWAP_Pos);
    }
    else if (ctx->keySize == 24)
    {
        CRPT->AES_CTL = (enc << CRPT_AES_CTL_ENCRPT_Pos) |
                        (AES_MODE_CCM << CRPT_AES_CTL_OPMODE_Pos) |
                        (AES_KEY_SIZE_192 << CRPT_AES_CTL_KEYSZ_Pos) |
                        (AES_OUT_SWAP << CRPT_AES_CTL_OUTSWAP_Pos);
    }
    else
    {
        CRPT->AES_CTL = (enc << CRPT_AES_CTL_ENCRPT_Pos) |
                        (AES_MODE_CCM << CRPT_AES_CTL_OPMODE_Pos) |
                        (AES_KEY_SIZE_256 << CRPT_AES_CTL_KEYSZ_Pos) |
                        (AES_OUT_SWAP << CRPT_AES_CTL_OUTSWAP_Pos);
    }


    pu32 = (uint32_t *)&ctx->ccm_buf[size];
    CRPT->AES_IV[0] = pu32[0];
    CRPT->AES_IV[1] = pu32[1];
    CRPT->AES_IV[2] = pu32[2];
    CRPT->AES_IV[3] = pu32[3];


    /* Set bytes count of A */
    CRPT->AES_GCM_ACNT[0] = size - plen_aligned;
    CRPT->AES_GCM_ACNT[1] = 0;
    CRPT->AES_GCM_PCNT[0] = plen;
    CRPT->AES_GCM_PCNT[1] = 0;



    CRPT->AES_SADDR = (uint32_t)ctx->ccm_buf;
    CRPT->AES_DADDR = (uint32_t)ctx->out_buf;
    CRPT->AES_CNT   = size;

    /* Start AES Eecrypt */
    CRPT->AES_CTL |= CRPT_AES_CTL_START_Msk | (CRYPTO_DMA_ONE_SHOT << CRPT_AES_CTL_DMALAST_Pos);

    /* Waiting for AES calculation */
    while ((CRPT->INTSTS & CRPT_INTSTS_AESIF_Msk) == 0)
    {
        if (timeout-- < 0)
            return -1;
    }

    /* Clear flag */
    CRPT->INTSTS = CRPT_INTSTS_AESIF_Msk;

    memcpy(buf, ctx->out_buf, plen);

    if (tlen > 16)
    {
        tlen = 16;
    }

    memcpy(tag, &ctx->out_buf[plen_aligned], tlen);

    return 0;
}




/*
 * Initialize context
 */
void mbedtls_ccm_init(mbedtls_ccm_context *ctx)
{
    CCM_VALIDATE(ctx != NULL);
    memset(ctx, 0, sizeof(mbedtls_ccm_context));
}

int mbedtls_ccm_setkey(mbedtls_ccm_context *ctx,
                       mbedtls_cipher_id_t cipher,
                       const unsigned char *key,
                       unsigned int keybits)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    const mbedtls_cipher_info_t *cipher_info;

    CCM_VALIDATE_RET(ctx != NULL);
    CCM_VALIDATE_RET(key != NULL);

    uint32_t au32Buf[8];
    int32_t i, klen;

    cipher_info = mbedtls_cipher_info_from_values(cipher, keybits, MBEDTLS_MODE_ECB);

    if (cipher_info == NULL)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    if (cipher_info->block_size != 16)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    mbedtls_cipher_free(&ctx->cipher_ctx);

    if ((ret = mbedtls_cipher_setup(&ctx->cipher_ctx, cipher_info)) != 0)
        return (ret);

    if ((ret = mbedtls_cipher_setkey(&ctx->cipher_ctx, key, keybits,
                                     MBEDTLS_ENCRYPT)) != 0)
    {
        return (ret);
    }

    klen = keybits / 8;
    ctx->keySize = klen;
    memcpy(au32Buf, key, klen);
    ToBigEndian((uint8_t *)au32Buf, klen);

    for (i = 0; i < klen / 4; i++)
    {
        CRPT->AES_KEY[i] = au32Buf[i];
    }

    return (0);
}

/*
 * Free context
 */
void mbedtls_ccm_free(mbedtls_ccm_context *ctx)
{
    if (ctx == NULL)
        return;

    mbedtls_cipher_free(&ctx->cipher_ctx);
    mbedtls_platform_zeroize(ctx, sizeof(mbedtls_ccm_context));
}

/*
 * Macros for common operations.
 * Results in smaller compiled code than static inline functions.
 */

/*
 * Update the CBC-MAC state in y using a block in b
 * (Always using b as the source helps the compiler optimise a bit better.)
 */
#define UPDATE_CBC_MAC                                                      \
    for( i = 0; i < 16; i++ )                                               \
        y[i] ^= b[i];                                                       \
    \
    if( ( ret = mbedtls_cipher_update( &ctx->cipher_ctx, y, 16, y, &olen ) ) != 0 ) \
        return( ret );

/*
 * Encrypt or decrypt a partial block with CTR
 * Warning: using b for temporary storage! src and dst must not be b!
 * This avoids allocating one more 16 bytes buffer while allowing src == dst.
 */
#define CTR_CRYPT( dst, src, len  )                                            \
    if( ( ret = mbedtls_cipher_update( &ctx->cipher_ctx, ctr, 16, b, &olen ) ) != 0 )  \
        return( ret );                                                         \
    \
    for( i = 0; i < len; i++ )                                                 \
        dst[i] = src[i] ^ b[i];

/*
 * Authenticated encryption or decryption
 */
static int ccm_auth_crypt(mbedtls_ccm_context *ctx, int mode, size_t length,
                          const unsigned char *iv, size_t iv_len,
                          const unsigned char *add, size_t add_len,
                          const unsigned char *input, unsigned char *output,
                          unsigned char *tag, size_t tag_len)
{
    int ret;

    if (iv_len < 7 || iv_len > 13)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);


    ret = _CCM(ctx, mode, iv, iv_len, add, add_len, input, length, output, tag, tag_len);
    return (ret);
}

/*
 * Authenticated encryption
 */
int mbedtls_ccm_star_encrypt_and_tag(mbedtls_ccm_context *ctx, size_t length,
                                     const unsigned char *iv, size_t iv_len,
                                     const unsigned char *add, size_t add_len,
                                     const unsigned char *input, unsigned char *output,
                                     unsigned char *tag, size_t tag_len)
{
    CCM_VALIDATE_RET(ctx != NULL);
    CCM_VALIDATE_RET(iv != NULL);
    CCM_VALIDATE_RET(add_len == 0 || add != NULL);
    CCM_VALIDATE_RET(length == 0 || input != NULL);
    CCM_VALIDATE_RET(length == 0 || output != NULL);
    CCM_VALIDATE_RET(tag_len == 0 || tag != NULL);
    return (ccm_auth_crypt(ctx, CCM_ENCRYPT, length, iv, iv_len,
                           add, add_len, input, output, tag, tag_len));
}

int mbedtls_ccm_encrypt_and_tag(mbedtls_ccm_context *ctx, size_t length,
                                const unsigned char *iv, size_t iv_len,
                                const unsigned char *add, size_t add_len,
                                const unsigned char *input, unsigned char *output,
                                unsigned char *tag, size_t tag_len)
{
    CCM_VALIDATE_RET(ctx != NULL);
    CCM_VALIDATE_RET(iv != NULL);
    CCM_VALIDATE_RET(add_len == 0 || add != NULL);
    CCM_VALIDATE_RET(length == 0 || input != NULL);
    CCM_VALIDATE_RET(length == 0 || output != NULL);
    CCM_VALIDATE_RET(tag_len == 0 || tag != NULL);

    if (tag_len == 0)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    return (mbedtls_ccm_star_encrypt_and_tag(ctx, length, iv, iv_len, add,
                                             add_len, input, output, tag, tag_len));
}

/*
 * Authenticated decryption
 */
int mbedtls_ccm_star_auth_decrypt(mbedtls_ccm_context *ctx, size_t length,
                                  const unsigned char *iv, size_t iv_len,
                                  const unsigned char *add, size_t add_len,
                                  const unsigned char *input, unsigned char *output,
                                  const unsigned char *tag, size_t tag_len)
{
    int ret;
    unsigned char check_tag[16];
    unsigned char i;
    int diff;

    CCM_VALIDATE_RET(ctx != NULL);
    CCM_VALIDATE_RET(iv != NULL);
    CCM_VALIDATE_RET(add_len == 0 || add != NULL);
    CCM_VALIDATE_RET(length == 0 || input != NULL);
    CCM_VALIDATE_RET(length == 0 || output != NULL);
    CCM_VALIDATE_RET(tag_len == 0 || tag != NULL);

    if ((ret = ccm_auth_crypt(ctx, CCM_DECRYPT, length,
                              iv, iv_len, add, add_len,
                              input, output, check_tag, tag_len)) != 0)
    {
        return (ret);
    }

    /* Check tag in "constant-time" */
    for (diff = 0, i = 0; i < tag_len; i++)
        diff |= tag[i] ^ check_tag[i];

    if (diff != 0)
    {
        mbedtls_platform_zeroize(output, length);
        return (MBEDTLS_ERR_CCM_AUTH_FAILED);
    }

    return (0);
}

int mbedtls_ccm_auth_decrypt(mbedtls_ccm_context *ctx, size_t length,
                             const unsigned char *iv, size_t iv_len,
                             const unsigned char *add, size_t add_len,
                             const unsigned char *input, unsigned char *output,
                             const unsigned char *tag, size_t tag_len)
{
    CCM_VALIDATE_RET(ctx != NULL);
    CCM_VALIDATE_RET(iv != NULL);
    CCM_VALIDATE_RET(add_len == 0 || add != NULL);
    CCM_VALIDATE_RET(length == 0 || input != NULL);
    CCM_VALIDATE_RET(length == 0 || output != NULL);
    CCM_VALIDATE_RET(tag_len == 0 || tag != NULL);

    if (tag_len == 0)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    return (mbedtls_ccm_star_auth_decrypt(ctx, length, iv, iv_len, add,
                                          add_len, input, output, tag, tag_len));
}
#endif /* MBEDTLS_CCM_ALT */
#endif /* MBEDTLS_CCM_C */
