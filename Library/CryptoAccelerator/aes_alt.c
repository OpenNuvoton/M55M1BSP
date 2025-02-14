/*
 * Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
 * Copyright (C) 2022, Nuvoton Technology Corporation, All Rights Reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 *  The AES block cipher was designed by Vincent Rijmen and Joan Daemen.
 *
 *  http://csrc.nist.gov/encryption/aes/rijndael/Rijndael.pdf
 *  http://csrc.nist.gov/publications/fips/fips197/fips-197.pdf
 */

#include "common.h"

#include "mbedtls/platform_util.h"
#include "mbedtls/aes.h"
#include "mbedtls/error.h"

#if defined(MBEDTLS_AES_C)
#if defined(MBEDTLS_AES_ALT)

#include <string.h>
#include "NuMicro.h"


/* Parameter validation macros based on platform_util.h */
#define AES_VALIDATE_RET( cond )    \
    MBEDTLS_INTERNAL_VALIDATE_RET( cond, MBEDTLS_ERR_AES_BAD_INPUT_DATA )
#define AES_VALIDATE( cond )        \
    MBEDTLS_INTERNAL_VALIDATE( cond )


#ifndef ARG_UNUSED
    #define ARG_UNUSED(arg)  ((void)arg)
#endif

#define AES_BLOCK_SIZE  (16)


#if (NVT_DCACHE_ON == 1)
    // DCache-line aligned buffer for improved performance when DCache is enabled
    uint8_t s_u8in[DCACHE_ALIGN_LINE_SIZE(AES_BLOCK_SIZE)] __attribute__((aligned(DCACHE_LINE_SIZE)));
    uint8_t s_u8out[DCACHE_ALIGN_LINE_SIZE(AES_BLOCK_SIZE)] __attribute__((aligned(DCACHE_LINE_SIZE)));
#else
    // Standard buffer alignment when DCache is disabled
    __attribute__((aligned(4))) static uint8_t s_u8in[AES_BLOCK_SIZE];
    __attribute__((aligned(4))) static uint8_t s_u8out[AES_BLOCK_SIZE];
#endif



#if defined(MBEDTLS_CIPHER_MODE_CBC)
__STATIC_INLINE uint32_t nu_get32_le(const uint8_t *pos)
{
    uint32_t val;

    val = *pos++;
    val += (*pos++ << 8);
    val += (*pos++ << 16);
    val += (*pos++ << 24);

    return val;
}

__STATIC_INLINE void nu_set32_le(uint8_t *pos, uint32_t val)
{
    *pos++ = val & 0xFF;
    *pos++ = (val >> 8) & 0xFF;
    *pos++ = (val >> 16) & 0xFF;
    *pos = (val >> 24) & 0xFF;
}
#endif


/* Implementation that should never be optimized out by the compiler */
static void mbedtls_zeroize(void *v, size_t n)
{
    volatile unsigned char *p = (unsigned char *)v;

    while (n--) *p++ = 0;
}

/* AES DMA compatible backup buffer if user buffer doesn't meet requirements
 *
 * AES DMA buffer location requires to be:
 * (1) Word-aligned
 * (2) Located in 0x2xxxxxxx region. Check linker files to ensure global variables are placed in this region.
 *
 * AES DMA buffer size MAX_DMA_CHAIN_SIZE must be a multiple of 16-byte block size.
 * Its value is estimated to trade memory footprint off against performance.
 *
 */
#define MAX_DMA_CHAIN_SIZE (AES_BLOCK_SIZE*6)

void mbedtls_aes_init(mbedtls_aes_context *ctx)
{
    AES_VALIDATE(ctx != NULL);

    memset(ctx, 0, sizeof(mbedtls_aes_context));
}

void mbedtls_aes_free(mbedtls_aes_context *ctx)
{
    if (ctx == NULL)
        return;

    mbedtls_zeroize(ctx, sizeof(mbedtls_aes_context));
}

/*
 * AES key schedule (encryption)
 */
int mbedtls_aes_setkey_enc(mbedtls_aes_context *ctx, const unsigned char *key,
                           unsigned int keybits)
{
    unsigned int i;
    uint8_t *pu8;

    AES_VALIDATE_RET(ctx != NULL);
    AES_VALIDATE_RET(key != NULL);

    switch (keybits)
    {
        case 128:
            ctx->keySize = 16;
            ctx->keySizeOp = 0;
            break;

        case 192:
            ctx->keySize = 24;
            ctx->keySizeOp = 1 << CRYPTO_AES_CTL_KEYSZ_Pos;
            break;

        case 256:
            ctx->keySize = 32;
            ctx->keySizeOp = 2 << CRYPTO_AES_CTL_KEYSZ_Pos;
            break;

        default:
            return (MBEDTLS_ERR_AES_INVALID_KEY_LENGTH);
    }

    pu8 = (uint8_t *)&ctx->keys[0];

    for (i = 0; i < ctx->keySize; i++)
    {
        pu8[i] = key[i];
    }

    return (0);
}

/*
 * AES key schedule (decryption)
 */
int mbedtls_aes_setkey_dec(mbedtls_aes_context *ctx, const unsigned char *key,
                           unsigned int keybits)
{
    int ret;

    AES_VALIDATE_RET(ctx != NULL);
    AES_VALIDATE_RET(key != NULL);

    /* Also checks keybits */
    if ((ret = mbedtls_aes_setkey_enc(ctx, key, keybits)) != 0)
        goto exit;

exit:
    return (ret);
}

/* Do AES encrypt/decrypt with H/W accelerator
 *
 * NOTE: As input/output buffer doesn't follow constraint of DMA buffer, static allocated
 *       DMA compatible buffer is used for DMA instead and this needs extra copy.
 *
 * NOTE: dataSize requires to be:
 *       1) Multiple of block size 16
 *       2) <= MAX_DMA_CHAIN_SIZE
 */
static int __nvt_aes_crypt(mbedtls_aes_context *ctx,
                           const unsigned char *input,
                           unsigned char *output, size_t dataSize)
{
    int32_t i, wcnt, timeout = 0x10000000;

    if (dataSize > AES_BLOCK_SIZE)
        return -1;

    /* Force AES free */
    CRYPTO->AES_CTL = CRYPTO_AES_CTL_STOP_Msk;

    memcpy(s_u8in, input, dataSize);
#if (NVT_DCACHE_ON == 1)
    SCB_CleanDCache_by_Addr(s_u8in, sizeof(s_u8in));
#endif

    /* Enable AES interrupt */
    AES_ENABLE_INT(CRYPTO);

    /* We support multiple contexts with context save & restore and so needs just one
     * H/W channel. Always use H/W channel #0. */

    /* AES_IN_OUT_SWAP: Let H/W know both input/output data are arranged in little-endian */
    //AES_Open(CRYPTO, 0, ctx->encDec, ctx->opMode, ctx->keySize, AES_IN_OUT_SWAP);
    CRYPTO->AES_CTL = ctx->encDec | ctx->opMode | ctx->keySizeOp |
                      (AES_IN_OUT_SWAP << CRYPTO_AES_CTL_OUTSWAP_Pos) | CRYPTO_AES_CTL_KINSWAP_Msk | CRYPTO_AES_CTL_DMAEN_Msk;

    //AES_SetInitVect(CRYPTO, 0, ctx->iv);
    CRYPTO->AES_IV[0] = ctx->iv[0];
    CRYPTO->AES_IV[1] = ctx->iv[1];
    CRYPTO->AES_IV[2] = ctx->iv[2];
    CRYPTO->AES_IV[3] = ctx->iv[3];


    //AES_SetKey(CRYPTO, 0, ctx->keys, ctx->keySize);

    wcnt = ctx->keySize / 4;

    for (i = 0; i < wcnt; i++)
    {
        CRYPTO->AES_KEY[i] = ctx->keys[i];
    }


    //AES_SetDMATransfer(CRYPTO, 0, (uint32_t)pIn, (uint32_t)pOut, dataSize);
    CRYPTO->AES_SADDR = (uint32_t)s_u8in;
    CRYPTO->AES_DADDR = (uint32_t)s_u8out;
    CRYPTO->AES_CNT = dataSize;

    /* Clear flag */
    CRYPTO->INTSTS = CRYPTO_INTSTS_AESIF_Msk;

    //AES_Start(CRYPTO, 0, CRYPTO_DMA_ONE_SHOT);
    CRYPTO->AES_CTL |= CRYPTO_AES_CTL_START_Msk;

    while ((CRYPTO->INTSTS & CRYPTO_INTSTS_AESIF_Msk) == 0)
    {
        /* Check timeout */
        if (timeout-- <= 0)
            return -1;
    }

#if (NVT_DCACHE_ON == 1)
    SCB_InvalidateDCache_by_Addr(s_u8out, sizeof(s_u8out));
#endif
    memcpy(output, s_u8out, dataSize);

    return 0;

}

/*
 * AES-ECB block encryption
 */
void mbedtls_aes_encrypt(mbedtls_aes_context *ctx,
                         const unsigned char input[AES_BLOCK_SIZE],
                         unsigned char output[AES_BLOCK_SIZE])
{
    ctx->encDec = CRYPTO_AES_CTL_ENCRYPTO_Msk;
    __nvt_aes_crypt(ctx, input, output, AES_BLOCK_SIZE);
}

/*
 * AES-ECB block decryption
 */
void mbedtls_aes_decrypt(mbedtls_aes_context *ctx,
                         const unsigned char input[AES_BLOCK_SIZE],
                         unsigned char output[AES_BLOCK_SIZE])
{
    ctx->encDec = 0;
    __nvt_aes_crypt(ctx, input, output, AES_BLOCK_SIZE);
}

/*
 * AES-ECB block encryption/decryption
 */
int mbedtls_aes_crypt_ecb(mbedtls_aes_context *ctx,
                          int mode,
                          const unsigned char input[AES_BLOCK_SIZE],
                          unsigned char output[AES_BLOCK_SIZE])
{

    AES_VALIDATE_RET(ctx != NULL);
    AES_VALIDATE_RET(input != NULL);
    AES_VALIDATE_RET(output != NULL);
    AES_VALIDATE_RET(mode == MBEDTLS_AES_ENCRYPT ||
                     mode == MBEDTLS_AES_DECRYPT);

    ctx->opMode = AES_MODE_ECB << CRYPTO_AES_CTL_OPMODE_Pos;

    if (mode == MBEDTLS_AES_ENCRYPT)
        mbedtls_aes_encrypt(ctx, input, output);
    else
        mbedtls_aes_decrypt(ctx, input, output);


    return (0);
}

#if defined(MBEDTLS_CIPHER_MODE_CBC)
/*
 * AES-CBC buffer encryption/decryption
 */
int mbedtls_aes_crypt_cbc(mbedtls_aes_context *ctx,
                          int mode,
                          size_t len,
                          unsigned char iv[AES_BLOCK_SIZE],
                          const unsigned char *input,
                          unsigned char *output)
{
    int length = len;
    int blockChainLen;
    uint8_t au8Tmp[AES_BLOCK_SIZE];

    AES_VALIDATE_RET(ctx != NULL);
    AES_VALIDATE_RET(mode == MBEDTLS_AES_ENCRYPT ||
                     mode == MBEDTLS_AES_DECRYPT);
    AES_VALIDATE_RET(iv != NULL);
    AES_VALIDATE_RET(input != NULL);
    AES_VALIDATE_RET(output != NULL);

    if (length & (AES_BLOCK_SIZE - 1))
        return (MBEDTLS_ERR_AES_INVALID_INPUT_LENGTH);

    ctx->opMode = AES_MODE_CBC << CRYPTO_AES_CTL_OPMODE_Pos;
    /* Fetch IV byte data in big-endian */
    ctx->iv[0] = nu_get32_le(iv);
    ctx->iv[1] = nu_get32_le(iv + 4);
    ctx->iv[2] = nu_get32_le(iv + 8);
    ctx->iv[3] = nu_get32_le(iv + 12);

    if (mode == MBEDTLS_AES_ENCRYPT)
    {
        ctx->encDec = CRYPTO_AES_CTL_ENCRYPTO_Msk;
    }
    else
    {
        ctx->encDec = 0;
    }

    memcpy(au8Tmp, input, AES_BLOCK_SIZE);

    while (length > 0)
    {
        blockChainLen = (length > MAX_DMA_CHAIN_SIZE) ? MAX_DMA_CHAIN_SIZE : length;

        __nvt_aes_crypt(ctx, input, output, blockChainLen);

        if (ctx->encDec)
        {
            ctx->iv[0] = nu_get32_le(output);
            ctx->iv[1] = nu_get32_le(output + 4);
            ctx->iv[2] = nu_get32_le(output + 8);
            ctx->iv[3] = nu_get32_le(output + 12);
        }
        else
        {
            ctx->iv[0] = nu_get32_le(au8Tmp);
            ctx->iv[1] = nu_get32_le(au8Tmp + 4);
            ctx->iv[2] = nu_get32_le(au8Tmp + 8);
            ctx->iv[3] = nu_get32_le(au8Tmp + 12);
        }

        length -= blockChainLen;
        input  += blockChainLen;
        output += blockChainLen;

    }

    /* Save IV for next block cipher */
    nu_set32_le(iv, ctx->iv[0]);
    nu_set32_le(iv + 4, ctx->iv[1]);
    nu_set32_le(iv + 8, ctx->iv[2]);
    nu_set32_le(iv + 12, ctx->iv[3]);

    return (0);
}
#endif /* MBEDTLS_CIPHER_MODE_CBC */

#if defined(MBEDTLS_CIPHER_MODE_CFB)
int mbedtls_aes_crypt_cfb128(mbedtls_aes_context *ctx,
                             int mode,
                             size_t length,
                             size_t *iv_off,
                             unsigned char iv[AES_BLOCK_SIZE],
                             const unsigned char *input,
                             unsigned char *output)
{

    int32_t i, wcnt, timeout = 0x10000000;
    uint32_t len;
    uint32_t u32Ctl;
    int32_t first = 1;
    uint32_t u32Tmp;

    AES_VALIDATE_RET(ctx != NULL);
    AES_VALIDATE_RET(mode == MBEDTLS_AES_ENCRYPT ||
                     mode == MBEDTLS_AES_DECRYPT);
    AES_VALIDATE_RET(iv_off != NULL);
    AES_VALIDATE_RET(iv != NULL);
    AES_VALIDATE_RET(input != NULL);
    AES_VALIDATE_RET(output != NULL);

    AES_VALIDATE_RET(*iv_off == 0);

    /* Force AES free */
    CRYPTO->AES_CTL = CRYPTO_AES_CTL_STOP_Msk;

    wcnt = ctx->keySize / 4;

    for (i = 0; i < wcnt; i++)
    {
        CRYPTO->AES_KEY[i] = ctx->keys[i];
    }

    ctx->encDec = mode << CRYPTO_AES_CTL_ENCRYPTO_Pos;
    ctx->opMode = AES_MODE_CFB << CRYPTO_AES_CTL_OPMODE_Pos;

    /* Enable AES interrupt */
    AES_ENABLE_INT(CRYPTO);

    u32Ctl = ctx->encDec | ctx->opMode | ctx->keySizeOp |
             (AES_IN_OUT_SWAP << CRYPTO_AES_CTL_OUTSWAP_Pos) | CRYPTO_AES_CTL_KINSWAP_Msk | CRYPTO_AES_CTL_DMAEN_Msk;

    for (i = 0; i < 4; i++)
    {
        u32Tmp = iv[i * 4 + 3];
        u32Tmp = (u32Tmp << 8) + iv[i * 4 + 2];
        u32Tmp = (u32Tmp << 8) + iv[i * 4 + 1];
        u32Tmp = (u32Tmp << 8) + iv[i * 4 + 0];
        CRYPTO->AES_IV[i] = u32Tmp;
    }

    CRYPTO->AES_SADDR = (uint32_t)s_u8in;
    CRYPTO->AES_DADDR = (uint32_t)s_u8out;
    CRYPTO->AES_CNT = AES_BLOCK_SIZE;

    len = AES_BLOCK_SIZE;

    if (length <= AES_BLOCK_SIZE)
    {
        memcpy(s_u8in, input, length);
#if (NVT_DCACHE_ON == 1)
        SCB_CleanDCache_by_Addr(s_u8in, sizeof(s_u8in));
#endif

        CRYPTO->AES_CNT = length;

        /* Clear flag */
        CRYPTO->INTSTS = CRYPTO_INTSTS_AESIF_Msk;
        CRYPTO->AES_CTL = u32Ctl | CRYPTO_AES_CTL_DMAEN_Msk | CRYPTO_AES_CTL_START_Msk;

        while ((CRYPTO->INTSTS & CRYPTO_INTSTS_AESIF_Msk) == 0)
        {
            /* Check timeout */
            if (timeout-- <= 0)
                return -1;
        }

#if (NVT_DCACHE_ON == 1)
        SCB_InvalidateDCache_by_Addr(s_u8out, sizeof(s_u8out));
#endif
        memcpy(output, s_u8out, length);

    }
    else
    {
        while (length)
        {
            /* Clear flag */
            CRYPTO->INTSTS = CRYPTO_INTSTS_AESIF_Msk;

            if (length > AES_BLOCK_SIZE)
            {
                memcpy(s_u8in, input, len);
#if (NVT_DCACHE_ON == 1)
                SCB_CleanDCache_by_Addr(s_u8in, sizeof(s_u8in));
#endif

                if (first)
                {
                    /* First block */
                    CRYPTO->AES_CTL = u32Ctl | CRYPTO_AES_CTL_START_Msk;
                    first = 0;
                }
                else
                {
                    /* subsequential blocks */
                    CRYPTO->AES_CTL = u32Ctl | CRYPTO_AES_CTL_DMACSCAD_Msk | CRYPTO_AES_CTL_START_Msk;
                }
            }
            else
            {
                /* Last block */
                len = length;
                memcpy(s_u8in, input, len);
#if (NVT_DCACHE_ON == 1)
                SCB_CleanDCache_by_Addr(s_u8in, sizeof(s_u8in));
#endif
                CRYPTO->AES_CNT = len;
                CRYPTO->AES_CTL = u32Ctl | CRYPTO_AES_CTL_DMACSCAD_Msk | CRYPTO_AES_CTL_DMALAST_Msk |
                                  CRYPTO_AES_CTL_START_Msk;
            }

            while ((CRYPTO->INTSTS & CRYPTO_INTSTS_AESIF_Msk) == 0)
            {
                /* Check timeout */
                if (timeout-- <= 0)
                    return -1;
            }

#if (NVT_DCACHE_ON == 1)
            SCB_InvalidateDCache_by_Addr(s_u8out, sizeof(s_u8out));
#endif
            memcpy(output, s_u8out, len);
            output += len;
            input += len;
            length -= len;
        }
    }

    return 0;
}


/*
 * AES-CFB8 buffer encryption/decryption
 */
int mbedtls_aes_crypt_cfb8(mbedtls_aes_context *ctx,
                           int mode,
                           size_t length,
                           unsigned char iv[AES_BLOCK_SIZE],
                           const unsigned char *input,
                           unsigned char *output)
{
    unsigned char c;
    unsigned char ov[AES_BLOCK_SIZE + 1];

    AES_VALIDATE_RET(ctx != NULL);
    AES_VALIDATE_RET(mode == MBEDTLS_AES_ENCRYPT ||
                     mode == MBEDTLS_AES_DECRYPT);
    AES_VALIDATE_RET(iv != NULL);
    AES_VALIDATE_RET(input != NULL);
    AES_VALIDATE_RET(output != NULL);

    while (length--)
    {
        memcpy(ov, iv, AES_BLOCK_SIZE);
        mbedtls_aes_crypt_ecb(ctx, MBEDTLS_AES_ENCRYPT, iv, iv);

        if (mode == MBEDTLS_AES_DECRYPT)
            ov[AES_BLOCK_SIZE] = *input;

        c = *output++ = (unsigned char)(iv[0] ^ *input++);

        if (mode == MBEDTLS_AES_ENCRYPT)
            ov[AES_BLOCK_SIZE] = c;

        memcpy(iv, ov + 1, AES_BLOCK_SIZE);
    }

    return (0);
}
#endif /*MBEDTLS_CIPHER_MODE_CFB */

#if defined(MBEDTLS_CIPHER_MODE_CTR)
/*
 * AES-CTR buffer encryption/decryption
 */
int mbedtls_aes_crypt_ctr(mbedtls_aes_context *ctx,
                          size_t length,
                          size_t *nc_off,
                          unsigned char nonce_counter[AES_BLOCK_SIZE],
                          unsigned char stream_block[AES_BLOCK_SIZE],
                          const unsigned char *input,
                          unsigned char *output)
{
    int c, i;
    size_t n = *nc_off;

    AES_VALIDATE_RET(ctx != NULL);
    AES_VALIDATE_RET(nc_off != NULL);
    AES_VALIDATE_RET(nonce_counter != NULL);
    AES_VALIDATE_RET(stream_block != NULL);
    AES_VALIDATE_RET(input != NULL);
    AES_VALIDATE_RET(output != NULL);

    while (length--)
    {
        if (n == 0)
        {
            mbedtls_aes_crypt_ecb(ctx, MBEDTLS_AES_ENCRYPT, nonce_counter, stream_block);

            for (i = AES_BLOCK_SIZE; i > 0; i--)
                if (++nonce_counter[i - 1] != 0)
                    break;
        }

        c = *input++;
        *output++ = (unsigned char)(c ^ stream_block[n]);

        n = (n + 1) & 0x0F;
    }

    *nc_off = n;

    return (0);
}
#endif /* MBEDTLS_CIPHER_MODE_CTR */

#if defined(MBEDTLS_CIPHER_MODE_OFB)
/*
 * AES-CTR buffer encryption/decryption
 */
int mbedtls_aes_crypt_ofb(mbedtls_aes_context *ctx,
                          size_t length,
                          size_t *iv_off,
                          unsigned char iv[AES_BLOCK_SIZE],
                          const unsigned char *input,
                          unsigned char *output)
{
    int32_t i, wcnt, timeout = 0x10000000;
    uint32_t len;
    uint32_t u32Ctl;
    int32_t first = 1;
    uint32_t u32Tmp;

    if (length == 0)
    {
        /* Just do nothing */
        return (0);
    }

    if ((ctx == NULL) || (iv == NULL) || (input == NULL) || (output == NULL))
    {
        return MBEDTLS_ERR_AES_INVALID_INPUT_LENGTH;
    }

    if ((iv_off != NULL) && (*iv_off != 0))
    {
        /* only support iv_off == 0 */
        return MBEDTLS_ERR_AES_INVALID_INPUT_LENGTH;
    }


    if (*iv_off != 0)
    {
        printf("iv_off is not zero!\n");

    }

    wcnt = ctx->keySize / 4;

    for (i = 0; i < wcnt; i++)
    {
        CRYPTO->AES_KEY[i] = ctx->keys[i];
    }

    ctx->opMode = AES_MODE_OFB << CRYPTO_AES_CTL_OPMODE_Pos;

    /* Enable AES interrupt */
    AES_ENABLE_INT(CRYPTO);

    u32Ctl = ctx->opMode | ctx->keySizeOp | CRYPTO_AES_CTL_ENCRYPTO_Msk |
             (AES_IN_OUT_SWAP << CRYPTO_AES_CTL_OUTSWAP_Pos) | CRYPTO_AES_CTL_KINSWAP_Msk | CRYPTO_AES_CTL_DMAEN_Msk;

    for (i = 0; i < 4; i++)
    {
        u32Tmp = iv[i * 4 + 3];
        u32Tmp = (u32Tmp << 8) + iv[i * 4 + 2];
        u32Tmp = (u32Tmp << 8) + iv[i * 4 + 1];
        u32Tmp = (u32Tmp << 8) + iv[i * 4 + 0];
        CRYPTO->AES_IV[i] = u32Tmp;
    }

    CRYPTO->AES_SADDR = (uint32_t)s_u8in;
    CRYPTO->AES_DADDR = (uint32_t)s_u8out;
    CRYPTO->AES_CNT = AES_BLOCK_SIZE;

    len = AES_BLOCK_SIZE;

    if (length <= AES_BLOCK_SIZE)
    {
        memcpy(s_u8in, input, length);
#if (NVT_DCACHE_ON == 1)
        SCB_CleanDCache_by_Addr(s_u8in, sizeof(s_u8in));
        CRYPTO->AES_SADDR = (uint32_t)s_u8in;
        CRYPTO->AES_DADDR = (uint32_t)s_u8out;
#endif
        CRYPTO->AES_CNT = length;

        /* Clear flag */
        CRYPTO->INTSTS = CRYPTO_INTSTS_AESIF_Msk;
        CRYPTO->AES_CTL = u32Ctl | CRYPTO_AES_CTL_DMAEN_Msk | CRYPTO_AES_CTL_START_Msk;

        while ((CRYPTO->INTSTS & CRYPTO_INTSTS_AESIF_Msk) == 0)
        {
            /* Check timeout */
            if (timeout-- <= 0)
                return -1;
        }

#if (NVT_DCACHE_ON == 1)
        SCB_InvalidateDCache_by_Addr(s_u8out, sizeof(s_u8out));
#endif
        memcpy(output, s_u8out, length);

    }
    else
    {
        while (length)
        {
            /* Clear flag */
            CRYPTO->INTSTS = CRYPTO_INTSTS_AESIF_Msk;

            if (length > AES_BLOCK_SIZE)
            {
                memcpy(s_u8in, input, len);
#if (NVT_DCACHE_ON == 1)
                SCB_CleanDCache_by_Addr(s_u8in, sizeof(s_u8in));
                CRYPTO->AES_SADDR = (uint32_t)s_u8in;
                CRYPTO->AES_DADDR = (uint32_t)s_u8out;
#endif

                if (first)
                {
                    /* First block */
                    CRYPTO->AES_CTL = u32Ctl | CRYPTO_AES_CTL_START_Msk;
                    first = 0;
                }
                else
                {
                    /* subsequential blocks */
                    CRYPTO->AES_CTL = u32Ctl | CRYPTO_AES_CTL_DMACSCAD_Msk | CRYPTO_AES_CTL_START_Msk;
                }
            }
            else
            {
                /* Last block */
                len = length;
                memcpy(s_u8in, input, len);
#if (NVT_DCACHE_ON == 1)
                SCB_CleanDCache_by_Addr(s_u8in, sizeof(s_u8in));
                CRYPTO->AES_SADDR = (uint32_t)s_u8in;
                CRYPTO->AES_DADDR = (uint32_t)s_u8out;
#endif
                CRYPTO->AES_CNT = len;
                CRYPTO->AES_CTL = u32Ctl | CRYPTO_AES_CTL_DMACSCAD_Msk | CRYPTO_AES_CTL_DMALAST_Msk |
                                  CRYPTO_AES_CTL_START_Msk;
            }

            while ((CRYPTO->INTSTS & CRYPTO_INTSTS_AESIF_Msk) == 0)
            {
                /* Check timeout */
                if (timeout-- <= 0)
                    return -1;
            }

#if (NVT_DCACHE_ON == 1)
            SCB_InvalidateDCache_by_Addr(s_u8out, sizeof(s_u8out));
#endif
            memcpy(output, s_u8out, len);
            output += len;
            input += len;
            length -= len;
        }
    }

    return 0;
}
#endif /* MBEDTLS_CIPHER_MODE_OFB */




#endif /* MBEDTLS_AES_ALT */


#endif /* MBEDTLS_AES_C */
