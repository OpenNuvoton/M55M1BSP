/*
 *  NIST SP800-38C compliant CCM implementation
 *
 *  Copyright The Mbed TLS Contributors
 *  Copyright (c) 2023, Nuvoton Technology Corporation
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

#if defined(MBEDTLS_PLATFORM_C)
    #include "mbedtls/platform.h"
#else
    #if defined(MBEDTLS_SELF_TEST) && defined(MBEDTLS_AES_C)
        #include <stdio.h>
        #define mbedtls_printf printf
    #endif /* MBEDTLS_SELF_TEST && MBEDTLS_AES_C */
#endif /* MBEDTLS_PLATFORM_C */

#if (NVT_DCACHE_ON == 1)
    // DCache-line aligned buffer for improved performance when DCache is enabled
    uint8_t ccm_buf_array[DCACHE_ALIGN_LINE_SIZE(MAX_CCM_BUF)] __attribute__((aligned(DCACHE_LINE_SIZE)));
    uint8_t out_buf_array[DCACHE_ALIGN_LINE_SIZE(MAX_CCM_BUF + 16)] __attribute__((aligned(DCACHE_LINE_SIZE)));
#else
    // Standard buffer alignment when DCache is disabled
    __attribute__((aligned(4))) static uint8_t ccm_buf_array[MAX_CCM_BUF];
    __attribute__((aligned(4))) static uint8_t out_buf_array[MAX_CCM_BUF + 16];
#endif


/*
 * Initialize context
 */
void mbedtls_ccm_init(mbedtls_ccm_context *ctx)
{
    memset(ctx, 0, sizeof(mbedtls_ccm_context));

    //For DCache
    ctx->ccm_buf = &ccm_buf_array[0];
    ctx->out_buf = &out_buf_array[0];
}

int mbedtls_ccm_setkey(mbedtls_ccm_context *ctx,
                       mbedtls_cipher_id_t cipher,
                       const unsigned char *key,
                       unsigned int keybits)
{
    int i;

    /* Store the key to ctx */
    for (i = 0; i < keybits / 8; i++)
        ctx->key[i] = key[i];

    ctx->keybits = keybits;

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

#define CCM_STATE__CLEAR                0
#define CCM_STATE__STARTED              (1 << 0)
#define CCM_STATE__LENGHTS_SET          (1 << 1)
#define CCM_STATE__AUTH_DATA_STARTED    (1 << 2)
#define CCM_STATE__AUTH_DATA_FINISHED   (1 << 3)
#define CCM_STATE__ERROR                (1 << 4)

static void mbedtls_ccm_clear_state(mbedtls_ccm_context *ctx)
{
    ctx->state = CCM_STATE__CLEAR;
    memset(ctx->y, 0, 16);
    memset(ctx->ctr, 0, 16);
}

static int ccm_calculate_first_block_if_ready(mbedtls_ccm_context *ctx)
{
    unsigned char i;
    size_t len_left;

    /* length calulcation can be done only after both
     * mbedtls_ccm_starts() and mbedtls_ccm_set_lengths() have been executed
     */
    if (!(ctx->state & CCM_STATE__STARTED) || !(ctx->state & CCM_STATE__LENGHTS_SET))
        return 0;

    /* CCM expects non-empty tag.
     * CCM* allows empty tag. For CCM* without tag, ignore plaintext length.
     */
    if (ctx->tag_len == 0)
    {
        if (ctx->mode == MBEDTLS_CCM_STAR_ENCRYPT || ctx->mode == MBEDTLS_CCM_STAR_DECRYPT)
        {
            ctx->plaintext_len = 0;
        }
        else
        {
            return (MBEDTLS_ERR_CCM_BAD_INPUT);
        }
    }

    /*
     * First block:
     * 0        .. 0        flags
     * 1        .. iv_len   nonce (aka iv)  - set by: mbedtls_ccm_starts()
     * iv_len+1 .. 15       length
     *
     * With flags as (bits):
     * 7        0
     * 6        add present?
     * 5 .. 3   (t - 2) / 2
     * 2 .. 0   q - 1
     */
    ctx->y[0] |= (ctx->add_len > 0) << 6;
    ctx->y[0] |= ((ctx->tag_len - 2) / 2) << 3;
    ctx->y[0] |= ctx->q - 1;

    for (i = 0, len_left = ctx->plaintext_len; i < ctx->q; i++, len_left >>= 8)
        ctx->y[15 - i] = MBEDTLS_BYTE_0(len_left);

    if (len_left > 0)
    {
        ctx->state |= CCM_STATE__ERROR;
        return (MBEDTLS_ERR_CCM_BAD_INPUT);
    }

    return (0);
}

int mbedtls_ccm_starts(mbedtls_ccm_context *ctx,
                       int mode,
                       const unsigned char *iv,
                       size_t iv_len)
{
    /* Also implies q is within bounds */
    if (iv_len < 7 || iv_len > 13)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    ctx->mode = mode;
    ctx->q = 16 - 1 - (unsigned char) iv_len;

    /*
     * Prepare counter block for encryption:
     * 0        .. 0        flags
     * 1        .. iv_len   nonce (aka iv)
     * iv_len+1 .. 15       counter (initially 1)
     *
     * With flags as (bits):
     * 7 .. 3   0
     * 2 .. 0   q - 1
     */
    memset(ctx->ctr, 0, 16);
    ctx->ctr[0] = ctx->q - 1;
    memcpy(ctx->ctr + 1, iv, iv_len);
    memset(ctx->ctr + 1 + iv_len, 0, ctx->q);
    ctx->ctr[15] = 1;

    /*
     * See ccm_calculate_first_block_if_ready() for block layout description
     */
    memcpy(ctx->y + 1, iv, iv_len);

    ctx->state |= CCM_STATE__STARTED;
    return ccm_calculate_first_block_if_ready(ctx);
}

int mbedtls_ccm_set_lengths(mbedtls_ccm_context *ctx,
                            size_t total_ad_len,
                            size_t plaintext_len,
                            size_t tag_len)
{
    /*
     * Check length requirements: SP800-38C A.1
     * Additional requirement: a < 2^16 - 2^8 to simplify the code.
     * 'length' checked later (when writing it to the first block)
     *
     * Also, loosen the requirements to enable support for CCM* (IEEE 802.15.4).
     */
    if (tag_len == 2 || tag_len > 16 || tag_len % 2 != 0)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    if (total_ad_len >= 0xFF00)
        return (MBEDTLS_ERR_CCM_BAD_INPUT);

    ctx->plaintext_len = plaintext_len;
    ctx->add_len = total_ad_len;
    ctx->tag_len = tag_len;
    ctx->processed = 0;

    ctx->state |= CCM_STATE__LENGHTS_SET;
    return ccm_calculate_first_block_if_ready(ctx);
}

int mbedtls_ccm_update_ad(mbedtls_ccm_context *ctx,
                          const unsigned char *add,
                          size_t add_len)
{

    return MBEDTLS_ERR_PLATFORM_HW_ACCEL_FAILED;

}

int mbedtls_ccm_update(mbedtls_ccm_context *ctx,
                       const unsigned char *input, size_t input_len,
                       unsigned char *output, size_t output_size,
                       size_t *output_len)
{
    return MBEDTLS_ERR_PLATFORM_HW_ACCEL_FAILED;
}

int mbedtls_ccm_finish(mbedtls_ccm_context *ctx,
                       unsigned char *tag, size_t tag_len)
{
    return (0);
}

/*
 * Authenticated encryption or decryption
 */
static int ccm_auth_crypt(mbedtls_ccm_context *ctx, int mode, size_t length,
                          const unsigned char *iv, size_t iv_len,
                          const unsigned char *add, size_t add_len,
                          const unsigned char *input, unsigned char *output,
                          unsigned char *tag, size_t tag_len)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;

    size_t add_len_aligned, length_aligned;
    int32_t i, j;
    uint32_t keySizeOp;
    int32_t timeout;

    if ((ret = mbedtls_ccm_starts(ctx, mode, iv, iv_len)) != 0)
        return (ret);

    /* The default value is 1 in mbedtls_ccm_starts but we need 0 */
    ctx->ctr[15] = 0;

    if ((ret = mbedtls_ccm_set_lengths(ctx, add_len, length, tag_len)) != 0)
        return (ret);

    /* Check size of input block */
    add_len_aligned = ((add_len + 2) & 0xf) ? (((add_len + 2) & (~0xf)) + 16) : (add_len + 2);
    length_aligned = (length & 0xf) ? ((length & (~0xf)) + 16) : (length);

    if (16 + add_len_aligned + length_aligned > MAX_CCM_BUF)
        return MBEDTLS_ERR_CCM_BAD_INPUT;

    /* Prepare input block for hardware CCM */
    memset(ctx->ccm_buf, 0, MAX_CCM_BUF);
    memcpy(ctx->ccm_buf, ctx->y, 16);
    /* add len in big endian. and support only support 2^16 - 2^8 length */
    ctx->ccm_buf[16] = add_len >> 8;
    ctx->ccm_buf[17] = add_len & 0xff;
    memcpy(ctx->ccm_buf + 16 + 2, add, add_len);
    memset(ctx->ccm_buf + 16 + 2 + add_len, 0, add_len_aligned - add_len - 2); // padding 0
    memcpy(ctx->ccm_buf + 16 + add_len_aligned, input, length);
    memset(ctx->ccm_buf + 16 + add_len_aligned + length, 0, length_aligned - length); // padding 0

    /* Init CCM Hardware */

    /* Enable AES interrupt */
    AES_ENABLE_INT(CRYPTO);

    /* Basic AES CCM Option */
    keySizeOp = (ctx->keybits == 256) ? AES_KEY_SIZE_256 : (ctx->keybits == 192) ? AES_KEY_SIZE_192 : AES_KEY_SIZE_128;

    CRYPTO->AES_CTL = (mode << CRYPTO_AES_CTL_ENCRYPTO_Pos) | (AES_MODE_CCM << CRYPTO_AES_CTL_OPMODE_Pos) |
                      (keySizeOp << CRYPTO_AES_CTL_KEYSZ_Pos) | (AES_IN_OUT_SWAP << CRYPTO_AES_CTL_OUTSWAP_Pos) | CRYPTO_AES_CTL_KINSWAP_Msk;

    /* Set Key */
    for (i = 0, j = 0; i < ctx->keybits / 8; i += 4)
    {
        CRYPTO->AES_KEY[j++] = ctx->key[i] | (ctx->key[i + 1] << 8) | (ctx->key[i + 2] << 16) | (ctx->key[i + 3] << 24);
    }

    /* Set Ctr0 */
    for (i = 0, j = 0; i < 16; i += 4)
    {
        CRYPTO->AES_IV[j++] = ctx->ctr[i] | (ctx->ctr[i + 1] << 8) | (ctx->ctr[i + 2] << 16) | (ctx->ctr[i + 3] << 24);
    }

    /* Length settings. we assume length < 2^32 here */
    CRYPTO->AES_GCM_ACNT[0] = 16 + add_len_aligned;
    CRYPTO->AES_GCM_ACNT[1] = 0;
    CRYPTO->AES_GCM_PCNT[0] = length;
    CRYPTO->AES_GCM_PCNT[1] = 0;

#if (NVT_DCACHE_ON == 1)
    SCB_CleanDCache_by_Addr(ctx->ccm_buf, sizeof(ccm_buf_array));
#endif

    /* Init DMA */
    CRYPTO->AES_SADDR = (uint32_t)ctx->ccm_buf;
    CRYPTO->AES_DADDR = (uint32_t)ctx->out_buf;
    CRYPTO->AES_CNT = 16 + add_len_aligned + length_aligned;

    /* Clear flag */
    CRYPTO->INTSTS = CRYPTO_INTSTS_AESIF_Msk;

    /* Start AES CCM */
    CRYPTO->AES_CTL |= CRYPTO_AES_CTL_START_Msk | (CRYPTO_DMA_ONE_SHOT << CRYPTO_AES_CTL_DMALAST_Pos);

    timeout = SystemCoreClock;

    while ((CRYPTO->INTSTS & CRYPTO_INTSTS_AESIF_Msk) == 0)
    {
        /* Check timeout */
        if (timeout-- <= 0)
            return MBEDTLS_ERR_PLATFORM_HW_ACCEL_FAILED;
    }

#if (NVT_DCACHE_ON == 1)
    SCB_InvalidateDCache_by_Addr(ctx->out_buf, sizeof(out_buf_array));
#endif

    /* output */
    if (output != NULL)
        memcpy(output, ctx->out_buf, length);

    if (tag != NULL)
        memcpy(tag, ctx->out_buf + length_aligned, tag_len);

    mbedtls_ccm_clear_state(ctx);

    return (0);
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
    return (ccm_auth_crypt(ctx, MBEDTLS_CCM_STAR_ENCRYPT, length, iv, iv_len,
                           add, add_len, input, output, tag, tag_len));
}

int mbedtls_ccm_encrypt_and_tag(mbedtls_ccm_context *ctx, size_t length,
                                const unsigned char *iv, size_t iv_len,
                                const unsigned char *add, size_t add_len,
                                const unsigned char *input, unsigned char *output,
                                unsigned char *tag, size_t tag_len)
{
    return (ccm_auth_crypt(ctx, MBEDTLS_CCM_ENCRYPT, length, iv, iv_len,
                           add, add_len, input, output, tag, tag_len));
}

/*
 * Authenticated decryption
 */
static int mbedtls_ccm_compare_tags(const unsigned char *tag1, const unsigned char *tag2, size_t tag_len)
{
    unsigned char i;
    int diff;

    /* Check tag in "constant-time" */
    for (diff = 0, i = 0; i < tag_len; i++)
        diff |= tag1[i] ^ tag2[i];

    if (diff != 0)
    {
        return (MBEDTLS_ERR_CCM_AUTH_FAILED);
    }

    return (0);
}

static int ccm_auth_decrypt(mbedtls_ccm_context *ctx, int mode, size_t length,
                            const unsigned char *iv, size_t iv_len,
                            const unsigned char *add, size_t add_len,
                            const unsigned char *input, unsigned char *output,
                            const unsigned char *tag, size_t tag_len)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    unsigned char check_tag[16];

    if ((ret = ccm_auth_crypt(ctx, mode, length,
                              iv, iv_len, add, add_len,
                              input, output, check_tag, tag_len)) != 0)
    {
        return (ret);
    }

    if ((ret = mbedtls_ccm_compare_tags(tag, check_tag, tag_len)) != 0)
    {
        mbedtls_platform_zeroize(output, length);
        return (ret);
    }

    return (0);
}

int mbedtls_ccm_star_auth_decrypt(mbedtls_ccm_context *ctx, size_t length,
                                  const unsigned char *iv, size_t iv_len,
                                  const unsigned char *add, size_t add_len,
                                  const unsigned char *input, unsigned char *output,
                                  const unsigned char *tag, size_t tag_len)
{
    return ccm_auth_decrypt(ctx, MBEDTLS_CCM_STAR_DECRYPT, length,
                            iv, iv_len, add, add_len,
                            input, output, tag, tag_len);
}

int mbedtls_ccm_auth_decrypt(mbedtls_ccm_context *ctx, size_t length,
                             const unsigned char *iv, size_t iv_len,
                             const unsigned char *add, size_t add_len,
                             const unsigned char *input, unsigned char *output,
                             const unsigned char *tag, size_t tag_len)
{
    return ccm_auth_decrypt(ctx, MBEDTLS_CCM_DECRYPT, length,
                            iv, iv_len, add, add_len,
                            input, output, tag, tag_len);
}

#endif /* MBEDTLS_CCM_C */
