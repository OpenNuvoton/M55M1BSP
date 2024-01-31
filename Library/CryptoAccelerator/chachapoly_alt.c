/**
 * \file chachapoly_alt.c
 *
 * \brief ChaCha20-Poly1305 AEAD construction based on RFC 7539.
 *
 *  Copyright The Mbed TLS Contributors
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
 */
#include "common.h"

#if defined(MBEDTLS_CHACHAPOLY_C)

#include "mbedtls/chachapoly.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/error.h"

#include <string.h>

#if defined(MBEDTLS_SELF_TEST)
    #if defined(MBEDTLS_PLATFORM_C)
        #include "mbedtls/platform.h"
    #else
        #include <stdio.h>
        #define mbedtls_printf printf
    #endif /* MBEDTLS_PLATFORM_C */
#endif /* MBEDTLS_SELF_TEST */

#if defined(MBEDTLS_CHACHAPOLY_ALT)
#include "NuMicro.h"
/* Parameter validation macros */
#define CHACHAPOLY_VALIDATE_RET( cond )                                       \
    MBEDTLS_INTERNAL_VALIDATE_RET( cond, MBEDTLS_ERR_POLY1305_BAD_INPUT_DATA )
#define CHACHAPOLY_VALIDATE( cond )                                           \
    MBEDTLS_INTERNAL_VALIDATE( cond )

extern uint8_t  *au8InputData;
extern uint8_t  *au8OutputData;
extern uint8_t  *au8CascadeOut;
extern uint8_t  *au8FDBCK;
extern uint8_t  au8TempBuff1[];
extern uint8_t  au8TempBuff2[];
extern uint8_t  au8CascadeBuff[];

static void nvt_chapoly_aead_run(int is_encrypt, unsigned char *key, unsigned char *nonce,
                                 int aad_len, unsigned char *aad,
                                 int input_len, unsigned char *input,
                                 unsigned char *output, unsigned char *mac);
/**
 * \brief           Adds nul bytes to pad the AAD for Poly1305.
 *
 * \param ctx       The ChaCha20-Poly1305 context.
 */
static int chachapoly_pad_aad(mbedtls_chachapoly_context *ctx)
{
    return 0;
}

/**
 * \brief           Adds nul bytes to pad the ciphertext for Poly1305.
 *
 * \param ctx       The ChaCha20-Poly1305 context.
 */
static int chachapoly_pad_ciphertext(mbedtls_chachapoly_context *ctx)
{
    return 0;
}

void mbedtls_chachapoly_init(mbedtls_chachapoly_context *ctx)
{

}

void mbedtls_chachapoly_free(mbedtls_chachapoly_context *ctx)
{
    if (ctx == NULL)
        return;
}

int mbedtls_chachapoly_setkey(mbedtls_chachapoly_context *ctx,
                              const unsigned char key[32])
{
    int i;
    printf("mbedtls_chachapoly_setkey\r\n");

    for (i = 0; i < 32; i++)
    {
        ctx->key[i] = key[i ];
    }

    return (0);
}

int mbedtls_chachapoly_starts(mbedtls_chachapoly_context *ctx,
                              const unsigned char nonce[12],
                              mbedtls_chachapoly_mode_t mode)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    return (ret);
}

int mbedtls_chachapoly_update_aad(mbedtls_chachapoly_context *ctx,
                                  const unsigned char *aad,
                                  size_t aad_len)
{

    return 0;
}

int mbedtls_chachapoly_update(mbedtls_chachapoly_context *ctx,
                              size_t len,
                              const unsigned char *input,
                              unsigned char *output)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;


    return (0);
}

int mbedtls_chachapoly_finish(mbedtls_chachapoly_context *ctx,
                              unsigned char mac[16])
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;

    return (ret);
}

static int chachapoly_crypt_and_tag(mbedtls_chachapoly_context *ctx,
                                    mbedtls_chachapoly_mode_t mode,
                                    size_t length,
                                    const unsigned char nonce[12],
                                    const unsigned char *aad,
                                    size_t aad_len,
                                    const unsigned char *input,
                                    unsigned char *output,
                                    unsigned char tag[16])
{
    //int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    int encrypt_flag;

    if (mode == MBEDTLS_CHACHAPOLY_ENCRYPT)  encrypt_flag = 1;
    else if (mode == MBEDTLS_CHACHAPOLY_DECRYPT)  encrypt_flag = 0;
    else
    {
        printf("missing mode\r\n");
        return -1;
    }

    nvt_chapoly_aead_run(encrypt_flag, ctx->key, nonce, aad_len, aad, length, input, output, tag);
    return (0);
}

int mbedtls_chachapoly_encrypt_and_tag(mbedtls_chachapoly_context *ctx,
                                       size_t length,
                                       const unsigned char nonce[12],
                                       const unsigned char *aad,
                                       size_t aad_len,
                                       const unsigned char *input,
                                       unsigned char *output,
                                       unsigned char tag[16])
{
    CHACHAPOLY_VALIDATE_RET(ctx   != NULL);
    CHACHAPOLY_VALIDATE_RET(nonce != NULL);
    CHACHAPOLY_VALIDATE_RET(tag   != NULL);
    CHACHAPOLY_VALIDATE_RET(aad_len == 0 || aad    != NULL);
    CHACHAPOLY_VALIDATE_RET(length  == 0 || input  != NULL);
    CHACHAPOLY_VALIDATE_RET(length  == 0 || output != NULL);

    return (chachapoly_crypt_and_tag(ctx, MBEDTLS_CHACHAPOLY_ENCRYPT,
                                     length, nonce, aad, aad_len,
                                     input, output, tag));
}

int mbedtls_chachapoly_auth_decrypt(mbedtls_chachapoly_context *ctx,
                                    size_t length,
                                    const unsigned char nonce[12],
                                    const unsigned char *aad,
                                    size_t aad_len,
                                    const unsigned char tag[16],
                                    const unsigned char *input,
                                    unsigned char *output)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    unsigned char check_tag[16];
    size_t i;
    int diff;
    CHACHAPOLY_VALIDATE_RET(ctx   != NULL);
    CHACHAPOLY_VALIDATE_RET(nonce != NULL);
    CHACHAPOLY_VALIDATE_RET(tag   != NULL);
    CHACHAPOLY_VALIDATE_RET(aad_len == 0 || aad    != NULL);
    CHACHAPOLY_VALIDATE_RET(length  == 0 || input  != NULL);
    CHACHAPOLY_VALIDATE_RET(length  == 0 || output != NULL);

    if ((ret = chachapoly_crypt_and_tag(ctx,
                                        MBEDTLS_CHACHAPOLY_DECRYPT, length, nonce,
                                        aad, aad_len, input, output, check_tag)) != 0)
    {
        return (ret);
    }

    /* Check tag in "constant-time" */
    for (diff = 0, i = 0; i < sizeof(check_tag); i++)
        diff |= tag[i] ^ check_tag[i];

    if (diff != 0)
    {
        mbedtls_platform_zeroize(output, length);
        return (MBEDTLS_ERR_CHACHAPOLY_AUTH_FAILED);
    }

    return (0);
}


static void nvt_chapoly_aead_run(int is_encrypt, unsigned char *key, unsigned char *nonce,
                                 int aad_len, unsigned char *aad,
                                 int input_len, unsigned char *input,
                                 unsigned char *output, unsigned char *mac)
{
    uint32_t  ctrl = 0;
    uint8_t   *in_block = au8TempBuff1;
    int       i, dlen;

    dlen = 0;
    memcpy(&in_block[dlen], aad, aad_len);
    dlen += aad_len;

    if (aad_len % 16)
    {
        memset(&in_block[dlen], 0, (16 - (aad_len % 16)));
        dlen += (16 - (aad_len % 16));
    }

    memcpy(&in_block[dlen], input, input_len);
    dlen += input_len;

    if (input_len % 16)
    {
        memset(&in_block[dlen], 0, (16 - (input_len % 16)));
        dlen += (16 - (input_len % 16));
    }


    for (i = 0; i < 8; i++)
    {
        CRYPTO->CHAPOLY_KEY[i] = *(uint32_t *)(&key[i * 4]);
        //printf("CRYPTO->CHAPOLY_KEY%d(W only) = 0x%08x [0x%08x]\n", i, CRYPTO->CHAPOLY_KEY[i], *(uint32_t *)(&key[i * 4]));
    }

    for (i = 0; i < 3; i++)
    {
        CRYPTO->CHAPOLY_NONCE[i] = *(uint32_t *)(&nonce[i * 4]);
        //printf("CRYPTO->CHAPOLY_NONCE%d(R/W) = 0x%08x [0x%08x]\n", i, CRYPTO->CHAPOLY_NONCE[i], *(uint32_t *)(&nonce[i * 4]));
    }

    CRYPTO->CHAPOLY_ACNT[0] = aad_len;
    CRYPTO->CHAPOLY_ACNT[1] = 0;
    CRYPTO->CHAPOLY_PCNT[0] = input_len;
    CRYPTO->CHAPOLY_PCNT[1] = 0;

    CRYPTO->CHAPOLY_BLOCKCNT = 0;
    CRYPTO->CHAPOLY_SADDR = (uint32_t)in_block;
    CRYPTO->CHAPOLY_DADDR = (uint32_t)output;
    CRYPTO->CHAPOLY_CNT = dlen;

    CHAPOLY_Start(CRYPTO);

    dlen = ((input_len + 15) / 16) * 16;
    memcpy(mac, &output[dlen], 16);
    printf("nvt_chapoly_aead_run done\n");
}
#endif /* MBEDTLS_CHACHAPOLY_ALT */

#endif /* MBEDTLS_CHACHAPOLY_C */
