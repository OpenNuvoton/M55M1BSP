/**
 * \file poly1305_alt.c
 *
 * \brief Poly1305 authentication algorithm with crypto engine.
 *
 *  Copyright The Mbed TLS Contributors
 *  Copyright (C) 2022, Nuvoton Technology Corporation, All Rights Reserved.
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

#if defined(MBEDTLS_POLY1305_C)

#include "mbedtls/poly1305.h"
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

#if defined(MBEDTLS_POLY1305_ALT)
#include "NuMicro.h"

#if defined(MBEDTLS_NO_64BIT_MULTIPLICATION)
static uint64_t mul64(uint32_t a, uint32_t b)
{
    /* a = al + 2**16 ah, b = bl + 2**16 bh */
    const uint16_t al = (uint16_t) a;
    const uint16_t bl = (uint16_t) b;
    const uint16_t ah = a >> 16;
    const uint16_t bh = b >> 16;

    /* ab = al*bl + 2**16 (ah*bl + bl*bh) + 2**32 ah*bh */
    const uint32_t lo = (uint32_t) al * bl;
    const uint64_t me = (uint64_t)((uint32_t) ah * bl) + (uint32_t) al * bh;
    const uint32_t hi = (uint32_t) ah * bh;

    return (lo + (me << 16) + ((uint64_t) hi << 32));
}
#else
static inline uint64_t mul64(uint32_t a, uint32_t b)
{
    return ((uint64_t) a * b);
}
#endif


/**
 * \brief                   Process blocks with Poly1305.
 *
 * \param ctx               The Poly1305 context.
 * \param nblocks           Number of blocks to process. Note that this
 *                          function only processes full blocks.
 * \param input             Buffer containing the input block(s).
 * \param needs_padding     Set to 0 if the padding bit has already been
 *                          applied to the input data before calling this
 *                          function.  Otherwise, set this parameter to 1.
 */
static void poly1305_process(mbedtls_poly1305_context *ctx,
                             size_t nblocks,
                             const unsigned char *input,
                             uint32_t needs_padding)
{

}

/**
 * \brief                   Compute the Poly1305 MAC
 *
 * \param ctx               The Poly1305 context.
 * \param mac               The buffer to where the MAC is written. Must be
 *                          big enough to contain the 16-byte MAC.
 */
static void poly1305_compute_mac(const mbedtls_poly1305_context *ctx,
                                 unsigned char mac[16])
{

}

void mbedtls_poly1305_init(mbedtls_poly1305_context *ctx)
{


}

void mbedtls_poly1305_free(mbedtls_poly1305_context *ctx)
{

}

int mbedtls_poly1305_starts(mbedtls_poly1305_context *ctx,
                            const unsigned char key[32])
{


    return (0);
}

int mbedtls_poly1305_update(mbedtls_poly1305_context *ctx,
                            const unsigned char *input,
                            size_t ilen)
{


    return (0);
}

int mbedtls_poly1305_finish(mbedtls_poly1305_context *ctx,
                            unsigned char mac[16])
{


    return (0);
}

int mbedtls_poly1305_mac(const unsigned char key[32],
                         const unsigned char *input,
                         size_t ilen,
                         unsigned char mac[16])
{

    POLY1305_SetKeyandClearNonce(CRYPTO, (unsigned char *)key);

    POLY1305_SetDMATransfer(CRYPTO, input,  mac, ilen);

    POLY1305_Start(CRYPTO);
    return (0);
}

#endif /* MBEDTLS_POLY1305_ALT */


#endif /* MBEDTLS_POLY1305_C */
