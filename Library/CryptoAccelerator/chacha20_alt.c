/**
 * \file chacha20_alt.c
 *
 * \brief ChaCha20 cipher with crypto engine.
 *
 *
 *  Copyright (C) 2022, Nuvoton Technology Corporation, All Rights Reserved.
 *  SPDX-License-Identifier: Apache-2.0
 *
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

#if defined(MBEDTLS_CHACHA20_C)

#include "mbedtls/chacha20.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/error.h"

#include <stddef.h>
#include <string.h>

//#if defined(MBEDTLS_SELF_TEST)
#if defined(MBEDTLS_PLATFORM_C)
    #include "mbedtls/platform.h"
#else
    #include <stdio.h>
    #define mbedtls_printf printf
#endif /* MBEDTLS_PLATFORM_C */
//#endif /* MBEDTLS_SELF_TEST */

#if defined(MBEDTLS_CHACHA20_ALT)
#include "NuMicro.h"

////#if ( defined(__ARMCC_VERSION) || defined(_MSC_VER) ) && \
////    !defined(inline) && !defined(__cplusplus)
////#define inline __inline
////#endif

/////* Parameter validation macros */
////#define CHACHA20_VALIDATE_RET( cond )                                       \
////    MBEDTLS_INTERNAL_VALIDATE_RET( cond, MBEDTLS_ERR_CHACHA20_BAD_INPUT_DATA )
////#define CHACHA20_VALIDATE( cond )                                           \
////    MBEDTLS_INTERNAL_VALIDATE( cond )

////#define ROTL32( value, amount ) \
////    ( (uint32_t) ( (value) << (amount) ) | ( (value) >> ( 32 - (amount) ) ) )

////#define CHACHA20_CTR_INDEX ( 12U )

////#define CHACHA20_BLOCK_SIZE_BYTES ( 4U * 16U )

extern volatile int g_CHAPOLY_done;
extern uint8_t  *au8InputData;
extern uint8_t  *au8OutputData;

void mbedtls_chacha20_init(mbedtls_chacha20_context *ctx)
{

    printf("mbedtls_chacha20_init\n");
}

void mbedtls_chacha20_free(mbedtls_chacha20_context *ctx)
{

    printf("mbedtls_chacha20_free\n");
}

int mbedtls_chacha20_setkey(mbedtls_chacha20_context *ctx,
                            const unsigned char key[32])
{

    printf("mbedtls_chacha20_setkey\n");

    return (0);
}

int mbedtls_chacha20_starts(mbedtls_chacha20_context *ctx,
                            const unsigned char nonce[12],
                            uint32_t counter)
{
    printf("mbedtls_chacha20_starts\n");
    return (0);
}

int mbedtls_chacha20_update(mbedtls_chacha20_context *ctx,
                            size_t size,
                            const unsigned char *input,
                            unsigned char *output)
{
    size_t offset = 0U;
    size_t i;


    printf("mbedtls_chacha20_update\n");
    return (0);
}

int mbedtls_chacha20_crypt(const unsigned char key[32],
                           const unsigned char nonce[12],
                           uint32_t counter,
                           size_t data_len,
                           const unsigned char *input,
                           unsigned char *output)
{
    printf("mbedtls_chacha20_crypt\n");

    CHA_SetKeyandNonce(CRYPTO, (unsigned char *)key, (unsigned char *)nonce, counter);

    CHA_SetDMATransfer(CRYPTO, input,  output, data_len);

    CHA_Start(CRYPTO, 1);
    return 0;
}



#endif /* MBEDTLS_CHACHA20_ALT */



#endif /* MBEDTLS_CHACHA20_C */
