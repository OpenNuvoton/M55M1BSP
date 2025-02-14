/*
 *  HKDF implementation -- RFC 5869
 *
 *  Copyright The Mbed TLS Contributors
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

#if defined(MBEDTLS_HKDF_C)
#if defined(NVT_HKDF_ALT)
#include <string.h>
#include <stdio.h>
#include "mbedtls/hkdf.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/error.h"
#include <string.h>
#include "NuMicro.h"
#include "tests/include/test/helpers.h"

#define NVT_KDF_INFO_CTXT_OFFSET    13
#define NVT_KDF_INFO_MAGICBYTE_DATA 0x00
#define NVT_KDF_INFO_MAGICBYTE_IDX  12
#define NVT_KDF_INFO_KLEN_HBYTE_IDX 29
#define NVT_KDF_INFO_KLEN_LBYTE_IDX 30

void libByte2Reg_Order(char byteInput[], uint32_t volatile u32Reg[], uint16_t u16inputlen)
{
    char      hex;
    int       si, ri;
    uint32_t  i, val32;

    si = 0;
    ri = 0;

    while (si < u16inputlen)
    {
        val32 = 0UL;

        for (i = 0UL; (i < 4UL) && (si < u16inputlen); i++)
        {
            hex = byteInput[si];
            val32 |= ((uint32_t)(hex) & (0x000000ff)) << ((3 - i) * 8UL); /*Fit GCC casting to uint32_t(0xFFFFFFxx)*/
            //val32 |= (uint32_t)hex << ((3 - i) * 8UL);
            //printf("hex: 0x%02X, shitf:%d\n", hex, (3 - i) * 8UL);
            si++;
        }

        u32Reg[ri++] = val32;
        //printf("val32: 0x%08X\n", val32);
    }
}
/*
   HKDF is the composition of two functions,
   HKDF-Extract and HKDF-Expand:
   HKDF(salt, IKM, info, length) = HKDF-Expand(HKDF-Extract(salt, IKM), info, length)
*/

static int __nvt_hkdf(const mbedtls_md_info_t *md, const unsigned char *salt,
                      size_t salt_len, const unsigned char *ikm, size_t ikm_len,
                      const unsigned char *info, size_t info_len,
                      unsigned char *okm, size_t okm_len)
{

    int32_t i32RetCode = 0;
    uint32_t g_au32Keyout[16]  = { 0 };

    /* Use REG to keep HKDF material info. */

    libByte2Reg_Order((char *)ikm, KDF->KEYIN, 32);

    libByte2Reg_Order((char *)salt, KDF->SALT, 32);

    /* 0~11 bytes are label */
    libByte2Reg_Order((char *)info, KDF->LABEL, 12);

    /* 12th byte is skipped, fed 0x00 by HW*/
    /* 13~28 bytes are ctxt*/
    libByte2Reg_Order((char *)(info + 13), KDF->CTXT, 16);
    /* 29~30 bytes are skipped. They are output key length in bits. It is fed by HW. */
    memset(g_au32Keyout, 0x0,  sizeof(g_au32Keyout));   /* Clear keyout buffer */

    /*
      okm_len is number of words of okm.
      okm_len*8 is the number of bits of okm.
    */
    if ((i32RetCode = KDF_DeriveKey(eKDF_MODE_HKDF, (KDF_KEYIN_FROM_REG | KDF_SALT_FROM_REG | KDF_LABEL_FROM_REG | KDF_CONTEXT_FROM_REG), okm_len * 8, (uint32_t *)okm)) != 0)
    {
        printf("Failed to derive key !\n");
        goto Error_Exit;
    }

    printf("\r\nKDF_DeriveKey done\r\n");

Error_Exit:
    return i32RetCode;
}

int mbedtls_hkdf(const mbedtls_md_info_t *md, const unsigned char *salt,
                 size_t salt_len, const unsigned char *ikm, size_t ikm_len,
                 const unsigned char *info, size_t info_len,
                 unsigned char *okm, size_t okm_len)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;

    ret = __nvt_hkdf(md, salt, salt_len, ikm, ikm_len, info, info_len, okm, okm_len);

    return (ret);
}

int mbedtls_hkdf_extract(const mbedtls_md_info_t *md,
                         const unsigned char *salt, size_t salt_len,
                         const unsigned char *ikm, size_t ikm_len,
                         unsigned char *prk)
{

    return 0;
}

int mbedtls_hkdf_expand(const mbedtls_md_info_t *md, const unsigned char *prk,
                        size_t prk_len, const unsigned char *info,
                        size_t info_len, unsigned char *okm, size_t okm_len)
{

    return 0;
}
#endif /*NVT_HKDF_ALT*/
#endif /* MBEDTLS_HKDF_C */