/*
 *  Elliptic curve Diffie-Hellman
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

/*
 * References:
 *
 * SEC1 http://www.secg.org/index.MBEDTLS_PRIVATE(p)hp?action=secg,docs_secg
 * RFC 4492
 */

#include "common.h"


#if defined(MBEDTLS_ECDH_C)

#include "mbedtls/ecdh.h"
#include "mbedtls/platform_util.h"
#include "mbedtls/error.h"

#if defined(MBEDTLS_PLATFORM_C)
    #include "mbedtls/platform.h"
#else
    #include <stdlib.h>
    #include <stdio.h>
    #define mbedtls_printf     printf
    #define mbedtls_calloc    calloc
    #define mbedtls_free       free
#endif

#include <string.h>
#include "NuMicro.h"

#define MBEDTLS_ERR_ECP_HW_ACCEL_FAILED -1

/* Parameter validation macros based on platform_util.h */
#define ECDH_VALIDATE_RET( cond )    \
    MBEDTLS_INTERNAL_VALIDATE_RET( cond, MBEDTLS_ERR_ECP_BAD_INPUT_DATA )
#define ECDH_VALIDATE( cond )        \
    MBEDTLS_INTERNAL_VALIDATE( cond )

#if (defined(MBEDTLS_ECDH_GEN_PUBLIC_ALT) || defined(MBEDTLS_ECDH_COMPUTE_SHARED_ALT))

#define ECCOP_POINT_MUL     (0x0UL << CRYPTO_ECC_CTL_ECCOP_Pos)
#define ECCOP_MODULE        (0x1UL << CRYPTO_ECC_CTL_ECCOP_Pos)
#define ECCOP_POINT_ADD     (0x2UL << CRYPTO_ECC_CTL_ECCOP_Pos)
#define ECCOP_POINT_DOUBLE  (0x3UL << CRYPTO_ECC_CTL_ECCOP_Pos)

#define MODOP_DIV           (0x0UL << CRYPTO_ECC_CTL_MODOP_Pos)
#define MODOP_MUL           (0x1UL << CRYPTO_ECC_CTL_MODOP_Pos)
#define MODOP_ADD           (0x2UL << CRYPTO_ECC_CTL_MODOP_Pos)
#define MODOP_SUB           (0x3UL << CRYPTO_ECC_CTL_MODOP_Pos)



static void ECC_Copy(uint32_t *dest, uint32_t *src, uint32_t size)
{
    uint32_t u32Data, *pu32Dest, *pu32Src;
    int32_t i;
    uint32_t len;
    uint8_t *pu8;

    len = (uint32_t)size;
    pu32Dest = (uint32_t *)dest;
    pu32Src = (uint32_t *)src;

    for (i = 0; i < len / 4; i++)
    {
        *pu32Dest++ = *pu32Src++;
    }

    len = size & 0x3;

    if (len > 0)
    {
        pu8 = (uint8_t *)pu32Src;
        u32Data = 0;

        for (i = 0; i < len; i++)
        {
            u32Data += (*pu8++) << (i * 8);
        }

        *pu32Dest = u32Data;
    }
}

/* Add mission parameters of the curve */
static int ECC_FixCurve(mbedtls_ecp_group *grp)
{

    if (grp->MBEDTLS_PRIVATE(T) == NULL)
    {
        grp->MBEDTLS_PRIVATE(T) = mbedtls_calloc(1, sizeof(mbedtls_ecp_point));

        if (grp->MBEDTLS_PRIVATE(T) == NULL)
        {
            return MBEDTLS_ERR_ECP_ALLOC_FAILED;
        }

        mbedtls_ecp_point_init(grp->MBEDTLS_PRIVATE(T));
        mbedtls_mpi_lset(&grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(Z), 0);
        grp->MBEDTLS_PRIVATE(T_size) = 1;
    }


    if (mbedtls_mpi_size(&grp->A) < 1)
    {
        if (grp->id == MBEDTLS_ECP_DP_SECP192R1)
        {
            mbedtls_mpi_read_string(&grp->A, 16, "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFFFFFFFFFFFC");
            //mbedtls_mpi_read_string(&grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(X), 16, "188da80eb03090f67cbf20eb43a18800f4ff0afd82ff1012");
            //mbedtls_mpi_read_string(&grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(Y), 16, "07192b95ffc8da78631011ed6b24cdd573f977a11e794811");
        }
        else if (grp->id == MBEDTLS_ECP_DP_SECP224R1)
        {
            mbedtls_mpi_read_string(&grp->A, 16, "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFFFFFFFFFFFFFFFFFFFE");
            //mbedtls_mpi_read_string(&grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(X), 16, "b70e0cbd6bb4bf7f321390b94a03c1d356c21122343280d6115c1d21");
            //mbedtls_mpi_read_string(&grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(Y), 16, "bd376388b5f723fb4c22dfe6cd4375a05a07476444d5819985007e34");
        }
        else if (grp->id == MBEDTLS_ECP_DP_SECP256R1)
        {
            mbedtls_mpi_read_string(&grp->A, 16, "FFFFFFFF00000001000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFC");
            //mbedtls_mpi_read_string(&grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(X), 16, "6b17d1f2e12c4247f8bce6e563a440f277037d812deb33a0f4a13945d898c296");
            //mbedtls_mpi_read_string(&grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(Y), 16, "4fe342e2fe1a7f9b8ee7eb4a7c0f9e162bce33576b315ececbb6406837bf51f5");
        }
        else if (grp->id == MBEDTLS_ECP_DP_SECP384R1)
        {
            mbedtls_mpi_read_string(&grp->A, 16, "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFFFF0000000000000000FFFFFFFC");
            //mbedtls_mpi_read_string(&grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(X), 16, "aa87ca22be8b05378eb1c71ef320ad746e1d3b628ba79b9859f741e082542a385502f25dbf55296c3a545e3872760ab7");
            //mbedtls_mpi_read_string(&grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(Y), 16, "3617de4a96262c6f5d9e98bf9292dc29f8f41dbd289a147ce9da3113b5f0b8c00a60b1ce1d7e819d7a431d7c90ea0e5f");
        }
        else if (grp->id == MBEDTLS_ECP_DP_SECP521R1)
        {
            mbedtls_mpi_read_string(&grp->A, 16, "1FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC");
            //mbedtls_mpi_read_string(&grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(X), 16, "0c6858e06b70404e9cd9e3ecb662395b4429c648139053fb521f828af606b4d3dbaa14b5e77efe75928fe1dc127a2ffa8de3348b3c1856a429bf97e7e31c2e5bd66");
            //mbedtls_mpi_read_string(&grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(Y), 16, "11839296a789a3bc0045c8a5fb42c7d1bd998f54449579b446817afbd17273e662c97ee72995ef42640c550b9013fad0761353c7086a272c24088be94769fd16650");
        }
        else if (grp->id == MBEDTLS_ECP_DP_CURVE25519)
        {
            mbedtls_mpi_read_string(&grp->A, 16, "0000000000000000000000000000000000000000000000000000000000076D06");
            mbedtls_mpi_read_string(&grp->B, 16, "0000000000000000000000000000000000000000000000000000000000000001");
            mbedtls_mpi_read_string(&grp->P, 16, "7fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffed");
            mbedtls_mpi_read_string(&grp->N, 16, "1000000000000000000000000000000014def9dea2f79cd65812631a5cf5d3ed");
            //mbedtls_mpi_read_string(&grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(X), 16, "0000000000000000000000000000000000000000000000000000000000000009");
            //mbedtls_mpi_read_string(&grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(Y), 16, "20ae19a1b8a086b4e01edd2c7748d14c923d4d7e6d7c61b229e9c5a27eced3d9");
        }
    }

    return 0;
}
#endif //defined(MBEDTLS_ECDH_GEN_PUBLIC_ALT) || defined(MBEDTLS_ECDH_COMPUTE_SHARED_ALT)

#if defined(MBEDTLS_ECDH_GEN_PUBLIC_ALT)

static int32_t ECC_GenPubKey(mbedtls_ecp_group *grp, mbedtls_mpi *d, mbedtls_ecp_point *Q)
{
    CRYPTO_T *crpt;
    uint32_t timeout = 200000000;
    int32_t len;

    /* Reset Crypto */
    SYS_UnlockReg();
    SYS->CRYPTORST |= SYS_CRYPTORST_CRYPTO0RST_Msk;
    SYS->CRYPTORST = 0;
    SYS_LockReg();

    crpt = CRYPTO;

    //crpt->ECC_KSCTL = 0;
    ECC_ENABLE_INT(crpt);

    /* Curve relative init */
    memset((void *)crpt->ECC_A, 0, 72);
    memset((void *)crpt->ECC_B, 0, 72);
    memset((void *)crpt->ECC_X1, 0, 72);
    memset((void *)crpt->ECC_Y1, 0, 72);
    memset((void *)crpt->ECC_N, 0, 72);
    memset((void *)crpt->ECC_X2, 0, 72);

    ECC_FixCurve(grp);

    ECC_Copy((void *)crpt->ECC_A, grp->A.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->A));
    ECC_Copy((void *)crpt->ECC_B, grp->B.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->B));

    ECC_Copy((void *)crpt->ECC_X1, grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(X).MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(X)));
    ECC_Copy((void *)crpt->ECC_Y1, grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(Y).MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(Y)));
    ECC_Copy((void *)crpt->ECC_N, grp->P.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->P));
    ECC_Copy((void *)crpt->ECC_X2, grp->N.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->N));

    memset((void *)crpt->ECC_K, 0, 72);
    ECC_Copy((void *)crpt->ECC_K, d->MBEDTLS_PRIVATE(p), mbedtls_mpi_size(d));

    /* set FSEL (Field selection) */
    // For CURVE_GF_2M
    //crpt->ECC_CTL = 0UL;

    // for CURVE_GF_P
    crpt->ECC_CTL = CRYPTO_ECC_CTL_FSEL_Msk;

    /* enable side-channel attack protection */
    crpt->ECC_CTL |= CRYPTO_ECC_CTL_SCAP_Msk | CRYPTO_ECC_CTL_ASCAP_Msk;

    /* Clear ECC flag */
    crpt->INTSTS = CRYPTO_INTSTS_ECCIF_Msk;

    /* Start calculation */
    crpt->ECC_CTL |= (grp->pbits << CRYPTO_ECC_CTL_CURVEM_Pos) |
                     ECCOP_POINT_MUL | CRYPTO_ECC_CTL_PFA2C_Msk | CRYPTO_ECC_CTL_START_Msk;

    /* Waiting for calculation */
    while ((crpt->INTSTS & CRYPTO_INTSTS_ECCIF_Msk) == 0)
    {
        if (timeout-- <= 0)
            return MBEDTLS_ERR_ECP_HW_ACCEL_FAILED;
    }

    len = grp->pbits / 8 + ((grp->pbits & 0x7) != 0);
    mbedtls_mpi_grow(&Q->MBEDTLS_PRIVATE(X), len);
    mbedtls_mpi_grow(&Q->MBEDTLS_PRIVATE(Y), len);

    memcpy((void *)Q->MBEDTLS_PRIVATE(X).MBEDTLS_PRIVATE(p), (void *)crpt->ECC_X1, len);
    memcpy((void *)Q->MBEDTLS_PRIVATE(Y).MBEDTLS_PRIVATE(p), (void *)crpt->ECC_Y1, len);
    mbedtls_mpi_lset(&Q->MBEDTLS_PRIVATE(Z), 1);

    return 0;
}

/*
 * Generate public key (restartable version)
 *
 * Note: this internal function relies on its caller preserving the value of
 * the output parameter 'd' across continuation calls. This would not be
 * acceptable for a public function but is OK here as we control call sites.
 */
static int ecdh_gen_public_restartable(mbedtls_ecp_group *grp,
                                       mbedtls_mpi *d, mbedtls_ecp_point *Q,
                                       int (*f_rng)(void *, unsigned char *, size_t),
                                       void *p_rng,
                                       mbedtls_ecp_restart_ctx *rs_ctx)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;

    MBEDTLS_MPI_CHK(mbedtls_ecp_gen_privkey(grp, d, f_rng, p_rng));

    ECC_GenPubKey(grp, d, Q);


cleanup:
    return (ret);
}

/*
 * Generate public key
 */
int mbedtls_ecdh_gen_public(mbedtls_ecp_group *grp, mbedtls_mpi *d, mbedtls_ecp_point *Q,
                            int (*f_rng)(void *, unsigned char *, size_t),
                            void *p_rng)
{
    ECDH_VALIDATE_RET(grp != NULL);
    ECDH_VALIDATE_RET(d != NULL);
    ECDH_VALIDATE_RET(Q != NULL);
    ECDH_VALIDATE_RET(f_rng != NULL);

    return (ecdh_gen_public_restartable(grp, d, Q, f_rng, p_rng, NULL));
}
#endif /* MBEDTLS_ECDH_GEN_PUBLIC_ALT */



#if defined(MBEDTLS_ECDH_COMPUTE_SHARED_ALT)



int32_t  ECC_ComputeShared(mbedtls_ecp_group *grp, mbedtls_mpi *z, const mbedtls_ecp_point *Q, const mbedtls_mpi *d)
{
    CRYPTO_T *crpt;
    uint32_t timeout = 200000000;
    int32_t len;
    int32_t ret;


    /* Reset Crypto */
    SYS_UnlockReg();
    SYS->CRYPTORST |= SYS_CRYPTORST_CRYPTO0RST_Msk;
    SYS->CRYPTORST = 0;
    SYS_LockReg();

    crpt = CRYPTO;

    //crpt->ECC_KSCTL = 0;
    ECC_ENABLE_INT(crpt);

    /* Curve relative init */
    memset((void *)crpt->ECC_A, 0, 72);
    memset((void *)crpt->ECC_B, 0, 72);
    memset((void *)crpt->ECC_X1, 0, 72);
    memset((void *)crpt->ECC_Y1, 0, 72);
    memset((void *)crpt->ECC_N, 0, 72);

    memset((void *)crpt->ECC_K, 0, 72);
    memset((void *)crpt->ECC_X2, 0, 72);
    memset((void *)crpt->ECC_Y2, 0, 72);

    if ((ret = ECC_FixCurve(grp)) != 0)
        return ret;


    ECC_Copy((void *)crpt->ECC_A, grp->A.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->A));
    ECC_Copy((void *)crpt->ECC_B, grp->B.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->B));
    ECC_Copy((void *)crpt->ECC_N, grp->P.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->P));
    ECC_Copy((void *)crpt->ECC_X2, grp->N.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->N));

    ECC_Copy((void *)crpt->ECC_K, d->MBEDTLS_PRIVATE(p), mbedtls_mpi_size(d));
    ECC_Copy((void *)crpt->ECC_X1, Q->MBEDTLS_PRIVATE(X).MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&Q->MBEDTLS_PRIVATE(X)));
    ECC_Copy((void *)crpt->ECC_Y1, Q->MBEDTLS_PRIVATE(Y).MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&Q->MBEDTLS_PRIVATE(Y)));

    /* set FSEL (Field selection) */
    // For CURVE_GF_2M
    //crpt->ECC_CTL = 0UL;

    // for CURVE_GF_P
    crpt->ECC_CTL = CRYPTO_ECC_CTL_FSEL_Msk;

    /* Clear ECC flag */
    crpt->INTSTS = CRYPTO_INTSTS_ECCIF_Msk;

    /* Start calculation */
    crpt->ECC_CTL |= (grp->pbits << CRYPTO_ECC_CTL_CURVEM_Pos) | ECCOP_POINT_MUL | CRYPTO_ECC_CTL_SCAP_Msk | CRYPTO_ECC_CTL_START_Msk;

    /* Waiting for calculation */
    while ((crpt->INTSTS & CRYPTO_INTSTS_ECCIF_Msk) == 0)
    {
        if (timeout-- <= 0)
            return MBEDTLS_ERR_ECP_HW_ACCEL_FAILED;
    }

    mbedtls_mpi_lset(z, 0);

    len = grp->pbits / 8 + ((grp->pbits & 0x7) != 0);
    mbedtls_mpi_grow(z, len);
    memcpy(z->MBEDTLS_PRIVATE(p), (void *)crpt->ECC_X1, len);

    return 0;
}



/*
 * Compute shared secret (SEC1 3.3.1)
 */
static int ecdh_compute_shared_restartable(mbedtls_ecp_group *grp,
                                           mbedtls_mpi *z,
                                           const mbedtls_ecp_point *Q, const mbedtls_mpi *d,
                                           int (*f_rng)(void *, unsigned char *, size_t),
                                           void *p_rng,
                                           mbedtls_ecp_restart_ctx *rs_ctx)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;

    MBEDTLS_MPI_CHK(ECC_ComputeShared(grp, z, Q, d));

    /* No error */
    ret = 0;

cleanup:

    return (ret);
}

/*
 * Compute shared secret (SEC1 3.3.1)
 */
int mbedtls_ecdh_compute_shared(mbedtls_ecp_group *grp, mbedtls_mpi *z,
                                const mbedtls_ecp_point *Q, const mbedtls_mpi *d,
                                int (*f_rng)(void *, unsigned char *, size_t),
                                void *p_rng)
{
    ECDH_VALIDATE_RET(grp != NULL);
    ECDH_VALIDATE_RET(Q != NULL);
    ECDH_VALIDATE_RET(d != NULL);
    ECDH_VALIDATE_RET(z != NULL);

    return (ecdh_compute_shared_restartable(grp, z, Q, d,
                                            f_rng, p_rng, NULL));
}
#endif /* MBEDTLS_ECDH_COMPUTE_SHARED_ALT */
#endif /* MBEDTLS_ECDH_C */
