/*
 *  Elliptic curve DSA
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
 * SEC1 http://www.secg.org/index.php?action=secg,docs_secg
 */


#include "common.h"


#if defined(MBEDTLS_ECDSA_C)

#include "mbedtls/ecdsa.h"
#include "mbedtls/asn1write.h"

#include <string.h>

#if defined(MBEDTLS_PLATFORM_C)
    #include "mbedtls/platform.h"
#else
    #include <stdlib.h>
    #define mbedtls_calloc    calloc
    #define mbedtls_free       free
#endif

#include "mbedtls/platform_util.h"
#include "mbedtls/error.h"

#include "NuMicro.h"

#define MBEDTLS_ERR_ECP_HW_ACCEL_FAILED -1

/* Parameter validation macros based on platform_util.h */
#define ECDSA_VALIDATE_RET( cond )    \
    MBEDTLS_INTERNAL_VALIDATE_RET( cond, MBEDTLS_ERR_ECP_BAD_INPUT_DATA )
#define ECDSA_VALIDATE( cond )        \
    MBEDTLS_INTERNAL_VALIDATE( cond )

//#if defined( MBEDTLS_ECDSA_SIGN_ALT || MBEDTLS_ECDSA_VERIFY_ALT )
#if( defined(MBEDTLS_ECDSA_SIGN_ALT) || defined(MBEDTLS_ECDSA_VERIFY_ALT) )

#define ECCOP_POINT_MUL     (0x0UL << CRYPTO_ECC_CTL_ECCOP_Pos)
#define ECCOP_MODULE        (0x1UL << CRYPTO_ECC_CTL_ECCOP_Pos)
#define ECCOP_POINT_ADD     (0x2UL << CRYPTO_ECC_CTL_ECCOP_Pos)
#define ECCOP_POINT_DOUBLE  (0x3UL << CRYPTO_ECC_CTL_ECCOP_Pos)

#define MODOP_DIV           (0x0UL << CRYPTO_ECC_CTL_MODOP_Pos)
#define MODOP_MUL           (0x1UL << CRYPTO_ECC_CTL_MODOP_Pos)
#define MODOP_ADD           (0x2UL << CRYPTO_ECC_CTL_MODOP_Pos)
#define MODOP_SUB           (0x3UL << CRYPTO_ECC_CTL_MODOP_Pos)


/*
 * Derive a suitable integer for group grp from a buffer of length len
 * SEC1 4.1.3 step 5 aka SEC1 4.1.4 step 3
 */
int derive_mpi(const mbedtls_ecp_group *grp, mbedtls_mpi *x,
               const unsigned char *buf, size_t blen)
{
    int ret = MBEDTLS_ERR_ERROR_CORRUPTION_DETECTED;
    size_t n_size = (grp->nbits + 7) / 8;
    size_t use_size = blen > n_size ? n_size : blen;

    MBEDTLS_MPI_CHK(mbedtls_mpi_read_binary(x, buf, use_size));

    if (use_size * 8 > grp->nbits)
        MBEDTLS_MPI_CHK(mbedtls_mpi_shift_r(x, use_size * 8 - grp->nbits));

    /* While at it, reduce modulo N */
    if (mbedtls_mpi_cmp_mpi(x, &grp->N) >= 0)
        MBEDTLS_MPI_CHK(mbedtls_mpi_sub_mpi(x, x, &grp->N));

cleanup:
    return (ret);
}



/* Clean ECC reg to zero */
static void ECC_ZeroReg(unsigned int *reg)
{
    for (int i = 0; i < 18; i++)
    {
        reg[i] = 0;
    }
}

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

static void ECC_InitCurve(mbedtls_ecp_group *grp)
{
    CRYPTO_T *crpt = CRYPTO;

    /* Reset Crypto */
    SYS->CRYPTORST |= SYS_CRYPTORST_CRYPTO0RST_Msk;
    SYS->CRYPTORST = 0;
    ECC_ENABLE_INT(crpt);

    ECC_Copy((uint32_t *)crpt->ECC_A, grp->A.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->A));
    ECC_Copy((uint32_t *)crpt->ECC_B, grp->B.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->B));

    ECC_Copy((uint32_t *)crpt->ECC_X1, grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(X).MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(X)));
    ECC_Copy((uint32_t *)crpt->ECC_Y1, grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(Y).MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->MBEDTLS_PRIVATE(T)->MBEDTLS_PRIVATE(Y)));
    ECC_Copy((uint32_t *)crpt->ECC_N, grp->P.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->P));
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
        }
        else if (grp->id == MBEDTLS_ECP_DP_SECP224R1)
        {
            mbedtls_mpi_read_string(&grp->A, 16, "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFFFFFFFFFFFFFFFFFFFE");
        }
        else if (grp->id == MBEDTLS_ECP_DP_SECP256R1)
        {
            mbedtls_mpi_read_string(&grp->A, 16, "FFFFFFFF00000001000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFC");
        }
        else if (grp->id == MBEDTLS_ECP_DP_SECP384R1)
        {
            mbedtls_mpi_read_string(&grp->A, 16, "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFFFF0000000000000000FFFFFFFC");
        }
        else if (grp->id == MBEDTLS_ECP_DP_SECP521R1)
        {
            mbedtls_mpi_read_string(&grp->A, 16, "1FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC");
        }
        else if (grp->id == MBEDTLS_ECP_DP_CURVE25519)
        {
            mbedtls_mpi_read_string(&grp->A, 16, "0000000000000000000000000000000000000000000000000000000000076D06");
            mbedtls_mpi_read_string(&grp->B, 16, "0000000000000000000000000000000000000000000000000000000000000001");
            mbedtls_mpi_read_string(&grp->P, 16, "7fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffed");
            mbedtls_mpi_read_string(&grp->N, 16, "1000000000000000000000000000000014def9dea2f79cd65812631a5cf5d3ed");
        }
    }

    return 0;
}


static int run_ecc_codec(mbedtls_ecp_group *grp, uint32_t mode)
{
    CRYPTO_T *crpt = CRYPTO;
    uint32_t eccop;
    uint32_t timeout = 200000000;

    //CRYPTO_ECC_CTL_CURVEM_Pos) | ECCOP_POINT_MUL | CRYPTO_ECC_CTL_SCAP_Msk | CRYPTO_ECC_CTL_START_Msk;

    eccop = mode & CRYPTO_ECC_CTL_ECCOP_Msk;

    if (eccop == ECCOP_MODULE)
    {
        crpt->ECC_CTL = CRYPTO_ECC_CTL_FSEL_Msk;
    }
    else
    {
        /* CURVE_GF_P */
        crpt->ECC_CTL = CRYPTO_ECC_CTL_FSEL_Msk;

        if (eccop == ECCOP_POINT_MUL)
        {
            /* Enable side-channel protection in some operation */
            crpt->ECC_CTL |= CRYPTO_ECC_CTL_SCAP_Msk;
            /* If SCAP enabled, the curve order must be written to ECC_X2 */
            //Hex2Reg(pCurve->Eorder, crpt->ECC_X2);
            ECC_Copy((uint32_t *)crpt->ECC_X2, grp->N.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->N));
        }
    }

    /* Clear ECC flag */
    crpt->INTSTS = CRYPTO_INTSTS_ECCIF_Msk;

    /* Start calculation */
    crpt->ECC_CTL |= (grp->pbits << CRYPTO_ECC_CTL_CURVEM_Pos) | mode | CRYPTO_ECC_CTL_START_Msk;

    /* Waiting for calculation */
    while ((crpt->INTSTS & CRYPTO_INTSTS_ECCIF_Msk) == 0)
    {
        if (timeout-- <= 0)
            return MBEDTLS_ERR_ECP_HW_ACCEL_FAILED;
    }

    return 0;
}
#endif /* MBEDTLS_ECDSA_SIGN_ALT || MBEDTLS_ECDSA_VERIFY_ALT */

#if defined(MBEDTLS_ECDSA_SIGN_ALT)


int mbedtls_ecdsa_can_do(mbedtls_ecp_group_id gid)
{
    switch (gid)
    {
#ifdef MBEDTLS_ECP_DP_CURVE25519_ENABLED

        case MBEDTLS_ECP_DP_CURVE25519:
            return 0;
#endif
#ifdef MBEDTLS_ECP_DP_CURVE448_ENABLED

        case MBEDTLS_ECP_DP_CURVE448:
            return 0;
#endif

        default:
            return 1;
    }
}


int32_t  ECC_Sign(mbedtls_ecp_group *grp, mbedtls_mpi *r, mbedtls_mpi *s,
                  const mbedtls_mpi *d, const unsigned char *buf, size_t blen,
                  int (*f_rng)(void *, unsigned char *, size_t), void *p_rng)
{
    uint32_t volatile temp_result1[18], temp_result2[18];
    CRYPTO_T *crpt = CRYPTO;
    size_t len, nblimbs;
    mbedtls_mpi e;
    mbedtls_mpi k, *pk;


    /* Fail cleanly on curves such as Curve25519 that can't be used for ECDSA */
    if (!mbedtls_ecdsa_can_do(grp->id) || grp->N.MBEDTLS_PRIVATE(p) == NULL)
        return (MBEDTLS_ERR_ECP_BAD_INPUT_DATA);

    /* Make sure d is in range 1..n-1 */
    if (mbedtls_mpi_cmp_int(d, 1) < 0 || mbedtls_mpi_cmp_mpi(d, &grp->N) >= 0)
        return (MBEDTLS_ERR_ECP_INVALID_KEY);


    pk = &k;
    mbedtls_mpi_init(pk);

    ECC_FixCurve(grp);

    /* Generate a random k. It will use CRYPTO SHA*/
    mbedtls_ecp_gen_privkey(grp, pk, f_rng, p_rng);

    /* Reset Crypto */
    SYS->CRYPTORST |= SYS_CRYPTORST_CRYPTO0RST_Msk;
    SYS->CRYPTORST = 0;

    ECC_InitCurve(grp);

    /*
        *   1. Calculate e = HASH(m), where HASH is a cryptographic hashing algorithm, (i.e. SHA-1)
        *      (1) Use SHA to calculate e
        */

    /*   2. Select a random integer k form [1, n-1]
    *      (1) Notice that n is order, not prime modulus or irreducible polynomial function
    */

    /*
    *   3. Compute r = x1 (mod n), where (x1, y1) = k * G. If r = 0, go to step 2
    *      (1) Write the curve parameter A, B, and curve length M to corresponding registers
    *      (2) Write the prime modulus or irreducible polynomial function to N registers according
    *      (3) Write the point G(x, y) to X1, Y1 registers
    *      (4) Write the random integer k to K register
    *      (5) Set ECCOP(CRPT_ECC_CTL[10:9]) to 00
    *      (6) Set FSEL(CRPT_ECC_CTL[8]) according to used curve of prime field or binary field
    *      (7) Set START(CRPT_ECC_CTL[0]) to 1
    *      (8) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
    *      (9) Write the curve order and curve length to N ,M registers according
    *      (10) Write 0x0 to Y1 registers
    *      (11) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
    *      (12) Set MOPOP(CRPT_ECC_CTL[12:11]) to 10
    *      (13) Set START(CRPT_ECC_CTL[0]) to 1         *
    *      (14) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
    *      (15) Read X1 registers to get r
    */

    /* 3-(4) Write the random integer k to K register */
    ECC_ZeroReg((uint32_t *)crpt->ECC_K);
    ECC_Copy((uint32_t *)crpt->ECC_K, pk->MBEDTLS_PRIVATE(p), mbedtls_mpi_size(pk));

    run_ecc_codec(grp, ECCOP_POINT_MUL);

    /*  3-(9) Write the curve order to N registers */
    //Hex2Reg(pCurve->Eorder, crpt->ECC_N);
    ECC_ZeroReg((uint32_t *)crpt->ECC_N);
    ECC_Copy((uint32_t *)crpt->ECC_N, grp->N.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->N));

    /* 3-(10) Write 0x0 to Y1 registers */
    ECC_ZeroReg((uint32_t *)crpt->ECC_Y1);

    run_ecc_codec(grp, ECCOP_MODULE | MODOP_ADD);

    /* 3-(15) Read X1 registers to get r */
    len = grp->pbits / 8 + ((grp->pbits & 0x7) != 0);
    nblimbs = len / 4 + ((len & 0x3) != 0);
    mbedtls_mpi_grow(r, nblimbs);
    mbedtls_mpi_grow(s, nblimbs);
    ECC_Copy(r->MBEDTLS_PRIVATE(p), (uint32_t *)crpt->ECC_X1, len);
    ECC_Copy((uint32_t *)temp_result1, (uint32_t *)crpt->ECC_X1, len);

    /*
        *   4. Compute s = k^-1 * (e + d * r)(mod n). If s = 0, go to step 2
        *      (1) Write the curve order to N registers according
        *      (2) Write 0x1 to Y1 registers
        *      (3) Write the random integer k to X1 registers according
        *      (4) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
        *      (5) Set MOPOP(CRPT_ECC_CTL[12:11]) to 00
        *      (6) Set START(CRPT_ECC_CTL[0]) to 1
        *      (7) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
        *      (8) Read X1 registers to get k^-1
        *      (9) Write the curve order and curve length to N ,M registers
        *      (10) Write r, d to X1, Y1 registers
        *      (11) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
        *      (12) Set MOPOP(CRPT_ECC_CTL[12:11]) to 01
        *      (13) Set START(CRPT_ECC_CTL[0]) to 1
        *      (14) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
        *      (15) Write the curve order to N registers
        *      (16) Write e to Y1 registers
        *      (17) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
        *      (18) Set MOPOP(CRPT_ECC_CTL[12:11]) to 10
        *      (19) Set START(CRPT_ECC_CTL[0]) to 1
        *      (20) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
        *      (21) Write the curve order and curve length to N ,M registers
        *      (22) Write k^-1 to Y1 registers
        *      (23) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
        *      (24) Set MOPOP(CRPT_ECC_CTL[12:11]) to 01
        *      (25) Set START(CRPT_ECC_CTL[0]) to 1
        *      (26) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
        *      (27) Read X1 registers to get s
        */

    /* S/W: GFp_add_mod_order(pCurve->key_len+2, 0, x1, a, R); */

    /*  4-(1) Write the curve order to N registers */
    //Hex2Reg(pCurve->Eorder, crpt->ECC_N);
    ECC_ZeroReg((uint32_t *)crpt->ECC_N);
    ECC_Copy((uint32_t *)crpt->ECC_N, grp->N.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->N));

    /*  4-(2) Write 0x1 to Y1 registers */
    ECC_ZeroReg((uint32_t *)crpt->ECC_Y1);
    crpt->ECC_Y1[0] = 0x1UL;

    /*  4-(3) Write the random integer k to X1 registers */
    ECC_ZeroReg((uint32_t *)crpt->ECC_X1);
    ECC_Copy((uint32_t *)crpt->ECC_X1, pk->MBEDTLS_PRIVATE(p), mbedtls_mpi_size(pk));


    run_ecc_codec(grp, ECCOP_MODULE | MODOP_DIV);

    /*  4-(8) Read X1 registers to get k^-1 */
    ECC_Copy((uint32_t *)temp_result2, (uint32_t *)crpt->ECC_X1, 72);


    /*  4-(9) Write the curve order and curve length to N ,M registers */
    ECC_ZeroReg((uint32_t *)crpt->ECC_N);
    ECC_Copy((uint32_t *)crpt->ECC_N, grp->N.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->N));

    /*  4-(10) Write r, d to X1, Y1 registers */
    ECC_Copy((uint32_t *)crpt->ECC_X1, (uint32_t *)temp_result1, 72);

    ECC_ZeroReg((uint32_t *)crpt->ECC_Y1);
    ECC_Copy((uint32_t *)crpt->ECC_Y1, d->MBEDTLS_PRIVATE(p), mbedtls_mpi_size(d));

    run_ecc_codec(grp, ECCOP_MODULE | MODOP_MUL);

    /*  4-(15) Write the curve order to N registers */
    ECC_ZeroReg((uint32_t *)crpt->ECC_N);
    ECC_Copy((uint32_t *)crpt->ECC_N, grp->N.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->N));


    /*  4-(16) Write e to Y1 registers */
    ECC_ZeroReg((uint32_t *)crpt->ECC_Y1);
    mbedtls_mpi_init(&e);
    derive_mpi(grp, &e, buf, blen);

    ECC_Copy((uint32_t *)crpt->ECC_Y1, e.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&e));
    mbedtls_mpi_free(&e);

    run_ecc_codec(grp, ECCOP_MODULE | MODOP_ADD);


    /*  4-(21) Write the curve order and curve length to N ,M registers */
    ECC_ZeroReg((uint32_t *)crpt->ECC_N);
    ECC_Copy((uint32_t *)crpt->ECC_N, grp->N.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->N));

    /*  4-(22) Write k^-1 to Y1 registers */
    ECC_Copy((uint32_t *)crpt->ECC_Y1, (uint32_t *)temp_result2, 72);


    run_ecc_codec(grp, ECCOP_MODULE | MODOP_MUL);

    /*  4-(27) Read X1 registers to get s */
    ECC_Copy(s->MBEDTLS_PRIVATE(p), (uint32_t *)crpt->ECC_X1, len);


    mbedtls_mpi_free(pk);


    return 0;
}


/*
 * Compute ECDSA signature of a hashed message
 */
int mbedtls_ecdsa_sign(mbedtls_ecp_group *grp, mbedtls_mpi *r, mbedtls_mpi *s,
                       const mbedtls_mpi *d, const unsigned char *buf, size_t blen,
                       int (*f_rng)(void *, unsigned char *, size_t), void *p_rng)
{
    int ret;

    ECDSA_VALIDATE_RET(grp   != NULL);
    ECDSA_VALIDATE_RET(r     != NULL);
    ECDSA_VALIDATE_RET(s     != NULL);
    ECDSA_VALIDATE_RET(d     != NULL);
    ECDSA_VALIDATE_RET(f_rng != NULL);
    ECDSA_VALIDATE_RET(buf   != NULL || blen == 0);

    ret = ECC_Sign(grp, r, s, d, buf, blen, f_rng, p_rng);

    return ret;

}
#endif /* MBEDTLS_ECDSA_SIGN_ALT */

#if defined(MBEDTLS_ECDSA_VERIFY_ALT)

int  ECC_Verify(mbedtls_ecp_group *grp,
                const unsigned char *buf, size_t blen,
                const mbedtls_ecp_point *Q,
                const mbedtls_mpi *r,
                const mbedtls_mpi *s)
{
    CRYPTO_T *crpt;
    uint32_t  temp_result1[18], temp_result2[18];
    uint32_t  temp_x[18], temp_y[18];
    int32_t   i, ret = 0;
    mbedtls_mpi e;
    int32_t u1_zero_flag = 0;

    /* Reset Crypto */
    SYS->CRYPTORST |= SYS_CRYPTORST_CRYPTO0RST_Msk;
    SYS->CRYPTORST = 0;

    crpt = CRYPTO;
    ECC_FixCurve(grp);
    ECC_InitCurve(grp);

    /*
     *   1. Verify that r and s are integers in the interval [1, n-1]. If not, the signature is invalid
     *   2. Compute e = HASH (m), where HASH is the hashing algorithm in signature generation
     *      (1) Use SHA to calculate e
     */

    /*
     *   3. Compute w = s^-1 (mod n)
     *      (1) Write the curve order to N registers
     *      (2) Write 0x1 to Y1 registers
     *      (3) Write s to X1 registers
     *      (4) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
     *      (5) Set MOPOP(CRPT_ECC_CTL[12:11]) to 00
     *      (6) Set FSEL(CRPT_ECC_CTL[8]) according to used curve of prime field or binary field
     *      (7) Set START(CRPT_ECC_CTL[0]) to 1
     *      (8) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
     *      (9) Read X1 registers to get w
     */

    /*  3-(1) Write the curve order to N registers */
    ECC_Copy((uint32_t *)crpt->ECC_N, grp->N.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->N));

    /*  3-(2) Write 0x1 to Y1 registers */
    ECC_ZeroReg((uint32_t *)crpt->ECC_Y1);
    crpt->ECC_Y1[0] = 0x1UL;

    /*  3-(3) Write s to X1 registers */
    ECC_Copy((uint32_t *)crpt->ECC_X1, s->MBEDTLS_PRIVATE(p), mbedtls_mpi_size(s));

    if ((ret = run_ecc_codec(grp, ECCOP_MODULE | MODOP_DIV)))
    {
        return ret;
    }

    /*  3-(9) Read X1 registers to get w */
    ECC_Copy((uint32_t *)temp_result2, (uint32_t *)crpt->ECC_X1, 72);



    /*
        *   4. Compute u1 = e * w (mod n) and u2 = r * w (mod n)
        *      (1) Write the curve order and curve length to N ,M registers
        *      (2) Write e, w to X1, Y1 registers
        *      (3) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
        *      (4) Set MOPOP(CRPT_ECC_CTL[12:11]) to 01
        *      (5) Set START(CRPT_ECC_CTL[0]) to 1
        *      (6) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
        *      (7) Read X1 registers to get u1
        *      (8) Write the curve order and curve length to N ,M registers
        *      (9) Write r, w to X1, Y1 registers
        *      (10) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
        *      (11) Set MOPOP(CRPT_ECC_CTL[12:11]) to 01
        *      (12) Set START(CRPT_ECC_CTL[0]) to 1
        *      (13) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
        *      (14) Read X1 registers to get u2
        */

    /*  4-(1) Write the curve order and curve length to N ,M registers */
    ECC_ZeroReg((uint32_t *)crpt->ECC_N);

    //Hex2Reg(pCurve->Eorder, crpt->ECC_N);
    ECC_Copy((uint32_t *)crpt->ECC_N, grp->N.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->N));

    /* 4-(2) Write e, w to X1, Y1 registers */
    ECC_ZeroReg((uint32_t *)crpt->ECC_X1);

    mbedtls_mpi_init(&e);
    derive_mpi(grp, &e, buf, blen);

    ECC_Copy((uint32_t *)crpt->ECC_X1, e.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&e));
    mbedtls_mpi_free(&e);

    ECC_Copy((uint32_t *)crpt->ECC_Y1, temp_result2, 72);

    if ((ret = run_ecc_codec(grp, ECCOP_MODULE | MODOP_MUL)))
    {
        return ret;
    }

    /*  4-(7) Read X1 registers to get u1 */
    ECC_Copy((uint32_t *)temp_result1, (uint32_t *)crpt->ECC_X1, 72);

    /*  4-(8) Write the curve order and curve length to N ,M registers */
    ECC_ZeroReg((uint32_t *)crpt->ECC_N);

    ECC_Copy((uint32_t *)crpt->ECC_N, grp->N.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->N));

    /* 4-(9) Write r, w to X1, Y1 registers */
    ECC_ZeroReg((uint32_t *)crpt->ECC_X1);
    ECC_Copy((uint32_t *)crpt->ECC_X1, r->MBEDTLS_PRIVATE(p), mbedtls_mpi_size(r));

    ECC_Copy((uint32_t *)crpt->ECC_Y1, temp_result2, 72);

    if ((ret = run_ecc_codec(grp, ECCOP_MODULE | MODOP_MUL)))
    {
        return ret;
    }

    /*  4-(14) Read X1 registers to get u2 */
    ECC_Copy((uint32_t *)temp_result2, (uint32_t *)crpt->ECC_X1, 72);

    /*
        *   5. Compute X * (x1', y1') = u1 * G + u2 * Q
        *      (1) Write the curve parameter A, B, N, and curve length M to corresponding registers
        *      (2) Write the point G(x, y) to X1, Y1 registers
        *      (3) Write u1 to K registers
        *      (4) Set ECCOP(CRPT_ECC_CTL[10:9]) to 00
        *      (5) Set START(CRPT_ECC_CTL[0]) to 1
        *      (6) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
        *      (7) Read X1, Y1 registers to get u1*G
        *      (8) Write the curve parameter A, B, N, and curve length M to corresponding registers
        *      (9) Write the public key Q(x,y) to X1, Y1 registers
        *      (10) Write u2 to K registers
        *      (11) Set ECCOP(CRPT_ECC_CTL[10:9]) to 00
        *      (12) Set START(CRPT_ECC_CTL[0]) to 1
        *      (13) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
        *      (14) Write the curve parameter A, B, N, and curve length M to corresponding registers
        *      (15) Write the result data u1*G to X2, Y2 registers
        *      (16) Set ECCOP(CRPT_ECC_CTL[10:9]) to 10
        *      (17) Set START(CRPT_ECC_CTL[0]) to 1
        *      (18) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
        *      (19) Read X1, Y1 registers to get X *(x1', y1')
        *      (20) Write the curve order and curve length to N ,M registers
        *      (21) Write x1 * to X1 registers
        *      (22) Write 0x0 to Y1 registers
        *      (23) Set ECCOP(CRPT_ECC_CTL[10:9]) to 01
        *      (24) Set MOPOP(CRPT_ECC_CTL[12:11]) to 10
        *      (25) Set START(CRPT_ECC_CTL[0]) to 1
        *      (26) Wait for BUSY(CRPT_ECC_STS[0]) be cleared
        *      (27) Read X1 registers to get x1 * (mod n)
        *
        *   6. The signature is valid if x1 * = r, otherwise it is invalid
        */

    /*
    *  (1) Write the curve parameter A, B, N, and curve length M to corresponding registers
    *  (2) Write the point G(x, y) to X1, Y1 registers
    */
    ECC_InitCurve(grp);

    // Skip u1 if e = 0
    for (i = 0; i < 18; i++)
    {
        if (temp_result1[i])
            break;
    }

    // if e != 0
    if (i != 18)
    {
        /* (3) Write u1 to K registers */
        ECC_Copy((uint32_t *)crpt->ECC_K, temp_result1, 72);

        if ((ret = run_ecc_codec(grp, ECCOP_POINT_MUL)))
        {
            return ret;
        }

        /* (7) Read X1, Y1 registers to get u1*G */
        ECC_Copy(temp_x, (uint32_t *)crpt->ECC_X1, 72);
        ECC_Copy(temp_y, (uint32_t *)crpt->ECC_Y1, 72);
    }
    else
    {
        /* u1 is zero */
        u1_zero_flag = 1;
    }

    /* (8) Write the curve parameter A, B, N, and curve length M to corresponding registers */
    ECC_InitCurve(grp);

    /* (9) Write the public key Q(x,y) to X1, Y1 registers */
    ECC_Copy((uint32_t *)crpt->ECC_X1, Q->MBEDTLS_PRIVATE(X).MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&Q->MBEDTLS_PRIVATE(X)));
    ECC_Copy((uint32_t *)crpt->ECC_Y1, Q->MBEDTLS_PRIVATE(Y).MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&Q->MBEDTLS_PRIVATE(Y)));


    /* (10) Write u2 to K registers */
    ECC_Copy((uint32_t *)crpt->ECC_K, (uint32_t *)temp_result2, 72);

    if ((ret = run_ecc_codec(grp, ECCOP_POINT_MUL)))
    {
        return ret;
    }

    ECC_Copy((uint32_t *)temp_result1, (uint32_t *)crpt->ECC_X1, 72);
    ECC_Copy((uint32_t *)temp_result2, (uint32_t *)crpt->ECC_Y1, 72);

    if (u1_zero_flag == 0)
    {
        /* (14) Write the curve parameter A, B, N, and curve length M to corresponding registers */
        ECC_InitCurve(grp);

        /* Write the result data u2*Q to X1, Y1 registers */
        ECC_Copy((uint32_t *)crpt->ECC_X1, (uint32_t *)temp_result1, 72);
        ECC_Copy((uint32_t *)crpt->ECC_Y1, (uint32_t *)temp_result2, 72);

        /* (15) Write the result data u1*G to X2, Y2 registers */
        ECC_Copy((uint32_t *)crpt->ECC_X2, (uint32_t *)temp_x, 72);
        ECC_Copy((uint32_t *)crpt->ECC_Y2, (uint32_t *)temp_y, 72);

        if ((ret = run_ecc_codec(grp, ECCOP_POINT_ADD)))
        {
            return ret;
        }

        /* (19) Read X1, Y1 registers to get X' (x1', y1') */
        ECC_Copy(temp_x, (uint32_t *)crpt->ECC_X1, 72);
        ECC_Copy(temp_y, (uint32_t *)crpt->ECC_Y1, 72);
    }
    else
    {
        ECC_Copy(temp_x, temp_result1, 72);
        ECC_Copy(temp_y, temp_result2, 72);
    }

    /*  (20) Write the curve order and curve length to N ,M registers */
    for (i = 0; i < 18; i++)
    {
        crpt->ECC_N[i] = 0UL;
    }

    ECC_ZeroReg((uint32_t *)crpt->ECC_N);

    ECC_Copy((uint32_t *)crpt->ECC_N, grp->P.MBEDTLS_PRIVATE(p), mbedtls_mpi_size(&grp->P));

    /*
        *  (21) Write x1 * to X1 registers
        *  (22) Write 0x0 to Y1 registers
        */
    ECC_Copy((uint32_t *)crpt->ECC_X1, temp_x, 72);
    ECC_ZeroReg((uint32_t *)crpt->ECC_Y1);

    if ((ret = run_ecc_codec(grp, ECCOP_MODULE | MODOP_ADD)))
    {
        return ret;
    }

    /*  (27) Read X1 registers to get x1 * (mod n) */

    /* 6. The signature is valid if x1 * = r, otherwise it is invalid */
    if (memcmp((uint8_t *)crpt->ECC_X1, (uint8_t *)r->MBEDTLS_PRIVATE(p), mbedtls_mpi_size(r)) != 0)
    {
        return MBEDTLS_ERR_ECP_VERIFY_FAILED;
    }


    return 0;
}

/*
 * Verify ECDSA signature of hashed message
 */
int mbedtls_ecdsa_verify(mbedtls_ecp_group *grp,
                         const unsigned char *buf, size_t blen,
                         const mbedtls_ecp_point *Q,
                         const mbedtls_mpi *r,
                         const mbedtls_mpi *s)
{
    int ret;

    ECDSA_VALIDATE_RET(grp != NULL);
    ECDSA_VALIDATE_RET(Q   != NULL);
    ECDSA_VALIDATE_RET(r   != NULL);
    ECDSA_VALIDATE_RET(s   != NULL);
    ECDSA_VALIDATE_RET(buf != NULL || blen == 0);

    ret = ECC_Verify(grp, buf, blen, Q, r, s);

    return (ret);
}
#endif /* MBEDTLS_ECDSA_VERIFY_ALT */
#endif /* MBEDTLS_ECDSA_C */
