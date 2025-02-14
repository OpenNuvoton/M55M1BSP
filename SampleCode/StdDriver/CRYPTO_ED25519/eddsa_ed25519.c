/*************************************************************************//**
 * @file     eddsa_ed25519.c
 * @version  V1.00
 * @brief    ED25519 core code for M55M1 series MCU
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "EdDsa.h"
#define SIGN_METHOD_1
#define VERIFY_METHOD_1

#define ECC_REG_BASE    CRYPTO_BASE

#define REG_HMAC_DGST   (ECC_REG_BASE + 0x308)
#define REG_ECC_X1      (ECC_REG_BASE + 0x808)
#define REG_ECC_Y1      (ECC_REG_BASE + 0x850)
#define REG_ECC_X2      (ECC_REG_BASE + 0x898)
#define REG_ECC_Y2      (ECC_REG_BASE + 0x8E0)
#define REG_ECC_A       (ECC_REG_BASE + 0x928)
#define REG_ECC_B       (ECC_REG_BASE + 0x970)
#define REG_ECC_N       (ECC_REG_BASE + 0x9B8)
#define REG_ECC_K       (ECC_REG_BASE + 0xA00)


#define ECC_SCAP_CONFIG     0   // (CRYPTO_ECC_CTL_SCAP_Msk | CRYPTO_ECC_CTL_ASCAP_Msk | CRYPTO_ECC_CTL_PFA2C_Msk)


#define swap32(x) (((x << 24) & 0xff000000) | ((x << 8) & 0xff0000) | ((x >> 8) & 0xff00) | ((x >> 24) & 0xff))




extern volatile uint32_t g_ECC_done, g_ECCERR_done;
static uint8_t  g_dma_buff[0x10000] __attribute__((aligned(32)));

/* Twisted Edwards curve-specific parameters */
struct eddsa_param
{
    uint32_t    x1[18];   /* Gx */
    uint32_t    y1[18];   /* Gy */
    uint32_t    x2[18];
    uint32_t    y2[18];
    uint32_t    a[18];    /* a */
    uint32_t    b[18];    /* d */
    uint32_t    n[18];    /* p */
    uint32_t    k[18];
};

static const struct eddsa_param ed25519_curve =
{
    /* Gx */{
        0x8F25D51A, 0xC9562D60, 0x9525A7B2, 0x692CC760, 0xFDD6DC5C, 0xC0A4E231, 0xCD6E53FE, 0x216936D3, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
    },
    /* Gy */{
        0x66666658, 0x66666666, 0x66666666, 0x66666666, 0x66666666, 0x66666666, 0x66666666, 0x66666666, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
    },
    /* x2 */{
        0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
    },
    /* y2 */{
        0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
    },
    /* a */ {
        0xFFFFFFEC, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x7FFFFFFF, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
    },
    /* b */ {
        0x135978A3, 0x75EB4DCA, 0x4141D8AB, 0x00700A4D, 0x7779E898, 0x8CC74079, 0x2B6FFE73, 0x52036CEE, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
    },
    /* n */ {
        0xFFFFFFED, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x7FFFFFFF, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
    },
    /* K */ {
        0x5a5aa5a5, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
    }
};


static char ed25519ctx_str[] = "SigEd25519 no Ed25519 collisions";


static uint32_t __ed25519_N[8] = { 0x5CF5D3ED, 0x5812631A, 0xA2F79CD6, 0x14DEF9DE,
                                   0x00000000, 0x00000000, 0x00000000, 0x10000000
                                 };


static void write_reg_array(uint32_t reg_base, uint32_t *data, int count, int max)
{
    uint32_t  *reg = u32_to_ptr32(reg_base);

    int i;

    for (i = 0; i < count; i++)
    {
        reg[i] = data[i];
    }

    for (i = count; i < max; i++)
    {
        reg[i] = 0;
    }
}

static void read_reg_array(uint32_t reg_base, uint32_t *data, int count, int max)
{
    uint32_t  *reg = u32_to_ptr32(reg_base);

    int i;

    for (i = 0; i < count; i++)
        data[i] = reg[i];

    for (i = count; i < max; i++)
        data[i] = 0;
}

static void clear_reg_array(uint32_t reg_base, int count)
{
    uint32_t  *reg = u32_to_ptr32(reg_base);

    int i;

    for (i = 0; i < count; i++)
        reg[i] = 0;
}

static int compare_word_array(uint32_t *aw1, uint32_t *aw2, int count)
{
    int i;

    for (i = 0; i < count; i++)
    {
        if (aw1[i] != aw2[i])
            return -1;
    }

    return 0;
}

static void dump_word_array(uint32_t *data, int count)
{
    int i;

    for (i = 0; i < count; i++)
        printf("%08x ", data[i]);
}

static void set_ed25519_param(void *addr)
{
    write_reg_array(REG_ECC_X1, (uint32_t *)(ed25519_curve.x1), 18, 18);
    write_reg_array(REG_ECC_Y1, (uint32_t *)(ed25519_curve.y1), 18, 18);
    write_reg_array(REG_ECC_X2, (uint32_t *)(ed25519_curve.x2), 18, 18);
    write_reg_array(REG_ECC_Y2, (uint32_t *)(ed25519_curve.y2), 18, 18);
    write_reg_array(REG_ECC_A, (uint32_t *)(ed25519_curve.a), 18, 18);
    write_reg_array(REG_ECC_B, (uint32_t *)(ed25519_curve.b), 18, 18);
    write_reg_array(REG_ECC_N, (uint32_t *)(ed25519_curve.n), 18, 18);
    write_reg_array(REG_ECC_K, (uint32_t *)(ed25519_curve.k), 18, 18);

}

static void set_ed25519_N(void *addr)
{
    if (addr == NULL)
        write_reg_array(REG_ECC_N, __ed25519_N, 8, 18);
    else
        write_reg_array(ptr_to_u32(addr), __ed25519_N, 8, 18);
}

static void ECC_Start_And_Wait(uint32_t ecc_ctl)
{
    g_ECC_done = g_ECCERR_done = 0UL;
    printf("ECC_Start_And_Wait - 0x%x\n", ecc_ctl);
    CRYPTO->ECC_CTL = ecc_ctl | ECC_SCAP_CONFIG;

    while ((g_ECC_done == 0UL) && (g_ECCERR_done == 0UL))
    {
        // printf("ECC_start - CTL = 0x%x, STS = 0x%x\n",  CRYPTO->ECC_CTL, CRYPTO->ECC_STS);
    }
}

static void ECC_KP(void)  //ECC point multiplication
{
    uint32_t ecc_ctl;

    printf("%s, %d\n", __func__, __LINE__);
    ecc_ctl = (255 << CRYPTO_ECC_CTL_CURVEM_Pos) | (0x0 << CRYPTO_ECC_CTL_ECCOP_Pos);
    ecc_ctl |= CRYPTO_ECC_CTL_SPCEN_Msk;
    ecc_ctl |= CRYPTO_ECC_CTL_SPCSEL_Msk | CRYPTO_ECC_CTL_FSEL_Msk | CRYPTO_ECC_CTL_START_Msk;
    ecc_ctl |= ECC_SCAP_CONFIG;
    printf("%s, %d\n", __func__, __LINE__);
    ECC_Start_And_Wait(ecc_ctl);
    printf("%s, %d\n", __func__, __LINE__);
}

static void ECC_PA(void) //ECC point addition
{
    uint32_t ecc_ctl;

    ecc_ctl = (255 << CRYPTO_ECC_CTL_CURVEM_Pos) | (0x2 << CRYPTO_ECC_CTL_ECCOP_Pos);
    ecc_ctl |= CRYPTO_ECC_CTL_SPCEN_Msk;
    ecc_ctl |= CRYPTO_ECC_CTL_SPCSEL_Msk | CRYPTO_ECC_CTL_FSEL_Msk | CRYPTO_ECC_CTL_START_Msk;
    ecc_ctl |= ECC_SCAP_CONFIG;
    ECC_Start_And_Wait(ecc_ctl);
}

//op=0 : (y1 / x1) % n; op=1 : (x1 * y1) % n; op=2 : (x1 + y1) % n; op=3 : (x1 - y1) % n
static void ECC_MODOP(uint32_t *x1, uint32_t *y1,
                      uint32_t *n, uint32_t op)   //ECC modular operation
{

    uint32_t ecc_ctl;

    // point x1
    write_reg_array(REG_ECC_X1, x1, 8, 18);

    // point y1
    if (op == 5)
        write_reg_array(REG_ECC_K, y1, 8, 18);
    else
        write_reg_array(REG_ECC_Y1, y1, 8, 18);

    // point n
    write_reg_array(REG_ECC_N, n, 8, 18);

    ecc_ctl = (255 << CRYPTO_ECC_CTL_CURVEM_Pos) | (0x1 << CRYPTO_ECC_CTL_ECCOP_Pos);
    ecc_ctl |= CRYPTO_ECC_CTL_SPCEN_Msk;
    ecc_ctl |= CRYPTO_ECC_CTL_SPCSEL_Msk | CRYPTO_ECC_CTL_FSEL_Msk | CRYPTO_ECC_CTL_START_Msk;
    ecc_ctl |= ECC_SCAP_CONFIG;

    if (op < 4)
        ecc_ctl |= (op << CRYPTO_ECC_CTL_MODOP_Pos);
    else
        ecc_ctl |= ((op % 4) << CRYPTO_ECC_CTL_MODOP_Pos) | CRYPTO_ECC_CTL_SMOD_Msk;

    ECC_Start_And_Wait(ecc_ctl);
}

static void do_SHA512(uint8_t *data, int data_len, int swap)
{
    uint32_t hmac_ctl = 0;

    if (swap)
        hmac_ctl |= CRYPTO_HMAC_CTL_INSWAP_Msk;

    hmac_ctl = hmac_ctl | CRYPTO_HMAC_CTL_OUTSWAP_Msk |
               (SHA_MODE_SHA512 << CRYPTO_HMAC_CTL_OPMODE_Pos) |
               CRYPTO_HMAC_CTL_DMAEN_Msk | CRYPTO_HMAC_CTL_DMALAST_Msk |
               CRYPTO_HMAC_CTL_START_Msk;
    CRYPTO->HMAC_KEYCNT = 0;
    CRYPTO->HMAC_DMACNT = data_len;
#if (NVT_DCACHE_ON == 1)
    SCB_CleanDCache_by_Addr(data, data_len);
#endif
    CRYPTO->HMAC_SADDR = ptr_to_u32(data);
    CRYPTO->HMAC_CTL = hmac_ctl;
    __ISB();

    while (CRYPTO->HMAC_STS & CRYPTO_HMAC_STS_BUSY_Msk) ;

    //printf("do_SHA512 output: ");
    //dump_word_array((uint32_t *)REG_HMAC_DGST, 16);
}

static void Decoding(uint32_t data[8], int size,
                     uint32_t Decoding_X[8], uint32_t Decoding_Y[8])
{
    int i;
    int case1 = 0, case2 = 0;
    uint32_t x_0;
    uint32_t U[8];
    uint32_t V[8];
    uint32_t X1[8], X2[8], X3[8];
    uint32_t Ed_P[8], Ed_A[8], Ed_D[8], Ed_1[8];

    // Edwards-curve formula : A * x^2 + y^2 = 1 + D * X^2 * Y^2 in GF(P)
    // initial A, D, P and 0x1
    memcpy(Ed_P, ed25519_curve.n, 8 * 4);
    memcpy(Ed_A, ed25519_curve.a, 8 * 4);
    memcpy(Ed_D, ed25519_curve.b, 8 * 4);
    Ed_1[0] = 0x1;

    for (i = 1; i < 8; i++) Ed_1[i] = 0;

    /*------------------------------------------------------------------------*/
    /*  [Step1.1] y                                                           */
    /*------------------------------------------------------------------------*/
    for (i = 0; i < size - 1; i++)
        Decoding_Y[i] = data[i];

    Decoding_Y[7] = data[7] & 0x7FFFFFFF;

    /*------------------------------------------------------------------------*/
    /*  [Step1.2] x_0                                                         */
    /*------------------------------------------------------------------------*/
    if ((data[7] & 0x80000000) == 0x80000000)
        x_0 = 0x1;
    else
        x_0 = 0x0;

    // printf("## y and x_0 is done ##");

    /*------------------------------------------------------------------------*/
    /*  U = y^2 - 1 and V = d * y^2 - a                                       */
    /*                                                                        */
    /*  [Step2.1~9] U = y^2                                                   */
    /*------------------------------------------------------------------------*/
    ECC_MODOP(Decoding_Y, Decoding_Y, Ed_P, 1);

    for (i = 0; i < 8; i++)
        U[i] = CRYPTO->ECC_X1[i];

    /*------------------------------------------------------------------------*/
    /*  [Step3.1~9] V = d * y^2                                               */
    /*------------------------------------------------------------------------*/
    ECC_MODOP(U, Ed_D, Ed_P, 1);

    for (i = 0; i < 8; i++)
        V[i] = CRYPTO->ECC_X1[i];

    /*------------------------------------------------------------------------*/
    /*  [Step3.10~17] V = d * y^2 - a                                         */
    /*------------------------------------------------------------------------*/
    ECC_MODOP(V, Ed_A, Ed_P, 3);

    for (i = 0; i < 8; i++)
        V[i] = CRYPTO->ECC_X1[i];

    /*------------------------------------------------------------------------*/
    /*  [Step2.10~17] U = y^2 -1                                              */
    /*------------------------------------------------------------------------*/
    ECC_MODOP(U, Ed_1, Ed_P, 3);

    for (i = 0; i < 8; i++)
        U[i] = CRYPTO->ECC_X1[i];

    /*------------------------------------------------------------------------*/
    /*  [Step4.1~8] X2 = u/v                                                  */
    /*------------------------------------------------------------------------*/
    ECC_MODOP(V, U, Ed_P, 0);

    for (i = 0; i < 8; i++)
        X2[i] = CRYPTO->ECC_X1[i];

    printf("uuuu/vvvv: ");
    dump_word_array(X2, 8);

    // printf("## M^E H algorith start : ##");
    /*------------------------------------------------------------------------*/
    /*  [Step5.1~9] U = ((p+3)/8) is a fixed value. Thus, this pattern        */
    /*              does not execute Step5.1~9                                */
    /*------------------------------------------------------------------------*/
    U[0] = 0xfffffffe;
    U[1] = 0xffffffff;
    U[2] = 0xffffffff;
    U[3] = 0xffffffff;
    U[4] = 0xffffffff;
    U[5] = 0xffffffff;
    U[6] = 0xffffffff;
    U[7] = 0x0fffffff;

    /*------------------------------------------------------------------------*/
    /*  [Step5.10~18] X1=(X2)^((p+3)/8)=(X2)^(U)                              */
    /*------------------------------------------------------------------------*/
    ECC_MODOP(X2, U, Ed_P, 5);
    read_reg_array(REG_ECC_X1, X1, 8, 8);

    /*------------------------------------------------------------------------*/
    /*  [Step6.1~9] x3 = x1^2                                                 */
    /*------------------------------------------------------------------------*/
    ECC_MODOP(X1, X1, Ed_P, 1);
    read_reg_array(REG_ECC_X1, X3, 8, 8);

    /*------------------------------------------------------------------------*/
    /*  [Step6.10~17] case 1 = (x1^2) - x2 = U                                */
    /*------------------------------------------------------------------------*/
    ECC_MODOP(X3, X2, Ed_P, 3);
    read_reg_array(REG_ECC_X1, U, 8, 8);

    /*------------------------------------------------------------------------*/
    /*  [Step7.1~8] case 2 = (x1^2) + x2 = V                                  */
    /*------------------------------------------------------------------------*/
    ECC_MODOP(X3, X2, Ed_P, 2);
    read_reg_array(REG_ECC_X1, V, 8, 8);

    // case1 == 0 ?
    for (i = 0; i < 8; i++)
    {
        if (U[i] != 0x0)
            case1 = 1;
    }

    // case2 == 0 ?
    for (i = 0; i < 8; i++)
    {
        if (V[i] != 0x0)
            case2 = 1;
    }

    /*------------------------------------------------------------------------*/
    /*  [Step8] x                                                             */
    /*------------------------------------------------------------------------*/
    if (case1 == 0)
    {
        printf("Case 1 occurs !!!!!\n");

        for (i = 0; i < 8; i++)
            Decoding_X[i] = X1[i];
    }
    else if (case2 == 0)
    {

        /*----------------------------------------------------------------*/
        /*  [Step9] x                                                     */
        /*----------------------------------------------------------------*/
        printf("Case 2 occurs  : x = x * (2)^((p-1)/4)!!!!!\n");

        /*----------------------------------------------------------------*/
        /*  [Step9.1~2] U = p2 = (p-1)/4 is a fixed value. Thus, this     */
        /*              pattern does not execute Step9.1~2                */
        /*----------------------------------------------------------------*/
        U[0] = 0xfffffffb;
        U[1] = 0xffffffff;
        U[2] = 0xffffffff;
        U[3] = 0xffffffff;
        U[4] = 0xffffffff;
        U[5] = 0xffffffff;
        U[6] = 0xffffffff;
        U[7] = 0x1fffffff;

        /*----------------------------------------------------------------*/
        /*  [Step9.3] x3 = 2                                              */
        /*----------------------------------------------------------------*/
        for (i = 1; i < 8; i++)
            X3[i] = 0x0;

        X3[0] = 0x2;

        /*----------------------------------------------------------------*/
        /*  [Step9.4~10] X=(2)^((p-1)/4)                                  */
        /*----------------------------------------------------------------*/
        ECC_MODOP(X3, U, Ed_P, 5);
        read_reg_array(REG_ECC_X1, Decoding_X, 8, 8);

        /*----------------------------------------------------------------*/
        /*  [Step9.11~19] x=x*(2^((p-1)/4))                               */
        /*----------------------------------------------------------------*/
        ECC_MODOP(X1, Decoding_X, Ed_P, 1);
        read_reg_array(REG_ECC_X1, Decoding_X, 8, 8);
    }
    else
    {
        return;  // [Step10]
    }

    /*------------------------------------------------------------------------*/
    /*  [Step12.1~9] if(x[0]!==x_0) x = p - x                                 */
    /*------------------------------------------------------------------------*/
    if ((Decoding_X[0] & 0x1) != x_0)
    {
        printf("x = p - x!!!!!\n");
        Ed_1[0] = 0x0;
        ECC_MODOP(Ed_1, Decoding_X, Ed_P, 3);
        read_reg_array(REG_ECC_X1, Decoding_X, 8, 8);
    }

    printf("Decoding answer: ");
    dump_word_array(Decoding_X, 8);
}

/*
  * @brief  Get EdDSA ED25519 scalar and prefix
  * @param[in]  ecc_curve   The pre-defined ECC curve.
  * @param[in]  priv_key    The private key
  * @param[out] scalar      The scalar factor
  * @param[out] prefix      The prefix
 */
static int  ed25519_get_scalar(enum ed_type eddsa_type,
                               uint8_t *priv_key,
                               uint32_t *scalar, uint32_t *prefix)
{
    int i;

    do_SHA512(priv_key, 32, 1);

    read_reg_array(REG_HMAC_DGST, scalar, 8, 8);
    scalar[7] = (scalar[7] & 0x7FFFFFFF) | 0x40000000;
    scalar[0] =  scalar[0] & 0xFFFFFFF8;

    //obtain prefix
    for (i = 0; i < 8; i++)
    {
        prefix[i] = CRYPTO->HMAC_DGST[i + 8];
        printf("%x\n", prefix[i]);
        //prefix[i] = swap32(prefix[i]);
    }

    return 0;
}


/**
  * @brief  EdDSA ED25519 Key Generation
  * @param[in]  priv_key    The private key
  * @param[out] A       The output public key
  * @param[out] prefix      The output prefix
  * @return  0    Success.
  * @return  -1   "ecc_curve" value is invalid.
  */
static int32_t  ed25519_gen_pub_key(enum ed_type eddsa_type,
                                    uint8_t *priv_key,
                                    uint32_t *A, uint32_t *scalar, uint32_t *prefix)
{
    //uint32_t scalar[8];
    uint32_t Px[8], Py[8];
    int i;

    printf("%s, %d\n", __func__, __LINE__);
    ed25519_get_scalar(eddsa_type, priv_key, scalar, prefix);

    /*------------------------------------------------------------------------*/
    /*  3. Compute (Px, Py) = s * G                                           */
    /*     (1) ~ (3)    init curve                                        */
    /*     (4) Write    the scalar s to K register                        */
    /*------------------------------------------------------------------------*/
    set_ed25519_param(0);
    write_reg_array(REG_ECC_K, scalar, 8, 18);
    printf("%s, %d\n", __func__, __LINE__);
    ECC_KP();
    printf("%s, %d\n", __func__, __LINE__);

    read_reg_array(REG_ECC_X1, Px, 8, 8);
    read_reg_array(REG_ECC_Y1, Py, 8, 8);


    /*------------------------------------------------------------------------*/
    /*  4. Public key A = encoding(Px,Py)                                     */
    /*     (1) Assign Py to the rightmost Lkey bits of A, where Lkey is       */
    /*         the bit length of private key (i.e. 256 bits in Ed25519        */
    /*         series and 456 bits in Ed448 series)                           */
    /*     (2) Assign the rightmost 1 bit of Px to the leftmost 1 bits of A   */
    /*------------------------------------------------------------------------*/
    for (i = 0; i < 8; i++)
        A[i] = Py[i];

    if (Px[0] & 0x1)
        A[7] |= 0x80000000;
    else
        A[7] &= 0x7FFFFFFF;

    for (i = 0; i < 8; i++)
        printf("A[%d] = 0x%08x\n", i, A[i]);

    return 0;
}

uint8_t *eddsa_dom2(enum ed_type eddsa_type, uint8_t *buff, uint8_t *ctx, int ctx_len)
{
    uint8_t *bptr = buff;

    switch (eddsa_type)
    {
        case EDDSA_ED25519:
            // dom2(F,C) is empty string in Ed25519 :
            return bptr;

        case EDDSA_ED25519CTX:
            memcpy(bptr, ed25519ctx_str, 32);
            bptr += 32;
            *bptr++ = 0x00;
            *bptr++ = ctx_len;

            if (ctx_len)
            {
                memcpy(bptr, ctx, ctx_len);
                bptr += ctx_len;
            }

            return bptr;

        case EDDSA_ED25519PH:
            memcpy(bptr, ed25519ctx_str, 32);
            bptr += 32;
            *bptr++ = 0x01;
            *bptr++ = ctx_len;

            if (ctx_len)
            {
                memcpy(bptr, ctx, ctx_len);
                bptr += ctx_len;
            }

            return bptr;

        default:
            break;

    }

    return bptr;
}

/**
  * @brief  EdDSA ED25519 signature generation
  * @param[in]  eddsa_type  ed25519, ed25519ph, or ed25519ctx
  * @param[in]  ecc_curve   The pre-defined ECC curve.
  * @param[in]  priv_key    The private key
  * @param[in]  message     Message
  * @param[in]  msg_len     Message length in byte
  * @param[in]  ctx     context
  * @param[in]  ctx_len     context length
  * @param[in]  R           The output signature R
  * @param[in]  S           The output signature S
  * @return  0    Success.
  * @return  -1   "ecc_curve" value is invalid.
  */
int32_t  ECC_ED25519_SigGen(enum ed_type eddsa_type, uint8_t *priv_key,
                            uint8_t *message, int msg_len,
                            uint8_t *ctx, int ctx_len,
                            uint32_t *R, uint32_t *S)
{
    uint32_t A[8], prefix[8];
    uint32_t r[16], k[16], digest[16];
    uint32_t scalar[8];
    uint8_t *bptr;
    int i;

    /*------------------------------------------------------------------------*/
    /*  1. & 2.                                                               */
    /*     dom2(F,C) is empty string in Ed25519                               */
    /*------------------------------------------------------------------------*/

    printf("%s, %d\n", __func__, __LINE__);

    if (eddsa_type == EDDSA_ED25519CTX)
    {
        /*-----------------------------------------------------------------------------------------------------------------------------------*/
        /*  ED25519ctx                                                                                                                       */
        /*  dom2(F,C) = dom2(0,context) in Ed25519ctx = "SigEd25519 no Ed25519 collisions" || octet(0) || octtex(OLEN(context)) || context   */
        /*  "SigEd25519 " = "53","69","67","45","64","32","35","35","31","39","20" in ASCII format                                           */
        /*  "no Ed25519 " = "6e","6f","20","45","64","32","35","35","31","39","20" in ASCII format                                           */
        /*  "collisions"  = "63","6f","6c","6c","69","73","69","6f","6e","73"      in ASCII format                                           */
        /*  "SigEd25519 no Ed25519 collisions" -> 5369674564323535313920 || 6e6f204564323535313920 || 636f6c6c6973696f6e73                   */
        /*  In this test vector, F = 0x00, context = 666f6f, octtex(OLEN(context)) + context = 00 + 03 + 666f6f -> 0003666f6f                */
        /*  Finally, dom2(F,C) in this test vector = 5369674564323535313920 || 6e6f204564323535313920 || 636f6c6c6973696f6e73 || 0003666f6f  */
        /*  53696745643235353139206e6f204564323535313920636f6c6c6973696f6e730003666f6f -> total 37 bytes                                     */
        /*  53696745643235353139206e6f204564323535313920636f6c6c6973696f6e730100 -> total 34 bytes                                           */
        /*-----------------------------------------------------------------------------------------------------------------------------------*/
    }

    if (eddsa_type == EDDSA_ED25519PH)
    {
        /*-----------------------------------------------------------------------------------------------------------------------------------*/
        /*  ED25519PH                                                                                                                        */
        /*  [Sign Step1~2]                                                                                                                   */
        /*  dom2(F,C) = dom2(1,context) in Ed25519ph = "SigEd25519 no Ed25519 collisions" || octet(1) || octtex(OLEN(context)) || context    */
        /*  "SigEd25519 " = "53","69","67","45","64","32","35","35","31","39","20" in ASCII format                                           */
        /*  "no Ed25519 " = "6e","6f","20","45","64","32","35","35","31","39","20" in ASCII format                                           */
        /*  "collisions"  = "63","6f","6c","6c","69","73","69","6f","6e","73"      in ASCII format                                           */
        /*  "SigEd25519 no Ed25519 collisions" -> 5369674564323535313920 || 6e6f204564323535313920 || 636f6c6c6973696f6e73                   */
        /*  In this test vector, F = 0x01, context = None, octtex(OLEN(context)) + context = 01 + 00 + '' -> 0100                            */
        /*  Finally, dom2(F,C) in this test vector = 5369674564323535313920 || 6e6f204564323535313920 || 636f6c6c6973696f6e73 || 0100        */
        /*  53696745643235353139206e6f204564323535313920636f6c6c6973696f6e730100 -> total 34 bytes                                           */
        /*-----------------------------------------------------------------------------------------------------------------------------------*/
    }

    /*------------------------------------------------------------------------*/
    /*  3. Extract public key A and prefix by EdDSA key generation steps.     */
    /*     dom2(F,C) is empty string in Ed25519                               */
    /*------------------------------------------------------------------------*/

    printf("%s, %d\n", __func__, __LINE__);
    ed25519_gen_pub_key(eddsa_type, priv_key, A, scalar, prefix);
    printf("ed25519_gen_pub_key result: \n");
    printf("ed25519_gen_pub_key result: \n");
    dump_word_array(A, 8);
    printf("%s, %d\n", __func__, __LINE__);

    /*------------------------------------------------------------------------*/
    /*  4. Compute PH_M = PH(M), where M is the signed message.               */
    /*     (1) In PureEdDSA (i.e., Ed25519, Ed25519ctx and Ed448),            */
    /*         PH(M) = M, where PH() is the identity function.                */
    /*     (2) In HashEdDSA (i.e., Ed25519ph and Ed448ph), PH(M) = HASH(M),   */
    /*         where HASH is a cryptographic hashing algorithm, (i.e. SHA-512 */
    /*         in Ed25519 series and SHAKE256 in Ed448 series)                */
    /*------------------------------------------------------------------------*/

    if (eddsa_type == EDDSA_ED25519PH)
    {
        //PH(M) = SHA512(M) in Ed25519ph; MESSAGE = 616263
        memcpy(g_dma_buff, message, msg_len);
        memset(&g_dma_buff[msg_len], 0, 4);

        do_SHA512(g_dma_buff, msg_len, 1);
        read_reg_array(REG_HMAC_DGST, digest, 16, 16);
    }

    printf("%s, %d\n", __func__, __LINE__);

    /*------------------------------------------------------------------------*/
    /*  5. Calculate r = HASH(dom2_FC || prefix || PH_M) in Ed25519 series    */
    /*     or HASH(dom4_FC || prefix || PH_M) in Ed448 series.                */
    /*------------------------------------------------------------------------*/
    printf("## Write data to SHA512 for r ##\n");


    bptr = eddsa_dom2(eddsa_type, g_dma_buff, ctx, ctx_len);
    printf("%s, %d, %d\n", __func__, __LINE__, bptr - g_dma_buff);

    memcpy(bptr, prefix, 32);
    bptr += 32;

    switch (eddsa_type)
    {
        case EDDSA_ED25519:

        //  break;

        case EDDSA_ED25519CTX:
            if (msg_len)
            {
                memcpy(bptr, message, msg_len);
                bptr += msg_len;
            }

            break;

        case EDDSA_ED25519PH:
            memcpy(bptr, digest, 64);
            bptr += 64;
            break;

        default:
            break;
    }

    printf("%s, %d, %d\n", __func__, __LINE__, bptr - g_dma_buff);
    memset(bptr, 0, 4);
    do_SHA512(g_dma_buff, bptr - g_dma_buff, 1);
    read_reg_array(REG_HMAC_DGST, r, 16, 16);

    /*------------------------------------------------------------------------*/
    /*  6. Compute (P1x, P1y) = (r mod L) * G, where L is the group           */
    /*     order of G.                                                        */
    /*     (1) Write the modulus L to N register.                             */
    /*     (2) Write the lowest 544 bits of dividend r to X1 register and     */
    /*         the highest 544 bits of r to Y1 register.                      */
    /*     (3) Set SMOD(CRYPTO_ECC_CTL[2]) to 1                               */
    /*     (4) Set MODOP(CRYPTO_ECC_CTL[12:11]) to 00                         */
    /*     (5) Set ECCOP(CRYPTO_ECC_CTL[10:9]) to 01                          */
    /*     (6) Set FSEL(CRYPTO_ECC_CTL[8]) to 1                               */
    /*     (7) Set START(CRYPTO_ECC_CTL[0]) to 1                              */
    /*     (8) Wait for BUSY(CRYPTO_ECC_STS[0]) to be cleared                 */
    /*     (9) Read result r_mod_L from X1 registers                          */
    /*------------------------------------------------------------------------*/
    write_reg_array(REG_ECC_X1, r, 16, 18);
    clear_reg_array(REG_ECC_Y1, 18);
    set_ed25519_N(NULL);

    ECC_Start_And_Wait((255 << CRYPTO_ECC_CTL_CURVEM_Pos) |
                       CRYPTO_ECC_CTL_SPCSEL_Msk |
                       CRYPTO_ECC_CTL_SPCEN_Msk |
                       (0x1 << CRYPTO_ECC_CTL_ECCOP_Pos) |
                       CRYPTO_ECC_CTL_FSEL_Msk |
                       CRYPTO_ECC_CTL_SMOD_Msk |
                       CRYPTO_ECC_CTL_START_Msk);

    read_reg_array(REG_ECC_X1, r, 8, 8);

    printf("Sign 6-9 result: ");
    dump_word_array(r, 8);


    /*------------------------------------------------------------------------*/
    /*  6. Compute (P1x, P1y) = (r mod L) * G, where L is the group           */
    /*     order of G.                                                        */
    /*     (10) Write the curve parameter A, B, N and curve length M to       */
    /*          corresponding registers.                                      */
    /*     (11) Write the prime modulus p to N registers.                     */
    /*     (12) Write the point G(x, y) to X1, Y1 registers.                  */
    /*     (13) Write the scalar r_mod_L to K register.                       */
    /*     (14) Set CURVEM (CRYPTO_ECC_CTL[31:22]) to key length              */
    /*     (15) Set SPCSEL(CRYPTO_ECC_CTL[16]) to 1                           */
    /*     (16) Set SPCEN(CRYPTO_ECC_CTL[13]) to 1                            */
    /*     (17) Set ECCOP(CRYPTO_ECC_CTL[10:9]) to 00                         */
    /*     (18) Set FSEL(CRYPTO_ECC_CTL[8]) to 0                              */
    /*     (19) Set START(CRYPTO_ECC_CTL[0]) to 1                             */
    /*     (20) Wait for BUSY(CRYPTO_ECC_STS[0]) to be cleared                */
    /*     (21) Read X1 registers to get P1x and Y1 registers to get P1y      */
    /*------------------------------------------------------------------------*/
    printf("R = (r mod order) * B :");
    set_ed25519_param(0);
    write_reg_array(REG_ECC_K, r, 8, 18);
    ECC_KP();

    printf("Sign 6-21 result: ");
    dump_word_array((uint32_t *)REG_ECC_X1, 8);
    dump_word_array((uint32_t *)REG_ECC_Y1, 8);


    /*------------------------------------------------------------------------*/
    /*  7. R = encoding(P1x,P1y)                                              */
    /*     (1) Assign P1y to the rightmost Lkey bits of R, where Lkey is      */
    /*         the bit length of private key (i.e. 256 bits in Ed25519 and    */
    /*         456 bits in Ed448)                                             */
    /*     (2) Assign the rightmost 1 bit of P1x to the leftmost 1 bits of R  */
    /*     order of G.                                                        */
    /*------------------------------------------------------------------------*/
    read_reg_array(REG_ECC_Y1, R, 8, 8);

    if ((CRYPTO->ECC_X1[0] & 0x1) == 0x1)   // MSB from X1[0]
        R[7] |= 0x80000000;
    else
        R[7] &= ~0x80000000;

    printf("Sign 7-2 result: ");
    dump_word_array(R, 8);

#if 0   // check
    // ED25519
    R[7] = 0x55014922;
    R[6] = 0x65e073d8;
    R[5] = 0x74d9e5b8;
    R[4] = 0x1e7f8784;
    R[3] = 0x8a826e80;
    R[2] = 0xcce28690;
    R[1] = 0x72ac60c3;
    R[0] = 0x004356e5;

    // ED25519CTX: 7ad88d192373daca7618919262b320b57b5ae4d14c5f8c28044ea5702fcca455

    // ED25519PH: 41ae6d4eb99b493976f87f9c462b469e803f683d810fd3a91a12b8f02202a798
#endif

    /*------------------------------------------------------------------------*/
    /*  8. Calculate k = HASH(dom2_FC || R || A || PH_M) in Ed25519 series    */
    /*     or HASH(dom4_FC || R || A || PH_M) in Ed448 series.                */
    /*------------------------------------------------------------------------*/
    printf("8. Calculate k\n");
    /* dom2(F,C) is empty string in Ed25519 */

    bptr = eddsa_dom2(eddsa_type, g_dma_buff, ctx, ctx_len);

    /* R */
    memcpy(bptr, R, 32);
    bptr += 32;

    /* A */
    memcpy(bptr, A, 32);
    bptr += 32;

    /* PH(M) */
    switch (eddsa_type)
    {
        case EDDSA_ED25519:
        case EDDSA_ED25519CTX:
            if (msg_len)
            {
                memcpy(bptr, message, msg_len);
                bptr += msg_len;
            }

            break;

        case EDDSA_ED25519PH:
            memcpy(bptr, digest, 64);
            bptr += 64;
            break;
    }

    memset(bptr, 0, 4);
    do_SHA512(g_dma_buff, bptr - g_dma_buff, 1);
    read_reg_array(REG_HMAC_DGST, k, 16, 16);

    printf("SigGen 8: k =");
    dump_word_array(k, 16);

#if 0
    // ED25519CTX :
    // 633b91b4180a2daae02bacb33f37eff0d58e57a8afaed09803a6c77a3361907237ddf70d4e4adcce68c0a8791c78f2bee8d8d96b5e789ca32f6d5743139ff39b

    // ED25519PH :
    // 390ccbe641ab6519def85aef521fab8806cc8687f426942cb1b32965cff87926a5495377ac0715a2e40cdf421ec68fa64edbdbfee01e8267b7c33abf28a22f39
#endif

#ifdef SIGN_METHOD_1
    /*------------------------------------------------------------------------*/
    /*  9. Calculate S = (r + (k mod L) * s) mod L by three modulus           */
    /*     operations                                                         */
    /*     (1) Write the modulus L to N register.                             */
    /*     (2) Write the lowest 544 bits of k to X1 register and the          */
    /*         highest 544 bits of k to Y1 register.                          */
    /*     (3) Set SMOD(CRYPTO_ECC_CTL[2]) to 1                               */
    /*     (4) Set MODOP(CRYPTO_ECC_CTL[12:11]) to 00                         */
    /*     (5) Set ECCOP(CRYPTO_ECC_CTL[10:9]) to 01                          */
    /*     (6) Set FSEL(CRYPTO_ECC_CTL[8]) to 1                               */
    /*     (7) Set START(CRYPTO_ECC_CTL[0]) to 1                              */
    /*     (8) Wait for BUSY(CRYPTO_ECC_STS[0]) to be cleared                 */
    /*     (9) Read result k_mod_L from X1 registers 10) Write the            */
    /*         modulus L to N register.                                       */
    /*------------------------------------------------------------------------*/

    /*
     *  k_mod_L = k mod order L
     */
    write_reg_array(REG_ECC_X1, k, 16, 18);

    printf("EdDSA S : k before mod = ");
    dump_word_array((uint32_t *)REG_ECC_X1, 18);

    clear_reg_array(REG_ECC_Y1, 18);

    set_ed25519_N(NULL);

    ECC_Start_And_Wait((255 << CRYPTO_ECC_CTL_CURVEM_Pos) |
                       CRYPTO_ECC_CTL_SPCSEL_Msk |
                       CRYPTO_ECC_CTL_SPCEN_Msk |
                       (0x1 << CRYPTO_ECC_CTL_ECCOP_Pos) |
                       CRYPTO_ECC_CTL_FSEL_Msk |
                       CRYPTO_ECC_CTL_SMOD_Msk |
                       CRYPTO_ECC_CTL_START_Msk);

    read_reg_array(REG_ECC_X1, k, 8, 8);

#if 0
    // check data mod_x
    // ED25519
    err_x = check_data(CRYPTO_BA + CRYPTO_ECC_X1_0, 0x8ebcea86, err_x);
    err_x = check_data(CRYPTO_BA + CRYPTO_ECC_X1_1, 0x3d19964c, err_x);
    err_x = check_data(CRYPTO_BA + CRYPTO_ECC_X1_2, 0xe7040529, err_x);
    err_x = check_data(CRYPTO_BA + CRYPTO_ECC_X1_3, 0x6cdf00c6, err_x);
    err_x = check_data(CRYPTO_BA + CRYPTO_ECC_X1_4, 0x6125d8f8, err_x);
    err_x = check_data(CRYPTO_BA + CRYPTO_ECC_X1_5, 0x132cec31, err_x);
    err_x = check_data(CRYPTO_BA + CRYPTO_ECC_X1_6, 0x167e3e8a, err_x);
    err_x = check_data(CRYPTO_BA + CRYPTO_ECC_X1_7, 0x0454522e, err_x);
    err_x = check_data(CRYPTO_BA + CRYPTO_ECC_X1_8, 0x0, err_x);

    // ED25519CTX
    // ec5d44b80ba4cc3c94ffd36edeb4fe22304f0ec051bfc9f825f831ad75212c6

    // ED25519PH
    // 0xdd63af668d7f03cc8669d82355d48664fd19a7c2556aa6f32f0564d08d63bbd
#endif
    printf("EdDSA S : k after mod = ");

    for (i = 0; i < 18; i++) printf("0x%x ", CRYPTO->ECC_X1[i]);

    printf("\n");

    /*------------------------------------------------------------------------*/
    /*  9. Calculate S = (r + (k mod L) * s) mod L by three modulus           */
    /*     operations                                                         */
    /*     (10) Write the modulus L to N register.                            */
    /*     (11) Set CURVEM(CRYPTO_ECC_CTL[31:22]) to the bit length of        */
    /*          multiplication.                                               */
    /*     (12) Write the multiplicand k_mod_L to X1 register and the         */
    /*           multiplier s to Y1 register.                                 */
    /*     (13) Set CURVEM (CRYPTO_ECC_CTL[31:22]) to key length              */
    /*     (14) Set MODOP(CRYPTO_ECC_CTL[12:11]) to 01                        */
    /*     (15) Set ECCOP(CRYPTO_ECC_CTL[10:9]) to 01                         */
    /*     (16) Set FSEL(CRYPTO_ECC_CTL[8]) to 1                              */
    /*     (17) Set START(CRYPTO_ECC_CTL[0]) to 1                             */
    /*     (18) Wait for BUSY(CRYPTO_ECC_STS[0]) to be cleared                */
    /*     (19) Read result ks_mod_L from X1 registers                        */
    /*------------------------------------------------------------------------*/

    /*
     *  [Sign Step9.10~19] ks_mod_L = ( k_mod_L *scalar) mod order
     *
     *   S = r + (k mod order) * scalar mod order
     */
    printf("## S=r+(k mod order)*scalar mod order ##\n");
    write_reg_array(REG_ECC_X1, k, 8, 18);
    write_reg_array(REG_ECC_Y1, scalar, 8, 18);
    set_ed25519_N(NULL);

    printf("EdDSA S : scalar = ");
    dump_word_array((uint32_t *)REG_ECC_Y1, 18);

    printf("EdDSA S : n =");
    dump_word_array((uint32_t *)REG_ECC_N, 18);

    /* k * scalar */
    ECC_Start_And_Wait((255 << CRYPTO_ECC_CTL_CURVEM_Pos) |
                       CRYPTO_ECC_CTL_SPCSEL_Msk |
                       CRYPTO_ECC_CTL_SPCEN_Msk |
                       (0x1 << CRYPTO_ECC_CTL_MODOP_Pos) |
                       (0x1 << CRYPTO_ECC_CTL_ECCOP_Pos) |
                       CRYPTO_ECC_CTL_FSEL_Msk |
                       CRYPTO_ECC_CTL_START_Msk);

    printf("EdDSA S: k * scalar = ");
    dump_word_array((uint32_t *)REG_ECC_X1, 18);

    /*------------------------------------------------------------------------*/
    /*  9. Calculate S = (r + (k mod L) * s) mod L by three modulus           */
    /*     operations                                                         */
    /*     (20) Write the modulus L to N register.                            */
    /*     (21) Write the summand ks_mod_L to X1 register and the addend      */
    /*          r_mod_L to Y1 register.                                       */
    /*     (22) Set MODOP(CRYPTO_ECC_CTL[12:11]) to 10                        */
    /*     (23) Set ECCOP(CRYPTO_ECC_CTL[10:9]) to 01                         */
    /*     (24) Set FSEL(CRYPTO_ECC_CTL[8]) to 1                              */
    /*     (25) Set START(CRYPTO_ECC_CTL[0]) to 1                             */
    /*     (26) Wait for BUSY(CRYPTO_ECC_STS[0]) to be cleared                */
    /*     (27) Read result S from X1 registers(10) Write the modulus L       */
    /*          to N register.                                                */
    /*------------------------------------------------------------------------*/

    write_reg_array(REG_ECC_Y1, r, 8, 18);

    printf("EdDSA S : r =");
    dump_word_array((uint32_t *)REG_ECC_Y1, 18);

    /* r + (k * scalar) */
    ECC_Start_And_Wait((255 << CRYPTO_ECC_CTL_CURVEM_Pos) |
                       CRYPTO_ECC_CTL_SPCSEL_Msk |
                       CRYPTO_ECC_CTL_SPCEN_Msk |
                       (0x2 << CRYPTO_ECC_CTL_MODOP_Pos) |
                       (0x1 << CRYPTO_ECC_CTL_ECCOP_Pos) |
                       CRYPTO_ECC_CTL_FSEL_Msk |
                       CRYPTO_ECC_CTL_START_Msk);

    read_reg_array(REG_ECC_X1, S, 8, 8);

#else /* !SIGN_METHOD_1 */

    /*------------------------------------------------------------------------*/
    /*  10. Calculate S = (r + (k mod L) * s) mod L by ECDSAS bit             */
    /*      operations                                                        */
    /*     (1) Write the lowest 544 bits of k to X1 register and the          */
    /*         highest 544 bits of k to Y1 register.                          */
    /*     (2) Write the the multiplier s to X2 register and the summand      */
    /*         r to Y2 register.                                              */
    /*     (3) Write the modulus L to N register.                             */
    /*     (4) Set CURVEM(CRYPTO_ECC_CTL[31:22]) to the bit length of         */
    /*         modulus L.                                                     */
    /*     (5) Set SPCSEL(CRYPTO_ECC_CTL[16]) to 1.                           */
    /*     (6) Set SPCEN(CRYPTO_ECC_CTL[13]) to 1.                            */
    /*     (7) Set ECCOP(CRYPTO_ECC_CTL[10:9]) to 01                          */
    /*     (8) Set FSEL(CRYPTO_ECC_CTL[8]) to 1                               */
    /*     (9) Set ECDSAS(CRYPTO_ECC_CTL[4]) to 1                             */
    /*     (10) Set START(CRYPTO_ECC_CTL[0]) to 1                             */
    /*     (11) Wait for BUSY(CRYPTO_ECC_STS[0]) to be cleared                */
    /*     (12) Read result S from X1 registers                               */
    /*------------------------------------------------------------------------*/

    printf("## S=r+(k mod order)*scalar mod order by ECDSAS ##");

    write_reg_array(REG_ECC_X1, k, 8, 18);
    clear_reg_array(REG_ECC_Y1, 18);

    write_reg_array(REG_ECC_X2, scalar, 8, 18);
    write_reg_array(REG_ECC_Y2, r, 8, 18);

    set_ed25519_N(NULL);

    ECC_Start_And_Wait((255 << CRYPTO_ECC_CTL_CURVEM_Pos) |
                       CRYPTO_ECC_CTL_SPCSEL_Msk |
                       CRYPTO_ECC_CTL_SPCEN_Msk |
                       CRYPTO_ECC_CTL_FSEL_Msk |
                       CRYPTO_ECC_CTL_ECDSAS_Msk |
                       CRYPTO_ECC_CTL_START_Msk);

    read_reg_array(REG_ECC_X1, S, 8, 8);

#endif /* SIGN_METHOD_1 */

#if 0
    // step 9 or 10 result
    // check data mod_x for S
    // ED25519CTX
    // 0xddb7e4e8e884b1fb2b487a585a6ccf6d5b2e9c4b77f7a90220013950b95368b

    // ED25519PH
    // 0x63408262a1cd1aa2a0636e6618c0ba1aaf5ad62d003205a352a3c464250f831
#endif

    return 0;
}

/**
  * @brief  EdDSA ED25519 signature generation
  * @param[in]  eddsa_type  ed25519, ed25519ph, or ed25519ctx
  * @param[in]  ecc_curve   The pre-defined ECC curve.
  * @param[in]  priv_key    The private key
  * @param[in]  message     Message
  * @param[in]  msg_len     Message length in byte
  * @param[in]  A           The public key
  * @param[in]  R           The signature R
  * @param[in]  S           The signature S
  * @return  0    Success.
  * @return  -1   "ecc_curve" value is invalid.
  */
int32_t  ECC_ED25519_Verify(enum ed_type eddsa_type,
                            uint8_t *message, int msg_len,
                            uint8_t *ctx, int ctx_len,
                            uint8_t *pub_key,
                            uint32_t *R, uint32_t *S)
{
    uint32_t digest[16];
    uint32_t *A = (uint32_t *)pub_key;
    uint32_t k[8];
    uint32_t Rx[8], Ry[8];
    uint32_t Ax[8], Ay[8];
    uint32_t KAx[8], KAy[8];
    uint32_t SBx[8], SBy[8];
    uint8_t *bptr;

    /*------------------------------------------------------------------------*/
    /*  1. verify a signature with message M, public key A and flag F         */
    /*     (1) R = the leftmost Lkey bits of signature, where Lkey is the     */
    /*         bit length of private key                                      */
    /*     (2) S = the rightmost Lkey bits of signature                       */
    /*     (3) If ((order L ? S) || (S < 0)), then signature is invalid       */
    /*------------------------------------------------------------------------*/

    /*
     *  Skip to check M, A and F
     */

    /*------------------------------------------------------------------------*/
    /*  2. Compute dom2_FC = dom2(F,C) in Ed25519 series, where a context     */
    /*     C of at most 255 octets; a flag F is 1 for HashEdDSA; a flag F     */
    /*     is 0 or irrelevant for PureEdDSA;                                  */
    /*     (1) When signing or verifying Ed25519, dom2(x,y) return blank      */
    /*         octet string.                                                  */
    /*     (2) In other cases, dom2(x,y) return the octet string:             */
    /*         "SigEd25519 no Ed25519 collisions" || octet(x) ||              */
    /*         octet(OLEN(y)) || y, where "SigEd25519 no Ed25519 collisions"  */
    /*         is in ASCII format with 32 octets; x is between 0 and 255;     */
    /*         OLEN(y) is the number of octets in string y in range 0-255;    */
    /*         y is an octet string (at most 255 octets).                     */
    /*------------------------------------------------------------------------*/

    /*
     *  dom2(F,C) is empty string in Ed25519
     */

    /*------------------------------------------------------------------------*/
    /*  3. Compute dom4_FC = dom4(F,C) in Ed448 series, where a context C     */
    /*     of at most 255 octets; a flag F is 1 for HashEdDSA; a flag F is    */
    /*     0 or irrelevant for PureEdDSA;                                     */
    /*     (1) The dom4(x,y) return the octet string: " SigEd448" ||          */
    /*         octet(x) || octet(OLEN(y)) || y, where " SigEd448" is in       */
    /*         ASCII format with 8 octets; x is between 0 and 255; OLEN(y)    */
    /*         is the number of octets in string y in range 0-255; y is an    */
    /*         octet string (at most 255 octets).                             */
    /*------------------------------------------------------------------------*/


    /*------------------------------------------------------------------------*/
    /*  4. Compute PH_M = PH(M), where M is the signed message.               */
    /*     (1) In PureEdDSA (i.e., Ed25519, Ed25519ctx and Ed448),            */
    /*         PH(M) = M, where PH() is the identity function.                */
    /*     (2) In HashEdDSA (i.e., Ed25519ph and Ed448ph), PH(M) = HASH(M),   */
    /*         where HASH is a cryptographic hashing algorithm,               */
    /*         (i.e. SHA-512 in Ed25519 series and SHAKE256 in Ed448 series)  */
    /*------------------------------------------------------------------------*/

    /*
     *  PH(M) = M in Ed25519 and ED25519CTX
     */
    if (eddsa_type == EDDSA_ED25519PH)
    {
        //PH(M) = SHA512(M) in Ed25519ph; MESSAGE = 616263
        memcpy(g_dma_buff, message, msg_len);
        memset(&g_dma_buff[msg_len], 0, 4);

        do_SHA512(g_dma_buff, msg_len, 1);
        read_reg_array(REG_HMAC_DGST, digest, 16, 16);
    }

    /*------------------------------------------------------------------------*/
    /*  5. Calculate k = HASH(dom2_FC || R || A || PH_M) in Ed25519 series    */
    /*     or HASH(dom4_FC || R || A || PH_M) in Ed448 series.                */
    /*------------------------------------------------------------------------*/

    printf("## Write data to SHA512 for k ##\n");
    bptr = eddsa_dom2(eddsa_type, g_dma_buff, ctx, ctx_len);

    // R :
    memcpy(bptr, R, 32);//signature R
    bptr += 32;
    printf("public key:=\n");
    dump_word_array(A, 32);
    memcpy(bptr, A, 32);//public Key
    bptr += 32;

    switch (eddsa_type)
    {
        case EDDSA_ED25519:
        case EDDSA_ED25519CTX:
            if (msg_len)
            {
                memcpy(bptr, message, msg_len);
                bptr += msg_len;//append msg
            }

            break;

        case EDDSA_ED25519PH:
            memcpy(bptr, digest, 64);
            bptr += 64;
            break;

        default:
            break;
    }

    memset(bptr, 0, 4);
    do_SHA512(g_dma_buff, bptr - g_dma_buff, 1);
    printf("########### SHA512(h) is generated !! ##########\n");
    read_reg_array(REG_HMAC_DGST, digest, 16, 16);


    printf("Verify 5: digest =\n");
    dump_word_array(digest, 16);
#if 0
    // ED25519CTX
    err_x = check_data(CRYPTO_BA + 0x308 + (0x4 * 15), 0x633b91b4, err_x);
    err_x = check_data(CRYPTO_BA + 0x308 + (0x4 * 14), 0x180a2daa, err_x);
    err_x = check_data(CRYPTO_BA + 0x308 + (0x4 * 13), 0xe02bacb3, err_x);
    err_x = check_data(CRYPTO_BA + 0x308 + (0x4 * 12), 0x3f37eff0, err_x);
    err_x = check_data(CRYPTO_BA + 0x308 + (0x4 * 11), 0xd58e57a8, err_x);
    err_x = check_data(CRYPTO_BA + 0x308 + (0x4 * 10), 0xafaed098, err_x);
    err_x = check_data(CRYPTO_BA + 0x308 + (0x4 * 9), 0x03a6c77a, err_x);
    err_x = check_data(CRYPTO_BA + 0x308 + (0x4 * 8), 0x33619072, err_x);
    err_x = check_data(CRYPTO_BA + 0x308 + (0x4 * 7), 0x37ddf70d, err_x);
    err_x = check_data(CRYPTO_BA + 0x308 + (0x4 * 6), 0x4e4adcce, err_x);
    err_x = check_data(CRYPTO_BA + 0x308 + (0x4 * 5), 0x68c0a879, err_x);
    err_x = check_data(CRYPTO_BA + 0x308 + (0x4 * 4), 0x1c78f2be, err_x);
    err_x = check_data(CRYPTO_BA + 0x308 + (0x4 * 3), 0xe8d8d96b, err_x);
    err_x = check_data(CRYPTO_BA + 0x308 + (0x4 * 2), 0x5e789ca3, err_x);
    err_x = check_data(CRYPTO_BA + 0x308 + (0x4 * 1), 0x2f6d5743, err_x);
    err_x = check_data(CRYPTO_BA + 0x308 + (0x4 * 0), 0x139ff39b, err_x);

    // ED25519PH
    // 390ccbe641ab6519def85aef521fab8806cc8687f426942cb1b32965cff87926a5495377ac0715a2e40cdf421ec68fa64edbdbfee01e8267b7c33abf28a22f39
#endif

    /*------------------------------------------------------------------------*/
    /*  6. Recover (P1x, P1y) = Decoding(R) by EdDSA Decoding() steps         */
    /*     (1) If Decoding(R) fails, then signature is invalid                */
    /*------------------------------------------------------------------------*/
    // R' = Decoding(R)
    printf("## Decoding(R) start ##\n");
    Decoding(R, 8, Rx, Ry);

    printf("Verify 6: Rx =\n");
    dump_word_array(Rx, 8);
    printf("Verify 6: Ry =\n");
    dump_word_array(Ry, 8);

#if 0
    //    ED25519
    //    Rx[7] =  0x6218e309;
    //    Rx[6] =  0xd40065fc;
    //    Rx[5] =  0xc338b312;
    //    Rx[4] =  0x7f468371;
    //    Rx[3] =  0x82324bd0;
    //    Rx[2] =  0x1ce6f3cf;
    //    Rx[1] =  0x81ab44e6;
    //    Rx[0] =  0x2959c82a;
    //    Ry[7] =  0x55014922;
    //    Ry[6] =  0x65e073d8;
    //    Ry[5] =  0x74d9e5b8;
    //    Ry[4] =  0x1e7f8784;
    //    Ry[3] =  0x8a826e80;
    //    Ry[2] =  0xcce28690;
    //    Ry[1] =  0x72ac60c3;
    //    Ry[0] =  0x004356e5;

    // ED25519CTX
    Rx[7] =  0x500f1ad4;
    Rx[6] =  0x21cc7515;
    Rx[5] =  0xe6ece3ed;
    Rx[4] =  0x0fdff710;
    Rx[3] =  0x425e87ef;
    Rx[2] =  0x6a683f2c;
    Rx[1] =  0x1646c2d8;
    Rx[0] =  0xc18474c6;
    Ry[7] =  0x7ad88d19;
    Ry[6] =  0x2373daca;
    Ry[5] =  0x76189192;
    Ry[4] =  0x62b320b5;
    Ry[3] =  0x7b5ae4d1;
    Ry[2] =  0x4c5f8c28;
    Ry[1] =  0x044ea570;
    Ry[0] =  0x2fcca455;

    // ED25519PH
    Rx[7] = 0x5375fe52;
    Rx[6] = 0xbc22ffe4;
    Rx[5] = 0x1e242f75;
    Rx[4] = 0xfc9f808d;
    Rx[3] = 0xdf60dbf1;
    Rx[2] = 0xe6a40c5c;
    Rx[1] = 0xc5c514a8;
    Rx[0] = 0x5422d0a8;
    Ry[7] = 0x41ae6d4e;
    Ry[6] = 0xb99b4939;
    Ry[5] = 0x76f87f9c;
    Ry[4] = 0x462b469e;
    Ry[3] = 0x803f683d;
    Ry[2] = 0x810fd3a9;
    Ry[1] = 0x1a12b8f0;
    Ry[0] = 0x2202a798;
#endif
    /*------------------------------------------------------------------------*/
    /*  7. Recover the point of public key PA (Ax,Ay) = Decoding(A) by        */
    /*     EdDSA Decoding() steps                                             */
    /*     (1) If Decoding(A) fails, then signature is invalid                */
    /*------------------------------------------------------------------------*/

    printf("## Decoding(A) start ##");
    Decoding(A, 8, Ax, Ay);

    printf("Verify 6: Ax =\n");
    dump_word_array(Ax, 8);
    printf("Verify 6: Ay =\n");
    dump_word_array(Ay, 8);

#if 0
    //   ED25519
    //    Ax[7] =  0x55d0e09a;
    //    Ax[6] =  0x2b9d3429;
    //    Ax[5] =  0x2297e08d;
    //    Ax[4] =  0x60d0f620;
    //    Ax[3] =  0xc513d472;
    //    Ax[2] =  0x53187c24;
    //    Ax[1] =  0xb12786bd;
    //    Ax[0] =  0x777645ce;
    //    Ay[7] =  0x1a5107f7;
    //    Ay[6] =  0x681a02af;
    //    Ay[5] =  0x2523a6da;
    //    Ay[4] =  0xf372e10e;
    //    Ay[3] =  0x3a0764c9;
    //    Ay[2] =  0xd3fe4bd5;
    //    Ay[1] =  0xb70ab182;
    //    Ay[0] =  0x01985ad7;

    // ED25519CTX
    Ax[7] =  0x3493c89a;
    Ax[6] =  0x1d429617;
    Ax[5] =  0x95326fb7;
    Ax[4] =  0x7ddda9b1;
    Ax[3] =  0x073eb509;
    Ax[2] =  0x54eec3ac;
    Ax[1] =  0xc573cd71;
    Ax[0] =  0x8bed3093;
    Ay[7] =  0x128224ab;
    Ay[6] =  0xe0fe0c86;
    Ay[5] =  0xfb8badb4;
    Ay[4] =  0x2b1c85d6;
    Ay[3] =  0xaef9f59c;
    Ay[2] =  0x25f0290c;
    Ay[1] =  0x7f8f964f;
    Ay[0] =  0x5e42c9df;

    // ED25519PH
    // check data kp_x
    // 58b401b9df6f65a34625400a43fa6e89dd5ae7440e9899c9c96eea995b72fc2f
    // check data kp_y
    // 3fe267346819f8eb644dfd2eef6754c3345024e1702c93f43b565ead932b17ec
#endif
    /*------------------------------------------------------------------------*/
    /*  8. Compute (P2x, P2y) = (k mod L) * PA, where L is the group          */
    /*     order of G.                                                        */
    /*     (1) Write the modulus L to N register.                             */
    /*     (2) Write the lowest 544 bits of dividend k to X1 register and     */
    /*         the highest 544 bits of k to Y1 register.                      */
    /*     (3) Set SMOD(CRYPTO_ECC_CTL[2]) to 1                               */
    /*     (4) Set MODOP(CRYPTO_ECC_CTL[12:11]) to 00                         */
    /*     (5) Set ECCOP(CRYPTO_ECC_CTL[10:9]) to 01                          */
    /*     (6) Set FSEL(CRYPTO_ECC_CTL[8]) to 1                               */
    /*     (7) Set START(CRYPTO_ECC_CTL[0]) to 1                              */
    /*     (8) Wait for BUSY(CRYPTO_ECC_STS[0]) to be cleared                 */
    /*     (9) Read result k_mod_L from X1 registers                          */
    /*------------------------------------------------------------------------*/
    printf("## k[511:0] mod order ##\n");
    write_reg_array(REG_ECC_X1, digest, 16, 18);
    clear_reg_array(REG_ECC_Y1, 18);
    set_ed25519_N(NULL);

    ECC_Start_And_Wait((255 << CRYPTO_ECC_CTL_CURVEM_Pos) |
                       CRYPTO_ECC_CTL_SPCSEL_Msk |
                       CRYPTO_ECC_CTL_SPCEN_Msk |
                       (0x1 << CRYPTO_ECC_CTL_ECCOP_Pos) |
                       CRYPTO_ECC_CTL_FSEL_Msk |
                       CRYPTO_ECC_CTL_SMOD_Msk |
                       CRYPTO_ECC_CTL_START_Msk);

    read_reg_array(REG_ECC_X1, k, 8, 8);

    printf("Verify 8: k =\n");
    dump_word_array(k, 8);
#if 0
    // check data mod_x
    // ED25519
    err_x = check_data(CRYPTO_BA + CRYPTO_ECC_X1_0, 0x8ebcea86, err_x);
    err_x = check_data(CRYPTO_BA + CRYPTO_ECC_X1_1, 0x3d19964c, err_x);
    err_x = check_data(CRYPTO_BA + CRYPTO_ECC_X1_2, 0xe7040529, err_x);
    err_x = check_data(CRYPTO_BA + CRYPTO_ECC_X1_3, 0x6cdf00c6, err_x);
    err_x = check_data(CRYPTO_BA + CRYPTO_ECC_X1_4, 0x6125d8f8, err_x);
    err_x = check_data(CRYPTO_BA + CRYPTO_ECC_X1_5, 0x132cec31, err_x);
    err_x = check_data(CRYPTO_BA + CRYPTO_ECC_X1_6, 0x167e3e8a, err_x);
    err_x = check_data(CRYPTO_BA + CRYPTO_ECC_X1_7, 0x0454522e, err_x);
    err_x = check_data(CRYPTO_BA + CRYPTO_ECC_X1_8, 0x0, err_x);

    // ED25519CTX
    // ec5d44b80ba4cc3c94ffd36edeb4fe22304f0ec051bfc9f825f831ad75212c6

    // ED25519PH
    // dd63af668d7f03cc8669d82355d48664fd19a7c2556aa6f32f0564d08d63bbd
#endif

    /*------------------------------------------------------------------------*/
    /*  8. Compute (P2x, P2y) = (k mod L) * PA, where L is the group          */
    /*     order of G.                                                        */
    /*     (10) Write the curve parameter A, B, N and curve length M to       */
    /*          corresponding registers                                       */
    /*     (11) Write the prime modulus p to N registers                      */
    /*     (12) Write the point PA(Ax, Ay) to X1 and Y1 registers             */
    /*     (13) Write the scalar k_mod_L to K register                        */
    /*     (14) Set CURVEM (CRYPTO_ECC_CTL[31:22]) to key length              */
    /*     (15) Set SPCSEL(CRYPTO_ECC_CTL[16]) to 1                           */
    /*     (16) Set SPCEN(CRYPTO_ECC_CTL[13]) to 1                            */
    /*     (17) Set ECCOP(CRYPTO_ECC_CTL[10:9]) to 00                         */
    /*     (18) Set FSEL(CRYPTO_ECC_CTL[8]) to 0                              */
    /*     (19) Set START(CRYPTO_ECC_CTL[0]) to 1                             */
    /*     (20) Wait for BUSY(CRYPTO_ECC_STS[0]) to be cleared                */
    /*     (21) Read X1 registers to get P2x and Y1 registers to get P2y      */
    /*------------------------------------------------------------------------*/

    //-----------------------------------------------------------------------------------
    //[Verify Step8.10~21] (P2x,P2y) = k_mod_L * PA
    //-----------------------------------------------------------------------------------
    // KA = K * A'
    // input ECC parameter
    printf("### Verify 8-10 KA = K * A' :");
    set_ed25519_param(0);
    write_reg_array(REG_ECC_X1, Ax, 8, 18);
    write_reg_array(REG_ECC_Y1, Ay, 8, 18);
    write_reg_array(REG_ECC_K, k, 8, 18);
    ECC_KP();

    read_reg_array(REG_ECC_X1, KAx, 8, 8);
    read_reg_array(REG_ECC_Y1, KAy, 8, 8);

    printf("Verify 8-10: KAx =\n");
    dump_word_array(KAx, 8);
    printf("Verify 8-10: KAy =\n");
    dump_word_array(KAy, 8);

#if 0
    // ED25519
    KAx[7] =  0x7fa3aaef;
    KAx[6] =  0x0a604a9a;
    KAx[5] =  0x4c2a9b83;
    KAx[4] =  0xfae02fb0;
    KAx[3] =  0x6a02a19c;
    KAx[2] =  0x943e3af8;
    KAx[1] =  0x94ce5856;
    KAx[0] =  0xc8936f22;
    KAy[7] =  0x0dcede2a;
    KAy[6] =  0x33bab5c4;
    KAy[5] =  0x4f1ee965;
    KAy[4] =  0x995e21df;
    KAy[3] =  0x5cfc6784;
    KAy[2] =  0x2f1b90e2;
    KAy[1] =  0xc69f9ea6;
    KAy[0] =  0x590a899d;

#endif
    /*------------------------------------------------------------------------*/
    /*  10. Compute SG (SGx, SGy) = S * G, where L is the group order of G.   */
    /*      (1) Write the curve parameter A, B, N and curve length M to       */
    /*          corresponding registers                                       */
    /*      (2) Write the prime modulus p to N registers                      */
    /*      (3) Write the point G to X1 and Y1 registers                      */
    /*      (4) Write the scalar S to K register                              */
    /*      (5) Set CURVEM (CRYPTO_ECC_CTL[31:22]) to key length              */
    /*      (6) Set SPCSEL(CRYPTO_ECC_CTL[16]) to 1                           */
    /*      (7) Set SPCEN(CRYPTO_ECC_CTL[13]) to 1                            */
    /*      (8) Set ECCOP(CRYPTO_ECC_CTL[10:9]) to 00                         */
    /*      (9) Set FSEL(CRYPTO_ECC_CTL[8]) to 0                              */
    /*      (10) Set START(CRYPTO_ECC_CTL[0]) to 1                            */
    /*      (11) Wait for BUSY(CRYPTO_ECC_STS[0]) to be cleared               */
    /*      (12) Read X1 registers to get SGx and Y1 registers to get SGy     */
    /*------------------------------------------------------------------------*/

    // SB = S * B
    // input ECC parameter
    printf("### Verify 10: SB = S * B' :");
    set_ed25519_param(0);
    //write scalar to sk
    write_reg_array(REG_ECC_K, S, 8, 18);

    ECC_KP();

    //Result
    read_reg_array(REG_ECC_X1, SBx, 8, 8);
    read_reg_array(REG_ECC_Y1, SBy, 8, 8);

    printf("Verify 10: SBx =\n");
    dump_word_array(SBx, 8);
    printf("Verify 10: SBy =\n");
    dump_word_array(SBy, 8);

#if 0
    // ED25519
    SBx[7] =  0x114237e0;
    SBx[6] =  0x16d3f259;
    SBx[5] =  0x8171d4cc;
    SBx[4] =  0x242eee95;
    SBx[3] =  0x460ae1ed;
    SBx[2] =  0x35857f80;
    SBx[1] =  0xc2d214b7;
    SBx[0] =  0xe9804938;
    SBy[7] =  0x2794dddd;
    SBy[6] =  0xd6a5cf42;
    SBy[5] =  0xe54475f2;
    SBy[4] =  0x90e8c5d9;
    SBy[3] =  0x2c9c6f36;
    SBy[2] =  0xc5c16df1;
    SBy[1] =  0xe5f992af;
    SBy[0] =  0x998a6cfe;

#endif
    /*------------------------------------------------------------------------*/
    /*  9. Compute RA (RAx, RAy) = (P1x, P1y) + (P2x, P2y)                    */
    /*     (1) If (P1x, P1y) is the negative point of (P2x, P2y), the         */
    /*         signature is invalid and return.                               */
    /*     (2) Write the curve parameter A, B, N, and curve length M to       */
    /*         corresponding registers                                        */
    /*     (3) Write the P1x, P1y to X1, Y1 registers                         */
    /*     (4) Write the P2x, P2y to X2, Y2 registers                         */
    /*     (5) Set ECCOP(CRYPTO_ECC_CTL[10:9]) to 10                          */
    /*     (6) Set START(CRYPTO_ECC_CTL[0]) to 1                              */
    /*     (7) Wait for BUSY(CRYPTO_ECC_STS[0]) to be cleared                 */
    /*     (8) Read X1, Y1 registers to get RA (RAx, RAy)                     */
    /*------------------------------------------------------------------------*/

    //-----------------------------------------------------------------------------------
    //[Verify Step9] RA = (P1x,P1y) + (P2x,P2y)
    //-----------------------------------------------------------------------------------
    // (SBx,SBy) = (Rx,Ry) + (KAx,KAy)
    // input ECC parameter
    printf("### Verify 9: point addition : R + KA :");
    set_ed25519_param(0);

    //write A' to x1 and y1
    write_reg_array(REG_ECC_X1, Rx, 8, 18);
    write_reg_array(REG_ECC_Y1, Ry, 8, 18);

    //write A' to x2 and y2
    write_reg_array(REG_ECC_X2, KAx, 8, 18);
    write_reg_array(REG_ECC_Y2, KAy, 8, 18);

    //point addition
    ECC_PA();

    /*------------------------------------------------------------------------*/
    /*  11. If ((SGx == RAx) && (SGy == RAy)), then signature is valid;       */
    /*      others signature is invalid                                       */
    /*------------------------------------------------------------------------*/

    if (compare_word_array((uint32_t *)REG_ECC_X1, SBx, 8) != 0)
    {
        printf("Verify 11: error: \n");
        dump_word_array((uint32_t *)REG_ECC_X1, 8);
        printf("Expected: \n");
        dump_word_array((uint32_t *)SBx, 8);
        return -1;
    }

    if (compare_word_array((uint32_t *)REG_ECC_Y1, SBy, 8) != 0)
    {
        printf("Verify 11: error: \n");
        dump_word_array((uint32_t *)REG_ECC_Y1, 8);
        return -1;
    }

    return 0;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/