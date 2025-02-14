/**************************************************************************//**
 * @file     crypto.h
 * @version  V1.10
 * @brief    Cryptographic Accelerator driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __CRYPTO_H__
#define __CRYPTO_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup CRYPTO_Driver CRYPTO Driver
  @{
*/

/** @addtogroup CRYPTO_EXPORTED_CONSTANTS CRYPTO Exported Constants
  @{
*/

#define PRNG_KEY_SIZE_128       0UL    /*!< PRNG select 128-bit key length           \hideinitializer */
#define PRNG_KEY_SIZE_163       1UL    /*!< PRNG select 163-bit key length           \hideinitializer */
#define PRNG_KEY_SIZE_192       2UL    /*!< PRNG select 192-bit key length           \hideinitializer */
#define PRNG_KEY_SIZE_224       3UL    /*!< PRNG select 224-bit key length           \hideinitializer */
#define PRNG_KEY_SIZE_233       4UL    /*!< PRNG select 233-bit key length           \hideinitializer */
#define PRNG_KEY_SIZE_255       5UL    /*!< PRNG select 255-bit key length           \hideinitializer */
#define PRNG_KEY_SIZE_256       6UL    /*!< PRNG select 256-bit key length           \hideinitializer */
#define PRNG_KEY_SIZE_283       7UL    /*!< PRNG select 283-bit key length           \hideinitializer */
#define PRNG_KEY_SIZE_384       8UL    /*!< PRNG select 384-bit key length           \hideinitializer */
#define PRNG_KEY_SIZE_409       9UL    /*!< PRNG select 409-bit key length           \hideinitializer */
#define PRNG_KEY_SIZE_512       10UL   /*!< PRNG select 512-bit key length           \hideinitializer */
#define PRNG_KEY_SIZE_521       11UL   /*!< PRNG select 521-bit key length           \hideinitializer */
#define PRNG_KEY_SIZE_571       12UL   /*!< PRNG select 571-bit key length           \hideinitializer */

#define PRNG_SEED_CONT          0UL     /*!< PRNG using current seed                 \hideinitializer */
#define PRNG_SEED_RELOAD        1UL     /*!< PRNG reload new seed                    \hideinitializer */

#define AES_KEY_SIZE_128        0UL     /*!< AES select 128-bit key length           \hideinitializer */
#define AES_KEY_SIZE_192        1UL     /*!< AES select 192-bit key length           \hideinitializer */
#define AES_KEY_SIZE_256        2UL     /*!< AES select 256-bit key length           \hideinitializer */

#define AES_MODE_ECB            0UL     /*!< AES select ECB mode                     \hideinitializer */
#define AES_MODE_CBC            1UL     /*!< AES select CBC mode                     \hideinitializer */
#define AES_MODE_CFB            2UL     /*!< AES select CFB mode                     \hideinitializer */
#define AES_MODE_OFB            3UL     /*!< AES select OFB mode                     \hideinitializer */
#define AES_MODE_CTR            4UL     /*!< AES select CTR mode                     \hideinitializer */
#define AES_MODE_CBC_CS1        0x10UL  /*!< AES select CBC CS1 mode                 \hideinitializer */
#define AES_MODE_CBC_CS2        0x11UL  /*!< AES select CBC CS2 mode                 \hideinitializer */
#define AES_MODE_CBC_CS3        0x12UL  /*!< AES select CBC CS3 mode                 \hideinitializer */
#define AES_MODE_GCM            0x20UL  /*!< AES select GCM (Galois/Counter Mode)    \hideinitializer */
#define AES_MODE_GHASH          0x21UL  /*!< AES select GHASH (Galois Hash Function) \hideinitializer */
#define AES_MODE_CCM            0x22UL  /*!< AES select CCM (Counter with CBC-MAC Mode)   \hideinitializer */

#define AES_NO_SWAP             0UL     /*!< AES do not swap input and output data   \hideinitializer */
#define AES_OUT_SWAP            1UL     /*!< AES swap output data                    \hideinitializer */
#define AES_IN_SWAP             2UL     /*!< AES swap input data                     \hideinitializer */
#define AES_IN_OUT_SWAP         3UL     /*!< AES swap both input and output data     \hideinitializer */

#define SHA_MODE_SEL_SHA1       0UL     /*!< SHA engine select SHA1                  \hideinitializer */
#define SHA_MODE_SEL_SHA2       0UL     /*!< SHA engine select SHA2                  \hideinitializer */
#define SHA_MODE_SEL_SHA3       1UL     /*!< SHA engine select SHA3                  \hideinitializer */
#define SHA_MODE_SEL_SM3        2UL     /*!< SHA engine select SM3                   \hideinitializer */
#define SHA_MODE_SEL_MD5        4UL     /*!< SHA engine select MD5                   \hideinitializer */

#define SHA_MODE_SHA1           0UL     /*!< SHA select SHA-1 160-bit                \hideinitializer */
#define SHA_MODE_SHA224         5UL     /*!< SHA select SHA-224 224-bit              \hideinitializer */
#define SHA_MODE_SHA256         4UL     /*!< SHA select SHA-256 256-bit              \hideinitializer */
#define SHA_MODE_SHA384         7UL     /*!< SHA select SHA-384 384-bit              \hideinitializer */
#define SHA_MODE_SHA512         6UL     /*!< SHA select SHA-512 512-bit              \hideinitializer */
#define SHA_MODE_SHAKE128       0UL     /*!< SHA select SHA-3 SHAKE128               \hideinitializer */
#define SHA_MODE_SHAKE256       1UL     /*!< SHA select SHA-3 SHAKE256               \hideinitializer */

#define HMAC_MODE_SHA224        (13UL)    /*!< HMAC select SHA-224 224-bit              \hideinitializer */
#define HMAC_MODE_SHA256        (12UL)    /*!< HMAC select SHA-256 256-bit              \hideinitializer */
#define HMAC_MODE_SHA384        (15UL)    /*!< HMAC select SHA-384 384-bit              \hideinitializer */
#define HMAC_MODE_SHA512        (14UL)    /*!< HMAC select SHA-512 512-bit              \hideinitializer */

#define SHA_NO_SWAP             0UL     /*!< SHA do not swap input and output data   \hideinitializer */
#define SHA_OUT_SWAP            1UL     /*!< SHA swap output data                    \hideinitializer */
#define SHA_IN_SWAP             2UL     /*!< SHA swap input data                     \hideinitializer */
#define SHA_IN_OUT_SWAP         3UL     /*!< SHA swap both input and output data     \hideinitializer */

#define CRYPTO_DMA_FIRST        0x4UL   /*!< Do first encrypt/decrypt in DMA cascade         \hideinitializer */
#define CRYPTO_DMA_ONE_SHOT     0x5UL   /*!< Do one shot encrypt/decrypt with DMA            \hideinitializer */
#define CRYPTO_DMA_CONTINUE     0x6UL   /*!< Do continuous encrypt/decrypt in DMA cascade    \hideinitializer */
#define CRYPTO_DMA_LAST         0x7UL   /*!< Do last encrypt/decrypt in DMA cascade          \hideinitializer */


#define RSA_MAX_KLEN            (4096)                  /*!< RSA Max key length                        \hideinitializer */
#define RSA_KBUF_HLEN           (RSA_MAX_KLEN/4 + 8)    /*!< RSA Max key length in halfword            \hideinitializer */
#define RSA_KBUF_BLEN           (RSA_MAX_KLEN + 32)     /*!< RSA Max key length in byte                \hideinitializer */

#define RSA_KEY_SIZE_1024       (0UL)     /*!< RSA select 1024-bit key length           \hideinitializer */
#define RSA_KEY_SIZE_2048       (1UL)     /*!< RSA select 2048-bit key length           \hideinitializer */
#define RSA_KEY_SIZE_3072       (2UL)     /*!< RSA select 3072-bit key length           \hideinitializer */
#define RSA_KEY_SIZE_4096       (3UL)     /*!< RSA select 4096-bit key length           \hideinitializer */

#define RSA_MODE_NORMAL         (0x000UL)     /*!< RSA select normal mode                \hideinitializer */
#define RSA_MODE_CRT            (0x004UL)     /*!< RSA select CRT mode                   \hideinitializer */
#define RSA_MODE_CRTBYPASS      (0x00CUL)     /*!< RSA select CRT bypass mode            \hideinitializer */
#define RSA_MODE_SCAP           (0x100UL)     /*!< RSA select SCAP mode                  \hideinitializer */
#define RSA_MODE_CRT_SCAP       (0x104UL)     /*!< RSA select CRT SCAP mode              \hideinitializer */
#define RSA_MODE_CRTBYPASS_SCAP (0x10CUL)     /*!< RSA select CRT bypass SCAP mode       \hideinitializer */
#define RSA_USE_KS              (1UL)         /*!< RSA use KeyStore                      \hideinitializer */
#define RSA_NOT_KS              (0UL)         /*!< RSA not use KeyStore                  \hideinitializer */

/** @} end of group CRYPTO_EXPORTED_CONSTANTS */

typedef enum
{
    /*!< ECC curve                \hideinitializer */
    CURVE_P_192  = 0x01,                /*!< ECC curve P-192          \hideinitializer */
    CURVE_P_224  = 0x02,                /*!< ECC curve P-224          \hideinitializer */
    CURVE_P_256  = 0x03,                /*!< ECC curve P-256          \hideinitializer */
    CURVE_P_384  = 0x04,                /*!< ECC curve P-384          \hideinitializer */
    CURVE_P_521  = 0x05,                /*!< ECC curve P-521          \hideinitializer */
    CURVE_K_163  = 0x11,                /*!< ECC curve K-163          \hideinitializer */
    CURVE_K_233  = 0x12,                /*!< ECC curve K-233          \hideinitializer */
    CURVE_K_283  = 0x13,                /*!< ECC curve K-283          \hideinitializer */
    CURVE_K_409  = 0x14,                /*!< ECC curve K-409          \hideinitializer */
    CURVE_K_571  = 0x15,                /*!< ECC curve K-571          \hideinitializer */
    CURVE_B_163  = 0x21,                /*!< ECC curve B-163          \hideinitializer */
    CURVE_B_233  = 0x22,                /*!< ECC curve B-233          \hideinitializer */
    CURVE_B_283  = 0x23,                /*!< ECC curve B-283          \hideinitializer */
    CURVE_B_409  = 0x24,                /*!< ECC curve B-409          \hideinitializer */
    CURVE_B_571  = 0x25,                /*!< ECC curve K-571          \hideinitializer */
    CURVE_KO_192 = 0x31,                /*!< ECC 192-bits "Koblitz" curve   \hideinitializer */
    CURVE_KO_224 = 0x32,                /*!< ECC 224-bits "Koblitz" curve   \hideinitializer */
    CURVE_KO_256 = 0x33,                /*!< ECC 256-bits "Koblitz" curve   \hideinitializer */
    CURVE_BP_256 = 0x41,                /*!< ECC Brainpool 256-bits curve   \hideinitializer */
    CURVE_BP_384 = 0x42,                /*!< ECC Brainpool 256-bits curve   \hideinitializer */
    CURVE_BP_512 = 0x43,                /*!< ECC Brainpool 256-bits curve   \hideinitializer */
    CURVE_SM2_256 = 0x50,               /*!< ECC SM2 curve            \hideinitializer */
    CURVE_25519  = 0x51,                /*!< ED25519 curve            \hideinitializer */
    CURVE_Ed448  = 0x52,                /*!< ED448   curve            \hideinitializer */
    CURVE_UNDEF,                        /*!< Invalid curve            \hideinitializer */
}
E_ECC_CURVE;                            /*!< ECC curve                \hideinitializer */

typedef struct e_curve_t
{
    E_ECC_CURVE  curve_id;
    int32_t   Echar;
    char  Ea[144];
    char  Eb[144];
    char  Px[144];
    char  Py[144];
    int32_t   Epl;
    char  Pp[176];
    int32_t   Eol;
    char  Eorder[176];
    int32_t   key_len;
    int32_t   irreducible_k1;
    int32_t   irreducible_k2;
    int32_t   irreducible_k3;
    int32_t   GF;
}  ECC_CURVE;

enum
{
    CURVE_GF_P,
    CURVE_GF_2M,
};

#define ECC_TIME_OUT            (300)   /* 3 seconds time-out */

/* RSA working buffer for normal mode */
typedef struct
{
    /**
    * @var RSA_BUF_NORMAL_T::au32RsaOutput[128]
    * The RSA answer.
    * ---------------------------------------------------------------------------------------------------
    * @var RSA_BUF_NORMAL_T::au32RsaN[128]
    * The base of modulus operation word.
    * ---------------------------------------------------------------------------------------------------
    * @var RSA_BUF_NORMAL_T::au32RsaM[128]
    * The base of exponentiation words.
    * ---------------------------------------------------------------------------------------------------
    * @var RSA_BUF_NORMAL_T::au32RsaE[128]
    * The exponent of exponentiation words.
    * ---------------------------------------------------------------------------------------------------
    */
    uint32_t au32RsaOutput[128];
    uint32_t au32RsaN[128];
    uint32_t au32RsaM[128];
    uint32_t au32RsaE[128];
} RSA_BUF_NORMAL_T;

/* RSA working buffer for CRT ( + CRT bypass) mode */
typedef struct
{
    uint32_t au32RsaOutput[128]; /* The RSA answer. */
    uint32_t au32RsaN[128]; /* The base of modulus operation word. */
    uint32_t au32RsaM[128]; /* The base of exponentiation words. */
    uint32_t au32RsaE[128]; /* The exponent of exponentiation words. */
    uint32_t au32RsaP[128]; /* The Factor of Modulus Operation. */
    uint32_t au32RsaQ[128]; /* The Factor of Modulus Operation. */
    uint32_t au32RsaTmpCp[128]; /* The Temporary Value(Cp) of RSA CRT. */
    uint32_t au32RsaTmpCq[128]; /* The Temporary Value(Cq) of RSA CRT. */
    uint32_t au32RsaTmpDp[128]; /* The Temporary Value(Dp) of RSA CRT. */
    uint32_t au32RsaTmpDq[128]; /* The Temporary Value(Dq) of RSA CRT. */
    uint32_t au32RsaTmpRp[128]; /* The Temporary Value(Rp) of RSA CRT. */
    uint32_t au32RsaTmpRq[128]; /* The Temporary Value(Rq) of RSA CRT. */
} RSA_BUF_CRT_T;

/* RSA working buffer for SCAP mode */
typedef struct
{
    uint32_t au32RsaOutput[128]; /* The RSA answer. */
    uint32_t au32RsaN[128]; /* The base of modulus operation word. */
    uint32_t au32RsaM[128]; /* The base of exponentiation words. */
    uint32_t au32RsaE[128]; /* The exponent of exponentiation words. */
    uint32_t au32RsaP[128]; /* The Factor of Modulus Operation. */
    uint32_t au32RsaQ[128]; /* The Factor of Modulus Operation. */
    uint32_t au32RsaTmpBlindKey[128]; /* The Temporary Value(blind key) of RSA SCAP. */
} RSA_BUF_SCAP_T;

/* RSA working buffer for CRT ( + CRT bypass ) + SCAP mode */
typedef struct
{
    uint32_t au32RsaOutput[128]; /* The RSA answer. */
    uint32_t au32RsaN[128]; /* The base of modulus operation word. */
    uint32_t au32RsaM[128]; /* The base of exponentiation words. */
    uint32_t au32RsaE[128]; /* The exponent of exponentiation words. */
    uint32_t au32RsaP[128]; /* The Factor of Modulus Operation. */
    uint32_t au32RsaQ[128]; /* The Factor of Modulus Operation. */
    uint32_t au32RsaTmpCp[128]; /* The Temporary Value(Cp) of RSA CRT. */
    uint32_t au32RsaTmpCq[128]; /* The Temporary Value(Cq) of RSA CRT. */
    uint32_t au32RsaTmpDp[128]; /* The Temporary Value(Dp) of RSA CRT. */
    uint32_t au32RsaTmpDq[128]; /* The Temporary Value(Dq) of RSA CRT. */
    uint32_t au32RsaTmpRp[128]; /* The Temporary Value(Rp) of RSA CRT. */
    uint32_t au32RsaTmpRq[128]; /* The Temporary Value(Rq) of RSA CRT. */
    uint32_t au32RsaTmpBlindKey[128]; /* The Temporary Value(blind key) of RSA SCAP. */
} RSA_BUF_CRT_SCAP_T;

/* RSA working buffer for using key store */
typedef struct
{
    uint32_t au32RsaOutput[128]; /* The RSA answer. */
    uint32_t au32RsaN[128]; /* The base of modulus operation word. */
    uint32_t au32RsaM[128]; /* The base of exponentiation words. */
} RSA_BUF_KS_T;

/** @addtogroup CRYPTO_EXPORTED_MACROS
  @{
*/
/**
  * @brief This macro enables PRNG interrupt.
  * @param crypto     Specified crypto module
  * @return None
  * \hideinitializer
  */
#define PRNG_ENABLE_INT(crypto)       ((crypto)->INTEN |= CRYPTO_INTEN_PRNGIEN_Msk)

/**
  * @brief This macro disables PRNG interrupt.
  * @param crypto     Specified crypto module
  * @return None
  * \hideinitializer
  */
#define PRNG_DISABLE_INT(crypto)      ((crypto)->INTEN &= ~CRYPTO_INTEN_PRNGIEN_Msk)

/**
  * @brief This macro gets PRNG interrupt flag.
  * @param crypto     Specified crypto module
  * @return PRNG interrupt flag.
  * \hideinitializer
  */
#define PRNG_GET_INT_FLAG(crypto)     ((crypto)->INTSTS & CRYPTO_INTSTS_PRNGIF_Msk)

/**
  * @brief This macro clears PRNG interrupt flag.
  * @param crypto     Specified crypto module
  * @return None
  * \hideinitializer
  */
#define PRNG_CLR_INT_FLAG(crypto)     ((crypto)->INTSTS = CRYPTO_INTSTS_PRNGIF_Msk)

/**
  * @brief This macro enables AES interrupt.
  * @param crypto     Specified crypto module
  * @return None
  * \hideinitializer
  */
#define AES_ENABLE_INT(crypto)        ((crypto)->INTEN |= (CRYPTO_INTEN_AESIEN_Msk|CRYPTO_INTEN_AESEIEN_Msk))

/**
  * @brief This macro disables AES interrupt.
  * @param crypto     Specified crypto module
  * @return None
  * \hideinitializer
  */
#define AES_DISABLE_INT(crypto)       ((crypto)->INTEN &= ~(CRYPTO_INTEN_AESIEN_Msk|CRYPTO_INTEN_AESEIEN_Msk))

/**
  * @brief This macro gets AES interrupt flag.
  * @param crypto     Specified crypto module
  * @return AES interrupt flag.
  * \hideinitializer
  */
#define AES_GET_INT_FLAG(crypto)      ((crypto)->INTSTS & (CRYPTO_INTSTS_AESIF_Msk|CRYPTO_INTSTS_AESEIF_Msk))

/**
  * @brief This macro clears AES interrupt flag.
  * @param crypto     Specified crypto module
  * @return None
  * \hideinitializer
  */
#define AES_CLR_INT_FLAG(crypto)      ((crypto)->INTSTS = (CRYPTO_INTSTS_AESIF_Msk|CRYPTO_INTSTS_AESEIF_Msk))

/**
  * @brief This macro enables AES key protection.
  * @param crypto     Specified crypto module
  * @return None
  * \hideinitializer
  */
#define AES_ENABLE_KEY_PROTECT(crypto)  ((crypto)->AES_CTL |= CRYPTO_AES_CTL_KEYPRT_Msk)

/**
  * @brief This macro disables AES key protection.
  * @param crypto     Specified crypto module
  * @return None
  * \hideinitializer
  */
#define AES_DISABLE_KEY_PROTECT(crypto) ((crypto)->AES_CTL = ((crypto)->AES_CTL & ~CRYPTO_AES_CTL_KEYPRT_Msk) | (0x16UL<<CRYPTO_AES_CTL_KEYUNPRT_Pos)); \
    ((crypto)->AES_CTL &= ~CRYPTO_AES_CTL_KEYPRT_Msk)

/**
  * @brief This macro enables SHA interrupt.
  * @param crypto     Specified crypto module
  * @return None
  * \hideinitializer
  */
#define SHA_ENABLE_INT(crypto)        ((crypto)->INTEN |= (CRYPTO_INTEN_HMACIEN_Msk|CRYPTO_INTEN_HMACEIEN_Msk))

/**
  * @brief This macro disables SHA interrupt.
  * @param crypto     Specified crypto module
  * @return None
  * \hideinitializer
  */
#define SHA_DISABLE_INT(crypto)       ((crypto)->INTEN &= ~(CRYPTO_INTEN_HMACIEN_Msk|CRYPTO_INTEN_HMACEIEN_Msk))

/**
  * @brief This macro gets SHA interrupt flag.
  * @param crypto     Specified crypto module
  * @return SHA interrupt flag.
  * \hideinitializer
  */
#define SHA_GET_INT_FLAG(crypto)      ((crypto)->INTSTS & (CRYPTO_INTSTS_HMACIF_Msk|CRYPTO_INTSTS_HMACEIF_Msk))

/**
  * @brief This macro clears SHA interrupt flag.
  * @param crypto     Specified crypto module
  * @return None
  * \hideinitializer
  */
#define SHA_CLR_INT_FLAG(crypto)      ((crypto)->INTSTS = (CRYPTO_INTSTS_HMACIF_Msk|CRYPTO_INTSTS_HMACEIF_Msk))

/**
  * @brief This macro enables ECC interrupt.
  * @param crypto     Specified crypto module
  * @return None
  * \hideinitializer
  */
#define ECC_ENABLE_INT(crypto)        ((crypto)->INTEN |= (CRYPTO_INTEN_ECCIEN_Msk|CRYPTO_INTEN_ECCEIEN_Msk))

/**
  * @brief This macro disables ECC interrupt.
  * @param crypto     Specified crypto module
  * @return None
  * \hideinitializer
  */
#define ECC_DISABLE_INT(crypto)       ((crypto)->INTEN &= ~(CRYPTO_INTEN_ECCIEN_Msk|CRYPTO_INTEN_ECCEIEN_Msk))

/**CRYPTO
  * @brief This macro gets ECC interrupt flag.
  * @param crypto     Specified crypto module
  * @return ECC interrupt flag.
  * \hideinitializer
  */
#define ECC_GET_INT_FLAG(crypto)      ((crypto)->INTSTS & (CRYPTO_INTSTS_ECCIF_Msk|CRYPTO_INTSTS_ECCEIF_Msk))

/**
  * @brief This macro clears ECC interrupt flag.
  * @param crypto     Specified crypto module
  * @return None
  * \hideinitializer
  */
#define ECC_CLR_INT_FLAG(crypto)      ((crypto)->INTSTS = (CRYPTO_INTSTS_ECCIF_Msk|CRYPTO_INTSTS_ECCEIF_Msk))

/**
  * @brief This macro enables RSA interrupt.
  * @param crypto     Specified crypto module
  * @return None
  * \hideinitializer
  */
#define RSA_ENABLE_INT(crypto)        ((crypto)->INTEN |= (CRYPTO_INTEN_RSAIEN_Msk|CRYPTO_INTEN_RSAEIEN_Msk))

/**
  * @brief This macro disables RSA interrupt.
  * @param crypto     Specified crypto module
  * @return None
  * \hideinitializer
  */
#define RSA_DISABLE_INT(crypto)       ((crypto)->INTEN &= ~(CRYPTO_INTEN_RSAIEN_Msk|CRYPTO_INTEN_RSAEIEN_Msk))

/**
  * @brief This macro gets RSA interrupt flag.
  * @param crypto     Specified crypto module
  * @return ECC interrupt flag.
  * \hideinitializer
  */
#define RSA_GET_INT_FLAG(crypto)      ((crypto)->INTSTS & (CRYPTO_INTSTS_RSAIF_Msk|CRYPTO_INTSTS_RSAEIF_Msk))

/**
  * @brief This macro clears RSA interrupt flag.
  * @param crypto     Specified crypto module
  * @return None
  * \hideinitializer
  */
#define RSA_CLR_INT_FLAG(crypto)      ((crypto)->INTSTS = (CRYPTO_INTSTS_RSAIF_Msk|CRYPTO_INTSTS_RSAEIF_Msk))
/** @} end of group CRYPTO_EXPORTED_MACROS */


/** @addtogroup CRYPTO_EXPORTED_FUNCTIONS
  @{
*/
void PRNG_Open(CRYPTO_T *crypto, uint32_t u32KeySize, uint32_t u32SeedReload, uint32_t u32Seed);
void PRNG_Start(CRYPTO_T *crypto);
void PRNG_Read(CRYPTO_T *crypto, uint32_t u32RandKey[]);

void AES_Open(CRYPTO_T *crypto, uint32_t u32Channel, uint32_t u32EncDec, uint32_t u32OpMode, uint32_t u32KeySize, uint32_t u32SwapType);
void AES_Start(CRYPTO_T *crypto, uint32_t u32Channel, uint32_t u32DMAMode);
void AES_SetKey(CRYPTO_T *crypto, uint32_t u32Channel, uint32_t au32Keys[], uint32_t u32KeySize);
void AES_SetInitVect(CRYPTO_T *crypto, uint32_t u32Channel, uint32_t au32IV[]);
void AES_SetDMATransfer(CRYPTO_T *crypto, uint32_t u32Channel, uint32_t u32SrcAddr, uint32_t u32DstAddr, uint32_t u32TransCnt);
void AES_Start_KS(CRYPTO_T *crypto, uint32_t u32Channel, uint32_t u32DMAMode, int ksel, int knum);
void AES_SetKey_KS(CRYPTO_T *crypto, KS_MEM_Type mem, int32_t i32KeyIdx);

void SHA_Open(CRYPTO_T *crypto, uint32_t u32OpMode, uint32_t u32SwapType, uint32_t hmac_key_len);
void SHA_Start(CRYPTO_T *crypto, uint32_t u32DMAMode);
void SHA_SetDMATransfer(CRYPTO_T *crypto, uint32_t u32SrcAddr, uint32_t u32TransCnt);
void SHA_Read(CRYPTO_T *crypto, uint32_t u32Digest[]);

void ECC_Enable_DFAP(void);
void ECC_Complete(CRYPTO_T *crypto);
int  ECC_IsPrivateKeyValid(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve,  char private_k[]);
int32_t ECC_GeneratePublicKey(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, char *private_k, char public_k1[], char public_k2[]);
int32_t ECC_Mutiply(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, char x1[], char y1[], char *k, char x2[], char y2[]);
int32_t ECC_GenerateSecretZ(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, char *private_k, char public_k1[], char public_k2[], char secret_z[]);
int32_t ECC_GenerateSignature(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, char *message, char *d, char *k, char *R, char *S);
int32_t ECC_VerifySignature(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, char *message, char *public_k1, char *public_k2, char *R, char *S);
int32_t ECC_Write_N(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve);
int32_t ECC_GeneratePublicKey_KS(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, int k_ksnum, int is_ecdh, char public_k1[], char public_k2[]);
int32_t ECC_Mutiply_KS(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, int x1_ksnum, char x1[], int y1_ksnum, char y1[], int k_ksnum, char *k, char x2[], char y2[]);
int32_t ECC_GenerateSecretZ_KS(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, int k_ksnum, char *private_k, char public_x[], char public_y[], int z_to_ks, int z_owner, char secret_z[]);
int32_t ECC_GenerateSignature_KS(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, char *message, int d_ksnum, int k_ksnum, char *R, char *S);
int32_t ECC_VerifySignature_KS(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, char *message, int x_ksnum, int y_ksnum, char *R, char *S);
int32_t ECC_GenerateSignature_KS(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, char *message,  int d_ksnum, int k_ksnum, char *R, char *S);
int32_t ECC_GetCurve(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, ECC_CURVE *curve);
int32_t ECC_VerifySignature_KS(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, char *message, int x_ksnum, int y_ksnum, char *R, char *S);
void Hex2Reg(char input[], uint32_t volatile reg[]);
void Reg2Hex(int32_t count, uint32_t volatile reg[], char output[]);
void Hex2RegEx(char input[], uint32_t volatile reg[], int shift);
int32_t ECC_GetCurve(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, ECC_CURVE *curve);
int ecc_strcmp(char *s1, char *s2);
int32_t  ECC_VerifySignature_KS(CRYPTO_T *crypto, E_ECC_CURVE ecc_curve, char *message, int x_ksnum, int y_ksnum, char *R, char *S);
int32_t RSA_Open(CRYPTO_T *crypto, uint32_t u32OpMode, uint32_t u32KeySize, void *psRSA_Buf, uint32_t u32BufSize, uint32_t u32UseKS);
int32_t RSA_SetKey(CRYPTO_T *crypto, char *Key);
int32_t RSA_SetDMATransfer(CRYPTO_T *crypto, char *Src, char *n, char *P, char *Q);
void RSA_Start(CRYPTO_T *crypto);
int32_t RSA_Read(CRYPTO_T *crypto, char *Output);
int32_t RSA_SetKey_KS(CRYPTO_T *crypto, uint32_t u32KeyNum, uint32_t u32KSMemType, uint32_t u32BlindKeyNum);
int32_t RSA_SetDMATransfer_KS(CRYPTO_T *crypto, char *Src, char *n, uint32_t u32PNum,
                              uint32_t u32QNum, uint32_t u32CpNum, uint32_t u32CqNum, uint32_t u32DpNum,
                              uint32_t u32DqNum, uint32_t u32RpNum, uint32_t u32RqNum);
/** @} end of group CRYPTO_EXPORTED_FUNCTIONS */

/** @} end of group CRYPTO_Driver */

/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif  /* __CRYPTO_H__ */
