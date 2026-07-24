/**************************************************************************//**
 * @file     rng.c
 * @version  V1.0
 * @brief    Show how to get true random number.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/* Macro to limit (left)shift amount to [0,31] for MISRA-C 2012 Rule 12.2 compliance */
#define RNG_LSHIFT32(val, pos)   ((uint32_t)(val) << ((uint32_t)(pos) & 0x1FU))

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup RNG_Driver RNG Driver
  @{
*/


/** @addtogroup RNG_EXPORTED_FUNCTIONS RNG Exported Functions
  @{
*/
/**
 *  @brief      Basic Configuration of TRNG and PRNG
 *
 *  @details    Set basic configurations for TRNG and PRNG. Make sure module clock is enabled before calling this function.
 */
static void RNG_BasicConfig(void)
{
    uint32_t retry_count;

    /* TRNG module reset*/
    SYS_ResetModule((uint32_t)SYS_TRNG0RST);

    /* Enable LDOEN  */
    TRNG->CTL |= TRNG_CTL_LDOEN_Msk;

    /* Wait LDORDY */
    retry_count = 0U;

    while ((TRNG->STS & TRNG_STS_LDORDY_Msk) == 0U)
    {
        retry_count++;

        if (retry_count > 5U)
        {
            break;
        }
    }

    /* Set NRST, then enable TRNGEN*/
    TRNG->CTL |= (TRNG_CTL_NRST_Msk);

    TRNG->CTL |= (TRNG_CTL_TRNGEN_Msk);

}




/**
 *  @brief      Open random number generator
 *
 *  @return      0  Successful
 *              -1  Failed
 *
 *  @details    The function is used to initialize PRNG ready to generate random number.
 */
int32_t RNG_Open(void)
{
    int32_t i;
    int32_t timeout = 0x1000000;

    RNG_BasicConfig();

    /* Waiting for PRNG busy */
    i = 0;


    while ((CRYPTO->PRNG_CTL & CRYPTO_PRNG_CTL_BUSY_Msk) == CRYPTO_PRNG_CTL_BUSY_Msk)
    {
        if (i++ > timeout)
        {
            /* PRNG busy timeout */
            return -1;
        }
    }

    /* Reload seed from TRNG only at first time */
    CRYPTO->PRNG_CTL = RNG_LSHIFT32(PRNG_KEY_SIZE_256, CRYPTO_PRNG_CTL_KEYSZ_Pos) | CRYPTO_PRNG_CTL_START_Msk | CRYPTO_PRNG_CTL_SEEDRLD_Msk | RNG_LSHIFT32(0, CRYPTO_PRNG_CTL_SEEDSRC_Pos);

    i = 0;


    while (CRYPTO->PRNG_CTL & CRYPTO_PRNG_CTL_BUSY_Msk)
    {
        if (i++ > timeout)
        {
            /* busy timeout */
            return -1;
        }
    }

    return 0;
}


/**
 *  @brief      Get random words
 *
 *  @param[in]  pu32Buf Buffer pointer to store the random number
 *
 *  @param[in]  nWords  Buffer size in word count. nWords must <= 8
 *
 *  @return     Word count of random number in buffer
 *
 *  @details    The function is used to generate random numbers
 */
int32_t RNG_Random(uint32_t *pu32Buf, int32_t nWords)
{
    int32_t i;
    int32_t timeout = 0x10000;
    int32_t nWordsLocal = nWords;

    /* Waiting for Busy */
    while (CRYPTO->PRNG_CTL & CRYPTO_PRNG_CTL_BUSY_Msk)
    {
        if (timeout-- < 0)
        {
            return 0;
        }
    }

    if (nWordsLocal > 8)
    {
        nWordsLocal = 8;
    }

    /* Trig to generate seed 256 bits random number */
    CRYPTO->PRNG_CTL = RNG_LSHIFT32(PRNG_KEY_SIZE_256, CRYPTO_PRNG_CTL_KEYSZ_Pos) | CRYPTO_PRNG_CTL_START_Msk | CRYPTO_PRNG_CTL_SEEDRLD_Msk | RNG_LSHIFT32(0U, CRYPTO_PRNG_CTL_SEEDSRC_Pos);

    timeout = 0x10000;


    while (CRYPTO->PRNG_CTL & CRYPTO_PRNG_CTL_BUSY_Msk)
    {
        if (timeout-- < 0)
        {
            return 0;
        }
    }

    for (i = 0; i < nWordsLocal; i++)
    {
        pu32Buf[i] = CRYPTO->PRNG_KEY[i];
    }

    return nWordsLocal;
}



/**
 *  @brief      Initial function for ECDSA key generator for Key Store
 *
 *  @param[in]  u32KeySize  It could be PRNG_KEY_SIZE_128 ~ PRNG_KEY_SIZE_571
 *
 *  @param[in]  au32ECC_N   The N value of specified ECC curve.
 *
 *  @return     -1      Failed
 *              Others  The key number in KS SRAM
 *
 *  @details    The function is initial funciton of RNG_ECDSA function.
 *              This funciton should be called before calling RNG_ECDSA().
 */
int32_t RNG_ECDSA_Init(uint32_t u32KeySize, const uint32_t au32ECC_N[18])
{
    int32_t i;

    (void)(u32KeySize);

    /* Initial TRNG and PRNG for random number */
    if (RNG_Open())
    {
        return -1;
    }

    /* It is necessary to set ECC_N for ECDSA */
    for (i = 0; i < 18; i++)
    {
        CRYPTO->ECC_N[i] = au32ECC_N[i];
    }

    CRYPTO->PRNG_KSCTL = RNG_LSHIFT32(KS_OWNER_ECC, CRYPTO_PRNG_KSCTL_OWNER_Pos) |
                         CRYPTO_PRNG_KSCTL_ECDSA_Msk |
                         (CRYPTO_PRNG_KSCTL_WDST_Msk) |
                         RNG_LSHIFT32(KS_SRAM, CRYPTO_PRNG_KSCTL_WSDST_Pos);

    return 0;
}


/**
 *  @brief      To generate a key to KS SRAM for ECDSA.
 *
 *  @param[in]  u32KeySize  It could be PRNG_KEY_SIZE_128 ~ PRNG_KEY_SIZE_571
 *
 *  @return     -1      Failed
 *              Others  The key number in KS SRAM
 *
 *  @details    The function is used to generate a key to KS SRAM for ECDSA.
 *              This key is necessary for ECDSA+Key Store function of ECC.
 */
int32_t RNG_ECDSA(uint32_t u32KeySize)
{

    int32_t timeout;
    int32_t i;

    /* Reload seed only at first time */
    CRYPTO->PRNG_CTL = RNG_LSHIFT32(u32KeySize, CRYPTO_PRNG_CTL_KEYSZ_Pos) | CRYPTO_PRNG_CTL_START_Msk | RNG_LSHIFT32(0U, CRYPTO_PRNG_CTL_SEEDSRC_Pos);

    timeout = 0x10000;
    i = 0;

    while (CRYPTO->PRNG_CTL & CRYPTO_PRNG_CTL_BUSY_Msk)
    {
        if (i++ > timeout)
        {
            return -1;
        }
    }

    if (CRYPTO->PRNG_KSSTS & CRYPTO_PRNG_KSSTS_KCTLERR_Msk)
    {
        return -1;
    }

    return (CRYPTO->PRNG_KSSTS & CRYPTO_PRNG_KSCTL_NUM_Msk);
}



/**
 *  @brief      Initial funciton for RNG_ECDH.
 *
 *  @param[in]  u32KeySize  It could be PRNG_KEY_SIZE_128 ~ PRNG_KEY_SIZE_571
 *
 *  @param[in]  au32ECC_N   The N value of specified ECC curve.
 *
 *  @return     -1      Failed
 *              Others  The key number in KS SRAM
 *
 *  @details    The function is initial function of RNG_ECDH.
 *
 */
int32_t RNG_ECDH_Init(uint32_t u32KeySize, const uint32_t au32ECC_N[18])
{
    int32_t i;

    (void)(u32KeySize);

    /* Initial Random Number Generator */
    if (RNG_Open())
    {
        return -1;
    }

    /* It is necessary to set ECC_N for ECDSA */
    for (i = 0; i < 18; i++)
    {
        CRYPTO->ECC_N[i] = au32ECC_N[i];
    }

    CRYPTO->PRNG_KSCTL = RNG_LSHIFT32(KS_OWNER_ECC, CRYPTO_PRNG_KSCTL_OWNER_Pos) |
                         (CRYPTO_PRNG_KSCTL_ECDH_Msk) |
                         (CRYPTO_PRNG_KSCTL_WDST_Msk) |
                         RNG_LSHIFT32(KS_SRAM, CRYPTO_PRNG_KSCTL_WSDST_Pos);

    return 0;
}


/**
 *  @brief      To generate a key to KS SRAM for ECDH.
 *
 *  @param[in]  u32KeySize  It could be PRNG_KEY_SIZE_128 ~ PRNG_KEY_SIZE_571
 *
 *  @return     -1      Failed
 *              Others  The key number in KS SRAM
 *
 *  @details    The function is used to generate a key to KS SRAM for ECDH.
 *              This key is necessary for ECDH+Key Store function of ECC.
 */
int32_t RNG_ECDH(uint32_t u32KeySize)
{
    int32_t timeout;
    int32_t i;

    /* Reload seed only at first time */
    CRYPTO->PRNG_CTL = RNG_LSHIFT32(u32KeySize, CRYPTO_PRNG_CTL_KEYSZ_Pos) | CRYPTO_PRNG_CTL_START_Msk | RNG_LSHIFT32(0U, CRYPTO_PRNG_CTL_SEEDSRC_Pos);

    timeout = 0x10000;
    i = 0;

    while (CRYPTO->PRNG_CTL & CRYPTO_PRNG_CTL_BUSY_Msk)
    {
        if (i++ > timeout)
        {
            return -1;
        }
    }

    if (CRYPTO->PRNG_KSSTS & CRYPTO_PRNG_KSSTS_KCTLERR_Msk)
    {
        return -1;
    }

    return (CRYPTO->PRNG_KSSTS & CRYPTO_PRNG_KSCTL_NUM_Msk);
}


/**
 *  @brief      To generate entropy(TRNG)
 *
 *  @param[in]  pu8Out  Buffer pointer to store the random number in byte
 *
 *  @param[in]  i32Len   The specified number of byte to get.
 *
 *  @return     -1       Failed
 *               Others  The bytes in pu8Out buffer
 *
 *  @details    The function is used to generate entropy from TRNG.
 */
int32_t RNG_EntropyPoll(uint8_t *pu8Out, int32_t i32Len)
{
    int32_t i;

    if ((TRNG->STS & TRNG_STS_TRNGRDY_Msk) == 0UL)
    {
        /* TRNG is not ready */
        return -1;
    }

    /* Trigger entropy generate */
    TRNG->CTL |= (TRNG_CTL_TRNGEN_Msk | (uint32_t)TRNG_CTL_MODE_OUTPUT_NRBG);

    for (i = 0; i < i32Len; i += 4)
    {
        int32_t timeout;
        int32_t j;
        int32_t i32Remain = i32Len - i;
        uint32_t u32Temp;

        /* Trigger entropy generate */
        TRNG->CTL |= TRNG_CTL_START_Msk;

        timeout = SystemCoreClock;
        __DSB();

        while ((TRNG->STS & TRNG_STS_DVIF_Msk) == 0UL)
        {
            if (timeout <= 0)
            {
                /* Timeout error */
                return -1;
            }

            timeout--;
        }

        /* Get one word entropy */
        u32Temp = TRNG->DATA_OUT[0];

        /* Write only the remaining bytes in the last iteration */
        if (i32Remain > 4)
        {
            i32Remain = 4;
        }

        for (j = 0; j < i32Remain; j++)
        {
            pu8Out[i + j] = (uint8_t)((u32Temp >> ((uint32_t)j * 8U)) & 0xFFU);
        }
    }

    return i32Len;
}

/**@}end of group RNG_EXPORTED_FUNCTIONS */

/**@}end of group RNG_Driver */

/**@}end of group Standard_Driver */

