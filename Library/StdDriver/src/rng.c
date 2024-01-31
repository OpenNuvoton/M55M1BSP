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
static void RNG_BasicConfig()
{
    uint32_t retry_count;

    /* TRNG module reset*/
    SYS_ResetModule(SYS_TRNG0RST);

    /* Enable LDOEN  */
    TRNG->CTL |= TRNG_CTL_LDOEN_Msk;

    /* Wait LDORDY */
    retry_count = 0;

    while ((TRNG->STS & TRNG_STS_LDORDY_Msk) == 0)
    {
        retry_count++;

        if (retry_count > 5) break;
    };

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
int32_t RNG_Open()
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
    CRYPTO->PRNG_CTL = (PRNG_KEY_SIZE_256 << CRYPTO_PRNG_CTL_KEYSZ_Pos) | CRYPTO_PRNG_CTL_START_Msk | CRYPTO_PRNG_CTL_SEEDRLD_Msk | (0 << CRYPTO_PRNG_CTL_SEEDSRC_Pos);

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

    /* Waiting for Busy */
    while (CRYPTO->PRNG_CTL & CRYPTO_PRNG_CTL_BUSY_Msk)
    {
        if (timeout-- < 0)
            return 0;
    }

    if (nWords > 8)
        nWords = 8;

    /* Trig to generate seed 256 bits random number */
    CRYPTO->PRNG_CTL = (PRNG_KEY_SIZE_256 << CRYPTO_PRNG_CTL_KEYSZ_Pos) | CRYPTO_PRNG_CTL_START_Msk | CRYPTO_PRNG_CTL_SEEDRLD_Msk | (0 << CRYPTO_PRNG_CTL_SEEDSRC_Pos);

    timeout = 0x10000;

    while (CRYPTO->PRNG_CTL & CRYPTO_PRNG_CTL_BUSY_Msk)
    {
        if (timeout-- < 0)
            return 0;
    }

    for (i = 0; i < nWords; i++)
    {
        pu32Buf[i] = CRYPTO->PRNG_KEY[i];
    }

    return nWords;
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
int32_t RNG_ECDSA_Init(uint32_t u32KeySize, uint32_t au32ECC_N[18])
{
    int32_t i;

    /* Initial TRNG and PRNG for random number */
    if (RNG_Open())
        return -1;

    /* It is necessary to set ECC_N for ECDSA */
    for (i = 0; i < 18; i++)
        CRYPTO->ECC_N[i] = au32ECC_N[i];

    CRYPTO->PRNG_KSCTL = (KS_OWNER_ECC << CRYPTO_PRNG_KSCTL_OWNER_Pos) |
                         CRYPTO_PRNG_KSCTL_ECDSA_Msk |
                         (CRYPTO_PRNG_KSCTL_WDST_Msk) |
                         (KS_SRAM << CRYPTO_PRNG_KSCTL_WSDST_Pos);

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
    CRYPTO->PRNG_CTL = (u32KeySize << CRYPTO_PRNG_CTL_KEYSZ_Pos) | CRYPTO_PRNG_CTL_START_Msk | (0 << CRYPTO_PRNG_CTL_SEEDSRC_Pos);

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
int32_t RNG_ECDH_Init(uint32_t u32KeySize, uint32_t au32ECC_N[18])
{
    int32_t i;

    /* Initial Random Number Generator */
    if (RNG_Open())
        return -1;

    /* It is necessary to set ECC_N for ECDSA */
    for (i = 0; i < 18; i++)
        CRYPTO->ECC_N[i] = au32ECC_N[i];

    CRYPTO->PRNG_KSCTL = (KS_OWNER_ECC << CRYPTO_PRNG_KSCTL_OWNER_Pos) |
                         (CRYPTO_PRNG_KSCTL_ECDH_Msk) |
                         (CRYPTO_PRNG_KSCTL_WDST_Msk) |
                         (KS_SRAM << CRYPTO_PRNG_KSCTL_WSDST_Pos);

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
    CRYPTO->PRNG_CTL = (u32KeySize << CRYPTO_PRNG_CTL_KEYSZ_Pos) | CRYPTO_PRNG_CTL_START_Msk | (0 << CRYPTO_PRNG_CTL_SEEDSRC_Pos);

    timeout = 0x10000;
    i = 0;

    while (CRYPTO->PRNG_CTL & CRYPTO_PRNG_CTL_BUSY_Msk)
    {
        if (i++ > timeout)
            return -1;
    }

    if (CRYPTO->PRNG_KSSTS & CRYPTO_PRNG_KSSTS_KCTLERR_Msk)
        return -1;

    return (CRYPTO->PRNG_KSSTS & CRYPTO_PRNG_KSCTL_NUM_Msk);
}


/**
 *  @brief      To generate entropy from hardware entropy source (TRNG)
 *
 *  @param[in]  pu32Out  Buffer pointer to store the random number in word
 *
 *  @param[in]  i32Len   The specified number of byte to get.
 *
 *  @return     -1       Failed
 *               Others  The bytes in pu8Out buffer
 *
 *  @details    The function is used to generate entropy from TRNG.
 */
int32_t RNG_EntropyPoll(uint32_t *pu32Out, int32_t i32Len)
{
    int32_t timeout;
    int32_t i;

    if ((TRNG->CTL & TRNG_STS_TRNGRDY_Msk) == 0)
    {
        /* TRNG is not in active */
        return -1;
    }

    /* Trigger entropy generate */
    TRNG->CTL |= (TRNG_CTL_TRNGEN_Msk | TRNG_CTL_DVIEN_Msk);

    for (i = 0; i < i32Len; i += 4)
    {
        /* Trigger entropy generate */
        TRNG->CTL |=  TRNG_CTL_START_Msk ;

        timeout = SystemCoreClock;

        while ((TRNG->CTL & TRNG_STS_DVIF_Msk) == 0)
        {
            if (timeout-- <= 0)
            {
                /* Timeout error */
                return -1;
            }
        }

        /* Get one word entroy */
        *pu32Out++ = (uint32_t)(TRNG->DATA_OUT[0]);
        CLK_SysTickDelay(100000);//Do NOT remove this line
    }

    return i32Len;
}

/**@}end of group RNG_EXPORTED_FUNCTIONS */

/**@}end of group RNG_Driver */

/**@}end of group Standard_Driver */

