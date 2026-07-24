/**************************************************************************//**
 * @file     trng.c
 * @version  V1.00
 * @brief    TRNG driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
/** @cond HIDDEN_SYMBOLS */
#if defined ENABLE_DEBUG
    #define TRNG_DBGMSG   printf
#else
    #define TRNG_DBGMSG(...)   do { } while (0)       /* disable debug */
#endif
/** @endcond HIDDEN_SYMBOLS */

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup TRNG_Driver TRNG Driver
  @{
*/

/** @addtogroup TRNG_EXPORTED_FUNCTIONS TRNG Exported Functions
  @{
*/

/**
  * @brief Initialize TRNG hardware.
  * @return None
  */
int32_t TRNG_Open(void)
{
    int i;
    SYS_ResetModule((uint32_t)SYS_TRNG0RST);

    TRNG->CTL |= TRNG_CTL_LDOEN_Msk;

    /* Waiting for ready */
    while ((TRNG->STS & TRNG_STS_LDORDY_Msk) == 0U)
    {
        TRNG_DBGMSG("Waiting for ready\n");
    }

    TRNG->CTL |= (TRNG_CTL_NRST_Msk);

    TRNG->CTL |= (TRNG_CTL_TRNGEN_Msk);

    TRNG_DBGMSG("TRNG->STS0 0x%x \n", TRNG->STS);

    /* Waiting for ready */
    while ((TRNG->STS & TRNG_STS_TRNGRDY_Msk) == 0U)
    {
    }

    for (i = 0; i < 3; i++)
    {
        TRNG_DBGMSG("TRNG->STS: loop%d  0x%x \n", i, TRNG->STS);
    }

    if ((TRNG->STS & 0x70U) != 0x70U)
    {
        TRNG_DBGMSG("Entropy source test fail!\n");

        return -1;
    }

    return 0;
}


/**
  * @brief   Generate a 32-bits random number word.
  * @param[out]  u32RndNum    The output 32-bits word random number.
  *
  * @return  Success or time-out.
  * @retval  0   Success
  * @retval  -1  Time-out. TRNG hardware may not be enabled.
  */
int32_t TRNG_GenWord(uint32_t *u32RndNum)
{
    uint32_t i;
    uint32_t u32Reg;
    uint32_t timeout;

    *u32RndNum = 0U;

    u32Reg = TRNG->CTL;

    for (i = 0U; i < 4U; i++)
    {
        TRNG->CTL = TRNG_CTL_START_Msk | u32Reg;

        /* TRNG should generate one byte per 125*8 us */
        for (timeout = (CLK_GetHCLK0Freq() / 100U); timeout > 0U; timeout--)
        {
            if (TRNG->STS & TRNG_STS_DVIF_Msk)
            {
                break;
            }
        }

        if (timeout == 0U)
        {
            return -1;
        }

        *u32RndNum |= ((TRNG->DATA_OUT[0] & 0xFFU) << (i * 8U));

    }

    return 0;
}

/**
  * @brief   Generate a big number in binary format.
  * @param[out]  u8BigNum  The output big number.
  * @param[in]   i32Len    Request bit length of the output big number. It must be multiple of 8.
  *
  * @return  Success or time-out.
  * @retval  0   Success
  * @retval  -1  Time-out. TRNG hardware may not be enabled.
  */
int32_t TRNG_GenBignum(uint8_t u8BigNum[], int32_t i32Len)
{
    int32_t   i;
    uint32_t  u32Reg;
    uint32_t  timeout;
    u32Reg = TRNG->CTL;

    for (i = 0; i < (i32Len / 8); i++)
    {
        TRNG->CTL = TRNG_CTL_START_Msk | u32Reg;

        /* TRNG should generate one byte per 125*8 us */
        for (timeout = (CLK_GetHCLK0Freq() / 100U); timeout > 0U; timeout--)
        {
            if (TRNG->STS & TRNG_STS_DVIF_Msk)
            {
                break;
            }
        }

        if (timeout == 0U)
        {
            return -1;
        }

        u8BigNum[i] = (uint8_t)(TRNG->DATA_OUT[0] & 0xFFU);
    }

    return 0;
}

/**
  * @brief   Generate a big number in hex format.
  * @param[out]  cBigNumHex  Buffer of output hex format big number. Buffer size should be (i32Len/2)+1
  * @param[in]   i32Len      Request bit length of the output big number. It must be multiple of 8.
  *
  * @return  Success or time-out.
  * @retval  0   Success
  * @retval  -1  Time-out. TRNG hardware may not be enabled.
  */
int32_t TRNG_GenBignumHex(char cBigNumHex[], int32_t i32Len)
{
    int32_t   i;
    uint32_t  idx;
    uint32_t  u32Reg;
    uint32_t  timeout;

    u32Reg = TRNG->CTL;
    idx = 0U;

    for (i = 0; i < (i32Len / 8); i++)
    {
        uint32_t data;
        uint32_t u32Ch;

        TRNG->CTL = TRNG_CTL_START_Msk | u32Reg;

        /* TRNG should generate one byte per 125*8 us */
        for (timeout = (CLK_GetHCLK0Freq() / 100U); timeout > 0U; timeout--)
        {
            if (TRNG->STS & TRNG_STS_DVIF_Msk)
            {
                break;
            }
        }

        if (timeout == 0U)
        {
            return -1;
        }

        data = (TRNG->DATA_OUT[0] & 0xFFU);

        if (data >= 0xA0U)
        {
            u32Ch = ((data >> 4U) & 0xFU) - 10U + (uint32_t)'A';
            cBigNumHex[idx] = (char)u32Ch;
            idx++;
        }
        else
        {
            u32Ch = ((data >> 4U) & 0xFU) + (uint32_t)'0';
            cBigNumHex[idx] = (char)u32Ch;
            idx++;
        }

        data &= 0xFU;

        if (data >= 0xAU)
        {
            u32Ch = data - 10U + (uint32_t)'A';
            cBigNumHex[idx] = (char)u32Ch;
            idx++;
        }
        else
        {
            u32Ch = data + (uint32_t)'0';
            cBigNumHex[idx] = (char)u32Ch;
            idx++;
        }
    }

    cBigNumHex[idx] = '\0';
    return 0;
}

/** @} end of group TRNG_EXPORTED_FUNCTIONS */
/** @} end of group TRNG_Driver */
/** @} end of group Standard_Driver */
