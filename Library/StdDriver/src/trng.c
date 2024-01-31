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
    SYS_ResetModule(SYS_TRNG0RST);

    TRNG->CTL |= TRNG_CTL_LDOEN_Msk;

    /* Waiting for ready */
    while ((TRNG->STS & TRNG_STS_LDORDY_Msk) == 0)
    {
        printf("Waiting for ready\n");
    }

    TRNG->CTL |= (TRNG_CTL_NRST_Msk);

    TRNG->CTL |= (TRNG_CTL_TRNGEN_Msk);

    printf("TRNG->STS0 0x%x \n", TRNG->STS);

    /* Waiting for ready */
    while ((TRNG->STS & TRNG_STS_TRNGRDY_Msk) == 0);

    for (i = 0; i < 3; i++)
        printf("TRNG->STS: loop%d  0x%x \n", i, TRNG->STS);

    if ((TRNG->STS & 0x70) != 0x70)
    {
        printf("Entropy source test fail!\n");

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
    uint32_t   i, u32Reg, timeout;

    *u32RndNum = 0;

    u32Reg = TRNG->CTL;

    for (i = 0; i < 4; i++)
    {
        TRNG->CTL = TRNG_CTL_START_Msk | u32Reg;

        /* TRNG should generate one byte per 125*8 us */
        for (timeout = (CLK_GetHCLK0Freq() / 100); timeout > 0; timeout--)
        {
            if (TRNG->STS & TRNG_STS_DVIF_Msk)
                break;
        }

        if (timeout == 0)
            return -1;

        *u32RndNum |= ((TRNG->DATA_OUT[0] & 0xff) << i * 8);

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
    uint32_t   i, u32Reg, timeout;

    u32Reg = TRNG->CTL;

    for (i = 0; i < i32Len / 8; i++)
    {
        TRNG->CTL = TRNG_CTL_START_Msk | u32Reg;

        /* TRNG should generate one byte per 125*8 us */
        for (timeout = (CLK_GetHCLK0Freq() / 100); timeout > 0; timeout--)
        {
            if (TRNG->STS & TRNG_STS_DVIF_Msk)
                break;
        }

        if (timeout == 0)
            return -1;

        u8BigNum[i] = (TRNG->DATA_OUT[0] & 0xff);
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
    uint32_t   i, idx, u32Reg, timeout;
    uint32_t   data;

    u32Reg = TRNG->CTL;
    idx = 0;

    for (i = 0; i < i32Len / 8; i++)
    {
        TRNG->CTL = TRNG_CTL_START_Msk | u32Reg;

        /* TRNG should generate one byte per 125*8 us */
        for (timeout = (CLK_GetHCLK0Freq() / 100); timeout > 0; timeout--)
        {
            if (TRNG->STS & TRNG_STS_DVIF_Msk)
                break;
        }

        if (timeout == 0)
            return -1;

        data = (TRNG->DATA_OUT [0] & 0xff);

        if (data >= 0xA0)
            cBigNumHex[idx++] = ((data >> 4) & 0xf) - 10 + 'A';
        else
            cBigNumHex[idx++] = ((data >> 4) & 0xf) + '0';

        data &= 0xf;

        if (data >= 0xA)
            cBigNumHex[idx++] = data - 10 + 'A';
        else
            cBigNumHex[idx++] = data + '0';
    }

    cBigNumHex[idx] = 0;
    return 0;
}

/** @} end of group TRNG_EXPORTED_FUNCTIONS */
/** @} end of group TRNG_Driver */
/** @} end of group Standard_Driver */
