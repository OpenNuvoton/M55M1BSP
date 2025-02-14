/**************************************************************************//**
 * @file     NuBL_crypto.c
 * @version  V1.00
 * @brief    NuBL crypto source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "NuBL2.h"
#include "NuBL_crypto.h"

#define printf(...)

#if 0
static uint32_t Swap32(uint32_t val)
{
    return (val << 24) | ((val << 8) & 0xff0000) | ((val >> 8) & 0xff00) | (val >> 24);
}
#endif

/**
  * @brief      Open SHA encrypt function
  * @param[in]  start       SHA encrypt start address
  * @param[in]  end         SHA encrypt end address
  * @param[out] digest      The SHA encrypt output digest
  * @param[in]  mode        SHA operation mode, including:
  *                             - \ref SHA_ONESHOT
  *                             - \ref SHA_CONTI_START
  *                             - \ref SHA_CONTI_ING
  *                             - \ref SHA_CONTI_END
  * @retval     0           Success
  * @retval     -1          Failed
  */
int32_t NuBL_CalculateSHA256(uint32_t start, uint32_t end, uint32_t digest[], E_SHA_OP_MODE mode, E_SHA_SRC src)
{
    int32_t  i, bytes;
    uint32_t *ptr, addr, data = 0, u32TimeOutCnt;

    bytes   = (int32_t)(end - start);
    ptr     = (uint32_t *)start;
    addr    = (uint32_t)ptr;

    if ((mode == SHA_ONESHOT) || (mode == SHA_CONTI_START))
    {
        NUBL_MSG("\n[Start SHA256 from 0x%x to 0x%x. (size: %d) (mode: %d)] (%s)\n", start, end, bytes, mode, (src == SHA_SRC_SRAM) ? "SRAM" : "Flash");
        CRYPTO->HMAC_CTL = (SHA_MODE_SHA256 << CRYPTO_HMAC_CTL_OPMODE_Pos) | CRYPTO_HMAC_CTL_INSWAP_Msk | CRYPTO_HMAC_CTL_OUTSWAP_Msk;
        CRYPTO->HMAC_DMACNT = 64;
        CRYPTO->HMAC_CTL |= CRYPTO_HMAC_CTL_START_Msk;
    }
    else
    {
        NUBL_MSG("[Continue SHA256 from 0x%x to 0x%x. (size: %d) (mode: %d)]\n", start, end, bytes, mode);
    }

    /* Start to calculate ... */
    while (bytes > 0)
    {
        if (bytes < 64)
            CRYPTO->HMAC_DMACNT = (uint32_t)bytes;

        if (CRYPTO->HMAC_STS & CRYPTO_HMAC_STS_DATINREQ_Msk)
        {
            if (src == SHA_SRC_SRAM)
            {
                data = *ptr++;
                bytes -= 4;
            }
            else if (src == SHA_SRC_FLASH)
            {
                data = FMC_Read(addr & 0x0FFFFFFF);
                addr += 4;
                bytes -= 4;
            }

            if (bytes <= 0)
                bytes = 0;

            /* bytes means remain byte counts */
            if (bytes != 0)
            {
                CRYPTO->HMAC_DATIN = data;
            }
            else
            {
                if ((mode == SHA_ONESHOT) || (mode == SHA_CONTI_END)) /* The last SHA operation */
                {
                    /* It's last word ... *-* */
                    CRYPTO->HMAC_CTL |= CRYPTO_HMAC_CTL_START_Msk | CRYPTO_HMAC_CTL_DMALAST_Msk;
                    CRYPTO->HMAC_DATIN = data;
                    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

                    while (CRYPTO->HMAC_STS & CRYPTO_HMAC_STS_BUSY_Msk)
                    {
                        if (--u32TimeOutCnt == 0)
                            return -1;
                    }

                    for (i = 0; i < 8; i++)
                        digest[i] = *(uint32_t *)((uint32_t) & (CRYPTO->HMAC_DGST[0]) + ((uint32_t)i * 4));

                    NUBL_MSG("\t[SHA256 done]\n");
                    break;
                }
                else
                {
                    CRYPTO->HMAC_DATIN = data;
                    /* Exit to wait next SHA operation */
                }
            }
        }
    }

    return 0;
    /*
        Verify:
            Input:
                {0x30313233,0x34353637} or String "32107654"
            Result:
                6952CF8EACE972CD4F10567331B46D85104E9E57402364F205876D13F84F7E42
                ==> Arraty {0x6952CF8E, 0xACE972CD, 0x4F105673....}
    */
}

/**
  * @brief      Perform AES-256 CFB NoPadding decrypt
  */
int32_t NuBL_AES256Decrypt(uint32_t *in, uint32_t *out, uint32_t len, uint32_t *KEY)
{
    uint32_t au32AESIV[4] = {0}, u32TimeOutCnt;

    /* KEY and IV are byte order (32 bit) reversed, __REV(x)) and stored in ISP_INFO_T */
    memcpy((void *)((uint32_t)&CRYPTO->AES_KEY[0]), KEY, (4 * 8));
    memcpy((void *)((uint32_t)&CRYPTO->AES_IV[0]), au32AESIV, (4 * 4));

    CRYPTO->AES_SADDR = (uint32_t)in;
    CRYPTO->AES_DADDR = (uint32_t)out;
    CRYPTO->AES_CNT   = len;
    CRYPTO->AES_CTL = ((AES_KEY_SIZE_256 << CRYPTO_AES_CTL_KEYSZ_Pos) | (AES_IN_OUT_SWAP << CRYPTO_AES_CTL_OUTSWAP_Pos));
    CRYPTO->AES_CTL |= ((AES_MODE_CFB << CRYPTO_AES_CTL_OPMODE_Pos) | CRYPTO_AES_CTL_START_Msk | CRYPTO_AES_CTL_DMAEN_Msk);
    //    CRYPTO->AES_CTL |= ((AES_MODE_ECB << CRYPTO_AES_CTL_OPMODE_Pos) | CRYPTO_AES_CTL_START_Msk | CRYPTO_AES_CTL_DMAEN_Msk);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (CRYPTO->AES_STS & CRYPTO_AES_STS_BUSY_Msk)
    {
        if (--u32TimeOutCnt == 0)
            return -1;
    }

    return 0;
}


/*-----------------------------------------------------------------------------------------------*/
/*                                                                                               */
/*    ECC                                                                                        */
/*                                                                                               */
/*-----------------------------------------------------------------------------------------------*/
#define ECC_CURVE_TYPE      CURVE_P_256
#define CURVE_P_SIZE        CURVE_P_256
#define ECC_KEY_SIZE        256 /* bits */

#define ECCOP_POINT_MUL     (0x0UL << CRYPTO_ECC_CTL_ECCOP_Pos)
#define ECCOP_MODULE        (0x1UL << CRYPTO_ECC_CTL_ECCOP_Pos)
#define ECCOP_POINT_ADD     (0x2UL << CRYPTO_ECC_CTL_ECCOP_Pos)
#define ECCOP_POINT_DOUBLE  (0x0UL << CRYPTO_ECC_CTL_ECCOP_Pos)

#define MODOP_DIV           (0x0UL << CRYPTO_ECC_CTL_MODOP_Pos)
#define MODOP_MUL           (0x1UL << CRYPTO_ECC_CTL_MODOP_Pos)
#define MODOP_ADD           (0x2UL << CRYPTO_ECC_CTL_MODOP_Pos)
#define MODOP_SUB           (0x3UL << CRYPTO_ECC_CTL_MODOP_Pos)

#define B2C(c)              (((uint8_t)(c)<10)? ((uint8_t)(c)+'0'):((uint8_t)(c)-10+'a'))
#if 0
static char ch2hex(char ch)
{
    if (ch <= '9')
    {
        return ch - '0';
    }
    else if ((ch <= 'z') && (ch >= 'a'))
    {
        return ch - 'a' + 10U;
    }
    else
    {
        return ch - 'A' + 10U;
    }
}

static void Hex2Reg(char input[], uint32_t volatile reg[])
{
    char      hex;
    int       si, ri;
    uint32_t  i, val32;

    si = (int)strlen(input) - 1;
    ri = 0;

    while (si >= 0)
    {
        val32 = 0UL;

        for (i = 0UL; (i < 8UL) && (si >= 0); i++)
        {
            hex = ch2hex(input[si]);
            val32 |= (uint32_t)hex << (i * 4UL);
            si--;
        }

        reg[ri++] = val32;
    }
}

static int  get_nibble_value(char c)
{
    char ch;

    if ((c >= '0') && (c <= '9'))
    {
        ch = '0';
        return ((int)c - (int)ch);
    }

    if ((c >= 'a') && (c <= 'f'))
    {
        ch = 'a';
        return ((int)c - (int)ch - 10);
    }

    if ((c >= 'A') && (c <= 'F'))
    {
        ch = 'A';
        return ((int)c - (int)ch - 10);
    }

    return 0;
}
#endif

/**
  * @brief      Initial ECC Curve P-256
  */
int32_t NuBL_ECCInitCurve(void)
{
    volatile int32_t i, ret = 0;

    /* Enable CRYPTO interrupt */
    ECC_ENABLE_INT(CRYPTO);
    /* Use ECC polling mode */
    NVIC_DisableIRQ(CRYPTO_IRQn);

    for (i = 0; i < 18; i++)
    {
        CRYPTO->ECC_A[i]  = 0UL;
        CRYPTO->ECC_B[i]  = 0UL;
        CRYPTO->ECC_X1[i] = 0UL;
        CRYPTO->ECC_Y1[i] = 0UL;
        CRYPTO->ECC_N[i]  = 0UL;
        CRYPTO->ECC_K[i]  = 0UL;
    }

    //char  Ea[144];
    //char  Eb[144];
    //char  Px[144];
    //char  Py[144];
    //"FFFFFFFF-00000001-00000000-00000000-00000000-FFFFFFFF-FFFFFFFF-FFFFFFFC",  /* "0000000000000000000000000000000000000000000000000000000000000003" */
    CRYPTO->ECC_A[0] = 0xFFFFFFFC;
    CRYPTO->ECC_A[1] = 0xFFFFFFFF;
    CRYPTO->ECC_A[2] = 0xFFFFFFFF;
    CRYPTO->ECC_A[3] = 0x00000000;
    CRYPTO->ECC_A[4] = 0x00000000;
    CRYPTO->ECC_A[5] = 0x00000000;
    CRYPTO->ECC_A[6] = 0x00000001;
    CRYPTO->ECC_A[7] = 0xFFFFFFFF;
    //"5ac635d8-aa3a93e7-b3ebbd55-769886bc-651d06b0-cc53b0f6-3bce3c3e-27d2604b",
    CRYPTO->ECC_B[0] = 0x27d2604b;
    CRYPTO->ECC_B[1] = 0x3bce3c3e;
    CRYPTO->ECC_B[2] = 0xcc53b0f6;
    CRYPTO->ECC_B[3] = 0x651d06b0;
    CRYPTO->ECC_B[4] = 0x769886bc;
    CRYPTO->ECC_B[5] = 0xb3ebbd55;
    CRYPTO->ECC_B[6] = 0xaa3a93e7;
    CRYPTO->ECC_B[7] = 0x5ac635d8;
    //"6b17d1f2-e12c4247-f8bce6e5-63a440f2-77037d81-2deb33a0-f4a13945-d898c296",
    CRYPTO->ECC_X1[0] = 0xd898c296;
    CRYPTO->ECC_X1[1] = 0xf4a13945;
    CRYPTO->ECC_X1[2] = 0x2deb33a0;
    CRYPTO->ECC_X1[3] = 0x77037d81;
    CRYPTO->ECC_X1[4] = 0x63a440f2;
    CRYPTO->ECC_X1[5] = 0xf8bce6e5;
    CRYPTO->ECC_X1[6] = 0xe12c4247;
    CRYPTO->ECC_X1[7] = 0x6b17d1f2;
    //"4fe342e2-fe1a7f9b-8ee7eb4a-7c0f9e16-2bce3357-6b315ece-cbb64068-37bf51f5",
    CRYPTO->ECC_Y1[0] = 0x37bf51f5;
    CRYPTO->ECC_Y1[1] = 0xcbb64068;
    CRYPTO->ECC_Y1[2] = 0x6b315ece;
    CRYPTO->ECC_Y1[3] = 0x2bce3357;
    CRYPTO->ECC_Y1[4] = 0x7c0f9e16;
    CRYPTO->ECC_Y1[5] = 0x8ee7eb4a;
    CRYPTO->ECC_Y1[6] = 0xfe1a7f9b;
    CRYPTO->ECC_Y1[7] = 0x4fe342e2;

    /* pCurve->GF == (int)CURVE_GF_P */

    //char  Pp[176];
    //"FFFFFFFF-00000001-00000000-00000000-00000000-FFFFFFFF-FFFFFFFF-FFFFFFFF",  /* "115792089210356248762697446949407573530086143415290314195533631308867097853951" */
    CRYPTO->ECC_N[0] = 0xFFFFFFFF;
    CRYPTO->ECC_N[1] = 0xFFFFFFFF;
    CRYPTO->ECC_N[2] = 0xFFFFFFFF;
    CRYPTO->ECC_N[3] = 0x00000000;
    CRYPTO->ECC_N[4] = 0x00000000;
    CRYPTO->ECC_N[5] = 0x00000000;
    CRYPTO->ECC_N[6] = 0x00000001;
    CRYPTO->ECC_N[7] = 0xFFFFFFFF;

    return ret;
}

/**
  * @brief      Generate ECDH key
  */
int32_t NuBL_ECCGenerateECDHKey(void)
{
    /* set FSEL (Field selection) */
    CRYPTO->ECC_CTL = CRYPTO_ECC_CTL_FSEL_Msk; /* CURVE_GF_P */

    ECC_CLR_INT_FLAG(CRYPTO);
    CRYPTO->ECC_CTL |= (ECC_KEY_SIZE << CRYPTO_ECC_CTL_CURVEM_Pos) |
                       (0x0UL << CRYPTO_ECC_CTL_ECCOP_Pos) | CRYPTO_ECC_CTL_START_Msk;

    while (1)
    {
        if (ECC_GET_INT_FLAG(CRYPTO) != 0x0)
        {
            if ((ECC_GET_INT_FLAG(CRYPTO)&CRYPTO_INTSTS_ECCEIF_Msk) == CRYPTO_INTSTS_ECCEIF_Msk)
            {
                NVIC_DisableIRQ(CRYPTO_IRQn);
                ECC_CLR_INT_FLAG(CRYPTO);
                return -1;
            }

            break;
        }
    }

    ECC_CLR_INT_FLAG(CRYPTO);

    return 0;
}

/*** (C) COPYRIGHT 2024 Nuvoton Technology Corp. ***/
