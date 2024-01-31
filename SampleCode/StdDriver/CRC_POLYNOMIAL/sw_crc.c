/**************************************************************************//**
 * @file     sw_crc.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 23/02/24 4:40p $
 * @brief    Implement CRC in CRC-CCITT mode and get the CRC checksum result.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

const int8_t direct = 1;
int8_t order = 16;
uint32_t g_polynom;
uint32_t crcinit = 0xFFFF;
uint32_t crcxor = 0x0;
int8_t refin = 0;
int8_t refout = 0;
uint32_t crcmask;
uint32_t crchighbit;
uint32_t crcinit_direct;
uint32_t crcinit_nondirect;
uint32_t crctab[256];

static uint32_t reflect(uint32_t crc, int bitnum)
{
    uint32_t i, j = 1, crcout = 0;

    for (i = (uint32_t)1 << (bitnum - 1); i; i >>= 1)
    {
        if (crc & i)
            crcout |= j;

        j <<= 1;
    }

    return (crcout);
}

static void generate_crc_table(void)
{
    int32_t i, j;
    uint32_t bit, crc;

    for (i = 0; i < 256; i++)
    {
        crc = (uint32_t)i;

        if (refin)
            crc = reflect(crc, 8);

        crc <<= order - 8;

        for (j = 0; j < 8; j++)
        {
            bit = crc & crchighbit;
            crc <<= 1;

            if (bit)
                crc ^= g_polynom;
        }

        if (refin)
            crc = reflect(crc, order);

        crc &= crcmask;
        crctab[i] = crc;
    }
}

static uint32_t crcbitbybit(uint8_t *p, uint32_t len, int8_t IsWrite1sCOM, int8_t IsCRC1sCOM)
{
    unsigned long i, j, c, bit;
    unsigned long crc = crcinit_direct;

    for (i = 0; i < len; i++)
    {
        c = (unsigned long) * p++;

        if (IsWrite1sCOM)
        {
            c = (~c) & 0xFF;
        }

        if (refin) c = reflect(c, 8);

        for (j = 0x80; j; j >>= 1)
        {
            bit = crc & crchighbit;
            crc <<= 1;

            if (c & j) bit ^= crchighbit;

            if (bit) crc ^= g_polynom;
        }
    }

    if (refout) crc = reflect(crc, order);

    crc ^= crcxor;
    crc &= crcmask;

    return (crc);
}

uint32_t CRC_SWResult(uint32_t mode, uint32_t polynom, uint32_t seed, uint8_t *string, uint32_t count, int8_t IsWrite1sCOM, int8_t IsWriteRVS, int8_t IsCRC1sCOM, int8_t IsCRCRVS)
{
    uint8_t i;
    uint32_t bit, crc, crc_result;

    if (mode == CRC_8)
        order = 8;
    else if (mode == CRC_16)
        order = 16;
    else if (mode == CRC_32)
        order = 32;

    g_polynom = polynom;
    crcinit = seed;

    refin = IsWriteRVS;
    refout = IsCRCRVS;

    crcmask = ((((uint32_t)1 << (order - 1)) - 1) << 1) | 1;
    crchighbit = (uint32_t)1 << (order - 1);
    crcxor = (IsCRC1sCOM) ? crcmask : 0;

    if ((order < 1) || (order > 32))
    {
        printf("ERROR, invalid order, it must be between 1..32.\n");
        return (0);
    }

    if (polynom != (polynom & crcmask))
    {
        printf("ERROR, invalid polynom.\n");
        return (0);
    }

    if (crcinit != (crcinit & crcmask))
    {
        printf("ERROR, invalid crcinit.\n");
        return (0);
    }

    if (crcxor != (crcxor & crcmask))
    {
        printf("ERROR, invalid crcxor.\n");
        return (0);
    }

    /* generate lookup table */
    generate_crc_table();

    /* compute missing initial CRC value ... direct always is 1 */
    crcinit_direct = crcinit;
    crc = crcinit;

    for (i = 0; i < order; i++)
    {
        bit = crc & 1;

        if (bit)
            crc ^= polynom;

        crc >>= 1;

        if (bit)
            crc |= crchighbit;
    }

    crcinit_nondirect = crc;

    crc_result = crcbitbybit((uint8_t *)string, count, IsWrite1sCOM, IsCRC1sCOM);

    return crc_result;
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
