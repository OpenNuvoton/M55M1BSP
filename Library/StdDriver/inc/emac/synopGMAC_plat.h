/* ===================================================================================
 * Copyright (c) <2009> Synopsys, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software annotated with this license and associated documentation files
 * (the "Software"), to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * =================================================================================== */

/**\file
 *  This file serves as the wrapper for the platform/OS dependent functions
 *  It is needed to modify these functions accordingly based on the platform and the
 *  OS. Whenever the synopsys GMAC driver ported on to different platform, this file
 *  should be handled at most care.
 *  The corresponding function definitions for non-inline functions are available in
 *  synopGMAC_plat.c file.
 * \internal
 * -------------------------------------REVISION HISTORY---------------------------
 * Synopsys                 01/Aug/2007            Created
 */


#ifndef SYNOP_GMAC_PLAT_H
#define SYNOP_GMAC_PLAT_H 1

#include <stdio.h>
#include "NuMicro.h"

#define TR0(fmt, args...) printf("SynopGMAC: " fmt, ##args)

//#define DEBUG
#ifdef DEBUG
    #undef TR
    #define TR(fmt, args...) printf("SynopGMAC: " fmt, ##args)
#else
    #define TR(fmt, args...) /* not debugging: nothing */
#endif

typedef unsigned char   u8;         ///< Define 8-bit unsigned data type
typedef unsigned short  u16;        ///< Define 16-bit unsigned data type
typedef unsigned int    u32;        ///< Define 32-bit unsigned data type
typedef signed   int    s32;        ///< Define 32-bit signed data type
//typedef unsigned long long u64;
//typedef unsigned int    u64;

typedef u32 *dma_addr_t;

//typedef int bool;
//enum synopGMAC_boolean {
//    false = 0,
//    true = 1
//};

#define DEFAULT_DELAY_VARIABLE  10
#define DEFAULT_LOOP_VARIABLE   10000

/* Error Codes */
#define ESYNOPGMACNOERR   0
#define ESYNOPGMACNOMEM   1
#define ESYNOPGMACPHYERR  2
#define ESYNOPGMACBUSY    3


/**
  * These are the wrapper function prototypes for OS/platform related routines
  */
void plat_delay(u32 delay);
u32  synopGMACReadReg(u32 RegBase, u32 RegOffset);
void synopGMACWriteReg(u32 RegBase, u32 RegOffset, u32 RegData);
void synopGMACSetBits(u32 RegBase, u32 RegOffset, u32 BitPos);
void synopGMACClearBits(u32 RegBase, u32 RegOffset, u32 BitPos);
bool synopGMACCheckBits(u32 RegBase, u32 RegOffset, u32 BitPos);

#endif
