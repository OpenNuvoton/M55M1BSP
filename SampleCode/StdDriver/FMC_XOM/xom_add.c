/******************************************************************************
 * @file     xom_add.c
 * @version  V3.01
 * @brief    Show how to use XOM Lirbary
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

int32_t Lib_XOM_ADD(uint32_t a, uint32_t b)
{
    uint32_t c;
    c =  a + b;
    return c;
}
