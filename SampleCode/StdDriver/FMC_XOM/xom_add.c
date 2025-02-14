/******************************************************************************
 * @file     xom_add.c
 * @version  V3.00
 * @brief    Show how to use XOM Lirbary
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

__attribute__((section(".xom"))) int32_t Lib_XOM_ADD(uint32_t a, uint32_t b)
{
    uint32_t c;
    c =  a + b;
    return c;
}
