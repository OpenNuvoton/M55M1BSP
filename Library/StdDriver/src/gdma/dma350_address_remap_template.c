/*
 * Copyright (c) 2022, Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "dma350_lib.h"

/* Remap struct for DMA350 */
/* begin-end: range of CPU addresses to be remapped.
 * offset: This value is added to the addresses within the range. */
static const struct dma350_remap_range_t dma350_address_remap_list[] =
{
    {.begin = 0x00000000, .end = 0x0FFFFFFF, .offset = 0x00000000},
    {.begin = 0x10000000, .end = 0xFFFFFFFF, .offset = 0x00000000}
};
const struct dma350_remap_list_t dma350_address_remap =
{
    .size = sizeof(dma350_address_remap_list) /
    sizeof(dma350_address_remap_list[0]),
    .map = dma350_address_remap_list
};
