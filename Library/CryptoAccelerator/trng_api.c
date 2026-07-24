/*
 * Copyright (c) 2001-2019, Arm Limited and Contributors. All rights reserved.
 * Copyright (c) 2022 Nuvoton Technology Corp. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stddef.h>
#include "NuMicro.h"
#include "rng.h"
#ifndef ARG_UNUSED
    #define ARG_UNUSED(arg)  ((void)(arg))
#endif

int mbedtls_hardware_poll(void *data, unsigned char *output, size_t len, size_t *olen);

int mbedtls_hardware_poll(void *data, unsigned char *output, size_t len, size_t *olen)
{
    ARG_UNUSED(data);

    if (NULL == output)
    {
        return -1;
    }

    if (NULL == olen)
    {
        return -1;
    }

    if (0U == len)
    {
        return -1;
    }

    /* Generate the seed by TRNG */
    (void)RNG_Open();

    /* Get TRNG generated random number */
    *olen =  RNG_EntropyPoll((uint8_t *)output, len);
    return 0;
}
