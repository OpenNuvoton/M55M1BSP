/* Copyright 2014 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* Atomic operations for ARMv6-M */

#ifndef __CROS_EC_ATOMIC_H
#define __CROS_EC_ATOMIC_H
#include "NuMicro.h"
#include "common.h"

//#include "core_cm23.h"
#if 0
    typedef int atomic_t;
    typedef atomic_t atomic_val_t;
#else
    typedef uint32_t atomic_t;
    typedef uint32_t atomic_val_t; //atomic_t atomic_val_t;

    typedef uint64_t atomic_64_t;
    typedef uint64_t atomic_val_64_t; //atomic_t atomic_val_t;
#endif

/**
 * Implements atomic arithmetic operations on 32-bit integers.
 *
 * There is no load/store exclusive on ARMv6-M, just disable interrupts
 */
#define bic 0
#define orr 1
#define add 2
#define sub 3


static inline uint64_t ATOMIC_OP(uint32_t operation, atomic_t *addr, atomic_val_t value)
{
    //    int was_masked = __disable_irq();        //We are not in RTOS environment
    switch (operation)
    {
        case orr:
            *addr |= value;
            break;

        case add:
            *addr += value;
            break;

        case sub:
            *addr -= value;
            break;

        case bic:
            *addr &= ~value;
            break;
    }

    //    if (!was_masked)                //We are not in RTOS environment
    //        __enable_irq();
    return *addr;

}
//#endif

static inline uint64_t ATOMIC_OP64(uint32_t operation, atomic_64_t *addr, atomic_val_64_t value)
{
    //    int was_masked = __disable_irq();        //We are not in RTOS environment
    switch (operation)
    {
        case orr:
            *addr |= value;
            break;

        case add:
            *addr += value;
            break;

        case sub:
            *addr -= value;
            break;

        case bic:
            *addr &= ~value;
            break;
    }

    //    if (!was_masked)                //We are not in RTOS environment
    //        __enable_irq();
    return *addr;

}

static inline void atomic_clear_bits(atomic_t *addr, atomic_val_t bits)
{
    ATOMIC_OP(bic, addr, bits);
}
static inline void atomic_clear_64bits(atomic_64_t *addr, atomic_val_64_t bits)
{
    ATOMIC_OP64(bic, addr, bits);
}

static inline atomic_val_t atomic_or(atomic_t *addr, atomic_val_t bits)
{
    return ATOMIC_OP(orr, addr, bits);
}
static inline atomic_val_t atomic_or64(atomic_64_t *addr, atomic_val_64_t bits)
{
    return ATOMIC_OP64(orr, addr, bits);
}

static inline atomic_val_t atomic_add(atomic_t *addr, atomic_val_t value)
{
    return ATOMIC_OP(add, addr, value);
}

static inline atomic_val_t atomic_sub(atomic_t *addr, atomic_val_t value)
{
    return ATOMIC_OP(sub, addr, value);
}

static inline atomic_val_t atomic_clear(atomic_t *addr)
{
    atomic_t ret;

    //    int was_masked = __disable_irq();      //We are not in RTOS environment

    ret = *addr;
    *addr = 0;

    /* ... */
    //    if (!was_masked)                          //We are not in RTOS environment
    //        __enable_irq();
    return ret;
}
//#endif


#endif  /* __CROS_EC_ATOMIC_H */
