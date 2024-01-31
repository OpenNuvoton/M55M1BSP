/*
 * This file is part of the coreboot project.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Originally imported from linux/include/asm-arm/io.h. This file has changed
 * substantially since then.
 */

#ifndef __ARCH_MMIO_H__
#define __ARCH_MMIO_H__

#include <stdint.h>
#include <barrier.h>
#include <lib_helpers.h>

static inline uint8_t read8(const void *addr)
{
    dmb();
    return *(volatile uint8_t *)addr;
}

static inline uint16_t read16(const void *addr)
{
    dmb();
    return *(volatile uint16_t *)addr;
}

static inline uint32_t read32(const void *addr)
{
    dmb();
    return *(volatile uint32_t *)addr;
}

static inline uint64_t read64(const void *addr)
{
    dmb();
    return *(volatile uint64_t *)addr;
}

static inline void write8(void *addr, uint8_t val)
{
    dmb();
    *(volatile uint8_t *)addr = val;
    dmb();
}

static inline void write16(void *addr, uint16_t val)
{
    dmb();
    *(volatile uint16_t *)addr = val;
    dmb();
}

static inline void write32(void *addr, uint32_t val)
{
    dmb();
    *(volatile uint32_t *)addr = val;
    dmb();
}

static inline void write64(void *addr, uint64_t val)
{
    dmb();
    *(volatile uint64_t *)addr = val;
    dmb();
}

#define ptr_to_u32(x)   ((uint32_t)((uint64_t)(x)))
#define u32_to_ptr32(x)   ((uint32_t *)((uint64_t)(x)))
#define u32_to_ptr8(x)   ((uint8_t *)((uint64_t)(x)))

/**
  * @brief Set a 32-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 32-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 32-bit aligned
  */
#define outpw   write32
#define outp32   write32
#define writel  write32
#define writew  write16
#define writeb  write8

/**
  * @brief Get a 32-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified I/O port
  * @note The input port must be 32-bit aligned
  */
#define inpw    read32
#define inp32   read32
#define readl   read32
#define readw   read16
#define readb   read8

#endif /* __ARCH_MMIO_H__ */
