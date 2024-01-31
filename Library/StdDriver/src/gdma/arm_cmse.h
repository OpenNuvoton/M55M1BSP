/*
 * CMSE intrinsics, as defined in "ARMv8-M Security Extensions:
 * Requirements on Development Tools"
 *
 * Copyright 2016-2017 IAR Systems AB.
 */

#ifndef ARM_CMSE_H
#define ARM_CMSE_H

#ifndef _SYSTEM_BUILD
    #pragma system_include
#endif

#include <ycheck.h>
#include <intrinsics.h>
#include <stddef.h>
#include <stdint.h>

#ifndef __ICCARM__
    #error "This file can only be used with iccarm."
#endif

#ifdef __cplusplus
extern "C" {
#endif

#pragma language=save
#pragma language=extended

/* Do nothing unless bit 0 of __ARM_FEATURE_CMSE is set */
#if (__ARM_FEATURE_CMSE & 1) == 1

#if __ARM_BIG_ENDIAN
#pragma bitfields=disjoint_types
#endif

typedef union
{
    struct cmse_address_info
    {
        unsigned mpu_region: 8;
#if (__ARM_FEATURE_CMSE & 2) == 2
        unsigned sau_region: 8;
#else
        unsigned : 8;
#endif
        unsigned mpu_region_valid: 1;
#if (__ARM_FEATURE_CMSE & 2) == 2
        unsigned sau_region_valid: 1;
#else
        unsigned : 1;
#endif
        unsigned read_ok: 1;
        unsigned readwrite_ok: 1;
#if (__ARM_FEATURE_CMSE & 2) == 2
        unsigned nonsecure_read_ok: 1;
        unsigned nonsecure_readwrite_ok: 1;
        unsigned secure: 1;
        unsigned idau_region_valid: 1;
        unsigned idau_region: 8;
#else
        unsigned : 12;
#endif
    } flags;
    unsigned value;
} cmse_address_info_t;

#if __ARM_BIG_ENDIAN
#pragma bitfields=default /* Restores to default setting. */
#endif

/* 4.2 TT intrinsics */
__ATTRIBUTES cmse_address_info_t cmse_TT(void const *p);
__ATTRIBUTES cmse_address_info_t cmse_TTT(void const *p);
#define cmse_TT_fptr(fp) cmse_TT((void const *)fp)
#define cmse_TTT_fptr(fp) cmse_TTT((void const *)fp)

/* 4.3 Address range check intrinsic */
__ATTRIBUTES
void *cmse_check_address_range(void *p, size_t size, int flags);

#define CMSE_MPU_UNPRIV     4
#define CMSE_MPU_READWRITE  1
#define CMSE_MPU_READ       8

#define cmse_check_pointed_object(p, f) \
    __iar_cast_to(p, cmse_check_address_range(p, sizeof(*p), f))

/* 5.2 TT intrinsics for CMSE */

#if (__ARM_FEATURE_CMSE & 2) == 2
__ATTRIBUTES cmse_address_info_t cmse_TTA(void const *p);
__ATTRIBUTES cmse_address_info_t cmse_TTAT(void const *p);
#define cmse_TTA_fptr(fp) cmse_TTA((void const *)fp)
#define cmse_TTAT_fptr(fp) cmse_TTAT((void const *)fp)

#endif

/* 5.3 Address range check intrinsic for CMSE */

#if (__ARM_FEATURE_CMSE & 2) == 2
#define CMSE_AU_NONSECURE   2
#define CMSE_MPU_NONSECURE 16
#define CMSE_NONSECURE     18
#endif

/* 5.4.3 Security state of the caller */
#if (__ARM_FEATURE_CMSE & 2) == 2
/* Returns non-zero if entry function is called from non-secure state,
 * and zero otherwise
 */
#pragma inline=forced
__ATTRIBUTES
int cmse_nonsecure_caller(void)
{
    return (~__get_return_address()) & 1;
}
#endif

/* 5.6 Non-secure function pointer */
#if (__ARM_FEATURE_CMSE & 2) == 2
#define cmse_nsfptr_create(p)  (__iar_cast_to(p, ((unsigned long) (p) & ~1)))
#define cmse_is_nsfptr(p)      (!(((unsigned long) (p)) & 1))
#endif


#pragma inline
__ATTRIBUTES
cmse_address_info_t cmse_TT(void const *p)
{
    cmse_address_info_t r;
    r.value = __TT((unsigned long)p);
    return r;
}

#pragma inline
__ATTRIBUTES
cmse_address_info_t cmse_TTT(void const *p)
{
    cmse_address_info_t r;
    r.value = __TTT((unsigned long)p);
    return r;
}

#if (__ARM_FEATURE_CMSE & 2) == 2
#pragma inline
__ATTRIBUTES cmse_address_info_t cmse_TTA(void const *p)
{
    cmse_address_info_t r;
    r.value = __TTA((unsigned long)p);
    return r;
}
#endif

#if (__ARM_FEATURE_CMSE & 2) == 2
#pragma inline
__ATTRIBUTES
cmse_address_info_t cmse_TTAT(void const *p)
{
    cmse_address_info_t r;
    r.value = __TTAT((unsigned long)p);
    return r;
}
#endif


/* 7.1 Address range checking intrinsic */
#pragma inline
__ATTRIBUTES
void *cmse_check_address_range(void *p, size_t s, int flags)
{
    /* check if the range wraps around */
    if ((uintptr_t) p + s < (uintptr_t) p) return NULL;

    cmse_address_info_t permb, perme;

    char const *pb = (char const *) p;
    char const *pe = pb + s - 1;

    /* execute the right variant of the TT instructions */
    const uintptr_t bitDiff = (uintptr_t) pb ^ (uintptr_t) pe;

    if ((flags & 16 /* CMSE_MPU_NONSECURE */) == 0)
    {
        if ((flags & CMSE_MPU_UNPRIV) == 0)
        {
            permb = cmse_TT(pb);
            perme = (bitDiff < 32) ? permb : cmse_TT(pe);
        }
        else
        {
            permb = cmse_TTT(pb);
            perme = (bitDiff < 32) ? permb : cmse_TTT(pe);
        }
    }
    else
    {
#if (__ARM_FEATURE_CMSE & 2) == 2

        if ((flags & CMSE_MPU_UNPRIV) == 0)
        {
            permb = cmse_TTA(pb);
            perme = (bitDiff < 32) ? permb : cmse_TTA(pe);
        }
        else
        {
            permb = cmse_TTAT(pb);
            perme = (bitDiff < 32) ? permb : cmse_TTAT(pe);
        }

#else
        /* CMSE_MPU_NONSECURE is not supported */
        return NULL;
#endif
    }

    /* check that range does not cross MPU, SAU, or IDAU region boundaries */
    if (permb.value != perme.value) return NULL;

    /* check the permission on the range */
    if (flags & 2)   /* CMSE_AU_NONSECURE */
    {
#if (__ARM_FEATURE_CMSE & 2) == 2

        if (flags & CMSE_MPU_READWRITE)
        {
            if ((permb.value >> 21) & 1) return p;
        }
        else if (flags & CMSE_MPU_READ)
        {
            if ((permb.value >> 20) & 1) return p;
        }
        else
        {
            if (((permb.value >> 22) & 1) == 0) return p;
        }

#endif /* (__ARM_FEATURE_CMSE & 0x2) == 2 */
    }
    else
    {
        if (flags & CMSE_MPU_READWRITE)
        {
            if ((permb.value >> 19) & 1) return p;
        }
        else if (flags & CMSE_MPU_READ)
        {
            if ((permb.value >> 18) & 1) return p;
        }
    }

    return NULL;
}

#endif /* Bit 0 of __ARM_FEATURE_CMSE is set */

#pragma language=restore

#ifdef __cplusplus
}
#endif

#endif /* ARM_CMSE_H */
