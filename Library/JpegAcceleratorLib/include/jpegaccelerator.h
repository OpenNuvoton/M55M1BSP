/**************************************************************************//**
 * @file     simd_helium.h
 * @version  V1.00
 * @brief    Libjpeg simd porting related header
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef  __SIMD_HELIUM_H__
#define  __SIMD_HELIUM_H__
#include <arm_mve.h>
#include <stdlib.h>
#include <inttypes.h>
#include "jpeglib.h"

/****************************************************************************
 * Function Declaration
 ****************************************************************************/
#ifdef  __cplusplus
extern  "C" {
#endif
void jsimd_fdct_islow_helium(int16_t *data);
void jsimd_quantize_helium(JCOEFPTR coef_block, DCTELEM *divisors, DCTELEM *workspace);
#ifdef  __cplusplus
}
#endif
#endif

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
