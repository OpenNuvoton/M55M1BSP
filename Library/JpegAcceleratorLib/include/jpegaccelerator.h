/**************************************************************************//**
 * @file     jpegaccelerator.h
 * @version  V1.00
 * @brief    Libjpeg accelerator porting related header
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef  __JPEGACCELERATOR_H__
#define  __JPEGACCELERATOR_H__
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
void jsimd_idct_islow_helium(int16_t *indata_ptr, uint8_t *output_buf, int *quantptr);
void jsimd_idct_16x16_helium(int16_t *indata_ptr, uint8_t *output_buf, int *quantptr);
#ifdef WITH_JPEGACC
extern int16_t i16simdbuf[DCTSIZE2 + 8];
extern int16_t g_i16divisors_recp0[DCTSIZE2 * 4];
extern int16_t g_i16divisors_recp1[DCTSIZE2 * 4];
#endif
#ifdef  __cplusplus
}
#endif
#endif

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
