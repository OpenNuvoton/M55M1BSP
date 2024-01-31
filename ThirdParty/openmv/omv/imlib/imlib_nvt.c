/**************************************************************************//**
 * @file     imlib_nvt.c
 * @version  V0.10
 * @brief    Nuvoton image processing function using SIMD/hardware accelerating
 * * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdlib.h>
#include <stdio.h>

#include "imlib.h"
#include "common.h"

#include <arm_mve.h>

	#define PIXELS_LOOP	16

	#define OFFSET_G3B5_LOW		0x0E0C0A0806040200
	#define OFFSET_G3B5_HIGH	0x1E1C1A1816141210
	#define OFFSET_R5G3_LOW		0x0F0D0B0907050301
	#define OFFSET_R5G3_HIGH	0x1F1D1B1917151311

	#define OFFSET_R_LOW	0x15120F0C09060300
	#define OFFSET_R_HIGH	0x2D2A2724211E1B18
	#define OFFSET_G_LOW	0x1613100D0A070401
	#define OFFSET_G_HIGH	0x2E2B2825221F1C19
	#define OFFSET_B_LOW	0x1714110E0B080502
	#define OFFSET_B_HIGH	0x2F2C292623201D1A


void imlib_nvt_RGB565toRGB888_SIMD(image_t *src, image_t *dst)
{

	uint8_t *pu8SrcData = src->data;
	uint8_t *pu8DestData = dst->data;
	
	if((src->pixfmt != PIXFORMAT_RGB565) || (dst->pixfmt != PIXFORMAT_RGB888))
		return;

	//for helium intrinsics
	int pixels = src->h * src->w;
	uint8x16_t offset_8_16_r5g3;
	uint8x16_t offset_8_16_g3b5;
	uint8x16_t offset_8_16_r;
	uint8x16_t offset_8_16_g;
	uint8x16_t offset_8_16_b;

	//create offset Q
#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
	offset_8_16_r5g3 = vcreateq_u8(OFFSET_R5G3_HIGH, OFFSET_R5G3_LOW);
	offset_8_16_g3b5 = vcreateq_u8(OFFSET_G3B5_HIGH, OFFSET_G3B5_LOW);
#else
	offset_8_16_r5g3 = vcreateq_u8(OFFSET_R5G3_LOW, OFFSET_R5G3_HIGH);
	offset_8_16_g3b5 = vcreateq_u8(OFFSET_G3B5_LOW, OFFSET_G3B5_HIGH);
#endif

#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
	offset_8_16_r = vcreateq_u8(OFFSET_R_HIGH, OFFSET_R_LOW);
	offset_8_16_g = vcreateq_u8(OFFSET_G_HIGH, OFFSET_G_LOW);
	offset_8_16_b = vcreateq_u8(OFFSET_B_HIGH, OFFSET_B_LOW);
#else
	offset_8_16_r = vcreateq_u8(OFFSET_R_LOW, OFFSET_R_HIGH);
	offset_8_16_g = vcreateq_u8(OFFSET_G_LOW, OFFSET_G_HIGH);
	offset_8_16_b = vcreateq_u8(OFFSET_B_LOW, OFFSET_B_HIGH);
#endif

	while(pixels >= PIXELS_LOOP)
	{
		uint8x16_t vsrc_8_16_r5g3;
		uint8x16_t vsrc_8_16_g3b5;

		uint8x16_t vdst_8_16_r;
		uint8x16_t vdst_8_16_g;
		uint8x16_t vdst_8_16_b;
		uint8x16_t vdst_8_16_temp;

		//Load RGB565 data from source buffer. Using vector scale-gater load
		vsrc_8_16_r5g3 = vldrbq_gather_offset(pu8SrcData, offset_8_16_r5g3);
		vsrc_8_16_g3b5 = vldrbq_gather_offset(pu8SrcData, offset_8_16_g3b5);

		vdst_8_16_r = vsrc_8_16_r5g3;
		vdst_8_16_temp = vshrq(vdst_8_16_r, 5);
		vdst_8_16_r = vorrq(vdst_8_16_r, vdst_8_16_temp);

		vdst_8_16_g = vshlq_n(vsrc_8_16_r5g3, 5);
		vdst_8_16_temp = vshrq(vsrc_8_16_g3b5, 5);
		vdst_8_16_temp = vshlq_n(vdst_8_16_temp, 2);
		vdst_8_16_g = vorrq(vdst_8_16_g, vdst_8_16_temp);
		vdst_8_16_temp = vshrq(vdst_8_16_g, 6);
		vdst_8_16_g = vorrq(vdst_8_16_g, vdst_8_16_temp);

		vdst_8_16_b = vshlq_n(vsrc_8_16_g3b5, 3);
		vdst_8_16_temp = vshrq(vdst_8_16_b, 5);
		vdst_8_16_b = vorrq(vdst_8_16_b, vdst_8_16_temp);


		vstrbq_scatter_offset(pu8DestData, offset_8_16_r, vdst_8_16_r);
		vstrbq_scatter_offset(pu8DestData, offset_8_16_g, vdst_8_16_g);
		vstrbq_scatter_offset(pu8DestData, offset_8_16_b, vdst_8_16_b);

		pu8DestData += PIXELS_LOOP*3;
		pu8SrcData += PIXELS_LOOP*2;
		pixels -= PIXELS_LOOP;
	}

	uint16_t u16PixelData; 
	uint16_t *pu16SrcData = (uint16_t *)pu8SrcData;

	for(int i = 0; i < pixels; i ++)
	{
		u16PixelData = *pu16SrcData;
		*(pu8DestData ++) = COLOR_RGB565_TO_R8(u16PixelData);
		*(pu8DestData ++) = COLOR_RGB565_TO_G8(u16PixelData);
		*(pu8DestData ++) = COLOR_RGB565_TO_B8(u16PixelData);
		pu16SrcData ++;
	}
}


void imlib_nvt_RGB888toRGB565_SIMD(image_t *src, image_t *dst)
{

	uint8_t *pu8SrcData = src->data;
	uint8_t *pu8DestData = dst->data;
	
	if((src->pixfmt != PIXFORMAT_RGB888) || (dst->pixfmt != PIXFORMAT_RGB565))
		return;
	
	//for helium intrinsics
	int pixels = src->h * src->w;
	uint8x16_t offset_8_16_r;
	uint8x16_t offset_8_16_g;
	uint8x16_t offset_8_16_b;
	
	//create offset Q
#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
	offset_8_16_r = vcreateq_u8(OFFSET_R_HIGH, OFFSET_R_LOW);
	offset_8_16_g = vcreateq_u8(OFFSET_G_HIGH, OFFSET_G_LOW);
	offset_8_16_b = vcreateq_u8(OFFSET_B_HIGH, OFFSET_B_LOW);
#else
	offset_8_16_r = vcreateq_u8(OFFSET_R_LOW, OFFSET_R_HIGH);
	offset_8_16_g = vcreateq_u8(OFFSET_G_LOW, OFFSET_G_HIGH);
	offset_8_16_b = vcreateq_u8(OFFSET_B_LOW, OFFSET_B_HIGH);
#endif
	
	while(pixels >= PIXELS_LOOP)
	{
		uint8x16_t vsrc_8_16_r;
		uint8x16_t vsrc_8_16_g;
		uint8x16_t vsrc_8_16_b;

		uint16x8x2_t vdst_16_8_2;

		//Load RGB data from source buffer. Using vector scale-gater load
		vsrc_8_16_r = vldrbq_gather_offset(pu8SrcData, offset_8_16_r);
		vsrc_8_16_g = vldrbq_gather_offset(pu8SrcData, offset_8_16_g);
		vsrc_8_16_b = vldrbq_gather_offset(pu8SrcData, offset_8_16_b);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
		uint16x8_t vsrc_16_8_r;
		uint16x8_t vsrc_16_8_g;

		vsrc_16_8_r = vmovlbq(vsrc_8_16_r);
		vsrc_16_8_r = vshrq(vsrc_16_8_r, 3);
		vsrc_16_8_g = vmovlbq(vsrc_8_16_g);
		vsrc_16_8_g = vshrq(vsrc_16_8_g, 2);
		vdst_16_8_2.val[0] = vmovlbq(vsrc_8_16_b);
		vdst_16_8_2.val[0] = vshrq(vdst_16_8_2.val[0], 3);

		vdst_16_8_2.val[0] = vsliq(vdst_16_8_2.val[0], vsrc_16_8_g, 5);
		vdst_16_8_2.val[0] = vsliq(vdst_16_8_2.val[0], vsrc_16_8_r, 11);
#else
		vdst_16_8_2.val[0] = vshllbq(vsrc_8_16_r, 8);
		vdst_16_8_2.val[0] = vsriq(vdst_16_8_2.val[0], vshllbq(vsrc_8_16_g, 8), 5);
		vdst_16_8_2.val[0] = vsriq(vdst_16_8_2.val[0], vshllbq(vsrc_8_16_b, 8), 11);
#endif

#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
		vsrc_16_8_r = vmovltq(vsrc_8_16_r);
		vsrc_16_8_r = vshrq(vsrc_16_8_r, 3);
		vsrc_16_8_g = vmovltq(vsrc_8_16_g);
		vsrc_16_8_g = vshrq(vsrc_16_8_g, 2);
		vdst_16_8_2.val[1] = vmovltq(vsrc_8_16_b);
		vdst_16_8_2.val[1] = vshrq(vdst_16_8_2.val[1], 3);

		vdst_16_8_2.val[1] = vsliq(vdst_16_8_2.val[1], vsrc_16_8_g, 5);
		vdst_16_8_2.val[1] = vsliq(vdst_16_8_2.val[1], vsrc_16_8_r, 11);
#else
		vdst_16_8_2.val[1] = vshlltq(vsrc_8_16_r, 8);
		vdst_16_8_2.val[1] = vsriq(vdst_16_8_2.val[1], vshlltq(vsrc_8_16_g, 8), 5);
		vdst_16_8_2.val[1] = vsriq(vdst_16_8_2.val[1], vshlltq(vsrc_8_16_b, 8), 11);
#endif

		//Store RGB565 16 pixel data. Using vector interleave store. 
		vst2q_u16((uint16_t*)pu8DestData, vdst_16_8_2);
		
		pu8DestData += PIXELS_LOOP*2;
		pu8SrcData += PIXELS_LOOP*3;
		pixels -= PIXELS_LOOP;
	}

	uint16_t *pu16DestData = (uint16_t *)pu8DestData;
	
	for(int i = 0; i < pixels; i ++)
	{
		*pu16DestData = COLOR_R8_G8_B8_TO_RGB565(pu8SrcData[0], pu8SrcData[1], pu8SrcData[2]);
		pu16DestData ++;
		pu8SrcData +=3;
	}	
}

void imlib_nvt_RGB565toRGB888_SW(image_t *src, image_t *dst)
{
	uint16_t *pu16SrcData = (uint16_t *)src->data;
	uint8_t *pu8DestData = dst->data;
	int pixels = src->h * src->w;
	uint16_t u16PixelData; 

	if((src->pixfmt != PIXFORMAT_RGB565) || (dst->pixfmt != PIXFORMAT_RGB888))
		return;
	
	for(int i = 0; i < pixels; i ++)
	{
		u16PixelData = *pu16SrcData;
		*(pu8DestData ++) = COLOR_RGB565_TO_R8(u16PixelData);
		*(pu8DestData ++) = COLOR_RGB565_TO_G8(u16PixelData);
		*(pu8DestData ++) = COLOR_RGB565_TO_B8(u16PixelData);
		pu16SrcData ++;
	}
}


static int gcd(int n1, int n2) {
    if (n2 != 0)
        return gcd(n2, n1 % n2);
    else
        return n1;
}

#define UTIL_DDA(FactorM, FactorN, Delta, ReptCnt)  { ReptCnt = (Delta == 0); \
                                                      Delta += FactorM; \
                                                      while (Delta > FactorN) \
                                                      { ReptCnt++; Delta -= FactorN; } \
                                                    }


static void RGB565toRGB888_16Pixels_SIMD(
	image_t *src,
	image_t *dst,
	uint8_t *pu8SrcData,
	uint8_t *pu8DstData,
	uint16_t au16RepeatePos[],
	uint32_t u32YRepeateCnt
)
{
	uint64_t u64Offset_R5G3_Lo = 0;
	uint64_t u64Offset_R5G3_Hi = 0;
	uint64_t u64Offset_G3B5_Lo = 0;
	uint64_t u64Offset_G3B5_Hi = 0;
	uint8x16_t offset_8_16_r5g3;
	uint8x16_t offset_8_16_g3b5;
	uint8x16_t offset_8_16_r;
	uint8x16_t offset_8_16_g;
	uint8x16_t offset_8_16_b;

	int i;
	
	uint64_t u64Temp;

	for(i = 0; i < 8 ; i ++)
	{
		u64Temp = au16RepeatePos[i] * src->bpp;
		u64Offset_G3B5_Lo |= u64Temp << (i * 8);
		u64Offset_R5G3_Lo |= (u64Temp + 1) << (i * 8);
	}

	for( i = 0; i < 8 ; i ++)
	{
		u64Temp = au16RepeatePos[i + 8] * src->bpp;
		u64Offset_G3B5_Hi |= u64Temp << (i * 8);
		u64Offset_R5G3_Hi |= (u64Temp + 1) << (i * 8);
	}

	//create offset Q
#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
	offset_8_16_r5g3 = vcreateq_u8(u64Offset_R5G3_Hi, u64Offset_R5G3_Lo);
	offset_8_16_g3b5 = vcreateq_u8(u64Offset_G3B5_Hi, u64Offset_G3B5_Lo);
#else
	offset_8_16_r5g3 = vcreateq_u8(u64Offset_R5G3_Lo, u64Offset_R5G3_Hi);
	offset_8_16_g3b5 = vcreateq_u8(u64Offset_G3B5_Lo, u64Offset_G3B5_Hi);
#endif

#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
	offset_8_16_r = vcreateq_u8(OFFSET_R_HIGH, OFFSET_R_LOW);
	offset_8_16_g = vcreateq_u8(OFFSET_G_HIGH, OFFSET_G_LOW);
	offset_8_16_b = vcreateq_u8(OFFSET_B_HIGH, OFFSET_B_LOW);
#else
	offset_8_16_r = vcreateq_u8(OFFSET_R_LOW, OFFSET_R_HIGH);
	offset_8_16_g = vcreateq_u8(OFFSET_G_LOW, OFFSET_G_HIGH);
	offset_8_16_b = vcreateq_u8(OFFSET_B_LOW, OFFSET_B_HIGH);
#endif
	
	
	uint8x16_t vsrc_8_16_r5g3;
	uint8x16_t vsrc_8_16_g3b5;

	uint8x16_t vdst_8_16_r;
	uint8x16_t vdst_8_16_g;
	uint8x16_t vdst_8_16_b;
	uint8x16_t vdst_8_16_temp;
	
	//Load RGB565 data from source buffer. Using vector scale-gater load
	vsrc_8_16_r5g3 = vldrbq_gather_offset(pu8SrcData, offset_8_16_r5g3);
	vsrc_8_16_g3b5 = vldrbq_gather_offset(pu8SrcData, offset_8_16_g3b5);

	vdst_8_16_r = vsrc_8_16_r5g3;
	vdst_8_16_temp = vshrq(vdst_8_16_r, 5);
	vdst_8_16_r = vorrq(vdst_8_16_r, vdst_8_16_temp);

	vdst_8_16_g = vshlq_n(vsrc_8_16_r5g3, 5);
	vdst_8_16_temp = vshrq(vsrc_8_16_g3b5, 5);
	vdst_8_16_temp = vshlq_n(vdst_8_16_temp, 2);
	vdst_8_16_g = vorrq(vdst_8_16_g, vdst_8_16_temp);
	vdst_8_16_temp = vshrq(vdst_8_16_g, 6);
	vdst_8_16_g = vorrq(vdst_8_16_g, vdst_8_16_temp);

	vdst_8_16_b = vshlq_n(vsrc_8_16_g3b5, 3);
	vdst_8_16_temp = vshrq(vdst_8_16_b, 5);
	vdst_8_16_b = vorrq(vdst_8_16_b, vdst_8_16_temp);

	while(u32YRepeateCnt)
	{
		//Store RGB888 16 pixel data. 
		vstrbq_scatter_offset(pu8DstData, offset_8_16_r, vdst_8_16_r);
		vstrbq_scatter_offset(pu8DstData, offset_8_16_g, vdst_8_16_g);
		vstrbq_scatter_offset(pu8DstData, offset_8_16_b, vdst_8_16_b);

		pu8DstData += dst->w * dst->bpp;
		u32YRepeateCnt --;
	}
}
													
static void RGB888toRGB565_16Pixels_SIMD(
	image_t *src,
	image_t *dst,
	uint8_t *pu8SrcData,
	uint8_t *pu8DstData,
	uint16_t au16RepeatePos[],
	uint32_t u32YRepeateCnt
)
{
	uint64_t u64Offset_R_Lo = 0;
	uint64_t u64Offset_R_Hi = 0;
	uint64_t u64Offset_G_Lo = 0;
	uint64_t u64Offset_G_Hi = 0;
	uint64_t u64Offset_B_Lo = 0;
	uint64_t u64Offset_B_Hi = 0;
	uint8x16_t offset_8_16_r;
	uint8x16_t offset_8_16_g;
	uint8x16_t offset_8_16_b;
	
	int i;
	
	uint64_t u64Temp;

	for(i = 0; i < 8 ; i ++)
	{
		u64Temp = au16RepeatePos[i] * src->bpp;
		u64Offset_R_Lo |= u64Temp << (i * 8);
		u64Offset_G_Lo |= (u64Temp + 1) << (i * 8);
		u64Offset_B_Lo |= (u64Temp + 2) << (i * 8);
	}

	for( i = 0; i < 8 ; i ++)
	{
		u64Temp = au16RepeatePos[i + 8] * src->bpp;
		u64Offset_R_Hi |= u64Temp << (i * 8);
		u64Offset_G_Hi |= (u64Temp + 1) << (i * 8);
		u64Offset_B_Hi |= (u64Temp + 2) << (i * 8);
	}

	//create offset Q
#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
	offset_8_16_r = vcreateq_u8(u64Offset_R_Hi, u64Offset_R_Lo);
	offset_8_16_g = vcreateq_u8(u64Offset_G_Hi, u64Offset_G_Lo);
	offset_8_16_b = vcreateq_u8(u64Offset_B_Hi, u64Offset_B_Lo);
#else
	offset_8_16_r = vcreateq_u8(u64Offset_R_Lo, u64Offset_R_Hi);
	offset_8_16_g = vcreateq_u8(u64Offset_G_Lo, u64Offset_G_Hi);
	offset_8_16_b = vcreateq_u8(u64Offset_B_Lo, u64Offset_B_Hi);
#endif

	uint8x16_t vsrc_8_16_r;
	uint8x16_t vsrc_8_16_g;
	uint8x16_t vsrc_8_16_b;

	uint16x8x2_t vdst_16_8_2;
	
	//Load RGB data from source buffer. Using vector scale-gater load
	vsrc_8_16_r = vldrbq_gather_offset(pu8SrcData, offset_8_16_r);
	vsrc_8_16_g = vldrbq_gather_offset(pu8SrcData, offset_8_16_g);
	vsrc_8_16_b = vldrbq_gather_offset(pu8SrcData, offset_8_16_b);

#if defined (__GNUC__) && !defined(__ARMCC_VERSION)
	uint16x8_t vsrc_16_8_r;
	uint16x8_t vsrc_16_8_g;

	vsrc_16_8_r = vmovlbq(vsrc_8_16_r);
	vsrc_16_8_r = vshrq(vsrc_16_8_r, 3);
	vsrc_16_8_g = vmovlbq(vsrc_8_16_g);
	vsrc_16_8_g = vshrq(vsrc_16_8_g, 2);
	vdst_16_8_2.val[0] = vmovlbq(vsrc_8_16_b);
	vdst_16_8_2.val[0] = vshrq(vdst_16_8_2.val[0], 3);

	vdst_16_8_2.val[0] = vsliq(vdst_16_8_2.val[0], vsrc_16_8_g, 5);
	vdst_16_8_2.val[0] = vsliq(vdst_16_8_2.val[0], vsrc_16_8_r, 11);
#else
	vdst_16_8_2.val[0] = vshllbq(vsrc_8_16_r, 8);
	vdst_16_8_2.val[0] = vsriq(vdst_16_8_2.val[0], vshllbq(vsrc_8_16_g, 8), 5);
	vdst_16_8_2.val[0] = vsriq(vdst_16_8_2.val[0], vshllbq(vsrc_8_16_b, 8), 11);
#endif

#if defined (__GNUC__) && !defined(__ARMCC_VERSION)

	vsrc_16_8_r = vmovltq(vsrc_8_16_r);
	vsrc_16_8_r = vshrq(vsrc_16_8_r, 3);
	vsrc_16_8_g = vmovltq(vsrc_8_16_g);
	vsrc_16_8_g = vshrq(vsrc_16_8_g, 2);
	vdst_16_8_2.val[1] = vmovltq(vsrc_8_16_b);
	vdst_16_8_2.val[1] = vshrq(vdst_16_8_2.val[1], 3);

	vdst_16_8_2.val[1] = vsliq(vdst_16_8_2.val[1], vsrc_16_8_g, 5);
	vdst_16_8_2.val[1] = vsliq(vdst_16_8_2.val[1], vsrc_16_8_r, 11);
#else
	vdst_16_8_2.val[1] = vshlltq(vsrc_8_16_r, 8);
	vdst_16_8_2.val[1] = vsriq(vdst_16_8_2.val[1], vshlltq(vsrc_8_16_g, 8), 5);
	vdst_16_8_2.val[1] = vsriq(vdst_16_8_2.val[1], vshlltq(vsrc_8_16_b, 8), 11);
#endif

	while(u32YRepeateCnt)
	{
		//Store RGB565 16 pixel data. Using vector interleave store. 
		vst2q_u16((uint16_t*)pu8DstData, vdst_16_8_2);
		pu8DstData += dst->w * dst->bpp;
		u32YRepeateCnt --;
	}

}
													
static void RGB888toRGB565_SW(
	image_t *src,
	image_t *dst,
	uint8_t *pu8SrcData,
	uint8_t *pu8DstData,
	uint16_t au16RepeatePos[],
	uint16_t u32RepetePosCnt,
	uint32_t u32YRepeateCnt
)
{
	uint16_t *pu16DestData = (uint16_t *)pu8DstData;
	uint8_t *pu8RGB888Data;

	for(int i = 0; i < u32RepetePosCnt; i ++)
	{
		uint8_t u8Temp;
		uint16_t u16RGB565Data;
		u8Temp = au16RepeatePos[i];
		pu8RGB888Data = pu8SrcData + (u8Temp * src->bpp);
		u16RGB565Data = COLOR_R8_G8_B8_TO_RGB565(pu8RGB888Data[0], pu8RGB888Data[1], pu8RGB888Data[2]);
		
		for(int j = 0 ; j < u32YRepeateCnt; j ++)
		{
			*(pu16DestData + (j * dst->w))=  u16RGB565Data;
		}

		pu16DestData ++;
	}	
}

static void RGB565toRGB888_SW(
	image_t *src,
	image_t *dst,
	uint8_t *pu8SrcData,
	uint8_t *pu8DstData,
	uint16_t au16RepeatePos[],
	uint16_t u32RepetePosCnt,
	uint32_t u32YRepeateCnt
)
{
	uint16_t *pu16SrcData = (uint16_t *)pu8SrcData;

	for(int i = 0; i < u32RepetePosCnt; i ++)
	{
		uint8_t u8Temp;
		uint16_t u16RGB565Data;
		uint8_t u8R;
		uint8_t u8G;
		uint8_t u8B;

		u8Temp = au16RepeatePos[i];
		pu16SrcData = (uint16_t *)(pu8SrcData + (u8Temp * src->bpp));
		u16RGB565Data = *pu16SrcData;
		u8R = COLOR_RGB565_TO_R8(u16RGB565Data);
		u8G = COLOR_RGB565_TO_G8(u16RGB565Data);
		u8B = COLOR_RGB565_TO_B8(u16RGB565Data);

		for(int j = 0 ; j < u32YRepeateCnt; j ++)
		{
			int pos = j * dst->w * dst->bpp;
			pu8DstData[pos] = u8R;
			pu8DstData[pos + 1] = u8G;
			pu8DstData[pos + 2] = u8B;
		}

		pu8DstData += 3;
	}	
}



static void RGB888toRGB888_SW(
	image_t *src,
	image_t *dst,
	uint8_t *pu8SrcData,
	uint8_t *pu8DstData,
	uint16_t au16RepeatePos[],
	uint16_t u32RepetePosCnt,
	uint32_t u32YRepeateCnt
)
{
	uint8_t *pu8RGB888Data;
	

	for(int i = 0; i < u32RepetePosCnt; i ++)
	{
		uint8_t u8Temp;
		u8Temp = au16RepeatePos[i];
		pu8RGB888Data = pu8SrcData + (u8Temp * src->bpp);
		
		for(int j = 0 ; j < u32YRepeateCnt; j ++)
		{
			int pos = j * dst->w * dst->bpp;
			pu8DstData[pos] = pu8RGB888Data[0];
			pu8DstData[pos + 1] = pu8RGB888Data[1];
			pu8DstData[pos + 2] = pu8RGB888Data[2];

		}
		pu8DstData += 3;
	}	
}


void imlib_nvt_scale(image_t *src, image_t *dst, rectangle_t *roi)
{
	#define PIXELS_LOOP	16

	uint8_t *pu8SrcBuf;
	uint8_t *pu8DestBuf;

	uint16_t u16CropWinXPos;
	uint16_t u16CropWinYPos;
	
	uint32_t u32CropWinStartX = roi->x;
	uint32_t u32CropWinEndX = u32CropWinStartX + roi->w;

	uint32_t u32CropWinStartY = roi->y;
	uint32_t u32CropWinEndY = u32CropWinStartY + roi->h;
	
	int32_t i32ScaleOutXFac = dst->w / gcd(roi->w, dst->w);
	int32_t i32ScaleInXFac = roi->w / gcd(roi->w, dst->w);
	
	int32_t i32ScaleOutYFac = dst->h / gcd(roi->h, dst->h);
	int32_t i32ScaleInYFac = roi->h / gcd(roi->h, dst->h);

	uint32_t u32RepeatCntY = 0;
	uint32_t u32DeltaY = 0;		// Reset Y DDA
	uint32_t u32TotalCntY = 0;

	uint32_t u32RepeatCntX	= 0;
	uint32_t u32DeltaX = 0;		// Reset X DDA
	uint32_t u32TotalCntX = 0;

	uint16_t u16DestWinXPos = 0;
	uint16_t u16DestWinYPos = 0;

	uint32_t u32ScaleImgWidth = dst->w;
	uint32_t u32ScaleImgHeight = dst->h;

	uint16_t au16RepeatPosOffset[PIXELS_LOOP];
	uint16_t u16RepeatIndex = 0;
	uint16_t u16RepeatPosX = 0;

	pu8SrcBuf = src->data;
	pu8DestBuf = dst->data;

	uint8_t *pu8CurSrcRowPos;
	uint8_t *pu8CurDstRowPos;


	for(u16CropWinYPos = u32CropWinStartY; 
				u16CropWinYPos <  u32CropWinEndY;  u16CropWinYPos ++){

		UTIL_DDA(i32ScaleOutYFac, i32ScaleInYFac, u32DeltaY ,u32RepeatCntY);

		if(u32RepeatCntY == 0)
			continue;

		if(u32TotalCntY + u32RepeatCntY >= u32ScaleImgHeight){
			if(u32ScaleImgHeight > u32TotalCntY){
				u32RepeatCntY = u32ScaleImgHeight - u32TotalCntY;
			}
			else{
				continue;
			}
		} 
		
		u32TotalCntY += u32RepeatCntY;
	
		u32DeltaX = 0;
		u32TotalCntX = 0;
		u16DestWinXPos = 0;

		pu8CurSrcRowPos =  pu8SrcBuf + (u16CropWinYPos * src->w * src->bpp);
		pu8CurDstRowPos =  pu8DestBuf + (u16DestWinYPos * dst->w * dst->bpp);
		u16RepeatPosX = u32CropWinStartX;

		for(u16CropWinXPos = u32CropWinStartX; 
						u16CropWinXPos <  (u32CropWinEndX);  u16CropWinXPos ++){

			UTIL_DDA(i32ScaleOutXFac, i32ScaleInXFac, u32DeltaX ,u32RepeatCntX);

			if(u32RepeatCntX == 0)
				continue;

			if((u32TotalCntX + u32RepeatCntX) >= u32ScaleImgWidth){
				if(u32ScaleImgWidth > u32TotalCntX){
					u32RepeatCntX = u32ScaleImgWidth - u32TotalCntX;
				}
				else{
					continue;
				}
			}

			u32TotalCntX += u32RepeatCntX;

			while(u32RepeatCntX --){
				
				au16RepeatPosOffset[u16RepeatIndex] = u16CropWinXPos - u16RepeatPosX;
				u16RepeatIndex ++;

				if(u16RepeatIndex >= PIXELS_LOOP) {

					// RGB888 to RGB565
					if((src->pixfmt == PIXFORMAT_RGB888) && (dst->pixfmt == PIXFORMAT_RGB565))
					{

						RGB888toRGB565_16Pixels_SIMD(
							src,
							dst,
							pu8CurSrcRowPos + (u16RepeatPosX * src->bpp),
							pu8CurDstRowPos + (u16DestWinXPos * dst->bpp),
							au16RepeatPosOffset,
							u32RepeatCntY);						
					}
					else if((src->pixfmt == PIXFORMAT_RGB565) && (dst->pixfmt == PIXFORMAT_RGB888))
					{
#if 0
						RGB565toRGB888_16Pixels_SIMD(
							src,
							dst,
							pu8CurSrcRowPos + (u16RepeatPosX * src->bpp),
							pu8CurDstRowPos + (u16DestWinXPos * dst->bpp),
							au16RepeatPosOffset,
							u32RepeatCntY);						
#else
						RGB565toRGB888_SW(
							src,
							dst,
							pu8CurSrcRowPos + (u16RepeatPosX * src->bpp),
							pu8CurDstRowPos + (u16DestWinXPos * dst->bpp),
							au16RepeatPosOffset,
							PIXELS_LOOP,
							u32RepeatCntY);						

#endif
						
					}
					else if((src->pixfmt == PIXFORMAT_RGB888) && (dst->pixfmt == PIXFORMAT_RGB888))
					{
						RGB888toRGB888_SW(
							src,
							dst,
							pu8CurSrcRowPos + (u16RepeatPosX * src->bpp),
							pu8CurDstRowPos + (u16DestWinXPos * dst->bpp),
							au16RepeatPosOffset,
							PIXELS_LOOP,
							u32RepeatCntY);						
						
					}
					else
					{
						//scaling only
						
					}
			
					u16RepeatIndex = 0;
					u16RepeatPosX = u16CropWinXPos;
					u16DestWinXPos += PIXELS_LOOP;
				}
			}		
		}
		
		if(u16RepeatIndex)
		{
			//transfer < PIXELS_LOOP pixels
			if((src->pixfmt == PIXFORMAT_RGB888) && (dst->pixfmt == PIXFORMAT_RGB565))
			{
				RGB888toRGB565_SW(
							src,
							dst,
							pu8CurSrcRowPos + (u16RepeatPosX * src->bpp),
							pu8CurDstRowPos + (u16DestWinXPos * dst->bpp),
							au16RepeatPosOffset,
							u16RepeatIndex,
							u32RepeatCntY);						
			}
			else if((src->pixfmt == PIXFORMAT_RGB565) && (dst->pixfmt == PIXFORMAT_RGB888))
			{
				RGB565toRGB888_SW(
							src,
							dst,
							pu8CurSrcRowPos + (u16RepeatPosX * src->bpp),
							pu8CurDstRowPos + (u16DestWinXPos * dst->bpp),
							au16RepeatPosOffset,
							u16RepeatIndex,
							u32RepeatCntY);						

			}
			else if((src->pixfmt == PIXFORMAT_RGB888) && (dst->pixfmt == PIXFORMAT_RGB888))
			{
				RGB888toRGB888_SW(
							src,
							dst,
							pu8CurSrcRowPos + (u16RepeatPosX * src->bpp),
							pu8CurDstRowPos + (u16DestWinXPos * dst->bpp),
							au16RepeatPosOffset,
							u16RepeatIndex,
							u32RepeatCntY);						
			}

			u16RepeatIndex = 0;
		}
		
		u16DestWinYPos += u32RepeatCntY;
		u32RepeatCntY = 0;
	}
}

