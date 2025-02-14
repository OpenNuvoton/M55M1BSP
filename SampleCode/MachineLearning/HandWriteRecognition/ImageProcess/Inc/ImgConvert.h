
/**************************************************************************//**
 * @file     ImgConvert.h
 * @version  V0.10
 * @brief    image convert header file
 * * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef IMG_CONVERT_H
#define IMG_CONVERT_H

#include <stdio.h>
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    eCOLOR_RGB888,
    eCOLOR_RGB565
} E_COLOR_FORMAT;

typedef struct
{
    uint32_t u32X;
    uint32_t u32Y;
    uint32_t u32Width;
    uint32_t u32Height;
} S_IMG_BOX;

typedef struct
{
    E_COLOR_FORMAT eColorFormat;
    uint32_t u32Width;
    uint32_t u32Height;
    uint8_t *pu8Data;
} S_IMG_IMAGE;

int ImgConvert_Init();
int ImgConvert_Resize(S_IMG_IMAGE *psSrcImage, S_IMG_IMAGE *psDestImage, S_IMG_BOX *psROI);


#ifdef __cplusplus
}
#endif

#endif
