/**************************************************************************//**
 * @file     FaceDetect.hpp
 * @version  V0.10
 * @brief    Face detect header file
 * * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef FACE_DETECT_HPP
#define FACE_DETECT_HPP

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
    E_COLOR_FORMAT eColorFormat;
    uint32_t u32Width;
    uint32_t u32Height;
    uint8_t *pu8Data;
} S_FACE_IMAGE;

typedef struct
{
    uint32_t u32X;
    uint32_t u32Y;
    uint32_t u32Width;
    uint32_t u32Height;
} S_FACE_BOX;

int FaceDetect_Init();
int FaceDetect_Detect(S_FACE_IMAGE *psFaceImage, S_FACE_BOX *psFaceBox);
int FaceDetect_Resize(S_FACE_IMAGE *psSrcImage, S_FACE_IMAGE *psDestImage, S_FACE_BOX *psROI);


#ifdef __cplusplus
}
#endif
#endif
