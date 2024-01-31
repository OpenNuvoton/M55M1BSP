/**************************************************************************//**
 * @file     FaceDetect.c
 * @version  V0.10
 * @brief    Face detect
 * * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "FaceDetect.h"
#include "log_macros.h"

#include "imlib.h"
#include "framebuffer.h"

static cascade_t s_sHaarCascade;
static bool s_bCascadeInit = false;

int FaceDetect_Init()
{
    if (s_bCascadeInit == false)
    {
        if (imlib_load_cascade(&s_sHaarCascade, "frontalface") != 0)
            return -1;

        s_bCascadeInit = true;
    }

    return 0;
}

int FaceDetect_Detect(S_FACE_IMAGE *psFaceImage, S_FACE_BOX *psFaceBox)
{
    image_t sSrcImage;
    uint8_t *pu8RGB565Buf = NULL;
    rectangle_t sROI;
    array_t *psObjectsArray = NULL;

    sSrcImage.w = psFaceImage->u32Width;
    sSrcImage.h = psFaceImage->u32Height;
    sSrcImage.pixfmt = PIXFORMAT_RGB565;
    sSrcImage.data = psFaceImage->pu8Data;

    if (psFaceImage->eColorFormat == eCOLOR_RGB888)
    {
        image_t sDstImage;

        sSrcImage.pixfmt = PIXFORMAT_RGB888;

        //convert RGB888 to RGB565
        //Allocate RGB565 buffer
        pu8RGB565Buf = (uint8_t *)malloc(psFaceImage->u32Width * psFaceImage->u32Height * 2);

        if (pu8RGB565Buf == NULL)
            return -1;

        sDstImage.w = sSrcImage.w;
        sDstImage.h = sSrcImage.h;
        sDstImage.pixfmt = PIXFORMAT_RGB565;
        sDstImage.data = pu8RGB565Buf;

        imlib_nvt_RGB888toRGB565_SIMD(&sSrcImage, &sDstImage);

        sSrcImage.pixfmt = PIXFORMAT_RGB565;
        sSrcImage.data = pu8RGB565Buf;
    }

    sROI.x = 0;
    sROI.y = 0;
    sROI.w = psFaceImage->u32Width;
    sROI.h = psFaceImage->u32Height;


    s_sHaarCascade.scale_factor = 1.5;
    //s_sHaarCascade.scale_factor = 1.8;
    s_sHaarCascade.threshold  = 1;

    psObjectsArray = imlib_detect_objects(&sSrcImage, &s_sHaarCascade, &sROI);

    if (pu8RGB565Buf)
        free(pu8RGB565Buf);

    if (!psObjectsArray)
    {
        return -2;
    }

    uint32_t u32MaxObjectSize = 0;
    rectangle_t *psObjectRect = NULL;

    //find the largest object
    for (int i = 0; i < array_length(psObjectsArray); i++)
    {
        rectangle_t *psRect = (rectangle_t *)array_at(psObjectsArray, i);

        if ((psRect->w * psRect->h) > u32MaxObjectSize)
        {
            u32MaxObjectSize = psRect->w * psRect->h;
            psObjectRect = psRect;
        }
    }

    if (psObjectRect == NULL)
    {
        array_free(psObjectsArray);
        return -3;
    }

    psFaceBox->u32Width = psObjectRect->w;
    psFaceBox->u32Height = psObjectRect->h;
    psFaceBox->u32X = psObjectRect->x;
    psFaceBox->u32Y = psObjectRect->y;

    array_free(psObjectsArray);
    return 0;
}

#if 1
//for scaling with rgb conversion test
int FaceDetect_Resize(S_FACE_IMAGE *psSrcImage, S_FACE_IMAGE *psDestImage, S_FACE_BOX *psROI)
{
    image_t sSrcImage;
    image_t sDstImage;
    rectangle_t sROI;

    sSrcImage.w = psSrcImage->u32Width;
    sSrcImage.h = psSrcImage->u32Height;

    if (psSrcImage->eColorFormat == eCOLOR_RGB888)
    {
        sSrcImage.pixfmt = PIXFORMAT_RGB888;
    }
    else
    {
        sSrcImage.pixfmt = PIXFORMAT_RGB565;
    }

    sSrcImage.data = psSrcImage->pu8Data;

    if ((psDestImage->pu8Data == NULL) || (psSrcImage->pu8Data == NULL))
    {
        return -1;
    }

    sDstImage.w = psDestImage->u32Width;
    sDstImage.h = psDestImage->u32Height;

    if (psDestImage->eColorFormat == eCOLOR_RGB888)
    {
        sDstImage.pixfmt = PIXFORMAT_RGB888;
    }
    else
    {
        sDstImage.pixfmt = PIXFORMAT_RGB565;
    }

    sDstImage.data = psDestImage->pu8Data;

    sROI.w = psROI->u32Width;
    sROI.h = psROI->u32Height;
    sROI.x = psROI->u32X;
    sROI.y = psROI->u32Y;

    imlib_nvt_scale(&sSrcImage, &sDstImage, &sROI);

    return 0;
}
#else
int FaceDetect_Resize(S_FACE_IMAGE *psSrcImage, S_FACE_IMAGE *psDestImage, S_FACE_BOX *psROI)
{
    image_t sSrcImage;
    image_t sDstImage;
    uint8_t *pu8RGB565Buf_Src = NULL;
    uint8_t *pu8RGB565Buf_Dest = NULL;
    int i32Ret = 0;
    rectangle_t sROI;

    sSrcImage.w = psSrcImage->u32Width;
    sSrcImage.h = psSrcImage->u32Height;
    sSrcImage.pixfmt = PIXFORMAT_RGB565;
    sSrcImage.data = psSrcImage->pu8Data;

    if ((psDestImage->pu8Data == NULL) || (psSrcImage->pu8Data == NULL))
    {
        i32Ret = -1;
        goto FaceDetect_Resize_Done;
    }

    if (psSrcImage->eColorFormat == eCOLOR_RGB888)
    {
        //convert RGB888 to RGB565
        //Allocate RGB565 buffer

        sSrcImage.pixfmt = PIXFORMAT_RGB888;

        pu8RGB565Buf_Src = malloc(psSrcImage->u32Width * psSrcImage->u32Height * 2);

        if (pu8RGB565Buf_Src == NULL)
        {
            i32Ret = -2;
            goto FaceDetect_Resize_Done;
        }

        sDstImage.w = sSrcImage.w;
        sDstImage.h = sSrcImage.h;
        sDstImage.pixfmt = PIXFORMAT_RGB565;
        sDstImage.data = pu8RGB565Buf_Src;

        imlib_nvt_RGB888toRGB565(&sSrcImage, &sDstImage);

        sSrcImage.pixfmt = PIXFORMAT_RGB565;
        sSrcImage.data = pu8RGB565Buf_Src;
    }


    if (psDestImage->eColorFormat == eCOLOR_RGB888)
    {
        //Allocate dest RGB565 buffer
        pu8RGB565Buf_Dest = malloc(psDestImage->u32Width * psDestImage->u32Height * 2);

        if (pu8RGB565Buf_Dest == NULL)
        {
            i32Ret = -3;
            goto FaceDetect_Resize_Done;
        }

        sDstImage.data = pu8RGB565Buf_Dest;
    }
    else
    {
        sDstImage.data = psDestImage->pu8Data;
    }

    sDstImage.w = psDestImage->u32Width;
    sDstImage.h = psDestImage->u32Height;
    sDstImage.pixfmt = PIXFORMAT_RGB565;

    sROI.w = psROI->u32Width;
    sROI.h = psROI->u32Height;
    sROI.x = psROI->u32X;
    sROI.y = psROI->u32Y;

    float fScaleX = (float)sDstImage.w / (float)sROI.w;
    float fScaleY = (float)sDstImage.h / (float)sROI.h;

    imlib_draw_image(&sDstImage, &sSrcImage, 0, 0,
                     fScaleX, fScaleY, &sROI, -1, 256, NULL,
                     NULL, IMAGE_HINT_BLACK_BACKGROUND, NULL, NULL);

    if (psDestImage->eColorFormat == eCOLOR_RGB888)
    {
        sSrcImage.w = sDstImage.w;
        sSrcImage.h = sDstImage.h;
        sSrcImage.pixfmt = PIXFORMAT_RGB565;
        sSrcImage.data = sDstImage.data;

        sDstImage.pixfmt = PIXFORMAT_RGB888;
        sDstImage.data = psDestImage->pu8Data;

        //convert RGB565 to RGB888
        imlib_nvt_RGB565toRGB888(&sSrcImage, &sDstImage);
    }

FaceDetect_Resize_Done:

    if (pu8RGB565Buf_Src)
        free(pu8RGB565Buf_Src);

    if (pu8RGB565Buf_Dest)
        free(pu8RGB565Buf_Dest);

    return i32Ret;
}
#endif
