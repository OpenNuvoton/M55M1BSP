/**************************************************************************//**
 * @file     ImageSensor.h
 * @version  V1.00
 * @brief    image sensor capture function
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __IMAGE_SENSOR_H__
#define __IMAGE_SENSOR_H__

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    eIMAGE_FMT_YUV422 = 0,
    eIMAGE_FMT_RGB555,
    eIMAGE_FMT_RGB565,
    eIMAGE_FMT_ONLY_Y,
    eIMAGE_FMT_ONLY_Y_1BIT,
    eIMAGE_FMT_RGB888_U8,
    eIMAGE_FMT_BGR888_U8,
    eIMAGE_FMT_RGB888_I8,
    eIMAGE_FMT_BGR888_I8,
    eIMAGE_FMT_ARGB888_U8,
    eIMAGE_FMT_BGRA888_U8,
    eIMAGE_FMT_ARGB888_I8,
    eIMAGE_FMT_BGRA888_I8,
} E_IMAGE_FMT;

int ImageSensor_Init(void);
int ImageSensor_Capture(uint32_t u32FrameBufAddr);
int ImageSensor_Config(E_IMAGE_FMT eImgFmt, uint32_t u32ImgWidth, uint32_t u32ImgHeight, bool bKeepRatio);
int ImageSensor_TriggerCapture(uint32_t u32FrameBufAddr);
int ImageSensor_WaitCaptureDone(void);

#ifdef __cplusplus
}
#endif

#endif
