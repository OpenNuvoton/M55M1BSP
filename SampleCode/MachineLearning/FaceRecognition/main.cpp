/**************************************************************************//**
 * @file     main.cpp
 * @version  V1.00
 * @brief    MobileFaceNet network sample. Demonstrate face recognition
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <vector>
#include <string>
#include <cinttypes>

#include "BoardInit.hpp"      /* Board initialisation */
#include "log_macros.h"      /* Logging macros (optional) */

#include "BufAttributes.hpp" /* Buffer attributes to be applied */
#include "InputFiles.hpp"             /* Baked-in input (not needed for live data) */
#include "MobileNetModel.hpp"       /* Model API */
#include "Labels.hpp"
#include "Recognizer.hpp"
#include "FaceRecognProcessing.hpp"

#include "imlib.h"          /* Image processing */
#include "framebuffer.h"

#undef PI /* PI macro conflict with CMSIS/DSP */
#include "NuMicro.h"

//#define __PROFILE__
//#define __USE_CCAP__
#define __USE_DISPLAY__

#include "Profiler.hpp"

#if defined (__USE_CCAP__)
    #include "ImageSensor.h"
#endif
#if defined (__USE_DISPLAY__)
    #include "Display.h"
#endif

using FaceRecognizer = arm::app::Recognizer;

namespace arm
{
namespace app
{
/* Tensor arena buffer */
static uint8_t tensorArena[ACTIVATION_BUF_SZ] ACTIVATION_BUF_ATTRIBUTE;

/* Optional getter function for the model pointer and its size. */
namespace mobilenet
{
extern uint8_t *GetModelPointer();
extern size_t GetModelLen();
} /* namespace mobilenet */
} /* namespace app */
} /* namespace arm */

/* Image processing initiate function */
//Used by omv library
#define GLCD_WIDTH 320
#define GLCD_HEIGHT 240

#undef OMV_FB_SIZE
#define OMV_FB_SIZE ((GLCD_WIDTH * GLCD_HEIGHT * 2) + 1024)

__attribute__((section(".bss.sram.data"), aligned(16))) static char fb_array[OMV_FB_SIZE + OMV_FB_ALLOC_SIZE];
__attribute__((section(".bss.sram.data"), aligned(16))) static char jpeg_array[OMV_JPEG_BUF_SIZE];

char *_fb_base = NULL;
char *_fb_end = NULL;
char *_jpeg_buf = NULL;
char *_fballoc = NULL;

static void omv_init()
{
    image_t frameBuffer;

    frameBuffer.w = GLCD_WIDTH;
    frameBuffer.h = GLCD_HEIGHT;
    frameBuffer.size = GLCD_WIDTH * GLCD_HEIGHT * 2;
    frameBuffer.pixfmt = PIXFORMAT_RGB565;

    _fb_base = fb_array;
    _fb_end =  fb_array + OMV_FB_SIZE - 1;
    _fballoc = _fb_base + OMV_FB_SIZE + OMV_FB_ALLOC_SIZE;
    _jpeg_buf = jpeg_array;

    fb_alloc_init0();

    framebuffer_init0();
    framebuffer_init_from_image(&frameBuffer);
}

int main()
{
    /* Initialize the UART module to allow printf related functions (if using retarget) */
    BoardInit();

    /* Model object creation and initialisation. */
    arm::app::MobileNetModel model;

    if (!model.Init(arm::app::tensorArena,
                    sizeof(arm::app::tensorArena),
                    arm::app::mobilenet::GetModelPointer(),
                    arm::app::mobilenet::GetModelLen()))
    {
        printf_err("Failed to initialise model\n");
        return 1;
    }

    /* Setup cache poicy of tensor arean buffer */
    info("Set tesnor arena cache policy to WTRA \n");
    const std::vector<ARM_MPU_Region_t> mpuConfig =
    {
        {
            // SRAM for tensor arena
            ARM_MPU_RBAR(((unsigned int)arm::app::tensorArena),        // Base
                         ARM_MPU_SH_NON,    // Non-shareable
                         0,                 // Read-only
                         1,                 // Non-Privileged
                         1),                // eXecute Never enabled
            ARM_MPU_RLAR((((unsigned int)arm::app::tensorArena) + ACTIVATION_BUF_SZ - 1),        // Limit
                         eMPU_ATTR_CACHEABLE_WTRA) // Attribute index - Write-Through, Read-allocate
        },
#if defined (__USE_CCAP__)
        {
            // Image data from CCAP DMA, so must set frame buffer to Non-cache attribute
            ARM_MPU_RBAR(((unsigned int)fb_array),        // Base
                         ARM_MPU_SH_NON,    // Non-shareable
                         0,                 // Read-only
                         1,                 // Non-Privileged
                         1),                // eXecute Never enabled
            ARM_MPU_RLAR((((unsigned int)fb_array) + OMV_FB_SIZE - 1),        // Limit
                         eMPU_ATTR_NON_CACHEABLE) // NonCache
        },
#endif
    };

    // Setup MPU configuration
    InitPreDefMPURegion(&mpuConfig[0], mpuConfig.size());

    FaceRecognizer recognizer;  /* Classifier object. */
    uint8_t u8ImgIdx = 0;

    TfLiteTensor *inputTensor   = model.GetInputTensor(0);
    TfLiteTensor *outputTensor = model.GetOutputTensor(0);

    if (!inputTensor->dims)
    {
        printf_err("Invalid input tensor dims\n");
        return 2;
    }
    else if (inputTensor->dims->size < 3)
    {
        printf_err("Input tensor dimension should be >= 3\n");
        return 3;
    }

    TfLiteIntArray *inputShape = model.GetInputShape(0);

    const int inputImgCols = inputShape->data[arm::app::MobileNetModel::ms_inputColsIdx];
    const int inputImgRows = inputShape->data[arm::app::MobileNetModel::ms_inputRowsIdx];
    const uint32_t inputChannels = inputShape->data[arm::app::MobileNetModel::ms_inputChannelsIdx];

    //label information
    std::vector<std::string> labels;
    std::vector<S_LABEL_INFO> labelInfo;

    GetLabelsVector(labels);
    ParserLabelVector(labels, labelInfo, nullptr);

    /* Set up pre and post-processing. */
    arm::app::FaceRecognPreProcess preProcess = arm::app::FaceRecognPreProcess(&model);

    arm::app::RecognitionResult result;
    std::string predictLabelInfo;

    arm::app::FaceRecognPostProcess postProcess = arm::app::FaceRecognPostProcess(recognizer, &model,
                                                                                  labelInfo, result);

    //display framebuffer
    image_t frameBuffer;
    rectangle_t roi;

    //omv library init
    omv_init();
    framebuffer_init_image(&frameBuffer);

#if defined(__PROFILE__)

    arm::app::Profiler profiler;
    uint64_t u64StartCycle;
    uint64_t u64EndCycle;
    uint64_t u64CCAPStartCycle;
    uint64_t u64CCAPEndCycle;
#else
    pmu_reset_counters();
#endif

#define EACH_PERF_SEC 5
    uint64_t u64PerfCycle;
    uint64_t u64PerfFrames = 0;

    u64PerfCycle = pmu_get_systick_Count();
    u64PerfCycle += (SystemCoreClock * EACH_PERF_SEC);

#if defined (__USE_CCAP__)
    //Setup image senosr
    ImageSensor_Init();
    ImageSensor_Config(eIMAGE_FMT_RGB565, frameBuffer.w, frameBuffer.h);
#endif

#if defined (__USE_DISPLAY__)
    char szDisplayText[100];

    Display_Init();
    Display_ClearLCD(C_WHITE);
#endif

    bool bFaceDetected = false;

#if !defined (__USE_CCAP__)
    char chStdIn;

    info("Press 'n' to run next image inference \n");
    info("Press 'q' to exit program \n");

    while ((chStdIn = getchar()))
    {
        if (chStdIn == 'q')
            break;
        else if (chStdIn != 'n')
            continue;

#else

    while (1)
    {
#endif

#if !defined (__USE_CCAP__)

        const uint8_t *pu8ImgSrc = get_img_array(u8ImgIdx);

        if (nullptr == pu8ImgSrc)
        {
            printf_err("Failed to get image index %" PRIu32 " (max: %u)\n", u8ImgIdx,
                       NUMBER_OF_FILES - 1);
            return 4;
        }

        //resize source image to framebuffer
        image_t srcImg;

        srcImg.w = IMAGE_WIDTH;
        srcImg.h = IMAGE_HEIGHT;
        srcImg.data = (uint8_t *)pu8ImgSrc;
        srcImg.pixfmt = PIXFORMAT_RGB888;

        roi.x = 0;
        roi.y = 0;
        roi.w = IMAGE_WIDTH;
        roi.h = IMAGE_HEIGHT;
        imlib_nvt_scale(&srcImg, &frameBuffer, &roi);
#endif
        //Face detect
        arm::app::FaceRecognPreProcess::S_FACE_RECOGN_IMAGE sSrcImage;
        arm::app::FaceRecognPreProcess::S_FACE_RECOGN_IMAGE_ROI sROI;

        sSrcImage.eColorFormat = arm::app::FaceRecognPreProcess::eFACE_RECOGN_COLOR_RGB565;
        sSrcImage.u32Width = frameBuffer.w;
        sSrcImage.u32Height = frameBuffer.h;
        sSrcImage.pu8Data = (uint8_t *)frameBuffer.data;

        bFaceDetected = preProcess.RunFaceDetect(&sSrcImage, &sROI);

#if defined (__USE_DISPLAY__)
        //Display image on LCD

        if (bFaceDetected)
        {
            imlib_draw_rectangle(&frameBuffer, sROI.u32X, sROI.u32Y, sROI.u32Width, sROI.u32Height, COLOR_B5_MAX, 2, false);
        }

        //Display image on LCD
        S_DISP_RECT sDispRect;

        sDispRect.u32TopLeftX = 0;
        sDispRect.u32TopLeftY = 0;
        sDispRect.u32BottonRightX = (frameBuffer.w - 1);
        sDispRect.u32BottonRightY = (frameBuffer.h - 1);

#if defined(__PROFILE__)
        u64StartCycle = pmu_get_systick_Count();
#endif

        Display_FillRect((uint16_t *)frameBuffer.data, &sDispRect);

#if defined(__PROFILE__)
        u64EndCycle = pmu_get_systick_Count();
        info("display image cycles %llu \n", (u64EndCycle - u64StartCycle));
#endif

#endif

        if (bFaceDetected == false)
        {
            info("Face not found on image %" PRIu32 " => %s\n", u8ImgIdx, get_filename(u8ImgIdx));
            goto round_done;
        }

        info("Image %" PRIu32 " => %s\n", u8ImgIdx, get_filename(u8ImgIdx));
        info("Face detect on image (x,y,w,h) => (%d, %d, %d, %d)\n", sROI.u32X, sROI.u32Y, sROI.u32Width, sROI.u32Height);

        // Resize detected face to model input
        arm::app::FaceRecognPreProcess::S_FACE_RECOGN_IMAGE sResizeImage;

        sResizeImage.eColorFormat = arm::app::FaceRecognPreProcess::eFACE_RECOGN_COLOR_RGB888;
        sResizeImage.u32Width = inputImgCols;
        sResizeImage.u32Height = inputImgRows;
        sResizeImage.pu8Data = (uint8_t *)inputTensor->data.data;

#if defined(__PROFILE__)
        u64StartCycle = pmu_get_systick_Count();
#endif

        preProcess.FaceResize(&sSrcImage, &sResizeImage, &sROI);

#if defined(__PROFILE__)
        u64EndCycle = pmu_get_systick_Count();
        info("resize cycles %llu \n", (u64EndCycle - u64StartCycle));
#endif

#if defined (__USE_CCAP__)
        //Capture new image
#if defined(__PROFILE__)
        u64CCAPStartCycle = pmu_get_systick_Count();
#endif
        ImageSensor_TriggerCapture((uint32_t)frameBuffer.data);
#endif


#if defined(__PROFILE__)
        u64StartCycle = pmu_get_systick_Count();
#endif

        /* Run the pre-processing, inference and post-processing. */
        if (!preProcess.DoPreProcess(sResizeImage.pu8Data, (sResizeImage.u32Width * sResizeImage.u32Height * inputChannels)))
        {
            goto round_done;
        }

#if defined(__PROFILE__)
        u64EndCycle = pmu_get_systick_Count();
        info("quantize cycles %llu \n", (u64EndCycle - u64StartCycle));
#endif

#if defined(__PROFILE__)
        profiler.StartProfiling("Inference");
#endif

        if (!model.RunInference())
        {
            printf_err("Inference failed.");
            return 5;
        }

#if defined(__PROFILE__)
        profiler.StopProfiling();
#endif

#if defined (__USE_CCAP__)
        //Capture new image

        ImageSensor_WaitCaptureDone();
#if defined(__PROFILE__)
        u64CCAPEndCycle = pmu_get_systick_Count();
        info("ccap capture cycles %llu \n", (u64CCAPEndCycle - u64CCAPStartCycle));
#endif
#endif

        predictLabelInfo.clear();

#if defined(__PROFILE__)
        u64StartCycle = pmu_get_systick_Count();
#endif

        if (postProcess.DoPostProcess())
        {
            predictLabelInfo =  result.m_label + std::string(":") + std::to_string(result.m_predict);
        }
        else
        {
            predictLabelInfo = std::string("???") + std::string(":") + std::to_string(result.m_predict);
        }

#if defined(__PROFILE__)
        u64EndCycle = pmu_get_systick_Count();
        info("post processing cycles %llu \n", (u64EndCycle - u64StartCycle));
#endif

        //show result
        info("Final results:\n");
        info("%s\n", predictLabelInfo.c_str());

#if defined (__USE_DISPLAY__)
        sprintf(szDisplayText, "%s", predictLabelInfo.c_str());

        sDispRect.u32TopLeftX = 0;
        sDispRect.u32TopLeftY = frameBuffer.h;
        sDispRect.u32BottonRightX = (frameBuffer.w);
        sDispRect.u32BottonRightY = (frameBuffer.h + FONT_HTIGHT - 1);

        Display_ClearRect(C_WHITE, &sDispRect);
        Display_PutText(
            szDisplayText,
            strlen(szDisplayText),
            0,
            frameBuffer.h,
            C_BLUE,
            C_WHITE,
            true
        );
#endif

#if defined(__PROFILE__)
        profiler.PrintProfilingResult();
#endif

        u64PerfFrames ++;

        if (pmu_get_systick_Count() > u64PerfCycle)
        {
            info("Total inference rate: %llu\n", u64PerfFrames / EACH_PERF_SEC);

#if defined (__USE_DISPLAY__)
            sprintf(szDisplayText, "Frame Rate %llu", u64PerfFrames / EACH_PERF_SEC);

            sDispRect.u32TopLeftX = 0;
            sDispRect.u32TopLeftY = frameBuffer.h + FONT_HTIGHT;
            sDispRect.u32BottonRightX = (frameBuffer.w);
            sDispRect.u32BottonRightY = (frameBuffer.h + (2 * FONT_HTIGHT) - 1);

            Display_ClearRect(C_WHITE, &sDispRect);
            Display_PutText(
                szDisplayText,
                strlen(szDisplayText),
                0,
                frameBuffer.h + FONT_HTIGHT,
                C_BLUE,
                C_WHITE,
                true
            );
#endif

            u64PerfCycle = pmu_get_systick_Count();
            u64PerfCycle += (SystemCoreClock * EACH_PERF_SEC);
            u64PerfFrames = 0;
        }

round_done:
        u8ImgIdx ++;

        if (u8ImgIdx >= NUMBER_OF_FILES)
            u8ImgIdx = 0;
    }

    return 0;
}