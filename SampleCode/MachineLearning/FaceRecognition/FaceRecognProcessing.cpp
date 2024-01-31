/**************************************************************************//**
 * @file     FaceRecognProcessing.cc
 * @version  V0.10
 * @brief    Face recognition pre/post processing
 * * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "FaceRecognProcessing.hpp"
#include "log_macros.h"
#include "FaceDetect.h"

static void ConvertImgToInt8(void *data, const size_t kMaxImageSize)
{
    auto *tmp_req_data = static_cast<uint8_t *>(data);
    auto *tmp_signed_req_data = static_cast<int8_t *>(data);

    for (size_t i = 0; i < kMaxImageSize; i++)
    {
        tmp_signed_req_data[i] = (int8_t)(
                                     (int32_t)(tmp_req_data[i]) - 128);
    }
}


namespace arm
{
namespace app
{

FaceRecognPreProcess::FaceRecognPreProcess(Model *model)
{
    this->m_model = model;
    FaceDetect_Init();
}

bool FaceRecognPreProcess::RunFaceDetect(S_FACE_RECOGN_IMAGE *psSrcImage, S_FACE_RECOGN_IMAGE_ROI *psROI)
{
    if (psSrcImage->pu8Data == nullptr)
    {
        printf_err("Data pointer is null");
        return false;
    }

    S_FACE_IMAGE sFaceImage;
    S_FACE_BOX sFaceBox;

    if (psSrcImage->eColorFormat == eFACE_RECOGN_COLOR_RGB888)
        sFaceImage.eColorFormat = eCOLOR_RGB888;
    else
        sFaceImage.eColorFormat = eCOLOR_RGB565;

    sFaceImage.u32Width = psSrcImage->u32Width;
    sFaceImage.u32Height = psSrcImage->u32Height;
    sFaceImage.pu8Data = psSrcImage->pu8Data;

    if (FaceDetect_Detect(&sFaceImage, &sFaceBox) != 0)
        return false;

    psROI->u32X = sFaceBox.u32X;
    psROI->u32Y = sFaceBox.u32Y;
    psROI->u32Width = sFaceBox.u32Width;
    psROI->u32Height = sFaceBox.u32Height;

    return true;
}

bool FaceRecognPreProcess::FaceResize(S_FACE_RECOGN_IMAGE *psSrcImage, S_FACE_RECOGN_IMAGE *psDestImage, S_FACE_RECOGN_IMAGE_ROI *psROI)
{
    if ((psSrcImage->pu8Data == nullptr) || (psDestImage->pu8Data == nullptr))
    {
        printf_err("Data pointer is null");
        return false;
    }

    S_FACE_IMAGE sFaceSrcImage;
    S_FACE_IMAGE sFaceDestImage;
    S_FACE_BOX sFaceBox;

    if (psSrcImage->eColorFormat == eFACE_RECOGN_COLOR_RGB888)
        sFaceSrcImage.eColorFormat = eCOLOR_RGB888;
    else
        sFaceSrcImage.eColorFormat = eCOLOR_RGB565;

    sFaceSrcImage.u32Width = psSrcImage->u32Width;
    sFaceSrcImage.u32Height = psSrcImage->u32Height;
    sFaceSrcImage.pu8Data = psSrcImage->pu8Data;

    if (psDestImage->eColorFormat == eFACE_RECOGN_COLOR_RGB888)
        sFaceDestImage.eColorFormat = eCOLOR_RGB888;
    else
        sFaceDestImage.eColorFormat = eCOLOR_RGB565;

    sFaceDestImage.u32Width = psDestImage->u32Width;
    sFaceDestImage.u32Height = psDestImage->u32Height;
    sFaceDestImage.pu8Data = psDestImage->pu8Data;

    sFaceBox.u32X = psROI->u32X;
    sFaceBox.u32Y = psROI->u32Y;
    sFaceBox.u32Width = psROI->u32Width;
    sFaceBox.u32Height = psROI->u32Height;

    if (FaceDetect_Resize(&sFaceSrcImage, &sFaceDestImage, &sFaceBox) != 0)
        return false;

    return true;
}

bool FaceRecognPreProcess::DoPreProcess(const void *data, size_t inputSize)
{
    if (data == nullptr)
    {
        printf_err("Data pointer is null");
    }

    auto input = static_cast<const uint8_t *>(data);
    TfLiteTensor *inputTensor = this->m_model->GetInputTensor(0);

    memcpy(inputTensor->data.data, input, inputSize);
    debug("Input tensor populated \n");

    if (this->m_model->IsDataSigned())
    {
        ConvertImgToInt8(inputTensor->data.data, inputTensor->bytes);
    }

    return true;
}

FaceRecognPostProcess::FaceRecognPostProcess(Recognizer &recognizer, Model *model,
                                             const std::vector<S_LABEL_INFO> &labels,
                                             RecognitionResult &result)
    : m_recognizer{recognizer},
      m_labels{labels},
      m_result{result}
{
    this->m_model = model;
}

bool FaceRecognPostProcess::DoPostProcess()
{
    return this->m_recognizer.GetRecognitionResults(
               this->m_model->GetOutputTensor(0),
               this->m_result,
               this->m_labels);
}


} /* namespace app */
} /* namespace arm */