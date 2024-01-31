/**************************************************************************//**
 * @file     FaceRecognProcessing.hpp
 * @version  V0.10
 * @brief    Face recognition pre/post processing header file
 * * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef FACE_RECOGN_PROCESSING_HPP
#define FACE_RECOGN_PROCESSING_HPP

#include <string>

#include "BaseProcessing.hpp"
#include "Model.hpp"
#include "Labels.hpp"
#include "Recognizer.hpp"

namespace arm
{
namespace app
{

/**
 * @brief   Pre-processing class for face recognition use case.
 *          Implements methods declared by BasePreProcess and anything else needed
 *          to populate input tensors ready for inference.
 */
class FaceRecognPreProcess : public BasePreProcess
{

public:

    typedef enum
    {
        eFACE_RECOGN_COLOR_RGB888,
        eFACE_RECOGN_COLOR_RGB565
    } E_FACE_RECOGN_COLOR;

    typedef struct
    {
        E_FACE_RECOGN_COLOR eColorFormat;
        uint32_t u32Width;
        uint32_t u32Height;
        uint8_t *pu8Data;
    } S_FACE_RECOGN_IMAGE;

    typedef struct
    {
        uint32_t u32X;
        uint32_t u32Y;
        uint32_t u32Width;
        uint32_t u32Height;
    } S_FACE_RECOGN_IMAGE_ROI;

    typedef struct
    {
        uint32_t u32X;
        uint32_t u32Y;
        uint32_t u32Width;
        uint32_t u32Height;
    } S_FACE_RECOGN_FACE_BOX;

    explicit FaceRecognPreProcess(Model *model);

    bool RunFaceDetect(S_FACE_RECOGN_IMAGE *psSrcImage, S_FACE_RECOGN_IMAGE_ROI *psROI);
    bool FaceResize(S_FACE_RECOGN_IMAGE *psSrcImage, S_FACE_RECOGN_IMAGE *psDestImage, S_FACE_RECOGN_IMAGE_ROI *psROI);

    bool DoPreProcess(const void *input, size_t inputSize) override;

protected:
    Model *m_model = nullptr;
};

/**
 * @brief   Post-processing class for face recognition use case.
 *          Implements methods declared by BasePostProcess and anything else needed
 *          to populate result vector.
 */
class FaceRecognPostProcess : public BasePostProcess
{

private:
    Recognizer &m_recognizer;
    const std::vector<S_LABEL_INFO> &m_labels;
    RecognitionResult &m_result;

public:
    FaceRecognPostProcess(Recognizer &recognizer, Model *model,
                          const std::vector<S_LABEL_INFO> &labels,
                          RecognitionResult &result);
    bool DoPostProcess() override;
protected:
    Model *m_model = nullptr;
};
} /* namespace app */
} /* namespace arm */



#endif
