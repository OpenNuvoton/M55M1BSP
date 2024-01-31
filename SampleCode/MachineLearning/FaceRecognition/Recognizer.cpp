/**************************************************************************//**
 * @file     Recognizer.cc
 * @version  V0.10
 * @brief    Recognizer function
 * * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "Recognizer.hpp"

#include "TensorFlowLiteMicro.hpp"
#include "log_macros.h"

#include <vector>
#include <string>
#include <set>
#include <cstdint>
#include <cinttypes>
#include <cmath>

// https://en.wikipedia.org/wiki/Cosine_similarity
static float cosine_similarity(std::vector<float> embedArray1, std::vector<float> embedArray2)
{
    int numElem;
    float ret = 0, mod1 = 0, mod2 = 0;

    if (embedArray1.size() != embedArray2.size())
        return -1;

    //embedArray1 and embedArray2 must the same size.
    numElem = embedArray1.size();

    for (int i = 0; i < numElem; i++)
    {
        ret += embedArray1[i] * embedArray2[i];
        mod1 += embedArray1[i] * embedArray1[i];
        mod2 += embedArray2[i] * embedArray2[i];
    }

    return ret / sqrt(mod1) / sqrt(mod2);
}


namespace arm
{
namespace app
{
Recognizer::Recognizer()
{
    m_threshold = 0.7;
}

bool Recognizer::GetRecognitionResults(
    TfLiteTensor *outputTensor,
    RecognitionResult &predResults,
    const std::vector <S_LABEL_INFO> &labels)
{
    predResults.m_predict = 0;

    if (outputTensor == nullptr)
    {
        printf_err("Output vector is null pointer.\n");
        return false;
    }

    uint32_t embedDimension = 1;

    for (int inputDim = 0; inputDim < outputTensor->dims->size; inputDim++)
    {
        embedDimension *= outputTensor->dims->data[inputDim];
    }

    /* De-Quantize Output Tensor */
    QuantParams quantParams = GetTensorQuantParams(outputTensor);

    /* Floating point tensor data to be populated
     * NOTE: The assumption here is that the output tensor size isn't too
     * big and therefore, there's neglibible impact on heap usage. */
    std::vector<float> embedArray(embedDimension);

    /* Populate the floating point buffer */
    switch (outputTensor->type)
    {
        case kTfLiteUInt8:
        {
            uint8_t *tensor_buffer = tflite::GetTensorData<uint8_t>(outputTensor);

            for (size_t i = 0; i < embedDimension; ++i)
            {
                embedArray[i] = quantParams.scale *
                                (static_cast<float>(tensor_buffer[i]) - quantParams.offset);
            }

            break;
        }

        case kTfLiteInt8:
        {
            int8_t *tensor_buffer = tflite::GetTensorData<int8_t>(outputTensor);

            for (size_t i = 0; i < embedDimension; ++i)
            {
                embedArray[i] = quantParams.scale *
                                (static_cast<float>(tensor_buffer[i]) - quantParams.offset);
            }

            break;
        }

        case kTfLiteFloat32:
        {
            float *tensor_buffer = tflite::GetTensorData<float>(outputTensor);

            for (size_t i = 0; i < embedDimension; ++i)
            {
                embedArray[i] = tensor_buffer[i];
            }

            break;
        }

        default:
            printf_err("Tensor type %s not supported by classifier\n",
                       TfLiteTypeGetName(outputTensor->type));
            return false;
    }

    float distance;

    int predictLabelIndex = -1;
    float closedDisatance = -1;

    for (int i = 0 ; i < labels.size(); i ++)
    {
        distance = cosine_similarity(embedArray, labels[i].fParam);

        if (distance > closedDisatance)
        {
            predictLabelIndex = i;
            closedDisatance = distance;
        }
    }

    predResults.m_predict = closedDisatance;

    if ((predictLabelIndex >= 0) && (closedDisatance > m_threshold))
    {
        predResults.m_label =  labels[predictLabelIndex].szLable;
        return true;
    }

    return false;
}

void Recognizer::SetThreshold(double threshold)
{
    m_threshold = threshold;
}


} /* namespace app */
} /* namespace arm */
