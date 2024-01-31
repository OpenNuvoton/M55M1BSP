/**************************************************************************//**
 * @file     Recognizer.hpp
 * @version  V0.10
 * @brief    Recognizer header
 * * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef RECOGNIZER_HPP
#define RECOGNIZER_HPP

#include "RecognitionResult.hpp"
#include "TensorFlowLiteMicro.hpp"
#include "Labels.hpp"

#include <vector>

namespace arm
{
namespace app
{

/**
 * @brief   Classifier - a helper class to get certain number of top
 *          results from the output vector from a classification NN.
 **/
class Recognizer
{
public:
    /** @brief Constructor. */
    Recognizer();

    /**
     * @brief       Gets the recognition results from the
     *              output vector.
     * @param[in]   outputTensor   Inference output tensor from an NN model.
     * @param[out]  vecResults     A vector of classification results.
     *                             populated by this function.
     * @param[in]   labels         Labels vector to match classified classes.
     * @param[in]   topNCount      Number of top classifications to pick. Default is 1.
     * @param[in]   useSoftmax     Whether Softmax normalisation should be applied to output. Default is false.
     * @return      true if successful, false otherwise.
     **/

    virtual bool GetRecognitionResults(
        TfLiteTensor *outputTensor,
        RecognitionResult &predResults,
        const std::vector <S_LABEL_INFO> &labels);

    /**
    * @brief       Set predict threshold
    * @param[in]   threshold value.
    **/

    void SetThreshold(double threshold);

private:
    double m_threshold;
};

} /* namespace app */
} /* namespace arm */

#endif /* CLASSIFIER_HPP */
