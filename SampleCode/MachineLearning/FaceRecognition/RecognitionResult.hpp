/**************************************************************************//**
 * @file     RecognitionResult.hpp
 * @version  V0.10
 * @brief    Recognition result header
 * * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef RECOGNITION_RESULT_HPP
#define RECOGNITION_RESULT_HPP

#include <string>

namespace arm
{
namespace app
{

/**
 * @brief   Class representing a single recognition result.
 */
class RecognitionResult
{
public:
    double          m_predict = 0.0;
    std::string     m_label;

    RecognitionResult() = default;
    ~RecognitionResult() = default;
};

} /* namespace app */
} /* namespace arm */

#endif /* RECOGNITION_RESULT_HPP */