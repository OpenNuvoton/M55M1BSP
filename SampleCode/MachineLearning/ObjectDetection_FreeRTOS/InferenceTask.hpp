/**************************************************************************//**
 * @file     InferenceProcess.hpp
 * @version  V0.10
 * @brief    Inference process header file
 * * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef INFERENCE_TASK_HPP
#define INFERENCE_TASK_HPP

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "DetectorPostProcessing.hpp" /* Post-processing class. */
#include "Model.hpp"

#if defined(__PROFILE__)
    #include "Profiler.hpp"
#endif

using namespace arm::app;

namespace InferenceProcess
{

class InferenceProcess
{
public:
    InferenceProcess(Model *model);
    bool RunJob(
        object_detection::DetectorPostprocessing *pPostProc,
        int modelCols,
        int mode1Rows,
        int srcImgWidth,
        int srcImgHeight,
        std::vector<object_detection::DetectionResult> *results);
protected:

#if defined(__PROFILE__)
    arm::app::Profiler profiler;
#endif

    Model *m_model = nullptr;
};
}// namespace InferenceProcess

struct ProcessTaskParams
{
    Model *model;
    QueueHandle_t queueHandle;
};

struct xInferenceJob
{
    QueueHandle_t responseQueue;
    object_detection::DetectorPostprocessing *pPostProc;
    int modelCols;
    int mode1Rows;
    int srcImgWidth;
    int srcImgHeight;

    std::vector<object_detection::DetectionResult> *results;
};

void inferenceProcessTask(void *pvParameters);

#endif
