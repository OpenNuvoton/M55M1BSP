/**************************************************************************//**
 * @file     InferenceTask.cpp
 * @version  V0.10
 * @brief    Inference process source code
 * * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "InferenceTask.hpp"
#include "log_macros.h"      /* Logging macros */

namespace InferenceProcess
{

InferenceProcess::InferenceProcess(
    Model *model)
    :   m_model(model)
{}

bool InferenceProcess::RunJob(
    object_detection::DetectorPostprocessing *pPostProc,
    int modelCols,
    int mode1Rows,
    int srcImgWidth,
    int srcImgHeight,
    std::vector<object_detection::DetectionResult> *results
)
{
    //    info("Inference process task run job...\n");

#if defined(__PROFILE__)
    uint64_t u64StartCycle;
    uint64_t u64EndCycle;

    profiler.StartProfiling("Inference");
#endif

    bool runInf = m_model->RunInference();

#if defined(__PROFILE__)
    profiler.StopProfiling();
    profiler.PrintProfilingResult();
#endif

    TfLiteTensor *modelOutput0 = m_model->GetOutputTensor(0);
    TfLiteTensor *modelOutput1 = m_model->GetOutputTensor(1);

#if defined(__PROFILE__)
    u64StartCycle = pmu_get_systick_Count();
#endif

    pPostProc->RunPostProcessing(
        mode1Rows,
        modelCols,
        srcImgHeight,
        srcImgWidth,
        modelOutput0,
        modelOutput1,
        *results);

#if defined(__PROFILE__)
    u64EndCycle = pmu_get_systick_Count();
    info("post processing cycles %llu \n", (u64EndCycle - u64StartCycle));
#endif

    return runInf;
}

}// namespace InferenceProcess

/****************************************************************************
 * Mutex & Semaphore
 * Overrides weak-linked symbols in ethosu_driver.c to implement thread handling
 ****************************************************************************/

extern "C" {

    void *ethosu_mutex_create(void)
    {
        return xSemaphoreCreateMutex();
    }

    int ethosu_mutex_lock(void *mutex)
    {
        SemaphoreHandle_t handle = reinterpret_cast<SemaphoreHandle_t>(mutex);
        xSemaphoreTake(handle, portMAX_DELAY);
        return 0;
    }

    int ethosu_mutex_unlock(void *mutex)
    {
        SemaphoreHandle_t handle = reinterpret_cast<SemaphoreHandle_t>(mutex);
        xSemaphoreGive(handle);
        return 0;
    }

    void *ethosu_semaphore_create(void)
    {
        return xSemaphoreCreateBinary();
    }

    int ethosu_semaphore_take(void *sem)
    {
        SemaphoreHandle_t handle = reinterpret_cast<SemaphoreHandle_t>(sem);

        if (xSemaphoreTake(handle, portMAX_DELAY) == pdFALSE)
        {
            printf("xSemaphoreTake return false \n");
        }

        return 0;
    }

    int ethosu_semaphore_give(void *sem)
    {
        SemaphoreHandle_t handle = reinterpret_cast<SemaphoreHandle_t>(sem);
        xSemaphoreGive(handle);
        return 0;
    }

    int ethosu_semaphore_give_from_ISR(void *sem)
    {
        SemaphoreHandle_t handle = reinterpret_cast<SemaphoreHandle_t>(sem);
        BaseType_t xHighPriorityTaskWoken = pdFALSE;

        if (xSemaphoreGiveFromISR(handle, &xHighPriorityTaskWoken) == pdFALSE)
        {
            printf("xSemaphoreGiveFromISR return false \n");
        }

        portYIELD_FROM_ISR(xHighPriorityTaskWoken);
        return 0;
    }
}


void inferenceProcessTask(void *pvParameters)
{
    struct ProcessTaskParams params = *reinterpret_cast<struct ProcessTaskParams *>(pvParameters);

    InferenceProcess::InferenceProcess inferenceProcess(params.model);

    for (;;)
    {
        xInferenceJob *xJob;

        xQueueReceive(params.queueHandle, &xJob, portMAX_DELAY);

        inferenceProcess.RunJob(
                            xJob->pPostProc,
                            xJob->modelCols,
                            xJob->mode1Rows,
                            xJob->srcImgWidth,
                            xJob->srcImgHeight,
                            xJob->results
                        );

        xQueueSend(xJob->responseQueue, &xJob, portMAX_DELAY);
    }

}




