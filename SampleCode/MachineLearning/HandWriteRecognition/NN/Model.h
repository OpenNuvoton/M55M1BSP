/*
 * Copyright (C) 2020 Arm Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MODEL_H
#define MODEL_H

/* Use which model, please remember update the model */
//#define DNN
//#define DSCNN
#define CNN

#include <cstdint>

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"


/**
 * @brief   NN model class wrapping the underlying Tensorflow-Lite-Micro API
 */
class Model
{
private:
    tflite::ErrorReporter          *_errorReporterPtr   = nullptr;      /* Pointer to the error reporter */
    const tflite::Model            *_model              = nullptr;      /* Tflite model pointer */
    tflite::MicroInterpreter       *_interpreter        = nullptr;      /* Tflite interpreter */
    bool                            _inited             = false;        /* Indicates whether this object has been initialised */

    TfLiteTensor                   *_input              = nullptr;      /* Model's input tensor pointer */
    TfLiteTensor                   *_output             = nullptr;      /* Model's output tensor pointer */
    TfLiteType                      _type               = kTfLiteNoType;/* Model's data type */


    // /* Maximum number of individual operations that can be enlisted */
    // static constexpr int _maxOpCnt = 5;
    //
    // /* A mutable op resolver instance */
    // tflite::MicroMutableOpResolver<_maxOpCnt> _opResolver;

public:
    /** @brief  Destructor */
    virtual ~Model();

    /** @brief  Gets the pointer to the model's input tensor */
    TfLiteTensor *GetInputTensor() const;

    /** @brief  Gets the pointer to the model's output tensor */
    TfLiteTensor *GetOutputTensor() const;

    /** @brief  Gets the model's data type */
    TfLiteType GetType() const;

    /** @brief  Gets the pointer to the model's input shape */
    TfLiteIntArray *GetInputShape() const;

    /** @brief  Gets the pointer to the model's input shape */
    TfLiteIntArray *GetOutputShape() const;

    /** @brief  Initialise the model class object */
    bool Init();

    /** @brief  Checks if this object has been initialised */
    bool IsInited() const;

    /** @brief  Runs the inference (invokes the interpreter) */
    bool RunInference();

    /* Get model from exposing.  */
    virtual const uint8_t *ModelPointer();
    virtual size_t ModelSize();

protected:
    Model();

    virtual const tflite::MicroOpResolver &GetOpResolver() = 0;
    virtual bool EnlistOperations() = 0;
    virtual uint8_t *GetTensorArena() = 0;
    virtual size_t GetActivationBufferSize() = 0;

};

/* Model is exposed by the following functions */

/**
 * @brief   Gets the pointer to the model data
 * @return  a uint8_t pointer
 **/
const uint8_t *GetModelPointer();

/**
 * @brief   Gets the model length in bytes
 * @return  length expressed as size_t
 **/
size_t GetModelLen();

#endif /* MODEL_H */