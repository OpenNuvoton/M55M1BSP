/*
 * Copyright (C) 2021 Arm Limited or its affiliates. All rights reserved.
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

#ifndef __MainClassify_H__
#define __MainClassify_H__

#include <vector>
#include "Model.h"
//#include "MPU6500.h"
typedef struct
{
    float   scale;
    int     offset;
} QuantParams;


class MainClassify
{

public:
    explicit MainClassify(int8_t *pBuffer);

    ~MainClassify() = default;

    void ExtractFeatures();
    void Classify();
    void FillInTensorData();
    void AveragePredictions();
    int GetTopClass(const std::vector<int8_t> &prediction);
    int GetInferenceResult(const std::vector<int8_t> &prediction);
    uint8_t QuantizeInputData(uint8_t *u8pImg);
    void InitMainClassify();

    int8_t *gsensorBufferPtr;
    std::vector<int8_t> output;
    QuantParams GetTensorQuantParams(TfLiteTensor *tensor);

    int numOutClasses;
    int numInputDims[4];
    uint32_t gsensorBufferIdx;
    QuantParams inQuantParams;//To keep input quantization info
    QuantParams outQuantParams;

protected:
    /** @brief Initialises the model */
    bool _InitModel();
    //void InitMainClassify();
    std::unique_ptr<Model> model;
};


#endif /* __MainClassify_H__ */
