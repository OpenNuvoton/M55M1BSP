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



#include "Model.h"

#ifdef CNN

#ifndef CNNMODEL_H
#define CNNMODEL_H

class CNNModel : public Model
{

public:
    CNNModel();
private:
    /* Maximum number of individual operations that can be enlisted */
    static constexpr int _maxOpCnt = 10;

    /* A mutable op resolver instance */
    tflite::MicroMutableOpResolver<_maxOpCnt> _opResolver;
protected:
    /** @brief   Gets the reference to op resolver interface class */
    const tflite::MicroOpResolver &GetOpResolver() override;

    /** @brief   Adds operations to the op resolver instance */
    bool EnlistOperations() override;

    /** @brief   Gets a pointer to the tensor arena */
    uint8_t *GetTensorArena() override;

    /** @brief   Gets the total size of tensor arena available for use */
    size_t GetActivationBufferSize() override;
};

#endif /* CNNMODEL_H */

#endif /* CNN */
