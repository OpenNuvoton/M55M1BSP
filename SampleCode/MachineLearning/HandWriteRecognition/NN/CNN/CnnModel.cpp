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

#include "CNNModel.h"
#include "BufAttributes.h"
#include "board_m55m1.h"
#ifdef CNN

#ifndef ACTIVATION_BUF_SZ
    #define ACTIVATION_BUF_SZ       AD_ARENA /* default value of 128kiB */
#endif /* ACTIVATION_BUF_SZ */

uint8_t  _tensor_arena[ACTIVATION_BUF_SZ] MODEL_TFLITE_ATTRIBUTE;

CNNModel::CNNModel()
{

}

const tflite::MicroOpResolver &CNNModel::GetOpResolver()
{
    return this->_opResolver;
}

bool CNNModel::EnlistOperations()
{

    /*
          op used in Mobilenet eg
        For sfufflenet
          this->_opResolver.AddReshape();
          this->_opResolver.AddTranspose();
    */

    if (kTfLiteOk == this->_opResolver.AddEthosU())
    {
        printf("Added %s support to op resolver\n",
               tflite::GetString_ETHOSU());
    }
    else
    {
        printf("Failed to add Arm NPU support to op resolver.");
        return false;
    }

    return true;
}

uint8_t *CNNModel::GetTensorArena()
{
    return _tensor_arena;
}

size_t CNNModel::GetActivationBufferSize()
{
    return ACTIVATION_BUF_SZ;
}

#endif