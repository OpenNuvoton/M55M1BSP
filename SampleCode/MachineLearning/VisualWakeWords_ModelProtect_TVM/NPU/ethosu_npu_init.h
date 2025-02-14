/*
 * Copyright (c) 2022 Arm Limited. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef ETHOS_U_NPU_INIT_H
#define ETHOS_U_NPU_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(ARM_NPU)

#include "NuMicro.h"
#define ETHOS_U_BASE_ADDR NPU_BASE
#define ETHOS_U_IRQN NPU_IRQn
#define ETHOS_U_SEC_ENABLED 1
#define ETHOS_U_PRIV_ENABLED 1

/**
 * @brief   Initialises the Arm Ethos-U NPU
 * @return  0 if successful, error code otherwise
 **/
int arm_ethosu_npu_init(void);

#endif /* ARM_NPU */

#ifdef __cplusplus
}
#endif

#endif /* ETHOS_U_NPU_INIT_H */
