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

#ifndef ETHOS_U_NPU_MEM_CONFIG_H
#define ETHOS_U_NPU_MEM_CONFIG_H

#define ETHOS_U_CACHE_BUF_SZ    (0U)

#define ETHOS_U_MEM_BYTE_ALIGNMENT                  16

/**
 * Activation buffer aka tensor arena section name
 * We have to place the tensor arena in different region based on the memory config.
 **/
#define ACTIVATION_BUF_SECTION      section(".bss.NoInit.activation_buf_sram")
#define ACTIVATION_BUF_SECTION_NAME ("SRAM")

#endif /* ETHOS_U_NPU_MEM_CONFIG_H */
