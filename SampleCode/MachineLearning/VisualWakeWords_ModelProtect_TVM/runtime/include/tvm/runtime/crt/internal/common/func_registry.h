/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

// LINT_C_FILE

/*!
 * \file tvm/runtime/crt/include/tvm/runtime/crt/internal/common/func_registry.h
 * \brief Abstract device memory management API
 */
#ifndef TVM_RUNTIME_CRT_INCLUDE_TVM_RUNTIME_CRT_INTERNAL_COMMON_FUNC_REGISTRY_H_
#define TVM_RUNTIME_CRT_INCLUDE_TVM_RUNTIME_CRT_INTERNAL_COMMON_FUNC_REGISTRY_H_

#ifdef __cplusplus
extern "C" {
#endif

int strcmp_cursor(const char **cursor, const char *name);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // TVM_RUNTIME_CRT_INCLUDE_TVM_RUNTIME_CRT_INTERNAL_COMMON_FUNC_REGISTRY_H_
