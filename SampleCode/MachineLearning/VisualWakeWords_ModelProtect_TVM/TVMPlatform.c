/**************************************************************************//**
 * @file     TVMPlatform.c
 * @version  V1.00
 * @brief    TVM platform initiate function
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "tvm_runtime.h"
#include "tvm/runtime/crt/module.h"

// Called to start system timer.
tvm_crt_error_t TVMPlatformTimerStart()
{
    return kTvmErrorNoError;
}

// Called to stop system timer.
tvm_crt_error_t TVMPlatformTimerStop(double *elapsed_time_seconds)
{
    return kTvmErrorNoError;
}


const TVMModule *TVMSystemLibEntryPoint(void)
{
    return NULL;
}
