/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate run nn inference via TVM
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"
#include "BoardInit.h"
#include "ethosu_driver.h"

#include "tvm_runtime.h"
#include "tvm/runtime/crt/stack_allocator.h"
#include "tvmgen_default.h"
#include "tvm/runtime/crt/module.h"

#include "inputs.h"
#include "labels.h"

/****************************************************************************
 * autogen section: Output tensor data
 ****************************************************************************/
__attribute__((aligned(16), section(".bss.noinit.tvm"))) static int8_t s_StatefulPartitionedCall_0_buffer[TVMGEN_DEFAULT_STATEFULPARTITIONEDCALL_0_SIZE];
__attribute__((aligned(16), section(".bss.noinit.tvm"))) static int8_t s_Input_buffer[TVMGEN_DEFAULT_SERVING_DEFAULT_INPUT_1_0_SIZE];

/****************************************************************************
 * autogen section: TVM platform function
 ****************************************************************************/

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

/****************************************************************************
 * InferenceJob
 ****************************************************************************/
int main(void)
{

    /* Initialise the hardware resource(UART, NPU, HyperFlash) */
    BoardInit();

    printf("\nThis sample code run inference by TVM \n");

    /****************************************************************************
     * Setup cache poicy of tensor arean buffer
     ****************************************************************************/
    printf("Set workspace buffer to WTRA \n");
    uint32_t u32WorkspaceAddr = (uint32_t) tvmgen_get_workspace_address();

    const ARM_MPU_Region_t mpuConfig[] =
    {
        {
            // SRAM for tensor arena
            ARM_MPU_RBAR(((unsigned int)u32WorkspaceAddr),        // Base
                         ARM_MPU_SH_NON,    // Non-shareable
                         0,                 // Read-only
                         1,                 // Non-Privileged
                         1),                // eXecute Never enabled
            ARM_MPU_RLAR((((unsigned int)u32WorkspaceAddr) + TVMGEN_DEFAULT_WORKSPACE_SIZE - 1),        // Limit
                         eMPU_ATTR_CACHEABLE_WTRA) // Attribute index - Write-Through, Read-allocate
        }
    };

    // Setup MPU configuration
    InitPreDefMPURegion(mpuConfig, 1);

    /****************************************************************************
     * autogen section: Input tensor init
     ****************************************************************************/
    memcpy(s_Input_buffer, input, TVMGEN_DEFAULT_SERVING_DEFAULT_INPUT_1_0_SIZE);
    printf("copy input to sram \n");
    struct tvmgen_default_inputs inputs =
    {
        .serving_default_input_1_0 = (void *)s_Input_buffer,
    };

    /****************************************************************************
     * autogen section: ethosu device
     ****************************************************************************/
    struct ethosu_driver *driver = ethosu_reserve_driver();
    struct tvmgen_default_devices devices =
    {
        .ethos_u = driver,
    };

    ethosu_request_power(driver);
    /****************************************************************************
     * autogen section: Output tensor init
     ****************************************************************************/
    struct tvmgen_default_outputs outputs =
    {
        .StatefulPartitionedCall_0 = s_StatefulPartitionedCall_0_buffer,
    };

    printf("Running Inference \n");

    tvmgen_default_run(&inputs, &outputs, &devices);

    ethosu_release_power(driver);
    ethosu_release_driver(driver);
    // Calculate index of max value
    int8_t max_value = -128;
    int32_t max_index = -1;

    for (unsigned int i = 0; i < TVMGEN_DEFAULT_STATEFULPARTITIONEDCALL_0_SIZE; ++i)
    {
        if (s_StatefulPartitionedCall_0_buffer[i] > max_value)
        {
            max_value = s_StatefulPartitionedCall_0_buffer[i];
            max_index = i;
        }
    }

    printf("The image has been classified as '%s'\n", labels[max_index]);

    return 0;
}