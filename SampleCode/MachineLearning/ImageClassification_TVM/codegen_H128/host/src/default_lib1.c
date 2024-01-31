// tvm target: c -keys=arm_cpu,cpu -mcpu=cortex-m55
#define TVM_EXPORTS
#include "tvm/runtime/c_runtime_api.h"
#include "tvm/runtime/c_backend_api.h"
#include <math.h>
#include <stdbool.h>
#include <tvm_ethosu_runtime.h>
#ifdef __cplusplus
    extern "C"
#endif
TVM_DLL int32_t TVMDeviceEthosUActivate(void *device_context_ethos_u);
#ifdef __cplusplus
    extern "C"
#endif
TVM_DLL int32_t TVMDeviceEthosUOpen(void *device_context_ethos_u);
#ifdef __cplusplus
    extern "C"
#endif
TVM_DLL int32_t tvmgen_default_tvmgen_default_ethos_u_main_0(int8_t *serving_default_input_1_0_buffer_var, int8_t *sid_1_let, uint8_t *global_workspace_0_var, void *device_context_ethos_u);
#ifdef __cplusplus
    extern "C"
#endif
TVM_DLL int32_t TVMDeviceEthosUClose(void *device_context_ethos_u);
#ifdef __cplusplus
    extern "C"
#endif
TVM_DLL int32_t tvmgen_default_cmsis_nn_main_0(int8_t *sid_1_let, int8_t *StatefulPartitionedCall_0_buffer_var, uint8_t *global_workspace_0_var);
#ifdef __cplusplus
    extern "C"
#endif
TVM_DLL int32_t TVMDeviceEthosUDeactivate(void *device_context_ethos_u);
#ifdef __cplusplus
    extern "C"
#endif
TVM_DLL int32_t tvmgen_default___tvm_main__(int8_t *serving_default_input_1_0_buffer_var, int8_t *StatefulPartitionedCall_0_buffer_var, uint8_t *global_workspace_0_var, void *device_context_ethos_u)
{
    if (TVMDeviceEthosUActivate(device_context_ethos_u) != 0) return -1;

    void *sid_1_let = (&(global_workspace_0_var[0]));

    if (TVMDeviceEthosUOpen(device_context_ethos_u) != 0) return -1;

    if (tvmgen_default_tvmgen_default_ethos_u_main_0(serving_default_input_1_0_buffer_var, sid_1_let, global_workspace_0_var, device_context_ethos_u) != 0) return -1;

    if (TVMDeviceEthosUClose(device_context_ethos_u) != 0) return -1;

    if (tvmgen_default_cmsis_nn_main_0(sid_1_let, StatefulPartitionedCall_0_buffer_var, global_workspace_0_var) != 0) return -1;

    if (TVMDeviceEthosUDeactivate(device_context_ethos_u) != 0) return -1;

    return 0;
}

