#include "tvm/runtime/c_runtime_api.h"
#ifdef __cplusplus
extern "C" {
#endif
__attribute__((section(".bss.noinit.tvm"), aligned(16)))
static uint8_t global_workspace[74480];
#include <tvmgen_default.h>
TVM_DLL int32_t tvmgen_default___tvm_main__(void *input_1_int8, void *output0, uint8_t *global_workspace_0_var, void *ethos_u);
int32_t tvmgen_default_run(struct tvmgen_default_inputs *inputs, struct tvmgen_default_outputs *outputs, struct tvmgen_default_devices *devices)
{
    return tvmgen_default___tvm_main__(inputs->input_1_int8, outputs->Identity_int8, global_workspace, devices->ethos_u);
}

uint8_t *tvmgen_get_workspace_address(void)
{
    return &global_workspace[0];
}
#ifdef __cplusplus
}
#endif
;