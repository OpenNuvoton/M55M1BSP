// tvm target: cmsis-nn
#define TVM_EXPORTS
#include "tvm/runtime/c_runtime_api.h"
#include "tvm/runtime/c_backend_api.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <dlpack/dlpack.h>
#include <arm_nnfunctions.h>
#include <arm_nn_types.h>
#include <arm_nn_math_types.h>
#ifdef __cplusplus
    extern "C"
#endif
TVM_DLL int32_t tvmgen_default_cmsis_nn_main_0(int8_t *input_, int8_t *output_, uint8_t *global_workspace_2_var)
{
    arm_softmax_s8(input_, 1, 2, 2011586560, 20, -1984, output_);
    return 0;
}

