#ifndef TVMGEN_DEFAULT_H_
#define TVMGEN_DEFAULT_H_
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \brief Input tensor serving_default_input_1_0 size (in bytes) for TVM module "default"
 */
#define TVMGEN_DEFAULT_SERVING_DEFAULT_INPUT_1_0_SIZE 150528
/*!
 * \brief Output tensor StatefulPartitionedCall_0 size (in bytes) for TVM module "default"
 */
#define TVMGEN_DEFAULT_STATEFULPARTITIONEDCALL_0_SIZE 1000
/*!
 * \brief Input tensor pointers for TVM module "default"
 */
struct tvmgen_default_inputs
{
    void *serving_default_input_1_0;
};

/*!
 * \brief Output tensor pointers for TVM module "default"
 */
struct tvmgen_default_outputs
{
    void *StatefulPartitionedCall_0;
};

/*!
 * \brief Device context pointers for TVM module "default"
 */
struct tvmgen_default_devices
{
    void *ethos_u;
};

/*!
 * \brief entrypoint function for TVM module "default"
 * \param inputs Input tensors for the module
 * \param outputs Output tensors for the module
 * \param devices Device context pointers for the module
 */
int32_t tvmgen_default_run(
    struct tvmgen_default_inputs *inputs,
    struct tvmgen_default_outputs *outputs,
    struct tvmgen_default_devices *devices
);

/*!
 * \brief get workspace buffer address
 * \return workspace buffer address
 */
uint8_t *tvmgen_get_workspace_address(void);

/*!
 * \brief Workspace size for TVM module "default"
 */
#define TVMGEN_DEFAULT_WORKSPACE_SIZE 804912

#ifdef __cplusplus
}
#endif

#endif // TVMGEN_DEFAULT_H_
