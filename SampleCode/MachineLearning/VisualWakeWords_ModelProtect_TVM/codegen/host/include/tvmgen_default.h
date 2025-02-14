#ifndef TVMGEN_DEFAULT_H_
#define TVMGEN_DEFAULT_H_
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \brief Input tensor input_1_int8 size (in bytes) for TVM module "default"
 */
#define TVMGEN_DEFAULT_INPUT_1_INT8_SIZE 27648
/*!
 * \brief Output tensor Identity_int8 size (in bytes) for TVM module "default"
 */
#define TVMGEN_DEFAULT_IDENTITY_INT8_SIZE 2
/*!
 * \brief Input tensor pointers for TVM module "default"
 */
struct tvmgen_default_inputs
{
    void *input_1_int8;
};

/*!
 * \brief Output tensor pointers for TVM module "default"
 */
struct tvmgen_default_outputs
{
    void *Identity_int8;
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
 * \brief get the command stream/weight address and size of ethosu 0 operator
 * \return workspace buffer address
 */
int32_t tvmgen_get_default_ethos_u_main_0_address(
    uint32_t *cms_addr,
    uint32_t *cms_size,
    uint32_t *weight_addr,
    uint32_t *weight_size
);

/*!
 * \brief Workspace size for TVM module "default"
 */
#define TVMGEN_DEFAULT_WORKSPACE_SIZE 74480

#ifdef __cplusplus
}
#endif

#endif // TVMGEN_DEFAULT_H_
