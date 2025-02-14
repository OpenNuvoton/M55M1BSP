/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to protect VWW model by NPU XOM Region.
 *           VWW model was compiled by TVMC
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "BoardInit.hpp"
#include "ethosu_driver.h"

#include "tvmgen_default.h"

#include "labels.h"
#include "InputFiles.hpp"

/****************************************************************************
 * autogen section: Output tensor data
 ****************************************************************************/
__attribute__((aligned(16), section(".bss.noinit.tvm"))) static int8_t s_Identity_int8_buffer[TVMGEN_DEFAULT_IDENTITY_INT8_SIZE];
__attribute__((aligned(16), section(".bss.noinit.tvm"))) static int8_t s_input_1_int8_buffer[TVMGEN_DEFAULT_INPUT_1_INT8_SIZE];

/****************************************************************************
 * InferenceJob
 ****************************************************************************/
int main(void)
{

    /* Initialise the hardware resource(UART, NPU, HyperFlash) */
    BoardInit();

#if defined(XOM_NPU_PROTECT)

#define XOM_PAGE_SIZE 8192
#define XOM_BASE_ALIGN  XOM_PAGE_SIZE
#define CMS_HEADER_SIZE 32

    uint32_t commandStreamAddr;
    uint32_t commandStreamSize;
    uint32_t weightAddr;
    uint32_t weightSize;

    tvmgen_get_default_ethos_u_main_0_address(&commandStreamAddr, &commandStreamSize, &weightAddr, &weightSize);

    //Check NPU XOM3 status
    uint32_t u32Status;
    uint32_t u32XOMR3Base = (uint32_t)commandStreamAddr + CMS_HEADER_SIZE;
    uint32_t u32XOMR3Size = ((uint32_t)weightAddr + weightSize - u32XOMR3Base);
    uint32_t u32XOMR3Pages = u32XOMR3Size / XOM_PAGE_SIZE;

    if (u32XOMR3Size & (XOM_PAGE_SIZE - 1))
        u32XOMR3Pages ++;

    /* Unlock protected registers */
    SYS_UnlockReg();

    FMC_Open();                 /* Enable FMC ISP function    */
    FMC_ENABLE_AP_UPDATE();     /* Enable APROM update        */
    FMC_ENABLE_CFG_UPDATE();    /* Enable APROM update        */

    printf("XOM Status = 0x%X\n", FMC->XOMSTS);

    /* Config XOMR3 */
    if (FMC_GetXOMState(XOMR3) == 0)
    {
        printf("Config XOM3 base %x \n", u32XOMR3Base);
        printf("Config XOM3 pages %x \n", u32XOMR3Pages);

        ////config XOMR3CTRL and XOMR3DRBOUNDARY
        u32Status = FMC_ConfigNPUXOM(u32XOMR3Base, u32XOMR3Pages, (uint32_t)weightAddr);

        if (u32Status)
            printf("Failed config XOMR3 NPU config...\n");
        else
            printf("Passed config XOMR3 NPU config...\n");

        /* Reset chip to enable XOM region. */
        SYS_ResetChip();

        while (1) {};
    }

#endif

    printf("\nThis sample code run inference by TVM runtime\n");

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
    printf("copy input image to sram \n");

    uint8_t *pu8ImageData = (uint8_t *)im0;

    if (TVMGEN_DEFAULT_INPUT_1_INT8_SIZE != IMAGE_DATA_SIZE)
    {
        printf("The image size is not match model requirment %d \n", TVMGEN_DEFAULT_INPUT_1_INT8_SIZE);
        return 0;
    }

    for (unsigned int i = 0; i < TVMGEN_DEFAULT_INPUT_1_INT8_SIZE; i++)
    {
        s_input_1_int8_buffer[i] = pu8ImageData[i] - 128;
    }

    struct tvmgen_default_inputs inputs =
    {
        .input_1_int8 = (void *)s_input_1_int8_buffer,
    };

    /****************************************************************************
     * autogen section: ethosu device
     ****************************************************************************/
    struct ethosu_driver *driver = ethosu_reserve_driver();

    struct tvmgen_default_devices devices =
    {
        .ethos_u = driver,
    };

    ethosu_release_driver(driver);

    /****************************************************************************
     * autogen section: Output tensor init
     ****************************************************************************/
    struct tvmgen_default_outputs outputs =
    {
        .Identity_int8 = s_Identity_int8_buffer,
    };

    printf("Running Inference \n");

    tvmgen_default_run(&inputs, &outputs, &devices);

    ethosu_release_driver(driver);

    // Calculate index of max value
    int8_t max_value = -128;

    int32_t max_index = -1;

    for (unsigned int i = 0; i < TVMGEN_DEFAULT_IDENTITY_INT8_SIZE; ++i)
    {

        if (s_Identity_int8_buffer[i] > max_value)
        {
            max_value = s_Identity_int8_buffer[i];
            max_index = i;
        }
    }

    printf("The image has been classified as '%s'\n", labels[max_index]);

    return 0;
}