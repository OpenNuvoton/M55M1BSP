/**************************************************************************//**
* @file     main.c
* @version  V1.00
* @brief    Create SPI flash command phase table.
*
* @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

//------------------------------------------------------------------------------
/* 0x02h : CMD_NORMAL_PAGE_PROGRAM Command Phase Table */
SPIM_PHASE_T gsWb02hWrCMD =
{
    CMD_NORMAL_PAGE_PROGRAM,                                    //Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,       //Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,       //Address Phase
    PHASE_NORMAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR,   //Data Phase
    0,
};

/* 0x12h : CMD_NORMAL_PAGE_PROGRAM_4B Command Phase Table */
SPIM_PHASE_T gsWb12hWrCMD =
{
    CMD_NORMAL_PAGE_PROGRAM_4B,                                 //Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,       //Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_32, PHASE_DISABLE_DTR,       //Address Phase
    PHASE_NORMAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR,   //Data Phase
    0,
};

/* 0x0B: CMD_DMA_FAST_READ Command Phase Table */
SPIM_PHASE_T gsWb0BhRdCMD =
{
    CMD_DMA_FAST_READ,                                          // Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR,        // Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,       // Address Phase
    PHASE_NORMAL_MODE, PHASE_ORDER_MODE0, PHASE_DISABLE_DTR,    // Data Phase
    8,                                                          // Dummy Cycle Phase
};

/* 0xBB: CMD_DMA_FAST_DUAL_READ Command Phase Table */
SPIM_PHASE_T gsWbBBhRdCMD =
{
    CMD_DMA_FAST_DUAL_READ,                                                    // Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR,                       // Command Phase
    PHASE_DUAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,                        // Address Phase
    PHASE_DUAL_MODE, PHASE_ORDER_MODE0, PHASE_DISABLE_DTR,                     // Data Phase
    0,                                                                         // Dummy Cycle Phase
    PHASE_ENABLE_CONT_READ, PHASE_DUAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR, // Read Mode Phase
};

/* 0xBC: CMD_DMA_FAST_DUAL_READ_4B Command Phase Table */
SPIM_PHASE_T gsWbBChRdCMD =
{
    CMD_DMA_FAST_DUAL_READ_4B,                                                 // Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR,                       // Command Phase
    PHASE_DUAL_MODE, PHASE_WIDTH_32, PHASE_DISABLE_DTR,                        // Address Phase
    PHASE_DUAL_MODE, PHASE_ORDER_MODE0, PHASE_DISABLE_DTR,                     // Data Phase
    0,                                                                         // Dummy Cycle Phase
    PHASE_ENABLE_CONT_READ, PHASE_DUAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR, // Read Mode Phase
};

/* 0xEB: CMD_DMA_FAST_QUAD_READ Command Phase Table */
SPIM_PHASE_T gsWbEBhRdCMD =
{
    CMD_DMA_FAST_QUAD_READ,                                                    // Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR,                       // Command Phase
    PHASE_QUAD_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,                        // Address Phase
    PHASE_QUAD_MODE, PHASE_ORDER_MODE0, PHASE_DISABLE_DTR,                     // Data Phase
    4,                                                                         // Dummy Cycle Phase
    PHASE_ENABLE_CONT_READ, PHASE_QUAD_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR, // Read Mode Phase
};

/* 0xEC: CMD_DMA_FAST_QUAD_READ_4B Command Phase Table */
SPIM_PHASE_T gsWbEChRdCMD =
{
    CMD_DMA_FAST_QUAD_READ_4B,                                                 // Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR,                       // Command Phase
    PHASE_QUAD_MODE, PHASE_WIDTH_32, PHASE_DISABLE_DTR,                        // Address Phase
    PHASE_QUAD_MODE, PHASE_ORDER_MODE0, PHASE_DISABLE_DTR,                     // Data Phase
    4,                                                                         // Dummy Cycle Phase
    PHASE_ENABLE_CONT_READ, PHASE_QUAD_MODE, PHASE_WIDTH_8, PHASE_DISABLE_DTR, // Read Mode Phase
};
