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
// Program Command
//------------------------------------------------------------------------------
/* 0x02h : CMD_NORMAL_PAGE_PROGRAM Command Phase Table */
SPIM_PHASE_T gsMt02hWrCMD =
{
    CMD_NORMAL_PAGE_PROGRAM,                                                    //Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,                       //Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,                       //Address Phase
    PHASE_NORMAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR, SPIM_OP_DISABLE,  //Data Phase
    0,
};

/* 0x12h : CMD_NORMAL_PAGE_PROGRAM_4B Command Phase Table */
SPIM_PHASE_T gsMt12hWrCMD =
{
    CMD_NORMAL_PAGE_PROGRAM_4B,                                                 //Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,                       //Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_32, PHASE_DISABLE_DTR,                       //Address Phase
    PHASE_NORMAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR, SPIM_OP_DISABLE,  //Data Phase
    0,
};

/* 0xC2h : CMD_OCTAL_EX_PAGE_PROG_MICRON Command Phase Table */
SPIM_PHASE_T gsMtC2hWrCMD =
{
    CMD_OCTAL_EX_PAGE_PROG_MICRON,                                              //Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,                       //Command Phase
    PHASE_OCTAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,                        //Address Phase
    PHASE_OCTAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR, SPIM_OP_DISABLE,   //Data Phase
    0,
};

/* 0x8Eh : CMD_OCTAL_EX_PAGE_PROG_MICRON_4B Command Phase Table */
SPIM_PHASE_T gsMt8EhWrCMD =
{
    CMD_OCTAL_EX_PAGE_PROG_MICRON_4B,                                           //Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,                       //Command Phase
    PHASE_OCTAL_MODE, PHASE_WIDTH_32, PHASE_DISABLE_DTR,                        //Address Phase
    PHASE_OCTAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR, SPIM_OP_DISABLE,   //Data Phase
    0,
};

//------------------------------------------------------------------------------
// Read Command
//------------------------------------------------------------------------------
/* 0x0Bh : CMD_DMA_FAST_READ Command Phase Table */
SPIM_PHASE_T gsMt0BhRdCMD =
{
    CMD_DMA_FAST_READ,                                                          //Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,                       //Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,                       //Address Phase
    PHASE_NORMAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR, SPIM_OP_DISABLE,  //Data Phase
    8,
};

/* 0x0Ch : CMD_DMA_FAST_READ_4B Command Phase Table */
SPIM_PHASE_T gsMt0ChRdCMD =
{
    CMD_DMA_FAST_READ_4B,                                                       //Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,                       //Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_32, PHASE_DISABLE_DTR,                       //Address Phase
    PHASE_NORMAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR, SPIM_OP_DISABLE,  //Data Phase
    8,
};

/* 0x8Bh : CMD_OCTAL_FAST_READ_OUTPUT Single Data Rate Command Phase Table */
SPIM_PHASE_T gsMt8BhRdCMD =
{
    CMD_OCTAL_FAST_READ_OUTPUT,                                                 //Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,                       //Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,                       //Address Phase
    PHASE_OCTAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR, SPIM_OP_DISABLE,   //Data Phase
    8,
};

/* 0x7Ch : CMD_OCTAL_FAST_READ_OUTPUT_4B Command Phase Table */
SPIM_PHASE_T gsMt7ChRdCMD =
{
    CMD_OCTAL_FAST_READ_OUTPUT_4B,                                              //Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,                       //Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_32, PHASE_DISABLE_DTR,                       //Address Phase
    PHASE_OCTAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR, SPIM_OP_DISABLE,   //Data Phase
    8,
};

/* 0xCBh : CMD_OCTAL_FAST_IO_READ Command Phase Table */
SPIM_PHASE_T gsMtCBhRdCMD =
{
    CMD_OCTAL_FAST_IO_READ,                                                     //Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,                       //Command Phase
    PHASE_OCTAL_MODE, PHASE_WIDTH_24, PHASE_DISABLE_DTR,                        //Address Phase
    PHASE_OCTAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR, SPIM_OP_DISABLE,   //Data Phase
    16,
};

/* 0xCCh : CMD_OCTAL_FAST_IO_READ_4B Command Phase Table */
SPIM_PHASE_T gsMtCChRdCMD =
{
    CMD_OCTAL_FAST_IO_READ_4B,                                                  //Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,                       //Command Phase
    PHASE_OCTAL_MODE, PHASE_WIDTH_32, PHASE_DISABLE_DTR,                        //Address Phase
    PHASE_OCTAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR, SPIM_OP_DISABLE,   //Data Phase
    16,
};

/* 0x9Dh : CMD_OCTAL_DDR_FAST_READ_OUTPUT Command Phase Table */
SPIM_PHASE_T gsMt9DhRdCMD =
{
    CMD_OCTAL_DDR_FAST_READ_OUTPUT,                                             //Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,                       //Command Phase
    PHASE_NORMAL_MODE, PHASE_WIDTH_24, PHASE_ENABLE_DTR,                        //Address Phase
    PHASE_OCTAL_MODE, PHASE_ORDER_MODE0,  PHASE_ENABLE_DTR, SPIM_OP_DISABLE,    //Data Phase
    8,
};

/* 0xFDh : CMD_OCTAL_DDR_FAST_IO_READ Command Phase Table */
SPIM_PHASE_T gsMtFDhRdCMD =
{
    CMD_OCTAL_DDR_FAST_IO_READ,                                                 //Command Code
    PHASE_NORMAL_MODE, PHASE_WIDTH_8,  PHASE_DISABLE_DTR,                       //Command Phase
    PHASE_OCTAL_MODE, PHASE_WIDTH_32, PHASE_ENABLE_DTR,                         //Address Phase
    PHASE_OCTAL_MODE, PHASE_ORDER_MODE0,  PHASE_ENABLE_DTR, SPIM_OP_DISABLE,    //Data Phase
    16,
};

//------------------------------------------------------------------------------
// Octal DDR Mode Command
// @note ALL Command support DDR mode.
//       In octal DDR mode the command width is set to 16 bits,
//       the address width is set to 32bit, and the command, address,
//       and data phases set PHASE_OCTAL_MODE and PHASE_ENABLE_DTR.
//       The number of read command dummy cycles is set to 16.
//------------------------------------------------------------------------------
/* 0x02h : CMD_NORMAL_PAGE_PROGRAM Octal Double Data Rate Command Phase Table */
SPIM_PHASE_T gsMt02hWrDDRCMD =
{
    CMD_NORMAL_PAGE_PROGRAM,                                                    //Command Code
    PHASE_OCTAL_MODE, PHASE_WIDTH_16,  PHASE_ENABLE_DTR,                        //Command Phase
    PHASE_OCTAL_MODE, PHASE_WIDTH_32, PHASE_ENABLE_DTR,                         //Address Phase
    PHASE_OCTAL_MODE, PHASE_ORDER_MODE0,  PHASE_ENABLE_DTR, SPIM_OP_DISABLE,    //Data Phase
    0,
};

/* 0x8Bh : CMD_OCTAL_FAST_READ_OUTPUT Octal Double Data Rate Command Phase Table */
SPIM_PHASE_T gsMt8BhRdDDRCMD =
{
    CMD_OCTAL_FAST_READ_OUTPUT,                                                 //Command Code
    PHASE_OCTAL_MODE, PHASE_WIDTH_16,  PHASE_ENABLE_DTR,                        //Command Phase
    PHASE_OCTAL_MODE, PHASE_WIDTH_32, PHASE_ENABLE_DTR,                         //Address Phase
    PHASE_OCTAL_MODE, PHASE_ORDER_MODE0,  PHASE_ENABLE_DTR, SPIM_OP_DISABLE,    //Data Phase
    16,
};
