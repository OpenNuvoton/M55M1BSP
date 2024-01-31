/**************************************************************************//**
 * @file     MainClassify.h
 * @version  V1.00
 * @brief    Board and target application related macro
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef  __BOARD_M55M1_H__
#define __BOARD_M55M1_H__

//
// Includes
//
#include <stdint.h>
#include "MPU6500.h"
//
// Defines
//
#define AD_SIZE                      64
#define AD_WIDTH                  AD_SIZE
#define AD_HEIGHT                AD_SIZE
#define PREVIEW_WIDTH               (4* AD_SIZE)
#define PREVIEW_HEIGHT              (4* AD_SIZE)
#define   AD_ARENA                (0x14000)

#define IMU_DATAIN_LEN                 200
#define IMU_DATAIN_AXES_NUM         3
#define IMU_DATAIN_SIZE                 IMU_DATAIN_LEN * IMU_DATAIN_AXES_NUM
//Enumeration

//Struct

//
// Functions
//
#ifdef  __cplusplus
extern  "C" {
#endif

#ifdef  __cplusplus
}
#endif
#endif  // __BOARD_M55M1_H__ 
