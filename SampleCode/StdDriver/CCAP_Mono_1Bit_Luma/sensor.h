/**************************************************************************//**
 * @file     sensor.h
 * @version  V1.00
 * @brief    Sensor driver
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "NuMicro.h"

/* @return TRUE: Init ok, FALSE: Init failed */
typedef int32_t (*PFN_INIT_SENSOR_FUNC)(uint32_t u32Param);

typedef struct s_sensor_info
{
    char        m_strName[16];
    uint32_t    m_u32Polarity;
    uint32_t    m_u32InputFormat;
    uint16_t    m_u16Width;
    uint16_t    m_u16Height;
    PFN_INIT_SENSOR_FUNC    pfnInitSensor;
} S_SENSOR_INFO;

extern S_SENSOR_INFO g_sSensorHM01B0;
extern S_SENSOR_INFO g_sSensorNT99141;

#endif  // __SENSOR_H__
