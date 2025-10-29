/******************************************************************************
 * @file     vio_NuMaker-M55M1.c
 * @brief    Virtual I/O implementation for board Nuvoton NuMaker-M55M1
 * @version  V1.0.0
 * @date     13. October 2025
 ******************************************************************************/
/*
 * Copyright (c) 2025 Arm Limited (or its affiliates).
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*! \page vio_NuMaker-M55M1 Physical I/O Mapping

The table below lists the physical I/O mapping of this CMSIS-Driver VIO implementation.

| Virtual I/O   | Variable       | Board component      | Pin
|:--------------|:---------------|:---------------------|:------
| vioBUTTON0    | vioSignalIn.0  | BTN_0                | PI11
| vioBUTTON1    | vioSignalIn.1  | BTN_1                | PH1
| vioLED0       | vioSignalOut.0 | LED_R                | PH4
| vioLED1       | vioSignalOut.1 | LED_Y                | PD6
| vioLED2       | vioSignalOut.2 | LED_G                | PD5
*/

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "cmsis_vio.h"

#include "RTE_Components.h"                 // Component selection
#include CMSIS_device_header

#if !defined CMSIS_VOUT || !defined CMSIS_VIN
#endif

// VIO input, output definitions
#ifndef VIO_VALUE_NUM
    #define VIO_VALUE_NUM           5U              // Number of values
#endif

// VIO input, output variables
static uint32_t vioSignalIn             __USED; // Memory for incoming signal
static uint32_t vioSignalOut            __USED; // Memory for outgoing signal
static int32_t  vioValue[VIO_VALUE_NUM] __USED; // Memory for value used in vioGetValue/vioSetValue

#if !defined CMSIS_VOUT
    // Add global user types, variables, functions here:

#endif

#if !defined CMSIS_VIN
    // Add global user types, variables, functions here:

#endif

// Initialize test input, output.
void vioInit(void)
{
#if !defined CMSIS_VOUT
    // Add user variables here:
#endif
#if !defined CMSIS_VIN
    // Add user variables here:
#endif
    vioSignalIn = 0U;
    vioSignalOut = 0U;
    memset(vioValue, 0, sizeof(vioValue));
#if !defined CMSIS_VOUT
    // Initialize LEDs pins
    // Configure pins: Output Mode
    CLK_EnableModuleClock(GPIOD_MODULE);
    CLK_EnableModuleClock(GPIOH_MODULE);
    GPIO_SetMode(PD, BIT5 | BIT6, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PH, BIT4, GPIO_MODE_OUTPUT);
#endif
#if !defined CMSIS_VIN
    CLK_EnableModuleClock(GPIOH_MODULE);
    CLK_EnableModuleClock(GPIOI_MODULE);
    // Enable Internal low speed RC clock
    CLK_EnableXtalRC(CLK_SRCCTL_LIRCEN_Msk);
    // Waiting for Internal low speed RC clock ready
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);
    // Set De-bounce Sampling Cycle Time
    GPIO_SET_DEBOUNCE_TIME(PH, GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_4);
    GPIO_SET_DEBOUNCE_TIME(PI, GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_4);
    GPIO_ENABLE_DEBOUNCE(PH, BIT1);
    GPIO_ENABLE_DEBOUNCE(PI, BIT11);
    // Initialize buttons pins
    // Configure pins: Input Mode
    GPIO_SetMode(PH, BIT1, GPIO_MODE_INPUT);
    GPIO_SetMode(PI, BIT11, GPIO_MODE_INPUT);
#endif
}

// Set signal output.
void vioSetSignal(uint32_t mask, uint32_t signal)
{
#if !defined CMSIS_VOUT
    // Add user variables here:
#endif
    vioSignalOut &= ~mask;
    vioSignalOut |= mask & signal;
#if !defined CMSIS_VOUT

    // Output signals to LEDs
    if ((mask & vioLED0) != 0U)
    {
        if ((signal & vioLED0) != 0U)
        {
            PH4 = 0;
        }
        else
        {
            PH4 = 1;
        }
    }

    if ((mask & vioLED1) != 0U)
    {
        if ((signal & vioLED1) != 0U)
        {
            PD6 = 0;
        }
        else
        {
            PD6 = 1;
        }
    }

    if ((mask & vioLED2) != 0U)
    {
        if ((signal & vioLED1) != 0U)
        {
            PD5 = 0;
        }
        else
        {
            PD5 = 1;
        }
    }

#endif
}

// Get signal input.
uint32_t vioGetSignal(uint32_t mask)
{
    uint32_t signal;
#if !defined CMSIS_VIN
    // Add user variables here:
#endif
#if !defined CMSIS_VIN

    // Get input signals from buttons
    if ((mask & vioBUTTON0) != 0U)
    {
        if (PI11 == 0U)
        {
            vioSignalIn |=  vioBUTTON0;
        }
        else
        {
            vioSignalIn &= ~vioBUTTON0;
        }
    }

    if ((mask & vioBUTTON1) != 0U)
    {
        if (PH1 == 0U)
        {
            vioSignalIn |=  vioBUTTON1;
        }
        else
        {
            vioSignalIn &= ~vioBUTTON1;
        }
    }

#endif
    signal = vioSignalIn;
    return (signal & mask);
}

// Set value output.
void vioSetValue(uint32_t id, int32_t value)
{
    uint32_t index = id;
#if !defined CMSIS_VOUT
    // Add user variables here:
#endif

    if (index >= VIO_VALUE_NUM)
    {
        return; /* return in case of out-of-range index */
    }

    vioValue[index] = value;
#if !defined CMSIS_VOUT
    // Add user code here:
#endif
}

// Get value input.
int32_t vioGetValue(uint32_t id)
{
    uint32_t index = id;
    int32_t value = 0;
#if !defined CMSIS_VIN
    // Add user variables here:
#endif

    if (index >= VIO_VALUE_NUM)
    {
        return value; /* return default in case of out-of-range index */
    }

#if !defined CMSIS_VIN
    // Add user code here:
    //   vioValue[index] = ...;
#endif
    value = vioValue[index];
    return value;
}
