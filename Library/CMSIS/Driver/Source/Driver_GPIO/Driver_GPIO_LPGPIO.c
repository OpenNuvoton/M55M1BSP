/*
 * Copyright (c) 2023 ARM Limited. All rights reserved.
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
 *
 * $Date:        21. March 2023
 * $Revision:    V1.0
 *
 * Project:      LPGPIO Driver for M55M1
 */

#include "NuMicro.h"
#ifdef _RTE_
    #include "RTE_Components.h"
#endif
/* Project can define PRJ_RTE_DEVICE_HEADER macro to include private or global RTE_Device.h. */
#ifdef   PRJ_RTE_DEVICE_HEADER
    #include PRJ_RTE_DEVICE_HEADER
#else
    #include "RTE_Device/RTE_Device.h"
#endif

#if (RTE_LPIO == 1)

// Maximum number of pins
#define LPGPIO_MAX_PINS_NUM       (8)

// Setup GPIO Interface
static int32_t GPIO_Setup(ARM_GPIO_Pin_t pin, ARM_GPIO_SignalEvent_t cb_event)
{
    if (pin >= LPGPIO_MAX_PINS_NUM)                 // If pin id is out-of-bounds
    {
        return ARM_GPIO_ERROR_PIN;
    }

    if (cb_event != NULL)
    {
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    CLK->LPGPIOCTL = CLK_LPGPIOCTL_LPGPIO0CKEN_Msk;

    switch (pin)
    {
        case 0:
            SET_LPIO0_PIN();
            break;

        case 1:
            SET_LPIO1_PIN();
            break;

        case 2:
            SET_LPIO2_PIN();
            break;

        case 3:
            SET_LPIO3_PIN();
            break;

        case 4:
            SET_LPIO4_PIN();
            break;

        case 5:
            SET_LPIO5_PIN();
            break;

        case 6:
            SET_LPIO6_PIN();
            break;

        case 7:
            SET_LPIO7_PIN();
            break;

        default:
            return ARM_GPIO_ERROR_PIN;
            break;
    }

    LPGPIO_SetMode(LPGPIO, BIT0, LPGPIO_MODE_INPUT);
    return ARM_DRIVER_OK;
}

// Set GPIO Direction
static int32_t GPIO_SetDirection(ARM_GPIO_Pin_t pin, ARM_GPIO_DIRECTION direction)
{
    if (pin >= LPGPIO_MAX_PINS_NUM)                 // If pin id is out-of-bounds
    {
        return ARM_GPIO_ERROR_PIN;
    }

    switch (direction)
    {
        case ARM_GPIO_INPUT:
            LPGPIO_SetMode(LPGPIO, BIT0 << pin, LPGPIO_MODE_INPUT);
            break;

        case ARM_GPIO_OUTPUT:
            LPGPIO_SetMode(LPGPIO, BIT0 << pin, LPGPIO_MODE_OUTPUT);
            break;

        default:
            return ARM_DRIVER_ERROR_PARAMETER;
            break;
    }

    return ARM_DRIVER_OK;
}

// Set GPIO Output Mode
static int32_t GPIO_SetOutputMode(ARM_GPIO_Pin_t pin, ARM_GPIO_OUTPUT_MODE mode)
{
    if (pin >= LPGPIO_MAX_PINS_NUM)                 // If pin id is out-of-bounds
    {
        return ARM_GPIO_ERROR_PIN;
    }

    switch (mode)
    {
        case ARM_GPIO_PUSH_PULL:
            LPGPIO_SetMode(LPGPIO, BIT0 << pin, LPGPIO_MODE_OUTPUT);
            break;

        case ARM_GPIO_OPEN_DRAIN:
            return ARM_DRIVER_ERROR_PARAMETER;

        default:
            return ARM_DRIVER_ERROR_PARAMETER;
            break;
    }

    return ARM_DRIVER_OK;
}

// Set GPIO Pull Resistor
static int32_t GPIO_SetPullResistor(ARM_GPIO_Pin_t pin, ARM_GPIO_PULL_RESISTOR resistor)
{
    (void) pin;
    (void) resistor;
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

// Set GPIO Event Trigger
static int32_t GPIO_SetEventTrigger(ARM_GPIO_Pin_t pin, ARM_GPIO_EVENT_TRIGGER trigger)
{
    (void) pin;
    (void) trigger;
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

// Set GPIO Output Level
static void GPIO_SetOutput(ARM_GPIO_Pin_t pin, uint32_t val)
{
    if (pin < LPGPIO_MAX_PINS_NUM)
    {
        if (val)
        {
            LPGPIO_SET_OUT_DATA(LPGPIO, (LPGPIO_GET_IN_DATA(LPGPIO) | (BIT0 << pin)));
        }
        else
        {
            LPGPIO_SET_OUT_DATA(LPGPIO, (LPGPIO_GET_IN_DATA(LPGPIO) & ~(BIT0 << pin)));
        }
    }
}

// Get GPIO Input Level
static uint32_t GPIO_GetInput(ARM_GPIO_Pin_t pin)
{
    if (pin < LPGPIO_MAX_PINS_NUM)
    {
        uint32_t   val = LPGPIO_GET_IN_DATA(LPGPIO);
        val = (val >> pin) & 0x01;
        return val;
    }
    else
    {
        return 0;
    }
}

// GPIO Driver access structure
ARM_DRIVER_GPIO Driver_GPIO1 =
{
    GPIO_Setup,
    GPIO_SetDirection,
    GPIO_SetOutputMode,
    GPIO_SetPullResistor,
    GPIO_SetEventTrigger,
    GPIO_SetOutput,
    GPIO_GetInput
};

#endif // #if (RTE_LPIO == 1)
