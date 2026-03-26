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
 * Project:      GPIO Driver for M55M1
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

// Maximum number of ports
#ifndef GPIO_MAX_PORTS_NUM
    #define GPIO_MAX_PORTS_NUM      (10U)
#endif

// Maximum number of pins
#ifndef GPIO_MAX_PINS_NUM
    #define GPIO_MAX_PINS_NUM       ((GPIO_MAX_PORTS_NUM)*16U)
#endif

// Pin mapping
//   0 ..  15: PA 0..15
//  16 ..  31: PB 0..15
//  32 ..  47: PC 0..15
//  48 ..  63: PD 0..15
//  64 ..  79: PE 0..15
//  80 ..  95: PF 0..15
//  96 .. 111: PG 0..15
// 112 .. 127: PH 0..15
// 128 .. 143: PI 0..15
// 144 .. 159: PJ 0..15

static const uint32_t PinMapping[GPIO_MAX_PORTS_NUM] =
{
    0x0000FFFFU,  // PA  0..15, PA has 16 pins on port.
    0x0000FFFFU,  // PB  0..15, PB has 16 pins on port.
    0x00007FFFU,  // PC  0..14, PC has 15 pins on port.
    0x00007FFFU,  // PD  0..14, PD has 15 pins on port.
    0x0000FFFFU,  // PE  0..15, PE has 16 pins on port.
    0x00000FFFU,  // PF  0..11, PF has 12 pins on port.
    0x0000FFFFU,  // PG  0..15, PG has 16 pins on port.
    0x0000FFFFU,  // PH  0..15, PH has 16 pins on port.
    0x0000FFC0U,  // PI  6..15, PI has 10 pins on port.
    0x00003FFFU,  // PJ  0..13, PJ has 14 pins on port.
};

// GPIOx Base Pointers
static uint32_t const GPIO_Base[GPIO_MAX_PORTS_NUM] =
{
    GPIOA_BASE, GPIOB_BASE, GPIOC_BASE, GPIOD_BASE, GPIOE_BASE,
    GPIOF_BASE, GPIOG_BASE, GPIOH_BASE, GPIOI_BASE, GPIOJ_BASE,
};

// GPIOx IRQ Numbers
static IRQn_Type const GPIO_IRQn[GPIO_MAX_PORTS_NUM] =
{
    GPA_IRQn, GPB_IRQn, GPC_IRQn, GPD_IRQn, GPE_IRQn,
    GPF_IRQn, GPG_IRQn, GPH_IRQn, GPI_IRQn, GPJ_IRQn,
};


// Signal Event callback functions
static ARM_GPIO_SignalEvent_t SignalEvent[GPIO_MAX_PORTS_NUM][16];

typedef struct
{
    GPIO_T    *port;
    uint32_t   u32Port;
    uint32_t   u32Pin;
    uint32_t   u32PinMask;
} GPIODriverInput_t;

static void GPIOx_IRQHandler(uint32_t u32Port)
{
    GPIO_T *port;
    uint32_t i, u32INTSRC, u32INTEN;
    uint32_t   event;
    port = (GPIO_T *)GPIO_Base[u32Port];
    u32INTSRC = port->INTSRC;

    for (i = 0; i < 16; i++)
    {
        if (u32INTSRC & (BIT0 << i))
        {
            if (SignalEvent[u32Port][i] != NULL)
            {
                u32INTEN = port->INTEN & (0x00010001ul << i);

                if ((u32INTEN & 0x0000FFFF) == 0)
                {
                    event = ARM_GPIO_EVENT_RISING_EDGE;
                }
                else if ((u32INTEN & 0xFFFF0000) == 0)
                {
                    event = ARM_GPIO_EVENT_FALLING_EDGE;
                }
                else
                {
                    event = ARM_GPIO_EVENT_EITHER_EDGE;
                }

                SignalEvent[u32Port][i](u32Port * 16 + i, event);
            }

            port->INTSRC = (BIT0 << i);
            u32INTSRC &= ~(BIT0 << i);

            if (u32INTSRC == 0)
            {
                break;
            }
        }
    }
}

void GPA_IRQHandler(void)
{
    GPIOx_IRQHandler(0);
}

void GPB_IRQHandler(void)
{
    GPIOx_IRQHandler(1);
}

void GPC_IRQHandler(void)
{
    GPIOx_IRQHandler(2);
}

void GPD_IRQHandler(void)
{
    GPIOx_IRQHandler(3);
}

void GPE_IRQHandler(void)
{
    GPIOx_IRQHandler(4);
}

void GPF_IRQHandler(void)
{
    GPIOx_IRQHandler(5);
}

void GPG_IRQHandler(void)
{
    GPIOx_IRQHandler(6);
}

void GPH_IRQHandler(void)
{
    GPIOx_IRQHandler(7);
}

void GPI_IRQHandler(void)
{
    GPIOx_IRQHandler(8);
}

void GPJ_IRQHandler(void)
{
    GPIOx_IRQHandler(9);
}

static int32_t PIN_IS_AVAILABLE(ARM_GPIO_Pin_t pin, GPIODriverInput_t *arg)
{
    if (pin >= GPIO_MAX_PINS_NUM)   // If pin id is out-of-bounds
    {
        return ARM_GPIO_ERROR_PIN;
    }

    uint32_t u32Port = pin >> 4;
    uint32_t u32Pin  = pin & 0x0FU;
    uint32_t u32PinMask = (1U << u32Pin);

    if ((PinMapping[u32Port] & u32PinMask) == 0U)
    {
        return ARM_GPIO_ERROR_PIN;
    }

    arg->u32Port = u32Port;
    arg->u32Pin = u32Pin;
    arg->u32PinMask = u32PinMask;
    arg->port = (GPIO_T *)GPIO_Base[u32Port];
    return ARM_DRIVER_OK;
}

// Setup GPIO Interface
static int32_t GPIO_Setup(ARM_GPIO_Pin_t pin, ARM_GPIO_SignalEvent_t cb_event)
{
    GPIODriverInput_t arg;

    if (PIN_IS_AVAILABLE(pin, &arg) == ARM_GPIO_ERROR_PIN)
    {
        return ARM_GPIO_ERROR_PIN;
    }

    CLK->GPIOCTL |= (1U << arg.u32Port);
    GPIO_DisableInt(arg.port, arg.u32Pin);
    GPIO_SetMode(arg.port, arg.u32PinMask, GPIO_MODE_INPUT);
    SignalEvent[arg.u32Port][arg.u32Pin] = cb_event;

    if (cb_event != NULL)
    {
        NVIC_EnableIRQ(GPIO_IRQn[arg.u32Port]);
    }

    return ARM_DRIVER_OK;
}

// Set GPIO Direction
static int32_t GPIO_SetDirection(ARM_GPIO_Pin_t pin, ARM_GPIO_DIRECTION direction)
{
    GPIODriverInput_t arg;

    if (PIN_IS_AVAILABLE(pin, &arg) == ARM_GPIO_ERROR_PIN)
    {
        return ARM_GPIO_ERROR_PIN;
    }

    switch (direction)
    {
        case ARM_GPIO_INPUT:
            GPIO_SetMode(arg.port, arg.u32PinMask, GPIO_MODE_INPUT);
            break;

        case ARM_GPIO_OUTPUT:
            GPIO_SetMode(arg.port, arg.u32PinMask, GPIO_MODE_OUTPUT);
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
    GPIODriverInput_t arg;

    if (PIN_IS_AVAILABLE(pin, &arg) == ARM_GPIO_ERROR_PIN)
    {
        return ARM_GPIO_ERROR_PIN;
    }

    switch (mode)
    {
        case ARM_GPIO_PUSH_PULL:
            GPIO_SetMode(arg.port, arg.u32PinMask, GPIO_MODE_OUTPUT);
            break;

        case ARM_GPIO_OPEN_DRAIN:
            GPIO_SetMode(arg.port, arg.u32PinMask, GPIO_MODE_OPEN_DRAIN);
            break;

        default:
            return ARM_DRIVER_ERROR_PARAMETER;
            break;
    }

    return ARM_DRIVER_OK;
}

// Set GPIO Pull Resistor
static int32_t GPIO_SetPullResistor(ARM_GPIO_Pin_t pin, ARM_GPIO_PULL_RESISTOR resistor)
{
    GPIODriverInput_t arg;

    if (PIN_IS_AVAILABLE(pin, &arg) == ARM_GPIO_ERROR_PIN)
    {
        return ARM_GPIO_ERROR_PIN;
    }

    switch (resistor)
    {
        case ARM_GPIO_PULL_NONE:
            GPIO_SetPullCtl(arg.port, arg.u32PinMask, GPIO_PUSEL_DISABLE);
            break;

        case ARM_GPIO_PULL_UP:
            GPIO_SetPullCtl(arg.port, arg.u32PinMask, GPIO_PUSEL_PULL_UP);
            break;

        case ARM_GPIO_PULL_DOWN:
            GPIO_SetPullCtl(arg.port, arg.u32PinMask, GPIO_PUSEL_PULL_DOWN);
            break;

        default:
            return ARM_DRIVER_ERROR_PARAMETER;
            break;
    }

    return ARM_DRIVER_OK;
}

// Set GPIO Event Trigger
static int32_t GPIO_SetEventTrigger(ARM_GPIO_Pin_t pin, ARM_GPIO_EVENT_TRIGGER trigger)
{
    GPIODriverInput_t arg;

    if (PIN_IS_AVAILABLE(pin, &arg) == ARM_GPIO_ERROR_PIN)
    {
        return ARM_GPIO_ERROR_PIN;
    }

    switch (trigger)
    {
        case ARM_GPIO_TRIGGER_NONE:
            GPIO_DisableInt(arg.port, arg.u32Pin);
            break;

        case ARM_GPIO_TRIGGER_RISING_EDGE:
            GPIO_EnableInt(arg.port, arg.u32Pin, GPIO_INT_RISING);
            break;

        case ARM_GPIO_TRIGGER_FALLING_EDGE:
            GPIO_EnableInt(arg.port, arg.u32Pin, GPIO_INT_FALLING);
            break;

        case ARM_GPIO_TRIGGER_EITHER_EDGE:
            GPIO_EnableInt(arg.port, arg.u32Pin, GPIO_INT_BOTH_EDGE);
            break;

        default:
            return ARM_DRIVER_ERROR_PARAMETER;
            break;
    }

    return ARM_DRIVER_OK;
}

// Set GPIO Output Level
static void GPIO_SetOutput(ARM_GPIO_Pin_t pin, uint32_t val)
{
    GPIODriverInput_t arg;

    if (PIN_IS_AVAILABLE(pin, &arg) == ARM_DRIVER_OK)
    {
        if (val)
        {
            arg.port->DOUT |= arg.u32PinMask;
        }
        else
        {
            arg.port->DOUT &= ~arg.u32PinMask;
        }
    }
}

// Get GPIO Input Level
static uint32_t GPIO_GetInput(ARM_GPIO_Pin_t pin)
{
    GPIODriverInput_t arg;

    if (PIN_IS_AVAILABLE(pin, &arg) == ARM_DRIVER_OK)
    {
        uint32_t   val = arg.port->PIN;
        val = ((val & arg.u32PinMask) >> arg.u32Pin);
        return val;
    }
    else
    {
        return 0;
    }
}

// GPIO Driver access structure
ARM_DRIVER_GPIO Driver_GPIO0 =
{
    GPIO_Setup,
    GPIO_SetDirection,
    GPIO_SetOutputMode,
    GPIO_SetPullResistor,
    GPIO_SetEventTrigger,
    GPIO_SetOutput,
    GPIO_GetInput
};
