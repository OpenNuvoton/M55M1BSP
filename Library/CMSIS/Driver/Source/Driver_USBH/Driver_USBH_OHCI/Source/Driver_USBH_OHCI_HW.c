/*
 * Copyright (c) 2024 Arm Limited. All rights reserved.
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
 * -----------------------------------------------------------------------------
 *
 * $Date:       28. May 2024
 * $Revision:   V1.0
 *
 * Project:     USB Host OHCI Controller Driver Hardware-specific template
 *
 * -----------------------------------------------------------------------------
 */

#ifdef _RTE_
    #include "RTE_Components.h"
#endif
#ifdef    PRJ_RTE_DEVICE_HEADER
    #include  PRJ_RTE_DEVICE_HEADER
#else
    #include "RTE_Device/RTE_Device.h"
#endif

#include "misc.h"

#include "NuMicro.h"

#include "Driver_USBH_OHCI_Config.h"
#include "Driver_USBH_OHCI_HW.h"


#if    (USBH1_OHCI_ENABLED == 1)
    #define USBH_OHCI_TT_INSTANCES         (2U)
#else
    #define USBH_OHCI_TT_INSTANCES         (1U)
#endif

#define OHCI_HW_INITIALIZE            (1U)
#define OHCI_HW_UNINITIALIZE          (0U)

// 1 means hardware initialization is complete, while 0 means it is not initialized.
static uint32_t _Ohci_HW_Initialize_status[USBH_OHCI_TT_INSTANCES];

static USBH_OHCI_Interrupt_t OHCI0_IRQ_Handler;
static USBH_OHCI_Interrupt_t OHCI1_IRQ_Handler;

/**
  \fn          int32_t USBH_OHCI_HW_Initialize (uint8_t ctrl, USBH_OHCI_Interrupt_t interrupt_handler)
  \brief       Initialize USB Host OHCI Interface.
  \param[in]   ctrl               Index of USB Host controller
  \param[in]   interrupt_handler  Pointer to Interrupt Handler Routine
  \return      0 on success, -1 on error.
*/
int32_t USBH_OHCI_HW_Initialize(uint8_t ctrl, USBH_OHCI_Interrupt_t interrupt_handler)
{
    // Add hardware-specific initialization code
    SYS_UnlockReg();

    if (ctrl == USBH0_OHCI_DRV_NUM)
    {
        /* Set the polarity of external USB VBUS power switch status signal.*/
#if (RTE_FS_VBUS_PORT0_ACTIVE == 1)
        HSOTG_SET_VBUS_STS_POL(HSOTG_VBUS_ST_VALID_HIGH);
#elif  (RTE_FS_VBUS_PORT0_ACTIVE == 0)
        HSOTG_SET_VBUS_STS_POL(HSOTG_VBUS_ST_VALID_LOW);
#endif

        /*set the polarity of external USB VBUS power switch over current signal.*/
#if (RTE_FS_OC_PORT0_ACTIVE == 1)
        HSOTG_SET_VBUS_OC_POL(HSOTG_VBUS_OC_VALID_HIGH);
#else
        HSOTG_SET_VBUS_OC_POL(HSOTG_VBUS_OC_VALID_LOW);
#endif

        OHCI0_IRQ_Handler = interrupt_handler;

        NVIC_EnableIRQ(USBH1_IRQn);

        _Ohci_HW_Initialize_status[ctrl] = OHCI_HW_INITIALIZE;
    }
    else
    {
        USBH_T     *_ohci;
        _ohci = USBH0;

        /* Set the polarity of external USB VBUS power switch status signal.*/
#if (RTE_FS_VBUS_PORT1_ACTIVE == 1)
        _ohci->HcMiscControl &= ~USBH_HcMiscControl_PPCAL_Msk;
#elif (RTE_HS_VBUS_ACTIVE == 0)
        _ohci->HcMiscControl |= USBH_HcMiscControl_PPCAL_Msk;
#endif

        /*set the polarity of external USB VBUS power switch over current signal.*/
#if (RTE_FS_OC_PORT1_ACTIVE == 1)
        _ohci->HcMiscControl &= ~USBH_HcMiscControl_OCAL_Msk;
#else
        _ohci->HcMiscControl |= USBH_HcMiscControl_OCAL_Msk;
#endif

        delay_ms(10U); //wait 10ms

        OHCI1_IRQ_Handler = interrupt_handler;

        NVIC_EnableIRQ(USBH0_IRQn);

        delay_ms(20U); //wait 1ms

        _Ohci_HW_Initialize_status[ctrl] = OHCI_HW_INITIALIZE;
    }

    SYS_LockReg();

    return 0;
}

/**
  \fn          int32_t USBH_OHCI_HW_Uninitialize (uint8_t ctrl)
  \brief       De-initialize USB Host OHCI Interface.
  \param[in]   ctrl               Index of USB Host controller
  \return      0 on success, -1 on error.
*/
int32_t USBH_OHCI_HW_Uninitialize(uint8_t ctrl)
{
    // Add hardware-specific uninitialization code
    if (ctrl > USBH_OHCI_TT_INSTANCES)
    {
        return (-1U);
    }

    _Ohci_HW_Initialize_status[ctrl] = OHCI_HW_UNINITIALIZE;

    return 0;
}

/**
  \fn          int32_t USBH_OHCI_HW_PowerControl (uint8_t ctrl, uint32_t state)
  \brief       Control USB Host OHCI Interface Power.
  \param[in]   ctrl               Index of USB Host controller
  \param[in]   state              Power state (0 = power off, 1 = power on)
  \return      0 on success, -1 on error.
*/
int32_t USBH_OHCI_HW_PowerControl(uint8_t ctrl, uint32_t state)
{
    if (state == 0U)                      // If power off requested
    {
        // Add hardware-specific code to power off the Host interface
    }
    else                                  // If power on requested
    {
        // Add hardware-specific code to power on the Host interface
        if (_Ohci_HW_Initialize_status[ctrl] == OHCI_HW_UNINITIALIZE)
        {
            return (-1U);
        }
    }

    return 0;
}

/**
  \fn          void USBH0_IRQ (void)
  \brief       USB0 Host Interrupt Routine (IRQ).
*/
NVT_ITCM void USBH1_IRQHandler(void)
{
    OHCI0_IRQ_Handler();                   // Call registered OHCI interrupt handler from hardware USB interrupt handler
}


#if (USBH1_OHCI_ENABLED == 1)
NVT_ITCM void USBH0_IRQHandler(void)
{
    OHCI1_IRQ_Handler();                   // Call registered OHCI interrupt handler from hardware USB interrupt handler
}
#endif
