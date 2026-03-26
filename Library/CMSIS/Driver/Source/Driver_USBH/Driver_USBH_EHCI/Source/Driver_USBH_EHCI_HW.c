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
 * Project:     USB Host EHCI Controller Driver Hardware-specific template
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


#include "NuMicro.h"
#include "Driver_USBH_EHCI_Config.h"
#include "Driver_USBH_EHCI_HW.h"

#if    (USBH1_EHCI_ENABLED == 1)
    #define USBH_EHCI_TT_INSTANCES         (2U)
    #error "M55M1 hasn't EHCI Port 1."
#else
    #define USBH_EHCI_TT_INSTANCES         (1U)
#endif

#define EHCI_HW_INITIALIZE            (1U)
#define EHCI_HW_UNINITIALIZE          (0U)

// 1 means hardware initialization is complete, while 0 means it is not initialized.
static uint32_t _Ehci_HW_Initialize_status[USBH_EHCI_TT_INSTANCES];
static USBH_EHCI_Interrupt_t EHCI_IRQ_Handler;

/**
  \fn          int32_t USBH_EHCI_HW_Initialize (uint8_t ctrl, USBH_EHCI_Interrupt_t interrupt_handler)
  \brief       Initialize USB Host EHCI Interface.
  \param[in]   ctrl               Index of USB Host controller
  \param[in]   interrupt_handler  Pointer to Interrupt Handler Routine
  \return      0 on success, -1 on error.
*/
int32_t USBH_EHCI_HW_Initialize(uint8_t ctrl, USBH_EHCI_Interrupt_t interrupt_handler)
{

    SYS_UnlockReg();

    /* Set the polarity of external USB VBUS power switch status signal.*/
#if (RTE_HS_VBUS_ACTIVE == 1)
    HSOTG_SET_VBUS_STS_POL(HSOTG_VBUS_ST_VALID_HIGH);
#else
    HSOTG_SET_VBUS_STS_POL(HSOTG_VBUS_ST_VALID_LOW);
#endif

    /*set the polarity of external USB VBUS power switch over current signal.*/
#if (RTE_HS_OC_ACTIVE == 1)
    HSOTG_SET_VBUS_OC_POL(HSOTG_VBUS_OC_VALID_HIGH);
#else
    HSOTG_SET_VBUS_OC_POL(HSOTG_VBUS_OC_VALID_LOW);
#endif

    // Add hardware-specific initialization code
    EHCI_IRQ_Handler = interrupt_handler; // Register EHCI interrupt handler

    // Enable IRQ
    NVIC_EnableIRQ(HSUSBH_IRQn);

    _Ehci_HW_Initialize_status[ctrl] = EHCI_HW_INITIALIZE;

    SYS_LockReg();

    return 0;
}

/**
  \fn          int32_t USBH_EHCI_HW_Uninitialize (uint8_t ctrl)
  \brief       De-initialize USB Host EHCI Interface.
  \param[in]   ctrl               Index of USB Host controller
  \return      0 on success, -1 on error.
*/
int32_t USBH_EHCI_HW_Uninitialize(uint8_t ctrl)
{

    _Ehci_HW_Initialize_status[ctrl] = EHCI_HW_UNINITIALIZE;

    return 0;
}

/**
  \fn          int32_t USBH_EHCI_HW_PowerControl (uint8_t ctrl, uint32_t state)
  \brief       Control USB Host EHCI Interface Power.
  \param[in]   ctrl               Index of USB Host controller
  \param[in]   state              Power state (0 = power off, 1 = power on)
  \return      0 on success, -1 on error.
*/
int32_t USBH_EHCI_HW_PowerControl(uint8_t ctrl, uint32_t state)
{


    if (state == 0U)                      // If power off requested
    {
        // Add hardware-specific code to power off the Host interface
    }
    else                                  // If power on requested
    {
        // Add hardware-specific code to power on the Host interface
        if (_Ehci_HW_Initialize_status[ctrl] == EHCI_HW_UNINITIALIZE)
        {
            return (-1);
        }
    }

    return 0;
}

/**
  \fn          void USBH0_IRQ (void)
  \brief       USB0 Host Interrupt Routine (IRQ).
*/
NVT_ITCM void HSUSBH_IRQHandler(void)
{
    EHCI_IRQ_Handler();                   // Call registered EHCI interrupt handler from hardware USB interrupt handler
}
