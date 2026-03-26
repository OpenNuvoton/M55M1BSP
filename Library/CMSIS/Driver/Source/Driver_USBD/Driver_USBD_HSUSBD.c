/*
 * Copyright (c) 2013-2020 Arm Limited. All rights reserved.
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

/**************************************************************************//**
* @file     Driver_USBD_HSUSBD.c
* @version  V1.00
* @brief    HSUSBD driver for Nuvoton M55M1
*
* @copyright SPDX-License-Identifier: Apache-2.0
* @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
******************************************************************************/

/*! \page Dirver_USBD HSUSBD

# Revision History

  - Initial release

# Requirements

This driver requires the M55M1 BSP.
The driver instance is mapped to hardware as shown in the table below:

  CMSIS Driver Instance | Hardware Resource
  :---------------------|:-----------------------
  Driver_USBD1          | HSUSBD0

*/

#ifdef _RTE_
    #include "RTE_Components.h"
#endif
/* Project can define PRJ_RTE_DEVICE_HEADER macro to include private or global RTE_Device.h. */
#ifdef   PRJ_RTE_DEVICE_HEADER
    #include PRJ_RTE_DEVICE_HEADER
#else
    #include "RTE_Device/RTE_Device.h"
#endif

#include "Driver_USBD.h"
#include <string.h>
#include "NuMicro.h"

// Configuration depending on RTE_Device_USBD.h
// Check if at least one peripheral instance is configured in RTE_Device_USBD.h
#if (RTE_USBD1 == 1)
// *****************************************************************************
// Compile-time configuration (that can be externally overridden if necessary)

// Maximum number of endpoints
#ifndef USBD_MAX_ENDPOINT_NUM
    #define USBD_MAX_ENDPOINT_NUM           (HSUSBD_MAX_EP)
#endif

// Maximum packet size for Endpoint 0
#ifndef USBD_EP0_MAX_PACKET_SIZE
    #define USBD_EP0_MAX_PACKET_SIZE        (64)
#endif

// Maximum RAM for endpoint buffers
#ifndef USBD_BUF_SIZE
    #define USBD_BUF_SIZE                   (8192)
#endif

// *****************************************************************************

// Macros
#define ARM_USBD_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */
// Macro for porting compatibility
#define USBD_t          HSUSBD_T
#define USBD_EP_t       HSUSBD_EP_T
// Macro for section for RW info
#ifdef  USBD_SECTION_NAME
    #define USBDn_SECTION_(name,n)  __attribute__((section(name #n)))
    #define USBDn_SECTION(n)        USBDn_SECTION_(USBD_SECTION_NAME,n)
#else
    #define USBDn_SECTION(n)
#endif

// Macro for declaring functions (for instances)
#define FUNCS_DECLARE(n)                                                                                                   \
    static  ARM_USBD_CAPABILITIES   USBD##n##_GetCapabilities           (void);                                            \
    static  int32_t                 USBD##n##_Initialize                (ARM_USBD_SignalDeviceEvent_t   cb_device_event,   \
                                                                         ARM_USBD_SignalEndpointEvent_t cb_endpoint_event);\
    static  int32_t                 USBD##n##_Uninitialize              (void);                                            \
    static  int32_t                 USBD##n##_PowerControl              (ARM_POWER_STATE state);                           \
    static  int32_t                 USBD##n##_DeviceConnect             (void);                                            \
    static  int32_t                 USBD##n##_DeviceDisconnect          (void);                                            \
    static  ARM_USBD_STATE          USBD##n##_DeviceGetState            (void);                                            \
    static  int32_t                 USBD##n##_DeviceRemoteWakeup        (void);                                            \
    static  int32_t                 USBD##n##_DeviceSetAddress          (uint8_t  dev_addr);                               \
    static  int32_t                 USBD##n##_ReadSetupPacket           (uint8_t *setup);                                  \
    static  int32_t                 USBD##n##_EndpointConfigure         (uint8_t  ep_addr,                                 \
                                                                         uint8_t  ep_type,                                 \
                                                                         uint16_t ep_max_packet_size);                     \
    static  int32_t                 USBD##n##_EndpointUnconfigure       (uint8_t  ep_addr);                                \
    static  int32_t                 USBD##n##_EndpointStall             (uint8_t  ep_addr, bool stall);                    \
    static  int32_t                 USBD##n##_EndpointTransfer          (uint8_t  ep_addr,                                 \
                                                                         uint8_t *data,                                    \
                                                                         uint32_t num);                                    \
    static  uint32_t                USBD##n##_EndpointTransferGetResult (uint8_t  ep_addr);                                \
    static  int32_t                 USBD##n##_EndpointTransferAbort     (uint8_t  ep_addr);                                \
    static  uint16_t                USBD##n##_GetFrameNumber            (void);

// Macro for defining functions (for instances)
#define FUNCS_DEFINE(n)                                                                                                                                                                                                        \
    static  ARM_USBD_CAPABILITIES   USBD##n##_GetCapabilities           (void)                                             { return USBDn_GetCapabilities           (&usbd##n##_info); }                                       \
    static  int32_t                 USBD##n##_Initialize                (ARM_USBD_SignalDeviceEvent_t   cb_device_event,                                                                                                       \
                                                                         ARM_USBD_SignalEndpointEvent_t cb_endpoint_event) { return USBDn_Initialize                (&usbd##n##_info, cb_device_event, cb_endpoint_event); }   \
    static  int32_t                 USBD##n##_Uninitialize              (void)                                             { return USBDn_Uninitialize              (&usbd##n##_info); }                                       \
    static  int32_t                 USBD##n##_PowerControl              (ARM_POWER_STATE state)                            { return USBDn_PowerControl              (&usbd##n##_info, state); }                                \
    static  int32_t                 USBD##n##_DeviceConnect             (void)                                             { return USBDn_DeviceConnect             (&usbd##n##_info); }                                       \
    static  int32_t                 USBD##n##_DeviceDisconnect          (void)                                             { return USBDn_DeviceDisconnect          (&usbd##n##_info); }                                       \
    static  ARM_USBD_STATE          USBD##n##_DeviceGetState            (void)                                             { return USBDn_DeviceGetState            (&usbd##n##_info); }                                       \
    static  int32_t                 USBD##n##_DeviceRemoteWakeup        (void)                                             { return USBDn_DeviceRemoteWakeup        (&usbd##n##_info); }                                       \
    static  int32_t                 USBD##n##_DeviceSetAddress          (uint8_t  dev_addr)                                { return USBDn_DeviceSetAddress          (&usbd##n##_info, dev_addr); }                             \
    static  int32_t                 USBD##n##_ReadSetupPacket           (uint8_t *setup)                                   { return USBDn_ReadSetupPacket           (&usbd##n##_info, setup); }                                \
    static  int32_t                 USBD##n##_EndpointConfigure         (uint8_t  ep_addr,                                                                                                                                     \
                                                                         uint8_t  ep_type,                                                                                                                                     \
                                                                         uint16_t ep_max_packet_size)                      { return USBDn_EndpointConfigure         (&usbd##n##_info, ep_addr, ep_type, ep_max_packet_size); } \
    static  int32_t                 USBD##n##_EndpointUnconfigure       (uint8_t  ep_addr)                                 { return USBDn_EndpointUnconfigure       (&usbd##n##_info, ep_addr); }                              \
    static  int32_t                 USBD##n##_EndpointStall             (uint8_t  ep_addr, bool stall)                     { return USBDn_EndpointStall             (&usbd##n##_info, ep_addr, stall); }                       \
    static  int32_t                 USBD##n##_EndpointTransfer          (uint8_t  ep_addr,                                                                                                                                     \
                                                                         uint8_t *data,                                                                                                                                        \
                                                                         uint32_t num)                                     { return USBDn_EndpointTransfer          (&usbd##n##_info, ep_addr, data, num); }                   \
    static  uint32_t                USBD##n##_EndpointTransferGetResult (uint8_t  ep_addr)                                 { return USBDn_EndpointTransferGetResult (&usbd##n##_info, ep_addr); }                              \
    static  int32_t                 USBD##n##_EndpointTransferAbort     (uint8_t  ep_addr)                                 { return USBDn_EndpointTransferAbort     (&usbd##n##_info, ep_addr); }                              \
    static  uint16_t                USBD##n##_GetFrameNumber            (void)                                             { return USBDn_GetFrameNumber            (&usbd##n##_info); }
// Macro for defining driver structures (for instances)
#define USBD_DRIVER(n)                  \
    ARM_DRIVER_USBD Driver_USBD##n = {    \
                                          USBD_GetVersion,                      \
                                          USBD##n##_GetCapabilities,            \
                                          USBD##n##_Initialize,                 \
                                          USBD##n##_Uninitialize,               \
                                          USBD##n##_PowerControl,               \
                                          USBD##n##_DeviceConnect,              \
                                          USBD##n##_DeviceDisconnect,           \
                                          USBD##n##_DeviceGetState,             \
                                          USBD##n##_DeviceRemoteWakeup,         \
                                          USBD##n##_DeviceSetAddress,           \
                                          USBD##n##_ReadSetupPacket,            \
                                          USBD##n##_EndpointConfigure,          \
                                          USBD##n##_EndpointUnconfigure,        \
                                          USBD##n##_EndpointStall,              \
                                          USBD##n##_EndpointTransfer,           \
                                          USBD##n##_EndpointTransferGetResult,  \
                                          USBD##n##_EndpointTransferAbort,      \
                                          USBD##n##_GetFrameNumber              \
                                     };

// Endpoint related macros
#define EP_OUT_INDEX            (0U)
#define EP_IN_INDEX             (1U)
#define EP_DIR(ep_addr)         (((ep_addr) >> 7) & 1U)
#define EP_NUM(ep_addr)         (ep_addr & ARM_USB_ENDPOINT_NUMBER_MASK)

typedef enum
{
    PERIPH_EPA = 0,
    PERIPH_EPB = 1,
    PERIPH_EPC = 2,
    PERIPH_EPD = 3,
    PERIPH_EPE = 4,
    PERIPH_EPF = 5,
    PERIPH_EPG = 6,
    PERIPH_EPH = 7,
    PERIPH_EPI = 8,
    PERIPH_EPJ = 9,
    PERIPH_EPK = 10,
    PERIPH_EPL = 11,
    PERIPH_EPM = 12,
    PERIPH_EPN = 13,
    PERIPH_EPO = 14,
    PERIPH_EPP = 15,
    PERIPH_EPQ = 16,
    PERIPH_EPR = 17,
    PERIPH_EPS = 18,
    PERIPH_EPT = 19,
    PERIPH_EPU = 20,
    PERIPH_EPV = 21,
    PERIPH_EPW = 22,
    PERIPH_EPX = 23,
    PERIPH_EPY = 24,
    PERIPH_EPZ = 25,
    PERIPH_MAX_EP = USBD_MAX_ENDPOINT_NUM,
} EP_Num_t;

static const uint8_t epcfg_eptype_table[] =
{
    [ARM_USB_ENDPOINT_ISOCHRONOUS] = 3 << HSUSBD_EPCFG_EPTYPE_Pos,
                                       [ARM_USB_ENDPOINT_BULK]        = 1 << HSUSBD_EPCFG_EPTYPE_Pos,
                                       [ARM_USB_ENDPOINT_INTERRUPT]   = 2 << HSUSBD_EPCFG_EPTYPE_Pos,
};

static const uint8_t eprspctl_eptype_table[] =
{
    [ARM_USB_ENDPOINT_ISOCHRONOUS] = 2 << HSUSBD_EPRSPCTL_MODE_Pos, /* Fly Mode */
                                       [ARM_USB_ENDPOINT_BULK]        = 0 << HSUSBD_EPRSPCTL_MODE_Pos, /* Auto-Validate Mode */
                                       [ARM_USB_ENDPOINT_INTERRUPT]   = 1 << HSUSBD_EPRSPCTL_MODE_Pos, /* Manual-Validate Mode */
};
// centralized location for USBD interrupt enable bit masks
static const uint32_t enabled_irqs = HSUSBD_GINTEN_USBIEN_Msk | HSUSBD_GINTEN_CEPIEN_Msk | \
                                     HSUSBD_GINTEN_EPAIEN_Msk | HSUSBD_GINTEN_EPBIEN_Msk | HSUSBD_GINTEN_EPCIEN_Msk | HSUSBD_GINTEN_EPDIEN_Msk | HSUSBD_GINTEN_EPEIEN_Msk | HSUSBD_GINTEN_EPFIEN_Msk | \
                                     HSUSBD_GINTEN_EPGIEN_Msk | HSUSBD_GINTEN_EPHIEN_Msk | HSUSBD_GINTEN_EPIIEN_Msk | HSUSBD_GINTEN_EPJIEN_Msk | HSUSBD_GINTEN_EPKIEN_Msk | HSUSBD_GINTEN_EPLIEN_Msk | \
                                     HSUSBD_GINTEN_EPMIEN_Msk | HSUSBD_GINTEN_EPNIEN_Msk | HSUSBD_GINTEN_EPOIEN_Msk | HSUSBD_GINTEN_EPPIEN_Msk | HSUSBD_GINTEN_EPQIEN_Msk | HSUSBD_GINTEN_EPRIEN_Msk;

// Driver status
typedef struct
{
    uint8_t                       initialized  : 1;       // Initialized status: 0 - not initialized, 1 - initialized
    uint8_t                       powered      : 1;       // Power status:       0 - not powered,     1 - powered
    uint8_t                       reserved     : 6;       // Reserved (for padding)
} DriverStatus_t;

// USB Device state
typedef struct
{
    volatile uint8_t              vbus;                   // USB Device VBUS state
    volatile uint8_t              speed;                  // USB Device speed (ARM_USB_SPEED_xxx) state
    volatile uint8_t              active;                 // USB Device active state
} USBD_State_t;

// Endpoint information
typedef struct
{
    uint8_t *volatile             data;                   // Pointer to data
    volatile uint32_t             num;                    // Number of bytes to transfer
    volatile uint16_t             configured;             // Endpoint configuration status
    uint16_t                      max_packet_size;        // Maximum packet size (in bytes)
    volatile uint32_t             num_transferred_total;  // Number of totally transferred bytes
    uint8_t                       ep_addr;                // Endpoint address
    uint8_t                       reserved[3];            // Reserved (for padding)
} EP_Info_t;

// Instance run-time information (RW)
typedef struct
{
    ARM_USBD_SignalDeviceEvent_t  cb_device_event;        // Device event callback
    ARM_USBD_SignalEndpointEvent_t cb_endpoint_event;     // Endpoint event callback
    DriverStatus_t                drv_status;             // Driver status
    USBD_State_t                  usbd_state;             // USB Device state
    uint32_t                      bufseg_addr;            // Allocated USB RAM
    volatile int32_t              cep_event;              // Control endpoint event
    volatile uint32_t             setup_received;         // Setup Packet received flag (0 - not received or read already, 1 - received and unread yet)
    volatile uint8_t              setup_packet[8];        // Setup Packet data
    EP_Info_t                     ep_info[PERIPH_MAX_EP];         // Endpoint information
    EP_Info_t                     cep_info[2];               // Control Endpoint information.(0 - out, 1 - in)
} RW_Info_t;

// Instance compile-time information (RO)
// also contains pointer to run-time information
typedef struct
{
    USBD_t                       *ptr_USBD;               // Pointer to USBD handle
    int32_t                       irq_n;
} RO_Info_t;

typedef struct
{
    const RO_Info_t              *ptr_ro_info;            // Pointer to compile-time information (RO)
    RW_Info_t                    *ptr_rw_info;            // Pointer to run-time information (RW)
} USBD_Info_t;


// Local functions prototypes
static const USBD_Info_t        *USBD_GetInfo(const USBD_t *husbd);
static ARM_DRIVER_VERSION       USBD_GetVersion(void);
static ARM_USBD_CAPABILITIES    USBDn_GetCapabilities(const USBD_Info_t *const ptr_usbd_info);
static int32_t                  USBDn_Initialize(const USBD_Info_t *const ptr_usbd_info, ARM_USBD_SignalDeviceEvent_t cb_device_event, ARM_USBD_SignalEndpointEvent_t cb_endpoint_event);
static int32_t                  USBDn_Uninitialize(const USBD_Info_t *const ptr_usbd_info);
static int32_t                  USBDn_PowerControl(const USBD_Info_t *const ptr_usbd_info, ARM_POWER_STATE state);
static int32_t                  USBDn_DeviceConnect(const USBD_Info_t *const ptr_usbd_info);
static int32_t                  USBDn_DeviceDisconnect(const USBD_Info_t *const ptr_usbd_info);
static ARM_USBD_STATE           USBDn_DeviceGetState(const USBD_Info_t *const ptr_usbd_info);
static int32_t                  USBDn_DeviceRemoteWakeup(const USBD_Info_t *const ptr_usbd_info);
static int32_t                  USBDn_DeviceSetAddress(const USBD_Info_t *const ptr_usbd_info, uint8_t  dev_addr);
static int32_t                  USBDn_ReadSetupPacket(const USBD_Info_t *const ptr_usbd_info, uint8_t *setup);
static int32_t                  USBDn_EndpointConfigure(const USBD_Info_t *const ptr_usbd_info, uint8_t  ep_addr, uint8_t  ep_type, uint16_t ep_max_packet_size);
static int32_t                  USBDn_EndpointUnconfigure(const USBD_Info_t *const ptr_usbd_info, uint8_t  ep_addr);
static int32_t                  USBDn_EndpointStall(const USBD_Info_t *const ptr_usbd_info, uint8_t  ep_addr, bool stall);
static int32_t                  USBDn_EndpointTransfer(const USBD_Info_t *const ptr_usbd_info, uint8_t  ep_addr, uint8_t *data, uint32_t num);
static uint32_t                 USBDn_EndpointTransferGetResult(const USBD_Info_t *const ptr_usbd_info, uint8_t  ep_addr);
static int32_t                  USBDn_EndpointTransferAbort(const USBD_Info_t *const ptr_usbd_info, uint8_t  ep_addr);
static uint16_t                 USBDn_GetFrameNumber(const USBD_Info_t *const ptr_usbd_info);

#if(RTE_USBD1)
static const RO_Info_t usbd1_ro_info = {HSUSBD, HSUSBD_IRQn};
static        RW_Info_t         usbd1_rw_info USBDn_SECTION(1);
static  const USBD_Info_t       usbd1_info = { &usbd1_ro_info,
                                               &usbd1_rw_info                                                                                                 \
                                             };
FUNCS_DECLARE(1)
FUNCS_DEFINE(1)
USBD_DRIVER(1)
#endif

// List of available USBD instance infos
static const USBD_Info_t *const usbd_info_list[] =
{
#if (RTE_USBD1)
    &usbd1_info,
#endif
    NULL
};

/* Driver Version */
static const ARM_DRIVER_VERSION usbd_driver_version =
{
    ARM_USBD_API_VERSION,
    ARM_USBD_DRV_VERSION
};

// Auxiliary functions

/**
  \fn          USBD_Info_t *USBD_GetInfo (const USBD_t *husbd)
  \brief       Get pointer to USBD_GetInfo structure corresponding to specified husbd.
  \param[in]   husbd    Pointer to USBD handle structure (USBD_t)
  \return      pointer to USBD info structure (USBD_Info_t)
*/
static const USBD_Info_t *USBD_GetInfo(const USBD_t *husbd)
{
    const USBD_Info_t *ptr_usbd_info;
    uint8_t i;

    ptr_usbd_info = NULL;

    // Find USBD which uses same husbd handle as parameter ptr_USBD
    for (i = 0U; i < (sizeof(usbd_info_list) / sizeof(USBD_t *)); i++)
    {
        if (usbd_info_list[i] != NULL)
        {
            if (usbd_info_list[i]->ptr_ro_info->ptr_USBD == husbd)
            {
                ptr_usbd_info = usbd_info_list[i];
                break;
            }
        }
    }

    return ptr_usbd_info;
}

/**
  \fn          USBD_EP_t *USBD_EndpointEntry(const USBD_Info_t *const ptr_usbd_info uint8_t ep_addr, bool add)
  \brief       Find or allocate a peripheral endpoint entry.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   add  Operation
                - \b false Find peripheral endpoint that matches ep_addr
                - \b true Open peripheral endpoint
  \return      Pointer to USBD EP handle structure (USBD_EP_t)
*/

static USBD_EP_t *USBD_EndpointEntry(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr, bool add)
{
    EP_Num_t ep_index;
    EP_Info_t *ptr_ep;
    USBD_t *husbd = ptr_usbd_info->ptr_ro_info->ptr_USBD;
    USBD_EP_t *ep;

    for (ep_index = PERIPH_EPA, ptr_ep = &ptr_usbd_info->ptr_rw_info->ep_info[PERIPH_EPA], ep = husbd->EP;
            ep_index < PERIPH_MAX_EP;
            ep_index++, ptr_ep++, ep++)
    {
        if (add)
        {
            if (0 == (ep->EPCFG & HSUSBD_EPCFG_EPEN_Msk)) return ep;
        }
        else
        {
            if (ptr_ep->ep_addr == ep_addr) return ep;
        }
    }

    return NULL;
}

/**
  \fn          USBD_WriteEpBuffer(uint32_t u32EpDat[], uint8_t u8Src[], uint32_t num)
  \brief       Write data into the USB endpoint buffer (IN transaction).
  \detail      This function writes data from a source buffer into the USB endpoint data(DAT) buffer.
  \param[out]  u32EpDat   Pointer to the endpoint DAT register (Endpoint Data Buffer).
  \param[in]   u8Src      Pointer to the source data buffer in system memory.
  \param[in]   num        Number of bytes to read from the endpoint buffer.
*/
void USBD_WriteEpBuffer(uint32_t u32EpDat[], uint8_t u8Src[], uint32_t num)
{
    uint32_t i = 0;

    for (; i + 4 <= num; i += 4, u8Src += 4)
        outpw(u32EpDat, *((uint32_t *)u8Src));

    for (; i < num; i++)
        outpb(u32EpDat, *u8Src++);
}

/**
  \fn          USBD_ReadEpBuffer(uint8_t u8Dst[], uint32_t u32EpDat[], uint32_t num)
  \brief       Read data from the USB endpoint buffer (OUT transaction).
  \detail      This function reads data from a USB endpoint data buffer(DAT) into a destination buffer.
  \param[out]  u8Dst      Pointer to the destination buffer in system memory.
  \param[in]   u32EpDat   Pointer to the endpoint DAT register (Endpoint Data Buffer).
  \param[in]   num        Number of bytes to read from the endpoint buffer.
*/

void USBD_ReadEpBuffer(uint8_t u8Dst[], uint32_t u32EpDat[], uint32_t num)
{
    uint32_t i = 0;

    for (; i + 4 <= num; i += 4, u8Dst += 4)
        * ((uint32_t *)u8Dst) = inpw(u32EpDat);

    for (; i < num; i++)
        *u8Dst++ = inpb(u32EpDat);
}

/**
  \fn          void USBD_EndpointConfigureBuffer (const USBD_Info_t *const ptr_usbd_info)
  \brief       Configure and reassign USB endpoint buffer addresses in USB SRAM.
  \detail      This function resets the buffer allocation pointer and reconfigures the buffer segmentation for all enabled endpoints.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
*/
static void USBD_EndpointConfigureBuffer(const USBD_Info_t *const ptr_usbd_info)
{
    // USB RAM beyond what we've allocated above is available to the user(Control endpoint used.)
    ptr_usbd_info->ptr_rw_info->bufseg_addr = USBD_EP0_MAX_PACKET_SIZE;

    // Reconfigures the buffer segmentation for all enabled endpoints.
    EP_Num_t ep_index;
    USBD_EP_t *ep;

    for (ep_index = PERIPH_EPA, ep = &ptr_usbd_info->ptr_ro_info->ptr_USBD->EP[PERIPH_EPA]; ep_index < PERIPH_MAX_EP; ep_index++, ep++)
    {
        if (0 == (ep->EPCFG & HSUSBD_EPCFG_EPEN_Msk)) continue;

        // Update the Endpoint Buffer Segmentation
        ep->EPBUFSTART = ptr_usbd_info->ptr_rw_info->bufseg_addr;
        ep->EPBUFEND = ptr_usbd_info->ptr_rw_info->bufseg_addr + ptr_usbd_info->ptr_rw_info->ep_info[ep_index].max_packet_size - 1U;
        ptr_usbd_info->ptr_rw_info->bufseg_addr += ptr_usbd_info->ptr_rw_info->ep_info[ep_index].max_packet_size;
        // Set endpoint payload
        ep->EPMPS = ptr_usbd_info->ptr_rw_info->ep_info[ep_index].max_packet_size;
    }
}

// Driver functions ************************************************************

/**
  \fn          ARM_DRIVER_VERSION USBD_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION USBD_GetVersion(void)
{
    return usbd_driver_version;
}

/**
  \fn          ARM_USBD_CAPABILITIES USBDn_GetCapabilities (const USBD_Info_t * const ptr_usbd_info)
  \brief       Get driver capabilities.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \return      \ref ARM_USBD_CAPABILITIES
*/
static ARM_USBD_CAPABILITIES USBDn_GetCapabilities(const USBD_Info_t *const ptr_usbd_info)
{
    ARM_USBD_CAPABILITIES driver_capabilities;

    (void)ptr_usbd_info;
    // Clear capabilities structure
    memset(&driver_capabilities, 0, sizeof(ARM_USBD_CAPABILITIES));

    // If VBUS detection is available
    driver_capabilities.vbus_detection = 1U;
    driver_capabilities.event_vbus_on  = 1U;
    driver_capabilities.event_vbus_off = 1U;

    return driver_capabilities;
}

/**
  \fn          int32_t USBDn_Initialize (const USBD_Info_t * const      ptr_usbd_info,
                                         ARM_USBD_SignalDeviceEvent_t   cb_device_event,
                                         ARM_USBD_SignalEndpointEvent_t cb_endpoint_event)
  \brief       Initialize USB Device Interface.
  \param[in]   ptr_usbd_info      Pointer to USBD info structure (USBD_Info_t)
  \param[in]   cb_device_event    Pointer to \ref ARM_USBD_SignalDeviceEvent
  \param[in]   cb_endpoint_event  Pointer to \ref ARM_USBD_SignalEndpointEvent
  \return      \ref execution_status
*/
static int32_t USBDn_Initialize(const USBD_Info_t *const ptr_usbd_info, ARM_USBD_SignalDeviceEvent_t cb_device_event,
                                ARM_USBD_SignalEndpointEvent_t cb_endpoint_event)
{
    // Clear run-time info
    memset((void *)ptr_usbd_info->ptr_rw_info, 0, sizeof(RW_Info_t));

    // Register callback functions
    ptr_usbd_info->ptr_rw_info->cb_device_event   = cb_device_event;
    ptr_usbd_info->ptr_rw_info->cb_endpoint_event = cb_endpoint_event;
    // Set driver status to initialized
    ptr_usbd_info->ptr_rw_info->drv_status.initialized = 1U;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBDn_Uninitialize (const USBD_Info_t * const ptr_usbd_info)
  \brief       De-initialize USB Device Interface.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \return      \ref execution_status
*/
static int32_t USBDn_Uninitialize(const USBD_Info_t *const ptr_usbd_info)
{
    if (ptr_usbd_info->ptr_rw_info->drv_status.powered != 0U)
    {
        // If peripheral is powered, power off the peripheral
        (void)USBDn_PowerControl(ptr_usbd_info, ARM_POWER_OFF);
    }

    // Clear run-time info
    memset((void *)ptr_usbd_info->ptr_rw_info, 0, sizeof(RW_Info_t));

    return ARM_DRIVER_OK;
}

static int32_t USBDn_PowerControl(const USBD_Info_t *const ptr_usbd_info, ARM_POWER_STATE state)
{
    ARM_USBD_SignalDeviceEvent_t   cb_device_event;
    ARM_USBD_SignalEndpointEvent_t cb_endpoint_event;
    DriverStatus_t                 drv_status;
    USBD_t *husbd = ptr_usbd_info->ptr_ro_info->ptr_USBD;

    switch (state)
    {
        case ARM_POWER_FULL:
            if (ptr_usbd_info->ptr_rw_info->drv_status.initialized == 0U)
            {
                return ARM_DRIVER_ERROR;
            }

            // Store variables we need to preserve
            cb_device_event   = ptr_usbd_info->ptr_rw_info->cb_device_event;
            cb_endpoint_event = ptr_usbd_info->ptr_rw_info->cb_endpoint_event;
            drv_status        = ptr_usbd_info->ptr_rw_info->drv_status;

            // Clear run-time info
            memset((void *)ptr_usbd_info->ptr_rw_info, 0, sizeof(RW_Info_t));

            // Restore variables we wanted to preserve
            ptr_usbd_info->ptr_rw_info->cb_device_event   = cb_device_event;
            ptr_usbd_info->ptr_rw_info->cb_endpoint_event = cb_endpoint_event;
            ptr_usbd_info->ptr_rw_info->drv_status        = drv_status;

            // Set driver status to powered
            ptr_usbd_info->ptr_rw_info->drv_status.powered = 1U;

            uint32_t u32TimeOutCnt = SystemCoreClock;
            // Initial USB engine
            husbd->PHYCTL |= HSUSBD_PHYCTL_PHYEN_Msk;

            // wait PHY clock ready
            while (!(husbd->PHYCTL & HSUSBD_PHYCTL_PHYCLKSTB_Msk))
            {
                if (--u32TimeOutCnt == 0) return ARM_DRIVER_ERROR_TIMEOUT;
            }

            NVIC_EnableIRQ(ptr_usbd_info->ptr_ro_info->irq_n);

            // Enable USB BUS, CEP and EPA-R global interrupt
            husbd->GINTEN = enabled_irqs;
            // Enable BUS interrupt
            husbd->BUSINTEN = HSUSBD_BUSINTEN_DMADONEIEN_Msk | HSUSBD_BUSINTEN_RESUMEIEN_Msk | HSUSBD_BUSINTEN_RSTIEN_Msk | HSUSBD_BUSINTEN_VBUSDETIEN_Msk;
            // USB High-speed initiate a chirp-sequence
            husbd->OPER = HSUSBD_OPER_HISPDEN_Msk;

            break;

        case ARM_POWER_OFF:

            for (EP_Num_t ep_index = PERIPH_EPA; ep_index < PERIPH_MAX_EP; ep_index++)
            {
                husbd->EP[ep_index].EPCFG = 0;
            }

            NVIC_DisableIRQ(ptr_usbd_info->ptr_ro_info->irq_n);

            // Set driver status to not powered
            ptr_usbd_info->ptr_rw_info->drv_status.powered = 0U;

            // Store variables we need to preserve
            cb_device_event   = ptr_usbd_info->ptr_rw_info->cb_device_event;
            cb_endpoint_event = ptr_usbd_info->ptr_rw_info->cb_endpoint_event;
            drv_status        = ptr_usbd_info->ptr_rw_info->drv_status;

            // Clear run-time info
            memset((void *)ptr_usbd_info->ptr_rw_info, 0, sizeof(RW_Info_t));

            // Restore variables we wanted to preserve
            ptr_usbd_info->ptr_rw_info->cb_device_event   = cb_device_event;
            ptr_usbd_info->ptr_rw_info->cb_endpoint_event = cb_endpoint_event;
            ptr_usbd_info->ptr_rw_info->drv_status        = drv_status;
            break;

        case ARM_POWER_LOW:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        default:
            return ARM_DRIVER_ERROR_PARAMETER;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBDn_DeviceConnect (const USBD_Info_t * const ptr_usbd_info)
  \brief       Connect USB Device.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \return      \ref execution_status
*/
static int32_t USBDn_DeviceConnect(const USBD_Info_t *const ptr_usbd_info)
{
    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    // Pull D+ high to let the host detect a Full-Speed device and complete the connection.
    ptr_usbd_info->ptr_ro_info->ptr_USBD->PHYCTL |= HSUSBD_PHYCTL_DPPUEN_Msk;

    if (ptr_usbd_info->ptr_ro_info->ptr_USBD->OPER & HSUSBD_OPER_HISPDEN_Msk)
        ptr_usbd_info->ptr_ro_info->ptr_USBD->OPER |= HSUSBD_OPER_HISHSEN_Msk;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBDn_DeviceDisconnect (const USBD_Info_t * const ptr_usbd_info)
  \brief       Disconnect USB Device.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \return      \ref execution_status
*/
static int32_t USBDn_DeviceDisconnect(const USBD_Info_t *const ptr_usbd_info)
{
    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    // To simulate a disconnection, the pull-up is disabled to create a Single-Ended Zero (SE0) condition.
    ptr_usbd_info->ptr_ro_info->ptr_USBD->PHYCTL &= ~HSUSBD_PHYCTL_DPPUEN_Msk;
    return ARM_DRIVER_OK;
}

/**
  \fn          ARM_USBD_STATE USBDn_DeviceGetState (const USBD_Info_t * const ptr_usbd_info)
  \brief       Get current USB Device State.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \return      Device State \ref ARM_USBD_STATE
*/
static ARM_USBD_STATE USBDn_DeviceGetState(const USBD_Info_t *const ptr_usbd_info)
{
    ARM_USBD_STATE state;
    // Clear state structure
    memset(&state, 0, sizeof(ARM_USBD_STATE));

    // Process additionally handled communication information
    if (ptr_usbd_info->ptr_rw_info->usbd_state.vbus != 0U)
    {
        state.vbus = 1U;
    }

    switch (ptr_usbd_info->ptr_rw_info->usbd_state.speed)
    {
        case ARM_USB_SPEED_FULL:
            state.speed = ARM_USB_SPEED_FULL;
            break;

        case ARM_USB_SPEED_HIGH:
            state.speed = ARM_USB_SPEED_HIGH;
            break;

        default:
            break;
    }

    if (ptr_usbd_info->ptr_rw_info->usbd_state.active != 0U)
    {
        state.active = 1U;
    }

    return state;
}

/**
  \fn          int32_t USBDn_DeviceRemoteWakeup (const USBD_Info_t * const ptr_usbd_info)
  \brief       Trigger USB Remote Wakeup.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \return      \ref execution_status
*/
static int32_t USBDn_DeviceRemoteWakeup(const USBD_Info_t *const ptr_usbd_info)
{
    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    ptr_usbd_info->ptr_ro_info->ptr_USBD->OPER |= HSUSBD_OPER_RESUMEEN_Msk;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBDn_DeviceSetAddress (const USBD_Info_t * const ptr_usbd_info, uint8_t dev_addr)
  \brief       Set USB Device Address.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \param[in]   dev_addr  Device Address
  \return      \ref execution_status
*/
static int32_t USBDn_DeviceSetAddress(const USBD_Info_t *const ptr_usbd_info, uint8_t dev_addr)
{
    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    ptr_usbd_info->ptr_ro_info->ptr_USBD->FADDR = dev_addr;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBDn_ReadSetupPacket (const USBD_Info_t * const ptr_usbd_info, uint8_t *setup)
  \brief       Read setup packet received over Control Endpoint.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \param[out]  setup  Pointer to buffer for setup packet
  \return      \ref execution_status
*/
static int32_t USBDn_ReadSetupPacket(const USBD_Info_t *const ptr_usbd_info, uint8_t *setup)
{
    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    if (ptr_usbd_info->ptr_rw_info->setup_received == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    do
    {
        ptr_usbd_info->ptr_rw_info->setup_received = 0U;
        memcpy((uint8_t *)setup, (uint8_t *)ptr_usbd_info->ptr_rw_info->setup_packet, 8U);
    } while (ptr_usbd_info->ptr_rw_info->setup_received != 0U);

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBDn_EndpointConfigure (const USBD_Info_t * const ptr_usbd_info,
                                                      uint8_t           ep_addr,
                                                      uint8_t           ep_type,
                                                      uint16_t          ep_max_packet_size)
  \brief       Configure USB Endpoint.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   ep_type  Endpoint Type (ARM_USB_ENDPOINT_xxx)
  \param[in]   ep_max_packet_size Endpoint Maximum Packet Size
  \return      \ref execution_status
*/
static int32_t USBDn_EndpointConfigure(const USBD_Info_t *const ptr_usbd_info, uint8_t  ep_addr,
                                       uint8_t  ep_type,
                                       uint16_t ep_max_packet_size)
{
    EP_Info_t *ptr_ep;
    uint32_t   ep_dir_mask, ep_type_mask;
    uint8_t    ep_num = EP_NUM(ep_addr);
    uint8_t    ep_dir = EP_DIR(ep_addr);

    // Unconfigure Endpoint
    USBDn_EndpointUnconfigure(ptr_usbd_info, ep_addr);

    if (ep_num != 0)
    {
        USBD_EP_t *ep = USBD_EndpointEntry(ptr_usbd_info, ep_addr, true);

        // Error if all periph endpoints used
        if (ep == NULL)
        {
            return ARM_DRIVER_ERROR_PARAMETER;
        }

        ptr_ep = &ptr_usbd_info->ptr_rw_info->ep_info[ep - ptr_usbd_info->ptr_ro_info->ptr_USBD->EP];

        // Error if USB buffer is insufficient
        if (ptr_usbd_info->ptr_rw_info->bufseg_addr + ep_max_packet_size > USBD_BUF_SIZE)
        {
            return ARM_DRIVER_ERROR;
        }

        ep->EPRSPCTL = (HSUSBD_EP_RSPCTL_FLUSH | eprspctl_eptype_table[ep_type]);

        // Configured endpoint
        ep_dir_mask = ep_dir ? HSUSBD_EP_CFG_DIR_IN : HSUSBD_EP_CFG_DIR_OUT;
        ep_type_mask = epcfg_eptype_table[ep_type];

        ep->EPCFG = ((ep_num << HSUSBD_EPCFG_EPNUM_Pos) | ep_dir_mask | ep_type_mask | HSUSBD_EPCFG_EPEN_Msk);
    }
    else
    {
        ptr_ep = &ptr_usbd_info->ptr_rw_info->cep_info[ep_dir];
    }

    // Store ep address information
    ptr_ep->ep_addr = ep_addr;

    // Store max packet size information
    ptr_ep->max_packet_size = ep_max_packet_size;

    // Reconfigures the buffer segmentation for all enabled endpoints(must configure EP before configure buffer segmentation)
    USBD_EndpointConfigureBuffer(ptr_usbd_info);

    // Update endpoint configured status
    ptr_ep->configured = 1U;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBDn_EndpointUnconfigure (const USBD_Info_t * const ptr_usbd_info, uint8_t ep_addr)
  \brief       Unconfigure USB Endpoint.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      \ref execution_status
*/
static int32_t USBDn_EndpointUnconfigure(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr)
{
    EP_Info_t *ptr_ep;

    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    if (EP_NUM(ep_addr) != 0)
    {
        USBD_EP_t *ep = USBD_EndpointEntry(ptr_usbd_info, ep_addr, false);

        if (ep != NULL)
        {
            ptr_ep = &ptr_usbd_info->ptr_rw_info->ep_info[ep - ptr_usbd_info->ptr_ro_info->ptr_USBD->EP];
            // Clear Endpoint information
            memset((void *)ptr_ep, 0, sizeof(EP_Info_t));
            ptr_ep->data = NULL;
            ptr_ep->num = 0U;

            // Clear Endpoint configure
            ep->EPCFG = 0;
            ep->EPRSPCTL = HSUSBD_EP_RSPCTL_TOGGLE;
        }
    }
    else
    {
        ptr_ep = &ptr_usbd_info->ptr_rw_info->cep_info[EP_DIR(ep_addr)];
        // Clear Endpoint information
        memset((void *)ptr_ep, 0, sizeof(EP_Info_t));
        ptr_ep->data = NULL;
        ptr_ep->num = 0U;
    }

    // Deallocate USB RAM and update bufseg_addr
    USBD_EndpointConfigureBuffer(ptr_usbd_info);

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBDn_EndpointStall (const USBD_Info_t * const ptr_usbd_info, uint8_t ep_addr, bool stall)
  \brief       Set/Clear Stall for USB Endpoint.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[in]   stall  Operation
                - \b false Clear
                - \b true Set
  \return      \ref execution_status
*/
static int32_t USBDn_EndpointStall(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr, bool stall)
{

    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    if (stall)
    {
        // Set STALL
        if (EP_NUM(ep_addr) != 0)
        {
            USBD_EP_t *ep = USBD_EndpointEntry(ptr_usbd_info, ep_addr, false);
            ep->EPRSPCTL = (ep->EPRSPCTL & 0xf7) | HSUSBD_EPRSPCTL_HALT_Msk;
        }
        else
        {
            ptr_usbd_info->ptr_ro_info->ptr_USBD->CEPCTL = HSUSBD_CEPCTL_STALLEN_Msk;
        }
    }
    else
    {
        if (EP_NUM(ep_addr) != 0)
        {
            // Clear STALL
            USBD_EP_t *ep = USBD_EndpointEntry(ptr_usbd_info, ep_addr, false);
            ep->EPRSPCTL = HSUSBD_EPRSPCTL_TOGGLE_Msk;
        }
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USBDn_EndpointTransfer (const USBD_Info_t * const ptr_usbd_info, uint8_t ep_addr, uint8_t *data, uint32_t num)
  \brief       Read data from or Write data to USB Endpoint.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \param[out]  data Pointer to buffer for data to read or with data to write
  \param[in]   num  Number of data bytes to transfer
  \return      \ref execution_status
*/
static int32_t USBDn_EndpointTransfer(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr, uint8_t *data, uint32_t num)
{
    EP_Info_t *ptr_ep;
    USBD_t *husbd = ptr_usbd_info->ptr_ro_info->ptr_USBD;
    USBD_EP_t *ep = NULL;
    uint8_t    ep_num = EP_NUM(ep_addr);

    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    if (ep_num != 0)
    {
        ep = USBD_EndpointEntry(ptr_usbd_info, ep_addr, false);
        uint8_t  periph_epnum = ep - husbd->EP;
        ptr_ep = &ptr_usbd_info->ptr_rw_info->ep_info[periph_epnum];
    }
    else
    {
        ptr_ep = &ptr_usbd_info->ptr_rw_info->cep_info[EP_DIR(ep_addr)];
    }

    // Clear number of transferred bytes
    ptr_ep->num_transferred_total = 0U;

    // Register pointer to data and number of bytes to transfer for Endpoint
    ptr_ep->data = data;
    ptr_ep->num = num;

    // Prepare to transfer
    if (EP_DIR(ep_addr))//IN
    {
        if (EP_NUM(ep_addr) != 0)
        {
            ep->EPINTEN = HSUSBD_EPINTEN_BUFEMPTYIEN_Msk;
        }
        else
        {
            if (ptr_ep->num)
            {
                husbd->CEPCTL = HSUSBD_CEPCTL_FLUSH_Msk;
                husbd->CEPINTSTS = HSUSBD_CEPINTSTS_INTKIF_Msk;
                husbd->CEPINTEN = HSUSBD_CEPINTEN_INTKIEN_Msk;
            }
            else//Zero Length Packet
            {
                ptr_usbd_info->ptr_rw_info->cep_event |= ARM_USBD_EVENT_IN;
                husbd->CEPINTSTS = HSUSBD_CEPINTSTS_STSDONEIF_Msk;
                husbd->CEPCTL = HSUSBD_CEPCTL_NAKCLR;
                husbd->CEPINTEN = HSUSBD_CEPINTEN_STSDONEIEN_Msk;
            }
        }
    }
    else//OUT
    {
        if (EP_NUM(ep_addr) != 0)
        {
            ep->EPINTEN = HSUSBD_EPINTEN_RXPKIEN_Msk;
        }
        else
        {
            if (ptr_ep->num)
            {
                uint32_t u32TimeOutCnt = SystemCoreClock >> 2;

                while (ptr_ep->num < (husbd->CEPRXCNT & HSUSBD_CEPRXCNT_RXCNT_Msk))
                    if (--u32TimeOutCnt == 0) break;

                USBD_ReadEpBuffer(ptr_ep->data, (uint32_t *)&husbd->CEPDAT, ptr_ep->num);
                ptr_ep->num_transferred_total += ptr_ep->num;
            }

            if ((ptr_ep->num_transferred_total < ptr_ep->max_packet_size) || (ptr_ep->num_transferred_total == ptr_ep->num))
            {
                ptr_usbd_info->ptr_rw_info->cb_endpoint_event(0x00U, ARM_USBD_EVENT_OUT);
            }
        }
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t USBDn_EndpointTransferGetResult (const USBD_Info_t * const ptr_usbd_info, uint8_t ep_addr)
  \brief       Get result of USB Endpoint transfer.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
  \return      number of successfully transferred data bytes
*/
static uint32_t USBDn_EndpointTransferGetResult(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr)
{
    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return 0U;
    }

    EP_Info_t *ptr_ep;

    if (EP_NUM(ep_addr) != 0)
    {
        USBD_EP_t *ep = USBD_EndpointEntry(ptr_usbd_info, ep_addr, false);
        uint8_t   periph_epnum = ep - ptr_usbd_info->ptr_ro_info->ptr_USBD->EP;
        ptr_ep = &ptr_usbd_info->ptr_rw_info->ep_info[periph_epnum];

        if (periph_epnum >= PERIPH_MAX_EP)
        {
            return 0U;
        }
    }
    else
    {
        ptr_ep = &ptr_usbd_info->ptr_rw_info->cep_info[EP_DIR(ep_addr)];
    }

    return ptr_ep->num_transferred_total;
}

static int32_t USBDn_EndpointTransferAbort(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr)
{
    // Reset DMA
    ptr_usbd_info->ptr_ro_info->ptr_USBD->DMACNT = 0U;
    ptr_usbd_info->ptr_ro_info->ptr_USBD->DMACTL = HSUSBD_DMACTL_DMARST_Msk;
    ptr_usbd_info->ptr_ro_info->ptr_USBD->DMACTL = 0U;

    if (EP_NUM(ep_addr) != 0U)
    {
        USBD_EP_t *ep = USBD_EndpointEntry(ptr_usbd_info, ep_addr, false);
        ep->EPRSPCTL |= HSUSBD_EPRSPCTL_FLUSH_Msk;
    }
    else
    {
        ptr_usbd_info->ptr_ro_info->ptr_USBD->CEPCTL |= HSUSBD_CEPCTL_FLUSH_Msk;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          uint16_t USBDn_GetFrameNumber (const USBD_Info_t * const ptr_usbd_info)
  \brief       Get current USB Frame Number.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \return      Frame Number
*/
static uint16_t USBDn_GetFrameNumber(const USBD_Info_t *const ptr_usbd_info)
{
    return ((ptr_usbd_info->ptr_ro_info->ptr_USBD->FRAMECNT & HSUSBD_FRAMECNT_FRAMECNT_Msk) >> HSUSBD_FRAMECNT_FRAMECNT_Pos);
}

// Event functions *****************************************************************
/**
  \fn          void USBD_DataOutStage(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr)
  \brief       Data OUT stage.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
*/
static void USBD_DataOutStage(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr)
{
    EP_Info_t *ptr_ep;
    uint32_t   num_transferred;
    USBD_EP_t *ep;

    ep = USBD_EndpointEntry(ptr_usbd_info, ep_addr, false);
    ptr_ep = &ptr_usbd_info->ptr_rw_info->ep_info[ep - ptr_usbd_info->ptr_ro_info->ptr_USBD->EP];

    num_transferred = ep->EPDATCNT & HSUSBD_EPDATCNT_DATCNT_Msk;
    ptr_ep->num_transferred_total += num_transferred;

    USBD_ReadEpBuffer((uint8_t *)ptr_ep->data, (uint32_t *)&ep->EPDAT, num_transferred);

    if ((ptr_usbd_info->ptr_rw_info->cb_endpoint_event != NULL))
    {
        ptr_usbd_info->ptr_rw_info->cb_endpoint_event(ep_addr, ARM_USBD_EVENT_OUT);
    }
}

/**
  \fn          void USBD_DataInStage(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr)
  \brief       Data IN stage.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
*/
static void USBD_DataInStage(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr)
{
    EP_Info_t *ptr_ep;
    uint32_t   num_to_transfer;
    uint8_t   *data_to_transfer;
    USBD_EP_t *ep;

    ep = USBD_EndpointEntry(ptr_usbd_info, ep_addr, false);
    ptr_ep = &ptr_usbd_info->ptr_rw_info->ep_info[ep - ptr_usbd_info->ptr_ro_info->ptr_USBD->EP];


    // If there is more data to transfer
    data_to_transfer = ptr_ep->data + ptr_ep->num_transferred_total;
    num_to_transfer  = ptr_ep->num - ptr_ep->num_transferred_total;

    if (num_to_transfer > ptr_ep->max_packet_size)
    {
        num_to_transfer = ptr_ep->max_packet_size;
    }

    // Update transferred number
    ptr_ep->num_transferred_total += num_to_transfer;

    if (ptr_ep->num_transferred_total == ptr_ep->num)
    {
        ep->EPINTSTS = HSUSBD_EPINTSTS_TXPKIF_Msk;
        ep->EPINTEN = HSUSBD_EPINTEN_TXPKIEN_Msk;
    }

    USBD_WriteEpBuffer((uint32_t *)&ep->EPDAT, (uint8_t *)data_to_transfer, num_to_transfer);

    if (num_to_transfer != ptr_ep->max_packet_size) ep->EPRSPCTL = HSUSBD_EPRSPCTL_SHORTTXEN_Msk;

}

/**
  \fn          USBD_SetupStage(const USBD_Info_t *const ptr_usbd_info)
  \brief       Setup stage.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  */
static void USBD_SetupStage(const USBD_Info_t *const ptr_usbd_info)
{
    ptr_usbd_info->ptr_rw_info->setup_packet[0] = (uint8_t)((ptr_usbd_info->ptr_ro_info->ptr_USBD->SETUP1_0 >> 0) & 0xFF);
    ptr_usbd_info->ptr_rw_info->setup_packet[1] = (uint8_t)((ptr_usbd_info->ptr_ro_info->ptr_USBD->SETUP1_0 >> 8) & 0xFF);
    ptr_usbd_info->ptr_rw_info->setup_packet[2] = (uint8_t)((ptr_usbd_info->ptr_ro_info->ptr_USBD->SETUP3_2 >> 0) & 0xFF);
    ptr_usbd_info->ptr_rw_info->setup_packet[3] = (uint8_t)((ptr_usbd_info->ptr_ro_info->ptr_USBD->SETUP3_2 >> 8) & 0xFF);
    ptr_usbd_info->ptr_rw_info->setup_packet[4] = (uint8_t)((ptr_usbd_info->ptr_ro_info->ptr_USBD->SETUP5_4 >> 0) & 0xFF);
    ptr_usbd_info->ptr_rw_info->setup_packet[5] = (uint8_t)((ptr_usbd_info->ptr_ro_info->ptr_USBD->SETUP5_4 >> 8) & 0xFF);
    ptr_usbd_info->ptr_rw_info->setup_packet[6] = (uint8_t)((ptr_usbd_info->ptr_ro_info->ptr_USBD->SETUP7_6 >> 0) & 0xFF);
    ptr_usbd_info->ptr_rw_info->setup_packet[7] = (uint8_t)((ptr_usbd_info->ptr_ro_info->ptr_USBD->SETUP7_6 >> 8) & 0xFF);

    ptr_usbd_info->ptr_rw_info->setup_received = 1U;

    if (ptr_usbd_info->ptr_rw_info->cb_endpoint_event != NULL)
    {
        ptr_usbd_info->ptr_rw_info->cb_endpoint_event(0x00U, ARM_USBD_EVENT_SETUP);
    }
}

/**
  \fn          void USBD_BusReset (const USBD_Info_t *const ptr_usbd_info)
  \brief       USBD Bus Reset.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  */
static void USBD_BusReset(const USBD_Info_t *const ptr_usbd_info)
{
    USBD_t *husbd = ptr_usbd_info->ptr_ro_info->ptr_USBD;
    // Clear Endpoints information
    memset((void *)ptr_usbd_info->ptr_rw_info->ep_info, 0U, PERIPH_MAX_EP * sizeof(EP_Info_t));
    memset((void *)ptr_usbd_info->ptr_rw_info->cep_info, 0U, 2 * sizeof(EP_Info_t));

    // Reset USBD state information
    ptr_usbd_info->ptr_rw_info->usbd_state.speed  = ARM_USB_SPEED_FULL;
    ptr_usbd_info->ptr_rw_info->usbd_state.active = 0U;

    if (ptr_usbd_info->ptr_rw_info->cb_device_event != NULL)
    {
        ptr_usbd_info->ptr_rw_info->cb_device_event(ARM_USBD_EVENT_RESET);
    }

    // Reset DMA
    ptr_usbd_info->ptr_ro_info->ptr_USBD->DMACNT = 0U;
    ptr_usbd_info->ptr_ro_info->ptr_USBD->DMACTL = HSUSBD_DMACTL_DMARST_Msk;
    ptr_usbd_info->ptr_ro_info->ptr_USBD->DMACTL = 0U;

    for (EP_Num_t ep_index = PERIPH_EPA; ep_index < PERIPH_MAX_EP; ep_index++)
    {
        ptr_usbd_info->ptr_ro_info->ptr_USBD->EP[ep_index].EPCFG = 0U;
    }

    // Buffer for control endpoint.
    husbd->CEPBUFSTART = 0U;
    husbd->CEPBUFEND = USBD_EP0_MAX_PACKET_SIZE - 1U;

    // Configure Endpoint 0 OUT
    (void)USBDn_EndpointConfigure(ptr_usbd_info, 0x00U, ARM_USB_ENDPOINT_CONTROL, USBD_EP0_MAX_PACKET_SIZE);

    // Configure Endpoint 0 IN
    (void)USBDn_EndpointConfigure(ptr_usbd_info, 0x80U, ARM_USB_ENDPOINT_CONTROL, USBD_EP0_MAX_PACKET_SIZE);

    ptr_usbd_info->ptr_ro_info->ptr_USBD->FADDR = 0U;
    // After reset we consider USB as active
    ptr_usbd_info->ptr_rw_info->usbd_state.active = 1U;
}

// IRQ handler *********************************************************************

NVT_ITCM void HSUSBDn_IRQHandler(const USBD_Info_t *const ptr_usbd_info)
{
    volatile uint32_t u32IntSts, u32BusSts, u32EpIntSts;
    uint32_t u32TimeOutCnt;
    USBD_t *husbd = ptr_usbd_info->ptr_ro_info->ptr_USBD;
    EP_Info_t *ptr_ep;
    uint32_t event = 0;

    u32IntSts = husbd->GINTSTS & husbd->GINTEN;

    if (!u32IntSts)    return;

    //------------------------------------------------------------------
    if (u32IntSts & HSUSBD_GINTSTS_USBIF_Msk)
    {
        // Bus event
        u32BusSts = husbd->BUSINTSTS & husbd->BUSINTEN;

        if (u32BusSts & HSUSBD_BUSINTSTS_SOFIF_Msk)
            husbd->BUSINTSTS = HSUSBD_BUSINTSTS_SOFIF_Msk;

        if (u32BusSts & HSUSBD_BUSINTSTS_RSTIF_Msk)
        {
            USBD_BusReset(ptr_usbd_info);

            ptr_usbd_info->ptr_rw_info->usbd_state.speed = (husbd->OPER & HSUSBD_OPER_CURSPD_Msk) ? ARM_USB_SPEED_HIGH : ARM_USB_SPEED_FULL;

            if (ptr_usbd_info->ptr_rw_info->usbd_state.speed == ARM_USB_SPEED_HIGH)
                event |= ARM_USBD_EVENT_HIGH_SPEED;

            husbd->CEPINTEN = HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk;
            husbd->BUSINTEN = (HSUSBD_BUSINTEN_RSTIEN_Msk | HSUSBD_BUSINTEN_RESUMEIEN_Msk | HSUSBD_BUSINTEN_SUSPENDIEN_Msk);
            husbd->BUSINTSTS = HSUSBD_BUSINTSTS_RSTIF_Msk;
            husbd->CEPINTSTS = 0x1ffc;
        }

        if (u32BusSts & HSUSBD_BUSINTSTS_RESUMEIF_Msk)
        {
            husbd->PHYCTL |= (HSUSBD_PHYCTL_PHYEN_Msk | HSUSBD_PHYCTL_DPPUEN_Msk);
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

            while (!(ptr_usbd_info->ptr_ro_info->ptr_USBD->PHYCTL & HSUSBD_PHYCTL_PHYCLKSTB_Msk))
                if (--u32TimeOutCnt == 0) break;

            husbd->BUSINTEN = (HSUSBD_BUSINTEN_RSTIEN_Msk | HSUSBD_BUSINTEN_SUSPENDIEN_Msk);
            husbd->BUSINTSTS = HSUSBD_BUSINTSTS_RESUMEIF_Msk;
            event |= ARM_USBD_EVENT_RESUME;

        }

        if (u32BusSts & HSUSBD_BUSINTSTS_SUSPENDIF_Msk)
        {
            husbd->BUSINTEN = (HSUSBD_BUSINTEN_RSTIEN_Msk | HSUSBD_BUSINTEN_RESUMEIEN_Msk | HSUSBD_BUSINTEN_SUSPENDIEN_Msk);
            husbd->BUSINTSTS = HSUSBD_BUSINTSTS_SUSPENDIF_Msk;
            event |= ARM_USBD_EVENT_SUSPEND;
        }

        if (u32BusSts & HSUSBD_BUSINTSTS_HISPDIF_Msk)
        {
            husbd->CEPINTEN = HSUSBD_CEPINTEN_SETUPPKIEN_Msk;
            husbd->BUSINTSTS = HSUSBD_BUSINTSTS_HISPDIF_Msk;
        }

        if (u32BusSts & HSUSBD_BUSINTSTS_DMADONEIF_Msk)
        {
            husbd->BUSINTSTS = HSUSBD_BUSINTSTS_DMADONEIF_Msk;
        }

        if (u32BusSts & HSUSBD_BUSINTSTS_PHYCLKVLDIF_Msk)
            husbd->BUSINTSTS = HSUSBD_BUSINTSTS_PHYCLKVLDIF_Msk;

        if (u32BusSts & HSUSBD_BUSINTSTS_VBUSDETIF_Msk)
        {
            if (husbd->PHYCTL & HSUSBD_PHYCTL_VBUSDET_Msk)
            {
                // USB Plug In
                husbd->PHYCTL |= (HSUSBD_PHYCTL_PHYEN_Msk | HSUSBD_PHYCTL_DPPUEN_Msk);

                u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

                while (!(husbd->PHYCTL & HSUSBD_PHYCTL_PHYCLKSTB_Msk))
                    if (--u32TimeOutCnt == 0) break;

                husbd->OPER = HSUSBD_OPER_HISPDEN_Msk;
                husbd->OPER |= HSUSBD_OPER_HISHSEN_Msk;
                event |= ARM_USBD_EVENT_VBUS_ON;
            }
            else
            {
                // USB Un-plug
                husbd->PHYCTL &= ~HSUSBD_PHYCTL_DPPUEN_Msk;
                husbd->OPER &= ~HSUSBD_OPER_HISHSEN_Msk;
                event |= ARM_USBD_EVENT_VBUS_OFF;
            }

            husbd->BUSINTSTS = HSUSBD_BUSINTSTS_VBUSDETIF_Msk;
        }

    }

    if ((ptr_usbd_info->ptr_rw_info->cb_device_event != NULL) && (event != 0U))
    {
        ptr_usbd_info->ptr_rw_info->cb_device_event(event);
    }

    //------------------------------------------------------------------
    // Control endpoint event
    if (u32IntSts & HSUSBD_GINTSTS_CEPIF_Msk)
    {
        u32EpIntSts = husbd->CEPINTSTS & husbd->CEPINTEN;

        if (u32EpIntSts & HSUSBD_CEPINTSTS_SETUPTKIF_Msk)
        {
            husbd->CEPINTSTS = HSUSBD_CEPINTSTS_SETUPTKIF_Msk;
            return;
        }

        if (u32EpIntSts & HSUSBD_CEPINTSTS_SETUPPKIF_Msk)
        {
            husbd->CEPINTSTS = HSUSBD_CEPINTSTS_SETUPPKIF_Msk;
            USBD_SetupStage(ptr_usbd_info);
            return;
        }

        if (u32EpIntSts & HSUSBD_CEPINTSTS_OUTTKIF_Msk)
        {
            husbd->CEPINTSTS = HSUSBD_CEPINTSTS_OUTTKIF_Msk;
            husbd->CEPINTEN = HSUSBD_CEPINTEN_STSDONEIEN_Msk;
            return;
        }

        if (u32EpIntSts & HSUSBD_CEPINTSTS_INTKIF_Msk)
        {
            husbd->CEPINTSTS = HSUSBD_CEPINTSTS_INTKIF_Msk;

            if (!(u32EpIntSts & HSUSBD_CEPINTSTS_STSDONEIF_Msk))
            {
                husbd->CEPINTSTS = HSUSBD_CEPINTSTS_TXPKIF_Msk;

                ptr_ep = &ptr_usbd_info->ptr_rw_info->cep_info[1];

                uint8_t *data_to_transfer = ptr_ep->data + ptr_ep->num_transferred_total;
                uint32_t num_to_transfer  = ptr_ep->num - ptr_ep->num_transferred_total;

                if (num_to_transfer > ptr_ep->max_packet_size)
                {
                    num_to_transfer = ptr_ep->max_packet_size;
                }

                USBD_WriteEpBuffer((uint32_t *)&husbd->CEPDAT, data_to_transfer, num_to_transfer);

                ptr_ep->num_transferred_total += num_to_transfer;

                husbd->CEPINTEN = HSUSBD_CEPINTEN_TXPKIEN_Msk;
                husbd->CEPTXCNT = num_to_transfer;
            }
            else
            {
                husbd->CEPINTSTS = HSUSBD_CEPINTSTS_TXPKIF_Msk;
                husbd->CEPINTEN = (HSUSBD_CEPINTEN_TXPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);
            }

            return;
        }

        if (u32EpIntSts & HSUSBD_CEPINTSTS_PINGIF_Msk)
        {
            husbd->CEPINTSTS = HSUSBD_CEPINTSTS_PINGIF_Msk;
            return;
        }

        if (u32EpIntSts & HSUSBD_CEPINTSTS_TXPKIF_Msk)
        {
            ptr_ep = &ptr_usbd_info->ptr_rw_info->cep_info[1];
            husbd->CEPINTSTS = HSUSBD_CEPINTSTS_STSDONEIF_Msk;
            husbd->CEPCTL = HSUSBD_CEPCTL_NAKCLR;

            if (ptr_ep->num_transferred_total != ptr_ep->num)
            {
                husbd->CEPINTSTS = HSUSBD_CEPINTSTS_INTKIF_Msk;
                husbd->CEPINTEN = HSUSBD_CEPINTEN_INTKIEN_Msk;
            }
            else
            {
                ptr_usbd_info->ptr_rw_info->cep_event |= ARM_USBD_EVENT_IN;
                husbd->CEPINTSTS = HSUSBD_CEPINTSTS_STSDONEIF_Msk;
                husbd->CEPINTEN = (HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);
            }

            husbd->CEPINTSTS = HSUSBD_CEPINTSTS_TXPKIF_Msk;
            return;
        }

        if (u32EpIntSts & HSUSBD_CEPINTSTS_RXPKIF_Msk)
        {
            husbd->CEPINTSTS = HSUSBD_CEPINTSTS_RXPKIF_Msk;
            husbd->CEPCTL = HSUSBD_CEPCTL_NAKCLR;
            husbd->CEPINTEN = (HSUSBD_CEPINTEN_SETUPPKIEN_Msk | HSUSBD_CEPINTEN_STSDONEIEN_Msk);
            ptr_usbd_info->ptr_rw_info->cep_event |= ARM_USBD_EVENT_OUT;
            return;
        }

        if (u32EpIntSts & HSUSBD_CEPINTSTS_NAKIF_Msk)
        {
            husbd->CEPINTSTS = HSUSBD_CEPINTSTS_NAKIF_Msk;
            return;
        }

        if (u32EpIntSts & HSUSBD_CEPINTSTS_STALLIF_Msk)
        {
            husbd->CEPINTSTS = HSUSBD_CEPINTSTS_STALLIF_Msk;
            return;
        }

        if (u32EpIntSts & HSUSBD_CEPINTSTS_ERRIF_Msk)
        {
            husbd->CEPINTSTS = HSUSBD_CEPINTSTS_ERRIF_Msk;
            return;
        }

        if (u32EpIntSts & HSUSBD_CEPINTSTS_STSDONEIF_Msk)
        {

            if ((ptr_usbd_info->ptr_rw_info->cb_endpoint_event != NULL) && (ptr_usbd_info->ptr_rw_info->cep_event != 0))
            {
                if ((ptr_usbd_info->ptr_rw_info->cep_event & ARM_USBD_EVENT_IN) && (husbd->CEPINTSTS & HSUSBD_CEPINTSTS_INTKIF_Msk))
                {
                    ptr_usbd_info->ptr_rw_info->cep_event &= ~ARM_USBD_EVENT_IN;
                    ptr_usbd_info->ptr_rw_info->cb_endpoint_event(0x80, ARM_USBD_EVENT_IN);
                }

                if ((ptr_usbd_info->ptr_rw_info->cep_event & ARM_USBD_EVENT_OUT) && (husbd->CEPINTSTS & HSUSBD_CEPINTSTS_OUTTKIF_Msk))
                {
                    ptr_usbd_info->ptr_rw_info->cep_event &= ~ARM_USBD_EVENT_OUT;
                    ptr_usbd_info->ptr_rw_info->cb_endpoint_event(0x00, ARM_USBD_EVENT_OUT);
                }
            }

            husbd->CEPINTSTS = HSUSBD_CEPINTSTS_STSDONEIF_Msk;
            husbd->CEPINTEN = HSUSBD_CEPINTEN_SETUPPKIEN_Msk;
            return;
        }

        if (u32EpIntSts & HSUSBD_CEPINTSTS_BUFFULLIF_Msk)
        {
            husbd->CEPINTSTS = HSUSBD_CEPINTSTS_BUFFULLIF_Msk;
            return;
        }

        if (u32EpIntSts & HSUSBD_CEPINTSTS_BUFEMPTYIF_Msk)
        {
            husbd->CEPINTSTS = HSUSBD_CEPINTSTS_BUFEMPTYIF_Msk;
            return;
        }
    }

    //------------------------------------------------------------------
    // Endpoint event
    if (u32IntSts & \
            (HSUSBD_GINTSTS_EPAIF_Msk | HSUSBD_GINTSTS_EPBIF_Msk | HSUSBD_GINTSTS_EPCIF_Msk | HSUSBD_GINTSTS_EPDIF_Msk | HSUSBD_GINTSTS_EPEIF_Msk | HSUSBD_GINTSTS_EPFIF_Msk | \
             HSUSBD_GINTSTS_EPGIF_Msk | HSUSBD_GINTSTS_EPHIF_Msk | HSUSBD_GINTSTS_EPIIF_Msk | HSUSBD_GINTSTS_EPJIF_Msk | HSUSBD_GINTSTS_EPKIF_Msk | HSUSBD_GINTSTS_EPLIF_Msk | \
             HSUSBD_GINTSTS_EPMIF_Msk | HSUSBD_GINTSTS_EPNIF_Msk | HSUSBD_GINTSTS_EPOIF_Msk | HSUSBD_GINTSTS_EPPIF_Msk | HSUSBD_GINTSTS_EPQIF_Msk | HSUSBD_GINTSTS_EPRIF_Msk))
    {
        EP_Num_t ep_index;
        uint32_t mask;
        USBD_EP_t *ep;

        for (ep_index = PERIPH_EPA, mask = HSUSBD_GINTSTS_EPAIF_Msk, ep = &husbd->EP[PERIPH_EPA]; ep_index < PERIPH_MAX_EP;
                ep_index++, mask <<= 1U, ep++)
        {
            if (u32IntSts & mask)
            {
                uint32_t u32EpIntSts = ep->EPINTSTS & ep->EPINTEN;
                // Clear endpoint event flag
                ep->EPINTSTS = u32EpIntSts;
                uint8_t const ep_addr = ptr_usbd_info->ptr_rw_info->ep_info[ep_index].ep_addr;

                if (EP_DIR(ep_addr))
                {
                    if (u32EpIntSts & HSUSBD_EPINTSTS_BUFEMPTYIF_Msk)
                    {
                        USBD_DataInStage(ptr_usbd_info, ep_addr);
                    }
                    else if (u32EpIntSts & HSUSBD_EPINTSTS_TXPKIF_Msk)
                    {
                        if ((ptr_usbd_info->ptr_rw_info->cb_endpoint_event != NULL))
                        {
                            ptr_usbd_info->ptr_rw_info->cb_endpoint_event(ep_addr, ARM_USBD_EVENT_IN);
                            ep->EPINTEN = 0;
                        }
                    }
                }
                else
                {
                    USBD_DataOutStage(ptr_usbd_info, ep_addr);
                }
            }
        }
    }
}

// End USBD Interface
#if (RTE_USBD1)
NVT_ITCM void HSUSBD_IRQHandler(void)
{
    const USBD_Info_t *ptr_usbd_info = NULL;
    ptr_usbd_info = USBD_GetInfo(HSUSBD);

    if (ptr_usbd_info)
    {
        HSUSBDn_IRQHandler(ptr_usbd_info);
    }
}

#endif
#endif  // DRIVER_CONFIG_VALID

/*! \endcond */
