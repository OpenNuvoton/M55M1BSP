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
#if (RTE_USBD0 == 1)
// *****************************************************************************
// Compile-time configuration (that can be externally overridden if necessary)

// Maximum number of endpoints
#ifndef USBD_MAX_ENDPOINT_NUM
    #define USBD_MAX_ENDPOINT_NUM           (USBD_MAX_EP)
#endif

// Maximum packet size for Endpoint 0
#ifndef USBD_EP0_MAX_PACKET_SIZE
    #define USBD_EP0_MAX_PACKET_SIZE        (64)
#endif

// Maximum RAM for endpoint buffers
#ifndef USBD_BUF_SIZE
    #define USBD_BUF_SIZE                   (1536)
#endif
// *****************************************************************************

// Macros
#define ARM_USBD_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */
// Macro for porting compatibility
#define USBD_t          USBD_T
#define USBD_EP_t       USBD_EP_T
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
    ARM_DRIVER_USBD Driver_USBD##n = {      \
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

#define PERIPH_SETUP_BUF_BASE  0
#define PERIPH_SETUP_BUF_LEN   8
#define PERIPH_EP0_BUF_BASE    (PERIPH_SETUP_BUF_BASE + PERIPH_SETUP_BUF_LEN)
#define PERIPH_EP0_BUF_LEN     USBD_EP0_MAX_PACKET_SIZE
#define PERIPH_EP1_BUF_BASE    (PERIPH_EP0_BUF_BASE + PERIPH_EP0_BUF_LEN)
#define PERIPH_EP1_BUF_LEN     USBD_EP0_MAX_PACKET_SIZE
#define PERIPH_EP2_BUF_BASE    (PERIPH_EP1_BUF_BASE + PERIPH_EP1_BUF_LEN)

typedef enum
{
    PERIPH_EP0 = 0,
    PERIPH_EP1 = 1,
    PERIPH_EP2 = 2,
    PERIPH_EP3 = 3,
    PERIPH_EP4 = 4,
    PERIPH_EP5 = 5,
    PERIPH_EP6 = 6,
    PERIPH_EP7 = 7,
    PERIPH_EP8 = 8,
    PERIPH_EP9 = 9,
    PERIPH_EP10 = 10,
    PERIPH_EP11 = 11,
    PERIPH_EP12 = 12,
    PERIPH_EP13 = 13,
    PERIPH_EP14 = 14,
    PERIPH_EP15 = 15,
    PERIPH_EP16 = 16,
    PERIPH_EP17 = 17,
    PERIPH_EP18 = 18,
    PERIPH_EP19 = 19,
    PERIPH_EP20 = 20,
    PERIPH_EP21 = 21,
    PERIPH_EP22 = 22,
    PERIPH_EP23 = 23,
    PERIPH_EP24 = 24,
    PERIPH_EP25 = 25,
    PERIPH_EP26 = 26,
    PERIPH_EP27 = 27,
    PERIPH_EP28 = 28,
    PERIPH_EP29 = 29,
    PERIPH_EP30 = 30,
    PERIPH_EP31 = 31,
    PERIPH_MAX_EP = USBD_MAX_ENDPOINT_NUM,
} EP_Num_t;

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
    volatile uint32_t             num_transferring;       // Number of transferred bytes in last transfer
    uint8_t                       ep_addr;                // Endpoint address
} EP_Info_t;

// Instance run-time information (RW)
typedef struct
{
    ARM_USBD_SignalDeviceEvent_t  cb_device_event;        // Device event callback
    ARM_USBD_SignalEndpointEvent_t cb_endpoint_event;     // Endpoint event callback
    DriverStatus_t                drv_status;             // Driver status
    USBD_State_t                  usbd_state;             // USB Device state
    uint32_t                      bufseg_addr;            // Allocated USB RAM
    volatile uint32_t             setup_received;         // Setup Packet received flag (0 - not received or read already, 1 - received and unread yet)
    volatile uint8_t              setup_packet[8];        // Setup Packet data
    EP_Info_t                     ep_info[PERIPH_MAX_EP]; // Endpoint information
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
static int32_t                  USBDn_DeviceSetAddress(const USBD_Info_t *const ptr_usbd_info, uint8_t dev_addr);
static int32_t                  USBDn_ReadSetupPacket(const USBD_Info_t *const ptr_usbd_info, uint8_t *setup);
static int32_t                  USBDn_EndpointConfigure(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_max_packet_size);
static int32_t                  USBDn_EndpointUnconfigure(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr);
static int32_t                  USBDn_EndpointStall(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr, bool stall);
static int32_t                  USBDn_EndpointTransfer(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr, uint8_t *data, uint32_t num);
static uint32_t                 USBDn_EndpointTransferGetResult(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr);
static int32_t                  USBDn_EndpointTransferAbort(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr);
static uint16_t                 USBDn_GetFrameNumber(const USBD_Info_t *const ptr_usbd_info);

#if(RTE_USBD0)
static const RO_Info_t usbd0_ro_info = {USBD, USBD_IRQn};
static        RW_Info_t         usbd0_rw_info USBDn_SECTION(0);
static  const USBD_Info_t       usbd0_info = { &usbd0_ro_info,
                                               &usbd0_rw_info                                                                                                 \
                                             };
FUNCS_DECLARE(0)
FUNCS_DEFINE(0)
USBD_DRIVER(0)
#endif

// List of available USBD instance infos
static const USBD_Info_t *const usbd_info_list[] =
{
#if (RTE_USBD0)
    &usbd0_info,
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
  \param[in]   husbd Pointer to USBD handle structure (USBD_t)
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

    for (ep_index = PERIPH_EP0, ptr_ep = &ptr_usbd_info->ptr_rw_info->ep_info[PERIPH_EP0], ep = husbd->EP;
            ep_index < PERIPH_MAX_EP;
            ep_index++, ptr_ep++, ep++)
    {
        if (add)
        {
            if (0 == (ep->CFG & USBD_CFG_STATE_Msk)) return ep;
        }
        else
        {
            if (ptr_ep->ep_addr == ep_addr) return ep;
        }
    }

    return NULL;
}

static void USBD_EndpointXfer(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr, uint8_t *data, uint32_t num)
{
    USBD_EP_t *ep = USBD_EndpointEntry(ptr_usbd_info, ep_addr, false);
    uint8_t periph_epnum = ep - ptr_usbd_info->ptr_ro_info->ptr_USBD->EP;

    if (EP_DIR(ep_addr))
    {
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + ep->BUFSEG), (uint8_t *)data, num);
        // Prepare the data for next IN transfer
        ep->MXPLD = num;
    }
    else
    {
        // Ready to get next OUT transfer
        ep->MXPLD = ptr_usbd_info->ptr_rw_info->ep_info[periph_epnum].max_packet_size;
    }
}

/**
  \fn          void USBD_EndpointConfigureBuffer (const USBD_Info_t *const ptr_usbd_info)
  \brief       Configure and reassign USB endpoint buffer addresses in USB SRAM.
  \detail      This function resets the buffer allocation pointer and reconfigures the buffer segmentation for all enabled endpoints.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
*/
static void USBD_EndpointConfigureBuffer(const USBD_Info_t *const ptr_usbd_info)
{
    // USB RAM beyond what we've allocated above is available to the user(Control endpoint and setup package used.)
    ptr_usbd_info->ptr_rw_info->bufseg_addr = PERIPH_EP2_BUF_BASE;

    // Reconfigures the buffer segmentation for all enabled endpoints.
    EP_Num_t ep_index;
    USBD_EP_t *ep;

    for (ep_index = PERIPH_EP2, ep = &ptr_usbd_info->ptr_ro_info->ptr_USBD->EP[PERIPH_EP2]; ep_index < PERIPH_MAX_EP; ep_index++, ep++)
    {
        if (0 == (ep->CFG & USBD_CFG_STATE_Msk)) continue;

        // Update the Endpoint Buffer Segmentation
        ep->BUFSEG = ptr_usbd_info->ptr_rw_info->bufseg_addr;
        ptr_usbd_info->ptr_rw_info->bufseg_addr += ptr_usbd_info->ptr_rw_info->ep_info[ep_index].max_packet_size;
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
  \fn          ARM_USBD_CAPABILITIES USBDn_GetCapabilities (const USBD_Info_t *const ptr_usbd_info)
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
  \fn          int32_t USBDn_Initialize (const USBD_Info_t * const ptr_usbd_info,
                                         ARM_USBD_SignalDeviceEvent_t cb_device_event,
                                         ARM_USBD_SignalEndpointEvent_t cb_endpoint_event)
  \brief       Initialize USB Device Interface.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
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
  \fn          int32_t USBDn_Uninitialize (const USBD_Info_t *const ptr_usbd_info)
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
    USBD_t                         *husbd = ptr_usbd_info->ptr_ro_info->ptr_USBD;

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
            // Initialize interrupts.
            NVIC_EnableIRQ(ptr_usbd_info->ptr_ro_info->irq_n);

            // Set driver status to powered
            ptr_usbd_info->ptr_rw_info->drv_status.powered = 1U;

            // Initial USB engine
            husbd->ATTR = 0x7D0U;

            // Clear USB-related interrupts before enable interrupt
            husbd->INTSTS = (USBD_INT_BUS | USBD_INT_USB | USBD_INT_FLDET | USBD_INT_WAKEUP);

            // Enable USB-related interrupts.
            husbd->INTEN |= (USBD_INT_BUS | USBD_INT_USB | USBD_INT_FLDET | USBD_INT_WAKEUP);

            // Clear SOF
            husbd->INTSTS = USBD_INTSTS_SOFIF_Msk;

            break;

        case ARM_POWER_OFF:
            husbd->ATTR &= ~USBD_PHY_EN;

            for (EP_Num_t ep_index = PERIPH_EP0; ep_index < PERIPH_MAX_EP; ep_index++)
            {
                husbd->EP[ep_index].CFGP |= USBD_CFGP_CLRRDY_Msk;
                husbd->EP[ep_index].CFG = 0U;
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

    // Disable software-disconnect function
    ptr_usbd_info->ptr_ro_info->ptr_USBD->SE0 &= ~USBD_DRVSE0;

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

    // Enable software-disconnect function.
    ptr_usbd_info->ptr_ro_info->ptr_USBD->SE0 |= USBD_DRVSE0;
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

    // Enable PHY before sending Resume('K') state
    ptr_usbd_info->ptr_ro_info->ptr_USBD->ATTR |= USBD_PHY_EN;
    ptr_usbd_info->ptr_ro_info->ptr_USBD->ATTR |= USBD_RWAKEUP;

    // Delay for 1 ms
    uint32_t count = SystemCoreClock / 1000;

    while (count--) __NOP();

    ptr_usbd_info->ptr_ro_info->ptr_USBD->ATTR &= ~USBD_ATTR_RWAKEUP_Msk;
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

    uint32_t addr = ptr_usbd_info->ptr_ro_info->ptr_USBD->FADDR;

    if ((addr != dev_addr) && (addr == 0U))
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
static int32_t USBDn_EndpointConfigure(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr,
                                       uint8_t ep_type,
                                       uint16_t ep_max_packet_size)
{
    uint32_t   ep_dir_mask, ep_type_mask;
    uint8_t    ep_num = EP_NUM(ep_addr);
    uint8_t    ep_dir = EP_DIR(ep_addr);
    USBD_EP_t  *ep;

    // Unconfigure Endpoint the endpoint if it has been used
    USBDn_EndpointUnconfigure(ptr_usbd_info, ep_addr);

    // Open endpoint
    ep = USBD_EndpointEntry(ptr_usbd_info, ep_addr, true);

    // Error if all periph endpoints used
    if (ep == NULL)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    EP_Info_t *ptr_ep = &ptr_usbd_info->ptr_rw_info->ep_info[ep - ptr_usbd_info->ptr_ro_info->ptr_USBD->EP];

    // Error if USB buffer is insufficient
    if (ptr_usbd_info->ptr_rw_info->bufseg_addr + ep_max_packet_size > USBD_BUF_SIZE)
    {
        return ARM_DRIVER_ERROR;
    }

    // Store max packet size information
    ptr_ep->max_packet_size = ep_max_packet_size;

    // configured endpoint
    if (EP_NUM(ep_addr) != 0U)
    {
        ep_dir_mask = ep_dir ? USBD_CFG_EPMODE_IN : USBD_CFG_EPMODE_OUT;

        if (ep_type == ARM_USB_ENDPOINT_ISOCHRONOUS)
            ep_type_mask = USBD_CFG_TYPE_ISO;
        else
            ep_type_mask = 0;

        ep->CFG = (ep_dir_mask | ep_type_mask | ep_num);
        ptr_ep->ep_addr = ep_addr;
    }
    else
    {
        // Control endpoint
        if (ep_dir)
        {
            ptr_usbd_info->ptr_ro_info->ptr_USBD->EP[PERIPH_EP0].CFG = USBD_CFG_CSTALL_Msk | USBD_CFG_EPMODE_IN;
            ptr_usbd_info->ptr_rw_info->ep_info[PERIPH_EP0].ep_addr = 0x80;
        }
        else
        {
            ptr_usbd_info->ptr_ro_info->ptr_USBD->EP[PERIPH_EP1].CFG = USBD_CFG_CSTALL_Msk | USBD_CFG_EPMODE_OUT;
            ptr_usbd_info->ptr_rw_info->ep_info[PERIPH_EP1].ep_addr = 0x00;
        }
    }

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
    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    USBD_EP_t *ep = USBD_EndpointEntry(ptr_usbd_info, ep_addr, false);

    if (ep != NULL)
    {
        EP_Info_t *ptr_ep = &ptr_usbd_info->ptr_rw_info->ep_info[ep - ptr_usbd_info->ptr_ro_info->ptr_USBD->EP];

        // Clear Endpoint information
        memset((void *)ptr_ep, 0, sizeof(EP_Info_t));
        ptr_ep->data = NULL;
        ptr_ep->num = 0U;
        // Clear Endpoint configure
        ep->CFG = 0U;
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

    USBD_EP_t *ep = USBD_EndpointEntry(ptr_usbd_info, ep_addr, false);

    if (stall != 0U)
    {
        // Set STALL
        ep->CFGP |= USBD_CFGP_SSTALL_Msk;
    }
    else
    {
        // Clear STALL and reset Packet Identifier(PID)
        ep->CFGP &= ~USBD_CFGP_SSTALL_Msk;
        ep->CFG &= ~USBD_CFG_DSQSYNC_Msk;
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

    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    USBD_EP_t *ep = USBD_EndpointEntry(ptr_usbd_info, ep_addr, false);
    EP_Info_t *ptr_ep = &ptr_usbd_info->ptr_rw_info->ep_info[ep - ptr_usbd_info->ptr_ro_info->ptr_USBD->EP];
    // Clear number of transferred bytes
    ptr_ep->num_transferred_total = 0U;

    // Register pointer to data and number of bytes to transfer for Endpoint
    ptr_ep->data = data;
    ptr_ep->num = num;

    if (ep_addr == 0x80U)
    {
        //Set EP0 IN token PID to DATA1
        ep->CFG |= USBD_CFG_DSQSYNC_Msk;
    }

    ptr_ep->num_transferring = num;

    if (ptr_ep->max_packet_size < num)
    {
        // For Endpoint total transfer is done in transfers of maximum packet size at a time
        ptr_ep->num_transferring = ptr_ep->max_packet_size;
    }

    USBD_EndpointXfer(ptr_usbd_info, ep_addr, data, ptr_ep->num_transferring);

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
    USBD_EP_t *ep = USBD_EndpointEntry(ptr_usbd_info, ep_addr, false);

    if (ptr_usbd_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return 0U;
    }

    return ptr_usbd_info->ptr_rw_info->ep_info[ep - ptr_usbd_info->ptr_ro_info->ptr_USBD->EP].num_transferred_total;
}

static int32_t USBDn_EndpointTransferAbort(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr)
{
    USBD_EP_t *ep = USBD_EndpointEntry(ptr_usbd_info, ep_addr, false);

    // STOP_TRANSACTION
    ep->CFGP |= USBD_CFGP_CLRRDY_Msk;

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
    return ptr_usbd_info->ptr_ro_info->ptr_USBD->FN;
}

// Event functions *****************************************************************
/**
  \fn          void USBD_DataOutStage (const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr)
  \brief       Data OUT stage.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
*/
static void USBD_DataOutStage(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr)
{
    uint32_t   num_transferred;
    uint32_t   num_to_transfer;
    uint8_t   *data_to_transfer;
    uint32_t   event;

    USBD_EP_t *ep = USBD_EndpointEntry(ptr_usbd_info, ep_addr, false);
    EP_Info_t *ptr_ep = &ptr_usbd_info->ptr_rw_info->ep_info[ep - ptr_usbd_info->ptr_ro_info->ptr_USBD->EP];

    event  = 0U;

    if (EP_NUM(ep_addr) != 0U)
    {
        ptr_ep->num_transferred_total = ep->MXPLD;
        USBD_MemCopy((uint8_t *)ptr_ep->data, (uint8_t *)(USBD_BUF_BASE + ep->BUFSEG), ptr_ep->num_transferred_total);
        event = ARM_USBD_EVENT_OUT;
    }
    else
    {
        num_transferred = ep->MXPLD;
        ptr_ep->num_transferred_total += num_transferred;

        if ((num_transferred < ptr_ep->max_packet_size) || (ptr_ep->num_transferred_total == ptr_ep->num))
        {
            // If all data was transferred
            event = ARM_USBD_EVENT_OUT;
        }
        else
        {
            // If there is more data to transfer
            data_to_transfer = ptr_ep->data + ptr_ep->num_transferred_total;
            num_to_transfer  = ptr_ep->num - ptr_ep->num_transferred_total;

            if (num_to_transfer > ptr_ep->max_packet_size)
            {
                num_to_transfer = ptr_ep->max_packet_size;
            }

            ptr_ep->num_transferring = num_to_transfer;
            USBD_EndpointXfer(ptr_usbd_info, ep_addr, data_to_transfer, ptr_ep->num_transferring);
        }
    }

    if ((ptr_usbd_info->ptr_rw_info->cb_endpoint_event != NULL) && (event != 0U))
    {
        ptr_usbd_info->ptr_rw_info->cb_endpoint_event(ep_addr, event);
    }
}

/**
  \fn          void USBD_DataInStage (const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr)
  \brief       Data IN stage.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  \param[in]   ep_addr  Endpoint Address
                - ep_addr.0..3: Address
                - ep_addr.7:    Direction
*/
static void USBD_DataInStage(const USBD_Info_t *const ptr_usbd_info, uint8_t ep_addr)
{
    uint32_t   num_to_transfer;
    uint8_t   *data_to_transfer;
    uint32_t   event;

    USBD_EP_t *ep = USBD_EndpointEntry(ptr_usbd_info, ep_addr, false);
    EP_Info_t *ptr_ep = &ptr_usbd_info->ptr_rw_info->ep_info[ep - ptr_usbd_info->ptr_ro_info->ptr_USBD->EP];

    event  = 0U;

    // Update transferred number
    ptr_ep->num_transferred_total += ptr_ep->num_transferring;

    if (ptr_ep->num_transferred_total == ptr_ep->num)
    {
        // If all data was transferred
        event = ARM_USBD_EVENT_IN;
    }
    else
    {
        // If there is more data to transfer
        data_to_transfer = ptr_ep->data + ptr_ep->num_transferred_total;
        num_to_transfer  = ptr_ep->num - ptr_ep->num_transferred_total;

        if (num_to_transfer > ptr_ep->max_packet_size)
        {
            num_to_transfer = ptr_ep->max_packet_size;
        }

        ptr_ep->num_transferring = num_to_transfer;
        USBD_EndpointXfer(ptr_usbd_info, ep_addr, data_to_transfer, num_to_transfer);
    }

    if ((ptr_usbd_info->ptr_rw_info->cb_endpoint_event != NULL) && (event != 0U))
    {
        ptr_usbd_info->ptr_rw_info->cb_endpoint_event(ep_addr, event);
    }
}

/**
  \fn          USBD_SetupStage (const USBD_Info_t *const ptr_usbd_info)
  \brief       Setup stage.
  \param[in]   ptr_usbd_info    Pointer to USBD info structure (USBD_Info_t)
  */
static void USBD_SetupStage(const USBD_Info_t *const ptr_usbd_info)
{
    USBD_MemCopy((uint8_t *)ptr_usbd_info->ptr_rw_info->setup_packet, (uint8_t *)USBD_BUF_BASE, 8U);
    ptr_usbd_info->ptr_rw_info->setup_received = 1U;

    if (ptr_usbd_info->ptr_rw_info->cb_endpoint_event != NULL)
    {
        ptr_usbd_info->ptr_rw_info->cb_endpoint_event(0U, ARM_USBD_EVENT_SETUP);
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

    for (EP_Num_t ep_index = PERIPH_EP0; ep_index < PERIPH_MAX_EP; ep_index++)
    {
        husbd->EP[ep_index].CFG = 0U;
    }

    husbd->FADDR = 0U;
    // Reset USBD state information
    ptr_usbd_info->ptr_rw_info->usbd_state.speed  = ARM_USB_SPEED_FULL;
    ptr_usbd_info->ptr_rw_info->usbd_state.active = 0U;

    // Buffer for setup packet
    husbd->STBUFSEG = PERIPH_SETUP_BUF_BASE;
    // Buffer for control endpoint, share same buffer.
    husbd->EP[PERIPH_EP0].BUFSEG = PERIPH_EP0_BUF_BASE;
    husbd->EP[PERIPH_EP1].BUFSEG = PERIPH_EP1_BUF_BASE;
    // Configure Endpoint 0 OUT
    (void)USBDn_EndpointConfigure(ptr_usbd_info, 0x00U, ARM_USB_ENDPOINT_CONTROL, USBD_EP0_MAX_PACKET_SIZE);
    // Configure Endpoint 0 IN
    (void)USBDn_EndpointConfigure(ptr_usbd_info, 0x80U, ARM_USB_ENDPOINT_CONTROL, USBD_EP0_MAX_PACKET_SIZE);

    if (ptr_usbd_info->ptr_rw_info->cb_device_event != NULL)
    {
        ptr_usbd_info->ptr_rw_info->cb_device_event(ARM_USBD_EVENT_RESET);
    }

    // After reset we consider USB as active
    ptr_usbd_info->ptr_rw_info->usbd_state.active = 1U;
}

// IRQ handler *********************************************************************
NVT_ITCM void USBDn_IRQHandler(const USBD_Info_t *const ptr_usbd_info)
{
    USBD_t *husbd = ptr_usbd_info->ptr_ro_info->ptr_USBD;
    volatile uint32_t u32IntSts = husbd->INTSTS;
    volatile uint32_t u32EpIntSts = husbd->EPINTSTS;
    volatile uint32_t u32BusSts = husbd->ATTR & 0x300f;
    uint32_t event = 0;

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_FLDET)
    {
        // Floating detect
        husbd->INTSTS = USBD_INTSTS_FLDET;

        if (husbd->VBUSDET & USBD_VBUSDET_VBUSDET_Msk)
        {
            event |= ARM_USBD_EVENT_VBUS_ON;
            // USB Plug In
            husbd->ATTR |= 0x7D0U;
        }
        else
        {
            event |= ARM_USBD_EVENT_VBUS_OFF;
            // USB Un-plug
            husbd->ATTR &= ~USBD_USB_EN;
        }
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_WAKEUP)
    {
        // Clear event flag
        husbd->INTSTS = USBD_INTSTS_WAKEUP;
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_BUS)
    {
        // Clear event flag
        husbd->INTSTS = USBD_INTSTS_BUS;

        if (u32BusSts & USBD_STATE_USBRST)
        {
            // Bus reset
            USBD_BusReset(ptr_usbd_info);
            // Enable USB and enable PHY
            husbd->ATTR |= 0x7D0U;
        }

        if (u32BusSts & USBD_STATE_SUSPEND)
        {
            event |= ARM_USBD_EVENT_SUSPEND;
            // Disable PHY
            husbd->ATTR &= ~USBD_PHY_EN;
        }

        if (u32BusSts & USBD_STATE_RESUME)
        {
            event |= ARM_USBD_EVENT_RESUME;
            // Enable USB and enable PHY
            husbd->ATTR |= 0x7D0U;
        }
    }

    if ((ptr_usbd_info->ptr_rw_info->cb_device_event != NULL) && (event != 0U))
    {
        ptr_usbd_info->ptr_rw_info->cb_device_event(event);
    }

    //------------------------------------------------------------------
    if (u32IntSts & USBD_INTSTS_USB)
    {
        // USB event
        if (u32IntSts & USBD_INTSTS_SETUP)
        {
            // Clear event flag
            husbd->INTSTS = USBD_INTSTS_SETUP;
            // Clear the data IN/OUT ready flag of control end-points
            husbd->EP[PERIPH_EP0].CFGP |= USBD_CFGP_CLRRDY_Msk;
            husbd->EP[PERIPH_EP1].CFGP |= USBD_CFGP_CLRRDY_Msk;
            USBD_SetupStage(ptr_usbd_info);
        }

        // EP event
        if (u32EpIntSts)
        {
            EP_Num_t ep_index;
            uint32_t mask;

            for (ep_index = PERIPH_EP0, mask = USBD_EPINTSTS_EPEVT0_Msk; ep_index < PERIPH_MAX_EP; ep_index++, mask <<= 1U)
            {
                if (u32EpIntSts & mask)
                {
                    // Clear event flag
                    husbd->EPINTSTS = mask;
                    uint8_t const ep_addr = ptr_usbd_info->ptr_rw_info->ep_info[ep_index].ep_addr;

                    if (EP_DIR(ep_addr))
                    {
                        USBD_DataInStage(ptr_usbd_info, ep_addr);
                    }
                    else
                    {
                        USBD_DataOutStage(ptr_usbd_info, ep_addr);
                    }
                }
            }
        }
    }
}

// End USBD Interface
#if (RTE_USBD0)
NVT_ITCM void USBD_IRQHandler(void)
{
    const USBD_Info_t *ptr_usbd_info = NULL;
    ptr_usbd_info = USBD_GetInfo(USBD);

    if (ptr_usbd_info)
    {
        USBDn_IRQHandler(ptr_usbd_info);
    }
}

#endif
#endif  // DRIVER_CONFIG_VALID

/*! \endcond */
