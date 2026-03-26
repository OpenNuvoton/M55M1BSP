/*
 * Copyright (c) 2015-2020 Arm Limited. All rights reserved.
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

/* The implementation does not include pin/clock settings, which must be configured in the application */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Driver_CAN.h"
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

#define CAN_CLOCK_TOLERANCE (15U)
#define ARM_CAN_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0)

/* Driver Version */
static const ARM_DRIVER_VERSION can_driver_version = { ARM_CAN_API_VERSION, ARM_CAN_DRV_VERSION };
void CAN_SetConfig(CANFD_T *psCanfd, CANFD_FD_T *psCanfdStr);

/* CAN FD Rx FIFO 0 Mask helper macro. */
#define CANFD_RX_FIFO0_STD_EXACT(match1, match2)            ((1UL << 30) | (1UL << 27) | ((match1 & 0x7FF) << 16) | (match2 & 0x7FF))
/* CAN FD Rx FIFO 0 extended Mask helper macro - low. */
#define CANFD_RX_FIFO0_EXT_EXACT_LOW(match1)                ((1UL << 29) | (match1 & 0x1FFFFFFF))
/* CAN FD Rx FIFO 0 extended Mask helper macro - high. */
#define CANFD_RX_FIFO0_EXT_EXACT_HIGH(match2)               ((1UL << 30) | (match2 & 0x1FFFFFFF))

/* CAN FD Rx FIFO 1 Mask helper macro. */
#define CANFD_RX_FIFO1_STD_EXACT(match1, match2)            ((1UL << 30) | (2UL << 27) | ((match1 & 0x7FF) << 16) | (match2 & 0x7FF))
/* CANFD Rx FIFO 1 extended Mask helper macro - low. */
#define CANFD_RX_FIFO1_EXT_EXACT_LOW(match1)                ((2UL << 29) | (match1 & 0x1FFFFFFF))
/* CANFD Rx FIFO 1 extended Mask helper macro - high. */
#define CANFD_RX_FIFO1_EXT_EXACT_HIGH(match2)               ((1UL << 30) | (match2 & 0x1FFFFFFF))

/* CAN FD Rx FIFO 0 Mask helper macro. */
#define CANFD_RX_FIFO0_STD_MASKABLE(match, mask)            ((2UL << 30) | (1UL << 27) | ((match & 0x7FF) << 16) | (mask & 0x7FF))
/* CAN FD Rx FIFO 0 extended Mask helper macro - low. */
#define CANFD_RX_FIFO0_EXT_MASKABLE_LOW(match)              ((1UL << 29) | (match & 0x1FFFFFFF))
/* CAN FD Rx FIFO 0 extended Mask helper macro - high. */
#define CANFD_RX_FIFO0_EXT_MASKABLE_HIGH(mask)              ((2UL << 30) | (mask & 0x1FFFFFFF))

/* CAN FD Rx FIFO 1 Mask helper macro. */
#define CANFD_RX_FIFO1_STD_MASKABLE(match, mask)            ((2UL << 30) | (2UL << 27) | ((match & 0x7FF) << 16) | (mask & 0x7FF))
/* CANFD Rx FIFO 1 extended Mask helper macro - low. */
#define CANFD_RX_FIFO1_EXT_MASKABLE_LOW(match)              ((2UL << 29) | (match & 0x1FFFFFFF))
/* CANFD Rx FIFO 1 extended Mask helper macro - high. */
#define CANFD_RX_FIFO1_EXT_MASKABLE_HIGH(mask)              ((2UL << 30) | (mask & 0x1FFFFFFF))

/* CAN FD Rx FIFO 0 Mask helper macro. */
#define CANFD_RX_FIFO0_STD_RANGE(range1, range2)            ((0UL << 30) | (1UL << 27) | ((range1 & 0x7FF) << 16) | (range2 & 0x7FF))
/* CAN FD Rx FIFO 0 extended Mask helper macro - low. */
#define CANFD_RX_FIFO0_EXT_RANGE_LOW(range1)                ((1UL << 29) | (range1 & 0x1FFFFFFF))
/* CAN FD Rx FIFO 0 extended Mask helper macro - high. */
#define CANFD_RX_FIFO0_EXT_RANGE_HIGH(range2)               ((0UL << 30) | (range2 & 0x1FFFFFFF))

/* CAN FD Rx FIFO 1 Mask helper macro. */
#define CANFD_RX_FIFO1_STD_RANGE(range1, range2)            ((0UL << 30) | (2UL << 27) | ((range1 & 0x7FF) << 16) | (range2 & 0x7FF))
/* CANFD Rx FIFO 1 extended Mask helper macro - low. */
#define CANFD_RX_FIFO1_EXT_RANGE_LOW(range1)                ((2UL << 29) | (range1 & 0x1FFFFFFF))
/* CANFD Rx FIFO 1 extended Mask helper macro - high. */
#define CANFD_RX_FIFO1_EXT_RANGE_HIGH(range2)               ((0UL << 30) | (range2 & 0x1FFFFFFF))

/* Remove ID filter */
#define CANFD_RX_STD_REMOVE                                 ((0UL << 30) | (0UL << 27))
#define CANFD_RX_EXT_REMOVE_LOW                             ((0UL << 29))
#define CANFD_RX_EXT_REMOVE_HIGH                            (0UL)

#define CAN_RX_OBJ_NUM                  (1U)
#define CAN_TX_OBJ_NUM                  (1U)
#define CAN_TOTAL_OBJ_NUM               (CAN_RX_OBJ_NUM + CAN_TX_OBJ_NUM)
#define CAN_MAX_PORTS_NUM               (2U)
#define CAN0                            (0U)
#define CAN1                            (1U)

/* Driver Capabilities */
static const ARM_CAN_CAPABILITIES can_driver_capabilities =
{
    CAN_TOTAL_OBJ_NUM,  // Number of CAN Objects available
    1U,                 // supports  reentrant calls to ARM_CAN_MessageSend, ARM_CAN_MessageRead, ARM_CAN_ObjectConfigure and abort message sending used by ARM_CAN_Control.
    1U,                 // supports  CAN with Flexible Data-rate mode (CAN_FD)
    1U,                 // supports  restricted operation mode
    1U,                 // supports  bus monitoring mode
    1U,                 // supports  internal loopback mode
    1U,                 // supports  external loopback mode
    0U                  // Reserved (must be zero)
};

/* Object Capabilities */
static const ARM_CAN_OBJ_CAPABILITIES can_object_capabilities_rx =
{
    0U,   // Object does not support transmission
    1U,   // Object supports reception
    0U,   // Object does not support RTR reception and automatic Data transmission
    0U,   // Object does not support RTR transmission and automatic Data reception
    1U,   // Object does not allow assignment of multiple filters to it
    1U,   // Object does not support exact identifier filtering
    1U,   // Object does not support range identifier filtering
    1U,   // Object does not support mask identifier filtering
    2U,   // Object can buffer 2 messages
    0U    // Reserved (must be zero)
};

static const ARM_CAN_OBJ_CAPABILITIES can_object_capabilities_tx =
{
    1U,   // Object supports transmission
    0U,   // Object does not support reception
    0U,   // Object does not support RTR reception and automatic Data transmission
    0U,   // Object does not support RTR transmission and automatic Data reception
    0U,   // Object does not allow assignment of multiple filters to it
    0U,   // Object does not support exact identifier filtering
    0U,   // Object does not support range identifier filtering
    0U,   // Object does not support mask identifier filtering
    1U,   // Object can buffer 1 message
    0U    // Reserved (must be zero)
};

/* Macro for declaring functions */
#define FUNCS_DECLARE(n)                                                                                                                                    \
    static int32_t                  CAN##n##_Initialize            (ARM_CAN_SignalUnitEvent_t cb_unit_event, ARM_CAN_SignalObjectEvent_t cb_object_event);  \
    static int32_t                  CAN##n##_Uninitialize          (void);                                                                                  \
    static int32_t                  CAN##n##_PowerControl          (ARM_POWER_STATE state);                                                                 \
    static uint32_t                 CAN##n##_GetClock              (void);                                                                                  \
    static int32_t                  CAN##n##_SetBitrate            (ARM_CAN_BITRATE_SELECT select, uint32_t bitrate, uint32_t bit_segments);                \
    static int32_t                  CAN##n##_SetMode               (ARM_CAN_MODE mode);                                                                     \
    static ARM_CAN_OBJ_CAPABILITIES CAN##n##_ObjectGetCapabilities (uint32_t obj_idx);                                                                      \
    static int32_t                  CAN##n##_ObjectSetFilter       (uint32_t obj_idx,ARM_CAN_FILTER_OPERATION operation,uint32_t id,uint32_t arg);          \
    static int32_t                  CAN##n##_ObjectConfigure       (uint32_t obj_idx,ARM_CAN_OBJ_CONFIG obj_cfg);                                           \
    static int32_t                  CAN##n##_MessageSend           (uint32_t obj_idx,ARM_CAN_MSG_INFO *msg_info,const uint8_t *data,uint8_t size);          \
    static int32_t                  CAN##n##_MessageRead           (uint32_t obj_idx,ARM_CAN_MSG_INFO *msg_info,uint8_t *data,uint8_t size);                \
    static int32_t                  CAN##n##_Control               (uint32_t control,uint32_t arg);                                                         \
    static ARM_CAN_STATUS           CAN##n##_GetStatus             (void);

/* Macro for defining functions */
#define FUNCS_DEFINE(n)                                                                                                                                                                                                         \
    static int32_t                  CAN##n##_Initialize            (ARM_CAN_SignalUnitEvent_t cb_unit_event, ARM_CAN_SignalObjectEvent_t cb_object_event)   { return CANn_Initialize (n, cb_unit_event, cb_object_event); }     \
    static int32_t                  CAN##n##_Uninitialize          (void)                                                                                   { return CANn_Uninitialize (n); }                                   \
    static int32_t                  CAN##n##_PowerControl          (ARM_POWER_STATE state)                                                                  { return CANn_PowerControl (n, state); }                            \
    static uint32_t                 CAN##n##_GetClock              (void)                                                                                   { return CANn_GetClock (n); }                                       \
    static int32_t                  CAN##n##_SetBitrate            (ARM_CAN_BITRATE_SELECT select, uint32_t bitrate, uint32_t bit_segments)                 { return CANn_SetBitrate (n, select, bitrate, bit_segments); }      \
    static int32_t                  CAN##n##_SetMode               (ARM_CAN_MODE mode)                                                                      { return CANn_SetMode (n, mode); }                                  \
    static ARM_CAN_OBJ_CAPABILITIES CAN##n##_ObjectGetCapabilities (uint32_t obj_idx)                                                                       { return CANn_ObjectGetCapabilities (n, obj_idx); }                 \
    static int32_t                  CAN##n##_ObjectSetFilter       (uint32_t obj_idx,ARM_CAN_FILTER_OPERATION operation,uint32_t id,uint32_t arg)           { return CANn_ObjectSetFilter (n, obj_idx, operation, id, arg); }   \
    static int32_t                  CAN##n##_ObjectConfigure       (uint32_t obj_idx,ARM_CAN_OBJ_CONFIG obj_cfg)                                            { return CANn_ObjectConfigure (n, obj_idx, obj_cfg); }              \
    static int32_t                  CAN##n##_MessageSend           (uint32_t obj_idx,ARM_CAN_MSG_INFO *msg_info,const uint8_t *data,uint8_t size)           { return CANn_MessageSend (n, obj_idx, msg_info, data, size); }     \
    static int32_t                  CAN##n##_MessageRead           (uint32_t obj_idx,ARM_CAN_MSG_INFO *msg_info,uint8_t *data,uint8_t size)                 { return CANn_MessageRead (n, obj_idx, msg_info, data, size); }     \
    static int32_t                  CAN##n##_Control               (uint32_t control,uint32_t arg)                                                          { return CANn_Control (n, control, arg); }                          \
    static ARM_CAN_STATUS           CAN##n##_GetStatus             (void)                                                                                   { return CANn_GetStatus (n); }

/* Macro for defining driver structures */
#define CAN_DRIVER(n)               \
    ARM_DRIVER_CAN Driver_CAN##n =      \
                                        {                                   \
                                                                            CAN_GetVersion,                 \
                                                                            CAN_GetCapabilities,            \
                                                                            CAN##n##_Initialize,            \
                                                                            CAN##n##_Uninitialize,          \
                                                                            CAN##n##_PowerControl,          \
                                                                            CAN##n##_GetClock,              \
                                                                            CAN##n##_SetBitrate,            \
                                                                            CAN##n##_SetMode,               \
                                                                            CAN##n##_ObjectGetCapabilities, \
                                                                            CAN##n##_ObjectSetFilter,       \
                                                                            CAN##n##_ObjectConfigure,       \
                                                                            CAN##n##_MessageSend,           \
                                                                            CAN##n##_MessageRead,           \
                                                                            CAN##n##_Control,               \
                                                                            CAN##n##_GetStatus              \
                                        };

/* Driver status */
typedef struct
{
    uint8_t                       initialized  : 1;       // Initialized status: 0 - not initialized, 1 - initialized
    uint8_t                       powered      : 1;       // Power status:       0 - not powered,     1 - powered
    uint8_t                       reserved     : 6;       // Reserved (for padding)
} DriverStatus_t;

/* Instance run-time information (RW) */
typedef struct
{
    DriverStatus_t              drv_status;
    ARM_CAN_SignalUnitEvent_t   CAN_SignalUnitEvent;
    ARM_CAN_SignalObjectEvent_t CAN_SignalObjectEvent;
    uint8_t                     can_status;
    uint8_t                     can_obj_cfg[CAN_TOTAL_OBJ_NUM];
    uint8_t                     can_fd_frame;
    CANFD_FD_MSG_T              sRxMsgFrame;
    CANFD_FD_MSG_T              sTxMsgFrame;
} RW_Info_t;

/* Instance compile-time information (RO), also contains pointer to run-time information */
typedef struct
{
    CANFD_T    *canfd;
    IRQn_Type  IRQn;
} RO_Info_t;

/* Instance information */
typedef struct
{
    const RO_Info_t  *ptr_ro_info;      // Pointer to compile-time information (RO)
    RW_Info_t  *ptr_rw_info;            // Pointer to run-time information (RW)
} CAN_Info_t;

/* Local functions prototypes */
static ARM_DRIVER_VERSION       CAN_GetVersion(void);
static ARM_CAN_CAPABILITIES     CAN_GetCapabilities(void);
static int32_t                  CANn_Initialize(uint32_t port, ARM_CAN_SignalUnitEvent_t cb_unit_event, ARM_CAN_SignalObjectEvent_t cb_object_event);
static int32_t                  CANn_Uninitialize(uint32_t port);
static int32_t                  CANn_PowerControl(uint32_t port, ARM_POWER_STATE state);
static uint32_t                 CANn_GetClock(uint32_t port);
static int32_t                  CANn_SetBitrate(uint32_t port, ARM_CAN_BITRATE_SELECT select, uint32_t bitrate, uint32_t bit_segments);
static int32_t                  CANn_SetMode(uint32_t port, ARM_CAN_MODE mode);
static ARM_CAN_OBJ_CAPABILITIES CANn_ObjectGetCapabilities(uint32_t port, uint32_t obj_idx);
static int32_t                  CANn_ObjectSetFilter(uint32_t port, uint32_t obj_idx, ARM_CAN_FILTER_OPERATION operation, uint32_t id, uint32_t arg);
static int32_t                  CANn_ObjectConfigure(uint32_t port, uint32_t obj_idx, ARM_CAN_OBJ_CONFIG obj_cfg);
static int32_t                  CANn_MessageSend(uint32_t port, uint32_t obj_idx, ARM_CAN_MSG_INFO *msg_info, const uint8_t *data, uint8_t size);
static int32_t                  CANn_MessageRead(uint32_t port, uint32_t obj_idx, ARM_CAN_MSG_INFO *msg_info, uint8_t *data, uint8_t size);
static int32_t                  CANn_Control(uint32_t port, uint32_t control, uint32_t arg);
static ARM_CAN_STATUS           CANn_GetStatus(uint32_t port);

#if (RTE_CAN0 == 1U)
static const RO_Info_t can0_ro_info = {(CANFD_T *)CANFD0, CANFD00_IRQn};
static RW_Info_t can0_rw_info;
FUNCS_DECLARE(0)
FUNCS_DEFINE(0)
CAN_DRIVER(0)
#endif

#if (RTE_CAN1 == 1U)
static const RO_Info_t can1_ro_info = {(CANFD_T *)CANFD1, CANFD10_IRQn};
static RW_Info_t can1_rw_info;
FUNCS_DECLARE(1)
FUNCS_DEFINE(1)
CAN_DRIVER(1)
#endif

static const CAN_Info_t can_info[CAN_MAX_PORTS_NUM] =
{
#if (RTE_CAN0 == 1)
    {&can0_ro_info, &can0_rw_info},
#endif

#if (RTE_CAN1 == 1)
    {&can1_ro_info, &can1_rw_info},
#endif
};

/**
  \fn          ARM_DRIVER_VERSION ARM_CAN_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION CAN_GetVersion(void)
{
    return can_driver_version;
}

/**
  \fn          ARM_CAN_CAPABILITIES ARM_CAN_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_CAN_CAPABILITIES
*/
static ARM_CAN_CAPABILITIES CAN_GetCapabilities(void)
{
    return can_driver_capabilities;
}

/**
  \fn          int32_t ARM_CAN_Initialize (ARM_CAN_SignalUnitEvent_t   cb_unit_event,
                                           ARM_CAN_SignalObjectEvent_t cb_object_event)
  \brief       Initialize CAN interface and register signal (callback) functions.
  \param[in]   cb_unit_event   Pointer to \ref ARM_CAN_SignalUnitEvent callback function
  \param[in]   cb_object_event Pointer to \ref ARM_CAN_SignalObjectEvent callback function
  \return      \ref execution_status
*/
static int32_t CANn_Initialize(uint32_t port, ARM_CAN_SignalUnitEvent_t cb_unit_event, ARM_CAN_SignalObjectEvent_t cb_object_event)
{
    if (port >= CAN_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (can_info[port].ptr_rw_info->drv_status.initialized != 0U)
    {
        return ARM_DRIVER_OK;
    }

    /* Clear run-time info */
    memset((void *)can_info[port].ptr_rw_info, 0, sizeof(RW_Info_t));

    can_info[port].ptr_rw_info->CAN_SignalUnitEvent = cb_unit_event;
    can_info[port].ptr_rw_info->CAN_SignalObjectEvent = cb_object_event;
    can_info[port].ptr_rw_info->drv_status.initialized = 1U;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_CAN_Uninitialize (void)
  \brief       De-initialize CAN interface.
  \return      \ref execution_status
*/
static int32_t CANn_Uninitialize(uint32_t port)
{
    if (port >= CAN_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /* Clear run-time info */
    memset((void *)can_info[port].ptr_rw_info, 0, sizeof(RW_Info_t));

    can_info[port].ptr_rw_info->CAN_SignalUnitEvent = NULL;
    can_info[port].ptr_rw_info->CAN_SignalObjectEvent = NULL;
    can_info[port].ptr_rw_info->drv_status.initialized = 0U;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_CAN_PowerControl (ARM_POWER_STATE state)
  \brief       Control CAN interface power.
  \param[in]   state  Power state
                 - \ref ARM_POWER_OFF :  power off: no operation possible
                 - \ref ARM_POWER_LOW :  low power mode: retain state, detect and signal wake-up events
                 - \ref ARM_POWER_FULL : power on: full operation at maximum performance
  \return      \ref execution_status
*/
static int32_t CANn_PowerControl(uint32_t port, ARM_POWER_STATE state)
{
    if (port >= CAN_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    switch (state)
    {
        case ARM_POWER_OFF:

            can_info[port].ptr_rw_info->drv_status.powered = 0U;

            /* Disable IRQ handler */
            NVIC_DisableIRQ(can_info[port].ptr_ro_info->IRQn);

            if (port == CAN0)
            {
                CLK_DisableModuleClock(CANFD0_MODULE);
            }
            else
            {
                CLK_DisableModuleClock(CANFD1_MODULE);
            }

            break;

        case ARM_POWER_FULL:

            if (can_info[port].ptr_rw_info->drv_status.initialized == 0U)
            {
                return ARM_DRIVER_ERROR;
            }

            if (can_info[port].ptr_rw_info->drv_status.powered != 0U)
            {
                return ARM_DRIVER_OK;
            }

            /* Enable IRQ handler */
            NVIC_EnableIRQ(can_info[port].ptr_ro_info->IRQn);

            if (port == CAN0)
            {
                CLK_EnableModuleClock(CANFD0_MODULE);
                SYS_ResetModule(SYS_CANFD0RST);

                /* Enable Interrupt */
                CANFD_EnableInt(CANFD0, (CANFD_IE_RF0NE_Msk | CANFD_IE_RF1NE_Msk | CANFD_IE_TCE_Msk | CANFD_IE_RF0LE_Msk | CANFD_IE_RF1LE_Msk), 0, 0, 0);
            }
            else
            {
                CLK_EnableModuleClock(CANFD1_MODULE);
                SYS_ResetModule(SYS_CANFD1RST);

                /* Enable Interrupt */
                CANFD_EnableInt(CANFD1, (CANFD_IE_RF0NE_Msk | CANFD_IE_RF1NE_Msk | CANFD_IE_TCE_Msk | CANFD_IE_RF0LE_Msk | CANFD_IE_RF1LE_Msk), 0, 0, 0);
            }

            can_info[port].ptr_rw_info->drv_status.powered = 1U;

            break;

        case ARM_POWER_LOW:

            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t ARM_CAN_GetClock (void)
  \brief       Retrieve CAN base clock frequency.
  \return      base clock frequency
*/
static uint32_t CANn_GetClock(uint32_t port)
{
    uint32_t u32CanFdClock;

    if (port >= CAN_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /* The frequency obtained according to the CANFD clock selection */
    if (port == CAN0)
    {
        if ((CLK->CANFDSEL & CLK_CANFDSEL_CANFD0SEL_Msk) == CLK_CANFDSEL_CANFD0SEL_HXT)
        {
            u32CanFdClock = CLK_GetHXTFreq();
        }
        else if ((CLK->CANFDSEL & CLK_CANFDSEL_CANFD0SEL_Msk) == CLK_CANFDSEL_CANFD0SEL_APLL0_DIV2)
        {
            u32CanFdClock = CLK_GetAPLL0ClockFreq() / 2;
        }
        else if ((CLK->CANFDSEL & CLK_CANFDSEL_CANFD0SEL_Msk) == CLK_CANFDSEL_CANFD0SEL_HCLK0)
        {
            u32CanFdClock = CLK_GetHCLK0Freq();
        }
        else if ((CLK->CANFDSEL & CLK_CANFDSEL_CANFD0SEL_Msk) == CLK_CANFDSEL_CANFD0SEL_HIRC)
        {
            u32CanFdClock = __HIRC;
        }
        else if ((CLK->CANFDSEL & CLK_CANFDSEL_CANFD0SEL_Msk) == CLK_CANFDSEL_CANFD0SEL_HIRC48M_DIV4)
        {
            u32CanFdClock = __HIRC48M / 4;
        }
        else
        {
            return ARM_DRIVER_ERROR;
        }
    }
    else
    {
        if ((CLK->CANFDSEL & CLK_CANFDSEL_CANFD1SEL_Msk) == CLK_CANFDSEL_CANFD1SEL_HXT)
        {
            u32CanFdClock = __HXT;
        }
        else if ((CLK->CANFDSEL & CLK_CANFDSEL_CANFD1SEL_Msk) == CLK_CANFDSEL_CANFD1SEL_APLL0_DIV2)
        {
            u32CanFdClock = CLK_GetAPLL0ClockFreq() / 2;
        }
        else if ((CLK->CANFDSEL & CLK_CANFDSEL_CANFD1SEL_Msk) == CLK_CANFDSEL_CANFD1SEL_HCLK0)
        {
            u32CanFdClock = CLK_GetHCLK0Freq();
        }
        else if ((CLK->CANFDSEL & CLK_CANFDSEL_CANFD1SEL_Msk) == CLK_CANFDSEL_CANFD1SEL_HIRC)
        {
            u32CanFdClock = __HIRC;
        }
        else if ((CLK->CANFDSEL & CLK_CANFDSEL_CANFD1SEL_Msk) == CLK_CANFDSEL_CANFD1SEL_HIRC48M_DIV4)
        {
            u32CanFdClock = __HIRC48M / 4;
        }
        else
        {
            return ARM_DRIVER_ERROR;
        }
    }

    return u32CanFdClock;
}

/**
  \fn          int32_t ARM_CAN_SetBitrate (ARM_CAN_BITRATE_SELECT select, uint32_t bitrate, uint32_t bit_segments)
  \brief       Set bitrate for CAN interface.
  \param[in]   select       Bitrate selection
                 - \ref ARM_CAN_BITRATE_NOMINAL : nominal (flexible data-rate arbitration) bitrate
                 - \ref ARM_CAN_BITRATE_FD_DATA : flexible data-rate data bitrate
  \param[in]   bitrate      Bitrate
  \param[in]   bit_segments Bit segments settings
                 - \ref ARM_CAN_BIT_PROP_SEG(x) :   number of time quanta for propagation time segment
                 - \ref ARM_CAN_BIT_PHASE_SEG1(x) : number of time quanta for phase buffer segment 1
                 - \ref ARM_CAN_BIT_PHASE_SEG2(x) : number of time quanta for phase buffer Segment 2
                 - \ref ARM_CAN_BIT_SJW(x) :        number of time quanta for (re-)synchronization jump width
  \return      \ref execution_status
*/
static int32_t CANn_SetBitrate(uint32_t port, ARM_CAN_BITRATE_SELECT select, uint32_t bitrate, uint32_t bit_segments)
{
    uint32_t sjw, prop_seg, phase_seg1, phase_seg2, can_clk, prescaler, tq_num;

    if (port >= CAN_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (can_info[port].ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Parse bit segments */
    prop_seg   = (bit_segments & ARM_CAN_BIT_PROP_SEG_Msk) >> ARM_CAN_BIT_PROP_SEG_Pos;
    phase_seg1 = (bit_segments & ARM_CAN_BIT_PHASE_SEG1_Msk) >> ARM_CAN_BIT_PHASE_SEG1_Pos;
    phase_seg2 = (bit_segments & ARM_CAN_BIT_PHASE_SEG2_Msk) >> ARM_CAN_BIT_PHASE_SEG2_Pos;
    sjw        = (bit_segments & ARM_CAN_BIT_SJW_Msk) >> ARM_CAN_BIT_SJW_Pos;

    if (((prop_seg + phase_seg1) < 1U) || ((prop_seg + phase_seg1) > 16U))
    {
        return ARM_CAN_INVALID_BIT_PROP_SEG;
    }

    if ((phase_seg2 < 1U) || (phase_seg2 > 8U))
    {
        return ARM_CAN_INVALID_BIT_PHASE_SEG2;
    }

    if ((sjw < 1U) || (sjw > 4U))
    {
        return ARM_CAN_INVALID_BIT_SJW;
    }

    tq_num = 1U + prop_seg + phase_seg1 + phase_seg2;

    can_clk = CANn_GetClock(port);

    prescaler = can_clk / (bitrate * tq_num);

    if (prescaler == 0 || prescaler > 1024)
    {
        return ARM_CAN_INVALID_BITRATE;
    }

    if (can_clk > (prescaler * tq_num * bitrate))
    {
        if ((((can_clk - (prescaler * tq_num * bitrate)) * 1024U) / can_clk) > CAN_CLOCK_TOLERANCE)
        {
            return ARM_CAN_INVALID_BITRATE;
        }
    }
    else if (can_clk < (prescaler * tq_num * bitrate))
    {
        if (((((prescaler * tq_num * bitrate) - can_clk) * 1024U) / can_clk) > CAN_CLOCK_TOLERANCE)
        {
            return ARM_CAN_INVALID_BITRATE;
        }
    }

    /* Configuration change enable */
    CANFD_RunToNormal(can_info[port].ptr_ro_info->canfd, FALSE);

    if (select == ARM_CAN_BITRATE_NOMINAL)
    {
        /* Nominal bit rate */
        can_info[port].ptr_ro_info->canfd->NBTP = (((sjw & 0x7F) - 1) << 25) +
                                                  (((prescaler & 0x1FF) - 1) << 16) +
                                                  ((((phase_seg1 + prop_seg) & 0xFF) - 1) << 8) +
                                                  (((phase_seg2 & 0x7F) - 1) << 0);
    }
    else if (select == ARM_CAN_BITRATE_FD_DATA)
    {
        /* Data bit rate */
        can_info[port].ptr_ro_info->canfd->DBTP = (((prescaler & 0x1F) - 1) << 16) +
                                                  ((((phase_seg1 + prop_seg) & 0x1F) - 1) << 8) +
                                                  (((phase_seg2 & 0xF) - 1) << 4) +
                                                  (((sjw & 0xF) - 1) << 0);
    }
    else
    {
        return ARM_CAN_INVALID_BITRATE_SELECT;
    }

    CANFD_RunToNormal(can_info[port].ptr_ro_info->canfd, TRUE);

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_CAN_SetMode (ARM_CAN_MODE mode)
  \brief       Set operating mode for CAN interface.
  \param[in]   mode   Operating mode
                 - \ref ARM_CAN_MODE_INITIALIZATION :    initialization mode
                 - \ref ARM_CAN_MODE_NORMAL :            normal operation mode
                 - \ref ARM_CAN_MODE_RESTRICTED :        restricted operation mode
                 - \ref ARM_CAN_MODE_MONITOR :           bus monitoring mode
                 - \ref ARM_CAN_MODE_LOOPBACK_INTERNAL : loopback internal mode
                 - \ref ARM_CAN_MODE_LOOPBACK_EXTERNAL : loopback external mode
  \return      \ref execution_status
*/
static int32_t CANn_SetMode(uint32_t port, ARM_CAN_MODE mode)
{
    uint32_t event;
    CANFD_FD_T sCANFD_Config;

    if (port >= CAN_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (can_info[port].ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Configuratoin change enable */
    CANFD_RunToNormal(can_info[port].ptr_ro_info->canfd, FALSE);

    /* Clear mode configuration */
    can_info[port].ptr_ro_info->canfd->CCCR = (can_info[port].ptr_ro_info->canfd->CCCR & ~(CANFD_CCCR_ASM_Msk
                                                                                           | CANFD_CCCR_MON_Msk | CANFD_CCCR_TEST_Msk));
    can_info[port].ptr_ro_info->canfd->TEST = can_info[port].ptr_ro_info->canfd->TEST & ~CANFD_TEST_LBCK_Msk;

    switch (mode)
    {
        case ARM_CAN_MODE_INITIALIZATION:

            can_info[port].ptr_rw_info->can_status = ARM_CAN_UNIT_STATE_INACTIVE;
            event = ARM_CAN_EVENT_UNIT_INACTIVE;

            break;

        case ARM_CAN_MODE_NORMAL:

            can_info[port].ptr_rw_info->can_status = ARM_CAN_UNIT_STATE_ACTIVE;
            event = ARM_CAN_EVENT_UNIT_ACTIVE;

            break;

        case ARM_CAN_MODE_RESTRICTED:

            /* Restricted Operation Mode */
            can_info[port].ptr_ro_info->canfd->CCCR |= CANFD_CCCR_ASM_Msk;

            can_info[port].ptr_rw_info->can_status = ARM_CAN_UNIT_STATE_PASSIVE;
            event = ARM_CAN_EVENT_UNIT_PASSIVE;

            break;

        case ARM_CAN_MODE_MONITOR:

            /* Bus Monitoring Mode */
            can_info[port].ptr_ro_info->canfd->CCCR |= CANFD_CCCR_MON_Msk;

            can_info[port].ptr_rw_info->can_status = ARM_CAN_UNIT_STATE_PASSIVE;
            event = ARM_CAN_EVENT_UNIT_PASSIVE;

            break;

        case ARM_CAN_MODE_LOOPBACK_INTERNAL:

            /* Internal loopback setting */
            can_info[port].ptr_ro_info->canfd->CCCR |= CANFD_CCCR_TEST_Msk;
            can_info[port].ptr_ro_info->canfd->TEST |= CANFD_TEST_LBCK_Msk;
            can_info[port].ptr_ro_info->canfd->CCCR |= CANFD_CCCR_MON_Msk;

            can_info[port].ptr_rw_info->can_status = ARM_CAN_UNIT_STATE_ACTIVE;
            event = ARM_CAN_EVENT_UNIT_ACTIVE;

            break;

        case ARM_CAN_MODE_LOOPBACK_EXTERNAL:

            can_info[port].ptr_ro_info->canfd->CCCR |= CANFD_CCCR_TEST_Msk;
            can_info[port].ptr_ro_info->canfd->TEST |= CANFD_TEST_LBCK_Msk;

            can_info[port].ptr_rw_info->can_status = ARM_CAN_UNIT_STATE_ACTIVE;
            event = ARM_CAN_EVENT_UNIT_ACTIVE;

            break;
    }

    switch (mode)
    {
        case ARM_CAN_MODE_INITIALIZATION:
            break;

        case ARM_CAN_MODE_NORMAL:
        case ARM_CAN_MODE_RESTRICTED:
        case ARM_CAN_MODE_MONITOR:
        case ARM_CAN_MODE_LOOPBACK_INTERNAL:
        case ARM_CAN_MODE_LOOPBACK_EXTERNAL:

            CANFD_RunToNormal(can_info[port].ptr_ro_info->canfd, FALSE);

            /* Use defined configuration */
            sCANFD_Config.sElemSize.u32UserDef = 0;

            /* Get the CAN FD configuration value */
            CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_FD_MODE);

            /* Config Message RAM */
            CAN_SetConfig(can_info[port].ptr_ro_info->canfd, &sCANFD_Config);

            CANFD_RunToNormal(can_info[port].ptr_ro_info->canfd, TRUE);

            break;
    }

    if ((can_info[port].ptr_rw_info->CAN_SignalUnitEvent != NULL) && (event != 0))
    {
        can_info[port].ptr_rw_info->CAN_SignalUnitEvent(event);
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          ARM_CAN_OBJ_CAPABILITIES ARM_CAN_ObjectGetCapabilities (uint32_t obj_idx)
  \brief       Retrieve capabilities of an object.
  \param[in]   obj_idx  Object index
  \return      \ref ARM_CAN_OBJ_CAPABILITIES
*/
static ARM_CAN_OBJ_CAPABILITIES CANn_ObjectGetCapabilities(uint32_t port, uint32_t obj_idx)
{
    (void)port;
    ARM_CAN_OBJ_CAPABILITIES obj_cap_null;

    if (obj_idx >= CAN_TOTAL_OBJ_NUM)
    {
        memset((void *)&obj_cap_null, 0, sizeof(ARM_CAN_OBJ_CAPABILITIES));
        return obj_cap_null;
    }

    /* Return object capabilities */
    if (obj_idx >= CAN_RX_OBJ_NUM)
    {
        return can_object_capabilities_tx;
    }
    else
    {
        return can_object_capabilities_rx;
    }
}

/**
  \fn          int32_t ARM_CAN_ObjectSetFilter (uint32_t obj_idx, ARM_CAN_FILTER_OPERATION operation, uint32_t id, uint32_t arg)
  \brief       Add or remove filter for message reception.
  \param[in]   obj_idx      Object index of object that filter should be or is assigned to
  \param[in]   operation    Operation on filter
                 - \ref ARM_CAN_FILTER_ID_EXACT_ADD :       add    exact id filter
                 - \ref ARM_CAN_FILTER_ID_EXACT_REMOVE :    remove exact id filter
                 - \ref ARM_CAN_FILTER_ID_RANGE_ADD :       add    range id filter
                 - \ref ARM_CAN_FILTER_ID_RANGE_REMOVE :    remove range id filter
                 - \ref ARM_CAN_FILTER_ID_MASKABLE_ADD :    add    maskable id filter
                 - \ref ARM_CAN_FILTER_ID_MASKABLE_REMOVE : remove maskable id filter
  \param[in]   id           ID or start of ID range (depending on filter type)
  \param[in]   arg          Mask or end of ID range (depending on filter type)
  \return      \ref execution_status
*/
static int32_t CANn_ObjectSetFilter(uint32_t port, uint32_t obj_idx, ARM_CAN_FILTER_OPERATION operation, uint32_t id, uint32_t arg)
{
    if (port >= CAN_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (can_info[port].ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    switch (operation)
    {
        case ARM_CAN_FILTER_ID_EXACT_ADD:

            if (id & ARM_CAN_ID_IDE_Msk)
            {
                /* Add extended exact id filter */
                CANFD_SetXIDFltr(can_info[port].ptr_ro_info->canfd, obj_idx, CANFD_RX_FIFO0_EXT_EXACT_LOW(id), CANFD_RX_FIFO1_EXT_EXACT_HIGH(arg));
            }
            else
            {
                /* Add standard exact id filter */
                CANFD_SetSIDFltr(can_info[port].ptr_ro_info->canfd, obj_idx, CANFD_RX_FIFO0_STD_EXACT(id, arg));
            }

            break;

        case ARM_CAN_FILTER_ID_MASKABLE_ADD:

            if (id & ARM_CAN_ID_IDE_Msk)
            {
                /* Add extended maskable id filter */
                CANFD_SetXIDFltr(can_info[port].ptr_ro_info->canfd, obj_idx, CANFD_RX_FIFO0_EXT_MASKABLE_LOW(id), CANFD_RX_FIFO1_EXT_MASKABLE_HIGH(arg));
            }
            else
            {
                /* Add standard maskable id filter */
                CANFD_SetSIDFltr(can_info[port].ptr_ro_info->canfd, obj_idx, CANFD_RX_FIFO0_STD_MASKABLE(id, arg));
            }

            break;

        case ARM_CAN_FILTER_ID_RANGE_ADD:

            if (id & ARM_CAN_ID_IDE_Msk)
            {
                /* Add extended range id filter */
                CANFD_SetXIDFltr(can_info[port].ptr_ro_info->canfd, obj_idx, CANFD_RX_FIFO0_EXT_RANGE_LOW(id), CANFD_RX_FIFO1_EXT_RANGE_HIGH(arg));
            }
            else
            {
                /* Add standard range id filter */
                CANFD_SetSIDFltr(can_info[port].ptr_ro_info->canfd, obj_idx, CANFD_RX_FIFO0_STD_RANGE(id, arg));
            }

            break;

        case ARM_CAN_FILTER_ID_EXACT_REMOVE:
        case ARM_CAN_FILTER_ID_MASKABLE_REMOVE:
        case ARM_CAN_FILTER_ID_RANGE_REMOVE:

            if (id & ARM_CAN_ID_IDE_Msk)
            {
                /* Remove extended id filter */
                CANFD_SetXIDFltr(can_info[port].ptr_ro_info->canfd, obj_idx, CANFD_RX_EXT_REMOVE_LOW, CANFD_RX_EXT_REMOVE_HIGH);
            }
            else
            {
                /* Remove standard id filter */
                CANFD_SetSIDFltr(can_info[port].ptr_ro_info->canfd, obj_idx, CANFD_RX_STD_REMOVE);
            }

            break;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_CAN_ObjectConfigure (uint32_t obj_idx, ARM_CAN_OBJ_CONFIG obj_cfg)
  \brief       Configure object.
  \param[in]   obj_idx  Object index
  \param[in]   obj_cfg  Object configuration state
                 - \ref ARM_CAN_OBJ_INACTIVE :       deactivate object
                 - \ref ARM_CAN_OBJ_RX :             configure object for reception
                 - \ref ARM_CAN_OBJ_TX :             configure object for transmission
                 - \ref ARM_CAN_OBJ_RX_RTR_TX_DATA : configure object that on RTR reception automatically transmits Data Frame
                 - \ref ARM_CAN_OBJ_TX_RTR_RX_DATA : configure object that transmits RTR and automatically receives Data Frame
  \return      \ref execution_status
*/
static int32_t CANn_ObjectConfigure(uint32_t port, uint32_t obj_idx, ARM_CAN_OBJ_CONFIG obj_cfg)
{
    if (port >= CAN_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (can_info[port].ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    if (obj_idx >= CAN_TOTAL_OBJ_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    switch (obj_cfg)
    {
        case ARM_CAN_OBJ_INACTIVE:

            can_info[port].ptr_rw_info->can_obj_cfg[obj_idx] = ARM_CAN_OBJ_INACTIVE;

            break;

        case ARM_CAN_OBJ_RX_RTR_TX_DATA:

            if (obj_idx < CAN_RX_OBJ_NUM)
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }

            can_info[port].ptr_rw_info->can_obj_cfg[obj_idx] = ARM_CAN_OBJ_RX_RTR_TX_DATA;

            break;

        case ARM_CAN_OBJ_TX_RTR_RX_DATA:

            if (obj_idx >= CAN_RX_OBJ_NUM)
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }

            can_info[port].ptr_rw_info->can_obj_cfg[obj_idx] = ARM_CAN_OBJ_TX_RTR_RX_DATA;

            break;

        case ARM_CAN_OBJ_TX:

            if (obj_idx < CAN_RX_OBJ_NUM)
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }

            can_info[port].ptr_rw_info->can_obj_cfg[obj_idx] = ARM_CAN_OBJ_TX;

            break;

        case ARM_CAN_OBJ_RX:

            if (obj_idx >= CAN_RX_OBJ_NUM)
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }

            can_info[port].ptr_rw_info->can_obj_cfg[obj_idx] = ARM_CAN_OBJ_RX;

            break;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_CAN_MessageSend (uint32_t obj_idx, ARM_CAN_MSG_INFO *msg_info, const uint8_t *data, uint8_t size)
  \brief       Send message on CAN bus.
  \param[in]   obj_idx  Object index
  \param[in]   msg_info Pointer to CAN message information
  \param[in]   data     Pointer to data buffer
  \param[in]   size     Number of data bytes to send
  \return      value >= 0  number of data bytes accepted to send
  \return      value < 0   \ref execution_status
*/
static int32_t CANn_MessageSend(uint32_t port, uint32_t obj_idx, ARM_CAN_MSG_INFO *msg_info, const uint8_t *data, uint8_t size)
{
    CANFD_FD_MSG_T *psTxMsg = &(can_info[port].ptr_rw_info->sTxMsgFrame);
    uint8_t u8Cnt;

    if (port >= CAN_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (can_info[port].ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    if (can_info[port].ptr_rw_info->can_obj_cfg[obj_idx] != ARM_CAN_OBJ_TX)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Get Tx message from msg_info */
    psTxMsg->u32Id = msg_info->id;
    psTxMsg->eIdType = (msg_info->id) >> 31;
    psTxMsg->bBitRateSwitch = msg_info->brs;
    psTxMsg->bErrStaInd = msg_info->esi;
    psTxMsg->eFrmType = msg_info->rtr;

    /* Set FD frame format attribute */
    if (can_info[port].ptr_rw_info->can_fd_frame)
    {
        psTxMsg->bFDFormat = 1;

        /* Extended data length */
        if (msg_info->edl)
        {
            if (msg_info->dlc <= 8)      psTxMsg->u32DLC = msg_info->dlc;
            else if (msg_info->dlc == 9) psTxMsg->u32DLC = 12;
            else if (msg_info->dlc == 10) psTxMsg->u32DLC = 16;
            else if (msg_info->dlc == 11) psTxMsg->u32DLC = 20;
            else if (msg_info->dlc == 12) psTxMsg->u32DLC = 24;
            else if (msg_info->dlc == 13) psTxMsg->u32DLC = 32;
            else if (msg_info->dlc == 14) psTxMsg->u32DLC = 48;
            else if (msg_info->dlc == 15) psTxMsg->u32DLC = 64;
        }
        else
        {
            psTxMsg->u32DLC = msg_info->dlc;
        }
    }
    else
    {
        psTxMsg->bFDFormat = 0;
        psTxMsg->u32DLC = msg_info->dlc;
    }

    if (size <= 8)      psTxMsg->u32DLC = size;
    else if (size > 48) psTxMsg->u32DLC = 64;
    else if (size > 32) psTxMsg->u32DLC = 48;
    else if (size > 24) psTxMsg->u32DLC = 32;
    else if (size > 20) psTxMsg->u32DLC = 24;
    else if (size > 16) psTxMsg->u32DLC = 20;
    else if (size > 12) psTxMsg->u32DLC = 16;
    else if (size > 8) psTxMsg->u32DLC = 12;

    for (u8Cnt = 0; u8Cnt < size; u8Cnt++)
    {
        psTxMsg->au8Data[u8Cnt] = data[u8Cnt];
    }

    /* Copy Tx message to TX buffer and request transmission */
    if (CANFD_TransmitTxMsg(can_info[port].ptr_ro_info->canfd, 0, psTxMsg) != eCANFD_TRANSMIT_SUCCESS)
    {
        return ARM_DRIVER_ERROR;
    }

    return ((int32_t)size);
}

/**
  \fn          int32_t ARM_CAN_MessageRead (uint32_t obj_idx, ARM_CAN_MSG_INFO *msg_info, uint8_t *data, uint8_t size)
  \brief       Read message received on CAN bus.
  \param[in]   obj_idx  Object index
  \param[out]  msg_info Pointer to read CAN message information
  \param[out]  data     Pointer to data buffer for read data
  \param[in]   size     Maximum number of data bytes to read
  \return      value >= 0  number of data bytes read
  \return      value < 0   \ref execution_status
*/
static int32_t CANn_MessageRead(uint32_t port, uint32_t obj_idx, ARM_CAN_MSG_INFO *msg_info, uint8_t *data, uint8_t size)
{
    (void)msg_info;

    if (port >= CAN_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (can_info[port].ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    if (can_info[port].ptr_rw_info->can_obj_cfg[obj_idx] != ARM_CAN_OBJ_RX)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Copy the data from Rx message to the data pointer */
    memcpy(&data[0], &(can_info[port].ptr_rw_info->sRxMsgFrame.au8Data[0]), size);

    return ((int32_t)size);
}

/**
  \fn          int32_t ARM_CAN_Control (uint32_t control, uint32_t arg)
  \brief       Control CAN interface.
  \param[in]   control  Operation
                 - \ref ARM_CAN_SET_FD_MODE :            set FD operation mode
                 - \ref ARM_CAN_ABORT_MESSAGE_SEND :     abort sending of CAN message
                 - \ref ARM_CAN_CONTROL_RETRANSMISSION : enable/disable automatic retransmission
                 - \ref ARM_CAN_SET_TRANSCEIVER_DELAY :  set transceiver delay
  \param[in]   arg      Argument of operation
  \return      \ref execution_status
*/
static int32_t CANn_Control(uint32_t port, uint32_t control, uint32_t arg)
{
    if (port >= CAN_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (can_info[port].ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    switch (control & ARM_CAN_CONTROL_Msk)
    {
        case ARM_CAN_ABORT_MESSAGE_SEND:

            if ((arg < CAN_RX_OBJ_NUM) || (arg >= CAN_TOTAL_OBJ_NUM))
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }

            arg -= CAN_RX_OBJ_NUM;

            /* Tx buffer cancellation request */
            can_info[port].ptr_ro_info->canfd->TXBCR |= 1 << arg;

            break;

        case ARM_CAN_CONTROL_RETRANSMISSION:

            /* Configuration change enable */
            CANFD_RunToNormal(can_info[port].ptr_ro_info->canfd, FALSE);

            if (arg != ((can_info[port].ptr_ro_info->canfd->CCCR & CANFD_CCCR_DAR_Msk) >> CANFD_CCCR_DAR_Pos))
            {
                can_info[port].ptr_ro_info->canfd->CCCR = (can_info[port].ptr_ro_info->canfd->CCCR & ~CANFD_CCCR_DAR_Msk) | (arg << CANFD_CCCR_DAR_Pos);
            }

            CANFD_RunToNormal(can_info[port].ptr_ro_info->canfd, TRUE);

            break;

        case ARM_CAN_SET_FD_MODE:

            if (arg == 1)
            {
                can_info[port].ptr_rw_info->can_fd_frame = 1;
            }
            else
            {
                can_info[port].ptr_rw_info->can_fd_frame = 0;
            }

            break;

        case ARM_CAN_SET_TRANSCEIVER_DELAY:

            /* Configuration change enable */
            CANFD_RunToNormal(can_info[port].ptr_ro_info->canfd, FALSE);

            /* Set transmitter delay compensation SSP offset */
            can_info[port].ptr_ro_info->canfd->TDCR = (can_info[port].ptr_ro_info->canfd->TDCR & ~CANFD_TDCR_TDCO_Msk) | (arg << CANFD_TDCR_TDCO_Pos);

            CANFD_RunToNormal(can_info[port].ptr_ro_info->canfd, TRUE);

            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          ARM_CAN_STATUS ARM_CAN_GetStatus (void)
  \brief       Get CAN status.
  \return      CAN status \ref ARM_CAN_STATUS
*/
static ARM_CAN_STATUS CANn_GetStatus(uint32_t port)
{
    ARM_CAN_STATUS can_status;
    uint32_t u32LastErrorCode;

    /* Get last error code */
    u32LastErrorCode = ((can_info[port].ptr_ro_info->canfd->PSR & CANFD_PSR_LEC_Msk) >> CANFD_PSR_LEC_Pos);

    can_status.unit_state = can_info[port].ptr_rw_info->can_status & 3U;

    switch (u32LastErrorCode)
    {
        case 0:
            can_status.last_error_code = ARM_CAN_LEC_NO_ERROR;
            break;

        case 1:
            can_status.last_error_code = ARM_CAN_LEC_STUFF_ERROR;
            break;

        case 2:
            can_status.last_error_code = ARM_CAN_LEC_FORM_ERROR;
            break;

        case 3:
            can_status.last_error_code = ARM_CAN_LEC_ACK_ERROR;
            break;

        case 4:
        case 5:
            can_status.last_error_code = ARM_CAN_LEC_BIT_ERROR;
            break;

        case 6:
            can_status.last_error_code = ARM_CAN_LEC_CRC_ERROR;
            break;

        case 7:
            /* No change */
            break;
    }

    /* Get transmit error count */
    can_status.tx_error_count  = (uint8_t)((can_info[port].ptr_ro_info->canfd->ECR & CANFD_ECR_TEC_Msk) >> CANFD_ECR_TEC_Pos);

    /* Get receive error count */
    can_status.rx_error_count  = (uint8_t)((can_info[port].ptr_ro_info->canfd->ECR & CANFD_ECR_REC_Msk) >> CANFD_ECR_REC_Pos);

    return can_status;
}

void CANn_IRQHandler(CANFD_T *psCanfd, uint32_t port)
{
    if (can_info[port].ptr_ro_info->canfd->IR & CANFD_IR_RF0N_Msk)
    {
        /* Clear the Interrupt flag */
        CANFD_ClearStatusFlag(psCanfd, CANFD_IR_RF0N_Msk);

        /* Receive the Rx Fifo0 buffer */
        CANFD_ReadRxFifoMsg(psCanfd, 0, &(can_info[port].ptr_rw_info->sRxMsgFrame));

        if (can_info[port].ptr_rw_info->CAN_SignalObjectEvent != NULL)
        {
            can_info[port].ptr_rw_info->CAN_SignalObjectEvent(0U, ARM_CAN_EVENT_RECEIVE);
        }
    }

    if (can_info[port].ptr_ro_info->canfd->IR & CANFD_IR_RF1N_Msk)
    {
        /* Clear the Interrupt flag */
        CANFD_ClearStatusFlag(psCanfd, CANFD_IR_RF1N_Msk);

        /* Receive the Rx Fifo1 buffer */
        CANFD_ReadRxFifoMsg(psCanfd, 1, &(can_info[port].ptr_rw_info->sRxMsgFrame));

        if (can_info[port].ptr_rw_info->CAN_SignalObjectEvent != NULL)
        {
            can_info[port].ptr_rw_info->CAN_SignalObjectEvent(1U, ARM_CAN_EVENT_RECEIVE);
        }
    }

    if (can_info[port].ptr_ro_info->canfd->IR & CANFD_IR_RF0L_Msk)
    {
        /* Clear the Interrupt flag */
        CANFD_ClearStatusFlag(psCanfd, CANFD_IR_RF0L_Msk);

        if (can_info[port].ptr_rw_info->CAN_SignalObjectEvent != NULL)
        {
            can_info[port].ptr_rw_info->CAN_SignalObjectEvent(0U, ARM_CAN_EVENT_RECEIVE_OVERRUN);
        }
    }

    if (can_info[port].ptr_ro_info->canfd->IR & CANFD_IR_RF1L_Msk)
    {
        /* Clear the Interrupt flag */
        CANFD_ClearStatusFlag(psCanfd, CANFD_IR_RF1L_Msk);

        if (can_info[port].ptr_rw_info->CAN_SignalObjectEvent != NULL)
        {
            can_info[port].ptr_rw_info->CAN_SignalObjectEvent(1U, ARM_CAN_EVENT_RECEIVE_OVERRUN);
        }
    }

    if (can_info[port].ptr_ro_info->canfd->IR & CANFD_IR_TC_Msk)
    {
        /* Clear the Interrupt flag */
        CANFD_ClearStatusFlag(psCanfd, CANFD_IR_TC_Msk);

        if (can_info[port].ptr_rw_info->CAN_SignalObjectEvent != NULL)
        {
            can_info[port].ptr_rw_info->CAN_SignalObjectEvent(0U, ARM_CAN_EVENT_SEND_COMPLETE);
        }
    }

    if (((can_info[port].ptr_ro_info->canfd->PSR & CANFD_PSR_BO_Msk) != 0) && ((can_info[port].ptr_ro_info->canfd->IR & CANFD_IR_BO_Msk) != 0))
    {
        /* Clear the Interrupt flag */
        CANFD_ClearStatusFlag(psCanfd, CANFD_IR_BO_Msk);

        can_info[port].ptr_rw_info->CAN_SignalUnitEvent(ARM_CAN_EVENT_UNIT_BUS_OFF);
    }

    if (((can_info[port].ptr_ro_info->canfd->PSR & CANFD_PSR_EW_Msk) != 0) && ((can_info[port].ptr_ro_info->canfd->IR & CANFD_IR_EW_Msk) != 0))
    {
        /* Clear the Interrupt flag */
        CANFD_ClearStatusFlag(psCanfd, CANFD_IR_EW_Msk);

        can_info[port].ptr_rw_info->CAN_SignalUnitEvent(ARM_CAN_EVENT_UNIT_WARNING);
    }

    if (((can_info[port].ptr_ro_info->canfd->PSR & CANFD_PSR_EP_Msk) != 0) && ((can_info[port].ptr_ro_info->canfd->IR & CANFD_IR_EP_Msk) != 0))
    {
        /* Clear the Interrupt flag */
        CANFD_ClearStatusFlag(psCanfd, CANFD_IR_EP_Msk);

        can_info[port].ptr_rw_info->CAN_SignalUnitEvent(ARM_CAN_EVENT_UNIT_PASSIVE);
    }
}

void CANFD00_IRQHandler(void)
{
    CANn_IRQHandler(CANFD0, CAN0);
}

void CANFD10_IRQHandler(void)
{
    CANn_IRQHandler(CANFD1, CAN1);
}

void CAN_SetConfig(CANFD_T *psCanfd, CANFD_FD_T *psCanfdStr)
{
    if (psCanfdStr->sBtConfig.bBitRateSwitch)
    {
        /* enable FD and baud-rate switching */
        psCanfd->CCCR |= CANFD_CCCR_BRSE_Msk;
    }

    if (psCanfdStr->sBtConfig.bFDEn)
    {
        /*FD Operation enabled*/
        psCanfd->CCCR |= CANFD_CCCR_FDOE_Msk;
    }

    /*Clear the Rx Fifo0 element setting */
    psCanfd->RXF0C = 0;
    /*Clear the Rx Fifo1 element setting */
    psCanfd->RXF1C = 0;

    /* Configures the Standard ID Filter element */
    if (psCanfdStr->sElemSize.u32SIDFC != 0)
        CANFD_ConfigSIDFC(psCanfd, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize);

    /*Configures the Extended ID Filter element */
    if (psCanfdStr->sElemSize.u32XIDFC != 0)
        CANFD_ConfigXIDFC(psCanfd, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize);

    /*Configures the Tx Buffer element */
    if (psCanfdStr->sElemSize.u32TxBuf != 0 || psCanfdStr->sElemSize.u32TxFifoQueue != 0)
        CANFD_InitTxDBuf(psCanfd, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize, eCANFD_BYTE64);

    /*Configures the Rx Buffer element */
    if (psCanfdStr->sElemSize.u32RxBuf != 0)
        CANFD_InitRxDBuf(psCanfd, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize, eCANFD_BYTE64);

    /*Configures the Rx Fifo0 element */
    if (psCanfdStr->sElemSize.u32RxFifo0 != 0)
        CANFD_InitRxFifo(psCanfd, 0, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize, 0, eCANFD_BYTE64);

    /*Configures the Rx Fifo1 element */
    if (psCanfdStr->sElemSize.u32RxFifo1 != 0)
        CANFD_InitRxFifo(psCanfd, 1, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize, 0, eCANFD_BYTE64);

    /*Configures the Tx Event FIFO element */
    if (psCanfdStr->sElemSize.u32TxEventFifo != 0)
        CANFD_InitTxEvntFifo(psCanfd, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize, 0);

    /*Reject all Non-matching Frames Extended ID and Frames Standard ID,Reject all remote frames with 11-bit standard IDs and 29-bit extended IDs */
    CANFD_SetGFC(psCanfd, eCANFD_REJ_NON_MATCH_FRM, eCANFD_REJ_NON_MATCH_FRM, 1, 1);

    if (psCanfdStr->sBtConfig.bEnableLoopBack)
    {
        psCanfd->CCCR |= CANFD_CCCR_TEST_Msk;
        psCanfd->TEST |= CANFD_TEST_LBCK_Msk;
    }
}
