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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Driver_USART.h"
#include "NuMicro.h"

#include "drv_pdma.h"
#include "misc.h"

// Configuration depending on RTE_USART.h
// Check if at least one peripheral instance is configured in RTE_Device_USART.h
#if (RTE_USART13 == 1)

// Macros
#define ARM_USART_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)  /* driver version */
// Macro for porting compatibility
#define USART_HandleTypeDef  LPUART_T
// Macro for section for RW info
#ifdef  USART_SECTION_NAME
    #define USARTn_SECTION_(name,n) __attribute__((section(name #n)))
    #define USARTn_SECTION(n)       USARTn_SECTION_(USART_SECTION_NAME,n)
#else
    #define USARTn_SECTION(n)
#endif

// Macro to create usart_ro_info (for U(S)ART instances)
#define RO_INFO_DEFINE(n,huart,irq,pdma_rx,pdma_tx) \
    static        RO_Info_t         usart##n##_ro_info = {  \
                                                            huart,                             \
                                                            irq,                               \
                                                            RTE_USART##n##_RX_LPPDMA,          \
                                                            RTE_USART##n##_TX_LPPDMA,          \
                                                            RTE_USART##n##_RX_LPPDMA_NUMBER,   \
                                                            RTE_USART##n##_TX_LPPDMA_NUMBER,   \
                                                            RTE_USART##n##_RX_LPPDMA_CHANNEL,  \
                                                            RTE_USART##n##_TX_LPPDMA_CHANNEL,  \
                                                            pdma_rx,                           \
                                                            pdma_tx                            \
                                                         };
// Macro to create usart_rw_info (for U(S)ART instances)
#define RW_INFO_DEFINE(n)                                                      \
    static        RW_Info_t         usart##n##_rw_info USARTn_SECTION(n) =  {  \
                                                                               .lppdma_rx_chan_id = -1,  \
                                                                               .lppdma_tx_chan_id = -1,  \
                                                                            };                            \
    static  const USART_Info_t      usart##n##_info = {  \
                                                         &usart##n##_ro_info,  \
                                                         &usart##n##_rw_info,  \
                                                      };


// Macro for declaring functions (for instances)
#define FUNCS_DECLARE(n)                                                                                            \
    static  ARM_USART_CAPABILITIES  USART##n##_GetCapabilities (void);                                              \
    static  int32_t                 USART##n##_Initialize      (ARM_USART_SignalEvent_t cb_event);                  \
    static  int32_t                 USART##n##_Uninitialize    (void);                                              \
    static  int32_t                 USART##n##_PowerControl    (ARM_POWER_STATE state);                             \
    static  int32_t                 USART##n##_Send            (const void *data, uint32_t num);                    \
    static  int32_t                 USART##n##_Receive         (void *data, uint32_t num);                          \
    static  int32_t                 USART##n##_Transfer        (const void *data_out, void *data_in, uint32_t num); \
    static  uint32_t                USART##n##_GetTxCount      (void);                                              \
    static  uint32_t                USART##n##_GetRxCount      (void);                                              \
    static  int32_t                 USART##n##_Control         (uint32_t control, uint32_t arg);                    \
    static  ARM_USART_STATUS        USART##n##_GetStatus       (void);

// Macro for defining functions (for instances)
#define FUNCS_DEFINE(n)                                                                                                                                                                           \
    static  ARM_USART_CAPABILITIES  USART##n##_GetCapabilities (void)                                              { return USARTn_GetCapabilities (&usart##n##_info); }                          \
    static  int32_t                 USART##n##_Initialize      (ARM_USART_SignalEvent_t cb_event)                  { return USARTn_Initialize      (&usart##n##_info, cb_event); }                \
    static  int32_t                 USART##n##_Uninitialize    (void)                                              { return USARTn_Uninitialize    (&usart##n##_info); }                          \
    static  int32_t                 USART##n##_PowerControl    (ARM_POWER_STATE state)                             { return USARTn_PowerControl    (&usart##n##_info, state); }                   \
    static  int32_t                 USART##n##_Send            (const void *data, uint32_t num)                    { return USARTn_Send            (&usart##n##_info, data, num); }               \
    static  int32_t                 USART##n##_Receive         (void *data, uint32_t num)                          { return USARTn_Receive         (&usart##n##_info, data, num); }               \
    static  int32_t                 USART##n##_Transfer        (const void *data_out, void *data_in, uint32_t num) { return USARTn_Transfer        (&usart##n##_info, data_out, data_in, num); }  \
    static  uint32_t                USART##n##_GetTxCount      (void)                                              { return USARTn_GetTxCount      (&usart##n##_info); }                          \
    static  uint32_t                USART##n##_GetRxCount      (void)                                              { return USARTn_GetRxCount      (&usart##n##_info); }                          \
    static  int32_t                 USART##n##_Control         (uint32_t control, uint32_t arg)                    { return USARTn_Control         (&usart##n##_info, control, arg); }            \
    static  ARM_USART_STATUS        USART##n##_GetStatus       (void)                                              { return USARTn_GetStatus       (&usart##n##_info); }

// Macro for defining driver structures (for instances)
#define USART_DRIVER(n)                   \
    ARM_DRIVER_USART Driver_USART##n =    \
                                          {                                     \
                                                                                USART_GetVersion,                 \
                                                                                USART##n##_GetCapabilities,       \
                                                                                USART##n##_Initialize,            \
                                                                                USART##n##_Uninitialize,          \
                                                                                USART##n##_PowerControl,          \
                                                                                USART##n##_Send,                  \
                                                                                USART##n##_Receive,               \
                                                                                USART##n##_Transfer,              \
                                                                                USART##n##_GetTxCount,            \
                                                                                USART##n##_GetRxCount,            \
                                                                                USART##n##_Control,               \
                                                                                USART##n##_GetStatus,             \
                                                                                USART_SetModemControl,            \
                                                                                USART_GetModemStatus              \
                                          };

// Driver status
typedef struct
{
    uint8_t                       initialized  : 1;       // Initialized status: 0 - not initialized, 1 - initialized
    uint8_t                       powered      : 1;       // Power status:       0 - not powered,     1 - powered
    uint8_t                       configured   : 1;       // Configured status:  0 - not configured,  1 - configured
    uint8_t                       reserved     : 5;       // Reserved (for padding)
} DriverStatus_t;

// Rx trans
typedef struct
{
    uint8_t   *data;
    uint16_t  num_bytes;
    uint16_t  trans_count;
} RX_Trans_t;

// Tx trans
typedef struct
{
    uint8_t   *data;
    uint16_t  num_bytes;
    uint16_t  trans_count;
} TX_Trans_t;

// Instance run-time information (RW)
typedef struct
{
    ARM_USART_SignalEvent_t       cb_event;               // Event callback
    DriverStatus_t                drv_status;             // Driver status
    volatile uint8_t              rx_overflow;            // Receive data overflow detected (cleared on start of next receive operation)
    volatile uint8_t              rx_framing_error;       // Framing error detected on receive (cleared on start of next receive operation)
    volatile uint8_t              rx_parity_error;        // Parity error detected on receive (cleared on start of next receive operation)

    RX_Trans_t                    rx_trans;               // RX trans contrl
    TX_Trans_t                    tx_trans;               // TX trans contrl

    int32_t                       lppdma_rx_chan_id;      // lpPDMA RX channel ID;
    int32_t                       lppdma_tx_chan_id;      // lpPDMA TX channel ID;

} RW_Info_t;

// Instance compile-time information (RO)
// also contains pointer to run-time information
typedef struct
{
    USART_HandleTypeDef           *ptr_USART;
    int32_t                       irq_n;
    uint32_t                      lppdma_rx_used;          // use LPPDMA for RX transfer
    uint32_t                      lppdma_tx_used;          // use LPPDMA for TX transfer
    uint32_t                      lppdma_rx_num;           // LPPDMA RX number
    uint32_t                      lppdma_tx_num;           // LPPDMA TX number
    uint32_t                      lppdma_rx_channel;       // LPPDMA RX channel
    uint32_t                      lppdma_tx_channel;       // LPPDMA TX channel
    uint32_t                      lppdma_rx_perip_mode;    // LPPDMA RX peripheral transfer moode
    uint32_t                      lppdma_tx_perip_mode;    // LPPDMA TX peripheral transfer moode
} RO_Info_t;

typedef struct
{
    const RO_Info_t              *ptr_ro_info;             // Pointer to compile-time information (RO)
    RW_Info_t                    *ptr_rw_info;             // Pointer to run-time information (RW)

} USART_Info_t;


// Local functions prototypes
static const USART_Info_t       *USART_GetInfo(const USART_HandleTypeDef *huart);
static ARM_DRIVER_VERSION       USART_GetVersion(void);
static ARM_USART_CAPABILITIES   USARTn_GetCapabilities(const USART_Info_t *ptr_usart_info);
static int32_t                  USARTn_Initialize(const USART_Info_t *ptr_usart_info, ARM_USART_SignalEvent_t cb_event);
static int32_t                  USARTn_Uninitialize(const USART_Info_t *ptr_usart_info);
static int32_t                  USARTn_PowerControl(const USART_Info_t *ptr_usart_info, ARM_POWER_STATE state);
static int32_t                  USARTn_Send(const USART_Info_t *ptr_usart_info, const void *data, uint32_t num);
static int32_t                  USARTn_Receive(const USART_Info_t *ptr_usart_info,       void *data, uint32_t num);
static int32_t                  USARTn_Transfer(const USART_Info_t *ptr_usart_info, const void *data_out, void *data_in, uint32_t num);
static uint32_t                 USARTn_GetTxCount(const USART_Info_t *ptr_usart_info);
static uint32_t                 USARTn_GetRxCount(const USART_Info_t *ptr_usart_info);
static int32_t                  USARTn_Control(const USART_Info_t *ptr_usart_info, uint32_t control, uint32_t arg);
static ARM_USART_STATUS         USARTn_GetStatus(const USART_Info_t *ptr_usart_info);
static int32_t                  USART_SetModemControl(ARM_USART_MODEM_CONTROL control);
static ARM_USART_MODEM_STATUS   USART_GetModemStatus(void);

#if(RTE_USART13)
    RO_INFO_DEFINE(13, LPUART0, LPUART0_IRQn, LPPDMA_LPUART0_RX, LPPDMA_LPUART0_TX)
    RW_INFO_DEFINE(13)
    FUNCS_DECLARE(13)
    FUNCS_DEFINE(13)
    USART_DRIVER(13)
#endif

// List of available USART instance infos
static const USART_Info_t *const usart_info_list[] =
{
#if (RTE_USART13)
    &usart13_info,
#endif
    NULL
};

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion =
{
    ARM_USART_API_VERSION,
    ARM_USART_DRV_VERSION
};

// Auxiliary functions

/**
  \fn          USART_Info_t *USART_GetInfo (const USART_HandleTypeDef *huart)
  \brief       Get pointer to USART_Info_t structure corresponding to specified huart.
  \param[in]   huart    Pointer to USART_HandleTypeDef structure (USART_HandleTypeDef)
  \return      pointer to USART info structure (USART_Info_t)
*/
static const USART_Info_t *USART_GetInfo(const USART_HandleTypeDef *huart)
{
    const USART_Info_t *ptr_usart_info;
    uint8_t    i;

    ptr_usart_info = NULL;
    i           = 0U;

    for (i = 0U; i < (sizeof(usart_info_list) / sizeof(USART_Info_t *)); i++)
    {
        if (usart_info_list[i] != NULL)
        {
            if (usart_info_list[i]->ptr_ro_info->ptr_USART == huart)
            {
                ptr_usart_info = usart_info_list[i];
                break;
            }
        }
    }

    return ptr_usart_info;
}

static void LPPDMA_TX_CB_Handler(void *ptr_priv, uint32_t event)
{
    USART_Info_t *ptr_usart_info = (USART_Info_t *)ptr_priv;
    RW_Info_t *ptr_rw_info = ptr_usart_info->ptr_rw_info;
    TX_Trans_t *ptr_tx_trans = &ptr_rw_info->tx_trans;

    /* Disable UART Tx PDMA0 function */
    LPUART_DISABLE_INT(ptr_usart_info->ptr_ro_info->ptr_USART, LPUART_INTEN_TXPDMAEN_Msk);

    if (event & (NU_PDMA_EVENT_TRANSFER_DONE | NU_PDMA_EVENT_ABORT))
    {
        if (event & (NU_PDMA_EVENT_TRANSFER_DONE))
        {
            ptr_tx_trans->trans_count = ptr_tx_trans->num_bytes;
        }
        else
        {
            ptr_tx_trans->trans_count =  nu_lppdma_transferred_byte_get(ptr_rw_info->lppdma_tx_chan_id, ptr_tx_trans->num_bytes);
        }

        if (ptr_rw_info->cb_event)
        {
            /* Wait the STOP bit of the last byte has been transmitted. */
            LPUART_WAIT_TX_EMPTY(ptr_usart_info->ptr_ro_info->ptr_USART);
            ptr_rw_info->cb_event(ARM_USART_EVENT_TX_COMPLETE | ARM_USART_EVENT_SEND_COMPLETE);
        }
    }
}

static void LPPDMA_RX_CB_Handler(void *ptr_priv, uint32_t event)
{
    USART_Info_t *ptr_usart_info = (USART_Info_t *)ptr_priv;
    RW_Info_t *ptr_rw_info = ptr_usart_info->ptr_rw_info;
    RX_Trans_t *ptr_rx_trans = &ptr_rw_info->rx_trans;

    /* Disable UART Tx PDMA0 function */
    LPUART_DISABLE_INT(ptr_usart_info->ptr_ro_info->ptr_USART, LPUART_INTEN_RXPDMAEN_Msk);

    if (event & (NU_PDMA_EVENT_TRANSFER_DONE | NU_PDMA_EVENT_ABORT))
    {
        if (event & (NU_PDMA_EVENT_TRANSFER_DONE))
        {
            ptr_rx_trans->trans_count = ptr_rx_trans->num_bytes;
        }
        else
        {
            ptr_rx_trans->trans_count =  nu_lppdma_transferred_byte_get(ptr_rw_info->lppdma_rx_chan_id, ptr_rx_trans->num_bytes);
        }

        if (ptr_rw_info->cb_event)
        {
            ptr_rw_info->cb_event(ARM_USART_EVENT_RECEIVE_COMPLETE);
        }
    }
}


// Driver functions ************************************************************

/**
  \fn          ARM_DRIVER_VERSION USART_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION USART_GetVersion(void)
{
    return DriverVersion;
}

/**
  \fn          ARM_USART_CAPABILITIES USARTn_GetCapabilities (const USART_Info_t * const ptr_usart_info)
  \brief       Get driver capabilities.
  \param[in]   ptr_usart_info     Pointer to USART RO info structure (USART_Info_t)
  \return      \ref ARM_USART_CAPABILITIES
*/
static ARM_USART_CAPABILITIES USARTn_GetCapabilities(const USART_Info_t *ptr_usart_info)
{
    (void)ptr_usart_info;
    ARM_USART_CAPABILITIES driver_capabilities;

    // Clear capabilities structure
    memset(&driver_capabilities, 0, sizeof(ARM_USART_CAPABILITIES));

    // Load capability fields different than 0
    driver_capabilities.asynchronous = 1U;
    driver_capabilities.event_tx_complete = 1U;
    driver_capabilities.event_rx_timeout = 1U;
    driver_capabilities.flow_control_rts = 1U;
    driver_capabilities.flow_control_cts = 1U;
    driver_capabilities.rts = 1U;
    driver_capabilities.cts = 1U;
    return driver_capabilities;
}

/**
  \fn          int32_t USARTn_Initialize (const USART_Info_t * const ptr_usart_info, ARM_USART_SignalEvent_t cb_event)
  \brief       Initialize USART Interface.
  \param[in]   ptr_usart_info     Pointer to USART RO info structure (USART_Info_t)
  \param[in]   cb_event        Pointer to \ref ARM_USART_SignalEvent
  \return      \ref execution_status
*/
static int32_t USARTn_Initialize(const USART_Info_t *ptr_usart_info, ARM_USART_SignalEvent_t cb_event)
{

    RW_Info_t *ptr_rw_info = ptr_usart_info->ptr_rw_info;

    if (ptr_usart_info->ptr_ro_info->lppdma_rx_used)
    {
        if ((ptr_rw_info->lppdma_rx_chan_id >= 0) && ((uint32_t)ptr_rw_info->lppdma_rx_chan_id < PDMA_CH_MAX * PDMA_CNT))
        {
            nu_lppdma_channel_terminate(ptr_rw_info->lppdma_rx_chan_id);
            nu_lppdma_channel_free(ptr_rw_info->lppdma_rx_chan_id);
        }
    }

    if (ptr_usart_info->ptr_ro_info->lppdma_tx_used)
    {
        if ((ptr_rw_info->lppdma_tx_chan_id >= 0) && ((uint32_t)ptr_rw_info->lppdma_tx_chan_id < PDMA_CH_MAX * PDMA_CNT))
        {
            nu_lppdma_channel_terminate(ptr_rw_info->lppdma_tx_chan_id);
            nu_lppdma_channel_free(ptr_rw_info->lppdma_tx_chan_id);
        }
    }

    // Clear run-time info
    memset((void *)ptr_rw_info, 0, sizeof(RW_Info_t));

    ptr_rw_info->lppdma_rx_chan_id = -1;
    ptr_rw_info->lppdma_tx_chan_id = -1;

    // Register callback function
    ptr_rw_info->cb_event = cb_event;

    // Set driver status to initialized
    ptr_rw_info->drv_status.initialized = 1U;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USARTn_Uninitialize (const USART_Info_t *ptr_usart_info)
  \brief       De-initialize USART Interface.
  \param[in]   ptr_usart_info     Pointer to USART info structure (USART_Info_t)
  \return      \ref execution_status
*/
static int32_t USARTn_Uninitialize(const USART_Info_t *ptr_usart_info)
{

    RW_Info_t *ptr_rw_info = ptr_usart_info->ptr_rw_info;

    if (ptr_rw_info->drv_status.powered != 0U)
    {
        // If peripheral is powered, power off the peripheral
        (void)USARTn_PowerControl(ptr_usart_info, ARM_POWER_OFF);
    }

    // Clear run-time info
    memset((void *)ptr_rw_info, 0, sizeof(RW_Info_t));

    ptr_rw_info->lppdma_rx_chan_id = -1;
    ptr_rw_info->lppdma_tx_chan_id = -1;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USARTn_PowerControl (const USART_Info_t *ptr_usart_info, ARM_POWER_STATE state)
  \brief       Control USART Interface Power.
  \param[in]   ptr_usart_info     Pointer to USART info structure (USART_Info_t)
  \param[in]   state           Power state
  \return      \ref execution_status
*/
static int32_t USARTn_PowerControl(const USART_Info_t *ptr_usart_info, ARM_POWER_STATE state)
{

    RW_Info_t *ptr_rw_info = ptr_usart_info->ptr_rw_info;
    const RO_Info_t *ptr_ro_info = ptr_usart_info->ptr_ro_info;

    switch (state)
    {
        case ARM_POWER_FULL:
            if (ptr_rw_info->drv_status.initialized == 0U)
            {
                return ARM_DRIVER_ERROR;
            }

            // Clear communication error status
            ptr_rw_info->rx_overflow      = 0U;
            ptr_rw_info->rx_framing_error = 0U;
            ptr_rw_info->rx_parity_error  = 0U;



            if ((ptr_usart_info->ptr_ro_info->lppdma_tx_used) || (ptr_usart_info->ptr_ro_info->lppdma_rx_used))
            {
                CLK_EnableModuleClock(LPPDMA0_MODULE);
            }

            // Initialize interrupt and peripheral
            LPUART_Open(ptr_ro_info->ptr_USART, 0);
            NVIC_EnableIRQ(ptr_ro_info->irq_n);

            // Allocate LPPDMA RX channel if LPPDMA RX used
            if ((ptr_ro_info->lppdma_rx_used) && (ptr_rw_info->lppdma_rx_chan_id == -1))
            {
                ptr_rw_info->lppdma_rx_chan_id = nu_lppdma_channel_allocate(
                                                     ptr_ro_info->lppdma_rx_perip_mode,
                                                     ptr_ro_info->lppdma_rx_num,
                                                     ptr_ro_info->lppdma_rx_channel
                                                 );
            }

            // Allocate LPPDMA TX channel if LPPDMA TX used
            if ((ptr_ro_info->lppdma_tx_used) && (ptr_rw_info->lppdma_tx_chan_id == -1))
            {
                ptr_rw_info->lppdma_tx_chan_id = nu_lppdma_channel_allocate(
                                                     ptr_ro_info->lppdma_tx_perip_mode,
                                                     ptr_ro_info->lppdma_tx_num,
                                                     ptr_ro_info->lppdma_tx_channel
                                                 );
            }

            // Set driver status to powered
            ptr_rw_info->drv_status.powered = 1U;
            break;

        case ARM_POWER_OFF:

            // If send operation is in progress, abort it
            if (USARTn_GetStatus(ptr_usart_info).tx_busy != 0U)
            {
                (void)USARTn_Control(ptr_usart_info, ARM_USART_ABORT_SEND, 0U);
            }

            // If receive operation is in progress, abort it
            if (USARTn_GetStatus(ptr_usart_info).rx_busy != 0U)
            {
                (void)USARTn_Control(ptr_usart_info, ARM_USART_ABORT_RECEIVE, 0U);
            }

            // De-initialize interrupts and peripheral
            LPUART_Close(ptr_ro_info->ptr_USART);
            NVIC_DisableIRQ(ptr_ro_info->irq_n);

            // Free LPPDMA channels
            if (ptr_rw_info->lppdma_rx_chan_id != -1)
            {
                nu_lppdma_channel_free(ptr_rw_info->lppdma_rx_chan_id);
                ptr_rw_info->lppdma_rx_chan_id = -1;
            }

            if (ptr_rw_info->lppdma_tx_chan_id != -1)
            {
                nu_lppdma_channel_free(ptr_rw_info->lppdma_tx_chan_id);
                ptr_rw_info->lppdma_tx_chan_id = -1;
            }

            // Set driver status to not powered
            ptr_rw_info->drv_status.powered = 0U;

            // Clear communication error status
            ptr_rw_info->rx_overflow      = 0U;
            ptr_rw_info->rx_framing_error = 0U;
            ptr_rw_info->rx_parity_error  = 0U;
            break;

        case ARM_POWER_LOW:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        default:
            return ARM_DRIVER_ERROR_PARAMETER;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USARTn_Send (const USART_Info_t * const ptr_usart_info, const void *data, uint32_t num)
  \brief       Start sending data to USART transmitter.
  \param[in]   ptr_usart_info     Pointer to USART info structure (USART_Info_t)
  \param[in]   data            Pointer to buffer with data to send to USART transmitter
  \param[in]   num             Number of data items to send
  \return      \ref execution_status
*/
static int32_t USARTn_Send(const USART_Info_t *ptr_usart_info, const void *data, uint32_t num)
{
    ARM_USART_STATUS usart_status;
    RW_Info_t *ptr_rw_info = ptr_usart_info->ptr_rw_info;
    TX_Trans_t *ptr_tx_trans = &ptr_rw_info->tx_trans;

    if ((data == NULL) || (num == 0U) || (num > (uint32_t)UINT16_MAX))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (ptr_rw_info->drv_status.configured == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    usart_status = USARTn_GetStatus(ptr_usart_info);

    if (usart_status.tx_busy)
        return ARM_DRIVER_ERROR_BUSY;

    ptr_tx_trans->data = (uint8_t *)data;
    ptr_tx_trans->num_bytes = num;
    ptr_tx_trans->trans_count = 0;

    if (ptr_rw_info->lppdma_tx_chan_id != -1)
    {
        //Use LPPDMA transfer mode
        struct nu_pdma_chn_cb lppdma_chn_cb;

        lppdma_chn_cb.m_eCBType = eCBType_Event;
        lppdma_chn_cb.m_pfnCBHandler = LPPDMA_TX_CB_Handler;
        lppdma_chn_cb.m_pvUserData = (void *)ptr_usart_info;

        //Use LPPDMA transfer mode
        nu_lppdma_filtering_set(ptr_rw_info->lppdma_tx_chan_id, NU_PDMA_EVENT_TRANSFER_DONE | NU_PDMA_EVENT_ABORT);
        nu_lppdma_callback_register(ptr_rw_info->lppdma_tx_chan_id, &lppdma_chn_cb);
        nu_lppdma_transfer(ptr_rw_info->lppdma_tx_chan_id, 8, (uint32_t)ptr_tx_trans->data, (uint32_t)&ptr_usart_info->ptr_ro_info->ptr_USART->DAT, ptr_tx_trans->num_bytes, 0);
        LPUART_EnableInt(ptr_usart_info->ptr_ro_info->ptr_USART, (LPUART_INTEN_TXPDMAEN_Msk | LPUART_INTEN_BUFERRIEN_Msk));
    }
    else
    {
        //Use IRQ transfer mode
        LPUART_EnableInt(ptr_usart_info->ptr_ro_info->ptr_USART, (LPUART_INTEN_THREIEN_Msk | LPUART_INTEN_BUFERRIEN_Msk));
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USARTn_Receive (const USART_Info_t *ptr_usart_info, void *data, uint32_t num)
  \brief       Start receiving data from USART receiver.
  \param[in]   ptr_usart_info  Pointer to USART info structure (USART_Info_t)
  \param[out]  data            Pointer to buffer for data to receive from USART receiver
  \param[in]   num             Number of data items to receive
  \return      \ref execution_status
*/
static int32_t USARTn_Receive(const USART_Info_t *ptr_usart_info, void *data, uint32_t num)
{
    ARM_USART_STATUS usart_status;
    RW_Info_t *ptr_rw_info = ptr_usart_info->ptr_rw_info;
    RX_Trans_t *ptr_rx_trans = &ptr_rw_info->rx_trans;

    if ((data == NULL) || (num == 0U) || (num > (uint32_t)UINT16_MAX))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (ptr_rw_info->drv_status.configured == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    usart_status = USARTn_GetStatus(ptr_usart_info);

    if (usart_status.rx_busy)
        return ARM_DRIVER_ERROR_BUSY;

    ptr_rx_trans->data = (uint8_t *)data;
    ptr_rx_trans->num_bytes = num;
    ptr_rx_trans->trans_count = 0;

    // Clear communication error status
    ptr_rw_info->rx_overflow      = 0U;
    ptr_rw_info->rx_framing_error = 0U;
    ptr_rw_info->rx_parity_error  = 0U;

    if (ptr_rw_info->lppdma_rx_chan_id != -1)
    {
        //Use LPPDMA transfer mode
        struct nu_pdma_chn_cb pdma_chn_cb;

        pdma_chn_cb.m_eCBType = eCBType_Event;
        pdma_chn_cb.m_pfnCBHandler = LPPDMA_RX_CB_Handler;
        pdma_chn_cb.m_pvUserData = (void *)ptr_usart_info;

        //Use LPPDMA transfer mode
        nu_lppdma_filtering_set(ptr_rw_info->lppdma_rx_chan_id, NU_PDMA_EVENT_TRANSFER_DONE | NU_PDMA_EVENT_ABORT);
        nu_lppdma_callback_register(ptr_rw_info->lppdma_rx_chan_id, &pdma_chn_cb);
        nu_lppdma_transfer(ptr_rw_info->lppdma_rx_chan_id, 8, (uint32_t)&ptr_usart_info->ptr_ro_info->ptr_USART->DAT, (uint32_t)ptr_rx_trans->data, ptr_rx_trans->num_bytes, 0);
        LPUART_EnableInt(ptr_usart_info->ptr_ro_info->ptr_USART, (LPUART_INTEN_RXPDMAEN_Msk | LPUART_INTEN_RLSIEN_Msk | LPUART_INTEN_BUFERRIEN_Msk | LPUART_INTEN_TOCNTEN_Msk | LPUART_INTEN_RXTOIEN_Msk));
    }
    else
    {
        //Use IRQ transfer mode
        LPUART_EnableInt(ptr_usart_info->ptr_ro_info->ptr_USART, (LPUART_INTEN_RDAIEN_Msk | LPUART_INTEN_RLSIEN_Msk | LPUART_INTEN_BUFERRIEN_Msk | LPUART_INTEN_TOCNTEN_Msk | LPUART_INTEN_RXTOIEN_Msk));
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USARTn_Transfer (const USART_Info_t *ptr_usart_info, const void *data_out, void *data_in, uint32_t num)
  \brief       Start sending/receiving data to/from USART transmitter/receiver.
  \param[in]   ptr_usart_info  Pointer to USART info structure (USART_Info_t)
  \param[in]   data_out        Pointer to buffer with data to send to USART transmitter
  \param[out]  data_in         Pointer to buffer for data to receive from USART receiver
  \param[in]   num             Number of data items to transfer
  \return      \ref execution_status
*/
static int32_t USARTn_Transfer(const USART_Info_t *ptr_usart_info, const void *data_out, void *data_in, uint32_t num)
{
    (void)data_out;
    (void)data_in;
    (void)num;
    (void)ptr_usart_info;

    if (ptr_usart_info->ptr_rw_info->drv_status.configured == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    // Supported only in Synchronous mode
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}


/**
  \fn          uint32_t USARTn_GetTxCount (const USART_Info_t *ptr_usart_info)
  \brief       Get transmitted data count.
  \param[in]   ptr_usart_info     Pointer to USART info structure (USART_Info_t)
  \return      number of data items transmitted
*/
static uint32_t USARTn_GetTxCount(const USART_Info_t *ptr_usart_info)
{
    uint32_t cnt;
    RW_Info_t *ptr_rw_info = ptr_usart_info->ptr_rw_info;
    TX_Trans_t *ptr_tx_trans = &ptr_rw_info->tx_trans;

    if (ptr_rw_info->drv_status.powered == 0U)
    {
        return 0U;
    }

    if (ptr_rw_info->lppdma_tx_chan_id != -1)
    {
        if (ptr_tx_trans->trans_count != ptr_tx_trans->num_bytes)
            ptr_tx_trans->trans_count = nu_lppdma_transferred_byte_get(ptr_rw_info->lppdma_tx_chan_id, ptr_tx_trans->num_bytes);
    }

    cnt = (uint32_t)ptr_rw_info->tx_trans.trans_count;

    return cnt;
}

/**
  \fn          uint32_t USARTn_GetRxCount (const USART_Info_t *ptr_usart_info)
  \brief       Get received data count.
  \param[in]   ptr_usart_info     Pointer to USART info structure (USART_Info_t)
  \return      number of data items received
*/
static uint32_t USARTn_GetRxCount(const USART_Info_t *ptr_usart_info)
{
    uint32_t cnt;

    RW_Info_t *ptr_rw_info = ptr_usart_info->ptr_rw_info;
    RX_Trans_t *ptr_rx_trans = &ptr_rw_info->rx_trans;

    if (ptr_rw_info->drv_status.powered == 0U)
    {
        return 0U;
    }

    if (ptr_rw_info->lppdma_rx_chan_id != -1)
    {
        if (ptr_rx_trans->trans_count != ptr_rx_trans->num_bytes)
            ptr_rx_trans->trans_count = nu_lppdma_transferred_byte_get(ptr_rw_info->lppdma_rx_chan_id, ptr_rx_trans->num_bytes);
    }

    cnt = (uint32_t)ptr_rw_info->rx_trans.trans_count;
    return cnt;
}

/**
  \fn          int32_t USARTn_Control (const USART_Info_t *ptr_usart_info, uint32_t control, uint32_t arg)
  \brief       Control USART Interface.
  \param[in]   ptr_usart_info     Pointer to USART info structure (USART_Info_t)
  \param[in]   control         Operation
  \param[in]   arg             Argument of operation (optional)
  \return      common \ref execution_status and driver specific \ref usart_execution_status
*/
static int32_t USARTn_Control(const USART_Info_t *ptr_usart_info, uint32_t control, uint32_t arg)
{
    ARM_USART_STATUS status;
    uint32_t         word_len;
    uint32_t         parity = LPUART_PARITY_NONE;
    uint32_t         stop_bits = LPUART_STOP_BIT_1;
    uint32_t         baud_rate = 0;
    RW_Info_t *ptr_rw_info = ptr_usart_info->ptr_rw_info;

    if (ptr_usart_info->ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    // Special handling for Abort Send command
    if ((control & ARM_USART_CONTROL_Msk) == ARM_USART_ABORT_SEND)
    {
        LPUART_DisableInt(ptr_usart_info->ptr_ro_info->ptr_USART, (LPUART_INTEN_THREIEN_Msk | LPUART_INTEN_BUFERRIEN_Msk));

        if (ptr_rw_info->lppdma_tx_chan_id != -1)
        {
            nu_lppdma_channel_terminate(ptr_rw_info->lppdma_tx_chan_id);
        }

        ptr_rw_info->tx_trans.data = NULL;
        ptr_rw_info->tx_trans.num_bytes = 0;
        ptr_rw_info->tx_trans.trans_count = 0;
        return ARM_DRIVER_OK;
    }

    // Special handling for Abort Receive command
    if ((control & ARM_USART_CONTROL_Msk) == ARM_USART_ABORT_RECEIVE)
    {
        LPUART_DisableInt(ptr_usart_info->ptr_ro_info->ptr_USART, (LPUART_INTEN_RDAIEN_Msk | LPUART_INTEN_RLSIEN_Msk | LPUART_INTEN_BUFERRIEN_Msk | LPUART_INTEN_TOCNTEN_Msk | LPUART_INTEN_RXTOIEN_Msk));

        if (ptr_rw_info->lppdma_rx_chan_id != -1)
        {
            nu_lppdma_channel_terminate(ptr_rw_info->lppdma_rx_chan_id);
        }

        ptr_rw_info->rx_trans.data = NULL;
        ptr_rw_info->rx_trans.num_bytes = 0;
        ptr_rw_info->rx_trans.trans_count = 0;
        return ARM_DRIVER_OK;
    }

    // Check if peripheral is busy
    status = USARTn_GetStatus(ptr_usart_info);

    if ((status.tx_busy != 0U) || (status.rx_busy != 0U))
    {
        return ARM_DRIVER_ERROR_BUSY;
    }


    switch (control & ARM_USART_CONTROL_Msk)      // --- Control: Mode and Miscellaneous
    {
        // --- Control Mode
        case ARM_USART_MODE_ASYNCHRONOUS:           // Mode: Asynchronous
            ptr_rw_info->drv_status.configured = 0U;
            break;                                    // Continue configuring parameters after this switch block

        case ARM_USART_MODE_SYNCHRONOUS_MASTER:     // Mode: Synchronous Master
        case ARM_USART_MODE_SYNCHRONOUS_SLAVE:      // Mode: Synchronous Slave
        case ARM_USART_MODE_SINGLE_WIRE:            // Mode: Single-wire
        case ARM_USART_MODE_IRDA:                   // Mode: IrDA
        case ARM_USART_MODE_SMART_CARD:             // Mode: Smart Card
            return ARM_USART_ERROR_MODE;

        // --- Control Miscellaneous
        case ARM_USART_CONTROL_TX:                  // Transmitter; arg: 0=disabled, 1=enabled
            if (arg != 0U)
            {
                // Enable transmitter
                //UART_ENABLE_INT(ptr_usart_info->ptr_ro_info->ptr_USART, UART_INTEN_THREIEN_Msk);
            }
            else
            {
                // Disable transmitter
                LPUART_DISABLE_INT(ptr_usart_info->ptr_ro_info->ptr_USART, (LPUART_INTEN_THREIEN_Msk | LPUART_INTEN_TXPDMAEN_Msk));
            }

            return ARM_DRIVER_OK;

        case ARM_USART_CONTROL_RX:                  // Receiver; arg: 0=disabled, 1=enabled
            if (arg != 0U)
            {
                // Enable receiver
                //UART_ENABLE_INT(ptr_usart_info->ptr_ro_info->ptr_USART, UART_INTEN_RDAIEN_Msk);
            }
            else
            {
                // Disable receiver
                LPUART_DISABLE_INT(ptr_usart_info->ptr_ro_info->ptr_USART, (LPUART_INTEN_RDAIEN_Msk | LPUART_INTEN_RXPDMAEN_Msk));
            }

            return ARM_DRIVER_OK;

        case ARM_USART_SET_DEFAULT_TX_VALUE:        // Set default Transmit value
        case ARM_USART_SET_IRDA_PULSE:              // Set IrDA Pulse in ns
        case ARM_USART_SET_SMART_CARD_GUARD_TIME:   // Set Smart Card Guard Time
        case ARM_USART_SET_SMART_CARD_CLOCK:        // Set Smart Card Clock in Hz
        case ARM_USART_CONTROL_SMART_CARD_NACK:     // Smart Card NACK generation
        case ARM_USART_CONTROL_BREAK:               // Continuous Break transmission
        case ARM_USART_ABORT_TRANSFER:              // Abort ARM_USART_Transfer
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        default:                                    // Unknown Control
            return ARM_DRIVER_ERROR_PARAMETER;
    }

    // Configure all other parameters if command was to set Asynchronous mode (ARM_USART_MODE_ASYNCHRONOUS)

    switch (control & ARM_USART_DATA_BITS_Msk)    // --- Mode Parameters: Data Bits
    {
        case ARM_USART_DATA_BITS_6:                 // Data bits: 6
            word_len = LPUART_WORD_LEN_6;
            break;

        case ARM_USART_DATA_BITS_7:                 // Data bits: 7
            word_len = LPUART_WORD_LEN_7;
            break;

        case ARM_USART_DATA_BITS_8:                 // Data bits: 8
            word_len = LPUART_WORD_LEN_8;
            break;

        default:
            return ARM_USART_ERROR_DATA_BITS;
    }

    switch (control & ARM_USART_PARITY_Msk)       // --- Mode Parameters: Parity
    {
        case ARM_USART_PARITY_NONE:                 // Parity: none
            parity = LPUART_PARITY_NONE;
            break;

        case ARM_USART_PARITY_EVEN:                 // Parity: even
            parity = LPUART_PARITY_EVEN;
            break;

        case ARM_USART_PARITY_ODD:                  // Parity: odd
            parity = LPUART_PARITY_ODD;
            break;

        default:
            return ARM_USART_ERROR_PARITY;
    }

    switch (control & ARM_USART_STOP_BITS_Msk)    // --- Mode Parameters: Stop Bits
    {
        case ARM_USART_STOP_BITS_1:                 // Stop Bits: 1
            stop_bits = LPUART_STOP_BIT_1;
            break;

        case ARM_USART_STOP_BITS_2:                 // Stop Bits: 2
            stop_bits = LPUART_STOP_BIT_2;
            break;

        case ARM_USART_STOP_BITS_1_5:               // Stop Bits: 1.5
            stop_bits = LPUART_STOP_BIT_1_5;
            break;

        default:
            return ARM_USART_ERROR_STOP_BITS;
    }

    switch (control & ARM_USART_FLOW_CONTROL_Msk)   // --- Mode Parameters: Flow Control
    {
        case ARM_USART_FLOW_CONTROL_NONE:             // Flow Control: none
            LPUART_DisableFlowCtrl(ptr_usart_info->ptr_ro_info->ptr_USART);
            break;

        case ARM_USART_FLOW_CONTROL_RTS:              // Flow Control: RTS
            /* Set RTS pin output is low level active */
            ptr_usart_info->ptr_ro_info->ptr_USART->MODEM |= LPUART_MODEM_RTSACTLV_Msk;

            /* Set RTS auto flow control enable */
            ptr_usart_info->ptr_ro_info->ptr_USART->INTEN |= LPUART_INTEN_ATORTSEN_Msk;

            break;

        case ARM_USART_FLOW_CONTROL_CTS:              // Flow Control: CTS
            /* Set CTS pin input is low level active */
            ptr_usart_info->ptr_ro_info->ptr_USART->MODEMSTS |= LPUART_MODEMSTS_CTSACTLV_Msk;

            /* Set CTS auto flow control enable */
            ptr_usart_info->ptr_ro_info->ptr_USART->INTEN |= LPUART_INTEN_ATOCTSEN_Msk;
            break;

        case ARM_USART_FLOW_CONTROL_RTS_CTS:          // Flow Control: RTS/CTS
            /* Enable auto flow control function(RTS/CTS) */
            LPUART_EnableFlowCtrl(ptr_usart_info->ptr_ro_info->ptr_USART);
            break;

        default:
            return ARM_USART_ERROR_FLOW_CONTROL;
    }

    baud_rate = arg;

    // Reconfigure USART
    LPUART_SetLineConfig(ptr_usart_info->ptr_ro_info->ptr_USART, baud_rate, word_len, parity, stop_bits);

    // Set driver status to configured
    ptr_rw_info->drv_status.configured = 1U;
    return ARM_DRIVER_OK;
}

/**
  \fn          ARM_USART_STATUS USARTn_GetStatus (const USART_Info_t * const ptr_usart_info)
  \brief       Get USART status.
  \param[in]   ptr_usart_info     Pointer to USART info structure (USART_Info_t)
  \return      USART status \ref ARM_USART_STATUS
*/
static ARM_USART_STATUS USARTn_GetStatus(const USART_Info_t *ptr_usart_info)
{
    ARM_USART_STATUS status;
    RW_Info_t *ptr_rw_info = ptr_usart_info->ptr_rw_info;

    // Clear status structure
    memset(&status, 0, sizeof(ARM_USART_STATUS));

    if (ptr_rw_info->tx_trans.num_bytes != ptr_rw_info->tx_trans.trans_count)
        status.tx_busy = 1U;

    if (ptr_rw_info->rx_trans.num_bytes != ptr_rw_info->rx_trans.trans_count)
        status.rx_busy = 1U;

    // Process additionally handled communication information
    if (ptr_rw_info->rx_overflow != 0U)
    {
        status.rx_overflow = 1U;
    }

    if (ptr_rw_info->rx_framing_error != 0U)
    {
        status.rx_framing_error = 1U;
    }

    if (ptr_rw_info->rx_parity_error != 0U)
    {
        status.rx_parity_error = 1U;
    }

    return status;
}

/**
  \fn          int32_t USART_SetModemControl (ARM_USART_MODEM_CONTROL control)
  \brief       Set USART Modem Control line state.
  \param[in]   control  \ref ARM_USART_MODEM_CONTROL
  \return      \ref execution_status
*/
static int32_t USART_SetModemControl(ARM_USART_MODEM_CONTROL control)
{
    (void)control;

    // Manual control of modem control lines is not supported
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          ARM_USART_MODEM_STATUS USART_GetModemStatus (void)
  \brief       Get USART Modem Status lines state.
  \return      modem status \ref ARM_USART_MODEM_STATUS
*/
static ARM_USART_MODEM_STATUS USART_GetModemStatus(void)
{
    ARM_USART_MODEM_STATUS modem_status;

    // Retrieving modem control lines status information is supported
    memset(&modem_status, 0, sizeof(ARM_USART_MODEM_STATUS));

    return modem_status;
}

// IRQ handler
void LPUARTn_IRQHandler(const USART_Info_t *ptr_usart_info)
{
    USART_HandleTypeDef *huart = ptr_usart_info->ptr_ro_info->ptr_USART;
    RW_Info_t *ptr_rw_info = ptr_usart_info->ptr_rw_info;
    uint32_t u32IRQStatus = huart->INTSTS;

    if (LPUART_GET_INT_FLAG(huart, LPUART_INTSTS_RDAINT_Msk))
    {
        RX_Trans_t *ptr_rx_trans;
        uint8_t u8InChar = 0xFF;
        ptr_rx_trans = &ptr_rw_info->rx_trans;

        /* Get all the input characters */
        while (LPUART_IS_RX_READY(huart))
        {
            /* Get the character from UART Buffer */
            u8InChar = LPUART_READ(huart);

            /* Put buffer */
            if (ptr_rx_trans->data)
            {
                if (ptr_rx_trans->num_bytes > ptr_rx_trans->trans_count)
                {
                    ptr_rx_trans->data[ptr_rx_trans->trans_count] = u8InChar;
                    ptr_rx_trans->trans_count ++;
                }

                if (ptr_rx_trans->num_bytes == ptr_rx_trans->trans_count)
                {
                    if (ptr_rw_info->cb_event)
                        ptr_rw_info->cb_event(ARM_USART_EVENT_RECEIVE_COMPLETE);
                }
            }
        }
    }

    if (LPUART_GET_INT_FLAG(huart, LPUART_INTSTS_THREINT_Msk))
    {
        TX_Trans_t *ptr_tx_trans;
        uint8_t u8OutChar;
        ptr_tx_trans = &ptr_rw_info->tx_trans;

        if (!LPUART_IS_TX_FULL(huart))
        {
            if (ptr_tx_trans->data)
            {
                if (ptr_tx_trans->num_bytes > ptr_tx_trans->trans_count)
                {
                    u8OutChar = ptr_tx_trans->data[ptr_tx_trans->trans_count];
                    ptr_tx_trans->trans_count++;
                    LPUART_WRITE(huart, u8OutChar);
                }

                if (ptr_tx_trans->num_bytes == ptr_tx_trans->trans_count)
                {
                    LPUART_DisableInt(huart, (UART_INTEN_THREIEN_Msk));

                    if (ptr_rw_info->cb_event)
                    {
                        /* Wait the STOP bit of the last byte has been transmitted. */
                        LPUART_WAIT_TX_EMPTY(huart);
                        ptr_rw_info->cb_event(ARM_USART_EVENT_TX_COMPLETE | ARM_USART_EVENT_SEND_COMPLETE);
                    }
                }
            }
        }
    }

    uint32_t   event = 0;

    if (u32IRQStatus & LPUART_INTSTS_HWTOINT_Msk)
    {
        event |= ARM_USART_EVENT_RX_TIMEOUT;
    }

    if (u32IRQStatus & LPUART_INTSTS_RXTOINT_Msk)
    {
        event |= ARM_USART_EVENT_RX_TIMEOUT;
    }

    if (huart->FIFOSTS & (LPUART_FIFOSTS_FEF_Msk))
    {
        event |= ARM_USART_EVENT_RX_FRAMING_ERROR;
        ptr_rw_info->rx_framing_error = 1U;
    }

    if (huart->FIFOSTS & (LPUART_FIFOSTS_PEF_Msk))
    {
        event |= ARM_USART_EVENT_RX_PARITY_ERROR;
        ptr_rw_info->rx_parity_error = 1U;
    }

    if (huart->FIFOSTS & (LPUART_FIFOSTS_RXOVIF_Msk))
    {
        event |= ARM_USART_EVENT_RX_OVERFLOW;
        ptr_rw_info->rx_overflow = 1U;
    }

    if ((ptr_rw_info->cb_event != NULL) && (event != 0U))
    {
        ptr_rw_info->cb_event(event);
    }

    //Clear RLSINT and BUFERRINT flag
    if (huart->FIFOSTS & (LPUART_FIFOSTS_BIF_Msk | LPUART_FIFOSTS_FEF_Msk | LPUART_FIFOSTS_PEF_Msk | LPUART_FIFOSTS_RXOVIF_Msk))
    {
        LPUART_ClearIntFlag(huart, (LPUART_INTSTS_RLSINT_Msk | LPUART_INTSTS_BUFERRINT_Msk));
    }

    if (LPUART_GET_INT_FLAG(huart, LPUART_INTSTS_BUFERRINT_Msk))
    {
        LPUART_ClearIntFlag(huart, (LPUART_INTSTS_BUFERRINT_Msk));
    }
}

#if (RTE_USART13)
NVT_ITCM void LPUART0_IRQHandler(void)
{
    const USART_Info_t *ptr_usart_info = NULL;
    ptr_usart_info = USART_GetInfo(LPUART0);

    if (ptr_usart_info)
    {
        LPUARTn_IRQHandler(ptr_usart_info);
    }
}

#endif
#endif  // DRIVER_CONFIG_VALID
