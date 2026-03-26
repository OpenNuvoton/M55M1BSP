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
#include "NuMicro.h"
#include "Driver_I2C.h"
#include <string.h>

/*
    1. The implementation does not include pin/clock settings, which need to be configured within the application.
    2. Does not support thread-safety or DMA.
    3. If the master's data transfer exceeds the specified size, I2Cn_SlaveReceive will discard any extra received data.
    4. If the master's data request exceeds the specified size, I2Cn_SlaveTransmit will send dummy data (all zeros).
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

#ifndef I2C_MAX_PORTS_NUM
    #define I2C_MAX_PORTS_NUM      (4U)
#endif

#define SLV_10BIT_ADDR (0x1E<<2)             //1111+0xx+r/w

typedef void (*I2C_FUNC)(uint32_t port);

// Driver status
typedef struct
{
    uint8_t                       initialized  : 1;       // Initialized status: 0 - not initialized, 1 - initialized
    uint8_t                       powered      : 1;       // Power status:       0 - not powered,     1 - powered
    uint8_t                       reserved     : 6;       // Reserved (for padding)
} DriverStatus_t;

// Instance run-time information (RW)
typedef struct
{
    I2C_FUNC                      i2c_handler_Fn;
    ARM_I2C_STATUS                status;
    ARM_I2C_SignalEvent_t         cb_event;               // Event callback
    DriverStatus_t                drv_status;             // Driver status
    uint8_t                       xfer_no_stop;           // Transfer not generating STOP: 0 - not generating STOP, not 0 - generating STOP
    volatile uint8_t              xfer_abort;             // Transfer abort staus: 1 - abort not done might be in progress, 0 - abort done
    uint32_t                      xfer_size;              // Requested transfer size (in bytes)
    uint32_t                      xfer_size_tx;
    uint32_t                      xfer_size_rx;
    uint32_t                      xfer_remain;
    uint32_t                      xfer_remain_tx;
    uint32_t                      xfer_remain_rx;
    const    uint8_t *volatile    slave_xfer_tx_data;     // Pointer to transmit data (for Slave only)
    uint8_t *volatile             slave_xfer_rx_data;     // Pointer to receive  data (for Slave only)
    uint16_t                      slave_xfer_tx_num;      // Requested number of bytes to transmit
    uint16_t                      slave_xfer_rx_num;      // Requested number of bytes to receive
    const    uint8_t *volatile    master_xfer_tx_data;    // Pointer to transmit data (for Master only)
    uint8_t *volatile             master_xfer_rx_data;    // Pointer to receive  data (for Master only)
    uint16_t                      master_xfer_tx_num;     // Requested number of bytes to transmit
    uint16_t                      master_xfer_rx_num;     // Requested number of bytes to receive
    volatile uint8_t              master_xfer_addrH;
    volatile uint8_t              master_xfer_addrL;
    uint32_t                      master_prev_status;
} RW_Info_t;

// Instance compile-time information (RO)
// also contains pointer to run-time information
typedef struct
{
    I2C_T      *i2c;
    IRQn_Type  IRQn;
} RO_Info_t;

// Instance information
typedef struct
{
    const RO_Info_t  *ptr_ro_info;      // Pointer to compile-time information (RO)
    RW_Info_t  *ptr_rw_info;            // Pointer to run-time information (RW)
} I2C_Info_t;

// Macro for declaring functions (for instances)
#define FUNCS_DECLARE(n)                                                                                       \
    static  int32_t                 I2C##n##_Initialize      (ARM_I2C_SignalEvent_t cb_event);                     \
    static  int32_t                 I2C##n##_Uninitialize    (void);                                               \
    static  int32_t                 I2C##n##_PowerControl    (ARM_POWER_STATE state);                              \
    static  int32_t                 I2C##n##_MasterTransmit  (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending); \
    static  int32_t                 I2C##n##_MasterReceive   (uint32_t addr,       uint8_t *data, uint32_t num, bool xfer_pending); \
    static  int32_t                 I2C##n##_SlaveTransmit   (               const uint8_t *data, uint32_t num);   \
    static  int32_t                 I2C##n##_SlaveReceive    (                     uint8_t *data, uint32_t num);   \
    static  int32_t                 I2C##n##_GetDataCount    (void);                                               \
    static  int32_t                 I2C##n##_Control         (uint32_t control, uint32_t arg);                     \
    static  ARM_I2C_STATUS          I2C##n##_GetStatus       (void);

// Macro for defining functions (for instances)
#define FUNCS_DEFINE(n)                                                                                       \
    static  int32_t                 I2C##n##_Initialize      (ARM_I2C_SignalEvent_t cb_event)                     { return I2Cn_Initialize    (n, cb_event); }                         \
    static  int32_t                 I2C##n##_Uninitialize    (void)                                               { return I2Cn_Uninitialize    (n); }                         \
    static  int32_t                 I2C##n##_PowerControl    (ARM_POWER_STATE state)                                                { return I2Cn_PowerControl    (n, state); }                         \
    static  int32_t                 I2C##n##_MasterTransmit  (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)  { return I2Cn_MasterTransmit  (n, addr, data, num, xfer_pending); } \
    static  int32_t                 I2C##n##_MasterReceive   (uint32_t addr,       uint8_t *data, uint32_t num, bool xfer_pending)  { return I2Cn_MasterReceive   (n, addr, data, num, xfer_pending); } \
    static  int32_t                 I2C##n##_SlaveTransmit   (               const uint8_t *data, uint32_t num)                     { return I2Cn_SlaveTransmit   (n, data, num); }                     \
    static  int32_t                 I2C##n##_SlaveReceive    (                     uint8_t *data, uint32_t num)                     { return I2Cn_SlaveReceive    (n, data, num); }                     \
    static  int32_t                 I2C##n##_GetDataCount    (void)                                                                 { return I2Cn_GetDataCount    (n); }                                \
    static  int32_t                 I2C##n##_Control         (uint32_t control, uint32_t arg)                                       { return I2Cn_Control         (n, control, arg); }                  \
    static  ARM_I2C_STATUS          I2C##n##_GetStatus       (void)                                                                 { return I2Cn_GetStatus       (n); }

// Macro for defining driver structures (for instances)
#define I2C_DRIVER(n)                   \
    ARM_DRIVER_I2C Driver_I2C##n = {        \
                                            I2C_GetVersion,                       \
                                            I2C_GetCapabilities,                  \
                                            I2C##n##_Initialize,                  \
                                            I2C##n##_Uninitialize,                \
                                            I2C##n##_PowerControl,                \
                                            I2C##n##_MasterTransmit,              \
                                            I2C##n##_MasterReceive,               \
                                            I2C##n##_SlaveTransmit,               \
                                            I2C##n##_SlaveReceive,                \
                                            I2C##n##_GetDataCount,                \
                                            I2C##n##_Control,                     \
                                            I2C##n##_GetStatus                    \
                                   };


#define ARM_I2C_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion =
{
    ARM_I2C_API_VERSION,
    ARM_I2C_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_I2C_CAPABILITIES DriverCapabilities =
{
    1U,                           // 10-bit addressing supported
    0U                            // Reserved (must be zero)
};

// Local functions prototypes
static ARM_DRIVER_VERSION       I2C_GetVersion(void);
static ARM_I2C_CAPABILITIES     I2C_GetCapabilities(void);
static int32_t                  I2Cn_Initialize(uint32_t port, ARM_I2C_SignalEvent_t cb_event);
static int32_t                  I2Cn_Uninitialize(uint32_t port);
static int32_t                  I2Cn_PowerControl(uint32_t port, ARM_POWER_STATE state);
static int32_t                  I2Cn_MasterTransmit(uint32_t port, uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending);
static int32_t                  I2Cn_MasterReceive(uint32_t port, uint32_t addr,       uint8_t *data, uint32_t num, bool xfer_pending);
static int32_t                  I2Cn_SlaveTransmit(uint32_t port,                const uint8_t *data, uint32_t num);
static int32_t                  I2Cn_SlaveReceive(uint32_t port,                      uint8_t *data, uint32_t num);
static int32_t                  I2Cn_GetDataCount(uint32_t port);
static int32_t                  I2Cn_Control(uint32_t port, uint32_t control, uint32_t arg);
static ARM_I2C_STATUS           I2Cn_GetStatus(uint32_t port);
static void                     I2C_MasterTx(uint32_t port);
static void                     I2C_MasterRx(uint32_t port);
static void                     I2C_SlaveTRx(uint32_t port);

#if (RTE_I2C0 == 1)
static const RO_Info_t i2c0_ro_info =
{
    (I2C_T *)I2C0_BASE, I2C0_IRQn,
};
static RW_Info_t i2c0_rw_info;
FUNCS_DECLARE(0)
FUNCS_DEFINE(0)
I2C_DRIVER(0)
#endif

#if (RTE_I2C1 == 1)
static const RO_Info_t i2c1_ro_info =
{
    (I2C_T *)I2C1_BASE, I2C1_IRQn,
};
static RW_Info_t i2c1_rw_info;
FUNCS_DECLARE(1)
FUNCS_DEFINE(1)
I2C_DRIVER(1)
#endif

#if (RTE_I2C2 == 1)
static const RO_Info_t i2c2_ro_info =
{
    (I2C_T *)I2C2_BASE, I2C2_IRQn,
};
static RW_Info_t i2c2_rw_info;
FUNCS_DECLARE(2)
FUNCS_DEFINE(2)
I2C_DRIVER(2)
#endif

#if (RTE_I2C3 == 1)
static const RO_Info_t i2c3_ro_info =
{
    (I2C_T *)I2C3_BASE, I2C3_IRQn,
};
static RW_Info_t i2c3_rw_info;
FUNCS_DECLARE(3)
FUNCS_DEFINE(3)
I2C_DRIVER(3)
#endif

static const I2C_Info_t i2c_info[I2C_MAX_PORTS_NUM] =
{
#if (RTE_I2C0 == 1)
    {&i2c0_ro_info, &i2c0_rw_info},
#else
    {NULL, NULL},
#endif

#if (RTE_I2C1 == 1)
    {&i2c1_ro_info, &i2c1_rw_info},
#else
    {NULL, NULL},
#endif

#if (RTE_I2C2 == 1)
    {&i2c2_ro_info, &i2c2_rw_info},
#else
    {NULL, NULL},
#endif

#if (RTE_I2C3 == 1)
    {&i2c3_ro_info, &i2c3_rw_info},
#else
    {NULL, NULL},
#endif
};

//
//  Functions
//


static ARM_DRIVER_VERSION I2C_GetVersion(void)
{
    return DriverVersion;
}

static ARM_I2C_CAPABILITIES I2C_GetCapabilities(void)
{
    return DriverCapabilities;
}

static int32_t I2Cn_Initialize(uint32_t port, ARM_I2C_SignalEvent_t cb_event)
{
    if (port >= I2C_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    // Clear run-time info
    memset((void *)i2c_info[port].ptr_rw_info, 0, sizeof(RW_Info_t));
    i2c_info[port].ptr_rw_info->cb_event = cb_event;
    i2c_info[port].ptr_rw_info->drv_status.initialized = 1;
    return ARM_DRIVER_OK;
}

static int32_t I2Cn_Uninitialize(uint32_t port)
{
    if (port >= I2C_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    // Clear run-time info
    memset((void *)i2c_info[port].ptr_rw_info, 0, sizeof(RW_Info_t));
    i2c_info[port].ptr_rw_info->cb_event = NULL;
    i2c_info[port].ptr_rw_info->drv_status.initialized = 0;
    return ARM_DRIVER_OK;
}

static int32_t I2Cn_PowerControl(uint32_t port, ARM_POWER_STATE state)
{
    if (port >= I2C_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    switch (state)
    {
        case ARM_POWER_OFF:
            CLK->I2CCTL &= ~(1U << port);
            NVIC_DisableIRQ(i2c_info[port].ptr_ro_info->IRQn);
            i2c_info[port].ptr_rw_info->drv_status.powered = 0U;
            break;

        case ARM_POWER_LOW:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_POWER_FULL:
            if (i2c_info[port].ptr_rw_info->drv_status.initialized == 0U)
            {
                return ARM_DRIVER_ERROR;
            }

            i2c_info[port].ptr_rw_info->drv_status.powered = 1U;
            CLK->I2CCTL |= (1U << port);
            NVIC_EnableIRQ(i2c_info[port].ptr_ro_info->IRQn);
            break;

        default:
            return ARM_DRIVER_ERROR_PARAMETER;
    }

    return ARM_DRIVER_OK;
}

static int32_t I2Cn_MasterTransmit(uint32_t port, uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)
{
    if (port >= I2C_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    ARM_I2C_STATUS status;
    I2C_T *i2c = i2c_info[port].ptr_ro_info->i2c;

    if ((data == NULL) || (num == 0U))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (addr & ARM_I2C_ADDRESS_10BIT)
    {
        i2c_info[port].ptr_rw_info->master_xfer_addrL = addr & 0xFF;
        i2c_info[port].ptr_rw_info->master_xfer_addrH = (((addr >> 8) & 0x03) | SLV_10BIT_ADDR);
    }
    else
    {
        i2c_info[port].ptr_rw_info->master_xfer_addrL = addr & 0x7F;
        i2c_info[port].ptr_rw_info->master_xfer_addrH = 0;
    }

    i2c_info[port].ptr_rw_info->master_xfer_tx_data = data;
    i2c_info[port].ptr_rw_info->master_xfer_tx_num = num;
    i2c_info[port].ptr_rw_info->xfer_no_stop = xfer_pending;
    i2c_info[port].ptr_rw_info->i2c_handler_Fn = I2C_MasterTx;
    // set xfer_size to 0 for I2Cn_GetDataCount
    i2c_info[port].ptr_rw_info->xfer_size = 0;
    //
    status.busy = 1;
    status.mode = 1; // 1=Master
    status.direction = 0; // 0=Transmitter, 1=Receiver
    i2c_info[port].ptr_rw_info->status = status;
    i2c->CTL0 |= (I2C_CTL0_SI_Msk | I2C_CTL0_STA_Msk | I2C_CTL0_INTEN_Msk);
    return ARM_DRIVER_OK;
}

static int32_t I2Cn_MasterReceive(uint32_t port, uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{
    if (port >= I2C_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    ARM_I2C_STATUS status;
    I2C_T *i2c = i2c_info[port].ptr_ro_info->i2c;

    if ((data == NULL) || (num == 0U))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (addr & ARM_I2C_ADDRESS_10BIT)
    {
        i2c_info[port].ptr_rw_info->master_xfer_addrL = addr & 0xFF;
        i2c_info[port].ptr_rw_info->master_xfer_addrH = (((addr >> 8) & 0x03) | SLV_10BIT_ADDR);
    }
    else
    {
        i2c_info[port].ptr_rw_info->master_xfer_addrL = addr & 0x7F;
        i2c_info[port].ptr_rw_info->master_xfer_addrH = 0;
    }

    i2c_info[port].ptr_rw_info->master_xfer_rx_data = data;
    i2c_info[port].ptr_rw_info->master_xfer_rx_num = num;
    i2c_info[port].ptr_rw_info->xfer_no_stop = xfer_pending;
    i2c_info[port].ptr_rw_info->i2c_handler_Fn = I2C_MasterRx;
    // set xfer_size to 0 for I2Cn_GetDataCount
    i2c_info[port].ptr_rw_info->xfer_size = 0;
    //
    status.busy = 1;
    status.mode = 1; // 1=Master
    status.direction = (addr & ARM_I2C_ADDRESS_10BIT) ? 0 : 1; // 0=Transmitter, 1=Receiver
    i2c_info[port].ptr_rw_info->status = status;
    i2c->CTL0 |= (I2C_CTL0_SI_Msk | I2C_CTL0_STA_Msk | I2C_CTL0_INTEN_Msk);
    return ARM_DRIVER_OK;
}

static int32_t I2Cn_SlaveTransmit(uint32_t port, const uint8_t *data, uint32_t num)
{
    ARM_I2C_STATUS status;
    I2C_T *i2c = i2c_info[port].ptr_ro_info->i2c;

    if ((data == NULL) || (num == 0U) || (port >= I2C_MAX_PORTS_NUM))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    i2c_info[port].ptr_rw_info->slave_xfer_tx_data = data;
    i2c_info[port].ptr_rw_info->slave_xfer_tx_num = num;
    i2c_info[port].ptr_rw_info->i2c_handler_Fn = I2C_SlaveTRx;
    //
    status.busy = 1;
    status.mode = 0; // 0=Slave
    status.direction = 0; // 0=Transmitter, 1=Receiver
    i2c_info[port].ptr_rw_info->status = status;
    i2c->CTL0 |= (I2C_CTL0_SI_Msk | I2C_CTL0_AA_Msk | I2C_CTL0_INTEN_Msk | I2C_CTL0_I2CEN_Msk);
    return ARM_DRIVER_OK;
}

static int32_t I2Cn_SlaveReceive(uint32_t port, uint8_t *data, uint32_t num)
{
    ARM_I2C_STATUS status;
    I2C_T *i2c = i2c_info[port].ptr_ro_info->i2c;

    if ((data == NULL) || (num == 0U) || (port >= I2C_MAX_PORTS_NUM))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    i2c_info[port].ptr_rw_info->slave_xfer_rx_data = data;
    i2c_info[port].ptr_rw_info->slave_xfer_rx_num = num;
    i2c_info[port].ptr_rw_info->i2c_handler_Fn = I2C_SlaveTRx;
    //
    status.busy = 1;
    status.mode = 0; // 0=Slave
    status.direction = 1; // 0=Transmitter, 1=Receiver
    i2c_info[port].ptr_rw_info->status = status;
    i2c->CTL0 |= (I2C_CTL0_SI_Msk | I2C_CTL0_AA_Msk | I2C_CTL0_INTEN_Msk | I2C_CTL0_I2CEN_Msk);
    return ARM_DRIVER_OK;
}

static int32_t I2Cn_GetDataCount(uint32_t port)
{
    return i2c_info[port].ptr_rw_info->xfer_size;
}

static int32_t I2Cn_Control(uint32_t port, uint32_t control, uint32_t arg)
{
    if (port >= I2C_MAX_PORTS_NUM)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    I2C_T *i2c = i2c_info[port].ptr_ro_info->i2c;

    if (i2c_info[port].ptr_rw_info->drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    switch (control)
    {
        case ARM_I2C_OWN_ADDRESS:
            if ((arg & ARM_I2C_ADDRESS_10BIT) != 0U)
            {
                // Own address is a 10-bit address
                i2c->CTL1 |= I2C_CTL1_ADDR10EN_Msk;
            }
            else
            {
                // Own address is a 7-bit address
                i2c->CTL1 &= ~I2C_CTL1_ADDR10EN_Msk;
            }

            if ((arg & ARM_I2C_ADDRESS_GC) != 0U)
            {
                i2c->ADDR0 = ((arg & 0x3FF) << 1) | I2C_ADDR0_GC_Msk;
            }
            else
            {
                // Disable general call
                i2c->ADDR0 = ((arg & 0x3FF) << 1);
            }

            break;

        case ARM_I2C_BUS_SPEED:
            switch (arg)
            {
                case ARM_I2C_BUS_SPEED_STANDARD:
                    I2C_Open(i2c, 100000);
                    break;

                case ARM_I2C_BUS_SPEED_FAST:
                    I2C_Open(i2c, 400000);
                    break;

                case ARM_I2C_BUS_SPEED_FAST_PLUS:
                    I2C_Open(i2c, 1000000);
                    break;

                default:
                    return ARM_DRIVER_ERROR_UNSUPPORTED;
            }

            break;

        case ARM_I2C_BUS_CLEAR:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_I2C_ABORT_TRANSFER:
            i2c_info[port].ptr_rw_info->xfer_abort = 1;
            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

static ARM_I2C_STATUS I2Cn_GetStatus(uint32_t port)
{
    return i2c_info[port].ptr_rw_info->status;
}



// End I2C Interface


#if (RTE_I2C0 == 1)

NVT_ITCM void I2C0_IRQHandler(void)
{
    I2C_FUNC func = i2c_info[0].ptr_rw_info->i2c_handler_Fn;

    if (func != NULL)
    {
        func(0);
    }
}
#endif

#if (RTE_I2C1 == 1)

NVT_ITCM void I2C1_IRQHandler(void)
{
    I2C_FUNC func = i2c_info[1].ptr_rw_info->i2c_handler_Fn;

    if (func != NULL)
    {
        func(1);
    }
}
#endif

#if (RTE_I2C2 == 1)

NVT_ITCM void I2C2_IRQHandler(void)
{
    I2C_FUNC func = i2c_info[2].ptr_rw_info->i2c_handler_Fn;

    if (func != NULL)
    {
        func(2);
    }
}
#endif

#if (RTE_I2C3 == 1)

NVT_ITCM void I2C3_IRQHandler(void)
{
    I2C_FUNC func = i2c_info[3].ptr_rw_info->i2c_handler_Fn;

    if (func != NULL)
    {
        func(3);
    }
}
#endif


/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
static void I2C_MasterRx(uint32_t port)
{
    I2C_T *i2c = i2c_info[port].ptr_ro_info->i2c;
    uint32_t u32Status;
    u32Status = I2C_GET_STATUS(i2c);
    ARM_I2C_SignalEvent_t cb_event = i2c_info[port].ptr_rw_info->cb_event;
    uint32_t event = 0;

    if (i2c_info[port].ptr_rw_info->xfer_abort == 1)
    {
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
        i2c_info[port].ptr_rw_info->xfer_abort = 0;
        //
        i2c_info[port].ptr_rw_info->status.busy = 0;

        if (cb_event != NULL)
        {
            cb_event(ARM_I2C_EVENT_TRANSFER_DONE | ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
        }

        return;
    }

    // Master Start or Repeat Start
    if ((u32Status == 0x08) || (u32Status == 0x10))
    {
        uint8_t u8DeviceHAddr = i2c_info[port].ptr_rw_info->master_xfer_addrH;
        uint8_t u8DeviceLAddr = i2c_info[port].ptr_rw_info->master_xfer_addrL;
        ARM_I2C_STATUS status = i2c_info[port].ptr_rw_info->status;
        i2c_info[port].ptr_rw_info->xfer_size = 0;
        i2c_info[port].ptr_rw_info->xfer_remain = i2c_info[port].ptr_rw_info->master_xfer_rx_num;

        if (u8DeviceHAddr)
        {
            if (status.direction == 0)
            {
                I2C_SET_DATA(i2c, (u8DeviceHAddr << 1)); /* Write SLA+W to Register I2CDAT */
            }
            else
            {
                I2C_SET_DATA(i2c, (u8DeviceHAddr << 1) | 1u); /* Write SLA+R to Register I2CDAT */
            }
        }
        else
        {
            I2C_SET_DATA(i2c, (u8DeviceLAddr << 1) | 1u); /* Write SLA+R to Register I2CDAT */
            i2c_info[port].ptr_rw_info->status.direction = 1;
        }

        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
    }
    // Master Transmit Address ACK
    else if (u32Status == 0x18)
    {
        uint8_t u8DeviceHAddr = i2c_info[port].ptr_rw_info->master_xfer_addrH;
        uint8_t u8DeviceLAddr = i2c_info[port].ptr_rw_info->master_xfer_addrL;

        if ((i2c_info[port].ptr_rw_info->master_prev_status != 0x18)
                && (u8DeviceHAddr != 0))
        {
            I2C_SET_DATA(i2c, u8DeviceLAddr);
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
        }
        else
        {
            i2c_info[port].ptr_rw_info->status.direction = 1;
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_STA_SI);
        }
    }
    // Master Transmit Address NACK or Master Receive Address NACK
    else if ((u32Status == 0x20) || (u32Status == 0x48))
    {
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
        i2c_info[port].ptr_rw_info->status.busy = 0;
        event = ARM_I2C_EVENT_ADDRESS_NACK;
    }
    // Master Transmit Data ACK
    else if (u32Status == 0x28)
    {
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_STA_SI);
    }
    // Master Receive Address ACK
    else if (u32Status == 0x40)
    {
        if (i2c_info[port].ptr_rw_info->master_xfer_rx_num == 1)
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
        }
    }
    // Master Receive Data ACK
    else if (u32Status == 0x50)
    {
        *(i2c_info[port].ptr_rw_info->master_xfer_rx_data + i2c_info[port].ptr_rw_info->xfer_size) = I2C_GET_DATA(i2c);
        i2c_info[port].ptr_rw_info->xfer_size += 1;
        i2c_info[port].ptr_rw_info->xfer_remain -= 1;

        if (i2c_info[port].ptr_rw_info->xfer_remain > 1)
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
        }
        else if (i2c_info[port].ptr_rw_info->xfer_remain == 1)
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
        }
        else
        {
            i2c_info[port].ptr_rw_info->status.busy = 0;
            event = ARM_I2C_EVENT_TRANSFER_DONE;

            if (i2c_info[port].ptr_rw_info->xfer_no_stop != 0)
            {
                I2C_DisableInt(i2c);
            }
            else
            {
                I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
            }
        }
    }
    // Master Receive Data NACK
    else if (u32Status == 0x58)
    {
        *(i2c_info[port].ptr_rw_info->master_xfer_rx_data + i2c_info[port].ptr_rw_info->xfer_size) = I2C_GET_DATA(i2c);
        i2c_info[port].ptr_rw_info->xfer_size += 1;
        i2c_info[port].ptr_rw_info->xfer_remain -= 1;
        i2c_info[port].ptr_rw_info->status.busy = 0;
        event = (i2c_info[port].ptr_rw_info->xfer_remain == 0) \
                ? ARM_I2C_EVENT_TRANSFER_DONE : ARM_I2C_EVENT_TRANSFER_INCOMPLETE;

        if (i2c_info[port].ptr_rw_info->xfer_no_stop != 0)
        {
            I2C_DisableInt(i2c);
        }
        else
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
        }
    }
    else if (u32Status == 0xF8)     /*I2C wave keeps going*/
    {
    }
    else /* Error condition process */
    {
        // Master Arbitration Lost
        if (u32Status == 0x38)                /* Master arbitration lost, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
            event = ARM_I2C_EVENT_ARBITRATION_LOST;
        }
        // Master Transmit Data NACK
        else if (u32Status == 0x30)
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);

            if (i2c_info[port].ptr_rw_info->xfer_remain == 0)
            {
                event = ARM_I2C_EVENT_TRANSFER_DONE;
            }
            else
            {
                event = ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
            }
        }
        // Master Receive Address NACK
        else if (u32Status == 0x48)
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
            event = ARM_I2C_EVENT_ADDRESS_NACK;
        }
        // Bus error
        else if (u32Status == 0x00)
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
            event = ARM_I2C_EVENT_BUS_ERROR;
        }
        else
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
        }
    }

    if ((event != 0) && (cb_event != NULL))
    {
        cb_event(event | ARM_I2C_EVENT_TRANSFER_DONE);
        i2c_info[port].ptr_rw_info->status.busy = 0;
    }

    i2c_info[port].ptr_rw_info->master_prev_status = u32Status;
    u32Status = I2C_GET_STATUS(i2c);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
static void I2C_MasterTx(uint32_t port)
{
    I2C_T *i2c = i2c_info[port].ptr_ro_info->i2c;
    uint32_t u32Status = I2C_GET_STATUS(i2c);
    ARM_I2C_SignalEvent_t cb_event = i2c_info[port].ptr_rw_info->cb_event;
    uint32_t event = 0;

    if (i2c_info[port].ptr_rw_info->xfer_abort == 1)
    {
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
        i2c_info[port].ptr_rw_info->xfer_abort = 0;
        //
        i2c_info[port].ptr_rw_info->status.busy = 0;

        if (cb_event != NULL)
        {
            cb_event(ARM_I2C_EVENT_TRANSFER_DONE | ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
        }

        return;
    }

    // Master Start or Repeat Start
    if ((u32Status == 0x08) || (u32Status == 0x10))
    {
        uint8_t u8DeviceHAddr = i2c_info[port].ptr_rw_info->master_xfer_addrH;
        uint8_t u8DeviceLAddr = i2c_info[port].ptr_rw_info->master_xfer_addrL;
        i2c_info[port].ptr_rw_info->xfer_size = 0;

        if (u8DeviceHAddr)
        {
            I2C_SET_DATA(i2c, (u8DeviceHAddr << 1)); /* Write SLA+W to Register I2CDAT */
        }
        else
        {
            I2C_SET_DATA(i2c, (u8DeviceLAddr << 1)); /* Write SLA+W to Register I2CDAT */
        }

        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
    }
    // Master Transmit Address ACK
    else if (u32Status == 0x18)
    {
        uint8_t u8DeviceHAddr = i2c_info[port].ptr_rw_info->master_xfer_addrH;
        uint8_t u8DeviceLAddr = i2c_info[port].ptr_rw_info->master_xfer_addrL;

        if ((i2c_info[port].ptr_rw_info->master_prev_status != 0x18)
                && (u8DeviceHAddr != 0))
        {
            I2C_SET_DATA(i2c, u8DeviceLAddr);
        }
        else
        {
            I2C_SET_DATA(i2c, *(i2c_info[port].ptr_rw_info->master_xfer_tx_data + i2c_info[port].ptr_rw_info->xfer_size));
            i2c_info[port].ptr_rw_info->xfer_size += 1;
        }

        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
    }
    // Master Transmit Address NACK
    else if (u32Status == 0x20)
    {
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
        i2c_info[port].ptr_rw_info->status.busy = 0;
        event = ARM_I2C_EVENT_ADDRESS_NACK;
    }
    // Master Transmit Data ACK
    else if (u32Status == 0x28)
    {
        if (i2c_info[port].ptr_rw_info->xfer_size != i2c_info[port].ptr_rw_info->master_xfer_tx_num)
        {
            I2C_SET_DATA(i2c, *(i2c_info[port].ptr_rw_info->master_xfer_tx_data + i2c_info[port].ptr_rw_info->xfer_size));
            i2c_info[port].ptr_rw_info->xfer_size += 1;
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
        }
        else
        {
            i2c_info[port].ptr_rw_info->status.busy = 0;
            event = ARM_I2C_EVENT_TRANSFER_DONE;

            if (i2c_info[port].ptr_rw_info->xfer_no_stop != 0)
            {
                I2C_DisableInt(i2c);
            }
            else
            {
                I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
            }
        }
    }
    else if (u32Status == 0xF8)     /*I2C wave keeps going*/
    {
    }
    else // /* Error condition process */
    {
        // Master Arbitration Lost
        if (u32Status == 0x38)
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
            event = ARM_I2C_EVENT_ARBITRATION_LOST;
        }
        // Bus error
        else if (u32Status == 0x00)
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
            event = ARM_I2C_EVENT_BUS_ERROR;
        }
        else if (u32Status == 0x30)             /* Master transmit data NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);

            if (i2c_info[port].ptr_rw_info->xfer_remain == 0)
            {
                event = ARM_I2C_EVENT_TRANSFER_DONE;
            }
            else
            {
                event = ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
            }
        }
        // Master Receive Address NACK
        else if (u32Status == 0x48)
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
            event = ARM_I2C_EVENT_ADDRESS_NACK;
        }
        else
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
            event = ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
        }
    }

    if ((event != 0) && (cb_event != NULL))
    {
        cb_event(event | ARM_I2C_EVENT_TRANSFER_DONE);
        i2c_info[port].ptr_rw_info->status.busy = 0;
    }

    i2c_info[port].ptr_rw_info->master_prev_status = u32Status;
    u32Status = I2C_GET_STATUS(i2c);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  I2C TRx Callback Function                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
static void I2C_SlaveTRx(uint32_t port)
{
    I2C_T *i2c = i2c_info[port].ptr_ro_info->i2c;
    RW_Info_t *ptr_rw_info = i2c_info[port].ptr_rw_info;
    uint32_t u32Status = I2C_GET_STATUS(i2c);
    uint32_t direction = ptr_rw_info->status.direction;
    ARM_I2C_SignalEvent_t cb_event = i2c_info[port].ptr_rw_info->cb_event;
    uint32_t event = 0;

    // Slave Receive Address ACK
    if (u32Status == 0x60)
    {
        // ptr_rw_info->xfer_size = 0;
        ptr_rw_info->xfer_size_rx = 0;
        ptr_rw_info->xfer_remain_rx = ptr_rw_info->slave_xfer_rx_num;
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    // Slave Receive Data ACK or Slave Receive Data NACK
    else if ((u32Status == 0x80) || (u32Status == 0x88))
    {
        uint8_t u8Data = (unsigned char) I2C_GET_DATA(i2c);
        ptr_rw_info->xfer_size += 1;

        if (direction == 1)   // Direction: 0=Transmitter, 1=Receiver
        {
            if (ptr_rw_info->xfer_remain_rx > 0)
            {
                *(ptr_rw_info->slave_xfer_rx_data + ptr_rw_info->xfer_size_rx) = u8Data;
                ptr_rw_info->xfer_remain_rx -= 1;
                ptr_rw_info->xfer_size_rx += 1;
            }
        }

        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    // Slave Transmit Address ACK
    else if (u32Status == 0xA8)
    {
        ptr_rw_info->xfer_size_tx = 0;
        ptr_rw_info->xfer_remain_tx = ptr_rw_info->slave_xfer_tx_num;

        if ((direction == 0) && (ptr_rw_info->xfer_remain_tx > 0))   // Direction: 0=Transmitter, 1=Receiver
        {
            I2C_SET_DATA(i2c, *(ptr_rw_info->slave_xfer_tx_data + ptr_rw_info->xfer_size_tx));
            ptr_rw_info->xfer_remain_tx -= 1;
            ptr_rw_info->xfer_size_tx += 1;
        }
        else
        {
            I2C_SET_DATA(i2c, 0x00); // dummy
        }

        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    // Slave Transmit Data ACK
    else if (u32Status == 0xB8)
    {
        if ((direction == 0) && (ptr_rw_info->xfer_remain_tx > 0))   // Direction: 0=Transmitter, 1=Receiver
        {
            I2C_SET_DATA(i2c, *(ptr_rw_info->slave_xfer_tx_data + ptr_rw_info->xfer_size_tx));
            ptr_rw_info->xfer_remain_tx -= 1;
            ptr_rw_info->xfer_size_tx += 1;
        }
        else
        {
            I2C_SET_DATA(i2c, 0x00); // dummy
        }

        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    // Slave Transmit Data NACK
    else if (u32Status == 0xC0)
    {
        if (direction == 0)
        {
            if (ptr_rw_info->xfer_remain_tx == 0)
            {
                ptr_rw_info->status.busy = 0;
                event = ARM_I2C_EVENT_TRANSFER_DONE;
            }
        }

        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    // Slave Receive Data NACK
    else if (u32Status == 0x88)
    {
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    // Slave Transmit Repeat Start or Stop
    else if (u32Status == 0xA0)
    {
        ptr_rw_info->status.busy = 0;

        if (direction == 1)   // Direction: 0=Transmitter, 1=Receiver
        {
            if (ptr_rw_info->xfer_remain_rx == 0)
            {
                event = ARM_I2C_EVENT_TRANSFER_DONE;
            }
        }

        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    // GC mode Address ACK
    else if (u32Status == 0x70)
    {
        ptr_rw_info->xfer_size_rx = 0;
        ptr_rw_info->xfer_remain_rx = ptr_rw_info->slave_xfer_rx_num;
        event = ARM_I2C_EVENT_GENERAL_CALL;
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    // GC mode Data ACK
    else if (u32Status == 0x90)
    {
        uint8_t u8Data = (unsigned char) I2C_GET_DATA(i2c);
        ptr_rw_info->xfer_size += 1;

        if (direction == 1)   // Direction: 0=Transmitter, 1=Receiver
        {
            if (ptr_rw_info->xfer_remain_rx > 0)
            {
                *(ptr_rw_info->slave_xfer_rx_data + ptr_rw_info->xfer_size_rx) = u8Data;
                ptr_rw_info->xfer_remain_rx -= 1;
                ptr_rw_info->xfer_size_rx += 1;
            }
        }

        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    // GC mode Data NACK
    else if (u32Status == 0x98)
    {
        I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
    }
    else if (u32Status == 0xF8)     /*I2C wave keeps going*/
    {
    }
    else
    {
        // Slave Receive Arbitration Lost
        if (u32Status == 0x68)
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
            event = ARM_I2C_EVENT_ARBITRATION_LOST;
        }
        // Address Transmit Arbitration Lost
        else if (u32Status == 0xB0)
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI_AA);
            event = ARM_I2C_EVENT_ARBITRATION_LOST;
        }
        else
        {
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
            event = ARM_I2C_EVENT_BUS_ERROR;
        }
    }

    if ((event != 0) && (cb_event != NULL))
    {
        cb_event(event | ARM_I2C_EVENT_TRANSFER_DONE);
        ptr_rw_info->status.busy = 0;
    }

    u32Status = I2C_GET_STATUS(i2c);
}


