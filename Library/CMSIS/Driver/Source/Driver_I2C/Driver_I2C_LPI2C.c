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
    3. If the master's data transfer exceeds the specified size, LPI2C0_SlaveReceive will discard any extra received data.
    4. If the master's data request exceeds the specified size, LPI2C0_SlaveTransmit will send dummy data (all zeros).
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

#if (RTE_I2C4 == 1)

typedef void (*LPI2C_FUNC)(void);

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
    LPI2C_FUNC                    i2c_handler_fn;
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
    volatile uint8_t              master_xfer_addr;
    uint32_t                      master_prev_status;
} RW_Info_t;


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
    0U,  // do not support 10-bit addressing
    0U   // Reserved (must be zero)
};

// Local functions prototypes
static ARM_DRIVER_VERSION       LPI2C0_GetVersion(void);
static ARM_I2C_CAPABILITIES     LPI2C0_GetCapabilities(void);
static int32_t                  LPI2C0_Initialize(ARM_I2C_SignalEvent_t cb_event);
static int32_t                  LPI2C0_Uninitialize();
static int32_t                  LPI2C0_PowerControl(ARM_POWER_STATE state);
static int32_t                  LPI2C0_MasterTransmit(uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending);
static int32_t                  LPI2C0_MasterReceive(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending);
static int32_t                  LPI2C0_SlaveTransmit(const uint8_t *data, uint32_t num);
static int32_t                  LPI2C0_SlaveReceive(uint8_t *data, uint32_t num);
static int32_t                  LPI2C0_GetDataCount(void);
static int32_t                  LPI2C0_Control(uint32_t control, uint32_t arg);
static ARM_I2C_STATUS           LPI2C0_GetStatus(void);
static void                     LPI2C0_MasterTx(void);
static void                     LPI2C0_MasterRx(void);
static void                     LPI2C0_SlaveTRx(void);

static RW_Info_t lpi2c0_rw_info;


//
//  Functions
//

static ARM_DRIVER_VERSION LPI2C0_GetVersion(void)
{
    return DriverVersion;
}

static ARM_I2C_CAPABILITIES LPI2C0_GetCapabilities(void)
{
    return DriverCapabilities;
}

static int32_t LPI2C0_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
    // Clear run-time info
    memset((void *)&lpi2c0_rw_info, 0, sizeof(RW_Info_t));
    lpi2c0_rw_info.cb_event = cb_event;
    lpi2c0_rw_info.drv_status.initialized = 1;
    return ARM_DRIVER_OK;
}

static int32_t LPI2C0_Uninitialize(void)
{
    // Clear run-time info
    memset((void *)&lpi2c0_rw_info, 0, sizeof(RW_Info_t));
    lpi2c0_rw_info.cb_event = NULL;
    lpi2c0_rw_info.drv_status.initialized = 0;
    return ARM_DRIVER_OK;
}

static int32_t LPI2C0_PowerControl(ARM_POWER_STATE state)
{
    switch (state)
    {
        case ARM_POWER_OFF:
            CLK->LPI2CCTL &= ~(CLK_LPI2CCTL_LPI2C0CKEN_Msk);
            NVIC_DisableIRQ(LPI2C0_IRQn);
            lpi2c0_rw_info.drv_status.powered = 0U;
            break;

        case ARM_POWER_LOW:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_POWER_FULL:
            if (lpi2c0_rw_info.drv_status.initialized == 0U)
            {
                return ARM_DRIVER_ERROR;
            }

            lpi2c0_rw_info.drv_status.powered = 1U;
            CLK->LPI2CCTL |= CLK_LPI2CCTL_LPI2C0CKEN_Msk;
            NVIC_EnableIRQ(LPI2C0_IRQn);
            break;

        default:
            return ARM_DRIVER_ERROR_PARAMETER;
    }

    return ARM_DRIVER_OK;
}

static int32_t LPI2C0_MasterTransmit(uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)
{
    ARM_I2C_STATUS status;
    LPI2C_T *i2c = (LPI2C_T *)LPI2C0;

    if ((data == NULL) || (num == 0U) || (addr & ARM_I2C_ADDRESS_10BIT))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    // Only Support 7-Bit Address Mode
    lpi2c0_rw_info.master_xfer_addr = addr & 0x7F;
    lpi2c0_rw_info.master_xfer_tx_data = data;
    lpi2c0_rw_info.master_xfer_tx_num = num;
    lpi2c0_rw_info.xfer_no_stop = xfer_pending;
    lpi2c0_rw_info.i2c_handler_fn = LPI2C0_MasterTx;
    // set xfer_size to 0 for LPI2C0_GetDataCount
    lpi2c0_rw_info.xfer_size = 0;
    //
    status.busy = 1;
    status.mode = 1; // 1=Master
    status.direction = 0; // 0=Transmitter, 1=Receiver
    lpi2c0_rw_info.status = status;
    i2c->CTL0 |= (LPI2C_CTL0_SI_Msk | LPI2C_CTL0_STA_Msk | LPI2C_CTL0_INTEN_Msk);
    return ARM_DRIVER_OK;
}

static int32_t LPI2C0_MasterReceive(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{
    ARM_I2C_STATUS status;
    LPI2C_T *i2c = (LPI2C_T *)LPI2C0;

    if ((data == NULL) || (num == 0U) || (addr & ARM_I2C_ADDRESS_10BIT))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    lpi2c0_rw_info.master_xfer_addr = addr & 0x7F;
    lpi2c0_rw_info.master_xfer_rx_data = data;
    lpi2c0_rw_info.master_xfer_rx_num = num;
    lpi2c0_rw_info.xfer_no_stop = xfer_pending;
    lpi2c0_rw_info.i2c_handler_fn = LPI2C0_MasterRx;
    // set xfer_size to 0 for LPI2C_GetDataCount
    lpi2c0_rw_info.xfer_size = 0;
    //
    status.busy = 1;
    status.mode = 1; // 1=Master
    status.direction = 1; // 0=Transmitter, 1=Receiver
    lpi2c0_rw_info.status = status;
    i2c->CTL0 |= (LPI2C_CTL0_SI_Msk | LPI2C_CTL0_STA_Msk | LPI2C_CTL0_INTEN_Msk);
    return ARM_DRIVER_OK;
}

static int32_t LPI2C0_SlaveTransmit(const uint8_t *data, uint32_t num)
{
    ARM_I2C_STATUS status;
    LPI2C_T *i2c = (LPI2C_T *)LPI2C0;

    if ((data == NULL) || (num == 0U))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    lpi2c0_rw_info.slave_xfer_tx_data = data;
    lpi2c0_rw_info.slave_xfer_tx_num = num;
    lpi2c0_rw_info.i2c_handler_fn = LPI2C0_SlaveTRx;
    //
    status.busy = 1;
    status.mode = 0; // 0=Slave
    status.direction = 0; // 0=Transmitter, 1=Receiver
    lpi2c0_rw_info.status = status;
    i2c->CTL0 |= (LPI2C_CTL0_SI_Msk | LPI2C_CTL0_AA_Msk | LPI2C_CTL0_INTEN_Msk | LPI2C_CTL0_LPI2CEN_Msk);
    return ARM_DRIVER_OK;
}

static int32_t LPI2C0_SlaveReceive(uint8_t *data, uint32_t num)
{
    ARM_I2C_STATUS status;
    LPI2C_T *i2c = (LPI2C_T *)LPI2C0;

    if ((data == NULL) || (num == 0U))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    lpi2c0_rw_info.slave_xfer_rx_data = data;
    lpi2c0_rw_info.slave_xfer_rx_num = num;
    lpi2c0_rw_info.i2c_handler_fn = LPI2C0_SlaveTRx;
    //
    status.busy = 1;
    status.mode = 0; // 0=Slave
    status.direction = 1; // 0=Transmitter, 1=Receiver
    lpi2c0_rw_info.status = status;
    i2c->CTL0 |= (LPI2C_CTL0_SI_Msk | LPI2C_CTL0_AA_Msk | LPI2C_CTL0_INTEN_Msk | LPI2C_CTL0_LPI2CEN_Msk);
    return ARM_DRIVER_OK;
}

static int32_t LPI2C0_GetDataCount(void)
{
    return lpi2c0_rw_info.xfer_size;
}

static int32_t LPI2C0_Control(uint32_t control, uint32_t arg)
{
    LPI2C_T *i2c = (LPI2C_T *)LPI2C0;

    if (lpi2c0_rw_info.drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    switch (control)
    {
        case ARM_I2C_OWN_ADDRESS:
            if ((arg & ARM_I2C_ADDRESS_10BIT) != 0U)
            {
                return ARM_DRIVER_ERROR_UNSUPPORTED;
            }

            if ((arg & ARM_I2C_ADDRESS_GC) != 0U)
            {
                i2c->ADDR0 = ((arg & 0x7F) << 1) | LPI2C_ADDR0_GC_Msk;
            }
            else
            {
                // Disable general call
                i2c->ADDR0 = ((arg & 0x7F) << 1);
            }

            break;

        case ARM_I2C_BUS_SPEED:
            switch (arg)
            {
                case ARM_I2C_BUS_SPEED_STANDARD:
                    LPI2C_Open(i2c, 100000);
                    break;

                case ARM_I2C_BUS_SPEED_FAST:
                    LPI2C_Open(i2c, 400000);
                    break;

                case ARM_I2C_BUS_SPEED_FAST_PLUS:
                    LPI2C_Open(i2c, 1000000);
                    break;

                default:
                    return ARM_DRIVER_ERROR_UNSUPPORTED;
            }

            break;

        case ARM_I2C_BUS_CLEAR:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_I2C_ABORT_TRANSFER:
            lpi2c0_rw_info.xfer_abort = 1;
            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

static ARM_I2C_STATUS LPI2C0_GetStatus(void)
{
    return lpi2c0_rw_info.status;
}

// End I2C Interface


NVT_ITCM void LPI2C0_IRQHandler(void)
{
    LPI2C_FUNC func = lpi2c0_rw_info.i2c_handler_fn;

    if (func != NULL)
    {
        func();
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
static void LPI2C0_MasterRx(void)
{
    LPI2C_T *i2c = (LPI2C_T *)LPI2C0;
    uint32_t u32Status;
    u32Status = LPI2C_GET_STATUS(i2c);
    ARM_I2C_SignalEvent_t cb_event = lpi2c0_rw_info.cb_event;
    uint32_t event = 0;

    if (lpi2c0_rw_info.xfer_abort == 1)
    {
        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);
        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
        lpi2c0_rw_info.xfer_abort = 0;
        //
        lpi2c0_rw_info.status.busy = 0;

        if (cb_event != NULL)
        {
            cb_event(ARM_I2C_EVENT_TRANSFER_DONE | ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
        }

        return;
    }

    // Master Start or Repeat Start
    if ((u32Status == 0x08) || (u32Status == 0x10))
    {
        uint8_t u8DeviceAddr = lpi2c0_rw_info.master_xfer_addr;
        lpi2c0_rw_info.xfer_size = 0;
        lpi2c0_rw_info.xfer_remain = lpi2c0_rw_info.master_xfer_rx_num;
        LPI2C_SET_DATA(i2c, (u8DeviceAddr << 1) | 1u); /* Write SLA+R to Register I2CDAT */
        lpi2c0_rw_info.status.direction = 1;
        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
    }
    // Master Transmit Address ACK
    else if (u32Status == 0x18)
    {
        lpi2c0_rw_info.status.direction = 1;
        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STA_SI);
    }
    // Master Transmit Address NACK or Master Receive Address NACK
    else if ((u32Status == 0x20) || (u32Status == 0x48))
    {
        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);
        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
        lpi2c0_rw_info.status.busy = 0;
        event = ARM_I2C_EVENT_ADDRESS_NACK;
    }
    // Master Transmit Data ACK
    else if (u32Status == 0x28)
    {
        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STA_SI);
    }
    // Master Receive Address ACK
    else if (u32Status == 0x40)
    {
        if (lpi2c0_rw_info.master_xfer_rx_num == 1)
        {
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
        }
        else
        {
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI_AA);
        }
    }
    // Master Receive Data ACK
    else if (u32Status == 0x50)
    {
        *(lpi2c0_rw_info.master_xfer_rx_data + lpi2c0_rw_info.xfer_size) = LPI2C_GET_DATA(i2c);
        lpi2c0_rw_info.xfer_size += 1;
        lpi2c0_rw_info.xfer_remain -= 1;

        if (lpi2c0_rw_info.xfer_remain > 1)
        {
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI_AA);
        }
        else if (lpi2c0_rw_info.xfer_remain == 1)
        {
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
        }
        else
        {
            lpi2c0_rw_info.status.busy = 0;
            event = ARM_I2C_EVENT_TRANSFER_DONE;

            if (lpi2c0_rw_info.xfer_no_stop != 0)
            {
                LPI2C_DisableInt(i2c);
            }
            else
            {
                LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);
            }
        }
    }
    // Master Receive Data NACK
    else if (u32Status == 0x58)
    {
        *(lpi2c0_rw_info.master_xfer_rx_data + lpi2c0_rw_info.xfer_size) = LPI2C_GET_DATA(i2c);
        lpi2c0_rw_info.xfer_size += 1;
        lpi2c0_rw_info.xfer_remain -= 1;
        lpi2c0_rw_info.status.busy = 0;
        event = (lpi2c0_rw_info.xfer_remain == 0) \
                ? ARM_I2C_EVENT_TRANSFER_DONE : ARM_I2C_EVENT_TRANSFER_INCOMPLETE;

        if (lpi2c0_rw_info.xfer_no_stop != 0)
        {
            LPI2C_DisableInt(i2c);
        }
        else
        {
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);
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
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
            event = ARM_I2C_EVENT_ARBITRATION_LOST;
        }
        // Master Transmit Data NACK
        else if (u32Status == 0x30)
        {
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);

            if (lpi2c0_rw_info.xfer_remain == 0)
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
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
            event = ARM_I2C_EVENT_ADDRESS_NACK;
        }
        // Bus error
        else if (u32Status == 0x00)
        {
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
            event = ARM_I2C_EVENT_BUS_ERROR;
        }
        else
        {
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
        }
    }

    if ((event != 0) && (cb_event != NULL))
    {
        cb_event(event | ARM_I2C_EVENT_TRANSFER_DONE);
        lpi2c0_rw_info.status.busy = 0;
    }

    lpi2c0_rw_info.master_prev_status = u32Status;
    u32Status = LPI2C_GET_STATUS(i2c);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
static void LPI2C0_MasterTx(void)
{
    LPI2C_T *i2c = (LPI2C_T *)LPI2C0;
    uint32_t u32Status = LPI2C_GET_STATUS(i2c);
    ARM_I2C_SignalEvent_t cb_event = lpi2c0_rw_info.cb_event;
    uint32_t event = 0;

    if (lpi2c0_rw_info.xfer_abort == 1)
    {
        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);
        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
        lpi2c0_rw_info.xfer_abort = 0;
        //
        lpi2c0_rw_info.status.busy = 0;

        if (cb_event != NULL)
        {
            cb_event(ARM_I2C_EVENT_TRANSFER_DONE | ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
        }

        return;
    }

    // Master Start or Repeat Start
    if ((u32Status == 0x08) || (u32Status == 0x10))
    {
        uint8_t u8DeviceAddr = lpi2c0_rw_info.master_xfer_addr;
        lpi2c0_rw_info.xfer_size = 0;
        LPI2C_SET_DATA(i2c, (u8DeviceAddr << 1)); /* Write SLA+W to Register I2CDAT */
        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
    }
    // Master Transmit Address ACK
    else if (u32Status == 0x18)
    {
        LPI2C_SET_DATA(i2c, *(lpi2c0_rw_info.master_xfer_tx_data + lpi2c0_rw_info.xfer_size));
        lpi2c0_rw_info.xfer_size += 1;
        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
    }
    // Master Transmit Address NACK
    else if (u32Status == 0x20)
    {
        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);
        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
        lpi2c0_rw_info.status.busy = 0;
        event = ARM_I2C_EVENT_ADDRESS_NACK;
    }
    // Master Transmit Data ACK
    else if (u32Status == 0x28)
    {
        if (lpi2c0_rw_info.xfer_size != lpi2c0_rw_info.master_xfer_tx_num)
        {
            LPI2C_SET_DATA(i2c, *(lpi2c0_rw_info.master_xfer_tx_data + lpi2c0_rw_info.xfer_size));
            lpi2c0_rw_info.xfer_size += 1;
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
        }
        else
        {
            lpi2c0_rw_info.status.busy = 0;
            event = ARM_I2C_EVENT_TRANSFER_DONE;

            if (lpi2c0_rw_info.xfer_no_stop != 0)
            {
                LPI2C_DisableInt(i2c);
            }
            else
            {
                LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);
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
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
            event = ARM_I2C_EVENT_ARBITRATION_LOST;
        }
        // Bus error
        else if (u32Status == 0x00)
        {
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
            event = ARM_I2C_EVENT_BUS_ERROR;
        }
        else if (u32Status == 0x30)             /* Master transmit data NACK, stop I2C and clear SI */
        {
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);

            if (lpi2c0_rw_info.xfer_remain == 0)
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
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
            event = ARM_I2C_EVENT_ADDRESS_NACK;
        }
        else
        {
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
            event = ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
        }
    }

    if ((event != 0) && (cb_event != NULL))
    {
        cb_event(event | ARM_I2C_EVENT_TRANSFER_DONE);
        lpi2c0_rw_info.status.busy = 0;
    }

    lpi2c0_rw_info.master_prev_status = u32Status;
    u32Status = LPI2C_GET_STATUS(i2c);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  I2C TRx Callback Function                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
static void LPI2C0_SlaveTRx(void)
{
    LPI2C_T *i2c = (LPI2C_T *)LPI2C0;
    RW_Info_t *ptr_rw_info = &lpi2c0_rw_info;
    uint32_t u32Status = LPI2C_GET_STATUS(i2c);
    uint32_t direction = ptr_rw_info->status.direction;
    ARM_I2C_SignalEvent_t cb_event = ptr_rw_info->cb_event;
    uint32_t event = 0;

    // Slave Receive Address ACK
    if (u32Status == 0x60)
    {
        // ptr_rw_info->xfer_size = 0;
        ptr_rw_info->xfer_size_rx = 0;
        ptr_rw_info->xfer_remain_rx = ptr_rw_info->slave_xfer_rx_num;
        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI_AA);
    }
    // Slave Receive Data ACK or Slave Receive Data NACK
    else if ((u32Status == 0x80) || (u32Status == 0x88))
    {
        uint8_t u8Data = (unsigned char) LPI2C_GET_DATA(i2c);
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

        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI_AA);
    }
    // Slave Transmit Address ACK
    else if (u32Status == 0xA8)
    {
        ptr_rw_info->xfer_size_tx = 0;
        ptr_rw_info->xfer_remain_tx = ptr_rw_info->slave_xfer_tx_num;

        if ((direction == 0) && (ptr_rw_info->xfer_remain_tx > 0))   // Direction: 0=Transmitter, 1=Receiver
        {
            LPI2C_SET_DATA(i2c, *(ptr_rw_info->slave_xfer_tx_data + ptr_rw_info->xfer_size_tx));
            ptr_rw_info->xfer_remain_tx -= 1;
            ptr_rw_info->xfer_size_tx += 1;
        }
        else
        {
            LPI2C_SET_DATA(i2c, 0x00); // dummy
        }

        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI_AA);
    }
    // Slave Transmit Data ACK
    else if (u32Status == 0xB8)
    {
        if ((direction == 0) && (ptr_rw_info->xfer_remain_tx > 0))   // Direction: 0=Transmitter, 1=Receiver
        {
            LPI2C_SET_DATA(i2c, *(ptr_rw_info->slave_xfer_tx_data + ptr_rw_info->xfer_size_tx));
            ptr_rw_info->xfer_remain_tx -= 1;
            ptr_rw_info->xfer_size_tx += 1;
        }
        else
        {
            LPI2C_SET_DATA(i2c, 0x00); // dummy
        }

        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI_AA);
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

        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI_AA);
    }
    // Slave Receive Data NACK
    else if (u32Status == 0x88)
    {
        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI_AA);
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

        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI_AA);
    }
    // GC mode Address ACK
    else if (u32Status == 0x70)
    {
        ptr_rw_info->xfer_size_rx = 0;
        ptr_rw_info->xfer_remain_rx = ptr_rw_info->slave_xfer_rx_num;
        event = ARM_I2C_EVENT_GENERAL_CALL;
        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI_AA);
    }
    // GC mode Data ACK
    else if (u32Status == 0x90)
    {
        uint8_t u8Data = (unsigned char) LPI2C_GET_DATA(i2c);
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

        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI_AA);
    }
    // GC mode Data NACK
    else if (u32Status == 0x98)
    {
        LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI_AA);
    }
    else if (u32Status == 0xF8)     /*I2C wave keeps going*/
    {
    }
    else
    {
        // Slave Receive Arbitration Lost
        if (u32Status == 0x68)
        {
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI_AA);
            event = ARM_I2C_EVENT_ARBITRATION_LOST;
        }
        // Address Transmit Arbitration Lost
        else if (u32Status == 0xB0)
        {
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI_AA);
            event = ARM_I2C_EVENT_ARBITRATION_LOST;
        }
        else
        {
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_STO_SI);
            LPI2C_SET_CONTROL_REG(i2c, LPI2C_CTL_SI);
            event = ARM_I2C_EVENT_BUS_ERROR;
        }
    }

    if ((event != 0) && (cb_event != NULL))
    {
        cb_event(event | ARM_I2C_EVENT_TRANSFER_DONE);
        ptr_rw_info->status.busy = 0;
    }

    u32Status = LPI2C_GET_STATUS(i2c);
}


extern \
ARM_DRIVER_I2C Driver_I2C4;
ARM_DRIVER_I2C Driver_I2C4 =
{
    LPI2C0_GetVersion,
    LPI2C0_GetCapabilities,
    LPI2C0_Initialize,
    LPI2C0_Uninitialize,
    LPI2C0_PowerControl,
    LPI2C0_MasterTransmit,
    LPI2C0_MasterReceive,
    LPI2C0_SlaveTransmit,
    LPI2C0_SlaveReceive,
    LPI2C0_GetDataCount,
    LPI2C0_Control,
    LPI2C0_GetStatus
};

#endif // #if (RTE_I2C4 == 1)
