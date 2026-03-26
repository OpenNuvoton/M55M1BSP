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
    3. If the master's data transfer exceeds the specified size, UI2C_SlaveReceive will discard any extra received data.
    4. If the master's data request exceeds the specified size, UI2C_SlaveTransmit will send dummy data (all zeros).
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

#if (RTE_I2C5 == 1)

#if defined (RTE_Driver_USART) || defined (RTE_Driver_SPI)
    #if (RTE_USART14 || RTE_SPI7)
        #error "USCI0 is used by multiple CMSIS Drivers! Please check RTE device configuration to fix it."
    #endif
#endif

#define ARM_I2C_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */


#define SLV_10BIT_ADDR (0x1E<<2)             //1111+0xx+r/w

typedef void (*UI2C_FUNC)(void);

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
    UI2C_FUNC                     i2c_handler_fn;
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
    volatile enum UI2C_MASTER_EVENT master_event;
    volatile enum UI2C_SLAVE_EVENT  slave_event;
} RW_Info_t;



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
static ARM_DRIVER_VERSION       UI2C_GetVersion(void);
static ARM_I2C_CAPABILITIES     UI2C_GetCapabilities(void);
static int32_t                  UI2C_Initialize(ARM_I2C_SignalEvent_t cb_event);
static int32_t                  UI2C_Uninitialize();
static int32_t                  UI2C_PowerControl(ARM_POWER_STATE state);
static int32_t                  UI2C_MasterTransmit(uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending);
static int32_t                  UI2C_MasterReceive(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending);
static int32_t                  UI2C_SlaveTransmit(const uint8_t *data, uint32_t num);
static int32_t                  UI2C_SlaveReceive(uint8_t *data, uint32_t num);
static int32_t                  UI2C_GetDataCount(void);
static int32_t                  UI2C_Control(uint32_t control, uint32_t arg);
static ARM_I2C_STATUS           UI2C_GetStatus(void);
static void                     UI2C_MasterTx(void);
static void                     UI2C_MasterRx(void);
static void                     UI2C_SlaveTRx(void);

static RW_Info_t ui2c0_rw_info;

//
//  Functions
//

static ARM_DRIVER_VERSION UI2C_GetVersion(void)
{
    return DriverVersion;
}

static ARM_I2C_CAPABILITIES UI2C_GetCapabilities(void)
{
    return DriverCapabilities;
}

static int32_t UI2C_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
    // Clear run-time info
    memset((void *)&ui2c0_rw_info, 0, sizeof(RW_Info_t));
    ui2c0_rw_info.cb_event = cb_event;
    ui2c0_rw_info.drv_status.initialized = 1;
    return ARM_DRIVER_OK;
}

static int32_t UI2C_Uninitialize(void)
{
    // Clear run-time info
    memset((void *)&ui2c0_rw_info, 0, sizeof(RW_Info_t));
    ui2c0_rw_info.cb_event = NULL;
    ui2c0_rw_info.drv_status.initialized = 0;
    return ARM_DRIVER_OK;
}

static int32_t UI2C_PowerControl(ARM_POWER_STATE state)
{
    switch (state)
    {
        case ARM_POWER_OFF:
            CLK->USCICTL &= ~CLK_USCICTL_USCI0CKEN_Msk;
            NVIC_DisableIRQ(USCI0_IRQn);
            ui2c0_rw_info.drv_status.powered = 0U;
            break;

        case ARM_POWER_LOW:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_POWER_FULL:
            if (ui2c0_rw_info.drv_status.initialized == 0U)
            {
                return ARM_DRIVER_ERROR;
            }

            ui2c0_rw_info.drv_status.powered = 1U;
            CLK->USCICTL |= CLK_USCICTL_USCI0CKEN_Msk;
            UI2C_Open(UI2C0, 100000);
            NVIC_EnableIRQ(USCI0_IRQn);
            break;
    }

    return ARM_DRIVER_OK;
}

static int32_t UI2C_MasterTransmit(uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)
{
    ARM_I2C_STATUS status;
    RW_Info_t *ptr_rw_info = &ui2c0_rw_info;

    if ((data == NULL) || (num == 0U) || (num > (uint32_t)UINT16_MAX) ||
            ((addr & ~((uint32_t)ARM_I2C_ADDRESS_10BIT | (uint32_t)ARM_I2C_ADDRESS_GC)) > 0x3FFU))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (addr & ARM_I2C_ADDRESS_10BIT)
    {
        ptr_rw_info->master_xfer_addrL = addr & 0xFF;
        ptr_rw_info->master_xfer_addrH = (((addr >> 8) & 0x03) | SLV_10BIT_ADDR);
    }
    else
    {
        ptr_rw_info->master_xfer_addrL = addr & 0x7F;
        ptr_rw_info->master_xfer_addrH = 0;
    }

    ui2c0_rw_info.master_xfer_tx_data = data;
    ui2c0_rw_info.master_xfer_tx_num = num;
    ui2c0_rw_info.xfer_no_stop = xfer_pending;
    ui2c0_rw_info.i2c_handler_fn = UI2C_MasterTx;
    // set xfer_size to 0 for I2Cn_GetDataCount
    ui2c0_rw_info.xfer_size = 0;
    //
    status.busy = 1;
    status.mode = 1; // 1=Master
    status.direction = 0; // 0=Transmitter, 1=Receiver
    ui2c0_rw_info.status = status;
    ui2c0_rw_info.master_event = MASTER_SEND_START;
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk
                                 | UI2C_PROTIEN_ARBLOIEN_Msk | UI2C_PROTIEN_ERRIEN_Msk));
    UI2C0->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;
    UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STA);
    UI2C_START(UI2C0);                                                       /* Send START */
    return ARM_DRIVER_OK;
}

static int32_t UI2C_MasterReceive(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{
    ARM_I2C_STATUS status;
    RW_Info_t *ptr_rw_info = &ui2c0_rw_info;

    if ((data == NULL) || (num == 0U) || (num > (uint32_t)UINT16_MAX) ||
            ((addr & ~((uint32_t)ARM_I2C_ADDRESS_10BIT | (uint32_t)ARM_I2C_ADDRESS_GC)) > 0x3FFU))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (addr & ARM_I2C_ADDRESS_10BIT)
    {
        ptr_rw_info->master_xfer_addrL = addr & 0xFF;
        ptr_rw_info->master_xfer_addrH = (((addr >> 8) & 0x03) | SLV_10BIT_ADDR);
    }
    else
    {
        ptr_rw_info->master_xfer_addrL = addr & 0x7F;
        ptr_rw_info->master_xfer_addrH = 0;
    }

    ptr_rw_info->master_xfer_rx_data = data;
    ptr_rw_info->master_xfer_rx_num = num;
    ptr_rw_info->xfer_no_stop = xfer_pending;
    ptr_rw_info->i2c_handler_fn = UI2C_MasterRx;
    // set xfer_size to 0 for I2Cn_GetDataCount
    ptr_rw_info->xfer_size = 0;
    //
    status.busy = 1;
    status.mode = 1; // 1=Master
    status.direction = (addr > 0x80) ? 0 : 1; // 0=Transmitter, 1=Receiver
    ptr_rw_info->status = status;
    ptr_rw_info->master_event = MASTER_SEND_START;
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk
                                 | UI2C_PROTIEN_ARBLOIEN_Msk | UI2C_PROTIEN_ERRIEN_Msk));
    UI2C0->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;
    UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_STA);
    return ARM_DRIVER_OK;
}

static int32_t UI2C_SlaveTransmit(const uint8_t *data, uint32_t num)
{
    ARM_I2C_STATUS status;
    RW_Info_t *ptr_rw_info = &ui2c0_rw_info;

    if ((data == NULL) || (num == 0U))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    ptr_rw_info->slave_xfer_tx_data = data;
    ptr_rw_info->slave_xfer_tx_num = num;
    ptr_rw_info->i2c_handler_fn = UI2C_SlaveTRx;
    //
    status.busy = 1;
    status.mode = 0; // 0=Slave
    status.direction = 0; // 0=Transmitter, 1=Receiver
    ptr_rw_info->status = status;
    ptr_rw_info->slave_event = SLAVE_ADDRESS_ACK;
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk
                                 | UI2C_PROTIEN_ARBLOIEN_Msk | UI2C_PROTIEN_ERRIEN_Msk));
    UI2C0->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;
    UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    return ARM_DRIVER_OK;
}

static int32_t UI2C_SlaveReceive(uint8_t *data, uint32_t num)
{
    ARM_I2C_STATUS status;
    RW_Info_t *ptr_rw_info = &ui2c0_rw_info;

    if ((data == NULL) || (num == 0U))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    ptr_rw_info->slave_xfer_rx_data = data;
    ptr_rw_info->slave_xfer_rx_num = num;
    ptr_rw_info->i2c_handler_fn = UI2C_SlaveTRx;
    //
    status.busy = 1;
    status.mode = 0; // 0=Slave
    status.direction = 1; // 0=Transmitter, 1=Receiver
    ptr_rw_info->status = status;
    ptr_rw_info->slave_event = SLAVE_ADDRESS_ACK;
    UI2C_ENABLE_PROT_INT(UI2C0, (UI2C_PROTIEN_ACKIEN_Msk | UI2C_PROTIEN_NACKIEN_Msk | UI2C_PROTIEN_STORIEN_Msk | UI2C_PROTIEN_STARIEN_Msk
                                 | UI2C_PROTIEN_ARBLOIEN_Msk | UI2C_PROTIEN_ERRIEN_Msk));
    UI2C0->PROTCTL |=  UI2C_PROTCTL_PROTEN_Msk;
    UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    return ARM_DRIVER_OK;
}

static int32_t UI2C_GetDataCount(void)
{
    return ui2c0_rw_info.xfer_size;
}

static int32_t UI2C_Control(uint32_t control, uint32_t arg)
{
    switch (control)
    {
        case ARM_I2C_OWN_ADDRESS:
            if ((arg & ARM_I2C_ADDRESS_10BIT) != 0U)
            {
                // Own address is a 10-bit address
                UI2C_ENABLE_10BIT_ADDR_MODE(UI2C0);
            }
            else
            {
                // Own address is a 7-bit address
                UI2C_DISABLE_10BIT_ADDR_MODE(UI2C0);
            }

            if ((arg & ARM_I2C_ADDRESS_GC) != 0U)
            {
                UI2C_SetSlaveAddr(UI2C0, 0, (arg & 0x3FF), UI2C_GCMODE_ENABLE);
            }
            else
            {
                // Disable general call
                UI2C_SetSlaveAddr(UI2C0, 0, (arg & 0x3FF), UI2C_GCMODE_DISABLE);
            }

            break;

        case ARM_I2C_BUS_SPEED:
            switch (arg)
            {
                case ARM_I2C_BUS_SPEED_STANDARD:
                    UI2C_Open(UI2C0, 100000);
                    break;

                case ARM_I2C_BUS_SPEED_FAST:
                    UI2C_Open(UI2C0, 400000);
                    break;

                case ARM_I2C_BUS_SPEED_FAST_PLUS:
                    UI2C_Open(UI2C0, 1000000);
                    break;

                default:
                    return ARM_DRIVER_ERROR_UNSUPPORTED;
            }

            break;

        case ARM_I2C_BUS_CLEAR:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_I2C_ABORT_TRANSFER:
            ui2c0_rw_info.xfer_abort = 1;
            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

static ARM_I2C_STATUS UI2C_GetStatus(void)
{
    return ui2c0_rw_info.status;
}

// End I2C Interface


//void USCI0_IRQHandler(void)
void USCI0_IRQHandler(void)
{
    UI2C_FUNC func = ui2c0_rw_info.i2c_handler_fn;

    if (func != NULL)
    {
        func();
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
static void UI2C_MasterRx(void)
{
    UI2C_T *i2c = (UI2C_T *)UI2C0;
    RW_Info_t *ptr_rw_info = &ui2c0_rw_info;
    uint32_t u32Status = UI2C_GET_PROT_STATUS(UI2C0);
    ARM_I2C_SignalEvent_t cb_event = ptr_rw_info->cb_event;
    uint32_t event = 0;

    if (ptr_rw_info->xfer_abort == 1)
    {
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));
        ptr_rw_info->xfer_abort = 0;
        //
        ptr_rw_info->status.busy = 0;

        if (cb_event != NULL)
        {
            cb_event(ARM_I2C_EVENT_TRANSFER_DONE | ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
        }

        return;
    }

    if (UI2C_GET_TIMEOUT_FLAG(UI2C0))
    {
        /* Clear USCI_I2C0 Timeout Flag */
        UI2C_ClearTimeoutFlag(UI2C0);
    }
    else if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        uint8_t u8DeviceHAddr = ptr_rw_info->master_xfer_addrH;
        uint8_t u8DeviceLAddr = ptr_rw_info->master_xfer_addrL;
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk); /* Clear START INT Flag */
        ptr_rw_info->xfer_remain = ptr_rw_info->master_xfer_rx_num;

        if (ptr_rw_info->master_event == MASTER_SEND_START)
        {
            if (u8DeviceHAddr)
            {
                UI2C_SET_DATA(i2c, (u8DeviceHAddr << 1)); /* Write SLA+W to Register I2CDAT */
                ptr_rw_info->master_event = MASTER_SEND_H_WR_ADDRESS;
            }
            else
            {
                UI2C_SET_DATA(i2c, (u8DeviceLAddr << 1) | 0x01); /* Write SLA+R to Register TXDAT */
                ptr_rw_info->master_event = MASTER_SEND_ADDRESS;
            }
        }
        else if (ptr_rw_info->master_event == MASTER_SEND_REPEAT_START)
        {
            UI2C_SET_DATA(i2c, (u8DeviceHAddr << 1) | 0x01); /* Write SLA+R to Register TXDAT */
            ptr_rw_info->master_event = MASTER_SEND_ADDRESS;
        }

        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);  /* Clear ACK INT Flag */
        uint8_t u8DeviceLAddr = ptr_rw_info->master_xfer_addrL;

        if (ptr_rw_info->master_event == MASTER_SEND_H_WR_ADDRESS)
        {
            UI2C_SET_DATA(UI2C0, u8DeviceLAddr);
            ptr_rw_info->master_event = MASTER_SEND_L_ADDRESS;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else if (ptr_rw_info->master_event == MASTER_SEND_L_ADDRESS)
        {
            ptr_rw_info->master_event = MASTER_SEND_REPEAT_START;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STA));
        }
        else if (ptr_rw_info->master_event == MASTER_SEND_ADDRESS)
        {
            if (ptr_rw_info->master_xfer_rx_num == 1)
            {
                UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_PTRG);
            }
            else
            {
                UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_PTRG | UI2C_CTL_AA);
            }

            ptr_rw_info->master_event = MASTER_READ_DATA;
        }
        else if (ptr_rw_info->master_event == MASTER_READ_DATA)
        {
            uint8_t u8Data = UI2C_GET_DATA(i2c);

            if (ptr_rw_info->xfer_remain > 0)
            {
                *(ptr_rw_info->master_xfer_rx_data + ptr_rw_info->xfer_size) = u8Data;
                ptr_rw_info->xfer_size += 1;
                ptr_rw_info->xfer_remain -= 1;
            }

            if (ptr_rw_info->xfer_remain > 1)
            {
                UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_PTRG | UI2C_CTL_AA);
            }
            else if (ptr_rw_info->xfer_remain == 1)
            {
                UI2C_SET_CONTROL_REG(i2c, UI2C_CTL_PTRG);
            }
            else
            {
                ptr_rw_info->status.busy = 0;
                event = ARM_I2C_EVENT_TRANSFER_DONE;

                if (ptr_rw_info->xfer_no_stop != 0)
                {
                    i2c->PROTCTL &= ~UI2C_PROTCTL_PROTEN_Msk;
                    UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
                }
                else
                {
                    UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));
                }
            }
        }
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk); /* Clear NACK INT Flag */

        if (ptr_rw_info->master_event != MASTER_READ_DATA)
        {
            event = ARM_I2C_EVENT_ADDRESS_NACK;
            UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));
        }
        else
        {
            uint8_t u8Data = UI2C_GET_DATA(i2c);

            if (ptr_rw_info->xfer_remain > 0)
            {
                *(ptr_rw_info->master_xfer_rx_data + ptr_rw_info->xfer_size) = u8Data;
                ptr_rw_info->xfer_size += 1;
                ptr_rw_info->xfer_remain -= 1;
            }

            if (ptr_rw_info->xfer_remain == 0)
            {
                event = ARM_I2C_EVENT_TRANSFER_DONE;

                if (ptr_rw_info->xfer_no_stop != 0)
                {
                    i2c->PROTCTL &= ~UI2C_PROTCTL_PROTEN_Msk;
                    UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
                }
                else
                {
                    UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));
                }
            }
            else
            {
                event = ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
                UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));
            }
        }
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);  /* Clear STOP INT Flag */
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if ((u32Status & UI2C_PROTSTS_ARBLOIF_Msk) == UI2C_PROTSTS_ARBLOIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ARBLOIF_Msk);  /* Clear Arbitration Lost INT Flag */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));
        event = ARM_I2C_EVENT_ARBITRATION_LOST;
    }
    else if ((u32Status & UI2C_PROTSTS_ERRIF_Msk) == UI2C_PROTSTS_ERRIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ERRIF_Msk);  /* Clear Error INT Flag */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));
        event = ARM_I2C_EVENT_BUS_ERROR;
    }

    if ((event != 0) && (cb_event != NULL))
    {
        cb_event(event | ARM_I2C_EVENT_TRANSFER_DONE);
        ptr_rw_info->status.busy = 0;
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
static void UI2C_MasterTx(void)
{
    UI2C_T *i2c = (UI2C_T *)UI2C0;
    RW_Info_t *ptr_rw_info = &ui2c0_rw_info;
    uint32_t u32Status = UI2C_GET_PROT_STATUS(UI2C0);
    ARM_I2C_SignalEvent_t cb_event = ptr_rw_info->cb_event;
    uint32_t event = 0;
    uint8_t u8Data = 0;

    if (ptr_rw_info->xfer_abort == 1)
    {
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));
        ptr_rw_info->xfer_abort = 0;
        //
        ptr_rw_info->status.busy = 0;

        if (cb_event != NULL)
        {
            cb_event(ARM_I2C_EVENT_TRANSFER_DONE | ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
        }

        return;
    }

    if (UI2C_GET_TIMEOUT_FLAG(UI2C0))
    {
        /* Clear USCI_I2C0 Timeout Flag */
        UI2C_ClearTimeoutFlag(UI2C0);
    }
    else if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);    /* Clear START INT Flag */
        uint8_t u8DeviceHAddr = ptr_rw_info->master_xfer_addrH;
        uint8_t u8DeviceLAddr = ptr_rw_info->master_xfer_addrL;
        ptr_rw_info->xfer_size = 0;

        if (u8DeviceHAddr)
        {
            UI2C_SET_DATA(i2c, (u8DeviceHAddr << 1)); /* Write SLA+W to Register I2CDAT */
            ptr_rw_info->master_event = MASTER_SEND_H_WR_ADDRESS;
        }
        else
        {
            UI2C_SET_DATA(i2c, (u8DeviceLAddr << 1)); /* Write SLA+W to Register I2CDAT */
            ptr_rw_info->master_event = MASTER_SEND_L_ADDRESS;
        }

        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);   /* Clear ACK INT Flag */

        /* Event process */
        if (ptr_rw_info->master_event == MASTER_SEND_H_WR_ADDRESS)
        {
            uint8_t u8DeviceLAddr = ptr_rw_info->master_xfer_addrL;
            UI2C_SET_DATA(UI2C0, u8DeviceLAddr);
            ptr_rw_info->master_event = MASTER_SEND_L_ADDRESS;
            UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
        }
        else
        {
            ptr_rw_info->master_event = MASTER_SEND_DATA;

            if (ptr_rw_info->xfer_size != ptr_rw_info->master_xfer_tx_num)
            {
                u8Data = *(ptr_rw_info->master_xfer_tx_data + ptr_rw_info->xfer_size);
                UI2C_SET_DATA(i2c, u8Data);
                ptr_rw_info->xfer_size += 1;
                UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
            }
            else
            {
                ptr_rw_info->status.busy = 0;
                event = ARM_I2C_EVENT_TRANSFER_DONE;

                if (ptr_rw_info->xfer_no_stop != 0)
                {
                    i2c->PROTCTL &= ~UI2C_PROTCTL_PROTEN_Msk;
                    UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
                }
                else
                {
                    UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));
                }
            }
        }
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk); /* Clear NACK INT Flag */

        if (ptr_rw_info->master_event != MASTER_SEND_DATA)
        {
            /* SLA+W has been transmitted and NACK has been received */
            event = ARM_I2C_EVENT_ADDRESS_NACK;
        }
        else
        {
            event = ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
        }

        if (cb_event != NULL)
        {
            cb_event(event | ARM_I2C_EVENT_TRANSFER_DONE);
        }

        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);  /* Clear STOP INT Flag */
        UI2C_SET_CONTROL_REG(UI2C0, UI2C_CTL_PTRG);
    }
    else if ((u32Status & UI2C_PROTSTS_ARBLOIF_Msk) == UI2C_PROTSTS_ARBLOIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ARBLOIF_Msk);  /* Clear Arbitration Lost INT Flag */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));
        event = ARM_I2C_EVENT_ARBITRATION_LOST;
    }
    else if ((u32Status & UI2C_PROTSTS_ERRIF_Msk) == UI2C_PROTSTS_ERRIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ERRIF_Msk);  /* Clear Error INT Flag */
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_STO));
        event = ARM_I2C_EVENT_BUS_ERROR;
    }

    if ((event != 0) && (cb_event != NULL))
    {
        cb_event(event | ARM_I2C_EVENT_TRANSFER_DONE);
        ptr_rw_info->status.busy = 0;
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/*  I2C TRx Callback Function                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
static void UI2C_SlaveTRx(void)
{
    UI2C_T *i2c = (UI2C_T *)UI2C0;
    RW_Info_t *ptr_rw_info = &ui2c0_rw_info;
    uint32_t u32Status = UI2C_GET_PROT_STATUS(UI2C0);
    uint32_t direction = ptr_rw_info->status.direction;
    ARM_I2C_SignalEvent_t cb_event = ptr_rw_info->cb_event;
    uint32_t event = 0;
    uint8_t u8Data;

    if ((u32Status & UI2C_PROTSTS_STARIF_Msk) == UI2C_PROTSTS_STARIF_Msk)
    {
        /* Clear START INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STARIF_Msk);
        /* Event process */
        ptr_rw_info->slave_event = SLAVE_ADDRESS_ACK;
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_ACKIF_Msk) == UI2C_PROTSTS_ACKIF_Msk)
    {
        /* Clear ACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ACKIF_Msk);

        /* Event process */
        if (ptr_rw_info->slave_event == SLAVE_ADDRESS_ACK)
        {
            if ((UI2C0->PROTSTS & UI2C_PROTSTS_SLAREAD_Msk) == UI2C_PROTSTS_SLAREAD_Msk)
            {
                /* Own SLA+R has been receive; ACK has been return */
                ptr_rw_info->xfer_size_tx = 0;
                ptr_rw_info->xfer_remain_tx = ptr_rw_info->slave_xfer_tx_num;
                ptr_rw_info->slave_event = SLAVE_SEND_DATA;

                if ((direction == 0) && (ptr_rw_info->xfer_remain_tx > 0))   // Direction: 0=Transmitter, 1=Receiver
                {
                    u8Data = *(ptr_rw_info->slave_xfer_tx_data + ptr_rw_info->xfer_size_tx);
                    UI2C_SET_DATA(i2c, u8Data);
                    ptr_rw_info->xfer_remain_tx -= 1;
                    ptr_rw_info->xfer_size_tx += 1;
                }
                else
                {
                    UI2C_SET_DATA(i2c, 0x00); // dummy
                }
            }
            else
            {
                /* Own SLA+W has been receive; ACK has been return */
                ptr_rw_info->xfer_size_rx = 0;
                ptr_rw_info->xfer_remain_rx = ptr_rw_info->slave_xfer_rx_num;

                if (i2c->PROTCTL & UI2C_PROTCTL_ADDR10EN_Msk)
                {
                    ptr_rw_info->slave_event = SLAVE_GET_DATA;
                    // Address Byte (High)
                    u8Data = (uint8_t)UI2C_GET_DATA(UI2C0);
                }
                else
                {
                    ptr_rw_info->slave_event = SLAVE_GET_DATA;
                }
            }

            // Address Byte (Low)
            u8Data = (uint8_t)UI2C_GET_DATA(UI2C0);

            if ((u8Data == 0) && ptr_rw_info->slave_event == SLAVE_GET_DATA)
            {
                event = ARM_I2C_EVENT_GENERAL_CALL;
            }
        }
        else if (ptr_rw_info->slave_event == SLAVE_L_WR_ADDRESS_ACK)
        {
            // Address Byte (Low)
            // u8Data = (uint8_t)UI2C_GET_DATA(UI2C0);
            ptr_rw_info->slave_event = SLAVE_GET_DATA;
        }
        else if (ptr_rw_info->slave_event == SLAVE_GET_DATA)
        {
            u8Data = UI2C_GET_DATA(UI2C0);
            ptr_rw_info->xfer_size += 1;

            if (direction == 1)   // Direction: 0=Transmitter, 1=Receiver
            {
                if (ptr_rw_info->xfer_remain_rx > 0)
                {
                    *(ptr_rw_info->slave_xfer_rx_data + ptr_rw_info->xfer_size_rx) = u8Data;
                    ptr_rw_info->xfer_remain_rx -= 1;
                    ptr_rw_info->xfer_size_rx += 1;

                    if (ptr_rw_info->xfer_remain_rx == 0)
                    {
                        event = ARM_I2C_EVENT_TRANSFER_DONE;
                    }
                }
            }
        }
        else if (ptr_rw_info->slave_event == SLAVE_SEND_DATA)
        {
            if ((direction == 0) && (ptr_rw_info->xfer_remain_tx > 0))   // Direction: 0=Transmitter, 1=Receiver
            {
                u8Data = *(ptr_rw_info->slave_xfer_tx_data + ptr_rw_info->xfer_size_tx);
                UI2C_SET_DATA(i2c, u8Data);
                ptr_rw_info->xfer_remain_tx -= 1;
                ptr_rw_info->xfer_size_tx += 1;

                if (ptr_rw_info->xfer_remain_tx == 0)
                {
                    event = ARM_I2C_EVENT_TRANSFER_DONE;
                }
            }
            else
            {
                UI2C_SET_DATA(i2c, 0x00); // dummy
            }
        }

        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_NACKIF_Msk) == UI2C_PROTSTS_NACKIF_Msk)
    {
        /* Clear NACK INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_NACKIF_Msk);

        if (ptr_rw_info->slave_event == SLAVE_GET_DATA)
        {
            u8Data = UI2C_GET_DATA(UI2C0);
            ptr_rw_info->xfer_size += 1;

            if (direction == 1)   // Direction: 0=Transmitter, 1=Receiver
            {
                if (ptr_rw_info->xfer_remain_rx > 0)
                {
                    *(ptr_rw_info->slave_xfer_rx_data + ptr_rw_info->xfer_size_rx) = u8Data;
                    ptr_rw_info->xfer_remain_rx -= 1;
                    ptr_rw_info->xfer_size_rx += 1;

                    if (ptr_rw_info->xfer_remain_rx == 0)
                    {
                        event = ARM_I2C_EVENT_TRANSFER_DONE;
                    }
                }
            }
        }
        else if (ptr_rw_info->slave_event == SLAVE_SEND_DATA)
        {
            // Transfer Done. Received NACK
        }

        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_STORIF_Msk) == UI2C_PROTSTS_STORIF_Msk)
    {
        /* Clear STOP INT Flag */
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_STORIF_Msk);
        UI2C_SET_CONTROL_REG(UI2C0, (UI2C_CTL_PTRG | UI2C_CTL_AA));
    }
    else if ((u32Status & UI2C_PROTSTS_ARBLOIF_Msk) == UI2C_PROTSTS_ARBLOIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ARBLOIF_Msk);  /* Clear Arbitration Lost INT Flag */
        event = ARM_I2C_EVENT_ARBITRATION_LOST;
    }
    else if ((u32Status & UI2C_PROTSTS_ERRIF_Msk) == UI2C_PROTSTS_ERRIF_Msk)
    {
        UI2C_CLR_PROT_INT_FLAG(UI2C0, UI2C_PROTSTS_ERRIF_Msk);  /* Clear Error INT Flag */
        event = ARM_I2C_EVENT_BUS_ERROR;
    }

    if ((event != 0) && (cb_event != NULL))
    {
        if (event & ARM_I2C_EVENT_GENERAL_CALL)
        {
            cb_event(event);
        }
        else
        {
            cb_event(event | ARM_I2C_EVENT_TRANSFER_DONE);
            ptr_rw_info->status.busy = 0;
        }
    }
}


extern \
ARM_DRIVER_I2C Driver_I2C5;
ARM_DRIVER_I2C Driver_I2C5 =
{
    UI2C_GetVersion,
    UI2C_GetCapabilities,
    UI2C_Initialize,
    UI2C_Uninitialize,
    UI2C_PowerControl,
    UI2C_MasterTransmit,
    UI2C_MasterReceive,
    UI2C_SlaveTransmit,
    UI2C_SlaveReceive,
    UI2C_GetDataCount,
    UI2C_Control,
    UI2C_GetStatus
};

#endif // #if (RTE_I2C5 == 1)
