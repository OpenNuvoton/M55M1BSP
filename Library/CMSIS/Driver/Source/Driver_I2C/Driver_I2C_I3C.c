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
    3. Only support I2C Master mode.
    4. There are some limitation for Master transmisson (Transmit & Receive).
       - do not support xfer_pending.
       - do not support gc mode, and only support 7-bit address.
       - for performance consideration, the data buffer is restricted to word alignment.
    5. I3C is designed to prohibit clock stretching by target devices.
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

#if (RTE_I2C6 == 1)

typedef void (*I3C_FUNC)(void);

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
    ARM_I2C_STATUS                status;
    ARM_I2C_SignalEvent_t         cb_event;               // Event callback
    DriverStatus_t                drv_status;             // Driver status
    uint32_t                      xfer_size;              // Requested transfer size (in bytes)
    const    uint32_t *volatile   master_xfer_tx_data;    // Pointer to transmit data (for Master only)
    uint32_t *volatile            master_xfer_rx_data;    // Pointer to receive data (for Master only)
    uint16_t                      master_xfer_tx_num;     // Requested number of bytes to transmit
    uint16_t                      master_xfer_rx_num;     // Requested number of bytes to receive
    // i3c
    uint32_t                      i2c_bus_speed;


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
static ARM_DRIVER_VERSION       I3C0_GetVersion(void);
static ARM_I2C_CAPABILITIES     I3C0_GetCapabilities(void);
static int32_t                  I3C0_Initialize(ARM_I2C_SignalEvent_t cb_event);
static int32_t                  I3C0_Uninitialize();
static int32_t                  I3C0_PowerControl(ARM_POWER_STATE state);
static int32_t                  I3C0_MasterTransmit(uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending);
static int32_t                  I3C0_MasterReceive(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending);
static int32_t                  I3C0_SlaveTransmit(const uint8_t *data, uint32_t num);
static int32_t                  I3C0_SlaveReceive(uint8_t *data, uint32_t num);
static int32_t                  I3C0_GetDataCount(void);
static int32_t                  I3C0_Control(uint32_t control, uint32_t arg);
static ARM_I2C_STATUS           I3C0_GetStatus(void);

static RW_Info_t i3c0_rw_info;


//
//  Functions
//

bool is_aligned(const void *ptr, size_t alignment)
{
    // Ensure alignment is a power of two for bitwise operations
    if (alignment == 0 || (alignment & (alignment - 1)) != 0)
    {
        // Handle invalid alignment values (e.g., return false or assert)
        return false;
    }

    // Using modulo operator
    // return ((uintptr_t)ptr % alignment) == 0;
    // Using bitwise AND (more efficient if alignment is a power of two)
    return (((uintptr_t)ptr) & (alignment - 1)) == 0;
}

static ARM_DRIVER_VERSION I3C0_GetVersion(void)
{
    return DriverVersion;
}

static ARM_I2C_CAPABILITIES I3C0_GetCapabilities(void)
{
    return DriverCapabilities;
}


static ARM_I2C_STATUS I3C0_GetStatus()
{
    return i3c0_rw_info.status;
}

static int32_t I3C0_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
    // Clear run-time info
    memset((void *)&i3c0_rw_info, 0, sizeof(RW_Info_t));
    i3c0_rw_info.cb_event = cb_event;
    i3c0_rw_info.i2c_bus_speed = I3C_DEVI2C_SPEED_I2CFM;
    i3c0_rw_info.drv_status.initialized = 1;
    return ARM_DRIVER_OK;
}

static int32_t I3C0_Uninitialize(void)
{
    // Clear run-time info
    memset((void *)&i3c0_rw_info, 0, sizeof(RW_Info_t));
    i3c0_rw_info.cb_event = NULL;
    i3c0_rw_info.drv_status.initialized = 0;
    return ARM_DRIVER_OK;
}

static int32_t I3C0_PowerControl(ARM_POWER_STATE state)
{
    switch (state)
    {
        case ARM_POWER_OFF:
            CLK->I3CCTL &= ~(CLK_I3CCTL_I3C0CKEN_Msk);
            NVIC_DisableIRQ(I3C0_IRQn);
            i3c0_rw_info.drv_status.powered = 0U;
            break;

        case ARM_POWER_LOW:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_POWER_FULL:
            if (i3c0_rw_info.drv_status.initialized == 0U)
            {
                return ARM_DRIVER_ERROR;
            }

            i3c0_rw_info.drv_status.powered = 1U;
            CLK->I3CCTL |= CLK_I3CCTL_I3C0CKEN_Msk;
            CLK_SetModuleClock(I3C0_MODULE, CLK_I3CSEL_I3C0SEL_HCLK0, 0);
            I3C0->DEVCTLE = 0; // [0] 0: master, 1: slave
            /* Enable all interrupt status */
            I3C0->INTSTSEN = 0xFFFFFFFF;
            /* Set Tx/Rx/CmdQ/ReapQ threshold to 0 */
            I3C0->DBTHCTL = 0x0;
            I3C0->QUETHCTL = 0x0;
            // In Master mode, self-assigns its dynamic address.
            I3C0->DEVADDR = (I3C_DEVADDR_SAVALID_Msk | I3C_DEVADDR_DAVALID_Msk);
            NVIC_EnableIRQ(I3C0_IRQn);
            {
                uint32_t engclk, count;
                engclk = CLK_GetHCLK0Freq();
                count = (engclk / (400 * 1000) / 2);

                if (count < 5)
                {
                    count = 5;
                }

                I3C0->SCLFM = ((count << I3C_SCLFM_FMHCNT_Pos) | (count << I3C_SCLFM_FMLCNT_Pos));
                count = (engclk / (1000 * 1000) / 2);

                if (count < 5)
                {
                    count = 5;
                }

                I3C0->SCLFMP = ((count << I3C_SCLFMP_FMPHCNT_Pos) | (count << I3C_SCLFMP_FMPLCNT_Pos));
                I3C_Enable(I3C0);
            }
            break;

        default:
            return ARM_DRIVER_ERROR_PARAMETER;
    }

    return ARM_DRIVER_OK;
}

__STATIC_INLINE void I3C_FILL_TXFIFO(void)
{
    const uint32_t *tx_data = i3c0_rw_info.master_xfer_tx_data;
    uint32_t xfer_size = i3c0_rw_info.xfer_size;
    uint32_t master_xfer_tx_num = i3c0_rw_info.master_xfer_tx_num;

    while (xfer_size < master_xfer_tx_num)
    {
        if (I3C_IS_TX_FULL(I3C0))
        {
            break;
        }

        I3C0->TXRXDAT = tx_data[xfer_size / 4];
        xfer_size += 4;
    }

    if (xfer_size >= master_xfer_tx_num)
    {
        xfer_size = master_xfer_tx_num;
    }

    i3c0_rw_info.xfer_size = xfer_size;

    if (xfer_size != master_xfer_tx_num)
    {
        I3C0->INTEN |= I3C_INTEN_TXTH_Msk;
    }
    else
    {
        I3C0->INTEN &= ~I3C_INTEN_TXTH_Msk;
    }
}

__STATIC_INLINE void I3C_RETRIEVE_RXFIFO(void)
{
    uint32_t *rx_data = i3c0_rw_info.master_xfer_rx_data;
    uint32_t xfer_size = i3c0_rw_info.xfer_size;
    uint32_t master_xfer_rx_num = i3c0_rw_info.master_xfer_rx_num;
    uint32_t received = (I3C0->DBSTSLV & I3C_DBSTSLV_RXLV_Msk) >> I3C_DBSTSLV_RXLV_Pos;

    while (received)
    {
        if (I3C_IS_TX_FULL(I3C0))
        {
            break;
        }

        rx_data[xfer_size / 4] = I3C0->TXRXDAT;
        xfer_size += 4;
        received--;
    }

    if (xfer_size >= master_xfer_rx_num)
    {
        xfer_size = master_xfer_rx_num;
    }

    i3c0_rw_info.xfer_size = xfer_size;
}

static int32_t I3C0_MasterTransmit(uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)
{
    (void) xfer_pending;

    if ((data == NULL) || (num == 0U) || (addr & ARM_I2C_ADDRESS_10BIT))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (!is_aligned(data, 4))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    I3C0->DEVADR[0] = (addr & 0x7F) | I3C_DEVADR_DEVICE_Msk;
    NVIC_DisableIRQ(I3C0_IRQn);
    I3C0->INTSTSEN = 0xFFFFFFFF;
    I3C0->INTEN = I3C_INTEN_RESPRDY_Msk | I3C_INTEN_TFRABORT_Msk | I3C_INTEN_TFRERR_Msk;
    I3C0->DBTHCTL = 0x3;
    I3C0->QUETHCTL = 0x0;
    uint32_t val = 0;
    i3c0_rw_info.status.mode = 1; // 1=Master
    i3c0_rw_info.status.busy = 1;
    i3c0_rw_info.status.direction = 0; // 0=Transmitter
    i3c0_rw_info.master_xfer_tx_data = (uint32_t *)data;
    i3c0_rw_info.master_xfer_tx_num = num;
    // set xfer_size to 0 for I3C0_GetDataCount
    i3c0_rw_info.xfer_size = 0;
    {
        uint32_t cmdque = ((num << I3C_CMDQUE_DATLEN_Pos) | I3C_CMDATTR_TRANSFER_ARG);
        I3C0->CMDQUE = cmdque;
        I3C_FILL_TXFIFO();
    }
    /* Program transfer command */
    val |= ((0 << I3C_CMDQUE_DEVINDX_Pos)
            | (i3c0_rw_info.i2c_bus_speed & I3C_CMDQUE_SPEED_Msk)
            | (I3C_TX_TID << I3C_CMDQUE_TID_Pos)
            | I3C_CMDQUE_ROC_Msk | I3C_CMDQUE_TOC_Msk
            | I3C_CMDATTR_TRANSFER_CMD);
    I3C0->CMDQUE = val;
    NVIC_EnableIRQ(I3C0_IRQn);
    return ARM_DRIVER_OK;
}

static int32_t I3C0_MasterReceive(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{
    (void) xfer_pending;

    if ((data == NULL) || (num == 0U) || (addr & ARM_I2C_ADDRESS_10BIT))
    {
        // If any parameter is invalid
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (!is_aligned(data, 4))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    uint8_t u8DevIndex = 0;
    I3C0->DEVADR[0] = (addr & 0x7F) | I3C_DEVADR_DEVICE_Msk;
    NVIC_DisableIRQ(I3C0_IRQn);
    I3C0->INTSTSEN = 0xFFFFFFFF;
    I3C0->INTEN = I3C_INTEN_RXTH_Msk | I3C_INTEN_RESPRDY_Msk | I3C_INTEN_TFRABORT_Msk | I3C_INTEN_TFRERR_Msk;
    I3C0->DBTHCTL = 0x2 << I3C_DBTHCTL_RXTH_Pos;
    I3C0->QUETHCTL = 0x0;
    i3c0_rw_info.status.mode = 1; // 1=Master
    i3c0_rw_info.status.busy = 1;
    i3c0_rw_info.status.direction = 1; // 1=Receiver
    i3c0_rw_info.master_xfer_rx_data = (uint32_t *)data;
    i3c0_rw_info.master_xfer_rx_num = num;
    // set xfer_size to 0 for I3C0_GetDataCount
    i3c0_rw_info.xfer_size = 0;
    {
        uint32_t cmdque = ((num << I3C_CMDQUE_DATLEN_Pos) | I3C_CMDATTR_TRANSFER_ARG);
        //
        I3C0->CMDQUE = cmdque;
        cmdque = (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_RNW_Msk | I3C_CMDQUE_ROC_Msk
                  | (i3c0_rw_info.i2c_bus_speed & I3C_CMDQUE_SPEED_Msk)
                  | (u8DevIndex << I3C_CMDQUE_DEVINDX_Pos)
                  | (I3C_RX_TID << I3C_CMDQUE_TID_Pos)
                  | I3C_CMDATTR_TRANSFER_CMD);
        UART_WAIT_TX_EMPTY(UART0);
        I3C0->CMDQUE = cmdque;
        NVIC_EnableIRQ(I3C0_IRQn);
    }
    return ARM_DRIVER_OK;
}

static int32_t I3C0_SlaveTransmit(const uint8_t *data, uint32_t num)
{
    (void) data;
    (void) num;
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t I3C0_SlaveReceive(uint8_t *data, uint32_t num)
{
    (void) data;
    (void) num;
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t I3C0_GetDataCount(void)
{
    return i3c0_rw_info.xfer_size;
}

static int32_t I3C0_Control(uint32_t control, uint32_t arg)
{
    I3C_T *i3c = (I3C_T *)I3C0;

    if (i3c0_rw_info.drv_status.powered == 0U)
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
                return ARM_DRIVER_ERROR_UNSUPPORTED;
            }

            // In Master mode, self-assigns its dynamic address.
            i3c->DEVADDR = (((arg & 0x7F) << I3C_DEVADDR_SA_Pos) | I3C_DEVADDR_SAVALID_Msk);
            break;

        case ARM_I2C_BUS_SPEED:
            switch (arg)
            {
                case ARM_I2C_BUS_SPEED_FAST:
                    i3c0_rw_info.i2c_bus_speed = I3C_DEVI2C_SPEED_I2CFM;
                    break;

                case ARM_I2C_BUS_SPEED_FAST_PLUS:
                    i3c0_rw_info.i2c_bus_speed = I3C_DEVI2C_SPEED_I2CFMPLUS;
                    break;

                case ARM_I2C_BUS_SPEED_STANDARD:
                default:
                    return ARM_DRIVER_ERROR_UNSUPPORTED;
            }

            break;

        case ARM_I2C_BUS_CLEAR:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_I2C_ABORT_TRANSFER:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

// End I2C Interface


volatile uint32_t   g_RespQ[I3C_DEVICE_RESP_QUEUE_CNT];
volatile uint32_t   g_RxBuf[I3C_DEVICE_RX_BUF_CNT], g_TxBuf[I3C_DEVICE_RX_BUF_CNT];
volatile uint32_t   g_u32IntSelMask = 0, g_u32IntOccurredMask = 0;
volatile uint32_t   g_u32RespStatus = I3C_STS_NO_ERR;


void I3C0_IRQHandler(void)
{
    g_u32IntOccurredMask = I3C0->INTSTS;
    // ignore the interrupts we didn't ask for
    g_u32IntSelMask = g_u32IntOccurredMask & I3C0->INTEN;

    if (g_u32IntSelMask & I3C_INTEN_TX_EMPTY_THLD)
    {
        I3C_FILL_TXFIFO();
    }

    if (g_u32IntSelMask & I3C_INTEN_RX_THLD)
    {
        I3C_RETRIEVE_RXFIFO();
    }

    if (g_u32IntSelMask & I3C_INTEN_RESPQ_READY)
    {
        ARM_I2C_SignalEvent_t cb_event = i3c0_rw_info.cb_event;
        uint32_t cmd_response = I3C0->RESPQUE;
        (void)cmd_response;
        I3C_RETRIEVE_RXFIFO();

        if (cb_event != NULL)
        {
            cb_event(ARM_I2C_EVENT_TRANSFER_DONE);
        }

        i3c0_rw_info.status.busy = 0;
        I3C0->INTEN &= ~(I3C_INTEN_TXTH_Msk | I3C_INTEN_RXTH_Msk);
    }

    if (g_u32IntSelMask & I3C_INTEN_TRANSFER_ERR)
    {
        ARM_I2C_SignalEvent_t cb_event = i3c0_rw_info.cb_event;
        I3C_CLEAR_TRANSFER_ERR_STATUS(I3C0);
        I3C0->INTEN &= ~(I3C_INTEN_TXTH_Msk | I3C_INTEN_RXTH_Msk);

        if (cb_event != NULL)
        {
            cb_event(ARM_I2C_EVENT_TRANSFER_DONE);
        }

        i3c0_rw_info.status.busy = 0;
    }
}

extern \
ARM_DRIVER_I2C Driver_I2C6;
ARM_DRIVER_I2C Driver_I2C6 =
{
    I3C0_GetVersion,
    I3C0_GetCapabilities,
    I3C0_Initialize,
    I3C0_Uninitialize,
    I3C0_PowerControl,
    I3C0_MasterTransmit,
    I3C0_MasterReceive,
    I3C0_SlaveTransmit,
    I3C0_SlaveReceive,
    I3C0_GetDataCount,
    I3C0_Control,
    I3C0_GetStatus
};

#endif // #if (RTE_I2C6 == 1)
