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

/* The implementation does not include pin/clock settings, which must be configured in the application */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"
#include "Driver_ETH_MAC.h"
#include "synopGMAC_network_interface.h"

uint8_t *EMAC_AllocatePktBuf(void);
void EMAC_Open(uint8_t *macaddr);
void EMAC_Close(void);
int32_t EMAC_TransmitPkt(uint8_t *pbuf, uint32_t len);

static synopGMACdevice GMACdev = {0};

#if (NVT_DCACHE_ON == 1)
/* Descriptor and data buffer are placed in a non-cacheable region */
NVT_NONCACHEABLE static DmaDesc tx_desc[TRANSMIT_DESC_SIZE] __attribute__((aligned(32))) = {0};
NVT_NONCACHEABLE static DmaDesc rx_desc[RECEIVE_DESC_SIZE] __attribute__((aligned(32))) = {0};
NVT_NONCACHEABLE static PKT_FRAME_T tx_buf[TRANSMIT_DESC_SIZE] __attribute__((aligned(32))) = {0};
NVT_NONCACHEABLE static PKT_FRAME_T rx_buf[RECEIVE_DESC_SIZE] __attribute__((aligned(32))) = {0};
#else
static DmaDesc tx_desc[TRANSMIT_DESC_SIZE] __attribute__((aligned(32))) = {0};
static DmaDesc rx_desc[RECEIVE_DESC_SIZE] __attribute__((aligned(32))) = {0};
static PKT_FRAME_T tx_buf[TRANSMIT_DESC_SIZE] __attribute__((aligned(32))) = {0};
static PKT_FRAME_T rx_buf[RECEIVE_DESC_SIZE] __attribute__((aligned(32))) = {0};
#endif

#define ARM_ETH_MAC_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */

uint8_t g_au8MacAddr[6] = DEFAULT_MAC0_ADDRESS;
static PKT_FRAME_T *psPktFrame;

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion =
{
    ARM_ETH_MAC_API_VERSION,
    ARM_ETH_MAC_DRV_VERSION
};

/* Driver status */
typedef struct
{
    uint8_t                       initialized  : 1;         // Initialized status: 0 - not initialized, 1 - initialized
    uint8_t                       powered      : 1;         // Power status:       0 - not powered,     1 - powered
    uint8_t                       reserved     : 6;         // Reserved
} DriverStatus_t;

/* Run-time information (RW) */
typedef struct
{
    ARM_ETH_MAC_SignalEvent_t       cb_event;               // Event callback
    DriverStatus_t                  drv_status;             // Driver status
    uint32_t                        tx_len;                 // Transmit frame length
    uint32_t                        rx_len;                 // Receive frame length
    uint32_t                        offload_needed;         // Checksum offload needed
    uint32_t                        mac_tx_enabled;         // MAC transmit enabled
    uint32_t                        mac_tx_event;           // tx cb_event enabled
} RW_Info_t;

static       RW_Info_t          eth_mac0_rw_info;

/* Local functions prototypes */
static ARM_DRIVER_VERSION       ARM_ETH_MAC_GetVersion(void);
static ARM_ETH_MAC_CAPABILITIES ARM_ETH_MAC_GetCapabilities(void);
static int32_t                  ARM_ETH_MAC_Initialize(ARM_ETH_MAC_SignalEvent_t cb_event);
static int32_t                  ARM_ETH_MAC_Uninitialize(void);
static int32_t                  ARM_ETH_MAC_PowerControl(ARM_POWER_STATE state);
static int32_t                  ARM_ETH_MAC_GetMacAddress(ARM_ETH_MAC_ADDR *ptr_addr);
static int32_t                  ARM_ETH_MAC_SetMacAddress(const ARM_ETH_MAC_ADDR *ptr_addr);
static int32_t                  ARM_ETH_MAC_SetAddressFilter(const ARM_ETH_MAC_ADDR *ptr_addr, uint32_t num_addr);
static int32_t                  ARM_ETH_MAC_SendFrame(const uint8_t *frame, uint32_t len, uint32_t flags);
static int32_t                  ARM_ETH_MAC_ReadFrame(uint8_t *frame, uint32_t len);
static uint32_t                 ARM_ETH_MAC_GetRxFrameSize(void);
static int32_t                  ARM_ETH_MAC_GetRxFrameTime(ARM_ETH_MAC_TIME *time);
static int32_t                  ARM_ETH_MAC_GetTxFrameTime(ARM_ETH_MAC_TIME *time);
static int32_t                  ARM_ETH_MAC_Control(uint32_t control, uint32_t arg);
static int32_t                  ARM_ETH_MAC_ControlTimer(uint32_t control, ARM_ETH_MAC_TIME *time);
static int32_t                  ARM_ETH_MAC_PHY_Read(uint8_t phy_addr, uint8_t reg_addr, uint16_t *data);
static int32_t                  ARM_ETH_MAC_PHY_Write(uint8_t phy_addr, uint8_t reg_addr, uint16_t data);

/* Driver Capabilities */
static const ARM_ETH_MAC_CAPABILITIES DriverCapabilities =
{
    1, /* 1 = IPv4 header checksum verified on receive */
    1, /* 1 = IPv6 checksum verification supported on receive */
    1, /* 1 = UDP payload checksum verified on receive */
    1, /* 1 = TCP payload checksum verified on receive */
    1, /* 1 = ICMP payload checksum verified on receive */
    1, /* 1 = IPv4 header checksum generated on transmit */
    1, /* 1 = IPv6 checksum generation supported on transmit */
    1, /* 1 = UDP payload checksum generated on transmit */
    1, /* 1 = TCP payload checksum generated on transmit */
    1, /* 1 = ICMP payload checksum generated on transmit */
    1, /* 0 = MII, 1 = RMII, 2 = SMII */
    1, /* 1 = driver provides initial valid MAC address */
    1, /* 1 = callback event \ref ARM_ETH_MAC_EVENT_RX_FRAME generated */
    1, /* 1 = callback event \ref ARM_ETH_MAC_EVENT_TX_FRAME generated */
    1, /* 1 = wakeup event \ref ARM_ETH_MAC_EVENT_WAKEUP generated */
    0, /* 1 = Precision Timer supported */
    0  /* Reserved (must be zero) */
};

/**
  \fn          ARM_DRIVER_VERSION GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION ARM_ETH_MAC_GetVersion(void)
{
    return DriverVersion;
}

/**
  \fn          ARM_ETH_MAC_CAPABILITIES GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_ETH_MAC_CAPABILITIES
*/
static ARM_ETH_MAC_CAPABILITIES ARM_ETH_MAC_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
  \fn          int32_t Initialize (ARM_ETH_MAC_SignalEvent_t cb_event)
  \brief       Initialize Ethernet MAC Device.
  \param[in]   cb_event  Pointer to \ref ARM_ETH_MAC_SignalEvent
  \return      \ref execution_status
*/
static int32_t ARM_ETH_MAC_Initialize(ARM_ETH_MAC_SignalEvent_t cb_event)
{
    /* Clear run-time info */
    memset((void *)&eth_mac0_rw_info, 0, sizeof(RW_Info_t));

    /* Register callback function */
    eth_mac0_rw_info.cb_event = cb_event;

    /* Set driver status to initialized */
    eth_mac0_rw_info.drv_status.initialized = 1U;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t Uninitialize (void)
  \brief       De-initialize Ethernet MAC Device.
  \return      \ref execution_status
*/
static int32_t ARM_ETH_MAC_Uninitialize(void)
{
    /* The implementation does not include clock settings, which must be configured in the application */
#if 0
    if (eth_mac0_rw_info.drv_status.powered != 0U)
    {
        /* If peripheral is powered, power off the peripheral */
        ARM_ETH_MAC_PowerControl(ARM_POWER_OFF);
    }

#endif

    /* Clear run-time info */
    memset((void *)&eth_mac0_rw_info, 0, sizeof(RW_Info_t));

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t PowerControl (ARM_POWER_STATE state)
  \brief       Control Ethernet MAC Device Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t ARM_ETH_MAC_PowerControl(ARM_POWER_STATE state)
{
    switch (state)
    {
        case ARM_POWER_OFF:

            if (eth_mac0_rw_info.drv_status.powered == 0U)
            {
                return ARM_DRIVER_OK;
            }

            /* Close EMAC function */
            EMAC_Close();

            /* Set driver status to not powered */
            eth_mac0_rw_info.drv_status.powered = 0U;

            break;

        case ARM_POWER_LOW:

            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_POWER_FULL:

            if (eth_mac0_rw_info.drv_status.initialized == 0U)
            {
                return ARM_DRIVER_ERROR;
            }

            if (eth_mac0_rw_info.drv_status.powered == 1U)
            {
                return ARM_DRIVER_OK;
            }

            SYS_ResetModule(SYS_EMAC0RST);

            /* Open and set EMAC function */
            EMAC_Open(&g_au8MacAddr[0]);

            eth_mac0_rw_info.tx_len = 0;

            /* Set driver status to powered */
            eth_mac0_rw_info.drv_status.powered = 1U;

            break;

        default:
            return ARM_DRIVER_ERROR_PARAMETER;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t GetMacAddress (ARM_ETH_MAC_ADDR *ptr_addr)
  \brief       Get Ethernet MAC Address.
  \param[in]   ptr_addr  Pointer to address
  \return      \ref execution_status
*/
static int32_t ARM_ETH_MAC_GetMacAddress(ARM_ETH_MAC_ADDR *ptr_addr)
{
    if (ptr_addr == NULL)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (eth_mac0_rw_info.drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    synopGMAC_get_mac_addr(&GMACdev, GmacAddr0High, GmacAddr0Low, ptr_addr->b);

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SetMacAddress (const ARM_ETH_MAC_ADDR *ptr_addr)
  \brief       Set Ethernet MAC Address.
  \param[in]   ptr_addr  Pointer to address
  \return      \ref execution_status
*/
static int32_t ARM_ETH_MAC_SetMacAddress(const ARM_ETH_MAC_ADDR *ptr_addr)
{
    if (ptr_addr == NULL)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (eth_mac0_rw_info.drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    synopGMAC_set_mac_addr(&GMACdev, GmacAddr0High, GmacAddr0Low, (u8 *)ptr_addr->b);

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SetAddressFilter (const ARM_ETH_MAC_ADDR *ptr_addr,
                                               uint32_t          num_addr)
  \brief       Configure Address Filter.
  \param[in]   ptr_addr  Pointer to addresses
  \param[in]   num_addr  Number of addresses to configure
  \return      \ref execution_status
*/
static int32_t ARM_ETH_MAC_SetAddressFilter(const ARM_ETH_MAC_ADDR *ptr_addr, uint32_t num_addr)
{
    (void)ptr_addr;
    (void)num_addr;

    /* The hash filter function is not supported */
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t SendFrame (const uint8_t *frame, uint32_t len, uint32_t flags)
  \brief       Send Ethernet frame.
  \param[in]   frame  Pointer to frame buffer with data to send
  \param[in]   len    Frame buffer length in bytes
  \param[in]   flags  Frame transmit flags (see ARM_ETH_MAC_TX_FRAME_...)
  \return      \ref execution_status
*/
static int32_t ARM_ETH_MAC_SendFrame(const uint8_t *frame, uint32_t len, uint32_t flags)
{
    uint8_t *tx_buf = NULL;

    tx_buf = EMAC_AllocatePktBuf();

    if ((flags & ARM_ETH_MAC_TX_FRAME_TIMESTAMP) != 0U)
    {
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    /* Enable event bit ARM_ETH_MAC_EVENT_TX_FRAME called when frame send is complete */
    if ((flags & ARM_ETH_MAC_TX_FRAME_EVENT) != 0U)
    {
        eth_mac0_rw_info.mac_tx_event = 1U;
    }
    else
    {
        eth_mac0_rw_info.mac_tx_event = 0U;
    }

    if ((frame == NULL) || (len == 0U))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if ((eth_mac0_rw_info.drv_status.powered == 0U) || (eth_mac0_rw_info.mac_tx_enabled == 0U))
    {
        return ARM_DRIVER_ERROR;
    }

    if (synopGMAC_is_desc_owned_by_dma(GMACdev.TxBusyDesc))
    {
        /* The descriptor is owned by DMA */
        return ARM_DRIVER_ERROR_BUSY;
    }

    if ((flags & ARM_ETH_MAC_TX_FRAME_FRAGMENT) != 0U)
    {
        /* Collect multiple fragments before the frame is sent */
        memcpy(tx_buf + eth_mac0_rw_info.tx_len, frame, len);

        eth_mac0_rw_info.tx_len += len;

        return ARM_DRIVER_OK;
    }

    memcpy(tx_buf + eth_mac0_rw_info.tx_len, frame, len);

    eth_mac0_rw_info.tx_len += len;

    if (eth_mac0_rw_info.tx_len > PKT_FRAME_BUF_SIZE)
    {
        memset(tx_buf, 0, eth_mac0_rw_info.tx_len);

        eth_mac0_rw_info.tx_len = 0;

        return ARM_DRIVER_ERROR;
    }

    if (synopGMAC_xmit_frames(&GMACdev, tx_buf, eth_mac0_rw_info.tx_len, eth_mac0_rw_info.offload_needed, 0) != 0)
    {
        eth_mac0_rw_info.tx_len = 0;

        return ARM_DRIVER_ERROR;
    }

    eth_mac0_rw_info.tx_len = 0;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ReadFrame (uint8_t *frame, uint32_t len)
  \brief       Read data of received Ethernet frame.
  \param[in]   frame  Pointer to frame buffer for data to read into
  \param[in]   len    Frame buffer length in bytes
  \return      number of data bytes read or execution status
                 - value >= 0: number of data bytes read
                 - value < 0: error occurred, value is execution status as defined with \ref execution_status
*/
static int32_t ARM_ETH_MAC_ReadFrame(uint8_t *frame, uint32_t len)
{
    int32_t ret = 0;

    if ((frame == NULL) && (len != 0U))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (eth_mac0_rw_info.drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    if (eth_mac0_rw_info.rx_len > 0)
    {
        if (eth_mac0_rw_info.rx_len > len)
        {
            return ARM_DRIVER_ERROR;
        }
        else
        {
            memcpy(frame, psPktFrame, len);
            ret = len;
            eth_mac0_rw_info.rx_len = 0;
        }
    }

    return ret;
}

/**
  \fn          uint32_t GetRxFrameSize (void)
  \brief       Get size of received Ethernet frame.
  \return      number of bytes in received frame
*/
static uint32_t ARM_ETH_MAC_GetRxFrameSize(void)
{
    if (eth_mac0_rw_info.drv_status.powered == 0U)
    {
        return 0U;
    }

    return eth_mac0_rw_info.rx_len;
}

/**
  \fn          int32_t GetRxFrameTime (ARM_ETH_MAC_TIME *time)
  \brief       Get time of received Ethernet frame.
  \param[in]   time  Pointer to time structure for data to read into
  \return      \ref execution_status
*/
static int32_t ARM_ETH_MAC_GetRxFrameTime(ARM_ETH_MAC_TIME *time)
{
    (void)time;
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t GetTxFrameTime (ARM_ETH_MAC_TIME *time)
  \brief       Get time of transmitted Ethernet frame.
  \param[in]   time  Pointer to time structure for data to read into
  \return      \ref execution_status
*/
static int32_t ARM_ETH_MAC_GetTxFrameTime(ARM_ETH_MAC_TIME *time)
{
    (void)time;
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t Control (uint32_t control, uint32_t arg)
  \brief       Control Ethernet Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \return      \ref execution_status
*/
static int32_t ARM_ETH_MAC_Control(uint32_t control, uint32_t arg)
{
    uint32_t data;
    synopGMACdevice *gmacdev = &GMACdev;

    if (eth_mac0_rw_info.drv_status.powered == 0U)
    {
        return ARM_DRIVER_ERROR;
    }

    switch (control)
    {
        case ARM_ETH_MAC_CONFIGURE:

            switch (arg & ARM_ETH_MAC_SPEED_Msk)
            {
                case ARM_ETH_MAC_SPEED_10M:

                    gmacdev->Speed = SPEED10;
                    synopGMAC_set_speed(gmacdev);

                    break;

                case ARM_ETH_MAC_SPEED_100M:

                    gmacdev->Speed = SPEED100;
                    synopGMAC_set_speed(gmacdev);

                    break;

                default:
                    return ARM_DRIVER_ERROR_UNSUPPORTED;
            }

            switch (arg & ARM_ETH_MAC_DUPLEX_Msk)
            {
                case ARM_ETH_MAC_DUPLEX_FULL:

                    synopGMAC_set_full_duplex(gmacdev);

                    break;

                case ARM_ETH_MAC_DUPLEX_HALF:

                    synopGMAC_set_half_duplex(gmacdev);

                    break;
            }

            if (arg & ARM_ETH_MAC_LOOPBACK)
            {
                synopGMAC_loopback_on(gmacdev);
            }
            else
            {
                synopGMAC_loopback_off(gmacdev);
            }

            if ((arg & ARM_ETH_MAC_CHECKSUM_OFFLOAD_RX) ||
                    (arg & ARM_ETH_MAC_CHECKSUM_OFFLOAD_TX))
            {
                synopGMAC_enable_rx_chksum_offload(gmacdev);
                eth_mac0_rw_info.offload_needed = 1;
            }
            else
            {
                synopGMAC_disable_rx_chksum_offload(gmacdev);
                eth_mac0_rw_info.offload_needed = 0;
            }

            if (arg & ARM_ETH_MAC_ADDRESS_BROADCAST)
            {
                synopGMAC_broadcast_enable(gmacdev);
            }
            else
            {
                synopGMAC_broadcast_disable(gmacdev);
            }

            if (arg & ARM_ETH_MAC_ADDRESS_MULTICAST)
            {
                synopGMAC_multicast_enable(gmacdev);
            }
            else
            {
                synopGMAC_multicast_disable(gmacdev);
            }

            if (arg & ARM_ETH_MAC_ADDRESS_ALL)
            {
                synopGMAC_promisc_enable(gmacdev);
            }
            else
            {
                synopGMAC_promisc_disable(gmacdev);
            }

            break;

        case ARM_ETH_MAC_CONTROL_TX:

            if (arg)
            {
                synopGMAC_tx_enable(gmacdev);
                synopGMAC_enable_dma_tx(gmacdev);
                eth_mac0_rw_info.mac_tx_enabled = 1;
            }
            else
            {
                synopGMAC_tx_disable(gmacdev);
                synopGMAC_disable_dma_tx(gmacdev);
                eth_mac0_rw_info.mac_tx_enabled = 0;
            }

            break;

        case ARM_ETH_MAC_CONTROL_RX:

            if (arg)
            {
                synopGMAC_rx_enable(gmacdev);
                synopGMAC_enable_dma_rx(gmacdev);
            }
            else
            {
                synopGMAC_rx_disable(gmacdev);
                synopGMAC_disable_dma_rx(gmacdev);
            }

            break;

        case ARM_ETH_MAC_FLUSH:

            data = synopGMACReadReg(gmacdev->DmaBase, DmaControl);

            if (arg & ARM_ETH_MAC_FLUSH_RX)
            {
                /* Enable Flushing of Received Frames */
                data &= ~BIT24;
                synopGMACWriteReg(gmacdev->DmaBase, DmaControl, data);
            }
            else
            {
                /* Disable Flushing of Received Frames */
                data |= BIT24;
                synopGMACWriteReg(gmacdev->DmaBase, DmaControl, data);
            }

            if (arg & ARM_ETH_MAC_FLUSH_TX)
            {
                /* Flush Transmit FIFO */
                data |= BIT20;
                synopGMACWriteReg(gmacdev->DmaBase, DmaControl, data);
            }

            break;

        case ARM_ETH_MAC_SLEEP:

            if (arg)
            {
                synopGMAC_magic_packet_enable(gmacdev);
                synopGMAC_power_down_enable(gmacdev);
            }
            else
            {
                synopGMAC_magic_packet_disable(gmacdev);
                synopGMAC_power_down_disable(gmacdev);
            }

            break;

        case ARM_ETH_MAC_VLAN_FILTER:

            if (!arg)
            {
                synopGMACClearBits(gmacdev->MacBase, GmacFrameFilter, GmacVlanTagFilter);
            }
            else
            {
                synopGMACSetBits(gmacdev->MacBase, GmacFrameFilter, GmacVlanTagFilter);

                if ((arg & ARM_ETH_MAC_VLAN_FILTER_ID_ONLY) != 0U)
                {
                    /* Compare only the 12-bit VLAN identifier */
                    synopGMACWriteReg(gmacdev->MacBase, GmacVlan, GmacEnable12BitComp | (arg & 0xFFF));
                }
                else
                {
                    /* Compare the complete 16-bit VLAN tag value */
                    synopGMACWriteReg(gmacdev->MacBase, GmacVlan, (arg & 0xFFFF));
                }
            }

            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ControlTimer (uint32_t control, ARM_ETH_MAC_TIME *time)
  \brief       Control Precision Timer.
  \param[in]   control  Operation
  \param[in]   time     Pointer to time structure
  \return      \ref execution_status
*/
static int32_t ARM_ETH_MAC_ControlTimer(uint32_t control, ARM_ETH_MAC_TIME *time)
{
    (void)time;
    (void)control;
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t PHY_Read (uint8_t phy_addr, uint8_t reg_addr, uint16_t *data)
  \brief       Read Ethernet PHY Register through Management Interface.
  \param[in]   phy_addr  5-bit device address
  \param[in]   reg_addr  5-bit register address
  \param[out]  data      Pointer where the result is written to
  \return      \ref execution_status
*/
static int32_t ARM_ETH_MAC_PHY_Read(uint8_t phy_addr, uint8_t reg_addr, uint16_t *data)
{
    synopGMACdevice *gmacdev = &GMACdev;

    if (synopGMAC_read_phy_reg((u32)gmacdev->MacBase, phy_addr, reg_addr, data) != 0)
        return ARM_DRIVER_ERROR;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t PHY_Write (uint8_t phy_addr, uint8_t reg_addr, uint16_t data)
  \brief       Write Ethernet PHY Register through Management Interface.
  \param[in]   phy_addr  5-bit device address
  \param[in]   reg_addr  5-bit register address
  \param[in]   data      16-bit data to write
  \return      \ref execution_status
*/
static int32_t ARM_ETH_MAC_PHY_Write(uint8_t phy_addr, uint8_t reg_addr, uint16_t data)
{
    synopGMACdevice *gmacdev = &GMACdev;

    if (synopGMAC_write_phy_reg((u32)gmacdev->MacBase, phy_addr, reg_addr, data) != 0)
        return ARM_DRIVER_ERROR;

    return ARM_DRIVER_OK;
}

extern \
ARM_DRIVER_ETH_MAC Driver_ETH_MAC0;
ARM_DRIVER_ETH_MAC Driver_ETH_MAC0 =
{
    ARM_ETH_MAC_GetVersion,
    ARM_ETH_MAC_GetCapabilities,
    ARM_ETH_MAC_Initialize,
    ARM_ETH_MAC_Uninitialize,
    ARM_ETH_MAC_PowerControl,
    ARM_ETH_MAC_GetMacAddress,
    ARM_ETH_MAC_SetMacAddress,
    ARM_ETH_MAC_SetAddressFilter,
    ARM_ETH_MAC_SendFrame,
    ARM_ETH_MAC_ReadFrame,
    ARM_ETH_MAC_GetRxFrameSize,
    ARM_ETH_MAC_GetRxFrameTime,
    ARM_ETH_MAC_GetTxFrameTime,
    ARM_ETH_MAC_ControlTimer,
    ARM_ETH_MAC_Control,
    ARM_ETH_MAC_PHY_Read,
    ARM_ETH_MAC_PHY_Write
};

/**
  \fn          void EMAC_Open(uint8_t *macaddr)
  \brief       Open and set EMAC function
  \param[in]   Pointer to MAC address
  \return      None
*/
void EMAC_Open(uint8_t *macaddr)
{
    uint32_t i;

    /* Attach the device to MAC struct This will configure all the required base addresses
      such as Mac base, configuration base, phy base address(out of 32 possible phys) */
    synopGMAC_attach(&GMACdev, (EMAC0_BASE + MACBASE), (EMAC0_BASE + DMABASE), DEFAULT_PHY_BASE, macaddr);

    synopGMAC_reset(&GMACdev);

    memcpy((void *)&GMACdev.mac_addr[0], (void *)&macaddr[0], 6);

    /* Lets read the version of IP in to device structure */
    synopGMAC_read_version(&GMACdev);

    /* Get system clock */
    SystemCoreClockUpdate();

    /* Check for Phy initialization */
    if (SystemCoreClock >= 250000000)
        synopGMAC_set_mdc_clk_div(&GMACdev, GmiiCsrClk5);
    else if (SystemCoreClock >= 150000000)
        synopGMAC_set_mdc_clk_div(&GMACdev, GmiiCsrClk4);
    else if (SystemCoreClock >= 100000000)
        synopGMAC_set_mdc_clk_div(&GMACdev, GmiiCsrClk1);
    else if (SystemCoreClock >= 60000000)
        synopGMAC_set_mdc_clk_div(&GMACdev, GmiiCsrClk0);
    else if (SystemCoreClock >= 35000000)
        synopGMAC_set_mdc_clk_div(&GMACdev, GmiiCsrClk3);
    else
        synopGMAC_set_mdc_clk_div(&GMACdev, GmiiCsrClk2);

    GMACdev.ClockDivMdc = synopGMAC_get_mdc_clk_div(&GMACdev);

    /* Set up the tx and rx descriptor queue/ring */
    synopGMAC_setup_tx_desc_queue(&GMACdev, &tx_desc[0], TRANSMIT_DESC_SIZE, RINGMODE);
    /* Program the transmit descriptor base address in to DmaTxBase addr */
    synopGMAC_init_tx_desc_base(&GMACdev);

    synopGMAC_setup_rx_desc_queue(&GMACdev, &rx_desc[0], RECEIVE_DESC_SIZE, RINGMODE);
    /* Program the transmit descriptor base address in to DmaRxBase addr */
    synopGMAC_init_rx_desc_base(&GMACdev);

    synopGMAC_dma_bus_mode_init(&GMACdev, (DmaBurstLength32 | DmaDescriptorSkip0 | DmaDescriptor8Words));
    synopGMAC_dma_control_init(&GMACdev, (DmaStoreAndForward | DmaTxSecondFrame | DmaRxThreshCtrl128));

    /* Initialize the mac interface */
    synopGMAC_mac_init(&GMACdev);
    /* This enables pause control in full-duplex mode of operation */
    synopGMAC_pause_control(&GMACdev);

    for (i = 0; i < RECEIVE_DESC_SIZE; i ++)
    {
        synopGMAC_set_rx_qptr(&GMACdev, (u32)&rx_buf[i], PKT_FRAME_BUF_SIZE, 0);
    }

    /* Enable interrupt */
    synopGMAC_clear_interrupt(&GMACdev);
    synopGMAC_disable_interrupt_all(&GMACdev);
    synopGMAC_enable_interrupt(&GMACdev, DmaIntEnable);

    /* Enable DMA */
    synopGMAC_enable_dma_rx(&GMACdev);
    synopGMAC_enable_dma_tx(&GMACdev);

    synopGMAC_set_mac_addr(&GMACdev, GmacAddr0High, GmacAddr0Low, &GMACdev.mac_addr[0]);

    synopGMAC_set_mode(&GMACdev, GMACdev.Speed);

    NVIC_EnableIRQ(EMAC0_IRQn);
}

/**
  \fn          void EMAC_Close(uint8_t *macaddr)
  \brief       Close EMAC function
  \param[in]   None
  \return      None
*/
void EMAC_Close(void)
{
    /* Disable interrupt */
    synopGMAC_clear_interrupt(&GMACdev);
    synopGMAC_disable_interrupt_all(&GMACdev);
    NVIC_DisableIRQ(EMAC0_IRQn);

    /* Disable DMA */
    synopGMAC_disable_dma_rx(&GMACdev);
    synopGMAC_disable_dma_tx(&GMACdev);
}

/**
  \fn          void EMAC0_IRQHandler(void)
  \brief       EMAC0 IRQHandler
  \param[in]   None
  \return      None
*/
NVT_ITCM void EMAC0_IRQHandler(void)
{
    uint32_t u32Status;
    uint32_t u32Event = 0;
    u32 interrupt, dma_status_reg;
    s32 status;
    u32 u32GmacIntSts;
    u32 u32GmacDmaIE = DmaIntEnable;

    u32GmacIntSts = synopGMACReadReg(GMACdev.MacBase, GmacInterruptStatus);

    if (u32GmacIntSts & GmacTSIntSts)
    {
        GMACdev.synopGMACNetStats.ts_int = 1;
        status = synopGMACReadReg(GMACdev.MacBase, GmacTSStatus);
    }

    synopGMACWriteReg(GMACdev.MacBase, GmacInterruptStatus, u32GmacIntSts);

    dma_status_reg = synopGMACReadReg(GMACdev.DmaBase, DmaStatus);

    if (dma_status_reg == 0)
    {
        return;
    }

    if (dma_status_reg & GmacPmtIntr)
    {
        synopGMAC_powerup_mac(&GMACdev);
        u32Event |= ARM_ETH_MAC_EVENT_WAKEUP;
    }

    interrupt = synopGMAC_get_interrupt_type(&GMACdev);

    if (interrupt & synopGMACDmaError)
    {
        synopGMAC_disable_dma_tx(&GMACdev);
        synopGMAC_disable_dma_rx(&GMACdev);

        synopGMAC_take_desc_ownership_tx(&GMACdev);
        synopGMAC_take_desc_ownership_rx(&GMACdev);

        synopGMAC_init_tx_rx_desc_queue(&GMACdev);

        synopGMAC_reset(&GMACdev);

        synopGMAC_set_mac_addr(&GMACdev, GmacAddr0High, GmacAddr0Low, &GMACdev.mac_addr[0]);
        synopGMAC_dma_bus_mode_init(&GMACdev, DmaBurstLength32 | DmaDescriptorSkip0 | DmaDescriptor8Words);
        synopGMAC_dma_control_init(&GMACdev, DmaStoreAndForward | DmaTxSecondFrame | DmaRxThreshCtrl128);
        synopGMAC_init_rx_desc_base(&GMACdev);
        synopGMAC_init_tx_desc_base(&GMACdev);
        synopGMAC_mac_init(&GMACdev);
        synopGMAC_enable_dma_rx(&GMACdev);
        synopGMAC_enable_dma_tx(&GMACdev);
    }

    if ((interrupt & synopGMACDmaRxNormal) ||
            (interrupt & synopGMACDmaRxAbnormal))
    {
        if (interrupt & synopGMACDmaRxNormal)
        {
            u32GmacDmaIE &= ~DmaIntRxNormMask;
            u32Event |= ARM_ETH_MAC_EVENT_RX_FRAME;
        }

        if (interrupt & synopGMACDmaRxAbnormal)
        {
            if (GMACdev.GMAC_Power_down == 0)
            {
                GMACdev.synopGMACNetStats.rx_over_errors++;
                u32GmacDmaIE &= ~DmaIntRxAbnMask;
                synopGMAC_resume_dma_rx(&GMACdev);
            }
        }
    }

    if (interrupt & synopGMACDmaRxStopped)
    {
        if (GMACdev.GMAC_Power_down == 0)
        {
            GMACdev.synopGMACNetStats.rx_over_errors++;
            synopGMAC_enable_dma_rx(&GMACdev);
        }
    }

    if (interrupt & synopGMACDmaTxNormal)
    {
        synop_handle_transmit_over(&GMACdev);

        if (eth_mac0_rw_info.mac_tx_event)
        {
            u32Event |= ARM_ETH_MAC_EVENT_TX_FRAME;
        }
    }

    if (interrupt & synopGMACDmaTxAbnormal)
    {
        if (GMACdev.GMAC_Power_down == 0)
        {
            synop_handle_transmit_over(&GMACdev);
        }
    }

    if (interrupt & synopGMACDmaTxStopped)
    {
        /* If Mac is not in powerdown */
        if (GMACdev.GMAC_Power_down == 0)
        {
            synopGMAC_disable_dma_tx(&GMACdev);
            synopGMAC_take_desc_ownership_tx(&GMACdev);
            synopGMAC_enable_dma_tx(&GMACdev);
        }
    }

    /* Enable the interrrupt before returning from ISR */
    synopGMAC_enable_interrupt(&GMACdev, u32GmacDmaIE);

    if (interrupt & synopGMACDmaRxNormal)
    {
        // call back function
        // move to read frame
        if ((eth_mac0_rw_info.rx_len = synop_handle_received_data(&GMACdev, &psPktFrame)) > 0)
        {
            synopGMAC_set_rx_qptr(&GMACdev, (u32)psPktFrame, PKT_FRAME_BUF_SIZE, 0);
            synopGMAC_enable_interrupt(&GMACdev, DmaIntEnable);
        }
    }

    if ((eth_mac0_rw_info.cb_event != NULL) && (u32Event != 0U))
    {
        eth_mac0_rw_info.cb_event(u32Event);
    }

    /* CPU read interrupt flag register to wait write(clear) instruction completement */
    u32Status = synopGMACReadReg(GMACdev.MacBase, GmacInterruptStatus);

    (void)u32Status;
    (void)status;
}

/**
  \fn          uint8_t *EMAC_AllocatePktBuf(void)
  \brief       Allocate packet buffer
  \param[in]   None
  \return      tx_buf address
*/
uint8_t *EMAC_AllocatePktBuf(void)
{
    u32 index = GMACdev.TxNext;
    return &tx_buf[index].au8Buf[0];
}

