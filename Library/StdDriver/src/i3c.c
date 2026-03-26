/**************************************************************************//**
 * @file     i3c.c
 * @version  V1.00
 * @brief    I3C driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "NuMicro.h"

/** @cond HIDDEN_SYMBOLS */

#ifndef I3C_DrvMsg_EN
    #define I3C_DrvMsg_EN (0) // default turn off debug message
#endif

#if (I3C_DrvMsg_EN)
    #define I3C_DrvMsg printf
#else
    #define I3C_DrvMsg(...)
#endif
/** @endcond HIDDEN_SYMBOLS */


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup I3C_Driver I3C Driver
  @{
*/

/** @addtogroup I3C_EXPORTED_FUNCTIONS I3C Exported Functions
  @{
*/


/**
  * @brief  Initial I3C Device Type
  */
int32_t I3C_DeviceInit(I3C_DEVICE_T *dev)
{
    /* Response buffer threshold */
    dev->port->QUETHCTL  = (0UL << I3C_QUETHCTL_RESPTH_Pos);
    /* Command buffer empty threshold */
    dev->port->QUETHCTL |= (1UL << I3C_QUETHCTL_CMDETH_Pos);
    /* IBI data segment size to 2-words(8-bytes) */
    dev->port->QUETHCTL |= (2UL << I3C_QUETHCTL_IBIDATTH_Pos);
    /* Rx receive and Tx empty threshold */
    dev->port->DBTHCTL  = ((0UL << I3C_DBTHCTL_TXTH_Pos) | (0UL << I3C_DBTHCTL_RXTH_Pos));
    /* Rx and Tx start threshold */
    dev->port->DBTHCTL |= ((0UL << I3C_DBTHCTL_TXSTATH_Pos) | (0UL << I3C_DBTHCTL_RXSTATH_Pos));
    /* Clear current interrupt status */
    dev->port->INTSTS |= dev->port->INTSTS;
    /* Enable all interrupt status */
    dev->port->INTSTSEN = ~0UL;
    /* Enable specified interrupt signal */
    dev->port->INTEN = (I3C_INTEN_RESPRDY_Msk  | I3C_INTEN_CCCUPD_Msk   | I3C_INTEN_DAA_Msk |
                        I3C_INTEN_TFRERR_Msk   | I3C_INTEN_READREQ_Msk  | I3C_INTEN_TFRABORT_Msk |
                        I3C_INTEN_IBIUPD_Msk   | I3C_INTEN_DEFTGTS_Msk  | I3C_INTEN_BUSOWNER_Msk |
                        I3C_INTEN_BUSRSTDN_Msk | I3C_INTEN_IBITH_Msk);

    /* Configure related bus timings, SCL clock */
    if (dev->engclk == 0UL)
    {
        return I3C_STS_INVALID_INPUT;
    }

    I3C_BusClkConfig(dev);

    if (dev->device_role == I3C_CONTROLLER)
    {
        /* Program Dynamic Address for Controller */
        dev->port->DEVADDR = (I3C_DEVADDR_DAVALID_Msk | (dev->main_controller_da << I3C_DEVADDR_DA_Pos));
        /* Set as Controller role */
        dev->port->DEVCTLE = dev->device_role;
        /* Set Conroller includes I3C Broadcast Address */
        dev->port->DEVCTL |= (I3C_DEVCTL_ENABLE_Msk | I3C_DEVCTL_IBAINCL_Msk);
        /* Enable device */
        dev->port->DEVCTL |= I3C_DEVCTL_ENABLE_Msk;
        /* Enable GETMXDS CCC response */
        dev->port->SLVCHAR |= I3C_SLVCHAR_MXDSLIM_Msk;
    }
    else if (dev->device_role == I3C_TARGET)
    {
        /* Set as Target role */
        dev->port->DEVCTLE = dev->device_role;
        /* Includes I3C Broadcast Address */
        dev->port->DEVCTL |= (I3C_DEVCTL_IBAINCL_Msk);

        if (dev->main_target_sa != 0UL)
        {
            /* Program Statis Address for Target */
            dev->port->DEVADDR |= (I3C_DEVADDR_SAVALID_Msk | (dev->main_target_sa << I3C_DEVADDR_SA_Pos));
            dev->target_sa[0] = dev->main_target_sa;
        }

        /* Select the Target ability for enter I3C mode */
        if (dev->target_daa_mode == I3C_SUPPORT_ENTDAA)
        {
            /* HJEN disabled: Target supports ENTDAA CCC */
            dev->port->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
            dev->port->SLVEVNTS &= ~I3C_SLVEVNTS_HJEN_Msk;
        }
        else if (dev->target_daa_mode == I3C_SUPPORT_ADAPTIVE_HJ)
        {
            /* Both ADAPTIVE and HJEN enabled: Target generates a Hot-Join request until receive I3C header 7'h7E on the bus */
            dev->port->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
            dev->port->SLVEVNTS |= I3C_SLVEVNTS_HJEN_Msk;
        }
        else if (dev->target_daa_mode == I3C_SUPPORT_IMMEDIATE_HJ)
        {
            /* Only HJEN enabled: Target generates a Hot-Join request immediately */
            dev->port->DEVCTL &= ~I3C_DEVCTL_ADAPTIVE_Msk;
            dev->port->SLVEVNTS |= I3C_SLVEVNTS_HJEN_Msk;
        }
        else
        {
            /* HJEN disabled: Target supports ENTDAA CCC */
            dev->port->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
            dev->port->SLVEVNTS &= ~I3C_SLVEVNTS_HJEN_Msk;
        }

        /* Enable GETMXDS CCC response */
        dev->port->SLVCHAR |= I3C_SLVCHAR_MXDSLIM_Msk;
    }
    else
    {
        return I3C_STS_INVALID_INPUT;
    }

    if (dev->device_role == I3C_CONTROLLER)
    {
        if (!(dev->port->DEVCTL & I3C_DEVCTL_ENABLE_Msk))
        {
            return (~I3C_STS_NO_ERR);
        }
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief      Setup I2C / I3C Device Address Table
  *
  * @param[in]  *i3c            Point to I3C peripheral
  * @param[in]  u8DevIndex      the offset of Device Address Table.
  *                             It could be 0 ~ 6 for DEV1ADR to DEV7ADR.
  * @param[in]  u8DevType       the Target device type. It could be I3C_DEVTYPE_I3C for I3C device and I3C_DEVTYPE_I2C for I2C device
  * @param[in]  u8DAddr         7Bits Device Synamic Address
  * @param[in]  u8SAddr         7Bits Device Static Address
  *
  * @retval     I3C_STS_NO_ERR          No error
  * @retval     I3C_STS_INVALID_INPUT   Invalid input parameter
  *
  * @details    The function is used for I3C Controller to setup Device Address Table.
  * @note       Device Address Table must be set before communication with Target Devices.
  *
  */
int32_t I3C_SetDeviceAddr(I3C_T *i3c, uint8_t u8DevIndex, uint8_t u8DevType, uint8_t u8DAddr, uint8_t u8SAddr)
{
    volatile uint32_t i;
    volatile uint32_t count = 0;
    volatile uint32_t u32Device = 0;

    if ((u8DAddr & 0x80UL) || (u8SAddr & 0x80UL))
    {
        return I3C_STS_INVALID_INPUT;
    }

    // I3C Device Dynamic Address with Odd Parity
    for (i = 0UL; i < 8UL; i++)
    {
        if ((u8DAddr >> i) & 0x1UL)
        {
            count ++;
        }
    }

    if ((count % 2UL) == 0UL)
    {
        u32Device = ((u8DAddr | 0x80UL) << I3C_DEVADR_DADR_Pos);
    }
    else
    {
        u32Device = (u8DAddr << I3C_DEVADR_DADR_Pos);
    }

    // I2C Device Static Address
    u32Device |= (u8SAddr << I3C_DEVADR_STADR_Pos);
    // Support IBI with one or more Mandatory Bytes
    u32Device |= (I3C_DEVADR_IBIWDATA_Msk);

    // Device Type
    if (u8DevType != I3C_DEVTYPE_I3C)
    {
        u32Device |= I3C_DEVADR_DEVICE_Msk;
    }

    switch (u8DevIndex)
    {
        case 0UL:
        case 1UL:
        case 2UL:
        case 3UL:
        case 4UL:
        case 5UL:
        case 6UL:
            /* Configure device address on specify table location */
            i3c->DEVADR[u8DevIndex] = u32Device;
            break;

        default:
            return I3C_STS_INVALID_INPUT;
            break;
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief  Configure Bus Timing
  */
void I3C_BusClkConfig(I3C_DEVICE_T *dev)
{
    volatile uint32_t count;
    I3C_DrvMsg("\n");
    I3C_DrvMsg("I3C_ENG_CLK = %d\n", dev->engclk);
    I3C_DrvMsg("I2C_FM_FREQ = %d\n", dev->i2c_fm_freq);
    I3C_DrvMsg("I2C_FM+_FREQ = %d\n", dev->i2c_fm_plus_freq);
    I3C_DrvMsg("I3C_SDR_FREQ = %d\n", dev->i3c_sdr_freq);

    if (dev->i2c_fm_freq != 0UL)
    {
        /* SCL freq for I2C FM mode */
        if (dev->i2c_fm_freq > dev->engclk)
        {
            dev->i2c_fm_freq = dev->engclk;
        }

        count = ((dev->engclk / dev->i2c_fm_freq) / 2UL);

        if (count < 5UL)
        {
            count = 5;
        }

        dev->port->SCLFM = ((count << I3C_SCLFM_FMHCNT_Pos) | (count << I3C_SCLFM_FMLCNT_Pos));
        I3C_DrvMsg("SCLFM = 0x%08X\n", dev->port->SCLFM);
    }

    if (dev->i2c_fm_plus_freq != 0UL)
    {
        /* SCL freq for I2C FM+ mode */
        if (dev->i2c_fm_plus_freq > dev->engclk)
        {
            dev->i2c_fm_plus_freq = dev->engclk;
        }

        count = ((dev->engclk / dev->i2c_fm_plus_freq) / 2UL);

        if (count < 5UL)
        {
            count = 5;
        }

        dev->port->SCLFMP = ((count << I3C_SCLFMP_FMPHCNT_Pos) | (count << I3C_SCLFMP_FMPLCNT_Pos));
        I3C_DrvMsg("SCLFMP = 0x%08X\n", dev->port->SCLFMP);
    }

    if (dev->i3c_sdr_freq != 0UL)
    {
        if (dev->i3c_sdr_freq > 12500000UL)
        {
            dev->i3c_sdr_freq = 12500000;
        }

        /* Set OD mode SCL freq 1MHz */
        count = ((dev->engclk / 1000000UL) / 2UL);

        if (count < 5UL)
        {
            count = 5;
        }

        dev->port->SCLOD = ((count << I3C_SCLOD_ODHCNT_Pos) | (count << I3C_SCLOD_ODLCNT_Pos));

        /* Set PP mode SCL freq for SDR0 */
        if (dev->i3c_sdr_freq > dev->engclk)
        {
            dev->i3c_sdr_freq = dev->engclk;
        }

        count = ((dev->engclk / dev->i3c_sdr_freq) / 2UL);

        if (count < 5UL)
        {
            count = 5;
        }

        if ((dev->i3c_sdr_freq == 12500000UL) && ((dev->engclk % dev->i3c_sdr_freq) != 0UL))
        {
            dev->port->SCLPP = ((count + 1UL) << I3C_SCLPP_PPLCNT_Pos);
            dev->port->SCLPP |= ((count + 1UL) << I3C_SCLPP_PPHCNT_Pos);
            dev->i3c_sdr_freq = (dev->engclk / ((count + 1UL) + (count + 1UL)));
        }
        else
        {
            dev->port->SCLPP = ((count << I3C_SCLPP_PPHCNT_Pos) | (count << I3C_SCLPP_PPLCNT_Pos));
            dev->i3c_sdr_freq = (dev->engclk / (count + count));
        }

        I3C_DrvMsg("SCLPP = 0x%08X\n", dev->port->SCLPP);
        I3C_DrvMsg("[ DRV ] Set SCL %d Hz\n", dev->i3c_sdr_freq);
    }

    /*  Bus Idle Timing ~ 200us */
    count = (uint32_t)(((float)200 * (float)dev->engclk) / (float)1000000);

    if (count == 0UL)
    {
        count = 0x50;
    }

    dev->port->BUSIDLET = count;
    /* Bus Available(1.0us) Timing ~ 1.0us */
    count = (uint32_t)(((float)1 * (float)dev->engclk) / (float)1000000);

    if (count == 0UL)
    {
        count = 0x50;
    }

    dev->port->BUSFAT = (count << I3C_BUSFAT_AVAILTC_Pos);
    /* Bus Free Timing ~ 38.4ns/0.5us/1.3us */
    count = (uint32_t)(((float)1300 * (float)dev->engclk) / (float)1000000000);

    if (count == 0UL)
    {
        count = 0x50;
    }

    dev->port->BUSFAT |= (count << I3C_BUSFAT_FREETC_Pos);
}

/**
  * @brief  Show Present State Information
  */
void I3C_PresentStateInfo(I3C_DEVICE_T *dev)
{
    volatile uint32_t role, sts, reg_val[2];
    I3C_DrvMsg("\n");
    role = ((dev->port->DEVCTLE & I3C_DEVCTLE_OPERMODE_Msk) >> I3C_DEVCTLE_OPERMODE_Pos);
    reg_val[0] = dev->port->PRESENTS;
    reg_val[1] = dev->port->CCCDEVS;
    I3C_DrvMsg("[ DRV ] Present state info: 0x%08x (%s mode)\n", reg_val[0], (role == 0) ? "Controller" : "Target");
    sts = ((reg_val[0] & I3C_PRESENTS_TFRTYPE_Msk) >> I3C_PRESENTS_TFRTYPE_Pos);

    if (sts == 0)
    {
        I3C_DrvMsg("\tDevice in IDLE state\n");
    }
    else
    {
        if (role == I3C_CONTROLLER)
        {
            if (reg_val[0] & I3C_PRESENTS_MAIDLE_Msk)
            {
                I3C_DrvMsg("\tController is in IDLE State\n");
            }
            else
            {
                I3C_DrvMsg("\tController is NOT in IDLE State\n");
            }

            if (sts == 0xF)
            {
                I3C_DrvMsg("\tController in Halt State, waiting for resume\n");
            }
            else
            {
                I3C_DrvMsg("\tController error code: 0x%x\n", sts);
            }
        }
        else
        {
            if (sts == 1)
            {
                I3C_DrvMsg("\tHot-Join Transfer State\n");
            }
            else if (sts == 2)
            {
                I3C_DrvMsg("\nIBI Transfer State\n");
            }
            else if (sts == 3)
            {
                I3C_DrvMsg("\nController Write Transfer Ongoing\n");
            }
            //else if(sts == 4) Not support in M3331
            //    I3C_DrvMsg("\nRead Data Prefetch State\n");
            else if (sts == 5)
            {
                I3C_DrvMsg("\nController Read Transfer Ongoing\n");
            }
            else if (sts == 6)
            {
                I3C_DrvMsg("\nTarget in Halt State, waiting for resume\n");
            }
            else
            {
                I3C_DrvMsg("\nTarget error code: 0x%x\n", sts);
            }
        }
    }

    I3C_DrvMsg("\n");
}

/**
  * @brief  Enable I3C Rx DMA function on PDMA ch-n.
  */
int32_t I3C_ConfigRxDMA(I3C_DEVICE_T *dev, uint32_t u32Src, uint32_t u32Dest, uint32_t u32ByteCnts)
{
    PDMA_T *pdma = dev->DMAPort;
    uint8_t ch = dev->RxDMACh;
    __IO uint32_t *reg = &((pdma)->REQSEL0_3);

    if (ch >= PDMA_CH_MAX)
    {
        return -1;
    }

    PDMA_RESET(pdma, ch);
    /* PDMA-ch for I3C Rx */
    pdma->DSCT[ch].CTL =
        PDMA_OP_BASIC | PDMA_REQ_SINGLE |
        PDMA_SAR_FIX  | PDMA_DAR_INC |
        PDMA_WIDTH_32 |
        ((((u32ByteCnts + 3UL) / 4UL) - 1UL) << PDMA_DSCT_CTL_TXCNT_Pos);
    pdma->DSCT[ch].SA = u32Src;
    pdma->DSCT[ch].DA = u32Dest;
    /* Select peripheral for the channel */
    reg[(ch / 4UL)] = (reg[(ch / 4UL)] & ~(PDMA_REQSEL0_3_REQSRC0_Msk << ((ch % 4UL) * 8UL))) | (PDMA_I3C0_RX << ((ch % 4UL) * 8UL));
    pdma->CHCTL |= (1UL << ch);
    I3C_EnableDMA(dev->port);
    return 0;
}

/**
  * @brief  Enable I3C Tx DMA function on PDMA ch-n.
  */
int32_t I3C_ConfigTxDMA(I3C_DEVICE_T *dev, uint32_t u32Src, uint32_t u32Dest, uint32_t u32ByteCnts)
{
    PDMA_T *pdma = dev->DMAPort;
    uint8_t ch = dev->TxDMACh;
    __IO uint32_t *reg = &((pdma)->REQSEL0_3);

    if (ch >= PDMA_CH_MAX)
    {
        return -1;
    }

    PDMA_RESET(pdma, ch);
    /* PDMA-ch for I3C Tx */
    pdma->DSCT[ch].CTL =
        PDMA_OP_BASIC | PDMA_REQ_SINGLE |
        PDMA_SAR_INC  | PDMA_DAR_FIX |
        PDMA_WIDTH_32 |
        ((((u32ByteCnts + 3UL) / 4UL) - 1UL) << PDMA_DSCT_CTL_TXCNT_Pos);
    pdma->DSCT[ch].SA = u32Src;
    pdma->DSCT[ch].DA = u32Dest;
    /* Select peripheral for the channel */
    reg[(ch / 4UL)] = (reg[(ch / 4UL)] & ~(PDMA_REQSEL0_3_REQSRC0_Msk << ((ch % 4UL) * 8UL))) | (PDMA_I3C0_TX << ((ch % 4UL) * 8UL));
    pdma->CHCTL |= (1UL << ch);
    /* Set DMAEN bit 0 then 1 to load Tx data by I3C DMA */
    I3C_EnableDMA(dev->port);
    return 0;
}

/**
  * @brief  Perform Dynamic Address Assignment by ENTDAA and SETDASA CCC
  */
int32_t I3C_CtrDAA(I3C_DEVICE_T *dev)
{
    uint32_t val;
    uint32_t count;
    uint32_t index;
    I3C_DrvMsg("\n");
    /* Initialize command response value */
    dev->cmd_response = I3C_CTRRESP_INITIAL_VALUE;

    if (!((dev->ccc_code == I3C_CCC_ENTDAA) || (dev->ccc_code == I3C_CCC_SETDASA)))
    {
        return I3C_STS_INVALID_INPUT;
    }

    if (dev->ccc_code == I3C_CCC_SETDASA)
    {
        if (dev->target_index > 6UL)
        {
            return I3C_STS_INVALID_INPUT;
        }

        index = dev->target_index;
        count = 1;
    }
    else
    {
        if (dev->target_count > 7UL)
        {
            return I3C_STS_INVALID_INPUT;
        }

        index = dev->target_index;
        count = dev->target_count;
    }

    /* Program Address Assignment Command Data Structure */
    val = 0;
    val |= ((count << I3C_CMDQUE_DEVCOUNT_Pos)
            | (index << I3C_CMDQUE_DEVINDX_Pos)
            | (dev->tx_id << I3C_CMDQUE_TID_Pos)
            | (dev->ccc_code << I3C_CMDQUE_CMD_Pos)
            | I3C_CMDATTR_ADDR_ASSGN_CMD);
    /* Stop and Response Status on Transfer Completion */
    val |= (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk);
    I3C_DrvMsg("[CMD val: 0x%08x] - I3C_CtrDAA\n", val);
    dev->port->CMDQUE = val;
    /* Clear parameters */
    dev->is_last_cmd  = TRUE;
    dev->is_HDR_cmd   = FALSE;
    dev->is_HDRBT_cmd = FALSE;
    dev->is_DB        = FALSE;
    dev->speed_mode   = I3C_DEVI3C_SPEED_SDR0;
    return I3C_STS_NO_ERR;
}

/**
  * @brief  Perform Write CCC Operation
  */
int32_t I3C_CtrCCCSet(I3C_DEVICE_T *dev)
{
    volatile uint32_t i;
    uint32_t val;
    uint32_t *p32Buf;
    I3C_DrvMsg("\n");
    /* Initialize command response value */
    dev->cmd_response = I3C_CTRRESP_INITIAL_VALUE;
    val = 0;

    /* Try to use SDAP */
    if ((dev->tx_len > 0UL) && (dev->tx_len <= I3C_SDAP_MAX_SIZE))
    {
        /* Set transfer argument params */
        switch (dev->tx_len)
        {
            case 3:
                val |= (1 << (I3C_CMDQUE_BYTESTRB_Pos + 2));
                val |= (dev->tx_buf[2] << I3C_CMDQUE_DATBYTE2_Pos);

            case 2:
                val |= (1 << (I3C_CMDQUE_BYTESTRB_Pos + 1));
                val |= (dev->tx_buf[1] << I3C_CMDQUE_DATBYTE1_Pos);

            case 1:
                val |= (1 << (I3C_CMDQUE_BYTESTRB_Pos + 0));
                val |= (dev->tx_buf[0] << I3C_CMDQUE_DATBYTE0_Pos);

            default:
                break;
        }

        val |= I3C_CMDATTR_SHORT_DATA_ARG;
        I3C_DrvMsg("[SDAP val: 0x%08x] - I3C_CtrCCCSet\n", val);
        dev->port->CMDQUE = val;
        /* Set transfer command params */
        val = I3C_CMDQUE_SDAP_Msk;
    }
    else if (dev->tx_len > I3C_SDAP_MAX_SIZE)
    {
        p32Buf = (uint32_t *)dev->tx_buf;

        /* Write bytes to tx port */
        for (i = 0UL; i < ((dev->tx_len + 3UL) / 4UL); i++)
        {
            dev->port->TXRXDAT = p32Buf[i];
        }

        /* Program transfer argument */
        val = ((dev->tx_len << I3C_CMDQUE_DATLEN_Pos) | I3C_CMDATTR_TRANSFER_ARG);
        I3C_DrvMsg("[TRANS val: 0x%08x] - I3C_CtrCCCSet\n", val);
        dev->port->CMDQUE = val;
        val = 0;
    }
    else if (dev->tx_len == 0)
    {
        dev->port->CMDQUE = ((0 << I3C_CMDQUE_DATLEN_Pos) | I3C_CMDATTR_TRANSFER_ARG);
    }

    /* Program transfer command */
    val |= (I3C_CMDQUE_CP_Msk
            | (dev->target_index << I3C_CMDQUE_DEVINDX_Pos)
            | (dev->tx_id << I3C_CMDQUE_TID_Pos)
            | (dev->ccc_code << I3C_CMDQUE_CMD_Pos)
            | I3C_CMDATTR_TRANSFER_CMD);

    if (dev->is_last_cmd)
    {
        /* Stop and Response Status on Transfer Completion */
        val |= (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk);
    }

    I3C_DrvMsg("[CMD val: 0x%08x] - I3C_CtrCCCSet\n", val);
    dev->port->CMDQUE = val;
    /* Clear parameters */
    dev->is_last_cmd  = TRUE;
    dev->is_HDR_cmd   = FALSE;
    dev->is_HDRBT_cmd = FALSE;
    dev->is_DB        = FALSE;
    dev->speed_mode   = I3C_DEVI3C_SPEED_SDR0;
    return I3C_STS_NO_ERR;
}

/**
  * @brief  Perform Read CCC Operation
  */
int32_t I3C_CtrCCCGet(I3C_DEVICE_T *dev)
{
    uint32_t val;
    I3C_DrvMsg("\n");
    /* Initialize command response value */
    dev->cmd_response = I3C_CTRRESP_INITIAL_VALUE;
    /* Program transfer argument */
    val = ((dev->rx_len << I3C_CMDQUE_DATLEN_Pos) |
           I3C_CMDATTR_TRANSFER_ARG);

    /* Program Defining Byte Value */
    if (dev->is_DB)
    {
        val |= (dev->DB << I3C_CMDQUE_DB_Pos);
    }

    I3C_DrvMsg("[ DRV ] [ARG val: 0x%08x] - I3C_CtrCCCGet\n", val);
    dev->port->CMDQUE = val;
    /* Program transfer command */
    val = (I3C_CMDQUE_CP_Msk
           | I3C_CMDQUE_RNW_Msk
           | dev->speed_mode
           | (dev->target_index << I3C_CMDQUE_DEVINDX_Pos)
           | (dev->tx_id << I3C_CMDQUE_TID_Pos)
           | (dev->ccc_code << I3C_CMDQUE_CMD_Pos)
           | I3C_CMDATTR_TRANSFER_CMD);

    /* Program Defining Byte Present */
    if (dev->is_DB)
    {
        val |= I3C_CMDQUE_DBP_Msk;
    }

    if (dev->is_last_cmd)
    {
        /* Stop and Response Status on Transfer Completion */
        val |= (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk);
    }

    I3C_DrvMsg("[ DRV ] [CMD val: 0x%08x] - I3C_CtrCCCGet\n", val);
    dev->port->CMDQUE = val;
    /* Clear parameters */
    dev->is_last_cmd  = TRUE;
    dev->is_HDR_cmd   = FALSE;
    dev->is_HDRBT_cmd = FALSE;
    dev->is_DB        = FALSE;
    dev->speed_mode   = I3C_DEVI3C_SPEED_SDR0;
    return I3C_STS_NO_ERR;
}

/**
  * @brief  Perform Private/HDR-DDR Write Oeration in Controller
  */
int32_t I3C_CtrWrite(I3C_DEVICE_T *dev)
{
    volatile uint32_t i;
    uint32_t val;
    uint32_t *p32Buf;
    I3C_DrvMsg("\n");
    /* Initialize command response value */
    dev->cmd_response = I3C_CTRRESP_INITIAL_VALUE;
    val = 0;

    /* Try to use SDAP */
    if ((dev->tx_len > 0UL) && (dev->tx_len <= I3C_SDAP_MAX_SIZE))
    {
        /* Set transfer argument params */
        switch (dev->tx_len)
        {
            case 3:
                val |= (1 << (I3C_CMDQUE_BYTESTRB_Pos + 2));
                val |= (dev->tx_buf[2] << I3C_CMDQUE_DATBYTE2_Pos);

            case 2:
                val |= (1 << (I3C_CMDQUE_BYTESTRB_Pos + 1));
                val |= (dev->tx_buf[1] << I3C_CMDQUE_DATBYTE1_Pos);

            case 1:
                val |= (1 << (I3C_CMDQUE_BYTESTRB_Pos + 0));
                val |= (dev->tx_buf[0] << I3C_CMDQUE_DATBYTE0_Pos);

            default:
                break;
        }

        val |= I3C_CMDATTR_SHORT_DATA_ARG;
        I3C_DrvMsg("[ DRV ] [SDAP val: 0x%08x] - I3C_CtrWrite\n", val);
        dev->port->CMDQUE = val;
        /* Set transfer command params */
        val = I3C_CMDQUE_SDAP_Msk;
    }
    else if (dev->tx_len > I3C_SDAP_MAX_SIZE)
    {
        p32Buf = (uint32_t *)dev->tx_buf;

        /* Write bytes to tx port */
        if (dev->is_DMA)
        {
            /* Use Tx PDAM */
            I3C_ConfigTxDMA(dev, (uint32_t)(p32Buf), (uint32_t)(&dev->port->TXRXDAT), dev->tx_len);
        }
        else
        {
            for (i = 0UL; i < ((dev->tx_len + 3UL) / 4UL); i++)
            {
                dev->port->TXRXDAT = p32Buf[i];
            }
        }

        /* Program transfer argument */
        val = ((dev->tx_len << I3C_CMDQUE_DATLEN_Pos) | I3C_CMDATTR_TRANSFER_ARG);
        I3C_DrvMsg("[ DRV ] [TRANS val: 0x%08x] - I3C_CtrWrite\n", val);
        dev->port->CMDQUE = val;
        val = 0;
    }
    else
    {
    }

    /* Program transfer command */
    val |= (dev->speed_mode
            | (dev->target_index << I3C_CMDQUE_DEVINDX_Pos)
            | (dev->tx_id << I3C_CMDQUE_TID_Pos)
            | I3C_CMDATTR_TRANSFER_CMD);

    /* Program HDR command */
    if (dev->is_HDR_cmd)
    {
        val |= (I3C_CMDQUE_CP_Msk | (dev->ccc_code << I3C_CMDQUE_CMD_Pos));
    }

    if (dev->is_last_cmd)
    {
        /* Stop and Response Status on Transfer Completion */
        val |= (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk);
    }

    I3C_DrvMsg("[ DRV ] [CMD val: 0x%08x] - I3C_CtrWrite\n", val);
    dev->port->CMDQUE = val;
    /* Clear parameters */
    dev->is_last_cmd  = TRUE;
    dev->is_HDR_cmd   = FALSE;
    dev->is_HDRBT_cmd = FALSE;
    dev->is_DB        = FALSE;
    dev->speed_mode   = I3C_DEVI3C_SPEED_SDR0;
    return I3C_STS_NO_ERR;
}

/**
  * @brief  Perform Private/HDR-DDR Read Oeration in Controller
  */
int32_t I3C_CtrRead(I3C_DEVICE_T *dev)
{
    uint32_t val;
    I3C_DrvMsg("\n");

    if (dev->is_DMA)
    {
        /* Enable PDMA channel for I3C Rx function */
        I3C_ConfigRxDMA(dev, (uint32_t)(&dev->port->TXRXDAT), (uint32_t)(dev->rx_buf), (I3C_DEVICE_RX_BUF_CNT * 4));
    }

    /* Initialize command response value */
    dev->cmd_response = I3C_CTRRESP_INITIAL_VALUE;
    /* Program transfer argument */
    val = ((dev->rx_len << I3C_CMDQUE_DATLEN_Pos) |
           I3C_CMDATTR_TRANSFER_ARG);
    I3C_DrvMsg("[ DRV ] [ARG val: 0x%08x] - I3C_CtrRead\n", val);
    dev->port->CMDQUE = val;
    /* Program transfer command */
    val = (I3C_CMDQUE_RNW_Msk
           | dev->speed_mode
           | (dev->target_index << I3C_CMDQUE_DEVINDX_Pos)
           | (dev->tx_id << I3C_CMDQUE_TID_Pos)
           | I3C_CMDATTR_TRANSFER_CMD);

    /* Program HDR command */
    if (dev->is_HDR_cmd)
    {
        val |= (I3C_CMDQUE_CP_Msk | (dev->HDR_cmd << I3C_CMDQUE_CMD_Pos));
    }

    if (dev->is_last_cmd)
    {
        /* Stop and Response Status on Transfer Completion */
        val |= (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk);
    }

    I3C_DrvMsg("[ DRV ] [CMD val: 0x%08x] - I3C_CtrRead\n", val);
    dev->port->CMDQUE = val;
    /* Clear parameters */
    dev->is_last_cmd  = TRUE;
    dev->is_HDR_cmd   = FALSE;
    dev->is_HDRBT_cmd = FALSE;
    dev->is_DB        = FALSE;
    dev->speed_mode   = I3C_DEVI3C_SPEED_SDR0;
    return I3C_STS_NO_ERR;
}

/**
  * @brief  Perform DEFTGTS CCC to Secondary Controller (Target)
  */
int32_t I3C_CtrDEFTGTS(I3C_DEVICE_T *dev)
{
    volatile uint32_t i;
    volatile uint32_t j = 0;
    I3C_DrvMsg("\n");
    /* DEFTGTS CCC */
    /* 1-bytes: Set valid Target counts for Secondary Controller */
    dev->tx_buf[0] = dev->target_count;
    /* 4-bytes: Set Controller's info */
    dev->tx_buf[1] = (dev->main_controller_da << 1); // DA
    dev->tx_buf[2] = ((dev->port->SLVCHAR & I3C_SLVCHAR_DCR_Msk) >> I3C_SLVCHAR_DCR_Pos); // DCR;
    dev->tx_buf[3] = (dev->port->SLVCHAR & 0xFF); // BCR;
    dev->tx_buf[4] = 0x0; // SA

    /* N-bytes: Set valid Target's info */
    for (i = 0; i < dev->tx_buf[0]; i++)
    {
        if (dev->ibi_id == dev->port->TGTCHAR[i].DADDR)
        {
            continue;
        }

        dev->tx_buf[(j * 4) + 5 + 0] = (dev->port->TGTCHAR[i].DADDR << 1); // DA
        dev->tx_buf[(j * 4) + 5 + 1] = (dev->port->TGTCHAR[i].BCRDCR & 0xFF); // DCR;
        dev->tx_buf[(j * 4) + 5 + 2] = ((dev->port->TGTCHAR[i].BCRDCR & 0xFF00) >> 8); // BCR;
        dev->tx_buf[(j * 4) + 5 + 3] = 0x0; // SA
        j++;
    }

    dev->target_index = 0; // for Broadcast CCC
    dev->tx_len       = (1 + 4 + (dev->tx_buf[0] * 4));
    dev->is_last_cmd  = TRUE;
    dev->ccc_code     = I3C_CCC_DEFTGTS; // Not support IRQ
    I3C_CtrCCCSet(dev);

    while ((dev->port->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0UL) {}

    dev->cmd_response = dev->port->RESPQUE;
    I3C_DrvMsg("[ DRV ] DEFTGTS CCC - RespQ status 0x%08x.\n", dev->cmd_response);

    if ((dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk) == I3C_CTRRESP_NO_ERR)
    {
        I3C_DrvMsg("[ DRV ] [ DEFTGTS, PASS ] (RESP: 0x%08x)\n", dev->cmd_response);
    }
    else
    {
        I3C_DrvMsg("[ DRV ] [ DEFTGTS, error code %d ] (RESP: 0x%08x)\n",
                   (uint32_t)((dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk) >> I3C_CTRRESP_ERRSTS_Pos), dev->cmd_response);
        return -1;
    }

    I3C_DrvMsg("\n");
    return I3C_STS_NO_ERR;
}

/**
  * @brief  Perform GETACCCR CCC for Active the Secondary Controller
  */
int32_t I3C_CtrGETACCCR(I3C_DEVICE_T *dev)
{
    uint32_t *p32Buf;
    dev->rx_len       = 1;
    dev->is_last_cmd  = TRUE;
    dev->ccc_code     = I3C_CCC_GETACCCR; // Not support IRQ
    I3C_CtrCCCGet(dev);

    while ((dev->port->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0UL) {}

    dev->cmd_response = dev->port->RESPQUE;
    I3C_DrvMsg("[ DRV ] GETACCCR CCC - RespQ status 0x%08x.\n", dev->cmd_response);

    if (dev->is_DMA)
    {
        /* Enable PDMA channel for I3C Rx function */
        I3C_ConfigRxDMA(dev, (uint32_t)(&dev->port->TXRXDAT), (uint32_t)(dev->rx_buf), (I3C_DEVICE_RX_BUF_CNT * 4));
    }

    if ((dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk) == I3C_CTRRESP_NO_ERR)
    {
        p32Buf = (uint32_t *)&dev->rx_buf[0];

        if (dev->is_DMA == 0)
        {
            p32Buf[0] = dev->port->TXRXDAT;
        }

        if (dev->ibi_id == ((p32Buf[0] & 0xFF) >> 1))
        {
            I3C_DrvMsg("[ DRV ] [ GETACCCR result ] matched: 0x%02x.\n", dev->ibi_id);
        }
        else
        {
            I3C_DrvMsg("[ DRV ] [ GETACCCR result ] mismatch: 0x%02x, 0x%02x\n", dev->ibi_id, ((p32Buf[0] & 0xFF) >> 1));
            return -1;
        }
    }
    else
    {
        I3C_DrvMsg("[ DRV ] [ GETACCCR, error code %d ]\n", (uint32_t)((dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk) >> I3C_CTRRESP_ERRSTS_Pos));
        return -1;
    }

    I3C_DrvMsg("\n");
    return I3C_STS_NO_ERR;
}

/**
  * @brief  Get In-Band Interrupt Event in Controller
  */
int32_t I3C_CtrGetIBI(I3C_DEVICE_T *dev)
{
    volatile uint32_t i;
    uint32_t ibi_len, ibi_id, *p32Buf, word_cnt, RWBit = 0;

    if (dev->ibi_status & I3C_IBIQSTS_NACK)
    {
        I3C_DrvMsg("\n[ DRV ] NACK IBI, status 0x%08x\n", dev->ibi_status);
    }
    else
    {
        ibi_len = ((dev->ibi_status & I3C_IBISTS_DATLEN_Msk) >> I3C_IBISTS_DATLEN_Pos);
        ibi_id  = (((dev->ibi_status & I3C_IBISTS_IBIID_Msk) >> I3C_IBISTS_IBIID_Pos) >> 1);
        RWBit   = ((dev->ibi_status >> I3C_IBISTS_IBIID_Pos) & BIT0);

        if (ibi_len == 0UL)
        {
            /* For Hot-Join request or CR operation */
            if (RWBit == BIT0)
            {
                I3C_DrvMsg("\n[ DRV ] Error Hot-Join or CR request RW bit=%d.\n", RWBit);
                return -1;
            }

            if (ibi_id == 0x2UL)
            {
                dev->ibi_type = I3C_IBI_TYPE_HJ;
                dev->ibi_id   = ibi_id;
                dev->ibi_len  = ibi_len;
#if (0)

                // perform ENTDAA for new Hot-Join Target and update Target's DA table
                for (i = 0UL; i < 7UL; i++)
                {
                    if (dev->target_da[i] != 0x0UL)
                    {
                        i++; // set next target index
                        break;
                    }
                }

                I3C_DrvMsg("\n[ DRV ] Hot-Join ID (0x02) is detected ... process ENTDAA (get idx: %d)\n", i);

                if (i > 7UL)
                {
                    return -2;
                }
                else
                {
                    if (i == 7UL)
                    {
                        i = 0;
                    }

                    dev->target_index = i;  // set ENTDAA index to max. target count
                }

                /* Add delay loop */
                I3C_DelayLoop(SystemCoreClock / 500UL);
                dev->target_count = 7;
                dev->ccc_code     = I3C_CCC_ENTDAA;
                I3C_CtrDAA(dev);

                if (dev->irq_enable)
                {
                    while ((dev->port->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0UL) {}

                    dev->cmd_response = dev->port->RESPQUE;
                }
                else
                {
                    while ((dev->port->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0UL) {}

                    dev->cmd_response = dev->port->RESPQUE;
                }

                if ((dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk) == I3C_CTRRESP_NO_ERR)
                {
                    dev->target_count = dev->target_index + dev->target_count;
                    I3C_DrvMsg("\t[ ENTDAA PASS ] (total cnts: %d)\n", dev->target_count);
                    //i = dev->target_index;
                    //I3C_DrvMsg("\tTarget #%d:\n", dev->target_index);
                    //dev->target_da[i] = ((dev->port->TGTCHAR[i].DADDR & I3C_TGTCHAR4_DADDR_Msk) >> I3C_TGTCHAR4_DADDR_Pos);
                    //I3C_DrvMsg("\t - Provisional ID = 0x%08x%02x \n", dev->port->TGTCHAR[i].PIDMSB, dev->port->TGTCHAR[i].PIDLSB);
                    //I3C_DrvMsg("\t - BCR, DCR       = 0x%08x \n", dev->port->TGTCHAR[i].BCRDCR);
                    //I3C_DrvMsg("\t - DADDR          = 0x%02x \n", dev->target_da[i]);
                }
                else
                {
                    remain_cnts = (((uint32_t)dev->cmd_response & I3C_CTRRESP_DATLEN_Msk) >> I3C_CTRRESP_DATLEN_Pos);

                    if (dev->target_count > remain_cnts)
                    {
                        dev->target_count = dev->target_index + (dev->target_count - remain_cnts);
                        I3C_DrvMsg("\t[ ENTDAA get valid Target, error code %d ]\n", (uint32_t)((dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk) >> I3C_CTRRESP_ERRSTS_Pos));
                    }
                    else
                    {
                        dev->ibi_id = 0x0;
                        dev->target_count = dev->target_index; // ENTDAA fail, and restore target count
                        I3C_DrvMsg("\t[ ENTDAA no valid Target, error code %d ]\n", (uint32_t)((dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk) >> I3C_CTRRESP_ERRSTS_Pos));
                    }

                    I3C_DrvMsg("\tResuming the Controller\n\n");
                    dev->port->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
                }

#endif
            }
            else
            {
                I3C_DrvMsg("\n[ DRV ] Get Target Addr 0x%02x for CR request ...\n", ibi_id);
                dev->ibi_type = I3C_IBI_TYPE_CR;
                dev->ibi_id   = ibi_id;
                dev->ibi_len  = ibi_len;

                // accept Target CR request after Target DA matched
                if (dev->ibi_id != 0UL)
                {
                    // Check if Target's DA matched and send GETACCCR CCC
                    for (i = 0UL; i < 7UL; i++)
                    {
                        I3C_DrvMsg("dev->target_da[%d] = 0x%X\n", i, dev->target_da[i]);

                        if (dev->target_da[i] == dev->ibi_id)
                        {
                            break;
                        }
                    }

                    if (i >= 7UL)
                    {
                        I3C_DrvMsg("No Target's DA matched\n", ibi_id);
                        return -3;    /* No Target's DA matched */
                    }

#if (1)

                    if (dev->ibi_id == dev->main_controller_da)
                    {
                        /*
                            Device Role of Main Controller will return to Controller Role from Target Role.
                            Current active Controller no need to send DEFTGTS CCC.
                        */
                        I3C_DrvMsg("\t[ DRV ] Direct return to Controller\n");
                    }
                    else
                    {
                        //                        /* Perform DEFTGTS CCC while "ibi_id is matched with valid Target DA" */
                        //                        if (I3C_CtrDEFTGTS(dev) != I3C_STS_NO_ERR)
                        //                        {
                        //                            return -4;    /* I3C_DEFTGTS error */
                        //                        }
                    }

#else
                    //                    /* Perform DEFTGTS CCC while "ibi_id is matched with valid Target DA" */
                    //                    if (I3C_CtrDEFTGTS(dev) != I3C_STS_NO_ERR)
                    //                    {
                    //                        return -4;    /* I3C_DEFTGTS error */
                    //                    }
#endif
                    /* Add delay loop */
                    I3C_DelayLoop(SystemCoreClock / 500UL);

                    /* Perform GETACCCR CCC */
                    if (I3C_CtrGETACCCR(dev) != I3C_STS_NO_ERR)
                    {
                        return -5;    /* I3C_DEFTGTS error */
                    }
                }

                // monitor bus owner status
                if (dev->port->INTSTS & I3C_INTSTS_BUSOWNER_UPDATED)
                {
                    dev->port->INTSTS  = I3C_INTSTS_BUSOWNER_UPDATED;
                    dev->intsts       |= I3C_INTSTS_BUSOWNER_UPDATED;
                    dev->port->DEVCTL |= I3C_DEVCTL_RESUME_Msk;

                    while ((dev->port->DEVCTL & I3C_DEVCTL_RESUME_Msk) == I3C_DEVCTL_RESUME_Msk) {}

                    if (I3C_IS_CONTROLLER(dev->port))
                    {
                        I3C_DrvMsg("[ DRV ]I3C role change from Target to Controller\n");
                    }
                    else
                    {
                        I3C_DrvMsg("[ DRV ]I3C role change from Controller to Target\n");
                    }
                }
            }
        }
        else
        {
            /* For In-Band interrupt payload */
            p32Buf   = (uint32_t *)&dev->rx_buf[0];
            word_cnt = (ibi_len + 3UL) / 4UL;

            for (i = 0; i < word_cnt; i++)
            {
                p32Buf[i] = dev->port->IBIQUE;
            }

            I3C_DrvMsg("\n[ DRV ] ibi_id: 0x%02x, len: %d\n\t", ibi_id, ibi_len);

            for (i = 0; i < ibi_len; i++)
            {
                I3C_DrvMsg(" 0x%02x", dev->rx_buf[i]);
            }

            I3C_DrvMsg("\n");
            dev->ibi_type    = I3C_IBI_TYPE_TIR;
            dev->ibi_MDB     = dev->rx_buf[0];
            dev->ibi_id      = ibi_id;
            dev->ibi_len     = ibi_len - 1UL;
            dev->ibi_buf     = (uint8_t *)&dev->rx_buf[1];
            //dev->ibi_payload = p32Buf[0];
        }
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief  Parse Error Condition and Recovery in Controller
  */
void I3C_CtrHandleTransErr(I3C_DEVICE_T *dev)
{
    uint32_t    err_status;
    uint32_t    resume = FALSE;
    uint32_t    TID;
    uint32_t    LEN;
    (void)TID; // for I3C_DrvMsg
    I3C_DrvMsg("\n");
    err_status = (dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk);
    TID        = (((uint32_t)dev->cmd_response & I3C_CTRRESP_TID_Msk) >> I3C_CTRRESP_TID_Pos);
    LEN        = (((uint32_t)dev->cmd_response & I3C_CTRRESP_DATLEN_Msk) >> I3C_CTRRESP_DATLEN_Pos);
    I3C_DrvMsg("[ DRV ] cmd_response 0x%08x.\n", dev->cmd_response);
    I3C_DrvMsg("[ DRV ] Controller error status 0x%08x.\n", err_status);

    switch (err_status)
    {
        case I3C_CTRRESP_CRC_ERR:
            I3C_DrvMsg("\t# Transfer Error: CRC Error occurred in the HDR-DDR or HDR-BT Read Transfer \n");
            resume = TRUE;
            break;

        case I3C_CTRRESP_PARITY_ERR:
            I3C_DrvMsg("\t# Transfer Error: Parity Error occurred in HDR Read Transfers \n");
            resume = TRUE;
            break;

        case I3C_CTRRESP_FRAME_ERR:
            I3C_DrvMsg("\t# Transfer Error: Frame Error occurred in HDR Read Transfers \n");;
            resume = TRUE;
            break;

        case I3C_CTRRESP_BRD_ADDR_NACK_ERR:
            if (dev->ibi_type == I3C_IBI_TYPE_HJ)
            {
                // set dev->target_count in I3C_CtrGetIBI()
            }
            else
            {
                I3C_DrvMsg("\t# Transfer Error: I3C Broadcast Address NACK Error \n");
                I3C_DrvMsg("\tTID %d, remaining device count %d\n", TID, LEN);

                if (dev->target_count >= LEN)
                {
                    dev->target_count = dev->target_count - LEN;
                }
            }

            resume = TRUE;
            break;

        case I3C_CTRRESP_ADDR_NACK_ERR:
            I3C_DrvMsg("\t# Transfer Error: Target Address NACK \n");
            resume = TRUE;
            break;

        case I3C_CTRRESP_FLOW_ERR:
            I3C_DrvMsg("\t# Transfer Error: Receive Buffer Overflow/Transmit Buffer Underflow in HDR Transfers \n");
            resume = TRUE;
            break;

        case I3C_CTRRESP_TRANS_ABORTED_ERR:
            I3C_DrvMsg("\t# Transfer Error: Transfer Aborted \n");
            resume = TRUE;
            break;

        case I3C_CTRRESP_WRITE_NACK_ERR:
            I3C_DrvMsg("\t# Transfer Error: I2C Target Write Data NACK Error \n");
            resume = TRUE;
            break;

        case I3C_CTRRESP_PEC_ERR:
            I3C_DrvMsg("\t# Transfer Error: PEC byte validation error occurs in read transfers \n");
            resume = TRUE;
            break;

        default:
            I3C_DrvMsg("\t# Unkown Error \n");
            resume = TRUE;
            break;
    }

    /* Reset all FIFO */
    dev->port->RSTCTL = (I3C_RSTCTL_RESPRST_Msk | I3C_RSTCTL_RXRST_Msk | I3C_RSTCTL_TXRST_Msk);

    while (dev->port->RSTCTL != 0) {}

    /* Resume Controller if necessary */
    if (resume)
    {
        I3C_DrvMsg("\tResuming the Controller\n\n");
        dev->port->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
    }
}

static void I3C_TgtResetAndResume(I3C_DEVICE_T *dev, uint8_t ExtCmdIdx)
{
    (void)ExtCmdIdx;
    I3C_DrvMsg("\n");
    /* Reset all FIFO -> apply resume */
    dev->port->RSTCTL = (I3C_RSTCTL_RESPRST_Msk | I3C_RSTCTL_RXRST_Msk | I3C_RSTCTL_IBIQRST_Msk);

    while (dev->port->RSTCTL != 0) {}

    dev->port->DEVCTL |= I3C_DEVCTL_RESUME_Msk;

    while ((dev->port->DEVCTL & I3C_DEVCTL_RESUME_Msk) != 0UL) {}

    I3C_DrvMsg("[ DRV ] Target Reset and Resume Completed.\n");
}

/**
  * @brief  Get Target Response Result in Controller Write Operation
  */
int32_t I3C_TgtRecv(I3C_DEVICE_T *dev)
{
    uint8_t             u8TargetID, u8ErrSts;
    uint16_t            u16DataLen;
    volatile uint16_t   i, RxBufIdx;
    volatile uint32_t   u32RespQ;
    uint32_t            *pu32RxBuf;
    (void)u8TargetID;
    I3C_DrvMsg("\n");
    dev->tgtRespQ[0].ErrSts = (uint8_t)I3C_TGTRESP_INITIAL_VALUE;

    if (!(dev->port->INTSTS & I3C_INTSTS_RESPRDY_Msk))
    {
        return I3C_STS_RESPQ_EMPTY;
    }

    RxBufIdx  = 0;
    pu32RxBuf = (uint32_t *)dev->rx_buf;
    dev->tgtRespQ[0].RxBufAddr = (uint32_t)(&pu32RxBuf[0]);
    dev->tgtRespQ[0].RxBufLen  = 0;
    u32RespQ = dev->port->RESPQUE;
    I3C_DrvMsg("M55M1 RESPQUE = 0x%08X\n", u32RespQ);
    u16DataLen  = ((u32RespQ & I3C_TGTRESP_DATLEN_Msk) >> I3C_TGTRESP_DATLEN_Pos);
    u8TargetID = ((u32RespQ & I3C_TGTRESP_TID_Msk) >> I3C_TGTRESP_TID_Pos);
    u8ErrSts   = ((u32RespQ & I3C_TGTRESP_ERRSTS_Msk) >> I3C_TGTRESP_ERRSTS_Pos);

    if (u8ErrSts != I3C_STS_NO_ERR)
    {
        I3C_DrvMsg("\tError RESPQ: 0x%08x (TID: %d) (L-%d)\n", u32RespQ, u8TargetID, __LINE__);
        return (u32RespQ & I3C_TGTRESP_ERRSTS_Msk);
    }

    dev->tgtRespQ[0].TargetID = u8TargetID;
    dev->tgtRespQ[0].ErrSts   = u8ErrSts;

    if (dev->is_DMA)
    {
        /* Use PDAM RX */
    }
    else
    {
        for (i = 0; i < ((u16DataLen + 3) / 4); i++, RxBufIdx++)
        {
            pu32RxBuf[RxBufIdx] = dev->port->TXRXDAT;
            //if( (u32RespQ & I3C_TGTRESP_CCCWR_Msk) ) // for CCC Write operation
            //    I3C_DrvMsg("\tRX: 0x%08x\n", pu32RxBuf[RxBufIdx]);
        }
    }

    I3C_DrvMsg("[ DRV ] RESPQ: 0x%08x (TID: %d) (#1)\n", u32RespQ, u8TargetID);
    dev->tgtRespQ[0].RxBufLen += u16DataLen;
    return I3C_STS_NO_ERR;
}

/**
  * @brief  Prepare Target Transmit Command Data for Controller Read Operation
  */
int32_t I3C_TgtSend(I3C_DEVICE_T *dev)
{
    volatile uint32_t i;
    uint32_t txlen     = dev->tx_len;
    uint32_t *p32Buf;
    I3C_DrvMsg("\n");
    /* Push data to EXT CMD TX Buffer */
    p32Buf = (uint32_t *)dev->tx_buf;

    if (dev->is_DMA)
    {
        /* Use Tx PDAM */
        I3C_ConfigTxDMA(dev, (uint32_t)(p32Buf), (uint32_t)&dev->port->TXRXDAT, txlen);
    }
    else
    {
        for (i = 0; i < ((txlen + 3) / 4); i++)
        {
            dev->port->TXRXDAT = p32Buf[i];
        }
    }

    uint32_t CMDQUE     = ((0 << I3C_CMDQUE_TID_Pos) | (txlen << I3C_CMDQUE_DATLEN_Pos));
    I3C_DrvMsg("[ DRV ] [CMD val: 0x%08X] - I3C_CtrWrite\n", CMDQUE);
    dev->port->CMDQUE = CMDQUE;
    /*
        User need to parse Extended Command Status in EXTCMD.WORD1[15:8] while EXTFINS set to 1 (EXTCMD Has Finished Status).
        Refer to I3C_TgtGetSendResult(...) API.
    */
    dev->ccc_code     = 0x0;
    dev->is_DB        = FALSE;
    dev->is_HDR_cmd   = FALSE;
    dev->is_HDRBT_cmd = FALSE;
    return I3C_STS_NO_ERR;
}

/**
  * @brief  Get Target Extended Transmit Command Result in Controller Read Operation
  */
int32_t I3C_TgtGetSendResult(I3C_DEVICE_T *dev)
{
    (void)dev;
    I3C_DrvMsg("\n");
    return I3C_STS_NO_ERR;
}

/**
  * @brief  Issue In-Band Interrupt Event in Target
  */
int32_t I3C_TgtIssueIBI(I3C_DEVICE_T *dev)
{
    (void)dev;
#if (1)
    I3C_DrvMsg("\n");

    if ((dev->port->SLVEVNTS & I3C_SLVEVNTS_SIREN_Msk) == 0UL)
    {
        I3C_DrvMsg("[ DRV ] ERROR. Target Interrupt Request NOT Enabled. \n\n");
        return I3C_STS_INVALID_INPUT;
    }

    if ((dev->port->SIR & (I3C_SIR_EN_Msk | I3C_SIR_MR_Msk)) != 0UL)
    {
        I3C_DrvMsg("[ DRV ] ERROR. SIR pending, 0x%x. \n\n", dev->port->SIR);
        return I3C_STS_INVALID_INPUT;
    }

    if (dev->target_index > 4UL)
    {
        I3C_DrvMsg("[ DRV ] ERROR. Invalid Target index, 0x%x. \n\n", dev->target_index);
        return I3C_STS_INVALID_INPUT;
    }

    switch (dev->ibi_type)
    {
        case I3C_IBI_TYPE_TIR:
            break;

        case I3C_IBI_TYPE_CR:
            if ((dev->port->SLVEVNTS & I3C_SLVEVNTS_MREN_Msk) == 0)
            {
                I3C_DrvMsg("[ DRV ] ERROR. Controller Request NOT Enabled.\n\n");
                return I3C_STS_INVALID_INPUT;
            }

            /* Support "ACK GETACCCR CCC" */
            dev->port->DEVCTLE &= ~(I3C_DEVCTLE_MRACKCTL_Msk);
            /* Trigger MR request */
            dev->port->SIR |= I3C_SIR_MR_Msk;
            /* Bus Owner Updated event in Target's IRQ Handler */
            /* The Controller sends GETACCCR CCC (Get Accept Controller Role) while received Controller request from Target */
            return I3C_STS_NO_ERR;
            break;

        default:
            return I3C_STS_INVALID_INPUT;
            break;
    }

    /* Check if payload length > 4-bytes */
    if (dev->ibi_len > 4UL)
    {
        return I3C_STS_INVALID_INPUT;
    }

    /* Program IBI payload data, payload length and MDB */
    dev->port->SIR    = ((dev->ibi_len << I3C_SIR_DATLEN_Pos) | (dev->ibi_MDB << I3C_SIR_MDB_Pos) | (0 << I3C_SIR_CTL_Pos));
    dev->port->SIRDAT = dev->ibi_payload;
    /* Trigger IBI request */
    /* SIR EN bit be cleared automatically after the Controller accepts the IBI request or Target unable to issue the IBI request */
    dev->port->SIR |= I3C_SIR_EN_Msk;
#endif
    return I3C_STS_NO_ERR;
}

/**
  * @brief  Parse Error Condition and Recovery in Target
  */
void I3C_TgtHandleTransErr(I3C_DEVICE_T *dev)
{
    uint32_t dev_status;
    uint32_t err_status;
    I3C_DrvMsg("\n");
    dev_status = dev->port->CCCDEVS;
    I3C_DrvMsg("[ DRV ] Target device status 0x%08x.\n", dev_status);

    if (dev_status)
    {
        uint32_t resume = FALSE;

        if (dev_status & I3C_CCCDEVS_SLVBUSY_Msk)
        {
            I3C_DrvMsg("\t#Target busy status\n");
            resume = TRUE;
        }

        if (dev_status & I3C_CCCDEVS_FRAMEERR_Msk)
        {
            I3C_DrvMsg("\t# Dev frame error\n");
        }

        if (dev_status & I3C_CCCDEVS_BFNAVAIL_Msk)
        {
            I3C_DrvMsg("\t# Dev buffer not available\n");
        }

        if (dev_status & I3C_CCCDEVS_DATNRDY_Msk)
        {
            I3C_DrvMsg("\t# Dev data not ready\n");
        }

        if (dev_status & I3C_CCCDEVS_OVFERR_Msk)
        {
            I3C_DrvMsg("\t# Dev overflow error\n");
        }

        if (dev_status & I3C_CCCDEVS_UDFERR_Msk)
        {
            I3C_DrvMsg("\t# Dev underflow error\n");
        }

        if (dev_status & I3C_CCCDEVS_PROTERR_Msk)
        {
            I3C_DrvMsg("\t# Dev protocol error\n");
        }

        I3C_DrvMsg("\t# Activity Mode:[0x%x] / Pending Interrupt:[0x%x]",
                   (uint32_t)((dev_status & I3C_CCCDEVS_ACTMODE_Msk) >> I3C_CCCDEVS_ACTMODE_Pos),
                   (uint32_t)((dev_status & I3C_CCCDEVS_PENDINT_Msk) >> I3C_CCCDEVS_PENDINT_Pos));

        /* Resume Target if necessary */
        if (resume)
        {
            I3C_DrvMsg("\tResuming the Target\n\n");
            dev->port->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
        }
    }

    err_status = dev->tgtRespQ[0].ErrSts;
    I3C_DrvMsg("[ DRV ] Target response error status 0x%08x.\n", err_status);

    switch (err_status)
    {
        case I3C_TGTRESP_CRC_ERR:
            I3C_DrvMsg("\t# Transfer Error: CRC Error (Controller write in DDR mode) \n");
            break;

        case I3C_TGTRESP_PARITY_ERR:
            I3C_DrvMsg("\t# Transfer Error: Parity Error (Controller write in both DDR and SDR mode) \n");
            break;

        case I3C_TGTRESP_FRAME_ERR:
            I3C_DrvMsg("\t# Transfer Error: Frame Error (Controller write in HDR mode) \n");;
            break;

        case I3C_TGTRESP_FLOW_ERR:
            I3C_DrvMsg("\t# Transfer Error: Underflow/Overflow Error \n");
            break;

        case I3C_TGTRESP_CONTROLLER_TERMINATE_ERR:
            I3C_DrvMsg("\t# Transfer Error: Controller early terminal Error \n");
            break;

        default:
            I3C_DrvMsg("\t# Unkown Error \n");
            break;
    }

    I3C_DrvMsg("\n");
}

/**
  * @brief  Handle Target Interrupt Status
  */
void I3C_TgtHandleIntSts(I3C_DEVICE_T *dev)
{
    I3C_DrvMsg("\n");

    if (dev->intsts & I3C_INTSTS_DA_ASSIGNED)
    {
        /* Main Target Address */
        if (dev->port->DEVADDR & I3C_DEVADDR_DAVALID_Msk)
        {
            I3C_DrvMsg("[ DRV ] Set to I3C mode, DA: 0x%02x.\n", (uint8_t)((dev->port->DEVADDR & I3C_DEVADDR_DA_Msk) >> I3C_DEVADDR_DA_Pos));
            dev->target_da[0] = (uint8_t)((dev->port->DEVADDR & I3C_DEVADDR_DA_Msk) >> I3C_DEVADDR_DA_Pos);
        }
        else
        {
            I3C_DrvMsg("[ DRV ] Set to I2C mode, SA: 0x%02x.\n", (uint8_t)((dev->port->DEVADDR & I3C_DEVADDR_SA_Msk) >> I3C_DEVADDR_SA_Pos));
            dev->target_da[0] = 0x0;
            dev->target_sa[0] = (uint8_t)((dev->port->DEVADDR & I3C_DEVADDR_SA_Msk) >> I3C_DEVADDR_SA_Pos);
        }
    }

    if (dev->intsts & I3C_INTSTS_CCC_UPDATED)
    {
        if (dev->port->SLVEVNTS & I3C_SLVEVNTS_MWLUPD_Msk)
        {
            dev->port->SLVEVNTS = I3C_SLVEVNTS_MWLUPD_Msk;
            I3C_DrvMsg("[ DRV ] Updated MWL to 0x%x.\n", (uint32_t)((dev->port->SLVMXLEN & I3C_SLVMXLEN_MWL_Msk) >> I3C_SLVMXLEN_MWL_Pos));
        }
        else if (dev->port->SLVEVNTS & I3C_SLVEVNTS_MRLUPD_Msk)
        {
            dev->port->SLVEVNTS = I3C_SLVEVNTS_MRLUPD_Msk;
            I3C_DrvMsg("[ DRV ] Updated MRL to 0x%x.\n",
                       (uint32_t)((dev->port->SLVMXLEN & I3C_SLVMXLEN_MRL_Msk) >> I3C_SLVMXLEN_MRL_Pos));
            I3C_TgtResetAndResume(dev, 0xFF);
        }
        else
        {
            I3C_DrvMsg("[ DRV ] Updated - ENTAS%d.\n", (uint32_t)((dev->port->SLVEVNTS & I3C_SLVEVNTS_ACTSTATE_Msk) >> I3C_SLVEVNTS_ACTSTATE_Pos));
            I3C_DrvMsg("[ DRV ] Updated - HJEN %d.\n", (uint32_t)((dev->port->SLVEVNTS & I3C_SLVEVNTS_HJEN_Msk) >> I3C_SLVEVNTS_HJEN_Pos));
            I3C_DrvMsg("[ DRV ] Updated - SIREN %d.\n", (uint32_t)((dev->port->SLVEVNTS & I3C_SLVEVNTS_SIREN_Msk) >> I3C_SLVEVNTS_SIREN_Pos));
        }
    }

    I3C_DrvMsg("\n");
}

/** @} end of group I3C_EXPORTED_FUNCTIONS */
/** @} end of group I3C_Driver */
/** @} end of group Standard_Driver */
