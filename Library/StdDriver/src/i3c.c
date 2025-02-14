/**************************************************************************//**
 * @file     i3c.c
 * @version  V1.00
 * @brief    I3C driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "NuMicro.h"

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
  * @brief      Open I3C with Static Address and Initial Mode
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  * @param[in]  u32MasterSlave  Decides the I3C module is operating in master mode or in slave mode. Valid values are:
  *                                 - \ref I3C_SLAVE
  *                                 - \ref I3C_MASTER
  * @param[in]  u8StaticAddr    7-bit slave address for I2C operation.
  * @param[in]  u32ModeSel      Initial mode selection to support ENTDAA CCC or Hot-Join generation. Valid values are:
  *                                 - \ref I3C_SUPPORT_ENTDAA
  *                                 - \ref I3C_SUPPORT_ADAPTIVE_HJ
  *                                 - \ref I3C_SUPPORT_IMMEDIATE_HJ
  *
  * @details    This API is used to configure I3C controller can receive ENTDAA CCC or generate a Hot-Join request.
  *             The 7-bit slave address is used for related I2C operations until I3C controller has a valid Dynamic Address.
  * @note       This API also enables all interrupt status and configures TX, RX FIFO and Response, Command Queue threshold to 0.
  */
void I3C_Open(I3C_T *i3c,
              uint32_t u32MasterSlave,
              uint8_t u8StaticAddr,
              uint32_t u32ModeSel)
{
    i3c->DEVCTLE = u32MasterSlave; // [0] 0: master, 1: slave
    /* Enable all interrupt status */
    i3c->INTSTSEN = 0xFFFFFFFF;
    /* Set Tx/Rx/CmdQ/ReapQ threshold to 0 */
    i3c->DBTHCTL = 0x0;
    i3c->QUETHCTL = 0x0;

    if (u32MasterSlave == I3C_MASTER)
    {
        // In Master mode, self-assigns its dynamic address.
        i3c->DEVADDR = ((u8StaticAddr << I3C_DEVADDR_SA_Pos) | I3C_DEVADDR_SAVALID_Msk)
                       | ((u8StaticAddr << I3C_DEVADDR_DA_Pos) | I3C_DEVADDR_DAVALID_Msk);
        return;
    }
    else if (u8StaticAddr == 0)
    {
        i3c->DEVADDR &= ~I3C_DEVADDR_SAVALID_Msk;
    }
    else
    {
        i3c->DEVADDR &= ~I3C_DEVADDR_SA_Msk;
        i3c->DEVADDR |= (I3C_DEVADDR_SAVALID_Msk | u8StaticAddr);
    }

    /* Select the slave ability for enter I3C mode */
    if (u32ModeSel == I3C_SUPPORT_ENTDAA)
    {
        /* HJEN disabled: Slave supports ENTDAA CCC */
        i3c->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
        i3c->SLVEVNTS &= ~I3C_SLVEVNTS_HJEN_Msk;
    }
    else if (u32ModeSel == I3C_SUPPORT_ADAPTIVE_HJ)
    {
        /* Both ADAPTIVE and HJEN enabled: Slave generates a Hot-Join request until receive I3C header 7'h7E on the bus */
        i3c->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
        i3c->SLVEVNTS |= I3C_SLVEVNTS_HJEN_Msk;
    }
    else if (u32ModeSel == I3C_SUPPORT_IMMEDIATE_HJ)
    {
        /* Only HJEN enabled: Slave generates a Hot-Join request immediately */
        i3c->DEVCTL &= ~I3C_DEVCTL_ADAPTIVE_Msk;
        i3c->SLVEVNTS |= I3C_SLVEVNTS_HJEN_Msk;
    }
    else
    {
        /* HJEN disabled: Slave supports ENTDAA CCC */
        i3c->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
        i3c->SLVEVNTS &= ~I3C_SLVEVNTS_HJEN_Msk;
    }
}

/**
  * @brief      Reset and Resume I3C Controller
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  * @param[in]  u32ResetMask    Software reset operation of I3C module. Valid values are:
  *                                 - \ref I3C_RESET_CMD_QUEUE
  *                                 - \ref I3C_RESET_RESP_QUEUE
  *                                 - \ref I3C_RESET_TX_BUF
  *                                 - \ref I3C_RESET_RX_BUF
  *                                 - \ref I3C_RESET_ALL_QUEUE_AND_BUF
  * @param[in]  u32EnableResume Enable resume I3C Slave controller. Valid values are TRUE and FALSE.
  *
  * @retval     I3C_STS_NO_ERR      No error
  * @retval     I3C_TIMEOUT_ERR     Function time-out
  *
  * @details    This API is used to reset specified FIFO and Queue or resume the I3C Slave controller from the halt state.
  * @note       THe application can reset Queues and FIFO only when the Slave controller is disabled or
  *             when the Slave controller is in Halt state(I3C_CCCDEVS[9] SLVBUSY = 1) after any error occurred from the I3C Master request.
  */
int32_t I3C_ResetAndResume(I3C_T *i3c, uint32_t u32ResetMask, uint32_t u32EnableResume)
{
    uint8_t u8InHaltState = 0;
    volatile uint32_t u32Timeout;

    if (I3C_IS_SLAVE_BUSY(i3c))
    {
        u8InHaltState = 1;
    }

    if (u32ResetMask)
    {
        if (u8InHaltState == 0)
        {
            /* Disable I3C controller for reset buffer and queue */
            if (I3C_Disable(i3c) != I3C_STS_NO_ERR)
            {
                return I3C_TIMEOUT_ERR;
            }
        }

        /* Reset specify source */
        i3c->RSTCTL = u32ResetMask;
        u32Timeout = (SystemCoreClock / 1000);

        while ((i3c->RSTCTL != 0) && (--u32Timeout)) {}

        if (u32Timeout == 0)
        {
            return I3C_TIMEOUT_ERR;
        }

        if (u8InHaltState == 0)
        {
            /* Enable I3C controller again */
            if (I3C_Enable(i3c) != I3C_STS_NO_ERR)
            {
                return I3C_TIMEOUT_ERR;
            }
        }
    }

    if (u32EnableResume || u8InHaltState)
    {
        /* The application has to take necessary action to handle the error condition and
            then set RESUME bit to resume the controller. */
        /* Slave will receive GETSTATUS CCC to clear specify status in I3C_CCCDEVS register. */
        i3c->DEVCTL |= I3C_DEVCTL_RESUME_Msk;

        while ((i3c->DEVCTL & I3C_DEVCTL_RESUME_Msk) == I3C_DEVCTL_RESUME_Msk) {}

        /* RESUME bit is auto-cleared once the controller is ready to accept new transfers. */
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief      Get Resopnse Status and Receive Data
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  * @param[out] pu32RespQ       The response data structure to get the response data.
  *
  * @retval     I3C_STS_NO_ERR          No error
  * @retval     I3C_STS_INVALID_INPUT   Invalid input parameter
  * @retval     I3C_STS_RESPQ_EMPTY     No Response Queue data
  * @return     Response error status
  *
  * @details    This API is used to get the response data and the received data on Master write operation.
  */
int32_t I3C_ParseRespQueue(I3C_T *i3c, uint32_t *pu32RespQ)
{
    uint8_t qn, u8RespQCnt;

    /* Check if RespQ buffer is empty */
    if (pu32RespQ == NULL)
    {
        return I3C_STS_INVALID_INPUT;
    }

    /* Check if RespQ is empty */
    if (I3C_IS_RESPQ_EMPTY(i3c))
    {
        return I3C_STS_RESPQ_EMPTY;
    }

    u8RespQCnt = I3C_GET_RESPQ_THLD(i3c) + 1;
    qn = 0; // Queue number

    do
    {
        pu32RespQ[qn] = I3C_GET_RESP_DATA(i3c);

        if (!I3C_IS_RESP_NO_ERR(pu32RespQ[qn]))
        {
            return (pu32RespQ[qn] & I3C_RESPQUE_ERRSTS_Msk);
        }

        qn++;
        u8RespQCnt--;
    } while (u8RespQCnt);

    return I3C_STS_NO_ERR;
}

/**
  * @brief      Set Command Queue and Transmit Data for Master Read
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  * @param[in]  u8TID           Specified Transmit Transaction ID in Command Queue.
  * @param[in]  pu32TxBuf       The buffer to send the data to transmit FIFO.
  * @param[in]  u16WriteBytes   The byte number of TX data.
  *
  * @retval     I3C_STS_NO_ERR          No error
  * @retval     I3C_STS_INVALID_INPUT   Invalid input parameter
  * @retval     I3C_STS_CMDQ_FULL       Command Queue is full
  * @retval     I3C_STS_TX_FULL         TX FIFO is full
  *
  * @details    This API is used to prepare a Command Queue and TX response data for Master read operation.
  */
int32_t I3C_SetCmdQueueAndData(I3C_T *i3c, uint8_t u8TID, uint32_t *pu32TxBuf, uint16_t u16WriteBytes)
{
    uint32_t i;

    /* Check if write bytes is exceeded */
    if (u16WriteBytes > (I3C_DEVICE_TX_BUF_CNT * 4))
    {
        return I3C_STS_INVALID_INPUT;
    }

    /* Check if CmdQ is full */
    if (I3C_IS_CMDQ_FULL(i3c))
    {
        return I3C_STS_CMDQ_FULL;
    }

    if (pu32TxBuf != NULL)
    {
        for (i = 0; i < ((u16WriteBytes + 3) / 4); i++)
        {
            /* Check if TX buffer is full */
            if (I3C_IS_TX_FULL(i3c))
            {
                return I3C_STS_TX_FULL;
            }

            i3c->TXRXDAT = pu32TxBuf[i];
        }
    }

    i3c->CMDQUE = ((u8TID << I3C_CMDQUE_TID_Pos) | (u16WriteBytes << I3C_CMDQUE_DATLEN_Pos));
    return I3C_STS_NO_ERR;
}

/**
  * @brief      Generate IBI Request
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  * @param[in]  u8MandatoryData The mandatory data byte.
  * @param[in]  u32PayloadData  The payload data.
  * @param[in]  u8PayloadLen    The byte number of payload data. The maximum length is 4 bytes.
  *
  * @retval     I3C_STS_NO_ERR              No error, IBI request accepted by the Master
  * @retval     I3C_STS_INVALID_STATE       Invalid state
  * @retval     I3C_STS_INVALID_INPUT       Invalid input parameter
  *
  * @details    This API is used to generate an IBI request on the bus.
  */
int32_t I3C_SendIBIRequest(I3C_T *i3c, uint8_t u8MandatoryData, uint32_t u32PayloadData, uint8_t u8PayloadLen)
{
    /* Check if Controller is in busy state */
    if (I3C_IS_SLAVE_BUSY(i3c))
    {
        return I3C_STS_INVALID_STATE;
    }

    /* Check if SIR function enabled */
    if (!(i3c->SLVEVNTS & I3C_SLVEVNTS_SIREN_Msk))
    {
        return I3C_STS_INVALID_STATE;
    }

    /* Check if payload length > 4-bytes */
    if (u8PayloadLen > 4)
    {
        return I3C_STS_INVALID_INPUT;
    }

    /* Program IBI payload data, payload length and MDB */
    i3c->SIR = (u8PayloadLen << I3C_SIR_DATLEN_Pos) | (u8MandatoryData << I3C_SIR_MDB_Pos) | (0 << I3C_SIR_CTL_Pos);
    i3c->SIRDAT = u32PayloadData;
    /* Trigger IBI request */
    /* SIR_EN bit be cleared automatically after the Master accepts the IBI request or Slave unable to issue the IBI request */
    i3c->SIR |= I3C_SIR_EN_Msk;
    return I3C_STS_NO_ERR;
}

/**
  * @brief      Generate MR Request
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  *
  * @retval     I3C_STS_NO_ERR              No error, IBI request accepted by the Master
  * @retval     I3C_STS_INVALID_STATE       Invalid state
  *
  * @details    This API is used to generate an IBI request on the bus.
  */
int32_t I3C_SendMRRequest(I3C_T *i3c)
{
    /* Check if Controller is in busy state */
    if (I3C_IS_SLAVE_BUSY(i3c))
    {
        return I3C_STS_INVALID_STATE;
    }

    /* Check if MR function enabled */
    if (!(i3c->SLVEVNTS & I3C_SLVEVNTS_MREN_Msk))
    {
        return I3C_STS_INVALID_STATE;
    }

    /* Trigger MR request */
    i3c->SIR |= I3C_SIR_MR_Msk;
    return I3C_STS_NO_ERR;
}

/**
  * @brief      Disable HJ Generation
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  *
  * @retval     I3C_STS_NO_ERR      No error
  * @retval     I3C_TIMEOUT_ERR     Enable/Disable I3C time-out
  *
  * @details    This API is used to disable the HJ generation.
  * @note       I3C Slave controller can recognize the ENTDAA CCC after disabling the HJ generation.
  */
int32_t I3C_DisableHJRequest(I3C_T *i3c)
{
    /* HJEN disabled: Slave supports ENTDAA CCC */
    i3c->SLVEVNTS &= ~I3C_SLVEVNTS_HJEN_Msk;
    i3c->DEVCTL |= I3C_DEVCTL_ADAPTIVE_Msk;
    return I3C_STS_NO_ERR;
}

/**
  * @brief      I3C Response Error Recovery
  *
  * @param[in]  i3c             The pointer of the specified I3C module.
  * @param[in]  u32RespStatus   Response error status from the response queue.
  *
  * @return     I3C_STS_NO_ERR
  *
  * @details    This API is used to perform error recovery then the I3C Slave controller can leave Halt(Busy) state.
  * @note       The RESUME operation is completed after a GETSTATUS CCC is received or the specified Master operation is successfully.
  */
int32_t I3C_RespErrorRecovery(I3C_T *i3c, uint32_t u32RespStatus)
{
    if (u32RespStatus != I3C_STS_NO_ERR)
    {
        if (I3C_IS_SLAVE_BUSY(i3c))
        {
            switch (u32RespStatus)
            {
                case I3C_RESP_CRC_ERR:
                case I3C_RESP_PARITY_ERR:
                case I3C_RESP_FRAME_ERR:
                case I3C_RESP_FLOW_ERR:
                    /* Reset RX FIFO -> apply resume */
                    I3C_ResetAndResume(i3c, I3C_RESET_RX_BUF, TRUE);
                    break;

                case I3C_RESP_MASTER_TERMINATE_ERR:
                    /* Reset TX FIFO and CMDQ Queue -> apply resume */
                    I3C_ResetAndResume(i3c, (I3C_RESET_TX_BUF | I3C_RESET_CMD_QUEUE), TRUE);
                    break;

                default:
                    /* Reset all FIFO and Queue */
                    I3C_ResetAndResume(i3c, I3C_RESET_ALL_QUEUE_AND_BUF, FALSE);
                    break;
            }
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
  * @param[in]  u8DevType       the slave device type. It could be I3C_DEVTYPE_I3C for I3C device and I3C_DEVTYPE_I2C for I2C device
  * @param[in]  u8DAddr         7Bits Device Synamic Address
  * @param[in]  u8SAddr         7Bits Device Static Address
  *
  * @retval     I3C_STS_NO_ERR          No error
  * @retval     I3C_STS_INVALID_INPUT   Invalid input parameter
  *
  * @details    The function is used for I3C Master to setup Device Address Table.
  * @note       Device Address Table must be set before communication with Slave Devices.
  *
  */
int32_t I3C_SetDeviceAddr(I3C_T *i3c, uint8_t u8DevIndex, uint8_t u8DevType, uint8_t u8DAddr, int8_t u8SAddr)
{
    uint32_t i, count, u32Device = 0;

    if ((u8DAddr & 0x80) || (u8SAddr & 0x80))
    {
        return I3C_STS_INVALID_INPUT;
    }

    // I3C Device Dynamic Address with Odd Parity
    for (i = 0, count = 0; i < 8; i++)
    {
        if ((u8DAddr >> i) & 0x1)
        {
            count ++;
        }
    }

    if ((count % 2) == 0)
    {
        u32Device = ((u8DAddr | 0x80) << I3C_DEVADR_DADDR_Pos);
    }
    else
    {
        u32Device = (u8DAddr << I3C_DEVADR_DADDR_Pos);
    }

    // I2C Device Static Address
    u32Device |= (u8SAddr << I3C_DEVADR_SADDR_Pos);

    // Device Type
    if (u8DevType != I3C_DEVTYPE_I3C)
    {
        u32Device |= I3C_DEVADR_DEVICE_Msk;
    }

    switch (u8DevIndex)
    {
        case 0ul:
            i3c->DEV1ADR = u32Device;
            break;

        case 1ul:
            i3c->DEV2ADR = u32Device;
            break;

        case 2ul:
            i3c->DEV3ADR = u32Device;
            break;

        case 3ul:
            i3c->DEV4ADR = u32Device;
            break;

        case 4ul:
            i3c->DEV5ADR = u32Device;
            break;

        case 5ul:
            i3c->DEV6ADR = u32Device;
            break;

        case 6ul:
            i3c->DEV7ADR = u32Device;
            break;

        default:
            return I3C_STS_INVALID_INPUT;
            break;
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief      Write data to Slave
  *
  * @param[in]  *i3c            Point to I3C peripheral
  * @param[in]  u8DevIndex      the offset of Device Address Table.
  *                             It could be 0 ~ 6 for DEV1ADR to DEV7ADR.
  * @param[in]  u32Speed        the speed in which the transfer should be driven. It could be
  *                                 \ref I3C_DEVI3C_SPEED_SDR0
  *                                 \ref I3C_DEVI3C_SPEED_SDR1
  *                                 \ref I3C_DEVI3C_SPEED_SDR2
  *                                 \ref I3C_DEVI3C_SPEED_SDR3
  *                                 \ref I3C_DEVI3C_SPEED_SDR4
  *                                 \ref I3C_DEVI3C_SPEED_HDRDDR
  *                                 \ref I3C_DEVI3C_SPEED_I2CFM
  *                                 \ref I3C_DEVI2C_SPEED_I2CFM
  *                                 \ref I3C_DEVI2C_SPEED_I2CFMPLUS
  * @param[in]  *pu32TxBuf      Pointer to array to write data to Slave
  * @param[in]  u16WriteBytes   How many bytes need to write to Slave
  *
  * @retval     I3C_STS_NO_ERR          No error
  * @retval     I3C_STS_INVALID_INPUT   Invalid input parameter
  * @retval     I3C_STS_CMDQ_FULL       Command Queue is full
  * @retval     I3C_STS_TX_FULL         TX FIFO is full
  *
  * @details    The function is used for I3C Master write data to Slave.
  *
  * @note       Device Address Table must be set before using this function.
  *
  */
int32_t I3C_Write(I3C_T *i3c, uint8_t u8DevIndex, uint32_t u32Speed, uint32_t *pu32TxBuf, uint16_t u16WriteBytes)
{
    uint32_t i;
    uint32_t u32TimeOutCount = 0u;
    uint32_t response;

    if ((u16WriteBytes == 0) || (pu32TxBuf == NULL))
    {
        return I3C_STS_INVALID_INPUT;
    }

    /* Check if CmdQ is full */
    if (I3C_IS_CMDQ_FULL(i3c))
    {
        return I3C_STS_CMDQ_FULL;
    }

    if (u16WriteBytes <= 3)
    {
        i3c->CMDQUE = (((pu32TxBuf[0] & 0x00FFFFFF) << I3C_CMDQUE_DATBYTE0_Pos)
                       | ((((1 << u16WriteBytes) - 1) & 0x07) << I3C_CMDQUE_BYTESTRB_Pos)
                       | I3C_CMDATTR_SHORT_DATA_ARG);

        /* Check if CmdQ is full */
        if (I3C_IS_CMDQ_FULL(i3c))
        {
            return I3C_STS_CMDQ_FULL;
        }

        i3c->CMDQUE = (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_SDAP_Msk | I3C_CMDQUE_ROC_Msk
                       | (u32Speed & I3C_CMDQUE_SPEED_Msk)
                       | ((u8DevIndex & 0x1F) << I3C_CMDQUE_DEVINDX_Pos)
                       | (I3C_TX_TID << I3C_CMDQUE_TID_Pos)
                       | I3C_CMDATTR_TRANSFER_CMD);
    }
    else
    {
        i3c->CMDQUE = ((u16WriteBytes << I3C_CMDQUE_DATLEN_Pos) | I3C_CMDATTR_TRANSFER_ARG);

        /* Check if CmdQ is full */
        if (I3C_IS_CMDQ_FULL(i3c))
        {
            return I3C_STS_CMDQ_FULL;
        }

        i3c->CMDQUE = (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk
                       | (u32Speed & I3C_CMDQUE_SPEED_Msk)
                       | ((u8DevIndex & 0x1F) << I3C_CMDQUE_DEVINDX_Pos)
                       | (I3C_TX_TID << I3C_CMDQUE_TID_Pos)
                       | I3C_CMDATTR_TRANSFER_CMD);

        for (i = 0; i < ((u16WriteBytes + 3) / 4); i++)
        {
            u32TimeOutCount = SystemCoreClock;

            /* Check if TX buffer is full */
            while (I3C_IS_TX_FULL(i3c))
            {
                if (--u32TimeOutCount == 0)
                {
                    return I3C_STS_TX_FULL;
                }
            }

            i3c->TXRXDAT = pu32TxBuf[i];
        }
    }

    while ((i3c->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0);

    response = i3c->RESPQUE;

    if (response & I3C_RESPQUE_ERRSTS_Msk)
    {
        i3c->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
        return I3C_STS_INVALID_STATE;
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief      Read data from Slave
  *
  * @param[in]  *i3c            Point to I3C peripheral
  * @param[in]  u8DevIndex      the offset of Device Address Table.
  *                             It could be 0 ~ 6 for DEV1ADR to DEV7ADR.
  * @param[in]  u32Speed        the speed in which the transfer should be driven. It could be
  *                                 \ref I3C_DEVI3C_SPEED_SDR0
  *                                 \ref I3C_DEVI3C_SPEED_SDR1
  *                                 \ref I3C_DEVI3C_SPEED_SDR2
  *                                 \ref I3C_DEVI3C_SPEED_SDR3
  *                                 \ref I3C_DEVI3C_SPEED_SDR4
  *                                 \ref I3C_DEVI3C_SPEED_HDRDDR
  *                                 \ref I3C_DEVI3C_SPEED_I2CFM
  *                                 \ref I3C_DEVI2C_SPEED_I2CFM
  *                                 \ref I3C_DEVI2C_SPEED_I2CFMPLUS
  * @param[in]  *pu32RxBuf      Pointer to array to read data from Slave
  * @param[in]  u16ReadBytes    How many bytes need to read from Slave
  *
  * @retval     I3C_STS_NO_ERR          No error
  * @retval     I3C_STS_INVALID_INPUT   Invalid input parameter
  * @retval     I3C_STS_CMDQ_FULL       Command Queue is full
  *
  * @details    The function is used for I3C Master write data to Slave.
  *
  * @note       Device Address Table must be set before using this function.
  * @note       if u16ReadBytes is not
  *
  */
int32_t I3C_Read(I3C_T *i3c, uint8_t u8DevIndex, uint32_t u32Speed, uint32_t *pu32RxBuf, uint16_t u16ReadBytes)
{
    uint32_t i;
    uint32_t u32TimeOutCount = (SystemCoreClock / 1000);
    uint32_t response;

    if ((u16ReadBytes == 0) || (pu32RxBuf == NULL))
    {
        return I3C_STS_INVALID_INPUT;
    }

    /* Check if CmdQ is full */
    if (I3C_IS_CMDQ_FULL(i3c))
    {
        return I3C_STS_CMDQ_FULL;
    }

    i3c->CMDQUE = ((u16ReadBytes << I3C_CMDQUE_DATLEN_Pos) | I3C_CMDATTR_TRANSFER_ARG);

    /* Check if CmdQ is full */
    if (I3C_IS_CMDQ_FULL(i3c))
    {
        return I3C_STS_CMDQ_FULL;
    }

    i3c->CMDQUE = (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_RNW_Msk | I3C_CMDQUE_ROC_Msk
                   | (u32Speed & I3C_CMDQUE_SPEED_Msk)
                   | ((u8DevIndex & 0x1F) << I3C_CMDQUE_DEVINDX_Pos)
                   | (I3C_RX_TID << I3C_CMDQUE_TID_Pos)
                   | I3C_CMDATTR_TRANSFER_CMD);

    while ((i3c->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0)
    {
        if (--u32TimeOutCount == 0)
        {
            return I3C_TIMEOUT_ERR;
        }
    };

    response = i3c->RESPQUE;

    if ((response & I3C_RESPQUE_ERRSTS_Msk) != I3C_RESP_NO_ERR)
    {
        return I3C_STS_INVALID_STATE;
    }

    response = ((response  & I3C_RESPQUE_DATLEN_Msk) + 3) / 4;

    if (response != 0)
    {
        for (i = 0; i < response; i++)
        {
            pu32RxBuf[i] = i3c->TXRXDAT ;
        }
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief      Broadcast RSTDAA command
  *
  * @param[in]  *i3c            Point to I3C peripheral
  *
  * @retval     I3C_STS_NO_ERR              No error
  * @retval     I3C_STS_INVALID_STATE       Invalid state
  * @details    The function is used for I3C Master.
  *
  */

int32_t I3C_BroadcastRSTDAA(I3C_T *i3c)
{
    uint32_t response;
    i3c->CMDQUE = ((0 << I3C_CMDQUE_DATLEN_Pos) | I3C_CMDATTR_TRANSFER_ARG);
    i3c->CMDQUE = (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk
                   | I3C_CMDQUE_CP_Msk
                   | ((I3C_CCC_RSTDAA(TRUE) <<  I3C_CMDQUE_CMD_Pos) & I3C_CMDQUE_CMD_Msk)
                   | I3C_CMDATTR_TRANSFER_CMD);

    while ((i3c->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0);

    response = i3c->RESPQUE;

    if (response & I3C_RESPQUE_ERRSTS_Msk)
    {
        i3c->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
        return I3C_STS_INVALID_STATE;
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief      Broadcast ENTDAA command
  *
  * @param[in]  *i3c            Point to I3C peripheral
  * @param[in]  u8DevCount      Indicating the number of devices to be assigned with the ENTDAA command
  *
  * @retval     positive integer            number of devices assigned, this value should be less or equal to u8DevCount
  * @retval     I3C_STS_INVALID_STATE       Invalid state
  * @details    The function is used for I3C Master.
  *
  */

int32_t I3C_BroadcastENTDAA(I3C_T *i3c, uint8_t u8DevCount)
{
    uint32_t response, error;
    i3c->CMDQUE = (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk
                   | (0 << I3C_CMDQUE_DEVINDX_Pos)
                   | ((u8DevCount << I3C_CMDQUE_DEVCOUNT_Pos) & I3C_CMDQUE_DEVCOUNT_Msk)
                   | (I3C_CCC_ENTDAA <<  I3C_CMDQUE_CMD_Pos)
                   | I3C_CMDATTR_ADDR_ASSGN_CMD);

    while ((i3c->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0);

    response = i3c->RESPQUE;
    error = (response & I3C_RESPQUE_ERRSTS_Msk);

    if (i3c->INTSTS & I3C_INTSTS_TFRERR_Msk)
    {
        i3c->INTSTS |= I3C_INTSTS_TFRERR_Msk;
    }

    if (error == I3C_RESP_NO_ERR)
    {
        return u8DevCount;
    }
    else if (error == I3C_RESP_BROADCAST_ADDR_NACK_ERR)
    {
        i3c->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
        /*
            The controller writes the transfer complete status into the Command Response queue. The Data
            Length Field of Response Data Structure indicates remaining device count in case if the transfer is
            terminated abruptly due to NACK response from the target.
        */
        return (u8DevCount - (response & I3C_RESPQUE_DATLEN_Msk));
    }
    else
    {
        i3c->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
        return I3C_STS_INVALID_STATE;
    }
}

/**
  * @brief      Unicast SETDASA command
  *
  * @param[in]  *i3c            Point to I3C peripheral
  * @param[in]  u8DevIndex      the offset of Device Address Table.
  *                             It could be 0 ~ 6 for DEV1ADR to DEV7ADR.
  *
  * @retval     I3C_STS_NO_ERR              No error
  * @retval     I3C_STS_INVALID_STATE       Invalid state
  * @details    The function is used for I3C Master.
  * @note       Device Address Table must be set before using this function.
  *
  */

int32_t I3C_UnicastSETDASA(I3C_T *i3c, uint8_t u8DevIndex)
{
    uint32_t response;
    i3c->CMDQUE = (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk
                   | (1 << I3C_CMDQUE_DEVCOUNT_Pos)
                   | ((u8DevIndex & 0x1F) << I3C_CMDQUE_DEVINDX_Pos)
                   | (I3C_CCC_SETDASA <<  I3C_CMDQUE_CMD_Pos)
                   | I3C_CMDATTR_ADDR_ASSGN_CMD);

    while ((i3c->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0);

    response = i3c->RESPQUE;

    if (response & I3C_RESPQUE_ERRSTS_Msk)
    {
        i3c->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
        return I3C_STS_INVALID_STATE;
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief      Unicast I3C_CCC_GETACCMST command
  *
  * @param[in]  *i3c            Point to I3C peripheral
  * @param[in]  u8DevIndex      the offset of Device Address Table.
  *                             It could be 0 ~ 6 for DEV1ADR to DEV7ADR.
  * @param[in]  *pu32RxBuf      Pointer to array to read data from Slave
  *
  * @retval     I3C_STS_NO_ERR              No error
  * @retval     I3C_STS_INVALID_STATE       Invalid state
  * @details    The function is used for I3C Master.
  * @note       Device Address Table must be set before using this function.
  *
  */

int32_t I3C_UnicastGETACCMST(I3C_T *i3c, uint8_t u8DevIndex, uint32_t *pu32RxBuf)
{
    uint32_t i, cmd, response, data;
    i3c->CMDQUE = ((1 << I3C_CMDQUE_DATLEN_Pos) | I3C_CMDATTR_TRANSFER_ARG);
    //
    cmd = (I3C_CMDQUE_TOC_Msk | I3C_CMDQUE_ROC_Msk
           | ((u8DevIndex & 0x1F) << I3C_CMDQUE_DEVINDX_Pos)
           | I3C_CMDQUE_RNW_Msk // 0: Write, 1: Read
           | I3C_CMDQUE_CP_Msk | (I3C_CCC_GETACCMST <<  I3C_CMDQUE_CMD_Pos)
           | I3C_CMDATTR_TRANSFER_CMD);
    i3c->CMDQUE = cmd;

    while ((I3C0->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0);

    response = I3C0->RESPQUE;

    if (response & I3C_RESPQUE_ERRSTS_Msk)
    {
        I3C0->DEVCTL |= I3C_DEVCTL_RESUME_Msk;
    }

    response = ((response  & I3C_RESPQUE_DATLEN_Msk) + 3) / 4;

    if (response != 0)
    {
        for (i = 0; i < response; i++)
        {
            data = I3C0->TXRXDAT;

            if ((pu32RxBuf != NULL))
            {
                pu32RxBuf[0] = data;
            }
        }
    }

    return I3C_STS_NO_ERR;
}


/** @} end of group I3C_EXPORTED_FUNCTIONS */
/** @} end of group I3C_Driver */
/** @} end of group Standard_Driver */
