/**************************************************************************//**
 * @file     i3c_DeviceFunc.c
 * @version  V3.00
 * @brief    Functions for I3C Controller and Target Role.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "i3c_DeviceFunc.h"

uint8_t const       g_TgtDA[] = {0x18, 0x28, 0x38, 0x48, 0x58, 0x68, 0x78};
uint8_t const       g_TgtSA[] = {I3C0_TGT_SA, (I3C0_TGT_SA + 1), (I3C0_TGT_SA + 2), (I3C0_TGT_SA + 3), (I3C0_TGT_SA + 4), (I3C0_TGT_SA + 5), (I3C0_TGT_SA + 6)};
#if (NVT_DCACHE_ON == 1)
    NVT_NONCACHEABLE volatile uint32_t   g_I3CDevRx[I3C_DEVICE_RX_BUF_CNT];
    NVT_NONCACHEABLE volatile uint32_t   g_I3CDevTx[I3C_DEVICE_TX_BUF_CNT];
#else
    volatile uint32_t   g_I3CDevRx[I3C_DEVICE_RX_BUF_CNT], g_I3CDevTx[I3C_DEVICE_TX_BUF_CNT];
#endif

volatile uint32_t   g_DA_ASSIGNED_MODE = I3C_SUPPORT_ENTDAA;

I3C_DEVICE_T g_I3CDev =
{
    .port = (I3C_T *)I3C0,

#if (DEVICE_CONTROLLER_ROLE == 1)
    .device_role        = I3C_CONTROLLER,
    .main_controller_da = I3C0_CTR_DA,

    .target_index = 0,
    .target_count = 7,
#else
    .device_role        = I3C_TARGET,
    .main_target_sa     = I3C0_TGT_SA,
    .main_controller_da = I3C0_CTR_DA,

    .target_index = 0,
    .target_count = 3,
#endif

    .speed_mode       = I3C_DEVI3C_SPEED_SDR0,
    .i2c_fm_freq      = (400 * 1000),
    .i2c_fm_plus_freq = (1000 * 1000),
    .i3c_sdr_freq     = DEVICE_SDR_FREQ,

    .cmd_response = I3C_CTRRESP_INITIAL_VALUE,

    .is_HDR_cmd  = FALSE,
    .HDR_cmd     = 0x82,

    .is_HDRBT_cmd = FALSE,
    .HDRBT_cmd    = 0x01020304,

    .is_last_cmd = TRUE,

    .is_DB = FALSE,

    .tx_id  = I3C_TX_TID,
    .tx_len = 0,
    .tx_buf = (uint8_t *)g_I3CDevTx,

    .rx_len = 0,
    .rx_buf = (uint8_t *)g_I3CDevRx,

    .tgtRespQ[0].ErrSts = (uint8_t)I3C_TGTRESP_INITIAL_VALUE,

#if (DEVICE_DMA_ENABLED == 1)
    .DMAPort = PDMA0,
    .is_DMA = TRUE,
    .RxDMACh = 0,
    .TxDMACh = 1,
#endif
};


/**
  * @brief  The I3C0 default IRQ, declared in startup_m3331.S.
  */
void I3C0_IRQHandler(void)
{
    I3C_IRQFunc(&g_I3CDev);
}

/**
  * @brief  Process I3C interrupt events.
  */
void I3C_IRQFunc(I3C_DEVICE_T *dev)
{
    volatile uint32_t int_sts;
    int_sts = dev->port->INTSTS;

    if (int_sts & I3C_INTSTS_RESPQ_READY)
    {
        /* Read-only */
        /* Read RESPQUE to clear I3C_INTSTS_RESPQ_READY */
        DGBINT("[ INT ] RESPQ_READY\n");

        if (dev->device_role == I3C_CONTROLLER)
        {
            dev->cmd_response = dev->port->RESPQUE;
        }
        else
        {
            /* Get Target's response by Controller Write operation */
            I3C_TgtRecv(dev);
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /* For Controller role */
    if (int_sts & I3C_INTSTS_IBI_RECEIVED)
    {
        /* Read IBISTS to clear I3C_INTSTS_IBI_RECEIVED */
        dev->ibi_status = dev->port->IBISTS;
        dev->intsts     |= I3C_INTSTS_IBI_RECEIVED;
        DGBINT("[ INT ] IBI_RECEIVED\n");
        /* Parse IBI queue for MR, In-Band interrupt or Hot-Join event */
        I3C_CtrGetIBI(dev);
    }

    if (int_sts & I3C_INTSTS_TRANSFER_ABORT)
    {
        dev->port->INTSTS = I3C_INTSTS_TRANSFER_ABORT;
        DGBINT("[ INT ] TRANSFER_ABORT\n");
    }

    if (int_sts & I3C_INTSTS_BUSRST_DONE)
    {
        dev->port->INTSTS = I3C_INTSTS_BUSRST_DONE;
        DGBINT("[ INT ] BUSRST_DONE\n");
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /* For Target role */
    if (int_sts & I3C_INTSTS_CCC_UPDATED)
    {
        dev->port->INTSTS  = I3C_INTSTS_CCC_UPDATED;
        dev->intsts       |= I3C_INTSTS_CCC_UPDATED;
        DGBINT("[ INT ] CCC_UPDATED\n");

        if (!(int_sts & I3C_INTSTS_DA_ASSIGNED))
        {
            /* Process CCC updated events */
            I3C_TgtHandleIntSts(dev);
        }
    }

    if (int_sts & I3C_INTSTS_DA_ASSIGNED)
    {
        dev->port->INTSTS = I3C_INTSTS_DA_ASSIGNED;
        dev->intsts      |= I3C_INTSTS_DA_ASSIGNED;
        DGBINT("[ INT ] DA_ASSIGNED\n");
        /* Dump Target's Main and Vitrual address */
        I3C_TgtHandleIntSts(dev);
    }

    if (int_sts & I3C_INTSTS_DEFSLV)
    {
        dev->port->INTSTS = I3C_INTSTS_DEFSLV;
        DGBINT("[ INT ] DEFSLV\n");
    }

    if (int_sts & I3C_INTSTS_READ_REQUEST)
    {
        dev->port->INTSTS = I3C_INTSTS_READ_REQUEST;
        DGBINT("[ INT ] READ_REQUEST\n");
    }

    if (int_sts & I3C_INTSTS_IBI_UPDATED)
    {
        dev->port->INTSTS = I3C_INTSTS_IBI_UPDATED;
        dev->intsts      |= I3C_INTSTS_IBI_UPDATED;
        DGBINT("[ INT ] IBI_UPDATED\n");
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (int_sts & I3C_INTSTS_TX_EMPTY_THLD)
    {
        /* Read-only */
        //DGBINT("[ INT ] TX_EMPTY_THLD\n");
    }

    if (int_sts & I3C_INTSTS_RX_THLD)
    {
        /* Read-only */
        //DGBINT("[ INT ] RX_THLD\n");
    }

    if (int_sts & I3C_INTSTS_CMDQ_EMPTY_THLD)
    {
        /* Read-only */
        //DGBINT("[ INT ] CMDQ_EMPTY_THLD\n");
    }

    if (int_sts & I3C_INTSTS_BUSOWNER_UPDATED)
    {
        dev->port->INTSTS = I3C_INTSTS_BUSOWNER_UPDATED;
        dev->intsts      |= I3C_INTSTS_BUSOWNER_UPDATED;
        DGBINT("[ INT ] BUSOWNER_UPDATED\n");
        /* Reset all FIFO -> apply resume */
        dev->port->RSTCTL = (I3C_RSTCTL_RESPRST_Msk | I3C_RSTCTL_RXRST_Msk | I3C_RSTCTL_IBIQRST_Msk);

        while (dev->port->RSTCTL != 0) {}

        dev->port->DEVCTL |= I3C_DEVCTL_RESUME_Msk;

        while ((dev->port->DEVCTL & I3C_DEVCTL_RESUME_Msk) == I3C_DEVCTL_RESUME_Msk) {}

        if (I3C_IS_CONTROLLER(dev->port))
        {
            DGBINT("\t[ INT ] Device changes to Controller Role\n");
        }
        else
        {
            DGBINT("\t[ INT ] Device changes to Target Role\n");
        }
    }

    if (int_sts & I3C_INTSTS_TRANSFER_ERR)
    {
        dev->port->INTSTS = I3C_INTSTS_TRANSFER_ERR;
        DGBINT("[ INT ] TRANSFER_ERR\n");

        /* Parse error code and recovery to idle state */
        if (dev->device_role == I3C_CONTROLLER)
        {
            I3C_CtrHandleTransErr(dev);
        }
        else
        {
            I3C_TgtHandleTransErr(dev);
        }
    }

    DGBINT("[ INT ] status: 0x%08x -> 0x%08x\n\n", int_sts, dev->port->INTSTS);

    if (int_sts & I3C_INTSTS_RESPQ_READY)
    {
        if (dev->device_role == I3C_CONTROLLER)
        {
            if ((dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk) == I3C_CTRRESP_NO_ERR)
            {
                dev->tx_id  = ((dev->cmd_response & I3C_CTRRESP_TID_Msk) >> I3C_CTRRESP_TID_Pos);
                dev->rx_len = ((dev->cmd_response & I3C_CTRRESP_DATLEN_Msk) >> I3C_CTRRESP_DATLEN_Pos);
                DGBINT("[ INT ] #CTR Resp:\n\tTID %d, Len %d\n\n", dev->tx_id, dev->rx_len);
            }
        }
    }
}

/**
  * @brief  Initialize I3C device role according to DEVICE_CONTROLLER_ROLE in i3c_cfg.h.
  */
void I3C_Init(I3C_DEVICE_T *dev)
{
    uint32_t i3c_engclk;
    /* Enable I3C module clock */
    CLK_EnableModuleClock(I3C0_MODULE);
    /* Select I3C module clock source */
    CLK_SetModuleClock(I3C0_MODULE, CLK_I3CSEL_I3C0SEL_HCLK0, 0);
    i3c_engclk = SystemCoreClock;
    SYS_ResetModule(SYS_I3C0RST);

    /* Enable DMA before setting the I3C ENABLE bit */
    if (dev->is_DMA)
    {
        /* Enable PDMA0 module clock */
        CLK_EnableModuleClock(PDMA0_MODULE);
        /* Enable PDMA channel for I3C Rx function */
        I3C_ConfigRxDMA(dev, (uint32_t)(&dev->port->TXRXDAT), (uint32_t)(dev->rx_buf), (I3C_DEVICE_RX_BUF_CNT * 4));
        /* Enable I3C DMA function */
        I3C_EnableDMA(dev->port);
    }

    dev->engclk = i3c_engclk;
    dev->irq_enable = TRUE;
    //
    dev->target_daa_mode = DA_ASSIGNED_MODE;

    if (I3C_DeviceInit(dev) != I3C_STS_NO_ERR)
    {
        DGBINT("\n# ERROR - I3C_DeviceInit\n\n");

        while (1) {}
    }

#if (DEVICE_CONTROLLER_ROLE == 1)
    /* Configure MID and PID for Main Controller */
    dev->port->SLVMID = I3C0_CTR_MID;
    dev->port->SLVPID = I3C0_CTR_PID;
#else
    /* Configure MID and PID for Main Target */
    dev->port->SLVMID = I3C0_TGT_MID;
    dev->port->SLVPID = I3C0_TGT_PID;
#endif
    dev->intsts = dev->port->INTSTS;

    if (dev->irq_enable)
    {
        NVIC_EnableIRQ(I3C0_IRQn);
    }
    else
    {
        NVIC_DisableIRQ(I3C0_IRQn);
    }

    /* Enable device */
    dev->port->DEVCTL |= I3C_DEVCTL_ENABLE_Msk;
#if (DEVICE_CONTROLLER_ROLE == 1)

    if (dev->device_role == I3C_CONTROLLER)
    {
        uint32_t i;

        /* Allocate default "Device Address Table for Targets" at I2C mode */
        for (i = 0; i < 7; i++)
        {
            dev->target_sa[i] = g_TgtSA[i];
            I3C_SetDeviceAddr(dev->port, i, I3C_DEVTYPE_I2C, g_TgtDA[i], dev->target_sa[i]);
        }
    }

#endif
}

/**
  * @brief  Send Dynamic Address Assignment CCC, used for Controller.
  */
uint32_t I3C_FuncDAAssign(I3C_DEVICE_T *dev, uint32_t ccc, uint8_t tgt, uint8_t da)
{
    volatile uint32_t i;
    static uint8_t sTotalAddrCnt = 0xFF;
    (void)tgt;
    (void)da;
    DGBINT("\n");

    switch (ccc)
    {
        case I3C_CCC_RSTDAA:
            /* RSTDAA CCC */
            dev->tx_len       = 0;
            dev->is_last_cmd  = TRUE;
            dev->ccc_code     = I3C_CCC_RSTDAA;
            dev->tx_len       = 0;
            I3C_CtrCCCSet(dev);

            // Waiting for IRQ
            while (dev->cmd_response == I3C_CTRRESP_INITIAL_VALUE) {}

            if ((dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk) == I3C_CTRRESP_NO_ERR)
            {
                /* Reset valid target count to 0 */
                dev->target_count = 0;
                sTotalAddrCnt = 0xFF;

                for (i = 0; i < 7; i++)
                {
                    dev->target_da[i] = 0x0;
                }

                for (i = 0; i < 7; i++)
                {
                    I3C_SetDeviceAddr(dev->port, i, I3C_DEVTYPE_I2C, dev->target_da[i], dev->target_sa[i]);
                }
            }
            else
            {
                dev->target_count = (sTotalAddrCnt == 0xFF) ? 0 : sTotalAddrCnt;
            }

            printf("Current Target info after RSTDAA CCC. (DA count: %d)\n", dev->target_count);

            /* Dump all Targets info. */
            for (i = 0; i < 7; i++)
            {
                if (dev->port->DEVADR[i] & I3C_DEVADR_DEVICE_Msk)
                {
                    // Target at I2C mode
                    dev->target_sa[i] = ((dev->port->DEVADR[i] & I3C_DEVADR_STADR_Msk) >> I3C_DEVADR_STADR_Pos);

                    if (dev->target_sa[i] == 0x0)
                    {
                        continue;
                    }

                    printf("\tTarget #%d:\n", i);
                    printf("\t - SADDR          = 0x%02x \n", dev->target_sa[i]);
                }
                else
                {
                    // Target at I3C mode
                    dev->target_da[i] = ((dev->port->TGTCHAR[i].DADDR & I3C_TGTCHAR4_DADDR_Msk) >> I3C_TGTCHAR4_DADDR_Pos);
                    printf("\tTarget #%d:\n", i);
                    printf("\t - Provisional ID = 0x%08x%02x \n", dev->port->TGTCHAR[i].PIDMSB, dev->port->TGTCHAR[i].PIDLSB);
                    printf("\t - BCR, DCR       = 0x%08x \n", dev->port->TGTCHAR[i].BCRDCR);
                    printf("\t - DADDR          = 0x%02x \n", dev->target_da[i]);
                }
            }

            printf("\n");
            break;

        case I3C_CCC_ENTDAA:
            for (i = 0; i < 7; i++)
            {
                I3C_SetDeviceAddr(dev->port, i, I3C_DEVTYPE_I3C, g_TgtDA[i], dev->target_sa[i]);
            }

            if (dev->target_count >= sTotalAddrCnt)
            {
                sTotalAddrCnt = dev->target_count;
            }

            /* ENTDAA CCC */
            dev->target_index = 0;
            dev->target_count = 7;
            dev->ccc_code     = I3C_CCC_ENTDAA;
            I3C_CtrDAA(dev);

            // Waiting for IRQ
            while (dev->cmd_response == I3C_CTRRESP_INITIAL_VALUE) {}

            if ((dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk) == I3C_CTRRESP_NO_ERR)
            {
                sTotalAddrCnt = dev->target_count;
            }
            else
            {
                //printf("\n[ ENTDAA, error code %d ]\n", ((uint32_t)(dev->cmd_response&I3C_CTRRESP_ERRSTS_Msk) >> I3C_CTRRESP_ERRSTS_Pos));
                if (dev->target_count != 0)
                {
                    sTotalAddrCnt = dev->target_count;
                }
                else
                {
                    dev->target_count = (sTotalAddrCnt == 0xFF) ? 0 : sTotalAddrCnt;
                }
            }

            printf("Total I3C Target x%d after ENTDAA CCC.\n", dev->target_count);

            /* Dump all Targets info. */
            if (dev->target_count == 0)
            {
                for (i = 0; i < 7; i++)
                {
                    dev->target_da[i] = 0x0;
                    I3C_SetDeviceAddr(dev->port, i, I3C_DEVTYPE_I2C, dev->target_da[i], dev->target_sa[i]);
                }

                printf("\tNo valid I3C Target on the bus\n");
            }
            else
            {
                for (i = 0; i < 7; i++)
                {
                    if (dev->target_count >  i)
                    {
                        // Target at I3C mode
                        dev->target_da[i] = ((dev->port->TGTCHAR[i].DADDR & I3C_TGTCHAR4_DADDR_Msk) >> I3C_TGTCHAR4_DADDR_Pos);
                        printf("\tTarget #%d:\n", i);
                        printf("\t - Provisional ID = 0x%08x%02x \n", dev->port->TGTCHAR[i].PIDMSB, dev->port->TGTCHAR[i].PIDLSB);
                        printf("\t - BCR, DCR       = 0x%08x \n", dev->port->TGTCHAR[i].BCRDCR);
                        printf("\t - DADDR          = 0x%02x \n", dev->target_da[i]);
                    }
                    else
                    {
                        // Target at I2C mode
                        dev->target_da[i] = 0x0;
                        I3C_SetDeviceAddr(dev->port, i, I3C_DEVTYPE_I2C, dev->target_da[i], dev->target_sa[i]);
                        dev->target_sa[i] = ((dev->port->DEVADR[i] & I3C_DEVADR_STADR_Msk) >> I3C_DEVADR_STADR_Pos);

                        if (dev->target_sa[i] == 0x0)
                        {
                            continue;
                        }

                        printf("\tTarget #%d:\n", i);
                        printf("\t - SADDR          = 0x%02x \n", dev->target_sa[i]);
                    }
                }
            }

            printf("\n");
            break;

        default:
            printf("[ Invalid CCC code - 0x%x in I3C_FuncDAAssign ]\n", ccc);
            break;
    }

    return sTotalAddrCnt;
}

/**
  * @brief  Process IBI response, used for Controller.
  */
int32_t I3C_FuncIBIReceived(I3C_DEVICE_T *dev)
{
    int32_t iRet = -1;
    volatile uint32_t i;
    uint8_t *pu8Buf;

    if (dev->ibi_status & (uint32_t)(I3C_IBIQSTS_NACK))
    {
        printf("NACK In-Band Interrupt.\n\n");
    }
    else
    {
        switch (dev->ibi_type)
        {
            case I3C_IBI_TYPE_TIR:
                iRet = I3C_IBI_TYPE_TIR;
                pu8Buf = (uint8_t *)dev->ibi_buf;
                printf("#IBI: Process In-Band Interrupt from Target-0x%02x and get MDB 0x%02x, %d-bytes payload.\n\t",
                       dev->ibi_id, dev->ibi_MDB, dev->ibi_len);

                for (i = 0; i < dev->ibi_len; i++)
                {
                    printf("0x%02x ", pu8Buf[i]);
                }

                printf("\n");
                break;

            case I3C_IBI_TYPE_CR:
                iRet = I3C_IBI_TYPE_CR;
                printf("#IBI: Process Controller Request from Target-0x%02x, and change to %s role.\n",
                       dev->ibi_id, I3C_IS_CONTROLLER(dev->port) ? "Controller" : "Target");
                break;

            case I3C_IBI_TYPE_HJ:
                iRet = I3C_IBI_TYPE_HJ;
                printf("#IBI: Process Hot-Join request ... %s.\n", (dev->ibi_id == 0x2) ? "PASS" : "FAIL");
                printf("Total I3C Target x%d after Hot-Join reguest:\n", dev->target_count);
                I3C_FuncDAAssign(dev, I3C_CCC_ENTDAA, 7, 0);
                break;

            default:
                break;
        }

        printf("\n");
    }

    /* Clear ibi status */
    dev->ibi_type = 0;
    dev->ibi_status = 0;
    return iRet;
}

/**
  * @brief  Send In-Band Interrupt Request to Controller, used for Target Role.
  */
int32_t  I3C_FuncIBIRequest(I3C_DEVICE_T *dev, uint32_t mdb, uint32_t payload, uint32_t len)
{
    int32_t iRet = -1;

    if (!I3C_IS_DA_VALID(dev->port))
    {
        printf("\nTarget device isn't in I3C mode. Please enter to I3C mode first.\n\n");
        return iRet;
    }

    if (I3C_IS_CONTROLLER(dev->port))
    {
        printf("\nCurrent device is in I3C Controller Role.\n\n");
        return iRet;
    }

    printf("Target issue IBI request to Controller ...\n");
    dev->target_index = 0; // Use main Target
    dev->ibi_type     = I3C_IBI_TYPE_TIR;
    dev->ibi_MDB      = mdb;
    dev->ibi_len      = len;
    dev->ibi_payload  = payload;

    if (I3C_TgtIssueIBI(dev) != I3C_STS_NO_ERR)
    {
        printf("\tError in IBI request\n");
        return iRet;
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief  Send Controller Role Request to Controller, used for Target Role.
  */
int32_t I3C_FuncCRRequest(I3C_DEVICE_T *dev)
{
    int32_t iRet = -1;

    if (!I3C_IS_DA_VALID(dev->port))
    {
        printf("\nTarget device isn't in I3C mode. Please enter to I3C mode first.\n\n");
        return iRet;
    }

    if (I3C_IS_CONTROLLER(dev->port))
    {
        printf("\nCurrent device is in I3C Controller Role.\n\n");
        return iRet;
    }

    printf("Target issue Controller request to become Controller ...\n");
    dev->target_index = 0; // Use main Target
    dev->ibi_type     = I3C_IBI_TYPE_CR;

    if (I3C_TgtIssueIBI(dev) != I3C_STS_NO_ERR)
    {
        printf("\tError in Controller Role request\n");
        return iRet;
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief  Process DEFTGTS CCC response, used for Target Role.
  */
int32_t I3C_FuncDEFTGTSResp(I3C_DEVICE_T *dev)
{
    volatile uint32_t i;
    uint32_t u32TGTCnts, u32TGTTblAddr;
    u32TGTCnts = dev->tgtRespQ[0].RxBufLen; /* Indicates Valid Target Counts send by Controller */
    u32TGTTblAddr = (uint32_t)&dev->port->TGTCHAR[0];
    printf("*** [ Resp for DEFTGTS CCC Write from Controller DA 0x%02x ] ***\n", (((inpw(u32TGTTblAddr) >> 24) & 0xFF) >> 1));
    dev->target_count = u32TGTCnts;

    if (dev->target_count > 0)
    {
        printf("\tCTR-0, DA 0x%02x, DCR 0x%02x, BCR 0x%02x, SA 0x%02x\n",
               (((inpw(u32TGTTblAddr) >> 24) & 0xFF) >> 1), ((inpw(u32TGTTblAddr) >> 16) & 0xFF), ((inpw(u32TGTTblAddr) >> 8) & 0xFF), (((inpw(u32TGTTblAddr) >> 0) & 0xFF) >> 1));
        dev->target_da[0] = (((inpw(u32TGTTblAddr) >> 24) & 0xFF) >> 1);
        dev->main_controller_da = dev->target_da[0];

        for (i = 0; i < (dev->target_count - 1); i++)
        {
            u32TGTTblAddr += 4;
            printf("\tTGT-%d, DA 0x%02x, DCR 0x%02x, BCR 0x%02x, SA 0x%02x\n",
                   i, (((inpw(u32TGTTblAddr) >> 24) & 0xFF) >> 1), ((inpw(u32TGTTblAddr) >> 16) & 0xFF), ((inpw(u32TGTTblAddr) >> 8) & 0xFF), (((inpw(u32TGTTblAddr) >> 0) & 0xFF) >> 1));
            dev->target_da[i + 1] = (((inpw(u32TGTTblAddr) >> 24) & 0xFF) >> 1);
        }

        printf("\n");
    }
    else
    {
        printf("\n\tNo vaild Target on the I3C Bus\n\n");
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief  Send SDR write to Target, used for Controller Role.
  */
int32_t I3C_FuncSDRWrite(I3C_DEVICE_T *dev, uint8_t tgt, uint8_t *buf, uint16_t len)
{
    int32_t resp_sts;
    memcpy(dev->tx_buf, buf, len);
    dev->target_index = tgt;
    dev->tx_len       = len;
    dev->is_last_cmd  = TRUE;
    DGBINT("[ Non-IRQ ] INTSTS = 0x08%X\n", dev->port->INTSTS);

    if (dev->speed_mode != I3C_DEVI3C_SPEED_SDR0)
    {
        // select user's dev->speed_mode beform calling I3C_FuncSDRWrite(...)
    }

    I3C_CtrWrite(dev);

    if (dev->irq_enable)
    {
        // Process in IRQ
        while (dev->cmd_response == I3C_CTRRESP_INITIAL_VALUE) {}

        resp_sts = (dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk);

        if (resp_sts != I3C_CTRRESP_NO_ERR)
        {
            resp_sts = (resp_sts >> I3C_CTRRESP_ERRSTS_Pos);
            printf("\n[ Write error occurred, error code %d ]\n", (uint32_t)resp_sts);
            return resp_sts;
        }
    }
    else
    {
        while ((dev->port->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0) {}

        printf("\n[ TODO: Users need to manually parse the RespQ status 0x%08x ]\n", dev->port->RESPQUE);
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief  Send SDR read to Target, used for Controller Role.
  */
int32_t I3C_FuncSDRRead(I3C_DEVICE_T *dev, uint8_t tgt, uint8_t *buf, uint16_t *len)
{
    int32_t resp_sts;
    volatile uint32_t i;
    uint32_t *p32Buf;
    dev->target_index = tgt;
    dev->rx_len       = *len;
    dev->is_last_cmd  = TRUE;
    DGBINT("[ Non-IRQ ] INTSTS = 0x08%X\n", dev->port->INTSTS);

    if (dev->speed_mode != I3C_DEVI3C_SPEED_SDR0)
    {
        // select user's dev->speed_mode beform calling I3C_FuncSDRRead(...)
    }

    I3C_CtrRead(dev);

    if (dev->irq_enable)
    {
        // Process in IRQ
        while (dev->cmd_response == I3C_CTRRESP_INITIAL_VALUE) {}

        resp_sts = (dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk);

        if (dev->is_DMA)
        {
            /* Enable PDMA channel for I3C Rx function */
            I3C_ConfigRxDMA(dev, (uint32_t)(&dev->port->TXRXDAT), (uint32_t)(dev->rx_buf), (I3C_DEVICE_RX_BUF_CNT * 4));
        }

        if (resp_sts != I3C_CTRRESP_NO_ERR)
        {
            resp_sts = (resp_sts >> I3C_CTRRESP_ERRSTS_Pos);
            printf("\n[ Read error occurred, error code %d ]\n", (uint32_t)resp_sts);
            return resp_sts;
        }
        else
        {
            p32Buf = (uint32_t *)&dev->rx_buf[0];

            if (dev->is_DMA)
            {
                /* Get data by Rx PDAM */
            }
            else
            {
                for (i = 0; i < ((dev->rx_len + 3) / 4); i++)
                {
                    p32Buf[i] = dev->port->TXRXDAT;
                }
            }

            memcpy(buf, dev->rx_buf, dev->rx_len);
            *len = dev->rx_len;
        }
    }
    else
    {
        while ((dev->port->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0) {}

        printf("\n[ TODO: Users need to manually parse the RespQ status 0x%08x ]\n", dev->port->RESPQUE);
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief  Send HDR-DDR write to Target, used for Controller Role.
  */
int32_t I3C_FuncDDRWrite(I3C_DEVICE_T *dev, uint8_t tgt, uint8_t *buf, uint16_t len)
{
    int32_t resp_sts;
    memcpy(dev->tx_buf, buf, len);
    dev->target_index = tgt;
    dev->tx_len       = len;
    dev->is_last_cmd  = TRUE;
    dev->speed_mode   = I3C_DEVI3C_SPEED_HDRDDR;
    dev->is_HDR_cmd   = TRUE;
    dev->ccc_code     = I3C_CCC_HDRDDR;
    I3C_CtrWrite(dev);

    if (dev->irq_enable)
    {
        // Process in IRQ
        while (dev->cmd_response == I3C_CTRRESP_INITIAL_VALUE) {}

        resp_sts = (dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk);

        if (resp_sts != I3C_CTRRESP_NO_ERR)
        {
            resp_sts = (resp_sts >> I3C_CTRRESP_ERRSTS_Pos);
            printf("\n[ Write error occurred, error code %d ]\n", (uint32_t)resp_sts);
            return resp_sts;
        }
    }
    else
    {
        while ((dev->port->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0) {}

        printf("\n[ TODO: Users need to manually parse the RespQ status 0x%08x ]\n", dev->port->RESPQUE);
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief  Send HDR-DDR read to Target, used for Controller Role.
  */
int32_t I3C_FuncDDRRead(I3C_DEVICE_T *dev, uint8_t tgt, uint8_t *buf, uint16_t *len)
{
    int32_t resp_sts;
    volatile uint32_t i;
    uint32_t *p32Buf;
    dev->target_index = tgt;
    dev->rx_len       = *len;
    dev->is_last_cmd  = TRUE;
    dev->speed_mode   = I3C_DEVI3C_SPEED_HDRDDR;
    dev->is_HDR_cmd   = TRUE;
    dev->ccc_code     = I3C_CCC_HDRDDR;
    I3C_CtrRead(dev);

    if (dev->irq_enable)
    {
        // Process in IRQ
        while (dev->cmd_response == I3C_CTRRESP_INITIAL_VALUE) {}

        resp_sts = (dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk);

        if (dev->is_DMA)
        {
            /* Enable PDMA channel for I3C Rx function */
            I3C_ConfigRxDMA(dev, (uint32_t)(&dev->port->TXRXDAT), (uint32_t)(dev->rx_buf), (I3C_DEVICE_RX_BUF_CNT * 4));
        }

        if (resp_sts != I3C_CTRRESP_NO_ERR)
        {
            resp_sts = (resp_sts >> I3C_CTRRESP_ERRSTS_Pos);
            printf("\n[ Read error occurred, error code %d ]\n", (uint32_t)resp_sts);
            return resp_sts;
        }
        else
        {
            p32Buf = (uint32_t *)&dev->rx_buf[0];

            if (dev->is_DMA)
            {
                /* Get data by Rx PDAM */
            }
            else
            {
                for (i = 0; i < ((dev->rx_len + 3) / 4); i++)
                {
                    p32Buf[i] = dev->port->TXRXDAT;
                }
            }

            memcpy(buf, dev->rx_buf, dev->rx_len);
            *len = dev->rx_len;
        }
    }
    else
    {
        while ((dev->port->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0) {}

        printf("\n[ TODO: Users need to manually parse the RespQ status 0x%08x ]\n", dev->port->RESPQUE);
    }

    return I3C_STS_NO_ERR;
}

/**
  * @brief  Send GET CCC and get response, used for Controller Role.
  */
int32_t I3C_FuncGETCCCResp(I3C_DEVICE_T *dev, uint32_t ccc, uint8_t tgt)
{
    int32_t resp_sts;
    volatile uint32_t i;
    uint32_t *p32Buf;

    if (dev->port->DEVADR[tgt] & I3C_DEVADR_DEVICE_Msk)
    {
        printf("\nTarget index-%d NOT at I3C mode.\n", tgt);
        return -1;
    }

    printf("\n");

    switch (ccc)
    {
        case I3C_CCC_GETPID:
            printf("[ GETPID ] (Tgt: 0x%02x)\n\t", dev->target_da[tgt]);
            dev->rx_len       = 6;
            break;

        case I3C_CCC_GETBCR:
            printf("[ GETBCR ] (Tgt: 0x%02x)\n\t", dev->target_da[tgt]);
            dev->rx_len       = 1;
            break;

        case I3C_CCC_GETDCR:
            printf("[ GETDCR ] (Tgt: 0x%02x)\n\t", dev->target_da[tgt]);
            dev->rx_len       = 1;
            break;

        case I3C_CCC_GETSTATUS:
            printf("[ GETSTATUS ] (Tgt: 0x%02x)\n\t", dev->target_da[tgt]);
            dev->rx_len       = 2;
            dev->is_DB        = TRUE;
            dev->DB           = 0x00;
            break;

        case I3C_CCC_GETCAPS:
            printf("[ GETCAPS ] (Tgt: 0x%02x)\n\t", dev->target_da[tgt]);
            dev->rx_len       = 4;
            dev->is_DB        = TRUE;
            dev->DB           = 0x00;
            break;

        case I3C_CCC_GETMWL:
            printf("[ GETMWL ] (Tgt: 0x%02x)\n\t", dev->target_da[tgt]);
            dev->rx_len       = 2;
            break;

        case I3C_CCC_GETMRL:
            printf("[ GETMRL ] (Tgt: 0x%02x)\n\t", dev->target_da[tgt]);
            dev->rx_len       = 2;
            break;

        case I3C_CCC_GETMXDS:
            printf("[ GETMXDS ] (Tgt: 0x%02x)\n\t", dev->target_da[tgt]);
            dev->rx_len       = 4;
            break;

        default:
            printf("[ Unsupported CCC code - 0x%x in I3C_FuncGETCCCResp ]\n", ccc);
            ccc = (uint32_t)I3C_STS_INVALID_INPUT;
            break;
    }

    if (ccc == (uint32_t)I3C_STS_INVALID_INPUT)
    {
        return I3C_STS_INVALID_INPUT;
    }

    dev->target_index = tgt;
    dev->is_last_cmd  = TRUE;
    dev->ccc_code     = ccc;
    I3C_CtrCCCGet(dev);

    // Process in IRQ
    while (dev->cmd_response == I3C_CTRRESP_INITIAL_VALUE) {}

    resp_sts = (dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk);

    if (dev->is_DMA)
    {
        /* Enable PDMA channel for I3C Rx function */
        I3C_ConfigRxDMA(dev, (uint32_t)(&dev->port->TXRXDAT), (uint32_t)(dev->rx_buf), (I3C_DEVICE_RX_BUF_CNT * 4));
    }

    if (resp_sts != I3C_CTRRESP_NO_ERR)
    {
        resp_sts = (resp_sts >> I3C_CTRRESP_ERRSTS_Pos);
        printf("\n[ Read error occurred, error code %d ]\n", (uint32_t)resp_sts);
        return resp_sts;
    }
    else
    {
        p32Buf = (uint32_t *)&dev->rx_buf[0];

        if (dev->is_DMA)
        {
            /* Get data by Rx PDAM */
        }
        else
        {
            for (i = 0; i < ((dev->rx_len + 3) / 4); i++)
            {
                p32Buf[i] = dev->port->TXRXDAT;
            }
        }

        for (i = 0; i < dev->rx_len; i++)
        {
            printf("%02x ", dev->rx_buf[i]);
        }

        printf("\n");
    }

    printf("\n");
    return I3C_STS_NO_ERR;
}

int32_t I3C_FuncSETCCC(I3C_DEVICE_T *dev, uint32_t ccc, uint8_t tgt)
{
    int32_t resp_sts;

    if (ccc & I3C_CCC_DIRECT)
    {
        if (dev->port->DEVADR[tgt] & I3C_DEVADR_DEVICE_Msk)
        {
            printf("\nTarget index-%d NOT at I3C mode.\n", tgt);
            return -1;
        }
    }

    printf("\n");

    switch (ccc)
    {
        case I3C_CCC_SETMWL(1):
            tgt = 0;

        case I3C_CCC_SETMWL(0):
            printf("[ SETMWL ] (Tgt: 0x%02x)\n\t", dev->target_da[tgt]);
            dev->tx_len       = 2;
            dev->tx_buf[0]    = 0x01;
            dev->tx_buf[1]    = 0x10;
            break;

        case I3C_CCC_SETMRL(1):
            tgt = 0;

        case I3C_CCC_SETMRL(0):
            printf("[ SETMRL ] (Tgt: 0x%02x)\n\t", dev->target_da[tgt]);
            dev->tx_len       = 3;
            dev->tx_buf[0]    = 0x01;
            dev->tx_buf[1]    = 0x10;
            dev->tx_buf[2]    = 4;
            break;

        case I3C_CCC_ENEC(1):
            tgt = 0;

        case I3C_CCC_ENEC(0):
            printf("[ ENEC ] (Tgt: 0x%02x)\n\t", dev->target_da[tgt]);
            dev->tx_len       = 1;
            dev->tx_buf[0]    = 0x0F;
            break;

        case I3C_CCC_DISEC(1):
            tgt = 0;

        case I3C_CCC_DISEC(0):
            printf("[ DISEC ] (Tgt: 0x%02x)\n\t", dev->target_da[tgt]);
            dev->tx_len       = 1;
            dev->tx_buf[0]    = 0x0F;
            break;

        default:
            printf("[ Unsupported CCC code - 0x%x in I3C_FuncSETCCC ]\n", ccc);
            ccc = (uint32_t)I3C_STS_INVALID_INPUT;
            break;
    }

    if (ccc == (uint32_t)I3C_STS_INVALID_INPUT)
    {
        return I3C_STS_INVALID_INPUT;
    }

    dev->target_index = tgt;
    dev->is_last_cmd  = TRUE;
    dev->ccc_code     = ccc;
    I3C_CtrCCCSet(dev);

    if (dev->irq_enable)
    {
        // Process in IRQ
        while (dev->cmd_response == I3C_CTRRESP_INITIAL_VALUE) {}

        resp_sts = (dev->cmd_response & I3C_CTRRESP_ERRSTS_Msk);

        if (resp_sts != I3C_CTRRESP_NO_ERR)
        {
            resp_sts = (resp_sts >> I3C_CTRRESP_ERRSTS_Pos);
            printf("\n[ Write error occurred, error code %d ]\n", (uint32_t)resp_sts);
            return resp_sts;
        }

        printf("~ Set CCC NO Error.\n");
    }
    else
    {
        while ((dev->port->INTSTS & I3C_INTSTS_RESPRDY_Msk) == 0) {}

        printf("\n[ TODO: Users need to manually parse the RespQ status 0x%08x ]\n", dev->port->RESPQUE);
    }

    printf("\n");
    return I3C_STS_NO_ERR;
}
