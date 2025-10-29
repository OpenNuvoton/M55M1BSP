/**************************************************************************//**
 * @file     emac.c
 * @version  V1.00
 * @brief    EMAC driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "string.h"
#include "m55m1_emac.h"
#include "m55m1_mii.h"


#if (LWIP_USING_HW_CHECKSUM == 1)
    #define USING_HW_CHECKSUM
#endif

NVT_NONCACHEABLE static synopGMACdevice GMACdev = {0};
#if (NVT_DCACHE_ON == 1)
/* Descriptors and data buffers are non-cacheable to avoid cache issues. */
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


static void EMAC_ModuleInit(void)
{
    /* Enable module clock */
    CLK_EnableModuleClock(GPIOE_MODULE);
    CLK_EnableModuleClock(EMAC0_MODULE);

    SYS_ResetModule(SYS_EMAC0RST);

    SET_EMAC0_RMII_MDC_PE8();
    SET_EMAC0_RMII_MDIO_PE9();
    SET_EMAC0_RMII_TXD0_PE10();
    SET_EMAC0_RMII_TXD1_PE11();
    SET_EMAC0_RMII_TXEN_PE12();
    SET_EMAC0_RMII_REFCLK_PC8();
    SET_EMAC0_RMII_RXD0_PC7();
    SET_EMAC0_RMII_RXD1_PC6();
    SET_EMAC0_RMII_CRSDV_PA7();
    SET_EMAC0_RMII_RXERR_PA6();

    GPIO_SetSlewCtl(PE, (BIT10 | BIT11 | BIT12), GPIO_SLEWCTL_FAST0);

    /* PE.13 Set high */
    GPIO_SetMode(PE, BIT13, GPIO_MODE_OUTPUT);
    PE13 = 1;

}

void EMAC_Open(uint8_t *macaddr)
{
    uint32_t i;

    EMAC_ModuleInit();

    /* Attach the device to MAC struct This will configure all the required base addresses
      such as Mac base, configuration base, phy base address(out of 32 possible phys) */
    synopGMAC_attach(&GMACdev, (EMAC0_BASE + MACBASE), (EMAC0_BASE + DMABASE), DEFAULT_PHY_BASE, macaddr);

    synopGMAC_reset(&GMACdev);

    memcpy((void *)&GMACdev.mac_addr[0], (void *)&macaddr[0], 6);

    /* Lets read the version of ip in to device structure */
    synopGMAC_read_version(&GMACdev);

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

    if (mii_check_phy_init(&GMACdev) < 0)
    {
        printf("emac:: Init PHY FAIL.\n");
        printf("\n??? Check the PHY device...\n");

        while (1) {}
    }

    /* Set up the tx and rx descriptor queue/ring */
    synopGMAC_setup_tx_desc_queue(&GMACdev, &tx_desc[0], TRANSMIT_DESC_SIZE, RINGMODE);
    synopGMAC_init_tx_desc_base(&GMACdev);  // Program the transmit descriptor base address in to DmaTxBase addr
    TR("DmaTxBaseAddr = %08x\n", synopGMACReadReg(GMACdev.DmaBase, DmaTxBaseAddr));

    synopGMAC_setup_rx_desc_queue(&GMACdev, &rx_desc[0], RECEIVE_DESC_SIZE, RINGMODE);
    synopGMAC_init_rx_desc_base(&GMACdev); // Program the transmit descriptor base address in to DmaTxBase addr
    TR("DmaTxBaseAddr = %08x\n", synopGMACReadReg(GMACdev.DmaBase, DmaRxBaseAddr));

    synopGMAC_dma_bus_mode_init(&GMACdev, (DmaBurstLength32 | DmaDescriptorSkip0 | DmaDescriptor8Words));
    synopGMAC_dma_control_init(&GMACdev, (DmaStoreAndForward | DmaTxSecondFrame | DmaRxThreshCtrl128));

    /* Initialize the mac interface */
    synopGMAC_mac_init(&GMACdev);
    synopGMAC_promisc_enable(&GMACdev);

    synopGMAC_pause_control(&GMACdev); // This enables the pause control in Full duplex mode of operation

#if defined(USING_HW_CHECKSUM)
    /* IPC Checksum offloading is enabled for this driver. Should only be used if Full Ip checksumm offload engine is configured in the hardware */
    synopGMAC_enable_rx_chksum_offload(&GMACdev);    // Enable the offload engine in the receive path
    synopGMAC_rx_tcpip_chksum_drop_enable(&GMACdev); // This is default configuration, DMA drops the packets if error in encapsulated ethernet payload
#endif

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

    printf("Link speed......%d\n", GMACdev.Speed);

    synopGMAC_set_mode(&GMACdev, GMACdev.Speed);

    NVIC_EnableIRQ(EMAC0_IRQn);
}

/*----------------------------------------------------------------------------
  EMAC IRQ Handler
 *----------------------------------------------------------------------------*/
NVT_ITCM void EMAC0_IRQHandler(void)
{
    u32 interrupt, dma_status_reg;
    s32 status;
    u32 u32GmacIntSts;
    u32 u32GmacDmaIE = DmaIntEnable;

    // Check GMAC interrupt
    u32GmacIntSts = synopGMACReadReg(GMACdev.MacBase, GmacInterruptStatus);

    if (u32GmacIntSts & GmacTSIntSts)
    {
        GMACdev.synopGMACNetStats.ts_int = 1;
        status = synopGMACReadReg(GMACdev.MacBase, GmacTSStatus);

        if (!(status & (1 << 1)))
        {
            TR("TS alarm flag not set??\n");
        }
        else
        {
            TR("TS alarm!!!!!!!!!!!!!!!!\n");
        }
    }

    synopGMACWriteReg(GMACdev.MacBase, GmacInterruptStatus, u32GmacIntSts);

    dma_status_reg = synopGMACReadReg(GMACdev.DmaBase, DmaStatus);

    if (dma_status_reg == 0)
    {
        TR("dma_status ==0 \n");
        return;
    }

    if (dma_status_reg & GmacPmtIntr)
    {
        TR("%s:: Interrupt due to PMT module\n", __FUNCTION__);
        synopGMAC_powerup_mac(&GMACdev);
    }

    if (dma_status_reg & GmacLineIntfIntr)
    {
        TR("%s:: Interrupt due to GMAC LINE module\n", __FUNCTION__);
    }

    interrupt = synopGMAC_get_interrupt_type(&GMACdev);
    TR("%s:Interrupts to be handled: 0x%08x  %08x\n", __func__, interrupt, synopGMAC_get_ie(&GMACdev));

    if (interrupt & synopGMACDmaError)
    {
        TR("%s::Fatal Bus Error Inetrrupt Seen\n", __FUNCTION__);
        synopGMAC_disable_dma_tx(&GMACdev);
        synopGMAC_disable_dma_rx(&GMACdev);

        synopGMAC_take_desc_ownership_tx(&GMACdev);
        synopGMAC_take_desc_ownership_rx(&GMACdev);

        synopGMAC_init_tx_rx_desc_queue(&GMACdev);

        synopGMAC_reset(&GMACdev);

        synopGMAC_set_mac_addr(&GMACdev, GmacAddr0High, GmacAddr0Low, &GMACdev.mac_addr[0]);
        synopGMAC_dma_bus_mode_init(&GMACdev, DmaBurstLength32 | DmaDescriptorSkip0/*DmaDescriptorSkip2*/ | DmaDescriptor8Words);
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
            TR("%s:: Rx Normal \n", __FUNCTION__);
            u32GmacDmaIE &= ~DmaIntRxNormMask;  // disable RX interrupt
        }

        if (interrupt & synopGMACDmaRxAbnormal)
        {
            TR("%s::Abnormal Rx Interrupt Seen %08x\n", __FUNCTION__, dma_status_reg);

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
        TR("%s::Receiver stopped seeing Rx interrupts\n", __FUNCTION__); //Receiver gone in to stopped state

        if (GMACdev.GMAC_Power_down == 0)   // If Mac is not in powerdown
        {
            GMACdev.synopGMACNetStats.rx_over_errors++;
            synopGMAC_enable_dma_rx(&GMACdev);
        }
    }

    if (interrupt & synopGMACDmaTxNormal)
    {
        TR("%s::Finished Normal Transmission \n", __FUNCTION__);
        synop_handle_transmit_over(&GMACdev);//Do whatever you want after the transmission is over
    }

    if (interrupt & synopGMACDmaTxAbnormal)
    {
        TR0("%s::Abnormal Tx Interrupt Seen\n", __FUNCTION__);

        if (GMACdev.GMAC_Power_down == 0)   // If Mac is not in powerdown
        {
            synop_handle_transmit_over(&GMACdev);
        }
    }

    if (interrupt & synopGMACDmaTxStopped)
    {
        TR("%s::Transmitter stopped sending the packets\n", __FUNCTION__);

        if (GMACdev.GMAC_Power_down == 0)    // If Mac is not in powerdown
        {
            synopGMAC_disable_dma_tx(&GMACdev);
            synopGMAC_take_desc_ownership_tx(&GMACdev);
            synopGMAC_enable_dma_tx(&GMACdev);
            TR("%s::Transmission Resumed\n", __FUNCTION__);
        }
    }

    /* Enable the interrrupt before returning from ISR */
    synopGMAC_enable_interrupt(&GMACdev, u32GmacDmaIE);

    //    if (interrupt & synopGMACDmaRxNormal)
    //    {
    //        EMAC_ReceivePkt();
    //    }

}

extern uint8_t rndis_indata[EMAC_RX_DESC_SIZE + 1][1580];
extern uint32_t u32CurrentRxBuf;
uint32_t EMAC_ReceivePkt(void)
{
    uint32_t len = 0;
    PKT_FRAME_T *psPktFrame;

    if ((len = synop_handle_received_data(&GMACdev, &psPktFrame)) > 0)
    {
        synopGMAC_set_rx_qptr(&GMACdev, (u32)psPktFrame, PKT_FRAME_BUF_SIZE, 0);
        memcpy((uint8_t *)&rndis_indata[u32CurrentRxBuf][44], psPktFrame, len);
        synopGMAC_enable_interrupt(&GMACdev, DmaIntEnable);
    }

    return len;
}

uint8_t *EMAC_AllocatePktBuf(void)
{
    u32 index = GMACdev.TxNext;
    return &tx_buf[index].au8Buf[0];
}

int32_t EMAC_TransmitPkt(uint8_t *pbuf, uint32_t len)
{
    u32 offload_needed;

    if ((pbuf != NULL) && (len > 0) &&
            !synopGMAC_is_desc_owned_by_dma(GMACdev.TxNextDesc))
    {

#if defined(USING_HW_CHECKSUM)
        offload_needed = 1;
#else
        offload_needed = 0;
#endif
        return synopGMAC_xmit_frames(&GMACdev, pbuf, len, offload_needed, 0);
    }

    return -1;
}

uint32_t My_EMAC_SendPkt(uint8_t *pu8Data, uint32_t u32Size)
{
    int32_t ret = -1;
    uint8_t *buf = NULL;

    buf = EMAC_AllocatePktBuf();

    memcpy((uint8_t *)buf, pu8Data, u32Size);
    ret = EMAC_TransmitPkt((uint8_t *)buf, u32Size);

    if (ret == -1)
        return 0;
    else
        return 1;
}

uint32_t EMAC_CheckLinkStatus(void)
{
    uint32_t linkstatus;
    (void)linkstatus;
    linkstatus = (uint32_t)mii_ethtool_gset(&GMACdev, 0);

    return GMACdev.LinkState;
}

uint32_t EMAC_CheckLinkSpeed(void)
{
    uint32_t linkspeed;

    linkspeed = (uint32_t)mii_ethtool_gset(&GMACdev, 0);

    if (linkspeed != 0)
        return 0;//Fail

    if ((GMACdev.Speed & SPEED100) && (GMACdev.DuplexMode & FULLDUPLEX))
    {
        printf("mii:: 100M FULLDUPLEX\n");
        linkspeed = LPA_100FULL;
    }
    else if ((GMACdev.Speed & SPEED100) && (GMACdev.DuplexMode & HALFDUPLEX))
    {
        printf("mii:: 100M HALFDUPLEX\n");
        linkspeed = LPA_100HALF;
    }
    else if ((GMACdev.Speed & SPEED10) && (GMACdev.DuplexMode & FULLDUPLEX))
    {
        printf("mii:: 10M FULLDUPLEX\n");
        linkspeed = LPA_10FULL;
    }
    else if ((GMACdev.Speed & SPEED10) && (GMACdev.DuplexMode & HALFDUPLEX))
    {
        printf("mii:: 10M HALFDUPLEX\n");
        linkspeed = LPA_10FULL;
    }
    else
    {
        printf("mii:: 100M FULLDUPLEX (Default: LPA)\n");
        linkspeed = LPA_100FULL;
    }

    return linkspeed;
}
