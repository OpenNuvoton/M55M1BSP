/***************************************************************************//**
 * @file     config_pdma.c
 * @version  V3.00
 * @brief    PDMA setting.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*!<Includes */
#include <string.h>
#include "NuMicro.h"
#include "usbd_audio.h"

#if (NVT_DCACHE_ON == 1)
    /* Declare the DMA scatter-gather descriptor buffer as non-cacheable. */
    NVT_NONCACHEABLE DMA_DESC_T DMA_TXDESC[PDMA_TXBUFFER_CNT];
    NVT_NONCACHEABLE DMA_DESC_T DMA_RXDESC[PDMA_RXBUFFER_CNT];
#else
    DMA_DESC_T DMA_TXDESC[PDMA_TXBUFFER_CNT];
    DMA_DESC_T DMA_RXDESC[PDMA_RXBUFFER_CNT];
#endif

/* PDMA Interrupt handler */
NVT_ITCM void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if (u32Status & 0x1)   /* abort */
    {
        if (PDMA_GET_ABORT_STS(PDMA0) & 0x4)
            PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if (u32Status & 0x2)
    {
        if (PDMA_GET_TD_STS(PDMA0) & 0x2)            /* channel 1 done, Tx */
        {
            PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF1_Msk);

            /* Decrease number of full buffer */
            if (u8TxDataCntInBuffer > 0)
                u8TxDataCntInBuffer --;

            /* Change to next buffer */
            u8PDMATxIdx ++;

            if (u8PDMATxIdx >= PDMA_TXBUFFER_CNT)
                u8PDMATxIdx = 0;
        }

        if (PDMA_GET_TD_STS(PDMA0) & 0x4)            /* channel 2 done, Rx */
        {

            PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);

            /* Set PCM buffer full flag */
            u8PcmRxBufFull[u8PDMARxIdx] = 1;
            u8RxDataCntInBuffer++;

            /* Change to next buffer */
            u8PDMARxIdx ++;

            if (u8PDMARxIdx >= PDMA_RXBUFFER_CNT)
                u8PDMARxIdx = 0;
        }
    }
}

// Configure PDMA to Scatter Gather mode */
void PDMA_Init(void)
{
    PDMA_WriteTxSGTable();
    PDMA_WriteRxSGTable();

    /* Open PDMA channel 1 for I2S TX and channel 2 for I2S RX */
    PDMA_Open(PDMA0, (1 << PDMA_I2S_TX_CH) | (1 << PDMA_I2S_RX_CH));

    /* We want to enable these channels at run time */
    PDMA0->CHCTL = 0;

    /* Configure PDMA transfer mode */
    PDMA_SetTransferMode(PDMA0, PDMA_I2S_TX_CH, PDMA_I2S0_TX, 1, (uint32_t)&DMA_TXDESC[0]);
    PDMA_SetTransferMode(PDMA0, PDMA_I2S_RX_CH, PDMA_I2S0_RX, 1, (uint32_t)&DMA_RXDESC[0]);

    /* Enable PDMA channel 1&2 interrupt */
    PDMA_EnableInt(PDMA0, PDMA_I2S_TX_CH, 0);
    PDMA_EnableInt(PDMA0, PDMA_I2S_RX_CH, 0);

    /* Enable PDMA interrupt */
    NVIC_EnableIRQ(PDMA0_IRQn);
}

/* init TX scatter-gather table */
void PDMA_WriteTxSGTable(void)
{
    uint16_t i;

    /* Use PDMA_TXBUFFER_CNT scatter-gather tables and link with each other */
    for (i = 0; i < PDMA_TXBUFFER_CNT; i++)
    {
        DMA_TXDESC[i].ctl = ((u32BuffLen - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
        DMA_TXDESC[i].src = (uint32_t)&PcmPlayBuff[i];
        DMA_TXDESC[i].dest = (uint32_t)&I2S0->TXFIFO;


        if (i != PDMA_TXBUFFER_CNT - 1)
            DMA_TXDESC[i].offset = (uint32_t)&DMA_TXDESC[i + 1];
        else
            DMA_TXDESC[i].offset = (uint32_t)&DMA_TXDESC[0];
    }
}

/* init RX scatter-gather table */
void PDMA_WriteRxSGTable(void)
{
    uint16_t i;

    /* Use PDMA_RXBUFFER_CNT scatter-gather tables and link with each other */
    for (i = 0; i < PDMA_RXBUFFER_CNT; i++)
    {
        DMA_RXDESC[i].ctl = (((u32RxBuffLen / 4) - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
        DMA_RXDESC[i].src = (uint32_t)&I2S0->RXFIFO;
        DMA_RXDESC[i].dest = (uint32_t)&PcmRecBuff[i];

        if (i != (PDMA_RXBUFFER_CNT - 1))
            DMA_RXDESC[i].offset = (uint32_t)&DMA_RXDESC[i + 1];
        else
            DMA_RXDESC[i].offset = (uint32_t)&DMA_RXDESC[0];
    }
}
