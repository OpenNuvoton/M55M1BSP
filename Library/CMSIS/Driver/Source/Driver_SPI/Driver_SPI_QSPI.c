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

/* Driver_SPI_QSPI.c - CMSIS-Driver for Nuvoton QSPI */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _RTE_
    #include "RTE_Components.h"
#endif
/* Project can define PRJ_RTE_DEVICE_HEADER macro to include private or global RTE_Device.h. */
#ifdef   PRJ_RTE_DEVICE_HEADER
    #include PRJ_RTE_DEVICE_HEADER
#else
    #include "../RTE_Device/RTE_Device.h"
#endif

#include "Driver_SPI.h"
#include "drv_pdma.h"
#include "NuMicro.h"
#include "spi_hal.h"

//------------------------------------------------------------------------------
#define RTE_SPI_QSPI0                       RTE_SPI4
#define RTE_SPI_QSPI1                       RTE_SPI5

// Configuration depending on RTE_SPI.h
// Check if at least one peripheral instance is configured in RTE_SPI.h
#if (!(RTE_SPI_QSPI0) && !(RTE_SPI_QSPI1))
    //#warning  SPI driver requires at least one SPI peripheral configured in RTE_SPI.h
#else
    #define DRIVER_CONFIG_VALID     1
#endif

// *****************************************************************************

#ifdef DRIVER_CONFIG_VALID     // Driver code is available only if configuration is valid

#if (RTE_SPI_QSPI0 == 1)
    #define SPI_QSPI_IDX0               4
    #define SPI_QSPI_PORT0              0
    #define QSPI_PORTSYM_FROM_RTE_4     SPI_QSPI_PORT0
#endif

#if (RTE_SPI_QSPI1 == 1)
    #define SPI_QSPI_IDX1               5
    #define SPI_QSPI_PORT1              1
    #define QSPI_PORTSYM_FROM_RTE_5     SPI_QSPI_PORT1
#endif

static inline uint32_t spi_idx_from_port_rt(uint32_t n)
{
    switch (n)
    {
#if (RTE_SPI_QSPI0 == 1)

        case SPI_QSPI_IDX0:
            return SPI_QSPI_PORT0;
#endif

#if (RTE_SPI_QSPI1 == 1)

        case SPI_QSPI_IDX1:
            return SPI_QSPI_PORT1;
#endif

        default:
            return 0;
    }
}

#define QSPI_IDX_FROM_PORT_CAT(x)        QSPI_PORTSYM_FROM_RTE_##x
#define QSPI_IDX_FROM_PORT(x)            QSPI_IDX_FROM_PORT_CAT(x)

#define SPI_TO_QSPI_INSTANCE(n)          ( spi_idx_from_port_rt((uint32_t)(n)) )
#define SPI_TO_QSPI(n)                   ( SPI_CONCAT2(QSPI, QSPI_IDX_FROM_PORT(n)) )

#define SPI_TO_QSPI_XFER_MODE(n)         ( SPI_CONCAT3(RTE_QSPI, QSPI_IDX_FROM_PORT(n), _XFER_MODE) )
#define SPI_TO_QSPI_PDMA_RX(n)           ( SPI_CONCAT3(RTE_QSPI, QSPI_IDX_FROM_PORT(n), _RX_PDMA) )
#define SPI_TO_QSPI_PDMA_TX(n)           ( SPI_CONCAT3(RTE_QSPI, QSPI_IDX_FROM_PORT(n), _TX_PDMA) )
#define SPI_TO_QSPI_PDMA_RX_PORT(n)      ( SPI_CONCAT3(RTE_QSPI, QSPI_IDX_FROM_PORT(n), _RX_PDMA_PORT) )
#define SPI_TO_QSPI_PDMA_TX_PORT(n)      ( SPI_CONCAT3(RTE_QSPI, QSPI_IDX_FROM_PORT(n), _TX_PDMA_PORT) )
#define SPI_TO_QSPI_PDMA_RX_CH(n)        ( SPI_CONCAT3(RTE_QSPI, QSPI_IDX_FROM_PORT(n), _RX_PDMA_CHANNEL) )
#define SPI_TO_QSPI_PDMA_TX_CH(n)        ( SPI_CONCAT3(RTE_QSPI, QSPI_IDX_FROM_PORT(n), _TX_PDMA_CHANNEL) )

#define SPI_TO_QSPI_PDMA_RX_NUM(n)       ( SPI_CONCAT3(PDMA_QSPI, QSPI_IDX_FROM_PORT(n), _RX) )
#define SPI_TO_QSPI_PDMA_TX_NUM(n)       ( SPI_CONCAT3(PDMA_QSPI, QSPI_IDX_FROM_PORT(n), _TX) )

// Local driver functions declarations (for instances)
#define SPI_INFO_DEFINE(n)                                                     \
    static SPI_RESOURCES SPI_RES_NAME(n) = { SPI_TO_QSPI(n),                   \
                                             { \
                                               SPI_TO_QSPI_XFER_MODE(n),       \
                                               0,                              \
                                             },                                \
                                             {0},                              \
                                             {0},                              \
                                             {-1,                              \
                                              -1,                              \
                                              SPI_TO_QSPI_PDMA_RX(n),          \
                                              SPI_TO_QSPI_PDMA_TX(n),          \
                                              SPI_TO_QSPI_PDMA_RX_PORT(n),     \
                                              SPI_TO_QSPI_PDMA_TX_PORT(n),     \
                                              SPI_TO_QSPI_PDMA_RX_CH(n),       \
                                              SPI_TO_QSPI_PDMA_TX_CH(n),       \
                                              SPI_TO_QSPI_PDMA_RX_NUM(n),      \
                                              SPI_TO_QSPI_PDMA_TX_NUM(n),      \
                                             }                                 \
                                           };

// Local driver functions declarations (for instances)
#if (RTE_SPI_QSPI0 == 1)
    SPI_INFO_DEFINE(SPI_QSPI_IDX0)
#endif
#if (RTE_SPI_QSPI1 == 1)
    SPI_INFO_DEFINE(SPI_QSPI_IDX1)
#endif

// List of available SPI instance infos
static const SPI_RESOURCES *const spi_res_list[] =
{
#if defined(RTE_SPI_QSPI0) && (RTE_SPI_QSPI0 == 1)
    &SPI_RES_NAME(SPI_QSPI_IDX0),
#else
    NULL,
#endif

#if defined(RTE_SPI_QSPI1) && (RTE_SPI_QSPI1 == 1)
    &SPI_RES_NAME(SPI_QSPI_IDX1),
#else
    NULL,
#endif

    NULL,
};

/* Local Functions */
static ARM_DRIVER_VERSION SPIn_GetVersion(void);
static ARM_SPI_CAPABILITIES SPIn_GetCapabilities(void);
static int32_t SPIn_Initialize(uint32_t u32Inst, ARM_SPI_SignalEvent_t cb_event);
static int32_t SPIn_Uninitialize(uint32_t u32Inst);
static int32_t SPIn_PowerControl(uint32_t u32Inst, ARM_POWER_STATE state);
static int32_t SPIn_Send(uint32_t u32Inst, const void *pvTxData, uint32_t u32Size);
static int32_t SPIn_Receive(uint32_t u32Inst, void *pvRxData, uint32_t u32Size);
static int32_t SPIn_Transfer(uint32_t u32Inst, const void *data_out, void *data_in, uint32_t num);
static uint32_t SPIn_GetDataCount(uint32_t u32Inst);
static int32_t SPIn_Control(uint32_t u32Inst, uint32_t control, uint32_t arg);
static ARM_SPI_STATUS SPIn_GetStatus(uint32_t u32Inst);

static int32_t SPI_PrepareDefaultRxBuffer(SPI_RESOURCES *pSPIn, uint32_t u32DataBits, uint32_t u32Num);
static int32_t SPI_PrepareDefaultTxBuffer(SPI_RESOURCES *pSPIn, uint32_t u32DataBits, uint32_t u32Num);
static void SPI_ReleaseDefaultTxBuffer(SPI_RESOURCES *pSPIn);
static void SPI_PDMA_TXInit(SPI_RESOURCES *pSPIn);
static void SPI_PDMA_RXInit(SPI_RESOURCES *pSPIn);

#if (RTE_SPI_QSPI0 == 1)
    FUNCS_DECLARE(SPI_QSPI_IDX0)
#endif
#if (RTE_SPI_QSPI1 == 1)
    FUNCS_DECLARE(SPI_QSPI_IDX1)
#endif

//------------------------------------------------------------------------------

// Driver Version
static const ARM_DRIVER_VERSION DriverVersion =
{
    ARM_SPI_API_VERSION,
    0x100
};

// Driver Capabilities
static const ARM_SPI_CAPABILITIES DriverCapabilities =
{
    1U,  // Simplex Mode (Master and Slave)
    0U,  // TI Synchronous Serial Interface
    0U,  // Microwire Interface
    1U,  // Signal Mode Fault event: \ref ARM_SPI_EVENT_MODE_FAULT
    0U   // Reserved
};

//------------------------------------------------------------------------------
static void SPI_PDMA_RX_CB(void *ptr_priv, uint32_t event)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)ptr_priv;
    QSPI_T *phspi = (QSPI_T *)pSPIn->phspi;
    uint32_t u32DataBits = ((phspi->CTL & QSPI_CTL_DWIDTH_Msk) >> QSPI_CTL_DWIDTH_Pos);
    uint32_t item_size = (u32DataBits == 0) ? 4 : ((u32DataBits + 7U) / 8U);
    volatile int32_t timeout = SystemCoreClock / 1000;

    while ((QSPI_IS_BUSY(phspi) || ((phspi->STATUS & QSPI_STATUS_RXCNT_Msk) != 0)) && (--timeout > 0)) {}

    if (event & NU_PDMA_EVENT_TRANSFER_DONE)
    {
        pSPIn->sXfer.u32RxCnt = pSPIn->sXfer.u32Num;
    }
    else
    {
        uint32_t bytes = nu_pdma_transferred_byte_get(pSPIn->spdma.i32RxChnId, pSPIn->sXfer.u32Num * item_size);
        pSPIn->sXfer.u32RxCnt = bytes / item_size;
    }

    if ((pSPIn->sXfer.u32TxCnt >= pSPIn->sXfer.u32Num ||
            pSPIn->sXfer.pu8TxBuf == NULL) &&
            (pSPIn->sXfer.u32RxCnt >= pSPIn->sXfer.u32Num ||
             pSPIn->sXfer.pu8RxBuf == NULL) &&
            pSPIn->sState.sDrvStatus.u8Busy)
    {
        QSPI_DISABLE_TX_RX_PDMA(phspi);
        SPI_ReleaseDefaultTxBuffer(pSPIn);
        pSPIn->sState.sDrvStatus.u8Busy = 0U;

        if (pSPIn->sState.cb_event)
            pSPIn->sState.cb_event(ARM_SPI_EVENT_TRANSFER_COMPLETE);
    }
}

static void QSPI_PDMA_TX_CB(void *ptr_priv, uint32_t event)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)ptr_priv;
    QSPI_T *phspi = (QSPI_T *)pSPIn->phspi;
    uint32_t u32DataBits = ((phspi->CTL & QSPI_CTL_DWIDTH_Msk) >> QSPI_CTL_DWIDTH_Pos);
    uint32_t item_size = (u32DataBits == 0) ? 4 : ((u32DataBits + 7U) / 8U);
    volatile int32_t timeout = SystemCoreClock / 1000;

    while ((QSPI_IS_BUSY(phspi) || ((phspi->STATUS & QSPI_STATUS_TXCNT_Msk) != 0)) && (--timeout > 0)) {}

    if (event & NU_PDMA_EVENT_TRANSFER_DONE)
    {
        pSPIn->sXfer.u32TxCnt = pSPIn->sXfer.u32Num;
    }
    else
    {
        uint32_t bytes = nu_pdma_transferred_byte_get(pSPIn->spdma.i32TxChnId, pSPIn->sXfer.u32Num * item_size);
        pSPIn->sXfer.u32TxCnt = bytes / item_size;
    }

    if ((pSPIn->sXfer.u32TxCnt >= pSPIn->sXfer.u32Num ||
            pSPIn->sXfer.pu8TxBuf == NULL) &&
            (pSPIn->sXfer.u32RxCnt >= pSPIn->sXfer.u32Num ||
             pSPIn->sXfer.pu8RxBuf == NULL) &&
            pSPIn->sState.sDrvStatus.u8Busy)
    {
        QSPI_DISABLE_TX_RX_PDMA(phspi);
        SPI_ReleaseDefaultTxBuffer(pSPIn);
        pSPIn->sState.sDrvStatus.u8Busy = 0U;

        if (pSPIn->sState.cb_event)
            pSPIn->sState.cb_event(ARM_SPI_EVENT_TRANSFER_COMPLETE);
    }
}

static inline void SPI_StoreRxData(SPI_RESOURCES *pSPIn, uint32_t u32RxData, uint32_t u32DataBits)
{
    uint32_t index = pSPIn->sXfer.u32RxCnt;

    if (pSPIn->sXfer.pu8RxBuf == NULL) return;  // Dummy read case

    if ((u32DataBits <= 8) && (u32DataBits != 0))
    {
        pSPIn->sXfer.pu8RxBuf[index] = (uint8_t)u32RxData;
    }
    else if ((u32DataBits <= 16) && (u32DataBits != 0))
    {
        ((uint16_t *)pSPIn->sXfer.pu8RxBuf)[index] = (uint16_t)u32RxData;
    }
    else
    {
        ((uint32_t *)pSPIn->sXfer.pu8RxBuf)[index] = u32RxData;
    }
}

static void SPI_RxIRQHandler(uint32_t u32Inst, uint32_t u32DataBits, uint32_t u32PatternMask)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_QSPI_INSTANCE(u32Inst)];
    QSPI_T *phspi = pSPIn->phspi;
    uint32_t au32RxBuf[8] = {0};  // Local temporary buffer for batch RX from FIFO
    uint32_t u32Count = 0, u32i = 0;

    // Step 1: Batch read all available data from RX FIFO (store raw data first)
    while (!QSPI_GET_RX_FIFO_EMPTY_FLAG(phspi) &&
            (u32Count < SPI_ARRAY_SIZE(au32RxBuf)) &&
            ((pSPIn->sXfer.u32RxCnt + u32Count) < pSPIn->sXfer.u32Num))
    {
        au32RxBuf[u32Count++] = QSPI_READ_RX(phspi);
    }

    // Step 2: Process DataBits conversion after raw data is fetched
    for (u32i = 0; u32i < u32Count; u32i++)
    {
        uint32_t raw = au32RxBuf[u32i] & u32PatternMask;
        SPI_StoreRxData(pSPIn, raw, u32DataBits);
        pSPIn->sXfer.u32RxCnt++;
    }
}

static void SPI_TxIRQHandler(uint32_t u32Inst, uint32_t u32DataBits, uint32_t u32PatternMask)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_QSPI_INSTANCE(u32Inst)];
    QSPI_T *phspi = (QSPI_T *)pSPIn->phspi;
    uint8_t  *pu8TxBuf  = pSPIn->sXfer.pu8TxBuf;
    uint16_t *pu16TxBuf = (uint16_t *)pSPIn->sXfer.pu8TxBuf;
    uint32_t *pu32TxBuf = (uint32_t *)pSPIn->sXfer.pu8TxBuf;
    uint32_t u32TxData = 0;

    // Half-Duplex mode handling TX
    if ((pSPIn->sConfig.u32XferMode == RTE_SPI_HALF_XFER_MODE) &&
            (pSPIn->sXfer.pu8TxBuf == NULL))
    {
        return;
    }

    // Check TX FIFO not full and there's data left to send
    while ((!QSPI_GET_TX_FIFO_FULL_FLAG(phspi)) &&
            (pSPIn->sXfer.u32TxCnt < pSPIn->sXfer.u32Num))
    {
        if (pu8TxBuf == NULL)
        {
            u32TxData = pSPIn->sXfer.u32DefVal;
        }
        else
        {
            // Retrieve data and mask accurately based on DataBits.
            if ((u32DataBits <= 8) && (u32DataBits != 0))
            {
                u32TxData = (uint8_t)(pu8TxBuf[pSPIn->sXfer.u32TxCnt]);
            }
            else if ((u32DataBits <= 16) && (u32DataBits != 0))
            {
                u32TxData = (uint16_t)(pu16TxBuf[pSPIn->sXfer.u32TxCnt]);
            }
            else // default write as 32-bit
            {
                u32TxData = (uint32_t)(pu32TxBuf[pSPIn->sXfer.u32TxCnt]);
            }
        }

        phspi->TX = (u32TxData & u32PatternMask);
        pSPIn->sXfer.u32TxCnt++;
    }

    // Transmission completed, disable interrupt
    if ((QSPI_IS_BUSY(phspi) == 0) && (pSPIn->sXfer.u32TxCnt >= pSPIn->sXfer.u32Num))
    {
        QSPI_ClearIntFlag(phspi, QSPI_FIFO_TXTH_INT_MASK);
        QSPI_DisableInt(phspi, QSPI_FIFO_TXTH_INT_MASK); // Disable TX FIFO threshold interrupt
    }
}

// SPI interrupt handler
static void QSPI_IRQHandler(uint32_t u32Inst)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_QSPI_INSTANCE(u32Inst)];
    QSPI_T *phspi = (QSPI_T *)pSPIn->phspi;
    uint32_t u32DataBits = ((phspi->CTL & QSPI_CTL_DWIDTH_Msk) >> QSPI_CTL_DWIDTH_Pos);
    uint32_t u32PatternMask = (0xFFFFFFFF >> (32 - ((u32DataBits == 0) ? 32 : u32DataBits)));

    if (QSPI_GetIntFlag(phspi, QSPI_SSINACT_INT_MASK))
    {
        QSPI_ClearIntFlag(phspi, QSPI_SSINACT_INT_MASK);
    }

    if (QSPI_GetIntFlag(phspi, QSPI_FIFO_RXTO_INT_MASK))
    {
        QSPI_ClearIntFlag(phspi, QSPI_FIFO_RXTO_INT_MASK);
        SPI_RxIRQHandler(u32Inst, u32DataBits, u32PatternMask);
    }

    if (!QSPI_GET_TX_FIFO_FULL_FLAG(phspi))
    {
        SPI_TxIRQHandler(u32Inst, u32DataBits, u32PatternMask);
    }

    if (!QSPI_GET_RX_FIFO_EMPTY_FLAG(phspi))
    {
        SPI_RxIRQHandler(u32Inst, u32DataBits, u32PatternMask);
    }

    // Relax the Transfer Complete condition
    if ((pSPIn->sXfer.u32RxCnt >= pSPIn->sXfer.u32Num) &&
            (pSPIn->sXfer.u32TxCnt >= pSPIn->sXfer.u32Num) &&
            QSPI_GET_TX_FIFO_EMPTY_FLAG(phspi) &&
            (!QSPI_IS_BUSY(phspi)))
    {
        pSPIn->sState.sDrvStatus.u8Busy = 0U;

        if (pSPIn->sState.cb_event)
        {
            pSPIn->sState.cb_event(ARM_SPI_EVENT_TRANSFER_COMPLETE);
        }
    }
}

// Configure SPI interrupt
static void QSPI_InterruptConfig(uint32_t u32Inst, uint32_t u32IntEn)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_QSPI_INSTANCE(u32Inst)];
    QSPI_T *phspi = (QSPI_T *)pSPIn->phspi;
    IRQn_Type irq_n = QSPI0_IRQn; // Default SPI0 interrupt
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    // Select the corresponding interrupt number based on the SPI instance
    irq_n = ((uint32_t)phspi == (uint32_t)QSPI0) ? QSPI0_IRQn :
            QSPI1_IRQn;

    /* Unlock protected registers */
    if (u32RegLockLevel)
    {
        SYS_UnlockReg();
    }

    if (u32IntEn == SPI_OP_ENABLE)
    {
        // Enable the corresponding SPI interrupt
        NVIC_EnableIRQ(irq_n);
    }
    else
    {
        // Disable the corresponding SPI interrupt
        NVIC_DisableIRQ(irq_n);
    }

    /* Lock protected registers */
    if (u32RegLockLevel)
    {
        SYS_LockReg();
    }
}

/**
  \fn          ARM_DRIVER_VERSION SPI_GetVersion (void)
  \brief       Get SPI driver version.
  \return      \ref ARM_DRV_VERSION
*/
static ARM_DRIVER_VERSION SPIn_GetVersion(void)
{
    return DriverVersion;
}

/**
  \fn          ARM_SPI_CAPABILITIES SPI_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_SPI_CAPABILITIES
*/
static ARM_SPI_CAPABILITIES SPIn_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
  \fn          int32_t SPI_Initialize (ARM_SPI_SignalEvent_t cb_event)
  \brief       Initialize SPI Interface.
  \param[in]   cb_event  Pointer to \ref ARM_SPI_SignalEvent
  \return      \ref execution_status
*/
static int32_t SPIn_Initialize(uint32_t u32Inst, ARM_SPI_SignalEvent_t cb_event)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_QSPI_INSTANCE(u32Inst)];

    if (pSPIn->sState.u8State & SPI_INITIALIZED) return ARM_DRIVER_OK;

    // Initialize SPI Run-Time Resources
    pSPIn->sState.cb_event = cb_event;
    pSPIn->sState.sDrvStatus.u8Busy = 0U;
    pSPIn->sState.sDrvStatus.u8DataLost = 0U;
    pSPIn->sState.sDrvStatus.u8ModeFault = 0U;

    // Clear transfer information
    memset(&pSPIn->sXfer, 0, sizeof(SPI_TRANSFER_INFO));

    // Terminate and free RX PDMA channel if used
    if (pSPIn->spdma.u32RxUsed == 1)
    {
        if ((pSPIn->spdma.i32RxChnId >= 0) &&
                (pSPIn->spdma.i32RxChnId < (int32_t)(PDMA_CH_MAX * PDMA_CNT)))
        {
            nu_pdma_channel_terminate(pSPIn->spdma.i32RxChnId);
            nu_pdma_channel_free(pSPIn->spdma.i32RxChnId);
        }
    }

    // Terminate and free TX PDMA channel if used
    if (pSPIn->spdma.u32TxUsed == 1)
    {
        if ((pSPIn->spdma.i32TxChnId >= 0) &&
                (pSPIn->spdma.i32TxChnId < (int32_t)(PDMA_CH_MAX * PDMA_CNT)))
        {
            nu_pdma_channel_terminate(pSPIn->spdma.i32TxChnId);
            nu_pdma_channel_free(pSPIn->spdma.i32TxChnId);
        }
    }

    // Reset PDMA channel IDs
    pSPIn->spdma.i32RxChnId = -1;
    pSPIn->spdma.i32TxChnId = -1;

    pSPIn->sState.u8State = SPI_INITIALIZED; // SPI is initialized

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SPI_Uninitialize (void)
  \brief       De-initialize SPI Interface.
  \return      \ref execution_status
*/
static int32_t SPIn_Uninitialize(uint32_t u32Inst)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_QSPI_INSTANCE(u32Inst)];
    QSPI_T *phspi = (QSPI_T *)pSPIn->phspi;

    QSPI_InterruptConfig(u32Inst, SPI_OP_DISABLE); // Disable SPI interrupts

    QSPI_Close(phspi);

    // Free PDMA channels
    if (pSPIn->spdma.i32RxChnId != -1)
    {
        nu_pdma_channel_free(pSPIn->spdma.i32RxChnId);
        pSPIn->spdma.i32RxChnId = -1;
    }

    if (pSPIn->spdma.i32TxChnId != -1)
    {
        nu_pdma_channel_free(pSPIn->spdma.i32TxChnId);
        pSPIn->spdma.i32TxChnId = -1;
    }

    pSPIn->sState.u8State = 0U;  // SPI is uninitialized
    memset(&pSPIn->sState.sDrvStatus, 0, sizeof(SPI_DRV_STATUS)); // Clear driver status

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SPI_PowerControl (ARM_POWER_STATE state)
  \brief       Control SPI Interface Power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t SPIn_PowerControl(uint32_t u32Inst, ARM_POWER_STATE state)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_QSPI_INSTANCE(u32Inst)];
    QSPI_T *phspi = (QSPI_T *)pSPIn->phspi;
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    // Check if the power state is valid
    if ((state != ARM_POWER_OFF)  &&
            (state != ARM_POWER_FULL) &&
            (state != ARM_POWER_LOW))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    switch (state)
    {
        case ARM_POWER_OFF:
            QSPI_InterruptConfig(u32Inst, SPI_OP_DISABLE); // Disable SPI interrupts

            // Reset SPI Run-Time Resources
            pSPIn->sState.sDrvStatus.u8Busy = 0U;
            pSPIn->sState.sDrvStatus.u8DataLost = 0U;
            pSPIn->sState.sDrvStatus.u8ModeFault = 0U;

            // Free PDMA channels
            if (pSPIn->spdma.i32RxChnId != -1)
            {
                nu_pdma_channel_free(pSPIn->spdma.i32RxChnId);
                pSPIn->spdma.i32RxChnId = -1;
            }

            if (pSPIn->spdma.i32TxChnId != -1)
            {
                nu_pdma_channel_free(pSPIn->spdma.i32TxChnId);
                pSPIn->spdma.i32TxChnId = -1;
            }

            pSPIn->sState.u8State &= ~SPI_POWERED; // SPI is not powered

            break;

        case ARM_POWER_FULL:
            if ((pSPIn->sState.u8State & SPI_INITIALIZED) == 0U)
            {
                return ARM_DRIVER_ERROR;
            }

            if ((pSPIn->sState.u8State & SPI_POWERED) != 0U)
            {
                return ARM_DRIVER_OK;
            }

            /* Unlock protected registers */
            if (u32RegLockLevel)
            {
                SYS_UnlockReg();
            }

            if ((pSPIn->spdma.u32TxUsed == 1) || (pSPIn->spdma.u32RxUsed == 1))
            {
                CLK_EnableModuleClock(PDMA0_MODULE);
                CLK_EnableModuleClock(PDMA1_MODULE);
            }

            // Reset SPI Run-Time Resources
            pSPIn->sState.sDrvStatus.u8Busy = 0U;
            pSPIn->sState.sDrvStatus.u8DataLost = 0U;
            pSPIn->sState.sDrvStatus.u8ModeFault = 0U;

            // Allocate PDMA RX channel if PDMA RX used
            if ((pSPIn->spdma.u32RxUsed == 1) &&
                    (pSPIn->spdma.i32RxChnId == -1))
            {
                pSPIn->spdma.i32RxChnId = nu_pdma_channel_allocate(
                                              pSPIn->spdma.u32RxPerIpMode,
                                              pSPIn->spdma.u32RxPort,
                                              pSPIn->spdma.u32RxChn);
            }

            // Allocate PDMA TX channel if PDMA TX used
            if ((pSPIn->spdma.u32TxUsed == 1) && (pSPIn->spdma.i32TxChnId == -1))
            {
                pSPIn->spdma.i32TxChnId = nu_pdma_channel_allocate(
                                              pSPIn->spdma.u32TxPerIpMode,
                                              pSPIn->spdma.u32TxPort,
                                              pSPIn->spdma.u32TxChn
                                          );
            }

            /* Lock protected registers */
            if (u32RegLockLevel)
            {
                SYS_LockReg();
            }

            if ((pSPIn->spdma.i32TxChnId == -1) ||
                    (pSPIn->spdma.i32RxChnId == -1))
            {
                // Free PDMA channels
                if (pSPIn->spdma.i32RxChnId != -1)
                {
                    nu_pdma_channel_free(pSPIn->spdma.i32RxChnId);
                    pSPIn->spdma.i32RxChnId = -1;
                }

                if (pSPIn->spdma.i32TxChnId != -1)
                {
                    nu_pdma_channel_free(pSPIn->spdma.i32TxChnId);
                    pSPIn->spdma.i32TxChnId = -1;
                }

                QSPI_InterruptConfig(u32Inst, SPI_OP_ENABLE); // Enable SPI interrupts
            }

            //SPI_SetFIFO(phspi, 4, 4);
            QSPI_ENABLE(phspi);

            pSPIn->sState.u8State |= SPI_POWERED;  // SPI is powered
            break;

        case ARM_POWER_LOW:

            // Check if the SPI is already initialized
            if (!(pSPIn->sState.u8State & SPI_INITIALIZED)) return ARM_DRIVER_ERROR;

            return ARM_DRIVER_ERROR_UNSUPPORTED;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SPI_Send (const void *data, uint32_t num)
  \brief       Start sending data to SPI transmitter.
  \param[in]   data  Pointer to buffer with data to send to SPI transmitter
  \param[in]   num   Number of data items to send
  \return      \ref execution_status
*/
static int32_t SPIn_Send(uint32_t u32Inst, const void *data, uint32_t num)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_QSPI_INSTANCE(u32Inst)];
    QSPI_T *phspi = (QSPI_T *)pSPIn->phspi;

    // Check if data and num are valid
    if ((data == NULL) || (num == 0U))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    // Check if SPI is configured
    if (!(pSPIn->sState.u8State & SPI_CONFIGURED))
    {
        return ARM_DRIVER_ERROR;
    }

    // Check if SPI is busy
    if (pSPIn->sState.sDrvStatus.u8Busy)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    // Set SPI status
    pSPIn->sState.sDrvStatus.u8Busy = 1U;
    pSPIn->sState.sDrvStatus.u8DataLost = 0U;
    pSPIn->sState.sDrvStatus.u8ModeFault = 0U;

    // Set transfer buffers and count
    pSPIn->sXfer.pu8RxBuf = NULL;
    pSPIn->sXfer.pu8TxBuf = (uint8_t *)(uint32_t)data;
    pSPIn->sXfer.u32Num = num;
    pSPIn->sXfer.u32RxCnt = 0U;
    pSPIn->sXfer.u32TxCnt = 0U;

    if (pSPIn->sConfig.u32XferMode == RTE_SPI_HALF_XFER_MODE)
    {
        phspi->CTL |= QSPI_CTL_HALFDPX_Msk;
        phspi->CTL |= QSPI_CTL_DATDIR_Msk; // Set TX mode
    }

    // Check if PDMA channel is available for TX
    if (pSPIn->spdma.i32TxChnId != -1)
    {
        if ((pSPIn->sXfer.pu8RxBuf == NULL) &&
                (pSPIn->sState.u32Mode & ARM_SPI_MODE_SLAVE))
        {
            // Dummy TX buffer for PDMA Send, even in Receive-only mode
            SPI_PrepareDefaultRxBuffer(pSPIn, ((phspi->CTL & QSPI_CTL_DWIDTH_Msk) >> QSPI_CTL_DWIDTH_Pos), num);
        }

        SPI_PDMA_RXInit(pSPIn);
        SPI_PDMA_TXInit(pSPIn);
        /* Enable SPI master DMA function */
        QSPI_TRIGGER_TX_RX_PDMA(phspi);
    }
    else
    {
        QSPI_ClearIntFlag(phspi, QSPI_FIFO_TXTH_INT_MASK | QSPI_FIFO_RXTO_INT_MASK);

        /* Set TX FIFO threshold, enable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
        QSPI_EnableInt(phspi, ((pSPIn->sConfig.u32XferMode == RTE_SPI_HALF_XFER_MODE) ?
                               QSPI_FIFO_TXTH_INT_MASK :
                               (QSPI_FIFO_TXTH_INT_MASK | QSPI_FIFO_RXTO_INT_MASK)));
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SPI_Receive (void *data, uint32_t num)
  \brief       Start receiving data from SPI receiver.
  \param[out]  data  Pointer to buffer for data to receive from SPI receiver
  \param[in]   num   Number of data items to receive
  \return      \ref execution_status
*/
static int32_t SPIn_Receive(uint32_t u32Inst, void *data, uint32_t num)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_QSPI_INSTANCE(u32Inst)];
    QSPI_T *phspi = (QSPI_T *)pSPIn->phspi;

    // Check for valid parameters
    if ((data == NULL) || (num == 0U))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    // Check if SPI is configured
    if (!(pSPIn->sState.u8State & SPI_CONFIGURED))
    {
        return ARM_DRIVER_ERROR;
    }

    // Check if SPI is busy
    if (pSPIn->sState.sDrvStatus.u8Busy)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    // Set SPI as busy
    pSPIn->sState.sDrvStatus.u8Busy = 1U;
    pSPIn->sState.sDrvStatus.u8DataLost = 0U;
    pSPIn->sState.sDrvStatus.u8ModeFault = 0U;

    // Set data buffers and transfer count
    pSPIn->sXfer.pu8RxBuf = (uint8_t *)data;
    pSPIn->sXfer.pu8TxBuf = NULL;

    pSPIn->sXfer.u32Num = num;
    pSPIn->sXfer.u32RxCnt = 0U;
    pSPIn->sXfer.u32TxCnt = 0U;

    if (pSPIn->sConfig.u32XferMode == RTE_SPI_HALF_XFER_MODE)
    {
        phspi->CTL |= QSPI_CTL_HALFDPX_Msk;
        phspi->CTL &= ~QSPI_CTL_DATDIR_Msk; // Set RX mode
    }

    if (pSPIn->spdma.i32RxChnId != -1)
    {
        if (pSPIn->sXfer.pu8TxBuf == NULL)
        {
            // Dummy TX buffer for PDMA Send, even in Receive-only mode
            SPI_PrepareDefaultTxBuffer(pSPIn, ((phspi->CTL & QSPI_CTL_DWIDTH_Msk) >> QSPI_CTL_DWIDTH_Pos), num);

            pSPIn->sXfer.u32TxPrepared = SPI_OP_ENABLE;

            SPI_PDMA_TXInit(pSPIn);
        }

        SPI_PDMA_RXInit(pSPIn);
        /* Enable SPI master DMA function */
        QSPI_TRIGGER_TX_RX_PDMA(phspi);
    }
    else
    {
        QSPI_ClearIntFlag(phspi, QSPI_FIFO_TXTH_INT_MASK | QSPI_FIFO_RXTO_INT_MASK);

        /* Set TX FIFO threshold, enable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
        QSPI_EnableInt(phspi,
                       ((pSPIn->sConfig.u32XferMode == RTE_SPI_HALF_XFER_MODE) ?
                        QSPI_FIFO_RXTO_INT_MASK :
                        (QSPI_FIFO_TXTH_INT_MASK | QSPI_FIFO_RXTO_INT_MASK)));
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SPI_Transfer (const void     *data_out,
                                            void    *data_in,
                                            uint32_t num)
  \brief       Start sending/receiving data to/from SPI transmitter/receiver.
  \param[in]   data_out  Pointer to buffer with data to send to SPI transmitter
  \param[out]  data_in   Pointer to buffer for data to receive from SPI receiver
  \param[in]   num       Number of data items to transfer
  \return      \ref execution_status
*/
static int32_t SPIn_Transfer(uint32_t u32Inst, const void *data_out, void *data_in, uint32_t num)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_QSPI_INSTANCE(u32Inst)];
    QSPI_T *phspi = (QSPI_T *)pSPIn->phspi;

    // Check for valid parameters
    if ((data_out == NULL) || (data_in == NULL) || (num == 0U))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    // Check if SPI is configured
    if (!(pSPIn->sState.u8State & SPI_CONFIGURED))
    {
        return ARM_DRIVER_ERROR;
    }

    // Check if SPI is busy
    if (pSPIn->sState.sDrvStatus.u8Busy)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    // Set SPI as busy
    pSPIn->sState.sDrvStatus.u8Busy = 1U;
    pSPIn->sState.sDrvStatus.u8DataLost = 0U;
    pSPIn->sState.sDrvStatus.u8ModeFault = 0U;

    // Set data buffers and transfer count
    pSPIn->sXfer.pu8RxBuf = (uint8_t *)data_in;
    pSPIn->sXfer.pu8TxBuf = (uint8_t *)(uint32_t)data_out;
    pSPIn->sXfer.u32Num = num;
    pSPIn->sXfer.u32RxCnt = 0U;
    pSPIn->sXfer.u32TxCnt = 0U;

    // **Support Half-Duplex mode**
    if (pSPIn->sConfig.u32XferMode == RTE_SPI_HALF_XFER_MODE)
    {
        phspi->CTL |= QSPI_CTL_HALFDPX_Msk; // Enable Half-Duplex mode

        if (data_out && !data_in)
        {
            phspi->CTL |= QSPI_CTL_DATDIR_Msk; // Set TX mode
        }
        else if (data_in && !data_out)
        {
            phspi->CTL &= ~QSPI_CTL_DATDIR_Msk; // Set RX mode
        }
        else
        {
            return ARM_DRIVER_ERROR_PARAMETER; // In Half-Duplex mode, cannot perform TX and RX simultaneously
        }
    }

    // Initialize and trigger PDMA if available
    if (pSPIn->spdma.i32RxChnId != -1)
    {
        if (data_in && !data_out)  // **Half-Duplex RX Mode**
        {
            SPI_PDMA_RXInit(pSPIn);
            QSPI_TRIGGER_RX_PDMA(phspi);
        }
        else if (data_out && !data_in)  // **Half-Duplex TX Mode**
        {
            SPI_PDMA_TXInit(pSPIn);
            QSPI_TRIGGER_TX_PDMA(phspi);
        }
        else  // **Full-Duplex Mode**
        {
            SPI_PDMA_RXInit(pSPIn);
            SPI_PDMA_TXInit(pSPIn);
            QSPI_TRIGGER_TX_RX_PDMA(phspi);
        }
    }
    else
    {
        QSPI_ClearIntFlag(phspi, QSPI_FIFO_TXTH_INT_MASK | QSPI_FIFO_RXTO_INT_MASK);

        // Set TX FIFO threshold, enable TX FIFO threshold interrupt and RX FIFO time-out interrupt
        if (pSPIn->sConfig.u32XferMode == RTE_SPI_HALF_XFER_MODE)
        {
            if (data_out && !data_in)  // **Half-Duplex TX Mode**
            {
                QSPI_EnableInt(phspi, QSPI_FIFO_TXTH_INT_MASK);
            }
            else if (data_in && !data_out)  // **Half-Duplex RX Mode**
            {
                QSPI_EnableInt(phspi, QSPI_FIFO_RXTO_INT_MASK);
            }
        }
        else  // **Full-Duplex Mode**
        {
            QSPI_EnableInt(phspi, QSPI_FIFO_TXTH_INT_MASK | QSPI_FIFO_RXTO_INT_MASK);
        }
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t SPI_GetDataCount (void)
  \brief       Get transferred data count.
  \return      number of data items transferred
*/
static uint32_t SPIn_GetDataCount(uint32_t u32Inst)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_QSPI_INSTANCE(u32Inst)];

    // Check if the SPI is configured
    if (!(pSPIn->sState.u8State & SPI_CONFIGURED))
    {
        return 0U;
    }

    // If Rx buffer is used → Return RxCnt
    if (pSPIn->sXfer.pu8RxBuf != NULL)
    {
        if (pSPIn->spdma.i32RxChnId != -1)
        {
            pSPIn->sXfer.u32RxCnt = nu_pdma_transferred_byte_get(
                                        pSPIn->spdma.i32RxChnId, pSPIn->sXfer.u32Num);
        }

        return pSPIn->sXfer.u32RxCnt;
    }
    else // Else fallback to TxCnt
    {
        if (pSPIn->spdma.i32TxChnId != -1)
        {
            pSPIn->sXfer.u32TxCnt = nu_pdma_transferred_byte_get(
                                        pSPIn->spdma.i32TxChnId, pSPIn->sXfer.u32Num);
        }

        return pSPIn->sXfer.u32TxCnt;
    }
}

static int32_t SPIn_Control(uint32_t u32Inst, uint32_t control, uint32_t arg)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_QSPI_INSTANCE(u32Inst)];
    QSPI_T *phspi = (QSPI_T *)pSPIn->phspi;
    uint32_t u32DataBits;
    volatile int32_t i32TimeoutCnt = (SystemCoreClock / 1000);

    if (!(pSPIn->sState.u8State & SPI_POWERED))
    {
        return ARM_DRIVER_ERROR;
    }

    if ((control & ARM_SPI_CONTROL_Msk) == ARM_SPI_ABORT_TRANSFER)
    {
        if (pSPIn->spdma.i32TxChnId != -1)
            nu_pdma_channel_terminate(pSPIn->spdma.i32TxChnId);

        if (pSPIn->spdma.i32RxChnId != -1)
            nu_pdma_channel_terminate(pSPIn->spdma.i32RxChnId);

        // Disable TX and RX DMA
        QSPI_DISABLE_TX_RX_PDMA(phspi);
        QSPI_DisableInt(phspi, QSPI_FIFO_TXTH_INT_MASK | QSPI_FIFO_RXTO_INT_MASK);
        memset(&pSPIn->sXfer, 0, sizeof(SPI_TRANSFER_INFO));
        QSPI_DISABLE(phspi); // Disable SPI
        // Release the default TX buffer
        SPI_ReleaseDefaultTxBuffer(pSPIn);
        pSPIn->sState.sDrvStatus.u8Busy = 0U;

        return ARM_DRIVER_OK;
    }

    // Process according to the operation command
    switch (control & ARM_SPI_CONTROL_Msk)
    {
        case ARM_SPI_MODE_INACTIVE:
            pSPIn->sState.u32Mode &= ~ARM_SPI_CONTROL_Msk;
            pSPIn->sState.u32Mode |= ARM_SPI_MODE_INACTIVE;
            QSPI_DISABLE(phspi);
            pSPIn->sState.u8State &= ~SPI_CONFIGURED;
            return ARM_DRIVER_OK;

        case ARM_SPI_MODE_MASTER:
        case ARM_SPI_MODE_SLAVE:
        {
            // Stop SPI and wait for SPIENSTS to be cleared
            QSPI_DISABLE(phspi);
            i32TimeoutCnt = (SystemCoreClock / 1000);

            while (phspi->STATUS & SPI_STATUS_SPIENSTS_Msk)
            {
                if (--i32TimeoutCnt <= 0)
                    return ARM_DRIVER_ERROR;
            }

            // Clear control settings
            phspi->CTL = 0;
            phspi->SSCTL = 0;

            if ((control & ARM_SPI_CONTROL_Msk) == ARM_SPI_MODE_MASTER)
            {
                phspi->CTL |= QSPI_MASTER;
                pSPIn->sState.u32Mode &= ~ARM_SPI_CONTROL_Msk;
                pSPIn->sState.u32Mode |= ARM_SPI_MODE_MASTER;
                goto set_speed;
            }
            else
            {
                phspi->CTL |= QSPI_SLAVE;
                phspi->CLKDIV = 2;

                if (pSPIn->sConfig.u32XferMode == RTE_SPI_3WIRE_XFER_MODE)
                {
                    phspi->SSCTL |= QSPI_SSCTL_SLV3WIRE_Msk;
                }

                pSPIn->sState.u32Mode &= ~ARM_SPI_CONTROL_Msk;
                pSPIn->sState.u32Mode |= ARM_SPI_MODE_SLAVE;
                break;
            }
        }

        case ARM_SPI_SET_BUS_SPEED:
set_speed:
            pSPIn->sConfig.u32BusSpeed = QSPI_SetBusClock(phspi, arg);

            if (pSPIn->sConfig.u32BusSpeed == 0)
                return ARM_DRIVER_ERROR;

            if ((control & ARM_SPI_CONTROL_Msk) == ARM_SPI_SET_BUS_SPEED)
                return ARM_DRIVER_OK;

            break;

        case ARM_SPI_GET_BUS_SPEED:
            return pSPIn->sConfig.u32BusSpeed;

        case ARM_SPI_SET_DEFAULT_TX_VALUE:
            pSPIn->sXfer.u32DefVal = arg;
            return ARM_DRIVER_OK;

        case ARM_SPI_CONTROL_SS:
            if ((arg != ARM_SPI_SS_ACTIVE) && (arg != ARM_SPI_SS_INACTIVE))
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }

            if ((pSPIn->sState.u32Mode & ARM_SPI_CONTROL_Msk) == ARM_SPI_MODE_MASTER &&
                    (pSPIn->sState.u32Mode & ARM_SPI_SS_MASTER_MODE_Msk) == ARM_SPI_SS_MASTER_SW)
            {

                QSPI_DisableAutoSS(phspi);
                (arg == ARM_SPI_SS_INACTIVE) ? QSPI_SET_SS_HIGH(phspi) : QSPI_SET_SS_LOW(phspi);
                return ARM_DRIVER_OK;
            }

            if ((pSPIn->sState.u32Mode & ARM_SPI_CONTROL_Msk) == ARM_SPI_MODE_SLAVE &&
                    (pSPIn->sState.u32Mode & ARM_SPI_SS_SLAVE_MODE_Msk) == ARM_SPI_SS_SLAVE_SW)
            {

                QSPI_DisableAutoSS(phspi);
                (arg == ARM_SPI_SS_INACTIVE) ? QSPI_SET_SS_HIGH(phspi) : QSPI_SET_SS_LOW(phspi);
                return ARM_DRIVER_OK;
            }

            return ARM_DRIVER_ERROR;

        default:
            return ARM_DRIVER_ERROR_PARAMETER;
    }

    // SS Master mode configuration
    if ((pSPIn->sState.u32Mode & ARM_SPI_CONTROL_Msk) == ARM_SPI_MODE_MASTER)
    {
        switch (control & ARM_SPI_SS_MASTER_MODE_Msk)
        {
            case ARM_SPI_SS_MASTER_UNUSED:
                QSPI_DisableAutoSS(phspi);
                phspi->SSCTL |= QSPI_SSCTL_SLV3WIRE_Msk;
                pSPIn->sState.u32Mode &= ~ARM_SPI_SS_MASTER_MODE_Msk;
                pSPIn->sState.u32Mode |= ARM_SPI_SS_MASTER_UNUSED;
                break;

            case ARM_SPI_SS_MASTER_HW_INPUT:
                return ARM_SPI_ERROR_SS_MODE;

            case ARM_SPI_SS_MASTER_SW:
                QSPI_DisableAutoSS(phspi);
                pSPIn->sState.u32Mode &= ~ARM_SPI_SS_MASTER_MODE_Msk;
                pSPIn->sState.u32Mode |= ARM_SPI_SS_MASTER_SW;
                break;

            case ARM_SPI_SS_MASTER_HW_OUTPUT:
                QSPI_EnableAutoSS(phspi, QSPI_SS, QSPI_SS_ACTIVE_LOW);
                pSPIn->sState.u32Mode &= ~ARM_SPI_SS_MASTER_MODE_Msk;
                pSPIn->sState.u32Mode |= ARM_SPI_SS_MASTER_HW_OUTPUT;
                break;

            default:
                break;
        }
    }

    // SS Slave mode configuration
    if ((pSPIn->sState.u32Mode & ARM_SPI_CONTROL_Msk) == ARM_SPI_MODE_SLAVE)
    {
        switch (control & ARM_SPI_SS_SLAVE_MODE_Msk)
        {
            case ARM_SPI_SS_SLAVE_HW:
                QSPI_EnableAutoSS(phspi, QSPI_SS, QSPI_SS_ACTIVE_LOW);
                pSPIn->sState.u32Mode &= ~ARM_SPI_SS_SLAVE_MODE_Msk;
                pSPIn->sState.u32Mode |= ARM_SPI_SS_SLAVE_HW;
                break;

            case ARM_SPI_SS_SLAVE_SW:
                if ((arg != ARM_SPI_SS_ACTIVE) && (arg != ARM_SPI_SS_INACTIVE))
                {
                    return ARM_DRIVER_ERROR_PARAMETER;
                }

                QSPI_DisableAutoSS(phspi);
                (arg == ARM_SPI_SS_INACTIVE) ? QSPI_SET_SS_HIGH(phspi) : QSPI_SET_SS_LOW(phspi);
                pSPIn->sState.u32Mode &= ~ARM_SPI_SS_SLAVE_MODE_Msk;
                pSPIn->sState.u32Mode |= ARM_SPI_SS_SLAVE_SW;
                break;
        }
    }

    // Frame Format
    switch (control & ARM_SPI_FRAME_FORMAT_Msk)
    {
        case ARM_SPI_CPOL0_CPHA0:
            phspi->CTL = (phspi->CTL & ~(QSPI_CTL_CLKPOL_Msk | QSPI_CTL_TXNEG_Msk | QSPI_CTL_RXNEG_Msk)) | QSPI_MODE_0;
            break;

        case ARM_SPI_CPOL0_CPHA1:
            phspi->CTL = (phspi->CTL & ~(QSPI_CTL_CLKPOL_Msk | QSPI_CTL_TXNEG_Msk | QSPI_CTL_RXNEG_Msk)) | QSPI_MODE_1;
            break;

        case ARM_SPI_CPOL1_CPHA0:
            phspi->CTL = (phspi->CTL & ~(QSPI_CTL_CLKPOL_Msk | QSPI_CTL_TXNEG_Msk | QSPI_CTL_RXNEG_Msk)) | QSPI_MODE_2;
            break;

        case ARM_SPI_CPOL1_CPHA1:
            phspi->CTL = (phspi->CTL & ~(QSPI_CTL_CLKPOL_Msk | QSPI_CTL_TXNEG_Msk | QSPI_CTL_RXNEG_Msk)) | QSPI_MODE_3;
            break;

        default:
            return ARM_SPI_ERROR_FRAME_FORMAT;
    }

    // Data Bits
    u32DataBits = ((control & ARM_SPI_DATA_BITS_Msk) >> ARM_SPI_DATA_BITS_Pos);

    if ((u32DataBits > 0) && (u32DataBits < 8))
    {
        return ARM_SPI_ERROR_DATA_BITS;
    }

    phspi->CTL = (phspi->CTL & ~QSPI_CTL_DWIDTH_Msk) |
                 (((u32DataBits >= 32) ? 0 : u32DataBits) << QSPI_CTL_DWIDTH_Pos);

    // MSB / LSB First
    if ((control & ARM_SPI_BIT_ORDER_Msk) == ARM_SPI_LSB_MSB)
    {
        QSPI_SET_LSB_FIRST(phspi);
    }
    else
    {
        QSPI_SET_MSB_FIRST(phspi);
    }

    QSPI_SetFIFO(phspi, 7, 7);

    // Clear RX & TX FIFOs
    QSPI_ClearRxFIFO(phspi);
    QSPI_ClearTxFIFO(phspi);

    // Enable SPI if needed
    if (pSPIn->sState.u8State & SPI_POWERED)
    {
        QSPI_ENABLE(phspi);
    }

    pSPIn->sState.u8State |= SPI_CONFIGURED;

    return ARM_DRIVER_OK;
}

/**
  \fn          ARM_SPI_STATUS SPI_GetStatus (void)
  \brief       Get SPI status.
  \return      SPI status \ref ARM_SPI_STATUS
*/
static ARM_SPI_STATUS SPIn_GetStatus(uint32_t u32Inst)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_QSPI_INSTANCE(u32Inst)];
    ARM_SPI_STATUS status = {0};

    status.busy = pSPIn->sState.sDrvStatus.u8Busy;
    status.data_lost = pSPIn->sState.sDrvStatus.u8DataLost;
    status.mode_fault = pSPIn->sState.sDrvStatus.u8ModeFault;

    return (status);
}

static int32_t SPI_PrepareDefaultRxBuffer(SPI_RESOURCES *pSPIn, uint32_t u32DataBits, uint32_t u32Num)
{
    uint32_t u32ItemSize = (u32DataBits == 0) ? 4 : ((u32DataBits + 7) / 8);
    uint32_t u32TotalBytes = u32Num * u32ItemSize;

    if ((u32Num == 0) || (pSPIn->sXfer.pu8RxBuf != NULL))
        return ARM_DRIVER_OK;

    void *pBuf = malloc(u32TotalBytes);

    if (!pBuf)
        return ARM_DRIVER_ERROR;

    // Store the result and pointer of malloc for TX use
    pSPIn->sXfer.pu8RxBuf = (uint8_t *)pBuf;

    pSPIn->sXfer.u32RxPrepared = SPI_OP_ENABLE;

    return ARM_DRIVER_OK;
}

static int32_t SPI_PrepareDefaultTxBuffer(SPI_RESOURCES *pSPIn, uint32_t u32DataBits, uint32_t u32Num)
{
    uint32_t u32DefVal = pSPIn->sXfer.u32DefVal;
    uint32_t u32ItemSize = (u32DataBits == 0) ? 4 : ((u32DataBits + 7) / 8);
    uint32_t u32TotalBytes = u32Num * u32ItemSize;

    void *pBuf = malloc(u32TotalBytes);

    if (!pBuf)
        return ARM_DRIVER_ERROR;

    if ((u32DataBits <= 8) && (u32DataBits > 0))
    {
        uint8_t *buf = (uint8_t *)pBuf;

        for (uint32_t i = 0; i < u32Num; i++)
            buf[i] = (uint8_t)(u32DefVal & 0xFF);
    }
    else if ((u32DataBits <= 16) && (u32DataBits > 0))
    {
        uint16_t *buf = (uint16_t *)pBuf;

        for (uint32_t i = 0; i < u32Num; i++)
            buf[i] = (uint16_t)(u32DefVal & 0xFFFF);
    }
    else
    {
        uint32_t *buf = (uint32_t *)pBuf;

        for (uint32_t i = 0; i < u32Num; i++)
            buf[i] = u32DefVal;
    }

    // Store the result and pointer of malloc for TX use
    pSPIn->sXfer.pu8TxBuf = (uint8_t *)pBuf;

    return ARM_DRIVER_OK;
}

static void SPI_ReleaseDefaultTxBuffer(SPI_RESOURCES *pSPIn)
{
    if ((pSPIn->sXfer.pu8TxBuf == NULL) ||
            ((!pSPIn->sXfer.u32TxPrepared) && (!pSPIn->sXfer.u32RxPrepared)))
        return;

    if (pSPIn->sXfer.u32TxPrepared == SPI_OP_ENABLE)
    {
        pSPIn->sXfer.u32TxPrepared = SPI_OP_DISABLE;
        free(pSPIn->sXfer.pu8TxBuf);
        pSPIn->sXfer.pu8TxBuf = NULL;
    }

    if (pSPIn->sXfer.u32RxPrepared)
    {
        pSPIn->sXfer.u32RxPrepared = SPI_OP_DISABLE;
        free(pSPIn->sXfer.pu8RxBuf);
        pSPIn->sXfer.pu8RxBuf = NULL;
    }
}

static void SPI_PDMA_TXInit(SPI_RESOURCES *pSPIn)
{
    QSPI_T *phspi = (QSPI_T *)pSPIn->phspi;
    uint32_t u32DataBits = ((phspi->CTL & (QSPI_CTL_DWIDTH_Msk)) >> QSPI_CTL_DWIDTH_Pos);
    //Use PDMA transfer mode
    struct nu_pdma_chn_cb pdma_chn_cb;

    if (pSPIn->sXfer.pu8TxBuf == NULL)
    {
        return;
    }

    pdma_chn_cb.m_eCBType = eCBType_Event;
    pdma_chn_cb.m_pfnCBHandler = QSPI_PDMA_TX_CB;
    pdma_chn_cb.m_pvUserData = (void *)pSPIn;

    //Use PDMA transfer mode
    nu_pdma_filtering_set(pSPIn->spdma.i32TxChnId, NU_PDMA_EVENT_TRANSFER_DONE);
    nu_pdma_callback_register(pSPIn->spdma.i32TxChnId, &pdma_chn_cb);
    nu_pdma_transfer(
        pSPIn->spdma.i32TxChnId,
        ((u32DataBits <= 8) && (u32DataBits > 0)) ? 8
        : ((u32DataBits > 8) && (u32DataBits <= 16)) ? 16
        : 32,
        (uint32_t)pSPIn->sXfer.pu8TxBuf,
        (uint32_t)&phspi->TX,
        pSPIn->sXfer.u32Num,
        0);
}

static void SPI_PDMA_RXInit(SPI_RESOURCES *pSPIn)
{
    QSPI_T *phspi = (QSPI_T *)pSPIn->phspi;
    uint32_t u32DataBits = ((phspi->CTL & (QSPI_CTL_DWIDTH_Msk)) >> QSPI_CTL_DWIDTH_Pos);
    //Use PDMA transfer mode
    struct nu_pdma_chn_cb pdma_chn_cb;

    if (pSPIn->sXfer.pu8RxBuf == NULL)
    {
        return;
    }

    pdma_chn_cb.m_eCBType = eCBType_Event;
    pdma_chn_cb.m_pfnCBHandler = SPI_PDMA_RX_CB;
    pdma_chn_cb.m_pvUserData = (void *)pSPIn;

    //Use PDMA transfer mode
    nu_pdma_filtering_set(pSPIn->spdma.i32RxChnId, NU_PDMA_EVENT_TRANSFER_DONE);
    nu_pdma_callback_register(pSPIn->spdma.i32RxChnId, &pdma_chn_cb);
    nu_pdma_transfer(
        pSPIn->spdma.i32RxChnId,
        ((u32DataBits <= 8) && (u32DataBits > 0)) ? 8
        : ((u32DataBits > 8) && (u32DataBits <= 16)) ? 16
        : 32,
        (uint32_t)&phspi->RX,
        (uint32_t)pSPIn->sXfer.pu8RxBuf,
        pSPIn->sXfer.u32Num,
        0);
}

// Local driver functions definitions (for instances)
// Information definitions (for instances)
#if (RTE_SPI_QSPI0 == 1)
FUNCS_DEFINE(SPI_QSPI_IDX0)
SPI_DRIVER(SPI_QSPI_IDX0)

NVT_ITCM void QSPI0_IRQHandler(void)
{
    QSPI_IRQHandler(SPI_QSPI_IDX0);
}
#endif

#if (RTE_SPI_QSPI1 == 1)
FUNCS_DEFINE(SPI_QSPI_IDX1)
SPI_DRIVER(SPI_QSPI_IDX1)

NVT_ITCM void QSPI1_IRQHandler(void)
{
    QSPI_IRQHandler(SPI_QSPI_IDX1);
}
#endif

#endif //DRIVER_CONFIG_VALID
