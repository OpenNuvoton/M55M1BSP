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

/* Driver_SAI_I2S.c - CMSIS-Driver for Nuvoton I2S
   The implementation does not include pin/clock settings,
    which must be configured in the application.
*/

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

#include "Driver_SAI.h"
#include "drv_pdma.h"
#include "NuMicro.h"
#include "i2s_hal.h"

//------------------------------------------------------------------------------
#define RTE_SAI_SPII2S0                     RTE_SAI2
#define RTE_SAI_SPII2S1                     RTE_SAI3
#define RTE_SAI_SPII2S2                     RTE_SAI4
#define RTE_SAI_SPII2S3                     RTE_SAI5

// Configuration depending on RTE_SAI.h
// Check if at least one peripheral instance is configured in RTE_SAI.h
#if (!(RTE_SAI_SPII2S0) && !(RTE_SAI_SPII2S1) && !(RTE_SAI_SPII2S2) && !(RTE_SAI_SPII2S3))
    #warning  I2S driver requires at least one I2S peripheral configured in RTE_SAI.h
#else
    #define DRIVER_CONFIG_VALID     1
#endif

#if defined (RTE_Driver_SPI)
    #if RTE_SAI2 && RTE_SPI0
        #error "SPI0 is used by multiple CMSIS Drivers! Please check RTE device configuration to fix it."
    #endif

    #if RTE_SAI3 && RTE_SPI1
        #error "SPI1 is used by multiple CMSIS Drivers! Please check RTE device configuration to fix it."
    #endif

    #if RTE_SAI4 && RTE_SPI2
        #error "SPI2 is used by multiple CMSIS Drivers! Please check RTE device configuration to fix it."
    #endif

    #if RTE_SAI5 && RTE_SPI3
        #error "SPI3 is used by multiple CMSIS Drivers! Please check RTE device configuration to fix it."
    #endif
#endif

// *****************************************************************************

#ifdef DRIVER_CONFIG_VALID          // Driver code is available only if configuration is valid

#define SAI_ARRAY_SIZE(arr)                 (sizeof(arr) / sizeof((arr)[0]))

#if (RTE_SAI_SPII2S0 == 1)
    // Define index and port for SAI_SPII2S0
    #define SAI_SPII2S_IDX0                 2
    #define SAI_SPII2S_PORT0                0
    // Map the index to the corresponding port symbol
    #define SAI_SPII2S_PORTSYM_FROM_RTE_2   SAI_SPII2S_PORT0
#endif

#if (RTE_SAI_SPII2S1 == 1)
    // Define index and port for SAI_SPII2S1
    #define SAI_SPII2S_IDX1                 3
    #define SAI_SPII2S_PORT1                1
    // Map the index to the corresponding port symbol
    #define SAI_SPII2S_PORTSYM_FROM_RTE_3   SAI_SPII2S_PORT1
#endif

#if (RTE_SAI_SPII2S2 == 1)
    // Define index and port for SAI_SPII2S2
    #define SAI_SPII2S_IDX2                 4
    #define SAI_SPII2S_PORT2                2
    // Map the index to the corresponding port symbol
    #define SAI_SPII2S_PORTSYM_FROM_RTE_4   SAI_SPII2S_PORT2
#endif

#if (RTE_SAI_SPII2S3 == 1)
    // Define index and port for SAI_SPII2S3
    #define SAI_SPII2S_IDX3                 5
    #define SAI_SPII2S_PORT3                3
    // Map the index to the corresponding port symbol
    #define SAI_SPII2S_PORTSYM_FROM_RTE_5   SAI_SPII2S_PORT3
#endif

static inline uint32_t sai_idx_from_port_rt(uint32_t n)
{
    switch (n)
    {
#if (RTE_SAI_SPII2S0 == 1)

        case SAI_SPII2S_IDX0:
            return SAI_SPII2S_PORT0;
#endif
#if (RTE_SAI_SPII2S1 == 1)

        case SAI_SPII2S_IDX1:
            return SAI_SPII2S_PORT1;
#endif
#if (RTE_SAI_SPII2S2 == 1)

        case SAI_SPII2S_IDX2:
            return SAI_SPII2S_PORT2;
#endif
#if (RTE_SAI_SPII2S3 == 1)

        case SAI_SPII2S_IDX3:
            return SAI_SPII2S_PORT3;
#endif

        default:
            return 0;
    }
}

#define SAI_IDX_FROM_PORT_CAT(x)    SAI_SPII2S_PORTSYM_FROM_RTE_##x
#define SAI_IDX_FROM_PORT(x)        SAI_IDX_FROM_PORT_CAT(x)

#define SAI_TO_SPI_INSTANCE(n)        (sai_idx_from_port_rt((uint32_t)(n)))
#define SAI_TO_SPI(n)                 ( SAI_CAT2(SPI, SAI_IDX_FROM_PORT(n)) )

#define SAI_TO_SPI_PDMA_RX(n)         ( SAI_CAT3(RTE_SAI, SAI_IDX_FROM_PORT(n), _RX_PDMA) )
#define SAI_TO_SPI_PDMA_TX(n)         ( SAI_CAT3(RTE_SAI, SAI_IDX_FROM_PORT(n), _TX_PDMA) )
#define SAI_TO_SPI_PDMA_RX_PORT(n)    ( SAI_CAT3(RTE_SAI, SAI_IDX_FROM_PORT(n), _RX_PDMA_PORT) )
#define SAI_TO_SPI_PDMA_TX_PORT(n)    ( SAI_CAT3(RTE_SAI, SAI_IDX_FROM_PORT(n), _TX_PDMA_PORT) )
#define SAI_TO_SPI_PDMA_RX_CH(n)      ( SAI_CAT3(RTE_SAI, SAI_IDX_FROM_PORT(n), _RX_PDMA_CHANNEL) )
#define SAI_TO_SPI_PDMA_TX_CH(n)      ( SAI_CAT3(RTE_SAI, SAI_IDX_FROM_PORT(n), _TX_PDMA_CHANNEL) )

#define SAI_TO_SPI_PDMA_RX_NUM(n)     ( SAI_CAT3(PDMA_SPI, SAI_IDX_FROM_PORT(n), _RX) )
#define SAI_TO_SPI_PDMA_TX_NUM(n)     ( SAI_CAT3(PDMA_SPI, SAI_IDX_FROM_PORT(n), _TX) )

// Local driver functions declarations (for instances)
#define I2S_INFO_DEFINE(n)                                                     \
    static I2S_RESOURCES SAI_RES_NAME(n) = { SAI_TO_SPI(n),                    \
                                             {0},                              \
                                             {-1,                              \
                                              -1,                              \
                                              SAI_TO_SPI_PDMA_RX(n),           \
                                              SAI_TO_SPI_PDMA_TX(n),           \
                                              SAI_TO_SPI_PDMA_RX_PORT(n),      \
                                              SAI_TO_SPI_PDMA_TX_PORT(n),      \
                                              SAI_TO_SPI_PDMA_RX_CH(n),        \
                                              SAI_TO_SPI_PDMA_TX_CH(n),        \
                                              SAI_TO_SPI_PDMA_RX_NUM(n),       \
                                              SAI_TO_SPI_PDMA_TX_NUM(n),       \
                                             }                                \
                                           };

// Local driver functions declarations (for instances)
#if (RTE_SAI_SPII2S0 == 1)
    I2S_INFO_DEFINE(SAI_SPII2S_IDX0)
#endif

#if (RTE_SAI_SPII2S1 == 1)
    I2S_INFO_DEFINE(SAI_SPII2S_IDX1)
#endif

#if (RTE_SAI_SPII2S2 == 1)
    I2S_INFO_DEFINE(SAI_SPII2S_IDX2)
#endif

#if (RTE_SAI_SPII2S3 == 1)
    I2S_INFO_DEFINE(SAI_SPII2S_IDX3)
#endif

// List of available I2S instance infos
static const I2S_RESOURCES *const i2s_res_list[] =
{
#if defined(RTE_SAI_SPII2S0) && (RTE_SAI_SPII2S0 == 1)
    &SAI_RES_NAME(SAI_SPII2S_IDX0),
#else
    NULL,
#endif

#if defined(RTE_SAI_SPII2S1) && (RTE_SAI_SPII2S1 == 1)
    &SAI_RES_NAME(SAI_SPII2S_IDX1),
#else
    NULL,
#endif

#if defined(RTE_SAI_SPII2S2) && (RTE_SAI_SPII2S2 == 1)
    &SAI_RES_NAME(SAI_SPII2S_IDX2),
#else
    NULL,
#endif

#if defined(RTE_SAI_SPII2S3) && (RTE_SAI_SPII2S3 == 1)
    &SAI_RES_NAME(SAI_SPII2S_IDX3),
#else
    NULL,
#endif

    NULL,
};

/* Local Functions */
static ARM_DRIVER_VERSION SAIn_GetVersion(void);
static ARM_SAI_CAPABILITIES SAIn_GetCapabilities(void);
static int32_t SAIn_Initialize(uint32_t u32Inst, ARM_SAI_SignalEvent_t cb_event);
static int32_t SAIn_Uninitialize(uint32_t u32Inst);
static int32_t SAIn_PowerControl(uint32_t u32Inst, ARM_POWER_STATE state);
static int32_t SAIn_Send(uint32_t u32Inst, const void *pvTxData, uint32_t u32Size);
static int32_t SAIn_Receive(uint32_t u32Inst, void *pvRxData, uint32_t u32Size);
static uint32_t SAIn_GetTxDataCount(uint32_t u32Inst);
static uint32_t SAIn_GetRxDataCount(uint32_t u32Inst);
static int32_t SAIn_Control(uint32_t u32Inst, uint32_t control, uint32_t arg1, uint32_t arg2);
static ARM_SAI_STATUS SAIn_GetStatus(uint32_t u32Inst);

static void SAI_PDMA_TXInit(I2S_RESOURCES *pI2Sn);
static void SAI_PDMA_RXInit(I2S_RESOURCES *pI2Sn);

#if (RTE_SAI_SPII2S0 == 1)
    FUNCS_DECLARE(SAI_SPII2S_IDX0)
#endif

#if (RTE_SAI_SPII2S1 == 1)
    FUNCS_DECLARE(SAI_SPII2S_IDX1)
#endif

#if (RTE_SAI_SPII2S2 == 1)
    FUNCS_DECLARE(SAI_SPII2S_IDX2)
#endif

#if (RTE_SAI_SPII2S3 == 1)
    FUNCS_DECLARE(SAI_SPII2S_IDX3)
#endif

//------------------------------------------------------------------------------
// Driver Version
static const ARM_DRIVER_VERSION DriverVersion =
{
    ARM_SAI_API_VERSION,
    ARM_SAI_DRV_VERSION
};

static const ARM_SAI_CAPABILITIES DriverCapabilities =
{
    // Capabilities
    1U, ///< supports asynchronous Transmit/Receive
    1U, ///< supports synchronous Transmit/Receive
    0U, ///< supports user defined Protocol
    1U, ///< supports I2S Protocol
    1U, ///< supports MSB/LSB justified Protocol
    1U, ///< supports PCM short/long frame Protocol
    0U, ///< supports AC'97 Protocol
    1U, ///< supports Mono mode
    0U, ///< supports Companding
    1U, ///< supports MCLK (Master Clock) pin
    0U, ///< supports Frame error event: \ref ARM_SAI_EVENT_FRAME_ERROR
    0U ///< reserved bits
};

//------------------------------------------------------------------------------
static void SAI_PDMA_RX_CB(void *ptr_priv, uint32_t event)
{
    // Wait for SPI to become idle
    I2S_RESOURCES *pI2Sn = (I2S_RESOURCES *)ptr_priv;
    SPI_T *phi2s = (SPI_T *)pI2Sn->phi2s;
    uint32_t u32DataBits = ((phi2s->I2SCTL & SPI_I2SCTL_WDWIDTH_Msk) >> SPI_I2SCTL_WDWIDTH_Pos);
    uint32_t item_size = ((u32DataBits + 1U) * 8U);

    if (event & NU_PDMA_EVENT_TRANSFER_DONE)
    {
        pI2Sn->sInfo.sRx.u32Cnt = pI2Sn->sInfo.sRx.u32Num;
    }
    else
    {
        uint32_t bytes = nu_pdma_transferred_byte_get(pI2Sn->spdma.i32RxChnId, pI2Sn->sInfo.sRx.u32Num * item_size);
        pI2Sn->sInfo.sRx.u32Cnt = bytes / item_size;
    }

    if ((pI2Sn->sInfo.sRx.u32Cnt >= pI2Sn->sInfo.sRx.u32Num ||
            pI2Sn->sInfo.sRx.pu8Buf == NULL) &&
            pI2Sn->sInfo.sStatus.u8RxBusy)
    {
        SPII2S_DISABLE_RXDMA(phi2s);

        pI2Sn->sInfo.sStatus.u8RxBusy = 0U;

        if (pI2Sn->sInfo.cb_event)
            pI2Sn->sInfo.cb_event(ARM_SAI_EVENT_RECEIVE_COMPLETE);
    }
}

static void SAI_PDMA_TX_CB(void *ptr_priv, uint32_t event)
{
    // Wait for SPI to become idle
    I2S_RESOURCES *pI2Sn = (I2S_RESOURCES *)ptr_priv;
    SPI_T *phi2s = (SPI_T *)pI2Sn->phi2s;
    uint32_t u32DataBits = ((phi2s->I2SCTL & SPI_I2SCTL_WDWIDTH_Msk) >> SPI_I2SCTL_WDWIDTH_Pos);
    uint32_t item_size = ((u32DataBits + 1U) * 8U);

    if (event & NU_PDMA_EVENT_TRANSFER_DONE)
    {
        pI2Sn->sInfo.sTx.u32Cnt = pI2Sn->sInfo.sTx.u32Num;
    }
    else
    {
        uint32_t bytes = nu_pdma_transferred_byte_get(pI2Sn->spdma.i32TxChnId, pI2Sn->sInfo.sTx.u32Num * item_size);
        pI2Sn->sInfo.sTx.u32Cnt = bytes / item_size;
    }

    if ((pI2Sn->sInfo.sTx.u32Cnt >= pI2Sn->sInfo.sTx.u32Num ||
            pI2Sn->sInfo.sTx.pu8Buf == NULL) &&
            pI2Sn->sInfo.sStatus.u8TxBusy)
    {
        SPII2S_DISABLE_TXDMA(phi2s);

        pI2Sn->sInfo.sStatus.u8TxBusy = 0U;

        if (pI2Sn->sInfo.cb_event)
            pI2Sn->sInfo.cb_event(ARM_SAI_EVENT_SEND_COMPLETE);
    }
}

static inline void SPI_StoreRxData(I2S_RESOURCES *pI2Sn, uint32_t u32RxData, uint32_t u32DataBits)
{
    uint32_t index = pI2Sn->sInfo.sRx.u32Cnt;

    if (pI2Sn->sInfo.sRx.pu8Buf == NULL) return;  // Dummy read case

    if ((u32DataBits <= 8) && (u32DataBits != 0))
    {
        pI2Sn->sInfo.sRx.pu8Buf[index] = (uint8_t)u32RxData;
    }
    else if ((u32DataBits <= 16) && (u32DataBits != 0))
    {
        ((uint16_t *)pI2Sn->sInfo.sRx.pu8Buf)[index] = (uint16_t)u32RxData;
    }
    else
    {
        ((uint32_t *)pI2Sn->sInfo.sRx.pu8Buf)[index] = u32RxData;
    }
}

static void SPI_RxIRQHandler(uint32_t u32Inst, uint32_t u32DataBits, uint32_t u32PatternMask)
{
    I2S_RESOURCES *pI2Sn = (I2S_RESOURCES *)i2s_res_list[SAI_TO_SPI_INSTANCE(u32Inst)];
    SPI_T *phi2s = (SPI_T *)pI2Sn->phi2s;
    uint32_t au32RxBuf[8] = {0};  // Local temporary buffer for batch RX from FIFO
    uint32_t u32Count = 0, u32i = 0;

    // Step 1: Batch read all available data from RX FIFO (store raw data first)
    while (!SPI_GET_RX_FIFO_EMPTY_FLAG(phi2s) &&
            (u32Count < SAI_ARRAY_SIZE(au32RxBuf)) &&
            ((pI2Sn->sInfo.sRx.u32Cnt + u32Count) < pI2Sn->sInfo.sRx.u32Num))
    {
        au32RxBuf[u32Count++] = SPI_READ_RX(phi2s);
    }

    // Step 2: Process DataBits conversion after raw data is fetched
    for (u32i = 0; u32i < u32Count; u32i++)
    {
        uint32_t raw = au32RxBuf[u32i] & u32PatternMask;
        SPI_StoreRxData(pI2Sn, raw, u32DataBits);
        pI2Sn->sInfo.sRx.u32Cnt++;
    }

    if (pI2Sn->sInfo.sRx.u32Cnt >= pI2Sn->sInfo.sRx.u32Num)
    {
        // No more space to receive, disable RX interrupt
        SPII2S_DisableInt(phi2s, SPII2S_FIFO_RXTH_INT_MASK);
        //u32RxZeroSkipCnt = 0;
        pI2Sn->sInfo.sStatus.u8RxBusy = 0U;

        // Trigger RX complete callback if registered
        if (pI2Sn->sInfo.cb_event)
            pI2Sn->sInfo.cb_event(ARM_SAI_EVENT_RECEIVE_COMPLETE);
    }
}

static void SPI_TxIRQHandler(uint32_t u32Inst, uint32_t u32DataBits, uint32_t u32PatternMask)
{
    I2S_RESOURCES *pI2Sn = (I2S_RESOURCES *)i2s_res_list[SAI_TO_SPI_INSTANCE(u32Inst)];
    SPI_T *phi2s = (SPI_T *)pI2Sn->phi2s;
    uint8_t  *pu8TxBuf  = pI2Sn->sInfo.sTx.pu8Buf;
    uint16_t *pu16TxBuf = (uint16_t *)pI2Sn->sInfo.sTx.pu8Buf;
    uint32_t *pu32TxBuf = (uint32_t *)pI2Sn->sInfo.sTx.pu8Buf;
    uint32_t u32TxData = 0;

    // Check TX FIFO not full and there's data left to send
    while ((!SPI_GET_TX_FIFO_FULL_FLAG(phi2s)) &&
            (pI2Sn->sInfo.sTx.u32Cnt < pI2Sn->sInfo.sTx.u32Num))
    {
        // Retrieve data and mask accurately based on DataBits.
        if ((u32DataBits <= 8) && (u32DataBits != 0))
        {
            u32TxData = (uint8_t)(pu8TxBuf[pI2Sn->sInfo.sTx.u32Cnt]);
        }
        else if ((u32DataBits <= 16) && (u32DataBits != 0))
        {
            u32TxData = (uint16_t)(pu16TxBuf[pI2Sn->sInfo.sTx.u32Cnt]);
        }
        else // default write as 32-bit
        {
            u32TxData = (uint32_t)(pu32TxBuf[pI2Sn->sInfo.sTx.u32Cnt]);
        }

        phi2s->TX = (u32TxData & u32PatternMask);
        pI2Sn->sInfo.sTx.u32Cnt++;
    }

    // Transmission completed, disable interrupt
    if (pI2Sn->sInfo.sTx.u32Cnt >= pI2Sn->sInfo.sTx.u32Num)
    {
        SPI_ClearIntFlag(phi2s, SPI_FIFO_TXTH_INT_MASK);
        SPI_DisableInt(phi2s, SPI_FIFO_TXTH_INT_MASK); // Disable TX FIFO threshold interrupt
        pI2Sn->sInfo.sStatus.u8TxBusy = 0U;

        if (pI2Sn->sInfo.cb_event)
            pI2Sn->sInfo.cb_event(ARM_SAI_EVENT_SEND_COMPLETE);
    }
}

static void SAI_IRQHandler(uint32_t u32Inst)
{
    I2S_RESOURCES *pI2Sn = (I2S_RESOURCES *)i2s_res_list[SAI_TO_SPI_INSTANCE(u32Inst)];
    SPI_T *phi2s = (SPI_T *)pI2Sn->phi2s;
    //volatile uint32_t status = SPII2S_GET_INT_FLAG(phi2s, (SPII2S_FIFO_TXTH_INT_MASK | SPII2S_FIFO_RXTH_INT_MASK |
    //                                                       SPII2S_TXUF_INT_MASK | SPII2S_FIFO_RXOV_INT_MASK));
    uint32_t u32DataBits = ((((phi2s->I2SCTL & SPI_I2SCTL_WDWIDTH_Msk) >> SPI_I2SCTL_WDWIDTH_Pos) + 1) * 8);
    uint32_t u32PatternMask = (0xFFFFFFFF >> (32 - u32DataBits));

    SPI_TxIRQHandler(u32Inst, u32DataBits, u32PatternMask);
    SPI_RxIRQHandler(u32Inst, u32DataBits, u32PatternMask);

    // TX underflow
    if ((phi2s->I2SSTS & SPI_I2SSTS_TXUFIF_Msk) &&
            (phi2s->FIFOCTL & SPI_FIFOCTL_TXUFIEN_Msk))
    {
        phi2s->I2SSTS = SPI_I2SSTS_TXUFIF_Msk;

        if (pI2Sn->sInfo.cb_event)
            pI2Sn->sInfo.cb_event(ARM_SAI_EVENT_TX_UNDERFLOW);
    }

    // RX overflow
    if ((phi2s->I2SSTS & SPI_I2SSTS_RXOVIF_Msk) &&
            (phi2s->FIFOCTL & SPI_FIFOCTL_RXOVIEN_Msk))
    {
        phi2s->I2SSTS = SPI_I2SSTS_RXOVIF_Msk;

        if (pI2Sn->sInfo.cb_event)
            pI2Sn->sInfo.cb_event(ARM_SAI_EVENT_RX_OVERFLOW);
    }
}

// Configure I2S interrupt
static void I2S_InterruptConfig(uint32_t u32Inst, uint32_t u32IntEn)
{
    I2S_RESOURCES *pI2Sn = (I2S_RESOURCES *)i2s_res_list[SAI_TO_SPI_INSTANCE(u32Inst)];
    SPI_T *phi2s = (SPI_T *)pI2Sn->phi2s;
    IRQn_Type irq_n = SPI0_IRQn; // Default I2S0 interrupt
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    // Select the corresponding interrupt number based on the I2S instance
    irq_n = ((uint32_t)phi2s == (uint32_t)SPI0) ? SPI0_IRQn :
            ((uint32_t)phi2s == (uint32_t)SPI1) ? SPI1_IRQn :
            ((uint32_t)phi2s == (uint32_t)SPI2) ? SPI2_IRQn :
            ((uint32_t)phi2s == (uint32_t)SPI3) ? SPI3_IRQn :
            SPI0_IRQn;

    /* Unlock protected registers */
    if (u32RegLockLevel)
    {
        SYS_UnlockReg();
    }

    if (u32IntEn == SAI_OP_ENABLE)
    {
        // Enable the corresponding I2S interrupt
        NVIC_EnableIRQ(irq_n);
    }
    else
    {
        // Disable the corresponding I2S interrupt
        NVIC_DisableIRQ(irq_n);
    }

    /* Lock protected registers */
    if (u32RegLockLevel)
    {
        SYS_LockReg();
    }
}

/**
  \fn          ARM_DRIVER_VERSION SAIn_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION SAIn_GetVersion(void)
{
    return (DriverVersion);
}

/**
  \fn          ARM_SAI_CAPABILITIES SAIn_GetCapabilities (void)
  \brief       Get driver capabilities.
  \param[in]   i2s       Pointer to I2S resources
  \return      \ref ARM_SAI_CAPABILITIES
*/
static ARM_SAI_CAPABILITIES SAIn_GetCapabilities(void)
{
    return (DriverCapabilities);
}

/**
  \fn          int32_t SAIn_Initialize (ARM_SAI_SignalEvent_t cb_event, I2S_RESOURCES *i2s)
  \brief       Initialize I2S Interface.
  \param[in]   cb_event  Pointer to \ref ARM_SAI_SignalEvent
  \param[in]   i2s       Pointer to I2S resources
  \return      \ref execution_status
*/
static int32_t SAIn_Initialize(uint32_t u32Inst, ARM_SAI_SignalEvent_t cb_event)
{
    I2S_RESOURCES *pI2Sn = (I2S_RESOURCES *)i2s_res_list[SAI_TO_SPI_INSTANCE(u32Inst)];

    if (pI2Sn->sInfo.u8Flags & SAI_FLAG_INITIALIZED)
    {
        // Driver is already initialized
        return ARM_DRIVER_OK;
    }

    // Initialize I2S Run-Time resources
    pI2Sn->sInfo.cb_event               = cb_event;
    pI2Sn->sInfo.sStatus.u8FrameError   = 0U;
    pI2Sn->sInfo.sStatus.u8RxBusy       = 0U;
    pI2Sn->sInfo.sStatus.u8RxOverflow   = 0U;
    pI2Sn->sInfo.sStatus.u8TxBusy       = 0U;
    pI2Sn->sInfo.sStatus.u8TxUnderflow  = 0U;

    // Terminate and free RX PDMA channel if used
    if (pI2Sn->spdma.u32RxUsed == 1)
    {
        if ((pI2Sn->spdma.i32RxChnId >= 0) &&
                (pI2Sn->spdma.i32RxChnId < (int32_t)(PDMA_CH_MAX * PDMA_CNT)))
        {
            nu_pdma_channel_terminate(pI2Sn->spdma.i32RxChnId);
            nu_pdma_channel_free(pI2Sn->spdma.i32RxChnId);
        }
    }

    // Terminate and free TX PDMA channel if used
    if (pI2Sn->spdma.u32TxUsed == 1)
    {
        if ((pI2Sn->spdma.i32TxChnId >= 0) &&
                (pI2Sn->spdma.i32TxChnId < (int32_t)(PDMA_CH_MAX * PDMA_CNT)))
        {
            nu_pdma_channel_terminate(pI2Sn->spdma.i32TxChnId);
            nu_pdma_channel_free(pI2Sn->spdma.i32TxChnId);
        }
    }

    // Reset PDMA channel IDs
    pI2Sn->spdma.i32RxChnId = -1;
    pI2Sn->spdma.i32TxChnId = -1;

    pI2Sn->sInfo.u8Flags = SAI_FLAG_INITIALIZED;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SAIn_Uninitialize (uint32_t u32Inst)
  \brief       De-initialize I2S Interface.
  \param[in]   i2s       Pointer to I2S resources
  \return      \ref execution_status
*/
static int32_t SAIn_Uninitialize(uint32_t u32Inst)
{
    I2S_RESOURCES *pI2Sn = (I2S_RESOURCES *)i2s_res_list[SAI_TO_SPI_INSTANCE(u32Inst)];
    SPI_T *phi2s = (SPI_T *)pI2Sn->phi2s;

    I2S_InterruptConfig(u32Inst, SAI_OP_DISABLE); // Disable SPI interrupts

    SPII2S_Close(phi2s);

    // Free PDMA channels
    if (pI2Sn->spdma.i32RxChnId != -1)
    {
        nu_pdma_channel_free(pI2Sn->spdma.i32RxChnId);
        pI2Sn->spdma.i32RxChnId = -1;
    }

    if (pI2Sn->spdma.i32TxChnId != -1)
    {
        nu_pdma_channel_free(pI2Sn->spdma.i32TxChnId);
        pI2Sn->spdma.i32TxChnId = -1;
    }

    memset(&pI2Sn->sInfo, 0, sizeof(I2S_INFO));

    // Reset I2S status flags
    pI2Sn->sInfo.u8Flags = 0U;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SAIn_PowerControl (uint32_t u32Inst, ARM_POWER_STATE state)
  \brief       Control I2S Interface Power.
  \param[in]   state    Power state
  \param[in]   i2s      Pointer to I2S resources
  \return      \ref execution_status
*/
static int32_t SAIn_PowerControl(uint32_t u32Inst, ARM_POWER_STATE state)
{
    I2S_RESOURCES *pI2Sn = (I2S_RESOURCES *)i2s_res_list[SAI_TO_SPI_INSTANCE(u32Inst)];
    SPI_T *phi2s = (SPI_T *)pI2Sn->phi2s;
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    if ((state != ARM_POWER_OFF)  &&
            (state != ARM_POWER_FULL) &&
            (state != ARM_POWER_LOW))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    switch (state)
    {
        case ARM_POWER_OFF:
            // Disable I2S IRQ
            I2S_InterruptConfig(u32Inst, SAI_OP_DISABLE); // Disable SPI interrupts

            // Free PDMA channels
            if (pI2Sn->spdma.i32RxChnId != -1)
            {
                nu_pdma_channel_free(pI2Sn->spdma.i32RxChnId);
                pI2Sn->spdma.i32RxChnId = -1;
            }

            if (pI2Sn->spdma.i32TxChnId != -1)
            {
                nu_pdma_channel_free(pI2Sn->spdma.i32TxChnId);
                pI2Sn->spdma.i32TxChnId = -1;
            }

            // Clear driver variables
            pI2Sn->sInfo.sStatus.u8FrameError   = 0U;
            pI2Sn->sInfo.sStatus.u8RxBusy       = 0U;
            pI2Sn->sInfo.sStatus.u8RxOverflow   = 0U;
            pI2Sn->sInfo.sStatus.u8TxBusy       = 0U;
            pI2Sn->sInfo.sStatus.u8TxUnderflow  = 0U;

            pI2Sn->sInfo.u8Flags &= ~SAI_FLAG_POWERED;
            break;

        case ARM_POWER_LOW:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_POWER_FULL:
            if ((pI2Sn->sInfo.u8Flags & SAI_FLAG_INITIALIZED) == 0U)
            {
                return ARM_DRIVER_ERROR;
            }

            if ((pI2Sn->sInfo.u8Flags & SAI_FLAG_POWERED)     != 0U)
            {
                return ARM_DRIVER_OK;
            }

            /* Unlock protected registers */
            if (u32RegLockLevel)
            {
                SYS_UnlockReg();
            }

            if ((pI2Sn->spdma.u32TxUsed == 1) || (pI2Sn->spdma.u32RxUsed == 1))
            {
                CLK_EnableModuleClock(PDMA0_MODULE);
                CLK_EnableModuleClock(PDMA1_MODULE);
            }

            // Allocate PDMA RX channel if PDMA RX used
            if ((pI2Sn->spdma.u32RxUsed == 1) &&
                    (pI2Sn->spdma.i32RxChnId == -1))
            {
                pI2Sn->spdma.i32RxChnId = nu_pdma_channel_allocate(
                                              pI2Sn->spdma.u32RxPerIpMode,
                                              pI2Sn->spdma.u32RxPort,
                                              pI2Sn->spdma.u32RxChn);
            }

            // Allocate PDMA TX channel if PDMA TX used
            if ((pI2Sn->spdma.u32TxUsed == 1) && (pI2Sn->spdma.i32TxChnId == -1))
            {
                pI2Sn->spdma.i32TxChnId = nu_pdma_channel_allocate(
                                              pI2Sn->spdma.u32TxPerIpMode,
                                              pI2Sn->spdma.u32TxPort,
                                              pI2Sn->spdma.u32TxChn
                                          );
            }

            /* Lock protected registers */
            if (u32RegLockLevel)
            {
                SYS_LockReg();
            }

            // Clear driver variables
            pI2Sn->sInfo.sStatus.u8FrameError   = 0U;
            pI2Sn->sInfo.sStatus.u8RxBusy       = 0U;
            pI2Sn->sInfo.sStatus.u8RxOverflow   = 0U;
            pI2Sn->sInfo.sStatus.u8TxBusy       = 0U;
            pI2Sn->sInfo.sStatus.u8TxUnderflow  = 0U;

            if ((pI2Sn->spdma.i32TxChnId == -1) ||
                    (pI2Sn->spdma.i32RxChnId == -1))
            {
                // Free PDMA channels
                if (pI2Sn->spdma.i32RxChnId != -1)
                {
                    nu_pdma_channel_free(pI2Sn->spdma.i32RxChnId);
                    pI2Sn->spdma.i32RxChnId = -1;
                }

                if (pI2Sn->spdma.i32TxChnId != -1)
                {
                    nu_pdma_channel_free(pI2Sn->spdma.i32TxChnId);
                    pI2Sn->spdma.i32TxChnId = -1;
                }

                I2S_InterruptConfig(u32Inst, SAI_OP_ENABLE); // Enable SPI interrupts
            }

            // Enable I2S
            phi2s->I2SCTL |= (SPI_I2SCTL_I2SEN_Msk);

            pI2Sn->sInfo.u8Flags = (SAI_FLAG_POWERED | SAI_FLAG_INITIALIZED);
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SAIn_Send (const void *data, uint32_t num, I2S_RESOURCES *i2s)
  \brief       Start sending data to I2S transmitter.
  \param[in]   data  Pointer to buffer with data to send to I2S transmitter
  \param[in]   num   Number of data items to send
  \param[in]   i2s       Pointer to I2S resources
  \return      \ref execution_status
*/
static int32_t SAIn_Send(uint32_t u32Inst, const void *data, uint32_t num)
{
    I2S_RESOURCES *pI2Sn = (I2S_RESOURCES *)i2s_res_list[SAI_TO_SPI_INSTANCE(u32Inst)];
    SPI_T *phi2s = (SPI_T *)pI2Sn->phi2s;

    if ((data == NULL) || (num == 0U))
    {
        // Invalid parameters
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if ((pI2Sn->sInfo.u8Flags & SAI_FLAG_CONFIGURED) == 0U)
    {
        // I2S is not configured (mode not selected)
        return ARM_DRIVER_ERROR;
    }

    if (pI2Sn->sInfo.sStatus.u8TxBusy)
    {
        // Send is not completed yet
        return ARM_DRIVER_ERROR_BUSY;
    }

    // Set Send active flag
    pI2Sn->sInfo.sStatus.u8TxBusy = 1U;

    // Clear TX underflow flag
    pI2Sn->sInfo.sStatus.u8TxUnderflow = 0U;

    // Save transmit buffer info
    pI2Sn->sInfo.sTx.pu8Buf = (uint8_t *)(uint32_t)data;
    pI2Sn->sInfo.sTx.u32Cnt = 0U;
    pI2Sn->sInfo.sTx.u32Num = num;

    // Check if PDMA channel is available for TX
    if (pI2Sn->spdma.i32TxChnId != -1)
    {
        SAI_PDMA_TXInit(pI2Sn);
        /* Enable I2S master DMA function */
        SPII2S_ENABLE_TXDMA(phi2s);
    }
    else
    {
        SPII2S_CLR_INT_FLAG(phi2s, SPII2S_FIFO_TXTH_INT_MASK);

        SPII2S_EnableInt(phi2s, SPII2S_FIFO_TXTH_INT_MASK);
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SAIn_Receive (uint32_t u32Inst, void *data, uint32_t num)
  \brief       Start receiving data from I2S receiver.
  \param[out]  data  Pointer to buffer for data to receive from I2S receiver
  \param[in]   num   Number of data items to receive
  \param[in]   i2s       Pointer to I2S resources
  \return      \ref execution_status
*/
static int32_t SAIn_Receive(uint32_t u32Inst, void *data, uint32_t num)
{
    I2S_RESOURCES *pI2Sn = (I2S_RESOURCES *)i2s_res_list[SAI_TO_SPI_INSTANCE(u32Inst)];
    SPI_T *phi2s = (SPI_T *)pI2Sn->phi2s;

    if ((data == NULL) || (num == 0U))
    {
        // Invalid parameters
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if ((pI2Sn->sInfo.u8Flags & SAI_FLAG_CONFIGURED) == 0U)
    {
        // I2S is not configured (mode not selected)
        return ARM_DRIVER_ERROR;
    }

    if (pI2Sn->sInfo.sStatus.u8RxBusy)
    {
        // Receive is not completed yet
        return ARM_DRIVER_ERROR_BUSY;
    }

    // Set Receive active flag
    pI2Sn->sInfo.sStatus.u8RxBusy = 1U;

    // Clear RX overflow flag
    pI2Sn->sInfo.sStatus.u8RxOverflow = 0U;

    // Save receive buffer info
    pI2Sn->sInfo.sRx.pu8Buf = (uint8_t *)data;
    pI2Sn->sInfo.sRx.u32Cnt = 0U;
    pI2Sn->sInfo.sRx.u32Num = num;

    if (pI2Sn->spdma.i32RxChnId != -1)
    {
        SAI_PDMA_RXInit(pI2Sn);

        // Enable I2S master DMA function
        SPII2S_ENABLE_RXDMA(phi2s);
    }
    else
    {
        SPII2S_CLR_INT_FLAG(phi2s, SPII2S_FIFO_RXTH_INT_MASK);
        SPII2S_EnableInt(phi2s, SPII2S_FIFO_RXTH_INT_MASK);
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t SAIn_GetTxDataCount (uint32_t u32Inst)
  \brief       Get transmitted data count.
  \param[in]   i2s       Pointer to I2S resources
  \return      number of data items transmitted
*/
static uint32_t SAIn_GetTxDataCount(uint32_t u32Inst)
{
    I2S_RESOURCES *pI2Sn = (I2S_RESOURCES *)i2s_res_list[SAI_TO_SPI_INSTANCE(u32Inst)];

    // Check if the SPI has an associated DMA channel for receiving data
    if (pI2Sn->spdma.i32TxChnId != -1)
    {
        // Check if the received data count is not equal to the expected count
        if (pI2Sn->sInfo.sTx.u32Cnt != pI2Sn->sInfo.sTx.u32Num)
            pI2Sn->sInfo.sTx.u32Cnt = nu_pdma_transferred_byte_get(
                                          pI2Sn->spdma.i32TxChnId, pI2Sn->sInfo.sTx.u32Num);
    }

    return (pI2Sn->sInfo.sTx.u32Cnt);
}

/**
  \fn          uint32_t SAIn_GetRxDataCount (uint32_t u32Inst)
  \brief       Get received data count.
  \param[in]   i2s       Pointer to I2S resources
  \return      number of data items received
*/
static uint32_t SAIn_GetRxDataCount(uint32_t u32Inst)
{
    I2S_RESOURCES *pI2Sn = (I2S_RESOURCES *)i2s_res_list[SAI_TO_SPI_INSTANCE(u32Inst)];

    // Check if the SPI has an associated DMA channel for receiving data
    if (pI2Sn->spdma.i32RxChnId != -1)
    {
        // Check if the received data count is not equal to the expected count
        if (pI2Sn->sInfo.sRx.u32Cnt != pI2Sn->sInfo.sRx.u32Num)
            pI2Sn->sInfo.sRx.u32Cnt = nu_pdma_transferred_byte_get(
                                          pI2Sn->spdma.i32RxChnId, pI2Sn->sInfo.sRx.u32Num);
    }

    return (pI2Sn->sInfo.sRx.u32Cnt);
}

/**
  \fn          int32_t SAIn_Control (uint32_t u32Inst, uint32_t control, uint32_t arg1, uint32_t arg2)
  \brief       Control I2S Interface.
  \param[in]   control  Operation
  \param[in]   arg1     Argument 1 of operation (optional)
  \param[in]   arg2     Argument 2 of operation (optional)
  \param[in]   i2s      Pointer to I2S resources
  \return      common \ref execution_status and driver specific \ref sai_execution_status
*/
static int32_t SAIn_Control(uint32_t u32Inst, uint32_t control, uint32_t arg1, uint32_t arg2)
{
    I2S_RESOURCES *pI2Sn = (I2S_RESOURCES *)i2s_res_list[SAI_TO_SPI_INSTANCE(u32Inst)];
    SPI_T *phi2s = (SPI_T *)pI2Sn->phi2s;
    uint32_t u32Mode, u32Protocol, u32DataBits, /* u32LSBFirst, */ u32MonoMode;
    uint32_t u32SampleRate, u32MclkPrescaler/* , u32SlotCount, u32SlotSize */;
    uint32_t u32Ctrl = (control & ARM_SAI_CONTROL_Msk);

    if (!(pI2Sn->sInfo.u8Flags & SAI_FLAG_POWERED))
        return ARM_DRIVER_ERROR;  // Driver not powered

    if ((pI2Sn->sInfo.sStatus.u8TxBusy) ||
            (pI2Sn->sInfo.sStatus.u8RxBusy))
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    // --------- Step 1: Handle CMSIS runtime control commands ---------
    switch (u32Ctrl)
    {
        case ARM_SAI_CONFIGURE_TX:
        case ARM_SAI_CONFIGURE_RX:
            if (pI2Sn->sInfo.u8Flags & SAI_FLAG_CONFIGURED)
            {
                return ARM_DRIVER_OK;
            }

            break; // Proceed to configuration

        case ARM_SAI_CONTROL_TX:
            if (arg1 & 0x01)
            {
                if (pI2Sn->spdma.i32TxChnId != -1)
                {
                    SPII2S_ENABLE_TXDMA(phi2s);
                }

                SPII2S_ENABLE_TX(phi2s);
            }
            else
            {
                if (pI2Sn->spdma.i32TxChnId != -1)
                {
                    SPII2S_DISABLE_TXDMA(phi2s);
                }

                SPII2S_DISABLE_TX(phi2s);
            }

            if (pI2Sn->sInfo.u8Flags & SAI_FLAG_CONFIGURED)
            {
                return ARM_DRIVER_OK;
            }

            break;

        case ARM_SAI_CONTROL_RX:
            if (arg1 & 0x01)
            {
                if (pI2Sn->spdma.i32RxChnId != -1)
                {
                    SPII2S_ENABLE_RXDMA(phi2s);
                }

                SPII2S_ENABLE_RX(phi2s);
            }
            else
            {
                if (pI2Sn->spdma.i32RxChnId != -1)
                {
                    SPII2S_DISABLE_RXDMA(phi2s);
                }

                SPII2S_DISABLE_RX(phi2s);
            }

            if (pI2Sn->sInfo.u8Flags & SAI_FLAG_CONFIGURED)
            {
                return ARM_DRIVER_OK;
            }

            break;

        case ARM_SAI_MASK_SLOTS_TX:
            return ARM_DRIVER_ERROR;

        case ARM_SAI_MASK_SLOTS_RX:

            SPII2S_SET_MONO_RX_CHANNEL(phi2s, (arg1 == 0x02) ? SPII2S_MONO_LEFT : SPII2S_MONO_RIGHT);

            if (pI2Sn->sInfo.u8Flags & SAI_FLAG_CONFIGURED)
            {
                return ARM_DRIVER_OK;
            }

            break;

        case ARM_SAI_ABORT_SEND:
            if (pI2Sn->spdma.i32TxChnId != -1)
            {
                SPII2S_DISABLE_TXDMA(phi2s);
            }

            SPII2S_DISABLE_TX(phi2s);

            pI2Sn->sInfo.sStatus.u8TxBusy = 0U;
            return ARM_DRIVER_OK;

        case ARM_SAI_ABORT_RECEIVE:
            if (pI2Sn->spdma.i32RxChnId != -1)
            {
                SPII2S_DISABLE_RXDMA(phi2s);
            }

            SPII2S_DISABLE_RX(phi2s);

            pI2Sn->sInfo.sStatus.u8RxBusy = 0U;
            return ARM_DRIVER_OK;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    // --------- Step 2: Parse mode and protocol ---------
    u32Mode = (control & ARM_SAI_MODE_MASTER) ? SPII2S_MODE_MASTER : SPII2S_MODE_SLAVE;

    // Parse supported protocols (I2S / MSB / LSB / PCM)
    switch (control & ARM_SAI_PROTOCOL_Msk)
    {
        case ARM_SAI_PROTOCOL_I2S:
        case ARM_SAI_PROTOCOL_LSB_JUSTIFIED:
            u32Protocol = SPII2S_FORMAT_I2S;
            break;

        case ARM_SAI_PROTOCOL_MSB_JUSTIFIED:
            u32Protocol = SPII2S_FORMAT_MSB;
            break;

        case ARM_SAI_PROTOCOL_PCM_SHORT:
        case ARM_SAI_PROTOCOL_PCM_LONG:
            u32Protocol = SPII2S_FORMAT_PCMA;
            break;

        default:
            return ARM_SAI_ERROR_PROTOCOL;
    }

    // --------- Step 3: Parse data width and channel settings ---------
    u32DataBits = ((control & ARM_SAI_DATA_SIZE_Msk) >> ARM_SAI_DATA_SIZE_Pos) + 1;
    // Map data bits to hardware word width
    u32DataBits = (u32DataBits <= 8)    ? SPII2S_DATABIT_8
                  : (u32DataBits <= 16) ? SPII2S_DATABIT_16
                  : SPII2S_DATABIT_32;

    //u32LSBFirst = (((control & ARM_SAI_BIT_ORDER_Msk) >> ARM_SAI_BIT_ORDER_Pos) ? I2S_ORDER_AT_LSB : I2S_ORDER_AT_MSB);

    u32MonoMode = (control & ARM_SAI_MONO_MODE) ? SPII2S_MONO : SPII2S_STEREO;

    // --------- Step 4: Check unsupported features ---------
    //if (control & ARM_SAI_COMPANDING_Msk) return ARM_SAI_ERROR_COMPANDING;

    //if (control & ARM_SAI_CLOCK_POLARITY_Msk) return ARM_SAI_ERROR_CLOCK_POLARITY;

    // --------- Step 5: Handle MCLK pin control ---------
    switch (control & ARM_SAI_MCLK_PIN_Msk)
    {
        case ARM_SAI_MCLK_PIN_INACTIVE:
            phi2s->I2SCTL &= ~SPI_I2SCTL_MCLKEN_Msk; // Disable MCLK output
            break;

        case ARM_SAI_MCLK_PIN_OUTPUT:
            /* Enable MCLK output */
            phi2s->I2SCTL = ((phi2s->I2SCTL & ~(SPI_I2SCTL_MCLKEN_Msk)) | SPI_I2SCTL_MCLKEN_Msk);
            break;

        case ARM_SAI_MCLK_PIN_INPUT:
        default:
            return ARM_SAI_ERROR_MCLK_PIN;
    }

    // --------- Step 6: Parse sampling rate and MCLK prescaler ---------
    u32SampleRate = (arg2 & ARM_SAI_AUDIO_FREQ_Msk);
    u32MclkPrescaler = ((arg2 & ARM_SAI_MCLK_PRESCALER_Msk) >> ARM_SAI_MCLK_PRESCALER_Pos) + 1;

    // --------- Step 7: Parse TDM slot settings ---------
    //u32SlotCount = ((arg1 & ARM_SAI_SLOT_COUNT_Msk) >> ARM_SAI_SLOT_COUNT_Pos) + 1;
    //u32SlotSize = ((arg1 & ARM_SAI_SLOT_SIZE_Msk) == ARM_SAI_SLOT_SIZE_16) ? I2S_TDM_WIDTH_16BIT :
    //              ((arg1 & ARM_SAI_SLOT_SIZE_Msk) == ARM_SAI_SLOT_SIZE_32) ? I2S_TDM_WIDTH_32BIT :
    //              I2S_TDM_WIDTH_8BIT;

    // --------- Step 8: Re-configure I2S with new settings ---------
    if (SPII2S_Open(phi2s, u32Mode, u32SampleRate, u32DataBits, u32MonoMode, u32Protocol) < 0)
        return ARM_DRIVER_ERROR;

    // --------- Step 9: Enable TDM if slots > 2 ---------
    //if (u32SlotCount >= 2)
    //{
    //    u32SlotCount = (u32SlotCount <= 2) ? I2S_TDM_2CH :
    //                   (u32SlotCount <= 4) ? I2S_TDM_4CH :
    //                   (u32SlotCount <= 6) ? I2S_TDM_6CH :
    //                   I2S_TDM_8CH;
    //    I2S_ConfigureTDM(phi2s, u32SlotSize, u32SlotCount, I2S_TDM_SYNC_ONE_BCLK);
    //}

    // --------- Step 10: Configure MCLK divider if needed ---------
    if (u32MclkPrescaler >= 1)
    {
        /* Write u32Divider to MCLKDIV (SPI_I2SCLK[5:0]) */
        phi2s->I2SCLK = (phi2s->I2SCLK & ~SPI_I2SCLK_MCLKDIV_Msk) | ((u32MclkPrescaler >> 1) << SPI_I2SCLK_MCLKDIV_Pos);
        phi2s->I2SCTL = ((phi2s->I2SCTL & ~(SPI_I2SCTL_MCLKEN_Msk)) | SPI_I2SCTL_MCLKEN_Msk);
    }

    // --------- Step 11: Mark as configured ---------
    pI2Sn->sInfo.u8Flags |= SAI_FLAG_CONFIGURED;

    return ARM_DRIVER_OK;
}

/**
  \fn          ARM_SAI_STATUS SAIn_GetStatus (uint32_t u32Inst)
  \brief       Get I2S status.
  \param[in]   i2s       Pointer to I2S resources
  \return      SAI status \ref ARM_SAI_STATUS
*/
static ARM_SAI_STATUS SAIn_GetStatus(uint32_t u32Inst)
{
    I2S_RESOURCES *pI2Sn = (I2S_RESOURCES *)i2s_res_list[SAI_TO_SPI_INSTANCE(u32Inst)];
    ARM_SAI_STATUS status;

    status.frame_error   = pI2Sn->sInfo.sStatus.u8FrameError;
    status.rx_busy       = pI2Sn->sInfo.sStatus.u8RxBusy;
    status.rx_overflow   = pI2Sn->sInfo.sStatus.u8RxOverflow;
    status.tx_busy       = pI2Sn->sInfo.sStatus.u8TxBusy;
    status.tx_underflow  = pI2Sn->sInfo.sStatus.u8TxUnderflow;

    return status;
}

static void SAI_PDMA_TXInit(I2S_RESOURCES *pI2Sn)
{
    SPI_T *phi2s = (SPI_T *)pI2Sn->phi2s;
    uint32_t u32DataBits = ((((phi2s->I2SCTL & SPI_I2SCTL_WDWIDTH_Msk) >> SPI_I2SCTL_WDWIDTH_Pos) + 1) * 8);

    //Use PDMA transfer mode
    struct nu_pdma_chn_cb pdma_chn_cb;

    pdma_chn_cb.m_eCBType = eCBType_Event;
    pdma_chn_cb.m_pfnCBHandler = SAI_PDMA_TX_CB;
    pdma_chn_cb.m_pvUserData = (void *)pI2Sn;

    //Use PDMA transfer mode
    nu_pdma_filtering_set(pI2Sn->spdma.i32TxChnId, NU_PDMA_EVENT_TRANSFER_DONE);
    nu_pdma_callback_register(pI2Sn->spdma.i32TxChnId, &pdma_chn_cb);
    nu_pdma_transfer(
        pI2Sn->spdma.i32TxChnId,
        u32DataBits,
        (uint32_t)pI2Sn->sInfo.sTx.pu8Buf,
        (uint32_t)&phi2s->TX,
        pI2Sn->sInfo.sTx.u32Num,
        0);
}

static void SAI_PDMA_RXInit(I2S_RESOURCES *pI2Sn)
{
    SPI_T *phi2s = (SPI_T *)pI2Sn->phi2s;
    uint32_t u32DataBits = ((((phi2s->I2SCTL & SPI_I2SCTL_WDWIDTH_Msk) >> SPI_I2SCTL_WDWIDTH_Pos) + 1) * 8);

    //Use PDMA transfer mode
    struct nu_pdma_chn_cb pdma_chn_cb;

    pdma_chn_cb.m_eCBType = eCBType_Event;
    pdma_chn_cb.m_pfnCBHandler = SAI_PDMA_RX_CB;
    pdma_chn_cb.m_pvUserData = (void *)pI2Sn;

    //Use PDMA transfer mode
    nu_pdma_filtering_set(pI2Sn->spdma.i32RxChnId, NU_PDMA_EVENT_TRANSFER_DONE);
    nu_pdma_callback_register(pI2Sn->spdma.i32RxChnId, &pdma_chn_cb);
    nu_pdma_transfer(
        pI2Sn->spdma.i32RxChnId,
        u32DataBits,
        (uint32_t)&phi2s->RX,
        (uint32_t)pI2Sn->sInfo.sRx.pu8Buf,
        pI2Sn->sInfo.sRx.u32Num,
        0);
}

// Local driver functions definitions (for instances)
// Information definitions (for instances)
#if (RTE_SAI_SPII2S0 == 1)
FUNCS_DEFINE(SAI_SPII2S_IDX0)
I2S_DRIVER(SAI_SPII2S_IDX0)

NVT_ITCM void SPI0_IRQHandler(void)
{
    SAI_IRQHandler(SAI_SPII2S_IDX0);
}
#endif

#if (RTE_SAI_SPII2S1 == 1)
FUNCS_DEFINE(SAI_SPII2S_IDX1)
I2S_DRIVER(SAI_SPII2S_IDX1)

NVT_ITCM void SPI1_IRQHandler(void)
{
    SAI_IRQHandler(SAI_SPII2S_IDX1);
}
#endif

#if (RTE_SAI_SPII2S2 == 1)
FUNCS_DEFINE(SAI_SPII2S_IDX2)
I2S_DRIVER(SAI_SPII2S_IDX2)

NVT_ITCM void SPI2_IRQHandler(void)
{
    SAI_IRQHandler(SAI_SPII2S_IDX2);
}
#endif

#if (RTE_SAI_SPII2S3 == 1)
FUNCS_DEFINE(SAI_SPII2S_IDX3)
I2S_DRIVER(SAI_SPII2S_IDX3)

NVT_ITCM void SPI3_IRQHandler(void)
{
    SAI_IRQHandler(SAI_SPII2S_IDX3);
}
#endif

#endif //DRIVER_CONFIG_VALID
