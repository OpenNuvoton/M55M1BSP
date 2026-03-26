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

/* Driver_SPI_SPIM.c - CMSIS-Driver for Nuvoton SPIM */

#include <stdint.h>
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
#define RTE_SPI_SPIM0                   RTE_SPI8
//#define RTE_SPI_SPIM1                   RTE_SPIx

// Configuration depending on RTE_SPI.h
// Check if at least one peripheral instance is configured in RTE_SPI.h
//#if (!(RTE_SPI8) && !(RTE_SPI1) && !(RTE_SPI2) && !(RTE_SPI3) && !(RTE_SPI4) && !(RTE_SPI5) && !(RTE_SPI6) && !(RTE_SPI7) && !(RTE_SPI8))
#if (!(RTE_SPI_SPIM0) && !(RTE_SPI_SPIM1))
    //#warning  SPI driver requires at least one SPI peripheral configured in RTE_SPI.h
#else
    #define DRIVER_CONFIG_VALID     1
#endif

// *****************************************************************************

#ifdef DRIVER_CONFIG_VALID     // Driver code is available only if configuration is valid

#if (RTE_SPI_SPIM0 == 1)
    #define SPI_SPIM_IDX0              8
    #define SPI_SPIM_PORT0             0
    #define SPIM_PORTSYM_FROM_RTE_8    SPI_SPIM_PORT0
#endif

#if (RTE_SPI_SPIM1 == 1)
    #define SPI_SPIM_IDX1              8
    #define SPI_SPIM_PORT1             1
    #define SPIM_PORTSYM_FROM_RTE_8    SPI_SPIM_PORT1
#endif

static inline uint32_t spi_idx_from_port_rt(uint32_t n)
{
    switch (n)
    {
#if (RTE_SPI_SPIM0 == 1)

        case SPI_SPIM_IDX0:
            return SPI_SPIM_PORT0;
#endif

#if (RTE_SPI_SPIM1 == 1)

        case SPI_SPIM_IDX1:
            return SPI_SPIM_PORT1;
#endif

        default:
            return 0;
    }
}

#define SPIM_IDX_FROM_PORT_CAT(x)        SPIM_PORTSYM_FROM_RTE_##x
#define SPIM_IDX_FROM_PORT(x)            SPIM_IDX_FROM_PORT_CAT(x)

#define SPI_TO_SPIM_INSTANCE(n)          ( spi_idx_from_port_rt((uint32_t)(n)) )
#define SPI_TO_SPIM(n)                   ( SPI_CONCAT2(SPIM, SPIM_IDX_FROM_PORT(n)) )

#define SPIM_TRX_MAX_SIZE                   0x2

// Local driver functions declarations (for instances)
#define SPI_INFO_DEFINE(n)                                                     \
    static SPI_RESOURCES SPI_RES_NAME(n) = { SPI_TO_SPIM(n),                   \
                                             {\
                                              0,                               \
                                              0,                               \
                                             },                                \
                                             {0},                              \
                                             {0},                              \
                                             {-1,                              \
                                              -1,                              \
                                              0,                               \
                                              0,                               \
                                              0,                               \
                                              0,                               \
                                              0,                               \
                                              0,                               \
                                              0,                               \
                                              0,                               \
                                             }                                 \
                                           };

// Local driver functions declarations (for instances)
#if (RTE_SPI_SPIM0 == 1)
    SPI_INFO_DEFINE(SPI_SPIM_IDX0)
#endif

#if (RTE_SPI_SPIM1 == 1)
    SPI_INFO_DEFINE(SPI_SPIM_IDX1)
#endif

// List of available SPI instance infos
static const SPI_RESOURCES *const spi_res_list[] =
{
#if defined(RTE_SPI_SPIM0) && (RTE_SPI_SPIM0 == 1)
    &SPI_RES_NAME(SPI_SPIM_IDX0),
#else
    NULL,
#endif

#if defined(RTE_SPI_SPIM1) && (RTE_SPI_SPIM1 == 1)
    &SPI_RES_NAME(SPI_SPIM_IDX1),
#else
    NULL,
#endif

    NULL,
};

static volatile uint8_t gu8INTDone = SPIM_OP_DISABLE;

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

void SPI_IO_RWDataByPhase(SPI_RESOURCES *pSPIn, uint32_t u32OPMode, uint8_t *pu8TRxBuf, uint32_t u32TRxSize, uint32_t u32DataPhase, uint32_t u32DTREn);
static void SPIM_Close(SPIM_T *spim);

#if (RTE_SPI_SPIM0 == 1)
    FUNCS_DECLARE(SPI_SPIM_IDX0)
#endif

#if (RTE_SPI_SPIM1 == 1)
    FUNCS_DECLARE(SPI_SPIM_IDX1)
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
// SPI interrupt handler
static void SPI_IRQHandler(uint32_t u32Inst)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_SPIM_INSTANCE(u32Inst)];
    SPIM_T *phspi = (SPIM_T *)pSPIn->phspi;

    if (SPIM_IS_IF_ON(phspi))
    {
        phspi->CTL0 |= SPIM_CTL0_IF_Msk;

        if ((pSPIn->sXfer.u32RxCnt >= pSPIn->sXfer.u32Num) ||
                (pSPIn->sXfer.u32TxCnt >= pSPIn->sXfer.u32Num))
        {
            gu8INTDone = SPIM_OP_ENABLE;
            pSPIn->sState.sDrvStatus.u8Busy = 0;

            if (pSPIn->sState.cb_event)
            {
                pSPIn->sState.cb_event(ARM_SPI_EVENT_TRANSFER_COMPLETE);
            }
        }
    }
}

// Configure SPI interrupt
static void SPI_InterruptConfig(uint32_t u32Inst, uint32_t u32IntEn)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_SPIM_INSTANCE(u32Inst)];
    SPIM_T *phspi = (SPIM_T *)pSPIn->phspi;
    IRQn_Type irq_n = SPIM0_IRQn; // Default SPI0 interrupt
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    irq_n = (phspi == SPIM0) ? SPIM0_IRQn : SPIM0_IRQn;

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
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_SPIM_INSTANCE(u32Inst)];

    if (pSPIn->sState.u8State & SPI_INITIALIZED) return ARM_DRIVER_OK;

    // Initialize SPI Run-Time Resources
    pSPIn->sState.cb_event = cb_event;
    pSPIn->sState.sDrvStatus.u8Busy = 0U;
    pSPIn->sState.sDrvStatus.u8DataLost = 0U;
    pSPIn->sState.sDrvStatus.u8ModeFault = 0U;

    // Clear transfer information
    memset(&pSPIn->sXfer, 0, sizeof(SPI_TRANSFER_INFO));

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
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_SPIM_INSTANCE(u32Inst)];
    SPIM_T *phspi = (SPIM_T *)pSPIn->phspi;

    SPI_InterruptConfig(u32Inst, SPI_OP_DISABLE); // Disable SPI interrupts

    SPIM_Close(phspi);

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
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_SPIM_INSTANCE(u32Inst)];
    SPIM_T *phspi = (SPIM_T *)pSPIn->phspi;

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
            SPI_InterruptConfig(u32Inst, SPI_OP_DISABLE); // Disable SPI interrupts

            // Reset SPI Run-Time Resources
            pSPIn->sState.sDrvStatus.u8Busy = 0U;
            pSPIn->sState.sDrvStatus.u8DataLost = 0U;
            pSPIn->sState.sDrvStatus.u8ModeFault = 0U;

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

            // Reset SPI Run-Time Resources
            pSPIn->sState.sDrvStatus.u8Busy = 0U;
            pSPIn->sState.sDrvStatus.u8DataLost = 0U;
            pSPIn->sState.sDrvStatus.u8ModeFault = 0U;

            SPI_InterruptConfig(u32Inst, SPI_OP_ENABLE); // Enable SPI interrupts

            /* Enable SPI Flash Mode */
            SPIM_SET_FLASH_MODE(phspi);

            SPIM_SET_CLOCK_DIVIDER(phspi, 8);

            /* Disable Cipher */
            SPIM_DISABLE_CIPHER(phspi);

            /* Enable DLL */
            SPIM_INIT_DLL(phspi);

            SPIM_SET_SS_ACTLVL(phspi, SPIM_OP_DISABLE);

            /* Set SPIM clock as HCLK divided by 1 */
            //SPIM_SET_CLOCK_DIVIDER(phspi, 8);

            //SPIM_DISABLE_CIPHER(phspi);

            //SPIM_InitFlash(phspi, 0);

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
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_SPIM_INSTANCE(u32Inst)];
    SPIM_T *phspi = (SPIM_T *)pSPIn->phspi;
    /* 0x02h : CMD_NORMAL_PAGE_PROGRAM Command Phase Table */
    static SPIM_PHASE_T sWrCMD =
    {
        CMD_NORMAL_PAGE_PROGRAM,                                    //Command Code
        PHASE_NORMAL_MODE, PHASE_WIDTH_0,  PHASE_DISABLE_DTR,       //Command Phase
        PHASE_NORMAL_MODE, PHASE_WIDTH_0, PHASE_DISABLE_DTR,       //Address Phase
        PHASE_NORMAL_MODE, PHASE_ORDER_MODE0,  PHASE_DISABLE_DTR, SPIM_OP_DISABLE,  //Data Phase
        0,
        PHASE_DISABLE_CONT_READ, 0, 0, 0
    };

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

    gu8INTDone = SPIM_OP_DISABLE;
    SPIM_ENABLE_INT(phspi);

    // Check if PDMA channel is available for TX
    if (pSPIn->spdma.u32TxUsed == SPI_OP_ENABLE)
    {
        SPIM_DMADMM_InitPhase(phspi, &sWrCMD, SPIM_CTL0_OPMODE_PAGEWRITE);

        SPIM_SET_OPMODE(phspi, SPIM_CTL0_OPMODE_PAGEWRITE); /* Switch to Page Read mode.      */

        phspi->SRAMADDR = (uint32_t) data;        /* SRAM u32Address. */
        phspi->DMACNT = num;                   /* Transfer length. */
        //phspi->FADDR = 0;

        SPIM_SET_GO(phspi);
    }
    else
    {
        SPI_IO_RWDataByPhase(pSPIn, SPIM_IO_WRITE_PHASE, pSPIn->sXfer.pu8TxBuf, pSPIn->sXfer.u32Num,
                             PHASE_NORMAL_MODE, SPIM_OP_DISABLE);
    }

    //while (gu8INTDone != SPIM_OP_ENABLE) {}

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
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_SPIM_INSTANCE(u32Inst)];
    SPIM_T *phspi = (SPIM_T *)pSPIn->phspi;

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

    gu8INTDone = SPIM_OP_DISABLE;
    SPIM_ENABLE_INT(phspi);

    if (pSPIn->spdma.u32RxUsed == SPI_OP_ENABLE)
    {
        SPIM_SET_OPMODE(phspi, SPIM_CTL0_OPMODE_PAGEREAD); /* Switch to Page Read mode.      */

        phspi->SRAMADDR = (uint32_t) data;        /* SRAM u32Address. */
        phspi->DMACNT = num;                       /* Transfer length. */
        phspi->FADDR = 0;
        SPIM_SET_GO(phspi);
    }
    else
    {
        SPI_IO_RWDataByPhase(pSPIn, SPIM_IO_READ_PHASE, pSPIn->sXfer.pu8RxBuf, pSPIn->sXfer.u32Num,
                             PHASE_NORMAL_MODE, SPIM_OP_DISABLE);
    }

    //while (gu8INTDone != SPIM_OP_ENABLE) {}

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
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_SPIM_INSTANCE(u32Inst)];
    SPIM_T *phspi = (SPIM_T *)pSPIn->phspi;

    return ARM_DRIVER_ERROR_UNSUPPORTED;

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

    SPIM_ENABLE_INT(phspi);

    // Initialize and trigger PDMA if available
    if (pSPIn->spdma.i32RxChnId != -1)
    {
        SPIM_SET_OPMODE(phspi, SPIM_CTL0_OPMODE_PAGEWRITE); /* Switch to Page Read mode.      */

        phspi->SRAMADDR = (uint32_t) data_in;        /* SRAM u32Address. */
        phspi->DMACNT = num;                       /* Transfer length. */
        phspi->FADDR = 0;
    }

    SPIM_SET_GO(phspi);

    return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t SPI_GetDataCount (void)
  \brief       Get transferred data count.
  \return      number of data items transferred
*/
static uint32_t SPIn_GetDataCount(uint32_t u32Inst)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_SPIM_INSTANCE(u32Inst)];

    // Check if the SPI is configured
    if (!(pSPIn->sState.u8State & SPI_CONFIGURED))
    {
        return 0U;
    }

    // If Rx buffer is used → Return RxCnt
    if (pSPIn->sXfer.pu8RxBuf != NULL)
    {
        return pSPIn->sXfer.u32RxCnt;
    }
    else // Else fallback to TxCnt
    {
        return pSPIn->sXfer.u32TxCnt;
    }

    return pSPIn->sXfer.u32RxCnt;
}

static int32_t SPIn_Control(uint32_t u32Inst, uint32_t control, uint32_t arg)
{
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_SPIM_INSTANCE(u32Inst)];
    SPIM_T *phspi = (SPIM_T *)pSPIn->phspi;
    uint32_t u32DataBits;
    uint32_t u32Div = 0;

    if (!(pSPIn->sState.u8State & SPI_POWERED))
    {
        return ARM_DRIVER_ERROR;
    }

    // Check if the control command is to abort the transfer
    if ((control & ARM_SPI_CONTROL_Msk) == ARM_SPI_ABORT_TRANSFER)
    {
        // Terminate the TX DMA channel if it is in use
        if (pSPIn->spdma.u32TxUsed == SPI_OP_ENABLE)
            SPIM_SET_OPMODE(phspi, SPIM_CTL0_OPMODE_IO);

        // Terminate the RX DMA channel if it is in use
        if (pSPIn->spdma.u32RxUsed == SPI_OP_ENABLE)
            SPIM_SET_OPMODE(phspi, SPIM_CTL0_OPMODE_IO);

        SPIM_DISABLE_INT(phspi);

        // Clear the transfer information
        memset(&pSPIn->sXfer, 0, sizeof(SPI_TRANSFER_INFO));

        // Set the busy status to 0 (not busy)
        pSPIn->sState.sDrvStatus.u8Busy = 0U;

        return ARM_DRIVER_OK;
    }

    // Process according to the operation command
    switch (control & ARM_SPI_CONTROL_Msk)
    {
        case ARM_SPI_MODE_INACTIVE:
            pSPIn->sState.u32Mode &= ~ARM_SPI_CONTROL_Msk;
            pSPIn->sState.u32Mode |= ARM_SPI_MODE_INACTIVE;

            SPIM_SET_SS_EN(phspi, SPI_OP_DISABLE);

            pSPIn->sState.u8State &= ~SPI_CONFIGURED;
            return ARM_DRIVER_OK;

        case ARM_SPI_MODE_MASTER:
            SPIM_SET_OPMODE(phspi, SPIM_CTL0_OPMODE_IO);

            pSPIn->sState.u32Mode &= ~ARM_SPI_CONTROL_Msk;
            pSPIn->sState.u32Mode |= ARM_SPI_MODE_MASTER;
            goto set_speed;

        case ARM_SPI_SET_BUS_SPEED:
set_speed:
            u32Div = (((CLK_GetSCLKFreq() * 10U) / arg + 5U) / 10U) - 1U; /* Round to the nearest integer */

            if (u32Div > 0xFFFFU)
            {
                u32Div = 0xFFFFU;
                phspi->CTL1 |= SPIM_CTL1_DIVIDER_Msk;
                /* Return master peripheral clock rate */
                pSPIn->sConfig.u32BusSpeed = (CLK_GetSCLKFreq() / (0xFFFU + 1U));
            }
            else
            {
                phspi->CTL1 = (phspi->CTL1 & ~(SPIM_CTL1_DIVIDER_Msk)) |
                              (u32Div << SPIM_CTL1_DIVIDER_Pos);
                /* Return master peripheral clock rate */
                pSPIn->sConfig.u32BusSpeed = (CLK_GetSCLKFreq() / (u32Div + 1U));
            }

            //printf("SPIM clock = %d\n", pSPIn->sConfig.u32BusSpeed);

            //SPIM_SET_CLOCK_DIVIDER(phspi, clk_div);
            SPIM_SET_RXCLKDLY_RDDLYSEL(phspi, 0); // 預設延遲為0

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
                if (arg == ARM_SPI_SS_INACTIVE)
                    SPIM_SET_SS_EN(phspi, SPI_OP_DISABLE);
                else
                    SPIM_SET_SS_EN(phspi, SPI_OP_ENABLE);

                return ARM_DRIVER_OK;
            }

            if ((pSPIn->sState.u32Mode & ARM_SPI_CONTROL_Msk) == ARM_SPI_MODE_SLAVE &&
                    (pSPIn->sState.u32Mode & ARM_SPI_SS_SLAVE_MODE_Msk) == ARM_SPI_SS_SLAVE_SW)
            {
                if (arg == ARM_SPI_SS_INACTIVE)
                    SPIM_SET_SS_EN(phspi, SPI_OP_DISABLE);
                else
                    SPIM_SET_SS_EN(phspi, SPI_OP_ENABLE);

                return ARM_DRIVER_OK;
            }

            return ARM_DRIVER_ERROR;

        case ARM_SPI_MODE_SLAVE:
        default:
            return ARM_DRIVER_ERROR_PARAMETER;
    }

    // SS Master mode configuration
    if ((pSPIn->sState.u32Mode & ARM_SPI_CONTROL_Msk) == ARM_SPI_MODE_MASTER)
    {
        switch (control & ARM_SPI_SS_MASTER_MODE_Msk)
        {
            case ARM_SPI_SS_MASTER_UNUSED:
                pSPIn->sState.u32Mode &= ~ARM_SPI_SS_MASTER_MODE_Msk;
                pSPIn->sState.u32Mode |= ARM_SPI_SS_MASTER_UNUSED;
                break;

            case ARM_SPI_SS_MASTER_HW_INPUT:
                return ARM_SPI_ERROR_SS_MODE;

            case ARM_SPI_SS_MASTER_SW:
                pSPIn->sState.u32Mode &= ~ARM_SPI_SS_MASTER_MODE_Msk;
                pSPIn->sState.u32Mode |= ARM_SPI_SS_MASTER_SW;
                break;

            case ARM_SPI_SS_MASTER_HW_OUTPUT:
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
                pSPIn->sState.u32Mode &= ~ARM_SPI_SS_SLAVE_MODE_Msk;
                pSPIn->sState.u32Mode |= ARM_SPI_SS_SLAVE_HW;
                break;

            case ARM_SPI_SS_SLAVE_SW:
                if ((arg != ARM_SPI_SS_ACTIVE) && (arg != ARM_SPI_SS_INACTIVE))
                {
                    return ARM_DRIVER_ERROR_PARAMETER;
                }

                if (arg == ARM_SPI_SS_INACTIVE)
                    SPIM_SET_SS_EN(phspi, SPI_OP_DISABLE);
                else
                    SPIM_SET_SS_EN(phspi, SPI_OP_ENABLE);

                pSPIn->sState.u32Mode &= ~ARM_SPI_SS_SLAVE_MODE_Msk;
                pSPIn->sState.u32Mode |= ARM_SPI_SS_SLAVE_SW;
                break;
        }
    }

    // Frame Format
    //switch (control & ARM_SPI_FRAME_FORMAT_Msk)
    //{
    //    case ARM_SPI_CPOL0_CPHA0:
    //        phspi->CTL = (phspi->CTL & ~(SPI_CTL_CLKPOL_Msk | SPI_CTL_TXNEG_Msk | SPI_CTL_RXNEG_Msk)) | SPI_MODE_0;
    //        break;
    //    case ARM_SPI_CPOL0_CPHA1:
    //        phspi->CTL = (phspi->CTL & ~(SPI_CTL_CLKPOL_Msk | SPI_CTL_TXNEG_Msk | SPI_CTL_RXNEG_Msk)) | SPI_MODE_1;
    //        break;
    //    case ARM_SPI_CPOL1_CPHA0:
    //        phspi->CTL = (phspi->CTL & ~(SPI_CTL_CLKPOL_Msk | SPI_CTL_TXNEG_Msk | SPI_CTL_RXNEG_Msk)) | SPI_MODE_2;
    //        break;
    //    case ARM_SPI_CPOL1_CPHA1:
    //        phspi->CTL = (phspi->CTL & ~(SPI_CTL_CLKPOL_Msk | SPI_CTL_TXNEG_Msk | SPI_CTL_RXNEG_Msk)) | SPI_MODE_3;
    //        break;
    //    default:
    //        return ARM_SPI_ERROR_FRAME_FORMAT;
    //}

    // Data Bits
    u32DataBits = ((control & ARM_SPI_DATA_BITS_Msk) >> ARM_SPI_DATA_BITS_Pos);

    if ((u32DataBits != 8) && (u32DataBits != 16) && (u32DataBits != 24) && (u32DataBits != 32))
    {
        return ARM_SPI_ERROR_DATA_BITS;
    }

    SPIM_SET_DATA_WIDTH(phspi, u32DataBits);

    //phspi->CTL = (phspi->CTL & ~SPI_CTL_DWIDTH_Msk) |
    //             (((u32DataBits >= 32) ? 0 : u32DataBits) << SPI_CTL_DWIDTH_Pos);

    // MSB / LSB First
    //if ((control & ARM_SPI_BIT_ORDER_Msk) == ARM_SPI_LSB_MSB)
    //{
    //    SPI_SET_LSB_FIRST(phspi);
    //}
    //else
    //{
    //    SPI_SET_MSB_FIRST(phspi);
    //}

    //SPI_SetFIFO(phspi, (u32DataBits <= 16) ? 7 : 3, (u32DataBits <= 16) ? 7 : 3);

    // Clear RX & TX FIFOs
    //SPI_ClearRxFIFO(phspi);
    //SPI_ClearTxFIFO(phspi);

    // Enable SPI if needed
    //if (pSPIn->sState.u8State & SPI_POWERED)
    //{
    //    SPI_ENABLE(phspi);
    //}

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
    SPI_RESOURCES *pSPIn = (SPI_RESOURCES *)spi_res_list[SPI_TO_SPIM_INSTANCE(u32Inst)];
    ARM_SPI_STATUS status = {0};

    status.busy = pSPIn->sState.sDrvStatus.u8Busy;
    status.data_lost = pSPIn->sState.sDrvStatus.u8DataLost;
    status.mode_fault = pSPIn->sState.sDrvStatus.u8ModeFault;

    return (status);
}

/**
 * @brief      Write data to SPI slave.
 * @param      spim
 * @param      pu8TxBuf    Transmit buffer.
 * @param      u32NTx      Number of bytes to transmit.
 * @param      u32NBit     N-bit transmit/receive.
 *                         - \ref SPIM_BITMODE_1
 *                         - \ref SPIM_BITMODE_2
 *                         - \ref SPIM_BITMODE_4
 *                         - \ref SPIM_BITMODE_8
 * @return     SPIM_OK             SPIM write done.
 *             SPIM_ERR_TIMEOUT    SPIM operation abort due to timeout error.
 */
static int32_t SPI_WriteData(SPI_RESOURCES *pSPIn, uint8_t pu8TxBuf[], uint32_t u32NTx, uint32_t u32NBit)
{
    SPIM_T *phspi = (SPIM_T *)pSPIn->phspi;
    /* Write data to TX FIFO */
    uint32_t u32BufIdx = 0UL;   /* Transmit buffer index */
    uint32_t u32i;              /* Loop index */
    uint32_t u32Tmp;            /* Temporary variable */
    uint32_t u32ChunkSize;      /* Data chunk size */
    uint32_t u32BurstSize;      /* Burst data number */

    /* Switch between N-bit output mode */
    SPIM_SwitchNBitOutput(phspi, u32NBit);

    while (u32NTx)
    {
        /* Calculate the number of data to be transferred in one burst */
        u32ChunkSize = (u32NTx >= 16) ? 4UL :
                       (u32NTx >= 12) ? 3UL :
                       (u32NTx >= 8) ? 2UL :
                       (u32NTx >= 4) ? 1UL :
                       0UL;

        if (u32ChunkSize)
        {
            /* Transfer data in burst mode */
            u32i = u32ChunkSize;

            while (u32i)
            {
                memcpy(&u32Tmp, &pu8TxBuf[u32BufIdx], 4UL);
                u32i--;
                phspi->TX[u32i] = u32Tmp;
                u32BufIdx += 4UL;
                u32NTx -= 4UL;
            }

            u32BurstSize = u32ChunkSize;
            u32ChunkSize = 4UL;
        }
        else
        {
            u32ChunkSize = u32NTx;
            /* Transfer data in single mode */
            memcpy(&u32Tmp, &pu8TxBuf[u32BufIdx], u32NTx);
            u32BurstSize = 1UL;
            u32NTx = 0UL;
            phspi->TX[0] = u32Tmp;
        }

        pSPIn->sXfer.u32TxCnt += (((u32ChunkSize * 8UL) * u32BurstSize) / 8);
        //printf("TX cnt = %d, %d\r\n", pSPIn->sXfer.u32TxCnt, pSPIn->sXfer.u32Num);
        /* Switch to Normal mode */
        SPIM_SET_OPMODE(phspi, SPIM_CTL0_OPMODE_IO);
        /* Set data width */
        SPIM_SET_DATA_WIDTH(phspi, (u32ChunkSize * 8UL));
        /* Set burst data number */
        SPIM_SET_BURST_DATA(phspi, u32BurstSize);

        /* Wait until transfer complete */
        if (SPIM_WaitOpDone(phspi, SPIM_OP_ENABLE) != SPIM_OK)
        {
            return SPIM_ERR_TIMEOUT;
        }
    }

    return SPIM_OK;
}

/**
 * @brief      Read data from SPI slave.
 * @param      spim
 * @param      pu8RxBuf    Receive buffer.
 * @param      u32NRx      Size of receive buffer in bytes.
 * @param      u32NBit     N-bit transmit/receive.
 *                         - \ref SPIM_BITMODE_1
 *                         - \ref SPIM_BITMODE_2
 *                         - \ref SPIM_BITMODE_4
 *                         - \ref SPIM_BITMODE_8
 * @return     SPIM_OK          SPIM write done.
 *             SPIM_ERR_TIMEOUT SPIM operation abort due to timeout error.
 */
static int32_t SPI_ReadData(SPI_RESOURCES *pSPIn, uint8_t pu8RxBuf[], uint32_t u32NRx, uint32_t u32NBit)
{
    /*
     * Read data in burst mode to improve performance.
     */
    SPIM_T *phspi = (SPIM_T *)pSPIn->phspi;
    uint32_t u32BufIdx = 0UL;       /* Buffer index */
    uint32_t u32Tmp = 0;            /* Temporary variable for storing received data */
    uint32_t u32ChunkSize = 0;      /* Number of data in one burst */
    uint32_t u32TmpChunkSize = 0;   /* Temporary value for chunk_size */
    uint32_t u32BurstSize = 0;      /* Number of data in one burst */

    /* Configure SPIM to use N-bit input */
    SPIM_SwitchNBitInput(phspi, u32NBit);

    while (u32NRx)
    {
        /* Determine the number of data to be read in one burst */
        u32ChunkSize = (u32NRx >= 16) ? 4UL : /* At least 16 bytes */
                       (u32NRx >= 12) ? 3UL : /* 12 <= N < 16 */
                       (u32NRx >= 8) ? 2UL : /* 8 <= N < 12 */
                       (u32NRx >= 4) ? 1UL : /* 4 <= N < 8 */
                       0UL; /* N < 4 */

        u32TmpChunkSize = u32ChunkSize;

        if (u32ChunkSize)
        {
            /*
             * At least 2 data to be read in one burst, set burst size to
             * chunk_size.
             */
            u32BurstSize = u32ChunkSize;
            u32ChunkSize = 4UL;
        }
        else
        {
            /*
             * 1 data to be read, set burst size to 1 to read the data.
             */
            u32ChunkSize = u32NRx;
            u32BurstSize = 1UL;
        }

        pSPIn->sXfer.u32RxCnt += (((u32ChunkSize * 8UL) * u32BurstSize) / 8);
        //printf("RX cnt = %d\r\n", pSPIn->sXfer.u32RxCnt);
        /* Configure SPIM to use Normal mode, N-bit data width and burst size */
        SPIM_SET_OPMODE(phspi, SPIM_CTL0_OPMODE_IO);
        SPIM_SET_DATA_WIDTH(phspi, (u32ChunkSize * 8UL));
        SPIM_SET_BURST_DATA(phspi, u32BurstSize);

        /* Wait until transfer complete */
        if (SPIM_WaitOpDone(phspi, SPIM_OP_ENABLE) != SPIM_OK)
        {
            return SPIM_ERR_TIMEOUT;
        }

        u32ChunkSize = u32TmpChunkSize;

        /* Read received data */
        if (u32ChunkSize >= 2UL)
        {
            /* Read multiple data in one burst */
            while (u32ChunkSize)
            {
                u32Tmp = phspi->RX[u32ChunkSize - 1UL];
                memcpy(&pu8RxBuf[u32BufIdx], &u32Tmp, 4UL);
                u32ChunkSize--;
                u32BufIdx += 4UL;
                u32NRx -= 4UL;
            }
        }
        else
        {
            /* Read 1 data */
            u32Tmp = phspi->RX[0];
            memcpy(&pu8RxBuf[u32BufIdx], &u32Tmp, u32NRx);
            u32BufIdx += u32NRx;
            u32NRx = 0UL;
        }
    }

    return SPIM_OK;
}

/**
 * @brief Normal I/O mode data phase.
 * @param spim
 * @param u32OPMode     Normal I/O read or wirte mode.
 *                      - \ref SPIM_IO_WRITE_PHASE
 *                      - \ref SPIM_IO_READ_PHASE
 * @param pu8TRxBuf     Read/Write Data Buffer
 * @param u32TRxSize    Read/Write Data Size
 * @param u32DataPhase  Data Bit Mode
 *                      - \ref PHASE_NORMAL_MODE : Send data use standard mode.
 *                      - \ref PHASE_DUAL_MODE   : Send data use dual mode.
 *                      - \ref PHASE_QUAD_MODE   : Send data use quad mode.
 *                      - \ref PHASE_OCTAL_MODE  : Send data use octal mode.
 * @param u32DTREn      DTR mode
 *                      - \ref SPIM_OP_ENABLE
 *                      - \ref SPIM_OP_DISABLE
 * @param u32RdDQS      Receive data from SPI Flash when read DQS
 *                      - \ref SPIM_OP_ENABLE
 *                      - \ref SPIM_OP_DISABLE
 */
void SPI_IO_RWDataByPhase(SPI_RESOURCES *pSPIn, uint32_t u32OPMode, uint8_t *pu8TRxBuf,
                          uint32_t u32TRxSize, uint32_t u32DataPhase, uint32_t u32DTREn)
{
    SPIM_T *phspi = (SPIM_T *)pSPIn->phspi;

    /* DTR Activated. */
    SPIM_SET_DTR_MODE(phspi, u32DTREn);

    /* CS activated. */
    SPIM_SET_SS_EN(phspi, SPIM_OP_ENABLE);

    if (u32OPMode == SPIM_IO_WRITE_PHASE)
    {
        /* Write out data. */
        SPI_WriteData(pSPIn, pu8TRxBuf, u32TRxSize, SPIM_PhaseModeToNBit(u32DataPhase));
    }
    else
    {
        /* Read back data. */
        SPI_ReadData(pSPIn, pu8TRxBuf, u32TRxSize, SPIM_PhaseModeToNBit(u32DataPhase));
    }

    /* CS Deactivated. */
    SPIM_SET_SS_EN(phspi, SPIM_OP_DISABLE);

    /* DTR Deactivated. */
    SPIM_SET_DTR_MODE(phspi, SPIM_OP_DISABLE);
}

/**
  * @brief  Disable SPIM controller.
  * @param[in]  spim The pointer of the specified SPI module.
  * @details This function will reset SPIM controller.
  */
static void SPIM_Close(SPIM_T *spim)
{
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    /* Unlock protected registers */
    if (u32RegLockLevel)
    {
        SYS_UnlockReg();
    }

    /* Reset SPI */
    if (spim == SPIM0)
    {
        SYS_ResetModule(SYS_SPIM0RST);
    }

    if (u32RegLockLevel)
    {
        /* Lock protected registers */
        SYS_LockReg();
    }
}

// Local driver functions definitions (for instances)
// Information definitions (for instances)
#if (RTE_SPI_SPIM0 == 1)
FUNCS_DEFINE(SPI_SPIM_IDX0)
SPI_DRIVER(SPI_SPIM_IDX0)

NVT_ITCM void SPIM0_IRQHandler(void)
{
    SPI_IRQHandler(SPI_SPIM_IDX0);
}
#endif

#if (RTE_SPI_SPIM1 == 1)
FUNCS_DEFINE(SPI_SPIM_IDX1)
SPI_DRIVER(SPI_SPIM_IDX1)

NVT_ITCM void SPIM1_IRQHandler(void)
{
    SPI_IRQHandler(SPI_SPIM_IDX1);
}
#endif

#endif //DRIVER_CONFIG_VALID
