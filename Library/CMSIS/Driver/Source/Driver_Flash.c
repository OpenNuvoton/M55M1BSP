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

/**************************************************************************//**
 * @file     Driver_Flash.c
 * @version  V1.00
 * @brief    CMSIS Flash driver for Nuvoton M55M1
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*! \page Driver_Flash Flash

# Revision History

- Version 1.0
  - Initial release

# Requirements

This driver requires the M55M1 BSP.
The driver instance is mapped to hardware as shown in the table below:

  CMSIS Driver Instance | M55M1 Hardware Resource
  :---------------------|:-----------------------
  Driver_Flash0         | FMC

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

#include "Driver_Flash.h"
#include "NuMicro.h"
#include "drv_pdma.h"
#include <string.h>

#if ((RTE_FLASH_START_OFFSET + RTE_FLASH_BYTE_SIZE) > FMC_APROM_SIZE)
    #error "* Invalid Flash Region Configuration ! Please configure valid Flash Region in RTE_Device_Flash.h."
#endif

#define ARM_FLASH_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0) /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion =
{
    ARM_FLASH_API_VERSION,
    ARM_FLASH_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_FLASH_CAPABILITIES DriverCapabilities =
{
    1,  /* event_ready */
    2,  /* data_width = 0:8-bit, 1:16-bit, 2:32-bit */
    0,  /* erase_chip */
    0   /* reserved (must be zero) */
};

#ifndef DRIVER_FLASH_NUM
    #define DRIVER_FLASH_NUM    0           /* Default driver number */
#endif

/* Sector Information */
#define FLASH_SECTOR_INFO       NULL        /* (NULL=Uniform sectors) */

#define FLASH_SECTOR_COUNT      (RTE_FLASH_BYTE_SIZE / FMC_FLASH_PAGE_SIZE) /* Number of sectors */
#define FLASH_SECTOR_SIZE       FMC_FLASH_PAGE_SIZE                         /* FLASH_SECTORS information used */
#define FLASH_PAGE_SIZE         FMC_FLASH_PAGE_SIZE                         /* Programming page size in bytes */
#define FLASH_PROGRAM_UNIT      4                                           /* Smallest programmable unit in bytes */
#define FLASH_ERASED_VALUE      0xFF                                        /* Contents of erased memory */

/* Flash Information */
static ARM_FLASH_INFO FlashInfo =
{
    FLASH_SECTOR_INFO,
    FLASH_SECTOR_COUNT,
    FLASH_SECTOR_SIZE,
    FLASH_PAGE_SIZE,
    FLASH_PROGRAM_UNIT,
    FLASH_ERASED_VALUE,
    { 0, 0, 0 }     /* Reserved (must be zero) */
};

/* Flash Status */
static volatile ARM_FLASH_STATUS FlashStatus;

static struct
{
    ARM_Flash_SignalEvent_t m_cb_event;
    int32_t m_i32NuPDMA_Chan;
    struct nu_pdma_chn_cb m_pfnNuPDMA_ReadCB;
    volatile uint32_t *m_pu32DataPtr;
    volatile uint32_t m_u32ProcessBytes;
    uint32_t m_u32RequestBytes;
} s_sFlashContextInfo;

void ISP_IRQHandler(void)
{
    uint32_t u32FlashEvent = ARM_FLASH_EVENT_READY;

    if (FMC_GET_FAIL_FLAG())
    {
        FMC_CLR_FAIL_FLAG();
        u32FlashEvent = ARM_FLASH_EVENT_ERROR;
        FlashStatus.error = 1;
    }
    else
    {
        if (FMC->ISPCMD == FMC_ISPCMD_READ)
        {
            M32((uint32_t)s_sFlashContextInfo.m_pu32DataPtr + s_sFlashContextInfo.m_u32ProcessBytes) = FMC->ISPDAT;
        }

        s_sFlashContextInfo.m_u32ProcessBytes += FLASH_PROGRAM_UNIT;

        if (s_sFlashContextInfo.m_u32ProcessBytes < s_sFlashContextInfo.m_u32RequestBytes)
        {
            if (FMC->ISPCMD == FMC_ISPCMD_PROGRAM)
            {
                FMC->ISPDAT = M32((uint32_t)s_sFlashContextInfo.m_pu32DataPtr + s_sFlashContextInfo.m_u32ProcessBytes);
            }

            /* Clear INT flag and trigger next operation */
            FMC->ISPSTS  = FMC_ISPSTS_INTFLAG_Msk;
            FMC->ISPADDR = (FMC->ISPADDR + 4);
            FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;
            return ;
        }
    }

    FlashStatus.busy  = 0;

    if (s_sFlashContextInfo.m_cb_event != NULL)
    {
        s_sFlashContextInfo.m_cb_event(u32FlashEvent);
    }

    FMC->ISPSTS = FMC_ISPSTS_INTFLAG_Msk;
}

#if defined (RTE_FLASH_READ_PDMA) && (RTE_FLASH_READ_PDMA == 1)
static void Flash_Read_PDMA_CB_Handler(void *ptr_priv, uint32_t event)
{
    uint32_t u32FlashEvent = ARM_FLASH_EVENT_READY;

    NVT_UNUSED(ptr_priv);

    if (event & (NU_PDMA_EVENT_TRANSFER_DONE | NU_PDMA_EVENT_ABORT))
    {
        FlashStatus.busy = 0;

        if (event & (NU_PDMA_EVENT_ABORT))
        {
            FlashStatus.error = 1;
            u32FlashEvent = ARM_FLASH_EVENT_ERROR;
        }
        else
        {
#if (NVT_DCACHE_ON == 1)

            if ((s_sFlashContextInfo.m_pu32DataPtr != NULL) && (s_sFlashContextInfo.m_u32RequestBytes > 0))
            {
                SCB_InvalidateDCache_by_Addr((void *)s_sFlashContextInfo.m_pu32DataPtr, (int32_t)s_sFlashContextInfo.m_u32RequestBytes);
            }

#endif
        }

        if (s_sFlashContextInfo.m_cb_event != NULL)
        {
            s_sFlashContextInfo.m_cb_event(u32FlashEvent);
        }
    }
}
#endif  /* RTE_FLASH_READ_PDMA */

/**
  \fn          ARM_DRIVER_VERSION ARM_Flash_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION ARM_Flash_GetVersion(void)
{
    return DriverVersion;
}

/**
  \fn          ARM_FLASH_CAPABILITIES ARM_Flash_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_FLASH_CAPABILITIES
*/
static ARM_FLASH_CAPABILITIES ARM_Flash_GetCapabilities(void)
{
    return DriverCapabilities;
}


/**
  \fn          int32_t ARM_Flash_Initialize (ARM_Flash_SignalEvent_t cb_event)
  \brief       ARM_Flash_Initialize the Flash Interface.
  \param[in]   cb_event  Pointer to \ref ARM_Flash_SignalEvent
  \return      \ref execution_status
*/
static int32_t ARM_Flash_Initialize(ARM_Flash_SignalEvent_t cb_event)
{
    FlashStatus.busy  = 0;
    FlashStatus.error = 0;

    if (cb_event != NULL)
    {
        s_sFlashContextInfo.m_cb_event = cb_event;
    }

#if defined (RTE_FLASH_READ_PDMA) && (RTE_FLASH_READ_PDMA == 1)
    s_sFlashContextInfo.m_i32NuPDMA_Chan = -1;
    s_sFlashContextInfo.m_pfnNuPDMA_ReadCB.m_eCBType = eCBType_Event;
    s_sFlashContextInfo.m_pfnNuPDMA_ReadCB.m_pfnCBHandler = Flash_Read_PDMA_CB_Handler;
    s_sFlashContextInfo.m_pfnNuPDMA_ReadCB.m_pvUserData = NULL;
#endif

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_Flash_Uninitialize (void)
  \brief       De-initialize the Flash Interface.
  \return      \ref execution_status
*/
static int32_t ARM_Flash_Uninitialize(void)
{
    FlashStatus.busy  = 0;
    FlashStatus.error = 0;
    s_sFlashContextInfo.m_cb_event = NULL;

#if defined (RTE_FLASH_READ_PDMA) && (RTE_FLASH_READ_PDMA == 1)
    s_sFlashContextInfo.m_i32NuPDMA_Chan = -1;
#endif

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_Flash_PowerControl (ARM_POWER_STATE state)
  \brief       Control the Flash interface power.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t ARM_Flash_PowerControl(ARM_POWER_STATE state)
{
    uint32_t u32IsRegLocked = SYS_IsRegLocked();

    switch (state)
    {
        case ARM_POWER_OFF:
            if (u32IsRegLocked)
            {
                SYS_UnlockReg();
            }

            NVIC_DisableIRQ(ISP_IRQn);
            FMC_DisableINT();
            FMC_DISABLE_AP_UPDATE();
            FMC_Close();
            CLK_DisableModuleClock(ISP0_MODULE);

#if defined (RTE_FLASH_READ_PDMA) && (RTE_FLASH_READ_PDMA == 1)

            if (s_sFlashContextInfo.m_i32NuPDMA_Chan >= 0)
            {
                nu_pdma_channel_terminate(s_sFlashContextInfo.m_i32NuPDMA_Chan);
                nu_pdma_channel_free(s_sFlashContextInfo.m_i32NuPDMA_Chan);
                s_sFlashContextInfo.m_i32NuPDMA_Chan = -1;
            }

#endif
            break;

        case ARM_POWER_LOW:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_POWER_FULL:
            if (u32IsRegLocked)
            {
                SYS_UnlockReg();
            }

            CLK_EnableModuleClock(ISP0_MODULE);
            FMC_EnableINT();
            NVIC_EnableIRQ(ISP_IRQn);

#if defined (RTE_FLASH_READ_PDMA) && (RTE_FLASH_READ_PDMA == 1)

            if (RTE_FLASH_READ_PDMA_FIXED)
            {
                /* Allocate a fixed PDMA channel */
                s_sFlashContextInfo.m_i32NuPDMA_Chan = nu_pdma_channel_allocate(PDMA_MEM, RTE_FLASH_PDMA_PORT, RTE_FLASH_PDMA_CHANNEL);
            }
            else
            {
                /* Dynamically allocate a free PDMA channel */
                s_sFlashContextInfo.m_i32NuPDMA_Chan = nu_pdma_channel_dynamic_allocate(PDMA_MEM);
            }

            if (s_sFlashContextInfo.m_i32NuPDMA_Chan < 0)
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }

            /* Register PDMA callback function for allocated channel */
            nu_pdma_filtering_set(s_sFlashContextInfo.m_i32NuPDMA_Chan, NU_PDMA_EVENT_TRANSFER_DONE | NU_PDMA_EVENT_ABORT);
            nu_pdma_callback_register(s_sFlashContextInfo.m_i32NuPDMA_Chan, &s_sFlashContextInfo.m_pfnNuPDMA_ReadCB);
#endif
            FMC_Open();
            FMC_ENABLE_AP_UPDATE();
            break;

        default:
            return ARM_DRIVER_ERROR_PARAMETER;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_Flash_ReadData (uint32_t addr, void *data, uint32_t cnt)
  \brief       Read data from Flash.
  \param[in]   addr  Data address.
  \param[out]  data  Pointer to a buffer storing the data read from Flash.
  \param[in]   cnt   Number of data items to read.
  \return      number of data items read or \ref execution_status
*/
static int32_t ARM_Flash_ReadData(uint32_t addr, void *data, uint32_t cnt)
{
    if (FlashStatus.busy)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    if ((addr % FLASH_PROGRAM_UNIT) || ((cnt * FLASH_PROGRAM_UNIT) > RTE_FLASH_BYTE_SIZE) ||
            ((addr + (cnt * FLASH_PROGRAM_UNIT)) > RTE_FLASH_BYTE_SIZE))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    FlashStatus.busy  = 1;
    FlashStatus.error = 0;

    s_sFlashContextInfo.m_pu32DataPtr     = (uint32_t *)data;
    s_sFlashContextInfo.m_u32RequestBytes = cnt * FLASH_PROGRAM_UNIT;
    s_sFlashContextInfo.m_u32ProcessBytes = 0;

#if defined (RTE_FLASH_READ_PDMA) && (RTE_FLASH_READ_PDMA == 1)

    /* Try to use PDMA for read data transfer */
    if (cnt >= 8)
    {
        if (s_sFlashContextInfo.m_i32NuPDMA_Chan >= 0)
        {
            nu_pdma_transfer(s_sFlashContextInfo.m_i32NuPDMA_Chan, 32,
                             FMC_APROM_BASE + RTE_FLASH_START_OFFSET + addr,
                             (uint32_t)data,
                             cnt, 0);


            /* Non blocking mode: Return immediately, transfer in progress */
            return ARM_DRIVER_OK;
        }
    }

    /* Fall through to read data with FMC ISP command */
#endif

    FMC->ISPCMD  = FMC_ISPCMD_READ;
    FMC->ISPADDR = FMC_APROM_BASE + RTE_FLASH_START_OFFSET + addr;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_Flash_ProgramData (uint32_t addr, const void *data, uint32_t cnt)
  \brief       Program data to Flash.
  \param[in]   addr  Data address.
  \param[in]   data  Pointer to a buffer containing the data to be programmed to Flash.
  \param[in]   cnt   Number of data items to program.
  \return      number of data items programmed or \ref execution_status
*/
static int32_t ARM_Flash_ProgramData(uint32_t addr, const void *data, uint32_t cnt)
{
    if (FlashStatus.busy)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    if ((addr % FLASH_PROGRAM_UNIT) || ((cnt * FLASH_PROGRAM_UNIT) > RTE_FLASH_BYTE_SIZE) ||
            ((addr + (cnt * FLASH_PROGRAM_UNIT)) > RTE_FLASH_BYTE_SIZE))
        return ARM_DRIVER_ERROR_PARAMETER;

    FlashStatus.busy  = 1;
    FlashStatus.error = 0;

    s_sFlashContextInfo.m_pu32DataPtr     = (uint32_t *)data;
    s_sFlashContextInfo.m_u32RequestBytes = cnt * FLASH_PROGRAM_UNIT;
    s_sFlashContextInfo.m_u32ProcessBytes = 0;

    FMC->ISPCMD  = FMC_ISPCMD_PROGRAM;
    FMC->ISPADDR = FMC_APROM_BASE + RTE_FLASH_START_OFFSET + addr;
    FMC->ISPDAT  = *s_sFlashContextInfo.m_pu32DataPtr;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_Flash_EraseSector (uint32_t addr)
  \brief       Erase Flash Sector.
  \param[in]   addr  Sector address
  \return      \ref execution_status
*/
static int32_t ARM_Flash_EraseSector(uint32_t addr)
{
    if (FlashStatus.busy)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    if ((addr % FLASH_SECTOR_SIZE) || (addr >= RTE_FLASH_BYTE_SIZE))
        return ARM_DRIVER_ERROR_PARAMETER;

    FlashStatus.busy  = 1;
    FlashStatus.error = 0;

    s_sFlashContextInfo.m_pu32DataPtr     = NULL;
    s_sFlashContextInfo.m_u32RequestBytes = FLASH_PROGRAM_UNIT;
    s_sFlashContextInfo.m_u32ProcessBytes = 0;

    FMC->ISPCMD  = FMC_ISPCMD_PAGE_ERASE;
    FMC->ISPADDR = FMC_APROM_BASE + RTE_FLASH_START_OFFSET + addr;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_Flash_EraseChip (void)
  \brief       Erase complete Flash.
               Optional function for faster full chip erase.
  \return      \ref execution_status
*/
static int32_t ARM_Flash_EraseChip(void)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          ARM_FLASH_STATUS ARM_Flash_GetStatus (void)
  \brief       Get Flash status.
  \return      Flash status \ref ARM_FLASH_STATUS
*/
static ARM_FLASH_STATUS ARM_Flash_GetStatus(void)
{
    return FlashStatus;
}

/**
  \fn          ARM_FLASH_INFO * ARM_Flash_GetInfo (void)
  \brief       Get Flash information.
  \return      Pointer to Flash information \ref ARM_FLASH_INFO
*/
static ARM_FLASH_INFO *ARM_Flash_GetInfo(void)
{
    return &FlashInfo;
}


/* Flash Driver Control Block */
extern ARM_DRIVER_FLASH ARM_Driver_Flash_(DRIVER_FLASH_NUM);

ARM_DRIVER_FLASH ARM_Driver_Flash_(DRIVER_FLASH_NUM) =
{
    ARM_Flash_GetVersion,
    ARM_Flash_GetCapabilities,
    ARM_Flash_Initialize,
    ARM_Flash_Uninitialize,
    ARM_Flash_PowerControl,
    ARM_Flash_ReadData,
    ARM_Flash_ProgramData,
    ARM_Flash_EraseSector,
    ARM_Flash_EraseChip,
    ARM_Flash_GetStatus,
    ARM_Flash_GetInfo
};
