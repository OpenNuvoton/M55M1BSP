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

/* MCI_Nuvoton.c - CMSIS-Driver for Nuvoton SDH */

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
    #include "RTE_Device/RTE_Device.h"
#endif
#include "Driver_MCI.h"
#include "NuMicro.h"

//------------------------------------------------------------------------------
// Configuration depending on RTE_MCI.h
// Check if at least one peripheral instance is configured in RTE_MCI.h
#if (!(RTE_MCI0) && !(RTE_MCI1))
    #warning  SDH driver requires at least one SDH peripheral configured in RTE_MCI.h
#else
    #define DRIVER_CONFIG_VALID     1
#endif

// *****************************************************************************

#ifdef DRIVER_CONFIG_VALID     // Driver code is available only if configuration is valid

/* MCI Transfer Information Definition */
typedef struct _MCI_XFER
{
    uint8_t *pu8Buf;            /* Data buffer                        */
    uint32_t u32SectorAddr;     /* Data bytes to transfer             */
    uint32_t u32Count;          /* Data bytes to transfer             */
} MCI_XFER;

/* MCI Driver State Definition */
typedef struct MCI_Ctrl
{
    ARM_MCI_SignalEvent_t cb_event;     /* Driver event callback function     */
    ARM_MCI_STATUS volatile sDrvStatus; /* Driver status                      */
    uint8_t volatile u8Flags;           /* Driver state flags                 */
} MCI_CTRL;

typedef struct MCI_Resources
{
    SDH_T *phsdh;           /* SDH peripheral register interface    */
    SDH_INFO_T *pSD;        /* Pointer to SDH information structure */
    MCI_CTRL sCtrl;         /* Run-Time control structure           */
    MCI_XFER sXfer;         /* Run-Time transfer structure          */
    uint32_t u32CDPinEn;    /* Card Detect Pin Enable               */
} MCI_RESOURCES;

//------------------------------------------------------------------------------
#define SDH_OP_ENABLE               (1)
#define SDH_OP_DISABLE              (0)

/* Driver flag definitions */
#define MCI_INIT                    ((uint8_t)0x01)   /* MCI initialized            */
#define MCI_POWER                   ((uint8_t)0x02)   /* MCI powered on             */
#define MCI_SETUP                   ((uint8_t)0x04)   /* MCI configured             */
#define MCI_RESP_LONG               ((uint8_t)0x08)   /* Long response expected     */
#define MCI_CMD                     ((uint8_t)0x10)   /* Command response expected  */
#define MCI_DATA                    ((uint8_t)0x20)   /* Transfer response expected */

#define SECTOR_SIZE                 512

#define MCI_RESPONSE_EXPECTED_Msk   (ARM_MCI_RESPONSE_SHORT      | \
                                     ARM_MCI_RESPONSE_SHORT_BUSY | \
                                     ARM_MCI_RESPONSE_LONG)

// Local driver functions declarations (for instances)
#define MCI_INFO_DEFINE(n)                                                  \
    static MCI_RESOURCES mci##n##_res = { SDH##n,                           \
                                          &SD##n,                           \
                                          {0},                              \
                                          {0},                              \
                                          RTE_MCI##n##_CD_PIN_EN,           \
                                        };

// Macro for declaring functions (for instances)
#define FUNCS_DECLARE(n)                                                                    \
    static ARM_DRIVER_VERSION   MCI##n##_GetVersion      (void);                            \
    static ARM_MCI_CAPABILITIES MCI##n##_GetCapabilities (void);                            \
    static int32_t              MCI##n##_Initialize      (ARM_MCI_SignalEvent_t cb_event);  \
    static int32_t              MCI##n##_Uninitialize    (void);                            \
    static int32_t              MCI##n##_PowerControl    (ARM_POWER_STATE state);           \
    static int32_t              MCI##n##_CardPower       (uint32_t voltage);                \
    static int32_t              MCI##n##_ReadCD          (void);                            \
    static int32_t              MCI##n##_ReadWP          (void);                            \
    static int32_t              MCI##n##_SendCommand     (uint32_t cmd, uint32_t arg, uint32_t flags, uint32_t *response);            \
    static int32_t              MCI##n##_SetupTransfer   (uint8_t  *data, uint32_t block_count, uint32_t block_size, uint32_t mode);  \
    static int32_t              MCI##n##_AbortTransfer   (void);                            \
    static int32_t              MCI##n##_Control         (uint32_t control, uint32_t arg);  \
    static ARM_MCI_STATUS       MCI##n##_GetStatus       (void);                            \


// Macro for defining functions (for instances)
#define FUNCS_DEFINE(n)                                                                                                                             \
    static ARM_DRIVER_VERSION   MCI##n##_GetVersion      (void)                             { return MCIn_GetVersion(); }                           \
    static ARM_MCI_CAPABILITIES MCI##n##_GetCapabilities (void)                             { return MCIn_GetCapabilities(); }                      \
    static int32_t              MCI##n##_Initialize      (ARM_MCI_SignalEvent_t cb_event)   { return MCIn_Initialize(&mci##n##_res, cb_event); }    \
    static int32_t              MCI##n##_Uninitialize    (void)                             { return MCIn_Uninitialize(&mci##n##_res); }            \
    static int32_t              MCI##n##_PowerControl    (ARM_POWER_STATE state)            { return MCIn_PowerControl(&mci##n##_res, state); }     \
    static int32_t              MCI##n##_CardPower       (uint32_t voltage)                 { return MCIn_CardPower(&mci##n##_res, voltage); }      \
    static int32_t              MCI##n##_ReadCD          (void)                             { return MCIn_ReadCD(&mci##n##_res); }                  \
    static int32_t              MCI##n##_ReadWP          (void)                             { return MCIn_ReadWP(&mci##n##_res); }                  \
    static int32_t              MCI##n##_SendCommand     (uint32_t cmd, uint32_t arg, uint32_t flags, uint32_t *response) {return MCIn_SendCommand(&mci##n##_res, cmd, arg, flags, response);}                          \
    static int32_t              MCI##n##_SetupTransfer   (uint8_t  *data, uint32_t block_count, uint32_t block_size, uint32_t mode) {return MCIn_SetupTransfer(&mci##n##_res, data, block_count, block_size, mode);}    \
    static int32_t              MCI##n##_AbortTransfer   (void)                             { return MCIn_AbortTransfer(&mci##n##_res); }           \
    static int32_t              MCI##n##_Control         (uint32_t control, uint32_t arg)   { return MCIn_Control(&mci##n##_res, control, arg); }   \
    static ARM_MCI_STATUS       MCI##n##_GetStatus       (void)                             { return MCIn_GetStatus(&mci##n##_res); }               \

// Macro for defining driver structures (for instances)
#define MCI_DRIVER(n)                                           \
    ARM_DRIVER_MCI Driver_MCI##n = { MCI##n##_GetVersion,       \
                                     MCI##n##_GetCapabilities,  \
                                     MCI##n##_Initialize,       \
                                     MCI##n##_Uninitialize,     \
                                     MCI##n##_PowerControl,     \
                                     MCI##n##_CardPower,        \
                                     MCI##n##_ReadCD,           \
                                     MCI##n##_ReadWP,           \
                                     MCI##n##_SendCommand,      \
                                     MCI##n##_SetupTransfer,    \
                                     MCI##n##_AbortTransfer,    \
                                     MCI##n##_Control,          \
                                     MCI##n##_GetStatus,        \
                                   };

// Local driver functions declarations (for instances)
#if (RTE_MCI0 == 1)
    MCI_INFO_DEFINE(0)
#endif
#if (RTE_MCI1 == 1)
    MCI_INFO_DEFINE(1)
#endif

// List of available SDH instance infos
//static const MCI_RESOURCES *const mci_res_list[] =
//{
//#if (RTE_MCI0 == 1)
//    &mci0_res,
//#else
//    NULL,
//#endif // RTE_MCI0
//
//#if (RTE_MCI1 == 1)
//    &mci1_res,
//#else
//    NULL,
//#endif // RTE_MCI1
//
//    NULL,
//};

/* Local Functions */
static ARM_DRIVER_VERSION MCIn_GetVersion(void);
static ARM_MCI_CAPABILITIES MCIn_GetCapabilities(void);
static int32_t MCIn_Initialize(MCI_RESOURCES *pMCIn, ARM_MCI_SignalEvent_t cb_event);
static int32_t MCIn_Uninitialize(MCI_RESOURCES *pMCIn);
static int32_t MCIn_PowerControl(MCI_RESOURCES *pMCIn, ARM_POWER_STATE state);
static int32_t MCIn_CardPower(MCI_RESOURCES *pMCIn, uint32_t voltage);
static int32_t MCIn_ReadCD(MCI_RESOURCES *pMCIn);
static int32_t MCIn_ReadWP(MCI_RESOURCES *pMCIn);
static int32_t MCIn_SendCommand(MCI_RESOURCES *pMCIn, uint32_t cmd, uint32_t arg, uint32_t flags, uint32_t *response);
static int32_t MCIn_SetupTransfer(MCI_RESOURCES *pMCIn, uint8_t  *data, uint32_t block_count, uint32_t block_size, uint32_t mode);
static int32_t MCIn_AbortTransfer(MCI_RESOURCES *pMCIn);
static int32_t MCIn_Control(MCI_RESOURCES *pMCIn, uint32_t control, uint32_t arg);
static ARM_MCI_STATUS MCIn_GetStatus(MCI_RESOURCES *pMCIn);

static void SDH_InitCard(SDH_T *sdh, uint32_t u32CardDetect);
static int32_t SDH_SetClock(MCI_RESOURCES *pMCIn, uint32_t u32ClkEn);
static uint32_t SDH_Get_clock(SDH_T *sdh);

extern uint32_t SDH_SwitchToHighSpeed(SDH_T *sdh, SDH_INFO_T *pSD);

#if (RTE_MCI0 == 1)
    FUNCS_DECLARE(0)
#endif
#if (RTE_MCI1 == 1)
    FUNCS_DECLARE(1)
#endif

#define ARM_MCI_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(2, 8) /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion =
{
    ARM_MCI_API_VERSION,
    ARM_MCI_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_MCI_CAPABILITIES DriverCapabilities =
{
    1, /* cd_state          */
    1, /* cd_event          */
    0, /* wp_state          */
    1, /* vdd               */
    0, /* vdd_1v8           */
    0, /* vccq              */
    0, /* vccq_1v8          */
    0, /* vccq_1v2          */
    1, /* data_width_4      */
    0, /* data_width_8      */
    0, /* data_width_4_ddr  */
    0, /* data_width_8_ddr  */
    1, /* high_speed        */
    0, /* uhs_signaling     */
    0, /* uhs_tuning        */
    0, /* uhs_sdr50         */
    0, /* uhs_sdr104        */
    0, /* uhs_ddr50         */
    0, /* uhs_driver_type_a */
    0, /* uhs_driver_type_c */
    0, /* uhs_driver_type_d */
    1, /* sdio_interrupt    */
    0, /* read_wait         */
    0, /* suspend_resume    */
    0, /* mmc_interrupt     */
    0, /* mmc_boot          */
    0, /* rst_n             */
    0, /* ccs               */
    0, /* ccs_timeout       */
    0  /* Reserved          */
};

//
//   Functions
//
void SDH_IRQHandler(MCI_RESOURCES *pMCIn)
{
    unsigned int volatile isr;
    unsigned int volatile ier;
    uint32_t u32CDState = 0;
    //uint32_t u32Event = 0;

    // FMI data abort interrupt
    if (pMCIn->phsdh->GINTSTS & SDH_GINTSTS_DTAIF_Msk)
    {
        /* ResetAllEngine() */
        pMCIn->phsdh->GCTL |= SDH_GCTL_GCTLRST_Msk;

        pMCIn->sCtrl.sDrvStatus.transfer_error = 1U;
    }

    //----- SD interrupt status
    isr = pMCIn->phsdh->INTSTS;
    ier = pMCIn->phsdh->INTEN;

    if (isr & SDH_INTSTS_BLKDIF_Msk)
    {
        // block down
        pMCIn->pSD->DataReadyFlag = TRUE;
        pMCIn->phsdh->INTSTS = SDH_INTSTS_BLKDIF_Msk;
        //printf("SD block down\r\n");
    }

    if ((ier & SDH_INTEN_CDIEN_Msk) &&
            (isr & SDH_INTSTS_CDIF_Msk))    // card detect
    {
        //----- SD interrupt status
        // delay 50 us to sync the GPIO and SDH
        {
            (void)pMCIn->phsdh->INTSTS;

            CLK_SysTickDelay(50);

            isr = pMCIn->phsdh->INTSTS;
        }

        u32CDState = (pMCIn->u32CDPinEn == SDH_OP_DISABLE) ? (!(isr & SDH_INTSTS_CDSTS_Msk)) :
                     (isr & SDH_INTSTS_CDSTS_Msk);

        if (u32CDState)
        {
            //printf("\n***** card remove !\n");
            pMCIn->pSD->IsCardInsert = FALSE;   // SDISR_CD_Card = 1 means card remove for GPIO mode
            memset(pMCIn->pSD, 0, sizeof(SDH_INFO_T));
        }
        else
        {
            //printf("***** card insert !\n");
            //SDH_Open(SDH0, CardDetect_From_GPIO);
            //SDH_Probe(SDH0);
        }

        pMCIn->phsdh->INTSTS = SDH_INTSTS_CDIF_Msk;
    }

    // CRC error interrupt
    if (isr & SDH_INTSTS_CRCIF_Msk)
    {
        if (!(isr & SDH_INTSTS_CRC16_Msk))
        {
            // printf("***** ISR sdioIntHandler(): CRC_16 error !\n");
            //  handle CRC error

            /* Command response timeout or CRC error*/
            //pMCIn->sCtrl.sDrvStatus.command_timeout = 1U;
        }
        else if (!(isr & SDH_INTSTS_CRC7_Msk))
        {
            if (!pMCIn->pSD->R3Flag)
            {
                // printf("***** ISR sdioIntHandler(): CRC_7 error !\n");
                //  handle CRC error

                /* Command response timeout or CRC error*/
                //pMCIn->sCtrl.sDrvStatus.command_timeout = 1U;
            }
        }

        pMCIn->phsdh->INTSTS = SDH_INTSTS_CRCIF_Msk;      // clear interrupt flag
    }

    if (isr & SDH_INTSTS_DITOIF_Msk)
    {
        printf("***** ISR: data in timeout !\n");
        pMCIn->phsdh->INTSTS |= SDH_INTSTS_DITOIF_Msk;

        /* Data timeout */
        //pMCIn->sCtrl.sDrvStatus.transfer_timeout = 1U;
    }

    // Response in timeout interrupt
    if (isr & SDH_INTSTS_RTOIF_Msk)
    {
        printf("***** ISR: response in timeout !\n");
        pMCIn->phsdh->INTSTS |= SDH_INTSTS_RTOIF_Msk;
    }
}

static ARM_DRIVER_VERSION MCIn_GetVersion(void)
{
    return DriverVersion;
}

static ARM_MCI_CAPABILITIES MCIn_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
  \fn            int32_t Initialize (ARM_MCI_SignalEvent_t cb_event)
  \brief         Initialize the Memory Card Interface
  \param[in]     cb_event  Pointer to \ref ARM_MCI_SignalEvent
  \return        \ref execution_status
*/
static int32_t MCIn_Initialize(MCI_RESOURCES *pMCIn,
                               ARM_MCI_SignalEvent_t cb_event)
{
    // Check if the driver is already initialized
    if (pMCIn->sCtrl.u8Flags & MCI_INIT)
    {
        return ARM_DRIVER_OK;
    }

    // Clear transfer information
    memset(&pMCIn->sCtrl, 0, sizeof(MCI_CTRL));

    pMCIn->sCtrl.cb_event = cb_event; /* Event callback function */
    pMCIn->sCtrl.u8Flags = MCI_INIT;  /* Set driver initialized flag */

    return ARM_DRIVER_OK;
}

/**
  \fn            int32_t Uninitialize (void)
  \brief         De-initialize Memory Card Interface.
  \return        \ref execution_status
*/
static int32_t MCIn_Uninitialize(MCI_RESOURCES *pMCIn)
{
    // Clear driver flags
    pMCIn->sCtrl.u8Flags = 0;

    SDH_Close(pMCIn->phsdh);

    // Disable SD clock
    SDH_SetClock(pMCIn, SDH_OP_DISABLE);

    return ARM_DRIVER_OK;
}

static int32_t MCIn_PowerControl(MCI_RESOURCES *pMCIn, ARM_POWER_STATE state)
{
    if ((state != ARM_POWER_OFF)  &&
            (state != ARM_POWER_FULL) &&
            (state != ARM_POWER_LOW))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    switch (state)
    {
        case ARM_POWER_OFF:
            pMCIn->sCtrl.u8Flags &= ~MCI_POWER;

            /* Clear status */
            pMCIn->sCtrl.sDrvStatus.command_active   = 0U;
            pMCIn->sCtrl.sDrvStatus.command_timeout  = 0U;
            pMCIn->sCtrl.sDrvStatus.command_error    = 0U;
            pMCIn->sCtrl.sDrvStatus.transfer_active  = 0U;
            pMCIn->sCtrl.sDrvStatus.transfer_timeout = 0U;
            pMCIn->sCtrl.sDrvStatus.transfer_error   = 0U;
            pMCIn->sCtrl.sDrvStatus.sdio_interrupt   = 0U;
            pMCIn->sCtrl.sDrvStatus.ccs              = 0U;

            SDH_SetClock(pMCIn, SDH_OP_DISABLE);

            break;

        case ARM_POWER_LOW:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_POWER_FULL:
            if ((pMCIn->sCtrl.u8Flags & MCI_INIT)  == 0U)
            {
                return ARM_DRIVER_ERROR;
            }

            if ((pMCIn->sCtrl.u8Flags & MCI_POWER) != 0U)
            {
                return ARM_DRIVER_OK;
            }

            SDH_SetClock(pMCIn, SDH_OP_ENABLE);

            pMCIn->sCtrl.u8Flags |= MCI_POWER;
    }

    return ARM_DRIVER_OK;
}

static int32_t MCIn_CardPower(MCI_RESOURCES *pMCIn, uint32_t voltage)
{
    if (!(pMCIn->sCtrl.u8Flags & MCI_POWER))
    {
        return ARM_DRIVER_ERROR;
    }

    (void)voltage;

    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn            int32_t ReadCD (void)
  \brief         Read Card Detect (CD) state.
  \return        1:card detected, 0:card not detected, or error
*/
static int32_t MCIn_ReadCD(MCI_RESOURCES *pMCIn)
{
    if (!(pMCIn->sCtrl.u8Flags & MCI_POWER))
    {
        return ARM_DRIVER_ERROR;
    }

    return SDH_CardDetection(pMCIn->phsdh);
}

/**
  \fn            int32_t ReadWP (void)
  \brief         Read Write Protect (WP) state.
  \return        1:write protected, 0:not write protected, or error
*/
static int32_t MCIn_ReadWP(MCI_RESOURCES *pMCIn)
{
    if (!(pMCIn->sCtrl.u8Flags & MCI_POWER))
    {
        return ARM_DRIVER_ERROR;
    }

    return (0);
}

static void MCIn_GetSdInfo(SDH_T *sdh, uint32_t *u32Resp)
{
    unsigned int R_LEN, C_Size, MULT, size;
    uint32_t Buffer[4];
    SDH_INFO_T *pSD;

    if (sdh == SDH0)
    {
        pSD = &SD0;
    }
    else
    {
        pSD = &SD1;
    }

    pSD->i32ErrCode = 0;

    SDH_SDCmdAndRsp2(sdh, 9ul, pSD->RCA, Buffer);

    u32Resp[3] = Buffer[0];
    u32Resp[2] = Buffer[1];
    u32Resp[1] = Buffer[2];
    u32Resp[0] = Buffer[3];

    if ((pSD->CardType == SDH_TYPE_MMC) || (pSD->CardType == SDH_TYPE_EMMC))
    {
        /* for MMC/eMMC card */
        if ((Buffer[0] & 0xc0000000) == 0xc0000000)
        {
            /* CSD_STRUCTURE [127:126] is 3 */
            /* CSD version depend on EXT_CSD register in eMMC v4.4 for card size > 2GB */
            SDH_SDCmdAndRsp(sdh, 7ul, pSD->RCA, 0ul);

            //ptr = (uint8_t *)((uint32_t)_SDH_ucSDHCBuffer );
            sdh->DMASA = (uint32_t)pSD->dmabuf;;
            sdh->BLEN = 511ul;  /* read 512 bytes for EXT_CSD */

            //if (SDH_SDCmdAndRspDataIn(sdh, 8ul, 0x00ul) == Successful)
            //{
            //    SDH_SDCommand(sdh, 7ul, 0ul);
            //    sdh->CTL |= SDH_CTL_CLK8OEN_Msk;
            //    while ((sdh->CTL & SDH_CTL_CLK8OEN_Msk) == SDH_CTL_CLK8OEN_Msk)
            //    {
            //        if (--u32TimeOutCount <= 0)
            //        {
            //            pSD->i32ErrCode = SDH_ERR_TIMEOUT;
            //            break;
            //        }
            //    }
            //    pSD->totalSectorN = (uint32_t)(*(pSD->dmabuf + 215)) << 24;
            //    pSD->totalSectorN |= (uint32_t)(*(pSD->dmabuf + 214)) << 16;
            //    pSD->totalSectorN |= (uint32_t)(*(pSD->dmabuf + 213)) << 8;
            //    pSD->totalSectorN |= (uint32_t)(*(pSD->dmabuf + 212));
            //    pSD->diskSize = pSD->totalSectorN / 2ul;
            //}
        }
        else
        {
            /* CSD version v1.0/1.1/1.2 in eMMC v4.4 spec for card size <= 2GB */
            R_LEN = (Buffer[1] & 0x000f0000ul) >> 16;
            C_Size = ((Buffer[1] & 0x000003fful) << 2) | ((Buffer[2] & 0xc0000000ul) >> 30);
            MULT = (Buffer[2] & 0x00038000ul) >> 15;
            size = (C_Size + 1ul) * (1ul << (MULT + 2ul)) * (1ul << R_LEN);

            pSD->diskSize = size / 1024ul;
            pSD->totalSectorN = size / 512ul;
        }
    }
    else
    {
        if ((Buffer[0] & 0xc0000000) != 0x0ul)
        {
            C_Size = ((Buffer[1] & 0x0000003ful) << 16) | ((Buffer[2] & 0xffff0000ul) >> 16);
            size = (C_Size + 1ul) * 512ul;  /* Kbytes */

            pSD->diskSize = size;
            pSD->totalSectorN = size << 1;
        }
        else
        {
            R_LEN = (Buffer[1] & 0x000f0000ul) >> 16;
            C_Size = ((Buffer[1] & 0x000003fful) << 2) | ((Buffer[2] & 0xc0000000ul) >> 30);
            MULT = (Buffer[2] & 0x00038000ul) >> 15;
            size = (C_Size + 1ul) * (1ul << (MULT + 2ul)) * (1ul << R_LEN);

            pSD->diskSize = size / 1024ul;
            pSD->totalSectorN = size / 512ul;
        }
    }

    pSD->sectorSize = (int)512;
}

/**
  \fn            int32_t SendCommand (MCI_RESOURCES *mci,
                                      uint32_t  cmd,
                                      uint32_t  arg,
                                      uint32_t  flags,
                                      uint32_t *response)
  \brief         Send Command to card and get the response.
  \param[in]     cmd       Memory Card command
  \param[in]     arg       Command argument
  \param[in]     flags     Command flags
  \param[out]    response  Pointer to buffer for response
  \return        \ref execution_status
*/
int32_t MCIn_SendCommand(MCI_RESOURCES *pMCIn, uint32_t cmd, uint32_t arg,
                         uint32_t flags, uint32_t *response)
{
    SDH_T *sdh = pMCIn->phsdh;
    SDH_INFO_T *pSD = pMCIn->pSD;
    int32_t i32Res = 0;
    int32_t i32CbEvent = ARM_MCI_EVENT_COMMAND_COMPLETE;
    //uint32_t card_type = pSD->CardType;
    uint32_t u32Sector = 0;

    if ((flags & MCI_RESPONSE_EXPECTED_Msk) && (response == NULL))
        return ARM_DRIVER_ERROR_PARAMETER;

    if (!(pMCIn->sCtrl.u8Flags & MCI_SETUP))
        return ARM_DRIVER_ERROR;

    if (pMCIn->sCtrl.sDrvStatus.command_active)
        return ARM_DRIVER_ERROR_BUSY;

    pMCIn->sCtrl.u8Flags |= MCI_CMD;

    pMCIn->sCtrl.sDrvStatus.command_active = 1U;

    sdh->INTSTS = 0xFFFFFFFF;

    sdh->DMACTL |= SDH_DMACTL_DMARST_Msk;

    while (sdh->DMACTL & SDH_DMACTL_DMARST_Msk);

    sdh->DMACTL = SDH_DMACTL_DMAEN_Msk;
    sdh->DMAINTSTS = 0xFFFFFFFF;

    pSD->R3Flag = (cmd == 41);
    pSD->R7Flag = (cmd == 8);

    if (cmd == 0)
    {
        i32CbEvent = ARM_MCI_EVENT_COMMAND_COMPLETE;
        goto exit;
    }

    if (cmd == 12)
    {
        pMCIn->sCtrl.cb_event(ARM_MCI_EVENT_TRANSFER_COMPLETE);
        i32CbEvent = ARM_MCI_EVENT_COMMAND_COMPLETE;
        goto exit;
    }

    SDH_InitCard(pMCIn->phsdh, pMCIn->u32CDPinEn);

    if (cmd == 9)
    {
        MCIn_GetSdInfo(sdh, response);

        i32CbEvent = ARM_MCI_EVENT_COMMAND_COMPLETE;
        goto exit;
    }

    if ((cmd == 17) || (cmd == 18) || (cmd == 24) || (cmd == 25))
    {
        goto IgnoreSendCMD;
    }

    if (cmd == 41)
    {
        arg |= (1 << 30);
    }

    switch (flags & ARM_MCI_RESPONSE_Msk)
    {
        case ARM_MCI_RESPONSE_NONE:
            i32Res = SDH_SDCommand(sdh, cmd, arg);
            break;

        case ARM_MCI_RESPONSE_SHORT:
        case ARM_MCI_RESPONSE_SHORT_BUSY:
            i32Res = SDH_SDCmdAndRsp(sdh, cmd, arg, (cmd == 41 || cmd == 55 || cmd == 8) ? 0xFFFFF : 0);

            if (response) response[0] = sdh->RESP0;

            break;

        case ARM_MCI_RESPONSE_LONG:
            i32Res = SDH_SDCmdAndRsp2(sdh, cmd, arg, response);
            break;

        default:
            pMCIn->sCtrl.sDrvStatus.command_active = 0;
            return ARM_DRIVER_ERROR;
    }

IgnoreSendCMD:

    //printf("CMD %d, arg 0x%08X, flags 0x%08X, i32Res %d, Resp = %x\n",
    //       cmd, arg, flags, i32Res, response[0]);

    switch (cmd)
    {
        case 8:
            break;

        case 55:
        {
            uint32_t r1 = response[0];

            if ((r1 & (1UL << 5)) == 0)
            {
                // If it is an SDSC card, simulate ACCEPTED
                if (pSD->CardType == SDH_TYPE_SD_HIGH ||
                        pSD->CardType == SDH_TYPE_SD_LOW)
                {
                    r1 |= (1UL << 5);          // bit5 = APP_CMD
                    r1 |= (1UL << 8);          // bit8 = READY_FOR_DATA
                    r1 |= (4UL << 9);          // bit12:9 = TRAN state

                    if (response) response[0] = r1;

                    i32CbEvent = ARM_MCI_EVENT_COMMAND_COMPLETE;
                }
                else
                {
                    // If it is SDHC, do not override
                    i32CbEvent = ARM_MCI_EVENT_COMMAND_ERROR;
                }
            }
            else
            {
                i32CbEvent = ARM_MCI_EVENT_COMMAND_COMPLETE;
            }

            break;
        }

        case 41:
        {
            uint32_t resp = response[0];

            if ((resp == 0xFFFFFFFF) || (resp == 0x00000000))
            {
                i32CbEvent = ARM_MCI_EVENT_COMMAND_ERROR;
                break;
            }

            if (resp & (1UL << 23))  // Bit23 = ready
            {
                resp |= (1UL << 31);  // busy done (optional)

                if (response) response[0] = resp;

                i32CbEvent = ARM_MCI_EVENT_COMMAND_COMPLETE;
            }
            else
            {
                // If the card is not ready, but the host has identified it as an SD card, simulate ready (decide whether to force it if necessary)
                if (pSD->CardType == SDH_TYPE_SD_LOW || pSD->CardType == SDH_TYPE_SD_HIGH)
                {
                    resp |= (1UL << 23);  // ready
                    resp |= (1UL << 31);  // busy done

                    if (pSD->CardType == SDH_TYPE_SD_HIGH)
                        resp |= (1UL << 30);  // SDHC
                    else
                        resp &= ~(1UL << 30); // SDSC

                    if (response) response[0] = resp;

                    i32CbEvent = ARM_MCI_EVENT_COMMAND_COMPLETE;
                }
                else
                {
                    // Cannot simulate if it is not an SD card
                    i32CbEvent = ARM_MCI_EVENT_COMMAND_ERROR;
                }
            }

            break;
        }

        case 3:
            if (response)
            {
                if (pSD->CardType == SDH_TYPE_MMC ||
                        pSD->CardType == SDH_TYPE_EMMC)
                    response[0] = arg;
                else
                    response[0] = sdh->RESP0 & 0xFFFF0000;

                response[0] |= 0x00000500;
            }

            break;

        case 7:
        {
            if ((flags & ARM_MCI_RESPONSE_Msk) == ARM_MCI_RESPONSE_NONE)
            {
                if (SDH_CheckRB(sdh) != 0)
                {
                    i32CbEvent = ARM_MCI_EVENT_COMMAND_ERROR;
                }
                else
                {
                    i32CbEvent = ARM_MCI_EVENT_COMMAND_COMPLETE;
                }
            }
            else
            {
                uint32_t resp7 = sdh->RESP0;

                if (response)
                    response[0] = resp7;

                // Check if Busy (DAT0), wait for release
                if (SDH_CheckRB(sdh) != 0)
                {
                    i32CbEvent = ARM_MCI_EVENT_COMMAND_ERROR;
                    break;
                }

                // Simulate response status: Simulate entering TRAN state based on card type
                uint32_t fake_r1 = 0;
                fake_r1 |= (1UL << 8);         // Ready for Data
                fake_r1 |= (4UL << 9);         // State = TRAN
                fake_r1 &= ~(1UL << 25);       // Not Locked

                if (response)
                    response[0] = fake_r1;

                i32CbEvent = ARM_MCI_EVENT_COMMAND_COMPLETE;
            }

            break;
        }

        case 13: // CMD13 - SEND_STATUS
        {
            uint32_t resp = response[0];

            uint32_t state = (resp >> 9) & 0xF;
            uint32_t ready = (resp >> 8) & 0x1;
            uint32_t locked = (resp >> 25) & 0x1;

            if ((state == 4) && (ready == 1) && (locked == 0) && ((resp & 0x3F) == 0))
            {
                i32CbEvent = ARM_MCI_EVENT_COMMAND_COMPLETE;
            }
            else
            {
                resp &= ~(0xF << 9);  // clear state
                resp |= (8UL << 8);   // ready for data
                resp |= (1UL << 8);   // ready for data
                resp &= ~(1UL << 25);   // ready for data
                resp &= ~(0x3F);      // clear error bits

                pMCIn->sCtrl.sDrvStatus.transfer_active = 0;

                if (response) response[0] = resp;

                i32CbEvent = ARM_MCI_EVENT_COMMAND_COMPLETE;
            }

            break;
        }

        case 16:
            if (pSD->CardType == SDH_TYPE_SD_HIGH)
            {
                // SDHC does not support changing block size, it is fixed at 512 bytes
                i32CbEvent = ARM_MCI_EVENT_COMMAND_COMPLETE;
            }
            else if (pSD->CardType == SDH_TYPE_SD_LOW)
            {
                // SDSC card: need to set BLEN manually
                //sdh->BLEN = (arg - 1);  // ex: 512-1 = 511
                i32CbEvent = ARM_MCI_EVENT_COMMAND_COMPLETE;
            }
            else if (pSD->CardType == SDH_TYPE_MMC || pSD->CardType == SDH_TYPE_EMMC)
            {
                // MMC/eMMC may support variable Block Size
                //sdh->BLEN = (arg - 1);
                i32CbEvent = ARM_MCI_EVENT_COMMAND_COMPLETE;
            }
            else
            {
                i32CbEvent = ARM_MCI_EVENT_COMMAND_ERROR;
            }

            break;

        case 17: // CMD17 - READ_SINGLE_BLOCK
        case 18: // CMD18 - READ_MULTIPLE_BLOCK
            if (pMCIn->sXfer.pu8Buf)
            {
                u32Sector = (pSD->CardType == SDH_TYPE_SD_HIGH) ? arg : arg / 512;
                i32Res = SDH_Read(sdh, pMCIn->sXfer.pu8Buf, u32Sector, pMCIn->sXfer.u32Count);

                //printf("Read %x bytes, i32Res %d\n", pMCIn->sXfer.u32Count, i32Res);

                if (i32Res == 0)
                {
                    if (cmd == 17)
                    {
                        pMCIn->sCtrl.cb_event(ARM_MCI_EVENT_TRANSFER_COMPLETE);
                    }

                    i32CbEvent = ARM_MCI_EVENT_COMMAND_COMPLETE;
                }
                else
                {
                    pMCIn->sCtrl.cb_event(ARM_MCI_EVENT_TRANSFER_ERROR);
                    i32CbEvent = ARM_MCI_EVENT_COMMAND_ERROR;
                }

                pMCIn->sCtrl.u8Flags &= ~MCI_DATA;
                pMCIn->sCtrl.sDrvStatus.transfer_active = 0u;
            }

            break;

        case 24: // CMD24 - WRITE_SINGLE_BLOCK
        case 25: // CMD25 - WRITE_MULTIPLE_BLOCK
            if (pMCIn->sXfer.pu8Buf)
            {
                u32Sector = (pSD->CardType == SDH_TYPE_SD_HIGH) ? arg : arg / 512;
                i32Res = SDH_Write(sdh, pMCIn->sXfer.pu8Buf, u32Sector, pMCIn->sXfer.u32Count);

                //printf("Write %x bytes, i32Res %d\n", pMCIn->sXfer.u32Count, i32Res);

                if (i32Res == 0)
                {
                    if (cmd == 24)
                    {
                        pMCIn->sCtrl.cb_event(ARM_MCI_EVENT_TRANSFER_COMPLETE);
                    }

                    i32CbEvent = ARM_MCI_EVENT_COMMAND_COMPLETE;
                }
                else
                {
                    pMCIn->sCtrl.cb_event(ARM_MCI_EVENT_TRANSFER_ERROR);
                    i32CbEvent = ARM_MCI_EVENT_COMMAND_ERROR;
                }

                pMCIn->sCtrl.u8Flags &= ~MCI_DATA;
                pMCIn->sCtrl.sDrvStatus.transfer_active = 0u;
            }

            break;
    }

exit:
    pMCIn->sCtrl.cb_event(i32CbEvent);
    pMCIn->sCtrl.sDrvStatus.command_active = 0;
    return (i32CbEvent == ARM_MCI_EVENT_COMMAND_COMPLETE) ? ARM_DRIVER_OK : ARM_DRIVER_ERROR;
}

/**
  \fn            int32_t SetupTransfer (uint8_t *data,
                                        uint32_t block_count,
                                        uint32_t block_size,
                                        uint32_t mode)
  \brief         Setup read or write transfer operation.
  \param[in,out] data         Pointer to data block(s) to be written or read
  \param[in]     block_count  Number of blocks
  \param[in]     block_size   Size of a block in bytes
  \param[in]     mode         Transfer mode
  \return        \ref execution_status
*/
static int32_t MCIn_SetupTransfer(MCI_RESOURCES *pMCIn, uint8_t *data,
                                  uint32_t block_count, uint32_t block_size,
                                  uint32_t mode)
{
    if ((data == NULL) || (block_size == 0U))
    {
        //printf("data is null\r\n");
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (!(pMCIn->sCtrl.u8Flags & MCI_SETUP))
    {
        //printf("MCI not setup\r\n");
        return ARM_DRIVER_ERROR;
    }

    if (pMCIn->sCtrl.sDrvStatus.transfer_active)
    {
        //printf("status not ready\r\n");
        return ARM_DRIVER_ERROR_BUSY;
    }

    pMCIn->sCtrl.sDrvStatus.command_active   = 0U;
    pMCIn->sCtrl.sDrvStatus.command_timeout  = 0U;
    pMCIn->sCtrl.sDrvStatus.command_error    = 0U;
    pMCIn->sCtrl.sDrvStatus.transfer_active  = 1U;
    pMCIn->sCtrl.sDrvStatus.transfer_timeout = 0U;
    pMCIn->sCtrl.sDrvStatus.transfer_error   = 0U;
    pMCIn->sCtrl.sDrvStatus.sdio_interrupt   = 0U;
    pMCIn->sCtrl.sDrvStatus.ccs = 0U;

    if ((mode & ARM_MCI_TRANSFER_WRITE) == 0)
    {
        pMCIn->sCtrl.u8Flags |= MCI_DATA;
    }
    else
    {
        pMCIn->sCtrl.u8Flags &= ~MCI_DATA;
    }

    //printf("MCIn_SetupTransfer: block_count %d, block_size %d, mode %d\n",
    //       block_count, block_size, mode);
    // Save transfer parameters
    pMCIn->sXfer.pu8Buf        = data;
    pMCIn->sXfer.u32SectorAddr = block_count;
    pMCIn->sXfer.u32Count      = (block_count == 0) ? 1 : block_count;

    return ARM_DRIVER_OK;
}

/**
  \fn            int32_t AbortTransfer (MCI_RESOURCES *mci)
  \brief         Abort current read/write data transfer.
  \return        \ref execution_status
*/
static int32_t MCIn_AbortTransfer(MCI_RESOURCES *pMCIn)
{
    if (!(pMCIn->sCtrl.u8Flags & MCI_SETUP))
    {
        return ARM_DRIVER_ERROR;
    }

    return ARM_DRIVER_OK;
}

static int32_t MCIn_Control(MCI_RESOURCES *pMCIn, uint32_t control, uint32_t arg)
{
    NULL;
    uint32_t u32Status;

    if (!(pMCIn->sCtrl.u8Flags & MCI_POWER))
    {
        return ARM_DRIVER_ERROR;
    }

    switch (control)
    {
        case ARM_MCI_BUS_SPEED:
            pMCIn->sCtrl.u8Flags |= MCI_SETUP;

            if (arg == 0) arg = 300000;  // Default to 300KHz

            SDH_Set_clock(pMCIn->phsdh, arg);

            return SDH_Get_clock(pMCIn->phsdh);

        case ARM_MCI_BUS_SPEED_MODE:
            switch (arg)
            {
                case ARM_MCI_BUS_DEFAULT_SPEED:
                    /* Speed mode up to 25MHz */
                    SDH_Set_clock(pMCIn->phsdh, SD_FREQ);
                    break;

                case ARM_MCI_BUS_HIGH_SPEED:
                    /* Speed mode up to 50MHz */
                    u32Status = SDH_SwitchToHighSpeed(pMCIn->phsdh, pMCIn->pSD);

                    if (u32Status == Successful)
                    {
                        /* divider */
                        SDH_Set_clock(pMCIn->phsdh, SDHC_FREQ);
                    }

                    break;

                case ARM_MCI_BUS_UHS_SDR12:
                case ARM_MCI_BUS_UHS_SDR25:
                case ARM_MCI_BUS_UHS_SDR50:
                case ARM_MCI_BUS_UHS_SDR104:
                case ARM_MCI_BUS_UHS_DDR50:
                default:
                    return ARM_DRIVER_ERROR_UNSUPPORTED;
            }

            break;

        case ARM_MCI_BUS_CMD_MODE:

            /* Implement external pull-up control to support MMC cards in open-drain mode */
            /* Default mode is push-pull and is configured in Driver_MCI0.Initialize()    */
            if (arg == ARM_MCI_BUS_CMD_PUSH_PULL)
            {
                /* Configure external circuit to work in push-pull mode */
            }
            else if (arg == ARM_MCI_BUS_CMD_OPEN_DRAIN)
            {
                /* Configure external circuit to work in open-drain mode */
            }
            else
            {
                return ARM_DRIVER_ERROR_UNSUPPORTED;
            }

            break;

        case ARM_MCI_BUS_DATA_WIDTH:
            switch (arg)
            {
                case ARM_MCI_BUS_DATA_WIDTH_1:
                    pMCIn->phsdh->CTL &= ~SDH_CTL_DBW_Msk;
                    break;

                case ARM_MCI_BUS_DATA_WIDTH_4:
                    pMCIn->phsdh->CTL |= SDH_CTL_DBW_Msk;
                    break;

                case ARM_MCI_BUS_DATA_WIDTH_8:
                default:
                    return ARM_DRIVER_ERROR_UNSUPPORTED;
            }

            break;

        case ARM_MCI_CONTROL_RESET:
            /* ResetAllEngine() */
            pMCIn->phsdh->GCTL |= SDH_GCTL_GCTLRST_Msk;
            break;

        case ARM_MCI_CONTROL_CLOCK_IDLE:
            if (arg)
            {
                /* Clock generation enabled when idle */
                pMCIn->phsdh->DMACTL &= ~SDH_DMACTL_DMAEN_Msk;
            }
            else
            {
                /* Clock generation disabled when idle */
                pMCIn->phsdh->DMACTL |= SDH_DMACTL_DMAEN_Msk;
            }

            break;

        case ARM_MCI_DATA_TIMEOUT:
            pMCIn->phsdh->TOUT = (pMCIn->phsdh->TOUT & ~SDH_TOUT_TOUT_Msk) | ((arg & 0xFFFFFF) << SDH_TOUT_TOUT_Pos);
            break;

        case ARM_MCI_MONITOR_SDIO_INTERRUPT:
            if (arg == SDH_OP_ENABLE)
            {
                NVIC_EnableIRQ(((uint32_t)pMCIn->phsdh == (uint32_t)SDH0) ? SDH0_IRQn : SDH1_IRQn);
            }
            else
            {
                NVIC_DisableIRQ(((uint32_t)pMCIn->phsdh == (uint32_t)SDH0) ? SDH0_IRQn : SDH1_IRQn);
            }

            break;

        case ARM_MCI_CSS_TIMEOUT:
        case ARM_MCI_CONTROL_READ_WAIT:
        case ARM_MCI_DRIVER_STRENGTH:
        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

static ARM_MCI_STATUS MCIn_GetStatus(MCI_RESOURCES *pMCIn)
{
    return pMCIn->sCtrl.sDrvStatus;
}

static int32_t SDH_SetClock(MCI_RESOURCES *pMCIn, uint32_t u32ClkEn)
{
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    /* Unlock protected registers */
    if (u32RegLockLevel)
    {
        SYS_UnlockReg();
    }

    /* Reset controller */
    SYS_ResetModule(((uint32_t)pMCIn->phsdh == (uint32_t)SDH0) ? SYS_SDH0RST : SYS_SDH1RST);

    if (u32ClkEn == SDH_OP_ENABLE)
    {
        SDH_InitCard(pMCIn->phsdh, pMCIn->u32CDPinEn);

        pMCIn->sCtrl.u8Flags |= MCI_SETUP;
    }
    else
    {
        pMCIn->sCtrl.u8Flags &= ~MCI_SETUP;
    }

    /* Lock protected registers */
    if (u32RegLockLevel)
    {
        SYS_LockReg();
    }

    return ARM_DRIVER_OK;
}

/**
 * @brief Get the clock frequency of the SDH controller.
 *
 * This function retrieves the clock frequency of the SDH controller.
 *
 * @param sdh Pointer to the SDH controller.
 * @return The clock frequency of the SDH controller.
 */
static uint32_t SDH_Get_clock(SDH_T *sdh)
{
    uint32_t refClock, div1, rate;

    if (sdh == SDH0)
    {
        switch (CLK->SDHSEL & CLK_SDHSEL_SDH0SEL_Msk)
        {
            case CLK_SDHSEL_SDH0SEL_APLL1_DIV2:
                refClock = (CLK_GetAPLL1ClockFreq() >> 1) / 1000ul;
                break;

            case CLK_SDHSEL_SDH0SEL_HCLK0:
                refClock = CLK_GetHCLK0Freq() / 1000ul;
                break;

            case CLK_SDHSEL_SDH0SEL_HIRC:
                refClock = __HIRC / 1000ul;
                break;

            case CLK_SDHSEL_SDH0SEL_HIRC48M_DIV4:
                refClock = (__HIRC48M / 1000ul) / 4;
                break;

            case CLK_SDHSEL_SDH0SEL_HXT:
            default:
                refClock = CLK_GetHXTFreq() / 1000ul;
                break;
        }

        div1 = (CLK->SDHDIV & CLK_SDHDIV_SDH0DIV_Msk) >> CLK_SDHDIV_SDH0DIV_Pos;
    }
    else
    {
        switch (CLK->SDHSEL & CLK_SDHSEL_SDH1SEL_Msk)
        {
            case CLK_SDHSEL_SDH1SEL_APLL1_DIV2:
                refClock = (CLK_GetAPLL1ClockFreq() >> 1) / 1000ul;
                break;

            case CLK_SDHSEL_SDH1SEL_HCLK0:
                refClock = CLK_GetHCLK0Freq() / 1000ul;
                break;

            case CLK_SDHSEL_SDH1SEL_HIRC:
                refClock = __HIRC / 1000ul;
                break;

            case CLK_SDHSEL_SDH1SEL_HIRC48M_DIV4:
                refClock = (__HIRC48M / 1000ul) / 4;
                break;

            case CLK_SDHSEL_SDH1SEL_HXT:
            default:
                refClock = CLK_GetHXTFreq() / 1000ul;
                break;
        }

        div1 = (CLK->SDHDIV & CLK_SDHDIV_SDH1DIV_Msk) >> CLK_SDHDIV_SDH1DIV_Pos;
    }

    rate = refClock / (div1 + 1ul);

    return rate;
}

/**
 * @brief Initializes the SD card.
 *
 * This function initializes the SD card for the specified SDH controller.
 *
 * @param sdh The SDH controller to initialize the card for.
 * @param u32CardDetect The card detection status.
 */
static void SDH_InitCard(SDH_T *sdh, uint32_t u32CardDetect)
{
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    /* Unlock protected registers */
    if (u32RegLockLevel)
    {
        SYS_UnlockReg();
    }

    //SDH_Close(sdh);

    // Open SDH interface
    SDH_Open(sdh,
             ((u32CardDetect == SDH_OP_ENABLE) ?
              CardDetect_From_GPIO :
              CardDetect_From_DAT3)
            );

    // Probe SDH interface
    SDH_Probe(sdh);

    sdh->INTEN &= ~SDH_INTEN_CDIEN_Msk;

    /* Lock protected registers */
    if (u32RegLockLevel)
    {
        SYS_LockReg();
    }
}
// Local driver functions definitions (for instances)
// Information definitions (for instances)
#if (RTE_MCI0 == 1)
FUNCS_DEFINE(0)
MCI_DRIVER(0)

NVT_ITCM void SDH0_IRQHandler(void)
{
    SDH_IRQHandler(&mci0_res);
}
#endif // RTE_MCI0

#if (RTE_MCI1 == 1)
FUNCS_DEFINE(1)
MCI_DRIVER(1)

NVT_ITCM  void SDH1_IRQHandler(void)
{
    SDH_IRQHandler(&mci1_res);
}
#endif // RTE_MCI1

#endif // DRIVER_CONFIG_VALID
