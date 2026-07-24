/****************************************************************************
 * @file     canfd.c
 * @version  V1.00
 * @brief    CAN FD driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NuMicro.h"
#include "string.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Minimum number of time quanta in a bit. */
#define MIN_TIME_QUANTA    8
/* Maximum number of time quanta in a bit. */
#define MAX_TIME_QUANTA    25
/* Number of receive FIFOs (1 - 2) */
#define CANFD_NUM_RX_FIFOS  2

/*CAN FD max nominal bit rate*/
#define MAX_NOMINAL_BAUDRATE (1000000UL)

/*CAN FD memory size */
#define CANFD_SRAM_SIZE      6144

/* Tx Event FIFO Element ESI(Error State Indicator)  */
#define TX_FIFO_E0_EVENT_ESI_Pos   (31)
#define TX_FIFO_E0_EVENT_ESI_Msk   (0x1UL << TX_FIFO_E0_EVENT_ESI_Pos)

/* Tx Event FIFO Element XTD(Extended Identifier)    */
#define TX_FIFO_E0_EVENT_XTD_Pos   (30)
#define TX_FIFO_E0_EVENT_XTD_Msk   (0x1UL << TX_FIFO_E0_EVENT_XTD_Pos)

/* Tx Event FIFO Element RTR(Remote Transmission Request)    */
#define TX_FIFO_E0_EVENT_RTR_Pos   (29)
#define TX_FIFO_E0_EVENT_RTR_Msk   (0x1UL << TX_FIFO_E0_EVENT_RTR_Pos)

/* Tx Event FIFO Element ID(Identifier)    */
#define TX_FIFO_E0_EVENT_ID_Pos    (0)
#define TX_FIFO_E0_EVENT_ID_Msk    (0x1FFFFFFFUL << TX_FIFO_E0_EVENT_ID_Pos)

/* Tx Event FIFO Element MM(Message Marker)    */
#define TX_FIFO_E1_EVENT_MM_Pos    (24)
#define TX_FIFO_E1_EVENT_MM_Msk    (0xFFUL << TX_FIFO_E1_EVENT_MM_Pos)

/* Tx Event FIFO Element ET(Event Type)    */
#define TX_FIFO_E1_EVENT_ET_Pos    (22)
#define TX_FIFO_E1_EVENT_ET_Msk    (0x3UL << TX_FIFO_E1_EVENT_ET_Pos)

/* Tx Event FIFO Element FDF(FD Format)    */
#define TX_FIFO_E1_EVENT_FDF_Pos    (21)
#define TX_FIFO_E1_EVENT_FDF_Msk    (0x1UL << TX_FIFO_E1_EVENT_FDF_Pos)

/* Tx Event FIFO Element BRS(Bit Rate Switch)    */
#define TX_FIFO_E1_EVENT_BRS_Pos    (20)
#define TX_FIFO_E1_EVENT_BRS_Msk    (0x1UL << TX_FIFO_E1_EVENT_BRS_Pos)

/* Tx Event FIFO Element DLC(Data Length Code)    */
#define TX_FIFO_E1_EVENT_DLC_Pos    (16)
#define TX_FIFO_E1_EVENT_DLC_Msk    (0xFUL << TX_FIFO_E1_EVENT_DLC_Pos)

/* Tx Event FIFO Element TXTS(Tx Timestamp)    */
#define TX_FIFO_E1A_EVENT_TXTS_Pos  (0)
#define TX_FIFO_E1A_EVENT_TXTS_Msk  (0xFFFFUL << TX_FIFO_E1A_EVENT_TXTS_Pos)

/* Tx Event FIFO Element MM(Message Marker)    */
#define TX_FIFO_E1B_EVENT_MM_Pos    (8)
#define TX_FIFO_E1B_EVENT_MM_Msk    (0xFFUL << TX_FIFO_E1B_EVENT_MM_Pos)

/* Tx Event FIFO Element TSC(Timestamp Captured)    */
#define TX_FIFO_E1B_EVENT_TSC_Pos   (4)
#define TX_FIFO_E1B_EVENT_TSC_Msk   (0x1UL << TX_FIFO_E1B_EVENT_TSC_Pos)

/* Tx Event FIFO Element TXTSP(Tx Timestamp Pointer) */
#define TX_FIFO_E1B_EVENT_TXTSP_Pos (0)
#define TX_FIFO_E1B_EVENT_TXTSP_Msk (0xFUL << TX_FIFO_E1B_EVENT_TXTSP_Pos)

/* Rx Buffer and FIFO Element ESI2(Error State Indicator)    */
#define RX_BUFFER_AND_FIFO_R0_ELEM_ESI_Pos  (31)
#define RX_BUFFER_AND_FIFO_R0_ELEM_ESI_Msk  (0x1UL << RX_BUFFER_AND_FIFO_R0_ELEM_ESI_Pos)

/* Rx Buffer and FIFO Element XTD(Extended Identifier)    */
#define RX_BUFFER_AND_FIFO_R0_ELEM_XTD_Pos  (30)
#define RX_BUFFER_AND_FIFO_R0_ELEM_XTD_Msk  (0x1UL << RX_BUFFER_AND_FIFO_R0_ELEM_XTD_Pos)

/* Rx Buffer and FIFO Element RTR(Remote Transmission Request)    */
#define RX_BUFFER_AND_FIFO_R0_ELEM_RTR_Pos  (29)
#define RX_BUFFER_AND_FIFO_R0_ELEM_RTR_Msk  (0x1UL << RX_BUFFER_AND_FIFO_R0_ELEM_RTR_Pos)

/* Rx Buffer and FIFO Element ID(Identifier)    */
#define RX_BUFFER_AND_FIFO_R0_ELEM_ID_Pos  (0)
#define RX_BUFFER_AND_FIFO_R0_ELEM_ID_Msk  (0x1FFFFFFFUL << RX_BUFFER_AND_FIFO_R0_ELEM_ID_Pos)

/* Rx Buffer and FIFO Element ANMF(Accepted Non-matching Frame)    */
#define RX_BUFFER_AND_FIFO_R1_ELEM_ANMF_Pos  (31)
#define RX_BUFFER_AND_FIFO_R1_ELEM_ANMF_Msk  (0x1UL << RX_BUFFER_AND_FIFO_R1_ELEM_ANMF_Pos)

/* Rx Buffer and FIFO Element FIDX(Filter Index)    */
#define RX_BUFFER_AND_FIFO_R1_ELEM_FIDX_Pos  (24)
#define RX_BUFFER_AND_FIFO_R1_ELEM_FIDX_Msk  (0x7FUL << RX_BUFFER_AND_FIFO_R1_ELEM_FIDX_Pos)

/* Rx Buffer and FIFO Element FDF(FD Format)    */
#define RX_BUFFER_AND_FIFO_R1_ELEM_FDF_Pos  (21)
#define RX_BUFFER_AND_FIFO_R1_ELEM_FDF_Msk  (0x1UL << RX_BUFFER_AND_FIFO_R1_ELEM_FDF_Pos)

/* Rx Buffer and FIFO Element BRS(Bit Rate Swit)    */
#define RX_BUFFER_AND_FIFO_R1_ELEM_BSR_Pos  (20)
#define RX_BUFFER_AND_FIFO_R1_ELEM_BSR_Msk  (0x1UL << RX_BUFFER_AND_FIFO_R1_ELEM_BSR_Pos)

/* Rx Buffer and FIFO Element DLC(Bit Rate Swit)    */
#define RX_BUFFER_AND_FIFO_R1_ELEM_DLC_Pos  (16)
#define RX_BUFFER_AND_FIFO_R1_ELEM_DLC_Msk  (0xFUL << RX_BUFFER_AND_FIFO_R1_ELEM_DLC_Pos)

/* Rx Buffer and FIFO Element RXTS(Rx Timestamp)    */
#define RX_BUFFER_AND_FIFO_R1_ELEM_RXTS_Pos  (0)
#define RX_BUFFER_AND_FIFO_R1_ELEM_RXTS_Msk  (0xFFFFUL << RX_BUFFER_AND_FIFO_R1_ELEM_RXTS_Pos)

/* Tx Buffer Element ESI(Error State Indicator)    */
#define TX_BUFFER_T0_ELEM_ESI_Pos  (31)
#define TX_BUFFER_T0_ELEM_ESI_Msk  (0x1UL << TX_BUFFER_T0_ELEM_ESI_Pos)

/* Tx Buffer Element XTD(Extended Identifier)    */
#define TX_BUFFER_T0_ELEM_XTD_Pos  (30)
#define TX_BUFFER_T0_ELEM_XTD_Msk (0x1UL << TX_BUFFER_T0_ELEM_XTD_Pos)

/* Tx Buffer RTR(Remote Transmission Request)    */
#define TX_BUFFER_T0_ELEM_RTR_Pos  (29)
#define TX_BUFFER_T0_ELEM_RTR_Msk  (0x1UL << TX_BUFFER_T0_ELEM_RTR_Pos)

/* Tx Buffer Element ID(Identifier)    */
#define TX_BUFFER_T0_ELEM_ID_Pos  (0)
#define TX_BUFFER_T0_ELEM_ID_Msk  (0x1FFFFFFFUL << TX_BUFFER_T0_ELEM_ID_Pos)

/* Tx Buffer Element MM(Message Marker)    */
#define TX_BUFFER_T1_ELEM_MM1_Pos  (24)
#define TX_BUFFER_T1_ELEM_MM1_Msk  (0xFFUL << TX_BUFFER_T1_ELEM_MM1_Pos)

/* Tx Buffer Element EFC(Event FIFO Control)    */
#define TX_BUFFER_T1_ELEM_EFC_Pos  (23)
#define TX_BUFFER_T1_ELEM_EFC_Msk  (0x1UL << TX_BUFFER_T1_ELEM_EFC_Pos)

/* Tx Buffer Element TSCE(Time Stamp Capture Enable for TSU)    */
#define TX_BUFFER_T1_ELEM_TSCE_Pos  (22)
#define TX_BUFFER_T1_ELEM_TSCE_Msk  (0x1UL << TX_BUFFER_T1_ELEM_TSCE_Pos)

/* Tx Buffer Element FDF(FD Format)    */
#define TX_BUFFER_T1_ELEM_FDF_Pos  (21)
#define TX_BUFFER_T1_ELEM_FDF_Msk  (0x1UL << TX_BUFFER_T1_ELEM_FDF_Pos)

/* Tx Buffer Element BRS(Bit Rate Swit)    */
#define TX_BUFFER_T1_ELEM_BSR_Pos  (20)
#define TX_BUFFER_T1_ELEM_BSR_Msk  (0x1UL << TX_BUFFER_T1_ELEM_BSR_Pos)

/* Tx Buffer Element DLC(Bit Rate Swit)    */
#define TX_BUFFER_T1_ELEM_DLC_Pos  (16)
#define TX_BUFFER_T1_ELEM_DLC_Msk  (0xFUL << TX_BUFFER_T1_ELEM_DLC_Pos)

/* Tx Buffer Element MM(Message Marker)    */
#define TX_BUFFER_T1_ELEM_MM0_Pos  (8)
#define TX_BUFFER_T1_ELEM_MM0_Msk  (0xFFUL << TX_BUFFER_T1_ELEM_MM0_Pos)

#define CANFD_RXFS_RFL CANFD_RXF0S_RF0L_Msk


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup CANFD_Driver CAN_FD Driver
  @{
*/

/** @addtogroup CANFD_EXPORTED_FUNCTIONS CAN_FD Exported Functions
  @{
*/

/** @cond HIDDEN_SYMBOLS */
/**
  * @brief      Returns module clock of specified CAN FD interface
  *
  * @param[in]  psCanfd      The pointer of CAN FD module.
  *
  * @return     Module clock of specified CAN FD interface.
  */
static uint32_t CANFD_GetSourceClock(const CANFD_T *psCanfd)
{
    uint32_t u32ClkSrc = 0;
    uint32_t u32ClkFreq = __HIRC;

    /* Get CAN FD module clock source and divider */
    if (psCanfd == (CANFD_T *)CANFD0)
    {
        u32ClkSrc = CLK_GetModuleClockSource(CANFD0_MODULE);
    }
    else if (psCanfd == (CANFD_T *)CANFD1)
    {
        u32ClkSrc = CLK_GetModuleClockSource(CANFD1_MODULE);
    }
    else
    {
        u32ClkFreq = 0UL;
    }

    if (u32ClkFreq != 0UL)
    {
        /* Get CAN FD module clock */
        if (u32ClkSrc == 0UL)
        {
            u32ClkFreq = __HXT;
        }
        else if (u32ClkSrc == 1UL)
        {
            u32ClkFreq = CLK_GetAPLL0ClockFreq() / 2UL;
        }
        else if (u32ClkSrc == 2UL)
        {
            u32ClkFreq = CLK_GetHCLK0Freq();
        }
        else if (u32ClkSrc == 3UL)
        {
            u32ClkFreq = __HIRC;
        }
        else
        {
            u32ClkFreq = __HIRC48M / 4UL;
        }
    }

    return u32ClkFreq;
}
/** @endcond HIDDEN_SYMBOLS */

/**
 * @brief       Gets the CAN FD interface Nominal bit rate.
 *
 * @param[in]   psCanfd      The pointer of CAN FD module.
 *
 * @return      Nominal bit rate of specified CAN FD interface.
 * @details     Gets the CAN FD interface Nominal bit rate.
 */
uint32_t CANFD_GetNominalBitRate(const CANFD_T *psCanfd)
{
    uint32_t u8Tq = 0UL;
    uint32_t u8NtSeg1 = 0UL;
    uint32_t u8NtSeg2 = 0UL;
    uint32_t u32BitRate = 0UL;
    uint32_t u32CanfdClk = 0UL;
    uint32_t u32CanfdDiv = 0UL;

    u32CanfdClk = CANFD_GetSourceClock(psCanfd);

    if (psCanfd == (CANFD_T *)CANFD0)
    {
        u32CanfdDiv = ((CLK->CANFDDIV & CLK_CANFDDIV_CANFD0DIV_Msk) >> CLK_CANFDDIV_CANFD0DIV_Pos) + 1UL;
    }
    else if (psCanfd == (CANFD_T *)CANFD1)
    {
        u32CanfdDiv = ((CLK->CANFDDIV & CLK_CANFDDIV_CANFD1DIV_Msk) >> CLK_CANFDDIV_CANFD1DIV_Pos) + 1UL;
    }
    else
    {
        return 0UL;
    }

    u32CanfdClk = u32CanfdClk / u32CanfdDiv;
    u8Tq = ((psCanfd->NBTP & CANFD_NBTP_NBRP_Msk) >> CANFD_NBTP_NBRP_Pos) + 1UL;
    u8NtSeg1 = ((psCanfd->NBTP & CANFD_NBTP_NTSEG1_Msk) >> CANFD_NBTP_NTSEG1_Pos);
    u8NtSeg2 = ((psCanfd->NBTP & CANFD_NBTP_NTSEG2_Msk) >> CANFD_NBTP_NTSEG2_Pos);
    u32BitRate = (u32CanfdClk / u8Tq) / (u8NtSeg1 + u8NtSeg2 + 3UL);

    return u32BitRate;
}

/**
 * @brief       Gets the CAN FD interface Data bit rate.
 *
 * @param[in]   psCanfd      The pointer of CAN FD module.
 *
 * @return      Data bit rate of specified CAN FD interface.
 * @details     Gets the CAN FD interface Data bit rate.
 *              If CAN FD interface is in CAN mode, this function will return 0. Because data bit rate is only valid in CAN FD mode.
 */
uint32_t CANFD_GetDataBitRate(const CANFD_T *psCanfd)
{
    uint32_t u8Tq = 0UL;
    uint32_t u8NtSeg1 = 0UL;
    uint32_t u8NtSeg2 = 0UL;
    uint32_t u32BitRate = 0UL;
    uint32_t u32CanfdClk = 0UL;
    uint32_t u32CanfdDiv = 0UL;

    if ((psCanfd->CCCR & CANFD_CCCR_FDOE_Msk) == 0UL)
    {
        return 0UL;
    }

    u32CanfdClk = CANFD_GetSourceClock(psCanfd);

    if (psCanfd == (CANFD_T *)CANFD0)
    {
        u32CanfdDiv = ((CLK->CANFDDIV & CLK_CANFDDIV_CANFD0DIV_Msk) >> CLK_CANFDDIV_CANFD0DIV_Pos) + 1UL;
    }
    else if (psCanfd == (CANFD_T *)CANFD1)
    {
        u32CanfdDiv = ((CLK->CANFDDIV & CLK_CANFDDIV_CANFD1DIV_Msk) >> CLK_CANFDDIV_CANFD1DIV_Pos) + 1UL;
    }
    else
    {
        return 0UL;
    }

    u32CanfdClk = u32CanfdClk / u32CanfdDiv;
    u8Tq = ((psCanfd->DBTP & CANFD_DBTP_DBRP_Msk) >> CANFD_DBTP_DBRP_Pos) + 1UL;
    u8NtSeg1 = ((psCanfd->DBTP & CANFD_DBTP_DTSEG1_Msk) >> CANFD_DBTP_DTSEG1_Pos);
    u8NtSeg2 = ((psCanfd->DBTP & CANFD_DBTP_DTSEG2_Msk) >> CANFD_DBTP_DTSEG2_Pos);
    u32BitRate = (u32CanfdClk / u8Tq) / (u8NtSeg1 + u8NtSeg2 + 3UL);

    return u32BitRate;
}

/**
 * @brief       Calculates the CAN FD RAM buffer address.
 *
 * @param[in]   psConfigAddr  CAN FD element star address structure.
 * @param[in]   psConfigSize  CAN FD element size structure.
 *
 * @return      None.
 *
 * @details     Calculates the CAN FD RAM buffer address.
 */
static void CANFD_CalculateRamAddress(CANFD_RAM_PART_T *psConfigAddr, CANFD_ELEM_SIZE_T *psConfigSize)
{
    uint32_t u32RamAddrOffset = 0;

    /* Get the Standard Message ID Filter element address */
    if (psConfigSize->u32SIDFC > 0UL)
    {
        psConfigAddr->u32SIDFC_FLSSA = 0;
        u32RamAddrOffset += psConfigSize->u32SIDFC * sizeof(CANFD_STD_FILTER_T);
    }

    /* Get the Extend Message ID Filter element address */
    if (psConfigSize->u32XIDFC > 0UL)
    {
        psConfigAddr->u32XIDFC_FLESA = u32RamAddrOffset;
        u32RamAddrOffset += psConfigSize->u32XIDFC * sizeof(CANFD_EXT_FILTER_T);
    }

    /* Get the Tx Buffer and Tx FIFO/Queue element address */
    if ((psConfigSize->u32TxBuf + psConfigSize->u32TxFifoQueue)  > 0UL)
    {
        if ((psConfigSize->u32TxBuf + psConfigSize->u32TxFifoQueue) > 32UL)
        {
            psConfigSize->u32TxBuf = 16;
            psConfigSize->u32TxFifoQueue = 16;
        }

        psConfigAddr->u32TXBC_TBSA = u32RamAddrOffset;
        u32RamAddrOffset += (psConfigSize->u32TxBuf + psConfigSize->u32TxFifoQueue) * sizeof(CANFD_BUF_T);
    }

    /* Get the Rx Buffer element address */
    if (psConfigSize->u32RxBuf > 0UL)
    {
        psConfigAddr->u32RXBC_RBSA = u32RamAddrOffset;
        u32RamAddrOffset += psConfigSize->u32RxBuf * sizeof(CANFD_BUF_T);
    }

    /* Get the Rx FIFO0 element address */
    if (psConfigSize->u32RxFifo0 > 0UL)
    {
        psConfigAddr->u32RXF0C_F0SA = u32RamAddrOffset;
        u32RamAddrOffset += psConfigSize->u32RxFifo0 * sizeof(CANFD_BUF_T);
    }

    /* Get the Rx FIFO1 element address */
    if (psConfigSize->u32RxFifo1 > 0UL)
    {
        psConfigAddr->u32RXF1C_F1SA = u32RamAddrOffset;
        u32RamAddrOffset += psConfigSize->u32RxFifo1 * sizeof(CANFD_BUF_T);
    }

    /* Get the Rx FIFO1 element address */
    if (psConfigSize->u32TxEventFifo > 0UL)
    {
        psConfigAddr->u32TXEFC_EFSA = u32RamAddrOffset;
    }
}


/**
 * @brief       Get the default configuration structure.
 *
 * @param[in]   psConfig       Pointer to CAN FD configuration structure.
 * @param[in]   u8OpMode       Setting the CAN FD Operating mode.
 *
 * @return      None.
 *
 * @details     This function initializes the CAN FD configure structure to default value.
 *              The default value are:
 *              sNormBitRate.u32BitRate   = 500000bps;
 *              u32DataBaudRate     = 0(CAN mode) or 1000000(CAN FD mode) ;
 *              u32MRamSize         = 6144 bytes (1536 words);
 *              bEnableLoopBack     = FALSE;
 *              bBitRateSwitch      = FALSE(CAN Mode) or TRUE(CAN FD Mode);
 *              bFDEn               = FALSE(CAN Mode) or TRUE(CAN FD Mode);
 *              CAN FD Standard ID elements = 30
 *              CAN FD Extended ID elements = 30
 *              CAN FD TX Buffer elements = 20
 *              CAN FD RX Buffer elements = 20
 *              CAN FD RX FIFO0 elements = 20
 *              CAN FD RX FIFO1 elements = 20
 *              CAN FD TX Event FIFO elements = 3
*/
void CANFD_GetDefaultConfig(CANFD_FD_T *psConfig, uint8_t u8OpMode)
{
    if (psConfig->sElemSize.u32UserDef == 0UL)
    {
        (void)memset(psConfig, 0, sizeof(CANFD_FD_T));
    }

    psConfig->sBtConfig.sNormBitRate.u32BitRate = 500000;

    if (u8OpMode == (uint8_t)CANFD_OP_CAN_MODE)
    {
        psConfig->sBtConfig.sDataBitRate.u32BitRate = 0;
        psConfig->sBtConfig.bFDEn = FALSE;
        psConfig->sBtConfig.bBitRateSwitch = FALSE;
    }
    else
    {
        psConfig->sBtConfig.sDataBitRate.u32BitRate = 1000000;
        psConfig->sBtConfig.bFDEn = TRUE;
        psConfig->sBtConfig.bBitRateSwitch = TRUE;
    }

    /*Disable the Internal Loopback mode */
    psConfig->sBtConfig.bEnableLoopBack = FALSE;
    /*Get the CAN FD memory size(number of byte)*/
    psConfig->u32MRamSize  = CANFD_SRAM_SIZE;

    if (psConfig->sElemSize.u32UserDef == 0UL)
    {
        /* CAN FD Standard message ID elements as 30 elements    */
        psConfig->sElemSize.u32SIDFC = 30;
        /* CAN FD Extended message ID elements as 30 elements    */
        psConfig->sElemSize.u32XIDFC = 30;
        /* CAN FD TX Buffer elements as 10 elements    */
        psConfig->sElemSize.u32TxBuf = 15;
        /* CAN FD Tx FIFO/Queue elements as 5 elements    */
        psConfig->sElemSize.u32TxFifoQueue = 5;
        /* CAN FD RX Buffer elements as 20 elements    */
        psConfig->sElemSize.u32RxBuf = 20;
        /* CAN FD RX FIFO0 elements as 20 elements    */
        psConfig->sElemSize.u32RxFifo0 = 20;
        /* CAN FD RX FIFO1 elements as 20 elements    */
        psConfig->sElemSize.u32RxFifo1 = 20;
        /* CAN FD TX Event FOFI elements as 3 elements    */
        psConfig->sElemSize.u32TxEventFifo = 3;
        /*Calculates the CAN FD RAM buffer address*/
    }

    CANFD_CalculateRamAddress(&psConfig->sMRamStartAddr, &psConfig->sElemSize);
}


/**
 * @brief       Encode the Data Length Code.
 *
 * @param[in]   u8NumberOfBytes  Number of bytes in a message.
 *
 * @return      Data Length Code.
 *
 * @details     Converts number of bytes in a message into a Data Length Code.
 */
static uint8_t CANFD_EncodeDLC(uint8_t u8NumberOfBytes)
{
    if (u8NumberOfBytes <= (uint8_t)8)
    {
        return u8NumberOfBytes;
    }
    else if (u8NumberOfBytes <= (uint8_t)12)
    {
        return 9;
    }
    else if (u8NumberOfBytes <= (uint8_t)16)
    {
        return 10;
    }
    else if (u8NumberOfBytes <= (uint8_t)20)
    {
        return 11;
    }
    else if (u8NumberOfBytes <= (uint8_t)24)
    {
        return 12;
    }
    else if (u8NumberOfBytes <= (uint8_t)32)
    {
        return 13;
    }
    else if (u8NumberOfBytes <= (uint8_t)48)
    {
        return 14;
    }
    else
    {
        return 15;
    }
}


/**
 * @brief       Decode the Data Length Code.
 *
 * @param[in]   u8Dlc   Data Length Code.
 *
 * @return      Number of bytes in a message.
 *
 * @details     Converts a Data Length Code into a number of message bytes.
 */
static uint8_t CANFD_DecodeDLC(uint8_t u8Dlc)
{
    if (u8Dlc <= (uint8_t)8)
    {
        return u8Dlc;
    }
    else if (u8Dlc == (uint8_t)9)
    {
        return 12;
    }
    else if (u8Dlc == (uint8_t)10)
    {
        return 16;
    }
    else if (u8Dlc == (uint8_t)11)
    {
        return 20;
    }
    else if (u8Dlc == (uint8_t)12)
    {
        return 24;
    }
    else if (u8Dlc == (uint8_t)13)
    {
        return 32;
    }
    else if (u8Dlc == (uint8_t)14)
    {
        return 48;
    }
    else
    {
        return 64;
    }
}


/**
 * @brief       Sets the CAN FD protocol timing characteristic.
 *
 * @param[in]   psCanfd     The pointer of the specified CANFD module.
 * @param[in]   psConfig    Pointer to the timing configuration structure.
 *
 * @return      None.
 *
 * @details     This function gives user settings to CAN bus timing characteristic.
 *              The function is for an experienced user. For less experienced users, call
 *              the CANFD_Open() and fill the baud rate field with a desired value.
 *              This provides the default timing characteristics to the module.
 */
static void CANFD_SetTimingConfig(CANFD_T *psCanfd, const CANFD_TIMEING_CONFIG_T *psConfig)
{

    /* configuration change enable */
    psCanfd->CCCR |= CANFD_CCCR_CCE_Msk;

    if (psCanfd == (CANFD_T *)CANFD0)
    {
        /* Get CANF D0 clock divider number */
        CLK->CANFDDIV = (CLK->CANFDDIV & ~CLK_CANFDDIV_CANFD0DIV_Msk) | CLK_CANFDDIV_CANFD0DIV(psConfig->u8PreDivider);
    }
    else
    {
        CLK->CANFDDIV = (CLK->CANFDDIV & ~CLK_CANFDDIV_CANFD1DIV_Msk) | CLK_CANFDDIV_CANFD1DIV(psConfig->u8PreDivider);
    }

    /* nominal bit rate */
    psCanfd->NBTP = (((psConfig->u8NominalRJumpwidth & 0x7FUL) - 1UL) << 25) +
                    (((psConfig->u16NominalPrescaler & 0x1FFUL) - 1UL) << 16) +
                    ((((psConfig->u8NominalPhaseSeg1 + psConfig->u8NominalPropSeg) & 0xFFUL) - 1UL) << 8) +
                    (((psConfig->u8NominalPhaseSeg2 & 0x7FUL) - 1UL) << 0);


    /* canfd->DBTP */
    if (psCanfd->CCCR & CANFD_CCCR_FDOE_Msk)
    {

        psCanfd->DBTP = (((psConfig->u8DataPrescaler & 0x1FUL) - 1UL) << 16) +
                        ((((psConfig->u8DataPhaseSeg1 + psConfig->u8DataPropSeg) & 0x1FUL) - 1UL) << 8) +
                        (((psConfig->u8DataPhaseSeg2 & 0xFUL) - 1UL) << 4) +
                        (((psConfig->u8DataRJumpwidth & 0xFUL) - 1UL) << 0);

    }
}


/**
 * @brief       Get the segment values.
 *
 * @param[in]   u32NominalBaudRate  The nominal speed in bps.
 * @param[in]   u32DataBaudRate     The data speed in bps.
 * @param[in]   u32Ntq              Number of nominal time quanta per bit.
 * @param[in]   u32Dtq              Number of data time quanta per bit.
 * @param[in]   psConfig            Passed is a configuration structure, on return the configuration is stored in the structure
 *
 * @return      None.
 *
 * @details     Calculates the segment values for a single bit time for nominal and data baudrates.
 */
static void CANFD_GetSegments(uint32_t u32NominalBaudRate, uint32_t u32DataBaudRate, uint32_t u32Ntq, uint32_t u32Dtq, CANFD_TIMEING_CONFIG_T *psConfig)
{
    float ideal_sp;
    uint32_t u32P1;

    /* get ideal sample point */
    if (u32NominalBaudRate >= 1000000UL)
    {
        ideal_sp = 0.750;
    }
    else if (u32NominalBaudRate >= 800000UL)
    {
        ideal_sp = 0.800;
    }
    else
    {
        ideal_sp = 0.875;
    }

    /* distribute time quanta */
    u32P1 = (uint32_t)((float)u32Ntq * ideal_sp);
    /* can controller doesn't separate prop seg and phase seg 1 */
    psConfig->u8NominalPropSeg = 0;
    /* subtract one TQ for sync seg */
    psConfig->u8NominalPhaseSeg1 = u32P1 - 1UL;
    psConfig->u8NominalPhaseSeg2 = u32Ntq - u32P1;
    /* sjw is 20% of total TQ, rounded to nearest int */
    psConfig->u8NominalRJumpwidth = (u32Ntq + (4UL)) / 5UL;


    /* if using baud rate switching then distribute time quanta for data rate */
    if (u32Dtq > 0UL)
    {
        /* get ideal sample point */
        if (u32DataBaudRate >= 1000000UL)
        {
            ideal_sp = 0.750;
        }
        else if (u32DataBaudRate >= 800000UL)
        {
            ideal_sp = 0.800;
        }
        else
        {
            ideal_sp = 0.875;
        }

        /* distribute time quanta */
        u32P1 = (uint32_t)((float)u32Dtq * ideal_sp);
        /* can controller doesn't separate prop seg and phase seg 1 */
        psConfig->u8DataPropSeg = 0;
        /* subtract one TQ for sync seg */
        psConfig->u8DataPhaseSeg1 = u32P1 - 1UL;
        psConfig->u8DataPhaseSeg2 = u32Dtq - u32P1;
        /* sjw is 20% of total TQ, rounded to nearest int */
        psConfig->u8DataRJumpwidth = (u32Dtq + (4UL)) / 5UL;
    }
    else
    {
        psConfig->u8DataPropSeg = 0;
        psConfig->u8DataPhaseSeg1 = 0;
        psConfig->u8DataPhaseSeg2 = 0;
        psConfig->u8DataRJumpwidth = 0;
    }
}


/**
 * @brief       Calculates the CAN controller timing values for specific baudrates.
 *
 * @param[in]   psCanfd             Pointer to CAN FD configuration structure.
 * @param[in]   u32NominalBaudRate  The nominal speed in bps.
 * @param[in]   u32DataBaudRate     The data speed in bps. Zero to disable baudrate switching.
 * @param[in]   u32SourceClock_Hz   CAN FD Protocol Engine clock source frequency in Hz.
 * @param[in]   psConfig            Passed is a configuration structure, on return the configuration is stored in the structure
 *
 * @return      true if timing configuration found, false if failed to find configuration.
 *
 * @details     Calculates the CAN controller timing values for specific baudrates.
 */
static uint32_t CANFD_CalculateTimingValues(const CANFD_T *psCanfd, uint32_t u32NominalBaudRate, uint32_t u32DataBaudRate, uint32_t u32SourceClock_Hz, CANFD_TIMEING_CONFIG_T *psConfig)
{
    uint32_t u32Ntq;
    uint32_t u32Dtq;
    uint32_t u32NominalBaudRateTemp = u32NominalBaudRate;

    /* observe baud rate maximums */
    if (u32NominalBaudRateTemp > MAX_NOMINAL_BAUDRATE)
    {
        u32NominalBaudRateTemp = MAX_NOMINAL_BAUDRATE;
    }

    for (u32Ntq = (uint32_t)MAX_TIME_QUANTA; u32Ntq >= (uint32_t)MIN_TIME_QUANTA; u32Ntq--)
    {
        uint32_t u32Nclk = u32NominalBaudRateTemp * u32Ntq;

        for (psConfig->u16NominalPrescaler = 0x001UL; psConfig->u16NominalPrescaler <= 0x400UL; (psConfig->u16NominalPrescaler)++)
        {
            uint32_t u32Nclk2;

            u32Nclk2 = u32Nclk * psConfig->u16NominalPrescaler;

            if (((u32SourceClock_Hz / u32Nclk2) <= 256UL) && ((u32SourceClock_Hz % u32Nclk2) == 0UL))
            {
                psConfig->u8PreDivider = u32SourceClock_Hz / u32Nclk2;

                /*FD Operation enabled*/
                if (psCanfd->CCCR & CANFD_CCCR_FDOE_Msk)
                {
                    /* if not using baudrate switch then we are done */
                    if (!u32DataBaudRate)
                    {
                        u32Dtq = 0;
                        psConfig->u8DataPrescaler = 0;
                        CANFD_GetSegments(u32NominalBaudRateTemp, u32DataBaudRate, u32Ntq, u32Dtq, psConfig);
                        return TRUE;
                    }

                    /* if baudrates are the same and the solution for nominal will work for
                    data, then use the nominal settings for both */
                    if ((u32DataBaudRate == u32NominalBaudRateTemp) && (psConfig->u16NominalPrescaler <= 0x20UL))
                    {
                        u32Dtq = u32Ntq;
                        psConfig->u8DataPrescaler = (uint8_t)psConfig->u16NominalPrescaler;
                        CANFD_GetSegments(u32NominalBaudRateTemp, u32DataBaudRate, u32Ntq, u32Dtq, psConfig);
                        return TRUE;
                    }

                    /* calculate data settings */
                    for (u32Dtq = (uint32_t)MAX_TIME_QUANTA; u32Dtq >= (uint32_t)MIN_TIME_QUANTA; u32Dtq--)
                    {
                        uint32_t u32Dclk;

                        u32Dclk = u32DataBaudRate * u32Dtq;

                        for (psConfig->u8DataPrescaler = 0x01UL; psConfig->u8DataPrescaler <= 0x20UL; (psConfig->u8DataPrescaler)++)
                        {
                            uint32_t u32Dclk2;

                            u32Dclk2 = u32Dclk * psConfig->u8DataPrescaler;

                            if (u32SourceClock_Hz == ((uint32_t)u32Dclk2 * psConfig->u8PreDivider))
                            {
                                CANFD_GetSegments(u32NominalBaudRateTemp, u32DataBaudRate, u32Ntq, u32Dtq, psConfig);
                                return TRUE;
                            }
                        }
                    }
                }
                else
                {
                    CANFD_GetSegments(u32NominalBaudRateTemp, 0, u32Ntq, 0, psConfig);
                    return TRUE;
                }
            }
        }
    }

    /* failed to find solution */
    return 0;
}


/**
 * @brief       Config message ram and Set bit-time.
 *
 * @param[in]   psCanfd     The pointer to CAN FD module base address.
 * @param[in]   psCanfdStr  message ram setting and bit-time setting
 *
 * @return      None.
 *
 * @details     Converts a Data Length Code into a number of message bytes.
 */
void CANFD_Open(CANFD_T *psCanfd, CANFD_FD_T *psCanfdStr)
{
    uint32_t u32CanFdClock = 0;
    /* Unlock protected registers */
    SYS_UnlockReg();

    if (psCanfd == (CANFD_T *)CANFD0)
    {
        CLK_EnableModuleClock(CANFD0_MODULE);
        SYS_ResetModule(SYS_CANFD0RST);

        if ((CLK->CANFDSEL & CLK_CANFDSEL_CANFD0SEL_Msk) == CLK_CANFDSEL_CANFD0SEL_HXT)
        {
            u32CanFdClock = CLK_GetHXTFreq();
        }
        else if ((CLK->CANFDSEL & CLK_CANFDSEL_CANFD0SEL_Msk) == CLK_CANFDSEL_CANFD0SEL_APLL0_DIV2)
        {
            u32CanFdClock = CLK_GetAPLL0ClockFreq() / 2UL;
        }
        else if ((CLK->CANFDSEL & CLK_CANFDSEL_CANFD0SEL_Msk) == CLK_CANFDSEL_CANFD0SEL_HCLK0)
        {
            u32CanFdClock = CLK_GetHCLK0Freq();
        }
        else if ((CLK->CANFDSEL & CLK_CANFDSEL_CANFD0SEL_Msk) == CLK_CANFDSEL_CANFD0SEL_HIRC)
        {
            u32CanFdClock = __HIRC;
        }
        else
        {
            u32CanFdClock = __HIRC48M / 4UL;
        }

    }
    else
    {
        CLK_EnableModuleClock(CANFD1_MODULE);
        SYS_ResetModule(SYS_CANFD1RST);

        if ((CLK->CANFDSEL & CLK_CANFDSEL_CANFD1SEL_Msk) == CLK_CANFDSEL_CANFD1SEL_HXT)
        {
            u32CanFdClock = __HXT;
        }
        else if ((CLK->CANFDSEL & CLK_CANFDSEL_CANFD1SEL_Msk) == CLK_CANFDSEL_CANFD1SEL_APLL0_DIV2)
        {
            u32CanFdClock = CLK_GetAPLL0ClockFreq() / 2UL;
        }
        else if ((CLK->CANFDSEL & CLK_CANFDSEL_CANFD1SEL_Msk) == CLK_CANFDSEL_CANFD1SEL_HCLK0)
        {
            u32CanFdClock = CLK_GetHCLK0Freq();
        }
        else if ((CLK->CANFDSEL & CLK_CANFDSEL_CANFD1SEL_Msk) == CLK_CANFDSEL_CANFD1SEL_HIRC)
        {
            u32CanFdClock = __HIRC;
        }
        else
        {
            u32CanFdClock = __HIRC48M / 4UL;
        }
    }

    /* Lock protected registers */
    SYS_LockReg();

    /* configuration change enable */
    psCanfd->CCCR |= CANFD_CCCR_CCE_Msk;

    if (psCanfdStr->sBtConfig.bBitRateSwitch)
    {
        /* enable FD and baud-rate switching */
        psCanfd->CCCR |= CANFD_CCCR_BRSE_Msk;
    }

    if (psCanfdStr->sBtConfig.bFDEn)
    {
        /*FD Operation enabled*/
        psCanfd->CCCR |= CANFD_CCCR_FDOE_Msk;
    }

    /* Clear the Rx Fifo0 element setting */
    psCanfd->RXF0C = 0;
    /* Clear the Rx Fifo1 element setting */
    psCanfd->RXF1C = 0;

    /* calculate and apply timing */
    if (CANFD_CalculateTimingValues(psCanfd, psCanfdStr->sBtConfig.sNormBitRate.u32BitRate, psCanfdStr->sBtConfig.sDataBitRate.u32BitRate,
                                    u32CanFdClock, &psCanfdStr->sBtConfig.sConfigBitTing))
    {
        CANFD_SetTimingConfig(psCanfd, &psCanfdStr->sBtConfig.sConfigBitTing);

    }

    /* Configures the Standard ID Filter element */
    if (psCanfdStr->sElemSize.u32SIDFC != 0UL)
    {
        CANFD_ConfigSIDFC(psCanfd, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize);
    }

    /*Configures the Extended ID Filter element */
    if (psCanfdStr->sElemSize.u32XIDFC != 0UL)
    {
        CANFD_ConfigXIDFC(psCanfd, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize);
    }

    /*Configures the Tx Buffer element */
    if ((psCanfdStr->sElemSize.u32TxBuf != 0UL) || (psCanfdStr->sElemSize.u32TxFifoQueue != 0UL))
    {
        CANFD_InitTxDBuf(psCanfd, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize, eCANFD_BYTE64);
    }

    /*Configures the Rx Buffer element */
    if (psCanfdStr->sElemSize.u32RxBuf != 0UL)
    {
        CANFD_InitRxDBuf(psCanfd, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize, eCANFD_BYTE64);
    }

    /*Configures the Rx Fifo0 element */
    if (psCanfdStr->sElemSize.u32RxFifo0 != 0UL)
    {
        CANFD_InitRxFifo(psCanfd, 0, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize, 0, eCANFD_BYTE64);
    }

    /*Configures the Rx Fifo1 element */
    if (psCanfdStr->sElemSize.u32RxFifo1 != 0UL)
    {
        CANFD_InitRxFifo(psCanfd, 1, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize, 0, eCANFD_BYTE64);
    }

    /*Configures the Tx Event FIFO element */
    if (psCanfdStr->sElemSize.u32TxEventFifo != 0UL)
    {
        CANFD_InitTxEvntFifo(psCanfd, &psCanfdStr->sMRamStartAddr, &psCanfdStr->sElemSize, 0);
    }

    /*Reject all Non-matching Frames Extended ID and Frames Standard ID,Reject all remote frames with 11-bit standard IDs and 29-bit extended IDs */
    CANFD_SetGFC(psCanfd, eCANFD_REJ_NON_MATCH_FRM, eCANFD_REJ_NON_MATCH_FRM, 1, 1);

    if (psCanfdStr->sBtConfig.bEnableLoopBack)
    {
        psCanfd->CCCR |= CANFD_CCCR_TEST_Msk;
        psCanfd->TEST |= CANFD_TEST_LBCK_Msk;
    }

}


/**
 * @brief       Close the CAN FD Bus.
 *
 * @param[in]   psCanfd   The pointer to CANFD module base address.
 *
 * @return      None.
 *
 * @details     Disable the CAN FD clock and Interrupt.
 */
void CANFD_Close(const CANFD_T *psCanfd)
{
    if (psCanfd == (CANFD_T *)CANFD0)
    {
        CLK_DisableModuleClock(CANFD0_MODULE);
        NVIC_DisableIRQ(CANFD00_IRQn);
        NVIC_DisableIRQ(CANFD01_IRQn);
    }
    else
    {
        CLK_DisableModuleClock(CANFD1_MODULE);
        NVIC_DisableIRQ(CANFD10_IRQn);
        NVIC_DisableIRQ(CANFD11_IRQn);
    }
}

/**
 * @brief       Get the element's address when read transmit buffer.
 *
 * @param[in]   psCanfd      The pointer of the specified CAN FD module.
 * @param[in]   u32Idx       The number of the transmit buffer element
 *
 * @return      Address of the element in transmit buffer.
 *
 * @details     The function is used to get the element's address when read transmit buffer.
 */
static uint32_t CANFD_GetTxEventFifoElementAddress(const CANFD_T *psCanfd, uint32_t u32Idx)
{
    return (psCanfd->TXEFC & CANFD_TXEFC_EFSA_Msk) + (u32Idx  * 8U);
}


/**
 * @brief       Enables CAN FD interrupts according to provided mask .
 *
 * @param[in]   psCanfd          The pointer of the specified CAN FD module.
 * @param[in]   u32IntLine0      The Interrupt Line 0 type select.
 * @param[in]   u32IntLine1      The Interrupt Line 1 type select.
 *                              - \ref CANFD_IE_ARAE_Msk     : Access to Reserved Address Interrupt
 *                              - \ref CANFD_IE_PEDE_Msk     : Protocol Error in Data Phase Interrupt
 *                              - \ref CANFD_IE_PEAE_Msk     : Protocol Error in Arbitration Phase Interrupt
 *                              - \ref CANFD_IE_WDIE_Msk     : Watchdog Interrupt
 *                              - \ref CANFD_IE_BOE_Msk      : Bus_Off Status Interrupt
 *                              - \ref CANFD_IE_EWE_Msk      : Warning Status Interrupt
 *                              - \ref CANFD_IE_EPE_Msk      : Error Passive Interrupt
 *                              - \ref CANFD_IE_ELOE_Msk     : Error Logging Overflow Interrupt
 *                              - \ref CANFD_IE_BEUE_Msk     : Bit Error Uncorrected Interrupt
 *                              - \ref CANFD_IE_BECE_Msk     : Bit Error Corrected Interrupt
 *                              - \ref CANFD_IE_DRXE_Msk     : Message stored to Dedicated Rx Buffer Interrupt
 *                              - \ref CANFD_IE_TOOE_Msk     : Timeout Occurred Interrupt
 *                              - \ref CANFD_IE_MRAFE_Msk    : Message RAM Access Failure Interrupt
 *                              - \ref CANFD_IE_TSWE_Msk     : Timestamp Wraparound Interrupt
 *                              - \ref CANFD_IE_TEFLE_Msk    : Tx Event FIFO Event Lost Interrupt
 *                              - \ref CANFD_IE_TEFFE_Msk    : Tx Event FIFO Full Interrupt
 *                              - \ref CANFD_IE_TEFWE_Msk    : Tx Event FIFO Watermark Reached Interrupt
 *                              - \ref CANFD_IE_TEFNE_Msk    : Tx Event FIFO New Entry Interrupt
 *                              - \ref CANFD_IE_TFEE_Msk     : Tx FIFO Empty Interrupt
 *                              - \ref CANFD_IE_TCFE_Msk     : Transmission Cancellation Finished Interrupt
 *                              - \ref CANFD_IE_TCE_Msk      : Transmission Completed Interrupt
 *                              - \ref CANFD_IE_HPME_Msk     : High Priority Message Interrupt
 *                              - \ref CANFD_IE_RF1LE_Msk    : Rx FIFO 1 Message Lost Interrupt
 *                              - \ref CANFD_IE_RF1FE_Msk    : Rx FIFO 1 Full Interrupt
 *                              - \ref CANFD_IE_RF1WE_Msk    : Rx FIFO 1 Watermark Reached Interrupt
 *                              - \ref CANFD_IE_RF1NE_Msk    : Rx FIFO 1 New Message Interrupt
 *                              - \ref CANFD_IE_RF0LE_Msk    : Rx FIFO 0 Message Lost Interrupt
 *                              - \ref CANFD_IE_RF0FE_Msk    : Rx FIFO 0 Full Interrupt
 *                              - \ref CANFD_IE_RF0WE_Msk    : Rx FIFO 0 Watermark Reached Interrupt
 *                              - \ref CANFD_IE_RF0NE_Msk    : Rx FIFO 0 New Message Interrupt
 *
 * @param[in]   u32TXBTIE        Enable Tx Buffer Transmission 0-31 Interrupt.
 * @param[in]   u32TXBCIE        Enable Tx Buffer Cancellation Finished 0-31 Interrupt.
 * @return      None.
 *
 * @details     This macro enable specified CAN FD interrupt.
 */
void CANFD_EnableInt(CANFD_T *psCanfd, uint32_t u32IntLine0, uint32_t u32IntLine1, uint32_t u32TXBTIE, uint32_t u32TXBCIE)
{

    if (u32IntLine0 != 0UL)
    {
        /*Setting the CANFD0_IRQ0 Interrupt*/
        psCanfd->IE |= u32IntLine0;
        /* Enable CAN FD specified interrupt */
        psCanfd->ILE |= ((uint32_t)1U << 0);
    }

    if (u32IntLine1 != 0UL)
    {
        /*Setting the CANFD0_IRQ1 Interrupt*/
        psCanfd->ILS |= u32IntLine1;
        /* Enable CAN FD specified interrupt */
        psCanfd->ILE |= ((uint32_t)1U << 1);
    }

    /*Setting the Tx Buffer Transmission Interrupt Enable*/
    psCanfd->TXBTIE |= u32TXBTIE;

    /*Tx Buffer Cancellation Finished Interrupt Enable*/
    psCanfd->TXBCIE |= u32TXBCIE;
}


/**
 * @brief       Disables CAN FD interrupts according to provided mask .
 *
 * @param[in]   psCanfd          The pointer of the specified CAN FD module.
 * @param[in]   u32IntLine0      The Interrupt Line 0 type select.
 * @param[in]   u32IntLine1      The Interrupt Line 1 type select.
 *                              - \ref CANFD_IE_ARAE_Msk     : Access to Reserved Address Interrupt
 *                              - \ref CANFD_IE_PEDE_Msk     : Protocol Error in Data Phase Interrupt
 *                              - \ref CANFD_IE_PEAE_Msk     : Protocol Error in Arbitration Phase Interrupt
 *                              - \ref CANFD_IE_WDIE_Msk     : Watchdog Interrupt
 *                              - \ref CANFD_IE_BOE_Msk      : Bus_Off Status Interrupt
 *                              - \ref CANFD_IE_EWE_Msk      : Warning Status Interrupt
 *                              - \ref CANFD_IE_EPE_Msk      : Error Passive Interrupt
 *                              - \ref CANFD_IE_ELOE_Msk     : Error Logging Overflow Interrupt
 *                              - \ref CANFD_IE_BEUE_Msk     : Bit Error Uncorrected Interrupt
 *                              - \ref CANFD_IE_BECE_Msk     : Bit Error Corrected Interrupt
 *                              - \ref CANFD_IE_DRXE_Msk     : Message stored to Dedicated Rx Buffer Interrupt
 *                              - \ref CANFD_IE_TOOE_Msk     : Timeout Occurred Interrupt
 *                              - \ref CANFD_IE_MRAFE_Msk    : Message RAM Access Failure Interrupt
 *                              - \ref CANFD_IE_TSWE_Msk     : Timestamp Wraparound Interrupt
 *                              - \ref CANFD_IE_TEFLE_Msk    : Tx Event FIFO Event Lost Interrupt
 *                              - \ref CANFD_IE_TEFFE_Msk    : Tx Event FIFO Full Interrupt
 *                              - \ref CANFD_IE_TEFWE_Msk    : Tx Event FIFO Watermark Reached Interrupt
 *                              - \ref CANFD_IE_TEFNE_Msk    : Tx Event FIFO New Entry Interrupt
 *                              - \ref CANFD_IE_TFEE_Msk     : Tx FIFO Empty Interrupt
 *                              - \ref CANFD_IE_TCFE_Msk     : Transmission Cancellation Finished Interrupt
 *                              - \ref CANFD_IE_TCE_Msk      : Transmission Completed Interrupt
 *                              - \ref CANFD_IE_HPME_Msk     : High Priority Message Interrupt
 *                              - \ref CANFD_IE_RF1LE_Msk    : Rx FIFO 1 Message Lost Interrupt
 *                              - \ref CANFD_IE_RF1FE_Msk    : Rx FIFO 1 Full Interrupt
 *                              - \ref CANFD_IE_RF1WE_Msk    : Rx FIFO 1 Watermark Reached Interrupt
 *                              - \ref CANFD_IE_RF1NE_Msk    : Rx FIFO 1 New Message Interrupt
 *                              - \ref CANFD_IE_RF0LE_Msk    : Rx FIFO 0 Message Lost Interrupt
 *                              - \ref CANFD_IE_RF0FE_Msk    : Rx FIFO 0 Full Interrupt
 *                              - \ref CANFD_IE_RF0WE_Msk    : Rx FIFO 0 Watermark Reached Interrupt
 *                              - \ref CANFD_IE_RF0NE_Msk    : Rx FIFO 0 New Message Interrupt
 *
 * @param[in]   u32TXBTIE        Disable Tx Buffer Transmission 0-31 Interrupt.
 * @param[in]   u32TXBCIE        Disable Tx Buffer Cancellation Finished 0-31 Interrupt.
 * @return      None.
 *
 * @details     This macro disable specified CAN FD interrupt.
 */
void CANFD_DisableInt(CANFD_T *psCanfd, uint32_t u32IntLine0, uint32_t u32IntLine1, uint32_t u32TXBTIE, uint32_t u32TXBCIE)
{
    if (u32IntLine0 != 0UL)
    {
        /*Clear the CANFD0_IRQ0 Interrupt*/
        psCanfd->IE &= ~u32IntLine0;
        /* Disable CAN FD specified interrupt */
        psCanfd->ILE &= ~((uint32_t)1U << 0);
    }

    if (u32IntLine1 != 0UL)
    {
        /*Clear the CANFD0_IRQ1 Interrupt*/
        psCanfd->ILS &= ~u32IntLine1;
        /* Disable CAN FD specified interrupt */
        psCanfd->ILE &= ~((uint32_t)1U << 1);
    }

    /*Setting the Tx Buffer Transmission Interrupt Disable*/
    psCanfd->TXBTIE &= ~u32TXBTIE;

    /*Tx Buffer Cancellation Finished Interrupt Disable*/
    psCanfd->TXBCIE &= ~u32TXBCIE;
}


/**
 * @brief       Copy Tx Message to TX buffer and Request transmission.
 *
 * @param[in]   psCanfd         The pointer to CAN FD module base address.
 * @param[in]   u32TxBufIdx     The Message Buffer index.
 * @param[in]   psTxMsg         Message to be copied.
 *
 * @return      number of tx requests set: 0 = Tx Message Buffer is currently in use.
 *                                         1 = Write Tx Message Buffer Successfully.
 *                                         2 = Tx message buffer transfer request timeout.
 * @details     Copy Tx Message to FIFO/Queue TX buffer and Request transmission.
 */
uint32_t CANFD_TransmitTxMsg(CANFD_T *psCanfd, uint32_t u32TxBufIdx, CANFD_FD_MSG_T *psTxMsg)
{
    uint32_t u32Success = 0;
    uint32_t u32TimeOutCount = SystemCoreClock; // 1 second timeout
    /* write the message to the message buffer */
    u32Success = CANFD_TransmitDMsg(psCanfd, u32TxBufIdx, psTxMsg);

    if (u32Success == 1UL)
    {
        /* wait for completion */
        while ((psCanfd->TXBRP & (1UL << u32TxBufIdx)))
        {
            if (u32TimeOutCount == 0UL)
            {
                return 2;
            }

            u32TimeOutCount--;
        }
    }

    return u32Success;
}


/**
 * @brief       Writes a Tx Message to Transmit Message Buffer.
 *
 * @param[in]   psCanfd        The pointer of the specified CAN FD module.
 * @param[in]   u32TxBufIdx    The Message Buffer index.
 * @param[in]   psTxMsg        Pointer to CAN FD message frame to be sent.
 *
 * @return      1  Write Tx Message Buffer Successfully.
 *              0  Tx Message Buffer is currently in use.
 *
 * @details     This function writes a CANFD Message to the specified Transmit Message Buffer
 *              and changes the Message Buffer state to start CANFD Message transmit. After
 *              that the function returns immediately.
 */
uint32_t CANFD_TransmitDMsg(CANFD_T *psCanfd, uint32_t u32TxBufIdx, CANFD_FD_MSG_T *psTxMsg)
{

    uint32_t u32Success = 1;

    if (u32TxBufIdx >= CANFD_MAX_TX_BUF_ELEMS)
    {
        return 0;
    }

    /* transmission is pending in this message buffer */
    if ((psCanfd->TXBRP & (1UL << u32TxBufIdx)))
    {
        return 0;
    }

    CANFD_CopyDataToTransmitBuffer(psCanfd, u32TxBufIdx, psTxMsg);

    psCanfd->TXBAR = (1UL << u32TxBufIdx);

    return u32Success;
}

/**
 * @brief       Copies a message from a dedicated Tx frame into a Transmit message buffer.
 *
 * @param[in]   psCanfd         The pointer of the specified CAN FD module.
 * @param[in]   u32TxBufIdx     The Message Buffer index.
 * @param[in]   psTxMsg         Pointer to CAN FD message frame to be sent.
 *
 * @return      None.
 *
 * @details     Copies a message from a dedicated Tx frame into a Transmit message buffer.
 */
void CANFD_CopyDataToTransmitBuffer(const CANFD_T *psCanfd, uint32_t u32TxBufIdx, CANFD_FD_MSG_T *psTxMsg)
{
    uint32_t u32Idx = 0;
    CANFD_BUF_T *psTxBuffer;

    psTxBuffer = (CANFD_BUF_T *)(CANFD_SRAM_BASE_ADDR(psCanfd) + (psCanfd->TXBC & 0xFFFFUL) + (u32TxBufIdx * sizeof(CANFD_BUF_T)));

    if (psTxMsg->eIdType == eCANFD_XID)
    {
        psTxBuffer->u32Id = TX_BUFFER_T0_ELEM_XTD_Msk | (psTxMsg->u32Id & 0x1FFFFFFFUL);
    }
    else
    {
        psTxBuffer->u32Id = (psTxMsg->u32Id & 0x7FFUL) << 18UL;
    }

    /*ESI bit in CAN FD format transmitted recessive*/
    if (psTxMsg->bErrStaInd == 1UL)
    {
        psTxBuffer->u32Id |= TX_BUFFER_T0_ELEM_ESI_Msk;
    }

    if (psTxMsg->eFrmType == eCANFD_REMOTE_FRM)
    {
        psTxBuffer->u32Id |= TX_BUFFER_T0_ELEM_RTR_Msk;
    }

    psTxBuffer->u32Config = (CANFD_EncodeDLC(psTxMsg->u32DLC) << 16);

    if (psTxMsg->bFDFormat)
    {
        psTxBuffer->u32Config |= TX_BUFFER_T1_ELEM_FDF_Msk;
    }

    if (psTxMsg->bBitRateSwitch)
    {
        psTxBuffer->u32Config |= TX_BUFFER_T1_ELEM_BSR_Msk;
    }

    if (psTxMsg->bEvntFifoCon)
    {
        psTxBuffer->u32Config = (psTxBuffer->u32Config & (~TX_BUFFER_T1_ELEM_MM1_Msk));
        psTxBuffer->u32Config |= (psTxMsg->u8MsgMarker << TX_BUFFER_T1_ELEM_MM1_Pos);
        psTxBuffer->u32Config |= TX_BUFFER_T1_ELEM_EFC_Msk;
    }

    for (u32Idx = 0; u32Idx < (psTxMsg->u32DLC + (3UL)) / 4UL; u32Idx++)
    {
        psTxBuffer->au32Data[u32Idx] = psTxMsg->au32Data[u32Idx];
    }

}


/**
 * @brief       Global Filter Configuration (GFC).
 *
 * @param[in]   psCanfd          The pointer to CAN FD module base address.
 * @param[in]   eNMStdFrm        Accept/Reject Non-Matching Standard(11-bits) Frames.
 * @param[in]   eEMExtFrm        Accept/Reject Non-Matching Extended(29-bits) Frames.
 * @param[in]   u32RejRmtStdFrm  Reject/Filter Remote Standard Frames.
 * @param[in]   u32RejRmtExtFrm  Reject/Filter Remote Extended Frames.
 *
 * @return      None.
 *
 * @details     Global Filter Configuration.
 */
void CANFD_SetGFC(CANFD_T *psCanfd, E_CANFD_ACC_NON_MATCH_FRM eNMStdFrm, E_CANFD_ACC_NON_MATCH_FRM eEMExtFrm, uint32_t u32RejRmtStdFrm, uint32_t u32RejRmtExtFrm)
{
    psCanfd->GFC = ((uint32_t)eNMStdFrm << CANFD_GFC_ANFS_Pos) | ((uint32_t)eEMExtFrm << CANFD_GFC_ANFE_Pos)
                   | (u32RejRmtStdFrm << CANFD_GFC_RRFS_Pos) | (u32RejRmtExtFrm << CANFD_GFC_RRFE_Pos);
}


/**
 * @brief       Rx FIFO Configuration for RX_FIFO_0 and RX_FIFO_1.
 *
 * @param[in]   psCanfd          The pointer to CAN FD module base address.
 * @param[in]   u32RxFifoNum     0: RX FIFO_0, 1: RX_FIFO_1.
 * @param[in]   psRamConfig      Rx FIFO Size in number of configuration ram address.
 * @param[in]   psElemSize       Rx FIFO Size in number of Rx FIFO elements (element number (max. = 64)).
 * @param[in]   u32FifoWM        Watermark in number of Rx FIFO elements
 * @param[in]   eFifoSize        Maximum data field size that should be stored in this Rx FIFO
 *                               (configure BYTE64 if you are unsure, as this is the largest data field allowed in CAN FD)
 *
 * @return      None.
 *
 * @details     Rx FIFO Configuration for RX_FIFO_0 and RX_FIFO_1.
 */
void CANFD_InitRxFifo(CANFD_T *psCanfd, uint32_t u32RxFifoNum, CANFD_RAM_PART_T *psRamConfig, const CANFD_ELEM_SIZE_T *psElemSize, uint32_t u32FifoWM, E_CANFD_DATA_FIELD_SIZE eFifoSize)
{
    uint32_t u32Address;
    uint32_t u32Size;

    /* ignore if index is too high */
    if (u32RxFifoNum > (uint32_t)CANFD_NUM_RX_FIFOS)
    {
        return;
    }

    /* ignore if index is too high */
    if (psElemSize-> u32RxFifo0 > CANFD_MAX_RX_FIFO0_ELEMS)
    {
        return;
    }

    /* ignore if index is too high */
    if (psElemSize-> u32RxFifo1 > CANFD_MAX_RX_FIFO1_ELEMS)
    {
        return;
    }

    switch (u32RxFifoNum)
    {
        case 0:
            if (psElemSize-> u32RxFifo0)
            {
                /* set size of Rx FIFO 0, set offset, blocking mode */
                psCanfd->RXF0C = (psRamConfig->u32RXF0C_F0SA) | (psElemSize->u32RxFifo0 << CANFD_RXF0C_F0S_Pos)
                                 | (u32FifoWM << CANFD_RXF0C_F0WM_Pos);
                psCanfd->RXESC = (psCanfd->RXESC & (~CANFD_RXESC_F0DS_Msk)) | ((uint32_t)eFifoSize << CANFD_RXESC_F0DS_Pos);
                /*Get the RX FIFO 0 Start Address in the RAM*/
                u32Address = CANFD_SRAM_BASE_ADDR(psCanfd) + (psRamConfig->u32RXF0C_F0SA & CANFD_RXF0C_F0SA_Msk);
                u32Size = eFifoSize;

                if (u32Size < 5UL)
                {
                    u32Size += 4UL;
                }
                else
                {
                    u32Size = (u32Size * 4UL) - 10UL;
                }

                /*Clear the RX FIFO 0 Memory*/
                (void)memset((uint32_t *)(u32Address), 0x00, (u32Size * 4UL * psElemSize->u32RxFifo0));
            }
            else
            {
                psCanfd->RXF0C = 0;
            }

            break;

        case 1:
            if (psElemSize-> u32RxFifo1)
            {

                /* set size of Rx FIFO 1, set offset, blocking mode */
                psCanfd->RXF1C = (psRamConfig->u32RXF1C_F1SA) | (psElemSize->u32RxFifo1 << CANFD_RXF1C_F1S_Pos)
                                 | (u32FifoWM << CANFD_RXF1C_F1WM_Pos);
                psCanfd->RXESC = (psCanfd->RXESC & (~CANFD_RXESC_F1DS_Msk)) | ((uint32_t)eFifoSize << CANFD_RXESC_F1DS_Pos);
                /*Get the RX FIFO 1 Start Address in the RAM*/
                u32Address = CANFD_SRAM_BASE_ADDR(psCanfd) + (psRamConfig->u32RXF1C_F1SA & CANFD_RXF1C_F1SA_Msk);

                u32Size = eFifoSize;

                if (u32Size < 5UL)
                {
                    u32Size += 4UL;
                }
                else
                {
                    u32Size = (u32Size * 4UL) - 10UL;
                }

                /*Clear the RX FIFO 0 Memory*/
                (void)memset((uint32_t *)(u32Address), 0x00, (u32Size * 4UL * psElemSize->u32RxFifo1));
            }
            else
            {
                psCanfd->RXF1C = 0;
            }

            break;

        default:
            break;
    }
}


/**
 * @brief       Function configures the data structures used by a dedicated Rx Buffer.
 *
 * @param[in]   psCanfd          The pointer to CAN FD module base address.
 * @param[in]   psRamConfig      Tx buffer configuration ram address.
 * @param[in]   psElemSize       Tx buffer configuration element size.
 * @param[in]   eTxBufSize       Maximum data field size that should be stored in a dedicated Tx Buffer
 *                              (configure BYTE64 if you are unsure, as this is the largest data field allowed in CAN FD)largest data field allowed in CAN FD)
 *
 * @return      None.
 *
 * @details     Function configures the data structures used by a dedicated Rx Buffer.
 */
void CANFD_InitTxDBuf(CANFD_T *psCanfd, const CANFD_RAM_PART_T *psRamConfig, CANFD_ELEM_SIZE_T *psElemSize, E_CANFD_DATA_FIELD_SIZE eTxBufSize)
{
    uint32_t u32Address;
    uint32_t u32Size;

    /*Note: Be aware that the sum of TFQS and NDTB may be not greater than 32. There is no check
            for erroneous configurations. The Tx Buffers section in the Message RAM starts with the
            dedicated Tx Buffers.*/
    if ((psElemSize->u32TxBuf + psElemSize->u32TxFifoQueue) > 32UL)
    {
        psElemSize->u32TxBuf = 16;
        psElemSize->u32TxFifoQueue = 16;
    }

    /*Setting the Tx Buffer Start Address*/
    psCanfd->TXBC = (((psElemSize->u32TxFifoQueue & 0x3FUL) << CANFD_TXBC_TFQS_Pos) | ((psElemSize->u32TxBuf & 0x3FUL) << CANFD_TXBC_NDTB_Pos) | (psRamConfig->u32TXBC_TBSA & CANFD_TXBC_TBSA_Msk));
    /*Get the TX Buffer Start Address in the RAM*/
    u32Address = CANFD_SRAM_BASE_ADDR(psCanfd) + (psRamConfig->u32TXBC_TBSA & CANFD_TXBC_TBSA_Msk);
    /*Setting the Tx Buffer Data Field Size*/
    psCanfd->TXESC = (psCanfd->TXESC & (~CANFD_TXESC_TBDS_Msk)) | ((uint32_t)eTxBufSize <<  CANFD_TXESC_TBDS_Pos);
    /*Get the Buffer Data Field Size*/
    u32Size = eTxBufSize;

    if (u32Size < 5UL)
    {
        u32Size += 4UL;
    }
    else
    {
        u32Size = (u32Size * 4UL) - 10UL;
    }

    /*Clear the TX Buffer Memory*/
    (void)memset((uint32_t *)(u32Address), 0x00, (u32Size * 4UL * psElemSize->u32TxBuf));
}


/**
 * @brief       Function configures the data structures used by a dedicated Rx Buffer.
 *
 * @param[in]   psCanfd          The pointer to CAN FD module base address.
 * @param[in]   psRamConfig      Rx buffer configuration ram address.
 * @param[in]   psElemSize       Rx buffer configuration element size.
 * @param[in]   eRxBufSize       Maximum data field size that should be stored in a dedicated Rx Buffer
 *                              (configure BYTE64 if you are unsure, as this is the largest data field allowed in CAN FD)largest data field allowed in CAN FD)
 *
 * @return      None.
 *
 * @details     Function configures the data structures used by a dedicated Rx Buffer.
 */
void CANFD_InitRxDBuf(CANFD_T *psCanfd, const CANFD_RAM_PART_T *psRamConfig, const CANFD_ELEM_SIZE_T *psElemSize, E_CANFD_DATA_FIELD_SIZE eRxBufSize)
{
    uint32_t u32Address;
    uint32_t u32Size;
    /*Setting the Rx Buffer Start Address*/
    psCanfd->RXBC = (psRamConfig->u32RXBC_RBSA & CANFD_RXBC_RBSA_Msk);
    /*Get the RX Buffer Start Address in the RAM*/
    u32Address = CANFD_SRAM_BASE_ADDR(psCanfd) + (psRamConfig->u32RXBC_RBSA & CANFD_RXBC_RBSA_Msk);
    /*Setting the Rx Buffer Data Field Size*/
    psCanfd->RXESC = (psCanfd->RXESC & (~CANFD_RXESC_RBDS_Msk)) | ((uint32_t)eRxBufSize <<  CANFD_RXESC_RBDS_Pos);
    /*Get the Buffer Data Field Size*/
    u32Size = eRxBufSize;

    if (u32Size < 5UL)
    {
        u32Size += 4UL;
    }
    else
    {
        u32Size = (u32Size * 4UL) - 10UL;
    }

    /*Clear the RX Buffer Memory*/
    (void)memset((uint32_t *)(u32Address), 0x00, (u32Size * 4UL * psElemSize->u32RxBuf));
}


/**
 * @brief       Configures the register SIDFC for the 11-bit Standard Message ID Filter elements.
 *
 * @param[in]   psCanfd           The pointer to CAN FD module base address.
 * @param[in]   psRamConfig       Standard ID filter configuration ram address
 * @param[in]   psElemSize        Standard ID filter configuration element size
 *
 * @return      None.
 *
 * @details     Function configures the data structures used by a dedicated Rx Buffer.
 */
void CANFD_ConfigSIDFC(CANFD_T *psCanfd, CANFD_RAM_PART_T *psRamConfig, const CANFD_ELEM_SIZE_T *psElemSize)
{
    uint32_t u32Address;
    /*Setting the Filter List Standard Start Address and List Size  */
    psCanfd->SIDFC = ((psElemSize->u32SIDFC & 0xFFUL) << CANFD_SIDFC_LSS_Pos) | (psRamConfig->u32SIDFC_FLSSA & CANFD_SIDFC_FLSSA_Msk);
    /*Get the Filter List Standard Start Address in the RAM*/
    u32Address = CANFD_SRAM_BASE_ADDR(psCanfd) + (psRamConfig->u32SIDFC_FLSSA & CANFD_SIDFC_FLSSA_Msk);
    /*Clear the Filter List Memory*/
    (void)memset((uint32_t *)(u32Address), 0x00, (psElemSize->u32SIDFC * 4UL));
}


/**
 * @brief       Configures the register XIDFC for the 29-bit Extended Message ID Filter elements.
 *
 * @param[in]   psCanfd           The pointer to CAN FD module base address.
 * @param[in]   psRamConfig       Extended ID filter configuration ram address
 * @param[in]   psElemSize        Extended ID filter configuration element size
 *
 * @return      None.
 *
 * @details     Configures the register XIDFC for the 29-bit Extended Message ID Filter elements.
 */
void CANFD_ConfigXIDFC(CANFD_T *psCanfd, CANFD_RAM_PART_T *psRamConfig, const CANFD_ELEM_SIZE_T *psElemSize)
{
    uint32_t u32Address;
    /*Setting the Filter List Extended Start Address and List Size  */
    psCanfd->XIDFC = ((psElemSize->u32XIDFC & 0xFFUL) << CANFD_XIDFC_LSE_Pos) | (psRamConfig->u32XIDFC_FLESA & CANFD_XIDFC_FLESA_Msk);
    /*Get the Filter List Standard Start Address in the RAM*/
    u32Address = CANFD_SRAM_BASE_ADDR(psCanfd) + (psRamConfig->u32XIDFC_FLESA & CANFD_XIDFC_FLESA_Msk);
    /*Clear the Filter List Memory*/
    (void)memset((uint32_t *)(u32Address), 0x00, (psElemSize->u32XIDFC * 8UL));
}


/**
 * @brief       Writes a 11-bit Standard ID filter element in the Message RAM.
 *
 * @param[in]   psCanfd          The pointer to CAN FD module base address.
 * @param[in]   u32FltrIdx       Index at which the filter element should be written in the '11-bit Filter' section of Message RAM
 * @param[in]   u32Filter        Rx Individual filter value.
 *
 * @return      None.
 *
 * @details     Writes a 11-bit Standard ID filter element in the Message RAM.
 */
void CANFD_SetSIDFltr(const CANFD_T *psCanfd, uint32_t u32FltrIdx, uint32_t u32Filter)
{
    CANFD_STD_FILTER_T *psFilter;

    /* ignore if index is too high */
    if (u32FltrIdx >= CANFD_MAX_11_BIT_FTR_ELEMS)
    {
        return;
    }

    /*Get the Filter List Configuration Address in the RAM*/
    psFilter = (CANFD_STD_FILTER_T *)(CANFD_SRAM_BASE_ADDR(psCanfd) + (psCanfd->SIDFC & CANFD_SIDFC_FLSSA_Msk) + (u32FltrIdx * sizeof(CANFD_STD_FILTER_T)));
    /*Wirted the Standard ID filter element to RAM */
    psFilter->VALUE = u32Filter;
}


/**
 * @brief       Writes a 29-bit extended id filter element in the Message RAM.
 *              Size of an Extended Id filter element is 2 words. So 2 words are written into the Message RAM for each filter element
 *
 * @param[in]   psCanfd         The pointer to CAN FD module base address.
 * @param[in]   u32FltrIdx      Index at which the filter element should be written in the '29-bit Filter' section of Message RAM.
 * @param[in]   u32FilterLow    Rx Individual filter low value.
 * @param[in]   u32FilterHigh   Rx Individual filter high value.
 *
 * @return      None.
 *
 * @details     Writes a 29-bit extended id filter element in the Message RAM.
 */
void CANFD_SetXIDFltr(const CANFD_T *psCanfd, uint32_t u32FltrIdx, uint32_t u32FilterLow, uint32_t u32FilterHigh)
{
    CANFD_EXT_FILTER_T *psFilter;

    /* ignore if index is too high */
    if (u32FltrIdx >= CANFD_MAX_29_BIT_FTR_ELEMS)
    {
        return;
    }

    /*Get the Filter List Configuration Address on RAM*/
    psFilter = (CANFD_EXT_FILTER_T *)(CANFD_SRAM_BASE_ADDR(psCanfd) + (psCanfd->XIDFC & CANFD_XIDFC_FLESA_Msk) + (u32FltrIdx * sizeof(CANFD_EXT_FILTER_T)));
    /*Wirted the Extended ID filter element to RAM */
    psFilter->LOWVALUE = u32FilterLow;
    psFilter->HIGHVALUE = u32FilterHigh;
}


/**
 * @brief       Reads a CAN FD Message from Receive Message Buffer.
 *
 * @param[in]   psCanfd     The pointer of the specified CAN FD module.
 * @param[in]   u8MbIdx     The CANFD Message Buffer index.
 * @param[in]   psMsgBuf    Pointer to CAN FD message frame structure for reception.
 *
 * @return       1:Rx Message Buffer is full and has been read successfully.
 *               0:Rx Message Buffer is empty.
 *
 * @details     This function reads a CAN message from a specified Receive Message Buffer.
 *              The function fills a receive CAN message frame structure with just received data
 *              and activates the Message Buffer again.The function returns immediately.
*/
uint32_t CANFD_ReadRxBufMsg(CANFD_T *psCanfd, uint8_t u8MbIdx, CANFD_FD_MSG_T *psMsgBuf)
{
    CANFD_BUF_T *psRxBuffer;
    uint32_t u32Success = 0;

    if (u8MbIdx < CANFD_MAX_RX_BUF_ELEMS)
    {
        uint32_t newData = 0;

        if (u8MbIdx < (uint8_t)32)
        {
            newData = (psCanfd->NDAT1 >> u8MbIdx) & 1UL;
        }
        else
        {
            newData = (psCanfd->NDAT2 >> (u8MbIdx - (uint8_t)32)) & 1UL;
        }

        /* new message is waiting to be read */
        if (newData)
        {
            /* get memory location of rx buffer */

            psRxBuffer = (CANFD_BUF_T *)(CANFD_SRAM_BASE_ADDR(psCanfd) + (psCanfd->RXBC & 0xFFFFUL) + (u8MbIdx * sizeof(CANFD_BUF_T)));
            /* read the message */
            CANFD_CopyDBufToMsgBuf(psCanfd, psRxBuffer, psMsgBuf);

            /* clear 'new data' flag */
            if (u8MbIdx < 32UL)
            {
                psCanfd->NDAT1 = psCanfd->NDAT1 | (1UL << u8MbIdx);
            }
            else
            {
                psCanfd->NDAT2 = psCanfd->NDAT2 | (1UL << ((uint32_t)u8MbIdx - 32UL));
            }

            u32Success = 1;
        }
    }

    return u32Success;
}


/**
 * @brief       Reads a CAN FD Message from Rx FIFO.
 *
 * @param[in]   psCanfd     The pointer of the specified CANFD module.
 * @param[in]   u8FifoIdx   Number of the FIFO, 0 or 1.
 * @param[in]   psMsgBuf    Pointer to CANFD message frame structure for reception.
 *
 * @return      0           Rx FIFO is empty.
 *              1           Read Message from Rx FIFO successfully.
 *              2           Rx FIFO is already overflowed and has been read successfully.
 *
 * @details     This function reads a CAN message from the CANFD build-in Rx FIFO.
 */
uint32_t CANFD_ReadRxFifoMsg(CANFD_T *psCanfd, uint8_t u8FifoIdx, CANFD_FD_MSG_T *psMsgBuf)
{
    CANFD_BUF_T *pRxBuffer;

    uint32_t u32Success = 0;

    /* check for valid FIFO number */
    if (u8FifoIdx < (uint8_t)CANFD_NUM_RX_FIFOS)
    {
        __I  uint32_t *pRXFS;
        __IO const uint32_t *pRXFC;
        __IO uint32_t *pRXFA;
        uint8_t msgLostBit;

        if (u8FifoIdx == 0UL)
        {
            pRXFS = &(psCanfd->RXF0S);
            pRXFC = &(psCanfd->RXF0C);
            pRXFA = &(psCanfd->RXF0A);
            msgLostBit = 3;
        }
        else
        {
            pRXFS = &(psCanfd->RXF1S);
            pRXFC = &(psCanfd->RXF1C);
            pRXFA = &(psCanfd->RXF1A);
            msgLostBit = 7;
        }

        /* if FIFO is not empty */

        if ((*pRXFS & 0x7F) > 0)
        {
            uint8_t GetIndex;

            GetIndex = (uint8_t)((*pRXFS >> 8UL) & 0x3FUL);
            pRxBuffer = (CANFD_BUF_T *)(CANFD_SRAM_BASE_ADDR(psCanfd) + (*pRXFC & 0xFFFF) + (GetIndex * sizeof(CANFD_BUF_T)));
            CANFD_CopyRxFifoToMsgBuf(psCanfd, pRxBuffer, psMsgBuf);
            /* we got the message */
            *pRXFA = GetIndex;

            /* check for overflow */
            if ((*pRXFS & CANFD_RXFS_RFL))
            {
                /* clear overflow flag */
                psCanfd->IR = (1UL << msgLostBit);
                u32Success = 2;
            }
            else
            {
                u32Success = 1;
            }
        }
    }

    return u32Success;
}


/**
 * @brief       Copies a message from a dedicated Rx buffer into a message buffer.
 *
 * @param[in]   psCanfd         The pointer to CANFD module base address.
 * @param[in]   psRxBuf         Buffer to read from.
 * @param[in]   psMsgBuf        Location to store read message.
 *
 * @return      None.
 *
 * @details     Copies a message from a dedicated Rx buffer into a message buffer.
 */
void CANFD_CopyDBufToMsgBuf(const CANFD_T *psCanfd, CANFD_BUF_T *psRxBuf, CANFD_FD_MSG_T *psMsgBuf)
{
    uint32_t u32Idx;

    if (psRxBuf->u32Id & RX_BUFFER_AND_FIFO_R0_ELEM_ESI_Msk)
    {
        psMsgBuf->bErrStaInd = TRUE;
    }
    else
    {
        psMsgBuf->bErrStaInd = FALSE;
    }

    /* if 29-bit ID */
    if (psRxBuf->u32Id & RX_BUFFER_AND_FIFO_R0_ELEM_XTD_Msk)
    {
        psMsgBuf->u32Id = (psRxBuf->u32Id & RX_BUFFER_AND_FIFO_R0_ELEM_ID_Msk);
        psMsgBuf->eIdType = eCANFD_XID;
    }
    /* if 11-bit ID */
    else
    {
        psMsgBuf->u32Id = (psRxBuf->u32Id  >> 18UL) & 0x7FFUL;
        psMsgBuf->eIdType = eCANFD_SID;
    }

    if (psRxBuf->u32Id  & RX_BUFFER_AND_FIFO_R0_ELEM_RTR_Msk)
    {
        psMsgBuf->eFrmType = eCANFD_REMOTE_FRM;
    }
    else
    {
        psMsgBuf->eFrmType = eCANFD_DATA_FRM;
    }

    if (psRxBuf->u32Config &  RX_BUFFER_AND_FIFO_R1_ELEM_ANMF_Msk)
    {
        psMsgBuf->bNonMatchingFrame = TRUE;
    }
    else
    {
        psMsgBuf->bNonMatchingFrame = TRUE;
        psMsgBuf->u8FilterIndex = ((psRxBuf->u32Config & RX_BUFFER_AND_FIFO_R1_ELEM_FIDX_Msk) >> RX_BUFFER_AND_FIFO_R1_ELEM_FIDX_Pos);
    }

    if (psRxBuf->u32Config &  RX_BUFFER_AND_FIFO_R1_ELEM_FDF_Msk)
    {
        psMsgBuf->bFDFormat = TRUE;
    }
    else
    {
        psMsgBuf->bFDFormat = FALSE;
    }

    if (psRxBuf->u32Config &  RX_BUFFER_AND_FIFO_R1_ELEM_BSR_Msk)
    {
        psMsgBuf->bBitRateSwitch = TRUE;
    }
    else
    {
        psMsgBuf->bBitRateSwitch = FALSE;
    }

    if (psCanfd->TSCC & CANFD_TSCC_TSS_Msk)
    {
        psMsgBuf->bTimestampCaptured = FALSE;
        psMsgBuf->u8RxTimestampPointer = 0;
        psMsgBuf->u16RxTimestamp = (psRxBuf->u32Config & RX_BUFFER_AND_FIFO_R1_ELEM_RXTS_Msk);
    }

    psMsgBuf->u32DLC = CANFD_DecodeDLC((psRxBuf->u32Config & RX_BUFFER_AND_FIFO_R1_ELEM_DLC_Msk) >> RX_BUFFER_AND_FIFO_R1_ELEM_DLC_Pos);

    for (u32Idx = 0 ; u32Idx < psMsgBuf->u32DLC ; u32Idx++)
    {
        psMsgBuf->au8Data[u32Idx] = psRxBuf->au8Data[u32Idx];
    }
}


/**
 * @brief       Get Rx FIFO water level.
 *
 * @param[in]   psCanfd         The pointer to CANFD module base address.
 * @param[in]   u32RxFifoNum    0: RX FIFO_0, 1: RX_FIFO_1
 *
 * @return      Rx FIFO water level.
 *
 * @details     Get Rx FIFO water level.
 */
uint32_t CANFD_GetRxFifoWaterLvl(const CANFD_T *psCanfd, uint32_t u32RxFifoNum)
{
    uint32_t u32WaterLevel = 0;

    if (u32RxFifoNum == 0UL)
    {
        u32WaterLevel = ((psCanfd->RXF0C & CANFD_RXF0C_F0WM_Msk) >> CANFD_RXF0C_F0WM_Pos);
    }
    else
    {
        u32WaterLevel = ((psCanfd->RXF1C & CANFD_RXF1C_F1WM_Msk) >> CANFD_RXF1C_F1WM_Pos);
    }

    return u32WaterLevel;
}


/**
 * @brief       Copies messages from FIFO into a message buffert.
 *
 * @param[in]   psCanfd         The pointer to CANFD module base address.
 * @param[in]   psRxBuf         Buffer to read from.
 * @param[in]   psMsgBuf        Location to store read message.
 *
 * @return      None.
 *
 * @details      Copies messages from FIFO into a message buffert.
 */
void CANFD_CopyRxFifoToMsgBuf(const CANFD_T *psCanfd, CANFD_BUF_T *psRxBuf, CANFD_FD_MSG_T *psMsgBuf)
{
    /*Copies a message from a dedicated Rx FIFO into a message buffer*/
    CANFD_CopyDBufToMsgBuf(psCanfd, psRxBuf, psMsgBuf);
}


/**
 * @brief       Cancel a Tx buffer transmission request.
 *
 * @param[in]   psCanfd         The pointer to CANFD module base address.
 * @param[in]   u32TxBufIdx     Tx buffer index number
 *
 * @return      None.
 *
 * @details     Cancel a Tx buffer transmission request.
 */
void CANFD_TxBufCancelReq(CANFD_T *psCanfd, uint32_t u32TxBufIdx)
{
    psCanfd->TXBCR = psCanfd->TXBCR | (0x1UL << u32TxBufIdx);
}


/**
 * @brief       Checks if a Tx buffer cancellation request has been finished or not.
 *
 * @param[in]   psCanfd         The pointer to CAN FD module base address.
 * @param[in]   u32TxBufIdx     Tx buffer index number
 *
 * @return      0: cancellation finished.
 *              1: cancellation pending
 *
 * @details     Checks if a Tx buffer cancellation request has been finished or not.
 */
uint32_t CANFD_IsTxBufCancelFin(const CANFD_T *psCanfd, uint32_t u32TxBufIdx)
{
    /* wait for completion */
    return ((psCanfd->TXBCR & (0x1UL << u32TxBufIdx)) >> u32TxBufIdx);
}

/**
 * @brief       Checks if a Tx buffer transmission has occurred or not.
 *
 * @param[in]   psCanfd         The pointer to CAN FD module base address.
 * @param[in]   u32TxBufIdx     Tx buffer index number
 *
 * @return     0: No transmission occurred.
 *             1: Transmission occurred
 *
 * @details     Checks if a Tx buffer transmission has occurred or not.
 */
uint32_t CANFD_IsTxBufTransmitOccur(const CANFD_T *psCanfd, uint32_t u32TxBufIdx)
{
    return ((psCanfd->TXBTO & (0x1UL << u32TxBufIdx)) >> u32TxBufIdx);
}


/**
 * @brief       Init Tx event fifo
 *
 * @param[in]   psCanfd          The pointer to CAN FD module base address.
 * @param[in]   psRamConfig      Tx Event Fifo configuration ram address.
 * @param[in]   psElemSize       Tx Event Fifo configuration element size
 * @param[in]   u32FifoWaterLvl  FIFO water level
 *
 * @return      None.
 *
 * @details     Init Tx event fifo.
 */
void CANFD_InitTxEvntFifo(CANFD_T *psCanfd, const CANFD_RAM_PART_T *psRamConfig, const CANFD_ELEM_SIZE_T *psElemSize, uint32_t u32FifoWaterLvl)
{
    /* Set TX Event FIFO element size,watermark,start address. */
    psCanfd->TXEFC = (u32FifoWaterLvl << CANFD_TXEFC_EFWM_Pos) | (psElemSize->u32TxEventFifo << CANFD_TXEFC_EFS_Pos)
                     | (psRamConfig->u32TXEFC_EFSA & CANFD_TXEFC_EFSA_Msk);
}


/**
 * @brief       Get Tx event fifo water level
 *
 * @param[in]   psCanfd       The pointer to CANFD module base address.
 *
 * @return      Tx event fifo water level.
 *
 * @details     Get Tx event fifo water level.
 */
uint32_t CANFD_GetTxEvntFifoWaterLvl(const CANFD_T *psCanfd)
{

    return ((psCanfd->TXEFC & CANFD_TXEFC_EFWM_Msk) >> CANFD_TXEFC_EFWM_Pos);
}

/**
 * @brief        Read the TX Event FIFO  Massage
 *
 * @param[in]   psCanfd               The pointer to CAN FD module base address.
 * @param[in]   u8TxEvntIdx           Tx Event FIFO number
 * @param[in]   psEventFifoMsgBuf     Tx Event Message struct
 *
 * @return      None.
 *
 * @details     Copy all Event Elements from TX Event FIFO to the Software Event List .
 */

uint32_t CANFD_ReadTxFifoEventMsg(CANFD_T *psCanfd, uint8_t u8TxEvntIdx, CANFD_TX_EVNT_ELEM_T *psEventFifoMsgBuf)
{
    uint32_t u32Success = 0;

    /* check for valid FIFO number */
    if (u8TxEvntIdx < CANFD_MAX_TX_EVNT_FIFO_ELEMS)
    {
        if ((psCanfd->TXEFS & 0x3FUL) > 0UL)
        {
            uint8_t GetIndex;

            GetIndex = (uint8_t)((psCanfd->TXEFS >> 8UL) & 0x3FUL);
            CANFD_CopyTxEvntFifoToUsrBuf(psCanfd, GetIndex, psEventFifoMsgBuf);
            /* we got the message */
            psCanfd->TXEFA = GetIndex;

            /* check for overflow */
            if ((psCanfd->TXEFS & CANFD_TXEFS_TEFL_Msk))
            {
                /* clear overflow flag */
                psCanfd->IR |= CANFD_IR_TEFF_Msk;
                u32Success = 2;
            }
            else
            {
                u32Success = 1;
            }
        }
    }

    return u32Success;
}
/**
 * @brief        Copy Event Elements from TX Event FIFO to user buffer
 *
 * @param[in]   psCanfd          The pointer to CAN FD module base address.
 * @param[in]   u32TxEvntNum     Tx Event FIFO number
 * @param[in]   psTxEvntElem     Tx Event Message struct
 *
 * @return      None.
 *
 * @details     Copy all Event Elements from TX Event FIFO to the Software Event List .
 */
void CANFD_CopyTxEvntFifoToUsrBuf(const CANFD_T *psCanfd, uint32_t u32TxEvntNum, CANFD_TX_EVNT_ELEM_T *psTxEvntElem)
{
    const uint32_t *pu32TxEvnt;

    /*Get the Tx Event FIFO Address*/
    pu32TxEvnt = (uint32_t *)(CANFD_SRAM_BASE_ADDR(psCanfd) + CANFD_GetTxEventFifoElementAddress(psCanfd, u32TxEvntNum));

    /*Get the Error State Indicator*/
    if ((pu32TxEvnt[0] & TX_FIFO_E0_EVENT_ESI_Msk) > 0UL)
    {
        psTxEvntElem->bErrStaInd = TRUE; //Transmitting node is error passive
    }
    else
    {
        psTxEvntElem->bErrStaInd = FALSE;//Transmitting node is error active
    }

    /*Get the Tx FIFO Identifier type and Identifier*/
    if ((pu32TxEvnt[0] & TX_FIFO_E0_EVENT_XTD_Msk) > 0UL)
    {
        psTxEvntElem->eIdType = eCANFD_XID;
        psTxEvntElem->u32Id = (pu32TxEvnt[0] & TX_FIFO_E0_EVENT_ID_Msk);// Extended ID
    }
    else
    {
        psTxEvntElem->eIdType = eCANFD_SID;
        psTxEvntElem->u32Id = (pu32TxEvnt[0] & TX_FIFO_E0_EVENT_ID_Msk) >> 18UL;// Standard ID
    }

    /*Get the Frame type*/
    if ((pu32TxEvnt[0] & TX_FIFO_E0_EVENT_RTR_Msk) > 0UL)
    {
        psTxEvntElem->bRemote = TRUE; //Remote frame
    }
    else
    {
        psTxEvntElem->bRemote = FALSE; //Data frame
    }

    /*Get the FD Format type*/
    if ((pu32TxEvnt[1] & TX_FIFO_E1_EVENT_FDF_Msk) > 0UL)
    {
        psTxEvntElem->bFDFormat = TRUE; //CAN FD frame format
    }
    else
    {
        psTxEvntElem->bFDFormat = FALSE; //Classical CAN frame format
    }

    /*Get the Bit Rate Switch type*/
    if ((pu32TxEvnt[1] & TX_FIFO_E1_EVENT_BRS_Msk) > 0UL)
    {
        psTxEvntElem->bBitRateSwitch = TRUE;  //Frame transmitted with bit rate switching
    }
    else
    {
        psTxEvntElem->bBitRateSwitch = FALSE; //Frame transmitted without bit rate switching
    }

    /*Get the Tx FIFO Data Length  */
    psTxEvntElem->u32DLC = CANFD_DecodeDLC((uint8_t)((pu32TxEvnt[1] & TX_FIFO_E1_EVENT_DLC_Msk) >> TX_FIFO_E1_EVENT_DLC_Pos));

    /*Get the Tx FIFO Timestamp  */
    psTxEvntElem->u32TxTs = (((pu32TxEvnt[1] & TX_FIFO_E1A_EVENT_TXTS_Msk) >> TX_FIFO_E1A_EVENT_TXTS_Pos));
    /*Get the Tx FIFO Message marker  */
    psTxEvntElem->u32MsgMarker = (((pu32TxEvnt[1] & TX_FIFO_E1_EVENT_MM_Msk) >> TX_FIFO_E1_EVENT_MM_Pos));
}


/**
 * @brief       Get CAN FD interrupts status.
 *
 * @param[in]   psCanfd         The pointer of the specified CAN FD module.
 * @param[in]   u32IntTypeFlag  Interrupt Type Flag, should be
 *                              - \ref CANFD_IR_ARA_Msk     : Access to Reserved Address interrupt Indicator
 *                              - \ref CANFD_IR_PED_Msk     : Protocol Error in Data Phase interrupt Indicator
 *                              - \ref CANFD_IR_PEA_Msk     : Protocol Error in Arbitration Phase interrupt Indicator
 *                              - \ref CANFD_IR_WDI_Msk     : Watchdog interrupt Indicator
 *                              - \ref CANFD_IR_BO_Msk      : Bus_Off Status interrupt Indicator
 *                              - \ref CANFD_IR_EW_Msk      : Warning Status interrupt Indicator
 *                              - \ref CANFD_IR_EP_Msk      : Error Passive interrupt Indicator
 *                              - \ref CANFD_IR_ELO_Msk     : Error Logging Overflow interrupt Indicator
 *                              - \ref CANFD_IR_DRX_Msk     : Message stored to Dedicated Rx Buffer interrupt Indicator
 *                              - \ref CANFD_IR_TOO_Msk     : Timeout Occurred interrupt Indicator
 *                              - \ref CANFD_IR_MRAF_Msk    : Message RAM Access Failure interrupt Indicator
 *                              - \ref CANFD_IR_TSW_Msk     : Timestamp Wraparound interrupt Indicator
 *                              - \ref CANFD_IR_TEFL_Msk    : Tx Event FIFO Event Lost interrupt Indicator
 *                              - \ref CANFD_IR_TEFF_Msk    : Tx Event FIFO Full Indicator
 *                              - \ref CANFD_IR_TEFW_Msk    : Tx Event FIFO Watermark Reached Interrupt Indicator
 *                              - \ref CANFD_IR_TEFN_Msk    : Tx Event FIFO New Entry Interrupt Indicator
 *                              - \ref CANFD_IR_TFE_Msk     : Tx FIFO Empty Interrupt Indicator
 *                              - \ref CANFD_IR_TCF_Msk     : Transmission Cancellation Finished Interrupt Indicator
 *                              - \ref CANFD_IR_TC_Msk      : Transmission Completed interrupt Indicator
 *                              - \ref CANFD_IR_HPM_Msk     : High Priority Message Interrupt Indicator
 *                              - \ref CANFD_IR_RF1L_Msk    : Rx FIFO 1 Message Lost Interrupt Indicator
 *                              - \ref CANFD_IR_RF1F_Msk    : Rx FIFO 1 Full Interrupt Indicator
 *                              - \ref CANFD_IR_RF1W_Msk    : Rx FIFO 1 Watermark Reached Interrupt Indicator
 *                              - \ref CANFD_IR_RF1N_Msk    : Rx FIFO 1 New Message Interrupt Indicator
 *                              - \ref CANFD_IR_RF0L_Msk    : Rx FIFO 0 Message Lost Interrupt Indicator
 *                              - \ref CANFD_IR_RF0F_Msk    : Rx FIFO 0 Full Interrupt Indicator
 *                              - \ref CANFD_IR_RF0W_Msk    : Rx FIFO 0 Watermark Reached Interrupt Indicator
 *                              - \ref CANFD_IR_RF0N_Msk    : Rx FIFO 0 New Message Interrupt Indicator
 *
 * @return      None.
 *
 * @details     This function gets all CAN FD interrupt status flags.
 */
uint32_t CANFD_GetStatusFlag(const CANFD_T *psCanfd, uint32_t u32IntTypeFlag)
{
    return (psCanfd->IR & u32IntTypeFlag);
}


/**
 * @brief       Clears the CAN FD module interrupt flags
 *
 * @param[in]   psCanfd           The pointer of the specified CANFD module.
 * @param[in]   u32InterruptFlag  The specified interrupt of CAN FD module
 *                               - \ref CANFD_IR_ARA_Msk     : Access to Reserved Address interrupt Indicator
 *                               - \ref CANFD_IR_PED_Msk     : Protocol Error in Data Phase interrupt Indicator
 *                               - \ref CANFD_IR_PEA_Msk     : Protocol Error in Arbitration Phase interrupt Indicator
 *                               - \ref CANFD_IR_WDI_Msk     : Watchdog interrupt Indicator
 *                               - \ref CANFD_IR_BO_Msk      : Bus_Off Status interrupt Indicator
 *                               - \ref CANFD_IR_EW_Msk      : Warning Status interrupt Indicator
 *                               - \ref CANFD_IR_EP_Msk      : Error Passive interrupt Indicator
 *                               - \ref CANFD_IR_ELO_Msk     : Error Logging Overflow interrupt Indicator
 *                               - \ref CANFD_IR_DRX_Msk     : Message stored to Dedicated Rx Buffer interrupt Indicator
 *                               - \ref CANFD_IR_TOO_Msk     : Timeout Occurred interrupt Indicator
 *                               - \ref CANFD_IR_MRAF_Msk    : Message RAM Access Failure interrupt Indicator
 *                               - \ref CANFD_IR_TSW_Msk     : Timestamp Wraparound interrupt Indicator
 *                               - \ref CANFD_IR_TEFL_Msk    : Tx Event FIFO Event Lost interrupt Indicator
 *                               - \ref CANFD_IR_TEFF_Msk    : Tx Event FIFO Full Indicator
 *                               - \ref CANFD_IR_TEFW_Msk    : Tx Event FIFO Watermark Reached Interrupt Indicator
 *                               - \ref CANFD_IR_TEFN_Msk    : Tx Event FIFO New Entry Interrupt Indicator
 *                               - \ref CANFD_IR_TFE_Msk     : Tx FIFO Empty Interrupt Indicator
 *                               - \ref CANFD_IR_TCF_Msk     : Transmission Cancellation Finished Interrupt Indicator
 *                               - \ref CANFD_IR_TC_Msk      : Transmission Completed interrupt Indicator
 *                               - \ref CANFD_IR_HPM_Msk     : High Priority Message Interrupt Indicator
 *                               - \ref CANFD_IR_RF1L_Msk    : Rx FIFO 1 Message Lost Interrupt Indicator
 *                               - \ref CANFD_IR_RF1F_Msk    : Rx FIFO 1 Full Interrupt Indicator
 *                               - \ref CANFD_IR_RF1W_Msk    : Rx FIFO 1 Watermark Reached Interrupt Indicator
 *                               - \ref CANFD_IR_RF1N_Msk    : Rx FIFO 1 New Message Interrupt Indicator
 *                               - \ref CANFD_IR_RF0L_Msk    : Rx FIFO 0 Message Lost Interrupt Indicator
 *                               - \ref CANFD_IR_RF0F_Msk    : Rx FIFO 0 Full Interrupt Indicator
 *                               - \ref CANFD_IR_RF0W_Msk    : Rx FIFO 0 Watermark Reached Interrupt Indicator
 *                               - \ref CANFD_IR_RF0N_Msk    : Rx FIFO 0 New Message Interrupt Indicator
 *
 * @return      None.
 *
 * @details     This function clears CAN FD interrupt status flags.
 */
void CANFD_ClearStatusFlag(CANFD_T *psCanfd, uint32_t u32InterruptFlag)
{
    /* Write 1 to clear status flag. */
    psCanfd->IR = u32InterruptFlag;
}


/**
 * @brief       Gets the CAN FD Bus Error Counter value.
 *
 * @param[in]   psCanfd        The pointer of the specified CAN FD module.
 * @param[in]   pu8TxErrBuf    TxErrBuf Buffer to store Tx Error Counter value.
 * @param[in]   pu8RxErrBuf    RxErrBuf Buffer to store Rx Error Counter value.
 *
 * @return      None.
 *
 * @details     This function gets the CAN FD Bus Error Counter value for both Tx and Rx direction.
 *              These values may be needed in the upper layer error handling.
 */
void CANFD_GetBusErrCount(const CANFD_T *psCanfd, uint8_t *pu8TxErrBuf, uint8_t *pu8RxErrBuf)
{
    if (pu8TxErrBuf)
    {
        *pu8TxErrBuf = (uint8_t)((psCanfd->ECR >> CANFD_ECR_TEC_Pos) & CANFD_ECR_TEC_Msk);
    }

    if (pu8RxErrBuf)
    {
        *pu8RxErrBuf = (uint8_t)((psCanfd->ECR >> CANFD_ECR_REC_Pos) & CANFD_ECR_REC_Msk);
    }
}


/**
 * @brief       CAN FD Run to the Normal Operation.
 *
 * @param[in]   psCanfd        The pointer of the specified CAN FD module.
 * @param[in]   u8Enable       TRUE or FALSE.
 *
 * @return      None.
 *
 * @details     This function is used to switch between initial mode and normal mode.
 */
void CANFD_RunToNormal(CANFD_T *psCanfd, uint8_t u8Enable)
{
    uint32_t u32TimeOutCount = SystemCoreClock; // 1 second timeout

    if (u8Enable)
    {
        /* start operation */
        psCanfd->CCCR &= ~(CANFD_CCCR_CCE_Msk | CANFD_CCCR_INIT_Msk);

        /* Check INIT bit is released */
        while (psCanfd->CCCR & CANFD_CCCR_INIT_Msk)
        {
            if (u32TimeOutCount == 0UL)
            {
                break;
            }

            u32TimeOutCount--;
        }
    }
    else
    {
        /* init mode */
        psCanfd->CCCR |= CANFD_CCCR_INIT_Msk;

        /* Wait run to INIT mode */
        while (!(psCanfd->CCCR & CANFD_CCCR_INIT_Msk))
        {
            if (u32TimeOutCount == 0UL)
            {
                break;
            }

            u32TimeOutCount--;
        }

        /* Configuration change enable */
        psCanfd->CCCR |= CANFD_CCCR_CCE_Msk;
    }
}


/** @} end of group CANFD_EXPORTED_FUNCTIONS */

/** @} end of group CANFD_Driver */

/** @} end of group Standard_Driver */
