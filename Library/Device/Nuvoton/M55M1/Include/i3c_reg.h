/**************************************************************************//**
 * @file     i3c_reg.h
 * @version  V1.00
 * @brief    I3C register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __I3C_REG_H__
#define __I3C_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup I3C I3C Serial Interface Controller (I3C)
    Memory Mapped Structure for I3C Controller
    @{
*/

typedef struct
{
    __IO uint32_t DEVCTL;               /*!< [0x0000] I3C Device Control Register */
    __IO uint32_t DEVADDR;              /*!< [0x0004] I3C Device Address Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE0;
    /// @endcond //HIDDEN_SYMBOLS
    __O  uint32_t CMDQUE;               /*!< [0x000c] I3C Command Queue Port Register */
    __I  uint32_t RESPQUE;              /*!< [0x0010] I3C Response Queue Port Register */
    __IO uint32_t TXRXDAT;              /*!< [0x0014] I3C Transmit and Receive Data Port Register */
    union
    {
        __IO uint32_t IBIQUE;           /*!< [0x0018] I3C In-Band Interrupt Queue Data Register */
        __IO uint32_t IBISTS;           /*!< [0x0018] I3C In-Band Interrupt Queue Status Register */
    };
    __IO uint32_t QUETHCTL;             /*!< [0x001c] I3C Queue Threshold Control Register */
    __IO uint32_t DBTHCTL;              /*!< [0x0020] I3C Data Buffer Threshold Control Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE1[4];
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t RSTCTL;               /*!< [0x0034] I3C Reset Control Register */
    __IO uint32_t SLVEVNTS;             /*!< [0x0038] I3C Slave Event Status Register */
    __IO uint32_t INTSTS;               /*!< [0x003c] I3C Interrupt Status Register */
    __IO uint32_t INTSTSEN;             /*!< [0x0040] I3C Interrupt Status Enable Register */
    __IO uint32_t INTEN;                /*!< [0x0044] I3C Interrupt Signal Enable Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE2;
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t QUESTSLV;             /*!< [0x004c] I3C Queue Status Level Register */
    __I  uint32_t DBSTSLV;              /*!< [0x0050] I3C Data Buffer Status Level Register */
    __I  uint32_t PRESENTS;             /*!< [0x0054] I3C Present State Register */
    __I  uint32_t CCCDEVS;              /*!< [0x0058] I3C Device Operating Status Register */
    __I  uint32_t ADDRTP;               /*!< [0x005c] I3C Pointer for Device Address Table Register */
    __I  uint32_t CHRTP;                /*!< [0x0060] I3C Pointer for Device Characteristics Table Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE3[2];
    /// @endcond //HIDDEN_SYMBOLS
    __I  uint32_t VENDOR;               /*!< [0x006c] I3C Pointer for Vendor specific Register */
    __IO uint32_t SLVMID;               /*!< [0x0070] I3C MIPI Manufacturer ID Register */
    __IO uint32_t SLVPID;               /*!< [0x0074] I3C Normal Provisional ID Register */
    __IO uint32_t SLVCHAR;              /*!< [0x0078] I3C Slave Characteristic Register */
    __I  uint32_t SLVMXLEN;             /*!< [0x007c] I3C Maximum Write/Read Length Register */
    __I  uint32_t MXRDTURN;             /*!< [0x0080] I3C Maximum Read Turnaround Time Register */
    __IO uint32_t MXDS;                 /*!< [0x0084] I3C Maximum Data Speed Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE4;
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t SIR;                  /*!< [0x008c] I3C Slave Interrupt Request Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE5;
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t SIRDAT;               /*!< [0x0094] I3C Slave Interrupt Request Data Register */
    __IO uint32_t SIRRESP;              /*!< [0x0098] I3C Slave Interrupt Request Response Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE6[5];
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t DEVCTLE;              /*!< [0x00b0] I3C Device Control Extended Register */
    __IO uint32_t SCLOD;                /*!< [0x00b4] SCL I3C Open Drain Timing Register */
    __IO uint32_t SCLPP;                /*!< [0x00b8] SCL I3C Push Pull Timing Register */
    __IO uint32_t SCLFM;                /*!< [0x00bc] SCL I2C Fast Mode Timing Register */
    __IO uint32_t SCLFMP;               /*!< [0x00c0] SCL I2C Fast Mode Plus Timing Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE7;
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t SCLEXTL;              /*!< [0x00c8] I3C SCL Extended Low Count Timing Register */
    __IO uint32_t SCLTERM;              /*!< [0x00cc] I3C SCL Termination Bit Low count Timing Register */
    __IO uint32_t SDAHOLD;              /*!< [0x00d0] SDA Hold and Mode Switch Delay Timing Register */
    __IO uint32_t BUSFAT;               /*!< [0x00d4] I3C Bus Free And Available Timing Register */
    __IO uint32_t BUSIDLET;             /*!< [0x00d8] I3C Bus Idle Timing Register */
    __I  uint32_t SCLLOW;               /*!< [0x00dc] SCL Low Timeout Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE8[72];
    /// @endcond //HIDDEN_SYMBOLS
    __I  uint32_t DEV1CH[4];            /*!< [0x0200 ~ 0x020c] Device Characteristic Table Location of Device1 Register */
    __I  uint32_t DEV2CH[4];            /*!< [0x0210 ~ 0x021c] Device Characteristic Table Location of Device2 Register */
    __I  uint32_t DEV3CH[4];            /*!< [0x0220 ~ 0x022c] Device Characteristic Table Location of Device3 Register */
    __I  uint32_t DEV4CH[4];            /*!< [0x0230 ~ 0x023c] Device Characteristic Table Location of Device4 Register */
    __I  uint32_t DEV5CH[4];            /*!< [0x0240 ~ 0x024c] Device Characteristic Table Location of Device5 Register */
    __I  uint32_t DEV6CH[4];            /*!< [0x0250 ~ 0x025c] Device Characteristic Table Location of Device6 Register */
    __I  uint32_t DEV7CH[4];            /*!< [0x0260 ~ 0x026c] Device Characteristic Table Location of Device7 Register */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE9[4];
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t DEV1ADR;              /*!< [0x0280] Device Address Table Location of Device1 Register */
    __IO uint32_t DEV2ADR;              /*!< [0x0284] Device Address Table Location of Device2 Register */
    __IO uint32_t DEV3ADR;              /*!< [0x0288] Device Address Table Location of Device3 Register */
    __IO uint32_t DEV4ADR;              /*!< [0x028c] Device Address Table Location of Device4 Register */
    __IO uint32_t DEV5ADR;              /*!< [0x0290] Device Address Table Location of Device5 Register */
    __IO uint32_t DEV6ADR;              /*!< [0x0294] Device Address Table Location of Device6 Register */
    __IO uint32_t DEV7ADR;              /*!< [0x0298] Device Address Table Location of Device7 Register */
} I3C_T;

/**
    @addtogroup I3C_CONST I3C Bit Field Definition
    Constant Definitions for I3C Controller
    @{
*/

#define I3C_DEVCTL_IBAINCL_Pos          (0U)
#define I3C_DEVCTL_IBAINCL_Msk          (1UL << I3C_DEVCTL_IBAINCL_Pos)

#define I3C_DEVCTL_PENDINT_Pos          (3U)
#define I3C_DEVCTL_PENDINT_Msk          (0xFUL << I3C_DEVCTL_PENDINT_Pos)

#define I3C_DEVCTL_HJCTL_Pos            (8U)
#define I3C_DEVCTL_HJCTL_Msk            (1UL << I3C_DEVCTL_HJCTL_Pos)

#define I3C_DEVCTL_IDLECNT_Pos          (24U)
#define I3C_DEVCTL_IDLECNT_Msk          (0x3UL << I3C_DEVCTL_IDLECNT_Pos)

#define I3C_DEVCTL_ADAPTIVE_Pos         (27U)
#define I3C_DEVCTL_ADAPTIVE_Msk         (1UL << I3C_DEVCTL_ADAPTIVE_Pos)

#define I3C_DEVCTL_DMAEN_Pos            (28U)
#define I3C_DEVCTL_DMAEN_Msk            (1UL << I3C_DEVCTL_DMAEN_Pos)

#define I3C_DEVCTL_ABORT_Pos            (29U)
#define I3C_DEVCTL_ABORT_Msk            (1UL << I3C_DEVCTL_ABORT_Pos)

#define I3C_DEVCTL_RESUME_Pos           (30U)
#define I3C_DEVCTL_RESUME_Msk           (1UL << I3C_DEVCTL_RESUME_Pos)

#define I3C_DEVCTL_ENABLE_Pos           (31U)
#define I3C_DEVCTL_ENABLE_Msk           (1UL << I3C_DEVCTL_ENABLE_Pos)

#define I3C_DEVADDR_SA_Pos              (0U)
#define I3C_DEVADDR_SA_Msk              (0x7FUL << I3C_DEVADDR_SA_Pos)

#define I3C_DEVADDR_SAVALID_Pos         (15U)
#define I3C_DEVADDR_SAVALID_Msk         (1UL << I3C_DEVADDR_SAVALID_Pos)

#define I3C_DEVADDR_DA_Pos              (16U)
#define I3C_DEVADDR_DA_Msk              (0x7FUL << I3C_DEVADDR_DA_Pos)

#define I3C_DEVADDR_DAVALID_Pos         (31U)
#define I3C_DEVADDR_DAVALID_Msk         (1UL << I3C_DEVADDR_DAVALID_Pos)

#define I3C_CMDQUE_ATTR_Pos             (0U)
#define I3C_CMDQUE_ATTR_Msk             (0x7UL << I3C_CMDQUE_ATTR_Pos)

#define I3C_CMDQUE_TID_Pos              (3U)
#define I3C_CMDQUE_TID_Msk              (0xFUL << I3C_CMDQUE_TID_Pos)

#define I3C_CMDQUE_CMD_Pos              (7U)
#define I3C_CMDQUE_CMD_Msk              (0xFFUL << I3C_CMDQUE_CMD_Pos)

#define I3C_CMDQUE_CP_Pos               (15U)
#define I3C_CMDQUE_CP_Msk               (1UL << I3C_CMDQUE_CP_Pos)

#define I3C_CMDQUE_DEVINDX_Pos          (16U)
#define I3C_CMDQUE_DEVINDX_Msk          (0x1FUL << I3C_CMDQUE_DEVINDX_Pos)

#define I3C_CMDQUE_SPEED_Pos            (21U)
#define I3C_CMDQUE_SPEED_Msk            (0x7UL << I3C_CMDQUE_SPEED_Pos)

#define I3C_CMDQUE_DBP_Pos              (25U)
#define I3C_CMDQUE_DBP_Msk              (0x1UL << I3C_CMDQUE_DBP_Pos)

#define I3C_CMDQUE_ROC_Pos              (26U)
#define I3C_CMDQUE_ROC_Msk              (0x1UL << I3C_CMDQUE_ROC_Pos)

#define I3C_CMDQUE_SDAP_Pos             (27U)
#define I3C_CMDQUE_SDAP_Msk             (0x1UL << I3C_CMDQUE_SDAP_Pos)

#define I3C_CMDQUE_RNW_Pos              (28U)
#define I3C_CMDQUE_RNW_Msk              (0x1UL << I3C_CMDQUE_RNW_Pos)

#define I3C_CMDQUE_TOC_Pos              (30U)
#define I3C_CMDQUE_TOC_Msk              (0x1UL << I3C_CMDQUE_TOC_Pos)

#define I3C_CMDQUE_PEC_Pos              (31U)
#define I3C_CMDQUE_PEC_Msk              (0x1UL << I3C_CMDQUE_PEC_Pos)

#define I3C_CMDQUE_DB_Pos               (8U)
#define I3C_CMDQUE_DB_Msk               (0xFFUL << I3C_CMDQUE_DB_Pos)

#define I3C_CMDQUE_DATLEN_Pos           (16U)
#define I3C_CMDQUE_DATLEN_Msk           (0xFFFFUL << I3C_CMDQUE_DATLEN_Pos)

#define I3C_CMDQUE_BYTESTRB_Pos         (3U)
#define I3C_CMDQUE_BYTESTRB_Msk         (0x7UL << I3C_CMDQUE_BYTESTRB_Pos)

#define I3C_CMDQUE_DATBYTE0_Pos         (8U)
#define I3C_CMDQUE_DATBYTE0_Msk         (0xFFUL << I3C_CMDQUE_DATBYTE0_Pos)

#define I3C_CMDQUE_DATBYTE1_Pos         (16U)
#define I3C_CMDQUE_DATBYTE1_Msk         (0xFFUL << I3C_CMDQUE_DATBYTE1_Pos)

#define I3C_CMDQUE_DATBYTE2_Pos         (24U)
#define I3C_CMDQUE_DATBYTE2_Msk         (0xFFUL << I3C_CMDQUE_DATBYTE2_Pos)

#define I3C_CMDQUE_DEVCOUNT_Pos         (21U)
#define I3C_CMDQUE_DEVCOUNT_Msk         (0x1FUL << I3C_CMDQUE_DEVCOUNT_Pos)

#define I3C_RESPQUE_DATLEN_Pos          (0U)
#define I3C_RESPQUE_DATLEN_Msk          (0xFFFFUL << I3C_RESPQUE_DATLEN_Pos)

#define I3C_RESPQUE_HDRCODE_Pos         (16U)
#define I3C_RESPQUE_HDRCODE_Msk         (0xFFUL << I3C_RESPQUE_HDRCODE_Pos)

#define I3C_RESPQUE_TID_Pos             (24U)
#define I3C_RESPQUE_TID_Msk             (0xFUL << I3C_RESPQUE_TID_Pos)

#define I3C_RESPQUE_RXRESP_Pos          (27U)
#define I3C_RESPQUE_RXRESP_Msk          (1UL << I3C_RESPQUE_RXRESP_Pos)

#define I3C_RESPQUE_ERRSTS_Pos          (28U)
#define I3C_RESPQUE_ERRSTS_Msk          (0xFUL << I3C_RESPQUE_ERRSTS_Pos)

#define I3C_TXRXDAT_DAT_Pos             (0U)
#define I3C_TXRXDAT_DAT_Msk             (0xFFFFFFFFUL << I3C_TXRXDAT_DAT_Pos)

#define I3C_IBIQUE_DAT_Pos              (0U)
#define I3C_IBIQUE_DAT_Msk              (0xFFFFFFFFUL << I3C_IBIQUE_DAT_Pos)

#define I3C_IBISTS_DATLEN_Pos           (0U)
#define I3C_IBISTS_DATLEN_Msk           (0xFFUL << I3C_IBISTS_DATLEN_Pos)

#define I3C_IBISTS_IBIID_Pos            (8U)
#define I3C_IBISTS_IBIID_Msk            (0xFFUL << I3C_IBISTS_IBIID_Pos)

#define I3C_IBISTS_IBISTS_Pos           (24U)
#define I3C_IBISTS_IBISTS_Msk           (0xFFUL << I3C_IBISTS_IBISTS_Pos)

#define I3C_QUETHCTL_CMDETH_Pos         (0U)
#define I3C_QUETHCTL_CMDETH_Msk         (0xFFUL << I3C_QUETHCTL_CMDETH_Pos)

#define I3C_QUETHCTL_RESPTH_Pos         (8U)
#define I3C_QUETHCTL_RESPTH_Msk         (0xFFUL << I3C_QUETHCTL_RESPTH_Pos)

#define I3C_QUETHCTL_IBIDATTH_Pos       (16U)
#define I3C_QUETHCTL_IBIDATTH_Msk       (0xFFUL << I3C_QUETHCTL_IBIDATTH_Pos)

#define I3C_QUETHCTL_IBISTSTH_Pos       (24U)
#define I3C_QUETHCTL_IBISTSTH_Msk       (0xFFUL << I3C_QUETHCTL_IBISTSTH_Pos)

#define I3C_DBTHCTL_TXTH_Pos            (0U)
#define I3C_DBTHCTL_TXTH_Msk            (0x7UL << I3C_DBTHCTL_TXTH_Pos)

#define I3C_DBTHCTL_RXTH_Pos            (8U)
#define I3C_DBTHCTL_RXTH_Msk            (0x7UL << I3C_DBTHCTL_RXTH_Pos)

#define I3C_DBTHCTL_TXSTATH_Pos         (16U)
#define I3C_DBTHCTL_TXSTATH_Msk         (0x7UL << I3C_DBTHCTL_TXSTATH_Pos)

#define I3C_DBTHCTL_RXSTATH_Pos         (24U)
#define I3C_DBTHCTL_RXSTATH_Msk         (0x7UL << I3C_DBTHCTL_RXSTATH_Pos)

#define I3C_RSTCTL_SWRST_Pos            (0U)
#define I3C_RSTCTL_SWRST_Msk            (1UL << I3C_RSTCTL_SWRST_Pos)

#define I3C_RSTCTL_CMDRST_Pos           (1U)
#define I3C_RSTCTL_CMDRST_Msk           (1UL << I3C_RSTCTL_CMDRST_Pos)

#define I3C_RSTCTL_RESPRST_Pos          (2U)
#define I3C_RSTCTL_RESPRST_Msk          (1UL << I3C_RSTCTL_RESPRST_Pos)

#define I3C_RSTCTL_TXRST_Pos            (3U)
#define I3C_RSTCTL_TXRST_Msk            (1UL << I3C_RSTCTL_TXRST_Pos)

#define I3C_RSTCTL_RXRST_Pos            (4U)
#define I3C_RSTCTL_RXRST_Msk            (1UL << I3C_RSTCTL_RXRST_Pos)

#define I3C_RSTCTL_IBIQRST_Pos          (5U)
#define I3C_RSTCTL_IBIQRXRST_Msk        (1UL << I3C_RSTCTL_IBIQRST_Pos)

#define I3C_SLVEVNTS_SIREN_Pos          (0U)
#define I3C_SLVEVNTS_SIREN_Msk          (1UL << I3C_SLVEVNTS_SIREN_Pos)

#define I3C_SLVEVNTS_MREN_Pos           (1U)
#define I3C_SLVEVNTS_MREN_Msk           (1UL << I3C_SLVEVNTS_MREN_Pos)

#define I3C_SLVEVNTS_HJEN_Pos           (3U)
#define I3C_SLVEVNTS_HJEN_Msk           (1UL << I3C_SLVEVNTS_HJEN_Pos)

#define I3C_SLVEVNTS_ACTSTATE_Pos       (4U)
#define I3C_SLVEVNTS_ACTSTATE_Msk       (3UL << I3C_SLVEVNTS_ACTSTATE_Pos)

#define I3C_SLVEVNTS_MRLUPD_Pos         (6U)
#define I3C_SLVEVNTS_MRLUPD_Msk         (1UL << I3C_SLVEVNTS_MRLUPD_Pos)

#define I3C_SLVEVNTS_MWLUPD_Pos         (7U)
#define I3C_SLVEVNTS_MWLUPD_Msk         (1UL << I3C_SLVEVNTS_MWLUPD_Pos)

#define I3C_INTSTS_TXTH_Pos             (0U)
#define I3C_INTSTS_TXTH_Msk             (1UL << I3C_INTSTS_TXTH_Pos)

#define I3C_INTSTS_RXTH_Pos             (1U)
#define I3C_INTSTS_RXTH_Msk             (1UL << I3C_INTSTS_RXTH_Pos)

#define I3C_INTSTS_IBITH_Pos            (2U)
#define I3C_INTSTS_IBITH_Msk            (1UL << I3C_INTSTS_IBITH_Pos)

#define I3C_INTSTS_CMDRDY_Pos           (3U)
#define I3C_INTSTS_CMDRDY_Msk           (1UL << I3C_INTSTS_CMDRDY_Pos)

#define I3C_INTSTS_RESPRDY_Pos          (4U)
#define I3C_INTSTS_RESPRDY_Msk          (1UL << I3C_INTSTS_RESPRDY_Pos)

#define I3C_INTSTS_TFRABORT_Pos         (5U)
#define I3C_INTSTS_TFRABORT_Msk         (1UL << I3C_INTSTS_TFRABORT_Pos)

#define I3C_INTSTS_CCCUPD_Pos           (6U)
#define I3C_INTSTS_CCCUPD_Msk           (1UL << I3C_INTSTS_CCCUPD_Pos)

#define I3C_INTSTS_DAA_Pos              (8U)
#define I3C_INTSTS_DAA_Msk              (1UL << I3C_INTSTS_DAA_Pos)

#define I3C_INTSTS_TFRERR_Pos           (9U)
#define I3C_INTSTS_TFRERR_Msk           (1UL << I3C_INTSTS_TFRERR_Pos)

#define I3C_INTSTS_DEFSLV_Pos           (10U)
#define I3C_INTSTS_DEFSLV_Msk           (1UL << I3C_INTSTS_DEFSLV_Pos)

#define I3C_INTSTS_READREQ_Pos          (11U)
#define I3C_INTSTS_READREQ_Msk          (1UL << I3C_INTSTS_READREQ_Pos)

#define I3C_INTSTS_IBIUPD_Pos           (12U)
#define I3C_INTSTS_IBIUPD_Msk           (1UL << I3C_INTSTS_IBIUPD_Pos)

#define I3C_INTSTS_BUSOWNER_Pos         (13U)
#define I3C_INTSTS_BUSOWNER_Msk         (1UL << I3C_INTSTS_BUSOWNER_Pos)

#define I3C_INTSTS_BUSRSTDN_Pos         (15U)
#define I3C_INTSTS_BUSRSTDN_Msk         (1UL << I3C_INTSTS_BUSRSTDN_Pos)

#define I3C_INTSTSEN_TXTH_Pos           (0U)
#define I3C_INTSTSEN_TXTH_Msk           (1UL << I3C_INTSTSEN_TXTH_Pos)

#define I3C_INTSTSEN_RXTH_Pos           (1U)
#define I3C_INTSTSEN_RXTH_Msk           (1UL << I3C_INTSTSEN_RXTH_Pos)

#define I3C_INTSTSEN_IBITH_Pos          (2U)
#define I3C_INTSTSEN_IBITH_Msk          (1UL << I3C_INTSTSEN_IBITH_Pos)

#define I3C_INTSTSEN_CMDRDY_Pos         (3U)
#define I3C_INTSTSEN_CMDRDY_Msk         (1UL << I3C_INTSTSEN_CMDRDY_Pos)

#define I3C_INTSTSEN_RESPRDY_Pos        (4U)
#define I3C_INTSTSEN_RESPRDY_Msk        (1UL << I3C_INTSTSEN_RESPRDY_Pos)

#define I3C_INTSTSEN_TFRABORT_Pos       (5U)
#define I3C_INTSTSEN_TFRABORT_Msk       (1UL << I3C_INTSTSEN_TFRABORT_Pos)

#define I3C_INTSTSEN_CCCUPD_Pos         (6U)
#define I3C_INTSTSEN_CCCUPD_Msk         (1UL << I3C_INTSTSEN_CCCUPD_Pos)

#define I3C_INTSTSEN_DAA_Pos            (8U)
#define I3C_INTSTSEN_DAA_Msk            (1UL << I3C_INTSTSEN_DAA_Pos)

#define I3C_INTSTSEN_TFRERR_Pos         (9U)
#define I3C_INTSTSEN_TFRERR_Msk         (1UL << I3C_INTSTSEN_TFRERR_Pos)

#define I3C_INTSTSEN_DEFSLV_Pos         (10U)
#define I3C_INTSTSEN_DEFSLV_Msk         (1UL << I3C_INTSTSEN_DEFSLV_Pos)

#define I3C_INTSTSEN_READREQ_Pos        (11U)
#define I3C_INTSTSEN_READREQ_Msk        (1UL << I3C_INTSTSEN_READREQ_Pos)

#define I3C_INTSTSEN_IBIUPD_Pos         (12U)
#define I3C_INTSTSEN_IBIUPD_Msk         (1UL << I3C_INTSTSEN_IBIUPD_Pos)

#define I3C_INTSTSEN_BUSOWNER_Pos       (13U)
#define I3C_INTSTSEN_BUSOWNER_Msk       (1UL << I3C_INTSTSEN_BUSOWNER_Pos)

#define I3C_INTSTSEN_BUSRSTDN_Pos       (15U)
#define I3C_INTSTSEN_BUSRSTDN_Msk       (1UL << I3C_INTSTSEN_BUSRSTDN_Pos)

#define I3C_INTEN_TXTH_Pos              (0U)
#define I3C_INTEN_TXTH_Msk              (1UL << I3C_INTEN_TXTH_Pos)

#define I3C_INTEN_RXTH_Pos              (1U)
#define I3C_INTEN_RXTH_Msk              (1UL << I3C_INTEN_RXTH_Pos)

#define I3C_INTEN_IBITH_Pos             (2U)
#define I3C_INTEN_IBITH_Msk             (1UL << I3C_INTEN_IBITH_Pos)

#define I3C_INTEN_CMDRDY_Pos            (3U)
#define I3C_INTEN_CMDRDY_Msk            (1UL << I3C_INTEN_CMDRDY_Pos)

#define I3C_INTEN_RESPRDY_Pos           (4U)
#define I3C_INTEN_RESPRDY_Msk           (1UL << I3C_INTEN_RESPRDY_Pos)

#define I3C_INTEN_TFRABORT_Pos          (5U)
#define I3C_INTEN_TFRABORT_Msk          (1UL << I3C_INTEN_TFRABORT_Pos)

#define I3C_INTEN_CCCUPD_Pos            (6U)
#define I3C_INTEN_CCCUPD_Msk            (1UL << I3C_INTEN_CCCUPD_Pos)

#define I3C_INTEN_DAA_Pos               (8U)
#define I3C_INTEN_DAA_Msk               (1UL << I3C_INTEN_DAA_Pos)

#define I3C_INTEN_TFRERR_Pos            (9U)
#define I3C_INTEN_TFRERR_Msk            (1UL << I3C_INTEN_TFRERR_Pos)

#define I3C_INTEN_DEFSLV_Pos            (10U)
#define I3C_INTEN_DEFSLV_Msk            (1UL << I3C_INTEN_DEFSLV_Pos)

#define I3C_INTEN_READREQ_Pos           (11U)
#define I3C_INTEN_READREQ_Msk           (1UL << I3C_INTEN_READREQ_Pos)

#define I3C_INTEN_IBIUPD_Pos            (12U)
#define I3C_INTEN_IBIUPD_Msk            (1UL << I3C_INTEN_IBIUPD_Pos)

#define I3C_INTEN_BUSOWNER_Pos          (13U)
#define I3C_INTEN_BUSOWNER_Msk          (1UL << I3C_INTEN_BUSOWNER_Pos)

#define I3C_INTEN_BUSRSTDN_Pos          (15U)
#define I3C_INTEN_BUSRSTDN_Msk          (1UL << I3C_INTEN_BUSRSTDN_Pos)

#define I3C_QUESTSLV_CMDELOC_Pos        (0U)
#define I3C_QUESTSLV_CMDELOC_Msk        (0xFFUL << I3C_QUESTSLV_CMDELOC_Pos)

#define I3C_QUESTSLV_RESPLV_Pos         (8U)
#define I3C_QUESTSLV_RESPLV_Msk         (0xFFUL << I3C_QUESTSLV_RESPLV_Pos)

#define I3C_QUESTSLV_IBIBLR_Pos         (16U)
#define I3C_QUESTSLV_IBIBLR_Msk         (0xFFUL << I3C_QUESTSLV_IBIBLR_Pos)

#define I3C_QUESTSLV_IBISCNT_Pos        (24U)
#define I3C_QUESTSLV_IBISCNT_Msk        (0x1FUL << I3C_QUESTSLV_IBISCNT_Pos)

#define I3C_DBSTSLV_TXELV_Pos           (0U)
#define I3C_DBSTSLV_TXELV_Msk           (0xFFUL << I3C_DBSTSLV_TXELV_Pos)

#define I3C_DBSTSLV_RXLV_Pos            (16U)
#define I3C_DBSTSLV_RXLV_Msk            (0xFFUL << I3C_DBSTSLV_RXLV_Pos)

#define I3C_PRESENTS_SCLLEL_Pos         (0U)
#define I3C_PRESENTS_SCLLEL_Msk         (1UL << I3C_PRESENTS_SCLLEL_Pos)

#define I3C_PRESENTS_SDALEL_Pos         (1U)
#define I3C_PRESENTS_SDALEL_Msk         (1UL << I3C_PRESENTS_SDALEL_Pos)

#define I3C_PRESENTS_TFRTYPE_Pos        (8U)
#define I3C_PRESENTS_TFRTYPE_Msk        (0x3FUL << I3C_PRESENTS_TFRTYPE_Pos)

#define I3C_PRESENTS_CMTFRSTS_Pos       (16U)
#define I3C_PRESENTS_CMTFRSTS_Msk       (0x3FUL << I3C_PRESENTS_CMTFRSTS_Pos)

#define I3C_PRESENTS_TID_Pos            (24U)
#define I3C_PRESENTS_TID_Msk            (0xFUL << I3C_PRESENTS_TID_Pos)

#define I3C_PRESENTS_MAIDLE_Pos         (28U)
#define I3C_PRESENTS_MAIDLE_Msk         (1UL << I3C_PRESENTS_MAIDLE_Pos)

#define I3C_CCCDEVS_PENDINT_Pos         (0U)
#define I3C_CCCDEVS_PENDINT_Msk         (0xFUL << I3C_CCCDEVS_PENDINT_Pos)

#define I3C_CCCDEVS_PROTERR_Pos         (5U)
#define I3C_CCCDEVS_PROTERR_Msk         (1UL << I3C_CCCDEVS_PROTERR_Pos)

#define I3C_CCCDEVS_ACTMODE_Pos         (6U)
#define I3C_CCCDEVS_ACTMODE_Msk         (3UL << I3C_CCCDEVS_ACTMODE_Pos)

#define I3C_CCCDEVS_UDFERR_Pos          (8U)
#define I3C_CCCDEVS_UDFERR_Msk          (1UL << I3C_CCCDEVS_UDFERR_Pos)

#define I3C_CCCDEVS_SLVBUSY_Pos         (9U)
#define I3C_CCCDEVS_SLVBUSY_Msk         (1UL << I3C_CCCDEVS_SLVBUSY_Pos)

#define I3C_CCCDEVS_OVFERR_Pos          (10U)
#define I3C_CCCDEVS_OVFERR_Msk          (1UL << I3C_CCCDEVS_OVFERR_Pos)

#define I3C_CCCDEVS_DATNRDY_Pos         (11U)
#define I3C_CCCDEVS_DATNRDY_Msk         (1UL << I3C_CCCDEVS_DATNRDY_Pos)

#define I3C_CCCDEVS_BFNAVAIL_Pos        (12U)
#define I3C_CCCDEVS_BFNAVAIL_Msk        (1UL << I3C_CCCDEVS_BFNAVAIL_Pos)

#define I3C_CCCDEVS_FRAMEERR_Pos        (13U)
#define I3C_CCCDEVS_FRAMEERR_Msk        (1UL << I3C_CCCDEVS_FRAMEERR_Pos)

#define I3C_ADDRTP_STARTADDR_Pos        (0U)
#define I3C_ADDRTP_STARTADDR_Msk        (0xFFFFUL << I3C_ADDRTP_STARTADDR_Pos)

#define I3C_ADDRTP_TABLEDEPTH_Pos       (16U)
#define I3C_ADDRTP_TABLEDEPTH_Msk       (0xFFFFUL << I3C_ADDRTP_TABLEDEPTH_Pos)

#define I3C_CHRTP_CHARSTART_Pos         (0U)
#define I3C_CHRTP_CHARSTART_Msk         (0xFFFUL << I3C_CHRTP_CHARSTART_Pos)

#define I3C_CHRTP_CHARDEPTH_Pos         (12U)
#define I3C_CHRTP_CHARDEPTH_Msk         (0x7FUL << I3C_CHRTP_CHARDEPTH_Pos)

#define I3C_CHRTP_CHARINDEX_Pos         (19U)
#define I3C_CHRTP_CHARINDEX_Msk         (0x1FFFUL << I3C_CHRTP_CHARINDEX_Pos)

#define I3C_VENDOR_VENODRADDR_Pos       (0U)
#define I3C_VENDOR_VENODRADDR_Msk       (0xFFFFUL << I3C_VENDOR_VENODRADDR_Pos)

#define I3C_SLVMID_PIDTYPE_Pos          (0U)
#define I3C_SLVMID_PIDTYPE_Msk          (1UL << I3C_SLVMID_PIDTYPE_Pos)

#define I3C_SLVMID_MID_Pos              (1U)
#define I3C_SLVMID_MID_Msk              (0x7FFFUL << I3C_SLVMID_MID_Pos)

#define I3C_SLVPID_DCR_Pos              (0U)
#define I3C_SLVPID_DCR_Msk              (0xFFFUL << I3C_SLVPID_DCR_Pos)

#define I3C_SVLPID_INSTID_Pos           (12U)
#define I3C_SVLPID_INSTID_Msk           (0xFUL << I3C_SVLPID_INSTID_Pos)

#define I3C_SLVPID_PARTID_Pos           (16U)
#define I3C_SLVPID_PARTID_Msk           (0xFFFFUL << I3C_SLVPID_PARTID_Pos)

#define I3C_SLVCHAR_MAXDSLIM_Pos        (0U)
#define I3C_SLVCHAR_MAXDSLIM_Msk        (1UL << I3C_SLVCHAR_MAXDSLIM_Pos)

#define I3C_SLVCHAR_IBICAP_Pos          (1U)
#define I3C_SLVCHAR_IBICAP_Msk          (1UL << I3C_SLVCHAR_IBICAP_Pos)

#define I3C_SLVCHAR_IBIPL_Pos           (2U)
#define I3C_SLVCHAR_IBIPL_Msk           (1UL << I3C_SLVCHAR_IBIPL_Pos)

#define I3C_SLVCHAR_OLCAP_Pos           (3U)
#define I3C_SLVCHAR_OLCAP_Msk           (1UL << I3C_SLVCHAR_OLCAP_Pos)

#define I3C_SLVCHAR_BRIDGEID_Pos        (4U)
#define I3C_SLVCHAR_BRIDGEID_Msk        (1UL << I3C_SLVCHAR_BRIDGEID_Pos)

#define I3C_SLVCHAR_HDRCAP_Pos          (5U)
#define I3C_SLVCHAR_HDRCAP_Msk          (1UL << I3C_SLVCHAR_HDRCAP_Pos)

#define I3C_SLVCHAR_DEVROLE_Pos         (6U)
#define I3C_SLVCHAR_DEVROLE_Msk         (0x3UL << I3C_SLVCHAR_DEVROLE_Pos)

#define I3C_SLVCHAR_DCR_Pos             (8U)
#define I3C_SLVCHAR_DCR_Msk             (0xFFUL << I3C_SLVCHAR_DCR_Pos)

#define I3C_SLVCHAR_HDRCPVAL_Pos        (16U)
#define I3C_SLVCHAR_HDRCPVAL_Msk        (0xFFUL << I3C_SLVCHAR_HDRCPVAL_Pos)

#define I3C_SLVMXLEN_MWL_Pos            (0U)
#define I3C_SLVMXLEN_MWL_Msk            (0xFFFFUL << I3C_SLVMXLEN_MWL_Pos)

#define I3C_SLVMXLEN_MRL_Pos            (16U)
#define I3C_SLVMXLEN_MRL_Msk            (0xFFFFUL << I3C_SLVMXLEN_MRL_Pos)

#define I3C_MXRDTURN_MXRDTURN_Pos       (0U)
#define I3C_MXRDTURN_MXRDTURN_Msk       (0xFFFFFFUL << I3C_MXRDTURN_MXRDTURN_Pos)

#define I3C_MXDS_MXWR_Pos               (0U)
#define I3C_MXDS_MXWR_Msk               (0x7UL << I3C_MXDS_MXWR_Pos)

#define I3C_MXDS_MXRD_Pos               (8U)
#define I3C_MXDS_MXRD_Msk               (0x7UL << I3C_MXDS_MXRD_Pos)

#define I3C_MXDS_MXDTURN_Pos            (16U)
#define I3C_MXDS_MXDTURN_Msk            (0x7UL << I3C_MXDS_MXDTURN_Pos)

#define I3C_SIR_EN_Pos                  (0U)
#define I3C_SIR_EN_Msk                  (1UL << I3C_SIR_EN_Pos)

#define I3C_SIR_CTL_Pos                 (1U)
#define I3C_SIR_CTL_Msk                 (0x3UL << I3C_SIR_CTL_Pos)

#define I3C_SIR_MR_Pos                  (3U)
#define I3C_SIR_MR_Msk                  (0x1UL << I3C_SIR_MR_Pos)

#define I3C_SIR_MDB_Pos                 (8U)
#define I3C_SIR_MDB_Msk                 (0xFFUL << I3C_SIR_MDB_Pos)

#define I3C_SIR_DATLEN_Pos              (16U)
#define I3C_SIR_DATLEN_Msk              (0xFFUL << I3C_SIR_DATLEN_Pos)

#define I3C_SIRDAT_DAT0_Pos             (0U)
#define I3C_SIRDAT_DAT0_Msk             (0xFFUL << I3C_SIRDAT_DAT0_Pos)

#define I3C_SIRDAT_DAT1_Pos             (8U)
#define I3C_SIRDAT_DAT1_Msk             (0xFFUL << I3C_SIRDAT_DAT1_Pos)

#define I3C_SIRDAT_DAT2_Pos             (16U)
#define I3C_SIRDAT_DAT2_Msk             (0xFFUL << I3C_SIRDAT_DAT2_Pos)

#define I3C_SIRDAT_DAT3_Pos             (24U)
#define I3C_SIRDAT_DAT3_Msk             (0xFFUL << I3C_SIRDAT_DAT3_Pos)

#define I3C_SIRRESP_IBISTS_Pos          (0U)
#define I3C_SIRRESP_IBISTS_Msk          (0x7UL << I3C_SIRRESP_IBISTS_Pos)

#define I3C_SIRRESP_DATLEN_Pos          (8U)
#define I3C_SIRRESP_DATLEN_Msk          (0xFFFFUL << I3C_SIRRESP_DATLEN_Pos)

#define I3C_DEVCTLE_OPERMODE_Pos        (0U)
#define I3C_DEVCTLE_OPERMODE_Msk        (0x3UL << I3C_DEVCTLE_OPERMODE_Pos)

#define I3C_SCLOD_ODLCNT_Pos            (0U)
#define I3C_SCLOD_ODLCNT_Msk            (0xFFUL << I3C_SCLOD_ODLCNT_Pos)

#define I3C_SCLOD_ODHCNT_Pos            (16U)
#define I3C_SCLOD_ODHCNT_Msk            (0xFFUL << I3C_SCLOD_ODHCNT_Pos)

#define I3C_SCLPP_PPLCNT_Pos            (0U)
#define I3C_SCLPP_PPLCNT_Msk            (0xFFUL << I3C_SCLPP_PPLCNT_Pos)

#define I3C_SCLPP_PPHCNT_Pos            (16U)
#define I3C_SCLPP_PPHCNT_Msk            (0xFFUL << I3C_SCLPP_PPHCNT_Pos)

#define I3C_SCLFM_FMLCNT_Pos            (0U)
#define I3C_SCLFM_FMLCNT_Msk            (0xFFFFUL << I3C_SCLFM_FMLCNT_Pos)

#define I3C_SCLFM_FMHCNT_Pos            (16U)
#define I3C_SCLFM_FMHCNT_Msk            (0xFFFFUL << I3C_SCLFM_FMHCNT_Pos)

#define I3C_SCLFMP_FMPLCNT_Pos          (0U)
#define I3C_SCLFMP_FMPLCNT_Msk          (0xFFFFUL << I3C_SCLFMP_FMPLCNT_Pos)

#define I3C_SCLFMP_FMPHCNT_Pos          (16U)
#define I3C_SCLFMP_FMPHCNT_Msk          (0xFFUL << I3C_SCLFMP_FMPHCNT_Pos)

#define I3C_SCLEXTL_EXLCNT1_Pos         (0U)
#define I3C_SCLEXTL_EXLCNT1_Msk         (0xFFUL << I3C_SCLEXTL_EXLCNT1_Pos)

#define I3C_SCLEXTL_EXLCNT2_Pos         (8U)
#define I3C_SCLEXTL_EXLCNT2_Msk         (0xFFUL << I3C_SCLEXTL_EXLCNT2_Pos)

#define I3C_SCLEXTL_EXLCNT3_Pos         (16U)
#define I3C_SCLEXTL_EXLCNT3_Msk         (0xFFUL << I3C_SCLEXTL_EXLCNT3_Pos)

#define I3C_SCLEXTL_EXLCNT4_Pos         (24U)
#define I3C_SCLEXTL_EXLCNT4_Msk         (0xFFUL << I3C_SCLEXTL_EXLCNT4_Pos)

#define I3C_SCLTERM_EXTERM_Pos          (0U)
#define I3C_SCLTERM_EXTERM_Msk          (0xFUL << I3C_SCLTERM_EXTERM_Pos)

#define I3C_SCLTERM_STOPHLD_Pos         (28U)
#define I3C_SCLTERM_STOPHLD_Msk         (0xFUL << I3C_SCLTERM_STOPHLD_Pos)

#define I3C_SDAHOLD_TXHOLD_Pos          (16U)
#define I3C_SDAHOLD_TXHOLD_Msk          (0x7UL << I3C_SDAHOLD_TXHOLD_Pos)

#define I3C_BUSFAT_AVAILTC_Pos          (16U)
#define I3C_BUSFAT_AVAILTC_Msk          (0xFFFFUL << I3C_BUSFAT_AVAILTC_Pos)

#define I3C_BUSIDLET_IDLETC_Pos         (0U)
#define I3C_BUSIDLET_IDLETC_Msk         (0xFFFFFUL << I3C_BUSIDLET_IDLETC_Pos)

#define I3C_SCLLOW_LOWTIMER_Pos         (0U)
#define I3C_SCLLOW_LOWTIMER_Msk         (0xFFFFFFFFUL << I3C_SCLLOW_LOWTIMER_Pos)

#define I3C_DEVADR_SADDR_Pos            (0U)
#define I3C_DEVADR_SADDR_Msk            (0x7FUL << I3C_DEVADR_SADDR_Pos)

#define I3C_DEVADR_IBIPECEN_Pos         (11U)
#define I3C_DEVADR_IBIPECEN_Msk         (0x1UL << I3C_DEVADR_IBIPECEN_Pos)

#define I3C_DEVADR_IBIWDAT_Pos          (12U)
#define I3C_DEVADR_IBIWDAT_Msk          (0x1UL << I3C_DEVADR_IBIWDAT_Pos)

#define I3C_DEVADR_SIRREJECT_Pos        (13U)
#define I3C_DEVADR_SIRREJECT_Msk        (0x1UL << I3C_DEVADR_SIRREJECT_Pos)

#define I3C_DEVADR_MRREJECT_Pos         (14U)
#define I3C_DEVADR_MRREJECT_Msk         (0x1UL << I3C_DEVADR_MRREJECT_Pos)

#define I3C_DEVADR_DADDR_Pos            (16U)
#define I3C_DEVADR_DADDR_Msk            (0xFFUL << I3C_DEVADR_DADDR_Pos)

#define I3C_DEVADR_NAKRCNT_Pos          (29U)
#define I3C_DEVADR_NAKRCNT_Msk          (0x3UL << I3C_DEVADR_NAKRCNT_Pos)

#define I3C_DEVADR_DEVICE_Pos           (31U)
#define I3C_DEVADR_DEVICE_Msk           (0x1UL << I3C_DEVADR_DEVICE_Pos)

/** @} I3C_CONST */
/** @} end of I3C register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __I3C_REG_H__ */
