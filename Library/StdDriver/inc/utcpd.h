/****************************************************************************//**
 * @file     utcpd.h
 * @version  V1.00
 * @brief    M55M1 series UTCPD driver header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __UTCPD_H__
#define __UTCPD_H__

#include "NuMicro.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup UTCPD_Driver UTCPD Driver
  @{
*/

/** @addtogroup UTCPD_EXPORTED_CONSTANTS UTCPD Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  ALERT constant definitions.                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_ALERT_CCSCHIS          UTCPD_IS_CCSCHIS_Msk                             /*!< UTCPD_T::ALERT: CCSCHIS Mask          */
#define UTCPD_ALERT_PWRSCHIS         UTCPD_IS_PWRSCHIS_Msk                            /*!< UTCPD_T::ALERT: PWRSCHIS Mask         */
#define UTCPD_ALERT_RXSOPIS          UTCPD_IS_RXSOPIS_Msk                             /*!< UTCPD_T::ALERT: RXSOPIS Mask          */
#define UTCPD_ALERT_RXHRSTIS         UTCPD_IS_RXHRSTIS_Msk                            /*!< UTCPD_T::ALERT: RXHRSTIS Mask         */
#define UTCPD_ALERT_TXFAILIS         UTCPD_IS_TXFALIS_Msk                             /*!< UTCPD_T::ALERT: TXFAILIS Mask         */
#define UTCPD_ALERT_TXDCUIS          UTCPD_IS_TXDCUDIS_Msk                            /*!< UTCPD_T::ALERT: TXDCUIS Mask          */
#define UTCPD_ALERT_TXOKIS           UTCPD_IS_TXOKIS_Msk                              /*!< UTCPD_T::ALERT: TXOKIS Mask           */
#define UTCPD_ALERT_VBAMHIS          UTCPD_IS_VBAMHIS_Msk                             /*!< UTCPD_T::ALERT: VBAMHIS Mask          */
#define UTCPD_ALERT_VBAMLIS          UTCPD_IS_VBAMLIS_Msk                             /*!< UTCPD_T::ALERT: VBAMLIS Mask          */
#define UTCPD_ALERT_FUTIS            UTCPD_IS_FUTIS_Msk                               /*!< UTCPD_T::ALERT: FUTIS Mask            */
#define UTCPD_ALERT_RXOFIS           UTCPD_IS_RXOFIS_Msk                              /*!< UTCPD_T::ALERT: RXOFIS Mask           */
#define UTCPD_ALERT_SKDCDTIS         UTCPD_IS_SKDCDTIS_Msk                            /*!< UTCPD_T::ALERT: SKDCDTIS Mask         */
#define UTCPD_ALERT_VNDIS            UTCPD_IS_VNDIS_Msk                               /*!< UTCPD_T::ALERT: VNDIS Mask            */


/*---------------------------------------------------------------------------------------------------------*/
/*  ALERT Mask constant definitions.                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_ALERTM_CCSCHIE         UTCPD_IE_CCSCHIE_Msk                             /*!< UTCPD_T::ALERTM: CCSCHIE Mask         */
#define UTCPD_ALERTM_PWRSCHIE        UTCPD_IE_PWRSCHIE_Msk                            /*!< UTCPD_T::ALERTM: PWRSCHIE Mask        */
#define UTCPD_ALERTM_RXSOPIE         UTCPD_IE_RXSOPIE_Msk                             /*!< UTCPD_T::ALERTM: RXSOPIE Mask         */
#define UTCPD_ALERTM_RXHRSTIE        UTCPD_IE_RXHRSTIE_Msk                            /*!< UTCPD_T::ALERTM: RXHRSTIE Mask        */
#define UTCPD_ALERTM_TXFAILIE        UTCPD_IE_TXFAILIE_Msk                            /*!< UTCPD_T::ALERTM: TXFAILIE Mask        */
#define UTCPD_ALERTM_TXDCUIE         UTCPD_IE_TXDCUDIE_Msk                            /*!< UTCPD_T::ALERTM: TXDCUIE Mask         */
#define UTCPD_ALERTM_TXSOKIE         UTCPD_IE_TXSOKIE_Msk                             /*!< UTCPD_T::ALERTM: TXSOKIE Mask          */
#define UTCPD_ALERTM_VBAMHIE         UTCPD_IE_VBAMHIE_Msk                             /*!< UTCPD_T::ALERTM: VBAMHIE Mask         */
#define UTCPD_ALERTM_VBAMLIE         UTCPD_IE_VBAMLIE_Msk                             /*!< UTCPD_T::ALERTM: VBAMLIE Mask         */
#define UTCPD_ALERTM_FUTIE           UTCPD_IE_FUTIE_Msk                               /*!< UTCPD_T::ALERTM: FUTIE Mask           */
#define UTCPD_ALERTM_RXOFIE          UTCPD_IE_RXOFIE_Msk                              /*!< UTCPD_T::ALERTM: RXOFIE Mask          */
#define UTCPD_ALERTM_SKDCDTIE        UTCPD_IE_SKDCDTIE_Msk                            /*!< UTCPD_T::ALERTM: SKDCDTIE Mask        */
#define UTCPD_ALERTM_VNDIE           UTCPD_IE_VNDIE_Msk                               /*!< UTCPD_T::ALERTM: VNDIE Mask           */

/*---------------------------------------------------------------------------------------------------------*/
/* PWRSM Power Status Mask constant definitions.                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_PWRSM_SKVBIE           UTCPD_PWRSTSIE_SKVBIE_Msk                        /*!< UTCPD_T::PWRSM: SKVBIE Mask            */
#define UTCPD_PWRSM_VCPSIE           UTCPD_PWRSTSIE_VCPSIE_Msk                        /*!< UTCPD_T::PWRSM: VCPSIE Mask            */
#define UTCPD_PWRSM_VBPSIE           UTCPD_PWRSTSIE_VBPSIE_Msk                        /*!< UTCPD_T::PWRSM: VBPSIE Mask            */
#define UTCPD_PWRSM_VBDTDGIE         UTCPD_PWRSTSIE_VBDTDGIE_Msk                      /*!< UTCPD_T::PWRSM: VBDTDGIE Mask          */
#define UTCPD_PWRSM_SRVBIE           UTCPD_PWRSTSIE_SRVBIE_Msk                        /*!< UTCPD_T::PWRSM: SRVBIE Mask            */
#define UTCPD_PWRSM_SRHVIE           UTCPD_PWRSTSIE_SRHVIE_Msk                        /*!< UTCPD_T::PWRSM: SRHVIE Mask            */
#define UTCPD_PWRSM_DACONIE          UTCPD_PWRSTSIE_DACONIE_Msk                       /*!< UTCPD_T::PWRSM: DACONIE Mask           */


/*---------------------------------------------------------------------------------------------------------*/
/*  FAULTSM Fault Status Mask constant definitions.                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_FAULTSM_VCOCIE         UTCPD_FUTSTSIE_VCOCIE_Msk                        /*!< UTCPD_T::FAULTSM: VCOCIE Mask          */
#define UTCPD_FAULTSM_VBOVIE         UTCPD_FUTSTSIE_VBOVIE_Msk                        /*!< UTCPD_T::FAULTSM: VBOVIE Mask          */
#define UTCPD_FAULTSM_VBOCIE         UTCPD_FUTSTSIE_VBOCIE_Msk                        /*!< UTCPD_T::FAULTSM: VBOCIE Mask          */
#define UTCPD_FAULTSM_FDGFALIE       UTCPD_FUTSTSIE_FDGFALIE_Msk                      /*!< UTCPD_T::FAULTSM: FDGFALIE Mask        */
#define UTCPD_FAULTSM_ADGFALIE       UTCPD_FUTSTSIE_ADGFALIE_Msk                      /*!< UTCPD_T::FAULTSM: ADGFALIE Mask        */
#define UTCPD_FAULTSM_FOFFVBIE       UTCPD_FUTSTSIE_FOFFVBIE_Msk                      /*!< UTCPD_T::FAULTSM: FOFFVBIE Mask        */

/*---------------------------------------------------------------------------------------------------------*/
/*  FAULTCTL constant definitions.                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_FAULTCTL_VCOCDTDS      UTCPD_FUTCTL_VCOCDTDS_Msk                        /*!< UTCPD_T::FAULTCTL: VCOCDTDS Mask       */
#define UTCPD_FAULTCTL_VBOVDTDS      UTCPD_FUTCTL_VBOVDTDS_Msk                        /*!< UTCPD_T::FAULTCTL: VBOVDTDS Mask       */
#define UTCPD_FAULTCTL_VBOCDTDS      UTCPD_FUTCTL_VBOCDTDS_Msk                        /*!< UTCPD_T::FAULTCTL: VBOCDTDS Position   */
#define UTCPD_FAULTCTL_VBDGTMDS      UTCPD_FUTCTL_VBDGTMDS_Msk                        /*!< UTCPD_T::FAULTCTL: VBDGTMDS Mask       */
#define UTCPD_FAULTCTL_FOFFVBDS      UTCPD_FUTCTL_FOFFVBDS_Msk                        /*!< UTCPD_T::FAULTCTL: FOFFVBDS Mask       */

/*---------------------------------------------------------------------------------------------------------*/
/*  PWRCTL constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_PWRCTL_VCEN            UTCPD_PWRCTL_VCEN_Msk                            /*!< UTCPD_T::PWRCTL: VCEN Mask             */
#define UTCPD_PWRCTL_VCPWR           UTCPD_PWRCTL_VCPWR_Msk                           /*!< UTCPD_T::PWRCTL: VCPWR Mask            */
#define UTCPD_PWRCTL_FDGEN           UTCPD_PWRCTL_FDGEN_Msk                           /*!< UTCPD_T::PWRCTL: FDGEN Mask            */
#define UTCPD_PWRCTL_BDGEN           UTCPD_PWRCTL_BDGEN_Msk                           /*!< UTCPD_T::PWRCTL: BDGEN Mask            */
#define UTCPD_PWRCTL_ADGDC           UTCPD_PWRCTL_ADGDC_Msk                           /*!< UTCPD_T::PWRCTL: ADGDC Mask            */
#define UTCPD_PWRCTL_DSVBAM_DIS      UTCPD_PWRCTL_DSVBAM_Msk                          /*!< UTCPD_T::PWRCTL: DSVBAM Mask           */
#define UTCPD_PWRCTL_VBMONI_DIS      UTCPD_PWRCTL_VBMONI_Msk                          /*!< UTCPD_T::PWRCTL: VBMONI Mask           */

/*---------------------------------------------------------------------------------------------------------*/
/*  TCPCCTL constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_TCPCCTL_ORIENT         UTCPD_CTL_ORIENT_Msk                             /*!< UTCPD_T::TCPCCTL: ORIENT Mask          */
#define UTCPD_TCPCCTL_BISTEN         UTCPD_CTL_BISTEN_Msk                             /*!< UTCPD_T::TCPCCTL: BISTEN Mask          */

/*---------------------------------------------------------------------------------------------------------*/
/*  ROLECTL constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_ROLECTL_CC1            UTCPD_ROLCTL_CC1_Msk                             /*!< UTCPD_T::ROLECTL: CC1 Mask             */
#define UTCPD_ROLECTL_CC2            UTCPD_ROLCTL_CC2_Msk                             /*!< UTCPD_T::ROLECTL: CC2 Mask             */
#define UTCPD_ROLECTL_RPVALUE        UTCPD_ROLCTL_RPVALUE_Msk                         /*!< UTCPD_T::ROLECTL: RPVALUE Mask         */
#define UTCPD_ROLECTL_DRP            UTCPD_ROLCTL_DRP_Msk                             /*!< UTCPD_T::ROLECTL: DRP Mask             */

#define UTCPD_ROLECTL_CC1_RA        (0 << 0)
#define UTCPD_ROLECTL_CC1_RP        (1 << 0)
#define UTCPD_ROLECTL_CC1_RD        (2 << 0)
#define UTCPD_ROLECTL_CC1_OPEN      (3 << 0)

#define UTCPD_ROLECTL_CC2_RA        (0 << 2)
#define UTCPD_ROLECTL_CC2_RP        (1 << 2)
#define UTCPD_ROLECTL_CC2_RD        (2 << 2)
#define UTCPD_ROLECTL_CC2_OPEN      (3 << 2)

#define UTCPD_ROLECTL_RPVALUE_DEF   (0 << 4)
#define UTCPD_ROLECTL_RPVALUE_1P5A  (1 << 4)
#define UTCPD_ROLECTL_RPVALUE_3A    (2 << 4)

/*---------------------------------------------------------------------------------------------------------*/
/*  CCSTS constant definitions.                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_CCSTS_CC1STATE         UTCPD_CCSTS_CC1STATE_Msk                         /*!< UTCPD_T::CCSTS: CC1STATE Mask          */
#define UTCPD_CCSTS_CC2STATE         UTCPD_CCSTS_CC2STATE_Msk                         /*!< UTCPD_T::CCSTS: CC2STATE Mask          */
#define UTCPD_CCSTS_CONRLT           UTCPD_CCSTS_CONRLT_Msk                           /*!< UTCPD_T::CCSTS: CONRLT Mask            */
#define UTCPD_CCSTS_LK4CONN          UTCPD_CCSTS_LK4CONN_Msk                          /*!< UTCPD_T::CCSTS: LK4CONN Mask           */

#define UTCPD_UNDER_LK4CONN         (1 << 5)
#define UTCPD_LK4CONN_DONE          (0 << 5)

#define UTCPD_CONN_RESULT_RP        (0 << 4)
#define UTCPD_CONN_RESULT_RD        (1 << 4)

#define UTCPD_CCSTS_CC2STATE_SRC_OPEN       (0 << 2)
#define UTCPD_CCSTS_CC2STATE_SRC_RA         (1 << 2)
#define UTCPD_CCSTS_CC2STATE_SRC_RD         (2 << 2)

#define UTCPD_CCSTS_CC2STATE_SNK_OPEN       (0 << 2)
#define UTCPD_CCSTS_CC2STATE_SNK_DEF        (1 << 2)
#define UTCPD_CCSTS_CC2STATE_SNK_1P5A       (2 << 2)
#define UTCPD_CCSTS_CC2STATE_SNK_3A         (3 << 2)

#define UTCPD_CCSTS_CC1STATE_SRC_OPEN       (0 << 0)
#define UTCPD_CCSTS_CC1STATE_SRC_RA         (1 << 0)
#define UTCPD_CCSTS_CC1STATE_SRC_RD         (2 << 0)

#define UTCPD_CCSTS_CC1STATE_SNK_OPEN       (0 << 0)
#define UTCPD_CCSTS_CC1STATE_SNK_DEF        (1 << 0)
#define UTCPD_CCSTS_CC1STATE_SNK_1P5A       (2 << 0)
#define UTCPD_CCSTS_CC1STATE_SNK_3A         (3 << 0)

/*---------------------------------------------------------------------------------------------------------*/
/*  PWRSTS constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_PWRSTS_SKVB            UTCPD_PWRSTS_SKVB_Msk                            /*!< UTCPD_T::PWRSTS: SKVB Mask             */
#define UTCPD_PWRSTS_VCPS            UTCPD_PWRSTS_VCPS_Msk                            /*!< UTCPD_T::PWRSTS: VCPS Mask             */
#define UTCPD_PWRSTS_VBPS            UTCPD_PWRSTS_VBPS_Msk                            /*!< UTCPD_T::PWRSTS: VBPS Mask             */
#define UTCPD_PWRSTS_VBPSDTEN        UTCPD_PWRSTS_VBPSDTEN_Msk                        /*!< UTCPD_T::PWRSTS: VBPSDTEN Mask         */
#define UTCPD_PWRSTS_SRVB            UTCPD_PWRSTS_SRVB_Msk                            /*!< UTCPD_T::PWRSTS: SRVB Mask             */
#define UTCPD_PWRSTS_SRHV            UTCPD_PWRSTS_SRHV_Msk                            /*!< UTCPD_T::PWRSTS: SRHV Mask             */
#define UTCPD_PWRSTS_DACON           UTCPD_PWRSTS_DACON_Msk                           /*!< UTCPD_T::PWRSTS: DACON Mask            */

/*---------------------------------------------------------------------------------------------------------*/
/*  FAULTSTS constant definitions.                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

#define UTCPD_FAULTSTS_VCOCFUT       UTCPD_FUTSTS_VCOCFUT_Pos                         /*!< UTCPD_T::FAULTSTS: VCOCFUT Mask        */
#define UTCPD_FAULTSTS_VBOVFUT       UTCPD_FUTSTS_VBOVFUT_Msk                         /*!< UTCPD_T::FAULTSTS: VBOVFUT Mask        */
#define UTCPD_FAULTSTS_VBOCFUT       UTCPD_FUTSTS_VBOCFUT_Msk                         /*!< UTCPD_T::FAULTSTS: VBOCFUT Mask        */
#define UTCPD_FAULTSTS_FDGFAL        UTCPD_FUTSTS_FDGFAL_Msk                          /*!< UTCPD_T::FAULTSTS: FDGFAL Mask         */
#define UTCPD_FAULTSTS_ADGFAL        UTCPD_FUTSTS_ADGFAL_Msk                          /*!< UTCPD_T::FAULTSTS: ADGFAL Mask         */
#define UTCPD_FAULTSTS_FOFFVB        UTCPD_FUTSTS_FOFFVB_Msk                          /*!< UTCPD_T::FAULTSTS: FOFFVB Mask         */

/*---------------------------------------------------------------------------------------------------------*/
/*  CMD constant definitions.                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
//#define NPD48_CMD_WAKEI2C                   (0x11) /* Not Support */
#define UTCPD_CMD_DISABLE_VBUS_DETECT       (0x22)
#define UTCPD_CMD_ENABLE_VBUS_DETECT        (0x33)
#define UTCPD_CMD_DISABLE_SINK_VBUS         (0x44)
#define UTCPD_CMD_SINK_VBUS                 (0x55)
#define UTCPD_CMD_DISABLE_SRC_VBUS          (0x66)
#define UTCPD_CMD_SRC_VBUS_DEFAULT          (0x77)
#define UTCPD_CMD_SRC_VBUS_NONDEFAULT       (0x88)
#define UTCPD_CMD_LOOK4CONNECTION           (0x99)
//#define UTCPD_CMD_RX_ONE_MORE               (0xAA)    /* Not Support */
//#define UTCPD_CMD_SEND_FRS_SWAP_SIGNAL      (0xCC)    /* Not Support */
//#define UTCPD_CMD_SEND_RESET_TX_BUF         (0xDD)    /* Not Support */
//#define UTCPD_CMD_SEND_RESET_RX_BUF         (0xEE)    /* Not Support */
//#define UTCPD_CMD_I2C_IDLE                  (0xFF)    /* Not Support */

/*---------------------------------------------------------------------------------------------------------*/
/*  NPD48_MHINFO constant definitions.                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_MHINFO_DROLE_UFP              (0x0UL << UTCPD_MSHEAD_DAROL_Pos)         /*!< UTCPD_T::MHINFO: DROLE Position        */
#define UTCPD_MHINFO_DROLE_DFP              (0x1UL << UTCPD_MSHEAD_DAROL_Pos)         /*!< UTCPD_T::MHINFO: DROLE Position        */

#define UTCPD_MHINFO_PDREV_10               (0x0UL << UTCPD_MSHEAD_PDREV_Pos)         /*!< UTCPD_T::MHINFO: PDREV Position        */
#define UTCPD_MHINFO_PDREV_20               (0x1UL << UTCPD_MSHEAD_PDREV_Pos)         /*!< UTCPD_T::MHINFO: PDREV Position        */
#define UTCPD_MHINFO_PDREV_30               (0x2UL << UTCPD_MSHEAD_PDREV_Pos)         /*!< UTCPD_T::MHINFO: PDREV Position        */
#define UTCPD_MHINFO_PDREV_31               (0x3UL << UTCPD_MSHEAD_PDREV_Pos)         /*!< UTCPD_T::MHINFO: PDREV Position        */

#define UTCPD_MHINFO_PROLE_SNK              (0x0UL << UTCPD_MSHEAD_PWRROL_Pos)        /*!< UTCPD_T::MHINFO: PROLE Position        */
#define UTCPD_MHINFO_PROLE_SRC              (0x1UL << UTCPD_MSHEAD_PWRROL_Pos)        /*!< UTCPD_T::MHINFO: PROLE Position        */

/*---------------------------------------------------------------------------------------------------------*/
/*  RDET constant definitions.                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_RDET_SOPEN                    UTCPD_DTRXEVNT_SOPEN_Msk                  /*!< UTCPD_T::RDET: SOPEN Mask              */
#define UTCPD_RDET_SOPPEN                   UTCPD_DTRXEVNT_SOPPEN_Msk                 /*!< UTCPD_T::RDET: SOPPEN Mask             */
#define UTCPD_RDET_SOPPPEN                  UTCPD_DTRXEVNT_SOPPPEN_Msk                /*!< UTCPD_T::RDET: SOPPPEN Mask            */
#define UTCPD_RDET_SDBGPEN                  UTCPD_DTRXEVNT_SDBGPEN_Msk                /*!< UTCPD_T::RDET: SDBGPEN Mask            */
#define UTCPD_RDET_SDBGPPEN                 UTCPD_DTRXEVNT_SDBGPPEN_Msk               /*!< UTCPD_T::RDET: SDBGPPEN Mask           */
#define UTCPD_RDET_HRSTEN                   UTCPD_DTRXEVNT_HRSTEN_Msk                 /*!< UTCPD_T::RDET: HRSTEN Mask             */
#define UTCPD_RDET_CABRSTEN                 UTCPD_DTRXEVNT_CABRSTEN_Msk               /*!< UTCPD_T::RDET: CABRSTEN Mask           */


/*---------------------------------------------------------------------------------------------------------*/
/*  FSTXCTL constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_FSTXCTL_FSTXEN                UTCPD_FRSRXCTL_FRSTX_Msk                  /*!< UTCPD_T::FSTXCTL: FSTXEN Mask          */
#define UTCPD_FSTXCTL_FSDVVB                UTCPD_FRSRXCTL_FRSDVVB_Msk                /*!< UTCPD_T::FSTXCTL: FSDVVB Mask          */
#define UTCPD_FSTXCTL_FSRXDET               UTCPD_FRSRXCTL_FRSRXEN_Msk                /*!< UTCPD_T::FSTXCTL: FSRXDET Mask         */



/*---------------------------------------------------------------------------------------------------------*/
/*  MUXSEL constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_VCONN_OC_EINT0      (0UL << UTCPD_MUXSEL_VCOCS_Pos)                     /*!< UTCPD_T::MUXSEL: VCONN OC Source From EINT0    */
#define UTCPD_VCONN_OC_EINT1      (1UL << UTCPD_MUXSEL_VCOCS_Pos)                     /*!< UTCPD_T::MUXSEL: VCONN OC Source From EINT1    */
#define UTCPD_VCONN_OC_EINT2      (2UL << UTCPD_MUXSEL_VCOCS_Pos)                     /*!< UTCPD_T::MUXSEL: VCONN OC Source From EINT2    */
#define UTCPD_VCONN_OC_EINT3      (3UL << UTCPD_MUXSEL_VCOCS_Pos)                     /*!< UTCPD_T::MUXSEL: VCONN OC Source From EINT3    */
#define UTCPD_VCONN_OC_ADC_CMP1   (4UL << UTCPD_MUXSEL_VCOCS_Pos)                     /*!< UTCPD_T::MUXSEL: VCONN OC Source From ADC_CMP1 */
#define UTCPD_VCONN_OC_ACMP0      (5UL << UTCPD_MUXSEL_VCOCS_Pos)                     /*!< UTCPD_T::MUXSEL: VCONN OC Source From ACMP0    */
#define UTCPD_VCONN_OC_ACMP1      (6UL << UTCPD_MUXSEL_VCOCS_Pos)                     /*!< UTCPD_T::MUXSEL: VCONN OC Source From ACMP1    */
#define UTCPD_VCONN_OC_ACMP2      (7UL << UTCPD_MUXSEL_VCOCS_Pos)                     /*!< UTCPD_T::MUXSEL: VCONN OC Source From ACMP2    */


#define UTCPD_VBUS_OC_EINT0       (0UL << UTCPD_MUXSEL_VBOCS_Pos)                     /*!< UTCPD_T::MUXSEL: VBUS OC Source From EINT0     */
#define UTCPD_VBUS_OC_EINT1       (1UL << UTCPD_MUXSEL_VBOCS_Pos)                     /*!< UTCPD_T::MUXSEL: VBUS OC Source From EINT1     */
#define UTCPD_VBUS_OC_EINT2       (2UL << UTCPD_MUXSEL_VBOCS_Pos)                     /*!< UTCPD_T::MUXSEL: VBUS OC Source From EINT2     */
#define UTCPD_VBUS_OC_EINT3       (3UL << UTCPD_MUXSEL_VBOCS_Pos)                     /*!< UTCPD_T::MUXSEL: VBUS OC Source From EINT3     */
#define UTCPD_VBUS_OC_ADC_CMP0    (4UL << UTCPD_MUXSEL_VBOCS_Pos)                     /*!< UTCPD_T::MUXSEL: VBUS OC Source From ADC_CMP0  */
#define UTCPD_VBUS_OC_ACMP0       (5UL << UTCPD_MUXSEL_VBOCS_Pos)                     /*!< UTCPD_T::MUXSEL: VBUS OC Source From ACMP0     */
#define UTCPD_VBUS_OC_ACMP1       (6UL << UTCPD_MUXSEL_VBOCS_Pos)                     /*!< UTCPD_T::MUXSEL: VBUS OC Source From ACMP1     */
#define UTCPD_VBUS_OC_ACMP2       (7UL << UTCPD_MUXSEL_VBOCS_Pos)                     /*!< UTCPD_T::MUXSEL: VBUS OC Source From ACMP2     */
/*---------------------------------------------------------------------------------------------------------*/
/*  PWRASTS constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define UTCPD_PWRASTS_SRCENLVL_HIGH  (0x0ul << 0)                                      /*!< UTCPD_T::PWRASTS: SRC Enable Level     */
#define UTCPD_PWRASTS_SRCENLVL_LOW   (0x1ul << 0)                                      /*!< UTCPD_T::PWRASTS: SRC Enable Level     */

#define UTCPD_PWRASTS_SNKENLVL_HIGH  (0x0ul << 1)                                      /*!< UTCPD_T::PWRASTS: SNK Enable Level     */
#define UTCPD_PWRASTS_SNKENLVL_LOW   (0x1ul << 1)                                      /*!< UTCPD_T::PWRASTS: SNK Enable Level     */

#define UTCPD_PWRASTS_FORCEDLVL_HIGH (0x0ul << 2)                                      /*!< UTCPD_T::PWRASTS: Force Discharge Level*/
#define UTCPD_PWRASTS_FORCEDLVL_LOW  (0x1ul << 2)                                      /*!< UTCPD_T::PWRASTS: Force Discharge Level*/

#define UTCPD_PWRASTS_BLEEDDLVL_HIGH (0x0ul << 3)                                      /*!< UTCPD_T::PWRASTS: Bleed Discharge Level*/
#define UTCPD_PWRASTS_BLEEDDLVL_LOW  (0x1ul << 3)                                      /*!< UTCPD_T::PWRASTS: Bleed Discharge Level*/

/** @} end of group UTCPD_EXPORTED_CONSTANTS */

/** @addtogroup UTCPD_EXPORTED_FUNCTIONS UTCPD Exported Functions
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* Define utcpd functions prototype                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
int32_t UTCPD_GetAlertStatus(int port, int *i32AlertSts);
int32_t UTCPD_ClearAlertStatus(int port, int AlertStClr);
int32_t UTCPD_EnableAlertMask(int port, int mask_set);
int32_t UTCPD_DisableAlertMask(int port, int mask_clr);
int32_t UTCPD_EnablePowerStatusMask(int port, int mask_set);
int32_t UTCPD_DisablePowerStatusMask(int port, int mask_clr);
int32_t UTCPD_EnableFaultMask(int port, int mask_set);
int32_t UTCPD_DisableFaultMask(int port, int mask_clr);
int32_t UTCPD_EnablePowerCtrl(int port, uint32_t mask_set);
int32_t UTCPD_DisablePowerCtrl(int port, uint32_t maskclr);
int32_t UTCPD_EnableFaultCtrl(int port, uint32_t mask_set);
int32_t UTCPD_DisableFaultCtrl(int port, uint32_t maskclr);
int32_t UTCPD_ClearPowerStatus(int port, int PowerStClr);
int32_t UTCPD_EnableFaultStatusMask(int port, int mask_set);
int32_t UTCPD_DisableFaultStatusMask(int port, int mask_clr);
int32_t UTCPD_GetFaultStatus(int port, int *pi32RegData);
int32_t UTCPD_ClearFaultStatus(int port, int FaultStClr);
int32_t UTCPD_SetRoleCtrl(int port, uint32_t u32DrpToggle, uint32_t u32Rpvalue, uint32_t u32CC2, uint32_t u32CC1);
int32_t UTCPD_GetRoleCtrl(int port, uint32_t *pu32DrpToggle, uint32_t *pu32CC1, uint32_t *pu32CC2, uint32_t *pu32Rpvalue);
int32_t UTCPD_SetTypeCPortCtrl(int port, uint32_t u32BistMode, uint32_t u32Orient);
int32_t UTCPD_GetTypeCPortCtrl(int port, uint32_t *pu32BistMode, uint32_t *pu32Orient);
int32_t UTCPD_IsssueCmd(int port, uint32_t cmd);
int32_t UTCPD_GetCCSts(int port, uint32_t *pu32Look4Con, uint32_t *pu32ConRlt, uint32_t *pu32CC2Sts, uint32_t *pu32CC1Sts);
int32_t UTCPD_GetPwrSts(int port, uint32_t *pu32VBUSDetEn, uint32_t *pu32VBUSPresent, uint32_t *pu32VCONNPresent, uint32_t *pu32SnkVBUS);
int32_t UTCPD_GetPwrStsExt(int port, uint32_t *pu32DbgAccessory, uint32_t *pu32SrcNonDefVBUS, uint32_t *pu32SrcDefVBUS);
int32_t UTCPD_GetFaultSts(int port, uint32_t *pu32VBUSOverCurr, uint32_t *pu32VBUSOverVolt, uint32_t *pu32VCONNOverCurr, uint32_t *pu32I2CInfErr);
int32_t UTCPD_GetFaultStsExt(int port, uint32_t *pu32ForceOffFat, uint32_t *pu32AutoDiscFat, uint32_t *pu32ForceDiscFat);
int32_t UTCPD_SetMsgHeaderInfo(int port, uint32_t u32DataRole, uint32_t u32Revision, uint32_t u32PwrRole);
int32_t UTCPD_SetRecDetect(int port, uint32_t u32RegData);
int32_t UTCPD_SetVBUSAlarm(int port, uint32_t u32AlarmH, uint32_t u32AlarmL);
int32_t UTCPD_SetSnkDisconnect(int port, uint32_t u32SnkDiscVolt);
int32_t UTCPD_SetStopDischargeVolt(int port, uint32_t u32StopDischgVolt);
void UTCPD_vconn_disable_src_cc(int port);
void UTCPD_vconn_enable_src_cc(int port);
void UTCPD_vconn_enable_from_cc2(int port);
void UTCPD_vconn_enable_from_cc1(int port);
void UTCPD_vconn_polarity_active_low(int port);
void UTCPD_vconn_polarity_active_high(int port);
void UTCPD_vconn_disable_oc_fault(int port);
void UTCPD_vconn_enable_oc_fault(int port);
void UTCPD_vconn_mux_selection(int port, uint32_t cc1vcensel, uint32_t cc2vcensel);
void UTCPD_vconn_configure_oc_detection_soruce(int port, uint32_t u32Src);
void UTCPD_vbus_srcen_polarity_active_low(int port);
void UTCPD_vbus_srcen_polarity_active_high(int port);
void UTCPD_vbus_snken_polarity_active_low(int port);
void UTCPD_vbus_snken_polarity_active_high(int port);
void UTCPD_vbus_disable_oc_fault(int port);
void UTCPD_vbus_enable_oc_fault(int port);
void UTCPD_vbus_discharge_polarity_active_low(int port);
void UTCPD_vbus_discharge_polarity_active_high(int port);
void UTCPD_vbus_configure_oc_soruce(int port, uint32_t u32Src);
void UTCPD_vbus_disable_ov_fault(int port);
void UTCPD_vbus_enable_ov_fault(int port);
void UTCPD_vbus_disable_forceoff_fault(int port);
void UTCPD_vbus_enable_forceoff_fault(int port);
uint32_t UTCPD_vbus_is_source(int port);
uint32_t UTCPD_vbus_is_sink(int port);
uint32_t UTCPD_vbus_is_source_hv(int port);
void UTCPD_power_enable_monitor(int port);
void UTCPD_power_disable_monitor(int port);
void UTCPD_power_disable_auto_discharge(int port);
void UTCPD_power_enable_auto_discharge(int port);
void UTCPD_frs_tx_polarity_active_low(int port);
void UTCPD_frs_tx_polarity_active_high(int port);
void UTCPD_frs_mux_selection(int port, uint32_t cc1frssel, uint32_t cc2frssel);
void UTCPD_GetVoltagInfo(int port, uint16_t *pu16VbusVol, uint16_t *pu16VconnVol);
uint32_t UTCPD_Open(int port);


/** @} end of group UTCPD_EXPORTED_FUNCTIONS */

/** @} end of group UTCPD_Driver */

/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif  /* __UTCPD_H__ */

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/