/**************************************************************************//**
 * @file     lppdma.h
 * @version  V1.00
 * @brief    LPPDMA driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __LPPDMA_H__
#define __LPPDMA_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LPPDMA_Driver LPPDMA Driver
  @{
*/

/** @addtogroup LPPDMA_EXPORTED_CONSTANTS LPPDMA Exported Constants
  @{
*/

#define LPPDMA_CH_MAX    4UL   /*!< Specify Maximum Channels of LPPDMA  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Operation Mode Constant Definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define LPPDMA_OP_STOP        0x00000000UL            /*!<DMA Stop Mode  \hideinitializer */
#define LPPDMA_OP_BASIC       0x00000001UL            /*!<DMA Basic Mode  \hideinitializer */
#define LPPDMA_OP_SCATTER     0x00000002UL            /*!<DMA Scatter-gather Mode  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Data Width Constant Definitions                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define LPPDMA_WIDTH_8        0x00000000UL            /*!<DMA Transfer Width 8-bit  \hideinitializer */
#define LPPDMA_WIDTH_16       0x00001000UL            /*!<DMA Transfer Width 16-bit  \hideinitializer */
#define LPPDMA_WIDTH_32       0x00002000UL            /*!<DMA Transfer Width 32-bit  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Address Attribute Constant Definitions                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define LPPDMA_SAR_INC        0x00000000UL            /*!<DMA SAR increment  \hideinitializer */
#define LPPDMA_SAR_FIX        0x00000300UL            /*!<DMA SAR fix address  \hideinitializer */
#define LPPDMA_DAR_INC        0x00000000UL            /*!<DMA DAR increment  \hideinitializer */
#define LPPDMA_DAR_FIX        0x00000C00UL            /*!<DMA DAR fix address  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Burst Mode Constant Definitions                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define LPPDMA_REQ_SINGLE     0x00000004UL            /*!<DMA Single Request  \hideinitializer */
#define LPPDMA_REQ_BURST      0x00000000UL            /*!<DMA Burst Request  \hideinitializer */

#define LPPDMA_BURST_128      0x00000000UL            /*!<DMA Burst 128 Transfers  \hideinitializer */
#define LPPDMA_BURST_64       0x00000010UL            /*!<DMA Burst 64 Transfers  \hideinitializer */
#define LPPDMA_BURST_32       0x00000020UL            /*!<DMA Burst 32 Transfers  \hideinitializer */
#define LPPDMA_BURST_16       0x00000030UL            /*!<DMA Burst 16 Transfers  \hideinitializer */
#define LPPDMA_BURST_8        0x00000040UL            /*!<DMA Burst 8 Transfers  \hideinitializer */
#define LPPDMA_BURST_4        0x00000050UL            /*!<DMA Burst 4 Transfers  \hideinitializer */
#define LPPDMA_BURST_2        0x00000060UL            /*!<DMA Burst 2 Transfers  \hideinitializer */
#define LPPDMA_BURST_1        0x00000070UL            /*!<DMA Burst 1 Transfers  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Table Interrupt Disable Constant Definitions                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define LPPDMA_TBINTDIS_ENABLE  (0x0UL<<LPPDMA_DSCT_CTL_TBINTDIS_Pos)  /*!<DMA Table Interrupt Enabled   \hideinitializer */
#define LPPDMA_TBINTDIS_DISABLE (0x1UL<<LPPDMA_DSCT_CTL_TBINTDIS_Pos)  /*!<DMA Table Interrupt Disabled  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Peripheral Transfer Mode Constant Definitions                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define LPPDMA_MEM            0UL   /*!<DMA Connect to Memory \hideinitializer */
#define LPPDMA_LPUART0_TX     1UL   /*!<DMA Connect to LPUART0_TX \hideinitializer */
#define LPPDMA_LPUART0_RX     2UL   /*!<DMA Connect to LPUART0_RX \hideinitializer */
#define LPPDMA_LPSPI0_TX      3UL   /*!<DMA Connect to LPSPI0_TX \hideinitializer */
#define LPPDMA_LPSPI0_RX      4UL   /*!<DMA Connect to LPSPI0_RX \hideinitializer */
#define LPPDMA_LPI2C0_TX      5UL   /*!<DMA Connect to LPI2C0_TX \hideinitializer */
#define LPPDMA_LPI2C0_RX      6UL   /*!<DMA Connect to LPI2C0_RX \hideinitializer */
#define LPPDMA_LPTMR0         7UL   /*!<DMA Connect to LPTMR0 \hideinitializer */
#define LPPDMA_LPTMR1         8UL   /*!<DMA Connect to LPTMR1 \hideinitializer */
#define LPPDMA_TTMR0          9UL   /*!<DMA Connect to TTMR0 \hideinitializer */
#define LPPDMA_TTMR1          10UL  /*!<DMA Connect to TTMR1 \hideinitializer */
#define LPPDMA_LPADC0_RX      11UL  /*!<DMA Connect to LPADC0_RX \hideinitializer */
#define LPPDMA_DMIC0_RX       13UL  /*!<DMA Connect to DMIC0_RX \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Interrupt Type Constant Definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define LPPDMA_INT_TRANS_DONE 0x00000000UL            /*!<Transfer Done Interrupt  \hideinitializer */
#define LPPDMA_INT_TEMPTY     0x00000001UL            /*!<Table Empty Interrupt  \hideinitializer */


/** @} end of group LPPDMA_EXPORTED_CONSTANTS */

/** @addtogroup LPPDMA_EXPORTED_FUNCTIONS LPPDMA Exported Functions
  @{
*/

/**
 * @brief       Get LPPDMA Interrupt Status
 *
 * @param[in]   lppdma      The pointer of the specified LPPDMA module
 *
 * @details     This macro gets the interrupt status.
 * \hideinitializer
 */
#define LPPDMA_GET_INT_STATUS(lppdma) ((uint32_t)(lppdma->INTSTS))

/**
 * @brief       Get Transfer Done Interrupt Status
 *
 * @param[in]   lppdma      The pointer of the specified LPPDMA module
 *
 * @details     Get the transfer done Interrupt status.
 * \hideinitializer
 */
#define LPPDMA_GET_TD_STS(lppdma) ((uint32_t)(lppdma->TDSTS))

/**
 * @brief       Clear Transfer Done Interrupt Status
 *
 * @param[in]   lppdma      The pointer of the specified LPPDMA module
 * @param[in]   u32Mask     The channel mask
 *
 * @details     Clear the transfer done Interrupt status.
 * \hideinitializer
 */
#define LPPDMA_CLR_TD_FLAG(lppdma,u32Mask) ((uint32_t)(lppdma->TDSTS = (u32Mask)))

/**
 * @brief       Get Target Abort Interrupt Status
 *
 * @param[in]   lppdma      The pointer of the specified LPPDMA module
 *
 * @details     Get the target abort Interrupt status.
 * \hideinitializer
 */
#define LPPDMA_GET_ABORT_STS(lppdma) ((uint32_t)(lppdma->ABTSTS))

/**
 * @brief       Clear Target Abort Interrupt Status
 *
 * @param[in]   lppdma      The pointer of the specified LPPDMA module
 * @param[in]   u32Mask     The channel mask
 *
 * @details     Clear the target abort Interrupt status.
 * \hideinitializer
 */
#define LPPDMA_CLR_ABORT_FLAG(lppdma,u32Mask) ((uint32_t)(lppdma->ABTSTS = (u32Mask)))

/**
 * @brief       Get Alignment Interrupt Status
 *
 * @param[in]   lppdma      The pointer of the specified LPPDMA module
 *
 * @details     Get Alignment Interrupt status.
 * \hideinitializer
 */
#define LPPDMA_GET_ALIGN_STS(lppdma) ((uint32_t)(lppdma->ALIGN))

/**
 * @brief       Clear Alignment Interrupt Status
  *
 * @param[in]   lppdma      The pointer of the specified LPPDMA module
 * @param[in]   u32Mask     The channel mask
 *
 * @details     Clear the Alignment Interrupt status.
 * \hideinitializer
 */
#define LPPDMA_CLR_ALIGN_FLAG(lppdma,u32Mask) ((uint32_t)(lppdma->ALIGN = (u32Mask)))

/**
 * @brief       Check Channel Status
  *
 * @param[in]   lppdma    The pointer of the specified LPPDMA module
 * @param[in]   u32Ch     The selected channel
 *
 * @retval      0 Idle state
 * @retval      1 Busy state
 *
 * @details     Check the selected channel is busy or not.
 * \hideinitializer
 */
#define LPPDMA_IS_CH_BUSY(lppdma,u32Ch) ((uint32_t)(lppdma->TRGSTS & (1 << (u32Ch)))? 1 : 0)

/**
 * @brief       Set Source Address
  *
 * @param[in]   lppdma    The pointer of the specified LPPDMA module
 * @param[in]   u32Ch     The selected channel
 * @param[in]   u32Addr   The selected address
 *
 * @details     This macro set the selected channel source address.
 * \hideinitializer
 */
#define LPPDMA_SET_SRC_ADDR(lppdma,u32Ch, u32Addr) ((uint32_t)(lppdma->LPDSCT[(u32Ch)].SA = (u32Addr)))

/**
 * @brief       Set Destination Address
 *
 * @param[in]   lppdma      The pointer of the specified LPPDMA module
 * @param[in]   u32Ch     The selected channel
 * @param[in]   u32Addr   The selected address
 *
 * @details     This macro set the selected channel destination address.
 * \hideinitializer
 */
#define LPPDMA_SET_DST_ADDR(lppdma,u32Ch, u32Addr) ((uint32_t)(lppdma->LPDSCT[(u32Ch)].DA = (u32Addr)))

/**
 * @brief       Set Transfer Count
 *
 * @param[in]   lppdma         The pointer of the specified LPPDMA module
 * @param[in]   u32Ch          The selected channel
 * @param[in]   u32TransCount  Transfer Count
 *
 * @details     This macro set the selected channel transfer count.
 * \hideinitializer
 */
#define LPPDMA_SET_TRANS_CNT(lppdma,u32Ch, u32TransCount) ((uint32_t)(lppdma->LPDSCT[(u32Ch)].CTL=(lppdma->LPDSCT[(u32Ch)].CTL&~LPPDMA_DSCT_CTL_TXCNT_Msk)|(((u32TransCount)-1) << LPPDMA_DSCT_CTL_TXCNT_Pos)))

/**
 * @brief       Set Scatter-gather descriptor Address
 *
 * @param[in]   lppdma    The pointer of the specified LPPDMA module
 * @param[in]   u32Ch     The selected channel
 * @param[in]   u32Addr   The descriptor address
 *
 * @details     This macro set the selected channel scatter-gather descriptor address.
 * \hideinitializer
 */
#define LPPDMA_SET_SCATTER_DESC(lppdma,u32Ch, u32Addr) ((uint32_t)(lppdma->LPDSCT[(u32Ch)].NEXT = u32Addr))

/**
 * @brief       Stop the channel
 *
 * @param[in]   lppdma    The pointer of the specified LPPDMA module
 * @param[in]   u32Ch     The selected channel
 *
 * @details     This macro stop the selected channel.
 * \hideinitializer
 */
#define LPPDMA_STOP(lppdma,u32Ch) ((uint32_t)(lppdma->PAUSE = (1 << (u32Ch))))

/**
 * @brief       Pause the channel
 *
 * @param[in]   lppdma    The pointer of the specified LPPDMA module
 * @param[in]   u32Ch     The selected channel
 *
 * @details     This macro pause the selected channel.
 * \hideinitializer
 */
#define LPPDMA_PAUSE(lppdma,u32Ch) ((uint32_t)(lppdma->PAUSE = (1 << (u32Ch))))

/**
 * @brief       Reset the channel
 *
 * @param[in]   lppdma    The pointer of the specified LPPDMA module
 * @param[in]   u32Ch     The selected channel
 *
 * @details     This macro reset the selected channel.
 * \hideinitializer
 */
#define LPPDMA_RESET(lppdma,u32Ch) ((uint32_t)(lppdma->CHRST = (1 << (u32Ch))))

/*---------------------------------------------------------------------------------------------------------*/
/* Define PDMA functions prototype                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void LPPDMA_Open(LPPDMA_T *lppdma, uint32_t u32Mask);
void LPPDMA_Close(LPPDMA_T *lppdma);
void LPPDMA_SetTransferCnt(LPPDMA_T *lppdma, uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount);
void LPPDMA_SetTransferAddr(LPPDMA_T *lppdma, uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl);
void LPPDMA_SetTransferMode(LPPDMA_T *lppdma, uint32_t u32Ch, uint32_t u32Peripheral, uint32_t u32ScatterEn, uint32_t u32DescAddr);
void LPPDMA_SetBurstType(LPPDMA_T *lppdma, uint32_t u32Ch, uint32_t u32BurstType, uint32_t u32BurstSize);
void LPPDMA_Trigger(LPPDMA_T *lppdma, uint32_t u32Ch);
void LPPDMA_EnableInt(LPPDMA_T *lppdma, uint32_t u32Ch, uint32_t u32Mask);
void LPPDMA_DisableInt(LPPDMA_T *lppdma, uint32_t u32Ch, uint32_t u32Mask);

/** @} end of group LPPDMA_EXPORTED_FUNCTIONS */
/** @} end of group LPPDMA_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __LPPDMA_H__ */
