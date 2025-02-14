/**************************************************************************//**
 * @file     lppdma.c
 * @version  V1.00
 * @brief    LPPDMA driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"

static uint8_t u32ChSelect[LPPDMA_CH_MAX];

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LPPDMA_Driver LPPDMA Driver
  @{
*/

/** @addtogroup LPPDMA_EXPORTED_FUNCTIONS LPPDMA Exported Functions
  @{
*/

/**
 * @brief       LPPDMA Open
 *
 * @param[in]   lppdma            The pointer of the specified LPPDMA module
 * @param[in]   u32Mask     Channel enable bits.
 *
 * @details     This function enable the LPPDMA channels.
 */
void LPPDMA_Open(LPPDMA_T *lppdma, uint32_t u32Mask)
{
    uint32_t i;

    for (i = 0UL; i < LPPDMA_CH_MAX; i++)
    {
        if ((1 << i) & u32Mask)
        {
            lppdma->LPDSCT[i].CTL = 0UL;
            u32ChSelect[i] = LPPDMA_MEM;
        }
    }

    lppdma->CHCTL |= u32Mask;
}

/**
 * @brief       LPPDMA Close
 *
 * @param[in]   lppdma            The pointer of the specified LPPDMA module
 *
 * @details     This function disable all LPPDMA channels.
 */
void LPPDMA_Close(LPPDMA_T *lppdma)
{
    lppdma->CHCTL = 0UL;
}

/**
 * @brief       Set LPPDMA Transfer Count
 *
 * @param[in]   lppdma            The pointer of the specified LPPDMA module
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Width        Data width. Valid values are
 *                - \ref LPPDMA_WIDTH_8
 *                - \ref LPPDMA_WIDTH_16
 *                - \ref LPPDMA_WIDTH_32
 * @param[in]   u32TransCount   Transfer count
 *
 * @details     This function set the selected channel data width and transfer count.
 */
void LPPDMA_SetTransferCnt(LPPDMA_T *lppdma, uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount)
{
    lppdma->LPDSCT[u32Ch].CTL &= ~(LPPDMA_DSCT_CTL_TXCNT_Msk | LPPDMA_DSCT_CTL_TXWIDTH_Msk);
    lppdma->LPDSCT[u32Ch].CTL |= (u32Width | ((u32TransCount - 1UL) << LPPDMA_DSCT_CTL_TXCNT_Pos));
}

/**
 * @brief       Set LPPDMA Transfer Address
  *
 * @param[in]   lppdma            The pointer of the specified LPPDMA module
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32SrcAddr      Source address
 * @param[in]   u32SrcCtrl      Source control attribute. Valid values are
 *                - \ref LPPDMA_SAR_INC
 *                - \ref LPPDMA_SAR_FIX
 * @param[in]   u32DstAddr      destination address
 * @param[in]   u32DstCtrl      destination control attribute. Valid values are
 *                - \ref LPPDMA_DAR_INC
 *                - \ref LPPDMA_DAR_FIX
 *
 * @details     This function set the selected channel source/destination address and attribute.
 */
void LPPDMA_SetTransferAddr(LPPDMA_T *lppdma, uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl)
{
    lppdma->LPDSCT[u32Ch].SA = u32SrcAddr;
    lppdma->LPDSCT[u32Ch].DA = u32DstAddr;
    lppdma->LPDSCT[u32Ch].CTL &= ~(LPPDMA_DSCT_CTL_SAINC_Msk | LPPDMA_DSCT_CTL_DAINC_Msk);
    lppdma->LPDSCT[u32Ch].CTL |= (u32SrcCtrl | u32DstCtrl);
}

/**
 * @brief       Set LPPDMA Transfer Mode
  *
 * @param[in]   lppdma            The pointer of the specified LPPDMA module
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Peripheral   The selected peripheral. Valid values are
 *                - \ref LPPDMA_MEM
 *                - \ref LPPDMA_LPUART0_TX
 *                - \ref LPPDMA_LPUART0_RX
 *                - \ref LPPDMA_LPSPI0_TX
 *                - \ref LPPDMA_LPSPI0_RX
 *                - \ref LPPDMA_LPI2C0_TX
 *                - \ref LPPDMA_LPI2C0_RX
 *                - \ref LPPDMA_LPTMR0
 *                - \ref LPPDMA_LPTMR1
 *                - \ref LPPDMA_TTMR0
 *                - \ref LPPDMA_TTMR1
 *                - \ref LPPDMA_LPADC0_RX
 *                - \ref LPPDMA_DMIC0_RX
 * @param[in]   u32ScatterEn    Scatter-gather mode enable
 * @param[in]   u32DescAddr     Scatter-gather descriptor address
 *
 * @details     This function set the selected channel transfer mode. Include peripheral setting.
 */
void LPPDMA_SetTransferMode(LPPDMA_T *lppdma, uint32_t u32Ch, uint32_t u32Peripheral, uint32_t u32ScatterEn, uint32_t u32DescAddr)
{
    u32ChSelect[u32Ch] = u32Peripheral;

    switch (u32Ch)
    {
        case 0ul:
            lppdma->REQSEL0_3 = (lppdma->REQSEL0_3 & ~LPPDMA_REQSEL0_3_REQSRC0_Msk) | u32Peripheral;
            break;

        case 1ul:
            lppdma->REQSEL0_3 = (lppdma->REQSEL0_3 & ~LPPDMA_REQSEL0_3_REQSRC1_Msk) | (u32Peripheral << LPPDMA_REQSEL0_3_REQSRC1_Pos);
            break;

        case 2ul:
            lppdma->REQSEL0_3 = (lppdma->REQSEL0_3 & ~LPPDMA_REQSEL0_3_REQSRC2_Msk) | (u32Peripheral << LPPDMA_REQSEL0_3_REQSRC2_Pos);
            break;

        case 3ul:
            lppdma->REQSEL0_3 = (lppdma->REQSEL0_3 & ~LPPDMA_REQSEL0_3_REQSRC3_Msk) | (u32Peripheral << LPPDMA_REQSEL0_3_REQSRC3_Pos);
            break;

        default:
            break;
    }

    if (u32ScatterEn)
    {
        lppdma->LPDSCT[u32Ch].NEXT = u32DescAddr;
        lppdma->LPDSCT[u32Ch].CTL = (lppdma->LPDSCT[u32Ch].CTL & ~LPPDMA_DSCT_CTL_OPMODE_Msk) | LPPDMA_OP_SCATTER;
    }
    else
    {
        lppdma->LPDSCT[u32Ch].CTL = (lppdma->LPDSCT[u32Ch].CTL & ~LPPDMA_DSCT_CTL_OPMODE_Msk) | LPPDMA_OP_BASIC;
    }
}

/**
 * @brief       Set LPPDMA Burst Type and Size
 *
 * @param[in]   lppdma            The pointer of the specified LPPDMA module
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32BurstType    Burst mode or single mode. Valid values are
 *                - \ref LPPDMA_REQ_SINGLE
 *                - \ref LPPDMA_REQ_BURST
 * @param[in]   u32BurstSize    Set the size of burst mode. Valid values are
 *                - \ref LPPDMA_BURST_128
 *                - \ref LPPDMA_BURST_64
 *                - \ref LPPDMA_BURST_32
 *                - \ref LPPDMA_BURST_16
 *                - \ref LPPDMA_BURST_8
 *                - \ref LPPDMA_BURST_4
 *                - \ref LPPDMA_BURST_2
 *                - \ref LPPDMA_BURST_1
 *
 * @details     This function set the selected channel burst type and size.
 */
void LPPDMA_SetBurstType(LPPDMA_T *lppdma, uint32_t u32Ch, uint32_t u32BurstType, uint32_t u32BurstSize)
{
    lppdma->LPDSCT[u32Ch].CTL &= ~(LPPDMA_DSCT_CTL_TXTYPE_Msk | LPPDMA_DSCT_CTL_BURSIZE_Msk);
    lppdma->LPDSCT[u32Ch].CTL |= (u32BurstType | u32BurstSize);
}

/**
 * @brief       Trigger LPPDMA
 *
 * @param[in]   lppdma            The pointer of the specified LPPDMA module
 * @param[in]   u32Ch           The selected channel
 *
 * @details     This function trigger the selected channel.
 */
void LPPDMA_Trigger(LPPDMA_T *lppdma, uint32_t u32Ch)
{
    if (u32ChSelect[u32Ch] == LPPDMA_MEM)
    {
        /* Ensure completion of memory access */
        __DSB();
        lppdma->SWREQ = (1ul << u32Ch);
    }
    else {}
}

/**
 * @brief       Enable Interrupt
 *
 * @param[in]   lppdma            The pointer of the specified LPPDMA module
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Mask         The Interrupt Type. Valid values are
 *                - \ref LPPDMA_INT_TRANS_DONE
 *                - \ref LPPDMA_INT_TEMPTY
 *
 * @details     This function enable the selected channel interrupt.
 */
void LPPDMA_EnableInt(LPPDMA_T *lppdma, uint32_t u32Ch, uint32_t u32Mask)
{
    switch (u32Mask)
    {
        case LPPDMA_INT_TRANS_DONE:
            lppdma->INTEN |= (1ul << u32Ch);
            break;

        case LPPDMA_INT_TEMPTY:
            lppdma->LPDSCT[u32Ch].CTL &= ~LPPDMA_DSCT_CTL_TBINTDIS_Msk;
            break;

        default:
            break;
    }
}

/**
 * @brief       Disable Interrupt
 *
 * @param[in]   lppdma            The pointer of the specified LPPDMA module
 * @param[in]   u32Ch           The selected channel
 * @param[in]   u32Mask         The Interrupt Type. Valid values are
 *                - \ref LPPDMA_INT_TRANS_DONE
 *                - \ref LPPDMA_INT_TEMPTY
 *
 * @details     This function disable the selected channel interrupt.
 */
void LPPDMA_DisableInt(LPPDMA_T *lppdma, uint32_t u32Ch, uint32_t u32Mask)
{
    switch (u32Mask)
    {
        case LPPDMA_INT_TRANS_DONE:
            lppdma->INTEN &= ~(1ul << u32Ch);
            break;

        case LPPDMA_INT_TEMPTY:
            lppdma->LPDSCT[u32Ch].CTL |= LPPDMA_DSCT_CTL_TBINTDIS_Msk;
            break;

        default:
            break;
    }
}

/** @} end of group LPPDMA_EXPORTED_FUNCTIONS */
/** @} end of group LPPDMA_Driver */
/** @} end of group Standard_Driver */
