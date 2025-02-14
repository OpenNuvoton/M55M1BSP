/**************************************************************************//**
 * @file     lpspi.h
 * @version  V1.00
 * @brief    M55M1 series LPSPI driver header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __LPSPI_H__
#define __LPSPI_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LPSPI_Driver LPSPI Driver
  @{
*/

/** @addtogroup LPSPI_EXPORTED_CONSTANTS LPSPI Exported Constants
  @{
*/

#define LPSPI_MODE_0                        (LPSPI_CTL_TXNEG_Msk)                             /*!< CLKPOL=0; RXNEG=0; TXNEG=1 */
#define LPSPI_MODE_1                        (LPSPI_CTL_RXNEG_Msk)                             /*!< CLKPOL=0; RXNEG=1; TXNEG=0 */
#define LPSPI_MODE_2                        (LPSPI_CTL_CLKPOL_Msk | LPSPI_CTL_RXNEG_Msk)      /*!< CLKPOL=1; RXNEG=1; TXNEG=0 */
#define LPSPI_MODE_3                        (LPSPI_CTL_CLKPOL_Msk | LPSPI_CTL_TXNEG_Msk)      /*!< CLKPOL=1; RXNEG=0; TXNEG=1 */

#define LPSPI_SLAVE                         (LPSPI_CTL_SLAVE_Msk)             /*!< Set as slave */
#define LPSPI_MASTER                        (0x0U)                            /*!< Set as master */

#define LPSPI_SS                            (LPSPI_SSCTL_SS_Msk)              /*!< Set SS */
#define LPSPI_SS_ACTIVE_HIGH                (LPSPI_SSCTL_SSACTPOL_Msk)        /*!< SS active high */
#define LPSPI_SS_ACTIVE_LOW                 (0x0U)                            /*!< SS active low */

/* LPSPI Interrupt Mask */
#define LPSPI_UNIT_INT_MASK                 (0x001U)                          /*!< Unit transfer interrupt mask */
#define LPSPI_SSACT_INT_MASK                (0x002U)                          /*!< Slave selection signal active interrupt mask */
#define LPSPI_SSINACT_INT_MASK              (0x004U)                          /*!< Slave selection signal inactive interrupt mask */
#define LPSPI_SLVUR_INT_MASK                (0x008U)                          /*!< Slave under run interrupt mask */
#define LPSPI_SLVBE_INT_MASK                (0x010U)                          /*!< Slave bit count error interrupt mask */
#define LPSPI_TXUF_INT_MASK                 (0x040U)                          /*!< Slave TX underflow interrupt mask */
#define LPSPI_FIFO_TXTH_INT_MASK            (0x080U)                          /*!< FIFO TX threshold interrupt mask */
#define LPSPI_FIFO_RXTH_INT_MASK            (0x100U)                          /*!< FIFO RX threshold interrupt mask */
#define LPSPI_FIFO_RXOV_INT_MASK            (0x200U)                          /*!< FIFO RX overrun interrupt mask */
#define LPSPI_FIFO_RXTO_INT_MASK            (0x400U)                          /*!< FIFO RX time-out interrupt mask */

/* LPSPI Status Mask */
#define LPSPI_BUSY_MASK                     (0x01U)                           /*!< Busy status mask */
#define LPSPI_RX_EMPTY_MASK                 (0x02U)                           /*!< RX empty status mask */
#define LPSPI_RX_FULL_MASK                  (0x04U)                           /*!< RX full status mask */
#define LPSPI_TX_EMPTY_MASK                 (0x08U)                           /*!< TX empty status mask */
#define LPSPI_TX_FULL_MASK                  (0x10U)                           /*!< TX full status mask */
#define LPSPI_TXRX_RESET_MASK               (0x20U)                           /*!< TX or RX reset status mask */
#define LPSPI_SPIEN_STS_MASK                (0x40U)                           /*!< LPSPIEN status mask */
#define LPSPI_SSLINE_STS_MASK               (0x80U)                           /*!< LPSPIx_SS line status mask */

/* LPSPI Auto Trigger Source */
#define LPSPI_AUTOCTL_TRIGSEL_LPTMR0        (0x00UL)
#define LPSPI_AUTOCTL_TRIGSEL_LPTMR1        (0x01UL)
#define LPSPI_AUTOCTL_TRIGSEL_TTMR0         (0x02UL)
#define LPSPI_AUTOCTL_TRIGSEL_TTMR1         (0x03UL)
#define LPSPI_AUTOCTL_TRIGSEL_WKIOA         (0x04UL)
#define LPSPI_AUTOCTL_TRIGSEL_WKIOB         (0x05UL)
#define LPSPI_AUTOCTL_TRIGSEL_WKIOC         (0x06UL)
#define LPSPI_AUTOCTL_TRIGSEL_WKIOD         (0x07UL)

/* LPSPI Clock Source */
#define LPSPI_CLKSEL_PCLK                   (0x0UL)
#define LPSPI_CLKSEL_MIRC                   (0x1UL)
#define LPSPI_CLKSEL_HIRC                   (0x2UL)

/** @} end of group LPSPI_EXPORTED_CONSTANTS */


/** @addtogroup LPSPI_EXPORTED_FUNCTIONS LPSPI Exported Functions
  @{
*/

/**
  * @brief      Clear the unit transfer interrupt flag.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Write 1 to UNITIF bit of LPSPI_STATUS register to clear the unit transfer interrupt flag.
  * \hideinitializer
  */
#define LPSPI_CLR_UNIT_TRANS_INT_FLAG(lpspi)    (lpspi->STATUS |= LPSPI_STATUS_UNITIF_Msk)

/**
  * @brief      Trigger RX PDMA function.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Set RXPDMAEN bit of LPSPI_PDMACTL register to enable RX PDMA transfer function.
  * \hideinitializer
  */
#define LPSPI_TRIGGER_RX_PDMA(lpspi)    (lpspi->PDMACTL |= LPSPI_PDMACTL_RXPDMAEN_Msk)

/**
  * @brief      Trigger TX PDMA function.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Set TXPDMAEN bit of LPSPI_PDMACTL register to enable TX PDMA transfer function.
  * \hideinitializer
  */
#define LPSPI_TRIGGER_TX_PDMA(lpspi)    (lpspi->PDMACTL |= LPSPI_PDMACTL_TXPDMAEN_Msk)

/**
  * @brief      Trigger TX and RX PDMA function.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Set TXPDMAEN bit and RXPDMAEN bit of LPSPI_PDMACTL register to enable TX and RX PDMA transfer function.
  * \hideinitializer
  */
#define LPSPI_TRIGGER_TX_RX_PDMA(lpspi) \
    (lpspi->PDMACTL = (LPSPI_PDMACTL_TXPDMAEN_Msk | LPSPI_PDMACTL_RXPDMAEN_Msk))

/**
  * @brief      Disable RX PDMA transfer.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Clear RXPDMAEN bit of LPSPI_PDMACTL register to disable RX PDMA transfer function.
  * \hideinitializer
  */
#define LPSPI_DISABLE_RX_PDMA(lpspi)    (lpspi->PDMACTL &= ~(LPSPI_PDMACTL_RXPDMAEN_Msk))

/**
  * @brief      Disable TX PDMA transfer.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Clear TXPDMAEN bit of LPSPI_PDMACTL register to disable TX PDMA transfer function.
  * \hideinitializer
  */
#define LPSPI_DISABLE_TX_PDMA(lpspi)    (lpspi->PDMACTL &= ~(LPSPI_PDMACTL_TXPDMAEN_Msk))

/**
  * @brief      Disable TX and RX PDMA transfer.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Clear TXPDMAEN bit and RXPDMAEN bit of LPSPI_PDMACTL register to disable TX and RX PDMA transfer function.
  * \hideinitializer
  */
#define LPSPI_DISABLE_TX_RX_PDMA(lpspi) \
    (lpspi->PDMACTL &= ~(LPSPI_PDMACTL_TXPDMAEN_Msk | LPSPI_PDMACTL_RXPDMAEN_Msk))

/**
  * @brief      Get the count of available data in RX FIFO.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     The count of available data in RX FIFO.
  * @details    Read RXCNT (LPSPI_STATUS[27:24]) to get the count of available data in RX FIFO.
  * \hideinitializer
  */
#define LPSPI_GET_RX_FIFO_COUNT(lpspi)  \
    ((lpspi->STATUS & LPSPI_STATUS_RXCNT_Msk) >> LPSPI_STATUS_RXCNT_Pos)

/**
  * @brief      Get the RX FIFO empty flag.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @retval     0 RX FIFO is not empty.
  * @retval     1 RX FIFO is empty.
  * @details    Read RXEMPTY bit of LPSPI_STATUS register to get the RX FIFO empty flag.
  * \hideinitializer
  */
#define LPSPI_GET_RX_FIFO_EMPTY_FLAG(lpspi) \
    ((lpspi->STATUS & LPSPI_STATUS_RXEMPTY_Msk) >> LPSPI_STATUS_RXEMPTY_Pos)

/**
  * @brief      Get the TX FIFO empty flag.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @retval     0 TX FIFO is not empty.
  * @retval     1 TX FIFO is empty.
  * @details    Read TXEMPTY bit of LPSPI_STATUS register to get the TX FIFO empty flag.
  * \hideinitializer
  */
#define LPSPI_GET_TX_FIFO_EMPTY_FLAG(lpspi) \
    ((lpspi->STATUS & LPSPI_STATUS_TXEMPTY_Msk) >> LPSPI_STATUS_TXEMPTY_Pos)

/**
  * @brief      Get the TX FIFO full flag.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @retval     0 TX FIFO is not full.
  * @retval     1 TX FIFO is full.
  * @details    Read TXFULL bit of LPSPI_STATUS register to get the TX FIFO full flag.
  * \hideinitializer
  */
#define LPSPI_GET_TX_FIFO_FULL_FLAG(lpspi)  \
    ((lpspi->STATUS & LPSPI_STATUS_TXFULL_Msk) >> LPSPI_STATUS_TXFULL_Pos)

/**
  * @brief      Get the datum read from RX register.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     Data in RX register.
  * @details    Read LPSPI_RX register to get the received datum.
  * \hideinitializer
  */
#define LPSPI_READ_RX(lpspi)  (lpspi->RX)

/**
  * @brief      Write datum to TX register.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32TxData The datum which user attempt to transfer through LPSPI bus.
  * @return     None.
  * @details    Write u32TxData to LPSPI_TX register.
  * \hideinitializer
  */
#define LPSPI_WRITE_TX(lpspi, u32TxData)    (lpspi->TX = (u32TxData))

/**
  * @brief      Set SPIx_SS pin to high state.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Disable automatic slave selection function and set SPIx_SS pin to high state.
  * \hideinitializer
  */
#define LPSPI_SET_SS_HIGH(lpspi)    \
    (lpspi->SSCTL = (lpspi->SSCTL & ~(LPSPI_SSCTL_AUTOSS_Msk)) | \
                    (LPSPI_SSCTL_SSACTPOL_Msk | LPSPI_SSCTL_SS_Msk))

/**
  * @brief      Set SPIx_SS pin to low state.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Disable automatic slave selection function and set SPIx_SS pin to low state.
  * \hideinitializer
  */
#define LPSPI_SET_SS_LOW(lpspi) \
    (lpspi->SSCTL = (lpspi->SSCTL & ~(LPSPI_SSCTL_AUTOSS_Msk | LPSPI_SSCTL_SSACTPOL_Msk)) | \
                    LPSPI_SSCTL_SS_Msk)

/**
  * @brief      Enable Byte Reorder function.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Enable Byte Reorder function. The suspend interval depends on the setting of SUSPITV (LPSPI_CTL[7:4]).
  * \hideinitializer
  */
#define LPSPI_ENABLE_BYTE_REORDER(lpspi)    (lpspi->CTL |= LPSPI_CTL_REORDER_Msk)

/**
  * @brief      Disable Byte Reorder function.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Clear REORDER bit field of LPSPI_CTL register to disable Byte Reorder function.
  * \hideinitializer
  */
#define LPSPI_DISABLE_BYTE_REORDER(lpspi)   (lpspi->CTL &= ~(LPSPI_CTL_REORDER_Msk))

/**
  * @brief      Set the length of suspend interval.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32SuspCycle Decides the length of suspend interval. It could be 0 ~ 15.
  * @return     None.
  * @details    Set the length of suspend interval according to u32SuspCycle.
  *             The length of suspend interval is ((u32SuspCycle + 0.5) * the length of one LPSPI bus clock cycle).
  * \hideinitializer
  */
#define LPSPI_SET_SUSPEND_CYCLE(lpspi, u32SuspCycle)        \
    (lpspi->CTL = (lpspi->CTL & ~(LPSPI_CTL_SUSPITV_Msk)) | \
                  ((u32SuspCycle) << LPSPI_CTL_SUSPITV_Pos))

/**
  * @brief      Set the LPSPI transfer sequence with LSB first.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Set LSB bit of LPSPI_CTL register to set the LPSPI transfer sequence with LSB first.
  * \hideinitializer
  */
#define LPSPI_SET_LSB_FIRST(lpspi)  (lpspi->CTL |= LPSPI_CTL_LSB_Msk)

/**
  * @brief      Set the LPSPI transfer sequence with MSB first.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Clear LSB bit of LPSPI_CTL register to set the LPSPI transfer sequence with MSB first.
  * \hideinitializer
  */
#define LPSPI_SET_MSB_FIRST(lpspi)  (lpspi->CTL &= ~(LPSPI_CTL_LSB_Msk))

/**
  * @brief      Set the data width of a LPSPI transaction.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32Width The bit width of one transaction.
  * @return     None.
  * @details    The data width can be 8 ~ 32 bits.
  * \hideinitializer
  */
#define LPSPI_SET_DATA_WIDTH(lpspi, u32Width)               \
    (lpspi->CTL = (lpspi->CTL & ~(LPSPI_CTL_DWIDTH_Msk)) |  \
                  (((u32Width) & 0x1F) << LPSPI_CTL_DWIDTH_Pos))

/**
  * @brief      Get the LPSPI busy state.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @retval     0 LPSPI controller is not busy.
  * @retval     1 LPSPI controller is busy.
  * @details    This macro will return the busy state of LPSPI controller.
  * \hideinitializer
  */
#define LPSPI_IS_BUSY(lpspi)  \
    ((lpspi->STATUS & LPSPI_STATUS_BUSY_Msk) >> LPSPI_STATUS_BUSY_Pos)

/**
  * @brief      Enable LPSPI controller.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Set SPIEN (LPSPI_CTL[0]) to enable LPSPI controller.
  * \hideinitializer
  */
#define LPSPI_ENABLE(lpspi) (lpspi->CTL |= LPSPI_CTL_SPIEN_Msk)

/**
  * @brief      Disable LPSPI controller.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Clear SPIEN (LPSPI_CTL[0]) to disable LPSPI controller.
  * \hideinitializer
  */
#define LPSPI_DISABLE(lpspi)    (lpspi->CTL &= ~(LPSPI_CTL_SPIEN_Msk))

/**
  * @brief      Set Auto Operation RX transfer count.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32TCNT The transfer count specified in RX phase.
  * @return     None.
  * @details    Set RX transfer count (LPSPI_AUTOCTL[23:16]).
  * \hideinitializer
  */
#define LPSPI_SET_AUTO_RX_TCNT(lpspi, u32TCNT)  \
    ((lpspi)->AUTOCTL = ((lpspi)->AUTOCTL & ~LPSPI_AUTOCTL_TCNT_Msk) | \
                        ((u32TCNT & 0xFF) << LPSPI_AUTOCTL_TCNT_Pos))

/**
  * @brief      Enable RX TCNT count match wake up.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Set CNTWKEN (LPSPI_AUTOCTL[10]) to enable RX TCNT count
  *             match wake up function.
  * \hideinitializer
  */
#define LPSPI_ENABLE_AUTO_CNT_WAKEUP(lpspi) ((lpspi)->AUTOCTL |= LPSPI_AUTOCTL_CNTWKEN_Msk)

/**
  * @brief      Disable RX TCNT count match wake up.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Clear CNTWKEN (LPSPI_AUTOCTL[10]) to disable RX TCNT count
  *             match wake up function.
  * \hideinitializer
  */
#define LPSPI_DISABLE_AUTO_CNT_WAKEUP(lpspi)    ((lpspi)->AUTOCTL &= ~LPSPI_AUTOCTL_CNTWKEN_Msk)

/**
  * @brief      Enable Software Trigger for Auto Operation.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Set SWTRIG (LPSPI_AUTOCTL[9], Write Only bit) to
  *             enable Software Trigger in Auto Operation Mode.
  * \hideinitializer
  */
#define LPSPI_ENABLE_AUTO_SW_TRIG(lpspi)    ((lpspi)->AUTOCTL |= LPSPI_AUTOCTL_SWTRG_Msk)

/**
  * @brief      Disable Software Trigger for Auto Operation.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Clear SWTRIG (LPSPI_AUTOCTL[9], Write Only bit) to
  *             disable Software Trigger in Auto Operation Mode.
  * \hideinitializer
  */
#define LPSPI_DISABLE_AUTO_SW_TRIG(lpspi)   ((lpspi)->AUTOCTL &= ~LPSPI_AUTOCTL_SWTRG_Msk)

/**
  * @brief      Enable LPSPI Auto Operation Mode.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Set AUTOEN (LPSPI_AUTOCTL[8]) to enable Auto Operation Mode.
  * \hideinitializer
  */
#define LPSPI_ENABLE_AUTO(lpspi)    ((lpspi)->AUTOCTL |= LPSPI_AUTOCTL_AUTOEN_Msk)

/**
  * @brief      Disable LPSPI Auto Operation Mode.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Clear AUTOEN (LPSPI_AUTOCTL[8]) to disable Auto Operation Mode.
  * \hideinitializer
  */
#define LPSPI_DISABLE_AUTO(lpspi)   ((lpspi)->AUTOCTL &= ~LPSPI_AUTOCTL_AUTOEN_Msk)

/**
  * @brief      Enable Slave Selection wake up.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Set SSWKEN (LPSPI_AUTOCTL[7]) to enable Slave Selection
  *             wake up function. This bit is not related to Auto Operation.
  * \hideinitializer
  */
#define LPSPI_ENABLE_SS_WAKEUP(lpspi)   ((lpspi)->AUTOCTL |= LPSPI_AUTOCTL_SSWKEN_Msk)

/**
  * @brief      Disable Slave Selection wake up.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Clear SSWKEN (LPSPI_AUTOCTL[7]) to disable Slave Selection
  *             wake up function. This bit is not related to Auto Operation.
  * \hideinitializer
  */
#define LPSPI_DISABLE_SS_WAKEUP(lpspi)  ((lpspi)->AUTOCTL &= ~LPSPI_AUTOCTL_SSWKEN_Msk)

/**
  * @brief      Enable RX function in Auto Opertion TX phase.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Set FULLRXEN (LPSPI_AUTOCTL[6]) to enable RX function
  *             in Auto Operation TX phase.
  * \hideinitializer
  */
#define LPSPI_ENABLE_AUTO_FULLRX(lpspi) ((lpspi)->AUTOCTL |= LPSPI_AUTOCTL_FULLRXEN_Msk)

/**
  * @brief      Disable RX function in Auto Opertion TX phase.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Clear FULLRXEN (LPSPI_AUTOCTL[6]) to disable RX function
  *             in Auto Operation TX phase.
  * \hideinitializer
  */
#define LPSPI_DISABLE_AUTO_FULLRX(lpspi)    ((lpspi)->AUTOCTL &= ~LPSPI_AUTOCTL_FULLRXEN_Msk)

/**
  * @brief      Enable RX TCNT count match Interrupt.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Set CNTIEN (LPSPI_AUTOCTL[5]) to enable RX TCNT count
  *             match interrpu up function.
  * \hideinitializer
  */
#define LPSPI_ENABLE_AUTO_CNT_INT(lpspi)    ((lpspi)->AUTOCTL |= LPSPI_AUTOCTL_CNTIEN_Msk)

/**
  * @brief      Disable RX TCNT count match Interrupt.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Clear CNTIEN (LPSPI_AUTOCTL[5]) to disable RX TCNT count
  *             match interrpu up function.
  * \hideinitializer
  */
#define LPSPI_DISABLE_AUTO_CNT_INT(lpspi)   ((lpspi)->AUTOCTL &= ~LPSPI_AUTOCTL_CNTIEN_Msk)

/**
  * @brief      Enable Trigger function in Auto Opertion Mode.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Set TRIGEN (LPSPI_AUTOCTL[4]) to enable Trigger function
  *             for Trigger source from LPTMR0/1, TTMR0/1 and LPGPIO0/1/2/3
  *             in Auto Operation Mode.
  * \hideinitializer
  */
#define LPSPI_ENABLE_AUTO_TRIG(lpspi)   ((lpspi)->AUTOCTL |= LPSPI_AUTOCTL_TRIGEN_Msk)

/**
  * @brief      Disable Trigger function in Auto Opertion Mode.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Clear TRIGEN (LPSPI_AUTOCTL[4]) to disable Trigger function
  *             for Trigger source from LPTMR0/1, TTMR0/1 and LPGPIO0/1/2/3
  *             in Auto Operation Mode.
  * \hideinitializer
  */
#define LPSPI_DISABLE_AUTO_TRIG(lpspi)  ((lpspi)->AUTOCTL &= ~LPSPI_AUTOCTL_TRIGEN_Msk)

/**
  * @brief      Set Trigger Source Selection for Auto Operation Mode.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @param[in]  u32TrigSrc The triggered source specified in Auto Operation Mode.
  *                        This parameter could be only one of these following selections:
  *                          - \ref LPSPI_AUTOCTL_TRIGSEL_LPTMR0
  *                          - \ref LPSPI_AUTOCTL_TRIGSEL_LPTMR1
  *                          - \ref LPSPI_AUTOCTL_TRIGSEL_TTMR0
  *                          - \ref LPSPI_AUTOCTL_TRIGSEL_TTMR1
  *                          - \ref LPSPI_AUTOCTL_TRIGSEL_WKIOA
  *                          - \ref LPSPI_AUTOCTL_TRIGSEL_WKIOB
  *                          - \ref LPSPI_AUTOCTL_TRIGSEL_WKIOC
  *                          - \ref LPSPI_AUTOCTL_TRIGSEL_WKIOD
  * @return     None.
  * @details    Set Trigger Souce Selection (LPSPI_AUTOCTL[3:0]).
  * \hideinitializer
  */
#define LPSPI_SET_AUTO_TRIG_SOURCE(lpspi, u32TrigSrc)   \
    ((lpspi)->AUTOCTL = ((lpspi)->AUTOCTL & ~LPSPI_AUTOCTL_TRIGSEL_Msk)| \
                        ((u32TrigSrc & 0x0F) << LPSPI_AUTOCTL_TRIGSEL_Pos))

/**
  * @brief      Get TCNT count match wake up flag in Auto Operation Mode.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @retval     0 System is not woken up by CNT match in Auto Operation Mode.
  * @retval     1 System is woken up by CNT match in Auto Operation Mode.
  * @details    Read CNTWKF (LPSPI_AUTOSTS[3]) register to get the CNTWK flag.
  * \hideinitializer
  */
#define LPSPI_GET_AUTO_CNTWK_FLAG(lpspi)    \
    (((lpspi)->AUTOSTS & LPSPI_AUTOSTS_CNTWKF_Msk) >> LPSPI_AUTOSTS_CNTWKF_Pos)

/**
  * @brief      Clear TCNT count match wake up flag in Auto Operation Mode
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Write 1 to CNTWKF bit of LPSPI_AUTOSTATUS register to clear CNTWK flag.
  * \hideinitializer
  */
#define LPSPI_CLR_AUTO_CNTWK_FLAG(lpspi)    ((lpspi)->AUTOSTS = LPSPI_AUTOSTS_CNTWKF_Msk)

/**
  * @brief      Get Auto Busy flag in Auto Operation Mode.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @retval     0 No more request from triiger source during Auto Operation Mode
  * @retval     1 One more request from triiger source during Auto Operation Mode
  * @details    Read AOBUSY (LPSPI_AUTOSTS[2]) register to get the AOBUSY flag.
  * \hideinitializer
  */
#define LPSPI_GET_AUTO_AOBUSY_FLAG(lpspi)   \
    (((lpspi)->AUTOSTS & LPSPI_AUTOSTS_AOBUSY_Msk) >> LPSPI_AUTOSTS_AOBUSY_Pos)

/**
  * @brief      Clear Auto Busy flag in Auto Operation Mode.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Write 1 to AOBUSY bit of LPSPI_AUTOSTATUS register to clear AOBUSY flag.
  * \hideinitializer
  */
#define LPSPI_CLR_AUTO_AOBUSY_FLAG(lpspi)   ((lpspi)->AUTOSTS = LPSPI_AUTOSTS_AOBUSY_Msk)

/**
  * @brief      Get SS wake up flag.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @retval     0 System is not woken up by Slave Select flag.
  * @retval     1 System is woken up by Slave Select flag.
  * @details    Read SSWKF (LPSPI_AUTOSTS[1]) register to get the SSWKF flag.
  * \hideinitializer
  */
#define LPSPI_GET_SSWK_FLAG(lpspi)  \
    (((lpspi)->AUTOSTS & LPSPI_AUTOSTS_SSWKF_Msk) >> LPSPI_AUTOSTS_SSWKF_Pos)

/**
  * @brief      Clear SS wake up flag.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Write 1 to SSWKF bit of LPSPI_AUTOSTATUS register to clear SSWKF flag.
  * \hideinitializer
  */
#define LPSPI_CLR_SSWK_FLAG(lpspi)  ((lpspi)->AUTOSTS = LPSPI_AUTOSTS_SSWKF_Msk)

/**
  * @brief      Get TCNT count match interrupt flag in Auto Operation Mode.
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @retval     0 RX CNT is not matched in Auto Operation Mode.
  * @retval     1 RX CNT is matched in Auto Operation Mode.
  * @details    Read CNTIF (LPSPI_AUTOSTS[0]) register to get the CNTIF flag.
  * \hideinitializer
  */
#define LPSPI_GET_AUTO_CNT_INT_FLAG(lpspi)  \
    (((lpspi)->AUTOSTS & LPSPI_AUTOSTS_CNTIF_Msk) >> LPSPI_AUTOSTS_CNTIF_Pos)

/**
  * @brief      Clear TCNT count match interrupt flag in Auto Operation Mode
  * @param[in]  lpspi The pointer of the specified LPSPI module.
  * @return     None.
  * @details    Write 1 to CNTIF bit of LPSPI_AUTOSTATUS register to clear CNTIF flag.
  * \hideinitializer
  */
#define LPSPI_CLR_AUTO_CNT_INT_FLAG(lpspi)   ((lpspi)->AUTOSTS = LPSPI_AUTOSTS_CNTIF_Msk)

/* Function prototype declaration */
uint32_t LPSPI_Open(LPSPI_T *lpspi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void LPSPI_Close(LPSPI_T *lpspi);
void LPSPI_ClearRxFIFO(LPSPI_T *lpspi);
void LPSPI_ClearTxFIFO(LPSPI_T *lpspi);
void LPSPI_DisableAutoSS(LPSPI_T *lpspi);
void LPSPI_EnableAutoSS(LPSPI_T *lpspi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t LPSPI_SetBusClock(LPSPI_T *lpspi, uint32_t u32BusClock);
void LPSPI_SetFIFO(LPSPI_T *lpspi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
uint32_t LPSPI_GetBusClock(LPSPI_T *lpspi);
void LPSPI_EnableInt(LPSPI_T *lpspi, uint32_t u32Mask);
void LPSPI_DisableInt(LPSPI_T *lpspi, uint32_t u32Mask);
uint32_t LPSPI_GetIntFlag(LPSPI_T *lpspi, uint32_t u32Mask);
void LPSPI_ClearIntFlag(LPSPI_T *lpspi, uint32_t u32Mask);
uint32_t LPSPI_GetStatus(LPSPI_T *lpspi, uint32_t u32Mask);

/** @} end of group LPSPI_EXPORTED_FUNCTIONS */

/** @} end of group LPSPI_Driver */

/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __LPSPI_H__ */
