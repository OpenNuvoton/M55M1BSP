/**************************************************************************//**
 * @file     lpuart.h
 * @version  V1.00
 * @brief    M55M1 series LPUART driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __LPUART_H__
#define __LPUART_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LPUART_Driver LPUART Driver
  @{
*/

/** @addtogroup LPUART_EXPORTED_CONSTANTS LPUART Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* LPUART FIFO size constants definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define LPUART0_FIFO_SIZE 16UL /*!< LPUART0 supports separated receive/transmit 16/16 bytes entry FIFO */

/*---------------------------------------------------------------------------------------------------------*/
/* LPUART_FIFO constants definitions                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define LPUART_FIFO_RFITL_1BYTE      (0x0UL << LPUART_FIFO_RFITL_Pos)   /*!< LPUART_FIFO setting to set RX FIFO Trigger Level to 1 byte */
#define LPUART_FIFO_RFITL_4BYTES     (0x1UL << LPUART_FIFO_RFITL_Pos)   /*!< LPUART_FIFO setting to set RX FIFO Trigger Level to 4 bytes */
#define LPUART_FIFO_RFITL_8BYTES     (0x2UL << LPUART_FIFO_RFITL_Pos)   /*!< LPUART_FIFO setting to set RX FIFO Trigger Level to 8 bytes */
#define LPUART_FIFO_RFITL_14BYTES    (0x3UL << LPUART_FIFO_RFITL_Pos)   /*!< LPUART_FIFO setting to set RX FIFO Trigger Level to 14 bytes */

#define LPUART_FIFO_RTSTRGLV_1BYTE      (0x0UL << LPUART_FIFO_RTSTRGLV_Pos)  /*!< LPUART_FIFO setting to set RTS Trigger Level to 1 byte */
#define LPUART_FIFO_RTSTRGLV_4BYTES     (0x1UL << LPUART_FIFO_RTSTRGLV_Pos)  /*!< LPUART_FIFO setting to set RTS Trigger Level to 4 bytes */
#define LPUART_FIFO_RTSTRGLV_8BYTES     (0x2UL << LPUART_FIFO_RTSTRGLV_Pos)  /*!< LPUART_FIFO setting to set RTS Trigger Level to 8 bytes */
#define LPUART_FIFO_RTSTRGLV_14BYTES    (0x3UL << LPUART_FIFO_RTSTRGLV_Pos)  /*!< LPUART_FIFO setting to set RTS Trigger Level to 14 bytes */

/*---------------------------------------------------------------------------------------------------------*/
/* LPUART_LINE constants definitions                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define LPUART_WORD_LEN_5     (0UL) /*!< LPUART_LINE setting to set LPUART word length to 5 bits */
#define LPUART_WORD_LEN_6     (1UL) /*!< LPUART_LINE setting to set LPUART word length to 6 bits */
#define LPUART_WORD_LEN_7     (2UL) /*!< LPUART_LINE setting to set LPUART word length to 7 bits */
#define LPUART_WORD_LEN_8     (3UL) /*!< LPUART_LINE setting to set LPUART word length to 8 bits */

#define LPUART_PARITY_NONE    (0x0UL << LPUART_LINE_PBE_Pos) /*!< LPUART_LINE setting to set LPUART as no parity   */
#define LPUART_PARITY_ODD     (0x1UL << LPUART_LINE_PBE_Pos) /*!< LPUART_LINE setting to set LPUART as odd parity  */
#define LPUART_PARITY_EVEN    (0x3UL << LPUART_LINE_PBE_Pos) /*!< LPUART_LINE setting to set LPUART as even parity */
#define LPUART_PARITY_MARK    (0x5UL << LPUART_LINE_PBE_Pos) /*!< LPUART_LINE setting to keep parity bit as '1'  */
#define LPUART_PARITY_SPACE   (0x7UL << LPUART_LINE_PBE_Pos) /*!< LPUART_LINE setting to keep parity bit as '0'  */

#define LPUART_STOP_BIT_1     (0x0UL << LPUART_LINE_NSB_Pos) /*!< LPUART_LINE setting for one stop bit */
#define LPUART_STOP_BIT_1_5   (0x1UL << LPUART_LINE_NSB_Pos) /*!< LPUART_LINE setting for 1.5 stop bit when 5-bit word length */
#define LPUART_STOP_BIT_2     (0x1UL << LPUART_LINE_NSB_Pos) /*!< LPUART_LINE setting for two stop bit when 6, 7, 8-bit word length */

#define LPUART_AUTOCTL_TRIGSEL_SOFTWARE   (0UL)                                                             /*!< Low Power Auto-operation Trigger Source from Software \hideinitializer */
#define LPUART_AUTOCTL_TRIGSEL_LPTMR0     (0UL<<LPUART_AUTOCTL_TRIGSEL_Pos | LPUART_AUTOCTL_TRIGEN_Msk)     /*!< Low Power Auto-operation Trigger Source from LPTMR0   \hideinitializer */
#define LPUART_AUTOCTL_TRIGSEL_LPTMR1     (1UL<<LPUART_AUTOCTL_TRIGSEL_Pos | LPUART_AUTOCTL_TRIGEN_Msk)     /*!< Low Power Auto-operation Trigger Source from LPTMR1   \hideinitializer */
#define LPUART_AUTOCTL_TRIGSEL_TTMR0      (2UL<<LPUART_AUTOCTL_TRIGSEL_Pos | LPUART_AUTOCTL_TRIGEN_Msk)     /*!< Low Power Auto-operation Trigger Source from TTMR0    \hideinitializer */
#define LPUART_AUTOCTL_TRIGSEL_TTMR1      (3UL<<LPUART_AUTOCTL_TRIGSEL_Pos | LPUART_AUTOCTL_TRIGEN_Msk)     /*!< Low Power Auto-operation Trigger Source from TTMR1    \hideinitializer */
#define LPUART_AUTOCTL_TRIGSEL_WKIOA0     (4UL<<LPUART_AUTOCTL_TRIGSEL_Pos | LPUART_AUTOCTL_TRIGEN_Msk)     /*!< Low Power Auto-operation Trigger Source from WKIOA0.  \hideinitializer */
#define LPUART_AUTOCTL_TRIGSEL_WKIOB0     (5UL<<LPUART_AUTOCTL_TRIGSEL_Pos | LPUART_AUTOCTL_TRIGEN_Msk)     /*!< Low Power Auto-operation Trigger Source from WKIOB0.  \hideinitializer */
#define LPUART_AUTOCTL_TRIGSEL_WKIOC0     (6UL<<LPUART_AUTOCTL_TRIGSEL_Pos | LPUART_AUTOCTL_TRIGEN_Msk)     /*!< Low Power Auto-operation Trigger Source from WKIOC0.  \hideinitializer */
#define LPUART_AUTOCTL_TRIGSEL_WKIOD0     (7UL<<LPUART_AUTOCTL_TRIGSEL_Pos | LPUART_AUTOCTL_TRIGEN_Msk)     /*!< Low Power Auto-operation Trigger Source from WKIOD0.  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* LPUART RTS ACTIVE LEVEL constants definitions                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define LPUART_RTS_IS_LOW_LEV_ACTIVE   (0x1UL << LPUART_MODEM_RTSACTLV_Pos) /*!< Set RTS is Low Level Active */
#define LPUART_RTS_IS_HIGH_LEV_ACTIVE  (0x0UL << LPUART_MODEM_RTSACTLV_Pos) /*!< Set RTS is High Level Active */


/*---------------------------------------------------------------------------------------------------------*/
/* LPUART_FUNCSEL constants definitions                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define LPUART_FUNCSEL_LPUART  (0x0UL << LPUART_FUNCSEL_FUNCSEL_Pos)       /*!< LPUART_FUNCSEL setting to set LPUART Function (Default) */
#define LPUART_FUNCSEL_RS485   (0x3UL << LPUART_FUNCSEL_FUNCSEL_Pos)       /*!< LPUART_FUNCSEL setting to set RS485 Function          */




/*---------------------------------------------------------------------------------------------------------*/
/* LPUART BAUDRATE MODE constants definitions                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define LPUART_BAUD_MODE0     (0UL) /*!< Set LPUART Baudrate Mode is Mode0 */
#define LPUART_BAUD_MODE1     (LPUART_BAUD_BAUDM1_Msk)
#define LPUART_BAUD_MODE2     (LPUART_BAUD_BAUDM1_Msk | LPUART_BAUD_BAUDM0_Msk) /*!< Set LPUART Baudrate Mode is Mode2 */



/** @} end of group LPUART_EXPORTED_CONSTANTS */


/** @addtogroup LPUART_EXPORTED_FUNCTIONS LPUART Exported Functions
  @{
*/


/**
 *    @brief        Calculate LPUART baudrate mode0 divider
 *
 *    @param[in]    u32SrcFreq      LPUART clock frequency
 *    @param[in]    u32BaudRate     Baudrate of LPUART module
 *
 *    @return       LPUART baudrate mode0 divider
 *
 *    @details      This macro calculate LPUART baudrate mode0 divider.
 */
#define LPUART_BAUD_MODE0_DIVIDER(u32SrcFreq, u32BaudRate)    ((((u32SrcFreq) + ((u32BaudRate)*8ul)) / (u32BaudRate) >> 4ul)-2ul)


/**
 *    @brief        Calculate LPUART baudrate mode2 divider
 *
 *    @param[in]    u32SrcFreq      LPUART clock frequency
 *    @param[in]    u32BaudRate     Baudrate of LPUART module
 *
 *    @return       LPUART baudrate mode2 divider
 *
 *    @details      This macro calculate LPUART baudrate mode2 divider.
 */
#define LPUART_BAUD_MODE2_DIVIDER(u32SrcFreq, u32BaudRate)    ((((u32SrcFreq) + ((u32BaudRate)/2ul)) / (u32BaudRate))-2ul)


/**
 *    @brief        Write LPUART data
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module
 *    @param[in]    u8Data  Data byte to transmit.
 *
 *    @return       None
 *
 *    @details      This macro write Data to Tx data register.
 */
#define LPUART_WRITE(lpuart, u8Data)    ((lpuart)->DAT = (u8Data))


/**
 *    @brief        Read LPUART data
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module
 *
 *    @return       The oldest data byte in RX FIFO.
 *
 *    @details      This macro read Rx data register.
 */
#define LPUART_READ(lpuart)    ((lpuart)->DAT)


/**
 *    @brief        Get Tx empty
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module
 *
 *    @retval       0   Tx FIFO is not empty
 *    @retval       >=1 Tx FIFO is empty
 *
 *    @details      This macro get Transmitter FIFO empty register value.
 */
#define LPUART_GET_TX_EMPTY(lpuart)    ((lpuart)->FIFOSTS & LPUART_FIFOSTS_TXEMPTY_Msk)


/**
 *    @brief        Get Rx empty
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module
 *
 *    @retval       0   Rx FIFO is not empty
 *    @retval       >=1 Rx FIFO is empty
 *
 *    @details      This macro get Receiver FIFO empty register value.
 */
#define LPUART_GET_RX_EMPTY(lpuart)    ((lpuart)->FIFOSTS & LPUART_FIFOSTS_RXEMPTY_Msk)


/**
 *    @brief        Check specified lpuart port transmission is over.
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module
 *
 *    @retval       0 Tx transmission is not over
 *    @retval       1 Tx transmission is over
 *
 *    @details      This macro return Transmitter Empty Flag register bit value.
 *                  It indicates if specified lpuart port transmission is over nor not.
 */
#define LPUART_IS_TX_EMPTY(lpuart)    (((lpuart)->FIFOSTS & LPUART_FIFOSTS_TXEMPTYF_Msk) >> LPUART_FIFOSTS_TXEMPTYF_Pos)


/**
 *    @brief        Wait specified lpuart port transmission is over
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module
 *
 *    @return       None
 *
 *    @details      This macro wait specified lpuart port transmission is over.
 */
#define LPUART_WAIT_TX_EMPTY(lpuart)    while(!((((lpuart)->FIFOSTS) & LPUART_FIFOSTS_TXEMPTYF_Msk) >> LPUART_FIFOSTS_TXEMPTYF_Pos))


/**
 *    @brief        Check RX is ready or not
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module
 *
 *    @retval       0 The number of bytes in the RX FIFO is less than the RFITL
 *    @retval       1 The number of bytes in the RX FIFO equals or larger than RFITL
 *
 *    @details      This macro check receive data available interrupt flag is set or not.
 */
#define LPUART_IS_RX_READY(lpuart)    (((lpuart)->INTSTS & LPUART_INTSTS_RDAIF_Msk)>>LPUART_INTSTS_RDAIF_Pos)


/**
 *    @brief        Check TX FIFO is full or not
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module
 *
 *    @retval       1 TX FIFO is full
 *    @retval       0 TX FIFO is not full
 *
 *    @details      This macro check TX FIFO is full or not.
 */
#define LPUART_IS_TX_FULL(lpuart)    (((lpuart)->FIFOSTS & LPUART_FIFOSTS_TXFULL_Msk)>>LPUART_FIFOSTS_TXFULL_Pos)


/**
 *    @brief        Check RX FIFO is full or not
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module
 *
 *    @retval       1 RX FIFO is full
 *    @retval       0 RX FIFO is not full
 *
 *    @details      This macro check RX FIFO is full or not.
 */
#define LPUART_IS_RX_FULL(lpuart)    (((lpuart)->FIFOSTS & LPUART_FIFOSTS_RXFULL_Msk)>>LPUART_FIFOSTS_RXFULL_Pos)


/**
 *    @brief        Get Tx full register value
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module
 *
 *    @retval       0   Tx FIFO is not full.
 *    @retval       >=1 Tx FIFO is full.
 *
 *    @details      This macro get Tx full register value.
 */
#define LPUART_GET_TX_FULL(lpuart)    ((lpuart)->FIFOSTS & LPUART_FIFOSTS_TXFULL_Msk)


/**
 *    @brief        Get Rx full register value
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module
 *
 *    @retval       0   Rx FIFO is not full.
 *    @retval       >=1 Rx FIFO is full.
 *
 *    @details      This macro get Rx full register value.
 */
#define LPUART_GET_RX_FULL(lpuart)    ((lpuart)->FIFOSTS & LPUART_FIFOSTS_RXFULL_Msk)

/**
 *    @brief        Rx Idle Status register value
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module
 *
 *    @retval       0 Rx is busy.
 *    @retval       1 Rx is Idle(Default)
 *
 *    @details      This macro get Rx Idle Status register value.
 *    \hideinitializer
 */
#define LPUART_RX_IDLE(lpuart) (((lpuart)->FIFOSTS & LPUART_FIFOSTS_RXIDLE_Msk )>> LPUART_FIFOSTS_RXIDLE_Pos)


/**
 *    @brief        Enable specified LPUART interrupt
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module
 *    @param[in]    u32eIntSel  Interrupt type select
 *                              - \ref LPUART_INTEN_TXENDIEN_Msk   : Transmitter empty interrupt
 *                              - \ref LPUART_INTEN_ABRIEN_Msk     : Auto baud rate interrupt
 *                              - \ref LPUART_INTEN_WKIEN_Msk      : Wake-up interrupt
 *                              - \ref LPUART_INTEN_BUFERRIEN_Msk  : Buffer Error interrupt
 *                              - \ref LPUART_INTEN_RXTOIEN_Msk    : Rx time-out interrupt
 *                              - \ref LPUART_INTEN_MODEMIEN_Msk   : Modem interrupt
 *                              - \ref LPUART_INTEN_RLSIEN_Msk     : Rx Line status interrupt
 *                              - \ref LPUART_INTEN_THREIEN_Msk    : Tx empty interrupt
 *                              - \ref LPUART_INTEN_RDAIEN_Msk     : Rx ready interrupt
 *
 *    @return       None
 *
 *    @details      This macro enable specified LPUART interrupt.
 */
#define LPUART_ENABLE_INT(lpuart, u32eIntSel)    ((lpuart)->INTEN |= (u32eIntSel))


/**
 *    @brief        Disable specified LPUART interrupt
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module
 *    @param[in]    u32eIntSel  Interrupt type select
 *                              - \ref LPUART_INTEN_TXENDIEN_Msk   : Transmitter Empty Interrupt
 *                              - \ref LPUART_INTEN_ABRIEN_Msk     : Auto-baud Rate Interrupt
 *                              - \ref LPUART_INTEN_WKIEN_Msk      : Wake-up interrupt
 *                              - \ref LPUART_INTEN_BUFERRIEN_Msk  : Buffer Error interrupt
 *                              - \ref LPUART_INTEN_RXTOIEN_Msk    : Rx Time-out Interrupt
 *                              - \ref LPUART_INTEN_MODEMIEN_Msk   : MODEM Status Interrupt
 *                              - \ref LPUART_INTEN_RLSIEN_Msk     : Receive Line Status Interrupt
 *                              - \ref LPUART_INTEN_THREIEN_Msk    : Transmit Holding Register Empty Interrupt
 *                              - \ref LPUART_INTEN_RDAIEN_Msk     : Receive Data Available Interrupt
 *
 *    @return       None
 *
 *    @details      This macro enable specified LPUART interrupt.
 */
#define LPUART_DISABLE_INT(lpuart, u32eIntSel)    ((lpuart)->INTEN &= ~ (u32eIntSel))


/**
 *    @brief        Get specified interrupt flag/status
 *
 *    @param[in]    lpuart            The pointer of the specified LPUART module
 *    @param[in]    u32eIntTypeFlag Interrupt Type Flag, should be
 *                                  - \ref LPUART_INTSTS_ABRINT_Msk    : Auto-baud Rate Interrupt Indicator
 *                                  - \ref LPUART_INTSTS_HWBUFEINT_Msk : PDMA Mode Buffer Error Interrupt Indicator
 *                                  - \ref LPUART_INTSTS_HWTOINT_Msk   : PDMA Mode Rx Time-out Interrupt Indicator
 *                                  - \ref LPUART_INTSTS_HWMODINT_Msk  : PDMA Mode MODEM Status Interrupt Indicator
 *                                  - \ref LPUART_INTSTS_HWRLSINT_Msk  : PDMA Mode Receive Line Status Interrupt Indicator
 *                                  - \ref LPUART_INTSTS_HWBUFEIF_Msk  : PDMA Mode Buffer Error Interrupt Flag
 *                                  - \ref LPUART_INTSTS_HWTOIF_Msk    : PDMA Mode Time-out Interrupt Flag
 *                                  - \ref LPUART_INTSTS_HWMODIF_Msk   : PDMA Mode MODEM Status Interrupt Flag
 *                                  - \ref LPUART_INTSTS_HWRLSIF_Msk   : PDMA Mode Receive Line Status Flag
 *                                  - \ref LPUART_INTSTS_TXENDINT_Msk  : Transmitter Empty Interrupt Indicator
 *                                  - \ref LPUART_INTSTS_WKINT_Msk     : Wake-up Interrupt Indicator
 *                                  - \ref LPUART_INTSTS_BUFERRINT_Msk : Buffer Error Interrupt Indicator
 *                                  - \ref LPUART_INTSTS_RXTOINT_Msk   : Rx Time-out Interrupt Indicator
 *                                  - \ref LPUART_INTSTS_MODEMINT_Msk  : Modem Status Interrupt Indicator
 *                                  - \ref LPUART_INTSTS_RLSINT_Msk    : Receive Line Status Interrupt Indicator
 *                                  - \ref LPUART_INTSTS_THREINT_Msk   : Transmit Holding Register Empty Interrupt Indicator
 *                                  - \ref LPUART_INTSTS_RDAINT_Msk    : Receive Data Available Interrupt Indicator
 *                                  - \ref LPUART_INTSTS_TXENDIF_Msk   : Transmitter Empty Interrupt Flag
 *                                  - \ref LPUART_INTSTS_WKIF_Msk      : Wake-up Interrupt Flag
 *                                  - \ref LPUART_INTSTS_BUFERRIF_Msk  : Buffer Error Interrupt Flag
 *                                  - \ref LPUART_INTSTS_RXTOIF_Msk    : Rx Time-out Interrupt Flag
 *                                  - \ref LPUART_INTSTS_MODEMIF_Msk   : MODEM Status Interrupt Flag
 *                                  - \ref LPUART_INTSTS_RLSIF_Msk     : Receive Line Status Interrupt Flag
 *                                  - \ref LPUART_INTSTS_THREIF_Msk    : Transmit Holding Register Empty Interrupt Flag
 *                                  - \ref LPUART_INTSTS_RDAIF_Msk     : Receive Data Available Interrupt Flag
 *
 *    @retval       0 The specified interrupt is not happened.
 *                  1 The specified interrupt is happened.
 *
 *    @details      This macro get specified interrupt flag or interrupt indicator status.
 */
#define LPUART_GET_INT_FLAG(lpuart,u32eIntTypeFlag)    (((lpuart)->INTSTS & (u32eIntTypeFlag))?1:0)


/**
 *    @brief        Enable specified LPUART Deglitch function
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module
 *
 *    @return       None
 *
 *    \hideinitializer
 */
#define LPUART_DEGLITCH_ENABLE(lpuart)    ((lpuart)->FUNCSEL |= LPUART_FUNCSEL_DGE_Msk)
/**
 *    @brief        Disable specified LPUART Deglitch function
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module
 *
 *    @return       None
 *
 *    \hideinitializer
 */
#define LPUART_DEGLITCH_DISABLE(lpuart)    ((lpuart)->FUNCSEL &= ~LPUART_FUNCSEL_DGE_Msk)
/**
 *    @brief        Enable specific LPUART swap TX pin and RX pin
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module
 *
 *    @return       None
 *
 *    \hideinitializer
 */
#define LPUART_TXRX_SWAP_ENABLE(lpuart)    ((lpuart)->FUNCSEL |= LPUART_FUNCSEL_TXRXSWP_Msk)
/**
 *    @brief        Disable specific UART swap TX pin and RX pin
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module
 *
 *    @return       None
 *
 *    \hideinitializer
 */
#define LPUART_TXRX_SWAP_DISABLE(lpuart)   ((lpuart)->FUNCSEL &= ~LPUART_FUNCSEL_TXRXSWP_Msk)

/*---------------------------------------------------------------------------------------------------------*/
/* static inline functions                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/

static __INLINE void LPUART_CLEAR_RTS(LPUART_T *lpuart);
static __INLINE void LPUART_SET_RTS(LPUART_T *lpuart);


/**
 *    @brief        Set RTS pin to low
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module
 *
 *    @return       None
 *
 *    @details      This macro set RTS pin to low.
 */
__STATIC_INLINE void LPUART_CLEAR_RTS(LPUART_T *lpuart)
{
    lpuart->MODEM |= LPUART_MODEM_RTSACTLV_Msk;
    lpuart->MODEM &= ~LPUART_MODEM_RTS_Msk;
}


/**
 *    @brief        Set RTS pin to high
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module
 *
 *    @return       None
 *
 *    @details      This macro set RTS pin to high.
 */
__STATIC_INLINE void LPUART_SET_RTS(LPUART_T *lpuart)
{
    lpuart->MODEM |= LPUART_MODEM_RTSACTLV_Msk | LPUART_MODEM_RTS_Msk;
}


/**
 *    @brief        Clear RS-485 Address Byte Detection Flag
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module
 *
 *    @return       None
 *
 *    @details      This macro clear RS-485 address byte detection flag.
 */
#define LPUART_RS485_CLEAR_ADDR_FLAG(lpuart)    ((lpuart)->FIFOSTS = LPUART_FIFOSTS_ADDRDETF_Msk)


/**
 *    @brief        Get RS-485 Address Byte Detection Flag
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module
 *
 *    @retval       0 Receiver detects a data that is not an address bit.
 *    @retval       1 Receiver detects a data that is an address bit.
 *
 *    @details      This macro get RS-485 address byte detection flag.
 */
#define LPUART_RS485_GET_ADDR_FLAG(lpuart)    (((lpuart)->FIFOSTS  & LPUART_FIFOSTS_ADDRDETF_Msk) >> LPUART_FIFOSTS_ADDRDETF_Pos)
/**
 *    @brief        Set specified RS485 transceiver deactivate delay value
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module
 *    @param[in]    u32RSDly    Deactivate delay value
 *
 *    @return       None
 *
 *    @details       This macro set RS485 transceiver deactivate delay value.
 */
#define LPUART_RS485RTSDLY_SET(lpuart, u32RSDly)    ((lpuart)->RS485DD = ((lpuart)->RS485DD &(~LPUART_RS485DD_RTSDDLY_Msk))|u32RSDly)

/**
 *    @brief        Enable specified LPUART PDMA function
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module
 *    @param[in]    u32FuncSel  Combination of following functions
 *                             - \ref LPUART_INTEN_TXPDMAEN_Msk
 *                             - \ref LPUART_INTEN_RXPDMAEN_Msk
 *
 *    @return       None
 *
 *    @details      This macro enable specified LPUART PDMA function.
 */
#define LPUART_PDMA_ENABLE(lpuart, u32FuncSel)    ((lpuart)->INTEN |= (u32FuncSel))


/**
 *    @brief        Disable specified LPUART PDMA function
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module
 *    @param[in]    u32FuncSel  Combination of following functions
 *                             - \ref LPUART_INTEN_TXPDMAEN_Msk
 *                             - \ref LPUART_INTEN_RXPDMAEN_Msk
 *
 *    @return       None
 *
 *    @details      This macro disable specified LPUART PDMA function.
 */
#define LPUART_PDMA_DISABLE(lpuart, u32FuncSel)    ((lpuart)->INTEN &= ~(u32FuncSel))

/**
 *    @brief        Enable specified LPUART Automatic Operation function
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module
 *
 *    @return       None
 *
 *    @details      This macro enable specified LPUART Automatic Operation function.
 */
#define LPUART_AUTO_OP_ENABLE(lpuart)    ((lpuart)->AUTOCTL |= LPUART_AUTOCTL_AOEN_Msk)

/**
 *    @brief        Disable specified LPUART Automatic Operation function
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module
 *
 *    @return       None
 *
 *    @details      This macro disable specified LPUART Automatic Operation function.
 */
#define LPUART_AUTO_OP_DISABLE(lpuart)    ((lpuart)->AUTOCTL &= ~LPUART_AUTOCTL_AOEN_Msk)

/**
 *    @brief        Enable specified LPUART Automatic Operation Clock Always-on function
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module
 *
 *    @return       None
 *
 *    @details      This macro enable specified LPUART Automatic Operation Clock Always-on function.
 */
#define LPUART_AOUT_OP_CLOCK_ALWAYS_ON_ENABLE(lpuart)    ((lpuart)->AUTOCTL |= LPUART_AUTOCTL_CKAWOEN_Msk)

/**
 *    @brief        Disable specified LPUART Automatic Operation Clock Always-on function
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module
 *
 *    @return       None
 *
 *    @details      This macro disable specified LPUART Automatic Operation Clock Always-on function.
 */
#define LPUART_AOUT_OP_CLOCK_ALWAYS_ON_DISABLE(lpuart)    ((lpuart)->AUTOCTL &= ~LPUART_AUTOCTL_CKAWOEN_Msk)

/**
 *    @brief        Enable specified LPUART Automatic Operation Software Trigger function
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module
 *
 *    @return       None
 *
 *    @details      This macro enable specified LPUART Automatic Operation Software Trigger function.
 */
#define LPUART_AUTO_OP_SW_TRIGGER_ENABLE(lpuart)    ((lpuart)->AUTOCTL |= LPUART_AUTOCTL_SWTRIG_Msk)

/**
 *    @brief        Enable specified LPUART Bus Idle Time-out Wake-Up function
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module
 *
 *    @return       None
 *
 *    \hideinitializer
 */

#define LPUART_BUS_IDLE_TIMEOUT_WAKEUP_ENABLE(lpuart)    ((lpuart)->AUTOCTL |= LPUART_AUTOCTL_WKAOTOEN_Msk)
/**
 *    @brief        Disable specified LPUART Bus Idle Time-out Wake-Up  function
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module
 *
 *    @return       None
 *
 *    \hideinitializer
 */
#define LPUART_BUS_IDLE_TIMEOUT_WAKEUP_DISABLE(lpuart)    ((lpuart)->AUTOCTL &= ~LPUART_AUTOCTL_WKAOTOEN_Msk)

/**
 *    @brief        Bus Idle Time-out Wake-up Status register value
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module
 *
 *    @retval       0 There is no Bus Idle Time-out Wake-up.
 *    @retval       1 Chip wake-up from power-down state by Bus Idle Time-out Wake-up
 *
 *    @details      This macro get Bus Idle Time-out Wake-up Status register value.
 *    \hideinitializer
 */
#define LPUART_IS_BUS_IDLE_TIMEOUT_WAKEUP(lpuart) (((lpuart)->AUTOSTS & LPUART_AUTOSTS_AOTOWKF_Msk )>> LPUART_AUTOSTS_AOTOWKF_Pos)
/**
 *    @brief        Clear Bus Idle Time-out Wake-up Flag
 *
 *    @param[in]    lpuart    The pointer of the specified LPUART module
 *
 *    @return       None
 *
 *    @details      This macro clear Bus Idle time-out wake-up flag.
 */
#define LPUART_CLEAR_BUS_IDLE_TIMEOUT_WAKEUP_FLAG(lpuart)    ((lpuart)->AUTOSTS = LPUART_AUTOSTS_AOTOWKF_Msk)
/**
 *    @brief        Enable specified LPUART Bus Idle Time-out function
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module
 *
 *    @return       None
 *
 *    \hideinitializer
 */
#define LPUART_BUS_IDLE_TIMEOUT_ENABLE(lpuart)    ((lpuart)->TOUT |= LPUART_TOUT_BITOMEN_Msk)
/**
 *    @brief        Disable specified LPUART Bus Idle Time-out function
 *
 *    @param[in]    lpuart        The pointer of the specified LPUART module
 *
 *    @return       None
 *
 *    \hideinitializer
 */
#define LPUART_BUS_IDLE_TIMEOUT_DISABLE(lpuart)    ((lpuart)->TOUT &= ~LPUART_TOUT_BITOMEN_Msk)


/* Function prototype declaration */
void LPUART_ClearIntFlag(LPUART_T *lpuart, uint32_t u32InterruptFlag);
void LPUART_Close(LPUART_T *lpuart);
void LPUART_DisableFlowCtrl(LPUART_T *lpuart);
void LPUART_DisableInt(LPUART_T  *lpuart, uint32_t u32InterruptFlag);
void LPUART_EnableFlowCtrl(LPUART_T *lpuart);
void LPUART_EnableInt(LPUART_T  *lpuart, uint32_t u32InterruptFlag);
void LPUART_Open(LPUART_T *lpuart, uint32_t u32baudrate);
uint32_t LPUART_Read(LPUART_T *lpuart, uint8_t pu8RxBuf[], uint32_t u32ReadBytes);
void LPUART_SetLineConfig(LPUART_T *lpuart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits);
void LPUART_SetTimeoutCnt(LPUART_T *lpuart, uint32_t u32TOC);
void LPUART_SelectRS485Mode(LPUART_T *lpuart, uint32_t u32Mode, uint32_t u32Addr);
uint32_t LPUART_Write(LPUART_T *lpuart, uint8_t pu8TxBuf[], uint32_t u32WriteBytes);
void LPUART_SelectAutoOperationMode(LPUART_T *lpuart, uint32_t u32TrigSel);

/** @} end of group LPUART_EXPORTED_FUNCTIONS */

/** @} end of group LPUART_Driver */

/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __LPUART_H__ */
