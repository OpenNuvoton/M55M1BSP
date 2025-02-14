/**************************************************************************//**
 * @file     lpuart_reg.h
 * @version  V1.00
 * @brief    LPUART register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __LPUART_REG_H__
#define __LPUART_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup LPUART Low Power UART Interface Controller (LPUART)
    Memory Mapped Structure for LPUART Controller
    @{
*/

typedef struct
{


    /**
     * @var LPUART_T::DAT
     * Offset: 0x00  LPUART Receive/Transmit Buffer Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DAT       |Data Receive/Transmit Buffer
     * |        |          |Write Operation:
     * |        |          |By writing one byte to this register, the data byte will be stored in transmitter FIFO
     * |        |          |The LPUART controller will send out the data stored in transmitter FIFO top location through the LPUART_TXD.
     * |        |          |Read Operation:
     * |        |          |By reading this register, the LPUART controller will return an 8-bit data received from receiver FIFO.
     * |[8]     |PARITY    |Parity Bit Receive/Transmit Buffer
     * |        |          |Write Operation:
     * |        |          |By writing to this bit, the parity bit will be stored in transmitter FIFO
     * |        |          |If PBE (LPUART_LINE[3]) and PSS (LPUART_LINE[7]) are set,
     * |        |          |the LPUART controller will send out this bit follow the DAT (LPUART_DAT[7:0]) through the LPUART_TXD.
     * |        |          |Read Operation:
     * |        |          |If PBE (LPUART_LINE[3]) and PSS (LPUART_LINE[7]) are enabled, the parity bit can be read by this bit.
     * |        |          |Note: This bit has effect only when PBE (LPUART_LINE[3]) and PSS (LPUART_LINE[7]) are set.
     * @var LPUART_T::INTEN
     * Offset: 0x04  LPUART Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RDAIEN    |Receive Data Available Interrupt Enable Bit
     * |        |          |0 = Receive data available interrupt Disabled.
     * |        |          |1 = Receive data available interrupt Enabled.
     * |[1]     |THREIEN   |Transmit Holding Register Empty Interrupt Enable Bit
     * |        |          |0 = Transmit holding register empty interrupt Disabled.
     * |        |          |1 = Transmit holding register empty interrupt Enabled.
     * |[2]     |RLSIEN    |Receive Line Status Interrupt Enable Bit
     * |        |          |0 = Receive Line Status interrupt Disabled.
     * |        |          |1 = Receive Line Status interrupt Enabled.
     * |[3]     |MODEMIEN  |Modem Status Interrupt Enable Bit
     * |        |          |0 = Modem status interrupt Disabled.
     * |        |          |1 = Modem status interrupt Enabled.
     * |[4]     |RXTOIEN   |RX Time-out Interrupt Enable Bit
     * |        |          |0 = RX time-out interrupt Disabled.
     * |        |          |1 = RX time-out interrupt Enabled.
     * |[5]     |BUFERRIEN |Buffer Error Interrupt Enable Bit
     * |        |          |0 = Buffer error interrupt Disabled.
     * |        |          |1 = Buffer error interrupt Enabled.
     * |[6]     |WKIEN     |Wake-up Interrupt Enable Bit
     * |        |          |0 = Wake-up Interrupt Disabled.
     * |        |          |1 = Wake-up Interrupt Enabled.
     * |[11]    |TOCNTEN   |Receive Buffer Time-out Counter Enable Bit
     * |        |          |0 = Receive Buffer Time-out counter Disabled.
     * |        |          |1 = Receive Buffer Time-out counter Enabled.
     * |[12]    |ATORTSEN  |nRTS Auto-flow Control Enable Bit
     * |        |          |0 = nRTS auto-flow control Disabled.
     * |        |          |1 = nRTS auto-flow control Enabled.
     * |        |          |Note: When nRTS auto-flow is enabled, if the number of bytes in the RX FIFO equals the RTSTRGLV (LPUART_FIFO[19:16]), the LPUART will de-assert nRTS signal.
     * |[13]    |ATOCTSEN  |nCTS Auto-flow Control Enable Bit
     * |        |          |0 = nCTS auto-flow control Disabled.
     * |        |          |1 = nCTS auto-flow control Enabled.
     * |        |          |Note: When nCTS auto-flow is enabled, the LPUART will send data to external device if nCTS input assert (LPUART will not send data to device until nCTS is asserted).
     * |[14]    |TXPDMAEN  |TX LPPDMA Enable Bit
     * |        |          |0 = TX LPPDMA Disabled.
     * |        |          |1 = TX LPPDMA Enabled.
     * |        |          |Note: If RLSIEN (LPUART_INTEN[2]) is enabled and HWRLSINT (LPUART_INTSTS[26]) is set to 1, the RLS (Receive Line Status) Interrupt is caused
     * |        |          |If RLS interrupt is caused by Break Error Flag BIF(LPUART_FIFOSTS[6]), Frame Error Flag FEF(LPUART_FIFO[5]) or Parity Error Flag PEF(LPUART_FIFOSTS[4]), LPUART LPPDMA transmit request operation is stopped
     * |        |          |Clear Break Error Flag BIF or Frame Error Flag FEF or Parity Error Flag PEF by writing '1' to corresponding BIF, FEF and PEF to make LPUART LPPDMA transmit request operation continue.
     * |[15]    |RXPDMAEN  |RX LPPDMA Enable Bit
     * |        |          |This bit can enable or disable RX LPPDMA service.
     * |        |          |0 = RX LPPDMA Disabled.
     * |        |          |1 = RX LPPDMA Enabled.
     * |        |          |Note: If RLSIEN (LPUART_INTEN[2]) is enabled and HWRLSINT (LPUART_INTSTS[26]) is set to 1, the RLS (Receive Line Status) Interrupt is caused
     * |        |          |If RLS interrupt is caused by Break Error Flag BIF(LPUART_FIFOSTS[6]), Frame Error Flag FEF(LPUART_FIFO[5]) or Parity Error Flag PEF(LPUART_FIFOSTS[4]), LPUART LPPDMA receive request operation is stopped
     * |        |          |Clear Break Error Flag BIF or Frame Error Flag FEF or Parity Error Flag PEF by writing '1' to corresponding BIF, FEF and PEF to make LPUART LPPDMA receive request operation continue.
     * |[18]    |ABRIEN    |Auto-baud Rate Interrupt Enable Bit
     * |        |          |0 = Auto-baud rate interrupt Disabled.
     * |        |          |1 = Auto-baud rate interrupt Enabled.
     * |[22]    |TXENDIEN  |Transmitter Empty Interrupt Enable Bit
     * |        |          |If TXENDIEN (LPUART_INTEN[22]) is enabled, the Transmitter Empty interrupt TXENDINT (LPUART_INTSTS[30]) will be generated when TXENDIF (LPUART_INTSTS[22]) is set (TX FIFO (LPUART_DAT) is empty and the STOP bit of the last byte has been transmitted).
     * |        |          |0 = Transmitter empty interrupt Disabled.
     * |        |          |1 = Transmitter empty interrupt Enabled.
     * @var LPUART_T::FIFO
     * Offset: 0x08  LPUART FIFO Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |RXRST     |RX Field Software Reset
     * |        |          |When RXRST (LPUART_FIFO[1]) is set, all the byte in the receiver FIFO and RX internal state machine are cleared.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset the RX internal state machine and pointers.
     * |        |          |Note 1: This bit will automatically clear at least 3 LPUART peripheral clock cycles.
     * |        |          |Note 2: Before setting this bit, it should wait for the RXIDLE (LPUART_FIFOSTS[29]) be set.
     * |[2]     |TXRST     |TX Field Software Reset
     * |        |          |When TXRST (LPUART_FIFO[2]) is set, all the byte in the transmit FIFO and TX internal state machine are cleared.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset the TX internal state machine and pointers.
     * |        |          |Note 1: This bit will automatically clear at least 3 LPUART peripheral clock cycles.
     * |        |          |Note 2: Before setting this bit, it should wait for the TXEMPTYF (LPUART_FIFOSTS[28]) be set.
     * |[7:4]   |RFITL     |RX FIFO Interrupt Trigger Level
     * |        |          |When the number of bytes in the receive FIFO equals the RFITL, the RDAIF (LPUART_INTSTS[0]) will be set (if RDAIEN (LPUART_INTEN [0]) enabled, and an interrupt will be generated).
     * |        |          |0000 = RX FIFO Interrupt Trigger Level is 1 byte.
     * |        |          |0001 = RX FIFO Interrupt Trigger Level is 4 bytes.
     * |        |          |0010 = RX FIFO Interrupt Trigger Level is 8 bytes.
     * |        |          |0011 = RX FIFO Interrupt Trigger Level is 14 bytes.
     * |        |          |Others = Reserved.
     * |[8]     |RXOFF     |Receiver Disable Bit
     * |        |          |The receiver is disabled or not (set 1 to disable receiver).
     * |        |          |0 = Receiver Enabled.
     * |        |          |1 = Receiver Disabled.
     * |        |          |Note: This bit is used for RS-485 Normal Multi-drop mode
     * |        |          |It should be programmed before RS485NMM (LPUART_ALTCTL [8]) is programmed.
     * |[19:16] |RTSTRGLV  |nRTS Trigger Level for Auto-flow Control
     * |        |          |0000 = nRTS Trigger Level is 1 byte.
     * |        |          |0001 = nRTS Trigger Level is 4 bytes.
     * |        |          |0010 = nRTS Trigger Level is 8 bytes.
     * |        |          |0011 = nRTS Trigger Level is 14 bytes.
     * |        |          |Others = Reserved.
     * |        |          |Note: This field is used for auto nRTS flow control.
     * @var LPUART_T::LINE
     * Offset: 0x0C  LPUART Line Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |WLS       |Word Length Selection
     * |        |          |This field sets LPUART word length.
     * |        |          |00 = 5 bits.
     * |        |          |01 = 6 bits.
     * |        |          |10 = 7 bits.
     * |        |          |11 = 8 bits.
     * |[2]     |NSB       |Number of 'STOP Bit'
     * |        |          |0 = One 'STOP Bit' is generated in the transmitted data.
     * |        |          |1 = When select 5-bit word length, 1.5 'STOP Bit' is generated in the transmitted data
     * |        |          |When select 6-, 7- and 8-bit word length, 2 'STOP Bit' is generated in the transmitted data.
     * |[3]     |PBE       |Parity Bit Enable Bit
     * |        |          |0 = Parity bit generated Disabled.
     * |        |          |1 = Parity bit generated Enabled.
     * |        |          |Note: Parity bit is generated on each outgoing character and is checked on each incoming data.
     * |[4]     |EPE       |Even Parity Enable Bit
     * |        |          |0 = Odd number of logic '1's is transmitted and checked in each word.
     * |        |          |1 = Even number of logic '1's is transmitted and checked in each word.
     * |        |          |Note: This bit has effect only when PBE (LPUART_LINE[3]) is set.
     * |[5]     |SPE       |Stick Parity Enable Bit
     * |        |          |0 = Stick parity Disabled.
     * |        |          |1 = Stick parity Enabled.
     * |        |          |Note: If PBE (LPUART_LINE[3]) and EPE (LPUART_LINE[4]) are logic 1, the parity bit is transmitted and checked as logic 0
     * |        |          |If PBE (LPUART_LINE[3]) is 1 and EPE (LPUART_LINE[4]) is 0 then the parity bit is transmitted and checked as 1.
     * |[6]     |BCB       |Break Control Bit
     * |        |          |0 = Break Control Disabled.
     * |        |          |1 = Break Control Enabled.
     * |        |          |Note: When this bit is set to logic 1, the transmitted serial data output (TX) is forced to the Spacing State (logic 0)
     * |        |          |This bit acts only on TX line and has no effect on the transmitter logic.
     * |[7]     |PSS       |Parity Bit Source Selection
     * |        |          |The parity bit can be selected to be generated and checked automatically or by software.
     * |        |          |0 = Parity bit is generated by EPE (LPUART_LINE[4]) and SPE (LPUART_LINE[5]) setting and checked automatically.
     * |        |          |1 = Parity bit generated and checked by software.
     * |        |          |Note 1: This bit has effect only when PBE (LPUART_LINE[3]) is set.
     * |        |          |Note 2: If PSS is 0, the parity bit is transmitted and checked automatically
     * |        |          |If PSS is 1, the transmitted parity bit value can be determined by writing PARITY (LPUART_DAT[8]) and the parity bit can be read by reading PARITY (LPUART_DAT[8]).
     * |[8]     |TXDINV    |TX Data Inverted
     * |        |          |0 = Transmitted data signal inverted Disabled.
     * |        |          |1 = Transmitted data signal inverted Enabled.
     * |        |          |Note 1: Before setting this bit, TXRXDIS (LPUART_FUNCSEL[3]) should be set then waited for TXRXACT (LPUART_FIFOSTS[31]) is cleared
     * |        |          |When the configuration is done, cleared TXRXDIS (LPUART_FUNCSEL[3]) to activate LPUART controller.
     * |        |          |Note 2: This bit is valid when FUNCSEL (LPUART_FUNCSEL[2:0]) is select LPUART or RS485 function.
     * |[9]     |RXDINV    |RX Data Inverted
     * |        |          |0 = Received data signal inverted Disabled.
     * |        |          |1 = Received data signal inverted Enabled.
     * |        |          |Note 1: Before setting this bit, TXRXDIS (LPUART_FUNCSEL[3]) should be set then waited for TXRXACT (LPUART_FIFOSTS[31]) is cleared
     * |        |          |When the configuration is done, cleared TXRXDIS (LPUART_FUNCSEL[3]) to activate LPUART controller.
     * |        |          |Note 2: This bit is valid when FUNCSEL (LPUART_FUNCSEL[2:0]) is select LPUART or RS485 function.
     * @var LPUART_T::MODEM
     * Offset: 0x10  LPUART Modem Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |RTS       |nRTS (Request-to-send) Signal Control
     * |        |          |This bit is direct control internal nRTS signal active or not, and then drive the nRTS pin output with RTSACTLV bit configuration.
     * |        |          |0 = nRTS signal is active.
     * |        |          |1 = nRTS signal is inactive.
     * |        |          |Note 1: The nRTS signal control bit is not effective when nRTS auto-flow control is enabled in LPUART function mode.
     * |        |          |Note 2: The nRTS signal control bit is not effective when RS-485 auto direction mode (AUD) is enabled in RS-485 function mode.
     * |[9]     |RTSACTLV  |nRTS Pin Active Level
     * |        |          |This bit defines the active level state of nRTS pin output.
     * |        |          |0 = nRTS pin output is high level active.
     * |        |          |1 = nRTS pin output is low level active. (Default)
     * |        |          |Note: Before setting this bit, TXRXDIS (LPUART_FUNCSEL[3]) should be set then waited for TXRXACT (LPUART_FIFOSTS[31]) is cleared
     * |        |          |When the configuration is done, cleared TXRXDIS (LPUART_FUNCSEL[3]) to activate LPUART controller.
     * |[13]    |RTSSTS    |nRTS Pin Status (Read Only)
     * |        |          |This bit mirror from nRTS pin output of voltage logic status.
     * |        |          |0 = nRTS pin output is low level voltage logic state.
     * |        |          |1 = nRTS pin output is high level voltage logic state.
     * @var LPUART_T::MODEMSTS
     * Offset: 0x14  LPUART Modem Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CTSDETF   |Detect nCTS State Change Flag
     * |        |          |This bit is set whenever nCTS input has change state, and it will generate Modem interrupt to CPU when MODEMIEN (LPUART_INTEN [3]) is set to 1.
     * |        |          |0 = nCTS input has not change state.
     * |        |          |1 = nCTS input has change state.
     * |        |          |Note: This bit can be cleared by writing '1' to it.
     * |[4]     |CTSSTS    |nCTS Pin Status (Read Only)
     * |        |          |This bit mirror from nCTS pin input of voltage logic status.
     * |        |          |0 = nCTS pin input is low level voltage logic state.
     * |        |          |1 = nCTS pin input is high level voltage logic state.
     * |        |          |Note: This bit echoes when LPUART controller peripheral clock is enabled, and nCTS multi-function port is selected.
     * |[8]     |CTSACTLV  |nCTS Pin Active Level
     * |        |          |This bit defines the active level state of nCTS pin input.
     * |        |          |0 = nCTS pin input is high level active.
     * |        |          |1 = nCTS pin input is low level active. (Default)
     * |        |          |Note: Before setting this bit, TXRXDIS (LPUART_FUNCSEL[3]) should be set then waited for TXRXACT (LPUART_FIFOSTS[31]) is cleared
     * |        |          |When the configuration is done, cleared TXRXDIS (LPUART_FUNCSEL[3]) to activate LPUART controller.
     * @var LPUART_T::FIFOSTS
     * Offset: 0x18  LPUART FIFO Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXOVIF    |RX Overflow Error Interrupt Flag
     * |        |          |This bit is set when RX FIFO overflow.
     * |        |          |If the number of bytes of received data is greater than RX_FIFO (LPUART_DAT) size 16 bytes, this bit will be set.
     * |        |          |0 = RX FIFO is not overflow.
     * |        |          |1 = RX FIFO is overflow.
     * |        |          |Note: This bit can be cleared by writing '1' to it.
     * |[1]     |ABRDIF    |Auto-baud Rate Detect Interrupt Flag
     * |        |          |This bit is set to logic '1' when auto-baud rate detect function is finished.
     * |        |          |0 = Auto-baud rate detect function is not finished.
     * |        |          |1 = Auto-baud rate detect function is finished.
     * |        |          |Note: This bit can be cleared by writing '1' to it.
     * |[2]     |ABRDTOIF  |Auto-baud Rate Detect Time-out Interrupt Flag
     * |        |          |This bit is set to logic '1' in Auto-baud Rate Detect mode when the baud rate counter is overflow.
     * |        |          |0 = Auto-baud rate counter is underflow.
     * |        |          |1 = Auto-baud rate counter is overflow.
     * |        |          |Note: This bit can be cleared by writing '1' to it.
     * |[3]     |ADDRDETF  |RS-485 Address Byte Detect Flag
     * |        |          |0 = Receiver detects a data that is not an address bit (bit 9 ='0').
     * |        |          |1 = Receiver detects a data that is an address bit (bit 9 ='1').
     * |        |          |Note 1: This field is used for RS-485 function mode and ADDRDEN (LPUART_ALTCTL[15]) is set to 1 to enable Address detection mode.
     * |        |          |Note 2: This bit can be cleared by writing '1' to it.
     * |[4]     |PEF       |Parity Error Flag
     * |        |          |This bit is set to logic 1 whenever the received character does not have a valid 'parity bit'.
     * |        |          |0 = No parity error is generated.
     * |        |          |1 = Parity error is generated.
     * |        |          |Note: This bit can be cleared by writing '1' to it.
     * |[5]     |FEF       |Framing Error Flag
     * |        |          |This bit is set to logic 1 whenever the received character does not have a valid 'stop bit'
     * |        |          |(that is, the stop bit following the last data bit or parity bit is detected as logic 0).
     * |        |          |0 = No framing error is generated.
     * |        |          |1 = Framing error is generated.
     * |        |          |Note: This bit can be cleared by writing '1' to it.
     * |[6]     |BIF       |Break Interrupt Flag
     * |        |          |This bit is set to logic 1 whenever the received data input (RX) is held in the 'spacing state' (logic 0)
     * |        |          |for longer than a full word transmission time (that is, the total time of start bit + data bits + parity + stop bits).
     * |        |          |0 = No Break interrupt is generated.
     * |        |          |1 = Break interrupt is generated.
     * |        |          |Note: This bit can be cleared by writing '1' to it.
     * |[13:8]  |RXPTR     |RX FIFO Pointer (Read Only)
     * |        |          |This field indicates the RX FIFO Buffer Pointer
     * |        |          |When LPUART receives one byte from external device, RXPTR increases one
     * |        |          |When one byte of RX FIFO is read by CPU, RXPTR decreases one.
     * |        |          |The Maximum value shown in RXPTR is 15
     * |        |          |When the using level of RX FIFO Buffer equal to 16, the RXFULL bit is set to 1 and RXPTR will show 0
     * |        |          |As one byte of RX FIFO is read by CPU, the RXFULL bit is cleared to 0 and RXPTR will show 15
     * |[14]    |RXEMPTY   |Receiver FIFO Empty (Read Only)
     * |        |          |This bit initiate RX FIFO empty or not.
     * |        |          |0 = RX FIFO is not empty.
     * |        |          |1 = RX FIFO is empty.
     * |        |          |Note: When the last byte of RX FIFO has been read by CPU, hardware sets this bit high
     * |        |          |It will be cleared when LPUART receives any new data.
     * |[15]    |RXFULL    |Receiver FIFO Full (Read Only)
     * |        |          |This bit initiates RX FIFO full or not.
     * |        |          |0 = RX FIFO is not full.
     * |        |          |1 = RX FIFO is full.
     * |        |          |Note: This bit is set when the number of usage in RX FIFO Buffer is equal to 16, otherwise it is cleared by hardware.
     * |[21:16] |TXPTR     |TX FIFO Pointer (Read Only)
     * |        |          |This field indicates the TX FIFO Buffer Pointer
     * |        |          |When CPU writes one byte into LPUART_DAT, TXPTR increases one
     * |        |          |When one byte of TX FIFO is transferred to Transmitter Shift Register, TXPTR decreases one.
     * |        |          |The Maximum value shown in TXPTR is 15
     * |        |          |When the using level of TX FIFO Buffer equal to 16, the TXFULL bit is set to 1 and TXPTR will show 0
     * |        |          |As one byte of TX FIFO is transferred to Transmitter Shift Register, the TXFULL bit is cleared to 0 and TXPTR will show 15
     * |[22]    |TXEMPTY   |Transmitter FIFO Empty (Read Only)
     * |        |          |This bit indicates TX FIFO empty or not.
     * |        |          |0 = TX FIFO is not empty.
     * |        |          |1 = TX FIFO is empty.
     * |        |          |Note: When the last byte of TX FIFO has been transferred to Transmitter Shift Register, hardware sets this bit high
     * |        |          |It will be cleared when writing data into LPUART_DAT (TX FIFO not empty).
     * |[23]    |TXFULL    |Transmitter FIFO Full (Read Only)
     * |        |          |This bit indicates TX FIFO full or not.
     * |        |          |0 = TX FIFO is not full.
     * |        |          |1 = TX FIFO is full.
     * |        |          |Note: This bit is set when the number of usage in TX FIFO Buffer is equal to 16, otherwise it is cleared by hardware.
     * |[24]    |TXOVIF    |TX Overflow Error Interrupt Flag
     * |        |          |If TX FIFO (LPUART_DAT) is full, an additional write to LPUART_DAT will cause this bit to logic 1.
     * |        |          |0 = TX FIFO is not overflow.
     * |        |          |1 = TX FIFO is overflow.
     * |        |          |Note: This bit can be cleared by writing '1' to it.
     * |[28]    |TXEMPTYF  |Transmitter Empty Flag (Read Only)
     * |        |          |This bit is set by hardware when TX FIFO (LPUART_DAT) is empty and the STOP bit of the last byte has been transmitted.
     * |        |          |0 = TX FIFO is not empty or the STOP bit of the last byte has been not transmitted.
     * |        |          |1 = TX FIFO is empty and the STOP bit of the last byte has been transmitted.
     * |        |          |Note: This bit is cleared automatically when TX FIFO is not empty or the last byte transmission has not completed.
     * |[29]    |RXIDLE    |RX Idle Status (Read Only)
     * |        |          |This bit is set by hardware when RX is idle.
     * |        |          |0 = RX is busy.
     * |        |          |1 = RX is idle. (Default)
     * |[31]    |TXRXACT   |TX and RX Active Status (Read Only)
     * |        |          |This bit indicates TX and RX are active or inactive.
     * |        |          |0 = TX and RX are inactive.
     * |        |          |1 = TX and RX are active. (Default)
     * |        |          |Note: When TXRXDIS (LPUART_FUNCSEL[3]) is set and both TX and RX are in idle state, this bit is cleared
     * |        |          |The LPUART controller cannot transmit or receive data at this moment
     * |        |          |Otherwise this bit is set.
     * @var LPUART_T::INTSTS
     * Offset: 0x1C  LPUART Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RDAIF     |Receive Data Available Interrupt Flag (Read Only)
     * |        |          |When the number of bytes in the RX FIFO equals the RFITL then the RDAIF(LPUART_INTSTS[0]) will be set
     * |        |          |If RDAIEN (LPUART_INTEN [0]) is enabled, the RDA interrupt will be generated.
     * |        |          |0 = No RDA interrupt flag is generated.
     * |        |          |1 = RDA interrupt flag is generated.
     * |        |          |Note: This bit is read only and will be cleared when the number of unread bytes of RX FIFO drops below the threshold level (RFITL(LPUART_FIFO[7:4]).
     * |[1]     |THREIF    |Transmit Holding Register Empty Interrupt Flag (Read Only)
     * |        |          |This bit is set when the last data of TX FIFO is transferred to Transmitter Shift Register
     * |        |          |If THREIEN (LPUART_INTEN[1]) is enabled, the THRE interrupt will be generated.
     * |        |          |0 = No THRE interrupt flag is generated.
     * |        |          |1 = THRE interrupt flag is generated.
     * |        |          |Note: This bit is read only and will be cleared when writing data into LPUART_DAT (TX FIFO is not empty).
     * |[2]     |RLSIF     |Receive Line Interrupt Flag (Read Only)
     * |        |          |This bit is set when the RX receive data have parity error, frame error or break error (at least one of 3 bits, BIF(LPUART_FIFOSTS[6]), FEF(LPUART_FIFOSTS[5]) and PEF(LPUART_FIFOSTS[4]), is set)
     * |        |          |If RLSIEN (LPUART_INTEN [2]) is enabled, the RLS interrupt will be generated.
     * |        |          |0 = No RLS interrupt flag is generated.
     * |        |          |1 = RLS interrupt flag is generated.
     * |        |          |Note 1: In RS-485 function mode, this field is set include "receiver detect and received address byte character (bit9 = '1') bit"
     * |        |          |At the same time, the bit of ADDRDETF (LPUART_FIFOSTS[3]) is also set.
     * |        |          |Note 2: This bit is read only and reset to 0 when all bits of BIF (LPUART_FIFOSTS[6]), FEF(LPUART_FIFOSTS[5]) and PEF(LPUART_FIFOSTS[4]) are cleared.
     * |        |          |Note 3: In RS-485 function mode, this bit is read only and reset to 0 when all bits of BIF (LPUART_FIFOSTS[6]), FEF(LPUART_FIFOSTS[5]), PEF(LPUART_FIFOSTS[4]) and ADDRDETF (LPUART_FIFOSTS[3]) are cleared.
     * |[3]     |MODEMIF   |MODEM Interrupt Flag (Read Only)
     * |        |          |This bit is set when the nCTS pin has state change (CTSDETF (LPUART_MODEMSTS[0]) = 1)
     * |        |          |If MODEMIEN (LPUART_INTEN [3]) is enabled, the Modem interrupt will be generated.
     * |        |          |0 = No Modem interrupt flag is generated.
     * |        |          |1 = Modem interrupt flag is generated.
     * |        |          |Note: This bit is read only and reset to 0 when bit CTSDETF is cleared by a write 1 on CTSDETF(LPUART_MODEMSTS[0]).
     * |[4]     |RXTOIF    |RX Time-out Interrupt Flag
     * |        |          |This bit is set when the RX FIFO is not empty and no activities occurred in the RX FIFO and the time-out counter equal to TOIC (LPUART_TOUT[7:0])
     * |        |          |If RXTOIEN (LPUART_INTEN [4]) is enabled, the RX time-out interrupt will be generated.
     * |        |          |0 = No RX time-out interrupt flag is generated.
     * |        |          |1 = RX time-out interrupt flag is generated.
     * |        |          |Note: This bit can be cleared by writing '1' to it
     * |        |          |When BITOMEN (LPUART_TOUT[31]) is reset, this bit can also be cleared by reading LPUART_DAT.
     * |[5]     |BUFERRIF  |Buffer Error Interrupt Flag (Read Only)
     * |        |          |This bit is set when the TX FIFO or RX FIFO overflows (TXOVIF (LPUART_FIFOSTS[24]) or RXOVIF (LPUART_FIFOSTS[0]) is set)
     * |        |          |When BUFERRIF (LPUART_INTSTS[5]) is set, the transfer is not correct
     * |        |          |If BUFERRIEN (LPUART_INTEN [5]) is enabled, the buffer error interrupt will be generated.
     * |        |          |0 = No buffer error interrupt flag is generated.
     * |        |          |1 = Buffer error interrupt flag is generated.
     * |        |          |Note: This bit is cleared if both of RXOVIF(LPUART_FIFOSTS[0]) and TXOVIF(LPUART_FIFOSTS[24]) are cleared to 0 by writing 1 to RXOVIF(LPUART_FIFOSTS[0]) and TXOVIF(LPUART_FIFOSTS[24]).
     * |[6]     |WKIF      |LPUART Wake-up Interrupt Flag (Read Only)
     * |        |          |This bit is set when TOUTWKF (LPUART_WKSTS[4]), RS485WKF (LPUART_WKSTS[3]), RFRTWKF (LPUART_WKSTS[2]), DATWKF (LPUART_WKSTS[1]) or CTSWKF(LPUART_WKSTS[0]) is set to 1.
     * |        |          |0 = No LPUART wake-up interrupt flag is generated.
     * |        |          |1 = LPUART wake-up interrupt flag is generated.
     * |        |          |Note: This bit is cleared if all of TOUTWKF, RS485WKF, RFRTWKF, DATWKF and CTSWKF are cleared to 0 by writing 1 to the corresponding interrupt flag.
     * |[8]     |RDAINT    |Receive Data Available Interrupt Indicator (Read Only)
     * |        |          |This bit is set if RDAIEN (LPUART_INTEN[0]) and RDAIF (LPUART_INTSTS[0]) are both set to 1.
     * |        |          |0 = No RDA interrupt is generated.
     * |        |          |1 = RDA interrupt is generated.
     * |[9]     |THREINT   |Transmit Holding Register Empty Interrupt Indicator (Read Only)
     * |        |          |This bit is set if THREIEN (LPUART_INTEN[1]) and THREIF(LPUART_INTSTS[1]) are both set to 1.
     * |        |          |0 = No THRE interrupt is generated.
     * |        |          |1 = THRE interrupt is generated.
     * |[10]    |RLSINT    |Receive Line Status Interrupt Indicator (Read Only)
     * |        |          |This bit is set if RLSIEN (LPUART_INTEN[2]) and RLSIF(LPUART_INTSTS[2]) are both set to 1.
     * |        |          |0 = No RLS interrupt is generated.
     * |        |          |1 = RLS interrupt is generated.
     * |[11]    |MODEMINT  |MODEM Status Interrupt Indicator (Read Only)
     * |        |          |This bit is set if MODEMIEN(LPUART_INTEN[3]) and MODEMIF(LPUART_INTSTS[3]) are both set to 1
     * |        |          |0 = No Modem interrupt is generated.
     * |        |          |1 = Modem interrupt is generated.
     * |[12]    |RXTOINT   |RX Time-out Interrupt Indicator (Read Only)
     * |        |          |This bit is set if RXTOIEN (LPUART_INTEN[4]) and RXTOIF(LPUART_INTSTS[4]) are both set to 1.
     * |        |          |0 = No RX time-out interrupt is generated.
     * |        |          |1 = RX time-out interrupt is generated.
     * |[13]    |BUFERRINT |Buffer Error Interrupt Indicator (Read Only)
     * |        |          |This bit is set if BUFERRIEN(LPUART_INTEN[5]) and BUFERRIF(LPUART_ INTSTS[5]) are both set to 1.
     * |        |          |0 = No buffer error interrupt is generated.
     * |        |          |1 = Buffer error interrupt is generated.
     * |[14]    |WKINT     |LPUART Wake-up Interrupt Indicator (Read Only)
     * |        |          |This bit is set if WKIEN (LPUART_INTEN[6]) and WKIF (LPUART_INTSTS[6]) are both set to 1.
     * |        |          |0 = No LPUART wake-up interrupt is generated.
     * |        |          |1 = LPUART wake-up interrupt is generated.
     * |[18]    |HWRLSIF   |LPPDMA Mode Receive Line Status Flag (Read Only)
     * |        |          |This bit is set when the RX receive data have parity error, frame error or break error (at least one of 3 bits, BIF (LPUART_FIFOSTS[6]), FEF (LPUART_FIFOSTS[5]) and PEF (LPUART_FIFOSTS[4]) is set)
     * |        |          |If RLSIEN (LPUART_INTEN [2]) is enabled, the RLS interrupt will be generated.
     * |        |          |0 = No RLS interrupt flag is generated in LPPDMA mode.
     * |        |          |1 = RLS interrupt flag is generated in LPPDMA mode.
     * |        |          |Note 1: In RS-485 function mode, this field include 'receiver detect any address byte received address byte character (bit9 = '1') bit".
     * |        |          |Note 2: In LPUART function mode, this bit is read only and reset to 0 when all bits of BIF(LPUART_FIFOSTS[6]), FEF(LPUART_FIFOSTS[5]) and PEF(LPUART_FIFOSTS[4]) are cleared.
     * |        |          |Note 3: In RS-485 function mode, this bit is read only and reset to 0 when all bits of BIF(LPUART_FIFOSTS[6]), FEF(LPUART_FIFOSTS[5]), PEF(LPUART_FIFOSTS[4]) and ADDRDETF (LPUART_FIFOSTS[3]) are cleared.
     * |[19]    |HWMODIF   |LPPDMA Mode MODEM Interrupt Flag (Read Only)
     * |        |          |This bit is set when the nCTS pin has state change (CTSDETF (LPUART_MODEMSTS [0] =1))
     * |        |          |If MODEMIEN (LPUART_INTEN [3]) is enabled, the Modem interrupt will be generated.
     * |        |          |0 = No Modem interrupt flag is generated in LPPDMA mode.
     * |        |          |1 = Modem interrupt flag is generated in LPPDMA mode.
     * |        |          |Note: This bit is read only and reset to 0 when the bit CTSDETF (LPUART_MODEMSTS[0]) is cleared by writing 1 on CTSDETF (LPUART_MODEMSTS [0]).
     * |[20]    |HWTOIF    |LPPDMA Mode RX Time-out Interrupt Flag
     * |        |          |This bit is set when the RX FIFO is not empty and no activities occurred in the RX FIFO and the time-out counter equal to TOIC (LPUART_TOUT[7:0])
     * |        |          |If RXTOIEN (LPUART_INTEN [4]) is enabled, the RX time-out interrupt will be generated.
     * |        |          |0 = No RX time-out interrupt flag is generated in LPPDMA mode.
     * |        |          |1 = RX time-out interrupt flag is generated in LPPDMA mode.
     * |        |          |Note: This bit can be cleared by writing '1' to it
     * |        |          |When BITOMEN (LPUART_TOUT[31]) is reset, this bit can also be cleared by reading LPUART_DAT.
     * |[21]    |HWBUFEIF  |LPPDMA Mode Buffer Error Interrupt Flag (Read Only)
     * |        |          |This bit is set when the TX or RX FIFO overflows (TXOVIF (LPUART_FIFOSTS [24]) or RXOVIF (LPUART_FIFOSTS[0]) is set)
     * |        |          |When BUFERRIF (LPUART_INTSTS[5]) is set, the transfer maybe is not correct
     * |        |          |If BUFERRIEN (LPUART_INTEN [5]) is enabled, the buffer error interrupt will be generated.
     * |        |          |0 = No buffer error interrupt flag is generated in LPPDMA mode.
     * |        |          |1 = Buffer error interrupt flag is generated in LPPDMA mode.
     * |        |          |Note: This bit is cleared when both TXOVIF (LPUART_FIFOSTS[24]]) and RXOVIF (LPUART_FIFOSTS[0]) are cleared.
     * |[22]    |TXENDIF   |Transmitter Empty Interrupt Flag
     * |        |          |This bit is set when TX FIFO (LPUART_DAT) is empty and the STOP bit of the last byte has been transmitted (TXEMPTYF (LPUART_FIFOSTS[28]) is set)
     * |        |          |If TXENDIEN (LPUART_INTEN[22]) is enabled, the Transmitter Empty interrupt will be generated.
     * |        |          |0 = No transmitter empty interrupt flag is generated.
     * |        |          |1 = Transmitter empty interrupt flag is generated.
     * |        |          |Note: This bit is cleared automatically when TX FIFO is not empty or the last byte transmission has not completed.
     * |[26]    |HWRLSINT  |LPPDMA Mode Receive Line Status Interrupt Indicator (Read Only)
     * |        |          |This bit is set if RLSIEN (LPUART_INTEN[2]) and HWRLSIF(LPUART_INTSTS[18]) are both set to 1.
     * |        |          |0 = No RLS interrupt is generated in LPPDMA mode.
     * |        |          |1 = RLS interrupt is generated in LPPDMA mode.
     * |[27]    |HWMODINT  |LPPDMA Mode MODEM Status Interrupt Indicator (Read Only)
     * |        |          |This bit is set if MODEMIEN (LPUART_INTEN[3]) and HWMODIF(LPUART_INTSTS[19]) are both set to 1.
     * |        |          |0 = No Modem interrupt is generated in LPPDMA mode.
     * |        |          |1 = Modem interrupt is generated in LPPDMA mode.
     * |[28]    |HWTOINT   |LPPDMA Mode RX Time-out Interrupt Indicator (Read Only)
     * |        |          |This bit is set if RXTOIEN (LPUART_INTEN[4]) and HWTOIF(LPUART_INTSTS[20]) are both set to 1.
     * |        |          |0 = No RX time-out interrupt is generated in LPPDMA mode.
     * |        |          |1 = RX time-out interrupt is generated in LPPDMA mode.
     * |[29]    |HWBUFEINT |LPPDMA Mode Buffer Error Interrupt Indicator (Read Only)
     * |        |          |This bit is set if BUFERRIEN (LPUART_INTEN[5]) and HWBUFEIF (LPUART_INTSTS[21]) are both set to 1.
     * |        |          |0 = No buffer error interrupt is generated in LPPDMA mode.
     * |        |          |1 = Buffer error interrupt is generated in LPPDMA mode.
     * |[30]    |TXENDINT  |Transmitter Empty Interrupt Indicator (Read Only)
     * |        |          |This bit is set if TXENDIEN (LPUART_INTEN[22]) and TXENDIF(LPUART_INTSTS[22]) are both set to 1.
     * |        |          |0 = No Transmitter Empty interrupt is generated.
     * |        |          |1 = Transmitter Empty interrupt is generated.
     * |[31]    |ABRINT    |Auto-baud Rate Interrupt Indicator (Read Only)
     * |        |          |This bit is set if ABRIEN (LPUART_INTEN[18]) and ABRIF (LPUART_ALTCTL[17]) are both set to 1.
     * |        |          |0 = No Auto-baud Rate interrupt is generated.
     * |        |          |1 = The Auto-baud Rate interrupt is generated.
     * @var LPUART_T::TOUT
     * Offset: 0x20  LPUART Time-out Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |TOIC      |Time-out Interrupt Comparator
     * |        |          |The time-out counter resets and starts counting (the counting clock = baud rate) whenever the RX FIFO receives a new data word if time out counter is enabled by setting TOCNTEN (LPUART_INTEN[11])
     * |        |          |Once the content of time-out counter is equal to that of time-out interrupt comparator (TOIC (LPUART_TOUT[7:0])), a receiver time-out interrupt (RXTOINT(LPUART_INTSTS[12])) is generated if RXTOIEN (LPUART_INTEN [4]) enabled
     * |        |          |A new incoming data word or RX FIFO empty will clear RXTOIF (LPUART_INTSTS[4])
     * |        |          |In order to avoid receiver time-out interrupt generation immediately during one character is being received, TOIC value should be set between 40 and 255
     * |        |          |So, for example, if TOIC is set with 40, the time-out interrupt is generated after four characters are not received when 1 stop bit and no parity check is set for LPUART transfer.
     * |[15:8]  |DLY       |TX Delay Time Value
     * |        |          |This field is used to program the transfer delay time between the last stop bit and next start bit
     * |        |          |The unit is bit time.
     * |[31]    |BITOMEN   |Bus Idle Time-out Mode Enable Bit
     * |        |          |If BITOMEN (LPUART_TOUT[31]) is enabled, the reset conditions of the time-out counter and RXTOIF (LPUART_INTSTS[4]) will be changed to detect the bus idle.
     * |        |          |When BITOMEN (LPUART_TOUT[31]) is disabled, the time-out counter and RXTOIF (LPUART_INTSTS[4]) maintain reset value whenever the RX FIFO is empty.
     * |        |          |In addition, a new incoming data word will also clear RXTOIF (LPUART_INTSTS[4]).
     * |        |          |On the other hand, when BITOMEN (LPUART_TOUT[31]) is enabled, the RX FIFO empty state will not reset the time-out counter and RXTOIF (LPUART_INTSTS[4]), and the new incoming data word event will not clear RXTOIF (LPUART_INTSTS[4]).
     * |        |          |0 = Bus idle time-out mode Disabled.
     * |        |          |1 = Bus idle time-out mode Enabled.
     * @var LPUART_T::BAUD
     * Offset: 0x24  LPUART Baud Rate Divider Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |BRD       |Baud Rate Divider
     * |        |          |The field indicates the baud rate divider
     * |        |          |This filed is used in baud rate calculation
     * |        |          |The detail description is shown in Table 6.27-4.
     * |[27:24] |EDIVM1    |Extra Divider for BAUD Rate Mode 1
     * |        |          |This field is used for baud rate calculation in mode 1 and has no effect for baud rate calculation in mode 0 and mode 2
     * |        |          |The detail description is shown in Table 6.27-4
     * |[28]    |BAUDM0    |BAUD Rate Mode Selection Bit 0
     * |        |          |This bit is baud rate mode selection bit 0
     * |        |          |LPUART provides three baud rate calculation modes
     * |        |          |This bit combines with BAUDM1 (LPUART_BAUD[29]) to select baud rate calculation mode
     * |        |          |The detail description is shown in Table 6.27-4.
     * |[29]    |BAUDM1    |BAUD Rate Mode Selection Bit 1
     * |        |          |This bit is baud rate mode selection bit 1
     * |        |          |LPUART provides three baud rate calculation modes
     * |        |          |This bit combines with BAUDM0 (LPUART_BAUD[28]) to select baud rate calculation mode
     * |        |          |The detail description is shown in Table 6.27-4.
          * @var LPUART_T::ALTCTL
     * Offset: 0x2C  LPUART Alternate Control/Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |        |          |Note: When TX break field transfer operation finished, this bit will be cleared automatically.
     * |[8]     |RS485NMM  |RS-485 Normal Multi-drop Operation Mode (NMM)
     * |        |          |0 = RS-485 Normal Multi-drop Operation mode (NMM) Disabled.
     * |        |          |1 = RS-485 Normal Multi-drop Operation mode (NMM) Enabled.
     * |        |          |Note: It cannot be active with RS-485_AAD operation mode.
     * |[9]     |RS485AAD  |RS-485 Auto Address Detection Operation Mode (AAD)
     * |        |          |0 = RS-485 Auto Address Detection Operation mode (AAD) Disabled.
     * |        |          |1 = RS-485 Auto Address Detection Operation mode (AAD) Enabled.
     * |        |          |Note: It cannot be active with RS-485_NMM operation mode.
     * |[10]    |RS485AUD  |RS-485 Auto Direction Function (AUD)
     * |        |          |0 = RS-485 Auto Direction Operation function (AUD) Disabled.
     * |        |          |1 = RS-485 Auto Direction Operation function (AUD) Enabled.
     * |        |          |Note: It can be active with RS-485_AAD or RS-485_NMM operation mode.
     * |[15]    |ADDRDEN   |RS-485 Address Detection Enable Bit
     * |        |          |This bit is used to enable RS-485 Address Detection mode.
     * |        |          |0 = Address detection mode Disabled.
     * |        |          |1 = Address detection mode Enabled.
     * |        |          |Note: This bit is used for RS-485 any operation mode.
     * |[17]    |ABRIF     |Auto-baud Rate Interrupt Flag (Read Only)
     * |        |          |This bit is set when auto-baud rate detection function finished or the auto-baud rate counter was overflow and if ABRIEN(LPUART_INTEN [18]) is set then the auto-baud rate interrupt will be generated.
     * |        |          |0 = No auto-baud rate interrupt flag is generated.
     * |        |          |1 = Auto-baud rate interrupt flag is generated.
     * |        |          |Note: This bit is read only, but it can be cleared by writing '1' to ABRDTOIF (LPUART_FIFOSTS[2]) and ABRDIF(LPUART_FIFOSTS[1])
     * |[18]    |ABRDEN    |Auto-baud Rate Detect Enable Bit
     * |        |          |0 = Auto-baud rate detect function Disabled.
     * |        |          |1 = Auto-baud rate detect function Enabled.
     * |        |          |Note : This bit is cleared automatically after auto-baud detection is finished.
     * |[20:19] |ABRDBITS  |Auto-baud Rate Detect Bit Length
     * |        |          |00 = 1-bit time from START bit to the 1st rising edge. The input pattern shall be 0x01.
     * |        |          |01 = 2-bit time from START bit to the 1st rising edge. The input pattern shall be 0x02.
     * |        |          |10 = 4-bit time from START bit to the 1st rising edge. The input pattern shall be 0x08.
     * |        |          |11 = 8-bit time from START bit to the 1st rising edge. The input pattern shall be 0x80.
     * |        |          |Note : The calculation of bit number includes the START bit.
     * |[31:24] |ADDRMV    |Address Match Value
     * |        |          |This field contains the RS-485 address match values.
     * |        |          |Note: This field is used for RS-485 auto address detection mode.
     * @var LPUART_T::FUNCSEL
     * Offset: 0x30  LPUART Function Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |FUNCSEL   |Function Select
     * |        |          |000 = LPUART function.
     * |        |          |011 = RS-485 function.
     * |        |          |Others = Reserved.
     * |[3]     |TXRXDIS   |TX and RX Disable Bit
     * |        |          |Setting this bit can disable TX and RX.
     * |        |          |0 = TX and RX Enabled.
     * |        |          |1 = TX and RX Disabled.
     * |        |          |Note: The TX and RX will not be disabled immediately when this bit is set
     * |        |          |The TX and RX complete current task before TX and RX are disabled
     * |        |          |When TX and RX are disabled, the TXRXACT (LPUART_FIFOSTS[31]) is cleared.
     * |[6]     |DGE       |Deglitch Enable Bit
     * |        |          |0 = Deglitch Disabled.
     * |        |          |1 = Deglitch Enabled.
     * |        |          |Note 1: When this bit is set to logic 1, any pulse width less than about 150 ns will be considered a glitch and will be removed in the serial data input (RX).
     * |        |          |This bit acts only on RX line and has no effect on the transmitter logic.
     * |        |          |Note 2: It is recommended to set this bit only when operating at baud rate under 2.5 Mbps.
     * |[7]     |TXRXSWP   |TX and RX Swap Enable Bit
     * |        |          |Setting this bit Swaps TX pin and RX pin.
     * |        |          |0 = TX and RX Swap Disabled.
     * |        |          |1 = TX and RX Swap Enabled.
     * @var LPUART_T::BRCOMP
     * Offset: 0x3C  LPUART Baud Rate Compensation Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8:0]   |BRCOMP    |Baud Rate Compensation Patten
     * |        |          |These 9-bits are used to define the relative bit is compensated or not.
     * |        |          |BRCOMP[7:0] is used to define the compensation of DAT (LPUART_DAT[7:0]) and BRCOM[8] is used to define PARITY (LPUART_DAT[8]).
     * |[31]    |BRCOMPDEC |Baud Rate Compensation Decrease
     * |        |          |0 = Positive (increase one module clock) compensation for each compensated bit.
     * |        |          |1 = Negative (decrease one module clock) compensation for each compensated bit.
     * @var LPUART_T::WKCTL
     * Offset: 0x40  LPUART Wake-up Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKCTSEN   |nCTS Wake-up Enable Bit
     * |        |          |0 = nCTS Wake-up system function Disabled.
     * |        |          |1 = nCTS Wake-up system function Enabled.
     * |        |          |Note: When the system is in Power-down mode, an external nCTS change will wake up system from Power-down mode.
     * |[1]     |WKDATEN   |Incoming Data Wake-up Enable Bit
     * |        |          |0 = Incoming data wake-up system function Disabled.
     * |        |          |1 = Incoming data wake-up system function Enabled.
     * |        |          |Note: When the system is in Power-down mode, incoming data will wake-up system from Power-down mode.
     * |[2]     |WKRFRTEN  |Received Data FIFO Reached Threshold Wake-up Enable Bit
     * |        |          |0 = Received Data FIFO reached threshold wake-up system function Disabled.
     * |        |          |1 = Received Data FIFO reached threshold wake-up system function Enabled.
     * |        |          |Note: When the system is in Power-down mode, Received Data FIFO reached threshold will wake-up system from Power-down mode.
     * |[3]     |WKRS485EN |RS-485 Address Match (AAD Mode) Wake-up Enable Bit
     * |        |          |0 = RS-485 Address Match (AAD mode) wake-up system function Disabled.
     * |        |          |1 = RS-485 Address Match (AAD mode) wake-up system function Enabled.
     * |        |          |Note 1: When the system is in Power-down mode, RS-485 Address Match will wake-up system from Power-down mode.
     * |        |          |Note 2: This bit is used for RS-485 Auto Address Detection (AAD) mode in RS-485 function mode and ADDRDEN (LPUART_ALTCTL[15]) is set to 1.
     * |[4]     |WKTOUTEN  |Received Data FIFO Reached Threshold Time-out Wake-up Enable Bit
     * |        |          |0 = Received Data FIFO reached threshold time-out wake-up system function Disabled.
     * |        |          |1 = Received Data FIFO reached threshold time-out wake-up system function Enabled.
     * |        |          |Note 1: When the system is in Power-down mode, Received Data FIFO reached threshold time-out will wake up system from Power-down mode.
     * |        |          |Note 2: It is suggested the function is enabled when the WKRFRTEN (LPUART_WKCTL[2]) is set to 1.
     * @var LPUART_T::WKSTS
     * Offset: 0x44  LPUART Wake-up Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CTSWKF    |nCTS Wake-up Flag
     * |        |          |This bit is set if chip wake-up from power-down state by nCTS wake-up.
     * |        |          |0 = Chip stays in power-down state.
     * |        |          |1 = Chip wake-up from power-down state by nCTS wake-up.
     * |        |          |Note 1: If WKCTSEN (LPUART_WKCTL[0]) is enabled, the nCTS wake-up cause this bit set to '1'.
     * |        |          |Note 2: This bit can be cleared by writing '1' to it.
     * |[1]     |DATWKF    |Incoming Data Wake-up Flag
     * |        |          |This bit is set if chip wake-up from power-down state by data wake-up.
     * |        |          |0 = Chip stays in power-down state.
     * |        |          |1 = Chip wake-up from power-down state by Incoming Data wake-up.
     * |        |          |Note 1: If WKDATEN (LPUART_WKCTL[1]) is enabled, the Incoming Data wake-up cause this bit set to '1'.
     * |        |          |Note 2: This bit can be cleared by writing '1' to it.
     * |[2]     |RFRTWKF   |Received Data FIFO Reached Threshold Wake-up Flag
     * |        |          |This bit is set if chip wake-up from power-down state by Received Data FIFO reached threshold wake-up.
     * |        |          |0 = Chip stays in power-down state.
     * |        |          |1 = Chip wake-up from power-down state by Received Data FIFO Reached Threshold wake-up.
     * |        |          |Note 1: If WKRFRTEN (LPUART_WKCTL[2]) is enabled, the Received Data FIFO Reached Threshold wake-up cause this bit set to '1'.
     * |        |          |Note 2: This bit can be cleared by writing '1' to it.
     * |[3]     |RS485WKF  |RS-485 Address Match (AAD Mode) Wake-up Flag
     * |        |          |This bit is set if chip wake-up from power-down state by RS-485 Address Match (AAD mode).
     * |        |          |0 = Chip stays in power-down state.
     * |        |          |1 = Chip wake-up from power-down state by RS-485 Address Match (AAD mode) wake-up.
     * |        |          |Note 1: If WKRS485EN (LPUART_WKCTL[3]) is enabled, the RS-485 Address Match (AAD mode) wake-up cause this bit set to '1'.
     * |        |          |Note 2: This bit can be cleared by writing '1' to it.
     * |[4]     |TOUTWKF   |Received Data FIFO Threshold Time-out Wake-up Flag
     * |        |          |This bit is set if chip wake-up from power-down state by Received Data FIFO Threshold Time-out wake-up.
     * |        |          |0 = Chip stays in power-down state.
     * |        |          |1 = Chip wake-up from power-down state by Received Data FIFO reached threshold time-out.
     * |        |          |Note 1: If WKTOUTEN (LPUART_WKCTL[4]) is enabled, the Received Data FIFO reached threshold time-out wake-up cause this bit set to '1'.
     * |        |          |Note 2: This bit can be cleared by writing '1' to it.
     * @var LPUART_T::DWKCOMP
     * Offset: 0x48  LPUART Incoming Data Wake-up Compensation Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |STCOMP    |START Bit Compensation Value
     * |        |          |These bits field indicate how many clock cycle selected by LPUART_CLK do the LPUART controller can get the 1st bit (START bit) when the device is wake-up from power-down mode.
     * |        |          |Note: It is valid only when WKDATEN (LPUART_WKCTL[1]) is set.
     * @var LPUART_T::RS485DD
     * Offset: 0x4C  LPUART RS485 Transceiver Deactivate Delay Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RTSDDLY   |RS485 Transceiver Deactivate Delay Value
     * |        |          |These bits field indicate how many clock cycles selected by LPUART_CLK do the LPUART controller delay the RS485 transceiver state trancing when the state trancing of RS485 transceiver is from TX to RX state.
     * |        |          |These bits field have no effect when the state trancing of RS485 transceiver is from RX to TX state.
     * |        |          |Note: It is valid only when RS485AUD (LPUART_ALTCTL[10]) is set.
     * @var LPUART_T::AUTOCTL
     * Offset: 0x58  LPUART Automatic Operation Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |TRIGSEL   |Automatic Operation Trigger Source Select
     * |        |          |0000 = Low Power Auto-operation Trigger Source from LPTMR0.
     * |        |          |0001 = Low Power Auto-operation Trigger Source from LPTMR1.
     * |        |          |0010 = Low Power Auto-operation Trigger Source from TTMR0.
     * |        |          |0011 = Low Power Auto-operation Trigger Source from TTMR1.
     * |        |          |0100 = Low Power Auto-operation Trigger Source from WKIOA0.
     * |        |          |0101 = Low Power Auto-operation Trigger Source from WKIOB0.
     * |        |          |0110 = Low Power Auto-operation Trigger Source from WKIOC0.
     * |        |          |0111 = Low Power Auto-operation Trigger Source from WKIOD0.
     * |        |          |1000~1111 = Reserved.
     * |[4]     |TRIGEN    |Automatic Operation Trigger Enable Bit
     * |        |          |When TRIGEN (LPUART_AUTOCTL[4]) is set to '1', the automatic TX operation of LPUART will be triggered by an event sent from the trigger source selected by TRIGSEL[3:0].
     * |        |          |0 = LPUART Automatic Operation Trigger disabled.
     * |        |          |1 = LPUART Automatic Operation Trigger enabled.
     * |[5]     |CKAWOEN   |Automatic Operation Clock Always-on Enable Bit
     * |        |          |0 = Automatic Operation Clock Always-on Disabled.
     * |        |          |1 = Automatic Operation Clock Always-on Enabled.
     * |[8]     |WKAOTOEN  |Bus Idle Time-out Wake-up Enable Bit
     * |        |          |0 = Bus Idle Time-out Wake-up system function Disabled.
     * |        |          |1 = Bus Idle Time-out Wake-up system function Enabled.
     * |        |          |Note 1: When the system is in Power-down mode, Bus Idle Time-out event will wake-up system from Power-down mode if WKAOTOEN (LPUART_AUTOCTL[8]), AOEN (LPUART_AUTOCTL[31]), and TOCNTEN (LPUART_INTEN[11]) are all set to '1'.
     * |        |          |Note 2: When WKAOTOEN (LPUART_AUTOCTL[8]) is reset to '0', Bus Idle Time-out Wake-up event will not wake up the system
     * |        |          |Moreover, Bus Idle Time-out Wake-up event will gate the UART clock off if WKAOTOEN (LPUART_AUTOCTL[8]) and TRIGEN (LPUART_AUTOCTL[4]) are both reset to '0'.
     * |[9]     |SWTRIG    |Software Trigger (Write Only)
     * |        |          |After AOEN is set to 1, software can manually trigger the automatic TX operation by writing 1 to this bit.
     * |[31]    |AOEN      |Automatic Operation Enable Bit
     * |        |          |0 = Automatic Operation Disabled.
     * |        |          |1 = Automatic Operation Enabled.
     * @var LPUART_T::AUTOSTS
     * Offset: 0x5C  LPUART Automatic Operation Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |AOTOWKF   |Bus Idle Time-out Wake-up Flag
     * |        |          |This bit is set if chip wake-up from power-down state by Bus Idle Time-out Wake-up
     * |        |          |0 = There is no Bus Idle Time-out Wake-up event.
     * |        |          |1 = Chip wake-up from power-down state by Bus Idle Time-out Wake-up.
     * |        |          |Note 1: When RXPDMAEN (LPUART_INTEN[15]) or TXPDMAEN (LPUART_INTEN[14]) is enabled, this bit is the shadow bit of HWTOIF (LPUART_INTSTS[20]);
     * |        |          |user can read/clear it by reading/resetting HWTOIF (LPUART_INTSTS[20]) or AOTOWKF (LPUART_AUTOSTS[0]).
     * |        |          |Note 2: When RXPDMAEN (LPUART_INTEN[15]) and TXPDMAEN (LPUART_INTEN[14]) are both disabled, this bit is the shadow bit of RXTOIF (LPUART_INTSTS[4]);
     * |        |          |user can read/clear it by reading/resetting RXTOIF (LPUART_INTSTS[4]) or AOTOWKF (LPUART_AUTOSTS[0]).
     * |        |          |Note 3: If WKAOTOEN (LPUART_AUTOCTL[8]) is enabled, the Bus Idle Time-out Wake-up event will set this bit.
     * |        |          |Note 4: This bit can be cleared by writing '1' to it.
     */
    __IO uint32_t DAT;                   /*!< [0x0000] LPUART Receive/Transmit Buffer Register                          */
    __IO uint32_t INTEN;                 /*!< [0x0004] LPUART Interrupt Enable Register                                 */
    __IO uint32_t FIFO;                  /*!< [0x0008] LPUART FIFO Control Register                                     */
    __IO uint32_t LINE;                  /*!< [0x000c] LPUART Line Control Register                                     */
    __IO uint32_t MODEM;                 /*!< [0x0010] LPUART Modem Control Register                                    */
    __IO uint32_t MODEMSTS;              /*!< [0x0014] LPUART Modem Status Register                                     */
    __IO uint32_t FIFOSTS;               /*!< [0x0018] LPUART FIFO Status Register                                      */
    __IO uint32_t INTSTS;                /*!< [0x001c] LPUART Interrupt Status Register                                 */
    __IO uint32_t TOUT;                  /*!< [0x0020] LPUART Time-out Register                                         */
    __IO uint32_t BAUD;                  /*!< [0x0024] LPUART Baud Rate Divider Register                                */
    __IO uint32_t RESERVE0[1];
    __IO uint32_t ALTCTL;                /*!< [0x002c] LPUART Alternate Control/Status Register                         */
    __IO uint32_t FUNCSEL;               /*!< [0x0030] LPUART Function Select Register                                  */
    __IO uint32_t RESERVE1[2];
    __IO uint32_t BRCOMP;                /*!< [0x003c] LPUART Baud Rate Compensation Register                           */
    __IO uint32_t WKCTL;                 /*!< [0x0040] LPUART Wake-up Control Register                                  */
    __IO uint32_t WKSTS;                 /*!< [0x0044] LPUART Wake-up Status Register                                   */
    __IO uint32_t DWKCOMP;               /*!< [0x0048] LPUART Incoming Data Wake-up Compensation Register               */
    __IO uint32_t RS485DD;               /*!< [0x004c] LPUART RS485 Transceiver Deactivate Delay Register               */
    __I  uint32_t RESERVE2[2];
    __IO uint32_t AUTOCTL;               /*!< [0x0058] LPUART Automatic Operation Control Register                      */
    __IO uint32_t AUTOSTS;               /*!< [0x005c] LPUART Automatic Operation Status Register                       */

} LPUART_T;

/**
    @addtogroup LPUART_CONST LPUART Bit Field Definition
    Constant Definitions for LPUART Controller
@{ */

#define LPUART_DAT_DAT_Pos               (0)                                               /*!< LPUART_T::DAT: DAT Position            */
#define LPUART_DAT_DAT_Msk               (0xfful << LPUART_DAT_DAT_Pos)                    /*!< LPUART_T::DAT: DAT Mask                */

#define LPUART_DAT_PARITY_Pos            (8)                                               /*!< LPUART_T::DAT: PARITY Position         */
#define LPUART_DAT_PARITY_Msk            (0x1ul << LPUART_DAT_PARITY_Pos)                  /*!< LPUART_T::DAT: PARITY Mask             */

#define LPUART_INTEN_RDAIEN_Pos          (0)                                               /*!< LPUART_T::INTEN: RDAIEN Position       */
#define LPUART_INTEN_RDAIEN_Msk          (0x1ul << LPUART_INTEN_RDAIEN_Pos)                /*!< LPUART_T::INTEN: RDAIEN Mask           */

#define LPUART_INTEN_THREIEN_Pos         (1)                                               /*!< LPUART_T::INTEN: THREIEN Position      */
#define LPUART_INTEN_THREIEN_Msk         (0x1ul << LPUART_INTEN_THREIEN_Pos)               /*!< LPUART_T::INTEN: THREIEN Mask          */

#define LPUART_INTEN_RLSIEN_Pos          (2)                                               /*!< LPUART_T::INTEN: RLSIEN Position       */
#define LPUART_INTEN_RLSIEN_Msk          (0x1ul << LPUART_INTEN_RLSIEN_Pos)                /*!< LPUART_T::INTEN: RLSIEN Mask           */

#define LPUART_INTEN_MODEMIEN_Pos        (3)                                               /*!< LPUART_T::INTEN: MODEMIEN Position     */
#define LPUART_INTEN_MODEMIEN_Msk        (0x1ul << LPUART_INTEN_MODEMIEN_Pos)              /*!< LPUART_T::INTEN: MODEMIEN Mask         */

#define LPUART_INTEN_RXTOIEN_Pos         (4)                                               /*!< LPUART_T::INTEN: RXTOIEN Position      */
#define LPUART_INTEN_RXTOIEN_Msk         (0x1ul << LPUART_INTEN_RXTOIEN_Pos)               /*!< LPUART_T::INTEN: RXTOIEN Mask          */

#define LPUART_INTEN_BUFERRIEN_Pos       (5)                                               /*!< LPUART_T::INTEN: BUFERRIEN Position    */
#define LPUART_INTEN_BUFERRIEN_Msk       (0x1ul << LPUART_INTEN_BUFERRIEN_Pos)             /*!< LPUART_T::INTEN: BUFERRIEN Mask        */

#define LPUART_INTEN_WKIEN_Pos           (6)                                               /*!< LPUART_T::INTEN: WKIEN Position        */
#define LPUART_INTEN_WKIEN_Msk           (0x1ul << LPUART_INTEN_WKIEN_Pos)                 /*!< LPUART_T::INTEN: WKIEN Mask            */

#define LPUART_INTEN_TOCNTEN_Pos         (11)                                              /*!< LPUART_T::INTEN: TOCNTEN Position      */
#define LPUART_INTEN_TOCNTEN_Msk         (0x1ul << LPUART_INTEN_TOCNTEN_Pos)               /*!< LPUART_T::INTEN: TOCNTEN Mask          */

#define LPUART_INTEN_ATORTSEN_Pos        (12)                                              /*!< LPUART_T::INTEN: ATORTSEN Position     */
#define LPUART_INTEN_ATORTSEN_Msk        (0x1ul << LPUART_INTEN_ATORTSEN_Pos)              /*!< LPUART_T::INTEN: ATORTSEN Mask         */

#define LPUART_INTEN_ATOCTSEN_Pos        (13)                                              /*!< LPUART_T::INTEN: ATOCTSEN Position     */
#define LPUART_INTEN_ATOCTSEN_Msk        (0x1ul << LPUART_INTEN_ATOCTSEN_Pos)              /*!< LPUART_T::INTEN: ATOCTSEN Mask         */

#define LPUART_INTEN_TXPDMAEN_Pos        (14)                                              /*!< LPUART_T::INTEN: TXPDMAEN Position     */
#define LPUART_INTEN_TXPDMAEN_Msk        (0x1ul << LPUART_INTEN_TXPDMAEN_Pos)              /*!< LPUART_T::INTEN: TXPDMAEN Mask         */

#define LPUART_INTEN_RXPDMAEN_Pos        (15)                                              /*!< LPUART_T::INTEN: RXPDMAEN Position     */
#define LPUART_INTEN_RXPDMAEN_Msk        (0x1ul << LPUART_INTEN_RXPDMAEN_Pos)              /*!< LPUART_T::INTEN: RXPDMAEN Mask         */

#define LPUART_INTEN_ABRIEN_Pos          (18)                                              /*!< LPUART_T::INTEN: ABRIEN Position       */
#define LPUART_INTEN_ABRIEN_Msk          (0x1ul << LPUART_INTEN_ABRIEN_Pos)                /*!< LPUART_T::INTEN: ABRIEN Mask           */

#define LPUART_INTEN_TXENDIEN_Pos        (22)                                              /*!< LPUART_T::INTEN: TXENDIEN Position     */
#define LPUART_INTEN_TXENDIEN_Msk        (0x1ul << LPUART_INTEN_TXENDIEN_Pos)              /*!< LPUART_T::INTEN: TXENDIEN Mask         */

#define LPUART_FIFO_RXRST_Pos            (1)                                               /*!< LPUART_T::FIFO: RXRST Position         */
#define LPUART_FIFO_RXRST_Msk            (0x1ul << LPUART_FIFO_RXRST_Pos)                  /*!< LPUART_T::FIFO: RXRST Mask             */

#define LPUART_FIFO_TXRST_Pos            (2)                                               /*!< LPUART_T::FIFO: TXRST Position         */
#define LPUART_FIFO_TXRST_Msk            (0x1ul << LPUART_FIFO_TXRST_Pos)                  /*!< LPUART_T::FIFO: TXRST Mask             */

#define LPUART_FIFO_RFITL_Pos            (4)                                               /*!< LPUART_T::FIFO: RFITL Position         */
#define LPUART_FIFO_RFITL_Msk            (0xful << LPUART_FIFO_RFITL_Pos)                  /*!< LPUART_T::FIFO: RFITL Mask             */

#define LPUART_FIFO_RXOFF_Pos            (8)                                               /*!< LPUART_T::FIFO: RXOFF Position         */
#define LPUART_FIFO_RXOFF_Msk            (0x1ul << LPUART_FIFO_RXOFF_Pos)                  /*!< LPUART_T::FIFO: RXOFF Mask             */

#define LPUART_FIFO_RTSTRGLV_Pos         (16)                                              /*!< LPUART_T::FIFO: RTSTRGLV Position      */
#define LPUART_FIFO_RTSTRGLV_Msk         (0xful << LPUART_FIFO_RTSTRGLV_Pos)               /*!< LPUART_T::FIFO: RTSTRGLV Mask          */

#define LPUART_LINE_WLS_Pos              (0)                                               /*!< LPUART_T::LINE: WLS Position           */
#define LPUART_LINE_WLS_Msk              (0x3ul << LPUART_LINE_WLS_Pos)                    /*!< LPUART_T::LINE: WLS Mask               */

#define LPUART_LINE_NSB_Pos              (2)                                               /*!< LPUART_T::LINE: NSB Position           */
#define LPUART_LINE_NSB_Msk              (0x1ul << LPUART_LINE_NSB_Pos)                    /*!< LPUART_T::LINE: NSB Mask               */

#define LPUART_LINE_PBE_Pos              (3)                                               /*!< LPUART_T::LINE: PBE Position           */
#define LPUART_LINE_PBE_Msk              (0x1ul << LPUART_LINE_PBE_Pos)                    /*!< LPUART_T::LINE: PBE Mask               */

#define LPUART_LINE_EPE_Pos              (4)                                               /*!< LPUART_T::LINE: EPE Position           */
#define LPUART_LINE_EPE_Msk              (0x1ul << LPUART_LINE_EPE_Pos)                    /*!< LPUART_T::LINE: EPE Mask               */

#define LPUART_LINE_SPE_Pos              (5)                                               /*!< LPUART_T::LINE: SPE Position           */
#define LPUART_LINE_SPE_Msk              (0x1ul << LPUART_LINE_SPE_Pos)                    /*!< LPUART_T::LINE: SPE Mask               */

#define LPUART_LINE_BCB_Pos              (6)                                               /*!< LPUART_T::LINE: BCB Position           */
#define LPUART_LINE_BCB_Msk              (0x1ul << LPUART_LINE_BCB_Pos)                    /*!< LPUART_T::LINE: BCB Mask               */

#define LPUART_LINE_PSS_Pos              (7)                                               /*!< LPUART_T::LINE: PSS Position           */
#define LPUART_LINE_PSS_Msk              (0x1ul << LPUART_LINE_PSS_Pos)                    /*!< LPUART_T::LINE: PSS Mask               */

#define LPUART_LINE_TXDINV_Pos           (8)                                               /*!< LPUART_T::LINE: TXDINV Position        */
#define LPUART_LINE_TXDINV_Msk           (0x1ul << LPUART_LINE_TXDINV_Pos)                 /*!< LPUART_T::LINE: TXDINV Mask            */

#define LPUART_LINE_RXDINV_Pos           (9)                                               /*!< LPUART_T::LINE: RXDINV Position        */
#define LPUART_LINE_RXDINV_Msk           (0x1ul << LPUART_LINE_RXDINV_Pos)                 /*!< LPUART_T::LINE: RXDINV Mask            */

#define LPUART_MODEM_RTS_Pos             (1)                                               /*!< LPUART_T::MODEM: RTS Position          */
#define LPUART_MODEM_RTS_Msk             (0x1ul << LPUART_MODEM_RTS_Pos)                   /*!< LPUART_T::MODEM: RTS Mask              */

#define LPUART_MODEM_LOOKBACK_Pos        (4)                                               /*!< LPUART_T::MODEM: LOOKBACK Position     */
#define LPUART_MODEM_LOOKBACK_Msk        (0x1ul << LPUART_MODEM_LOOKBACK_Pos)              /*!< LPUART_T::MODEM: LOOKBACK Mask         */

#define LPUART_MODEM_RTSACTLV_Pos        (9)                                               /*!< LPUART_T::MODEM: RTSACTLV Position     */
#define LPUART_MODEM_RTSACTLV_Msk        (0x1ul << LPUART_MODEM_RTSACTLV_Pos)              /*!< LPUART_T::MODEM: RTSACTLV Mask         */

#define LPUART_MODEM_RTSSTS_Pos          (13)                                              /*!< LPUART_T::MODEM: RTSSTS Position       */
#define LPUART_MODEM_RTSSTS_Msk          (0x1ul << LPUART_MODEM_RTSSTS_Pos)                /*!< LPUART_T::MODEM: RTSSTS Mask           */

#define LPUART_MODEMSTS_CTSDETF_Pos      (0)                                               /*!< LPUART_T::MODEMSTS: CTSDETF Position   */
#define LPUART_MODEMSTS_CTSDETF_Msk      (0x1ul << LPUART_MODEMSTS_CTSDETF_Pos)            /*!< LPUART_T::MODEMSTS: CTSDETF Mask       */

#define LPUART_MODEMSTS_CTSSTS_Pos       (4)                                               /*!< LPUART_T::MODEMSTS: CTSSTS Position    */
#define LPUART_MODEMSTS_CTSSTS_Msk       (0x1ul << LPUART_MODEMSTS_CTSSTS_Pos)             /*!< LPUART_T::MODEMSTS: CTSSTS Mask        */

#define LPUART_MODEMSTS_CTSACTLV_Pos     (8)                                               /*!< LPUART_T::MODEMSTS: CTSACTLV Position  */
#define LPUART_MODEMSTS_CTSACTLV_Msk     (0x1ul << LPUART_MODEMSTS_CTSACTLV_Pos)           /*!< LPUART_T::MODEMSTS: CTSACTLV Mask      */

#define LPUART_FIFOSTS_RXOVIF_Pos        (0)                                               /*!< LPUART_T::FIFOSTS: RXOVIF Position     */
#define LPUART_FIFOSTS_RXOVIF_Msk        (0x1ul << LPUART_FIFOSTS_RXOVIF_Pos)              /*!< LPUART_T::FIFOSTS: RXOVIF Mask         */

#define LPUART_FIFOSTS_ABRDIF_Pos        (1)                                               /*!< LPUART_T::FIFOSTS: ABRDIF Position     */
#define LPUART_FIFOSTS_ABRDIF_Msk        (0x1ul << LPUART_FIFOSTS_ABRDIF_Pos)              /*!< LPUART_T::FIFOSTS: ABRDIF Mask         */

#define LPUART_FIFOSTS_ABRDTOIF_Pos      (2)                                               /*!< LPUART_T::FIFOSTS: ABRDTOIF Position   */
#define LPUART_FIFOSTS_ABRDTOIF_Msk      (0x1ul << LPUART_FIFOSTS_ABRDTOIF_Pos)            /*!< LPUART_T::FIFOSTS: ABRDTOIF Mask       */

#define LPUART_FIFOSTS_ADDRDETF_Pos      (3)                                               /*!< LPUART_T::FIFOSTS: ADDRDETF Position   */
#define LPUART_FIFOSTS_ADDRDETF_Msk      (0x1ul << LPUART_FIFOSTS_ADDRDETF_Pos)            /*!< LPUART_T::FIFOSTS: ADDRDETF Mask       */

#define LPUART_FIFOSTS_PEF_Pos           (4)                                               /*!< LPUART_T::FIFOSTS: PEF Position        */
#define LPUART_FIFOSTS_PEF_Msk           (0x1ul << LPUART_FIFOSTS_PEF_Pos)                 /*!< LPUART_T::FIFOSTS: PEF Mask            */

#define LPUART_FIFOSTS_FEF_Pos           (5)                                               /*!< LPUART_T::FIFOSTS: FEF Position        */
#define LPUART_FIFOSTS_FEF_Msk           (0x1ul << LPUART_FIFOSTS_FEF_Pos)                 /*!< LPUART_T::FIFOSTS: FEF Mask            */

#define LPUART_FIFOSTS_BIF_Pos           (6)                                               /*!< LPUART_T::FIFOSTS: BIF Position        */
#define LPUART_FIFOSTS_BIF_Msk           (0x1ul << LPUART_FIFOSTS_BIF_Pos)                 /*!< LPUART_T::FIFOSTS: BIF Mask            */

#define LPUART_FIFOSTS_RXPTR_Pos         (8)                                               /*!< LPUART_T::FIFOSTS: RXPTR Position      */
#define LPUART_FIFOSTS_RXPTR_Msk         (0x3ful << LPUART_FIFOSTS_RXPTR_Pos)              /*!< LPUART_T::FIFOSTS: RXPTR Mask          */

#define LPUART_FIFOSTS_RXEMPTY_Pos       (14)                                              /*!< LPUART_T::FIFOSTS: RXEMPTY Position    */
#define LPUART_FIFOSTS_RXEMPTY_Msk       (0x1ul << LPUART_FIFOSTS_RXEMPTY_Pos)             /*!< LPUART_T::FIFOSTS: RXEMPTY Mask        */

#define LPUART_FIFOSTS_RXFULL_Pos        (15)                                              /*!< LPUART_T::FIFOSTS: RXFULL Position     */
#define LPUART_FIFOSTS_RXFULL_Msk        (0x1ul << LPUART_FIFOSTS_RXFULL_Pos)              /*!< LPUART_T::FIFOSTS: RXFULL Mask         */

#define LPUART_FIFOSTS_TXPTR_Pos         (16)                                              /*!< LPUART_T::FIFOSTS: TXPTR Position      */
#define LPUART_FIFOSTS_TXPTR_Msk         (0x3ful << LPUART_FIFOSTS_TXPTR_Pos)              /*!< LPUART_T::FIFOSTS: TXPTR Mask          */

#define LPUART_FIFOSTS_TXEMPTY_Pos       (22)                                              /*!< LPUART_T::FIFOSTS: TXEMPTY Position    */
#define LPUART_FIFOSTS_TXEMPTY_Msk       (0x1ul << LPUART_FIFOSTS_TXEMPTY_Pos)             /*!< LPUART_T::FIFOSTS: TXEMPTY Mask        */

#define LPUART_FIFOSTS_TXFULL_Pos        (23)                                              /*!< LPUART_T::FIFOSTS: TXFULL Position     */
#define LPUART_FIFOSTS_TXFULL_Msk        (0x1ul << LPUART_FIFOSTS_TXFULL_Pos)              /*!< LPUART_T::FIFOSTS: TXFULL Mask         */

#define LPUART_FIFOSTS_TXOVIF_Pos        (24)                                              /*!< LPUART_T::FIFOSTS: TXOVIF Position     */
#define LPUART_FIFOSTS_TXOVIF_Msk        (0x1ul << LPUART_FIFOSTS_TXOVIF_Pos)              /*!< LPUART_T::FIFOSTS: TXOVIF Mask         */

#define LPUART_FIFOSTS_TXEMPTYF_Pos      (28)                                              /*!< LPUART_T::FIFOSTS: TXEMPTYF Position   */
#define LPUART_FIFOSTS_TXEMPTYF_Msk      (0x1ul << LPUART_FIFOSTS_TXEMPTYF_Pos)            /*!< LPUART_T::FIFOSTS: TXEMPTYF Mask       */

#define LPUART_FIFOSTS_RXIDLE_Pos        (29)                                              /*!< LPUART_T::FIFOSTS: RXIDLE Position     */
#define LPUART_FIFOSTS_RXIDLE_Msk        (0x1ul << LPUART_FIFOSTS_RXIDLE_Pos)              /*!< LPUART_T::FIFOSTS: RXIDLE Mask         */

#define LPUART_FIFOSTS_TXRXACT_Pos       (31)                                              /*!< LPUART_T::FIFOSTS: TXRXACT Position    */
#define LPUART_FIFOSTS_TXRXACT_Msk       (0x1ul << LPUART_FIFOSTS_TXRXACT_Pos)             /*!< LPUART_T::FIFOSTS: TXRXACT Mask        */

#define LPUART_INTSTS_RDAIF_Pos          (0)                                               /*!< LPUART_T::INTSTS: RDAIF Position       */
#define LPUART_INTSTS_RDAIF_Msk          (0x1ul << LPUART_INTSTS_RDAIF_Pos)                /*!< LPUART_T::INTSTS: RDAIF Mask           */

#define LPUART_INTSTS_THREIF_Pos         (1)                                               /*!< LPUART_T::INTSTS: THREIF Position      */
#define LPUART_INTSTS_THREIF_Msk         (0x1ul << LPUART_INTSTS_THREIF_Pos)               /*!< LPUART_T::INTSTS: THREIF Mask          */

#define LPUART_INTSTS_RLSIF_Pos          (2)                                               /*!< LPUART_T::INTSTS: RLSIF Position       */
#define LPUART_INTSTS_RLSIF_Msk          (0x1ul << LPUART_INTSTS_RLSIF_Pos)                /*!< LPUART_T::INTSTS: RLSIF Mask           */

#define LPUART_INTSTS_MODEMIF_Pos        (3)                                               /*!< LPUART_T::INTSTS: MODEMIF Position     */
#define LPUART_INTSTS_MODEMIF_Msk        (0x1ul << LPUART_INTSTS_MODEMIF_Pos)              /*!< LPUART_T::INTSTS: MODEMIF Mask         */

#define LPUART_INTSTS_RXTOIF_Pos         (4)                                               /*!< LPUART_T::INTSTS: RXTOIF Position      */
#define LPUART_INTSTS_RXTOIF_Msk         (0x1ul << LPUART_INTSTS_RXTOIF_Pos)               /*!< LPUART_T::INTSTS: RXTOIF Mask          */

#define LPUART_INTSTS_BUFERRIF_Pos       (5)                                               /*!< LPUART_T::INTSTS: BUFERRIF Position    */
#define LPUART_INTSTS_BUFERRIF_Msk       (0x1ul << LPUART_INTSTS_BUFERRIF_Pos)             /*!< LPUART_T::INTSTS: BUFERRIF Mask        */

#define LPUART_INTSTS_WKIF_Pos           (6)                                               /*!< LPUART_T::INTSTS: WKIF Position        */
#define LPUART_INTSTS_WKIF_Msk           (0x1ul << LPUART_INTSTS_WKIF_Pos)                 /*!< LPUART_T::INTSTS: WKIF Mask            */

#define LPUART_INTSTS_RDAINT_Pos         (8)                                               /*!< LPUART_T::INTSTS: RDAINT Position      */
#define LPUART_INTSTS_RDAINT_Msk         (0x1ul << LPUART_INTSTS_RDAINT_Pos)               /*!< LPUART_T::INTSTS: RDAINT Mask          */

#define LPUART_INTSTS_THREINT_Pos        (9)                                               /*!< LPUART_T::INTSTS: THREINT Position     */
#define LPUART_INTSTS_THREINT_Msk        (0x1ul << LPUART_INTSTS_THREINT_Pos)              /*!< LPUART_T::INTSTS: THREINT Mask         */

#define LPUART_INTSTS_RLSINT_Pos         (10)                                              /*!< LPUART_T::INTSTS: RLSINT Position      */
#define LPUART_INTSTS_RLSINT_Msk         (0x1ul << LPUART_INTSTS_RLSINT_Pos)               /*!< LPUART_T::INTSTS: RLSINT Mask          */

#define LPUART_INTSTS_MODEMINT_Pos       (11)                                              /*!< LPUART_T::INTSTS: MODEMINT Position    */
#define LPUART_INTSTS_MODEMINT_Msk       (0x1ul << LPUART_INTSTS_MODEMINT_Pos)             /*!< LPUART_T::INTSTS: MODEMINT Mask        */

#define LPUART_INTSTS_RXTOINT_Pos        (12)                                              /*!< LPUART_T::INTSTS: RXTOINT Position     */
#define LPUART_INTSTS_RXTOINT_Msk        (0x1ul << LPUART_INTSTS_RXTOINT_Pos)              /*!< LPUART_T::INTSTS: RXTOINT Mask         */

#define LPUART_INTSTS_BUFERRINT_Pos      (13)                                              /*!< LPUART_T::INTSTS: BUFERRINT Position   */
#define LPUART_INTSTS_BUFERRINT_Msk      (0x1ul << LPUART_INTSTS_BUFERRINT_Pos)            /*!< LPUART_T::INTSTS: BUFERRINT Mask       */

#define LPUART_INTSTS_WKINT_Pos          (14)                                              /*!< LPUART_T::INTSTS: WKINT Position       */
#define LPUART_INTSTS_WKINT_Msk          (0x1ul << LPUART_INTSTS_WKINT_Pos)                /*!< LPUART_T::INTSTS: WKINT Mask           */

#define LPUART_INTSTS_HWRLSIF_Pos        (18)                                              /*!< LPUART_T::INTSTS: HWRLSIF Position     */
#define LPUART_INTSTS_HWRLSIF_Msk        (0x1ul << LPUART_INTSTS_HWRLSIF_Pos)              /*!< LPUART_T::INTSTS: HWRLSIF Mask         */

#define LPUART_INTSTS_HWMODIF_Pos        (19)                                              /*!< LPUART_T::INTSTS: HWMODIF Position     */
#define LPUART_INTSTS_HWMODIF_Msk        (0x1ul << LPUART_INTSTS_HWMODIF_Pos)              /*!< LPUART_T::INTSTS: HWMODIF Mask         */

#define LPUART_INTSTS_HWTOIF_Pos         (20)                                              /*!< LPUART_T::INTSTS: HWTOIF Position      */
#define LPUART_INTSTS_HWTOIF_Msk         (0x1ul << LPUART_INTSTS_HWTOIF_Pos)               /*!< LPUART_T::INTSTS: HWTOIF Mask          */

#define LPUART_INTSTS_HWBUFEIF_Pos       (21)                                              /*!< LPUART_T::INTSTS: HWBUFEIF Position    */
#define LPUART_INTSTS_HWBUFEIF_Msk       (0x1ul << LPUART_INTSTS_HWBUFEIF_Pos)             /*!< LPUART_T::INTSTS: HWBUFEIF Mask        */

#define LPUART_INTSTS_TXENDIF_Pos        (22)                                              /*!< LPUART_T::INTSTS: TXENDIF Position     */
#define LPUART_INTSTS_TXENDIF_Msk        (0x1ul << LPUART_INTSTS_TXENDIF_Pos)              /*!< LPUART_T::INTSTS: TXENDIF Mask         */

#define LPUART_INTSTS_HWRLSINT_Pos       (26)                                              /*!< LPUART_T::INTSTS: HWRLSINT Position    */
#define LPUART_INTSTS_HWRLSINT_Msk       (0x1ul << LPUART_INTSTS_HWRLSINT_Pos)             /*!< LPUART_T::INTSTS: HWRLSINT Mask        */

#define LPUART_INTSTS_HWMODINT_Pos       (27)                                              /*!< LPUART_T::INTSTS: HWMODINT Position    */
#define LPUART_INTSTS_HWMODINT_Msk       (0x1ul << LPUART_INTSTS_HWMODINT_Pos)             /*!< LPUART_T::INTSTS: HWMODINT Mask        */

#define LPUART_INTSTS_HWTOINT_Pos        (28)                                              /*!< LPUART_T::INTSTS: HWTOINT Position     */
#define LPUART_INTSTS_HWTOINT_Msk        (0x1ul << LPUART_INTSTS_HWTOINT_Pos)              /*!< LPUART_T::INTSTS: HWTOINT Mask         */

#define LPUART_INTSTS_HWBUFEINT_Pos      (29)                                              /*!< LPUART_T::INTSTS: HWBUFEINT Position   */
#define LPUART_INTSTS_HWBUFEINT_Msk      (0x1ul << LPUART_INTSTS_HWBUFEINT_Pos)            /*!< LPUART_T::INTSTS: HWBUFEINT Mask       */

#define LPUART_INTSTS_TXENDINT_Pos       (30)                                              /*!< LPUART_T::INTSTS: TXENDINT Position    */
#define LPUART_INTSTS_TXENDINT_Msk       (0x1ul << LPUART_INTSTS_TXENDINT_Pos)             /*!< LPUART_T::INTSTS: TXENDINT Mask        */

#define LPUART_INTSTS_ABRINT_Pos         (31)                                              /*!< LPUART_T::INTSTS: ABRINT Position      */
#define LPUART_INTSTS_ABRINT_Msk         (0x1ul << LPUART_INTSTS_ABRINT_Pos)               /*!< LPUART_T::INTSTS: ABRINT Mask          */

#define LPUART_TOUT_TOIC_Pos             (0)                                               /*!< LPUART_T::TOUT: TOIC Position          */
#define LPUART_TOUT_TOIC_Msk             (0xfful << LPUART_TOUT_TOIC_Pos)                  /*!< LPUART_T::TOUT: TOIC Mask              */

#define LPUART_TOUT_DLY_Pos              (8)                                               /*!< LPUART_T::TOUT: DLY Position           */
#define LPUART_TOUT_DLY_Msk              (0xfful << LPUART_TOUT_DLY_Pos)                   /*!< LPUART_T::TOUT: DLY Mask               */

#define LPUART_TOUT_BITOMEN_Pos          (31)                                              /*!< LPUART_T::TOUT: BITOMEN Position       */
#define LPUART_TOUT_BITOMEN_Msk          (0x1ul << LPUART_TOUT_BITOMEN_Pos)                /*!< LPUART_T::TOUT: BITOMEN Mask           */

#define LPUART_BAUD_BRD_Pos              (0)                                               /*!< LPUART_T::BAUD: BRD Position           */
#define LPUART_BAUD_BRD_Msk              (0xfffful << LPUART_BAUD_BRD_Pos)                 /*!< LPUART_T::BAUD: BRD Mask               */

#define LPUART_BAUD_EDIVM1_Pos           (24)                                              /*!< LPUART_T::BAUD: EDIVM1 Position        */
#define LPUART_BAUD_EDIVM1_Msk           (0xful << LPUART_BAUD_EDIVM1_Pos)                 /*!< LPUART_T::BAUD: EDIVM1 Mask            */

#define LPUART_BAUD_BAUDM0_Pos           (28)                                              /*!< LPUART_T::BAUD: BAUDM0 Position        */
#define LPUART_BAUD_BAUDM0_Msk           (0x1ul << LPUART_BAUD_BAUDM0_Pos)                 /*!< LPUART_T::BAUD: BAUDM0 Mask            */

#define LPUART_BAUD_BAUDM1_Pos           (29)                                              /*!< LPUART_T::BAUD: BAUDM1 Position        */
#define LPUART_BAUD_BAUDM1_Msk           (0x1ul << LPUART_BAUD_BAUDM1_Pos)                 /*!< LPUART_T::BAUD: BAUDM1 Mask            */

#define LPUART_ALTCTL_RS485NMM_Pos       (8)                                               /*!< LPUART_T::ALTCTL: RS485NMM Position    */
#define LPUART_ALTCTL_RS485NMM_Msk       (0x1ul << LPUART_ALTCTL_RS485NMM_Pos)             /*!< LPUART_T::ALTCTL: RS485NMM Mask        */

#define LPUART_ALTCTL_RS485AAD_Pos       (9)                                               /*!< LPUART_T::ALTCTL: RS485AAD Position    */
#define LPUART_ALTCTL_RS485AAD_Msk       (0x1ul << LPUART_ALTCTL_RS485AAD_Pos)             /*!< LPUART_T::ALTCTL: RS485AAD Mask        */

#define LPUART_ALTCTL_RS485AUD_Pos       (10)                                              /*!< LPUART_T::ALTCTL: RS485AUD Position    */
#define LPUART_ALTCTL_RS485AUD_Msk       (0x1ul << LPUART_ALTCTL_RS485AUD_Pos)             /*!< LPUART_T::ALTCTL: RS485AUD Mask        */

#define LPUART_ALTCTL_ADDRDEN_Pos        (15)                                              /*!< LPUART_T::ALTCTL: ADDRDEN Position     */
#define LPUART_ALTCTL_ADDRDEN_Msk        (0x1ul << LPUART_ALTCTL_ADDRDEN_Pos)              /*!< LPUART_T::ALTCTL: ADDRDEN Mask         */

#define LPUART_ALTCTL_ABRIF_Pos          (17)                                              /*!< LPUART_T::ALTCTL: ABRIF Position       */
#define LPUART_ALTCTL_ABRIF_Msk          (0x1ul << LPUART_ALTCTL_ABRIF_Pos)                /*!< LPUART_T::ALTCTL: ABRIF Mask           */

#define LPUART_ALTCTL_ABRDEN_Pos         (18)                                              /*!< LPUART_T::ALTCTL: ABRDEN Position      */
#define LPUART_ALTCTL_ABRDEN_Msk         (0x1ul << LPUART_ALTCTL_ABRDEN_Pos)               /*!< LPUART_T::ALTCTL: ABRDEN Mask          */

#define LPUART_ALTCTL_ABRDBITS_Pos       (19)                                              /*!< LPUART_T::ALTCTL: ABRDBITS Position    */
#define LPUART_ALTCTL_ABRDBITS_Msk       (0x3ul << LPUART_ALTCTL_ABRDBITS_Pos)             /*!< LPUART_T::ALTCTL: ABRDBITS Mask        */

#define LPUART_ALTCTL_ADDRMV_Pos         (24)                                              /*!< LPUART_T::ALTCTL: ADDRMV Position      */
#define LPUART_ALTCTL_ADDRMV_Msk         (0xfful << LPUART_ALTCTL_ADDRMV_Pos)              /*!< LPUART_T::ALTCTL: ADDRMV Mask          */

#define LPUART_FUNCSEL_FUNCSEL_Pos       (0)                                               /*!< LPUART_T::FUNCSEL: FUNCSEL Position    */
#define LPUART_FUNCSEL_FUNCSEL_Msk       (0x7ul << LPUART_FUNCSEL_FUNCSEL_Pos)             /*!< LPUART_T::FUNCSEL: FUNCSEL Mask        */

#define LPUART_FUNCSEL_TXRXDIS_Pos       (3)                                               /*!< LPUART_T::FUNCSEL: TXRXDIS Position    */
#define LPUART_FUNCSEL_TXRXDIS_Msk       (0x1ul << LPUART_FUNCSEL_TXRXDIS_Pos)             /*!< LPUART_T::FUNCSEL: TXRXDIS Mask        */

#define LPUART_FUNCSEL_DGE_Pos           (6)                                               /*!< LPUART_T::FUNCSEL: DGE Position        */
#define LPUART_FUNCSEL_DGE_Msk           (0x1ul << LPUART_FUNCSEL_DGE_Pos)                 /*!< LPUART_T::FUNCSEL: DGE Mask            */

#define LPUART_FUNCSEL_TXRXSWP_Pos       (7)                                               /*!< LPUART_T::FUNCSEL: TXRXSWP Position    */
#define LPUART_FUNCSEL_TXRXSWP_Msk       (0x1ul << LPUART_FUNCSEL_TXRXSWP_Pos)             /*!< LPUART_T::FUNCSEL: TXRXSWP Mask        */

#define LPUART_BRCOMP_BRCOMP_Pos         (0)                                               /*!< LPUART_T::BRCOMP: BRCOMP Position      */
#define LPUART_BRCOMP_BRCOMP_Msk         (0x1fful << LPUART_BRCOMP_BRCOMP_Pos)             /*!< LPUART_T::BRCOMP: BRCOMP Mask          */

#define LPUART_BRCOMP_BRCOMPDEC_Pos      (31)                                              /*!< LPUART_T::BRCOMP: BRCOMPDEC Position   */
#define LPUART_BRCOMP_BRCOMPDEC_Msk      (0x1ul << LPUART_BRCOMP_BRCOMPDEC_Pos)            /*!< LPUART_T::BRCOMP: BRCOMPDEC Mask       */

#define LPUART_WKCTL_WKCTSEN_Pos         (0)                                               /*!< LPUART_T::WKCTL: WKCTSEN Position      */
#define LPUART_WKCTL_WKCTSEN_Msk         (0x1ul << LPUART_WKCTL_WKCTSEN_Pos)               /*!< LPUART_T::WKCTL: WKCTSEN Mask          */

#define LPUART_WKCTL_WKDATEN_Pos         (1)                                               /*!< LPUART_T::WKCTL: WKDATEN Position      */
#define LPUART_WKCTL_WKDATEN_Msk         (0x1ul << LPUART_WKCTL_WKDATEN_Pos)               /*!< LPUART_T::WKCTL: WKDATEN Mask          */

#define LPUART_WKCTL_WKRFRTEN_Pos        (2)                                               /*!< LPUART_T::WKCTL: WKRFRTEN Position     */
#define LPUART_WKCTL_WKRFRTEN_Msk        (0x1ul << LPUART_WKCTL_WKRFRTEN_Pos)              /*!< LPUART_T::WKCTL: WKRFRTEN Mask         */

#define LPUART_WKCTL_WKRS485EN_Pos       (3)                                               /*!< LPUART_T::WKCTL: WKRS485EN Position    */
#define LPUART_WKCTL_WKRS485EN_Msk       (0x1ul << LPUART_WKCTL_WKRS485EN_Pos)             /*!< LPUART_T::WKCTL: WKRS485EN Mask        */

#define LPUART_WKCTL_WKTOUTEN_Pos        (4)                                               /*!< LPUART_T::WKCTL: WKTOUTEN Position     */
#define LPUART_WKCTL_WKTOUTEN_Msk        (0x1ul << LPUART_WKCTL_WKTOUTEN_Pos)              /*!< LPUART_T::WKCTL: WKTOUTEN Mask         */

#define LPUART_WKSTS_CTSWKF_Pos          (0)                                               /*!< LPUART_T::WKSTS: CTSWKF Position       */
#define LPUART_WKSTS_CTSWKF_Msk          (0x1ul << LPUART_WKSTS_CTSWKF_Pos)                /*!< LPUART_T::WKSTS: CTSWKF Mask           */

#define LPUART_WKSTS_DATWKF_Pos          (1)                                               /*!< LPUART_T::WKSTS: DATWKF Position       */
#define LPUART_WKSTS_DATWKF_Msk          (0x1ul << LPUART_WKSTS_DATWKF_Pos)                /*!< LPUART_T::WKSTS: DATWKF Mask           */

#define LPUART_WKSTS_RFRTWKF_Pos         (2)                                               /*!< LPUART_T::WKSTS: RFRTWKF Position      */
#define LPUART_WKSTS_RFRTWKF_Msk         (0x1ul << LPUART_WKSTS_RFRTWKF_Pos)               /*!< LPUART_T::WKSTS: RFRTWKF Mask          */

#define LPUART_WKSTS_RS485WKF_Pos        (3)                                               /*!< LPUART_T::WKSTS: RS485WKF Position     */
#define LPUART_WKSTS_RS485WKF_Msk        (0x1ul << LPUART_WKSTS_RS485WKF_Pos)              /*!< LPUART_T::WKSTS: RS485WKF Mask         */

#define LPUART_WKSTS_TOUTWKF_Pos         (4)                                               /*!< LPUART_T::WKSTS: TOUTWKF Position      */
#define LPUART_WKSTS_TOUTWKF_Msk         (0x1ul << LPUART_WKSTS_TOUTWKF_Pos)               /*!< LPUART_T::WKSTS: TOUTWKF Mask          */

#define LPUART_DWKCOMP_STCOMP_Pos        (0)                                               /*!< LPUART_T::DWKCOMP: STCOMP Position     */
#define LPUART_DWKCOMP_STCOMP_Msk        (0xfffful << LPUART_DWKCOMP_STCOMP_Pos)           /*!< LPUART_T::DWKCOMP: STCOMP Mask         */

#define LPUART_RS485DD_RTSDDLY_Pos       (0)                                               /*!< LPUART_T::RS485DD: RTSDDLY Position    */
#define LPUART_RS485DD_RTSDDLY_Msk       (0xfffful << LPUART_RS485DD_RTSDDLY_Pos)          /*!< LPUART_T::RS485DD: RTSDDLY Mask        */

#define LPUART_AUTOCTL_TRIGSEL_Pos       (0)                                               /*!< LPUART_T::AUTOCTL: TRIGSEL Position    */
#define LPUART_AUTOCTL_TRIGSEL_Msk       (0xful << LPUART_AUTOCTL_TRIGSEL_Pos)             /*!< LPUART_T::AUTOCTL: TRIGSEL Mask        */

#define LPUART_AUTOCTL_TRIGEN_Pos        (4)                                               /*!< LPUART_T::AUTOCTL: TRIGEN Position     */
#define LPUART_AUTOCTL_TRIGEN_Msk        (0x1ul << LPUART_AUTOCTL_TRIGEN_Pos)              /*!< LPUART_T::AUTOCTL: TRIGEN Mask         */

#define LPUART_AUTOCTL_CKAWOEN_Pos       (5)                                               /*!< LPUART_T::AUTOCTL: CKAWOEN Position    */
#define LPUART_AUTOCTL_CKAWOEN_Msk       (0x1ul << LPUART_AUTOCTL_CKAWOEN_Pos)             /*!< LPUART_T::AUTOCTL: CKAWOEN Mask        */

#define LPUART_AUTOCTL_WKAOTOEN_Pos      (8)                                               /*!< LPUART_T::AUTOCTL: WKAOTOEN Position   */
#define LPUART_AUTOCTL_WKAOTOEN_Msk      (0x1ul << LPUART_AUTOCTL_WKAOTOEN_Pos)            /*!< LPUART_T::AUTOCTL: WKAOTOEN Mask       */

#define LPUART_AUTOCTL_SWTRIG_Pos        (9)                                               /*!< LPUART_T::AUTOCTL: SWTRIG Position     */
#define LPUART_AUTOCTL_SWTRIG_Msk        (0x1ul << LPUART_AUTOCTL_SWTRIG_Pos)              /*!< LPUART_T::AUTOCTL: SWTRIG Mask         */

#define LPUART_AUTOCTL_AOEN_Pos          (31)                                              /*!< LPUART_T::AUTOCTL: AOEN Position       */
#define LPUART_AUTOCTL_AOEN_Msk          (0x1ul << LPUART_AUTOCTL_AOEN_Pos)                /*!< LPUART_T::AUTOCTL: AOEN Mask           */

#define LPUART_AUTOSTS_AOTOWKF_Pos       (0)                                               /*!< LPUART_T::AUTOSTS: AOTOWKF Position    */
#define LPUART_AUTOSTS_AOTOWKF_Msk       (0x1ul << LPUART_AUTOSTS_AOTOWKF_Pos)             /*!< LPUART_T::AUTOSTS: AOTOWKF Mask        */

/** @} LPUART_CONST */
/** @} end of LPUART register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __LPUART_REG_H__ */
