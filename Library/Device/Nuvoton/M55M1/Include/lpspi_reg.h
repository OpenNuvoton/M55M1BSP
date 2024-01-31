/**************************************************************************//**
 * @file     lpspi_reg.h
 * @version  V1.00
 * @brief    LPSPI register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __LPSPI_REG_H__
#define __LPSPI_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup LPSPI Low Power Serial Peripheral Interface (LPSPI)
    Memory Mapped Structure for LPSPI Controller
    @{
*/

typedef struct
{
    /**
     * @var LPSPI_T::CTL
     * Offset: 0x00  SPI Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SPIEN     |SPI Transfer Control Enable Bit
     * |        |          |In Master mode, the transfer will start when there is data in the FIFO buffer after this bit is set to 1
     * |        |          |In Slave mode, this device is ready to receive data when this bit is set to 1.
     * |        |          |0 = Transfer control Disabled.
     * |        |          |1 = Transfer control Enabled.
     * |        |          |Note: Before changing the configurations of LPSPI_CTL, LPSPI_CLKDIV, LPSPI_SSCTL and LPSPI_FIFOCTL registers, user shall clear the SPIEN (LPSPI_CTL[0]) and confirm the SPIENSTS (LPSPI_STATUS[15]) is 0.
     * |[1]     |RXNEG     |Receive on Negative Edge
     * |        |          |0 = Received data input signal is latched on the rising edge of SPI bus clock.
     * |        |          |1 = Received data input signal is latched on the falling edge of SPI bus clock.
     * |[2]     |TXNEG     |Transmit on Negative Edge
     * |        |          |0 = Transmitted data output signal is changed on the rising edge of SPI bus clock.
     * |        |          |1 = Transmitted data output signal is changed on the falling edge of SPI bus clock.
     * |[3]     |CLKPOL    |Clock Polarity
     * |        |          |0 = SPI bus clock is idle low.
     * |        |          |1 = SPI bus clock is idle high.
     * |[7:4]   |SUSPITV   |Suspend Interval
     * |        |          |The four bits provide configurable suspend interval between two successive transmit/receive transaction in a transfer
     * |        |          |The definition of the suspend interval is the interval between the last clock edge of the preceding transaction word and the first clock edge of the following transaction word
     * |        |          |The default value is 0x3
     * |        |          |The period of the suspend interval is obtained according to the following equation.
     * |        |          |(SUSPITV[3:0] + 0.5) * period of LPSPI_CLK clock cycle
     * |        |          |Example:
     * |        |          |SUSPITV = 0x0 u2026. 0.5 LPSPI_CLK clock cycle.
     * |        |          |SUSPITV = 0x1 u2026. 1.5 LPSPI_CLK clock cycle.
     * |        |          |u2026u2026
     * |        |          |SUSPITV = 0xE u2026. 14.5 LPSPI_CLK clock cycle.
     * |        |          |SUSPITV = 0xF u2026. 15.5 LPSPI_CLK clock cycle.
     * |        |          |Note: These bits are for Master Mode only.
     * |[12:8]  |DWIDTH    |Data Width
     * |        |          |This field specifies how many bits can be transmitted/received in one transaction
     * |        |          |The minimum bit length is 4 bits and can up to 32 bits.
     * |        |          |DWIDTH = 0x04 u2026. 4 bits.
     * |        |          |DWIDTH = 0x05 u2026. 5 bits.
     * |        |          |DWIDTH = 0x06 u2026. 6 bits.
     * |        |          |DWIDTH = 0x07 u2026. 7 bits.
     * |        |          |DWIDTH = 0x08 u2026. 8 bits.
     * |        |          |DWIDTH = 0x09 u2026. 9 bits.
     * |        |          |u2026u2026
     * |        |          |DWIDTH = 0x1F u2026. 31 bits.
     * |        |          |DWIDTH = 0x00 u2026. 32 bits.
     * |        |          |Note: This bit field will decide the depth of TX/RX FIFO configuration in SPI mode
     * |        |          |Therefore, changing this bit field will clear TX/RX FIFO by hardware automatically.
     * |[13]    |LSB       |Send LSB First
     * |        |          |0 = The MSB, which bit of transmit/receive register depends on the setting of DWIDTH, is transmitted/received first.
     * |        |          |1 = The LSB, bit 0 of the SPI TX register, is sent first to the SPI data output pin, and the first bit received from the SPI data input pin will be put in the LSB position of the RX register (bit 0 of LPSPI_RX).
     * |[14]    |HALFDPX   |SPI Half-duplex Transfer Enable Bit
     * |        |          |This bit is used to select full-duplex or half-duplex for SPI transfer
     * |        |          |The bit field DATDIR (LPSPI_CTL[20]) can be used to set the data direction in half-duplex transfer.
     * |        |          |0 = SPI operates in full-duplex transfer.
     * |        |          |1 = SPI operates in half-duplex transfer.
     * |[15]    |RXONLY    |Receive-only Mode Enable Bit
     * |        |          |This bit field is only available in Master mode
     * |        |          |In receive-only mode, SPI Master will generate SPI bus clock continuously for receiving data bit from SPI slave device and assert the BUSY status.
     * |        |          |0 = Receive-only mode Disabled.
     * |        |          |1 = Receive-only mode Enabled.
     * |        |          |Note: This bit is for Master Mode only.
     * |[17]    |UNITIEN   |Unit Transfer Interrupt Enable Bit
     * |        |          |0 = SPI unit transfer interrupt Disabled.
     * |        |          |1 = SPI unit transfer interrupt Enabled.
     * |[18]    |SLAVE     |Slave Mode Control
     * |        |          |0 = Master mode.
     * |        |          |1 = Slave mode.
     * |[19]    |REORDER   |Byte Reorder Function Enable Bit
     * |        |          |0 = Byte Reorder function Disabled.
     * |        |          |1 = Byte Reorder function Enabled
     * |        |          |A byte suspend interval will be inserted among each byte
     * |        |          |The period of the byte suspend interval depends on the setting of SUSPITV.
     * |        |          |Note: Byte Reorder function is only available if DWIDTH is defined as 16, 24, and 32 bits.
     * |[20]    |DATDIR    |Data Port Direction Control
     * |        |          |This bit is used to select the data input/output direction in half-duplex transfer and Dual/Quad transfer
     * |        |          |0 = SPI data is input direction.
     * |        |          |1 = SPI data is output direction.
     * @var LPSPI_T::CLKDIV
     * Offset: 0x04  SPI Clock Divider Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8:0]   |DIVIDER   |Clock Divider
     * |        |          |The value in this field is the frequency divider for generating the peripheral clock, fspi_eclk, and the SPI bus clock of SPI Master
     * |        |          |The frequency is obtained according to the following equation.
     * |        |          |where
     * |        |          |is the peripheral clock source, which is defined in the clock control register, CLK_CLKSEL2 or CLK_CLKSEL3.
     * |        |          |Note 1: The time interval must be larger than or equal to 8 peripheral clock cycles between releasing SPI IP software reset and setting this clock divider register.
     * @var LPSPI_T::SSCTL
     * Offset: 0x08  SPI Slave Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SS        |Slave Selection Control
     * |        |          |If AUTOSS bit is cleared to 0,
     * |        |          |0 = set the LPSPI_SS line to inactive state.
     * |        |          |1 = set the LPSPI_SS line to active state.
     * |        |          |If the AUTOSS bit is set to 1,
     * |        |          |0 = Keep the LPSPI_SS line at inactive state.
     * |        |          |1 = LPSPI_SS line will be automatically driven to active state for the duration of data transfer, and will be driven to inactive state for the rest of the time
     * |        |          |The active state of LPSPI_SS is specified in SSACTPOL (LPSPI_SSCTL[2]).
     * |        |          |Note: This bit is for Master Mode only.
     * |[2]     |SSACTPOL  |Slave Selection Active Polarity
     * |        |          |This bit defines the active polarity of slave selection signal (LPSPI_SS).
     * |        |          |0 = The slave selection signal LPSPI_SS is active low.
     * |        |          |1 = The slave selection signal LPSPI_SS is active high.
     * |[3]     |AUTOSS    |Automatic Slave Selection Function Enable Bit
     * |        |          |0 = Automatic slave selection function Disabled
     * |        |          |Slave selection signal will be asserted/de-asserted according to SS (LPSPI_SSCTL[0]).
     * |        |          |1 = Automatic slave selection function Enabled.
     * |        |          |Note: This bit is for Master Mode only.
     * |[4]     |SLV3WIRE  |Slave 3-wire Mode Enable Bit
     * |        |          |In Slave 3-wire mode, the SPI controller can work with 3-wire interface including LPSPI_CLK, LPSPI_MISO and LPSPI_MOSI pins.
     * |        |          |0 = 4-wire bi-direction interface.
     * |        |          |1 = 3-wire bi-direction interface.
     * |        |          |Note: This bit is for Slave Mode only.
     * |[8]     |SLVBEIEN  |Slave Mode Bit Count Error Interrupt Enable Bit
     * |        |          |0 = Slave mode bit count error interrupt Disabled.
     * |        |          |1 = Slave mode bit count error interrupt Enabled.
     * |[9]     |SLVURIEN  |Slave Mode TX Under Run Interrupt Enable Bit
     * |        |          |0 = Slave mode TX under run interrupt Disabled.
     * |        |          |1 = Slave mode TX under run interrupt Enabled.
     * |[12]    |SSACTIEN  |Slave Select Active Interrupt Enable Bit
     * |        |          |0 = Slave select active interrupt Disabled.
     * |        |          |1 = Slave select active interrupt Enabled.
     * |[13]    |SSINAIEN  |Slave Select Inactive Interrupt Enable Bit
     * |        |          |0 = Slave select inactive interrupt Disabled.
     * |        |          |1 = Slave select inactive interrupt Enabled.
     * @var LPSPI_T::PDMACTL
     * Offset: 0x0C  SPI PDMA Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TXPDMAEN  |Transmit PDMA Enable Bit
     * |        |          |0 = Transmit PDMA function Disabled.
     * |        |          |1 = Transmit PDMA function Enabled.
     * |        |          |Note 1: In SPI Master mode with full duplex transfer, if both TX and RX PDMA functions are enabled, RX PDMA function cannot be enabled prior to TX PDMA function
     * |        |          |User can enable TX PDMA function firstly or enable both functions simultaneously.
     * |        |          |Note 2: In SPI Master mode with full duplex transfer, if both TX and RX PDMA functions are enabled, TX PDMA function cannot be disabled prior to RX PDMA function
     * |        |          |User can disable RX PDMA function firstly or disable both functions simultaneously.
     * |[1]     |RXPDMAEN  |Receive PDMA Enable Bit
     * |        |          |0 = Receive PDMA function Disabled.
     * |        |          |1 = Receive PDMA function Enabled.
     * |[2]     |PDMARST   |PDMA Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Reset the PDMA control logic of the SPI controller. This bit will be automatically cleared to 0.
     * @var LPSPI_T::FIFOCTL
     * Offset: 0x10  SPI FIFO Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXRST     |Receive Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Reset receive FIFO pointer and receive circuit
     * |        |          |The RXFULL bit will be cleared to 0 and the RXEMPTY bit will be set to 1
     * |        |          |This bit will be cleared to 0 by hardware about 3 system clock cycles + 2 peripheral clock cycles after it is set to 1
     * |        |          |User can read TXRXRST (LPSPI_STATUS[23]) to check if reset is accomplished or not.
     * |[1]     |TXRST     |Transmit Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Reset transmit FIFO pointer and transmit circuit
     * |        |          |The TXFULL bit will be cleared to 0 and the TXEMPTY bit will be set to 1
     * |        |          |This bit will be cleared to 0 by hardware about 3 system clock cycles + 2 peripheral clock cycles after it is set to 1
     * |        |          |User can read TXRXRST (LPSPI_STATUS[23]) to check if reset is accomplished or not.
     * |        |          |Note: If TX underflow event occurs in SPI Slave mode, this bit can be used to make SPI return to idle state.
     * |[2]     |RXTHIEN   |Receive FIFO Threshold Interrupt Enable Bit
     * |        |          |0 = RX FIFO threshold interrupt Disabled.
     * |        |          |1 = RX FIFO threshold interrupt Enabled.
     * |[3]     |TXTHIEN   |Transmit FIFO Threshold Interrupt Enable Bit
     * |        |          |0 = TX FIFO threshold interrupt Disabled.
     * |        |          |1 = TX FIFO threshold interrupt Enabled.
     * |[4]     |RXTOIEN   |Receive Time-out Interrupt Enable Bit
     * |        |          |0 = Receive time-out interrupt Disabled.
     * |        |          |1 = Receive time-out interrupt Enabled.
     * |[5]     |RXOVIEN   |Receive FIFO Overrun Interrupt Enable Bit
     * |        |          |0 = Receive FIFO overrun interrupt Disabled.
     * |        |          |1 = Receive FIFO overrun interrupt Enabled.
     * |[6]     |TXUFPOL   |TX Underflow Data Polarity
     * |        |          |0 = The SPI data out is keep 0 if there is TX underflow event in Slave mode.
     * |        |          |1 = The SPI data out is keep 1 if there is TX underflow event in Slave mode.
     * |        |          |Note 1: The TX underflow event occurs if there is no any data in TX FIFO when the slave selection signal is active.
     * |        |          |Note 2: When TX underflow event occurs, LPSPI_MISO pin state will be determined by this setting even though TX FIFO is not empty afterward
     * |        |          |Data stored in TX FIFO will be sent through LPSPI_MISO pin in the next transfer frame.
     * |[7]     |TXUFIEN   |TX Underflow Interrupt Enable Bit
     * |        |          |When TX underflow event occurs in Slave mode, TXUFIF (LPSPI_STATUS[19]) will be set to 1
     * |        |          |This bit is used to enable the TX underflow interrupt.
     * |        |          |0 = Slave TX underflow interrupt Disabled.
     * |        |          |1 = Slave TX underflow interrupt Enabled.
     * |[8]     |RXFBCLR   |Receive FIFO Buffer Clear
     * |        |          |0 = No effect.
     * |        |          |1 = Clear receive FIFO pointer
     * |        |          |The RXFULL bit will be cleared to 0 and the RXEMPTY bit will be set to 1
     * |        |          |This bit will be cleared to 0 by hardware about 1 system clock after it is set to 1.
     * |        |          |Note: The RX shift register will not be cleared.
     * |[9]     |TXFBCLR   |Transmit FIFO Buffer Clear
     * |        |          |0 = No effect.
     * |        |          |1 = Clear transmit FIFO pointer
     * |        |          |The TXFULL bit will be cleared to 0 and the TXEMPTY bit will be set to 1
     * |        |          |This bit will be cleared to 0 by hardware about 1 system clock after it is set to 1.
     * |        |          |Note: The TX shift register will not be cleared.
     * |[10]    |SLVBERX   |RX FIFO Write Data Enable Bit When Slave Mode Bit Count Error
     * |        |          |0 = Uncompleted RX data will be dropped from RX FIFO when bit count error event happened in SPI Slave mode.
     * |        |          |1 = Uncompleted RX data will be written into RX FIFO when bit count error event happened in SPI Slave mode
     * |        |          |User can read SLVBENUM (LPSPI_STATUS2[29:24]) to know that the effective bit number of uncompleted RX data when SPI slave bit count error happened.
     * |        |          |Note: This bit is for Slave Mode only.
     * |[26:24] |RXTH      |Receive FIFO Threshold
     * |        |          |If the valid data count of the receive FIFO buffer is larger than the RXTH setting, the RXTHIF bit will be set to 1, else the RXTHIF bit will be cleared to 0
     * |        |          |The MSB of this bit field is only meaningful while SPI mode 4~16 bits of data length.
     * |[30:28] |TXTH      |Transmit FIFO Threshold
     * |        |          |If the valid data count of the transmit FIFO buffer is less than or equal to the TXTH setting, the TXTHIF bit will be set to 1, else the TXTHIF bit will be cleared to 0
     * |        |          |The MSB of this bit field is only meaningful while SPI mode 4~16 bits of data length
     * @var LPSPI_T::STATUS
     * Offset: 0x14  SPI Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BUSY      |Busy Status (Read Only)
     * |        |          |0 = SPI controller is in idle state.
     * |        |          |1 = SPI controller is in busy state.
     * |        |          |The following lists the bus busy conditions:
     * |        |          |a. SPIEN (LPSPI_CTL[0]) = 1 and TXEMPTY = 0.
     * |        |          |b
     * |        |          |For SPI Master mode, SPIEN (LPSPI_CTL[0]) = 1 and TXEMPTY = 1 but the current transaction is not finished yet.
     * |        |          |c. For SPI Master mode, SPIEN (LPSPI_CTL[0]) = 1 and RXONLY = 1.
     * |        |          |d
     * |        |          |For SPI Slave mode, SPIEN (LPSPI_CTL[0]) = 1 and there is serial clock input into the SPI core logic when slave select is active.
     * |        |          |e
     * |        |          |For SPI Slave mode, SPIEN (LPSPI_CTL[0]) = 1 and the transmit buffer or transmit shift register is not empty even if the slave select is inactive.
     * |        |          |Note: By applications, this SPI busy flag should be used with other status registers in LPSPI_STATUS such as TXCNT, RXCNT, TXTHIF, TXFULL, TXEMPTY, RXTHIF, RXFULL, RXEMPTY, and UNITIF
     * |        |          |Therefore the SPI transfer done events of TX/RX operations can be obtained at correct timing point.
     * |[1]     |UNITIF    |Unit Transfer Interrupt Flag
     * |        |          |0 = No transaction has been finished since this bit was cleared to 0.
     * |        |          |1 = SPI controller has finished one unit transfer.
     * |        |          |Note: This bit will be cleared by writing 1 to it.
     * |[2]     |SSACTIF   |Slave Select Active Interrupt Flag
     * |        |          |0 = Slave select active interrupt was cleared or not occurred.
     * |        |          |1 = Slave select active interrupt event occurred.
     * |        |          |Note: Only available in Slave mode. This bit will be cleared by writing 1 to it.
     * |[3]     |SSINAIF   |Slave Select Inactive Interrupt Flag
     * |        |          |0 = Slave select inactive interrupt was cleared or not occurred.
     * |        |          |1 = Slave select inactive interrupt event occurred.
     * |        |          |Note: Only available in Slave mode. This bit will be cleared by writing 1 to it.
     * |[4]     |SSLINE    |Slave Select Line Bus Status (Read Only)
     * |        |          |0 = The slave select line status is 0.
     * |        |          |1 = The slave select line status is 1.
     * |        |          |Note: This bit is only available in Slave mode
     * |        |          |If SSACTPOL (LPSPI_SSCTL[2]) is set 0, and the SSLINE is 1, the SPI slave select is in inactive status.
     * |[6]     |SLVBEIF   |Slave Mode Bit Count Error Interrupt Flag
     * |        |          |In Slave mode, when the slave select line goes to inactive state, if bit counter is mismatch with DWIDTH, this interrupt flag will be set to 1.
     * |        |          |0 = No Slave mode bit count error event.
     * |        |          |1 = Slave mode bit count error event occurred.
     * |        |          |Note: If the slave select active but there is no any bus clock input, the SLVBEIF also active when the slave select goes to inactive state
     * |        |          |This bit will be cleared by writing 1 to it.
     * |[7]     |SLVURIF   |Slave Mode TX Under Run Interrupt Flag
     * |        |          |In Slave mode, if TX underflow event occurs and the slave select line goes to inactive state, this interrupt flag will be set to 1.
     * |        |          |0 = No Slave TX under run event.
     * |        |          |1 = Slave TX under run event occurred.
     * |        |          |Note: This bit will be cleared by writing 1 to it.
     * |[8]     |RXEMPTY   |Receive FIFO Buffer Empty Indicator (Read Only)
     * |        |          |0 = Receive FIFO buffer is not empty.
     * |        |          |1 = Receive FIFO buffer is empty.
     * |[9]     |RXFULL    |Receive FIFO Buffer Full Indicator (Read Only)
     * |        |          |0 = Receive FIFO buffer is not full.
     * |        |          |1 = Receive FIFO buffer is full.
     * |[10]    |RXTHIF    |Receive FIFO Threshold Interrupt Flag (Read Only)
     * |        |          |0 = The valid data count within the receive FIFO buffer is smaller than or equal to the setting value of RXTH.
     * |        |          |1 = The valid data count within the receive FIFO buffer is larger than the setting value of RXTH.
     * |[11]    |RXOVIF    |Receive FIFO Overrun Interrupt Flag
     * |        |          |When the receive FIFO buffer is full, the follow-up data will be dropped and this bit will be set to 1.
     * |        |          |0 = No FIFO is overrun.
     * |        |          |1 = Receive FIFO is overrun.
     * |        |          |Note: This bit will be cleared by writing 1 to it.
     * |[12]    |RXTOIF    |Receive Time-out Interrupt Flag
     * |        |          |0 = No receive FIFO time-out event.
     * |        |          |1 = Receive FIFO buffer is not empty and no read operation on receive FIFO buffer over 64 SPI peripheral clock periods in Master mode or over 576 SPI peripheral clock periods in Slave mode
     * |        |          |When the received FIFO buffer is read by software, the time-out status will be cleared automatically.
     * |        |          |Note: This bit will be cleared by writing 1 to it.
     * |[15]    |SPIENSTS  |SPI Enable Status (Read Only)
     * |        |          |0 = SPI controller Disabled.
     * |        |          |1 = SPI controller Enabled.
     * |        |          |Note: The SPI peripheral clock is asynchronous with the system clock
     * |        |          |In order to make sure the SPI control logic is disabled, this bit indicates the real status of SPI controller.
     * |[16]    |TXEMPTY   |Transmit FIFO Buffer Empty Indicator (Read Only)
     * |        |          |0 = Transmit FIFO buffer is not empty.
     * |        |          |1 = Transmit FIFO buffer is empty.
     * |[17]    |TXFULL    |Transmit FIFO Buffer Full Indicator (Read Only)
     * |        |          |0 = Transmit FIFO buffer is not full.
     * |        |          |1 = Transmit FIFO buffer is full.
     * |[18]    |TXTHIF    |Transmit FIFO Threshold Interrupt Flag (Read Only)
     * |        |          |0 = The valid data count within the transmit FIFO buffer is larger than the setting value of TXTH.
     * |        |          |1 = The valid data count within the transmit FIFO buffer is less than or equal to the setting value of TXTH.
     * |[19]    |TXUFIF    |TX Underflow Interrupt Flag
     * |        |          |When the TX underflow event occurs, this bit will be set to 1, the state of data output pin depends on the setting of TXUFPOL.
     * |        |          |0 = No effect.
     * |        |          |1 = No data in Transmit FIFO and TX shift register when the slave selection signal is active.
     * |        |          |Note 1: This bit will be cleared by writing 1 to it.
     * |        |          |Note 2: If reset slaveu2019s transmission circuit when slave selection signal is active, this flag will be set to 1 after 2 peripheral clock cycles + 3 system clock cycles since the reset operation is done.
     * |[23]    |TXRXRST   |TX or RX Reset Status (Read Only)
     * |        |          |0 = The reset function of TXRST or RXRST is done.
     * |        |          |1 = Doing the reset function of TXRST or RXRST.
     * |        |          |Note: Both the reset operations of TXRST and RXRST need 3 system clock cycles + 2 peripheral clock cycles
     * |        |          |User can check the status of this bit to monitor the reset function is doing or done.
     * |[27:24] |RXCNT     |Receive FIFO Data Count (Read Only)
     * |        |          |This bit field indicates the valid data count of receive FIFO buffer.
     * |[31:28] |TXCNT     |Transmit FIFO Data Count (Read Only)
     * |        |          |This bit field indicates the valid data count of transmit FIFO buffer.
     * @var LPSPI_T::STATUS2
     * Offset: 0x18  SPI Status2 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[29:24] |SLVBENUM  |Effective Bit Number of Uncompleted RX data
     * |        |          |This status register indicates that effective bit number of uncompleted RX data when SLVBERX (LPSPI_FIFOCTL[10]) is enabled and RX bit count error event happen in SPI Slave mode.
     * |        |          |This status register will be fixed to 0x0 when SLVBERX (LPSPI_FIFOCTL[10]) is disabled.
     * |        |          |Note: This register will be cleared to 0x0 when user writes 0x1 to SLVBEIF (LPSPI_STATUS[6]).
     * |        |          |Note: This bit is for Slave Mode only.
     * @var LPSPI_T::TX
     * Offset: 0x20  SPI Data Transmit Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |TX        |Data Transmit Register
     * |        |          |The data transmit registers pass through the transmitted data into the 4-level transmit FIFO buffers
     * |        |          |The number of valid bits depends on the setting of DWIDTH (LPSPI_CTL[12:8]) in SPI mode.
     * |        |          |In SPI mode, if DWIDTH is set to 0x08, the bits TX[7:0] will be transmitted
     * |        |          |If DWIDTH is set to 0x00, the SPI controller will perform a 32-bit transfer.
     * |        |          |Note: In Master mode, SPI controller will start to transfer the SPI bus clock after 1 APB clock and 6 peripheral clock cycles after user writes to this register.
     * @var LPSPI_T::RX
     * Offset: 0x30  SPI Data Receive Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RX        |Data Receive Register (Read Only)
     * |        |          |There are 4-level FIFO buffers in this controller
     * |        |          |The data receive register holds the data received from SPI data input pin
     * |        |          |If the RXEMPTY (LPSPI_STATUS[8]) is not set to 1, the receive FIFO buffers can be accessed through software by reading this register.
     * @var LPSPI_T::AUTOCTL
     * Offset: 0x50  LPSPI Automatic Operation Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |TRIGSEL   |Automatic Operation Trigger Source Select
     * |        |          |0000 = Low Power Auto-operation Trigger Source from LPTMR0.
     * |        |          |0001 = Low Power Auto-operation Trigger Source from LPTMR1.
     * |        |          |0010 = Low Power Auto-operation Trigger Source from TTMR0.
     * |        |          |0011 = Low Power Auto-operation Trigger Source from TTMR1.
     * |        |          |0100 = Low Power Auto-operation Trigger Source from LPGPIO0.
     * |        |          |0101 = Low Power Auto-operation Trigger Source from LPGPIO1.
     * |        |          |0110 = Low Power Auto-operation Trigger Source from LPGPIO2.
     * |        |          |0111 = Low Power Auto-operation Trigger Source from LPGPIO3.
     * |        |          |1000~1111 = Reserved.
     * |[4]     |TRIGEN    |Automatic Operation Trigger Enable Bit
     * |        |          |In Auomatic Operation Master mode, the automatic operation of LPSPI will be triggered by a event sent from the trigger source selected by TRIGSEL[3:0] after this bit is set to 1.
     * |        |          |0 = LPSPI Automatic Operation Trigger disabled.
     * |        |          |1 = LPSPI Automatic Operation Trigger enabled.
     * |[5]     |CNTIEN    |TCNT Count Match Interrupt Enable
     * |        |          |0 = TCNT count match interrupt disabled.
     * |        |          |1 = TCNT count match interrupt enabled.
     * |[6]     |FULLRXEN  |Full RX Data Acception Enable Bit
     * |        |          |In Auomatic Operation Master mode and LPSPI operates in full-duplex mode, the RX data will be saved from RX buffer by LPPDMA when TXPDMAEN is 1 after this bit is set to 1.
     * |[7]     |SSWKEN    |Slave Select Wake Up Enable Bit
     * |        |          |In Auomatic Operation Slave mode, the CPU will be woken up by SS falling or rising edge in NPDx mode after this bit is set to 1.
     * |        |          |0 = LPSPI SS wake-up function disabled.
     * |        |          |1 = LPSPI SS wake-up function enabled.
     * |[23:16] |TCNT      |Auomatic Operation RX Transfer Count
     * |        |          |In Auomatic Operation Master mode, TCNT represents the required number of RX data received from external SPI slave device while TXPDMAEN is 0 and RXPDMAEN is 1
     * |        |          |The maximum transfer count is 255.
     * @var LPSPI_T::AUTOSTS
     * Offset: 0x54  LPSPI Automatic Operation Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTIF     |TCNT Count Match Interrupt Flag
     * |        |          |When the received data count matchs the setting value of TCNT, this bit will be set to 1.
     * |        |          |0 = The received data count is less than the setting value of TCNT.
     * |        |          |1 = The received data count is equal than the setting value of TCNT.
     * |[1]     |SSWKF     |Slave Select Wake Up Flag
     * |        |          |In Slave mode, when chip is woken up from NPDx mode by LPSPI, this bit is set to 1
     * |        |          |Software can write 1 to clear this bit.
     * |[2]     |AOBUSY    |Automatic Operation Busy Flag
     * |        |          |When there were one or more requests from trigger sources during auto operation, this bit will be set to 1
     * |        |          |Software can write 1 to clear this bit.
     */
    __IO uint32_t CTL;                   /*!< [0x0000] SPI Control Register                                             */
    __IO uint32_t CLKDIV;                /*!< [0x0004] SPI Clock Divider Register                                       */
    __IO uint32_t SSCTL;                 /*!< [0x0008] SPI Slave Select Control Register                                */
    __IO uint32_t PDMACTL;               /*!< [0x000c] SPI PDMA Control Register                                        */
    __IO uint32_t FIFOCTL;               /*!< [0x0010] SPI FIFO Control Register                                        */
    __IO uint32_t STATUS;                /*!< [0x0014] SPI Status Register                                              */
    __I  uint32_t STATUS2;               /*!< [0x0018] SPI Status2 Register                                             */
    __I  uint32_t RESERVE0[1];
    __O  uint32_t TX;                    /*!< [0x0020] SPI Data Transmit Register                                       */
    __I  uint32_t RESERVE1[3];
    __I  uint32_t RX;                    /*!< [0x0030] SPI Data Receive Register                                        */
    __I  uint32_t RESERVE2[5];
    __IO uint32_t INTERNAL;
    __I  uint32_t RESERVE3[1];
    __IO uint32_t AUTOCTL;               /*!< [0x0050] LPSPI Automatic Operation Control Register                       */
    __IO uint32_t AUTOSTS;               /*!< [0x0054] LPSPI Automatic Operation Status Register                        */

} LPSPI_T;

/**
    @addtogroup LPSPI_CONST LPSPI Bit Field Definition
    Constant Definitions for LPSPI Controller
@{ */

#define LPSPI_CTL_SPIEN_Pos              (0)                                               /*!< LPSPI_T::CTL: SPIEN Position           */
#define LPSPI_CTL_SPIEN_Msk              (0x1ul << LPSPI_CTL_SPIEN_Pos)                    /*!< LPSPI_T::CTL: SPIEN Mask               */

#define LPSPI_CTL_RXNEG_Pos              (1)                                               /*!< LPSPI_T::CTL: RXNEG Position           */
#define LPSPI_CTL_RXNEG_Msk              (0x1ul << LPSPI_CTL_RXNEG_Pos)                    /*!< LPSPI_T::CTL: RXNEG Mask               */

#define LPSPI_CTL_TXNEG_Pos              (2)                                               /*!< LPSPI_T::CTL: TXNEG Position           */
#define LPSPI_CTL_TXNEG_Msk              (0x1ul << LPSPI_CTL_TXNEG_Pos)                    /*!< LPSPI_T::CTL: TXNEG Mask               */

#define LPSPI_CTL_CLKPOL_Pos             (3)                                               /*!< LPSPI_T::CTL: CLKPOL Position          */
#define LPSPI_CTL_CLKPOL_Msk             (0x1ul << LPSPI_CTL_CLKPOL_Pos)                   /*!< LPSPI_T::CTL: CLKPOL Mask              */

#define LPSPI_CTL_SUSPITV_Pos            (4)                                               /*!< LPSPI_T::CTL: SUSPITV Position         */
#define LPSPI_CTL_SUSPITV_Msk            (0xful << LPSPI_CTL_SUSPITV_Pos)                  /*!< LPSPI_T::CTL: SUSPITV Mask             */

#define LPSPI_CTL_DWIDTH_Pos             (8)                                               /*!< LPSPI_T::CTL: DWIDTH Position          */
#define LPSPI_CTL_DWIDTH_Msk             (0x1ful << LPSPI_CTL_DWIDTH_Pos)                  /*!< LPSPI_T::CTL: DWIDTH Mask              */

#define LPSPI_CTL_LSB_Pos                (13)                                              /*!< LPSPI_T::CTL: LSB Position             */
#define LPSPI_CTL_LSB_Msk                (0x1ul << LPSPI_CTL_LSB_Pos)                      /*!< LPSPI_T::CTL: LSB Mask                 */

#define LPSPI_CTL_HALFDPX_Pos            (14)                                              /*!< LPSPI_T::CTL: HALFDPX Position         */
#define LPSPI_CTL_HALFDPX_Msk            (0x1ul << LPSPI_CTL_HALFDPX_Pos)                  /*!< LPSPI_T::CTL: HALFDPX Mask             */

#define LPSPI_CTL_RXONLY_Pos             (15)                                              /*!< LPSPI_T::CTL: RXONLY Position          */
#define LPSPI_CTL_RXONLY_Msk             (0x1ul << LPSPI_CTL_RXONLY_Pos)                   /*!< LPSPI_T::CTL: RXONLY Mask              */

#define LPSPI_CTL_UNITIEN_Pos            (17)                                              /*!< LPSPI_T::CTL: UNITIEN Position         */
#define LPSPI_CTL_UNITIEN_Msk            (0x1ul << LPSPI_CTL_UNITIEN_Pos)                  /*!< LPSPI_T::CTL: UNITIEN Mask             */

#define LPSPI_CTL_SLAVE_Pos              (18)                                              /*!< LPSPI_T::CTL: SLAVE Position           */
#define LPSPI_CTL_SLAVE_Msk              (0x1ul << LPSPI_CTL_SLAVE_Pos)                    /*!< LPSPI_T::CTL: SLAVE Mask               */

#define LPSPI_CTL_REORDER_Pos            (19)                                              /*!< LPSPI_T::CTL: REORDER Position         */
#define LPSPI_CTL_REORDER_Msk            (0x1ul << LPSPI_CTL_REORDER_Pos)                  /*!< LPSPI_T::CTL: REORDER Mask             */

#define LPSPI_CTL_DATDIR_Pos             (20)                                              /*!< LPSPI_T::CTL: DATDIR Position          */
#define LPSPI_CTL_DATDIR_Msk             (0x1ul << LPSPI_CTL_DATDIR_Pos)                   /*!< LPSPI_T::CTL: DATDIR Mask              */

#define LPSPI_CLKDIV_DIVIDER_Pos         (0)                                               /*!< LPSPI_T::CLKDIV: DIVIDER Position      */
#define LPSPI_CLKDIV_DIVIDER_Msk         (0x1fful << LPSPI_CLKDIV_DIVIDER_Pos)             /*!< LPSPI_T::CLKDIV: DIVIDER Mask          */

#define LPSPI_SSCTL_SS_Pos               (0)                                               /*!< LPSPI_T::SSCTL: SS Position            */
#define LPSPI_SSCTL_SS_Msk               (0x1ul << LPSPI_SSCTL_SS_Pos)                     /*!< LPSPI_T::SSCTL: SS Mask                */

#define LPSPI_SSCTL_SSACTPOL_Pos         (2)                                               /*!< LPSPI_T::SSCTL: SSACTPOL Position      */
#define LPSPI_SSCTL_SSACTPOL_Msk         (0x1ul << LPSPI_SSCTL_SSACTPOL_Pos)               /*!< LPSPI_T::SSCTL: SSACTPOL Mask          */

#define LPSPI_SSCTL_AUTOSS_Pos           (3)                                               /*!< LPSPI_T::SSCTL: AUTOSS Position        */
#define LPSPI_SSCTL_AUTOSS_Msk           (0x1ul << LPSPI_SSCTL_AUTOSS_Pos)                 /*!< LPSPI_T::SSCTL: AUTOSS Mask            */

#define LPSPI_SSCTL_SLV3WIRE_Pos         (4)                                               /*!< LPSPI_T::SSCTL: SLV3WIRE Position      */
#define LPSPI_SSCTL_SLV3WIRE_Msk         (0x1ul << LPSPI_SSCTL_SLV3WIRE_Pos)               /*!< LPSPI_T::SSCTL: SLV3WIRE Mask          */

#define LPSPI_SSCTL_SLVBEIEN_Pos         (8)                                               /*!< LPSPI_T::SSCTL: SLVBEIEN Position      */
#define LPSPI_SSCTL_SLVBEIEN_Msk         (0x1ul << LPSPI_SSCTL_SLVBEIEN_Pos)               /*!< LPSPI_T::SSCTL: SLVBEIEN Mask          */

#define LPSPI_SSCTL_SLVURIEN_Pos         (9)                                               /*!< LPSPI_T::SSCTL: SLVURIEN Position      */
#define LPSPI_SSCTL_SLVURIEN_Msk         (0x1ul << LPSPI_SSCTL_SLVURIEN_Pos)               /*!< LPSPI_T::SSCTL: SLVURIEN Mask          */

#define LPSPI_SSCTL_SSACTIEN_Pos         (12)                                              /*!< LPSPI_T::SSCTL: SSACTIEN Position      */
#define LPSPI_SSCTL_SSACTIEN_Msk         (0x1ul << LPSPI_SSCTL_SSACTIEN_Pos)               /*!< LPSPI_T::SSCTL: SSACTIEN Mask          */

#define LPSPI_SSCTL_SSINAIEN_Pos         (13)                                              /*!< LPSPI_T::SSCTL: SSINAIEN Position      */
#define LPSPI_SSCTL_SSINAIEN_Msk         (0x1ul << LPSPI_SSCTL_SSINAIEN_Pos)               /*!< LPSPI_T::SSCTL: SSINAIEN Mask          */

#define LPSPI_PDMACTL_TXPDMAEN_Pos       (0)                                               /*!< LPSPI_T::PDMACTL: TXPDMAEN Position    */
#define LPSPI_PDMACTL_TXPDMAEN_Msk       (0x1ul << LPSPI_PDMACTL_TXPDMAEN_Pos)             /*!< LPSPI_T::PDMACTL: TXPDMAEN Mask        */

#define LPSPI_PDMACTL_RXPDMAEN_Pos       (1)                                               /*!< LPSPI_T::PDMACTL: RXPDMAEN Position    */
#define LPSPI_PDMACTL_RXPDMAEN_Msk       (0x1ul << LPSPI_PDMACTL_RXPDMAEN_Pos)             /*!< LPSPI_T::PDMACTL: RXPDMAEN Mask        */

#define LPSPI_PDMACTL_PDMARST_Pos        (2)                                               /*!< LPSPI_T::PDMACTL: PDMARST Position     */
#define LPSPI_PDMACTL_PDMARST_Msk        (0x1ul << LPSPI_PDMACTL_PDMARST_Pos)              /*!< LPSPI_T::PDMACTL: PDMARST Mask         */

#define LPSPI_FIFOCTL_RXRST_Pos          (0)                                               /*!< LPSPI_T::FIFOCTL: RXRST Position       */
#define LPSPI_FIFOCTL_RXRST_Msk          (0x1ul << LPSPI_FIFOCTL_RXRST_Pos)                /*!< LPSPI_T::FIFOCTL: RXRST Mask           */

#define LPSPI_FIFOCTL_TXRST_Pos          (1)                                               /*!< LPSPI_T::FIFOCTL: TXRST Position       */
#define LPSPI_FIFOCTL_TXRST_Msk          (0x1ul << LPSPI_FIFOCTL_TXRST_Pos)                /*!< LPSPI_T::FIFOCTL: TXRST Mask           */

#define LPSPI_FIFOCTL_RXTHIEN_Pos        (2)                                               /*!< LPSPI_T::FIFOCTL: RXTHIEN Position     */
#define LPSPI_FIFOCTL_RXTHIEN_Msk        (0x1ul << LPSPI_FIFOCTL_RXTHIEN_Pos)              /*!< LPSPI_T::FIFOCTL: RXTHIEN Mask         */

#define LPSPI_FIFOCTL_TXTHIEN_Pos        (3)                                               /*!< LPSPI_T::FIFOCTL: TXTHIEN Position     */
#define LPSPI_FIFOCTL_TXTHIEN_Msk        (0x1ul << LPSPI_FIFOCTL_TXTHIEN_Pos)              /*!< LPSPI_T::FIFOCTL: TXTHIEN Mask         */

#define LPSPI_FIFOCTL_RXTOIEN_Pos        (4)                                               /*!< LPSPI_T::FIFOCTL: RXTOIEN Position     */
#define LPSPI_FIFOCTL_RXTOIEN_Msk        (0x1ul << LPSPI_FIFOCTL_RXTOIEN_Pos)              /*!< LPSPI_T::FIFOCTL: RXTOIEN Mask         */

#define LPSPI_FIFOCTL_RXOVIEN_Pos        (5)                                               /*!< LPSPI_T::FIFOCTL: RXOVIEN Position     */
#define LPSPI_FIFOCTL_RXOVIEN_Msk        (0x1ul << LPSPI_FIFOCTL_RXOVIEN_Pos)              /*!< LPSPI_T::FIFOCTL: RXOVIEN Mask         */

#define LPSPI_FIFOCTL_TXUFPOL_Pos        (6)                                               /*!< LPSPI_T::FIFOCTL: TXUFPOL Position     */
#define LPSPI_FIFOCTL_TXUFPOL_Msk        (0x1ul << LPSPI_FIFOCTL_TXUFPOL_Pos)              /*!< LPSPI_T::FIFOCTL: TXUFPOL Mask         */

#define LPSPI_FIFOCTL_TXUFIEN_Pos        (7)                                               /*!< LPSPI_T::FIFOCTL: TXUFIEN Position     */
#define LPSPI_FIFOCTL_TXUFIEN_Msk        (0x1ul << LPSPI_FIFOCTL_TXUFIEN_Pos)              /*!< LPSPI_T::FIFOCTL: TXUFIEN Mask         */

#define LPSPI_FIFOCTL_RXFBCLR_Pos        (8)                                               /*!< LPSPI_T::FIFOCTL: RXFBCLR Position     */
#define LPSPI_FIFOCTL_RXFBCLR_Msk        (0x1ul << LPSPI_FIFOCTL_RXFBCLR_Pos)              /*!< LPSPI_T::FIFOCTL: RXFBCLR Mask         */

#define LPSPI_FIFOCTL_TXFBCLR_Pos        (9)                                               /*!< LPSPI_T::FIFOCTL: TXFBCLR Position     */
#define LPSPI_FIFOCTL_TXFBCLR_Msk        (0x1ul << LPSPI_FIFOCTL_TXFBCLR_Pos)              /*!< LPSPI_T::FIFOCTL: TXFBCLR Mask         */

#define LPSPI_FIFOCTL_SLVBERX_Pos        (10)                                              /*!< LPSPI_T::FIFOCTL: SLVBERX Position     */
#define LPSPI_FIFOCTL_SLVBERX_Msk        (0x1ul << LPSPI_FIFOCTL_SLVBERX_Pos)              /*!< LPSPI_T::FIFOCTL: SLVBERX Mask         */

#define LPSPI_FIFOCTL_RXTH_Pos           (24)                                              /*!< LPSPI_T::FIFOCTL: RXTH Position        */
#define LPSPI_FIFOCTL_RXTH_Msk           (0x7ul << LPSPI_FIFOCTL_RXTH_Pos)                 /*!< LPSPI_T::FIFOCTL: RXTH Mask            */

#define LPSPI_FIFOCTL_TXTH_Pos           (28)                                              /*!< LPSPI_T::FIFOCTL: TXTH Position        */
#define LPSPI_FIFOCTL_TXTH_Msk           (0x7ul << LPSPI_FIFOCTL_TXTH_Pos)                 /*!< LPSPI_T::FIFOCTL: TXTH Mask            */

#define LPSPI_STATUS_BUSY_Pos            (0)                                               /*!< LPSPI_T::STATUS: BUSY Position         */
#define LPSPI_STATUS_BUSY_Msk            (0x1ul << LPSPI_STATUS_BUSY_Pos)                  /*!< LPSPI_T::STATUS: BUSY Mask             */

#define LPSPI_STATUS_UNITIF_Pos          (1)                                               /*!< LPSPI_T::STATUS: UNITIF Position       */
#define LPSPI_STATUS_UNITIF_Msk          (0x1ul << LPSPI_STATUS_UNITIF_Pos)                /*!< LPSPI_T::STATUS: UNITIF Mask           */

#define LPSPI_STATUS_SSACTIF_Pos         (2)                                               /*!< LPSPI_T::STATUS: SSACTIF Position      */
#define LPSPI_STATUS_SSACTIF_Msk         (0x1ul << LPSPI_STATUS_SSACTIF_Pos)               /*!< LPSPI_T::STATUS: SSACTIF Mask          */

#define LPSPI_STATUS_SSINAIF_Pos         (3)                                               /*!< LPSPI_T::STATUS: SSINAIF Position      */
#define LPSPI_STATUS_SSINAIF_Msk         (0x1ul << LPSPI_STATUS_SSINAIF_Pos)               /*!< LPSPI_T::STATUS: SSINAIF Mask          */

#define LPSPI_STATUS_SSLINE_Pos          (4)                                               /*!< LPSPI_T::STATUS: SSLINE Position       */
#define LPSPI_STATUS_SSLINE_Msk          (0x1ul << LPSPI_STATUS_SSLINE_Pos)                /*!< LPSPI_T::STATUS: SSLINE Mask           */

#define LPSPI_STATUS_SLVBEIF_Pos         (6)                                               /*!< LPSPI_T::STATUS: SLVBEIF Position      */
#define LPSPI_STATUS_SLVBEIF_Msk         (0x1ul << LPSPI_STATUS_SLVBEIF_Pos)               /*!< LPSPI_T::STATUS: SLVBEIF Mask          */

#define LPSPI_STATUS_SLVURIF_Pos         (7)                                               /*!< LPSPI_T::STATUS: SLVURIF Position      */
#define LPSPI_STATUS_SLVURIF_Msk         (0x1ul << LPSPI_STATUS_SLVURIF_Pos)               /*!< LPSPI_T::STATUS: SLVURIF Mask          */

#define LPSPI_STATUS_RXEMPTY_Pos         (8)                                               /*!< LPSPI_T::STATUS: RXEMPTY Position      */
#define LPSPI_STATUS_RXEMPTY_Msk         (0x1ul << LPSPI_STATUS_RXEMPTY_Pos)               /*!< LPSPI_T::STATUS: RXEMPTY Mask          */

#define LPSPI_STATUS_RXFULL_Pos          (9)                                               /*!< LPSPI_T::STATUS: RXFULL Position       */
#define LPSPI_STATUS_RXFULL_Msk          (0x1ul << LPSPI_STATUS_RXFULL_Pos)                /*!< LPSPI_T::STATUS: RXFULL Mask           */

#define LPSPI_STATUS_RXTHIF_Pos          (10)                                              /*!< LPSPI_T::STATUS: RXTHIF Position       */
#define LPSPI_STATUS_RXTHIF_Msk          (0x1ul << LPSPI_STATUS_RXTHIF_Pos)                /*!< LPSPI_T::STATUS: RXTHIF Mask           */

#define LPSPI_STATUS_RXOVIF_Pos          (11)                                              /*!< LPSPI_T::STATUS: RXOVIF Position       */
#define LPSPI_STATUS_RXOVIF_Msk          (0x1ul << LPSPI_STATUS_RXOVIF_Pos)                /*!< LPSPI_T::STATUS: RXOVIF Mask           */

#define LPSPI_STATUS_RXTOIF_Pos          (12)                                              /*!< LPSPI_T::STATUS: RXTOIF Position       */
#define LPSPI_STATUS_RXTOIF_Msk          (0x1ul << LPSPI_STATUS_RXTOIF_Pos)                /*!< LPSPI_T::STATUS: RXTOIF Mask           */

#define LPSPI_STATUS_SPIENSTS_Pos        (15)                                              /*!< LPSPI_T::STATUS: SPIENSTS Position     */
#define LPSPI_STATUS_SPIENSTS_Msk        (0x1ul << LPSPI_STATUS_SPIENSTS_Pos)              /*!< LPSPI_T::STATUS: SPIENSTS Mask         */

#define LPSPI_STATUS_TXEMPTY_Pos         (16)                                              /*!< LPSPI_T::STATUS: TXEMPTY Position      */
#define LPSPI_STATUS_TXEMPTY_Msk         (0x1ul << LPSPI_STATUS_TXEMPTY_Pos)               /*!< LPSPI_T::STATUS: TXEMPTY Mask          */

#define LPSPI_STATUS_TXFULL_Pos          (17)                                              /*!< LPSPI_T::STATUS: TXFULL Position       */
#define LPSPI_STATUS_TXFULL_Msk          (0x1ul << LPSPI_STATUS_TXFULL_Pos)                /*!< LPSPI_T::STATUS: TXFULL Mask           */

#define LPSPI_STATUS_TXTHIF_Pos          (18)                                              /*!< LPSPI_T::STATUS: TXTHIF Position       */
#define LPSPI_STATUS_TXTHIF_Msk          (0x1ul << LPSPI_STATUS_TXTHIF_Pos)                /*!< LPSPI_T::STATUS: TXTHIF Mask           */

#define LPSPI_STATUS_TXUFIF_Pos          (19)                                              /*!< LPSPI_T::STATUS: TXUFIF Position       */
#define LPSPI_STATUS_TXUFIF_Msk          (0x1ul << LPSPI_STATUS_TXUFIF_Pos)                /*!< LPSPI_T::STATUS: TXUFIF Mask           */

#define LPSPI_STATUS_TXRXRST_Pos         (23)                                              /*!< LPSPI_T::STATUS: TXRXRST Position      */
#define LPSPI_STATUS_TXRXRST_Msk         (0x1ul << LPSPI_STATUS_TXRXRST_Pos)               /*!< LPSPI_T::STATUS: TXRXRST Mask          */

#define LPSPI_STATUS_RXCNT_Pos           (24)                                              /*!< LPSPI_T::STATUS: RXCNT Position        */
#define LPSPI_STATUS_RXCNT_Msk           (0xful << LPSPI_STATUS_RXCNT_Pos)                 /*!< LPSPI_T::STATUS: RXCNT Mask            */

#define LPSPI_STATUS_TXCNT_Pos           (28)                                              /*!< LPSPI_T::STATUS: TXCNT Position        */
#define LPSPI_STATUS_TXCNT_Msk           (0xful << LPSPI_STATUS_TXCNT_Pos)                 /*!< LPSPI_T::STATUS: TXCNT Mask            */

#define LPSPI_STATUS2_SLVBENUM_Pos       (24)                                              /*!< LPSPI_T::STATUS2: SLVBENUM Position    */
#define LPSPI_STATUS2_SLVBENUM_Msk       (0x3ful << LPSPI_STATUS2_SLVBENUM_Pos)            /*!< LPSPI_T::STATUS2: SLVBENUM Mask        */

#define LPSPI_TX_TX_Pos                  (0)                                               /*!< LPSPI_T::TX: TX Position               */
#define LPSPI_TX_TX_Msk                  (0xfffffffful << LPSPI_TX_TX_Pos)                 /*!< LPSPI_T::TX: TX Mask                   */

#define LPSPI_RX_RX_Pos                  (0)                                               /*!< LPSPI_T::RX: RX Position               */
#define LPSPI_RX_RX_Msk                  (0xfffffffful << LPSPI_RX_RX_Pos)                 /*!< LPSPI_T::RX: RX Mask                   */

#define LPSPI_AUTOCTL_TRIGSEL_Pos        (0)                                               /*!< LPSPI_T::AUTOCTL: TRIGSEL Position     */
#define LPSPI_AUTOCTL_TRIGSEL_Msk        (0xful << LPSPI_AUTOCTL_TRIGSEL_Pos)              /*!< LPSPI_T::AUTOCTL: TRIGSEL Mask         */

#define LPSPI_AUTOCTL_TRIGEN_Pos         (4)                                               /*!< LPSPI_T::AUTOCTL: TRIGEN Position      */
#define LPSPI_AUTOCTL_TRIGEN_Msk         (0x1ul << LPSPI_AUTOCTL_TRIGEN_Pos)               /*!< LPSPI_T::AUTOCTL: TRIGEN Mask          */

#define LPSPI_AUTOCTL_CNTIEN_Pos         (5)                                               /*!< LPSPI_T::AUTOCTL: CNTIEN Position      */
#define LPSPI_AUTOCTL_CNTIEN_Msk         (0x1ul << LPSPI_AUTOCTL_CNTIEN_Pos)               /*!< LPSPI_T::AUTOCTL: CNTIEN Mask          */

#define LPSPI_AUTOCTL_FULLRXEN_Pos       (6)                                               /*!< LPSPI_T::AUTOCTL: FULLRXEN Position    */
#define LPSPI_AUTOCTL_FULLRXEN_Msk       (0x1ul << LPSPI_AUTOCTL_FULLRXEN_Pos)             /*!< LPSPI_T::AUTOCTL: FULLRXEN Mask        */

#define LPSPI_AUTOCTL_SSWKEN_Pos         (7)                                               /*!< LPSPI_T::AUTOCTL: SSWKEN Position      */
#define LPSPI_AUTOCTL_SSWKEN_Msk         (0x1ul << LPSPI_AUTOCTL_SSWKEN_Pos)               /*!< LPSPI_T::AUTOCTL: SSWKEN Mask          */

#define LPSPI_AUTOCTL_AUTOEN_Pos         (8)                                               /*!< LPSPI_T::AUTOCTL: AUTOEN Position      */
#define LPSPI_AUTOCTL_AUTOEN_Msk         (0x1ul << LPSPI_AUTOCTL_AUTOEN_Pos)               /*!< LPSPI_T::AUTOCTL: AUTOEN Mask          */

#define LPSPI_AUTOCTL_SWTRG_Pos          (9)                                               /*!< LPSPI_T::AUTOCTL: SWTRG Position      */
#define LPSPI_AUTOCTL_SWTRG_Msk          (0x1ul << LPSPI_AUTOCTL_SWTRG_Pos)                /*!< LPSPI_T::AUTOCTL: SWTRG Mask          */

#define LPSPI_AUTOCTL_CNTWKEN_Pos        (10)                                              /*!< LPSPI_T::AUTOCTL: CNTWKEN Position      */
#define LPSPI_AUTOCTL_CNTWKEN_Msk        (0x1ul << LPSPI_AUTOCTL_CNTWKEN_Pos)              /*!< LPSPI_T::AUTOCTL: CNTWKEN Mask          */

#define LPSPI_AUTOCTL_TCNT_Pos           (16)                                              /*!< LPSPI_T::AUTOCTL: TCNT Position        */
#define LPSPI_AUTOCTL_TCNT_Msk           (0xfful << LPSPI_AUTOCTL_TCNT_Pos)                /*!< LPSPI_T::AUTOCTL: TCNT Mask            */

#define LPSPI_AUTOSTS_CNTIF_Pos          (0)                                               /*!< LPSPI_T::AUTOSTS: CNTIF Position       */
#define LPSPI_AUTOSTS_CNTIF_Msk          (0x1ul << LPSPI_AUTOSTS_CNTIF_Pos)                /*!< LPSPI_T::AUTOSTS: CNTIF Mask           */

#define LPSPI_AUTOSTS_SSWKF_Pos          (1)                                               /*!< LPSPI_T::AUTOSTS: SSWKF Position       */
#define LPSPI_AUTOSTS_SSWKF_Msk          (0x1ul << LPSPI_AUTOSTS_SSWKF_Pos)                /*!< LPSPI_T::AUTOSTS: SSWKF Mask           */

#define LPSPI_AUTOSTS_AOBUSY_Pos         (2)                                               /*!< LPSPI_T::AUTOSTS: AOBUSY Position      */
#define LPSPI_AUTOSTS_AOBUSY_Msk         (0x1ul << LPSPI_AUTOSTS_AOBUSY_Pos)               /*!< LPSPI_T::AUTOSTS: AOBUSY Mask          */

#define LPSPI_AUTOSTS_CNTWKF_Pos         (3)                                               /*!< LPSPI_T::AUTOSTS: CNTWKF Position       */
#define LPSPI_AUTOSTS_CNTWKF_Msk         (0x1ul << LPSPI_AUTOSTS_CNTWKF_Pos)                /*!< LPSPI_T::AUTOSTS: CNTWKF Mask           */

/** @} LPSPI_CONST */
/** @} end of LPSPI register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __LPSPI_REG_H__ */
