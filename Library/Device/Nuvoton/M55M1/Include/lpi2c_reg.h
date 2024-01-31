/***************************************************************************//**
 * @file     lpi2c_reg.h
 * @version  V1.00
 * @brief    LPI2C register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __LPI2C_REG_H__
#define __LPI2C_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup LPI2C Low Power Inter-IC Bus Controller (LPI2C)
    Memory Mapped Structure for LPI2C Controller
    @{
*/

typedef struct
{
    /**
     * @var LPI2C_T::CTL0
     * Offset: 0x00  LPI2C Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2]     |AA        |Assert Acknowledge Control
     * |        |          |When AA =1 prior to address or data is received, an acknowledged (low level to SDA) will be returned during the acknowledge clock pulse on the SCL line when 1.) A slave is acknowledging the address sent from master, 2.) The receiver devices are acknowledging the data sent by transmitter
     * |        |          |When AA=0 prior to address or data received, a Not acknowledged (high level to SDA) will be returned during the acknowledge clock pulse on the SCL line
     * |[3]     |SI        |LPI2C Interrupt Flag
     * |        |          |When a new LPI2C state is present in the LPI2C_STATUS0 register, the SI flag is set by hardware
     * |        |          |If bit INTEN (LPI2C_CTL0 [7]) is set, the LPI2C interrupt is requested
     * |        |          |SI must be cleared by software
     * |        |          |Clear SI by writing 1 to this bit.
     * |[4]     |STO       |LPI2C STOP Control
     * |        |          |In Master mode, setting STO to transmit a STOP condition to bus then LPI2C will check the bus condition if a STOP condition is detected
     * |        |          |This bit will be cleared by hardware automatically.
     * |[5]     |STA       |LPI2C START Control
     * |        |          |Setting STA to logic 1 to enter Master mode, the LPI2C hardware sends a START or repeat START condition to bus when the bus is free.
     * |[6]     |LPI2CEN   |LPI2C Controller Enable Bit
     * |        |          |Set to enable LPI2C serial function controller
     * |        |          |When LPI2CEN=1 the LPI2C serial function enable
     * |        |          |The multi-function pin function must set to SDA, and SCL of LPI2C function first.
     * |        |          |0 = LPI2C controller Disabled.
     * |        |          |1 = LPI2C controller Enabled.
     * |[7]     |INTEN     |Enable Interrupt
     * |        |          |0 = LPI2C interrupt Disabled.
     * |        |          |1 = LPI2C interrupt Enabled.
     * @var LPI2C_T::ADDR0
     * Offset: 0x04  LPI2C Slave Address Register0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call function Disabled.
     * |        |          |1 = General Call function Enabled.
     * |[7:1]   |ADDR      |LPI2C Address
     * |        |          |The content of this register is irrelevant when LPI2C is in Master mode
     * |        |          |In the slave mode, the seven most significant bits must be loaded with the chip's own address
     * |        |          |The LPI2C hardware will react if either of the address is matched.
     * |        |          |Note: When software set 7'h000, the address cannot be used.
     * @var LPI2C_T::DAT
     * Offset: 0x08  LPI2C Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DAT       |LPI2C Data
     * |        |          |Bit [7:0] is located with the 8-bit transferred/received data of LPI2C serial port.
     * @var LPI2C_T::STATUS0
     * Offset: 0x0C  LPI2C Status Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |STATUS    |LPI2C Status
     * |        |          |The three least significant bits are always 0
     * |        |          |The five most significant bits contain the status code
     * |        |          |There are 28 possible status codes
     * |        |          |When the content of LPI2C_STATUS0 is F8H, no serial interrupt is requested
     * |        |          |Others LPI2C_STATUS0 values correspond to defined LPI2C states
     * |        |          |When each of these states is entered, a status interrupt is requested (SI = 1)
     * |        |          |A valid status code is present in LPI2C_STATUS0 one cycle after SI is set by hardware and is still present one cycle after SI has been reset by software
     * |        |          |In addition, states 00H stands for a Bus Error
     * |        |          |A Bus Error occurs when a START or STOP condition is present at an illegal position in the formation frame
     * |        |          |Example of illegal position are during the serial transfer of an address byte, a data byte or an acknowledge bit.
     * @var LPI2C_T::CLKDIV
     * Offset: 0x10  LPI2C Clock Divided Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |DIVIDER   |LPI2C Clock Divider
     * |        |          |Indicates the LPI2C clock rate: Data Baud Rate of LPI2C = (system clock) / (4x (DIVDER+1)).
     * |        |          |Note: The minimum value of DIVIDER is 4.
     * |[15:12] |NFCNT     |Noise Filter Count
     * |        |          |The register bits control the input filter width.
     * |        |          |If the pulse width is narrower than the setting((3+N)*PCLK), it will be ignored.
     * |        |          |0 = Filter width 3*PCLK
     * |        |          |1 = Filter width 4*PCLK
     * |        |          |N = Filter width (3+N)*PCKL
     * |        |          |Note: Filter width Min :3*PCLK, Max : 18*PCLK
     * @var LPI2C_T::TOCTL
     * Offset: 0x14  LPI2C Time-out Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TOIF      |Time-out Flag
     * |        |          |This bit is set by hardware when LPI2C time-out happened and it can interrupt CPU if LPI2C interrupt enable bit (INTEN) is set to 1.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[1]     |TOCDIV4   |Time-out Counter Input Clock Divided by 4
     * |        |          |When enabled, the time-out period is extended 4 times.
     * |        |          |0 = Time-out period is extend 4 times Disabled.
     * |        |          |1 = Time-out period is extend 4 times Enabled.
     * |[2]     |TOCEN     |Time-out Counter Enable Bit
     * |        |          |When enabled, the 14-bit time-out counter will start counting when SI is cleared
     * |        |          |Setting flag SI to '1' will reset counter and re-start up counting after SI is cleared.
     * |        |          |0 = Time-out counter Disabled.
     * |        |          |1 = Time-out counter Enabled.
     * @var LPI2C_T::ADDR1
     * Offset: 0x18  LPI2C Slave Address Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call function Disabled.
     * |        |          |1 = General Call function Enabled.
     * |[7:1]   |ADDR      |LPI2C Address
     * |        |          |The content of this register is irrelevant when LPI2C is in Master mode
     * |        |          |In the slave mode, the seven most significant bits must be loaded with the chip's own address
     * |        |          |The LPI2C hardware will react if either of the address is matched.
     * |        |          |Note: When software set 7'h000, the address cannot be used.
     * @var LPI2C_T::ADDR2
     * Offset: 0x1C  LPI2C Slave Address Register2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call function Disabled.
     * |        |          |1 = General Call function Enabled.
     * |[7:1]   |ADDR      |LPI2C Address
     * |        |          |The content of this register is irrelevant when LPI2C is in Master mode
     * |        |          |In the slave mode, the seven most significant bits must be loaded with the chip's own address
     * |        |          |The LPI2C hardware will react if either of the address is matched.
     * |        |          |Note: When software set 7'h000, the address cannot be used.
     * @var LPI2C_T::ADDR3
     * Offset: 0x20  LPI2C Slave Address Register3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call function Disabled.
     * |        |          |1 = General Call function Enabled.
     * |[7:1]   |ADDR      |LPI2C Address
     * |        |          |The content of this register is irrelevant when LPI2C is in Master mode
     * |        |          |In the slave mode, the seven most significant bits must be loaded with the chip's own address
     * |        |          |The LPI2C hardware will react if either of the address is matched.
     * |        |          |Note: When software set 7'h000, the address cannot be used.
     * @var LPI2C_T::ADDRMSK0
     * Offset: 0x24  LPI2C Slave Address Mask Register0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |ADDRMSK   |LPI2C Address Mask
     * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address register.).
     * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
     * |        |          |LPI2C bus controllers support multiple address recognition with four address mask register
     * |        |          |When the bit in the address mask register is set to one, it means the received corresponding address bit is don't-care
     * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact the same as address register.
     * @var LPI2C_T::ADDRMSK1
     * Offset: 0x28  LPI2C Slave Address Mask Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |ADDRMSK   |LPI2C Address Mask
     * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address register.).
     * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
     * |        |          |LPI2C bus controllers support multiple address recognition with four address mask register
     * |        |          |When the bit in the address mask register is set to one, it means the received corresponding address bit is don't-care
     * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact the same as address register.
     * @var LPI2C_T::ADDRMSK2
     * Offset: 0x2C  LPI2C Slave Address Mask Register2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |ADDRMSK   |LPI2C Address Mask
     * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address register.).
     * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
     * |        |          |LPI2C bus controllers support multiple address recognition with four address mask register
     * |        |          |When the bit in the address mask register is set to one, it means the received corresponding address bit is don't-care
     * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact the same as address register.
     * @var LPI2C_T::ADDRMSK3
     * Offset: 0x30  LPI2C Slave Address Mask Register3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |ADDRMSK   |LPI2C Address Mask
     * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address register.).
     * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
     * |        |          |LPI2C bus controllers support multiple address recognition with four address mask register
     * |        |          |When the bit in the address mask register is set to one, it means the received corresponding address bit is don't-care
     * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact the same as address register.
     * @var LPI2C_T::WKCTL
     * Offset: 0x3C  LPI2C Wake-up Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKEN      |LPI2C Wake-up Enable Bit
     * |        |          |0 = LPI2C wake-up function Disabled.
     * |        |          |1 = LPI2C wake-up function Enabled.
     * |        |          |Note: This is for slave address match wakeup.
     * |[7]     |NHDBUSEN  |LPI2C No Hold BUS Enable Bit
     * |        |          |0 = LPI2C hold bus after wake-up.
     * |        |          |1= LPI2C don't hold bus after wake-up.
     * |        |          |Note: LPI2C could respond when WKIF event is not clear, it may cause error data transmitted or received
     * |        |          |If data transmitted or received when WKIF event is not clear, user must reset LPI2C controller and execute the original operation again.
     * @var LPI2C_T::WKSTS
     * Offset: 0x40  LPI2C Wake-up Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKIF      |LPI2C Wake-up Flag
     * |        |          |When chip is woken up from Power-down mode by LPI2C, this bit is set to 1
     * |        |          |Software can write 1 to clear this bit.
     * |[1]     |WKAKDONE  |Wakeup Address Frame Acknowledge Bit Done
     * |        |          |0 = The ACK bit cycle of address match frame isn't done.
     * |        |          |1 = The ACK bit cycle of address match frame is done in power-down.
     * |        |          |Note: This bit cannot release WKIF. Software can write 1 to clear this bit.
     * |[2]     |WRSTSWK   |Read/Write Status Bit in Address Wakeup Frame
     * |        |          |0 = Write command be record on the address match wakeup frame.
     * |        |          |1 = Read command be record on the address match wakeup frame.
     * |        |          |Note: This bit will be cleared when software can write 1 to WKAKDONE bit.
     * @var LPI2C_T::CTL1
     * Offset: 0x44  LPI2C Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TXPDMAEN  |LPPDMA Transmit Channel Available
     * |        |          |0 = Transmit LPPDMA function Disabled.
     * |        |          |1 = Transmit LPPDMA function Enabled.
     * |[1]     |RXPDMAEN  |LPPDMA Receive Channel Available
     * |        |          |0 = Receive LPPDMA function Disabled.
     * |        |          |1 = Receive LPPDMA function Enabled.
     * |[2]     |PDMARST   |LPPDMA Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Reset the LPI2C request to LPPDMA.
     * |[8]     |PDMASTR   |LPPDMA Stretch Bit
     * |        |          |0 = LPI2C send STOP automatically after LPPDMA transfer done. (only master TX)
     * |        |          |1 = LPI2C SCL bus is stretched by hardware after LPPDMA transfer done if the SI is not cleared
     * |        |          |(only master TX)
     * |[10]    |SWITCHEN  |SCL And SDA Pin Switch Enable Bit
     * |        |          |0 = LPI&sup2;C use original pin configuration.
     * |        |          |1 = LPI&sup2;C switch SCL and SDA pin configuration.
     * |        |          |Note: Original pin configuration table is shown in Basic Configuration chapter.
     * @var LPI2C_T::STATUS1
     * Offset: 0x48  LPI2C Status Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADMAT0    |LPI2C Address 0 Match Status
     * |        |          |When address 0 is matched, hardware will inform which address used
     * |        |          |This bit will set to 1, and software can write 1 to clear this bit.
     * |[1]     |ADMAT1    |LPI2C Address 1 Match Status
     * |        |          |When address 1 is matched, hardware will inform which address used
     * |        |          |This bit will set to 1, and software can write 1 to clear this bit.
     * |[2]     |ADMAT2    |LPI2C Address 2 Match Status
     * |        |          |When address 2 is matched, hardware will inform which address used
     * |        |          |This bit will set to 1, and software can write 1 to clear this bit.
     * |[3]     |ADMAT3    |LPI2C Address 3 Match Status
     * |        |          |When address 3 is matched, hardware will inform which address used
     * |        |          |This bit will set to 1, and software can write 1 to clear this bit.
     * |[8]     |ONBUSY    |On Bus Busy (Read Only)
     * |        |          |Indicates that a communication is in progress on the bus
     * |        |          |It is set by hardware when a START condition is detected
     * |        |          |It is cleared by hardware when a STOP condition is detected.
     * |        |          |0 = The bus is IDLE (both SCL and SDA High).
     * |        |          |1 = The bus is busy.
     * @var LPI2C_T::TMCTL
     * Offset: 0x4C  LPI2C Timing Configure Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8:0]   |STCTL     |Setup Time Configure Control
     * |        |          |This field is used to generate a delay timing between SDA falling edge and SCL rising edge in transmission mode.
     * |        |          |The delay setup time is numbers of peripheral clock = STCTL x PCLK.
     * |        |          |Note: Setup time setting should not make SCL output less than three PCLKs.
     * |[24:16] |HTCTL     |Hold Time Configure Control
     * |        |          |This field is used to generate the delay timing between SCL falling edge and SDA rising edge in transmission mode.
     * |        |          |The delay hold time is numbers of peripheral clock = HTCTL x PCLK.
     * @var LPI2C_T::AUTOCTL
     * Offset: 0x70  LPI2C Automatic Operation Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |TGSRCSEL  |Low Power Auto-operation Trigger Source Select
     * |        |          |0000 = Low Power Auto-operation Trigger Source from LPTMR0.
     * |        |          |0001 = Low Power Auto-operation Trigger Source from LPTMR1.
     * |        |          |0010 = Low Power Auto-operation Trigger Source from TTMR0.
     * |        |          |0011 = Low Power Auto-operation Trigger Source from TTMR1.
     * |        |          |0100 = Low Power Auto-operation Trigger Source from WKIOA0(GPA0).
     * |        |          |0101 = Low Power Auto-operation Trigger Source from WKIOB0(GPB0).
     * |        |          |0110 = Low Power Auto-operation Trigger Source from WKIOC0(GPC0).
     * |        |          |0111 = Low Power Auto-operation Trigger Source from WKIOD0(GPD0).
     * |        |          |1000 = Low Power Auto-operation Trigger Source from Software setting SWTRG(LPI2C_AUTOCTL[31]) to 1.
     * |        |          |1001~1111 = Reserved.
     * |[4]     |TRGEN     |Trigger Source Enable bit
     * |        |          |0 = LPI2C trigger source disable.
     * |        |          |1 = LPI2C trigger source enable.
     * |[5]     |TXWKEN    |TX Transfer Count Match Wakeup Enable bit
     * |        |          |0 = Power off after TX transfer count matches TXCNT.
     * |        |          |1 = Wakeup after TX transfer count matches TXCNT.
     * |[6]     |RXWKEN    |RX Transfer Count Match Wakeup Enable bit
     * |        |          |0 = Power off after RX transfer count matches RXCNT.
     * |        |          |1 = Wakeup after RX transfer count matches RXCNT.
     * |[7]     |NACKWKEN  |Receive Slave NACK Wakeup Enable bit
     * |        |          |0 = Stop after receiving NACK.
     * |        |          |1 = Wakeup after receiving NACK.
     * |[10:8]  |AUTOMODE  |Auto-operation Mode Select
     * |        |          |000 = No auto-operation.
     * |        |          |001 = Auto TXPDMA transfer mode.
     * |        |          |010 = Auto RXPDMA transfer mode.
     * |        |          |011 = Random Read mode, repeat start between TX/RX.
     * |        |          |100 = Random Read mode, stop and start between TX/RX.
     * |        |          |Others = Reserved.
     * |[31]    |SWTRG     |Auto-operation Mode Software Trigger Bit
     * |        |          |0 = No operation.
     * |        |          |1 = Software trigger auto-operattion.
     * |        |          |Note: TGSRCSEL(LPI2C_AUTOCTL[3:0]) need to be set to 0x8 first
     * @var LPI2C_T::AUTOSTS
     * Offset: 0x74  LPI2C Automatic Operation Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TXWKF     |TX Transfer Count Match Wakeup Flag
     * |        |          |When chip is woken up from Power-down mode by LPI2C TX transfer count match, this bit is set to 1
     * |        |          |Software can write 1 to clear this bit.
     * |[1]     |RXWKF     |RX Transfer Count Match Wakeup Flag
     * |        |          |When chip is woken up from Power-down mode by LPI2C RX transfer count match, this bit is set to 1
     * |        |          |Software can write 1 to clear this bit.
     * |[2]     |ERRORWKF  |Error Condition Wakeup Flag
     * |        |          |When chip is woken up from Power-down mode by LPI2C receiving NACK from slave, running into bus error or arbitration lost, this bit is set to 1
     * |        |          |Software can write 1 to clear this bit.
     * |[8]     |TXFINISH  |Automatic Operation TX Finish Flag
     * |        |          |AOFINISH is set when finishing one round of auto operation(Automode = 1), but it will not issue interrupt
     * |        |          |Software can write 1 to clear this bit.
     * |[9]     |RXFINISH  |Automatic Operation RX Finish Flag
     * |        |          |RXFINISH is set when finishing one round of auto operation(Automode = 2/3/4), but it will not issue interrupt
     * |        |          |Software can write 1 to clear this bit.
     * |[10]    |ERRORIF   |ERROR Interrupt Flag
     * |        |          |When LPI2C receiving NACK from slave, running into bus error or arbitration lost, this bit is set to 1
     * |        |          |Software can write 1 to clear this bit.
     * |[24]    |AOFINISH  |Automatic Operation Finish Flag
     * |        |          |AOFINISH is set when finishing one round of auto operation, but it will not issue interrupt
     * |        |          |Software can write 1 to clear this bit.
     * |[31]    |BUSY      |Busy Flag
     * |        |          |When chip is woken up and this bit is set to 1, it means that there are one or more requests from trigger source during transfer
     * |        |          |Software can write 1 to clear this bit.
     * @var LPI2C_T::AUTOCNT
     * Offset: 0x78  LPI2C Automatic Operation Transfer Count Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |TXCNT     |TX Transfer Count
     * |        |          |Bit [7:0] is set for TXPDMA and Random Read transfer mode
     * |        |          |TXCNT represents the required number of the transfer, the real transfer count is TXCNT + 1
     * |        |          |The maximum transfer count is 255 + 1.
     * |[23:16] |RXCNT     |RX Transfer Count
     * |        |          |Bit [23:16] is set for RXPDMA and Random Read transfer mode
     * |        |          |RXCNT represents the required number of the transfer, the real transfer count is RXCNT + 1
     * |        |          |The maximum transfer count is 255 + 1.
     */
    __IO uint32_t CTL0;                  /*!< [0x0000] LPI2C Control Register 0                                         */
    __IO uint32_t ADDR0;                 /*!< [0x0004] LPI2C Slave Address Register0                                    */
    __IO uint32_t DAT;                   /*!< [0x0008] LPI2C Data Register                                              */
    __I  uint32_t STATUS0;               /*!< [0x000c] LPI2C Status Register 0                                          */
    __IO uint32_t CLKDIV;                /*!< [0x0010] LPI2C Clock Divided Register                                     */
    __IO uint32_t TOCTL;                 /*!< [0x0014] LPI2C Time-out Control Register                                  */
    __IO uint32_t ADDR1;                 /*!< [0x0018] LPI2C Slave Address Register1                                    */
    __IO uint32_t ADDR2;                 /*!< [0x001c] LPI2C Slave Address Register2                                    */
    __IO uint32_t ADDR3;                 /*!< [0x0020] LPI2C Slave Address Register3                                    */
    __IO uint32_t ADDRMSK0;              /*!< [0x0024] LPI2C Slave Address Mask Register0                               */
    __IO uint32_t ADDRMSK1;              /*!< [0x0028] LPI2C Slave Address Mask Register1                               */
    __IO uint32_t ADDRMSK2;              /*!< [0x002c] LPI2C Slave Address Mask Register2                               */
    __IO uint32_t ADDRMSK3;              /*!< [0x0030] LPI2C Slave Address Mask Register3                               */
    __I  uint32_t RESERVE0[2];
    __IO uint32_t WKCTL;                 /*!< [0x003c] LPI2C Wake-up Control Register                                   */
    __IO uint32_t WKSTS;                 /*!< [0x0040] LPI2C Wake-up Status Register                                    */
    __IO uint32_t CTL1;                  /*!< [0x0044] LPI2C Control Register 1                                         */
    __IO uint32_t STATUS1;               /*!< [0x0048] LPI2C Status Register 1                                          */
    __IO uint32_t TMCTL;                 /*!< [0x004c] LPI2C Timing Configure Control Register                          */
    __I  uint32_t RESERVE1[8];
    __IO uint32_t AUTOCTL;               /*!< [0x0070] LPI2C Automatic Operation Control Register                       */
    __IO uint32_t AUTOSTS;               /*!< [0x0074] LPI2C Automatic Operation Status Register                        */
    __IO uint32_t AUTOCNT;               /*!< [0x0078] LPI2C Automatic Operation Transfer Count Register                */

} LPI2C_T;

/**
    @addtogroup LPI2C_CONST LPI2C Bit Field Definition
    Constant Definitions for LPI2C Controller
@{ */

#define LPI2C_CTL0_AA_Pos                (2)                                               /*!< LPI2C_T::CTL0: AA Position             */
#define LPI2C_CTL0_AA_Msk                (0x1ul << LPI2C_CTL0_AA_Pos)                      /*!< LPI2C_T::CTL0: AA Mask                 */

#define LPI2C_CTL0_SI_Pos                (3)                                               /*!< LPI2C_T::CTL0: SI Position             */
#define LPI2C_CTL0_SI_Msk                (0x1ul << LPI2C_CTL0_SI_Pos)                      /*!< LPI2C_T::CTL0: SI Mask                 */

#define LPI2C_CTL0_STO_Pos               (4)                                               /*!< LPI2C_T::CTL0: STO Position            */
#define LPI2C_CTL0_STO_Msk               (0x1ul << LPI2C_CTL0_STO_Pos)                     /*!< LPI2C_T::CTL0: STO Mask                */

#define LPI2C_CTL0_STA_Pos               (5)                                               /*!< LPI2C_T::CTL0: STA Position            */
#define LPI2C_CTL0_STA_Msk               (0x1ul << LPI2C_CTL0_STA_Pos)                     /*!< LPI2C_T::CTL0: STA Mask                */

#define LPI2C_CTL0_LPI2CEN_Pos           (6)                                               /*!< LPI2C_T::CTL0: LPI2CEN Position        */
#define LPI2C_CTL0_LPI2CEN_Msk           (0x1ul << LPI2C_CTL0_LPI2CEN_Pos)                 /*!< LPI2C_T::CTL0: LPI2CEN Mask            */

#define LPI2C_CTL0_INTEN_Pos             (7)                                               /*!< LPI2C_T::CTL0: INTEN Position          */
#define LPI2C_CTL0_INTEN_Msk             (0x1ul << LPI2C_CTL0_INTEN_Pos)                   /*!< LPI2C_T::CTL0: INTEN Mask              */

#define LPI2C_ADDR0_GC_Pos               (0)                                               /*!< LPI2C_T::ADDR0: GC Position            */
#define LPI2C_ADDR0_GC_Msk               (0x1ul << LPI2C_ADDR0_GC_Pos)                     /*!< LPI2C_T::ADDR0: GC Mask                */

#define LPI2C_ADDR0_ADDR_Pos             (1)                                               /*!< LPI2C_T::ADDR0: ADDR Position          */
#define LPI2C_ADDR0_ADDR_Msk             (0x7ful << LPI2C_ADDR0_ADDR_Pos)                  /*!< LPI2C_T::ADDR0: ADDR Mask              */

#define LPI2C_DAT_DAT_Pos                (0)                                               /*!< LPI2C_T::DAT: DAT Position             */
#define LPI2C_DAT_DAT_Msk                (0xfful << LPI2C_DAT_DAT_Pos)                     /*!< LPI2C_T::DAT: DAT Mask                 */

#define LPI2C_STATUS0_STATUS_Pos         (0)                                               /*!< LPI2C_T::STATUS0: STATUS Position      */
#define LPI2C_STATUS0_STATUS_Msk         (0xfful << LPI2C_STATUS0_STATUS_Pos)              /*!< LPI2C_T::STATUS0: STATUS Mask          */

#define LPI2C_CLKDIV_DIVIDER_Pos         (0)                                               /*!< LPI2C_T::CLKDIV: DIVIDER Position      */
#define LPI2C_CLKDIV_DIVIDER_Msk         (0x3fful << LPI2C_CLKDIV_DIVIDER_Pos)             /*!< LPI2C_T::CLKDIV: DIVIDER Mask          */

#define LPI2C_CLKDIV_NFCNT_Pos           (12)                                              /*!< LPI2C_T::CLKDIV: NFCNT Position        */
#define LPI2C_CLKDIV_NFCNT_Msk           (0xful << LPI2C_CLKDIV_NFCNT_Pos)                 /*!< LPI2C_T::CLKDIV: NFCNT Mask            */

#define LPI2C_TOCTL_TOIF_Pos             (0)                                               /*!< LPI2C_T::TOCTL: TOIF Position          */
#define LPI2C_TOCTL_TOIF_Msk             (0x1ul << LPI2C_TOCTL_TOIF_Pos)                   /*!< LPI2C_T::TOCTL: TOIF Mask              */

#define LPI2C_TOCTL_TOCDIV4_Pos          (1)                                               /*!< LPI2C_T::TOCTL: TOCDIV4 Position       */
#define LPI2C_TOCTL_TOCDIV4_Msk          (0x1ul << LPI2C_TOCTL_TOCDIV4_Pos)                /*!< LPI2C_T::TOCTL: TOCDIV4 Mask           */

#define LPI2C_TOCTL_TOCEN_Pos            (2)                                               /*!< LPI2C_T::TOCTL: TOCEN Position         */
#define LPI2C_TOCTL_TOCEN_Msk            (0x1ul << LPI2C_TOCTL_TOCEN_Pos)                  /*!< LPI2C_T::TOCTL: TOCEN Mask             */

#define LPI2C_ADDR1_GC_Pos               (0)                                               /*!< LPI2C_T::ADDR1: GC Position            */
#define LPI2C_ADDR1_GC_Msk               (0x1ul << LPI2C_ADDR1_GC_Pos)                     /*!< LPI2C_T::ADDR1: GC Mask                */

#define LPI2C_ADDR1_ADDR_Pos             (1)                                               /*!< LPI2C_T::ADDR1: ADDR Position          */
#define LPI2C_ADDR1_ADDR_Msk             (0x7ful << LPI2C_ADDR1_ADDR_Pos)                  /*!< LPI2C_T::ADDR1: ADDR Mask              */

#define LPI2C_ADDR2_GC_Pos               (0)                                               /*!< LPI2C_T::ADDR2: GC Position            */
#define LPI2C_ADDR2_GC_Msk               (0x1ul << LPI2C_ADDR2_GC_Pos)                     /*!< LPI2C_T::ADDR2: GC Mask                */

#define LPI2C_ADDR2_ADDR_Pos             (1)                                               /*!< LPI2C_T::ADDR2: ADDR Position          */
#define LPI2C_ADDR2_ADDR_Msk             (0x7ful << LPI2C_ADDR2_ADDR_Pos)                  /*!< LPI2C_T::ADDR2: ADDR Mask              */

#define LPI2C_ADDR3_GC_Pos               (0)                                               /*!< LPI2C_T::ADDR3: GC Position            */
#define LPI2C_ADDR3_GC_Msk               (0x1ul << LPI2C_ADDR3_GC_Pos)                     /*!< LPI2C_T::ADDR3: GC Mask                */

#define LPI2C_ADDR3_ADDR_Pos             (1)                                               /*!< LPI2C_T::ADDR3: ADDR Position          */
#define LPI2C_ADDR3_ADDR_Msk             (0x7ful << LPI2C_ADDR3_ADDR_Pos)                  /*!< LPI2C_T::ADDR3: ADDR Mask              */

#define LPI2C_ADDRMSK0_ADDRMSK_Pos       (1)                                               /*!< LPI2C_T::ADDRMSK0: ADDRMSK Position    */
#define LPI2C_ADDRMSK0_ADDRMSK_Msk       (0x7ful << LPI2C_ADDRMSK0_ADDRMSK_Pos)            /*!< LPI2C_T::ADDRMSK0: ADDRMSK Mask        */

#define LPI2C_ADDRMSK1_ADDRMSK_Pos       (1)                                               /*!< LPI2C_T::ADDRMSK1: ADDRMSK Position    */
#define LPI2C_ADDRMSK1_ADDRMSK_Msk       (0x7ful << LPI2C_ADDRMSK1_ADDRMSK_Pos)            /*!< LPI2C_T::ADDRMSK1: ADDRMSK Mask        */

#define LPI2C_ADDRMSK2_ADDRMSK_Pos       (1)                                               /*!< LPI2C_T::ADDRMSK2: ADDRMSK Position    */
#define LPI2C_ADDRMSK2_ADDRMSK_Msk       (0x7ful << LPI2C_ADDRMSK2_ADDRMSK_Pos)            /*!< LPI2C_T::ADDRMSK2: ADDRMSK Mask        */

#define LPI2C_ADDRMSK3_ADDRMSK_Pos       (1)                                               /*!< LPI2C_T::ADDRMSK3: ADDRMSK Position    */
#define LPI2C_ADDRMSK3_ADDRMSK_Msk       (0x7ful << LPI2C_ADDRMSK3_ADDRMSK_Pos)            /*!< LPI2C_T::ADDRMSK3: ADDRMSK Mask        */

#define LPI2C_WKCTL_WKEN_Pos             (0)                                               /*!< LPI2C_T::WKCTL: WKEN Position          */
#define LPI2C_WKCTL_WKEN_Msk             (0x1ul << LPI2C_WKCTL_WKEN_Pos)                   /*!< LPI2C_T::WKCTL: WKEN Mask              */

#define LPI2C_WKCTL_NHDBUSEN_Pos         (7)                                               /*!< LPI2C_T::WKCTL: NHDBUSEN Position      */
#define LPI2C_WKCTL_NHDBUSEN_Msk         (0x1ul << LPI2C_WKCTL_NHDBUSEN_Pos)               /*!< LPI2C_T::WKCTL: NHDBUSEN Mask          */

#define LPI2C_WKSTS_WKIF_Pos             (0)                                               /*!< LPI2C_T::WKSTS: WKIF Position          */
#define LPI2C_WKSTS_WKIF_Msk             (0x1ul << LPI2C_WKSTS_WKIF_Pos)                   /*!< LPI2C_T::WKSTS: WKIF Mask              */

#define LPI2C_WKSTS_WKAKDONE_Pos         (1)                                               /*!< LPI2C_T::WKSTS: WKAKDONE Position      */
#define LPI2C_WKSTS_WKAKDONE_Msk         (0x1ul << LPI2C_WKSTS_WKAKDONE_Pos)               /*!< LPI2C_T::WKSTS: WKAKDONE Mask          */

#define LPI2C_WKSTS_WRSTSWK_Pos          (2)                                               /*!< LPI2C_T::WKSTS: WRSTSWK Position       */
#define LPI2C_WKSTS_WRSTSWK_Msk          (0x1ul << LPI2C_WKSTS_WRSTSWK_Pos)                /*!< LPI2C_T::WKSTS: WRSTSWK Mask           */

#define LPI2C_CTL1_TXPDMAEN_Pos          (0)                                               /*!< LPI2C_T::CTL1: TXPDMAEN Position       */
#define LPI2C_CTL1_TXPDMAEN_Msk          (0x1ul << LPI2C_CTL1_TXPDMAEN_Pos)                /*!< LPI2C_T::CTL1: TXPDMAEN Mask           */

#define LPI2C_CTL1_RXPDMAEN_Pos          (1)                                               /*!< LPI2C_T::CTL1: RXPDMAEN Position       */
#define LPI2C_CTL1_RXPDMAEN_Msk          (0x1ul << LPI2C_CTL1_RXPDMAEN_Pos)                /*!< LPI2C_T::CTL1: RXPDMAEN Mask           */

#define LPI2C_CTL1_PDMARST_Pos           (2)                                               /*!< LPI2C_T::CTL1: PDMARST Position        */
#define LPI2C_CTL1_PDMARST_Msk           (0x1ul << LPI2C_CTL1_PDMARST_Pos)                 /*!< LPI2C_T::CTL1: PDMARST Mask            */

#define LPI2C_CTL1_PDMASTR_Pos           (8)                                               /*!< LPI2C_T::CTL1: PDMASTR Position        */
#define LPI2C_CTL1_PDMASTR_Msk           (0x1ul << LPI2C_CTL1_PDMASTR_Pos)                 /*!< LPI2C_T::CTL1: PDMASTR Mask            */

#define LPI2C_CTL1_SWITCHEN_Pos          (10)                                              /*!< LPI2C_T::CTL1: SWITCHEN Position       */
#define LPI2C_CTL1_SWITCHEN_Msk          (0x1ul << LPI2C_CTL1_SWITCHEN_Pos)                /*!< LPI2C_T::CTL1: SWITCHEN Mask           */

#define LPI2C_STATUS1_ADMAT0_Pos         (0)                                               /*!< LPI2C_T::STATUS1: ADMAT0 Position      */
#define LPI2C_STATUS1_ADMAT0_Msk         (0x1ul << LPI2C_STATUS1_ADMAT0_Pos)               /*!< LPI2C_T::STATUS1: ADMAT0 Mask          */

#define LPI2C_STATUS1_ADMAT1_Pos         (1)                                               /*!< LPI2C_T::STATUS1: ADMAT1 Position      */
#define LPI2C_STATUS1_ADMAT1_Msk         (0x1ul << LPI2C_STATUS1_ADMAT1_Pos)               /*!< LPI2C_T::STATUS1: ADMAT1 Mask          */

#define LPI2C_STATUS1_ADMAT2_Pos         (2)                                               /*!< LPI2C_T::STATUS1: ADMAT2 Position      */
#define LPI2C_STATUS1_ADMAT2_Msk         (0x1ul << LPI2C_STATUS1_ADMAT2_Pos)               /*!< LPI2C_T::STATUS1: ADMAT2 Mask          */

#define LPI2C_STATUS1_ADMAT3_Pos         (3)                                               /*!< LPI2C_T::STATUS1: ADMAT3 Position      */
#define LPI2C_STATUS1_ADMAT3_Msk         (0x1ul << LPI2C_STATUS1_ADMAT3_Pos)               /*!< LPI2C_T::STATUS1: ADMAT3 Mask          */

#define LPI2C_STATUS1_ONBUSY_Pos         (8)                                               /*!< LPI2C_T::STATUS1: ONBUSY Position      */
#define LPI2C_STATUS1_ONBUSY_Msk         (0x1ul << LPI2C_STATUS1_ONBUSY_Pos)               /*!< LPI2C_T::STATUS1: ONBUSY Mask          */

#define LPI2C_TMCTL_STCTL_Pos            (0)                                               /*!< LPI2C_T::TMCTL: STCTL Position         */
#define LPI2C_TMCTL_STCTL_Msk            (0x1fful << LPI2C_TMCTL_STCTL_Pos)                /*!< LPI2C_T::TMCTL: STCTL Mask             */

#define LPI2C_TMCTL_HTCTL_Pos            (16)                                              /*!< LPI2C_T::TMCTL: HTCTL Position         */
#define LPI2C_TMCTL_HTCTL_Msk            (0x1fful << LPI2C_TMCTL_HTCTL_Pos)                /*!< LPI2C_T::TMCTL: HTCTL Mask             */

#define LPI2C_AUTOCTL_TGSRCSEL_Pos       (0)                                               /*!< LPI2C_T::AUTOCTL: TGSRCSEL Position    */
#define LPI2C_AUTOCTL_TGSRCSEL_Msk       (0xful << LPI2C_AUTOCTL_TGSRCSEL_Pos)             /*!< LPI2C_T::AUTOCTL: TGSRCSEL Mask        */

#define LPI2C_AUTOCTL_TRGEN_Pos          (4)                                               /*!< LPI2C_T::AUTOCTL: TRGEN Position       */
#define LPI2C_AUTOCTL_TRGEN_Msk          (0x1ul << LPI2C_AUTOCTL_TRGEN_Pos)                /*!< LPI2C_T::AUTOCTL: TRGEN Mask           */

#define LPI2C_AUTOCTL_TXWKEN_Pos         (5)                                               /*!< LPI2C_T::AUTOCTL: TXWKEN Position      */
#define LPI2C_AUTOCTL_TXWKEN_Msk         (0x1ul << LPI2C_AUTOCTL_TXWKEN_Pos)               /*!< LPI2C_T::AUTOCTL: TXWKEN Mask          */

#define LPI2C_AUTOCTL_RXWKEN_Pos         (6)                                               /*!< LPI2C_T::AUTOCTL: RXWKEN Position      */
#define LPI2C_AUTOCTL_RXWKEN_Msk         (0x1ul << LPI2C_AUTOCTL_RXWKEN_Pos)               /*!< LPI2C_T::AUTOCTL: RXWKEN Mask          */

#define LPI2C_AUTOCTL_NACKWKEN_Pos       (7)                                               /*!< LPI2C_T::AUTOCTL: NACKWKEN Position    */
#define LPI2C_AUTOCTL_NACKWKEN_Msk       (0x1ul << LPI2C_AUTOCTL_NACKWKEN_Pos)             /*!< LPI2C_T::AUTOCTL: NACKWKEN Mask        */

#define LPI2C_AUTOCTL_AUTOMODE_Pos       (8)                                               /*!< LPI2C_T::AUTOCTL: AUTOMODE Position    */
#define LPI2C_AUTOCTL_AUTOMODE_Msk       (0x7ul << LPI2C_AUTOCTL_AUTOMODE_Pos)             /*!< LPI2C_T::AUTOCTL: AUTOMODE Mask        */

#define LPI2C_AUTOCTL_SWTRG_Pos          (31)                                              /*!< LPI2C_T::AUTOCTL: SWTRG Position       */
#define LPI2C_AUTOCTL_SWTRG_Msk          (0x1ul << LPI2C_AUTOCTL_SWTRG_Pos)                /*!< LPI2C_T::AUTOCTL: SWTRG Mask           */

#define LPI2C_AUTOSTS_TXWKF_Pos          (0)                                               /*!< LPI2C_T::AUTOSTS: TXWKF Position       */
#define LPI2C_AUTOSTS_TXWKF_Msk          (0x1ul << LPI2C_AUTOSTS_TXWKF_Pos)                /*!< LPI2C_T::AUTOSTS: TXWKF Mask           */

#define LPI2C_AUTOSTS_RXWKF_Pos          (1)                                               /*!< LPI2C_T::AUTOSTS: RXWKF Position       */
#define LPI2C_AUTOSTS_RXWKF_Msk          (0x1ul << LPI2C_AUTOSTS_RXWKF_Pos)                /*!< LPI2C_T::AUTOSTS: RXWKF Mask           */

#define LPI2C_AUTOSTS_ERRORWKF_Pos       (2)                                               /*!< LPI2C_T::AUTOSTS: ERRORWKF Position    */
#define LPI2C_AUTOSTS_ERRORWKF_Msk       (0x1ul << LPI2C_AUTOSTS_ERRORWKF_Pos)             /*!< LPI2C_T::AUTOSTS: ERRORWKF Mask        */

#define LPI2C_AUTOSTS_TXFINISH_Pos       (8)                                               /*!< LPI2C_T::AUTOSTS: TXFINISH Position    */
#define LPI2C_AUTOSTS_TXFINISH_Msk       (0x1ul << LPI2C_AUTOSTS_TXFINISH_Pos)             /*!< LPI2C_T::AUTOSTS: TXFINISH Mask        */

#define LPI2C_AUTOSTS_RXFINISH_Pos       (9)                                               /*!< LPI2C_T::AUTOSTS: RXFINISH Position    */
#define LPI2C_AUTOSTS_RXFINISH_Msk       (0x1ul << LPI2C_AUTOSTS_RXFINISH_Pos)             /*!< LPI2C_T::AUTOSTS: RXFINISH Mask        */

#define LPI2C_AUTOSTS_ERRORIF_Pos        (10)                                              /*!< LPI2C_T::AUTOSTS: ERRORIF Position     */
#define LPI2C_AUTOSTS_ERRORIF_Msk        (0x1ul << LPI2C_AUTOSTS_ERRORIF_Pos)              /*!< LPI2C_T::AUTOSTS: ERRORIF Mask         */

#define LPI2C_AUTOSTS_AOFINISH_Pos       (24)                                              /*!< LPI2C_T::AUTOSTS: AOFINISH Position    */
#define LPI2C_AUTOSTS_AOFINISH_Msk       (0x1ul << LPI2C_AUTOSTS_AOFINISH_Pos)             /*!< LPI2C_T::AUTOSTS: AOFINISH Mask        */

#define LPI2C_AUTOSTS_BUSY_Pos           (31)                                              /*!< LPI2C_T::AUTOSTS: BUSY Position        */
#define LPI2C_AUTOSTS_BUSY_Msk           (0x1ul << LPI2C_AUTOSTS_BUSY_Pos)                 /*!< LPI2C_T::AUTOSTS: BUSY Mask            */

#define LPI2C_AUTOCNT_TXCNT_Pos          (0)                                               /*!< LPI2C_T::AUTOCNT: TXCNT Position       */
#define LPI2C_AUTOCNT_TXCNT_Msk          (0xfful << LPI2C_AUTOCNT_TXCNT_Pos)               /*!< LPI2C_T::AUTOCNT: TXCNT Mask           */

#define LPI2C_AUTOCNT_RXCNT_Pos          (16)                                              /*!< LPI2C_T::AUTOCNT: RXCNT Position       */
#define LPI2C_AUTOCNT_RXCNT_Msk          (0xfful << LPI2C_AUTOCNT_RXCNT_Pos)               /*!< LPI2C_T::AUTOCNT: RXCNT Mask           */

/** @} LPI2C_CONST */
/** @} end of LPI2C register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __LPI2C_REG_H__ */
