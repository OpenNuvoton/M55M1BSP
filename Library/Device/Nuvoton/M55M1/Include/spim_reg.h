/**************************************************************************//**
 * @file     spim_reg.h
 * @version  V1.00
 * @brief    SPIM register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __SPIM_REG_H__
#define __SPIM_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

#define SPIM_REG_CACHE                  (0) /*!< SPIM cache on/off    \hideinitializer */

/**
    @addtogroup REGISTER Control Register
  @{

*/

/**
    @addtogroup SPIM SPI/Hyper Bus Synchronous Serial Interface Controller (SPIM)
    Memory Mapped Structure for SPIM Controller
    @{
*/

typedef struct
{
    /**
     * @var SPIM_T::CTL0
     * Offset: 0x00  Control and Status Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CIPHOFF   |Cipher Disable Bit
     * |        |          |0 = Cipher function Enabled.
     * |        |          |1 = Cipher function Disabled.
     * |        |          |Note 1: When CIPHOFF(SPIM_CTL0[0]) is 0 and OTF AES engine outside SPIM is enabled, encryption/decryption of SPIM is enabled.
     * |        |          |Note 2: When encryption/decryption of SPIM is enabled, please set DESELTIM (SPIM_DMMCTL[23:16]) >= 0x10
     * |        |          |When encryption/decryption of SPIM is disabled, please set DESELTIM(SPIM_DMMCTL[23:16]) >= 0x8.
     * |[2]     |BALEN     |Balance the AHB Control Time Between Cipher Enable and Disable Control
     * |        |          |When cipher is enabled, the AHB control signal will delay some time caused by the encoding or decoding calculation
     * |        |          |Therefore, if set BALEN to 1, it will make the AHB signal processing time with cipher disabled be equal to that with cipher enabled.
     * |        |          |Note: Only useful when both of cipher and OTF AES are disabled.
     * |[3]     |HYPER_EN  |HYPER RAM Mode Enable Bit
     * |        |          |0 = SPIROM operates in SPI Flash mode.
     * |        |          |1 = SPIROM operates in HYPER Device mode.
     * |[5]     |B4ADDREN  |4-byte Address Mode Enable Bit
     * |        |          |0 = 3-byte address mode Enabled.
     * |        |          |1 = 4-byte address mode Enabled.
     * |        |          |Note: Used for DMA write mode, DMA read mode, and DMM mode.
     * |[6]     |IEN       |Interrupt Enable Bit
     * |        |          |0 = SPIM Interrupt Disabled.
     * |        |          |1 = SPIM Interrupt Enabled.
     * |[7]     |IF        |Interrupt Flag
     * |        |          |Write Operation:
     * |        |          |0 = No effect.
     * |        |          |1 = Write 1 to clear.
     * |        |          |Read Operation:
     * |        |          |0 = The transfer has not finished yet.
     * |        |          |1 = The transfer has done.
     * |[12:8]  |DWIDTH    |Transmit/Receive Bit Length
     * |        |          |This specifies how many bits are transmitted/received in one transmit/receive transaction.
     * |        |          |0x7 = 8 bits.
     * |        |          |0xF = 16 bits.
     * |        |          |0x17 = 24 bits.
     * |        |          |0x1F = 32 bits.
     * |        |          |Others = Incorrect transfer result.
     * |        |          |Note 1: Only used for normal I/O mode.
     * |        |          |Note 2: Only 8, 16, 24, and 32 bits are allowed. Other bit length will result in incorrect transfer.
     * |        |          |Note 3: Only 16 and 32 bits are allowed when both of Octal mode and DTR mode of normal I/O mode are enabled
     * |        |          |Other bit length will result in incorrect transfer.
     * |[14:13] |BURSTNUM  |Transmit/Receive Burst Number
     * |        |          |This field specifies how many transmit/receive transactions should be executed continuously in one transfer.
     * |        |          |0x0 = Only one transmit/receive transaction will be executed in one transfer.
     * |        |          |0x1 = Two successive transmit/receive transactions will be executed in one transfer.
     * |        |          |0x2 = Three successive transmit/receive transactions will be executed in one transfer.
     * |        |          |0x3 = Four successive transmit/receive transactions will be executed in one transfer.
     * |        |          |Note: Only used for normal I/O Mode.
     * |[15]    |QDIODIR   |SPI Interface Direction Select for Normal I/O Mode
     * |        |          |0 = Interface signals are input.
     * |        |          |1 = Interface signals are output.
     * |        |          |Note: Only used for normal I/O mode.
     * |[19:16] |SUSPITV   |Suspend Interval
     * |        |          |These four bits provide the configuration of suspend interval between two successive transmit/receive transactions in a transfer
     * |        |          |The default value is 0x00
     * |        |          |When BURSTNUM = 00, setting this field has no effect on transfer
     * |        |          |The desired interval is obtained according to the following equation (from the last falling edge of current SPI clock to the first rising edge of next SPI clock):
     * |        |          |(SUSPITV+2)*period of AHB clock
     * |        |          |0x0 = Hardware will control this item by DIVIDER (SPIM_CTL1[31:16]) automatically.
     * |        |          |0x1 = 8 AHB clock cycles.
     * |        |          |....
     * |        |          |0xE = 21 AHB clock cycles.
     * |        |          |0xF = 22 AHB clock cycles.
     * |        |          |Note: Only used for normal I/O mode.
     * |[21:20] |BITMODE   |SPI Interface Bit Mode
     * |        |          |0x0 = Standard mode.
     * |        |          |0x1 = Dual mode.
     * |        |          |0x2 = Quad mode.
     * |        |          |0x3 = Octal mode.
     * |        |          |Note: Only used for normal I/O mode.
     * |[23:22] |OPMODE    |SPI Function Operation Mode
     * |        |          |0x0 = Normal I/O mode. (Note1) (Note3)
     * |        |          |0x1 = DMA write mode. (Note2) (Note3)
     * |        |          |0x2 = DMA read mode. (Note3)
     * |        |          |0x3 = Direct Memory Mapping mode (DMM mode) (Default). (Note4)
     * |        |          |Note 1: After using Normal I/O mode of SPI Flash/Hyper device controller to program the content of SPI Flash and Hyper device, please set CDINVAL(SPIM_CTL1[3]) to 0x1 (Set all cache data to be invalid).
     * |        |          |Note 2: In DMA write mode, hardware will send just one page program command per operation
     * |        |          |Users must take care of cross-page cases
     * |        |          |After using DMA write mode to program the content of SPI Flash and Hyper device, please set CDINVAL(SPIM_CTL1[3]) to 0x1 (Set all cache data to be invalid).
     * |        |          |Note 3: For SPI Flash/Hyper device with 32 Mbytes, access address range is from 0x00000000 to 0x01FFFFFF when using Normal I/O mode, DMA write mode, and DMA read mode to write/read SPI Flash and Hyper device
     * |        |          |Please user check size of used SPI Flash component to know access address range of SPI Flash and Hyper device.
     * |        |          |Note 4: For SPI Flash/Hyper device with 32 Mbytes, access address range is from 0x08000000 to 0x09FFFFFF when using Direct Memory mapping mode (DMM mode) to read SPI Flash and write/read Hyper device
     * |        |          |Please user check size of used storage components to know access address range
     * |        |          |Mbytes Mbytes
     * |[24]    |DTR_NORM  |Double Transfer Rate Mode Enable Bit for Normal I/O Mode
     * |        |          |In master mode, SPI will receive data at both rising edge and falling edge of SPI bus clock.
     * |        |          |0 = DTR mode Disabled (i.e. Single Transfer Rate (STR) mode enabled).
     * |        |          |1 = DTR mode Enabled.
     * |        |          |Note: Only used for normal I/O mode.
     * |[25]    |RDQS_NORM |Read DQS Mode Enable Bit for Normal I/O Mode
     * |        |          |In SPI master mode, SPI uses read DQS signal, which is generated by SPI Flash, to sample RX data from SPI Flash when Read DQS mode is enabled.
     * |        |          |0 = Read DQS mode Disabled.
     * |        |          |1 = Read DQS mode Enabled.
     * |        |          |Note: Only used for normal I/O mode.
     * |[27:26] |RBO_NORM  |Received Data Byte Order of Normal I/O Mode for Octal SPI Flash
     * |        |          |(1) Data width = 8-bits:.
     * |        |          |0x0 = Byte order of received data from SPI Flash is byte0.
     * |        |          |Other = Reserved.
     * |        |          |(2) Data width = 16-bits:.
     * |        |          |0x0 = Byte order of received data from SPI Flash is byte0, byte1.
     * |        |          |0x1 = Byte order of received data from SPI Flash is byte1, byte0.
     * |        |          |Other = Reserved.
     * |        |          |(3) Data width = 24-bits:.
     * |        |          |0x0 = Byte order of received data from SPI Flash is byte0, byte1, byte2.
     * |        |          |0x1 = Byte order of received data from SPI Flash is byte2, byte1, byte0.
     * |        |          |Other = Reserved.
     * |        |          |(4) Data width = 32-bits:.
     * |        |          |0x0 = Byte order of received data from SPI Flash is byte0, byte1, byte2, byte3.
     * |        |          |0x1 = Byte order of received data from SPI Flash is byte3, byte2, byte1, byte0.
     * |        |          |0x2 = Byte order of received data from SPI Flash is byte1, byte0, byte3, byte2.
     * |        |          |0x3 = Byte order of received data from SPI Flash is byte2, byte3, byte0, byte1.
     * @var SPIM_T::CTL1
     * Offset: 0x04  Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SPIMEN    |Go and Busy Status
     * |        |          |Write Operation:
     * |        |          |0 = No effect.
     * |        |          |1 = Start the transfer
     * |        |          |This bit remains set during the transfer and is automatically cleared after transfer finished.
     * |        |          |Read Operation:
     * |        |          |0 = The transfer has done.
     * |        |          |1 = The transfer has not finished yet.
     * |        |          |Note: All registers should be set before writing 1 to the SPIMEN bit
     * |        |          |When a transfer is in progress, user should not write to any register of this peripheral.
     * |[1]     |CACHEOFF  |Cache Memory Function Disable Bit
     * |        |          |0 = Cache memory function Enabled. (Default)
     * |        |          |1 = Cache memory function Disabled.
     * |[3]     |CDINVAL   |Cache Data Invalid Enable Bit
     * |        |          |Write Operation:
     * |        |          |0 = No effect.
     * |        |          |1 = Set all cache data to be invalid. This bit is cleared by hardware automatically.
     * |        |          |Read Operation: No effect
     * |        |          |Note: When SPI Flash memory is page erasing or whole Flash erasing, please set CDINVAL to 0x1
     * |        |          |After using normal I/O mode to program or erase the content of SPI Flash, please set CDINVAL to 0x1.
     * |[4]     |SS        |Slave Select Active Enable Bit
     * |        |          |0 = SPIM_SS is in active level.
     * |        |          |1 = SPIM_SS is in inactive level (Default).
     * |        |          |Note: This interface can only drive one device/slave at a given time
     * |        |          |Therefore, the slave selects of the selected device must be set to its active level before starting any read or write transfer
     * |        |          |Functional description of SSACTPOL(SPIM_CTL1[5]) and SS is shown in Table 1.1-11.11
     * |[5]     |SSACTPOL  |Slave Select Active Level
     * |        |          |It defines the active level of device/slave select signal (SPIM_SS), as shown in Table 1.1-11.11
     * |        |          |0 = The SPIM_SS slave select signal is active low.
     * |        |          |1 = The SPIM_SS slave select signal is active high.
     * |[6]     |CAWRTHEN  |Cache Write Through Enable Bit
     * |        |          |0 = Cache write-through function is disabled.
     * |        |          |1 = Cache write-through function is enabled.
     * |[7]     |AUTOSCLN  |Auto Selection Updated Cache Line Number Enable Bit
     * |        |          |0 = Auto-selection function is diabled for number of updated cache line per cache miss, and user can use UPDCLNUM (SPIM_CTL1[15:12]) to adjust number of updated cache line per cache miss manually
     * |        |          |(default)
     * |        |          |1 = Auto-selection function is enabled for number of updated cache line per cache miss
     * |        |          |SPIM will load one cache data line (1x16bytes data), two cache data lines (2x16bytes data), three cache data lines (3x16bytes data), or four cache data lines (4x16bytes data) by hardware automatically with previous history of cache miss and data correlation.Reserved.
     * |[11:8]  |IDLETIME  |Idle Time Interval
     * |        |          |In DMM mode, IDLETIME is set to control the minimum idle time between two SPI Flash accesses.
     * |        |          |Minimum idle time = (IDLETIME + 1) * AHB clock cycle time.
     * |        |          |Note 1: Only used for DMM mode.
     * |        |          |Note 2: AHB clock cycle time = 1/AHB clock frequency.
     * |[15:12] |UPDCLNUM  |Updated Cache Line Number per Cache Miss
     * |        |          |0x1 = Update one cache line per cache miss. (default)
     * |        |          |0x2 = Update two cache lines per cache miss.
     * |        |          |0x3 = Update three cache lines per cache miss.
     * |        |          |0x4 = Update four cache lines per cache miss.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: Data size of each cache line is 16 bytes.
     * |[31:16] |DIVIDER   |Clock Divider Register
     * |        |          |The value in this field is the frequency divider of the AHB clock (HCLK) to generate the serial SPI output clock "SCLK" on the output SPIM_CLK pin
     * |        |          |The desired frequency is obtained according to the following equation:
     * |        |          |Note 1: When set DIVIDER to zero, the frequency of SPIM_CLK will be equal to the frequency of HCLK.
     * |        |          |Note 2: SCLK is serial SPI output clock.
     * |        |          |Note 3: Each SPI Flash command has the limitation of the maximum operation frequency of SCLK
     * |        |          |Please check the specification of the used SPI Flash component to decide the frequency of SPI Flash clock for different command operations of SPI Flash.
     * |        |          |Note 4: For DTR commands, the setting values of DIVIDER are only 1,2,4,8,16,32,...
     * |        |          |Note 5: For commands of Hyper bus device, the setting values of DIVIDER are only 1 and 2.
     * @var SPIM_T::RXCLKDLY
     * Offset: 0x0C  RX Clock Delay Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |RDDLYSEL  |Sampling Clock Delay Selection for Received Data for Normal I/O Mode, DMA Read Mode, DMA Write Mode, and Direct Memory Mapping Mode Only
     * |        |          |Determine the number of inserted delay cycles
     * |        |          |Used to adjust the sampling clock of received data to latch the correct data.
     * |        |          |0x0: No delay. (Default)
     * |        |          |0x1: Delay 0.5 SPI Flash clock.
     * |        |          |0x2: Delay 1 SPI Flash clocks.
     * |        |          |0x3: Delay 1.5 SPI Flash clocks.
     * |        |          |....
     * |        |          |0xf: Delay 7.5 SPI Flash clocks
     * |        |          |Note: The manufacturer or device ID of SPI Flash component can be used to determine the correct setting value of RDDLYSEL
     * |        |          |An example is given as follows.
     * |        |          |For example, the manufacturer ID and device ID of SPI Flash for some vendor are 0xEF and 0x1234 separately
     * |        |          |First, set RDDLYSEL to 0x0, and use read manufacturer id/device id command to read the manufacturer id of SPI Flash by using normal I/O mode (the manufacturer id is 0xEF (1110_1111) in this example).
     * |        |          |If the manufacturer ID which reads from SPI Flash is 0xF7 (1111_0111), it denotes that manufacturer id is shifted the right by 1 bit and most significant bit (MSB) of manufacturer id is assigned to 1
     * |        |          |According to manufacturer id reads from SPI Flash, RDDLYSEL needs to be set to 0x1 to receive SPI Flash data correctly.
     * @var SPIM_T::RX[4]
     * Offset: 0x10 ~ 0x1C Data Receive Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RXDAT     |Data Receive Register
     * |        |          |The Data Receive Registers hold the received data of the last executed transfer.
     * |        |          |Number of valid RX registers is specified in SPIM_CTL0[BURSTNUM]
     * |        |          |If BURSTNUM > 0, received data are held in the most significant RXDAT register first.
     * |        |          |Number of valid-bit is specified in SPIM_CTL0[DWIDTH]
     * |        |          |If DWIDTH is 16, 24, or 32, received data are held in the least significant byte of RXDAT register first.
     * |        |          |In a byte, received data are held in the most significant bit of RXDAT register first.
     * |        |          |Example1: If SPIM_CTL0[BURSTNUM] = 0x3 and SPIM_CTL1[DWIDTH] = 0x17, received data will be held in the order SPIM_RX3[23:0], SPIM_RX2[23:0], SPIM_RX1[23:0], SPIM_RX0[23:0].
     * |        |          |Example2: If SPIM_CTL0[BURSTNUM = 0x0 and SPIM_CTL0[DWIDTH] = 0x07, received data will be held in the order SPIM_RX0[7], SPIM_RX0[6], .....,
     * |        |          |SPIM_RX0[0].
     * @var SPIM_T::TX[4]
     * Offset: 0x20 ~ 0x2C Data Transmit Register 0 ~ 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |TXDAT     |Data Transmit Register
     * |        |          |The Data Transmit Registers hold the data to be transmitted in next transfer.
     * |        |          |Number of valid TXDAT registers is specified in SPIM_CTL0[BURSTNUM]
     * |        |          |If BURSTNUM > 0, data are transmitted in the most significant TXDAT register first.
     * |        |          |Number of valid-bit is specified in SPIM_CTL0[DWIDTH]
     * |        |          |If DWIDTH is 16, 24, or 32, data are transmitted in the least significant byte of TXDAT register first.
     * |        |          |In a byte, data are transmitted in the most significant bit of TXDAT register first.
     * |        |          |Example1: If SPIM_CTL0[BURSTNUM] = 0x3 and SPIM_CTL1[DWIDTH] = 0x17, data will be transmitted in the order SPIM_TX3[23:0], SPIM_TX2[23:0], SPIM_TX1[23:0], SPIM_TX0[23:0] in next transfer.
     * |        |          |Example2: If SPIM_CTL0[BURSTNUM] = 0x0 and SPIM_CTL0[DWIDTH] = 0x17, data will be transmitted in the order SPIM_TX0[7:0], SPIM_TX0[15:8], SPIM_TX0[23:16] in next transfer.
     * |        |          |Example3: If SPIM_CTL0[BURSTNUM] = 0x0 and SPIM_CTL0[DWIDTH] = 0x07, data will be transmitted in the order SPIM_TX0[7], SPIM_TX0[6], ...,
     * |        |          |SPIM_TX0[0] in next transfer.
     * @var SPIM_T::SRAMADDR
     * Offset: 0x30  SRAM Memory Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ADDR      |SRAM Memory Address
     * |        |          |For DMA Read mode, this is the destination address for DMA transfer.
     * |        |          |For DMA Write mode, this is the source address for DMA transfer.
     * |        |          |Note : This address must be 8bytes aligned.
     * @var SPIM_T::DMACNT
     * Offset: 0x34  DMA Transfer Byte Count Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |DMACNT    |DMA Transfer Byte Count Register
     * |        |          |It indicates the transfer length for DMA process.
     * |        |          |Note 1: The unit for counting is byte and the number must be the multiple of 8.
     * |        |          |Note 2: The page program instruction allows from one byte to 256 bytes (a page) of data to be programmed at previously erased (FFh) memory locations for SPI flash device.
     * |        |          |Note 3: A maximum of 512 bytes can be programmed as long as an aligned 512-byte address boundary is not crossed for HYPER Flash device.
     * |        |          |Note 4: Please check specification of used SPI Flash and Hyper Flash to know maximum byte length of page program and program address boundary.
     * @var SPIM_T::FADDR
     * Offset: 0x38  Address Register for SPI Flash and Hyper Device
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ADDR      |Address Register for SPI Flash and Hyper Device
     * |        |          |For DMA Read mode, this is the source address for DMA transfer.
     * |        |          |For DMA Write mode, this is the destination address for DMA transfer.
     * |        |          |Note 1: This address must be 8bytes aligned.
     * |        |          |Note 2: A maximum of 512 bytes can be programmed as long as an aligned 512-byte address boundary is not crossed for HYPER Flash device.
     * |        |          |Note 3: For SPI Flash and Hyper device with 32 Mbytes, the value "ADDR" is from 0x00000000 to 0x00FFFFFF when using DMA write mode and DMA read mode to write/read data of SPI Flash and Hyper device
     * |        |          |Please check specification of used SPI Flash and Hyper Flash to know access address range and program address boundary
     * |        |          |Mbytes
     * @var SPIM_T::DMMCTL
     * Offset: 0x44  Direct Memory Mapping Mode Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |ACTSCLKT  |SPI Flash Active SCLK Time for Direct Memory Mapping Mode, DMA Write Mode, and DMA Read Mode Only
     * |        |          |The bits set time interval between SPIM SS active edge and the position edge of the first serial SPI output clock, as shown in Figure 1.1-5.
     * |        |          |(1) ACTSCLKT = 0 (function disable):.
     * |        |          |Time interval = 1 AHB clock cycle time.
     * |        |          |(2) ACTSCLKT u2260 0 (function enable):
     * |        |          |Time interval = (ACTSCLKT + 3) * AHB clock cycle time.
     * |        |          |Note 1: AHB clock cycle time = 1/AHB clock frequency.
     * |        |          |Note 2: SCLK is SPI output clock
     * |        |          |Note 3: Please check the used SPI Flash specification to know the setting value of this register, and different SPI Flash vendor may use different setting values.
     * |[23:16] |DESELTIM  |SPI Flash Deselect Time for Direct Memory Mapping Mode Only
     * |        |          |Set the minimum time width of SPI Flash deselect time (i.e
     * |        |          |Minimum SPIM_SS deselect time), as shown in Figure 1.1-5.
     * |        |          |(1) Cache function disable:
     * |        |          |Minimum time width of SPIM_SS deselect time = (DESELTIM + 1) * AHB clock cycle time.
     * |        |          |(2) Cache function enable:
     * |        |          |Minimum time width of SPIM_SS deselect time = (DESELTIM + 4) * AHB clock cycle time.
     * |        |          |Note 1: AHB clock cycle time = 1/AHB clock frequency.
     * |        |          |Note 2: When cipher encryption/decryption is enabled, please set this register value >= 0x10
     * |        |          |When cipher encryption/decryption is disabled, please set this register value >= 0x8.
     * |        |          |Note 3: Please check the used SPI Flash specification to know the setting value of this register, and different SPI Flash vendor may use different setting values.
     * |[24]    |BWEN      |16 Bytes Burst Wrap Mode Enable Bit for Direct Memory Mapping Mode, Cache Enable, and Read Command Code 0xEB, and 0xE7 Only
     * |        |          |0 = Burst Wrap Mode Disabled. (Default)
     * |        |          |1 = Burst Wrap Mode Enabled.
     * |        |          |In direct memory mapping mode, both of quad read commands "0xEB" and "0xE7" support burst wrap mode for cache application and performance enhance
     * |        |          |For cache application, the burst wrap mode can be used to fill the cache line quickly (In this SPI Flash controller, use cache data line with 16 bytes size)
     * |        |          |For performance enhance with direct memory mapping mode and cache enable, when cache data is miss, the burst wrap mode can let MCU get the required SPI Flash data quickly.
     * |        |          |Note: In Direct Memory Mapping Mode, the cache function needs to be enabled when 16 bytes burst wrap mode is enabled.
     * |[25]    |CREN      |Continuous Read Mode Enable Bit for Direct Memory Mapping Mode, Read Command Codes 0xBB, 0xEB, 0xE7, 0x0D, 0xBD, and 0xED Only
     * |        |          |0 = Continuous Read Mode Disabled. (Default)
     * |        |          |1 = Continuous Read Mode Enabled.
     * |        |          |For read operations of SPI Flash, commands of fast read quad I/O (0xEB), word read quad I/O (0xE7), fast read dual I/O (0xBB), DTR fast read (0x0D), DTR fast read dual I/O (0xBD), and DTR fast read quad I/O (0xED) can further reduce cycles of command phase overhead through setting the "continuous read mode" after address phase.
     * |        |          |Note: When user sets CREN to 1 to use continuous read mode, SPIM_MODE must be set by used SPI Flash specifications.
     * |[26]    |UACTSCLK  |User Sets SPI Flash Active SCLK Time for Direct Memory Mapping Mode, DMA Write Mode, and DMA Read Mode Only
     * |        |          |0 = According to DIVIDER(SPIM_CTL1[31:16]), ACTSCLKT(SPIM_DMMCTL[7:0]) is set by hardware automatically
     * |        |          |(Default)
     * |        |          |1 = Set ACTSCLKT(SPIM_DMMCTL[7:0]) by user manually.
     * |        |          |Note: When user wants to set ACTSCLKT(SPIM_DMMCTL[7:0]) manually, please set UACTSCLK to 1.
     * |[27]    |HYPDONE   |Set All Read/Write Operations Done in Direct Memory Mapping Mode for HYPER Device
     * |        |          |Write Operation:
     * |        |          |0 = Finish process of all read/write operations is done in DMM mode for HYPER device.
     * |        |          |1 = Finish process of all read/write operations is executed in DMM mode for HYPER device
     * |        |          |This bit is cleared by hardware automatically.
     * |        |          |Note: User must set this bit to 1 after all read/write operations are done in DMM mode for HYPER device, and wait this bit to 0 to indicate that finish process of all read/write operations is done in DMM mode for HYPER device.
     * @var SPIM_T::CTL2
     * Offset: 0x48  Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:8]  |DC_DMAR   |Dummy Cycle Number for DMA Read Mode Only
     * |        |          |Set number of dummy cycles for DMA read mode only.
     * |        |          |Note 1: When there is not any dummy cycles in this used read command of SPI Flash, user must set DC_DMAR to 0x0.
     * |        |          |Note 2: Number of dummy cycles depends on the frequency of SPI output clock, SPI Flash vendor, and read command types
     * |        |          |Please check the used SPI Flash specification to know the setting value of this number of dummy cycles.
     * |[23:16] |DC_DMM    |Dummy Cycle Number for Direct Memory Mapping Mode Only
     * |        |          |Set number of dummy cycles for direct memory mapping mode only.
     * |        |          |Note 1: When there is not any dummy cycles in this used read command of SPI Flash, user must set DC_DMM to 0x0.
     * |        |          |Note 2: Number of dummy cycles depends on the frequency of SPI output clock, SPI Flash vendor, and read command types
     * |        |          |Please check the used SPI Flash specification to know the setting value of this number of dummy cycles.
     * @var SPIM_T::CMDCODE
     * Offset: 0x4C  Command Code Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |CMDCODE   |SPI Flash Command Code
     * |        |          |The SPI Flash command codes are separated in the following types.
     * |        |          |(1) Page Program Command Code for DMA Write mode of SPIM
     * |        |          |(2) Read Command Code for DMA Read mode and DMM mode of SPIM
     * |        |          |Note 1: Quad mode of SPI Flash must be enabled first by normal I/O mode before using quad page program/quad read commands.
     * |        |          |Note 2: Please check SPI Flash specifications for support command codes.
     * |        |          |Note 3: Please disable "continuous read mode" and "burst wrap mode" before DMA write mode of SPI Flash controller is used to program data of SPI Flash
     * |        |          |After using DMA write mode of SPI Flash controller to program the content of SPI Flash, please set CDINVAL(SPIM_CTL1[3]) to 0x1 (Set all cache data to be invalid).
     * @var SPIM_T::MODE
     * Offset: 0x50  Mode Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |MODEDATA  |Mode Data Register
     * |        |          |Set output data for mode phase.
     * |        |          |When user sets this mode data (Note1) and set CREN(SPIM_DMMCTL[25]) to enter continuous read mode, this will reduce the command phase by eight clocks and allows the read address to be immediately entered after SPIM_SS asserted to active
     * |        |          |(Note1)
     * |        |          |Note 1: Please check the used SPI Flash specification to know the setting value of this mode bits data, and different SPI Flash vendor may use different setting values.
     * @var SPIM_T::PHDMAW
     * Offset: 0x54  Phase Setting Register for DMA Write Mode
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |DW_CMD    |Data Width for Command Phase
     * |        |          |This specifies how many bits are transmitted in this phase.
     * |        |          |0x0 = No command phase.
     * |        |          |0x1 = 8 bits.
     * |        |          |0x2 = 16 bits.
     * |        |          |0x3 = 24 bits.
     * |        |          |0x4 = 32 bits.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for DMA Write mode.
     * |[3]     |DTR_CMD   |Double Transfer Rate Mode Enable Bit for Command Phase
     * |        |          |In master mode, SPI will transmit data at both rising edge and falling edge of SPI bus clock.
     * |        |          |0 = DTR mode Disabled (i.e. Single Transfer Rate (STR) mode enabled).
     * |        |          |1 = DTR mode Enabled.
     * |        |          |Note: Only used for DMA Write mode.
     * |[6:4]   |BM_CMD    |SPI Interface Bit Mode for Command Phase
     * |        |          |0x0 = Standard mode.
     * |        |          |0x1 = Dual mode.
     * |        |          |0x2 = Quad mode.
     * |        |          |0x3 = Octal mode.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for DMA Write mode.
     * |[10:8]  |DW_ADDR   |Data Width for Address Phase
     * |        |          |This specifies how many bits are transmitted in this phase.
     * |        |          |0x0 = No address phase.
     * |        |          |0x1 = 8 bits.
     * |        |          |0x2 = 16 bits.
     * |        |          |0x3 = 24 bits.
     * |        |          |0x4 = 32 bits.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for DMA Write mode.
     * |[11]    |DTR_ADDR  |Double Transfer Rate Mode Enable Bit for Address Phase
     * |        |          |In master mode, SPI will transmit data at both rising edge and falling edge of SPI bus clock.
     * |        |          |0 = DTR mode Disabled (i.e. Single Transfer Rate (STR) mode enabled).
     * |        |          |1 = DTR mode Enabled.
     * |        |          |Note: Only used for DMA Write mode.
     * |[14:12] |BM_ADDR   |SPI Interface Bit Mode for Address Phase
     * |        |          |0x0 = Standard mode.
     * |        |          |0x1 = Dual mode.
     * |        |          |0x2 = Quad mode.
     * |        |          |0x3 = Octal mode.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for DMA Write mode.
     * |[24]    |DTR_DATA  |Double Transfer Rate Mode Enable Bit for Data Phase
     * |        |          |In master mode, SPI will receive data at both rising edge and falling edge of SPI bus clock.
     * |        |          |0 = DTR mode Disabled (i.e. Single Transfer Rate (STR) mode enabled).
     * |        |          |1 = DTR mode Enabled.
     * |        |          |Note: Only used for DMA Write mode.
     * |[27:26] |PBO_DATA  |Program Data Byte Order of Program Data Phase for Octal SPI Flash
     * |        |          |0x0 = Byte order of program data to SPI Flash is byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7.
     * |        |          |0x1 = Byte order of program data to SPI Flash is byte7, byte6, byte5, byte4, byte3, byte2, byte1, byte0.
     * |        |          |0x2 = Byte order of program data to SPI Flash is byte1, byte0, byte3, byte2, byte5, byte4, byte7, byte6.
     * |        |          |0x3 = Byte order of program data to SPI Flash is byte6, byte7, byte4, byte5, byte2, byte3, byte0, byte1.
     * |        |          |Note: Only used for DMA Write mode.
     * |[30:28] |BM_DATA   |SPI Interface Bit Mode for Data Phase
     * |        |          |0x0 = Standard mode.
     * |        |          |0x1 = Dual mode.
     * |        |          |0x2 = Quad mode.
     * |        |          |0x3 = Octal mode.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for DMA Write mode.
     * @var SPIM_T::PHDMAR
     * Offset: 0x58  Phase Setting Register for DMA Read Mode
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |DW_CMD    |Data Width for Command Phase
     * |        |          |This specifies how many bits are transmitted in this phase.
     * |        |          |0x0 = No command phase.
     * |        |          |0x1 = 8 bits.
     * |        |          |0x2 = 16 bits.
     * |        |          |0x3 = 24 bits.
     * |        |          |0x4 = 32 bits.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for DMA Read mode.
     * |[3]     |DTR_CMD   |Double Transfer Rate Mode Enable Bit for Command Phase
     * |        |          |In master mode, SPI will transmit data at both rising edge and falling edge of SPI bus clock.
     * |        |          |0 = DTR mode Disabled (i.e. Single Transfer Rate (STR) mode enabled).
     * |        |          |1 = DTR mode Enabled.
     * |        |          |Note: Only used for DMA Read mode.
     * |[6:4]   |BM_CMD    |SPI Interface Bit Mode for Command Phase
     * |        |          |0x0 = Standard mode.
     * |        |          |0x1 = Dual mode.
     * |        |          |0x2 = Quad mode.
     * |        |          |0x3 = Octal mode.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for DMA Read mode.
     * |[10:8]  |DW_ADDR   |Data Width for Address Phase
     * |        |          |This specifies how many bits are transmitted in this phase.
     * |        |          |0x0 = No address phase.
     * |        |          |0x1 = 8 bits.
     * |        |          |0x2 = 16 bits.
     * |        |          |0x3 = 24 bits.
     * |        |          |0x4 = 32 bits.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for DMA Read mode.
     * |[11]    |DTR_ADDR  |Double Transfer Rate Mode Enable Bit for Address Phase
     * |        |          |In master mode, SPI will transmit data at both rising edge and falling edge of SPI bus clock.
     * |        |          |0 = DTR mode Disabled (i.e. Single Transfer Rate (STR) mode enabled).
     * |        |          |1 = DTR mode Enabled.
     * |        |          |Note: Only used for DMA Read mode.
     * |[14:12] |BM_ADDR   |SPI Interface Bit Mode for Address Phase
     * |        |          |0x0 = Standard mode.
     * |        |          |0x1 = Dual mode.
     * |        |          |0x2 = Quad mode.
     * |        |          |0x3 = Octal mode.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for DMA Read mode.
     * |[18:16] |DW_MODE   |Data Width for Mode Phase
     * |        |          |This specifies how many bits are transmitted in this phase.
     * |        |          |0x0 = No mode phase.
     * |        |          |0x1 = 8 bits.
     * |        |          |0x2 = 16 bits.
     * |        |          |0x3 = 24 bits.
     * |        |          |0x4 = 32 bits.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for DMA Read mode.
     * |[19]    |DTR_MODE  |Double Transfer Rate Mode Enable Bit for Mode Phase
     * |        |          |In master mode, SPI will transmit data at both rising edge and falling edge of SPI bus clock.
     * |        |          |0 = DTR mode Disabled (i.e. Single Transfer Rate (STR) mode enabled).
     * |        |          |1 = DTR mode Enabled.
     * |        |          |Note: Only used for DMA Read mode.
     * |[22:20] |BM_MODE   |SPI Interface Bit Mode for Mode Phase
     * |        |          |0x0 = Standard mode.
     * |        |          |0x1 = Dual mode.
     * |        |          |0x2 = Quad mode.
     * |        |          |0x3 = Octal mode.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for DMA Read mode.
     * |[24]    |DTR_DATA  |Double Transfer Rate Mode Enable Bit for Data Phase
     * |        |          |In master mode, SPI will receive data at both rising edge and falling edge of SPI bus clock.
     * |        |          |0 = DTR mode Disabled (i.e. Single Transfer Rate (STR) mode enabled).
     * |        |          |1 = DTR mode Enabled.
     * |        |          |Note: Only used for DMA Read mode.
     * |[25]    |RDQS_DATA |Read DQS Mode Enable Bit for Data Phase
     * |        |          |In SPI master mode, SPI uses read DQS signal, which is generated by SPI Flash, to sample RX data from SPI Flash when Read DQS mode is enabled.
     * |        |          |0 = Read DQS mode Disabled.
     * |        |          |1 = Read DQS mode Enabled.
     * |        |          |Note: Only used for DMA Read mode.
     * |[27:26] |RBO_DATA  |Received Data Byte Order of Received Data Phase for Octal SPI Flash
     * |        |          |0x0 = Byte order of received data to SPI Flash is byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7.
     * |        |          |0x1 = Byte order of received data to SPI Flash is byte7, byte6, byte5, byte4, byte3, byte2, byte1, byte0.
     * |        |          |0x2 = Byte order of received data to SPI Flash is byte1, byte0, byte3, byte2, byte5, byte4, byte7, byte6.
     * |        |          |0x3 = Byte order of received data to SPI Flash is byte6, byte7, byte4, byte5, byte2, byte3, byte0, byte1.
     * |        |          |Note: Only used for DMA Read mode.
     * |[30:28] |BM_DATA   |SPI Interface Bit Mode for Data Phase
     * |        |          |0x0 = Standard mode.
     * |        |          |0x1 = Dual mode.
     * |        |          |0x2 = Quad mode.
     * |        |          |0x3 = Octal mode.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for DMA Read mode.
     * @var SPIM_T::PHDMM
     * Offset: 0x5C  Phase Setting Register for Direct Memory Mapping Mode
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |DW_CMD    |Data Width for Command Phase
     * |        |          |This specifies how many bits are transmitted in this phase.
     * |        |          |0x0 = No command phase.
     * |        |          |0x1 = 8 bits.
     * |        |          |0x2 = 16 bits.
     * |        |          |0x3 = 24 bits.
     * |        |          |0x4 = 32 bits.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for Direct Memory Mapping mode.
     * |[3]     |DTR_CMD   |Double Transfer Rate Mode Enable Bit for Command Phase
     * |        |          |In master mode, SPI will transmit data at both rising edge and falling edge of SPI bus clock.
     * |        |          |0 = DTR mode Disabled (i.e. Single Transfer Rate (STR) mode enabled).
     * |        |          |1 = DTR mode Enabled.
     * |        |          |Note: Only used for Direct Memory Mapping mode.
     * |[6:4]   |BM_CMD    |SPI Interface Bit Mode for Command Phase
     * |        |          |0x0 = Standard mode.
     * |        |          |0x1 = Dual mode.
     * |        |          |0x2 = Quad mode.
     * |        |          |0x3 = Octal mode.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for Direct Memory Mapping mode.
     * |[10:8]  |DW_ADDR   |Data Width for Address Phase
     * |        |          |This specifies how many bits are transmitted in this phase.
     * |        |          |0x0 = No address phase.
     * |        |          |0x1 = 8 bits.
     * |        |          |0x2 = 16 bits.
     * |        |          |0x3 = 24 bits.
     * |        |          |0x4 = 32 bits.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for Direct Memory Mapping mode.
     * |[11]    |DTR_ADDR  |Double Transfer Rate Mode Enable Bit for Address Phase
     * |        |          |In master mode, SPI will transmit data at both rising edge and falling edge of SPI bus clock.
     * |        |          |0 = DTR mode Disabled (i.e. Single Transfer Rate (STR) mode enabled).
     * |        |          |1 = DTR mode Enabled.
     * |        |          |Note: Only used for Direct Memory Mapping mode.
     * |[14:12] |BM_ADDR   |SPI Interface Bit Mode for Address Phase
     * |        |          |0x0 = Standard mode.
     * |        |          |0x1 = Dual mode.
     * |        |          |0x2 = Quad mode.
     * |        |          |0x3 = Octal mode.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for Direct Memory Mapping mode.
     * |[18:16] |DW_MODE   |Data Width for Mode Phase
     * |        |          |This specifies how many bits are transmitted in this phase.
     * |        |          |0x0 = No mode phase.
     * |        |          |0x1 = 8 bits.
     * |        |          |0x2 = 16 bits.
     * |        |          |0x3 = 24 bits.
     * |        |          |0x4 = 32 bits.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for Direct Memory Mapping mode.
     * |[19]    |DTR_MODE  |Double Transfer Rate Mode Enable Bit for Mode Phase
     * |        |          |In master mode, SPI will transmit data at both rising edge and falling edge of SPI bus clock.
     * |        |          |0 = DTR mode Disabled (i.e. Single Transfer Rate (STR) mode enabled).
     * |        |          |1 = DTR mode Enabled.
     * |        |          |Note: Only used for Direct Memory Mapping mode.
     * |[22:20] |BM_MODE   |SPI Interface Bit Mode for Mode Phase
     * |        |          |0x0 = Standard mode.
     * |        |          |0x1 = Dual mode.
     * |        |          |0x2 = Quad mode.
     * |        |          |0x3 = Octal mode.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for Direct Memory Mapping mode.
     * |[24]    |DTR_DATA  |Double Transfer Rate Mode Enable Bit for Data Phase
     * |        |          |In master mode, SPI will receive data at both rising edge and falling edge of SPI bus clock.
     * |        |          |0 = DTR mode Disabled (i.e. Single Transfer Rate (STR) mode enabled).
     * |        |          |1 = DTR mode Enabled.
     * |        |          |Note: Only used for Direct Memory Mapping mode.
     * |[25]    |RDQS_DATA |Read DQS Mode Enable Bit for Data Phase
     * |        |          |In SPI master mode, SPI uses read DQS signal, which is generated by SPI Flash, to sample RX data from SPI Flash when Read DQS mode is enabled.
     * |        |          |0 = Read DQS mode Disabled.
     * |        |          |1 = Read DQS mode Enabled.
     * |        |          |Note: Only used for Direct Memory Mapping mode.
     * |[27:26] |RBO_DATA  |Received Data Byte Order of Received Data Phase for Octal SPI Flash
     * |        |          |0x0 = Byte order of received data to SPI Flash is byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7.
     * |        |          |0x1 = Byte order of received data to SPI Flash is byte7, byte6, byte5, byte4, byte3, byte2, byte1, byte0.
     * |        |          |0x2 = Byte order of received data to SPI Flash is byte1, byte0, byte3, byte2, byte5, byte4, byte7, byte6.
     * |        |          |0x3 = Byte order of received data to SPI Flash is byte6, byte7, byte4, byte5, byte2, byte3, byte0, byte1.
     * |        |          |Note: Only used for Direct Memory Mapping mode.
     * |[30:28] |BM_DATA   |SPI Interface Bit Mode for Data Phase
     * |        |          |0x0 = Standard mode.
     * |        |          |0x1 = Dual mode.
     * |        |          |0x2 = Quad mode.
     * |        |          |0x3 = Octal mode.
     * |        |          |Other = Reserved.
     * |        |          |Note: Only used for Direct Memory Mapping mode.
     * @var SPIM_T::DLL0
     * Offset: 0x68  DLL Control and Status Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DLLOLDO   |DLL OLDO Enable Bit
     * |        |          |0 = DLL OLDO is disabled, and DLL circuit is in power down mode.
     * |        |          |1 = DLL OLDO is enabled, and DLL circuit is in power enable mode.
     * |        |          |Note 1: DLL reference clock will start to send after user sets DLLOLDO to 0x1.
     * |[1]     |DLLOVRST  |DLL Output Valid Counter Reset Enable
     * |        |          |0 = DLL output valid counter is not reseted to 0x0 and it does not start to count.
     * |        |          |1 = DLL output valid counter is reseted to 0x0 and it starts to count from 0x0 to DLLOVNUM (SPIM_DLL1[15:0]).
     * |        |          |Note 1: This bit will be cleared automatically.
     * |        |          |Note 2: User needs to set DLLOVRST again to wait DLL output valid when the frequency of DLL reference clock is changed.
     * |[2]     |DLLCLKON  |DLL Clock Divider Circuit Status Bit (Read Only)
     * |        |          |0 = DLL clock divider circuit is disabled after DLL OLDO is enabled.
     * |        |          |1 = DLL clock divider circuit is enabled after DLL OLDO is enabled.
     * |        |          |Note: User needs to check DLL Datasheet to know time interval that DLL clock divider circuit can be enabled after DLL OLDO is enabled.
     * |[3]     |DLLLOCK   |DLL Lock Status Bit (Read Only)
     * |        |          |0 = DLL is not locked after DLL reference clock sends to DLL circuit.
     * |        |          |1 = DLL is locked after DLL reference clock sends to DLL circuit.
     * |        |          |Note: User needs to check DLL Datasheet to know DLL lock time.
     * |[4]     |DLLREADY  |DLL Output Ready Status Bit (Read Only)
     * |        |          |0 = DLL output is not ready, and DLL output is not ready to use.
     * |        |          |1 = DLL output is ready, and DLL output is ready to use.
     * |        |          |Note: User needs to check status of DLL output to ready before DLL output can be used correctly.
     * |[5]     |DLL_REF   |DLL Refresh Status Bit (Read Only)
     * |        |          |0 = The updating flow of new DLL delay step number DLL_DNUM (SPIM_DLL0[13:8]) of DLL circuit is finished.
     * |        |          |1 = DLL circuit is updating the new DLL delay step number DLL_DNUM (SPIM_DLL0[13:8]).
     * |        |          |Note 1: This bit will be cleared automatically.
     * |        |          |Note 2: User needs to check this bit to know that the refresh flow of DLL circuit is finished or not.
     * |[6]     |DLLATRDY  |DLL Auto Trim Ready Status Bit (Read Only)
     * |        |          |0 = DLL auto trim function is not in ready state when DLLATEN (SPIM_DLL0[16]) is enabled and DLL output is ready.
     * |        |          |1 = DLL auto trim function is in ready state when DLLATEN (SPIM_DLL0[16]) is enabled and DLL output is ready.
     * |        |          |Note1: User needs to check status of DLL auto trim function before DLL output can be used for flow of DLL auto trim correctly.
     * |        |          |Note2: This status bit is valid only when DLLATEN (SPIM_DLL0[16]) is enabled.
     * |        |          |Note3: This status bit will be cleared to 0 when DLLATEN (SPIM_DLL0[16]) is disabled.
     * |[12:8]  |DLL_DNUM  |DLL Delay Step Number
     * |        |          |Set DLL delay step number.
     * |        |          |0x0: No delay. (Default)
     * |        |          |0x1: DLL delay read data strobe of RWDS and DQS with 1x (SPIM output bus clock period)/32
     * |        |          |0x2: DLL delay read data strobe of RWDS and DQS with 2x (SPIM output bus clock period)/32
     * |        |          |0x3: DLL delay read data strobe of RWDS and DQS with 3x (SPIM output bus clock period)/32
     * |        |          |....
     * |        |          |0x1f: DLL delay read data strobe of RWDS and DQS with 31x (SPIM output bus clock period)/32
     * |[16]    |DLLATEN   |DLL Auto Trim Enable Bit
     * |        |          |0 = DLL auto trim function is disabled for DLL normal mode and DLL BIST1 mode.
     * |        |          |1 = DLL auto trim function is enabled for DLL normal mode and DLL BIST1 mode.
     * |        |          |Note: In DLL BIST2 mode (loop=1), this control register DLLATEN must be disabled.
     * |[17]    |DLLLOOP   |DLL Delay Time Self-Test Enable Bit
     * |        |          |0 = DLL Delay time self-test is disabled.
     * |        |          |1 = DLL Delay time self-test is enabled.
     * |[18]    |SIGINTEN  |Selection for Clock Source (RWDS ) of DLL Open Loop Delay Line
     * |        |          |0 = DLL clock source (DQS/RWDS) of DLL open loop delay line is from SPI Flash and HYPER device.
     * |        |          |1 = DLL clock source (DQS/RWDS) of DLL open loop delay line is from DLL internal clock divider.
     * |[21:20] |DLLDIVER  |Divider Number Selection of DLL Internal Clock Divder
     * |        |          |00 = Output frequency of DLL internal clock divdier equals to input frequency of DLL internal clock divdier.
     * |        |          |01 = Output frequency of DLL internal clock divdier is (input frequency of DLL internal clock divdier)/2.
     * |        |          |10 = Output frequency of DLL internal clock divdier is (input frequency of DLL internal clock divdier)/4.
     * |        |          |11 = Output frequency of DLL internal clock divdier is (input frequency of DLL internal clock divdier)/8.
     * |[23:22] |DLLFAST   |Band Selection of DLL Reference Clock
     * |        |          |00 = When frequency of DLL reference clock equals to 100MHz.
     * |        |          |01 = When frequency of DLL reference clock is greater than 100MHz.
     * @var SPIM_T::DLL1
     * Offset: 0x6C  DLL Control and Status Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |DLLOVNUM  |Clock Cycle Number between DLL Lock and DLL Output Valid
     * |        |          |Set cycle number of DLL output valid
     * |        |          |The hardware counter of DLL output valid will count from 0 to the DLLOVNUM after user sets DLLOVRST (SPIM_DLL0[1]) of DLL output valid counter to 0x1.
     * |        |          |Note 1: User needs to check datasheet of DLL to know the time interval between DLL lock done and DLL output valid.
     * |        |          |Note 2: The unit of this DLLOVNUM is cycle number of DLL reference clock.
     * |        |          |Note 3: The default value is 0x1388 when frequency of DLL reference clock is 100 MHz and valid time of DLL output is 50us.
     * |[31:16] |DLLLKNUM  |Clock Cycle Number between DLL Clock Divider Enable and DLL Lock
     * |        |          |Sets cycle number of DLL lock time after DLL clock divider is enabled.
     * |        |          |Note 1: User needs to check datasheet of SPIMu2019s DLL to know the time interval between DLL clock divider enable and DLL lock done.
     * |        |          |Note 2: The unit of this DLLLKNUM is cycle number of DLL reference clock.
     * |        |          |Note 3: The default value is 0x07D0 when frequency of DLL reference clock is 100 MHz and DLL lock time is 20us.
     * @var SPIM_T::DLL2
     * Offset: 0x70  DLL Control and Status Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |TRIMNUM   |Clock Cycle Number between DLL Output Valid and DLL Auto Trim Enable
     * |        |          |Sets cycle number of DLL auto trim to be enabled after DLL output is valid.
     * |        |          |Note 1: User needs to check datasheet of SPIMu2019s DLL to know the time interval between DLL output valid and DLL auto trim enable.
     * |        |          |Note 2: The unit of this TRIMNUM is cycle number of DLL reference clock.
     * |        |          |Note 3: The default value is 0x07D0 when frequency of DLL reference clock is 100 MHz and DLL lock time is 20us.
     * |[31:16] |CLKONNUM  |Clock Cycle Number between DLL OLDO Enable and DLL Clock Divider Enable
     * |        |          |Sets cycle number of DLL divider to be enabled after DLL OLDO is enabled.
     * |        |          |Note 1: User needs to check datasheet of SPIMu2019s DLL to know the time interval between DLL OLDO enable and DLL clock divider enable.
     * |        |          |Note 2: The unit of this CLKONNUM is cycle number of DLL reference clock.
     * |        |          |Note 3: The default value is 0x07D0 when frequency of DLL reference clock is 100 MHz and DLL lock time is 20us.
     * @var SPIM_T::HYPER_CMD
     * Offset: 0x80  Hyper Bus Command and Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |HYPCMD    |Hyper Bus Command and Status
     * |        |          |Write
     * |        |          |0001 = Reset Hyper RAM.
     * |        |          |0010 = Read Hyper RAM regsiter (16-Bit, Read Data[15:0].
     * |        |          |0101 = Exit From Hybrid Sleep and deep power down.
     * |        |          |0111 = Write Hyper RAM regsiter (16-Bit, Write Data[15:0].
     * |        |          |1000 = Read 2 Bytes (Read Data[15:0]) from memory space of Hyper Bus device.
     * |        |          |1001 = Read 4 Bytes (Read Data[31:0]) from memory space of Hyper Bus device.
     * |        |          |1100 = Write 1 Byte (Write Data[7:0]) to memory space of Hyper Bus device.
     * |        |          |1101 = Write 2 Bytes (Write Data[15:0]) to memory space of Hyper Bus device.
     * |        |          |1110 = Write 3 Bytes (Write Data[23:0]) to memory space of Hyper Bus device.
     * |        |          |1111 = Write 4 Bytes (Write Data[31:0]) to memory space of Hyper Bus device.
     * |        |          |Other value = reserved.
     * |        |          |Read
     * |        |          |0000 = Hyper Bus interface is Idle.
     * |        |          |Other value = Hyper Bus interface is busy.
     * |        |          |Note 1: When an operation is Done, the read value automatically return to 4u2019b0000.
     * |        |          |Note 2: The encryption and decryption of cipher are not supported, and it only uses to send command to Hyper Bus device, write control register to Hyper Bus device, and read status register from Hyper Bus device.
     * @var SPIM_T::HYPER_CONFIG1
     * Offset: 0x84  Hyper Bus Configuration Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |CSST      |Chip Select Setup Time to Next CK Rising Edge
     * |        |          |This field indicates the setup time between the chip select and the next CK rising edge
     * |        |          |10 = 3.5 HCLK cycles.
     * |        |          |11 = 4.5 HCLK cycles.
     * |        |          |Others = Reserved.
     * |[5:4]   |CSH       |Chip Select Hold Time After CK Falling Edge
     * |        |          |This field indicates the hold time between the last CK falling edge and chip select
     * |        |          |00 = 0.5 HCLK cycles.
     * |        |          |01 = 1.5 HCLK cycles.
     * |        |          |10 = 2.5 HCLK cycles.
     * |        |          |11 = 3.5 HCLK cycles.
     * |[11:8]  |CSHI      |Chip Select High between Transaction
     * |        |          |This field indicates the inactive period between two Hyper Bus transactions
     * |        |          |0010 = 2 HCLK cycles.
     * |        |          |0011 = 3 HCLK cycles.
     * |        |          |....
     * |        |          |1111 = 15 HCLK cycles.
     * |        |          |Others = Reserved.
     * |        |          |Note: This field must meet the Hyper bus deviceu2019s specification of tCSHI.
     * |[31:16] |CSMAXLT   |Chip Select Maximum Low Time
     * |        |          |This field indicates the maximum Low period of the chip select (CS#) in one transaction
     * |        |          |0 = 1 HCLK cycle.
     * |        |          |1 = 2 HCLK cycles.
     * |        |          |2 = 3 HCLK cycles.
     * |        |          |3 = 4 HCLK cycles.
     * |        |          |....
     * |        |          |749 = 750 HCLK cycles (3750ns @200 MHz).
     * |        |          |....
     * |        |          |65534 = 65535 HCLK cycles.
     * |        |          |65535 = 65536 HCLK cycles.
     * |        |          |(1) Setting formula of CSMAXLT when CIPHOFF (SPIM_CTL0[0]) sets to 1u2019b1 :
     * |        |          |CSMAXLT = (tCSM/HCLK period) u2013 8*2*(DIVIDER) u2013 21
     * |        |          |(2) Setting formula of CSMAXLT when CIPHOFF (SPIM_CTL0[0]) sets to 1u2019b0 :
     * |        |          |CSMAXLT = (tCSM/HCLK period) u2013 8*2*(DIVIDER) u2013 54
     * |        |          |Note1 : This field inidcates the timing of Hyper RAM Chip Select specification so that it has to relative the frequency of HCLK and the DIVIDER (SPIM_CTL1[31:16]).
     * |        |          |Note2 : This register must be set to 0xFFFF for Hyper Flash device.
     * @var SPIM_T::HYPER_CONFIG2
     * Offset: 0x88  Hyper Bus Configuration Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[12:8]  |ACCTWR    |Initial Access Time for Write Data to HYPER Device
     * |        |          |This field indicates the initial access cycles for write transaction of the Hyper Bus
     * |        |          |00001 = 1 CK cycles.
     * |        |          |00010 = 2 CK cycles.
     * |        |          |00011 = 3 CK cycles.
     * |        |          |00100 = 4 CK cycles (default).
     * |        |          |00101 = 5 CK cycles.
     * |        |          |00110 = 6 CK cycles.
     * |        |          |00111 = 7 CK cycles.
     * |        |          |....
     * |        |          |11111 = 31 CK cycles.
     * |        |          |Others = Reserved.
     * |        |          |Note1 : This field must be set to the same value as "Initial Latency" for write operations in Configuration Register 0 in HYPER device.
     * |        |          |Note2 : If user sets value to 0000, the hardware will modify it to 0001 automatically.
     * |[23:16] |RSTNLT    |HYPER Bus Device RESETN Low Time
     * |        |          |This field indicates the low period of HYPER bus device RESETN (RESET#)
     * |        |          |1 = 2 HCLK cycles.
     * |        |          |2 = 3 HCLK cycles.
     * |        |          |3 = 4 HCLK cycles.
     * |        |          |....
     * |        |          |255 = 255 HCLK cycles.
     * |        |          |Note: This field inidcates the timing of Hyper bus device RESET# specification so that it has to relative the frequency of HCLK.
     * |[28:24] |ACCTRD    |Initial Access Time for Read Data from HYPER Device
     * |        |          |This field indicates the initial access cycles for read transaction of the Hyper Bus
     * |        |          |00001 = 1 CK cycles.
     * |        |          |00010 = 2 CK cycles.
     * |        |          |00011 = 3 CK cycles.
     * |        |          |00100 = 4 CK cycles (default).
     * |        |          |00101 = 5 CK cycles.
     * |        |          |00110 = 6 CK cycles.
     * |        |          |00111 = 7 CK cycles.
     * |        |          |....
     * |        |          |11111 = 31 CK cycles.
     * |        |          |Others = Reserved.
     * |        |          |Note1 : This field must be set to the same value as "Initial Latency" for read operations in Configuration Register 0 in HYPER device.
     * |        |          |Note2 : If user sets value to 0000, the hardware will modify it to 0001 automatically.
     * @var SPIM_T::HYPER_ADR
     * Offset: 0x8C  Hyper Bus Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[24:0]  |HBADDR    |Hyper Bus Register Address
     * |        |          |Memory Space Range:
     * |        |          | 0x0000_0000 ~ 0x01FF_FFFF
     * |        |          |Register Space Range:
     * |        |          | 0x0000_0000 = Identification Register 0
     * |        |          | 0x0000_0002 = Identification Register 1
     * |        |          | 0x0000_1000 = Configuration Register 0
     * |        |          | 0x0000_1002 = Configuration Register 1
     * |        |          |Note 1: It is "Byte" address.
     * |        |          |Note 2: Up to 32MBytes of memory space is supported.
     * @var SPIM_T::HYPER_WDATA
     * Offset: 0x90  Hyper Bus 32-Bits Write Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |HBWDATA   |Hyper Bus 32-Bits Write Data
     * |        |          |To write 1 Byte to Hyper bus device, Byte 0 (Data[7:0]) is used
     * |        |          |To write 2 Bytes to Hyper bus device, Byte 1~0 (Data[15:0]) is used
     * |        |          |To write 3 Bytes to Hyper bus device, Byte 2~0 (Data[23:0]) is used
     * |        |          |To write 4 Bytes to Hyper bus device, Byte 3~0 (Data[31:0]) is used
     * @var SPIM_T::HYPER_RDATA
     * Offset: 0x94  Hyper Bus 32-Bits Read Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |HBRDATA   |Hyper Bus 32-bits Read Data
     * |        |          |32-Bits data read from Hyper bus device
     * |        |          |Note 1: The byte order of read data is little endian only.
     * @var SPIM_T::HYPER_INTEN
     * Offset: 0x98  Hyper Bus Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |OPINTEN   |Hyper Bus Operation Done Interrupt Enable
     * |        |          |0 = Operation done interrupt is Disabled.
     * |        |          |1 = Operation done interrupt is Enabled.
     * @var SPIM_T::HYPER_INTSTS
     * Offset: 0x9C  Hyper Bus Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |OPDONE    |Hyper Bus Operation Done Interrupt
     * |        |          |0 = Hyper Bus operation is busy.
     * |        |          |1 = Hyper Bus operation is done.

     */
    __IO uint32_t CTL0;                  /*!< [0x0000] Control and Status Register 0                                    */
    __IO uint32_t CTL1;                  /*!< [0x0004] Control Register 1                                               */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t RXCLKDLY;              /*!< [0x000c] RX Clock Delay Control Register                                  */
    __I  uint32_t RX[4];                 /*!< [0x0010 ~ 0x001C] Data Receive Register 0                                          */
    //__I  uint32_t RX0;                   /*!< [0x0010] Data Receive Register 0                                          */
    //__I  uint32_t RX1;                   /*!< [0x0014] Data Receive Register 1                                          */
    //__I  uint32_t RX2;                   /*!< [0x0018] Data Receive Register 2                                          */
    //__I  uint32_t RX3;                   /*!< [0x001c] Data Receive Register 3                                          */
    __IO uint32_t TX[4];                 /*!< [0x0020 ~ 0x002C] Data Transmit Register 0                                         */
    //__IO uint32_t TX0;                   /*!< [0x0020] Data Transmit Register 0                                         */
    //__IO uint32_t TX1;                   /*!< [0x0024] Data Transmit Register 1                                         */
    //__IO uint32_t TX2;                   /*!< [0x0028] Data Transmit Register 2                                         */
    //__IO uint32_t TX3;                   /*!< [0x002c] Data Transmit Register 3                                         */
    __IO uint32_t SRAMADDR;              /*!< [0x0030] SRAM Memory Address Register                                     */
    __IO uint32_t DMACNT;                /*!< [0x0034] DMA Transfer Byte Count Register                                 */
    __IO uint32_t FADDR;                 /*!< [0x0038] Address Register for SPI Flash and Hyper Device                  */
    __I  uint32_t RESERVE1[2];
    __IO uint32_t DMMCTL;                /*!< [0x0044] Direct Memory Mapping Mode Control Register                      */
    __IO uint32_t CTL2;                  /*!< [0x0048] Control Register 2                                               */
    __IO uint32_t CMDCODE;               /*!< [0x004c] Command Code Register                                            */
    __IO uint32_t MODE;                  /*!< [0x0050] Mode Data Register                                               */
    __IO uint32_t PHDMAW;                /*!< [0x0054] Phase Setting Register for DMA Write Mode                        */
    __IO uint32_t PHDMAR;                /*!< [0x0058] Phase Setting Register for DMA Read Mode                         */
    __IO uint32_t PHDMM;                 /*!< [0x005c] Phase Setting Register for Direct Memory Mapping Mode            */
    __I  uint32_t RESERVE2[2];
    __IO uint32_t DLL0;                  /*!< [0x0068] DLL Control and Status Register 0                                */
    __IO uint32_t DLL1;                  /*!< [0x006c] DLL Control and Status Register 1                                */
    __IO uint32_t DLL2;                  /*!< [0x0070] DLL Control and Status Register 2                                */
    __I  uint32_t RESERVE3[3];
    __IO uint32_t HYPER_CMD;             /*!< [0x0080] Hyper Bus Command and Status Register                            */
    __IO uint32_t HYPER_CONFIG1;         /*!< [0x0084] Hyper Bus Configuration Register 1                               */
    __IO uint32_t HYPER_CONFIG2;         /*!< [0x0088] Hyper Bus Configuration Register 2                               */
    __IO uint32_t HYPER_ADR;             /*!< [0x008c] Hyper Bus Address Register                                       */
    __IO uint32_t HYPER_WDATA;           /*!< [0x0090] Hyper Bus 32-Bits Write Data Register                            */
    __I  uint32_t HYPER_RDATA;           /*!< [0x0094] Hyper Bus 32-Bits Read Data Register                             */
    __IO uint32_t HYPER_INTEN;           /*!< [0x0098] Hyper Bus Interrupt Enable Register                              */
    __IO uint32_t HYPER_INTSTS;          /*!< [0x009c] Hyper Bus Interrupt Status Register                              */
} SPIM_T;

/**
    @addtogroup SPIM_CONST SPIM Bit Field Definition
    Constant Definitions for SPIM Controller
@{ */

#define SPIM_CTL0_CIPHOFF_Pos            (0)                                               /*!< SPIM_T::CTL0: CIPHOFF Position         */
#define SPIM_CTL0_CIPHOFF_Msk            (0x1ul << SPIM_CTL0_CIPHOFF_Pos)                  /*!< SPIM_T::CTL0: CIPHOFF Mask             */

#define SPIM_CTL0_BALEN_Pos              (2)                                               /*!< SPIM_T::CTL0: BALEN Position           */
#define SPIM_CTL0_BALEN_Msk              (0x1ul << SPIM_CTL0_BALEN_Pos)                    /*!< SPIM_T::CTL0: BALEN Mask               */

#define SPIM_CTL0_HYPER_EN_Pos           (3)                                               /*!< SPIM_T::CTL0: HYPER_EN Position        */
#define SPIM_CTL0_HYPER_EN_Msk           (0x1ul << SPIM_CTL0_HYPER_EN_Pos)                 /*!< SPIM_T::CTL0: HYPER_EN Mask            */

#define SPIM_CTL0_B4ADDREN_Pos           (5)                                               /*!< SPIM_T::CTL0: B4ADDREN Position        */
#define SPIM_CTL0_B4ADDREN_Msk           (0x1ul << SPIM_CTL0_B4ADDREN_Pos)                 /*!< SPIM_T::CTL0: B4ADDREN Mask            */

#define SPIM_CTL0_IEN_Pos                (6)                                               /*!< SPIM_T::CTL0: IEN Position             */
#define SPIM_CTL0_IEN_Msk                (0x1ul << SPIM_CTL0_IEN_Pos)                      /*!< SPIM_T::CTL0: IEN Mask                 */

#define SPIM_CTL0_IF_Pos                 (7)                                               /*!< SPIM_T::CTL0: IF Position              */
#define SPIM_CTL0_IF_Msk                 (0x1ul << SPIM_CTL0_IF_Pos)                       /*!< SPIM_T::CTL0: IF Mask                  */

#define SPIM_CTL0_DWIDTH_Pos             (8)                                               /*!< SPIM_T::CTL0: DWIDTH Position          */
#define SPIM_CTL0_DWIDTH_Msk             (0x1ful << SPIM_CTL0_DWIDTH_Pos)                  /*!< SPIM_T::CTL0: DWIDTH Mask              */

#define SPIM_CTL0_BURSTNUM_Pos           (13)                                              /*!< SPIM_T::CTL0: BURSTNUM Position        */
#define SPIM_CTL0_BURSTNUM_Msk           (0x3ul << SPIM_CTL0_BURSTNUM_Pos)                 /*!< SPIM_T::CTL0: BURSTNUM Mask            */

#define SPIM_CTL0_QDIODIR_Pos            (15)                                              /*!< SPIM_T::CTL0: QDIODIR Position         */
#define SPIM_CTL0_QDIODIR_Msk            (0x1ul << SPIM_CTL0_QDIODIR_Pos)                  /*!< SPIM_T::CTL0: QDIODIR Mask             */

#define SPIM_CTL0_SUSPITV_Pos            (16)                                              /*!< SPIM_T::CTL0: SUSPITV Position         */
#define SPIM_CTL0_SUSPITV_Msk            (0xful << SPIM_CTL0_SUSPITV_Pos)                  /*!< SPIM_T::CTL0: SUSPITV Mask             */

#define SPIM_CTL0_BITMODE_Pos            (20)                                              /*!< SPIM_T::CTL0: BITMODE Position         */
#define SPIM_CTL0_BITMODE_Msk            (0x3ul << SPIM_CTL0_BITMODE_Pos)                  /*!< SPIM_T::CTL0: BITMODE Mask             */

#define SPIM_CTL0_OPMODE_Pos             (22)                                              /*!< SPIM_T::CTL0: OPMODE Position          */
#define SPIM_CTL0_OPMODE_Msk             (0x3ul << SPIM_CTL0_OPMODE_Pos)                   /*!< SPIM_T::CTL0: OPMODE Mask              */

#define SPIM_CTL0_DTR_NORM_Pos           (24)                                              /*!< SPIM_T::CTL0: DTR_NORM Position        */
#define SPIM_CTL0_DTR_NORM_Msk           (0x1ul << SPIM_CTL0_DTR_NORM_Pos)                 /*!< SPIM_T::CTL0: DTR_NORM Mask            */

#define SPIM_CTL0_RDQS_NORM_Pos          (25)                                              /*!< SPIM_T::CTL0: RDQS_NORM Position       */
#define SPIM_CTL0_RDQS_NORM_Msk          (0x1ul << SPIM_CTL0_RDQS_NORM_Pos)                /*!< SPIM_T::CTL0: RDQS_NORM Mask           */

#define SPIM_CTL0_RBO_NORM_Pos           (26)                                              /*!< SPIM_T::CTL0: RBO_NORM Position        */
#define SPIM_CTL0_RBO_NORM_Msk           (0x3ul << SPIM_CTL0_RBO_NORM_Pos)                 /*!< SPIM_T::CTL0: RBO_NORM Mask            */

#define SPIM_CTL1_SPIMEN_Pos             (0)                                               /*!< SPIM_T::CTL1: SPIMEN Position          */
#define SPIM_CTL1_SPIMEN_Msk             (0x1ul << SPIM_CTL1_SPIMEN_Pos)                   /*!< SPIM_T::CTL1: SPIMEN Mask              */

#if (SPIM_REG_CACHE == 1) //TESTCHIP_ONLY not support
    #define SPIM_CTL1_CACHEOFF_Pos           (1)                                               /*!< SPIM_T::CTL1: CACHEOFF Position        */
    #define SPIM_CTL1_CACHEOFF_Msk           (0x1ul << SPIM_CTL1_CACHEOFF_Pos)                 /*!< SPIM_T::CTL1: CACHEOFF Mask            */

    #define SPIM_CTL1_CDINVAL_Pos            (3)                                               /*!< SPIM_T::CTL1: CDINVAL Position         */
    #define SPIM_CTL1_CDINVAL_Msk            (0x1ul << SPIM_CTL1_CDINVAL_Pos)                  /*!< SPIM_T::CTL1: CDINVAL Mask             */
#endif

#define SPIM_CTL1_SS_Pos                 (4)                                               /*!< SPIM_T::CTL1: SS Position              */
#define SPIM_CTL1_SS_Msk                 (0x1ul << SPIM_CTL1_SS_Pos)                       /*!< SPIM_T::CTL1: SS Mask                  */

#define SPIM_CTL1_SSACTPOL_Pos           (5)                                               /*!< SPIM_T::CTL1: SSACTPOL Position        */
#define SPIM_CTL1_SSACTPOL_Msk           (0x1ul << SPIM_CTL1_SSACTPOL_Pos)                 /*!< SPIM_T::CTL1: SSACTPOL Mask            */

#if (SPIM_REG_CACHE == 1) //TESTCHIP_ONLY not support
    #define SPIM_CTL1_CAWRTHEN_Pos           (6)                                               /*!< SPIM_T::CTL1: CAWRTHEN Position        */
    #define SPIM_CTL1_CAWRTHEN_Msk           (0x1ul << SPIM_CTL1_CAWRTHEN_Pos)                 /*!< SPIM_T::CTL1: CAWRTHEN Mask            */

    #define SPIM_CTL1_AUTOSCLN_Pos           (7)                                               /*!< SPIM_T::CTL1: AUTOSCLN Position        */
    #define SPIM_CTL1_AUTOSCLN_Msk           (0x1ul << SPIM_CTL1_AUTOSCLN_Pos)                 /*!< SPIM_T::CTL1: AUTOSCLN Mask            */
#endif

#define SPIM_CTL1_IDLETIME_Pos           (8)                                               /*!< SPIM_T::CTL1: IDLETIME Position        */
#define SPIM_CTL1_IDLETIME_Msk           (0xful << SPIM_CTL1_IDLETIME_Pos)                 /*!< SPIM_T::CTL1: IDLETIME Mask            */

#if (SPIM_REG_CACHE == 1) //TESTCHIP_ONLY not support
    #define SPIM_CTL1_UPDCLNUM_Pos           (12)                                              /*!< SPIM_T::CTL1: UPDCLNUM Position        */
    #define SPIM_CTL1_UPDCLNUM_Msk           (0xful << SPIM_CTL1_UPDCLNUM_Pos)                 /*!< SPIM_T::CTL1: UPDCLNUM Mask            */
#endif

#define SPIM_CTL1_DIVIDER_Pos            (16)                                              /*!< SPIM_T::CTL1: DIVIDER Position         */
#define SPIM_CTL1_DIVIDER_Msk            (0xfffful << SPIM_CTL1_DIVIDER_Pos)               /*!< SPIM_T::CTL1: DIVIDER Mask             */

#define SPIM_RXCLKDLY_RDDLYSEL_Pos       (0)                                               /*!< SPIM_T::RXCLKDLY: RDDLYSEL Position    */
#define SPIM_RXCLKDLY_RDDLYSEL_Msk       (0xful << SPIM_RXCLKDLY_RDDLYSEL_Pos)             /*!< SPIM_T::RXCLKDLY: RDDLYSEL Mask        */

#define SPIM_RX0_RXDAT_Pos               (0)                                               /*!< SPIM_T::RX0: RXDAT Position            */
#define SPIM_RX0_RXDAT_Msk               (0xfffffffful << SPIM_RX0_RXDAT_Pos)              /*!< SPIM_T::RX0: RXDAT Mask                */

#define SPIM_RX1_RXDAT_Pos               (0)                                               /*!< SPIM_T::RX1: RXDAT Position            */
#define SPIM_RX1_RXDAT_Msk               (0xfffffffful << SPIM_RX1_RXDAT_Pos)              /*!< SPIM_T::RX1: RXDAT Mask                */

#define SPIM_RX2_RXDAT_Pos               (0)                                               /*!< SPIM_T::RX2: RXDAT Position            */
#define SPIM_RX2_RXDAT_Msk               (0xfffffffful << SPIM_RX2_RXDAT_Pos)              /*!< SPIM_T::RX2: RXDAT Mask                */

#define SPIM_RX3_RXDAT_Pos               (0)                                               /*!< SPIM_T::RX3: RXDAT Position            */
#define SPIM_RX3_RXDAT_Msk               (0xfffffffful << SPIM_RX3_RXDAT_Pos)              /*!< SPIM_T::RX3: RXDAT Mask                */

#define SPIM_TX0_TXDAT_Pos               (0)                                               /*!< SPIM_T::TX0: TXDAT Position            */
#define SPIM_TX0_TXDAT_Msk               (0xfffffffful << SPIM_TX0_TXDAT_Pos)              /*!< SPIM_T::TX0: TXDAT Mask                */

#define SPIM_TX1_TXDAT_Pos               (0)                                               /*!< SPIM_T::TX1: TXDAT Position            */
#define SPIM_TX1_TXDAT_Msk               (0xfffffffful << SPIM_TX1_TXDAT_Pos)              /*!< SPIM_T::TX1: TXDAT Mask                */

#define SPIM_TX2_TXDAT_Pos               (0)                                               /*!< SPIM_T::TX2: TXDAT Position            */
#define SPIM_TX2_TXDAT_Msk               (0xfffffffful << SPIM_TX2_TXDAT_Pos)              /*!< SPIM_T::TX2: TXDAT Mask                */

#define SPIM_TX3_TXDAT_Pos               (0)                                               /*!< SPIM_T::TX3: TXDAT Position            */
#define SPIM_TX3_TXDAT_Msk               (0xfffffffful << SPIM_TX3_TXDAT_Pos)              /*!< SPIM_T::TX3: TXDAT Mask                */

#define SPIM_SRAMADDR_ADDR_Pos           (0)                                               /*!< SPIM_T::SRAMADDR: ADDR Position        */
#define SPIM_SRAMADDR_ADDR_Msk           (0xfffffffful << SPIM_SRAMADDR_ADDR_Pos)          /*!< SPIM_T::SRAMADDR: ADDR Mask            */

#define SPIM_DMACNT_DMACNT_Pos           (0)                                               /*!< SPIM_T::DMACNT: DMACNT Position        */
#define SPIM_DMACNT_DMACNT_Msk           (0xfffffful << SPIM_DMACNT_DMACNT_Pos)            /*!< SPIM_T::DMACNT: DMACNT Mask            */

#define SPIM_FADDR_ADDR_Pos              (0)                                               /*!< SPIM_T::FADDR: ADDR Position           */
#define SPIM_FADDR_ADDR_Msk              (0xfffffffful << SPIM_FADDR_ADDR_Pos)             /*!< SPIM_T::FADDR: ADDR Mask               */

#define SPIM_DMMCTL_ACTSCLKT_Pos         (0)                                               /*!< SPIM_T::DMMCTL: ACTSCLKT Position      */
#define SPIM_DMMCTL_ACTSCLKT_Msk         (0xfful << SPIM_DMMCTL_ACTSCLKT_Pos)              /*!< SPIM_T::DMMCTL: ACTSCLKT Mask          */

#define SPIM_DMMCTL_DESELTIM_Pos         (16)                                              /*!< SPIM_T::DMMCTL: DESELTIM Position      */
#define SPIM_DMMCTL_DESELTIM_Msk         (0xfful << SPIM_DMMCTL_DESELTIM_Pos)              /*!< SPIM_T::DMMCTL: DESELTIM Mask          */

#define SPIM_DMMCTL_BWEN_Pos             (24)                                              /*!< SPIM_T::DMMCTL: BWEN Position          */
#define SPIM_DMMCTL_BWEN_Msk             (0x1ul << SPIM_DMMCTL_BWEN_Pos)                   /*!< SPIM_T::DMMCTL: BWEN Mask              */

#define SPIM_DMMCTL_CREN_Pos             (25)                                              /*!< SPIM_T::DMMCTL: CREN Position          */
#define SPIM_DMMCTL_CREN_Msk             (0x1ul << SPIM_DMMCTL_CREN_Pos)                   /*!< SPIM_T::DMMCTL: CREN Mask              */

#define SPIM_DMMCTL_UACTSCLK_Pos         (26)                                              /*!< SPIM_T::DMMCTL: UACTSCLK Position      */
#define SPIM_DMMCTL_UACTSCLK_Msk         (0x1ul << SPIM_DMMCTL_UACTSCLK_Pos)               /*!< SPIM_T::DMMCTL: UACTSCLK Mask          */

#define SPIM_DMMCTL_HYPDONE_Pos          (27)                                              /*!< SPIM_T::DMMCTL: HYPDONE Position       */
#define SPIM_DMMCTL_HYPDONE_Msk          (0x1ul << SPIM_DMMCTL_HYPDONE_Pos)                /*!< SPIM_T::DMMCTL: HYPDONE Mask           */

#define SPIM_CTL2_DC_DMAR_Pos            (8)                                               /*!< SPIM_T::CTL2: DC_DMAR Position         */
#define SPIM_CTL2_DC_DMAR_Msk            (0xfful << SPIM_CTL2_DC_DMAR_Pos)                 /*!< SPIM_T::CTL2: DC_DMAR Mask             */

#define SPIM_CTL2_DC_DMM_Pos             (16)                                              /*!< SPIM_T::CTL2: DC_DMM Position          */
#define SPIM_CTL2_DC_DMM_Msk             (0xfful << SPIM_CTL2_DC_DMM_Pos)                  /*!< SPIM_T::CTL2: DC_DMM Mask              */

#define SPIM_CMDCODE_CMDCODE_Pos         (0)                                               /*!< SPIM_T::CMDCODE: CMDCODE Position      */
#define SPIM_CMDCODE_CMDCODE_Msk         (0xfffffffful << SPIM_CMDCODE_CMDCODE_Pos)        /*!< SPIM_T::CMDCODE: CMDCODE Mask          */

#define SPIM_MODE_MODEDATA_Pos           (0)                                               /*!< SPIM_T::MODE: MODEDATA Position        */
#define SPIM_MODE_MODEDATA_Msk           (0xfffffffful << SPIM_MODE_MODEDATA_Pos)          /*!< SPIM_T::MODE: MODEDATA Mask            */

#define SPIM_PHDMAW_DW_CMD_Pos           (0)                                               /*!< SPIM_T::PHDMAW: DW_CMD Position        */
#define SPIM_PHDMAW_DW_CMD_Msk           (0x7ul << SPIM_PHDMAW_DW_CMD_Pos)                 /*!< SPIM_T::PHDMAW: DW_CMD Mask            */

#define SPIM_PHDMAW_DTR_CMD_Pos          (3)                                               /*!< SPIM_T::PHDMAW: DTR_CMD Position       */
#define SPIM_PHDMAW_DTR_CMD_Msk          (0x1ul << SPIM_PHDMAW_DTR_CMD_Pos)                /*!< SPIM_T::PHDMAW: DTR_CMD Mask           */

#define SPIM_PHDMAW_BM_CMD_Pos           (4)                                               /*!< SPIM_T::PHDMAW: BM_CMD Position        */
#define SPIM_PHDMAW_BM_CMD_Msk           (0x7ul << SPIM_PHDMAW_BM_CMD_Pos)                 /*!< SPIM_T::PHDMAW: BM_CMD Mask            */

#define SPIM_PHDMAW_DW_ADDR_Pos          (8)                                               /*!< SPIM_T::PHDMAW: DW_ADDR Position       */
#define SPIM_PHDMAW_DW_ADDR_Msk          (0x7ul << SPIM_PHDMAW_DW_ADDR_Pos)                /*!< SPIM_T::PHDMAW: DW_ADDR Mask           */

#define SPIM_PHDMAW_DTR_ADDR_Pos         (11)                                              /*!< SPIM_T::PHDMAW: DTR_ADDR Position      */
#define SPIM_PHDMAW_DTR_ADDR_Msk         (0x1ul << SPIM_PHDMAW_DTR_ADDR_Pos)               /*!< SPIM_T::PHDMAW: DTR_ADDR Mask          */

#define SPIM_PHDMAW_BM_ADDR_Pos          (12)                                              /*!< SPIM_T::PHDMAW: BM_ADDR Position       */
#define SPIM_PHDMAW_BM_ADDR_Msk          (0x7ul << SPIM_PHDMAW_BM_ADDR_Pos)                /*!< SPIM_T::PHDMAW: BM_ADDR Mask           */

#define SPIM_PHDMAW_DTR_DATA_Pos         (24)                                              /*!< SPIM_T::PHDMAW: DTR_DATA Position      */
#define SPIM_PHDMAW_DTR_DATA_Msk         (0x1ul << SPIM_PHDMAW_DTR_DATA_Pos)               /*!< SPIM_T::PHDMAW: DTR_DATA Mask          */

#define SPIM_PHDMAW_PBO_DATA_Pos         (26)                                              /*!< SPIM_T::PHDMAW: PBO_DATA Position      */
#define SPIM_PHDMAW_PBO_DATA_Msk         (0x3ul << SPIM_PHDMAW_PBO_DATA_Pos)               /*!< SPIM_T::PHDMAW: PBO_DATA Mask          */

#define SPIM_PHDMAW_BM_DATA_Pos          (28)                                              /*!< SPIM_T::PHDMAW: BM_DATA Position       */
#define SPIM_PHDMAW_BM_DATA_Msk          (0x7ul << SPIM_PHDMAW_BM_DATA_Pos)                /*!< SPIM_T::PHDMAW: BM_DATA Mask           */

#define SPIM_PHDMAR_DW_CMD_Pos           (0)                                               /*!< SPIM_T::PHDMAR: DW_CMD Position        */
#define SPIM_PHDMAR_DW_CMD_Msk           (0x7ul << SPIM_PHDMAR_DW_CMD_Pos)                 /*!< SPIM_T::PHDMAR: DW_CMD Mask            */

#define SPIM_PHDMAR_DTR_CMD_Pos          (3)                                               /*!< SPIM_T::PHDMAR: DTR_CMD Position       */
#define SPIM_PHDMAR_DTR_CMD_Msk          (0x1ul << SPIM_PHDMAR_DTR_CMD_Pos)                /*!< SPIM_T::PHDMAR: DTR_CMD Mask           */

#define SPIM_PHDMAR_BM_CMD_Pos           (4)                                               /*!< SPIM_T::PHDMAR: BM_CMD Position        */
#define SPIM_PHDMAR_BM_CMD_Msk           (0x7ul << SPIM_PHDMAR_BM_CMD_Pos)                 /*!< SPIM_T::PHDMAR: BM_CMD Mask            */

#define SPIM_PHDMAR_DW_ADDR_Pos          (8)                                               /*!< SPIM_T::PHDMAR: DW_ADDR Position       */
#define SPIM_PHDMAR_DW_ADDR_Msk          (0x7ul << SPIM_PHDMAR_DW_ADDR_Pos)                /*!< SPIM_T::PHDMAR: DW_ADDR Mask           */

#define SPIM_PHDMAR_DTR_ADDR_Pos         (11)                                              /*!< SPIM_T::PHDMAR: DTR_ADDR Position      */
#define SPIM_PHDMAR_DTR_ADDR_Msk         (0x1ul << SPIM_PHDMAR_DTR_ADDR_Pos)               /*!< SPIM_T::PHDMAR: DTR_ADDR Mask          */

#define SPIM_PHDMAR_BM_ADDR_Pos          (12)                                              /*!< SPIM_T::PHDMAR: BM_ADDR Position       */
#define SPIM_PHDMAR_BM_ADDR_Msk          (0x7ul << SPIM_PHDMAR_BM_ADDR_Pos)                /*!< SPIM_T::PHDMAR: BM_ADDR Mask           */

#define SPIM_PHDMAR_DW_MODE_Pos          (16)                                              /*!< SPIM_T::PHDMAR: DW_MODE Position       */
#define SPIM_PHDMAR_DW_MODE_Msk          (0x7ul << SPIM_PHDMAR_DW_MODE_Pos)                /*!< SPIM_T::PHDMAR: DW_MODE Mask           */

#define SPIM_PHDMAR_DTR_MODE_Pos         (19)                                              /*!< SPIM_T::PHDMAR: DTR_MODE Position      */
#define SPIM_PHDMAR_DTR_MODE_Msk         (0x1ul << SPIM_PHDMAR_DTR_MODE_Pos)               /*!< SPIM_T::PHDMAR: DTR_MODE Mask          */

#define SPIM_PHDMAR_BM_MODE_Pos          (20)                                              /*!< SPIM_T::PHDMAR: BM_MODE Position       */
#define SPIM_PHDMAR_BM_MODE_Msk          (0x7ul << SPIM_PHDMAR_BM_MODE_Pos)                /*!< SPIM_T::PHDMAR: BM_MODE Mask           */

#define SPIM_PHDMAR_DTR_DATA_Pos         (24)                                              /*!< SPIM_T::PHDMAR: DTR_DATA Position      */
#define SPIM_PHDMAR_DTR_DATA_Msk         (0x1ul << SPIM_PHDMAR_DTR_DATA_Pos)               /*!< SPIM_T::PHDMAR: DTR_DATA Mask          */

#define SPIM_PHDMAR_RDQS_DATA_Pos        (25)                                              /*!< SPIM_T::PHDMAR: RDQS_DATA Position     */
#define SPIM_PHDMAR_RDQS_DATA_Msk        (0x1ul << SPIM_PHDMAR_RDQS_DATA_Pos)              /*!< SPIM_T::PHDMAR: RDQS_DATA Mask         */

#define SPIM_PHDMAR_RBO_DATA_Pos         (26)                                              /*!< SPIM_T::PHDMAR: RBO_DATA Position      */
#define SPIM_PHDMAR_RBO_DATA_Msk         (0x3ul << SPIM_PHDMAR_RBO_DATA_Pos)               /*!< SPIM_T::PHDMAR: RBO_DATA Mask          */

#define SPIM_PHDMAR_BM_DATA_Pos          (28)                                              /*!< SPIM_T::PHDMAR: BM_DATA Position       */
#define SPIM_PHDMAR_BM_DATA_Msk          (0x7ul << SPIM_PHDMAR_BM_DATA_Pos)                /*!< SPIM_T::PHDMAR: BM_DATA Mask           */

#define SPIM_PHDMM_DW_CMD_Pos            (0)                                               /*!< SPIM_T::PHDMM: DW_CMD Position         */
#define SPIM_PHDMM_DW_CMD_Msk            (0x7ul << SPIM_PHDMM_DW_CMD_Pos)                  /*!< SPIM_T::PHDMM: DW_CMD Mask             */

#define SPIM_PHDMM_DTR_CMD_Pos           (3)                                               /*!< SPIM_T::PHDMM: DTR_CMD Position        */
#define SPIM_PHDMM_DTR_CMD_Msk           (0x1ul << SPIM_PHDMM_DTR_CMD_Pos)                 /*!< SPIM_T::PHDMM: DTR_CMD Mask            */

#define SPIM_PHDMM_BM_CMD_Pos            (4)                                               /*!< SPIM_T::PHDMM: BM_CMD Position         */
#define SPIM_PHDMM_BM_CMD_Msk            (0x7ul << SPIM_PHDMM_BM_CMD_Pos)                  /*!< SPIM_T::PHDMM: BM_CMD Mask             */

#define SPIM_PHDMM_DW_ADDR_Pos           (8)                                               /*!< SPIM_T::PHDMM: DW_ADDR Position        */
#define SPIM_PHDMM_DW_ADDR_Msk           (0x7ul << SPIM_PHDMM_DW_ADDR_Pos)                 /*!< SPIM_T::PHDMM: DW_ADDR Mask            */

#define SPIM_PHDMM_DTR_ADDR_Pos          (11)                                              /*!< SPIM_T::PHDMM: DTR_ADDR Position       */
#define SPIM_PHDMM_DTR_ADDR_Msk          (0x1ul << SPIM_PHDMM_DTR_ADDR_Pos)                /*!< SPIM_T::PHDMM: DTR_ADDR Mask           */

#define SPIM_PHDMM_BM_ADDR_Pos           (12)                                              /*!< SPIM_T::PHDMM: BM_ADDR Position        */
#define SPIM_PHDMM_BM_ADDR_Msk           (0x7ul << SPIM_PHDMM_BM_ADDR_Pos)                 /*!< SPIM_T::PHDMM: BM_ADDR Mask            */

#define SPIM_PHDMM_DW_MODE_Pos           (16)                                              /*!< SPIM_T::PHDMM: DW_MODE Position        */
#define SPIM_PHDMM_DW_MODE_Msk           (0x7ul << SPIM_PHDMM_DW_MODE_Pos)                 /*!< SPIM_T::PHDMM: DW_MODE Mask            */

#define SPIM_PHDMM_DTR_MODE_Pos          (19)                                              /*!< SPIM_T::PHDMM: DTR_MODE Position       */
#define SPIM_PHDMM_DTR_MODE_Msk          (0x1ul << SPIM_PHDMM_DTR_MODE_Pos)                /*!< SPIM_T::PHDMM: DTR_MODE Mask           */

#define SPIM_PHDMM_BM_MODE_Pos           (20)                                              /*!< SPIM_T::PHDMM: BM_MODE Position        */
#define SPIM_PHDMM_BM_MODE_Msk           (0x7ul << SPIM_PHDMM_BM_MODE_Pos)                 /*!< SPIM_T::PHDMM: BM_MODE Mask            */

#define SPIM_PHDMM_DTR_DATA_Pos          (24)                                              /*!< SPIM_T::PHDMM: DTR_DATA Position       */
#define SPIM_PHDMM_DTR_DATA_Msk          (0x1ul << SPIM_PHDMM_DTR_DATA_Pos)                /*!< SPIM_T::PHDMM: DTR_DATA Mask           */

#define SPIM_PHDMM_RDQS_DATA_Pos         (25)                                              /*!< SPIM_T::PHDMM: RDQS_DATA Position      */
#define SPIM_PHDMM_RDQS_DATA_Msk         (0x1ul << SPIM_PHDMM_RDQS_DATA_Pos)               /*!< SPIM_T::PHDMM: RDQS_DATA Mask          */

#define SPIM_PHDMM_RBO_DATA_Pos          (26)                                              /*!< SPIM_T::PHDMM: RBO_DATA Position       */
#define SPIM_PHDMM_RBO_DATA_Msk          (0x3ul << SPIM_PHDMM_RBO_DATA_Pos)                /*!< SPIM_T::PHDMM: RBO_DATA Mask           */

#define SPIM_PHDMM_BM_DATA_Pos           (28)                                              /*!< SPIM_T::PHDMM: BM_DATA Position        */
#define SPIM_PHDMM_BM_DATA_Msk           (0x7ul << SPIM_PHDMM_BM_DATA_Pos)                 /*!< SPIM_T::PHDMM: BM_DATA Mask            */

#define SPIM_DLL0_DLLOLDO_Pos            (0)                                               /*!< SPIM_T::DLL0: DLLOLDO Position         */
#define SPIM_DLL0_DLLOLDO_Msk            (0x1ul << SPIM_DLL0_DLLOLDO_Pos)                  /*!< SPIM_T::DLL0: DLLOLDO Mask             */

#define SPIM_DLL0_DLLOVRST_Pos           (1)                                               /*!< SPIM_T::DLL0: DLLOVRST Position        */
#define SPIM_DLL0_DLLOVRST_Msk           (0x1ul << SPIM_DLL0_DLLOVRST_Pos)                 /*!< SPIM_T::DLL0: DLLOVRST Mask            */

#define SPIM_DLL0_DLLCLKON_Pos           (2)                                               /*!< SPIM_T::DLL0: DLLCLKON Position        */
#define SPIM_DLL0_DLLCLKON_Msk           (0x1ul << SPIM_DLL0_DLLCLKON_Pos)                 /*!< SPIM_T::DLL0: DLLCLKON Mask            */

#define SPIM_DLL0_DLLLOCK_Pos            (3)                                               /*!< SPIM_T::DLL0: DLLLOCK Position         */
#define SPIM_DLL0_DLLLOCK_Msk            (0x1ul << SPIM_DLL0_DLLLOCK_Pos)                  /*!< SPIM_T::DLL0: DLLLOCK Mask             */

#define SPIM_DLL0_DLLREADY_Pos           (4)                                               /*!< SPIM_T::DLL0: DLLREADY Position        */
#define SPIM_DLL0_DLLREADY_Msk           (0x1ul << SPIM_DLL0_DLLREADY_Pos)                 /*!< SPIM_T::DLL0: DLLREADY Mask            */

#define SPIM_DLL0_DLL_REF_Pos            (5)                                               /*!< SPIM_T::DLL0: DLL_REF Position         */
#define SPIM_DLL0_DLL_REF_Msk            (0x1ul << SPIM_DLL0_DLL_REF_Pos)                  /*!< SPIM_T::DLL0: DLL_REF Mask             */

#define SPIM_DLL0_DLLATRDY_Pos           (6)                                               /*!< SPIM_T::DLL0: DLLATRDY Position        */
#define SPIM_DLL0_DLLATRDY_Msk           (0x1ul << SPIM_DLL0_DLLATRDY_Pos)                 /*!< SPIM_T::DLL0: DLLATRDY Mask            */

#define SPIM_DLL0_DLL_DNUM_Pos           (8)                                               /*!< SPIM_T::DLL0: DLL_DNUM Position        */
#define SPIM_DLL0_DLL_DNUM_Msk           (0x1ful << SPIM_DLL0_DLL_DNUM_Pos)                /*!< SPIM_T::DLL0: DLL_DNUM Mask            */

#define SPIM_DLL0_DLLATEN_Pos            (16)                                              /*!< SPIM_T::DLL0: DLLATEN Position         */
#define SPIM_DLL0_DLLATEN_Msk            (0x1ul << SPIM_DLL0_DLLATEN_Pos)                  /*!< SPIM_T::DLL0: DLLATEN Mask             */

#define SPIM_DLL0_DLLLOOP_Pos            (17)                                              /*!< SPIM_T::DLL0: DLLLOOP Position         */
#define SPIM_DLL0_DLLLOOP_Msk            (0x1ul << SPIM_DLL0_DLLLOOP_Pos)                  /*!< SPIM_T::DLL0: DLLLOOP Mask             */

#define SPIM_DLL0_SIGINTEN_Pos           (18)                                              /*!< SPIM_T::DLL0: SIGINTEN Position        */
#define SPIM_DLL0_SIGINTEN_Msk           (0x1ul << SPIM_DLL0_SIGINTEN_Pos)                 /*!< SPIM_T::DLL0: SIGINTEN Mask            */

#define SPIM_DLL0_DLLDIVER_Pos           (20)                                              /*!< SPIM_T::DLL0: DLLDIVER Position        */
#define SPIM_DLL0_DLLDIVER_Msk           (0x3ul << SPIM_DLL0_DLLDIVER_Pos)                 /*!< SPIM_T::DLL0: DLLDIVER Mask            */

#define SPIM_DLL0_DLLFAST_Pos            (22)                                              /*!< SPIM_T::DLL0: DLLFAST Position         */
#define SPIM_DLL0_DLLFAST_Msk            (0x3ul << SPIM_DLL0_DLLFAST_Pos)                  /*!< SPIM_T::DLL0: DLLFAST Mask             */

#define SPIM_DLL1_DLLOVNUM_Pos           (0)                                               /*!< SPIM_T::DLL1: DLLOVNUM Position        */
#define SPIM_DLL1_DLLOVNUM_Msk           (0xfffful << SPIM_DLL1_DLLOVNUM_Pos)              /*!< SPIM_T::DLL1: DLLOVNUM Mask            */

#define SPIM_DLL1_DLLLKNUM_Pos           (16)                                              /*!< SPIM_T::DLL1: DLLLKNUM Position        */
#define SPIM_DLL1_DLLLKNUM_Msk           (0xfffful << SPIM_DLL1_DLLLKNUM_Pos)              /*!< SPIM_T::DLL1: DLLLKNUM Mask            */

#define SPIM_DLL2_TRIMNUM_Pos            (0)                                               /*!< SPIM_T::DLL2: TRIMNUM Position         */
#define SPIM_DLL2_TRIMNUM_Msk            (0xfffful << SPIM_DLL2_TRIMNUM_Pos)               /*!< SPIM_T::DLL2: TRIMNUM Mask             */

#define SPIM_DLL2_CLKONNUM_Pos           (16)                                              /*!< SPIM_T::DLL2: CLKONNUM Position        */
#define SPIM_DLL2_CLKONNUM_Msk           (0xfffful << SPIM_DLL2_CLKONNUM_Pos)              /*!< SPIM_T::DLL2: CLKONNUM Mask            */

#define SPIM_HYPER_CMD_HYPCMD_Pos        (0)                                               /*!< SPIM_T::HYPER_CMD: HYPCMD Position     */
#define SPIM_HYPER_CMD_HYPCMD_Msk        (0xful << SPIM_HYPER_CMD_HYPCMD_Pos)              /*!< SPIM_T::HYPER_CMD: HYPCMD Mask         */

#define SPIM_HYPER_CONFIG1_CSST_Pos      (0)                                               /*!< SPIM_T::HYPER_CONFIG1: CSST Position   */
#define SPIM_HYPER_CONFIG1_CSST_Msk      (0x3ul << SPIM_HYPER_CONFIG1_CSST_Pos)            /*!< SPIM_T::HYPER_CONFIG1: CSST Mask       */

#define SPIM_HYPER_CONFIG1_CSH_Pos       (4)                                               /*!< SPIM_T::HYPER_CONFIG1: CSH Position    */
#define SPIM_HYPER_CONFIG1_CSH_Msk       (0x3ul << SPIM_HYPER_CONFIG1_CSH_Pos)             /*!< SPIM_T::HYPER_CONFIG1: CSH Mask        */

#define SPIM_HYPER_CONFIG1_CSHI_Pos      (8)                                               /*!< SPIM_T::HYPER_CONFIG1: CSHI Position   */
#define SPIM_HYPER_CONFIG1_CSHI_Msk      (0xful << SPIM_HYPER_CONFIG1_CSHI_Pos)            /*!< SPIM_T::HYPER_CONFIG1: CSHI Mask       */

#define SPIM_HYPER_CONFIG1_CSMAXLT_Pos   (16)                                              /*!< SPIM_T::HYPER_CONFIG1: CSMAXLT Position*/
#define SPIM_HYPER_CONFIG1_CSMAXLT_Msk   (0xfffful << SPIM_HYPER_CONFIG1_CSMAXLT_Pos)      /*!< SPIM_T::HYPER_CONFIG1: CSMAXLT Mask    */

#define SPIM_HYPER_CONFIG2_ACCTWR_Pos    (8)                                               /*!< SPIM_T::HYPER_CONFIG2: ACCTWR Position */
#define SPIM_HYPER_CONFIG2_ACCTWR_Msk    (0x1ful << SPIM_HYPER_CONFIG2_ACCTWR_Pos)         /*!< SPIM_T::HYPER_CONFIG2: ACCTWR Mask     */

#define SPIM_HYPER_CONFIG2_RSTNLT_Pos    (16)                                              /*!< SPIM_T::HYPER_CONFIG2: RSTNLT Position */
#define SPIM_HYPER_CONFIG2_RSTNLT_Msk    (0xfful << SPIM_HYPER_CONFIG2_RSTNLT_Pos)         /*!< SPIM_T::HYPER_CONFIG2: RSTNLT Mask     */

#define SPIM_HYPER_CONFIG2_ACCTRD_Pos    (24)                                              /*!< SPIM_T::HYPER_CONFIG2: ACCTRD Position */
#define SPIM_HYPER_CONFIG2_ACCTRD_Msk    (0x1ful << SPIM_HYPER_CONFIG2_ACCTRD_Pos)         /*!< SPIM_T::HYPER_CONFIG2: ACCTRD Mask     */

#define SPIM_HYPER_ADR_HBADDR_Pos        (0)                                               /*!< SPIM_T::HYPER_ADR: HBADDR Position     */
#define SPIM_HYPER_ADR_HBADDR_Msk        (0x1fffffful << SPIM_HYPER_ADR_HBADDR_Pos)        /*!< SPIM_T::HYPER_ADR: HBADDR Mask         */

#define SPIM_HYPER_WDATA_HBWDATA_Pos     (0)                                               /*!< SPIM_T::HYPER_WDATA: HBWDATA Position  */
#define SPIM_HYPER_WDATA_HBWDATA_Msk     (0xfffffffful << SPIM_HYPER_WDATA_HBWDATA_Pos)    /*!< SPIM_T::HYPER_WDATA: HBWDATA Mask      */

#define SPIM_HYPER_RDATA_HBRDATA_Pos     (0)                                               /*!< SPIM_T::HYPER_RDATA: HBRDATA Position  */
#define SPIM_HYPER_RDATA_HBRDATA_Msk     (0xfffffffful << SPIM_HYPER_RDATA_HBRDATA_Pos)    /*!< SPIM_T::HYPER_RDATA: HBRDATA Mask      */

#define SPIM_HYPER_INTEN_OPINTEN_Pos     (0)                                               /*!< SPIM_T::HYPER_INTEN: OPINTEN Position  */
#define SPIM_HYPER_INTEN_OPINTEN_Msk     (0x1ul << SPIM_HYPER_INTEN_OPINTEN_Pos)           /*!< SPIM_T::HYPER_INTEN: OPINTEN Mask      */

#define SPIM_HYPER_INTSTS_OPDONE_Pos     (0)                                               /*!< SPIM_T::HYPER_INTSTS: OPDONE Position  */
#define SPIM_HYPER_INTSTS_OPDONE_Msk     (0x1ul << SPIM_HYPER_INTSTS_OPDONE_Pos)           /*!< SPIM_T::HYPER_INTSTS: OPDONE Mask      */

/** @} SPIM_CONST */
/** @} end of SPIM register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __SPIM_REG_H__ */
