/**************************************************************************//**
 * @file     crc_reg.h
 * @version  V1.00
 * @brief    CRC register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __CRC_REG_H__
#define __CRC_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup CRC Cyclic Redundancy Check Controller (CRC)
    Memory Mapped Structure for CRC Controller
    @{
*/

typedef struct
{
    /**
     * @var CRC_T::CTL
     * Offset: 0x00  CRC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CRCEN     |CRC Channel Enable Bit
     * |        |          |0 = No effect.
     * |        |          |1 = CRC operation Enabled.
     * |[1]     |CHKSINIT  |Checksum Initialization
     * |        |          |0 = No effect.
     * |        |          |1 = Initial checksum value by auto reload CRC_SEED register value to CRC_CHECKSUM register value.
     * |        |          |Note: This bit will be cleared automatically.
     * |[24]    |DATREV    |Write Data Bit Order Reverse
     * |        |          |This bit is used to enable the bit order reverse function per byte for write data value in CRC_DAT register.
     * |        |          |0 = Bit order reversed for CRC write data in Disabled.
     * |        |          |1 = Bit order reversed for CRC write data in Enabled (per byte).
     * |        |          |Note: If the write data is 0xAABBCCDD, the bit order reverse for CRC write data in is 0x55DD33BB.
     * |[25]    |CHKSREV   |Checksum Bit Order Reverse
     * |        |          |This bit is used to enable the bit order reverse function for checksum result in CRC_CHECKSUM register.
     * |        |          |0 = Bit order reverse for CRC checksum Disabled.
     * |        |          |1 = Bit order reverse for CRC checksum Enabled.
     * |        |          |Note: If the checksum result is 0xDD7B0F2E, the bit order reverse for CRC checksum is 0x74F0DEBB.
     * |[26]    |DATFMT    |Write Data 1's Complement
     * |        |          |This bit is used to enable the 1's complement function for write data value in CRC_DAT register.
     * |        |          |0 = 1's complement for CRC writes data in Disabled.
     * |        |          |1 = 1's complement for CRC writes data in Enabled.
     * |[27]    |CHKSFMT   |Checksum 1's Complement
     * |        |          |This bit is used to enable the 1's complement function for checksum result in CRC_CHECKSUM register.
     * |        |          |0 = 1's complement for CRC checksum Disabled.
     * |        |          |1 = 1's complement for CRC checksum Enabled.
     * |[29:28] |DATLEN    |CPU Write Data Length
     * |        |          |This field indicates the write data length.
     * |        |          |00 = Data length is 8-bit mode.
     * |        |          |01 = Data length is 16-bit mode.
     * |        |          |1x = Data length is 32-bit mode.
     * |        |          |Note: When the write data length is 8-bit mode, the valid data in CRC_DAT register is only DATA[7:0] bits; if the write data length is 16-bit mode, the valid data in CRC_DAT register is only DATA[15:0]
     * |[31:30] |CRCMODE   |CRC Polynomial Mode
     * |        |          |This field indicates the CRC operation polynomial mode.
     * |        |          |01 = CRC-8 Polynomial mode.
     * |        |          |10 = CRC-16 Polynomial mode.
     * |        |          |11 = CRC-32 Polynomial mode.
     * @var CRC_T::DAT
     * Offset: 0x04  CRC Write Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |DATA      |CRC Write Data Bits
     * |        |          |User can write data directly by CPU mode or use PDMA function to write data to this field to perform CRC operation.
     * |        |          |Note: When the write data length is 8-bit mode, the valid data in CRC_DAT register is only DATA[7:0] bits; if the write data length is 16-bit mode, the valid data in CRC_DAT register is only DATA[15:0].
     * @var CRC_T::SEED
     * Offset: 0x08  CRC Seed Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |SEED      |CRC Seed Value
     * |        |          |This field indicates the CRC seed value.
     * |        |          |Note: This field will be reloaded as checksum initial value (CRC_CHECKSUM register) after perform CHKSINIT (CRC_CTL[1]).
     * @var CRC_T::CHECKSUM
     * Offset: 0x0C  CRC Checksum Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |CHECKSUM  |CRC Checksum Results
    * |        |          |This field indicates the CRC checksum result.
    * |        |          |Note: Data in CRC_CHECKSUM register has different length when user chooses different operation polynomial modes.
    * |        |          |For example:
    * |        |          |If final checksum result is 0x12 in CRC-8 polynomial mode, the CHECKSUM[31:0] value will be read as 0x12121212, only CHECKSUM[7:0] is valid in this mode.
    * |        |          |If final checksum result is 0x1234 in CRC-CCITT or CRC-16 mode, the CHECKSUM[31:0] value will be read as 0x12341234, only CHECKSUM[15:0] is valid in this mode.
    * |        |          |And the CHECKSUM[31:0] is valid for CRC-32 mode.
    * @var CRC_T::POLYNOMIAL
    * Offset: 0x10  CRC Polynomial Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[31:0]  |POLYNOMIAL|CRC Polynomial Value Results
    * |        |          |This field indicates the value of CRC polynomaial.
    * @var CRC_T::DMACTL
    * Offset: 0x40  CRC DMA Mode Control Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[0]     |START     |CRC DMA Start Bit
    * |        |          |Set CRC DMA to start operation.
    * |        |          |This bit is read-as-1 when the CRC DMA is operating.
    * |[1]     |PAUSE     |CRC DMA Pause Bit
    * |        |          |Set CRC DMA to pause the operation.
    * |        |          |This bit is read-as-1 when the CRC DMA is paused.
    * |        |          |This bit can be set to 1 when the CRC DMA is operating.
    * |[4]     |ABORT     |CRC DMA Operation Abort Bit
    * |        |          |Write 1 to abort the operation of CRC DMA.
    * |[8]     |INTEN     |CRC DMA Mode Interrupt Enable Bit
    * |        |          |Enable CRC DMA mode interrupt.
    * @var CRC_T::DMASTS
    * Offset: 0x44  CRC DMA Mode Status Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[0]     |FINISH    |DMA Operation Finish Flag
    * |        |          |Indicates the operation of DMA mode finished successfully.
    * |[1]     |ABORTED   |DMA Operation Aborted Flag
    * |        |          |Indicates the operation of DMA mode is aborted due to some error. (ex: bus error)
    * |[2]     |CFGERR    |Configuration Error Flag
    * |        |          |Indicates the configuration to run DMA operation is invalid and the operation of DMA mode is cancelled.
    * @var CRC_T::SADDR
    * Offset: 0x48  CRC DMA Source Address Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[31:2]  |SADDR     |CRC DMA Source Address
    * |        |          |DMA source address
    * @var CRC_T::DMACNT
    * Offset: 0x4C  CRC DMA Byte Count Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[31:2]  |DMACNT    |CRC DMA Operation Byte Count
    * |        |          |Indicates bytes for DMA to read.
    * @var CRC_T::VERSION
    * Offset: 0xFFC  CRC RTL Design Version Number
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[15:0]  |MINOR     |CRC RTL Design MINOR Version Number
    * |        |          |Minor version number is dependent on ECO version control
    * |        |          |0x0000: (current Minor Version Number)
    * |[23:16] |SUB       |CRC RTL Design MINOR Version Number
    * |        |          |Sub version number is relative to key feature
    * |        |          |0x01: (current Sub Version Number)
    * |[31:24] |MAJOR     |CRC RTL Design MAJOR Version Number
    * |        |          |Major version number is correlated to Product Line
    * |        |          |0x01: (current Major Version Number)
    */
    __IO uint32_t CTL;                   /*!< [0x0000] CRC Control Register                                             */
    __IO uint32_t DAT;                   /*!< [0x0004] CRC Write Data Register                                          */
    __IO uint32_t SEED;                  /*!< [0x0008] CRC Seed Register                                                */
    __I  uint32_t CHECKSUM;              /*!< [0x000c] CRC Checksum Register                                            */
    __IO uint32_t POLYNOMIAL;            /*!< [0x0010] CRC Polynomial Register                                          */
    __I  uint32_t RESERVE0[11];
    __IO uint32_t DMACTL;                /*!< [0x0040] CRC DMA Mode Control Register                                    */
    __IO uint32_t DMASTS;                /*!< [0x0044] CRC DMA Mode Status Register                                     */
    __IO uint32_t SADDR;                 /*!< [0x0048] CRC DMA Source Address Register                                  */
    __IO uint32_t DMACNT;                /*!< [0x004c] CRC DMA Byte Count Register                                      */
    __I  uint32_t RESERVE1[1003];
    __I  uint32_t VERSION;               /*!< [0x0ffc] CRC RTL Design Version Number                                    */

} CRC_T;

/**
    @addtogroup CRC_CONST CRC Bit Field Definition
    Constant Definitions for CRC Controller
@{ */

#define CRC_CTL_CRCEN_Pos                (0)                                               /*!< CRC_T::CTL: CRCEN Position             */
#define CRC_CTL_CRCEN_Msk                (0x1ul << CRC_CTL_CRCEN_Pos)                      /*!< CRC_T::CTL: CRCEN Mask                 */

#define CRC_CTL_CHKSINIT_Pos             (1)                                               /*!< CRC_T::CTL: CHKSINIT Position          */
#define CRC_CTL_CHKSINIT_Msk             (0x1ul << CRC_CTL_CHKSINIT_Pos)                   /*!< CRC_T::CTL: CHKSINIT Mask              */

#define CRC_CTL_DATREV_Pos               (24)                                              /*!< CRC_T::CTL: DATREV Position            */
#define CRC_CTL_DATREV_Msk               (0x1ul << CRC_CTL_DATREV_Pos)                     /*!< CRC_T::CTL: DATREV Mask                */

#define CRC_CTL_CHKSREV_Pos              (25)                                              /*!< CRC_T::CTL: CHKSREV Position           */
#define CRC_CTL_CHKSREV_Msk              (0x1ul << CRC_CTL_CHKSREV_Pos)                    /*!< CRC_T::CTL: CHKSREV Mask               */

#define CRC_CTL_DATFMT_Pos               (26)                                              /*!< CRC_T::CTL: DATFMT Position            */
#define CRC_CTL_DATFMT_Msk               (0x1ul << CRC_CTL_DATFMT_Pos)                     /*!< CRC_T::CTL: DATFMT Mask                */

#define CRC_CTL_CHKSFMT_Pos              (27)                                              /*!< CRC_T::CTL: CHKSFMT Position           */
#define CRC_CTL_CHKSFMT_Msk              (0x1ul << CRC_CTL_CHKSFMT_Pos)                    /*!< CRC_T::CTL: CHKSFMT Mask               */

#define CRC_CTL_DATLEN_Pos               (28)                                              /*!< CRC_T::CTL: DATLEN Position            */
#define CRC_CTL_DATLEN_Msk               (0x3ul << CRC_CTL_DATLEN_Pos)                     /*!< CRC_T::CTL: DATLEN Mask                */

#define CRC_CTL_CRCMODE_Pos              (30)                                              /*!< CRC_T::CTL: CRCMODE Position           */
#define CRC_CTL_CRCMODE_Msk              (0x3ul << CRC_CTL_CRCMODE_Pos)                    /*!< CRC_T::CTL: CRCMODE Mask               */

#define CRC_DAT_DATA_Pos                 (0)                                               /*!< CRC_T::DAT: DATA Position              */
#define CRC_DAT_DATA_Msk                 (0xfffffffful << CRC_DAT_DATA_Pos)                /*!< CRC_T::DAT: DATA Mask                  */

#define CRC_SEED_SEED_Pos                (0)                                               /*!< CRC_T::SEED: SEED Position             */
#define CRC_SEED_SEED_Msk                (0xfffffffful << CRC_SEED_SEED_Pos)               /*!< CRC_T::SEED: SEED Mask                 */

#define CRC_CHECKSUM_CHECKSUM_Pos        (0)                                               /*!< CRC_T::CHECKSUM: CHECKSUM Position     */
#define CRC_CHECKSUM_CHECKSUM_Msk        (0xfffffffful << CRC_CHECKSUM_CHECKSUM_Pos)       /*!< CRC_T::CHECKSUM: CHECKSUM Mask         */

#define CRC_POLYNOMIAL_POLYNOMIAL_Pos    (0)                                               /*!< CRC_T::POLYNOMIAL: POLYNOMIAL Position */
#define CRC_POLYNOMIAL_POLYNOMIAL_Msk    (0xfffffffful << CRC_POLYNOMIAL_POLYNOMIAL_Pos)   /*!< CRC_T::POLYNOMIAL: POLYNOMIAL Mask     */

#define CRC_DMACTL_START_Pos             (0)                                               /*!< CRC_T::DMACTL: START Position          */
#define CRC_DMACTL_START_Msk             (0x1ul << CRC_DMACTL_START_Pos)                   /*!< CRC_T::DMACTL: START Mask              */

#define CRC_DMACTL_PAUSE_Pos             (1)                                               /*!< CRC_T::DMACTL: PAUSE Position          */
#define CRC_DMACTL_PAUSE_Msk             (0x1ul << CRC_DMACTL_PAUSE_Pos)                   /*!< CRC_T::DMACTL: PAUSE Mask              */

#define CRC_DMACTL_ABORT_Pos             (4)                                               /*!< CRC_T::DMACTL: ABORT Position          */
#define CRC_DMACTL_ABORT_Msk             (0x1ul << CRC_DMACTL_ABORT_Pos)                   /*!< CRC_T::DMACTL: ABORT Mask              */

#define CRC_DMACTL_INTEN_Pos             (8)                                               /*!< CRC_T::DMACTL: INTEN Position          */
#define CRC_DMACTL_INTEN_Msk             (0x1ul << CRC_DMACTL_INTEN_Pos)                   /*!< CRC_T::DMACTL: INTEN Mask              */

#define CRC_DMASTS_FINISH_Pos            (0)                                               /*!< CRC_T::DMASTS: FINISH Position         */
#define CRC_DMASTS_FINISH_Msk            (0x1ul << CRC_DMASTS_FINISH_Pos)                  /*!< CRC_T::DMASTS: FINISH Mask             */

#define CRC_DMASTS_ABORTED_Pos           (1)                                               /*!< CRC_T::DMASTS: ABORTED Position        */
#define CRC_DMASTS_ABORTED_Msk           (0x1ul << CRC_DMASTS_ABORTED_Pos)                 /*!< CRC_T::DMASTS: ABORTED Mask            */

#define CRC_DMASTS_CFGERR_Pos            (2)                                               /*!< CRC_T::DMASTS: CFGERR Position         */
#define CRC_DMASTS_CFGERR_Msk            (0x1ul << CRC_DMASTS_CFGERR_Pos)                  /*!< CRC_T::DMASTS: CFGERR Mask             */

#define CRC_DMASTS_ACCERR_Pos            (3)                                               /*!< CRC_T::DMASTS: ACCERR Position         */
#define CRC_DMASTS_ACCERR_Msk            (0x1ul << CRC_DMASTS_ACCERR_Pos)                  /*!< CRC_T::DMASTS: ACCERR Mask             */

#define CRC_SADDR_SADDR_Pos              (2)                                               /*!< CRC_T::SADDR: SADDR Position           */
#define CRC_SADDR_SADDR_Msk              (0x3ffffffful << CRC_SADDR_SADDR_Pos)             /*!< CRC_T::SADDR: SADDR Mask               */

#define CRC_DMACNT_DMACNT_Pos            (2)                                               /*!< CRC_T::DMACNT: DMACNT Position         */
#define CRC_DMACNT_DMACNT_Msk            (0x3ffffffful << CRC_DMACNT_DMACNT_Pos)           /*!< CRC_T::DMACNT: DMACNT Mask             */

#define CRC_VERSION_MINOR_Pos            (0)                                               /*!< CRC_T::VERSION: MINOR Position         */
#define CRC_VERSION_MINOR_Msk            (0xfffful << CRC_VERSION_MINOR_Pos)               /*!< CRC_T::VERSION: MINOR Mask             */

#define CRC_VERSION_SUB_Pos              (16)                                              /*!< CRC_T::VERSION: SUB Position           */
#define CRC_VERSION_SUB_Msk              (0xfful << CRC_VERSION_SUB_Pos)                   /*!< CRC_T::VERSION: SUB Mask               */

#define CRC_VERSION_MAJOR_Pos            (24)                                              /*!< CRC_T::VERSION: MAJOR Position         */
#define CRC_VERSION_MAJOR_Msk            (0xfful << CRC_VERSION_MAJOR_Pos)                 /*!< CRC_T::VERSION: MAJOR Mask             */
/** @} CRC_CONST */
/** @} end of CRC register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __CRC_REG_H__ */
