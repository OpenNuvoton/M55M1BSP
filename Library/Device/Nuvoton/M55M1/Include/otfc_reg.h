/**************************************************************************//**
 * @file     otfc_reg.h
 * @version  V1.00
 * @brief    OTFC register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __OTFC_REG_H__
#define __OTFC_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
  @{
*/

/**
    @addtogroup OTFC On-The-Fly Cipher (OTFC)
    Memory Mapped Structure for OTFC Controller
    @{
*/

typedef struct
{
    /**
    * @var OTFC_PR_T::SADDR
    * Offset: 0x10 0x50 0x90 0xD0 OTFC Protection Region 0~3 Start Address Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[31:0]  |SADDR     |Protection Region 0~3 Start Address Register
    * |        |          |The start address indicates the start address of protection region 0~3 in the external memory space for OTFC
    * |        |          |The start address should be located at word boundary
    * |        |          |In other words, bit 1 and 0 of SADDR are ignored
    * |        |          |SADDR can be read and written
    * @var OTFC_PR_T::EADDR
    * Offset: 0x14 0x54 0x94 0xD4 OTFC Protection Region 0~3 End Address Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[31:0]  |EADDR     |OTFC Protection Region 0~3 End Address
    * |        |          |The end address indicates the end address of protection region 0~3 in the external memory space for OTFC
    * |        |          |The end address should be located at word boundary
    * |        |          |In other words, bit 1 and 0 of EADDR are ignored
    * |        |          |EADDR can be read and written
    * @var OTFC_PR_T::KSCTL
    * Offset: 0x18 0x58 0x98 0xD8 OTFC Protection Region 0~3 Key Control Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[4:0]   |NUM       |Read Key Number
    * |        |          |The key number is sent to Key Store
    * |[5]     |RSRC      |Read Key Source
    * |        |          |0 = Key is read from registers OTFC_PRx_KEYx.
    * |        |          |1 = Key is read from Key Store.
    * |[7:6]   |RSSRC     |Read Key Store Source
    * |        |          |RSSRC takes effect only when RSRC is 1 (the key is from Key Store).
    * |        |          |00 = Key is read from the SRAM of Key Store.
    * |        |          |10 = Key is read from the OTP of Key Store.
    * |        |          |Others = reserved.
    * @var OTFC_PR_T::KEY0
    * Offset: 0x20 0x60 0xA0 0xE0 OTFC Protection Region 0~3 KEY Word 0 Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[31:0]  |KEY       |OTFC_PRx_KEY0
    * |        |          |The KEY keeps the security key for AES operation in protection region 0~3.
    * |        |          |x = 0, 1, 2, 3.
    * |        |          |The security key for AES accelerator is 128 bits and four 32-bit registers are to store each security key.
    * |        |          |{OTFC_PRx_KEY3, OTFC_PRx_KEY2, OTFC_PRx_KEY1, OTFC_PRx_KEY0} stores the 128-bit security key for AES operation
    * @var OTFC_PR_T::KEY1
    * Offset: 0x24 0x64 0xA4 0xE4 OTFC Protection Region 0~3 KEY Word 1 Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[31:0]  |KEY       |OTFC_PRx_KEY1
    * |        |          |The KEY keeps the security key for AES operation in protection region 0~3.
    * |        |          |x = 0, 1, 2, 3.
    * |        |          |The security key for AES accelerator is 128 bits and four 32-bit registers are to store each security key.
    * |        |          |{OTFC_PRx_KEY3, OTFC_PRx_KEY2, OTFC_PRx_KEY1, OTFC_PRx_KEY0} stores the 128-bit security key for AES operation
    * @var OTFC_PR_T::KEY2
    * Offset: 0x28 0x68 0xA8 0xE8 OTFC Protection Region 0~3 KEY Word 2 Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[31:0]  |KEY       |OTFC_PRx_KEY2
    * |        |          |The KEY keeps the security key for AES operation in protection region 0~3.
    * |        |          |x = 0, 1, 2, 3.
    * |        |          |The security key for AES accelerator is 128 bits and four 32-bit registers are to store each security key.
    * |        |          |{OTFC_PRx_KEY3, OTFC_PRx_KEY2, OTFC_PRx_KEY1, OTFC_PRx_KEY0} stores the 128-bit security key for AES operation
    * @var OTFC_PR_T::KEY3
    * Offset: 0x2C 0x6C 0xAC 0xEC OTFC Protection Region 0~3 KEY Word 3 Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[31:0]  |KEY       |OTFC_PRx_KEY3
    * |        |          |The KEY keeps the security key for AES operation in protection region 0~3.
    * |        |          |x = 0, 1, 2, 3.
    * |        |          |The security key for AES accelerator is 128 bits and four 32-bit registers are to store each security key.
    * |        |          |{OTFC_PRx_KEY3, OTFC_PRx_KEY2, OTFC_PRx_KEY1, OTFC_PRx_KEY0} stores the 128-bit security key for AES operation
    * @var OTFC_PR_T::SCRAMBLE
    * Offset: 0x40 0x80 0xC0 0x100 OTFC Protection Region 0~3 Scramble key Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[31:0]  |SCRAMBLE  |Protection Region 0~3 Scramble key Register
    * |        |          |The scramble key for address cipher function in protection region 0~3.
    * @var OTFC_PR_T::NONCE0
    * Offset: 0x44 0x84 0xC4 0x104 OTFC Protection Region 0~3 Nonce Word 0 Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[31:0]  |NONCE     |Protection Region 0~3 Nonce Word x Register
    * |        |          |x = 0, 1, 2.
    * |        |          |Three nonces (OTFC_PRx_NONCE0, OTFC_PRx_NONCE1, and OTFC_PRx_NONCE2) are for the address cipher function in OTFC.
    * @var OTFC_PR_T::NONCE1
    * Offset: 0x48 0x88 0xC8 0x108 OTFC Protection Region 0~3 Nonce Word 1 Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[31:0]  |NONCE     |Protection Region 0~3 Nonce Word x Register
    * |        |          |x = 0, 1, 2.
    * |        |          |Three nonces (OTFC_PRx_NONCE0, OTFC_PRx_NONCE1, and OTFC_PRx_NONCE2) are for the address cipher function in OTFC.
    * @var OTFC_PR_T::NONCE2
    * Offset: 0x4C 0x8C 0xCC 0x10C OTFC Protection Region 0~3 Nonce Word 2 Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[31:0]  |NONCE     |Protection Region 0~3 Nonce Word x Register
    * |        |          |x = 0, 1, 2.
    * |        |          |Three nonces (OTFC_PRx_NONCE0, OTFC_PRx_NONCE1, and OTFC_PRx_NONCE2) are for the address cipher function in OTFC.
    */
    __IO uint32_t SADDR;             /*!< [0x0010] OTFC Protection Region 0~3 Start Address Register                  */
    __IO uint32_t EADDR;             /*!< [0x0014] OTFC Protection Region 0~3 End Address Register                    */
    __O  uint32_t KSCTL;             /*!< [0x0018] OTFC Protection Region 0~3 Key Control Register                    */
    __I  uint32_t RESERVE1[1];
    __O  uint32_t KEY0;              /*!< [0x0020] OTFC Protection Region 0~3 KEY Word 0 Register                     */
    __O  uint32_t KEY1;              /*!< [0x0024] OTFC Protection Region 0~3 KEY Word 1 Register                     */
    __O  uint32_t KEY2;              /*!< [0x0028] OTFC Protection Region 0~3 KEY Word 2 Register                     */
    __O  uint32_t KEY3;              /*!< [0x002c] OTFC Protection Region 0~3 KEY Word 3 Register                     */
    __I  uint32_t RESERVE2[4];
    __O  uint32_t SCRAMBLE;          /*!< [0x0040] OTFC Protection Region 0~3 Scramble key Register                   */
    __O  uint32_t NONCE0;            /*!< [0x0044] OTFC Protection Region 0~3 Nonce Word 0 Register                   */
    __O  uint32_t NONCE1;            /*!< [0x0048] OTFC Protection Region 0~3 Nonce Word 1 Register                   */
    __O  uint32_t NONCE2;            /*!< [0x004c] OTFC Protection Region 0~3 Nonce Word 2 Register                   */
} OTFC_PR_T;

typedef struct
{
    /**
     * @var OTFC_T::CTL
     * Offset: 0x00  OTFC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EN0       |Protection Region 0 Enable
     * |        |          |0 = No effect.
     * |        |          |1 = OTFC protection region 0 Enabled.
     * |[1]     |RST0      |Protection Region 0 Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Clear the setting setting of OTFC protection region 0.
     * |        |          |Note: This bit is always 0 when it is read back.
     * |[2]     |KSWAP0    |Protection Region 0 Key and Nonce Swap
     * |        |          |0 = Keep the original order in OTFCx_PR0_KEYy.
     * |        |          |1 = The order that CPU feeds key and nonce to the accelerator will be changed from {byte3, byte2, byte1, byte0} to {byte0, byte1, byte2, byte3}.
     * |[7]     |IEN0      |Protection Region 0 Interrupt Enable Bit
     * |        |          |0 = Protection Region 0 interrupt Disabled.
     * |        |          |1 = Protection Region 0 interrupt Enabled.
     * |[8]     |EN1       |Protection Region 1 Enable
     * |        |          |0 = No effect.
     * |        |          |1 = OTFC protection region 1 Enabled.
     * |[9]     |RST1      |Protection Region 1 Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Clear the setting setting of OTFC protection region 1.
     * |        |          |Note: This bit is always 0 when it is read back.
     * |[10]    |KSWAP1    |Protection Region 1 Key and Nonce Swap
     * |        |          |0 = Keep the original order in OTFCx_PR1_KEYy..
     * |        |          |1 = The order that CPU feeds key and nonce to the accelerator will be changed from {byte3, byte2, byte1, byte0} to {byte0, byte1, byte2, byte3}.
     * |[15]    |IEN1      |Protection Region 1 Interrupt Enable Bit
     * |        |          |0 = Protection Region 1 interrupt Disabled.
     * |        |          |1 = Protection Region 1 interrupt Enabled.
     * |[16]    |EN2       |Protection Region 2 Enable
     * |        |          |0 = No effect.
     * |        |          |1 = OTFC protection region 2 Enabled.
     * |[17]    |RST2      |Protection Region 2 Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Clear the setting setting of OTFC protection region 2.
     * |        |          |Note: This bit is always 0 when it is read back.
     * |[18]    |KSWAP2    |Protection Region 2 Key and Nonce Swap
     * |        |          |0 = Keep the original order in OTFCx_PR2_KEYy.
     * |        |          |1 = The order that CPU feeds key and nonce to the accelerator will be changed from {byte3, byte2, byte1, byte0} to {byte0, byte1, byte2, byte3}.
     * |[23]    |IEN2      |Protection Region 2 Interrupt Enable Bit
     * |        |          |0 = Protection Region 2 interrupt Disabled.
     * |        |          |1 = Protection Region 2 interrupt Enabled.
     * |[24]    |EN3       |Protection Region 3 Enable
     * |        |          |0 = No effect.
     * |        |          |1 = OTFC protection region 3 Enabled.
     * |[25]    |RST3      |Protection Region 3 Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Clear the setting setting of OTFC protection region 3.
     * |        |          |Note: This bit is always 0 when it is read back.
     * |[26]    |KSWAP3    |Protection Region 3 Key and Nonce Swap
     * |        |          |0 = Keep the original order in OTFCx_PR3_KEYy.
     * |        |          |1 = The order that CPU feeds key and nonce to the accelerator will be changed from {byte3, byte2, byte1, byte0} to {byte0, byte1, byte2, byte3}.
     * |[31]    |IEN3      |Protection Region 3 Interrupt Enable Bit
     * |        |          |0 = Protection Region 3 interrupt Disabled.
     * |        |          |1 = Protection Region 3 interrupt Enabled.
     * @var OTFC_T::STS
     * Offset: 0x04  OTFC Status Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BUSY0     |OTFC Protection Region 0 Busy (Read Only)
     * |        |          |0 = The OTFC Protection Region 0 is idle.
     * |        |          |1 = The OTFC Protection Region 0 is under processing.
     * |[5]     |KSERR0    |OTFC Protection Region 0 Access Key Store Error Flag (Read Only)
     * |        |          |0 = No error.
     * |        |          |1 = Key Store access error will stop OTFC engine in Protection Region 0.
     * |[6]     |DFAERR0   |OTFC Protection Region 0 Differential Fault Attack Error Flag (Read Only)
     * |        |          |0 = No error.
     * |        |          |1 = Differential Fault Attack happened in protection region 0
     * |        |          |The results from AES engine are incorrect.
     * |[7]     |IF0       |OTFC Protection Region 0 Interrupt Flag
     * |        |          |0 = No OTFC Protection Region 0 interrupt.
     * |        |          |1 = OTFC Protection Region 0 interrupt.
     * |        |          |Note: This bit is cleared by writing 1 and it has no effect by writing 0.
     * |[8]     |BUSY1     |OTFC Protection Region 1 Busy (Read Only)
     * |        |          |0 = The OTFC Protection Region 1 is idle.
     * |        |          |1 = The OTFC Protection Region 1 is under processing.
     * |[13]    |KSERR1    |OTFC Protection Region 1 Access Key Store Error Flag (Read Only)
     * |        |          |0 = No error.
     * |        |          |1 = Key Store access error will stop OTFC engine in Protection Region 1.
     * |[14]    |DFAERR1   |OTFC Protection Region 1 Differential Fault Attack Error Flag (Read Only)
     * |        |          |0 = No error.
     * |        |          |1 = Differential Fault Attack happened in protection region 1
     * |        |          |The results from AES engine are incorrect.
     * |[15]    |IF1       |OTFC Protection Region 1 Interrupt Flag
     * |        |          |0 = No OTFC Protection Region 1 interrupt.
     * |        |          |1 = OTFC Protection Region 1 interrupt.
     * |        |          |Note: This bit is cleared by writing 1 and it has no effect by writing 0.
     * |[16]    |BUSY2     |OTFC Protection Region 2 Busy (Read Only)
     * |        |          |0 = The OTFC Protection Region 2 is idle.
     * |        |          |1 = The OTFC Protection Region 2 is under processing.
     * |[21]    |KSERR2    |OTFC Protection Region 2 Access Key Store Error Flag (Read Only)
     * |        |          |0 = No error.
     * |        |          |1 = Key Store access error will stop OTFC engine in Protection Region 2.
     * |[22]    |DFAERR2   |OTFC Protection Region 2 Differential Fault Attack Error Flag (Read Only)
     * |        |          |0 = No error.
     * |        |          |1 = Differential Fault Attack happened in protection region 2
     * |        |          |The results from AES engine are incorrect.
     * |[23]    |IF2       |OTFC Protection Region 2 Interrupt Flag
     * |        |          |0 = No OTFC Protection Region 2 interrupt.
     * |        |          |1 = OTFC Protection Region 2 interrupt.
     * |        |          |Note: This bit is cleared by writing 1 and it has no effect by writing 0.
     * |[24]    |BUSY3     |OTFC Protection Region 3 Busy (Read Only)
     * |        |          |0 = The OTFC Protection Region 3 is idle.
     * |        |          |1 = The OTFC Protection Region 3 is under processing.
     * |[29]    |KSERR3    |OTFC Protection Region 3 Access Key Store Error Flag (Read Only)
     * |        |          |0 = No error.
     * |        |          |1 = Key Store access error will stop OTFC engine in Protection Region 3.
     * |[30]    |DFAERR3   |OTFC Protection Region 3 Differential Fault Attack Error Flag (Read Only)
     * |        |          |0 = No error.
     * |        |          |1 = Differential Fault Attack happened in protection region 3
     * |        |          |The results from AES engine are incorrect.
     * |[31]    |IF3       |OTFC Protection Region 3 Interrupt Flag
     * |        |          |0 = No OTFC Protection Region 3 interrupt.
     * |        |          |1 = OTFC Protection Region 3 interrupt.
     * |        |          |Note: This bit is cleared by writing 1 and it has no effect by writing 0.
     */
    __IO uint32_t CTL;                  /*!< [0x0000] OTFC Control Register                                            */
    __IO uint32_t STS;                  /*!< [0x0004] OTFC Status Flag Register                                        */
    __I  uint32_t RESERVE0[2];

    OTFC_PR_T PR[4];                    /* Protect region table */
} OTFC_T;

/**
    @addtogroup OTFC_CONST OTFC Bit Field Definition
    Constant Definitions for OTFC Controller
@{ */

#define OTFC_CTL_EN0_Pos                 (0)                                               /*!< OTFC_T::CTL: EN0 Position              */
#define OTFC_CTL_EN0_Msk                 (0x1ul << OTFC_CTL_EN0_Pos)                       /*!< OTFC_T::CTL: EN0 Mask                  */

#define OTFC_CTL_RST0_Pos                (1)                                               /*!< OTFC_T::CTL: RST0 Position             */
#define OTFC_CTL_RST0_Msk                (0x1ul << OTFC_CTL_RST0_Pos)                      /*!< OTFC_T::CTL: RST0 Mask                 */

#define OTFC_CTL_KSWAP0_Pos              (2)                                               /*!< OTFC_T::CTL: KSWAP0 Position           */
#define OTFC_CTL_KSWAP0_Msk              (0x1ul << OTFC_CTL_KSWAP0_Pos)                    /*!< OTFC_T::CTL: KSWAP0 Mask               */

#define OTFC_CTL_IEN0_Pos                (7)                                               /*!< OTFC_T::CTL: IEN0 Position             */
#define OTFC_CTL_IEN0_Msk                (0x1ul << OTFC_CTL_IEN0_Pos)                      /*!< OTFC_T::CTL: IEN0 Mask                 */

#define OTFC_CTL_EN1_Pos                 (8)                                               /*!< OTFC_T::CTL: EN1 Position              */
#define OTFC_CTL_EN1_Msk                 (0x1ul << OTFC_CTL_EN1_Pos)                       /*!< OTFC_T::CTL: EN1 Mask                  */

#define OTFC_CTL_RST1_Pos                (9)                                               /*!< OTFC_T::CTL: RST1 Position             */
#define OTFC_CTL_RST1_Msk                (0x1ul << OTFC_CTL_RST1_Pos)                      /*!< OTFC_T::CTL: RST1 Mask                 */

#define OTFC_CTL_KSWAP1_Pos              (10)                                              /*!< OTFC_T::CTL: KSWAP1 Position           */
#define OTFC_CTL_KSWAP1_Msk              (0x1ul << OTFC_CTL_KSWAP1_Pos)                    /*!< OTFC_T::CTL: KSWAP1 Mask               */

#define OTFC_CTL_IEN1_Pos                (15)                                              /*!< OTFC_T::CTL: IEN1 Position             */
#define OTFC_CTL_IEN1_Msk                (0x1ul << OTFC_CTL_IEN1_Pos)                      /*!< OTFC_T::CTL: IEN1 Mask                 */

#define OTFC_CTL_EN2_Pos                 (16)                                              /*!< OTFC_T::CTL: EN2 Position              */
#define OTFC_CTL_EN2_Msk                 (0x1ul << OTFC_CTL_EN2_Pos)                       /*!< OTFC_T::CTL: EN2 Mask                  */

#define OTFC_CTL_RST2_Pos                (17)                                              /*!< OTFC_T::CTL: RST2 Position             */
#define OTFC_CTL_RST2_Msk                (0x1ul << OTFC_CTL_RST2_Pos)                      /*!< OTFC_T::CTL: RST2 Mask                 */

#define OTFC_CTL_KSWAP2_Pos              (18)                                              /*!< OTFC_T::CTL: KSWAP2 Position           */
#define OTFC_CTL_KSWAP2_Msk              (0x1ul << OTFC_CTL_KSWAP2_Pos)                    /*!< OTFC_T::CTL: KSWAP2 Mask               */

#define OTFC_CTL_IEN2_Pos                (23)                                              /*!< OTFC_T::CTL: IEN2 Position             */
#define OTFC_CTL_IEN2_Msk                (0x1ul << OTFC_CTL_IEN2_Pos)                      /*!< OTFC_T::CTL: IEN2 Mask                 */

#define OTFC_CTL_EN3_Pos                 (24)                                              /*!< OTFC_T::CTL: EN3 Position              */
#define OTFC_CTL_EN3_Msk                 (0x1ul << OTFC_CTL_EN3_Pos)                       /*!< OTFC_T::CTL: EN3 Mask                  */

#define OTFC_CTL_RST3_Pos                (25)                                              /*!< OTFC_T::CTL: RST3 Position             */
#define OTFC_CTL_RST3_Msk                (0x1ul << OTFC_CTL_RST3_Pos)                      /*!< OTFC_T::CTL: RST3 Mask                 */

#define OTFC_CTL_KSWAP3_Pos              (26)                                              /*!< OTFC_T::CTL: KSWAP3 Position           */
#define OTFC_CTL_KSWAP3_Msk              (0x1ul << OTFC_CTL_KSWAP3_Pos)                    /*!< OTFC_T::CTL: KSWAP3 Mask               */

#define OTFC_CTL_IEN3_Pos                (31)                                              /*!< OTFC_T::CTL: IEN3 Position             */
#define OTFC_CTL_IEN3_Msk                (0x1ul << OTFC_CTL_IEN3_Pos)                      /*!< OTFC_T::CTL: IEN3 Mask                 */

#define OTFC_STS_BUSY0_Pos               (0)                                               /*!< OTFC_T::STS: BUSY0 Position            */
#define OTFC_STS_BUSY0_Msk               (0x1ul << OTFC_STS_BUSY0_Pos)                     /*!< OTFC_T::STS: BUSY0 Mask                */

#define OTFC_STS_KSERR0_Pos              (5)                                               /*!< OTFC_T::STS: KSERR0 Position           */
#define OTFC_STS_KSERR0_Msk              (0x1ul << OTFC_STS_KSERR0_Pos)                    /*!< OTFC_T::STS: KSERR0 Mask               */

#define OTFC_STS_DFAERR0_Pos             (6)                                               /*!< OTFC_T::STS: DFAERR0 Position          */
#define OTFC_STS_DFAERR0_Msk             (0x1ul << OTFC_STS_DFAERR0_Pos)                   /*!< OTFC_T::STS: DFAERR0 Mask              */

#define OTFC_STS_IF0_Pos                 (7)                                               /*!< OTFC_T::STS: IF0 Position              */
#define OTFC_STS_IF0_Msk                 (0x1ul << OTFC_STS_IF0_Pos)                       /*!< OTFC_T::STS: IF0 Mask                  */

#define OTFC_STS_BUSY1_Pos               (8)                                               /*!< OTFC_T::STS: BUSY1 Position            */
#define OTFC_STS_BUSY1_Msk               (0x1ul << OTFC_STS_BUSY1_Pos)                     /*!< OTFC_T::STS: BUSY1 Mask                */

#define OTFC_STS_KSERR1_Pos              (13)                                              /*!< OTFC_T::STS: KSERR1 Position           */
#define OTFC_STS_KSERR1_Msk              (0x1ul << OTFC_STS_KSERR1_Pos)                    /*!< OTFC_T::STS: KSERR1 Mask               */

#define OTFC_STS_DFAERR1_Pos             (14)                                              /*!< OTFC_T::STS: DFAERR1 Position          */
#define OTFC_STS_DFAERR1_Msk             (0x1ul << OTFC_STS_DFAERR1_Pos)                   /*!< OTFC_T::STS: DFAERR1 Mask              */

#define OTFC_STS_IF1_Pos                 (15)                                              /*!< OTFC_T::STS: IF1 Position              */
#define OTFC_STS_IF1_Msk                 (0x1ul << OTFC_STS_IF1_Pos)                       /*!< OTFC_T::STS: IF1 Mask                  */

#define OTFC_STS_BUSY2_Pos               (16)                                              /*!< OTFC_T::STS: BUSY2 Position            */
#define OTFC_STS_BUSY2_Msk               (0x1ul << OTFC_STS_BUSY2_Pos)                     /*!< OTFC_T::STS: BUSY2 Mask                */

#define OTFC_STS_KSERR2_Pos              (21)                                              /*!< OTFC_T::STS: KSERR2 Position           */
#define OTFC_STS_KSERR2_Msk              (0x1ul << OTFC_STS_KSERR2_Pos)                    /*!< OTFC_T::STS: KSERR2 Mask               */

#define OTFC_STS_DFAERR2_Pos             (22)                                              /*!< OTFC_T::STS: DFAERR2 Position          */
#define OTFC_STS_DFAERR2_Msk             (0x1ul << OTFC_STS_DFAERR2_Pos)                   /*!< OTFC_T::STS: DFAERR2 Mask              */

#define OTFC_STS_IF2_Pos                 (23)                                              /*!< OTFC_T::STS: IF2 Position              */
#define OTFC_STS_IF2_Msk                 (0x1ul << OTFC_STS_IF2_Pos)                       /*!< OTFC_T::STS: IF2 Mask                  */

#define OTFC_STS_BUSY3_Pos               (24)                                              /*!< OTFC_T::STS: BUSY3 Position            */
#define OTFC_STS_BUSY3_Msk               (0x1ul << OTFC_STS_BUSY3_Pos)                     /*!< OTFC_T::STS: BUSY3 Mask                */

#define OTFC_STS_KSERR3_Pos              (29)                                              /*!< OTFC_T::STS: KSERR3 Position           */
#define OTFC_STS_KSERR3_Msk              (0x1ul << OTFC_STS_KSERR3_Pos)                    /*!< OTFC_T::STS: KSERR3 Mask               */

#define OTFC_STS_DFAERR3_Pos             (30)                                              /*!< OTFC_T::STS: DFAERR3 Position          */
#define OTFC_STS_DFAERR3_Msk             (0x1ul << OTFC_STS_DFAERR3_Pos)                   /*!< OTFC_T::STS: DFAERR3 Mask              */

#define OTFC_STS_IF3_Pos                 (31)                                              /*!< OTFC_T::STS: IF3 Position              */
#define OTFC_STS_IF3_Msk                 (0x1ul << OTFC_STS_IF3_Pos)                       /*!< OTFC_T::STS: IF3 Mask                  */

#define OTFC_PR_SADDR_SADDR_Pos          (0)                                               /*!< OTFC_T::PR_SADDR: SADDR Position      */
#define OTFC_PR_SADDR_SADDR_Msk          (0xfffffffful << OTFC_PR_SADDR_SADDR_Pos)         /*!< OTFC_T::PR_SADDR: SADDR Mask          */

#define OTFC_PR_EADDR_EADDR_Pos          (0)                                               /*!< OTFC_T::PR_EADDR: EADDR Position      */
#define OTFC_PR_EADDR_EADDR_Msk          (0xfffffffful << OTFC_PR_EADDR_EADDR_Pos)         /*!< OTFC_T::PR_EADDR: EADDR Mask          */

#define OTFC_PR_KSCTL_NUM_Pos            (0)                                               /*!< OTFC_T::PR_KSCTL: NUM Position        */
#define OTFC_PR_KSCTL_NUM_Msk            (0x1ful << OTFC_PR_KSCTL_NUM_Pos)                 /*!< OTFC_T::PR_KSCTL: NUM Mask            */

#define OTFC_PR_KSCTL_RSRC_Pos           (5)                                               /*!< OTFC_T::PR_KSCTL: RSRC Position       */
#define OTFC_PR_KSCTL_RSRC_Msk           (0x1ul << OTFC_PR_KSCTL_RSRC_Pos)                 /*!< OTFC_T::PR_KSCTL: RSRC Mask           */

#define OTFC_PR_KSCTL_RSSRC_Pos          (6)                                               /*!< OTFC_T::PR_KSCTL: RSSRC Position      */
#define OTFC_PR_KSCTL_RSSRC_Msk          (0x3ul << OTFC_PR_KSCTL_RSSRC_Pos)                /*!< OTFC_T::PR_KSCTL: RSSRC Mask          */

#define OTFC_PR_KEY0_KEY_Pos             (0)                                               /*!< OTFC_T::PR_KEY0: KEY Position         */
#define OTFC_PR_KEY0_KEY_Msk             (0xfffffffful << OTFC_PR_KEY0_KEY_Pos)            /*!< OTFC_T::PR_KEY0: KEY Mask             */

#define OTFC_PR_KEY1_KEY_Pos             (0)                                               /*!< OTFC_T::PR_KEY1: KEY Position         */
#define OTFC_PR_KEY1_KEY_Msk             (0xfffffffful << OTFC_PR_KEY1_KEY_Pos)            /*!< OTFC_T::PR_KEY1: KEY Mask             */

#define OTFC_PR_KEY2_KEY_Pos             (0)                                               /*!< OTFC_T::PR_KEY2: KEY Position         */
#define OTFC_PR_KEY2_KEY_Msk             (0xfffffffful << OTFC_PR_KEY2_KEY_Pos)            /*!< OTFC_T::PR_KEY2: KEY Mask             */

#define OTFC_PR_KEY3_KEY_Pos             (0)                                               /*!< OTFC_T::PR_KEY3: KEY Position         */
#define OTFC_PR_KEY3_KEY_Msk             (0xfffffffful << OTFC_PR_KEY3_KEY_Pos)            /*!< OTFC_T::PR_KEY3: KEY Mask             */

#define OTFC_PR_SCRAMBLE_SCRAMBLE_Pos    (0)                                               /*!< OTFC_T::PR_SCRAMBLE: SCRAMBLE Position*/
#define OTFC_PR_SCRAMBLE_SCRAMBLE_Msk    (0xfffffffful << OTFC_PR_SCRAMBLE_SCRAMBLE_Pos)   /*!< OTFC_T::PR_SCRAMBLE: SCRAMBLE Mask    */

#define OTFC_PR_NONCE0_NONCE_Pos         (0)                                               /*!< OTFC_T::PR_NONCE0: NONCE Position     */
#define OTFC_PR_NONCE0_NONCE_Msk         (0xfffffffful << OTFC_PR_NONCE0_NONCE_Pos)        /*!< OTFC_T::PR_NONCE0: NONCE Mask         */

#define OTFC_PR_NONCE1_NONCE_Pos         (0)                                               /*!< OTFC_T::PR_NONCE1: NONCE Position     */
#define OTFC_PR_NONCE1_NONCE_Msk         (0xfffffffful << OTFC_PR_NONCE1_NONCE_Pos)        /*!< OTFC_T::PR_NONCE1: NONCE Mask         */

#define OTFC_PR_NONCE2_NONCE_Pos         (0)                                               /*!< OTFC_T::PR_NONCE2: NONCE Position     */
#define OTFC_PR_NONCE2_NONCE_Msk         (0xfffffffful << OTFC_PR_NONCE2_NONCE_Pos)        /*!< OTFC_T::PR_NONCE2: NONCE Mask         */

/** @} OTFC_CONST */
/** @} end of OTFC register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __OTFC_REG_H__ */
