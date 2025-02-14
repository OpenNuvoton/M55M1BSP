/**************************************************************************//**
 * @file     awf_reg.h
 * @version  V1.00
 * @brief    AWF register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __AWF_REG_H__
#define __AWF_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup AWF Awake Filter Controller (AWF)
    Memory Mapped Structure for AWF Controller
    @{
*/

typedef struct
{
    /**
     * @var AWF_T::CTL
     * Offset: 0x00  AWF Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |HTIEN     |AWF High Threshold Interrupt Enable Bit
     * |        |          |0 = AWF high threshold interrupt Disabled.
     * |        |          |1 = AWF high threshold interrupt Enabled.
     * |[1]     |HTWKEN    |AWF High Threshold Wake Up Enable Bit
     * |        |          |0 = AWF high threshold wake up Disabled.
     * |        |          |1 = AWF high threshold wake up Enabled.
     * |[2]     |LTIEN     |AWF Low Threshold Interrupt Enable Bit
     * |        |          |0 = AWF low threshold interrupt Disabled.
     * |        |          |1 = AWF low threshold interrupt Enabled.
     * |[3]     |LTWKEN    |AWF Low Threshold Wake Up Enable Bit
     * |        |          |0 = AWF low threshold wake up Disabled.
     * |        |          |1 = AWF low threshold wake up Enabled.
     * |[7:4]   |ACUCNT    |Accumulation Count
     * |        |          |The counter defines the acculated data to compare with high threshold and low threshold.The allowed maximum value is 8
     * |        |          |If the write value is larger than 8, ACUCNT will become 0.
     * |[8]     |LPPDMA_EN |LPPDMA Enable Bit
     * |        |          |This bit is used to enable LPPDMA to access AWF when chip is in NPD0/1/3.
     * |        |          |0 = LPPDMA can not access AWF when chip is in NPD0/1/3.
     * |        |          |1 = LPPDMA can access AWF when chip is in NPD0/1/3.
     * @var AWF_T::HTH
     * Offset: 0x04  AWF High Threshold Value Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[18:0]  |AWFHTH    |AWF High Threshold Value Register
     * |        |          |If the accumulated value exceeds the high threshold value, comparing function will activate once set interrupt enable respectively.
     * @var AWF_T::LTH
     * Offset: 0x08  AWF Low Threshold Value Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[18:0]  |AWFLTH    |AWF Low Threshold Value Register
     * |        |          |If the accumulated value is less than the low threshold value, comparing function will activate once set interrupt enable respectively.
     * @var AWF_T::STATUS
     * Offset: 0x0C  AWF Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |HTHIS     |AWF HTH Interupt Flag
     * |        |          |0 = AWF HTH interrupt event has not occurred.
     * |        |          |1 = AWF HTH interrupt event occurs.
     * |        |          |Note: This bit is set to 1 when ACUCNT is not zero and accumulation data is over than HTH
     * |        |          |User writes 1 to clear this bit to 0
     * |        |          |This bit will keep 1 if accumulation data is over than HTH after user writes 1 to clear.
     * |[1]     |LTHIS     |AWF LTH Interupt Flag
     * |        |          |0 = AWF LTH interrupt event has not occurred.
     * |        |          |1 = AWF LTH interrupt event occurs.
     * |        |          |Note: This bit is set to 1 when ACUCNT is not zero and accumulation data is less than LTH
     * |        |          |User writes 1 to clear this bit to 0
     * |        |          |This bit wil keep 1 if accumulation data is less than LTH after user writes 1 to clear.
     * @var AWF_T::WBINIT
     * Offset: 0x10  AWF Word Buffer Initial Value Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |WBINIT    |AWF Word Buffer Initial Value
     * |        |          |Writing this register will update WBINIT into all word registers (AWF_W0 ~ AWF_W7).
     * |        |          |WBINIT will also be updated into all word registers (AWF_W0 ~ AWF_W7) once ACUCNT is updated.
     * @var AWF_T::DAT
     * Offset: 0x1C  AWF Data Holding Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |AWFDAT    |AWF Data Holding Register
     * |        |          |These bits are written by user software and will be filled in circularly from word0 to word[ACUCNT-1].
     * @var AWF_T::W0
     * Offset: 0x20  AWF Word Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |WORD      |AWF Word Register
     * |        |          |Word will accumulate from Word0 to Word[ACUCNT-1], and will compare with AWF_HTH and AWF_LTH.
     * @var AWF_T::W1
     * Offset: 0x24  AWF Word Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |WORD      |AWF Word Register
     * |        |          |Word will accumulate from Word0 to Word[ACUCNT-1], and will compare with AWF_HTH and AWF_LTH.
     * @var AWF_T::W2
     * Offset: 0x28  AWF Word Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |WORD      |AWF Word Register
     * |        |          |Word will accumulate from Word0 to Word[ACUCNT-1], and will compare with AWF_HTH and AWF_LTH.
     * @var AWF_T::W3
     * Offset: 0x2C  AWF Word Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |WORD      |AWF Word Register
     * |        |          |Word will accumulate from Word0 to Word[ACUCNT-1], and will compare with AWF_HTH and AWF_LTH.
     * @var AWF_T::W4
     * Offset: 0x30  AWF Word Register 4
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |WORD      |AWF Word Register
     * |        |          |Word will accumulate from Word0 to Word[ACUCNT-1], and will compare with AWF_HTH and AWF_LTH.
     * @var AWF_T::W5
     * Offset: 0x34  AWF Word Register 5
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |WORD      |AWF Word Register
     * |        |          |Word will accumulate from Word0 to Word[ACUCNT-1], and will compare with AWF_HTH and AWF_LTH.
     * @var AWF_T::W6
     * Offset: 0x38  AWF Word Register 6
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |WORD      |AWF Word Register
     * |        |          |Word will accumulate from Word0 to Word[ACUCNT-1], and will compare with AWF_HTH and AWF_LTH.
     * @var AWF_T::W7
     * Offset: 0x3C  AWF Word Register 7
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |WORD      |AWF Word Register
     * |        |          |Word will accumulate from Word0 to Word[ACUCNT-1], and will compare with AWF_HTH and AWF_LTH.
     * @var AWF_T::ACUVAL
     * Offset: 0x40  AWF Accumulated Value Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[18:0]  |ACUVAL    |AWF Accumulated Value Register
     * |        |          |Accumulated value will be accumulated from word0 to word[ACUCNT-1].
     * @var AWF_T::VERSION
     * Offset: 0xFFC  AWF Version Number Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |MINOR     |Minor Version Number
     * |        |          |Minor version number is dependent on ECO version control
     * |        |          |0x0000: (current Minor Version Number)
     * |[23:16] |SUB       |Sub Version Number
     * |        |          |Sub version number is relative to key feature
     * |        |          |0x00: (current Sub Version Number)
     * |[31:24] |MAJOR     |Major Version Number
     * |        |          |Major version number is correlated to Product Line
     * |        |          |0x00: (current Major Version Number)
     */
    __IO uint32_t CTL;                   /*!< [0x0000] AWF Control Register                                             */
    __IO uint32_t HTH;                   /*!< [0x0004] AWF High Threshold Value Register                                */
    __IO uint32_t LTH;                   /*!< [0x0008] AWF Low Threshold Value Register                                 */
    __IO uint32_t STATUS;                /*!< [0x000c] AWF Status Register                                              */
    __IO uint32_t WBINIT;                /*!< [0x0010] AWF Word Buffer Initial Value Register                           */
    __I  uint32_t RESERVE0[2];
    __IO uint32_t DAT;                   /*!< [0x001c] AWF Data Holding Register                                        */
    __I  uint32_t W0;                    /*!< [0x0020] AWF Word Register 0                                              */
    __I  uint32_t W1;                    /*!< [0x0024] AWF Word Register 1                                              */
    __I  uint32_t W2;                    /*!< [0x0028] AWF Word Register 2                                              */
    __I  uint32_t W3;                    /*!< [0x002c] AWF Word Register 3                                              */
    __I  uint32_t W4;                    /*!< [0x0030] AWF Word Register 4                                              */
    __I  uint32_t W5;                    /*!< [0x0034] AWF Word Register 5                                              */
    __I  uint32_t W6;                    /*!< [0x0038] AWF Word Register 6                                              */
    __I  uint32_t W7;                    /*!< [0x003c] AWF Word Register 7                                              */
    __I  uint32_t ACUVAL;                /*!< [0x0040] AWF Accumulated Value Register                                   */
    __I  uint32_t RESERVE1[1006];
    __I  uint32_t VERSION;               /*!< [0x0ffc] AWF Version Number Register                                      */

} AWF_T;

/**
    @addtogroup AWF_CONST AWF Bit Field Definition
    Constant Definitions for AWF Controller
@{ */

#define AWF_CTL_HTIEN_Pos                (0)                                               /*!< AWF_T::CTL: HTIEN Position             */
#define AWF_CTL_HTIEN_Msk                (0x1ul << AWF_CTL_HTIEN_Pos)                      /*!< AWF_T::CTL: HTIEN Mask                 */

#define AWF_CTL_HTWKEN_Pos               (1)                                               /*!< AWF_T::CTL: HTWKEN Position            */
#define AWF_CTL_HTWKEN_Msk               (0x1ul << AWF_CTL_HTWKEN_Pos)                     /*!< AWF_T::CTL: HTWKEN Mask                */

#define AWF_CTL_LTIEN_Pos                (2)                                               /*!< AWF_T::CTL: LTIEN Position             */
#define AWF_CTL_LTIEN_Msk                (0x1ul << AWF_CTL_LTIEN_Pos)                      /*!< AWF_T::CTL: LTIEN Mask                 */

#define AWF_CTL_LTWKEN_Pos               (3)                                               /*!< AWF_T::CTL: LTWKEN Position            */
#define AWF_CTL_LTWKEN_Msk               (0x1ul << AWF_CTL_LTWKEN_Pos)                     /*!< AWF_T::CTL: LTWKEN Mask                */

#define AWF_CTL_ACUCNT_Pos               (4)                                               /*!< AWF_T::CTL: ACUCNT Position            */
#define AWF_CTL_ACUCNT_Msk               (0xful << AWF_CTL_ACUCNT_Pos)                     /*!< AWF_T::CTL: ACUCNT Mask                */

#define AWF_CTL_LPPDMA_EN_Pos            (8)                                               /*!< AWF_T::CTL: LPPDMA_EN Position         */
#define AWF_CTL_LPPDMA_EN_Msk            (0x1ul << AWF_CTL_LPPDMA_EN_Pos)                  /*!< AWF_T::CTL: LPPDMA_EN Mask             */

#define AWF_HTH_AWFHTH_Pos               (0)                                               /*!< AWF_T::HTH: AWFHTH Position            */
#define AWF_HTH_AWFHTH_Msk               (0x7fffful << AWF_HTH_AWFHTH_Pos)                 /*!< AWF_T::HTH: AWFHTH Mask                */

#define AWF_LTH_AWFLTH_Pos               (0)                                               /*!< AWF_T::LTH: AWFLTH Position            */
#define AWF_LTH_AWFLTH_Msk               (0x7fffful << AWF_LTH_AWFLTH_Pos)                 /*!< AWF_T::LTH: AWFLTH Mask                */

#define AWF_STATUS_HTHIS_Pos             (0)                                               /*!< AWF_T::STATUS: HTHIS Position          */
#define AWF_STATUS_HTHIS_Msk             (0x1ul << AWF_STATUS_HTHIS_Pos)                   /*!< AWF_T::STATUS: HTHIS Mask              */

#define AWF_STATUS_LTHIS_Pos             (1)                                               /*!< AWF_T::STATUS: LTHIS Position          */
#define AWF_STATUS_LTHIS_Msk             (0x1ul << AWF_STATUS_LTHIS_Pos)                   /*!< AWF_T::STATUS: LTHIS Mask              */

#define AWF_WBINIT_WBINIT_Pos            (0)                                               /*!< AWF_T::WBINIT: WBINIT Position         */
#define AWF_WBINIT_WBINIT_Msk            (0xfffful << AWF_WBINIT_WBINIT_Pos)               /*!< AWF_T::WBINIT: WBINIT Mask             */

#define AWF_DAT_AWFDAT_Pos               (0)                                               /*!< AWF_T::DAT: AWFDAT Position            */
#define AWF_DAT_AWFDAT_Msk               (0xfffful << AWF_DAT_AWFDAT_Pos)                  /*!< AWF_T::DAT: AWFDAT Mask                */

#define AWF_W0_WORD_Pos                  (0)                                               /*!< AWF_T::W0: WORD Position               */
#define AWF_W0_WORD_Msk                  (0xfffful << AWF_W0_WORD_Pos)                     /*!< AWF_T::W0: WORD Mask                   */

#define AWF_W1_WORD_Pos                  (0)                                               /*!< AWF_T::W1: WORD Position               */
#define AWF_W1_WORD_Msk                  (0xfffful << AWF_W1_WORD_Pos)                     /*!< AWF_T::W1: WORD Mask                   */

#define AWF_W2_WORD_Pos                  (0)                                               /*!< AWF_T::W2: WORD Position               */
#define AWF_W2_WORD_Msk                  (0xfffful << AWF_W2_WORD_Pos)                     /*!< AWF_T::W2: WORD Mask                   */

#define AWF_W3_WORD_Pos                  (0)                                               /*!< AWF_T::W3: WORD Position               */
#define AWF_W3_WORD_Msk                  (0xfffful << AWF_W3_WORD_Pos)                     /*!< AWF_T::W3: WORD Mask                   */

#define AWF_W4_WORD_Pos                  (0)                                               /*!< AWF_T::W4: WORD Position               */
#define AWF_W4_WORD_Msk                  (0xfffful << AWF_W4_WORD_Pos)                     /*!< AWF_T::W4: WORD Mask                   */

#define AWF_W5_WORD_Pos                  (0)                                               /*!< AWF_T::W5: WORD Position               */
#define AWF_W5_WORD_Msk                  (0xfffful << AWF_W5_WORD_Pos)                     /*!< AWF_T::W5: WORD Mask                   */

#define AWF_W6_WORD_Pos                  (0)                                               /*!< AWF_T::W6: WORD Position               */
#define AWF_W6_WORD_Msk                  (0xfffful << AWF_W6_WORD_Pos)                     /*!< AWF_T::W6: WORD Mask                   */

#define AWF_W7_WORD_Pos                  (0)                                               /*!< AWF_T::W7: WORD Position               */
#define AWF_W7_WORD_Msk                  (0xfffful << AWF_W7_WORD_Pos)                     /*!< AWF_T::W7: WORD Mask                   */

#define AWF_ACUVAL_ACUVAL_Pos            (0)                                               /*!< AWF_T::ACUVAL: ACUVAL Position         */
#define AWF_ACUVAL_ACUVAL_Msk            (0x7fffful << AWF_ACUVAL_ACUVAL_Pos)              /*!< AWF_T::ACUVAL: ACUVAL Mask             */

#define AWF_VERSION_MINOR_Pos            (0)                                               /*!< AWF_T::VERSION: MINOR Position         */
#define AWF_VERSION_MINOR_Msk            (0xfffful << AWF_VERSION_MINOR_Pos)               /*!< AWF_T::VERSION: MINOR Mask             */

#define AWF_VERSION_SUB_Pos              (16)                                              /*!< AWF_T::VERSION: SUB Position           */
#define AWF_VERSION_SUB_Msk              (0xfful << AWF_VERSION_SUB_Pos)                   /*!< AWF_T::VERSION: SUB Mask               */

#define AWF_VERSION_MAJOR_Pos            (24)                                              /*!< AWF_T::VERSION: MAJOR Position         */
#define AWF_VERSION_MAJOR_Msk            (0xfful << AWF_VERSION_MAJOR_Pos)                 /*!< AWF_T::VERSION: MAJOR Mask             */

/** @} AWF_CONST */
/** @} end of AWF register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __AWF_REG_H__ */
