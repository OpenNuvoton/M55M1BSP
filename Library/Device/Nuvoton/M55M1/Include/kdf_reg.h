/**************************************************************************//**
 * @file     kdf_reg.h
 * @version  V1.00
 * @brief    KDF register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __KDF_REG_H__
#define __KDF_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup KDF Key Derivation Function (KDF)
    Memory Mapped Structure for KDF Controller
    @{
*/

typedef struct
{
    /**
     * @var KDF_T::CTL
     * Offset: 0x00  KDF Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |START     |KDF Start
     * |        |          |0 = No operation.
     * |        |          |1 = Start the operation.
     * |[1]     |NEXT      |KDF Next Start
     * |        |          |0 = No effect.
     * |        |          |1 = Start KDF to generate the next partial key only when BUSY is 0. BUSY flag will be set.
     * |[2]     |FINISH    |KDF Next Finish
     * |        |          |0 = No effect.
     * |        |          |1 = Finish generating the next partial key.
     * |[4]     |MODE      |KDF Mode Selection
     * |        |          |0 = HKDF (RFC 5869).
     * |        |          |1 = KBKDF (NIST SP 800-108).
     * |[9:8]   |KEYINSEL  |KDF Input Key Source Selection
     * |        |          |00 = KEYIN from KDF_KEYIN register.
     * |        |          |01 = KEYIN from NVM (Flash/RRAM).
     * |        |          |1x = KEYIN from PUF.
     * |        |          |Others = reserved.
     * |[10]    |SALTSEL   |KDF Salt Selection
     * |        |          |0 = SALT from KDF_SALT register.
     * |        |          |1 = SALT from random value.
     * |[11]    |LBSEL     |KDF Label Selection
     * |        |          |0 = LABEL from KDF_LABEL register.
     * |        |          |1 = LABEL from random value.
     * |[12]    |CTXTSEL   |KDF Context Selection
     * |        |          |0 = CONTEXT from KDF_CONTEXT register.
     * |        |          |1 = CONTEXT from random value.
     * @var KDF_T::STS
     * Offset: 0x04  KDF Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BUSY      |KDF Busy Flag
     * |        |          |0 = KDF is idle or finished.
     * |        |          |1 = KDF is busy.
     * |[1]     |HMACBUSY  |HMAC Busy Flag
     * |        |          |0 = HMAC engine is idle or finished.
     * |        |          |1 = HMAC engine is busy.
     * |[16]    |KSERR     |KDF Access Key Store Error Flag
     * |        |          |0 = No error.
     * |        |          |1 = Access Key Store failed.
     * |[17]    |NEXTERR   |KDF Next Generation Error Flag
     * |        |          |0 = No error.
     * |        |          |1 = NEXT(KS_CTL[1]) times is larger than KLEN/256.
     * @var KDF_T::KLEN
     * Offset: 0x08  KDF Output Key Length Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |KLEN      |Output Key Length
     * |        |          |The maxinum is 0xFF00 (65280-bit).
     * |        |          |If KLEN is 0 or larger than maximum, KDF will not execute.
     * @var KDF_T::KEYIN[8]
     * Offset: 0x40 ~ 0x5C  KDF Input Key Word 0 ~ 7 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |KEYIN     |Input Key
     * |        |          |For RFC 5869, input keying material is to be filled in KEYIN.
     * |        |          |For NIST SP 800-108, KI is to be filled in KEYIN.
     * @var KDF_T::KEYOUT[8]
     * Offset: 0x80 ~ 0x9C  KDF Output Key Word 0 ~ 7 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |KEYOUT    |Output Key
     * |        |          |These registers will be cleared if CPU completes the reading key.
     * @var KDF_T::SALT[8]
     * Offset: 0x100 ~ 0x11C  KDF Salt 0 ~ 7 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |SALT      |Salt
     * |        |          |These registers are for RFC 5869.
     * @var KDF_T::LABEL[3]
     * Offset: 0x200 ~ 0x208  KDF Label 0 ~ 2 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |LABEL     |Label
     * |        |          |For RFC 5869, partial info is to be filled in LABEL according to Figure 1.1-2.
     * |        |          |For NIST SP 800-108, Label is to be filled in LABEL.
     * @var KDF_T::CTXT[4]
     * Offset: 0x280 ~ 0x28C  KDF Context 0 ~ 3 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |CTXT      |Context
     * |        |          |For RFC 5869, partial info is to be filled in CTXT according to Figure 1.1-2.
     * |        |          |For NIST SP 800-108, Context is to be filled in CTXT.
     * @var KDF_T::KSCTL
     * Offset: 0xF00  KDF Output Key Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[16]    |TRUST     |Write Key Trust Selection Bit
     * |        |          |0 = Set written key as the non-secure key.
     * |        |          |1 = Set written key as the secure key.
     * |[17]    |PRIV      |Write Key Priviledge Selection Bit
     * |        |          |0 = Set written key as the non-priviledged key.
     * |        |          |1 = Set written key as the priviledged key.
     * |[21]    |WDST      |Write Key Destination
     * |        |          |0 = Key is written to KDF_KEYOUT register.
     * |        |          |1 = Key is written to Key Store.
     * |[23:22] |WSDST     |Write Key Store Destination
     * |        |          |00 = Key is written to the SRAM of Key Store.
     * |        |          |01 = Key is written to the Flash of Key Store.
     * |        |          |Others = reserved.
     * |[26:24] |OWNER     |Write Key Owner Selection Bits
     * |        |          |000 = Only for AES use.
     * |        |          |001 = Only for HMAC engine use.
     * |        |          |100 = Only for ECC engine use.
     * |        |          |101 = Only for CPU engine use.
     * |        |          |110 = Only for ChaCha engine use.
     * |        |          |Others = reserved.
     * @var KDF_T::KSSTS
     * Offset: 0xF04  KDF Output Key Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |NUM       |Key Number
     * |        |          |The key number is generated by Key Store
     * @var KDF_T::KSSIZE
     * Offset: 0xF08  KDF Output Key Size Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |SIZE      |Key Size Selection Bits
     * |        |          |0000 = 128 bits.
     * |        |          |0001 = 163 bits.
     * |        |          |0010 = 192 bits.
     * |        |          |0011 = 224 bits.
     * |        |          |0100 = 233 bits.
     * |        |          |0101 = 255 bits.
     * |        |          |0110 = 256 bits.
     * |        |          |0111 = 283 bits.
     * |        |          |1000 = 384 bits.
     * |        |          |1001 = 409 bits.
     * |        |          |1010 = 512 bits.
     * |        |          |1011 = 521 bits.
     * |        |          |1100 = 571 bits.
     * |        |          |Others = reserved.
     */
    __IO uint32_t CTL;                   /*!< [0x0000] KDF Control Register                                             */
    __I  uint32_t STS;                   /*!< [0x0004] KDF Status Register                                              */
    __IO uint32_t KLEN;                  /*!< [0x0008] KDF Output Key Length Register                                   */
    __I  uint32_t RESERVE0[13];
    __O  uint32_t KEYIN[8];              /*!< [0x0040-0x005c] KDF Input Key Word 0-7 Register                           */
    __I  uint32_t RESERVE1[8];
    __I  uint32_t KEYOUT[8];             /*!< [0x0080-0x009c] KDF Output Key Word 0-7 Register                          */
    __I  uint32_t RESERVE2[24];
    __O  uint32_t SALT[8];               /*!< [0x0100-0x011c] KDF Salt 0-7 Register                                     */
    __I  uint32_t RESERVE3[56];
    __O  uint32_t LABEL[3];              /*!< [0x0200-0x0208] KDF Label 0-2 Register                                    */
    __I  uint32_t RESERVE4[29];
    __O  uint32_t CTXT[4];               /*!< [0x0280-0x028c] KDF Context 0-3 Register                                  */
    __I  uint32_t RESERVE5[796];
    __O  uint32_t KSCTL;                 /*!< [0x0f00] KDF Output Key Control Register                                  */
    __I  uint32_t KSSTS;                 /*!< [0x0f04] KDF Output Key Status Register                                   */
    __IO uint32_t KSSIZE;                /*!< [0x0f08] KDF Output Key Size Register                                     */

} KDF_T;

/**
    @addtogroup KDF_CONST KDF Bit Field Definition
    Constant Definitions for KDF Controller
    @{
*/

#define KDF_CTL_START_Pos                (0)                                               /*!< KDF_T::CTL: START Position             */
#define KDF_CTL_START_Msk                (0x1ul << KDF_CTL_START_Pos)                      /*!< KDF_T::CTL: START Mask                 */

#define KDF_CTL_NEXT_Pos                 (1)                                               /*!< KDF_T::CTL: NEXT Position              */
#define KDF_CTL_NEXT_Msk                 (0x1ul << KDF_CTL_NEXT_Pos)                       /*!< KDF_T::CTL: NEXT Mask                  */

#define KDF_CTL_FINISH_Pos               (2)                                               /*!< KDF_T::CTL: FINISH Position            */
#define KDF_CTL_FINISH_Msk               (0x1ul << KDF_CTL_FINISH_Pos)                     /*!< KDF_T::CTL: FINISH Mask                */

#define KDF_CTL_MODE_Pos                 (4)                                               /*!< KDF_T::CTL: MODE Position              */
#define KDF_CTL_MODE_Msk                 (0x1ul << KDF_CTL_MODE_Pos)                       /*!< KDF_T::CTL: MODE Mask                  */

#define KDF_CTL_KEYINSEL_Pos             (8)                                               /*!< KDF_T::CTL: KEYINSEL Position          */
#define KDF_CTL_KEYINSEL_Msk             (0x3ul << KDF_CTL_KEYINSEL_Pos)                   /*!< KDF_T::CTL: KEYINSEL Mask              */

#define KDF_CTL_SALTSEL_Pos              (10)                                              /*!< KDF_T::CTL: SALTSEL Position           */
#define KDF_CTL_SALTSEL_Msk              (0x1ul << KDF_CTL_SALTSEL_Pos)                    /*!< KDF_T::CTL: SALTSEL Mask               */

#define KDF_CTL_LBSEL_Pos                (11)                                              /*!< KDF_T::CTL: LBSEL Position             */
#define KDF_CTL_LBSEL_Msk                (0x1ul << KDF_CTL_LBSEL_Pos)                      /*!< KDF_T::CTL: LBSEL Mask                 */

#define KDF_CTL_CTXTSEL_Pos              (12)                                              /*!< KDF_T::CTL: CTXTSEL Position           */
#define KDF_CTL_CTXTSEL_Msk              (0x1ul << KDF_CTL_CTXTSEL_Pos)                    /*!< KDF_T::CTL: CTXTSEL Mask               */

#define KDF_STS_BUSY_Pos                 (0)                                               /*!< KDF_T::STS: BUSY Position              */
#define KDF_STS_BUSY_Msk                 (0x1ul << KDF_STS_BUSY_Pos)                       /*!< KDF_T::STS: BUSY Mask                  */

#define KDF_STS_HMACBUSY_Pos             (1)                                               /*!< KDF_T::STS: HMACBUSY Position          */
#define KDF_STS_HMACBUSY_Msk             (0x1ul << KDF_STS_HMACBUSY_Pos)                   /*!< KDF_T::STS: HMACBUSY Mask              */

#define KDF_STS_KSERR_Pos                (16)                                              /*!< KDF_T::STS: KSERR Position             */
#define KDF_STS_KSERR_Msk                (0x1ul << KDF_STS_KSERR_Pos)                      /*!< KDF_T::STS: KSERR Mask                 */

#define KDF_STS_NEXTERR_Pos              (17)                                              /*!< KDF_T::STS: NEXTERR Position           */
#define KDF_STS_NEXTERR_Msk              (0x1ul << KDF_STS_NEXTERR_Pos)                    /*!< KDF_T::STS: NEXTERR Mask               */

#define KDF_KLEN_KLEN_Pos                (0)                                               /*!< KDF_T::KLEN: KLEN Position             */
#define KDF_KLEN_KLEN_Msk                (0xfffful << KDF_KLEN_KLEN_Pos)                   /*!< KDF_T::KLEN: KLEN Mask                 */

#define KDF_KEYIN0_KEYIN_Pos             (0)                                               /*!< KDF_T::KEYIN0: KEYIN Position          */
#define KDF_KEYIN0_KEYIN_Msk             (0xfffffffful << KDF_KEYIN0_KEYIN_Pos)            /*!< KDF_T::KEYIN0: KEYIN Mask              */

#define KDF_KEYIN1_KEYIN_Pos             (0)                                               /*!< KDF_T::KEYIN1: KEYIN Position          */
#define KDF_KEYIN1_KEYIN_Msk             (0xfffffffful << KDF_KEYIN1_KEYIN_Pos)            /*!< KDF_T::KEYIN1: KEYIN Mask              */

#define KDF_KEYIN2_KEYIN_Pos             (0)                                               /*!< KDF_T::KEYIN2: KEYIN Position          */
#define KDF_KEYIN2_KEYIN_Msk             (0xfffffffful << KDF_KEYIN2_KEYIN_Pos)            /*!< KDF_T::KEYIN2: KEYIN Mask              */

#define KDF_KEYIN3_KEYIN_Pos             (0)                                               /*!< KDF_T::KEYIN3: KEYIN Position          */
#define KDF_KEYIN3_KEYIN_Msk             (0xfffffffful << KDF_KEYIN3_KEYIN_Pos)            /*!< KDF_T::KEYIN3: KEYIN Mask              */

#define KDF_KEYIN4_KEYIN_Pos             (0)                                               /*!< KDF_T::KEYIN4: KEYIN Position          */
#define KDF_KEYIN4_KEYIN_Msk             (0xfffffffful << KDF_KEYIN4_KEYIN_Pos)            /*!< KDF_T::KEYIN4: KEYIN Mask              */

#define KDF_KEYIN5_KEYIN_Pos             (0)                                               /*!< KDF_T::KEYIN5: KEYIN Position          */
#define KDF_KEYIN5_KEYIN_Msk             (0xfffffffful << KDF_KEYIN5_KEYIN_Pos)            /*!< KDF_T::KEYIN5: KEYIN Mask              */

#define KDF_KEYIN6_KEYIN_Pos             (0)                                               /*!< KDF_T::KEYIN6: KEYIN Position          */
#define KDF_KEYIN6_KEYIN_Msk             (0xfffffffful << KDF_KEYIN6_KEYIN_Pos)            /*!< KDF_T::KEYIN6: KEYIN Mask              */

#define KDF_KEYIN7_KEYIN_Pos             (0)                                               /*!< KDF_T::KEYIN7: KEYIN Position          */
#define KDF_KEYIN7_KEYIN_Msk             (0xfffffffful << KDF_KEYIN7_KEYIN_Pos)            /*!< KDF_T::KEYIN7: KEYIN Mask              */

#define KDF_KEYOUT0_KEYOUT_Pos           (0)                                               /*!< KDF_T::KEYOUT0: KEYOUT Position        */
#define KDF_KEYOUT0_KEYOUT_Msk           (0xfffffffful << KDF_KEYOUT0_KEYOUT_Pos)          /*!< KDF_T::KEYOUT0: KEYOUT Mask            */

#define KDF_KEYOUT1_KEYOUT_Pos           (0)                                               /*!< KDF_T::KEYOUT1: KEYOUT Position        */
#define KDF_KEYOUT1_KEYOUT_Msk           (0xfffffffful << KDF_KEYOUT1_KEYOUT_Pos)          /*!< KDF_T::KEYOUT1: KEYOUT Mask            */

#define KDF_KEYOUT2_KEYOUT_Pos           (0)                                               /*!< KDF_T::KEYOUT2: KEYOUT Position        */
#define KDF_KEYOUT2_KEYOUT_Msk           (0xfffffffful << KDF_KEYOUT2_KEYOUT_Pos)          /*!< KDF_T::KEYOUT2: KEYOUT Mask            */

#define KDF_KEYOUT3_KEYOUT_Pos           (0)                                               /*!< KDF_T::KEYOUT3: KEYOUT Position        */
#define KDF_KEYOUT3_KEYOUT_Msk           (0xfffffffful << KDF_KEYOUT3_KEYOUT_Pos)          /*!< KDF_T::KEYOUT3: KEYOUT Mask            */

#define KDF_KEYOUT4_KEYOUT_Pos           (0)                                               /*!< KDF_T::KEYOUT4: KEYOUT Position        */
#define KDF_KEYOUT4_KEYOUT_Msk           (0xfffffffful << KDF_KEYOUT4_KEYOUT_Pos)          /*!< KDF_T::KEYOUT4: KEYOUT Mask            */

#define KDF_KEYOUT5_KEYOUT_Pos           (0)                                               /*!< KDF_T::KEYOUT5: KEYOUT Position        */
#define KDF_KEYOUT5_KEYOUT_Msk           (0xfffffffful << KDF_KEYOUT5_KEYOUT_Pos)          /*!< KDF_T::KEYOUT5: KEYOUT Mask            */

#define KDF_KEYOUT6_KEYOUT_Pos           (0)                                               /*!< KDF_T::KEYOUT6: KEYOUT Position        */
#define KDF_KEYOUT6_KEYOUT_Msk           (0xfffffffful << KDF_KEYOUT6_KEYOUT_Pos)          /*!< KDF_T::KEYOUT6: KEYOUT Mask            */

#define KDF_KEYOUT7_KEYOUT_Pos           (0)                                               /*!< KDF_T::KEYOUT7: KEYOUT Position        */
#define KDF_KEYOUT7_KEYOUT_Msk           (0xfffffffful << KDF_KEYOUT7_KEYOUT_Pos)          /*!< KDF_T::KEYOUT7: KEYOUT Mask            */

#define KDF_SALT0_SALT_Pos               (0)                                               /*!< KDF_T::SALT0: SALT Position            */
#define KDF_SALT0_SALT_Msk               (0xfffffffful << KDF_SALT0_SALT_Pos)              /*!< KDF_T::SALT0: SALT Mask                */

#define KDF_SALT1_SALT_Pos               (0)                                               /*!< KDF_T::SALT1: SALT Position            */
#define KDF_SALT1_SALT_Msk               (0xfffffffful << KDF_SALT1_SALT_Pos)              /*!< KDF_T::SALT1: SALT Mask                */

#define KDF_SALT2_SALT_Pos               (0)                                               /*!< KDF_T::SALT2: SALT Position            */
#define KDF_SALT2_SALT_Msk               (0xfffffffful << KDF_SALT2_SALT_Pos)              /*!< KDF_T::SALT2: SALT Mask                */

#define KDF_SALT3_SALT_Pos               (0)                                               /*!< KDF_T::SALT3: SALT Position            */
#define KDF_SALT3_SALT_Msk               (0xfffffffful << KDF_SALT3_SALT_Pos)              /*!< KDF_T::SALT3: SALT Mask                */

#define KDF_SALT4_SALT_Pos               (0)                                               /*!< KDF_T::SALT4: SALT Position            */
#define KDF_SALT4_SALT_Msk               (0xfffffffful << KDF_SALT4_SALT_Pos)              /*!< KDF_T::SALT4: SALT Mask                */

#define KDF_SALT5_SALT_Pos               (0)                                               /*!< KDF_T::SALT5: SALT Position            */
#define KDF_SALT5_SALT_Msk               (0xfffffffful << KDF_SALT5_SALT_Pos)              /*!< KDF_T::SALT5: SALT Mask                */

#define KDF_SALT6_SALT_Pos               (0)                                               /*!< KDF_T::SALT6: SALT Position            */
#define KDF_SALT6_SALT_Msk               (0xfffffffful << KDF_SALT6_SALT_Pos)              /*!< KDF_T::SALT6: SALT Mask                */

#define KDF_SALT7_SALT_Pos               (0)                                               /*!< KDF_T::SALT7: SALT Position            */
#define KDF_SALT7_SALT_Msk               (0xfffffffful << KDF_SALT7_SALT_Pos)              /*!< KDF_T::SALT7: SALT Mask                */

#define KDF_LABEL0_LABEL_Pos             (0)                                               /*!< KDF_T::LABEL0: LABEL Position          */
#define KDF_LABEL0_LABEL_Msk             (0xfffffffful << KDF_LABEL0_LABEL_Pos)            /*!< KDF_T::LABEL0: LABEL Mask              */

#define KDF_LABEL1_LABEL_Pos             (0)                                               /*!< KDF_T::LABEL1: LABEL Position          */
#define KDF_LABEL1_LABEL_Msk             (0xfffffffful << KDF_LABEL1_LABEL_Pos)            /*!< KDF_T::LABEL1: LABEL Mask              */

#define KDF_LABEL2_LABEL_Pos             (0)                                               /*!< KDF_T::LABEL2: LABEL Position          */
#define KDF_LABEL2_LABEL_Msk             (0xfffffffful << KDF_LABEL2_LABEL_Pos)            /*!< KDF_T::LABEL2: LABEL Mask              */

#define KDF_CTXT0_CTXT_Pos               (0)                                               /*!< KDF_T::CTXT0: CTXT Position            */
#define KDF_CTXT0_CTXT_Msk               (0xfffffffful << KDF_CTXT0_CTXT_Pos)              /*!< KDF_T::CTXT0: CTXT Mask                */

#define KDF_CTXT1_CTXT_Pos               (0)                                               /*!< KDF_T::CTXT1: CTXT Position            */
#define KDF_CTXT1_CTXT_Msk               (0xfffffffful << KDF_CTXT1_CTXT_Pos)              /*!< KDF_T::CTXT1: CTXT Mask                */

#define KDF_CTXT2_CTXT_Pos               (0)                                               /*!< KDF_T::CTXT2: CTXT Position            */
#define KDF_CTXT2_CTXT_Msk               (0xfffffffful << KDF_CTXT2_CTXT_Pos)              /*!< KDF_T::CTXT2: CTXT Mask                */

#define KDF_CTXT3_CTXT_Pos               (0)                                               /*!< KDF_T::CTXT3: CTXT Position            */
#define KDF_CTXT3_CTXT_Msk               (0xfffffffful << KDF_CTXT3_CTXT_Pos)              /*!< KDF_T::CTXT3: CTXT Mask                */

#define KDF_KSCTL_TRUST_Pos              (16)                                              /*!< KDF_T::KSCTL: TRUST Position           */
#define KDF_KSCTL_TRUST_Msk              (0x1ul << KDF_KSCTL_TRUST_Pos)                    /*!< KDF_T::KSCTL: TRUST Mask               */

#define KDF_KSCTL_PRIV_Pos               (17)                                              /*!< KDF_T::KSCTL: PRIV Position            */
#define KDF_KSCTL_PRIV_Msk               (0x1ul << KDF_KSCTL_PRIV_Pos)                     /*!< KDF_T::KSCTL: PRIV Mask                */

#define KDF_KSCTL_WDST_Pos               (21)                                              /*!< KDF_T::KSCTL: WDST Position            */
#define KDF_KSCTL_WDST_Msk               (0x1ul << KDF_KSCTL_WDST_Pos)                     /*!< KDF_T::KSCTL: WDST Mask                */

#define KDF_KSCTL_WSDST_Pos              (22)                                              /*!< KDF_T::KSCTL: WSDST Position           */
#define KDF_KSCTL_WSDST_Msk              (0x3ul << KDF_KSCTL_WSDST_Pos)                    /*!< KDF_T::KSCTL: WSDST Mask               */

#define KDF_KSCTL_OWNER_Pos              (24)                                              /*!< KDF_T::KSCTL: OWNER Position           */
#define KDF_KSCTL_OWNER_Msk              (0x7ul << KDF_KSCTL_OWNER_Pos)                    /*!< KDF_T::KSCTL: OWNER Mask               */

#define KDF_KSSTS_NUM_Pos                (0)                                               /*!< KDF_T::KSSTS: NUM Position             */
#define KDF_KSSTS_NUM_Msk                (0x1ful << KDF_KSSTS_NUM_Pos)                     /*!< KDF_T::KSSTS: NUM Mask                 */

#define KDF_KSSIZE_SIZE_Pos              (0)                                               /*!< KDF_T::KSSIZE: SIZE Position           */
#define KDF_KSSIZE_SIZE_Msk              (0xful << KDF_KSSIZE_SIZE_Pos)                    /*!< KDF_T::KSSIZE: SIZE Mask               */

/** @} KDF_CONST */
/** @} end of KDF register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __KDF_REG_H__ */
