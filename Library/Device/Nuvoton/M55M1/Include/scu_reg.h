/**************************************************************************//**
 * @file     scu_reg.h
 * @version  V1.00
 * @brief    SCU register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __SCU_REG_H__
#define __SCU_REG_H__

/**
    @addtogroup REGISTER Control Register
    @{
*/



/*---------------------- Debug Protection Mechanism -------------------------*/
/**
    @addtogroup DPM Debug Protection Mechanism (DPM)
    Memory Mapped Structure for DPM Controller
    @{
*/

typedef struct
{

    /**
     * @var DPM_T::CTL
     * Offset: 0x00  Secure DPM Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DBGDIS    |Set Secure DPM Debug Disable Bit
     * |        |          |When this bit is read as 0, it can be written to 1 to configure the Secure DPM DBGDIS bit (DBGDISS).
     * |        |          |When written:
     * |        |          |0 = No operation.
     * |        |          |1 = Trigger the process to set DBGDISS configuration bit.
     * |        |          |Note: This bit can be set to 1 but cannot be cleared to 0.
     * |[1]     |LOCK      |Set Secure DPM Debug Lock Bit
     * |        |          |When this bit is read as 0, it can be written to 1 to configure the Secure DPM LOCK bit (LOCKS).
     * |        |          |When written:
     * |        |          |0 = No operation.
     * |        |          |1 = Trigger the process to set LOCKS configuration bit.
     * |        |          |Note: This bit can be set to 1 but cannot be cleared to 0.
     * |[2]     |PWCMP     |Secure DPM Password Compare Bit
     * |        |          |Set to enter the process of compare Secure DPM password.
     * |        |          |0 = No operation.
     * |        |          |1 = Compare Secure DPM password.
     * |        |          |Note: This bit will be cleared after the comparison process is finished.
     * |[3]     |PWUPD     |Secure DPM Password Update Bit
     * |        |          |Set to enter the process of updating Secure DPM password.
     * |        |          |0 = No operation.
     * |        |          |1 = Update Secure DPM password.
     * |        |          |Note 1: This bit should be set with PWCMP equal to 0.
     * |        |          |Note 2: This bit will be cleared after the update process is finished.
     * |[8]     |INTEN     |DPM Interrupt Enable Bit
     * |        |          |0 = Disable DPM interrupt function.
     * |        |          |1 = Enable DPM interrupt function.
     * |[12]    |DACCWDIS  |Secure DPM Debug Write Access Disable Bit
     * |        |          |This bit disables the ability of external debugger to set Secure DPM registers for debug authentication.
     * |        |          |0 = External debugger can set Secure DPM registers.
     * |        |          |1 = External debugger cannot set Secure DPM registers.
     * |[13]    |DACCDIS   |Debug Access Disable Bit
     * |        |          |This bit disables the accessibility of external debugger to all DPM registers.
     * |        |          |0 = External debugger can read/write DPM registers.
     * |        |          |1 = External debugger cannot read/write DPM registers.
     * |[31:24] |RWVCODE   |Write Verify Code and Read Verify Code
     * |        |          |Read operation:
     * |        |          |0xA5 = The read access for DPM_CTL is correct.
     * |        |          |Others = The read access for DPM_CTL is incorrect.
     * |        |          |Write operation:
     * |        |          |0x5A = The write verify code, 0x5A, is needed to do a valid write to DPM_CTL.
     * |        |          |Others = Invalid write verify code.
     * @var DPM_T::STS
     * Offset: 0x04  Secure DPM Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BUSY      |DPM Busy Flag (Read Only)
     * |        |          |This bit indicates the DPM is busy.
     * |        |          |0 = DPM is not busy and writing to any register is accepted.
     * |        |          |1 = DPM is busy and other bits in DPM_STS register are not valid and writing to any register is ignored.
     * |[1]     |INT       |DPM Interrupt Flag (Read Only)
     * |        |          |This bit indicates the interrupt is triggered.
     * |        |          |0 = Interrupt is not enabled or no password comparison flag is set.
     * |        |          |1 = Interrupt is enabled and PWCERR flag in either DPM_STS or DPM_NSSTS register is not cleared.
     * |        |          |Note: This bit is cleared automatically when PWCERR flag in both DPM_STS and DPM_NSSTS are 0.
     * |[4]     |PWCERR    |Secure DPM Password Compared Error Flag
     * |        |          |This bit indicates the result of Secure DPM password comparison.
     * |        |          |When read:
     * |        |          |0 = The result of Secure DPM password is correct.
     * |        |          |1 = The result of Secure DPM password is incorrect.
     * |        |          |Note: This flag is write-one-clear.
     * |[5]     |PWUOK     |Secure DPM Password Updated Flag
     * |        |          |This bit indicates Secure DPM password has been updated successfully.
     * |        |          |When read:
     * |        |          |0 = No successful updating process has happened.
     * |        |          |1 = There is at least one successful updating process since last clearing of this bit.
     * |        |          |Note: This flag is write-one-clear.
     * |[6]     |PWFMAX    |Secure DPM Password Fail Times Maximum Reached Flag (Read Only)
     * |        |          |This bit indicates if the fail times of comparing Secure DPM password reached max times.
     * |        |          |0 = Max time has not reached and Secure DPM password comparison can be triggered.
     * |        |          |1 = Max time reached and Secure DPM password comparison cannot be processed anymore.
     * |[10:8]  |PWUCNT    |Secure DPM Password Updated Times (Read Only)
     * |        |          |This bit indicates how many times of secure password has been updated.
     * |        |          |The max value is 7. If PWUCNT reached the max value, Secure DPM password cannot be updated anymore.
     * |[16]    |DBGDIS    |Secure Debug Disable Flag (Read Only)
     * |        |          |This bit indicates the current value of DBGDISS bit.
     * |        |          |{PWOK, LOCK, DBGDIS} bits define the current state of DPM.
     * |        |          |x00 = DEFAULT state.
     * |        |          |x1x = LOCKED state.
     * |        |          |001 = CLOSE state.
     * |        |          |101 = OPEN state.
     * |        |          |Others = Unknown.
     * |[17]    |LOCK      |Secure Debug Lock Flag (Read Only)
     * |        |          |This bit indicates the current value of LOCKS bit.
     * |[18]    |PWOK      |Secure Password OK Flag (Read Only)
     * |        |          |This bit indicates the Secure DPM password has been checked and is correct.
     * |        |          |0 = The Secure DPM password has not been checked pass, yet.
     * |        |          |1 = The Secure DPM password has been checked pass since last cold reset.
     * @var DPM_T::SPW[4]
     * Offset: 0x10  Secure DPM Password 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PW        |Password
     * |        |          |Write password[31:0] to this register to update or compare Secure DPM password
     * |        |          |It is write-only and always read as 0xFFFFFFFF.
     * Offset: 0x14  Secure DPM Password 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PW        |Password
     * |        |          |Write password[63:32] to this register to update or compare Secure DPM password
     * |        |          |It is write-only and always read as 0xFFFFFFFF.
     * Offset: 0x18  Secure DPM Password 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PW        |Password
     * |        |          |Write password[95:64] to this register to update or compare Secure DPM password
     * |        |          |It is write-only and always read as 0xFFFFFFFF.
     * Offset: 0x1C  Secure DPM Password 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PW        |Password
     * |        |          |Write password[127:96] to this register to update or compare Secure DPM password
     * |        |          |It is write-only and always read as 0xFFFFFFFF.
     * @var DPM_T::NSCTL
     * Offset: 0x50  Non-secure DPM Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DBGDIS    |Set Non-secure DPM Debug Disable Bit
     * |        |          |When this bit is read as 0, it can be written to 1 to configure the Non-secure DPM DBGDIS bit (DBGDISNS).
     * |        |          |When written:
     * |        |          |0 = No operation.
     * |        |          |1 = Trigger the process to set DBGDISNS configuration bit.
     * |        |          |Note: This bit can be set to 1 but cannot be cleared to 0.
     * |[1]     |LOCK      |Set Non-secure DPM Debug Lock Bit
     * |        |          |When this bit is read as 0, it can be written to 1 to configure the Non-secure DPM LOCK bit (LOCKNS).
     * |        |          |When written:
     * |        |          |0 = No operation.
     * |        |          |1 = Trigger the process to set LOCKNS configuration bit.
     * |        |          |Note: This bit can be set to 1 but cannot be cleared to 0.
     * |[2]     |PWCMP     |Non-secure DPM Password Compare Bit
     * |        |          |Set to enter the process of compare Non-secure DPM password.
     * |        |          |0 = No operation.
     * |        |          |1 = Compare Non-secure DPM password.
     * |        |          |Note: This bit will be cleared after the comparison process is finished.
     * |[3]     |PWUPD     |Non-secure DPM Password Update Bit
     * |        |          |Set to enter the process of updating Non-secure DPM password.
     * |        |          |0 = No operation.
     * |        |          |1 = Update Non-secure DPM password.
     * |        |          |Note 1: This bit should be set with PWCMP equal to 0.
     * |        |          |Note 2: This bit will be cleared after the update process is finished.
     * |[12]    |DACCWDIS  |Debug Write Access Disable Bit
     * |        |          |This bit disables the ability of external debugger to set Non-secure DPM registers for debug authentication.
     * |        |          |0 = External debugger can set Non-secure DPM registers.
     * |        |          |1 = External debugger cannot set Non-secure DPM registers.
     * |[31:24] |RWVCODE   |Write Verify Code and Read Verify Code
     * |        |          |Read operation:
     * |        |          |0xA5 = The read access for DPM_NSCTL is correct.
     * |        |          |Others = The read access for DPM_NSCTL is incorrect.
     * |        |          |Write operation:
     * |        |          |0x5A = The write verify code, 0x5A, is needed to do a valid write to DPM_NSCTL.
     * |        |          |Others = Invalid write verify code.
     * @var DPM_T::NSSTS
     * Offset: 0x54  Non-secure DPM Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BUSY      |DPM Busy Flag (Read Only)
     * |        |          |This bit indicates the DPM is busy.
     * |        |          |0 = DPM is not busy and writing to any register is accepted.
     * |        |          |1 = DPM is busy and other bits in DPM_NSSTS register are not valid and writing to any register is ignored.
     * |[4]     |PWCERR    |Non-secure DPM Password Compared Error Flag
     * |        |          |This bit indicates the result of Non-secure DPM password comparison.
     * |        |          |0 = The result of Non-secure DPM password is correct.
     * |        |          |1 = The result of Non-secure DPM password is incorrect.
     * |        |          |Note: This flag is write-one-clear.
     * |[5]     |PWUOK     |Non-secure DPM Password Updated Flag
     * |        |          |This bit indicates Non-secure DPM password has been updated correctly.
     * |        |          |When read:
     * |        |          |0 = No successful updating process has happened.
     * |        |          |1 = There is at least one successful updating process since last clearing of this bit.
     * |        |          |Note: This flag is write-one-clear.
     * |[6]     |PWFMAX    |Non-secure DPM Password Fail Times Maximum Reached Flag (Read Only)
     * |        |          |This bit indicates if the fail times of comparing Non-secure DPM password reached max times.
     * |        |          |0 = Max time has not reached and Non-secure DPM password comparison can be triggered.
     * |        |          |1 = Max time reached and Non-secure DPM password comparison cannot be processed anymore.
     * |[10:8]  |PWUCNT    |Non-secure DPM Password Updated Times (Read Only)
     * |        |          |This bit indicates how many times of non-secure password has been updated.
     * |        |          |The max value is 7. If PWUCNT reached the max value, Non-secure DPM password cannot be updated anymore.
     * |[16]    |DBGDIS    |Non-secure Debug Disable Flag (Read Only)
     * |        |          |This bit indicates the current value of DBGDISNS bit.
     * |        |          |{PWOK, LOCK, DBGDIS} bits define the current state of DPM.
     * |        |          |x00 = DEFAULT state.
     * |        |          |x1x = LOCKED state.
     * |        |          |001 = CLOSE state.
     * |        |          |101 = OPEN state.
     * |        |          |Others = Unknown.
     * |[17]    |LOCK      |Non-secure Debug Lock Flag (Read Only)
     * |        |          |This bit indicates the current value of LOCKNS bit.
     * |[18]    |PWOK      |Non-secure Password OK Flag (Read Only)
     * |        |          |This bit indicates the Non-secure DPM password has been checked and is correct.
     * |        |          |0 = The Non-secure DPM password has not been checked pass, yet.
     * |        |          |1 = The Non-secure DPM password has been checked pass since last cold reset.
     * @var DPM_T::NSPW[4]
     * Offset: 0x60  Non-secure DPM Password 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PW        |Password
     * |        |          |Write password[31:0] to this register to update or compare Non-secure DPM password
     * |        |          |It is write-only and always read as 0xFFFFFFFF.
     * Offset: 0x64  Non-secure DPM Password 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PW        |Password
     * |        |          |Write password[63:32] to this register to update or compare Non-secure DPM password
     * |        |          |It is write-only and always read as 0xFFFFFFFF.
     * Offset: 0x68  Non-secure DPM Password 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PW        |Password
     * |        |          |Write password[95:64] to this register to update or compare Non-secure DPM password
     * |        |          |It is write-only and always read as 0xFFFFFFFF.
     * Offset: 0x6C  Non-secure DPM Password 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PW        |Password
     * |        |          |Write password[127:96] to this register to update or compare Non-secure DPM password
     * |        |          |It is write-only and always read as 0xFFFFFFFF.
     * @var DPM_T::MISC
     * Offset: 0xF0  DPM Miscellaneous Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * @var DPM_T::ID
     * Offset: 0xFC  DPM Identification Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     */
    __IO uint32_t CTL;                   /*!< [0x0000] Secure DPM Control Register                                      */
    __IO uint32_t STS;                   /*!< [0x0004] Secure DPM Status Register                                       */
    __I  uint32_t RESERVE0[2];
    __O  uint32_t SPW[4];                /*!< [0x0010-0x001c] Secure DPM Password 0-3                                   */
    __I  uint32_t RESERVE1[12];
    __IO uint32_t NSCTL;                 /*!< [0x0050] Non-secure DPM Control Register                                  */
    __IO uint32_t NSSTS;                 /*!< [0x0054] Non-secure DPM Status Register                                   */
    __I  uint32_t RESERVE2[2];
    __O  uint32_t NSPW[4];               /*!< [0x0060-0x006c] Non-secure DPM Password 0-3                               */
    __I  uint32_t RESERVE3[32];
    __IO uint32_t MISC;                  /*!< [0x00f0] DPM Miscellaneous Register                                       */
    __I  uint32_t RESERVE4[2];
    __I  uint32_t ID;                    /*!< [0x00fc] DPM Identification Register                                      */
} DPM_T;

/**
    @addtogroup DPM_CONST DPM Bit Field Definition
    Constant Definitions for DPM Controller
    @{
*/

#define DPM_CTL_DBGDIS_Pos               (0)                                               /*!< DPM_T::CTL: DBGDIS Position            */
#define DPM_CTL_DBGDIS_Msk               (0x1ul << DPM_CTL_DBGDIS_Pos)                     /*!< DPM_T::CTL: DBGDIS Mask                */

#define DPM_CTL_LOCK_Pos                 (1)                                               /*!< DPM_T::CTL: LOCK Position              */
#define DPM_CTL_LOCK_Msk                 (0x1ul << DPM_CTL_LOCK_Pos)                       /*!< DPM_T::CTL: LOCK Mask                  */

#define DPM_CTL_PWCMP_Pos                (2)                                               /*!< DPM_T::CTL: PWCMP Position             */
#define DPM_CTL_PWCMP_Msk                (0x1ul << DPM_CTL_PWCMP_Pos)                      /*!< DPM_T::CTL: PWCMP Mask                 */

#define DPM_CTL_PWUPD_Pos                (3)                                               /*!< DPM_T::CTL: PWUPD Position             */
#define DPM_CTL_PWUPD_Msk                (0x1ul << DPM_CTL_PWUPD_Pos)                      /*!< DPM_T::CTL: PWUPD Mask                 */

#define DPM_CTL_INTEN_Pos                (8)                                               /*!< DPM_T::CTL: INTEN Position             */
#define DPM_CTL_INTEN_Msk                (0x1ul << DPM_CTL_INTEN_Pos)                      /*!< DPM_T::CTL: INTEN Mask                 */

#define DPM_CTL_DACCWDIS_Pos             (12)                                              /*!< DPM_T::CTL: DACCWDIS Position          */
#define DPM_CTL_DACCWDIS_Msk             (0x1ul << DPM_CTL_DACCWDIS_Pos)                   /*!< DPM_T::CTL: DACCWDIS Mask              */

#define DPM_CTL_DACCDIS_Pos              (13)                                              /*!< DPM_T::CTL: DACCDIS Position           */
#define DPM_CTL_DACCDIS_Msk              (0x1ul << DPM_CTL_DACCDIS_Pos)                    /*!< DPM_T::CTL: DACCDIS Mask               */

#define DPM_CTL_RWVCODE_Pos              (24)                                              /*!< DPM_T::CTL: RWVCODE Position           */
#define DPM_CTL_RWVCODE_Msk              (0xfful << DPM_CTL_RWVCODE_Pos)                   /*!< DPM_T::CTL: RWVCODE Mask               */

#define DPM_STS_BUSY_Pos                 (0)                                               /*!< DPM_T::STS: BUSY Position              */
#define DPM_STS_BUSY_Msk                 (0x1ul << DPM_STS_BUSY_Pos)                       /*!< DPM_T::STS: BUSY Mask                  */

#define DPM_STS_INT_Pos                  (1)                                               /*!< DPM_T::STS: INT Position               */
#define DPM_STS_INT_Msk                  (0x1ul << DPM_STS_INT_Pos)                        /*!< DPM_T::STS: INT Mask                   */

#define DPM_STS_PWCERR_Pos               (4)                                               /*!< DPM_T::STS: PWCERR Position            */
#define DPM_STS_PWCERR_Msk               (0x1ul << DPM_STS_PWCERR_Pos)                     /*!< DPM_T::STS: PWCERR Mask                */

#define DPM_STS_PWUOK_Pos                (5)                                               /*!< DPM_T::STS: PWUOK Position             */
#define DPM_STS_PWUOK_Msk                (0x1ul << DPM_STS_PWUOK_Pos)                      /*!< DPM_T::STS: PWUOK Mask                 */

#define DPM_STS_PWFMAX_Pos               (6)                                               /*!< DPM_T::STS: PWFMAX Position            */
#define DPM_STS_PWFMAX_Msk               (0x1ul << DPM_STS_PWFMAX_Pos)                     /*!< DPM_T::STS: PWFMAX Mask                */

#define DPM_STS_PWUCNT_Pos               (8)                                               /*!< DPM_T::STS: PWUCNT Position            */
#define DPM_STS_PWUCNT_Msk               (0x7ul << DPM_STS_PWUCNT_Pos)                     /*!< DPM_T::STS: PWUCNT Mask                */

#define DPM_STS_DBGDIS_Pos               (16)                                              /*!< DPM_T::STS: DBGDIS Position            */
#define DPM_STS_DBGDIS_Msk               (0x1ul << DPM_STS_DBGDIS_Pos)                     /*!< DPM_T::STS: DBGDIS Mask                */

#define DPM_STS_LOCK_Pos                 (17)                                              /*!< DPM_T::STS: LOCK Position              */
#define DPM_STS_LOCK_Msk                 (0x1ul << DPM_STS_LOCK_Pos)                       /*!< DPM_T::STS: LOCK Mask                  */

#define DPM_STS_PWOK_Pos                 (18)                                              /*!< DPM_T::STS: PWOK Position              */
#define DPM_STS_PWOK_Msk                 (0x1ul << DPM_STS_PWOK_Pos)                       /*!< DPM_T::STS: PWOK Mask                  */

#define DPM_SPW0_PW_Pos                  (0)                                               /*!< DPM_T::SPW0: PW Position               */
#define DPM_SPW0_PW_Msk                  (0xfffffffful << DPM_SPW0_PW_Pos)                 /*!< DPM_T::SPW0: PW Mask                   */

#define DPM_SPW1_PW_Pos                  (0)                                               /*!< DPM_T::SPW1: PW Position               */
#define DPM_SPW1_PW_Msk                  (0xfffffffful << DPM_SPW1_PW_Pos)                 /*!< DPM_T::SPW1: PW Mask                   */

#define DPM_SPW2_PW_Pos                  (0)                                               /*!< DPM_T::SPW2: PW Position               */
#define DPM_SPW2_PW_Msk                  (0xfffffffful << DPM_SPW2_PW_Pos)                 /*!< DPM_T::SPW2: PW Mask                   */

#define DPM_SPW3_PW_Pos                  (0)                                               /*!< DPM_T::SPW3: PW Position               */
#define DPM_SPW3_PW_Msk                  (0xfffffffful << DPM_SPW3_PW_Pos)                 /*!< DPM_T::SPW3: PW Mask                   */

#define DPM_NSCTL_DBGDIS_Pos             (0)                                               /*!< DPM_T::NSCTL: DBGDIS Position          */
#define DPM_NSCTL_DBGDIS_Msk             (0x1ul << DPM_NSCTL_DBGDIS_Pos)                   /*!< DPM_T::NSCTL: DBGDIS Mask              */

#define DPM_NSCTL_LOCK_Pos               (1)                                               /*!< DPM_T::NSCTL: LOCK Position            */
#define DPM_NSCTL_LOCK_Msk               (0x1ul << DPM_NSCTL_LOCK_Pos)                     /*!< DPM_T::NSCTL: LOCK Mask                */

#define DPM_NSCTL_PWCMP_Pos              (2)                                               /*!< DPM_T::NSCTL: PWCMP Position           */
#define DPM_NSCTL_PWCMP_Msk              (0x1ul << DPM_NSCTL_PWCMP_Pos)                    /*!< DPM_T::NSCTL: PWCMP Mask               */

#define DPM_NSCTL_PWUPD_Pos              (3)                                               /*!< DPM_T::NSCTL: PWUPD Position           */
#define DPM_NSCTL_PWUPD_Msk              (0x1ul << DPM_NSCTL_PWUPD_Pos)                    /*!< DPM_T::NSCTL: PWUPD Mask               */

#define DPM_NSCTL_DACCWDIS_Pos           (12)                                              /*!< DPM_T::NSCTL: DACCWDIS Position        */
#define DPM_NSCTL_DACCWDIS_Msk           (0x1ul << DPM_NSCTL_DACCWDIS_Pos)                 /*!< DPM_T::NSCTL: DACCWDIS Mask            */

#define DPM_NSCTL_RWVCODE_Pos            (24)                                              /*!< DPM_T::NSCTL: RWVCODE Position         */
#define DPM_NSCTL_RWVCODE_Msk            (0xfful << DPM_NSCTL_RWVCODE_Pos)                 /*!< DPM_T::NSCTL: RWVCODE Mask             */

#define DPM_NSSTS_BUSY_Pos               (0)                                               /*!< DPM_T::NSSTS: BUSY Position            */
#define DPM_NSSTS_BUSY_Msk               (0x1ul << DPM_NSSTS_BUSY_Pos)                     /*!< DPM_T::NSSTS: BUSY Mask                */

#define DPM_NSSTS_PWCERR_Pos             (4)                                               /*!< DPM_T::NSSTS: PWCERR Position          */
#define DPM_NSSTS_PWCERR_Msk             (0x1ul << DPM_NSSTS_PWCERR_Pos)                   /*!< DPM_T::NSSTS: PWCERR Mask              */

#define DPM_NSSTS_PWUOK_Pos              (5)                                               /*!< DPM_T::NSSTS: PWUOK Position           */
#define DPM_NSSTS_PWUOK_Msk              (0x1ul << DPM_NSSTS_PWUOK_Pos)                    /*!< DPM_T::NSSTS: PWUOK Mask               */

#define DPM_NSSTS_PWFMAX_Pos             (6)                                               /*!< DPM_T::NSSTS: PWFMAX Position          */
#define DPM_NSSTS_PWFMAX_Msk             (0x1ul << DPM_NSSTS_PWFMAX_Pos)                   /*!< DPM_T::NSSTS: PWFMAX Mask              */

#define DPM_NSSTS_PWUCNT_Pos             (8)                                               /*!< DPM_T::NSSTS: PWUCNT Position          */
#define DPM_NSSTS_PWUCNT_Msk             (0x7ul << DPM_NSSTS_PWUCNT_Pos)                   /*!< DPM_T::NSSTS: PWUCNT Mask              */

#define DPM_NSSTS_DBGDIS_Pos             (16)                                              /*!< DPM_T::NSSTS: DBGDIS Position          */
#define DPM_NSSTS_DBGDIS_Msk             (0x1ul << DPM_NSSTS_DBGDIS_Pos)                   /*!< DPM_T::NSSTS: DBGDIS Mask              */

#define DPM_NSSTS_LOCK_Pos               (17)                                              /*!< DPM_T::NSSTS: LOCK Position            */
#define DPM_NSSTS_LOCK_Msk               (0x1ul << DPM_NSSTS_LOCK_Pos)                     /*!< DPM_T::NSSTS: LOCK Mask                */

#define DPM_NSSTS_PWOK_Pos               (18)                                              /*!< DPM_T::NSSTS: PWOK Position            */
#define DPM_NSSTS_PWOK_Msk               (0x1ul << DPM_NSSTS_PWOK_Pos)                     /*!< DPM_T::NSSTS: PWOK Mask                */

#define DPM_NSPW0_PW_Pos                 (0)                                               /*!< DPM_T::NSPW0: PW Position              */
#define DPM_NSPW0_PW_Msk                 (0xfffffffful << DPM_NSPW0_PW_Pos)                /*!< DPM_T::NSPW0: PW Mask                  */

#define DPM_NSPW1_PW_Pos                 (0)                                               /*!< DPM_T::NSPW1: PW Position              */
#define DPM_NSPW1_PW_Msk                 (0xfffffffful << DPM_NSPW1_PW_Pos)                /*!< DPM_T::NSPW1: PW Mask                  */

#define DPM_NSPW2_PW_Pos                 (0)                                               /*!< DPM_T::NSPW2: PW Position              */
#define DPM_NSPW2_PW_Msk                 (0xfffffffful << DPM_NSPW2_PW_Pos)                /*!< DPM_T::NSPW2: PW Mask                  */

#define DPM_NSPW3_PW_Pos                 (0)                                               /*!< DPM_T::NSPW3: PW Position              */
#define DPM_NSPW3_PW_Msk                 (0xfffffffful << DPM_NSPW3_PW_Pos)                /*!< DPM_T::NSPW3: PW Mask                  */

/**@}*/ /* DPM_CONST */
/**@}*/ /* end of DPM register group */

/*---------------------- Firmware Version Counter -------------------------*/
/**
    @addtogroup FVC Firmware Version Counter (FVC)
    Memory Mapped Structure for FVC Controller
    @{
*/

typedef struct
{

    /**
     * @var FVC_T::CTL
     * Offset: 0x00  FVC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INIT      |FVC Init Bit
     * |        |          |Set to 1 to enable FVC
     * |        |          |This bit is writable when FVC is at Reset state.
     * |        |          |Note: After set to 1, this bit is cleared to 0 automatically when FVC is back to Reset state.
     * |[1]     |MONOEN    |Monotonic Enable Bit
     * |        |          |Set to 1 to enable the monotonic mechanism of FVC.
     * |        |          |Note: This bit can be set to1 but cannot be cleared to 0.
     * |[31:16] |WVCODE    |Verification Code
     * |        |          |When written, this field must be 0x7710
     * @var FVC_T::STS
     * Offset: 0x04  FVC Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BUSY      |FVC Busy Bit
     * |        |          |Indicates the FVC is at busy state.
     * |[1]     |RDY       |FVC Ready Bit
     * |        |          |Indicates the FVC is ready after the initial process.
     * @var FVC_T::NVC0
     * Offset: 0x10  Non-volatile Version Counter Control Register0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |FWVER     |Firmware Version
     * |        |          |Read: Indicates the current firmware version of NVC0.
     * |        |          |Write: Updates the firmware version of NVC0.
     * |        |          |The maximum value of this field is 63.
     * |[31:16] |WVCODE    |Verification Code
     * |        |          |When written, this field must be the current firmware version number
     * @var FVC_T::NVC1
     * Offset: 0x14  Non-volatile Version Counter Control Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |FWVER     |Firmware Version
     * |        |          |Read: Indicates the current firmware version of NVC1.
     * |        |          |Write: Updates the firmware version of NVC1.
     * |        |          |The maximum value of this field is 63.
     * |[31:16] |WVCODE    |Verification Code
     * |        |          |When written, this field must be the current firmware version number
     * @var FVC_T::NVC4
     * Offset: 0x20  Non-volatile Version Counter Control Register4
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |FWVER     |Firmware Version
     * |        |          |Read: Indicates the current firmware version of NVC4.
     * |        |          |Write: Updates the firmware version of NVC4.
     * |        |          |The maximum value of this field is 255.
     * |[31:16] |WVCODE    |Verification Code
     * |        |          |When written, this field must be the current firmware version number
     * @var FVC_T::NVC5
     * Offset: 0x24  Non-volatile Version Counter Control Register5
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |FWVER     |Firmware Version
     * |        |          |Read: Indicates the current firmware version of NVC5.
     * |        |          |Write: Updates the firmware version of NVC5.
     * |        |          |The maximum value of this field is 255.
     * |[31:16] |WVCODE    |Verification Code
     * |        |          |When written, this field must be the current firmware version number
     */
    __IO uint32_t CTL;                   /*!< [0x0000] FVC Control Register                                             */
    __I  uint32_t STS;                   /*!< [0x0004] FVC Status Register                                              */
    __I  uint32_t RESERVE0[2];
    union
    {
        __IO uint32_t NVC[6];                   /*!< [0x0010-0x0024] Non-volatile Version Counter Control Register Firmware. NVC[2], NVC[3] is reserved    */
        struct
        {
            __IO uint32_t NVC0;                  /*!< [0x0010] Non-volatile Version Counter Control Register0                   */
            __IO uint32_t NVC1;                  /*!< [0x0014] Non-volatile Version Counter Control Register1                   */
            __I  uint32_t RESERVE1[2];
            __IO uint32_t NVC4;                  /*!< [0x0020] Non-volatile Version Counter Control Register4                   */
            __IO uint32_t NVC5;                  /*!< [0x0024] Non-volatile Version Counter Control Register5                   */
        };
    };
} FVC_T;

/**
    @addtogroup FVC_CONST FVC Bit Field Definition
    Constant Definitions for FVC Controller
    @{
*/

#define FVC_CTL_INIT_Pos                 (0)                                               /*!< FVC_T::CTL: INIT Position              */
#define FVC_CTL_INIT_Msk                 (0x1ul << FVC_CTL_INIT_Pos)                       /*!< FVC_T::CTL: INIT Mask                  */

#define FVC_CTL_MONOEN_Pos               (1)                                               /*!< FVC_T::CTL: MONOEN Position            */
#define FVC_CTL_MONOEN_Msk               (0x1ul << FVC_CTL_MONOEN_Pos)                     /*!< FVC_T::CTL: MONOEN Mask                */

#define FVC_CTL_WVCODE_Pos               (16)                                              /*!< FVC_T::CTL: WVCODE Position            */
#define FVC_CTL_WVCODE_Msk               (0xfffful << FVC_CTL_WVCODE_Pos)                  /*!< FVC_T::CTL: WVCODE Mask                */

#define FVC_STS_BUSY_Pos                 (0)                                               /*!< FVC_T::STS: BUSY Position              */
#define FVC_STS_BUSY_Msk                 (0x1ul << FVC_STS_BUSY_Pos)                       /*!< FVC_T::STS: BUSY Mask                  */

#define FVC_STS_RDY_Pos                  (1)                                               /*!< FVC_T::STS: RDY Position               */
#define FVC_STS_RDY_Msk                  (0x1ul << FVC_STS_RDY_Pos)                        /*!< FVC_T::STS: RDY Mask                   */

#define FVC_NVC0_FWVER_Pos               (0)                                               /*!< FVC_T::NVC0: FWVER Position            */
#define FVC_NVC0_FWVER_Msk               (0xfffful << FVC_NVC0_FWVER_Pos)                  /*!< FVC_T::NVC0: FWVER Mask                */

#define FVC_NVC0_WVCODE_Pos              (16)                                              /*!< FVC_T::NVC0: WVCODE Position           */
#define FVC_NVC0_WVCODE_Msk              (0xfffful << FVC_NVC0_WVCODE_Pos)                 /*!< FVC_T::NVC0: WVCODE Mask               */

#define FVC_NVC1_FWVER_Pos               (0)                                               /*!< FVC_T::NVC1: FWVER Position            */
#define FVC_NVC1_FWVER_Msk               (0xfffful << FVC_NVC1_FWVER_Pos)                  /*!< FVC_T::NVC1: FWVER Mask                */

#define FVC_NVC1_WVCODE_Pos              (16)                                              /*!< FVC_T::NVC1: WVCODE Position           */
#define FVC_NVC1_WVCODE_Msk              (0xfffful << FVC_NVC1_WVCODE_Pos)                 /*!< FVC_T::NVC1: WVCODE Mask               */

#define FVC_NVC4_FWVER_Pos               (0)                                               /*!< FVC_T::NVC4: FWVER Position            */
#define FVC_NVC4_FWVER_Msk               (0xfffful << FVC_NVC4_FWVER_Pos)                  /*!< FVC_T::NVC4: FWVER Mask                */

#define FVC_NVC4_WVCODE_Pos              (16)                                              /*!< FVC_T::NVC4: WVCODE Position           */
#define FVC_NVC4_WVCODE_Msk              (0xfffful << FVC_NVC4_WVCODE_Pos)                 /*!< FVC_T::NVC4: WVCODE Mask               */

#define FVC_NVC5_FWVER_Pos               (0)                                               /*!< FVC_T::NVC5: FWVER Position            */
#define FVC_NVC5_FWVER_Msk               (0xfffful << FVC_NVC5_FWVER_Pos)                  /*!< FVC_T::NVC5: FWVER Mask                */

#define FVC_NVC5_WVCODE_Pos              (16)                                              /*!< FVC_T::NVC5: WVCODE Position           */
#define FVC_NVC5_WVCODE_Msk              (0xfffful << FVC_NVC5_WVCODE_Pos)                 /*!< FVC_T::NVC5: WVCODE Mask               */

/**@}*/ /* FVC_CONST */
/**@}*/ /* end of FVC register group */


/*---------------------- Product Life-cycle Manager -------------------------*/
/**
    @addtogroup PLM Product Life-cycle Manager (PLM)
    Memory Mapped Structure for PLM Controller
    @{
*/

typedef struct
{

    /**
     * @var PLM_T::CTL
     * Offset: 0x00  Product Life-cycle Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |STAGE     |Life-cycle Stage Update Bits
     * |        |          |Bits to update PLM stage. All bits can be set to one but cannot be cleared to 0.
     * |        |          |001 = progress to OEM stage.
     * |        |          |011 = progress to Deployed stage.
     * |        |          |111 = progress to RMA stage.
     * |        |          |Other value will be ignored.
     * |[31:16] |WVCODE    |Write Verify Code
     * |        |          |The code is 0x475A for a valid write to this register.
     * @var PLM_T::STS
     * Offset: 0x04  Product Life-cycle Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |STAGE     |Life-cycle Stage (Read Only)
     * |        |          |Indicates the current stage of PLM.
     * |        |          |000 = Vendor Stage.
     * |        |          |001 = OEM Stage.
     * |        |          |011 = Deployed Stage.
     * |        |          |111 = RMA Stage.
     * |        |          |Others = ERROR Stage.
     * |[8]     |DIRTY     |DIRTY Bit (Read Only)
     * |        |          |Indicate the life-cycle stage has been progressed after last cold-reset
     * |        |          |Value of STAGE bits is not Current stage of PLM
     * |        |          |It needs a cold reset to make it work.
     */
    __IO uint32_t CTL;                   /*!< [0x0000] Product Life-cycle Control Register                              */
    __I  uint32_t STS;                   /*!< [0x0004] Product Life-cycle Status Register                               */
} PLM_T;

/**
    @addtogroup PLM_CONST PLM Bit Field Definition
    Constant Definitions for PLM Controller
    @{
*/

#define PLM_CTL_STAGE_Pos                (0)                                               /*!< PLM_T::CTL: STAGE Position             */
#define PLM_CTL_STAGE_Msk                (0x7ul << PLM_CTL_STAGE_Pos)                      /*!< PLM_T::CTL: STAGE Mask                 */

#define PLM_CTL_WVCODE_Pos               (16)                                              /*!< PLM_T::CTL: WVCODE Position            */
#define PLM_CTL_WVCODE_Msk               (0xfffful << PLM_CTL_WVCODE_Pos)                  /*!< PLM_T::CTL: WVCODE Mask                */

#define PLM_STS_STAGE_Pos                (0)                                               /*!< PLM_T::STS: STAGE Position             */
#define PLM_STS_STAGE_Msk                (0x7ul << PLM_STS_STAGE_Pos)                      /*!< PLM_T::STS: STAGE Mask                 */

#define PLM_STS_DIRTY_Pos                (8)                                               /*!< PLM_T::STS: DIRTY Position             */
#define PLM_STS_DIRTY_Msk                (0x1ul << PLM_STS_DIRTY_Pos)                      /*!< PLM_T::STS: DIRTY Mask                 */

/**@}*/ /* PLM_CONST */
/**@}*/ /* end of PLM register group */


/*---------------------- Secure Configuration Unit -------------------------*/
/**
    @addtogroup SCU Secure Configuration Unit (SCU)
    Memory Mapped Structure for SCU Controller
    @{
*/

typedef struct
{


    /**
     * @var SCU_T::D0PNS0
     * Offset: 0x00  Domain 0 Peripheral Non-secure Attribution Set Register0 (0x4000_0000~0x4001_FFFF)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3]     |NPU       |Set NPU to Non-secure State
     * |        |          |0 = NPU is a secure module (default).
     * |        |          |1 = NPU is a non-secure module.
     * @var SCU_T::D0PNS2
     * Offset: 0x08  Domain 0 Peripheral Non-secure Attribution Set Register2 (0x4004_0000~0x4005_FFFF)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2]     |SPIM0     |Set SPIM0 to Non-secure State
     * |        |          |0 = SPIM0 is a secure module (default).
     * |        |          |1 = SPIM0 is a non-secure module.
     * @var SCU_T::D1PNS0
     * Offset: 0x20  Domain 1 Peripheral Non-secure Attribution Set Register0 (0x4020_0000~0x4021_FFFF)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PDMA0     |Set PDMA0 to Non-secure State
     * |        |          |0 = PDMA0 is a secure module (default).
     * |        |          |1 = PDMA0 is a non-secure module.
     * |[1]     |PDMA1     |Set PDMA1 to Non-secure State
     * |        |          |0 = PDMA1 is a secure module (default).
     * |        |          |1 = PDMA1 is a non-secure module.
     * |[2]     |USBH0     |Set USBH0 to Non-secure State
     * |        |          |0 = USBH0 is a secure module (default).
     * |        |          |1 = USBH0 is a non-secure module.
     * |[4]     |HSUSBH    |Set HSUSBH to Non-secure State
     * |        |          |0 = HSUSBH is a secure module (default).
     * |        |          |1 = HSUSBH is a non-secure module.
     * |[5]     |HSUSBD    |Set HSUSBD to Non-secure State
     * |        |          |0 = HSUSBD is a secure module (default).
     * |        |          |1 = HSUSBD is a non-secure module.
     * |[6]     |SDH0      |Set SDH0 to Non-secure State
     * |        |          |0 = SDH0 is a secure module (default).
     * |        |          |1 = SDH0 is a non-secure module.
     * |[7]     |SDH1      |Set SDH1 to Non-secure State
     * |        |          |0 = SDH1 is a secure module (default).
     * |        |          |1 = SDH1 is a non-secure module.
     * |[8]     |EMAC0     |Set EMAC0 to Non-secure State
     * |        |          |0 = EMAC0 is a secure module (default).
     * |        |          |1 = EMAC0 is a non-secure module.
     * |[10]    |CRYPTO    |Set CRYPTO to Non-secure State
     * |        |          |0 = CRYPTO is a secure module (default).
     * |        |          |1 = CRYPTO is a non-secure module.
     * |[11]    |CRC       |Set CRC to Non-secure State
     * |        |          |0 = CRC is a secure module (default).
     * |        |          |1 = CRC is a non-secure module.
     * @var SCU_T::D1PNS1
     * Offset: 0x24  Domain0 Peripheral Non-secure Attribution Set Register1 (0x4022_0000~0x4023_FFFF)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |KDF       |Set KDF to Non-secure State
     * |        |          |0 = KDF is a secure module (default).
     * |        |          |1 = KDF is a non-secure module.
     * |[2]     |CANFD0    |Set CANFD0 to Non-secure State
     * |        |          |0 = CANFD0 is a secure module (default).
     * |        |          |1 = CANFD0 is a non-secure module.
     * |[4]     |CANFD1    |Set CANFD1 to Non-secure State
     * |        |          |0 = CANFD1 is a secure module (default).
     * |        |          |1 = CANFD1 is a non-secure module.
     * |[16]    |EBI       |Set EBI to Non-secure State
     * |        |          |0 = EBI is a secure module (default).
     * |        |          |1 = EBI is a non-secure module.
     * @var SCU_T::D1PNS2
     * Offset: 0x28  Domain 1 Peripheral Non-secure Attribution Set Register2 (0x4024_0000~0x4025_FFFF)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDT0     |Set WWDT0 to Non-secure State
     * |        |          |0 = WWDT0 is a secure module (default).
     * |        |          |1 = WWDT0 is a non-secure module.
     * |[1]     |EADC0     |Set EADC0 to Non-secure State
     * |        |          |0 = EADC0 is a secure module (default).
     * |        |          |1 = EADC0 is a non-secure module.
     * |[2]     |EPWM0     |Set EPWM0 to Non-secure State
     * |        |          |0 = EPWM0 is a secure module (default).
     * |        |          |1 = EPWM0 is a non-secure module.
     * |[3]     |BPWM0     |Set BPWM0 to Non-secure State
     * |        |          |0 = BPWM0 is a secure module (default).
     * |        |          |1 = BPWM0 is a non-secure module.
     * |[4]     |EQEI0     |Set EQEI0 to Non-secure State
     * |        |          |0 = EQEI0 is a secure module (default).
     * |        |          |1 = EQEI0 is a non-secure module.
     * |[5]     |EQEI2     |Set EQEI2 to Non-secure State
     * |        |          |0 = EQEI2 is a secure module (default).
     * |        |          |1 = EQEI2 is a non-secure module.
     * |[6]     |ECAP0     |Set ECAP0 to Non-secure State
     * |        |          |0 = ECAP0 is a secure module (default).
     * |        |          |1 = ECAP0 is a non-secure module.
     * |[7]     |ECAP2     |Set ECAP2 to Non-secure State
     * |        |          |0 = ECAP2 is a secure module (default).
     * |        |          |1 = ECAP2 is a non-secure module.
     * |[8]     |I2C0      |Set I2C0 to Non-secure State
     * |        |          |0 = I2C0 is a secure module (default).
     * |        |          |1 = I2C0 is a non-secure module.
     * |[9]     |I2C2      |Set I2C2 to Non-secure State
     * |        |          |0 = I2C2 is a secure module (default).
     * |        |          |1 = I2C2 is a non-secure module.
     * |[10]    |QSPI0     |Set QSPI0 to Non-secure State
     * |        |          |0 = QSPI0 is a secure module (default).
     * |        |          |1 = QSPI0 is a non-secure module.
     * |[11]    |SPI0      |Set SPI0 to Non-secure State
     * |        |          |0 = SPI0 is a secure module (default).
     * |        |          |1 = SPI0 is a non-secure module.
     * |[12]    |SPI2      |Set SPI2 to Non-secure State
     * |        |          |0 = SPI2 is a secure module (default).
     * |        |          |1 = SPI2 is a non-secure module.
     * |[13]    |UART0     |Set UART0 to Non-secure State
     * |        |          |0 = UART0 is a secure module (default).
     * |        |          |1 = UART0 is a non-secure module.
     * |[14]    |UART2     |Set UART2 to Non-secure State
     * |        |          |0 = UART2 is a secure module (default).
     * |        |          |1 = UART2 is a non-secure module.
     * |[15]    |UART4     |Set UART4 to Non-secure State
     * |        |          |0 = UART4 is a secure module (default).
     * |        |          |1 = UART4 is a non-secure module.
     * |[16]    |UART6     |Set UART6 to Non-secure State
     * |        |          |0 = UART6 is a secure module (default).
     * |        |          |1 = UART6 is a non-secure module.
     * |[17]    |UART8     |Set UART8 to Non-secure State
     * |        |          |0 = UART8 is a secure module (default).
     * |        |          |1 = UART8 is a non-secure module.
     * |[18]    |USCI0     |Set USCI0 to Non-secure State
     * |        |          |0 = USCI0 is a secure module (default).
     * |        |          |1 = USCI0 is a non-secure module.
     * |[19]    |SC0       |Set SC0 to Non-secure State
     * |        |          |0 = SC0 is a secure module (default).
     * |        |          |1 = SC0 is a non-secure module.
     * |[20]    |SC2       |Set SC2 to Non-secure State
     * |        |          |0 = SC2 is a secure module (default).
     * |        |          |1 = SC2 is a non-secure module.
     * |[21]    |PSIO      |Set PSIO to Non-secure State
     * |        |          |0 = PSIO is a secure module (default).
     * |        |          |1 = PSIO is a non-secure module.
     * |[22]    |TMR01     |Set TMR01 to Non-secure State
     * |        |          |0 = TMR01 is a secure module (default).
     * |        |          |1 = TMR01 is a non-secure module.
     * |[23]    |DAC01     |Set DAC01 to Non-secure State
     * |        |          |0 = DAC01 is a secure module (default).
     * |        |          |1 = DAC01 is a non-secure module.
     * |[25]    |HSOTG     |Set HSOTG to Non-secure State
     * |        |          |0 = HSOTG is a secure module (default).
     * |        |          |1 = HSOTG is a non-secure module.
     * |[26]    |I2S0      |Set I2S0 to Non-secure State
     * |        |          |0 = I2S0 is a secure module (default).
     * |        |          |1 = I2S0 is a non-secure module.
     * |[27]    |ACMP01    |Set ACMP01 to Non-secure State
     * |        |          |0 = ACMP01 is a secure module (default).
     * |        |          |1 = ACMP01 is a non-secure module.
     * |[28]    |USBD      |Set USBD to Non-secure State
     * |        |          |0 = USBD is a secure module (default).
     * |        |          |1 = USBD is a non-secure module.
     * @var SCU_T::D1PNS4
     * Offset: 0x30  Domain 1 Peripheral Non-secure Attribution Set Register4 (0x4028_0000~0x4029_FFFF)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDT1     |Set WWDT1 to Non-secure State
     * |        |          |0 = WWDT1 is a secure module (default).
     * |        |          |1 = WWDT1 is a non-secure module.
     * |[2]     |EPWM1     |Set EPWM1 to Non-secure State
     * |        |          |0 = EPWM1 is a secure module (default).
     * |        |          |1 = EPWM1 is a non-secure module.
     * |[3]     |BPWM1     |Set BPWM1 to Non-secure State
     * |        |          |0 = BPWM1 is a secure module (default).
     * |        |          |1 = BPWM1 is a non-secure module.
     * |[4]     |EQEI1     |Set EQEI1 to Non-secure State
     * |        |          |0 = EQEI1 is a secure module (default).
     * |        |          |1 = EQEI1 is a non-secure module.
     * |[5]     |EQEI3     |Set EQEI3 to Non-secure State
     * |        |          |0 = EQEI3 is a secure module (default).
     * |        |          |1 = EQEI3 is a non-secure module.
     * |[6]     |ECAP1     |Set ECAP1 to Non-secure State
     * |        |          |0 = ECAP1 is a secure module (default).
     * |        |          |1 = ECAP1 is a non-secure module.
     * |[7]     |ECAP3     |Set ECAP3 to Non-secure State
     * |        |          |0 = ECAP3 is a secure module (default).
     * |        |          |1 = ECAP3 is a non-secure module.
     * |[8]     |I2C1      |Set I2C1 to Non-secure State
     * |        |          |0 = I2C1 is a secure module (default).
     * |        |          |1 = I2C1 is a non-secure module.
     * |[9]     |I2C3      |Set I2C3 to Non-secure State
     * |        |          |0 = I2C3 is a secure module (default).
     * |        |          |1 = I2C3 is a non-secure module.
     * |[10]    |QSPI1     |Set QSPI1 to Non-secure State
     * |        |          |0 = QSPI1 is a secure module (default).
     * |        |          |1 = QSPI1 is a non-secure module.
     * |[11]    |SPI1      |Set SPI1 to Non-secure State
     * |        |          |0 = SPI1 is a secure module (default).
     * |        |          |1 = SPI1 is a non-secure module.
     * |[12]    |SPI3      |Set SPI3 to Non-secure State
     * |        |          |0 = SPI3 is a secure module (default).
     * |        |          |1 = SPI3 is a non-secure module.
     * |[13]    |UART1     |Set UART1 to Non-secure State
     * |        |          |0 = UART1 is a secure module (default).
     * |        |          |1 = UART1 is a non-secure module.
     * |[14]    |UART3     |Set UART3 to Non-secure State
     * |        |          |0 = UART3 is a secure module (default).
     * |        |          |1 = UART3 is a non-secure module.
     * |[15]    |UART5     |Set UART5 to Non-secure State
     * |        |          |0 = UART5 is a secure module (default).
     * |        |          |1 = UART5 is a non-secure module.
     * |[16]    |UART7     |Set UART7 to Non-secure State
     * |        |          |0 = UART7 is a secure module (default).
     * |        |          |1 = UART7 is a non-secure module.
     * |[17]    |UART9     |Set UART9 to Non-secure State
     * |        |          |0 = UART9 is a secure module (default).
     * |        |          |1 = UART9 is a non-secure module.
     * |[18]    |SC1       |Set SC1 to Non-secure State
     * |        |          |0 = SC1 is a secure module (default).
     * |        |          |1 = SC1 is a non-secure module.
     * |[19]    |OTG       |Set OTG to Non-secure State
     * |        |          |0 = OTG is a secure module (default).
     * |        |          |1 = OTG is a non-secure module.
     * |[20]    |KPI       |Set KPI to Non-secure State
     * |        |          |0 = KPI is a secure module (default).
     * |        |          |1 = KPI is a non-secure module.
     * |[21]    |TMR23     |Set TMR23 to Non-secure State
     * |        |          |0 = TMR23 is a secure module (default).
     * |        |          |1 = TMR23 is a non-secure module.
     * |[22]    |TRNG      |Set TRNG to Non-secure State
     * |        |          |0 = TRNG is a secure module (default).
     * |        |          |1 = TRNG is a non-secure module.
     * |[23]    |RTC       |Set RTC to Non-secure State
     * |        |          |0 = RTC is a secure module (default).
     * |        |          |1 = RTC is a non-secure module.
     * |[24]    |I2S1      |Set I2S1 to Non-secure State
     * |        |          |0 = I2S1 is a secure module (default).
     * |        |          |1 = I2S1 is a non-secure module.
     * |[25]    |ACMP23    |Set ACMP23 to Non-secure State
     * |        |          |0 = ACMP23 is a secure module (default).
     * |        |          |1 = ACMP23 is a non-secure module.
     * |[26]    |I3C0      |Set I3C0 to Non-secure State
     * |        |          |0 = I3C0 is a secure module (default).
     * |        |          |1 = I3C0 is a non-secure module.
     * |[27]    |UTCPD     |Set UTCPD to Non-secure State
     * |        |          |0 = UTCPD is a secure module (default).
     * |        |          |1 = UTCPD is a non-secure module.
     * @var SCU_T::D2PNS0
     * Offset: 0x40  Domain 2 Peripheral Non-secure Attribution Set Register0 (0x4040_0000~0x4041_FFFF)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPPDMA    |Set LPPDMA to Non-secure State
     * |        |          |0 = LPPDMA is a secure module (default).
     * |        |          |1 = LPPDMA is a non-secure module.
     * |[1]     |CCAP      |Set CCAP to Non-secure State
     * |        |          |0 = CCAP is a secure module (default).
     * |        |          |1 = CCAP is a non-secure module.
     * |[3]     |LPGPIO    |Set LPGPIO to Non-secure State
     * |        |          |0 = LPGPIO is a secure module (default).
     * |        |          |1 = LPGPIO is a non-secure module.
     * @var SCU_T::D2PNS2
     * Offset: 0x48  Domain 2 Peripheral Non-secure Attribution Set Register2 (0x4044_0000~0x4045_FFFF)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPTMR01   |Set LPTMR01 to Non-secure State
     * |        |          |0 = LPTMR01 is a secure module (default).
     * |        |          |1 = LPTMR01 is a non-secure module.
     * |[1]     |TTMR01    |Set TTMR01 to Non-secure State
     * |        |          |0 = TTMR01 is a secure module (default).
     * |        |          |1 = TTMR01 is a non-secure module.
     * |[2]     |LPADC0    |Set LPADC0 to Non-secure State
     * |        |          |0 = LPADC0 is a secure module (default).
     * |        |          |1 = LPADC0 is a non-secure module.
     * |[3]     |LPI2C0    |Set LPI2C0 to Non-secure State
     * |        |          |0 = LPI2C0 is a secure module (default).
     * |        |          |1 = LPI2C0 is a non-secure module.
     * |[4]     |LPSPI0    |Set LPSPI0 to Non-secure State
     * |        |          |0 = LPSPI0 is a secure module (default).
     * |        |          |1 = LPSPI0 is a non-secure module.
     * |[5]     |DMIC0     |Set DMIC0 to Non-secure State
     * |        |          |0 = DMIC0 is a secure module (default).
     * |        |          |1 = DMIC0 is a non-secure module.
     * |[6]     |LPUART0   |Set LPUART0 to Non-secure State
     * |        |          |0 = LPUART0 is a secure module (default).
     * |        |          |1 = LPUART0 is a non-secure module.
     * |[9]     |AWF       |Set AWF to Non-secure State
     * |        |          |0 = AWF is a secure module (default).
     * |        |          |1 = AWF is a non-secure module.
     * @var SCU_T::FNSADDR
     * Offset: 0x60  Flash Non-secure Boundary Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |FNSADDR   |Flash Non-secure Boundary Address
     * |        |          |Indicate the base address of Non-secure region set in user configuration
     * |        |          |Refer to FMC section for more details.
     * @var SCU_T::EINTNS
     * Offset: 0x64  EINT Non-secure Attribution Set Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EINT0     |Set EINT0 to Non-secure State
     * |        |          |0 = EINT0 is a secure module (default).
     * |        |          |1 = EINT0 is a non-secure module.
     * |[1]     |EINT1     |Set EINT1 to Non-secure State
     * |        |          |0 = EINT1 is a secure module (default).
     * |        |          |1 = EINT1 is a non-secure module.
     * |[2]     |EINT2     |Set EINT2 to Non-secure State
     * |        |          |0 = EINT2 is a secure module (default).
     * |        |          |1 = EINT2 is a non-secure module.
     * |[3]     |EINT3     |Set EINT3 to Non-secure State
     * |        |          |0 = EINT3 is a secure module (default).
     * |        |          |1 = EINT3 is a non-secure module.
     * |[4]     |EINT4     |Set EINT4 to Non-secure State
     * |        |          |0 = EINT4 is a secure module (default).
     * |        |          |1 = EINT4 is a non-secure module.
     * |[5]     |EINT5     |Set EINT5 to Non-secure State
     * |        |          |0 = EINT5 is a secure module (default).
     * |        |          |1 = EINT5 is a non-secure module.
     * |[6]     |EINT6     |Set EINT6 to Non-secure State
     * |        |          |0 = EINT6 is a secure module (default).
     * |        |          |1 = EINT6 is a non-secure module.
     * |[7]     |EINT7     |Set EINT7 to Non-secure State
     * |        |          |0 = EINT7 is a secure module (default).
     * |        |          |1 = EINT7 is a non-secure module.
     * @var SCU_T::IONS[10]
     * Offset: 0x70  I/O Non-secure Attribution Set Register0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PA        |Set GPIO Port A to Non-secure State
     * |        |          |Write 1 to set PA to non-secure state. Each bit configures one pin in GPIO port A.
     * |        |          |0 = GPIO port A is secure (default).
     * |        |          |1 = GPIO port A is non-secure.
     * Offset: 0x74  I/O Non-secure Attribution Set Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PB        |Set GPIO Port B to Non-secure State
     * |        |          |Write 1 to set PB to non-secure state. Each bit configures one pin in GPIO port B.
     * |        |          |0 = GPIO port B is secure (default).
     * |        |          |1 = GPIO port B is non-secure.
     * Offset: 0x78  I/O Non-secure Attribution Set Register2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PC        |Set GPIO Port C to Non-secure State
     * |        |          |Write 1 to set PC to non-secure state. Each bit configures one pin in GPIO port C.
     * |        |          |0 = GPIO port C is secure (default).
     * |        |          |1 = GPIO port C is non-secure.
     * Offset: 0x7C  I/O Non-secure Attribution Set Register3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PD        |Set GPIO Port D to Non-secure State
     * |        |          |Write 1 to set PD to non-secure state. Each bit configures one pin in GPIO port D.
     * |        |          |0 = GPIO port D is secure (default).
     * |        |          |1 = GPIO port D is non-secure.
     * Offset: 0x80  I/O Non-secure Attribution Set Register4
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PE        |Set GPIO Port E to Non-secure State
     * |        |          |Write 1 to set PE to non-secure state. Each bit configures one pin in GPIO port E.
     * |        |          |0 = GPIO port E is secure (default).
     * |        |          |1 = GPIO port E is non-secure.
     * Offset: 0x84  I/O Non-secure Attribution Set Register5
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PF        |Set GPIO Port F to Non-secure State
     * |        |          |Write 1 to set PF to non-secure state. Each bit configures one pin in GPIO port F.
     * |        |          |0 = GPIO port F is secure (default).
     * |        |          |1 = GPIO port F is non-secure.
     * Offset: 0x88  I/O Non-secure Attribution Set Register6
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PG        |Set GPIO Port G to Non-secure State
     * |        |          |Write 1 to set PG to non-secure state. Each bit configures one pin in GPIO port G.
     * |        |          |0 = GPIO port G is secure (default).
     * |        |          |1 = GPIO port G is non-secure.
     * Offset: 0x8C  I/O Non-secure Attribution Set Register7
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PH        |Set GPIO Port H to Non-secure State
     * |        |          |Write 1 to set PH to non-secure state. Each bit configures one pin in GPIO port H.
     * |        |          |0 = GPIO port H is secure (default).
     * |        |          |1 = GPIO port H is non-secure.
     * Offset: 0x90  I/O Non-secure Attribution Set Register8
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PI        |Set GPIO Port I to Non-secure State
     * |        |          |Write 1 to set PI to non-secure state. Each bit configures one pin in GPIO port I.
     * |        |          |0 = GPIO port I is secure (default).
     * |        |          |1 = GPIO port I is non-secure.
     * Offset: 0x94  I/O Non-secure Attribution Set Register9
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PJ        |Set GPIO Port J to Non-secure State
     * |        |          |Write 1 to set PJ to non-secure state. Each bit configures one pin in GPIO port J.
     * |        |          |0 = GPIO port J is secure (default).
     * |        |          |1 = GPIO port J is non-secure.
     * @var SCU_T::SVIEN0
     * Offset: 0x100  Security Violation Interrupt Enable Register u2013 Slave peripherals
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |APB0IEN   |APB0 Security Violation Interrupt Enable Bit
     * |        |          |0 = Interrupt triggered from security violation of APB0 Disabled.
     * |        |          |1 = Interrupt triggered from security violation of APB0 Enabled.
     * |[1]     |APB1IEN   |APB1 Security Violation Interrupt Enable Bit
     * |        |          |0 = Interrupt triggered from security violation of APB1 Disabled.
     * |        |          |1 = Interrupt triggered from security violation of APB1 Enabled.
     * |[2]     |APB2IEN   |APB2 Security Violation Interrupt Enable Bit
     * |        |          |0 = Interrupt triggered from security violation of APB2 Disabled.
     * |        |          |1 = Interrupt triggered from security violation of APB2 Enabled.
     * |[3]     |APB3IEN   |APB3 Security Violation Interrupt Enable Bit
     * |        |          |0 = Interrupt triggered from security violation of APB3 Disabled.
     * |        |          |1 = Interrupt triggered from security violation of APB3 Enabled.
     * |[4]     |APB4IEN   |APB4 Security Violation Interrupt Enable Bit
     * |        |          |0 = Interrupt triggered from security violation of APB4 Disabled.
     * |        |          |1 = Interrupt triggered from security violation of APB4 Enabled.
     * |[5]     |APB5IEN   |APB5 Security Violation Interrupt Enable Bit
     * |        |          |0 = Interrupt triggered from security violation of APB5 Disabled.
     * |        |          |1 = Interrupt triggered from security violation of APB5 Enabled.
     * |[8]     |D0PPC0IEN |D0PPC0 Security Violation Interrupt Enable Bit
     * |        |          |0 = Interrupt triggered from security violation of D0PPC0 Disabled.
     * |        |          |1 = Interrupt triggered from security violation of D0PPC0 Enabled.
     * |[9]     |D1PPC0IEN |D1PPC0 Security Violation Interrupt Enable Bit
     * |        |          |0 = Interrupt triggered from security violation of D1PPC0 Disabled.
     * |        |          |1 = Interrupt triggered from security violation of D1PPC0 Enabled.
     * |[10]    |D1PPC1IEN |D1PPC1 Security Violation Interrupt Enable Bit
     * |        |          |0 = Interrupt triggered from security violation of D1PPC1 Disabled.
     * |        |          |1 = Interrupt triggered from security violation of D1PPC1 Enabled.
     * |[11]    |D2PPC0IEN |D2PPC0 Security Violation Interrupt Enable Bit
     * |        |          |0 = Interrupt triggered from security violation of D2PPC0 Disabled.
     * |        |          |1 = Interrupt triggered from security violation of D2PPC0 Enabled.
     * |[16]    |EBIIEN    |EBI Security Violation Interrupt Enable Bit
     * |        |          |0 = Interrupt triggered from security violation of EBI Disabled.
     * |        |          |1 = Interrupt triggered from security violation of EBI Enabled.
     * |[17]    |FLASHIEN  |FLASH Security Violation Interrupt Enable Bit
     * |        |          |0 = Interrupt triggered from security violation of FLASH Disabled.
     * |        |          |1 = Interrupt triggered from security violation of FLASH Enabled.
     * @var SCU_T::SVIEN1
     * Offset: 0x104  Security Violation Interrupt Enable Register u2013 Master peripherals (MSC)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GDMAIEN   |GDMA Security Violation Interrupt Enable Bit 0 = Interrupt triggered from security violation of GDMA Disabled. 1 = Interrupt triggered from security violation of GDMA Enabled.
     * |[1]     |PDMA0IEN  |PDMA0 Security Violation Interrupt Enable Bit 0 = Interrupt triggered from security violation of PDMA Disabled. 1 = Interrupt triggered from security violation of PDMA Enabled.
     * |[2]     |PDMA1IEN  |PDMA1 Security Violation Interrupt Enable Bit 0 = Interrupt triggered from security violation of PDMA1 Disabled. 1 = Interrupt triggered from security violation of PDMA1 Enabled.
     * |[3]     |USBH0IEN  |USBH0 Security Violation Interrupt Enable Bit 0 = Interrupt triggered from security violation of USBH0 Disabled. 1 = Interrupt triggered from security violation of USBH0 Enabled.
     * |[5]     |HSUSBHIEN |HSUSBH Security Violation Interrupt Enable Bit 0 = Interrupt triggered from security violation of HSUSBH Disabled. 1 = Interrupt triggered from security violation of HSUSBH Enabled.
     * |[6]     |HSUSBDIEN |HSUSBD Security Violation Interrupt Enable Bit 0 = Interrupt triggered from security violation of HSUSBD Disabled. 1 = Interrupt triggered from security violation of HSUSBD Enabled.
     * |[7]     |SDH0IEN   |SDH0 Security Violation Interrupt Enable Bit 0 = Interrupt triggered from security violation of SDH0 Disabled. 1 = Interrupt triggered from security violation of SDH0 Enabled.
     * |[8]     |SDH1IEN   |SDH1 Security Violation Interrupt Enable Bit 0 = Interrupt triggered from security violation of SDH1 Disabled. 1 = Interrupt triggered from security violation of SDH1 Enabled.
     * |[9]     |EMACIEN   |EMAC Security Violation Interrupt Enable Bit 0 = Interrupt triggered from security violation of EMAC Disabled. 1 = Interrupt triggered from security violation of EMAC Enabled.
     * |[10]    |CRYPTOIEN |CRYPTO Security Violation Interrupt Enable Bit 0 = Interrupt triggered from security violation of CRYPTO Disabled. 1 = Interrupt triggered from security violation of CRYPTO Enabled.
     * |[11]    |CRCIEN    |CRC Security Violation Interrupt Enable Bit 0 = Interrupt triggered from security violation of CRC Disabled. 1 = Interrupt triggered from security violation of CRC Enabled.
     * |[12]    |LPPDMAIEN |LPPDMA Security Violation Interrupt Enable Bit 0 = Interrupt triggered from security violation of LPPDMA Disabled. 1 = Interrupt triggered from security violation of LPPDMA Enabled.
     * |[13]    |CCAPIEN   |CCAP Security Violation Interrupt Enable Bit 0 = Interrupt triggered from security violation of CCAP Disabled. 1 = Interrupt triggered from security violation of CCAP Enabled.
     * |[14]    |NPUIF1IEN |NPU Interface 1 Security Violation Interrupt Enable Bit 0 = Interrupt triggered from security violation of NPU Interface 1 Disabled. 1 = Interrupt triggered from security violation of NPU Interface 1 Enabled.
     * |[15]    |NPUIF0IEN |NPU Interface 0 Security Violation Interrupt Enable Bit 0 = Interrupt triggered from security violation of NPU Interface 0 Disabled. 1 = Interrupt triggered from security violation of NPU Interface 0 Enabled.
     * |[16]    |SPIM0IEN  |SPIM0 Security Violation Interrupt Enable Bit 0 = Interrupt triggered from security violation of SPIM0 Disabled. 1 = Interrupt triggered from security violation of SPIM0 Enabled.
     * @var SCU_T::SVINTSTS0
     * Offset: 0x110  Security Violation Interrupt Status Register u2013 Slave peripherals
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |APB0IF    |APB0 Security Violation Interrupt Status
     * |        |          |0 = No APB0 violation interrupt event.
     * |        |          |1 = There is at least a APB0 violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[1]     |APB1IF    |APB1 Security Violation Interrupt Status
     * |        |          |0 = No APB1 violation interrupt event.
     * |        |          |1 = There is at least a APB1 violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[2]     |APB2IF    |APB2 Security Violation Interrupt Status
     * |        |          |0 = No APB2 violation interrupt event.
     * |        |          |1 = There is at least a APB2 violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[3]     |APB3IF    |APB3 Security Violation Interrupt Status
     * |        |          |0 = No APB3 violation interrupt event.
     * |        |          |1 = There is at least a APB3 violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[4]     |APB4IF    |APB4 Security Violation Interrupt Status
     * |        |          |0 = No APB4 violation interrupt event.
     * |        |          |1 = There is at least a APB4 violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[5]     |APB5IF    |APB5 Security Violation Interrupt Status
     * |        |          |0 = No APB5 violation interrupt event.
     * |        |          |1 = There is at least a APB5 violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[8]     |D0PPC0IF  |D0PPC0 Security Violation Interrupt Status
     * |        |          |0 = No D0PPC0 violation interrupt event.
     * |        |          |1 = There is at least a D0PPC0 violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[9]     |D1PPC0IF  |D1PPC0 Security Violation Interrupt Status
     * |        |          |0 = No D1PPC0 violation interrupt event.
     * |        |          |1 = There is at least a D1PPC0 violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[10]    |D1PPC1IF  |D1PPC1 Security Violation Interrupt Status
     * |        |          |0 = No D1PPC1 violation interrupt event.
     * |        |          |1 = There is at least a D1PPC1 violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[11]    |D2PPC0IF  |D2PPC0 Security Violation Interrupt Status
     * |        |          |0 = No D2PPC0 violation interrupt event.
     * |        |          |1 = There is at least a D2PPC0 violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[16]    |EBIIF     |EBI Security Violation Interrupt Status
     * |        |          |0 = No EBI violation interrupt event.
     * |        |          |1 = There is at least a EBI violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[17]    |FLASHIF   |FLASH Security Violation Interrupt Status
     * |        |          |0 = No FLASH violation interrupt event.
     * |        |          |1 = There is at least a FLASH violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * @var SCU_T::SVINTSTS1
     * Offset: 0x114  Security Violation Interrupt Status Register - Master peripherals (MSC)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GDMAIF    |GDMA Security Violation Interrupt Status
     * |        |          |0 = No GDMA violation interrupt event.
     * |        |          |1 = There is at least a GDMA violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[1]     |PDMA0IF   |PDMA0 Security Violation Interrupt Status
     * |        |          |0 = No PDMA0 violation interrupt event.
     * |        |          |1 = There is at least a PDMA0 violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[2]     |PDMA1IF   |PDMA1 Security Violation Interrupt Status
     * |        |          |0 = No PDMA1 violation interrupt event.
     * |        |          |1 = There is at least a PDMA1 violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[3]     |USBH0IF   |USBH0 Security Violation Interrupt Status
     * |        |          |0 = No USBH0 violation interrupt event.
     * |        |          |1 = There is at least a USBH0 violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[5]     |HSUSBHIF  |HSUSBH Security Violation Interrupt Status
     * |        |          |0 = No HSUSBH violation interrupt event.
     * |        |          |1 = There is at least a HSUSBH violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[6]     |HSUSBDIF  |HSUSBD Security Violation Interrupt Status
     * |        |          |0 = No HSUSBD violation interrupt event.
     * |        |          |1 = There is at least a HSUSBD violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[7]     |SDH0IF    |SDH0 Security Violation Interrupt Status
     * |        |          |0 = No SDH0 violation interrupt event.
     * |        |          |1 = There is at least a SDH0 violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[8]     |SDH1IF    |SDH1 Security Violation Interrupt Status
     * |        |          |0 = No SDH1 violation interrupt event.
     * |        |          |1 = There is at least a SDH1 violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[9]     |EMACIF    |EMAC Security Violation Interrupt Status
     * |        |          |0 = No EMAC violation interrupt event.
     * |        |          |1 = There is at least a EMAC violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[10]    |CRYPTOIF  |CRYPTO Security Violation Interrupt Status
     * |        |          |0 = No CRYPTO violation interrupt event.
     * |        |          |1 = There is at least a CRYPTO violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[11]    |CRCIF     |CRC Security Violation Interrupt Status
     * |        |          |0 = No CRC violation interrupt event.
     * |        |          |1 = There is at least a CRC violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[12]    |LPPDMAIF  |LPPDMA Security Violation Interrupt Status
     * |        |          |0 = No LPPDMA violation interrupt event.
     * |        |          |1 = There is at least a LPPDMA violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[13]    |CCAPIF    |CCAP Security Violation Interrupt Status
     * |        |          |0 = No CCAP violation interrupt event.
     * |        |          |1 = There is at least a CCAP violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[14]    |NPUIF1    |NPU Interface 1 Security Violation Interrupt Status
     * |        |          |0 = No NPU Interface 1 violation interrupt event.
     * |        |          |1 = There is at least a NPU Interface 1 violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[15]    |NPUIF0    |NPU Interface 0 Security Violation Interrupt Status
     * |        |          |0 = No NPU Interface 0 violation interrupt event.
     * |        |          |1 = There is at least a NPU Interface 0 violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * |[16]    |SPIM0IF   |SPIM0 Security Violation Interrupt Status
     * |        |          |0 = No SPIM0 violation interrupt event.
     * |        |          |1 = There is at least a SPIM0 violation interrupt event.
     * |        |          |Note: Write 1 to clear the interrupt flag.
     * @var SCU_T::SVINTSTS2
     * Offset: 0x118  Security Violation Interrupt Status Register u2013 Memory peripherals (MPC)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SRAM0IF   |SRAM0 Security Violation Interrupt Status
     * |        |          |0 = No SRAM0 violation interrupt event.
     * |        |          |1 = There is at least a SRAM0 violation interrupt event.
     * |[1]     |SRAM1IF   |SRAM1 Security Violation Interrupt Status
     * |        |          |0 = No SRAM1 violation interrupt event.
     * |        |          |1 = There is at least a SRAM1 violation interrupt event.
     * |[2]     |SRAM2IF   |SRAM2 Security Violation Interrupt Status
     * |        |          |0 = No SRAM2 violation interrupt event.
     * |        |          |1 = There is at least a SRAM2 violation interrupt event.
     * |[3]     |SRAM3IF   |SRAM3 Security Violation Interrupt Status
     * |        |          |0 = No SRAM3 violation interrupt event.
     * |        |          |1 = There is at least a SRAM3 violation interrupt event.
     * |[4]     |LPSRAMIF  |LPSRAM Security Violation Interrupt Status
     * |        |          |0 = No LPSRAM violation interrupt event.
     * |        |          |1 = There is at least a LPSRAM violation interrupt event.
     * |[5]     |SPIM0IF   |SPIM0 Security Violation Interrupt Status
     * |        |          |0 = No SPIM0 violation interrupt event.
     * |        |          |1 = There is at least a SPIM0 violation interrupt event.
     * @var SCU_T::APB0PVSRC
     * Offset: 0x120  APB0 Security Policy Violation Source
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |MASTER    |Master Violating Security Policy
     * |        |          |Indicate which master invokes the security violation.
     * |        |          |0x00 = Core processor
     * |        |          |0x01 = PDMA0
     * |        |          |0x02 = PDMA1
     * |        |          |0x03 = USBH0
     * |        |          |0x05 = HSUSBH
     * |        |          |0x06 = HSUSBD
     * |        |          |0x07 = SDH0
     * |        |          |0x08 = SDH1
     * |        |          |0x09 = EMAC
     * |        |          |0x0A = CRYPTO
     * |        |          |0x0B = CRC
     * |        |          |0x0C = GDMA
     * |        |          |0x0D = NPU
     * |        |          |0x0E = LPPDMA
     * |        |          |0x0F = CCAP
     * |        |          |0x10 = SPIM0
     * |        |          |Others are undefined.
     * @var SCU_T::APB1PVSRC
     * Offset: 0x124  APB1 Security Policy Violation Source
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |MASTER    |Master Violating Security Policy
     * |        |          |Indicate which master invokes the security violation.
     * |        |          |0x00 = Core processor
     * |        |          |0x01 = PDMA0
     * |        |          |0x02 = PDMA1
     * |        |          |0x03 = USBH0
     * |        |          |0x05 = HSUSBH
     * |        |          |0x06 = HSUSBD
     * |        |          |0x07 = SDH0
     * |        |          |0x08 = SDH1
     * |        |          |0x09 = EMAC
     * |        |          |0x0A = CRYPTO
     * |        |          |0x0B = CRC
     * |        |          |0x0C = GDMA
     * |        |          |0x0D = NPU
     * |        |          |0x0E = LPPDMA
     * |        |          |0x0F = CCAP
     * |        |          |0x10 = SPIM0
     * |        |          |Others are undefined.
     * @var SCU_T::APB2PVSRC
     * Offset: 0x128  APB2 Security Policy Violation Source
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |MASTER    |Master Violating Security Policy
     * |        |          |Indicate which master invokes the security violation.
     * |        |          |0x00 = Core processor
     * |        |          |0x01 = PDMA0
     * |        |          |0x02 = PDMA1
     * |        |          |0x03 = USBH0
     * |        |          |0x05 = HSUSBH
     * |        |          |0x06 = HSUSBD
     * |        |          |0x07 = SDH0
     * |        |          |0x08 = SDH1
     * |        |          |0x09 = EMAC
     * |        |          |0x0A = CRYPTO
     * |        |          |0x0B = CRC
     * |        |          |0x0C = GDMA
     * |        |          |0x0D = NPU
     * |        |          |0x0E = LPPDMA
     * |        |          |0x0F = CCAP
     * |        |          |0x10 = SPIM0
     * |        |          |Others are undefined.
     * @var SCU_T::APB3PVSRC
     * Offset: 0x12C  APB3 Security Policy Violation Source
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |MASTER    |Master Violating Security Policy
     * |        |          |Indicate which master invokes the security violation.
     * |        |          |0x00 = Core processor
     * |        |          |0x01 = PDMA0
     * |        |          |0x02 = PDMA1
     * |        |          |0x03 = USBH0
     * |        |          |0x05 = HSUSBH
     * |        |          |0x06 = HSUSBD
     * |        |          |0x07 = SDH0
     * |        |          |0x08 = SDH1
     * |        |          |0x09 = EMAC
     * |        |          |0x0A = CRYPTO
     * |        |          |0x0B = CRC
     * |        |          |0x0C = GDMA
     * |        |          |0x0D = NPU
     * |        |          |0x0E = LPPDMA
     * |        |          |0x0F = CCAP
     * |        |          |0x10 = SPIM0
     * |        |          |Others are undefined.
     * @var SCU_T::APB4PVSRC
     * Offset: 0x130  APB4 Security Policy Violation Source
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |MASTER    |Master Violating Security Policy
     * |        |          |Indicate which master invokes the security violation.
     * |        |          |0x00 = Core processor
     * |        |          |0x01 = PDMA0
     * |        |          |0x02 = PDMA1
     * |        |          |0x03 = USBH0
     * |        |          |0x05 = HSUSBH
     * |        |          |0x06 = HSUSBD
     * |        |          |0x07 = SDH0
     * |        |          |0x08 = SDH1
     * |        |          |0x09 = EMAC
     * |        |          |0x0A = CRYPTO
     * |        |          |0x0B = CRC
     * |        |          |0x0C = GDMA
     * |        |          |0x0D = NPU
     * |        |          |0x0E = LPPDMA
     * |        |          |0x0F = CCAP
     * |        |          |0x10 = SPIM0
     * |        |          |Others are undefined.
     * @var SCU_T::APB5PVSRC
     * Offset: 0x134  APB5 Security Policy Violation Source
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |MASTER    |Master Violating Security Policy
     * |        |          |Indicate which master invokes the security violation.
     * |        |          |0x00 = Core processor
     * |        |          |0x01 = PDMA0
     * |        |          |0x02 = PDMA1
     * |        |          |0x03 = USBH0
     * |        |          |0x05 = HSUSBH
     * |        |          |0x06 = HSUSBD
     * |        |          |0x07 = SDH0
     * |        |          |0x08 = SDH1
     * |        |          |0x09 = EMAC
     * |        |          |0x0A = CRYPTO
     * |        |          |0x0B = CRC
     * |        |          |0x0C = GDMA
     * |        |          |0x0D = NPU
     * |        |          |0x0E = LPPDMA
     * |        |          |0x0F = CCAP
     * |        |          |0x10 = SPIM0
     * |        |          |Others are undefined.
     * @var SCU_T::D0PPC0PVSRC
     * Offset: 0x140  D0PPC0 Security Policy Violation Source
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |MASTER    |Master Violating Security Policy
     * |        |          |Indicate which master invokes the security violation.
     * |        |          |0x00 = Core processor
     * |        |          |0x01 = PDMA0
     * |        |          |0x02 = PDMA1
     * |        |          |0x03 = USBH0
     * |        |          |0x05 = HSUSBH
     * |        |          |0x06 = HSUSBD
     * |        |          |0x07 = SDH0
     * |        |          |0x08 = SDH1
     * |        |          |0x09 = EMAC
     * |        |          |0x0A = CRYPTO
     * |        |          |0x0B = CRC
     * |        |          |0x0C = GDMA
     * |        |          |0x0D = NPU
     * |        |          |0x0E = LPPDMA
     * |        |          |0x0F = CCAP
     * |        |          |0x10 = SPIM0
     * |        |          |Others are undefined.
     * @var SCU_T::D1PPC0PVSRC
     * Offset: 0x144  D1PPC0 Security Policy Violation Source
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |MASTER    |Master Violating Security Policy
     * |        |          |Indicate which master invokes the security violation.
     * |        |          |0x00 = Core processor
     * |        |          |0x01 = PDMA0
     * |        |          |0x02 = PDMA1
     * |        |          |0x03 = USBH0
     * |        |          |0x05 = HSUSBH
     * |        |          |0x06 = HSUSBD
     * |        |          |0x07 = SDH0
     * |        |          |0x08 = SDH1
     * |        |          |0x09 = EMAC
     * |        |          |0x0A = CRYPTO
     * |        |          |0x0B = CRC
     * |        |          |0x0C = GDMA
     * |        |          |0x0D = NPU
     * |        |          |0x0E = LPPDMA
     * |        |          |0x0F = CCAP
     * |        |          |0x10 = SPIM0
     * |        |          |Others are undefined.
     * @var SCU_T::D1PPC1PVSRC
     * Offset: 0x148  D1PPC1 Security Policy Violation Source
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |MASTER    |Master Violating Security Policy
     * |        |          |Indicate which master invokes the security violation.
     * |        |          |0x00 = Core processor
     * |        |          |0x01 = PDMA0
     * |        |          |0x02 = PDMA1
     * |        |          |0x03 = USBH0
     * |        |          |0x05 = HSUSBH
     * |        |          |0x06 = HSUSBD
     * |        |          |0x07 = SDH0
     * |        |          |0x08 = SDH1
     * |        |          |0x09 = EMAC
     * |        |          |0x0A = CRYPTO
     * |        |          |0x0B = CRC
     * |        |          |0x0C = GDMA
     * |        |          |0x0D = NPU
     * |        |          |0x0E = LPPDMA
     * |        |          |0x0F = CCAP
     * |        |          |0x10 = SPIM0
     * |        |          |Others are undefined.
     * @var SCU_T::D2PPC0PVSRC
     * Offset: 0x14C  D2PPC0 Security Policy Violation Source
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |MASTER    |Master Violating Security Policy
     * |        |          |Indicate which master invokes the security violation.
     * |        |          |0x00 = Core processor
     * |        |          |0x01 = PDMA0
     * |        |          |0x02 = PDMA1
     * |        |          |0x03 = USBH0
     * |        |          |0x05 = HSUSBH
     * |        |          |0x06 = HSUSBD
     * |        |          |0x07 = SDH0
     * |        |          |0x08 = SDH1
     * |        |          |0x09 = EMAC
     * |        |          |0x0A = CRYPTO
     * |        |          |0x0B = CRC
     * |        |          |0x0C = GDMA
     * |        |          |0x0D = NPU
     * |        |          |0x0E = LPPDMA
     * |        |          |0x0F = CCAP
     * |        |          |0x10 = SPIM0
     * |        |          |Others are undefined.
     * @var SCU_T::EBIPVSRC
     * Offset: 0x160  EBI Security Policy Violation Source
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |MASTER    |Master Violating Security Policy
     * |        |          |Indicate which master invokes the security violation.
     * |        |          |0x00 = Core processor
     * |        |          |0x01 = PDMA0
     * |        |          |0x02 = PDMA1
     * |        |          |0x03 = USBH0
     * |        |          |0x05 = HSUSBH
     * |        |          |0x06 = HSUSBD
     * |        |          |0x07 = SDH0
     * |        |          |0x08 = SDH1
     * |        |          |0x09 = EMAC
     * |        |          |0x0A = CRYPTO
     * |        |          |0x0B = CRC
     * |        |          |0x0C = GDMA
     * |        |          |0x0D = NPU
     * |        |          |0x0E = LPPDMA
     * |        |          |0x0F = CCAP
     * |        |          |0x10 = SPIM0
     * |        |          |Others are undefined.
     * @var SCU_T::FLASHPVSRC
     * Offset: 0x164  FLASH Security Policy Violation Source
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |MASTER    |Master Violating Security Policy
     * |        |          |Indicate which master invokes the security violation.
     * |        |          |0x00 = Core processor
     * |        |          |0x01 = PDMA0
     * |        |          |0x02 = PDMA1
     * |        |          |0x03 = USBH0
     * |        |          |0x04 = Reserved
     * |        |          |0x05 = HSUSBH
     * |        |          |0x06 = HSUSBD
     * |        |          |0x07 = SDH0
     * |        |          |0x08 = SDH1
     * |        |          |0x09 = EMAC
     * |        |          |0x0A = CRYPTO
     * |        |          |0x0B = CRC
     * |        |          |0x0C = GDMA
     * |        |          |0x0D = NPU
     * |        |          |0x0E = LPDMA
     * |        |          |0x0F = CCAP
     * |        |          |0x10 = SPIM0
     * |        |          |Others are undefined.
     * @var SCU_T::APB0PVA
     * Offset: 0x1A0  APB0 Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::APB1PVA
     * Offset: 0x1A4  APB1 Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::APB2PVA
     * Offset: 0x1A8  APB2 Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::APB3PVA
     * Offset: 0x1AC  APB3 Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::APB4PVA
     * Offset: 0x1B0  APB4 Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::APB5PVA
     * Offset: 0x1B4  APB5 Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::D0PPC0PVA
     * Offset: 0x1C0  D0PPC0 Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::D1PPC0PVA
     * Offset: 0x1C4  D1PPC0 Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::D1PPC1PVA
     * Offset: 0x1C8  D1PPC1 Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::D2PPC0PVA
     * Offset: 0x1CC  D2PPC0 Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::EBIPVA
     * Offset: 0x1E0  EBI Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::FLASHPVA
     * Offset: 0x1E4  FLASH Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::GDMAMVA
     * Offset: 0x220  GDMA Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::PDMA0MVA
     * Offset: 0x224  PDMA Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::PDMA1MVA
     * Offset: 0x228  PDMA1 Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::USBH0MVA
     * Offset: 0x22C  USBH0 Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::HSUSBHMVA
     * Offset: 0x234  HSUSBH Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::HSUSBDMVA
     * Offset: 0x238  HSUSBD Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::SDH0MVA
     * Offset: 0x23C  SDH Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::SDH1MVA
     * Offset: 0x240  SDH1 Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::EMACMVA
     * Offset: 0x244  EMAC Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::CRYPTOMVA
     * Offset: 0x248  CRYPTO Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::CRCMVA
     * Offset: 0x24C  CRC Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::LPPDMAMVA
     * Offset: 0x250  LPPDMA Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::CCAPMVA
     * Offset: 0x254  CCAP Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::NPUIF1MVA
     * Offset: 0x258  NPU Interface 1 Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::NPUIF0MVA
     * Offset: 0x25C  NPU Interface 0 Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::SPIM0MVA
     * Offset: 0x260  SPIM 0 Master Violation Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VIOADDR   |Violation Address
     * |        |          |Indicate the target address of the access, which invokes the security violation.
     * @var SCU_T::D0PNP0
     * Offset: 0x2A0  Domain 0 Peripheral Privilege Attribution Set Register0 (0x4000_0000~0x4001_FFFF)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3]     |NPU       |Set NPU to Non-privileged State
     * |        |          |0 = NPU is a privileged module (default).
     * |        |          |1 = NPU is a non-privileged module.
     * |[8]     |SRAM0MPC  |Set SRAM0MPC to Non-privileged State
     * |        |          |0 = SRAM0MPC is a privileged module (default).
     * |        |          |1 = SRAM0MPC is a non-privileged module.
     * |[9]     |SRAM1MPC  |Set SRAM1MPC to Non-privileged State
     * |        |          |0 = SRAM1MPC is a privileged module (default).
     * |        |          |1 = SRAM1MPC is a non-privileged module.
     * |[10]    |SRAM2MPC  |Set SRAM2MPC to Non-privileged State
     * |        |          |0 = SRAM2MPC is a privileged module (default).
     * |        |          |1 = SRAM2MPC is a non-privileged module.
     * |[11]    |SRAM3MPC  |Set SRAM3MPC to Non-privileged State
     * |        |          |0 = SRAM3MPC is a privileged module (default).
     * |        |          |1 = SRAM3MPC is a non-privileged module.
     * |[13]    |SPIM0MPC  |Set SPIM0MPC to Non-privileged State
     * |        |          |0 = SPIM0MPC is a privileged module (default).
     * |        |          |1 = SPIM0MPC is a non-privileged module.
     * @var SCU_T::D0PNP2
     * Offset: 0x2A8  Domain 0 Peripheral Privilege Attribution Set Register2 (0x4004_0000~0x4005_FFFF)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |OTFC0     |Set OTFC0 to Non-privileged State
     * |        |          |0 = OTFC0 is a privileged module (default).
     * |        |          |1 = OTFC0 is a non-privileged module.
     * |[2]     |SPIM0     |Set SPIM0 to Non-privileged State
     * |        |          |0 = SPIM0 is a privileged module (default).
     * |        |          |1 = SPIM0 is a non-privileged module.
     * |[4]     |FMC       |Set FMC to Non-privileged State
     * |        |          |0 = FMC is a privileged module (default).
     * |        |          |1 = FMC is a non-privileged module.
     * @var SCU_T::D1PNP0
     * Offset: 0x2C0  Domain 1 Peripheral Privilege Attribution Set Register0 (0x4020_0000~0x4021_FFFF)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PDMA0     |Set PDMA0 to Non-privileged State
     * |        |          |0 = PDMA0 is a privileged module (default).
     * |        |          |1 = PDMA0 is a non-privileged module.
     * |[1]     |PDMA1     |Set PDMA1 to Non-privileged State
     * |        |          |0 = PDMA1 is a privileged module (default).
     * |        |          |1 = PDMA1 is a non-privileged module.
     * |[2]     |USBH0     |Set USBH0 to Non-privileged State
     * |        |          |0 = USBH0 is a privileged module (default).
     * |        |          |1 = USBH0 is a non-privileged module.
     * |[4]     |HSUSBH    |Set HSUSBH to Non-privileged State
     * |        |          |0 = HSUSBH is a privileged module (default).
     * |        |          |1 = HSUSBH is a non-privileged module.
     * |[5]     |HSUSBD    |Set HSUSBD to Non-privileged State
     * |        |          |0 = HSUSBD is a privileged module (default).
     * |        |          |1 = HSUSBD is a non-privileged module.
     * |[6]     |SDH0      |Set SDH0 to Non-privileged State
     * |        |          |0 = SDH0 is a privileged module (default).
     * |        |          |1 = SDH0 is a non-privileged module.
     * |[7]     |SDH1      |Set SDH1 to Non-privileged State
     * |        |          |0 = SDH1 is a privileged module (default).
     * |        |          |1 = SDH1 is a non-privileged module.
     * |[8]     |EMAC0     |Set EMAC0 to Non-privileged State
     * |        |          |0 = EMAC0 is a privileged module (default).
     * |        |          |1 = EMAC0 is a non-privileged module.
     * |[10]    |CRYPTO    |Set CRYPTO to Non-privileged State
     * |        |          |0 = CRYPTO is a privileged module (default).
     * |        |          |1 = CRYPTO is a non-privileged module.
     * |[11]    |CRC       |Set CRC to Non-privileged State
     * |        |          |0 = CRC is a privileged module (default).
     * |        |          |1 = CRC is a non-privileged module.
     * @var SCU_T::D1PNP1
     * Offset: 0x2C4  Domain0 Peripheral Privilege Attribution Set Register1 (0x4022_0000~0x4023_FFFF)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |KS        |Set KS to Non-privileged State
     * |        |          |0 = KS is a privileged module (default).
     * |        |          |1 = KS is a non-privileged module.
     * |[1]     |KDF       |Set KDF to Non-privileged State
     * |        |          |0 = KDF is a privileged module (default).
     * |        |          |1 = KDF is a non-privileged module.
     * |[2]     |CANFD0    |Set CANFD0 to Non-privileged State
     * |        |          |0 = CANFD0 is a privileged module (default).
     * |        |          |1 = CANFD0 is a non-privileged module.
     * |[4]     |CANFD1    |Set CANFD1 to Non-privileged State
     * |        |          |0 = CANFD1 is a privileged module (default).
     * |        |          |1 = CANFD1 is a non-privileged module.
     * |[16]    |EBI       |Set EBI to Non-privileged State
     * |        |          |0 = EBI is a privileged module (default).
     * |        |          |1 = EBI is a non-privileged module.
     * @var SCU_T::D1PNP2
     * Offset: 0x2C8  Domain 1 Peripheral Privilege Attribution Set Register2 (0x4024_0000~0x4025_FFFF)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDT0     |Set WWDT0 to Non-privileged State
     * |        |          |0 = WWDT0 is a privileged module (default).
     * |        |          |1 = WWDT0 is a non-privileged module.
     * |[1]     |EADC0     |Set EADC0 to Non-privileged State
     * |        |          |0 = EADC0 is a privileged module (default).
     * |        |          |1 = EADC0 is a non-privileged module.
     * |[2]     |EPWM0     |Set EPWM0 to Non-privileged State
     * |        |          |0 = EPWM0 is a privileged module (default).
     * |        |          |1 = EPWM0 is a non-privileged module.
     * |[3]     |BPWM0     |Set BPWM0 to Non-privileged State
     * |        |          |0 = BPWM0 is a privileged module (default).
     * |        |          |1 = BPWM0 is a non-privileged module.
     * |[4]     |EQEI0     |Set EQEI0 to Non-privileged State
     * |        |          |0 = EQEI0 is a privileged module (default).
     * |        |          |1 = EQEI0 is a non-privileged module.
     * |[5]     |EQEI2     |Set EQEI2 to Non-privileged State
     * |        |          |0 = EQEI2 is a privileged module (default).
     * |        |          |1 = EQEI2 is a non-privileged module.
     * |[6]     |ECAP0     |Set ECAP0 to Non-privileged State
     * |        |          |0 = ECAP0 is a privileged module (default).
     * |        |          |1 = ECAP0 is a non-privileged module.
     * |[7]     |ECAP2     |Set ECAP2 to Non-privileged State
     * |        |          |0 = ECAP2 is a privileged module (default).
     * |        |          |1 = ECAP2 is a non-privileged module.
     * |[8]     |I2C0      |Set I2C0 to Non-privileged State
     * |        |          |0 = I2C0 is a privileged module (default).
     * |        |          |1 = I2C0 is a non-privileged module.
     * |[9]     |I2C2      |Set I2C2 to Non-privileged State
     * |        |          |0 = I2C2 is a privileged module (default).
     * |        |          |1 = I2C2 is a non-privileged module.
     * |[10]    |QSPI0     |Set QSPI0 to Non-privileged State
     * |        |          |0 = QSPI0 is a privileged module (default).
     * |        |          |1 = QSPI0 is a non-privileged module.
     * |[11]    |SPI0      |Set SPI0 to Non-privileged State
     * |        |          |0 = SPI0 is a privileged module (default).
     * |        |          |1 = SPI0 is a non-privileged module.
     * |[12]    |SPI2      |Set SPI2 to Non-privileged State
     * |        |          |0 = SPI2 is a privileged module (default).
     * |        |          |1 = SPI2 is a non-privileged module.
     * |[13]    |UART0     |Set UART0 to Non-privileged State
     * |        |          |0 = UART0 is a privileged module (default).
     * |        |          |1 = UART0 is a non-privileged module.
     * |[14]    |UART2     |Set UART2 to Non-privileged State
     * |        |          |0 = UART2 is a privileged module (default).
     * |        |          |1 = UART2 is a non-privileged module.
     * |[15]    |UART4     |Set UART4 to Non-privileged State
     * |        |          |0 = UART4 is a privileged module (default).
     * |        |          |1 = UART4 is a non-privileged module.
     * |[16]    |UART6     |Set UART6 to Non-privileged State
     * |        |          |0 = UART6 is a privileged module (default).
     * |        |          |1 = UART6 is a non-privileged module.
     * |[17]    |UART8     |Set UART8 to Non-privileged State
     * |        |          |0 = UART8 is a privileged module (default).
     * |        |          |1 = UART8 is a non-privileged module.
     * |[18]    |USCI0     |Set USCI0 to Non-privileged State
     * |        |          |0 = USCI0 is a privileged module (default).
     * |        |          |1 = USCI0 is a non-privileged module.
     * |[19]    |SC0       |Set SC0 to Non-privileged State
     * |        |          |0 = SC0 is a privileged module (default).
     * |        |          |1 = SC0 is a non-privileged module.
     * |[20]    |SC2       |Set SC2 to Non-privileged State
     * |        |          |0 = SC2 is a privileged module (default).
     * |        |          |1 = SC2 is a non-privileged module.
     * |[21]    |PSIO      |Set PSIO to Non-privileged State
     * |        |          |0 = PSIO is a privileged module (default).
     * |        |          |1 = PSIO is a non-privileged module.
     * |[22]    |TMR01     |Set TMR01 to Non-privileged State
     * |        |          |0 = TMR01 is a privileged module (default).
     * |        |          |1 = TMR01 is a non-privileged module.
     * |[23]    |DAC01     |Set DAC01 to Non-privileged State
     * |        |          |0 = DAC01 is a privileged module (default).
     * |        |          |1 = DAC01 is a non-privileged module.
     * |[24]    |TAMPER    |Set TAMPER to Non-privileged State
     * |        |          |0 = TAMPER is a privileged module (default).
     * |        |          |1 = TAMPER is a non-privileged module.
     * |[25]    |HSOTG     |Set HSOTG to Non-privileged State
     * |        |          |0 = HSOTG is a privileged module (default).
     * |        |          |1 = HSOTG is a non-privileged module.
     * |[26]    |I2S0      |Set I2S0 to Non-privileged State
     * |        |          |0 = I2S0 is a privileged module (default).
     * |        |          |1 = I2S0 is a non-privileged module.
     * |[27]    |ACMP01    |Set ACMP01 to Non-privileged State
     * |        |          |0 = ACMP01 is a privileged module (default).
     * |        |          |1 = ACMP01 is a non-privileged module.
     * |[28]    |USBD      |Set USBD to Non-privileged State
     * |        |          |0 = USBD is a privileged module (default).
     * |        |          |1 = USBD is a non-privileged module.
     * @var SCU_T::D1PNP4
     * Offset: 0x2D0  Domain 1 Peripheral Privilege Attribution Set Register4 (0x4028_0000~0x4029_FFFF)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDT1     |Set WWDT1 to Non-privileged State
     * |        |          |0 = WWDT1 is a privileged module (default).
     * |        |          |1 = WWDT1 is a non-privileged module.
     * |[2]     |EPWM1     |Set EPWM1 to Non-privileged State
     * |        |          |0 = EPWM1 is a privileged module (default).
     * |        |          |1 = EPWM1 is a non-privileged module.
     * |[3]     |BPWM1     |Set BPWM1 to Non-privileged State
     * |        |          |0 = BPWM1 is a privileged module (default).
     * |        |          |1 = BPWM1 is a non-privileged module.
     * |[4]     |EQEI1     |Set EQEI1 to Non-privileged State
     * |        |          |0 = EQEI1 is a privileged module (default).
     * |        |          |1 = EQEI1 is a non-privileged module.
     * |[5]     |EQEI3     |Set EQEI3 to Non-privileged State
     * |        |          |0 = EQEI3 is a privileged module (default).
     * |        |          |1 = EQEI3 is a non-privileged module.
     * |[6]     |ECAP1     |Set ECAP1 to Non-privileged State
     * |        |          |0 = ECAP1 is a privileged module (default).
     * |        |          |1 = ECAP1 is a non-privileged module.
     * |[7]     |ECAP3     |Set ECAP3 to Non-privileged State
     * |        |          |0 = ECAP3 is a privileged module (default).
     * |        |          |1 = ECAP3 is a non-privileged module.
     * |[8]     |I2C1      |Set I2C1 to Non-privileged State
     * |        |          |0 = I2C1 is a privileged module (default).
     * |        |          |1 = I2C1 is a non-privileged module.
     * |[9]     |I2C3      |Set I2C3 to Non-privileged State
     * |        |          |0 = I2C3 is a privileged module (default).
     * |        |          |1 = I2C3 is a non-privileged module.
     * |[10]    |QSPI1     |Set QSPI1 to Non-privileged State
     * |        |          |0 = QSPI1 is a privileged module (default).
     * |        |          |1 = QSPI1 is a non-privileged module.
     * |[11]    |SPI1      |Set SPI1 to Non-privileged State
     * |        |          |0 = SPI1 is a privileged module (default).
     * |        |          |1 = SPI1 is a non-privileged module.
     * |[12]    |SPI3      |Set SPI3 to Non-privileged State
     * |        |          |0 = SPI3 is a privileged module (default).
     * |        |          |1 = SPI3 is a non-privileged module.
     * |[13]    |UART1     |Set UART1 to Non-privileged State
     * |        |          |0 = UART1 is a privileged module (default).
     * |        |          |1 = UART1 is a non-privileged module.
     * |[14]    |UART3     |Set UART3 to Non-privileged State
     * |        |          |0 = UART3 is a privileged module (default).
     * |        |          |1 = UART3 is a non-privileged module.
     * |[15]    |UART5     |Set UART5 to Non-privileged State
     * |        |          |0 = UART5 is a privileged module (default).
     * |        |          |1 = UART5 is a non-privileged module.
     * |[16]    |UART7     |Set UART7 to Non-privileged State
     * |        |          |0 = UART7 is a privileged module (default).
     * |        |          |1 = UART7 is a non-privileged module.
     * |[17]    |UART9     |Set UART9 to Non-privileged State
     * |        |          |0 = UART9 is a privileged module (default).
     * |        |          |1 = UART9 is a non-privileged module.
     * |[18]    |SC1       |Set SC1 to Non-privileged State
     * |        |          |0 = SC1 is a privileged module (default).
     * |        |          |1 = SC1 is a non-privileged module.
     * |[19]    |OTG       |Set OTG to Non-privileged State
     * |        |          |0 = OTG is a privileged module (default).
     * |        |          |1 = OTG is a non-privileged module.
     * |[20]    |KPI       |Set KPI to Non-privileged State
     * |        |          |0 = KPI is a privileged module (default).
     * |        |          |1 = KPI is a non-privileged module.
     * |[21]    |TMR23     |Set TMR23 to Non-privileged State
     * |        |          |0 = TMR23 is a privileged module (default).
     * |        |          |1 = TMR23 is a non-privileged module.
     * |[22]    |TRNG      |Set TRNG to Non-privileged State
     * |        |          |0 = TRNG is a privileged module (default).
     * |        |          |1 = TRNG is a non-privileged module.
     * |[23]    |RTC       |Set RTC to Non-privileged State
     * |        |          |0 = RTC is a privileged module (default).
     * |        |          |1 = RTC is a non-privileged module.
     * |[24]    |I2S1      |Set I2S1 to Non-privileged State
     * |        |          |0 = I2S1 is a privileged module (default).
     * |        |          |1 = I2S1 is a non-privileged module.
     * |[25]    |ACMP23    |Set ACMP23 to Non-privileged State
     * |        |          |0 = ACMP23 is a privileged module (default).
     * |        |          |1 = ACMP23 is a non-privileged module.
     * |[26]    |I3C0      |Set I3C0 to Non-privileged State
     * |        |          |0 = I3C0 is a privileged module (default).
     * |        |          |1 = I3C0 is a non-privileged module.
     * |[27]    |UTCPD     |Set UTCPD to Non-privileged State
     * |        |          |0 = UTCPD is a privileged module (default).
     * |        |          |1 = UTCPD is a non-privileged module.
     * @var SCU_T::D2PNP0
     * Offset: 0x2E0  Domain 2 Peripheral Privilege Attribution Set Register0 (0x4040_0000~0x4041_FFFF)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPPDMA    |Set LPPDMA to Non-privileged State
     * |        |          |0 = LPPDMA is a privileged module (default).
     * |        |          |1 = LPPDMA is a non-privileged module.
     * |[1]     |CCAP      |Set CCAP to Non-privileged State
     * |        |          |0 = CCAP is a privileged module (default).
     * |        |          |1 = CCAP is a non-privileged module.
     * |[3]     |LPGPIO    |Set LPGPIO to Non-privileged State
     * |        |          |0 = LPGPIO is a privileged module (default).
     * |        |          |1 = LPGPIO is a non-privileged module.
     * @var SCU_T::D2PNP2
     * Offset: 0x2E8  Domain 2 Peripheral Privilege Attribution Set Register2 (0x4044_0000~0x4045_FFFF)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPTMR01   |Set LPTMR01 to Non-privileged State
     * |        |          |0 = LPTMR01 is a privileged module (default).
     * |        |          |1 = LPTMR01 is a non-privileged module.
     * |[1]     |TTMR01    |Set TTMR01 to Non-privileged State
     * |        |          |0 = TTMR01 is a privileged module (default).
     * |        |          |1 = TTMR01 is a non-privileged module.
     * |[2]     |LPADC0    |Set LPADC0 to Non-privileged State
     * |        |          |0 = LPADC0 is a privileged module (default).
     * |        |          |1 = LPADC0 is a non-privileged module.
     * |[3]     |LPI2C0    |Set LPI2C0 to Non-privileged State
     * |        |          |0 = LPI2C0 is a privileged module (default).
     * |        |          |1 = LPI2C0 is a non-privileged module.
     * |[4]     |LPSPI0    |Set LPSPI0 to Non-privileged State
     * |        |          |0 = LPSPI0 is a privileged module (default).
     * |        |          |1 = LPSPI0 is a non-privileged module.
     * |[5]     |DMIC0     |Set DMIC0 to Non-privileged State
     * |        |          |0 = DMIC0 is a privileged module (default).
     * |        |          |1 = DMIC0 is a non-privileged module.
     * |[6]     |LPUART0   |Set LPUART0 to Non-privileged State
     * |        |          |0 = LPUART0 is a privileged module (default).
     * |        |          |1 = LPUART0 is a non-privileged module.
     * |[7]     |WDT0      |Set WDT0 to Non-privileged State
     * |        |          |0 = WDT0 is a privileged module (default).
     * |        |          |1 = WDT0 is a non-privileged module.
     * |[8]     |WDT1      |Set WDT1 to Non-privileged State
     * |        |          |0 = WDT1 is a privileged module (default).
     * |        |          |1 = WDT1 is a non-privileged module.
     * |[9]     |AWF       |Set AWF to Non-privileged State
     * |        |          |0 = AWF is a privileged module (default).
     * |        |          |1 = AWF is a non-privileged module.
     * |[16]    |LPSRAMMPC |Set LPSRAMMPC to Non-privileged State
     * |        |          |0 = LPSRAMMPC is a privileged module (default).
     * |        |          |1 = LPSRAMMPC is a non-privileged module.
     * @var SCU_T::IONP
     * Offset: 0x3A0  I/O Non-privileged Attribution Set Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PA        |Set GPIO Port a to Non-privileged State
     * |        |          |0 = GPIO port A is privileged (default).
     * |        |          |1 = GPIO port A is non-privileged.
     * |[1]     |PB        |Set GPIO Port B to Non-privileged State
     * |        |          |0 = GPIO port B is privileged (default).
     * |        |          |1 = GPIO port B is non-privileged.
     * |[2]     |PC        |Set GPIO Port C to Non-privileged State
     * |        |          |0 = GPIO port C is privileged (default).
     * |        |          |1 = GPIO port C is non-privileged.
     * |[3]     |PD        |Set GPIO Port D to Non-privileged State
     * |        |          |0 = GPIO port D is privileged (default).
     * |        |          |1 = GPIO port D is non-privileged.
     * |[4]     |PE        |Set GPIO Port E to Non-privileged State
     * |        |          |0 = GPIO port E is privileged (default).
     * |        |          |1 = GPIO port E is non-privileged.
     * |[5]     |PF        |Set GPIO Port F to Non-privileged State
     * |        |          |0 = GPIO port F is privileged (default).
     * |        |          |1 = GPIO port F is non-privileged.
     * |[6]     |PG        |Set GPIO Port G to Non-privileged State
     * |        |          |0 = GPIO port G is privileged (default).
     * |        |          |1 = GPIO port G is non-privileged.
     * |[7]     |PH        |Set GPIO Port H to Non-privileged State
     * |        |          |0 = GPIO port H is privileged (default).
     * |        |          |1 = GPIO port H is non-privileged.
     * |[8]     |PI        |Set GPIO Port I to Non-privileged State
     * |        |          |0 = GPIO port I is privileged (default).
     * |        |          |1 = GPIO port I is non-privileged.
     * |[9]     |PJ        |Set GPIO Port J to Non-privileged State
     * |        |          |0 = GPIO port J is privileged (default).
     * |        |          |1 = GPIO port J is non-privileged.
     * @var SCU_T::SINFAEN
     * Offset: 0x3C0  Shared Information Access Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SCUSIAEN  |SCU Shared Information Access Enable Bit
     * |        |          |0 = Non-secure CPU access SCU Shared information Disabled.
     * |        |          |1 = Non-secure CPU access SCU Shared information Enabled.
     * |[1]     |SYSSIAEN  |SYS Shared Information Access Enable Bit
     * |        |          |0 = Non-secure CPU access SYS Shared information Disabled.
     * |        |          |1 = Non-secure CPU access SYS Shared information Enabled.
     * |[2]     |FMCSIAEN  |FMC Shared Information Access Enable Bit
     * |        |          |0 = Non-secure CPU access FMC Shared information Disabled.
     * |        |          |1 = Non-secure CPU access FMC Shared information Enabled.
     * |[3]     |CLKSIAEN  |CLKCTL Shared Information Access Enable Bit
     * |        |          |0 = Non-secure CPU access CLKCTL Shared information Disabled.
     * |        |          |1 = Non-secure CPU access CLKCTL Shared information Enabled.
     * @var SCU_T::SCWP
     * Offset: 0x3C4  Security Configuration Write Protection Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |LOCK      |Enable Write Protection Lock Bit (Write Once)
     * |        |          |0 = Write protection lock Disabled.
     * |        |          |1 = Write protection Enabled and locked.
     * |        |          |Note: This bit cannot be cleared to 0 without a System-level reset after set to one.
     * |[31:16] |WVCODE    |Write Verify Code
     * |        |          |Read operation:
     * |        |          |Reserved, all zeros.
     * |        |          |Write operation:
     * |        |          |0x475A = The write verify code, 0x475A, is needed to do a valid write to SCU_SCWP.
     * |        |          |Others = Invalid write verify code.
     * @var SCU_T::NSMCTL
     * Offset: 0x400  Non-secure State Monitor Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PRESCALE  |Pre-scale Value of Non-secure State Monitor Counter
     * |        |          |0 = Counter Disabled.
     * |        |          |Others = Counter Enabled and the counter clock source = HCLK/PRESCALE.
     * |[8]     |NSMIEN    |Non-secure State Monitor Interrupt Enable Bit
     * |        |          |0 = Non-secure state monitor interrupt Disabled.
     * |        |          |1 = Non-secure state monitor interrupt Enabled.
     * |[9]     |AUTORLD   |Auto Reload Non-secure State Monitor Counter When CURRNS Changing to 1
     * |        |          |0 = Disable clearing non-secure state monitor counter automatically (default).
     * |        |          |1 = Enable clearing non-secure state monitor counter automatically when the core processor changes from secure state to non-secure state
     * |[10]    |TMRMOD    |Non-secure Monitor Mode Enable Bit
     * |        |          |0 = Monitor mode. The counter will count down when the core processor is in non-secure state. (default)
     * |        |          |1 = Free-counting mode
     * |        |          |The counter will keep counting no matter the core processor is in secure or non-secure state.
     * |[12]    |IDLEON    |Monitor Counter Keep Counting When the Chip Is in Idle Mode Enable Bit
     * |        |          |0 = The counter will be halted when the chip is in idle mode.
     * |        |          |1 = The counter will keep counting when the chip is in idle mode. (default)
     * |        |          |Note: In monitor mode, the counter is always halted when the core processor is in secure state.
     * |[13]    |DBGON     |Monitor Counter Keep Counting When the Chip Is in Debug Mode Enable Bit
     * |        |          |0 = The counter will be halted when the core processor is halted by ICE. (default)
     * |        |          |1 = The counter will keep counting when the core processor is halted by ICE.
     * @var SCU_T::NSMLOAD
     * Offset: 0x404  Non-secure State Monitor Reload Value Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |RELOAD    |Reload Value for Non-secure State Monitor Counter
     * |        |          |The RELOAD value will be reloaded to the counter whenever the counter counts down to 0.
     * @var SCU_T::NSMVAL
     * Offset: 0x408  Non-secure State Monitor Counter Value Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |VALUE     |Counter Value of Non-secure State Monitor Counter
     * |        |          |Current value of non-secure state monitor counter
     * |        |          |This is down counter and counts down only when CURRNS = 1
     * |        |          |When counting down to 0, VALUE will automatically be reloaded from NSMLOAD register.
     * |        |          |A write of any value clears the VALUE to 0 and also clears NSMIF.
     * @var SCU_T::NSMSTS
     * Offset: 0x40C  Non-secure State Monitor Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CURRNS    |Current Core Processor Secure/Non-secure State (Read Only)
     * |        |          |0 = Core processor is in secure state.
     * |        |          |1 = Core processor is in non-secure state.
     * |        |          |Note: This bit can be used to monitor the current secure/non-secure state of the core processor, even if the non-secure state monitor counter is disabled.
     * |[1]     |NSMIF     |Non-secure State Monitor Interrupt Flag
     * |        |          |0 = Counter does not count down to 0 since the last NSMIF has been cleared.
     * |        |          |1 = Counter counts down to 0.
     * |        |          |Note: This bit is cleared by writing 1.
     * @var SCU_T::SRAM0MPCLUT0
     * Offset: 0x410  Security Attribute of memory blocks controlled by SRAM0MPC LUT0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |BLK_LUT   |Block-based gating Look Up Table
     * |        |          |Each bit controls the Security state of a single block. When:
     * |        |          |l Bit[n] = 0, the block is in the Secure state.
     * |        |          |l Bit[n] = 1, the block is in the Non-secure state..
     * @var SCU_T::SRAM1MPCLUT0
     * Offset: 0x414  Security Attribute of memory blocks controlled by SRAM1MPC LUT0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |BLK_LUT   |Block-based gating Look Up Table
     * |        |          |Each bit controls the Security state of a single block. When:
     * |        |          |l Bit[n] = 0, the block is in the Secure state.
     * |        |          |l Bit[n] = 1, the block is in the Non-secure state..
     * @var SCU_T::SRAM2MPCLUT0
     * Offset: 0x418  Security Attribute of memory blocks controlled by SRAM2MPC LUT0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |BLK_LUT   |Block-based gating Look Up Table
     * |        |          |Each bit controls the Security state of a single block. When:
     * |        |          |l Bit[n] = 0, the block is in the Secure state.
     * |        |          |l Bit[n] = 1, the block is in the Non-secure state..
     * @var SCU_T::SRAM3MPCLUT0
     * Offset: 0x420  Security Attribute of memory blocks controlled by SRAM3MPC LUT0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |BLK_LUT   |Block-based gating Look Up Table
     * |        |          |Each bit controls the Security state of a single block. When:
     * |        |          |l Bit[n] = 0, the block is in the Secure state.
     * |        |          |l Bit[n] = 1, the block is in the Non-secure state..
     * @var SCU_T::LPSRAMMPCLUT0
     * Offset: 0x424  Security Attribute of memory blocks controlled by LPSRAMMPC LUT0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |BLK_LUT   |Block-based gating Look Up Table
     * |        |          |Each bit controls the Security state of a single block. When:
     * |        |          |l Bit[n] = 0, the block is in the Secure state.
     * |        |          |l Bit[n] = 1, the block is in the Non-secure state..
     * @var SCU_T::SPIM0MPCLUT[8]
     * Offset: 0x430 ~ 0x44C  Security Attribute of memory blocks controlled by SPIM0MPC LUT0 ~ 7
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |BLK_LUT   |Block-based gating Look Up Table
     * |        |          |Each bit controls the Security state of a single block. When:
     * |        |          |l Bit[n] = 0, the block is in the Secure state.
     * |        |          |l Bit[n] = 1, the block is in the Non-secure state..
     */
    union
    {
        __IO uint32_t DxPNSy[19];                /*!< [0x0000-0x0048] Peripheral Non-secure Attribution Set Register                              */
        struct
        {
            __IO uint32_t D0PNS0;                /*!< [0x0000] Domain 0 Peripheral Non-secure Attribution Set Register0 (0x4000_0000~0x4001_FFFF) */
            __I  uint32_t RESERVE0[1];
            __IO uint32_t D0PNS2;                /*!< [0x0008] Domain 0 Peripheral Non-secure Attribution Set Register2 (0x4004_0000~0x4005_FFFF) */
            __I  uint32_t RESERVE1[5];
            __IO uint32_t D1PNS0;                /*!< [0x0020] Domain 1 Peripheral Non-secure Attribution Set Register0 (0x4020_0000~0x4021_FFFF) */
            __IO uint32_t D1PNS1;                /*!< [0x0024] Domain 0 Peripheral Non-secure Attribution Set Register1 (0x4022_0000~0x4023_FFFF) */
            __IO uint32_t D1PNS2;                /*!< [0x0028] Domain 1 Peripheral Non-secure Attribution Set Register2 (0x4024_0000~0x4025_FFFF) */
            __I  uint32_t RESERVE2[1];
            __IO uint32_t D1PNS4;                /*!< [0x0030] Domain 1 Peripheral Non-secure Attribution Set Register4 (0x4028_0000~0x4029_FFFF) */
            __I  uint32_t RESERVE3[3];
            __IO uint32_t D2PNS0;                /*!< [0x0040] Domain 2 Peripheral Non-secure Attribution Set Register0 (0x4040_0000~0x4041_FFFF) */
            __I  uint32_t RESERVE4[1];
            __IO uint32_t D2PNS2;                /*!< [0x0048] Domain 2 Peripheral Non-secure Attribution Set Register2 (0x4044_0000~0x4045_FFFF) */
        };
    };
    __I  uint32_t RESERVE5[5];
    __I  uint32_t FNSADDR;               /*!< [0x0060] Flash Non-secure Boundary Address Register                       */
    __IO uint32_t EINTNS;                /*!< [0x0064] EINT Non-secure Attribution Set Register                         */
    __I  uint32_t RESERVE6[2];
    __IO uint32_t IONS[10];              /*!< [0x0070-0x0094] I/O Non-secure Attribution Set Register0-9                */
    __I  uint32_t RESERVE7[26];
    union
    {
        __IO uint32_t SVIEN[2];
        struct
        {
            __IO uint32_t SVIEN0;                /*!< [0x0100] Security Violation Interrupt Enable Register - Slave peripherals        */
            __IO uint32_t SVIEN1;                /*!< [0x0104] Security Violation Interrupt Enable Register - Master peripherals (MSC) */
        };
    };
    __I  uint32_t RESERVE8[2];
    union
    {
        __IO uint32_t SVINTSTS[3];
        struct
        {
            __IO uint32_t SVINTSTS0;             /*!< [0x0110] Security Violation Interrupt Status Register - Slave peripherals        */
            __IO uint32_t SVINTSTS1;             /*!< [0x0114] Security Violation Interrupt Status Register - Master peripherals (MSC) */
            __I  uint32_t SVINTSTS2;             /*!< [0x0118] Security Violation Interrupt Status Register - Memory peripherals (MPC) */
        };
    };
    __I  uint32_t RESERVE9[1];
    union
    {
        __I  uint32_t PVSRC[32];                 /*!< [0x0120-0x019f] Security Policy Violation Source                          */
        struct
        {
            __I  uint32_t APB0PVSRC;             /*!< [0x0120] APB0 Security Policy Violation Source                            */
            __I  uint32_t APB1PVSRC;             /*!< [0x0124] APB1 Security Policy Violation Source                            */
            __I  uint32_t APB2PVSRC;             /*!< [0x0128] APB2 Security Policy Violation Source                            */
            __I  uint32_t APB3PVSRC;             /*!< [0x012c] APB3 Security Policy Violation Source                            */
            __I  uint32_t APB4PVSRC;             /*!< [0x0130] APB4 Security Policy Violation Source                            */
            __I  uint32_t APB5PVSRC;             /*!< [0x0134] APB5 Security Policy Violation Source                            */
            __I  uint32_t RESERVE10[2];
            __I  uint32_t D0PPC0PVSRC;           /*!< [0x0140] D0PPC0 Security Policy Violation Source                          */
            __I  uint32_t D1PPC0PVSRC;           /*!< [0x0144] D1PPC0 Security Policy Violation Source                          */
            __I  uint32_t D1PPC1PVSRC;           /*!< [0x0148] D1PPC1 Security Policy Violation Source                          */
            __I  uint32_t D2PPC0PVSRC;           /*!< [0x014c] D2PPC0 Security Policy Violation Source                          */
            __I  uint32_t RESERVE11[4];
            __I  uint32_t EBIPVSRC;              /*!< [0x0160] EBI Security Policy Violation Source                             */
            __I  uint32_t FLASHPVSRC;            /*!< [0x0164] FLASH Security Policy Violation Source                           */
            __I  uint32_t RESERVE12[14];
        };
    };
    union
    {
        __I  uint32_t PVA[32];                   /*!< [0x01a0-0x021f] Peripheral Violation Address                              */
        struct
        {
            __I  uint32_t APB0PVA;               /*!< [0x01a0] APB0 Violation Address                                           */
            __I  uint32_t APB1PVA;               /*!< [0x01a4] APB1 Violation Address                                           */
            __I  uint32_t APB2PVA;               /*!< [0x01a8] APB2 Violation Address                                           */
            __I  uint32_t APB3PVA;               /*!< [0x01ac] APB3 Violation Address                                           */
            __I  uint32_t APB4PVA;               /*!< [0x01b0] APB4 Violation Address                                           */
            __I  uint32_t APB5PVA;               /*!< [0x01b4] APB5 Violation Address                                           */
            __I  uint32_t RESERVE13[2];
            __I  uint32_t D0PPC0PVA;             /*!< [0x01c0] D0PPC0 Violation Address                                         */
            __I  uint32_t D1PPC0PVA;             /*!< [0x01c4] D1PPC0 Violation Address                                         */
            __I  uint32_t D1PPC1PVA;             /*!< [0x01c8] D1PPC1 Violation Address                                         */
            __I  uint32_t D2PPC0PVA;             /*!< [0x01cc] D2PPC0 Violation Address                                         */
            __I  uint32_t RESERVE14[4];
            __I  uint32_t EBIPVA;                /*!< [0x01e0] EBI Violation Address                                            */
            __I  uint32_t FLASHPVA;              /*!< [0x01e4] FLASH Violation Address                                          */
            __I  uint32_t RESERVE15[14];
        };
    };
    union
    {
        __I  uint32_t MVA[32];                   /*!< [0x0220-0x029F] Master Violation Address                                  */
        struct
        {
            __I  uint32_t GDMAMVA;               /*!< [0x0220] GDMA Violation Address                                           */
            __I  uint32_t PDMA0MVA;              /*!< [0x0224] PDMA Violation Address                                           */
            __I  uint32_t PDMA1MVA;              /*!< [0x0228] PDMA1 Violation Address                                          */
            __I  uint32_t USBH0MVA;              /*!< [0x022c] USBH0 Violation Address                                          */
            __I  uint32_t RESERVE16[1];
            __I  uint32_t HSUSBHMVA;             /*!< [0x0234] HSUSBH Violation Address                                         */
            __I  uint32_t HSUSBDMVA;             /*!< [0x0238] HSUSBD Violation Address                                         */
            __I  uint32_t SDH0MVA;               /*!< [0x023c] SDH Violation Address                                            */
            __I  uint32_t SDH1MVA;               /*!< [0x0240] SDH1 Violation Address                                           */
            __I  uint32_t EMACMVA;               /*!< [0x0244] EMAC Violation Address                                           */
            __I  uint32_t CRYPTOMVA;             /*!< [0x0248] CRYPTO Violation Address                                         */
            __I  uint32_t CRCMVA;                /*!< [0x024c] CRC Violation Address                                            */
            __I  uint32_t LPPDMAMVA;             /*!< [0x0250] LPPDMA Violation Address                                         */
            __I  uint32_t CCAPMVA;               /*!< [0x0254] CCAP Violation Address                                           */
            __I  uint32_t NPUIF1MVA;             /*!< [0x0258] NPU Interface 1 Violation Address                                */
            __I  uint32_t NPUIF0MVA;             /*!< [0x025c] NPU Interface 0 Violation Address                                */
            __I  uint32_t SPIM0MVA;              /*!< [0x0260] SPIM 0 Master Violation Address                                  */
            __I  uint32_t RESERVE17[15];
        };
    };
    union
    {
        __IO uint32_t DxPNPy[19];                /*!< [0x02a0-0x02e8] Peripheral Privilege Attribution Set Registers                             */
        struct
        {
            __IO uint32_t D0PNP0;                /*!< [0x02a0] Domain 0 Peripheral Privilege Attribution Set Register0 (0x4000_0000~0x4001_FFFF) */
            __I  uint32_t RESERVE18[1];
            __IO uint32_t D0PNP2;                /*!< [0x02a8] Domain 0 Peripheral Privilege Attribution Set Register2 (0x4004_0000~0x4005_FFFF) */
            __I  uint32_t RESERVE19[5];
            __IO uint32_t D1PNP0;                /*!< [0x02c0] Domain 1 Peripheral Privilege Attribution Set Register0 (0x4020_0000~0x4021_FFFF) */
            __IO uint32_t D1PNP1;                /*!< [0x02c4] Domain 0 Peripheral Privilege Attribution Set Register1 (0x4022_0000~0x4023_FFFF) */
            __IO uint32_t D1PNP2;                /*!< [0x02c8] Domain 1 Peripheral Privilege Attribution Set Register2 (0x4024_0000~0x4025_FFFF) */
            __I  uint32_t RESERVE20[1];
            __IO uint32_t D1PNP4;                /*!< [0x02d0] Domain 1 Peripheral Privilege Attribution Set Register4 (0x4028_0000~0x4029_FFFF) */
            __I  uint32_t RESERVE21[3];
            __IO uint32_t D2PNP0;                /*!< [0x02e0] Domain 2 Peripheral Privilege Attribution Set Register0 (0x4040_0000~0x4041_FFFF) */
            __I  uint32_t RESERVE22[1];
            __IO uint32_t D2PNP2;                /*!< [0x02e8] Domain 2 Peripheral Privilege Attribution Set Register2 (0x4044_0000~0x4045_FFFF) */
        };
    };
    __I  uint32_t RESERVE23[45];
    __IO uint32_t IONP;                  /*!< [0x03a0] I/O Non-privileged Attribution Set Register                      */
    __I  uint32_t RESERVE24[7];
    __IO uint32_t SINFAEN;               /*!< [0x03c0] Shared Information Access Enable Register                        */
    __IO uint32_t SCWP;                  /*!< [0x03c4] Security Configuration Write Protection Register                 */
    __I  uint32_t RESERVE25[14];
    __IO uint32_t NSMCTL;                /*!< [0x0400] Non-secure State Monitor Control Register                        */
    __IO uint32_t NSMLOAD;               /*!< [0x0404] Non-secure State Monitor Reload Value Register                   */
    __IO uint32_t NSMVAL;                /*!< [0x0408] Non-secure State Monitor Counter Value Register                  */
    __IO uint32_t NSMSTS;                /*!< [0x040c] Non-secure State Monitor Status Register                         */
    __I  uint32_t SRAM0MPCLUT0;          /*!< [0x0410] Security Attribute of memory blocks controlled by SRAM0MPC LUT0  */
    __I  uint32_t SRAM1MPCLUT0;          /*!< [0x0414] Security Attribute of memory blocks controlled by SRAM1MPC LUT0  */
    __I  uint32_t SRAM2MPCLUT0;          /*!< [0x0418] Security Attribute of memory blocks controlled by SRAM2MPC LUT0  */
    __I  uint32_t RESERVE26[1];
    __I  uint32_t SRAM3MPCLUT0;          /*!< [0x0420] Security Attribute of memory blocks controlled by SRAM3MPC LUT0  */
    __I  uint32_t LPSRAMMPCLUT0;         /*!< [0x0424] Security Attribute of memory blocks controlled by LPSRAMMPC LUT0 */
    __I  uint32_t RESERVE27[2];
    __I  uint32_t SPIM0MPCLUT[8];        /*!< [0x0430-0x044c] Security Attribute of memory blocks controlled by SPIM0MPC LUT0-7  */
} SCU_T;

/**
    @addtogroup SCU_CONST SCU Bit Field Definition
    Constant Definitions for SCU Controller
    @{
*/

#define SCU_D0PNS0_NPU_Pos               (3)                                               /*!< SCU_T::D0PNS0: NPU Position            */
#define SCU_D0PNS0_NPU_Msk               (0x1ul << SCU_D0PNS0_NPU_Pos)                     /*!< SCU_T::D0PNS0: NPU Mask                */

#define SCU_D0PNS2_SPIM0_Pos             (2)                                               /*!< SCU_T::D0PNS2: SPIM0 Position          */
#define SCU_D0PNS2_SPIM0_Msk             (0x1ul << SCU_D0PNS2_SPIM0_Pos)                   /*!< SCU_T::D0PNS2: SPIM0 Mask              */

#define SCU_D1PNS0_PDMA0_Pos             (0)                                               /*!< SCU_T::D1PNS0: PDMA0 Position          */
#define SCU_D1PNS0_PDMA0_Msk             (0x1ul << SCU_D1PNS0_PDMA0_Pos)                   /*!< SCU_T::D1PNS0: PDMA0 Mask              */

#define SCU_D1PNS0_PDMA1_Pos             (1)                                               /*!< SCU_T::D1PNS0: PDMA1 Position          */
#define SCU_D1PNS0_PDMA1_Msk             (0x1ul << SCU_D1PNS0_PDMA1_Pos)                   /*!< SCU_T::D1PNS0: PDMA1 Mask              */

#define SCU_D1PNS0_USBH0_Pos             (2)                                               /*!< SCU_T::D1PNS0: USBH0 Position          */
#define SCU_D1PNS0_USBH0_Msk             (0x1ul << SCU_D1PNS0_USBH0_Pos)                   /*!< SCU_T::D1PNS0: USBH0 Mask              */

#define SCU_D1PNS0_HSUSBH_Pos            (4)                                               /*!< SCU_T::D1PNS0: HSUSBH Position         */
#define SCU_D1PNS0_HSUSBH_Msk            (0x1ul << SCU_D1PNS0_HSUSBH_Pos)                  /*!< SCU_T::D1PNS0: HSUSBH Mask             */

#define SCU_D1PNS0_HSUSBD_Pos            (5)                                               /*!< SCU_T::D1PNS0: HSUSBD Position         */
#define SCU_D1PNS0_HSUSBD_Msk            (0x1ul << SCU_D1PNS0_HSUSBD_Pos)                  /*!< SCU_T::D1PNS0: HSUSBD Mask             */

#define SCU_D1PNS0_SDH0_Pos              (6)                                               /*!< SCU_T::D1PNS0: SDH0 Position           */
#define SCU_D1PNS0_SDH0_Msk              (0x1ul << SCU_D1PNS0_SDH0_Pos)                    /*!< SCU_T::D1PNS0: SDH0 Mask               */

#define SCU_D1PNS0_SDH1_Pos              (7)                                               /*!< SCU_T::D1PNS0: SDH1 Position           */
#define SCU_D1PNS0_SDH1_Msk              (0x1ul << SCU_D1PNS0_SDH1_Pos)                    /*!< SCU_T::D1PNS0: SDH1 Mask               */

#define SCU_D1PNS0_EMAC0_Pos             (8)                                               /*!< SCU_T::D1PNS0: EMAC0 Position          */
#define SCU_D1PNS0_EMAC0_Msk             (0x1ul << SCU_D1PNS0_EMAC0_Pos)                   /*!< SCU_T::D1PNS0: EMAC0 Mask              */

#define SCU_D1PNS0_CRYPTO_Pos            (10)                                              /*!< SCU_T::D1PNS0: CRYPTO Position         */
#define SCU_D1PNS0_CRYPTO_Msk            (0x1ul << SCU_D1PNS0_CRYPTO_Pos)                  /*!< SCU_T::D1PNS0: CRYPTO Mask             */

#define SCU_D1PNS0_CRC_Pos               (11)                                              /*!< SCU_T::D1PNS0: CRC Position            */
#define SCU_D1PNS0_CRC_Msk               (0x1ul << SCU_D1PNS0_CRC_Pos)                     /*!< SCU_T::D1PNS0: CRC Mask                */

#define SCU_D1PNS1_KDF_Pos               (1)                                               /*!< SCU_T::D1PNS1: KDF Position            */
#define SCU_D1PNS1_KDF_Msk               (0x1ul << SCU_D1PNS1_KDF_Pos)                     /*!< SCU_T::D1PNS1: KDF Mask                */

#define SCU_D1PNS1_CANFD0_Pos            (2)                                               /*!< SCU_T::D1PNS1: CANFD0 Position         */
#define SCU_D1PNS1_CANFD0_Msk            (0x1ul << SCU_D1PNS1_CANFD0_Pos)                  /*!< SCU_T::D1PNS1: CANFD0 Mask             */

#define SCU_D1PNS1_CANFD1_Pos            (4)                                               /*!< SCU_T::D1PNS1: CANFD1 Position         */
#define SCU_D1PNS1_CANFD1_Msk            (0x1ul << SCU_D1PNS1_CANFD1_Pos)                  /*!< SCU_T::D1PNS1: CANFD1 Mask             */

#define SCU_D1PNS1_EBI_Pos               (16)                                              /*!< SCU_T::D1PNS1: EBI Position            */
#define SCU_D1PNS1_EBI_Msk               (0x1ul << SCU_D1PNS1_EBI_Pos)                     /*!< SCU_T::D1PNS1: EBI Mask                */

#define SCU_D1PNS2_WWDT0_Pos             (0)                                               /*!< SCU_T::D1PNS2: WWDT0 Position          */
#define SCU_D1PNS2_WWDT0_Msk             (0x1ul << SCU_D1PNS2_WWDT0_Pos)                   /*!< SCU_T::D1PNS2: WWDT0 Mask              */

#define SCU_D1PNS2_EADC0_Pos             (1)                                               /*!< SCU_T::D1PNS2: EADC0 Position          */
#define SCU_D1PNS2_EADC0_Msk             (0x1ul << SCU_D1PNS2_EADC0_Pos)                   /*!< SCU_T::D1PNS2: EADC0 Mask              */

#define SCU_D1PNS2_EPWM0_Pos             (2)                                               /*!< SCU_T::D1PNS2: EPWM0 Position          */
#define SCU_D1PNS2_EPWM0_Msk             (0x1ul << SCU_D1PNS2_EPWM0_Pos)                   /*!< SCU_T::D1PNS2: EPWM0 Mask              */

#define SCU_D1PNS2_BPWM0_Pos             (3)                                               /*!< SCU_T::D1PNS2: BPWM0 Position          */
#define SCU_D1PNS2_BPWM0_Msk             (0x1ul << SCU_D1PNS2_BPWM0_Pos)                   /*!< SCU_T::D1PNS2: BPWM0 Mask              */

#define SCU_D1PNS2_EQEI0_Pos             (4)                                               /*!< SCU_T::D1PNS2: EQEI0 Position          */
#define SCU_D1PNS2_EQEI0_Msk             (0x1ul << SCU_D1PNS2_EQEI0_Pos)                   /*!< SCU_T::D1PNS2: EQEI0 Mask              */

#define SCU_D1PNS2_EQEI2_Pos             (5)                                               /*!< SCU_T::D1PNS2: EQEI2 Position          */
#define SCU_D1PNS2_EQEI2_Msk             (0x1ul << SCU_D1PNS2_EQEI2_Pos)                   /*!< SCU_T::D1PNS2: EQEI2 Mask              */

#define SCU_D1PNS2_ECAP0_Pos             (6)                                               /*!< SCU_T::D1PNS2: ECAP0 Position          */
#define SCU_D1PNS2_ECAP0_Msk             (0x1ul << SCU_D1PNS2_ECAP0_Pos)                   /*!< SCU_T::D1PNS2: ECAP0 Mask              */

#define SCU_D1PNS2_ECAP2_Pos             (7)                                               /*!< SCU_T::D1PNS2: ECAP2 Position          */
#define SCU_D1PNS2_ECAP2_Msk             (0x1ul << SCU_D1PNS2_ECAP2_Pos)                   /*!< SCU_T::D1PNS2: ECAP2 Mask              */

#define SCU_D1PNS2_I2C0_Pos              (8)                                               /*!< SCU_T::D1PNS2: I2C0 Position           */
#define SCU_D1PNS2_I2C0_Msk              (0x1ul << SCU_D1PNS2_I2C0_Pos)                    /*!< SCU_T::D1PNS2: I2C0 Mask               */

#define SCU_D1PNS2_I2C2_Pos              (9)                                               /*!< SCU_T::D1PNS2: I2C2 Position           */
#define SCU_D1PNS2_I2C2_Msk              (0x1ul << SCU_D1PNS2_I2C2_Pos)                    /*!< SCU_T::D1PNS2: I2C2 Mask               */

#define SCU_D1PNS2_QSPI0_Pos             (10)                                              /*!< SCU_T::D1PNS2: QSPI0 Position          */
#define SCU_D1PNS2_QSPI0_Msk             (0x1ul << SCU_D1PNS2_QSPI0_Pos)                   /*!< SCU_T::D1PNS2: QSPI0 Mask              */

#define SCU_D1PNS2_SPI0_Pos              (11)                                              /*!< SCU_T::D1PNS2: SPI0 Position           */
#define SCU_D1PNS2_SPI0_Msk              (0x1ul << SCU_D1PNS2_SPI0_Pos)                    /*!< SCU_T::D1PNS2: SPI0 Mask               */

#define SCU_D1PNS2_SPI2_Pos              (12)                                              /*!< SCU_T::D1PNS2: SPI2 Position           */
#define SCU_D1PNS2_SPI2_Msk              (0x1ul << SCU_D1PNS2_SPI2_Pos)                    /*!< SCU_T::D1PNS2: SPI2 Mask               */

#define SCU_D1PNS2_UART0_Pos             (13)                                              /*!< SCU_T::D1PNS2: UART0 Position          */
#define SCU_D1PNS2_UART0_Msk             (0x1ul << SCU_D1PNS2_UART0_Pos)                   /*!< SCU_T::D1PNS2: UART0 Mask              */

#define SCU_D1PNS2_UART2_Pos             (14)                                              /*!< SCU_T::D1PNS2: UART2 Position          */
#define SCU_D1PNS2_UART2_Msk             (0x1ul << SCU_D1PNS2_UART2_Pos)                   /*!< SCU_T::D1PNS2: UART2 Mask              */

#define SCU_D1PNS2_UART4_Pos             (15)                                              /*!< SCU_T::D1PNS2: UART4 Position          */
#define SCU_D1PNS2_UART4_Msk             (0x1ul << SCU_D1PNS2_UART4_Pos)                   /*!< SCU_T::D1PNS2: UART4 Mask              */

#define SCU_D1PNS2_UART6_Pos             (16)                                              /*!< SCU_T::D1PNS2: UART6 Position          */
#define SCU_D1PNS2_UART6_Msk             (0x1ul << SCU_D1PNS2_UART6_Pos)                   /*!< SCU_T::D1PNS2: UART6 Mask              */

#define SCU_D1PNS2_UART8_Pos             (17)                                              /*!< SCU_T::D1PNS2: UART8 Position          */
#define SCU_D1PNS2_UART8_Msk             (0x1ul << SCU_D1PNS2_UART8_Pos)                   /*!< SCU_T::D1PNS2: UART8 Mask              */

#define SCU_D1PNS2_USCI0_Pos             (18)                                              /*!< SCU_T::D1PNS2: USCI0 Position          */
#define SCU_D1PNS2_USCI0_Msk             (0x1ul << SCU_D1PNS2_USCI0_Pos)                   /*!< SCU_T::D1PNS2: USCI0 Mask              */

#define SCU_D1PNS2_SC0_Pos               (19)                                              /*!< SCU_T::D1PNS2: SC0 Position            */
#define SCU_D1PNS2_SC0_Msk               (0x1ul << SCU_D1PNS2_SC0_Pos)                     /*!< SCU_T::D1PNS2: SC0 Mask                */

#define SCU_D1PNS2_SC2_Pos               (20)                                              /*!< SCU_T::D1PNS2: SC2 Position            */
#define SCU_D1PNS2_SC2_Msk               (0x1ul << SCU_D1PNS2_SC2_Pos)                     /*!< SCU_T::D1PNS2: SC2 Mask                */

#define SCU_D1PNS2_PSIO_Pos              (21)                                              /*!< SCU_T::D1PNS2: PSIO Position           */
#define SCU_D1PNS2_PSIO_Msk              (0x1ul << SCU_D1PNS2_PSIO_Pos)                    /*!< SCU_T::D1PNS2: PSIO Mask               */

#define SCU_D1PNS2_TMR01_Pos             (22)                                              /*!< SCU_T::D1PNS2: TMR01 Position          */
#define SCU_D1PNS2_TMR01_Msk             (0x1ul << SCU_D1PNS2_TMR01_Pos)                   /*!< SCU_T::D1PNS2: TMR01 Mask              */

#define SCU_D1PNS2_DAC01_Pos             (23)                                              /*!< SCU_T::D1PNS2: DAC01 Position          */
#define SCU_D1PNS2_DAC01_Msk             (0x1ul << SCU_D1PNS2_DAC01_Pos)                   /*!< SCU_T::D1PNS2: DAC01 Mask              */

#define SCU_D1PNS2_HSOTG_Pos             (25)                                              /*!< SCU_T::D1PNS2: HSOTG Position          */
#define SCU_D1PNS2_HSOTG_Msk             (0x1ul << SCU_D1PNS2_HSOTG_Pos)                   /*!< SCU_T::D1PNS2: HSOTG Mask              */

#define SCU_D1PNS2_I2S0_Pos              (26)                                              /*!< SCU_T::D1PNS2: I2S0 Position           */
#define SCU_D1PNS2_I2S0_Msk              (0x1ul << SCU_D1PNS2_I2S0_Pos)                    /*!< SCU_T::D1PNS2: I2S0 Mask               */

#define SCU_D1PNS2_ACMP01_Pos            (27)                                              /*!< SCU_T::D1PNS2: ACMP01 Position         */
#define SCU_D1PNS2_ACMP01_Msk            (0x1ul << SCU_D1PNS2_ACMP01_Pos)                  /*!< SCU_T::D1PNS2: ACMP01 Mask             */

#define SCU_D1PNS2_USBD_Pos              (28)                                              /*!< SCU_T::D1PNS2: USBD Position           */
#define SCU_D1PNS2_USBD_Msk              (0x1ul << SCU_D1PNS2_USBD_Pos)                    /*!< SCU_T::D1PNS2: USBD Mask               */

#define SCU_D1PNS4_WWDT1_Pos             (0)                                               /*!< SCU_T::D1PNS4: WWDT1 Position          */
#define SCU_D1PNS4_WWDT1_Msk             (0x1ul << SCU_D1PNS4_WWDT1_Pos)                   /*!< SCU_T::D1PNS4: WWDT1 Mask              */

#define SCU_D1PNS4_EPWM1_Pos             (2)                                               /*!< SCU_T::D1PNS4: EPWM1 Position          */
#define SCU_D1PNS4_EPWM1_Msk             (0x1ul << SCU_D1PNS4_EPWM1_Pos)                   /*!< SCU_T::D1PNS4: EPWM1 Mask              */

#define SCU_D1PNS4_BPWM1_Pos             (3)                                               /*!< SCU_T::D1PNS4: BPWM1 Position          */
#define SCU_D1PNS4_BPWM1_Msk             (0x1ul << SCU_D1PNS4_BPWM1_Pos)                   /*!< SCU_T::D1PNS4: BPWM1 Mask              */

#define SCU_D1PNS4_EQEI1_Pos             (4)                                               /*!< SCU_T::D1PNS4: EQEI1 Position          */
#define SCU_D1PNS4_EQEI1_Msk             (0x1ul << SCU_D1PNS4_EQEI1_Pos)                   /*!< SCU_T::D1PNS4: EQEI1 Mask              */

#define SCU_D1PNS4_EQEI3_Pos             (5)                                               /*!< SCU_T::D1PNS4: EQEI3 Position          */
#define SCU_D1PNS4_EQEI3_Msk             (0x1ul << SCU_D1PNS4_EQEI3_Pos)                   /*!< SCU_T::D1PNS4: EQEI3 Mask              */

#define SCU_D1PNS4_ECAP1_Pos             (6)                                               /*!< SCU_T::D1PNS4: ECAP1 Position          */
#define SCU_D1PNS4_ECAP1_Msk             (0x1ul << SCU_D1PNS4_ECAP1_Pos)                   /*!< SCU_T::D1PNS4: ECAP1 Mask              */

#define SCU_D1PNS4_ECAP3_Pos             (7)                                               /*!< SCU_T::D1PNS4: ECAP3 Position          */
#define SCU_D1PNS4_ECAP3_Msk             (0x1ul << SCU_D1PNS4_ECAP3_Pos)                   /*!< SCU_T::D1PNS4: ECAP3 Mask              */

#define SCU_D1PNS4_I2C1_Pos              (8)                                               /*!< SCU_T::D1PNS4: I2C1 Position           */
#define SCU_D1PNS4_I2C1_Msk              (0x1ul << SCU_D1PNS4_I2C1_Pos)                    /*!< SCU_T::D1PNS4: I2C1 Mask               */

#define SCU_D1PNS4_I2C3_Pos              (9)                                               /*!< SCU_T::D1PNS4: I2C3 Position           */
#define SCU_D1PNS4_I2C3_Msk              (0x1ul << SCU_D1PNS4_I2C3_Pos)                    /*!< SCU_T::D1PNS4: I2C3 Mask               */

#define SCU_D1PNS4_QSPI1_Pos             (10)                                              /*!< SCU_T::D1PNS4: QSPI1 Position          */
#define SCU_D1PNS4_QSPI1_Msk             (0x1ul << SCU_D1PNS4_QSPI1_Pos)                   /*!< SCU_T::D1PNS4: QSPI1 Mask              */

#define SCU_D1PNS4_SPI1_Pos              (11)                                              /*!< SCU_T::D1PNS4: SPI1 Position           */
#define SCU_D1PNS4_SPI1_Msk              (0x1ul << SCU_D1PNS4_SPI1_Pos)                    /*!< SCU_T::D1PNS4: SPI1 Mask               */

#define SCU_D1PNS4_SPI3_Pos              (12)                                              /*!< SCU_T::D1PNS4: SPI3 Position           */
#define SCU_D1PNS4_SPI3_Msk              (0x1ul << SCU_D1PNS4_SPI3_Pos)                    /*!< SCU_T::D1PNS4: SPI3 Mask               */

#define SCU_D1PNS4_UART1_Pos             (13)                                              /*!< SCU_T::D1PNS4: UART1 Position          */
#define SCU_D1PNS4_UART1_Msk             (0x1ul << SCU_D1PNS4_UART1_Pos)                   /*!< SCU_T::D1PNS4: UART1 Mask              */

#define SCU_D1PNS4_UART3_Pos             (14)                                              /*!< SCU_T::D1PNS4: UART3 Position          */
#define SCU_D1PNS4_UART3_Msk             (0x1ul << SCU_D1PNS4_UART3_Pos)                   /*!< SCU_T::D1PNS4: UART3 Mask              */

#define SCU_D1PNS4_UART5_Pos             (15)                                              /*!< SCU_T::D1PNS4: UART5 Position          */
#define SCU_D1PNS4_UART5_Msk             (0x1ul << SCU_D1PNS4_UART5_Pos)                   /*!< SCU_T::D1PNS4: UART5 Mask              */

#define SCU_D1PNS4_UART7_Pos             (16)                                              /*!< SCU_T::D1PNS4: UART7 Position          */
#define SCU_D1PNS4_UART7_Msk             (0x1ul << SCU_D1PNS4_UART7_Pos)                   /*!< SCU_T::D1PNS4: UART7 Mask              */

#define SCU_D1PNS4_UART9_Pos             (17)                                              /*!< SCU_T::D1PNS4: UART9 Position          */
#define SCU_D1PNS4_UART9_Msk             (0x1ul << SCU_D1PNS4_UART9_Pos)                   /*!< SCU_T::D1PNS4: UART9 Mask              */

#define SCU_D1PNS4_SC1_Pos               (18)                                              /*!< SCU_T::D1PNS4: SC1 Position            */
#define SCU_D1PNS4_SC1_Msk               (0x1ul << SCU_D1PNS4_SC1_Pos)                     /*!< SCU_T::D1PNS4: SC1 Mask                */

#define SCU_D1PNS4_OTG_Pos               (19)                                              /*!< SCU_T::D1PNS4: OTG Position            */
#define SCU_D1PNS4_OTG_Msk               (0x1ul << SCU_D1PNS4_OTG_Pos)                     /*!< SCU_T::D1PNS4: OTG Mask                */

#define SCU_D1PNS4_KPI_Pos               (20)                                              /*!< SCU_T::D1PNS4: KPI Position            */
#define SCU_D1PNS4_KPI_Msk               (0x1ul << SCU_D1PNS4_KPI_Pos)                     /*!< SCU_T::D1PNS4: KPI Mask                */

#define SCU_D1PNS4_TMR23_Pos             (21)                                              /*!< SCU_T::D1PNS4: TMR23 Position          */
#define SCU_D1PNS4_TMR23_Msk             (0x1ul << SCU_D1PNS4_TMR23_Pos)                   /*!< SCU_T::D1PNS4: TMR23 Mask              */

#define SCU_D1PNS4_TRNG_Pos              (22)                                              /*!< SCU_T::D1PNS4: TRNG Position           */
#define SCU_D1PNS4_TRNG_Msk              (0x1ul << SCU_D1PNS4_TRNG_Pos)                    /*!< SCU_T::D1PNS4: TRNG Mask               */

#define SCU_D1PNS4_RTC_Pos               (23)                                              /*!< SCU_T::D1PNS4: RTC Position            */
#define SCU_D1PNS4_RTC_Msk               (0x1ul << SCU_D1PNS4_RTC_Pos)                     /*!< SCU_T::D1PNS4: RTC Mask                */

#define SCU_D1PNS4_I2S1_Pos              (24)                                              /*!< SCU_T::D1PNS4: I2S1 Position           */
#define SCU_D1PNS4_I2S1_Msk              (0x1ul << SCU_D1PNS4_I2S1_Pos)                    /*!< SCU_T::D1PNS4: I2S1 Mask               */

#define SCU_D1PNS4_ACMP23_Pos            (25)                                              /*!< SCU_T::D1PNS4: ACMP23 Position         */
#define SCU_D1PNS4_ACMP23_Msk            (0x1ul << SCU_D1PNS4_ACMP23_Pos)                  /*!< SCU_T::D1PNS4: ACMP23 Mask             */

#define SCU_D1PNS4_I3C0_Pos              (26)                                              /*!< SCU_T::D1PNS4: I3C0 Position           */
#define SCU_D1PNS4_I3C0_Msk              (0x1ul << SCU_D1PNS4_I3C0_Pos)                    /*!< SCU_T::D1PNS4: I3C0 Mask               */

#define SCU_D1PNS4_UTCPD_Pos             (27)                                              /*!< SCU_T::D1PNS4: UTCPD Position          */
#define SCU_D1PNS4_UTCPD_Msk             (0x1ul << SCU_D1PNS4_UTCPD_Pos)                   /*!< SCU_T::D1PNS4: UTCPD Mask              */

#define SCU_D2PNS0_LPPDMA_Pos            (0)                                               /*!< SCU_T::D2PNS0: LPPDMA Position         */
#define SCU_D2PNS0_LPPDMA_Msk            (0x1ul << SCU_D2PNS0_LPPDMA_Pos)                  /*!< SCU_T::D2PNS0: LPPDMA Mask             */

#define SCU_D2PNS0_CCAP_Pos              (1)                                               /*!< SCU_T::D2PNS0: CCAP Position           */
#define SCU_D2PNS0_CCAP_Msk              (0x1ul << SCU_D2PNS0_CCAP_Pos)                    /*!< SCU_T::D2PNS0: CCAP Mask               */

#define SCU_D2PNS0_LPGPIO_Pos            (3)                                               /*!< SCU_T::D2PNS0: LPGPIO Position         */
#define SCU_D2PNS0_LPGPIO_Msk            (0x1ul << SCU_D2PNS0_LPGPIO_Pos)                  /*!< SCU_T::D2PNS0: LPGPIO Mask             */

#define SCU_D2PNS2_LPTMR01_Pos           (0)                                               /*!< SCU_T::D2PNS2: LPTMR01 Position        */
#define SCU_D2PNS2_LPTMR01_Msk           (0x1ul << SCU_D2PNS2_LPTMR01_Pos)                 /*!< SCU_T::D2PNS2: LPTMR01 Mask            */

#define SCU_D2PNS2_TTMR01_Pos            (1)                                               /*!< SCU_T::D2PNS2: TTMR01 Position         */
#define SCU_D2PNS2_TTMR01_Msk            (0x1ul << SCU_D2PNS2_TTMR01_Pos)                  /*!< SCU_T::D2PNS2: TTMR01 Mask             */

#define SCU_D2PNS2_LPADC0_Pos            (2)                                               /*!< SCU_T::D2PNS2: LPADC0 Position         */
#define SCU_D2PNS2_LPADC0_Msk            (0x1ul << SCU_D2PNS2_LPADC0_Pos)                  /*!< SCU_T::D2PNS2: LPADC0 Mask             */

#define SCU_D2PNS2_LPI2C0_Pos            (3)                                               /*!< SCU_T::D2PNS2: LPI2C0 Position         */
#define SCU_D2PNS2_LPI2C0_Msk            (0x1ul << SCU_D2PNS2_LPI2C0_Pos)                  /*!< SCU_T::D2PNS2: LPI2C0 Mask             */

#define SCU_D2PNS2_LPSPI0_Pos            (4)                                               /*!< SCU_T::D2PNS2: LPSPI0 Position         */
#define SCU_D2PNS2_LPSPI0_Msk            (0x1ul << SCU_D2PNS2_LPSPI0_Pos)                  /*!< SCU_T::D2PNS2: LPSPI0 Mask             */

#define SCU_D2PNS2_DMIC0_Pos             (5)                                               /*!< SCU_T::D2PNS2: DMIC0 Position          */
#define SCU_D2PNS2_DMIC0_Msk             (0x1ul << SCU_D2PNS2_DMIC0_Pos)                   /*!< SCU_T::D2PNS2: DMIC0 Mask              */

#define SCU_D2PNS2_LPUART0_Pos           (6)                                               /*!< SCU_T::D2PNS2: LPUART0 Position        */
#define SCU_D2PNS2_LPUART0_Msk           (0x1ul << SCU_D2PNS2_LPUART0_Pos)                 /*!< SCU_T::D2PNS2: LPUART0 Mask            */

#define SCU_D2PNS2_AWF_Pos               (9)                                               /*!< SCU_T::D2PNS2: AWF Position            */
#define SCU_D2PNS2_AWF_Msk               (0x1ul << SCU_D2PNS2_AWF_Pos)                     /*!< SCU_T::D2PNS2: AWF Mask                */

#define SCU_FNSADDR_FNSADDR_Pos          (0)                                               /*!< SCU_T::FNSADDR: FNSADDR Position       */
#define SCU_FNSADDR_FNSADDR_Msk          (0xfffffffful << SCU_FNSADDR_FNSADDR_Pos)         /*!< SCU_T::FNSADDR: FNSADDR Mask           */

#define SCU_EINTNS_EINT0_Pos             (0)                                               /*!< SCU_T::EINTNS: EINT0 Position          */
#define SCU_EINTNS_EINT0_Msk             (0x1ul << SCU_EINTNS_EINT0_Pos)                   /*!< SCU_T::EINTNS: EINT0 Mask              */

#define SCU_EINTNS_EINT1_Pos             (1)                                               /*!< SCU_T::EINTNS: EINT1 Position          */
#define SCU_EINTNS_EINT1_Msk             (0x1ul << SCU_EINTNS_EINT1_Pos)                   /*!< SCU_T::EINTNS: EINT1 Mask              */

#define SCU_EINTNS_EINT2_Pos             (2)                                               /*!< SCU_T::EINTNS: EINT2 Position          */
#define SCU_EINTNS_EINT2_Msk             (0x1ul << SCU_EINTNS_EINT2_Pos)                   /*!< SCU_T::EINTNS: EINT2 Mask              */

#define SCU_EINTNS_EINT3_Pos             (3)                                               /*!< SCU_T::EINTNS: EINT3 Position          */
#define SCU_EINTNS_EINT3_Msk             (0x1ul << SCU_EINTNS_EINT3_Pos)                   /*!< SCU_T::EINTNS: EINT3 Mask              */

#define SCU_EINTNS_EINT4_Pos             (4)                                               /*!< SCU_T::EINTNS: EINT4 Position          */
#define SCU_EINTNS_EINT4_Msk             (0x1ul << SCU_EINTNS_EINT4_Pos)                   /*!< SCU_T::EINTNS: EINT4 Mask              */

#define SCU_EINTNS_EINT5_Pos             (5)                                               /*!< SCU_T::EINTNS: EINT5 Position          */
#define SCU_EINTNS_EINT5_Msk             (0x1ul << SCU_EINTNS_EINT5_Pos)                   /*!< SCU_T::EINTNS: EINT5 Mask              */

#define SCU_EINTNS_EINT6_Pos             (6)                                               /*!< SCU_T::EINTNS: EINT6 Position          */
#define SCU_EINTNS_EINT6_Msk             (0x1ul << SCU_EINTNS_EINT6_Pos)                   /*!< SCU_T::EINTNS: EINT6 Mask              */

#define SCU_EINTNS_EINT7_Pos             (7)                                               /*!< SCU_T::EINTNS: EINT7 Position          */
#define SCU_EINTNS_EINT7_Msk             (0x1ul << SCU_EINTNS_EINT7_Pos)                   /*!< SCU_T::EINTNS: EINT7 Mask              */

#define SCU_IONS0_PA_Pos                 (0)                                               /*!< SCU_T::IONS0: PA Position              */
#define SCU_IONS0_PA_Msk                 (0xfffful << SCU_IONS0_PA_Pos)                    /*!< SCU_T::IONS0: PA Mask                  */

#define SCU_IONS1_PB_Pos                 (0)                                               /*!< SCU_T::IONS1: PB Position              */
#define SCU_IONS1_PB_Msk                 (0xfffful << SCU_IONS1_PB_Pos)                    /*!< SCU_T::IONS1: PB Mask                  */

#define SCU_IONS2_PC_Pos                 (0)                                               /*!< SCU_T::IONS2: PC Position              */
#define SCU_IONS2_PC_Msk                 (0xfffful << SCU_IONS2_PC_Pos)                    /*!< SCU_T::IONS2: PC Mask                  */

#define SCU_IONS3_PD_Pos                 (0)                                               /*!< SCU_T::IONS3: PD Position              */
#define SCU_IONS3_PD_Msk                 (0xfffful << SCU_IONS3_PD_Pos)                    /*!< SCU_T::IONS3: PD Mask                  */

#define SCU_IONS4_PE_Pos                 (0)                                               /*!< SCU_T::IONS4: PE Position              */
#define SCU_IONS4_PE_Msk                 (0xfffful << SCU_IONS4_PE_Pos)                    /*!< SCU_T::IONS4: PE Mask                  */

#define SCU_IONS5_PF_Pos                 (0)                                               /*!< SCU_T::IONS5: PF Position              */
#define SCU_IONS5_PF_Msk                 (0xfffful << SCU_IONS5_PF_Pos)                    /*!< SCU_T::IONS5: PF Mask                  */

#define SCU_IONS6_PG_Pos                 (0)                                               /*!< SCU_T::IONS6: PG Position              */
#define SCU_IONS6_PG_Msk                 (0xfffful << SCU_IONS6_PG_Pos)                    /*!< SCU_T::IONS6: PG Mask                  */

#define SCU_IONS7_PH_Pos                 (0)                                               /*!< SCU_T::IONS7: PH Position              */
#define SCU_IONS7_PH_Msk                 (0xfffful << SCU_IONS7_PH_Pos)                    /*!< SCU_T::IONS7: PH Mask                  */

#define SCU_IONS8_PI_Pos                 (0)                                               /*!< SCU_T::IONS8: PI Position              */
#define SCU_IONS8_PI_Msk                 (0xfffful << SCU_IONS8_PI_Pos)                    /*!< SCU_T::IONS8: PI Mask                  */

#define SCU_IONS9_PJ_Pos                 (0)                                               /*!< SCU_T::IONS9: PJ Position              */
#define SCU_IONS9_PJ_Msk                 (0xfffful << SCU_IONS9_PJ_Pos)                    /*!< SCU_T::IONS9: PJ Mask                  */

#define SCU_SVIEN0_APB0IEN_Pos           (0)                                               /*!< SCU_T::SVIEN0: APB0IEN Position        */
#define SCU_SVIEN0_APB0IEN_Msk           (0x1ul << SCU_SVIEN0_APB0IEN_Pos)                 /*!< SCU_T::SVIEN0: APB0IEN Mask            */

#define SCU_SVIEN0_APB1IEN_Pos           (1)                                               /*!< SCU_T::SVIEN0: APB1IEN Position        */
#define SCU_SVIEN0_APB1IEN_Msk           (0x1ul << SCU_SVIEN0_APB1IEN_Pos)                 /*!< SCU_T::SVIEN0: APB1IEN Mask            */

#define SCU_SVIEN0_APB2IEN_Pos           (2)                                               /*!< SCU_T::SVIEN0: APB2IEN Position        */
#define SCU_SVIEN0_APB2IEN_Msk           (0x1ul << SCU_SVIEN0_APB2IEN_Pos)                 /*!< SCU_T::SVIEN0: APB2IEN Mask            */

#define SCU_SVIEN0_APB3IEN_Pos           (3)                                               /*!< SCU_T::SVIEN0: APB3IEN Position        */
#define SCU_SVIEN0_APB3IEN_Msk           (0x1ul << SCU_SVIEN0_APB3IEN_Pos)                 /*!< SCU_T::SVIEN0: APB3IEN Mask            */

#define SCU_SVIEN0_APB4IEN_Pos           (4)                                               /*!< SCU_T::SVIEN0: APB4IEN Position        */
#define SCU_SVIEN0_APB4IEN_Msk           (0x1ul << SCU_SVIEN0_APB4IEN_Pos)                 /*!< SCU_T::SVIEN0: APB4IEN Mask            */

#define SCU_SVIEN0_APB5IEN_Pos           (5)                                               /*!< SCU_T::SVIEN0: APB5IEN Position        */
#define SCU_SVIEN0_APB5IEN_Msk           (0x1ul << SCU_SVIEN0_APB5IEN_Pos)                 /*!< SCU_T::SVIEN0: APB5IEN Mask            */

#define SCU_SVIEN0_D0PPC0IEN_Pos         (8)                                               /*!< SCU_T::SVIEN0: D0PPC0IEN Position      */
#define SCU_SVIEN0_D0PPC0IEN_Msk         (0x1ul << SCU_SVIEN0_D0PPC0IEN_Pos)               /*!< SCU_T::SVIEN0: D0PPC0IEN Mask          */

#define SCU_SVIEN0_D1PPC0IEN_Pos         (9)                                               /*!< SCU_T::SVIEN0: D1PPC0IEN Position      */
#define SCU_SVIEN0_D1PPC0IEN_Msk         (0x1ul << SCU_SVIEN0_D1PPC0IEN_Pos)               /*!< SCU_T::SVIEN0: D1PPC0IEN Mask          */

#define SCU_SVIEN0_D1PPC1IEN_Pos         (10)                                              /*!< SCU_T::SVIEN0: D1PPC1IEN Position      */
#define SCU_SVIEN0_D1PPC1IEN_Msk         (0x1ul << SCU_SVIEN0_D1PPC1IEN_Pos)               /*!< SCU_T::SVIEN0: D1PPC1IEN Mask          */

#define SCU_SVIEN0_D2PPC0IEN_Pos         (11)                                              /*!< SCU_T::SVIEN0: D2PPC0IEN Position      */
#define SCU_SVIEN0_D2PPC0IEN_Msk         (0x1ul << SCU_SVIEN0_D2PPC0IEN_Pos)               /*!< SCU_T::SVIEN0: D2PPC0IEN Mask          */

#define SCU_SVIEN0_EBIIEN_Pos            (16)                                              /*!< SCU_T::SVIEN0: EBIIEN Position         */
#define SCU_SVIEN0_EBIIEN_Msk            (0x1ul << SCU_SVIEN0_EBIIEN_Pos)                  /*!< SCU_T::SVIEN0: EBIIEN Mask             */

#define SCU_SVIEN0_FLASHIEN_Pos          (17)                                              /*!< SCU_T::SVIEN0: FLASHIEN Position       */
#define SCU_SVIEN0_FLASHIEN_Msk          (0x1ul << SCU_SVIEN0_FLASHIEN_Pos)                /*!< SCU_T::SVIEN0: FLASHIEN Mask           */

#define SCU_SVIEN1_GDMAIEN_Pos           (0)                                               /*!< SCU_T::SVIEN1: GDMAIEN Position        */
#define SCU_SVIEN1_GDMAIEN_Msk           (0x1ul << SCU_SVIEN1_GDMAIEN_Pos)                 /*!< SCU_T::SVIEN1: GDMAIEN Mask            */

#define SCU_SVIEN1_PDMA0IEN_Pos          (1)                                               /*!< SCU_T::SVIEN1: PDMA0IEN Position       */
#define SCU_SVIEN1_PDMA0IEN_Msk          (0x1ul << SCU_SVIEN1_PDMA0IEN_Pos)                /*!< SCU_T::SVIEN1: PDMA0IEN Mask           */

#define SCU_SVIEN1_PDMA1IEN_Pos          (2)                                               /*!< SCU_T::SVIEN1: PDMA1IEN Position       */
#define SCU_SVIEN1_PDMA1IEN_Msk          (0x1ul << SCU_SVIEN1_PDMA1IEN_Pos)                /*!< SCU_T::SVIEN1: PDMA1IEN Mask           */

#define SCU_SVIEN1_USBH0IEN_Pos          (3)                                               /*!< SCU_T::SVIEN1: USBH0IEN Position       */
#define SCU_SVIEN1_USBH0IEN_Msk          (0x1ul << SCU_SVIEN1_USBH0IEN_Pos)                /*!< SCU_T::SVIEN1: USBH0IEN Mask           */

#define SCU_SVIEN1_HSUSBHIEN_Pos         (5)                                               /*!< SCU_T::SVIEN1: HSUSBHIEN Position      */
#define SCU_SVIEN1_HSUSBHIEN_Msk         (0x1ul << SCU_SVIEN1_HSUSBHIEN_Pos)               /*!< SCU_T::SVIEN1: HSUSBHIEN Mask          */

#define SCU_SVIEN1_HSUSBDIEN_Pos         (6)                                               /*!< SCU_T::SVIEN1: HSUSBDIEN Position      */
#define SCU_SVIEN1_HSUSBDIEN_Msk         (0x1ul << SCU_SVIEN1_HSUSBDIEN_Pos)               /*!< SCU_T::SVIEN1: HSUSBDIEN Mask          */

#define SCU_SVIEN1_SDH0IEN_Pos           (7)                                               /*!< SCU_T::SVIEN1: SDH0IEN Position        */
#define SCU_SVIEN1_SDH0IEN_Msk           (0x1ul << SCU_SVIEN1_SDH0IEN_Pos)                 /*!< SCU_T::SVIEN1: SDH0IEN Mask            */

#define SCU_SVIEN1_SDH1IEN_Pos           (8)                                               /*!< SCU_T::SVIEN1: SDH1IEN Position        */
#define SCU_SVIEN1_SDH1IEN_Msk           (0x1ul << SCU_SVIEN1_SDH1IEN_Pos)                 /*!< SCU_T::SVIEN1: SDH1IEN Mask            */

#define SCU_SVIEN1_EMACIEN_Pos           (9)                                               /*!< SCU_T::SVIEN1: EMACIEN Position        */
#define SCU_SVIEN1_EMACIEN_Msk           (0x1ul << SCU_SVIEN1_EMACIEN_Pos)                 /*!< SCU_T::SVIEN1: EMACIEN Mask            */

#define SCU_SVIEN1_CRYPTOIEN_Pos         (10)                                              /*!< SCU_T::SVIEN1: CRYPTOIEN Position      */
#define SCU_SVIEN1_CRYPTOIEN_Msk         (0x1ul << SCU_SVIEN1_CRYPTOIEN_Pos)               /*!< SCU_T::SVIEN1: CRYPTOIEN Mask          */

#define SCU_SVIEN1_CRCIEN_Pos            (11)                                              /*!< SCU_T::SVIEN1: CRCIEN Position         */
#define SCU_SVIEN1_CRCIEN_Msk            (0x1ul << SCU_SVIEN1_CRCIEN_Pos)                  /*!< SCU_T::SVIEN1: CRCIEN Mask             */

#define SCU_SVIEN1_LPPDMAIEN_Pos         (12)                                              /*!< SCU_T::SVIEN1: LPPDMAIEN Position      */
#define SCU_SVIEN1_LPPDMAIEN_Msk         (0x1ul << SCU_SVIEN1_LPPDMAIEN_Pos)               /*!< SCU_T::SVIEN1: LPPDMAIEN Mask          */

#define SCU_SVIEN1_CCAPIEN_Pos           (13)                                              /*!< SCU_T::SVIEN1: CCAPIEN Position        */
#define SCU_SVIEN1_CCAPIEN_Msk           (0x1ul << SCU_SVIEN1_CCAPIEN_Pos)                 /*!< SCU_T::SVIEN1: CCAPIEN Mask            */

#define SCU_SVIEN1_NPUIF1IEN_Pos         (14)                                              /*!< SCU_T::SVIEN1: NPUIF1IEN Position      */
#define SCU_SVIEN1_NPUIF1IEN_Msk         (0x1ul << SCU_SVIEN1_NPUIF1IEN_Pos)               /*!< SCU_T::SVIEN1: NPUIF1IEN Mask          */

#define SCU_SVIEN1_NPUIF0IEN_Pos         (15)                                              /*!< SCU_T::SVIEN1: NPUIF0IEN Position      */
#define SCU_SVIEN1_NPUIF0IEN_Msk         (0x1ul << SCU_SVIEN1_NPUIF0IEN_Pos)               /*!< SCU_T::SVIEN1: NPUIF0IEN Mask          */

#define SCU_SVIEN1_SPIM0IEN_Pos          (16)                                              /*!< SCU_T::SVIEN1: SPIM0IEN Position       */
#define SCU_SVIEN1_SPIM0IEN_Msk          (0x1ul << SCU_SVIEN1_SPIM0IEN_Pos)                /*!< SCU_T::SVIEN1: SPIM0IEN Mask           */

#define SCU_SVINTSTS0_APB0IF_Pos         (0)                                               /*!< SCU_T::SVINTSTS0: APB0IF Position      */
#define SCU_SVINTSTS0_APB0IF_Msk         (0x1ul << SCU_SVINTSTS0_APB0IF_Pos)               /*!< SCU_T::SVINTSTS0: APB0IF Mask          */

#define SCU_SVINTSTS0_APB1IF_Pos         (1)                                               /*!< SCU_T::SVINTSTS0: APB1IF Position      */
#define SCU_SVINTSTS0_APB1IF_Msk         (0x1ul << SCU_SVINTSTS0_APB1IF_Pos)               /*!< SCU_T::SVINTSTS0: APB1IF Mask          */

#define SCU_SVINTSTS0_APB2IF_Pos         (2)                                               /*!< SCU_T::SVINTSTS0: APB2IF Position      */
#define SCU_SVINTSTS0_APB2IF_Msk         (0x1ul << SCU_SVINTSTS0_APB2IF_Pos)               /*!< SCU_T::SVINTSTS0: APB2IF Mask          */

#define SCU_SVINTSTS0_APB3IF_Pos         (3)                                               /*!< SCU_T::SVINTSTS0: APB3IF Position      */
#define SCU_SVINTSTS0_APB3IF_Msk         (0x1ul << SCU_SVINTSTS0_APB3IF_Pos)               /*!< SCU_T::SVINTSTS0: APB3IF Mask          */

#define SCU_SVINTSTS0_APB4IF_Pos         (4)                                               /*!< SCU_T::SVINTSTS0: APB4IF Position      */
#define SCU_SVINTSTS0_APB4IF_Msk         (0x1ul << SCU_SVINTSTS0_APB4IF_Pos)               /*!< SCU_T::SVINTSTS0: APB4IF Mask          */

#define SCU_SVINTSTS0_APB5IF_Pos         (5)                                               /*!< SCU_T::SVINTSTS0: APB5IF Position      */
#define SCU_SVINTSTS0_APB5IF_Msk         (0x1ul << SCU_SVINTSTS0_APB5IF_Pos)               /*!< SCU_T::SVINTSTS0: APB5IF Mask          */

#define SCU_SVINTSTS0_D0PPC0IF_Pos       (8)                                               /*!< SCU_T::SVINTSTS0: D0PPC0IF Position    */
#define SCU_SVINTSTS0_D0PPC0IF_Msk       (0x1ul << SCU_SVINTSTS0_D0PPC0IF_Pos)             /*!< SCU_T::SVINTSTS0: D0PPC0IF Mask        */

#define SCU_SVINTSTS0_D1PPC0IF_Pos       (9)                                               /*!< SCU_T::SVINTSTS0: D1PPC0IF Position    */
#define SCU_SVINTSTS0_D1PPC0IF_Msk       (0x1ul << SCU_SVINTSTS0_D1PPC0IF_Pos)             /*!< SCU_T::SVINTSTS0: D1PPC0IF Mask        */

#define SCU_SVINTSTS0_D1PPC1IF_Pos       (10)                                              /*!< SCU_T::SVINTSTS0: D1PPC1IF Position    */
#define SCU_SVINTSTS0_D1PPC1IF_Msk       (0x1ul << SCU_SVINTSTS0_D1PPC1IF_Pos)             /*!< SCU_T::SVINTSTS0: D1PPC1IF Mask        */

#define SCU_SVINTSTS0_D2PPC0IF_Pos       (11)                                              /*!< SCU_T::SVINTSTS0: D2PPC0IF Position    */
#define SCU_SVINTSTS0_D2PPC0IF_Msk       (0x1ul << SCU_SVINTSTS0_D2PPC0IF_Pos)             /*!< SCU_T::SVINTSTS0: D2PPC0IF Mask        */

#define SCU_SVINTSTS0_EBIIF_Pos          (16)                                              /*!< SCU_T::SVINTSTS0: EBIIF Position       */
#define SCU_SVINTSTS0_EBIIF_Msk          (0x1ul << SCU_SVINTSTS0_EBIIF_Pos)                /*!< SCU_T::SVINTSTS0: EBIIF Mask           */

#define SCU_SVINTSTS0_FLASHIF_Pos        (17)                                              /*!< SCU_T::SVINTSTS0: FLASHIF Position     */
#define SCU_SVINTSTS0_FLASHIF_Msk        (0x1ul << SCU_SVINTSTS0_FLASHIF_Pos)              /*!< SCU_T::SVINTSTS0: FLASHIF Mask         */

#define SCU_SVINTSTS1_GDMAIF_Pos         (0)                                               /*!< SCU_T::SVINTSTS1: GDMAIF Position      */
#define SCU_SVINTSTS1_GDMAIF_Msk         (0x1ul << SCU_SVINTSTS1_GDMAIF_Pos)               /*!< SCU_T::SVINTSTS1: GDMAIF Mask          */

#define SCU_SVINTSTS1_PDMA0IF_Pos        (1)                                               /*!< SCU_T::SVINTSTS1: PDMA0IF Position     */
#define SCU_SVINTSTS1_PDMA0IF_Msk        (0x1ul << SCU_SVINTSTS1_PDMA0IF_Pos)              /*!< SCU_T::SVINTSTS1: PDMA0IF Mask         */

#define SCU_SVINTSTS1_PDMA1IF_Pos        (2)                                               /*!< SCU_T::SVINTSTS1: PDMA1IF Position     */
#define SCU_SVINTSTS1_PDMA1IF_Msk        (0x1ul << SCU_SVINTSTS1_PDMA1IF_Pos)              /*!< SCU_T::SVINTSTS1: PDMA1IF Mask         */

#define SCU_SVINTSTS1_USBH0IF_Pos        (3)                                               /*!< SCU_T::SVINTSTS1: USBH0IF Position     */
#define SCU_SVINTSTS1_USBH0IF_Msk        (0x1ul << SCU_SVINTSTS1_USBH0IF_Pos)              /*!< SCU_T::SVINTSTS1: USBH0IF Mask         */

#define SCU_SVINTSTS1_HSUSBHIF_Pos       (5)                                               /*!< SCU_T::SVINTSTS1: HSUSBHIF Position    */
#define SCU_SVINTSTS1_HSUSBHIF_Msk       (0x1ul << SCU_SVINTSTS1_HSUSBHIF_Pos)             /*!< SCU_T::SVINTSTS1: HSUSBHIF Mask        */

#define SCU_SVINTSTS1_HSUSBDIF_Pos       (6)                                               /*!< SCU_T::SVINTSTS1: HSUSBDIF Position    */
#define SCU_SVINTSTS1_HSUSBDIF_Msk       (0x1ul << SCU_SVINTSTS1_HSUSBDIF_Pos)             /*!< SCU_T::SVINTSTS1: HSUSBDIF Mask        */

#define SCU_SVINTSTS1_SDH0IF_Pos         (7)                                               /*!< SCU_T::SVINTSTS1: SDH0IF Position      */
#define SCU_SVINTSTS1_SDH0IF_Msk         (0x1ul << SCU_SVINTSTS1_SDH0IF_Pos)               /*!< SCU_T::SVINTSTS1: SDH0IF Mask          */

#define SCU_SVINTSTS1_SDH1IF_Pos         (8)                                               /*!< SCU_T::SVINTSTS1: SDH1IF Position      */
#define SCU_SVINTSTS1_SDH1IF_Msk         (0x1ul << SCU_SVINTSTS1_SDH1IF_Pos)               /*!< SCU_T::SVINTSTS1: SDH1IF Mask          */

#define SCU_SVINTSTS1_EMACIF_Pos         (9)                                               /*!< SCU_T::SVINTSTS1: EMACIF Position      */
#define SCU_SVINTSTS1_EMACIF_Msk         (0x1ul << SCU_SVINTSTS1_EMACIF_Pos)               /*!< SCU_T::SVINTSTS1: EMACIF Mask          */

#define SCU_SVINTSTS1_CRYPTOIF_Pos       (10)                                              /*!< SCU_T::SVINTSTS1: CRYPTOIF Position    */
#define SCU_SVINTSTS1_CRYPTOIF_Msk       (0x1ul << SCU_SVINTSTS1_CRYPTOIF_Pos)             /*!< SCU_T::SVINTSTS1: CRYPTOIF Mask        */

#define SCU_SVINTSTS1_CRCIF_Pos          (11)                                              /*!< SCU_T::SVINTSTS1: CRCIF Position       */
#define SCU_SVINTSTS1_CRCIF_Msk          (0x1ul << SCU_SVINTSTS1_CRCIF_Pos)                /*!< SCU_T::SVINTSTS1: CRCIF Mask           */

#define SCU_SVINTSTS1_LPPDMAIF_Pos       (12)                                              /*!< SCU_T::SVINTSTS1: LPPDMAIF Position    */
#define SCU_SVINTSTS1_LPPDMAIF_Msk       (0x1ul << SCU_SVINTSTS1_LPPDMAIF_Pos)             /*!< SCU_T::SVINTSTS1: LPPDMAIF Mask        */

#define SCU_SVINTSTS1_CCAPIF_Pos         (13)                                              /*!< SCU_T::SVINTSTS1: CCAPIF Position      */
#define SCU_SVINTSTS1_CCAPIF_Msk         (0x1ul << SCU_SVINTSTS1_CCAPIF_Pos)               /*!< SCU_T::SVINTSTS1: CCAPIF Mask          */

#define SCU_SVINTSTS1_NPUIF1_Pos         (14)                                              /*!< SCU_T::SVINTSTS1: NPUIF1 Position      */
#define SCU_SVINTSTS1_NPUIF1_Msk         (0x1ul << SCU_SVINTSTS1_NPUIF1_Pos)               /*!< SCU_T::SVINTSTS1: NPUIF1 Mask          */

#define SCU_SVINTSTS1_NPUIF0_Pos         (15)                                              /*!< SCU_T::SVINTSTS1: NPUIF0 Position      */
#define SCU_SVINTSTS1_NPUIF0_Msk         (0x1ul << SCU_SVINTSTS1_NPUIF0_Pos)               /*!< SCU_T::SVINTSTS1: NPUIF0 Mask          */

#define SCU_SVINTSTS1_SPIM0IF_Pos        (16)                                              /*!< SCU_T::SVINTSTS1: SPIM0IF Position     */
#define SCU_SVINTSTS1_SPIM0IF_Msk        (0x1ul << SCU_SVINTSTS1_SPIM0IF_Pos)              /*!< SCU_T::SVINTSTS1: SPIM0IF Mask         */

#define SCU_SVINTSTS2_SRAM0IF_Pos        (0)                                               /*!< SCU_T::SVINTSTS2: SRAM0IF Position     */
#define SCU_SVINTSTS2_SRAM0IF_Msk        (0x1ul << SCU_SVINTSTS2_SRAM0IF_Pos)              /*!< SCU_T::SVINTSTS2: SRAM0IF Mask         */

#define SCU_SVINTSTS2_SRAM1IF_Pos        (1)                                               /*!< SCU_T::SVINTSTS2: SRAM1IF Position     */
#define SCU_SVINTSTS2_SRAM1IF_Msk        (0x1ul << SCU_SVINTSTS2_SRAM1IF_Pos)              /*!< SCU_T::SVINTSTS2: SRAM1IF Mask         */

#define SCU_SVINTSTS2_SRAM2IF_Pos        (2)                                               /*!< SCU_T::SVINTSTS2: SRAM2IF Position     */
#define SCU_SVINTSTS2_SRAM2IF_Msk        (0x1ul << SCU_SVINTSTS2_SRAM2IF_Pos)              /*!< SCU_T::SVINTSTS2: SRAM2IF Mask         */

#define SCU_SVINTSTS2_SRAM3IF_Pos        (3)                                               /*!< SCU_T::SVINTSTS2: SRAM3IF Position     */
#define SCU_SVINTSTS2_SRAM3IF_Msk        (0x1ul << SCU_SVINTSTS2_SRAM3IF_Pos)              /*!< SCU_T::SVINTSTS2: SRAM3IF Mask         */

#define SCU_SVINTSTS2_LPSRAMIF_Pos       (4)                                               /*!< SCU_T::SVINTSTS2: LPSRAMIF Position    */
#define SCU_SVINTSTS2_LPSRAMIF_Msk       (0x1ul << SCU_SVINTSTS2_LPSRAMIF_Pos)             /*!< SCU_T::SVINTSTS2: LPSRAMIF Mask        */

#define SCU_SVINTSTS2_SPIM0IF_Pos        (5)                                               /*!< SCU_T::SVINTSTS2: SPIM0IF Position     */
#define SCU_SVINTSTS2_SPIM0IF_Msk        (0x1ul << SCU_SVINTSTS2_SPIM0IF_Pos)              /*!< SCU_T::SVINTSTS2: SPIM0IF Mask         */

#define SCU_APB0PVSRC_MASTER_Pos         (0)                                               /*!< SCU_T::APB0PVSRC: MASTER Position      */
#define SCU_APB0PVSRC_MASTER_Msk         (0x1ful << SCU_APB0PVSRC_MASTER_Pos)              /*!< SCU_T::APB0PVSRC: MASTER Mask          */

#define SCU_APB1PVSRC_MASTER_Pos         (0)                                               /*!< SCU_T::APB1PVSRC: MASTER Position      */
#define SCU_APB1PVSRC_MASTER_Msk         (0x1ful << SCU_APB1PVSRC_MASTER_Pos)              /*!< SCU_T::APB1PVSRC: MASTER Mask          */

#define SCU_APB2PVSRC_MASTER_Pos         (0)                                               /*!< SCU_T::APB2PVSRC: MASTER Position      */
#define SCU_APB2PVSRC_MASTER_Msk         (0x1ful << SCU_APB2PVSRC_MASTER_Pos)              /*!< SCU_T::APB2PVSRC: MASTER Mask          */

#define SCU_APB3PVSRC_MASTER_Pos         (0)                                               /*!< SCU_T::APB3PVSRC: MASTER Position      */
#define SCU_APB3PVSRC_MASTER_Msk         (0x1ful << SCU_APB3PVSRC_MASTER_Pos)              /*!< SCU_T::APB3PVSRC: MASTER Mask          */

#define SCU_APB4PVSRC_MASTER_Pos         (0)                                               /*!< SCU_T::APB4PVSRC: MASTER Position      */
#define SCU_APB4PVSRC_MASTER_Msk         (0x1ful << SCU_APB4PVSRC_MASTER_Pos)              /*!< SCU_T::APB4PVSRC: MASTER Mask          */

#define SCU_APB5PVSRC_MASTER_Pos         (0)                                               /*!< SCU_T::APB5PVSRC: MASTER Position      */
#define SCU_APB5PVSRC_MASTER_Msk         (0x1ful << SCU_APB5PVSRC_MASTER_Pos)              /*!< SCU_T::APB5PVSRC: MASTER Mask          */

#define SCU_D0PPC0PVSRC_MASTER_Pos       (0)                                               /*!< SCU_T::D0PPC0PVSRC: MASTER Position    */
#define SCU_D0PPC0PVSRC_MASTER_Msk       (0x1ful << SCU_D0PPC0PVSRC_MASTER_Pos)            /*!< SCU_T::D0PPC0PVSRC: MASTER Mask        */

#define SCU_D1PPC0PVSRC_MASTER_Pos       (0)                                               /*!< SCU_T::D1PPC0PVSRC: MASTER Position    */
#define SCU_D1PPC0PVSRC_MASTER_Msk       (0x1ful << SCU_D1PPC0PVSRC_MASTER_Pos)            /*!< SCU_T::D1PPC0PVSRC: MASTER Mask        */

#define SCU_D1PPC1PVSRC_MASTER_Pos       (0)                                               /*!< SCU_T::D1PPC1PVSRC: MASTER Position    */
#define SCU_D1PPC1PVSRC_MASTER_Msk       (0x1ful << SCU_D1PPC1PVSRC_MASTER_Pos)            /*!< SCU_T::D1PPC1PVSRC: MASTER Mask        */

#define SCU_D2PPC0PVSRC_MASTER_Pos       (0)                                               /*!< SCU_T::D2PPC0PVSRC: MASTER Position    */
#define SCU_D2PPC0PVSRC_MASTER_Msk       (0x1ful << SCU_D2PPC0PVSRC_MASTER_Pos)            /*!< SCU_T::D2PPC0PVSRC: MASTER Mask        */

#define SCU_EBIPVSRC_MASTER_Pos          (0)                                               /*!< SCU_T::EBIPVSRC: MASTER Position       */
#define SCU_EBIPVSRC_MASTER_Msk          (0x1ful << SCU_EBIPVSRC_MASTER_Pos)               /*!< SCU_T::EBIPVSRC: MASTER Mask           */

#define SCU_FLASHPVSRC_MASTER_Pos        (0)                                               /*!< SCU_T::FLASHPVSRC: MASTER Position     */
#define SCU_FLASHPVSRC_MASTER_Msk        (0x1ful << SCU_FLASHPVSRC_MASTER_Pos)             /*!< SCU_T::FLASHPVSRC: MASTER Mask         */

#define SCU_APB0PVA_VIOADDR_Pos          (0)                                               /*!< SCU_T::APB0PVA: VIOADDR Position       */
#define SCU_APB0PVA_VIOADDR_Msk          (0xfffffffful << SCU_APB0PVA_VIOADDR_Pos)         /*!< SCU_T::APB0PVA: VIOADDR Mask           */

#define SCU_APB1PVA_VIOADDR_Pos          (0)                                               /*!< SCU_T::APB1PVA: VIOADDR Position       */
#define SCU_APB1PVA_VIOADDR_Msk          (0xfffffffful << SCU_APB1PVA_VIOADDR_Pos)         /*!< SCU_T::APB1PVA: VIOADDR Mask           */

#define SCU_APB2PVA_VIOADDR_Pos          (0)                                               /*!< SCU_T::APB2PVA: VIOADDR Position       */
#define SCU_APB2PVA_VIOADDR_Msk          (0xfffffffful << SCU_APB2PVA_VIOADDR_Pos)         /*!< SCU_T::APB2PVA: VIOADDR Mask           */

#define SCU_APB3PVA_VIOADDR_Pos          (0)                                               /*!< SCU_T::APB3PVA: VIOADDR Position       */
#define SCU_APB3PVA_VIOADDR_Msk          (0xfffffffful << SCU_APB3PVA_VIOADDR_Pos)         /*!< SCU_T::APB3PVA: VIOADDR Mask           */

#define SCU_APB4PVA_VIOADDR_Pos          (0)                                               /*!< SCU_T::APB4PVA: VIOADDR Position       */
#define SCU_APB4PVA_VIOADDR_Msk          (0xfffffffful << SCU_APB4PVA_VIOADDR_Pos)         /*!< SCU_T::APB4PVA: VIOADDR Mask           */

#define SCU_APB5PVA_VIOADDR_Pos          (0)                                               /*!< SCU_T::APB5PVA: VIOADDR Position       */
#define SCU_APB5PVA_VIOADDR_Msk          (0xfffffffful << SCU_APB5PVA_VIOADDR_Pos)         /*!< SCU_T::APB5PVA: VIOADDR Mask           */

#define SCU_D0PPC0PVA_VIOADDR_Pos        (0)                                               /*!< SCU_T::D0PPC0PVA: VIOADDR Position     */
#define SCU_D0PPC0PVA_VIOADDR_Msk        (0xfffffffful << SCU_D0PPC0PVA_VIOADDR_Pos)       /*!< SCU_T::D0PPC0PVA: VIOADDR Mask         */

#define SCU_D1PPC0PVA_VIOADDR_Pos        (0)                                               /*!< SCU_T::D1PPC0PVA: VIOADDR Position     */
#define SCU_D1PPC0PVA_VIOADDR_Msk        (0xfffffffful << SCU_D1PPC0PVA_VIOADDR_Pos)       /*!< SCU_T::D1PPC0PVA: VIOADDR Mask         */

#define SCU_D1PPC1PVA_VIOADDR_Pos        (0)                                               /*!< SCU_T::D1PPC1PVA: VIOADDR Position     */
#define SCU_D1PPC1PVA_VIOADDR_Msk        (0xfffffffful << SCU_D1PPC1PVA_VIOADDR_Pos)       /*!< SCU_T::D1PPC1PVA: VIOADDR Mask         */

#define SCU_D2PPC0PVA_VIOADDR_Pos        (0)                                               /*!< SCU_T::D2PPC0PVA: VIOADDR Position     */
#define SCU_D2PPC0PVA_VIOADDR_Msk        (0xfffffffful << SCU_D2PPC0PVA_VIOADDR_Pos)       /*!< SCU_T::D2PPC0PVA: VIOADDR Mask         */

#define SCU_EBIPVA_VIOADDR_Pos           (0)                                               /*!< SCU_T::EBIPVA: VIOADDR Position        */
#define SCU_EBIPVA_VIOADDR_Msk           (0xfffffffful << SCU_EBIPVA_VIOADDR_Pos)          /*!< SCU_T::EBIPVA: VIOADDR Mask            */

#define SCU_FLASHPVA_VIOADDR_Pos         (0)                                               /*!< SCU_T::FLASHPVA: VIOADDR Position      */
#define SCU_FLASHPVA_VIOADDR_Msk         (0xfffffffful << SCU_FLASHPVA_VIOADDR_Pos)        /*!< SCU_T::FLASHPVA: VIOADDR Mask          */

#define SCU_GDMAMVA_VIOADDR_Pos          (0)                                               /*!< SCU_T::GDMAMVA: VIOADDR Position       */
#define SCU_GDMAMVA_VIOADDR_Msk          (0xfffffffful << SCU_GDMAMVA_VIOADDR_Pos)         /*!< SCU_T::GDMAMVA: VIOADDR Mask           */

#define SCU_PDMA0MVA_VIOADDR_Pos         (0)                                               /*!< SCU_T::PDMA0MVA: VIOADDR Position      */
#define SCU_PDMA0MVA_VIOADDR_Msk         (0xfffffffful << SCU_PDMA0MVA_VIOADDR_Pos)        /*!< SCU_T::PDMA0MVA: VIOADDR Mask          */

#define SCU_PDMA1MVA_VIOADDR_Pos         (0)                                               /*!< SCU_T::PDMA1MVA: VIOADDR Position      */
#define SCU_PDMA1MVA_VIOADDR_Msk         (0xfffffffful << SCU_PDMA1MVA_VIOADDR_Pos)        /*!< SCU_T::PDMA1MVA: VIOADDR Mask          */

#define SCU_USBH0MVA_VIOADDR_Pos         (0)                                               /*!< SCU_T::USBH0MVA: VIOADDR Position      */
#define SCU_USBH0MVA_VIOADDR_Msk         (0xfffffffful << SCU_USBH0MVA_VIOADDR_Pos)        /*!< SCU_T::USBH0MVA: VIOADDR Mask          */

#define SCU_HSUSBHMVA_VIOADDR_Pos        (0)                                               /*!< SCU_T::HSUSBHMVA: VIOADDR Position     */
#define SCU_HSUSBHMVA_VIOADDR_Msk        (0xfffffffful << SCU_HSUSBHMVA_VIOADDR_Pos)       /*!< SCU_T::HSUSBHMVA: VIOADDR Mask         */

#define SCU_HSUSBDMVA_VIOADDR_Pos        (0)                                               /*!< SCU_T::HSUSBDMVA: VIOADDR Position     */
#define SCU_HSUSBDMVA_VIOADDR_Msk        (0xfffffffful << SCU_HSUSBDMVA_VIOADDR_Pos)       /*!< SCU_T::HSUSBDMVA: VIOADDR Mask         */

#define SCU_SDH0MVA_VIOADDR_Pos          (0)                                               /*!< SCU_T::SDH0MVA: VIOADDR Position       */
#define SCU_SDH0MVA_VIOADDR_Msk          (0xfffffffful << SCU_SDH0MVA_VIOADDR_Pos)         /*!< SCU_T::SDH0MVA: VIOADDR Mask           */

#define SCU_SDH1MVA_VIOADDR_Pos          (0)                                               /*!< SCU_T::SDH1MVA: VIOADDR Position       */
#define SCU_SDH1MVA_VIOADDR_Msk          (0xfffffffful << SCU_SDH1MVA_VIOADDR_Pos)         /*!< SCU_T::SDH1MVA: VIOADDR Mask           */

#define SCU_EMACMVA_VIOADDR_Pos          (0)                                               /*!< SCU_T::EMACMVA: VIOADDR Position       */
#define SCU_EMACMVA_VIOADDR_Msk          (0xfffffffful << SCU_EMACMVA_VIOADDR_Pos)         /*!< SCU_T::EMACMVA: VIOADDR Mask           */

#define SCU_CRYPTOMVA_VIOADDR_Pos        (0)                                               /*!< SCU_T::CRYPTOMVA: VIOADDR Position     */
#define SCU_CRYPTOMVA_VIOADDR_Msk        (0xfffffffful << SCU_CRYPTOMVA_VIOADDR_Pos)       /*!< SCU_T::CRYPTOMVA: VIOADDR Mask         */

#define SCU_CRCMVA_VIOADDR_Pos           (0)                                               /*!< SCU_T::CRCMVA: VIOADDR Position        */
#define SCU_CRCMVA_VIOADDR_Msk           (0xfffffffful << SCU_CRCMVA_VIOADDR_Pos)          /*!< SCU_T::CRCMVA: VIOADDR Mask            */

#define SCU_LPPDMAMVA_VIOADDR_Pos        (0)                                               /*!< SCU_T::LPPDMAMVA: VIOADDR Position     */
#define SCU_LPPDMAMVA_VIOADDR_Msk        (0xfffffffful << SCU_LPPDMAMVA_VIOADDR_Pos)       /*!< SCU_T::LPPDMAMVA: VIOADDR Mask         */

#define SCU_CCAPMVA_VIOADDR_Pos          (0)                                               /*!< SCU_T::CCAPMVA: VIOADDR Position       */
#define SCU_CCAPMVA_VIOADDR_Msk          (0xfffffffful << SCU_CCAPMVA_VIOADDR_Pos)         /*!< SCU_T::CCAPMVA: VIOADDR Mask           */

#define SCU_NPUIF1MVA_VIOADDR_Pos        (0)                                               /*!< SCU_T::NPUIF1MVA: VIOADDR Position     */
#define SCU_NPUIF1MVA_VIOADDR_Msk        (0xfffffffful << SCU_NPUIF1MVA_VIOADDR_Pos)       /*!< SCU_T::NPUIF1MVA: VIOADDR Mask         */

#define SCU_NPUIF0MVA_VIOADDR_Pos        (0)                                               /*!< SCU_T::NPUIF0MVA: VIOADDR Position     */
#define SCU_NPUIF0MVA_VIOADDR_Msk        (0xfffffffful << SCU_NPUIF0MVA_VIOADDR_Pos)       /*!< SCU_T::NPUIF0MVA: VIOADDR Mask         */

#define SCU_SPIM0MVA_VIOADDR_Pos         (0)                                               /*!< SCU_T::SPIM0MVA: VIOADDR Position      */
#define SCU_SPIM0MVA_VIOADDR_Msk         (0xfffffffful << SCU_SPIM0MVA_VIOADDR_Pos)        /*!< SCU_T::SPIM0MVA: VIOADDR Mask          */

#define SCU_D0PNP0_NPU_Pos               (3)                                               /*!< SCU_T::D0PNP0: NPU Position            */
#define SCU_D0PNP0_NPU_Msk               (0x1ul << SCU_D0PNP0_NPU_Pos)                     /*!< SCU_T::D0PNP0: NPU Mask                */

#define SCU_D0PNP0_SRAM0MPC_Pos          (8)                                               /*!< SCU_T::D0PNP0: SRAM0MPC Position       */
#define SCU_D0PNP0_SRAM0MPC_Msk          (0x1ul << SCU_D0PNP0_SRAM0MPC_Pos)                /*!< SCU_T::D0PNP0: SRAM0MPC Mask           */

#define SCU_D0PNP0_SRAM1MPC_Pos          (9)                                               /*!< SCU_T::D0PNP0: SRAM1MPC Position       */
#define SCU_D0PNP0_SRAM1MPC_Msk          (0x1ul << SCU_D0PNP0_SRAM1MPC_Pos)                /*!< SCU_T::D0PNP0: SRAM1MPC Mask           */

#define SCU_D0PNP0_SRAM2MPC_Pos          (10)                                              /*!< SCU_T::D0PNP0: SRAM2MPC Position       */
#define SCU_D0PNP0_SRAM2MPC_Msk          (0x1ul << SCU_D0PNP0_SRAM2MPC_Pos)                /*!< SCU_T::D0PNP0: SRAM2MPC Mask           */

#define SCU_D0PNP0_SRAM3MPC_Pos          (11)                                              /*!< SCU_T::D0PNP0: SRAM3MPC Position       */
#define SCU_D0PNP0_SRAM3MPC_Msk          (0x1ul << SCU_D0PNP0_SRAM3MPC_Pos)                /*!< SCU_T::D0PNP0: SRAM3MPC Mask           */

#define SCU_D0PNP0_SPIM0MPC_Pos          (13)                                              /*!< SCU_T::D0PNP0: SPIM0MPC Position       */
#define SCU_D0PNP0_SPIM0MPC_Msk          (0x1ul << SCU_D0PNP0_SPIM0MPC_Pos)                /*!< SCU_T::D0PNP0: SPIM0MPC Mask           */

#define SCU_D0PNP2_OTFC0_Pos             (0)                                               /*!< SCU_T::D0PNP2: OTFC0 Position          */
#define SCU_D0PNP2_OTFC0_Msk             (0x1ul << SCU_D0PNP2_OTFC0_Pos)                   /*!< SCU_T::D0PNP2: OTFC0 Mask              */

#define SCU_D0PNP2_SPIM0_Pos             (2)                                               /*!< SCU_T::D0PNP2: SPIM0 Position          */
#define SCU_D0PNP2_SPIM0_Msk             (0x1ul << SCU_D0PNP2_SPIM0_Pos)                   /*!< SCU_T::D0PNP2: SPIM0 Mask              */

#define SCU_D0PNP2_FMC_Pos               (4)                                               /*!< SCU_T::D0PNP2: FMC Position            */
#define SCU_D0PNP2_FMC_Msk               (0x1ul << SCU_D0PNP2_FMC_Pos)                     /*!< SCU_T::D0PNP2: FMC Mask                */

#define SCU_D1PNP0_PDMA0_Pos             (0)                                               /*!< SCU_T::D1PNP0: PDMA0 Position          */
#define SCU_D1PNP0_PDMA0_Msk             (0x1ul << SCU_D1PNP0_PDMA0_Pos)                   /*!< SCU_T::D1PNP0: PDMA0 Mask              */

#define SCU_D1PNP0_PDMA1_Pos             (1)                                               /*!< SCU_T::D1PNP0: PDMA1 Position          */
#define SCU_D1PNP0_PDMA1_Msk             (0x1ul << SCU_D1PNP0_PDMA1_Pos)                   /*!< SCU_T::D1PNP0: PDMA1 Mask              */

#define SCU_D1PNP0_USBH0_Pos             (2)                                               /*!< SCU_T::D1PNP0: USBH0 Position          */
#define SCU_D1PNP0_USBH0_Msk             (0x1ul << SCU_D1PNP0_USBH0_Pos)                   /*!< SCU_T::D1PNP0: USBH0 Mask              */

#define SCU_D1PNP0_HSUSBH_Pos            (4)                                               /*!< SCU_T::D1PNP0: HSUSBH Position         */
#define SCU_D1PNP0_HSUSBH_Msk            (0x1ul << SCU_D1PNP0_HSUSBH_Pos)                  /*!< SCU_T::D1PNP0: HSUSBH Mask             */

#define SCU_D1PNP0_HSUSBD_Pos            (5)                                               /*!< SCU_T::D1PNP0: HSUSBD Position         */
#define SCU_D1PNP0_HSUSBD_Msk            (0x1ul << SCU_D1PNP0_HSUSBD_Pos)                  /*!< SCU_T::D1PNP0: HSUSBD Mask             */

#define SCU_D1PNP0_SDH0_Pos              (6)                                               /*!< SCU_T::D1PNP0: SDH0 Position           */
#define SCU_D1PNP0_SDH0_Msk              (0x1ul << SCU_D1PNP0_SDH0_Pos)                    /*!< SCU_T::D1PNP0: SDH0 Mask               */

#define SCU_D1PNP0_SDH1_Pos              (7)                                               /*!< SCU_T::D1PNP0: SDH1 Position           */
#define SCU_D1PNP0_SDH1_Msk              (0x1ul << SCU_D1PNP0_SDH1_Pos)                    /*!< SCU_T::D1PNP0: SDH1 Mask               */

#define SCU_D1PNP0_EMAC0_Pos             (8)                                               /*!< SCU_T::D1PNP0: EMAC0 Position          */
#define SCU_D1PNP0_EMAC0_Msk             (0x1ul << SCU_D1PNP0_EMAC0_Pos)                   /*!< SCU_T::D1PNP0: EMAC0 Mask              */

#define SCU_D1PNP0_CRYPTO_Pos            (10)                                              /*!< SCU_T::D1PNP0: CRYPTO Position         */
#define SCU_D1PNP0_CRYPTO_Msk            (0x1ul << SCU_D1PNP0_CRYPTO_Pos)                  /*!< SCU_T::D1PNP0: CRYPTO Mask             */

#define SCU_D1PNP0_CRC_Pos               (11)                                              /*!< SCU_T::D1PNP0: CRC Position            */
#define SCU_D1PNP0_CRC_Msk               (0x1ul << SCU_D1PNP0_CRC_Pos)                     /*!< SCU_T::D1PNP0: CRC Mask                */

#define SCU_D1PNP1_KS_Pos                (0)                                               /*!< SCU_T::D1PNP1: KS Position             */
#define SCU_D1PNP1_KS_Msk                (0x1ul << SCU_D1PNP1_KS_Pos)                      /*!< SCU_T::D1PNP1: KS Mask                 */

#define SCU_D1PNP1_KDF_Pos               (1)                                               /*!< SCU_T::D1PNP1: KDF Position            */
#define SCU_D1PNP1_KDF_Msk               (0x1ul << SCU_D1PNP1_KDF_Pos)                     /*!< SCU_T::D1PNP1: KDF Mask                */

#define SCU_D1PNP1_CANFD0_Pos            (2)                                               /*!< SCU_T::D1PNP1: CANFD0 Position         */
#define SCU_D1PNP1_CANFD0_Msk            (0x1ul << SCU_D1PNP1_CANFD0_Pos)                  /*!< SCU_T::D1PNP1: CANFD0 Mask             */

#define SCU_D1PNP1_CANFD1_Pos            (4)                                               /*!< SCU_T::D1PNP1: CANFD1 Position         */
#define SCU_D1PNP1_CANFD1_Msk            (0x1ul << SCU_D1PNP1_CANFD1_Pos)                  /*!< SCU_T::D1PNP1: CANFD1 Mask             */

#define SCU_D1PNP1_EBI_Pos               (16)                                              /*!< SCU_T::D1PNP1: EBI Position            */
#define SCU_D1PNP1_EBI_Msk               (0x1ul << SCU_D1PNP1_EBI_Pos)                     /*!< SCU_T::D1PNP1: EBI Mask                */

#define SCU_D1PNP2_WWDT0_Pos             (0)                                               /*!< SCU_T::D1PNP2: WWDT0 Position          */
#define SCU_D1PNP2_WWDT0_Msk             (0x1ul << SCU_D1PNP2_WWDT0_Pos)                   /*!< SCU_T::D1PNP2: WWDT0 Mask              */

#define SCU_D1PNP2_EADC0_Pos             (1)                                               /*!< SCU_T::D1PNP2: EADC0 Position          */
#define SCU_D1PNP2_EADC0_Msk             (0x1ul << SCU_D1PNP2_EADC0_Pos)                   /*!< SCU_T::D1PNP2: EADC0 Mask              */

#define SCU_D1PNP2_EPWM0_Pos             (2)                                               /*!< SCU_T::D1PNP2: EPWM0 Position          */
#define SCU_D1PNP2_EPWM0_Msk             (0x1ul << SCU_D1PNP2_EPWM0_Pos)                   /*!< SCU_T::D1PNP2: EPWM0 Mask              */

#define SCU_D1PNP2_BPWM0_Pos             (3)                                               /*!< SCU_T::D1PNP2: BPWM0 Position          */
#define SCU_D1PNP2_BPWM0_Msk             (0x1ul << SCU_D1PNP2_BPWM0_Pos)                   /*!< SCU_T::D1PNP2: BPWM0 Mask              */

#define SCU_D1PNP2_EQEI0_Pos             (4)                                               /*!< SCU_T::D1PNP2: EQEI0 Position          */
#define SCU_D1PNP2_EQEI0_Msk             (0x1ul << SCU_D1PNP2_EQEI0_Pos)                   /*!< SCU_T::D1PNP2: EQEI0 Mask              */

#define SCU_D1PNP2_EQEI2_Pos             (5)                                               /*!< SCU_T::D1PNP2: EQEI2 Position          */
#define SCU_D1PNP2_EQEI2_Msk             (0x1ul << SCU_D1PNP2_EQEI2_Pos)                   /*!< SCU_T::D1PNP2: EQEI2 Mask              */

#define SCU_D1PNP2_ECAP0_Pos             (6)                                               /*!< SCU_T::D1PNP2: ECAP0 Position          */
#define SCU_D1PNP2_ECAP0_Msk             (0x1ul << SCU_D1PNP2_ECAP0_Pos)                   /*!< SCU_T::D1PNP2: ECAP0 Mask              */

#define SCU_D1PNP2_ECAP2_Pos             (7)                                               /*!< SCU_T::D1PNP2: ECAP2 Position          */
#define SCU_D1PNP2_ECAP2_Msk             (0x1ul << SCU_D1PNP2_ECAP2_Pos)                   /*!< SCU_T::D1PNP2: ECAP2 Mask              */

#define SCU_D1PNP2_I2C0_Pos              (8)                                               /*!< SCU_T::D1PNP2: I2C0 Position           */
#define SCU_D1PNP2_I2C0_Msk              (0x1ul << SCU_D1PNP2_I2C0_Pos)                    /*!< SCU_T::D1PNP2: I2C0 Mask               */

#define SCU_D1PNP2_I2C2_Pos              (9)                                               /*!< SCU_T::D1PNP2: I2C2 Position           */
#define SCU_D1PNP2_I2C2_Msk              (0x1ul << SCU_D1PNP2_I2C2_Pos)                    /*!< SCU_T::D1PNP2: I2C2 Mask               */

#define SCU_D1PNP2_QSPI0_Pos             (10)                                              /*!< SCU_T::D1PNP2: QSPI0 Position          */
#define SCU_D1PNP2_QSPI0_Msk             (0x1ul << SCU_D1PNP2_QSPI0_Pos)                   /*!< SCU_T::D1PNP2: QSPI0 Mask              */

#define SCU_D1PNP2_SPI0_Pos              (11)                                              /*!< SCU_T::D1PNP2: SPI0 Position           */
#define SCU_D1PNP2_SPI0_Msk              (0x1ul << SCU_D1PNP2_SPI0_Pos)                    /*!< SCU_T::D1PNP2: SPI0 Mask               */

#define SCU_D1PNP2_SPI2_Pos              (12)                                              /*!< SCU_T::D1PNP2: SPI2 Position           */
#define SCU_D1PNP2_SPI2_Msk              (0x1ul << SCU_D1PNP2_SPI2_Pos)                    /*!< SCU_T::D1PNP2: SPI2 Mask               */

#define SCU_D1PNP2_UART0_Pos             (13)                                              /*!< SCU_T::D1PNP2: UART0 Position          */
#define SCU_D1PNP2_UART0_Msk             (0x1ul << SCU_D1PNP2_UART0_Pos)                   /*!< SCU_T::D1PNP2: UART0 Mask              */

#define SCU_D1PNP2_UART2_Pos             (14)                                              /*!< SCU_T::D1PNP2: UART2 Position          */
#define SCU_D1PNP2_UART2_Msk             (0x1ul << SCU_D1PNP2_UART2_Pos)                   /*!< SCU_T::D1PNP2: UART2 Mask              */

#define SCU_D1PNP2_UART4_Pos             (15)                                              /*!< SCU_T::D1PNP2: UART4 Position          */
#define SCU_D1PNP2_UART4_Msk             (0x1ul << SCU_D1PNP2_UART4_Pos)                   /*!< SCU_T::D1PNP2: UART4 Mask              */

#define SCU_D1PNP2_UART6_Pos             (16)                                              /*!< SCU_T::D1PNP2: UART6 Position          */
#define SCU_D1PNP2_UART6_Msk             (0x1ul << SCU_D1PNP2_UART6_Pos)                   /*!< SCU_T::D1PNP2: UART6 Mask              */

#define SCU_D1PNP2_UART8_Pos             (17)                                              /*!< SCU_T::D1PNP2: UART8 Position          */
#define SCU_D1PNP2_UART8_Msk             (0x1ul << SCU_D1PNP2_UART8_Pos)                   /*!< SCU_T::D1PNP2: UART8 Mask              */

#define SCU_D1PNP2_USCI0_Pos             (18)                                              /*!< SCU_T::D1PNP2: USCI0 Position          */
#define SCU_D1PNP2_USCI0_Msk             (0x1ul << SCU_D1PNP2_USCI0_Pos)                   /*!< SCU_T::D1PNP2: USCI0 Mask              */

#define SCU_D1PNP2_SC0_Pos               (19)                                              /*!< SCU_T::D1PNP2: SC0 Position            */
#define SCU_D1PNP2_SC0_Msk               (0x1ul << SCU_D1PNP2_SC0_Pos)                     /*!< SCU_T::D1PNP2: SC0 Mask                */

#define SCU_D1PNP2_SC2_Pos               (20)                                              /*!< SCU_T::D1PNP2: SC2 Position            */
#define SCU_D1PNP2_SC2_Msk               (0x1ul << SCU_D1PNP2_SC2_Pos)                     /*!< SCU_T::D1PNP2: SC2 Mask                */

#define SCU_D1PNP2_PSIO_Pos              (21)                                              /*!< SCU_T::D1PNP2: PSIO Position           */
#define SCU_D1PNP2_PSIO_Msk              (0x1ul << SCU_D1PNP2_PSIO_Pos)                    /*!< SCU_T::D1PNP2: PSIO Mask               */

#define SCU_D1PNP2_TMR01_Pos             (22)                                              /*!< SCU_T::D1PNP2: TMR01 Position          */
#define SCU_D1PNP2_TMR01_Msk             (0x1ul << SCU_D1PNP2_TMR01_Pos)                   /*!< SCU_T::D1PNP2: TMR01 Mask              */

#define SCU_D1PNP2_DAC01_Pos             (23)                                              /*!< SCU_T::D1PNP2: DAC01 Position          */
#define SCU_D1PNP2_DAC01_Msk             (0x1ul << SCU_D1PNP2_DAC01_Pos)                   /*!< SCU_T::D1PNP2: DAC01 Mask              */

#define SCU_D1PNP2_HSOTG_Pos             (25)                                              /*!< SCU_T::D1PNP2: HSOTG Position          */
#define SCU_D1PNP2_HSOTG_Msk             (0x1ul << SCU_D1PNP2_HSOTG_Pos)                   /*!< SCU_T::D1PNP2: HSOTG Mask              */

#define SCU_D1PNP2_I2S0_Pos              (26)                                              /*!< SCU_T::D1PNP2: I2S0 Position           */
#define SCU_D1PNP2_I2S0_Msk              (0x1ul << SCU_D1PNP2_I2S0_Pos)                    /*!< SCU_T::D1PNP2: I2S0 Mask               */

#define SCU_D1PNP2_ACMP01_Pos            (27)                                              /*!< SCU_T::D1PNP2: ACMP01 Position         */
#define SCU_D1PNP2_ACMP01_Msk            (0x1ul << SCU_D1PNP2_ACMP01_Pos)                  /*!< SCU_T::D1PNP2: ACMP01 Mask             */

#define SCU_D1PNP2_USBD_Pos              (28)                                              /*!< SCU_T::D1PNP2: USBD Position           */
#define SCU_D1PNP2_USBD_Msk              (0x1ul << SCU_D1PNP2_USBD_Pos)                    /*!< SCU_T::D1PNP2: USBD Mask               */

#define SCU_D1PNP4_WWDT1_Pos             (0)                                               /*!< SCU_T::D1PNP4: WWDT1 Position          */
#define SCU_D1PNP4_WWDT1_Msk             (0x1ul << SCU_D1PNP4_WWDT1_Pos)                   /*!< SCU_T::D1PNP4: WWDT1 Mask              */

#define SCU_D1PNP4_EPWM1_Pos             (2)                                               /*!< SCU_T::D1PNP4: EPWM1 Position          */
#define SCU_D1PNP4_EPWM1_Msk             (0x1ul << SCU_D1PNP4_EPWM1_Pos)                   /*!< SCU_T::D1PNP4: EPWM1 Mask              */

#define SCU_D1PNP4_BPWM1_Pos             (3)                                               /*!< SCU_T::D1PNP4: BPWM1 Position          */
#define SCU_D1PNP4_BPWM1_Msk             (0x1ul << SCU_D1PNP4_BPWM1_Pos)                   /*!< SCU_T::D1PNP4: BPWM1 Mask              */

#define SCU_D1PNP4_EQEI1_Pos             (4)                                               /*!< SCU_T::D1PNP4: EQEI1 Position          */
#define SCU_D1PNP4_EQEI1_Msk             (0x1ul << SCU_D1PNP4_EQEI1_Pos)                   /*!< SCU_T::D1PNP4: EQEI1 Mask              */

#define SCU_D1PNP4_EQEI3_Pos             (5)                                               /*!< SCU_T::D1PNP4: EQEI3 Position          */
#define SCU_D1PNP4_EQEI3_Msk             (0x1ul << SCU_D1PNP4_EQEI3_Pos)                   /*!< SCU_T::D1PNP4: EQEI3 Mask              */

#define SCU_D1PNP4_ECAP1_Pos             (6)                                               /*!< SCU_T::D1PNP4: ECAP1 Position          */
#define SCU_D1PNP4_ECAP1_Msk             (0x1ul << SCU_D1PNP4_ECAP1_Pos)                   /*!< SCU_T::D1PNP4: ECAP1 Mask              */

#define SCU_D1PNP4_ECAP3_Pos             (7)                                               /*!< SCU_T::D1PNP4: ECAP3 Position          */
#define SCU_D1PNP4_ECAP3_Msk             (0x1ul << SCU_D1PNP4_ECAP3_Pos)                   /*!< SCU_T::D1PNP4: ECAP3 Mask              */

#define SCU_D1PNP4_I2C1_Pos              (8)                                               /*!< SCU_T::D1PNP4: I2C1 Position           */
#define SCU_D1PNP4_I2C1_Msk              (0x1ul << SCU_D1PNP4_I2C1_Pos)                    /*!< SCU_T::D1PNP4: I2C1 Mask               */

#define SCU_D1PNP4_I2C3_Pos              (9)                                               /*!< SCU_T::D1PNP4: I2C3 Position           */
#define SCU_D1PNP4_I2C3_Msk              (0x1ul << SCU_D1PNP4_I2C3_Pos)                    /*!< SCU_T::D1PNP4: I2C3 Mask               */

#define SCU_D1PNP4_QSPI1_Pos             (10)                                              /*!< SCU_T::D1PNP4: QSPI1 Position          */
#define SCU_D1PNP4_QSPI1_Msk             (0x1ul << SCU_D1PNP4_QSPI1_Pos)                   /*!< SCU_T::D1PNP4: QSPI1 Mask              */

#define SCU_D1PNP4_SPI1_Pos              (11)                                              /*!< SCU_T::D1PNP4: SPI1 Position           */
#define SCU_D1PNP4_SPI1_Msk              (0x1ul << SCU_D1PNP4_SPI1_Pos)                    /*!< SCU_T::D1PNP4: SPI1 Mask               */

#define SCU_D1PNP4_SPI3_Pos              (12)                                              /*!< SCU_T::D1PNP4: SPI3 Position           */
#define SCU_D1PNP4_SPI3_Msk              (0x1ul << SCU_D1PNP4_SPI3_Pos)                    /*!< SCU_T::D1PNP4: SPI3 Mask               */

#define SCU_D1PNP4_UART1_Pos             (13)                                              /*!< SCU_T::D1PNP4: UART1 Position          */
#define SCU_D1PNP4_UART1_Msk             (0x1ul << SCU_D1PNP4_UART1_Pos)                   /*!< SCU_T::D1PNP4: UART1 Mask              */

#define SCU_D1PNP4_UART3_Pos             (14)                                              /*!< SCU_T::D1PNP4: UART3 Position          */
#define SCU_D1PNP4_UART3_Msk             (0x1ul << SCU_D1PNP4_UART3_Pos)                   /*!< SCU_T::D1PNP4: UART3 Mask              */

#define SCU_D1PNP4_UART5_Pos             (15)                                              /*!< SCU_T::D1PNP4: UART5 Position          */
#define SCU_D1PNP4_UART5_Msk             (0x1ul << SCU_D1PNP4_UART5_Pos)                   /*!< SCU_T::D1PNP4: UART5 Mask              */

#define SCU_D1PNP4_UART7_Pos             (16)                                              /*!< SCU_T::D1PNP4: UART7 Position          */
#define SCU_D1PNP4_UART7_Msk             (0x1ul << SCU_D1PNP4_UART7_Pos)                   /*!< SCU_T::D1PNP4: UART7 Mask              */

#define SCU_D1PNP4_UART9_Pos             (17)                                              /*!< SCU_T::D1PNP4: UART9 Position          */
#define SCU_D1PNP4_UART9_Msk             (0x1ul << SCU_D1PNP4_UART9_Pos)                   /*!< SCU_T::D1PNP4: UART9 Mask              */

#define SCU_D1PNP4_SC1_Pos               (18)                                              /*!< SCU_T::D1PNP4: SC1 Position            */
#define SCU_D1PNP4_SC1_Msk               (0x1ul << SCU_D1PNP4_SC1_Pos)                     /*!< SCU_T::D1PNP4: SC1 Mask                */

#define SCU_D1PNP4_OTG_Pos               (19)                                              /*!< SCU_T::D1PNP4: OTG Position            */
#define SCU_D1PNP4_OTG_Msk               (0x1ul << SCU_D1PNP4_OTG_Pos)                     /*!< SCU_T::D1PNP4: OTG Mask                */

#define SCU_D1PNP4_KPI_Pos               (20)                                              /*!< SCU_T::D1PNP4: KPI Position            */
#define SCU_D1PNP4_KPI_Msk               (0x1ul << SCU_D1PNP4_KPI_Pos)                     /*!< SCU_T::D1PNP4: KPI Mask                */

#define SCU_D1PNP4_TMR23_Pos             (21)                                              /*!< SCU_T::D1PNP4: TMR23 Position          */
#define SCU_D1PNP4_TMR23_Msk             (0x1ul << SCU_D1PNP4_TMR23_Pos)                   /*!< SCU_T::D1PNP4: TMR23 Mask              */

#define SCU_D1PNP4_TRNG_Pos              (22)                                              /*!< SCU_T::D1PNP4: TRNG Position           */
#define SCU_D1PNP4_TRNG_Msk              (0x1ul << SCU_D1PNP4_TRNG_Pos)                    /*!< SCU_T::D1PNP4: TRNG Mask               */

#define SCU_D1PNP4_RTC_Pos               (23)                                              /*!< SCU_T::D1PNP4: RTC Position            */
#define SCU_D1PNP4_RTC_Msk               (0x1ul << SCU_D1PNP4_RTC_Pos)                     /*!< SCU_T::D1PNP4: RTC Mask                */

#define SCU_D1PNP4_I2S1_Pos              (24)                                              /*!< SCU_T::D1PNP4: I2S1 Position           */
#define SCU_D1PNP4_I2S1_Msk              (0x1ul << SCU_D1PNP4_I2S1_Pos)                    /*!< SCU_T::D1PNP4: I2S1 Mask               */

#define SCU_D1PNP4_ACMP23_Pos            (25)                                              /*!< SCU_T::D1PNP4: ACMP23 Position         */
#define SCU_D1PNP4_ACMP23_Msk            (0x1ul << SCU_D1PNP4_ACMP23_Pos)                  /*!< SCU_T::D1PNP4: ACMP23 Mask             */

#define SCU_D1PNP4_I3C0_Pos              (26)                                              /*!< SCU_T::D1PNP4: I3C0 Position           */
#define SCU_D1PNP4_I3C0_Msk              (0x1ul << SCU_D1PNP4_I3C0_Pos)                    /*!< SCU_T::D1PNP4: I3C0 Mask               */

#define SCU_D1PNP4_UTCPD_Pos             (27)                                              /*!< SCU_T::D1PNP4: UTCPD Position          */
#define SCU_D1PNP4_UTCPD_Msk             (0x1ul << SCU_D1PNP4_UTCPD_Pos)                   /*!< SCU_T::D1PNP4: UTCPD Mask              */

#define SCU_D2PNP0_LPPDMA_Pos            (0)                                               /*!< SCU_T::D2PNP0: LPPDMA Position         */
#define SCU_D2PNP0_LPPDMA_Msk            (0x1ul << SCU_D2PNP0_LPPDMA_Pos)                  /*!< SCU_T::D2PNP0: LPPDMA Mask             */

#define SCU_D2PNP0_CCAP_Pos              (1)                                               /*!< SCU_T::D2PNP0: CCAP Position           */
#define SCU_D2PNP0_CCAP_Msk              (0x1ul << SCU_D2PNP0_CCAP_Pos)                    /*!< SCU_T::D2PNP0: CCAP Mask               */

#define SCU_D2PNP0_LPGPIO_Pos            (3)                                               /*!< SCU_T::D2PNP0: LPGPIO Position         */
#define SCU_D2PNP0_LPGPIO_Msk            (0x1ul << SCU_D2PNP0_LPGPIO_Pos)                  /*!< SCU_T::D2PNP0: LPGPIO Mask             */

#define SCU_D2PNP2_LPTMR01_Pos           (0)                                               /*!< SCU_T::D2PNP2: LPTMR01 Position        */
#define SCU_D2PNP2_LPTMR01_Msk           (0x1ul << SCU_D2PNP2_LPTMR01_Pos)                 /*!< SCU_T::D2PNP2: LPTMR01 Mask            */

#define SCU_D2PNP2_TTMR01_Pos            (1)                                               /*!< SCU_T::D2PNP2: TTMR01 Position         */
#define SCU_D2PNP2_TTMR01_Msk            (0x1ul << SCU_D2PNP2_TTMR01_Pos)                  /*!< SCU_T::D2PNP2: TTMR01 Mask             */

#define SCU_D2PNP2_LPADC0_Pos            (2)                                               /*!< SCU_T::D2PNP2: LPADC0 Position         */
#define SCU_D2PNP2_LPADC0_Msk            (0x1ul << SCU_D2PNP2_LPADC0_Pos)                  /*!< SCU_T::D2PNP2: LPADC0 Mask             */

#define SCU_D2PNP2_LPI2C0_Pos            (3)                                               /*!< SCU_T::D2PNP2: LPI2C0 Position         */
#define SCU_D2PNP2_LPI2C0_Msk            (0x1ul << SCU_D2PNP2_LPI2C0_Pos)                  /*!< SCU_T::D2PNP2: LPI2C0 Mask             */

#define SCU_D2PNP2_LPSPI0_Pos            (4)                                               /*!< SCU_T::D2PNP2: LPSPI0 Position         */
#define SCU_D2PNP2_LPSPI0_Msk            (0x1ul << SCU_D2PNP2_LPSPI0_Pos)                  /*!< SCU_T::D2PNP2: LPSPI0 Mask             */

#define SCU_D2PNP2_DMIC0_Pos             (5)                                               /*!< SCU_T::D2PNP2: DMIC0 Position          */
#define SCU_D2PNP2_DMIC0_Msk             (0x1ul << SCU_D2PNP2_DMIC0_Pos)                   /*!< SCU_T::D2PNP2: DMIC0 Mask              */

#define SCU_D2PNP2_LPUART0_Pos           (6)                                               /*!< SCU_T::D2PNP2: LPUART0 Position        */
#define SCU_D2PNP2_LPUART0_Msk           (0x1ul << SCU_D2PNP2_LPUART0_Pos)                 /*!< SCU_T::D2PNP2: LPUART0 Mask            */

#define SCU_D2PNP2_WDT0_Pos              (7)                                               /*!< SCU_T::D2PNP2: WDT0 Position           */
#define SCU_D2PNP2_WDT0_Msk              (0x1ul << SCU_D2PNP2_WDT0_Pos)                    /*!< SCU_T::D2PNP2: WDT0 Mask               */

#define SCU_D2PNP2_WDT1_Pos              (8)                                               /*!< SCU_T::D2PNP2: WDT1 Position           */
#define SCU_D2PNP2_WDT1_Msk              (0x1ul << SCU_D2PNP2_WDT1_Pos)                    /*!< SCU_T::D2PNP2: WDT1 Mask               */

#define SCU_D2PNP2_AWF_Pos               (9)                                               /*!< SCU_T::D2PNP2: AWF Position            */
#define SCU_D2PNP2_AWF_Msk               (0x1ul << SCU_D2PNP2_AWF_Pos)                     /*!< SCU_T::D2PNP2: AWF Mask                */

#define SCU_D2PNP2_LPSRAMMPC_Pos         (16)                                              /*!< SCU_T::D2PNP2: LPSRAMMPC Position      */
#define SCU_D2PNP2_LPSRAMMPC_Msk         (0x1ul << SCU_D2PNP2_LPSRAMMPC_Pos)               /*!< SCU_T::D2PNP2: LPSRAMMPC Mask          */

#define SCU_IONP_PA_Pos                  (0)                                               /*!< SCU_T::IONP: PA Position               */
#define SCU_IONP_PA_Msk                  (0x1ul << SCU_IONP_PA_Pos)                        /*!< SCU_T::IONP: PA Mask                   */

#define SCU_IONP_PB_Pos                  (1)                                               /*!< SCU_T::IONP: PB Position               */
#define SCU_IONP_PB_Msk                  (0x1ul << SCU_IONP_PB_Pos)                        /*!< SCU_T::IONP: PB Mask                   */

#define SCU_IONP_PC_Pos                  (2)                                               /*!< SCU_T::IONP: PC Position               */
#define SCU_IONP_PC_Msk                  (0x1ul << SCU_IONP_PC_Pos)                        /*!< SCU_T::IONP: PC Mask                   */

#define SCU_IONP_PD_Pos                  (3)                                               /*!< SCU_T::IONP: PD Position               */
#define SCU_IONP_PD_Msk                  (0x1ul << SCU_IONP_PD_Pos)                        /*!< SCU_T::IONP: PD Mask                   */

#define SCU_IONP_PE_Pos                  (4)                                               /*!< SCU_T::IONP: PE Position               */
#define SCU_IONP_PE_Msk                  (0x1ul << SCU_IONP_PE_Pos)                        /*!< SCU_T::IONP: PE Mask                   */

#define SCU_IONP_PF_Pos                  (5)                                               /*!< SCU_T::IONP: PF Position               */
#define SCU_IONP_PF_Msk                  (0x1ul << SCU_IONP_PF_Pos)                        /*!< SCU_T::IONP: PF Mask                   */

#define SCU_IONP_PG_Pos                  (6)                                               /*!< SCU_T::IONP: PG Position               */
#define SCU_IONP_PG_Msk                  (0x1ul << SCU_IONP_PG_Pos)                        /*!< SCU_T::IONP: PG Mask                   */

#define SCU_IONP_PH_Pos                  (7)                                               /*!< SCU_T::IONP: PH Position               */
#define SCU_IONP_PH_Msk                  (0x1ul << SCU_IONP_PH_Pos)                        /*!< SCU_T::IONP: PH Mask                   */

#define SCU_IONP_PI_Pos                  (8)                                               /*!< SCU_T::IONP: PI Position               */
#define SCU_IONP_PI_Msk                  (0x1ul << SCU_IONP_PI_Pos)                        /*!< SCU_T::IONP: PI Mask                   */

#define SCU_IONP_PJ_Pos                  (9)                                               /*!< SCU_T::IONP: PJ Position               */
#define SCU_IONP_PJ_Msk                  (0x1ul << SCU_IONP_PJ_Pos)                        /*!< SCU_T::IONP: PJ Mask                   */

#define SCU_SINFAEN_SCUSIAEN_Pos         (0)                                               /*!< SCU_T::SINFAEN: SCUSIAEN Position      */
#define SCU_SINFAEN_SCUSIAEN_Msk         (0x1ul << SCU_SINFAEN_SCUSIAEN_Pos)               /*!< SCU_T::SINFAEN: SCUSIAEN Mask          */

#define SCU_SINFAEN_SYSSIAEN_Pos         (1)                                               /*!< SCU_T::SINFAEN: SYSSIAEN Position      */
#define SCU_SINFAEN_SYSSIAEN_Msk         (0x1ul << SCU_SINFAEN_SYSSIAEN_Pos)               /*!< SCU_T::SINFAEN: SYSSIAEN Mask          */

#define SCU_SINFAEN_FMCSIAEN_Pos         (2)                                               /*!< SCU_T::SINFAEN: FMCSIAEN Position      */
#define SCU_SINFAEN_FMCSIAEN_Msk         (0x1ul << SCU_SINFAEN_FMCSIAEN_Pos)               /*!< SCU_T::SINFAEN: FMCSIAEN Mask          */

#define SCU_SINFAEN_CLKSIAEN_Pos         (3)                                               /*!< SCU_T::SINFAEN: CLKSIAEN Position      */
#define SCU_SINFAEN_CLKSIAEN_Msk         (0x1ul << SCU_SINFAEN_CLKSIAEN_Pos)               /*!< SCU_T::SINFAEN: CLKSIAEN Mask          */

#define SCU_SCWP_LOCK_Pos                (1)                                               /*!< SCU_T::SCWP: LOCK Position             */
#define SCU_SCWP_LOCK_Msk                (0x1ul << SCU_SCWP_LOCK_Pos)                      /*!< SCU_T::SCWP: LOCK Mask                 */

#define SCU_SCWP_WVCODE_Pos              (16)                                              /*!< SCU_T::SCWP: WVCODE Position           */
#define SCU_SCWP_WVCODE_Msk              (0xfffful << SCU_SCWP_WVCODE_Pos)                 /*!< SCU_T::SCWP: WVCODE Mask               */

#define SCU_NSMCTL_PRESCALE_Pos          (0)                                               /*!< SCU_T::NSMCTL: PRESCALE Position       */
#define SCU_NSMCTL_PRESCALE_Msk          (0xfful << SCU_NSMCTL_PRESCALE_Pos)               /*!< SCU_T::NSMCTL: PRESCALE Mask           */

#define SCU_NSMCTL_NSMIEN_Pos            (8)                                               /*!< SCU_T::NSMCTL: NSMIEN Position         */
#define SCU_NSMCTL_NSMIEN_Msk            (0x1ul << SCU_NSMCTL_NSMIEN_Pos)                  /*!< SCU_T::NSMCTL: NSMIEN Mask             */

#define SCU_NSMCTL_AUTORLD_Pos           (9)                                               /*!< SCU_T::NSMCTL: AUTORLD Position        */
#define SCU_NSMCTL_AUTORLD_Msk           (0x1ul << SCU_NSMCTL_AUTORLD_Pos)                 /*!< SCU_T::NSMCTL: AUTORLD Mask            */

#define SCU_NSMCTL_TMRMOD_Pos            (10)                                              /*!< SCU_T::NSMCTL: TMRMOD Position         */
#define SCU_NSMCTL_TMRMOD_Msk            (0x1ul << SCU_NSMCTL_TMRMOD_Pos)                  /*!< SCU_T::NSMCTL: TMRMOD Mask             */

#define SCU_NSMCTL_IDLEON_Pos            (12)                                              /*!< SCU_T::NSMCTL: IDLEON Position         */
#define SCU_NSMCTL_IDLEON_Msk            (0x1ul << SCU_NSMCTL_IDLEON_Pos)                  /*!< SCU_T::NSMCTL: IDLEON Mask             */

#define SCU_NSMCTL_DBGON_Pos             (13)                                              /*!< SCU_T::NSMCTL: DBGON Position          */
#define SCU_NSMCTL_DBGON_Msk             (0x1ul << SCU_NSMCTL_DBGON_Pos)                   /*!< SCU_T::NSMCTL: DBGON Mask              */

#define SCU_NSMLOAD_RELOAD_Pos           (0)                                               /*!< SCU_T::NSMLOAD: RELOAD Position        */
#define SCU_NSMLOAD_RELOAD_Msk           (0xfffffful << SCU_NSMLOAD_RELOAD_Pos)            /*!< SCU_T::NSMLOAD: RELOAD Mask            */

#define SCU_NSMVAL_VALUE_Pos             (0)                                               /*!< SCU_T::NSMVAL: VALUE Position          */
#define SCU_NSMVAL_VALUE_Msk             (0xfffffful << SCU_NSMVAL_VALUE_Pos)              /*!< SCU_T::NSMVAL: VALUE Mask              */

#define SCU_NSMSTS_CURRNS_Pos            (0)                                               /*!< SCU_T::NSMSTS: CURRNS Position         */
#define SCU_NSMSTS_CURRNS_Msk            (0x1ul << SCU_NSMSTS_CURRNS_Pos)                  /*!< SCU_T::NSMSTS: CURRNS Mask             */

#define SCU_NSMSTS_NSMIF_Pos             (1)                                               /*!< SCU_T::NSMSTS: NSMIF Position          */
#define SCU_NSMSTS_NSMIF_Msk             (0x1ul << SCU_NSMSTS_NSMIF_Pos)                   /*!< SCU_T::NSMSTS: NSMIF Mask              */

#define SCU_SRAM0MPCLUT0_BLK_LUT_Pos     (0)                                               /*!< SCU_T::SRAM0MPCLUT0: BLK_LUT Position  */
#define SCU_SRAM0MPCLUT0_BLK_LUT_Msk     (0xfffffffful << SCU_SRAM0MPCLUT0_BLK_LUT_Pos)    /*!< SCU_T::SRAM0MPCLUT0: BLK_LUT Mask      */

#define SCU_SRAM1MPCLUT0_BLK_LUT_Pos     (0)                                               /*!< SCU_T::SRAM1MPCLUT0: BLK_LUT Position  */
#define SCU_SRAM1MPCLUT0_BLK_LUT_Msk     (0xfffffffful << SCU_SRAM1MPCLUT0_BLK_LUT_Pos)    /*!< SCU_T::SRAM1MPCLUT0: BLK_LUT Mask      */

#define SCU_SRAM2MPCLUT0_BLK_LUT_Pos     (0)                                               /*!< SCU_T::SRAM2MPCLUT0: BLK_LUT Position  */
#define SCU_SRAM2MPCLUT0_BLK_LUT_Msk     (0xfffffffful << SCU_SRAM2MPCLUT0_BLK_LUT_Pos)    /*!< SCU_T::SRAM2MPCLUT0: BLK_LUT Mask      */

#define SCU_SRAM3MPCLUT0_BLK_LUT_Pos     (0)                                               /*!< SCU_T::SRAM3MPCLUT0: BLK_LUT Position  */
#define SCU_SRAM3MPCLUT0_BLK_LUT_Msk     (0xfffffffful << SCU_SRAM3MPCLUT0_BLK_LUT_Pos)    /*!< SCU_T::SRAM3MPCLUT0: BLK_LUT Mask      */

#define SCU_LPSRAMMPCLUT0_BLK_LUT_Pos    (0)                                               /*!< SCU_T::LPSRAMMPCLUT0: BLK_LUT Position */
#define SCU_LPSRAMMPCLUT0_BLK_LUT_Msk    (0xfffffffful << SCU_LPSRAMMPCLUT0_BLK_LUT_Pos)   /*!< SCU_T::LPSRAMMPCLUT0: BLK_LUT Mask     */

#define SCU_SPIM0MPCLUT0_BLK_LUT_Pos     (0)                                               /*!< SCU_T::SPIM0MPCLUT0: BLK_LUT Position  */
#define SCU_SPIM0MPCLUT0_BLK_LUT_Msk     (0xfffffffful << SCU_SPIM0MPCLUT0_BLK_LUT_Pos)    /*!< SCU_T::SPIM0MPCLUT0: BLK_LUT Mask      */

#define SCU_SPIM0MPCLUT1_BLK_LUT_Pos     (0)                                               /*!< SCU_T::SPIM0MPCLUT1: BLK_LUT Position  */
#define SCU_SPIM0MPCLUT1_BLK_LUT_Msk     (0xfffffffful << SCU_SPIM0MPCLUT1_BLK_LUT_Pos)    /*!< SCU_T::SPIM0MPCLUT1: BLK_LUT Mask      */

#define SCU_SPIM0MPCLUT2_BLK_LUT_Pos     (0)                                               /*!< SCU_T::SPIM0MPCLUT2: BLK_LUT Position  */
#define SCU_SPIM0MPCLUT2_BLK_LUT_Msk     (0xfffffffful << SCU_SPIM0MPCLUT2_BLK_LUT_Pos)    /*!< SCU_T::SPIM0MPCLUT2: BLK_LUT Mask      */

#define SCU_SPIM0MPCLUT3_BLK_LUT_Pos     (0)                                               /*!< SCU_T::SPIM0MPCLUT3: BLK_LUT Position  */
#define SCU_SPIM0MPCLUT3_BLK_LUT_Msk     (0xfffffffful << SCU_SPIM0MPCLUT3_BLK_LUT_Pos)    /*!< SCU_T::SPIM0MPCLUT3: BLK_LUT Mask      */

#define SCU_SPIM0MPCLUT4_BLK_LUT_Pos     (0)                                               /*!< SCU_T::SPIM0MPCLUT4: BLK_LUT Position  */
#define SCU_SPIM0MPCLUT4_BLK_LUT_Msk     (0xfffffffful << SCU_SPIM0MPCLUT4_BLK_LUT_Pos)    /*!< SCU_T::SPIM0MPCLUT4: BLK_LUT Mask      */

#define SCU_SPIM0MPCLUT5_BLK_LUT_Pos     (0)                                               /*!< SCU_T::SPIM0MPCLUT5: BLK_LUT Position  */
#define SCU_SPIM0MPCLUT5_BLK_LUT_Msk     (0xfffffffful << SCU_SPIM0MPCLUT5_BLK_LUT_Pos)    /*!< SCU_T::SPIM0MPCLUT5: BLK_LUT Mask      */

#define SCU_SPIM0MPCLUT6_BLK_LUT_Pos     (0)                                               /*!< SCU_T::SPIM0MPCLUT6: BLK_LUT Position  */
#define SCU_SPIM0MPCLUT6_BLK_LUT_Msk     (0xfffffffful << SCU_SPIM0MPCLUT6_BLK_LUT_Pos)    /*!< SCU_T::SPIM0MPCLUT6: BLK_LUT Mask      */

#define SCU_SPIM0MPCLUT7_BLK_LUT_Pos     (0)                                               /*!< SCU_T::SPIM0MPCLUT7: BLK_LUT Position  */
#define SCU_SPIM0MPCLUT7_BLK_LUT_Msk     (0xfffffffful << SCU_SPIM0MPCLUT7_BLK_LUT_Pos)    /*!< SCU_T::SPIM0MPCLUT7: BLK_LUT Mask      */

/**@}*/ /* SCU_CONST */
/**@}*/ /* end of SCU register group */
/**@}*/ /* end of REGISTER group */

#endif /* __SCU_REG_H__ */
