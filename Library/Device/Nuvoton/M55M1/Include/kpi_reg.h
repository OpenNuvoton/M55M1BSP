/**************************************************************************//**
 * @file     kpi_reg.h
 * @version  V1.00
 * @brief    KPI register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __KEYPAD_REG_H__
#define __KEYPAD_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup KPI Keypad Control Interface (KPI)
    Memory Mapped Structure for KPI Controller
    @{
*/

typedef struct
{
    /**
     * @var KPI_T::CTL
     * Offset: 0x00  Keypad Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |KPEN      |Keypad Scan Enable Bit
     * |        |          |Setting this bit high enables the key scan function.
     * |        |          |0 = Keypad scan Disabled.
     * |        |          |1 = Keypad scan Enabled.
     * |[1]     |KPIEN     |Key Press Key Interrupt Enable Bit
     * |        |          |The keypad controller will generate an interrupt when the controller detects any effective key press.
     * |        |          |0 = Keypad press interrupt Disabled.
     * |        |          |1 = Keypad press interrupt Enabled.
     * |[2]     |KRIEN     |Key Release Key Interrupt Enable Bit
     * |        |          |The keypad controller will generate an interrupt when the controller detects keypad status changes from press to release.
     * |        |          |0 = Keypad release interrupt Disabled.
     * |        |          |1 = Keypad release interrupt Enabled.
     * |[3]     |KIEN      |Key Interrupt Enable Bit
     * |        |          |0 = Keypad interrupt Disabled.
     * |        |          |1 = Keypad interrupt Enabled.
     * |[5:4]   |KDBCT     |Keypad De-bounce Cycle Time
     * |        |          |For keypad debounce, keypad will generate an interrupt when key press, key release or three key reset continued n * key array scan time.
     * |        |          |00 = n=1.
     * |        |          |01 = n=2.
     * |        |          |10 = n=3.
     * |        |          |11 = n=4.
     * |        |          |Note: It would need more time to indicate key press and release event when users selected more debounce cycle time.
     * |[15:8]  |PSC       |Row Scan Cycle Pre-scale Value
     * |        |          |This value is used to pre-scale row scan cycle.
     * |        |          |The pre-scale counter is clocked by the divided crystal clock, xCLOCK.
     * |        |          |The divided number is from 1 to 256.
     * |        |          |E.g.If the crystal clock is 1Mhz then the xCLOCK period is 1us.
     * |        |          |If the keypad matric is 3x3 then
     * |        |          |Each row scan time = xCLOCK x PRESCALE PSC x PrescaleDividerPSCDIV.
     * |        |          |Key array scan time = Each row scan time x ROWS.
     * |        |          |Example scan time for PRESCALE = 0x40, and PrescaleDividerPSCDIV = 0x1F.
     * |        |          |Each row scan time = 1us x 65 x 32 = 2.08ms.
     * |        |          |Scan time = 2.08 x 3 = 6.24ms.
     * |[19:16] |DBCLKSEL  |Scan in De-bounce Sampling Cycle Selection
     * |        |          |0000 = Reserved.
     * |        |          |0001 = Reserved.
     * |        |          |0010 = Reserved.
     * |        |          |0011 = Sample interrupt input once per 8 XCLOCK.
     * |        |          |0100 = Sample interrupt input once per 16 XCLOCK.
     * |        |          |0101 = Sample interrupt input once per 32 XCLOCK.
     * |        |          |0110 = Sample interrupt input once per 64 XCLOCK.
     * |        |          |0111 = Sample interrupt input once per 128 XCLOCK.
     * |        |          |1000 = Sample interrupt input once per 256 XCLOCK.
     * |        |          |1001 = Sample interrupt input once per 512 XCLOCK.
     * |        |          |1010 = Sample interrupt input once per 1024 XCLOCK.
     * |        |          |1011 = Sample interrupt input once per 2048 XCLOCK.
     * |        |          |1100 = Sample interrupt input once per 4096 XCLOCK.
     * |        |          |1101 = Sample interrupt input once per 8192 XCLOCK.
     * |        |          |1110 = reserved.
     * |        |          |1111 = reserved.
     * |[23:22] |ROWDLY    |Scan Row Delay
     * |        |          |Setting delay cycle when row change, for avoid KPI from detecting wrong key..
     * |        |          |00 = 4 XCLOCK.
     * |        |          |01 = 8 XCLOCK.
     * |        |          |10 = 16 XCLOCK.
     * |        |          |11 = 32 XCLOCK.
     * |[26:24] |KCOL      |Keypad Matrix COL Number
     * |        |          |The keypad matrix is set by ROW x COL. The COL number can be set 1 to 8.
     * |        |          |000 = 1.
     * |        |          |001 = 2.
     * |        |          |010 = 3.
     * |        |          |011 = 4.
     * |        |          |100 = 5.
     * |        |          |101 = 6.
     * |        |          |110 = 7.
     * |        |          |111 = 8.
     * |[30:28] |KROW      |Keypad Matrix ROW Number
     * |        |          |The keypad matrix is set by ROW x COL. The ROW number can be set 2 to 6.
     * |        |          |000 = reserved.
     * |        |          |001 = 2.
     * |        |          |010 = 3.
     * |        |          |011 = 4.
     * |        |          |100 = 5.
     * |        |          |101 = 6.
     * |        |          |110 = Reserved.
     * |        |          |111 = Reserved.
     * @var KPI_T::STATUS
     * Offset: 0x08  Keypad Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2]     |KIF       |Key Interrupt Flag
     * |        |          |This bit indicates the key scan interrupt is active when any key press or, key release or three key reset or wake up.
     * |        |          |When READ:
     * |        |          |0 = No reset.
     * |        |          |1 = Key press/Key release interrupt occurred.
     * |        |          |To clear KIF, software must clear KPIF and KRIF.
     * |[3]     |KRIF      |Key Release Interrupt Flag
     * |        |          |This bit indicates that some keys (one or multiple key) have been released.
     * |        |          |When READ:
     * |        |          |0 = No key release.
     * |        |          |1 = At least one key release.
     * |        |          |Note: To clear KRIF, software must clear each releasing flag that are shown on KPI_KRF0/1 registers.
     * |[4]     |KPIF      |Key Press Interrupt Flag
     * |        |          |This bit indicates that some keys (one or multiple key) have been pressed.
     * |        |          |When READ:
     * |        |          |0 = No key press.
     * |        |          |1 = At least one key press.
     * |        |          |Note: To clear KPIF, software must clear each pressing flag that are shown on KPI_KPF0/1 registers.
     * @var KPI_T::KST[2]
     * Offset: 0x10 ~ 0x14  Keypad State Register 0 ~ 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |KSTmn     |Key State
     * |        |          |m is row number, n is column number.
     * |        |          |0 = Key is pressing.
     * |        |          |1 = Key is releasing.
     * @var KPI_T::KPF[2]
     * Offset: 0x18 ~0x1c  Key Press Flag Register 0 ~ 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |KPFmn     |Key Press Flag
     * |        |          |m is row number, n is column number.
     * |        |          |0 = No key event.
     * |        |          |1 = Corresponding key has a high to low event change.
     * |        |          |Note: This bit will be set by hardware, and should be cleared by software writing 1.
     * @var KPI_T::KRF[2]
     * Offset: 0x20 ~ 0x24  Key Release Flag Register 0 ~ 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |KRFmn     |Key Release Event Change IndicatorFlag
     * |        |          |KRE mn[X] = 1, m= is row number, n= is column number.:.
     * |        |          |0 = No key event.
     * |        |          |1 = Corresponding key has a low to high event change.
     * |        |          |Note:
     * |        |          |Hardware will set this bit, and software should clear this bit by writing 1.
     * |        |          |Software can clear RKEYINT (KPISTATUS[3]) by writing 1 bit by bit to this register.
     * @var KPI_T::DLYCTL
     * Offset: 0x28  Delay Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PSCDIV    |Pre-scale Divider
     * |        |          |This value is used to divide RESCALE that is set in KPICONFKPI_CTL[15:8]
     * |        |          |The prescale divider counter is clocked by the divided crystal clock, xCLOCK
     * |        |          |The number is from 1 to 256.
     * |        |          |E.g. If the crystal clock is 1Mhz then the xCLOCK period is 1us. If the keypad matrix is 3x3. Then,
     * |        |          |each row scan time = xCLOCK x PRESCALEPSC x PrescaleDividerPSCDIV.
     * |        |          |key array scan time = each row scan time x ROWS.
     * |        |          |example scan time for PRESCALEPSC = 0x40, and PrescaleDividerPSCDIV = 0x1F.
     * |        |          |each row scan time = 1us x 65 x 32 = 2.08ms.
     * |        |          |scan time = 2.08 x 3 = 6.24ms.
     * |        |          |Note:
     * |        |          |When PRESCALEPSC (KPICONFKPI_CTL[15:8]) is determined, De-bounce sampling cycle should not exceed the half of (PRESCALEPSC x PrescaleDividerPSCDIV),
     * |        |          |in the above example, and if scan row delay cycle is 4 xclock
     * |        |          |The maximum DBCLKSEL(KPICONFKPI_CTL[19:16]) should be 1024 x clock, bouncing time is 1ms.
     * |[31:8]  |SCANDLY   |Key Array Scan Delay
     * |        |          |This value is used to insert delay cycle between each key array scan.
     * |        |          |The key array scan delay counter is clocked by the divided crystal clock, xCLOCK.
     * |        |          |Key array scan delay time = xCLOCK x KASDSCANDLY.
     * |        |          |The number of key array scan delay cycle is 0 and from 2 to 1024.
     * |        |          |0 = No delay.
     * |        |          |Others = others + 1 cycles.
     * |        |          |Note:
     * |        |          |If the key array scan delay is set to 0, there are no delay between each key array scan.
     * |        |          |There are no delay 1 cycle situation.
     */
    __IO uint32_t CTL;                   /*!< [0x0000] Keypad Control Register                                          */
    __IO uint32_t Reserved0;             /*!< [0x0004] Reserved                                                         */
    __IO uint32_t STATUS;                /*!< [0x0008] Keypad Status Register                                           */
    __IO uint32_t Reserved1;             /*!< [0x000c] Reserved                                                         */
    __I  uint32_t KST[2];                /*!< [0x0010-0x0014] Keypad State Register 0                                   */
    __IO uint32_t KPF[2];                /*!< [0x0018-0x001c] Lower 32 Key Press Flag Register 0                        */
    __IO uint32_t KRF[2];                /*!< [0x0020-0x0024] Lower 32 Key Release Flag Register 0                      */
    __IO uint32_t DLYCTL;                /*!< [0x0028] Delay Control Register                                           */

} KPI_T;

/**
    @addtogroup KPI_CONST KPI Bit Field Definition
    Constant Definitions for KPI Controller
@{ */

#define KPI_CTL_KPEN_Pos                 (0)                                               /*!< KPI_T::CTL: KPEN Position              */
#define KPI_CTL_KPEN_Msk                 (0x1ul << KPI_CTL_KPEN_Pos)                       /*!< KPI_T::CTL: KPEN Mask                  */

#define KPI_CTL_KPIEN_Pos                (1)                                               /*!< KPI_T::CTL: KPIEN Position             */
#define KPI_CTL_KPIEN_Msk                (0x1ul << KPI_CTL_KPIEN_Pos)                      /*!< KPI_T::CTL: KPIEN Mask                 */

#define KPI_CTL_KRIEN_Pos                (2)                                               /*!< KPI_T::CTL: KRIEN Position             */
#define KPI_CTL_KRIEN_Msk                (0x1ul << KPI_CTL_KRIEN_Pos)                      /*!< KPI_T::CTL: KRIEN Mask                 */

#define KPI_CTL_KIEN_Pos                 (3)                                               /*!< KPI_T::CTL: KIEN Position              */
#define KPI_CTL_KIEN_Msk                 (0x1ul << KPI_CTL_KIEN_Pos)                       /*!< KPI_T::CTL: KIEN Mask                  */

#define KPI_CTL_KDBCT_Pos                (4)                                               /*!< KPI_T::CTL: KDBCT Position             */
#define KPI_CTL_KDBCT_Msk                (0x3ul << KPI_CTL_KDBCT_Pos)                      /*!< KPI_T::CTL: KDBCT Mask                 */

#define KPI_CTL_PSC_Pos                  (8)                                               /*!< KPI_T::CTL: PSC Position               */
#define KPI_CTL_PSC_Msk                  (0xfful << KPI_CTL_PSC_Pos)                       /*!< KPI_T::CTL: PSC Mask                   */

#define KPI_CTL_DBCLKSEL_Pos             (16)                                              /*!< KPI_T::CTL: DBCLKSEL Position          */
#define KPI_CTL_DBCLKSEL_Msk             (0xful << KPI_CTL_DBCLKSEL_Pos)                   /*!< KPI_T::CTL: DBCLKSEL Mask              */

#define KPI_CTL_ROWDLY_Pos               (22)                                              /*!< KPI_T::CTL: ROWDLY Position            */
#define KPI_CTL_ROWDLY_Msk               (0x3ul << KPI_CTL_ROWDLY_Pos)                     /*!< KPI_T::CTL: ROWDLY Mask                */

#define KPI_CTL_KCOL_Pos                 (24)                                              /*!< KPI_T::CTL: KCOL Position              */
#define KPI_CTL_KCOL_Msk                 (0x7ul << KPI_CTL_KCOL_Pos)                       /*!< KPI_T::CTL: KCOL Mask                  */

#define KPI_CTL_KROW_Pos                 (28)                                              /*!< KPI_T::CTL: KROW Position              */
#define KPI_CTL_KROW_Msk                 (0x7ul << KPI_CTL_KROW_Pos)                       /*!< KPI_T::CTL: KROW Mask                  */

#define KPI_STATUS_KIF_Pos               (2)                                               /*!< KPI_T::STATUS: KIF Position            */
#define KPI_STATUS_KIF_Msk               (0x1ul << KPI_STATUS_KIF_Pos)                     /*!< KPI_T::STATUS: KIF Mask                */

#define KPI_STATUS_KRIF_Pos              (3)                                               /*!< KPI_T::STATUS: KRIF Position           */
#define KPI_STATUS_KRIF_Msk              (0x1ul << KPI_STATUS_KRIF_Pos)                    /*!< KPI_T::STATUS: KRIF Mask               */

#define KPI_STATUS_KPIF_Pos              (4)                                               /*!< KPI_T::STATUS: KPIF Position           */
#define KPI_STATUS_KPIF_Msk              (0x1ul << KPI_STATUS_KPIF_Pos)                    /*!< KPI_T::STATUS: KPIF Mask               */

#define KPI_KST0_KSTmn_Pos               (0)                                               /*!< KPI_T::KST0: KSTmn Position            */
#define KPI_KST0_KSTmn_Msk               (0xfffffffful << KPI_KST0_KSTmn_Pos)              /*!< KPI_T::KST0: KSTmn Mask                */

#define KPI_KST1_KSTmn_Pos               (0)                                               /*!< KPI_T::KST1: KSTmn Position            */
#define KPI_KST1_KSTmn_Msk               (0xfffful << KPI_KST1_KSTmn_Pos)                  /*!< KPI_T::KST1: KSTmn Mask                */

#define KPI_KPF0_KPFmn_Pos               (0)                                               /*!< KPI_T::KPF0: KPFmn Position            */
#define KPI_KPF0_KPFmn_Msk               (0xfffffffful << KPI_KPF0_KPFmn_Pos)              /*!< KPI_T::KPF0: KPFmn Mask                */

#define KPI_KPF1_KPEmn_Pos               (0)                                               /*!< KPI_T::KPF1: KPEmn Position            */
#define KPI_KPF1_KPEmn_Msk               (0xfffful << KPI_KPF1_KPEmn_Pos)                  /*!< KPI_T::KPF1: KPEmn Mask                */

#define KPI_KRF0_KRFmn_Pos               (0)                                               /*!< KPI_T::KRF0: KRFmn Position            */
#define KPI_KRF0_KRFmn_Msk               (0xfffffffful << KPI_KRF0_KRFmn_Pos)              /*!< KPI_T::KRF0: KRFmn Mask                */

#define KPI_KRF1_KRFmn_Pos               (0)                                               /*!< KPI_T::KRF1: KRFmn Position            */
#define KPI_KRF1_KRFmn_Msk               (0xfffful << KPI_KRF1_KRFmn_Pos)                  /*!< KPI_T::KRF1: KRFmn Mask                */

#define KPI_DLYCTL_PSCDIV_Pos            (0)                                               /*!< KPI_T::DLYCTL: PSCDIV Position         */
#define KPI_DLYCTL_PSCDIV_Msk            (0xfful << KPI_DLYCTL_PSCDIV_Pos)                 /*!< KPI_T::DLYCTL: PSCDIV Mask             */

#define KPI_DLYCTL_SCANDLY_Pos           (8)                                               /*!< KPI_T::DLYCTL: SCANDLY Position        */
#define KPI_DLYCTL_SCANDLY_Msk           (0xfffffful << KPI_DLYCTL_SCANDLY_Pos)            /*!< KPI_T::DLYCTL: SCANDLY Mask            */

/** @} KPI_CONST */
/** @} end of KPI register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __KEYPAD_REG_H__ */
