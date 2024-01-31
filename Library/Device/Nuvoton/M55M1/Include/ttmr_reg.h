/**************************************************************************//**
 * @file     ttmr_reg.h
 * @version  V1.00
 * @brief    TTMR register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __TTMR_REG_H__
#define __TTMR_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup TTMR Tick Timer Controller (TTMR)
    Memory Mapped Structure for TTMR Controller
    @{
*/

typedef struct
{
    /**
     * @var TTMR_T::CTL
     * Offset: 0x00  TTMR0 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PSC       |Prescale Counter
     * |        |          |Tick Timer input clock or event source is divided by (PSC+1) before it is fed to the tick timer up counter
     * |        |          |If this field is 0 (PSC = 0), then there is no scaling.
     * |        |          |Note: Update prescale counter value will reset internal 8-bit prescale counter and 24-bit up counter value.
     * |[16]    |PDCLKEN   |Power-down Engine Clock Enable
     * |        |          |0 = Disable engine clock in power-down mode.
     * |        |          |1 = Enable engine clock in power-down mode.
     * |[20]    |PERIOSEL  |Periodic Mode Behavior Selection Enable Bit
     * |        |          |0 = The behavior selection in periodic mode is Disabled.
     * |        |          |When user updates CMPDAT while timer is running in periodic mode,
     * |        |          |CNT will be reset to default value.
     * |        |          |1 = The behavior selection in periodic mode is Enabled.
     * |        |          |When user updates CMPDAT while timer is running in periodic mode, the limitations as bellows list,
     * |        |          |If updated CMPDAT value > CNT, CMPDAT will be updated and CNT keep running continually.
     * |        |          |If updated CMPDAT value = CNT, timer time-out interrupt will be asserted immediately.
     * |        |          |If updated CMPDAT value < CNT, CNT will be reset to default value.
     * |[23]    |WKEN      |Wake-up Function Enable Bit
     * |        |          |If this bit is set to 1, while timer interrupt flag TIF (TTMRx_INTSTS[0]) is 1 and INTEN (TTMRx_CTL[29]) is enabled, the tick timer interrupt signal will generate a wake-up trigger event to CPU.
     * |        |          |0 = Wake-up function Disabled if timer interrupt signal generated.
     * |        |          |1 = Wake-up function Enabled if timer interrupt signal generated.
     * |[25]    |ACTSTS    |Tick Timer Active Status Bit (Read Only)
     * |        |          |This bit indicates the 24-bit up counter status.
     * |        |          |0 = 24-bit up counter is not active.
     * |        |          |1 = 24-bit up counter is active.
     * |        |          |Note: This bit may active when CNT 0 transition to CNT 1.
     * |[28:27] |OPMODE    |Tick Timer Counting Mode Select
     * |        |          |00 = The tick controller is operated in One-shot mode.
     * |        |          |01 = The tick controller is operated in Periodic mode.
     * |        |          |11 = The tick controller is operated in Continuous Counting mode.
     * |[29]    |INTEN     |Tick Timer Interrupt Enable Bit
     * |        |          |0 = Tick Timer time-out interrupt Disabled.
     * |        |          |1 = Tick Timer time-out interrupt Enabled.
     * |        |          |Note: If this bit is enabled, when the tick timer time-out interrupt flag TIF is set to 1, the tick timer interrupt signal is generated and inform to CPU.
     * |[30]    |CNTEN     |Tick Timer Counting Enable Bit
     * |        |          |0 = Stops/Suspends counting.
     * |        |          |1 = Starts counting.
     * |        |          |Note 1: In stop status, and then set CNTEN to 1 will enable the 24-bit up counter to keep counting from the last stop counting value.
     * |        |          |Note 2: This bit is auto-cleared by hardware in one-shot mode (TTMR_CTL[28:27] = 00) when the tick timer time-out interrupt flag TIF (TTMRx_INTSTS[0]) is generated.
     * |        |          |Note 3: Set enable/disable this bit needs 2 * TTMR_CLK period to become active, user can read ACTSTS (TTMRx_CTL[25]) to check enable/disable command is completed or not.
     * |[31]    |ICEDEBUG  |ICE Debug Mode Acknowledge Disable Bit (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgement effects TTMR counting.
     * |        |          |TTMR counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgement Disabled.
     * |        |          |TTMR counter will keep going no matter CPU is held by ICE or not.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var TTMR_T::CMP
     * Offset: 0x04  TTMR0 Comparator Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CMPDAT    |Tick Timer Comparator Value
     * |        |          |CMPDAT is a 24-bit compared value register
     * |        |          |When the internal 24-bit up counter value is equal to CMPDAT value, the TIF (TTMRx_INTSTS[0] Tick Timer Interrupt Flag) will set to 1.
     * |        |          |Time-out period = (Period of timer clock input) * (8-bit PSC + 1) * (24-bit CMPDAT).
     * |        |          |Note 1: Never write 0x0 or 0x1 in CMPDAT field, or the core will run into unknown state.
     * |        |          |Note 2: When timer is operating at continuous counting mode, the 24-bit up counter will keep counting continuously even if user writes a new value into CMPDAT field
     * |        |          |But if timer is operating at other modes, the 24-bit up counter will restart counting from 0 and using newest CMPDAT value to be the tick timer compared value while user writes a new value into CMPDAT field.
     * @var TTMR_T::INTSTS
     * Offset: 0x08  TTMR0 Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TIF       |Tick Timer Interrupt Flag
     * |        |          |This bit indicates the interrupt flag status of Tick Timer while 24-bit timer up counter CNT (TTMRx_CNT[23:0]) value reaches to CMPDAT (TTMRx_CMP[23:0]) value.
     * |        |          |0 = No effect.
     * |        |          |1 = CNT value matches the CMPDAT value.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[1]     |TWKF      |Tick Timer Wake-up Flag
     * |        |          |This bit indicates the interrupt wake-up flag status of timer.
     * |        |          |0 = Tick Timer does not cause CPU wake-up.
     * |        |          |1 = CPU wake-up from Idle or Power-down mode if timer time-out interrupt signal generated.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * @var TTMR_T::CNT
     * Offset: 0x0C  TTMR0 Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CNT       |Tick Timer Data Register
     * |        |          |Read operation.
     * |        |          |Read this register to get CNT value. For example:
     * |        |          |If EXTCNTEN (TTMRx_CTL[24]) is 0, user can read CNT value for getting current 24-bit counter value.
     * |        |          |If EXTCNTEN (TTMRx_CTL[24]) is 1, user can read CNT value for getting current 24-bit event input counter value.
     * |        |          |Write operation.
     * |        |          |Writing any value to this register will reset current CNT value to 0 and reload internal 8-bit prescale counter.
     * |[31]    |RSTACT    |Tick Timer Data Register Reset Active (Read Only)
     * |        |          |This bit indicates if the counter reset operation active.
     * |        |          |When user writes this CNT register, tick timer starts to reset its internal 24-bit timer up-counter to 0 and reload 8-bit pre-scale counter
     * |        |          |At the same time, timer set this flag to 1 to indicate the counter reset operation is in progress
     * |        |          |Once the counter reset operation done, tick timer clear this bit to 0 automatically.
     * |        |          |0 = Reset operation is done.
     * |        |          |1 = Reset operation triggered by writing TTMRx_CNT is in progress.
     * @var TTMR_T::TRGCTL
     * Offset: 0x1C  TTMR0 Trigger Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TRGEN     |Trigger Low power IPs Enable Bit
     * |        |          |If this bit is set to 1, each timer time-out event or capture event can be triggered Low Power IPs conversion.
     * |        |          |0 = Tick Timer interrupt trigger Low Power IPs Disabled.
     * |        |          |1 = Tick Timer interrupt trigger Low Power IPs Enabled.
     * |[4]     |TRGLPPDMA |Trigger LPPDMA Enable Bit
     * |        |          |If this bit is set to 1, each timer time-out event or capture event can be triggered LPPDMA transfer.
     * |        |          |0 = Tick Timer interrupt trigger LPPDMA Disabled.
     * |        |          |1 = Tick Timer interrupt trigger LPPDMA Enabled.
     * @var TTMR_T::CTL
     * Offset: 0x100  TTMR1 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PSC       |Prescale Counter
     * |        |          |Tick Timer input clock or event source is divided by (PSC+1) before it is fed to the tick timer up counter
     * |        |          |If this field is 0 (PSC = 0), then there is no scaling.
     * |        |          |Note: Update prescale counter value will reset internal 8-bit prescale counter and 24-bit up counter value.
     * |[16]    |PDCLKEN   |Power-down Engine Clock Enable
     * |        |          |0 = Disable engine clock in power-down mode.
     * |        |          |1 = Enable engine clock in power-down mode.
     * |[20]    |PERIOSEL  |Periodic Mode Behavior Selection Enable Bit
     * |        |          |0 = The behavior selection in periodic mode is Disabled.
     * |        |          |When user updates CMPDAT while timer is running in periodic mode,
     * |        |          |CNT will be reset to default value.
     * |        |          |1 = The behavior selection in periodic mode is Enabled.
     * |        |          |When user updates CMPDAT while timer is running in periodic mode, the limitations as bellows list,
     * |        |          |If updated CMPDAT value > CNT, CMPDAT will be updated and CNT keep running continually.
     * |        |          |If updated CMPDAT value = CNT, timer time-out interrupt will be asserted immediately.
     * |        |          |If updated CMPDAT value < CNT, CNT will be reset to default value.
     * |[23]    |WKEN      |Wake-up Function Enable Bit
     * |        |          |If this bit is set to 1, while timer interrupt flag TIF (TTMRx_INTSTS[0]) is 1 and INTEN (TTMRx_CTL[29]) is enabled, the tick timer interrupt signal will generate a wake-up trigger event to CPU.
     * |        |          |0 = Wake-up function Disabled if timer interrupt signal generated.
     * |        |          |1 = Wake-up function Enabled if timer interrupt signal generated.
     * |[25]    |ACTSTS    |Tick Timer Active Status Bit (Read Only)
     * |        |          |This bit indicates the 24-bit up counter status.
     * |        |          |0 = 24-bit up counter is not active.
     * |        |          |1 = 24-bit up counter is active.
     * |        |          |Note: This bit may active when CNT 0 transition to CNT 1.
     * |[28:27] |OPMODE    |Tick Timer Counting Mode Select
     * |        |          |00 = The tick controller is operated in One-shot mode.
     * |        |          |01 = The tick controller is operated in Periodic mode.
     * |        |          |10 = The tick controller is operated in Toggle-output mode.
     * |        |          |11 = The tick controller is operated in Continuous Counting mode.
     * |[29]    |INTEN     |Tick Timer Interrupt Enable Bit
     * |        |          |0 = Tick Timer time-out interrupt Disabled.
     * |        |          |1 = Tick Timer time-out interrupt Enabled.
     * |        |          |Note: If this bit is enabled, when the tick timer time-out interrupt flag TIF is set to 1, the tick timer interrupt signal is generated and inform to CPU.
     * |[30]    |CNTEN     |Tick Timer Counting Enable Bit
     * |        |          |0 = Stops/Suspends counting.
     * |        |          |1 = Starts counting.
     * |        |          |Note 1: In stop status, and then set CNTEN to 1 will enable the 24-bit up counter to keep counting from the last stop counting value.
     * |        |          |Note 2: This bit is auto-cleared by hardware in one-shot mode (TTMR_CTL[28:27] = 00) when the tick timer time-out interrupt flag TIF (TTMRx_INTSTS[0]) is generated.
     * |        |          |Note 3: Set enable/disable this bit needs 2 * TTMR_CLK period to become active, user can read ACTSTS (TTMRx_CTL[25]) to check enable/disable command is completed or not.
     * |[31]    |ICEDEBUG  |ICE Debug Mode Acknowledge Disable Bit (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgement effects TTMR counting.
     * |        |          |TTMR counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgement Disabled.
     * |        |          |TTMR counter will keep going no matter CPU is held by ICE or not.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var TTMR_T::CMP
     * Offset: 0x104  TTMR1 Comparator Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CMPDAT    |Tick Timer Comparator Value
     * |        |          |CMPDAT is a 24-bit compared value register
     * |        |          |When the internal 24-bit up counter value is equal to CMPDAT value, the TIF (TTMRx_INTSTS[0] Tick Timer Interrupt Flag) will set to 1.
     * |        |          |Time-out period = (Period of timer clock input) * (8-bit PSC + 1) * (24-bit CMPDAT).
     * |        |          |Note 1: Never write 0x0 or 0x1 in CMPDAT field, or the core will run into unknown state.
     * |        |          |Note 2: When timer is operating at continuous counting mode, the 24-bit up counter will keep counting continuously even if user writes a new value into CMPDAT field
     * |        |          |But if timer is operating at other modes, the 24-bit up counter will restart counting from 0 and using newest CMPDAT value to be the tick timer compared value while user writes a new value into CMPDAT field.
     * @var TTMR_T::INTSTS
     * Offset: 0x108  TTMR1 Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TIF       |Tick Timer Interrupt Flag
     * |        |          |This bit indicates the interrupt flag status of Tick Timer while 24-bit timer up counter CNT (TTMRx_CNT[23:0]) value reaches to CMPDAT (TTMRx_CMP[23:0]) value.
     * |        |          |0 = No effect.
     * |        |          |1 = CNT value matches the CMPDAT value.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[1]     |TWKF      |Tick Timer Wake-up Flag
     * |        |          |This bit indicates the interrupt wake-up flag status of timer.
     * |        |          |0 = Tick Timer does not cause CPU wake-up.
     * |        |          |1 = CPU wake-up from Idle or Power-down mode if timer time-out interrupt signal generated.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * @var TTMR_T::CNT
     * Offset: 0x10C  TTMR1 Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CNT       |Tick Timer Data Register
     * |        |          |Read operation.
     * |        |          |Read this register to get CNT value. For example:
     * |        |          |If EXTCNTEN (TTMRx_CTL[24]) is 0, user can read CNT value for getting current 24-bit counter value.
     * |        |          |If EXTCNTEN (TTMRx_CTL[24]) is 1, user can read CNT value for getting current 24-bit event input counter value.
     * |        |          |Write operation.
     * |        |          |Writing any value to this register will reset current CNT value to 0 and reload internal 8-bit prescale counter.
     * |[31]    |RSTACT    |Tick Timer Data Register Reset Active (Read Only)
     * |        |          |This bit indicates if the counter reset operation active.
     * |        |          |When user writes this CNT register, tick timer starts to reset its internal 24-bit timer up-counter to 0 and reload 8-bit pre-scale counter
     * |        |          |At the same time, timer set this flag to 1 to indicate the counter reset operation is in progress
     * |        |          |Once the counter reset operation done, tick timer clear this bit to 0 automatically.
     * |        |          |0 = Reset operation is done.
     * |        |          |1 = Reset operation triggered by writing TTMRx_CNT is in progress.
     * @var TTMR_T::TRGCTL
     * Offset: 0x11C  TTMR1 Trigger Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TRGEN     |Trigger Low power IPs Enable Bit
     * |        |          |If this bit is set to 1, each timer time-out event or capture event can be triggered Low Power IPs conversion.
     * |        |          |0 = Tick Timer interrupt trigger Low Power IPs Disabled.
     * |        |          |1 = Tick Timer interrupt trigger Low Power IPs Enabled.
     * |[4]     |TRGLPPDMA |Trigger LPPDMA Enable Bit
     * |        |          |If this bit is set to 1, each timer time-out event or capture event can be triggered LPPDMA transfer.
     * |        |          |0 = Tick Timer interrupt trigger LPPDMA Disabled.
     * |        |          |1 = Tick Timer interrupt trigger LPPDMA Enabled.
     */
    __IO uint32_t CTL;                   /*!< [0x0000] TTMRx Control Register                                           */
    __IO uint32_t CMP;                   /*!< [0x0004] TTMRx Comparator Register                                        */
    __IO uint32_t INTSTS;                /*!< [0x0008] TTMRx Interrupt Status Register                                  */
    __IO uint32_t CNT;                   /*!< [0x000c] TTMRx Data Register                                              */
    __I  uint32_t RESERVE0[3];
    __IO uint32_t TRGCTL;                /*!< [0x001c] TTMRx Trigger Control Register                                   */
} TTMR_T;

/**
    @addtogroup TTMR_CONST TTMR Bit Field Definition
    Constant Definitions for TTMR Controller
@{ */

#define TTMR_CTL_PSC_Pos              (0)                                         /*!< TTMR_T::CTL: PSC Position             */
#define TTMR_CTL_PSC_Msk              (0xfful << TTMR_CTL_PSC_Pos)              /*!< TTMR_T::CTL: PSC Mask                 */

#define TTMR_CTL_PDCLKEN_Pos          (16)                                      /*!< TTMR_T::CTL: PDCLKEN Position */
#define TTMR_CTL_PDCLKEN_Msk          (0x1UL << TTMR_CTL_PDCLKEN_Pos)           /*!< TTMR_T::CTL: PDCLKEN Mask     */

#define TTMR_CTL_PERIOSEL_Pos         (20)                                        /*!< TTMR_T::CTL: PERIOSEL Position        */
#define TTMR_CTL_PERIOSEL_Msk         (0x1ul << TTMR_CTL_PERIOSEL_Pos)          /*!< TTMR_T::CTL: PERIOSEL Mask            */

#define TTMR_CTL_WKEN_Pos             (23)                                        /*!< TTMR_T::CTL: WKEN Position            */
#define TTMR_CTL_WKEN_Msk             (0x1ul << TTMR_CTL_WKEN_Pos)              /*!< TTMR_T::CTL: WKEN Mask                */

#define TTMR_CTL_ACTSTS_Pos           (25)                                        /*!< TTMR_T::CTL: ACTSTS Position          */
#define TTMR_CTL_ACTSTS_Msk           (0x1ul << TTMR_CTL_ACTSTS_Pos)            /*!< TTMR_T::CTL: ACTSTS Mask              */

#define TTMR_CTL_OPMODE_Pos           (27)                                        /*!< TTMR_T::CTL: OPMODE Position          */
#define TTMR_CTL_OPMODE_Msk           (0x3ul << TTMR_CTL_OPMODE_Pos)            /*!< TTMR_T::CTL: OPMODE Mask              */

#define TTMR_CTL_INTEN_Pos            (29)                                        /*!< TTMR_T::CTL: INTEN Position           */
#define TTMR_CTL_INTEN_Msk            (0x1ul << TTMR_CTL_INTEN_Pos)             /*!< TTMR_T::CTL: INTEN Mask               */

#define TTMR_CTL_CNTEN_Pos            (30)                                        /*!< TTMR_T::CTL: CNTEN Position           */
#define TTMR_CTL_CNTEN_Msk            (0x1ul << TTMR_CTL_CNTEN_Pos)             /*!< TTMR_T::CTL: CNTEN Mask               */

#define TTMR_CTL_ICEDEBUG_Pos         (31)                                        /*!< TTMR_T::CTL: ICEDEBUG Position        */
#define TTMR_CTL_ICEDEBUG_Msk         (0x1ul << TTMR_CTL_ICEDEBUG_Pos)          /*!< TTMR_T::CTL: ICEDEBUG Mask            */

#define TTMR_CMP_CMPDAT_Pos           (0)                                         /*!< TTMR_T::CMP: CMPDAT Position          */
#define TTMR_CMP_CMPDAT_Msk           (0xfffffful << TTMR_CMP_CMPDAT_Pos)       /*!< TTMR_T::CMP: CMPDAT Mask              */

#define TTMR_INTSTS_TIF_Pos           (0)                                         /*!< TTMR_T::INTSTS: TIF Position          */
#define TTMR_INTSTS_TIF_Msk           (0x1ul << TTMR_INTSTS_TIF_Pos)            /*!< TTMR_T::INTSTS: TIF Mask              */

#define TTMR_INTSTS_TWKF_Pos          (1)                                         /*!< TTMR_T::INTSTS: TWKF Position         */
#define TTMR_INTSTS_TWKF_Msk          (0x1ul << TTMR_INTSTS_TWKF_Pos)           /*!< TTMR_T::INTSTS: TWKF Mask             */

#define TTMR_CNT_CNT_Pos              (0)                                         /*!< TTMR_T::CNT: CNT Position             */
#define TTMR_CNT_CNT_Msk              (0xfffffful << TTMR_CNT_CNT_Pos)          /*!< TTMR_T::CNT: CNT Mask                 */

#define TTMR_CNT_RSTACT_Pos           (31)                                        /*!< TTMR_T::CNT: RSTACT Position          */
#define TTMR_CNT_RSTACT_Msk           (0x1ul << TTMR_CNT_RSTACT_Pos)            /*!< TTMR_T::CNT: RSTACT Mask              */

#define TTMR_TRGCTL_TRGEN_Pos         (1)                                         /*!< TTMR_T::TRGCTL: TRGEN Position        */
#define TTMR_TRGCTL_TRGEN_Msk         (0x1ul << TTMR_TRGCTL_TRGEN_Pos)          /*!< TTMR_T::TRGCTL: TRGEN Mask            */

#define TTMR_TRGCTL_TRGLPPDMA_Pos     (4)                                         /*!< TTMR_T::TRGCTL: TRGLPPDMA Position        */
#define TTMR_TRGCTL_TRGLPPDMA_Msk     (0x1ul << TTMR_TRGCTL_TRGLPPDMA_Pos)      /*!< TTMR_T::TRGCTL: TRGLPPDMA Mask            */

/** @} TTMR_CONST */
/** @} end of TTMR register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __TTMR_REG_H__ */
