/**************************************************************************//**
 * @file     lppdma_reg.h
 * @version  V1.00
 * @brief    LPPDMA register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __LPPDMA_REG_H__
#define __LPPDMA_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup LPPDMA Low Power Peripheral Direct Memory Access Controller (LPPDMA)
    Memory Mapped Structure for LPPDMA Controller
    @{
*/

typedef struct
{
    /**
     * @var LPDSCT_T::CTL
     * Offset: 0x00  Descriptor Table Control Register of LPPDMA Channel n
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |OPMODE    |LPPDMA Operation Mode Selection
     * |        |          |00 = Idle state: Channel is stopped or this table is complete, when LPPDMA finish channel table task, OPMODE will be cleared to idle state automatically.
     * |        |          |01 = Basic mode: The descriptor table only has one task
     * |        |          |When this task is finished, the LPPDMA_INTSTS[1] will be asserted.
     * |        |          |10 = Scatter-gather mode: When operating in this mode, user must give the next descriptor table address in LPPDMA_DSCTn_NEXT register; LPPDMA controller will ignore this task, then load the next task to execute.
     * |        |          |11 = Reserved.
     * |        |          |Note: Before filling new transfer task in the Descriptor Table, user must check the LPPDMA_INTSTS[1] to make sure the current task is complete.
     * |[2]     |TXTYPE    |Transfer Type
     * |        |          |0 = Burst transfer type.
     * |        |          |1 = Single transfer type.
     * |[6:4]   |BURSIZE   |Burst Size
     * |        |          |This field is used for peripheral to determine the burst size or used for determine the re-arbitration size.
     * |        |          |000 = 128 Transfers.
     * |        |          |001 = 64 Transfers.
     * |        |          |010 = 32 Transfers.
     * |        |          |011 = 16 Transfers.
     * |        |          |100 = 8 Transfers.
     * |        |          |101 = 4 Transfers.
     * |        |          |110 = 2 Transfers.
     * |        |          |111 = 1 Transfers.
     * |        |          |Note: This field is only useful in burst transfer type.
     * |[7]     |TBINTDIS  |Table Interrupt Disable Bit
     * |        |          |This field can be used to decide whether to enable table interrupt or not
     * |        |          |If the TBINTDIS bit is enabled it will not generates TDIFn(LPPDMA_TDSTS[7:0]) when LPPDMA controller finishes transfer task.
     * |        |          |0 = Table interrupt Enabled.
     * |        |          |1 = Table interrupt Disabled.
     * |        |          |Note: This function only for Scatter-gather mode.
     * |[9:8]   |SAINC     |Source Address Increment
     * |        |          |This field is used to set the source address increment size.
     * |        |          |11 = No increment (fixed address).
     * |        |          |Others = Increment and size is depended on TXWIDTH selection.
     * |        |          |Note: The fixed address function does not support in memory to memory transfer type.
     * |[11:10] |DAINC     |Destination Address Increment
     * |        |          |This field is used to set the destination address increment size.
     * |        |          |11 = No increment (fixed address).
     * |        |          |Others = Increment and size is depended on TXWIDTH selection.
     * |        |          |Note: The fixed address function does not support in memory to memory transfer type.
     * |[13:12] |TXWIDTH   |Transfer Width Selection
     * |        |          |This field is used for transfer width.
     * |        |          |00 = One byte (8 bit) is transferred for every operation.
     * |        |          |01= One half-word (16 bit) is transferred for every operation.
     * |        |          |10 = One word (32-bit) is transferred for every operation.
     * |        |          |11 = Reserved.
     * |        |          |Note: The LPPDMA transfer source address (LPPDMA_DSCTn_SA) and LPPDMA transfer destination address (LPPDMA_DSCTn_DA) should be alignment under the TXWIDTH selection
     * |[31:16] |TXCNT     |Transfer Count
     * |        |          |The TXCNT represents the required number of LPPDMA transfer, the real transfer count is (TXCNT + 1); The maximum transfer count is 65536, every transfer may be byte, half-word or word that is dependent on TXWIDTH field.
     * |        |          |Note: When LPPDMA finishes each transfer data, this field will be decreased immediately.
     * @var LPDSCT_T::SA
     * Offset: 0x04  Source Address Register of LPPDMA Channel n
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |SA        |LPPDMA Transfer Source Address
     * |        |          |This field indicates a 32-bit source address of LPPDMA controller.
     * @var LPDSCT_T::DA
     * Offset: 0x08  Destination Address Register of LPPDMA Channel n
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |DA        |LPPDMA Transfer Destination Address
     * |        |          |This field indicates a 32-bit destination address of LPPDMA controller.
     * @var LPDSCT_T::NEXT
     * Offset: 0x0C  Next Scatter-gather Descriptor Table Offset Address of LPPDMA Channel n
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |NEXT      |LPPDMA Next Descriptor Table Offset
     * |        |          |This field indicates the offset of the next descriptor table address in system memory.
     * |        |          |Read Operation:
     * |        |          |When operating in Scatter-gather mode, the last two bits NEXT[1:0] will become reserved, and indicate the first next address of system memory.
     * |        |          |Note 1: The descriptor table address must be word boundary.
     * |        |          |Note 2: Before filling transfer task in the descriptor table, user must check if the descriptor table is complete.
     * |[31:16] |EXENEXT   |LPPDMA Execution Next Descriptor Table Offset
     * |        |          |This field indicates the offset of next descriptor table address of current execution descriptor table in system memory.
     * |        |          |Note: Write operation is useless in this field.
         */
    __IO uint32_t CTL;             /*!< [0x0000] Descriptor Table Control Register of LPPDMA Channel n.             */
    __IO uint32_t SA;              /*!< [0x0004] Source Address Register of LPPDMA Channel n                        */
    __IO uint32_t DA;              /*!< [0x0008] Destination Address Register of LPPDMA Channel n                   */
    __IO uint32_t NEXT;            /*!< [0x000c] Next Scatter-gather Descriptor Table Offset Address of LPPDMA Channel n */
} LPDSCT_T;

typedef struct
{

    /**
     * @var LPPDMA_T::CURSCAT
     * Offset: 0x80  Current Scatter-gather Descriptor Table Address of LPPDMA Channel n
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |CURADDR   |LPPDMA Current Description Address (Read Only)
     * |        |          |This field indicates a 32-bit current external description address of LPPDMA controller.
     * |        |          |Note: This field is read only and used for Scatter-gather mode only to indicate the current external description address.
     * @var LPPDMA_T::CHCTL
     * Offset: 0x400  LPPDMA Channel Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |CHENn     |LPPDMA Channel Enable Bits
     * |        |          |Set this bit to 1 to enable LPPDMAn operation. Channel cannot be active if it is not set as enabled.
     * |        |          |0 = LPPDMA channel [n] Disabled.
     * |        |          |1 = LPPDMA channel [n] Enabled.
     * |        |          |Note: Setting the corresponding bit of LPPDMA_PAUSE or LPPDMA_CHRST register will also clear this bit.
     * @var LPPDMA_T::PAUSE
     * Offset: 0x404  LPPDMA Transfer Pause Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PAUSEn    |LPPDMA Channel n Transfer Pause Control (Write Only)
     * |        |          |User can set PAUSEn bit field to pause the LPPDMA transfer
     * |        |          |When user sets PAUSEn bit, the LPPDMA controller will pause the on-going transfer, then clear the channel enable bit CHENn(LPPDMA_CHCTL [n], n=0,1..3) and clear request active flag(LPPDMA_TRGSTS[n:0], n=0,1..3)
     * |        |          |If the paused channel is re-enabled again, the remaining transfers will be processed.
     * |        |          |0 = No effect.
     * |        |          |1 = Pause LPPDMA channel n transfer.
     * @var LPPDMA_T::SWREQ
     * Offset: 0x408  LPPDMA Software Request Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |SWREQn    |LPPDMA Software Request (Write Only)
     * |        |          |Set this bit to 1 to generate a software request to LPPDMA [n].
     * |        |          |0 = No effect.
     * |        |          |1 = Generate a software request.
     * |        |          |Note 1: User can read LPPDMA_TRGSTS register to know which channel is on active
     * |        |          |Active flag may be triggered by software request or peripheral request.
     * |        |          |Note 2: If user does not enable corresponding LPPDMA channel, the software request will be ignored.
     * @var LPPDMA_T::TRGSTS
     * Offset: 0x40C  LPPDMA Channel Request Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |REQSTSn   |LPPDMA Channel Request Status (Read Only)
     * |        |          |This flag indicates whether channel[n] have a request or not, no matter request from software or peripheral
     * |        |          |When LPPDMA controller finishes channel transfer, this bit will be cleared automatically.
     * |        |          |0 = LPPDMA Channel n has no request.
     * |        |          |1 = LPPDMA Channel n has a request.
     * |        |          |Note: If user pauses or resets each LPPDMA transfer by setting LPPDMA_PAUSE or LPPDMA_CHRST register respectively, this bit will be cleared automatically after finishing the current transfer.
     * @var LPPDMA_T::PRISET
     * Offset: 0x410  LPPDMA Fixed Priority Setting Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |FPRISETn  |LPPDMA Fixed Priority Setting
     * |        |          |Set this bit to 1 to enable fixed priority level.
     * |        |          |Write Operation:
     * |        |          |0 = No effect.
     * |        |          |1 = Set LPPDMA channel [n] to fixed priority channel.
     * |        |          |Read Operation:
     * |        |          |0 = Corresponding LPPDMA channel is round-robin priority.
     * |        |          |1 = Corresponding LPPDMA channel is fixed priority.
     * |        |          |Note: This field is only set to fixed priority. To clear fixed priority, use LPPDMA_PRICLR register.
     * @var LPPDMA_T::PRICLR
     * Offset: 0x414  LPPDMA Fixed Priority Clear Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |FPRICLRn  |LPPDMA Fixed Priority Clear Bits (Write Only)
     * |        |          |Set this bit to 1 to clear fixed priority level.
     * |        |          |0 = No effect.
     * |        |          |1 = Clear LPPDMA channel [n] fixed priority setting.
     * |        |          |Note: User can read LPPDMA_PRISET register to know the channel priority.
     * @var LPPDMA_T::INTEN
     * Offset: 0x418  LPPDMA Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |INTENn    |LPPDMA Interrupt Enable Bits
     * |        |          |This field is used to enable LPPDMA channel[n] interrupt.
     * |        |          |0 = LPPDMA channel n interrupt Disabled.
     * |        |          |1 = LPPDMA channel n interrupt Enabled.
     * |        |          |Note: The interrupt flag is time-out, abort, transfer done and align.
     * @var LPPDMA_T::INTSTS
     * Offset: 0x41C  LPPDMA Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ABTIF     |LPPDMA Read/Write Target Abort Interrupt Flag (Read Only)
     * |        |          |This bit indicates that LPPDMA has target abort error; Software can read LPPDMA_ABTSTS register to find which channel has target abort error.
     * |        |          |0 = No AHB bus ERROR response received.
     * |        |          |1 = AHB bus ERROR response received.
     * |[1]     |TDIF      |Transfer Done Interrupt Flag (Read Only)
     * |        |          |This bit indicates that LPPDMA controller has finished transmission; User can read LPPDMA_TDSTS register to indicate which channel finished transfer.
     * |        |          |0 = Not finished yet.
     * |        |          |1 = LPPDMA channel has finished transmission.
     * |[2]     |ALIGNF    |Transfer Alignment Interrupt Flag (Read Only)
     * |        |          |0 = LPPDMA channel source address and destination address both follow transfer width setting.
     * |        |          |1 = LPPDMA channel source address or destination address is not follow transfer width setting.
     * |[3]     |WKF       |Wake Up Flag
     * |        |          |0 = LPPDMA no wake up event.
     * |        |          |1 = LPPDMA wake up event happened.
     * @var LPPDMA_T::ABTSTS
     * Offset: 0x420  LPPDMA Channel Read/Write Target Abort Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |ABTIFn    |LPPDMA Read/Write Target Abort Interrupt Status Flag
     * |        |          |This bit indicates which LPPDMA controller has target abort error.
     * |        |          |0 = No AHB bus ERROR response received when channel n transfer.
     * |        |          |1 = AHB bus ERROR response received when channel n transfer.
     * |        |          |Note 1: If channel n target abort, REQSRCn should set0 to disable peripheral request.
     * |        |          |Note 2: User can write 1 to clear this bit.
     * @var LPPDMA_T::TDSTS
     * Offset: 0x424  LPPDMA Channel Transfer Done Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |TDIFn     |Transfer Done Flag
     * |        |          |This bit indicates whether LPPDMA controller channel transfer has been finished or not.
     * |        |          |0 = LPPDMA channel transfer has not finished.
     * |        |          |1 = LPPDMA channel has finished transmission.
     * |        |          |Note: User can write 1 to clear these bits.
     * @var LPPDMA_T::ALIGN
     * Offset: 0x428  LPPDMA Transfer Alignment Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |ALIGNn    |Transfer Alignment Flag
     * |        |          |This bit indicates whether source and destination address both follow transfer width setting.
     * |        |          |0 = LPPDMA channel source address and destination address both follow transfer width setting.
     * |        |          |1 = LPPDMA channel source address or destination address is not follow transfer width setting.
     * |        |          |Note: User can write 1 to clear these bits.
     * @var LPPDMA_T::TACTSTS
     * Offset: 0x42C  LPPDMA Transfer Active Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |TXACTFn   |Transfer on Active Flag (Read Only)
     * |        |          |This bit indicates which LPPDMA channel is in active.
     * |        |          |0 = LPPDMA channel is finished.
     * |        |          |1 = LPPDMA channel is active.
     * @var LPPDMA_T::CHRST
     * Offset: 0x460  LPPDMA Channel Reset Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |CHnRST    |Channel n Reset
     * |        |          |0 = Corresponding channel n is not reset.
     * |        |          |1 = Corresponding channel n is reset.
     * @var LPPDMA_T::REQSEL0_3
     * Offset: 0x480  LPPDMA Request Source Select Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[6:0]   |REQSRC0   |Channel 0 Request Source Selection
     * |        |          |This filed defines which peripheral is connected to LPPDMA channel 0
     * |        |          |User can configure the peripheral by setting REQSRC0.
     * |        |          |0 = Disable LPPDMA peripheral request.
     * |        |          |1 = Channel connects to LPUART0_TX.
     * |        |          |2 = Channel connects to LPUART0_RX.
     * |        |          |3 = Channel connects to LPSPI0_TX.
     * |        |          |4 = Channel connects to LPSPI0_RX.
     * |        |          |5 = Channel connects to LPI2C0_TX.
     * |        |          |6 = Channel connects to LPI2C0_RX.
     * |        |          |7 = Channel connects to LPTMR0.
     * |        |          |8 = Channel connects to LPTMR1.
     * |        |          |9 = Channel connects to TTMR0.
     * |        |          |10= Channel connects to TTMR1.
     * |        |          |11 = Channel connects to LPADC0_RX.
     * |        |          |12 = Reserved.
     * |        |          |13 = Channel connects to DMIC0_RX.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: A peripheral cannot be assigned to two channels at the same time.
     * |        |          |Note 2: This field is useless when transfer between memory and memory.
     * |[14:8]  |REQSRC1   |Channel 1 Request Source Selection
     * |        |          |This filed defines which peripheral is connected to LPPDMA channel 1
     * |        |          |User can configure the peripheral setting by REQSRC1.
     * |        |          |Note: The channel configuration is the same as REQSRC0 field
     * |        |          |Please refer to the explanation of REQSRC0.
     * |[22:16] |REQSRC2   |Channel 2 Request Source Selection
     * |        |          |This filed defines which peripheral is connected to LPPDMA channel 2
     * |        |          |User can configure the peripheral setting by REQSRC2.
     * |        |          |Note: The channel configuration is the same as REQSRC0 field
     * |        |          |Please refer to the explanation of REQSRC0.
     * |[30:24] |REQSRC3   |Channel 3 Request Source Selection
     * |        |          |This filed defines which peripheral is connected to LPPDMA channel 3
     * |        |          |User can configure the peripheral setting by REQSRC3.
     * |        |          |Note: The channel configuration is the same as REQSRC0 field
     * |        |          |Please refer to the explanation of REQSRC0.
     */
    LPDSCT_T LPDSCT[4];
    __I  uint32_t CURSCAT[4];              /*!< [0x0040] Current Scatter-gather Descriptor Table Address of LPPDMA Channel n */
    __I  uint32_t RESERVE1[236];
    __IO uint32_t CHCTL;                 /*!< [0x0400] LPPDMA Channel Control Register                                  */
    __O  uint32_t PAUSE;                 /*!< [0x0404] LPPDMA Transfer Pause Control Register                           */
    __O  uint32_t SWREQ;                 /*!< [0x0408] LPPDMA Software Request Register                                 */
    __I  uint32_t TRGSTS;                /*!< [0x040c] LPPDMA Channel Request Status Register                           */
    __IO uint32_t PRISET;                /*!< [0x0410] LPPDMA Fixed Priority Setting Register                           */
    __O  uint32_t PRICLR;                /*!< [0x0414] LPPDMA Fixed Priority Clear Register                             */
    __IO uint32_t INTEN;                 /*!< [0x0418] LPPDMA Interrupt Enable Register                                 */
    __IO uint32_t INTSTS;                /*!< [0x041c] LPPDMA Interrupt Status Register                                 */
    __IO uint32_t ABTSTS;                /*!< [0x0420] LPPDMA Channel Read/Write Target Abort Flag Register             */
    __IO uint32_t TDSTS;                 /*!< [0x0424] LPPDMA Channel Transfer Done Flag Register                       */
    __IO uint32_t ALIGN;                 /*!< [0x0428] LPPDMA Transfer Alignment Status Register                        */
    __I  uint32_t TACTSTS;               /*!< [0x042c] LPPDMA Transfer Active Flag Register                             */
    __I  uint32_t RESERVE2[12];
    __IO uint32_t CHRST;                 /*!< [0x0460] LPPDMA Channel Reset Register                                    */
    __I  uint32_t RESERVE4[7];
    __IO uint32_t REQSEL0_3;             /*!< [0x0480] LPPDMA Request Source Select Register 0                          */

} LPPDMA_T;

/**
    @addtogroup LPPDMA_CONST LPPDMA Bit Field Definition
    Constant Definitions for LPPDMA Controller
@{ */

#define LPPDMA_DSCT_CTL_OPMODE_Pos      (0)                                                /*!< LPDSCT_T::CTL: OPMODE Position   */
#define LPPDMA_DSCT_CTL_OPMODE_Msk      (0x3ul << LPPDMA_DSCT_CTL_OPMODE_Pos)              /*!< LPDSCT_T::CTL: OPMODE Mask       */

#define LPPDMA_DSCT_CTL_TXTYPE_Pos      (2)                                                /*!< LPDSCT_T::CTL: TXTYPE Position   */
#define LPPDMA_DSCT_CTL_TXTYPE_Msk      (0x1ul << LPPDMA_DSCT_CTL_TXTYPE_Pos)              /*!< LPDSCT_T::CTL: TXTYPE Mask       */

#define LPPDMA_DSCT_CTL_BURSIZE_Pos     (4)                                                /*!< LPDSCT_T::CTL: BURSIZE Position  */
#define LPPDMA_DSCT_CTL_BURSIZE_Msk     (0x7ul << LPPDMA_DSCT_CTL_BURSIZE_Pos)             /*!< LPDSCT_T::CTL: BURSIZE Mask      */

#define LPPDMA_DSCT_CTL_TBINTDIS_Pos    (7)                                                /*!< LPDSCT_T::CTL: TBINTDIS Position */
#define LPPDMA_DSCT_CTL_TBINTDIS_Msk    (0x1ul << LPPDMA_DSCT_CTL_TBINTDIS_Pos)            /*!< LPDSCT_T::CTL: TBINTDIS Mask     */

#define LPPDMA_DSCT_CTL_SAINC_Pos       (8)                                                /*!< LPDSCT_T::CTL: SAINC Position    */
#define LPPDMA_DSCT_CTL_SAINC_Msk       (0x3ul << LPPDMA_DSCT_CTL_SAINC_Pos)               /*!< LPDSCT_T::CTL: SAINC Mask        */

#define LPPDMA_DSCT_CTL_DAINC_Pos       (10)                                               /*!< LPDSCT_T::CTL: DAINC Position    */
#define LPPDMA_DSCT_CTL_DAINC_Msk       (0x3ul << LPPDMA_DSCT_CTL_DAINC_Pos)               /*!< LPDSCT_T::CTL: DAINC Mask        */

#define LPPDMA_DSCT_CTL_TXWIDTH_Pos     (12)                                               /*!< LPDSCT_T::CTL: TXWIDTH Position  */
#define LPPDMA_DSCT_CTL_TXWIDTH_Msk     (0x3ul << LPPDMA_DSCT_CTL_TXWIDTH_Pos)             /*!< LPDSCT_T::CTL: TXWIDTH Mask      */

#define LPPDMA_DSCT_CTL_TXCNT_Pos       (16)                                               /*!< LPDSCT_T::CTL: TXCNT Position    */
#define LPPDMA_DSCT_CTL_TXCNT_Msk       (0xfffful << LPPDMA_DSCT_CTL_TXCNT_Pos)            /*!< LPDSCT_T::CTL: TXCNT Mask        */

#define LPPDMA_DSCT_SA_SA_Pos           (0)                                                /*!< LPDSCT_T::SA: SA Position        */
#define LPPDMA_DSCT_SA_SA_Msk           (0xfffffffful << LPPDMA_DSCT_SA_SA_Pos)            /*!< LPDSCT_T::SA: SA Mask            */

#define LPPDMA_DSCT_DA_DA_Pos           (0)                                                /*!< LPDSCT_T::DA: DA Position        */
#define LPPDMA_DSCT_DA_DA_Msk           (0xfffffffful << LPPDMA_DSCT_DA_DA_Pos)            /*!< LPDSCT_T::DA: DA Mask            */

#define LPPDMA_DSCT_NEXT_NEXT_Pos       (0)                                                /*!< LPDSCT_T::NEXT: NEXT Position    */
#define LPPDMA_DSCT_NEXT_NEXT_Msk       (0xfffful << LPPDMA_DSCT_NEXT_NEXT_Pos)            /*!< LPDSCT_T::NEXT: NEXT Mask        */

#define LPPDMA_DSCT_NEXT_EXENEXT_Pos    (16)                                               /*!< LPDSCT_T::NEXT: EXENEXT Position */
#define LPPDMA_DSCT_NEXT_EXENEXT_Msk    (0xfffful << LPPDMA_DSCT_NEXT_EXENEXT_Pos)         /*!< LPDSCT_T::NEXT: EXENEXT Mask     */

#define LPPDMA_CURSCAT_CURADDR_Pos      (0)                                                /*!< LPPDMA_T::CURSCAT: CURADDR Position   */
#define LPPDMA_CURSCAT_CURADDR_Msk      (0xfffffffful << LPPDMA_CURSCAT_CURADDR_Pos)       /*!< LPPDMA_T::CURSCAT: CURADDR Mask       */

#define LPPDMA_CHCTL_CHENn_Pos           (0)                                               /*!< LPPDMA_T::CHCTL: CHENn Position        */
#define LPPDMA_CHCTL_CHENn_Msk           (0xful << LPPDMA_CHCTL_CHENn_Pos)                 /*!< LPPDMA_T::CHCTL: CHENn Mask            */

#define LPPDMA_PAUSE_PAUSEn_Pos          (0)                                               /*!< LPPDMA_T::PAUSE: PAUSEn Position       */
#define LPPDMA_PAUSE_PAUSEn_Msk          (0xful << LPPDMA_PAUSE_PAUSEn_Pos)                /*!< LPPDMA_T::PAUSE: PAUSEn Mask           */

#define LPPDMA_SWREQ_SWREQn_Pos          (0)                                               /*!< LPPDMA_T::SWREQ: SWREQn Position       */
#define LPPDMA_SWREQ_SWREQn_Msk          (0xful << LPPDMA_SWREQ_SWREQn_Pos)                /*!< LPPDMA_T::SWREQ: SWREQn Mask           */

#define LPPDMA_TRGSTS_REQSTSn_Pos        (0)                                               /*!< LPPDMA_T::TRGSTS: REQSTSn Position     */
#define LPPDMA_TRGSTS_REQSTSn_Msk        (0xful << LPPDMA_TRGSTS_REQSTSn_Pos)              /*!< LPPDMA_T::TRGSTS: REQSTSn Mask         */

#define LPPDMA_PRISET_FPRISETn_Pos       (0)                                               /*!< LPPDMA_T::PRISET: FPRISETn Position    */
#define LPPDMA_PRISET_FPRISETn_Msk       (0xful << LPPDMA_PRISET_FPRISETn_Pos)             /*!< LPPDMA_T::PRISET: FPRISETn Mask        */

#define LPPDMA_PRICLR_FPRICLRn_Pos       (0)                                               /*!< LPPDMA_T::PRICLR: FPRICLRn Position    */
#define LPPDMA_PRICLR_FPRICLRn_Msk       (0xful << LPPDMA_PRICLR_FPRICLRn_Pos)             /*!< LPPDMA_T::PRICLR: FPRICLRn Mask        */

#define LPPDMA_INTEN_INTENn_Pos          (0)                                               /*!< LPPDMA_T::INTEN: INTENn Position       */
#define LPPDMA_INTEN_INTENn_Msk          (0xful << LPPDMA_INTEN_INTENn_Pos)                /*!< LPPDMA_T::INTEN: INTENn Mask           */

#define LPPDMA_INTSTS_ABTIF_Pos          (0)                                               /*!< LPPDMA_T::INTSTS: ABTIF Position       */
#define LPPDMA_INTSTS_ABTIF_Msk          (0x1ul << LPPDMA_INTSTS_ABTIF_Pos)                /*!< LPPDMA_T::INTSTS: ABTIF Mask           */

#define LPPDMA_INTSTS_TDIF_Pos           (1)                                               /*!< LPPDMA_T::INTSTS: TDIF Position        */
#define LPPDMA_INTSTS_TDIF_Msk           (0x1ul << LPPDMA_INTSTS_TDIF_Pos)                 /*!< LPPDMA_T::INTSTS: TDIF Mask            */

#define LPPDMA_INTSTS_ALIGNF_Pos         (2)                                               /*!< LPPDMA_T::INTSTS: ALIGNF Position      */
#define LPPDMA_INTSTS_ALIGNF_Msk         (0x1ul << LPPDMA_INTSTS_ALIGNF_Pos)               /*!< LPPDMA_T::INTSTS: ALIGNF Mask          */

#define LPPDMA_INTSTS_WKF_Pos            (3)                                               /*!< LPPDMA_T::INTSTS: WKF Position         */
#define LPPDMA_INTSTS_WKF_Msk            (0x1ul << LPPDMA_INTSTS_WKF_Pos)                  /*!< LPPDMA_T::INTSTS: WKF Mask             */

#define LPPDMA_ABTSTS_ABTIF0_Pos         (0)                                               /*!< LPPDMA_T::ABTSTS: ABTIF0 Position      */
#define LPPDMA_ABTSTS_ABTIF0_Msk         (0x1ul << LPPDMA_ABTSTS_ABTIF0_Pos)               /*!< LPPDMA_T::ABTSTS: ABTIF0 Mask          */

#define LPPDMA_ABTSTS_ABTIF1_Pos         (1)                                               /*!< LPPDMA_T::ABTSTS: ABTIF1 Position      */
#define LPPDMA_ABTSTS_ABTIF1_Msk         (0x1ul << LPPDMA_ABTSTS_ABTIF1_Pos)               /*!< LPPDMA_T::ABTSTS: ABTIF1 Mask          */

#define LPPDMA_ABTSTS_ABTIF2_Pos         (2)                                               /*!< LPPDMA_T::ABTSTS: ABTIF2 Position      */
#define LPPDMA_ABTSTS_ABTIF2_Msk         (0x1ul << LPPDMA_ABTSTS_ABTIF2_Pos)               /*!< LPPDMA_T::ABTSTS: ABTIF2 Mask          */

#define LPPDMA_ABTSTS_ABTIF3_Pos         (3)                                               /*!< LPPDMA_T::ABTSTS: ABTIF3 Position      */
#define LPPDMA_ABTSTS_ABTIF3_Msk         (0x1ul << LPPDMA_ABTSTS_ABTIF3_Pos)               /*!< LPPDMA_T::ABTSTS: ABTIF3 Mask          */

#define LPPDMA_TDSTS_TDIF0_Pos           (0)                                               /*!< LPPDMA_T::TDSTS: TDIF0 Position        */
#define LPPDMA_TDSTS_TDIF0_Msk           (0x1ul << LPPDMA_TDSTS_TDIF0_Pos)                 /*!< LPPDMA_T::TDSTS: TDIF0 Mask            */

#define LPPDMA_TDSTS_TDIF1_Pos           (1)                                               /*!< LPPDMA_T::TDSTS: TDIF1 Position        */
#define LPPDMA_TDSTS_TDIF1_Msk           (0x1ul << LPPDMA_TDSTS_TDIF1_Pos)                 /*!< LPPDMA_T::TDSTS: TDIF1 Mask            */

#define LPPDMA_TDSTS_TDIF2_Pos           (2)                                               /*!< LPPDMA_T::TDSTS: TDIF2 Position        */
#define LPPDMA_TDSTS_TDIF2_Msk           (0x1ul << LPPDMA_TDSTS_TDIF2_Pos)                 /*!< LPPDMA_T::TDSTS: TDIF2 Mask            */

#define LPPDMA_TDSTS_TDIF3_Pos           (3)                                               /*!< LPPDMA_T::TDSTS: TDIF3 Position        */
#define LPPDMA_TDSTS_TDIF3_Msk           (0x1ul << LPPDMA_TDSTS_TDIF3_Pos)                 /*!< LPPDMA_T::TDSTS: TDIF3 Mask            */

#define LPPDMA_ALIGN_ALIGNn_Pos          (0)                                               /*!< LPPDMA_T::ALIGN: ALIGNn Position       */
#define LPPDMA_ALIGN_ALIGNn_Msk          (0xful << LPPDMA_ALIGN_ALIGNn_Pos)                /*!< LPPDMA_T::ALIGN: ALIGNn Mask           */

#define LPPDMA_TACTSTS_TXACTFn_Pos       (0)                                               /*!< LPPDMA_T::TACTSTS: TXACTFn Position    */
#define LPPDMA_TACTSTS_TXACTFn_Msk       (0xful << LPPDMA_TACTSTS_TXACTFn_Pos)             /*!< LPPDMA_T::TACTSTS: TXACTFn Mask        */

#define LPPDMA_CHRST_CHnRST_Pos          (0)                                               /*!< LPPDMA_T::CHRST: CHnRST Position       */
#define LPPDMA_CHRST_CHnRST_Msk          (0xful << LPPDMA_CHRST_CHnRST_Pos)                /*!< LPPDMA_T::CHRST: CHnRST Mask           */

#define LPPDMA_REQSEL0_3_REQSRC0_Pos     (0)                                               /*!< LPPDMA_T::REQSEL0_3: REQSRC0 Position  */
#define LPPDMA_REQSEL0_3_REQSRC0_Msk     (0x7ful << LPPDMA_REQSEL0_3_REQSRC0_Pos)          /*!< LPPDMA_T::REQSEL0_3: REQSRC0 Mask      */

#define LPPDMA_REQSEL0_3_REQSRC1_Pos     (8)                                               /*!< LPPDMA_T::REQSEL0_3: REQSRC1 Position  */
#define LPPDMA_REQSEL0_3_REQSRC1_Msk     (0x7ful << LPPDMA_REQSEL0_3_REQSRC1_Pos)          /*!< LPPDMA_T::REQSEL0_3: REQSRC1 Mask      */

#define LPPDMA_REQSEL0_3_REQSRC2_Pos     (16)                                              /*!< LPPDMA_T::REQSEL0_3: REQSRC2 Position  */
#define LPPDMA_REQSEL0_3_REQSRC2_Msk     (0x7ful << LPPDMA_REQSEL0_3_REQSRC2_Pos)          /*!< LPPDMA_T::REQSEL0_3: REQSRC2 Mask      */

#define LPPDMA_REQSEL0_3_REQSRC3_Pos     (24)                                              /*!< LPPDMA_T::REQSEL0_3: REQSRC3 Position  */
#define LPPDMA_REQSEL0_3_REQSRC3_Msk     (0x7ful << LPPDMA_REQSEL0_3_REQSRC3_Pos)          /*!< LPPDMA_T::REQSEL0_3: REQSRC3 Mask      */

/** @} LPPDMA_CONST */
/** @} end of LPPDMA register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __LPPDMA_REG_H__ */
