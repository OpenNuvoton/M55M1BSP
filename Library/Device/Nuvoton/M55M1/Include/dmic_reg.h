/**************************************************************************//**
 * @file     dmic_reg.h
 * @version  V1.00
 * @brief    DMIC register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __DMIC_REG_H__
#define __DMIC_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup DMIC Digital Microphone Inputs (DMIC)
    Memory Mapped Structure for DMIC Controller
    @{
*/

typedef struct
{

    /**
     * @var DMIC_T::CTL
     * Offset: 0x00  DMIC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHEN0     |DMIC Channel 0 Enable
     * |        |          |Set this bit to 1 to enable DMIC channel 0 operation.
     * |        |          |0 = DMIC Channel 0 Disabled.
     * |        |          |1 = DMIC Channel 0 Enabled.
     * |[1]     |CHEN1     |DMIC Channel 1 Enable
     * |        |          |Set this bit to 1 to enable DMIC channel 1 operation.
     * |        |          |0 = DMIC Channel 1 Disabled.
     * |        |          |1 = DMIC Channel 1 Enabled.
     * |[2]     |CHEN2     |DMIC Channel 2 Enable
     * |        |          |Set this bit to 1 to enable DMIC channel 2 operation.
     * |        |          |0 = DMIC Channel 2 Disabled.
     * |        |          |1 = DMIC Channel 2 Enabled.
     * |[3]     |CHEN3     |DMIC Channel 3 Enable
     * |        |          |Set this bit to 1 to enable DMIC channel 3 operation.
     * |        |          |0 = DMIC Channel 3 Disabled.
     * |        |          |1 = DMIC Channel 3 Enabled.
     * |[8]     |LCHEDGE01 |DMIC Channel 01 Data Latch Edge
     * |        |          |The data of DMIC channel 0 and channel 1 is latched on DMIC_DATA0 pin
     * |        |          |This bit is used to select the data of DMIC channel 0 and channel 1 is latched on rising or falling edge of DMIC_CLK (DMIC bus clock).
     * |        |          |0 = The data of channel 0 is latched on falling edge of DMIC_CLK
     * |        |          |The data of channel 1 is latched on rising edge of DMIC_CLK.
     * |        |          |1 = The data of channel 0 is latched on rising edge of DMIC_CLK
     * |        |          |The data of channel 1 is latched on falling edge of DMIC_CLK.
     * |[9]     |LCHEDGE23 |DMIC Channel 23 Data Latch Edge
     * |        |          |The data of DMIC channel 2 and channel 3 is latched on DMIC_DATA0 pin
     * |        |          |This bit is used to select the data of DMIC channel 2 and channel 3 is latched on rising or falling edge of DMIC_CLK (DMIC bus clock).
     * |        |          |0 = The data of channel 2 is latched on falling edge of DMIC_CLK
     * |        |          |The data of channel 3 is latched on rising edge of DMIC_CLK.
     * |        |          |1 = The data of channel 2 is latched on rising edge of DMIC_CLK
     * |        |          |The data of channel 3 is latched on falling edge of DMIC_CLK.
     * |[12]    |CH0MUTE   |DMIC Channel 0 Mute Enable
     * |        |          |Set this bit to 1 to mute DMIC channel 0.
     * |        |          |0 = DMIC Channel 0 Unmute.
     * |        |          |1 = DMIC Channel 0 Mute.
     * |[13]    |CH1MUTE   |DMIC Channel 1 Mute Enable
     * |        |          |Set this bit to 1 to mute DMIC channel 1.
     * |        |          |0 = DMIC Channel 1 Unmute.
     * |        |          |1 = DMIC Channel 1 Mute.
     * |[14]    |CH2MUTE   |DMIC Channel 2 Mute Enable
     * |        |          |Set this bit to 1 to mute DMIC channel 2.
     * |        |          |0 = DMIC Channel 2 Unmute.
     * |        |          |1 = DMIC Channel 2 Mute.
     * |[15]    |CH3MUTE   |DMIC Channel 3 Mute Enable
     * |        |          |Set this bit to 1 to mute DMIC channel 3.
     * |        |          |0 = DMIC Channel 3 Unmute.
     * |        |          |1 = DMIC Channel 3 Mute.
     * |[20]    |CH01HPFEN |DMIC Channel 01 HPF filter Enable.
     * |        |          |Set this bit to 1 to Enable DMIC channel 0 and channel 1 HPF filter for remove DC component.
     * |        |          |0 = DMIC Channel 01 HPF Disabled.
     * |        |          |1 = DMIC Channel 01 HPF Enabled.
     * |[21]    |CH23HPFEN |DMIC Channel 23 HPF filter Enable.
     * |        |          |Set this bit to 1 to Enable DMIC channel 2 and channel 3 HPF filter for remove DC component.
     * |        |          |0 = DMIC Channel 23 HPF Disabled.
     * |        |          |1 = DMIC Channel 23 HPF Enabled.
     * |[24]    |SWRST     |Internal State Software Reset.
     * |        |          |Set this bit to 1 to Clear DMIC internal state, but all DMIC registers are kept unchanged.
     * |        |          |0 = Normal operation
     * |        |          |1 = Internal DMIC State Reset
     * |[28:27] |GAINSTEP  |Volume Control Gain Adjust Step for Decimal Point.
     * |        |          |00 = 0.5dB (1/2)
     * |        |          |01 = 0.25dB (1/4)
     * |        |          |10 = 0.125dB (1/8)
     * |        |          |11 = 0.0625dB (1/16)
     * |[31]    |FIFOWIDTH |Data Effective Bit in FIFO
     * |        |          |0 = 16-bit, Data Truncated LSB bit0~bit7. [15:0]
     * |        |          |1 = 24-bit
     * @var DMIC_T::DIV
     * Offset: 0x04  DMIC Clock Divider Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[13:8]  |DMCLKDIV  |Divider to generate the DMIC Bus Clock
     * |        |          |The value in this field is the frequency divider for generating the DMIC bus clock
     * |        |          |The frequency is obtained according to the following equation.
     * |        |          |F_DMIC_CLK = (F_DMIC_MCLK)/(1 + DMCLKDIV)
     * |        |          |where F_DMIC_MCLK is the frequency of DMIC working main clock (DMIC_MCLK) and F_DMIC_CLK is the frequency of DMIC bus clock (DMIC_CLK).
     * |[20:16] |DMTH      |FIFO Threshold Level
     * |        |          |If the valid data count of the FIFO data buffer is more than or equal to DMTH (DMIC_DIV[20:16]) setting, the DMTHIF (DMIC_STATUS[2]) bit will set to 1, else the DMTHIF (DMIC_STATUS[2]) bit will be cleared to 0.
     * |[21]    |DMTHIE    |FIFO Threshold Interrupt
     * |        |          |0 = FIFO threshold interrupt Disabled
     * |        |          |1 = FIFO threshold interrupt Enabled.
     * |[23:22] |FCLR      |FIFO Clear
     * |        |          |11 = Clear the FIFO.
     * |        |          |Others = Reserved. Do not use.
     * |        |          |Note 1: To clear the FIFO, need to write FCLR (DMIC_DIV[23:22]) to 11b, and can read the EMPTY (DMIC_STATUS[1]) bit to make sure that the FIFO has been cleared.
     * |        |          |Note 2: This field is auto cleared by hardware.
     * |[27:24] |OSR       |DMIC OSR Setting
     * |        |          |000 = Down sample 64
     * |        |          |001 = Down sample 128
     * |        |          |010 = Down sample 256
     * |        |          |011 = Down sample 100
     * |        |          |111 = Down sample 50
     * @var DMIC_T::STATUS
     * Offset: 0x08  DMIC Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FULL      |FIFO Full Indicator (Read Only)
     * |        |          |0 = FIFO is not full.
     * |        |          |1 = FIFO is full.
     * |[1]     |EMPTY     |FIFO Empty Indicator (Read Only)
     * |        |          |0 = FIFO is not empty.
     * |        |          |1 = FIFO is empty.
     * |[2]     |DMTHIF    |FIFO Threshold Interrupt Status (Read Only)
     * |        |          |0 = The valid data count within the FIFO data buffer is less than the setting value of DMTH (DMIC_DIV[20:16]).
     * |        |          |1 = The valid data count within the FIFO data buffer is more than or equal to the setting value of TH (DMIC_DIV[20:16])
     * |[8:4]   |FIFOPTR   |FIFO Pointer (Read Only)
     * |        |          |The FULL (DMIC_STATUS[0]) and FIFOPTR (DMIC_STATUS[8:4]) indicates the field that the valid data count within the DMIC FIFO buffer.
     * |        |          |The maximum value shown in FIFOPTR (DMIC_STATUS[8:4]) is 31
     * |        |          |When the using level of DMIC FIFO buffer equal to 32, The FULL (DMIC_STATUS[0]) is set to 1.
     * @var DMIC_T::LPPDMACTL
     * Offset: 0x0C  DMIC LPPDMA Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPPDMAEN  |LPPDMA Transfer Enable Bit
     * |        |          |0 = LPPDMA data transfer Disabled.
     * |        |          |1 = LPPDMA data transfer Enabled.
     * @var DMIC_T::FIFO
     * Offset: 0x10  DMIC FIFO Data Output Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |FIFO      |FIFO Data Output Register
     * |        |          |DMIC contains 32 level (32x24 bit) data buffer for data receive
     * |        |          |A read to this register pushes data out from FIFO data buffer and decrements the read pointer
     * |        |          |This is the address that PDMA reads audio data from
     * |        |          |The remaining data word number is indicated by FIFOPTR (DMIC_STATUS[8:4]).
     * @var DMIC_T::GAINCTL0
     * Offset: 0x14  DMIC Channel 0 and 1 Volume Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CHyyVOL   |Channel even volume/gain control, 0.125dB step
     * |        |          |A 16-bit signed 2u2019s complement number, format is as following
     * |        |          |[15]Sign
     * |        |          |[14:7]Integer Part
     * |        |          |[6]1/2 dB
     * |        |          |[5]1/4 dB
     * |        |          |[4]1/8 dB
     * |        |          |[3]1/16dB
     * |        |          |[2:0]Reserved
     * |        |          |The mapping between real gain and programmed number is
     * |        |          |CHxVOL= -128-(Real_Gain) (dB)
     * |        |          |For example,
     * |        |          |if the desired gain is 0dB, the program value will be -128 (0xC000)
     * |        |          |if the desired gain is -20.5dB, then the programmed value will be -107.5(0xCA40)
     * |[31:16] |CHxxVOL   |Channel odd volume/gain control, 0.125dB step
     * |        |          |A 16-bit signed 2u2019s complement number, format is as following
     * |        |          |[31]Sign
     * |        |          |[30:23]Integer Part
     * |        |          |[22]1/2 dB
     * |        |          |[21]1/4 dB
     * |        |          |[20]1/8 dB
     * |        |          |[19]1/16dB
     * |        |          |[18:16]Reserved
     * |        |          |The mapping between real gain and programmed number is
     * |        |          |CHxVOL= -128-(Real_Gain) (dB)
     * |        |          |For example,
     * |        |          |if the desired gain is 0dB, the program value will be -128 (0xC000)
     * |        |          |if the desired gain is -20.5dB, then the programmed value will be -107.5(0xCA40)
     * @var DMIC_T::GAINCTL1
     * Offset: 0x18  DMIC Channel 2 and 3 Volume Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CHyyVOL   |Channel yy volume/gain control, 0.125dB step
     * |        |          |A 16-bit signed 2u2019s complement number, format is as following
     * |        |          |[15]Sign
     * |        |          |[14:7]Integer Part
     * |        |          |[6]1/2 dB
     * |        |          |[5]1/4 dB
     * |        |          |[4]1/8 dB
     * |        |          |[3]1/16dB
     * |        |          |[2:0]Reserved
     * |        |          |The mapping between real gain and programmed number is
     * |        |          |CHxVOL= -128-(Real_Gain) (dB)
     * |        |          |For example,
     * |        |          |if the desired gain is 0dB, the program value will be -128 (0xC000)
     * |        |          |if the desired gain is -20.5dB, then the programmed value will be -107.5(0xCA40)
     * |[31:16] |CHxxVOL   |Channel xx volume/gain control, 0.125dB step
     * |        |          |A 16-bit signed 2u2019s complement number, format is as following
     * |        |          |[31]Sign
     * |        |          |[30:23]Integer Part
     * |        |          |[22]1/2 dB
     * |        |          |[21]1/4 dB
     * |        |          |[20]1/8 dB
     * |        |          |[19]1/16dB
     * |        |          |[18:16]Reserved
     * |        |          |The mapping between real gain and programmed number is
     * |        |          |CHxVOL= -128-(Real_Gain) (dB)
     * |        |          |For example,
     * |        |          |if the desired gain is 0dB, the program value will be -128 (0xC000)
     * |        |          |if the desired gain is -20.5dB, then the programmed value will be -107.5(0xCA40)
     */
    __IO uint32_t CTL;                   /*!< [0x0000] DMIC Control Register                                            */
    __IO uint32_t DIV;                   /*!< [0x0004] DMIC Clock Divider Register                                      */
    __I  uint32_t STATUS;                /*!< [0x0008] DMIC Status Register                                             */
    __IO uint32_t LPPDMACTL;             /*!< [0x000c] DMIC LPPDMA Control Register                                       */
    __I  uint32_t FIFO;                  /*!< [0x0010] DMIC FIFO Data Output Register                                  */
    __IO uint32_t GAINCTL0;              /*!< [0x0014] DMIC Channel 0 and 1 Volume Control Register                     */
    __IO uint32_t GAINCTL1;              /*!< [0x0018] DMIC Channel 2 and 3 Volume Control Register                     */
    __I  uint32_t RESERVE0[6];
} DMIC_T;


typedef struct
{

    /**
     * @var VAD_T::SINCCTL
     * Offset: 0x00  VAD SINC Filter Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:8]  |SINCOSR   |VAD SINC Filter OSR Setting
     * |        |          |000 = Down sample 48
     * |        |          |001 = Down sample 64
     * |        |          |010 = Down sample 96
     * |        |          |Others = Reserved. Do not use.
     * |[23:16] |VADMCLKDIV|Divider to generate the VAD Working Main Clock
     * |        |          |The value in this field is the frequency divider for generating the DMIC working main clock. The frequency is obtained according to the following equation.
     * |        |          |F_DMIC_MCLK = (F_VAD_CLK_SRC)/(1 + VADMCLKDIV).
     * |        |          |where F_VAD_CLK_SRC is the frequency of DMIC module clock source, which is defined in the clock control register DMICSEL VAD0SEL (CLK_DMICSEL [5:4]) and   F_DMIC_MCLK depends on the cycle of DMIC DSP processor needed.
     * |[30]    |ACTCL     |VAD Active Flag Clear
     * |        |          |0 = No effect.
     * |        |          |1 = Clear ACTIVE(VAD_STATUS0[31]).
     * |        |          |Note: After ACTIVE(VAD_STATUS0[31]) is cleared, user need to set set this bit to 0.
     * |[31]    |VADEN     |VAD Enable Control
     * |        |          |0 = VAD Disabled.
     * |        |          |1 = VAD Enabled.
     * |        |          |Note 1: When set this bit to 1, CHEN0 (DMIC_CTL[0]) will be set to 1 and CHEN1 (DMIC_CTL[1]), CHEN2 (DMIC_CTL[2]) and CHEN3 (DMIC_CTL[3]) will be set to 0 automatically.
     * |        |          |Note 2: When set this bit to 1, DMIC_CLK is generated by VAD module.
     * @var VAD_T::BIQCTL0
     * Offset: 0x04  VAD Biquad Filter Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |BIQA1     |VAD Biquad Filter Coefficient
     * |        |          |Biquad Filter Coefficient a1, in 3 intergers + 13 fractional bits
     * |[31:16] |BIQA2     |VAD Biquad Filter Coefficient
     * |        |          |Biquad Filter Coefficient a2, in 3 intergers + 13 fractional bits.
     * @var VAD_T::BIQCTL1
     * Offset: 0x08  VAD Biquad Filter Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |BIQB0     |VAD Biquad Filter Coefficient
     * |        |          |Biquad Filter Coefficient b0, in 3 intergers + 13 fractional bits.
     * |[31:16] |BIQB1     |VAD Biquad Filter Coefficient
     * |        |          |Biquad Filter Coefficient b1, in 3 intergers + 13 fractional bits.
     * @var VAD_T::BIQCTL2
     * Offset: 0x0C  VAD Biquad Filter Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |BIQB2     |VAD Biquad Filter Coefficient
     * |        |          |Biquad Filter Coefficient b2, in 3 intergers + 13 fractional bits.
     * |[31]    |BIQEN     |VAD Biquad Filter Enable Bit
     * |        |          |0 = VAD Biquad Filter Disabled.
     * |        |          |1 = VAD Biquad Filter Enabled.
     * @var VAD_T::CTL0
     * Offset: 0x10  VAD Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |STAT      |Short Term Power Attack Time
     * |        |          |Slow attack (e.g., 0x99): slow responding to voice, but more stable.
     * |        |          |Fast attack (e.g., 0xCC): fast responding to voice, but more sensitive to other sounds.
     * |        |          |Suggested default attack time setting: Long term power attack time (0x5), Short term power attack time (0xAA).
     * |        |          |The u201CShort Term Poweru201D, in order to detect the instant power of the voices, requires faster attack time, while u201CLong Term Poweru201D, in order to get the averaged power of the background environment, requires slower attack time to maintain its stability
     * |        |          |So the Short term power attack time should be always bigger than the Long term power attack time.
     * |[19:16] |LTAT      |Long Term Power Attack Time
     * |        |          |Slow attack (e.g., 0x5): less sensitive to environment change.
     * |        |          |Fast attack (e.g., 0x8): more sensitive to environment change.
     * @var VAD_T::CTL1
     * Offset: 0x14  VAD Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |STTHREHWM |Short Term Power Threshold Upper Limit
     * |        |          |To check if the incoming signal is big enough to be ready for VAD activation.
     * @var VAD_T::CTL2
     * Offset: 0x18  VAD Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:16] |LTTHRE    |Long Term Power Threshold
     * |        |          |To check the background energy, also serve as the lower limit of long term power
     * |        |          |When the long term power value is lower than the threshold, it will be set to the threshold value for VAD decision.
     * @var VAD_T::CTL3
     * Offset: 0x1C  VAD Control Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |DEVTHRE   |Deviation Threshold
     * |        |          |To check if the incoming signal is substantially bigger than its background
     * |        |          |This may work to exclude breath sound as it is slowly varying, but not other sounds (e.g., footsteps, hand claps) with sudden amplitude increase.
     * |        |          |Small: easy to trigger, good for far-field pick-up, but requiring quiet environment.
     * |        |          |Large: good for handheld applications, but requiring louder voice to trigger.
     * @var VAD_T::STATUS0
     * Offset: 0x20  VAD Status Read-back Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |STP       |Short Term Signal Power (Read Only)
     * |        |          |This field shows the short term signal power value.
     * |[31]    |ACTIVE    |VAD Activation Flag (Read Only)
     * |        |          |When the voice active event occurs, this bit will be set to 1.
     * |        |          |0 = No effect.
     * |        |          |1 = Voice detected.
     * |        |          |Note: When wake-up from idle mode, user need to set CHENn DMIC_CTL[3:0] for DMIC path normal operation.
     * @var VAD_T::STATUS1
     * Offset: 0x24  VAD Status Read-back Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |DEV       |Deviation (Read Only)
     * |        |          |This field shows deviation of the Long Term Signal Power and Short Term Signal Power.
     * |[31:16] |LTP       |Long Term Signal Power (Read Only)
     * |        |          |This field shows the long term signal power value.
     */
    __IO uint32_t SINCCTL;               /*!< [0x0000] VAD SINC Filter Control Register                                 */
    __IO uint32_t BIQCTL0;               /*!< [0x0004] VAD Biquad Filter Control Register 0                             */
    __IO uint32_t BIQCTL1;               /*!< [0x0008] VAD Biquad Filter Control Register 1                             */
    __IO uint32_t BIQCTL2;               /*!< [0x000c] VAD Biquad Filter Control Register 2                             */
    __IO uint32_t CTL0;                  /*!< [0x0010] VAD Control Register 0                                           */
    __IO uint32_t CTL1;                  /*!< [0x0014] VAD Control Register 1                                           */
    __IO uint32_t CTL2;                  /*!< [0x0018] VAD Control Register 2                                           */
    __IO uint32_t CTL3;                  /*!< [0x001c] VAD Control Register 3                                           */
    __I  uint32_t STATUS0;               /*!< [0x0020] VAD Status Read-back Register 0                                  */
    __I  uint32_t STATUS1;               /*!< [0x0024] VAD Status Read-back Register 1                                 */

} VAD_T;

/**
    @addtogroup DMIC_CONST DMIC Bit Field Definition
    Constant Definitions for DMIC Controller
@{ */


#define DMIC_CTL_CHEN0_Pos               (0)                                               /*!< DMIC_T::CTL: CHEN0 Position            */
#define DMIC_CTL_CHEN0_Msk               (0x1ul << DMIC_CTL_CHEN0_Pos)                     /*!< DMIC_T::CTL: CHEN0 Mask                */

#define DMIC_CTL_CHEN1_Pos               (1)                                               /*!< DMIC_T::CTL: CHEN1 Position            */
#define DMIC_CTL_CHEN1_Msk               (0x1ul << DMIC_CTL_CHEN1_Pos)                     /*!< DMIC_T::CTL: CHEN1 Mask                */

#define DMIC_CTL_CHEN2_Pos               (2)                                               /*!< DMIC_T::CTL: CHEN2 Position            */
#define DMIC_CTL_CHEN2_Msk               (0x1ul << DMIC_CTL_CHEN2_Pos)                     /*!< DMIC_T::CTL: CHEN2 Mask                */

#define DMIC_CTL_CHEN3_Pos               (3)                                               /*!< DMIC_T::CTL: CHEN3 Position            */
#define DMIC_CTL_CHEN3_Msk               (0x1ul << DMIC_CTL_CHEN3_Pos)                     /*!< DMIC_T::CTL: CHEN3 Mask                */

#define DMIC_CTL_LCHEDGE01_Pos           (8)                                               /*!< DMIC_T::CTL: LCHEDGE01 Position        */
#define DMIC_CTL_LCHEDGE01_Msk           (0x1ul << DMIC_CTL_LCHEDGE01_Pos)                 /*!< DMIC_T::CTL: LCHEDGE01 Mask            */

#define DMIC_CTL_LCHEDGE23_Pos           (9)                                               /*!< DMIC_T::CTL: LCHEDGE23 Position        */
#define DMIC_CTL_LCHEDGE23_Msk           (0x1ul << DMIC_CTL_LCHEDGE23_Pos)                 /*!< DMIC_T::CTL: LCHEDGE23 Mask            */

#define DMIC_CTL_CH0MUTE_Pos             (12)                                              /*!< DMIC_T::CTL: CH0MUTE Position          */
#define DMIC_CTL_CH0MUTE_Msk             (0x1ul << DMIC_CTL_CH0MUTE_Pos)                   /*!< DMIC_T::CTL: CH0MUTE Mask              */

#define DMIC_CTL_CH1MUTE_Pos             (13)                                              /*!< DMIC_T::CTL: CH1MUTE Position          */
#define DMIC_CTL_CH1MUTE_Msk             (0x1ul << DMIC_CTL_CH1MUTE_Pos)                   /*!< DMIC_T::CTL: CH1MUTE Mask              */

#define DMIC_CTL_CH2MUTE_Pos             (14)                                              /*!< DMIC_T::CTL: CH2MUTE Position          */
#define DMIC_CTL_CH2MUTE_Msk             (0x1ul << DMIC_CTL_CH2MUTE_Pos)                   /*!< DMIC_T::CTL: CH2MUTE Mask              */

#define DMIC_CTL_CH3MUTE_Pos             (15)                                              /*!< DMIC_T::CTL: CH3MUTE Position          */
#define DMIC_CTL_CH3MUTE_Msk             (0x1ul << DMIC_CTL_CH3MUTE_Pos)                   /*!< DMIC_T::CTL: CH3MUTE Mask              */

#define DMIC_CTL_CH01HPFEN_Pos           (20)                                              /*!< DMIC_T::CTL: CH01HPFEN Position        */
#define DMIC_CTL_CH01HPFEN_Msk           (0x1ul << DMIC_CTL_CH01HPFEN_Pos)                 /*!< DMIC_T::CTL: CH01HPFEN Mask            */

#define DMIC_CTL_CH23HPFEN_Pos           (21)                                              /*!< DMIC_T::CTL: CH23HPFEN Position        */
#define DMIC_CTL_CH23HPFEN_Msk           (0x1ul << DMIC_CTL_CH23HPFEN_Pos)                 /*!< DMIC_T::CTL: CH23HPFEN Mask            */

#define DMIC_CTL_SWRST_Pos               (24)                                              /*!< DMIC_T::CTL: SWRST Position            */
#define DMIC_CTL_SWRST_Msk               (0x1ul << DMIC_CTL_SWRST_Pos)                     /*!< DMIC_T::CTL: SWRST Mask                */

#define DMIC_CTL_GAINSTEP_Pos            (27)                                              /*!< DMIC_T::CTL: GAINSTEP Position         */
#define DMIC_CTL_GAINSTEP_Msk            (0x3ul << DMIC_CTL_GAINSTEP_Pos)                  /*!< DMIC_T::CTL: GAINSTEP Mask             */

#define DMIC_CTL_FIFOWIDTH_Pos           (31)                                              /*!< DMIC_T::CTL: FIFOWIDTH Position        */
#define DMIC_CTL_FIFOWIDTH_Msk           (0x1ul << DMIC_CTL_FIFOWIDTH_Pos)                 /*!< DMIC_T::CTL: FIFOWIDTH Mask            */

#define DMIC_DIV_DMCLKDIV_Pos            (8)                                               /*!< DMIC_T::DIV: DMCLKDIV Position         */
#define DMIC_DIV_DMCLKDIV_Msk            (0x3ful << DMIC_DIV_DMCLKDIV_Pos)                 /*!< DMIC_T::DIV: DMCLKDIV Mask             */

#define DMIC_DIV_DMTH_Pos                (16)                                              /*!< DMIC_T::DIV: DMTH Position             */
#define DMIC_DIV_DMTH_Msk                (0x1ful << DMIC_DIV_DMTH_Pos)                     /*!< DMIC_T::DIV: DMTH Mask                 */

#define DMIC_DIV_DMTHIE_Pos              (21)                                              /*!< DMIC_T::DIV: DMTHIE Position           */
#define DMIC_DIV_DMTHIE_Msk              (0x1ul << DMIC_DIV_DMTHIE_Pos)                    /*!< DMIC_T::DIV: DMTHIE Mask               */

#define DMIC_DIV_FCLR_Pos                (22)                                              /*!< DMIC_T::DIV: FCLR Position             */
#define DMIC_DIV_FCLR_Msk                (0x3ul << DMIC_DIV_FCLR_Pos)                      /*!< DMIC_T::DIV: FCLR Mask                 */

#define DMIC_DIV_OSR_Pos                 (24)                                              /*!< DMIC_T::DIV: OSR Position              */
#define DMIC_DIV_OSR_Msk                 (0x7ul << DMIC_DIV_OSR_Pos)                       /*!< DMIC_T::DIV: OSR Mask                  */

#define DMIC_STATUS_FULL_Pos             (0)                                               /*!< DMIC_T::STATUS: FULL Position          */
#define DMIC_STATUS_FULL_Msk             (0x1ul << DMIC_STATUS_FULL_Pos)                   /*!< DMIC_T::STATUS: FULL Mask              */

#define DMIC_STATUS_EMPTY_Pos            (1)                                               /*!< DMIC_T::STATUS: EMPTY Position         */
#define DMIC_STATUS_EMPTY_Msk            (0x1ul << DMIC_STATUS_EMPTY_Pos)                  /*!< DMIC_T::STATUS: EMPTY Mask             */

#define DMIC_STATUS_DMTHIF_Pos           (2)                                               /*!< DMIC_T::STATUS: DMTHIF Position        */
#define DMIC_STATUS_DMTHIF_Msk           (0x1ul << DMIC_STATUS_DMTHIF_Pos)                 /*!< DMIC_T::STATUS: DMTHIF Mask            */

#define DMIC_STATUS_FIFOPTR_Pos          (4)                                               /*!< DMIC_T::STATUS: FIFOPTR Position       */
#define DMIC_STATUS_FIFOPTR_Msk          (0x1ful << DMIC_STATUS_FIFOPTR_Pos)               /*!< DMIC_T::STATUS: FIFOPTR Mask           */

#define DMIC_LPPDMACTL_LPPDMAEN_Pos          (0)                                               /*!< DMIC_T::LPPDMACTL: LPPDMAEN Position       */
#define DMIC_LPPDMACTL_LPPDMAEN_Msk          (0x1ul << DMIC_LPPDMACTL_LPPDMAEN_Pos)                /*!< DMIC_T::LPPDMACTL: LPPDMAEN Mask           */

#define DMIC_FIFO_FIFO_Pos               (0)                                               /*!< DMIC_T::FIFO: FIFO Position            */
#define DMIC_FIFO_FIFO_Msk               (0xfffffful << DMIC_FIFO_FIFO_Pos)              /*!< DMIC_T::FIFO: FIFO Mask                */

#define DMIC_GAINCTL0_CHyyLVOL_Pos        (0)                                               /*!< DMIC_T::GAINCTL0: CHyyLVOL Position     */
#define DMIC_GAINCTL0_CHyyLVOL_Msk        (0xfffful << DMIC_GAINCTL0_CHyyLVOL_Pos)           /*!< DMIC_T::GAINCTL0: CHyyLVOL Mask         */

#define DMIC_GAINCTL0_CHxxRVOL_Pos        (16)                                              /*!< DMIC_T::GAINCTL0: CHxxRVOL Position     */
#define DMIC_GAINCTL0_CHxxRVOL_Msk        (0xfffful << DMIC_GAINCTL0_CHxxRVOL_Pos)           /*!< DMIC_T::GAINCTL0: CHxxRVOL Mask         */

#define DMIC_GAINCTL1_CHyyLVOL_Pos        (0)                                               /*!< DMIC_T::GAINCTL1: CHyyLVOL Position     */
#define DMIC_GAINCTL1_CHyyLVOL_Msk        (0xfffful << DMIC_GAINCTL1_CHyyLVOL_Pos)           /*!< DMIC_T::GAINCTL1: CHyyLVOL Mask         */

#define DMIC_GAINCTL1_CHxxRVOL_Pos        (16)                                              /*!< DMIC_T::GAINCTL1: CHxxRVOL Position     */
#define DMIC_GAINCTL1_CHxxRVOL_Msk        (0xfffful << DMIC_GAINCTL1_CHxxRVOL_Pos)           /*!< DMIC_T::GAINCTL1: CHxxRVOL Mask         */

#define VAD_SINCCTL_SINCOSR_Pos          (8)                                               /*!< VAD_T::SINCCTL: SINCOSR Position       */
#define VAD_SINCCTL_SINCOSR_Msk          (0xful << VAD_SINCCTL_SINCOSR_Pos)                /*!< VAD_T::SINCCTL: SINCOSR Mask           */

#define VAD_SINCCTL_VADMCLKDIV_Pos       (16)                                              /*!< VAD_T::SINCCTL: VADMCLKDIV Position      */
#define VAD_SINCCTL_VADMCLKDIV_Msk       (0xfful << VAD_SINCCTL_VADMCLKDIV_Pos)               /*!< VAD_T::SINCCTL: VADMCLKDIV Mask          */

#define VAD_SINCCTL_ACTCL_Pos            (30)                                              /*!< VAD_T::SINCCTL: ACTCL Position         */
#define VAD_SINCCTL_ACTCL_Msk            (0x1ul << VAD_SINCCTL_ACTCL_Pos)                  /*!< VAD_T::SINCCTL: ACTCL Mask             */

#define VAD_SINCCTL_VADEN_Pos            (31)                                              /*!< VAD_T::SINCCTL: VADEN Position         */
#define VAD_SINCCTL_VADEN_Msk            (0x1ul << VAD_SINCCTL_VADEN_Pos)                  /*!< VAD_T::SINCCTL: VADEN Mask             */

#define VAD_BIQCTL0_BIQA1_Pos            (0)                                               /*!< VAD_T::BIQCTL0: BIQA1 Position         */
#define VAD_BIQCTL0_BIQA1_Msk            (0xfffful << VAD_BIQCTL0_BIQA1_Pos)               /*!< VAD_T::BIQCTL0: BIQA1 Mask             */

#define VAD_BIQCTL0_BIQA2_Pos            (16)                                              /*!< VAD_T::BIQCTL0: BIQA2 Position         */
#define VAD_BIQCTL0_BIQA2_Msk            (0xfffful << VAD_BIQCTL0_BIQA2_Pos)               /*!< VAD_T::BIQCTL0: BIQA2 Mask             */

#define VAD_BIQCTL1_BIQB0_Pos            (0)                                               /*!< VAD_T::BIQCTL1: BIQB0 Position         */
#define VAD_BIQCTL1_BIQB0_Msk            (0xfffful << VAD_BIQCTL1_BIQB0_Pos)               /*!< VAD_T::BIQCTL1: BIQB0 Mask             */

#define VAD_BIQCTL1_BIQB1_Pos            (16)                                              /*!< VAD_T::BIQCTL1: BIQB1 Position         */
#define VAD_BIQCTL1_BIQB1_Msk            (0xfffful << VAD_BIQCTL1_BIQB1_Pos)               /*!< VAD_T::BIQCTL1: BIQB1 Mask             */

#define VAD_BIQCTL2_BIQB2_Pos            (0)                                               /*!< VAD_T::BIQCTL2: BIQB2 Position         */
#define VAD_BIQCTL2_BIQB2_Msk            (0xfffful << VAD_BIQCTL2_BIQB2_Pos)               /*!< VAD_T::BIQCTL2: BIQB2 Mask             */

#define VAD_BIQCTL2_BIQEN_Pos            (31)                                              /*!< VAD_T::BIQCTL2: BIQEN Position         */
#define VAD_BIQCTL2_BIQEN_Msk            (0x1ul << VAD_BIQCTL2_BIQEN_Pos)                  /*!< VAD_T::BIQCTL2: BIQEN Mask             */

#define VAD_CTL0_STAT_Pos                (0)                                               /*!< VAD_T::CTL0: STAT Position             */
#define VAD_CTL0_STAT_Msk                (0xfful << VAD_CTL0_STAT_Pos)                     /*!< VAD_T::CTL0: STAT Mask                 */

#define VAD_CTL0_LTAT_Pos                (16)                                              /*!< VAD_T::CTL0: LTAT Position             */
#define VAD_CTL0_LTAT_Msk                (0xful << VAD_CTL0_LTAT_Pos)                      /*!< VAD_T::CTL0: LTAT Mask                 */

#define VAD_CTL1_STTHREHWM_Pos           (0)                                               /*!< VAD_T::CTL1: STTHREHWM Position        */
#define VAD_CTL1_STTHREHWM_Msk           (0xfffful << VAD_CTL1_STTHREHWM_Pos)              /*!< VAD_T::CTL1: STTHREHWM Mask            */

#define VAD_CTL2_LTTHRE_Pos              (16)                                              /*!< VAD_T::CTL2: LTTHRE Position           */
#define VAD_CTL2_LTTHRE_Msk              (0xfffful << VAD_CTL2_LTTHRE_Pos)                 /*!< VAD_T::CTL2: LTTHRE Mask               */

#define VAD_CTL3_DEVTHRE_Pos             (0)                                               /*!< VAD_T::CTL3: DEVTHRE Position          */
#define VAD_CTL3_DEVTHRE_Msk             (0xfffful << VAD_CTL3_DEVTHRE_Pos)                /*!< VAD_T::CTL3: DEVTHRE Mask              */

#define VAD_STATUS0_STP_Pos              (0)                                               /*!< VAD_T::STATUS0: STP Position           */
#define VAD_STATUS0_STP_Msk              (0xfffful << VAD_STATUS0_STP_Pos)                 /*!< VAD_T::STATUS0: STP Mask               */

#define VAD_STATUS0_ACTIVE_Pos           (31)                                              /*!< VAD_T::STATUS0: ACTIVE Position        */
#define VAD_STATUS0_ACTIVE_Msk           (0x1ul << VAD_STATUS0_ACTIVE_Pos)                 /*!< VAD_T::STATUS0: ACTIVE Mask            */

#define VAD_STATUS1_DEV_Pos              (0)                                               /*!< VAD_T::STATUS1: DEV Position           */
#define VAD_STATUS1_DEV_Msk              (0xfffful << VAD_STATUS1_DEV_Pos)                 /*!< VAD_T::STATUS1: DEV Mask               */

#define VAD_STATUS1_LTP_Pos              (16)                                              /*!< VAD_T::STATUS1: LTP Position           */
#define VAD_STATUS1_LTP_Msk              (0xfffful << VAD_STATUS1_LTP_Pos)                 /*!< VAD_T::STATUS1: LTP Mask               */

/** @} DMIC_CONST */
/** @} end of DMIC register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __DMIC_REG_H__ */
