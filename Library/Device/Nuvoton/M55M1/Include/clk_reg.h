/**************************************************************************//**
 * @file     clk_reg.h
 * @version  V1.00
 * @brief    CLK register definition header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __CLK_REG_H__
#define __CLK_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup CLK Clock Controller (CLK)
    Memory Mapped Structure for CLK Controller
    @{
*/

typedef struct
{


    /**
     * @var CLK_T::SRCCTL
     * Offset: 0x00  Clock Source Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LIRCEN    |LIRC Enable Bit (Write Protect)
     * |        |          |0 = 32 kHz internal low speed RC oscillator (LIRC) Disabled.
     * |        |          |1 = 32 kHz internal low speed RC oscillator (LIRC) Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D4 power is turned off.
     * |[1]     |LXTEN     |LXT Enable Bit (Write Protect) (Write Only)
     * |        |          |0 = 32.768 kHz external low speed crystal (LXT) Disabled.
     * |        |          |1 = 32.768 kHz external low speed crystal (LXT) Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D5 power is turned off.
     * |        |          |Note 3: This bit is clear if LXTFIF(CLK_CLKDSTS[13]) is 1.
     * |[2]     |MIRCEN    |MIRC Enable Bit (Write Protect)
     * |        |          |0 = 1~8 MHz internal middle speed RC oscillator (MIRC) Disabled.
     * |        |          |1 = 1~8 MHz internal middle speed RC oscillator (MIRC) Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |        |          |Note 3: This bit is write ignore when SCLKSEL(CLK_SCLKSEL[2:0]) is set to MIRC.
     * |[3]     |HIRCEN    |HIRC Enable Bit (Write Protect)
     * |        |          |0 = 12 MHz internal high speed RC oscillator (HIRC) Disabled.
     * |        |          |1 = 12 MHz internal high speed RC oscillator (HIRC) Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |        |          |Note 3: This bit is write ignore when SCLKSEL(CLK_SCLKSEL[2:0]) is set to HIRC.
     * |[4]     |HXTEN     |HXT Enable Bit (Write Protect)
     * |        |          |0 = 4~32 MHz external high speed crystal (HXT) Disabled.
     * |        |          |1 = 4~32 MHz external high speed crystal (HXT) Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |        |          |Note 3: This bit is write ignore when SCLKSEL(CLK_SCLKSEL[2:0]) is set to HXT.
     * |        |          |Note 4: This bit is clear if HXTFIF(CLK_CLKDSTS[5]) is 1.
     * |[5]     |HIRC48MEN |HIRC48M Enable Bit (Write Protect)
     * |        |          |0 = 48 MHz internal high speed RC oscillator (HIRC48M) Disabled.
     * |        |          |1 = 48 MHz internal high speed RC oscillator (HIRC48M) Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |        |          |Note 3: This bit is write ignore when SCLKSEL(CLK_SCLKSEL[2:0]) is set to HIRC48M.
     * |[6]     |APLL0EN   |APLL0 Enable Bit (Write Protect)
     * |        |          |0 = APLL0 Disabled.
     * |        |          |1 = APLL0 Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |        |          |Note 3: This bit is write ignore when SCLKSEL(CLK_SCLKSEL[2:0]) is set to APLL0.
     * |[7]     |APLL1EN   |APLL1 Enable Bit (Write Protect)
     * |        |          |0 = APLL1 Disabled.
     * |        |          |1 = APLL1 Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * @var CLK_T::STATUS
     * Offset: 0x04  Clock Status Monitor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LIRCSTB   |LIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = 32 kHz internal low speed RC oscillator (LIRC) clock is not stable or disabled.
     * |        |          |1 = 32 kHz internal low speed RC oscillator (LIRC) clock is stable and enabled.
     * |        |          |Note: This bit is not retained when D2 power is turned off.
     * |[1]     |LXTSTB    |LXT Clock Source Stable Flag (Read Only)
     * |        |          |0 = 32.768 kHz external low speed crystal oscillator (LXT) clock is not stable or disabled.
     * |        |          |1 = 32.768 kHz external low speed crystal oscillator (LXT) clock is stabled and enabled.
     * |        |          |Note: This bit is not retained when D2 power is turned off.
     * |[2]     |MIRCSTB   |MIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = Internal middle speed RC oscillator (MIRC) clock is not stable or disabled.
     * |        |          |1 = Internal middle speed RC oscillator (MIRC) clock is stable and enabled.
     * |        |          |Note: This bit is not retained when D2 power is turned off.
     * |[3]     |HIRCSTB   |HIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = 12 MHz internal high speed RC oscillator (HIRC) clock is not stable or disabled.
     * |        |          |1 = 12 MHz internal high speed RC oscillator (HIRC) clock is stable and enabled.
     * |        |          |Note: This bit is not retained when D2 power is turned off.
     * |[4]     |HXTSTB    |HXT Clock Source Stable Flag (Read Only)
     * |        |          |0 = 4~32 MHz external high speed crystal oscillator (HXT) clock is not stable or disabled.
     * |        |          |1 = 4~32 MHz external high speed crystal oscillator (HXT) clock is stable and enabled.
     * |        |          |Note: This bit is not retained when D2 power is turned off.
     * |[5]     |HIRC48MSTB|HIRC48M Clock Source Stable Flag (Read Only)
     * |        |          |0 = 48 MHz internal high speed RC oscillator (HIRC48M) clock is not stable or disabled.
     * |        |          |1 = 48 MHz internal high speed RC oscillator (HIRC48M) clock is stable and enabled.
     * |        |          |Note: This bit is not retained when D2 power is turned off.
     * |[6]     |APLL0STB  |Internal APLL0 Clock Source Stable Flag (Read Only)
     * |        |          |0 = Internal APLL0 clock is not stable or disabled.
     * |        |          |1 = Internal APLL0 clock is stable and enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[7]     |APLL1STB  |Internal APLL1 Clock Source Stable Flag (Read Only)
     * |        |          |0 = Internal APLL1 clock is not stable or disabled.
     * |        |          |1 = Internal APLL1 clock is stable and enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[8]     |SCLKSWF   |System Clock Switching Finish Flag (Read Only)
     * |        |          |This bit is updated when software switches system clock source
     * |        |          |If switch target clock is stable, this bit will be set to 1
     * |        |          |If switch target clock is not stable, this bit will be set to 0.
     * |        |          |0 = Clock switching not finish.
     * |        |          |1 = Clock switching finish.
     * |        |          |Note 1: This bit is read only
     * |        |          |After selected clock source is stable, hardware will switch system clock to selected clock automatically, and SCLKSWF will be setted automatically by hardware.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var CLK_T::MIRCCTL
     * Offset: 0x08  MIRC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |MIRC1MSTBS|MIRC1M Stable Count Select (Write Protect)
     * |        |          |00 = MIRC1M stable count is 4 clocks and clock output deviation about 2%.
     * |        |          |01 = MIRC1M stable count is 4 clocks and clock output deviation about 3%.
     * |        |          |10 = MIRC1M stable count is 3 clocks and clock output deviation about 4%.
     * |        |          |11 = MIRC1M stable count is 3 clocks and clock output deviation about 5%.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |[5:4]   |MIRC2MSTBS|MIRC2M Stable Count Select (Write Protect)
     * |        |          |00 = MIRC2M stable count is 6 clocks and clock output deviation about 2%.
     * |        |          |01 = MIRC2M stable count is 6 clocks and clock output deviation about 3%.
     * |        |          |10 = MIRC2M stable count is 5 clocks and clock output deviation about 4%.
     * |        |          |11 = MIRC2M stable count is 5 clocks and clock output deviation about 5%.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |[9:8]   |MIRC4MSTBS|MIRC4M Stable Count Select (Write Protect)
     * |        |          |00 = MIRC4M stable count is 20 clocks and clock output deviation about 2%.
     * |        |          |01 = MIRC4M stable count is 17 clocks and clock output deviation about 3%.
     * |        |          |10 = MIRC4M stable count is 16 clocks and clock output deviation about 4%.
     * |        |          |11 = MIRC4M stable count is 14 clocks and clock output deviation about 5%.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |[13:12] |MIRC8MSTBS|MIRC8M Stable Count Select (Write Protect)
     * |        |          |00 = MIRC8M stable count is 75 clocks and clock output deviation about 2%.
     * |        |          |01 = MIRC8M stable count is 56 clocks and clock output deviation about 3%.
     * |        |          |10 = MIRC8M stable count is 48 clocks and clock output deviation about 4%.
     * |        |          |11 = MIRC8M stable count is 44 clocks and clock output deviation about 5%.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |[17:16] |MIRCFSEL  |MIRC Frequency Select Bits (Write Protect)
     * |        |          |00 = Internal middle speed RC oscillator frequency is 1 MHz.
     * |        |          |01 = Internal middle speed RC oscillator frequency is 2 MHz.
     * |        |          |10 = Internal middle speed RC oscillator frequency is 4 MHz.
     * |        |          |11 = Internal middle speed RC oscillator frequency is 8 MHz.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |[20]    |MIRCFDIS  |MIRC Clock Filter Disable Bit
     * |        |          |0 = MIRC Filter enabled.
     * |        |          |1 = MIRC Filter disabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * @var CLK_T::HIRCCTL
     * Offset: 0x0C  HIRC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |HIRCSTBS  |HIRC Stable Count Select (Write Protect)
     * |        |          |00 = HIRC stable count is 23 clocks and clock output deviation about 2%.
     * |        |          |01 = HIRC stable count is 16 clocks and clock output deviation about 3%.
     * |        |          |10 = HIRC stable count is 14 clocks and clock output deviation about 4%.
     * |        |          |11 = HIRC stable count is 13 clocks and clock output deviation about 5%.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D3 power is turned off.
     * |[4]     |HIRCFDIS  |HIRC Clock Filter Disable Bit
     * |        |          |0 = HIRC Filter enabled.
     * |        |          |1 = HIRC Filter disabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * @var CLK_T::HXTCTL
     * Offset: 0x10  HXT Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |HXTGAIN   |HXT Gain Control Bit (Write Protect)
     * |        |          |The default value is set by Flash controller user configuration register XT1XSG (CONFIG0 [18:16]).
     * |        |          |Gain control is used to enlarge the gain of crystal to make sure crystal work normally.
     * |        |          |000 = HXT frequency is from 4 MHz to 8 MHz.
     * |        |          |001 = HXT frequency is from 8 MHz to 12 MHz.
     * |        |          |010 = HXT frequency is from 12 MHz to 16 MHz.
     * |        |          |011 = HXT frequency is from 16 MHz to 24 MHz.
     * |        |          |100 = HXT frequency is from 24 MHz to 32 MHz.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |        |          |Note 3: The default value is set by UCFG0[18:16].
     * |[4]     |HXTSELTYP |HXT Crystal Type Select Bit (Write Protect)
     * |        |          |0 = Select INV type.
     * |        |          |1 = Select GM type.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |[5]     |HXTMD     |HXT Mode Selection (Write Protect)
     * |        |          |0 = HXT work as crystal mode. PF.2 and PF.3 are configured as external high speed crystal (HXT) pins.
     * |        |          |1 = HXT works as external clock mode. PF.3 is configured as external clock input pin.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGCTL register.
     * |        |          |Note 2: When external clock mode enable, HXTSELTYP(CLK_HXTCTL[4]) must be set as GM type.
     * |        |          |Note 3: This bit is not retained when D2 power is turned off.
     * |[6]     |HXTFDIS   |HXT Clock Filter Disable Bit (Write Protect)
     * |        |          |0 = HXT Filter enabled.
     * |        |          |1 = HXT Filter disabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |[7]     |HXTFSEL   |HXT Filter Select (Write Protect)
     * |        |          |0 = HXT frequency is <= 32 MHz.
     * |        |          |1 = HXT frequency is <= 4 MHz.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * @var CLK_T::HIRC48MCTL
     * Offset: 0x14  HIRC48M Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |HIRC48MSTBS|HIRC48M Stable Count Select (Write Protect)
     * |        |          |00 = HIRC48M stable count is 23 clocks and clock output deviation about 2%.
     * |        |          |01 = HIRC48M stable count is 22 clocks and clock output deviation about 3%.
     * |        |          |10 = HIRC48M stable count is 21 clocks and clock output deviation about 4%.
     * |        |          |11 = HIRC48M stable count is 21 clocks and clock output deviation about 5%.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |[4]     |HIRC48MFDIS|HIRC48M Clock Filter Disable Bit
     * |        |          |0 = HIRC48M Filter enabled.
     * |        |          |1 = HIRC48M Filter disabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * @var CLK_T::APLL0CTL
     * Offset: 0x18  APLL0 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8:0]   |FBDIV     |APLL0 Feedback Divider Control (Write Protect)
     * |        |          |Refer to the formulas below the table.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |        |          |Note 3: These bits are write ignore when APLL0EN(CLK_SRCCTL[6]) is set to enable.
     * |[13:9]  |INDIV     |APLL0 Input Divider Control (Write Protect)
     * |        |          |Refer to the formulas below the table.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |        |          |Note 3: These bits are write ignore when APLL0EN(CLK_SRCCTL[6]) is set to enable.
     * |[15:14] |OUTDIV    |APLL0 Output Divider Control (Write Protect)
     * |        |          |Refer to the formulas below the table.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |        |          |Note 3: These bits are write ignore when APLL0EN(CLK_SRCCTL[6]) is set to enable.
     * |[27:16] |FRDIV     |APLL0 Fractional Divider Control (Write Protect)
     * |        |          |Refer to the formulas below the table.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |        |          |Note 3: These bits are write ignore when APLL0EN(CLK_SRCCTL[6]) is set to enable.
     * |[29:28] |STBSEL    |APLL0 Stable Counter Selection (Write Protect)
     * |        |          |00 = APLL0 stable time is 820 APLL0 source clock (source clock is equal to 4M).
     * |        |          |01 = APLL0 stable time is 2460 APLL0 source clock (4 MHz < source clock <=12 MHz).
     * |        |          |10 = APLL0 stable time is 4920 APLL0 source clock (12 MHz < source clock <=24 MHz).
     * |        |          |11 = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |        |          |Note 3: These bits are write ignore when APLL0EN(CLK_SRCCTL[6]) is set to enable.
     * |[30]    |BP        |APLL0 Bypass Control (Write Protect)
     * |        |          |0 = APLL0 is in normal mode (default).
     * |        |          |1 = APLL0 clock output is same as APLL0 input clock FIN.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |        |          |Note 3: This bit is write ignore when APLL0EN(CLK_SRCCTL[6]) is set to enable.
     * |[31]    |PLLFEN    |APLL0 Clock Filter Enable Bit
     * |        |          |0 = APLL0 Filter Disable.
     * |        |          |1 = APLL0 Filter Enable.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |        |          |Note 3: This bit is write ignore when APLL0EN(CLK_SRCCTL[6]) is set to enable.
     * @var CLK_T::APLL0SEL
     * Offset: 0x1C  APLL0 Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |APLLSRC   |APLL0 Source Clock Selection (Write Protect)
     * |        |          |00 = APLL0 source clock from 4~24 MHz external high-speed crystal oscillator (HXT).
     * |        |          |01 = APLL0 source clock from 24~32 MHz external high-speed crystal oscillator (HXT/2).
     * |        |          |10 = APLL0 source clock from 12 MHz internal high-speed oscillator (HIRC).
     * |        |          |11 = APLL0 source clock from 48 MHz internal high-speed oscillator (HIRC48M/4).
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |        |          |Note 3: These bits are write ignore when APLL0EN(CLK_SRCCTL[6]) is set to enable.
     * @var CLK_T::APLL1CTL
     * Offset: 0x20  APLL1 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8:0]   |FBDIV     |APLL1 Feedback Divider Control (Write Protect)
     * |        |          |Refer to the formulas below the table.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |        |          |Note 3: These bits are write ignore when APLL1EN(CLK_SRCCTL[7]) is set to enable.
     * |[13:9]  |INDIV     |APLL1 Input Divider Control (Write Protect)
     * |        |          |Refer to the formulas below the table.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |        |          |Note 3: These bits are write ignore when APLL1EN(CLK_SRCCTL[7]) is set to enable.
     * |[15:14] |OUTDIV    |APLL1 Output Divider Control (Write Protect)
     * |        |          |Refer to the formulas below the table.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |        |          |Note 3: These bits are write ignore when APLL1EN(CLK_SRCCTL[7]) is set to enable.
     * |[27:16] |FRDIV     |APLL1 Fractional Divider Control (Write Protect)
     * |        |          |Refer to the formulas below the table.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |        |          |Note 3: These bits are write ignore when APLL1EN(CLK_SRCCTL[7]) is set to enable.
     * |[29:28] |STBSEL    |APLL1 Stable Counter Selection (Write Protect)
     * |        |          |00 = APLL1 stable time is 820 APLL1 source clock (source clock is equal to 4M).
     * |        |          |01 = APLL1 stable time is 2460 APLL1 source clock (4 MHz < source clock <=12 MHz).
     * |        |          |10 = APLL1 stable time is 4920 APLL1 source clock (12 MHz < source clock <=24 MHz).
     * |        |          |11 = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |        |          |Note 3: These bits are write ignore when APLL1EN(CLK_SRCCTL[7]) is set to enable.
     * |[30]    |BP        |APLL1 Bypass Control (Write Protect)
     * |        |          |0 = APLL1 is in normal mode (default).
     * |        |          |1 = APLL1 clock output is same as APLL1 input clock FIN.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |        |          |Note 3: This bit is write ignore when APLL1EN(CLK_SRCCTL[7]) is set to enable.
     * |[31]    |PLLFEN    |APLL1 Clock Filter Enable Bit
     * |        |          |0 = APLL1 Filter Disable.
     * |        |          |1 = APLL1 Filter Enable.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |        |          |Note 3: This bit is write ignore when APLL1EN(CLK_SRCCTL[7]) is set to enable.
     * @var CLK_T::APLL1SEL
     * Offset: 0x24  APLL1 Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |APLLSRC   |APLL1 Source Clock Selection (Write Protect)
     * |        |          |00 = APLL1 source clock from 4~24 MHz external high-speed crystal oscillator (HXT).
     * |        |          |01 = APLL1 source clock from 24~32 MHz external high-speed crystal oscillator (HXT/2).
     * |        |          |10 = APLL1 source clock from 12 MHz internal high-speed oscillator (HIRC).
     * |        |          |11 = APLL1 source clock from 48 MHz internal high-speed oscillator (HIRC48M/4).
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |        |          |Note 3: These bits are write ignore when APLL1EN(CLK_SRCCTL[7]) is set to enable.
     * @var CLK_T::CLKOCTL
     * Offset: 0x28  Clock Output Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |FREQSEL   |Clock Output Frequency Selection
     * |        |          |The formula of output frequency is
     * |        |          |Fout = Fin/2(N+1).
     * |        |          |Fin is the input clock frequency.
     * |        |          |Fout is the frequency of divider output clock.
     * |        |          |N is the 4-bit value of FREQSEL[3:0].
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[4]     |CLKOEN    |Clock Output Enable Bit
     * |        |          |0 = Clock Output function Disabled.
     * |        |          |1 = Clock Output function Enabled.
     * |        |          |Note: This bit is not retained when D2 power is turned off.
     * |[5]     |DIV1EN    |Clock Output Divide One Enable Bit
     * |        |          |0 = Clock Output will output clock with source frequency divided by FREQSEL.
     * |        |          |1 = Clock Output will output clock with source frequency.
     * |        |          |Note: This bit is not retained when D2 power is turned off.
     * |[6]     |CLK1HZEN  |Clock Output 1Hz Enable Bit
     * |        |          |0 = 1 Hz clock output for 32.768 kHz frequency compensation Disabled.
     * |        |          |1 = 1 Hz clock output for 32.768 kHz frequency compensation Enabled.
     * |        |          |Note: This bit is not retained when D2 power is turned off.
     * @var CLK_T::CLKDCTL
     * Offset: 0x30  Clock Fail Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4]     |HXTFDEN   |HXT Clock Fail Detector Enable Bit
     * |        |          |0 = 4~32 MHz external high speed crystal oscillator (HXT) clock fail detector Disabled.
     * |        |          |1 = 4~32 MHz external high speed crystal oscillator (HXT) clock fail detector Enabled.
     * |        |          |Note: This bit is not retained when D2 power is turned off.
     * |[5]     |HXTFIEN   |HXT Clock Fail Interrupt Enable Bit
     * |        |          |0 = 4~32 MHz external high speed crystal oscillator (HXT) clock fail interrupt Disabled.
     * |        |          |1 = 4~32 MHz external high speed crystal oscillator (HXT) clock fail interrupt Enabled.
     * |        |          |Note: This bit is not retained when D2 power is turned off.
     * |[6]     |HXTFDSEL  |HXT Clock Fail Detector Selection
     * |        |          |0 = 4~32 MHz external high speed crystal oscillator (HXT) clock fail detector after HXT stable.
     * |        |          |1 = 4~32 MHz external high speed crystal oscillator (HXT) clock fail detector bypass HXT stable.
     * |        |          |Note 1: When HXT Clock Fail Detector Selection is set, detector will keep detect whether HXT is stable or not, prevent HXT fail before stable.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |[12]    |LXTFDEN   |LXT Clock Fail Detector Enable Bit
     * |        |          |0 = 32.768 kHz external low speed crystal oscillator (LXT) clock fail detector Disabled.
     * |        |          |1 = 32.768 kHz external low speed crystal oscillator (LXT) clock fail detector Enabled.
     * |        |          |Note: This bit is not retained when D2 power is turned off.
     * |[13]    |LXTFIEN   |LXT Clock Fail Interrupt Enable Bit
     * |        |          |0 = 32.768 kHz external low speed crystal oscillator (LXT) clock fail interrupt Disabled.
     * |        |          |1 = 32.768 kHz external low speed crystal oscillator (LXT) clock fail interrupt Enabled.
     * |        |          |Note: This bit is not retained when D2 power is turned off.
     * |[16]    |HXTFQDEN  |HXT Clock Frequency Range Detector Enable Bit
     * |        |          |0 = 4~32 MHz external high speed crystal oscillator (HXT) clock frequency range detector Disabled.
     * |        |          |1 = 4~32 MHz external high speed crystal oscillator (HXT) clock frequency range detector Enabled.
     * |        |          |Note: This bit is not retained when D2 power is turned off.
     * |[17]    |HXTFQIEN  |HXT Clock Frequency Range Detector Interrupt Enable Bit
     * |        |          |0 = 4~32 MHz external high speed crystal oscillator (HXT) clock frequency range detector fail interrupt Disabled.
     * |        |          |1 = 4~32 MHz external high speed crystal oscillator (HXT) clock frequency range detector fail interrupt Enabled.
     * |        |          |Note: This bit is not retained when D2 power is turned off.
     * |[18]    |HXTFQASW  |HXT Clock Frequency Range Detector Event Auto Switch Enable Bit
     * |        |          |0 = 4~32 MHz external high speed crystal oscillator (HXT) clock frequency range detector fail event happened and SCLK will not switch to HIRC automatically.
     * |        |          |1 = 4~32 MHz external high speed crystal oscillator (HXT) clock frequency range detector fail event happened and SCLK will switch to HIRC automatically.
     * |        |          |Note 1: This bit should be set before HXTFQDEN(CLK_CLKDCTL[16]).
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * @var CLK_T::CLKDSTS
     * Offset: 0x34  Clock Fail Detector Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4]     |HXTFDST   |HXT Clock Fail Detector Status Bit
     * |        |          |0 = Indicate 4~32 MHz external high speed crystal oscillator (HXT) clock fail detector already disabled.
     * |        |          |1 = Indicate 4~32 MHz external high speed crystal oscillator (HXT) clock fail detector already enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[5]     |HXTFIF    |HXT Clock Fail Interrupt Flag (Write Protect)
     * |        |          |0 = 4~32 MHz external high speed crystal oscillator (HXT) clock is normal.
     * |        |          |1 = 4~32 MHz external high speed crystal oscillator (HXT) clock stops.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: This bit is not retained when D2 power is turned off.
     * |[12]    |LXTFDST   |LXT Clock Fail Detector Status Bit
     * |        |          |0 = Indicate 32.768 kHz external low speed crystal oscillator (LXT) clock fail detector already disabled.
     * |        |          |1 = Indicate 32.768 kHz external low speed crystal oscillator (LXT) clock fail detector already enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[13]    |LXTFIF    |LXT Clock Fail Interrupt Flag (Write Protect)
     * |        |          |0 = 32.768 kHz external low speed crystal oscillator (LXT) clock is normal.
     * |        |          |1 = 32.768 kHz external low speed crystal oscillator (LXT) stops.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: This bit is not retained when D2 power is turned off.
     * |[17]    |HXTFQIF   |HXT Clock Frequency Range Detector Interrupt Flag (Write Protect)
     * |        |          |0 = 4~32 MHz external high speed crystal oscillator (HXT) clock frequency is normal.
     * |        |          |1 = 4~32 MHz external high speed crystal oscillator (HXT) clock frequency is abnormal.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: This bit is not retained when D2 power is turned off.
     * @var CLK_T::CDUPB
     * Offset: 0x38  Clock Frequency Range Detector Upper Boundary Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |UPERBD    |HXT Clock Frequency Range Detector Upper Boundary Value
     * |        |          |The bits define the high value of frequency monitor window.
     * |        |          |When HXT frequency monitor value higher than this register, the HXT frequency detect fail interrupt flag will set to 1.
     * |        |          |Frequency out of range will be asserted when HXT frequency > ((UPERBD+1)/512)* HIRC48M frequency.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var CLK_T::CDLOWB
     * Offset: 0x3C  Clock Frequency Range Detector Lower Boundary Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |LOWERBD   |HXT Clock Frequency Range Detector Lower Boundary Value
     * |        |          |The bits define the low value of frequency monitor window.
     * |        |          |When HXT frequency monitor value lower than this register, the HXT frequency detect fail interrupt flag will set to 1.
     * |        |          |Frequency out of range will be asserted when HXT frequency < ((LOWERBD+1)/512)* HIRC48M frequency.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var CLK_T::STOPREQ
     * Offset: 0x40  Clock Stop Request Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CANFD0STR |CANFD0 Clock Stop Request
     * |        |          |This bit is used to stop CANFD0 clock.
     * |        |          |0 = CANFD0 clock is not stopped by this bit. (default)
     * |        |          |1 = Set this bit and check the CANFD0STA(CLK_STOPACK[0]) is 1, then CANFD0 clock stop.
     * |        |          |Note: This bit is not retained when D1 power is turned off.
     * |[1]     |CANFD1STR |CANFD1 Clock Stop Request
     * |        |          |This bit is used to stop CANFD1 clock.
     * |        |          |0 = CANFD1 clock is not stopped by this bit. (default)
     * |        |          |1 = Set this bit and check the CANFD1STA(CLK_STOPACK[1]) is 1, then CANFD1 clock stop.
     * |        |          |Note: This bit is not retained when D1 power is turned off.
     * @var CLK_T::STOPACK
     * Offset: 0x44  Clock Stop Acknowledge Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CANFD0STA |CANFD0 Clock Stop Acknowledge (Read Only)
     * |        |          |This bit is used to check CANFD0 clock stop by setting CANFD0STR(CLK_STOPREQ[0]).
     * |        |          |0 = CANFD0 clock not stopped.
     * |        |          |1 = CANFD0 clock stopped.
     * |        |          |Note: This bit is not retained when D1 power is turned off.
     * |[1]     |CANFD1STA |CANFD1 Clock Stop Acknowledge (Read Only)
     * |        |          |This bit is used to check CANFD1 clock stop by setting CANFD1STR(CLK_STOPREQ[1]).
     * |        |          |0 = CANFD1 clock not stopped.
     * |        |          |1 = CANFD1 clock stopped.
     * |        |          |Note: This bit is not retained when D1 power is turned off.
     * @var CLK_T::ACMPCTL
     * Offset: 0x200  ACMP Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMP01CKEN|ACMP0/1 Clock Enable Bit
     * |        |          |0 = ACMP0/1 clock Disabled.
     * |        |          |1 = ACMP0/1 clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[1]     |ACMP23CKEN|ACMP2/3 Clock Enable Bit
     * |        |          |0 = ACMP2/3 clock Disabled.
     * |        |          |1 = ACMP2/3 clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::AWFCTL
     * Offset: 0x204  AWF Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |AWF0CKEN  |AWFx Clock Enable Bit
     * |        |          |0 = AWFx clock Disabled.
     * |        |          |1 = AWFx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::BPWMCTL
     * Offset: 0x208  BPWM Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BPWM0CKEN |BPWMx Clock Enable Bit
     * |        |          |0 = BPWMx clock Disabled.
     * |        |          |1 = BPWMx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[1]     |BPWM1CKEN |BPWMx Clock Enable Bit
     * |        |          |0 = BPWMx clock Disabled.
     * |        |          |1 = BPWMx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::CANFDCTL
     * Offset: 0x20C  CANFD Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CANFD0CKEN|CANFDx Clock Enable Bit
     * |        |          |0 = CANFDx clock Disabled.
     * |        |          |1 = CANFDx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[1]     |CANFD1CKEN|CANFDx Clock Enable Bit
     * |        |          |0 = CANFDx clock Disabled.
     * |        |          |1 = CANFDx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[16]    |CANRM0CKEN|CANFD0 Message SRAM Clock Enable Bit
     * |        |          |0 = CANFD0 Message SRAM clock Disabled.
     * |        |          |1 = CANFD0 Message SRAM clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[17]    |CANRM1CKEN|CANFD1 Message SRAM Clock Enable Bit
     * |        |          |0 = CANFD1 Message SRAM clock Disabled.
     * |        |          |1 = CANFD1 Message SRAM clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::CCAPCTL
     * Offset: 0x210  CCAP Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CCAP0CKEN |CCAPx Clock Enable Bit
     * |        |          |0 = CCAPx clock Disabled.
     * |        |          |1 = CCAPx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::CRCCTL
     * Offset: 0x214  CRC Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CRC0CKEN  |CRCx Clock Enable Bit
     * |        |          |0 = CRCx clock Disabled.
     * |        |          |1 = CRCx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::CRYPTOCTL
     * Offset: 0x218  CRYPTO Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CRYPTO0CKEN|CRYPTOx Clock Enable Bit
     * |        |          |0 = CRYPTOx clock Disabled.
     * |        |          |1 = CRYPTOx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::DACCTL
     * Offset: 0x21C  DAC Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DAC01CKEN |DAC0/DAC1 Clock Enable Bit
     * |        |          |0 = DAC0/1 clock Disabled.
     * |        |          |1 = DAC0/1 clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::DMICCTL
     * Offset: 0x220  DMIC Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DMIC0CKEN |DMICx Clock Enable Bit
     * |        |          |0 = DMICx clock Disabled.
     * |        |          |1 = DMICx clock Enabled.
     * |        |          |Note: This bit is not retained when D2 power is turned off.
     * @var CLK_T::EADCCTL
     * Offset: 0x224  EADC Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EADC0CKEN |EADCx Clock Enable Bit
     * |        |          |0 = EADCx clock Disabled.
     * |        |          |1 = EADCx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::EBICTL
     * Offset: 0x228  EBI Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EBI0CKEN  |EBIx Clock Enable Bit
     * |        |          |0 = EBIx clock Disabled.
     * |        |          |1 = EBIx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::ECAPCTL
     * Offset: 0x22C  ECAP Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ECAP0CKEN |ECAPx Clock Enable Bit
     * |        |          |0 = ECAPx clock Disabled.
     * |        |          |1 = ECAPx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[1]     |ECAP1CKEN |ECAPx Clock Enable Bit
     * |        |          |0 = ECAPx clock Disabled.
     * |        |          |1 = ECAPx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[2]     |ECAP2CKEN |ECAPx Clock Enable Bit
     * |        |          |0 = ECAPx clock Disabled.
     * |        |          |1 = ECAPx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[3]     |ECAP3CKEN |ECAPx Clock Enable Bit
     * |        |          |0 = ECAPx clock Disabled.
     * |        |          |1 = ECAPx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::EMACCTL
     * Offset: 0x230  EMAC Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EMAC0CKEN |EMACx Clock Enable Bit
     * |        |          |0 = EMACx clock Disabled.
     * |        |          |1 = EMACx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::EPWMCTL
     * Offset: 0x234  EPWM Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EPWM0CKEN |EPWMx Clock Enable Bit
     * |        |          |0 = EPWMx clock Disabled.
     * |        |          |1 = EPWMx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[1]     |EPWM1CKEN |EPWMx Clock Enable Bit
     * |        |          |0 = EPWMx clock Disabled.
     * |        |          |1 = EPWMx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::EQEICTL
     * Offset: 0x238  EQEI Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EQEI0CKEN |EQEIx Clock Enable Bit
     * |        |          |0 = EQEIx clock Disabled.
     * |        |          |1 = EQEIx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[1]     |EQEI1CKEN |EQEIx Clock Enable Bit
     * |        |          |0 = EQEIx clock Disabled.
     * |        |          |1 = EQEIx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[2]     |EQEI2CKEN |EQEIx Clock Enable Bit
     * |        |          |0 = EQEIx clock Disabled.
     * |        |          |1 = EQEIx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[3]     |EQEI3CKEN |EQEIx Clock Enable Bit
     * |        |          |0 = EQEIx clock Disabled.
     * |        |          |1 = EQEIx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::FMCCTL
     * Offset: 0x23C  FMC Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FMC0CKEN  |FMCx Clock Enable Bit
     * |        |          |0 = FMC clock Disabled when chip is under IDLE mode.
     * |        |          |1 = FMC clock Enabled when chip is under IDLE mode.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[16]    |ISP0CKEN  |ISPx Clock Enable Bit
     * |        |          |0 = ISPx clock Disabled.
     * |        |          |1 = ISPx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::GDMACTL
     * Offset: 0x240  GDMA Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GDMA0CKEN |GDMAx Clock Enable Bit
     * |        |          |0 = GDMAx clock Disabled.
     * |        |          |1 = GDMAx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::GPIOCTL
     * Offset: 0x244  GPIO Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GPIOACKEN |GPIOA Clock Enable Bit
     * |        |          |0 = GPIOA clock Disabled.
     * |        |          |1 = GPIOA clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[1]     |GPIOBCKEN |GPIOB Clock Enable Bit
     * |        |          |0 = GPIOB clock Disabled.
     * |        |          |1 = GPIOB clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[2]     |GPIOCCKEN |GPIOC Clock Enable Bit
     * |        |          |0 = GPIOC clock Disabled.
     * |        |          |1 = GPIOC clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[3]     |GPIODCKEN |GPIOD Clock Enable Bit
     * |        |          |0 = GPIOD clock Disabled.
     * |        |          |1 = GPIOD clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[4]     |GPIOECKEN |GPIOE Clock Enable Bit
     * |        |          |0 = GPIOE clock Disabled.
     * |        |          |1 = GPIOE clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[5]     |GPIOFCKEN |GPIOF Clock Enable Bit
     * |        |          |0 = GPIOF clock Disabled.
     * |        |          |1 = GPIOF clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[6]     |GPIOGCKEN |GPIOG Clock Enable Bit
     * |        |          |0 = GPIOG clock Disabled.
     * |        |          |1 = GPIOG clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[7]     |GPIOHCKEN |GPIOH Clock Enable Bit
     * |        |          |0 = GPIOH clock Disabled.
     * |        |          |1 = GPIOH clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[8]     |GPIOICKEN |GPIOI Clock Enable Bit
     * |        |          |0 = GPIOI clock Disabled.
     * |        |          |1 = GPIOI clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[9]     |GPIOJCKEN |GPIOJ Clock Enable Bit
     * |        |          |0 = GPIOJ clock Disabled.
     * |        |          |1 = GPIOJ clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::HSOTGCTL
     * Offset: 0x248  HSOTG Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |HSOTG0CKEN|HSOTGx Clock Enable Bit
     * |        |          |0 = HSOTGx clock Disabled.
     * |        |          |1 = HSOTGx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::HSUSBDCTL
     * Offset: 0x24C  HSUSBD Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |HSUSBD0CKEN|HSUSBDx Clock Enable Bit
     * |        |          |0 = HSUSBDx clock Disabled.
     * |        |          |1 = HSUSBDx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::HSUSBHCTL
     * Offset: 0x250  HSUSBH Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |HSUSBH0CKEN|HSUSBHx Clock Enable Bit
     * |        |          |0 = HSUSBHx clock Disabled.
     * |        |          |1 = HSUSBHx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::I2CCTL
     * Offset: 0x254  I2C Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |I2C0CKEN  |I2Cx Clock Enable Bit
     * |        |          |0 = I2Cx clock Disabled.
     * |        |          |1 = I2Cx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[1]     |I2C1CKEN  |I2Cx Clock Enable Bit
     * |        |          |0 = I2Cx clock Disabled.
     * |        |          |1 = I2Cx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[2]     |I2C2CKEN  |I2Cx Clock Enable Bit
     * |        |          |0 = I2Cx clock Disabled.
     * |        |          |1 = I2Cx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[3]     |I2C3CKEN  |I2Cx Clock Enable Bit
     * |        |          |0 = I2Cx clock Disabled.
     * |        |          |1 = I2Cx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::I2SCTL
     * Offset: 0x258  I2S Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |I2S0CKEN  |I2Sx Clock Enable Bit
     * |        |          |0 = I2Sx clock Disabled.
     * |        |          |1 = I2Sx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[1]     |I2S1CKEN  |I2Sx Clock Enable Bit
     * |        |          |0 = I2Sx clock Disabled.
     * |        |          |1 = I2Sx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::I3CCTL
     * Offset: 0x25C  I3C Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |I3C0CKEN  |I3Cx Clock Enable Bit
     * |        |          |0 = I3Cx clock Disabled.
     * |        |          |1 = I3Cx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::KDFCTL
     * Offset: 0x260  KDF Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |KDF0CKEN  |KDFx Clock Enable Bit
     * |        |          |0 = KDFx clock Disabled.
     * |        |          |1 = KDFx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::KPICTL
     * Offset: 0x264  KPI Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |KPI0CKEN  |KPIx Clock Enable Bit
     * |        |          |0 = KPIx clock Disabled.
     * |        |          |1 = KPIx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::KSCTL
     * Offset: 0x268  KS Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |KS0CKEN   |KSx Clock Enable Bit
     * |        |          |0 = KSx clock Disabled.
     * |        |          |1 = KSx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::LPADCCTL
     * Offset: 0x26C  Low Power ADC Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPADC0CKEN|LPADCx Clock Enable Bit
     * |        |          |0 = LPADCx clock Disabled.
     * |        |          |1 = LPADCx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::LPPDMACTL
     * Offset: 0x270  Low Power PDMA Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPPDMA0CKEN|LPPDMAx Clock Enable Bit
     * |        |          |0 = LPPDMAx clock Disabled.
     * |        |          |1 = LPPDMAx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::LPGPIOCTL
     * Offset: 0x274  Low Power GPIO Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPGPIO0CKEN|LPGPIOx Clock Enable Bit
     * |        |          |0 = LPGPIOx clock Disabled.
     * |        |          |1 = LPGPIOx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::LPI2CCTL
     * Offset: 0x278  Low Power I2C Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPI2C0CKEN|LPI2Cx Clock Enable Bit
     * |        |          |0 = LPI2Cx clock Disabled.
     * |        |          |1 = LPI2Cx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::LPSPICTL
     * Offset: 0x27C  Low Power SPI Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPSPI0CKEN|LPSPIx Clock Enable Bit
     * |        |          |0 = LPSPIx clock Disabled.
     * |        |          |1 = LPSPIx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::LPSRAMCTL
     * Offset: 0x280  Low Power SRAM Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPSRAM0CKEN|LPSRAMx Clock Enable Bit
     * |        |          |0 = LPSRAMx clock Disabled.
     * |        |          |1 = LPSRAMx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::LPTMRCTL
     * Offset: 0x284  Low Power Timer Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPTMR0CKEN|LPTMRx Clock Enable Bit
     * |        |          |0 = LPTMRx clock Disabled.
     * |        |          |1 = LPTMRx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[1]     |LPTMR1CKEN|LPTMRx Clock Enable Bit
     * |        |          |0 = LPTMRx clock Disabled.
     * |        |          |1 = LPTMRx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::LPUARTCTL
     * Offset: 0x288  Low Power UART Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPUART0CKEN|LPUARTx Clock Enable Bit
     * |        |          |0 = LPUARTx clock Disabled.
     * |        |          |1 = LPUARTx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::NPUCTL
     * Offset: 0x28C  NPU Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |NPU0CKEN  |NPUx Clock Enable Bit
     * |        |          |0 = NPUx clock Disabled.
     * |        |          |1 = NPUx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::OTFCCTL
     * Offset: 0x294  OTFC Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |OTFC0CKEN |OTFCx Clock Enable Bit
     * |        |          |0 = OTFCx clock Disabled.
     * |        |          |1 = OTFCx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::OTGCTL
     * Offset: 0x298  OTG Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |OTG0CKEN  |OTGx Clock Enable Bit
     * |        |          |0 = OTGx clock Disabled.
     * |        |          |1 = OTGx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::PDMACTL
     * Offset: 0x29C  PDMA Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PDMA0CKEN |PDMAx Clock Enable Bit
     * |        |          |0 = PDMAx clock Disabled.
     * |        |          |1 = PDMAx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[1]     |PDMA1CKEN |PDMAx Clock Enable Bit
     * |        |          |0 = PDMAx clock Disabled.
     * |        |          |1 = PDMAx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::PSIOCTL
     * Offset: 0x2A0  PSIO Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PSIO0CKEN |PSIOx Clock Enable Bit
     * |        |          |0 = PSIOx clock Disabled.
     * |        |          |1 = PSIOx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::QSPICTL
     * Offset: 0x2A4  QSPI Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |QSPI0CKEN |QSPIx Clock Enable Bit
     * |        |          |0 = QSPIx clock Disabled.
     * |        |          |1 = QSPIx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[1]     |QSPI1CKEN |QSPIx Clock Enable Bit
     * |        |          |0 = QSPIx clock Disabled.
     * |        |          |1 = QSPIx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::RTCCTL
     * Offset: 0x2A8  RTC Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RTC0CKEN  |RTCx Clock Enable Bit
     * |        |          |0 = RTCx clock Disabled.
     * |        |          |1 = RTCx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::SCCTL
     * Offset: 0x2AC  SC Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SC0CKEN   |SCx Clock Enable Bit
     * |        |          |0 = SCx clock Disabled.
     * |        |          |1 = SCx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[1]     |SC1CKEN   |SCx Clock Enable Bit
     * |        |          |0 = SCx clock Disabled.
     * |        |          |1 = SCx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[2]     |SC2CKEN   |SCx Clock Enable Bit
     * |        |          |0 = SCx clock Disabled.
     * |        |          |1 = SCx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::SCUCTL
     * Offset: 0x2B0  SCU Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SCU0CKEN  |SCUx Clock Enable Bit
     * |        |          |0 = SCUx clock Disabled.
     * |        |          |1 = SCUx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::SDHCTL
     * Offset: 0x2B4  SDH Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SDH0CKEN  |SDHx Clock Enable Bit
     * |        |          |0 = SDHx clock Disabled.
     * |        |          |1 = SDHx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[1]     |SDH1CKEN  |SDHx Clock Enable Bit
     * |        |          |0 = SDHx clock Disabled.
     * |        |          |1 = SDHx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::SPICTL
     * Offset: 0x2B8  SPI Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SPI0CKEN  |SPIx Clock Enable Bit
     * |        |          |0 = SPIx clock Disabled.
     * |        |          |1 = SPIx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[1]     |SPI1CKEN  |SPIx Clock Enable Bit
     * |        |          |0 = SPIx clock Disabled.
     * |        |          |1 = SPIx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[2]     |SPI2CKEN  |SPIx Clock Enable Bit
     * |        |          |0 = SPIx clock Disabled.
     * |        |          |1 = SPIx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[3]     |SPI3CKEN  |SPIx Clock Enable Bit
     * |        |          |0 = SPIx clock Disabled.
     * |        |          |1 = SPIx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::SPIMCTL
     * Offset: 0x2BC  SPIM Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SPIM0CKEN |SPIMx Clock Enable Bit
     * |        |          |0 = SPIMx clock Disabled.
     * |        |          |1 = SPIMx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::SRAMCTL
     * Offset: 0x2C0  System SRAM Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SRAM0CKEN |SRAM0 Clock Enable Bit
     * |        |          |0 = SRAM0 clock Disabled.
     * |        |          |1 = SRAM0 clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[1]     |SRAM1CKEN |SRAM1 Clock Enable Bit
     * |        |          |0 = SRAM1 clock Disabled.
     * |        |          |1 = SRAM1 clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[2]     |SRAM2CKEN |SRAM2 Clock Enable Bit
     * |        |          |0 = SRAM2 clock Disabled.
     * |        |          |1 = SRAM2 clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[3]     |SRAM3CKEN |SRAM3 Clock Enable Bit
     * |        |          |0 = SRAM3 clock Disabled.
     * |        |          |1 = SRAM3 clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::STCTL
     * Offset: 0x2CC  System Tick Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ST0CKEN   |SYSTICKx Clock Enable Bit
     * |        |          |0 = SYSTICKx clock Disabled.
     * |        |          |1 = SYSTICKx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::TMRCTL
     * Offset: 0x2D4  Timer Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TMR0CKEN  |TIMERx Clock Enable Bit
     * |        |          |0 = TIMERx clock Disabled.
     * |        |          |1 = TIMERx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[1]     |TMR1CKEN  |TIMERx Clock Enable Bit
     * |        |          |0 = TIMERx clock Disabled.
     * |        |          |1 = TIMERx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[2]     |TMR2CKEN  |TIMERx Clock Enable Bit
     * |        |          |0 = TIMERx clock Disabled.
     * |        |          |1 = TIMERx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[3]     |TMR3CKEN  |TIMERx Clock Enable Bit
     * |        |          |0 = TIMERx clock Disabled.
     * |        |          |1 = TIMERx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::TRNGCTL
     * Offset: 0x2D8  TRNG Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TRNG0CKEN |TRNGx Clock Enable Bit
     * |        |          |0 = TRNGx clock Disabled.
     * |        |          |1 = TRNGx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::TTMRCTL
     * Offset: 0x2DC  Tick Timer Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TTMR0CKEN |TTMRx Clock Enable Bit
     * |        |          |0 = TTMRx clock Disabled.
     * |        |          |1 = TTMRx clock Enabled.
     * |        |          |Note: This bit is not retained when D2 power is turned off.
     * |[1]     |TTMR1CKEN |TTMRx Clock Enable Bit
     * |        |          |0 = TTMRx clock Disabled.
     * |        |          |1 = TTMRx clock Enabled.
     * |        |          |Note: This bit is not retained when D2 power is turned off.
     * @var CLK_T::UARTCTL
     * Offset: 0x2E0  UART Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |UART0CKEN |UARTx Clock Enable Bit
     * |        |          |0 = UARTx clock Disabled.
     * |        |          |1 = UARTx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[1]     |UART1CKEN |UARTx Clock Enable Bit
     * |        |          |0 = UARTx clock Disabled.
     * |        |          |1 = UARTx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[2]     |UART2CKEN |UARTx Clock Enable Bit
     * |        |          |0 = UARTx clock Disabled.
     * |        |          |1 = UARTx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[3]     |UART3CKEN |UARTx Clock Enable Bit
     * |        |          |0 = UARTx clock Disabled.
     * |        |          |1 = UARTx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[4]     |UART4CKEN |UARTx Clock Enable Bit
     * |        |          |0 = UARTx clock Disabled.
     * |        |          |1 = UARTx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[5]     |UART5CKEN |UARTx Clock Enable Bit
     * |        |          |0 = UARTx clock Disabled.
     * |        |          |1 = UARTx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[6]     |UART6CKEN |UARTx Clock Enable Bit
     * |        |          |0 = UARTx clock Disabled.
     * |        |          |1 = UARTx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[7]     |UART7CKEN |UARTx Clock Enable Bit
     * |        |          |0 = UARTx clock Disabled.
     * |        |          |1 = UARTx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[8]     |UART8CKEN |UARTx Clock Enable Bit
     * |        |          |0 = UARTx clock Disabled.
     * |        |          |1 = UARTx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * |[9]     |UART9CKEN |UARTx Clock Enable Bit
     * |        |          |0 = UARTx clock Disabled.
     * |        |          |1 = UARTx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::USBDCTL
     * Offset: 0x2E4  USBD Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |USBD0CKEN |USBDx Clock Enable Bit
     * |        |          |0 = USBDx clock Disabled.
     * |        |          |1 = USBDx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::USBHCTL
     * Offset: 0x2E8  USBH Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |USBH0CKEN |USBHx Clock Enable Bit
     * |        |          |0 = USBHx clock Disabled.
     * |        |          |1 = USBHx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::USCICTL
     * Offset: 0x2EC  USCI Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |USCI0CKEN |USCIx Clock Enable Bit
     * |        |          |0 = USCIx clock Disabled.
     * |        |          |1 = USCIx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::UTCPDCTL
     * Offset: 0x2F0  UTCPD Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |UTCPD0CKEN|UTCPDx Clock Enable Bit
     * |        |          |0 = UTCPDx clock Disabled.
     * |        |          |1 = UTCPDx clock Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var CLK_T::WDTCTL
     * Offset: 0x2F4  WDT Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WDT0CKEN  |WDTx Clock Enable Bit
     * |        |          |0 = WDTx clock Disabled.
     * |        |          |1 = WDTx clock Enabled.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |[1]     |WDT1CKEN  |WDTx Clock Enable Bit
     * |        |          |0 = WDTx clock Disabled.
     * |        |          |1 = WDTx clock Enabled.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * @var CLK_T::WWDTCTL
     * Offset: 0x2F8  WWDT Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDT0CKEN |WWDTx Clock Enable Bit
     * |        |          |0 = WWDTx clock Disabled.
     * |        |          |1 = WWDTx clock Enabled.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |WWDT1CKEN |WWDTx Clock Enable Bit
     * |        |          |0 = WWDTx clock Disabled.
     * |        |          |1 = WWDTx clock Enabled.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var CLK_T::SCLKSEL
     * Offset: 0x400  System Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |SCLKSEL   |System Clock Source Selection (Write Protect)
     * |        |          |The system clock source is defined by SCLKSEL.
     * |        |          |000 = Clock source from HIRC.
     * |        |          |001 = Clock source from MIRC.
     * |        |          |010 = Clock source from HIRC48M.
     * |        |          |011 = Clock source from HXT.
     * |        |          |100 = Clock source from APLL0.
     * |        |          |Others = Reserved.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var CLK_T::BPWMSEL
     * Offset: 0x404  BPWM Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BPWM0SEL  |BPWM0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of BPWM0 is defined by BPWM0SEL.
     * |        |          |0 = Clock source from PCLK0.
     * |        |          |1 = Clock source from HCLK0.
     * |        |          |Note 0: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 1: This bit is not retained when D1 power is turned off.
     * |[4]     |BPWM1SEL  |BPWM1 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of BPWM1 is defined by BPWM1SEL.
     * |        |          |0 = Clock source from PCLK2.
     * |        |          |1 = Clock source from HCLK0.
     * |        |          |Note 0: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 1: This bit is not retained when D1 power is turned off.
     * @var CLK_T::CANFDSEL
     * Offset: 0x408  CANFD Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |CANFD0SEL |CANFD0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of CANFD0 is defined by CANFD0SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from APLL0/2.
     * |        |          |010 = Clock source from HCLK0.
     * |        |          |011 = Clock source from HIRC.
     * |        |          |100 = Clock source from HIRC48M/4.
     * |        |          |Others = Reserved.
     * |        |          |Note 0: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 1: These bits are not retained when D1 power is turned off.
     * |[6:4]   |CANFD1SEL |CANFD1 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of CANFD1 is defined by CANFD1SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from APLL0/2.
     * |        |          |010 = Clock source from HCLK0.
     * |        |          |011 = Clock source from HIRC.
     * |        |          |100 = Clock source from HIRC48M/4.
     * |        |          |Others = Reserved.
     * |        |          |Note 0: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 1: These bits are not retained when D1 power is turned off.
     * @var CLK_T::CCAPSEL
     * Offset: 0x40C  CCAP Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |CCAP0SEL  |CCAP Sensor Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of CCAP is defined by CCAP0SEL.
     * |        |          |000 = Clock source from MIRC.
     * |        |          |001 = Clock source from HCLK2.
     * |        |          |010 = Clock source from HIRC.
     * |        |          |011 = Clock source from APLL0/2.
     * |        |          |100 = Clock source from HXT.
     * |        |          |Others = Reserved.
     * |        |          |Note 0: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * @var CLK_T::CLKOSEL
     * Offset: 0x410  Clock Output Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |CLKOSEL   |Clock Output Clock Source Selection (Write Protect)
     * |        |          |0000 = Clock source from SCLK.
     * |        |          |0001 = Clock source from ACLK.
     * |        |          |0010 = Clock source from HCLK0.
     * |        |          |0011 = Clock source from HCLK1.
     * |        |          |0100 = Clock source from HCLK2.
     * |        |          |0101 = Clock source from APLL0/2.
     * |        |          |0110 = Clock source from APLL1/2.
     * |        |          |0111 = Clock source from HIRC48M.
     * |        |          |1000 = Clock source from HXT.
     * |        |          |1001 = Clock source from HIRC.
     * |        |          |1010 = Clock source from MIRC.
     * |        |          |1011 = Clock source from LXT.
     * |        |          |1100 = Clock source from LIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var CLK_T::DMICSEL
     * Offset: 0x414  DMIC Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |DMIC0SEL  |DMIC Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of DIMC0 is defined by DMIC0SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from APLL1/2.
     * |        |          |010 = Clock source from MIRC.
     * |        |          |011 = Clock source from HIRC.
     * |        |          |100 = Clock source from HIRC48M.
     * |        |          |101 = Clock source from PCLK4.
     * |        |          |Others = Reserved.
     * |        |          |Note 0: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * |[5:4]   |VAD0SEL   |VAD Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock VAD source of DIMC0 is defined by VAD0SEL.
     * |        |          |00 = Clock source from PCLK4.
     * |        |          |01 = Clock source from MIRC.
     * |        |          |10 = Clock source from HIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note 0: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * @var CLK_T::EADCSEL
     * Offset: 0x418  EADC Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |EADC0SEL  |EADC0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of EADC0 is defined by EADC0SEL.
     * |        |          |00 = Clock source from APLL1/2.
     * |        |          |01 = Clock source from APLL0/2.
     * |        |          |10 = PCLK0.
     * |        |          |Others = Reserved.
     * |        |          |Note 0: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 1: These bits are not retained when D1 power is turned off.
     * @var CLK_T::EPWMSEL
     * Offset: 0x41C  EPWM Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EPWM0SEL  |EPWM0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of EPWM0 is defined by EPWM0SEL.
     * |        |          |0 = Clock source from PCLK0.
     * |        |          |1 = Clock source from HCLK0.
     * |        |          |Note 0: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 1: This bit is not retained when D1 power is turned off.
     * |[4]     |EPWM1SEL  |EPWM1 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of EPWM1 is defined by EPWM1SEL.
     * |        |          |0 = Clock source from PCLK2.
     * |        |          |1 = Clock source from HCLK0.
     * |        |          |Note 0: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 1: This bit is not retained when D1 power is turned off.
     * @var CLK_T::FMCSEL
     * Offset: 0x420  FMC Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FMC0SEL   |FMC0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of FMC0 is defined by FMC0SEL.
     * |        |          |0 = Clock source from HIRC.
     * |        |          |1 = Clock source from HIRC48M/4.
     * |        |          |Note 0: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 1: This bit is not retained when D0 power is turned off.
     * @var CLK_T::I2SSEL
     * Offset: 0x424  I2S Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |I2S0SEL   |I2S0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of I2S0 is defined by I2S0SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from APLL1/2.
     * |        |          |010 = Clock source from APLL0/2.
     * |        |          |011 = Clock source from PCLK1.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |101 = Clock source from HIRC48M.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[6:4]   |I2S1SEL   |I2S1 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of I2S1 is defined by I2S1SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from APLL1/2.
     * |        |          |010 = Clock source from APLL0/2.
     * |        |          |011 = Clock source from PCLK3.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |101 = Clock source from HIRC48M.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * @var CLK_T::I3CSEL
     * Offset: 0x428  I3C Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |I3C0SEL   |I3C0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of I3C0 is defined by I3C0SEL.
     * |        |          |0 = Clock source from HCLK0.
     * |        |          |1 = Clock source from APLL1/2.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * @var CLK_T::KPISEL
     * Offset: 0x42C  KPI Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |KPI0SEL   |KPI0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of KPI0 is defined by KPI0SEL.
     * |        |          |00 = Clock source from HIRC48M/4.
     * |        |          |01 = Clock source from HIRC.
     * |        |          |10 = Clock source from LIRC.
     * |        |          |11 = Clock source from HXT.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * @var CLK_T::LPADCSEL
     * Offset: 0x430  Low Power ADC Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |LPADC0SEL |LPADC0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of LPADC0 is defined by LPADC0SEL.
     * |        |          |00 = Clock source from PCLK4.
     * |        |          |01 = Clock source from LXT.
     * |        |          |10 = Clock source from MIRC.
     * |        |          |11 = Clock source from HIRC.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * @var CLK_T::LPSPISEL
     * Offset: 0x434  Low Power SPI Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |LPSPI0SEL |LPSPI0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of LPSPI0 is defined by LPSPI0SEL.
     * |        |          |00 = Clock source from PCLK4.
     * |        |          |01 = Clock source from MIRC.
     * |        |          |10 = Clock source from HIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * @var CLK_T::LPTMRSEL
     * Offset: 0x438  Low Power Timer Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |LPTMR0SEL |LPTMR0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of LPTMR0 is defined by LPTMR0SEL.
     * |        |          |000 = Clock source from PCLK4.
     * |        |          |001 = Clock source from LXT.
     * |        |          |010 = Clock source from LIRC.
     * |        |          |011 = Clock source from MIRC.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |101 = Clock source from external clock TM0 pin.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |[6:4]   |LPTMR1SEL |LPTMR1 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of LPTMR1 is defined by LPTMR1SEL.
     * |        |          |000 = Clock source from PCLK4.
     * |        |          |001 = Clock source from LXT.
     * |        |          |010 = Clock source from LIRC.
     * |        |          |011 = Clock source from MIRC.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |101 = Clock source from external clock TM1 pin.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * @var CLK_T::LPUARTSEL
     * Offset: 0x43C  Low Power UART Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |LPUART0SEL|LPUART0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of LPUART0 is defined by LPUART0SEL.
     * |        |          |00 = Clock source from PCLK4.
     * |        |          |01 = Clock source from LXT.
     * |        |          |10 = Clock source from MIRC.
     * |        |          |11 = Clock source from HIRC.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * @var CLK_T::PSIOSEL
     * Offset: 0x440  PSIO Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |PSIO0SEL  |PSIO0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of PSIO0 is defined by PSIO0SEL.
     * |        |          |000 = Clock source from LXT.
     * |        |          |001 = Clock source from HXT.
     * |        |          |010 = Clock source from LIRC.
     * |        |          |011 = Clock source from HIRC.
     * |        |          |100 = Clock source from HIRC48M/4.
     * |        |          |101 = Clock source from PCLK1.
     * |        |          |110 = Clock source from APLL0/2.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * @var CLK_T::QSPISEL
     * Offset: 0x444  QSPI Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |QSPI0SEL  |QSPI0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of QSPI0 is defined by QSPI0SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from APLL0/2.
     * |        |          |010 = Clock source from PCLK0.
     * |        |          |011 = Clock source from HIRC.
     * |        |          |100 = Clock source from HIRC48M/4.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[6:4]   |QSPI1SEL  |QSPI1 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of QSPI1 is defined by QSPI1SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from APLL0/2.
     * |        |          |010 = Clock source from PCLK2.
     * |        |          |011 = Clock source from HIRC.
     * |        |          |100 = Clock source from HIRC48M/4.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * @var CLK_T::SCSEL
     * Offset: 0x448  SC Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |SC0SEL    |SC0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of SC0 is defined by SC0SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from APLL0/2.
     * |        |          |010 = Clock source from PCLK1.
     * |        |          |011 = Clock source from HIRC.
     * |        |          |100 = Clock source from HIRC48M/4.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[6:4]   |SC1SEL    |SC1 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of SC1 is defined by SC1SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from APLL0/2.
     * |        |          |010 = Clock source from PCLK3.
     * |        |          |011 = Clock source from HIRC.
     * |        |          |100 = Clock source from HIRC48M/4.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[10:8]  |SC2SEL    |SC2 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of SC2 is defined by SC2SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from APLL0/2.
     * |        |          |010 = Clock source from PCLK1.
     * |        |          |011 = Clock source from HIRC.
     * |        |          |100 = Clock source from HIRC48M/4.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * @var CLK_T::SDHSEL
     * Offset: 0x44C  SDH Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |SDH0SEL   |SDH0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of SDH0 is defined by SDH0SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from APLL1/2.
     * |        |          |010 = Clock source from HCLK0.
     * |        |          |011 = Clock source from HIRC.
     * |        |          |100 = Clock source from HIRC48M/4.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[6:4]   |SDH1SEL   |SDH1 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of SDH1 is defined by SDH1SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from APLL1/2.
     * |        |          |010 = Clock source from HCLK0.
     * |        |          |011 = Clock source from HIRC.
     * |        |          |100 = Clock source from HIRC48M/4.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * @var CLK_T::SPISEL
     * Offset: 0x450  SPI Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |SPI0SEL   |SPI0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of SPI0 is defined by SPI0SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from APLL1/2.
     * |        |          |010 = Clock source from APLL0/2.
     * |        |          |011 = Clock source from PCLK0.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |101 = Clock source from HIRC48M.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[6:4]   |SPI1SEL   |SPI1 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of SPI1 is defined by SPI1SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from APLL1/2.
     * |        |          |010 = Clock source from APLL0/2.
     * |        |          |011 = Clock source from PCLK2.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |101 = Clock source from HIRC48M.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[10:8]  |SPI2SEL   |SPI2 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of SPI2 is defined by SPI2SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from APLL1/2.
     * |        |          |010 = Clock source from APLL0/2.
     * |        |          |011 = Clock source from PCLK0.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |101 = Clock source from HIRC48M.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[14:12] |SPI3SEL   |SPI3 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of SPI3 is defined by SPI3SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from APLL1/2.
     * |        |          |010 = Clock source from APLL0/2.
     * |        |          |011 = Clock source from PCLK2.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |101 = Clock source from HIRC48M.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * @var CLK_T::STSEL
     * Offset: 0x454  System Tick Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |ST0SEL    |Cortex-M55 SysTick 0 Clock Source Selection (Write Protect)
     * |        |          |If SYST_CTRL[2]=0, SysTick uses listed clock source below.
     * |        |          |00 = Clock source from HXT.
     * |        |          |01 = Clock source from HXT/2.
     * |        |          |10 = Clock source from ACLK/2.
     * |        |          |11 = Clock source from HIRC/2.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: if SysTick clock source is not from CPUCLK (i.e
     * |        |          |SYST_CTRL[2] = 0), SysTick needs to enable ST0CKEN(CLK_STCTL [0])
     * |        |          |SysTick clock source must less than or equal to ACLK/2.
     * |        |          |Note 2: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: These bits are not retained when D0 power is turned off.
     * @var CLK_T::TMRSEL
     * Offset: 0x458  Timer Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |TMR0SEL   |Timer 0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of TMR0 is defined by TMR0SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from LXT.
     * |        |          |010 = Clock source from PCLK1.
     * |        |          |011 = Clock source from external clock TM0 pin.
     * |        |          |100 = Clock source from LIRC.
     * |        |          |101 = Clock source from HIRC.
     * |        |          |110 = Clock source from HIRC48M/4.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[6:4]   |TMR1SEL   |Timer 1 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of TMR1 is defined by TMR1SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from LXT.
     * |        |          |010 = Clock source from PCLK1.
     * |        |          |011 = Clock source from external clock TM1 pin.
     * |        |          |100 = Clock source from LIRC.
     * |        |          |101 = Clock source from HIRC.
     * |        |          |110 = Clock source from HIRC48M/4.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[10:8]  |TMR2SEL   |Timer 2 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of TMR2 is defined by TMR2SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from LXT.
     * |        |          |010 = Clock source from PCLK3.
     * |        |          |011 = Clock source from external clock TM2 pin.
     * |        |          |100 = Clock source from LIRC.
     * |        |          |101 = Clock source from HIRC.
     * |        |          |110 = Clock source from HIRC48M/4.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[14:12] |TMR3SEL   |Timer 3 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of TMR3 is defined by TMR3SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from LXT.
     * |        |          |010 = Clock source from PCLK3.
     * |        |          |011 = Clock source from external clock TM3 pin.
     * |        |          |100 = Clock source from LIRC.
     * |        |          |101 = Clock source from HIRC.
     * |        |          |110 = Clock source from HIRC48M/4.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * @var CLK_T::TTMRSEL
     * Offset: 0x45C  Tick Timer Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |TTMR0SEL  |Tick Timer 0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of TTMR0 is defined by TTMR0SEL.
     * |        |          |000 = Clock source from PCLK4.
     * |        |          |001 = Clock source from LXT.
     * |        |          |010 = Clock source from LIRC.
     * |        |          |011 = Clock source from MIRC.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |[6:4]   |TTMR1SEL  |Tick Timer 1 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of TTMR1 is defined by TTMR1SEL.
     * |        |          |000 = Clock source from PCLK4.
     * |        |          |001 = Clock source from LXT.
     * |        |          |010 = Clock source from LIRC.
     * |        |          |011 = Clock source from MIRC.
     * |        |          |100 = Clock source from HIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * @var CLK_T::UARTSEL0
     * Offset: 0x460  UART Clock Source Select Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |UART0SEL  |UART 0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of UART0 is defined by UART0SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from HIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from APLL0/2.
     * |        |          |100 = Clock source from HIRC48M.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[6:4]   |UART1SEL  |UART 1 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of UART1 is defined by UART1SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from HIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from APLL0/2.
     * |        |          |100 = Clock source from HIRC48M.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[10:8]  |UART2SEL  |UART 2 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of UART2 is defined by UART2SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from HIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from APLL0/2.
     * |        |          |100 = Clock source from HIRC48M.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[14:12] |UART3SEL  |UART 3 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of UART3 is defined by UART3SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from HIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from APLL0/2.
     * |        |          |100 = Clock source from HIRC48M.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[18:16] |UART4SEL  |UART 4 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of UART4 is defined by UART4SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from HIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from APLL0/2.
     * |        |          |100 = Clock source from HIRC48M.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[22:20] |UART5SEL  |UART 5 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of UART5 is defined by UART5SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from HIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from APLL0/2.
     * |        |          |100 = Clock source from HIRC48M.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[26:24] |UART6SEL  |UART 6 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of UART6 is defined by UART6SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from HIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from APLL0/2.
     * |        |          |100 = Clock source from HIRC48M.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[30:28] |UART7SEL  |UART 7 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of UART7 is defined by UART7SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from HIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from APLL0/2.
     * |        |          |100 = Clock source from HIRC48M.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * @var CLK_T::UARTSEL1
     * Offset: 0x464  UART Clock Source Select Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |UART8SEL  |UART 8 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of UART8 is defined by UART8SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from HIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from APLL0/2.
     * |        |          |100 = Clock source from HIRC48M.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[6:4]   |UART9SEL  |UART 9 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of UART9 is defined by UART9SEL.
     * |        |          |000 = Clock source from HXT.
     * |        |          |001 = Clock source from HIRC.
     * |        |          |010 = Clock source from LXT.
     * |        |          |011 = Clock source from APLL0/2.
     * |        |          |100 = Clock source from HIRC48M.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * @var CLK_T::USBSEL
     * Offset: 0x468  USB Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |USBSEL    |USB Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of USBH, USBD and OTG is defined by USBSEL.
     * |        |          |0 = Clock source from HIRC48M.
     * |        |          |1 = Clock source from APLL1/2.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * @var CLK_T::WDTSEL
     * Offset: 0x46C  WDT Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WDT0SEL   |WDT0 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of WDT0 is defined by WDT0SEL.
     * |        |          |0 = Clock source from LXT.
     * |        |          |1 = Clock source from LIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |[4]     |WDT1SEL   |WDT1 Clock Source Selection (Write Protect)
     * |        |          |The peripheral clock source of WDT1 is defined by WDT1SEL.
     * |        |          |0 = Clock source from LXT.
     * |        |          |1 = Clock source from LIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * @var CLK_T::DLLSEL
     * Offset: 0x474  DLL Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DLL0SEL   |DLL0 Clock Source Selection (Write Protect)
     * |        |          |0 = Clock source from APLL0.
     * |        |          |1 = Clock source from APLL1.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var CLK_T::WWDTSEL
     * Offset: 0x47C  WWDT Clock Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDT0SEL  |WWDT0 Clock Source Selection (Write Protect)
     * |        |          |0 = Clock source from LIRC.
     * |        |          |1 = Clock source from LXT.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[4]     |WWDT1SEL  |WWDT1 Clock Source Selection (Write Protect)
     * |        |          |0 = Clock source from LIRC.
     * |        |          |1 = Clock source from LXT.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var CLK_T::SCLKDIV
     * Offset: 0x500  SCLK Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |SCLKDIV   |SCLK Clock Divide Number from SCLK Clock Source
     * |        |          |SCLK clock frequency = (SCLK clock source frequency) / (SCLKDIV + 1).
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var CLK_T::HCLKDIV
     * Offset: 0x508  HCLK Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:8]  |HCLK2DIV  |HCLK2 Clock Divide Number from HCLK2 Clock Source
     * |        |          |HCLK2 clock frequency = (HCLK2 clock source frequency) / (HCLK2DIV + 1).
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var CLK_T::PCLKDIV
     * Offset: 0x50C  PCLK Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PCLK0DIV  |PCLK0 Clock Divide Number from PCLK0 Clock Source
     * |        |          |PCLK0 clock frequency = (PCLK0 clock source frequency) / (PCLK0DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * |[7:4]   |PCLK1DIV  |PCLK1 Clock Divide Number from PCLK1 Clock Source
     * |        |          |PCLK1 clock frequency = (PCLK1 clock source frequency) / (PCLK1DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * |[11:8]  |PCLK2DIV  |PCLK2 Clock Divide Number from PCLK2 Clock Source
     * |        |          |PCLK2 clock frequency = (PCLK2 clock source frequency) / (PCLK2DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * |[15:12] |PCLK3DIV  |PCLK3 Clock Divide Number from PCLK3 Clock Source
     * |        |          |PCLK3 clock frequency = (PCLK3 clock source frequency) / (PCLK3DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * |[19:16] |PCLK4DIV  |PCLK4 Clock Divide Number from PCLK4 Clock Source
     * |        |          |PCLK4 clock frequency = (PCLK4 clock source frequency) / (PCLK4DIV + 1).
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var CLK_T::CANFDDIV
     * Offset: 0x510  CANFD Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CANFD0DIV |CANFD0 Clock Divide Number from CANFD0 Clock Source
     * |        |          |CANFD0 clock frequency = (CANFD0 clock source frequency) / (CANFD0DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * |[15:8]  |CANFD1DIV |CANFD1 Clock Divide Number from CANFD1 Clock Source
     * |        |          |CANFD1 clock frequency = (CANFD1 clock source frequency) / (CANFD1DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * @var CLK_T::DMICDIV
     * Offset: 0x514  DMIC Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DMIC0DIV  |DMIC0 Clock Divide Number from DMIC0 Clock Source
     * |        |          |DMIC0 clock frequency = (DMIC0DIV clock source frequency) / (DMIC0DIV + 1).
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var CLK_T::EADCDIV
     * Offset: 0x518  EADC Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |EADC0DIV  |EADC0 Clock Divide Number from EADC0 Clock Source
     * |        |          |EADC0 clock frequency = (EADC0 clock source frequency) / (EADC0DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * @var CLK_T::I2SDIV
     * Offset: 0x51C  I2S Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |I2S0DIV   |I2S0 Clock Divide Number from I2S0 Clock Source
     * |        |          |I2S0 clock frequency = (I2S0 clock source frequency) / (I2S0DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * |[15:8]  |I2S1DIV   |I2S1 Clock Divide Number from I2S1 Clock Source
     * |        |          |I2S1 clock frequency = (I2S1 clock source frequency) / (I2S1DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * @var CLK_T::KPIDIV
     * Offset: 0x520  KPI Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |KPI0DIV   |KPI0 Clock Divide Number from KPI0 Clock Source
     * |        |          |KPI0 clock frequency = (KPI0 clock source frequency) / (KPI0DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * @var CLK_T::LPADCDIV
     * Offset: 0x524  Low Power ADC Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |LPADC0DIV |LPADC0 Clock Divide Number from LPADC0 Clock Source
     * |        |          |LPADC0 clock frequency = (LPADC0 clock source frequency) / (LPADC0DIV + 1).
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var CLK_T::LPUARTDIV
     * Offset: 0x528  Low Power UART Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |LPUART0DIV|LPUART0 Clock Divide Number from LPUART0 Clock Source
     * |        |          |LPUART0 clock frequency = (LPUART0 clock source frequency) / (LPUART0DIV + 1).
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var CLK_T::PSIODIV
     * Offset: 0x52C  PSIO Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PSIO0DIV  |PSIO0 Clock Divide Number from PSIO0 Clock Source
     * |        |          |PSIO0 clock frequency = (PSIO0 clock source frequency) / (PSIO0DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * @var CLK_T::SCDIV
     * Offset: 0x530  SC Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SC0DIV    |SC0 Clock Divide Number from SC0 Clock Source
     * |        |          |SC0 clock frequency = (SC0 clock source frequency) / (SC0DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * |[15:8]  |SC1DIV    |SC1 Clock Divide Number from SC1 Clock Source
     * |        |          |SC1 clock frequency = (SC1 clock source frequency) / (SC1DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * |[23:16] |SC2DIV    |SC2 Clock Divide Number from SC2 Clock Source
     * |        |          |SC2 clock frequency = (SC2 clock source frequency) / (SC2DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * @var CLK_T::SDHDIV
     * Offset: 0x534  SDH Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |SDH0DIV   |SDH0 Clock Divide Number from SDH0 Clock Source
     * |        |          |SDH0 clock frequency = (SDH0 clock source frequency) / (SDH0DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * |[15:8]  |SDH1DIV   |SDH1 Clock Divide Number from SDH1 Clock Source
     * |        |          |SDH1 clock frequency = (SDH1 clock source frequency) / (SDH1DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * @var CLK_T::STDIV
     * Offset: 0x538  System Tick Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |ST0DIV    |System Tick 0 Clock Divide Number from System Tick 0 Clock Source
     * |        |          |ST0 clock frequency = (ST0 clock source frequency) / (ST0DIV + 1).
     * |        |          |Note: These bits are not retained when D0 power is turned off.
     * @var CLK_T::UARTDIV0
     * Offset: 0x53C  UART Clock Divider Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |UART0DIV  |UART0 Clock Divide Number from UART0 Clock Source
     * |        |          |UART0 clock frequency = (UART0 clock source frequency) / (UART0DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * |[7:4]   |UART1DIV  |UART1 Clock Divide Number from UART1 Clock Source
     * |        |          |UART1 clock frequency = (UART1 clock source frequency) / (UART1DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * |[11:8]  |UART2DIV  |UART2 Clock Divide Number from UART2 Clock Source
     * |        |          |UART2 clock frequency = (UART2 clock source frequency) / (UART2DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * |[15:12] |UART3DIV  |UART3 Clock Divide Number from UART3 Clock Source
     * |        |          |UART3 clock frequency = (UART3 clock source frequency) / (UART3DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * |[19:16] |UART4DIV  |UART4 Clock Divide Number from UART4 Clock Source
     * |        |          |UART4 clock frequency = (UART4 clock source frequency) / (UART4DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * |[23:20] |UART5DIV  |UART5 Clock Divide Number from UART5 Clock Source
     * |        |          |UART5 clock frequency = (UART5 clock source frequency) / (UART5DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * |[27:24] |UART6DIV  |UART6 Clock Divide Number from UART6 Clock Source
     * |        |          |UART6 clock frequency = (UART6 clock source frequency) / (UART6DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * |[31:28] |UART7DIV  |UART7 Clock Divide Number from UART7 Clock Source
     * |        |          |UART7 clock frequency = (UART7 clock source frequency) / (UART7DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * @var CLK_T::UARTDIV1
     * Offset: 0x540  UART Clock Divider Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |UART8DIV  |UART8 Clock Divide Number from UART8 Clock Source
     * |        |          |UART8 clock frequency = (UART8 clock source frequency) / (UART8DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * |[7:4]   |UART9DIV  |UART9 Clock Divide Number from UART9 Clock Source
     * |        |          |UART9 clock frequency = (UART9 clock source frequency) / (UART9DIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * @var CLK_T::USBDIV
     * Offset: 0x544  USB Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |USBDIV    |USB Clock Divide Number from USB Clock Source
     * |        |          |USB clock frequency = (USB clock source frequency) / (USBDIV + 1).
     * |        |          |Note: These bits are not retained when D1 power is turned off.
     * @var CLK_T::VSENSEDIV
     * Offset: 0x548  Video Pixel Clock Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |VSENSEDIV |Video Pixel Clock Divide Number from Video Pixel Clock Source
     * |        |          |VSENSE clock frequency = (VSENSE clock source frequency) / (VSENSEDIV + 1).
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     */
    __IO uint32_t SRCCTL;                /*!< [0x0000] Clock Source Control Register                                    */
    __I  uint32_t STATUS;                /*!< [0x0004] Clock Status Monitor Register                                    */
    __IO uint32_t MIRCCTL;               /*!< [0x0008] MIRC Control Register                                            */
    __IO uint32_t HIRCCTL;               /*!< [0x000c] HIRC Control Register                                            */
    __IO uint32_t HXTCTL;                /*!< [0x0010] HXT Control Register                                             */
    __IO uint32_t HIRC48MCTL;            /*!< [0x0014] HIRC48M Control Register                                         */
    __IO uint32_t APLL0CTL;              /*!< [0x0018] APLL0 Control Register                                           */
    __IO uint32_t APLL0SEL;              /*!< [0x001c] APLL0 Clock Source Select Control Register                       */
    __IO uint32_t APLL1CTL;              /*!< [0x0020] APLL1 Control Register                                           */
    __IO uint32_t APLL1SEL;              /*!< [0x0024] APLL1 Clock Source Select Control Register                       */
    __IO uint32_t CLKOCTL;               /*!< [0x0028] Clock Output Control Register                                    */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t CLKDCTL;               /*!< [0x0030] Clock Fail Detector Control Register                             */
    __IO uint32_t CLKDSTS;               /*!< [0x0034] Clock Fail Detector Status Register                              */
    __IO uint32_t CDUPB;                 /*!< [0x0038] Clock Frequency Range Detector Upper Boundary Register           */
    __IO uint32_t CDLOWB;                /*!< [0x003c] Clock Frequency Range Detector Lower Boundary Register           */
    __IO uint32_t STOPREQ;               /*!< [0x0040] Clock Stop Request Register                                      */
    __I  uint32_t STOPACK;               /*!< [0x0044] Clock Stop Acknowledge Register                                  */
    __I  uint32_t RESERVE1[110];
    __IO uint32_t ACMPCTL;               /*!< [0x0200] ACMP Clock Enable Control Register                               */
    __IO uint32_t AWFCTL;                /*!< [0x0204] AWF Clock Enable Control Register                                */
    __IO uint32_t BPWMCTL;               /*!< [0x0208] BPWM Clock Enable Control Register                               */
    __IO uint32_t CANFDCTL;              /*!< [0x020c] CANFD Clock Enable Control Register                              */
    __IO uint32_t CCAPCTL;               /*!< [0x0210] CCAP Clock Enable Control Register                               */
    __IO uint32_t CRCCTL;                /*!< [0x0214] CRC Clock Enable Control Register                                */
    __IO uint32_t CRYPTOCTL;             /*!< [0x0218] CRYPTO Clock Enable Control Register                             */
    __IO uint32_t DACCTL;                /*!< [0x021c] DAC Clock Enable Control Register                                */
    __IO uint32_t DMICCTL;               /*!< [0x0220] DMIC Clock Enable Control Register                               */
    __IO uint32_t EADCCTL;               /*!< [0x0224] EADC Clock Enable Control Register                               */
    __IO uint32_t EBICTL;                /*!< [0x0228] EBI Clock Enable Control Register                                */
    __IO uint32_t ECAPCTL;               /*!< [0x022c] ECAP Clock Enable Control Register                               */
    __IO uint32_t EMACCTL;               /*!< [0x0230] EMAC Clock Enable Control Register                               */
    __IO uint32_t EPWMCTL;               /*!< [0x0234] EPWM Clock Enable Control Register                               */
    __IO uint32_t EQEICTL;               /*!< [0x0238] EQEI Clock Enable Control Register                               */
    __IO uint32_t FMCCTL;                /*!< [0x023c] FMC Clock Enable Control Register                                */
    __IO uint32_t GDMACTL;               /*!< [0x0240] GDMA Clock Enable Control Register                               */
    __IO uint32_t GPIOCTL;               /*!< [0x0244] GPIO Clock Enable Control Register                               */
    __IO uint32_t HSOTGCTL;              /*!< [0x0248] HSOTG Clock Enable Control Register                              */
    __IO uint32_t HSUSBDCTL;             /*!< [0x024c] HSUSBD Clock Enable Control Register                             */
    __IO uint32_t HSUSBHCTL;             /*!< [0x0250] HSUSBH Clock Enable Control Register                             */
    __IO uint32_t I2CCTL;                /*!< [0x0254] I2C Clock Enable Control Register                                */
    __IO uint32_t I2SCTL;                /*!< [0x0258] I2S Clock Enable Control Register                                */
    __IO uint32_t I3CCTL;                /*!< [0x025c] I3C Clock Enable Control Register                                */
    __IO uint32_t KDFCTL;                /*!< [0x0260] KDF Clock Enable Control Register                                */
    __IO uint32_t KPICTL;                /*!< [0x0264] KPI Clock Enable Control Register                                */
    __IO uint32_t KSCTL;                 /*!< [0x0268] KS Clock Enable Control Register                                 */
    __IO uint32_t LPADCCTL;              /*!< [0x026c] Low Power ADC Clock Enable Control Register                      */
    __IO uint32_t LPPDMACTL;             /*!< [0x0270] Low Power PDMA Clock Enable Control Register                     */
    __IO uint32_t LPGPIOCTL;             /*!< [0x0274] Low Power GPIO Clock Enable Control Register                     */
    __IO uint32_t LPI2CCTL;              /*!< [0x0278] Low Power I2C Clock Enable Control Register                      */
    __IO uint32_t LPSPICTL;              /*!< [0x027c] Low Power SPI Clock Enable Control Register                      */
    __IO uint32_t LPSRAMCTL;             /*!< [0x0280] Low Power SRAM Clock Enable Control Register                     */
    __IO uint32_t LPTMRCTL;              /*!< [0x0284] Low Power Timer Clock Enable Control Register                    */
    __IO uint32_t LPUARTCTL;             /*!< [0x0288] Low Power UART Clock Enable Control Register                     */
    __IO uint32_t NPUCTL;                /*!< [0x028c] NPU Clock Enable Control Register                                */
    __I  uint32_t RESERVE2[1];
    __IO uint32_t OTFCCTL;               /*!< [0x0294] OTFC Clock Enable Control Register                               */
    __IO uint32_t OTGCTL;                /*!< [0x0298] OTG Clock Enable Control Register                                */
    __IO uint32_t PDMACTL;               /*!< [0x029c] PDMA Clock Enable Control Register                               */
    __IO uint32_t PSIOCTL;               /*!< [0x02a0] PSIO Clock Enable Control Register                               */
    __IO uint32_t QSPICTL;               /*!< [0x02a4] QSPI Clock Enable Control Register                               */
    __IO uint32_t RTCCTL;                /*!< [0x02a8] RTC Clock Enable Control Register                                */
    __IO uint32_t SCCTL;                 /*!< [0x02ac] SC Clock Enable Control Register                                 */
    __IO uint32_t SCUCTL;                /*!< [0x02b0] SCU Clock Enable Control Register                                */
    __IO uint32_t SDHCTL;                /*!< [0x02b4] SDH Clock Enable Control Register                                */
    __IO uint32_t SPICTL;                /*!< [0x02b8] SPI Clock Enable Control Register                                */
    __IO uint32_t SPIMCTL;               /*!< [0x02bc] SPIM Clock Enable Control Register                               */
    __IO uint32_t SRAMCTL;               /*!< [0x02c0] System SRAM Clock Enable Control Register                        */
    __I  uint32_t RESERVE3[2];
    __IO uint32_t STCTL;                 /*!< [0x02cc] System Tick Clock Enable Control Register                        */
    __I  uint32_t RESERVE4[1];
    __IO uint32_t TMRCTL;                /*!< [0x02d4] Timer Clock Enable Control Register                              */
    __IO uint32_t TRNGCTL;               /*!< [0x02d8] TRNG Clock Enable Control Register                               */
    __IO uint32_t TTMRCTL;               /*!< [0x02dc] Tick Timer Clock Enable Control Register                         */
    __IO uint32_t UARTCTL;               /*!< [0x02e0] UART Clock Enable Control Register                               */
    __IO uint32_t USBDCTL;               /*!< [0x02e4] USBD Clock Enable Control Register                               */
    __IO uint32_t USBHCTL;               /*!< [0x02e8] USBH Clock Enable Control Register                               */
    __IO uint32_t USCICTL;               /*!< [0x02ec] USCI Clock Enable Control Register                               */
    __IO uint32_t UTCPDCTL;              /*!< [0x02f0] UTCPD Clock Enable Control Register                              */
    __IO uint32_t WDTCTL;                /*!< [0x02f4] WDT Clock Enable Control Register                                */
    __IO uint32_t WWDTCTL;               /*!< [0x02f8] WWDT Clock Enable Control Register                               */
    __I  uint32_t RESERVE5[65];
    __IO uint32_t SCLKSEL;               /*!< [0x0400] System Clock Source Select Control Register                      */
    __IO uint32_t BPWMSEL;               /*!< [0x0404] BPWM Clock Source Select Control Register                        */
    __IO uint32_t CANFDSEL;              /*!< [0x0408] CANFD Clock Source Select Control Register                       */
    __IO uint32_t CCAPSEL;               /*!< [0x040c] CCAP Clock Source Select Control Register                        */
    __IO uint32_t CLKOSEL;               /*!< [0x0410] Clock Output Clock Source Select Control Register                */
    __IO uint32_t DMICSEL;               /*!< [0x0414] DMIC Clock Source Select Control Register                        */
    __IO uint32_t EADCSEL;               /*!< [0x0418] EADC Clock Source Select Control Register                        */
    __IO uint32_t EPWMSEL;               /*!< [0x041c] EPWM Clock Source Select Control Register                        */
    __IO uint32_t FMCSEL;                /*!< [0x0420] FMC Clock Source Select Control Register                         */
    __IO uint32_t I2SSEL;                /*!< [0x0424] I2S Clock Source Select Control Register                         */
    __IO uint32_t I3CSEL;                /*!< [0x0428] I3C Clock Source Select Control Register                         */
    __IO uint32_t KPISEL;                /*!< [0x042c] KPI Clock Source Select Control Register                         */
    __IO uint32_t LPADCSEL;              /*!< [0x0430] Low Power ADC Clock Source Select Control Register               */
    __IO uint32_t LPSPISEL;              /*!< [0x0434] Low Power SPI Clock Source Select Control Register               */
    __IO uint32_t LPTMRSEL;              /*!< [0x0438] Low Power Timer Clock Source Select Control Register             */
    __IO uint32_t LPUARTSEL;             /*!< [0x043c] Low Power UART Clock Source Select Control Register              */
    __IO uint32_t PSIOSEL;               /*!< [0x0440] PSIO Clock Source Select Control Register                        */
    __IO uint32_t QSPISEL;               /*!< [0x0444] QSPI Clock Source Select Control Register                        */
    __IO uint32_t SCSEL;                 /*!< [0x0448] SC Clock Source Select Control Register                          */
    __IO uint32_t SDHSEL;                /*!< [0x044c] SDH Clock Source Select Control Register                         */
    __IO uint32_t SPISEL;                /*!< [0x0450] SPI Clock Source Select Control Register                         */
    __IO uint32_t STSEL;                 /*!< [0x0454] System Tick Clock Source Select Control Register                 */
    __IO uint32_t TMRSEL;                /*!< [0x0458] Timer Clock Source Select Control Register                       */
    __IO uint32_t TTMRSEL;               /*!< [0x045c] Tick Timer Clock Source Select Control Register                  */
    __IO uint32_t UARTSEL0;              /*!< [0x0460] UART Clock Source Select Control Register 0                      */
    __IO uint32_t UARTSEL1;              /*!< [0x0464] UART Clock Source Select Control Register 1                      */
    __IO uint32_t USBSEL;                /*!< [0x0468] USB Clock Source Select Control Register                         */
    __IO uint32_t WDTSEL;                /*!< [0x046c] WDT Clock Source Select Control Register                         */
    __I  uint32_t RESERVE6[1];
    __IO uint32_t DLLSEL;                /*!< [0x0474] DLL Clock Source Select Control Register                         */
    __I  uint32_t RESERVE7[1];
    __IO uint32_t WWDTSEL;               /*!< [0x047c] WWDT Clock Source Select Control Register                        */
    __I  uint32_t RESERVE8[32];
    __IO uint32_t SCLKDIV;               /*!< [0x0500] SCLK Clock Divider Control Register                              */
    __I  uint32_t RESERVE9[1];
    __IO uint32_t HCLKDIV;               /*!< [0x0508] HCLK Clock Divider Control Register                              */
    __IO uint32_t PCLKDIV;               /*!< [0x050c] PCLK Clock Divider Control Register                              */
    __IO uint32_t CANFDDIV;              /*!< [0x0510] CANFD Clock Divider Control Register                             */
    __IO uint32_t DMICDIV;               /*!< [0x0514] DMIC Clock Divider Control Register                              */
    __IO uint32_t EADCDIV;               /*!< [0x0518] EADC Clock Divider Control Register                              */
    __IO uint32_t I2SDIV;                /*!< [0x051c] I2S Clock Divider Control Register                               */
    __IO uint32_t KPIDIV;                /*!< [0x0520] KPI Clock Divider Control Register                               */
    __IO uint32_t LPADCDIV;              /*!< [0x0524] Low Power ADC Clock Divider Control Register                     */
    __IO uint32_t LPUARTDIV;             /*!< [0x0528] Low Power UART Clock Divider Control Register                    */
    __IO uint32_t PSIODIV;               /*!< [0x052c] PSIO Clock Divider Control Register                              */
    __IO uint32_t SCDIV;                 /*!< [0x0530] SC Clock Divider Control Register                                */
    __IO uint32_t SDHDIV;                /*!< [0x0534] SDH Clock Divider Control Register                               */
    __IO uint32_t STDIV;                 /*!< [0x0538] System Tick Clock Divider Control Register                       */
    __IO uint32_t UARTDIV0;              /*!< [0x053c] UART Clock Divider Control Register 0                            */
    __IO uint32_t UARTDIV1;              /*!< [0x0540] UART Clock Divider Control Register 1                            */
    __IO uint32_t USBDIV;                /*!< [0x0544] USB Clock Divider Control Register                               */
    __IO uint32_t VSENSEDIV;             /*!< [0x0548] Video Pixel Clock Divider Control Register                       */

} CLK_T;

/**
    @addtogroup CLK_CONST CLK Bit Field Definition
    Constant Definitions for CLK Controller
@{ */

#define CLK_SRCCTL_LIRCEN_Pos            (0)                                               /*!< CLK_T::SRCCTL: LIRCEN Position         */
#define CLK_SRCCTL_LIRCEN_Msk            (0x1ul << CLK_SRCCTL_LIRCEN_Pos)                  /*!< CLK_T::SRCCTL: LIRCEN Mask             */

#define CLK_SRCCTL_LXTEN_Pos             (1)                                               /*!< CLK_T::SRCCTL: LXTEN Position          */
#define CLK_SRCCTL_LXTEN_Msk             (0x1ul << CLK_SRCCTL_LXTEN_Pos)                   /*!< CLK_T::SRCCTL: LXTEN Mask              */

#define CLK_SRCCTL_MIRCEN_Pos            (2)                                               /*!< CLK_T::SRCCTL: MIRCEN Position         */
#define CLK_SRCCTL_MIRCEN_Msk            (0x1ul << CLK_SRCCTL_MIRCEN_Pos)                  /*!< CLK_T::SRCCTL: MIRCEN Mask             */

#define CLK_SRCCTL_HIRCEN_Pos            (3)                                               /*!< CLK_T::SRCCTL: HIRCEN Position         */
#define CLK_SRCCTL_HIRCEN_Msk            (0x1ul << CLK_SRCCTL_HIRCEN_Pos)                  /*!< CLK_T::SRCCTL: HIRCEN Mask             */

#define CLK_SRCCTL_HXTEN_Pos             (4)                                               /*!< CLK_T::SRCCTL: HXTEN Position          */
#define CLK_SRCCTL_HXTEN_Msk             (0x1ul << CLK_SRCCTL_HXTEN_Pos)                   /*!< CLK_T::SRCCTL: HXTEN Mask              */

#define CLK_SRCCTL_HIRC48MEN_Pos         (5)                                               /*!< CLK_T::SRCCTL: HIRC48MEN Position      */
#define CLK_SRCCTL_HIRC48MEN_Msk         (0x1ul << CLK_SRCCTL_HIRC48MEN_Pos)               /*!< CLK_T::SRCCTL: HIRC48MEN Mask          */

#define CLK_SRCCTL_APLL0EN_Pos           (6)                                               /*!< CLK_T::SRCCTL: APLL0EN Position        */
#define CLK_SRCCTL_APLL0EN_Msk           (0x1ul << CLK_SRCCTL_APLL0EN_Pos)                 /*!< CLK_T::SRCCTL: APLL0EN Mask            */

#define CLK_SRCCTL_APLL1EN_Pos           (7)                                               /*!< CLK_T::SRCCTL: APLL1EN Position        */
#define CLK_SRCCTL_APLL1EN_Msk           (0x1ul << CLK_SRCCTL_APLL1EN_Pos)                 /*!< CLK_T::SRCCTL: APLL1EN Mask            */

#define CLK_STATUS_LIRCSTB_Pos           (0)                                               /*!< CLK_T::STATUS: LIRCSTB Position        */
#define CLK_STATUS_LIRCSTB_Msk           (0x1ul << CLK_STATUS_LIRCSTB_Pos)                 /*!< CLK_T::STATUS: LIRCSTB Mask            */

#define CLK_STATUS_LXTSTB_Pos            (1)                                               /*!< CLK_T::STATUS: LXTSTB Position         */
#define CLK_STATUS_LXTSTB_Msk            (0x1ul << CLK_STATUS_LXTSTB_Pos)                  /*!< CLK_T::STATUS: LXTSTB Mask             */

#define CLK_STATUS_MIRCSTB_Pos           (2)                                               /*!< CLK_T::STATUS: MIRCSTB Position        */
#define CLK_STATUS_MIRCSTB_Msk           (0x1ul << CLK_STATUS_MIRCSTB_Pos)                 /*!< CLK_T::STATUS: MIRCSTB Mask            */

#define CLK_STATUS_HIRCSTB_Pos           (3)                                               /*!< CLK_T::STATUS: HIRCSTB Position        */
#define CLK_STATUS_HIRCSTB_Msk           (0x1ul << CLK_STATUS_HIRCSTB_Pos)                 /*!< CLK_T::STATUS: HIRCSTB Mask            */

#define CLK_STATUS_HXTSTB_Pos            (4)                                               /*!< CLK_T::STATUS: HXTSTB Position         */
#define CLK_STATUS_HXTSTB_Msk            (0x1ul << CLK_STATUS_HXTSTB_Pos)                  /*!< CLK_T::STATUS: HXTSTB Mask             */

#define CLK_STATUS_HIRC48MSTB_Pos        (5)                                               /*!< CLK_T::STATUS: HIRC48MSTB Position     */
#define CLK_STATUS_HIRC48MSTB_Msk        (0x1ul << CLK_STATUS_HIRC48MSTB_Pos)              /*!< CLK_T::STATUS: HIRC48MSTB Mask         */

#define CLK_STATUS_APLL0STB_Pos          (6)                                               /*!< CLK_T::STATUS: APLL0STB Position       */
#define CLK_STATUS_APLL0STB_Msk          (0x1ul << CLK_STATUS_APLL0STB_Pos)                /*!< CLK_T::STATUS: APLL0STB Mask           */

#define CLK_STATUS_APLL1STB_Pos          (7)                                               /*!< CLK_T::STATUS: APLL1STB Position       */
#define CLK_STATUS_APLL1STB_Msk          (0x1ul << CLK_STATUS_APLL1STB_Pos)                /*!< CLK_T::STATUS: APLL1STB Mask           */

#define CLK_STATUS_SCLKSWF_Pos           (8)                                               /*!< CLK_T::STATUS: SCLKSWF Position        */
#define CLK_STATUS_SCLKSWF_Msk           (0x1ul << CLK_STATUS_SCLKSWF_Pos)                 /*!< CLK_T::STATUS: SCLKSWF Mask            */

#define CLK_MIRCCTL_MIRC1MSTBS_Pos       (0)                                               /*!< CLK_T::MIRCCTL: MIRC1MSTBS Position    */
#define CLK_MIRCCTL_MIRC1MSTBS_Msk       (0x3ul << CLK_MIRCCTL_MIRC1MSTBS_Pos)             /*!< CLK_T::MIRCCTL: MIRC1MSTBS Mask        */

#define CLK_MIRCCTL_MIRC2MSTBS_Pos       (4)                                               /*!< CLK_T::MIRCCTL: MIRC2MSTBS Position    */
#define CLK_MIRCCTL_MIRC2MSTBS_Msk       (0x3ul << CLK_MIRCCTL_MIRC2MSTBS_Pos)             /*!< CLK_T::MIRCCTL: MIRC2MSTBS Mask        */

#define CLK_MIRCCTL_MIRC4MSTBS_Pos       (8)                                               /*!< CLK_T::MIRCCTL: MIRC4MSTBS Position    */
#define CLK_MIRCCTL_MIRC4MSTBS_Msk       (0x3ul << CLK_MIRCCTL_MIRC4MSTBS_Pos)             /*!< CLK_T::MIRCCTL: MIRC4MSTBS Mask        */

#define CLK_MIRCCTL_MIRC8MSTBS_Pos       (12)                                              /*!< CLK_T::MIRCCTL: MIRC8MSTBS Position    */
#define CLK_MIRCCTL_MIRC8MSTBS_Msk       (0x3ul << CLK_MIRCCTL_MIRC8MSTBS_Pos)             /*!< CLK_T::MIRCCTL: MIRC8MSTBS Mask        */

#define CLK_MIRCCTL_MIRCFSEL_Pos         (16)                                              /*!< CLK_T::MIRCCTL: MIRCFSEL Position      */
#define CLK_MIRCCTL_MIRCFSEL_Msk         (0x3ul << CLK_MIRCCTL_MIRCFSEL_Pos)               /*!< CLK_T::MIRCCTL: MIRCFSEL Mask          */

#define CLK_MIRCCTL_MIRCFDIS_Pos         (20)                                              /*!< CLK_T::MIRCCTL: MIRCFDIS Position      */
#define CLK_MIRCCTL_MIRCFDIS_Msk         (0x1ul << CLK_MIRCCTL_MIRCFDIS_Pos)               /*!< CLK_T::MIRCCTL: MIRCFDIS Mask          */

#define CLK_HIRCCTL_HIRCSTBS_Pos         (0)                                               /*!< CLK_T::HIRCCTL: HIRCSTBS Position      */
#define CLK_HIRCCTL_HIRCSTBS_Msk         (0x3ul << CLK_HIRCCTL_HIRCSTBS_Pos)               /*!< CLK_T::HIRCCTL: HIRCSTBS Mask          */

#define CLK_HIRCCTL_HIRCFDIS_Pos         (4)                                               /*!< CLK_T::HIRCCTL: HIRCFDIS Position      */
#define CLK_HIRCCTL_HIRCFDIS_Msk         (0x1ul << CLK_HIRCCTL_HIRCFDIS_Pos)               /*!< CLK_T::HIRCCTL: HIRCFDIS Mask          */

#define CLK_HXTCTL_HXTGAIN_Pos           (0)                                               /*!< CLK_T::HXTCTL: HXTGAIN Position        */
#define CLK_HXTCTL_HXTGAIN_Msk           (0x7ul << CLK_HXTCTL_HXTGAIN_Pos)                 /*!< CLK_T::HXTCTL: HXTGAIN Mask            */

#define CLK_HXTCTL_HXTSELTYP_Pos         (4)                                               /*!< CLK_T::HXTCTL: HXTSELTYP Position      */
#define CLK_HXTCTL_HXTSELTYP_Msk         (0x1ul << CLK_HXTCTL_HXTSELTYP_Pos)               /*!< CLK_T::HXTCTL: HXTSELTYP Mask          */

#define CLK_HXTCTL_HXTMD_Pos             (5)                                               /*!< CLK_T::HXTCTL: HXTMD Position          */
#define CLK_HXTCTL_HXTMD_Msk             (0x1ul << CLK_HXTCTL_HXTMD_Pos)                   /*!< CLK_T::HXTCTL: HXTMD Mask              */

#define CLK_HXTCTL_HXTFDIS_Pos           (6)                                               /*!< CLK_T::HXTCTL: HXTFDIS Position        */
#define CLK_HXTCTL_HXTFDIS_Msk           (0x1ul << CLK_HXTCTL_HXTFDIS_Pos)                 /*!< CLK_T::HXTCTL: HXTFDIS Mask            */

#define CLK_HXTCTL_HXTFSEL_Pos           (7)                                               /*!< CLK_T::HXTCTL: HXTFSEL Position        */
#define CLK_HXTCTL_HXTFSEL_Msk           (0x1ul << CLK_HXTCTL_HXTFSEL_Pos)                 /*!< CLK_T::HXTCTL: HXTFSEL Mask            */

#define CLK_HIRC48MCTL_HIRC48MSTBS_Pos   (0)                                               /*!< CLK_T::HIRC48MCTL: HIRC48MSTBS Position*/
#define CLK_HIRC48MCTL_HIRC48MSTBS_Msk   (0x3ul << CLK_HIRC48MCTL_HIRC48MSTBS_Pos)         /*!< CLK_T::HIRC48MCTL: HIRC48MSTBS Mask    */

#define CLK_HIRC48MCTL_HIRC48MFDIS_Pos   (4)                                               /*!< CLK_T::HIRC48MCTL: HIRC48MFDIS Position*/
#define CLK_HIRC48MCTL_HIRC48MFDIS_Msk   (0x1ul << CLK_HIRC48MCTL_HIRC48MFDIS_Pos)         /*!< CLK_T::HIRC48MCTL: HIRC48MFDIS Mask    */

#define CLK_APLL0CTL_FBDIV_Pos           (0)                                               /*!< CLK_T::APLL0CTL: FBDIV Position        */
#define CLK_APLL0CTL_FBDIV_Msk           (0x1fful << CLK_APLL0CTL_FBDIV_Pos)               /*!< CLK_T::APLL0CTL: FBDIV Mask            */

#define CLK_APLL0CTL_INDIV_Pos           (9)                                               /*!< CLK_T::APLL0CTL: INDIV Position        */
#define CLK_APLL0CTL_INDIV_Msk           (0x1ful << CLK_APLL0CTL_INDIV_Pos)                /*!< CLK_T::APLL0CTL: INDIV Mask            */

#define CLK_APLL0CTL_OUTDIV_Pos          (14)                                              /*!< CLK_T::APLL0CTL: OUTDIV Position       */
#define CLK_APLL0CTL_OUTDIV_Msk          (0x3ul << CLK_APLL0CTL_OUTDIV_Pos)                /*!< CLK_T::APLL0CTL: OUTDIV Mask           */

#define CLK_APLL0CTL_FRDIV_Pos           (16)                                              /*!< CLK_T::APLL0CTL: FRDIV Position        */
#define CLK_APLL0CTL_FRDIV_Msk           (0xffful << CLK_APLL0CTL_FRDIV_Pos)               /*!< CLK_T::APLL0CTL: FRDIV Mask            */

#define CLK_APLL0CTL_STBSEL_Pos          (28)                                              /*!< CLK_T::APLL0CTL: STBSEL Position       */
#define CLK_APLL0CTL_STBSEL_Msk          (0x3ul << CLK_APLL0CTL_STBSEL_Pos)                /*!< CLK_T::APLL0CTL: STBSEL Mask           */

#define CLK_APLL0CTL_BP_Pos              (30)                                              /*!< CLK_T::APLL0CTL: BP Position           */
#define CLK_APLL0CTL_BP_Msk              (0x1ul << CLK_APLL0CTL_BP_Pos)                    /*!< CLK_T::APLL0CTL: BP Mask               */

#define CLK_APLL0CTL_PLLFEN_Pos          (31)                                              /*!< CLK_T::APLL0CTL: PLLFEN Position       */
#define CLK_APLL0CTL_PLLFEN_Msk          (0x1ul << CLK_APLL0CTL_PLLFEN_Pos)                /*!< CLK_T::APLL0CTL: PLLFEN Mask           */

#define CLK_APLL0SEL_APLLSRC_Pos         (0)                                               /*!< CLK_T::APLL0SEL: APLLSRC Position      */
#define CLK_APLL0SEL_APLLSRC_Msk         (0x3ul << CLK_APLL0SEL_APLLSRC_Pos)               /*!< CLK_T::APLL0SEL: APLLSRC Mask          */

#define CLK_APLL1CTL_FBDIV_Pos           (0)                                               /*!< CLK_T::APLL1CTL: FBDIV Position        */
#define CLK_APLL1CTL_FBDIV_Msk           (0x1fful << CLK_APLL1CTL_FBDIV_Pos)               /*!< CLK_T::APLL1CTL: FBDIV Mask            */

#define CLK_APLL1CTL_INDIV_Pos           (9)                                               /*!< CLK_T::APLL1CTL: INDIV Position        */
#define CLK_APLL1CTL_INDIV_Msk           (0x1ful << CLK_APLL1CTL_INDIV_Pos)                /*!< CLK_T::APLL1CTL: INDIV Mask            */

#define CLK_APLL1CTL_OUTDIV_Pos          (14)                                              /*!< CLK_T::APLL1CTL: OUTDIV Position       */
#define CLK_APLL1CTL_OUTDIV_Msk          (0x3ul << CLK_APLL1CTL_OUTDIV_Pos)                /*!< CLK_T::APLL1CTL: OUTDIV Mask           */

#define CLK_APLL1CTL_FRDIV_Pos           (16)                                              /*!< CLK_T::APLL1CTL: FRDIV Position        */
#define CLK_APLL1CTL_FRDIV_Msk           (0xffful << CLK_APLL1CTL_FRDIV_Pos)               /*!< CLK_T::APLL1CTL: FRDIV Mask            */

#define CLK_APLL1CTL_STBSEL_Pos          (28)                                              /*!< CLK_T::APLL1CTL: STBSEL Position       */
#define CLK_APLL1CTL_STBSEL_Msk          (0x3ul << CLK_APLL1CTL_STBSEL_Pos)                /*!< CLK_T::APLL1CTL: STBSEL Mask           */

#define CLK_APLL1CTL_BP_Pos              (30)                                              /*!< CLK_T::APLL1CTL: BP Position           */
#define CLK_APLL1CTL_BP_Msk              (0x1ul << CLK_APLL1CTL_BP_Pos)                    /*!< CLK_T::APLL1CTL: BP Mask               */

#define CLK_APLL1CTL_PLLFEN_Pos          (31)                                              /*!< CLK_T::APLL1CTL: PLLFEN Position       */
#define CLK_APLL1CTL_PLLFEN_Msk          (0x1ul << CLK_APLL1CTL_PLLFEN_Pos)                /*!< CLK_T::APLL1CTL: PLLFEN Mask           */

#define CLK_APLL1SEL_APLLSRC_Pos         (0)                                               /*!< CLK_T::APLL1SEL: APLLSRC Position      */
#define CLK_APLL1SEL_APLLSRC_Msk         (0x3ul << CLK_APLL1SEL_APLLSRC_Pos)               /*!< CLK_T::APLL1SEL: APLLSRC Mask          */

#define CLK_CLKOCTL_FREQSEL_Pos          (0)                                               /*!< CLK_T::CLKOCTL: FREQSEL Position       */
#define CLK_CLKOCTL_FREQSEL_Msk          (0xful << CLK_CLKOCTL_FREQSEL_Pos)                /*!< CLK_T::CLKOCTL: FREQSEL Mask           */

#define CLK_CLKOCTL_CLKOEN_Pos           (4)                                               /*!< CLK_T::CLKOCTL: CLKOEN Position        */
#define CLK_CLKOCTL_CLKOEN_Msk           (0x1ul << CLK_CLKOCTL_CLKOEN_Pos)                 /*!< CLK_T::CLKOCTL: CLKOEN Mask            */

#define CLK_CLKOCTL_DIV1EN_Pos           (5)                                               /*!< CLK_T::CLKOCTL: DIV1EN Position        */
#define CLK_CLKOCTL_DIV1EN_Msk           (0x1ul << CLK_CLKOCTL_DIV1EN_Pos)                 /*!< CLK_T::CLKOCTL: DIV1EN Mask            */

#define CLK_CLKOCTL_CLK1HZEN_Pos         (6)                                               /*!< CLK_T::CLKOCTL: CLK1HZEN Position      */
#define CLK_CLKOCTL_CLK1HZEN_Msk         (0x1ul << CLK_CLKOCTL_CLK1HZEN_Pos)               /*!< CLK_T::CLKOCTL: CLK1HZEN Mask          */

#define CLK_CLKDCTL_HXTFDEN_Pos          (4)                                               /*!< CLK_T::CLKDCTL: HXTFDEN Position       */
#define CLK_CLKDCTL_HXTFDEN_Msk          (0x1ul << CLK_CLKDCTL_HXTFDEN_Pos)                /*!< CLK_T::CLKDCTL: HXTFDEN Mask           */

#define CLK_CLKDCTL_HXTFIEN_Pos          (5)                                               /*!< CLK_T::CLKDCTL: HXTFIEN Position       */
#define CLK_CLKDCTL_HXTFIEN_Msk          (0x1ul << CLK_CLKDCTL_HXTFIEN_Pos)                /*!< CLK_T::CLKDCTL: HXTFIEN Mask           */

#define CLK_CLKDCTL_HXTFDSEL_Pos         (6)                                               /*!< CLK_T::CLKDCTL: HXTFDSEL Position      */
#define CLK_CLKDCTL_HXTFDSEL_Msk         (0x1ul << CLK_CLKDCTL_HXTFDSEL_Pos)               /*!< CLK_T::CLKDCTL: HXTFDSEL Mask          */

#define CLK_CLKDCTL_LXTFDEN_Pos          (12)                                              /*!< CLK_T::CLKDCTL: LXTFDEN Position       */
#define CLK_CLKDCTL_LXTFDEN_Msk          (0x1ul << CLK_CLKDCTL_LXTFDEN_Pos)                /*!< CLK_T::CLKDCTL: LXTFDEN Mask           */

#define CLK_CLKDCTL_LXTFIEN_Pos          (13)                                              /*!< CLK_T::CLKDCTL: LXTFIEN Position       */
#define CLK_CLKDCTL_LXTFIEN_Msk          (0x1ul << CLK_CLKDCTL_LXTFIEN_Pos)                /*!< CLK_T::CLKDCTL: LXTFIEN Mask           */

#define CLK_CLKDCTL_HXTFQDEN_Pos         (16)                                              /*!< CLK_T::CLKDCTL: HXTFQDEN Position      */
#define CLK_CLKDCTL_HXTFQDEN_Msk         (0x1ul << CLK_CLKDCTL_HXTFQDEN_Pos)               /*!< CLK_T::CLKDCTL: HXTFQDEN Mask          */

#define CLK_CLKDCTL_HXTFQIEN_Pos         (17)                                              /*!< CLK_T::CLKDCTL: HXTFQIEN Position      */
#define CLK_CLKDCTL_HXTFQIEN_Msk         (0x1ul << CLK_CLKDCTL_HXTFQIEN_Pos)               /*!< CLK_T::CLKDCTL: HXTFQIEN Mask          */

#define CLK_CLKDCTL_HXTFQASW_Pos         (18)                                              /*!< CLK_T::CLKDCTL: HXTFQASW Position      */
#define CLK_CLKDCTL_HXTFQASW_Msk         (0x1ul << CLK_CLKDCTL_HXTFQASW_Pos)               /*!< CLK_T::CLKDCTL: HXTFQASW Mask          */

#define CLK_CLKDSTS_HXTFDST_Pos          (4)                                               /*!< CLK_T::CLKDSTS: HXTFDST Position       */
#define CLK_CLKDSTS_HXTFDST_Msk          (0x1ul << CLK_CLKDSTS_HXTFDST_Pos)                /*!< CLK_T::CLKDSTS: HXTFDST Mask           */

#define CLK_CLKDSTS_HXTFIF_Pos           (5)                                               /*!< CLK_T::CLKDSTS: HXTFIF Position        */
#define CLK_CLKDSTS_HXTFIF_Msk           (0x1ul << CLK_CLKDSTS_HXTFIF_Pos)                 /*!< CLK_T::CLKDSTS: HXTFIF Mask            */

#define CLK_CLKDSTS_LXTFDST_Pos          (12)                                              /*!< CLK_T::CLKDSTS: LXTFDST Position       */
#define CLK_CLKDSTS_LXTFDST_Msk          (0x1ul << CLK_CLKDSTS_LXTFDST_Pos)                /*!< CLK_T::CLKDSTS: LXTFDST Mask           */

#define CLK_CLKDSTS_LXTFIF_Pos           (13)                                              /*!< CLK_T::CLKDSTS: LXTFIF Position        */
#define CLK_CLKDSTS_LXTFIF_Msk           (0x1ul << CLK_CLKDSTS_LXTFIF_Pos)                 /*!< CLK_T::CLKDSTS: LXTFIF Mask            */

#define CLK_CLKDSTS_HXTFQIF_Pos          (17)                                              /*!< CLK_T::CLKDSTS: HXTFQIF Position       */
#define CLK_CLKDSTS_HXTFQIF_Msk          (0x1ul << CLK_CLKDSTS_HXTFQIF_Pos)                /*!< CLK_T::CLKDSTS: HXTFQIF Mask           */

#define CLK_CDUPB_UPERBD_Pos             (0)                                               /*!< CLK_T::CDUPB: UPERBD Position          */
#define CLK_CDUPB_UPERBD_Msk             (0x3fful << CLK_CDUPB_UPERBD_Pos)                 /*!< CLK_T::CDUPB: UPERBD Mask              */

#define CLK_CDLOWB_LOWERBD_Pos           (0)                                               /*!< CLK_T::CDLOWB: LOWERBD Position        */
#define CLK_CDLOWB_LOWERBD_Msk           (0x3fful << CLK_CDLOWB_LOWERBD_Pos)               /*!< CLK_T::CDLOWB: LOWERBD Mask            */

#define CLK_STOPREQ_CANFD0STR_Pos        (0)                                               /*!< CLK_T::STOPREQ: CANFD0STR Position     */
#define CLK_STOPREQ_CANFD0STR_Msk        (0x1ul << CLK_STOPREQ_CANFD0STR_Pos)              /*!< CLK_T::STOPREQ: CANFD0STR Mask         */

#define CLK_STOPREQ_CANFD1STR_Pos        (1)                                               /*!< CLK_T::STOPREQ: CANFD1STR Position     */
#define CLK_STOPREQ_CANFD1STR_Msk        (0x1ul << CLK_STOPREQ_CANFD1STR_Pos)              /*!< CLK_T::STOPREQ: CANFD1STR Mask         */

#define CLK_STOPACK_CANFD0STA_Pos        (0)                                               /*!< CLK_T::STOPACK: CANFD0STA Position     */
#define CLK_STOPACK_CANFD0STA_Msk        (0x1ul << CLK_STOPACK_CANFD0STA_Pos)              /*!< CLK_T::STOPACK: CANFD0STA Mask         */

#define CLK_STOPACK_CANFD1STA_Pos        (1)                                               /*!< CLK_T::STOPACK: CANFD1STA Position     */
#define CLK_STOPACK_CANFD1STA_Msk        (0x1ul << CLK_STOPACK_CANFD1STA_Pos)              /*!< CLK_T::STOPACK: CANFD1STA Mask         */

#define CLK_ACMPCTL_ACMP01CKEN_Pos       (0)                                               /*!< CLK_T::ACMPCTL: ACMP01CKEN Position    */
#define CLK_ACMPCTL_ACMP01CKEN_Msk       (0x1ul << CLK_ACMPCTL_ACMP01CKEN_Pos)             /*!< CLK_T::ACMPCTL: ACMP01CKEN Mask        */

#define CLK_ACMPCTL_ACMP23CKEN_Pos       (1)                                               /*!< CLK_T::ACMPCTL: ACMP23CKEN Position    */
#define CLK_ACMPCTL_ACMP23CKEN_Msk       (0x1ul << CLK_ACMPCTL_ACMP23CKEN_Pos)             /*!< CLK_T::ACMPCTL: ACMP23CKEN Mask        */

#define CLK_AWFCTL_AWF0CKEN_Pos          (0)                                               /*!< CLK_T::AWFCTL: AWF0CKEN Position       */
#define CLK_AWFCTL_AWF0CKEN_Msk          (0x1ul << CLK_AWFCTL_AWF0CKEN_Pos)                /*!< CLK_T::AWFCTL: AWF0CKEN Mask           */

#define CLK_BPWMCTL_BPWM0CKEN_Pos        (0)                                               /*!< CLK_T::BPWMCTL: BPWM0CKEN Position     */
#define CLK_BPWMCTL_BPWM0CKEN_Msk        (0x1ul << CLK_BPWMCTL_BPWM0CKEN_Pos)              /*!< CLK_T::BPWMCTL: BPWM0CKEN Mask         */

#define CLK_BPWMCTL_BPWM1CKEN_Pos        (1)                                               /*!< CLK_T::BPWMCTL: BPWM1CKEN Position     */
#define CLK_BPWMCTL_BPWM1CKEN_Msk        (0x1ul << CLK_BPWMCTL_BPWM1CKEN_Pos)              /*!< CLK_T::BPWMCTL: BPWM1CKEN Mask         */

#define CLK_CANFDCTL_CANFD0CKEN_Pos      (0)                                               /*!< CLK_T::CANFDCTL: CANFD0CKEN Position   */
#define CLK_CANFDCTL_CANFD0CKEN_Msk      (0x1ul << CLK_CANFDCTL_CANFD0CKEN_Pos)            /*!< CLK_T::CANFDCTL: CANFD0CKEN Mask       */

#define CLK_CANFDCTL_CANFD1CKEN_Pos      (1)                                               /*!< CLK_T::CANFDCTL: CANFD1CKEN Position   */
#define CLK_CANFDCTL_CANFD1CKEN_Msk      (0x1ul << CLK_CANFDCTL_CANFD1CKEN_Pos)            /*!< CLK_T::CANFDCTL: CANFD1CKEN Mask       */

#define CLK_CANFDCTL_CANRM0CKEN_Pos      (16)                                              /*!< CLK_T::CANFDCTL: CANRM0CKEN Position   */
#define CLK_CANFDCTL_CANRM0CKEN_Msk      (0x1ul << CLK_CANFDCTL_CANRM0CKEN_Pos)            /*!< CLK_T::CANFDCTL: CANRM0CKEN Mask       */

#define CLK_CANFDCTL_CANRM1CKEN_Pos      (17)                                              /*!< CLK_T::CANFDCTL: CANRM1CKEN Position   */
#define CLK_CANFDCTL_CANRM1CKEN_Msk      (0x1ul << CLK_CANFDCTL_CANRM1CKEN_Pos)            /*!< CLK_T::CANFDCTL: CANRM1CKEN Mask       */

#define CLK_CCAPCTL_CCAP0CKEN_Pos        (0)                                               /*!< CLK_T::CCAPCTL: CCAP0CKEN Position     */
#define CLK_CCAPCTL_CCAP0CKEN_Msk        (0x1ul << CLK_CCAPCTL_CCAP0CKEN_Pos)              /*!< CLK_T::CCAPCTL: CCAP0CKEN Mask         */

#define CLK_CRCCTL_CRC0CKEN_Pos          (0)                                               /*!< CLK_T::CRCCTL: CRC0CKEN Position       */
#define CLK_CRCCTL_CRC0CKEN_Msk          (0x1ul << CLK_CRCCTL_CRC0CKEN_Pos)                /*!< CLK_T::CRCCTL: CRC0CKEN Mask           */

#define CLK_CRYPTOCTL_CRYPTO0CKEN_Pos    (0)                                               /*!< CLK_T::CRYPTOCTL: CRYPTO0CKEN Position */
#define CLK_CRYPTOCTL_CRYPTO0CKEN_Msk    (0x1ul << CLK_CRYPTOCTL_CRYPTO0CKEN_Pos)          /*!< CLK_T::CRYPTOCTL: CRYPTO0CKEN Mask     */

#define CLK_DACCTL_DAC01CKEN_Pos         (0)                                               /*!< CLK_T::DACCTL: DAC01CKEN Position      */
#define CLK_DACCTL_DAC01CKEN_Msk         (0x1ul << CLK_DACCTL_DAC01CKEN_Pos)               /*!< CLK_T::DACCTL: DAC01CKEN Mask          */

#define CLK_DMICCTL_DMIC0CKEN_Pos        (0)                                               /*!< CLK_T::DMICCTL: DMIC0CKEN Position     */
#define CLK_DMICCTL_DMIC0CKEN_Msk        (0x1ul << CLK_DMICCTL_DMIC0CKEN_Pos)              /*!< CLK_T::DMICCTL: DMIC0CKEN Mask         */

#define CLK_EADCCTL_EADC0CKEN_Pos        (0)                                               /*!< CLK_T::EADCCTL: EADC0CKEN Position     */
#define CLK_EADCCTL_EADC0CKEN_Msk        (0x1ul << CLK_EADCCTL_EADC0CKEN_Pos)              /*!< CLK_T::EADCCTL: EADC0CKEN Mask         */

#define CLK_EBICTL_EBI0CKEN_Pos          (0)                                               /*!< CLK_T::EBICTL: EBI0CKEN Position       */
#define CLK_EBICTL_EBI0CKEN_Msk          (0x1ul << CLK_EBICTL_EBI0CKEN_Pos)                /*!< CLK_T::EBICTL: EBI0CKEN Mask           */

#define CLK_ECAPCTL_ECAP0CKEN_Pos        (0)                                               /*!< CLK_T::ECAPCTL: ECAP0CKEN Position     */
#define CLK_ECAPCTL_ECAP0CKEN_Msk        (0x1ul << CLK_ECAPCTL_ECAP0CKEN_Pos)              /*!< CLK_T::ECAPCTL: ECAP0CKEN Mask         */

#define CLK_ECAPCTL_ECAP1CKEN_Pos        (1)                                               /*!< CLK_T::ECAPCTL: ECAP1CKEN Position     */
#define CLK_ECAPCTL_ECAP1CKEN_Msk        (0x1ul << CLK_ECAPCTL_ECAP1CKEN_Pos)              /*!< CLK_T::ECAPCTL: ECAP1CKEN Mask         */

#define CLK_ECAPCTL_ECAP2CKEN_Pos        (2)                                               /*!< CLK_T::ECAPCTL: ECAP2CKEN Position     */
#define CLK_ECAPCTL_ECAP2CKEN_Msk        (0x1ul << CLK_ECAPCTL_ECAP2CKEN_Pos)              /*!< CLK_T::ECAPCTL: ECAP2CKEN Mask         */

#define CLK_ECAPCTL_ECAP3CKEN_Pos        (3)                                               /*!< CLK_T::ECAPCTL: ECAP3CKEN Position     */
#define CLK_ECAPCTL_ECAP3CKEN_Msk        (0x1ul << CLK_ECAPCTL_ECAP3CKEN_Pos)              /*!< CLK_T::ECAPCTL: ECAP3CKEN Mask         */

#define CLK_EMACCTL_EMAC0CKEN_Pos        (0)                                               /*!< CLK_T::EMACCTL: EMAC0CKEN Position     */
#define CLK_EMACCTL_EMAC0CKEN_Msk        (0x1ul << CLK_EMACCTL_EMAC0CKEN_Pos)              /*!< CLK_T::EMACCTL: EMAC0CKEN Mask         */

#define CLK_EPWMCTL_EPWM0CKEN_Pos        (0)                                               /*!< CLK_T::EPWMCTL: EPWM0CKEN Position     */
#define CLK_EPWMCTL_EPWM0CKEN_Msk        (0x1ul << CLK_EPWMCTL_EPWM0CKEN_Pos)              /*!< CLK_T::EPWMCTL: EPWM0CKEN Mask         */

#define CLK_EPWMCTL_EPWM1CKEN_Pos        (1)                                               /*!< CLK_T::EPWMCTL: EPWM1CKEN Position     */
#define CLK_EPWMCTL_EPWM1CKEN_Msk        (0x1ul << CLK_EPWMCTL_EPWM1CKEN_Pos)              /*!< CLK_T::EPWMCTL: EPWM1CKEN Mask         */

#define CLK_EQEICTL_EQEI0CKEN_Pos        (0)                                               /*!< CLK_T::EQEICTL: EQEI0CKEN Position     */
#define CLK_EQEICTL_EQEI0CKEN_Msk        (0x1ul << CLK_EQEICTL_EQEI0CKEN_Pos)              /*!< CLK_T::EQEICTL: EQEI0CKEN Mask         */

#define CLK_EQEICTL_EQEI1CKEN_Pos        (1)                                               /*!< CLK_T::EQEICTL: EQEI1CKEN Position     */
#define CLK_EQEICTL_EQEI1CKEN_Msk        (0x1ul << CLK_EQEICTL_EQEI1CKEN_Pos)              /*!< CLK_T::EQEICTL: EQEI1CKEN Mask         */

#define CLK_EQEICTL_EQEI2CKEN_Pos        (2)                                               /*!< CLK_T::EQEICTL: EQEI2CKEN Position     */
#define CLK_EQEICTL_EQEI2CKEN_Msk        (0x1ul << CLK_EQEICTL_EQEI2CKEN_Pos)              /*!< CLK_T::EQEICTL: EQEI2CKEN Mask         */

#define CLK_EQEICTL_EQEI3CKEN_Pos        (3)                                               /*!< CLK_T::EQEICTL: EQEI3CKEN Position     */
#define CLK_EQEICTL_EQEI3CKEN_Msk        (0x1ul << CLK_EQEICTL_EQEI3CKEN_Pos)              /*!< CLK_T::EQEICTL: EQEI3CKEN Mask         */

#define CLK_FMCCTL_FMC0CKEN_Pos          (0)                                               /*!< CLK_T::FMCCTL: FMC0CKEN Position       */
#define CLK_FMCCTL_FMC0CKEN_Msk          (0x1ul << CLK_FMCCTL_FMC0CKEN_Pos)                /*!< CLK_T::FMCCTL: FMC0CKEN Mask           */

#define CLK_FMCCTL_ISP0CKEN_Pos          (16)                                              /*!< CLK_T::FMCCTL: ISP0CKEN Position       */
#define CLK_FMCCTL_ISP0CKEN_Msk          (0x1ul << CLK_FMCCTL_ISP0CKEN_Pos)                /*!< CLK_T::FMCCTL: ISP0CKEN Mask           */

#define CLK_GDMACTL_GDMA0CKEN_Pos        (0)                                               /*!< CLK_T::GDMACTL: GDMA0CKEN Position     */
#define CLK_GDMACTL_GDMA0CKEN_Msk        (0x1ul << CLK_GDMACTL_GDMA0CKEN_Pos)              /*!< CLK_T::GDMACTL: GDMA0CKEN Mask         */

#define CLK_GPIOCTL_GPIOACKEN_Pos        (0)                                               /*!< CLK_T::GPIOCTL: GPIOACKEN Position     */
#define CLK_GPIOCTL_GPIOACKEN_Msk        (0x1ul << CLK_GPIOCTL_GPIOACKEN_Pos)              /*!< CLK_T::GPIOCTL: GPIOACKEN Mask         */

#define CLK_GPIOCTL_GPIOBCKEN_Pos        (1)                                               /*!< CLK_T::GPIOCTL: GPIOBCKEN Position     */
#define CLK_GPIOCTL_GPIOBCKEN_Msk        (0x1ul << CLK_GPIOCTL_GPIOBCKEN_Pos)              /*!< CLK_T::GPIOCTL: GPIOBCKEN Mask         */

#define CLK_GPIOCTL_GPIOCCKEN_Pos        (2)                                               /*!< CLK_T::GPIOCTL: GPIOCCKEN Position     */
#define CLK_GPIOCTL_GPIOCCKEN_Msk        (0x1ul << CLK_GPIOCTL_GPIOCCKEN_Pos)              /*!< CLK_T::GPIOCTL: GPIOCCKEN Mask         */

#define CLK_GPIOCTL_GPIODCKEN_Pos        (3)                                               /*!< CLK_T::GPIOCTL: GPIODCKEN Position     */
#define CLK_GPIOCTL_GPIODCKEN_Msk        (0x1ul << CLK_GPIOCTL_GPIODCKEN_Pos)              /*!< CLK_T::GPIOCTL: GPIODCKEN Mask         */

#define CLK_GPIOCTL_GPIOECKEN_Pos        (4)                                               /*!< CLK_T::GPIOCTL: GPIOECKEN Position     */
#define CLK_GPIOCTL_GPIOECKEN_Msk        (0x1ul << CLK_GPIOCTL_GPIOECKEN_Pos)              /*!< CLK_T::GPIOCTL: GPIOECKEN Mask         */

#define CLK_GPIOCTL_GPIOFCKEN_Pos        (5)                                               /*!< CLK_T::GPIOCTL: GPIOFCKEN Position     */
#define CLK_GPIOCTL_GPIOFCKEN_Msk        (0x1ul << CLK_GPIOCTL_GPIOFCKEN_Pos)              /*!< CLK_T::GPIOCTL: GPIOFCKEN Mask         */

#define CLK_GPIOCTL_GPIOGCKEN_Pos        (6)                                               /*!< CLK_T::GPIOCTL: GPIOGCKEN Position     */
#define CLK_GPIOCTL_GPIOGCKEN_Msk        (0x1ul << CLK_GPIOCTL_GPIOGCKEN_Pos)              /*!< CLK_T::GPIOCTL: GPIOGCKEN Mask         */

#define CLK_GPIOCTL_GPIOHCKEN_Pos        (7)                                               /*!< CLK_T::GPIOCTL: GPIOHCKEN Position     */
#define CLK_GPIOCTL_GPIOHCKEN_Msk        (0x1ul << CLK_GPIOCTL_GPIOHCKEN_Pos)              /*!< CLK_T::GPIOCTL: GPIOHCKEN Mask         */

#define CLK_GPIOCTL_GPIOICKEN_Pos        (8)                                               /*!< CLK_T::GPIOCTL: GPIOICKEN Position     */
#define CLK_GPIOCTL_GPIOICKEN_Msk        (0x1ul << CLK_GPIOCTL_GPIOICKEN_Pos)              /*!< CLK_T::GPIOCTL: GPIOICKEN Mask         */

#define CLK_GPIOCTL_GPIOJCKEN_Pos        (9)                                               /*!< CLK_T::GPIOCTL: GPIOJCKEN Position     */
#define CLK_GPIOCTL_GPIOJCKEN_Msk        (0x1ul << CLK_GPIOCTL_GPIOJCKEN_Pos)              /*!< CLK_T::GPIOCTL: GPIOJCKEN Mask         */

#define CLK_HSOTGCTL_HSOTG0CKEN_Pos      (0)                                               /*!< CLK_T::HSOTGCTL: HSOTG0CKEN Position   */
#define CLK_HSOTGCTL_HSOTG0CKEN_Msk      (0x1ul << CLK_HSOTGCTL_HSOTG0CKEN_Pos)            /*!< CLK_T::HSOTGCTL: HSOTG0CKEN Mask       */

#define CLK_HSUSBDCTL_HSUSBD0CKEN_Pos    (0)                                               /*!< CLK_T::HSUSBDCTL: HSUSBD0CKEN Position */
#define CLK_HSUSBDCTL_HSUSBD0CKEN_Msk    (0x1ul << CLK_HSUSBDCTL_HSUSBD0CKEN_Pos)          /*!< CLK_T::HSUSBDCTL: HSUSBD0CKEN Mask     */

#define CLK_HSUSBHCTL_HSUSBH0CKEN_Pos    (0)                                               /*!< CLK_T::HSUSBHCTL: HSUSBH0CKEN Position */
#define CLK_HSUSBHCTL_HSUSBH0CKEN_Msk    (0x1ul << CLK_HSUSBHCTL_HSUSBH0CKEN_Pos)          /*!< CLK_T::HSUSBHCTL: HSUSBH0CKEN Mask     */

#define CLK_I2CCTL_I2C0CKEN_Pos          (0)                                               /*!< CLK_T::I2CCTL: I2C0CKEN Position       */
#define CLK_I2CCTL_I2C0CKEN_Msk          (0x1ul << CLK_I2CCTL_I2C0CKEN_Pos)                /*!< CLK_T::I2CCTL: I2C0CKEN Mask           */

#define CLK_I2CCTL_I2C1CKEN_Pos          (1)                                               /*!< CLK_T::I2CCTL: I2C1CKEN Position       */
#define CLK_I2CCTL_I2C1CKEN_Msk          (0x1ul << CLK_I2CCTL_I2C1CKEN_Pos)                /*!< CLK_T::I2CCTL: I2C1CKEN Mask           */

#define CLK_I2CCTL_I2C2CKEN_Pos          (2)                                               /*!< CLK_T::I2CCTL: I2C2CKEN Position       */
#define CLK_I2CCTL_I2C2CKEN_Msk          (0x1ul << CLK_I2CCTL_I2C2CKEN_Pos)                /*!< CLK_T::I2CCTL: I2C2CKEN Mask           */

#define CLK_I2CCTL_I2C3CKEN_Pos          (3)                                               /*!< CLK_T::I2CCTL: I2C3CKEN Position       */
#define CLK_I2CCTL_I2C3CKEN_Msk          (0x1ul << CLK_I2CCTL_I2C3CKEN_Pos)                /*!< CLK_T::I2CCTL: I2C3CKEN Mask           */

#define CLK_I2SCTL_I2S0CKEN_Pos          (0)                                               /*!< CLK_T::I2SCTL: I2S0CKEN Position       */
#define CLK_I2SCTL_I2S0CKEN_Msk          (0x1ul << CLK_I2SCTL_I2S0CKEN_Pos)                /*!< CLK_T::I2SCTL: I2S0CKEN Mask           */

#define CLK_I2SCTL_I2S1CKEN_Pos          (1)                                               /*!< CLK_T::I2SCTL: I2S1CKEN Position       */
#define CLK_I2SCTL_I2S1CKEN_Msk          (0x1ul << CLK_I2SCTL_I2S1CKEN_Pos)                /*!< CLK_T::I2SCTL: I2S1CKEN Mask           */

#define CLK_I3CCTL_I3C0CKEN_Pos          (0)                                               /*!< CLK_T::I3CCTL: I3C0CKEN Position       */
#define CLK_I3CCTL_I3C0CKEN_Msk          (0x1ul << CLK_I3CCTL_I3C0CKEN_Pos)                /*!< CLK_T::I3CCTL: I3C0CKEN Mask           */

#define CLK_KDFCTL_KDF0CKEN_Pos          (0)                                               /*!< CLK_T::KDFCTL: KDF0CKEN Position       */
#define CLK_KDFCTL_KDF0CKEN_Msk          (0x1ul << CLK_KDFCTL_KDF0CKEN_Pos)                /*!< CLK_T::KDFCTL: KDF0CKEN Mask           */

#define CLK_KPICTL_KPI0CKEN_Pos          (0)                                               /*!< CLK_T::KPICTL: KPI0CKEN Position       */
#define CLK_KPICTL_KPI0CKEN_Msk          (0x1ul << CLK_KPICTL_KPI0CKEN_Pos)                /*!< CLK_T::KPICTL: KPI0CKEN Mask           */

#define CLK_KSCTL_KS0CKEN_Pos            (0)                                               /*!< CLK_T::KSCTL: KS0CKEN Position         */
#define CLK_KSCTL_KS0CKEN_Msk            (0x1ul << CLK_KSCTL_KS0CKEN_Pos)                  /*!< CLK_T::KSCTL: KS0CKEN Mask             */

#define CLK_LPADCCTL_LPADC0CKEN_Pos      (0)                                               /*!< CLK_T::LPADCCTL: LPADC0CKEN Position   */
#define CLK_LPADCCTL_LPADC0CKEN_Msk      (0x1ul << CLK_LPADCCTL_LPADC0CKEN_Pos)            /*!< CLK_T::LPADCCTL: LPADC0CKEN Mask       */

#define CLK_LPPDMACTL_LPPDMA0CKEN_Pos    (0)                                               /*!< CLK_T::LPPDMACTL: LPPDMA0CKEN Position */
#define CLK_LPPDMACTL_LPPDMA0CKEN_Msk    (0x1ul << CLK_LPPDMACTL_LPPDMA0CKEN_Pos)          /*!< CLK_T::LPPDMACTL: LPPDMA0CKEN Mask     */

#define CLK_LPGPIOCTL_LPGPIO0CKEN_Pos    (0)                                               /*!< CLK_T::LPGPIOCTL: LPGPIO0CKEN Position */
#define CLK_LPGPIOCTL_LPGPIO0CKEN_Msk    (0x1ul << CLK_LPGPIOCTL_LPGPIO0CKEN_Pos)          /*!< CLK_T::LPGPIOCTL: LPGPIO0CKEN Mask     */

#define CLK_LPI2CCTL_LPI2C0CKEN_Pos      (0)                                               /*!< CLK_T::LPI2CCTL: LPI2C0CKEN Position   */
#define CLK_LPI2CCTL_LPI2C0CKEN_Msk      (0x1ul << CLK_LPI2CCTL_LPI2C0CKEN_Pos)            /*!< CLK_T::LPI2CCTL: LPI2C0CKEN Mask       */

#define CLK_LPSPICTL_LPSPI0CKEN_Pos      (0)                                               /*!< CLK_T::LPSPICTL: LPSPI0CKEN Position   */
#define CLK_LPSPICTL_LPSPI0CKEN_Msk      (0x1ul << CLK_LPSPICTL_LPSPI0CKEN_Pos)            /*!< CLK_T::LPSPICTL: LPSPI0CKEN Mask       */

#define CLK_LPSRAMCTL_LPSRAM0CKEN_Pos    (0)                                               /*!< CLK_T::LPSRAMCTL: LPSRAM0CKEN Position */
#define CLK_LPSRAMCTL_LPSRAM0CKEN_Msk    (0x1ul << CLK_LPSRAMCTL_LPSRAM0CKEN_Pos)          /*!< CLK_T::LPSRAMCTL: LPSRAM0CKEN Mask     */

#define CLK_LPTMRCTL_LPTMR0CKEN_Pos      (0)                                               /*!< CLK_T::LPTMRCTL: LPTMR0CKEN Position   */
#define CLK_LPTMRCTL_LPTMR0CKEN_Msk      (0x1ul << CLK_LPTMRCTL_LPTMR0CKEN_Pos)            /*!< CLK_T::LPTMRCTL: LPTMR0CKEN Mask       */

#define CLK_LPTMRCTL_LPTMR1CKEN_Pos      (1)                                               /*!< CLK_T::LPTMRCTL: LPTMR1CKEN Position   */
#define CLK_LPTMRCTL_LPTMR1CKEN_Msk      (0x1ul << CLK_LPTMRCTL_LPTMR1CKEN_Pos)            /*!< CLK_T::LPTMRCTL: LPTMR1CKEN Mask       */

#define CLK_LPUARTCTL_LPUART0CKEN_Pos    (0)                                               /*!< CLK_T::LPUARTCTL: LPUART0CKEN Position */
#define CLK_LPUARTCTL_LPUART0CKEN_Msk    (0x1ul << CLK_LPUARTCTL_LPUART0CKEN_Pos)          /*!< CLK_T::LPUARTCTL: LPUART0CKEN Mask     */

#define CLK_NPUCTL_NPU0CKEN_Pos          (0)                                               /*!< CLK_T::NPUCTL: NPU0CKEN Position       */
#define CLK_NPUCTL_NPU0CKEN_Msk          (0x1ul << CLK_NPUCTL_NPU0CKEN_Pos)                /*!< CLK_T::NPUCTL: NPU0CKEN Mask           */

#define CLK_OTFCCTL_OTFC0CKEN_Pos        (0)                                               /*!< CLK_T::OTFCCTL: OTFC0CKEN Position     */
#define CLK_OTFCCTL_OTFC0CKEN_Msk        (0x1ul << CLK_OTFCCTL_OTFC0CKEN_Pos)              /*!< CLK_T::OTFCCTL: OTFC0CKEN Mask         */

#define CLK_OTGCTL_OTG0CKEN_Pos          (0)                                               /*!< CLK_T::OTGCTL: OTG0CKEN Position       */
#define CLK_OTGCTL_OTG0CKEN_Msk          (0x1ul << CLK_OTGCTL_OTG0CKEN_Pos)                /*!< CLK_T::OTGCTL: OTG0CKEN Mask           */

#define CLK_PDMACTL_PDMA0CKEN_Pos        (0)                                               /*!< CLK_T::PDMACTL: PDMA0CKEN Position     */
#define CLK_PDMACTL_PDMA0CKEN_Msk        (0x1ul << CLK_PDMACTL_PDMA0CKEN_Pos)              /*!< CLK_T::PDMACTL: PDMA0CKEN Mask         */

#define CLK_PDMACTL_PDMA1CKEN_Pos        (1)                                               /*!< CLK_T::PDMACTL: PDMA1CKEN Position     */
#define CLK_PDMACTL_PDMA1CKEN_Msk        (0x1ul << CLK_PDMACTL_PDMA1CKEN_Pos)              /*!< CLK_T::PDMACTL: PDMA1CKEN Mask         */

#define CLK_PSIOCTL_PSIO0CKEN_Pos        (0)                                               /*!< CLK_T::PSIOCTL: PSIO0CKEN Position     */
#define CLK_PSIOCTL_PSIO0CKEN_Msk        (0x1ul << CLK_PSIOCTL_PSIO0CKEN_Pos)              /*!< CLK_T::PSIOCTL: PSIO0CKEN Mask         */

#define CLK_QSPICTL_QSPI0CKEN_Pos        (0)                                               /*!< CLK_T::QSPICTL: QSPI0CKEN Position     */
#define CLK_QSPICTL_QSPI0CKEN_Msk        (0x1ul << CLK_QSPICTL_QSPI0CKEN_Pos)              /*!< CLK_T::QSPICTL: QSPI0CKEN Mask         */

#define CLK_QSPICTL_QSPI1CKEN_Pos        (1)                                               /*!< CLK_T::QSPICTL: QSPI1CKEN Position     */
#define CLK_QSPICTL_QSPI1CKEN_Msk        (0x1ul << CLK_QSPICTL_QSPI1CKEN_Pos)              /*!< CLK_T::QSPICTL: QSPI1CKEN Mask         */

#define CLK_RTCCTL_RTC0CKEN_Pos          (0)                                               /*!< CLK_T::RTCCTL: RTC0CKEN Position       */
#define CLK_RTCCTL_RTC0CKEN_Msk          (0x1ul << CLK_RTCCTL_RTC0CKEN_Pos)                /*!< CLK_T::RTCCTL: RTC0CKEN Mask           */

#define CLK_SCCTL_SC0CKEN_Pos            (0)                                               /*!< CLK_T::SCCTL: SC0CKEN Position         */
#define CLK_SCCTL_SC0CKEN_Msk            (0x1ul << CLK_SCCTL_SC0CKEN_Pos)                  /*!< CLK_T::SCCTL: SC0CKEN Mask             */

#define CLK_SCCTL_SC1CKEN_Pos            (1)                                               /*!< CLK_T::SCCTL: SC1CKEN Position         */
#define CLK_SCCTL_SC1CKEN_Msk            (0x1ul << CLK_SCCTL_SC1CKEN_Pos)                  /*!< CLK_T::SCCTL: SC1CKEN Mask             */

#define CLK_SCCTL_SC2CKEN_Pos            (2)                                               /*!< CLK_T::SCCTL: SC2CKEN Position         */
#define CLK_SCCTL_SC2CKEN_Msk            (0x1ul << CLK_SCCTL_SC2CKEN_Pos)                  /*!< CLK_T::SCCTL: SC2CKEN Mask             */

#define CLK_SCUCTL_SCU0CKEN_Pos          (0)                                               /*!< CLK_T::SCUCTL: SCU0CKEN Position       */
#define CLK_SCUCTL_SCU0CKEN_Msk          (0x1ul << CLK_SCUCTL_SCU0CKEN_Pos)                /*!< CLK_T::SCUCTL: SCU0CKEN Mask           */

#define CLK_SDHCTL_SDH0CKEN_Pos          (0)                                               /*!< CLK_T::SDHCTL: SDH0CKEN Position       */
#define CLK_SDHCTL_SDH0CKEN_Msk          (0x1ul << CLK_SDHCTL_SDH0CKEN_Pos)                /*!< CLK_T::SDHCTL: SDH0CKEN Mask           */

#define CLK_SDHCTL_SDH1CKEN_Pos          (1)                                               /*!< CLK_T::SDHCTL: SDH1CKEN Position       */
#define CLK_SDHCTL_SDH1CKEN_Msk          (0x1ul << CLK_SDHCTL_SDH1CKEN_Pos)                /*!< CLK_T::SDHCTL: SDH1CKEN Mask           */

#define CLK_SPICTL_SPI0CKEN_Pos          (0)                                               /*!< CLK_T::SPICTL: SPI0CKEN Position       */
#define CLK_SPICTL_SPI0CKEN_Msk          (0x1ul << CLK_SPICTL_SPI0CKEN_Pos)                /*!< CLK_T::SPICTL: SPI0CKEN Mask           */

#define CLK_SPICTL_SPI1CKEN_Pos          (1)                                               /*!< CLK_T::SPICTL: SPI1CKEN Position       */
#define CLK_SPICTL_SPI1CKEN_Msk          (0x1ul << CLK_SPICTL_SPI1CKEN_Pos)                /*!< CLK_T::SPICTL: SPI1CKEN Mask           */

#define CLK_SPICTL_SPI2CKEN_Pos          (2)                                               /*!< CLK_T::SPICTL: SPI2CKEN Position       */
#define CLK_SPICTL_SPI2CKEN_Msk          (0x1ul << CLK_SPICTL_SPI2CKEN_Pos)                /*!< CLK_T::SPICTL: SPI2CKEN Mask           */

#define CLK_SPICTL_SPI3CKEN_Pos          (3)                                               /*!< CLK_T::SPICTL: SPI3CKEN Position       */
#define CLK_SPICTL_SPI3CKEN_Msk          (0x1ul << CLK_SPICTL_SPI3CKEN_Pos)                /*!< CLK_T::SPICTL: SPI3CKEN Mask           */

#define CLK_SPIMCTL_SPIM0CKEN_Pos        (0)                                               /*!< CLK_T::SPIMCTL: SPIM0CKEN Position     */
#define CLK_SPIMCTL_SPIM0CKEN_Msk        (0x1ul << CLK_SPIMCTL_SPIM0CKEN_Pos)              /*!< CLK_T::SPIMCTL: SPIM0CKEN Mask         */

#define CLK_SRAMCTL_SRAM0CKEN_Pos        (0)                                               /*!< CLK_T::SRAMCTL: SRAM0CKEN Position     */
#define CLK_SRAMCTL_SRAM0CKEN_Msk        (0x1ul << CLK_SRAMCTL_SRAM0CKEN_Pos)              /*!< CLK_T::SRAMCTL: SRAM0CKEN Mask         */

#define CLK_SRAMCTL_SRAM1CKEN_Pos        (1)                                               /*!< CLK_T::SRAMCTL: SRAM1CKEN Position     */
#define CLK_SRAMCTL_SRAM1CKEN_Msk        (0x1ul << CLK_SRAMCTL_SRAM1CKEN_Pos)              /*!< CLK_T::SRAMCTL: SRAM1CKEN Mask         */

#define CLK_SRAMCTL_SRAM2CKEN_Pos        (2)                                               /*!< CLK_T::SRAMCTL: SRAM2CKEN Position     */
#define CLK_SRAMCTL_SRAM2CKEN_Msk        (0x1ul << CLK_SRAMCTL_SRAM2CKEN_Pos)              /*!< CLK_T::SRAMCTL: SRAM2CKEN Mask         */

#define CLK_SRAMCTL_SRAM3CKEN_Pos        (3)                                               /*!< CLK_T::SRAMCTL: SRAM3CKEN Position     */
#define CLK_SRAMCTL_SRAM3CKEN_Msk        (0x1ul << CLK_SRAMCTL_SRAM3CKEN_Pos)              /*!< CLK_T::SRAMCTL: SRAM3CKEN Mask         */

#define CLK_STCTL_ST0CKEN_Pos            (0)                                               /*!< CLK_T::STCTL: ST0CKEN Position         */
#define CLK_STCTL_ST0CKEN_Msk            (0x1ul << CLK_STCTL_ST0CKEN_Pos)                  /*!< CLK_T::STCTL: ST0CKEN Mask             */

#define CLK_TMRCTL_TMR0CKEN_Pos          (0)                                               /*!< CLK_T::TMRCTL: TMR0CKEN Position       */
#define CLK_TMRCTL_TMR0CKEN_Msk          (0x1ul << CLK_TMRCTL_TMR0CKEN_Pos)                /*!< CLK_T::TMRCTL: TMR0CKEN Mask           */

#define CLK_TMRCTL_TMR1CKEN_Pos          (1)                                               /*!< CLK_T::TMRCTL: TMR1CKEN Position       */
#define CLK_TMRCTL_TMR1CKEN_Msk          (0x1ul << CLK_TMRCTL_TMR1CKEN_Pos)                /*!< CLK_T::TMRCTL: TMR1CKEN Mask           */

#define CLK_TMRCTL_TMR2CKEN_Pos          (2)                                               /*!< CLK_T::TMRCTL: TMR2CKEN Position       */
#define CLK_TMRCTL_TMR2CKEN_Msk          (0x1ul << CLK_TMRCTL_TMR2CKEN_Pos)                /*!< CLK_T::TMRCTL: TMR2CKEN Mask           */

#define CLK_TMRCTL_TMR3CKEN_Pos          (3)                                               /*!< CLK_T::TMRCTL: TMR3CKEN Position       */
#define CLK_TMRCTL_TMR3CKEN_Msk          (0x1ul << CLK_TMRCTL_TMR3CKEN_Pos)                /*!< CLK_T::TMRCTL: TMR3CKEN Mask           */

#define CLK_TRNGCTL_TRNG0CKEN_Pos        (0)                                               /*!< CLK_T::TRNGCTL: TRNG0CKEN Position     */
#define CLK_TRNGCTL_TRNG0CKEN_Msk        (0x1ul << CLK_TRNGCTL_TRNG0CKEN_Pos)              /*!< CLK_T::TRNGCTL: TRNG0CKEN Mask         */

#define CLK_TTMRCTL_TTMR0CKEN_Pos        (0)                                               /*!< CLK_T::TTMRCTL: TTMR0CKEN Position     */
#define CLK_TTMRCTL_TTMR0CKEN_Msk        (0x1ul << CLK_TTMRCTL_TTMR0CKEN_Pos)              /*!< CLK_T::TTMRCTL: TTMR0CKEN Mask         */

#define CLK_TTMRCTL_TTMR1CKEN_Pos        (1)                                               /*!< CLK_T::TTMRCTL: TTMR1CKEN Position     */
#define CLK_TTMRCTL_TTMR1CKEN_Msk        (0x1ul << CLK_TTMRCTL_TTMR1CKEN_Pos)              /*!< CLK_T::TTMRCTL: TTMR1CKEN Mask         */

#define CLK_UARTCTL_UART0CKEN_Pos        (0)                                               /*!< CLK_T::UARTCTL: UART0CKEN Position     */
#define CLK_UARTCTL_UART0CKEN_Msk        (0x1ul << CLK_UARTCTL_UART0CKEN_Pos)              /*!< CLK_T::UARTCTL: UART0CKEN Mask         */

#define CLK_UARTCTL_UART1CKEN_Pos        (1)                                               /*!< CLK_T::UARTCTL: UART1CKEN Position     */
#define CLK_UARTCTL_UART1CKEN_Msk        (0x1ul << CLK_UARTCTL_UART1CKEN_Pos)              /*!< CLK_T::UARTCTL: UART1CKEN Mask         */

#define CLK_UARTCTL_UART2CKEN_Pos        (2)                                               /*!< CLK_T::UARTCTL: UART2CKEN Position     */
#define CLK_UARTCTL_UART2CKEN_Msk        (0x1ul << CLK_UARTCTL_UART2CKEN_Pos)              /*!< CLK_T::UARTCTL: UART2CKEN Mask         */

#define CLK_UARTCTL_UART3CKEN_Pos        (3)                                               /*!< CLK_T::UARTCTL: UART3CKEN Position     */
#define CLK_UARTCTL_UART3CKEN_Msk        (0x1ul << CLK_UARTCTL_UART3CKEN_Pos)              /*!< CLK_T::UARTCTL: UART3CKEN Mask         */

#define CLK_UARTCTL_UART4CKEN_Pos        (4)                                               /*!< CLK_T::UARTCTL: UART4CKEN Position     */
#define CLK_UARTCTL_UART4CKEN_Msk        (0x1ul << CLK_UARTCTL_UART4CKEN_Pos)              /*!< CLK_T::UARTCTL: UART4CKEN Mask         */

#define CLK_UARTCTL_UART5CKEN_Pos        (5)                                               /*!< CLK_T::UARTCTL: UART5CKEN Position     */
#define CLK_UARTCTL_UART5CKEN_Msk        (0x1ul << CLK_UARTCTL_UART5CKEN_Pos)              /*!< CLK_T::UARTCTL: UART5CKEN Mask         */

#define CLK_UARTCTL_UART6CKEN_Pos        (6)                                               /*!< CLK_T::UARTCTL: UART6CKEN Position     */
#define CLK_UARTCTL_UART6CKEN_Msk        (0x1ul << CLK_UARTCTL_UART6CKEN_Pos)              /*!< CLK_T::UARTCTL: UART6CKEN Mask         */

#define CLK_UARTCTL_UART7CKEN_Pos        (7)                                               /*!< CLK_T::UARTCTL: UART7CKEN Position     */
#define CLK_UARTCTL_UART7CKEN_Msk        (0x1ul << CLK_UARTCTL_UART7CKEN_Pos)              /*!< CLK_T::UARTCTL: UART7CKEN Mask         */

#define CLK_UARTCTL_UART8CKEN_Pos        (8)                                               /*!< CLK_T::UARTCTL: UART8CKEN Position     */
#define CLK_UARTCTL_UART8CKEN_Msk        (0x1ul << CLK_UARTCTL_UART8CKEN_Pos)              /*!< CLK_T::UARTCTL: UART8CKEN Mask         */

#define CLK_UARTCTL_UART9CKEN_Pos        (9)                                               /*!< CLK_T::UARTCTL: UART9CKEN Position     */
#define CLK_UARTCTL_UART9CKEN_Msk        (0x1ul << CLK_UARTCTL_UART9CKEN_Pos)              /*!< CLK_T::UARTCTL: UART9CKEN Mask         */

#define CLK_USBDCTL_USBD0CKEN_Pos        (0)                                               /*!< CLK_T::USBDCTL: USBD0CKEN Position     */
#define CLK_USBDCTL_USBD0CKEN_Msk        (0x1ul << CLK_USBDCTL_USBD0CKEN_Pos)              /*!< CLK_T::USBDCTL: USBD0CKEN Mask         */

#define CLK_USBHCTL_USBH0CKEN_Pos        (0)                                               /*!< CLK_T::USBHCTL: USBH0CKEN Position     */
#define CLK_USBHCTL_USBH0CKEN_Msk        (0x1ul << CLK_USBHCTL_USBH0CKEN_Pos)              /*!< CLK_T::USBHCTL: USBH0CKEN Mask         */

#define CLK_USCICTL_USCI0CKEN_Pos        (0)                                               /*!< CLK_T::USCICTL: USCI0CKEN Position     */
#define CLK_USCICTL_USCI0CKEN_Msk        (0x1ul << CLK_USCICTL_USCI0CKEN_Pos)              /*!< CLK_T::USCICTL: USCI0CKEN Mask         */

#define CLK_UTCPDCTL_UTCPD0CKEN_Pos      (0)                                               /*!< CLK_T::UTCPDCTL: UTCPD0CKEN Position   */
#define CLK_UTCPDCTL_UTCPD0CKEN_Msk      (0x1ul << CLK_UTCPDCTL_UTCPD0CKEN_Pos)            /*!< CLK_T::UTCPDCTL: UTCPD0CKEN Mask       */

#define CLK_WDTCTL_WDT0CKEN_Pos          (0)                                               /*!< CLK_T::WDTCTL: WDT0CKEN Position       */
#define CLK_WDTCTL_WDT0CKEN_Msk          (0x1ul << CLK_WDTCTL_WDT0CKEN_Pos)                /*!< CLK_T::WDTCTL: WDT0CKEN Mask           */

#define CLK_WDTCTL_WDT1CKEN_Pos          (1)                                               /*!< CLK_T::WDTCTL: WDT1CKEN Position       */
#define CLK_WDTCTL_WDT1CKEN_Msk          (0x1ul << CLK_WDTCTL_WDT1CKEN_Pos)                /*!< CLK_T::WDTCTL: WDT1CKEN Mask           */

#define CLK_WWDTCTL_WWDT0CKEN_Pos        (0)                                               /*!< CLK_T::WWDTCTL: WWDT0CKEN Position     */
#define CLK_WWDTCTL_WWDT0CKEN_Msk        (0x1ul << CLK_WWDTCTL_WWDT0CKEN_Pos)              /*!< CLK_T::WWDTCTL: WWDT0CKEN Mask         */

#define CLK_WWDTCTL_WWDT1CKEN_Pos        (1)                                               /*!< CLK_T::WWDTCTL: WWDT1CKEN Position     */
#define CLK_WWDTCTL_WWDT1CKEN_Msk        (0x1ul << CLK_WWDTCTL_WWDT1CKEN_Pos)              /*!< CLK_T::WWDTCTL: WWDT1CKEN Mask         */

#define CLK_SCLKSEL_SCLKSEL_Pos          (0)                                               /*!< CLK_T::SCLKSEL: SCLKSEL Position       */
#define CLK_SCLKSEL_SCLKSEL_Msk          (0x7ul << CLK_SCLKSEL_SCLKSEL_Pos)                /*!< CLK_T::SCLKSEL: SCLKSEL Mask           */

#define CLK_BPWMSEL_BPWM0SEL_Pos         (0)                                               /*!< CLK_T::BPWMSEL: BPWM0SEL Position      */
#define CLK_BPWMSEL_BPWM0SEL_Msk         (0x1ul << CLK_BPWMSEL_BPWM0SEL_Pos)               /*!< CLK_T::BPWMSEL: BPWM0SEL Mask          */

#define CLK_BPWMSEL_BPWM1SEL_Pos         (4)                                               /*!< CLK_T::BPWMSEL: BPWM1SEL Position      */
#define CLK_BPWMSEL_BPWM1SEL_Msk         (0x1ul << CLK_BPWMSEL_BPWM1SEL_Pos)               /*!< CLK_T::BPWMSEL: BPWM1SEL Mask          */

#define CLK_CANFDSEL_CANFD0SEL_Pos       (0)                                               /*!< CLK_T::CANFDSEL: CANFD0SEL Position    */
#define CLK_CANFDSEL_CANFD0SEL_Msk       (0x7ul << CLK_CANFDSEL_CANFD0SEL_Pos)             /*!< CLK_T::CANFDSEL: CANFD0SEL Mask        */

#define CLK_CANFDSEL_CANFD1SEL_Pos       (4)                                               /*!< CLK_T::CANFDSEL: CANFD1SEL Position    */
#define CLK_CANFDSEL_CANFD1SEL_Msk       (0x7ul << CLK_CANFDSEL_CANFD1SEL_Pos)             /*!< CLK_T::CANFDSEL: CANFD1SEL Mask        */

#define CLK_CCAPSEL_CCAP0SEL_Pos         (0)                                               /*!< CLK_T::CCAPSEL: CCAP0SEL Position      */
#define CLK_CCAPSEL_CCAP0SEL_Msk         (0x7ul << CLK_CCAPSEL_CCAP0SEL_Pos)               /*!< CLK_T::CCAPSEL: CCAP0SEL Mask          */

#define CLK_CLKOSEL_CLKOSEL_Pos          (0)                                               /*!< CLK_T::CLKOSEL: CLKOSEL Position       */
#define CLK_CLKOSEL_CLKOSEL_Msk          (0xful << CLK_CLKOSEL_CLKOSEL_Pos)                /*!< CLK_T::CLKOSEL: CLKOSEL Mask           */

#define CLK_DMICSEL_DMIC0SEL_Pos         (0)                                               /*!< CLK_T::DMICSEL: DMIC0SEL Position      */
#define CLK_DMICSEL_DMIC0SEL_Msk         (0x7ul << CLK_DMICSEL_DMIC0SEL_Pos)               /*!< CLK_T::DMICSEL: DMIC0SEL Mask          */

#define CLK_DMICSEL_VAD0SEL_Pos          (4)                                               /*!< CLK_T::DMICSEL: VAD0SEL Position       */
#define CLK_DMICSEL_VAD0SEL_Msk          (0x3ul << CLK_DMICSEL_VAD0SEL_Pos)                /*!< CLK_T::DMICSEL: VAD0SEL Mask           */

#define CLK_EADCSEL_EADC0SEL_Pos         (0)                                               /*!< CLK_T::EADCSEL: EADC0SEL Position      */
#define CLK_EADCSEL_EADC0SEL_Msk         (0x3ul << CLK_EADCSEL_EADC0SEL_Pos)               /*!< CLK_T::EADCSEL: EADC0SEL Mask          */

#define CLK_EPWMSEL_EPWM0SEL_Pos         (0)                                               /*!< CLK_T::EPWMSEL: EPWM0SEL Position      */
#define CLK_EPWMSEL_EPWM0SEL_Msk         (0x1ul << CLK_EPWMSEL_EPWM0SEL_Pos)               /*!< CLK_T::EPWMSEL: EPWM0SEL Mask          */

#define CLK_EPWMSEL_EPWM1SEL_Pos         (4)                                               /*!< CLK_T::EPWMSEL: EPWM1SEL Position      */
#define CLK_EPWMSEL_EPWM1SEL_Msk         (0x1ul << CLK_EPWMSEL_EPWM1SEL_Pos)               /*!< CLK_T::EPWMSEL: EPWM1SEL Mask          */

#define CLK_FMCSEL_FMC0SEL_Pos           (0)                                               /*!< CLK_T::FMCSEL: FMC0SEL Position        */
#define CLK_FMCSEL_FMC0SEL_Msk           (0x1ul << CLK_FMCSEL_FMC0SEL_Pos)                 /*!< CLK_T::FMCSEL: FMC0SEL Mask            */

#define CLK_I2SSEL_I2S0SEL_Pos           (0)                                               /*!< CLK_T::I2SSEL: I2S0SEL Position        */
#define CLK_I2SSEL_I2S0SEL_Msk           (0x7ul << CLK_I2SSEL_I2S0SEL_Pos)                 /*!< CLK_T::I2SSEL: I2S0SEL Mask            */

#define CLK_I2SSEL_I2S1SEL_Pos           (4)                                               /*!< CLK_T::I2SSEL: I2S1SEL Position        */
#define CLK_I2SSEL_I2S1SEL_Msk           (0x7ul << CLK_I2SSEL_I2S1SEL_Pos)                 /*!< CLK_T::I2SSEL: I2S1SEL Mask            */

#define CLK_I3CSEL_I3C0SEL_Pos           (0)                                               /*!< CLK_T::I3CSEL: I3C0SEL Position        */
#define CLK_I3CSEL_I3C0SEL_Msk           (0x1ul << CLK_I3CSEL_I3C0SEL_Pos)                 /*!< CLK_T::I3CSEL: I3C0SEL Mask            */

#define CLK_KPISEL_KPI0SEL_Pos           (0)                                               /*!< CLK_T::KPISEL: KPI0SEL Position        */
#define CLK_KPISEL_KPI0SEL_Msk           (0x3ul << CLK_KPISEL_KPI0SEL_Pos)                 /*!< CLK_T::KPISEL: KPI0SEL Mask            */

#define CLK_LPADCSEL_LPADC0SEL_Pos       (0)                                               /*!< CLK_T::LPADCSEL: LPADC0SEL Position    */
#define CLK_LPADCSEL_LPADC0SEL_Msk       (0x3ul << CLK_LPADCSEL_LPADC0SEL_Pos)             /*!< CLK_T::LPADCSEL: LPADC0SEL Mask        */

#define CLK_LPSPISEL_LPSPI0SEL_Pos       (0)                                               /*!< CLK_T::LPSPISEL: LPSPI0SEL Position    */
#define CLK_LPSPISEL_LPSPI0SEL_Msk       (0x3ul << CLK_LPSPISEL_LPSPI0SEL_Pos)             /*!< CLK_T::LPSPISEL: LPSPI0SEL Mask        */

#define CLK_LPTMRSEL_LPTMR0SEL_Pos       (0)                                               /*!< CLK_T::LPTMRSEL: LPTMR0SEL Position    */
#define CLK_LPTMRSEL_LPTMR0SEL_Msk       (0x7ul << CLK_LPTMRSEL_LPTMR0SEL_Pos)             /*!< CLK_T::LPTMRSEL: LPTMR0SEL Mask        */

#define CLK_LPTMRSEL_LPTMR1SEL_Pos       (4)                                               /*!< CLK_T::LPTMRSEL: LPTMR1SEL Position    */
#define CLK_LPTMRSEL_LPTMR1SEL_Msk       (0x7ul << CLK_LPTMRSEL_LPTMR1SEL_Pos)             /*!< CLK_T::LPTMRSEL: LPTMR1SEL Mask        */

#define CLK_LPUARTSEL_LPUART0SEL_Pos     (0)                                               /*!< CLK_T::LPUARTSEL: LPUART0SEL Position  */
#define CLK_LPUARTSEL_LPUART0SEL_Msk     (0x3ul << CLK_LPUARTSEL_LPUART0SEL_Pos)           /*!< CLK_T::LPUARTSEL: LPUART0SEL Mask      */

#define CLK_PSIOSEL_PSIO0SEL_Pos         (0)                                               /*!< CLK_T::PSIOSEL: PSIO0SEL Position      */
#define CLK_PSIOSEL_PSIO0SEL_Msk         (0x7ul << CLK_PSIOSEL_PSIO0SEL_Pos)               /*!< CLK_T::PSIOSEL: PSIO0SEL Mask          */

#define CLK_QSPISEL_QSPI0SEL_Pos         (0)                                               /*!< CLK_T::QSPISEL: QSPI0SEL Position      */
#define CLK_QSPISEL_QSPI0SEL_Msk         (0x7ul << CLK_QSPISEL_QSPI0SEL_Pos)               /*!< CLK_T::QSPISEL: QSPI0SEL Mask          */

#define CLK_QSPISEL_QSPI1SEL_Pos         (4)                                               /*!< CLK_T::QSPISEL: QSPI1SEL Position      */
#define CLK_QSPISEL_QSPI1SEL_Msk         (0x7ul << CLK_QSPISEL_QSPI1SEL_Pos)               /*!< CLK_T::QSPISEL: QSPI1SEL Mask          */

#define CLK_SCSEL_SC0SEL_Pos             (0)                                               /*!< CLK_T::SCSEL: SC0SEL Position          */
#define CLK_SCSEL_SC0SEL_Msk             (0x7ul << CLK_SCSEL_SC0SEL_Pos)                   /*!< CLK_T::SCSEL: SC0SEL Mask              */

#define CLK_SCSEL_SC1SEL_Pos             (4)                                               /*!< CLK_T::SCSEL: SC1SEL Position          */
#define CLK_SCSEL_SC1SEL_Msk             (0x7ul << CLK_SCSEL_SC1SEL_Pos)                   /*!< CLK_T::SCSEL: SC1SEL Mask              */

#define CLK_SCSEL_SC2SEL_Pos             (8)                                               /*!< CLK_T::SCSEL: SC2SEL Position          */
#define CLK_SCSEL_SC2SEL_Msk             (0x7ul << CLK_SCSEL_SC2SEL_Pos)                   /*!< CLK_T::SCSEL: SC2SEL Mask              */

#define CLK_SDHSEL_SDH0SEL_Pos           (0)                                               /*!< CLK_T::SDHSEL: SDH0SEL Position        */
#define CLK_SDHSEL_SDH0SEL_Msk           (0x7ul << CLK_SDHSEL_SDH0SEL_Pos)                 /*!< CLK_T::SDHSEL: SDH0SEL Mask            */

#define CLK_SDHSEL_SDH1SEL_Pos           (4)                                               /*!< CLK_T::SDHSEL: SDH1SEL Position        */
#define CLK_SDHSEL_SDH1SEL_Msk           (0x7ul << CLK_SDHSEL_SDH1SEL_Pos)                 /*!< CLK_T::SDHSEL: SDH1SEL Mask            */

#define CLK_SPISEL_SPI0SEL_Pos           (0)                                               /*!< CLK_T::SPISEL: SPI0SEL Position        */
#define CLK_SPISEL_SPI0SEL_Msk           (0x7ul << CLK_SPISEL_SPI0SEL_Pos)                 /*!< CLK_T::SPISEL: SPI0SEL Mask            */

#define CLK_SPISEL_SPI1SEL_Pos           (4)                                               /*!< CLK_T::SPISEL: SPI1SEL Position        */
#define CLK_SPISEL_SPI1SEL_Msk           (0x7ul << CLK_SPISEL_SPI1SEL_Pos)                 /*!< CLK_T::SPISEL: SPI1SEL Mask            */

#define CLK_SPISEL_SPI2SEL_Pos           (8)                                               /*!< CLK_T::SPISEL: SPI2SEL Position        */
#define CLK_SPISEL_SPI2SEL_Msk           (0x7ul << CLK_SPISEL_SPI2SEL_Pos)                 /*!< CLK_T::SPISEL: SPI2SEL Mask            */

#define CLK_SPISEL_SPI3SEL_Pos           (12)                                              /*!< CLK_T::SPISEL: SPI3SEL Position        */
#define CLK_SPISEL_SPI3SEL_Msk           (0x7ul << CLK_SPISEL_SPI3SEL_Pos)                 /*!< CLK_T::SPISEL: SPI3SEL Mask            */

#define CLK_STSEL_ST0SEL_Pos             (0)                                               /*!< CLK_T::STSEL: ST0SEL Position          */
#define CLK_STSEL_ST0SEL_Msk             (0x3ul << CLK_STSEL_ST0SEL_Pos)                   /*!< CLK_T::STSEL: ST0SEL Mask              */

#define CLK_TMRSEL_TMR0SEL_Pos           (0)                                               /*!< CLK_T::TMRSEL: TMR0SEL Position        */
#define CLK_TMRSEL_TMR0SEL_Msk           (0x7ul << CLK_TMRSEL_TMR0SEL_Pos)                 /*!< CLK_T::TMRSEL: TMR0SEL Mask            */

#define CLK_TMRSEL_TMR1SEL_Pos           (4)                                               /*!< CLK_T::TMRSEL: TMR1SEL Position        */
#define CLK_TMRSEL_TMR1SEL_Msk           (0x7ul << CLK_TMRSEL_TMR1SEL_Pos)                 /*!< CLK_T::TMRSEL: TMR1SEL Mask            */

#define CLK_TMRSEL_TMR2SEL_Pos           (8)                                               /*!< CLK_T::TMRSEL: TMR2SEL Position        */
#define CLK_TMRSEL_TMR2SEL_Msk           (0x7ul << CLK_TMRSEL_TMR2SEL_Pos)                 /*!< CLK_T::TMRSEL: TMR2SEL Mask            */

#define CLK_TMRSEL_TMR3SEL_Pos           (12)                                              /*!< CLK_T::TMRSEL: TMR3SEL Position        */
#define CLK_TMRSEL_TMR3SEL_Msk           (0x7ul << CLK_TMRSEL_TMR3SEL_Pos)                 /*!< CLK_T::TMRSEL: TMR3SEL Mask            */

#define CLK_TTMRSEL_TTMR0SEL_Pos         (0)                                               /*!< CLK_T::TTMRSEL: TTMR0SEL Position      */
#define CLK_TTMRSEL_TTMR0SEL_Msk         (0x7ul << CLK_TTMRSEL_TTMR0SEL_Pos)               /*!< CLK_T::TTMRSEL: TTMR0SEL Mask          */

#define CLK_TTMRSEL_TTMR1SEL_Pos         (4)                                               /*!< CLK_T::TTMRSEL: TTMR1SEL Position      */
#define CLK_TTMRSEL_TTMR1SEL_Msk         (0x7ul << CLK_TTMRSEL_TTMR1SEL_Pos)               /*!< CLK_T::TTMRSEL: TTMR1SEL Mask          */

#define CLK_UARTSEL0_UART0SEL_Pos        (0)                                               /*!< CLK_T::UARTSEL0: UART0SEL Position     */
#define CLK_UARTSEL0_UART0SEL_Msk        (0x7ul << CLK_UARTSEL0_UART0SEL_Pos)              /*!< CLK_T::UARTSEL0: UART0SEL Mask         */

#define CLK_UARTSEL0_UART1SEL_Pos        (4)                                               /*!< CLK_T::UARTSEL0: UART1SEL Position     */
#define CLK_UARTSEL0_UART1SEL_Msk        (0x7ul << CLK_UARTSEL0_UART1SEL_Pos)              /*!< CLK_T::UARTSEL0: UART1SEL Mask         */

#define CLK_UARTSEL0_UART2SEL_Pos        (8)                                               /*!< CLK_T::UARTSEL0: UART2SEL Position     */
#define CLK_UARTSEL0_UART2SEL_Msk        (0x7ul << CLK_UARTSEL0_UART2SEL_Pos)              /*!< CLK_T::UARTSEL0: UART2SEL Mask         */

#define CLK_UARTSEL0_UART3SEL_Pos        (12)                                              /*!< CLK_T::UARTSEL0: UART3SEL Position     */
#define CLK_UARTSEL0_UART3SEL_Msk        (0x7ul << CLK_UARTSEL0_UART3SEL_Pos)              /*!< CLK_T::UARTSEL0: UART3SEL Mask         */

#define CLK_UARTSEL0_UART4SEL_Pos        (16)                                              /*!< CLK_T::UARTSEL0: UART4SEL Position     */
#define CLK_UARTSEL0_UART4SEL_Msk        (0x7ul << CLK_UARTSEL0_UART4SEL_Pos)              /*!< CLK_T::UARTSEL0: UART4SEL Mask         */

#define CLK_UARTSEL0_UART5SEL_Pos        (20)                                              /*!< CLK_T::UARTSEL0: UART5SEL Position     */
#define CLK_UARTSEL0_UART5SEL_Msk        (0x7ul << CLK_UARTSEL0_UART5SEL_Pos)              /*!< CLK_T::UARTSEL0: UART5SEL Mask         */

#define CLK_UARTSEL0_UART6SEL_Pos        (24)                                              /*!< CLK_T::UARTSEL0: UART6SEL Position     */
#define CLK_UARTSEL0_UART6SEL_Msk        (0x7ul << CLK_UARTSEL0_UART6SEL_Pos)              /*!< CLK_T::UARTSEL0: UART6SEL Mask         */

#define CLK_UARTSEL0_UART7SEL_Pos        (28)                                              /*!< CLK_T::UARTSEL0: UART7SEL Position     */
#define CLK_UARTSEL0_UART7SEL_Msk        (0x7ul << CLK_UARTSEL0_UART7SEL_Pos)              /*!< CLK_T::UARTSEL0: UART7SEL Mask         */

#define CLK_UARTSEL1_UART8SEL_Pos        (0)                                               /*!< CLK_T::UARTSEL1: UART8SEL Position     */
#define CLK_UARTSEL1_UART8SEL_Msk        (0x7ul << CLK_UARTSEL1_UART8SEL_Pos)              /*!< CLK_T::UARTSEL1: UART8SEL Mask         */

#define CLK_UARTSEL1_UART9SEL_Pos        (4)                                               /*!< CLK_T::UARTSEL1: UART9SEL Position     */
#define CLK_UARTSEL1_UART9SEL_Msk        (0x7ul << CLK_UARTSEL1_UART9SEL_Pos)              /*!< CLK_T::UARTSEL1: UART9SEL Mask         */

#define CLK_USBSEL_USBSEL_Pos            (0)                                               /*!< CLK_T::USBSEL: USBSEL Position         */
#define CLK_USBSEL_USBSEL_Msk            (0x1ul << CLK_USBSEL_USBSEL_Pos)                  /*!< CLK_T::USBSEL: USBSEL Mask             */

#define CLK_WDTSEL_WDT0SEL_Pos           (0)                                               /*!< CLK_T::WDTSEL: WDT0SEL Position        */
#define CLK_WDTSEL_WDT0SEL_Msk           (0x1ul << CLK_WDTSEL_WDT0SEL_Pos)                 /*!< CLK_T::WDTSEL: WDT0SEL Mask            */

#define CLK_WDTSEL_WDT1SEL_Pos           (4)                                               /*!< CLK_T::WDTSEL: WDT1SEL Position        */
#define CLK_WDTSEL_WDT1SEL_Msk           (0x1ul << CLK_WDTSEL_WDT1SEL_Pos)                 /*!< CLK_T::WDTSEL: WDT1SEL Mask            */

#define CLK_DLLSEL_DLL0SEL_Pos           (0)                                               /*!< CLK_T::DLLSEL: DLL0SEL Position        */
#define CLK_DLLSEL_DLL0SEL_Msk           (0x1ul << CLK_DLLSEL_DLL0SEL_Pos)                 /*!< CLK_T::DLLSEL: DLL0SEL Mask            */

#define CLK_WWDTSEL_WWDT0SEL_Pos         (0)                                               /*!< CLK_T::WWDTSEL: WWDT0SEL Position      */
#define CLK_WWDTSEL_WWDT0SEL_Msk         (0x1ul << CLK_WWDTSEL_WWDT0SEL_Pos)               /*!< CLK_T::WWDTSEL: WWDT0SEL Mask          */

#define CLK_WWDTSEL_WWDT1SEL_Pos         (4)                                               /*!< CLK_T::WWDTSEL: WWDT1SEL Position      */
#define CLK_WWDTSEL_WWDT1SEL_Msk         (0x1ul << CLK_WWDTSEL_WWDT1SEL_Pos)               /*!< CLK_T::WWDTSEL: WWDT1SEL Mask          */

#define CLK_SCLKDIV_SCLKDIV_Pos          (0)                                               /*!< CLK_T::SCLKDIV: SCLKDIV Position       */
#define CLK_SCLKDIV_SCLKDIV_Msk          (0xful << CLK_SCLKDIV_SCLKDIV_Pos)                /*!< CLK_T::SCLKDIV: SCLKDIV Mask           */

#define CLK_HCLKDIV_HCLK2DIV_Pos         (8)                                               /*!< CLK_T::HCLKDIV: HCLK2DIV Position      */
#define CLK_HCLKDIV_HCLK2DIV_Msk         (0xful << CLK_HCLKDIV_HCLK2DIV_Pos)               /*!< CLK_T::HCLKDIV: HCLK2DIV Mask          */

#define CLK_PCLKDIV_PCLK0DIV_Pos         (0)                                               /*!< CLK_T::PCLKDIV: PCLK0DIV Position      */
#define CLK_PCLKDIV_PCLK0DIV_Msk         (0xful << CLK_PCLKDIV_PCLK0DIV_Pos)               /*!< CLK_T::PCLKDIV: PCLK0DIV Mask          */

#define CLK_PCLKDIV_PCLK1DIV_Pos         (4)                                               /*!< CLK_T::PCLKDIV: PCLK1DIV Position      */
#define CLK_PCLKDIV_PCLK1DIV_Msk         (0xful << CLK_PCLKDIV_PCLK1DIV_Pos)               /*!< CLK_T::PCLKDIV: PCLK1DIV Mask          */

#define CLK_PCLKDIV_PCLK2DIV_Pos         (8)                                               /*!< CLK_T::PCLKDIV: PCLK2DIV Position      */
#define CLK_PCLKDIV_PCLK2DIV_Msk         (0xful << CLK_PCLKDIV_PCLK2DIV_Pos)               /*!< CLK_T::PCLKDIV: PCLK2DIV Mask          */

#define CLK_PCLKDIV_PCLK3DIV_Pos         (12)                                              /*!< CLK_T::PCLKDIV: PCLK3DIV Position      */
#define CLK_PCLKDIV_PCLK3DIV_Msk         (0xful << CLK_PCLKDIV_PCLK3DIV_Pos)               /*!< CLK_T::PCLKDIV: PCLK3DIV Mask          */

#define CLK_PCLKDIV_PCLK4DIV_Pos         (16)                                              /*!< CLK_T::PCLKDIV: PCLK4DIV Position      */
#define CLK_PCLKDIV_PCLK4DIV_Msk         (0xful << CLK_PCLKDIV_PCLK4DIV_Pos)               /*!< CLK_T::PCLKDIV: PCLK4DIV Mask          */

#define CLK_CANFDDIV_CANFD0DIV_Pos       (0)                                               /*!< CLK_T::CANFDDIV: CANFD0DIV Position    */
#define CLK_CANFDDIV_CANFD0DIV_Msk       (0xfful << CLK_CANFDDIV_CANFD0DIV_Pos)            /*!< CLK_T::CANFDDIV: CANFD0DIV Mask        */

#define CLK_CANFDDIV_CANFD1DIV_Pos       (8)                                               /*!< CLK_T::CANFDDIV: CANFD1DIV Position    */
#define CLK_CANFDDIV_CANFD1DIV_Msk       (0xfful << CLK_CANFDDIV_CANFD1DIV_Pos)            /*!< CLK_T::CANFDDIV: CANFD1DIV Mask        */

#define CLK_DMICDIV_DMIC0DIV_Pos         (0)                                               /*!< CLK_T::DMICDIV: DMIC0DIV Position      */
#define CLK_DMICDIV_DMIC0DIV_Msk         (0xfful << CLK_DMICDIV_DMIC0DIV_Pos)              /*!< CLK_T::DMICDIV: DMIC0DIV Mask          */

#define CLK_EADCDIV_EADC0DIV_Pos         (0)                                               /*!< CLK_T::EADCDIV: EADC0DIV Position      */
#define CLK_EADCDIV_EADC0DIV_Msk         (0xfful << CLK_EADCDIV_EADC0DIV_Pos)              /*!< CLK_T::EADCDIV: EADC0DIV Mask          */

#define CLK_I2SDIV_I2S0DIV_Pos           (0)                                               /*!< CLK_T::I2SDIV: I2S0DIV Position        */
#define CLK_I2SDIV_I2S0DIV_Msk           (0xfful << CLK_I2SDIV_I2S0DIV_Pos)                /*!< CLK_T::I2SDIV: I2S0DIV Mask            */

#define CLK_I2SDIV_I2S1DIV_Pos           (8)                                               /*!< CLK_T::I2SDIV: I2S1DIV Position        */
#define CLK_I2SDIV_I2S1DIV_Msk           (0xfful << CLK_I2SDIV_I2S1DIV_Pos)                /*!< CLK_T::I2SDIV: I2S1DIV Mask            */

#define CLK_KPIDIV_KPI0DIV_Pos           (0)                                               /*!< CLK_T::KPIDIV: KPI0DIV Position        */
#define CLK_KPIDIV_KPI0DIV_Msk           (0xfful << CLK_KPIDIV_KPI0DIV_Pos)                /*!< CLK_T::KPIDIV: KPI0DIV Mask            */

#define CLK_LPADCDIV_LPADC0DIV_Pos       (0)                                               /*!< CLK_T::LPADCDIV: LPADC0DIV Position    */
#define CLK_LPADCDIV_LPADC0DIV_Msk       (0xfful << CLK_LPADCDIV_LPADC0DIV_Pos)            /*!< CLK_T::LPADCDIV: LPADC0DIV Mask        */

#define CLK_LPUARTDIV_LPUART0DIV_Pos     (0)                                               /*!< CLK_T::LPUARTDIV: LPUART0DIV Position  */
#define CLK_LPUARTDIV_LPUART0DIV_Msk     (0xful << CLK_LPUARTDIV_LPUART0DIV_Pos)           /*!< CLK_T::LPUARTDIV: LPUART0DIV Mask      */

#define CLK_PSIODIV_PSIO0DIV_Pos         (0)                                               /*!< CLK_T::PSIODIV: PSIO0DIV Position      */
#define CLK_PSIODIV_PSIO0DIV_Msk         (0xfful << CLK_PSIODIV_PSIO0DIV_Pos)              /*!< CLK_T::PSIODIV: PSIO0DIV Mask          */

#define CLK_SCDIV_SC0DIV_Pos             (0)                                               /*!< CLK_T::SCDIV: SC0DIV Position          */
#define CLK_SCDIV_SC0DIV_Msk             (0xfful << CLK_SCDIV_SC0DIV_Pos)                  /*!< CLK_T::SCDIV: SC0DIV Mask              */

#define CLK_SCDIV_SC1DIV_Pos             (8)                                               /*!< CLK_T::SCDIV: SC1DIV Position          */
#define CLK_SCDIV_SC1DIV_Msk             (0xfful << CLK_SCDIV_SC1DIV_Pos)                  /*!< CLK_T::SCDIV: SC1DIV Mask              */

#define CLK_SCDIV_SC2DIV_Pos             (16)                                              /*!< CLK_T::SCDIV: SC2DIV Position          */
#define CLK_SCDIV_SC2DIV_Msk             (0xfful << CLK_SCDIV_SC2DIV_Pos)                  /*!< CLK_T::SCDIV: SC2DIV Mask              */

#define CLK_SDHDIV_SDH0DIV_Pos           (0)                                               /*!< CLK_T::SDHDIV: SDH0DIV Position        */
#define CLK_SDHDIV_SDH0DIV_Msk           (0xfful << CLK_SDHDIV_SDH0DIV_Pos)                /*!< CLK_T::SDHDIV: SDH0DIV Mask            */

#define CLK_SDHDIV_SDH1DIV_Pos           (8)                                               /*!< CLK_T::SDHDIV: SDH1DIV Position        */
#define CLK_SDHDIV_SDH1DIV_Msk           (0xfful << CLK_SDHDIV_SDH1DIV_Pos)                /*!< CLK_T::SDHDIV: SDH1DIV Mask            */

#define CLK_STDIV_ST0DIV_Pos             (0)                                               /*!< CLK_T::STDIV: ST0DIV Position          */
#define CLK_STDIV_ST0DIV_Msk             (0xfful << CLK_STDIV_ST0DIV_Pos)                  /*!< CLK_T::STDIV: ST0DIV Mask              */

#define CLK_UARTDIV0_UART0DIV_Pos        (0)                                               /*!< CLK_T::UARTDIV0: UART0DIV Position     */
#define CLK_UARTDIV0_UART0DIV_Msk        (0xful << CLK_UARTDIV0_UART0DIV_Pos)              /*!< CLK_T::UARTDIV0: UART0DIV Mask         */

#define CLK_UARTDIV0_UART1DIV_Pos        (4)                                               /*!< CLK_T::UARTDIV0: UART1DIV Position     */
#define CLK_UARTDIV0_UART1DIV_Msk        (0xful << CLK_UARTDIV0_UART1DIV_Pos)              /*!< CLK_T::UARTDIV0: UART1DIV Mask         */

#define CLK_UARTDIV0_UART2DIV_Pos        (8)                                               /*!< CLK_T::UARTDIV0: UART2DIV Position     */
#define CLK_UARTDIV0_UART2DIV_Msk        (0xful << CLK_UARTDIV0_UART2DIV_Pos)              /*!< CLK_T::UARTDIV0: UART2DIV Mask         */

#define CLK_UARTDIV0_UART3DIV_Pos        (12)                                              /*!< CLK_T::UARTDIV0: UART3DIV Position     */
#define CLK_UARTDIV0_UART3DIV_Msk        (0xful << CLK_UARTDIV0_UART3DIV_Pos)              /*!< CLK_T::UARTDIV0: UART3DIV Mask         */

#define CLK_UARTDIV0_UART4DIV_Pos        (16)                                              /*!< CLK_T::UARTDIV0: UART4DIV Position     */
#define CLK_UARTDIV0_UART4DIV_Msk        (0xful << CLK_UARTDIV0_UART4DIV_Pos)              /*!< CLK_T::UARTDIV0: UART4DIV Mask         */

#define CLK_UARTDIV0_UART5DIV_Pos        (20)                                              /*!< CLK_T::UARTDIV0: UART5DIV Position     */
#define CLK_UARTDIV0_UART5DIV_Msk        (0xful << CLK_UARTDIV0_UART5DIV_Pos)              /*!< CLK_T::UARTDIV0: UART5DIV Mask         */

#define CLK_UARTDIV0_UART6DIV_Pos        (24)                                              /*!< CLK_T::UARTDIV0: UART6DIV Position     */
#define CLK_UARTDIV0_UART6DIV_Msk        (0xful << CLK_UARTDIV0_UART6DIV_Pos)              /*!< CLK_T::UARTDIV0: UART6DIV Mask         */

#define CLK_UARTDIV0_UART7DIV_Pos        (28)                                              /*!< CLK_T::UARTDIV0: UART7DIV Position     */
#define CLK_UARTDIV0_UART7DIV_Msk        (0xful << CLK_UARTDIV0_UART7DIV_Pos)              /*!< CLK_T::UARTDIV0: UART7DIV Mask         */

#define CLK_UARTDIV1_UART8DIV_Pos        (0)                                               /*!< CLK_T::UARTDIV1: UART8DIV Position     */
#define CLK_UARTDIV1_UART8DIV_Msk        (0xful << CLK_UARTDIV1_UART8DIV_Pos)              /*!< CLK_T::UARTDIV1: UART8DIV Mask         */

#define CLK_UARTDIV1_UART9DIV_Pos        (4)                                               /*!< CLK_T::UARTDIV1: UART9DIV Position     */
#define CLK_UARTDIV1_UART9DIV_Msk        (0xful << CLK_UARTDIV1_UART9DIV_Pos)              /*!< CLK_T::UARTDIV1: UART9DIV Mask         */

#define CLK_USBDIV_USBDIV_Pos            (0)                                               /*!< CLK_T::USBDIV: USBDIV Position         */
#define CLK_USBDIV_USBDIV_Msk            (0xful << CLK_USBDIV_USBDIV_Pos)                  /*!< CLK_T::USBDIV: USBDIV Mask             */

#define CLK_VSENSEDIV_VSENSEDIV_Pos      (0)                                               /*!< CLK_T::VSENSEDIV: VSENSEDIV Position   */
#define CLK_VSENSEDIV_VSENSEDIV_Msk      (0xful << CLK_VSENSEDIV_VSENSEDIV_Pos)            /*!< CLK_T::VSENSEDIV: VSENSEDIV Mask       */

/** @} CLK_CONST */
/** @} end of CLK register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __CLK_REG_H__ */
