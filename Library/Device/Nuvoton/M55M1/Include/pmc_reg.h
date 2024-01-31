/**************************************************************************//**
 * @file     pmc_reg.h
 * @version  V1.00
 * @brief    PMC register definition header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __PMC_REG_H__
#define __PMC_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup PMC Power Manager (PMC)
    Memory Mapped Structure for PMC Controller
    @{
*/

typedef struct
{
    /**
     * @var PMC_T::PWRCTL
     * Offset: 0x00  System Power-down Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |D0PGEN    |Power Domain 0 Power Gating Enable Bit (Write Protect)
     * |        |          |This bit control power domain 0 (Performance Domain) power gating function when system enter Power-down mode(CPU execute WFI/WFE instruction and SLEEPDEEP(SCR[2]) is 1).
     * |        |          |0 = Power domain 0 keep supply.
     * |        |          |1 = Power domain 0 stop supply.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: This bit is not retained when D4 power is turned off.
     * |[1]     |D1PGEN    |Power Domain 1 Power Gating Enable Bit (Write Protect)
     * |        |          |This bit control power domain 1 (Base Domain) power gating function when system enter Power-down mode(CPU execute WFI/WFE instruction and SLEEPDEEP(SCR[2]) is 1).
     * |        |          |0 = Power domain 1 keep supply.
     * |        |          |1 = Power domain 1 stop supply.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: If D0PGEN(PMC_PWRCTL[0]) is 0, the power domain 1 always keep supply, even if D1PGEN is 1.
     * |        |          |Note 4: This bit is not retained when D4 power is turned off.
     * |[2]     |D2PGEN    |Power Domain 2 Power Gating Enable Bit (Write Protect)
     * |        |          |This bit control power domain 2 (Low Power Domain) power gating function when system enter Power-down mode(CPU execute WFI/WFE instruction and SLEEPDEEP(SCR[2]) is 1).
     * |        |          |0 = Power domain 2 kept supply.
     * |        |          |1 = Power domain 2 stopped supply.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: If D1PGEN(PMC_PWRCTL[1]) is 0, the power domain 2 always keep supply, even if D2PGEN is 1.
     * |        |          |Note 4: This bit is not retained when D4 power is turned off.
     * |[3]     |D3PGEN    |Power Domain 3 Power Gating Enable Bit (Write Protect)
     * |        |          |This bit control power domain 3 (Standby Power Domain) power gating function when system enters Power-down mode(CPU execute WFI/WFE instruction and SLEEPDEEP(SCR[2]) is 1).
     * |        |          |0 = Power domain 3 keep supply.
     * |        |          |1 = Power domain 3 stop supply.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: If D2PGEN(PMC_PWRCTL[2]) is 0, The power domain 3 always keep supply, even if D3PGEN is 1.
     * |        |          |Note 4: This bit is not retained when D4 power is turned off.
     * |[8]     |PDEN      |System Power-down Enable (Write Protect)
     * |        |          |When this bit is set to 1, Power-down mode is enabled and the chip keeps active till the CPU sleep mode is also active and then the chip enters Power-down mode.
     * |        |          |When chip wakes up from Power-down mode, this bit is auto cleared
     * |        |          |Users need to set this bit again for next Power-down.
     * |        |          |In Power-down mode, HXT, HIRC48M, APLL0, APLL1 and system clock will be disabled and ignored the clock source selection
     * |        |          |The clocks of peripheral are not controlled by Power-down mode, if the peripheral clock source is from LXT, LIRC, MIRC or HIRC.
     * |        |          |0 = Chip operating normally or chip in idle mode because of WFI command.
     * |        |          |1 = Chip waits CPU sleep command WFI and then enters Power-down mode.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: System tick interrupt TICKINT(SYS_CTRL[1]) has to be disabled before entering to Power-down mode, to avoid system tick interrupt may influence system not entering power-down mode and keep operation.
     * |        |          |Note 3: This bit is not not retained when D0 power is turned off.
     * |[10]    |VDROPEN   |Voltage Regulator Output Drop Enable Bit (Write Protect)
     * |        |          |When this bit is set to 1, voltage regulator output drop to 0.7v in Power-down mode.
     * |        |          |0 = Regulator voltage auto drop function Disabled.
     * |        |          |1 = Regulator voltage auto drop function Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: This bit is not not retained when D4 power is turned off.
     * |[11]    |FWEN      |Fast Wake-up Enable Bit (Write Protect)
     * |        |          |When this bit is set to 1, wake-up will be speed up, but power consumption is increase about 30uA in Power-down mode.
     * |        |          |0 = Regulator voltage auto drop function Disabled.
     * |        |          |1 = Regulator voltage auto drop function Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: This bit is not not retained when D4 power is turned off.
     * |[12]    |PRSTDBEN  |Pin Reset De-Bounce Enable Bit (Write Protect)
     * |        |          |When this bit is set to 1, pin reset clock base de-bounce circuit is enable in Power-down mode.
     * |        |          |0 = Pin reset de-bounce circuit Disable, and de-bounce time about 300ns.
     * |        |          |1 = Pin reset de-bounce circuit Enable, and de-bounce time about 32us.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not not retained when D2 power is turned off.
     * |        |          |Note 3: If D2PGEN(PMC_PWRCTL[2]) is 1, The pin reset de-bounce circuit always keep disable, even if PRSTDBEN is 1.
     * |        |          |Note 4: If VDROPEN(PMC_PWRCTL[10]) is 1, The pin reset de-bounce circuit always keep disable, even if PRSTDBEN is 1.
     * |[31]    |WRBUSY    |Write Busy Flag (Read Only)
     * |        |          |If PMC_PWRCTL be written, this bit be asserted automatic by hardware, and be de-asserted when write procedure is finished.
     * |        |          |0 = PMC_PWRCTL register is ready for write operation.
     * |        |          |1 = PMC_PWRCTL register is busy on the last write operation. Other write operations are ignored.
     * @var PMC_T::INTEN
     * Offset: 0x04  PMC Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PDWKIEN   |Power-down Mode Wake-up Interrupt Enable Bit (Write Protect)
     * |        |          |0 = Power-down mode wake-up interrupt Disabled.
     * |        |          |1 = Power-down mode wake-up interrupt Enabled.
     * |        |          |Note 1: The PMC interrupt will occur when both PDWKIF(PMC_INTSTS[0] and PDWKIEN(PMC_INTEN[0]) are 1.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: This bit is not retained when D4 power is turned off.
     * |[1]     |STMRWKIEN |Standby Timer Wake-up Interrupt Enable Bit (Write Protect)
     * |        |          |0 = High Voltage Timer wake-up interrupt Disabled.
     * |        |          |1 = High Voltage Timer wake-up interrupt Enabled.
     * |        |          |Note 1: The PMC interrupt will occur when both STMRWKIF(PMC_INTSTS[1] and STMRWKIEN(PMC_INTEN[1]) are 1.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: This bit is not retained when D4 power is turned off.
     * |[8]     |PIN0WKIEN |Pin 0 Wake-up Interrupt Enable Bit (Write Protect)
     * |        |          |0 = Pin 0 wake-up interrupt Disabled.
     * |        |          |1 = Pin 0 wake-up interrupt Enabled.
     * |        |          |Note 1: The PMC interrupt will occur when both PIN0WKIF(PMC_INTSTS[8] and PIN0WKIEN(PMC_INTEN[8]) are 1.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: This bit is not retained when D4 power is turned off.
     * |[9]     |PIN1WKIEN |Pin 1 Wake-up Interrupt Enable Bit (Write Protect)
     * |        |          |0 = Pin 1 wake-up interrupt Disabled.
     * |        |          |1 = Pin 1 wake-up interrupt Enabled.
     * |        |          |Note 1: The PMC interrupt will occur when both PIN1WKIF(PMC_INTSTS[9] and PIN1WKIEN(PMC_INTEN[9]) are 1.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: This bit is not retained when D4 power is turned off.
     * |[10]    |PIN2WKIEN |Pin 2 Wake-up Interrupt Enable Bit (Write Protect)
     * |        |          |0 = Pin 2 wake-up interrupt Disabled.
     * |        |          |1 = Pin 2 wake-up interrupt Enabled.
     * |        |          |Note 1: The PMC interrupt will occur when both PIN2WKIF(PMC_INTSTS[10] and PIN2WKIEN(PMC_INTEN[10]) are 1.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: This bit is not retained when D4 power is turned off.
     * |[11]    |PIN3WKIEN |Pin 3 Wake-up Interrupt Enable Bit (Write Protect)
     * |        |          |0 = Pin 3 wake-up interrupt Disabled.
     * |        |          |1 = Pin 3 wake-up interrupt Enabled.
     * |        |          |Note 1: The PMC interrupt will occur when both PIN3WKIF(PMC_INTSTS[11] and PIN3WKIEN(PMC_INTEN[11]) are 1.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: This bit is not retained when D4 power is turned off.
     * |[12]    |PIN4WKIEN |Pin 4 Wake-up Interrupt Enable Bit (Write Protect)
     * |        |          |0 = Pin 4 wake-up interrupt Disabled.
     * |        |          |1 = Pin 4 wake-up interrupt Enabled.
     * |        |          |Note 1: The PMC interrupt will occur when both PIN4WKIF(PMC_INTSTS[12] and PIN4WKIEN(PMC_INTEN[12]) are 1.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: This bit is not retained when D4 power is turned off.
     * |[13]    |PIN5WKIEN |Pin 5 Wake-up Interrupt Enable Bit (Write Protect)
     * |        |          |0 = Pin 5 wake-up interrupt Disabled.
     * |        |          |1 = Pin 5 wake-up interrupt Enabled.
     * |        |          |Note 1: The PMC interrupt will occur when both PIN4WKIF(PMC_INTSTS[13] and PIN4WKIEN(PMC_INTEN[13]) are 1.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: This bit is not retained when D4 power is turned off.
     * |[16]    |GPATGWKIEN|GPA Trigger Wake-up Interrupt Enable Bit (Write Protect)
     * |        |          |0 = GPA triggrer wake-up interrupt Disabled.
     * |        |          |1 = GPA triggrer wake-up interrupt Enabled.
     * |        |          |Note 1: The PMC interrupt will occur when both GPATGWKIF(PMC_INTSTS[16] and GPATGWKIEN(PMC_INTEN[16]) are 1.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: This bit is not retained when D3 power is turned off.
     * |[17]    |GPBTGWKIEN|GPB Trigger Wake-up Interrupt Enable Bit (Write Protect)
     * |        |          |0 = GPB triggrer wake-up interrupt Disabled.
     * |        |          |1 = GPB triggrer wake-up interrupt Enabled.
     * |        |          |Note 1: The PMC interrupt will occur when both GPBTGWKIF(PMC_INTSTS[17] and GPBTGWKIEN(PMC_INTEN[17]) are 1.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: This bit is not retained when D3 power is turned off.
     * |[18]    |GPCTGWKIEN|GPC Trigger Wake-up Interrupt Enable Bit (Write Protect)
     * |        |          |0 = GPC triggrer wake-up interrupt Disabled.
     * |        |          |1 = GPC triggrer wake-up interrupt Enabled.
     * |        |          |Note 1: The PMC interrupt will occur when both GPCTGWKIF(PMC_INTSTS[18] and GPCTGWKIEN(PMC_INTEN[18]) are 1.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: This bit is not retained when D3 power is turned off.
     * |[19]    |GPDTGWKIEN|GPD Trigger Wake-up Interrupt Enable Bit (Write Protect)
     * |        |          |0 = GPD triggrer wake-up interrupt Disabled.
     * |        |          |1 = GPD triggrer wake-up interrupt Enabled.
     * |        |          |Note 1: The PMC interrupt will occur when both GPDTGWKIF(PMC_INTSTS[19] and GPDTGWKIEN(PMC_INTEN[19]) are 1.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: This bit is not retained when D3 power is turned off.
     * @var PMC_T::INTSTS
     * Offset: 0x08  PMC Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PDWKIF    |Power-down Mode Wake-up Interrupt Status
     * |        |          |Set by u201CPower-down wake-up eventu201D, it indicates that resume from Power-down mode.
     * |        |          |The flag is set if any wake-up source occurred. Refer Power Modes and Wake-up Sources chapter.
     * |        |          |Note 1: The PMC interrupt will occur when both PDWKIF(PMC_INTSTS[0] and PDWKIEN(PMC_INTEN[0]) are 1.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |STMRWKIF  |Standby Timer Wake-up Interrupt Status
     * |        |          |The flag is set if stadnby timer wake-up event occurred in Power-down mode.
     * |        |          |Note 1: The PMC interrupt will occur when both STMRWKIF(PMC_INTSTS[1] and STMRWKIEN(PMC_INTEN[1]) are 1.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[8]     |PIN0WKIF  |Pin 0 Wake-up Interrupt Status
     * |        |          |The flag is set if Pin 0 wake-up event occurred in Power-down mode.
     * |        |          |Note 1: The PMC interrupt will occur when both PIN0WKIF(PMC_INTSTS[8] and PIN0WKIEN(PMC_INTEN[8]) are 1.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[9]     |PIN1WKIF  |Pin 1 Wake-up Interrupt Status
     * |        |          |The flag is set if Pin 1 wake-up event occurred in Power-down mode.
     * |        |          |Note 1: The PMC interrupt will occur when both PIN1WKIF(PMC_INTSTS[9] and PIN1WKIEN(PMC_INTEN[9]) are 1.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[10]    |PIN2WKIF  |Pin 2 Wake-up Interrupt Status
     * |        |          |The flag is set if Pin 2 wake-up event occurred in Power-down mode.
     * |        |          |Note 1: The PMC interrupt will occur when both PIN2WKIF(PMC_INTSTS[10] and PIN2WKIEN(PMC_INTEN[10]) are 1.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[11]    |PIN3WKIF  |Pin 3 Wake-up Interrupt Status
     * |        |          |The flag is set if Pin 3 wake-up event occurred in Power-down mode.
     * |        |          |Note 1: The PMC interrupt will occur when both PIN3WKIF(PMC_INTSTS[11] and PIN3WKIEN(PMC_INTEN[11]) are 1.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[12]    |PIN4WKIF  |Pin 4 Wake-up Interrupt Status
     * |        |          |The flag is set if Pin 4 wake-up event occurred in Power-down mode.
     * |        |          |Note 1: The PMC interrupt will occur when both PIN4WKIF(PMC_INTSTS[12] and PIN4WKIEN(PMC_INTEN[12]) are 1.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[13]    |PIN5WKIF  |Pin 5 Wake-up Interrupt Status
     * |        |          |The flag is set if Pin 5 wake-up event occurred in Power-down mode.
     * |        |          |Note 1: The PMC interrupt will occur when both PIN4WKIF(PMC_INTSTS[13] and PIN4WKIEN(PMC_INTEN[13]) are 1.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[16]    |GPATGWKIF |GPA Trigger Wake-up Interrupt Status
     * |        |          |The flag is set if GPA trigger wake-up event occurred in Power-down mode.
     * |        |          |Note 1: The PMC interrupt will occur when both GPATGWKIF(PMC_INTSTS[16] and GPATGWKIEN(PMC_INTEN[16]) are 1.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[17]    |GPBTGWKIF |GPB Trigger Wake-up Interrupt Status
     * |        |          |The flag is set if GPB trigger wake-up event occurred in Power-down mode.
     * |        |          |Note 1: The PMC interrupt will occur when both GPBTGWKIF(PMC_INTSTS[17] and GPBTGWKIEN(PMC_INTEN[17]) are 1.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[18]    |GPCTGWKIF |GPC Trigger Wake-up Interrupt Status
     * |        |          |The flag is set if GPC trigger wake-up event occurred in Power-down mode.
     * |        |          |Note 1: The PMC interrupt will occur when both GPCTGWKIF(PMC_INTSTS[18] and GPCTGWKIEN(PMC_INTEN[18]) are 1.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[19]    |GPDTGWKIF |GPC Trigger Wake-up Interrupt Status
     * |        |          |The flag is set if GPD trigger wake-up event occurred in Power-down mode.
     * |        |          |Note 1: The PMC interrupt will occur when both GPDTGWKIF(PMC_INTSTS[19] and GPDTGWKIEN(PMC_INTEN[19]) are 1.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[31]    |CLRWK     |Clear Wake-up Flag
     * |        |          |0 = No clear.
     * |        |          |1 = Clear all wake-up flags.
     * |        |          |Note 1: This bit is auto cleared by hardware.
     * @var PMC_T::PLCTL
     * Offset: 0x10  Power Level Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PLSEL     |Power Level Select (Write Protect)
     * |        |          |0 = Set to Power level 0 (PL0).
     * |        |          |1 = Set to Power level 1 (PL1). (default)
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: These bits are not retained when D3 power is turned off.
     * |[31]    |WRBUSY    |Write Busy Flag (Read Only)
     * |        |          |If PMC_PLCTL be written, this bit be asserted automatic by hardware, and be de-asserted when write procedure is finished.
     * |        |          |0 = PMC_PLCTL register is ready for write operation.
     * |        |          |1 = PMC_PLCTL register is busy on the last write operation. Other write operations are ignored.
     * @var PMC_T::PLSTS
     * Offset: 0x14  Power Level Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PLSTATUS  |Power Level Status (Read Only)
     * |        |          |This bit field reflect the current power level.
     * |        |          |0 = Power level is PL0.
     * |        |          |1 = Power level is PL1.
     * |[31]    |PLCBUSY   |Power Level Change Busy Bit (Read Only)
     * |        |          |This bit is set by hardware when power level is changing
     * |        |          |After power level change is completed, this bit will be cleared automatically by hardware.
     * |        |          |0 = Power level change is completed.
     * |        |          |1 = Power level change is ongoing.
     * @var PMC_T::VRCTL
     * Offset: 0x18  Voltage Regulator Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MVRS      |Main Voltage Regulator Type Select (Write Protect)
     * |        |          |This bit field sets main voltage regulator type.
     * |        |          |After setting main voltage regulator type to DCDC (MVRS (PMC_VRCTL[0]) = 1), system will set main voltage regulator type change busy flag MVRCBUSY(PMC_VRSTS[31]), detect inductor connection and update inductor connection status LCONS (PMC_VRSTS[2])
     * |        |          |if inductor exist LCONS will be cleared and main voltage regulator type can switch to DCDC (CURMVR (PM_VRSTS[0)=1).
     * |        |          |0 = Set main voltage regulator to LDO.
     * |        |          |1 = Set main voltage regulator to DCDC.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: This bit is not retained when D3 power is turned off.
     * |[31]    |WRBUSY    |Write Busy Flag (Read Only)
     * |        |          |If PMC_VRCTL be written, this bit be asserted automatic by hardware, and be de-asserted when write procedure is finished.
     * |        |          |0 = PMC_VRCTL register is ready for write operation.
     * |        |          |1 = PMC_VRCTL register is busy on the last write operation. Other write operations are ignored.
     * @var PMC_T::VRSTS
     * Offset: 0x1C  Voltage Regulator Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CURMVR    |Current Main Voltage Regulator Type (Read Only)
     * |        |          |This bit field reflects current main voltage regulator type.
     * |        |          |0 = Current main voltage regulator in active and Idle mode is LDO.
     * |        |          |1 = Current main voltage regulator in active and Idle mode is DCDC.
     * |        |          |Note 1: This bit is not retained when D3 power is turned off.
     * |[1]     |MVRCERR   |Main Voltage Regulator Type Change Error Bit (Write Protect)
     * |        |          |This bit is set to 1 when main voltage regulator type change from LDO to DCDC error, the following conditions will cause change errors:
     * |        |          |u00A7 System changed to DC-DC mode but LDO change voltage process not finished.
     * |        |          |u00A7 Detect inductor failed.
     * |        |          |0 = No main voltage regulator type change error.
     * |        |          |1 = Main voltage regulator type change to DCDC error occurred.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write 1 to clear this bit to 0.
     * |[2]     |LCONS     |Inductor for DCDC Connect Status (Read Only)
     * |        |          |This bit is valid whencurrent main voltage regulator type is DCDC (CURMVR (PMC_VRSTS[0])=1)
     * |        |          |If current main voltage regulator type is LDO (CURMVR (PMC_VRSTS[0])=0) this bit is set to 1.
     * |        |          |0 = Inductor connect between Vsw and LDO_CAP pin.
     * |        |          |1 = No Inductor connect between Vsw and LDO_CAP pin.
     * |        |          |Note: This bit is 1 when main voltage regulator is LDO.
     * |[31]    |MVRCBUSY  |Main Voltage Regulator Type Change Busy Bit (Read Only)
     * |        |          |This bit is set by hardware when main voltage regulator type is changing
     * |        |          |After main voltage regulator type change is completed, this bit will be cleared automatically by hardware.
     * |        |          |0 = Main voltage regulator type change is completed.
     * |        |          |1 = Main voltage regulator type change is ongoing.
     * @var PMC_T::IOSHCTL
     * Offset: 0x20  GPIO Status Hold Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |IOHR      |GPIO Hold Release
     * |        |          |When system enter Power-down mode with D1PGEN = 1, all I/O status are hold to keep normal operating status
     * |        |          |After chip is woken from Power-down mode, the I/O are still keep hold status until user set this bit to release I/O hold status.
     * |        |          |Note 1: This bit is auto cleared by hardware.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var PMC_T::FSHPCTL
     * Offset: 0x24  FLASH Power Mode Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * @var PMC_T::BATLDCTL
     * Offset: 0x28  Battery Loss Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BATLDEN   |Battery Loss Detector Enable Bit (Write Protect)
     * |        |          |0 = Battery Loss Detector Disabled.
     * |        |          |1 = Battery Loss Detector Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D3 power is turned off.
     * @var PMC_T::IOTGDBCTL
     * Offset: 0x30  GPIO Trigger De-bounce Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |IOTGDBSEL |GPIO Trigger Pin De-bounce Sampling Cycle Selection
     * |        |          |0000 = Sample GPIO input once per 1 clock.
     * |        |          |0001 = Sample GPIO input once per 2 clocks.
     * |        |          |0010 = Sample GPIO input once per 4 clocks.
     * |        |          |0011 = Sample GPIO input once per 8 clocks.
     * |        |          |0100 = Sample GPIO input once per 16 clocks.
     * |        |          |0101 = Sample GPIO input once per 32 clocks.
     * |        |          |0110 = Sample GPIO input once per 64 clocks.
     * |        |          |0111 = Sample GPIO input once per 128 clocks.
     * |        |          |1000 = Sample GPIO input once per 256 clocks.
     * |        |          |1001 = Sample GPIO input once per 2*256 clocks.
     * |        |          |1010 = Sample GPIO input once per 4*256 clocks.
     * |        |          |1011 = Sample GPIO input once per 8*256 clocks.
     * |        |          |1100 = Sample GPIO input once per 16*256 clocks.
     * |        |          |1101 = Sample GPIO input once per 32*256 clocks.
     * |        |          |1110 = Sample GPIO input once per 64*256 clocks.
     * |        |          |1111 = Sample GPIO input once per 128*256 clocks.
     * |        |          |Note 1: De-bounce counter clock source is the 32 kHz internal low speed RC oscillator (LIRC).
     * |        |          |Note 2: These bits are not retained when D3 power is turned off.
     * @var PMC_T::GPATGCTL
     * Offset: 0x34  GPA Trigger Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TGPWKEN   |GPA Trigger Pin Wake-up Enable Bit
     * |        |          |0 = GPA group trigger pin wake-up chip Disabled, trigger ip Enabled.
     * |        |          |1 = GPA group trigger pin wake-up chip Enabled, trigger ip Enabled.
     * |        |          |Note 1: The GPA Trigger Pin wake-up is always enable when D2PGEN(PMC_PWRCTL[2] is 1.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |[1]     |TGPREN    |GPA Trigger Pin Rising Edge Enable Bit
     * |        |          |0 = GPA group trigger pin rising edge trigger function Disabled.
     * |        |          |1 = GPA group trigger pin rising edge trigger function Enabled.
     * |        |          |Note: This bit is not retained when D3 power is turned off.
     * |[2]     |TGPFEN    |GPA Trigge Pin Falling Edge Enable Bit
     * |        |          |0 = GPA group trigger pin falling edge trigger function Disabled.
     * |        |          |1 = GPA group trigger pin falling edge trigger function Enabled.
     * |        |          |Note: This bit is not retained when D3 power is turned off.
     * |[3]     |TGPDBEN   |GPA Trigger Pin Input Signal De-bounce Enable Bit
     * |        |          |The DBEN bit is used to enable the de-bounce function for each corresponding I/O
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger and wake-up.The de-bounce clock source is the 32 kHz internal low speed RC oscillator (LIRC).
     * |        |          |0 = Trigger pin De-bounce function Disabled.
     * |        |          |1 = Trigger pin De-bounce function Enabled.
     * |        |          |Note 1: The de-bounce function is valid only for edgetriggered.
     * |        |          |Note 2: This bit is not retained when D3 power is turned off.
     * |[7:4]   |TGPSEL    |GPA Trigger Pin Select
     * |        |          |0000 = GPA.0 trigger function Enabled.
     * |        |          |0001 = GPA.1 trigger function Enabled.
     * |        |          |0010 = GPA.2 trigger function Enabled.
     * |        |          |0011 = GPA.3 trigger function Enabled.
     * |        |          |0100 = GPA.4 trigger function Enabled.
     * |        |          |0101 = GPA.5 trigger function Enabled.
     * |        |          |0110 = GPA.6 trigger function Enabled.
     * |        |          |0111 = GPA.7 trigger function Enabled.
     * |        |          |1000 = GPA.8 trigger function Enabled.
     * |        |          |1001 = GPA.9 trigger function Enabled.
     * |        |          |1010 = GPA.10 trigger function Enabled.
     * |        |          |1011 = GPA.11 trigger function Enabled.
     * |        |          |1100 = GPA.12 trigger function Enabled.
     * |        |          |1101 = GPA.13 trigger function Enabled.
     * |        |          |1110 = GPA.14 trigger function Enabled.
     * |        |          |1111 = GPA.15 trigger function Enabled.
     * |        |          |Note: These bits are not retained when D3 power is turned off.
     * @var PMC_T::GPBTGCTL
     * Offset: 0x38  GPB Trigger Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TGPWKEN   |GPB Trigger Pin Wake-up Enable Bit
     * |        |          |0 = GPB group trigger pin wake-up chip Disabled, trigger ip Enabled.
     * |        |          |1 = GPB group trigger pin wake-up chip Enabled, trigger ip Enabled.
     * |        |          |Note 1: The GPB Trigger Pin wake-up is always enable when D2PGEN(PMC_PWRCTL[2] is 1.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |[1]     |TGPREN    |GPB Trigger Pin Rising Edge Enable Bit
     * |        |          |0 = GPB group trigger pin rising edge trigger function Disabled.
     * |        |          |1 = GPB group trigger pin rising edge trigger function Enabled.
     * |        |          |Note: This bit is not retained when D3 power is turned off.
     * |[2]     |TGPFEN    |GPB Trigge Pin Falling Edge Enable Bit
     * |        |          |0 = GPB group trigger pin falling edge trigger function Disabled.
     * |        |          |1 = GPB group trigger pin falling edge trigger function Enabled.
     * |        |          |Note: This bit is not retained when D3 power is turned off.
     * |[3]     |TGPDBEN   |GPB Trigger Pin Input Signal De-bounce Enable Bit
     * |        |          |The DBEN bit is used to enable the de-bounce function for each corresponding I/O
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger and wake-up.The de-bounce clock source is the 32 kHz internal low speed RC oscillator (LIRC).
     * |        |          |0 = Trigger pin De-bounce function Disabled.
     * |        |          |1 = Trigger pin De-bounce function Enabled.
     * |        |          |Note 1: The de-bounce function is valid only for edgetriggered.
     * |        |          |Note 2: This bit is not retained when D3 power is turned off.
     * |[7:4]   |TGPSEL    |GPB Trigger Pin Select
     * |        |          |0000 = GPB.0 trigger function Enabled.
     * |        |          |0001 = GPB.1 trigger function Enabled.
     * |        |          |0010 = GPB.2 trigger function Enabled.
     * |        |          |0011 = GPB.3 trigger function Enabled.
     * |        |          |0100 = GPB.4 trigger function Enabled.
     * |        |          |0101 = GPB.5 trigger function Enabled.
     * |        |          |0110 = GPB.6 trigger function Enabled.
     * |        |          |0111 = GPB.7 trigger function Enabled.
     * |        |          |1000 = GPB.8 trigger function Enabled.
     * |        |          |1001 = GPB.9 trigger function Enabled.
     * |        |          |1010 = GPB.10 trigger function Enabled.
     * |        |          |1011 = GPB.11 trigger function Enabled.
     * |        |          |1100 = GPB.12 trigger function Enabled.
     * |        |          |1101 = GPB.13 trigger function Enabled.
     * |        |          |1110 = GPB.14 trigger function Enabled.
     * |        |          |1111 = GPB.15 trigger function Enabled.
     * |        |          |Note: These bits are not retained when D3 power is turned off.
     * @var PMC_T::GPCTGCTL
     * Offset: 0x3C  GPC Trigger Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TGPWKEN   |GPC Trigger Pin Wake-up Enable Bit
     * |        |          |0 = GPC group trigger pin wake-up chip Disabled, trigger ip Enabled.
     * |        |          |1 = GPC group trigger pin wake-up chip Enabled, trigger ip Enabled.
     * |        |          |Note 1: The GPC Trigger Pin wake-up is always enable when D2PGEN(PMC_PWRCTL[2] is 1.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |[1]     |TGPREN    |GPC Trigger Pin Rising Edge Enable Bit
     * |        |          |0 = GPC group trigger pin rising edge trigger function Disabled.
     * |        |          |1 = GPC group trigger pin rising edge trigger function Enabled.
     * |        |          |Note: This bit is not retained when D3 power is turned off.
     * |[2]     |TGPFEN    |GPC Trigge Pin Falling Edge Enable Bit
     * |        |          |0 = GPC group trigger pin falling edge trigger function Disabled.
     * |        |          |1 = GPC group trigger pin falling edge trigger function Enabled.
     * |        |          |Note: This bit is not retained when D3 power is turned off.
     * |[3]     |TGPDBEN   |GPC Trigger Pin Input Signal De-bounce Enable Bit
     * |        |          |The TGPDBEN bit is used to enable the de-bounce function for each corresponding I/O
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger and wake-up.The de-bounce clock source is the 32 kHz internal low speed RC oscillator (LIRC).
     * |        |          |0 = Trigger pin De-bounce function Disabled.
     * |        |          |1 = Trigger pin De-bounce function Enabled.
     * |        |          |Note 1: The de-bounce function is valid only for edgetriggered.
     * |        |          |Note 2: This bit is not retained when D3 power is turned off.
     * |[7:4]   |TGPSEL    |GPC Trigger Pin Select
     * |        |          |0000 = GPC.0 trigger function Enabled.
     * |        |          |0001 = GPC.1 trigger function Enabled.
     * |        |          |0010 = GPC.2 trigger function Enabled.
     * |        |          |0011 = GPC.3 trigger function Enabled.
     * |        |          |0100 = GPC.4 trigger function Enabled.
     * |        |          |0101 = GPC.5 trigger function Enabled.
     * |        |          |0110 = GPC.6 trigger function Enabled.
     * |        |          |0111 = GPC.7 trigger function Enabled.
     * |        |          |1000 = GPC.8 trigger function Enabled.
     * |        |          |1001 = GPC.9 trigger function Enabled.
     * |        |          |1010 = GPC.10 trigger function Enabled.
     * |        |          |1011 = GPC.11 trigger function Enabled.
     * |        |          |1100 = GPC.12 trigger function Enabled.
     * |        |          |1101 = GPC.13 trigger function Enabled.
     * |        |          |1110 = GPC.14 trigger function Enabled.
     * |        |          |1111 = Reserved.
     * |        |          |Note: These bits are not retained when D3 power is turned off.
     * @var PMC_T::GPDTGCTL
     * Offset: 0x40  GPD Trigger Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TGPWKEN   |GPD Trigger Pin Wake-up Enable Bit
     * |        |          |0 = GPD group trigger pin wake-up chip Disabled, trigger ip Enabled.
     * |        |          |1 = GPD group trigger pin wake-up chip Enabled, trigger ip Enabled.
     * |        |          |Note 1: The GPD Trigger Pin wake-up is always enable when D2PGEN(PMC_PWRCTL[2] is 1.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |[1]     |TGPREN    |GPD Trigger Pin Rising Edge Enable Bit
     * |        |          |0 = GPD group trigger pin rising edge trigger function Disabled.
     * |        |          |1 = GPD group trigger pin rising edge trigger function Enabled.
     * |        |          |Note: This bit is not retained when D3 power is turned off.
     * |[2]     |TGPFEN    |GPD Trigge Pin Falling Edge Enable Bit
     * |        |          |0 = GPD group trigger pin falling edge trigger function Disabled.
     * |        |          |1 = GPD group trigger pin falling edge trigger function Enabled.
     * |        |          |Note: This bit is not retained when D3 power is turned off.
     * |[3]     |TGPDBEN   |GPD Trigger Pin Input Signal De-bounce Enable Bit
     * |        |          |The DBEN bit is used to enable the de-bounce function for each corresponding I/O
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger and wake-up.The de-bounce clock source is the 32 kHz internal low speed RC oscillator (LIRC).
     * |        |          |0 = Trigger pin De-bounce function Disabled.
     * |        |          |1 = Trigger pin De-bounce function Enabled.
     * |        |          |Note 1: The de-bounce function is valid only for edgetriggered.
     * |        |          |Note 2: This bit is not retained when D3 power is turned off.
     * |[7:4]   |TGPSEL    |GPD Trigger Pin Select
     * |        |          |0000 = GPD.0 trigger function Enabled.
     * |        |          |0001 = GPD.1 trigger function Enabled.
     * |        |          |0010 = GPD.2 trigger function Enabled.
     * |        |          |0011 = GPD.3 trigger function Enabled.
     * |        |          |0100 = GPD.4 trigger function Enabled.
     * |        |          |0101 = GPD.5 trigger function Enabled.
     * |        |          |0110 = GPD.6 trigger function Enabled.
     * |        |          |0111 = GPD.7 trigger function Enabled.
     * |        |          |1000 = GPD.8 trigger function Enabled.
     * |        |          |1001 = GPD.9 trigger function Enabled.
     * |        |          |1010 = GPD.10 trigger function Enabled.
     * |        |          |1011 = GPD.11 trigger function Enabled.
     * |        |          |1100 = GPD.12 trigger function Enabled.
     * |        |          |1101 = GPD.13 trigger function Enabled.
     * |        |          |1110 = GPD.14 trigger function Enabled.
     * |        |          |1111 = Reserved.
     * |        |          |Note: These bits are not retained when D3 power is turned off.
     * @var PMC_T::STMRWKCTL
     * Offset: 0x50  Standby Timer Wake-up Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |STMREN    |Standby Timer Enable Bit (Write Protect)
     * |        |          |0 = Standby timer disabled.
     * |        |          |1 = Standby timer Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: This bit is not retained when D4 power is turned off.
     * |[1]     |STMRMOD   |Standby Timer Mode (Write Protect)
     * |        |          |0 = Standby timer start when enter any of Power-down mode ( except CPU idle mode).
     * |        |          |1 = Standby timer start immedially when STMREN (PMC_STMRWKCTL[0]) =1.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: This bit is not retained when D4 power is turned off.
     * |[11:8]  |STMRIS    |Stadnby Timer Time-out Interval Select (Write Protect)
     * |        |          |These bits control standby timer time-out interval when chip at Power-down mode.
     * |        |          |0000 = Time-out interval is 512 LIRC clocks (16ms).
     * |        |          |0001 = Time-out interval is 1024 LIRC clocks (32ms).
     * |        |          |0010 = Time-out interval is 2048 LIRC clocks (64ms).
     * |        |          |0011 = Time-out interval is 4096 LIRC clocks (128ms).
     * |        |          |0100 = Time-out interval is 8192 LIRC clocks (256ms).
     * |        |          |0101 = Time-out interval is 16384 LIRC clocks (512ms).
     * |        |          |0110 = Time-out interval is 32768 LIRC clocks (1024ms).
     * |        |          |0111 = Time-out interval is 65536 LIRC clocks (2048ms).
     * |        |          |1000 = Time-out interval is 131072 LIRC clocks (4096ms).
     * |        |          |1001 = Time-out interval is 262144 LIRC clocks (8192ms).
     * |        |          |1010 = Time-out interval is 524288 LIRC clocks (16384ms).
     * |        |          |1011 = Time-out interval is 1048576 LIRC clocks (32768ms).
     * |        |          |1100 = Time-out interval is 2097152 LIRC clocks (65536ms).
     * |        |          |1101 = Time-out interval is 4194304 LIRC clocks (131072ms).
     * |        |          |Others = Time-out interval is 512 LIRC clocks (16ms).
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: These bits are not retained when D4 power is turned off.
     * |[31]    |WRBUSY    |Write Busy Flag (Read Only)
     * |        |          |If PMC_STMRWKCTL be written, this bit be asserted automatic by hardware, and be de-asserted when write procedure is finished.
     * |        |          |0 = PMC_ STMRWKCTL register is ready for write operation.
     * |        |          |1 = PMC_ STMRWKCTL register is busy on the last write operation. Other write operations are ignored.
     * @var PMC_T::PINWKCTL
     * Offset: 0x60  Pin Wake-up Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |WKPINEN0  |Wake-up Pin0 Enable Bit (Write Protect)
     * |        |          |This is control register for GPC.0 to wake-up pin.
     * |        |          |00 = Wake-up pin disable.
     * |        |          |01 = Wake-up pin rising edge enabled.
     * |        |          |10 = Wake-up pin falling edge enabled.
     * |        |          |11 = Wake-up pin both edge enabled.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D4 power is turned off.
     * |[3:2]   |WKPINEN1  |Wake-up Pin1 Enable Bit (Write Protect)
     * |        |          |This is control register for GPB.0 to wake-up pin.
     * |        |          |00 = Wake-up pin disable.
     * |        |          |01 = Wake-up pin rising edge enabled.
     * |        |          |10 = Wake-up pin falling edge enabled.
     * |        |          |11 = Wake-up pin both edge enabled.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D4 power is turned off.
     * |[5:4]   |WKPINEN2  |Wake-up Pin2 Enable Bit (Write Protect)
     * |        |          |This is control register for GPB.2 to wake-up pin.
     * |        |          |00 = Wake-up pin disable.
     * |        |          |01 = Wake-up pin rising edge enabled.
     * |        |          |10 = Wake-up pin falling edge enabled.
     * |        |          |11 = Wake-up pin both edge enabled.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D4 power is turned off.
     * |[7:6]   |WKPINEN3  |Wake-up Pin3 Enable Bit (Write Protect)
     * |        |          |This is control register for GPB.12 to wake-up pin.
     * |        |          |00 = Wake-up pin disable.
     * |        |          |01 = Wake-up pin rising edge enabled.
     * |        |          |10 = Wake-up pin falling edge enabled.
     * |        |          |11 = Wake-up pin both edge enabled.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D4 power is turned off.
     * |[9:8]   |WKPINEN4  |Wake-up Pin4 Enable Bit (Write Protect)
     * |        |          |This is control register for GPF.6 to wake-up pin.
     * |        |          |00 = Wake-up pin disable.
     * |        |          |01 = Wake-up pin rising edge enabled.
     * |        |          |10 = Wake-up pin falling edge enabled.
     * |        |          |11 = Wake-up pin both edge enabled.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D4 power is turned off.
     * |        |          |Note 3: Setting IOCTLSEL(RTC_LXTCTL[8]) to avoid GPF6 unexpected falling edge.
     * |[11:10] |WKPINEN5  |Wake-up Pin5 Enable Bit (Write Protect)
     * |        |          |This is control register for GPA.12 to wake-up pin.
     * |        |          |00 = Wake-up pin disable.
     * |        |          |01 = Wake-up pin rising edge enabled.
     * |        |          |10 = Wake-up pin falling edge enabled.
     * |        |          |11 = Wake-up pin both edge enabled.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D4 power is turned off.
     * @var PMC_T::SYSRB0PC
     * Offset: 0x100  System SRAM Bank 0 Power Mode Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |SRAM0PMS  |System SRAM0 In Bank0 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM0 in bank0 (64k) power mode for range 0x2010_0000 - 0x2010_FFFF.
     * |        |          |00 = System SRAM0 in bank0 is normal mode.
     * |        |          |01 = System SRAM0 in bank0 is retention mode.
     * |        |          |10 = System SRAM0 in bank0 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignore when PCBUSY is 1.
     * |        |          |Note 3: These bits are not retained when D3 power is turned off.
     * |[3:2]   |SRAM1PMS  |System SRAM1 In Bank0 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM1 in bank0 (64k) power mode for range 0x2011_0000 - 0x2011_FFFF.
     * |        |          |00 = System SRAM1 in bank0 is normal mode.
     * |        |          |01 = System SRAM1 in bank0 is retention mode.
     * |        |          |10 = System SRAM1 in bank0 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignore when PCBUSY is 1.
     * |        |          |Note 3: These bits are not retained when D3 power is turned off.
     * |[5:4]   |SRAM2PMS  |System SRAM2 In Bank0 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM2 in bank0 (64k) power mode for range 0x2012_0000 - 0x2012_FFFF.
     * |        |          |00 = System SRAM2 in bank0 is normal mode.
     * |        |          |01 = System SRAM2 in bank0 is retention mode.
     * |        |          |10 = System SRAM2 in bank0 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignore when PCBUSY is 1.
     * |        |          |Note 3: These bits are not retained when D3 power is turned off.
     * |[7:6]   |SRAM3PMS  |System SRAM3 In Bank0 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM3 in bank0 (64k) power mode for range 0x2013_0000 - 0x2013_FFFF.
     * |        |          |00 = System SRAM3 in bank0 is normal mode.
     * |        |          |01 = System SRAM3 in bank0 is retention mode.
     * |        |          |10 = System SRAM3 in bank0 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignore when PCBUSY is 1.
     * |        |          |Note 3.: These bits are not retained when D3 power is turned off.
     * |[9:8]   |SRAM4PMS  |System SRAM4 In Bank0 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM4 in bank0 (64k) power mode for range 0x2014_0000 - 0x2014_FFFF.
     * |        |          |00 = System SRAM4 in bank0 is normal mode.
     * |        |          |01 = System SRAM4 in bank0 is retention mode.
     * |        |          |10 = System SRAM4 in bank0 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignore when PCBUSY is 1.
     * |        |          |Note 3: These bits are not retained when D3 power is turned off.
     * |[11:10] |SRAM5PMS  |System SRAM5 In Bank0 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM5 in bank0 (64k) power mode for range 0x2015_0000 - 0x2015_FFFF.
     * |        |          |00 = System SRAM5 in bank0 is normal mode.
     * |        |          |01 = System SRAM5 in bank0 is retention mode.
     * |        |          |10 = System SRAM5 in bank0 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignore when PCBUSY is 1.
     * |        |          |Note 3: These bits are not retained when D3 power is turned off.
     * |[13:12] |SRAM6PMS  |System SRAM6 In Bank0 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM6 in bank0 (64k) power mode for range 0x2016_0000 - 0x2016_FFFF.
     * |        |          |00 = System SRAM6 in bank0 is normal mode.
     * |        |          |01 = System SRAM6 in bank0 is retention mode.
     * |        |          |10 = System SRAM6 in bank0 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignore when PCBUSY is 1.
     * |        |          |Note 3: These bits are not retained when D3 power is turned off.
     * |[15:14] |SRAM7PMS  |System SRAM7 In Bank0 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM7 in bank0 (64k) power mode for range 0x2017_0000 - 0x2017_FFFF.
     * |        |          |00 = System SRAM7 in bank0 is normal mode.
     * |        |          |01 = System SRAM7 in bank0 is retention mode.
     * |        |          |10 = System SRAM7 in bank0 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignore when PCBUSY is 1.
     * |        |          |Note 3: These bits are not retained when D3 power is turned off.
     * |[31]    |PCBUSY    |Power Changing Busy Flag (Read Only)
     * |        |          |If PMC_SYSRB0PC be written, this bit be asserted automatic by hardware, and be de-asserted when SRAM power change procedure is finished.
     * |        |          |0 = System SRAM bank 0 power change finished.
     * |        |          |1 = System SRAM bank 0 power changing. The PMC_SYSRB0PC write operations are ignored.
     * |        |          |set to power shut down mode by hardware.
     * |        |          |Note 1: This bit is not retained when D3 power is turned off.
     * @var PMC_T::SYSRB1PC
     * Offset: 0x104  System SRAM Bank 1 Power Mode Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |SRAM0PMS  |System SRAM0 In Bank1 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM0 in bank1 (64k) power mode for range 0x2018_0000 - 0x2018_FFFF.
     * |        |          |00 = System SRAM0 in bank1 is normal mode.
     * |        |          |01 = System SRAM0 in bank1 is retention mode.
     * |        |          |10 = System SRAM0 in bank1 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignored when PCBUSY is 1.
     * |        |          |Note 3: If system enters Power-down mode and D0PGEN(PMC_PWRCTL[0]) is set to 1, this SRAM is auto set to power shut down mode by hardware.
     * |        |          |Note 4: These bits are not retained when D0 power is turned off.
     * |[3:2]   |SRAM1PMS  |System SRAM1 In Bank1 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM1 in bank1 (64k) power mode for range 0x2019_0000 - 0x2019_FFFF.
     * |        |          |00 = System SRAM1 in bank1 is normal mode.
     * |        |          |01 = System SRAM1 in bank1 is retention mode.
     * |        |          |10 = System SRAM1 in bank1 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignored when PCBUSY is 1.
     * |        |          |Note 3: If system enters Power-down mode and D0PGEN(PMC_PWRCTL[0]) is set to 1, this SRAM is auto set to power shut down mode by hardware.
     * |        |          |Note 4: These bits are not retained when D0 power is turned off.
     * |[5:4]   |SRAM2PMS  |System SRAM2 In Bank1 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM2 in bank1 (64k) power mode for range 0x201A_0000 - 0x201A_FFFF.
     * |        |          |00 = System SRAM2 in bank1 is normal mode.
     * |        |          |01 = System SRAM2 in bank1 is retention mode.
     * |        |          |10 = System SRAM2 in bank1 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignored when PCBUSY is 1.
     * |        |          |Note 3: If system enters Power-down mode and D0PGEN(PMC_PWRCTL[0]) is set to 1, this SRAM is auto set to power shut down mode by hardware.
     * |        |          |Note 4: These bits are not retained when D0 power is turned off.
     * |[7:6]   |SRAM3PMS  |System SRAM3 In Bank1 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM3 in bank1 (64k) power mode for range 0x201B_0000 - 0x201B_FFFF.
     * |        |          |00 = System SRAM3 in bank1 is normal mode.
     * |        |          |01 = System SRAM3 in bank1 is retention mode.
     * |        |          |10 = System SRAM3 in bank1 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignored when PCBUSY is 1.
     * |        |          |Note 3: If system enters Power-down mode and D0PGEN(PMC_PWRCTL[0]) is set to 1, this SRAM is auto set to power shut down mode by hardware.
     * |        |          |Note 4: These bits are not retained when D0 power is turned off.
     * |[9:8]   |SRAM4PMS  |System SRAM4 In Bank1 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM4 in bank1 (64k) power mode for range 0x201C_0000 - 0x201C_FFFF.
     * |        |          |00 = System SRAM4 in bank1 is normal mode.
     * |        |          |01 = System SRAM4 in bank1 is retention mode.
     * |        |          |10 = System SRAM4 in bank1 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignored when PCBUSY is 1.
     * |        |          |Note 3: If system enters Power-down mode and D0PGEN(PMC_PWRCTL[0]) is set to 1, this SRAM is auto set to power shut down mode by hardware.
     * |        |          |Note 4: These bits are not retained when D0 power is turned off.
     * |[11:10] |SRAM5PMS  |System SRAM5 In Bank1 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM5 in bank1 (64k) power mode for range 0x201D_0000 - 0x201D_FFFF.
     * |        |          |00 = System SRAM5 in bank1 is normal mode.
     * |        |          |01 = System SRAM5 in bank1 is retention mode.
     * |        |          |10 = System SRAM5 in bank1 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignored when PCBUSY is 1.
     * |        |          |Note 3: If system enters Power-down mode and D0PGEN(PMC_PWRCTL[0]) is set to 1, this SRAM is auto set to power shut down mode by hardware.
     * |        |          |Note 4: These bits are not retained when D0 power is turned off.
     * |[13:12] |SRAM6PMS  |System SRAM6 In Bank1 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM6 in bank1 (64k) power mode for range 0x201E_0000 - 0x201E_FFFF.
     * |        |          |00 = System SRAM6 in bank1 is normal mode.
     * |        |          |01 = System SRAM6 in bank1 is retention mode.
     * |        |          |10 = System SRAM6 in bank1 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignored when PCBUSY is 1.
     * |        |          |Note 3: If system enters Power-down mode and D0PGEN(PMC_PWRCTL[0]) is set to 1, this SRAM is auto set to power shut down mode by hardware.
     * |        |          |Note 4: These bits are not retained when D0 power is turned off.
     * |[15:14] |SRAM7PMS  |System SRAM7 In Bank1 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM7 in bank1 (64k) power mode for range 0x201F_0000 - 0x201F_FFFF.
     * |        |          |00 = System SRAM7 in bank1 is normal mode.
     * |        |          |01 = System SRAM7 in bank1 is retention mode.
     * |        |          |10 = System SRAM7 in bank1 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignored when PCBUSY is 1.
     * |        |          |Note 3: If system enters Power-down mode and D0PGEN(PMC_PWRCTL[0]) is set to 1, this SRAM is auto set to power shut down mode by hardware.
     * |        |          |Note 4: These bits are not retained when D0 power is turned off.
     * |[31]    |PCBUSY    |Power Changing Busy Flag (Read Only)
     * |        |          |If PMC_SYSRB1PC be written, this bit be asserted automatic by hardware, and be de-asserted when SRAM power change procedure is finish.
     * |        |          |0 = System SRAM bank 1 power change finished.
     * |        |          |1 = System SRAM bank 1 power changing. The PMC_SYSRB1PC write operations are ignored.
     * |        |          |Note 1: This bit is not retained when D0 power is turned off.
     * @var PMC_T::SYSRB2PC
     * Offset: 0x108  System SRAM Bank 2 Power Mode Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |SRAM0PMS  |System SRAM0 In Bank2 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM0 in bank2 (64k) power mode for range 0x2020_0000 - 0x2020_FFFF.
     * |        |          |00 = System SRAM0 in bank2 is normal mode.
     * |        |          |01 = System SRAM0 in bank2 is retention mode.
     * |        |          |10 = System SRAM0 in bank2 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignored when PCBUSY is 1.
     * |        |          |Note 3: If system enters Power-down mode and D0PGEN(PMC_PWRCTL[0]) is set to 1, this SRAM is auto set to power shut down mode by hardware.
     * |        |          |Note 4: These bits are not retained when D0 power is turned off.
     * |[3:2]   |SRAM1PMS  |System SRAM1 In Bank2 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM1 in bank2 (64k) power mode for range 0x2021_0000 - 0x2021_FFFF.
     * |        |          |00 = System SRAM1 in bank2 is normal mode.
     * |        |          |01 = System SRAM1 in bank2 is retention mode.
     * |        |          |10 = System SRAM1 in bank2 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignored when PCBUSY is 1.
     * |        |          |Note 3: If system enters Power-down mode and D0PGEN(PMC_PWRCTL[0]) is set to 1, this SRAM is auto set to power shut down mode by hardware.
     * |        |          |Note 4: These bits are not retained when D0 power is turned off.
     * |[5:4]   |SRAM2PMS  |System SRAM2 In Bank2 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM2 in bank2 (64k) power mode for range 0x2022_0000 - 0x2022_FFFF.
     * |        |          |00 = System SRAM2 in bank2 is normal mode.
     * |        |          |01 = System SRAM2 in bank2 is retention mode.
     * |        |          |10 = System SRAM2 in bank2 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignored when PCBUSY is 1.
     * |        |          |Note 3: If system enters Power-down mode and D0PGEN(PMC_PWRCTL[0]) is set to 1, this SRAM is auto set to power shut down mode by hardware.
     * |        |          |Note 4: These bits are not retained when D0 power is turned off.
     * |[7:6]   |SRAM3PMS  |System SRAM3 In Bank2 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM3 in bank2 (64k) power mode for range 0x2023_0000 - 0x2023_FFFF.
     * |        |          |00 = System SRAM3 in bank2 is normal mode.
     * |        |          |01 = System SRAM3 in bank2 is retention mode.
     * |        |          |10 = System SRAM3 in bank2 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignored when PCBUSY is 1.
     * |        |          |Note 3: If system enters Power-down mode and D0PGEN(PMC_PWRCTL[0]) is set to 1, this SRAM is auto set to power shut down mode by hardware.
     * |        |          |Note 4: These bits are not retained when D0 power is turned off.
     * |[9:8]   |SRAM4PMS  |System SRAM4 In Bank2 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM2 in bank1 (64k) power mode for range 0x2024_0000 - 0x2024_FFFF.
     * |        |          |00 = System SRAM4 in bank2 is normal mode.
     * |        |          |01 = System SRAM4 in bank2 is retention mode.
     * |        |          |10 = System SRAM4 in bank2 is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignored when PCBUSY is 1.
     * |        |          |Note 3: If system enters Power-down mode and D0PGEN(PMC_PWRCTL[0]) is set to 1, this SRAM is auto set to power shut down mode by hardware.
     * |        |          |Note 4: These bits are not retained when D0 power is turned off.
     * |[31]    |PCBUSY    |Power Changing Busy Flag (Read Only)
     * |        |          |If PMC_SYSRB2PC be written, this bit be asserted automatic by hardware, and be de-asserted when SRAM power change procedure is finished.
     * |        |          |0 = System SRAM bank 2 power change finished.
     * |        |          |1 = System SRAM bank 2 power changing. The PMC_SYSRB2PC write operations are ignored.
     * |        |          |Note 1: This bit is not retained when D0 power is turned off.
     * @var PMC_T::LPSYSRPC
     * Offset: 0x110  Low Power Domain System SRAM Power Mode Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |SRAM0PMS  |Low Power Domain SRAM0 Power Mode Select (Write Protect)
     * |        |          |This field can control SRAM0 (8k) power mode for range 0x2031_0000 - 0x2031_0FFF.
     * |        |          |00 = System SRAM0 in low power domain is normal mode.
     * |        |          |01 = System SRAM0 in low power domain is retention mode.
     * |        |          |10 = System SRAM0 in low power domain is power shut down mode.
     * |        |          |11 = Reserved (Write Ignore).
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write ignored when PCBUSY is 1.
     * |        |          |Note 3: If system enters Power-down mode and D3PGEN(PMC_PWRCTL[3]) is set to 1, this SRAM is auto set to power shut down mode by hardware.
     * |        |          |Note 4: These bits are not retained when D3 power is turned off.
     * |[31]    |PCBUSY    |Power Changing Busy Flag (Read Only)
     * |        |          |If PMC_LPSYSRPC be written, this bit be asserted automatic by hardware, and be de-asserted when SRAM power change procedure is finished.
     * |        |          |0 = Low power domain SRAM bank 3 power change finished.
     * |        |          |1 = Low power domain power changing. The PMC_SYSLPRPC write operations are ignored.
     * |        |          |Note 1: This bit is not retained when D3 power is turned off.
     */
    __IO uint32_t PWRCTL;                /*!< [0x0000] System Power-down Control Register                               */
    __IO uint32_t INTEN;                 /*!< [0x0004] PMC Interrupt Enable Register                                    */
    __IO uint32_t INTSTS;                /*!< [0x0008] PMC Interrupt Status Register                                    */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t PLCTL;                 /*!< [0x0010] Power Level Control Register                                     */
    __IO uint32_t PLSTS;                 /*!< [0x0014] Power Level Status Register                                      */
    __IO uint32_t VRCTL;                 /*!< [0x0018] Voltage Regulator Control Register                               */
    __IO uint32_t VRSTS;                 /*!< [0x001c] Voltage Regulator Status Register                                */
    __IO uint32_t IOSHCTL;               /*!< [0x0020] GPIO Status Hold Control Register                                */
    __IO uint32_t FSHPCTL;               /*!< [0x0024] FLASH Power Mode Control Register                                */
    __IO uint32_t BATLDCTL;              /*!< [0x0028] Battery Loss Detector Control Register                           */
    __I  uint32_t RESERVE1[1];
    __IO uint32_t IOTGDBCTL;             /*!< [0x0030] GPIO Trigger De-bounce Control Register                          */
    __IO uint32_t GPATGCTL;              /*!< [0x0034] GPA Trigger Control Register                                     */
    __IO uint32_t GPBTGCTL;              /*!< [0x0038] GPB Trigger Control Register                                     */
    __IO uint32_t GPCTGCTL;              /*!< [0x003c] GPC Trigger Control Register                                     */
    __IO uint32_t GPDTGCTL;              /*!< [0x0040] GPD Trigger Control Register                                     */
    __I  uint32_t RESERVE2[3];
    __IO uint32_t STMRWKCTL;             /*!< [0x0050] Standby Timer Wake-up Control Register                           */
    __I  uint32_t RESERVE3[3];
    __IO uint32_t PINWKCTL;              /*!< [0x0060] Pin Wake-up Control Register                                     */
    __I  uint32_t RESERVE4[39];
    __IO uint32_t SYSRB0PC;              /*!< [0x0100] System SRAM Bank 0 Power Mode Control Register                   */
    __IO uint32_t SYSRB1PC;              /*!< [0x0104] System SRAM Bank 1 Power Mode Control Register                   */
    __IO uint32_t SYSRB2PC;              /*!< [0x0108] System SRAM Bank 2 Power Mode Control Register                   */
    __I  uint32_t RESERVE5[1];
    __IO uint32_t LPSYSRPC;              /*!< [0x0110] Low Power Domain System SRAM Power Mode Control Register         */

} PMC_T;

/**
    @addtogroup PMC_CONST PMC Bit Field Definition
    Constant Definitions for PMC Controller
@{ */

#define PMC_PWRCTL_D0PGEN_Pos            (0)                                               /*!< PMC_T::PWRCTL: D0PGEN Position         */
#define PMC_PWRCTL_D0PGEN_Msk            (0x1ul << PMC_PWRCTL_D0PGEN_Pos)                  /*!< PMC_T::PWRCTL: D0PGEN Mask             */

#define PMC_PWRCTL_D1PGEN_Pos            (1)                                               /*!< PMC_T::PWRCTL: D1PGEN Position         */
#define PMC_PWRCTL_D1PGEN_Msk            (0x1ul << PMC_PWRCTL_D1PGEN_Pos)                  /*!< PMC_T::PWRCTL: D1PGEN Mask             */

#define PMC_PWRCTL_D2PGEN_Pos            (2)                                               /*!< PMC_T::PWRCTL: D2PGEN Position         */
#define PMC_PWRCTL_D2PGEN_Msk            (0x1ul << PMC_PWRCTL_D2PGEN_Pos)                  /*!< PMC_T::PWRCTL: D2PGEN Mask             */

#define PMC_PWRCTL_D3PGEN_Pos            (3)                                               /*!< PMC_T::PWRCTL: D3PGEN Position         */
#define PMC_PWRCTL_D3PGEN_Msk            (0x1ul << PMC_PWRCTL_D3PGEN_Pos)                  /*!< PMC_T::PWRCTL: D3PGEN Mask             */

#define PMC_PWRCTL_PDEN_Pos              (8)                                               /*!< PMC_T::PWRCTL: PDEN Position           */
#define PMC_PWRCTL_PDEN_Msk              (0x1ul << PMC_PWRCTL_PDEN_Pos)                    /*!< PMC_T::PWRCTL: PDEN Mask               */

#define PMC_PWRCTL_VDROPEN_Pos           (10)                                              /*!< PMC_T::PWRCTL: VDROPEN Position        */
#define PMC_PWRCTL_VDROPEN_Msk           (0x1ul << PMC_PWRCTL_VDROPEN_Pos)                 /*!< PMC_T::PWRCTL: VDROPEN Mask            */

#define PMC_PWRCTL_FWEN_Pos              (11)                                              /*!< PMC_T::PWRCTL: FWEN Position           */
#define PMC_PWRCTL_FWEN_Msk              (0x1ul << PMC_PWRCTL_FWEN_Pos)                    /*!< PMC_T::PWRCTL: FWEN Mask               */

#define PMC_PWRCTL_PRSTDBEN_Pos          (12)                                              /*!< PMC_T::PWRCTL: PRSTDBEN Position       */
#define PMC_PWRCTL_PRSTDBEN_Msk          (0x1ul << PMC_PWRCTL_PRSTDBEN_Pos)                /*!< PMC_T::PWRCTL: PRSTDBEN Mask           */

#define PMC_PWRCTL_WRBUSY_Pos            (31)                                              /*!< PMC_T::PWRCTL: WRBUSY Position         */
#define PMC_PWRCTL_WRBUSY_Msk            (0x1ul << PMC_PWRCTL_WRBUSY_Pos)                  /*!< PMC_T::PWRCTL: WRBUSY Mask             */

#define PMC_INTEN_PDWKIEN_Pos            (0)                                               /*!< PMC_T::INTEN: PDWKIEN Position         */
#define PMC_INTEN_PDWKIEN_Msk            (0x1ul << PMC_INTEN_PDWKIEN_Pos)                  /*!< PMC_T::INTEN: PDWKIEN Mask             */

#define PMC_INTEN_STMRWKIEN_Pos          (1)                                               /*!< PMC_T::INTEN: STMRWKIEN Position       */
#define PMC_INTEN_STMRWKIEN_Msk          (0x1ul << PMC_INTEN_STMRWKIEN_Pos)                /*!< PMC_T::INTEN: STMRWKIEN Mask           */

#define PMC_INTEN_PIN0WKIEN_Pos          (8)                                               /*!< PMC_T::INTEN: PIN0WKIEN Position       */
#define PMC_INTEN_PIN0WKIEN_Msk          (0x1ul << PMC_INTEN_PIN0WKIEN_Pos)                /*!< PMC_T::INTEN: PIN0WKIEN Mask           */

#define PMC_INTEN_PIN1WKIEN_Pos          (9)                                               /*!< PMC_T::INTEN: PIN1WKIEN Position       */
#define PMC_INTEN_PIN1WKIEN_Msk          (0x1ul << PMC_INTEN_PIN1WKIEN_Pos)                /*!< PMC_T::INTEN: PIN1WKIEN Mask           */

#define PMC_INTEN_PIN2WKIEN_Pos          (10)                                              /*!< PMC_T::INTEN: PIN2WKIEN Position       */
#define PMC_INTEN_PIN2WKIEN_Msk          (0x1ul << PMC_INTEN_PIN2WKIEN_Pos)                /*!< PMC_T::INTEN: PIN2WKIEN Mask           */

#define PMC_INTEN_PIN3WKIEN_Pos          (11)                                              /*!< PMC_T::INTEN: PIN3WKIEN Position       */
#define PMC_INTEN_PIN3WKIEN_Msk          (0x1ul << PMC_INTEN_PIN3WKIEN_Pos)                /*!< PMC_T::INTEN: PIN3WKIEN Mask           */

#define PMC_INTEN_PIN4WKIEN_Pos          (12)                                              /*!< PMC_T::INTEN: PIN4WKIEN Position       */
#define PMC_INTEN_PIN4WKIEN_Msk          (0x1ul << PMC_INTEN_PIN4WKIEN_Pos)                /*!< PMC_T::INTEN: PIN4WKIEN Mask           */

#define PMC_INTEN_PIN5WKIEN_Pos          (13)                                              /*!< PMC_T::INTEN: PIN5WKIEN Position       */
#define PMC_INTEN_PIN5WKIEN_Msk          (0x1ul << PMC_INTEN_PIN5WKIEN_Pos)                /*!< PMC_T::INTEN: PIN5WKIEN Mask           */

#define PMC_INTEN_GPATGWKIEN_Pos         (16)                                              /*!< PMC_T::INTEN: GPATGWKIEN Position      */
#define PMC_INTEN_GPATGWKIEN_Msk         (0x1ul << PMC_INTEN_GPATGWKIEN_Pos)               /*!< PMC_T::INTEN: GPATGWKIEN Mask          */

#define PMC_INTEN_GPBTGWKIEN_Pos         (17)                                              /*!< PMC_T::INTEN: GPBTGWKIEN Position      */
#define PMC_INTEN_GPBTGWKIEN_Msk         (0x1ul << PMC_INTEN_GPBTGWKIEN_Pos)               /*!< PMC_T::INTEN: GPBTGWKIEN Mask          */

#define PMC_INTEN_GPCTGWKIEN_Pos         (18)                                              /*!< PMC_T::INTEN: GPCTGWKIEN Position      */
#define PMC_INTEN_GPCTGWKIEN_Msk         (0x1ul << PMC_INTEN_GPCTGWKIEN_Pos)               /*!< PMC_T::INTEN: GPCTGWKIEN Mask          */

#define PMC_INTEN_GPDTGWKIEN_Pos         (19)                                              /*!< PMC_T::INTEN: GPDTGWKIEN Position      */
#define PMC_INTEN_GPDTGWKIEN_Msk         (0x1ul << PMC_INTEN_GPDTGWKIEN_Pos)               /*!< PMC_T::INTEN: GPDTGWKIEN Mask          */

#define PMC_INTSTS_PDWKIF_Pos            (0)                                               /*!< PMC_T::INTSTS: PDWKIF Position         */
#define PMC_INTSTS_PDWKIF_Msk            (0x1ul << PMC_INTSTS_PDWKIF_Pos)                  /*!< PMC_T::INTSTS: PDWKIF Mask             */

#define PMC_INTSTS_STMRWKIF_Pos          (1)                                               /*!< PMC_T::INTSTS: STMRWKIF Position       */
#define PMC_INTSTS_STMRWKIF_Msk          (0x1ul << PMC_INTSTS_STMRWKIF_Pos)                /*!< PMC_T::INTSTS: STMRWKIF Mask           */

#define PMC_INTSTS_PIN0WKIF_Pos          (8)                                               /*!< PMC_T::INTSTS: PIN0WKIF Position       */
#define PMC_INTSTS_PIN0WKIF_Msk          (0x1ul << PMC_INTSTS_PIN0WKIF_Pos)                /*!< PMC_T::INTSTS: PIN0WKIF Mask           */

#define PMC_INTSTS_PIN1WKIF_Pos          (9)                                               /*!< PMC_T::INTSTS: PIN1WKIF Position       */
#define PMC_INTSTS_PIN1WKIF_Msk          (0x1ul << PMC_INTSTS_PIN1WKIF_Pos)                /*!< PMC_T::INTSTS: PIN1WKIF Mask           */

#define PMC_INTSTS_PIN2WKIF_Pos          (10)                                              /*!< PMC_T::INTSTS: PIN2WKIF Position       */
#define PMC_INTSTS_PIN2WKIF_Msk          (0x1ul << PMC_INTSTS_PIN2WKIF_Pos)                /*!< PMC_T::INTSTS: PIN2WKIF Mask           */

#define PMC_INTSTS_PIN3WKIF_Pos          (11)                                              /*!< PMC_T::INTSTS: PIN3WKIF Position       */
#define PMC_INTSTS_PIN3WKIF_Msk          (0x1ul << PMC_INTSTS_PIN3WKIF_Pos)                /*!< PMC_T::INTSTS: PIN3WKIF Mask           */

#define PMC_INTSTS_PIN4WKIF_Pos          (12)                                              /*!< PMC_T::INTSTS: PIN4WKIF Position       */
#define PMC_INTSTS_PIN4WKIF_Msk          (0x1ul << PMC_INTSTS_PIN4WKIF_Pos)                /*!< PMC_T::INTSTS: PIN4WKIF Mask           */

#define PMC_INTSTS_PIN5WKIF_Pos          (13)                                              /*!< PMC_T::INTSTS: PIN5WKIF Position       */
#define PMC_INTSTS_PIN5WKIF_Msk          (0x1ul << PMC_INTSTS_PIN5WKIF_Pos)                /*!< PMC_T::INTSTS: PIN5WKIF Mask           */

#define PMC_INTSTS_GPATGWKIF_Pos         (16)                                              /*!< PMC_T::INTSTS: GPATGWKIF Position      */
#define PMC_INTSTS_GPATGWKIF_Msk         (0x1ul << PMC_INTSTS_GPATGWKIF_Pos)               /*!< PMC_T::INTSTS: GPATGWKIF Mask          */

#define PMC_INTSTS_GPBTGWKIF_Pos         (17)                                              /*!< PMC_T::INTSTS: GPBTGWKIF Position      */
#define PMC_INTSTS_GPBTGWKIF_Msk         (0x1ul << PMC_INTSTS_GPBTGWKIF_Pos)               /*!< PMC_T::INTSTS: GPBTGWKIF Mask          */

#define PMC_INTSTS_GPCTGWKIF_Pos         (18)                                              /*!< PMC_T::INTSTS: GPCTGWKIF Position      */
#define PMC_INTSTS_GPCTGWKIF_Msk         (0x1ul << PMC_INTSTS_GPCTGWKIF_Pos)               /*!< PMC_T::INTSTS: GPCTGWKIF Mask          */

#define PMC_INTSTS_GPDTGWKIF_Pos         (19)                                              /*!< PMC_T::INTSTS: GPDTGWKIF Position      */
#define PMC_INTSTS_GPDTGWKIF_Msk         (0x1ul << PMC_INTSTS_GPDTGWKIF_Pos)               /*!< PMC_T::INTSTS: GPDTGWKIF Mask          */

#define PMC_INTSTS_CLRWK_Pos             (31)                                              /*!< PMC_T::INTSTS: CLRWK Position          */
#define PMC_INTSTS_CLRWK_Msk             (0x1ul << PMC_INTSTS_CLRWK_Pos)                   /*!< PMC_T::INTSTS: CLRWK Mask              */

#define PMC_PLCTL_PLSEL_Pos              (0)                                               /*!< PMC_T::PLCTL: PLSEL Position           */
#define PMC_PLCTL_PLSEL_Msk              (0x1ul << PMC_PLCTL_PLSEL_Pos)                    /*!< PMC_T::PLCTL: PLSEL Mask               */

#define PMC_PLCTL_WRBUSY_Pos             (31)                                              /*!< PMC_T::PLCTL: WRBUSY Position          */
#define PMC_PLCTL_WRBUSY_Msk             (0x1ul << PMC_PLCTL_WRBUSY_Pos)                   /*!< PMC_T::PLCTL: WRBUSY Mask              */

#define PMC_PLSTS_PLSTATUS_Pos           (0)                                               /*!< PMC_T::PLSTS: PLSTATUS Position        */
#define PMC_PLSTS_PLSTATUS_Msk           (0x1ul << PMC_PLSTS_PLSTATUS_Pos)                 /*!< PMC_T::PLSTS: PLSTATUS Mask            */

#define PMC_PLSTS_PLCBUSY_Pos            (31)                                              /*!< PMC_T::PLSTS: PLCBUSY Position         */
#define PMC_PLSTS_PLCBUSY_Msk            (0x1ul << PMC_PLSTS_PLCBUSY_Pos)                  /*!< PMC_T::PLSTS: PLCBUSY Mask             */

#define PMC_VRCTL_MVRS_Pos               (0)                                               /*!< PMC_T::VRCTL: MVRS Position            */
#define PMC_VRCTL_MVRS_Msk               (0x1ul << PMC_VRCTL_MVRS_Pos)                     /*!< PMC_T::VRCTL: MVRS Mask                */

#define PMC_VRCTL_WRBUSY_Pos             (31)                                              /*!< PMC_T::VRCTL: WRBUSY Position          */
#define PMC_VRCTL_WRBUSY_Msk             (0x1ul << PMC_VRCTL_WRBUSY_Pos)                   /*!< PMC_T::VRCTL: WRBUSY Mask              */

#define PMC_VRSTS_CURMVR_Pos             (0)                                               /*!< PMC_T::VRSTS: CURMVR Position          */
#define PMC_VRSTS_CURMVR_Msk             (0x1ul << PMC_VRSTS_CURMVR_Pos)                   /*!< PMC_T::VRSTS: CURMVR Mask              */

#define PMC_VRSTS_MVRCERR_Pos            (1)                                               /*!< PMC_T::VRSTS: MVRCERR Position         */
#define PMC_VRSTS_MVRCERR_Msk            (0x1ul << PMC_VRSTS_MVRCERR_Pos)                  /*!< PMC_T::VRSTS: MVRCERR Mask             */

#define PMC_VRSTS_LCONS_Pos              (2)                                               /*!< PMC_T::VRSTS: LCONS Position           */
#define PMC_VRSTS_LCONS_Msk              (0x1ul << PMC_VRSTS_LCONS_Pos)                    /*!< PMC_T::VRSTS: LCONS Mask               */

#define PMC_VRSTS_MVRCBUSY_Pos           (31)                                              /*!< PMC_T::VRSTS: MVRCBUSY Position        */
#define PMC_VRSTS_MVRCBUSY_Msk           (0x1ul << PMC_VRSTS_MVRCBUSY_Pos)                 /*!< PMC_T::VRSTS: MVRCBUSY Mask            */

#define PMC_IOSHCTL_IOHR_Pos             (0)                                               /*!< PMC_T::IOSHCTL: IOHR Position          */
#define PMC_IOSHCTL_IOHR_Msk             (0x1ul << PMC_IOSHCTL_IOHR_Pos)                   /*!< PMC_T::IOSHCTL: IOHR Mask              */

#define PMC_BATLDCTL_BATLDEN_Pos         (0)                                               /*!< PMC_T::BATLDCTL: BATLDEN Position      */
#define PMC_BATLDCTL_BATLDEN_Msk         (0x1ul << PMC_BATLDCTL_BATLDEN_Pos)               /*!< PMC_T::BATLDCTL: BATLDEN Mask          */

#define PMC_IOTGDBCTL_IOTGDBSEL_Pos      (0)                                               /*!< PMC_T::IOTGDBCTL: IOTGDBSEL Position   */
#define PMC_IOTGDBCTL_IOTGDBSEL_Msk      (0xful << PMC_IOTGDBCTL_IOTGDBSEL_Pos)            /*!< PMC_T::IOTGDBCTL: IOTGDBSEL Mask       */

#define PMC_GPATGCTL_TGPWKEN_Pos         (0)                                               /*!< PMC_T::GPATGCTL: TGPWKEN Position      */
#define PMC_GPATGCTL_TGPWKEN_Msk         (0x1ul << PMC_GPATGCTL_TGPWKEN_Pos)               /*!< PMC_T::GPATGCTL: TGPWKEN Mask          */

#define PMC_GPATGCTL_TGPREN_Pos          (1)                                               /*!< PMC_T::GPATGCTL: TGPREN Position       */
#define PMC_GPATGCTL_TGPREN_Msk          (0x1ul << PMC_GPATGCTL_TGPREN_Pos)                /*!< PMC_T::GPATGCTL: TGPREN Mask           */

#define PMC_GPATGCTL_TGPFEN_Pos          (2)                                               /*!< PMC_T::GPATGCTL: TGPFEN Position       */
#define PMC_GPATGCTL_TGPFEN_Msk          (0x1ul << PMC_GPATGCTL_TGPFEN_Pos)                /*!< PMC_T::GPATGCTL: TGPFEN Mask           */

#define PMC_GPATGCTL_TGPDBEN_Pos         (3)                                               /*!< PMC_T::GPATGCTL: TGPDBEN Position      */
#define PMC_GPATGCTL_TGPDBEN_Msk         (0x1ul << PMC_GPATGCTL_TGPDBEN_Pos)               /*!< PMC_T::GPATGCTL: TGPDBEN Mask          */

#define PMC_GPATGCTL_TGPSEL_Pos          (4)                                               /*!< PMC_T::GPATGCTL: TGPSEL Position       */
#define PMC_GPATGCTL_TGPSEL_Msk          (0xful << PMC_GPATGCTL_TGPSEL_Pos)                /*!< PMC_T::GPATGCTL: TGPSEL Mask           */

#define PMC_GPBTGCTL_TGPWKEN_Pos         (0)                                               /*!< PMC_T::GPBTGCTL: TGPWKEN Position      */
#define PMC_GPBTGCTL_TGPWKEN_Msk         (0x1ul << PMC_GPBTGCTL_TGPWKEN_Pos)               /*!< PMC_T::GPBTGCTL: TGPWKEN Mask          */

#define PMC_GPBTGCTL_TGPREN_Pos          (1)                                               /*!< PMC_T::GPBTGCTL: TGPREN Position       */
#define PMC_GPBTGCTL_TGPREN_Msk          (0x1ul << PMC_GPBTGCTL_TGPREN_Pos)                /*!< PMC_T::GPBTGCTL: TGPREN Mask           */

#define PMC_GPBTGCTL_TGPFEN_Pos          (2)                                               /*!< PMC_T::GPBTGCTL: TGPFEN Position       */
#define PMC_GPBTGCTL_TGPFEN_Msk          (0x1ul << PMC_GPBTGCTL_TGPFEN_Pos)                /*!< PMC_T::GPBTGCTL: TGPFEN Mask           */

#define PMC_GPBTGCTL_TGPDBEN_Pos         (3)                                               /*!< PMC_T::GPBTGCTL: TGPDBEN Position      */
#define PMC_GPBTGCTL_TGPDBEN_Msk         (0x1ul << PMC_GPBTGCTL_TGPDBEN_Pos)               /*!< PMC_T::GPBTGCTL: TGPDBEN Mask          */

#define PMC_GPBTGCTL_TGPSEL_Pos          (4)                                               /*!< PMC_T::GPBTGCTL: TGPSEL Position       */
#define PMC_GPBTGCTL_TGPSEL_Msk          (0xful << PMC_GPBTGCTL_TGPSEL_Pos)                /*!< PMC_T::GPBTGCTL: TGPSEL Mask           */

#define PMC_GPCTGCTL_TGPWKEN_Pos         (0)                                               /*!< PMC_T::GPCTGCTL: TGPWKEN Position      */
#define PMC_GPCTGCTL_TGPWKEN_Msk         (0x1ul << PMC_GPCTGCTL_TGPWKEN_Pos)               /*!< PMC_T::GPCTGCTL: TGPWKEN Mask          */

#define PMC_GPCTGCTL_TGPREN_Pos          (1)                                               /*!< PMC_T::GPCTGCTL: TGPREN Position       */
#define PMC_GPCTGCTL_TGPREN_Msk          (0x1ul << PMC_GPCTGCTL_TGPREN_Pos)                /*!< PMC_T::GPCTGCTL: TGPREN Mask           */

#define PMC_GPCTGCTL_TGPFEN_Pos          (2)                                               /*!< PMC_T::GPCTGCTL: TGPFEN Position       */
#define PMC_GPCTGCTL_TGPFEN_Msk          (0x1ul << PMC_GPCTGCTL_TGPFEN_Pos)                /*!< PMC_T::GPCTGCTL: TGPFEN Mask           */

#define PMC_GPCTGCTL_TGPDBEN_Pos         (3)                                               /*!< PMC_T::GPCTGCTL: TGPDBEN Position      */
#define PMC_GPCTGCTL_TGPDBEN_Msk         (0x1ul << PMC_GPCTGCTL_TGPDBEN_Pos)               /*!< PMC_T::GPCTGCTL: TGPDBEN Mask          */

#define PMC_GPCTGCTL_TGPSEL_Pos          (4)                                               /*!< PMC_T::GPCTGCTL: TGPSEL Position       */
#define PMC_GPCTGCTL_TGPSEL_Msk          (0xful << PMC_GPCTGCTL_TGPSEL_Pos)                /*!< PMC_T::GPCTGCTL: TGPSEL Mask           */

#define PMC_GPDTGCTL_TGPWKEN_Pos         (0)                                               /*!< PMC_T::GPDTGCTL: TGPWKEN Position      */
#define PMC_GPDTGCTL_TGPWKEN_Msk         (0x1ul << PMC_GPDTGCTL_TGPWKEN_Pos)               /*!< PMC_T::GPDTGCTL: TGPWKEN Mask          */

#define PMC_GPDTGCTL_TGPREN_Pos          (1)                                               /*!< PMC_T::GPDTGCTL: TGPREN Position       */
#define PMC_GPDTGCTL_TGPREN_Msk          (0x1ul << PMC_GPDTGCTL_TGPREN_Pos)                /*!< PMC_T::GPDTGCTL: TGPREN Mask           */

#define PMC_GPDTGCTL_TGPFEN_Pos          (2)                                               /*!< PMC_T::GPDTGCTL: TGPFEN Position       */
#define PMC_GPDTGCTL_TGPFEN_Msk          (0x1ul << PMC_GPDTGCTL_TGPFEN_Pos)                /*!< PMC_T::GPDTGCTL: TGPFEN Mask           */

#define PMC_GPDTGCTL_TGPDBEN_Pos         (3)                                               /*!< PMC_T::GPDTGCTL: TGPDBEN Position      */
#define PMC_GPDTGCTL_TGPDBEN_Msk         (0x1ul << PMC_GPDTGCTL_TGPDBEN_Pos)               /*!< PMC_T::GPDTGCTL: TGPDBEN Mask          */

#define PMC_GPDTGCTL_TGPSEL_Pos          (4)                                               /*!< PMC_T::GPDTGCTL: TGPSEL Position       */
#define PMC_GPDTGCTL_TGPSEL_Msk          (0xful << PMC_GPDTGCTL_TGPSEL_Pos)                /*!< PMC_T::GPDTGCTL: TGPSEL Mask           */

#define PMC_STMRWKCTL_STMREN_Pos         (0)                                               /*!< PMC_T::STMRWKCTL: STMREN Position      */
#define PMC_STMRWKCTL_STMREN_Msk         (0x1ul << PMC_STMRWKCTL_STMREN_Pos)               /*!< PMC_T::STMRWKCTL: STMREN Mask          */

#define PMC_STMRWKCTL_STMRMOD_Pos        (1)                                               /*!< PMC_T::STMRWKCTL: STMRMOD Position     */
#define PMC_STMRWKCTL_STMRMOD_Msk        (0x1ul << PMC_STMRWKCTL_STMRMOD_Pos)              /*!< PMC_T::STMRWKCTL: STMRMOD Mask         */

#define PMC_STMRWKCTL_STMRIS_Pos         (8)                                               /*!< PMC_T::STMRWKCTL: STMRIS Position      */
#define PMC_STMRWKCTL_STMRIS_Msk         (0xful << PMC_STMRWKCTL_STMRIS_Pos)               /*!< PMC_T::STMRWKCTL: STMRIS Mask          */

#define PMC_STMRWKCTL_WRBUSY_Pos         (31)                                              /*!< PMC_T::STMRWKCTL: WRBUSY Position      */
#define PMC_STMRWKCTL_WRBUSY_Msk         (0x1ul << PMC_STMRWKCTL_WRBUSY_Pos)               /*!< PMC_T::STMRWKCTL: WRBUSY Mask          */

#define PMC_PINWKCTL_WKPINEN0_Pos        (0)                                               /*!< PMC_T::PINWKCTL: WKPINEN0 Position     */
#define PMC_PINWKCTL_WKPINEN0_Msk        (0x3ul << PMC_PINWKCTL_WKPINEN0_Pos)              /*!< PMC_T::PINWKCTL: WKPINEN0 Mask         */

#define PMC_PINWKCTL_WKPINEN1_Pos        (2)                                               /*!< PMC_T::PINWKCTL: WKPINEN1 Position     */
#define PMC_PINWKCTL_WKPINEN1_Msk        (0x3ul << PMC_PINWKCTL_WKPINEN1_Pos)              /*!< PMC_T::PINWKCTL: WKPINEN1 Mask         */

#define PMC_PINWKCTL_WKPINEN2_Pos        (4)                                               /*!< PMC_T::PINWKCTL: WKPINEN2 Position     */
#define PMC_PINWKCTL_WKPINEN2_Msk        (0x3ul << PMC_PINWKCTL_WKPINEN2_Pos)              /*!< PMC_T::PINWKCTL: WKPINEN2 Mask         */

#define PMC_PINWKCTL_WKPINEN3_Pos        (6)                                               /*!< PMC_T::PINWKCTL: WKPINEN3 Position     */
#define PMC_PINWKCTL_WKPINEN3_Msk        (0x3ul << PMC_PINWKCTL_WKPINEN3_Pos)              /*!< PMC_T::PINWKCTL: WKPINEN3 Mask         */

#define PMC_PINWKCTL_WKPINEN4_Pos        (8)                                               /*!< PMC_T::PINWKCTL: WKPINEN4 Position     */
#define PMC_PINWKCTL_WKPINEN4_Msk        (0x3ul << PMC_PINWKCTL_WKPINEN4_Pos)              /*!< PMC_T::PINWKCTL: WKPINEN4 Mask         */

#define PMC_PINWKCTL_WKPINEN5_Pos        (10)                                              /*!< PMC_T::PINWKCTL: WKPINEN5 Position     */
#define PMC_PINWKCTL_WKPINEN5_Msk        (0x3ul << PMC_PINWKCTL_WKPINEN5_Pos)              /*!< PMC_T::PINWKCTL: WKPINEN5 Mask         */

#define PMC_SYSRB0PC_SRAM0PMS_Pos        (0)                                               /*!< PMC_T::SYSRB0PC: SRAM0PMS Position     */
#define PMC_SYSRB0PC_SRAM0PMS_Msk        (0x3ul << PMC_SYSRB0PC_SRAM0PMS_Pos)              /*!< PMC_T::SYSRB0PC: SRAM0PMS Mask         */

#define PMC_SYSRB0PC_SRAM1PMS_Pos        (2)                                               /*!< PMC_T::SYSRB0PC: SRAM1PMS Position     */
#define PMC_SYSRB0PC_SRAM1PMS_Msk        (0x3ul << PMC_SYSRB0PC_SRAM1PMS_Pos)              /*!< PMC_T::SYSRB0PC: SRAM1PMS Mask         */

#define PMC_SYSRB0PC_SRAM2PMS_Pos        (4)                                               /*!< PMC_T::SYSRB0PC: SRAM2PMS Position     */
#define PMC_SYSRB0PC_SRAM2PMS_Msk        (0x3ul << PMC_SYSRB0PC_SRAM2PMS_Pos)              /*!< PMC_T::SYSRB0PC: SRAM2PMS Mask         */

#define PMC_SYSRB0PC_SRAM3PMS_Pos        (6)                                               /*!< PMC_T::SYSRB0PC: SRAM3PMS Position     */
#define PMC_SYSRB0PC_SRAM3PMS_Msk        (0x3ul << PMC_SYSRB0PC_SRAM3PMS_Pos)              /*!< PMC_T::SYSRB0PC: SRAM3PMS Mask         */

#define PMC_SYSRB0PC_SRAM4PMS_Pos        (8)                                               /*!< PMC_T::SYSRB0PC: SRAM4PMS Position     */
#define PMC_SYSRB0PC_SRAM4PMS_Msk        (0x3ul << PMC_SYSRB0PC_SRAM4PMS_Pos)              /*!< PMC_T::SYSRB0PC: SRAM4PMS Mask         */

#define PMC_SYSRB0PC_SRAM5PMS_Pos        (10)                                              /*!< PMC_T::SYSRB0PC: SRAM5PMS Position     */
#define PMC_SYSRB0PC_SRAM5PMS_Msk        (0x3ul << PMC_SYSRB0PC_SRAM5PMS_Pos)              /*!< PMC_T::SYSRB0PC: SRAM5PMS Mask         */

#define PMC_SYSRB0PC_SRAM6PMS_Pos        (12)                                              /*!< PMC_T::SYSRB0PC: SRAM6PMS Position     */
#define PMC_SYSRB0PC_SRAM6PMS_Msk        (0x3ul << PMC_SYSRB0PC_SRAM6PMS_Pos)              /*!< PMC_T::SYSRB0PC: SRAM6PMS Mask         */

#define PMC_SYSRB0PC_SRAM7PMS_Pos        (14)                                              /*!< PMC_T::SYSRB0PC: SRAM7PMS Position     */
#define PMC_SYSRB0PC_SRAM7PMS_Msk        (0x3ul << PMC_SYSRB0PC_SRAM7PMS_Pos)              /*!< PMC_T::SYSRB0PC: SRAM7PMS Mask         */

#define PMC_SYSRB0PC_PCBUSY_Pos          (31)                                              /*!< PMC_T::SYSRB0PC: PCBUSY Position       */
#define PMC_SYSRB0PC_PCBUSY_Msk          (0x1ul << PMC_SYSRB0PC_PCBUSY_Pos)                /*!< PMC_T::SYSRB0PC: PCBUSY Mask           */

#define PMC_SYSRB1PC_SRAM0PMS_Pos        (0)                                               /*!< PMC_T::SYSRB1PC: SRAM0PMS Position     */
#define PMC_SYSRB1PC_SRAM0PMS_Msk        (0x3ul << PMC_SYSRB1PC_SRAM0PMS_Pos)              /*!< PMC_T::SYSRB1PC: SRAM0PMS Mask         */

#define PMC_SYSRB1PC_SRAM1PMS_Pos        (2)                                               /*!< PMC_T::SYSRB1PC: SRAM1PMS Position     */
#define PMC_SYSRB1PC_SRAM1PMS_Msk        (0x3ul << PMC_SYSRB1PC_SRAM1PMS_Pos)              /*!< PMC_T::SYSRB1PC: SRAM1PMS Mask         */

#define PMC_SYSRB1PC_SRAM2PMS_Pos        (4)                                               /*!< PMC_T::SYSRB1PC: SRAM2PMS Position     */
#define PMC_SYSRB1PC_SRAM2PMS_Msk        (0x3ul << PMC_SYSRB1PC_SRAM2PMS_Pos)              /*!< PMC_T::SYSRB1PC: SRAM2PMS Mask         */

#define PMC_SYSRB1PC_SRAM3PMS_Pos        (6)                                               /*!< PMC_T::SYSRB1PC: SRAM3PMS Position     */
#define PMC_SYSRB1PC_SRAM3PMS_Msk        (0x3ul << PMC_SYSRB1PC_SRAM3PMS_Pos)              /*!< PMC_T::SYSRB1PC: SRAM3PMS Mask         */

#define PMC_SYSRB1PC_SRAM4PMS_Pos        (8)                                               /*!< PMC_T::SYSRB1PC: SRAM4PMS Position     */
#define PMC_SYSRB1PC_SRAM4PMS_Msk        (0x3ul << PMC_SYSRB1PC_SRAM4PMS_Pos)              /*!< PMC_T::SYSRB1PC: SRAM4PMS Mask         */

#define PMC_SYSRB1PC_SRAM5PMS_Pos        (10)                                              /*!< PMC_T::SYSRB1PC: SRAM5PMS Position     */
#define PMC_SYSRB1PC_SRAM5PMS_Msk        (0x3ul << PMC_SYSRB1PC_SRAM5PMS_Pos)              /*!< PMC_T::SYSRB1PC: SRAM5PMS Mask         */

#define PMC_SYSRB1PC_SRAM6PMS_Pos        (12)                                              /*!< PMC_T::SYSRB1PC: SRAM6PMS Position     */
#define PMC_SYSRB1PC_SRAM6PMS_Msk        (0x3ul << PMC_SYSRB1PC_SRAM6PMS_Pos)              /*!< PMC_T::SYSRB1PC: SRAM6PMS Mask         */

#define PMC_SYSRB1PC_SRAM7PMS_Pos        (14)                                              /*!< PMC_T::SYSRB1PC: SRAM7PMS Position     */
#define PMC_SYSRB1PC_SRAM7PMS_Msk        (0x3ul << PMC_SYSRB1PC_SRAM7PMS_Pos)              /*!< PMC_T::SYSRB1PC: SRAM7PMS Mask         */

#define PMC_SYSRB1PC_PCBUSY_Pos          (31)                                              /*!< PMC_T::SYSRB1PC: PCBUSY Position       */
#define PMC_SYSRB1PC_PCBUSY_Msk          (0x1ul << PMC_SYSRB1PC_PCBUSY_Pos)                /*!< PMC_T::SYSRB1PC: PCBUSY Mask           */

#define PMC_SYSRB2PC_SRAM0PMS_Pos        (0)                                               /*!< PMC_T::SYSRB2PC: SRAM0PMS Position     */
#define PMC_SYSRB2PC_SRAM0PMS_Msk        (0x3ul << PMC_SYSRB2PC_SRAM0PMS_Pos)              /*!< PMC_T::SYSRB2PC: SRAM0PMS Mask         */

#define PMC_SYSRB2PC_SRAM1PMS_Pos        (2)                                               /*!< PMC_T::SYSRB2PC: SRAM1PMS Position     */
#define PMC_SYSRB2PC_SRAM1PMS_Msk        (0x3ul << PMC_SYSRB2PC_SRAM1PMS_Pos)              /*!< PMC_T::SYSRB2PC: SRAM1PMS Mask         */

#define PMC_SYSRB2PC_SRAM2PMS_Pos        (4)                                               /*!< PMC_T::SYSRB2PC: SRAM2PMS Position     */
#define PMC_SYSRB2PC_SRAM2PMS_Msk        (0x3ul << PMC_SYSRB2PC_SRAM2PMS_Pos)              /*!< PMC_T::SYSRB2PC: SRAM2PMS Mask         */

#define PMC_SYSRB2PC_SRAM3PMS_Pos        (6)                                               /*!< PMC_T::SYSRB2PC: SRAM3PMS Position     */
#define PMC_SYSRB2PC_SRAM3PMS_Msk        (0x3ul << PMC_SYSRB2PC_SRAM3PMS_Pos)              /*!< PMC_T::SYSRB2PC: SRAM3PMS Mask         */

#define PMC_SYSRB2PC_SRAM4PMS_Pos        (8)                                               /*!< PMC_T::SYSRB2PC: SRAM4PMS Position     */
#define PMC_SYSRB2PC_SRAM4PMS_Msk        (0x3ul << PMC_SYSRB2PC_SRAM4PMS_Pos)              /*!< PMC_T::SYSRB2PC: SRAM4PMS Mask         */

#define PMC_SYSRB2PC_PCBUSY_Pos          (31)                                              /*!< PMC_T::SYSRB2PC: PCBUSY Position       */
#define PMC_SYSRB2PC_PCBUSY_Msk          (0x1ul << PMC_SYSRB2PC_PCBUSY_Pos)                /*!< PMC_T::SYSRB2PC: PCBUSY Mask           */

#define PMC_LPSYSRPC_SRAM0PMS_Pos        (0)                                               /*!< PMC_T::LPSYSRPC: SRAM0PMS Position     */
#define PMC_LPSYSRPC_SRAM0PMS_Msk        (0x3ul << PMC_LPSYSRPC_SRAM0PMS_Pos)              /*!< PMC_T::LPSYSRPC: SRAM0PMS Mask         */

#define PMC_LPSYSRPC_PCBUSY_Pos          (31)                                              /*!< PMC_T::LPSYSRPC: PCBUSY Position       */
#define PMC_LPSYSRPC_PCBUSY_Msk          (0x1ul << PMC_LPSYSRPC_PCBUSY_Pos)                /*!< PMC_T::LPSYSRPC: PCBUSY Mask           */

/** @} PMC_CONST */
/** @} end of PMC register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __PMC_REG_H__ */
