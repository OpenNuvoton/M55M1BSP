
/**************************************************************************//**
 * @file     utcpd_reg.h
 * @version  V1.00
 * @brief    UTCPD register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/



#ifndef __UTCPD_REG_H__
#define __UTCPD_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif
/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup UTCPD USB Type C Power Delivery Controller (UTCPD)
    Memory Mapped Structure for UTCPD Controller
    @{
*/

typedef struct
{


    /**
     * @var UTCPD_T::VID
     * Offset: 0x00  UTCPD Vendor ID Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |VID       |UTCPD Vendor ID
     * |        |          |Vendor identifier is used to identify the TCPC vendor, the VID is a unique 16-bit unsigned integer assigned by USB-IF to the Vendor
     * @var UTCPD_T::PID
     * Offset: 0x04  UTCPD Product ID Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PID       |UTCPD Product ID
     * |        |          |USB Product ID is used to identify the product.
     * @var UTCPD_T::DID
     * Offset: 0x08  UTCPD Device ID Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |DID       |UTCPD Device ID
     * |        |          |USB Device ID is used to identify the release version of the product.
     * @var UTCPD_T::TCREV
     * Offset: 0x0C  UTCPD USB Type C Revision Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |TCREV     |UTCPD USB Type C Revision
     * |        |          |USB Type-C Cable and Connector Specification Revision 1.3.
     * @var UTCPD_T::PDREV
     * Offset: 0x10  UTCPD USB PD Revision Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PDVER     |UTCPD USB PD Vision
     * |        |          |USB Power Delivery Specification Revision 1.1
     * |[15:8]  |PDREV     |UTCPD USB PD Revision
     * |        |          |USB Power Delivery Specification Revision 3.1.
     * @var UTCPD_T::IS
     * Offset: 0x14  UTCPD Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CCSCHIS   |CC Status Changed
     * |        |          |0 = CC status not change.
     * |        |          |1= CC status changed.
     * |        |          |Note: It is cleared by software writing 1 into this bit.
     * |[1]     |PWRSCHIS  |Power Status Changed
     * |        |          |0 = Power status not change.
     * |        |          |1 = Power status changed.
     * |        |          |Note: It is cleared by software writing 1 into this bit.
     * |[2]     |RXSOPIS   |Received SOP Message
     * |        |          |0 = No SOP message Received.
     * |        |          |1 = Received SOP message (Set after sending GoodCRC.)
     * |        |          |UTCPD_RXBCNT being set to 0 does not set this bit.
     * |        |          |Note: It is cleared by software writing 1 into this bit.
     * |[3]     |RXHRSTIS  |Received Hard Reset
     * |        |          |0 = No Hard reset Received.
     * |        |          |1 = Received Hard reset.
     * |        |          |Note: It is cleared by software writing 1 into this bit.
     * |[4]     |TXFALIS   |Transmit SOP Fail
     * |        |          |0 = No Transmit SOP fail.
     * |        |          |1 = SOP* message transmission not successful, no GoodCRC response received on SOP* message transmission
     * |        |          |Transmit SOP* message buffer registers are empty.
     * |        |          |Note: It is cleared by software writing 1 into this bit.
     * |[5]     |TXDCIS    |Transmit SOP* Message Discarded
     * |        |          |0 = No TX SOP discarded.
     * |        |          |1 = Reset or SOP* message transmission not sent due to incoming receive message
     * |        |          |Transmit SOP* message buffer registers are empty.
     * |        |          |Note: It is cleared by software writing 1 into this bit.
     * |[6]     |TXOKIS    |Transmit SOP* Message Successful
     * |        |          |0 = No TX SOP* transmit.
     * |        |          |1 = Reset or SOP* message transmission successful
     * |        |          |GoodCRC response received on SOP* message transmission
     * |        |          |Transmit SOP* message buffer registers are empty.
     * |        |          |Note: It is cleared by software writing 1 into this bit.
     * |[7]     |VBAMHIS   |VBUS Voltage Alarm High
     * |        |          |0 = No high voltage alarm has occurred.
     * |        |          |1= A high voltage alarm has occurred.
     * |        |          |This bit will be set high when DSVBAM (UTCPD_PWRCTL[5]) is low and VBUS voltage is higher than VBAMH (UTCPD_VBAMH[9:0]).
     * |        |          |Note: It is cleared by software writing 1 into this bit.
     * |[8]     |VBAMLIS   |VBUS Voltage Alarm Low
     * |        |          |0 = No Low voltage alarm has occurred.
     * |        |          |1= A Low voltage alarm has occurred.
     * |        |          |This bit will be set high when DSVBAM (UTCPD_PWRCTL[5]]) is low and VBUS voltage is lower than VBAML (UTCPD_VBAML[9:0]).
     * |        |          |Note: It is cleared by software writing 1 into this bit.
     * |[9]     |FUTIS     |Fault Occur
     * |        |          |0 = No fault occurs.
     * |        |          |1= A Fault has occurred. Read the FUT_STS register.
     * |        |          |Note: It is cleared by software writing 1 into this bit.
     * |[10]    |RXOFIS    |Rx Buffer Overflow
     * |        |          |0 = RX buffer is functioning properly.
     * |        |          |1 = RX buffer has overflowed.
     * |        |          |Writing 1 to this register acknowledges the overflow
     * |        |          |The overflow is cleared by writing 1 to RXSOPIS (UTCPD_IS[2])
     * |[11]    |SKDCDTIS  |VBUS Sink Disconnect Detected
     * |        |          |0 = No disconnect detected.
     * |        |          |1 = A VBUS Sink disconnect threshold crossing has been detected.
     * |        |          |This bit will be set when VBMONI (UTCPD_PWRCTL[6]) is enabled and VBUS voltage drop lower than SKVBDCTH (UTCPD_SKVBDCTH[9:0]).
     * |        |          |Note: It is cleared by software writing 1 into this bit.
     * |[15]    |VNDIS     |Vendor Define Event Detected
     * |        |          |0 = No vendor defined interrupt status has been detected.
     * |        |          |1 = A vendor defined interrupt status has been detected
     * |        |          |Refer the vender defined interrupt status register.
     * |        |          |Note: It is cleared by software writing 1 into this bit.
     * @var UTCPD_T::IE
     * Offset: 0x18  UTCPD Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CCSCHIE   |CC Status Changed Interrupt Enable
     * |        |          |0 = CC Status Changed Interrupt Disabled.
     * |        |          |1 = CC Status Changed Interrupt Enabled.
     * |[1]     |PWRSCHIE  |Power Status Changed Interrupt Enable
     * |        |          |0 = Power Status Changed Interrupt Disabled.
     * |        |          |1 = Power Status Changed Interrupt Enabled.
     * |[2]     |RXSOPIE   |Received SOP Message Interrupt Enable
     * |        |          |0 = Received SOP Message Interrupt Disabled.
     * |        |          |1 = Received SOP Message Interrupt Enabled.
     * |[3]     |RXHRSTIE  |Received Hard Reset Interrupt Enable
     * |        |          |0 = Received Hard Reset Interrupt Disabled.
     * |        |          |1 = Received Hard Reset Interrupt Enabled.
     * |[4]     |TXFAILIE  |Transmit SOP Fail Interrupt Enable
     * |        |          |0 = Transmit SOP Fail Interrupt Disabled.
     * |        |          |1 = Transmit SOP Fail Interrupt Enabled.
     * |[5]     |TXDCIE    |Transmit SOP* Message Discarded Interrupt Enable
     * |        |          |0 = Transmit SOP* Message Discarded Interrupt Disabled.
     * |        |          |1 = Transmit SOP* Message Discarded Interrupt Enabled.
     * |[6]     |TXSOKIE   |Transmit SOP* Message Successful Interrupt Enable
     * |        |          |0 = Transmit SOP* Message Successful Interrupt Disabled.
     * |        |          |1 = Transmit SOP* Message Successful Interrupt Enabled.
     * |[7]     |VBAMHIE   |VBUS Voltage Alarm High Interrupt Enable
     * |        |          |0 = VBUS Voltage Alarm High Interrupt Disabled.
     * |        |          |1 = VBUS Voltage Alarm High Interrupt Enabled.
     * |[8]     |VBAMLIE   |VBUS Voltage Alarm Low Interrupt Enable
     * |        |          |0 = VBUS Voltage Alarm Low Interrupt Disabled.
     * |        |          |1 = VBUS Voltage Alarm Low Interrupt Enabled.
     * |[9]     |FUTIE     |Fault Occur Interrupt Enable
     * |        |          |0 = Fault Occur Interrupt Disabled.
     * |        |          |1 = Fault Occur Interrupt Enabled.
     * |[10]    |RXOFIE    |Rx Buffer Overflow Interrupt Enable
     * |        |          |0 = Rx Buffer Overflow Interrupt Disabled.
     * |        |          |1 = Rx Buffer Overflow Interrupt Enabled.
     * |[11]    |SKDCDTIE  |VBUS Sink Disconnect Detected Interrupt Enable
     * |        |          |0 = VBUS Sink Disconnect Detected Interrupt Disabled.
     * |        |          |1 = VBUS Sink Disconnect Detected Interrupt Enabled.
     * |[15]    |VNDIE     |Vendor Define Event Detected Interrupt Enable
     * |        |          |0 = Vendor Define Event Detected Interrupt Disabled.
     * |        |          |1 = Vendor Define Event Detected Interrupt Enabled.
     * @var UTCPD_T::PWRSTSIE
     * Offset: 0x1C  UTCPD Power Status Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SKVBIE    |Sinking VBUS Status Interrupt Enable
     * |        |          |0 = Sinking VBUS Status Interrupt Disabled.
     * |        |          |1 = Sinking VBUS Status Interrupt Enabled.
     * |[1]     |VCPSIE    |VCONN Present Status Interrupt Enable
     * |        |          |0 = VCONN Present Status Interrupt Disabled.
     * |        |          |1 = VCONN Present Status Interrupt Enabled.
     * |[2]     |VBPSIE    |VBUS Present Status Interrupt Enable
     * |        |          |0 = VBUS Present Status Interrupt Disabled.
     * |        |          |1 = VBUS Present Status Interrupt Enabled.
     * |[3]     |VBDTDGIE  |VBUS Detection Status Change Interrupt Enable
     * |        |          |0 = VBUS Detection Status Change Interrupt Disabled.
     * |        |          |1 = VBUS Detection Status Change Interrupt Enabled.
     * |[4]     |SRVBIE    |Sourcing VBUS Status Interrupt Enable
     * |        |          |0 = Sourcing VBUS Status Interrupt Disabled.
     * |        |          |1 = Sourcing VBUS Status Interrupt Enabled.
     * |[5]     |SRHVIE    |Sourcing High Voltage Status Interrupt Enable
     * |        |          |0 = Sourcing High Voltage Status Interrupt Disabled.
     * |        |          |1 = Sourcing High Voltage Status Interrupt Enabled.
     * |[7]     |DACONIE   |Debug Accessory Connected Status Interrupt Enable
     * |        |          |0 = Debug Accessory Connected Status Interrupt Disabled.
     * |        |          |1 = Debug Accessory Connected Status Interrupt Enabled.
     * @var UTCPD_T::FUTSTSIE
     * Offset: 0x20  UTCPD Fault Status Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |VCOCIE    |VCONN OCP Fault Interrupt Enable
     * |        |          |0 = VCONN OCP Fault Interrupt Disabled.
     * |        |          |1 = VCONN OCP Fault Interrupt Enabled.
     * |[2]     |VBOVIE    |Internal VBUS OVP Fault Interrupt Enable
     * |        |          |0 = Internal VBUS OVP Fault Interrupt Disabled.
     * |        |          |1 = Internal VBUS OVP Fault Interrupt Enabled.
     * |[3]     |VBOCIE    |External VBUS OCP Fault Interrupt Enable
     * |        |          |0 = External VBUS OCP Fault Interrupt Disabled.
     * |        |          |1 = External VBUS OCP Fault Interrupt Enabled.
     * |[4]     |FDGFALIE  |Force Discharge Failed Interrupt Enable
     * |        |          |0 = Force Discharge Failed Interrupt Disabled.
     * |        |          |1 = Force Discharge Failed Interrupt Enabled.
     * |[5]     |ADGFALIE  |Auto Discharge Failed Interrupt Enable
     * |        |          |0 = Auto Discharge Failed Interrupt Disabled.
     * |        |          |1 = Auto Discharge Failed Interrupt Enabled.
     * |[6]     |FOFFVBIE  |Force Off VBUS Interrupt Enable
     * |        |          |0 = Force Off VBUS Interrupt Disabled.
     * |        |          |1 = Force Off VBUS Interrupt Enabled.
     * @var UTCPD_T::CTL
     * Offset: 0x24  UTCPD Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ORIENT    |Plug Orientation
     * |        |          |0 = When VCONN is enabled, apply it to the CC2 pin
     * |        |          |Monitor the CC1 pin for BMC communications if PD messaging is enabled.
     * |        |          |1 = When VCONN is enabled, apply it to the CC1 pin. Monitor the CC2 pin for BMC
     * |        |          |communications if PD messaging is enabled.
     * |[1]     |BISTEN    |BIST Test Mode
     * |        |          |Setting this bit to 1 is intended to be used only when a USB compliance tester is using USB BIST Test Data to test the PHY layer of the UTCPD
     * |        |          |The CPU should clear this bit when a detach is detected.
     * |        |          |0 = Normal Operation.
     * |        |          |1 = BIST Test Mode.
     * @var UTCPD_T::PINPL
     * Offset: 0x28  UTCPD Pin Polarity Control  Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |VBSRENPL  |VBUS Source Enable Polarity
     * |        |          |0 = Low active.
     * |        |          |1 = High active.
     * |[1]     |VBSKENPL  |VBUS Sink Enable Polarity
     * |        |          |0 = Low active.
     * |        |          |1 = High active.
     * |[2]     |VBDGENPL  |VBUS Discharge Enable Polarity
     * |        |          |0 = Low active.
     * |        |          |1 = High active.
     * |[3]     |TXFRSPL   |Fast Role Swap TX Polarity
     * |        |          |0 = Low active.
     * |        |          |1 = High active.
     * |[4]     |VCOCPL    |VCONN Overcurrent event Polarity
     * |        |          |0 = Low active.
     * |        |          |1 = High active.
     * |[5]     |VBOCPL    |VBUS Overcurrent event Polarity
     * |        |          |0 = Low active.
     * |        |          |1 = High active.
     * |[8]     |VCENPL    |VCONN Enable Polarity
     * |        |          |0 = Low active.
     * |        |          |1 = High active.
     * |[9]     |VCDGENPL  |VCONN Discharge Enable Polarity
     * |        |          |0 = Low active.
     * |        |          |1 = High active.
     * |[10]    |FOFFVBPL  |Force Off VBUS event Polarity
     * |        |          |0 = Low active.
     * |        |          |1 = High active.
     * @var UTCPD_T::ROLCTL
     * Offset: 0x2C  UTCPD Role Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |CC1       |CC1
     * |        |          |00 = Reserved.
     * |        |          |01b = Rp (Use Rp definition in RPVALUE).
     * |        |          |10 = Rd.
     * |        |          |11 = Open (Disconnect or don't care).
     * |[3:2]   |CC2       |CC2
     * |        |          |00 = Reserved.
     * |        |          |01 = Rp (Use Rp definition in RPVALUE).
     * |        |          |10 = Rd.
     * |        |          |11 = Open (Disconnect or don't care).
     * |[5:4]   |RPVALUE   |Rp Value
     * |        |          |00 = Rp default.
     * |        |          |01 = Rp 1.5A.
     * |        |          |10 = Rp 3.0A.
     * |        |          |11 = Reserved.
     * |[6]     |DRP       |DRP
     * |        |          |0 = No DRP.
     * |        |          |1 = DRP.
     * |        |          |The UTCPD toggles CC1 & CC2 after receiving LK4CON (UTCPD_CMD[5]) and until a connection is detected
     * |        |          |Upon connection, the UTCPD shall resolve to either an Rp or Rd and report the CC1/CC2 State in the CC_STS register
     * @var UTCPD_T::FUTCTL
     * Offset: 0x30  UTCPD Fault Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |VCOCDTDS  |External VCONN Overcurrent Fault Detect Disable
     * |        |          |0 = External VCONN OCP circuit Enabled.
     * |        |          |1 = External VCONN OCP circuit Disabled.
     * |[1]     |VBOVDTDS  |Internal VBUS Over Voltage Protection Fault Detect Disable
     * |        |          |0 = Internal OVP circuit Enabled.
     * |        |          |1 = Internal OVP circuit Disabled.
     * |        |          |Note: Internal VBUS over voltage protection means to use ADC to measure VBUS voltage.
     * |[2]     |VBOCDTDS  |External VBUS Overcurrent Protection Fault Detect Disable
     * |        |          |0 = External OCP circuit Enabled.
     * |        |          |1 = External OCP circuit Disabled.
     * |[3]     |VBDGTMDS  |VBUS Discharge Fault Detection Timer Disable
     * |        |          |0 = VBUS Discharge Fault Detection Timer Enabled.
     * |        |          |1 = VBUS Discharge Fault Detection Timer Disabled.
     * |        |          |This enables the timer for both force discharge and auto discharge
     * |        |          |No timer for bleed discharge
     * |        |          |Once time-out, UTCPD will compare VBUS voltage with VSAFE0V or SP_DGTH, depends on setting of ADGDC and FDGEN.
     * |[4]     |FOFFVBDS  |Force Off VBUS Disable
     * |        |          |0 = Force off VBUS Enabled.
     * |        |          |1 = Force off VBUS Disabled.
     * @var UTCPD_T::PWRCTL
     * Offset: 0x34  UTCPD Power Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |VCEN      |Enable VCONN
     * |        |          |0 = Disable VCONN Source (default).
     * |        |          |1 = Enable VCONN Source to CC.
     * |[1]     |VCPWR     |VCONN Power Supported
     * |        |          |0 = UTCPD delivers at least 1W on VCONN.
     * |        |          |1 = UTCPD delivers at least the power indicated in CPVCPWR (UTCPD_DVCAP2[3:1]).
     * |[2]     |FDGEN     |Enable Force Discharge
     * |        |          |0 = Disable forced discharge (default).
     * |        |          |1 = Enable forced discharge of VBUS.
     * |        |          |Force discharge is used for source only.
     * |        |          |This bit will only be cleared by CPU.
     * |[3]     |BDGEN     |Enable Bleed Discharge
     * |        |          |0 = Disable bleed discharge (default).
     * |        |          |1 = Enable bleed discharge of VBUS.
     * |        |          |Bleed Discharge is a low current discharge to provide a minimum load current if needed
     * |        |          |10K Ohms or 2mA recommended. Bleed discharge is used for sink only.
     * |        |          |This bit will only be cleared by CPU.
     * |[4]     |ADGDC     |Auto Discharge Disconnect
     * |        |          |0 = The UTCPD shall not automatically discharge VBUS based on VBUS voltage (default).
     * |        |          |1 = The UTCPD shall automatically discharge when disconnect detected.
     * |        |          |Setting this bit in a Source UTCPD triggers the following actions upon a disconnect detection:
     * |        |          |1. Disable sourcing power over VBUS
     * |        |          |2. VBUS discharge
     * |        |          |Sourcing power over VBUS shall be disabled before or at the same time as starting VBUS discharge.
     * |        |          |Setting this bit in a Sink UTCPD triggers the following action upon a disconnect detection:
     * |        |          |1. VBUS discharge
     * |        |          |The UTCPD shall automatically disable discharge (without clearing this bit) once the voltage on VBUS is below vSafe0V (max)
     * |        |          |UTCPD shall not re-apply discharge circuit if VBUS rises above vSafe0V.
     * |        |          |This bit will only be cleared by CPU.
     * |[5]     |DSVBAM    |Disable VBUS Voltage Alarms
     * |        |          |0 = VBUS Voltage Alarms Power status reporting Enabled.
     * |        |          |1 = VBUS Voltage Alarms Power status reporting Disabled (default).
     * |[6]     |VBMONI    |VBUS Voltage Monitor Enable
     * |        |          |0 = VBUS Voltage Monitoring Enabled.
     * |        |          |1 = VBUS Voltage Monitoring Disabled (default).
     * |        |          |Controls VBUS voltage Monitoring. UTCPD_VBVOL shall report all zeroes if disabled.
     * @var UTCPD_T::CCSTS
     * Offset: 0x38  UTCPD CC Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |CC1STATE  |CC1 State
     * |        |          |If (CC1 (UTCPD_ROLCTL[1:0])=Rp) or (CONRLT (UTCPD_CCSTS[4]) = 0).
     * |        |          |00 = SRC.Open (Open, Rp).
     * |        |          |01 = SRC.Ra (below maximum vRa).
     * |        |          |10 = SRC.Rd (within the vRd range).
     * |        |          |11 = reserved.
     * |        |          |If (CC1 (UTCPD_ROLCTL[1:0])=Rd) or (CONRLT (UTCPD_CCSTS[4]) = 1).
     * |        |          |00 = SNK.Open (Below maximum vRa).
     * |        |          |01 = SNK.Default (Above minimum vRd-Connect).
     * |        |          |10 = SNK.Power1.5 (Above minimum vRd-Connect) Detects Rp 1.5A.
     * |        |          |11 = SNK.Power3.0 (Above minimum vRd-Connect) Detects Rp 3.0A.
     * |        |          |If CC1 (UTCPD_ROLCTL[1:0])=Reserved, this field is set to 00.
     * |        |          |If CC1 (UTCPD_ROLCTL[1:0])=Open, this field is set to 00.
     * |        |          |This field always returns 00 if (LK4CONN=1) or (VCEN (UTCPD_PWRCTL[0])]=1 and ORIENT (UTCPD_CTL[0])=1)
     * |        |          |Otherwise, the returned value depends upon CC1 (UTCPD_ROLCTL[1:0])
     * |[3:2]   |CC2STATE  |CC2 State
     * |        |          |If (CC2 (UTCPD_ROLCTL[3:2]=Rp) or (CONRLT (UTCPD_CCSTS[4]) = 0).
     * |        |          |00 = SRC.Open (Open, Rp).
     * |        |          |01 = SRC.Ra (below maximum vRa).
     * |        |          |10 = SRC.Rd (within the vRd range).
     * |        |          |11 = reserved.
     * |        |          |If (CC2 (UTCPD_ROLCTL[3:2])=Rd) or (CONRLT (UTCPD_CCSTS[4]) = 1).
     * |        |          |00 = SNK.Open (Below maximum vRa).
     * |        |          |01 = SNK.Default (Above minimum vRd-Connect).
     * |        |          |10 = SNK.Power1.5 (Above minimum vRd-Connect) Detects Rp 1.5A.
     * |        |          |11 = SNK.Power3.0 (Above minimum vRd-Connect) Detects Rp 3.0A.
     * |        |          |If CC2 (UTCPD_ROLCTL[3:2])=Reserved, this field is set to 00.
     * |        |          |If CC2 (UTCPD_ROLCTL[3:2])=Open, this field is set to 00.
     * |        |          |This field always returns 00 if (LK4CONN=1) or (VCEN (UTCPD_PWRCTL[0])=1 and ORIENT (UTCPD_CTL[0])=0)
     * |        |          |Otherwise, the returned value depends upon CC2 (UTCPD_ROLCTL[3:2])
     * |[4]     |CONRLT    |Connect Result (Read Only)
     * |        |          |0 = the UTCPD is presenting Rp.
     * |        |          |1 = the UTCPD is presenting Rd.
     * |[5]     |LK4CONN   |Looking for Connection
     * |        |          |0 = UTCPD is not actively looking for a connection
     * |        |          |A transition from '1' to '0' indicates a potential connection has been found.
     * |        |          |1 = UTCPD is looking for a connection (toggling as a DRP or looking for a connection as Sink/Source only condition)
     * @var UTCPD_T::PWRSTS
     * Offset: 0x3C  UTCPD Power Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SKVB      |Sinking VBUS
     * |        |          |0 = Sink is Disconnected (Default).
     * |        |          |1 = UTCPD is sinking VBUS to the system load.
     * |[1]     |VCPS      |VCONN Present
     * |        |          |0 = VCONN is not present.
     * |        |          |1 = This bit is asserted when VCONN present CC1 or CC2. Threshold is fixed at 2.4V.
     * |        |          |If VCEN (UTCPD_PWRCTL[0])] is disabled VCONN Present should be set to 0.
     * |[2]     |VBPS      |VBUS Present
     * |        |          |0 = VBUS Disconnected.
     * |        |          |1 = VBUS Connected.
     * |        |          |The UTCPD shall report VBUS present when UTCPD detects VBUS rises above 4V.
     * |        |          |The UTCPD shall report VBUS is not present when UTCPD detects VBUS falls below 3.5V.
     * |[3]     |VBPSDTEN  |VBUS Present Detection Enabled
     * |        |          |0 = VBUS Present Detection Disabled.
     * |        |          |1 = VBUS Present Detection Enabled (default).
     * |[4]     |SRVB      |Sourcing VBUS
     * |        |          |0 = Sourcing VBUS is disabled.
     * |        |          |1 = Sourcing VBUS is enabled.
     * |        |          |This bit does not control the path, just provides a monitor of the status.
     * |[5]     |SRHV      |Sourcing High Voltage
     * |        |          |0 = VSAFE5V.
     * |        |          |1 = Higher voltage.
     * |        |          |This bit does not control the power path, it just provides a monitor of the status
     * |        |          |This bit is asserted as long as the UTCPD is sourcing nondefault voltage over VBUS (i.e
     * |        |          |not VSAFE5V) as a response to CPU writing 0x88 to CMD (Source VBUS High Voltage)
     * |[7]     |DACON     |Debug Accessory Connected
     * |        |          |0 = No Debug Accessory connected (default).
     * |        |          |1 = Debug Accessory connected.
     * @var UTCPD_T::FUTSTS
     * Offset: 0x40  UTCPD Fault Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |VCOCFUT   |VCONN Overcurrent Protection Fault
     * |        |          |0 = Not in an overcurrent protection state.
     * |        |          |1 = Overcurrent fault latched.
     * |[2]     |VBOVFUT   |VBUS Over Voltage Protection Fault
     * |        |          |0 = No Fault detected.
     * |        |          |1 = Over-voltage fault latched.
     * |[3]     |VBOCFUT   |VBUS Overcurrent Protection Fault
     * |        |          |0 = Not in an overcurrent protection state.
     * |        |          |1 = Overcurrent fault latched.
     * |[4]     |FDGFAL    |Force Discharge Failed
     * |        |          |0 = No discharge failure.
     * |        |          |1 = Discharge commanded by the TCPM failed.
     * |        |          |If FDGEN (UTCPD_PWRCTL[2]) is set, the UTCPD shall report a discharge fails if VBUS is not below vSafe0V within tSafe0V
     * |[5]     |ADGFAL    |Auto Discharge Failed
     * |        |          |0 = No discharge failure.
     * |        |          |1 = Discharge commanded by the TCPM failed.
     * |        |          |If ADGDC (UTCPD_PWRCTL[4]) is set, the UTCPD shall report discharge fails if VBUS is not below vSafe0V within tSafe0V
     * |[6]     |FOFFVB    |Force Off VBUS
     * |        |          |0 = No Fault Detected, no action (default and not supported).
     * |        |          |1 = VBUS Source/Sink has been forced off due to external fault.
     * |        |          |The UTCPD has disconnected VBUS due to external inputs (EINT0 ~ EINT5)
     * |[7]     |INVHRST   |Invalid Hard Reset
     * |        |          |0 = No invalid hard reset event.
     * |        |          |1 = Invalid hard reset event active.
     * @var UTCPD_T::CMD
     * Offset: 0x44  UTCPD Command Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CMD       |Command Set
     * |        |          |0x22 = Disable VBUS Detect.
     * |        |          |0x33 = Enable VBUS Detect.
     * |        |          |0x44 = Disable Sink VBUS.
     * |        |          |0x55h = Enable Sink VBUS.
     * |        |          |0x66 = Disable Source VBUS.
     * |        |          |0x77 = Enable Source VBUS 5V.
     * |        |          |0x88 = Source VBUS High Voltage.
     * |        |          |0x99h = Look4Connection.
     * |        |          |0xAA = RxOneMore.
     * |        |          |Others: Reserved.
     * |        |          |The Command is issued by the CPU
     * |        |          |The Command is cleared by the UTCPD after being acted upon
     * |        |          |It always read as 0.
     * @var UTCPD_T::DVCAP1
     * Offset: 0x48  UTCPD Device Capabilities 1 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CPSRVB    |Source VBUS
     * |        |          |0 = TCPC is not capable of controlling the source path to VBUS.
     * |        |          |1 = TCPC is capable of controlling the source path to VBUS.
     * |[1]     |CPSRHV    |Source High Voltage VBUS
     * |        |          |0 = UTCPD is not capable of controlling the source high voltage path to VBUS.
     * |        |          |1 = UTCPD is capable of controlling the source high voltage path to VBUS.
     * |[2]     |CPSKVB    |Sink VBUS
     * |        |          |0 = UTCPD is not capable controlling the sink path to the system load.
     * |        |          |1 = UTCPD is capable of controlling the sink path to the system load.
     * |[3]     |CPSRVC    |Source VCONN
     * |        |          |0 = UTCPD is not capable of switching VCONN.
     * |        |          |1 = UTCPD is capable of switching VCONN.
     * |[4]     |CPSDBG    |SOP'_DBG/SOPu201D_DBG Support
     * |        |          |0 = All SOP* except SOP'_DBG/SOP'_DBG.
     * |        |          |1 = All SOP* messages are supported.
     * |[7:5]   |CPROL     |Roles Supported
     * |        |          |000 = USB Type-C Port Manager can configure the Port as Source only or Sink only (not DRP).
     * |        |          |001 = Source only.
     * |        |          |010 = Sink only.
     * |        |          |011 = Sink with accessory support.
     * |        |          |100 = DRP only.
     * |        |          |101 = Source, Sink, DRP, Adapter/Cable all supported.
     * |        |          |110 = Source, Sink, DRP.
     * |        |          |111 = Not valid.
     * |[9:8]   |CPSRRE    |Source Resistor Supported
     * |        |          |00 = Rp default only.
     * |        |          |01 = Rp 1.5A and default.
     * |        |          |10 = Rp 3.0A, 1.5A, and default.
     * |        |          |11 = Reserved.
     * |[10]    |CPVBAM    |VBUS Measurement and Alarm Capable
     * |        |          |0 = No VBUS voltage measurement nor VBUS Alarms.
     * |        |          |1 = VBUS voltage measurement and VBUS Alarms.
     * |[11]    |CPFDG     |Force Discharge
     * |        |          |0 = No Force Discharge implemented.
     * |        |          |1 = Force Discharge is implemented.
     * |[12]    |CPBDG     |Bleed Discharge
     * |        |          |0 = No Bleed Discharge implemented.
     * |        |          |1 = Bleed Discharge is implemented.
     * |[13]    |CPVBOVP   |VBUS OVP Reporting
     * |        |          |0 = VBUS OVP is not reported.
     * |        |          |1 = VBUS OVP is reported.
     * |[14]    |CPVBOCP   |VBUS OCP Reporting
     * |        |          |0 = VBUS OCP is not reported.
     * |        |          |1 = VBUS OCP is reported.
     * @var UTCPD_T::DVCAP2
     * Offset: 0x4C  UTCPD Device Capabilities 2 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CPVCOC    |VCONN Overcurrent Fault Capable
     * |        |          |0 = UTCPD is not capable of detecting a Vconn fault.
     * |        |          |1 = UTCPD is capable of detecting a Vconn fault.
     * |[3:1]   |CPVCPWR   |VCONN Power Supported
     * |        |          |000 = 1.0W.
     * |        |          |001 = 1.5W.
     * |        |          |010 = 2.0W.
     * |        |          |011 = 3W.
     * |        |          |100 = 4W.
     * |        |          |101 = 5W.
     * |        |          |110 = 6W.
     * |        |          |111 = External.
     * |[5:4]   |CPVBAMLS  |VBUS Voltage Alarm LSB
     * |        |          |00 = UTCPD has 25mV LSB for its voltage alarm and uses all 10 bits in.
     * |        |          |VB_AMH and VB_AML.
     * |        |          |Others = Reserved.
     * |[6]     |CPSPDGTH  |Stop Discharge Threshold
     * |        |          |0 = UTCPD_SPDGTH not implemented.
     * |        |          |1 = UTCPD_SPDGTH implemented.
     * |[7]     |CPSKDCDT  |Sink Disconnect Detection
     * |        |          |0 = UTCPD_SKVBDCTH not implemented.
     * |        |          |1 = UTCPD_SKVBDCTH implemented.
     * @var UTCPD_T::MSHEAD
     * Offset: 0x50  UTCPD Message Header Info Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWRROL    |Power Role
     * |        |          |0 = Sink.
     * |        |          |1 = Source.
     * |[2:1]   |PDREV     |USB PD Specification Revision
     * |        |          |00 = Revision 1.0.
     * |        |          |01 = Revision 2.0.
     * |        |          |10 = Revision 3.0.
     * |        |          |11 = Revision 3.1.
     * |[3]     |DAROL     |Data Role
     * |        |          |0 = UFP.
     * |        |          |1 = DFP.
     * |[4]     |CABPLG    |Cable Plug
     * |        |          |0 = Message originated from Source, Sink, or DRP.
     * |        |          |1 = Message originated from a Cable Plug.
     * @var UTCPD_T::DTRXEVNT
     * Offset: 0x54  UTCPD Enable Detect RX Event Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SOPEN     |Enable SOP Message
     * |        |          |0 = UTCPD does not detect SOP message (default).
     * |        |          |1 = UTCPD detects SOP message.
     * |[1]     |SOPPEN    |Enable SOP' Message
     * |        |          |0 = UTCPD does not detect SOP' message (default).
     * |        |          |1 = UTCPD detects SOP' message.
     * |[2]     |SOPPPEN   |Enable SOP'' Message
     * |        |          |0 = UTCPD does not detect SOP'' message (default).
     * |        |          |1 = UTCPD detects SOP'' message.
     * |[3]     |SDBGPEN   |Enable SOP_DBG' Message
     * |        |          |0 = UTCPD does not detect SOP_DBG' message (default).
     * |        |          |1 = UTCPD detects SOP_DBG' message.
     * |[4]     |SDBGPPEN  |Enable SOP_DBG'' Message
     * |        |          |0 = UTCPD does not detect SOP_DBG'' message (default).
     * |        |          |1 = UTCPD detects SOP_DBG'' message.
     * |[5]     |HRSTEN    |Enable Hard Reset
     * |        |          |0 = UTCPD does not detect Hard Reset signaling (default).
     * |        |          |1 = UTCPD detects Hard Reset signaling.
     * |[6]     |CABRSTEN  |Enable Cable Reset
     * |        |          |0 = UTCPD does not detect Cable Reset signaling (default).
     * |        |          |1 = UTCPD detects Cable Reset signaling.
     * @var UTCPD_T::RXBCNT
     * Offset: 0x58  UTCPD RX Byte Count Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |RXBCNT    |Receive Byte Count
     * |        |          |Indicates number of bytes in this register that are not stale
     * |        |          |CPU should read the first RXBCNT bytes in this register.
     * |        |          |This is the number of bytes in the UTCPD_RXDAx plus three (for the RXFTYPE and RXHEAD).
     * |        |          |The UTCPD shall clear the UTCPD_DTRXEVNT and the UTCPD_RXBCNT register to disable the PD message passing when CPU writes the UTCPD_TXCTL register requesting a Hard Reset.
     * @var UTCPD_T::RXFTYPE
     * Offset: 0x5C  UTCPD Received Frame Type Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |RXFTYPE   |Received Frame Type
     * |        |          |000 = Received SOP.
     * |        |          |001 = Received SOP'.
     * |        |          |010 = Received SOP''.
     * |        |          |011 = Received SOP_DBG'.
     * |        |          |100 = Received SOP_DBG''.
     * |        |          |110 = Received Cable Reset.
     * |        |          |All others are reserved.
     * @var UTCPD_T::RXHEAD
     * Offset: 0x60  UTCPD Received Header Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |RXHEAD0   |USB PD message header byte 0
     * |[15:8]  |RXHEAD1   |USB PD message header byte 1
     * @var UTCPD_T::RXDA0
     * Offset: 0x70  UTCPD Received Data0 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |RXDAB0    |USB PD Received Datax byte 0
     * |[15:8]  |RXDAB1    |USB PD Received Datax byte 1
     * |[23:16] |RXDAB2    |USB PD Received Datax byte 2
     * |[31:24] |RXDAB3    |USB PD Received Datax byte 3
     * @var UTCPD_T::RXDA1
     * Offset: 0x74  UTCPD Received Data1 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |RXDAB0    |USB PD Received Datax byte 0
     * |[15:8]  |RXDAB1    |USB PD Received Datax byte 1
     * |[23:16] |RXDAB2    |USB PD Received Datax byte 2
     * |[31:24] |RXDAB3    |USB PD Received Datax byte 3
     * @var UTCPD_T::RXDA2
     * Offset: 0x78  UTCPD Received Data2 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |RXDAB0    |USB PD Received Datax byte 0
     * |[15:8]  |RXDAB1    |USB PD Received Datax byte 1
     * |[23:16] |RXDAB2    |USB PD Received Datax byte 2
     * |[31:24] |RXDAB3    |USB PD Received Datax byte 3
     * @var UTCPD_T::RXDA3
     * Offset: 0x7C  UTCPD Received Data3 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |RXDAB0    |USB PD Received Datax byte 0
     * |[15:8]  |RXDAB1    |USB PD Received Datax byte 1
     * |[23:16] |RXDAB2    |USB PD Received Datax byte 2
     * |[31:24] |RXDAB3    |USB PD Received Datax byte 3
     * @var UTCPD_T::RXDA4
     * Offset: 0x80  UTCPD Received Data4 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |RXDAB0    |USB PD Received Datax byte 0
     * |[15:8]  |RXDAB1    |USB PD Received Datax byte 1
     * |[23:16] |RXDAB2    |USB PD Received Datax byte 2
     * |[31:24] |RXDAB3    |USB PD Received Datax byte 3
     * @var UTCPD_T::RXDA5
     * Offset: 0x84  UTCPD Received Data5 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |RXDAB0    |USB PD Received Datax byte 0
     * |[15:8]  |RXDAB1    |USB PD Received Datax byte 1
     * |[23:16] |RXDAB2    |USB PD Received Datax byte 2
     * |[31:24] |RXDAB3    |USB PD Received Datax byte 3
     * @var UTCPD_T::RXDA6
     * Offset: 0x88  UTCPD Received Data6 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |RXDAB0    |USB PD Received Datax byte 0
     * |[15:8]  |RXDAB1    |USB PD Received Datax byte 1
     * |[23:16] |RXDAB2    |USB PD Received Datax byte 2
     * |[31:24] |RXDAB3    |USB PD Received Datax byte 3
     * @var UTCPD_T::TXCTL
     * Offset: 0x90  UTCPD TX Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |TXSTYPE   |Transmit SOP* Message
     * |        |          |000 = Transmit SOP.
     * |        |          |001 = Transmit SOP'.
     * |        |          |010 = Transmit SOP''.
     * |        |          |011 = Transmit SOP_DBG'.
     * |        |          |100 = Transmit SOP_DBG''.
     * |        |          |101 = Transmit Hard Reset.
     * |        |          |110 = Transmit Cable Reset.
     * |        |          |111 = Transmit BIST Carrier Mode 2.
     * |        |          |The UTCPD shall ignore the Retry Counter bits when transmitting a Hard Reset, Cable Reset, or BIST Carrier
     * |[5:4]   |RETRYCNT  |Retry Counter
     * |        |          |00 = No message retry is required.
     * |        |          |01 = Automatically retry message transmission once.
     * |        |          |10 = Automatically retry message transmission twice.
     * |        |          |11 = Automatically retry message transmission three times.
     * @var UTCPD_T::TXBCNT
     * Offset: 0x94  UTCPD TX Byte Count Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |TXBCNT    |Transmit Byte Count
     * |        |          |This is the number of bytes in the UTCPD_TXDA plus two (for the TXHEAD)
     * |        |          |If a previous transmit request has not yet completed when TX_CTL is written requesting a Hard Reset, the UTCPD shall assert the Transmission Discarded bit(TXDCIS (UTCPD_IS[5])).
     * |        |          |The UTCPD shall assert both TXOKIS (UTCPD_IS[6]) and TXFALIS (UTCPD_IS[4]) after it completes the sending of a Hard Reset
     * @var UTCPD_T::TXHEAD
     * Offset: 0x98  UTCPD TX Header Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |TXHEAD0   |Transmit Header Byte 0
     * |[15:8]  |TXHEAD1   |Transmit Header Byte 1
     * @var UTCPD_T::TXDA0
     * Offset: 0xA0  UTCPD Transmit Data0 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |TXDAB0    |USB PD Transmit Datax byte 0
     * |[15:8]  |TXDAB1    |USB PD Transmit Datax byte 1
     * |[23:16] |TXDAB2    |USB PD Transmit Datax byte 2
     * |[31:24] |TXDAB3    |USB PD Transmit Datax byte 3
     * @var UTCPD_T::TXDA1
     * Offset: 0xA4  UTCPD Transmit Data1 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |TXDAB0    |USB PD Transmit Datax byte 0
     * |[15:8]  |TXDAB1    |USB PD Transmit Datax byte 1
     * |[23:16] |TXDAB2    |USB PD Transmit Datax byte 2
     * |[31:24] |TXDAB3    |USB PD Transmit Datax byte 3
     * @var UTCPD_T::TXDA2
     * Offset: 0xA8  UTCPD Transmit Data2 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |TXDAB0    |USB PD Transmit Datax byte 0
     * |[15:8]  |TXDAB1    |USB PD Transmit Datax byte 1
     * |[23:16] |TXDAB2    |USB PD Transmit Datax byte 2
     * |[31:24] |TXDAB3    |USB PD Transmit Datax byte 3
     * @var UTCPD_T::TXDA3
     * Offset: 0xAC  UTCPD Transmit Data3 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |TXDAB0    |USB PD Transmit Datax byte 0
     * |[15:8]  |TXDAB1    |USB PD Transmit Datax byte 1
     * |[23:16] |TXDAB2    |USB PD Transmit Datax byte 2
     * |[31:24] |TXDAB3    |USB PD Transmit Datax byte 3
     * @var UTCPD_T::TXDA4
     * Offset: 0xB0  UTCPD Transmit Data4 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |TXDAB0    |USB PD Transmit Datax byte 0
     * |[15:8]  |TXDAB1    |USB PD Transmit Datax byte 1
     * |[23:16] |TXDAB2    |USB PD Transmit Datax byte 2
     * |[31:24] |TXDAB3    |USB PD Transmit Datax byte 3
     * @var UTCPD_T::TXDA5
     * Offset: 0xB4  UTCPD Transmit Data5 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |TXDAB0    |USB PD Transmit Datax byte 0
     * |[15:8]  |TXDAB1    |USB PD Transmit Datax byte 1
     * |[23:16] |TXDAB2    |USB PD Transmit Datax byte 2
     * |[31:24] |TXDAB3    |USB PD Transmit Datax byte 3
     * @var UTCPD_T::TXDA6
     * Offset: 0xB8  UTCPD Transmit Data6 Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |TXDAB0    |USB PD Transmit Datax byte 0
     * |[15:8]  |TXDAB1    |USB PD Transmit Datax byte 1
     * |[23:16] |TXDAB2    |USB PD Transmit Datax byte 2
     * |[31:24] |TXDAB3    |USB PD Transmit Datax byte 3
     * @var UTCPD_T::VBVOL
     * Offset: 0xC0  UTCPD VBUS Voltage Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |VBVOL     |VBUS Voltage Measurement
     * |        |          |10-bit measurement of (VBUS / Scale Factor). It is read only.
     * |        |          |CPU multiplies this value by the scale factor to obtain the voltage measurement
     * |        |          |Voltages greater than or equal to 4V shall meet +/-2% absolute value or +/- 50mV, whichever is greater
     * |        |          |The LSB is 25mV
     * |[11:10] |VBSCALE   |VBUS Scale Factor
     * |        |          |00 = External VBUS voltage divider circuit should be 1/20 for EPR application
     * |        |          |The divided voltage compares with 200mV to set or clean VBUS Present bit.
     * |        |          |01 = External VBUS voltage divider circuit should be 1/10 for SPR application
     * |        |          |The divided voltage compares with 400mV to set or clean VBUS Present bit.
     * |        |          |10 = Same as 00.
     * |        |          |11 = Same as 01.
     * @var UTCPD_T::SKVBDCTH
     * Offset: 0xC4  UTCPD VBUS Sink disconnect threshold Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |SKVBDCTH  |Sink VBUS Disconnect Threshold
     * |        |          |10-bit for voltage threshold with 25mV LSB. (Default 3.5V)
     * |        |          |A value of zero disables this threshold.
     * @var UTCPD_T::SPDGTH
     * Offset: 0xC8  UTCPD VBUS Stop Discharge threshold Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |SPDGTH    |VBUS Stop Force Discharge Threshold
     * |        |          |This value is used as a threshold hold for force discharge fail.
     * |        |          |The default is 0.8V.
     * |        |          |The CPU writes to this register to set the threshold at which a Source shall stop the Forced Discharge circuit when FDGEN (UTCPD_PWRCTL[2]) is 1.
     * @var UTCPD_T::VBAMH
     * Offset: 0xCC  UTCPD VBUS voltage high alarm threshold Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |VBAMH     |VBUS voltage high Alarm threshold register
     * @var UTCPD_T::VBAML
     * Offset: 0xD0  UTCPD VBUS voltage low alarm threshold Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |VBAML     |VBUS voltage low Alarm threshold register
     * @var UTCPD_T::VNDIS
     * Offset: 0xD4  UTCPD Vendor defined Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXFRSIS   |Fast Role Swap RX Interrupt Status
     * |        |          |0 = Cleared.
     * |        |          |1 = The event has been detected.
     * |[1]     |TXFRSIS   |Fast Role Swap TX Interrupt Status
     * |        |          |0 = Cleared.
     * |        |          |1 = The event has been detected.
     * |[3]     |CRCERRIS  |CRC Error Interrupt Status
     * |        |          |0 = Cleared.
     * |        |          |1 = The event has been detected.
     * |[5]     |VCDGIS    |VCONN Discharge Interrupt Status
     * @var UTCPD_T::VNDIE
     * Offset: 0xD8  UTCPD Vendor defined Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXFRSIE   |Fast Role Swap RX Interrupt Enable
     * |        |          |0 = Fast role swap RX interrupt Disabled.
     * |        |          |1 = Fast role swap RX interrupt Enabled.
     * |[1]     |TXFRSIE   |Fast Role Swap TX Interrupt Enable
     * |        |          |0 = Fast role swap TX interrupt Disabled.
     * |        |          |1 = Fast role swap TX interrupt Enabled.
     * |[3]     |CRCERRIE  |CRC Error Interrupt Enable
     * |        |          |0 = CRC error interrupt Disabled.
     * |        |          |1 = CRC error interrupt Enabled.
     * |[5]     |VCDGIE    |VCONN Discharge Interrupt Enable
     * |        |          |0 = VCOON discharge interrupt Disabled.
     * |        |          |1 = VCOON discharge interrupt Enabled.
     * @var UTCPD_T::MUXSEL
     * Offset: 0xDC  UTCPD Mux Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |VBOCS     |VBUS Overcurrent Source Select
     * |        |          |0000 = EINT0.
     * |        |          |0001 = EINT1.
     * |        |          |0010 = EINT2.
     * |        |          |0011 = EINT3.
     * |        |          |0100 = ADCMPF0.
     * |        |          |0101 = ACMP0.
     * |        |          |0110 = ACMP1.
     * |        |          |0111 = ACMP2.
     * |        |          |Others = Reserved.
     * |        |          |Note: For ACMP, the VBOCPL (PINPL[5]) shall be set as 1.
     * |[7:4]   |VCOCS     |VCONN Overcurrent Source Select
     * |        |          |0000 = EINT0.
     * |        |          |0001 = EINT1.
     * |        |          |0010 = EINT2.
     * |        |          |0011 = EINT3.
     * |        |          |0100 = ADCMPF1.
     * |        |          |0101 = ACMP0.
     * |        |          |0110 = ACMP1.
     * |        |          |0111 = ACMP2.
     * |        |          |Others = Reserved.
     * |        |          |Note: For ACMP, the VCOCPL (PINPL[10]) shall be set as 1.
     * |[10:8]  |FVBS      |Force off VBUS Select
     * |        |          |000 = ENIT0.
     * |        |          |001 = EINT1.
     * |        |          |010 = EINT2.
     * |        |          |011 = ENIT3.
     * |        |          |100 = EINT4.
     * |        |          |101 = EINT5.
     * |        |          |Others = Reserved.
     * |[16:12] |ADCSELVB  |ADC Channel Select for VBUS
     * |        |          |ADC result will be latched into VBVOL register once ADCCSELVB matches ADC_CHSEL and ADC finishes.
     * |[21:17] |ADCSELVC  |ADC Channel Select for VCONN
     * |        |          |ADC result will be latched into VCVOL register once ADCCSELVC matches ADC_CHSEL and ADC finishes.
     * |[24]    |CC1VCENS  |CC1 VCONN Enable Select
     * |        |          |0 = Merged CC1 and CC2 VCONN enable signal.
     * |        |          |1 = CC1 VCONN enable signal.
     * |[25]    |CC1FRSS   |CC1 VCONN Fast Role Swap Select
     * |        |          |0 = Merged CC1 and CC2 VCONN Fast Role Swap signal.
     * |        |          |1 = CC1 VCONN Fast Role Swap signal.
     * |[28]    |CC2VCENS  |CC2 VCONN Enable Select
     * |        |          |0 = Merged CC1 and CC2 VCONN enable signal.
     * |        |          |1 = CC2 VCONN enable signal.
     * |[29]    |CC2FRSS   |CC2 VCONN Fast Role Swap Select
     * |        |          |0 = Merged CC1 and CC2 Fast Role Swap signal.
     * |        |          |1 = CC2 VCONN discharge signal.
     * @var UTCPD_T::VCDGCTL
     * Offset: 0xE0  UTCPD VCONN Discharge Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |VCDGDTEN  |VCONN Discharge Detect Enable
     * |        |          |0 = VCONN Discharge Detect Disabled.
     * |        |          |1 = VCONN Discharge Detect Enabled.
     * |[1]     |VCDGEN    |VCONN Discharge Enable
     * |        |          |0 = VCONN Discharge Disabled.
     * |        |          |1 = VCONN Discharge Enabled.
     * @var UTCPD_T::PHYSLEW
     * Offset: 0xE4  UTCPD PHY Slew Rate Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |TXRTRIM   |TX Trim Rising Slew Rate
     * |        |          |It is for BMC eye diagram.
     * |        |          |Trim PHY TX Rising slew rate (from ROMMAP)
     * |[6:4]   |TXFTRIM   |TX Trim Falling Slew Rate
     * |        |          |It is for BMC eye diagram.
     * |        |          |Trim PHY TX falling slew rate. (from ROMMAP)
     * @var UTCPD_T::ADGTM
     * Offset: 0xE8  UTCPD Auto Discharge Time Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |ADGTM     |Auto Discharge Time
     * |        |          |Default Time = 31.25us x 16 x 100 (0x16) = 49.9ms.
     * @var UTCPD_T::VSAFE0V
     * Offset: 0xEC  UTCPD Auto Discharge VSAFE0V Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |VSAFE0V   |Set the vSafe0V voltage level.
     * @var UTCPD_T::VSAFE5V
     * Offset: 0xF0  UTCPD VSAFE5V Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |VSAFE5V   |Set the vSafe5V Voltage Level
     * |        |          |For fast role swap voltage comparison.
     * @var UTCPD_T::RATIO
     * Offset: 0xF4  UTCPD DRP Toggle Ratio Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |DRPRATIO  |The percent of time that a DRP shall advertise source & sink during tDRP
     * |        |          |000 = 50:50 (Sink: Source).
     * |        |          |010 = 30:70 (Sink: Source).
     * |        |          |001 = 40:60 (Sink: Source).
     * |        |          |101 = 60:40 (Sink: Source).
     * |        |          |110 =70:30 (Sink: Source).
     * |        |          |111 = Reserved.
     * |[3]     |VBSEL     |VBUS_VOLTAGE_SEL
     * |        |          |Select comparing value (vbus_voltage_s) during vbus discharge determines to stop discharging or not.
     * |        |          |0 = VBUS voltage value from adc only when VBMONI (UTCPD_PWRCT[6]) is 1.
     * |        |          |1 = VBUS voltage value from ADC.
     * |[6]     |ADCAVG    |ADC Moving Average Enable
     * |        |          |If enabling this bit, average the ADC value for 4 times.
     * @var UTCPD_T::INTFRAME
     * Offset: 0xF8  UTCPD Inter-Frame Time Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |INTFRAME  |USB PD Inter Frame Gap
     * |        |          |Each unit time: 83ns (12 MHz RC)
     * |        |          |Time = 83ns x 16 x Inter frame gap time.
     * |        |          |Example = 83ns x 16 x 25(8'b00011001) = 33.2us.
     * @var UTCPD_T::VBOVTH
     * Offset: 0xFC  UTCPD VBUS Over Voltage Threshold Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |VBOVTH    |VBUS Over Voltage Threshold
     * |        |          |This value defines the VBUS over voltage threshold
     * |        |          |10-bit for voltage threshold with 25mV LSB.
     * @var UTCPD_T::VNDINIT
     * Offset: 0x100  UTCPD Vendor Initial Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[6:4]   |DVCAPDEF  |Device Capability Default Setting
     * |        |          |Write the Register will update the value to Device_Cap.RoleSupport, ROLE_CONTROL and MESSAGE_HEADER_INFO register, please refer the following table
     * |        |          |(DevCap_DEF Table)
     * @var UTCPD_T::BMCTXBP
     * Offset: 0x104  UTCPD BMC TX Bit Period Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |BMCTXBP   |BMC Tx Bit Period
     * |        |          |It's for BMC eye diagram.
     * |        |          |Example 12 MHz = 83.33ns.
     * |        |          |83.33ns x (39 + 1) = 3.33us.
     * @var UTCPD_T::BMCTXDU
     * Offset: 0x108  UTCPD BMC TX Duty Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[6:0]   |DUOFFS2   |BMC Tx Duty Offset Parameter 2
     * |        |          |It's for BMC eye diagram.
     * |        |          |Offset count value.
     * |        |          |Example 12 MHz = 83.33ns.
     * |[7]     |DUOFFS1   |BMC Tx Duty Offset Parameter 1
     * |        |          |It's for BMC eye diagram.
     * |        |          |0 = Increase duty offset.
     * |        |          |1 = Decrease duty offset.
     * @var UTCPD_T::VCPSVOL
     * Offset: 0x10C  VCONN Present Voltage Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |VCPSVOL   |VCONN Present voltage
     * |        |          |Detect Voltage = 60h * 0.025V = 2.42V.
     * @var UTCPD_T::VCUV
     * Offset: 0x110  VCONN Under Voltage Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |VCUV      |VCONN under voltage comparator
     * |        |          |Detect Voltage = 20h * 0.025V = 0.8V.
     * @var UTCPD_T::BMCSLICE
     * Offset: 0x118  UTCPD BMC SLICE Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |SLICEL    |TX Slice Low Level Control
     * |        |          |Low level slice control (The LSB is 2mV.)
     * |        |          |00 = 0.18V.
     * |        |          |01 = 0.2V.
     * |        |          |10 = 0.22V.
     * |        |          |11 = 0.24V (Default).
     * |        |          |Others = Reserved.
     * |[3:2]   |SLICEH    |TX Slice High Level Control
     * |        |          |High level slice control (The LSB is 2mV.)
     * |        |          |00 = 0.84V (Default).
     * |        |          |01 = 0.86V.
     * |        |          |10 = 0.88V.
     * |        |          |11 = 0.9V.
     * |        |          |Others = Reserved.
     * |[6:4]   |SLICEM    |TX Slice Middle Level Control
     * |        |          |Middle level slice control (The LSB is 2mV.)
     * |        |          |000 = 0.48V.
     * |        |          |100 = 0.56V (Default).
     * |        |          |111 = 0.62V.
     * |        |          |Others = Reserved.
     * |[10:8]  |TRIMRD    |TRIMRD
     * |[15:12] |TRIMRP    |TRIMRP
     * |[18:16] |TRIMV1P1  |TRIMV1P1
     * |[22:20] |TRIMVBUS10|TRIMVBUS10
     * |        |          |Power good level trimming option of VBUS when the VBSCALE[1] in UTCPD_VBVOL[10] = 0.
     * |[26:24] |TRIMVBUS20|TRIMVBUS20
     * |        |          |Power good level trimming option of VBUS when the VBSCALE[1] in UTCPD_VBVOL[10] = 1.
     * |[31:28] |VTRIM     |Reference Voltage TRIM setting for TCPD PHY macro
     * @var UTCPD_T::PHYCTL
     * Offset: 0x11C  UTCPD PHY Power Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PHYPWR    |Analog PHY Power
     * |        |          |0 = Power down PHY.
     * |        |          |1 = Enable PHY.
     * |        |          |Analog PHY power default is off before UTCPD clock available
     * |        |          |Once UTCPD clock is on, the analog PHY power will be on as well.
     * |[1]     |DBCTL     |Dead Battery Control
     * |        |          |0 = Dead Battery circuit control internal Rd/Rp.
     * |        |          |1 = Role Control Register control internal Rd/Rp.
     * @var UTCPD_T::FRSRXCTL
     * Offset: 0x120  UTCPD CC Fast Swap RX Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FRSTX     |CC Transmitter Fast Swap Signal
     * |        |          |Pulse width is 85us.
     * |[2]     |FRSDVVB   |CC receive fast swap and auto drive Source VBUS
     * |        |          |0 = CC receive fast swap and auto drive Source VBUS Disabled.
     * |        |          |1 = CC receive fast swap and auto drive Source VBUS Enabled.
     * |[3]     |FRSRXEN   |CC receive fast swap Rx Enable
     * |        |          |0 = CC receive fast swap Rx Disabled.
     * |        |          |1 = CC receive fast swap Rx Enabled.
     * @var UTCPD_T::VCVOL
     * Offset: 0x124  UTCPD VCONN Voltage Measurement Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |VCVOL     |VCONN Voltage Measurement
     * |        |          |The LSB is 25mV.
     * @var UTCPD_T::CLKINFO
     * Offset: 0x300  UTCPD Clock Information Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ReadyFlag |RC32K Domain Ready Flag Check if the Register Value is Loaded to RC32K Domain by Reading This Flag
     * |        |          |0 = The RC32K signal is not ready.
     * |        |          |1 = The RC32K signal is ready.
     * |[4]     |WKEN      |Wakeup Enable
     * |        |          |0 = UTCPD wakeup function Disabled.
     * |        |          |1 = UTCPD wakeup function Enabled.
     * @var UTCPD_T::PREDET
     * Offset: 0x320  UTCPD Preamble Detect Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |RNICHK    |Round of Noise Immunity Check
     * |        |          |0000 = Noise immunity optimize Disabled.
     * |        |          |0001 = The detect is one round.
     * |        |          |0010 = The detect is two rounds.
     * |        |          |0011 = The detect is three rounds.
     * |        |          |0100 = The detect is four rounds.
     * |        |          |0101 = The detect is five rounds.
     * |        |          |0110 = The detect is six rounds.
     * |        |          |0111 = The detect is seven rounds.
     * |        |          |1000 = The detect is eight round.
     * |        |          |1001 = The detect is nini rounds.
     * |        |          |1010 = The detect is ten rounds.
     * |        |          |1011 = The detect is eleven rounds.
     * |        |          |1100 = The detect is twelve rounds.
     * |        |          |1101 = The detect is thirteen rounds.
     * |        |          |1110 = The detect is fourteen rounds.
     * |        |          |1111 = The detect is fifteen rounds.
     * |        |          |Note: RPCHK = 3'b000, Use the original detection solution. (case path 0).
     * |[6:4]   |BNICHK    |Bit Number for Noise Immunity Check each Round
     * |        |          |000 = The detected bit number of one round equal to 3 bits.
     * |        |          |001 = The detected bit number of one round equal to 4 bits.
     * |        |          |010 = The detected bit number of one round equal to 5 bits.
     * |        |          |011 = The detected bit number of one round equal to 6 bits.
     * |        |          |100 = The detected bit number of one round equal to 7 bits.
     * |        |          |101 = The detected bit number of one round equal to 8 bits.
     * |        |          |Others = Reserved.
     * |[9:8]   |BNW       |Bit Number Wait for next Round
     * |        |          |00 = The wait bit number for next round is 1 bits.
     * |        |          |01 = The wait bit number for next round is 2 bits.
     * |        |          |10 = The wait bit number for next round is 3 bits.
     * |        |          |11 = The wait bit number for next round is 4 bits.
     * |[12]    |DGEN      |Analog Macro Deglitch circuit Enable
     * |        |          |0 = Analog macro deglitch circuit Disabled.
     * |        |          |1 = Analog macro 3 slices comparator deglitch circuit Enabled.
     * |[13]    |DGACT     |Analog Macro Activity Signal Source from Deglitch Circuit
     * |        |          |0 = Analog macro activity signal source from deglitch circuit Disabled.
     * |        |          |1 = Analog macro activity signal source from deglitch circuit Enabled.
     * |[15:14] |PSDB      |Preamble Start Detect Bit
     * |        |          |00 = Round start after first 3 bits be detected.
     * |        |          |01 = Round start after first 5 bits be detected.
     * |        |          |10 = Round start after first 6 bits be detected.
     * |        |          |11 = Round start after first 7 bits be detected.
     * |[21:16] |MINWBCN   |Minimum Width Bit Counter Number
     * |        |          |Define the minuimun width of one bit counter number among preamble period
     * |        |          |If the counter of one bit width is less than this value, this case path will be ignore to juge the bit counter number.
     * |        |          |Note: The default value is the bit clock counter of maxmun bit rate frequency (330K) plus 10%.
     * |[29:24] |MAXWBCN   |Maximun Width Bit Counter Number
     * |        |          |Define the maximun width of one bit counter number among preamble period
     * |        |          |If the counter of one bit width is over this value, this case path will be ignore to juge the bit counter number.
     * |        |          |Note: The default value is the bit clock counter of minimun bit rate frequency (270K) minus 10%.
     * @var UTCPD_T::NIRR
     * Offset: 0x324  UTCPD Noise Immunity Record Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |WBW       |Wide Bit Width (Read Only)
     * |        |          |The field indicates the wide bit width (based on 12M).
     * |[13:8]  |NBW       |Narrow Bit Width (Read Only)
     * |        |          |The field indicates the narrow bit width (based on 12M).
     * |[15:14] |CSEL      |Case Selection (Read Only)
     * |        |          |The field indicates the selected case condition.
     * |        |          |00 = The case 0.
     * |        |          |01 = The case 1.
     * |        |          |10 = The case 2.
     * |        |          |11 = Reserved.
     * @var UTCPD_T::C0PRR
     * Offset: 0x328  UTCPD Case 0 Path Record Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |C0PWBPN   |Case 0 Path Wide Bit Period Number (Read Only)
     * |        |          |The field indicates the final width bit clock number based 12M clock.
     * |        |          |Note: It is cleared in next preamble start or write 1.
     * |[13:8]  |C0PNBPN   |Case 0 Path Narrow Bit Period Number (Read Only)
     * |        |          |The field indicates the final narrow bit clcok number based 12M
     * |        |          |If the RNICHK (UTCPD_PREDET[2:0]) is 3'b000, the default value is 0x3F.
     * |        |          |Note: It is cleared in next preamble start or write 1.
     * |[15]    |C0PORF    |Case 0 Path Out of Range Flag
     * |        |          |0 = Case 0 isn't out of range.
     * |        |          |1 = Case 0 is out of range.
     * |        |          |Note: It is cleared in next preamble start or write 1.
     * |[18:16] |C0BRT     |Case 0 Path Best Round Times
     * |        |          |The field indicates the best round times of bit width detection.
     * |        |          |Note: It is cleared in next preamble start or write 1.
     * @var UTCPD_T::C1PRR
     * Offset: 0x32C  UTCPD Case 1 Path Record Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |C1PWBPN   |Case 1 Path Wide Bit Period Number (Read Only)
     * |        |          |The field indicates the final width bit clock number based 12M clock.
     * |        |          |Note: It is cleared in next preamble start or write 1.
     * |[13:8]  |C1PNBPN   |Case 1 Path Narrow Bit Period Number (Read Only)
     * |        |          |The field indicates the final narrow bit clcok number based 12M.
     * |        |          |Note: It is cleared in next preamble start or write 1.
     * |[15]    |C1PORF    |Case 1 Path Out of Range Flag
     * |        |          |0 = Case 1 isn't out of range.
     * |        |          |1 = Case 1 is out of range.
     * |        |          |Note: It is cleared in next preamble start or write 1.
     * |[18:16] |C1BRT     |Case 1 Path Best Round bnTimes
     * |        |          |The field indicates the best round times of bit width detection.
     * |        |          |Note: It is cleared in next preamble start or write 1.
     * @var UTCPD_T::C2PRR
     * Offset: 0x330  UTCPD Case 2 Path Record Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |C2PWBPN   |Case 2 Path Wide Bit Period Number (Read Only)
     * |        |          |The field indicates the final width bit clock number based 12M clock.
     * |        |          |Note: It is cleared in next preamble start or write 1.
     * |[13:8]  |C2PNBPN   |Case 2 Path Narrow Bit Period Number (Read Only)
     * |        |          |The field indicates the final narrow bit clcok number based 12M.
     * |        |          |Note: It is cleared in next preamble start or write 1.
     * |[15]    |C2PORF    |Case 2 Path Out of Range Flag
     * |        |          |0 = Case 2 isn't out of range.
     * |        |          |1 = Case 2 is out of range.
     * |        |          |Note: It is cleared in next preamble start or write 1.
     * |[18:16] |C2BRT     |Case 2 Path Best Round Times
     * |        |          |The field indicates the best round times of bit width detection.
     * |        |          |Note: It is cleared in next preamble start or write 1.
     * @var UTCPD_T::NBMCSLICE
     * Offset: 0x334  UTCPD New BMC SLICE Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |NSLICEL   |New TX Slice Low Level Control
     * |        |          |Low level slice control (The LSB is 20mV.)
     * |        |          |000 = 0.14V.
     * |        |          |001 = 0.16V.
     * |        |          |010 = 0.18V (Default).
     * |        |          |011 = 0.20V.
     * |        |          |100 = 0.22V.
     * |        |          |101 = 0.24V.
     * |        |          |110 = 0.26V.
     * |        |          |111 = 0.28V.
     * |[6:4]   |NSLICEH   |New TX Slice High Level Control
     * |        |          |High level slice control (The LSB is 20mV.)
     * |        |          |000 = 0.80V.
     * |        |          |001 = 0.82V.
     * |        |          |010 = 0.84V (Default).
     * |        |          |011 = 0.86V.
     * |        |          |100 = 0.88V.
     * |        |          |101 = 0.90V.
     * |        |          |110 = 0.92V.
     * |        |          |111 = 0.94V.
     * |[10:8]  |NSLICEM   |New TX Slice Middle Level Control
     * |        |          |Middle level slice control (The LSB is 20mV.)
     * |        |          |000 = 0.48V.
     * |        |          |001 = 0.50V.
     * |        |          |010 = 0.52V.
     * |        |          |011 = 0.54V.
     * |        |          |100 = 0.56V (Default).
     * |        |          |101 = 0.58V.
     * |        |          |110 = 0.60V.
     * |        |          |111 = 0.62V.
     * |[15]    |NBCMEN    |New BMC Trim Value Control Enable
     * |        |          |0 = The trim value of BMC is compatiable with UTCPD_BMCSLICE.
     * |        |          |1 = New BMC trim value in this regsiter bit field.
     * @var UTCPD_T::BMCDBGR
     * Offset: 0x338  UTCPD BMC SLICE Value Debug Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |SLICEL    |TX Slice Low Level Control (Read Only)
     * |        |          |Low level slice control (The LSB is 20mV.)
     * |        |          |000 = 0.14V.
     * |        |          |001 = 0.16V.
     * |        |          |010 = 0.18V (Default).
     * |        |          |011 = 0.20V.
     * |        |          |100 = 0.22V.
     * |        |          |101 = 0.24V.
     * |        |          |110 = 0.26V.
     * |        |          |111 = 0.28V.
     * |        |          |Note: For FPGA Debug and Read Only.
     * |[6:4]   |SLICEH    |TX Slice High Level Control (Read Only)
     * |        |          |High level slice control (The LSB is 20mV.)
     * |        |          |000 = 0.80V.
     * |        |          |001 = 0.82V.
     * |        |          |010 = 0.84V (Default).
     * |        |          |011 = 0.86V.
     * |        |          |100 = 0.88V.
     * |        |          |101 = 0.90V.
     * |        |          |110 = 0.92V.
     * |        |          |111 = 0.94V.
     * |        |          |Note: For FPGA Debug and Read Only.
     * |[10:8]  |SLICEM    |TX Slice Middle Level Control (Read Only)
     * |        |          |Middle level slice control (The LSB is 20mV.)
     * |        |          |000 = 0.48V.
     * |        |          |001 = 0.50V.
     * |        |          |010 = 0.52V.
     * |        |          |011 = 0.54V.
     * |        |          |100 = 0.56V (Default).
     * |        |          |101 = 0.58V.
     * |        |          |110 = 0.60V.
     * |        |          |111 = 0.62V.
     * |        |          |Note: For FPGA Debug and Read Only.
     */
    __I  uint32_t VID;                   /*!< [0x0000] UTCPD Vendor ID Register                                         */
    __IO uint32_t PID;                   /*!< [0x0004] UTCPD Product ID Register                                        */
    __I  uint32_t DID;                   /*!< [0x0008] UTCPD Device ID Register                                         */
    __I  uint32_t TCREV;                 /*!< [0x000c] UTCPD USB Type C Revision Register                               */
    __IO uint32_t PDREV;                 /*!< [0x0010] UTCPD USB PD Revision Register                                   */
    __IO uint32_t IS;                    /*!< [0x0014] UTCPD Interrupt Status Register                                  */
    __IO uint32_t IE;                    /*!< [0x0018] UTCPD Interrupt Enable Register                                  */
    __IO uint32_t PWRSTSIE;              /*!< [0x001c] UTCPD Power Status Interrupt Enable Register                     */
    __IO uint32_t FUTSTSIE;              /*!< [0x0020] UTCPD Fault Status Interrupt Enable Register                     */
    __IO uint32_t CTL;                   /*!< [0x0024] UTCPD Control Register                                           */
    __IO uint32_t PINPL;                 /*!< [0x0028] UTCPD Pin Polarity Control  Register                             */
    __IO uint32_t ROLCTL;                /*!< [0x002c] UTCPD Role Control Register                                      */
    __IO uint32_t FUTCTL;                /*!< [0x0030] UTCPD Fault Control Register                                     */
    __IO uint32_t PWRCTL;                /*!< [0x0034] UTCPD Power Control Register                                     */
    __I  uint32_t CCSTS;                 /*!< [0x0038] UTCPD CC Status Register                                         */
    __I  uint32_t PWRSTS;                /*!< [0x003c] UTCPD Power Status Register                                      */
    __IO uint32_t FUTSTS;                /*!< [0x0040] UTCPD Fault Status Register                                      */
    __IO uint32_t CMD;                   /*!< [0x0044] UTCPD Command Register                                           */
    __IO uint32_t DVCAP1;                /*!< [0x0048] UTCPD Device Capabilities 1 Register                             */
    __IO uint32_t DVCAP2;                /*!< [0x004c] UTCPD Device Capabilities 2 Register                             */
    __IO uint32_t MSHEAD;                /*!< [0x0050] UTCPD Message Header Info Register                               */
    __IO uint32_t DTRXEVNT;              /*!< [0x0054] UTCPD Enable Detect RX Event Register                            */
    __I  uint32_t RXBCNT;                /*!< [0x0058] UTCPD RX Byte Count Register                                     */
    __I  uint32_t RXFTYPE;               /*!< [0x005c] UTCPD Received Frame Type Register                               */
    __I  uint32_t RXHEAD;                /*!< [0x0060] UTCPD Received Header Data Register                              */
    __I  uint32_t RESERVE0[3];
    __I  uint32_t RXDA0;                 /*!< [0x0070] UTCPD Received Data0 Register                                    */
    __I  uint32_t RXDA1;                 /*!< [0x0074] UTCPD Received Data1 Register                                    */
    __I  uint32_t RXDA2;                 /*!< [0x0078] UTCPD Received Data2 Register                                    */
    __I  uint32_t RXDA3;                 /*!< [0x007c] UTCPD Received Data3 Register                                    */
    __I  uint32_t RXDA4;                 /*!< [0x0080] UTCPD Received Data4 Register                                    */
    __I  uint32_t RXDA5;                 /*!< [0x0084] UTCPD Received Data5 Register                                    */
    __I  uint32_t RXDA6;                 /*!< [0x0088] UTCPD Received Data6 Register                                    */
    __I  uint32_t RESERVE1[1];
    __IO uint32_t TXCTL;                 /*!< [0x0090] UTCPD TX Control Register                                        */
    __IO uint32_t TXBCNT;                /*!< [0x0094] UTCPD TX Byte Count Register                                     */
    __IO uint32_t TXHEAD;                /*!< [0x0098] UTCPD TX Header Data Register                                    */
    __I  uint32_t RESERVE2[1];
    __IO uint32_t TXDA0;                 /*!< [0x00a0] UTCPD Transmit Data0 Register                                    */
    __IO uint32_t TXDA1;                 /*!< [0x00a4] UTCPD Transmit Data1 Register                                    */
    __IO uint32_t TXDA2;                 /*!< [0x00a8] UTCPD Transmit Data2 Register                                    */
    __IO uint32_t TXDA3;                 /*!< [0x00ac] UTCPD Transmit Data3 Register                                    */
    __IO uint32_t TXDA4;                 /*!< [0x00b0] UTCPD Transmit Data4 Register                                    */
    __IO uint32_t TXDA5;                 /*!< [0x00b4] UTCPD Transmit Data5 Register                                    */
    __IO uint32_t TXDA6;                 /*!< [0x00b8] UTCPD Transmit Data6 Register                                    */
    __I  uint32_t RESERVE3[1];
    __IO uint32_t VBVOL;                 /*!< [0x00c0] UTCPD VBUS Voltage Register                                      */
    __IO uint32_t SKVBDCTH;              /*!< [0x00c4] UTCPD VBUS Sink disconnect threshold Register                    */
    __IO uint32_t SPDGTH;                /*!< [0x00c8] UTCPD VBUS Stop Discharge threshold Register                     */
    __IO uint32_t VBAMH;                 /*!< [0x00cc] UTCPD VBUS voltage high alarm threshold Register                 */
    __IO uint32_t VBAML;                 /*!< [0x00d0] UTCPD VBUS voltage low alarm threshold Register                  */
    __IO uint32_t VNDIS;                 /*!< [0x00d4] UTCPD Vendor defined Interrupt Status Register                   */
    __IO uint32_t VNDIE;                 /*!< [0x00d8] UTCPD Vendor defined Interrupt Enable Register                   */
    __IO uint32_t MUXSEL;                /*!< [0x00dc] UTCPD Mux Select Register                                        */
    __IO uint32_t VCDGCTL;               /*!< [0x00e0] UTCPD VCONN Discharge Control Register                           */
    __IO uint32_t PHYSLEW;               /*!< [0x00e4] UTCPD PHY Slew Rate Control Register                             */
    __IO uint32_t ADGTM;                 /*!< [0x00e8] UTCPD Auto Discharge Time Register                               */
    __IO uint32_t VSAFE0V;               /*!< [0x00ec] UTCPD Auto Discharge VSAFE0V Register                            */
    __IO uint32_t VSAFE5V;               /*!< [0x00f0] UTCPD VSAFE5V Register                                           */
    __IO uint32_t RATIO;                 /*!< [0x00f4] UTCPD DRP Toggle Ratio Register                                  */
    __IO uint32_t INTFRAME;              /*!< [0x00f8] UTCPD Inter-Frame Time Register                                  */
    __IO uint32_t VBOVTH;                /*!< [0x00fc] UTCPD VBUS Over Voltage Threshold Register                       */
    __IO uint32_t VNDINIT;               /*!< [0x0100] UTCPD Vendor Initial Register                                    */
    __IO uint32_t BMCTXBP;               /*!< [0x0104] UTCPD BMC TX Bit Period Register                                 */
    __IO uint32_t BMCTXDU;               /*!< [0x0108] UTCPD BMC TX Duty Register                                       */
    __IO uint32_t VCPSVOL;               /*!< [0x010c] VCONN Present Voltage Register                                   */
    __IO uint32_t VCUV;                  /*!< [0x0110] VCONN Under Voltage Register                                     */
    __I  uint32_t RESERVE4[1];
    __IO uint32_t BMCSLICE;              /*!< [0x0118] UTCPD BMC SLICE Control Register                                 */
    __IO uint32_t PHYCTL;                /*!< [0x011c] UTCPD PHY Power Control Register                                 */
    __IO uint32_t FRSRXCTL;              /*!< [0x0120] UTCPD CC Fast Swap RX Control Register                           */
    __I  uint32_t VCVOL;                 /*!< [0x0124] UTCPD VCONN Voltage Measurement Register                         */
    __I  uint32_t RESERVE5[118];
    __IO uint32_t CLKINFO;               /*!< [0x0300] UTCPD Clock Information Register                                 */
    __I  uint32_t RESERVE8[7];
    __IO uint32_t PREDET;                /*!< [0x0320] UTCPD Preamble Detect Register                                   */
    __I  uint32_t NIRR;                  /*!< [0x0324] UTCPD Noise Immunity Record Register                             */
    __I  uint32_t C0PRR;                 /*!< [0x0328] UTCPD Case 0 Path Record Register                                */
    __I  uint32_t C1PRR;                 /*!< [0x032c] UTCPD Case 1 Path Record Register                                */
    __I  uint32_t C2PRR;                 /*!< [0x0330] UTCPD Case 2 Path Record Register                                */
    __IO uint32_t NBMCSLICE;             /*!< [0x0334] UTCPD New BMC SLICE Control Register                             */
    __I  uint32_t BMCDBGR;               /*!< [0x0338] UTCPD BMC SLICE Value Debug Register                             */

} UTCPD_T;

/**
    @addtogroup UTCPD_CONST UTCPD Bit Field Definition
    Constant Definitions for UTCPD Controller
@{ */

#define UTCPD_VID_VID_Pos                (0)                                               /*!< UTCPD_T::VID: VID Position             */
#define UTCPD_VID_VID_Msk                (0xfffful << UTCPD_VID_VID_Pos)                   /*!< UTCPD_T::VID: VID Mask                 */

#define UTCPD_PID_PID_Pos                (0)                                               /*!< UTCPD_T::PID: PID Position             */
#define UTCPD_PID_PID_Msk                (0xfffful << UTCPD_PID_PID_Pos)                   /*!< UTCPD_T::PID: PID Mask                 */

#define UTCPD_DID_DID_Pos                (0)                                               /*!< UTCPD_T::DID: DID Position             */
#define UTCPD_DID_DID_Msk                (0xfffful << UTCPD_DID_DID_Pos)                   /*!< UTCPD_T::DID: DID Mask                 */

#define UTCPD_TCREV_TCREV_Pos            (0)                                               /*!< UTCPD_T::TCREV: TCREV Position         */
#define UTCPD_TCREV_TCREV_Msk            (0xfful << UTCPD_TCREV_TCREV_Pos)                 /*!< UTCPD_T::TCREV: TCREV Mask             */

#define UTCPD_PDREV_PDVER_Pos            (0)                                               /*!< UTCPD_T::PDREV: PDVER Position         */
#define UTCPD_PDREV_PDVER_Msk            (0xfful << UTCPD_PDREV_PDVER_Pos)                 /*!< UTCPD_T::PDREV: PDVER Mask             */

#define UTCPD_PDREV_PDREV_Pos            (8)                                               /*!< UTCPD_T::PDREV: PDREV Position         */
#define UTCPD_PDREV_PDREV_Msk            (0xfful << UTCPD_PDREV_PDREV_Pos)                 /*!< UTCPD_T::PDREV: PDREV Mask             */

#define UTCPD_IS_CCSCHIS_Pos             (0)                                               /*!< UTCPD_T::IS: CCSCHIS Position          */
#define UTCPD_IS_CCSCHIS_Msk             (0x1ul << UTCPD_IS_CCSCHIS_Pos)                   /*!< UTCPD_T::IS: CCSCHIS Mask              */

#define UTCPD_IS_PWRSCHIS_Pos            (1)                                               /*!< UTCPD_T::IS: PWRSCHIS Position         */
#define UTCPD_IS_PWRSCHIS_Msk            (0x1ul << UTCPD_IS_PWRSCHIS_Pos)                  /*!< UTCPD_T::IS: PWRSCHIS Mask             */

#define UTCPD_IS_RXSOPIS_Pos             (2)                                               /*!< UTCPD_T::IS: RXSOPIS Position          */
#define UTCPD_IS_RXSOPIS_Msk             (0x1ul << UTCPD_IS_RXSOPIS_Pos)                   /*!< UTCPD_T::IS: RXSOPIS Mask              */

#define UTCPD_IS_RXHRSTIS_Pos            (3)                                               /*!< UTCPD_T::IS: RXHRSTIS Position         */
#define UTCPD_IS_RXHRSTIS_Msk            (0x1ul << UTCPD_IS_RXHRSTIS_Pos)                  /*!< UTCPD_T::IS: RXHRSTIS Mask             */

#define UTCPD_IS_TXFALIS_Pos             (4)                                               /*!< UTCPD_T::IS: TXFALIS Position          */
#define UTCPD_IS_TXFALIS_Msk             (0x1ul << UTCPD_IS_TXFALIS_Pos)                   /*!< UTCPD_T::IS: TXFALIS Mask              */

#define UTCPD_IS_TXDCIS_Pos              (5)                                               /*!< UTCPD_T::IS: TXDCIS Position           */
#define UTCPD_IS_TXDCIS_Msk              (0x1ul << UTCPD_IS_TXDCIS_Pos)                    /*!< UTCPD_T::IS: TXDCIS Mask               */

#define UTCPD_IS_TXOKIS_Pos              (6)                                               /*!< UTCPD_T::IS: TXOKIS Position           */
#define UTCPD_IS_TXOKIS_Msk              (0x1ul << UTCPD_IS_TXOKIS_Pos)                    /*!< UTCPD_T::IS: TXOKIS Mask               */

#define UTCPD_IS_VBAMHIS_Pos             (7)                                               /*!< UTCPD_T::IS: VBAMHIS Position          */
#define UTCPD_IS_VBAMHIS_Msk             (0x1ul << UTCPD_IS_VBAMHIS_Pos)                   /*!< UTCPD_T::IS: VBAMHIS Mask              */

#define UTCPD_IS_VBAMLIS_Pos             (8)                                               /*!< UTCPD_T::IS: VBAMLIS Position          */
#define UTCPD_IS_VBAMLIS_Msk             (0x1ul << UTCPD_IS_VBAMLIS_Pos)                   /*!< UTCPD_T::IS: VBAMLIS Mask              */

#define UTCPD_IS_FUTIS_Pos               (9)                                               /*!< UTCPD_T::IS: FUTIS Position            */
#define UTCPD_IS_FUTIS_Msk               (0x1ul << UTCPD_IS_FUTIS_Pos)                     /*!< UTCPD_T::IS: FUTIS Mask                */

#define UTCPD_IS_RXOFIS_Pos              (10)                                              /*!< UTCPD_T::IS: RXOFIS Position           */
#define UTCPD_IS_RXOFIS_Msk              (0x1ul << UTCPD_IS_RXOFIS_Pos)                    /*!< UTCPD_T::IS: RXOFIS Mask               */

#define UTCPD_IS_SKDCDTIS_Pos            (11)                                              /*!< UTCPD_T::IS: SKDCDTIS Position         */
#define UTCPD_IS_SKDCDTIS_Msk            (0x1ul << UTCPD_IS_SKDCDTIS_Pos)                  /*!< UTCPD_T::IS: SKDCDTIS Mask             */

#define UTCPD_IS_VNDIS_Pos               (15)                                              /*!< UTCPD_T::IS: VNDIS Position            */
#define UTCPD_IS_VNDIS_Msk               (0x1ul << UTCPD_IS_VNDIS_Pos)                     /*!< UTCPD_T::IS: VNDIS Mask                */

#define UTCPD_IE_CCSCHIE_Pos             (0)                                               /*!< UTCPD_T::IE: CCSCHIE Position          */
#define UTCPD_IE_CCSCHIE_Msk             (0x1ul << UTCPD_IE_CCSCHIE_Pos)                   /*!< UTCPD_T::IE: CCSCHIE Mask              */

#define UTCPD_IE_PWRSCHIE_Pos            (1)                                               /*!< UTCPD_T::IE: PWRSCHIE Position         */
#define UTCPD_IE_PWRSCHIE_Msk            (0x1ul << UTCPD_IE_PWRSCHIE_Pos)                  /*!< UTCPD_T::IE: PWRSCHIE Mask             */

#define UTCPD_IE_RXSOPIE_Pos             (2)                                               /*!< UTCPD_T::IE: RXSOPIE Position          */
#define UTCPD_IE_RXSOPIE_Msk             (0x1ul << UTCPD_IE_RXSOPIE_Pos)                   /*!< UTCPD_T::IE: RXSOPIE Mask              */

#define UTCPD_IE_RXHRSTIE_Pos            (3)                                               /*!< UTCPD_T::IE: RXHRSTIE Position         */
#define UTCPD_IE_RXHRSTIE_Msk            (0x1ul << UTCPD_IE_RXHRSTIE_Pos)                  /*!< UTCPD_T::IE: RXHRSTIE Mask             */

#define UTCPD_IE_TXFAILIE_Pos            (4)                                               /*!< UTCPD_T::IE: TXFAILIE Position         */
#define UTCPD_IE_TXFAILIE_Msk            (0x1ul << UTCPD_IE_TXFAILIE_Pos)                  /*!< UTCPD_T::IE: TXFAILIE Mask             */

#define UTCPD_IE_TXDCIE_Pos              (5)                                               /*!< UTCPD_T::IE: TXDCIE Position           */
#define UTCPD_IE_TXDCIE_Msk              (0x1ul << UTCPD_IE_TXDCIE_Pos)                    /*!< UTCPD_T::IE: TXDCIE Mask               */

#define UTCPD_IE_TXSOKIE_Pos             (6)                                               /*!< UTCPD_T::IE: TXSOKIE Position          */
#define UTCPD_IE_TXSOKIE_Msk             (0x1ul << UTCPD_IE_TXSOKIE_Pos)                   /*!< UTCPD_T::IE: TXSOKIE Mask              */

#define UTCPD_IE_VBAMHIE_Pos             (7)                                               /*!< UTCPD_T::IE: VBAMHIE Position          */
#define UTCPD_IE_VBAMHIE_Msk             (0x1ul << UTCPD_IE_VBAMHIE_Pos)                   /*!< UTCPD_T::IE: VBAMHIE Mask              */

#define UTCPD_IE_VBAMLIE_Pos             (8)                                               /*!< UTCPD_T::IE: VBAMLIE Position          */
#define UTCPD_IE_VBAMLIE_Msk             (0x1ul << UTCPD_IE_VBAMLIE_Pos)                   /*!< UTCPD_T::IE: VBAMLIE Mask              */

#define UTCPD_IE_FUTIE_Pos               (9)                                               /*!< UTCPD_T::IE: FUTIE Position            */
#define UTCPD_IE_FUTIE_Msk               (0x1ul << UTCPD_IE_FUTIE_Pos)                     /*!< UTCPD_T::IE: FUTIE Mask                */

#define UTCPD_IE_RXOFIE_Pos              (10)                                              /*!< UTCPD_T::IE: RXOFIE Position           */
#define UTCPD_IE_RXOFIE_Msk              (0x1ul << UTCPD_IE_RXOFIE_Pos)                    /*!< UTCPD_T::IE: RXOFIE Mask               */

#define UTCPD_IE_SKDCDTIE_Pos            (11)                                              /*!< UTCPD_T::IE: SKDCDTIE Position         */
#define UTCPD_IE_SKDCDTIE_Msk            (0x1ul << UTCPD_IE_SKDCDTIE_Pos)                  /*!< UTCPD_T::IE: SKDCDTIE Mask             */

#define UTCPD_IE_VNDIE_Pos               (15)                                              /*!< UTCPD_T::IE: VNDIE Position            */
#define UTCPD_IE_VNDIE_Msk               (0x1ul << UTCPD_IE_VNDIE_Pos)                     /*!< UTCPD_T::IE: VNDIE Mask                */

#define UTCPD_PWRSTSIE_SKVBIE_Pos        (0)                                               /*!< UTCPD_T::PWRSTSIE: SKVBIE Position     */
#define UTCPD_PWRSTSIE_SKVBIE_Msk        (0x1ul << UTCPD_PWRSTSIE_SKVBIE_Pos)              /*!< UTCPD_T::PWRSTSIE: SKVBIE Mask         */

#define UTCPD_PWRSTSIE_VCPSIE_Pos        (1)                                               /*!< UTCPD_T::PWRSTSIE: VCPSIE Position     */
#define UTCPD_PWRSTSIE_VCPSIE_Msk        (0x1ul << UTCPD_PWRSTSIE_VCPSIE_Pos)              /*!< UTCPD_T::PWRSTSIE: VCPSIE Mask         */

#define UTCPD_PWRSTSIE_VBPSIE_Pos        (2)                                               /*!< UTCPD_T::PWRSTSIE: VBPSIE Position     */
#define UTCPD_PWRSTSIE_VBPSIE_Msk        (0x1ul << UTCPD_PWRSTSIE_VBPSIE_Pos)              /*!< UTCPD_T::PWRSTSIE: VBPSIE Mask         */

#define UTCPD_PWRSTSIE_VBDTDGIE_Pos      (3)                                               /*!< UTCPD_T::PWRSTSIE: VBDTDGIE Position   */
#define UTCPD_PWRSTSIE_VBDTDGIE_Msk      (0x1ul << UTCPD_PWRSTSIE_VBDTDGIE_Pos)            /*!< UTCPD_T::PWRSTSIE: VBDTDGIE Mask       */

#define UTCPD_PWRSTSIE_SRVBIE_Pos        (4)                                               /*!< UTCPD_T::PWRSTSIE: SRVBIE Position     */
#define UTCPD_PWRSTSIE_SRVBIE_Msk        (0x1ul << UTCPD_PWRSTSIE_SRVBIE_Pos)              /*!< UTCPD_T::PWRSTSIE: SRVBIE Mask         */

#define UTCPD_PWRSTSIE_SRHVIE_Pos        (5)                                               /*!< UTCPD_T::PWRSTSIE: SRHVIE Position     */
#define UTCPD_PWRSTSIE_SRHVIE_Msk        (0x1ul << UTCPD_PWRSTSIE_SRHVIE_Pos)              /*!< UTCPD_T::PWRSTSIE: SRHVIE Mask         */

#define UTCPD_PWRSTSIE_DACONIE_Pos       (7)                                               /*!< UTCPD_T::PWRSTSIE: DACONIE Position    */
#define UTCPD_PWRSTSIE_DACONIE_Msk       (0x1ul << UTCPD_PWRSTSIE_DACONIE_Pos)             /*!< UTCPD_T::PWRSTSIE: DACONIE Mask        */

#define UTCPD_FUTSTSIE_VCOCIE_Pos        (1)                                               /*!< UTCPD_T::FUTSTSIE: VCOCIE Position     */
#define UTCPD_FUTSTSIE_VCOCIE_Msk        (0x1ul << UTCPD_FUTSTSIE_VCOCIE_Pos)              /*!< UTCPD_T::FUTSTSIE: VCOCIE Mask         */

#define UTCPD_FUTSTSIE_VBOVIE_Pos        (2)                                               /*!< UTCPD_T::FUTSTSIE: VBOVIE Position     */
#define UTCPD_FUTSTSIE_VBOVIE_Msk        (0x1ul << UTCPD_FUTSTSIE_VBOVIE_Pos)              /*!< UTCPD_T::FUTSTSIE: VBOVIE Mask         */

#define UTCPD_FUTSTSIE_VBOCIE_Pos        (3)                                               /*!< UTCPD_T::FUTSTSIE: VBOCIE Position     */
#define UTCPD_FUTSTSIE_VBOCIE_Msk        (0x1ul << UTCPD_FUTSTSIE_VBOCIE_Pos)              /*!< UTCPD_T::FUTSTSIE: VBOCIE Mask         */

#define UTCPD_FUTSTSIE_FDGFALIE_Pos      (4)                                               /*!< UTCPD_T::FUTSTSIE: FDGFALIE Position   */
#define UTCPD_FUTSTSIE_FDGFALIE_Msk      (0x1ul << UTCPD_FUTSTSIE_FDGFALIE_Pos)            /*!< UTCPD_T::FUTSTSIE: FDGFALIE Mask       */

#define UTCPD_FUTSTSIE_ADGFALIE_Pos      (5)                                               /*!< UTCPD_T::FUTSTSIE: ADGFALIE Position   */
#define UTCPD_FUTSTSIE_ADGFALIE_Msk      (0x1ul << UTCPD_FUTSTSIE_ADGFALIE_Pos)            /*!< UTCPD_T::FUTSTSIE: ADGFALIE Mask       */

#define UTCPD_FUTSTSIE_FOFFVBIE_Pos      (6)                                               /*!< UTCPD_T::FUTSTSIE: FOFFVBIE Position   */
#define UTCPD_FUTSTSIE_FOFFVBIE_Msk      (0x1ul << UTCPD_FUTSTSIE_FOFFVBIE_Pos)            /*!< UTCPD_T::FUTSTSIE: FOFFVBIE Mask       */

#define UTCPD_CTL_ORIENT_Pos             (0)                                               /*!< UTCPD_T::CTL: ORIENT Position          */
#define UTCPD_CTL_ORIENT_Msk             (0x1ul << UTCPD_CTL_ORIENT_Pos)                   /*!< UTCPD_T::CTL: ORIENT Mask              */

#define UTCPD_CTL_BISTEN_Pos             (1)                                               /*!< UTCPD_T::CTL: BISTEN Position          */
#define UTCPD_CTL_BISTEN_Msk             (0x1ul << UTCPD_CTL_BISTEN_Pos)                   /*!< UTCPD_T::CTL: BISTEN Mask              */

#define UTCPD_PINPL_VBSRENPL_Pos         (0)                                               /*!< UTCPD_T::PINPL: VBSRENPL Position      */
#define UTCPD_PINPL_VBSRENPL_Msk         (0x1ul << UTCPD_PINPL_VBSRENPL_Pos)               /*!< UTCPD_T::PINPL: VBSRENPL Mask          */

#define UTCPD_PINPL_VBSKENPL_Pos         (1)                                               /*!< UTCPD_T::PINPL: VBSKENPL Position      */
#define UTCPD_PINPL_VBSKENPL_Msk         (0x1ul << UTCPD_PINPL_VBSKENPL_Pos)               /*!< UTCPD_T::PINPL: VBSKENPL Mask          */

#define UTCPD_PINPL_VBDGENPL_Pos         (2)                                               /*!< UTCPD_T::PINPL: VBDGENPL Position      */
#define UTCPD_PINPL_VBDGENPL_Msk         (0x1ul << UTCPD_PINPL_VBDGENPL_Pos)               /*!< UTCPD_T::PINPL: VBDGENPL Mask          */

#define UTCPD_PINPL_TXFRSPL_Pos          (3)                                               /*!< UTCPD_T::PINPL: TXFRSPL Position       */
#define UTCPD_PINPL_TXFRSPL_Msk          (0x1ul << UTCPD_PINPL_TXFRSPL_Pos)                /*!< UTCPD_T::PINPL: TXFRSPL Mask           */

#define UTCPD_PINPL_VCOCPL_Pos           (4)                                               /*!< UTCPD_T::PINPL: VCOCPL Position        */
#define UTCPD_PINPL_VCOCPL_Msk           (0x1ul << UTCPD_PINPL_VCOCPL_Pos)                 /*!< UTCPD_T::PINPL: VCOCPL Mask            */

#define UTCPD_PINPL_VBOCPL_Pos           (5)                                               /*!< UTCPD_T::PINPL: VBOCPL Position        */
#define UTCPD_PINPL_VBOCPL_Msk           (0x1ul << UTCPD_PINPL_VBOCPL_Pos)                 /*!< UTCPD_T::PINPL: VBOCPL Mask            */

#define UTCPD_PINPL_VCENPL_Pos           (8)                                               /*!< UTCPD_T::PINPL: VCENPL Position        */
#define UTCPD_PINPL_VCENPL_Msk           (0x1ul << UTCPD_PINPL_VCENPL_Pos)                 /*!< UTCPD_T::PINPL: VCENPL Mask            */

#define UTCPD_PINPL_VCDGENPL_Pos         (9)                                               /*!< UTCPD_T::PINPL: VCDGENPL Position      */
#define UTCPD_PINPL_VCDGENPL_Msk         (0x1ul << UTCPD_PINPL_VCDGENPL_Pos)               /*!< UTCPD_T::PINPL: VCDGENPL Mask          */

#define UTCPD_PINPL_FOFFVBPL_Pos         (10)                                              /*!< UTCPD_T::PINPL: FOFFVBPL Position      */
#define UTCPD_PINPL_FOFFVBPL_Msk         (0x1ul << UTCPD_PINPL_FOFFVBPL_Pos)               /*!< UTCPD_T::PINPL: FOFFVBPL Mask          */

#define UTCPD_ROLCTL_CC1_Pos             (0)                                               /*!< UTCPD_T::ROLCTL: CC1 Position          */
#define UTCPD_ROLCTL_CC1_Msk             (0x3ul << UTCPD_ROLCTL_CC1_Pos)                   /*!< UTCPD_T::ROLCTL: CC1 Mask              */

#define UTCPD_ROLCTL_CC2_Pos             (2)                                               /*!< UTCPD_T::ROLCTL: CC2 Position          */
#define UTCPD_ROLCTL_CC2_Msk             (0x3ul << UTCPD_ROLCTL_CC2_Pos)                   /*!< UTCPD_T::ROLCTL: CC2 Mask              */

#define UTCPD_ROLCTL_RPVALUE_Pos         (4)                                               /*!< UTCPD_T::ROLCTL: RPVALUE Position      */
#define UTCPD_ROLCTL_RPVALUE_Msk         (0x3ul << UTCPD_ROLCTL_RPVALUE_Pos)               /*!< UTCPD_T::ROLCTL: RPVALUE Mask          */

#define UTCPD_ROLCTL_DRP_Pos             (6)                                               /*!< UTCPD_T::ROLCTL: DRP Position          */
#define UTCPD_ROLCTL_DRP_Msk             (0x1ul << UTCPD_ROLCTL_DRP_Pos)                   /*!< UTCPD_T::ROLCTL: DRP Mask              */

#define UTCPD_FUTCTL_VCOCDTDS_Pos        (0)                                               /*!< UTCPD_T::FUTCTL: VCOCDTDS Position     */
#define UTCPD_FUTCTL_VCOCDTDS_Msk        (0x1ul << UTCPD_FUTCTL_VCOCDTDS_Pos)              /*!< UTCPD_T::FUTCTL: VCOCDTDS Mask         */

#define UTCPD_FUTCTL_VBOVDTDS_Pos        (1)                                               /*!< UTCPD_T::FUTCTL: VBOVDTDS Position     */
#define UTCPD_FUTCTL_VBOVDTDS_Msk        (0x1ul << UTCPD_FUTCTL_VBOVDTDS_Pos)              /*!< UTCPD_T::FUTCTL: VBOVDTDS Mask         */

#define UTCPD_FUTCTL_VBOCDTDS_Pos        (2)                                               /*!< UTCPD_T::FUTCTL: VBOCDTDS Position     */
#define UTCPD_FUTCTL_VBOCDTDS_Msk        (0x1ul << UTCPD_FUTCTL_VBOCDTDS_Pos)              /*!< UTCPD_T::FUTCTL: VBOCDTDS Mask         */

#define UTCPD_FUTCTL_VBDGTMDS_Pos        (3)                                               /*!< UTCPD_T::FUTCTL: VBDGTMDS Position     */
#define UTCPD_FUTCTL_VBDGTMDS_Msk        (0x1ul << UTCPD_FUTCTL_VBDGTMDS_Pos)              /*!< UTCPD_T::FUTCTL: VBDGTMDS Mask         */

#define UTCPD_FUTCTL_FOFFVBDS_Pos        (4)                                               /*!< UTCPD_T::FUTCTL: FOFFVBDS Position     */
#define UTCPD_FUTCTL_FOFFVBDS_Msk        (0x1ul << UTCPD_FUTCTL_FOFFVBDS_Pos)              /*!< UTCPD_T::FUTCTL: FOFFVBDS Mask         */

#define UTCPD_PWRCTL_VCEN_Pos            (0)                                               /*!< UTCPD_T::PWRCTL: VCEN Position         */
#define UTCPD_PWRCTL_VCEN_Msk            (0x1ul << UTCPD_PWRCTL_VCEN_Pos)                  /*!< UTCPD_T::PWRCTL: VCEN Mask             */

#define UTCPD_PWRCTL_VCPWR_Pos           (1)                                               /*!< UTCPD_T::PWRCTL: VCPWR Position        */
#define UTCPD_PWRCTL_VCPWR_Msk           (0x1ul << UTCPD_PWRCTL_VCPWR_Pos)                 /*!< UTCPD_T::PWRCTL: VCPWR Mask            */

#define UTCPD_PWRCTL_FDGEN_Pos           (2)                                               /*!< UTCPD_T::PWRCTL: FDGEN Position        */
#define UTCPD_PWRCTL_FDGEN_Msk           (0x1ul << UTCPD_PWRCTL_FDGEN_Pos)                 /*!< UTCPD_T::PWRCTL: FDGEN Mask            */

#define UTCPD_PWRCTL_BDGEN_Pos           (3)                                               /*!< UTCPD_T::PWRCTL: BDGEN Position        */
#define UTCPD_PWRCTL_BDGEN_Msk           (0x1ul << UTCPD_PWRCTL_BDGEN_Pos)                 /*!< UTCPD_T::PWRCTL: BDGEN Mask            */

#define UTCPD_PWRCTL_ADGDC_Pos           (4)                                               /*!< UTCPD_T::PWRCTL: ADGDC Position        */
#define UTCPD_PWRCTL_ADGDC_Msk           (0x1ul << UTCPD_PWRCTL_ADGDC_Pos)                 /*!< UTCPD_T::PWRCTL: ADGDC Mask            */

#define UTCPD_PWRCTL_DSVBAM_Pos          (5)                                               /*!< UTCPD_T::PWRCTL: DSVBAM Position       */
#define UTCPD_PWRCTL_DSVBAM_Msk          (0x1ul << UTCPD_PWRCTL_DSVBAM_Pos)                /*!< UTCPD_T::PWRCTL: DSVBAM Mask           */

#define UTCPD_PWRCTL_VBMONI_Pos          (6)                                               /*!< UTCPD_T::PWRCTL: VBMONI Position       */
#define UTCPD_PWRCTL_VBMONI_Msk          (0x1ul << UTCPD_PWRCTL_VBMONI_Pos)                /*!< UTCPD_T::PWRCTL: VBMONI Mask           */

#define UTCPD_CCSTS_CC1STATE_Pos         (0)                                               /*!< UTCPD_T::CCSTS: CC1STATE Position      */
#define UTCPD_CCSTS_CC1STATE_Msk         (0x3ul << UTCPD_CCSTS_CC1STATE_Pos)               /*!< UTCPD_T::CCSTS: CC1STATE Mask          */

#define UTCPD_CCSTS_CC2STATE_Pos         (2)                                               /*!< UTCPD_T::CCSTS: CC2STATE Position      */
#define UTCPD_CCSTS_CC2STATE_Msk         (0x3ul << UTCPD_CCSTS_CC2STATE_Pos)               /*!< UTCPD_T::CCSTS: CC2STATE Mask          */

#define UTCPD_CCSTS_CONRLT_Pos           (4)                                               /*!< UTCPD_T::CCSTS: CONRLT Position        */
#define UTCPD_CCSTS_CONRLT_Msk           (0x1ul << UTCPD_CCSTS_CONRLT_Pos)                 /*!< UTCPD_T::CCSTS: CONRLT Mask            */

#define UTCPD_CCSTS_LK4CONN_Pos          (5)                                               /*!< UTCPD_T::CCSTS: LK4CONN Position       */
#define UTCPD_CCSTS_LK4CONN_Msk          (0x1ul << UTCPD_CCSTS_LK4CONN_Pos)                /*!< UTCPD_T::CCSTS: LK4CONN Mask           */

#define UTCPD_PWRSTS_SKVB_Pos            (0)                                               /*!< UTCPD_T::PWRSTS: SKVB Position         */
#define UTCPD_PWRSTS_SKVB_Msk            (0x1ul << UTCPD_PWRSTS_SKVB_Pos)                  /*!< UTCPD_T::PWRSTS: SKVB Mask             */

#define UTCPD_PWRSTS_VCPS_Pos            (1)                                               /*!< UTCPD_T::PWRSTS: VCPS Position         */
#define UTCPD_PWRSTS_VCPS_Msk            (0x1ul << UTCPD_PWRSTS_VCPS_Pos)                  /*!< UTCPD_T::PWRSTS: VCPS Mask             */

#define UTCPD_PWRSTS_VBPS_Pos            (2)                                               /*!< UTCPD_T::PWRSTS: VBPS Position         */
#define UTCPD_PWRSTS_VBPS_Msk            (0x1ul << UTCPD_PWRSTS_VBPS_Pos)                  /*!< UTCPD_T::PWRSTS: VBPS Mask             */

#define UTCPD_PWRSTS_VBPSDTEN_Pos        (3)                                               /*!< UTCPD_T::PWRSTS: VBPSDTEN Position     */
#define UTCPD_PWRSTS_VBPSDTEN_Msk        (0x1ul << UTCPD_PWRSTS_VBPSDTEN_Pos)              /*!< UTCPD_T::PWRSTS: VBPSDTEN Mask         */

#define UTCPD_PWRSTS_SRVB_Pos            (4)                                               /*!< UTCPD_T::PWRSTS: SRVB Position         */
#define UTCPD_PWRSTS_SRVB_Msk            (0x1ul << UTCPD_PWRSTS_SRVB_Pos)                  /*!< UTCPD_T::PWRSTS: SRVB Mask             */

#define UTCPD_PWRSTS_SRHV_Pos            (5)                                               /*!< UTCPD_T::PWRSTS: SRHV Position         */
#define UTCPD_PWRSTS_SRHV_Msk            (0x1ul << UTCPD_PWRSTS_SRHV_Pos)                  /*!< UTCPD_T::PWRSTS: SRHV Mask             */

#define UTCPD_PWRSTS_DACON_Pos           (7)                                               /*!< UTCPD_T::PWRSTS: DACON Position        */
#define UTCPD_PWRSTS_DACON_Msk           (0x1ul << UTCPD_PWRSTS_DACON_Pos)                 /*!< UTCPD_T::PWRSTS: DACON Mask            */

#define UTCPD_FUTSTS_VCOCFUT_Pos         (1)                                               /*!< UTCPD_T::FUTSTS: VCOCFUT Position      */
#define UTCPD_FUTSTS_VCOCFUT_Msk         (0x1ul << UTCPD_FUTSTS_VCOCFUT_Pos)               /*!< UTCPD_T::FUTSTS: VCOCFUT Mask          */

#define UTCPD_FUTSTS_VBOVFUT_Pos         (2)                                               /*!< UTCPD_T::FUTSTS: VBOVFUT Position      */
#define UTCPD_FUTSTS_VBOVFUT_Msk         (0x1ul << UTCPD_FUTSTS_VBOVFUT_Pos)               /*!< UTCPD_T::FUTSTS: VBOVFUT Mask          */

#define UTCPD_FUTSTS_VBOCFUT_Pos         (3)                                               /*!< UTCPD_T::FUTSTS: VBOCFUT Position      */
#define UTCPD_FUTSTS_VBOCFUT_Msk         (0x1ul << UTCPD_FUTSTS_VBOCFUT_Pos)               /*!< UTCPD_T::FUTSTS: VBOCFUT Mask          */

#define UTCPD_FUTSTS_FDGFAL_Pos          (4)                                               /*!< UTCPD_T::FUTSTS: FDGFAL Position       */
#define UTCPD_FUTSTS_FDGFAL_Msk          (0x1ul << UTCPD_FUTSTS_FDGFAL_Pos)                /*!< UTCPD_T::FUTSTS: FDGFAL Mask           */

#define UTCPD_FUTSTS_ADGFAL_Pos          (5)                                               /*!< UTCPD_T::FUTSTS: ADGFAL Position       */
#define UTCPD_FUTSTS_ADGFAL_Msk          (0x1ul << UTCPD_FUTSTS_ADGFAL_Pos)                /*!< UTCPD_T::FUTSTS: ADGFAL Mask           */

#define UTCPD_FUTSTS_FOFFVB_Pos          (6)                                               /*!< UTCPD_T::FUTSTS: FOFFVB Position       */
#define UTCPD_FUTSTS_FOFFVB_Msk          (0x1ul << UTCPD_FUTSTS_FOFFVB_Pos)                /*!< UTCPD_T::FUTSTS: FOFFVB Mask           */

#define UTCPD_FUTSTS_INVHRST_Pos         (7)                                               /*!< UTCPD_T::FUTSTS: INVHRST Position      */
#define UTCPD_FUTSTS_INVHRST_Msk         (0x1ul << UTCPD_FUTSTS_INVHRST_Pos)               /*!< UTCPD_T::FUTSTS: INVHRST Mask          */

#define UTCPD_CMD_CMD_Pos                (0)                                               /*!< UTCPD_T::CMD: CMD Position             */
#define UTCPD_CMD_CMD_Msk                (0xfful << UTCPD_CMD_CMD_Pos)                     /*!< UTCPD_T::CMD: CMD Mask                 */

#define UTCPD_DVCAP1_CPSRVB_Pos          (0)                                               /*!< UTCPD_T::DVCAP1: CPSRVB Position       */
#define UTCPD_DVCAP1_CPSRVB_Msk          (0x1ul << UTCPD_DVCAP1_CPSRVB_Pos)                /*!< UTCPD_T::DVCAP1: CPSRVB Mask           */

#define UTCPD_DVCAP1_CPSRHV_Pos          (1)                                               /*!< UTCPD_T::DVCAP1: CPSRHV Position       */
#define UTCPD_DVCAP1_CPSRHV_Msk          (0x1ul << UTCPD_DVCAP1_CPSRHV_Pos)                /*!< UTCPD_T::DVCAP1: CPSRHV Mask           */

#define UTCPD_DVCAP1_CPSKVB_Pos          (2)                                               /*!< UTCPD_T::DVCAP1: CPSKVB Position       */
#define UTCPD_DVCAP1_CPSKVB_Msk          (0x1ul << UTCPD_DVCAP1_CPSKVB_Pos)                /*!< UTCPD_T::DVCAP1: CPSKVB Mask           */

#define UTCPD_DVCAP1_CPSRVC_Pos          (3)                                               /*!< UTCPD_T::DVCAP1: CPSRVC Position       */
#define UTCPD_DVCAP1_CPSRVC_Msk          (0x1ul << UTCPD_DVCAP1_CPSRVC_Pos)                /*!< UTCPD_T::DVCAP1: CPSRVC Mask           */

#define UTCPD_DVCAP1_CPSDBG_Pos          (4)                                               /*!< UTCPD_T::DVCAP1: CPSDBG Position       */
#define UTCPD_DVCAP1_CPSDBG_Msk          (0x1ul << UTCPD_DVCAP1_CPSDBG_Pos)                /*!< UTCPD_T::DVCAP1: CPSDBG Mask           */

#define UTCPD_DVCAP1_CPROL_Pos           (5)                                               /*!< UTCPD_T::DVCAP1: CPROL Position        */
#define UTCPD_DVCAP1_CPROL_Msk           (0x7ul << UTCPD_DVCAP1_CPROL_Pos)                 /*!< UTCPD_T::DVCAP1: CPROL Mask            */

#define UTCPD_DVCAP1_CPSRRE_Pos          (8)                                               /*!< UTCPD_T::DVCAP1: CPSRRE Position       */
#define UTCPD_DVCAP1_CPSRRE_Msk          (0x3ul << UTCPD_DVCAP1_CPSRRE_Pos)                /*!< UTCPD_T::DVCAP1: CPSRRE Mask           */

#define UTCPD_DVCAP1_CPVBAM_Pos          (10)                                              /*!< UTCPD_T::DVCAP1: CPVBAM Position       */
#define UTCPD_DVCAP1_CPVBAM_Msk          (0x1ul << UTCPD_DVCAP1_CPVBAM_Pos)                /*!< UTCPD_T::DVCAP1: CPVBAM Mask           */

#define UTCPD_DVCAP1_CPFDG_Pos           (11)                                              /*!< UTCPD_T::DVCAP1: CPFDG Position        */
#define UTCPD_DVCAP1_CPFDG_Msk           (0x1ul << UTCPD_DVCAP1_CPFDG_Pos)                 /*!< UTCPD_T::DVCAP1: CPFDG Mask            */

#define UTCPD_DVCAP1_CPBDG_Pos           (12)                                              /*!< UTCPD_T::DVCAP1: CPBDG Position        */
#define UTCPD_DVCAP1_CPBDG_Msk           (0x1ul << UTCPD_DVCAP1_CPBDG_Pos)                 /*!< UTCPD_T::DVCAP1: CPBDG Mask            */

#define UTCPD_DVCAP1_CPVBOVP_Pos         (13)                                              /*!< UTCPD_T::DVCAP1: CPVBOVP Position      */
#define UTCPD_DVCAP1_CPVBOVP_Msk         (0x1ul << UTCPD_DVCAP1_CPVBOVP_Pos)               /*!< UTCPD_T::DVCAP1: CPVBOVP Mask          */

#define UTCPD_DVCAP1_CPVBOCP_Pos         (14)                                              /*!< UTCPD_T::DVCAP1: CPVBOCP Position      */
#define UTCPD_DVCAP1_CPVBOCP_Msk         (0x1ul << UTCPD_DVCAP1_CPVBOCP_Pos)               /*!< UTCPD_T::DVCAP1: CPVBOCP Mask          */

#define UTCPD_DVCAP2_CPVCOC_Pos          (0)                                               /*!< UTCPD_T::DVCAP2: CPVCOC Position       */
#define UTCPD_DVCAP2_CPVCOC_Msk          (0x1ul << UTCPD_DVCAP2_CPVCOC_Pos)                /*!< UTCPD_T::DVCAP2: CPVCOC Mask           */

#define UTCPD_DVCAP2_CPVCPWR_Pos         (1)                                               /*!< UTCPD_T::DVCAP2: CPVCPWR Position      */
#define UTCPD_DVCAP2_CPVCPWR_Msk         (0x7ul << UTCPD_DVCAP2_CPVCPWR_Pos)               /*!< UTCPD_T::DVCAP2: CPVCPWR Mask          */

#define UTCPD_DVCAP2_CPVBAMLS_Pos        (4)                                               /*!< UTCPD_T::DVCAP2: CPVBAMLS Position     */
#define UTCPD_DVCAP2_CPVBAMLS_Msk        (0x3ul << UTCPD_DVCAP2_CPVBAMLS_Pos)              /*!< UTCPD_T::DVCAP2: CPVBAMLS Mask         */

#define UTCPD_DVCAP2_CPSPDGTH_Pos        (6)                                               /*!< UTCPD_T::DVCAP2: CPSPDGTH Position     */
#define UTCPD_DVCAP2_CPSPDGTH_Msk        (0x1ul << UTCPD_DVCAP2_CPSPDGTH_Pos)              /*!< UTCPD_T::DVCAP2: CPSPDGTH Mask         */

#define UTCPD_DVCAP2_CPSKDCDT_Pos        (7)                                               /*!< UTCPD_T::DVCAP2: CPSKDCDT Position     */
#define UTCPD_DVCAP2_CPSKDCDT_Msk        (0x1ul << UTCPD_DVCAP2_CPSKDCDT_Pos)              /*!< UTCPD_T::DVCAP2: CPSKDCDT Mask         */

#define UTCPD_MSHEAD_PWRROL_Pos          (0)                                               /*!< UTCPD_T::MSHEAD: PWRROL Position       */
#define UTCPD_MSHEAD_PWRROL_Msk          (0x1ul << UTCPD_MSHEAD_PWRROL_Pos)                /*!< UTCPD_T::MSHEAD: PWRROL Mask           */

#define UTCPD_MSHEAD_PDREV_Pos           (1)                                               /*!< UTCPD_T::MSHEAD: PDREV Position        */
#define UTCPD_MSHEAD_PDREV_Msk           (0x3ul << UTCPD_MSHEAD_PDREV_Pos)                 /*!< UTCPD_T::MSHEAD: PDREV Mask            */

#define UTCPD_MSHEAD_DAROL_Pos           (3)                                               /*!< UTCPD_T::MSHEAD: DAROL Position        */
#define UTCPD_MSHEAD_DAROL_Msk           (0x1ul << UTCPD_MSHEAD_DAROL_Pos)                 /*!< UTCPD_T::MSHEAD: DAROL Mask            */

#define UTCPD_MSHEAD_CABPLG_Pos          (4)                                               /*!< UTCPD_T::MSHEAD: CABPLG Position       */
#define UTCPD_MSHEAD_CABPLG_Msk          (0x1ul << UTCPD_MSHEAD_CABPLG_Pos)                /*!< UTCPD_T::MSHEAD: CABPLG Mask           */

#define UTCPD_DTRXEVNT_SOPEN_Pos         (0)                                               /*!< UTCPD_T::DTRXEVNT: SOPEN Position      */
#define UTCPD_DTRXEVNT_SOPEN_Msk         (0x1ul << UTCPD_DTRXEVNT_SOPEN_Pos)               /*!< UTCPD_T::DTRXEVNT: SOPEN Mask          */

#define UTCPD_DTRXEVNT_SOPPEN_Pos        (1)                                               /*!< UTCPD_T::DTRXEVNT: SOPPEN Position     */
#define UTCPD_DTRXEVNT_SOPPEN_Msk        (0x1ul << UTCPD_DTRXEVNT_SOPPEN_Pos)              /*!< UTCPD_T::DTRXEVNT: SOPPEN Mask         */

#define UTCPD_DTRXEVNT_SOPPPEN_Pos       (2)                                               /*!< UTCPD_T::DTRXEVNT: SOPPPEN Position    */
#define UTCPD_DTRXEVNT_SOPPPEN_Msk       (0x1ul << UTCPD_DTRXEVNT_SOPPPEN_Pos)             /*!< UTCPD_T::DTRXEVNT: SOPPPEN Mask        */

#define UTCPD_DTRXEVNT_SDBGPEN_Pos       (3)                                               /*!< UTCPD_T::DTRXEVNT: SDBGPEN Position    */
#define UTCPD_DTRXEVNT_SDBGPEN_Msk       (0x1ul << UTCPD_DTRXEVNT_SDBGPEN_Pos)             /*!< UTCPD_T::DTRXEVNT: SDBGPEN Mask        */

#define UTCPD_DTRXEVNT_SDBGPPEN_Pos      (4)                                               /*!< UTCPD_T::DTRXEVNT: SDBGPPEN Position   */
#define UTCPD_DTRXEVNT_SDBGPPEN_Msk      (0x1ul << UTCPD_DTRXEVNT_SDBGPPEN_Pos)            /*!< UTCPD_T::DTRXEVNT: SDBGPPEN Mask       */

#define UTCPD_DTRXEVNT_HRSTEN_Pos        (5)                                               /*!< UTCPD_T::DTRXEVNT: HRSTEN Position     */
#define UTCPD_DTRXEVNT_HRSTEN_Msk        (0x1ul << UTCPD_DTRXEVNT_HRSTEN_Pos)              /*!< UTCPD_T::DTRXEVNT: HRSTEN Mask         */

#define UTCPD_DTRXEVNT_CABRSTEN_Pos      (6)                                               /*!< UTCPD_T::DTRXEVNT: CABRSTEN Position   */
#define UTCPD_DTRXEVNT_CABRSTEN_Msk      (0x1ul << UTCPD_DTRXEVNT_CABRSTEN_Pos)            /*!< UTCPD_T::DTRXEVNT: CABRSTEN Mask       */

#define UTCPD_RXBCNT_RXBCNT_Pos          (0)                                               /*!< UTCPD_T::RXBCNT: RXBCNT Position       */
#define UTCPD_RXBCNT_RXBCNT_Msk          (0xfful << UTCPD_RXBCNT_RXBCNT_Pos)               /*!< UTCPD_T::RXBCNT: RXBCNT Mask           */

#define UTCPD_RXFTYPE_RXFTYPE_Pos        (0)                                               /*!< UTCPD_T::RXFTYPE: RXFTYPE Position     */
#define UTCPD_RXFTYPE_RXFTYPE_Msk        (0x7ul << UTCPD_RXFTYPE_RXFTYPE_Pos)              /*!< UTCPD_T::RXFTYPE: RXFTYPE Mask         */

#define UTCPD_RXHEAD_RXHEAD0_Pos         (0)                                               /*!< UTCPD_T::RXHEAD: RXHEAD0 Position      */
#define UTCPD_RXHEAD_RXHEAD0_Msk         (0xfful << UTCPD_RXHEAD_RXHEAD0_Pos)              /*!< UTCPD_T::RXHEAD: RXHEAD0 Mask          */

#define UTCPD_RXHEAD_RXHEAD1_Pos         (8)                                               /*!< UTCPD_T::RXHEAD: RXHEAD1 Position      */
#define UTCPD_RXHEAD_RXHEAD1_Msk         (0xfful << UTCPD_RXHEAD_RXHEAD1_Pos)              /*!< UTCPD_T::RXHEAD: RXHEAD1 Mask          */

#define UTCPD_RXDA0_RXDAB0_Pos           (0)                                               /*!< UTCPD_T::RXDA0: RXDAB0 Position        */
#define UTCPD_RXDA0_RXDAB0_Msk           (0xfful << UTCPD_RXDA0_RXDAB0_Pos)                /*!< UTCPD_T::RXDA0: RXDAB0 Mask            */

#define UTCPD_RXDA0_RXDAB1_Pos           (8)                                               /*!< UTCPD_T::RXDA0: RXDAB1 Position        */
#define UTCPD_RXDA0_RXDAB1_Msk           (0xfful << UTCPD_RXDA0_RXDAB1_Pos)                /*!< UTCPD_T::RXDA0: RXDAB1 Mask            */

#define UTCPD_RXDA0_RXDAB2_Pos           (16)                                              /*!< UTCPD_T::RXDA0: RXDAB2 Position        */
#define UTCPD_RXDA0_RXDAB2_Msk           (0xfful << UTCPD_RXDA0_RXDAB2_Pos)                /*!< UTCPD_T::RXDA0: RXDAB2 Mask            */

#define UTCPD_RXDA0_RXDAB3_Pos           (24)                                              /*!< UTCPD_T::RXDA0: RXDAB3 Position        */
#define UTCPD_RXDA0_RXDAB3_Msk           (0xfful << UTCPD_RXDA0_RXDAB3_Pos)                /*!< UTCPD_T::RXDA0: RXDAB3 Mask            */

#define UTCPD_RXDA1_RXDAB0_Pos           (0)                                               /*!< UTCPD_T::RXDA1: RXDAB0 Position        */
#define UTCPD_RXDA1_RXDAB0_Msk           (0xfful << UTCPD_RXDA1_RXDAB0_Pos)                /*!< UTCPD_T::RXDA1: RXDAB0 Mask            */

#define UTCPD_RXDA1_RXDAB1_Pos           (8)                                               /*!< UTCPD_T::RXDA1: RXDAB1 Position        */
#define UTCPD_RXDA1_RXDAB1_Msk           (0xfful << UTCPD_RXDA1_RXDAB1_Pos)                /*!< UTCPD_T::RXDA1: RXDAB1 Mask            */

#define UTCPD_RXDA1_RXDAB2_Pos           (16)                                              /*!< UTCPD_T::RXDA1: RXDAB2 Position        */
#define UTCPD_RXDA1_RXDAB2_Msk           (0xfful << UTCPD_RXDA1_RXDAB2_Pos)                /*!< UTCPD_T::RXDA1: RXDAB2 Mask            */

#define UTCPD_RXDA1_RXDAB3_Pos           (24)                                              /*!< UTCPD_T::RXDA1: RXDAB3 Position        */
#define UTCPD_RXDA1_RXDAB3_Msk           (0xfful << UTCPD_RXDA1_RXDAB3_Pos)                /*!< UTCPD_T::RXDA1: RXDAB3 Mask            */

#define UTCPD_RXDA2_RXDAB0_Pos           (0)                                               /*!< UTCPD_T::RXDA2: RXDAB0 Position        */
#define UTCPD_RXDA2_RXDAB0_Msk           (0xfful << UTCPD_RXDA2_RXDAB0_Pos)                /*!< UTCPD_T::RXDA2: RXDAB0 Mask            */

#define UTCPD_RXDA2_RXDAB1_Pos           (8)                                               /*!< UTCPD_T::RXDA2: RXDAB1 Position        */
#define UTCPD_RXDA2_RXDAB1_Msk           (0xfful << UTCPD_RXDA2_RXDAB1_Pos)                /*!< UTCPD_T::RXDA2: RXDAB1 Mask            */

#define UTCPD_RXDA2_RXDAB2_Pos           (16)                                              /*!< UTCPD_T::RXDA2: RXDAB2 Position        */
#define UTCPD_RXDA2_RXDAB2_Msk           (0xfful << UTCPD_RXDA2_RXDAB2_Pos)                /*!< UTCPD_T::RXDA2: RXDAB2 Mask            */

#define UTCPD_RXDA2_RXDAB3_Pos           (24)                                              /*!< UTCPD_T::RXDA2: RXDAB3 Position        */
#define UTCPD_RXDA2_RXDAB3_Msk           (0xfful << UTCPD_RXDA2_RXDAB3_Pos)                /*!< UTCPD_T::RXDA2: RXDAB3 Mask            */

#define UTCPD_RXDA3_RXDAB0_Pos           (0)                                               /*!< UTCPD_T::RXDA3: RXDAB0 Position        */
#define UTCPD_RXDA3_RXDAB0_Msk           (0xfful << UTCPD_RXDA3_RXDAB0_Pos)                /*!< UTCPD_T::RXDA3: RXDAB0 Mask            */

#define UTCPD_RXDA3_RXDAB1_Pos           (8)                                               /*!< UTCPD_T::RXDA3: RXDAB1 Position        */
#define UTCPD_RXDA3_RXDAB1_Msk           (0xfful << UTCPD_RXDA3_RXDAB1_Pos)                /*!< UTCPD_T::RXDA3: RXDAB1 Mask            */

#define UTCPD_RXDA3_RXDAB2_Pos           (16)                                              /*!< UTCPD_T::RXDA3: RXDAB2 Position        */
#define UTCPD_RXDA3_RXDAB2_Msk           (0xfful << UTCPD_RXDA3_RXDAB2_Pos)                /*!< UTCPD_T::RXDA3: RXDAB2 Mask            */

#define UTCPD_RXDA3_RXDAB3_Pos           (24)                                              /*!< UTCPD_T::RXDA3: RXDAB3 Position        */
#define UTCPD_RXDA3_RXDAB3_Msk           (0xfful << UTCPD_RXDA3_RXDAB3_Pos)                /*!< UTCPD_T::RXDA3: RXDAB3 Mask            */

#define UTCPD_RXDA4_RXDAB0_Pos           (0)                                               /*!< UTCPD_T::RXDA4: RXDAB0 Position        */
#define UTCPD_RXDA4_RXDAB0_Msk           (0xfful << UTCPD_RXDA4_RXDAB0_Pos)                /*!< UTCPD_T::RXDA4: RXDAB0 Mask            */

#define UTCPD_RXDA4_RXDAB1_Pos           (8)                                               /*!< UTCPD_T::RXDA4: RXDAB1 Position        */
#define UTCPD_RXDA4_RXDAB1_Msk           (0xfful << UTCPD_RXDA4_RXDAB1_Pos)                /*!< UTCPD_T::RXDA4: RXDAB1 Mask            */

#define UTCPD_RXDA4_RXDAB2_Pos           (16)                                              /*!< UTCPD_T::RXDA4: RXDAB2 Position        */
#define UTCPD_RXDA4_RXDAB2_Msk           (0xfful << UTCPD_RXDA4_RXDAB2_Pos)                /*!< UTCPD_T::RXDA4: RXDAB2 Mask            */

#define UTCPD_RXDA4_RXDAB3_Pos           (24)                                              /*!< UTCPD_T::RXDA4: RXDAB3 Position        */
#define UTCPD_RXDA4_RXDAB3_Msk           (0xfful << UTCPD_RXDA4_RXDAB3_Pos)                /*!< UTCPD_T::RXDA4: RXDAB3 Mask            */

#define UTCPD_RXDA5_RXDAB0_Pos           (0)                                               /*!< UTCPD_T::RXDA5: RXDAB0 Position        */
#define UTCPD_RXDA5_RXDAB0_Msk           (0xfful << UTCPD_RXDA5_RXDAB0_Pos)                /*!< UTCPD_T::RXDA5: RXDAB0 Mask            */

#define UTCPD_RXDA5_RXDAB1_Pos           (8)                                               /*!< UTCPD_T::RXDA5: RXDAB1 Position        */
#define UTCPD_RXDA5_RXDAB1_Msk           (0xfful << UTCPD_RXDA5_RXDAB1_Pos)                /*!< UTCPD_T::RXDA5: RXDAB1 Mask            */

#define UTCPD_RXDA5_RXDAB2_Pos           (16)                                              /*!< UTCPD_T::RXDA5: RXDAB2 Position        */
#define UTCPD_RXDA5_RXDAB2_Msk           (0xfful << UTCPD_RXDA5_RXDAB2_Pos)                /*!< UTCPD_T::RXDA5: RXDAB2 Mask            */

#define UTCPD_RXDA5_RXDAB3_Pos           (24)                                              /*!< UTCPD_T::RXDA5: RXDAB3 Position        */
#define UTCPD_RXDA5_RXDAB3_Msk           (0xfful << UTCPD_RXDA5_RXDAB3_Pos)                /*!< UTCPD_T::RXDA5: RXDAB3 Mask            */

#define UTCPD_RXDA6_RXDAB0_Pos           (0)                                               /*!< UTCPD_T::RXDA6: RXDAB0 Position        */
#define UTCPD_RXDA6_RXDAB0_Msk           (0xfful << UTCPD_RXDA6_RXDAB0_Pos)                /*!< UTCPD_T::RXDA6: RXDAB0 Mask            */

#define UTCPD_RXDA6_RXDAB1_Pos           (8)                                               /*!< UTCPD_T::RXDA6: RXDAB1 Position        */
#define UTCPD_RXDA6_RXDAB1_Msk           (0xfful << UTCPD_RXDA6_RXDAB1_Pos)                /*!< UTCPD_T::RXDA6: RXDAB1 Mask            */

#define UTCPD_RXDA6_RXDAB2_Pos           (16)                                              /*!< UTCPD_T::RXDA6: RXDAB2 Position        */
#define UTCPD_RXDA6_RXDAB2_Msk           (0xfful << UTCPD_RXDA6_RXDAB2_Pos)                /*!< UTCPD_T::RXDA6: RXDAB2 Mask            */

#define UTCPD_RXDA6_RXDAB3_Pos           (24)                                              /*!< UTCPD_T::RXDA6: RXDAB3 Position        */
#define UTCPD_RXDA6_RXDAB3_Msk           (0xfful << UTCPD_RXDA6_RXDAB3_Pos)                /*!< UTCPD_T::RXDA6: RXDAB3 Mask            */

#define UTCPD_TXCTL_TXSTYPE_Pos          (0)                                               /*!< UTCPD_T::TXCTL: TXSTYPE Position       */
#define UTCPD_TXCTL_TXSTYPE_Msk          (0x7ul << UTCPD_TXCTL_TXSTYPE_Pos)                /*!< UTCPD_T::TXCTL: TXSTYPE Mask           */

#define UTCPD_TXCTL_RETRYCNT_Pos         (4)                                               /*!< UTCPD_T::TXCTL: RETRYCNT Position      */
#define UTCPD_TXCTL_RETRYCNT_Msk         (0x3ul << UTCPD_TXCTL_RETRYCNT_Pos)               /*!< UTCPD_T::TXCTL: RETRYCNT Mask          */

#define UTCPD_TXBCNT_TXBCNT_Pos          (0)                                               /*!< UTCPD_T::TXBCNT: TXBCNT Position       */
#define UTCPD_TXBCNT_TXBCNT_Msk          (0xfful << UTCPD_TXBCNT_TXBCNT_Pos)               /*!< UTCPD_T::TXBCNT: TXBCNT Mask           */

#define UTCPD_TXHEAD_TXHEAD0_Pos         (0)                                               /*!< UTCPD_T::TXHEAD: TXHEAD0 Position      */
#define UTCPD_TXHEAD_TXHEAD0_Msk         (0xfful << UTCPD_TXHEAD_TXHEAD0_Pos)              /*!< UTCPD_T::TXHEAD: TXHEAD0 Mask          */

#define UTCPD_TXHEAD_TXHEAD1_Pos         (8)                                               /*!< UTCPD_T::TXHEAD: TXHEAD1 Position      */
#define UTCPD_TXHEAD_TXHEAD1_Msk         (0xfful << UTCPD_TXHEAD_TXHEAD1_Pos)              /*!< UTCPD_T::TXHEAD: TXHEAD1 Mask          */

#define UTCPD_TXDA0_TXDAB0_Pos           (0)                                               /*!< UTCPD_T::TXDA0: TXDAB0 Position        */
#define UTCPD_TXDA0_TXDAB0_Msk           (0xfful << UTCPD_TXDA0_TXDAB0_Pos)                /*!< UTCPD_T::TXDA0: TXDAB0 Mask            */

#define UTCPD_TXDA0_TXDAB1_Pos           (8)                                               /*!< UTCPD_T::TXDA0: TXDAB1 Position        */
#define UTCPD_TXDA0_TXDAB1_Msk           (0xfful << UTCPD_TXDA0_TXDAB1_Pos)                /*!< UTCPD_T::TXDA0: TXDAB1 Mask            */

#define UTCPD_TXDA0_TXDAB2_Pos           (16)                                              /*!< UTCPD_T::TXDA0: TXDAB2 Position        */
#define UTCPD_TXDA0_TXDAB2_Msk           (0xfful << UTCPD_TXDA0_TXDAB2_Pos)                /*!< UTCPD_T::TXDA0: TXDAB2 Mask            */

#define UTCPD_TXDA0_TXDAB3_Pos           (24)                                              /*!< UTCPD_T::TXDA0: TXDAB3 Position        */
#define UTCPD_TXDA0_TXDAB3_Msk           (0xfful << UTCPD_TXDA0_TXDAB3_Pos)                /*!< UTCPD_T::TXDA0: TXDAB3 Mask            */

#define UTCPD_TXDA1_TXDAB0_Pos           (0)                                               /*!< UTCPD_T::TXDA1: TXDAB0 Position        */
#define UTCPD_TXDA1_TXDAB0_Msk           (0xfful << UTCPD_TXDA1_TXDAB0_Pos)                /*!< UTCPD_T::TXDA1: TXDAB0 Mask            */

#define UTCPD_TXDA1_TXDAB1_Pos           (8)                                               /*!< UTCPD_T::TXDA1: TXDAB1 Position        */
#define UTCPD_TXDA1_TXDAB1_Msk           (0xfful << UTCPD_TXDA1_TXDAB1_Pos)                /*!< UTCPD_T::TXDA1: TXDAB1 Mask            */

#define UTCPD_TXDA1_TXDAB2_Pos           (16)                                              /*!< UTCPD_T::TXDA1: TXDAB2 Position        */
#define UTCPD_TXDA1_TXDAB2_Msk           (0xfful << UTCPD_TXDA1_TXDAB2_Pos)                /*!< UTCPD_T::TXDA1: TXDAB2 Mask            */

#define UTCPD_TXDA1_TXDAB3_Pos           (24)                                              /*!< UTCPD_T::TXDA1: TXDAB3 Position        */
#define UTCPD_TXDA1_TXDAB3_Msk           (0xfful << UTCPD_TXDA1_TXDAB3_Pos)                /*!< UTCPD_T::TXDA1: TXDAB3 Mask            */

#define UTCPD_TXDA2_TXDAB0_Pos           (0)                                               /*!< UTCPD_T::TXDA2: TXDAB0 Position        */
#define UTCPD_TXDA2_TXDAB0_Msk           (0xfful << UTCPD_TXDA2_TXDAB0_Pos)                /*!< UTCPD_T::TXDA2: TXDAB0 Mask            */

#define UTCPD_TXDA2_TXDAB1_Pos           (8)                                               /*!< UTCPD_T::TXDA2: TXDAB1 Position        */
#define UTCPD_TXDA2_TXDAB1_Msk           (0xfful << UTCPD_TXDA2_TXDAB1_Pos)                /*!< UTCPD_T::TXDA2: TXDAB1 Mask            */

#define UTCPD_TXDA2_TXDAB2_Pos           (16)                                              /*!< UTCPD_T::TXDA2: TXDAB2 Position        */
#define UTCPD_TXDA2_TXDAB2_Msk           (0xfful << UTCPD_TXDA2_TXDAB2_Pos)                /*!< UTCPD_T::TXDA2: TXDAB2 Mask            */

#define UTCPD_TXDA2_TXDAB3_Pos           (24)                                              /*!< UTCPD_T::TXDA2: TXDAB3 Position        */
#define UTCPD_TXDA2_TXDAB3_Msk           (0xfful << UTCPD_TXDA2_TXDAB3_Pos)                /*!< UTCPD_T::TXDA2: TXDAB3 Mask            */

#define UTCPD_TXDA3_TXDAB0_Pos           (0)                                               /*!< UTCPD_T::TXDA3: TXDAB0 Position        */
#define UTCPD_TXDA3_TXDAB0_Msk           (0xfful << UTCPD_TXDA3_TXDAB0_Pos)                /*!< UTCPD_T::TXDA3: TXDAB0 Mask            */

#define UTCPD_TXDA3_TXDAB1_Pos           (8)                                               /*!< UTCPD_T::TXDA3: TXDAB1 Position        */
#define UTCPD_TXDA3_TXDAB1_Msk           (0xfful << UTCPD_TXDA3_TXDAB1_Pos)                /*!< UTCPD_T::TXDA3: TXDAB1 Mask            */

#define UTCPD_TXDA3_TXDAB2_Pos           (16)                                              /*!< UTCPD_T::TXDA3: TXDAB2 Position        */
#define UTCPD_TXDA3_TXDAB2_Msk           (0xfful << UTCPD_TXDA3_TXDAB2_Pos)                /*!< UTCPD_T::TXDA3: TXDAB2 Mask            */

#define UTCPD_TXDA3_TXDAB3_Pos           (24)                                              /*!< UTCPD_T::TXDA3: TXDAB3 Position        */
#define UTCPD_TXDA3_TXDAB3_Msk           (0xfful << UTCPD_TXDA3_TXDAB3_Pos)                /*!< UTCPD_T::TXDA3: TXDAB3 Mask            */

#define UTCPD_TXDA4_TXDAB0_Pos           (0)                                               /*!< UTCPD_T::TXDA4: TXDAB0 Position        */
#define UTCPD_TXDA4_TXDAB0_Msk           (0xfful << UTCPD_TXDA4_TXDAB0_Pos)                /*!< UTCPD_T::TXDA4: TXDAB0 Mask            */

#define UTCPD_TXDA4_TXDAB1_Pos           (8)                                               /*!< UTCPD_T::TXDA4: TXDAB1 Position        */
#define UTCPD_TXDA4_TXDAB1_Msk           (0xfful << UTCPD_TXDA4_TXDAB1_Pos)                /*!< UTCPD_T::TXDA4: TXDAB1 Mask            */

#define UTCPD_TXDA4_TXDAB2_Pos           (16)                                              /*!< UTCPD_T::TXDA4: TXDAB2 Position        */
#define UTCPD_TXDA4_TXDAB2_Msk           (0xfful << UTCPD_TXDA4_TXDAB2_Pos)                /*!< UTCPD_T::TXDA4: TXDAB2 Mask            */

#define UTCPD_TXDA4_TXDAB3_Pos           (24)                                              /*!< UTCPD_T::TXDA4: TXDAB3 Position        */
#define UTCPD_TXDA4_TXDAB3_Msk           (0xfful << UTCPD_TXDA4_TXDAB3_Pos)                /*!< UTCPD_T::TXDA4: TXDAB3 Mask            */

#define UTCPD_TXDA5_TXDAB0_Pos           (0)                                               /*!< UTCPD_T::TXDA5: TXDAB0 Position        */
#define UTCPD_TXDA5_TXDAB0_Msk           (0xfful << UTCPD_TXDA5_TXDAB0_Pos)                /*!< UTCPD_T::TXDA5: TXDAB0 Mask            */

#define UTCPD_TXDA5_TXDAB1_Pos           (8)                                               /*!< UTCPD_T::TXDA5: TXDAB1 Position        */
#define UTCPD_TXDA5_TXDAB1_Msk           (0xfful << UTCPD_TXDA5_TXDAB1_Pos)                /*!< UTCPD_T::TXDA5: TXDAB1 Mask            */

#define UTCPD_TXDA5_TXDAB2_Pos           (16)                                              /*!< UTCPD_T::TXDA5: TXDAB2 Position        */
#define UTCPD_TXDA5_TXDAB2_Msk           (0xfful << UTCPD_TXDA5_TXDAB2_Pos)                /*!< UTCPD_T::TXDA5: TXDAB2 Mask            */

#define UTCPD_TXDA5_TXDAB3_Pos           (24)                                              /*!< UTCPD_T::TXDA5: TXDAB3 Position        */
#define UTCPD_TXDA5_TXDAB3_Msk           (0xfful << UTCPD_TXDA5_TXDAB3_Pos)                /*!< UTCPD_T::TXDA5: TXDAB3 Mask            */

#define UTCPD_TXDA6_TXDAB0_Pos           (0)                                               /*!< UTCPD_T::TXDA6: TXDAB0 Position        */
#define UTCPD_TXDA6_TXDAB0_Msk           (0xfful << UTCPD_TXDA6_TXDAB0_Pos)                /*!< UTCPD_T::TXDA6: TXDAB0 Mask            */

#define UTCPD_TXDA6_TXDAB1_Pos           (8)                                               /*!< UTCPD_T::TXDA6: TXDAB1 Position        */
#define UTCPD_TXDA6_TXDAB1_Msk           (0xfful << UTCPD_TXDA6_TXDAB1_Pos)                /*!< UTCPD_T::TXDA6: TXDAB1 Mask            */

#define UTCPD_TXDA6_TXDAB2_Pos           (16)                                              /*!< UTCPD_T::TXDA6: TXDAB2 Position        */
#define UTCPD_TXDA6_TXDAB2_Msk           (0xfful << UTCPD_TXDA6_TXDAB2_Pos)                /*!< UTCPD_T::TXDA6: TXDAB2 Mask            */

#define UTCPD_TXDA6_TXDAB3_Pos           (24)                                              /*!< UTCPD_T::TXDA6: TXDAB3 Position        */
#define UTCPD_TXDA6_TXDAB3_Msk           (0xfful << UTCPD_TXDA6_TXDAB3_Pos)                /*!< UTCPD_T::TXDA6: TXDAB3 Mask            */

#define UTCPD_VBVOL_VBVOL_Pos            (0)                                               /*!< UTCPD_T::VBVOL: VBVOL Position         */
#define UTCPD_VBVOL_VBVOL_Msk            (0x3fful << UTCPD_VBVOL_VBVOL_Pos)                /*!< UTCPD_T::VBVOL: VBVOL Mask             */

#define UTCPD_VBVOL_VBSCALE_Pos          (10)                                              /*!< UTCPD_T::VBVOL: VBSCALE Position       */
#define UTCPD_VBVOL_VBSCALE_Msk          (0x3ul << UTCPD_VBVOL_VBSCALE_Pos)                /*!< UTCPD_T::VBVOL: VBSCALE Mask           */

#define UTCPD_SKVBDCTH_SKVBDCTH_Pos      (0)                                               /*!< UTCPD_T::SKVBDCTH: SKVBDCTH Position   */
#define UTCPD_SKVBDCTH_SKVBDCTH_Msk      (0x3fful << UTCPD_SKVBDCTH_SKVBDCTH_Pos)          /*!< UTCPD_T::SKVBDCTH: SKVBDCTH Mask       */

#define UTCPD_SPDGTH_SPDGTH_Pos          (0)                                               /*!< UTCPD_T::SPDGTH: SPDGTH Position       */
#define UTCPD_SPDGTH_SPDGTH_Msk          (0x3fful << UTCPD_SPDGTH_SPDGTH_Pos)              /*!< UTCPD_T::SPDGTH: SPDGTH Mask           */

#define UTCPD_VBAMH_VBAMH_Pos            (0)                                               /*!< UTCPD_T::VBAMH: VBAMH Position         */
#define UTCPD_VBAMH_VBAMH_Msk            (0x3fful << UTCPD_VBAMH_VBAMH_Pos)                /*!< UTCPD_T::VBAMH: VBAMH Mask             */

#define UTCPD_VBAML_VBAML_Pos            (0)                                               /*!< UTCPD_T::VBAML: VBAML Position         */
#define UTCPD_VBAML_VBAML_Msk            (0x3fful << UTCPD_VBAML_VBAML_Pos)                /*!< UTCPD_T::VBAML: VBAML Mask             */

#define UTCPD_VNDIS_RXFRSIS_Pos          (0)                                               /*!< UTCPD_T::VNDIS: RXFRSIS Position       */
#define UTCPD_VNDIS_RXFRSIS_Msk          (0x1ul << UTCPD_VNDIS_RXFRSIS_Pos)                /*!< UTCPD_T::VNDIS: RXFRSIS Mask           */

#define UTCPD_VNDIS_TXFRSIS_Pos          (1)                                               /*!< UTCPD_T::VNDIS: TXFRSIS Position       */
#define UTCPD_VNDIS_TXFRSIS_Msk          (0x1ul << UTCPD_VNDIS_TXFRSIS_Pos)                /*!< UTCPD_T::VNDIS: TXFRSIS Mask           */

#define UTCPD_VNDIS_CRCERRIS_Pos         (3)                                               /*!< UTCPD_T::VNDIS: CRCERRIS Position      */
#define UTCPD_VNDIS_CRCERRIS_Msk         (0x1ul << UTCPD_VNDIS_CRCERRIS_Pos)               /*!< UTCPD_T::VNDIS: CRCERRIS Mask          */

#define UTCPD_VNDIS_VCDGIS_Pos           (5)                                               /*!< UTCPD_T::VNDIS: VCDGIS Position        */
#define UTCPD_VNDIS_VCDGIS_Msk           (0x1ul << UTCPD_VNDIS_VCDGIS_Pos)                 /*!< UTCPD_T::VNDIS: VCDGIS Mask            */

#define UTCPD_VNDIE_RXFRSIE_Pos          (0)                                               /*!< UTCPD_T::VNDIE: RXFRSIE Position       */
#define UTCPD_VNDIE_RXFRSIE_Msk          (0x1ul << UTCPD_VNDIE_RXFRSIE_Pos)                /*!< UTCPD_T::VNDIE: RXFRSIE Mask           */

#define UTCPD_VNDIE_TXFRSIE_Pos          (1)                                               /*!< UTCPD_T::VNDIE: TXFRSIE Position       */
#define UTCPD_VNDIE_TXFRSIE_Msk          (0x1ul << UTCPD_VNDIE_TXFRSIE_Pos)                /*!< UTCPD_T::VNDIE: TXFRSIE Mask           */

#define UTCPD_VNDIE_CRCERRIE_Pos         (3)                                               /*!< UTCPD_T::VNDIE: CRCERRIE Position      */
#define UTCPD_VNDIE_CRCERRIE_Msk         (0x1ul << UTCPD_VNDIE_CRCERRIE_Pos)               /*!< UTCPD_T::VNDIE: CRCERRIE Mask          */

#define UTCPD_VNDIE_VCDGIE_Pos           (5)                                               /*!< UTCPD_T::VNDIE: VCDGIE Position        */
#define UTCPD_VNDIE_VCDGIE_Msk           (0x1ul << UTCPD_VNDIE_VCDGIE_Pos)                 /*!< UTCPD_T::VNDIE: VCDGIE Mask            */

#define UTCPD_MUXSEL_VBOCS_Pos           (0)                                               /*!< UTCPD_T::MUXSEL: VBOCS Position        */
#define UTCPD_MUXSEL_VBOCS_Msk           (0xful << UTCPD_MUXSEL_VBOCS_Pos)                 /*!< UTCPD_T::MUXSEL: VBOCS Mask            */

#define UTCPD_MUXSEL_VCOCS_Pos           (4)                                               /*!< UTCPD_T::MUXSEL: VCOCS Position        */
#define UTCPD_MUXSEL_VCOCS_Msk           (0xful << UTCPD_MUXSEL_VCOCS_Pos)                 /*!< UTCPD_T::MUXSEL: VCOCS Mask            */

#define UTCPD_MUXSEL_FVBS_Pos            (8)                                               /*!< UTCPD_T::MUXSEL: FVBS Position         */
#define UTCPD_MUXSEL_FVBS_Msk            (0x7ul << UTCPD_MUXSEL_FVBS_Pos)                  /*!< UTCPD_T::MUXSEL: FVBS Mask             */

#define UTCPD_MUXSEL_ADCSELVB_Pos        (12)                                              /*!< UTCPD_T::MUXSEL: ADCSELVB Position     */
#define UTCPD_MUXSEL_ADCSELVB_Msk        (0x1ful << UTCPD_MUXSEL_ADCSELVB_Pos)             /*!< UTCPD_T::MUXSEL: ADCSELVB Mask         */

#define UTCPD_MUXSEL_ADCSELVC_Pos        (17)                                              /*!< UTCPD_T::MUXSEL: ADCSELVC Position     */
#define UTCPD_MUXSEL_ADCSELVC_Msk        (0x1ful << UTCPD_MUXSEL_ADCSELVC_Pos)             /*!< UTCPD_T::MUXSEL: ADCSELVC Mask         */

#define UTCPD_MUXSEL_CC1VCENS_Pos        (24)                                              /*!< UTCPD_T::MUXSEL: CC1VCENS Position     */
#define UTCPD_MUXSEL_CC1VCENS_Msk        (0x1ul << UTCPD_MUXSEL_CC1VCENS_Pos)              /*!< UTCPD_T::MUXSEL: CC1VCENS Mask         */

#define UTCPD_MUXSEL_CC1FRSS_Pos         (25)                                              /*!< UTCPD_T::MUXSEL: CC1FRSS Position      */
#define UTCPD_MUXSEL_CC1FRSS_Msk         (0x1ul << UTCPD_MUXSEL_CC1FRSS_Pos)               /*!< UTCPD_T::MUXSEL: CC1FRSS Mask          */

#define UTCPD_MUXSEL_CC2VCENS_Pos        (28)                                              /*!< UTCPD_T::MUXSEL: CC2VCENS Position     */
#define UTCPD_MUXSEL_CC2VCENS_Msk        (0x1ul << UTCPD_MUXSEL_CC2VCENS_Pos)              /*!< UTCPD_T::MUXSEL: CC2VCENS Mask         */

#define UTCPD_MUXSEL_CC2FRSS_Pos         (29)                                              /*!< UTCPD_T::MUXSEL: CC2FRSS Position      */
#define UTCPD_MUXSEL_CC2FRSS_Msk         (0x1ul << UTCPD_MUXSEL_CC2FRSS_Pos)               /*!< UTCPD_T::MUXSEL: CC2FRSS Mask          */

#define UTCPD_VCDGCTL_VCDGDTEN_Pos       (0)                                               /*!< UTCPD_T::VCDGCTL: VCDGDTEN Position    */
#define UTCPD_VCDGCTL_VCDGDTEN_Msk       (0x1ul << UTCPD_VCDGCTL_VCDGDTEN_Pos)             /*!< UTCPD_T::VCDGCTL: VCDGDTEN Mask        */

#define UTCPD_VCDGCTL_VCDGEN_Pos         (1)                                               /*!< UTCPD_T::VCDGCTL: VCDGEN Position      */
#define UTCPD_VCDGCTL_VCDGEN_Msk         (0x1ul << UTCPD_VCDGCTL_VCDGEN_Pos)               /*!< UTCPD_T::VCDGCTL: VCDGEN Mask          */

#define UTCPD_PHYSLEW_TXRTRIM_Pos        (0)                                               /*!< UTCPD_T::PHYSLEW: TXRTRIM Position     */
#define UTCPD_PHYSLEW_TXRTRIM_Msk        (0x7ul << UTCPD_PHYSLEW_TXRTRIM_Pos)              /*!< UTCPD_T::PHYSLEW: TXRTRIM Mask         */

#define UTCPD_PHYSLEW_TXFTRIM_Pos        (4)                                               /*!< UTCPD_T::PHYSLEW: TXFTRIM Position     */
#define UTCPD_PHYSLEW_TXFTRIM_Msk        (0x7ul << UTCPD_PHYSLEW_TXFTRIM_Pos)              /*!< UTCPD_T::PHYSLEW: TXFTRIM Mask         */

#define UTCPD_ADGTM_ADGTM_Pos            (0)                                               /*!< UTCPD_T::ADGTM: ADGTM Position         */
#define UTCPD_ADGTM_ADGTM_Msk            (0xfful << UTCPD_ADGTM_ADGTM_Pos)                 /*!< UTCPD_T::ADGTM: ADGTM Mask             */

#define UTCPD_VSAFE0V_VSAFE0V_Pos        (0)                                               /*!< UTCPD_T::VSAFE0V: VSAFE0V Position     */
#define UTCPD_VSAFE0V_VSAFE0V_Msk        (0x3fful << UTCPD_VSAFE0V_VSAFE0V_Pos)            /*!< UTCPD_T::VSAFE0V: VSAFE0V Mask         */

#define UTCPD_VSAFE5V_VSAFE5V_Pos        (0)                                               /*!< UTCPD_T::VSAFE5V: VSAFE5V Position     */
#define UTCPD_VSAFE5V_VSAFE5V_Msk        (0x3fful << UTCPD_VSAFE5V_VSAFE5V_Pos)            /*!< UTCPD_T::VSAFE5V: VSAFE5V Mask         */

#define UTCPD_RATIO_DRPRATIO_Pos         (0)                                               /*!< UTCPD_T::RATIO: DRPRATIO Position      */
#define UTCPD_RATIO_DRPRATIO_Msk         (0x7ul << UTCPD_RATIO_DRPRATIO_Pos)               /*!< UTCPD_T::RATIO: DRPRATIO Mask          */

#define UTCPD_RATIO_VBSEL_Pos            (3)                                               /*!< UTCPD_T::RATIO: VBSEL Position         */
#define UTCPD_RATIO_VBSEL_Msk            (0x1ul << UTCPD_RATIO_VBSEL_Pos)                  /*!< UTCPD_T::RATIO: VBSEL Mask             */

#define UTCPD_RATIO_ADCAVG_Pos           (6)                                               /*!< UTCPD_T::RATIO: ADCAVG Position        */
#define UTCPD_RATIO_ADCAVG_Msk           (0x1ul << UTCPD_RATIO_ADCAVG_Pos)                 /*!< UTCPD_T::RATIO: ADCAVG Mask            */

#define UTCPD_INTFRAME_INTFRAME_Pos      (0)                                               /*!< UTCPD_T::INTFRAME: INTFRAME Position   */
#define UTCPD_INTFRAME_INTFRAME_Msk      (0xfful << UTCPD_INTFRAME_INTFRAME_Pos)           /*!< UTCPD_T::INTFRAME: INTFRAME Mask       */

#define UTCPD_VBOVTH_VBOVTH_Pos          (0)                                               /*!< UTCPD_T::VBOVTH: VBOVTH Position       */
#define UTCPD_VBOVTH_VBOVTH_Msk          (0x3fful << UTCPD_VBOVTH_VBOVTH_Pos)              /*!< UTCPD_T::VBOVTH: VBOVTH Mask           */

#define UTCPD_VNDINIT_DVCAPDEF_Pos       (4)                                               /*!< UTCPD_T::VNDINIT: DVCAPDEF Position    */
#define UTCPD_VNDINIT_DVCAPDEF_Msk       (0x7ul << UTCPD_VNDINIT_DVCAPDEF_Pos)             /*!< UTCPD_T::VNDINIT: DVCAPDEF Mask        */

#define UTCPD_BMCTXBP_BMCTXBP_Pos        (0)                                               /*!< UTCPD_T::BMCTXBP: BMCTXBP Position     */
#define UTCPD_BMCTXBP_BMCTXBP_Msk        (0xfful << UTCPD_BMCTXBP_BMCTXBP_Pos)             /*!< UTCPD_T::BMCTXBP: BMCTXBP Mask         */

#define UTCPD_BMCTXDU_DUOFFS2_Pos        (0)                                               /*!< UTCPD_T::BMCTXDU: DUOFFS2 Position     */
#define UTCPD_BMCTXDU_DUOFFS2_Msk        (0x7ful << UTCPD_BMCTXDU_DUOFFS2_Pos)             /*!< UTCPD_T::BMCTXDU: DUOFFS2 Mask         */

#define UTCPD_BMCTXDU_DUOFFS1_Pos        (7)                                               /*!< UTCPD_T::BMCTXDU: DUOFFS1 Position     */
#define UTCPD_BMCTXDU_DUOFFS1_Msk        (0x1ul << UTCPD_BMCTXDU_DUOFFS1_Pos)              /*!< UTCPD_T::BMCTXDU: DUOFFS1 Mask         */

#define UTCPD_VCPSVOL_VCPSVOL_Pos        (0)                                               /*!< UTCPD_T::VCPSVOL: VCPSVOL Position     */
#define UTCPD_VCPSVOL_VCPSVOL_Msk        (0x3fful << UTCPD_VCPSVOL_VCPSVOL_Pos)            /*!< UTCPD_T::VCPSVOL: VCPSVOL Mask         */

#define UTCPD_VCUV_VCUV_Pos              (0)                                               /*!< UTCPD_T::VCUV: VCUV Position           */
#define UTCPD_VCUV_VCUV_Msk              (0x3fful << UTCPD_VCUV_VCUV_Pos)                  /*!< UTCPD_T::VCUV: VCUV Mask               */

#define UTCPD_BMCSLICE_SLICEL_Pos        (0)                                               /*!< UTCPD_T::BMCSLICE: SLICEL Position     */
#define UTCPD_BMCSLICE_SLICEL_Msk        (0x3ul << UTCPD_BMCSLICE_SLICEL_Pos)              /*!< UTCPD_T::BMCSLICE: SLICEL Mask         */

#define UTCPD_BMCSLICE_SLICEH_Pos        (2)                                               /*!< UTCPD_T::BMCSLICE: SLICEH Position     */
#define UTCPD_BMCSLICE_SLICEH_Msk        (0x3ul << UTCPD_BMCSLICE_SLICEH_Pos)              /*!< UTCPD_T::BMCSLICE: SLICEH Mask         */

#define UTCPD_BMCSLICE_SLICEM_Pos        (4)                                               /*!< UTCPD_T::BMCSLICE: SLICEM Position     */
#define UTCPD_BMCSLICE_SLICEM_Msk        (0x7ul << UTCPD_BMCSLICE_SLICEM_Pos)              /*!< UTCPD_T::BMCSLICE: SLICEM Mask         */

#define UTCPD_BMCSLICE_TRIMRD_Pos        (8)                                               /*!< UTCPD_T::BMCSLICE: TRIMRD Position     */
#define UTCPD_BMCSLICE_TRIMRD_Msk        (0x7ul << UTCPD_BMCSLICE_TRIMRD_Pos)              /*!< UTCPD_T::BMCSLICE: TRIMRD Mask         */

#define UTCPD_BMCSLICE_TRIMRP_Pos        (12)                                              /*!< UTCPD_T::BMCSLICE: TRIMRP Position     */
#define UTCPD_BMCSLICE_TRIMRP_Msk        (0xful << UTCPD_BMCSLICE_TRIMRP_Pos)              /*!< UTCPD_T::BMCSLICE: TRIMRP Mask         */

#define UTCPD_BMCSLICE_TRIMV1P1_Pos      (16)                                              /*!< UTCPD_T::BMCSLICE: TRIMV1P1 Position   */
#define UTCPD_BMCSLICE_TRIMV1P1_Msk      (0x7ul << UTCPD_BMCSLICE_TRIMV1P1_Pos)            /*!< UTCPD_T::BMCSLICE: TRIMV1P1 Mask       */

#define UTCPD_BMCSLICE_TRIMVBUS10_Pos    (20)                                              /*!< UTCPD_T::BMCSLICE: TRIMVBUS10 Position */
#define UTCPD_BMCSLICE_TRIMVBUS10_Msk    (0x7ul << UTCPD_BMCSLICE_TRIMVBUS10_Pos)          /*!< UTCPD_T::BMCSLICE: TRIMVBUS10 Mask     */

#define UTCPD_BMCSLICE_TRIMVBUS20_Pos    (24)                                              /*!< UTCPD_T::BMCSLICE: TRIMVBUS20 Position */
#define UTCPD_BMCSLICE_TRIMVBUS20_Msk    (0x7ul << UTCPD_BMCSLICE_TRIMVBUS20_Pos)          /*!< UTCPD_T::BMCSLICE: TRIMVBUS20 Mask     */

#define UTCPD_BMCSLICE_VTRIM_Pos         (28)                                              /*!< UTCPD_T::BMCSLICE: VTRIM Position      */
#define UTCPD_BMCSLICE_VTRIM_Msk         (0xful << UTCPD_BMCSLICE_VTRIM_Pos)               /*!< UTCPD_T::BMCSLICE: VTRIM Mask          */

#define UTCPD_PHYCTL_PHYPWR_Pos          (0)                                               /*!< UTCPD_T::PHYCTL: PHYPWR Position       */
#define UTCPD_PHYCTL_PHYPWR_Msk          (0x1ul << UTCPD_PHYCTL_PHYPWR_Pos)                /*!< UTCPD_T::PHYCTL: PHYPWR Mask           */

#define UTCPD_PHYCTL_DBCTL_Pos           (1)                                               /*!< UTCPD_T::PHYCTL: DBCTL Position        */
#define UTCPD_PHYCTL_DBCTL_Msk           (0x1ul << UTCPD_PHYCTL_DBCTL_Pos)                 /*!< UTCPD_T::PHYCTL: DBCTL Mask            */

#define UTCPD_FRSRXCTL_FRSTX_Pos         (0)                                               /*!< UTCPD_T::FRSRXCTL: FRSTX Position      */
#define UTCPD_FRSRXCTL_FRSTX_Msk         (0x1ul << UTCPD_FRSRXCTL_FRSTX_Pos)               /*!< UTCPD_T::FRSRXCTL: FRSTX Mask          */


#define UTCPD_FRSRXCTL_FRSDVVB_Pos       (2)                                               /*!< UTCPD_T::FRSRXCTL: FRSDVVB Position    */
#define UTCPD_FRSRXCTL_FRSDVVB_Msk       (0x1ul << UTCPD_FRSRXCTL_FRSDVVB_Pos)             /*!< UTCPD_T::FRSRXCTL: FRSDVVB Mask        */

#define UTCPD_FRSRXCTL_FRSRXEN_Pos       (3)                                               /*!< UTCPD_T::FRSRXCTL: FRSRXEN Position    */
#define UTCPD_FRSRXCTL_FRSRXEN_Msk       (0x1ul << UTCPD_FRSRXCTL_FRSRXEN_Pos)             /*!< UTCPD_T::FRSRXCTL: FRSRXEN Mask        */

#define UTCPD_VCVOL_VCVOL_Pos            (0)                                               /*!< UTCPD_T::VCVOL: VCVOL Position         */
#define UTCPD_VCVOL_VCVOL_Msk            (0x3fful << UTCPD_VCVOL_VCVOL_Pos)                /*!< UTCPD_T::VCVOL: VCVOL Mask             */

#define UTCPD_CLKINFO_ReadyFlag_Pos      (0)                                               /*!< UTCPD_T::CLKINFO: ReadyFlag Position   */
#define UTCPD_CLKINFO_ReadyFlag_Msk      (0x1ul << UTCPD_CLKINFO_ReadyFlag_Pos)            /*!< UTCPD_T::CLKINFO: ReadyFlag Mask       */

#define UTCPD_CLKINFO_WKEN_Pos           (4)                                               /*!< UTCPD_T::CLKINFO: WKEN Position        */
#define UTCPD_CLKINFO_WKEN_Msk           (0x1ul << UTCPD_CLKINFO_WKEN_Pos)                 /*!< UTCPD_T::CLKINFO: WKEN Mask            */

#define UTCPD_PREDET_RNICHK_Pos          (0)                                               /*!< UTCPD_T::PREDET: RNICHK Position       */
#define UTCPD_PREDET_RNICHK_Msk          (0xful << UTCPD_PREDET_RNICHK_Pos)                /*!< UTCPD_T::PREDET: RNICHK Mask           */

#define UTCPD_PREDET_BNICHK_Pos          (4)                                               /*!< UTCPD_T::PREDET: BNICHK Position       */
#define UTCPD_PREDET_BNICHK_Msk          (0x7ul << UTCPD_PREDET_BNICHK_Pos)                /*!< UTCPD_T::PREDET: BNICHK Mask           */

#define UTCPD_PREDET_BNW_Pos             (8)                                               /*!< UTCPD_T::PREDET: BNW Position          */
#define UTCPD_PREDET_BNW_Msk             (0x3ul << UTCPD_PREDET_BNW_Pos)                   /*!< UTCPD_T::PREDET: BNW Mask              */

#define UTCPD_PREDET_DGEN_Pos            (12)                                              /*!< UTCPD_T::PREDET: DGEN Position         */
#define UTCPD_PREDET_DGEN_Msk            (0x1ul << UTCPD_PREDET_DGEN_Pos)                  /*!< UTCPD_T::PREDET: DGEN Mask             */

#define UTCPD_PREDET_DGACT_Pos           (13)                                              /*!< UTCPD_T::PREDET: DGACT Position        */
#define UTCPD_PREDET_DGACT_Msk           (0x1ul << UTCPD_PREDET_DGACT_Pos)                 /*!< UTCPD_T::PREDET: DGACT Mask            */

#define UTCPD_PREDET_PSDB_Pos            (14)                                              /*!< UTCPD_T::PREDET: PSDB Position         */
#define UTCPD_PREDET_PSDB_Msk            (0x3ul << UTCPD_PREDET_PSDB_Pos)                  /*!< UTCPD_T::PREDET: PSDB Mask             */

#define UTCPD_PREDET_MINWBCN_Pos         (16)                                              /*!< UTCPD_T::PREDET: MINWBCN Position      */
#define UTCPD_PREDET_MINWBCN_Msk         (0x3ful << UTCPD_PREDET_MINWBCN_Pos)              /*!< UTCPD_T::PREDET: MINWBCN Mask          */

#define UTCPD_PREDET_MAXWBCN_Pos         (24)                                              /*!< UTCPD_T::PREDET: MAXWBCN Position      */
#define UTCPD_PREDET_MAXWBCN_Msk         (0x3ful << UTCPD_PREDET_MAXWBCN_Pos)              /*!< UTCPD_T::PREDET: MAXWBCN Mask          */

#define UTCPD_NIRR_WBW_Pos               (0)                                               /*!< UTCPD_T::NIRR: WBW Position            */
#define UTCPD_NIRR_WBW_Msk               (0x3ful << UTCPD_NIRR_WBW_Pos)                    /*!< UTCPD_T::NIRR: WBW Mask                */

#define UTCPD_NIRR_NBW_Pos               (8)                                               /*!< UTCPD_T::NIRR: NBW Position            */
#define UTCPD_NIRR_NBW_Msk               (0x3ful << UTCPD_NIRR_NBW_Pos)                    /*!< UTCPD_T::NIRR: NBW Mask                */

#define UTCPD_NIRR_CSEL_Pos              (14)                                              /*!< UTCPD_T::NIRR: CSEL Position           */
#define UTCPD_NIRR_CSEL_Msk              (0x3ul << UTCPD_NIRR_CSEL_Pos)                    /*!< UTCPD_T::NIRR: CSEL Mask               */

#define UTCPD_C0PRR_C0PWBPN_Pos          (0)                                               /*!< UTCPD_T::C0PRR: C0PWBPN Position       */
#define UTCPD_C0PRR_C0PWBPN_Msk          (0x3ful << UTCPD_C0PRR_C0PWBPN_Pos)               /*!< UTCPD_T::C0PRR: C0PWBPN Mask           */

#define UTCPD_C0PRR_C0PNBPN_Pos          (8)                                               /*!< UTCPD_T::C0PRR: C0PNBPN Position       */
#define UTCPD_C0PRR_C0PNBPN_Msk          (0x3ful << UTCPD_C0PRR_C0PNBPN_Pos)               /*!< UTCPD_T::C0PRR: C0PNBPN Mask           */

#define UTCPD_C0PRR_C0PORF_Pos           (15)                                              /*!< UTCPD_T::C0PRR: C0PORF Position        */
#define UTCPD_C0PRR_C0PORF_Msk           (0x1ul << UTCPD_C0PRR_C0PORF_Pos)                 /*!< UTCPD_T::C0PRR: C0PORF Mask            */

#define UTCPD_C0PRR_C0BRT_Pos            (16)                                              /*!< UTCPD_T::C0PRR: C0BRT Position         */
#define UTCPD_C0PRR_C0BRT_Msk            (0x7ul << UTCPD_C0PRR_C0BRT_Pos)                  /*!< UTCPD_T::C0PRR: C0BRT Mask             */

#define UTCPD_C1PRR_C1PWBPN_Pos          (0)                                               /*!< UTCPD_T::C1PRR: C1PWBPN Position       */
#define UTCPD_C1PRR_C1PWBPN_Msk          (0x3ful << UTCPD_C1PRR_C1PWBPN_Pos)               /*!< UTCPD_T::C1PRR: C1PWBPN Mask           */

#define UTCPD_C1PRR_C1PNBPN_Pos          (8)                                               /*!< UTCPD_T::C1PRR: C1PNBPN Position       */
#define UTCPD_C1PRR_C1PNBPN_Msk          (0x3ful << UTCPD_C1PRR_C1PNBPN_Pos)               /*!< UTCPD_T::C1PRR: C1PNBPN Mask           */

#define UTCPD_C1PRR_C1PORF_Pos           (15)                                              /*!< UTCPD_T::C1PRR: C1PORF Position        */
#define UTCPD_C1PRR_C1PORF_Msk           (0x1ul << UTCPD_C1PRR_C1PORF_Pos)                 /*!< UTCPD_T::C1PRR: C1PORF Mask            */

#define UTCPD_C1PRR_C1BRT_Pos            (16)                                              /*!< UTCPD_T::C1PRR: C1BRT Position         */
#define UTCPD_C1PRR_C1BRT_Msk            (0x7ul << UTCPD_C1PRR_C1BRT_Pos)                  /*!< UTCPD_T::C1PRR: C1BRT Mask             */

#define UTCPD_C2PRR_C2PWBPN_Pos          (0)                                               /*!< UTCPD_T::C2PRR: C2PWBPN Position       */
#define UTCPD_C2PRR_C2PWBPN_Msk          (0x3ful << UTCPD_C2PRR_C2PWBPN_Pos)               /*!< UTCPD_T::C2PRR: C2PWBPN Mask           */

#define UTCPD_C2PRR_C2PNBPN_Pos          (8)                                               /*!< UTCPD_T::C2PRR: C2PNBPN Position       */
#define UTCPD_C2PRR_C2PNBPN_Msk          (0x3ful << UTCPD_C2PRR_C2PNBPN_Pos)               /*!< UTCPD_T::C2PRR: C2PNBPN Mask           */

#define UTCPD_C2PRR_C2PORF_Pos           (15)                                              /*!< UTCPD_T::C2PRR: C2PORF Position        */
#define UTCPD_C2PRR_C2PORF_Msk           (0x1ul << UTCPD_C2PRR_C2PORF_Pos)                 /*!< UTCPD_T::C2PRR: C2PORF Mask            */

#define UTCPD_C2PRR_C2BRT_Pos            (16)                                              /*!< UTCPD_T::C2PRR: C2BRT Position         */
#define UTCPD_C2PRR_C2BRT_Msk            (0x7ul << UTCPD_C2PRR_C2BRT_Pos)                  /*!< UTCPD_T::C2PRR: C2BRT Mask             */

#define UTCPD_NBMCSLICE_NSLICEL_Pos      (0)                                               /*!< UTCPD_T::NBMCSLICE: NSLICEL Position   */
#define UTCPD_NBMCSLICE_NSLICEL_Msk      (0x7ul << UTCPD_NBMCSLICE_NSLICEL_Pos)            /*!< UTCPD_T::NBMCSLICE: NSLICEL Mask       */

#define UTCPD_NBMCSLICE_NSLICEH_Pos      (4)                                               /*!< UTCPD_T::NBMCSLICE: NSLICEH Position   */
#define UTCPD_NBMCSLICE_NSLICEH_Msk      (0x7ul << UTCPD_NBMCSLICE_NSLICEH_Pos)            /*!< UTCPD_T::NBMCSLICE: NSLICEH Mask       */

#define UTCPD_NBMCSLICE_NSLICEM_Pos      (8)                                               /*!< UTCPD_T::NBMCSLICE: NSLICEM Position   */
#define UTCPD_NBMCSLICE_NSLICEM_Msk      (0x7ul << UTCPD_NBMCSLICE_NSLICEM_Pos)            /*!< UTCPD_T::NBMCSLICE: NSLICEM Mask       */

#define UTCPD_NBMCSLICE_NBCMEN_Pos       (15)                                              /*!< UTCPD_T::NBMCSLICE: NBCMEN Position    */
#define UTCPD_NBMCSLICE_NBCMEN_Msk       (0x1ul << UTCPD_NBMCSLICE_NBCMEN_Pos)             /*!< UTCPD_T::NBMCSLICE: NBCMEN Mask        */

#define UTCPD_BMCDBGR_SLICEL_Pos         (0)                                               /*!< UTCPD_T::BMCDBGR: SLICEL Position      */
#define UTCPD_BMCDBGR_SLICEL_Msk         (0x7ul << UTCPD_BMCDBGR_SLICEL_Pos)               /*!< UTCPD_T::BMCDBGR: SLICEL Mask          */

#define UTCPD_BMCDBGR_SLICEH_Pos         (4)                                               /*!< UTCPD_T::BMCDBGR: SLICEH Position      */
#define UTCPD_BMCDBGR_SLICEH_Msk         (0x7ul << UTCPD_BMCDBGR_SLICEH_Pos)               /*!< UTCPD_T::BMCDBGR: SLICEH Mask          */

#define UTCPD_BMCDBGR_SLICEM_Pos         (8)                                               /*!< UTCPD_T::BMCDBGR: SLICEM Position      */
#define UTCPD_BMCDBGR_SLICEM_Msk         (0x7ul << UTCPD_BMCDBGR_SLICEM_Pos)               /*!< UTCPD_T::BMCDBGR: SLICEM Mask          */


/** @} UTCPD_CONST */
/** @} end of UTCPD register group */
/** @} end of REGISTER group */


#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __UTCPD_REG_H__ */
