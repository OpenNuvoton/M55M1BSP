/**************************************************************************//**
 * @file     lpadc_reg.h
 * @version  V1.00
 * @brief    LPADC register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __LPADC_REG_H__
#define __LPADC_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup LPADC Enhanced Analog to Digital Converter (LPADC)
    Memory Mapped Structure for LPADC Controller
    @{
*/

typedef struct
{
    /**
     * @var LPADC_T::ADDR[32]
     * Offset: 0x00/0x04/0x08/0x0C/0x10/0x14/0x18/0x1C/0x20/0x24/0x28/0x2C/0x30/0x34/0x38/0x3C/0x40/0x44/0x48/0x4C/0x50/0x54/0x58/0x5C/0x60/0x64/0x68/0x6C/0x70/0x74/0x78/0x7C  ADC Data Register 0~31
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RSLT      |A/D Conversion Result (Read Only)
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |If converted data in RSLT bits has not been read before new conversion result is loaded to this register, OVERRUN bit is set to 1
     * |        |          |It is cleared by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not overwritten.
     * |        |          |1 = Data in RSLT bits is overwritten.
     * |[17]    |VALID     |Valid Flag (Read Only)
     * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is completed
     * |        |          |This bit will be cleared to 0 by hardware after ADDR register is read.
     * |        |          |0 = Data in RSLT bits is not valid.
     * |        |          |1 = Data in RSLT bits is valid.
     * @var LPADC_T::ADCR
     * Offset: 0x80  ADC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADEN      |A/D Converter Enable Bit
     * |        |          |0 = A/D converter Disabled.
     * |        |          |1 = A/D converter Enabled.
     * |        |          |Note: Before starting A/D conversion function, this bit should be set to 1
     * |        |          |Clear it to 0 to disable A/D converter analog circuit to save power consumption.
     * |[1]     |ADIE      |A/D Interrupt Enable Bit
     * |        |          |A/D conversion end interrupt request is generated if ADIE bit is set to 1.
     * |        |          |0 = A/D interrupt function Disabled.
     * |        |          |1 = A/D interrupt function Enabled.
     * |[3:2]   |ADMD      |A/D Converter Operation Mode Control
     * |        |          |00 = Single conversion.
     * |        |          |01 = Burst conversion.
     * |        |          |10 = Single-cycle Scan.
     * |        |          |11 = Continuous Scan.
     * |        |          |Note 1: When changing the operation mode, software should clear ADST(LPADC_ADCR[11]) bit first.
     * |        |          |Note 2: In Burst mode, the A/D result data is always at ADC Data Register 0.
     * |[7:6]   |TRGCOND   |External Trigger Condition
     * |        |          |These two bits decide external pin LPADC0_ST trigger event is level or edge
     * |        |          |The signal must be kept at stable state at least 8 PCLKs for level trigger and at least 4 PCLKs for edge trigger.
     * |        |          |00 = Low level.
     * |        |          |01 = High level.
     * |        |          |10 = Falling edge.
     * |        |          |11 = Rising edge.
     * |[8]     |TRGEN     |External Trigger Enable Bit
     * |        |          |Enable or disable triggering of A/D conversion by external LPADC0_ST pin, EPWM trigger, BPWM trigger and ACMP interrupt event
     * |        |          |If external trigger is enabled, the ADST(LPADC_ADCR[11]) bit can be set to 1 by the selected hardware trigger source.
     * |        |          |0 = External trigger Disabled.
     * |        |          |1 = External trigger Enabled.
     * |        |          |Note: The ADC external trigger function is only supported in Single-cycle Scan mode.
     * |[9]     |PTEN      |LPPDMA Transfer Enable Bit
     * |        |          |When A/D conversion is completed, the converted data is loaded into ADDR0~23 and ADDR28~ADDR31
     * |        |          |Software can enable this bit to generate a LPPDMA data transfer request.
     * |        |          |0 = LPPDMA data transfer Disabled.
     * |        |          |1 = LPPDMA data transfer in ADDR 0~23 and ADDR28~ADDR31 Enabled.
     * |        |          |Note: When PTEN=1, software must set ADIE=0 to disable interrupt.
     * |[10]    |DIFFEN    |Differential Input Mode Control
     * |        |          |Differential input voltage (Vdiff) = Vplus - Vminus.
     * |        |          |The relation between Vplus and Vminus is Vplus + Vminus = Vref.
     * |        |          |The Vplus of differential input paired channel x is from LPADC0_CHy pin; Vminus is from LPADC0_CHz pin, x=0,1..7, y=2*x, z=y+1.
     * |        |          |0 = Single-end analog input mode.
     * |        |          |1 = Differential analog input mode.
     * |        |          |Note: In Differential Input mode, only the even number of the two corresponding channels needs to be enabled in ADCHER register
     * |        |          |The conversion result will be placed to the corresponding data register of the enabled channel.
     * |[11]    |ADST      |A/D Conversion Start or Calibration Start
     * |        |          |ADST bit can be set to 1 from below sources: software, external pin LPADC0_ST, BPWM trigger, EPWM trigger and ACMP interrupt event
     * |        |          |ADST bit will be cleared to 0 by hardware automatically at the ends of Single mode, Single-cycle Scan mode and Calibration mode
     * |        |          |In Continuous Scan mode and Burst mode, A/D conversion is continuously performed until software writes 0 to this bit or chip is reset.
     * |        |          |0 = Conversion stops and A/D converter enters idle state.
     * |        |          |1 = Conversion starts or Calibration Start.
     * |        |          |Note 1: When ADST becomes from 1 to 0, ADC macro will reset to initial state
     * |        |          |After macro reset to initial state, user should wait at most 2 ADC clock and set this bit to start next conversion.
     * |        |          |Note 2: Calibration Start only if CALEN (LPADC_ADCAL[0]) = 1.
     * |[12]    |RESET     |ADC RESET (Write Protect)
     * |        |          |If user writes this bit, the ADC analog macro will reset
     * |        |          |Calibration data in macro will be deleted, but registers in ADC controller will keep.
     * |        |          |Note: This bit is cleared by hardware.
     * |[19:16] |TRGS      |Hardware Trigger Source
     * |        |          |0000 = A/D conversion is started by external LPADC0_ST pin.
     * |        |          |0010 = A/D conversion is started by BPWM trigger.
     * |        |          |0100 = A/D conversion is started by EPWM trigger.
     * |        |          |1000 = ACMP0 interrupt event.
     * |        |          |1001 = ACMP1 interrupt event.
     * |        |          |1010 = ACMP2 interrupt event.
     * |        |          |1011 = ACMP3 interrupt event.
     * |        |          |Others = Reserved.
     * |        |          |Note: Software should clear TRGEN bit and ADST(LPADC_ADCR[11]) bit to 0 before changing TRGS bits.
     * |[29:28] |RESSEL    |Resolution Select Bits
     * |        |          |00 = ADC resolution 12 bits.
     * |        |          |01 = ADC resolution 10 bits.
     * |        |          |10 = ADC resolution 8 bits.
     * |        |          |11 = Reserved.
     * |[31]    |DMOF      |Differential Input Mode Output Format
     * |        |          |If user enables differential input mode, the conversion result can be expressed with binary straight format (unsigned format) or 2's complement format (signed format).
     * |        |          |0 = A/D Conversion result will be filled in RSLT at ADDRx registers with unsigned format (straight binary format).
     * |        |          |1 = A/D Conversion result will be filled in RSLT at ADDRx registers with 2's complement format.
     * @var LPADC_T::ADCHER
     * Offset: 0x84  ADC Channel Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |CHEN      |Analog Input Channel Enable Control
     * |        |          |Set CHEN [23:0] bits to enable the corresponding analog external input channel 23 ~ 0
     * |        |          |If DIFFEN bit is set to 1, only the even number channel needs to be enabled.
     * |        |          |Besides, setting the CHEN [28] bit will enable internal channel for temperature sensor output voltage , setting the CHEN [29] bit will enable internal channel for AVDD divided by four, setting the CHEN [30] bit will enable internal channel for VBAT divided by four , setting the CHEN [31] bit will enable internal channel for band-gap voltage
     * |        |          |Other bits are reserved.
     * |        |          |0 = Channel Disabled.
     * |        |          |1 = Channel Enabled.
     * |        |          |Note: If the internal channel for AVDD divide by four (CHEN[28]), band-gap voltage (CHEN[29]), temperature sensor output voltage (CHEN[30]) or VBAT divided by four (CHEN[31]) is active, the maximum sampling rate will be 300k SPS.
     * @var LPADC_T::ADCMPR[2]
     * Offset: 0x88/0x8C  ADC Compare Register 0~1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CMPEN     |Compare Enable Bit
     * |        |          |Set this bit to 1 to enable ADC controller to compare CMPD (ADCMPRx[27:16]) with specified channel conversion result when converted data is loaded into ADDR register.
     * |        |          |0 = Compare function Disabled.
     * |        |          |1 = Compare function Enabled.
     * |[1]     |CMPIE     |Compare Interrupt Enable Bit
     * |        |          |If the compare function is enabled and the compare condition matches the setting of CMPCOND (ADCMPRx[2]) and CMPMCNT (ADCMPRx[11:8]), CMPFx(LPADC_ADSR0[1]/[2]) bit will be asserted, in the meanwhile, if CMPIE bit is set to 1, a compare interrupt request is generated.
     * |        |          |0 = Compare function interrupt Disabled.
     * |        |          |1 = Compare function interrupt Enabled.
     * |[2]     |CMPCOND   |Compare Condition
     * |        |          |0 = Set the compare condition as that when a 12-bit A/D conversion result is less than the 12-bit CMPD (ADCMPRx[27:16]), the internal match counter will increase one.
     * |        |          |1 = Set the compare condition as that when a 12-bit A/D conversion result is greater than or equal to the 12-bit CMPD (ADCMPRx[27:16]), the internal match counter will increase one.
     * |        |          |Note: When the internal counter reaches to (CMPMCNT +1), the CMPFx(LPADC_ADSR0[1]/[2]) bit will be set.
     * |[7:3]   |CMPCH     |Compare Channel Selection
     * |        |          |00000 = Channel 0 conversion result is selected to be compared.
     * |        |          |00001 = Channel 1 conversion result is selected to be compared.
     * |        |          |00010 = Channel 2 conversion result is selected to be compared.
     * |        |          |00011 = Channel 3 conversion result is selected to be compared.
     * |        |          |00100 = Channel 4 conversion result is selected to be compared.
     * |        |          |00101 = Channel 5 conversion result is selected to be compared.
     * |        |          |00110 = Channel 6 conversion result is selected to be compared.
     * |        |          |00111 = Channel 7 conversion result is selected to be compared.
     * |        |          |01000 = Channel 8 conversion result is selected to be compared.
     * |        |          |01001 = Channel 9 conversion result is selected to be compared.
     * |        |          |01010 = Channel 10 conversion result is selected to be compared.
     * |        |          |01011 = Channel 11 conversion result is selected to be compared.
     * |        |          |01100 = Channel 12 conversion result is selected to be compared.
     * |        |          |01101 = Channel 13 conversion result is selected to be compared.
     * |        |          |01110 = Channel 14 conversion result is selected to be compared.
     * |        |          |01111 = Channel 15 conversion result is selected to be compared.
     * |        |          |10000 = Channel 16 conversion result is selected to be compared.
     * |        |          |10001 = Channel 17 conversion result is selected to be compared.
     * |        |          |10010 = Channel 18 conversion result is selected to be compared.
     * |        |          |10011 = Channel 19 conversion result is selected to be compared.
     * |        |          |10100 = Channel 20 conversion result is selected to be compared.
     * |        |          |10101 = Channel 21 conversion result is selected to be compared.
     * |        |          |10110 = Channel 22 conversion result is selected to be compared.
     * |        |          |10111 = Channel 23 conversion result is selected to be compared.
     * |        |          |11100 = Temperature sensor voltage conversion result is selected to be compared.
     * |        |          |11101 = AVDD/4 channel conversion result is selected to be compared.
     * |        |          |11110 = VBAT/4 conversion result is selected to be compared.
     * |        |          |11111 = Band-gap voltage channel conversion result is selected to be compared.
     * |        |          |Others = Reserved.
     * |[11:8]  |CMPMCNT   |Compare Match Count
     * |        |          |When the specified A/D channel analog conversion result matches the compare condition defined by CMPCOND (LPADC_ADCMPRx[2]), the internal match counter will increase 1
     * |        |          |When the internal counter reaches the value to (CMPMCNT +1), the CMPFx(LPADC_ADSR0[1]/[2]) bit will be set.
     * |[15]    |CMPWEN    |Compare Window Mode Enable Bit
     * |        |          |0 = Compare Window Mode Disabled.
     * |        |          |1 = Compare Window Mode Enabled.
     * |        |          |Note: This bit is only presented in ADCMPR0 register.
     * |[27:16] |CMPD      |Comparison Data
     * |        |          |The 12-bit data is used to compare with conversion result of specified channel.
     * |        |          |Note: CMPD bits should be filled in unsigned format (straight binary format).
     * @var LPADC_T::ADSR0
     * Offset: 0x90  ADC Status Register0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADF       |A/D Conversion End Flag
     * |        |          |A status flag that indicates the end of A/D conversion. Software can write 1 to clear this bit.
     * |        |          |The ADF bit is set to 1 at the following three conditions:
     * |        |          |1. When A/D conversion ends in Single mode.
     * |        |          |2. When A/D conversion ends on all specified channels in Single-cycle Scan mode and Continuous Scan mode.
     * |        |          |3. When more than or equal to 8 samples in FIFO in Burst mode.
     * |[1]     |CMPF0     |Compare Flag 0
     * |        |          |When the A/D conversion result of the selected channel meets setting condition in ADCMPR0 register then this bit is set to 1
     * |        |          |This bit is cleared by writing 1 to it.
     * |        |          |0 = Conversion result in ADDR does not meet ADCMPR0 setting.
     * |        |          |1 = Conversion result in ADDR meets ADCMPR0 setting.
     * |[2]     |CMPF1     |Compare Flag 1
     * |        |          |When the A/D conversion result of the selected channel meets setting condition in ADCMPR1 register, this bit is set to 1; it is cleared by writing 1 to it
     * |        |          |0 = Conversion result in ADDR does not meet ADCMPR1 setting.
     * |        |          |1 = Conversion result in ADDR meets ADCMPR1 setting.
     * |[7]     |BUSY      |BUSY/IDLE (Read Only)
     * |        |          |This bit is a mirror of ADST(LPADC_ADCR[11]) bit in ADCR register.
     * |        |          |0 = A/D converter is in idle state.
     * |        |          |1 = A/D converter is busy at conversion.
     * |[8]     |VALIDF    |Data Valid Flag (Read Only)
     * |        |          |If any one of VALID (ADDRx[17]) is set, this flag will be set to 1.
     * |        |          |Note: When ADC is in burst mode and any conversion result is valid, this flag will be set to 1.
     * |[16]    |OVERRUNF  |Overrun Flag (Read Only)
     * |        |          |If any one of OVERRUN (ADDRx[16]) is set, this flag will be set to 1.
     * |        |          |Note: When ADC is in burst mode and the FIFO is overrun, this flag will be set to 1.
     * |[24]    |ADPRDY    |ADC Power On Ready (Read Only)
     * |        |          |After set ADEN (LPADC_ADCR[0]) to 1, the ADPRDY will set to 1 after ADC block power on ready.
     * |        |          |Note: The LPADC can start concerting after ADPRDY is 1.
     * |[31:27] |CHANNEL   |Current Conversion Channel (Read Only)
     * |        |          |When BUSY=1, this filed reflects current conversion channel
     * |        |          |When BUSY=0, it shows the number of the next converted channel.
     * @var LPADC_T::ADSR1
     * Offset: 0x94  ADC Status Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VALID     |Data Valid Flag (Read Only)
     * |        |          |VALID[31, 30, 29, 28, 23:0] are the mirror of the VALID bits in ADDR31[17], ADDR30[17], ADDR29[17], ADDR28[17], ADDR23[17]~ ADDR0[17].
     * |        |          |The other bits are reserved.
     * |        |          |Note: When ADC is in burst mode and any conversion result is valid, VALID[31, 30, 29, 28, 23:0] will be set to 1.
     * @var LPADC_T::ADSR2
     * Offset: 0x98  ADC Status Register2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |OVERRUN[31, 30, 29, 28, 23:0] are the mirror of the OVERRUN bit in ADDR31[16], ADDR30[16], ADDR29[16], ADDR28[16], ADDR23[16] ~ ADDR0[16].
     * |        |          |The other bits are reserved.
     * |        |          |Note: When ADC is in burst mode and the FIFO is overrun, OVERRUN[31, 30, 29, 28, 23:0] will be set to 1.
     * @var LPADC_T::ESMPCTL
     * Offset: 0xA0  ADC Extend Sample Time Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[13:0]  |EXTSMPT   |ADC Sampling Time Extend
     * |        |          |When ADC converting at high conversion rate, the sampling time of analog input voltage may not enough if input channel loading is heavy, user can extend ADC sampling time after trigger source is coming to get enough sampling time.
     * |        |          |The range of start delay time is from 0~16383 ADC clock.
     * @var LPADC_T::CFDCTL
     * Offset: 0xA4  ADC Channel Floating Detect Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PRECHEN   |Precharge Enable Bit
     * |        |          |0 = Channel precharge Disabled.
     * |        |          |1 = Channel precharge Enabled.
     * |        |          |Note: Analog input voltage is 1/2 VREF when PRECHEN and DISCHEN are all enable.
     * |[1]     |DISCHEN   |Discharge Enable Bit
     * |        |          |0 = Channel discharge Disabled.
     * |        |          |1 = Channel discharge Enabled.
     * |        |          |Note: Analog input voltage is 1/2 VREF when PRECHEN and DISCHEN are all enable.
     * |[8]     |FDETCHEN  |Floating Detect Channel Enable Bit
     * |        |          |0 = Floating Detect Channel Disabled.
     * |        |          |1 = Floating Detect Channel Enabled.
     * @var LPADC_T::ADPDMA
     * Offset: 0x100  ADC LPPDMA Current Transfer Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[17:0]  |CURDAT    |ADC LPPDMA Current Transfer Data Register (Read Only)
     * |        |          |When LPPDMA transferring, read this register can monitor current LPPDMA transfer data.
     * |        |          |Current LPPDMA transfer data could be the content of ADDR0 ~ ADDR23, ADDR28, ADDR29, ADDR30 and ADDR31 registers.
     * @var LPADC_T::ADCAL
     * Offset: 0x180  ADC Calibration Mode Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CALEN     |Calibration Function Enable Bit
     * |        |          |0 = Calibration function Disabled.
     * |        |          |1.= Calibration function Enabled.
     * |        |          |Note: If chip is powered off, calibration function should be executed again.
     * |[1]     |CALIE     |Calibration Interrupt Enable Bit
     * |        |          |If calibration function is enabled and the calibration is finished, CALIF bit will be asserted, in the meanwhile, if CALIE bit is set to 1, a calibration interrupt request is generated.
     * |        |          |0 = Calibration function Interrupt Disabled.
     * |        |          |1 = Calibration function Interrupt Enabled.
     * @var LPADC_T::ADCALSTS
     * Offset: 0x184  ADC Calibration Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CALIF     |Calibration Finish Interrupt Flag
     * |        |          |If calibration is finished, this flag will be set to 1. It is cleared by writing 1 to it.
     * @var LPADC_T::AUTOCTL
     * Offset: 0x800  ADC Auto Operation Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |TRIGSEL   |Automatic Operation Trigger Source Select
     * |        |          |0000 = Auto-operation Trigger Source from LPTMR0.
     * |        |          |0001 = Auto-operation Trigger Source from LPTMR1.
     * |        |          |0010 = Auto-operation Trigger Source from TTMR0.
     * |        |          |0011 = Auto-operation Trigger Source from TTMR1.
     * |        |          |0100 = Auto-operation Trigger Source from WKIOA0.
     * |        |          |0101 = Auto-operation Trigger Source from WKIOB0.
     * |        |          |0110 = Auto-operation Trigger Source from WKIOC0.
     * |        |          |0111 = Auto-operation Trigger Source from WKIOD0.
     * |        |          |1000 = Auto-operation Trigger Source from ACMP0.
     * |        |          |1001 = Auto-operation Trigger Source from ACMP1.
     * |        |          |1010 = Auto-operation Trigger Source from ACMP2.
     * |        |          |1011 = Auto-operation Trigger Source from ACMP3.
     * |        |          |Others = Reserved.
     * |[4]     |TRIGEN    |Automatic Operation Trigger Enable Bit
     * |        |          |When automatic operation mode is enabled, the LPADC will start working if LPADC is triggered by a event sent from the trigger source selected by TRIGSEL[3:0] after this bit is set to 1.
     * |        |          |0 = LPADC Automatic Operation Trigger disabled.
     * |        |          |1 = LPADC Automatic Operation Trigger enabled.
     * |[8]     |ADWKEN    |Automatic Operation Mode Conversion End Wake-up Enable Bit
     * |        |          |0 = LPADC automatic operation conversion end wake-up Disabled.
     * |        |          |1 = LPADC automatic operation conversion end wake-up Enabled.
     * |[9]     |CMP0WKEN  |Automatic Operation Mode Comparator 0 Wake-up Enable Bit
     * |        |          |0 = LPADC automatic operation comparator 0 wake-up Disabled.
     * |        |          |1 = LPADC automatic operation comparator 0 wake-up Enabled.
     * |[10]    |CMP1WKEN  |Automatic Operation Mode Comparator 1 Wake-up Enable Bit
     * |        |          |0 = LPADC automatic operation comparator 1 wake-up Disabled.
     * |        |          |1 = LPADC automatic operation comparator 1 wake-up Enabled.
     * |[31]    |AUTOEN    |Automatic Operation Mode Enable Bit
     * |        |          |0 = LPADC automatic operation Disabled.
     * |        |          |1 = LPADC automatic operation Enabled.
     * @var LPADC_T::AUTOSTRG
     * Offset: 0x804  ADC Auto Operation Software Trigger Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SWTRIG    |Automatic Operation Software Trigger Bit
     * |        |          |Write 1 to this bit will trigger LPADC start automatic operation
     * |        |          |Note: This bit will be auto cleared by hardware
     * @var LPADC_T::AUTOSTS
     * Offset: 0x810  ADC Auto Operation Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADWKF     |Automatic Operation Conversion End Wake-up Flag Bit
     * |        |          |When automatic operation mode is enabled, and the conversion is finished, this flag will be set
     * |        |          |If WKEN also set to 1, the chip will be wake-up.
     * |        |          |Software can write one to clear this bit
     * |[1]     |CMP0WKF   |Automatic Operation Compare 0 Wake-up Flag Bit
     * |        |          |When automatic operation mode is enabled, and the conversion result monitor comparator 0 is match setting criteria, this flag will be set
     * |        |          |If CMP0WKEN also set to 1, the chip will be wake-up.
     * |        |          |Software can write one to clear this bit
     * |[2]     |CMP1WKF   |Automatic Operation Compare 1 Wake-up Flag Bit
     * |        |          |When automatic operation mode is enabled, and the conversion result monitor comparator 1 is match setting criteria, this flag will be set
     * |        |          |If CMP1WKEN also set to 1, the chip will be wake-up.
     * |        |          |Software can write one to clear this bit
     */
    __I  uint32_t ADDR[32];              /*!< [0x0000-0x007C] ADC Data Register 0 ~ 31                                  */
    __IO uint32_t ADCR;                  /*!< [0x0080] ADC Control Register                                             */
    __IO uint32_t ADCHER;                /*!< [0x0084] ADC Channel Enable Register                                      */
    __IO uint32_t ADCMPR[2];             /*!< [0x0088-0x008C] ADC Compare Register 0~1                                  */
    __IO uint32_t ADSR0;                 /*!< [0x0090] ADC Status Register0                                             */
    __I  uint32_t ADSR1;                 /*!< [0x0094] ADC Status Register1                                             */
    __I  uint32_t ADSR2;                 /*!< [0x0098] ADC Status Register2                                             */
    __I  uint32_t RESERVE1[1];
    __IO uint32_t ESMPCTL;               /*!< [0x00a0] ADC Extend Sample Time Control Register                          */
    __IO uint32_t CFDCTL;                /*!< [0x00a4] ADC Channel Floating Detect Control Register                     */
    __I  uint32_t RESERVE2[22];
    __I  uint32_t ADPDMA;                /*!< [0x0100] ADC LPPDMA Current Transfer Data Register                        */
    __I  uint32_t RESERVE3[31];
    __IO uint32_t ADCAL;                 /*!< [0x0180] ADC Calibration Mode Register                                    */
    __IO uint32_t ADCALSTS;              /*!< [0x0184] ADC Calibration Status Register                                  */
    __I  uint32_t RESERVE4[414];
    __IO uint32_t AUTOCTL;               /*!< [0x0800] ADC Auto Operation Control Register                              */
    __O  uint32_t AUTOSTRG;              /*!< [0x0804] ADC Auto Operation Software Trigger Register                     */
    __I  uint32_t RESERVE5[2];
    __IO uint32_t AUTOSTS;               /*!< [0x0810] ADC Auto Operation Status Register                               */

} LPADC_T;

/**
    @addtogroup LPADC_CONST LPADC Bit Field Definition
    Constant Definitions for LPADC Controller
@{ */


#define LPADC_ADDR_RSLT_Pos              (0)                                               /*!< LPADC_T::ADDR: RSLT Position           */
#define LPADC_ADDR_RSLT_Msk              (0xfffful << LPADC_ADDR_RSLT_Pos)                 /*!< LPADC_T::ADDR: RSLT Mask               */

#define LPADC_ADDR_OVERRUN_Pos           (16)                                              /*!< LPADC_T::ADDR: OVERRUN Position        */
#define LPADC_ADDR_OVERRUN_Msk           (0x1ul << LPADC_ADDR_OVERRUN_Pos)                 /*!< LPADC_T::ADDR: OVERRUN Mask            */

#define LPADC_ADDR_VALID_Pos             (17)                                              /*!< LPADC_T::ADDR: VALID Position          */
#define LPADC_ADDR_VALID_Msk             (0x1ul << LPADC_ADDR_VALID_Pos)                   /*!< LPADC_T::ADDR: VALID Mask              */

#define LPADC_ADDR0_RSLT_Pos             (0)                                               /*!< LPADC_T::ADDR0: RSLT Position          */
#define LPADC_ADDR0_RSLT_Msk             (0xfffful << LPADC_ADDR0_RSLT_Pos)                /*!< LPADC_T::ADDR0: RSLT Mask              */

#define LPADC_ADDR0_OVERRUN_Pos          (16)                                              /*!< LPADC_T::ADDR0: OVERRUN Position       */
#define LPADC_ADDR0_OVERRUN_Msk          (0x1ul << LPADC_ADDR0_OVERRUN_Pos)                /*!< LPADC_T::ADDR0: OVERRUN Mask           */

#define LPADC_ADDR0_VALID_Pos            (17)                                              /*!< LPADC_T::ADDR0: VALID Position         */
#define LPADC_ADDR0_VALID_Msk            (0x1ul << LPADC_ADDR0_VALID_Pos)                  /*!< LPADC_T::ADDR0: VALID Mask             */

#define LPADC_ADDR1_RSLT_Pos             (0)                                               /*!< LPADC_T::ADDR1: RSLT Position          */
#define LPADC_ADDR1_RSLT_Msk             (0xfffful << LPADC_ADDR1_RSLT_Pos)                /*!< LPADC_T::ADDR1: RSLT Mask              */

#define LPADC_ADDR1_OVERRUN_Pos          (16)                                              /*!< LPADC_T::ADDR1: OVERRUN Position       */
#define LPADC_ADDR1_OVERRUN_Msk          (0x1ul << LPADC_ADDR1_OVERRUN_Pos)                /*!< LPADC_T::ADDR1: OVERRUN Mask           */

#define LPADC_ADDR1_VALID_Pos            (17)                                              /*!< LPADC_T::ADDR1: VALID Position         */
#define LPADC_ADDR1_VALID_Msk            (0x1ul << LPADC_ADDR1_VALID_Pos)                  /*!< LPADC_T::ADDR1: VALID Mask             */

#define LPADC_ADDR2_RSLT_Pos             (0)                                               /*!< LPADC_T::ADDR2: RSLT Position          */
#define LPADC_ADDR2_RSLT_Msk             (0xfffful << LPADC_ADDR2_RSLT_Pos)                /*!< LPADC_T::ADDR2: RSLT Mask              */

#define LPADC_ADDR2_OVERRUN_Pos          (16)                                              /*!< LPADC_T::ADDR2: OVERRUN Position       */
#define LPADC_ADDR2_OVERRUN_Msk          (0x1ul << LPADC_ADDR2_OVERRUN_Pos)                /*!< LPADC_T::ADDR2: OVERRUN Mask           */

#define LPADC_ADDR2_VALID_Pos            (17)                                              /*!< LPADC_T::ADDR2: VALID Position         */
#define LPADC_ADDR2_VALID_Msk            (0x1ul << LPADC_ADDR2_VALID_Pos)                  /*!< LPADC_T::ADDR2: VALID Mask             */

#define LPADC_ADDR3_RSLT_Pos             (0)                                               /*!< LPADC_T::ADDR3: RSLT Position          */
#define LPADC_ADDR3_RSLT_Msk             (0xfffful << LPADC_ADDR3_RSLT_Pos)                /*!< LPADC_T::ADDR3: RSLT Mask              */

#define LPADC_ADDR3_OVERRUN_Pos          (16)                                              /*!< LPADC_T::ADDR3: OVERRUN Position       */
#define LPADC_ADDR3_OVERRUN_Msk          (0x1ul << LPADC_ADDR3_OVERRUN_Pos)                /*!< LPADC_T::ADDR3: OVERRUN Mask           */

#define LPADC_ADDR3_VALID_Pos            (17)                                              /*!< LPADC_T::ADDR3: VALID Position         */
#define LPADC_ADDR3_VALID_Msk            (0x1ul << LPADC_ADDR3_VALID_Pos)                  /*!< LPADC_T::ADDR3: VALID Mask             */

#define LPADC_ADDR4_RSLT_Pos             (0)                                               /*!< LPADC_T::ADDR4: RSLT Position          */
#define LPADC_ADDR4_RSLT_Msk             (0xfffful << LPADC_ADDR4_RSLT_Pos)                /*!< LPADC_T::ADDR4: RSLT Mask              */

#define LPADC_ADDR4_OVERRUN_Pos          (16)                                              /*!< LPADC_T::ADDR4: OVERRUN Position       */
#define LPADC_ADDR4_OVERRUN_Msk          (0x1ul << LPADC_ADDR4_OVERRUN_Pos)                /*!< LPADC_T::ADDR4: OVERRUN Mask           */

#define LPADC_ADDR4_VALID_Pos            (17)                                              /*!< LPADC_T::ADDR4: VALID Position         */
#define LPADC_ADDR4_VALID_Msk            (0x1ul << LPADC_ADDR4_VALID_Pos)                  /*!< LPADC_T::ADDR4: VALID Mask             */

#define LPADC_ADDR5_RSLT_Pos             (0)                                               /*!< LPADC_T::ADDR5: RSLT Position          */
#define LPADC_ADDR5_RSLT_Msk             (0xfffful << LPADC_ADDR5_RSLT_Pos)                /*!< LPADC_T::ADDR5: RSLT Mask              */

#define LPADC_ADDR5_OVERRUN_Pos          (16)                                              /*!< LPADC_T::ADDR5: OVERRUN Position       */
#define LPADC_ADDR5_OVERRUN_Msk          (0x1ul << LPADC_ADDR5_OVERRUN_Pos)                /*!< LPADC_T::ADDR5: OVERRUN Mask           */

#define LPADC_ADDR5_VALID_Pos            (17)                                              /*!< LPADC_T::ADDR5: VALID Position         */
#define LPADC_ADDR5_VALID_Msk            (0x1ul << LPADC_ADDR5_VALID_Pos)                  /*!< LPADC_T::ADDR5: VALID Mask             */

#define LPADC_ADDR6_RSLT_Pos             (0)                                               /*!< LPADC_T::ADDR6: RSLT Position          */
#define LPADC_ADDR6_RSLT_Msk             (0xfffful << LPADC_ADDR6_RSLT_Pos)                /*!< LPADC_T::ADDR6: RSLT Mask              */

#define LPADC_ADDR6_OVERRUN_Pos          (16)                                              /*!< LPADC_T::ADDR6: OVERRUN Position       */
#define LPADC_ADDR6_OVERRUN_Msk          (0x1ul << LPADC_ADDR6_OVERRUN_Pos)                /*!< LPADC_T::ADDR6: OVERRUN Mask           */

#define LPADC_ADDR6_VALID_Pos            (17)                                              /*!< LPADC_T::ADDR6: VALID Position         */
#define LPADC_ADDR6_VALID_Msk            (0x1ul << LPADC_ADDR6_VALID_Pos)                  /*!< LPADC_T::ADDR6: VALID Mask             */

#define LPADC_ADDR7_RSLT_Pos             (0)                                               /*!< LPADC_T::ADDR7: RSLT Position          */
#define LPADC_ADDR7_RSLT_Msk             (0xfffful << LPADC_ADDR7_RSLT_Pos)                /*!< LPADC_T::ADDR7: RSLT Mask              */

#define LPADC_ADDR7_OVERRUN_Pos          (16)                                              /*!< LPADC_T::ADDR7: OVERRUN Position       */
#define LPADC_ADDR7_OVERRUN_Msk          (0x1ul << LPADC_ADDR7_OVERRUN_Pos)                /*!< LPADC_T::ADDR7: OVERRUN Mask           */

#define LPADC_ADDR7_VALID_Pos            (17)                                              /*!< LPADC_T::ADDR7: VALID Position         */
#define LPADC_ADDR7_VALID_Msk            (0x1ul << LPADC_ADDR7_VALID_Pos)                  /*!< LPADC_T::ADDR7: VALID Mask             */

#define LPADC_ADDR8_RSLT_Pos             (0)                                               /*!< LPADC_T::ADDR8: RSLT Position          */
#define LPADC_ADDR8_RSLT_Msk             (0xfffful << LPADC_ADDR8_RSLT_Pos)                /*!< LPADC_T::ADDR8: RSLT Mask              */

#define LPADC_ADDR8_OVERRUN_Pos          (16)                                              /*!< LPADC_T::ADDR8: OVERRUN Position       */
#define LPADC_ADDR8_OVERRUN_Msk          (0x1ul << LPADC_ADDR8_OVERRUN_Pos)                /*!< LPADC_T::ADDR8: OVERRUN Mask           */

#define LPADC_ADDR8_VALID_Pos            (17)                                              /*!< LPADC_T::ADDR8: VALID Position         */
#define LPADC_ADDR8_VALID_Msk            (0x1ul << LPADC_ADDR8_VALID_Pos)                  /*!< LPADC_T::ADDR8: VALID Mask             */

#define LPADC_ADDR9_RSLT_Pos             (0)                                               /*!< LPADC_T::ADDR9: RSLT Position          */
#define LPADC_ADDR9_RSLT_Msk             (0xfffful << LPADC_ADDR9_RSLT_Pos)                /*!< LPADC_T::ADDR9: RSLT Mask              */

#define LPADC_ADDR9_OVERRUN_Pos          (16)                                              /*!< LPADC_T::ADDR9: OVERRUN Position       */
#define LPADC_ADDR9_OVERRUN_Msk          (0x1ul << LPADC_ADDR9_OVERRUN_Pos)                /*!< LPADC_T::ADDR9: OVERRUN Mask           */

#define LPADC_ADDR9_VALID_Pos            (17)                                              /*!< LPADC_T::ADDR9: VALID Position         */
#define LPADC_ADDR9_VALID_Msk            (0x1ul << LPADC_ADDR9_VALID_Pos)                  /*!< LPADC_T::ADDR9: VALID Mask             */

#define LPADC_ADDR10_RSLT_Pos            (0)                                               /*!< LPADC_T::ADDR10: RSLT Position         */
#define LPADC_ADDR10_RSLT_Msk            (0xfffful << LPADC_ADDR10_RSLT_Pos)               /*!< LPADC_T::ADDR10: RSLT Mask             */

#define LPADC_ADDR10_OVERRUN_Pos         (16)                                              /*!< LPADC_T::ADDR10: OVERRUN Position      */
#define LPADC_ADDR10_OVERRUN_Msk         (0x1ul << LPADC_ADDR10_OVERRUN_Pos)               /*!< LPADC_T::ADDR10: OVERRUN Mask          */

#define LPADC_ADDR10_VALID_Pos           (17)                                              /*!< LPADC_T::ADDR10: VALID Position        */
#define LPADC_ADDR10_VALID_Msk           (0x1ul << LPADC_ADDR10_VALID_Pos)                 /*!< LPADC_T::ADDR10: VALID Mask            */

#define LPADC_ADDR11_RSLT_Pos            (0)                                               /*!< LPADC_T::ADDR11: RSLT Position         */
#define LPADC_ADDR11_RSLT_Msk            (0xfffful << LPADC_ADDR11_RSLT_Pos)               /*!< LPADC_T::ADDR11: RSLT Mask             */

#define LPADC_ADDR11_OVERRUN_Pos         (16)                                              /*!< LPADC_T::ADDR11: OVERRUN Position      */
#define LPADC_ADDR11_OVERRUN_Msk         (0x1ul << LPADC_ADDR11_OVERRUN_Pos)               /*!< LPADC_T::ADDR11: OVERRUN Mask          */

#define LPADC_ADDR11_VALID_Pos           (17)                                              /*!< LPADC_T::ADDR11: VALID Position        */
#define LPADC_ADDR11_VALID_Msk           (0x1ul << LPADC_ADDR11_VALID_Pos)                 /*!< LPADC_T::ADDR11: VALID Mask            */

#define LPADC_ADDR12_RSLT_Pos            (0)                                               /*!< LPADC_T::ADDR12: RSLT Position         */
#define LPADC_ADDR12_RSLT_Msk            (0xfffful << LPADC_ADDR12_RSLT_Pos)               /*!< LPADC_T::ADDR12: RSLT Mask             */

#define LPADC_ADDR12_OVERRUN_Pos         (16)                                              /*!< LPADC_T::ADDR12: OVERRUN Position      */
#define LPADC_ADDR12_OVERRUN_Msk         (0x1ul << LPADC_ADDR12_OVERRUN_Pos)               /*!< LPADC_T::ADDR12: OVERRUN Mask          */

#define LPADC_ADDR12_VALID_Pos           (17)                                              /*!< LPADC_T::ADDR12: VALID Position        */
#define LPADC_ADDR12_VALID_Msk           (0x1ul << LPADC_ADDR12_VALID_Pos)                 /*!< LPADC_T::ADDR12: VALID Mask            */

#define LPADC_ADDR13_RSLT_Pos            (0)                                               /*!< LPADC_T::ADDR13: RSLT Position         */
#define LPADC_ADDR13_RSLT_Msk            (0xfffful << LPADC_ADDR13_RSLT_Pos)               /*!< LPADC_T::ADDR13: RSLT Mask             */

#define LPADC_ADDR13_OVERRUN_Pos         (16)                                              /*!< LPADC_T::ADDR13: OVERRUN Position      */
#define LPADC_ADDR13_OVERRUN_Msk         (0x1ul << LPADC_ADDR13_OVERRUN_Pos)               /*!< LPADC_T::ADDR13: OVERRUN Mask          */

#define LPADC_ADDR13_VALID_Pos           (17)                                              /*!< LPADC_T::ADDR13: VALID Position        */
#define LPADC_ADDR13_VALID_Msk           (0x1ul << LPADC_ADDR13_VALID_Pos)                 /*!< LPADC_T::ADDR13: VALID Mask            */

#define LPADC_ADDR14_RSLT_Pos            (0)                                               /*!< LPADC_T::ADDR14: RSLT Position         */
#define LPADC_ADDR14_RSLT_Msk            (0xfffful << LPADC_ADDR14_RSLT_Pos)               /*!< LPADC_T::ADDR14: RSLT Mask             */

#define LPADC_ADDR14_OVERRUN_Pos         (16)                                              /*!< LPADC_T::ADDR14: OVERRUN Position      */
#define LPADC_ADDR14_OVERRUN_Msk         (0x1ul << LPADC_ADDR14_OVERRUN_Pos)               /*!< LPADC_T::ADDR14: OVERRUN Mask          */

#define LPADC_ADDR14_VALID_Pos           (17)                                              /*!< LPADC_T::ADDR14: VALID Position        */
#define LPADC_ADDR14_VALID_Msk           (0x1ul << LPADC_ADDR14_VALID_Pos)                 /*!< LPADC_T::ADDR14: VALID Mask            */

#define LPADC_ADDR15_RSLT_Pos            (0)                                               /*!< LPADC_T::ADDR15: RSLT Position         */
#define LPADC_ADDR15_RSLT_Msk            (0xfffful << LPADC_ADDR15_RSLT_Pos)               /*!< LPADC_T::ADDR15: RSLT Mask             */

#define LPADC_ADDR15_OVERRUN_Pos         (16)                                              /*!< LPADC_T::ADDR15: OVERRUN Position      */
#define LPADC_ADDR15_OVERRUN_Msk         (0x1ul << LPADC_ADDR15_OVERRUN_Pos)               /*!< LPADC_T::ADDR15: OVERRUN Mask          */

#define LPADC_ADDR15_VALID_Pos           (17)                                              /*!< LPADC_T::ADDR15: VALID Position        */
#define LPADC_ADDR15_VALID_Msk           (0x1ul << LPADC_ADDR15_VALID_Pos)                 /*!< LPADC_T::ADDR15: VALID Mask            */

#define LPADC_ADDR16_RSLT_Pos            (0)                                               /*!< LPADC_T::ADDR16: RSLT Position         */
#define LPADC_ADDR16_RSLT_Msk            (0xfffful << LPADC_ADDR16_RSLT_Pos)               /*!< LPADC_T::ADDR16: RSLT Mask             */

#define LPADC_ADDR16_OVERRUN_Pos         (16)                                              /*!< LPADC_T::ADDR16: OVERRUN Position      */
#define LPADC_ADDR16_OVERRUN_Msk         (0x1ul << LPADC_ADDR16_OVERRUN_Pos)               /*!< LPADC_T::ADDR16: OVERRUN Mask          */

#define LPADC_ADDR16_VALID_Pos           (17)                                              /*!< LPADC_T::ADDR16: VALID Position        */
#define LPADC_ADDR16_VALID_Msk           (0x1ul << LPADC_ADDR16_VALID_Pos)                 /*!< LPADC_T::ADDR16: VALID Mask            */

#define LPADC_ADDR17_RSLT_Pos            (0)                                               /*!< LPADC_T::ADDR17: RSLT Position         */
#define LPADC_ADDR17_RSLT_Msk            (0xfffful << LPADC_ADDR17_RSLT_Pos)               /*!< LPADC_T::ADDR17: RSLT Mask             */

#define LPADC_ADDR17_OVERRUN_Pos         (16)                                              /*!< LPADC_T::ADDR17: OVERRUN Position      */
#define LPADC_ADDR17_OVERRUN_Msk         (0x1ul << LPADC_ADDR17_OVERRUN_Pos)               /*!< LPADC_T::ADDR17: OVERRUN Mask          */

#define LPADC_ADDR17_VALID_Pos           (17)                                              /*!< LPADC_T::ADDR17: VALID Position        */
#define LPADC_ADDR17_VALID_Msk           (0x1ul << LPADC_ADDR17_VALID_Pos)                 /*!< LPADC_T::ADDR17: VALID Mask            */

#define LPADC_ADDR18_RSLT_Pos            (0)                                               /*!< LPADC_T::ADDR18: RSLT Position         */
#define LPADC_ADDR18_RSLT_Msk            (0xfffful << LPADC_ADDR18_RSLT_Pos)               /*!< LPADC_T::ADDR18: RSLT Mask             */

#define LPADC_ADDR18_OVERRUN_Pos         (16)                                              /*!< LPADC_T::ADDR18: OVERRUN Position      */
#define LPADC_ADDR18_OVERRUN_Msk         (0x1ul << LPADC_ADDR18_OVERRUN_Pos)               /*!< LPADC_T::ADDR18: OVERRUN Mask          */

#define LPADC_ADDR18_VALID_Pos           (17)                                              /*!< LPADC_T::ADDR18: VALID Position        */
#define LPADC_ADDR18_VALID_Msk           (0x1ul << LPADC_ADDR18_VALID_Pos)                 /*!< LPADC_T::ADDR18: VALID Mask            */

#define LPADC_ADDR19_RSLT_Pos            (0)                                               /*!< LPADC_T::ADDR19: RSLT Position         */
#define LPADC_ADDR19_RSLT_Msk            (0xfffful << LPADC_ADDR19_RSLT_Pos)               /*!< LPADC_T::ADDR19: RSLT Mask             */

#define LPADC_ADDR19_OVERRUN_Pos         (16)                                              /*!< LPADC_T::ADDR19: OVERRUN Position      */
#define LPADC_ADDR19_OVERRUN_Msk         (0x1ul << LPADC_ADDR19_OVERRUN_Pos)               /*!< LPADC_T::ADDR19: OVERRUN Mask          */

#define LPADC_ADDR19_VALID_Pos           (17)                                              /*!< LPADC_T::ADDR19: VALID Position        */
#define LPADC_ADDR19_VALID_Msk           (0x1ul << LPADC_ADDR19_VALID_Pos)                 /*!< LPADC_T::ADDR19: VALID Mask            */

#define LPADC_ADDR20_RSLT_Pos            (0)                                               /*!< LPADC_T::ADDR20: RSLT Position         */
#define LPADC_ADDR20_RSLT_Msk            (0xfffful << LPADC_ADDR20_RSLT_Pos)               /*!< LPADC_T::ADDR20: RSLT Mask             */

#define LPADC_ADDR20_OVERRUN_Pos         (16)                                              /*!< LPADC_T::ADDR20: OVERRUN Position      */
#define LPADC_ADDR20_OVERRUN_Msk         (0x1ul << LPADC_ADDR20_OVERRUN_Pos)               /*!< LPADC_T::ADDR20: OVERRUN Mask          */

#define LPADC_ADDR20_VALID_Pos           (17)                                              /*!< LPADC_T::ADDR20: VALID Position        */
#define LPADC_ADDR20_VALID_Msk           (0x1ul << LPADC_ADDR20_VALID_Pos)                 /*!< LPADC_T::ADDR20: VALID Mask            */

#define LPADC_ADDR21_RSLT_Pos            (0)                                               /*!< LPADC_T::ADDR21: RSLT Position         */
#define LPADC_ADDR21_RSLT_Msk            (0xfffful << LPADC_ADDR21_RSLT_Pos)               /*!< LPADC_T::ADDR21: RSLT Mask             */

#define LPADC_ADDR21_OVERRUN_Pos         (16)                                              /*!< LPADC_T::ADDR21: OVERRUN Position      */
#define LPADC_ADDR21_OVERRUN_Msk         (0x1ul << LPADC_ADDR21_OVERRUN_Pos)               /*!< LPADC_T::ADDR21: OVERRUN Mask          */

#define LPADC_ADDR21_VALID_Pos           (17)                                              /*!< LPADC_T::ADDR21: VALID Position        */
#define LPADC_ADDR21_VALID_Msk           (0x1ul << LPADC_ADDR21_VALID_Pos)                 /*!< LPADC_T::ADDR21: VALID Mask            */

#define LPADC_ADDR22_RSLT_Pos            (0)                                               /*!< LPADC_T::ADDR22: RSLT Position         */
#define LPADC_ADDR22_RSLT_Msk            (0xfffful << LPADC_ADDR22_RSLT_Pos)               /*!< LPADC_T::ADDR22: RSLT Mask             */

#define LPADC_ADDR22_OVERRUN_Pos         (16)                                              /*!< LPADC_T::ADDR22: OVERRUN Position      */
#define LPADC_ADDR22_OVERRUN_Msk         (0x1ul << LPADC_ADDR22_OVERRUN_Pos)               /*!< LPADC_T::ADDR22: OVERRUN Mask          */

#define LPADC_ADDR22_VALID_Pos           (17)                                              /*!< LPADC_T::ADDR22: VALID Position        */
#define LPADC_ADDR22_VALID_Msk           (0x1ul << LPADC_ADDR22_VALID_Pos)                 /*!< LPADC_T::ADDR22: VALID Mask            */

#define LPADC_ADDR23_RSLT_Pos            (0)                                               /*!< LPADC_T::ADDR23: RSLT Position         */
#define LPADC_ADDR23_RSLT_Msk            (0xfffful << LPADC_ADDR23_RSLT_Pos)               /*!< LPADC_T::ADDR23: RSLT Mask             */

#define LPADC_ADDR23_OVERRUN_Pos         (16)                                              /*!< LPADC_T::ADDR23: OVERRUN Position      */
#define LPADC_ADDR23_OVERRUN_Msk         (0x1ul << LPADC_ADDR23_OVERRUN_Pos)               /*!< LPADC_T::ADDR23: OVERRUN Mask          */

#define LPADC_ADDR23_VALID_Pos           (17)                                              /*!< LPADC_T::ADDR23: VALID Position        */
#define LPADC_ADDR23_VALID_Msk           (0x1ul << LPADC_ADDR23_VALID_Pos)                 /*!< LPADC_T::ADDR23: VALID Mask            */

#define LPADC_ADDR28_RSLT_Pos            (0)                                               /*!< LPADC_T::ADDR28: RSLT Position         */
#define LPADC_ADDR28_RSLT_Msk            (0xfffful << LPADC_ADDR28_RSLT_Pos)               /*!< LPADC_T::ADDR28: RSLT Mask             */

#define LPADC_ADDR28_OVERRUN_Pos         (16)                                              /*!< LPADC_T::ADDR28: OVERRUN Position      */
#define LPADC_ADDR28_OVERRUN_Msk         (0x1ul << LPADC_ADDR28_OVERRUN_Pos)               /*!< LPADC_T::ADDR28: OVERRUN Mask          */

#define LPADC_ADDR28_VALID_Pos           (17)                                              /*!< LPADC_T::ADDR28: VALID Position        */
#define LPADC_ADDR28_VALID_Msk           (0x1ul << LPADC_ADDR28_VALID_Pos)                 /*!< LPADC_T::ADDR28: VALID Mask            */

#define LPADC_ADDR29_RSLT_Pos            (0)                                               /*!< LPADC_T::ADDR29: RSLT Position         */
#define LPADC_ADDR29_RSLT_Msk            (0xfffful << LPADC_ADDR29_RSLT_Pos)               /*!< LPADC_T::ADDR29: RSLT Mask             */

#define LPADC_ADDR29_OVERRUN_Pos         (16)                                              /*!< LPADC_T::ADDR29: OVERRUN Position      */
#define LPADC_ADDR29_OVERRUN_Msk         (0x1ul << LPADC_ADDR29_OVERRUN_Pos)               /*!< LPADC_T::ADDR29: OVERRUN Mask          */

#define LPADC_ADDR29_VALID_Pos           (17)                                              /*!< LPADC_T::ADDR29: VALID Position        */
#define LPADC_ADDR29_VALID_Msk           (0x1ul << LPADC_ADDR29_VALID_Pos)                 /*!< LPADC_T::ADDR29: VALID Mask            */

#define LPADC_ADDR30_RSLT_Pos            (0)                                               /*!< LPADC_T::ADDR30: RSLT Position         */
#define LPADC_ADDR30_RSLT_Msk            (0xfffful << LPADC_ADDR30_RSLT_Pos)               /*!< LPADC_T::ADDR30: RSLT Mask             */

#define LPADC_ADDR30_OVERRUN_Pos         (16)                                              /*!< LPADC_T::ADDR30: OVERRUN Position      */
#define LPADC_ADDR30_OVERRUN_Msk         (0x1ul << LPADC_ADDR30_OVERRUN_Pos)               /*!< LPADC_T::ADDR30: OVERRUN Mask          */

#define LPADC_ADDR30_VALID_Pos           (17)                                              /*!< LPADC_T::ADDR30: VALID Position        */
#define LPADC_ADDR30_VALID_Msk           (0x1ul << LPADC_ADDR30_VALID_Pos)                 /*!< LPADC_T::ADDR30: VALID Mask            */

#define LPADC_ADDR31_RSLT_Pos            (0)                                               /*!< LPADC_T::ADDR31: RSLT Position         */
#define LPADC_ADDR31_RSLT_Msk            (0xfffful << LPADC_ADDR31_RSLT_Pos)               /*!< LPADC_T::ADDR31: RSLT Mask             */

#define LPADC_ADDR31_OVERRUN_Pos         (16)                                              /*!< LPADC_T::ADDR31: OVERRUN Position      */
#define LPADC_ADDR31_OVERRUN_Msk         (0x1ul << LPADC_ADDR31_OVERRUN_Pos)               /*!< LPADC_T::ADDR31: OVERRUN Mask          */

#define LPADC_ADDR31_VALID_Pos           (17)                                              /*!< LPADC_T::ADDR31: VALID Position        */
#define LPADC_ADDR31_VALID_Msk           (0x1ul << LPADC_ADDR31_VALID_Pos)                 /*!< LPADC_T::ADDR31: VALID Mask            */

#define LPADC_ADCR_ADEN_Pos              (0)                                               /*!< LPADC_T::ADCR: ADEN Position           */
#define LPADC_ADCR_ADEN_Msk              (0x1ul << LPADC_ADCR_ADEN_Pos)                    /*!< LPADC_T::ADCR: ADEN Mask               */

#define LPADC_ADCR_ADIE_Pos              (1)                                               /*!< LPADC_T::ADCR: ADIE Position           */
#define LPADC_ADCR_ADIE_Msk              (0x1ul << LPADC_ADCR_ADIE_Pos)                    /*!< LPADC_T::ADCR: ADIE Mask               */

#define LPADC_ADCR_ADMD_Pos              (2)                                               /*!< LPADC_T::ADCR: ADMD Position           */
#define LPADC_ADCR_ADMD_Msk              (0x3ul << LPADC_ADCR_ADMD_Pos)                    /*!< LPADC_T::ADCR: ADMD Mask               */

#define LPADC_ADCR_TRGCOND_Pos           (6)                                               /*!< LPADC_T::ADCR: TRGCOND Position        */
#define LPADC_ADCR_TRGCOND_Msk           (0x3ul << LPADC_ADCR_TRGCOND_Pos)                 /*!< LPADC_T::ADCR: TRGCOND Mask            */

#define LPADC_ADCR_TRGEN_Pos             (8)                                               /*!< LPADC_T::ADCR: TRGEN Position          */
#define LPADC_ADCR_TRGEN_Msk             (0x1ul << LPADC_ADCR_TRGEN_Pos)                   /*!< LPADC_T::ADCR: TRGEN Mask              */

#define LPADC_ADCR_PTEN_Pos              (9)                                               /*!< LPADC_T::ADCR: PTEN Position           */
#define LPADC_ADCR_PTEN_Msk              (0x1ul << LPADC_ADCR_PTEN_Pos)                    /*!< LPADC_T::ADCR: PTEN Mask               */

#define LPADC_ADCR_DIFFEN_Pos            (10)                                              /*!< LPADC_T::ADCR: DIFFEN Position         */
#define LPADC_ADCR_DIFFEN_Msk            (0x1ul << LPADC_ADCR_DIFFEN_Pos)                  /*!< LPADC_T::ADCR: DIFFEN Mask             */

#define LPADC_ADCR_ADST_Pos              (11)                                              /*!< LPADC_T::ADCR: ADST Position           */
#define LPADC_ADCR_ADST_Msk              (0x1ul << LPADC_ADCR_ADST_Pos)                    /*!< LPADC_T::ADCR: ADST Mask               */

#define LPADC_ADCR_RESET_Pos             (12)                                              /*!< LPADC_T::ADCR: RESET Position          */
#define LPADC_ADCR_RESET_Msk             (0x1ul << LPADC_ADCR_RESET_Pos)                   /*!< LPADC_T::ADCR: RESET Mask              */

#define LPADC_ADCR_TRGS_Pos              (16)                                              /*!< LPADC_T::ADCR: TRGS Position           */
#define LPADC_ADCR_TRGS_Msk              (0xful << LPADC_ADCR_TRGS_Pos)                    /*!< LPADC_T::ADCR: TRGS Mask               */

#define LPADC_ADCR_RESSEL_Pos            (28)                                              /*!< LPADC_T::ADCR: RESSEL Position         */
#define LPADC_ADCR_RESSEL_Msk            (0x3ul << LPADC_ADCR_RESSEL_Pos)                  /*!< LPADC_T::ADCR: RESSEL Mask             */

#define LPADC_ADCR_DMOF_Pos              (31)                                              /*!< LPADC_T::ADCR: DMOF Position           */
#define LPADC_ADCR_DMOF_Msk              (0x1ul << LPADC_ADCR_DMOF_Pos)                    /*!< LPADC_T::ADCR: DMOF Mask               */

#define LPADC_ADCHER_CHEN_Pos            (0)                                               /*!< LPADC_T::ADCHER: CHEN Position         */
#define LPADC_ADCHER_CHEN_Msk            (0xfffffffful << LPADC_ADCHER_CHEN_Pos)           /*!< LPADC_T::ADCHER: CHEN Mask             */

#define LPADC_ADCMPR_CMPEN_Pos           (0)                                               /*!< LPADC_T::ADCMPR: CMPEN Position        */
#define LPADC_ADCMPR_CMPEN_Msk           (0x1ul << LPADC_ADCMPR_CMPEN_Pos)                 /*!< LPADC_T::ADCMPR: CMPEN Mask            */

#define LPADC_ADCMPR_CMPIE_Pos           (1)                                               /*!< LPADC_T::ADCMPR: CMPIE Position        */
#define LPADC_ADCMPR_CMPIE_Msk           (0x1ul << LPADC_ADCMPR_CMPIE_Pos)                 /*!< LPADC_T::ADCMPR: CMPIE Mask            */

#define LPADC_ADCMPR_CMPCOND_Pos         (2)                                               /*!< LPADC_T::ADCMPR: CMPCOND Position      */
#define LPADC_ADCMPR_CMPCOND_Msk         (0x1ul << LPADC_ADCMPR_CMPCOND_Pos)               /*!< LPADC_T::ADCMPR: CMPCOND Mask          */

#define LPADC_ADCMPR_CMPCH_Pos           (3)                                               /*!< LPADC_T::ADCMPR: CMPCH Position        */
#define LPADC_ADCMPR_CMPCH_Msk           (0x1ful << LPADC_ADCMPR_CMPCH_Pos)                /*!< LPADC_T::ADCMPR: CMPCH Mask            */

#define LPADC_ADCMPR_CMPMATCNT_Pos       (8)                                               /*!< LPADC_T::ADCMPR: CMPMATCNT Position    */
#define LPADC_ADCMPR_CMPMATCNT_Msk       (0xful << LPADC_ADCMPR_CMPMATCNT_Pos)             /*!< LPADC_T::ADCMPR: CMPMATCNT Mask        */

#define LPADC_ADCMPR_CMPWEN_Pos          (15)                                              /*!< LPADC_T::ADCMPR: CMPWEN Position       */
#define LPADC_ADCMPR_CMPWEN_Msk          (0x1ul << LPADC_ADCMPR_CMPWEN_Pos)                /*!< LPADC_T::ADCMPR: CMPWEN Mask           */

#define LPADC_ADCMPR_CMPD_Pos            (16)                                              /*!< LPADC_T::ADCMPR: CMPD Position         */
#define LPADC_ADCMPR_CMPD_Msk            (0xffful << LPADC_ADCMPR_CMPD_Pos)                /*!< LPADC_T::ADCMPR: CMPD Mask             */

#define LPADC_ADCMPR0_CMPEN_Pos          (0)                                               /*!< LPADC_T::ADCMPR0: CMPEN Position       */
#define LPADC_ADCMPR0_CMPEN_Msk          (0x1ul << LPADC_ADCMPR0_CMPEN_Pos)                /*!< LPADC_T::ADCMPR0: CMPEN Mask           */

#define LPADC_ADCMPR0_CMPIE_Pos          (1)                                               /*!< LPADC_T::ADCMPR0: CMPIE Position       */
#define LPADC_ADCMPR0_CMPIE_Msk          (0x1ul << LPADC_ADCMPR0_CMPIE_Pos)                /*!< LPADC_T::ADCMPR0: CMPIE Mask           */

#define LPADC_ADCMPR0_CMPCOND_Pos        (2)                                               /*!< LPADC_T::ADCMPR0: CMPCOND Position     */
#define LPADC_ADCMPR0_CMPCOND_Msk        (0x1ul << LPADC_ADCMPR0_CMPCOND_Pos)              /*!< LPADC_T::ADCMPR0: CMPCOND Mask         */

#define LPADC_ADCMPR0_CMPCH_Pos          (3)                                               /*!< LPADC_T::ADCMPR0: CMPCH Position       */
#define LPADC_ADCMPR0_CMPCH_Msk          (0x1ful << LPADC_ADCMPR0_CMPCH_Pos)               /*!< LPADC_T::ADCMPR0: CMPCH Mask           */

#define LPADC_ADCMPR0_CMPMCNT_Pos        (8)                                               /*!< LPADC_T::ADCMPR0: CMPMCNT Position     */
#define LPADC_ADCMPR0_CMPMCNT_Msk        (0xful << LPADC_ADCMPR0_CMPMCNT_Pos)              /*!< LPADC_T::ADCMPR0: CMPMCNT Mask         */

#define LPADC_ADCMPR0_CMPWEN_Pos         (15)                                              /*!< LPADC_T::ADCMPR0: CMPWEN Position      */
#define LPADC_ADCMPR0_CMPWEN_Msk         (0x1ul << LPADC_ADCMPR0_CMPWEN_Pos)               /*!< LPADC_T::ADCMPR0: CMPWEN Mask          */

#define LPADC_ADCMPR0_CMPD_Pos           (16)                                              /*!< LPADC_T::ADCMPR0: CMPD Position        */
#define LPADC_ADCMPR0_CMPD_Msk           (0xffful << LPADC_ADCMPR0_CMPD_Pos)               /*!< LPADC_T::ADCMPR0: CMPD Mask            */

#define LPADC_ADCMPR1_CMPEN_Pos          (0)                                               /*!< LPADC_T::ADCMPR1: CMPEN Position       */
#define LPADC_ADCMPR1_CMPEN_Msk          (0x1ul << LPADC_ADCMPR1_CMPEN_Pos)                /*!< LPADC_T::ADCMPR1: CMPEN Mask           */

#define LPADC_ADCMPR1_CMPIE_Pos          (1)                                               /*!< LPADC_T::ADCMPR1: CMPIE Position       */
#define LPADC_ADCMPR1_CMPIE_Msk          (0x1ul << LPADC_ADCMPR1_CMPIE_Pos)                /*!< LPADC_T::ADCMPR1: CMPIE Mask           */

#define LPADC_ADCMPR1_CMPCOND_Pos        (2)                                               /*!< LPADC_T::ADCMPR1: CMPCOND Position     */
#define LPADC_ADCMPR1_CMPCOND_Msk        (0x1ul << LPADC_ADCMPR1_CMPCOND_Pos)              /*!< LPADC_T::ADCMPR1: CMPCOND Mask         */

#define LPADC_ADCMPR1_CMPCH_Pos          (3)                                               /*!< LPADC_T::ADCMPR1: CMPCH Position       */
#define LPADC_ADCMPR1_CMPCH_Msk          (0x1ful << LPADC_ADCMPR1_CMPCH_Pos)               /*!< LPADC_T::ADCMPR1: CMPCH Mask           */

#define LPADC_ADCMPR1_CMPMCNT_Pos        (8)                                               /*!< LPADC_T::ADCMPR1: CMPMCNT Position     */
#define LPADC_ADCMPR1_CMPMCNT_Msk        (0xful << LPADC_ADCMPR1_CMPMCNT_Pos)              /*!< LPADC_T::ADCMPR1: CMPMCNT Mask         */

#define LPADC_ADCMPR1_CMPWEN_Pos         (15)                                              /*!< LPADC_T::ADCMPR1: CMPWEN Position      */
#define LPADC_ADCMPR1_CMPWEN_Msk         (0x1ul << LPADC_ADCMPR1_CMPWEN_Pos)               /*!< LPADC_T::ADCMPR1: CMPWEN Mask          */

#define LPADC_ADCMPR1_CMPD_Pos           (16)                                              /*!< LPADC_T::ADCMPR1: CMPD Position        */
#define LPADC_ADCMPR1_CMPD_Msk           (0xffful << LPADC_ADCMPR1_CMPD_Pos)               /*!< LPADC_T::ADCMPR1: CMPD Mask            */

#define LPADC_ADSR0_ADF_Pos              (0)                                               /*!< LPADC_T::ADSR0: ADF Position           */
#define LPADC_ADSR0_ADF_Msk              (0x1ul << LPADC_ADSR0_ADF_Pos)                    /*!< LPADC_T::ADSR0: ADF Mask               */

#define LPADC_ADSR0_CMPF0_Pos            (1)                                               /*!< LPADC_T::ADSR0: CMPF0 Position         */
#define LPADC_ADSR0_CMPF0_Msk            (0x1ul << LPADC_ADSR0_CMPF0_Pos)                  /*!< LPADC_T::ADSR0: CMPF0 Mask             */

#define LPADC_ADSR0_CMPF1_Pos            (2)                                               /*!< LPADC_T::ADSR0: CMPF1 Position         */
#define LPADC_ADSR0_CMPF1_Msk            (0x1ul << LPADC_ADSR0_CMPF1_Pos)                  /*!< LPADC_T::ADSR0: CMPF1 Mask             */

#define LPADC_ADSR0_BUSY_Pos             (7)                                               /*!< LPADC_T::ADSR0: BUSY Position          */
#define LPADC_ADSR0_BUSY_Msk             (0x1ul << LPADC_ADSR0_BUSY_Pos)                   /*!< LPADC_T::ADSR0: BUSY Mask              */

#define LPADC_ADSR0_VALIDF_Pos           (8)                                               /*!< LPADC_T::ADSR0: VALIDF Position        */
#define LPADC_ADSR0_VALIDF_Msk           (0x1ul << LPADC_ADSR0_VALIDF_Pos)                 /*!< LPADC_T::ADSR0: VALIDF Mask            */

#define LPADC_ADSR0_OVERRUNF_Pos         (16)                                              /*!< LPADC_T::ADSR0: OVERRUNF Position      */
#define LPADC_ADSR0_OVERRUNF_Msk         (0x1ul << LPADC_ADSR0_OVERRUNF_Pos)               /*!< LPADC_T::ADSR0: OVERRUNF Mask          */

#define LPADC_ADSR0_ADPRDY_Pos           (24)                                              /*!< LPADC_T::ADSR0: ADPRDY Position        */
#define LPADC_ADSR0_ADPRDY_Msk           (0x1ul << LPADC_ADSR0_ADPRDY_Pos)                 /*!< LPADC_T::ADSR0: ADPRDY Mask            */

#define LPADC_ADSR0_CHANNEL_Pos          (27)                                              /*!< LPADC_T::ADSR0: CHANNEL Position       */
#define LPADC_ADSR0_CHANNEL_Msk          (0x1ful << LPADC_ADSR0_CHANNEL_Pos)               /*!< LPADC_T::ADSR0: CHANNEL Mask           */

#define LPADC_ADSR1_VALID_Pos            (0)                                               /*!< LPADC_T::ADSR1: VALID Position         */
#define LPADC_ADSR1_VALID_Msk            (0xfffffffful << LPADC_ADSR1_VALID_Pos)           /*!< LPADC_T::ADSR1: VALID Mask             */

#define LPADC_ADSR2_OVERRUN_Pos          (0)                                               /*!< LPADC_T::ADSR2: OVERRUN Position       */
#define LPADC_ADSR2_OVERRUN_Msk          (0xfffffffful << LPADC_ADSR2_OVERRUN_Pos)         /*!< LPADC_T::ADSR2: OVERRUN Mask           */

#define LPADC_ESMPCTL_EXTSMPT_Pos        (0)                                               /*!< LPADC_T::ESMPCTL: EXTSMPT Position     */
#define LPADC_ESMPCTL_EXTSMPT_Msk        (0x3ffful << LPADC_ESMPCTL_EXTSMPT_Pos)           /*!< LPADC_T::ESMPCTL: EXTSMPT Mask         */

#define LPADC_CFDCTL_PRECHEN_Pos         (0)                                               /*!< LPADC_T::CFDCTL: PRECHEN Position      */
#define LPADC_CFDCTL_PRECHEN_Msk         (0x1ul << LPADC_CFDCTL_PRECHEN_Pos)               /*!< LPADC_T::CFDCTL: PRECHEN Mask          */

#define LPADC_CFDCTL_DISCHEN_Pos         (1)                                               /*!< LPADC_T::CFDCTL: DISCHEN Position      */
#define LPADC_CFDCTL_DISCHEN_Msk         (0x1ul << LPADC_CFDCTL_DISCHEN_Pos)               /*!< LPADC_T::CFDCTL: DISCHEN Mask          */

#define LPADC_CFDCTL_FDETCHEN_Pos        (8)                                               /*!< LPADC_T::CFDCTL: FDETCHEN Position     */
#define LPADC_CFDCTL_FDETCHEN_Msk        (0x1ul << LPADC_CFDCTL_FDETCHEN_Pos)              /*!< LPADC_T::CFDCTL: FDETCHEN Mask         */

#define LPADC_ADPDMA_CURDAT_Pos          (0)                                               /*!< LPADC_T::ADPDMA: CURDAT Position       */
#define LPADC_ADPDMA_CURDAT_Msk          (0x3fffful << LPADC_ADPDMA_CURDAT_Pos)            /*!< LPADC_T::ADPDMA: CURDAT Mask           */

#define LPADC_ADCAL_CALEN_Pos            (0)                                               /*!< LPADC_T::ADCAL: CALEN Position         */
#define LPADC_ADCAL_CALEN_Msk            (0x1ul << LPADC_ADCAL_CALEN_Pos)                  /*!< LPADC_T::ADCAL: CALEN Mask             */

#define LPADC_ADCAL_CALIE_Pos            (1)                                               /*!< LPADC_T::ADCAL: CALIE Position         */
#define LPADC_ADCAL_CALIE_Msk            (0x1ul << LPADC_ADCAL_CALIE_Pos)                  /*!< LPADC_T::ADCAL: CALIE Mask             */

#define LPADC_ADCALSTS_CALIF_Pos         (0)                                               /*!< LPADC_T::ADCALSTS: CALIF Position      */
#define LPADC_ADCALSTS_CALIF_Msk         (0x1ul << LPADC_ADCALSTS_CALIF_Pos)               /*!< LPADC_T::ADCALSTS: CALIF Mask          */

#define LPADC_AUTOCTL_TRIGSEL_Pos        (0)                                               /*!< LPADC_T::AUTOCTL: TRIGSEL Position     */
#define LPADC_AUTOCTL_TRIGSEL_Msk        (0xful << LPADC_AUTOCTL_TRIGSEL_Pos)              /*!< LPADC_T::AUTOCTL: TRIGSEL Mask         */

#define LPADC_AUTOCTL_TRIGEN_Pos         (4)                                               /*!< LPADC_T::AUTOCTL: TRIGEN Position      */
#define LPADC_AUTOCTL_TRIGEN_Msk         (0x1ul << LPADC_AUTOCTL_TRIGEN_Pos)               /*!< LPADC_T::AUTOCTL: TRIGEN Mask          */

#define LPADC_AUTOCTL_ADWKEN_Pos         (8)                                               /*!< LPADC_T::AUTOCTL: ADWKEN Position      */
#define LPADC_AUTOCTL_ADWKEN_Msk         (0x1ul << LPADC_AUTOCTL_ADWKEN_Pos)               /*!< LPADC_T::AUTOCTL: ADWKEN Mask          */

#define LPADC_AUTOCTL_CMP0WKEN_Pos       (9)                                               /*!< LPADC_T::AUTOCTL: CMP0WKEN Position    */
#define LPADC_AUTOCTL_CMP0WKEN_Msk       (0x1ul << LPADC_AUTOCTL_CMP0WKEN_Pos)             /*!< LPADC_T::AUTOCTL: CMP0WKEN Mask        */

#define LPADC_AUTOCTL_CMP1WKEN_Pos       (10)                                              /*!< LPADC_T::AUTOCTL: CMP1WKEN Position    */
#define LPADC_AUTOCTL_CMP1WKEN_Msk       (0x1ul << LPADC_AUTOCTL_CMP1WKEN_Pos)             /*!< LPADC_T::AUTOCTL: CMP1WKEN Mask        */

#define LPADC_AUTOCTL_AUTOEN_Pos         (31)                                              /*!< LPADC_T::AUTOCTL: AUTOEN Position      */
#define LPADC_AUTOCTL_AUTOEN_Msk         (0x1ul << LPADC_AUTOCTL_AUTOEN_Pos)               /*!< LPADC_T::AUTOCTL: AUTOEN Mask          */

#define LPADC_AUTOSTRG_SWTRIG_Pos        (0)                                               /*!< LPADC_T::AUTOSTRG: SWTRIG Position     */
#define LPADC_AUTOSTRG_SWTRIG_Msk        (0x1ul << LPADC_AUTOSTRG_SWTRIG_Pos)              /*!< LPADC_T::AUTOSTRG: SWTRIG Mask         */

#define LPADC_AUTOSTS_ADWKF_Pos          (0)                                               /*!< LPADC_T::AUTOSTS: ADWKF Position       */
#define LPADC_AUTOSTS_ADWKF_Msk          (0x1ul << LPADC_AUTOSTS_ADWKF_Pos)                /*!< LPADC_T::AUTOSTS: ADWKF Mask           */

#define LPADC_AUTOSTS_CMP0WKF_Pos        (1)                                               /*!< LPADC_T::AUTOSTS: CMP0WKF Position     */
#define LPADC_AUTOSTS_CMP0WKF_Msk        (0x1ul << LPADC_AUTOSTS_CMP0WKF_Pos)              /*!< LPADC_T::AUTOSTS: CMP0WKF Mask         */

#define LPADC_AUTOSTS_CMP1WKF_Pos        (2)                                               /*!< LPADC_T::AUTOSTS: CMP1WKF Position     */
#define LPADC_AUTOSTS_CMP1WKF_Msk        (0x1ul << LPADC_AUTOSTS_CMP1WKF_Pos)              /*!< LPADC_T::AUTOSTS: CMP1WKF Mask         */


/** @} LPADC_CONST */
/** @} end of LPADC register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __LPADC_REG_H__ */
