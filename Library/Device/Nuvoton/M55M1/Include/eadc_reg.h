/**************************************************************************//**
 * @file     eadc_reg.h
 * @version  V1.00
 * @brief    EADC register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __EADC_REG_H__
#define __EADC_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup EADC Enhanced Analog to Digital Converter (EADC)
    Memory Mapped Structure for EADC Controller
    @{
*/

typedef struct
{


    /**
     * @var EADC_T::DAT[19]
     * Offset: 0x00  EADC Data Register 0~18 for Sample Module 0~18
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RESULT    |EADC Conversion Result
     * |        |          |This field contains 12 bits conversion result.
     * |        |          |The 12-bit EADC conversion result with unsigned format will be filled in RESULT[11:0] and zero will be filled in RESULT[15:12].
     * |        |          |Note: When operating in oversampling mode, RESULT[15:0] can represent oversampling results.
     * |[16]    |OV        |Overrun Flag
     * |        |          |If converted data in RESULT[11:0] has not been read before new conversion result is loaded to this register, OV is set to 1.
     * |        |          |0 = Data in RESULT[11:0] is recent conversion result.
     * |        |          |1 = Data in RESULT[11:0] is overwrite.
     * |        |          |Note: It is cleared by hardware after EADC_DAT register is read.
     * |[17]    |VALID     |Valid Flag
     * |        |          |This bit is set to 1 when corresponding sample module channel analog input conversion is completed and cleared by hardware after EADC_DAT register is read.
     * |        |          |0 = Data in RESULT[11:0] bits is not valid.
     * |        |          |1 = Data in RESULT[11:0] bits is valid.
     * @var EADC_T::CURDAT
     * Offset: 0x4C  EADC PDMA Current Transfer Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[27:0]  |CURDAT    |EADC PDMA Current Transfer Data
     * |        |          |This register is a shadow register of EADC_DATn (n=0~27) for PDMA support.
     * |        |          |Note: These bits are read only.
     * @var EADC_T::CTL
     * Offset: 0x50  EADC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADCEN     |EADC Converter Enable Bit
     * |        |          |0 = EADC Disabled.
     * |        |          |1 = EADC Enabled.
     * |        |          |Note: Before starting EADC conversion function, this bit should be set to 1
     * |        |          |Clear it to 0 to disable EADC converter analog circuit power consumption.
     * |[1]     |ADCRST    |EADC Converter Control Circuits Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Cause EADC control circuits reset to initial state, but not change the EADC registers value.
     * |        |          |Note: ADCRST bit remains 1 during EADC reset, when EADC reset end, the ADCRST bit is automatically cleared to 0.
     * |[2]     |ADCIEN0   |Specific Sample Module EADC ADINT0 Interrupt Enable Bit
     * |        |          |The EADC converter generates a conversion end ADIF0 (EADC_STATUS2[0]) upon the end of specific sample module EADC conversion
     * |        |          |If ADCIEN0 bit is set then conversion end interrupt request ADINT0 is generated.
     * |        |          |0 = Specific sample module EADC ADINT0 interrupt function Disabled.
     * |        |          |1 = Specific sample module EADC ADINT0 interrupt function Enabled.
     * |[3]     |ADCIEN1   |Specific Sample Module EADC ADINT1 Interrupt Enable Bit
     * |        |          |The EADC converter generates a conversion end ADIF1 (EADC_STATUS2[1]) upon the end of specific sample module EADC conversion
     * |        |          |If EADCIEN1 bit is set then conversion end interrupt request ADINT1 is generated.
     * |        |          |0 = Specific sample module EADC ADINT1 interrupt function Disabled.
     * |        |          |1 = Specific sample module EADC ADINT1 interrupt function Enabled.
     * |[4]     |ADCIEN2   |Specific Sample Module EADC ADINT2 Interrupt Enable Bit
     * |        |          |The EADC converter generates a conversion end ADIF2 (EADC_STATUS2[2]) upon the end of specific sample module EADC conversion
     * |        |          |If EADCIEN2 bit is set then conversion end interrupt request ADINT2 is generated.
     * |        |          |0 = Specific sample module EADC ADINT2 interrupt function Disabled.
     * |        |          |1 = Specific sample module EADC ADINT2 interrupt function Enabled.
     * |[5]     |ADCIEN3   |Specific Sample Module EADC ADINT3 Interrupt Enable Bit
     * |        |          |The EADC converter generates a conversion end ADIF3 (EADC_STATUS2[3]) upon the end of specific sample module EADC conversion
     * |        |          |If EADCIEN3 bit is set then conversion end interrupt request ADINT3 is generated.
     * |        |          |0 = Specific sample module EADC ADINT3 interrupt function Disabled.
     * |        |          |1 = Specific sample module EADC ADINT3 interrupt function Enabled.
     * |[8]     |DIFFEN    |Differential Analog Input Mode Enable Bit
     * |        |          |0 = Single-end analog input mode.
     * |        |          |1 = Differential analog input mode.
     * |        |          |Note: In the differential mode, the input channel pair must be configured to EADC_CHx, EADC_CHx+1 , x=0,2,4,6,8,10,12,14.
     * |[9]     |DMOF      |ADC Differential Input Mode Output Format
     * |        |          |0 = ADC conversion result will be filled in RESULT (EADC_DATn[15:0], where n= 0 ~27) with unsigned format.
     * |        |          |1 = ADC conversion result will be filled in RESULT (EADC_DATn[15:0], where n= 0 ~27) with 2'complement format.
     * @var EADC_T::SWTRG
     * Offset: 0x54  EADC Sample Module Software Start Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[27:0]  |SWTRG     |EADC Sample Module 0~27 Software Force to Start EADC Conversion
     * |        |          |0 = No effect.
     * |        |          |1 = Cause an EADC conversion when the priority is given to sample module.
     * |        |          |Note: After writing this register to start EADC conversion, the EADC_PENDSTS register will show which sample module will conversion
     * |        |          |If user want to disable the conversion of the sample module, user can write EADC_PENDSTS register to clear it.
     * @var EADC_T::PENDSTS
     * Offset: 0x58  EADC Start of Conversion Pending Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[27:0]  |STPF      |EADC Sample Module 0~27 Start of Conversion Pending Flag
     * |        |          |Read Operation:
     * |        |          |0 = There is no pending conversion for sample module.
     * |        |          |1 = Sample module EADC start of conversion is pending.
     * |        |          |Write Operation:
     * |        |          |1 = Clear pending flag & cancel the conversion for sample module.
     * |        |          |Note: This bit remains 1 during pending state, when the respective EADC conversion is end, the STPFn (n=0~27) bit is automatically cleared to 0
     * @var EADC_T::OVSTS
     * Offset: 0x5C  EADC Sample Module Start of Conversion Overrun Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[27:0]  |SPOVF     |EADC SAMPLE0~27 Overrun Flag
     * |        |          |0 = No sample module event overrun.
     * |        |          |1 = Indicates a new sample module event is generated while an old one event is pending.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * @var EADC_T::CTL1
     * Offset: 0x60  EADC Control1 Register
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
     * |[5:4]   |RESSEL    |Resolution Select Bits
     * |        |          |00 = ADC resolution 12 bits.
     * |        |          |01 = ADC resolution 10 bits.
     * |        |          |10 = ADC resolution 8 bits.
     * |        |          |11 = Reserved.
     * |[8]     |FDETCHEN  |Floating Detect Channel Enable Bit
     * |        |          |0 = Floating Detect Channel Disabled.
     * |        |          |1 = Floating Detect Channel Enabled.
     * |[20]    |CMP0TRG   |ADC Comparator 0 Trigger EPWM Brake Enable Bit
     * |        |          |0 = Comparator 0 trigger EPWM brake is disabled.
     * |        |          |1 = Comparator 0 trigger EPWM brake is enabled.
     * |[21]    |CMP1TRG   |ADC Comparator 1 Trigger EPWM Brake Enable Bit
     * |        |          |0 = Comparator 1 trigger EPWM brake is disabled.
     * |        |          |1 = Comparator 1 trigger EPWM brake is enabled.
     * |[22]    |CMP2TRG   |ADC Comparator 2 Trigger EPWM Brake Enable Bit
     * |        |          |0 = Comparator 2 trigger EPWM brake is disabled.
     * |        |          |1 = Comparator 2 trigger EPWM brake is enabled.
     * |[23]    |CMP3TRG   |ADC Comparator 3 Trigger EPWM Brake Enable Bit
     * |        |          |0 = Comparator 3 trigger EPWM brake is disabled.
     * |        |          |1 = Comparator 3 trigger EPWM brake is enabled.
     * @var EADC_T::SCTL[19]
     * Offset: 0x80  EADC Sample Module 0~18 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |CHSEL     |EADC Sample Module Channel Selection
     * |        |          |00H = EADC_CH0.
     * |        |          |01H = EADC_CH1.
     * |        |          |02H = EADC_CH2.
     * |        |          |03H = EADC_CH3.
     * |        |          |04H = EADC_CH4.
     * |        |          |05H = EADC_CH5.
     * |        |          |06H = EADC_CH6.
     * |        |          |07H = EADC_CH7.
     * |        |          |08H = EADC_CH8.
     * |        |          |09H = EADC_CH9.
     * |        |          |0AH = EADC_CH10.
     * |        |          |0BH = EADC_CH11.
     * |        |          |0CH = EADC_CH12.
     * |        |          |0DH = EADC_CH13.
     * |        |          |0EH = EADC_CH14.
     * |        |          |0FH = EADC_CH15.
     * |        |          |10H = EADC_CH16.
     * |        |          |11H = EADC_CH17.
     * |        |          |12H = EADC_CH18.
     * |        |          |13H = EADC_CH19.
     * |        |          |14H = EADC_CH20.
     * |        |          |15H = EADC_CH21.
     * |        |          |16H = EADC_CH22.
     * |        |          |17H = EADC_CH23.
     * |        |          |Others = Reserved.
     * |[5]     |INTPOS    |Interrupt Flag Position Select
     * |        |          |0 = Set ADIFn (EADC_STATUS2[n], n=0~3) at EADC end of conversion.
     * |        |          |1 = Set ADIFn (EADC_STATUS2[n], n=0~3) at EADC start of conversion.
     * |[7:6]   |TRGDLDIV  |EADC Sample Module Start of Conversion Trigger Delay Clock Divider Selection
     * |        |          |Trigger delay clock frequency:
     * |        |          |00 = EADC_CLK/1.
     * |        |          |01 = EADC_CLK/2.
     * |        |          |10 = EADC_CLK/4.
     * |        |          |11 = EADC_CLK/16.
     * |[15:8]  |TRGDLCNT  |EADC Sample Module Start of Conversion Trigger Delay Time
     * |        |          |Trigger delay time = TRGDLCNT x EADC_CLK period x n (n=1,2,4,16 from TRGDLDIV setting).
     * |[21:16] |TRGSEL    |EADC Sample Module Start of Conversion Trigger Source Selection
     * |        |          |0H = Disable trigger.
     * |        |          |1H = External trigger from EADC0_ST pin input.
     * |        |          |2H = EADC ADINT0 interrupt EOC pulse trigger.
     * |        |          |3H = EADC ADINT1 interrupt EOC pulse trigger.
     * |        |          |4H = Timer0 overflow pulse trigger.
     * |        |          |5H = Timer1 overflow pulse trigger.
     * |        |          |6H = Timer2 overflow pulse trigger.
     * |        |          |7H = Timer3 overflow pulse trigger.
     * |        |          |8H = EPWM0TG0.
     * |        |          |9H = EPWM0TG1.
     * |        |          |AH = EPWM0TG2.
     * |        |          |BH = EPWM0TG3.
     * |        |          |CH = EPWM0TG4.
     * |        |          |DH = EPWM0TG5.
     * |        |          |EH = EPWM1TG0.
     * |        |          |FH = EPWM1TG1.
     * |        |          |10H = EPWM1TG2.
     * |        |          |11H = EPWM1TG3.
     * |        |          |12H = EPWM1TG4.
     * |        |          |13H = EPWM1TG5.
     * |        |          |20H = BPWM0TG.
     * |        |          |21H = BPWM1TG.
     * |        |          |22H = ACMP0_INT.
     * |        |          |23H = ACMP1_INT.
     * |        |          |24H = ACMP2_INT.
     * |        |          |25H = ACMP3_INT.
     * |        |          |other = Reserved.
     * |[22]    |EXTREN    |EADC External Trigger Rising Edge Enable Bit
     * |        |          |0 = Rising edge Disabled when EADC selects EADC_ST as trigger source.
     * |        |          |1 = Rising edge Enabled when EADC selects EADC_ST as trigger source.
     * |[23]    |EXTFEN    |EADC External Trigger Falling Edge Enable Bit
     * |        |          |0 = Falling edge Disabled when EADC selects EADC_ST as trigger source.
     * |        |          |1 = Falling edge Enabled when EADC selects EADC_ST as trigger source.
     * |[31:24] |EXTSMPT   |EADC Sampling Time Extend
     * |        |          |When EADC converting at high conversion rate, the sampling time of analog input voltage may not be enough if input channel loading is heavy, and software can extend EADC sampling time after trigger source is coming to get enough sampling time.
     * |        |          |Extended Sampling Time = (EXTSMPT x EADC_CLK period x n)+2 [n=1,2,4,16 from EXTSTDIV setting].
     * @var EADC_T::INTSRC[4]
     * Offset: 0xD0  EADC interrupt 0~3 Source Enable Control Register.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SPLIE0    |Sample Module 0 Interrupt Enable Bit
     * |        |          |0 = Sample Module 0 interrupt Disabled.
     * |        |          |1 = Sample Module 0 interrupt Enabled.
     * |[1]     |SPLIE1    |Sample Module 1 Interrupt Enable Bit
     * |        |          |0 = Sample Module 1 interrupt Disabled.
     * |        |          |1 = Sample Module 1 interrupt Enabled.
     * |[2]     |SPLIE2    |Sample Module 2 Interrupt Enable Bit
     * |        |          |0 = Sample Module 2 interrupt Disabled.
     * |        |          |1 = Sample Module 2 interrupt Enabled.
     * |[3]     |SPLIE3    |Sample Module 3 Interrupt Enable Bit
     * |        |          |0 = Sample Module 3 interrupt Disabled.
     * |        |          |1 = Sample Module 3 interrupt Enabled.
     * |[4]     |SPLIE4    |Sample Module 4 Interrupt Enable Bit
     * |        |          |0 = Sample Module 4 interrupt Disabled.
     * |        |          |1 = Sample Module 4 interrupt Enabled.
     * |[5]     |SPLIE5    |Sample Module 5 Interrupt Enable Bit
     * |        |          |0 = Sample Module 5 interrupt Disabled.
     * |        |          |1 = Sample Module 5 interrupt Enabled.
     * |[6]     |SPLIE6    |Sample Module 6 Interrupt Enable Bit
     * |        |          |0 = Sample Module 6 interrupt Disabled.
     * |        |          |1 = Sample Module 6 interrupt Enabled.
     * |[7]     |SPLIE7    |Sample Module 7 Interrupt Enable Bit
     * |        |          |0 = Sample Module 7 interrupt Disabled.
     * |        |          |1 = Sample Module 7 interrupt Enabled.
     * |[8]     |SPLIE8    |Sample Module 8 Interrupt Enable Bit
     * |        |          |0 = Sample Module 8 interrupt Disabled.
     * |        |          |1 = Sample Module 8 interrupt Enabled.
     * |[9]     |SPLIE9    |Sample Module 9 Interrupt Enable Bit
     * |        |          |0 = Sample Module 9 interrupt Disabled.
     * |        |          |1 = Sample Module 9 interrupt Enabled.
     * |[10]    |SPLIE10   |Sample Module 10 Interrupt Enable Bit
     * |        |          |0 = Sample Module 10 interrupt Disabled.
     * |        |          |1 = Sample Module 10 interrupt Enabled.
     * |[11]    |SPLIE11   |Sample Module 11 Interrupt Enable Bit
     * |        |          |0 = Sample Module 11 interrupt Disabled.
     * |        |          |1 = Sample Module 11 interrupt Enabled.
     * |[12]    |SPLIE12   |Sample Module 12 Interrupt Enable Bit
     * |        |          |0 = Sample Module 12 interrupt Disabled.
     * |        |          |1 = Sample Module 12 interrupt Enabled.
     * |[13]    |SPLIE13   |Sample Module 13 Interrupt Enable Bit
     * |        |          |0 = Sample Module 13 interrupt Disabled.
     * |        |          |1 = Sample Module 13 interrupt Enabled.
     * |[14]    |SPLIE14   |Sample Module 14 Interrupt Enable Bit
     * |        |          |0 = Sample Module 14 interrupt Disabled.
     * |        |          |1 = Sample Module 14 interrupt Enabled.
     * |[15]    |SPLIE15   |Sample Module 15 Interrupt Enable Bit
     * |        |          |0 = Sample Module 15 interrupt Disabled.
     * |        |          |1 = Sample Module 15 interrupt Enabled.
     * |[16]    |SPLIE16   |Sample Module 16 Interrupt Enable Bit
     * |        |          |0 = Sample Module 16 interrupt Disabled.
     * |        |          |1 = Sample Module 16 interrupt Enabled.
     * |[17]    |SPLIE17   |Sample Module 17 Interrupt Enable Bit
     * |        |          |0 = Sample Module 17 interrupt Disabled.
     * |        |          |1 = Sample Module 17 interrupt Enabled.
     * |[18]    |SPLIE18   |Sample Module 18 Interrupt Enable Bit
     * |        |          |0 = Sample Module 18 interrupt Disabled.
     * |        |          |1 = Sample Module 18 interrupt Enabled.
     * |[19]    |SPLIE19   |Sample Module 19 Interrupt Enable Bit
     * |        |          |0 = Sample Module 19 interrupt Disabled.
     * |        |          |1 = Sample Module 19 interrupt Enabled.
     * |[20]    |SPLIE20   |Sample Module 20 Interrupt Enable Bit
     * |        |          |0 = Sample Module 20 interrupt Disabled.
     * |        |          |1 = Sample Module 20 interrupt Enabled.
     * |[21]    |SPLIE21   |Sample Module 21 Interrupt Enable Bit
     * |        |          |0 = Sample Module 21 interrupt Disabled.
     * |        |          |1 = Sample Module 21 interrupt Enabled.
     * |[22]    |SPLIE22   |Sample Module 22 Interrupt Enable Bit
     * |        |          |0 = Sample Module 22 interrupt Disabled.
     * |        |          |1 = Sample Module 22 interrupt Enabled.
     * |[23]    |SPLIE23   |Sample Module 23 Interrupt Enable Bit
     * |        |          |0 = Sample Module 23 interrupt Disabled.
     * |        |          |1 = Sample Module 23 interrupt Enabled.
     * |[24]    |SPLIE24   |Sample Module 24 Interrupt Enable Bit
     * |        |          |0 = Sample Module 24 interrupt Disabled.
     * |        |          |1 = Sample Module 24 interrupt Enabled.
     * |[25]    |SPLIE25   |Sample Module 25 Interrupt Enable Bit
     * |        |          |0 = Sample Module 25 interrupt Disabled.
     * |        |          |1 = Sample Module 25 interrupt Enabled.
     * |[26]    |SPLIE26   |Sample Module 26 Interrupt Enable Bit
     * |        |          |0 = Sample Module 26 interrupt Disabled.
     * |        |          |1 = Sample Module 26 interrupt Enabled.
     * |[27]    |SPLIE27   |Sample Module 27 Interrupt Enable Bit
     * |        |          |0 = Sample Module 27 interrupt Disabled.
     * |        |          |1 = Sample Module 27 interrupt Enabled.
     * @var EADC_T::CMP[4]
     * Offset: 0xE0  EADC Result Compare Register 0~3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADCMPEN   |EADC Result Compare Enable Bit
     * |        |          |0 = Compare Disabled.
     * |        |          |1 = Compare Enabled.
     * |        |          |Set this bit to 1 to enable compare CMPDAT (EADC_CMPn[27:16], n=0~3) with specified sample module conversion result when converted data is loaded into EADC_DAT register.
     * |[1]     |ADCMPIE   |EADC Result Compare Interrupt Enable Bit
     * |        |          |0 = Compare function interrupt Disabled.
     * |        |          |1 = Compare function interrupt Enabled.
     * |        |          |If the compare function is enabled and the compare condition matches the setting of CMPCOND (EADC_CMPn[2], n=0~3) and CMPMCNT (EADC_CMPn[11:8], n=0~3), ADCMPFn (EADC_STATUS2[n+4], n=0~3) will be asserted, in the meanwhile, if ADCMPIE is set to 1, a compare interrupt request is generated.
     * |[2]     |CMPCOND   |Compare Condition
     * |        |          |0= Set the compare condition as that when a 12-bit EADC conversion result is less than the 12-bit CMPDAT (EADC_CMPn [27:16]), the internal match counter will increase one.
     * |        |          |1= Set the compare condition as that when a 12-bit EADC conversion result is greater or equal to the 12-bit CMPDAT (EADC_CMPn [27:16]), the internal match counter will increase one.
     * |        |          |Note: When the internal counter reaches the value to (CMPMCNT (EADC_CMPn[11:8], n=0~3) +1), the ADCMPF bit will be set.
     * |[7:3]   |CMPSPL    |Compare Sample Module Selection
     * |        |          |00000 = Sample Module 0 conversion result EADC_DAT0 is selected to be compared.
     * |        |          |00001 = Sample Module 1 conversion result EADC_DAT1 is selected to be compared.
     * |        |          |00010 = Sample Module 2 conversion result EADC_DAT2 is selected to be compared.
     * |        |          |00011 = Sample Module 3 conversion result EADC_DAT3 is selected to be compared.
     * |        |          |00100 = Sample Module 4 conversion result EADC_DAT4 is selected to be compared.
     * |        |          |00101 = Sample Module 5 conversion result EADC_DAT5 is selected to be compared.
     * |        |          |00110 = Sample Module 6 conversion result EADC_DAT6 is selected to be compared.
     * |        |          |00111 = Sample Module 7 conversion result EADC_DAT7 is selected to be compared.
     * |        |          |01000 = Sample Module 8 conversion result EADC_DAT8 is selected to be compared.
     * |        |          |01001 = Sample Module 9 conversion result EADC_DAT9 is selected to be compared.
     * |        |          |01010 = Sample Module 10 conversion result EADC_DAT10 is selected to be compared.
     * |        |          |01011 = Sample Module 11 conversion result EADC_DAT11 is selected to be compared.
     * |        |          |01100 = Sample Module 12 conversion result EADC_DAT12 is selected to be compared.
     * |        |          |01101 = Sample Module 13 conversion result EADC_DAT13 is selected to be compared.
     * |        |          |01110 = Sample Module 14 conversion result EADC_DAT14 is selected to be compared.
     * |        |          |01111 = Sample Module 15 conversion result EADC_DAT15 is selected to be compared.
     * |        |          |10000 = Sample Module 16 conversion result EADC_DAT16 is selected to be compared.
     * |        |          |10001 = Sample Module 17 conversion result EADC_DAT17 is selected to be compared.
     * |        |          |10010 = Sample Module 18 conversion result EADC_DAT18 is selected to be compared.
     * |        |          |10011 = Sample Module 19 conversion result EADC_DAT19 is selected to be compared.
     * |        |          |10100 = Sample Module 20 conversion result EADC_DAT20 is selected to be compared.
     * |        |          |10101 = Sample Module 21 conversion result EADC_DAT21 is selected to be compared.
     * |        |          |10110 = Sample Module 22 conversion result EADC_DAT22 is selected to be compared.
     * |        |          |10111 = Sample Module 23 conversion result EADC_DAT23 is selected to be compared.
     * |        |          |11000 = Sample Module 24 conversion result EADC_DAT24 is selected to be compared.
     * |        |          |11001 = Sample Module 25 conversion result EADC_DAT25 is selected to be compared.
     * |        |          |11010 = Sample Module 26 conversion result EADC_DAT26 is selected to be compared.
     * |        |          |11011 = Sample Module 27 conversion result EADC_DAT27 is selected to be compared.
     * |        |          |Others = reserved.
     * |[11:8]  |CMPMCNT   |Compare Match Count
     * |        |          |When the specified EADC sample module analog conversion result matches the compare condition defined by CMPCOND (EADC_CMPn[2], n=0~3), the internal match counter will increase 1
     * |        |          |If the compare result does not meet the compare condition, the internal compare match counter will reset to 0
     * |        |          |When the internal counter reaches the value to (CMPMCNT +1), the ADCMPFn (EADC_STATUS2[n+4], n=0~3) will be set.
     * |[15]    |CMPWEN    |Compare Window Mode Enable Bit
     * |        |          |0 = ADCMPF0 (EADC_STATUS2[4]) will be set when EADC_CMP0 compared condition matched
     * |        |          |ADCMPF2 (EADC_STATUS2[6]) will be set when EADC_CMP2 compared condition matched
     * |        |          |1 = ADCMPF0 (EADC_STATUS2[4]) will be set when both EADC_CMP0 and EADC_CMP1 compared condition matched
     * |        |          |ADCMPF2 (EADC_STATUS2[6]) will be set when both EADC_CMP2 and EADC_CMP3 compared condition matched.
     * |        |          |Note: This bit is only present in EADC_CMP0 and EADC_CMP2 register.
     * |        |          |Note: When in compare window mode, the CMPCNT setting only follow EADC_CMP0, EADC_CMP2 registers
     * |[27:16] |CMPDAT    |Comparison Data
     * |        |          |The 12 bits data is used to compare with conversion result of specified sample module
     * |        |          |User can use it to monitor the external analog input pin voltage transition without imposing a load on software.
     * @var EADC_T::STATUS0
     * Offset: 0xF0  EADC Status Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |VALID     |EADC_DAT0~15 Data Valid Flag
     * |        |          |It is a mirror of VALID bit in sample module EADC result data register EADC_DATn. (n=0~15).
     * |[31:16] |OV        |EADC_DAT0~15 Overrun Flag
     * |        |          |It is a mirror to OV bit in sample module EADC result data register EADC_DATn. (n=0~15).
     * @var EADC_T::STATUS1
     * Offset: 0xF4  EADC Status Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:0]  |VALID     |EADC_DAT16~27 Data Valid Flag
     * |        |          |It is a mirror of VALID bit in sample module EADC result data register EADC_DATn. (n=16~27).
     * |[27:16] |OV        |EADC_DAT16~27 Overrun Flag
     * |        |          |It is a mirror to OV bit in sample module EADC result data register EADC_DATn. (n=16~27).
     * @var EADC_T::STATUS2
     * Offset: 0xF8  EADC Status Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADIF0     |EADC ADINT0 Interrupt Flag
     * |        |          |0 = No ADINT0 interrupt pulse received.
     * |        |          |1 = ADINT0 interrupt pulse has been received.
     * |        |          |Note 1: This bit is cleared by writing 1 to it.
     * |        |          |Note 2:This bit indicates whether an EADC conversion of specific sample module has been completed
     * |[1]     |ADIF1     |EADC ADINT1 Interrupt Flag
     * |        |          |0 = No ADINT1 interrupt pulse received.
     * |        |          |1 = ADINT1 interrupt pulse has been received.
     * |        |          |Note 1: This bit is cleared by writing 1 to it.
     * |        |          |Note 2:This bit indicates whether an EADC conversion of specific sample module has been completed
     * |[2]     |ADIF2     |EADC ADINT2 Interrupt Flag
     * |        |          |0 = No ADINT2 interrupt pulse received.
     * |        |          |1 = ADINT2 interrupt pulse has been received.
     * |        |          |Note 1: This bit is cleared by writing 1 to it.
     * |        |          |Note 2:This bit indicates whether an EADC conversion of specific sample module has been completed
     * |[3]     |ADIF3     |EADC ADINT3 Interrupt Flag
     * |        |          |0 = No ADINT3 interrupt pulse received.
     * |        |          |1 = ADINT3 interrupt pulse has been received.
     * |        |          |Note 1: This bit is cleared by writing 1 to it.
     * |        |          |Note 2:This bit indicates whether an EADC conversion of specific sample module has been completed
     * |[4]     |ADCMPF0   |EADC Compare 0 Flag
     * |        |          |When the specific sample module EADC conversion result meets setting condition in EADC_CMP0 then this bit is set to 1.
     * |        |          |0 = Conversion result in EADC_DAT does not meet EADC_CMP0 register setting.
     * |        |          |1 = Conversion result in EADC_DAT meets EADC_CMP0 register setting.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[5]     |ADCMPF1   |EADC Compare 1 Flag
     * |        |          |When the specific sample module EADC conversion result meets setting condition in EADC_CMP1 then this bit is set to 1.
     * |        |          |0 = Conversion result in EADC_DAT does not meet EADC_CMP1 register setting.
     * |        |          |1 = Conversion result in EADC_DAT meets EADC_CMP1 register setting.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[6]     |ADCMPF2   |EADC Compare 2 Flag
     * |        |          |When the specific sample module EADC conversion result meets setting condition in EADC_CMP2 then this bit is set to 1.
     * |        |          |0 = Conversion result in EADC_DAT does not meet EADC_CMP2 register setting.
     * |        |          |1 = Conversion result in EADC_DAT meets EADC_CMP2 register setting.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[7]     |ADCMPF3   |EADC Compare 3 Flag
     * |        |          |When the specific sample module EADC conversion result meets setting condition in EADC_CMP3 then this bit is set to 1.
     * |        |          |0 = Conversion result in EADC_DAT does not meet EADC_CMP3 register setting.
     * |        |          |1 = Conversion result in EADC_DAT meets EADC_CMP3 register setting.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[8]     |ADOVIF0   |EADC ADINT0 Interrupt Flag Overrun
     * |        |          |0 = ADINT0 interrupt flag is not overwritten to 1.
     * |        |          |1 = ADINT0 interrupt flag is overwritten to 1.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[9]     |ADOVIF1   |EADC ADINT1 Interrupt Flag Overrun
     * |        |          |0 = ADINT1 interrupt flag is not overwritten to 1.
     * |        |          |1 = ADINT1 interrupt flag is overwritten to 1.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[10]    |ADOVIF2   |EADC ADINT2 Interrupt Flag Overrun
     * |        |          |0 = ADINT2 interrupt flag is not overwritten to 1.
     * |        |          |1 = ADINT2 interrupt flag is s overwritten to 1.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[11]    |ADOVIF3   |EADC ADINT3 Interrupt Flag Overrun
     * |        |          |0 = ADINT3 interrupt flag is not overwritten to 1.
     * |        |          |1 = ADINT3 interrupt flag is overwritten to 1.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[12]    |ADCMPO0   |EADC Compare 0 Output Status (Read Only)
     * |        |          |The 12 bits compare0 data CMPDAT (EADC_CMP0[27:16]) is used to compare with conversion result of specified sample module
     * |        |          |User can use it to monitor the external analog input pin voltage status.
     * |        |          |0 = Conversion result in EADC_DAT less than CMPDAT0 setting.
     * |        |          |1 = Conversion result in EADC_DAT greater than or equal CMPDAT0 setting.
     * |[13]    |ADCMPO1   |EADC Compare 1 Output Status (Read Only)
     * |        |          |The 12 bits compare1 data CMPDAT (EADC_CMP1[27:16]) is used to compare with conversion result of specified sample module
     * |        |          |User can use it to monitor the external analog input pin voltage status.
     * |        |          |0 = Conversion result in EADC_DAT less than CMPDAT1 setting.
     * |        |          |1 = Conversion result in EADC_DAT greater than or equal to CMPDAT1 setting.
     * |[14]    |ADCMPO2   |EADC Compare 2 Output Status (Read Only)
     * |        |          |The 12 bits compare2 data CMPDAT (EADC_CMP2[27:16]) is used to compare with conversion result of specified sample module
     * |        |          |User can use it to monitor the external analog input pin voltage status.
     * |        |          |0 = Conversion result in EADC_DAT less than CMPDAT2 setting.
     * |        |          |1 = Conversion result in EADC_DAT greater than or equal to CMPDAT2 setting.
     * |[15]    |ADCMPO3   |EADC Compare 3 Output Status (Read Only)
     * |        |          |The 12 bits compare3 data CMPDAT (EADC_CMP3[27:16]) is used to compare with conversion result of specified sample module
     * |        |          |User can use it to monitor the external analog input pin voltage status.
     * |        |          |0 = Conversion result in EADC_DAT less than CMPDAT3 setting.
     * |        |          |1 = Conversion result in EADC_DAT greater than or equal to CMPDAT3 setting.
     * |[20:16] |CHANNEL   |Current Conversion Channel (Read Only)
     * |        |          |This filed reflects EADC current conversion channel when BUSY=1.
     * |        |          |It is read only.
     * |        |          |00H = EADC_CH0.
     * |        |          |01H = EADC_CH1.
     * |        |          |02H = EADC_CH2.
     * |        |          |03H = EADC_CH3.
     * |        |          |04H = EADC_CH4.
     * |        |          |05H = EADC_CH5.
     * |        |          |06H = EADC_CH6.
     * |        |          |07H = EADC_CH7.
     * |        |          |08H = EADC_CH8.
     * |        |          |09H = EADC_CH9.
     * |        |          |0AH = EADC_CH10.
     * |        |          |0BH = EADC_CH11.
     * |        |          |0CH = EADC_CH12.
     * |        |          |0DH = EADC_CH13.
     * |        |          |0EH = EADC_CH14.
     * |        |          |0FH = EADC_CH15.
     * |        |          |10H = EADC_CH16.
     * |        |          |11H = EADC_CH17.
     * |        |          |12H = EADC_CH18.
     * |        |          |13H = EADC_CH19.
     * |        |          |14H = EADC_CH20.
     * |        |          |15H = EADC_CH21.
     * |        |          |16H = EADC_CH22.
     * |        |          |17H = EADC_CH23.
     * |        |          |18H = VBG.
     * |        |          |19H = VTEMP.
     * |        |          |1AH = VBAT/4.
     * |        |          |1BH = AVDD/4.
     * |[23]    |BUSY      |Busy/Idle (Read Only)
     * |        |          |0 = EADC is in idle state.
     * |        |          |1 = EADC is busy at conversion.
     * |        |          |Note: this flag will be high after 4*EADC_CLK cycles, when the trigger source is coming.
     * |[24]    |ADOVIF    |All EADC Interrupt Flag Overrun Bits Check (Read Only)
     * |        |          |n=0~3.
     * |        |          |0 = None of ADINT interrupt flag ADOVIFn, n=0~3 is overwritten to 1.
     * |        |          |1 = Any one of ADINT interrupt flag ADOVIFn, n=0~3 is overwritten to 1.
     * |        |          |Note: This bit will keep 1 when any ADOVIFn Flag is equal to 1.
     * |[25]    |STOVF     |for All EADC Sample Module Start of Conversion Overrun Flags Check (Read Only)
     * |        |          |n=0~27.
     * |        |          |0 = None of sample module event overrun flag SPOVF (EADC_OVSTS[n]) is set to 1.
     * |        |          |1 = Any one of sample module event overrun flag SPOVF (EADC_OVSTS[n]) is set to 1.
     * |        |          |Note: This bit will keep 1 when any SPOVFn Flag is equal to 1.
     * |[26]    |AVALID    |for All Sample Module EADC Result Data Register EADC_DAT Data Valid Flag Check (Read Only)
     * |        |          |n=0~27.
     * |        |          |0 = None of sample module data register valid flag VALID (EADC_DATn[17]) is set to 1.
     * |        |          |1 = Any one of sample module data register valid flag VALID (EADC_DATn[17]) is set to 1.
     * |        |          |Note: This bit will keep 1 when any VALIDn Flag is equal to 1.
     * |[27]    |AOV       |for All Sample Module EADC Result Data Register Overrun Flags Check (Read Only)
     * |        |          |n=0~27.
     * |        |          |0 = None of sample module data register overrun flag OV (EADC_DATn[16]) is set to 1.
     * |        |          |1 = Any one of sample module data register overrun flag OV (EADC_DATn[16]) is set to 1.
     * |        |          |Note: This bit will keep 1 when any OVn Flag is equal to 1.
     * @var EADC_T::STATUS3
     * Offset: 0xFC  EADC Status Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |CURSPL    |EADC Current Sample Module (Read Only)
     * |        |          |This register shows the current EADC is controlled by which sample module control logic modules.
     * |        |          |If the EADC is Idle, the bit filed will set to 0x3F.
     * |        |          |This is a read only register.
     * @var EADC_T::DDAT[4]
     * Offset: 0x100  EADC Double Data Register 0~3 for Sample Module 0~3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RESULT    |EADC Conversion Results
     * |        |          |This field contains 12 bits conversion results.
     * |        |          |The 12-bit EADC conversion result with unsigned format will be filled in RESULT [11:0] and zero will be filled in RESULT [15:12].
     * |[16]    |OV        |Overrun Flag
     * |        |          |0 = Double Data in RESULT (EADC_DDATn[11:0], n=0~3) is recent conversion result.
     * |        |          |1 = Double Data in RESULT (EADC_DDATn[11:0], n=0~3) is overwrite.
     * |        |          |If converted data in RESULT[15:0] has not been read before new conversion result is loaded to this register, OV is set to 1
     * |        |          |It is cleared by hardware after EADC_DDAT register is read.
     * |[17]    |VALID     |Valid Flag
     * |        |          |0 = Double data in RESULT (EADC_DDATn[11:0]) is not valid.
     * |        |          |1 = Double data in RESULT (EADC_DDATn[11:0]) is valid.
     * |        |          |This bit is set to 1 when corresponding sample module channel analog input conversion is completed and cleared by hardware after EADC_DDATn register is read
     * |        |          |(n=0~3).
     * @var EADC_T::CALCTL
     * Offset: 0x114  EADC Calibration Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CAL       |Calibration Enable Bit
     * |        |          |0 = Calibration Disabled.
     * |        |          |1 = Calibration Enabled.
     * |        |          |Note: This bit is hardware auto cleared when calibration is done
     * |[1]     |CALIE     |Calibration Interrupt Enable Bit
     * |        |          |0 = Calibration interrupt Disabled.
     * |        |          |1 = Calibration interrupt Enabled.
     * @var EADC_T::CALSR
     * Offset: 0x118  EADC Calibration Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[16]    |CALIF     |Calibration Finish Interrupt Flag
     * |        |          |If calibration is finished, this flag will be set to 1. It is cleared by writing 1 to it.
     * @var EADC_T::PDMACTL
     * Offset: 0x130  EADC PDMA Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[27:0]  |PDMATEN   |PDMA Transfer Enable Bit
     * |        |          |When EADC conversion is completed, the converted data is loaded into EADC_DATn (n: 0 ~ 27) register, user can enable this bit to generate a PDMA data transfer request.
     * |        |          |0 = PDMA data transfer Disabled.
     * |        |          |1 = PDMA data transfer Enabled.
     * |        |          |Note:When setting this bit field to 1, user must set ADCIENn (EADC_CTL[n+2], n=0~3) = 0 to disable interrupt.
     * @var EADC_T::MCTL1[19]
     * Offset: 0x140  EADC Sample Module 0~3 Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ALIGN     |Alignment Selection
     * |        |          |0 = The conversion result will be right aligned in data register.
     * |        |          |1 = The conversion result will be left aligned in data register.
     * |[1]     |AVG       |Average Mode Selection
     * |        |          |0 = Conversion results will be stored in data register without averaging.
     * |        |          |1 = Conversion results in data register will be averaged.
     * |        |          |Note: This bit needs to work with ACU (EADC_MnCTL1[7:4], n=0~23).
     * |[7:4]   |ACU       |Number of Accumulated Conversion Results Selection
     * |        |          |0000 = 1 conversion result will be accumulated.
     * |        |          |0001 = 2 conversion result will be accumulated.
     * |        |          |0010 = 4 conversion result will be accumulated.
     * |        |          |0011 = 8 conversion result will be accumulated.
     * |        |          |0100 = 16 conversion result will be accumulated.
     * |        |          |0101 = 32 conversion result will be accumulated.
     * |        |          |0110 = 64 conversion result will be accumulated.
     * |        |          |0111 = 128 conversion result will be accumulated.
     * |        |          |1000 = 256 conversion result will be accumulated.
     * |        |          |Others = Reserved.
     * |[17:16] |EXTSTDIV  |EADC Extended Sampling Time Clock Divider Selection
     * |        |          |Clock frequency for extending sampling time:
     * |        |          |00 = EADC_CLK/1.
     * |        |          |01 = EADC_CLK/2.
     * |        |          |10 = EADC_CLK/4.
     * |        |          |11 = EADC_CLK/16.
     * |[20]    |DBMEN     |Double Buffer Mode Enable Bit
     * |        |          |0 = Sample has one sample result register (default).
     * |        |          |1 = Sample has two sample result registers.
     * Offset: 0x150  EADC Sample Module 4~18 Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ALIGN     |Alignment Selection
     * |        |          |0 = The conversion result will be right aligned in data register.
     * |        |          |1 = The conversion result will be left aligned in data register.
     * |[1]     |AVG       |Average Mode Selection
     * |        |          |0 = Conversion results will be stored in data register without averaging.
     * |        |          |1 = Conversion results in data register will be averaged.
     * |        |          |Note: This bit needs to work with ACU (EADC_MnCTL1[7:4], n=0~23).
     * |[7:4]   |ACU       |Number of Accumulated Conversion Results Selection
     * |        |          |0000 = 1 conversion result will be accumulated.
     * |        |          |0001 = 2 conversion result will be accumulated.
     * |        |          |0010 = 4 conversion result will be accumulated.
     * |        |          |0011 = 8 conversion result will be accumulated.
     * |        |          |0100 = 16 conversion result will be accumulated.
     * |        |          |0101 = 32 conversion result will be accumulated.
     * |        |          |0110 = 64 conversion result will be accumulated.
     * |        |          |0111 = 128 conversion result will be accumulated.
     * |        |          |1000 = 256 conversion result will be accumulated.
     * |        |          |Others = Reserved.
     * |[17:16] |EXTSTDIV  |EADC Extended Sampling Time Clock Divider Selection
     * |        |          |Clock frequency for extending sampling time:
     * |        |          |00 = EADC_CLK/1.
     * |        |          |01 = EADC_CLK/2.
     * |        |          |10 = EADC_CLK/4.
     * |        |          |11 = EADC_CLK/16.
     * @var EADC_T::DAT19[9]
     * Offset: 0x200  EADC Data Register 19~27 for Sample Module 19~27
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RESULT    |EADC Conversion Result
     * |        |          |This field contains 12 bits conversion result
     * |        |          |The 12-bit EADC conversion result with unsigned format will be filled in RESULT[11:0] and zero will be filled in RESULT[15:12].
     * |        |          |Note: When operating in oversampling mode, RESULT[15:0] can represent oversampling results.
     * |[16]    |OV        |Overrun Flag
     * |        |          |If converted data in RESULT[11:0] has not been read before new conversion result is loaded to this register, OV is set to 1.
     * |        |          |0 = Data in RESULT[11:0] is recent conversion result.
     * |        |          |1 = Data in RESULT[11:0] is overwrite.
     * |        |          |Note: It is cleared by hardware after EADC_DAT register is read.
     * |[17]    |VALID     |Valid Flag
     * |        |          |This bit is set to 1 when corresponding sample module channel analog input conversion is completed and cleared by hardware after EADC_DAT register is read.
     * |        |          |0 = Data in RESULT[11:0] bits is not valid.
     * |        |          |1 = Data in RESULT[11:0] bits is valid.
     * @var EADC_T::SCTL19[9]
     * Offset: 0x230  EADC Sample Module 19~27 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |CHSEL     |EADC Sample Module Channel Selection
     * |        |          |00H = EADC_CH0.
     * |        |          |01H = EADC_CH1.
     * |        |          |02H = EADC_CH2.
     * |        |          |03H = EADC_CH3.
     * |        |          |04H = EADC_CH4.
     * |        |          |05H = EADC_CH5.
     * |        |          |06H = EADC_CH6.
     * |        |          |07H = EADC_CH7.
     * |        |          |08H = EADC_CH8.
     * |        |          |09H = EADC_CH9.
     * |        |          |0AH = EADC_CH10.
     * |        |          |0BH = EADC_CH11.
     * |        |          |0CH = EADC_CH12.
     * |        |          |0DH = EADC_CH13.
     * |        |          |0EH = EADC_CH14.
     * |        |          |0FH = EADC_CH15.
     * |        |          |10H = EADC_CH16.
     * |        |          |11H = EADC_CH17.
     * |        |          |12H = EADC_CH18.
     * |        |          |13H = EADC_CH19.
     * |        |          |14H = EADC_CH20.
     * |        |          |15H = EADC_CH21.
     * |        |          |16H = EADC_CH22.
     * |        |          |17H = EADC_CH23.
     * |        |          |Others = Reserved.
     * |[5]     |INTPOS    |Interrupt Flag Position Select
     * |        |          |0 = Set ADIFn (EADC_STATUS2[n], n=0~3) at EADC end of conversion.
     * |        |          |1 = Set ADIFn (EADC_STATUS2[n], n=0~3) at EADC start of conversion.
     * |[7:6]   |TRGDLDIV  |EADC Sample Module Start of Conversion Trigger Delay Clock Divider Selection
     * |        |          |Trigger delay clock frequency:
     * |        |          |00 = EADC_CLK/1.
     * |        |          |01 = EADC_CLK/2.
     * |        |          |10 = EADC_CLK/4.
     * |        |          |11 = EADC_CLK/16.
     * |[15:8]  |TRGDLCNT  |EADC Sample Module Start of Conversion Trigger Delay Time
     * |        |          |Trigger delay time = TRGDLCNT x EADC_CLK period x n (n=1,2,4,16 from TRGDLDIV setting).
     * |[21:16] |TRGSEL    |EADC Sample Module Start of Conversion Trigger Source Selection
     * |        |          |0H = Disable trigger.
     * |        |          |1H = External trigger from EADC0_ST pin input.
     * |        |          |2H = EADC ADINT0 interrupt EOC pulse trigger.
     * |        |          |3H = EADC ADINT1 interrupt EOC pulse trigger.
     * |        |          |4H = Timer0 overflow pulse trigger.
     * |        |          |5H = Timer1 overflow pulse trigger.
     * |        |          |6H = Timer2 overflow pulse trigger.
     * |        |          |7H = Timer3 overflow pulse trigger.
     * |        |          |8H = EPWM0TG0.
     * |        |          |9H = EPWM0TG1.
     * |        |          |AH = EPWM0TG2.
     * |        |          |BH = EPWM0TG3.
     * |        |          |CH = EPWM0TG4.
     * |        |          |DH = EPWM0TG5.
     * |        |          |EH = EPWM1TG0.
     * |        |          |FH = EPWM1TG1.
     * |        |          |10H = EPWM1TG2.
     * |        |          |11H = EPWM1TG3.
     * |        |          |12H = EPWM1TG4.
     * |        |          |13H = EPWM1TG5.
     * |        |          |20H = BPWM0TG.
     * |        |          |21H = BPWM1TG.
     * |        |          |22H = ACMP0_INT.
     * |        |          |23H = ACMP1_INT.
     * |        |          |24H = ACMP2_INT.
     * |        |          |25H = ACMP3_INT.
     * |        |          |other = Reserved.
     * |[22]    |EXTREN    |EADC External Trigger Rising Edge Enable Bit
     * |        |          |0 = Rising edge Disabled when EADC selects EADC_ST(x=0, 1) as trigger source.
     * |        |          |1 = Rising edge Enabled when EADC selects EADC_ST(x=0, 1) as trigger source.
     * |[23]    |EXTFEN    |EADC External Trigger Falling Edge Enable Bit
     * |        |          |0 = Falling edge Disabled when EADC selects EADC_ST(x=0, 1) as trigger source.
     * |        |          |1 = Falling edge Enabled when EADC selects EADC_ST(x=0, 1) as trigger source.
     * |[31:24] |EXTSMPT   |EADC Sampling Time Extend
     * |        |          |When EADC converting at high conversion rate, the sampling time of analog input voltage may not be enough if input channel loading is heavy, and software can extend EADC sampling time after trigger source is coming to get enough sampling time.
     * |        |          |Extended Sampling Time = (EXTSMPT x EADC_CLK period x n)+2 [n=1,2,4,16 from EXTSTDIV setting].
     * Offset: 0x244  EADC Sample Module 24~27 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:24] |EXTSMPT   |ADC Sampling Time Extend
     * |        |          |When EADC converting at high conversion rate, the sampling time of analog input voltage may not be enough if input channel loading is heavy, and software can extend EADC sampling time after trigger source is coming to get enough sampling time.
     * |        |          |Extended Sampling Time = (EXTSMPT x EADC_CLK period x n)+2 [n=1,2,4,16 from EXTSTDIV setting].
    * @var EADC_T::M19CTL1[9]
     * Offset: 0x260  EADC Sample Module 19~23 Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ALIGN     |Alignment Selection
     * |        |          |0 = The conversion result will be right aligned in data register.
     * |        |          |1 = The conversion result will be left aligned in data register.
     * |[1]     |AVG       |Average Mode Selection
     * |        |          |0 = Conversion results will be stored in data register without averaging.
     * |        |          |1 = Conversion results in data register will be averaged.
     * |        |          |Note: This bit needs to work with ACU (EADC_MnCTL1[7:4], n=0~23).
     * |[7:4]   |ACU       |Number of Accumulated Conversion Results Selection
     * |        |          |0000 = 1 conversion result will be accumulated.
     * |        |          |0001 = 2 conversion result will be accumulated.
     * |        |          |0010 = 4 conversion result will be accumulated.
     * |        |          |0011 = 8 conversion result will be accumulated.
     * |        |          |0100 = 16 conversion result will be accumulated.
     * |        |          |0101 = 32 conversion result will be accumulated.
     * |        |          |0110 = 64 conversion result will be accumulated.
     * |        |          |0111 = 128 conversion result will be accumulated.
     * |        |          |1000 = 256 conversion result will be accumulated.
     * |        |          |Others = Reserved.
     * |[17:16] |EXTSTDIV  |EADC Extended Sampling Time Clock Divider Selection
     * |        |          |Clock frequency for extending sampling time:
     * |        |          |00 = EADC_CLK/1.
     * |        |          |01 = EADC_CLK/2.
     * |        |          |10 = EADC_CLK/4.
     * |        |          |11 = EADC_CLK/16.
     * Offset: 0x274  EADC Sample Module 24~27 Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[17:16] |EXTSTDIV  |EADC Extended Sampling Time Clock Divider Selection
     * |        |          |Clock frequency for extending sampling time:
     * |        |          |00 = EADC_CLK/1.
     * |        |          |01 = EADC_CLK/2.
     * |        |          |10 = EADC_CLK/4.
     * |        |          |11 = EADC_CLK/16.
     */
    __I  uint32_t DAT[19];               /*!< [0x0000-0x0048] EADC Data Register 0~18 for Sample Module 0~18            */
    __I  uint32_t CURDAT;                /*!< [0x004c] EADC PDMA Current Transfer Data Register                         */
    __IO uint32_t CTL;                   /*!< [0x0050] EADC Control Register                                            */
    __O  uint32_t SWTRG;                 /*!< [0x0054] EADC Sample Module Software Start Register                       */
    __IO uint32_t PENDSTS;               /*!< [0x0058] EADC Start of Conversion Pending Flag Register                   */
    __IO uint32_t OVSTS;                 /*!< [0x005c] EADC Sample Module Start of Conversion Overrun Flag Register     */
    __IO uint32_t CTL1;                  /*!< [0x0060] EADC Control1 Register                                           */
    __I  uint32_t RESERVE0[7];
    __IO uint32_t SCTL[19];              /*!< [0x0080-0x00C8] EADC Sample Module 0~18 Control Register                  */
    __I  uint32_t RESERVE1[1];
    __IO uint32_t INTSRC[4];             /*!< [0x00D0-0x00DC] EADC Interrupt 0~3 Source Enable Control Register.        */
    __IO uint32_t CMP[4];                /*!< [0x00E0-0x00EC] EADC Result Compare Register 0~3                          */
    __I  uint32_t STATUS0;               /*!< [0x00f0] EADC Status Register 0                                           */
    __I  uint32_t STATUS1;               /*!< [0x00f4] EADC Status Register 1                                           */
    __IO uint32_t STATUS2;               /*!< [0x00f8] EADC Status Register 2                                           */
    __I  uint32_t STATUS3;               /*!< [0x00fc] EADC Status Register 3                                           */
    __I  uint32_t DDAT[4];               /*!< [0x0100-0x010C] EADC Double Data Register 0~3 for Sample Module 0~3       */
    __I  uint32_t RESERVE2[1];
    __IO uint32_t CALCTL;                /*!< [0x0114] EADC Calibration Control Register                                */
    __IO uint32_t CALSR;                 /*!< [0x0118] EADC Calibration Status Register                                 */
    __I  uint32_t RESERVE3[5];
    __IO uint32_t PDMACTL;               /*!< [0x0130] EADC PDMA Control Register                                       */
    __I  uint32_t RESERVE4[3];
    __IO uint32_t MCTL1[19];             /*!< [0x0140-0x0188] EADC Sample Module 0~18 Control Register 1                */
    __I  uint32_t RESERVE5[29];
    __I  uint32_t DAT19[9];              /*!< [0x0200-0x0220] EADC Data Register 19~27 for Sample Module 19~27          */
    __I  uint32_t RESERVE6[3];
    __IO uint32_t SCTL19[9];             /*!< [0x0230-0x0250] EADC Sample Module 19~27 Control Register                 */
    __I  uint32_t RESERVE7[3];
    __IO uint32_t M19CTL1[9];            /*!< [0x0260-0x0280] EADC Sample Module 19~27 Control Register 1               */

} EADC_T;

/**
    @addtogroup EADC_CONST EADC Bit Field Definition
    Constant Definitions for EADC Controller
@{ */


#define EADC_DAT_RESULT_Pos              (0)                                               /*!< EADC_T::DAT: RESULT Position           */
#define EADC_DAT_RESULT_Msk              (0xfffful << EADC_DAT_RESULT_Pos)                 /*!< EADC_T::DAT: RESULT Mask               */

#define EADC_DAT_OV_Pos                  (16)                                              /*!< EADC_T::DAT: OV Position               */
#define EADC_DAT_OV_Msk                  (0x1ul << EADC_DAT_OV_Pos)                        /*!< EADC_T::DAT: OV Mask                   */

#define EADC_DAT_VALID_Pos               (17)                                              /*!< EADC_T::DAT: VALID Position            */
#define EADC_DAT_VALID_Msk               (0x1ul << EADC_DAT_VALID_Pos)                     /*!< EADC_T::DAT: VALID Mask                */

#define EADC_CURDAT_CURDAT_Pos           (0)                                               /*!< EADC_T::CURDAT: CURDAT Position        */
#define EADC_CURDAT_CURDAT_Msk           (0xffffffful << EADC_CURDAT_CURDAT_Pos)           /*!< EADC_T::CURDAT: CURDAT Mask            */

#define EADC_CTL_ADCEN_Pos               (0)                                               /*!< EADC_T::CTL: ADCEN Position            */
#define EADC_CTL_ADCEN_Msk               (0x1ul << EADC_CTL_ADCEN_Pos)                     /*!< EADC_T::CTL: ADCEN Mask                */

#define EADC_CTL_ADCRST_Pos              (1)                                               /*!< EADC_T::CTL: ADCRST Position           */
#define EADC_CTL_ADCRST_Msk              (0x1ul << EADC_CTL_ADCRST_Pos)                    /*!< EADC_T::CTL: ADCRST Mask               */

#define EADC_CTL_ADCIEN0_Pos             (2)                                               /*!< EADC_T::CTL: ADCIEN0 Position          */
#define EADC_CTL_ADCIEN0_Msk             (0x1ul << EADC_CTL_ADCIEN0_Pos)                   /*!< EADC_T::CTL: ADCIEN0 Mask              */

#define EADC_CTL_ADCIEN1_Pos             (3)                                               /*!< EADC_T::CTL: ADCIEN1 Position          */
#define EADC_CTL_ADCIEN1_Msk             (0x1ul << EADC_CTL_ADCIEN1_Pos)                   /*!< EADC_T::CTL: ADCIEN1 Mask              */

#define EADC_CTL_ADCIEN2_Pos             (4)                                               /*!< EADC_T::CTL: ADCIEN2 Position          */
#define EADC_CTL_ADCIEN2_Msk             (0x1ul << EADC_CTL_ADCIEN2_Pos)                   /*!< EADC_T::CTL: ADCIEN2 Mask              */

#define EADC_CTL_ADCIEN3_Pos             (5)                                               /*!< EADC_T::CTL: ADCIEN3 Position          */
#define EADC_CTL_ADCIEN3_Msk             (0x1ul << EADC_CTL_ADCIEN3_Pos)                   /*!< EADC_T::CTL: ADCIEN3 Mask              */

#define EADC_CTL_DIFFEN_Pos              (8)                                               /*!< EADC_T::CTL: DIFFEN Position           */
#define EADC_CTL_DIFFEN_Msk              (0x1ul << EADC_CTL_DIFFEN_Pos)                    /*!< EADC_T::CTL: DIFFEN Mask               */

#define EADC_CTL_DMOF_Pos                (9)                                               /*!< EADC_T::CTL: DMOF Position             */
#define EADC_CTL_DMOF_Msk                (0x1ul << EADC_CTL_DMOF_Pos)                      /*!< EADC_T::CTL: DMOF Mask                 */

#define EADC_SWTRG_SWTRG_Pos             (0)                                               /*!< EADC_T::SWTRG: SWTRG Position          */
#define EADC_SWTRG_SWTRG_Msk             (0xffffffful << EADC_SWTRG_SWTRG_Pos)             /*!< EADC_T::SWTRG: SWTRG Mask              */

#define EADC_PENDSTS_STPF_Pos            (0)                                               /*!< EADC_T::PENDSTS: STPF Position         */
#define EADC_PENDSTS_STPF_Msk            (0xffffffful << EADC_PENDSTS_STPF_Pos)            /*!< EADC_T::PENDSTS: STPF Mask             */

#define EADC_OVSTS_SPOVF_Pos             (0)                                               /*!< EADC_T::OVSTS: SPOVF Position          */
#define EADC_OVSTS_SPOVF_Msk             (0xffffffful << EADC_OVSTS_SPOVF_Pos)             /*!< EADC_T::OVSTS: SPOVF Mask              */

#define EADC_CTL1_PRECHEN_Pos            (0)                                               /*!< EADC_T::CTL1: PRECHEN Position         */
#define EADC_CTL1_PRECHEN_Msk            (0x1ul << EADC_CTL1_PRECHEN_Pos)                  /*!< EADC_T::CTL1: PRECHEN Mask             */

#define EADC_CTL1_DISCHEN_Pos            (1)                                               /*!< EADC_T::CTL1: DISCHEN Position         */
#define EADC_CTL1_DISCHEN_Msk            (0x1ul << EADC_CTL1_DISCHEN_Pos)                  /*!< EADC_T::CTL1: DISCHEN Mask             */

#define EADC_CTL1_RESSEL_Pos             (4)                                               /*!< EADC_T::CTL1: RESSEL Position          */
#define EADC_CTL1_RESSEL_Msk             (0x3ul << EADC_CTL1_RESSEL_Pos)                   /*!< EADC_T::CTL1: RESSEL Mask              */

#define EADC_CTL1_FDETCHEN_Pos           (8)                                               /*!< EADC_T::CTL1: FDETCHEN Position        */
#define EADC_CTL1_FDETCHEN_Msk           (0x1ul << EADC_CTL1_FDETCHEN_Pos)                 /*!< EADC_T::CTL1: FDETCHEN Mask            */

#define EADC_CTL1_CMP0TRG_Pos            (20)                                              /*!< EADC_T::CTL1: CMP0TRG Position         */
#define EADC_CTL1_CMP0TRG_Msk            (0x1ul << EADC_CTL1_CMP0TRG_Pos)                  /*!< EADC_T::CTL1: CMP0TRG Mask             */

#define EADC_CTL1_CMP1TRG_Pos            (21)                                              /*!< EADC_T::CTL1: CMP1TRG Position         */
#define EADC_CTL1_CMP1TRG_Msk            (0x1ul << EADC_CTL1_CMP1TRG_Pos)                  /*!< EADC_T::CTL1: CMP1TRG Mask             */

#define EADC_CTL1_CMP2TRG_Pos            (22)                                              /*!< EADC_T::CTL1: CMP2TRG Position         */
#define EADC_CTL1_CMP2TRG_Msk            (0x1ul << EADC_CTL1_CMP2TRG_Pos)                  /*!< EADC_T::CTL1: CMP2TRG Mask             */

#define EADC_CTL1_CMP3TRG_Pos            (23)                                              /*!< EADC_T::CTL1: CMP3TRG Position         */
#define EADC_CTL1_CMP3TRG_Msk            (0x1ul << EADC_CTL1_CMP3TRG_Pos)                  /*!< EADC_T::CTL1: CMP3TRG Mask             */

#define EADC_SCTL_CHSEL_Pos              (0)                                               /*!< EADC_T::SCTL: CHSEL Position           */
#define EADC_SCTL_CHSEL_Msk              (0x1ful << EADC_SCTL_CHSEL_Pos)                   /*!< EADC_T::SCTL: CHSEL Mask               */

#define EADC_SCTL_INTPOS_Pos             (5)                                               /*!< EADC_T::SCTL: INTPOS Position          */
#define EADC_SCTL_INTPOS_Msk             (0x1ul << EADC_SCTL_INTPOS_Pos)                   /*!< EADC_T::SCTL: INTPOS Mask              */

#define EADC_SCTL_TRGDLDIV_Pos           (6)                                               /*!< EADC_T::SCTL: TRGDLDIV Position        */
#define EADC_SCTL_TRGDLDIV_Msk           (0x3ul << EADC_SCTL_TRGDLDIV_Pos)                 /*!< EADC_T::SCTL: TRGDLYDIV Mask           */

#define EADC_SCTL_TRGDLCNT_Pos           (8)                                               /*!< EADC_T::SCTL: TRGDLCNT Position        */
#define EADC_SCTL_TRGDLCNT_Msk           (0xfful << EADC_SCTL_TRGDLCNT_Pos)                /*!< EADC_T::SCTL: TRGDLYCNT Mask           */

#define EADC_SCTL_TRGSEL_Pos             (16)                                              /*!< EADC_T::SCTL: TRGSEL Position          */
#define EADC_SCTL_TRGSEL_Msk             (0x3ful << EADC_SCTL_TRGSEL_Pos)                  /*!< EADC_T::SCTL: TRGSEL Mask              */

#define EADC_SCTL_EXTREN_Pos             (22)                                              /*!< EADC_T::SCTL: EXTREN Position          */
#define EADC_SCTL_EXTREN_Msk             (0x1ul << EADC_SCTL_EXTREN_Pos)                   /*!< EADC_T::SCTL: EXTREN Mask              */

#define EADC_SCTL_EXTFEN_Pos             (23)                                              /*!< EADC_T::SCTL: EXTFEN Position          */
#define EADC_SCTL_EXTFEN_Msk             (0x1ul << EADC_SCTL_EXTFEN_Pos)                   /*!< EADC_T::SCTL: EXTFEN Mask              */

#define EADC_SCTL_EXTSMPT_Pos            (24)                                              /*!< EADC_T::SCTL: EXTSMPT Position         */
#define EADC_SCTL_EXTSMPT_Msk            (0xfful << EADC_SCTL_EXTSMPT_Pos)                 /*!< EADC_T::SCTL: EXTSMPT Mask             */

#define EADC_INTSRC_SPLIE0_Pos           (0)                                               /*!< EADC_T::INTSRC: SPLIE0 Position        */
#define EADC_INTSRC_SPLIE0_Msk           (0x1ul << EADC_INTSRC_SPLIE0_Pos)                 /*!< EADC_T::INTSRC: SPLIE0 Mask            */

#define EADC_INTSRC_SPLIE1_Pos           (1)                                               /*!< EADC_T::INTSRC: SPLIE1 Position        */
#define EADC_INTSRC_SPLIE1_Msk           (0x1ul << EADC_INTSRC_SPLIE1_Pos)                 /*!< EADC_T::INTSRC: SPLIE1 Mask            */

#define EADC_INTSRC_SPLIE2_Pos           (2)                                               /*!< EADC_T::INTSRC: SPLIE2 Position        */
#define EADC_INTSRC_SPLIE2_Msk           (0x1ul << EADC_INTSRC_SPLIE2_Pos)                 /*!< EADC_T::INTSRC: SPLIE2 Mask            */

#define EADC_INTSRC_SPLIE3_Pos           (3)                                               /*!< EADC_T::INTSRC: SPLIE3 Position        */
#define EADC_INTSRC_SPLIE3_Msk           (0x1ul << EADC_INTSRC_SPLIE3_Pos)                 /*!< EADC_T::INTSRC: SPLIE3 Mask            */

#define EADC_INTSRC_SPLIE4_Pos           (4)                                               /*!< EADC_T::INTSRC: SPLIE4 Position        */
#define EADC_INTSRC_SPLIE4_Msk           (0x1ul << EADC_INTSRC_SPLIE4_Pos)                 /*!< EADC_T::INTSRC: SPLIE4 Mask            */

#define EADC_INTSRC_SPLIE5_Pos           (5)                                               /*!< EADC_T::INTSRC: SPLIE5 Position        */
#define EADC_INTSRC_SPLIE5_Msk           (0x1ul << EADC_INTSRC_SPLIE5_Pos)                 /*!< EADC_T::INTSRC: SPLIE5 Mask            */

#define EADC_INTSRC_SPLIE6_Pos           (6)                                               /*!< EADC_T::INTSRC: SPLIE6 Position        */
#define EADC_INTSRC_SPLIE6_Msk           (0x1ul << EADC_INTSRC_SPLIE6_Pos)                 /*!< EADC_T::INTSRC: SPLIE6 Mask            */

#define EADC_INTSRC_SPLIE7_Pos           (7)                                               /*!< EADC_T::INTSRC: SPLIE7 Position        */
#define EADC_INTSRC_SPLIE7_Msk           (0x1ul << EADC_INTSRC_SPLIE7_Pos)                 /*!< EADC_T::INTSRC: SPLIE7 Mask            */

#define EADC_INTSRC_SPLIE8_Pos           (8)                                               /*!< EADC_T::INTSRC: SPLIE8 Position        */
#define EADC_INTSRC_SPLIE8_Msk           (0x1ul << EADC_INTSRC_SPLIE8_Pos)                 /*!< EADC_T::INTSRC: SPLIE8 Mask            */

#define EADC_INTSRC_SPLIE9_Pos           (9)                                               /*!< EADC_T::INTSRC: SPLIE9 Position        */
#define EADC_INTSRC_SPLIE9_Msk           (0x1ul << EADC_INTSRC_SPLIE9_Pos)                 /*!< EADC_T::INTSRC: SPLIE9 Mask            */

#define EADC_INTSRC_SPLIE10_Pos          (10)                                              /*!< EADC_T::INTSRC: SPLIE10 Position       */
#define EADC_INTSRC_SPLIE10_Msk          (0x1ul << EADC_INTSRC_SPLIE10_Pos)                /*!< EADC_T::INTSRC: SPLIE10 Mask           */

#define EADC_INTSRC_SPLIE11_Pos          (11)                                              /*!< EADC_T::INTSRC: SPLIE11 Position       */
#define EADC_INTSRC_SPLIE11_Msk          (0x1ul << EADC_INTSRC_SPLIE11_Pos)                /*!< EADC_T::INTSRC: SPLIE11 Mask           */

#define EADC_INTSRC_SPLIE12_Pos          (12)                                              /*!< EADC_T::INTSRC: SPLIE12 Position       */
#define EADC_INTSRC_SPLIE12_Msk          (0x1ul << EADC_INTSRC_SPLIE12_Pos)                /*!< EADC_T::INTSRC: SPLIE12 Mask           */

#define EADC_INTSRC_SPLIE13_Pos          (13)                                              /*!< EADC_T::INTSRC: SPLIE13 Position       */
#define EADC_INTSRC_SPLIE13_Msk          (0x1ul << EADC_INTSRC_SPLIE13_Pos)                /*!< EADC_T::INTSRC: SPLIE13 Mask           */

#define EADC_INTSRC_SPLIE14_Pos          (14)                                              /*!< EADC_T::INTSRC: SPLIE14 Position       */
#define EADC_INTSRC_SPLIE14_Msk          (0x1ul << EADC_INTSRC_SPLIE14_Pos)                /*!< EADC_T::INTSRC: SPLIE14 Mask           */

#define EADC_INTSRC_SPLIE15_Pos          (15)                                              /*!< EADC_T::INTSRC: SPLIE15 Position       */
#define EADC_INTSRC_SPLIE15_Msk          (0x1ul << EADC_INTSRC_SPLIE15_Pos)                /*!< EADC_T::INTSRC: SPLIE15 Mask           */

#define EADC_INTSRC_SPLIE16_Pos          (16)                                              /*!< EADC_T::INTSRC: SPLIE16 Position       */
#define EADC_INTSRC_SPLIE16_Msk          (0x1ul << EADC_INTSRC_SPLIE16_Pos)                /*!< EADC_T::INTSRC: SPLIE16 Mask           */

#define EADC_INTSRC_SPLIE17_Pos          (17)                                              /*!< EADC_T::INTSRC: SPLIE17 Position       */
#define EADC_INTSRC_SPLIE17_Msk          (0x1ul << EADC_INTSRC_SPLIE17_Pos)                /*!< EADC_T::INTSRC: SPLIE17 Mask           */

#define EADC_INTSRC_SPLIE18_Pos          (18)                                              /*!< EADC_T::INTSRC: SPLIE18 Position       */
#define EADC_INTSRC_SPLIE18_Msk          (0x1ul << EADC_INTSRC_SPLIE18_Pos)                /*!< EADC_T::INTSRC: SPLIE18 Mask           */

#define EADC_INTSRC_SPLIE19_Pos          (19)                                              /*!< EADC_T::INTSRC: SPLIE19 Position       */
#define EADC_INTSRC_SPLIE19_Msk          (0x1ul << EADC_INTSRC_SPLIE19_Pos)                /*!< EADC_T::INTSRC: SPLIE19 Mask           */

#define EADC_INTSRC_SPLIE20_Pos          (20)                                              /*!< EADC_T::INTSRC: SPLIE20 Position       */
#define EADC_INTSRC_SPLIE20_Msk          (0x1ul << EADC_INTSRC_SPLIE20_Pos)                /*!< EADC_T::INTSRC: SPLIE20 Mask           */

#define EADC_INTSRC_SPLIE21_Pos          (21)                                              /*!< EADC_T::INTSRC: SPLIE21 Position       */
#define EADC_INTSRC_SPLIE21_Msk          (0x1ul << EADC_INTSRC_SPLIE21_Pos)                /*!< EADC_T::INTSRC: SPLIE21 Mask           */

#define EADC_INTSRC_SPLIE22_Pos          (22)                                              /*!< EADC_T::INTSRC: SPLIE22 Position       */
#define EADC_INTSRC_SPLIE22_Msk          (0x1ul << EADC_INTSRC_SPLIE22_Pos)                /*!< EADC_T::INTSRC: SPLIE22 Mask           */

#define EADC_INTSRC_SPLIE23_Pos          (23)                                              /*!< EADC_T::INTSRC: SPLIE23 Position       */
#define EADC_INTSRC_SPLIE23_Msk          (0x1ul << EADC_INTSRC_SPLIE23_Pos)                /*!< EADC_T::INTSRC: SPLIE23 Mask           */

#define EADC_INTSRC_SPLIE24_Pos          (24)                                              /*!< EADC_T::INTSRC: SPLIE24 Position       */
#define EADC_INTSRC_SPLIE24_Msk          (0x1ul << EADC_INTSRC_SPLIE24_Pos)                /*!< EADC_T::INTSRC: SPLIE24 Mask           */

#define EADC_INTSRC_SPLIE25_Pos          (25)                                              /*!< EADC_T::INTSRC: SPLIE25 Position       */
#define EADC_INTSRC_SPLIE25_Msk          (0x1ul << EADC_INTSRC_SPLIE25_Pos)                /*!< EADC_T::INTSRC: SPLIE25 Mask           */

#define EADC_INTSRC_SPLIE26_Pos          (26)                                              /*!< EADC_T::INTSRC: SPLIE26 Position       */
#define EADC_INTSRC_SPLIE26_Msk          (0x1ul << EADC_INTSRC_SPLIE26_Pos)                /*!< EADC_T::INTSRC: SPLIE26 Mask           */

#define EADC_INTSRC_SPLIE27_Pos          (27)                                              /*!< EADC_T::INTSRC: SPLIE27 Position       */
#define EADC_INTSRC_SPLIE27_Msk          (0x1ul << EADC_INTSRC_SPLIE27_Pos)                /*!< EADC_T::INTSRC: SPLIE27 Mask           */

#define EADC_CMP_ADCMPEN_Pos             (0)                                               /*!< EADC_T::CMP: ADCMPEN Position          */
#define EADC_CMP_ADCMPEN_Msk             (0x1ul << EADC_CMP_ADCMPEN_Pos)                   /*!< EADC_T::CMP: ADCMPEN Mask              */

#define EADC_CMP_ADCMPIE_Pos             (1)                                               /*!< EADC_T::CMP: ADCMPIE Position          */
#define EADC_CMP_ADCMPIE_Msk             (0x1ul << EADC_CMP_ADCMPIE_Pos)                   /*!< EADC_T::CMP: ADCMPIE Mask              */

#define EADC_CMP_CMPCOND_Pos             (2)                                               /*!< EADC_T::CMP: CMPCOND Position          */
#define EADC_CMP_CMPCOND_Msk             (0x1ul << EADC_CMP_CMPCOND_Pos)                   /*!< EADC_T::CMP: CMPCOND Mask              */

#define EADC_CMP_CMPSPL_Pos              (3)                                               /*!< EADC_T::CMP: CMPSPL Position           */
#define EADC_CMP_CMPSPL_Msk              (0x1ful << EADC_CMP_CMPSPL_Pos)                   /*!< EADC_T::CMP: CMPSPL Mask               */

#define EADC_CMP_CMPMCNT_Pos             (8)                                               /*!< EADC_T::CMP: CMPMCNT Position          */
#define EADC_CMP_CMPMCNT_Msk             (0xful << EADC_CMP_CMPMCNT_Pos)                   /*!< EADC_T::CMP: CMPMCNT Mask              */

#define EADC_CMP_CMPWEN_Pos              (15)                                              /*!< EADC_T::CMP: CMPWEN Position           */
#define EADC_CMP_CMPWEN_Msk              (0x1ul << EADC_CMP_CMPWEN_Pos)                    /*!< EADC_T::CMP: CMPWEN Mask               */

#define EADC_CMP_CMPDAT_Pos              (16)                                              /*!< EADC_T::CMP: CMPDAT Position           */
#define EADC_CMP_CMPDAT_Msk              (0xffful << EADC_CMP_CMPDAT_Pos)                  /*!< EADC_T::CMP: CMPDAT Mask               */

#define EADC_STATUS0_VALID_Pos           (0)                                               /*!< EADC_T::STATUS0: VALID Position        */
#define EADC_STATUS0_VALID_Msk           (0xfffful << EADC_STATUS0_VALID_Pos)              /*!< EADC_T::STATUS0: VALID Mask            */

#define EADC_STATUS0_OV_Pos              (16)                                              /*!< EADC_T::STATUS0: OV Position           */
#define EADC_STATUS0_OV_Msk              (0xfffful << EADC_STATUS0_OV_Pos)                 /*!< EADC_T::STATUS0: OV Mask               */

#define EADC_STATUS1_VALID_Pos           (0)                                               /*!< EADC_T::STATUS1: VALID Position        */
#define EADC_STATUS1_VALID_Msk           (0xffful << EADC_STATUS1_VALID_Pos)               /*!< EADC_T::STATUS1: VALID Mask            */

#define EADC_STATUS1_OV_Pos              (16)                                              /*!< EADC_T::STATUS1: OV Position           */
#define EADC_STATUS1_OV_Msk              (0xffful << EADC_STATUS1_OV_Pos)                  /*!< EADC_T::STATUS1: OV Mask               */

#define EADC_STATUS2_ADIF0_Pos           (0)                                               /*!< EADC_T::STATUS2: ADIF0 Position        */
#define EADC_STATUS2_ADIF0_Msk           (0x1ul << EADC_STATUS2_ADIF0_Pos)                 /*!< EADC_T::STATUS2: ADIF0 Mask            */

#define EADC_STATUS2_ADIF1_Pos           (1)                                               /*!< EADC_T::STATUS2: ADIF1 Position        */
#define EADC_STATUS2_ADIF1_Msk           (0x1ul << EADC_STATUS2_ADIF1_Pos)                 /*!< EADC_T::STATUS2: ADIF1 Mask            */

#define EADC_STATUS2_ADIF2_Pos           (2)                                               /*!< EADC_T::STATUS2: ADIF2 Position        */
#define EADC_STATUS2_ADIF2_Msk           (0x1ul << EADC_STATUS2_ADIF2_Pos)                 /*!< EADC_T::STATUS2: ADIF2 Mask            */

#define EADC_STATUS2_ADIF3_Pos           (3)                                               /*!< EADC_T::STATUS2: ADIF3 Position        */
#define EADC_STATUS2_ADIF3_Msk           (0x1ul << EADC_STATUS2_ADIF3_Pos)                 /*!< EADC_T::STATUS2: ADIF3 Mask            */

#define EADC_STATUS2_ADCMPF0_Pos         (4)                                               /*!< EADC_T::STATUS2: ADCMPF0 Position      */
#define EADC_STATUS2_ADCMPF0_Msk         (0x1ul << EADC_STATUS2_ADCMPF0_Pos)               /*!< EADC_T::STATUS2: ADCMPF0 Mask          */

#define EADC_STATUS2_ADCMPF1_Pos         (5)                                               /*!< EADC_T::STATUS2: ADCMPF1 Position      */
#define EADC_STATUS2_ADCMPF1_Msk         (0x1ul << EADC_STATUS2_ADCMPF1_Pos)               /*!< EADC_T::STATUS2: ADCMPF1 Mask          */

#define EADC_STATUS2_ADCMPF2_Pos         (6)                                               /*!< EADC_T::STATUS2: ADCMPF2 Position      */
#define EADC_STATUS2_ADCMPF2_Msk         (0x1ul << EADC_STATUS2_ADCMPF2_Pos)               /*!< EADC_T::STATUS2: ADCMPF2 Mask          */

#define EADC_STATUS2_ADCMPF3_Pos         (7)                                               /*!< EADC_T::STATUS2: ADCMPF3 Position      */
#define EADC_STATUS2_ADCMPF3_Msk         (0x1ul << EADC_STATUS2_ADCMPF3_Pos)               /*!< EADC_T::STATUS2: ADCMPF3 Mask          */

#define EADC_STATUS2_ADOVIF0_Pos         (8)                                               /*!< EADC_T::STATUS2: ADOVIF0 Position      */
#define EADC_STATUS2_ADOVIF0_Msk         (0x1ul << EADC_STATUS2_ADOVIF0_Pos)               /*!< EADC_T::STATUS2: ADOVIF0 Mask          */

#define EADC_STATUS2_ADOVIF1_Pos         (9)                                               /*!< EADC_T::STATUS2: ADOVIF1 Position      */
#define EADC_STATUS2_ADOVIF1_Msk         (0x1ul << EADC_STATUS2_ADOVIF1_Pos)               /*!< EADC_T::STATUS2: ADOVIF1 Mask          */

#define EADC_STATUS2_ADOVIF2_Pos         (10)                                              /*!< EADC_T::STATUS2: ADOVIF2 Position      */
#define EADC_STATUS2_ADOVIF2_Msk         (0x1ul << EADC_STATUS2_ADOVIF2_Pos)               /*!< EADC_T::STATUS2: ADOVIF2 Mask          */

#define EADC_STATUS2_ADOVIF3_Pos         (11)                                              /*!< EADC_T::STATUS2: ADOVIF3 Position      */
#define EADC_STATUS2_ADOVIF3_Msk         (0x1ul << EADC_STATUS2_ADOVIF3_Pos)               /*!< EADC_T::STATUS2: ADOVIF3 Mask          */

#define EADC_STATUS2_ADCMPO0_Pos         (12)                                              /*!< EADC_T::STATUS2: ADCMPO0 Position      */
#define EADC_STATUS2_ADCMPO0_Msk         (0x1ul << EADC_STATUS2_ADCMPO0_Pos)               /*!< EADC_T::STATUS2: ADCMPO0 Mask          */

#define EADC_STATUS2_ADCMPO1_Pos         (13)                                              /*!< EADC_T::STATUS2: ADCMPO1 Position      */
#define EADC_STATUS2_ADCMPO1_Msk         (0x1ul << EADC_STATUS2_ADCMPO1_Pos)               /*!< EADC_T::STATUS2: ADCMPO1 Mask          */

#define EADC_STATUS2_ADCMPO2_Pos         (14)                                              /*!< EADC_T::STATUS2: ADCMPO2 Position      */
#define EADC_STATUS2_ADCMPO2_Msk         (0x1ul << EADC_STATUS2_ADCMPO2_Pos)               /*!< EADC_T::STATUS2: ADCMPO2 Mask          */

#define EADC_STATUS2_ADCMPO3_Pos         (15)                                              /*!< EADC_T::STATUS2: ADCMPO3 Position      */
#define EADC_STATUS2_ADCMPO3_Msk         (0x1ul << EADC_STATUS2_ADCMPO3_Pos)               /*!< EADC_T::STATUS2: ADCMPO3 Mask          */

#define EADC_STATUS2_CHANNEL_Pos         (16)                                              /*!< EADC_T::STATUS2: CHANNEL Position      */
#define EADC_STATUS2_CHANNEL_Msk         (0x1ful << EADC_STATUS2_CHANNEL_Pos)              /*!< EADC_T::STATUS2: CHANNEL Mask          */

#define EADC_STATUS2_BUSY_Pos            (23)                                              /*!< EADC_T::STATUS2: BUSY Position         */
#define EADC_STATUS2_BUSY_Msk            (0x1ul << EADC_STATUS2_BUSY_Pos)                  /*!< EADC_T::STATUS2: BUSY Mask             */

#define EADC_STATUS2_ADOVIF_Pos          (24)                                              /*!< EADC_T::STATUS2: ADOVIF Position       */
#define EADC_STATUS2_ADOVIF_Msk          (0x1ul << EADC_STATUS2_ADOVIF_Pos)                /*!< EADC_T::STATUS2: ADOVIF Mask           */

#define EADC_STATUS2_STOVF_Pos           (25)                                              /*!< EADC_T::STATUS2: STOVF Position        */
#define EADC_STATUS2_STOVF_Msk           (0x1ul << EADC_STATUS2_STOVF_Pos)                 /*!< EADC_T::STATUS2: STOVF Mask            */

#define EADC_STATUS2_AVALID_Pos          (26)                                              /*!< EADC_T::STATUS2: AVALID Position       */
#define EADC_STATUS2_AVALID_Msk          (0x1ul << EADC_STATUS2_AVALID_Pos)                /*!< EADC_T::STATUS2: AVALID Mask           */

#define EADC_STATUS2_AOV_Pos             (27)                                              /*!< EADC_T::STATUS2: AOV Position          */
#define EADC_STATUS2_AOV_Msk             (0x1ul << EADC_STATUS2_AOV_Pos)                   /*!< EADC_T::STATUS2: AOV Mask              */

#define EADC_STATUS3_CURSPL_Pos          (0)                                               /*!< EADC_T::STATUS3: CURSPL Position       */
#define EADC_STATUS3_CURSPL_Msk          (0x3ful << EADC_STATUS3_CURSPL_Pos)               /*!< EADC_T::STATUS3: CURSPL Mask           */

#define EADC_DDAT_RESULT_Pos             (0)                                               /*!< EADC_T::DDAT: RESULT Position          */
#define EADC_DDAT_RESULT_Msk             (0xfffful << EADC_DDAT_RESULT_Pos)                /*!< EADC_T::DDAT: RESULT Mask              */

#define EADC_DDAT_OV_Pos                 (16)                                              /*!< EADC_T::DDAT: OV Position              */
#define EADC_DDAT_OV_Msk                 (0x1ul << EADC_DDAT_OV_Pos)                       /*!< EADC_T::DDAT: OV Mask                  */

#define EADC_DDAT_VALID_Pos              (17)                                              /*!< EADC_T::DDAT: VALID Position           */
#define EADC_DDAT_VALID_Msk              (0x1ul << EADC_DDAT_VALID_Pos)                    /*!< EADC_T::DDAT: VALID Mask               */

#define EADC_CALCTL_CAL_Pos              (0)                                               /*!< EADC_T::CALCTL: CAL Position           */
#define EADC_CALCTL_CAL_Msk              (0x1ul << EADC_CALCTL_CAL_Pos)                    /*!< EADC_T::CALCTL: CAL Mask               */

#define EADC_CALCTL_CALIE_Pos            (1)                                               /*!< EADC_T::CALCTL: CALIE Position         */
#define EADC_CALCTL_CALIE_Msk            (0x1ul << EADC_CALCTL_CALIE_Pos)                  /*!< EADC_T::CALCTL: CALIE Mask             */

#define EADC_CALSR_CALIF_Pos             (16)                                              /*!< EADC_T::CALSR: CALIF Position          */
#define EADC_CALSR_CALIF_Msk             (0x1ul << EADC_CALSR_CALIF_Pos)                   /*!< EADC_T::CALSR: CALIF Mask              */

#define EADC_PDMACTL_PDMATEN_Pos         (0)                                               /*!< EADC_T::PDMACTL: PDMATEN Position      */
#define EADC_PDMACTL_PDMATEN_Msk         (0xffffffful << EADC_PDMACTL_PDMATEN_Pos)         /*!< EADC_T::PDMACTL: PDMATEN Mask          */

#define EADC_MCTL1_ALIGN_Pos             (0)                                               /*!< EADC_T::MCTL1: ALIGN Position          */
#define EADC_MCTL1_ALIGN_Msk             (0x1ul << EADC_MCTL1_ALIGN_Pos)                   /*!< EADC_T::MCTL1: ALIGN Mask              */

#define EADC_MCTL1_AVG_Pos               (1)                                               /*!< EADC_T::MCTL1: AVG Position            */
#define EADC_MCTL1_AVG_Msk               (0x1ul << EADC_MCTL1_AVG_Pos)                     /*!< EADC_T::MCTL1: AVG Mask                */

#define EADC_MCTL1_ACU_Pos               (4)                                               /*!< EADC_T::MCTL1: ACU Position            */
#define EADC_MCTL1_ACU_Msk               (0xful << EADC_MCTL1_ACU_Pos)                     /*!< EADC_T::MCTL1: ACU Mask                */

#define EADC_MCTL1_EXTSTDIV_Pos          (16)                                              /*!< EADC_T::MCTL1: EXTSTDIV Position       */
#define EADC_MCTL1_EXTSTDIV_Msk          (0x3ul << EADC_MCTL1_EXTSTDIV_Pos)                /*!< EADC_T::MCTL1: EXTSTDIV Mask           */

#define EADC_MCTL1_DBMEN_Pos             (20)                                              /*!< EADC_T::MCTL1: DBMEN Position          */
#define EADC_MCTL1_DBMEN_Msk             (0x1ul << EADC_MCTL1_DBMEN_Pos)                   /*!< EADC_T::MCTL1: DBMEN Mask              */

/** @} EADC_CONST */
/** @} end of EADC register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __EADC_REG_H__ */
