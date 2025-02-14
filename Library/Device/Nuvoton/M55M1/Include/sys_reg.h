/**************************************************************************//**
 * @file     sys_reg.h
 * @version  V1.00
 * @brief    SYS register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __SYS_REG_H__
#define __SYS_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup SYS System Manger Controller (SYS)
    Memory Mapped Structure for SYS Controller
    @{
*/

typedef struct
{


    /**
     * @var SYS_T::PDID
     * Offset: 0x00  Part Device Identification Number Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PDID      |Part Device Identification Number (Read Only)
     * |        |          |This register reflects device part number code
     * |        |          |Software can read this register to identify which device is used
     * @var SYS_T::RSTCTL
     * Offset: 0x04  System Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHIPRST   |Chip One-shot Reset (Write Protect)
     * |        |          |Setting this bit will reset the whole chip, including Processor core and all peripherals, and this bit will automatically return to 0 after the 2 clock cycles.
     * |        |          |The CHIPRST is same as the POR reset, all the chip controllers is reset and the chip setting from Flash are also reload.
     * |        |          |About the difference between CHIPRST and SYSRESETREQ(AIRCR[2]), please refer to section 6.2.2
     * |        |          |0 = Chip normal operation.
     * |        |          |1 = Chip one-shot reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[6]     |NPURST    |NPU Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the NPU
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = NPU controller normal operation.
     * |        |          |1 = NPU controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[7]     |CPURSTEN  |Processor Core Reset Enable (Write Protect)
     * |        |          |Setting this bit will only reset the processor core and Flash Memory Controller(FMC) when CPU execute WFI instruction.
     * |        |          |0 = Processor core not reset when execute WFI instruction.
     * |        |          |1 = Processor core reset when execute WFI instruction.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register..
     * @var SYS_T::RSTSTS
     * Offset: 0x08  System Reset Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PORF      |POR Reset Flag
     * |        |          |The POR reset flag is set by the u201CReset Signalu201D from the Power-on Reset (POR) Controller or bit CHIPRST (SYS_RSTCTL[0]) to indicate the previous reset source.
     * |        |          |0 = No reset from POR or CHIPRST.
     * |        |          |1 = Power-on Reset (POR) or CHIPRST had issued the reset signal to reset the system.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |PINRF     |nRESET Pin Reset Flag
     * |        |          |The nRESET pin reset flag is set by the u201CReset Signalu201D from the nRESET Pin to indicate the previous reset source.
     * |        |          |0 = No reset from nRESET pin.
     * |        |          |1 = Pin nRESET had issued the reset signal to reset the system.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: This bit is not retained when D4 power is turned off.
     * |[2]     |WDTRF     |WDT Reset Flag
     * |        |          |The WDT reset flag is set by the u201CReset Signalu201D from the Watchdog Timer or Window Watchdog Timer to indicate the previous reset source.
     * |        |          |0 = No reset from watchdog timer or window watchdog timer.
     * |        |          |1 = The watchdog timer or window watchdog timer had issued the reset signal to reset the system.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: This bit is not retained when D2 power is turned off.
     * |        |          |Note 3: Watchdog Timer register RSTF(WDT_CTL[2]) bit is set if the system has been reset by WDT time-out reset
     * |        |          |Window Watchdog Timer register WWDTRF(WWDT_STATUS[1]) bit is set if the system has been reset by WWDT time-out reset.
     * |        |          |Note 4: Extra Watchdog Timer register RSTF(EWDT_CTL[2]) bit is set if the system has been reset by EWDT time-out reset
     * |        |          |Extra Window Watchdog Timer register WWDTRF(EWWDT_STATUS[1]) bit is set if the system has been reset by EWWDT time-out reset.
     * |[3]     |LVRRF     |LVR Reset Flag
     * |        |          |The LVR reset flag is set by the u201CReset Signalu201D from the Low Voltage Reset Controller to indicate the previous reset source.
     * |        |          |0 = No reset from LVR.
     * |        |          |1 = The LVR controller had issued the reset signal to reset the system.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: This bit is not retained when D3 power is turned off.
     * |[4]     |BODRF     |BOD Reset Flag
     * |        |          |The BOD reset flag is set by the u201CReset Signalu201D from the Brown-out Detector to indicate the previous reset source.
     * |        |          |0 = No reset from BOD.
     * |        |          |1 = The BOD had issued the reset signal to reset the system.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: This bit is not retained when D3 power is turned off.
     * |[5]     |SYSRF     |System Reset Flag
     * |        |          |The system reset flag is set by the u201CReset Signalu201D from the Cortex-M55 core to indicate the previous reset source.
     * |        |          |0 = No reset from the Cortex-M55.
     * |        |          |1 = The Cortex-M55 had issued the reset signal to reset the system by writing 1 to the bit SYSRESETREQ(AIRCR[2], Application Interrupt and Reset Control Register, address = 0xE000ED0C) in system control registers of Cortex-M55 core.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[7]     |CPURF     |CPU Reset Flag
     * |        |          |The CPU reset flag is set by hardware if software writes CPURST (SYS_RSTCTL[7]) 1 to reset the Cortex-M55 core and Flash Memory Controller (FMC).
     * |        |          |0 = No reset from CPU.
     * |        |          |1 = The Cortex-M55 core and FMC are reset by software setting CPURST to 1.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[8]     |CPULKRF   |CPU Lockup Reset Flag
     * |        |          |The CPU Lockup reset flag is set by hardware if Cortex-M55 lockup happened.
     * |        |          |0 = No reset from CPU lockup happened.
     * |        |          |1 = The Cortex-M55 lockup happened and chip is reset.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: When CPU lockup happened under ICE is connected, this flag will set to 1 but chip will not reset.
     * |        |          |Note 3: This bit is not retained when D0 power is turned off.
     * @var SYS_T::VTORSET
     * Offset: 0x0C  VTOR Setting Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:7]  |VTORSET   |VTOR Setting After SPD Wakeup (Write Protect)
     * |        |          |The value will be loaded to Vector Table Offset Register, which is at the address 0xE000ED08, when chip wake up and Perfomace Domain be turn off.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D4 power is turned off.
     * @var SYS_T::SRAMICTL
     * Offset: 0x10  System SRAM Parity Error Interrupt Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PERRIEN   |SRAM Parity Check Error Interrupt Enable Bit
     * |        |          |0 = SRAM parity check error interrupt Disabled.
     * |        |          |1 = SRAM parity check error interrupt Enabled.
     * |        |          |Note: This bit is not retained when D0 power is turned off.
     * @var SYS_T::SRAMSTS
     * Offset: 0x14  System SRAM Parity Check Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PERRIF    |SRAM Parity Check Error Flag
     * |        |          |This bit indicates the System SRAM byte 0 parity error occurred.
     * |        |          |0 = No System SRAM parity error.
     * |        |          |1 = System SRAM parity error occured.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |        |          |Note 3: This bit is not retained when D0 power is turned off.
     * |[8]     |B0PERRIF  |SRAM Byte 0 Parity Check Error Flag (Read Only)
     * |        |          |This bit indicates the System SRAM byte 0 parity error occurred.
     * |        |          |0 = No System SRAM byte 0 parity error.
     * |        |          |1 = System SRAM byte 0 parity error occured.
     * |        |          |Note 1: Write 1 to PERRIF for clear this bit to 0..
     * |        |          |Note 2: The default value depends on the content of SRAM
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |        |          |Note 3: This bit is not retained when D0 power is turned off.
     * |[9]     |B1PERRIF  |SRAM Byte 1 Parity Check Error Flag (Read Only)
     * |        |          |This bit indicates the System SRAM byte 1 parity error occurred.
     * |        |          |0 = No System SRAM byte 1 parity error.
     * |        |          |1 = System SRAM byte 1 parity error occured.
     * |        |          |Note 1: Write 1 to PERRIF for clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |        |          |Note 3: This bit is not retained when D0 power is turned off.
     * |[10]    |B2PERRIF  |SRAM Byte 2 Parity Check Error Flag (Read Only)
     * |        |          |This bit indicates the System SRAM byte 2 parity error occurred.
     * |        |          |0 = No System SRAM byte 2 parity error.
     * |        |          |1 = System SRAM byte 2 parity error occured.
     * |        |          |Note 1: Write 1 to PERRIF for clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |        |          |Note 3: This bit is not retained when D0 power is turned off.
     * |[11]    |B3PERRIF  |SRAM Byte 3 Parity Check Error Flag (Read Only)
     * |        |          |This bit indicates the System SRAM byte 3 parity error occurred.
     * |        |          |0 = No System SRAM byte 3 parity error.
     * |        |          |1 = System SRAM byte 3 parity error occured.
     * |        |          |Note 1: Write 1 to PERRIF for clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |        |          |Note 3: This bit is not retained when D0 power is turned off.
     * |[12]    |B4PERRIF  |SRAM Byte 4 Parity Check Error Flag (Read Only)
     * |        |          |This bit indicates the System SRAM byte 4 parity error occurred.
     * |        |          |0 = No System SRAM byte 4 parity error.
     * |        |          |1 = System SRAM byte 4 parity error occured.
     * |        |          |Note 1: Write 1 to PERRIF for clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |        |          |Note 3: This bit is not retained when D0 power is turned off.
     * |[13]    |B5PERRIF  |SRAM Byte 5 Parity Check Error Flag (Read Only)
     * |        |          |This bit indicates the System SRAM byte 5 parity error occurred.
     * |        |          |0 = No System SRAM byte 5 parity error.
     * |        |          |1 = System SRAM byte 5 parity error occured.
     * |        |          |Note 1: Write 1 to PERRIF for clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |        |          |Note 3: This bit is not retained when D0 power is turned off.
     * |[14]    |B6PERRIF  |SRAM Byte 6 Parity Check Error Flag (Read Only)
     * |        |          |This bit indicates the System SRAM byte 6 parity error occurred.
     * |        |          |0 = No System SRAM byte 6 parity error.
     * |        |          |1 = System SRAM byte 6 parity error occured.
     * |        |          |Note 1: Write 1 to PERRIF for clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |        |          |Note 3: This bit is not retained when D0 power is turned off.
     * |[15]    |B7PERRIF  |SRAM Byte 7 Parity Check Error Flag (Read Only)
     * |        |          |This bit indicates the System SRAM byte 7 parity error occurred.
     * |        |          |0 = No System SRAM byte 7 parity error.
     * |        |          |1 = System SRAM byte 7 parity error occured.
     * |        |          |Note 1: Write 1 to PERRIF for clear this bit to 0.
     * |        |          |Note 2: The default value depends on the content of SRAM
     * |        |          |It is better to check this value before use it
     * |        |          |If the default value is 1, it shall be cleared first.
     * |        |          |Note 3: This bit is not retained when D0 power is turned off.
     * @var SYS_T::SRAMEADR
     * Offset: 0x18  System SRAM Parity Error Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ERRADDR   |System SRAM Parity Error Address (Read Only)
     * |        |          |This register shows system SRAM parity error byte offset.
     * |        |          |Note: These bits are not retained when D0 power is turned off.
     * @var SYS_T::BODCTL
     * Offset: 0x20  Brown-out Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BODEN     |Brown-out Detector Enable Bit (Write Protect)
     * |        |          |The default value is set by Flash controller user configuration register CBODEN (CONFIG0 [19]).
     * |        |          |0 = Brown-out Detector function Disabled.
     * |        |          |1 = Brown-out Detector function Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: This bit is not retained when D3 power is turned off.
     * |        |          |Note 4: The default value is set by inverse UCFG0[19].
     * |[1]     |BODLPM    |Brown-out Detector Low Power Mode (Write Protect)
     * |        |          |0 = BOD operated in normal mode (default).
     * |        |          |1 = BOD Low Power mode Enabled.
     * |        |          |Note 1: The BOD consumes about 10uA in normal mode, the low power mode can reduce the current to about 1/10 but slow the BOD response.
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 4: This bit is not retained when D3 power is turned off.
     * |[2]     |BODRSTEN  |Brown-out Reset Enable Bit (Write Protect)
     * |        |          |The default value is set by Flash controller user configuration register CBORST(CONFIG0[20]) bit.
     * |        |          |0 = Brown-out u201CINTERRUPTu201D function Enabled.
     * |        |          |1 = Brown-out u201CRESETu201D function Enabled.
     * |        |          |Note 1: When the Brown-out Detector function is enabled (BODEN high) and BOD reset function is enabled (BODRSTEN high), BOD will assert a signal to reset chip when the detected voltage is lower than the threshold (BODOUT high).
     * |        |          |When the BOD function is enabled (BODEN high) and BOD interrupt function is enabled (BODRSTEN low), BOD will assert an interrupt if AVDD is lower than BODVL, BOD interrupt will keep till the BODIF set to 0
     * |        |          |BOD interrupt can be blocked by disabling the NVIC BOD interrupt or disabling BOD function (setting BODEN low).
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: This bit is not retained when D3 power is turned off.
     * |        |          |Note 4: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 5: The default value is set by inverse UCFG0[20]
     * |        |          |Note 6: If D2PGEN(PMC_PWRCTL[2]) is 1 and enters Power-down mode, the brown-out reset function always keeps enabled even if BODRSTEN is 0.
     * |[6:4]   |BODDGSEL  |Brown-out Detector Output De-glitch Time Select (Write Protect)
     * |        |          |000 = BOD output is sampled by LIRC clock.
     * |        |          |001 = 4 system clock (SCLK).
     * |        |          |010 = 8 system clock (SCLK).
     * |        |          |011 = 16 system clock (SCLK).
     * |        |          |100 = 32 system clock (SCLK).
     * |        |          |101 = 64 system clock (SCLK).
     * |        |          |110 = 128 system clock (SCLK).
     * |        |          |111 = 256 system clock (SCLK).
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: These bits are not retained when D0 power is turned off.
     * |        |          |Note 4: Because SCLK is invalid in Power-down mode, the de-glitch time must change to LIRC before system enters Power-down mode.
     * |        |          |Note 5: If BODLPM(SYS_BODCTL[1]) is 1, the Brown-out Detector Output de-glitch circuit always uses LIRC clock even if BODDGSEL is not 000.
     * |[10:8]  |BODVL     |Brown-out Detector Threshold Voltage Selection (Write Protect)
     * |        |          |The default value is set by Flash controller user configuration register CBOV (CONFIG0 [23:21]).
     * |        |          |000 = Brown-out Detector threshold voltage is 1.6V.
     * |        |          |001 = Brown-out Detector threshold voltage is 1.8V.
     * |        |          |010 = Brown-out Detector threshold voltage is 2.0V.
     * |        |          |011 = Brown-out Detector threshold voltage is 2.2V.
     * |        |          |100 = Brown-out Detector threshold voltage is 2.4V.
     * |        |          |101 = Brown-out Detector threshold voltage is 2.6V.
     * |        |          |110 = Brown-out Detector threshold voltage is 2.8V.
     * |        |          |111 = Brown-out Detector threshold voltage is 3.0V.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: These bits are not retained when D3 power is turned off.
     * |        |          |Note 4: The default value is set by UCFG0[23:21].
     * |[13:12] |BODWKEN   |Brown-out Detector Wake-Up Selection (Write Protect)
     * |        |          |00 = Brown-out Detector wake-up function Disable.
     * |        |          |01 = Brown-out Detector power rise wake-up function Enable.
     * |        |          |10 = Brown-out Detector power drop wake-up function Enable.
     * |        |          |11 = Brown-out Detector power rise and drop wake-up function Enable.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: These bits are not retained when D3 power is turned off.
     * |[16]    |LVREN     |Low Voltage Reset Enable Bit (Write Protect)
     * |        |          |The LVR function resets the chip when the input power voltage is lower than LVR circuit setting
     * |        |          |LVR function is enabled by default.
     * |        |          |0 = Low Voltage Reset function Disabled.
     * |        |          |1 = Low Voltage Reset function Enabled.
     * |        |          |Note 1: After enabling the bit, the LVR function will be active with 200us delay for LVR output stable (default).
     * |        |          |Note 2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 3: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 4: This bit is not retained when D3 power is turned off.
     * |[22:20] |LVRDGSEL  |LVR Output De-glitch Time Select (Write Protect)
     * |        |          |000 = Without de-glitch function.
     * |        |          |001 = 4 system clock (SCLK).
     * |        |          |010 = 8 system clock (SCLK).
     * |        |          |011 = 16 system clock (SCLK).
     * |        |          |100 = 32 system clock (SCLK).
     * |        |          |101 = 64 system clock (SCLK).
     * |        |          |110 = 128 system clock (SCLK).
     * |        |          |111 = 256 system clock (SCLK).
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D0 power is turned off.
     * |        |          |Note 3: Because SCLK is invalid in Power-down mode, the de-glitch time must change to without de-glitch before system enters Power-down mode.
     * |[31]    |WRBUSY    |Write Busy Flag (Read Only)
     * |        |          |If SYS_BODCTL is written, this bit is asserted automatically by hardware, and is de-asserted when write procedure is finished.
     * |        |          |0 = SYS_BODCTL register is ready for write operation.
     * |        |          |1 = SYS_BODCTL register is busy on the last write operation. Other write operations are ignored.
     * @var SYS_T::BODSTS
     * Offset: 0x24  Brown-out Detector Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BODOUT    |Brown-out Detector Output Status (Read Only)
     * |        |          |0 = Brown-out Detector output status is 0.
     * |        |          |It means the detected voltage is higher than BODVL(SYS_BODCTL[10:8]) setting or BODEN is 0.
     * |        |          |1 = Brown-out Detector output status is 1.
     * |        |          |It means the detected voltage is lower than BODVL(SYS_BODCTL[10:8]) setting
     * |        |          |If the BODEN is 0, BOD function disabled, this bit always responds 0.
     * |[1]     |BODIF     |Brown-out Detector Interrupt Flag
     * |        |          |0 = Brown-out Detector does not detect any voltage draft at VDD down through or up through the voltage of BODVL(SYS_BODCTL[10:8]) setting.
     * |        |          |1 = When Brown-out Detector detects the VDD is dropped down through the voltage of BODVL(SYS_BODCTL[10:8]) setting or the VDD is raised up through the voltage of BODVL(SYS_BODCTL[10:8]) setting, this bit is set to 1 and the brown-out interrupt is requested if brown-out interrupt is enabled.
     * |        |          |Note 1: Write 1 to clear this bit to 0.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * @var SYS_T::PORCTL
     * Offset: 0x28  Power-on Reset Controller Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PORMASK   |Power-on Reset Mask Enable Bit (Write Protect)
     * |        |          |When powered on, the POR circuit generates a reset signal to reset the whole chip function, but noise on the power may cause the POR active again
     * |        |          |User can mask internal POR signal to avoid unpredictable noise to cause chip reset by writing 0x5AA5 to this field.
     * |        |          |The POR function will be active again when this field is set to another value or chip is reset by other reset source, including:
     * |        |          |nRESET, Watchdog, LVR reset, BOD reset, ICE reset command and the software-chip reset function.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D3 power is turned off.
     * |[31:16] |POROFF    |Power-on Reset Enable Bit (Write Protect)
     * |        |          |When powered on, the POR circuit generates a reset signal to reset the whole chip function, but noise on the power may cause the POR active again
     * |        |          |User can disable internal POR circuit to avoid unpredictable noise to cause chip reset by writing 0x5AA5 to this field.
     * |        |          |The POR function will be active again when this field is set to another value or chip is reset by other reset source, including:
     * |        |          |nRESET, Watchdog, LVR reset, BOD reset, ICE reset command and the software-chip reset function.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D3 power is turned off.
     * @var SYS_T::VREFCTL
     * Offset: 0x2C  VREF Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |VREFCTL   |VREF Control Bits (Write Protect)
     * |        |          |00000 = VREF is from external pin.
     * |        |          |00011 = VREF is internal 1.6V.
     * |        |          |00111 = VREF is internal 2.048V.
     * |        |          |01011 = VREF is internal 2.500V.
     * |        |          |01111 = VREF is internal 3.072V.
     * |        |          |10000 = VREF is from AVDD.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: These bits are not retained when D2 power is turned off.
     * |[7:6]   |PRELOADSEL|Pre-load Timing Selection (Write Protect)
     * |        |          |00 = pre-load time is 480us for 1uF Capacitor, initial = 0v.
     * |        |          |01 = pre-load time is 2ms for 4.7uF Capacitor, initial = 0v.
     * |        |          |10 = pre-load time is 3ms for 1uF Capacitor, initial = 3.6v.
     * |        |          |11 = pre-load time is 15ms for 4.7uF Capacitor, initial = 3.6v.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: These bits are not retained when D2 power is turned off.
     * |[8]     |SCPTEST   |Short Circuit Protection Test Pin (Write Protect)
     * |        |          |0 = Short circuit protection Disabled.
     * |        |          |1 = Short circuit protection Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: Write operation is ignored when WRBUSY is high.
     * |        |          |Note 3: These bits are not retained when D2 power is turned off.
     * |[9]     |SCPFINISH |Short Circuit Protection Result (Read Only)
     * |        |          |Short circuit protection finish.
     * |[31]    |WRBUSY    |Write Busy Flag (Read Only)
     * |        |          |If SYS_VREFCTL is written, this bit is asserted automatically by hardware, and is de-asserted when write procedure is finished.
     * |        |          |0 = SYS_VREFCTL register is ready for write operation.
     * |        |          |1 = SYS_VREFCTL register is busy on the last write operation. Other write operations are ignored.
     * @var SYS_T::IVSCTL
     * Offset: 0x30  Internal Voltage Source Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |VTEMPEN   |Temperature Sensor Enable Bit
     * |        |          |This bit is used to enable/disable temperature sensor function.
     * |        |          |0 = Temperature sensor function Disabled (default).
     * |        |          |1 = Temperature sensor function Enabled.
     * |        |          |Note 1: After this bit is set to 1, the value of temperature sensor output can be obtained from ADC conversion result
     * |        |          |Please refer to ADC chapter for details.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |VBATUGEN  |VBAT Unity Gain Buffer Enable Bit
     * |        |          |This bit is used to enable/disable VBAT unity gain buffer function.
     * |        |          |0 = VBAT unity gain buffer function Disabled (default).
     * |        |          |1 = VBAT unity gain buffer function Enabled.
     * |        |          |Note: After this bit is set to 1, the value of VBAT unity gain buffer output voltage can be obtained from ADC conversion result.
     * |[2]     |AVDDDIV4EN|AVDD divide 4 Enable Bit
     * |        |          |This bit is used to enable/disable AVDD divide 4 function.
     * |        |          |0 = AVDD divide 4 function Disabled (default).
     * |        |          |1 = AVDD divide 4 function Enabled.
     * |        |          |Note 1: After this bit is set to 1, the value of AVDD divide 4 output voltage can be obtained from ADC conversion result
     * |        |          |Note 2: These bits are not retained when D2 power is turned off.
     * |[4]     |VTEMPSEL  |Temperature Sensor Slope Select Bit
     * |        |          |This bit is used to select temperature sensor slope trend.
     * |        |          |0 = Temperature sensor CTAT slope.
     * |        |          |1 = Temperature sensor PTAT slope.
     * |        |          |Note 2: These bits are not retained when D0 power is turned off.
     * @var SYS_T::USBPHY
     * Offset: 0x34  USB PHY Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |USBROLE   |USB Role Option (Write Protect)
     * |        |          |These two bits are used to select the role of USB.
     * |        |          |00 = Standard USB Device mode.
     * |        |          |01 = Standard USB Host mode.
     * |        |          |10 = ID dependent mode.
     * |        |          |11 = On-The-Go device mode.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[8]     |OTGPHYEN  |USB OTG PHY Enable (Write Protect)
     * |        |          |This bit is used to enable/disable OTG PHY function.
     * |        |          |0 = OTG PHY function Disabled (default).
     * |        |          |1 = OTG PHY function Enabled.
     * |        |          |Note: This bit is not retained when D1 power is turned off.
     * |[17:16] |HSUSBROLE |HSUSB Role Option (Write Protect)
     * |        |          |These two bits are used to select the role of HSUSB.
     * |        |          |00 = Standard HSUSB Device mode.
     * |        |          |01 = Standard HSUSB Host mode.
     * |        |          |10 = ID dependent mode.
     * |        |          |11 = On-The-Go device mode.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: These bits are not retained when D1 power is turned off.
     * |[24]    |HSOTGPHYEN|HSUSB OTG PHY Enable (Write Protect)
     * |        |          |This bit is used to enable/disable HSOTG PHY function.
     * |        |          |0 = HSUSB OTG PHY function Disabled (default).
     * |        |          |1 = HSUSB OTG PHY function Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * |[25]    |HSUSBACT  |HSOTG PHY Active Control
     * |        |          |This bit is used to control HSOTG PHY at reset state or active state.
     * |        |          |0 = HSOTG PHY at reset state.
     * |        |          |1 = HSOTG PHY at active state.
     * |        |          |Note: After setting HSOTGPHYEN (SYS_USBPHY[24]) to enable HSOTG PHY, user should keep HSOTG PHY at reset mode at lease 10us before changing to active mode.
     * @var SYS_T::UTCPDCTL
     * Offset: 0x38  UTCPD Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |IOMODE    |UTCPD0 as I/O mode
     * |        |          |This bit is used to define UTCPD0 CCx and CCDBx function
     * |        |          |0 = Pin as UTCPD0 CCx and CCDBx function.
     * |        |          |1 = Pin as general I/O function.
     * |        |          |Note: This bit is not retained when D4 power is turned off.
     * |[1]     |POREN0    |UTCPD0 Power-on Enable Bit
     * |        |          |0 = UTCPD0 PHY in reset mode.
     * |        |          |1 = UTCPD0 PHY in normal mode.
     * |        |          |Note 1: user should set POREN0 to 1 after IOMODE is setting down.
     * |        |          |Note 2: This bit is not retained when D4 power is turned off.
     * |[10:8]  |PD0VBDSS  |UTCPD0 VBUS Detect Source Select
     * |        |          |UTCPD0 controller need a VBUS detect result to note if VBUS is connected
     * |        |          |For NPD0~2 usage, ACMP can be another voltage detect method to detect VBUS pulg in or out
     * |        |          |This bit field is used to select UTCPD0 VBUS detect source
     * |        |          |And the selected result is VBDETSW0
     * |        |          |000 = UTCPD0 VBUS detect source from UTCPD0 PHY.
     * |        |          |001 = UTCPD0 VBUS detect source from ACMP0 output.
     * |        |          |010 = UTCPD0 VBUS detect source from ACMP1 output.
     * |        |          |011 = UTCPD0 VBUS detect source from ACMP2 output.
     * |        |          |Others = Reserved.
     * |        |          |Note 1: Before use UTCPD0 function, UDVBDETS should be set and cannot change during UTCPD0 operating.
     * |        |          |Note 2: This bit is not retained when D3 power is turned off.
     * |[12]    |UDVBDETS  |UDC11 VBUS Detect Source Select
     * |        |          |0 = UDC11 VBUS detect source from OTGPHY.
     * |        |          |1 = UDC11 VBUS detect source from VBDETSW0.
     * |        |          |Note 1: Before use UDC11 function, UDVBDETS should be set and cannot change during UDC11 operating.
     * |        |          |Note 2: This bit is not retained when D3 power is turned off.
     * @var SYS_T::DBUSCTL
     * Offset: 0x40  Data Bus Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |H2XBSTEN  |Burst AHB2AXI Bridge access Enable Bit (Write Protect)
     * |        |          |0 = Normal operating.
     * |        |          |1 = Force the attribute of all access from AHB to AXI modifiable.
     * |        |          |Note 1: These bits are write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::GPA_MFOS
     * Offset: 0x80  GPIOA Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MFOS0     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * |[1]     |MFOS1     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * |[2]     |MFOS2     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * |[3]     |MFOS3     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * |[4]     |MFOS4     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * |[5]     |MFOS5     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * |[6]     |MFOS6     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * |[7]     |MFOS7     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * |[8]     |MFOS8     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * |[9]     |MFOS9     |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * |[10]    |MFOS10    |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * |[11]    |MFOS11    |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * |[12]    |MFOS12    |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * |[13]    |MFOS13    |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * |[14]    |MFOS14    |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * |[15]    |MFOS15    |GPIOA Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PA.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note 1: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPB_MFOS
     * Offset: 0x84  GPIOB Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MFOS0     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[1]     |MFOS1     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[2]     |MFOS2     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[3]     |MFOS3     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[4]     |MFOS4     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[5]     |MFOS5     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[6]     |MFOS6     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[7]     |MFOS7     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[8]     |MFOS8     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[9]     |MFOS9     |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[10]    |MFOS10    |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[11]    |MFOS11    |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12]    |MFOS12    |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[13]    |MFOS13    |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[14]    |MFOS14    |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[15]    |MFOS15    |GPIOB Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PB.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPC_MFOS
     * Offset: 0x88  GPIOC Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MFOS0     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[1]     |MFOS1     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[2]     |MFOS2     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[3]     |MFOS3     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[4]     |MFOS4     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[5]     |MFOS5     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[6]     |MFOS6     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[7]     |MFOS7     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[8]     |MFOS8     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[9]     |MFOS9     |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[10]    |MFOS10    |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[11]    |MFOS11    |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12]    |MFOS12    |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[13]    |MFOS13    |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[14]    |MFOS14    |GPIOC Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PC.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPD_MFOS
     * Offset: 0x8C  GPIOD Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MFOS0     |GPIOD Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[1]     |MFOS1     |GPIOD Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[2]     |MFOS2     |GPIOD Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[3]     |MFOS3     |GPIOD Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[4]     |MFOS4     |GPIOD Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[5]     |MFOS5     |GPIOD Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[6]     |MFOS6     |GPIOD Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[7]     |MFOS7     |GPIOD Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[8]     |MFOS8     |GPIOD Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[9]     |MFOS9     |GPIOD Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[10]    |MFOS10    |GPIOD Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[11]    |MFOS11    |GPIOD Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12]    |MFOS12    |GPIOD Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[13]    |MFOS13    |GPIOD Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[14]    |MFOS14    |GPIOD Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PD.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPE_MFOS
     * Offset: 0x90  GPIOE Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MFOS0     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[1]     |MFOS1     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[2]     |MFOS2     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[3]     |MFOS3     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[4]     |MFOS4     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[5]     |MFOS5     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[6]     |MFOS6     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[7]     |MFOS7     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[8]     |MFOS8     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[9]     |MFOS9     |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[10]    |MFOS10    |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[11]    |MFOS11    |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12]    |MFOS12    |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[13]    |MFOS13    |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[14]    |MFOS14    |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[15]    |MFOS15    |GPIOE Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PE.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPF_MFOS
     * Offset: 0x94  GPIOF Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MFOS0     |GPIOF Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[1]     |MFOS1     |GPIOF Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[2]     |MFOS2     |GPIOF Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[3]     |MFOS3     |GPIOF Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[4]     |MFOS4     |GPIOF Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[5]     |MFOS5     |GPIOF Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[6]     |MFOS6     |GPIOF Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[7]     |MFOS7     |GPIOF Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[8]     |MFOS8     |GPIOF Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[9]     |MFOS9     |GPIOF Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[10]    |MFOS10    |GPIOF Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[11]    |MFOS11    |GPIOF Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PF.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPG_MFOS
     * Offset: 0x98  GPIOG Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MFOS0     |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[1]     |MFOS1     |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[2]     |MFOS2     |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[3]     |MFOS3     |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[4]     |MFOS4     |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[5]     |MFOS5     |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[6]     |MFOS6     |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[7]     |MFOS7     |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[8]     |MFOS8     |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[9]     |MFOS9     |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[10]    |MFOS10    |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[11]    |MFOS11    |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12]    |MFOS12    |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[13]    |MFOS13    |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[14]    |MFOS14    |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[15]    |MFOS15    |GPIOG Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PG.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPH_MFOS
     * Offset: 0x9C  GPIOH Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MFOS0     |GPIOH Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PH.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[1]     |MFOS1     |GPIOH Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PH.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[2]     |MFOS2     |GPIOH Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PH.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[3]     |MFOS3     |GPIOH Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PH.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[4]     |MFOS4     |GPIOH Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PH.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[5]     |MFOS5     |GPIOH Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PH.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[6]     |MFOS6     |GPIOH Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PH.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[7]     |MFOS7     |GPIOH Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PH.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[8]     |MFOS8     |GPIOH Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PH.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[9]     |MFOS9     |GPIOH Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PH.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[10]    |MFOS10    |GPIOH Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PH.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[11]    |MFOS11    |GPIOH Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PH.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12]    |MFOS12    |GPIOH Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PH.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[13]    |MFOS13    |GPIOH Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PH.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[14]    |MFOS14    |GPIOH Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PH.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[15]    |MFOS15    |GPIOH Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PH.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPI_MFOS
     * Offset: 0xA0  GPIOI Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[6]     |MFOS6     |GPIOI Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PI.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[7]     |MFOS7     |GPIOI Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PI.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[8]     |MFOS8     |GPIOI Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PI.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[9]     |MFOS9     |GPIOI Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PI.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[10]    |MFOS10    |GPIOI Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PI.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[11]    |MFOS11    |GPIOI Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PI.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12]    |MFOS12    |GPIOI Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PI.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[13]    |MFOS13    |GPIOI Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PI.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[14]    |MFOS14    |GPIOI Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PI.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[15]    |MFOS15    |GPIOI Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PI.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPJ_MFOS
     * Offset: 0xA4  GPIOJ Multiple Function Output Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MFOS0     |GPIOJ Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PJ.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[1]     |MFOS1     |GPIOJ Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PJ.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[2]     |MFOS2     |GPIOJ Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PJ.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[3]     |MFOS3     |GPIOJ Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PJ.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[4]     |MFOS4     |GPIOJ Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PJ.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[5]     |MFOS5     |GPIOJ Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PJ.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[6]     |MFOS6     |GPIOJ Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PJ.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[7]     |MFOS7     |GPIOJ Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PJ.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[8]     |MFOS8     |GPIOJ Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PJ.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[9]     |MFOS9     |GPIOJ Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PJ.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[10]    |MFOS10    |GPIOJ Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PJ.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[11]    |MFOS11    |GPIOJ Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PJ.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12]    |MFOS12    |GPIOJ Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PJ.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[13]    |MFOS13    |GPIOJ Pin[x] Multiple Function Pin Output Mode Select
     * |        |          |This bit used to select multiple function pin output mode type for PJ.x pin.
     * |        |          |0 = Multiple function pin output mode type is Push-pull mode.
     * |        |          |1 = Multiple function pin output mode type is Open-drain mode.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::REGLCTL
     * Offset: 0x100  Register Lock Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |REGLCTL   |Register Lock Control Code
     * |        |          |Some registers have write-protection function
     * |        |          |Writing these registers have to disable the protected function by writing the sequence value u201C59hu201D, u201C16hu201D, u201C88hu201D to this field
     * |        |          |After this sequence is completed, the REGLCTL bit will be set to 1 and write-protection registers can be normal write.
     * |        |          |REGLCTL[0]
     * |        |          |Register Lock Control Disable Index
     * |        |          |0 = Write-protection Enabled for writing protected registers
     * |        |          |Any write to the protected register is ignored.
     * |        |          |1 = Write-protection Disabled for writing protected registers.
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::TCTL48M
     * Offset: 0x110  HIRC 48M Trim Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |FREQSEL   |Trim Frequency Selection
     * |        |          |This field indicates the target frequency of 48 MHz internal high speed RC oscillator (HIRC48M) auto trim.
     * |        |          |During auto trim operation, if clock error detected with CESTOPEN is set to 1 or trim retry limitation count reached, this field will be cleared to 00 automatically.
     * |        |          |00 = Disable HIRC48M auto trim function.
     * |        |          |01 = Enable HIRC48M auto trim function and trim HIRC48M to 48 MHz.
     * |        |          |10 = Reserved..
     * |        |          |11 = Reserved.
     * |[3:2]   |ACCURSEL  |Trim Accuracy Selection
     * |        |          |This field indicates the target frequency accuracy of 48 MHz internal high speed RC oscillator (HIRC48M) auto trim.
     * |        |          |00 = Accuracy is +-0.25% deviation within all temperature ranges.
     * |        |          |01 = Accuracy is +-0.50% deviation within all temperature ranges.
     * |        |          |10 = Accuracy is +-0.75% deviation within all temperature ranges.
     * |        |          |11 = Accuracy is +-1% deviation within all temperature ranges.
     * |[5:4]   |LOOPSEL   |Trim Calculation Loop Selection
     * |        |          |This field defines that trim value calculation is based on how many reference clocks.
     * |        |          |00 = Trim value calculation is based on average difference in 4 clocks of reference clock.
     * |        |          |01 = Trim value calculation is based on average difference in 8 clocks of reference clock.
     * |        |          |10 = Trim value calculation is based on average difference in 16 clocks of reference clock.
     * |        |          |11 = Trim value calculation is based on average difference in 32 clocks of reference clock.
     * |        |          |Note: For example, if LOOPSEL is set as 00, auto trim circuit will calculate trim value based on the average frequency difference in 4 clocks of reference clock.
     * |[7:6]   |RETRYCNT  |Trim Value Update Limitation Count
     * |        |          |This field defines that how many times the auto trim circuit will try to update the HIRC48M trim value before the frequency of HIRC48M locked.
     * |        |          |Once the HIRC48M locked, the internal trim value update counter will be reset.
     * |        |          |If the trim value update counter reached this limitation value and frequency of HIRC48M is still not locked, the auto trim operation will be disabled and FREQSEL will be cleared to 00.
     * |        |          |00 = Trim retry count limitation is 64 loops.
     * |        |          |01 = Trim retry count limitation is 128 loops.
     * |        |          |10 = Trim retry count limitation is 256 loops.
     * |        |          |11 = Trim retry count limitation is 512 loops.
     * |[8]     |CESTOPEN  |Clock Error Stop Enable Bit
     * |        |          |0 = Trim operation is kept going if clock is inaccurate.
     * |        |          |1 = Trim operation is stopped if clock is inaccurate.
     * |[9]     |BOUNDEN   |Boundary Enable Bit
     * |        |          |0 = Boundary function Disabled.
     * |        |          |1 = Boundary function Enabled.
     * |[10]    |REFCKSEL  |Reference Clock Selection
     * |        |          |0 = HIRC48M trim reference clock is from external 32.768 kHz crystal oscillator.
     * |        |          |1 = HIRC48M trim reference clock is from internal USB synchronous mode.
     * |[20:16] |BOUNDARY  |Boundary Selection
     * |        |          |Fill the boundary range from 0x1 to 0x1F, 0x0 is reserved.
     * |        |          |Note: This field is effective only when the BOUNDEN(SYS_TCTL48M [9]) is enabled.
     * @var SYS_T::TIEN48M
     * Offset: 0x114  HIRC 48M Trim Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TFAILIEN  |Trim Failure Interrupt Enable Bit
     * |        |          |This bit controls if an interrupt will be triggered while HIRC48M trim value update limitation count reached and HIRC48M frequency still not locked on target frequency set by FREQSEL(SYS_TCTL48M[1:0]).
     * |        |          |If this bit is high and TFAILIF(SYS_TSTS48M [1]) is set during auto trim operation, an interrupt will be triggered to notify that HIRC48M trim value update limitation count was reached.
     * |        |          |0 = Disable TFAILIF(SYS_TISTS48M [1]) status to trigger an interrupt to CPU.
     * |        |          |1 = Enable TFAILIF(SYS_TISTS48M[1]) status to trigger an interrupt to CPU.
     * |[2]     |CLKEIEN   |Clock Error Interrupt Enable Bit
     * |        |          |This bit controls if CPU would get an interrupt while clock is inaccuracy during auto trim operation.
     * |        |          |If this bit is set to1, and CLKERRIF(SYS_TISTS48M [2]) is set during auto trim operation, an interrupt will be triggered to notify the clock frequency is inaccuracy.
     * |        |          |0 = Disable CLKERRIF(SYS_TISTS48M [2]) status to trigger an interrupt to CPU.
     * |        |          |1 = Enable CLKERRIF(SYS_TISTS48M [2]) status to trigger an interrupt to CPU.
     * @var SYS_T::TISTS48M
     * Offset: 0x118  HIRC 48M Trim Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FREQLOCK  |HIRC48M Frequency Lock Status
     * |        |          |This bit indicates the HIRC48M frequency is locked.
     * |        |          |This is a status bit and does not trigger any interrupt.
     * |        |          |This bit will be set automatically, if the frequency is lock and the RC_TRIM is enabled.
     * |        |          |0 = The internal high-speed oscillator frequency is not locked at 48 MHz.
     * |        |          |1 = The internal high-speed oscillator frequency locked at 48 MHz.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[1]     |TFAILIF   |Trim Failure Interrupt Status
     * |        |          |This bit indicates that HIRC48M trim value update limitation count reached and the HIRC48M clock frequency is still not locked
     * |        |          |Once this bit is set, the auto trim operation stopped and FREQSEL(SYS_TCTL48M[1:0]) will be cleared to 00 by hardware automatically.
     * |        |          |If this bit is set and TFAILIEN(SYS_TIEN48M[1]) is high, an interrupt will be triggered to notify that HIRC48M trim value update limitation count was reached.
     * |        |          |0 = Trim value update limitation count does not reach.
     * |        |          |1 = Trim value update limitation count reached and HIRC48M frequency still not locked.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[2]     |CLKERRIF  |Clock Error Interrupt Status
     * |        |          |When the frequency of 32.768 kHz external low speed crystal oscillator (LXT) or 48 MHz internal high speed RC oscillator (HIRC48M) is shift larger to unreasonable value, this bit will be set and to be an indicate that clock frequency is inaccuracy
     * |        |          |Once this bit is set to 1, the auto trim operation stopped and FREQSEL(SYS_TCTL48M[1:0]) will be cleared to 00 by hardware automatically if CESTOPEN(SYS_TCTL48M[8]) is set to 1.
     * |        |          |If this bit is set and CLKEIEN(SYS_TIEN48M[2]) is high, an interrupt will be triggered to notify the clock frequency is inaccuracy.
     * |        |          |0 = Clock frequency is accurate.
     * |        |          |1 = Clock frequency is inaccurate.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[3]     |OVBDIF    |Over Boundary Status
     * |        |          |When the over boundary function is set, if there occurs the over boundary condition, this flag will be set.
     * |        |          |This is a status bit and does not trigger any interrupt.
     * |        |          |0 = Over boundary condition did not occur.
     * |        |          |1 = Over boundary condition occurred.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * @var SYS_T::TCTL12M
     * Offset: 0x120  HIRC 12M Trim Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |FREQSEL   |Trim Frequency Selection
     * |        |          |This field indicates the target frequency of 12 MHz internal high speed RC oscillator (HIRC) auto trim.
     * |        |          |During auto trim operation, if clock error detected with CESTOPEN is set to 1 or trim retry limitation count reached, this field will be cleared to 00 automatically.
     * |        |          |00 = Disable HIRC auto trim function.
     * |        |          |01 = Enable HIRC auto trim function and trim HIRC to 12 MHz.
     * |        |          |10 = Reserved..
     * |        |          |11 = Reserved.
     * |[3:2]   |ACCURSEL  |Trim Accuracy Selection
     * |        |          |This field indicates the target frequency accuracy of 12 MHz internal high speed RC oscillator (IRC) auto trim.
     * |        |          |00 = Accuracy is +-0.25% deviation within all temperature ranges.
     * |        |          |01 = Accuracy is +-0.50% deviation within all temperature ranges.
     * |        |          |10 = Accuracy is +-0.75% deviation within all temperature ranges.
     * |        |          |11 = Accuracy is +-1% deviation within all temperature ranges.
     * |[5:4]   |LOOPSEL   |Trim Calculation Loop Selection
     * |        |          |This field defines that trim value calculation is based on how many reference clocks.
     * |        |          |00 = Trim value calculation is based on average difference in 4 clocks of reference clock.
     * |        |          |01 = Trim value calculation is based on average difference in 8 clocks of reference clock.
     * |        |          |10 = Trim value calculation is based on average difference in 16 clocks of reference clock.
     * |        |          |11 = Trim value calculation is based on average difference in 32 clocks of reference clock.
     * |        |          |Note: For example, if LOOPSEL is set as 00, auto trim circuit will calculate trim value based on the average frequency difference in 4 clocks of reference clock.
     * |[7:6]   |RETRYCNT  |Trim Value Update Limitation Count
     * |        |          |This field defines that how many times the auto trim circuit will try to update the HIRC trim value before the frequency of HIRC locked.
     * |        |          |Once the HIRC locked, the internal trim value update counter will be reset.
     * |        |          |If the trim value update counter reached this limitation value and frequency of HIRC is still not locked, the auto trim operation will be disabled and FREQSEL will be cleared to 00.
     * |        |          |00 = Trim retry count limitation is 64 loops.
     * |        |          |01 = Trim retry count limitation is 128 loops.
     * |        |          |10 = Trim retry count limitation is 256 loops.
     * |        |          |11 = Trim retry count limitation is 512 loops.
     * |[8]     |CESTOPEN  |Clock Error Stop Enable Bit
     * |        |          |0 = The trim operation keeps going if clock is inaccurate.
     * |        |          |1 = The trim operation is stopped if clock is inaccurate.
     * |[9]     |BOUNDEN   |Boundary Enable Bit
     * |        |          |0 = Boundary function Disabled.
     * |        |          |1 = Boundary function Enabled.
     * |[10]    |REFCKSEL  |Reference Clock Selection
     * |        |          |0 = HIRC trim reference clock is from external 32.768 kHz crystal oscillator.
     * |        |          |1 = HIRC trim reference clock is from internal USB synchronous mode.
     * |[20:16] |BOUNDARY  |Boundary Selection
     * |        |          |Fill the boundary range from 0x1 to 0x1F, 0x0 is reserved.
     * |        |          |Note: This field is effective only when the BOUNDEN(SYS_TCTL12M[9]) is enabled.
     * @var SYS_T::TIEN12M
     * Offset: 0x124  HIRC 12M Trim Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TFAILIEN  |Trim Failure Interrupt Enable Bit
     * |        |          |This bit controls if an interrupt will be triggered while HIRC trim value update limitation count reached and HIRC frequency still not locked on target frequency set by FREQSEL(SYS_TCTL12M[1:0]).
     * |        |          |If this bit is high and TFAILIF(SYS_TSTS12M[1]) is set during auto trim operation, an interrupt will be triggered to notify that HIRC trim value update limitation count was reached.
     * |        |          |0 = Disable TFAILIF(SYS_TISTS12M[1]) status to trigger an interrupt to CPU.
     * |        |          |1 = Enable TFAILIF(SYS_TISTS12M[1]) status to trigger an interrupt to CPU.
     * |[2]     |CLKEIEN   |Clock Error Interrupt Enable Bit
     * |        |          |This bit controls if CPU would get an interrupt while clock is inaccuracy during auto trim operation.
     * |        |          |If this bit is set to1, and CLKERRIF(SYS_TISTS12M[2]) is set during auto trim operation, an interrupt will be triggered to notify the clock frequency is inaccuracy.
     * |        |          |0 = Disable CLKERRIF(SYS_TISTS12M[2]) status to trigger an interrupt to CPU.
     * |        |          |1 = Enable CLKERRIF(SYS_TISTS12M[2]) status to trigger an interrupt to CPU.
     * @var SYS_T::TISTS12M
     * Offset: 0x128  HIRC 12M Trim Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FREQLOCK  |HIRC Frequency Lock Status
     * |        |          |This bit indicates the HIRC frequency is locked.
     * |        |          |This is a status bit and does not trigger any interrupt.
     * |        |          |This bit will be set automatically, if the frequency is lock and the RC_TRIM is enabled.
     * |        |          |0 = The internal high-speed oscillator frequency is not locked at 12 MHz yet.
     * |        |          |1 = The internal high-speed oscillator frequency locked at 12 MHz.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[1]     |TFAILIF   |Trim Failure Interrupt Status
     * |        |          |This bit indicates that HIRC trim value update limitation count reached and the HIRC clock frequency is still not locked
     * |        |          |Once this bit is set, the auto trim operation stopped and FREQSEL(SYS_TCTL12M[1:0]) will be cleared to 00 by hardware automatically.
     * |        |          |If this bit is set and TFAILIEN(SYS_TIEN12M[1]) is high, an interrupt will be triggered to notify that HIRC trim value update limitation count was reached.
     * |        |          |0 = Trim value update limitation count did not reach.
     * |        |          |1 = Trim value update limitation count reached and HIRC frequency still not locked.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[2]     |CLKERRIF  |Clock Error Interrupt Status
     * |        |          |When the frequency of 32.768 kHz external low speed crystal oscillator (LXT) or 12 MHz internal high speed RC oscillator (HIRC) is shift larger to unreasonable value, this bit will be set and to be an indicate that clock frequency is inaccuracy
     * |        |          |Once this bit is set to 1, the auto trim operation stopped and FREQSEL(SYS_TCTL12M[1:0]) will be cleared to 00 by hardware automatically if CESTOPEN(SYS_TCTL12M[8]) is set to 1.
     * |        |          |If this bit is set and CLKEIEN(SYS_IRCTIEN[2]) is high, an interrupt will be triggered to notify the clock frequency is inaccuracy.
     * |        |          |0 = Clock frequency is accurate.
     * |        |          |1 = Clock frequency is inaccurate.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[3]     |OVBDIF    |Over Boundary Status
     * |        |          |When the over boundary function is set, if there occurs the over boundary condition, this flag will be set.
     * |        |          |This is a status bit and does not trigger any interrupt.
     * |        |          |0 = Over boundary condition did not occur.
     * |        |          |1 = Over boundary condition occurred.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * @var SYS_T::TCTLMIRC
     * Offset: 0x130  MIRC Trim Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |FREQSEL   |Trim Frequency Selection
     * |        |          |This field indicates the target frequency of 1 MHz internal high speed RC oscillator (MIRC) auto trim.
     * |        |          |During auto trim operation, if clock error detected with CESTOPEN is set to 1 or trim retry limitation count reached, this field will be cleared to 00 automatically.
     * |        |          |00 = Disable MIRC auto trim function.
     * |        |          |01 = Enable MIRC auto trim function and trim MIRC to 1 MHz.
     * |        |          |10 = Reserved..
     * |        |          |11 = Reserved.
     * |[3:2]   |ACCURSEL  |Trim Accuracy Selection
     * |        |          |This field indicates the target frequency accuracy of 1 MHz internal high speed RC oscillator (MIRC) auto trim.
     * |        |          |00 = Accuracy is +-0.25% deviation within all temperature ranges.
     * |        |          |01 = Accuracy is +-0.50% deviation within all temperature ranges.
     * |        |          |10 = Accuracy is +-0.75% deviation within all temperature ranges.
     * |        |          |11 = Accuracy is +-1% deviation within all temperature ranges.
     * |[5:4]   |LOOPSEL   |Trim Calculation Loop Selection
     * |        |          |This field defines that trim value calculation is based on how many reference clocks.
     * |        |          |00 = Trim value calculation is based on average difference in 4 clocks of reference clock.
     * |        |          |01 = Trim value calculation is based on average difference in 8 clocks of reference clock.
     * |        |          |10 = Trim value calculation is based on average difference in 16 clocks of reference clock.
     * |        |          |11 = Trim value calculation is based on average difference in 32 clocks of reference clock.
     * |        |          |Note: For example, if LOOPSEL is set as 00, auto trim circuit will calculate trim value based on the average frequency difference in 4 clocks of reference clock.
     * |[7:6]   |RETRYCNT  |Trim Value Update Limitation Count
     * |        |          |This field defines that how many times the auto trim circuit will try to update the MIRC trim value before the frequency of MIRC locked.
     * |        |          |Once the MIRC locked, the internal trim value update counter will be reset.
     * |        |          |If the trim value update counter reached this limitation value and frequency of MIRC is still not locked, the auto trim operation will be disabled and FREQSEL will be cleared to 00.
     * |        |          |00 = Trim retry count limitation is 64 loops.
     * |        |          |01 = Trim retry count limitation is 128 loops.
     * |        |          |10 = Trim retry count limitation is 256 loops.
     * |        |          |11 = Trim retry count limitation is 512 loops.
     * |[8]     |CESTOPEN  |Clock Error Stop Enable Bit
     * |        |          |0 = The trim operation is kept going if clock is inaccurate.
     * |        |          |1 = The trim operation is stopped if clock is inaccurate.
     * |[9]     |BOUNDEN   |Boundary Enable Bit
     * |        |          |0 = Boundary function Disabled.
     * |        |          |1 = Boundary function Enabled.
     * |[10]    |REFCKSEL  |Reference Clock Selection
     * |        |          |0 = MIRC trim reference clock is from external 32.768 kHz crystal oscillator.
     * |        |          |1 = MIRC trim reference clock is from internal USB synchronous mode.
     * |[20:16] |BOUNDARY  |Boundary Selection
     * |        |          |Fill the boundary range from 0x1 to 0x1F, 0x0 is reserved.
     * |        |          |Note: This field is effective only when the BOUNDEN(SYS_TCTLMIRC [9]) is enabled.
     * @var SYS_T::TIENMIRC
     * Offset: 0x134  MIRC Trim Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TFAILIEN  |Trim Failure Interrupt Enable Bit
     * |        |          |This bit controls if an interrupt will be triggered while MIRC trim value update limitation count reached and MIRC frequency still not locked on target frequency set by FREQSEL(SYS_TCTLMIRC[1:0]).
     * |        |          |If this bit is high and TFAILIF(SYS_TSTSMIRC[1]) is set during auto trim operation, an interrupt will be triggered to notify that MIRC trim value update limitation count was reached.
     * |        |          |0 = Disable TFAILIF(SYS_TISTSMIRC[1]) status to trigger an interrupt to CPU.
     * |        |          |1 = Enable TFAILIF(SYS_TISTSMIRC[1]) status to trigger an interrupt to CPU.
     * |[2]     |CLKEIEN   |Clock Error Interrupt Enable Bit
     * |        |          |This bit controls if CPU would get an interrupt while clock is inaccuracy during auto trim operation.
     * |        |          |If this bit is set to1, and CLKERRIF(SYS_TISTSMIRC[2]) is set during auto trim operation, an interrupt will be triggered to notify the clock frequency is inaccuracy.
     * |        |          |0 = Disable CLKERRIF(SYS_TISTSMIRC[2]) status to trigger an interrupt to CPU.
     * |        |          |1 = Enable CLKERRIF(SYS_TISTSMIRC[2]) status to trigger an interrupt to CPU.
     * @var SYS_T::TISTSMIRC
     * Offset: 0x138  MIRC Trim Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FREQLOCK  |MIRC Frequency Lock Status
     * |        |          |This bit indicates the MIRC frequency is locked.
     * |        |          |This is a status bit and does not trigger any interrupt.
     * |        |          |This bit will be set automatically, if the frequency is lock and the RC_TRIM is enabled.
     * |        |          |0 = The internal high-speed oscillator frequency is not locked at 1 MHz.
     * |        |          |1 = The internal high-speed oscillator frequency locked at 1 MHz.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[1]     |TFAILIF   |Trim Failure Interrupt Status
     * |        |          |This bit indicates that MIRC trim value update limitation count reached and the MIRC clock frequency is still not locked
     * |        |          |Once this bit is set, the auto trim operation stopped and FREQSEL(SYS_TCTLMIRC[1:0]) will be cleared to 00 by hardware automatically.
     * |        |          |If this bit is set and TFAILIEN(SYS_TIENMIRC[1]) is high, an interrupt will be triggered to notify that MIRC trim value update limitation count was reached.
     * |        |          |0 = Trim value update limitation count does not reach.
     * |        |          |1 = Trim value update limitation count reached and MIRC frequency still not locked.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[2]     |CLKERRIF  |Clock Error Interrupt Status
     * |        |          |When the frequency of 32.768 kHz external low speed crystal oscillator (LXT) or internal high speed RC oscillator (MIRC) is shift larger to unreasonable value, this bit will be set and to be an indicate that clock frequency is inaccuracy
     * |        |          |Once this bit is set to 1, the auto trim operation stopped and FREQSEL(SYS_TCTLMIRC[1:0]) will be cleared to 00 by hardware automatically if CESTOPEN(SYS_TCTLMIRC[8]) is set to 1.
     * |        |          |If this bit is set and CLKEIEN(SYS_TIENMIRC[2]) is high, an interrupt will be triggered to notify the clock frequency is inaccuracy.
     * |        |          |0 = Clock frequency is accurate.
     * |        |          |1 = Clock frequency is inaccurate.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[3]     |OVBDIF    |Over Boundary Status
     * |        |          |When the over boundary function is set, if there occurs the over boundary condition, this flag will be set.
     * |        |          |This is a status bit and does not trigger any interrupt.
     * |        |          |0 = Over boundary condition did not occur.
     * |        |          |1 = Over boundary condition occurred.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * @var SYS_T::ACMPRST
     * Offset: 0x200  ACMP Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMP01RST |ACMP0/1 Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the ACMP0/1
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = ACMP0/1 controller normal operation.
     * |        |          |1 = ACMP0/1 controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |ACMP23RST |ACMP2/3 Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the ACMP2/3
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = ACMP2/3 controller normal operation.
     * |        |          |1 = ACMP2/3 controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::AWFRST
     * Offset: 0x204  AWF Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |AWF0RST   |AWF0 Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the AWF0
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = AWF0 controller normal operation.
     * |        |          |1 = AWF0 controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::BPWMRST
     * Offset: 0x208  BPWM Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BPWM0RST  |BPWMx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the BPWMx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = BPWMx controller normal operation.
     * |        |          |1 = BPWMx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |BPWM1RST  |BPWMx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the BPWMx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = BPWMx controller normal operation.
     * |        |          |1 = BPWMx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::CANFDRST
     * Offset: 0x20C  CANFD Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CANFD0RST |CANFDx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the CANFDx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = CANFDx controller normal operation.
     * |        |          |1 = CANFDx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |CANFD1RST |CANFDx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the CANFDx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = CANFDx controller normal operation.
     * |        |          |1 = CANFDx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::CCAPRST
     * Offset: 0x210  CCAP Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CCAP0RST  |CCAPx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the CCAPx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = CCAPx controller normal operation.
     * |        |          |1 = CCAPx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::CRCRST
     * Offset: 0x214  CRC Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CRC0RST   |CRCx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the CRCx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = CRCx controller normal operation.
     * |        |          |1 = CRCx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::CRYPTORST
     * Offset: 0x218  CRYPTO Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CRYPTO0RST|CRYPTOx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the CRYPTOx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = CRYPTOx controller normal operation.
     * |        |          |1 = CRYPTOx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::DACRST
     * Offset: 0x21C  DAC Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DAC01RST  |DAC0/1 Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the DAC0/1
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = DAC0/1 controller normal operation.
     * |        |          |1 = DAC0/1 controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::DMICRST
     * Offset: 0x220  DMIC Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DMIC0RST  |DMICx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the DMICx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = DMICx controller normal operation.
     * |        |          |1 = DMICx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::EADCRST
     * Offset: 0x224  EADC Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EADC0RST  |EADCx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the EADCx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = EADCx controller normal operation.
     * |        |          |1 = EADCx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::EBIRST
     * Offset: 0x228  EBI Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EBI0RST   |EBIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the EBIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = EBIx controller normal operation.
     * |        |          |1 = EBIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::ECAPRST
     * Offset: 0x22C  ECAP Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ECAP0RST  |ECAPx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the ECAPx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = ECAPx controller normal operation.
     * |        |          |1 = ECAPx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |ECAP1RST  |ECAPx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the ECAPx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = ECAPx controller normal operation.
     * |        |          |1 = ECAPx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[2]     |ECAP2RST  |ECAPx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the ECAPx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = ECAPx controller normal operation.
     * |        |          |1 = ECAPx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[3]     |ECAP3RST  |ECAPx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the ECAPx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = ECAPx controller normal operation.
     * |        |          |1 = ECAPx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::EMACRST
     * Offset: 0x230  EMAC Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EMAC0RST  |EMACx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the EMACx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = EMACx controller normal operation.
     * |        |          |1 = EMACx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::EPWMRST
     * Offset: 0x234  EPWM Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EPWM0RST  |EPWMx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the EPWMx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = EPWMx controller normal operation.
     * |        |          |1 = EPWMx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |EPWM1RST  |EPWMx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the EPWMx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = EPWMx controller normal operation.
     * |        |          |1 = EPWMx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::EQEIRST
     * Offset: 0x238  EQEI Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |EQEI0RST  |EQEIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the EQEIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = EQEIx controller normal operation.
     * |        |          |1 = EQEIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |EQEI1RST  |EQEIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the EQEIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = EQEIx controller normal operation.
     * |        |          |1 = EQEIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[2]     |EQEI2RST  |EQEIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the EQEIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = EQEIx controller normal operation.
     * |        |          |1 = EQEIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[3]     |EQEI3RST  |EQEIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the EQEIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = EQEIx controller normal operation.
     * |        |          |1 = EQEIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::FMCRST
     * Offset: 0x23C  FMC Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FMC0RST   |FMCx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the FMCx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = FMCx controller normal operation.
     * |        |          |1 = FMCx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::GDMARST
     * Offset: 0x240  GDMA Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GDMA0RST  |GDMAx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the GDMAx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = GDMAx controller normal operation.
     * |        |          |1 = GDMAx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::GPIORST
     * Offset: 0x244  GPIO Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GPIO0RST  |GPIOx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the GPIOx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = GPIOx controller normal operation.
     * |        |          |1 = GPIOx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::HSOTGRST
     * Offset: 0x248  HSOTG Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |HSOTG0RST |HSOTGx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the HSOTGx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = HSOTGx controller normal operation.
     * |        |          |1 = HSOTGx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::HSUSBDRST
     * Offset: 0x24C  HSUSBD Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |HSUSBD0RST|HSUSBDx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the HSUSBDx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = HSUSBDx controller normal operation.
     * |        |          |1 = HSUSBDx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::HSUSBHRST
     * Offset: 0x250  HSUSBH Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |HSUSBH0RST|HSUSBHx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the HSUSBHx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = HSUSBHx controller normal operation.
     * |        |          |1 = HSUSBHx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::I2CRST
     * Offset: 0x254  I2C Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |I2C0RST   |I2Cx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the I2Cx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = I2Cx controller normal operation.
     * |        |          |1 = I2Cx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |I2C1RST   |I2Cx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the I2Cx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = I2Cx controller normal operation.
     * |        |          |1 = I2Cx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[2]     |I2C2RST   |I2Cx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the I2Cx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = I2Cx controller normal operation.
     * |        |          |1 = I2Cx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[3]     |I2C3RST   |I2Cx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the I2Cx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = I2Cx controller normal operation.
     * |        |          |1 = I2Cx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::I2SRST
     * Offset: 0x258  I2S Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |I2S0RST   |I2Sx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the I2Sx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = I2Sx controller normal operation.
     * |        |          |1 = I2Sx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |I2S1RST   |I2Sx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the I2Sx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = I2Sx controller normal operation.
     * |        |          |1 = I2Sx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::I3CRST
     * Offset: 0x25C  I3C Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |I3C0RST   |I3Cx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the I3Cx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = I3Cx controller normal operation.
     * |        |          |1 = I3Cx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::KDFRST
     * Offset: 0x260  KDF Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |KDF0RST   |KDFx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the KDFx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = KDFx controller normal operation.
     * |        |          |1 = KDFx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::KPIRST
     * Offset: 0x264  KPI Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |KPI0RST   |KPIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the KPIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = KPIx controller normal operation.
     * |        |          |1 = KPIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::KSRST
     * Offset: 0x268  KS Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |KS0RST    |KSx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the KSx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = KSx controller normal operation.
     * |        |          |1 = KSx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::LPADCRST
     * Offset: 0x26C  Low Power ADC Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPADC0RST |LPADCx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the LPADCx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = LPADCx controller normal operation.
     * |        |          |1 = LPADCx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::LPPDMARST
     * Offset: 0x270  Low Power PDMA Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPPDMA0RST|LPPDMAx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the LPPDMAx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = LPPDMAx controller normal operation.
     * |        |          |1 = LPPDMAx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::LPGPIORST
     * Offset: 0x274  Low Power GPIO Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPGPIO0RST|LPGPIOx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the LPGPIOx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = LPGPIOx controller normal operation.
     * |        |          |1 = LPGPIOx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::LPI2CRST
     * Offset: 0x278  Low Power I2C Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPI2C0RST |LPI2Cx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the LPI2Cx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = LPI2Cx controller normal operation.
     * |        |          |1 = LPI2Cx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::LPSPIRST
     * Offset: 0x27C  Low Power SPI Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPSPI0RST |LPSPIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the LPSPIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = LPSPIx controller normal operation.
     * |        |          |1 = LPSPIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::LPTMRRST
     * Offset: 0x280  Low Power TMR Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPTMR0RST |LPTMRx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the LPTMRx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = LPTMRx controller normal operation.
     * |        |          |1 = LPTMRx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |LPTMR1RST |LPTMRx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the LPTMRx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = LPTMRx controller normal operation.
     * |        |          |1 = LPTMRx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::LPUARTRST
     * Offset: 0x284  Low Power UART Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LPUART0RST|LPUARTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the LPUARTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = LPUARTx controller normal operation.
     * |        |          |1 = LPUARTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::OTFCRST
     * Offset: 0x288  OTFC Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |OTFC0RST  |OTFCx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the OTFCx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = OTFCx controller normal operation.
     * |        |          |1 = OTFCx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::OTGRST
     * Offset: 0x28C  OTG Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |OTG0RST   |OTGx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the OTGx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = OTGx controller normal operation.
     * |        |          |1 = OTGx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::PDMARST
     * Offset: 0x290  PDMA Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PDMA0RST  |PDMAx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the PDMAx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = PDMAx controller normal operation.
     * |        |          |1 = PDMAx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |PDMA1RST  |PDMAx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the PDMAx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = PDMAx controller normal operation.
     * |        |          |1 = PDMAx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::PSIORST
     * Offset: 0x294  PSIO Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PSIO0RST  |PSIOx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the PSIOx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = PSIOx controller normal operation.
     * |        |          |1 = PSIOx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::QSPIRST
     * Offset: 0x298  QSPI Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |QSPI0RST  |QSPIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the QSPIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = QSPIx controller normal operation.
     * |        |          |1 = QSPIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |QSPI1RST  |QSPIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the QSPIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = QSPIx controller normal operation.
     * |        |          |1 = QSPIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::RTCRST
     * Offset: 0x29C  RTC Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RTC0RST   |RTCx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the RTCx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = RTCx controller normal operation.
     * |        |          |1 = RTCx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::SCRST
     * Offset: 0x2A0  SC Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SC0RST    |SCx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the SCx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = SCx controller normal operation.
     * |        |          |1 = SCx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |SC1RST    |SCx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the SCx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = SCx controller normal operation.
     * |        |          |1 = SCx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[2]     |SC2RST    |SCx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the SCx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = SCx controller normal operation.
     * |        |          |1 = SCx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::SCURST
     * Offset: 0x2A4  SCU Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SCU0RST   |SCUx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the SCUx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = SCUx controller normal operation.
     * |        |          |1 = SCxU controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::SDHRST
     * Offset: 0x2A8  SDH Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SDH0RST   |SDHx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the SDHx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = SDHx controller normal operation.
     * |        |          |1 = SDHx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |SDH1RST   |SDHx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the SDHx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = SDHx controller normal operation.
     * |        |          |1 = SDHx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::SPIRST
     * Offset: 0x2AC  SPI Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SPI0RST   |SPIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the SPIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = SPIx controller normal operation.
     * |        |          |1 = SPIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |SPI1RST   |SPIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the SPIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = SPIx controller normal operation.
     * |        |          |1 = SPIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[2]     |SPI2RST   |SPIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the SPIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = SPIx controller normal operation.
     * |        |          |1 = SPIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[3]     |SPI3RST   |SPIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the SPIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = SPIx controller normal operation.
     * |        |          |1 = SPIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::SPIMRST
     * Offset: 0x2B0  SPIM Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SPIM0RST  |SPIMx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the SPIMx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = SPIMx controller normal operation.
     * |        |          |1 = SPIMx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::TMRRST
     * Offset: 0x2C0  Timer Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TMR0RST   |TMRx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the TMRx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = TMRx controller normal operation.
     * |        |          |1 = TMRx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |TMR1RST   |TMRx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the TMRx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = TMRx controller normal operation.
     * |        |          |1 = TMRx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[2]     |TMR2RST   |TMRx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the TMRx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = TMRx controller normal operation.
     * |        |          |1 = TMRx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[3]     |TMR3RST   |TMRx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the TMRx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = TMRx controller normal operation.
     * |        |          |1 = TMRx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::TRNGRST
     * Offset: 0x2C4  TRNG Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TRNG0RST  |TRNGx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the TRNGx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = TRNGx controller normal operation.
     * |        |          |1 = TRNGx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::TTMRRST
     * Offset: 0x2C8  Tick Timer Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TTMR0RST  |TTMRx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the TTMRx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = TTMRx controller normal operation.
     * |        |          |1 = TTMRx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |TTMR1RST  |TTMRx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the TTMRx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = TTMRx controller normal operation.
     * |        |          |1 = TTMRx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::UARTRST
     * Offset: 0x2CC  UART Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |UART0RST  |UARTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the UARTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = UARTx controller normal operation.
     * |        |          |1 = UARTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |UART1RST  |UARTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the UARTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = UARTx controller normal operation.
     * |        |          |1 = UARTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[2]     |UART2RST  |UARTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the UARTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = UARTx controller normal operation.
     * |        |          |1 = UARTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[3]     |UART3RST  |UARTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the UARTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = UARTx controller normal operation.
     * |        |          |1 = UARTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[4]     |UART4RST  |UARTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the UARTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = UARTx controller normal operation.
     * |        |          |1 = UARTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[5]     |UART5RST  |UARTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the UARTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = UARTx controller normal operation.
     * |        |          |1 = UARTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[6]     |UART6RST  |UARTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the UARTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = UARTx controller normal operation.
     * |        |          |1 = UARTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[7]     |UART7RST  |UARTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the UARTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = UARTx controller normal operation.
     * |        |          |1 = UARTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[8]     |UART8RST  |UARTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the UARTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = UARTx controller normal operation.
     * |        |          |1 = UARTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[9]     |UART9RST  |UARTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the UARTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = UARTx controller normal operation.
     * |        |          |1 = UARTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::USBDRST
     * Offset: 0x2D0  USB Device Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |USBD0RST  |USBDx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the USBDx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = USBDx controller normal operation.
     * |        |          |1 = USBDx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::USBHRST
     * Offset: 0x2D4  USB Host Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |USBH0RST  |USBHx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the USBHx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = USBHx controller normal operation.
     * |        |          |1 = USBHx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::USCIRST
     * Offset: 0x2D8  USCI Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |USCI0RST  |USCIx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the USCIx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = USCIx controller normal operation.
     * |        |          |1 = USCIx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::UTCPDRST
     * Offset: 0x2DC  UTCPD Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |UTCPD0RST |UTCPDx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the UTCPDx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = UTCPDx controller normal operation.
     * |        |          |1 = UTCPDx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::WWDTRST
     * Offset: 0x2E0  WWDT Reset Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDT0RST  |WWDTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the WWDTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = WWDTx controller normal operation.
     * |        |          |1 = WWDTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[1]     |WWDT1RST  |WWDTx Controller Reset (Write Protect)
     * |        |          |Setting this bit to 1 will generate a reset signal to the WWDTx
     * |        |          |User needs to set this bit to 0 to release from reset state.
     * |        |          |0 = WWDTx controller normal operation.
     * |        |          |1 = WWDTx controller reset.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * @var SYS_T::GPA_MFP0
     * Offset: 0x300  GPIOA Multiple Function Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PA0MFP    |PA.0 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PA1MFP    |PA.1 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PA2MFP    |PA.2 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PA3MFP    |PA.3 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPA_MFP1
     * Offset: 0x304  GPIOA Multiple Function Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PA4MFP    |PA.4 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PA5MFP    |PA.5 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PA6MFP    |PA.6 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PA7MFP    |PA.7 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPA_MFP2
     * Offset: 0x308  GPIOA Multiple Function Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PA8MFP    |PA.8 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PA9MFP    |PA.9 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PA10MFP   |PA.10 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PA11MFP   |PA.11 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPA_MFP3
     * Offset: 0x30C  GPIOA Multiple Function Control Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PA12MFP   |PA.12 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PA13MFP   |PA.13 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PA14MFP   |PA.14 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PA15MFP   |PA.15 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPB_MFP0
     * Offset: 0x310  GPIOB Multiple Function Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PB0MFP    |PB.0 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PB1MFP    |PB.1 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PB2MFP    |PB.2 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PB3MFP    |PB.3 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPB_MFP1
     * Offset: 0x314  GPIOB Multiple Function Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PB4MFP    |PB.4 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PB5MFP    |PB.5 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PB6MFP    |PB.6 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PB7MFP    |PB.7 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPB_MFP2
     * Offset: 0x318  GPIOB Multiple Function Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PB8MFP    |PB.8 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PB9MFP    |PB.9 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PB10MFP   |PB.10 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PB11MFP   |PB.11 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPB_MFP3
     * Offset: 0x31C  GPIOB Multiple Function Control Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PB12MFP   |PB.12 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PB13MFP   |PB.13 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PB14MFP   |PB.14 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PB15MFP   |PB.15 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPC_MFP0
     * Offset: 0x320  GPIOC Multiple Function Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PC0MFP    |PC.0 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PC1MFP    |PC.1 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PC2MFP    |PC.2 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PC3MFP    |PC.3 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPC_MFP1
     * Offset: 0x324  GPIOC Multiple Function Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PC4MFP    |PC.4 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PC5MFP    |PC.5 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PC6MFP    |PC.6 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PC7MFP    |PC.7 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPC_MFP2
     * Offset: 0x328  GPIOC Multiple Function Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PC8MFP    |PC.8 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PC9MFP    |PC.9 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PC10MFP   |PC.10 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PC11MFP   |PC.11 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPC_MFP3
     * Offset: 0x32C  GPIOC Multiple Function Control Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PC12MFP   |PC.12 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PC13MFP   |PC.13 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PC14MFP   |PC.14 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPD_MFP0
     * Offset: 0x330  GPIOD Multiple Function Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PD0MFP    |PD.0 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PD1MFP    |PD.1 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PD2MFP    |PD.2 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PD3MFP    |PD.3 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPD_MFP1
     * Offset: 0x334  GPIOD Multiple Function Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PD4MFP    |PD.4 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PD5MFP    |PD.5 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PD6MFP    |PD.6 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PD7MFP    |PD.7 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPD_MFP2
     * Offset: 0x338  GPIOD Multiple Function Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PD8MFP    |PD.8 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PD9MFP    |PD.9 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PD10MFP   |PD.10 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PD11MFP   |PD.11 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPD_MFP3
     * Offset: 0x33C  GPIOD Multiple Function Control Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PD12MFP   |PD.12 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PD13MFP   |PD.13 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PD14MFP   |PD.14 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPE_MFP0
     * Offset: 0x340  GPIOE Multiple Function Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PE0MFP    |PE.0 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PE1MFP    |PE.1 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PE2MFP    |PE.2 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PE3MFP    |PE.3 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPE_MFP1
     * Offset: 0x344  GPIOE Multiple Function Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PE4MFP    |PE.4 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PE5MFP    |PE.5 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PE6MFP    |PE.6 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PE7MFP    |PE.7 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPE_MFP2
     * Offset: 0x348  GPIOE Multiple Function Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PE8MFP    |PE.8 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PE9MFP    |PE.9 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PE10MFP   |PE.10 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PE11MFP   |PE.11 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPE_MFP3
     * Offset: 0x34C  GPIOE Multiple Function Control Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PE12MFP   |PE.12 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PE13MFP   |PE.13 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PE14MFP   |PE.14 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PE15MFP   |PE.15 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPF_MFP0
     * Offset: 0x350  GPIOF Multiple Function Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PF0MFP    |PF.0 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PF1MFP    |PF.1 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PF2MFP    |PF.2 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PF3MFP    |PF.3 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPF_MFP1
     * Offset: 0x354  GPIOF Multiple Function Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PF4MFP    |PF.4 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PF5MFP    |PF.5 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PF6MFP    |PF.6 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PF7MFP    |PF.7 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPF_MFP2
     * Offset: 0x358  GPIOF Multiple Function Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PF8MFP    |PF.8 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PF9MFP    |PF.9 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PF10MFP   |PF.10 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PF11MFP   |PF.11 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPG_MFP0
     * Offset: 0x360  GPIOG Multiple Function Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PG0MFP    |PG.0 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PG1MFP    |PG.1 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PG2MFP    |PG.2 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PG3MFP    |PG.3 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPG_MFP1
     * Offset: 0x364  GPIOG Multiple Function Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PG4MFP    |PG.4 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PG5MFP    |PG.5 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PG6MFP    |PG.6 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PG7MFP    |PG.7 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPG_MFP2
     * Offset: 0x368  GPIOG Multiple Function Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PG8MFP    |PG.8 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PG9MFP    |PG.9 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PG10MFP   |PG.10 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PG11MFP   |PG.11 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPG_MFP3
     * Offset: 0x36C  GPIOG Multiple Function Control Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PG12MFP   |PG.12 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PG13MFP   |PG.13 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PG14MFP   |PG.14 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PG15MFP   |PG.15 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPH_MFP0
     * Offset: 0x370  GPIOH Multiple Function Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PH0MFP    |PH.0 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PH1MFP    |PH.1 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PH2MFP    |PH.2 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PH3MFP    |PH.3 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPH_MFP1
     * Offset: 0x374  GPIOH Multiple Function Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PH4MFP    |PH.4 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PH5MFP    |PH.5 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PH6MFP    |PH.6 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PH7MFP    |PH.7 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPH_MFP2
     * Offset: 0x378  GPIOH Multiple Function Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PH8MFP    |PH.8 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PH9MFP    |PH.9 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PH10MFP   |PH.10 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PH11MFP   |PH.11 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPH_MFP3
     * Offset: 0x37C  GPIOH Multiple Function Control Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PH12MFP   |PH.12 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PH13MFP   |PH.13 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PH14MFP   |PH.14 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PH15MFP   |PH.15 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPI_MFP1
     * Offset: 0x384  GPIOI Multiple Function Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[20:16] |PI6MFP    |PI.6 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PI7MFP    |PI.7 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPI_MFP2
     * Offset: 0x388  GPIOI Multiple Function Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PI8MFP    |PI.8 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PI9MFP    |PI.9 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PI10MFP   |PI.10 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PI11MFP   |PI.11 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPI_MFP3
     * Offset: 0x38C  GPIOI Multiple Function Control Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PI12MFP   |PI.12 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PI13MFP   |PI.13 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PI14MFP   |PI.14 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PI15MFP   |PI.15 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPJ_MFP0
     * Offset: 0x390  GPIOJ Multiple Function Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PJ0MFP    |PJ.0 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PJ1MFP    |PJ.1 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PJ2MFP    |PJ.2 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PJ3MFP    |PJ.3 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPJ_MFP1
     * Offset: 0x394  GPIOJ Multiple Function Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PJ4MFP    |PJ.4 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PJ5MFP    |PJ.5 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PJ6MFP    |PJ.6 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PJ7MFP    |PJ.7 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPJ_MFP2
     * Offset: 0x398  GPIOJ Multiple Function Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PJ8MFP    |PJ.8 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PJ9MFP    |PJ.9 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[20:16] |PJ10MFP   |PJ.10 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[28:24] |PJ11MFP   |PJ.11 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::GPJ_MFP3
     * Offset: 0x39C  GPIOJ Multiple Function Control Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |PJ12MFP   |PJ.12 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * |[12:8]  |PJ13MFP   |PJ.13 Multi-function Pin Selection
     * |        |          |Note: These bits are not retained when D2 power is turned off.
     * @var SYS_T::NMIEN
     * Offset: 0x500  NMI Source Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BODOUT    |BOD NMI Source Enable (Write Protect)
     * |        |          |0 = BOD NMI source Disabled.
     * |        |          |1 = BOD NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * |[1]     |IRCINT    |IRC TRIM NMI Source Enable (Write Protect)
     * |        |          |0 = IRC TRIM NMI source Disabled.
     * |        |          |1 = IRC TRIM NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D0 power is turned off.
     * |[2]     |PWRWUINT  |Power-down Mode Wake-up NMI Source Enable (Write Protect)
     * |        |          |0 = Power-down mode wake-up NMI source Disabled.
     * |        |          |1 = Power-down mode wake-up NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * |[3]     |SRAMPERR  |SRAM Parity Check Error NMI Source Enable (Write Protect)
     * |        |          |0 = SRAM parity check error NMI source Disabled.
     * |        |          |1 = SRAM parity check error NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * |[4]     |CLKFAIL   |Clock Fail Detected NMI Source Enable (Write Protect)
     * |        |          |0 = Clock fail detected interrupt NMI source Disabled.
     * |        |          |1 = Clock fail detected interrupt NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * |[6]     |RTCINT    |RTC NMI Source Enable (Write Protect)
     * |        |          |0 = RTC NMI source Disabled.
     * |        |          |1 = RTC NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * |[7]     |RTCTMPINT |RTC Tamper Interrupt NMI Source Enable (Write Protect)
     * |        |          |0 = Backup register tamper detected interrupt NMI source Disabled.
     * |        |          |1 = Backup register tamper detected interrupt NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * |[8]     |EINT0     |External Interrupt From PA.6, or PB.5 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PA.6, or PB.5 pin NMI source Disabled.
     * |        |          |1 = External interrupt from PA.6, or PB.5 pin NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * |[9]     |EINT1     |External Interrupt From PA.7 or PB.4 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PA.7 or PB.4 pin NMI source Disabled.
     * |        |          |1 = External interrupt from PA.7 or P4.4 pin NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * |[10]    |EINT2     |External Interrupt From PB.3 or PC.6 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PB.3 or PC.6 pin NMI source Disabled.
     * |        |          |1 = External interrupt from PB.3 or PC.6 pin NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * |[11]    |EINT3     |External Interrupt From PB.2 or PC.7 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PB.2 or PC.7pin NMI source Disabled.
     * |        |          |1 = External interrupt from PB.2 or PC.7 pin NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * |[12]    |EINT4     |External Interrupt From PA.8 or PB.6 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PA.8 or PB.6 pin NMI source Disabled.
     * |        |          |1 = External interrupt from PA.8 or PB.6 pin NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * |[13]    |EINT5     |External Interrupt From PB.7 or PD.12 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PB.7 or PD.12 pin NMI source Disabled.
     * |        |          |1 = External interrupt from PB.7 or PD.12 pin NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * |[14]    |EINT6     |External Interrupt From PB.8 or PD.11 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PB.8 or PD.11 pin NMI source Disabled.
     * |        |          |1 = External interrupt from PB.8 or PD.11 pin NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * |[15]    |EINT7     |External Interrupt From PB.9 or PD.10 Pin NMI Source Enable (Write Protect)
     * |        |          |0 = External interrupt from PB.9 or PD.10 pin NMI source Disabled.
     * |        |          |1 = External interrupt from PB.9 or PD.10 pin NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * |[16]    |UART0INT  |UART0 NMI Source Enable (Write Protect)
     * |        |          |0 = UART0 NMI source Disabled.
     * |        |          |1 = UART0 NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * |[17]    |UART1INT  |UART1 NMI Source Enable (Write Protect)
     * |        |          |0 = UART1 NMI source Disabled.
     * |        |          |1 = UART1 NMI source Enabled.
     * |        |          |Note 1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note 2: This bit is not retained when D1 power is turned off.
     * @var SYS_T::NMISTS
     * Offset: 0x504  NMI Source Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BODOUT    |BOD Interrupt Flag (Read Only)
     * |        |          |0 = BOD interrupt is deasserted.
     * |        |          |1 = BOD interrupt is asserted.
     * |[1]     |IRCINT    |IRC TRIM Interrupt Flag (Read Only)
     * |        |          |0 = HIRC TRIM interrupt is deasserted.
     * |        |          |1 = HIRC TRIM interrupt is asserted.
     * |[2]     |PWRWUINT  |Power-down Mode Wake-up Interrupt Flag (Read Only)
     * |        |          |0 = Power-down mode wake-up interrupt is deasserted.
     * |        |          |1 = Power-down mode wake-up interrupt is asserted.
     * |[3]     |SRAMPERR  |SRAM Parity Check Error Interrupt Flag (Read Only)
     * |        |          |0 = SRAM parity check error interrupt is deasserted.
     * |        |          |1 = SRAM parity check error interrupt is asserted.
     * |[4]     |CLKFAIL   |Clock Fail Detected Interrupt Flag (Read Only)
     * |        |          |0 = Clock fail detected interrupt is deasserted.
     * |        |          |1 = Clock fail detected interrupt is asserted.
     * |[6]     |RTCINT    |RTC Interrupt Flag (Read Only)
     * |        |          |0 = RTC interrupt is deasserted.
     * |        |          |1 = RTC interrupt is asserted.
     * |[7]     |RTCTMPINT |RTC Tamper Interrupt Flag (Read Only)
     * |        |          |0 = Backup register tamper detected interrupt is deasserted.
     * |        |          |1 = Backup register tamper detected interrupt is asserted.
     * |[8]     |EINT0     |External Interrupt From PA.6, or PB.5 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from PA.6, or PB.5 interrupt is deasserted.
     * |        |          |1 = External Interrupt from PA.6, or PB.5 interrupt is asserted.
     * |[9]     |EINT1     |External Interrupt From PA.7, or PB.4 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from PA.7, or PB.4 interrupt is deasserted.
     * |        |          |1 = External Interrupt from PA.7, or PB.4 interrupt is asserted.
     * |[10]    |EINT2     |External Interrupt From PB.3 or PC.6 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from PB.3 or PC.6 interrupt is deasserted.
     * |        |          |1 = External Interrupt from PB.3 or PC.6 interrupt is asserted.
     * |[11]    |EINT3     |External Interrupt From PB.2 or PC.7 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from PB.2 or PC.7 interrupt is deasserted.
     * |        |          |1 = External Interrupt from PB.2 or PC.7 interrupt is asserted.
     * |[12]    |EINT4     |External Interrupt From PA.8 or PB.6 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from PA.8 or PB.6 interrupt is deasserted.
     * |        |          |1 = External Interrupt from PA.8 or PB.6 interrupt is asserted.
     * |[13]    |EINT5     |External Interrupt From PB.7 or PD.12 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from PB.7 or PD.12 interrupt is deasserted.
     * |        |          |1 = External Interrupt from PB.7 or PD.12 interrupt is asserted.
     * |[14]    |EINT6     |External Interrupt From PB.8 or PD.11 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from PB.8 or PD.11 interrupt is deasserted.
     * |        |          |1 = External Interrupt from PB.8 or PD.11 interrupt is asserted.
     * |[15]    |EINT7     |External Interrupt From PB.9 or PD.10 Pin Interrupt Flag (Read Only)
     * |        |          |0 = External Interrupt from PB.9 or PD.10 interrupt is deasserted.
     * |        |          |1 = External Interrupt from PB.9 or PD.10 interrupt is asserted.
     * |[16]    |UART0INT  |UART0 Interrupt Flag (Read Only)
     * |        |          |0 = UART1 interrupt is deasserted.
     * |        |          |1 = UART1 interrupt is asserted.
     * |[17]    |UART1INT  |UART1 Interrupt Flag (Read Only)
     * |        |          |0 = UART1 interrupt is deasserted.
     * |        |          |1 = UART1 interrupt is asserted.
     * @var SYS_T::SRAMBCTL
     * Offset: 0xE90  System SRAM BIST Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SR0BIST   |System SRAM 0 BIST Enable Bit (Write Protect)
     * |        |          |This bit enables System SRAM 0 BIST function.
     * |        |          |0 = System SRAM 0 BIST Disabled.
     * |        |          |1 = System SRAM 0 BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |SR1BIST   |System SRAM 1 BIST Enable Bit (Write Protect)
     * |        |          |This bit enables System SRAM 1 BIST function.
     * |        |          |0 = System SRAM 1 BIST Disabled.
     * |        |          |1 = System SRAM 1 BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2]     |SR2BIST   |System SRAM 2 BIST Enable Bit (Write Protect)
     * |        |          |This bit enables System SRAM 2 BIST function.
     * |        |          |0 = System SRAM 2 BIST Disabled.
     * |        |          |1 = System SRAM 2 BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[3]     |SR3BIST   |System SRAM 3 BIST Enable Bit (Write Protect)
     * |        |          |This bit enables System SRAM 3 BIST function.
     * |        |          |0 = System SRAM 3 BIST Disabled.
     * |        |          |1 = System SRAM 3 BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[4]     |LPSRBIST  |Low Power System SRAM BIST Enable Bit (Write Protect)
     * |        |          |This bit enables Low Power System SRAM BIST function.
     * |        |          |0 = Low Power System SRAM BIST Disabled.
     * |        |          |1 = Low Power System SRAM BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[8]     |CAN0BIST  |CAN0 SRAM BIST Enable Bit (Write Protect)
     * |        |          |This bit enables CAN0 SRAM BIST function.
     * |        |          |0 = CAN0 SRAM BIST Disabled.
     * |        |          |1 = CAN0 SRAM BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[9]     |CAN1BIST  |CAN1 SRAM BIST Enable Bit (Write Protect)
     * |        |          |This bit enables CAN1 SRAM BIST function.
     * |        |          |0 = CAN1 SRAM BIST Disabled.
     * |        |          |1 = CAN1 SRAM BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[10]    |SPIM0BIST |SPIM0 SRAM BIST Enable Bit (Write Protect)
     * |        |          |This bit enables SPIM0 SRAM BIST function.
     * |        |          |0 = SPIM0 SRAM BIST Disabled.
     * |        |          |1 = SPIM0 SRAM BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[12]    |CCAPBIST  |CCAP SRAM BIST Enable Bit (Write Protect)
     * |        |          |This bit enables CCAP SRAM BIST function.
     * |        |          |0 = CCAP SRAM BIST Disabled.
     * |        |          |1 = CCAP SRAM BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[13]    |I3CBIST   |I3C SRAM BIST Enable Bit (Write Protect)
     * |        |          |This bit enables I3C SRAM BIST function.
     * |        |          |0 = I3C SRAM BIST Disabled.
     * |        |          |1 = I3C SRAM BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[14]    |KSBIST    |KS SRAM BIST Enable Bit (Write Protect)
     * |        |          |This bit enables KS SRAM BIST function.
     * |        |          |0 = KS SRAM BIST Disabled.
     * |        |          |1 = KS SRAM BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[15]    |RSABIST   |RSA SRAM BIST Enable Bit (Write Protect)
     * |        |          |This bit enables RSA SRAM BIST function.
     * |        |          |0 = RSA SRAM BIST Disabled.
     * |        |          |1 = RSA SRAM BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[16]    |DMICBIST  |DMIC SRAM BIST Enable Bit (Write Protect)
     * |        |          |This bit enables DMIC SRAM BIST function.
     * |        |          |0 = DMIC SRAM BIST Disabled.
     * |        |          |1 = DMIC SRAM BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[17]    |EMACBIST  |EMAC SRAM BIST Enable Bit (Write Protect)
     * |        |          |This bit enables EMAC SRAM BIST function.
     * |        |          |0 = EMAC SRAM BIST Disabled.
     * |        |          |1 = EMAC SRAM BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[18]    |NPUBIST   |NPU SRAM BIST Enable Bit (Write Protect)
     * |        |          |This bit enables NPU SRAM BIST function.
     * |        |          |0 = NPU SRAM BIST Disabled.
     * |        |          |1 = NPU SRAM BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[24]    |USBDBIST  |USBD SRAM BIST Enable Bit (Write Protect)
     * |        |          |This bit enables USBD SRAM BIST function.
     * |        |          |0 = USBD SRAM BIST Disabled.
     * |        |          |1 = USBD SRAM BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[25]    |HSUSBDBIST|HSUSBD SRAM BIST Enable Bit (Write Protect)
     * |        |          |This bit enables HSUSBD SRAM BIST function.
     * |        |          |0 = HSUSBD SRAM BIST Disabled.
     * |        |          |1 = HSUSBD SRAM BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[26]    |HSUSBHBIST|HSUSBH SRAM BIST Enable Bit (Write Protect)
     * |        |          |This bit enables HSUSBH SRAM BIST function.
     * |        |          |0 = HSUSBH SRAM BIST Disabled.
     * |        |          |1 = HSUSBH SRAM BIST Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::SRAMBFF
     * Offset: 0xE94  System SRAM BIST Finish Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SR0BFF    |System SRAM 0 BIST Finish Flag (Read Only)
     * |        |          |0 = System SRAM 0 BIST is active.
     * |        |          |1 = System SRAM 0 BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when SR0BIST(SYS_SRAMBCTL[0]) set to 0.
     * |[1]     |SR1BFF    |System SRAM 1 BIST Finish Flag (Read Only)
     * |        |          |0 = System SRAM 1 BIST is active.
     * |        |          |1 = System SRAM 1 BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when SR1BIST(SYS_SRAMBCTL[1]) set to 0.
     * |[2]     |SR2BFF    |System SRAM 2 BIST Finish Flag (Read Only)
     * |        |          |0 = System SRAM 2 BIST is active.
     * |        |          |1 = System SRAM 2 BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when SR2BIST(SYS_SRAMBCTL[2]) set to 0.
     * |[3]     |SR3BFF    |System SRAM 3 BIST Finish Flag (Read Only)
     * |        |          |0 = System SRAM 3 BIST is active.
     * |        |          |1 = System SRAM 3 BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when SR3BIST(SYS_SRAMBCTL[3]) set to 0.
     * |[4]     |LPSRBFF   |Low Power System SRAM BIST Finish Flag (Read Only)
     * |        |          |0 = Low Power System SRAM BIST is active.
     * |        |          |1 = Low Power System SRAM BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when LPSRBIST(SYS_SRAMBCTL[4]) set to 0.
     * |[8]     |CAN0BFF   |CAN0 SRAM BIST Finish Flag (Read Only)
     * |        |          |0 = CAN0 SRAM BIST is active.
     * |        |          |1 = CAN0 SRAM BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when CAN0BIST(SYS_SRAMBCTL[8]) set to 0.
     * |[9]     |CAN1BFF   |CAN1 SRAM BIST Finish Flag (Read Only)
     * |        |          |0 = CAN1 SRAM BIST is active.
     * |        |          |1 = CAN1 SRAM BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when CAN1BIST(SYS_SRAMBCTL[9]) set to 0.
     * |[10]    |SPIM0BFF  |SPIM0 SRAM BIST Finish Flag (Read Only)
     * |        |          |0 = SPIM0 SRAM BIST is active.
     * |        |          |1 = SPIM0 SRAM BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when SPIM0BIST(SYS_SRAMBCTL[10]) set to 0.
     * |[12]    |CCAPBFF   |CCAP SRAM BIST Finish Flag (Read Only)
     * |        |          |0 = CCAP SRAM BIST is active.
     * |        |          |1 = CCAP SRAM BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when CCAPBIST(SYS_SRAMBCTL[12]) set to 0.
     * |[13]    |I3CBFF    |I3C SRAM BIST Finish Flag (Read Only)
     * |        |          |0 = I3C SRAM BIST is active.
     * |        |          |1 = I3C SRAM BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when I3CBIST(SYS_SRAMBCTL[13]) set to 0.
     * |[14]    |KSBFF     |KS SRAM BIST Finish Flag (Read Only)
     * |        |          |0 = KS SRAM BIST is active.
     * |        |          |1 = KS SRAM BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when KSBIST(SYS_SRAMBCTL[14]) set to 0.
     * |[15]    |RSABFF    |RSA SRAM BIST Finish Flag (Read Only)
     * |        |          |0 = RSA SRAM BIST is active.
     * |        |          |1 = RSA SRAM BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when RSABIST(SYS_SRAMBCTL[15]) set to 0.
     * |[16]    |DMICBFF   |DMIC SRAM BIST Finish Flag (Read Only)
     * |        |          |0 = DMIC SRAM BIST is active.
     * |        |          |1 = DMIC SRAM BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when DMICBIST(SYS_SRAMBCTL[16]) set to 0.
     * |[17]    |EMACBFF   |EMAC SRAM BIST Finish Flag (Read Only)
     * |        |          |0 = EMAC SRAM BIST is active.
     * |        |          |1 = EMAC SRAM BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when EMACBIST(SYS_SRAMBCTL[17]) set to 0.
     * |[18]    |NPUBFF    |NPU SRAM BIST Finish Flag (Read Only)
     * |        |          |0 = NPU SRAM BIST is active.
     * |        |          |1 = NPU SRAM BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when NPUBIST(SYS_SRAMBCTL[18]) set to 0.
     * |[24]    |USBDBFF   |USBD SRAM BIST Finish Flag (Read Only)
     * |        |          |0 = USBD SRAM BIST is active.
     * |        |          |1 = USBD SRAM BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when USBDBIST(SYS_SRAMBCTL[24]) set to 0.
     * |[25]    |HSUSBDBFF |HSUSBD SRAM BIST Finish Flag (Read Only)
     * |        |          |0 = HSUSBD SRAM BIST is active.
     * |        |          |1 = HSUSBD SRAM BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when HSUSBDBIST(SYS_SRAMBCTL[25]) set to 0.
     * |[26]    |HSUSBHBFF |HSUSBH SRAM BIST Finish Flag (Read Only)
     * |        |          |0 = HSUSBH SRAM BIST is active.
     * |        |          |1 = HSUSBH SRAM BIST test finish.
     * |        |          |Note 1: This bit will be clear to 0 when HSUSBHBIST(SYS_SRAMBCTL[26]) set to 0.
     * @var SYS_T::SRAMBRF
     * Offset: 0xE98  System SRAM BIST Result Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SR0BRF    |System SRAM 0 BIST Result Flag (Read Only)
     * |        |          |0 = System SRAM 0 BIST test pass.
     * |        |          |1 = System SRAM 0 BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when SR0BIST(SYS_SRAMBCTL[0]) set to 0.
     * |[1]     |SR1BRF    |System SRAM 1 BIST Result Flag (Read Only)
     * |        |          |0 = System SRAM 1 BIST test pass.
     * |        |          |1 = System SRAM 1 BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when SR1BIST(SYS_SRAMBCTL[1]) set to 0.
     * |[2]     |SR2BRF    |System SRAM 2 BIST Result Flag (Read Only)
     * |        |          |0 = System SRAM 2 BIST test pass.
     * |        |          |1 = System SRAM 2 BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when SR2BIST(SYS_SRAMBCTL[3]) set to 0.
     * |[3]     |SR3BRF    |System SRAM 3 BIST Result Flag (Read Only)
     * |        |          |0 = System SRAM 3 BIST test pass.
     * |        |          |1 = System SRAM 3 BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when SR3BIST(SYS_SRAMBCTL[3]) set to 0.
     * |[4]     |LPSRBRF   |Low Power System SRAM BIST Result Flag (Read Only)
     * |        |          |0 = Low Power System SRAM BIST test pass.
     * |        |          |1 = Low Power System SRAM BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when LPSRBIST(SYS_SRAMBCTL[4]) set to 0.
     * |[8]     |CAN0BRF   |CAN0 SRAM BIST Result Flag (Read Only)
     * |        |          |0 = CAN0 SRAM BIST test pass.
     * |        |          |1 = CAN0 SRAM BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when CAN0BIST(SYS_SRAMBCTL[8]) set to 0.
     * |[9]     |CAN1BRF   |CAN1 SRAM BIST Result Flag (Read Only)
     * |        |          |0 = CAN1 SRAM BIST test pass.
     * |        |          |1 = CAN1 SRAM BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when CAN1BIST(SYS_SRAMBCTL[9]) set to 0.
     * |[10]    |SPIM0BRF  |SPIM0 SRAM BIST Result Flag (Read Only)
     * |        |          |0 = SPIM0 SRAM BIST test pass.
     * |        |          |1 = SPIM0 SRAM BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when SPIM0BIST(SYS_SRAMBCTL[10]) set to 0.
     * |[12]    |CCAPBRF   |CCAP SRAM BIST Result Flag (Read Only)
     * |        |          |0 = CCAP SRAM BIST test pass.
     * |        |          |1 = CCAP SRAM BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when CCAPBIST(SYS_SRAMBCTL[12]) set to 0.
     * |[13]    |I3CBRF    |I3C SRAM BIST Result Flag (Read Only)
     * |        |          |0 = I3C SRAM BIST test pass.
     * |        |          |1 = I3C SRAM BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when I3CBIST(SYS_SRAMBCTL[13]) set to 0.
     * |[14]    |KSBRF     |KS SRAM BIST Result Flag (Read Only)
     * |        |          |0 = KS SRAM BIST test pass.
     * |        |          |1 = KS SRAM BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when KSBIST(SYS_SRAMBCTL[14]) set to 0.
     * |[15]    |RSABRF    |RSA SRAM BIST Result Flag (Read Only)
     * |        |          |0 = RSA SRAM BIST test pass.
     * |        |          |1 = RSA SRAM BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when RSABIST(SYS_SRAMBCTL[15]) set to 0.
     * |[16]    |DMICBRF   |DMIC SRAM BIST Result Flag (Read Only)
     * |        |          |0 = DMIC SRAM BIST test pass.
     * |        |          |1 = DMIC SRAM BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when DMICBIST(SYS_SRAMBCTL[16]) set to 0.
     * |[17]    |EMACBRF   |EMAC SRAM BIST Result Flag (Read Only)
     * |        |          |0 = EMAC SRAM BIST test pass.
     * |        |          |1 = EMAC SRAM BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when EMACBIST(SYS_SRAMBCTL[17]) set to 0.
     * |[18]    |NPUBRF    |NPU SRAM BIST Result Flag (Read Only)
     * |        |          |0 = NPU SRAM BIST test pass.
     * |        |          |1 = NPU SRAM BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when NPUBIST(SYS_SRAMBCTL[18]) set to 0.
     * |[24]    |USBDBRF   |USBD SRAM BIST Result Flag (Read Only)
     * |        |          |0 = USBD SRAM BIST test pass.
     * |        |          |1 = USBD SRAM BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when USBDBIST(SYS_SRAMBCTL[24]) set to 0.
     * |[25]    |HSUSBDBRF |HSUSBD SRAM BIST Result Flag (Read Only)
     * |        |          |0 = HSUSBD SRAM BIST test pass.
     * |        |          |1 = HSUSBD SRAM BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when HSUSBDBIST(SYS_SRAMBCTL[25]) set to 0.
     * |[26]    |HSUSBHBRF |HSUSBH SRAM BIST Result Flag (Read Only)
     * |        |          |0 = HSUSBH SRAM BIST test pass.
     * |        |          |1 = HSUSBH SRAM BIST test fail.
     * |        |          |Note 1: This bit will be clear to 0 when HSUSBHBIST(SYS_SRAMBCTL[26]) set to 0.
     */
    __I  uint32_t PDID;                  /*!< [0x0000] Part Device Identification Number Register                       */
    __IO uint32_t RSTCTL;                /*!< [0x0004] System Reset Control Register                                    */
    __IO uint32_t RSTSTS;                /*!< [0x0008] System Reset Status Register                                     */
    __IO uint32_t VTORSET;               /*!< [0x000c] VTOR Setting Register                                            */
    __IO uint32_t SRAMICTL;              /*!< [0x0010] System SRAM Parity Error Interrupt Enable Control Register       */
    __IO uint32_t SRAMSTS;               /*!< [0x0014] System SRAM Parity Check Status Register                         */
    __I  uint32_t SRAMEADR;              /*!< [0x0018] System SRAM Parity Error Address Register                        */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t BODCTL;                /*!< [0x0020] Brown-out Detector Control Register                              */
    __IO uint32_t BODSTS;                /*!< [0x0024] Brown-out Detector Status Register                               */
    __IO uint32_t PORCTL;                /*!< [0x0028] Power-on Reset Controller Register                               */
    __IO uint32_t VREFCTL;               /*!< [0x002c] VREF Control Register                                            */
    __IO uint32_t IVSCTL;                /*!< [0x0030] Internal Voltage Source Control Register                         */
    __IO uint32_t USBPHY;                /*!< [0x0034] USB PHY Control Register                                         */
    __IO uint32_t UTCPDCTL;              /*!< [0x0038] UTCPD Control Register                                           */
    __I  uint32_t RESERVE1[1];
    __IO uint32_t DBUSCTL;               /*!< [0x0040] Data Bus Control Register                                        */
    __I  uint32_t RESERVE2[15];
    __IO uint32_t GPA_MFOS;              /*!< [0x0080] GPIOA Multiple Function Output Select Register                   */
    __IO uint32_t GPB_MFOS;              /*!< [0x0084] GPIOB Multiple Function Output Select Register                   */
    __IO uint32_t GPC_MFOS;              /*!< [0x0088] GPIOC Multiple Function Output Select Register                   */
    __IO uint32_t GPD_MFOS;              /*!< [0x008c] GPIOD Multiple Function Output Select Register                   */
    __IO uint32_t GPE_MFOS;              /*!< [0x0090] GPIOE Multiple Function Output Select Register                   */
    __IO uint32_t GPF_MFOS;              /*!< [0x0094] GPIOF Multiple Function Output Select Register                   */
    __IO uint32_t GPG_MFOS;              /*!< [0x0098] GPIOG Multiple Function Output Select Register                   */
    __IO uint32_t GPH_MFOS;              /*!< [0x009c] GPIOH Multiple Function Output Select Register                   */
    __IO uint32_t GPI_MFOS;              /*!< [0x00a0] GPIOI Multiple Function Output Select Register                   */
    __IO uint32_t GPJ_MFOS;              /*!< [0x00a4] GPIOJ Multiple Function Output Select Register                   */
    __I  uint32_t RESERVE3[22];
    __IO uint32_t REGLCTL;               /*!< [0x0100] Register Lock Control Register                                   */
    __I  uint32_t RESERVE4[3];
    __IO uint32_t TCTL48M;               /*!< [0x0110] HIRC 48M Trim Control Register                                   */
    __IO uint32_t TIEN48M;               /*!< [0x0114] HIRC 48M Trim Interrupt Enable Register                          */
    __IO uint32_t TISTS48M;              /*!< [0x0118] HIRC 48M Trim Interrupt Status Register                          */
    __I  uint32_t RESERVE5[1];
    __IO uint32_t TCTL12M;               /*!< [0x0120] HIRC 12M Trim Control Register                                   */
    __IO uint32_t TIEN12M;               /*!< [0x0124] HIRC 12M Trim Interrupt Enable Register                          */
    __IO uint32_t TISTS12M;              /*!< [0x0128] HIRC 12M Trim Interrupt Status Register                          */
    __I  uint32_t RESERVE6[1];
    __IO uint32_t TCTLMIRC;              /*!< [0x0130] MIRC Trim Control Register                                       */
    __IO uint32_t TIENMIRC;              /*!< [0x0134] MIRC Trim Interrupt Enable Register                              */
    __IO uint32_t TISTSMIRC;             /*!< [0x0138] MIRC Trim Interrupt Status Register                              */
    __I  uint32_t RESERVE7[49];
    __IO uint32_t ACMPRST;               /*!< [0x0200] ACMP Reset Control Register                                      */
    __IO uint32_t AWFRST;                /*!< [0x0204] AWF Reset Control Register                                       */
    __IO uint32_t BPWMRST;               /*!< [0x0208] BPWM Reset Control Register                                      */
    __IO uint32_t CANFDRST;              /*!< [0x020c] CANFD Reset Control Register                                     */
    __IO uint32_t CCAPRST;               /*!< [0x0210] CCAP Reset Control Register                                      */
    __IO uint32_t CRCRST;                /*!< [0x0214] CRC Reset Control Register                                       */
    __IO uint32_t CRYPTORST;             /*!< [0x0218] CRYPTO Reset Control Register                                    */
    __IO uint32_t DACRST;                /*!< [0x021c] DAC Reset Control Register                                       */
    __IO uint32_t DMICRST;               /*!< [0x0220] DMIC Reset Control Register                                      */
    __IO uint32_t EADCRST;               /*!< [0x0224] EADC Reset Control Register                                      */
    __IO uint32_t EBIRST;                /*!< [0x0228] EBI Reset Control Register                                       */
    __IO uint32_t ECAPRST;               /*!< [0x022c] ECAP Reset Control Register                                      */
    __IO uint32_t EMACRST;               /*!< [0x0230] EMAC Reset Control Register                                      */
    __IO uint32_t EPWMRST;               /*!< [0x0234] EPWM Reset Control Register                                      */
    __IO uint32_t EQEIRST;               /*!< [0x0238] EQEI Reset Control Register                                      */
    __IO uint32_t FMCRST;                /*!< [0x023c] FMC Reset Control Register                                       */
    __IO uint32_t GDMARST;               /*!< [0x0240] GDMA Reset Control Register                                      */
    __IO uint32_t GPIORST;               /*!< [0x0244] GPIO Reset Control Register                                      */
    __IO uint32_t HSOTGRST;              /*!< [0x0248] HSOTG Reset Control Register                                     */
    __IO uint32_t HSUSBDRST;             /*!< [0x024c] HSUSBD Reset Control Register                                    */
    __IO uint32_t HSUSBHRST;             /*!< [0x0250] HSUSBH Reset Control Register                                    */
    __IO uint32_t I2CRST;                /*!< [0x0254] I2C Reset Control Register                                       */
    __IO uint32_t I2SRST;                /*!< [0x0258] I2S Reset Control Register                                       */
    __IO uint32_t I3CRST;                /*!< [0x025c] I3C Reset Control Register                                       */
    __IO uint32_t KDFRST;                /*!< [0x0260] KDF Reset Control Register                                       */
    __IO uint32_t KPIRST;                /*!< [0x0264] KPI Reset Control Register                                       */
    __IO uint32_t KSRST;                 /*!< [0x0268] KS Reset Control Register                                        */
    __IO uint32_t LPADCRST;              /*!< [0x026c] Low Power ADC Reset Control Register                             */
    __IO uint32_t LPPDMARST;             /*!< [0x0270] Low Power PDMA Reset Control Register                            */
    __IO uint32_t LPGPIORST;             /*!< [0x0274] Low Power GPIO Reset Control Register                            */
    __IO uint32_t LPI2CRST;              /*!< [0x0278] Low Power I2C Reset Control Register                             */
    __IO uint32_t LPSPIRST;              /*!< [0x027c] Low Power SPI Reset Control Register                             */
    __IO uint32_t LPTMRRST;              /*!< [0x0280] Low Power TMR Reset Control Register                             */
    __IO uint32_t LPUARTRST;             /*!< [0x0284] Low Power UART Reset Control Register                            */
    __IO uint32_t OTFCRST;               /*!< [0x0288] OTFC Reset Control Register                                      */
    __IO uint32_t OTGRST;                /*!< [0x028c] OTG Reset Control Register                                       */
    __IO uint32_t PDMARST;               /*!< [0x0290] PDMA Reset Control Register                                      */
    __IO uint32_t PSIORST;               /*!< [0x0294] PSIO Reset Control Register                                      */
    __IO uint32_t QSPIRST;               /*!< [0x0298] QSPI Reset Control Register                                      */
    __IO uint32_t RTCRST;                /*!< [0x029c] RTC Reset Control Register                                       */
    __IO uint32_t SCRST;                 /*!< [0x02a0] SC Reset Control Register                                        */
    __IO uint32_t SCURST;                /*!< [0x02a4] SCU Reset Control Register                                       */
    __IO uint32_t SDHRST;                /*!< [0x02a8] SDH Reset Control Register                                       */
    __IO uint32_t SPIRST;                /*!< [0x02ac] SPI Reset Control Register                                       */
    __IO uint32_t SPIMRST;               /*!< [0x02b0] SPIM Reset Control Register                                      */
    __I  uint32_t RESERVE8[3];
    __IO uint32_t TMRRST;                /*!< [0x02c0] Timer Reset Control Register                                     */
    __IO uint32_t TRNGRST;               /*!< [0x02c4] TRNG Reset Control Register                                      */
    __IO uint32_t TTMRRST;               /*!< [0x02c8] Tick Timer Reset Control Register                                */
    __IO uint32_t UARTRST;               /*!< [0x02cc] UART Reset Control Register                                      */
    __IO uint32_t USBDRST;               /*!< [0x02d0] USB Device Reset Control Register                                */
    __IO uint32_t USBHRST;               /*!< [0x02d4] USB Host Reset Control Register                                  */
    __IO uint32_t USCIRST;               /*!< [0x02d8] USCI Reset Control Register                                      */
    __IO uint32_t UTCPDRST;              /*!< [0x02dc] UTCPD Reset Control Register                                     */
    __IO uint32_t WWDTRST;               /*!< [0x02e0] WWDT Reset Control Register                                      */
    __I  uint32_t RESERVE9[7];
    __IO uint32_t GPA_MFP0;              /*!< [0x0300] GPIOA Multiple Function Control Register 0                       */
    __IO uint32_t GPA_MFP1;              /*!< [0x0304] GPIOA Multiple Function Control Register 1                       */
    __IO uint32_t GPA_MFP2;              /*!< [0x0308] GPIOA Multiple Function Control Register 2                       */
    __IO uint32_t GPA_MFP3;              /*!< [0x030c] GPIOA Multiple Function Control Register 3                       */
    __IO uint32_t GPB_MFP0;              /*!< [0x0310] GPIOB Multiple Function Control Register 0                       */
    __IO uint32_t GPB_MFP1;              /*!< [0x0314] GPIOB Multiple Function Control Register 1                       */
    __IO uint32_t GPB_MFP2;              /*!< [0x0318] GPIOB Multiple Function Control Register 2                       */
    __IO uint32_t GPB_MFP3;              /*!< [0x031c] GPIOB Multiple Function Control Register 3                       */
    __IO uint32_t GPC_MFP0;              /*!< [0x0320] GPIOC Multiple Function Control Register 0                       */
    __IO uint32_t GPC_MFP1;              /*!< [0x0324] GPIOC Multiple Function Control Register 1                       */
    __IO uint32_t GPC_MFP2;              /*!< [0x0328] GPIOC Multiple Function Control Register 2                       */
    __IO uint32_t GPC_MFP3;              /*!< [0x032c] GPIOC Multiple Function Control Register 3                       */
    __IO uint32_t GPD_MFP0;              /*!< [0x0330] GPIOD Multiple Function Control Register 0                       */
    __IO uint32_t GPD_MFP1;              /*!< [0x0334] GPIOD Multiple Function Control Register 1                       */
    __IO uint32_t GPD_MFP2;              /*!< [0x0338] GPIOD Multiple Function Control Register 2                       */
    __IO uint32_t GPD_MFP3;              /*!< [0x033c] GPIOD Multiple Function Control Register 3                       */
    __IO uint32_t GPE_MFP0;              /*!< [0x0340] GPIOE Multiple Function Control Register 0                       */
    __IO uint32_t GPE_MFP1;              /*!< [0x0344] GPIOE Multiple Function Control Register 1                       */
    __IO uint32_t GPE_MFP2;              /*!< [0x0348] GPIOE Multiple Function Control Register 2                       */
    __IO uint32_t GPE_MFP3;              /*!< [0x034c] GPIOE Multiple Function Control Register 3                       */
    __IO uint32_t GPF_MFP0;              /*!< [0x0350] GPIOF Multiple Function Control Register 0                       */
    __IO uint32_t GPF_MFP1;              /*!< [0x0354] GPIOF Multiple Function Control Register 1                       */
    __IO uint32_t GPF_MFP2;              /*!< [0x0358] GPIOF Multiple Function Control Register 2                       */
    __I  uint32_t RESERVE10[1];
    __IO uint32_t GPG_MFP0;              /*!< [0x0360] GPIOG Multiple Function Control Register 0                       */
    __IO uint32_t GPG_MFP1;              /*!< [0x0364] GPIOG Multiple Function Control Register 1                       */
    __IO uint32_t GPG_MFP2;              /*!< [0x0368] GPIOG Multiple Function Control Register 2                       */
    __IO uint32_t GPG_MFP3;              /*!< [0x036c] GPIOG Multiple Function Control Register 3                       */
    __IO uint32_t GPH_MFP0;              /*!< [0x0370] GPIOH Multiple Function Control Register 0                       */
    __IO uint32_t GPH_MFP1;              /*!< [0x0374] GPIOH Multiple Function Control Register 1                       */
    __IO uint32_t GPH_MFP2;              /*!< [0x0378] GPIOH Multiple Function Control Register 2                       */
    __IO uint32_t GPH_MFP3;              /*!< [0x037c] GPIOH Multiple Function Control Register 3                       */
    __I  uint32_t RESERVE11[1];
    __IO uint32_t GPI_MFP1;              /*!< [0x0384] GPIOI Multiple Function Control Register 1                       */
    __IO uint32_t GPI_MFP2;              /*!< [0x0388] GPIOI Multiple Function Control Register 2                       */
    __IO uint32_t GPI_MFP3;              /*!< [0x038c] GPIOI Multiple Function Control Register 3                       */
    __IO uint32_t GPJ_MFP0;              /*!< [0x0390] GPIOJ Multiple Function Control Register 0                       */
    __IO uint32_t GPJ_MFP1;              /*!< [0x0394] GPIOJ Multiple Function Control Register 1                       */
    __IO uint32_t GPJ_MFP2;              /*!< [0x0398] GPIOJ Multiple Function Control Register 2                       */
    __IO uint32_t GPJ_MFP3;              /*!< [0x039c] GPIOJ Multiple Function Control Register 3                       */
    __I  uint32_t RESERVE12[88];
    __IO uint32_t NMIEN;                 /*!< [0x0500] NMI Source Interrupt Enable Register                             */
    __I  uint32_t NMISTS;                /*!< [0x0504] NMI Source Interrupt Status Register                             */
    __I  uint32_t RESERVE13[610];
    __IO uint32_t SRAMBCTL;              /*!< [0x0e90] System SRAM BIST Control Register                                */
    __I  uint32_t SRAMBFF;               /*!< [0x0e94] System SRAM BIST Finish Flag Register                            */
    __I  uint32_t SRAMBRF;               /*!< [0x0e98] System SRAM BIST Result Flag Register                            */

} SYS_T;

/**
    @addtogroup SYS_CONST SYS Bit Field Definition
    Constant Definitions for SYS Controller
@{ */

#define SYS_PDID_PDID_Pos                (0)                                               /*!< SYS_T::PDID: PDID Position             */
#define SYS_PDID_PDID_Msk                (0xfffffffful << SYS_PDID_PDID_Pos)               /*!< SYS_T::PDID: PDID Mask                 */

#define SYS_RSTCTL_CHIPRST_Pos           (0)                                               /*!< SYS_T::RSTCTL: CHIPRST Position        */
#define SYS_RSTCTL_CHIPRST_Msk           (0x1ul << SYS_RSTCTL_CHIPRST_Pos)                 /*!< SYS_T::RSTCTL: CHIPRST Mask            */

#define SYS_RSTCTL_NPURST_Pos            (6)                                               /*!< SYS_T::RSTCTL: NPURST Position         */
#define SYS_RSTCTL_NPURST_Msk            (0x1ul << SYS_RSTCTL_NPURST_Pos)                  /*!< SYS_T::RSTCTL: NPURST Mask             */

#define SYS_RSTCTL_CPURSTEN_Pos          (7)                                               /*!< SYS_T::RSTCTL: CPURSTEN Position       */
#define SYS_RSTCTL_CPURSTEN_Msk          (0x1ul << SYS_RSTCTL_CPURSTEN_Pos)                /*!< SYS_T::RSTCTL: CPURSTEN Mask           */

#define SYS_RSTSTS_PORF_Pos              (0)                                               /*!< SYS_T::RSTSTS: PORF Position           */
#define SYS_RSTSTS_PORF_Msk              (0x1ul << SYS_RSTSTS_PORF_Pos)                    /*!< SYS_T::RSTSTS: PORF Mask               */

#define SYS_RSTSTS_PINRF_Pos             (1)                                               /*!< SYS_T::RSTSTS: PINRF Position          */
#define SYS_RSTSTS_PINRF_Msk             (0x1ul << SYS_RSTSTS_PINRF_Pos)                   /*!< SYS_T::RSTSTS: PINRF Mask              */

#define SYS_RSTSTS_WDTRF_Pos             (2)                                               /*!< SYS_T::RSTSTS: WDTRF Position          */
#define SYS_RSTSTS_WDTRF_Msk             (0x1ul << SYS_RSTSTS_WDTRF_Pos)                   /*!< SYS_T::RSTSTS: WDTRF Mask              */

#define SYS_RSTSTS_LVRRF_Pos             (3)                                               /*!< SYS_T::RSTSTS: LVRRF Position          */
#define SYS_RSTSTS_LVRRF_Msk             (0x1ul << SYS_RSTSTS_LVRRF_Pos)                   /*!< SYS_T::RSTSTS: LVRRF Mask              */

#define SYS_RSTSTS_BODRF_Pos             (4)                                               /*!< SYS_T::RSTSTS: BODRF Position          */
#define SYS_RSTSTS_BODRF_Msk             (0x1ul << SYS_RSTSTS_BODRF_Pos)                   /*!< SYS_T::RSTSTS: BODRF Mask              */

#define SYS_RSTSTS_SYSRF_Pos             (5)                                               /*!< SYS_T::RSTSTS: SYSRF Position          */
#define SYS_RSTSTS_SYSRF_Msk             (0x1ul << SYS_RSTSTS_SYSRF_Pos)                   /*!< SYS_T::RSTSTS: SYSRF Mask              */

#define SYS_RSTSTS_CPURF_Pos             (7)                                               /*!< SYS_T::RSTSTS: CPURF Position          */
#define SYS_RSTSTS_CPURF_Msk             (0x1ul << SYS_RSTSTS_CPURF_Pos)                   /*!< SYS_T::RSTSTS: CPURF Mask              */

#define SYS_RSTSTS_CPULKRF_Pos           (8)                                               /*!< SYS_T::RSTSTS: CPULKRF Position        */
#define SYS_RSTSTS_CPULKRF_Msk           (0x1ul << SYS_RSTSTS_CPULKRF_Pos)                 /*!< SYS_T::RSTSTS: CPULKRF Mask            */

#define SYS_VTORSET_VTORSET_Pos          (7)                                               /*!< SYS_T::VTORSET: VTORSET Position       */
#define SYS_VTORSET_VTORSET_Msk          (0x1fffffful << SYS_VTORSET_VTORSET_Pos)          /*!< SYS_T::VTORSET: VTORSET Mask           */

#define SYS_SRAMICTL_PERRIEN_Pos         (0)                                               /*!< SYS_T::SRAMICTL: PERRIEN Position      */
#define SYS_SRAMICTL_PERRIEN_Msk         (0x1ul << SYS_SRAMICTL_PERRIEN_Pos)               /*!< SYS_T::SRAMICTL: PERRIEN Mask          */

#define SYS_SRAMSTS_PERRIF_Pos           (0)                                               /*!< SYS_T::SRAMSTS: PERRIF Position        */
#define SYS_SRAMSTS_PERRIF_Msk           (0x1ul << SYS_SRAMSTS_PERRIF_Pos)                 /*!< SYS_T::SRAMSTS: PERRIF Mask            */

#define SYS_SRAMSTS_B0PERRIF_Pos         (8)                                               /*!< SYS_T::SRAMSTS: B0PERRIF Position      */
#define SYS_SRAMSTS_B0PERRIF_Msk         (0x1ul << SYS_SRAMSTS_B0PERRIF_Pos)               /*!< SYS_T::SRAMSTS: B0PERRIF Mask          */

#define SYS_SRAMSTS_B1PERRIF_Pos         (9)                                               /*!< SYS_T::SRAMSTS: B1PERRIF Position      */
#define SYS_SRAMSTS_B1PERRIF_Msk         (0x1ul << SYS_SRAMSTS_B1PERRIF_Pos)               /*!< SYS_T::SRAMSTS: B1PERRIF Mask          */

#define SYS_SRAMSTS_B2PERRIF_Pos         (10)                                              /*!< SYS_T::SRAMSTS: B2PERRIF Position      */
#define SYS_SRAMSTS_B2PERRIF_Msk         (0x1ul << SYS_SRAMSTS_B2PERRIF_Pos)               /*!< SYS_T::SRAMSTS: B2PERRIF Mask          */

#define SYS_SRAMSTS_B3PERRIF_Pos         (11)                                              /*!< SYS_T::SRAMSTS: B3PERRIF Position      */
#define SYS_SRAMSTS_B3PERRIF_Msk         (0x1ul << SYS_SRAMSTS_B3PERRIF_Pos)               /*!< SYS_T::SRAMSTS: B3PERRIF Mask          */

#define SYS_SRAMSTS_B4PERRIF_Pos         (12)                                              /*!< SYS_T::SRAMSTS: B4PERRIF Position      */
#define SYS_SRAMSTS_B4PERRIF_Msk         (0x1ul << SYS_SRAMSTS_B4PERRIF_Pos)               /*!< SYS_T::SRAMSTS: B4PERRIF Mask          */

#define SYS_SRAMSTS_B5PERRIF_Pos         (13)                                              /*!< SYS_T::SRAMSTS: B5PERRIF Position      */
#define SYS_SRAMSTS_B5PERRIF_Msk         (0x1ul << SYS_SRAMSTS_B5PERRIF_Pos)               /*!< SYS_T::SRAMSTS: B5PERRIF Mask          */

#define SYS_SRAMSTS_B6PERRIF_Pos         (14)                                              /*!< SYS_T::SRAMSTS: B6PERRIF Position      */
#define SYS_SRAMSTS_B6PERRIF_Msk         (0x1ul << SYS_SRAMSTS_B6PERRIF_Pos)               /*!< SYS_T::SRAMSTS: B6PERRIF Mask          */

#define SYS_SRAMSTS_B7PERRIF_Pos         (15)                                              /*!< SYS_T::SRAMSTS: B7PERRIF Position      */
#define SYS_SRAMSTS_B7PERRIF_Msk         (0x1ul << SYS_SRAMSTS_B7PERRIF_Pos)               /*!< SYS_T::SRAMSTS: B7PERRIF Mask          */

#define SYS_SRAMEADR_ERRADDR_Pos         (0)                                               /*!< SYS_T::SRAMEADR: ERRADDR Position      */
#define SYS_SRAMEADR_ERRADDR_Msk         (0xfffffffful << SYS_SRAMEADR_ERRADDR_Pos)        /*!< SYS_T::SRAMEADR: ERRADDR Mask          */

#define SYS_BODCTL_BODEN_Pos             (0)                                               /*!< SYS_T::BODCTL: BODEN Position          */
#define SYS_BODCTL_BODEN_Msk             (0x1ul << SYS_BODCTL_BODEN_Pos)                   /*!< SYS_T::BODCTL: BODEN Mask              */

#define SYS_BODCTL_BODLPM_Pos            (1)                                               /*!< SYS_T::BODCTL: BODLPM Position         */
#define SYS_BODCTL_BODLPM_Msk            (0x1ul << SYS_BODCTL_BODLPM_Pos)                  /*!< SYS_T::BODCTL: BODLPM Mask             */

#define SYS_BODCTL_BODRSTEN_Pos          (2)                                               /*!< SYS_T::BODCTL: BODRSTEN Position       */
#define SYS_BODCTL_BODRSTEN_Msk          (0x1ul << SYS_BODCTL_BODRSTEN_Pos)                /*!< SYS_T::BODCTL: BODRSTEN Mask           */

#define SYS_BODCTL_BODDGSEL_Pos          (4)                                               /*!< SYS_T::BODCTL: BODDGSEL Position       */
#define SYS_BODCTL_BODDGSEL_Msk          (0x7ul << SYS_BODCTL_BODDGSEL_Pos)                /*!< SYS_T::BODCTL: BODDGSEL Mask           */

#define SYS_BODCTL_BODVL_Pos             (8)                                               /*!< SYS_T::BODCTL: BODVL Position          */
#define SYS_BODCTL_BODVL_Msk             (0x7ul << SYS_BODCTL_BODVL_Pos)                   /*!< SYS_T::BODCTL: BODVL Mask              */

#define SYS_BODCTL_BODWKEN_Pos           (12)                                              /*!< SYS_T::BODCTL: BODWKEN Position        */
#define SYS_BODCTL_BODWKEN_Msk           (0x3ul << SYS_BODCTL_BODWKEN_Pos)                 /*!< SYS_T::BODCTL: BODWKEN Mask            */

#define SYS_BODCTL_LVREN_Pos             (16)                                              /*!< SYS_T::BODCTL: LVREN Position          */
#define SYS_BODCTL_LVREN_Msk             (0x1ul << SYS_BODCTL_LVREN_Pos)                   /*!< SYS_T::BODCTL: LVREN Mask              */

#define SYS_BODCTL_LVRDGSEL_Pos          (20)                                              /*!< SYS_T::BODCTL: LVRDGSEL Position       */
#define SYS_BODCTL_LVRDGSEL_Msk          (0x7ul << SYS_BODCTL_LVRDGSEL_Pos)                /*!< SYS_T::BODCTL: LVRDGSEL Mask           */

#define SYS_BODCTL_WRBUSY_Pos            (31)                                              /*!< SYS_T::BODCTL: WRBUSY Position         */
#define SYS_BODCTL_WRBUSY_Msk            (0x1ul << SYS_BODCTL_WRBUSY_Pos)                  /*!< SYS_T::BODCTL: WRBUSY Mask             */

#define SYS_BODSTS_BODOUT_Pos            (0)                                               /*!< SYS_T::BODSTS: BODOUT Position         */
#define SYS_BODSTS_BODOUT_Msk            (0x1ul << SYS_BODSTS_BODOUT_Pos)                  /*!< SYS_T::BODSTS: BODOUT Mask             */

#define SYS_BODSTS_BODIF_Pos             (1)                                               /*!< SYS_T::BODSTS: BODIF Position          */
#define SYS_BODSTS_BODIF_Msk             (0x1ul << SYS_BODSTS_BODIF_Pos)                   /*!< SYS_T::BODSTS: BODIF Mask              */

#define SYS_PORCTL_PORMASK_Pos           (0)                                               /*!< SYS_T::PORCTL: PORMASK Position        */
#define SYS_PORCTL_PORMASK_Msk           (0xfffful << SYS_PORCTL_PORMASK_Pos)              /*!< SYS_T::PORCTL: PORMASK Mask            */

#define SYS_PORCTL_POROFF_Pos            (16)                                              /*!< SYS_T::PORCTL: POROFF Position         */
#define SYS_PORCTL_POROFF_Msk            (0xfffful << SYS_PORCTL_POROFF_Pos)               /*!< SYS_T::PORCTL: POROFF Mask             */

#define SYS_VREFCTL_VREFCTL_Pos          (0)                                               /*!< SYS_T::VREFCTL: VREFCTL Position       */
#define SYS_VREFCTL_VREFCTL_Msk          (0x1ful << SYS_VREFCTL_VREFCTL_Pos)               /*!< SYS_T::VREFCTL: VREFCTL Mask           */

#define SYS_VREFCTL_PRELOADSEL_Pos       (6)                                               /*!< SYS_T::VREFCTL: PRELOADSEL Position    */
#define SYS_VREFCTL_PRELOADSEL_Msk       (0x3ul << SYS_VREFCTL_PRELOADSEL_Pos)             /*!< SYS_T::VREFCTL: PRELOADSEL Mask        */

#define SYS_VREFCTL_SCPTEST_Pos          (8)                                               /*!< SYS_T::VREFCTL: SCPTEST Position       */
#define SYS_VREFCTL_SCPTEST_Msk          (0x1ul << SYS_VREFCTL_SCPTEST_Pos)                /*!< SYS_T::VREFCTL: SCPTEST Mask           */

#define SYS_VREFCTL_SCPFINISH_Pos        (9)                                               /*!< SYS_T::VREFCTL: SCPFINISH Position     */
#define SYS_VREFCTL_SCPFINISH_Msk        (0x1ul << SYS_VREFCTL_SCPFINISH_Pos)              /*!< SYS_T::VREFCTL: SCPFINISH Mask         */

#define SYS_VREFCTL_WRBUSY_Pos           (31)                                              /*!< SYS_T::VREFCTL: WRBUSY Position        */
#define SYS_VREFCTL_WRBUSY_Msk           (0x1ul << SYS_VREFCTL_WRBUSY_Pos)                 /*!< SYS_T::VREFCTL: WRBUSY Mask            */

#define SYS_IVSCTL_VTEMPEN_Pos           (0)                                               /*!< SYS_T::IVSCTL: VTEMPEN Position        */
#define SYS_IVSCTL_VTEMPEN_Msk           (0x1ul << SYS_IVSCTL_VTEMPEN_Pos)                 /*!< SYS_T::IVSCTL: VTEMPEN Mask            */

#define SYS_IVSCTL_VBATUGEN_Pos          (1)                                               /*!< SYS_T::IVSCTL: VBATUGEN Position       */
#define SYS_IVSCTL_VBATUGEN_Msk          (0x1ul << SYS_IVSCTL_VBATUGEN_Pos)                /*!< SYS_T::IVSCTL: VBATUGEN Mask           */

#define SYS_IVSCTL_AVDDDIV4EN_Pos        (2)                                               /*!< SYS_T::IVSCTL: AVDDDIV4EN Position     */
#define SYS_IVSCTL_AVDDDIV4EN_Msk        (0x1ul << SYS_IVSCTL_AVDDDIV4EN_Pos)              /*!< SYS_T::IVSCTL: AVDDDIV4EN Mask         */

#define SYS_IVSCTL_VTEMPSEL_Pos          (4)                                               /*!< SYS_T::IVSCTL: VTEMPSEL Position       */
#define SYS_IVSCTL_VTEMPSEL_Msk          (0x1ul << SYS_IVSCTL_VTEMPSEL_Pos)                /*!< SYS_T::IVSCTL: VTEMPSEL Mask           */

#define SYS_USBPHY_USBROLE_Pos           (0)                                               /*!< SYS_T::USBPHY: USBROLE Position        */
#define SYS_USBPHY_USBROLE_Msk           (0x3ul << SYS_USBPHY_USBROLE_Pos)                 /*!< SYS_T::USBPHY: USBROLE Mask            */

#define SYS_USBPHY_OTGPHYEN_Pos          (8)                                               /*!< SYS_T::USBPHY: OTGPHYEN Position       */
#define SYS_USBPHY_OTGPHYEN_Msk          (0x1ul << SYS_USBPHY_OTGPHYEN_Pos)                /*!< SYS_T::USBPHY: OTGPHYEN Mask           */

#define SYS_USBPHY_HSUSBROLE_Pos         (16)                                              /*!< SYS_T::USBPHY: HSUSBROLE Position      */
#define SYS_USBPHY_HSUSBROLE_Msk         (0x3ul << SYS_USBPHY_HSUSBROLE_Pos)               /*!< SYS_T::USBPHY: HSUSBROLE Mask          */

#define SYS_USBPHY_HSOTGPHYEN_Pos        (24)                                              /*!< SYS_T::USBPHY: HSOTGPHYEN Position     */
#define SYS_USBPHY_HSOTGPHYEN_Msk        (0x1ul << SYS_USBPHY_HSOTGPHYEN_Pos)              /*!< SYS_T::USBPHY: HSOTGPHYEN Mask         */

#define SYS_USBPHY_HSUSBACT_Pos          (25)                                              /*!< SYS_T::USBPHY: HSUSBACT Position       */
#define SYS_USBPHY_HSUSBACT_Msk          (0x1ul << SYS_USBPHY_HSUSBACT_Pos)                /*!< SYS_T::USBPHY: HSUSBACT Mask           */

#define SYS_UTCPDCTL_IOMODE_Pos          (0)                                               /*!< SYS_T::UTCPDCTL: IOMODE Position       */
#define SYS_UTCPDCTL_IOMODE_Msk          (0x1ul << SYS_UTCPDCTL_IOMODE_Pos)                /*!< SYS_T::UTCPDCTL: IOMODE Mask           */

#define SYS_UTCPDCTL_POREN0_Pos          (1)                                               /*!< SYS_T::UTCPDCTL: POREN0 Position       */
#define SYS_UTCPDCTL_POREN0_Msk          (0x1ul << SYS_UTCPDCTL_POREN0_Pos)                /*!< SYS_T::UTCPDCTL: POREN0 Mask           */

#define SYS_UTCPDCTL_PD0VBDSS_Pos        (8)                                               /*!< SYS_T::UTCPDCTL: PD0VBDSS Position     */
#define SYS_UTCPDCTL_PD0VBDSS_Msk        (0x7ul << SYS_UTCPDCTL_PD0VBDSS_Pos)              /*!< SYS_T::UTCPDCTL: PD0VBDSS Mask         */

#define SYS_UTCPDCTL_UDVBDETS_Pos        (12)                                              /*!< SYS_T::UTCPDCTL: UDVBDETS Position     */
#define SYS_UTCPDCTL_UDVBDETS_Msk        (0x1ul << SYS_UTCPDCTL_UDVBDETS_Pos)              /*!< SYS_T::UTCPDCTL: UDVBDETS Mask         */

#define SYS_DBUSCTL_H2XBSTEN_Pos         (0)                                               /*!< SYS_T::DBUSCTL: H2XBSTEN Position      */
#define SYS_DBUSCTL_H2XBSTEN_Msk         (0x1ul << SYS_DBUSCTL_H2XBSTEN_Pos)               /*!< SYS_T::DBUSCTL: H2XBSTEN Mask          */

#define SYS_GPA_MFOS_MFOS0_Pos           (0)                                               /*!< SYS_T::GPA_MFOS: MFOS0 Position        */
#define SYS_GPA_MFOS_MFOS0_Msk           (0x1ul << SYS_GPA_MFOS_MFOS0_Pos)                 /*!< SYS_T::GPA_MFOS: MFOS0 Mask            */

#define SYS_GPA_MFOS_MFOS1_Pos           (1)                                               /*!< SYS_T::GPA_MFOS: MFOS1 Position        */
#define SYS_GPA_MFOS_MFOS1_Msk           (0x1ul << SYS_GPA_MFOS_MFOS1_Pos)                 /*!< SYS_T::GPA_MFOS: MFOS1 Mask            */

#define SYS_GPA_MFOS_MFOS2_Pos           (2)                                               /*!< SYS_T::GPA_MFOS: MFOS2 Position        */
#define SYS_GPA_MFOS_MFOS2_Msk           (0x1ul << SYS_GPA_MFOS_MFOS2_Pos)                 /*!< SYS_T::GPA_MFOS: MFOS2 Mask            */

#define SYS_GPA_MFOS_MFOS3_Pos           (3)                                               /*!< SYS_T::GPA_MFOS: MFOS3 Position        */
#define SYS_GPA_MFOS_MFOS3_Msk           (0x1ul << SYS_GPA_MFOS_MFOS3_Pos)                 /*!< SYS_T::GPA_MFOS: MFOS3 Mask            */

#define SYS_GPA_MFOS_MFOS4_Pos           (4)                                               /*!< SYS_T::GPA_MFOS: MFOS4 Position        */
#define SYS_GPA_MFOS_MFOS4_Msk           (0x1ul << SYS_GPA_MFOS_MFOS4_Pos)                 /*!< SYS_T::GPA_MFOS: MFOS4 Mask            */

#define SYS_GPA_MFOS_MFOS5_Pos           (5)                                               /*!< SYS_T::GPA_MFOS: MFOS5 Position        */
#define SYS_GPA_MFOS_MFOS5_Msk           (0x1ul << SYS_GPA_MFOS_MFOS5_Pos)                 /*!< SYS_T::GPA_MFOS: MFOS5 Mask            */

#define SYS_GPA_MFOS_MFOS6_Pos           (6)                                               /*!< SYS_T::GPA_MFOS: MFOS6 Position        */
#define SYS_GPA_MFOS_MFOS6_Msk           (0x1ul << SYS_GPA_MFOS_MFOS6_Pos)                 /*!< SYS_T::GPA_MFOS: MFOS6 Mask            */

#define SYS_GPA_MFOS_MFOS7_Pos           (7)                                               /*!< SYS_T::GPA_MFOS: MFOS7 Position        */
#define SYS_GPA_MFOS_MFOS7_Msk           (0x1ul << SYS_GPA_MFOS_MFOS7_Pos)                 /*!< SYS_T::GPA_MFOS: MFOS7 Mask            */

#define SYS_GPA_MFOS_MFOS8_Pos           (8)                                               /*!< SYS_T::GPA_MFOS: MFOS8 Position        */
#define SYS_GPA_MFOS_MFOS8_Msk           (0x1ul << SYS_GPA_MFOS_MFOS8_Pos)                 /*!< SYS_T::GPA_MFOS: MFOS8 Mask            */

#define SYS_GPA_MFOS_MFOS9_Pos           (9)                                               /*!< SYS_T::GPA_MFOS: MFOS9 Position        */
#define SYS_GPA_MFOS_MFOS9_Msk           (0x1ul << SYS_GPA_MFOS_MFOS9_Pos)                 /*!< SYS_T::GPA_MFOS: MFOS9 Mask            */

#define SYS_GPA_MFOS_MFOS10_Pos          (10)                                              /*!< SYS_T::GPA_MFOS: MFOS10 Position       */
#define SYS_GPA_MFOS_MFOS10_Msk          (0x1ul << SYS_GPA_MFOS_MFOS10_Pos)                /*!< SYS_T::GPA_MFOS: MFOS10 Mask           */

#define SYS_GPA_MFOS_MFOS11_Pos          (11)                                              /*!< SYS_T::GPA_MFOS: MFOS11 Position       */
#define SYS_GPA_MFOS_MFOS11_Msk          (0x1ul << SYS_GPA_MFOS_MFOS11_Pos)                /*!< SYS_T::GPA_MFOS: MFOS11 Mask           */

#define SYS_GPA_MFOS_MFOS12_Pos          (12)                                              /*!< SYS_T::GPA_MFOS: MFOS12 Position       */
#define SYS_GPA_MFOS_MFOS12_Msk          (0x1ul << SYS_GPA_MFOS_MFOS12_Pos)                /*!< SYS_T::GPA_MFOS: MFOS12 Mask           */

#define SYS_GPA_MFOS_MFOS13_Pos          (13)                                              /*!< SYS_T::GPA_MFOS: MFOS13 Position       */
#define SYS_GPA_MFOS_MFOS13_Msk          (0x1ul << SYS_GPA_MFOS_MFOS13_Pos)                /*!< SYS_T::GPA_MFOS: MFOS13 Mask           */

#define SYS_GPA_MFOS_MFOS14_Pos          (14)                                              /*!< SYS_T::GPA_MFOS: MFOS14 Position       */
#define SYS_GPA_MFOS_MFOS14_Msk          (0x1ul << SYS_GPA_MFOS_MFOS14_Pos)                /*!< SYS_T::GPA_MFOS: MFOS14 Mask           */

#define SYS_GPA_MFOS_MFOS15_Pos          (15)                                              /*!< SYS_T::GPA_MFOS: MFOS15 Position       */
#define SYS_GPA_MFOS_MFOS15_Msk          (0x1ul << SYS_GPA_MFOS_MFOS15_Pos)                /*!< SYS_T::GPA_MFOS: MFOS15 Mask           */

#define SYS_GPB_MFOS_MFOS0_Pos           (0)                                               /*!< SYS_T::GPB_MFOS: MFOS0 Position        */
#define SYS_GPB_MFOS_MFOS0_Msk           (0x1ul << SYS_GPB_MFOS_MFOS0_Pos)                 /*!< SYS_T::GPB_MFOS: MFOS0 Mask            */

#define SYS_GPB_MFOS_MFOS1_Pos           (1)                                               /*!< SYS_T::GPB_MFOS: MFOS1 Position        */
#define SYS_GPB_MFOS_MFOS1_Msk           (0x1ul << SYS_GPB_MFOS_MFOS1_Pos)                 /*!< SYS_T::GPB_MFOS: MFOS1 Mask            */

#define SYS_GPB_MFOS_MFOS2_Pos           (2)                                               /*!< SYS_T::GPB_MFOS: MFOS2 Position        */
#define SYS_GPB_MFOS_MFOS2_Msk           (0x1ul << SYS_GPB_MFOS_MFOS2_Pos)                 /*!< SYS_T::GPB_MFOS: MFOS2 Mask            */

#define SYS_GPB_MFOS_MFOS3_Pos           (3)                                               /*!< SYS_T::GPB_MFOS: MFOS3 Position        */
#define SYS_GPB_MFOS_MFOS3_Msk           (0x1ul << SYS_GPB_MFOS_MFOS3_Pos)                 /*!< SYS_T::GPB_MFOS: MFOS3 Mask            */

#define SYS_GPB_MFOS_MFOS4_Pos           (4)                                               /*!< SYS_T::GPB_MFOS: MFOS4 Position        */
#define SYS_GPB_MFOS_MFOS4_Msk           (0x1ul << SYS_GPB_MFOS_MFOS4_Pos)                 /*!< SYS_T::GPB_MFOS: MFOS4 Mask            */

#define SYS_GPB_MFOS_MFOS5_Pos           (5)                                               /*!< SYS_T::GPB_MFOS: MFOS5 Position        */
#define SYS_GPB_MFOS_MFOS5_Msk           (0x1ul << SYS_GPB_MFOS_MFOS5_Pos)                 /*!< SYS_T::GPB_MFOS: MFOS5 Mask            */

#define SYS_GPB_MFOS_MFOS6_Pos           (6)                                               /*!< SYS_T::GPB_MFOS: MFOS6 Position        */
#define SYS_GPB_MFOS_MFOS6_Msk           (0x1ul << SYS_GPB_MFOS_MFOS6_Pos)                 /*!< SYS_T::GPB_MFOS: MFOS6 Mask            */

#define SYS_GPB_MFOS_MFOS7_Pos           (7)                                               /*!< SYS_T::GPB_MFOS: MFOS7 Position        */
#define SYS_GPB_MFOS_MFOS7_Msk           (0x1ul << SYS_GPB_MFOS_MFOS7_Pos)                 /*!< SYS_T::GPB_MFOS: MFOS7 Mask            */

#define SYS_GPB_MFOS_MFOS8_Pos           (8)                                               /*!< SYS_T::GPB_MFOS: MFOS8 Position        */
#define SYS_GPB_MFOS_MFOS8_Msk           (0x1ul << SYS_GPB_MFOS_MFOS8_Pos)                 /*!< SYS_T::GPB_MFOS: MFOS8 Mask            */

#define SYS_GPB_MFOS_MFOS9_Pos           (9)                                               /*!< SYS_T::GPB_MFOS: MFOS9 Position        */
#define SYS_GPB_MFOS_MFOS9_Msk           (0x1ul << SYS_GPB_MFOS_MFOS9_Pos)                 /*!< SYS_T::GPB_MFOS: MFOS9 Mask            */

#define SYS_GPB_MFOS_MFOS10_Pos          (10)                                              /*!< SYS_T::GPB_MFOS: MFOS10 Position       */
#define SYS_GPB_MFOS_MFOS10_Msk          (0x1ul << SYS_GPB_MFOS_MFOS10_Pos)                /*!< SYS_T::GPB_MFOS: MFOS10 Mask           */

#define SYS_GPB_MFOS_MFOS11_Pos          (11)                                              /*!< SYS_T::GPB_MFOS: MFOS11 Position       */
#define SYS_GPB_MFOS_MFOS11_Msk          (0x1ul << SYS_GPB_MFOS_MFOS11_Pos)                /*!< SYS_T::GPB_MFOS: MFOS11 Mask           */

#define SYS_GPB_MFOS_MFOS12_Pos          (12)                                              /*!< SYS_T::GPB_MFOS: MFOS12 Position       */
#define SYS_GPB_MFOS_MFOS12_Msk          (0x1ul << SYS_GPB_MFOS_MFOS12_Pos)                /*!< SYS_T::GPB_MFOS: MFOS12 Mask           */

#define SYS_GPB_MFOS_MFOS13_Pos          (13)                                              /*!< SYS_T::GPB_MFOS: MFOS13 Position       */
#define SYS_GPB_MFOS_MFOS13_Msk          (0x1ul << SYS_GPB_MFOS_MFOS13_Pos)                /*!< SYS_T::GPB_MFOS: MFOS13 Mask           */

#define SYS_GPB_MFOS_MFOS14_Pos          (14)                                              /*!< SYS_T::GPB_MFOS: MFOS14 Position       */
#define SYS_GPB_MFOS_MFOS14_Msk          (0x1ul << SYS_GPB_MFOS_MFOS14_Pos)                /*!< SYS_T::GPB_MFOS: MFOS14 Mask           */

#define SYS_GPB_MFOS_MFOS15_Pos          (15)                                              /*!< SYS_T::GPB_MFOS: MFOS15 Position       */
#define SYS_GPB_MFOS_MFOS15_Msk          (0x1ul << SYS_GPB_MFOS_MFOS15_Pos)                /*!< SYS_T::GPB_MFOS: MFOS15 Mask           */

#define SYS_GPC_MFOS_MFOS0_Pos           (0)                                               /*!< SYS_T::GPC_MFOS: MFOS0 Position        */
#define SYS_GPC_MFOS_MFOS0_Msk           (0x1ul << SYS_GPC_MFOS_MFOS0_Pos)                 /*!< SYS_T::GPC_MFOS: MFOS0 Mask            */

#define SYS_GPC_MFOS_MFOS1_Pos           (1)                                               /*!< SYS_T::GPC_MFOS: MFOS1 Position        */
#define SYS_GPC_MFOS_MFOS1_Msk           (0x1ul << SYS_GPC_MFOS_MFOS1_Pos)                 /*!< SYS_T::GPC_MFOS: MFOS1 Mask            */

#define SYS_GPC_MFOS_MFOS2_Pos           (2)                                               /*!< SYS_T::GPC_MFOS: MFOS2 Position        */
#define SYS_GPC_MFOS_MFOS2_Msk           (0x1ul << SYS_GPC_MFOS_MFOS2_Pos)                 /*!< SYS_T::GPC_MFOS: MFOS2 Mask            */

#define SYS_GPC_MFOS_MFOS3_Pos           (3)                                               /*!< SYS_T::GPC_MFOS: MFOS3 Position        */
#define SYS_GPC_MFOS_MFOS3_Msk           (0x1ul << SYS_GPC_MFOS_MFOS3_Pos)                 /*!< SYS_T::GPC_MFOS: MFOS3 Mask            */

#define SYS_GPC_MFOS_MFOS4_Pos           (4)                                               /*!< SYS_T::GPC_MFOS: MFOS4 Position        */
#define SYS_GPC_MFOS_MFOS4_Msk           (0x1ul << SYS_GPC_MFOS_MFOS4_Pos)                 /*!< SYS_T::GPC_MFOS: MFOS4 Mask            */

#define SYS_GPC_MFOS_MFOS5_Pos           (5)                                               /*!< SYS_T::GPC_MFOS: MFOS5 Position        */
#define SYS_GPC_MFOS_MFOS5_Msk           (0x1ul << SYS_GPC_MFOS_MFOS5_Pos)                 /*!< SYS_T::GPC_MFOS: MFOS5 Mask            */

#define SYS_GPC_MFOS_MFOS6_Pos           (6)                                               /*!< SYS_T::GPC_MFOS: MFOS6 Position        */
#define SYS_GPC_MFOS_MFOS6_Msk           (0x1ul << SYS_GPC_MFOS_MFOS6_Pos)                 /*!< SYS_T::GPC_MFOS: MFOS6 Mask            */

#define SYS_GPC_MFOS_MFOS7_Pos           (7)                                               /*!< SYS_T::GPC_MFOS: MFOS7 Position        */
#define SYS_GPC_MFOS_MFOS7_Msk           (0x1ul << SYS_GPC_MFOS_MFOS7_Pos)                 /*!< SYS_T::GPC_MFOS: MFOS7 Mask            */

#define SYS_GPC_MFOS_MFOS8_Pos           (8)                                               /*!< SYS_T::GPC_MFOS: MFOS8 Position        */
#define SYS_GPC_MFOS_MFOS8_Msk           (0x1ul << SYS_GPC_MFOS_MFOS8_Pos)                 /*!< SYS_T::GPC_MFOS: MFOS8 Mask            */

#define SYS_GPC_MFOS_MFOS9_Pos           (9)                                               /*!< SYS_T::GPC_MFOS: MFOS9 Position        */
#define SYS_GPC_MFOS_MFOS9_Msk           (0x1ul << SYS_GPC_MFOS_MFOS9_Pos)                 /*!< SYS_T::GPC_MFOS: MFOS9 Mask            */

#define SYS_GPC_MFOS_MFOS10_Pos          (10)                                              /*!< SYS_T::GPC_MFOS: MFOS10 Position       */
#define SYS_GPC_MFOS_MFOS10_Msk          (0x1ul << SYS_GPC_MFOS_MFOS10_Pos)                /*!< SYS_T::GPC_MFOS: MFOS10 Mask           */

#define SYS_GPC_MFOS_MFOS11_Pos          (11)                                              /*!< SYS_T::GPC_MFOS: MFOS11 Position       */
#define SYS_GPC_MFOS_MFOS11_Msk          (0x1ul << SYS_GPC_MFOS_MFOS11_Pos)                /*!< SYS_T::GPC_MFOS: MFOS11 Mask           */

#define SYS_GPC_MFOS_MFOS12_Pos          (12)                                              /*!< SYS_T::GPC_MFOS: MFOS12 Position       */
#define SYS_GPC_MFOS_MFOS12_Msk          (0x1ul << SYS_GPC_MFOS_MFOS12_Pos)                /*!< SYS_T::GPC_MFOS: MFOS12 Mask           */

#define SYS_GPC_MFOS_MFOS13_Pos          (13)                                              /*!< SYS_T::GPC_MFOS: MFOS13 Position       */
#define SYS_GPC_MFOS_MFOS13_Msk          (0x1ul << SYS_GPC_MFOS_MFOS13_Pos)                /*!< SYS_T::GPC_MFOS: MFOS13 Mask           */

#define SYS_GPC_MFOS_MFOS14_Pos          (14)                                              /*!< SYS_T::GPC_MFOS: MFOS14 Position       */
#define SYS_GPC_MFOS_MFOS14_Msk          (0x1ul << SYS_GPC_MFOS_MFOS14_Pos)                /*!< SYS_T::GPC_MFOS: MFOS14 Mask           */

#define SYS_GPD_MFOS_MFOS0_Pos           (0)                                               /*!< SYS_T::GPD_MFOS: MFOS0 Position        */
#define SYS_GPD_MFOS_MFOS0_Msk           (0x1ul << SYS_GPD_MFOS_MFOS0_Pos)                 /*!< SYS_T::GPD_MFOS: MFOS0 Mask            */

#define SYS_GPD_MFOS_MFOS1_Pos           (1)                                               /*!< SYS_T::GPD_MFOS: MFOS1 Position        */
#define SYS_GPD_MFOS_MFOS1_Msk           (0x1ul << SYS_GPD_MFOS_MFOS1_Pos)                 /*!< SYS_T::GPD_MFOS: MFOS1 Mask            */

#define SYS_GPD_MFOS_MFOS2_Pos           (2)                                               /*!< SYS_T::GPD_MFOS: MFOS2 Position        */
#define SYS_GPD_MFOS_MFOS2_Msk           (0x1ul << SYS_GPD_MFOS_MFOS2_Pos)                 /*!< SYS_T::GPD_MFOS: MFOS2 Mask            */

#define SYS_GPD_MFOS_MFOS3_Pos           (3)                                               /*!< SYS_T::GPD_MFOS: MFOS3 Position        */
#define SYS_GPD_MFOS_MFOS3_Msk           (0x1ul << SYS_GPD_MFOS_MFOS3_Pos)                 /*!< SYS_T::GPD_MFOS: MFOS3 Mask            */

#define SYS_GPD_MFOS_MFOS4_Pos           (4)                                               /*!< SYS_T::GPD_MFOS: MFOS4 Position        */
#define SYS_GPD_MFOS_MFOS4_Msk           (0x1ul << SYS_GPD_MFOS_MFOS4_Pos)                 /*!< SYS_T::GPD_MFOS: MFOS4 Mask            */

#define SYS_GPD_MFOS_MFOS5_Pos           (5)                                               /*!< SYS_T::GPD_MFOS: MFOS5 Position        */
#define SYS_GPD_MFOS_MFOS5_Msk           (0x1ul << SYS_GPD_MFOS_MFOS5_Pos)                 /*!< SYS_T::GPD_MFOS: MFOS5 Mask            */

#define SYS_GPD_MFOS_MFOS6_Pos           (6)                                               /*!< SYS_T::GPD_MFOS: MFOS6 Position        */
#define SYS_GPD_MFOS_MFOS6_Msk           (0x1ul << SYS_GPD_MFOS_MFOS6_Pos)                 /*!< SYS_T::GPD_MFOS: MFOS6 Mask            */

#define SYS_GPD_MFOS_MFOS7_Pos           (7)                                               /*!< SYS_T::GPD_MFOS: MFOS7 Position        */
#define SYS_GPD_MFOS_MFOS7_Msk           (0x1ul << SYS_GPD_MFOS_MFOS7_Pos)                 /*!< SYS_T::GPD_MFOS: MFOS7 Mask            */

#define SYS_GPD_MFOS_MFOS8_Pos           (8)                                               /*!< SYS_T::GPD_MFOS: MFOS8 Position        */
#define SYS_GPD_MFOS_MFOS8_Msk           (0x1ul << SYS_GPD_MFOS_MFOS8_Pos)                 /*!< SYS_T::GPD_MFOS: MFOS8 Mask            */

#define SYS_GPD_MFOS_MFOS9_Pos           (9)                                               /*!< SYS_T::GPD_MFOS: MFOS9 Position        */
#define SYS_GPD_MFOS_MFOS9_Msk           (0x1ul << SYS_GPD_MFOS_MFOS9_Pos)                 /*!< SYS_T::GPD_MFOS: MFOS9 Mask            */

#define SYS_GPD_MFOS_MFOS10_Pos          (10)                                              /*!< SYS_T::GPD_MFOS: MFOS10 Position       */
#define SYS_GPD_MFOS_MFOS10_Msk          (0x1ul << SYS_GPD_MFOS_MFOS10_Pos)                /*!< SYS_T::GPD_MFOS: MFOS10 Mask           */

#define SYS_GPD_MFOS_MFOS11_Pos          (11)                                              /*!< SYS_T::GPD_MFOS: MFOS11 Position       */
#define SYS_GPD_MFOS_MFOS11_Msk          (0x1ul << SYS_GPD_MFOS_MFOS11_Pos)                /*!< SYS_T::GPD_MFOS: MFOS11 Mask           */

#define SYS_GPD_MFOS_MFOS12_Pos          (12)                                              /*!< SYS_T::GPD_MFOS: MFOS12 Position       */
#define SYS_GPD_MFOS_MFOS12_Msk          (0x1ul << SYS_GPD_MFOS_MFOS12_Pos)                /*!< SYS_T::GPD_MFOS: MFOS12 Mask           */

#define SYS_GPD_MFOS_MFOS13_Pos          (13)                                              /*!< SYS_T::GPD_MFOS: MFOS13 Position       */
#define SYS_GPD_MFOS_MFOS13_Msk          (0x1ul << SYS_GPD_MFOS_MFOS13_Pos)                /*!< SYS_T::GPD_MFOS: MFOS13 Mask           */

#define SYS_GPD_MFOS_MFOS14_Pos          (14)                                              /*!< SYS_T::GPD_MFOS: MFOS14 Position       */
#define SYS_GPD_MFOS_MFOS14_Msk          (0x1ul << SYS_GPD_MFOS_MFOS14_Pos)                /*!< SYS_T::GPD_MFOS: MFOS14 Mask           */

#define SYS_GPE_MFOS_MFOS0_Pos           (0)                                               /*!< SYS_T::GPE_MFOS: MFOS0 Position        */
#define SYS_GPE_MFOS_MFOS0_Msk           (0x1ul << SYS_GPE_MFOS_MFOS0_Pos)                 /*!< SYS_T::GPE_MFOS: MFOS0 Mask            */

#define SYS_GPE_MFOS_MFOS1_Pos           (1)                                               /*!< SYS_T::GPE_MFOS: MFOS1 Position        */
#define SYS_GPE_MFOS_MFOS1_Msk           (0x1ul << SYS_GPE_MFOS_MFOS1_Pos)                 /*!< SYS_T::GPE_MFOS: MFOS1 Mask            */

#define SYS_GPE_MFOS_MFOS2_Pos           (2)                                               /*!< SYS_T::GPE_MFOS: MFOS2 Position        */
#define SYS_GPE_MFOS_MFOS2_Msk           (0x1ul << SYS_GPE_MFOS_MFOS2_Pos)                 /*!< SYS_T::GPE_MFOS: MFOS2 Mask            */

#define SYS_GPE_MFOS_MFOS3_Pos           (3)                                               /*!< SYS_T::GPE_MFOS: MFOS3 Position        */
#define SYS_GPE_MFOS_MFOS3_Msk           (0x1ul << SYS_GPE_MFOS_MFOS3_Pos)                 /*!< SYS_T::GPE_MFOS: MFOS3 Mask            */

#define SYS_GPE_MFOS_MFOS4_Pos           (4)                                               /*!< SYS_T::GPE_MFOS: MFOS4 Position        */
#define SYS_GPE_MFOS_MFOS4_Msk           (0x1ul << SYS_GPE_MFOS_MFOS4_Pos)                 /*!< SYS_T::GPE_MFOS: MFOS4 Mask            */

#define SYS_GPE_MFOS_MFOS5_Pos           (5)                                               /*!< SYS_T::GPE_MFOS: MFOS5 Position        */
#define SYS_GPE_MFOS_MFOS5_Msk           (0x1ul << SYS_GPE_MFOS_MFOS5_Pos)                 /*!< SYS_T::GPE_MFOS: MFOS5 Mask            */

#define SYS_GPE_MFOS_MFOS6_Pos           (6)                                               /*!< SYS_T::GPE_MFOS: MFOS6 Position        */
#define SYS_GPE_MFOS_MFOS6_Msk           (0x1ul << SYS_GPE_MFOS_MFOS6_Pos)                 /*!< SYS_T::GPE_MFOS: MFOS6 Mask            */

#define SYS_GPE_MFOS_MFOS7_Pos           (7)                                               /*!< SYS_T::GPE_MFOS: MFOS7 Position        */
#define SYS_GPE_MFOS_MFOS7_Msk           (0x1ul << SYS_GPE_MFOS_MFOS7_Pos)                 /*!< SYS_T::GPE_MFOS: MFOS7 Mask            */

#define SYS_GPE_MFOS_MFOS8_Pos           (8)                                               /*!< SYS_T::GPE_MFOS: MFOS8 Position        */
#define SYS_GPE_MFOS_MFOS8_Msk           (0x1ul << SYS_GPE_MFOS_MFOS8_Pos)                 /*!< SYS_T::GPE_MFOS: MFOS8 Mask            */

#define SYS_GPE_MFOS_MFOS9_Pos           (9)                                               /*!< SYS_T::GPE_MFOS: MFOS9 Position        */
#define SYS_GPE_MFOS_MFOS9_Msk           (0x1ul << SYS_GPE_MFOS_MFOS9_Pos)                 /*!< SYS_T::GPE_MFOS: MFOS9 Mask            */

#define SYS_GPE_MFOS_MFOS10_Pos          (10)                                              /*!< SYS_T::GPE_MFOS: MFOS10 Position       */
#define SYS_GPE_MFOS_MFOS10_Msk          (0x1ul << SYS_GPE_MFOS_MFOS10_Pos)                /*!< SYS_T::GPE_MFOS: MFOS10 Mask           */

#define SYS_GPE_MFOS_MFOS11_Pos          (11)                                              /*!< SYS_T::GPE_MFOS: MFOS11 Position       */
#define SYS_GPE_MFOS_MFOS11_Msk          (0x1ul << SYS_GPE_MFOS_MFOS11_Pos)                /*!< SYS_T::GPE_MFOS: MFOS11 Mask           */

#define SYS_GPE_MFOS_MFOS12_Pos          (12)                                              /*!< SYS_T::GPE_MFOS: MFOS12 Position       */
#define SYS_GPE_MFOS_MFOS12_Msk          (0x1ul << SYS_GPE_MFOS_MFOS12_Pos)                /*!< SYS_T::GPE_MFOS: MFOS12 Mask           */

#define SYS_GPE_MFOS_MFOS13_Pos          (13)                                              /*!< SYS_T::GPE_MFOS: MFOS13 Position       */
#define SYS_GPE_MFOS_MFOS13_Msk          (0x1ul << SYS_GPE_MFOS_MFOS13_Pos)                /*!< SYS_T::GPE_MFOS: MFOS13 Mask           */

#define SYS_GPE_MFOS_MFOS14_Pos          (14)                                              /*!< SYS_T::GPE_MFOS: MFOS14 Position       */
#define SYS_GPE_MFOS_MFOS14_Msk          (0x1ul << SYS_GPE_MFOS_MFOS14_Pos)                /*!< SYS_T::GPE_MFOS: MFOS14 Mask           */

#define SYS_GPE_MFOS_MFOS15_Pos          (15)                                              /*!< SYS_T::GPE_MFOS: MFOS15 Position       */
#define SYS_GPE_MFOS_MFOS15_Msk          (0x1ul << SYS_GPE_MFOS_MFOS15_Pos)                /*!< SYS_T::GPE_MFOS: MFOS15 Mask           */

#define SYS_GPF_MFOS_MFOS0_Pos           (0)                                               /*!< SYS_T::GPF_MFOS: MFOS0 Position        */
#define SYS_GPF_MFOS_MFOS0_Msk           (0x1ul << SYS_GPF_MFOS_MFOS0_Pos)                 /*!< SYS_T::GPF_MFOS: MFOS0 Mask            */

#define SYS_GPF_MFOS_MFOS1_Pos           (1)                                               /*!< SYS_T::GPF_MFOS: MFOS1 Position        */
#define SYS_GPF_MFOS_MFOS1_Msk           (0x1ul << SYS_GPF_MFOS_MFOS1_Pos)                 /*!< SYS_T::GPF_MFOS: MFOS1 Mask            */

#define SYS_GPF_MFOS_MFOS2_Pos           (2)                                               /*!< SYS_T::GPF_MFOS: MFOS2 Position        */
#define SYS_GPF_MFOS_MFOS2_Msk           (0x1ul << SYS_GPF_MFOS_MFOS2_Pos)                 /*!< SYS_T::GPF_MFOS: MFOS2 Mask            */

#define SYS_GPF_MFOS_MFOS3_Pos           (3)                                               /*!< SYS_T::GPF_MFOS: MFOS3 Position        */
#define SYS_GPF_MFOS_MFOS3_Msk           (0x1ul << SYS_GPF_MFOS_MFOS3_Pos)                 /*!< SYS_T::GPF_MFOS: MFOS3 Mask            */

#define SYS_GPF_MFOS_MFOS4_Pos           (4)                                               /*!< SYS_T::GPF_MFOS: MFOS4 Position        */
#define SYS_GPF_MFOS_MFOS4_Msk           (0x1ul << SYS_GPF_MFOS_MFOS4_Pos)                 /*!< SYS_T::GPF_MFOS: MFOS4 Mask            */

#define SYS_GPF_MFOS_MFOS5_Pos           (5)                                               /*!< SYS_T::GPF_MFOS: MFOS5 Position        */
#define SYS_GPF_MFOS_MFOS5_Msk           (0x1ul << SYS_GPF_MFOS_MFOS5_Pos)                 /*!< SYS_T::GPF_MFOS: MFOS5 Mask            */

#define SYS_GPF_MFOS_MFOS6_Pos           (6)                                               /*!< SYS_T::GPF_MFOS: MFOS6 Position        */
#define SYS_GPF_MFOS_MFOS6_Msk           (0x1ul << SYS_GPF_MFOS_MFOS6_Pos)                 /*!< SYS_T::GPF_MFOS: MFOS6 Mask            */

#define SYS_GPF_MFOS_MFOS7_Pos           (7)                                               /*!< SYS_T::GPF_MFOS: MFOS7 Position        */
#define SYS_GPF_MFOS_MFOS7_Msk           (0x1ul << SYS_GPF_MFOS_MFOS7_Pos)                 /*!< SYS_T::GPF_MFOS: MFOS7 Mask            */

#define SYS_GPF_MFOS_MFOS8_Pos           (8)                                               /*!< SYS_T::GPF_MFOS: MFOS8 Position        */
#define SYS_GPF_MFOS_MFOS8_Msk           (0x1ul << SYS_GPF_MFOS_MFOS8_Pos)                 /*!< SYS_T::GPF_MFOS: MFOS8 Mask            */

#define SYS_GPF_MFOS_MFOS9_Pos           (9)                                               /*!< SYS_T::GPF_MFOS: MFOS9 Position        */
#define SYS_GPF_MFOS_MFOS9_Msk           (0x1ul << SYS_GPF_MFOS_MFOS9_Pos)                 /*!< SYS_T::GPF_MFOS: MFOS9 Mask            */

#define SYS_GPF_MFOS_MFOS10_Pos          (10)                                              /*!< SYS_T::GPF_MFOS: MFOS10 Position       */
#define SYS_GPF_MFOS_MFOS10_Msk          (0x1ul << SYS_GPF_MFOS_MFOS10_Pos)                /*!< SYS_T::GPF_MFOS: MFOS10 Mask           */

#define SYS_GPF_MFOS_MFOS11_Pos          (11)                                              /*!< SYS_T::GPF_MFOS: MFOS11 Position       */
#define SYS_GPF_MFOS_MFOS11_Msk          (0x1ul << SYS_GPF_MFOS_MFOS11_Pos)                /*!< SYS_T::GPF_MFOS: MFOS11 Mask           */

#define SYS_GPG_MFOS_MFOS0_Pos           (0)                                               /*!< SYS_T::GPG_MFOS: MFOS0 Position        */
#define SYS_GPG_MFOS_MFOS0_Msk           (0x1ul << SYS_GPG_MFOS_MFOS0_Pos)                 /*!< SYS_T::GPG_MFOS: MFOS0 Mask            */

#define SYS_GPG_MFOS_MFOS1_Pos           (1)                                               /*!< SYS_T::GPG_MFOS: MFOS1 Position        */
#define SYS_GPG_MFOS_MFOS1_Msk           (0x1ul << SYS_GPG_MFOS_MFOS1_Pos)                 /*!< SYS_T::GPG_MFOS: MFOS1 Mask            */

#define SYS_GPG_MFOS_MFOS2_Pos           (2)                                               /*!< SYS_T::GPG_MFOS: MFOS2 Position        */
#define SYS_GPG_MFOS_MFOS2_Msk           (0x1ul << SYS_GPG_MFOS_MFOS2_Pos)                 /*!< SYS_T::GPG_MFOS: MFOS2 Mask            */

#define SYS_GPG_MFOS_MFOS3_Pos           (3)                                               /*!< SYS_T::GPG_MFOS: MFOS3 Position        */
#define SYS_GPG_MFOS_MFOS3_Msk           (0x1ul << SYS_GPG_MFOS_MFOS3_Pos)                 /*!< SYS_T::GPG_MFOS: MFOS3 Mask            */

#define SYS_GPG_MFOS_MFOS4_Pos           (4)                                               /*!< SYS_T::GPG_MFOS: MFOS4 Position        */
#define SYS_GPG_MFOS_MFOS4_Msk           (0x1ul << SYS_GPG_MFOS_MFOS4_Pos)                 /*!< SYS_T::GPG_MFOS: MFOS4 Mask            */

#define SYS_GPG_MFOS_MFOS5_Pos           (5)                                               /*!< SYS_T::GPG_MFOS: MFOS5 Position        */
#define SYS_GPG_MFOS_MFOS5_Msk           (0x1ul << SYS_GPG_MFOS_MFOS5_Pos)                 /*!< SYS_T::GPG_MFOS: MFOS5 Mask            */

#define SYS_GPG_MFOS_MFOS6_Pos           (6)                                               /*!< SYS_T::GPG_MFOS: MFOS6 Position        */
#define SYS_GPG_MFOS_MFOS6_Msk           (0x1ul << SYS_GPG_MFOS_MFOS6_Pos)                 /*!< SYS_T::GPG_MFOS: MFOS6 Mask            */

#define SYS_GPG_MFOS_MFOS7_Pos           (7)                                               /*!< SYS_T::GPG_MFOS: MFOS7 Position        */
#define SYS_GPG_MFOS_MFOS7_Msk           (0x1ul << SYS_GPG_MFOS_MFOS7_Pos)                 /*!< SYS_T::GPG_MFOS: MFOS7 Mask            */

#define SYS_GPG_MFOS_MFOS8_Pos           (8)                                               /*!< SYS_T::GPG_MFOS: MFOS8 Position        */
#define SYS_GPG_MFOS_MFOS8_Msk           (0x1ul << SYS_GPG_MFOS_MFOS8_Pos)                 /*!< SYS_T::GPG_MFOS: MFOS8 Mask            */

#define SYS_GPG_MFOS_MFOS9_Pos           (9)                                               /*!< SYS_T::GPG_MFOS: MFOS9 Position        */
#define SYS_GPG_MFOS_MFOS9_Msk           (0x1ul << SYS_GPG_MFOS_MFOS9_Pos)                 /*!< SYS_T::GPG_MFOS: MFOS9 Mask            */

#define SYS_GPG_MFOS_MFOS10_Pos          (10)                                              /*!< SYS_T::GPG_MFOS: MFOS10 Position       */
#define SYS_GPG_MFOS_MFOS10_Msk          (0x1ul << SYS_GPG_MFOS_MFOS10_Pos)                /*!< SYS_T::GPG_MFOS: MFOS10 Mask           */

#define SYS_GPG_MFOS_MFOS11_Pos          (11)                                              /*!< SYS_T::GPG_MFOS: MFOS11 Position       */
#define SYS_GPG_MFOS_MFOS11_Msk          (0x1ul << SYS_GPG_MFOS_MFOS11_Pos)                /*!< SYS_T::GPG_MFOS: MFOS11 Mask           */

#define SYS_GPG_MFOS_MFOS12_Pos          (12)                                              /*!< SYS_T::GPG_MFOS: MFOS12 Position       */
#define SYS_GPG_MFOS_MFOS12_Msk          (0x1ul << SYS_GPG_MFOS_MFOS12_Pos)                /*!< SYS_T::GPG_MFOS: MFOS12 Mask           */

#define SYS_GPG_MFOS_MFOS13_Pos          (13)                                              /*!< SYS_T::GPG_MFOS: MFOS13 Position       */
#define SYS_GPG_MFOS_MFOS13_Msk          (0x1ul << SYS_GPG_MFOS_MFOS13_Pos)                /*!< SYS_T::GPG_MFOS: MFOS13 Mask           */

#define SYS_GPG_MFOS_MFOS14_Pos          (14)                                              /*!< SYS_T::GPG_MFOS: MFOS14 Position       */
#define SYS_GPG_MFOS_MFOS14_Msk          (0x1ul << SYS_GPG_MFOS_MFOS14_Pos)                /*!< SYS_T::GPG_MFOS: MFOS14 Mask           */

#define SYS_GPG_MFOS_MFOS15_Pos          (15)                                              /*!< SYS_T::GPG_MFOS: MFOS15 Position       */
#define SYS_GPG_MFOS_MFOS15_Msk          (0x1ul << SYS_GPG_MFOS_MFOS15_Pos)                /*!< SYS_T::GPG_MFOS: MFOS15 Mask           */

#define SYS_GPH_MFOS_MFOS0_Pos           (0)                                               /*!< SYS_T::GPH_MFOS: MFOS0 Position        */
#define SYS_GPH_MFOS_MFOS0_Msk           (0x1ul << SYS_GPH_MFOS_MFOS0_Pos)                 /*!< SYS_T::GPH_MFOS: MFOS0 Mask            */

#define SYS_GPH_MFOS_MFOS1_Pos           (1)                                               /*!< SYS_T::GPH_MFOS: MFOS1 Position        */
#define SYS_GPH_MFOS_MFOS1_Msk           (0x1ul << SYS_GPH_MFOS_MFOS1_Pos)                 /*!< SYS_T::GPH_MFOS: MFOS1 Mask            */

#define SYS_GPH_MFOS_MFOS2_Pos           (2)                                               /*!< SYS_T::GPH_MFOS: MFOS2 Position        */
#define SYS_GPH_MFOS_MFOS2_Msk           (0x1ul << SYS_GPH_MFOS_MFOS2_Pos)                 /*!< SYS_T::GPH_MFOS: MFOS2 Mask            */

#define SYS_GPH_MFOS_MFOS3_Pos           (3)                                               /*!< SYS_T::GPH_MFOS: MFOS3 Position        */
#define SYS_GPH_MFOS_MFOS3_Msk           (0x1ul << SYS_GPH_MFOS_MFOS3_Pos)                 /*!< SYS_T::GPH_MFOS: MFOS3 Mask            */

#define SYS_GPH_MFOS_MFOS4_Pos           (4)                                               /*!< SYS_T::GPH_MFOS: MFOS4 Position        */
#define SYS_GPH_MFOS_MFOS4_Msk           (0x1ul << SYS_GPH_MFOS_MFOS4_Pos)                 /*!< SYS_T::GPH_MFOS: MFOS4 Mask            */

#define SYS_GPH_MFOS_MFOS5_Pos           (5)                                               /*!< SYS_T::GPH_MFOS: MFOS5 Position        */
#define SYS_GPH_MFOS_MFOS5_Msk           (0x1ul << SYS_GPH_MFOS_MFOS5_Pos)                 /*!< SYS_T::GPH_MFOS: MFOS5 Mask            */

#define SYS_GPH_MFOS_MFOS6_Pos           (6)                                               /*!< SYS_T::GPH_MFOS: MFOS6 Position        */
#define SYS_GPH_MFOS_MFOS6_Msk           (0x1ul << SYS_GPH_MFOS_MFOS6_Pos)                 /*!< SYS_T::GPH_MFOS: MFOS6 Mask            */

#define SYS_GPH_MFOS_MFOS7_Pos           (7)                                               /*!< SYS_T::GPH_MFOS: MFOS7 Position        */
#define SYS_GPH_MFOS_MFOS7_Msk           (0x1ul << SYS_GPH_MFOS_MFOS7_Pos)                 /*!< SYS_T::GPH_MFOS: MFOS7 Mask            */

#define SYS_GPH_MFOS_MFOS8_Pos           (8)                                               /*!< SYS_T::GPH_MFOS: MFOS8 Position        */
#define SYS_GPH_MFOS_MFOS8_Msk           (0x1ul << SYS_GPH_MFOS_MFOS8_Pos)                 /*!< SYS_T::GPH_MFOS: MFOS8 Mask            */

#define SYS_GPH_MFOS_MFOS9_Pos           (9)                                               /*!< SYS_T::GPH_MFOS: MFOS9 Position        */
#define SYS_GPH_MFOS_MFOS9_Msk           (0x1ul << SYS_GPH_MFOS_MFOS9_Pos)                 /*!< SYS_T::GPH_MFOS: MFOS9 Mask            */

#define SYS_GPH_MFOS_MFOS10_Pos          (10)                                              /*!< SYS_T::GPH_MFOS: MFOS10 Position       */
#define SYS_GPH_MFOS_MFOS10_Msk          (0x1ul << SYS_GPH_MFOS_MFOS10_Pos)                /*!< SYS_T::GPH_MFOS: MFOS10 Mask           */

#define SYS_GPH_MFOS_MFOS11_Pos          (11)                                              /*!< SYS_T::GPH_MFOS: MFOS11 Position       */
#define SYS_GPH_MFOS_MFOS11_Msk          (0x1ul << SYS_GPH_MFOS_MFOS11_Pos)                /*!< SYS_T::GPH_MFOS: MFOS11 Mask           */

#define SYS_GPH_MFOS_MFOS12_Pos          (12)                                              /*!< SYS_T::GPH_MFOS: MFOS12 Position       */
#define SYS_GPH_MFOS_MFOS12_Msk          (0x1ul << SYS_GPH_MFOS_MFOS12_Pos)                /*!< SYS_T::GPH_MFOS: MFOS12 Mask           */

#define SYS_GPH_MFOS_MFOS13_Pos          (13)                                              /*!< SYS_T::GPH_MFOS: MFOS13 Position       */
#define SYS_GPH_MFOS_MFOS13_Msk          (0x1ul << SYS_GPH_MFOS_MFOS13_Pos)                /*!< SYS_T::GPH_MFOS: MFOS13 Mask           */

#define SYS_GPH_MFOS_MFOS14_Pos          (14)                                              /*!< SYS_T::GPH_MFOS: MFOS14 Position       */
#define SYS_GPH_MFOS_MFOS14_Msk          (0x1ul << SYS_GPH_MFOS_MFOS14_Pos)                /*!< SYS_T::GPH_MFOS: MFOS14 Mask           */

#define SYS_GPH_MFOS_MFOS15_Pos          (15)                                              /*!< SYS_T::GPH_MFOS: MFOS15 Position       */
#define SYS_GPH_MFOS_MFOS15_Msk          (0x1ul << SYS_GPH_MFOS_MFOS15_Pos)                /*!< SYS_T::GPH_MFOS: MFOS15 Mask           */

#define SYS_GPI_MFOS_MFOS6_Pos           (6)                                               /*!< SYS_T::GPI_MFOS: MFOS6 Position        */
#define SYS_GPI_MFOS_MFOS6_Msk           (0x1ul << SYS_GPI_MFOS_MFOS6_Pos)                 /*!< SYS_T::GPI_MFOS: MFOS6 Mask            */

#define SYS_GPI_MFOS_MFOS7_Pos           (7)                                               /*!< SYS_T::GPI_MFOS: MFOS7 Position        */
#define SYS_GPI_MFOS_MFOS7_Msk           (0x1ul << SYS_GPI_MFOS_MFOS7_Pos)                 /*!< SYS_T::GPI_MFOS: MFOS7 Mask            */

#define SYS_GPI_MFOS_MFOS8_Pos           (8)                                               /*!< SYS_T::GPI_MFOS: MFOS8 Position        */
#define SYS_GPI_MFOS_MFOS8_Msk           (0x1ul << SYS_GPI_MFOS_MFOS8_Pos)                 /*!< SYS_T::GPI_MFOS: MFOS8 Mask            */

#define SYS_GPI_MFOS_MFOS9_Pos           (9)                                               /*!< SYS_T::GPI_MFOS: MFOS9 Position        */
#define SYS_GPI_MFOS_MFOS9_Msk           (0x1ul << SYS_GPI_MFOS_MFOS9_Pos)                 /*!< SYS_T::GPI_MFOS: MFOS9 Mask            */

#define SYS_GPI_MFOS_MFOS10_Pos          (10)                                              /*!< SYS_T::GPI_MFOS: MFOS10 Position       */
#define SYS_GPI_MFOS_MFOS10_Msk          (0x1ul << SYS_GPI_MFOS_MFOS10_Pos)                /*!< SYS_T::GPI_MFOS: MFOS10 Mask           */

#define SYS_GPI_MFOS_MFOS11_Pos          (11)                                              /*!< SYS_T::GPI_MFOS: MFOS11 Position       */
#define SYS_GPI_MFOS_MFOS11_Msk          (0x1ul << SYS_GPI_MFOS_MFOS11_Pos)                /*!< SYS_T::GPI_MFOS: MFOS11 Mask           */

#define SYS_GPI_MFOS_MFOS12_Pos          (12)                                              /*!< SYS_T::GPI_MFOS: MFOS12 Position       */
#define SYS_GPI_MFOS_MFOS12_Msk          (0x1ul << SYS_GPI_MFOS_MFOS12_Pos)                /*!< SYS_T::GPI_MFOS: MFOS12 Mask           */

#define SYS_GPI_MFOS_MFOS13_Pos          (13)                                              /*!< SYS_T::GPI_MFOS: MFOS13 Position       */
#define SYS_GPI_MFOS_MFOS13_Msk          (0x1ul << SYS_GPI_MFOS_MFOS13_Pos)                /*!< SYS_T::GPI_MFOS: MFOS13 Mask           */

#define SYS_GPI_MFOS_MFOS14_Pos          (14)                                              /*!< SYS_T::GPI_MFOS: MFOS14 Position       */
#define SYS_GPI_MFOS_MFOS14_Msk          (0x1ul << SYS_GPI_MFOS_MFOS14_Pos)                /*!< SYS_T::GPI_MFOS: MFOS14 Mask           */

#define SYS_GPI_MFOS_MFOS15_Pos          (15)                                              /*!< SYS_T::GPI_MFOS: MFOS15 Position       */
#define SYS_GPI_MFOS_MFOS15_Msk          (0x1ul << SYS_GPI_MFOS_MFOS15_Pos)                /*!< SYS_T::GPI_MFOS: MFOS15 Mask           */

#define SYS_GPJ_MFOS_MFOS0_Pos           (0)                                               /*!< SYS_T::GPJ_MFOS: MFOS0 Position        */
#define SYS_GPJ_MFOS_MFOS0_Msk           (0x1ul << SYS_GPJ_MFOS_MFOS0_Pos)                 /*!< SYS_T::GPJ_MFOS: MFOS0 Mask            */

#define SYS_GPJ_MFOS_MFOS1_Pos           (1)                                               /*!< SYS_T::GPJ_MFOS: MFOS1 Position        */
#define SYS_GPJ_MFOS_MFOS1_Msk           (0x1ul << SYS_GPJ_MFOS_MFOS1_Pos)                 /*!< SYS_T::GPJ_MFOS: MFOS1 Mask            */

#define SYS_GPJ_MFOS_MFOS2_Pos           (2)                                               /*!< SYS_T::GPJ_MFOS: MFOS2 Position        */
#define SYS_GPJ_MFOS_MFOS2_Msk           (0x1ul << SYS_GPJ_MFOS_MFOS2_Pos)                 /*!< SYS_T::GPJ_MFOS: MFOS2 Mask            */

#define SYS_GPJ_MFOS_MFOS3_Pos           (3)                                               /*!< SYS_T::GPJ_MFOS: MFOS3 Position        */
#define SYS_GPJ_MFOS_MFOS3_Msk           (0x1ul << SYS_GPJ_MFOS_MFOS3_Pos)                 /*!< SYS_T::GPJ_MFOS: MFOS3 Mask            */

#define SYS_GPJ_MFOS_MFOS4_Pos           (4)                                               /*!< SYS_T::GPJ_MFOS: MFOS4 Position        */
#define SYS_GPJ_MFOS_MFOS4_Msk           (0x1ul << SYS_GPJ_MFOS_MFOS4_Pos)                 /*!< SYS_T::GPJ_MFOS: MFOS4 Mask            */

#define SYS_GPJ_MFOS_MFOS5_Pos           (5)                                               /*!< SYS_T::GPJ_MFOS: MFOS5 Position        */
#define SYS_GPJ_MFOS_MFOS5_Msk           (0x1ul << SYS_GPJ_MFOS_MFOS5_Pos)                 /*!< SYS_T::GPJ_MFOS: MFOS5 Mask            */

#define SYS_GPJ_MFOS_MFOS6_Pos           (6)                                               /*!< SYS_T::GPJ_MFOS: MFOS6 Position        */
#define SYS_GPJ_MFOS_MFOS6_Msk           (0x1ul << SYS_GPJ_MFOS_MFOS6_Pos)                 /*!< SYS_T::GPJ_MFOS: MFOS6 Mask            */

#define SYS_GPJ_MFOS_MFOS7_Pos           (7)                                               /*!< SYS_T::GPJ_MFOS: MFOS7 Position        */
#define SYS_GPJ_MFOS_MFOS7_Msk           (0x1ul << SYS_GPJ_MFOS_MFOS7_Pos)                 /*!< SYS_T::GPJ_MFOS: MFOS7 Mask            */

#define SYS_GPJ_MFOS_MFOS8_Pos           (8)                                               /*!< SYS_T::GPJ_MFOS: MFOS8 Position        */
#define SYS_GPJ_MFOS_MFOS8_Msk           (0x1ul << SYS_GPJ_MFOS_MFOS8_Pos)                 /*!< SYS_T::GPJ_MFOS: MFOS8 Mask            */

#define SYS_GPJ_MFOS_MFOS9_Pos           (9)                                               /*!< SYS_T::GPJ_MFOS: MFOS9 Position        */
#define SYS_GPJ_MFOS_MFOS9_Msk           (0x1ul << SYS_GPJ_MFOS_MFOS9_Pos)                 /*!< SYS_T::GPJ_MFOS: MFOS9 Mask            */

#define SYS_GPJ_MFOS_MFOS10_Pos          (10)                                              /*!< SYS_T::GPJ_MFOS: MFOS10 Position       */
#define SYS_GPJ_MFOS_MFOS10_Msk          (0x1ul << SYS_GPJ_MFOS_MFOS10_Pos)                /*!< SYS_T::GPJ_MFOS: MFOS10 Mask           */

#define SYS_GPJ_MFOS_MFOS11_Pos          (11)                                              /*!< SYS_T::GPJ_MFOS: MFOS11 Position       */
#define SYS_GPJ_MFOS_MFOS11_Msk          (0x1ul << SYS_GPJ_MFOS_MFOS11_Pos)                /*!< SYS_T::GPJ_MFOS: MFOS11 Mask           */

#define SYS_GPJ_MFOS_MFOS12_Pos          (12)                                              /*!< SYS_T::GPJ_MFOS: MFOS12 Position       */
#define SYS_GPJ_MFOS_MFOS12_Msk          (0x1ul << SYS_GPJ_MFOS_MFOS12_Pos)                /*!< SYS_T::GPJ_MFOS: MFOS12 Mask           */

#define SYS_GPJ_MFOS_MFOS13_Pos          (13)                                              /*!< SYS_T::GPJ_MFOS: MFOS13 Position       */
#define SYS_GPJ_MFOS_MFOS13_Msk          (0x1ul << SYS_GPJ_MFOS_MFOS13_Pos)                /*!< SYS_T::GPJ_MFOS: MFOS13 Mask           */

#define SYS_REGLCTL_REGLCTL_Pos          (0)                                               /*!< SYS_T::REGLCTL: REGLCTL Position       */
#define SYS_REGLCTL_REGLCTL_Msk          (0xfful << SYS_REGLCTL_REGLCTL_Pos)               /*!< SYS_T::REGLCTL: REGLCTL Mask           */

#define SYS_TCTL48M_FREQSEL_Pos          (0)                                               /*!< SYS_T::TCTL48M: FREQSEL Position       */
#define SYS_TCTL48M_FREQSEL_Msk          (0x3ul << SYS_TCTL48M_FREQSEL_Pos)                /*!< SYS_T::TCTL48M: FREQSEL Mask           */

#define SYS_TCTL48M_ACCURSEL_Pos         (2)                                               /*!< SYS_T::TCTL48M: ACCURSEL Position      */
#define SYS_TCTL48M_ACCURSEL_Msk         (0x3ul << SYS_TCTL48M_ACCURSEL_Pos)               /*!< SYS_T::TCTL48M: ACCURSEL Mask          */

#define SYS_TCTL48M_LOOPSEL_Pos          (4)                                               /*!< SYS_T::TCTL48M: LOOPSEL Position       */
#define SYS_TCTL48M_LOOPSEL_Msk          (0x3ul << SYS_TCTL48M_LOOPSEL_Pos)                /*!< SYS_T::TCTL48M: LOOPSEL Mask           */

#define SYS_TCTL48M_RETRYCNT_Pos         (6)                                               /*!< SYS_T::TCTL48M: RETRYCNT Position      */
#define SYS_TCTL48M_RETRYCNT_Msk         (0x3ul << SYS_TCTL48M_RETRYCNT_Pos)               /*!< SYS_T::TCTL48M: RETRYCNT Mask          */

#define SYS_TCTL48M_CESTOPEN_Pos         (8)                                               /*!< SYS_T::TCTL48M: CESTOPEN Position      */
#define SYS_TCTL48M_CESTOPEN_Msk         (0x1ul << SYS_TCTL48M_CESTOPEN_Pos)               /*!< SYS_T::TCTL48M: CESTOPEN Mask          */

#define SYS_TCTL48M_BOUNDEN_Pos          (9)                                               /*!< SYS_T::TCTL48M: BOUNDEN Position       */
#define SYS_TCTL48M_BOUNDEN_Msk          (0x1ul << SYS_TCTL48M_BOUNDEN_Pos)                /*!< SYS_T::TCTL48M: BOUNDEN Mask           */

#define SYS_TCTL48M_REFCKSEL_Pos         (10)                                              /*!< SYS_T::TCTL48M: REFCKSEL Position      */
#define SYS_TCTL48M_REFCKSEL_Msk         (0x1ul << SYS_TCTL48M_REFCKSEL_Pos)               /*!< SYS_T::TCTL48M: REFCKSEL Mask          */

#define SYS_TCTL48M_BOUNDARY_Pos         (16)                                              /*!< SYS_T::TCTL48M: BOUNDARY Position      */
#define SYS_TCTL48M_BOUNDARY_Msk         (0x1ful << SYS_TCTL48M_BOUNDARY_Pos)              /*!< SYS_T::TCTL48M: BOUNDARY Mask          */

#define SYS_TIEN48M_TFAILIEN_Pos         (1)                                               /*!< SYS_T::TIEN48M: TFAILIEN Position      */
#define SYS_TIEN48M_TFAILIEN_Msk         (0x1ul << SYS_TIEN48M_TFAILIEN_Pos)               /*!< SYS_T::TIEN48M: TFAILIEN Mask          */

#define SYS_TIEN48M_CLKEIEN_Pos          (2)                                               /*!< SYS_T::TIEN48M: CLKEIEN Position       */
#define SYS_TIEN48M_CLKEIEN_Msk          (0x1ul << SYS_TIEN48M_CLKEIEN_Pos)                /*!< SYS_T::TIEN48M: CLKEIEN Mask           */

#define SYS_TISTS48M_FREQLOCK_Pos        (0)                                               /*!< SYS_T::TISTS48M: FREQLOCK Position     */
#define SYS_TISTS48M_FREQLOCK_Msk        (0x1ul << SYS_TISTS48M_FREQLOCK_Pos)              /*!< SYS_T::TISTS48M: FREQLOCK Mask         */

#define SYS_TISTS48M_TFAILIF_Pos         (1)                                               /*!< SYS_T::TISTS48M: TFAILIF Position      */
#define SYS_TISTS48M_TFAILIF_Msk         (0x1ul << SYS_TISTS48M_TFAILIF_Pos)               /*!< SYS_T::TISTS48M: TFAILIF Mask          */

#define SYS_TISTS48M_CLKERRIF_Pos        (2)                                               /*!< SYS_T::TISTS48M: CLKERRIF Position     */
#define SYS_TISTS48M_CLKERRIF_Msk        (0x1ul << SYS_TISTS48M_CLKERRIF_Pos)              /*!< SYS_T::TISTS48M: CLKERRIF Mask         */

#define SYS_TISTS48M_OVBDIF_Pos          (3)                                               /*!< SYS_T::TISTS48M: OVBDIF Position       */
#define SYS_TISTS48M_OVBDIF_Msk          (0x1ul << SYS_TISTS48M_OVBDIF_Pos)                /*!< SYS_T::TISTS48M: OVBDIF Mask           */

#define SYS_TCTL12M_FREQSEL_Pos          (0)                                               /*!< SYS_T::TCTL12M: FREQSEL Position       */
#define SYS_TCTL12M_FREQSEL_Msk          (0x3ul << SYS_TCTL12M_FREQSEL_Pos)                /*!< SYS_T::TCTL12M: FREQSEL Mask           */

#define SYS_TCTL12M_ACCURSEL_Pos         (2)                                               /*!< SYS_T::TCTL12M: ACCURSEL Position      */
#define SYS_TCTL12M_ACCURSEL_Msk         (0x3ul << SYS_TCTL12M_ACCURSEL_Pos)               /*!< SYS_T::TCTL12M: ACCURSEL Mask          */

#define SYS_TCTL12M_LOOPSEL_Pos          (4)                                               /*!< SYS_T::TCTL12M: LOOPSEL Position       */
#define SYS_TCTL12M_LOOPSEL_Msk          (0x3ul << SYS_TCTL12M_LOOPSEL_Pos)                /*!< SYS_T::TCTL12M: LOOPSEL Mask           */

#define SYS_TCTL12M_RETRYCNT_Pos         (6)                                               /*!< SYS_T::TCTL12M: RETRYCNT Position      */
#define SYS_TCTL12M_RETRYCNT_Msk         (0x3ul << SYS_TCTL12M_RETRYCNT_Pos)               /*!< SYS_T::TCTL12M: RETRYCNT Mask          */

#define SYS_TCTL12M_CESTOPEN_Pos         (8)                                               /*!< SYS_T::TCTL12M: CESTOPEN Position      */
#define SYS_TCTL12M_CESTOPEN_Msk         (0x1ul << SYS_TCTL12M_CESTOPEN_Pos)               /*!< SYS_T::TCTL12M: CESTOPEN Mask          */

#define SYS_TCTL12M_BOUNDEN_Pos          (9)                                               /*!< SYS_T::TCTL12M: BOUNDEN Position       */
#define SYS_TCTL12M_BOUNDEN_Msk          (0x1ul << SYS_TCTL12M_BOUNDEN_Pos)                /*!< SYS_T::TCTL12M: BOUNDEN Mask           */

#define SYS_TCTL12M_REFCKSEL_Pos         (10)                                              /*!< SYS_T::TCTL12M: REFCKSEL Position      */
#define SYS_TCTL12M_REFCKSEL_Msk         (0x1ul << SYS_TCTL12M_REFCKSEL_Pos)               /*!< SYS_T::TCTL12M: REFCKSEL Mask          */

#define SYS_TCTL12M_BOUNDARY_Pos         (16)                                              /*!< SYS_T::TCTL12M: BOUNDARY Position      */
#define SYS_TCTL12M_BOUNDARY_Msk         (0x1ful << SYS_TCTL12M_BOUNDARY_Pos)              /*!< SYS_T::TCTL12M: BOUNDARY Mask          */

#define SYS_TIEN12M_TFAILIEN_Pos         (1)                                               /*!< SYS_T::TIEN12M: TFAILIEN Position      */
#define SYS_TIEN12M_TFAILIEN_Msk         (0x1ul << SYS_TIEN12M_TFAILIEN_Pos)               /*!< SYS_T::TIEN12M: TFAILIEN Mask          */

#define SYS_TIEN12M_CLKEIEN_Pos          (2)                                               /*!< SYS_T::TIEN12M: CLKEIEN Position       */
#define SYS_TIEN12M_CLKEIEN_Msk          (0x1ul << SYS_TIEN12M_CLKEIEN_Pos)                /*!< SYS_T::TIEN12M: CLKEIEN Mask           */

#define SYS_TISTS12M_FREQLOCK_Pos        (0)                                               /*!< SYS_T::TISTS12M: FREQLOCK Position     */
#define SYS_TISTS12M_FREQLOCK_Msk        (0x1ul << SYS_TISTS12M_FREQLOCK_Pos)              /*!< SYS_T::TISTS12M: FREQLOCK Mask         */

#define SYS_TISTS12M_TFAILIF_Pos         (1)                                               /*!< SYS_T::TISTS12M: TFAILIF Position      */
#define SYS_TISTS12M_TFAILIF_Msk         (0x1ul << SYS_TISTS12M_TFAILIF_Pos)               /*!< SYS_T::TISTS12M: TFAILIF Mask          */

#define SYS_TISTS12M_CLKERRIF_Pos        (2)                                               /*!< SYS_T::TISTS12M: CLKERRIF Position     */
#define SYS_TISTS12M_CLKERRIF_Msk        (0x1ul << SYS_TISTS12M_CLKERRIF_Pos)              /*!< SYS_T::TISTS12M: CLKERRIF Mask         */

#define SYS_TISTS12M_OVBDIF_Pos          (3)                                               /*!< SYS_T::TISTS12M: OVBDIF Position       */
#define SYS_TISTS12M_OVBDIF_Msk          (0x1ul << SYS_TISTS12M_OVBDIF_Pos)                /*!< SYS_T::TISTS12M: OVBDIF Mask           */

#define SYS_TCTLMIRC_FREQSEL_Pos         (0)                                               /*!< SYS_T::TCTLMIRC: FREQSEL Position      */
#define SYS_TCTLMIRC_FREQSEL_Msk         (0x3ul << SYS_TCTLMIRC_FREQSEL_Pos)               /*!< SYS_T::TCTLMIRC: FREQSEL Mask          */

#define SYS_TCTLMIRC_ACCURSEL_Pos        (2)                                               /*!< SYS_T::TCTLMIRC: ACCURSEL Position     */
#define SYS_TCTLMIRC_ACCURSEL_Msk        (0x3ul << SYS_TCTLMIRC_ACCURSEL_Pos)              /*!< SYS_T::TCTLMIRC: ACCURSEL Mask         */

#define SYS_TCTLMIRC_LOOPSEL_Pos         (4)                                               /*!< SYS_T::TCTLMIRC: LOOPSEL Position      */
#define SYS_TCTLMIRC_LOOPSEL_Msk         (0x3ul << SYS_TCTLMIRC_LOOPSEL_Pos)               /*!< SYS_T::TCTLMIRC: LOOPSEL Mask          */

#define SYS_TCTLMIRC_RETRYCNT_Pos        (6)                                               /*!< SYS_T::TCTLMIRC: RETRYCNT Position     */
#define SYS_TCTLMIRC_RETRYCNT_Msk        (0x3ul << SYS_TCTLMIRC_RETRYCNT_Pos)              /*!< SYS_T::TCTLMIRC: RETRYCNT Mask         */

#define SYS_TCTLMIRC_CESTOPEN_Pos        (8)                                               /*!< SYS_T::TCTLMIRC: CESTOPEN Position     */
#define SYS_TCTLMIRC_CESTOPEN_Msk        (0x1ul << SYS_TCTLMIRC_CESTOPEN_Pos)              /*!< SYS_T::TCTLMIRC: CESTOPEN Mask         */

#define SYS_TCTLMIRC_BOUNDEN_Pos         (9)                                               /*!< SYS_T::TCTLMIRC: BOUNDEN Position      */
#define SYS_TCTLMIRC_BOUNDEN_Msk         (0x1ul << SYS_TCTLMIRC_BOUNDEN_Pos)               /*!< SYS_T::TCTLMIRC: BOUNDEN Mask          */

#define SYS_TCTLMIRC_REFCKSEL_Pos        (10)                                              /*!< SYS_T::TCTLMIRC: REFCKSEL Position     */
#define SYS_TCTLMIRC_REFCKSEL_Msk        (0x1ul << SYS_TCTLMIRC_REFCKSEL_Pos)              /*!< SYS_T::TCTLMIRC: REFCKSEL Mask         */

#define SYS_TCTLMIRC_BOUNDARY_Pos        (16)                                              /*!< SYS_T::TCTLMIRC: BOUNDARY Position     */
#define SYS_TCTLMIRC_BOUNDARY_Msk        (0x1ful << SYS_TCTLMIRC_BOUNDARY_Pos)             /*!< SYS_T::TCTLMIRC: BOUNDARY Mask         */

#define SYS_TIENMIRC_TFAILIEN_Pos        (1)                                               /*!< SYS_T::TIENMIRC: TFAILIEN Position     */
#define SYS_TIENMIRC_TFAILIEN_Msk        (0x1ul << SYS_TIENMIRC_TFAILIEN_Pos)              /*!< SYS_T::TIENMIRC: TFAILIEN Mask         */

#define SYS_TIENMIRC_CLKEIEN_Pos         (2)                                               /*!< SYS_T::TIENMIRC: CLKEIEN Position      */
#define SYS_TIENMIRC_CLKEIEN_Msk         (0x1ul << SYS_TIENMIRC_CLKEIEN_Pos)               /*!< SYS_T::TIENMIRC: CLKEIEN Mask          */

#define SYS_TISTSMIRC_FREQLOCK_Pos       (0)                                               /*!< SYS_T::TISTSMIRC: FREQLOCK Position    */
#define SYS_TISTSMIRC_FREQLOCK_Msk       (0x1ul << SYS_TISTSMIRC_FREQLOCK_Pos)             /*!< SYS_T::TISTSMIRC: FREQLOCK Mask        */

#define SYS_TISTSMIRC_TFAILIF_Pos        (1)                                               /*!< SYS_T::TISTSMIRC: TFAILIF Position     */
#define SYS_TISTSMIRC_TFAILIF_Msk        (0x1ul << SYS_TISTSMIRC_TFAILIF_Pos)              /*!< SYS_T::TISTSMIRC: TFAILIF Mask         */

#define SYS_TISTSMIRC_CLKERRIF_Pos       (2)                                               /*!< SYS_T::TISTSMIRC: CLKERRIF Position    */
#define SYS_TISTSMIRC_CLKERRIF_Msk       (0x1ul << SYS_TISTSMIRC_CLKERRIF_Pos)             /*!< SYS_T::TISTSMIRC: CLKERRIF Mask        */

#define SYS_TISTSMIRC_OVBDIF_Pos         (3)                                               /*!< SYS_T::TISTSMIRC: OVBDIF Position      */
#define SYS_TISTSMIRC_OVBDIF_Msk         (0x1ul << SYS_TISTSMIRC_OVBDIF_Pos)               /*!< SYS_T::TISTSMIRC: OVBDIF Mask          */

#define SYS_ACMPRST_ACMP01RST_Pos        (0)                                               /*!< SYS_T::ACMPRST: ACMP01RST Position     */
#define SYS_ACMPRST_ACMP01RST_Msk        (0x1ul << SYS_ACMPRST_ACMP01RST_Pos)              /*!< SYS_T::ACMPRST: ACMP01RST Mask         */

#define SYS_ACMPRST_ACMP23RST_Pos        (1)                                               /*!< SYS_T::ACMPRST: ACMP23RST Position     */
#define SYS_ACMPRST_ACMP23RST_Msk        (0x1ul << SYS_ACMPRST_ACMP23RST_Pos)              /*!< SYS_T::ACMPRST: ACMP23RST Mask         */

#define SYS_AWFRST_AWF0RST_Pos           (0)                                               /*!< SYS_T::AWFRST: AWF0RST Position        */
#define SYS_AWFRST_AWF0RST_Msk           (0x1ul << SYS_AWFRST_AWF0RST_Pos)                 /*!< SYS_T::AWFRST: AWF0RST Mask            */

#define SYS_BPWMRST_BPWM0RST_Pos         (0)                                               /*!< SYS_T::BPWMRST: BPWM0RST Position      */
#define SYS_BPWMRST_BPWM0RST_Msk         (0x1ul << SYS_BPWMRST_BPWM0RST_Pos)               /*!< SYS_T::BPWMRST: BPWM0RST Mask          */

#define SYS_BPWMRST_BPWM1RST_Pos         (1)                                               /*!< SYS_T::BPWMRST: BPWM1RST Position      */
#define SYS_BPWMRST_BPWM1RST_Msk         (0x1ul << SYS_BPWMRST_BPWM1RST_Pos)               /*!< SYS_T::BPWMRST: BPWM1RST Mask          */

#define SYS_CANFDRST_CANFD0RST_Pos       (0)                                               /*!< SYS_T::CANFDRST: CANFD0RST Position    */
#define SYS_CANFDRST_CANFD0RST_Msk       (0x1ul << SYS_CANFDRST_CANFD0RST_Pos)             /*!< SYS_T::CANFDRST: CANFD0RST Mask        */

#define SYS_CANFDRST_CANFD1RST_Pos       (1)                                               /*!< SYS_T::CANFDRST: CANFD1RST Position    */
#define SYS_CANFDRST_CANFD1RST_Msk       (0x1ul << SYS_CANFDRST_CANFD1RST_Pos)             /*!< SYS_T::CANFDRST: CANFD1RST Mask        */

#define SYS_CCAPRST_CCAP0RST_Pos         (0)                                               /*!< SYS_T::CCAPRST: CCAP0RST Position      */
#define SYS_CCAPRST_CCAP0RST_Msk         (0x1ul << SYS_CCAPRST_CCAP0RST_Pos)               /*!< SYS_T::CCAPRST: CCAP0RST Mask          */

#define SYS_CRCRST_CRC0RST_Pos           (0)                                               /*!< SYS_T::CRCRST: CRC0RST Position        */
#define SYS_CRCRST_CRC0RST_Msk           (0x1ul << SYS_CRCRST_CRC0RST_Pos)                 /*!< SYS_T::CRCRST: CRC0RST Mask            */

#define SYS_CRYPTORST_CRYPTO0RST_Pos     (0)                                               /*!< SYS_T::CRYPTORST: CRYPTO0RST Position  */
#define SYS_CRYPTORST_CRYPTO0RST_Msk     (0x1ul << SYS_CRYPTORST_CRYPTO0RST_Pos)           /*!< SYS_T::CRYPTORST: CRYPTO0RST Mask      */

#define SYS_DACRST_DAC01RST_Pos          (0)                                               /*!< SYS_T::DACRST: DAC01RST Position       */
#define SYS_DACRST_DAC01RST_Msk          (0x1ul << SYS_DACRST_DAC01RST_Pos)                /*!< SYS_T::DACRST: DAC01RST Mask           */

#define SYS_DMICRST_DMIC0RST_Pos         (0)                                               /*!< SYS_T::DMICRST: DMIC0RST Position      */
#define SYS_DMICRST_DMIC0RST_Msk         (0x1ul << SYS_DMICRST_DMIC0RST_Pos)               /*!< SYS_T::DMICRST: DMIC0RST Mask          */

#define SYS_EADCRST_EADC0RST_Pos         (0)                                               /*!< SYS_T::EADCRST: EADC0RST Position      */
#define SYS_EADCRST_EADC0RST_Msk         (0x1ul << SYS_EADCRST_EADC0RST_Pos)               /*!< SYS_T::EADCRST: EADC0RST Mask          */

#define SYS_EBIRST_EBI0RST_Pos           (0)                                               /*!< SYS_T::EBIRST: EBI0RST Position        */
#define SYS_EBIRST_EBI0RST_Msk           (0x1ul << SYS_EBIRST_EBI0RST_Pos)                 /*!< SYS_T::EBIRST: EBI0RST Mask            */

#define SYS_ECAPRST_ECAP0RST_Pos         (0)                                               /*!< SYS_T::ECAPRST: ECAP0RST Position      */
#define SYS_ECAPRST_ECAP0RST_Msk         (0x1ul << SYS_ECAPRST_ECAP0RST_Pos)               /*!< SYS_T::ECAPRST: ECAP0RST Mask          */

#define SYS_ECAPRST_ECAP1RST_Pos         (1)                                               /*!< SYS_T::ECAPRST: ECAP1RST Position      */
#define SYS_ECAPRST_ECAP1RST_Msk         (0x1ul << SYS_ECAPRST_ECAP1RST_Pos)               /*!< SYS_T::ECAPRST: ECAP1RST Mask          */

#define SYS_ECAPRST_ECAP2RST_Pos         (2)                                               /*!< SYS_T::ECAPRST: ECAP2RST Position      */
#define SYS_ECAPRST_ECAP2RST_Msk         (0x1ul << SYS_ECAPRST_ECAP2RST_Pos)               /*!< SYS_T::ECAPRST: ECAP2RST Mask          */

#define SYS_ECAPRST_ECAP3RST_Pos         (3)                                               /*!< SYS_T::ECAPRST: ECAP3RST Position      */
#define SYS_ECAPRST_ECAP3RST_Msk         (0x1ul << SYS_ECAPRST_ECAP3RST_Pos)               /*!< SYS_T::ECAPRST: ECAP3RST Mask          */

#define SYS_EMACRST_EMAC0RST_Pos         (0)                                               /*!< SYS_T::EMACRST: EMAC0RST Position      */
#define SYS_EMACRST_EMAC0RST_Msk         (0x1ul << SYS_EMACRST_EMAC0RST_Pos)               /*!< SYS_T::EMACRST: EMAC0RST Mask          */

#define SYS_EPWMRST_EPWM0RST_Pos         (0)                                               /*!< SYS_T::EPWMRST: EPWM0RST Position      */
#define SYS_EPWMRST_EPWM0RST_Msk         (0x1ul << SYS_EPWMRST_EPWM0RST_Pos)               /*!< SYS_T::EPWMRST: EPWM0RST Mask          */

#define SYS_EPWMRST_EPWM1RST_Pos         (1)                                               /*!< SYS_T::EPWMRST: EPWM1RST Position      */
#define SYS_EPWMRST_EPWM1RST_Msk         (0x1ul << SYS_EPWMRST_EPWM1RST_Pos)               /*!< SYS_T::EPWMRST: EPWM1RST Mask          */

#define SYS_EQEIRST_EQEI0RST_Pos         (0)                                               /*!< SYS_T::EQEIRST: EQEI0RST Position      */
#define SYS_EQEIRST_EQEI0RST_Msk         (0x1ul << SYS_EQEIRST_EQEI0RST_Pos)               /*!< SYS_T::EQEIRST: EQEI0RST Mask          */

#define SYS_EQEIRST_EQEI1RST_Pos         (1)                                               /*!< SYS_T::EQEIRST: EQEI1RST Position      */
#define SYS_EQEIRST_EQEI1RST_Msk         (0x1ul << SYS_EQEIRST_EQEI1RST_Pos)               /*!< SYS_T::EQEIRST: EQEI1RST Mask          */

#define SYS_EQEIRST_EQEI2RST_Pos         (2)                                               /*!< SYS_T::EQEIRST: EQEI2RST Position      */
#define SYS_EQEIRST_EQEI2RST_Msk         (0x1ul << SYS_EQEIRST_EQEI2RST_Pos)               /*!< SYS_T::EQEIRST: EQEI2RST Mask          */

#define SYS_EQEIRST_EQEI3RST_Pos         (3)                                               /*!< SYS_T::EQEIRST: EQEI3RST Position      */
#define SYS_EQEIRST_EQEI3RST_Msk         (0x1ul << SYS_EQEIRST_EQEI3RST_Pos)               /*!< SYS_T::EQEIRST: EQEI3RST Mask          */

#define SYS_FMCRST_FMC0RST_Pos           (0)                                               /*!< SYS_T::FMCRST: FMC0RST Position        */
#define SYS_FMCRST_FMC0RST_Msk           (0x1ul << SYS_FMCRST_FMC0RST_Pos)                 /*!< SYS_T::FMCRST: FMC0RST Mask            */

#define SYS_GDMARST_GDMA0RST_Pos         (0)                                               /*!< SYS_T::GDMARST: GDMA0RST Position      */
#define SYS_GDMARST_GDMA0RST_Msk         (0x1ul << SYS_GDMARST_GDMA0RST_Pos)               /*!< SYS_T::GDMARST: GDMA0RST Mask          */

#define SYS_GPIORST_GPIO0RST_Pos         (0)                                               /*!< SYS_T::GPIORST: GPIO0RST Position      */
#define SYS_GPIORST_GPIO0RST_Msk         (0x1ul << SYS_GPIORST_GPIO0RST_Pos)               /*!< SYS_T::GPIORST: GPIO0RST Mask          */

#define SYS_HSOTGRST_HSOTG0RST_Pos       (0)                                               /*!< SYS_T::HSOTGRST: HSOTG0RST Position    */
#define SYS_HSOTGRST_HSOTG0RST_Msk       (0x1ul << SYS_HSOTGRST_HSOTG0RST_Pos)             /*!< SYS_T::HSOTGRST: HSOTG0RST Mask        */

#define SYS_HSUSBDRST_HSUSBD0RST_Pos     (0)                                               /*!< SYS_T::HSUSBDRST: HSUSBD0RST Position  */
#define SYS_HSUSBDRST_HSUSBD0RST_Msk     (0x1ul << SYS_HSUSBDRST_HSUSBD0RST_Pos)           /*!< SYS_T::HSUSBDRST: HSUSBD0RST Mask      */

#define SYS_HSUSBHRST_HSUSBH0RST_Pos     (0)                                               /*!< SYS_T::HSUSBHRST: HSUSBH0RST Position  */
#define SYS_HSUSBHRST_HSUSBH0RST_Msk     (0x1ul << SYS_HSUSBHRST_HSUSBH0RST_Pos)           /*!< SYS_T::HSUSBHRST: HSUSBH0RST Mask      */

#define SYS_I2CRST_I2C0RST_Pos           (0)                                               /*!< SYS_T::I2CRST: I2C0RST Position        */
#define SYS_I2CRST_I2C0RST_Msk           (0x1ul << SYS_I2CRST_I2C0RST_Pos)                 /*!< SYS_T::I2CRST: I2C0RST Mask            */

#define SYS_I2CRST_I2C1RST_Pos           (1)                                               /*!< SYS_T::I2CRST: I2C1RST Position        */
#define SYS_I2CRST_I2C1RST_Msk           (0x1ul << SYS_I2CRST_I2C1RST_Pos)                 /*!< SYS_T::I2CRST: I2C1RST Mask            */

#define SYS_I2CRST_I2C2RST_Pos           (2)                                               /*!< SYS_T::I2CRST: I2C2RST Position        */
#define SYS_I2CRST_I2C2RST_Msk           (0x1ul << SYS_I2CRST_I2C2RST_Pos)                 /*!< SYS_T::I2CRST: I2C2RST Mask            */

#define SYS_I2CRST_I2C3RST_Pos           (3)                                               /*!< SYS_T::I2CRST: I2C3RST Position        */
#define SYS_I2CRST_I2C3RST_Msk           (0x1ul << SYS_I2CRST_I2C3RST_Pos)                 /*!< SYS_T::I2CRST: I2C3RST Mask            */

#define SYS_I2SRST_I2S0RST_Pos           (0)                                               /*!< SYS_T::I2SRST: I2S0RST Position        */
#define SYS_I2SRST_I2S0RST_Msk           (0x1ul << SYS_I2SRST_I2S0RST_Pos)                 /*!< SYS_T::I2SRST: I2S0RST Mask            */

#define SYS_I2SRST_I2S1RST_Pos           (1)                                               /*!< SYS_T::I2SRST: I2S1RST Position        */
#define SYS_I2SRST_I2S1RST_Msk           (0x1ul << SYS_I2SRST_I2S1RST_Pos)                 /*!< SYS_T::I2SRST: I2S1RST Mask            */

#define SYS_I3CRST_I3C0RST_Pos           (0)                                               /*!< SYS_T::I3CRST: I3C0RST Position        */
#define SYS_I3CRST_I3C0RST_Msk           (0x1ul << SYS_I3CRST_I3C0RST_Pos)                 /*!< SYS_T::I3CRST: I3C0RST Mask            */

#define SYS_KDFRST_KDF0RST_Pos           (0)                                               /*!< SYS_T::KDFRST: KDF0RST Position        */
#define SYS_KDFRST_KDF0RST_Msk           (0x1ul << SYS_KDFRST_KDF0RST_Pos)                 /*!< SYS_T::KDFRST: KDF0RST Mask            */

#define SYS_KPIRST_KPI0RST_Pos           (0)                                               /*!< SYS_T::KPIRST: KPI0RST Position        */
#define SYS_KPIRST_KPI0RST_Msk           (0x1ul << SYS_KPIRST_KPI0RST_Pos)                 /*!< SYS_T::KPIRST: KPI0RST Mask            */

#define SYS_KSRST_KS0RST_Pos             (0)                                               /*!< SYS_T::KSRST: KS0RST Position          */
#define SYS_KSRST_KS0RST_Msk             (0x1ul << SYS_KSRST_KS0RST_Pos)                   /*!< SYS_T::KSRST: KS0RST Mask              */

#define SYS_LPADCRST_LPADC0RST_Pos       (0)                                               /*!< SYS_T::LPADCRST: LPADC0RST Position    */
#define SYS_LPADCRST_LPADC0RST_Msk       (0x1ul << SYS_LPADCRST_LPADC0RST_Pos)             /*!< SYS_T::LPADCRST: LPADC0RST Mask        */

#define SYS_LPPDMARST_LPPDMA0RST_Pos     (0)                                               /*!< SYS_T::LPPDMARST: LPPDMA0RST Position  */
#define SYS_LPPDMARST_LPPDMA0RST_Msk     (0x1ul << SYS_LPPDMARST_LPPDMA0RST_Pos)           /*!< SYS_T::LPPDMARST: LPPDMA0RST Mask      */

#define SYS_LPGPIORST_LPGPIO0RST_Pos     (0)                                               /*!< SYS_T::LPGPIORST: LPGPIO0RST Position  */
#define SYS_LPGPIORST_LPGPIO0RST_Msk     (0x1ul << SYS_LPGPIORST_LPGPIO0RST_Pos)           /*!< SYS_T::LPGPIORST: LPGPIO0RST Mask      */

#define SYS_LPI2CRST_LPI2C0RST_Pos       (0)                                               /*!< SYS_T::LPI2CRST: LPI2C0RST Position    */
#define SYS_LPI2CRST_LPI2C0RST_Msk       (0x1ul << SYS_LPI2CRST_LPI2C0RST_Pos)             /*!< SYS_T::LPI2CRST: LPI2C0RST Mask        */

#define SYS_LPSPIRST_LPSPI0RST_Pos       (0)                                               /*!< SYS_T::LPSPIRST: LPSPI0RST Position    */
#define SYS_LPSPIRST_LPSPI0RST_Msk       (0x1ul << SYS_LPSPIRST_LPSPI0RST_Pos)             /*!< SYS_T::LPSPIRST: LPSPI0RST Mask        */

#define SYS_LPTMRRST_LPTMR0RST_Pos       (0)                                               /*!< SYS_T::LPTMRRST: LPTMR0RST Position    */
#define SYS_LPTMRRST_LPTMR0RST_Msk       (0x1ul << SYS_LPTMRRST_LPTMR0RST_Pos)             /*!< SYS_T::LPTMRRST: LPTMR0RST Mask        */

#define SYS_LPTMRRST_LPTMR1RST_Pos       (1)                                               /*!< SYS_T::LPTMRRST: LPTMR1RST Position    */
#define SYS_LPTMRRST_LPTMR1RST_Msk       (0x1ul << SYS_LPTMRRST_LPTMR1RST_Pos)             /*!< SYS_T::LPTMRRST: LPTMR1RST Mask        */

#define SYS_LPUARTRST_LPUART0RST_Pos     (0)                                               /*!< SYS_T::LPUARTRST: LPUART0RST Position  */
#define SYS_LPUARTRST_LPUART0RST_Msk     (0x1ul << SYS_LPUARTRST_LPUART0RST_Pos)           /*!< SYS_T::LPUARTRST: LPUART0RST Mask      */

#define SYS_OTFCRST_OTFC0RST_Pos         (0)                                               /*!< SYS_T::OTFCRST: OTFC0RST Position      */
#define SYS_OTFCRST_OTFC0RST_Msk         (0x1ul << SYS_OTFCRST_OTFC0RST_Pos)               /*!< SYS_T::OTFCRST: OTFC0RST Mask          */

#define SYS_OTGRST_OTG0RST_Pos           (0)                                               /*!< SYS_T::OTGRST: OTG0RST Position        */
#define SYS_OTGRST_OTG0RST_Msk           (0x1ul << SYS_OTGRST_OTG0RST_Pos)                 /*!< SYS_T::OTGRST: OTG0RST Mask            */

#define SYS_PDMARST_PDMA0RST_Pos         (0)                                               /*!< SYS_T::PDMARST: PDMA0RST Position      */
#define SYS_PDMARST_PDMA0RST_Msk         (0x1ul << SYS_PDMARST_PDMA0RST_Pos)               /*!< SYS_T::PDMARST: PDMA0RST Mask          */

#define SYS_PDMARST_PDMA1RST_Pos         (1)                                               /*!< SYS_T::PDMARST: PDMA1RST Position      */
#define SYS_PDMARST_PDMA1RST_Msk         (0x1ul << SYS_PDMARST_PDMA1RST_Pos)               /*!< SYS_T::PDMARST: PDMA1RST Mask          */

#define SYS_PSIORST_PSIO0RST_Pos         (0)                                               /*!< SYS_T::PSIORST: PSIO0RST Position      */
#define SYS_PSIORST_PSIO0RST_Msk         (0x1ul << SYS_PSIORST_PSIO0RST_Pos)               /*!< SYS_T::PSIORST: PSIO0RST Mask          */

#define SYS_QSPIRST_QSPI0RST_Pos         (0)                                               /*!< SYS_T::QSPIRST: QSPI0RST Position      */
#define SYS_QSPIRST_QSPI0RST_Msk         (0x1ul << SYS_QSPIRST_QSPI0RST_Pos)               /*!< SYS_T::QSPIRST: QSPI0RST Mask          */

#define SYS_QSPIRST_QSPI1RST_Pos         (1)                                               /*!< SYS_T::QSPIRST: QSPI1RST Position      */
#define SYS_QSPIRST_QSPI1RST_Msk         (0x1ul << SYS_QSPIRST_QSPI1RST_Pos)               /*!< SYS_T::QSPIRST: QSPI1RST Mask          */

#define SYS_RTCRST_RTC0RST_Pos           (0)                                               /*!< SYS_T::RTCRST: RTC0RST Position        */
#define SYS_RTCRST_RTC0RST_Msk           (0x1ul << SYS_RTCRST_RTC0RST_Pos)                 /*!< SYS_T::RTCRST: RTC0RST Mask            */

#define SYS_SCRST_SC0RST_Pos             (0)                                               /*!< SYS_T::SCRST: SC0RST Position          */
#define SYS_SCRST_SC0RST_Msk             (0x1ul << SYS_SCRST_SC0RST_Pos)                   /*!< SYS_T::SCRST: SC0RST Mask              */

#define SYS_SCRST_SC1RST_Pos             (1)                                               /*!< SYS_T::SCRST: SC1RST Position          */
#define SYS_SCRST_SC1RST_Msk             (0x1ul << SYS_SCRST_SC1RST_Pos)                   /*!< SYS_T::SCRST: SC1RST Mask              */

#define SYS_SCRST_SC2RST_Pos             (2)                                               /*!< SYS_T::SCRST: SC2RST Position          */
#define SYS_SCRST_SC2RST_Msk             (0x1ul << SYS_SCRST_SC2RST_Pos)                   /*!< SYS_T::SCRST: SC2RST Mask              */

#define SYS_SCURST_SCU0RST_Pos           (0)                                               /*!< SYS_T::SCURST: SCU0RST Position        */
#define SYS_SCURST_SCU0RST_Msk           (0x1ul << SYS_SCURST_SCU0RST_Pos)                 /*!< SYS_T::SCURST: SCU0RST Mask            */

#define SYS_SDHRST_SDH0RST_Pos           (0)                                               /*!< SYS_T::SDHRST: SDH0RST Position        */
#define SYS_SDHRST_SDH0RST_Msk           (0x1ul << SYS_SDHRST_SDH0RST_Pos)                 /*!< SYS_T::SDHRST: SDH0RST Mask            */

#define SYS_SDHRST_SDH1RST_Pos           (1)                                               /*!< SYS_T::SDHRST: SDH1RST Position        */
#define SYS_SDHRST_SDH1RST_Msk           (0x1ul << SYS_SDHRST_SDH1RST_Pos)                 /*!< SYS_T::SDHRST: SDH1RST Mask            */

#define SYS_SPIRST_SPI0RST_Pos           (0)                                               /*!< SYS_T::SPIRST: SPI0RST Position        */
#define SYS_SPIRST_SPI0RST_Msk           (0x1ul << SYS_SPIRST_SPI0RST_Pos)                 /*!< SYS_T::SPIRST: SPI0RST Mask            */

#define SYS_SPIRST_SPI1RST_Pos           (1)                                               /*!< SYS_T::SPIRST: SPI1RST Position        */
#define SYS_SPIRST_SPI1RST_Msk           (0x1ul << SYS_SPIRST_SPI1RST_Pos)                 /*!< SYS_T::SPIRST: SPI1RST Mask            */

#define SYS_SPIRST_SPI2RST_Pos           (2)                                               /*!< SYS_T::SPIRST: SPI2RST Position        */
#define SYS_SPIRST_SPI2RST_Msk           (0x1ul << SYS_SPIRST_SPI2RST_Pos)                 /*!< SYS_T::SPIRST: SPI2RST Mask            */

#define SYS_SPIRST_SPI3RST_Pos           (3)                                               /*!< SYS_T::SPIRST: SPI3RST Position        */
#define SYS_SPIRST_SPI3RST_Msk           (0x1ul << SYS_SPIRST_SPI3RST_Pos)                 /*!< SYS_T::SPIRST: SPI3RST Mask            */

#define SYS_SPIMRST_SPIM0RST_Pos         (0)                                               /*!< SYS_T::SPIMRST: SPIM0RST Position      */
#define SYS_SPIMRST_SPIM0RST_Msk         (0x1ul << SYS_SPIMRST_SPIM0RST_Pos)               /*!< SYS_T::SPIMRST: SPIM0RST Mask          */

#define SYS_TMRRST_TMR0RST_Pos           (0)                                               /*!< SYS_T::TMRRST: TMR0RST Position        */
#define SYS_TMRRST_TMR0RST_Msk           (0x1ul << SYS_TMRRST_TMR0RST_Pos)                 /*!< SYS_T::TMRRST: TMR0RST Mask            */

#define SYS_TMRRST_TMR1RST_Pos           (1)                                               /*!< SYS_T::TMRRST: TMR1RST Position        */
#define SYS_TMRRST_TMR1RST_Msk           (0x1ul << SYS_TMRRST_TMR1RST_Pos)                 /*!< SYS_T::TMRRST: TMR1RST Mask            */

#define SYS_TMRRST_TMR2RST_Pos           (2)                                               /*!< SYS_T::TMRRST: TMR2RST Position        */
#define SYS_TMRRST_TMR2RST_Msk           (0x1ul << SYS_TMRRST_TMR2RST_Pos)                 /*!< SYS_T::TMRRST: TMR2RST Mask            */

#define SYS_TMRRST_TMR3RST_Pos           (3)                                               /*!< SYS_T::TMRRST: TMR3RST Position        */
#define SYS_TMRRST_TMR3RST_Msk           (0x1ul << SYS_TMRRST_TMR3RST_Pos)                 /*!< SYS_T::TMRRST: TMR3RST Mask            */

#define SYS_TRNGRST_TRNG0RST_Pos         (0)                                               /*!< SYS_T::TRNGRST: TRNG0RST Position      */
#define SYS_TRNGRST_TRNG0RST_Msk         (0x1ul << SYS_TRNGRST_TRNG0RST_Pos)               /*!< SYS_T::TRNGRST: TRNG0RST Mask          */

#define SYS_TTMRRST_TTMR0RST_Pos         (0)                                               /*!< SYS_T::TTMRRST: TTMR0RST Position      */
#define SYS_TTMRRST_TTMR0RST_Msk         (0x1ul << SYS_TTMRRST_TTMR0RST_Pos)               /*!< SYS_T::TTMRRST: TTMR0RST Mask          */

#define SYS_TTMRRST_TTMR1RST_Pos         (1)                                               /*!< SYS_T::TTMRRST: TTMR1RST Position      */
#define SYS_TTMRRST_TTMR1RST_Msk         (0x1ul << SYS_TTMRRST_TTMR1RST_Pos)               /*!< SYS_T::TTMRRST: TTMR1RST Mask          */

#define SYS_UARTRST_UART0RST_Pos         (0)                                               /*!< SYS_T::UARTRST: UART0RST Position      */
#define SYS_UARTRST_UART0RST_Msk         (0x1ul << SYS_UARTRST_UART0RST_Pos)               /*!< SYS_T::UARTRST: UART0RST Mask          */

#define SYS_UARTRST_UART1RST_Pos         (1)                                               /*!< SYS_T::UARTRST: UART1RST Position      */
#define SYS_UARTRST_UART1RST_Msk         (0x1ul << SYS_UARTRST_UART1RST_Pos)               /*!< SYS_T::UARTRST: UART1RST Mask          */

#define SYS_UARTRST_UART2RST_Pos         (2)                                               /*!< SYS_T::UARTRST: UART2RST Position      */
#define SYS_UARTRST_UART2RST_Msk         (0x1ul << SYS_UARTRST_UART2RST_Pos)               /*!< SYS_T::UARTRST: UART2RST Mask          */

#define SYS_UARTRST_UART3RST_Pos         (3)                                               /*!< SYS_T::UARTRST: UART3RST Position      */
#define SYS_UARTRST_UART3RST_Msk         (0x1ul << SYS_UARTRST_UART3RST_Pos)               /*!< SYS_T::UARTRST: UART3RST Mask          */

#define SYS_UARTRST_UART4RST_Pos         (4)                                               /*!< SYS_T::UARTRST: UART4RST Position      */
#define SYS_UARTRST_UART4RST_Msk         (0x1ul << SYS_UARTRST_UART4RST_Pos)               /*!< SYS_T::UARTRST: UART4RST Mask          */

#define SYS_UARTRST_UART5RST_Pos         (5)                                               /*!< SYS_T::UARTRST: UART5RST Position      */
#define SYS_UARTRST_UART5RST_Msk         (0x1ul << SYS_UARTRST_UART5RST_Pos)               /*!< SYS_T::UARTRST: UART5RST Mask          */

#define SYS_UARTRST_UART6RST_Pos         (6)                                               /*!< SYS_T::UARTRST: UART6RST Position      */
#define SYS_UARTRST_UART6RST_Msk         (0x1ul << SYS_UARTRST_UART6RST_Pos)               /*!< SYS_T::UARTRST: UART6RST Mask          */

#define SYS_UARTRST_UART7RST_Pos         (7)                                               /*!< SYS_T::UARTRST: UART7RST Position      */
#define SYS_UARTRST_UART7RST_Msk         (0x1ul << SYS_UARTRST_UART7RST_Pos)               /*!< SYS_T::UARTRST: UART7RST Mask          */

#define SYS_UARTRST_UART8RST_Pos         (8)                                               /*!< SYS_T::UARTRST: UART8RST Position      */
#define SYS_UARTRST_UART8RST_Msk         (0x1ul << SYS_UARTRST_UART8RST_Pos)               /*!< SYS_T::UARTRST: UART8RST Mask          */

#define SYS_UARTRST_UART9RST_Pos         (9)                                               /*!< SYS_T::UARTRST: UART9RST Position      */
#define SYS_UARTRST_UART9RST_Msk         (0x1ul << SYS_UARTRST_UART9RST_Pos)               /*!< SYS_T::UARTRST: UART9RST Mask          */

#define SYS_USBDRST_USBD0RST_Pos         (0)                                               /*!< SYS_T::USBDRST: USBD0RST Position      */
#define SYS_USBDRST_USBD0RST_Msk         (0x1ul << SYS_USBDRST_USBD0RST_Pos)               /*!< SYS_T::USBDRST: USBD0RST Mask          */

#define SYS_USBHRST_USBH0RST_Pos         (0)                                               /*!< SYS_T::USBHRST: USBH0RST Position      */
#define SYS_USBHRST_USBH0RST_Msk         (0x1ul << SYS_USBHRST_USBH0RST_Pos)               /*!< SYS_T::USBHRST: USBH0RST Mask          */

#define SYS_USCIRST_USCI0RST_Pos         (0)                                               /*!< SYS_T::USCIRST: USCI0RST Position      */
#define SYS_USCIRST_USCI0RST_Msk         (0x1ul << SYS_USCIRST_USCI0RST_Pos)               /*!< SYS_T::USCIRST: USCI0RST Mask          */

#define SYS_UTCPDRST_UTCPD0RST_Pos       (0)                                               /*!< SYS_T::UTCPDRST: UTCPD0RST Position    */
#define SYS_UTCPDRST_UTCPD0RST_Msk       (0x1ul << SYS_UTCPDRST_UTCPD0RST_Pos)             /*!< SYS_T::UTCPDRST: UTCPD0RST Mask        */

#define SYS_WWDTRST_WWDT0RST_Pos         (0)                                               /*!< SYS_T::WWDTRST: WWDT0RST Position      */
#define SYS_WWDTRST_WWDT0RST_Msk         (0x1ul << SYS_WWDTRST_WWDT0RST_Pos)               /*!< SYS_T::WWDTRST: WWDT0RST Mask          */

#define SYS_WWDTRST_WWDT1RST_Pos         (1)                                               /*!< SYS_T::WWDTRST: WWDT1RST Position      */
#define SYS_WWDTRST_WWDT1RST_Msk         (0x1ul << SYS_WWDTRST_WWDT1RST_Pos)               /*!< SYS_T::WWDTRST: WWDT1RST Mask          */

#define SYS_GPA_MFP0_PA0MFP_Pos          (0)                                               /*!< SYS_T::GPA_MFP0: PA0MFP Position       */
#define SYS_GPA_MFP0_PA0MFP_Msk          (0x1ful << SYS_GPA_MFP0_PA0MFP_Pos)               /*!< SYS_T::GPA_MFP0: PA0MFP Mask           */

#define SYS_GPA_MFP0_PA1MFP_Pos          (8)                                               /*!< SYS_T::GPA_MFP0: PA1MFP Position       */
#define SYS_GPA_MFP0_PA1MFP_Msk          (0x1ful << SYS_GPA_MFP0_PA1MFP_Pos)               /*!< SYS_T::GPA_MFP0: PA1MFP Mask           */

#define SYS_GPA_MFP0_PA2MFP_Pos          (16)                                              /*!< SYS_T::GPA_MFP0: PA2MFP Position       */
#define SYS_GPA_MFP0_PA2MFP_Msk          (0x1ful << SYS_GPA_MFP0_PA2MFP_Pos)               /*!< SYS_T::GPA_MFP0: PA2MFP Mask           */

#define SYS_GPA_MFP0_PA3MFP_Pos          (24)                                              /*!< SYS_T::GPA_MFP0: PA3MFP Position       */
#define SYS_GPA_MFP0_PA3MFP_Msk          (0x1ful << SYS_GPA_MFP0_PA3MFP_Pos)               /*!< SYS_T::GPA_MFP0: PA3MFP Mask           */

#define SYS_GPA_MFP1_PA4MFP_Pos          (0)                                               /*!< SYS_T::GPA_MFP1: PA4MFP Position       */
#define SYS_GPA_MFP1_PA4MFP_Msk          (0x1ful << SYS_GPA_MFP1_PA4MFP_Pos)               /*!< SYS_T::GPA_MFP1: PA4MFP Mask           */

#define SYS_GPA_MFP1_PA5MFP_Pos          (8)                                               /*!< SYS_T::GPA_MFP1: PA5MFP Position       */
#define SYS_GPA_MFP1_PA5MFP_Msk          (0x1ful << SYS_GPA_MFP1_PA5MFP_Pos)               /*!< SYS_T::GPA_MFP1: PA5MFP Mask           */

#define SYS_GPA_MFP1_PA6MFP_Pos          (16)                                              /*!< SYS_T::GPA_MFP1: PA6MFP Position       */
#define SYS_GPA_MFP1_PA6MFP_Msk          (0x1ful << SYS_GPA_MFP1_PA6MFP_Pos)               /*!< SYS_T::GPA_MFP1: PA6MFP Mask           */

#define SYS_GPA_MFP1_PA7MFP_Pos          (24)                                              /*!< SYS_T::GPA_MFP1: PA7MFP Position       */
#define SYS_GPA_MFP1_PA7MFP_Msk          (0x1ful << SYS_GPA_MFP1_PA7MFP_Pos)               /*!< SYS_T::GPA_MFP1: PA7MFP Mask           */

#define SYS_GPA_MFP2_PA8MFP_Pos          (0)                                               /*!< SYS_T::GPA_MFP2: PA8MFP Position       */
#define SYS_GPA_MFP2_PA8MFP_Msk          (0x1ful << SYS_GPA_MFP2_PA8MFP_Pos)               /*!< SYS_T::GPA_MFP2: PA8MFP Mask           */

#define SYS_GPA_MFP2_PA9MFP_Pos          (8)                                               /*!< SYS_T::GPA_MFP2: PA9MFP Position       */
#define SYS_GPA_MFP2_PA9MFP_Msk          (0x1ful << SYS_GPA_MFP2_PA9MFP_Pos)               /*!< SYS_T::GPA_MFP2: PA9MFP Mask           */

#define SYS_GPA_MFP2_PA10MFP_Pos         (16)                                              /*!< SYS_T::GPA_MFP2: PA10MFP Position      */
#define SYS_GPA_MFP2_PA10MFP_Msk         (0x1ful << SYS_GPA_MFP2_PA10MFP_Pos)              /*!< SYS_T::GPA_MFP2: PA10MFP Mask          */

#define SYS_GPA_MFP2_PA11MFP_Pos         (24)                                              /*!< SYS_T::GPA_MFP2: PA11MFP Position      */
#define SYS_GPA_MFP2_PA11MFP_Msk         (0x1ful << SYS_GPA_MFP2_PA11MFP_Pos)              /*!< SYS_T::GPA_MFP2: PA11MFP Mask          */

#define SYS_GPA_MFP3_PA12MFP_Pos         (0)                                               /*!< SYS_T::GPA_MFP3: PA12MFP Position      */
#define SYS_GPA_MFP3_PA12MFP_Msk         (0x1ful << SYS_GPA_MFP3_PA12MFP_Pos)              /*!< SYS_T::GPA_MFP3: PA12MFP Mask          */

#define SYS_GPA_MFP3_PA13MFP_Pos         (8)                                               /*!< SYS_T::GPA_MFP3: PA13MFP Position      */
#define SYS_GPA_MFP3_PA13MFP_Msk         (0x1ful << SYS_GPA_MFP3_PA13MFP_Pos)              /*!< SYS_T::GPA_MFP3: PA13MFP Mask          */

#define SYS_GPA_MFP3_PA14MFP_Pos         (16)                                              /*!< SYS_T::GPA_MFP3: PA14MFP Position      */
#define SYS_GPA_MFP3_PA14MFP_Msk         (0x1ful << SYS_GPA_MFP3_PA14MFP_Pos)              /*!< SYS_T::GPA_MFP3: PA14MFP Mask          */

#define SYS_GPA_MFP3_PA15MFP_Pos         (24)                                              /*!< SYS_T::GPA_MFP3: PA15MFP Position      */
#define SYS_GPA_MFP3_PA15MFP_Msk         (0x1ful << SYS_GPA_MFP3_PA15MFP_Pos)              /*!< SYS_T::GPA_MFP3: PA15MFP Mask          */

#define SYS_GPB_MFP0_PB0MFP_Pos          (0)                                               /*!< SYS_T::GPB_MFP0: PB0MFP Position       */
#define SYS_GPB_MFP0_PB0MFP_Msk          (0x1ful << SYS_GPB_MFP0_PB0MFP_Pos)               /*!< SYS_T::GPB_MFP0: PB0MFP Mask           */

#define SYS_GPB_MFP0_PB1MFP_Pos          (8)                                               /*!< SYS_T::GPB_MFP0: PB1MFP Position       */
#define SYS_GPB_MFP0_PB1MFP_Msk          (0x1ful << SYS_GPB_MFP0_PB1MFP_Pos)               /*!< SYS_T::GPB_MFP0: PB1MFP Mask           */

#define SYS_GPB_MFP0_PB2MFP_Pos          (16)                                              /*!< SYS_T::GPB_MFP0: PB2MFP Position       */
#define SYS_GPB_MFP0_PB2MFP_Msk          (0x1ful << SYS_GPB_MFP0_PB2MFP_Pos)               /*!< SYS_T::GPB_MFP0: PB2MFP Mask           */

#define SYS_GPB_MFP0_PB3MFP_Pos          (24)                                              /*!< SYS_T::GPB_MFP0: PB3MFP Position       */
#define SYS_GPB_MFP0_PB3MFP_Msk          (0x1ful << SYS_GPB_MFP0_PB3MFP_Pos)               /*!< SYS_T::GPB_MFP0: PB3MFP Mask           */

#define SYS_GPB_MFP1_PB4MFP_Pos          (0)                                               /*!< SYS_T::GPB_MFP1: PB4MFP Position       */
#define SYS_GPB_MFP1_PB4MFP_Msk          (0x1ful << SYS_GPB_MFP1_PB4MFP_Pos)               /*!< SYS_T::GPB_MFP1: PB4MFP Mask           */

#define SYS_GPB_MFP1_PB5MFP_Pos          (8)                                               /*!< SYS_T::GPB_MFP1: PB5MFP Position       */
#define SYS_GPB_MFP1_PB5MFP_Msk          (0x1ful << SYS_GPB_MFP1_PB5MFP_Pos)               /*!< SYS_T::GPB_MFP1: PB5MFP Mask           */

#define SYS_GPB_MFP1_PB6MFP_Pos          (16)                                              /*!< SYS_T::GPB_MFP1: PB6MFP Position       */
#define SYS_GPB_MFP1_PB6MFP_Msk          (0x1ful << SYS_GPB_MFP1_PB6MFP_Pos)               /*!< SYS_T::GPB_MFP1: PB6MFP Mask           */

#define SYS_GPB_MFP1_PB7MFP_Pos          (24)                                              /*!< SYS_T::GPB_MFP1: PB7MFP Position       */
#define SYS_GPB_MFP1_PB7MFP_Msk          (0x1ful << SYS_GPB_MFP1_PB7MFP_Pos)               /*!< SYS_T::GPB_MFP1: PB7MFP Mask           */

#define SYS_GPB_MFP2_PB8MFP_Pos          (0)                                               /*!< SYS_T::GPB_MFP2: PB8MFP Position       */
#define SYS_GPB_MFP2_PB8MFP_Msk          (0x1ful << SYS_GPB_MFP2_PB8MFP_Pos)               /*!< SYS_T::GPB_MFP2: PB8MFP Mask           */

#define SYS_GPB_MFP2_PB9MFP_Pos          (8)                                               /*!< SYS_T::GPB_MFP2: PB9MFP Position       */
#define SYS_GPB_MFP2_PB9MFP_Msk          (0x1ful << SYS_GPB_MFP2_PB9MFP_Pos)               /*!< SYS_T::GPB_MFP2: PB9MFP Mask           */

#define SYS_GPB_MFP2_PB10MFP_Pos         (16)                                              /*!< SYS_T::GPB_MFP2: PB10MFP Position      */
#define SYS_GPB_MFP2_PB10MFP_Msk         (0x1ful << SYS_GPB_MFP2_PB10MFP_Pos)              /*!< SYS_T::GPB_MFP2: PB10MFP Mask          */

#define SYS_GPB_MFP2_PB11MFP_Pos         (24)                                              /*!< SYS_T::GPB_MFP2: PB11MFP Position      */
#define SYS_GPB_MFP2_PB11MFP_Msk         (0x1ful << SYS_GPB_MFP2_PB11MFP_Pos)              /*!< SYS_T::GPB_MFP2: PB11MFP Mask          */

#define SYS_GPB_MFP3_PB12MFP_Pos         (0)                                               /*!< SYS_T::GPB_MFP3: PB12MFP Position      */
#define SYS_GPB_MFP3_PB12MFP_Msk         (0x1ful << SYS_GPB_MFP3_PB12MFP_Pos)              /*!< SYS_T::GPB_MFP3: PB12MFP Mask          */

#define SYS_GPB_MFP3_PB13MFP_Pos         (8)                                               /*!< SYS_T::GPB_MFP3: PB13MFP Position      */
#define SYS_GPB_MFP3_PB13MFP_Msk         (0x1ful << SYS_GPB_MFP3_PB13MFP_Pos)              /*!< SYS_T::GPB_MFP3: PB13MFP Mask          */

#define SYS_GPB_MFP3_PB14MFP_Pos         (16)                                              /*!< SYS_T::GPB_MFP3: PB14MFP Position      */
#define SYS_GPB_MFP3_PB14MFP_Msk         (0x1ful << SYS_GPB_MFP3_PB14MFP_Pos)              /*!< SYS_T::GPB_MFP3: PB14MFP Mask          */

#define SYS_GPB_MFP3_PB15MFP_Pos         (24)                                              /*!< SYS_T::GPB_MFP3: PB15MFP Position      */
#define SYS_GPB_MFP3_PB15MFP_Msk         (0x1ful << SYS_GPB_MFP3_PB15MFP_Pos)              /*!< SYS_T::GPB_MFP3: PB15MFP Mask          */

#define SYS_GPC_MFP0_PC0MFP_Pos          (0)                                               /*!< SYS_T::GPC_MFP0: PC0MFP Position       */
#define SYS_GPC_MFP0_PC0MFP_Msk          (0x1ful << SYS_GPC_MFP0_PC0MFP_Pos)               /*!< SYS_T::GPC_MFP0: PC0MFP Mask           */

#define SYS_GPC_MFP0_PC1MFP_Pos          (8)                                               /*!< SYS_T::GPC_MFP0: PC1MFP Position       */
#define SYS_GPC_MFP0_PC1MFP_Msk          (0x1ful << SYS_GPC_MFP0_PC1MFP_Pos)               /*!< SYS_T::GPC_MFP0: PC1MFP Mask           */

#define SYS_GPC_MFP0_PC2MFP_Pos          (16)                                              /*!< SYS_T::GPC_MFP0: PC2MFP Position       */
#define SYS_GPC_MFP0_PC2MFP_Msk          (0x1ful << SYS_GPC_MFP0_PC2MFP_Pos)               /*!< SYS_T::GPC_MFP0: PC2MFP Mask           */

#define SYS_GPC_MFP0_PC3MFP_Pos          (24)                                              /*!< SYS_T::GPC_MFP0: PC3MFP Position       */
#define SYS_GPC_MFP0_PC3MFP_Msk          (0x1ful << SYS_GPC_MFP0_PC3MFP_Pos)               /*!< SYS_T::GPC_MFP0: PC3MFP Mask           */

#define SYS_GPC_MFP1_PC4MFP_Pos          (0)                                               /*!< SYS_T::GPC_MFP1: PC4MFP Position       */
#define SYS_GPC_MFP1_PC4MFP_Msk          (0x1ful << SYS_GPC_MFP1_PC4MFP_Pos)               /*!< SYS_T::GPC_MFP1: PC4MFP Mask           */

#define SYS_GPC_MFP1_PC5MFP_Pos          (8)                                               /*!< SYS_T::GPC_MFP1: PC5MFP Position       */
#define SYS_GPC_MFP1_PC5MFP_Msk          (0x1ful << SYS_GPC_MFP1_PC5MFP_Pos)               /*!< SYS_T::GPC_MFP1: PC5MFP Mask           */

#define SYS_GPC_MFP1_PC6MFP_Pos          (16)                                              /*!< SYS_T::GPC_MFP1: PC6MFP Position       */
#define SYS_GPC_MFP1_PC6MFP_Msk          (0x1ful << SYS_GPC_MFP1_PC6MFP_Pos)               /*!< SYS_T::GPC_MFP1: PC6MFP Mask           */

#define SYS_GPC_MFP1_PC7MFP_Pos          (24)                                              /*!< SYS_T::GPC_MFP1: PC7MFP Position       */
#define SYS_GPC_MFP1_PC7MFP_Msk          (0x1ful << SYS_GPC_MFP1_PC7MFP_Pos)               /*!< SYS_T::GPC_MFP1: PC7MFP Mask           */

#define SYS_GPC_MFP2_PC8MFP_Pos          (0)                                               /*!< SYS_T::GPC_MFP2: PC8MFP Position       */
#define SYS_GPC_MFP2_PC8MFP_Msk          (0x1ful << SYS_GPC_MFP2_PC8MFP_Pos)               /*!< SYS_T::GPC_MFP2: PC8MFP Mask           */

#define SYS_GPC_MFP2_PC9MFP_Pos          (8)                                               /*!< SYS_T::GPC_MFP2: PC9MFP Position       */
#define SYS_GPC_MFP2_PC9MFP_Msk          (0x1ful << SYS_GPC_MFP2_PC9MFP_Pos)               /*!< SYS_T::GPC_MFP2: PC9MFP Mask           */

#define SYS_GPC_MFP2_PC10MFP_Pos         (16)                                              /*!< SYS_T::GPC_MFP2: PC10MFP Position      */
#define SYS_GPC_MFP2_PC10MFP_Msk         (0x1ful << SYS_GPC_MFP2_PC10MFP_Pos)              /*!< SYS_T::GPC_MFP2: PC10MFP Mask          */

#define SYS_GPC_MFP2_PC11MFP_Pos         (24)                                              /*!< SYS_T::GPC_MFP2: PC11MFP Position      */
#define SYS_GPC_MFP2_PC11MFP_Msk         (0x1ful << SYS_GPC_MFP2_PC11MFP_Pos)              /*!< SYS_T::GPC_MFP2: PC11MFP Mask          */

#define SYS_GPC_MFP3_PC12MFP_Pos         (0)                                               /*!< SYS_T::GPC_MFP3: PC12MFP Position      */
#define SYS_GPC_MFP3_PC12MFP_Msk         (0x1ful << SYS_GPC_MFP3_PC12MFP_Pos)              /*!< SYS_T::GPC_MFP3: PC12MFP Mask          */

#define SYS_GPC_MFP3_PC13MFP_Pos         (8)                                               /*!< SYS_T::GPC_MFP3: PC13MFP Position      */
#define SYS_GPC_MFP3_PC13MFP_Msk         (0x1ful << SYS_GPC_MFP3_PC13MFP_Pos)              /*!< SYS_T::GPC_MFP3: PC13MFP Mask          */

#define SYS_GPC_MFP3_PC14MFP_Pos         (16)                                              /*!< SYS_T::GPC_MFP3: PC14MFP Position      */
#define SYS_GPC_MFP3_PC14MFP_Msk         (0x1ful << SYS_GPC_MFP3_PC14MFP_Pos)              /*!< SYS_T::GPC_MFP3: PC14MFP Mask          */

#define SYS_GPD_MFP0_PD0MFP_Pos          (0)                                               /*!< SYS_T::GPD_MFP0: PD0MFP Position       */
#define SYS_GPD_MFP0_PD0MFP_Msk          (0x1ful << SYS_GPD_MFP0_PD0MFP_Pos)               /*!< SYS_T::GPD_MFP0: PD0MFP Mask           */

#define SYS_GPD_MFP0_PD1MFP_Pos          (8)                                               /*!< SYS_T::GPD_MFP0: PD1MFP Position       */
#define SYS_GPD_MFP0_PD1MFP_Msk          (0x1ful << SYS_GPD_MFP0_PD1MFP_Pos)               /*!< SYS_T::GPD_MFP0: PD1MFP Mask           */

#define SYS_GPD_MFP0_PD2MFP_Pos          (16)                                              /*!< SYS_T::GPD_MFP0: PD2MFP Position       */
#define SYS_GPD_MFP0_PD2MFP_Msk          (0x1ful << SYS_GPD_MFP0_PD2MFP_Pos)               /*!< SYS_T::GPD_MFP0: PD2MFP Mask           */

#define SYS_GPD_MFP0_PD3MFP_Pos          (24)                                              /*!< SYS_T::GPD_MFP0: PD3MFP Position       */
#define SYS_GPD_MFP0_PD3MFP_Msk          (0x1ful << SYS_GPD_MFP0_PD3MFP_Pos)               /*!< SYS_T::GPD_MFP0: PD3MFP Mask           */

#define SYS_GPD_MFP1_PD4MFP_Pos          (0)                                               /*!< SYS_T::GPD_MFP1: PD4MFP Position       */
#define SYS_GPD_MFP1_PD4MFP_Msk          (0x1ful << SYS_GPD_MFP1_PD4MFP_Pos)               /*!< SYS_T::GPD_MFP1: PD4MFP Mask           */

#define SYS_GPD_MFP1_PD5MFP_Pos          (8)                                               /*!< SYS_T::GPD_MFP1: PD5MFP Position       */
#define SYS_GPD_MFP1_PD5MFP_Msk          (0x1ful << SYS_GPD_MFP1_PD5MFP_Pos)               /*!< SYS_T::GPD_MFP1: PD5MFP Mask           */

#define SYS_GPD_MFP1_PD6MFP_Pos          (16)                                              /*!< SYS_T::GPD_MFP1: PD6MFP Position       */
#define SYS_GPD_MFP1_PD6MFP_Msk          (0x1ful << SYS_GPD_MFP1_PD6MFP_Pos)               /*!< SYS_T::GPD_MFP1: PD6MFP Mask           */

#define SYS_GPD_MFP1_PD7MFP_Pos          (24)                                              /*!< SYS_T::GPD_MFP1: PD7MFP Position       */
#define SYS_GPD_MFP1_PD7MFP_Msk          (0x1ful << SYS_GPD_MFP1_PD7MFP_Pos)               /*!< SYS_T::GPD_MFP1: PD7MFP Mask           */

#define SYS_GPD_MFP2_PD8MFP_Pos          (0)                                               /*!< SYS_T::GPD_MFP2: PD8MFP Position       */
#define SYS_GPD_MFP2_PD8MFP_Msk          (0x1ful << SYS_GPD_MFP2_PD8MFP_Pos)               /*!< SYS_T::GPD_MFP2: PD8MFP Mask           */

#define SYS_GPD_MFP2_PD9MFP_Pos          (8)                                               /*!< SYS_T::GPD_MFP2: PD9MFP Position       */
#define SYS_GPD_MFP2_PD9MFP_Msk          (0x1ful << SYS_GPD_MFP2_PD9MFP_Pos)               /*!< SYS_T::GPD_MFP2: PD9MFP Mask           */

#define SYS_GPD_MFP2_PD10MFP_Pos         (16)                                              /*!< SYS_T::GPD_MFP2: PD10MFP Position      */
#define SYS_GPD_MFP2_PD10MFP_Msk         (0x1ful << SYS_GPD_MFP2_PD10MFP_Pos)              /*!< SYS_T::GPD_MFP2: PD10MFP Mask          */

#define SYS_GPD_MFP2_PD11MFP_Pos         (24)                                              /*!< SYS_T::GPD_MFP2: PD11MFP Position      */
#define SYS_GPD_MFP2_PD11MFP_Msk         (0x1ful << SYS_GPD_MFP2_PD11MFP_Pos)              /*!< SYS_T::GPD_MFP2: PD11MFP Mask          */

#define SYS_GPD_MFP3_PD12MFP_Pos         (0)                                               /*!< SYS_T::GPD_MFP3: PD12MFP Position      */
#define SYS_GPD_MFP3_PD12MFP_Msk         (0x1ful << SYS_GPD_MFP3_PD12MFP_Pos)              /*!< SYS_T::GPD_MFP3: PD12MFP Mask          */

#define SYS_GPD_MFP3_PD13MFP_Pos         (8)                                               /*!< SYS_T::GPD_MFP3: PD13MFP Position      */
#define SYS_GPD_MFP3_PD13MFP_Msk         (0x1ful << SYS_GPD_MFP3_PD13MFP_Pos)              /*!< SYS_T::GPD_MFP3: PD13MFP Mask          */

#define SYS_GPD_MFP3_PD14MFP_Pos         (16)                                              /*!< SYS_T::GPD_MFP3: PD14MFP Position      */
#define SYS_GPD_MFP3_PD14MFP_Msk         (0x1ful << SYS_GPD_MFP3_PD14MFP_Pos)              /*!< SYS_T::GPD_MFP3: PD14MFP Mask          */

#define SYS_GPE_MFP0_PE0MFP_Pos          (0)                                               /*!< SYS_T::GPE_MFP0: PE0MFP Position       */
#define SYS_GPE_MFP0_PE0MFP_Msk          (0x1ful << SYS_GPE_MFP0_PE0MFP_Pos)               /*!< SYS_T::GPE_MFP0: PE0MFP Mask           */

#define SYS_GPE_MFP0_PE1MFP_Pos          (8)                                               /*!< SYS_T::GPE_MFP0: PE1MFP Position       */
#define SYS_GPE_MFP0_PE1MFP_Msk          (0x1ful << SYS_GPE_MFP0_PE1MFP_Pos)               /*!< SYS_T::GPE_MFP0: PE1MFP Mask           */

#define SYS_GPE_MFP0_PE2MFP_Pos          (16)                                              /*!< SYS_T::GPE_MFP0: PE2MFP Position       */
#define SYS_GPE_MFP0_PE2MFP_Msk          (0x1ful << SYS_GPE_MFP0_PE2MFP_Pos)               /*!< SYS_T::GPE_MFP0: PE2MFP Mask           */

#define SYS_GPE_MFP0_PE3MFP_Pos          (24)                                              /*!< SYS_T::GPE_MFP0: PE3MFP Position       */
#define SYS_GPE_MFP0_PE3MFP_Msk          (0x1ful << SYS_GPE_MFP0_PE3MFP_Pos)               /*!< SYS_T::GPE_MFP0: PE3MFP Mask           */

#define SYS_GPE_MFP1_PE4MFP_Pos          (0)                                               /*!< SYS_T::GPE_MFP1: PE4MFP Position       */
#define SYS_GPE_MFP1_PE4MFP_Msk          (0x1ful << SYS_GPE_MFP1_PE4MFP_Pos)               /*!< SYS_T::GPE_MFP1: PE4MFP Mask           */

#define SYS_GPE_MFP1_PE5MFP_Pos          (8)                                               /*!< SYS_T::GPE_MFP1: PE5MFP Position       */
#define SYS_GPE_MFP1_PE5MFP_Msk          (0x1ful << SYS_GPE_MFP1_PE5MFP_Pos)               /*!< SYS_T::GPE_MFP1: PE5MFP Mask           */

#define SYS_GPE_MFP1_PE6MFP_Pos          (16)                                              /*!< SYS_T::GPE_MFP1: PE6MFP Position       */
#define SYS_GPE_MFP1_PE6MFP_Msk          (0x1ful << SYS_GPE_MFP1_PE6MFP_Pos)               /*!< SYS_T::GPE_MFP1: PE6MFP Mask           */

#define SYS_GPE_MFP1_PE7MFP_Pos          (24)                                              /*!< SYS_T::GPE_MFP1: PE7MFP Position       */
#define SYS_GPE_MFP1_PE7MFP_Msk          (0x1ful << SYS_GPE_MFP1_PE7MFP_Pos)               /*!< SYS_T::GPE_MFP1: PE7MFP Mask           */

#define SYS_GPE_MFP2_PE8MFP_Pos          (0)                                               /*!< SYS_T::GPE_MFP2: PE8MFP Position       */
#define SYS_GPE_MFP2_PE8MFP_Msk          (0x1ful << SYS_GPE_MFP2_PE8MFP_Pos)               /*!< SYS_T::GPE_MFP2: PE8MFP Mask           */

#define SYS_GPE_MFP2_PE9MFP_Pos          (8)                                               /*!< SYS_T::GPE_MFP2: PE9MFP Position       */
#define SYS_GPE_MFP2_PE9MFP_Msk          (0x1ful << SYS_GPE_MFP2_PE9MFP_Pos)               /*!< SYS_T::GPE_MFP2: PE9MFP Mask           */

#define SYS_GPE_MFP2_PE10MFP_Pos         (16)                                              /*!< SYS_T::GPE_MFP2: PE10MFP Position      */
#define SYS_GPE_MFP2_PE10MFP_Msk         (0x1ful << SYS_GPE_MFP2_PE10MFP_Pos)              /*!< SYS_T::GPE_MFP2: PE10MFP Mask          */

#define SYS_GPE_MFP2_PE11MFP_Pos         (24)                                              /*!< SYS_T::GPE_MFP2: PE11MFP Position      */
#define SYS_GPE_MFP2_PE11MFP_Msk         (0x1ful << SYS_GPE_MFP2_PE11MFP_Pos)              /*!< SYS_T::GPE_MFP2: PE11MFP Mask          */

#define SYS_GPE_MFP3_PE12MFP_Pos         (0)                                               /*!< SYS_T::GPE_MFP3: PE12MFP Position      */
#define SYS_GPE_MFP3_PE12MFP_Msk         (0x1ful << SYS_GPE_MFP3_PE12MFP_Pos)              /*!< SYS_T::GPE_MFP3: PE12MFP Mask          */

#define SYS_GPE_MFP3_PE13MFP_Pos         (8)                                               /*!< SYS_T::GPE_MFP3: PE13MFP Position      */
#define SYS_GPE_MFP3_PE13MFP_Msk         (0x1ful << SYS_GPE_MFP3_PE13MFP_Pos)              /*!< SYS_T::GPE_MFP3: PE13MFP Mask          */

#define SYS_GPE_MFP3_PE14MFP_Pos         (16)                                              /*!< SYS_T::GPE_MFP3: PE14MFP Position      */
#define SYS_GPE_MFP3_PE14MFP_Msk         (0x1ful << SYS_GPE_MFP3_PE14MFP_Pos)              /*!< SYS_T::GPE_MFP3: PE14MFP Mask          */

#define SYS_GPE_MFP3_PE15MFP_Pos         (24)                                              /*!< SYS_T::GPE_MFP3: PE15MFP Position      */
#define SYS_GPE_MFP3_PE15MFP_Msk         (0x1ful << SYS_GPE_MFP3_PE15MFP_Pos)              /*!< SYS_T::GPE_MFP3: PE15MFP Mask          */

#define SYS_GPF_MFP0_PF0MFP_Pos          (0)                                               /*!< SYS_T::GPF_MFP0: PF0MFP Position       */
#define SYS_GPF_MFP0_PF0MFP_Msk          (0x1ful << SYS_GPF_MFP0_PF0MFP_Pos)               /*!< SYS_T::GPF_MFP0: PF0MFP Mask           */

#define SYS_GPF_MFP0_PF1MFP_Pos          (8)                                               /*!< SYS_T::GPF_MFP0: PF1MFP Position       */
#define SYS_GPF_MFP0_PF1MFP_Msk          (0x1ful << SYS_GPF_MFP0_PF1MFP_Pos)               /*!< SYS_T::GPF_MFP0: PF1MFP Mask           */

#define SYS_GPF_MFP0_PF2MFP_Pos          (16)                                              /*!< SYS_T::GPF_MFP0: PF2MFP Position       */
#define SYS_GPF_MFP0_PF2MFP_Msk          (0x1ful << SYS_GPF_MFP0_PF2MFP_Pos)               /*!< SYS_T::GPF_MFP0: PF2MFP Mask           */

#define SYS_GPF_MFP0_PF3MFP_Pos          (24)                                              /*!< SYS_T::GPF_MFP0: PF3MFP Position       */
#define SYS_GPF_MFP0_PF3MFP_Msk          (0x1ful << SYS_GPF_MFP0_PF3MFP_Pos)               /*!< SYS_T::GPF_MFP0: PF3MFP Mask           */

#define SYS_GPF_MFP1_PF4MFP_Pos          (0)                                               /*!< SYS_T::GPF_MFP1: PF4MFP Position       */
#define SYS_GPF_MFP1_PF4MFP_Msk          (0x1ful << SYS_GPF_MFP1_PF4MFP_Pos)               /*!< SYS_T::GPF_MFP1: PF4MFP Mask           */

#define SYS_GPF_MFP1_PF5MFP_Pos          (8)                                               /*!< SYS_T::GPF_MFP1: PF5MFP Position       */
#define SYS_GPF_MFP1_PF5MFP_Msk          (0x1ful << SYS_GPF_MFP1_PF5MFP_Pos)               /*!< SYS_T::GPF_MFP1: PF5MFP Mask           */

#define SYS_GPF_MFP1_PF6MFP_Pos          (16)                                              /*!< SYS_T::GPF_MFP1: PF6MFP Position       */
#define SYS_GPF_MFP1_PF6MFP_Msk          (0x1ful << SYS_GPF_MFP1_PF6MFP_Pos)               /*!< SYS_T::GPF_MFP1: PF6MFP Mask           */

#define SYS_GPF_MFP1_PF7MFP_Pos          (24)                                              /*!< SYS_T::GPF_MFP1: PF7MFP Position       */
#define SYS_GPF_MFP1_PF7MFP_Msk          (0x1ful << SYS_GPF_MFP1_PF7MFP_Pos)               /*!< SYS_T::GPF_MFP1: PF7MFP Mask           */

#define SYS_GPF_MFP2_PF8MFP_Pos          (0)                                               /*!< SYS_T::GPF_MFP2: PF8MFP Position       */
#define SYS_GPF_MFP2_PF8MFP_Msk          (0x1ful << SYS_GPF_MFP2_PF8MFP_Pos)               /*!< SYS_T::GPF_MFP2: PF8MFP Mask           */

#define SYS_GPF_MFP2_PF9MFP_Pos          (8)                                               /*!< SYS_T::GPF_MFP2: PF9MFP Position       */
#define SYS_GPF_MFP2_PF9MFP_Msk          (0x1ful << SYS_GPF_MFP2_PF9MFP_Pos)               /*!< SYS_T::GPF_MFP2: PF9MFP Mask           */

#define SYS_GPF_MFP2_PF10MFP_Pos         (16)                                              /*!< SYS_T::GPF_MFP2: PF10MFP Position      */
#define SYS_GPF_MFP2_PF10MFP_Msk         (0x1ful << SYS_GPF_MFP2_PF10MFP_Pos)              /*!< SYS_T::GPF_MFP2: PF10MFP Mask          */

#define SYS_GPF_MFP2_PF11MFP_Pos         (24)                                              /*!< SYS_T::GPF_MFP2: PF11MFP Position      */
#define SYS_GPF_MFP2_PF11MFP_Msk         (0x1ful << SYS_GPF_MFP2_PF11MFP_Pos)              /*!< SYS_T::GPF_MFP2: PF11MFP Mask          */

#define SYS_GPG_MFP0_PG0MFP_Pos          (0)                                               /*!< SYS_T::GPG_MFP0: PG0MFP Position       */
#define SYS_GPG_MFP0_PG0MFP_Msk          (0x1ful << SYS_GPG_MFP0_PG0MFP_Pos)               /*!< SYS_T::GPG_MFP0: PG0MFP Mask           */

#define SYS_GPG_MFP0_PG1MFP_Pos          (8)                                               /*!< SYS_T::GPG_MFP0: PG1MFP Position       */
#define SYS_GPG_MFP0_PG1MFP_Msk          (0x1ful << SYS_GPG_MFP0_PG1MFP_Pos)               /*!< SYS_T::GPG_MFP0: PG1MFP Mask           */

#define SYS_GPG_MFP0_PG2MFP_Pos          (16)                                              /*!< SYS_T::GPG_MFP0: PG2MFP Position       */
#define SYS_GPG_MFP0_PG2MFP_Msk          (0x1ful << SYS_GPG_MFP0_PG2MFP_Pos)               /*!< SYS_T::GPG_MFP0: PG2MFP Mask           */

#define SYS_GPG_MFP0_PG3MFP_Pos          (24)                                              /*!< SYS_T::GPG_MFP0: PG3MFP Position       */
#define SYS_GPG_MFP0_PG3MFP_Msk          (0x1ful << SYS_GPG_MFP0_PG3MFP_Pos)               /*!< SYS_T::GPG_MFP0: PG3MFP Mask           */

#define SYS_GPG_MFP1_PG4MFP_Pos          (0)                                               /*!< SYS_T::GPG_MFP1: PG4MFP Position       */
#define SYS_GPG_MFP1_PG4MFP_Msk          (0x1ful << SYS_GPG_MFP1_PG4MFP_Pos)               /*!< SYS_T::GPG_MFP1: PG4MFP Mask           */

#define SYS_GPG_MFP1_PG5MFP_Pos          (8)                                               /*!< SYS_T::GPG_MFP1: PG5MFP Position       */
#define SYS_GPG_MFP1_PG5MFP_Msk          (0x1ful << SYS_GPG_MFP1_PG5MFP_Pos)               /*!< SYS_T::GPG_MFP1: PG5MFP Mask           */

#define SYS_GPG_MFP1_PG6MFP_Pos          (16)                                              /*!< SYS_T::GPG_MFP1: PG6MFP Position       */
#define SYS_GPG_MFP1_PG6MFP_Msk          (0x1ful << SYS_GPG_MFP1_PG6MFP_Pos)               /*!< SYS_T::GPG_MFP1: PG6MFP Mask           */

#define SYS_GPG_MFP1_PG7MFP_Pos          (24)                                              /*!< SYS_T::GPG_MFP1: PG7MFP Position       */
#define SYS_GPG_MFP1_PG7MFP_Msk          (0x1ful << SYS_GPG_MFP1_PG7MFP_Pos)               /*!< SYS_T::GPG_MFP1: PG7MFP Mask           */

#define SYS_GPG_MFP2_PG8MFP_Pos          (0)                                               /*!< SYS_T::GPG_MFP2: PG8MFP Position       */
#define SYS_GPG_MFP2_PG8MFP_Msk          (0x1ful << SYS_GPG_MFP2_PG8MFP_Pos)               /*!< SYS_T::GPG_MFP2: PG8MFP Mask           */

#define SYS_GPG_MFP2_PG9MFP_Pos          (8)                                               /*!< SYS_T::GPG_MFP2: PG9MFP Position       */
#define SYS_GPG_MFP2_PG9MFP_Msk          (0x1ful << SYS_GPG_MFP2_PG9MFP_Pos)               /*!< SYS_T::GPG_MFP2: PG9MFP Mask           */

#define SYS_GPG_MFP2_PG10MFP_Pos         (16)                                              /*!< SYS_T::GPG_MFP2: PG10MFP Position      */
#define SYS_GPG_MFP2_PG10MFP_Msk         (0x1ful << SYS_GPG_MFP2_PG10MFP_Pos)              /*!< SYS_T::GPG_MFP2: PG10MFP Mask          */

#define SYS_GPG_MFP2_PG11MFP_Pos         (24)                                              /*!< SYS_T::GPG_MFP2: PG11MFP Position      */
#define SYS_GPG_MFP2_PG11MFP_Msk         (0x1ful << SYS_GPG_MFP2_PG11MFP_Pos)              /*!< SYS_T::GPG_MFP2: PG11MFP Mask          */

#define SYS_GPG_MFP3_PG12MFP_Pos         (0)                                               /*!< SYS_T::GPG_MFP3: PG12MFP Position      */
#define SYS_GPG_MFP3_PG12MFP_Msk         (0x1ful << SYS_GPG_MFP3_PG12MFP_Pos)              /*!< SYS_T::GPG_MFP3: PG12MFP Mask          */

#define SYS_GPG_MFP3_PG13MFP_Pos         (8)                                               /*!< SYS_T::GPG_MFP3: PG13MFP Position      */
#define SYS_GPG_MFP3_PG13MFP_Msk         (0x1ful << SYS_GPG_MFP3_PG13MFP_Pos)              /*!< SYS_T::GPG_MFP3: PG13MFP Mask          */

#define SYS_GPG_MFP3_PG14MFP_Pos         (16)                                              /*!< SYS_T::GPG_MFP3: PG14MFP Position      */
#define SYS_GPG_MFP3_PG14MFP_Msk         (0x1ful << SYS_GPG_MFP3_PG14MFP_Pos)              /*!< SYS_T::GPG_MFP3: PG14MFP Mask          */

#define SYS_GPG_MFP3_PG15MFP_Pos         (24)                                              /*!< SYS_T::GPG_MFP3: PG15MFP Position      */
#define SYS_GPG_MFP3_PG15MFP_Msk         (0x1ful << SYS_GPG_MFP3_PG15MFP_Pos)              /*!< SYS_T::GPG_MFP3: PG15MFP Mask          */

#define SYS_GPH_MFP0_PH0MFP_Pos          (0)                                               /*!< SYS_T::GPH_MFP0: PH0MFP Position       */
#define SYS_GPH_MFP0_PH0MFP_Msk          (0x1ful << SYS_GPH_MFP0_PH0MFP_Pos)               /*!< SYS_T::GPH_MFP0: PH0MFP Mask           */

#define SYS_GPH_MFP0_PH1MFP_Pos          (8)                                               /*!< SYS_T::GPH_MFP0: PH1MFP Position       */
#define SYS_GPH_MFP0_PH1MFP_Msk          (0x1ful << SYS_GPH_MFP0_PH1MFP_Pos)               /*!< SYS_T::GPH_MFP0: PH1MFP Mask           */

#define SYS_GPH_MFP0_PH2MFP_Pos          (16)                                              /*!< SYS_T::GPH_MFP0: PH2MFP Position       */
#define SYS_GPH_MFP0_PH2MFP_Msk          (0x1ful << SYS_GPH_MFP0_PH2MFP_Pos)               /*!< SYS_T::GPH_MFP0: PH2MFP Mask           */

#define SYS_GPH_MFP0_PH3MFP_Pos          (24)                                              /*!< SYS_T::GPH_MFP0: PH3MFP Position       */
#define SYS_GPH_MFP0_PH3MFP_Msk          (0x1ful << SYS_GPH_MFP0_PH3MFP_Pos)               /*!< SYS_T::GPH_MFP0: PH3MFP Mask           */

#define SYS_GPH_MFP1_PH4MFP_Pos          (0)                                               /*!< SYS_T::GPH_MFP1: PH4MFP Position       */
#define SYS_GPH_MFP1_PH4MFP_Msk          (0x1ful << SYS_GPH_MFP1_PH4MFP_Pos)               /*!< SYS_T::GPH_MFP1: PH4MFP Mask           */

#define SYS_GPH_MFP1_PH5MFP_Pos          (8)                                               /*!< SYS_T::GPH_MFP1: PH5MFP Position       */
#define SYS_GPH_MFP1_PH5MFP_Msk          (0x1ful << SYS_GPH_MFP1_PH5MFP_Pos)               /*!< SYS_T::GPH_MFP1: PH5MFP Mask           */

#define SYS_GPH_MFP1_PH6MFP_Pos          (16)                                              /*!< SYS_T::GPH_MFP1: PH6MFP Position       */
#define SYS_GPH_MFP1_PH6MFP_Msk          (0x1ful << SYS_GPH_MFP1_PH6MFP_Pos)               /*!< SYS_T::GPH_MFP1: PH6MFP Mask           */

#define SYS_GPH_MFP1_PH7MFP_Pos          (24)                                              /*!< SYS_T::GPH_MFP1: PH7MFP Position       */
#define SYS_GPH_MFP1_PH7MFP_Msk          (0x1ful << SYS_GPH_MFP1_PH7MFP_Pos)               /*!< SYS_T::GPH_MFP1: PH7MFP Mask           */

#define SYS_GPH_MFP2_PH8MFP_Pos          (0)                                               /*!< SYS_T::GPH_MFP2: PH8MFP Position       */
#define SYS_GPH_MFP2_PH8MFP_Msk          (0x1ful << SYS_GPH_MFP2_PH8MFP_Pos)               /*!< SYS_T::GPH_MFP2: PH8MFP Mask           */

#define SYS_GPH_MFP2_PH9MFP_Pos          (8)                                               /*!< SYS_T::GPH_MFP2: PH9MFP Position       */
#define SYS_GPH_MFP2_PH9MFP_Msk          (0x1ful << SYS_GPH_MFP2_PH9MFP_Pos)               /*!< SYS_T::GPH_MFP2: PH9MFP Mask           */

#define SYS_GPH_MFP2_PH10MFP_Pos         (16)                                              /*!< SYS_T::GPH_MFP2: PH10MFP Position      */
#define SYS_GPH_MFP2_PH10MFP_Msk         (0x1ful << SYS_GPH_MFP2_PH10MFP_Pos)              /*!< SYS_T::GPH_MFP2: PH10MFP Mask          */

#define SYS_GPH_MFP2_PH11MFP_Pos         (24)                                              /*!< SYS_T::GPH_MFP2: PH11MFP Position      */
#define SYS_GPH_MFP2_PH11MFP_Msk         (0x1ful << SYS_GPH_MFP2_PH11MFP_Pos)              /*!< SYS_T::GPH_MFP2: PH11MFP Mask          */

#define SYS_GPH_MFP3_PH12MFP_Pos         (0)                                               /*!< SYS_T::GPH_MFP3: PH12MFP Position      */
#define SYS_GPH_MFP3_PH12MFP_Msk         (0x1ful << SYS_GPH_MFP3_PH12MFP_Pos)              /*!< SYS_T::GPH_MFP3: PH12MFP Mask          */

#define SYS_GPH_MFP3_PH13MFP_Pos         (8)                                               /*!< SYS_T::GPH_MFP3: PH13MFP Position      */
#define SYS_GPH_MFP3_PH13MFP_Msk         (0x1ful << SYS_GPH_MFP3_PH13MFP_Pos)              /*!< SYS_T::GPH_MFP3: PH13MFP Mask          */

#define SYS_GPH_MFP3_PH14MFP_Pos         (16)                                              /*!< SYS_T::GPH_MFP3: PH14MFP Position      */
#define SYS_GPH_MFP3_PH14MFP_Msk         (0x1ful << SYS_GPH_MFP3_PH14MFP_Pos)              /*!< SYS_T::GPH_MFP3: PH14MFP Mask          */

#define SYS_GPH_MFP3_PH15MFP_Pos         (24)                                              /*!< SYS_T::GPH_MFP3: PH15MFP Position      */
#define SYS_GPH_MFP3_PH15MFP_Msk         (0x1ful << SYS_GPH_MFP3_PH15MFP_Pos)              /*!< SYS_T::GPH_MFP3: PH15MFP Mask          */

#define SYS_GPI_MFP1_PI6MFP_Pos          (16)                                              /*!< SYS_T::GPI_MFP1: PI6MFP Position       */
#define SYS_GPI_MFP1_PI6MFP_Msk          (0x1ful << SYS_GPI_MFP1_PI6MFP_Pos)               /*!< SYS_T::GPI_MFP1: PI6MFP Mask           */

#define SYS_GPI_MFP1_PI7MFP_Pos          (24)                                              /*!< SYS_T::GPI_MFP1: PI7MFP Position       */
#define SYS_GPI_MFP1_PI7MFP_Msk          (0x1ful << SYS_GPI_MFP1_PI7MFP_Pos)               /*!< SYS_T::GPI_MFP1: PI7MFP Mask           */

#define SYS_GPI_MFP2_PI8MFP_Pos          (0)                                               /*!< SYS_T::GPI_MFP2: PI8MFP Position       */
#define SYS_GPI_MFP2_PI8MFP_Msk          (0x1ful << SYS_GPI_MFP2_PI8MFP_Pos)               /*!< SYS_T::GPI_MFP2: PI8MFP Mask           */

#define SYS_GPI_MFP2_PI9MFP_Pos          (8)                                               /*!< SYS_T::GPI_MFP2: PI9MFP Position       */
#define SYS_GPI_MFP2_PI9MFP_Msk          (0x1ful << SYS_GPI_MFP2_PI9MFP_Pos)               /*!< SYS_T::GPI_MFP2: PI9MFP Mask           */

#define SYS_GPI_MFP2_PI10MFP_Pos         (16)                                              /*!< SYS_T::GPI_MFP2: PI10MFP Position      */
#define SYS_GPI_MFP2_PI10MFP_Msk         (0x1ful << SYS_GPI_MFP2_PI10MFP_Pos)              /*!< SYS_T::GPI_MFP2: PI10MFP Mask          */

#define SYS_GPI_MFP2_PI11MFP_Pos         (24)                                              /*!< SYS_T::GPI_MFP2: PI11MFP Position      */
#define SYS_GPI_MFP2_PI11MFP_Msk         (0x1ful << SYS_GPI_MFP2_PI11MFP_Pos)              /*!< SYS_T::GPI_MFP2: PI11MFP Mask          */

#define SYS_GPI_MFP3_PI12MFP_Pos         (0)                                               /*!< SYS_T::GPI_MFP3: PI12MFP Position      */
#define SYS_GPI_MFP3_PI12MFP_Msk         (0x1ful << SYS_GPI_MFP3_PI12MFP_Pos)              /*!< SYS_T::GPI_MFP3: PI12MFP Mask          */

#define SYS_GPI_MFP3_PI13MFP_Pos         (8)                                               /*!< SYS_T::GPI_MFP3: PI13MFP Position      */
#define SYS_GPI_MFP3_PI13MFP_Msk         (0x1ful << SYS_GPI_MFP3_PI13MFP_Pos)              /*!< SYS_T::GPI_MFP3: PI13MFP Mask          */

#define SYS_GPI_MFP3_PI14MFP_Pos         (16)                                              /*!< SYS_T::GPI_MFP3: PI14MFP Position      */
#define SYS_GPI_MFP3_PI14MFP_Msk         (0x1ful << SYS_GPI_MFP3_PI14MFP_Pos)              /*!< SYS_T::GPI_MFP3: PI14MFP Mask          */

#define SYS_GPI_MFP3_PI15MFP_Pos         (24)                                              /*!< SYS_T::GPI_MFP3: PI15MFP Position      */
#define SYS_GPI_MFP3_PI15MFP_Msk         (0x1ful << SYS_GPI_MFP3_PI15MFP_Pos)              /*!< SYS_T::GPI_MFP3: PI15MFP Mask          */

#define SYS_GPJ_MFP0_PJ0MFP_Pos          (0)                                               /*!< SYS_T::GPJ_MFP0: PJ0MFP Position       */
#define SYS_GPJ_MFP0_PJ0MFP_Msk          (0x1ful << SYS_GPJ_MFP0_PJ0MFP_Pos)               /*!< SYS_T::GPJ_MFP0: PJ0MFP Mask           */

#define SYS_GPJ_MFP0_PJ1MFP_Pos          (8)                                               /*!< SYS_T::GPJ_MFP0: PJ1MFP Position       */
#define SYS_GPJ_MFP0_PJ1MFP_Msk          (0x1ful << SYS_GPJ_MFP0_PJ1MFP_Pos)               /*!< SYS_T::GPJ_MFP0: PJ1MFP Mask           */

#define SYS_GPJ_MFP0_PJ2MFP_Pos          (16)                                              /*!< SYS_T::GPJ_MFP0: PJ2MFP Position       */
#define SYS_GPJ_MFP0_PJ2MFP_Msk          (0x1ful << SYS_GPJ_MFP0_PJ2MFP_Pos)               /*!< SYS_T::GPJ_MFP0: PJ2MFP Mask           */

#define SYS_GPJ_MFP0_PJ3MFP_Pos          (24)                                              /*!< SYS_T::GPJ_MFP0: PJ3MFP Position       */
#define SYS_GPJ_MFP0_PJ3MFP_Msk          (0x1ful << SYS_GPJ_MFP0_PJ3MFP_Pos)               /*!< SYS_T::GPJ_MFP0: PJ3MFP Mask           */

#define SYS_GPJ_MFP1_PJ4MFP_Pos          (0)                                               /*!< SYS_T::GPJ_MFP1: PJ4MFP Position       */
#define SYS_GPJ_MFP1_PJ4MFP_Msk          (0x1ful << SYS_GPJ_MFP1_PJ4MFP_Pos)               /*!< SYS_T::GPJ_MFP1: PJ4MFP Mask           */

#define SYS_GPJ_MFP1_PJ5MFP_Pos          (8)                                               /*!< SYS_T::GPJ_MFP1: PJ5MFP Position       */
#define SYS_GPJ_MFP1_PJ5MFP_Msk          (0x1ful << SYS_GPJ_MFP1_PJ5MFP_Pos)               /*!< SYS_T::GPJ_MFP1: PJ5MFP Mask           */

#define SYS_GPJ_MFP1_PJ6MFP_Pos          (16)                                              /*!< SYS_T::GPJ_MFP1: PJ6MFP Position       */
#define SYS_GPJ_MFP1_PJ6MFP_Msk          (0x1ful << SYS_GPJ_MFP1_PJ6MFP_Pos)               /*!< SYS_T::GPJ_MFP1: PJ6MFP Mask           */

#define SYS_GPJ_MFP1_PJ7MFP_Pos          (24)                                              /*!< SYS_T::GPJ_MFP1: PJ7MFP Position       */
#define SYS_GPJ_MFP1_PJ7MFP_Msk          (0x1ful << SYS_GPJ_MFP1_PJ7MFP_Pos)               /*!< SYS_T::GPJ_MFP1: PJ7MFP Mask           */

#define SYS_GPJ_MFP2_PJ8MFP_Pos          (0)                                               /*!< SYS_T::GPJ_MFP2: PJ8MFP Position       */
#define SYS_GPJ_MFP2_PJ8MFP_Msk          (0x1ful << SYS_GPJ_MFP2_PJ8MFP_Pos)               /*!< SYS_T::GPJ_MFP2: PJ8MFP Mask           */

#define SYS_GPJ_MFP2_PJ9MFP_Pos          (8)                                               /*!< SYS_T::GPJ_MFP2: PJ9MFP Position       */
#define SYS_GPJ_MFP2_PJ9MFP_Msk          (0x1ful << SYS_GPJ_MFP2_PJ9MFP_Pos)               /*!< SYS_T::GPJ_MFP2: PJ9MFP Mask           */

#define SYS_GPJ_MFP2_PJ10MFP_Pos         (16)                                              /*!< SYS_T::GPJ_MFP2: PJ10MFP Position      */
#define SYS_GPJ_MFP2_PJ10MFP_Msk         (0x1ful << SYS_GPJ_MFP2_PJ10MFP_Pos)              /*!< SYS_T::GPJ_MFP2: PJ10MFP Mask          */

#define SYS_GPJ_MFP2_PJ11MFP_Pos         (24)                                              /*!< SYS_T::GPJ_MFP2: PJ11MFP Position      */
#define SYS_GPJ_MFP2_PJ11MFP_Msk         (0x1ful << SYS_GPJ_MFP2_PJ11MFP_Pos)              /*!< SYS_T::GPJ_MFP2: PJ11MFP Mask          */

#define SYS_GPJ_MFP3_PJ12MFP_Pos         (0)                                               /*!< SYS_T::GPJ_MFP3: PJ12MFP Position      */
#define SYS_GPJ_MFP3_PJ12MFP_Msk         (0x1ful << SYS_GPJ_MFP3_PJ12MFP_Pos)              /*!< SYS_T::GPJ_MFP3: PJ12MFP Mask          */

#define SYS_GPJ_MFP3_PJ13MFP_Pos         (8)                                               /*!< SYS_T::GPJ_MFP3: PJ13MFP Position      */
#define SYS_GPJ_MFP3_PJ13MFP_Msk         (0x1ful << SYS_GPJ_MFP3_PJ13MFP_Pos)              /*!< SYS_T::GPJ_MFP3: PJ13MFP Mask          */

#define SYS_NMIEN_BODOUT_Pos             (0)                                               /*!< SYS_T::NMIEN: BODOUT Position          */
#define SYS_NMIEN_BODOUT_Msk             (0x1ul << SYS_NMIEN_BODOUT_Pos)                   /*!< SYS_T::NMIEN: BODOUT Mask              */

#define SYS_NMIEN_IRCINT_Pos             (1)                                               /*!< SYS_T::NMIEN: IRCINT Position          */
#define SYS_NMIEN_IRCINT_Msk             (0x1ul << SYS_NMIEN_IRCINT_Pos)                   /*!< SYS_T::NMIEN: IRCINT Mask              */

#define SYS_NMIEN_PWRWUINT_Pos           (2)                                               /*!< SYS_T::NMIEN: PWRWUINT Position        */
#define SYS_NMIEN_PWRWUINT_Msk           (0x1ul << SYS_NMIEN_PWRWUINT_Pos)                 /*!< SYS_T::NMIEN: PWRWUINT Mask            */

#define SYS_NMIEN_SRAMPERR_Pos           (3)                                               /*!< SYS_T::NMIEN: SRAMPERR Position        */
#define SYS_NMIEN_SRAMPERR_Msk           (0x1ul << SYS_NMIEN_SRAMPERR_Pos)                 /*!< SYS_T::NMIEN: SRAMPERR Mask            */

#define SYS_NMIEN_CLKFAIL_Pos            (4)                                               /*!< SYS_T::NMIEN: CLKFAIL Position         */
#define SYS_NMIEN_CLKFAIL_Msk            (0x1ul << SYS_NMIEN_CLKFAIL_Pos)                  /*!< SYS_T::NMIEN: CLKFAIL Mask             */

#define SYS_NMIEN_RTCINT_Pos             (6)                                               /*!< SYS_T::NMIEN: RTCINT Position          */
#define SYS_NMIEN_RTCINT_Msk             (0x1ul << SYS_NMIEN_RTCINT_Pos)                   /*!< SYS_T::NMIEN: RTCINT Mask              */

#define SYS_NMIEN_RTCTMPINT_Pos          (7)                                               /*!< SYS_T::NMIEN: RTCTMPINT Position       */
#define SYS_NMIEN_RTCTMPINT_Msk          (0x1ul << SYS_NMIEN_RTCTMPINT_Pos)                /*!< SYS_T::NMIEN: RTCTMPINT Mask           */

#define SYS_NMIEN_EINT0_Pos              (8)                                               /*!< SYS_T::NMIEN: EINT0 Position           */
#define SYS_NMIEN_EINT0_Msk              (0x1ul << SYS_NMIEN_EINT0_Pos)                    /*!< SYS_T::NMIEN: EINT0 Mask               */

#define SYS_NMIEN_EINT1_Pos              (9)                                               /*!< SYS_T::NMIEN: EINT1 Position           */
#define SYS_NMIEN_EINT1_Msk              (0x1ul << SYS_NMIEN_EINT1_Pos)                    /*!< SYS_T::NMIEN: EINT1 Mask               */

#define SYS_NMIEN_EINT2_Pos              (10)                                              /*!< SYS_T::NMIEN: EINT2 Position           */
#define SYS_NMIEN_EINT2_Msk              (0x1ul << SYS_NMIEN_EINT2_Pos)                    /*!< SYS_T::NMIEN: EINT2 Mask               */

#define SYS_NMIEN_EINT3_Pos              (11)                                              /*!< SYS_T::NMIEN: EINT3 Position           */
#define SYS_NMIEN_EINT3_Msk              (0x1ul << SYS_NMIEN_EINT3_Pos)                    /*!< SYS_T::NMIEN: EINT3 Mask               */

#define SYS_NMIEN_EINT4_Pos              (12)                                              /*!< SYS_T::NMIEN: EINT4 Position           */
#define SYS_NMIEN_EINT4_Msk              (0x1ul << SYS_NMIEN_EINT4_Pos)                    /*!< SYS_T::NMIEN: EINT4 Mask               */

#define SYS_NMIEN_EINT5_Pos              (13)                                              /*!< SYS_T::NMIEN: EINT5 Position           */
#define SYS_NMIEN_EINT5_Msk              (0x1ul << SYS_NMIEN_EINT5_Pos)                    /*!< SYS_T::NMIEN: EINT5 Mask               */

#define SYS_NMIEN_EINT6_Pos              (14)                                              /*!< SYS_T::NMIEN: EINT6 Position           */
#define SYS_NMIEN_EINT6_Msk              (0x1ul << SYS_NMIEN_EINT6_Pos)                    /*!< SYS_T::NMIEN: EINT6 Mask               */

#define SYS_NMIEN_EINT7_Pos              (15)                                              /*!< SYS_T::NMIEN: EINT7 Position           */
#define SYS_NMIEN_EINT7_Msk              (0x1ul << SYS_NMIEN_EINT7_Pos)                    /*!< SYS_T::NMIEN: EINT7 Mask               */

#define SYS_NMIEN_UART0INT_Pos           (16)                                              /*!< SYS_T::NMIEN: UART0INT Position        */
#define SYS_NMIEN_UART0INT_Msk           (0x1ul << SYS_NMIEN_UART0INT_Pos)                 /*!< SYS_T::NMIEN: UART0INT Mask            */

#define SYS_NMIEN_UART1INT_Pos           (17)                                              /*!< SYS_T::NMIEN: UART1INT Position        */
#define SYS_NMIEN_UART1INT_Msk           (0x1ul << SYS_NMIEN_UART1INT_Pos)                 /*!< SYS_T::NMIEN: UART1INT Mask            */

#define SYS_NMISTS_BODOUT_Pos            (0)                                               /*!< SYS_T::NMISTS: BODOUT Position         */
#define SYS_NMISTS_BODOUT_Msk            (0x1ul << SYS_NMISTS_BODOUT_Pos)                  /*!< SYS_T::NMISTS: BODOUT Mask             */

#define SYS_NMISTS_IRCINT_Pos            (1)                                               /*!< SYS_T::NMISTS: IRCINT Position         */
#define SYS_NMISTS_IRCINT_Msk            (0x1ul << SYS_NMISTS_IRCINT_Pos)                  /*!< SYS_T::NMISTS: IRCINT Mask             */

#define SYS_NMISTS_PWRWUINT_Pos          (2)                                               /*!< SYS_T::NMISTS: PWRWUINT Position       */
#define SYS_NMISTS_PWRWUINT_Msk          (0x1ul << SYS_NMISTS_PWRWUINT_Pos)                /*!< SYS_T::NMISTS: PWRWUINT Mask           */

#define SYS_NMISTS_SRAMPERR_Pos          (3)                                               /*!< SYS_T::NMISTS: SRAMPERR Position       */
#define SYS_NMISTS_SRAMPERR_Msk          (0x1ul << SYS_NMISTS_SRAMPERR_Pos)                /*!< SYS_T::NMISTS: SRAMPERR Mask           */

#define SYS_NMISTS_CLKFAIL_Pos           (4)                                               /*!< SYS_T::NMISTS: CLKFAIL Position        */
#define SYS_NMISTS_CLKFAIL_Msk           (0x1ul << SYS_NMISTS_CLKFAIL_Pos)                 /*!< SYS_T::NMISTS: CLKFAIL Mask            */

#define SYS_NMISTS_RTCINT_Pos            (6)                                               /*!< SYS_T::NMISTS: RTCINT Position         */
#define SYS_NMISTS_RTCINT_Msk            (0x1ul << SYS_NMISTS_RTCINT_Pos)                  /*!< SYS_T::NMISTS: RTCINT Mask             */

#define SYS_NMISTS_RTCTMPINT_Pos         (7)                                               /*!< SYS_T::NMISTS: RTCTMPINT Position      */
#define SYS_NMISTS_RTCTMPINT_Msk         (0x1ul << SYS_NMISTS_RTCTMPINT_Pos)               /*!< SYS_T::NMISTS: RTCTMPINT Mask          */

#define SYS_NMISTS_EINT0_Pos             (8)                                               /*!< SYS_T::NMISTS: EINT0 Position          */
#define SYS_NMISTS_EINT0_Msk             (0x1ul << SYS_NMISTS_EINT0_Pos)                   /*!< SYS_T::NMISTS: EINT0 Mask              */

#define SYS_NMISTS_EINT1_Pos             (9)                                               /*!< SYS_T::NMISTS: EINT1 Position          */
#define SYS_NMISTS_EINT1_Msk             (0x1ul << SYS_NMISTS_EINT1_Pos)                   /*!< SYS_T::NMISTS: EINT1 Mask              */

#define SYS_NMISTS_EINT2_Pos             (10)                                              /*!< SYS_T::NMISTS: EINT2 Position          */
#define SYS_NMISTS_EINT2_Msk             (0x1ul << SYS_NMISTS_EINT2_Pos)                   /*!< SYS_T::NMISTS: EINT2 Mask              */

#define SYS_NMISTS_EINT3_Pos             (11)                                              /*!< SYS_T::NMISTS: EINT3 Position          */
#define SYS_NMISTS_EINT3_Msk             (0x1ul << SYS_NMISTS_EINT3_Pos)                   /*!< SYS_T::NMISTS: EINT3 Mask              */

#define SYS_NMISTS_EINT4_Pos             (12)                                              /*!< SYS_T::NMISTS: EINT4 Position          */
#define SYS_NMISTS_EINT4_Msk             (0x1ul << SYS_NMISTS_EINT4_Pos)                   /*!< SYS_T::NMISTS: EINT4 Mask              */

#define SYS_NMISTS_EINT5_Pos             (13)                                              /*!< SYS_T::NMISTS: EINT5 Position          */
#define SYS_NMISTS_EINT5_Msk             (0x1ul << SYS_NMISTS_EINT5_Pos)                   /*!< SYS_T::NMISTS: EINT5 Mask              */

#define SYS_NMISTS_EINT6_Pos             (14)                                              /*!< SYS_T::NMISTS: EINT6 Position          */
#define SYS_NMISTS_EINT6_Msk             (0x1ul << SYS_NMISTS_EINT6_Pos)                   /*!< SYS_T::NMISTS: EINT6 Mask              */

#define SYS_NMISTS_EINT7_Pos             (15)                                              /*!< SYS_T::NMISTS: EINT7 Position          */
#define SYS_NMISTS_EINT7_Msk             (0x1ul << SYS_NMISTS_EINT7_Pos)                   /*!< SYS_T::NMISTS: EINT7 Mask              */

#define SYS_NMISTS_UART0INT_Pos          (16)                                              /*!< SYS_T::NMISTS: UART0INT Position       */
#define SYS_NMISTS_UART0INT_Msk          (0x1ul << SYS_NMISTS_UART0INT_Pos)                /*!< SYS_T::NMISTS: UART0INT Mask           */

#define SYS_NMISTS_UART1INT_Pos          (17)                                              /*!< SYS_T::NMISTS: UART1INT Position       */
#define SYS_NMISTS_UART1INT_Msk          (0x1ul << SYS_NMISTS_UART1INT_Pos)                /*!< SYS_T::NMISTS: UART1INT Mask           */

#define SYS_SRAMBCTL_SR0BIST_Pos         (0)                                               /*!< SYS_T::SRAMBCTL: SR0BIST Position      */
#define SYS_SRAMBCTL_SR0BIST_Msk         (0x1ul << SYS_SRAMBCTL_SR0BIST_Pos)               /*!< SYS_T::SRAMBCTL: SR0BIST Mask          */

#define SYS_SRAMBCTL_SR1BIST_Pos         (1)                                               /*!< SYS_T::SRAMBCTL: SR1BIST Position      */
#define SYS_SRAMBCTL_SR1BIST_Msk         (0x1ul << SYS_SRAMBCTL_SR1BIST_Pos)               /*!< SYS_T::SRAMBCTL: SR1BIST Mask          */

#define SYS_SRAMBCTL_SR2BIST_Pos         (2)                                               /*!< SYS_T::SRAMBCTL: SR2BIST Position      */
#define SYS_SRAMBCTL_SR2BIST_Msk         (0x1ul << SYS_SRAMBCTL_SR2BIST_Pos)               /*!< SYS_T::SRAMBCTL: SR2BIST Mask          */

#define SYS_SRAMBCTL_SR3BIST_Pos         (3)                                               /*!< SYS_T::SRAMBCTL: SR3BIST Position      */
#define SYS_SRAMBCTL_SR3BIST_Msk         (0x1ul << SYS_SRAMBCTL_SR3BIST_Pos)               /*!< SYS_T::SRAMBCTL: SR3BIST Mask          */

#define SYS_SRAMBCTL_LPSRBIST_Pos        (4)                                               /*!< SYS_T::SRAMBCTL: LPSRBIST Position     */
#define SYS_SRAMBCTL_LPSRBIST_Msk        (0x1ul << SYS_SRAMBCTL_LPSRBIST_Pos)              /*!< SYS_T::SRAMBCTL: LPSRBIST Mask         */

#define SYS_SRAMBCTL_CAN0BIST_Pos        (8)                                               /*!< SYS_T::SRAMBCTL: CAN0BIST Position     */
#define SYS_SRAMBCTL_CAN0BIST_Msk        (0x1ul << SYS_SRAMBCTL_CAN0BIST_Pos)              /*!< SYS_T::SRAMBCTL: CAN0BIST Mask         */

#define SYS_SRAMBCTL_CAN1BIST_Pos        (9)                                               /*!< SYS_T::SRAMBCTL: CAN1BIST Position     */
#define SYS_SRAMBCTL_CAN1BIST_Msk        (0x1ul << SYS_SRAMBCTL_CAN1BIST_Pos)              /*!< SYS_T::SRAMBCTL: CAN1BIST Mask         */

#define SYS_SRAMBCTL_SPIM0BIST_Pos       (10)                                              /*!< SYS_T::SRAMBCTL: SPIM0BIST Position    */
#define SYS_SRAMBCTL_SPIM0BIST_Msk       (0x1ul << SYS_SRAMBCTL_SPIM0BIST_Pos)             /*!< SYS_T::SRAMBCTL: SPIM0BIST Mask        */

#define SYS_SRAMBCTL_CCAPBIST_Pos        (12)                                              /*!< SYS_T::SRAMBCTL: CCAPBIST Position     */
#define SYS_SRAMBCTL_CCAPBIST_Msk        (0x1ul << SYS_SRAMBCTL_CCAPBIST_Pos)              /*!< SYS_T::SRAMBCTL: CCAPBIST Mask         */

#define SYS_SRAMBCTL_I3CBIST_Pos         (13)                                              /*!< SYS_T::SRAMBCTL: I3CBIST Position      */
#define SYS_SRAMBCTL_I3CBIST_Msk         (0x1ul << SYS_SRAMBCTL_I3CBIST_Pos)               /*!< SYS_T::SRAMBCTL: I3CBIST Mask          */

#define SYS_SRAMBCTL_KSBIST_Pos          (14)                                              /*!< SYS_T::SRAMBCTL: KSBIST Position       */
#define SYS_SRAMBCTL_KSBIST_Msk          (0x1ul << SYS_SRAMBCTL_KSBIST_Pos)                /*!< SYS_T::SRAMBCTL: KSBIST Mask           */

#define SYS_SRAMBCTL_RSABIST_Pos         (15)                                              /*!< SYS_T::SRAMBCTL: RSABIST Position      */
#define SYS_SRAMBCTL_RSABIST_Msk         (0x1ul << SYS_SRAMBCTL_RSABIST_Pos)               /*!< SYS_T::SRAMBCTL: RSABIST Mask          */

#define SYS_SRAMBCTL_DMICBIST_Pos        (16)                                              /*!< SYS_T::SRAMBCTL: DMICBIST Position     */
#define SYS_SRAMBCTL_DMICBIST_Msk        (0x1ul << SYS_SRAMBCTL_DMICBIST_Pos)              /*!< SYS_T::SRAMBCTL: DMICBIST Mask         */

#define SYS_SRAMBCTL_EMACBIST_Pos        (17)                                              /*!< SYS_T::SRAMBCTL: EMACBIST Position     */
#define SYS_SRAMBCTL_EMACBIST_Msk        (0x1ul << SYS_SRAMBCTL_EMACBIST_Pos)              /*!< SYS_T::SRAMBCTL: EMACBIST Mask         */

#define SYS_SRAMBCTL_NPUBIST_Pos         (18)                                              /*!< SYS_T::SRAMBCTL: NPUBIST Position      */
#define SYS_SRAMBCTL_NPUBIST_Msk         (0x1ul << SYS_SRAMBCTL_NPUBIST_Pos)               /*!< SYS_T::SRAMBCTL: NPUBIST Mask          */

#define SYS_SRAMBCTL_USBDBIST_Pos        (24)                                              /*!< SYS_T::SRAMBCTL: USBDBIST Position     */
#define SYS_SRAMBCTL_USBDBIST_Msk        (0x1ul << SYS_SRAMBCTL_USBDBIST_Pos)              /*!< SYS_T::SRAMBCTL: USBDBIST Mask         */

#define SYS_SRAMBCTL_HSUSBDBIST_Pos      (25)                                              /*!< SYS_T::SRAMBCTL: HSUSBDBIST Position   */
#define SYS_SRAMBCTL_HSUSBDBIST_Msk      (0x1ul << SYS_SRAMBCTL_HSUSBDBIST_Pos)            /*!< SYS_T::SRAMBCTL: HSUSBDBIST Mask       */

#define SYS_SRAMBCTL_HSUSBHBIST_Pos      (26)                                              /*!< SYS_T::SRAMBCTL: HSUSBHBIST Position   */
#define SYS_SRAMBCTL_HSUSBHBIST_Msk      (0x1ul << SYS_SRAMBCTL_HSUSBHBIST_Pos)            /*!< SYS_T::SRAMBCTL: HSUSBHBIST Mask       */

#define SYS_SRAMBFF_SR0BFF_Pos           (0)                                               /*!< SYS_T::SRAMBFF: SR0BFF Position        */
#define SYS_SRAMBFF_SR0BFF_Msk           (0x1ul << SYS_SRAMBFF_SR0BFF_Pos)                 /*!< SYS_T::SRAMBFF: SR0BFF Mask            */

#define SYS_SRAMBFF_SR1BFF_Pos           (1)                                               /*!< SYS_T::SRAMBFF: SR1BFF Position        */
#define SYS_SRAMBFF_SR1BFF_Msk           (0x1ul << SYS_SRAMBFF_SR1BFF_Pos)                 /*!< SYS_T::SRAMBFF: SR1BFF Mask            */

#define SYS_SRAMBFF_SR2BFF_Pos           (2)                                               /*!< SYS_T::SRAMBFF: SR2BFF Position        */
#define SYS_SRAMBFF_SR2BFF_Msk           (0x1ul << SYS_SRAMBFF_SR2BFF_Pos)                 /*!< SYS_T::SRAMBFF: SR2BFF Mask            */

#define SYS_SRAMBFF_SR3BFF_Pos           (3)                                               /*!< SYS_T::SRAMBFF: SR3BFF Position        */
#define SYS_SRAMBFF_SR3BFF_Msk           (0x1ul << SYS_SRAMBFF_SR3BFF_Pos)                 /*!< SYS_T::SRAMBFF: SR3BFF Mask            */

#define SYS_SRAMBFF_LPSRBFF_Pos          (4)                                               /*!< SYS_T::SRAMBFF: LPSRBFF Position       */
#define SYS_SRAMBFF_LPSRBFF_Msk          (0x1ul << SYS_SRAMBFF_LPSRBFF_Pos)                /*!< SYS_T::SRAMBFF: LPSRBFF Mask           */

#define SYS_SRAMBFF_CAN0BFF_Pos          (8)                                               /*!< SYS_T::SRAMBFF: CAN0BFF Position       */
#define SYS_SRAMBFF_CAN0BFF_Msk          (0x1ul << SYS_SRAMBFF_CAN0BFF_Pos)                /*!< SYS_T::SRAMBFF: CAN0BFF Mask           */

#define SYS_SRAMBFF_CAN1BFF_Pos          (9)                                               /*!< SYS_T::SRAMBFF: CAN1BFF Position       */
#define SYS_SRAMBFF_CAN1BFF_Msk          (0x1ul << SYS_SRAMBFF_CAN1BFF_Pos)                /*!< SYS_T::SRAMBFF: CAN1BFF Mask           */

#define SYS_SRAMBFF_SPIM0BFF_Pos         (10)                                              /*!< SYS_T::SRAMBFF: SPIM0BFF Position      */
#define SYS_SRAMBFF_SPIM0BFF_Msk         (0x1ul << SYS_SRAMBFF_SPIM0BFF_Pos)               /*!< SYS_T::SRAMBFF: SPIM0BFF Mask          */

#define SYS_SRAMBFF_CCAPBFF_Pos          (12)                                              /*!< SYS_T::SRAMBFF: CCAPBFF Position       */
#define SYS_SRAMBFF_CCAPBFF_Msk          (0x1ul << SYS_SRAMBFF_CCAPBFF_Pos)                /*!< SYS_T::SRAMBFF: CCAPBFF Mask           */

#define SYS_SRAMBFF_I3CBFF_Pos           (13)                                              /*!< SYS_T::SRAMBFF: I3CBFF Position        */
#define SYS_SRAMBFF_I3CBFF_Msk           (0x1ul << SYS_SRAMBFF_I3CBFF_Pos)                 /*!< SYS_T::SRAMBFF: I3CBFF Mask            */

#define SYS_SRAMBFF_KSBFF_Pos            (14)                                              /*!< SYS_T::SRAMBFF: KSBFF Position         */
#define SYS_SRAMBFF_KSBFF_Msk            (0x1ul << SYS_SRAMBFF_KSBFF_Pos)                  /*!< SYS_T::SRAMBFF: KSBFF Mask             */

#define SYS_SRAMBFF_RSABFF_Pos           (15)                                              /*!< SYS_T::SRAMBFF: RSABFF Position        */
#define SYS_SRAMBFF_RSABFF_Msk           (0x1ul << SYS_SRAMBFF_RSABFF_Pos)                 /*!< SYS_T::SRAMBFF: RSABFF Mask            */

#define SYS_SRAMBFF_DMICBFF_Pos          (16)                                              /*!< SYS_T::SRAMBFF: DMICBFF Position       */
#define SYS_SRAMBFF_DMICBFF_Msk          (0x1ul << SYS_SRAMBFF_DMICBFF_Pos)                /*!< SYS_T::SRAMBFF: DMICBFF Mask           */

#define SYS_SRAMBFF_EMACBFF_Pos          (17)                                              /*!< SYS_T::SRAMBFF: EMACBFF Position       */
#define SYS_SRAMBFF_EMACBFF_Msk          (0x1ul << SYS_SRAMBFF_EMACBFF_Pos)                /*!< SYS_T::SRAMBFF: EMACBFF Mask           */

#define SYS_SRAMBFF_NPUBFF_Pos           (18)                                              /*!< SYS_T::SRAMBFF: NPUBFF Position        */
#define SYS_SRAMBFF_NPUBFF_Msk           (0x1ul << SYS_SRAMBFF_NPUBFF_Pos)                 /*!< SYS_T::SRAMBFF: NPUBFF Mask            */

#define SYS_SRAMBFF_USBDBFF_Pos          (24)                                              /*!< SYS_T::SRAMBFF: USBDBFF Position       */
#define SYS_SRAMBFF_USBDBFF_Msk          (0x1ul << SYS_SRAMBFF_USBDBFF_Pos)                /*!< SYS_T::SRAMBFF: USBDBFF Mask           */

#define SYS_SRAMBFF_HSUSBDBFF_Pos        (25)                                              /*!< SYS_T::SRAMBFF: HSUSBDBFF Position     */
#define SYS_SRAMBFF_HSUSBDBFF_Msk        (0x1ul << SYS_SRAMBFF_HSUSBDBFF_Pos)              /*!< SYS_T::SRAMBFF: HSUSBDBFF Mask         */

#define SYS_SRAMBFF_HSUSBHBFF_Pos        (26)                                              /*!< SYS_T::SRAMBFF: HSUSBHBFF Position     */
#define SYS_SRAMBFF_HSUSBHBFF_Msk        (0x1ul << SYS_SRAMBFF_HSUSBHBFF_Pos)              /*!< SYS_T::SRAMBFF: HSUSBHBFF Mask         */

#define SYS_SRAMBRF_SR0BRF_Pos           (0)                                               /*!< SYS_T::SRAMBRF: SR0BRF Position        */
#define SYS_SRAMBRF_SR0BRF_Msk           (0x1ul << SYS_SRAMBRF_SR0BRF_Pos)                 /*!< SYS_T::SRAMBRF: SR0BRF Mask            */

#define SYS_SRAMBRF_SR1BRF_Pos           (1)                                               /*!< SYS_T::SRAMBRF: SR1BRF Position        */
#define SYS_SRAMBRF_SR1BRF_Msk           (0x1ul << SYS_SRAMBRF_SR1BRF_Pos)                 /*!< SYS_T::SRAMBRF: SR1BRF Mask            */

#define SYS_SRAMBRF_SR2BRF_Pos           (2)                                               /*!< SYS_T::SRAMBRF: SR2BRF Position        */
#define SYS_SRAMBRF_SR2BRF_Msk           (0x1ul << SYS_SRAMBRF_SR2BRF_Pos)                 /*!< SYS_T::SRAMBRF: SR2BRF Mask            */

#define SYS_SRAMBRF_SR3BRF_Pos           (3)                                               /*!< SYS_T::SRAMBRF: SR3BRF Position        */
#define SYS_SRAMBRF_SR3BRF_Msk           (0x1ul << SYS_SRAMBRF_SR3BRF_Pos)                 /*!< SYS_T::SRAMBRF: SR3BRF Mask            */

#define SYS_SRAMBRF_LPSRBRF_Pos          (4)                                               /*!< SYS_T::SRAMBRF: LPSRBRF Position       */
#define SYS_SRAMBRF_LPSRBRF_Msk          (0x1ul << SYS_SRAMBRF_LPSRBRF_Pos)                /*!< SYS_T::SRAMBRF: LPSRBRF Mask           */

#define SYS_SRAMBRF_CAN0BRF_Pos          (8)                                               /*!< SYS_T::SRAMBRF: CAN0BRF Position       */
#define SYS_SRAMBRF_CAN0BRF_Msk          (0x1ul << SYS_SRAMBRF_CAN0BRF_Pos)                /*!< SYS_T::SRAMBRF: CAN0BRF Mask           */

#define SYS_SRAMBRF_CAN1BRF_Pos          (9)                                               /*!< SYS_T::SRAMBRF: CAN1BRF Position       */
#define SYS_SRAMBRF_CAN1BRF_Msk          (0x1ul << SYS_SRAMBRF_CAN1BRF_Pos)                /*!< SYS_T::SRAMBRF: CAN1BRF Mask           */

#define SYS_SRAMBRF_SPIM0BRF_Pos         (10)                                              /*!< SYS_T::SRAMBRF: SPIM0BRF Position      */
#define SYS_SRAMBRF_SPIM0BRF_Msk         (0x1ul << SYS_SRAMBRF_SPIM0BRF_Pos)               /*!< SYS_T::SRAMBRF: SPIM0BRF Mask          */

#define SYS_SRAMBRF_CCAPBRF_Pos          (12)                                              /*!< SYS_T::SRAMBRF: CCAPBRF Position       */
#define SYS_SRAMBRF_CCAPBRF_Msk          (0x1ul << SYS_SRAMBRF_CCAPBRF_Pos)                /*!< SYS_T::SRAMBRF: CCAPBRF Mask           */

#define SYS_SRAMBRF_I3CBRF_Pos           (13)                                              /*!< SYS_T::SRAMBRF: I3CBRF Position        */
#define SYS_SRAMBRF_I3CBRF_Msk           (0x1ul << SYS_SRAMBRF_I3CBRF_Pos)                 /*!< SYS_T::SRAMBRF: I3CBRF Mask            */

#define SYS_SRAMBRF_KSBRF_Pos            (14)                                              /*!< SYS_T::SRAMBRF: KSBRF Position         */
#define SYS_SRAMBRF_KSBRF_Msk            (0x1ul << SYS_SRAMBRF_KSBRF_Pos)                  /*!< SYS_T::SRAMBRF: KSBRF Mask             */

#define SYS_SRAMBRF_RSABRF_Pos           (15)                                              /*!< SYS_T::SRAMBRF: RSABRF Position        */
#define SYS_SRAMBRF_RSABRF_Msk           (0x1ul << SYS_SRAMBRF_RSABRF_Pos)                 /*!< SYS_T::SRAMBRF: RSABRF Mask            */

#define SYS_SRAMBRF_DMICBRF_Pos          (16)                                              /*!< SYS_T::SRAMBRF: DMICBRF Position       */
#define SYS_SRAMBRF_DMICBRF_Msk          (0x1ul << SYS_SRAMBRF_DMICBRF_Pos)                /*!< SYS_T::SRAMBRF: DMICBRF Mask           */

#define SYS_SRAMBRF_EMACBRF_Pos          (17)                                              /*!< SYS_T::SRAMBRF: EMACBRF Position       */
#define SYS_SRAMBRF_EMACBRF_Msk          (0x1ul << SYS_SRAMBRF_EMACBRF_Pos)                /*!< SYS_T::SRAMBRF: EMACBRF Mask           */

#define SYS_SRAMBRF_NPUBRF_Pos           (18)                                              /*!< SYS_T::SRAMBRF: NPUBRF Position        */
#define SYS_SRAMBRF_NPUBRF_Msk           (0x1ul << SYS_SRAMBRF_NPUBRF_Pos)                 /*!< SYS_T::SRAMBRF: NPUBRF Mask            */

#define SYS_SRAMBRF_USBDBRF_Pos          (24)                                              /*!< SYS_T::SRAMBRF: USBDBRF Position       */
#define SYS_SRAMBRF_USBDBRF_Msk          (0x1ul << SYS_SRAMBRF_USBDBRF_Pos)                /*!< SYS_T::SRAMBRF: USBDBRF Mask           */

#define SYS_SRAMBRF_HSUSBDBRF_Pos        (25)                                              /*!< SYS_T::SRAMBRF: HSUSBDBRF Position     */
#define SYS_SRAMBRF_HSUSBDBRF_Msk        (0x1ul << SYS_SRAMBRF_HSUSBDBRF_Pos)              /*!< SYS_T::SRAMBRF: HSUSBDBRF Mask         */

#define SYS_SRAMBRF_HSUSBHBRF_Pos        (26)                                              /*!< SYS_T::SRAMBRF: HSUSBHBRF Position     */
#define SYS_SRAMBRF_HSUSBHBRF_Msk        (0x1ul << SYS_SRAMBRF_HSUSBHBRF_Pos)              /*!< SYS_T::SRAMBRF: HSUSBHBRF Mask         */

/** @} SYS_CONST */
/** @} end of SYS register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __SYS_REG_H__ */
