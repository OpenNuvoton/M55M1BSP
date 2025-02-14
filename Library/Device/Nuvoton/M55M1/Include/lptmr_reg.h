/**************************************************************************//**
 * @file     lptmr_reg.h
 * @version  V1.00
 * @brief    LPTMR register definition header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __LPTMR_REG_H__
#define __LPTMR_REG_H__

#if defined ( __CC_ARM   )
    #pragma anon_unions
#endif

/**
    @addtogroup REGISTER Control Register
    @{
*/

/**
    @addtogroup LPTMR Low Power Timer Controller (LPTMR)
    Memory Mapped Structure for LPTMR Controller
    @{
*/

typedef struct
{
    /**
     * @var LPTMR_T::CTL
     * Offset: 0x00  LPTMR Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PSC       |Prescale Counter
     * |        |          |Timer input clock or event source is divided by (PSC+1) before it is fed to the timer up counter
     * |        |          |If this field is 0 (PSC = 0), then there is no scaling.
     * |        |          |Note: Update prescale counter value will reset internal 8-bit prescale counter and 24-bit up counter value.
     * |[15]    |FUNCSEL   |Function Selection
     * |        |          |0 = Timer controller is used as timer function.
     * |        |          |1 = Timer controller is used as PWM function.
     * |[16]    |PDCLKEN   |Power-down Engine Clock Enable
     * |        |          |0 = Disable engine clock in power-down mode.
     * |        |          |1 = Enable engine clock in power-down mode.
     * |[20]    |PERIOSEL  |Periodic Mode Behavior Selection Enable Bit
     * |        |          |0 = The behavior selection in periodic mode is Disabled.
     * |        |          |When user updates CMPDAT while timer is running in periodic mode,
     * |        |          |CNT will be reset to default value.
     * |        |          |1 = The behavior selection in periodic mode is Enabled.
     * |        |          |When user update CMPDAT while timer is running in periodic mode, the limitations as bellows list,
     * |        |          |If updated CMPDAT value > CNT, CMPDAT will be updated and CNT keep running continually.
     * |        |          |If updated CMPDAT value = CNT, timer time-out interrupt will be asserted immediately.
     * |        |          |If updated CMPDAT value < CNT, CNT will be reset to default value.
     * |[21]    |TGLPINSEL |Toggle-output Pin Select
     * |        |          |0 = Toggle mode output to TMx (Timer Event Counter Pin).
     * |        |          |1 = Toggle mode output to TMx_EXT (Timer External Capture Pin).
     * |[22]    |CAPSRC    |Capture Pin Source Selection
     * |        |          |0 = Capture Function source is from TMx_EXT (x= 0~3) pin.
     * |        |          |1 = Capture Function source is from internal ACMP output signal
     * |        |          |User can set ACMPSSEL (TIMERx_EXTCTL[8]) to decide which internal ACMP output signal as timer capture source.
     * |[23]    |WKEN      |Wake-up Function Enable Bit
     * |        |          |If this bit is set to 1, while timer interrupt flag TIF (TIMERx_INTSTS[0]) is 1 and INTEN (TIMERx_CTL[29]) is enabled, the timer interrupt signal will generate a wake-up trigger event to CPU.
     * |        |          |0 = Wake-up function Disabled if timer interrupt signal generated.
     * |        |          |1 = Wake-up function Enabled if timer interrupt signal generated.
     * |[24]    |EXTCNTEN  |Event Counter Mode Enable Bit
     * |        |          |This bit is for external counting pin function enabled.
     * |        |          |0 = Event counter mode Disabled.
     * |        |          |1 = Event counter mode Enabled.
     * |        |          |Note: When timer is used as an event counter, this bit should be set to 1 and select PCLK as timer clock source.
     * |[25]    |ACTSTS    |Timer Active Status Bit (Read Only)
     * |        |          |This bit indicates the 24-bit up counter status.
     * |        |          |0 = 24-bit up counter is not active.
     * |        |          |1 = 24-bit up counter is active.
     * |        |          |Note: This bit may active when CNT 0 transition to CNT 1.
     * |[28:27] |OPMODE    |Timer Counting Mode Select
     * |        |          |00 = The Timer controller is operated in One-shot mode.
     * |        |          |01 = The Timer controller is operated in Periodic mode.
     * |        |          |10 = The Timer controller is operated in Toggle-output mode.
     * |        |          |11 = The Timer controller is operated in Continuous Counting mode.
     * |[29]    |INTEN     |Timer Interrupt Enable Bit
     * |        |          |0 = Timer time-out interrupt Disabled.
     * |        |          |1 = Timer time-out interrupt Enabled.
     * |        |          |Note: If this bit is enabled, when the timer time-out interrupt flag TIF is set to 1, the timer interrupt signal is generated and inform to CPU.
     * |[30]    |CNTEN     |Timer Counting Enable Bit
     * |        |          |0 = Stops/Suspends counting.
     * |        |          |1 = Starts counting.
     * |        |          |Note1: In stop status, and then set CNTEN to 1 will enable the 24-bit up counter to keep counting from the last stop counting value.
     * |        |          |Note2: This bit is auto-cleared by hardware in one-shot mode (TIMER_CTL[28:27] = 00) when the timer time-out interrupt flag TIF (TIMERx_INTSTS[0]) is generated.
     * |        |          |Note3: Set enable/disable this bit needs 2 * TMR_CLK period to become active, user can read ACTSTS (TIMERx_CTL[25]) to check enable/disable command is completed or not.
     * |[31]    |ICEDEBUG  |ICE Debug Mode Acknowledge Disable Control (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgement effects TIMER counting.
     * |        |          |TIMER counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgement Disabled.
     * |        |          |TIMER counter will keep going no matter CPU is held by ICE or not.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var LPTMR_T::CMP
     * Offset: 0x04  LPTMR Comparator Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CMPDAT    |Timer Comparator Value
     * |        |          |CMPDAT is a 24-bit compared value register
     * |        |          |When the internal 24-bit up counter value is equal to CMPDAT value, the TIF (TIMERx_INTSTS[0] Timer Interrupt Flag) will set to 1.
     * |        |          |Time-out period = (Period of timer clock input) * (8-bit PSC + 1) * (24-bit CMPDAT).
     * |        |          |Note1: Never write 0x0 or 0x1 in CMPDAT field, or the core will run into unknown state.
     * |        |          |Note2: When timer is operating at continuous counting mode, the 24-bit up counter will keep counting continuously even if user writes a new value into CMPDAT field
     * |        |          |But if timer is operating at other modes, the 24-bit up counter will restart counting from 0 and using newest CMPDAT value to be the timer compared value while user writes a new value into CMPDAT field.
     * @var LPTMR_T::INTSTS
     * Offset: 0x08  LPTMR Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TIF       |Timer Interrupt Flag
     * |        |          |This bit indicates the interrupt flag status of Timer while 24-bit timer up counter CNT (TIMERx_CNT[23:0]) value reaches to CMPDAT (TIMERx_CMP[23:0]) value.
     * |        |          |0 = No effect.
     * |        |          |1 = CNT value matches the CMPDAT value.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[1]     |TWKF      |Timer Wake-up Flag
     * |        |          |This bit indicates the interrupt wake-up flag status of timer.
     * |        |          |0 = Timer does not cause CPU wake-up.
     * |        |          |1 = CPU wake-up from Idle or Power-down mode if timer time-out interrupt signal generated.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * @var LPTMR_T::CNT
     * Offset: 0x0C  LPTMR Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CNT       |Timer Data Register
     * |        |          |Read operation.
     * |        |          |Read this register to get CNT value. For example:
     * |        |          |If EXTCNTEN (TIMERx_CTL[24] ) is 0, user can read CNT value for getting current 24-bit counter value.
     * |        |          |If EXTCNTEN (TIMERx_CTL[24] ) is 1, user can read CNT value for getting current 24-bit event input counter value.
     * |        |          |Write operation.
     * |        |          |Writing any value to this register will reset current CNT value to 0 and reload internal 8-bit prescale counter.
     * |[31]    |RSTACT    |Timer Data Register Reset Active (Read Only)
     * |        |          |This bit indicates if the counter reset operation active.
     * |        |          |When user writes this CNT register, timer starts to reset its internal 24-bit timer up-counter to 0 and reload 8-bit pre-scale counter
     * |        |          |At the same time, timer set this flag to 1 to indicate the counter reset operation is in progress
     * |        |          |Once the counter reset operation done, timer clear this bit to 0 automatically.
     * |        |          |0 = Reset operation is done.
     * |        |          |1 = Reset operation triggered by writing TIMERx_CNT is in progress.
     * |        |          |Note: This bit is read only.
     * @var LPTMR_T::CAP
     * Offset: 0x10  LPTMR Capture Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CAPDAT    |Timer Capture Data Register
     * |        |          |When CAPEN (TIMERx_EXTCTL[3]) bit is set, CAPFUNCS (TIMERx_EXTCTL[4]) bit is 0, and a transition on TMx_EXT pin matched the CAPEDGE (TIMERx_EXTCTL[14:12]) setting, CAPIF (TIMERx_EINTSTS[0]) will set to 1 and the current timer counter value CNT (TIMERx_CNT[23:0]) will be auto-loaded into this CAPDAT field.
     * @var LPTMR_T::EXTCTL
     * Offset: 0x14  LPTMR External Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTPHASE  |Timer External Count Phase
     * |        |          |This bit indicates the detection phase of external counting pin TMx (x= 0~3).
     * |        |          |0 = A falling edge of external counting pin will be counted.
     * |        |          |1 = A rising edge of external counting pin will be counted.
     * |[3]     |CAPEN     |Timer External Capture Pin Enable Bit
     * |        |          |This bit enables the TMx_EXT capture pin input function.
     * |        |          |0 =TMx_EXT (x= 0~3) pin Disabled.
     * |        |          |1 =TMx_EXT (x= 0~3) pin Enabled.
     * |[4]     |CAPFUNCS  |Capture Function Selection
     * |        |          |0 = External Capture Mode Enabled.
     * |        |          |1 = External Reset Mode Enabled.
     * |        |          |Note1: When CAPFUNCS is 0, transition on TMx_EXT (x= 0~3) pin is using to save current 24-bit timer counter value (CNT value) to CAPDAT field.
     * |        |          |Note2: When CAPFUNCS is 1, transition on TMx_EXT (x= 0~3) pin is using to save current 24-bit timer counter value (CNT value) to CAPDAT field then CNT value will be reset immediately.
     * |[5]     |CAPIEN    |Timer External Capture Interrupt Enable Bit
     * |        |          |0 = TMx_EXT (x= 0~3) pin detection Interrupt Disabled.
     * |        |          |1 = TMx_EXT (x= 0~3) pin detection Interrupt Enabled.
     * |        |          |Note: CAPIEN is used to enable timer external interrupt
     * |        |          |If CAPIEN enabled, timer will rise an interrupt when CAPIF (TIMERx_EINTSTS[0]) is 1.
     * |        |          |For example, while CAPIEN = 1, CAPEN = 1, and CAPEDGE = 00, a 1 to 0 transition on the TMx_EXT pin will cause the CAPIF to be set then the interrupt signal is generated and sent to NVIC to inform CPU.
     * |[6]     |CAPDBEN   |Timer External Capture Pin De-bounce Enable Bit
     * |        |          |0 = TMx_EXT (x= 0~3) pin de-bounce or ACMP output de-bounce Disabled.
     * |        |          |1 = TMx_EXT (x= 0~3) pin de-bounce or ACMP output de-bounce Enabled.
     * |        |          |Note: If this bit is enabled, the edge detection of TMx_EXT pin or ACMP output is detected with de-bounce circuit.
     * |[7]     |CNTDBEN   |Timer Counter Pin De-bounce Enable Bit
     * |        |          |0 = TMx (x= 0~3) pin de-bounce Disabled.
     * |        |          |1 = TMx (x= 0~3) pin de-bounce Enabled.
     * |        |          |Note: If this bit is enabled, the edge detection of TMx pin is detected with de-bounce circuit.
     * |[11:8]  |ICAPSEL   |Internal Capture Source Select
     * |        |          |0000 = Capture Function source is from internal ACMP0 output signal.
     * |        |          |0001 = Capture Function source is from internal ACMP1 output signal.
     * |        |          |0111 = Capture Function source is from internal ACMP2 output signal.
     * |        |          |1000 = Capture Function source is from internal ACMP3 output signal.
     * |        |          |Note: these bits only available when CAPSRC (TIMERx_CTL[22]) is 1.
     * |[14:12] |CAPEDGE   |Timer External Capture Pin Edge Detect
     * |        |          |When first capture event is generated, the CNT (TIMERx_CNT[23:0]) will be reset to 0 and first CAPDAT (TIMERx_CAP[23:0]) should be to 0.
     * |        |          |000 = Capture event occurred when detect falling edge transfer on TMx_EXT (x= 0~3) pin.
     * |        |          |001 = Capture event occurred when detect rising edge transfer on TMx_EXT (x= 0~3) pin.
     * |        |          |010 = Capture event occurred when detect both falling and rising edge transfer on TMx_EXT (x= 0~3) pin, and first capture event occurred at falling edge transfer.
     * |        |          |011 = Capture event occurred when detect both rising and falling edge transfer on TMx_EXT (x= 0~3) pin, and first capture event occurred at rising edge transfer..
     * |        |          |110 = First capture event occurred at falling edge, follows capture events are at rising edge transfer on TMx_EXT (x= 0~3) pin.
     * |        |          |111 = First capture event occurred at rising edge, follows capture events are at falling edge transfer on TMx_EXT (x= 0~3) pin.
     * |        |          |100, 101 = Reserved.
     * |[18:16] |ECNTSSEL  |Event Counter Source Selection to Trigger Event Counter Function
     * |        |          |000 = Event Counter input source is from TMx (x= 0~3) pin.
     * |        |          |010 = Event Counter input source is from internal ACMP0 output signal.
     * |        |          |011 = Event Counter input source is from internal ACMP1 output signal.
     * |        |          |100 = Event Counter input source is from internal ACMP2 output signal.
     * |        |          |101 = Event Counter input source is from internal ACMP3 output signal.
     * |[31:28] |CAPDIVSCL |Timer Capture Source Divider
     * |        |          |This bits indicate the divide scale for capture source divider
     * |        |          |0000 = Capture source/1.
     * |        |          |0001 = Capture source/2.
     * |        |          |0010 = Capture source/4.
     * |        |          |0011 = Capture source/8.
     * |        |          |0100 = Capture source/16.
     * |        |          |0101 = Capture source/32.
     * |        |          |0110 = Capture source/64.
     * |        |          |0111 = Capture source/128.
     * |        |          |1000 = Capture source/256.
     * |        |          |1001~1111 = Reserved.
     * |        |          |Note: Sets INTERCAPSEL (TIMERx_EXTCTL[10:8]) and CAPSRC (TIMERx_CTL[22]) to select capture source.     * @var LPTMR_T::EINTSTS
     * Offset: 0x18  LPTMR External Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CAPIF     |Timer External Capture Interrupt Flag
     * |        |          |This bit indicates the timer external capture interrupt flag status.
     * |        |          |0 = TMx_EXT (x= 0~3) pin interrupt did not occur.
     * |        |          |1 = TMx_EXT (x= 0~3) pin interrupt occurred.
     * |        |          |Note1: This bit is cleared by writing 1 to it.
     * |        |          |Note2: When CAPEN (TIMERx_EXTCTL[3]) bit is set, CAPFUNCS (TIMERx_EXTCTL[4]) bit is 0, and a transition on TMx_EXT (x= 0~3) pin matched the CAPEDGE (TIMERx_EXTCTL[2:1]) setting, this bit will set to 1 by hardware.
     * |        |          |Note3: There is a new incoming capture event detected before CPU clearing the CAPIF status
     * |        |          |If the above condition occurred, the Timer will keep register TIMERx_CAP unchanged and drop the new capture value.
     * @var LPTMR_T::TRGCTL
     * Offset: 0x1C  LPTMR Trigger Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TRGSSEL   |Trigger Source Select Bit
     * |        |          |This bit is used to select internal trigger source is form timer time-out interrupt signal or
     * |        |          |capture interrupt signal.
     * |        |          |0 = Time-out interrupt signal is used to internal trigger LPADC0, LPI2C0, LPSPI0, LPUART, CCAP Motion detector and LPPDMA.
     * |        |          |1 = Capture interrupt signal is used to internal trigger LPADC0, LPI2C0, LPSPI0, LPUART, CCAP Motion detector and LPPDMA.
     * |[1]     |TRGEN     |Trigger Low power IPs Enable Bit
     * |        |          |If this bit is set to 1, each timer time-out event or capture event can be triggered Low Power IPs conversion.
     * |        |          |0 = Timer interrupt trigger Low Power IPs Disabled.
     * |        |          |1 = Timer interrupt trigger Low Power IPs Enabled.
     * |[4]     |TRGLPPDMA |Trigger LPPDMA Enable Bit
     * |        |          |If this bit is set to 1, each timer time-out event or capture event can be triggered LPPDMA transfer.
     * |        |          |0 = Timer interrupt trigger LPPDMA Disabled.
     * |        |          |1 = Timer interrupt trigger LPPDMA Enabled.
     * @var LPTMR_T::CAPNF
     * Offset: 0x24  LPTMR Capture Input Noise Filter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CAPNFEN   |Capture Noise Filter Enable
     * |        |          |0 = Capture Noise Filter function Disabled.
     * |        |          |1 = Capture Noise Filter function Enabled.
     * |[6:4]   |CAPNFSEL  |Capture Edge Detector Noise Filter Clock Selection
     * |        |          |000 = Noise filter clock is PCLKx.
     * |        |          |001 = Noise filter clock is PCLKx/2.
     * |        |          |010 = Noise filter clock is PCLKx/4.
     * |        |          |011 = Noise filter clock is PCLKx/8.
     * |        |          |100 = Noise filter clock is PCLKx/16.
     * |        |          |101 = Noise filter clock is PCLKx/32.
     * |        |          |110 = Noise filter clock is PCLKx/64.
     * |        |          |111 = Noise filter clock is PCLKx/128.
     * |[10:8]  |CAPNFCNT  |Capture Edge Detector Noise Filter Count
     * |        |          |These bits control the capture filter counter to count from 0 to CAPNFCNT.
     * @var LPTMR_T::PWMCTL
     * Offset: 0x40  LPTMR PWM Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTEN     |PWM Counter Enable Bit
     * |        |          |0 = PWM counter and clock prescale Stop Running.
     * |        |          |1 = PWM counter and clock prescale Start Running.
     * |[3]     |CNTMODE   |PWM Counter Mode
     * |        |          |0 = Auto-reload mode.
     * |        |          |1 = One-shot mode.
     * |[12]    |PWMINTWKEN|PWM Interrupt Wake-up Enable Bit
     * |        |          |If PWM interrupt occurs when chip is in Power-down mode, PWMINTWKEN can determine whether chip wake-up occurs or not.
     * |        |          |0 = PWM interrupt wake-up Disabled.
     * |        |          |1 = PWM interrupt wake-up Enabled.
     * |[30]    |DBGHALT   |ICE Debug Mode Counter Halt (Write Protect)
     * |        |          |If debug mode counter halt is enabled, PWM counter will keep current value until exit ICE debug mode.
     * |        |          |0 = ICE debug mode counter halt disable.
     * |        |          |1 = ICE debug mode counter halt enable.
     * |        |          |Note: This register is write protected. Refer toSYS_REGLCTL register.
     * |[31]    |DBGTRIOFF |ICE Debug Mode Acknowledge Disable Bit (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgement effects PWM output.
     * |        |          |PWM output pin will be forced as tri-state while ICE debug mode acknowledged.
     * |        |          |1 = ICE debug mode acknowledgement disabled.
     * |        |          |PWM output pin will keep output no matter ICE debug mode acknowledged or not.
     * |        |          |Note: This register is write protected. Refer toSYS_REGLCTL register.
     * @var LPTMR_T::PWMCLKPSC
     * Offset: 0x44  LPTMR PWM Counter Clock Pre-scale Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CLKPSC    |PWM Counter Clock Pre-scale
     * |        |          |The active clock of PWM counter is decided by counter clock prescale and divided by (CLKPSC + 1)
     * |        |          |If CLKPSC is 0, then there is no scaling in PWM counter clock source.
     * @var LPTMR_T::PWMCNTCLR
     * Offset: 0x48  LPTMR PWM Clear Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTCLR    |Clear PWM Counter Control Bit
     * |        |          |It is automatically cleared by hardware.
     * |        |          |0 = No effect.
     * |        |          |1 = Clear 16-bit PWM counter to 0x10000 in up and up-down count type and reset counter value to PERIOD in down count type.
     * @var LPTMR_T::PWMPERIOD
     * Offset: 0x4C  LPTMR PWM Period Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD    |PWM Period Register
     * |        |          |In up count type: PWM counter counts from 0 to PERIOD, and restarts from 0.
     * |        |          |In down count type: PWM counter counts from PERIOD to 0, and restarts from PERIOD.
     * |        |          |In up-down count type: PWM counter counts from 0 to PERIOD, then decrements to 0 and repeats again.
     * |        |          |In up and down count type:
     * |        |          |PWM period time = (PERIOD + 1) * (CLKPSC + 1) * TMRx_PWMCLK.
     * |        |          |In up-down count type:
     * |        |          |PWM period time = 2 * PERIOD * (CLKPSC+ 1) * TMRx_PWMCLK.
     * |        |          |Note: User should take care DIRF (TIMERx_PWMCNT[16]) bit in up/down/up-down count type to monitor current counter direction in each count type.
     * @var LPTMR_T::PWMCMPDAT
     * Offset: 0x50  LPTMR PWM Comparator Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMP       |PWM Comparator Register
     * |        |          |PWM CMP is used to compare with PWM CNT to generate PWM output waveform, interrupt events and trigger ADC to start convert.
     * @var LPTMR_T::PWMCNT
     * Offset: 0x54  LPTMR PWM Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CNT       |PWM Counter Value Register (Read Only)
     * |        |          |User can monitor CNT to know the current counter value in 16-bit period counter.
     * @var LPTMR_T::PWMPOLCTL
     * Offset: 0x58  LPTMR PWM Pin Output Polar Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PINV      |PWMx Output Pin Polar Control Bit
     * |        |          |The bit is used to control polarity state of PWMx output pin.
     * |        |          |0 = PWMx output pin polar inverse Disabled.
     * |        |          |1 = PWMx output pin polar inverse Enabled.
     * @var LPTMR_T::PWMPOCTL
     * Offset: 0x5C  LPTMR PWM Pin Output Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |POEN      |PWMx Output Pin Enable Bit
     * |        |          |0 = PWMx Output pin at tri-state mode.
     * |        |          |1 = PWMx Output pin in output mode.
     * |[8]     |POSEL     |PWM Output Pin Select
     * |        |          |0 = PWMx_OUT pin is LPTMx.
     * |        |          |1 = PWMx_OUT pin is LPTMx_EXT.
     * @var LPTMR_T::PWMINTEN0
     * Offset: 0x60  LPTMR PWM Interrupt Enable Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |PIEN      |PWM Period Point Interrupt Enable Bit
     * |        |          |0 = Period point interrupt Disabled.
     * |        |          |1 = Period point interrupt Enabled.
     * |        |          |Note: When in up-down count type, period point means the center point of current PWM period.
     * |[2]     |CMPUIEN   |PWM Compare Up Count Interrupt Enable Bit
     * |        |          |0 = Compare up count interrupt Disabled.
     * |        |          |1 = Compare up count interrupt Enabled.
     * @var LPTMR_T::PWMINTSTS0
     * Offset: 0x64  LPTMR PWM Interrupt Status Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |PIF       |PWM Period Point Interrupt Flag
     * |        |          |This bit is set by hardware when TIMERx_PWM counter reaches PERIOD.
     * |        |          |Note1: When in up-down count type, PIF flag means the center point flag of current PWM period.
     * |        |          |Note2: This bit is cleared by writing 1 to it.
     * |[2]     |CMPUIF    |PWM Compare Up Count Interrupt Flag
     * |        |          |This bit is set by hardware when TIMERx_PWM counter in up count direction and reaches CMP.
     * |        |          |Note1: If CMP equal to PERIOD, there is no CMPUIF flag in up count type and up-down count type.
     * |        |          |Note2: This bit is cleared by writing 1 to it.
     * @var LPTMR_T::PWMTRGCTL
     * Offset: 0x68  LPTMR PWM Trigger Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |TRGSEL    |PWM Counter Event Source Select to Trigger Conversion
     * |        |          |00 = Trigger conversion at period point (PIF).
     * |        |          |01 = Trigger conversion at compare up count point (CMPUIF).
     * |        |          |10 = Trigger conversion at period or compare up count point (PIF or CMPUIF).
     * |        |          |11 = Reserved.
     * |[2]     |TRGEN     |Trigger Low power IPs Enable Bit
     * |        |          |0 = Low Power Timer PWM counter event trigger Low Power IPs Disabled.
     * |        |          |1 = Low Power Timer PWM counter event trigger Low Power IPs Enabled.
     * |[9]     |PWMTRGLPPDMA|PWM Counter Event Trigger LPPDMA Conversion Enable Bit
     * |        |          |0 = PWM counter event trigger LPPDMA conversion Disabled.
     * |        |          |1 = PWM counter event trigger LPPDMA conversion Enabled.
     * |[10]    |AEPDMA    |Accumulator Every IFACNT LPPDMA Mode Enable Bit
     * |        |          |0 = Trigger LPPDMA Accumulator Mode Disabled.
     * |        |          |1 = Trigger LPPDMA Accumulator Mode Enabled.
     * @var LPTMR_T::PWMSTATUS
     * Offset: 0x6C  LPTMR PWM Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTMAXF   |PWM Counter Equal to 0xFFFF Flag
     * |        |          |0 = Indicates the PWM counter value never reached its maximum value 0xFFFF.
     * |        |          |1 = Indicates the PWM counter value has reached its maximum value.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[16]    |PWMINTWKF |PWM Interrupt Wake-up Flag
     * |        |          |0 = PWM interrupt wake-up has not occurred.
     * |        |          |1 = PWM interrupt wake-up has occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * @var LPTMR_T::PWMPBUF
     * Offset: 0x70  LPTMR PWM Period Buffer Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PBUF      |PWM Period Buffer Register (Read Only)
     * |        |          |Used as PERIOD active register.
     * @var LPTMR_T::PWMCMPBUF
     * Offset: 0x74  LPTMR PWM Comparator Buffer Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMPBUF    |PWM Comparator Buffer Register (Read Only)
     * |        |          |Used as CMP active register.
     * @var LPTMR_T::PWMIFA
     * Offset: 0xA8  LPTMR PWM Interrupt Flag Accumulator Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |IFACNT    |PWM Interrupt Flag Accumulator Counter
     * |        |          |This field sets the count number which defines (IFACNT+1) times of specify PWM interrupt occurs to set IFAIF bit to request the PWM accumulator interrupt.
     * |        |          |PWM accumulator flag (IFAIF) will be set in every (IFACNT+1) times during the the PWM counter operation.
     * |[24]    |STPMOD    |PWM Accumulator Stop Mode Enable Bit
     * |        |          |0 = PWM interrupt accumulator event to stop counting Disabled.
     * |        |          |1 = PWM interrupt accumulator event to stop counting Enabled.
     * |[29:28] |IFASEL    |PWM Interrupt Flag Accumulator Source Select
     * |        |          |00 = Accumulate at each PWM zero point.
     * |        |          |01 = Accumulate at each PWM period point.
     * |        |          |10 = Accumulate at each PWM up-count compared point.
     * |        |          |11 = Accumulate at each PWM down-count compared point.
     * |[31]    |IFAEN     |PWM Interrupt Flag Accumulator Enable Bit
     * |        |          |0 = PWM interrupt flag accumulator function Disabled.
     * |        |          |1 = PWM interrupt flag accumulator function Enabled.
     * @var LPTMR_T::PWMAINTSTS
     * Offset: 0xAC  LPTMR PWM Accumulator Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |IFAIF     |PWM Interrupt Flag Accumulator Interrupt Flag
     * |        |          |This bit is set by hardware when the accumulator value reaches (IFACNT+1).
     * |        |          |Note 1: This bit is cleared by writing 1 to it.
     * |        |          |Note 2: If APDMAEN (TIMERx_PWMAPDMACTL[0]) is set, this bit will be auto clear after PDMA transfer done.
     * @var LPTMR_T::PWMAINTEN
     * Offset: 0xB0  LPTMR PWM Accumulator Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |IFAIEN    |PWM Interrupt Flag Accumulator Interrupt Enable Bit
     * |        |          |0 = Interrupt Flag Accumulator interrupt Disabled.
     * |        |          |1 = Interrupt Flag Accumulator interrupt Enabled.
     * @var LPTMR_T::PWMAPDMACTL
     * Offset: 0xB4  LPTMR PWM Accumulator PDMA Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |APDMAEN   |PWM Accumulator PDMA Enable Bit
     * |        |          |0 = PWM interrupt accumulator event to trigger PDMA transfer Disabled.
     * |        |          |1 = PWM interrupt accumulator event to trigger PDMA transfer Enabled.
     */
    __IO uint32_t CTL;              /*!< [0x00] LPTMRx Control Register                                 */
    __IO uint32_t CMP;              /*!< [0x04] LPTMRx Comparator Register                              */
    __IO uint32_t INTSTS;           /*!< [0x08] LPTMRx Interrupt Status Register                        */
    __IO uint32_t CNT;              /*!< [0x0C] LPTMRx Data Register                                    */
    __I  uint32_t CAP;              /*!< [0x10] LPTMRx Capture Data Register                            */
    __IO uint32_t EXTCTL;           /*!< [0x14] LPTMRx External Control Register                        */
    __IO uint32_t EINTSTS;          /*!< [0x18] LPTMRx External Interrupt Status Register               */
    __IO uint32_t TRGCTL;           /*!< [0x1C] LPTMRx Trigger Control Register                         */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t CAPNF;            /*!< [0x24] LPTMRx Capture Input Noise Filter Register              */
    __I  uint32_t RESERVE1[6];
    __IO uint32_t PWMCTL;           /*!< [0x40] LPTMRx PWM Control Register                             */
    __IO uint32_t PWMCLKPSC;        /*!< [0x44] LPTMRx PWM Counter Clock Pre-scale Register             */
    __IO uint32_t PWMCNTCLR;        /*!< [0x48] LPTMRx PWM Clear Counter Register                       */
    __IO uint32_t PWMPERIOD;        /*!< [0x4C] LPTMRx PWM Period Register                              */
    __IO uint32_t PWMCMPDAT;        /*!< [0x50] LPTMRx PWM Comparator Register                          */
    __I  uint32_t PWMCNT;           /*!< [0x54] LPTMRx PWM Counter Register                             */
    __IO uint32_t PWMPOLCTL;        /*!< [0x58] LPTMRx PWM Pin Output Polar Control Register            */
    __IO uint32_t PWMPOCTL;         /*!< [0x5C] LPTMRx PWM Pin Output Control Register                  */
    __IO uint32_t PWMINTEN0;        /*!< [0x60] LPTMRx PWM Interrupt Enable Register 0                  */
    __IO uint32_t PWMINTSTS0;       /*!< [0x64] LPTMRx PWM Interrupt Status Register 0                  */
    __IO uint32_t PWMTRGCTL;        /*!< [0x68] LPTMRx PWM Trigger Control Register                     */
    __IO uint32_t PWMSTATUS;        /*!< [0x6C] LPTMRx PWM Status Register                              */
    __I  uint32_t PWMPBUF;          /*!< [0x70] LPTMRx PWM Period Buffer Register                       */
    __I  uint32_t PWMCMPBUF;        /*!< [0x74] LPTMRx PWM Comparator Buffer Register                   */
    __I  uint32_t RESERVE2[12];
    __IO uint32_t PWMIFA;           /*!< [0xA8] LPTMRx PWM Interrupt Flag Accumulator Register          */
    __IO uint32_t PWMAINTSTS;       /*!< [0xAC] LPTMRx PWM Accumulator Interrupt Flag Register          */
    __IO uint32_t PWMAINTEN;        /*!< [0xB0] LPTMRx PWM Accumulator Interrupt Enable Register        */
    __IO uint32_t PWMAPDMACTL;      /*!< [0xB4] LPTMRx PWM Accumulator LPPDMA Control Register          */
} LPTMR_T;

/**
    @addtogroup LPTMR_CONST TIMER Bit Field Definition
    Constant Definitions for LPTMR Controller
@{ */

#define LPTMR_CTL_PSC_Pos               (0)                                         /*!< LPTMR_T::CTL: PSC Position                 */
#define LPTMR_CTL_PSC_Msk               (0xffUL << LPTMR_CTL_PSC_Pos)               /*!< LPTMR_T::CTL: PSC Mask                     */

#define LPTMR_CTL_FUNCSEL_Pos           (15)                                        /*!< LPTMR_T::CTL: FUNCSEL Position             */
#define LPTMR_CTL_FUNCSEL_Msk           (0x1UL << LPTMR_CTL_FUNCSEL_Pos)            /*!< LPTMR_T::CTL: FUNCSEL Mask                 */

#define LPTMR_CTL_PDCLKEN_Pos           (16)                                        /*!< LPTMR_T::CTL: PDCLKEN Position             */
#define LPTMR_CTL_PDCLKEN_Msk           (0x1UL << LPTMR_CTL_PDCLKEN_Pos)            /*!< LPTMR_T::CTL: PDCLKEN Mask                 */

#define LPTMR_CTL_PERIOSEL_Pos          (20)                                        /*!< LPTMR_T::CTL: PERIOSEL Position            */
#define LPTMR_CTL_PERIOSEL_Msk          (0x1UL << LPTMR_CTL_PERIOSEL_Pos)           /*!< LPTMR_T::CTL: PERIOSEL Mask                */

#define LPTMR_CTL_TGLPINSEL_Pos         (21)                                        /*!< LPTMR_T::CTL: TGLPINSEL Position           */
#define LPTMR_CTL_TGLPINSEL_Msk         (0x1UL << LPTMR_CTL_TGLPINSEL_Pos)          /*!< LPTMR_T::CTL: TGLPINSEL Mask               */

#define LPTMR_CTL_CAPSRC_Pos            (22)                                        /*!< LPTMR_T::CTL: CAPSRC Position              */
#define LPTMR_CTL_CAPSRC_Msk            (0x1UL << LPTMR_CTL_CAPSRC_Pos)             /*!< LPTMR_T::CTL: CAPSRC Mask                  */

#define LPTMR_CTL_WKEN_Pos              (23)                                        /*!< LPTMR_T::CTL: WKEN Position                */
#define LPTMR_CTL_WKEN_Msk              (0x1UL << LPTMR_CTL_WKEN_Pos)               /*!< LPTMR_T::CTL: WKEN Mask                    */

#define LPTMR_CTL_EXTCNTEN_Pos          (24)                                        /*!< LPTMR_T::CTL: EXTCNTEN Position            */
#define LPTMR_CTL_EXTCNTEN_Msk          (0x1UL << LPTMR_CTL_EXTCNTEN_Pos)           /*!< LPTMR_T::CTL: EXTCNTEN Mask                */

#define LPTMR_CTL_ACTSTS_Pos            (25)                                        /*!< LPTMR_T::CTL: ACTSTS Position              */
#define LPTMR_CTL_ACTSTS_Msk            (0x1UL << LPTMR_CTL_ACTSTS_Pos)             /*!< LPTMR_T::CTL: ACTSTS Mask                  */

#define LPTMR_CTL_OPMODE_Pos            (27)                                        /*!< LPTMR_T::CTL: OPMODE Position              */
#define LPTMR_CTL_OPMODE_Msk            (0x3UL << LPTMR_CTL_OPMODE_Pos)             /*!< LPTMR_T::CTL: OPMODE Mask                  */

#define LPTMR_CTL_INTEN_Pos             (29)                                        /*!< LPTMR_T::CTL: INTEN Position               */
#define LPTMR_CTL_INTEN_Msk             (0x1UL << LPTMR_CTL_INTEN_Pos)              /*!< LPTMR_T::CTL: INTEN Mask                   */

#define LPTMR_CTL_CNTEN_Pos             (30)                                        /*!< LPTMR_T::CTL: CNTEN Position               */
#define LPTMR_CTL_CNTEN_Msk             (0x1UL << LPTMR_CTL_CNTEN_Pos)              /*!< LPTMR_T::CTL: CNTEN Mask                   */

#define LPTMR_CTL_ICEDEBUG_Pos          (31)                                        /*!< LPTMR_T::CTL: ICEDEBUG Position            */
#define LPTMR_CTL_ICEDEBUG_Msk          (0x1UL << LPTMR_CTL_ICEDEBUG_Pos)           /*!< LPTMR_T::CTL: ICEDEBUG Mask                */

#define LPTMR_CMP_CMPDAT_Pos            (0)                                         /*!< LPTMR_T::CMP: CMPDAT Position              */
#define LPTMR_CMP_CMPDAT_Msk            (0xffffffUL << LPTMR_CMP_CMPDAT_Pos)        /*!< LPTMR_T::CMP: CMPDAT Mask                  */

#define LPTMR_INTSTS_TIF_Pos            (0)                                         /*!< LPTMR_T::INTSTS: TIF Position              */
#define LPTMR_INTSTS_TIF_Msk            (0x1UL << LPTMR_INTSTS_TIF_Pos)             /*!< LPTMR_T::INTSTS: TIF Mask                  */

#define LPTMR_INTSTS_TWKF_Pos           (1)                                         /*!< LPTMR_T::INTSTS: TWKF Position             */
#define LPTMR_INTSTS_TWKF_Msk           (0x1UL << LPTMR_INTSTS_TWKF_Pos)            /*!< LPTMR_T::INTSTS: TWKF Mask                 */

#define LPTMR_CNT_CNT_Pos               (0)                                         /*!< LPTMR_T::CNT: CNT Position                 */
#define LPTMR_CNT_CNT_Msk               (0xffffffUL << LPTMR_CNT_CNT_Pos)           /*!< LPTMR_T::CNT: CNT Mask                     */

#define LPTMR_CNT_RSTACT_Pos            (31)                                        /*!< LPTMR_T::CNT: RSTACT Position              */
#define LPTMR_CNT_RSTACT_Msk            (0x1UL << LPTMR_CNT_RSTACT_Pos)             /*!< LPTMR_T::CNT: RSTACT Mask                  */

#define LPTMR_CAP_CAPDAT_Pos            (0)                                         /*!< LPTMR_T::CAP: CAPDAT Position              */
#define LPTMR_CAP_CAPDAT_Msk            (0xffffffUL << LPTMR_CAP_CAPDAT_Pos)        /*!< LPTMR_T::CAP: CAPDAT Mask                  */

#define LPTMR_EXTCTL_CNTPHASE_Pos       (0)                                         /*!< LPTMR_T::EXTCTL: CNTPHASE Position         */
#define LPTMR_EXTCTL_CNTPHASE_Msk       (0x1UL << LPTMR_EXTCTL_CNTPHASE_Pos)        /*!< LPTMR_T::EXTCTL: CNTPHASE Mask             */

#define LPTMR_EXTCTL_CAPEN_Pos          (3)                                         /*!< LPTMR_T::EXTCTL: CAPEN Position            */
#define LPTMR_EXTCTL_CAPEN_Msk          (0x1UL << LPTMR_EXTCTL_CAPEN_Pos)           /*!< LPTMR_T::EXTCTL: CAPEN Mask                */

#define LPTMR_EXTCTL_CAPFUNCS_Pos       (4)                                         /*!< LPTMR_T::EXTCTL: CAPFUNCS Position         */
#define LPTMR_EXTCTL_CAPFUNCS_Msk       (0x1UL << LPTMR_EXTCTL_CAPFUNCS_Pos)        /*!< LPTMR_T::EXTCTL: CAPFUNCS Mask             */

#define LPTMR_EXTCTL_CAPIEN_Pos         (5)                                         /*!< LPTMR_T::EXTCTL: CAPIEN Position           */
#define LPTMR_EXTCTL_CAPIEN_Msk         (0x1UL << LPTMR_EXTCTL_CAPIEN_Pos)          /*!< LPTMR_T::EXTCTL: CAPIEN Mask               */

#define LPTMR_EXTCTL_CAPDBEN_Pos        (6)                                         /*!< LPTMR_T::EXTCTL: CAPDBEN Position          */
#define LPTMR_EXTCTL_CAPDBEN_Msk        (0x1UL << LPTMR_EXTCTL_CAPDBEN_Pos)         /*!< LPTMR_T::EXTCTL: CAPDBEN Mask              */

#define LPTMR_EXTCTL_CNTDBEN_Pos        (7)                                         /*!< LPTMR_T::EXTCTL: CNTDBEN Position          */
#define LPTMR_EXTCTL_CNTDBEN_Msk        (0x1UL << LPTMR_EXTCTL_CNTDBEN_Pos)         /*!< LPTMR_T::EXTCTL: CNTDBEN Mask              */

#define LPTMR_EXTCTL_INTERCAPSEL_Pos    (8)                                         /*!< LPTMR_T::EXTCTL: INTERCAPSEL Position      */
#define LPTMR_EXTCTL_INTERCAPSEL_Msk    (0xfUL << LPTMR_EXTCTL_INTERCAPSEL_Pos)     /*!< LPTMR_T::EXTCTL: INTERCAPSEL Mask          */

#define LPTMR_EXTCTL_CAPEDGE_Pos        (12)                                        /*!< LPTMR_T::EXTCTL: CAPEDGE Position          */
#define LPTMR_EXTCTL_CAPEDGE_Msk        (0x7UL << LPTMR_EXTCTL_CAPEDGE_Pos)         /*!< LPTMR_T::EXTCTL: CAPEDGE Mask              */

#define LPTMR_EXTCTL_ECNTSSEL_Pos       (16)                                        /*!< LPTMR_T::EXTCTL: ECNTSSEL Position         */
#define LPTMR_EXTCTL_ECNTSSEL_Msk       (0x7UL << LPTMR_EXTCTL_ECNTSSEL_Pos)        /*!< LPTMR_T::EXTCTL: ECNTSSEL Mask             */

#define LPTMR_EXTCTL_CAPDIVSCL_Pos      (28)                                        /*!< LPTMR_T::EXTCTL: CAPDIVSCL Position        */
#define LPTMR_EXTCTL_CAPDIVSCL_Msk      (0xfUL << LPTMR_EXTCTL_CAPDIVSCL_Pos)       /*!< LPTMR_T::EXTCTL: CAPDIVSCL Mask            */

#define LPTMR_EINTSTS_CAPIF_Pos         (0)                                         /*!< LPTMR_T::EINTSTS: CAPIF Position           */
#define LPTMR_EINTSTS_CAPIF_Msk         (0x1UL << LPTMR_EINTSTS_CAPIF_Pos)          /*!< LPTMR_T::EINTSTS: CAPIF Mask               */

#define LPTMR_EINTSTS_CAPIFOV_Pos       (1)                                         /*!< LPTMR_T::EINTSTS: CAPIFOV Position         */
#define LPTMR_EINTSTS_CAPIFOV_Msk       (0x1UL << LPTMR_EINTSTS_CAPIFOV_Pos)        /*!< LPTMR_T::EINTSTS: CAPIFOV Mask             */

#define LPTMR_TRGCTL_TRGSSEL_Pos        (0)                                         /*!< LPTMR_T::TRGCTL: TRGSSEL Position          */
#define LPTMR_TRGCTL_TRGSSEL_Msk        (0x1UL << LPTMR_TRGCTL_TRGSSEL_Pos)         /*!< LPTMR_T::TRGCTL: TRGSSEL Mask              */

#define LPTMR_TRGCTL_TRGEN_Pos          (1)                                         /*!< LPTMR_T::TRGCTL: TRGEN Position            */
#define LPTMR_TRGCTL_TRGEN_Msk          (0x1UL << LPTMR_TRGCTL_TRGEN_Pos)           /*!< LPTMR_T::TRGCTL: TRGEN Mask                */

#define LPTMR_TRGCTL_TRGLPPDMA_Pos      (4)                                         /*!< LPTMR_T::TRGCTL: TRGLPPDMA Position        */
#define LPTMR_TRGCTL_TRGLPPDMA_Msk      (0x1UL << LPTMR_TRGCTL_TRGLPPDMA_Pos)       /*!< LPTMR_T::TRGCTL: TRGLPPDMA Mask            */

#define LPTMR_CAPNF_CAPNFEN_Pos         (0)                                         /*!< LPTMR_T::CAPNF: CAPNFEN Position           */
#define LPTMR_CAPNF_CAPNFEN_Msk         (0x1UL << LPTMR_CAPNF_CAPNFEN_Pos)          /*!< LPTMR_T::CAPNF: CAPNFEN Mask               */

#define LPTMR_CAPNF_CAPNFSEL_Pos        (4)                                         /*!< LPTMR_T::CAPNF: CAPNFSEL Position          */
#define LPTMR_CAPNF_CAPNFSEL_Msk        (0x7UL << LPTMR_CAPNF_CAPNFSEL_Pos)         /*!< LPTMR_T::CAPNF: CAPNFSEL Mask              */

#define LPTMR_CAPNF_CAPNFCNT_Pos        (8)                                         /*!< LPTMR_T::CAPNF: CAPNFCNT Position          */
#define LPTMR_CAPNF_CAPNFCNT_Msk        (0x7UL << LPTMR_CAPNF_CAPNFCNT_Pos)         /*!< LPTMR_T::CAPNF: CAPNFCNT Mask              */

#define LPTMR_PWMCTL_CNTEN_Pos          (0)                                         /*!< LPTMR_T::PWMCTL: CNTEN Position            */
#define LPTMR_PWMCTL_CNTEN_Msk          (0x1UL << LPTMR_PWMCTL_CNTEN_Pos)           /*!< LPTMR_T::PWMCTL: CNTEN Mask                */

#define LPTMR_PWMCTL_CNTMODE_Pos        (3)                                         /*!< LPTMR_T::PWMCTL: CNTMODE Position          */
#define LPTMR_PWMCTL_CNTMODE_Msk        (0x1UL << LPTMR_PWMCTL_CNTMODE_Pos)         /*!< LPTMR_T::PWMCTL: CNTMODE Mask              */

#define LPTMR_PWMCTL_PWMINTWKEN_Pos     (12)                                        /*!< LPTMR_T::PWMCTL: PWMINTWKEN Position       */
#define LPTMR_PWMCTL_PWMINTWKEN_Msk     (0x1UL << LPTMR_PWMCTL_PWMINTWKEN_Pos)      /*!< LPTMR_T::PWMCTL: PWMINTWKEN Mask           */

#define LPTMR_PWMCTL_DBGHALT_Pos        (30)                                        /*!< LPTMR_T::PWMCTL: DBGHALT Position          */
#define LPTMR_PWMCTL_DBGHALT_Msk        (0x1UL << LPTMR_PWMCTL_DBGHALT_Pos)         /*!< LPTMR_T::PWMCTL: DBGHALT Mask              */

#define LPTMR_PWMCTL_DBGTRIOFF_Pos      (31)                                        /*!< LPTMR_T::PWMCTL: DBGTRIOFF Position        */
#define LPTMR_PWMCTL_DBGTRIOFF_Msk      (0x1UL << LPTMR_PWMCTL_DBGTRIOFF_Pos)       /*!< LPTMR_T::PWMCTL: DBGTRIOFF Mask            */

#define LPTMR_PWMCLKPSC_CLKPSC_Pos      (0)                                         /*!< LPTMR_T::PWMCLKPSC: CLKPSC Position        */
#define LPTMR_PWMCLKPSC_CLKPSC_Msk      (0xffUL << LPTMR_PWMCLKPSC_CLKPSC_Pos)      /*!< LPTMR_T::PWMCLKPSC: CLKPSC Mask            */

#define LPTMR_PWMCNTCLR_CNTCLR_Pos      (0)                                         /*!< LPTMR_T::PWMCNTCLR: CNTCLR Position        */
#define LPTMR_PWMCNTCLR_CNTCLR_Msk      (0x1UL << LPTMR_PWMCNTCLR_CNTCLR_Pos)       /*!< LPTMR_T::PWMCNTCLR: CNTCLR Mask            */

#define LPTMR_PWMPERIOD_PERIOD_Pos      (0)                                         /*!< LPTMR_T::PWMPERIOD: PERIOD Position        */
#define LPTMR_PWMPERIOD_PERIOD_Msk      (0xffffUL << LPTMR_PWMPERIOD_PERIOD_Pos)    /*!< LPTMR_T::PWMPERIOD: PERIOD Mask            */

#define LPTMR_PWMCMPDAT_CMP_Pos         (0)                                         /*!< LPTMR_T::PWMCMPDAT: CMP Position           */
#define LPTMR_PWMCMPDAT_CMP_Msk         (0xffffUL << LPTMR_PWMCMPDAT_CMP_Pos)       /*!< LPTMR_T::PWMCMPDAT: CMP Mask               */

#define LPTMR_PWMCNT_CNT_Pos            (0)                                         /*!< LPTMR_T::PWMCNT: CNT Position              */
#define LPTMR_PWMCNT_CNT_Msk            (0xffffUL << LPTMR_PWMCNT_CNT_Pos)          /*!< LPTMR_T::PWMCNT: CNT Mask                  */

#define LPTMR_PWMPOLCTL_PINV_Pos        (0)                                         /*!< LPTMR_T::PWMPOLCTL: PINV Position          */
#define LPTMR_PWMPOLCTL_PINV_Msk        (0x1UL << LPTMR_PWMPOLCTL_PINV_Pos)         /*!< LPTMR_T::PWMPOLCTL: PINV Mask              */

#define LPTMR_PWMPOCTL_POEN_Pos         (0)                                         /*!< LPTMR_T::PWMPOCTL: POEN Position           */
#define LPTMR_PWMPOCTL_POEN_Msk         (0x1UL << LPTMR_PWMPOCTL_POEN_Pos)          /*!< LPTMR_T::PWMPOCTL: POEN Mask               */

#define LPTMR_PWMPOCTL_POSEL_Pos        (8)                                         /*!< LPTMR_T::PWMPOCTL: POSEL Position          */
#define LPTMR_PWMPOCTL_POSEL_Msk        (0x1UL << LPTMR_PWMPOCTL_POSEL_Pos)         /*!< LPTMR_T::PWMPOCTL: POSEL Mask              */

#define LPTMR_PWMINTEN0_PIEN_Pos        (1)                                         /*!< LPTMR_T::PWMINTEN0: PIEN Position          */
#define LPTMR_PWMINTEN0_PIEN_Msk        (0x1UL << LPTMR_PWMINTEN0_PIEN_Pos)         /*!< LPTMR_T::PWMINTEN0: PIEN Mask              */

#define LPTMR_PWMINTEN0_CMPUIEN_Pos     (2)                                         /*!< LPTMR_T::PWMINTEN0: CMPUIEN Position       */
#define LPTMR_PWMINTEN0_CMPUIEN_Msk     (0x1UL << LPTMR_PWMINTEN0_CMPUIEN_Pos)      /*!< LPTMR_T::PWMINTEN0: CMPUIEN Mask           */

#define LPTMR_PWMINTSTS0_PIF_Pos        (1)                                         /*!< LPTMR_T::PWMINTSTS0: PIF Position          */
#define LPTMR_PWMINTSTS0_PIF_Msk        (0x1UL << LPTMR_PWMINTSTS0_PIF_Pos)         /*!< LPTMR_T::PWMINTSTS0: PIF Mask              */

#define LPTMR_PWMINTSTS0_CMPUIF_Pos     (2)                                         /*!< LPTMR_T::PWMINTSTS0: CMPUIF Position       */
#define LPTMR_PWMINTSTS0_CMPUIF_Msk     (0x1UL << LPTMR_PWMINTSTS0_CMPUIF_Pos)      /*!< LPTMR_T::PWMINTSTS0: CMPUIF Mask           */

#define LPTMR_PWMTRGCTL_TRGSEL_Pos      (0)                                         /*!< LPTMR_T::PWMTRGCTL: TRGSEL Position        */
#define LPTMR_PWMTRGCTL_TRGSEL_Msk      (0x3UL << LPTMR_PWMTRGCTL_TRGSEL_Pos)       /*!< LPTMR_T::PWMTRGCTL: TRGSEL Mask            */

#define LPTMR_PWMTRGCTL_TRGEN_Pos       (2)                                         /*!< LPTMR_T::PWMTRGCTL: TRGEN Position         */
#define LPTMR_PWMTRGCTL_TRGEN_Msk       (0x1UL << LPTMR_PWMTRGCTL_TRGEN_Pos)        /*!< LPTMR_T::PWMTRGCTL: TRGEN Mask             */

#define LPTMR_PWMTRGCTL_PWMTRGLPPDMA_Pos    (9)                                         /*!< LPTMR_T::PWMTRGCTL: PWMTRGLPPDMA Position  */
#define LPTMR_PWMTRGCTL_PWMTRGLPPDMA_Msk    (0x1UL << LPTMR_PWMTRGCTL_PWMTRGLPPDMA_Pos) /*!< LPTMR_T::PWMTRGCTL: PWMTRGLPPDMA Mask      */

#define LPTMR_PWMTRGCTL_AEPDMA_Pos      (10)                                        /*!< LPTMR_T::PWMTRGCTL: AEPDMA Position  */
#define LPTMR_PWMTRGCTL_AEPDMA_Msk      (0x1UL << LPTMR_PWMTRGCTL_AEPDMA_Pos)       /*!< LPTMR_T::PWMTRGCTL: AEPDMA Mask      */

#define LPTMR_PWMSTATUS_CNTMAXF_Pos     (0)                                         /*!< LPTMR_T::PWMSTATUS: CNTMAXF Position       */
#define LPTMR_PWMSTATUS_CNTMAXF_Msk     (0x1UL << LPTMR_PWMSTATUS_CNTMAXF_Pos)      /*!< LPTMR_T::PWMSTATUS: CNTMAXF Mask           */

#define LPTMR_PWMSTATUS_PWMINTWKF_Pos   (8)                                         /*!< LPTMR_T::PWMSTATUS: PWMINTWKF Position     */
#define LPTMR_PWMSTATUS_PWMINTWKF_Msk   (0x1UL << LPTMR_PWMSTATUS_PWMINTWKF_Pos)    /*!< LPTMR_T::PWMSTATUS: PWMINTWKF Mask         */

#define LPTMR_PWMPBUF_PBUF_Pos          (0)                                         /*!< LPTMR_T::PWMPBUF: PBUF Position            */
#define LPTMR_PWMPBUF_PBUF_Msk          (0xffffUL << LPTMR_PWMPBUF_PBUF_Pos)        /*!< LPTMR_T::PWMPBUF: PBUF Mask                */

#define LPTMR_PWMCMPBUF_CMPBUF_Pos      (0)                                         /*!< LPTMR_T::PWMCMPBUF: CMPBUF Position        */
#define LPTMR_PWMCMPBUF_CMPBUF_Msk      (0xffffUL << LPTMR_PWMCMPBUF_CMPBUF_Pos)    /*!< LPTMR_T::PWMCMPBUF: CMPBUF Mask            */

#define LPTMR_PWMIFA_IFACNT_Pos         (0)                                         /*!< LPTMR_T::PWMIFA: IFACNT Position           */
#define LPTMR_PWMIFA_IFACNT_Msk         (0xffffUL << LPTMR_PWMIFA_IFACNT_Pos)       /*!< LPTMR_T::PWMIFA: IFACNT Mask               */

#define LPTMR_PWMIFA_STPMOD_Pos         (24)                                        /*!< LPTMR_T::PWMIFA: STPMOD Position           */
#define LPTMR_PWMIFA_STPMOD_Msk         (0x1UL << LPTMR_PWMIFA_STPMOD_Pos)          /*!< LPTMR_T::PWMIFA: STPMOD Mask               */

#define LPTMR_PWMIFA_IFASEL_Pos         (28)                                        /*!< LPTMR_T::PWMIFA: IFASEL Position           */
#define LPTMR_PWMIFA_IFASEL_Msk         (0x3UL << LPTMR_PWMIFA_IFASEL_Pos)          /*!< LPTMR_T::PWMIFA: IFASEL Mask               */

#define LPTMR_PWMIFA_IFAEN_Pos          (31)                                        /*!< LPTMR_T::PWMIFA: IFAEN Position            */
#define LPTMR_PWMIFA_IFAEN_Msk          (0x1UL << LPTMR_PWMIFA_IFAEN_Pos)           /*!< LPTMR_T::PWMIFA: IFAEN Mask                */

#define LPTMR_PWMAINTSTS_IFAIF_Pos      (0)                                         /*!< LPTMR_T::PWMAINTSTS: IFAIF Position        */
#define LPTMR_PWMAINTSTS_IFAIF_Msk      (0x1UL << LPTMR_PWMAINTSTS_IFAIF_Pos)       /*!< LPTMR_T::PWMAINTSTS: IFAIF Mask            */

#define LPTMR_PWMAINTEN_IFAIEN_Pos      (0)                                         /*!< LPTMR_T::PWMAINTEN: IFAIEN Position        */
#define LPTMR_PWMAINTEN_IFAIEN_Msk      (0x1UL << LPTMR_PWMAINTEN_IFAIEN_Pos)       /*!< LPTMR_T::PWMAINTEN: IFAIEN Mask            */

#define LPTMR_PWMAPDMACTL_APDMAEN_Pos   (0)                                         /*!< LPTMR_T::PWMAPDMACTL: APDMAEN Position     */
#define LPTMR_PWMAPDMACTL_APDMAEN_Msk   (0x1UL << LPTMR_PWMAPDMACTL_APDMAEN_Pos)    /*!< LPTMR_T::PWMAPDMACTL: APDMAEN Mask         */

/** @} LPTMR_CONST */
/** @} end of LPTMR register group */
/** @} end of REGISTER group */

#if defined ( __CC_ARM   )
    #pragma no_anon_unions
#endif

#endif /* __LPTMR_REG_H__ */
