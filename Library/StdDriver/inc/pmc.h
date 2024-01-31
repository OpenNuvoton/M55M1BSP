/**************************************************************************//**
 * @file     pmc.h
 * @version  V1.00
 * @brief    PMC driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __PMC_H__
#define __PMC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup PMC_Driver PMC Driver
  @{
*/

/** @addtogroup PMC_EXPORTED_CONSTANTS PMC Exported Constants
  @{
*/

enum
{
    SRAMNum0 = 0,
    SRAMNum1,
    SRAMNum2,
    SRAMNum3,
    SRAMNum4,
};

/*---------------------------------------------------------------------------------------------------------*/
/* PMC Timeout constant definitions.                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define PMC_TIMEOUT         SystemCoreClock     /*!< PMC time-out counter (1 second time-out) */

/*---------------------------------------------------------------------------------------------------------*/
/* PMC Define Error Code                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define PMC_OK              ( 0L)               /*!< PMC operation OK */
#define PMC_ERR_FAIL        (-1L)               /*!< PMC operation failed */
#define PMC_ERR_TIMEOUT     (-2L)               /*!< PMC operation abort due to timeout error */

/*---------------------------------------------------------------------------------------------------------*/
/*  PLCTL constant definitions. (Write-Protection Register)                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define PMC_PLCTL_PLSEL_PL0     (0x0UL<<PMC_PLCTL_PLSEL_Pos)   /*!< Set power level to power level 0. Supports system clock up to 96MHz. */
#define PMC_PLCTL_PLSEL_PL1     (0x1UL<<PMC_PLCTL_PLSEL_Pos)   /*!< Set power level to power level 1. Supports system clock up to 84MHz. */

/*---------------------------------------------------------------------------------------------------------*/
/*  PLSTS constant definitions. (Write-Protection Register)                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define PMC_PLSTS_PLSTATUS_PL0  (0x0UL<<PMC_PLSTS_PLSTATUS_Pos) /*!< Power level is power level 0. Supports system clock up to 96MHz. */
#define PMC_PLSTS_PLSTATUS_PL1  (0x1UL<<PMC_PLSTS_PLSTATUS_Pos) /*!< Power level is power level 1. Supports system clock up to 84MHz. */

/*---------------------------------------------------------------------------------------------------------*/
/*  VRCTL constant definitions. (Write-Protection Register)                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define PMC_VRCTL_MVRS_LDO      (0x0UL<<PMC_VRCTL_MVRS_Pos)    /*!< Set main voltage regulator type to LDO */
#define PMC_VRCTL_MVRS_DCDC     (0x1UL<<PMC_VRCTL_MVRS_Pos)    /*!< Set main voltage regulator type to DCDC */

/*---------------------------------------------------------------------------------------------------------*/
/*  VRSTS constant definitions. (Write-Protection Register)                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define PMC_VRSTS_CURMVR_LDO    (0x0UL<<PMC_VRSTS_CURMVR_Pos)   /*!< Main voltage regulator type is LDO */
#define PMC_VRSTS_CURMVR_DCDC   (0x1UL<<PMC_VRSTS_CURMVR_Pos)   /*!< Main voltage regulator type is DCDC */

/*---------------------------------------------------------------------------------------------------------*/
/*  SYSRB constant definitions. (Write-Protection Register)                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define PMC_SYSRB0PC_SRAM_NORMAL            0x00000000UL   /*!< Select SRAM power mode to normal mode */
#define PMC_SYSRB0PC_SRAM_RETENTION         0x00000001UL   /*!< Select SRAM power mode to retention mode */
#define PMC_SYSRB0PC_SRAM_POWER_SHUT_DOWN   0x00000002UL   /*!< Select SRAM power mode to power shut down mode */
#define PMC_SYSRB1PC_SRAM_NORMAL            0x10000000UL   /*!< Select SRAM power mode to normal mode */
#define PMC_SYSRB1PC_SRAM_RETENTION         0x10000001UL   /*!< Select SRAM power mode to retention mode */
#define PMC_SYSRB1PC_SRAM_POWER_SHUT_DOWN   0x10000002UL   /*!< Select SRAM power mode to power shut down mode */
#define PMC_SYSRB2PC_SRAM_NORMAL            0x20000000UL   /*!< Select SRAM power mode to normal mode */
#define PMC_SYSRB2PC_SRAM_RETENTION         0x20000001UL   /*!< Select SRAM power mode to retention mode */
#define PMC_SYSRB2PC_SRAM_POWER_SHUT_DOWN   0x20000002UL   /*!< Select SRAM power mode to power shut down mode */
#define PMC_SYSRB3PC_SRAM_NORMAL            0x30000000UL   /*!< Select SRAM power mode to normal mode */
#define PMC_SYSRB3PC_SRAM_RETENTION         0x30000001UL   /*!< Select SRAM power mode to retention mode */
#define PMC_SYSRB3PC_SRAM_POWER_SHUT_DOWN   0x30000002UL   /*!< Select SRAM power mode to power shut down mode */
#define PMC_LPSYSRPC_SRAM_NORMAL            0x40000000UL   /*!< Select SRAM power mode to normal mode */
#define PMC_LPSYSRPC_SRAM_RETENTION         0x40000001UL   /*!< Select SRAM power mode to retention mode */
#define PMC_LPSYSRPC_SRAM_POWER_SHUT_DOWN   0x40000002UL   /*!< Select SRAM power mode to power shut down mode */

/*---------------------------------------------------------------------------------------------------------*/
/*  Power mode constant definitions.                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define PMC_NPD0        0x0UL   /*!< Select NPD0 as power down mode */
#define PMC_NPD1        0x1UL   /*!< Select NPD1 as power down mode */
#if 0   // TESTCHIP_ONLY not support
#define PMC_NPD2        0x2UL   /*!< Select NPD2 as power down mode */
#define PMC_NPD3        0x3UL   /*!< Select NPD3 as power down mode */
#define PMC_NPD4        0x4UL   /*!< Select NPD4 as power down mode */
#endif
#define PMC_SPD0        0x5UL   /*!< Select SPD0 as power down mode */
#if 0   // TESTCHIP_ONLY not support
#define PMC_SPD1        0x6UL   /*!< Select SPD1 as power down mode */
#endif
#define PMC_DPD0        0x7UL   /*!< Select DPD0 as power down mode */
#if 0   // TESTCHIP_ONLY not support
#define PMC_DPD1        0x8UL   /*!< Select DPD1 as power down mode */
#endif
/*---------------------------------------------------------------------------------------------------------*/
/*  TG Pin Rising/Falling Edge Wake-up Enable constant definitions.                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define PMC_TGPIN_WAKEUP_ENABLE     (0x1UL << PMC_GPATGCTL_TGPWKEN_Pos)     /*!< Enable TG Pin Wake-up \hideinitializer */
#define PMC_TGPIN_WAKEUP_DISABLE    (0x0UL << PMC_GPATGCTL_TGPWKEN_Pos)     /*!< Enable TG Pin Wake-up \hideinitializer */
#define PMC_TGPIN_RISING            (0x1UL << PMC_GPATGCTL_TGPREN_Pos)      /*!< TG Pin trigger on rising edge \hideinitializer */
#define PMC_TGPIN_FALLING           (0x1UL << PMC_GPATGCTL_TGPFEN_Pos)      /*!< TG Pin trigger on falling edge \hideinitializer */
#define PMC_TGPIN_BOTH              (0x3UL << PMC_GPATGCTL_TGPREN_Pos)      /*!< TG Pin trigger on both edge \hideinitializer */
#define PMC_TGPIN_DEBOUNCEEN        (0x1UL << PMC_GPATGCTL_TGPDBEN_Pos)     /*!< Enable TG Pin De-bounce function \hideinitializer */
#define PMC_TGPIN_DEBOUNCEDIS       (0x0UL << PMC_GPATGCTL_TGPDBEN_Pos)     /*!< Disable TG Pin De-bounce function \hideinitializer */
#define PMC_TGPIN_PA                (0UL)            /*!< TG Pin port A select \hideinitializer */
#define PMC_TGPIN_PB                (1UL)            /*!< TG Pin port B select \hideinitializer */
#define PMC_TGPIN_PC                (2UL)            /*!< TG Pin port C select \hideinitializer */
#define PMC_TGPIN_PD                (3UL)            /*!< TG Pin port D select \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Busy flag address constant definitions.                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define PMC_PWRCTL_BUSY_FLAG        (PMC_BASE+0x000UL)  /*!< Select PWRCTL busy flag \hideinitializer */
#define PMC_PLCTL_BUSY_FLAG         (PMC_BASE+0x010UL)  /*!< Select PLCTL busy flag \hideinitializer */
#define PMC_PLSTS_BUSY_FLAG         (PMC_BASE+0x014UL)  /*!< Select PLSTS busy flag \hideinitializer */
#define PMC_VRCTL_BUSY_FLAG         (PMC_BASE+0x018UL)  /*!< Select VRCTL busy flag \hideinitializer */
#define PMC_VRSTS_BUSY_FLAG         (PMC_BASE+0x01CUL)  /*!< Select VRSTS busy flag \hideinitializer */
#define PMC_STMRWKCTL_BUSY_FLAG     (PMC_BASE+0x050UL)  /*!< Select STMRWKCTL busy flag \hideinitializer */
#define PMC_SYSRB0PC_BUSY_FLAG      (PMC_BASE+0x100UL)  /*!< Select SYSRB0PC busy flag \hideinitializer */
#define PMC_SYSRB1PC_BUSY_FLAG      (PMC_BASE+0x104UL)  /*!< Select SYSRB1PC busy flag \hideinitializer */
#define PMC_SYSRB2PC_BUSY_FLAG      (PMC_BASE+0x108UL)  /*!< Select SYSRB2PC busy flag \hideinitializer */
#define PMC_SYSRB3PC_BUSY_FLAG      (PMC_BASE+0x10CUL)  /*!< Select SYSRB3PC busy flag \hideinitializer */
#define PMC_LPSYSRPC_BUSY_FLAG      (PMC_BASE+0x110UL)  /*!< Select LPSYSRPC busy flag \hideinitializer */
#define PMC_KSRPC_BUSY_FLAG         (PMC_BASE+0x140UL)  /*!< Select KSRPC busy flag \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  STMRIS constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define PMC_STMRWKCTL_STMRIS_512          (0x0UL << PMC_STMRWKCTL_STMRIS_Pos)     /*!< Select Wake-up Timer Time-out Interval is 512 LIRC clocks (16ms) \hideinitializer */
#define PMC_STMRWKCTL_STMRIS_1024         (0x1UL << PMC_STMRWKCTL_STMRIS_Pos)     /*!< Select Wake-up Timer Time-out Interval is 1024 LIRC clocks (32ms) \hideinitializer */
#define PMC_STMRWKCTL_STMRIS_2048         (0x2UL << PMC_STMRWKCTL_STMRIS_Pos)     /*!< Select Wake-up Timer Time-out Interval is 2048 LIRC clocks (64ms) \hideinitializer */
#define PMC_STMRWKCTL_STMRIS_4096         (0x3UL << PMC_STMRWKCTL_STMRIS_Pos)     /*!< Select Wake-up Timer Time-out Interval is 4096 LIRC clocks (128ms) \hideinitializer */
#define PMC_STMRWKCTL_STMRIS_8192         (0x4UL << PMC_STMRWKCTL_STMRIS_Pos)     /*!< Select Wake-up Timer Time-out Interval is 8192 LIRC clocks (256ms) \hideinitializer */
#define PMC_STMRWKCTL_STMRIS_16384        (0x5UL << PMC_STMRWKCTL_STMRIS_Pos)     /*!< Select Wake-up Timer Time-out Interval is 16384 LIRC clocks (512ms) \hideinitializer */
#define PMC_STMRWKCTL_STMRIS_32768        (0x6UL << PMC_STMRWKCTL_STMRIS_Pos)     /*!< Select Wake-up Timer Time-out Interval is 32768 LIRC clocks (1024ms) \hideinitializer */
#define PMC_STMRWKCTL_STMRIS_65536        (0x7UL << PMC_STMRWKCTL_STMRIS_Pos)     /*!< Select Wake-up Timer Time-out Interval is 65536 LIRC clocks (2048ms) \hideinitializer */
#define PMC_STMRWKCTL_STMRIS_131072       (0x8UL << PMC_STMRWKCTL_STMRIS_Pos)     /*!< Select Wake-up Timer Time-out Interval is 131072 LIRC clocks (4096ms) \hideinitializer */
#define PMC_STMRWKCTL_STMRIS_262144       (0x9UL << PMC_STMRWKCTL_STMRIS_Pos)     /*!< Select Wake-up Timer Time-out Interval is 262144 LIRC clocks (8192ms) \hideinitializer */
#define PMC_STMRWKCTL_STMRIS_524288       (0xaUL << PMC_STMRWKCTL_STMRIS_Pos)     /*!< Select Wake-up Timer Time-out Interval is 524288 LIRC clocks (16384ms) \hideinitializer */
#define PMC_STMRWKCTL_STMRIS_1048576      (0xbUL << PMC_STMRWKCTL_STMRIS_Pos)     /*!< Select Wake-up Timer Time-out Interval is 1048576 LIRC clocks (32768ms) \hideinitializer */
#define PMC_STMRWKCTL_STMRIS_2097152      (0xcUL << PMC_STMRWKCTL_STMRIS_Pos)     /*!< Select Wake-up Timer Time-out Interval is 2097152 LIRC clocks (65536ms) \hideinitializer */
#define PMC_STMRWKCTL_STMRIS_4194304      (0xdUL << PMC_STMRWKCTL_STMRIS_Pos)     /*!< Select Wake-up Timer Time-out Interval is 4194304 LIRC clocks (131072ms) \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  Pin Rising/Falling Edge Wake-up Enable constant definitions.                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define PMC_WKPIN0_RISING     (0x1UL << PMC_PINWKCTL_WKPINEN0_Pos)    /*!< Enable Wake-up pin0 (GPC.0) rising edge at Deep Power-down mode \hideinitializer */
#define PMC_WKPIN0_FALLING    (0x2UL << PMC_PINWKCTL_WKPINEN0_Pos)    /*!< Enable Wake-up pin0 (GPC.0) falling edge at Deep Power-down mode \hideinitializer */
#define PMC_WKPIN0_BOTHEDGE   (0x3UL << PMC_PINWKCTL_WKPINEN0_Pos)    /*!< Enable Wake-up pin0 (GPC.0) both edge at Deep Power-down mode \hideinitializer */

#define PMC_WKPIN1_RISING     (0x1UL << PMC_PINWKCTL_WKPINEN1_Pos)    /*!< Enable Wake-up pin1 (GPB.0) rising edge at Deep Power-down mode \hideinitializer */
#define PMC_WKPIN1_FALLING    (0x2UL << PMC_PINWKCTL_WKPINEN1_Pos)    /*!< Enable Wake-up pin1 (GPB.0) falling edge at Deep Power-down mode \hideinitializer */
#define PMC_WKPIN1_BOTHEDGE   (0x3UL << PMC_PINWKCTL_WKPINEN1_Pos)    /*!< Enable Wake-up pin1 (GPB.0) both edge at Deep Power-down mode \hideinitializer */

#define PMC_WKPIN2_RISING     (0x1UL << PMC_PINWKCTL_WKPINEN2_Pos)    /*!< Enable Wake-up pin2 (GPB.2) rising edge at Deep Power-down mode \hideinitializer */
#define PMC_WKPIN2_FALLING    (0x2UL << PMC_PINWKCTL_WKPINEN2_Pos)    /*!< Enable Wake-up pin2 (GPB.2) falling edge at Deep Power-down mode \hideinitializer */
#define PMC_WKPIN2_BOTHEDGE   (0x3UL << PMC_PINWKCTL_WKPINEN2_Pos)    /*!< Enable Wake-up pin2 (GPB.2) both edge at Deep Power-down mode \hideinitializer */

#define PMC_WKPIN3_RISING     (0x1UL << PMC_PINWKCTL_WKPINEN3_Pos)    /*!< Enable Wake-up pin3 (GPB.12) rising edge at Deep Power-down mode \hideinitializer */
#define PMC_WKPIN3_FALLING    (0x2UL << PMC_PINWKCTL_WKPINEN3_Pos)    /*!< Enable Wake-up pin3 (GPB.12) falling edge at Deep Power-down mode \hideinitializer */
#define PMC_WKPIN3_BOTHEDGE   (0x3UL << PMC_PINWKCTL_WKPINEN3_Pos)    /*!< Enable Wake-up pin3 (GPB.12) both edge at Deep Power-down mode \hideinitializer */

#define PMC_WKPIN4_RISING     (0x1UL << PMC_PINWKCTL_WKPINEN4_Pos)    /*!< Enable Wake-up pin4 (GPF.6) rising edge at Deep Power-down mode \hideinitializer */
#define PMC_WKPIN4_FALLING    (0x2UL << PMC_PINWKCTL_WKPINEN4_Pos)    /*!< Enable Wake-up pin4 (GPF.6) falling edge at Deep Power-down mode \hideinitializer */
#define PMC_WKPIN4_BOTHEDGE   (0x3UL << PMC_PINWKCTL_WKPINEN4_Pos)    /*!< Enable Wake-up pin4 (GPF.6) both edge at Deep Power-down mode \hideinitializer */

#define PMC_WKPIN5_RISING     (0x1UL << PMC_PINWKCTL_WKPINEN5_Pos)    /*!< Enable Wake-up pin4 (GPA.12) rising edge at Deep Power-down mode \hideinitializer */
#define PMC_WKPIN5_FALLING    (0x2UL << PMC_PINWKCTL_WKPINEN5_Pos)    /*!< Enable Wake-up pin4 (GPA.12) falling edge at Deep Power-down mode \hideinitializer */
#define PMC_WKPIN5_BOTHEDGE   (0x3UL << PMC_PINWKCTL_WKPINEN5_Pos)    /*!< Enable Wake-up pin4 (GPA.12) both edge at Deep Power-down mode \hideinitializer */

/**
  * @brief      Disable Wake-up Pin 0
  * @details    This macro disables Wake-up pin 0 (GPC.0) at Power-down mode.
  */
#define PMC_DISABLE_WKPIN0()   (PMC->PINWKCTL &= ~PMC_PINWKCTL_WKPINEN0_Msk)

/**
  * @brief      Disable Wake-up Pin 1
  * @details    This macro disables Wake-up pin 1 (GPB.0) at Power-down mode.
  */
#define PMC_DISABLE_WKPIN1()   (PMC->PINWKCTL &= ~PMC_PINWKCTL_WKPINEN1_Msk)

/**
  * @brief      Disable Wake-up Pin 2
  * @details    This macro disables Wake-up pin 2 (GPB.2) at Power-down mode.
  */
#define PMC_DISABLE_WKPIN2()   (PMC->PINWKCTL &= ~PMC_PINWKCTL_WKPINEN2_Msk)

/**
  * @brief      Disable Wake-up Pin 3
  * @details    This macro disables Wake-up pin 3 (GPB.12) at Power-down mode.
  */
#define PMC_DISABLE_WKPIN3()   (PMC->PINWKCTL &= ~PMC_PINWKCTL_WKPINEN3_Msk)

/**
  * @brief      Disable Wake-up Pin 4
  * @details    This macro disables Wake-up pin 4 (GPF.6) at Power-down mode.
  */
#define PMC_DISABLE_WKPIN4()   (PMC->PINWKCTL &= ~PMC_PINWKCTL_WKPINEN4_Msk)

/**
  * @brief      Disable Wake-up Pin 5
  * @details    This macro disables Wake-up pin 5 (GPA.12) at Power-down mode.
  */
#define PMC_DISABLE_WKPIN5()   (PMC->PINWKCTL &= ~PMC_PINWKCTL_WKPINEN5_Msk)

/**
  * @brief      Disable GPA Trigger Pin
  * @details    This macro disables GPA trigger and wake-up function
  */
#define PMC_DISABLE_TGPIN_GPA()   (PMC->GPATGCTL &= ~(PMC_GPATGCTL_TGPFEN_Msk | PMC_GPATGCTL_TGPREN_Msk))

/**
  * @brief      Disable GPB Trigger Pin
  * @details    This macro disables GPB trigger and wake-up function
  */
#define PMC_DISABLE_TGPIN_GPB()   (PMC->GPBTGCTL &= ~(PMC_GPBTGCTL_TGPFEN_Msk | PMC_GPBTGCTL_TGPREN_Msk))

/**
  * @brief      Disable GPC Trigger Pin
  * @details    This macro disables GPC trigger and wake-up function
  */
#define PMC_DISABLE_TGPIN_GPC()   (PMC->GPCTGCTL &= ~(PMC_GPCTGCTL_TGPFEN_Msk | PMC_GPCTGCTL_TGPREN_Msk))

/**
  * @brief      Disable GPD Trigger Pin
  * @details    This macro disables GPD trigger and wake-up function
  */
#define PMC_DISABLE_TGPIN_GPD()   (PMC->GPDTGCTL &= ~(PMC_GPDTGCTL_TGPFEN_Msk | PMC_GPDTGCTL_TGPREN_Msk))

/**
  * @brief      Release GPIO Hold Status
  * @details    This macro releases GPIO hold status from power-down wake-up
  */
#define PMC_RELEASE_GPIO()    (PMC->IOSHCTL |= PMC_IOSHCTL_IOHR_Msk)

/**
  * @brief      Enable PMC Interrupt
  * @details    This macro enables PMC interrtup
  */
#define PMC_ENABLE_INT()    (PMC->INTEN |= PMC_INTEN_PDWKIEN_Msk)

/**
  * @brief      Disable PMC Interrupt
  * @details    This macro disables PMC interrtup
  */
#define PMC_DISABLE_INT()    (PMC->INTEN &= ~PMC_INTEN_PDWKIEN_Msk)

int32_t PMC_SetPowerLevel(uint32_t u32PowerLevel);
int32_t PMC_SetPowerRegulator(uint32_t u32PowerRegulator);
int32_t PMC_SetSRAMPowerMode(uint32_t u32SRAMSel, uint32_t u32PowerMode);
void PMC_PowerDown(void);
void PMC_Idle(void);
int32_t PMC_SetPowerDownMode(uint32_t u32PDMode, uint32_t u32PowerLevel);
void PMC_EnableWKPIN(uint32_t u32TriggerType);
uint32_t PMC_GetPMCWKSrc(void);
void PMC_EnableTGPin(uint32_t u32Port, uint32_t u32Pin, uint32_t u32TriggerType, uint32_t u32DebounceEn, uint32_t u32WakeupEn);
int32_t PMC_EnableSTMR(uint32_t u32Interval);
int32_t PMC_DisableSTMR(void);
int32_t PMC_Wait_BusyFlag(uint32_t PMCBusyFlagAddr);

/** @} end of group PMC_EXPORTED_FUNCTIONS */
/** @} end of group PMC_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __PMC_H__ */
