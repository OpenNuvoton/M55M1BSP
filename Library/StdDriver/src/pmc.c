/**************************************************************************//**
 * @file     pmc.c
 * @version  V1.00
 * @brief    PMC driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
    @{
*/

/** @addtogroup PMC_Driver PMC Driver
    @{
*/

/** @addtogroup PMC_EXPORTED_FUNCTIONS PMC Exported Functions
    @{
*/

/**
  * @brief      Set Power Level
  * @param[in]  u32PowerLevel is power level setting. Including :
  *             - \ref PMC_PLCTL_PLSEL_PL0  : Supports system clock up to 240MHz.
  *             - \ref PMC_PLCTL_PLSEL_PL1  : Supports system clock up to 200MHz.
  * @retval     PMC_OK          PMC operation OK.
  * @retval     PMC_ERR_TIMEOUT PMC operation abort due to timeout error.
  * @details    This function select power level.
  *             The register write-protection function should be disabled before using this function.
  */
int32_t PMC_SetPowerLevel(uint32_t u32PowerLevel)
{
    /* Wait for power level write busy flag is cleared */
    if (PMC_Wait_BusyFlag(PMC_PLCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    /* Set power voltage level */
    PMC->PLCTL = (PMC->PLCTL & (~PMC_PLCTL_PLSEL_Msk)) | (u32PowerLevel);

    /* Wait for power level change busy flag is cleared */
    if (PMC_Wait_BusyFlag(PMC_PLSTS_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    return PMC_OK;
}

/**
  * @brief      Set Main Voltage Regulator Type
  * @param[in]  u32PowerRegulator is main voltage regulator type. Including :
  *             - \ref PMC_VRCTL_MVRS_LDO
  *             - \ref PMC_VRCTL_MVRS_DCDC
  * @retval     PMC_ERR_FAIL    Main voltage regulator type setting is not finished.
  * @retval     PMC_OK          Main voltage regulator type setting is finished.
  * @retval     PMC_ERR_TIMEOUT PMC operation abort due to timeout error.
  * @details    This function set main voltage regulator type.
  *             The main voltage regulator type setting to DCDC cannot finished if the inductor is not detected.
  *             The register write-protection function should be disabled before using this function.
  */
int32_t PMC_SetPowerRegulator(uint32_t u32PowerRegulator)
{
    int32_t i32Ret = PMC_OK;
    uint32_t u32PowerRegStatus;

    /* Get main voltage regulator type status */
    u32PowerRegStatus = PMC->VRSTS & PMC_VRSTS_CURMVR_Msk;

    /* Set main voltage regulator type */
    if ((u32PowerRegulator == PMC_VRCTL_MVRS_DCDC) && (u32PowerRegStatus == PMC_VRSTS_CURMVR_LDO))
    {
        /* Set main voltage regulator type to DCDC if status is LDO */
        if (PMC_Wait_BusyFlag(PMC_VRCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

        PMC->VRCTL |= PMC_VRCTL_MVRS_Msk;

        /* Wait main voltage regulator type change ready */
        if (PMC_Wait_BusyFlag(PMC_VRSTS_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

        if ((PMC->VRSTS & PMC_VRSTS_CURMVR_Msk) != PMC_VRSTS_CURMVR_DCDC)
        {
            i32Ret = PMC_ERR_FAIL;    /* Main voltage regulator type change fail */
        }
    }
    else if (u32PowerRegulator == PMC_VRCTL_MVRS_LDO)
    {
        if (PMC_Wait_BusyFlag(PMC_VRCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

        /* Set main voltage regulator type to LDO */
        PMC->VRCTL &= (~PMC_VRCTL_MVRS_Msk);

        /* Wait main voltage regulator type change ready */
        if (PMC_Wait_BusyFlag(PMC_VRSTS_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

        if ((PMC->VRSTS & PMC_VRSTS_CURMVR_Msk) != PMC_VRSTS_CURMVR_LDO)
        {
            i32Ret = PMC_ERR_FAIL;    /* Main voltage regulator type change fail */
        }
    }

    /* Clear main voltage regulator type change error flag */
    if (PMC->VRSTS & PMC_VRSTS_MVRCERR_Msk)
    {
        PMC->VRSTS = PMC_VRSTS_MVRCERR_Msk;
        i32Ret = PMC_ERR_FAIL;
    }

    return i32Ret;
}

/**
  * @brief      Set System SRAM Power Mode
  * @param[in]  u32SRAMSel is SRAM region selection. Including :
  *             - \ref PMC_SYSRB0PC_SRAM0PMS_Msk       : 0x2010_0000 - 0x2010_FFFF
  *             - \ref PMC_SYSRB0PC_SRAM1PMS_Msk       : 0x2011_0000 - 0x2011_FFFF
  *             - \ref PMC_SYSRB0PC_SRAM2PMS_Msk       : 0x2012_0000 - 0x2012_FFFF
  *             - \ref PMC_SYSRB0PC_SRAM3PMS_Msk       : 0x2013_0000 - 0x2013_FFFF
  *             - \ref PMC_SYSRB0PC_SRAM4PMS_Msk       : 0x2014_0000 - 0x2014_FFFF
  *             - \ref PMC_SYSRB0PC_SRAM5PMS_Msk       : 0x2015_0000 - 0x2015_FFFF
  *             - \ref PMC_SYSRB0PC_SRAM6PMS_Msk       : 0x2016_0000 - 0x2016_FFFF
  *             - \ref PMC_SYSRB0PC_SRAM7PMS_Msk       : 0x2017_0000 - 0x2017_FFFF
  *             - \ref PMC_SYSRB1PC_SRAM0PMS_Msk       : 0x2018_0000 - 0x2018_FFFF
  *             - \ref PMC_SYSRB1PC_SRAM1PMS_Msk       : 0x2019_0000 - 0x2019_FFFF
  *             - \ref PMC_SYSRB1PC_SRAM2PMS_Msk       : 0x201A_0000 - 0x201A_FFFF
  *             - \ref PMC_SYSRB1PC_SRAM3PMS_Msk       : 0x201B_0000 - 0x201B_FFFF
  *             - \ref PMC_SYSRB1PC_SRAM4PMS_Msk       : 0x201C_0000 - 0x201C_FFFF
  *             - \ref PMC_SYSRB1PC_SRAM5PMS_Msk       : 0x201D_4000 - 0x201D_FFFF
  *             - \ref PMC_SYSRB1PC_SRAM6PMS_Msk       : 0x201E_8000 - 0x201E_FFFF
  *             - \ref PMC_SYSRB1PC_SRAM7PMS_Msk       : 0x201F_C000 - 0x201F_FFFF
  *             - \ref PMC_SYSRB2PC_SRAM0PMS_Msk       : 0x2020_0000 - 0x2020_FFFF
  *             - \ref PMC_SYSRB2PC_SRAM1PMS_Msk       : 0x2021_0000 - 0x2021_FFFF
  *             - \ref PMC_SYSRB2PC_SRAM2PMS_Msk       : 0x2022_0000 - 0x2022_FFFF
  *             - \ref PMC_SYSRB2PC_SRAM3PMS_Msk       : 0x2023_0000 - 0x2023_FFFF
  *             - \ref PMC_SYSRB2PC_SRAM4PMS_Msk       : 0x2024_0000 - 0x2024_FFFF
  *             - \\ref PMC_SYSRB3PC_SRAM0PMS_Msk       : 0x2030_0000 - 0x2030_1FFF  // TESTCHIP_ONLY not support
  *             - \ref PMC_LPSYSRPC_SRAM0PMS_Msk       : 0x2031_0000 - 0x2031_0FFF
  * @param[in]  u32PowerMode is SRAM power mode. Including :
  *             - \ref PMC_SYSRB0PC_SRAM_NORMAL
  *             - \ref PMC_SYSRB0PC_SRAM_RETENTION
  *             - \ref PMC_SYSRB0PC_SRAM_POWER_SHUT_DOWN
  *             - \ref PMC_SYSRB1PC_SRAM_NORMAL
  *             - \ref PMC_SYSRB1PC_SRAM_RETENTION
  *             - \ref PMC_SYSRB1PC_SRAM_POWER_SHUT_DOWN
  *             - \ref PMC_SYSRB2PC_SRAM_NORMAL
  *             - \ref PMC_SYSRB2PC_SRAM_RETENTION
  *             - \ref PMC_SYSRB2PC_SRAM_POWER_SHUT_DOWN
  *             - \\ref PMC_SYSRB3PC_SRAM_NORMAL             // TESTCHIP_ONLY not support
  *             - \\ref PMC_SYSRB3PC_SRAM_RETENTION          // TESTCHIP_ONLY not support
  *             - \\ref PMC_SYSRB3PC_SRAM_POWER_SHUT_DOWN    // TESTCHIP_ONLY not support
  *             - \ref PMC_LPSYSRPC_SRAM_NORMAL
  *             - \ref PMC_LPSYSRPC_SRAM_RETENTION
  *             - \ref PMC_LPSYSRPC_SRAM_POWER_SHUT_DOWN
  * @retval     PMC_OK          PMC operation OK.
  * @retval     PMC_ERR_FAIL    PMC operation failed.
  * @retval     PMC_ERR_TIMEOUT PMC operation abort due to timeout error.
  * @details    This function set system SRAM power mode.
  *             The register write-protection function should be disabled before using this function.
  */
int32_t PMC_SetSRAMPowerMode(uint32_t u32SRAMSel, uint32_t u32PowerMode)
{
    uint32_t u32SRAMSelPos = 0UL;
    uint32_t u32SRAMSelNum;

    /* Get system SRAM power mode setting position */
    while (u32SRAMSelPos < 30UL)
    {
        if (u32SRAMSel & (1 << u32SRAMSelPos))
        {
            break;
        }
        else
        {
            u32SRAMSelPos++;
        }
    }

    /* Set system SRAM power mode setting */
    u32SRAMSelNum = u32PowerMode >> 28;

    switch (u32SRAMSelNum)
    {
        case SRAMNum0:
            if (PMC_Wait_BusyFlag(PMC_SYSRB0PC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            PMC->SYSRB0PC = (PMC->SYSRB0PC & (~u32SRAMSel)) | (u32PowerMode << u32SRAMSelPos);
            break;

        case SRAMNum1:
            if (PMC_Wait_BusyFlag(PMC_SYSRB1PC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            PMC->SYSRB1PC = (PMC->SYSRB1PC & (~u32SRAMSel)) | (u32PowerMode << u32SRAMSelPos);
            break;

        case SRAMNum2:
            if (PMC_Wait_BusyFlag(PMC_SYSRB2PC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            PMC->SYSRB2PC = (PMC->SYSRB2PC & (~u32SRAMSel)) | (u32PowerMode << u32SRAMSelPos);
            break;
#if 0   // TESTCHIP_ONLY not support

        case SRAMNum3:
            if (PMC_Wait_BusyFlag(PMC_SYSRB3PC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            PMC->SYSRB3PC = (PMC->SYSRB3PC & (~u32SRAMSel)) | (u32PowerMode << u32SRAMSelPos);
            break;
#endif

        case SRAMNum4:
            if (PMC_Wait_BusyFlag(PMC_LPSYSRPC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            PMC->LPSYSRPC = (PMC->LPSYSRPC & (~u32SRAMSel)) | (u32PowerMode << u32SRAMSelPos);
            break;

        default:
            return PMC_ERR_FAIL;
    }

    return PMC_OK;
}

/**
  * @brief      Enter to Power-down mode
  * @details    This function is used to let system enter to Power-down mode. \n
  *             The register write-protection function should be disabled before using this function.
  */
void PMC_PowerDown(void)
{
    volatile uint32_t u32SysTickTICKINT = 0, u32TCTL48M = 0, u32TCTL12M = 0, u32TCTLMIRC = 0;

    /* Set the processor uses deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled */
    PMC->PWRCTL |= (PMC_PWRCTL_PDEN_Msk);

    /* Store SysTick interrupt and HIRC auto trim setting */
    u32SysTickTICKINT = SysTick->CTRL & SysTick_CTRL_TICKINT_Msk;
    u32TCTL48M = SYS->TCTL48M;
    u32TCTL12M = SYS->TCTL12M;
    u32TCTLMIRC = SYS->TCTLMIRC;

    /* Disable SysTick interrupt and HIRC auto trim */
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    SYS->TCTL48M &= ~SYS_TCTL48M_FREQSEL_Msk;
    SYS->TCTL12M &= ~SYS_TCTL12M_FREQSEL_Msk;
    SYS->TCTLMIRC &= ~SYS_TCTLMIRC_FREQSEL_Msk;

    /* Chip enter Power-down mode after CPU run WFI instruction */
    __WFI();

    /* Restore SysTick interrupt and HIRC auto trim setting */
    if (u32SysTickTICKINT) SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

    SYS->TCTL48M = u32TCTL48M;
    SYS->TCTL12M = u32TCTL12M;
    SYS->TCTLMIRC = u32TCTLMIRC;
}

/**
  * @brief      Enter to Idle mode
  * @details    This function let system enter to Idle mode. \n
  *             The register write-protection function should be disabled before using this function.
  */
void PMC_Idle(void)
{
    /* Set the processor uses sleep as its low power mode */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    /* Set chip in idle mode because of WFI command */
    PMC->PWRCTL &= ~PMC_PWRCTL_PDEN_Msk;

    /* Chip enter idle mode after CPU run WFI instruction */
    __WFI();
}

/**
  * @brief      Power-down mode selected
  * @param[in]  u32PowerDownMode is power down mode index. Including :
  *             - \ref PMC_NPD0
  *             - \ref PMC_NPD1
  *             - \\ref  PMC_NPD2 // TESTCHIP_ONLY not support
  *             - \\ref  PMC_NPD3 // TESTCHIP_ONLY not support
  *             - \\ref  PMC_NPD4 // TESTCHIP_ONLY not support
  *             - \ref PMC_SPD0
  *             - \\ref  PMC_SPD1 // TESTCHIP_ONLY not support
  *             - \ref PMC_DPD0
  *             - \\ref  PMC_DPD1 // TESTCHIP_ONLY not support
  * @param[in]  u32PowerLevel is power level setting. Including :
  *             - \ref PMC_PLCTL_PLSEL_PL0  : Supports system clock up to 240MHz.
  *             - \ref PMC_PLCTL_PLSEL_PL1  : Supports system clock up to 200MHz.
  *             Power level set value is ignored in PMC_NPD2/PMC_NPD4/PMC_SPD1/PMC_DPD0/PMC_DPD1 power-down mode.
  * @retval     PMC_OK          PMC operation OK.
  * @retval     PMC_ERR_TIMEOUT PMC operation abort due to timeout error.
  * @details    This function is used to set power-down mode.
  *             The register write-protection function should be disabled before using this function.
  */
int32_t PMC_SetPowerDownMode(uint32_t u32PowerDownMode, uint32_t u32PowerLevel)
{
    if (PMC_Wait_BusyFlag(PMC_PWRCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    PMC->PWRCTL = (PMC->PWRCTL & ~(PMC_PWRCTL_D0PGEN_Msk | PMC_PWRCTL_D1PGEN_Msk | PMC_PWRCTL_D2PGEN_Msk | PMC_PWRCTL_D3PGEN_Msk
                                   | PMC_PWRCTL_VDROPEN_Msk | PMC_PWRCTL_FWEN_Msk));

    switch (u32PowerDownMode)
    {
        case PMC_NPD0:
            if (PMC_SetPowerLevel(u32PowerLevel) != 0) return PMC_ERR_TIMEOUT;

            if (PMC_Wait_BusyFlag(PMC_PWRCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            PMC->PWRCTL |= PMC_PWRCTL_FWEN_Msk;
            break;

        case PMC_NPD1:
            if (PMC_SetPowerLevel(u32PowerLevel) != 0) return PMC_ERR_TIMEOUT;

            break;
#if 0   // TESTCHIP_ONLY not support

        case PMC_NPD2:
            if (PMC_Wait_BusyFlag(PMC_PWRCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            PMC->PWRCTL |= PMC_PWRCTL_VDROPEN_Msk;
            break;

        case PMC_NPD3:
            if (PMC_SetPowerLevel(u32PowerLevel) != 0) return PMC_ERR_TIMEOUT;

            if (PMC_Wait_BusyFlag(PMC_PWRCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            PMC->PWRCTL |= (PMC_PWRCTL_D0PGEN_Msk | PMC_PWRCTL_D1PGEN_Msk);
            break;

        case PMC_NPD4:
            if (PMC_Wait_BusyFlag(PMC_PWRCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            PMC->PWRCTL |= (PMC_PWRCTL_D0PGEN_Msk | PMC_PWRCTL_D1PGEN_Msk | PMC_PWRCTL_VDROPEN_Msk);
            break;
#endif

        case PMC_SPD0:
            if (PMC_SetPowerLevel(u32PowerLevel) != 0) return PMC_ERR_TIMEOUT;

            if (PMC_Wait_BusyFlag(PMC_PWRCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            PMC->PWRCTL |= (PMC_PWRCTL_D0PGEN_Msk | PMC_PWRCTL_D1PGEN_Msk | PMC_PWRCTL_D2PGEN_Msk);
            break;
#if 0   // TESTCHIP_ONLY not support

        case PMC_SPD1:
            if (PMC_Wait_BusyFlag(PMC_PWRCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            PMC->PWRCTL |= (PMC_PWRCTL_D0PGEN_Msk | PMC_PWRCTL_D1PGEN_Msk | PMC_PWRCTL_D2PGEN_Msk | PMC_PWRCTL_VDROPEN_Msk);
            break;
#endif

        case PMC_DPD0:
            if (PMC_Wait_BusyFlag(PMC_PWRCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            PMC->PWRCTL |= (PMC_PWRCTL_D0PGEN_Msk | PMC_PWRCTL_D1PGEN_Msk | PMC_PWRCTL_D2PGEN_Msk | PMC_PWRCTL_D3PGEN_Msk | PMC_PWRCTL_FWEN_Msk);
            break;
#if 0   // TESTCHIP_ONLY not support

        case PMC_DPD1:
            if (PMC_Wait_BusyFlag(PMC_PWRCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            PMC->PWRCTL |= (PMC_PWRCTL_D0PGEN_Msk | PMC_PWRCTL_D1PGEN_Msk | PMC_PWRCTL_D2PGEN_Msk | PMC_PWRCTL_D3PGEN_Msk);
            break;
#endif
    }

    return PMC_OK;
}

/**
 * @brief       Set Wake-up pin trigger type at Power down mode
 * @param[in]   u32TriggerType Wake-up pin trigger type
 *              - \ref PMC_WKPIN0_RISING
 *              - \ref PMC_WKPIN0_FALLING
 *              - \ref PMC_WKPIN0_BOTHEDGE
 *              - \ref PMC_WKPIN1_RISING
 *              - \ref PMC_WKPIN1_FALLING
 *              - \ref PMC_WKPIN1_BOTHEDGE
 *              - \ref PMC_WKPIN2_RISING
 *              - \ref PMC_WKPIN2_FALLING
 *              - \ref PMC_WKPIN2_BOTHEDGE
 *              - \ref PMC_WKPIN3_RISING
 *              - \ref PMC_WKPIN3_FALLING
 *              - \ref PMC_WKPIN3_BOTHEDGE
 *              - \ref PMC_WKPIN4_RISING
 *              - \ref PMC_WKPIN4_FALLING
 *              - \ref PMC_WKPIN4_BOTHEDGE
 *              - \ref PMC_WKPIN5_RISING
 *              - \ref PMC_WKPIN5_FALLING
 *              - \ref PMC_WKPIN5_BOTHEDGE
 * @details     This function is used to enable Wake-up pin trigger type.
 *              The register write-protection function should be disabled before using this function.
 */
void PMC_EnableWKPIN(uint32_t u32TriggerType)
{
    uint32_t u32Pin1, u32Pin2, u32Pin3, u32Pin4, u32Pin5;

    /* Get wake-up pin configuration */
    u32Pin1 = ((u32TriggerType) & PMC_PINWKCTL_WKPINEN1_Msk);
    u32Pin2 = ((u32TriggerType) & PMC_PINWKCTL_WKPINEN2_Msk);
    u32Pin3 = ((u32TriggerType) & PMC_PINWKCTL_WKPINEN3_Msk);
    u32Pin4 = ((u32TriggerType) & PMC_PINWKCTL_WKPINEN4_Msk);
    u32Pin5 = ((u32TriggerType) & PMC_PINWKCTL_WKPINEN5_Msk);

    /* Set wake-up pin configuration */
    if (u32Pin1)
    {
        PMC->PINWKCTL = (PMC->PINWKCTL & ~(PMC_PINWKCTL_WKPINEN1_Msk)) | u32TriggerType;
    }
    else if (u32Pin2)
    {
        PMC->PINWKCTL = (PMC->PINWKCTL & ~(PMC_PINWKCTL_WKPINEN2_Msk)) | u32TriggerType;
    }
    else if (u32Pin3)
    {
        PMC->PINWKCTL = (PMC->PINWKCTL & ~(PMC_PINWKCTL_WKPINEN3_Msk)) | u32TriggerType;
    }
    else if (u32Pin4)
    {
        PMC->PINWKCTL = (PMC->PINWKCTL & ~(PMC_PINWKCTL_WKPINEN4_Msk)) | u32TriggerType;
    }
    else if (u32Pin5)
    {
        PMC->PINWKCTL = (PMC->PINWKCTL & ~(PMC_PINWKCTL_WKPINEN5_Msk)) | u32TriggerType;
    }
    else
    {
        PMC->PINWKCTL = (PMC->PINWKCTL & ~(PMC_PINWKCTL_WKPINEN0_Msk)) | u32TriggerType;
    }
}

/**
 * @brief       Get power manager wake up source
 * @param[in]   None
 * @return      Interrupt Status
 * @details     This function get power manager wake up source.
 */
uint32_t PMC_GetPMCWKSrc(void)
{
    return (PMC->INTSTS);
}

/**
 * @brief       Set specified GPIO as wake up source or triggrt source
 * @param[in]   u32Port GPIO port.
 *              - \ref PMC_TGPIN_PA
 *              - \ref PMC_TGPIN_PB
 *              - \ref PMC_TGPIN_PC
 *              - \ref PMC_TGPIN_PD
 * @param[in]   u32Pin The pin of specified GPIO port. It could be 0 ~ 15.
 * @param[in]   u32TriggerType TG pin trigger type
 *              - \ref PMC_TGPIN_RISING
 *              - \ref PMC_TGPIN_FALLING
 *              - \ref PMC_TGPIN_BOTH
 * @param[in]   u32DebounceEn TG pin de-bounce function
 *              - \ref PMC_TGPIN_DEBOUNCEEN
 *              - \ref PMC_TGPIN_DEBOUNCEDIS
 * @param[in]   u32WakeupEn TG Pin wake-up function
 *              - \ref PMC_TGPIN_WAKEUP_ENABLE
 *              - \ref PMC_TGPIN_WAKEUP_DISABLE
 * @details     This function is used to set specified GPIO as trigger or wake up source.
 */
void PMC_EnableTGPin(uint32_t u32Port, uint32_t u32Pin, uint32_t u32TriggerType, uint32_t u32DebounceEn, uint32_t u32WakeupEn)
{
    uint32_t u32tmpAddr = 0UL;
    uint32_t u32tmpVal = 0UL;

    /* GPx TG Pin Select */
    u32tmpAddr = (uint32_t)&PMC->GPATGCTL;
    u32tmpAddr += (0x4UL * u32Port);

    u32tmpVal = inpw((uint32_t *)u32tmpAddr);

    u32tmpVal = (u32tmpVal & ~(PMC_GPATGCTL_TGPSEL_Msk | PMC_GPATGCTL_TGPREN_Msk | PMC_GPATGCTL_TGPFEN_Msk | PMC_GPATGCTL_TGPDBEN_Msk | PMC_GPATGCTL_TGPWKEN_Msk)) |
                (u32Pin << PMC_GPATGCTL_TGPSEL_Pos) | u32TriggerType | u32DebounceEn | u32WakeupEn;
    outpw((uint32_t *)u32tmpAddr, u32tmpVal);
}

/**
 * @brief       Enable standby wake-up timer and set Time-out interval.
 * @param[in]   u32Interval   The Time-out Interval selection. It could be
 *                             - \ref PMC_STMRWKCTL_STMRIS_512
 *                             - \ref PMC_STMRWKCTL_STMRIS_1024
 *                             - \ref PMC_STMRWKCTL_STMRIS_2048
 *                             - \ref PMC_STMRWKCTL_STMRIS_4096
 *                             - \ref PMC_STMRWKCTL_STMRIS_8192
 *                             - \ref PMC_STMRWKCTL_STMRIS_16384
 *                             - \ref PMC_STMRWKCTL_STMRIS_32768
 *                             - \ref PMC_STMRWKCTL_STMRIS_65536
 *                             - \ref PMC_STMRWKCTL_STMRIS_131072
 *                             - \ref PMC_STMRWKCTL_STMRIS_262144
 *                             - \ref PMC_STMRWKCTL_STMRIS_524288
 *                             - \ref PMC_STMRWKCTL_STMRIS_1048576
 *                             - \ref PMC_STMRWKCTL_STMRIS_2097152
 *                             - \ref PMC_STMRWKCTL_STMRIS_4194304
 * @retval      PMC_OK          PMC operation OK.
 * @retval      PMC_ERR_TIMEOUT PMC operation abort due to timeout error.
 * @details     This function enable standby wake-up timer and set Time-out interval.
                The register write-protection function should be disabled before using this function.
 */
int32_t PMC_EnableSTMR(uint32_t u32Interval)
{
    if (PMC_Wait_BusyFlag(PMC_STMRWKCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    PMC->STMRWKCTL = ((PMC->STMRWKCTL & ~PMC_STMRWKCTL_STMRIS_Msk) | u32Interval);

    if (PMC_Wait_BusyFlag(PMC_STMRWKCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    PMC->STMRWKCTL |= PMC_STMRWKCTL_STMREN_Msk;

    return PMC_OK;
}

/**
 * @brief       Disable standby wake-up timer.
 * @retval      PMC_OK          PMC operation OK.
 * @retval      PMC_ERR_TIMEOUT PMC operation abort due to timeout error.
 * @details     This function isable standby wake-up timer.
                The register write-protection function should be disabled before using this function.
 */
int32_t PMC_DisableSTMR(void)
{
    if (PMC_Wait_BusyFlag(PMC_STMRWKCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    PMC->STMRWKCTL &= ~PMC_STMRWKCTL_STMREN_Msk;

    return PMC_OK;
}

/**
 * @brief       Check PMC busy flag
 * @param[in]   PMCBusyFlagAddr   The Busy Flag selection. It could be
 *              - \ref PMC_PWRCTL_BUSY_FLAG
 *              - \ref PMC_PLCTL_BUSY_FLAG
 *              - \ref PMC_PLSTS_BUSY_FLAG
 *              - \ref PMC_VRCTL_BUSY_FLAG
 *              - \ref PMC_VRSTS_BUSY_FLAG
 *              - \ref PMC_STMRWKCTL_BUSY_FLAG
 *              - \ref PMC_SYSRB0PC_BUSY_FLAG
 *              - \ref PMC_SYSRB1PC_BUSY_FLAG
 *              - \ref PMC_SYSRB2PC_BUSY_FLAG
 *              - \ref PMC_SYSRB3PC_BUSY_FLAG
 *              - \ref PMC_LPSYSRPC_BUSY_FLAG
 *              - \ref PMC_KSRPC_BUSY_FLAG
 * @retval      PMC_OK          PMC operation OK.
 * @retval      PMC_ERR_TIMEOUT PMC operation abort due to timeout error.
 * @details     This function is check PMC busy flag.
 */
int32_t PMC_Wait_BusyFlag(uint32_t PMCBusyFlagAddr)
{
    int32_t i32TimeOutCnt = SystemCoreClock >> 1; /* 500ms time-out */

    while (*(volatile uint32_t *)PMCBusyFlagAddr & BIT31)
    {
        if (i32TimeOutCnt-- <= 0) return PMC_ERR_TIMEOUT;
    }

    return PMC_OK;
}

/** @} end of group PMC_EXPORTED_FUNCTIONS */
/** @} end of group PMC_Driver */
/** @} end of group Standard_Driver */
