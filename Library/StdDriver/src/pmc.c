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
  * @details    This function is used to select power level.
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
  *             - \ref PMC_SYSRB3PC_SRAM0PMS_Msk       : 0x2030_0000 - 0x2030_1FFF
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
  *             - \ref PMC_SYSRB3PC_SRAM_NORMAL
  *             - \ref PMC_SYSRB3PC_SRAM_RETENTION
  *             - \ref PMC_SYSRB3PC_SRAM_POWER_SHUT_DOWN
  *             - \ref PMC_LPSYSRPC_SRAM_NORMAL
  *             - \ref PMC_LPSYSRPC_SRAM_RETENTION
  *             - \ref PMC_LPSYSRPC_SRAM_POWER_SHUT_DOWN
  * @retval     PMC_OK          PMC operation OK.
  * @retval     PMC_ERR_FAIL    PMC operation failed.
  * @retval     PMC_ERR_TIMEOUT PMC operation abort due to timeout error.
  * @details    This function is used to set system SRAM power mode.
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

            if (PMC_Wait_BusyFlag(PMC_SYSRB0PC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            break;

        case SRAMNum1:
            if (PMC_Wait_BusyFlag(PMC_SYSRB1PC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            PMC->SYSRB1PC = (PMC->SYSRB1PC & (~u32SRAMSel)) | (u32PowerMode << u32SRAMSelPos);

            if (PMC_Wait_BusyFlag(PMC_SYSRB1PC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            break;

        case SRAMNum2:
            if (PMC_Wait_BusyFlag(PMC_SYSRB2PC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            PMC->SYSRB2PC = (PMC->SYSRB2PC & (~u32SRAMSel)) | (u32PowerMode << u32SRAMSelPos);

            if (PMC_Wait_BusyFlag(PMC_SYSRB2PC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            break;

        case SRAMNum3:
            if (PMC_Wait_BusyFlag(PMC_SYSRB3PC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            PMC->SYSRB3PC = (PMC->SYSRB3PC & (~u32SRAMSel)) | (u32PowerMode << u32SRAMSelPos);

            if (PMC_Wait_BusyFlag(PMC_SYSRB3PC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            break;

        case SRAMNum4:
            if (PMC_Wait_BusyFlag(PMC_LPSYSRPC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            PMC->LPSYSRPC = (PMC->LPSYSRPC & (~u32SRAMSel)) | (u32PowerMode << u32SRAMSelPos);

            if (PMC_Wait_BusyFlag(PMC_LPSYSRPC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

            break;

        default:
            return PMC_ERR_FAIL;
    }

    return PMC_OK;
}

/**
  * @brief      Set CCAP SRAM Power Mode
  * @param[in]  u32PowerMode is SRAM power mode. Including :
  *             - \ref PMC_SRAM_NORMAL
  *             - \ref PMC_SRAM_RETENTION
  *             - \ref PMC_SRAM_POWER_SHUT_DOWN
  * @retval     PMC_OK          PMC operation OK.
  * @retval     PMC_ERR_TIMEOUT PMC operation abort due to timeout error.
  * @details    This function is used to set CCAP SRAM power mode.
  *             The register write-protection function should be disabled before using this function.
  */
int32_t PMC_SetCCAP_SRAMPowerMode(uint32_t u32PowerMode)
{
    if (PMC_Wait_BusyFlag(PMC_CCAPRPC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    PMC->CCAPRPC = (PMC->CCAPRPC & (~PMC_CCAPRPC_CCAP0SRPMS_Msk)) | (u32PowerMode << PMC_CCAPRPC_CCAP0SRPMS_Pos);

    if (PMC_Wait_BusyFlag(PMC_CCAPRPC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    return PMC_OK;
}

/**
  * @brief      Set DMIC SRAM Power Mode
  * @param[in]  u32PowerMode is SRAM power mode. Including :
  *             - \ref PMC_SRAM_NORMAL
  *             - \ref PMC_SRAM_RETENTION
  *             - \ref PMC_SRAM_POWER_SHUT_DOWN
  * @retval     PMC_OK          PMC operation OK.
  * @retval     PMC_ERR_TIMEOUT PMC operation abort due to timeout error.
  * @details    This function is used to set DMIC SRAM power mode.
  *             The register write-protection function should be disabled before using this function.
  */
int32_t PMC_SetDMIC_SRAMPowerMode(uint32_t u32PowerMode)
{
    if (PMC_Wait_BusyFlag(PMC_DMICRPC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    PMC->DMICRPC = (PMC->DMICRPC & (~PMC_DMICRPC_DMIC0SRPMS_Msk)) | (u32PowerMode << PMC_DMICRPC_DMIC0SRPMS_Pos);

    if (PMC_Wait_BusyFlag(PMC_DMICRPC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    return PMC_OK;
}

/**
  * @brief      Set KS SRAM Power Mode
  * @param[in]  u32PowerMode is SRAM power mode. Including :
  *             - \ref PMC_SRAM_NORMAL
  *             - \ref PMC_SRAM_RETENTION
  *             - \ref PMC_SRAM_POWER_SHUT_DOWN
  * @retval     PMC_OK          PMC operation OK.
  * @retval     PMC_ERR_TIMEOUT PMC operation abort due to timeout error.
  * @details    This function is used to set KS SRAM power mode.
  *             The register write-protection function should be disabled before using this function.
  */
int32_t PMC_SetKS_SRAMPowerMode(uint32_t u32PowerMode)
{
    if (PMC_Wait_BusyFlag(PMC_KSRPC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    PMC->KSRPC = (PMC->KSRPC & (~PMC_KSRPC_KSSRPMS_Msk)) | (u32PowerMode << PMC_KSRPC_KSSRPMS_Pos);

    if (PMC_Wait_BusyFlag(PMC_KSRPC_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    return PMC_OK;
}

/**
  * @brief      Enter to Power-down mode
  * @details    This function is used to let system enter to Power-down mode. \n
  *             The register write-protection function should be disabled before using this function.
  *             Before entering powerdown, user must confirm the auto operation has been completed.
  */
void PMC_PowerDown(void)
{
    volatile uint32_t u32SysTickTICKINT = 0, u32TCTL48M = 0, u32TCTL12M = 0, u32TCTLMIRC = 0;

    /* Set the processor uses deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

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

    /* Clear deep sleep mode selection */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

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

    /* Chip enter idle mode after CPU run WFI instruction */
    __WFI();
}

/**
  * @brief      Power-down mode selected
  * @param[in]  u32PowerDownMode is power down mode index. Including :
  *             - \ref PMC_NPD0
  *             - \ref PMC_NPD1
  *             - \ref PMC_NPD2
  *             - \ref PMC_NPD3
  *             - \ref PMC_NPD4
  *             - \ref PMC_SPD0
  *             - \ref PMC_SPD1
  *             - \ref PMC_DPD
  * @param[in]  u32PowerLevel is power level setting. Including :
  *             - \ref PMC_PLCTL_PLSEL_PL0  : Supports system clock up to 220MHz.
  *             - \ref PMC_PLCTL_PLSEL_PL1  : Supports system clock up to 200MHz.
  *             The value set by u32PowerLevel is ignored in PMC_NPD2/PMC_NPD4/PMC_SPD1/PMC_DPD power-down mode.
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

    if (PMC_Wait_BusyFlag(PMC_PWRCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    switch (u32PowerDownMode)
    {
        case PMC_NPD0:
            if (PMC_SetPowerLevel(u32PowerLevel) != 0) return PMC_ERR_TIMEOUT;

            PMC->PWRCTL |= PMC_PWRCTL_FWEN_Msk;
            break;

        case PMC_NPD1:

            /* Workaround */
            if (PMC_Wait_BusyFlag(SYS_BASE + 0xF4C) != 0) return PMC_ERR_TIMEOUT;

            outp32(SYS_BASE + 0xF4C, inp32(SYS_BASE + 0xF4C) | BIT14);

            if (PMC_Wait_BusyFlag(SYS_BASE + 0xF4C) != 0) return PMC_ERR_TIMEOUT;

            if (PMC_SetPowerLevel(u32PowerLevel) != 0) return PMC_ERR_TIMEOUT;

            break;

        case PMC_NPD2:
            PMC->PWRCTL |= PMC_PWRCTL_VDROPEN_Msk;
            break;

        case PMC_NPD3:

            /* Workaround */
            if (PMC_Wait_BusyFlag(SYS_BASE + 0xF4C) != 0) return PMC_ERR_TIMEOUT;

            outp32(SYS_BASE + 0xF4C, inp32(SYS_BASE + 0xF4C) | BIT14);

            if (PMC_Wait_BusyFlag(SYS_BASE + 0xF4C) != 0) return PMC_ERR_TIMEOUT;

            if (PMC_SetPowerLevel(u32PowerLevel) != 0) return PMC_ERR_TIMEOUT;

            PMC->PWRCTL |= (PMC_PWRCTL_D0PGEN_Msk | PMC_PWRCTL_D1PGEN_Msk);
            break;

        case PMC_NPD4:
            PMC->PWRCTL |= (PMC_PWRCTL_D0PGEN_Msk | PMC_PWRCTL_D1PGEN_Msk | PMC_PWRCTL_VDROPEN_Msk);
            break;

        case PMC_SPD0:
            if (PMC_SetPowerLevel(u32PowerLevel) != 0) return PMC_ERR_TIMEOUT;

            PMC->PWRCTL |= (PMC_PWRCTL_D0PGEN_Msk | PMC_PWRCTL_D1PGEN_Msk | PMC_PWRCTL_D2PGEN_Msk);
            break;

        case PMC_SPD1:
            PMC->PWRCTL |= (PMC_PWRCTL_D0PGEN_Msk | PMC_PWRCTL_D1PGEN_Msk | PMC_PWRCTL_D2PGEN_Msk | PMC_PWRCTL_VDROPEN_Msk);
            break;

        case PMC_DPD:
            PMC->PWRCTL |= (PMC_PWRCTL_D0PGEN_Msk | PMC_PWRCTL_D1PGEN_Msk | PMC_PWRCTL_D2PGEN_Msk | PMC_PWRCTL_D3PGEN_Msk | PMC_PWRCTL_FWEN_Msk);
            break;

        default:
            break;
    }

    if (PMC_Wait_BusyFlag(PMC_PWRCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

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
 * @details     This function is used to get power management wake up source.
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
int32_t PMC_EnableTGPin(uint32_t u32Port, uint32_t u32Pin, uint32_t u32TriggerType, uint32_t u32DebounceEn, uint32_t u32WakeupEn)
{
    uint32_t u32tmpAddr = 0UL;
    uint32_t u32tmpVal = 0UL;

    /* GPx TG Pin Select */
    u32tmpAddr = (uint32_t)&PMC->GPATGCTL;
    u32tmpAddr += (0x4UL * u32Port);

    u32tmpVal = inpw((uint32_t *)u32tmpAddr);

    if (PMC_Wait_BusyFlag(u32tmpAddr) != 0) return PMC_ERR_TIMEOUT;

    u32tmpVal = (u32tmpVal & ~(PMC_GPATGCTL_TGPSEL_Msk | PMC_GPATGCTL_TGPREN_Msk | PMC_GPATGCTL_TGPFEN_Msk | PMC_GPATGCTL_TGPDBEN_Msk | PMC_GPATGCTL_TGPWKEN_Msk)) |
                (u32Pin << PMC_GPATGCTL_TGPSEL_Pos) | u32TriggerType | u32DebounceEn | u32WakeupEn;
    outpw((uint32_t *)u32tmpAddr, u32tmpVal);

    if (PMC_Wait_BusyFlag(u32tmpAddr) != 0) return PMC_ERR_TIMEOUT;

    return PMC_OK;
}

/**
 * @brief       Disable specified GPIO as wake up source or triggrt source
 * @param[in]   u32Port GPIO port.
 *              - \ref PMC_TGPIN_PA
 *              - \ref PMC_TGPIN_PB
 *              - \ref PMC_TGPIN_PC
 *              - \ref PMC_TGPIN_PD
 * @details     This function is used to set specified GPIO as trigger or wake up source.
 */
int32_t PMC_DisableTGPin(uint32_t u32Port)
{
    uint32_t u32tmpAddr = 0UL;
    uint32_t u32tmpVal = 0UL;

    /* GPx TG Pin Select */
    u32tmpAddr = (uint32_t)&PMC->GPATGCTL;
    u32tmpAddr += (0x4UL * u32Port);

    u32tmpVal = inpw((uint32_t *)u32tmpAddr);

    if (PMC_Wait_BusyFlag(u32tmpAddr) != 0) return PMC_ERR_TIMEOUT;

    u32tmpVal = (u32tmpVal & ~(PMC_GPATGCTL_TGPSEL_Msk | PMC_GPATGCTL_TGPREN_Msk | PMC_GPATGCTL_TGPFEN_Msk | PMC_GPATGCTL_TGPDBEN_Msk | PMC_GPATGCTL_TGPWKEN_Msk));

    outpw((uint32_t *)u32tmpAddr, u32tmpVal);

    if (PMC_Wait_BusyFlag(u32tmpAddr) != 0) return PMC_ERR_TIMEOUT;

    return PMC_OK;
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
 * @details     This function is used to enable standby wake-up timer and set Time-out interval.
                The register write-protection function should be disabled before using this function.
 */
int32_t PMC_EnableSTMR(uint32_t u32Interval)
{
    if (PMC_Wait_BusyFlag(PMC_STMRWKCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    PMC->STMRWKCTL = ((PMC->STMRWKCTL & ~PMC_STMRWKCTL_STMRIS_Msk) | u32Interval);

    if (PMC_Wait_BusyFlag(PMC_STMRWKCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    PMC->STMRWKCTL |= PMC_STMRWKCTL_STMREN_Msk;

    if (PMC_Wait_BusyFlag(PMC_STMRWKCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    return PMC_OK;
}

/**
 * @brief       Disable standby wake-up timer.
 * @retval      PMC_OK          PMC operation OK.
 * @retval      PMC_ERR_TIMEOUT PMC operation abort due to timeout error.
 * @details     This function is used to disable standby wake-up timer.
                The register write-protection function should be disabled before using this function.
 */
int32_t PMC_DisableSTMR(void)
{
    if (PMC_Wait_BusyFlag(PMC_STMRWKCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    PMC->STMRWKCTL &= ~PMC_STMRWKCTL_STMREN_Msk;

    if (PMC_Wait_BusyFlag(PMC_STMRWKCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    return PMC_OK;
}

/**
 * @brief       Set GPIO Trigger Pin De-bounce Sampling Cycle.
 * @param[in]   u32Sel   The Time-out Interval selection. It could be
 *                             - \ref PMC_IOTGDBCTL_IOTGDBSEL_1
 *                             - \ref PMC_IOTGDBCTL_IOTGDBSEL_2
 *                             - \ref PMC_IOTGDBCTL_IOTGDBSEL_4
 *                             - \ref PMC_IOTGDBCTL_IOTGDBSEL_8
 *                             - \ref PMC_IOTGDBCTL_IOTGDBSEL_16
 *                             - \ref PMC_IOTGDBCTL_IOTGDBSEL_32
 *                             - \ref PMC_IOTGDBCTL_IOTGDBSEL_64
 *                             - \ref PMC_IOTGDBCTL_IOTGDBSEL_128
 *                             - \ref PMC_IOTGDBCTL_IOTGDBSEL_256
 *                             - \ref PMC_IOTGDBCTL_IOTGDBSEL_512
 *                             - \ref PMC_IOTGDBCTL_IOTGDBSEL_1024
 *                             - \ref PMC_IOTGDBCTL_IOTGDBSEL_2048
 *                             - \ref PMC_IOTGDBCTL_IOTGDBSEL_4096
 *                             - \ref PMC_IOTGDBCTL_IOTGDBSEL_8192
 *                             - \ref PMC_IOTGDBCTL_IOTGDBSEL_16384
 *                             - \ref PMC_IOTGDBCTL_IOTGDBSEL_32768
 * @retval      PMC_OK          PMC operation OK.
 * @retval      PMC_ERR_TIMEOUT PMC operation abort due to timeout error.
 * @details     This function is used to set GPIO Trigger Pin De-bounce Sampling Cycle.
 */
int32_t PMC_SetTGPinDebounce(uint32_t u32Sel)
{
    if (PMC_Wait_BusyFlag(PMC_IOTGDBCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    PMC->IOTGDBCTL = ((PMC->IOTGDBCTL & ~PMC_IOTGDBCTL_IOTGDBSEL_Msk) | u32Sel);

    if (PMC_Wait_BusyFlag(PMC_IOTGDBCTL_BUSY_FLAG) != 0) return PMC_ERR_TIMEOUT;

    return PMC_OK;
}

/**
 * @brief       Check PMC busy flag
 * @param[in]   PMCBusyFlagAddr   The Busy Flag selection. It could be
 *              - \ref PMC_PWRCTL_BUSY_FLAG
 *              - \ref PMC_PLCTL_BUSY_FLAG
 *              - \ref PMC_PLSTS_BUSY_FLAG
 *              - \ref PMC_IOTGDBCTL_BUSY_FLAG
 *              - \ref PMC_GPATGCTL_BUSY_FLAG
 *              - \ref PMC_GPBTGCTL_BUSY_FLAG
 *              - \ref PMC_GPCTGCTL_BUSY_FLAG
 *              - \ref PMC_GPDTGCTL_BUSY_FLAG
 *              - \ref PMC_STMRWKCTL_BUSY_FLAG
 *              - \ref PMC_SYSRB0PC_BUSY_FLAG
 *              - \ref PMC_SYSRB1PC_BUSY_FLAG
 *              - \ref PMC_SYSRB2PC_BUSY_FLAG
 *              - \ref PMC_SYSRB3PC_BUSY_FLAG
 *              - \ref PMC_LPSYSRPC_BUSY_FLAG
 *              - \ref PMC_CCAPRPC_BUSY_FLAG
 *              - \ref PMC_DMICRPC_BUSY_FLAG
 *              - \ref PMC_KSRPC_BUSY_FLAG
 * @retval      PMC_OK          PMC operation OK.
 * @retval      PMC_ERR_TIMEOUT PMC operation abort due to timeout error.
 * @details     This function is used to check PMC busy flag.
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
