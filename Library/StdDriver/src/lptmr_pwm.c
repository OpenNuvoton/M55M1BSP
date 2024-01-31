/**************************************************************************//**
 * @file     lptmr_pwm.c
 * @version  V1.00
 * @brief    LPTMR PWM Controller(LPTMR_PWM) driver source file
 *
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LPTPWM_Driver LPTMR_PWM Driver
  @{
*/

/** @addtogroup LPTPWM_EXPORTED_FUNCTIONS LPTMR_PWM Exported Functions
  @{
*/

/**
  * @brief      Configure LPTPWM Output Frequency and Duty Cycle
  *
  * @param[in]  lptmr           The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  * @param[in]  u32Frequency    Target generator frequency.
  * @param[in]  u32DutyCycle    Target generator duty cycle percentage. Valid range are between 0~100. 10 means 10%, 20 means 20%...
  *
  * @return     Nearest frequency clock in nano second
  *
  * @details    This API is used to configure LPTPWM output frequency and duty cycle in up count type and auto-reload operation mode.
  * @note       This API is only available if LPTMR PWM counter clock source is from LPTMRx_CLK.
  */
uint32_t LPTPWM_ConfigOutputFreqAndDuty(LPTMR_T *lptmr, uint32_t u32Frequency, uint32_t u32DutyCycle)
{
    uint32_t u32PWMClockFreq, u32TargetFreq = 0UL, u32Src = 0UL;
    uint32_t u32Prescaler = 0x100UL, u32Period;
    const uint32_t au32Clk[] = {0UL, __LXT, __LIRC, __MIRC, __HIRC, 0UL};

    if (lptmr == LPTMR0)
    {
        u32Src = (CLK->LPTMRSEL & CLK_LPTMRSEL_LPTMR0SEL_Msk) >> CLK_LPTMRSEL_LPTMR0SEL_Pos;
    }
    else if (lptmr == LPTMR1)
    {
        u32Src = (CLK->LPTMRSEL & CLK_LPTMRSEL_LPTMR1SEL_Msk) >> CLK_LPTMRSEL_LPTMR1SEL_Pos;
    }
    else
        return (u32TargetFreq);

    if (u32Src == 0UL)
    {
        u32PWMClockFreq = CLK_GetPCLK4Freq();
    }
    else if (u32Src == 3UL)
    {
        u32PWMClockFreq = CLK_GetMIRCFreq();
    }
    else
    {
        u32PWMClockFreq = au32Clk[u32Src];
    }

    /* Calculate u8PERIOD and u8PSC */
    for (u32Prescaler = 1; u32Prescaler <= 0x100UL; u32Prescaler++)
    {
        u32Period = (u32PWMClockFreq / u32Prescaler) / u32Frequency;

        /* If target u32Period is larger than 0x10000, need to use a larger prescaler */
        if (u32Period > 0x10000UL)
            continue;

        break;
    }

    if (u32Prescaler == 0x101UL)
    {
        u32Prescaler = 0x100UL;
        u32Period = 0x10000UL;
    }

    /* Store return value here 'cos we're gonna change u32Prescaler & u32Period to the real value to fill into register */
    u32TargetFreq = (u32PWMClockFreq / u32Prescaler) / u32Period;

    /* Set PWM to auto-reload mode */
    lptmr->PWMCTL = (lptmr->PWMCTL & ~LPTMR_PWMCTL_CNTMODE_Msk) | (LPTPWM_AUTO_RELOAD_MODE << LPTMR_PWMCTL_CNTMODE_Pos);

    /* Convert to real register value */
    LPTPWM_SET_PRESCALER(lptmr, (u32Prescaler - 1UL));

    LPTPWM_SET_PERIOD(lptmr, (u32Period - 1UL));
    LPTPWM_SET_CMPDAT(lptmr, (u32DutyCycle * u32Period) / 100UL);

    return (u32TargetFreq);
}

/**
  * @brief      Enable LPTPWM Counter
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to enable LPTPWM generator and start counter counting.
  */
void LPTPWM_EnableCounter(LPTMR_T *lptmr)
{
    lptmr->PWMCTL |= LPTMR_PWMCTL_CNTEN_Msk;
}

/**
  * @brief      Disable LPTPWM Generator
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details This function is used to disable LPTPWM counter immediately by clear CNTEN (LPTIMERx_PWMCTL[0]) bit.
  */
void LPTPWM_DisableCounter(LPTMR_T *lptmr)
{
    lptmr->PWMCTL &= ~LPTMR_PWMCTL_CNTEN_Msk;
}

/**
  * @brief      Enable LPTPWM Trigger LPIPs/LPPDMA
  *
  * @param[in]  lptmr           The pointer of the specified LPTmr module. It could be LPTMR0, LPTMR1.
  * @param[in]  u32TargetMask   The mask of modules (LPADC, LPI2C, LPSPI, LPUART, CCAP and LPPDMA) trigger by LPTPWM, the combination of:
  *                                 - \ref LPTPWM_TRGEN
  *                                 - \ref LPTPWM_TRG_TO_LPPDMA
  * @param[in]  u32Condition    The condition to trigger (LPADC, LPI2C, LPSPI, LPUART, CCAP and LPPDMA). It could be one of following conditions:
  *                                 - \ref LPTPWM_TRIGGER_AT_PERIOD_POINT
  *                                 - \ref LPTPWM_TRIGGER_AT_COMPARE_POINT
  *                                 - \ref LPTPWM_TRIGGER_AT_PERIOD_OR_COMPARE_POINT
  * @return     None
  *
  * @details    This function is used to enable specified counter event to trigger (LPADC, LPI2C, LPSPI, LPUART, CCAP and LPPDMA).
  */
void LPTPWM_EnableTrigger(LPTMR_T *lptmr, uint32_t u32TargetMask, uint32_t u32Condition)
{
    lptmr->PWMTRGCTL &= ~(LPTMR_PWMTRGCTL_PWMTRGLPPDMA_Msk | LPTMR_PWMTRGCTL_TRGEN_Msk | LPTMR_PWMTRGCTL_TRGSEL_Msk);
    lptmr->PWMTRGCTL |= (u32TargetMask) | (u32Condition);
}

/**
  * @brief      Disable Trigger LPIPs/LPPDMA
  *
  * @param[in]  lptmr       The pointer of the specified LPTmr module. It could be LPTMR0, LPTMR1.
  * @param[in]  u32TargetMask   The mask of modules (LPADC, LPI2C, LPSPI, LPUART, CCAP and LPPDMA) trigger by LPTPWM, the combination of:
  *                                 - \ref LPTPWM_TRGEN
  *                                 - \ref LPTPWM_TRG_TO_LPPDMA
  *
  * @return     None
  *
  * @details    This function is used to disable counter event to trigger (LPADC, LPI2C, LPSPI, LPUART, CCAP and LPPDMA).
  */
void LPTPWM_DisableTrigger(LPTMR_T *lptmr, uint32_t u32TargetMask)
{
    lptmr->PWMTRGCTL &= ~(u32TargetMask);
}

/** @} end of group LPTPWM_EXPORTED_FUNCTIONS */
/** @} end of group LPTPWM_Driver */
/** @} end of group Standard_Driver */
