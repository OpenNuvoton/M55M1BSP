/**************************************************************************//**
 * @file     ttmr.c
 * @version  V1.00
 * @brief    TTMR (Tick Timer Controller) driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup TTMR_Driver TTMR Driver
  @{
*/

/** @addtogroup TTMR_EXPORTED_FUNCTIONS TTMR Exported Functions
  @{
*/

/**
  * @brief      Open TTMR with Operate Mode and Frequency
  *
  * @param[in]  ttmr        The pointer of the specified TTMR module. It could be TTMR0, TTMR1.
  * @param[in]  u32Mode     Operation mode. Possible options are
  *                         - \ref TTMR_ONESHOT_MODE
  *                         - \ref TTMR_PERIODIC_MODE
  *                         - \ref TTMR_CONTINUOUS_MODE
  * @param[in]  u32Freq     Target working frequency
  *
  * @return     Real ttmr working frequency
  *
  * @details    This API is used to configure ttmr to operate in specified mode and frequency.
  *             If ttmr cannot work in target frequency, a closest frequency will be chose and returned.
  * @note       After calling this API, Timer is \b NOT running yet. But could start ttmr running be calling
  *             \ref TTMR_Start macro or program registers directly.
  */
uint32_t TTMR_Open(TTMR_T *ttmr, uint32_t u32Mode, uint32_t u32Freq)
{
    uint32_t u32Clk = TTMR_GetModuleClock(ttmr);
    uint32_t u32Cmpr = 0UL, u32Prescale = 0UL;

    if (u32Freq == 0)
        return 0 ;

    /* Fastest possible ttmr working freq is (u32Clk / 2). While cmpr = 2, prescaler = 0. */
    if (u32Freq > (u32Clk / 2UL))
    {
        u32Cmpr = 2UL;
    }
    else
    {
        u32Cmpr = u32Clk / u32Freq;
        u32Prescale = (u32Cmpr >> 24);  /* for 24 bits CMPDAT */

        if (u32Prescale > 0UL)
            u32Cmpr = u32Cmpr / (u32Prescale + 1UL);
    }

    ttmr->CTL = (u32Mode | u32Prescale);
    ttmr->CMP = u32Cmpr;

    return (u32Clk / (u32Cmpr * (u32Prescale + 1UL)));
}


/**
  * @brief      Stop TTMR Counting
  *
  * @param[in]  ttmr        The pointer of the specified TTMR module. It could be TTMR0, TTMR1.
  *
  * @return     None
  *
  * @details    This API stops ttmr counting and disable all ttmr interrupt function.
  */
void TTMR_Close(TTMR_T *ttmr)
{
    ttmr->CTL = 0UL;
}


/**
  * @brief      Create a specify Delay Time
  *
  * @param[in]  ttmr        The pointer of the specified TTMR module. It could be TTMR0, TTMR1.
  * @param[in]  u32Usec     Delay period in micro seconds. Valid values are between 100~1000000 (100 micro second ~ 1 second).
  *
  * @retval     TTMR_OK          Delay success, target delay time reached
  * @retval     TTMR_ERR_TIMEOUT Delay function execute failed due to timer stop working
  *
  * @details    This API is used to create a delay loop for u32usec micro seconds by using ttmr one-shot mode.
  * @note       This API overwrites the register setting of the ttmr used to count the delay time.
  * @note       This API use polling mode. So there is no need to enable interrupt for the ttmr module used to generate delay.
  */
int32_t TTMR_Delay(TTMR_T *ttmr, uint32_t u32Usec)
{
    uint32_t u32Clk = TTMR_GetModuleClock(ttmr);
    uint32_t u32Prescale = 0UL, u32Delay;
    uint32_t u32Cmpr, u32Cntr, u32NsecPerTick, i = 0UL;

    /* Clear current ttmr configuration */
    ttmr->CTL = 0UL;

    if (u32Clk <= 1000000UL)  /* min delay is 1000 us if ttmr clock source is <= 1 MHz */
    {
        if (u32Usec < 1000UL)
        {
            u32Usec = 1000UL;
        }

        if (u32Usec > 1000000UL)
        {
            u32Usec = 1000000UL;
        }
    }
    else
    {
        if (u32Usec < 100UL)
        {
            u32Usec = 100UL;
        }

        if (u32Usec > 1000000UL)
        {
            u32Usec = 1000000UL;
        }
    }

    if (u32Clk <= 1000000UL)
    {
        u32Prescale = 0UL;
        u32NsecPerTick = 1000000000UL / u32Clk;
        u32Cmpr = (u32Usec * 1000UL) / u32NsecPerTick;
    }
    else
    {
        u32Cmpr = u32Usec * (u32Clk / 1000000UL);
        u32Prescale = (u32Cmpr >> 24);  /* for 24 bits CMPDAT */

        if (u32Prescale > 0UL)
            u32Cmpr = u32Cmpr / (u32Prescale + 1UL);
    }

    ttmr->CMP = u32Cmpr;
    ttmr->CTL = TTMR_CTL_CNTEN_Msk | TTMR_ONESHOT_MODE | u32Prescale;

    /* When system clock is faster than TTMR clock, it is possible TTMR active bit cannot set in time while we check it.
       And the while loop below return immediately, so put a tiny delay here allowing TTMR start counting and raise active flag. */
    for (u32Delay = (SystemCoreClock / u32Clk) + 1UL; u32Delay > 0UL; u32Delay--)
    {
        __NOP();
    }

    /* Add a bail out counter here in case timer clock source is disabled accidentally.
       Prescale counter reset every ECLK * (prescale value + 1).
       The u32Delay here is to make sure timer counter value changed when prescale counter reset */
    u32Delay = (SystemCoreClock / TTMR_GetModuleClock(ttmr)) * (u32Prescale + 1);
    u32Cntr = ttmr->CNT;

    while (ttmr->CTL & TTMR_CTL_ACTSTS_Msk)
    {
        /* Bailed out if timer stop counting e.g. Some interrupt handler close timer clock source. */
        if (u32Cntr == ttmr->CNT)
        {
            if (i++ > u32Delay)
            {
                return TTMR_ERR_TIMEOUT;
            }
        }
        else
        {
            i = 0;
            u32Cntr = ttmr->CNT;
        }
    }

    return TTMR_OK;
}

/**
  * @brief      Get TTMR Clock Frequency
  *
  * @param[in]  ttmr   The pointer of the specified TTMR module. It could be TTMR0, TTMR1.
  *
  * @return     TTMR clock frequency
  *
  * @details    This API is used to get the TTMR clock frequency.
  * @note       This API cannot return correct clock rate if TTMR source is from external clock input.
  */
uint32_t TTMR_GetModuleClock(TTMR_T *ttmr)
{
    uint32_t u32Src = 0UL, u32Clk;
    const uint32_t au32Clk[] = {0UL, __LXT, __LIRC, __MIRC, __HIRC};

    if (ttmr == TTMR0)
    {
        u32Src = (CLK->TTMRSEL & CLK_TTMRSEL_TTMR0SEL_Msk) >> CLK_TTMRSEL_TTMR0SEL_Pos;
    }
    else if (ttmr == TTMR1)
    {
        u32Src = (CLK->TTMRSEL & CLK_TTMRSEL_TTMR1SEL_Msk) >> CLK_TTMRSEL_TTMR1SEL_Pos;
    }

    if (u32Src == 0UL)
    {
        u32Clk = CLK_GetPCLK4Freq();
    }
    else if (u32Src == 3UL)
    {
        u32Clk = CLK_GetMIRCFreq();
    }
    else
    {
        u32Clk = au32Clk[u32Src];
    }

    return u32Clk;
}

/**
  * @brief This function is used to set modules trigger by TTMR interrupt
  * @param[in] ttmr    The pointer of the specified TTMR module. It could be TTMR0, TTMR1.
  * @param[in] u32Mask The mask of modules (LPADC, LPI2C, LPSPI, LPUART, CCAP and LPPDMA) trigger by ttmr. Is the combination of
  *             - \ref TTMR_TRGEN
  *             - \ref TTMR_TRG_TO_LPPDMA
  * @return None
  */
void TTMR_SetTriggerTarget(TTMR_T *ttmr, uint32_t u32Mask)
{
    ttmr->TRGCTL = (ttmr->TRGCTL & ~(TTMR_TRGCTL_TRGEN_Msk | TTMR_TRGCTL_TRGLPPDMA_Msk)) | u32Mask;
}

/**
  * @brief      Reset TTMR Counter
  *
  * @param[in]  ttmr       The pointer of the specified TTMR module. It could be TTMR0, TTMR1.
  *
  * @retval     TTMR_OK          TTMR reset success
  * @retval     TTMR_ERR_TIMEOUT TTMR reset failed
  *
  * @details    This function is used to reset current counter value and internal prescale counter value.
  */
int32_t TTMR_ResetCounter(TTMR_T *ttmr)
{
    uint32_t u32Delay;

    ttmr->CNT = 0UL;
    /* Takes 2~3 ECLKs to reset timer counter */
    u32Delay = (SystemCoreClock / TTMR_GetModuleClock(ttmr)) * 3;

    while (((ttmr->CNT & TTMR_CNT_RSTACT_Msk) == TTMR_CNT_RSTACT_Msk) && (--u32Delay))
    {
        __NOP();
    }

    return ((u32Delay > 0) ? TTMR_OK : TTMR_ERR_TIMEOUT);
}

/** @} end of group TTMR_EXPORTED_FUNCTIONS */
/** @} end of group TTMR_Driver */
/** @} end of group Standard_Driver */
