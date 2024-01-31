/**************************************************************************//**
 * @file     lptmr.c
 * @version  V1.00
 * @brief    LPTMR Controller (Low Power Timer) driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LPTMR_Driver LPTMR Driver
  @{
*/

/** @addtogroup LPTMR_EXPORTED_FUNCTIONS LPTMR Exported Functions
  @{
*/

/**
  * @brief      Open LPTMR with Operate Mode and Frequency
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  * @param[in]  u32Mode     Operation mode. Possible options are
  *                         - \ref LPTMR_ONESHOT_MODE
  *                         - \ref LPTMR_PERIODIC_MODE
  *                         - \ref LPTMR_TOGGLE_MODE
  *                         - \ref LPTMR_CONTINUOUS_MODE
  * @param[in]  u32Freq     Target working frequency
  *
  * @return     Real lptmr working frequency
  *
  * @details    This API is used to configure lptmr to operate in specified mode and frequency.
  *             If lptmr cannot work in target frequency, a closest frequency will be chose and returned.
  * @note       After calling this API, LPTMR is \b NOT running yet. But could start lptmr running be calling
  *             \ref LPTMR_Start macro or program registers directly.
  */
uint32_t LPTMR_Open(LPTMR_T *lptmr, uint32_t u32Mode, uint32_t u32Freq)
{
    uint32_t u32Clk = LPTMR_GetModuleClock(lptmr);
    uint32_t u32Cmpr = 0UL, u32Prescale = 0UL;

    if (u32Freq == 0)
        return 0 ;

    /* Fastest possible lptmr working freq is (u32Clk / 2). While cmpr = 2, prescaler = 0. */
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

    lptmr->CTL = (u32Mode | u32Prescale);
    lptmr->CMP = u32Cmpr;

    return (u32Clk / (u32Cmpr * (u32Prescale + 1UL)));
}


/**
  * @brief      Stop LPTMR Counting
  *
  * @param[in]  lptmr   The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This API stops lptmr counting and disable all lptmr interrupt function.
  */
void LPTMR_Close(LPTMR_T *lptmr)
{
    lptmr->CTL = 0UL;
    lptmr->EXTCTL = 0UL;
}


/**
  * @brief      Create a specify Delay Time
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  * @param[in]  u32Usec     Delay period in micro seconds. Valid values are between 100~1000000 (100 micro second ~ 1 second).
  *
  * @retval     LPTMR_OK          Delay success, target delay time reached
  * @retval     LPTMR_ERR_TIMEOUT Delay function execute failed due to timer stop working
  *
  * @details    This API is used to create a delay loop for u32usec micro seconds by using lptmr one-shot mode.
  * @note       This API overwrites the register setting of the lptmr used to count the delay time.
  * @note       This API use polling mode. So there is no need to enable interrupt for the lptmr module used to generate delay.
  */
int32_t LPTMR_Delay(LPTMR_T *lptmr, uint32_t u32Usec)
{
    uint32_t u32Clk = LPTMR_GetModuleClock(lptmr);
    uint32_t u32Prescale = 0UL, delay = (SystemCoreClock / u32Clk) + 1UL;
    uint32_t u32Cmpr, u32Cntr, u32NsecPerTick, i = 0UL;

    /* Clear current lptmr configuration */
    lptmr->CTL = 0UL;
    lptmr->EXTCTL = 0UL;

    if (u32Clk <= 1000000UL)  /* min delay is 1000 us if lptmr clock source is <= 1 MHz */
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

    lptmr->CMP = u32Cmpr;
    lptmr->CTL = LPTMR_CTL_CNTEN_Msk | LPTMR_ONESHOT_MODE | u32Prescale;

    /* When system clock is faster than lptmr clock, it is possible lptmr active bit cannot set in time while we check it.
       And the while loop below return immediately, so put a tiny delay here allowing lptmr start counting and raise active flag. */
    for (; delay > 0UL; delay--)
    {
        __NOP();
    }

    /* Add a bail out counter here in case timer clock source is disabled accidentally.
       Prescale counter reset every ECLK * (prescale value + 1).
       The u32Delay here is to make sure timer counter value changed when prescale counter reset */
    delay = (SystemCoreClock / LPTMR_GetModuleClock(lptmr)) * (u32Prescale + 1);
    u32Cntr = lptmr->CNT;

    while (lptmr->CTL & LPTMR_CTL_ACTSTS_Msk)
    {
        /* Bailed out if lptmr stop counting e.g. Some interrupt handler close lptmr clock source. */
        if (u32Cntr == lptmr->CNT)
        {
            if (i++ > delay)
            {
                return LPTMR_ERR_TIMEOUT;
            }
        }
        else
        {
            i = 0;
            u32Cntr = lptmr->CNT;
        }
    }

    return LPTMR_OK;
}

/**
  * @brief      Enable LPTMR Capture Function
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  * @param[in]  u32CapMode  LPTMR capture mode. Could be
  *                         - \ref LPTMR_CAPTURE_FREE_COUNTING_MODE
  *                         - \ref LPTMR_CAPTURE_COUNTER_RESET_MODE
  * @param[in]  u32Edge     LPTMR capture trigger edge. Possible values are
  *                         - \ref LPTMR_CAPTURE_EVENT_FALLING
  *                         - \ref LPTMR_CAPTURE_EVENT_RISING
  *                         - \ref LPTMR_CAPTURE_EVENT_FALLING_RISING
  *                         - \ref LPTMR_CAPTURE_EVENT_RISING_FALLING
  *                         - \ref LPTMR_CAPTURE_EVENT_GET_LOW_PERIOD
  *                         - \ref LPTMR_CAPTURE_EVENT_GET_HIGH_PERIOD
  *
  * @return     None
  *
  * @details    This API is used to enable lptmr capture function with specify capture trigger edge \n
  *             to get current counter value or reset counter value to 0.
  * @note       LPTMR frequency should be configured separately by using \ref LPTMR_Open API, or program registers directly.
  */
void LPTMR_EnableCapture(LPTMR_T *lptmr, uint32_t u32CapMode, uint32_t u32Edge)
{
    lptmr->EXTCTL = (lptmr->EXTCTL & ~(LPTMR_EXTCTL_CAPFUNCS_Msk | LPTMR_EXTCTL_CAPEDGE_Msk)) |
                    u32CapMode | u32Edge | LPTMR_EXTCTL_CAPEN_Msk;
}

/**
  * @brief      Select LPtmr Capture Source
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  * @param[in]  u32Src      LPtmr capture source. Possible values are
  *                         - \ref LPTMR_CAPTURE_FROM_EXTERNAL
  *                         - \ref LPTMR_CAPTURE_FROM_ACMP0
  *                         - \ref LPTMR_CAPTURE_FROM_ACMP1
  *                         - \ref LPTMR_CAPTURE_FROM_ACMP2
  *                         - \ref LPTMR_CAPTURE_FROM_ACMP3
  * @return     None
  *
  * @details    This API is used to select lptmr capture source from Tx_EXT or internal singal.
  */
void LPTMR_CaptureSelect(LPTMR_T *lptmr, uint32_t u32Src)
{
    if (u32Src == LPTMR_CAPTURE_FROM_EXTERNAL)
    {
        lptmr->CTL = (lptmr->CTL & ~(LPTMR_CTL_CAPSRC_Msk)) |
                     (LPTMR_CAPSRC_TMX_EXT);
    }
    else
    {
        lptmr->CTL = (lptmr->CTL & ~(LPTMR_CTL_CAPSRC_Msk)) |
                     (LPTMR_CAPSRC_INTERNAL);
        lptmr->EXTCTL = (lptmr->EXTCTL & ~(LPTMR_EXTCTL_INTERCAPSEL_Msk)) |
                        (u32Src);
    }
}

/**
  * @brief      Disable LPTMR Capture Function
  *
  * @param[in]  lptmr   The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This API is used to disable the lptmr capture function.
  */
void LPTMR_DisableCapture(LPTMR_T *lptmr)
{
    lptmr->EXTCTL &= ~LPTMR_EXTCTL_CAPEN_Msk;
}


/**
  * @brief      Enable LPTMR Counter Function
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  * @param[in]  u32Edge     Detection edge of counter pin. Could be ether
  *                         - \ref LPTMR_COUNTER_EVENT_FALLING, or
  *                         - \ref LPTMR_COUNTER_EVENT_RISING
  *
  * @return     None
  *
  * @details    This function is used to enable the lptmr counter function with specify detection edge.
  * @note       LPTMR compare value should be configured separately by using \ref LPTMR_SET_CMP_VALUE macro or program registers directly.
  * @note       While using event counter function, \ref LPTMR_TOGGLE_MODE cannot set as lptmr operation mode.
  */
void LPTMR_EnableEventCounter(LPTMR_T *lptmr, uint32_t u32Edge)
{
    lptmr->EXTCTL = (lptmr->EXTCTL & ~LPTMR_EXTCTL_CNTPHASE_Msk) | u32Edge;
    lptmr->CTL |= LPTMR_CTL_EXTCNTEN_Msk;
}


/**
  * @brief      Disable LPTMR Counter Function
  *
  * @param[in]  lptmr   The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This API is used to disable the lptmr event counter function.
  */
void LPTMR_DisableEventCounter(LPTMR_T *lptmr)
{
    lptmr->CTL &= ~LPTMR_CTL_EXTCNTEN_Msk;
}


/**
  * @brief      Get LPTMR Clock Frequency
  *
  * @param[in]  lptmr   The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     LPTMR clock frequency
  *
  * @details    This API is used to get the lptmr clock frequency.
  * @note       This API cannot return correct clock rate if lptmr source is from external clock input.
  */
uint32_t LPTMR_GetModuleClock(LPTMR_T *lptmr)
{
    uint32_t u32Src = 0UL, u32Clk = 0UL;
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
        return u32Clk;

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
  * @brief This function is used to select the interrupt source used to trigger other modules.
  * @param[in] lptmr The base address of LPTMR module
  * @param[in] u32Src Selects the interrupt source to trigger other modules. Could be:
  *              - \ref LPTMR_TRGSRC_TIMEOUT_EVENT
  *              - \ref LPTMR_TRGSRC_CAPTURE_EVENT
  * @return None
  */
void LPTMR_SetTriggerSource(LPTMR_T *lptmr, uint32_t u32Src)
{
    lptmr->TRGCTL = (lptmr->TRGCTL & ~LPTMR_TRGCTL_TRGSSEL_Msk) | u32Src;
}

/**
  * @brief This function is used to set modules trigger by lptmr interrupt
  * @param[in] lptmr The base address of LPTMR module
  * @param[in] u32Mask The mask of modules (LPADC, LPI2C, LPSPI, LPUART, CCAP and LPPDMA) trigger by lptmr. Is the combination of
  *             - \ref LPTMR_TRGEN
  *             - \ref LPTMR_TRG_TO_LPPDMA
  * @return None
  */
void LPTMR_SetTriggerTarget(LPTMR_T *lptmr, uint32_t u32Mask)
{
    lptmr->TRGCTL = (lptmr->TRGCTL & ~(LPTMR_TRGCTL_TRGEN_Msk | LPTMR_TRGCTL_TRGLPPDMA_Msk)) | u32Mask;
}

/**
  * @brief      Reset LPTMR Counter
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @retval     LPTMR_OK          LPtmr reset success
  * @retval     LPTMR_ERR_TIMEOUT LPtmr reset failed
  *
  * @details    This function is used to reset current counter value and internal prescale counter value.
  */
int32_t LPTMR_ResetCounter(LPTMR_T *lptmr)
{
    uint32_t u32Delay;

    lptmr->CNT = 0UL;
    /* Takes 2~3 ECLKs to reset lptmr counter */
    u32Delay = (SystemCoreClock / LPTMR_GetModuleClock(lptmr)) * 3;

    while (((lptmr->CNT & LPTMR_CNT_RSTACT_Msk) == LPTMR_CNT_RSTACT_Msk) && (--u32Delay))
    {
        __NOP();
    }

    return ((u32Delay > 0) ? LPTMR_OK : LPTMR_ERR_TIMEOUT);
}

/**
  * @brief      Enable Capture Input Noise Filter Function
  *
  * @param[in]  lptmr           The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @param[in]  u32FilterCount  Noise filter counter. Valid values are between 0~7.
  *
  * @param[in]  u32ClkSrcSel    Noise filter counter clock source, could be one of following source
  *                                 - \ref LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_1
  *                                 - \ref LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_2
  *                                 - \ref LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_4
  *                                 - \ref LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_8
  *                                 - \ref LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_16
  *                                 - \ref LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_32
  *                                 - \ref LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_64
  *                                 - \ref LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_128
  *
  * @return     None
  *
  * @details    This function is used to enable capture input noise filter function.
  */
void LPTMR_EnableCaptureInputNoiseFilter(LPTMR_T *lptmr, uint32_t u32FilterCount, uint32_t u32ClkSrcSel)
{
    lptmr->CAPNF = (((lptmr)->CAPNF & ~(LPTMR_CAPNF_CAPNFCNT_Msk | LPTMR_CAPNF_CAPNFSEL_Msk))
                    | (LPTMR_CAPNF_CAPNFEN_Msk | (u32FilterCount << LPTMR_CAPNF_CAPNFCNT_Pos) | (u32ClkSrcSel << LPTMR_CAPNF_CAPNFSEL_Pos)));
}

/**
  * @brief      Disable Capture Input Noise Filter Function
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to disable capture input noise filter function.
  */
void LPTMR_DisableCaptureInputNoiseFilter(LPTMR_T *lptmr)
{
    lptmr->CAPNF &= ~LPTMR_CAPNF_CAPNFEN_Msk;
}

/** @} end of group LPTMR_EXPORTED_FUNCTIONS */
/** @} end of group LPTMR_Driver */
/** @} end of group Standard_Driver */
