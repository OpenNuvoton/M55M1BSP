/**************************************************************************//**
 * @file     wdt.h
 * @version  V1.00
 * @brief    WDT driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __WDT_H__
#define __WDT_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup WDT_Driver WDT Driver
  @{
*/

/** @addtogroup WDT_EXPORTED_CONSTANTS WDT Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  WDT Time-out Interval Period Constant Definitions                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define WDT_TIMEOUT_2POW4           (0UL << WDT_CTL_TOUTSEL_Pos) /*!< Setting WDT time-out interval to 2^4 * WDT clocks \hideinitializer */
#define WDT_TIMEOUT_2POW6           (1UL << WDT_CTL_TOUTSEL_Pos) /*!< Setting WDT time-out interval to 2^6 * WDT clocks \hideinitializer */
#define WDT_TIMEOUT_2POW8           (2UL << WDT_CTL_TOUTSEL_Pos) /*!< Setting WDT time-out interval to 2^8 * WDT clocks \hideinitializer */
#define WDT_TIMEOUT_2POW10          (3UL << WDT_CTL_TOUTSEL_Pos) /*!< Setting WDT time-out interval to 2^10 * WDT clocks \hideinitializer */
#define WDT_TIMEOUT_2POW12          (4UL << WDT_CTL_TOUTSEL_Pos) /*!< Setting WDT time-out interval to 2^12 * WDT clocks \hideinitializer */
#define WDT_TIMEOUT_2POW14          (5UL << WDT_CTL_TOUTSEL_Pos) /*!< Setting WDT time-out interval to 2^14 * WDT clocks \hideinitializer */
#define WDT_TIMEOUT_2POW16          (6UL << WDT_CTL_TOUTSEL_Pos) /*!< Setting WDT time-out interval to 2^16 * WDT clocks \hideinitializer */
#define WDT_TIMEOUT_2POW18          (7UL << WDT_CTL_TOUTSEL_Pos) /*!< Setting WDT time-out interval to 2^18 * WDT clocks \hideinitializer */
#define WDT_TIMEOUT_2POW20          (8UL << WDT_CTL_TOUTSEL_Pos) /*!< Setting WDT time-out interval to 2^20 * WDT clocks \hideinitializer */
/*---------------------------------------------------------------------------------------------------------*/
/*  WDT  Reset Delay Period Constant Definitions                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define WDT_RESET_DELAY_1026CLK     (0UL << WDT_ALTCTL_RSTDSEL_Pos) /*!< Setting WDT reset delay period to 1026 * WDT clocks \hideinitializer */
#define WDT_RESET_DELAY_130CLK      (1UL << WDT_ALTCTL_RSTDSEL_Pos) /*!< Setting WDT reset delay period to 130 * WDT clocks \hideinitializer */
#define WDT_RESET_DELAY_18CLK       (2UL << WDT_ALTCTL_RSTDSEL_Pos) /*!< Setting WDT reset delay period to 18 * WDT clocks \hideinitializer */
#define WDT_RESET_DELAY_3CLK        (3UL << WDT_ALTCTL_RSTDSEL_Pos) /*!< Setting WDT reset delay period to 3 * WDT clocks \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  WDT Free Reset Counter Keyword Constant Definitions                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define WDT_RESET_COUNTER_KEYWORD   (0x00005AA5UL)    /*!< Fill this value to WDT_RSTCNT register to free reset WDT counter \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* WDT Define Error Code                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define WDT_TIMEOUT         SystemCoreClock     /*!< WDT time-out counter (1 second time-out) \hideinitializer */
#define WDT_OK              ( 0L)               /*!< WDT operation OK \hideinitializer */
#define WDT_FAIL            (-1L)               /*!< WDT operation failed \hideinitializer */
#define WDT_ERR_TIMEOUT     (-2L)               /*!< WDT operation abort due to timeout error \hideinitializer */

/** @} end of group WDT_EXPORTED_CONSTANTS */


/** @addtogroup WDT_EXPORTED_FUNCTIONS WDT Exported Functions
  @{
*/

/**
  * @brief      Clear WDT Reset System Flag
  *
  * @param      wdt is the base address of WDT module.
  *
  *
  * @details    This macro clears WDT time-out reset system flag.
  * \hideinitializer
  */
#define WDT_CLEAR_RESET_FLAG(wdt)          (wdt->STATUS = (wdt->STATUS & ~(WDT_STATUS_IF_Msk | WDT_STATUS_WKF_Msk)) | WDT_STATUS_RSTF_Msk)

/**
  * @brief      Clear WDT Time-out Interrupt Flag
  *
  * @param      wdt is the base address of WDT module.
  *
  *
  * @details    This macro clears WDT time-out interrupt flag.
  * \hideinitializer
  */
#define WDT_CLEAR_TIMEOUT_INT_FLAG(wdt)    (wdt->STATUS = (wdt->STATUS & ~(WDT_STATUS_RSTF_Msk | WDT_STATUS_WKF_Msk)) | WDT_STATUS_IF_Msk)

/**
  * @brief      Clear WDT Wake-up Flag
  *
  * @param      wdt is the base address of WDT module.
  *
  *
  * @details    This macro clears WDT time-out wake-up system flag.
  * \hideinitializer
  */
#define WDT_CLEAR_TIMEOUT_WAKEUP_FLAG(wdt) (wdt->STATUS= (wdt->STATUS & ~(WDT_STATUS_RSTF_Msk | WDT_STATUS_IF_Msk)) | WDT_STATUS_WKF_Msk)

/**
  * @brief      Get WDT Time-out Reset Flag
  *
  * @param      wdt is the base address of WDT module.
  *
  * @retval     0   WDT time-out reset system did not occur
  * @retval     1   WDT time-out reset system occurred
  *
  * @details    This macro indicates system has been reset by WDT time-out reset or not.
  * \hideinitializer
  */
#define WDT_GET_RESET_FLAG(wdt)            ((wdt->STATUS & WDT_STATUS_RSTF_Msk)? 1UL : 0UL)

/**
  * @brief      Get WDT Time-out Interrupt Flag
  *
  * @param      wdt is the base address of WDT module.
  *
  * @retval     0   WDT time-out interrupt did not occur
  * @retval     1   WDT time-out interrupt occurred
  *
  * @details    This macro indicates WDT time-out interrupt occurred or not.
  * \hideinitializer
  */
#define WDT_GET_TIMEOUT_INT_FLAG(wdt)      ((wdt->STATUS & WDT_STATUS_IF_Msk)? 1UL : 0UL)

/**
  * @brief      Get WDT Time-out Wake-up Flag
  *
  * @param      wdt is the base address of WDT module.
  *
  * @retval     0   WDT time-out interrupt does not cause CPU wake-up
  * @retval     1   WDT time-out interrupt event cause CPU wake-up
  *
  * @details    This macro indicates WDT time-out interrupt event has waked up system or not.
  * \hideinitializer
  */
#define WDT_GET_TIMEOUT_WAKEUP_FLAG(wdt)   ((wdt->STATUS & WDT_STATUS_WKF_Msk)? 1UL : 0UL)

/**
  * @brief      Reset WDT Counter
  *
  * @param      wdt is the base address of WDT module.
  *
  *
  * @details    This macro is used to reset the internal 18-bit WDT up counter value.
  * @note       If WDT is activated and time-out reset system function is enabled also, user should \n
  *             reset the 18-bit WDT up counter value to avoid generate WDT time-out reset signal to \n
  *             reset system before the WDT time-out reset delay period expires.
  * \hideinitializer
  */
#define WDT_RESET_COUNTER(wdt)             (wdt->RSTCNT = WDT_RESET_COUNTER_KEYWORD)

/* Declare these inline functions here to avoid MISRA C 2004 rule 8.1 error */
__STATIC_INLINE void WDT_Close(WDT_T *wdt);
__STATIC_INLINE void WDT_EnableInt(WDT_T *wdt);
__STATIC_INLINE void WDT_DisableInt(WDT_T *wdt);

/**
  * @brief      Stop WDT Counting
  *
  * @param      wdt is the base address of WDT module.
  *
  *
  * @details    This function will stop WDT counting and disable WDT module.
  */
__STATIC_INLINE void WDT_Close(WDT_T *wdt)
{
    uint32_t u32TimeOutCnt = WDT_TIMEOUT;

    wdt->CTL = 0UL;

    while (wdt->CTL & WDT_CTL_SYNC_Msk) /* Wait disable WDTEN bit completed, it needs 4 * WDT_CLK. */
    {
        if (--u32TimeOutCnt == 0) break;
    }

}

/**
  * @brief      Enable WDT Time-out Interrupt
  *
  * @param      wdt is the base address of WDT module.
  *
  *
  * @details    This function will enable the WDT time-out interrupt function.
  */
__STATIC_INLINE void WDT_EnableInt(WDT_T *wdt)
{
    wdt->CTL |= WDT_CTL_INTEN_Msk;
}

/**
  * @brief      Disable WDT Time-out Interrupt
  *
  * @param      wdt is the base address of WDT module.
  *
  *
  * @details    This function will disable the WDT time-out interrupt function.
  */
__STATIC_INLINE void WDT_DisableInt(WDT_T *wdt)
{
    /* Do not touch another write 1 clear bits */
    wdt->CTL &= ~(WDT_CTL_INTEN_Msk);
}

int32_t WDT_Open(WDT_T *wdt, uint32_t u32TimeoutInterval, uint32_t u32ResetDelay, uint32_t u32EnableReset, uint32_t u32EnableWakeup);

/** @} end of group WDT_EXPORTED_FUNCTIONS */
/** @} end of group WDT_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __WDT_H__ */
