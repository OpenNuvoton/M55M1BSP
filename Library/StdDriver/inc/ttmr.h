/**************************************************************************//**
 * @file     ttmr.h
 * @version  V1.00
 * @brief    TTMR (Tick Timer Controller) driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __TTMR_H__
#define __TTMR_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup TTMR_Driver TTMR Driver
  @{
*/

/** @addtogroup TTMR_EXPORTED_CONSTANTS TTMR Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  TTMR Operation Mode, External Counter and Capture Mode Constant Definitions                           */
/*---------------------------------------------------------------------------------------------------------*/
#define TTMR_ONESHOT_MODE                 (0UL << TTMR_CTL_OPMODE_Pos)      /*!< TTMR working in one-shot mode \hideinitializer */
#define TTMR_PERIODIC_MODE                (1UL << TTMR_CTL_OPMODE_Pos)      /*!< TTMR working in periodic mode \hideinitializer */
#define TTMR_CONTINUOUS_MODE              (3UL << TTMR_CTL_OPMODE_Pos)      /*!< TTMR working in continuous counting mode \hideinitializer */

#define TTMR_TRGEN                        (TTMR_TRGCTL_TRGEN_Msk)           /*!< Each TTMR event to trigger Low Power IP \hideinitializer */
#define TTMR_TRG_TO_LPPDMA                (TTMR_TRGCTL_TRGLPPDMA_Msk)       /*!< Each TTMR event to trigger LPPDMA transfer \hideinitializer */

#define TTMR_CMP_MAX_VALUE                (0xFFFFFFUL)                      /*!< Max TTMR compare value \hideinitializer */

#define TTMR_OK                           ( 0L)    /*!< TTMR operation OK \hideinitializer */
#define TTMR_ERR_FAIL                     (-1L)    /*!< TTMR operation failed \hideinitializer */
#define TTMR_ERR_TIMEOUT                  (-2L)    /*!< TTMR operation abort due to timeout error \hideinitializer */

/** @} end of group TTMR_EXPORTED_CONSTANTS */


/** @addtogroup TTMR_EXPORTED_FUNCTIONS TTMR Exported Functions
  @{
*/

/**
  * @brief      Set Timer Compared Value
  *
  * @param[in]  ttmr       The pointer of the specified Timer module. It could be TTMR0, TTMR1.
  * @param[in]  u32Value    Timer compare value. Valid values are between 2 to 0xFFFFFF.
  *
  * @return     None
  *
  * @details    This macro is used to set ttmr compared value to adjust ttmr time-out interval.
  * @note       1. Never write 0x0 or 0x1 in this field, or the core will run into unknown state. \n
  *             2. If update ttmr compared value in continuous counting mode, ttmr counter value will keep counting continuously. \n
  *                But if ttmr is operating at other modes, the ttmr up counter will restart counting and start from 0.
  * \hideinitializer
  */
#define TTMR_SET_CMP_VALUE(ttmr, u32Value)        ((ttmr)->CMP = (u32Value))

/**
  * @brief      Set Timer Prescale Value
  *
  * @param[in]  ttmr       The pointer of the specified Timer module. It could be TTMR0, TTMR1.
  * @param[in]  u32Value    Timer prescale value. Valid values are between 0 to 0xFF.
  *
  * @return     None
  *
  * @details    This macro is used to set ttmr prescale value and ttmr source clock will be divided by (prescale + 1) \n
  *             before it is fed into ttmr.
  * \hideinitializer
  */
#define TTMR_SET_PRESCALE_VALUE(ttmr, u32Value)   ((ttmr)->CTL = ((ttmr)->CTL & ~TTMR_CTL_PSC_Msk) | (u32Value))

/**
  * @brief      Check specify Timer Status
  *
  * @param[in]  ttmr       The pointer of the specified Timer module. It could be TTMR0, TTMR1.
  *
  * @retval     0   Timer 24-bit up counter is inactive
  * @retval     1   Timer 24-bit up counter is active
  *
  * @details    This macro is used to check if specify Timer counter is inactive or active.
  * \hideinitializer
  */
#define TTMR_IS_ACTIVE(ttmr)                      (((ttmr)->CTL & TTMR_CTL_ACTSTS_Msk)? 1 : 0)


/**
  * @brief      Select Timer operating mode
  *
  * @param[in]  ttmr       The pointer of the specified Timer module. It could be TTMR0, TTMR1.
  * @param[in]  u32OpMode   Operation mode. Possible options are
  *                         - \ref TTMR_ONESHOT_MODE
  *                         - \ref TTMR_PERIODIC_MODE
  *                         - \ref TTMR_CONTINUOUS_MODE
  *
  * @return     None
  * \hideinitializer
  */
#define TTMR_SET_OPMODE(ttmr, u32OpMode)   ((ttmr)->CTL = ((ttmr)->CTL & ~TTMR_CTL_OPMODE_Msk) | (u32OpMode))

/* Declare these inline functions here to avoid MISRA C 2004 rule 8.1 error */
__STATIC_INLINE void TTMR_Start(TTMR_T *ttmr);
__STATIC_INLINE void TTMR_Stop(TTMR_T *ttmr);
__STATIC_INLINE void TTMR_EnableWakeup(TTMR_T *ttmr);
__STATIC_INLINE void TTMR_DisableWakeup(TTMR_T *ttmr);
__STATIC_INLINE void TTMR_EnableInt(TTMR_T *ttmr);
__STATIC_INLINE void TTMR_DisableInt(TTMR_T *ttmr);
__STATIC_INLINE uint32_t TTMR_GetIntFlag(TTMR_T *ttmr);
__STATIC_INLINE void TTMR_ClearIntFlag(TTMR_T *ttmr);
__STATIC_INLINE uint32_t TTMR_GetWakeupFlag(TTMR_T *ttmr);
__STATIC_INLINE void TTMR_ClearWakeupFlag(TTMR_T *ttmr);
__STATIC_INLINE uint32_t TTMR_GetCounter(TTMR_T *ttmr);
__STATIC_INLINE void TTMR_EnablePDCLK(TTMR_T *ttmr);
__STATIC_INLINE void TTMR_DisablePDCLK(TTMR_T *ttmr);

/**
  * @brief      Start Timer Counting
  *
  * @param[in]  ttmr       The pointer of the specified Timer module. It could be TTMR0, TTMR1.
  *
  * @return     None
  *
  * @details    This function is used to start Timer counting.
  */
__STATIC_INLINE void TTMR_Start(TTMR_T *ttmr)
{
    ttmr->CTL |= TTMR_CTL_CNTEN_Msk;
}

/**
  * @brief      Stop Timer Counting
  *
  * @param[in]  ttmr       The pointer of the specified Timer module. It could be TTMR0, TTMR1.
  *
  * @return     None
  *
  * @details    This function is used to stop/suspend Timer counting.
  */
__STATIC_INLINE void TTMR_Stop(TTMR_T *ttmr)
{
    ttmr->CTL &= ~TTMR_CTL_CNTEN_Msk;
}

/**
  * @brief      Enable Timer Interrupt Wake-up Function
  *
  * @param[in]  ttmr       The pointer of the specified Timer module. It could be TTMR0, TTMR1.
  *
  * @return     None
  *
  * @details    This function is used to enable the ttmr interrupt wake-up function and interrupt source could be time-out interrupt, \n
  *             counter event interrupt or capture trigger interrupt.
  * @note       To wake the system from Power-down mode, ttmr clock source must be ether LXT or LIRC.
  */
__STATIC_INLINE void TTMR_EnableWakeup(TTMR_T *ttmr)
{
    ttmr->CTL |= (TTMR_CTL_WKEN_Msk | TTMR_CTL_PDCLKEN_Msk);
}

/**
  * @brief      Disable Timer Wake-up Function
  *
  * @param[in]  ttmr       The pointer of the specified Timer module. It could be TTMR0, TTMR1.
  *
  * @return     None
  *
  * @details    This function is used to disable the ttmr interrupt wake-up function.
  */
__STATIC_INLINE void TTMR_DisableWakeup(TTMR_T *ttmr)
{
    ttmr->CTL &= ~TTMR_CTL_WKEN_Msk;
}


/**
  * @brief      Enable Timer Time-out Interrupt
  *
  * @param[in]  ttmr       The pointer of the specified Timer module. It could be TTMR0, TTMR1.
  *
  * @return     None
  *
  * @details    This function is used to enable the ttmr time-out interrupt function.
  */
__STATIC_INLINE void TTMR_EnableInt(TTMR_T *ttmr)
{
    ttmr->CTL |= TTMR_CTL_INTEN_Msk;
}

/**
  * @brief      Disable Timer Time-out Interrupt
  *
  * @param[in]  ttmr       The pointer of the specified Timer module. It could be TTMR0, TTMR1.
  *
  * @return     None
  *
  * @details    This function is used to disable the ttmr time-out interrupt function.
  */
__STATIC_INLINE void TTMR_DisableInt(TTMR_T *ttmr)
{
    ttmr->CTL &= ~TTMR_CTL_INTEN_Msk;
}


/**
  * @brief      Get Timer Time-out Interrupt Flag
  *
  * @param[in]  ttmr   The pointer of the specified Timer module. It could be TTMR0, TTMR1.
  *
  * @retval     0   Timer time-out interrupt did not occur
  * @retval     1   Timer time-out interrupt occurred
  *
  * @details    This function indicates ttmr time-out interrupt occurred or not.
  */
__STATIC_INLINE uint32_t TTMR_GetIntFlag(TTMR_T *ttmr)
{
    return ((ttmr->INTSTS & TTMR_INTSTS_TIF_Msk) ? 1UL : 0UL);
}

/**
  * @brief      Clear Timer Time-out Interrupt Flag
  *
  * @param[in]  ttmr       The pointer of the specified Timer module. It could be TTMR0, TTMR1.
  *
  * @return     None
  *
  * @details    This function clears ttmr time-out interrupt flag to 0.
  */
__STATIC_INLINE void TTMR_ClearIntFlag(TTMR_T *ttmr)
{
    ttmr->INTSTS = TTMR_INTSTS_TIF_Msk;
}


/**
  * @brief      Get Timer Wake-up Flag
  *
  * @param[in]  ttmr   The pointer of the specified Timer module. It could be TTMR0, TTMR1.
  *
  * @retval     0   Timer does not cause CPU wake-up
  * @retval     1   Timer interrupt event cause CPU wake-up
  *
  * @details    This function indicates ttmr interrupt event has waked up system or not.
  */
__STATIC_INLINE uint32_t TTMR_GetWakeupFlag(TTMR_T *ttmr)
{
    return (ttmr->INTSTS & TTMR_INTSTS_TWKF_Msk ? 1UL : 0UL);
}

/**
  * @brief      Clear Timer Wake-up Flag
  *
  * @param[in]  ttmr       The pointer of the specified Timer module. It could be TTMR0, TTMR1.
  *
  * @return     None
  *
  * @details    This function clears the ttmr wake-up system flag to 0.
  */
__STATIC_INLINE void TTMR_ClearWakeupFlag(TTMR_T *ttmr)
{
    ttmr->INTSTS = TTMR_INTSTS_TWKF_Msk;
}


/**
  * @brief      Get Counter value
  *
  * @param[in]  ttmr       The pointer of the specified Timer module. It could be TTMR0, TTMR1.
  *
  * @return     24-bit Counter Value
  *
  * @details    This function reports the current 24-bit ttmr counter value.
  */
__STATIC_INLINE uint32_t TTMR_GetCounter(TTMR_T *ttmr)
{
    return ttmr->CNT;
}

/**
  * @brief      Enable Timer Power-down Engine Clock
  *
  * @param[in]  ttmr       The pointer of the specified Timer module. It could be TTMR0, TTMR1.
  *
  * @return     None
  *
  * @details    This function is used to enable the ttmr Power-down Engine Clock.
  */
__STATIC_INLINE void TTMR_EnablePDCLK(TTMR_T *ttmr)
{
    ttmr->CTL |= TTMR_CTL_PDCLKEN_Msk;
}

/**
  * @brief      Disable Timer Power-down Engine Clock
  *
  * @param[in]  ttmr       The pointer of the specified Timer module. It could be TTMR0, TTMR1.
  *
  * @return     None
  *
  * @details    This function is used to disable the ttmr Power-down Engine Clock.
  */
__STATIC_INLINE void TTMR_DisablePDCLK(TTMR_T *ttmr)
{
    ttmr->CTL &= ~TTMR_CTL_PDCLKEN_Msk;
}

uint32_t TTMR_Open(TTMR_T *ttmr, uint32_t u32Mode, uint32_t u32Freq);
void TTMR_Close(TTMR_T *ttmr);
int32_t TTMR_Delay(TTMR_T *ttmr, uint32_t u32Usec);
uint32_t TTMR_GetModuleClock(TTMR_T *ttmr);
void TTMR_SetTriggerTarget(TTMR_T *ttmr, uint32_t u32Mask);
int32_t TTMR_ResetCounter(TTMR_T *ttmr);

/** @} end of group TTMR_EXPORTED_FUNCTIONS */
/** @} end of group TTMR_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __TTMR_H__ */
