/**************************************************************************//**
 * @file     lptmr.h
 * @version  V1.00
 * @brief    LPTMR Controller (Low Power Timer) driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __LPTMR_H__
#define __LPTMR_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LPTMR_Driver LPTMR Driver
  @{
*/

/** @addtogroup LPTMR_EXPORTED_CONSTANTS LPTMR Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  LPTMR Operation Mode, External Counter and Capture Mode Constant Definitions                           */
/*---------------------------------------------------------------------------------------------------------*/
#define LPTMR_ONESHOT_MODE                      (0UL << LPTMR_CTL_OPMODE_Pos)           /*!< LPTMR working in one-shot mode \hideinitializer */
#define LPTMR_PERIODIC_MODE                     (1UL << LPTMR_CTL_OPMODE_Pos)           /*!< LPTMR working in periodic mode \hideinitializer */
#define LPTMR_TOGGLE_MODE                       (2UL << LPTMR_CTL_OPMODE_Pos)           /*!< LPTMR working in toggle-output mode \hideinitializer */
#define LPTMR_CONTINUOUS_MODE                   (3UL << LPTMR_CTL_OPMODE_Pos)           /*!< LPTMR working in continuous counting mode \hideinitializer */

#define LPTMR_TOUT_PIN_FROM_TMX                 (0UL << LPTMR_CTL_TGLPINSEL_Pos)        /*!< LPTMR toggle-output pin is from TMx pin \hideinitializer */
#define LPTMR_TOUT_PIN_FROM_TMX_EXT             (1UL << LPTMR_CTL_TGLPINSEL_Pos)        /*!< LPTMR toggle-output pin is from TMx_EXT pin \hideinitializer */
#define LPTMR_TOUT_PIN_FROM_TX                  LPTMR_TOUT_PIN_FROM_TMX
#define LPTMR_TOUT_PIN_FROM_TX_EXT              LPTMR_TOUT_PIN_FROM_TMX_EXT

#define LPTMR_COUNTER_EVENT_FALLING             (0UL << LPTMR_EXTCTL_CNTPHASE_Pos)      /*!< Counter increase on falling edge detection \hideinitializer */
#define LPTMR_COUNTER_EVENT_RISING              (1UL << LPTMR_EXTCTL_CNTPHASE_Pos)      /*!< Counter increase on rising edge detection \hideinitializer */

#define LPTMR_CAPTURE_FREE_COUNTING_MODE        (0UL << LPTMR_EXTCTL_CAPFUNCS_Pos)      /*!< LPTMR capture event to get lptmr counter value \hideinitializer */
#define LPTMR_CAPTURE_COUNTER_RESET_MODE        (1UL << LPTMR_EXTCTL_CAPFUNCS_Pos)      /*!< LPTMR capture event to reset lptmr counter \hideinitializer */

#define LPTMR_CAPTURE_EVENT_FALLING             (0UL << LPTMR_EXTCTL_CAPEDGE_Pos)       /*!< Falling edge detection to trigger capture event \hideinitializer */
#define LPTMR_CAPTURE_EVENT_RISING              (1UL << LPTMR_EXTCTL_CAPEDGE_Pos)       /*!< Rising edge detection to trigger capture event \hideinitializer */
#define LPTMR_CAPTURE_EVENT_FALLING_RISING      (2UL << LPTMR_EXTCTL_CAPEDGE_Pos)       /*!< Both falling and rising edge detection to trigger capture event, and first event at falling edge \hideinitializer */
#define LPTMR_CAPTURE_EVENT_RISING_FALLING      (3UL << LPTMR_EXTCTL_CAPEDGE_Pos)       /*!< Both rising and falling edge detection to trigger capture event, and first event at rising edge \hideinitializer */
#define LPTMR_CAPTURE_EVENT_GET_LOW_PERIOD      (6UL << LPTMR_EXTCTL_CAPEDGE_Pos)       /*!< First capture event is at falling edge, follows are at at rising edge \hideinitializer */
#define LPTMR_CAPTURE_EVENT_GET_HIGH_PERIOD     (7UL << LPTMR_EXTCTL_CAPEDGE_Pos)       /*!< First capture event is at rising edge, follows are at at falling edge \hideinitializer */

#define LPTMR_CAPSRC_TMX_EXT                    (0UL << LPTMR_CTL_CAPSRC_Pos)           /*!< Capture source from TMx_EXT pin      \hideinitializer */
#define LPTMR_CAPSRC_INTERNAL                   (1UL << LPTMR_CTL_CAPSRC_Pos)           /*!< Capture source from Internal event such as LIRC or ACMP0/1 \hideinitializer */
#define LPTMR_INTERCAPSEL_ACMP0                 (0UL << LPTMR_EXTCTL_INTERCAPSEL_Pos)   /*!< Capture source from Internal event ACMP0 \hideinitializer */
#define LPTMR_INTERCAPSEL_ACMP1                 (1UL << LPTMR_EXTCTL_INTERCAPSEL_Pos)   /*!< Capture source from Internal event ACMP1 \hideinitializer */
#define LPTMR_INTERCAPSEL_ACMP2                 (7UL << LPTMR_EXTCTL_INTERCAPSEL_Pos)   /*!< Capture source from Internal event ACMP2 \hideinitializer */
#define LPTMR_INTERCAPSEL_ACMP3                 (8UL << LPTMR_EXTCTL_INTERCAPSEL_Pos)   /*!< Capture source from Internal event ACMP3 \hideinitializer */

#define LPTMR_CAPTURE_FROM_ACMP0                (LPTMR_INTERCAPSEL_ACMP0)          /*!< Capture source from Internal event ACMP0 \hideinitializer  */
#define LPTMR_CAPTURE_FROM_ACMP1                (LPTMR_INTERCAPSEL_ACMP1)          /*!< Capture source from Internal event ACMP1 \hideinitializer */
#define LPTMR_CAPTURE_FROM_ACMP2                (LPTMR_INTERCAPSEL_ACMP2)          /*!< Capture source from Internal event ACMP2 \hideinitializer  */
#define LPTMR_CAPTURE_FROM_ACMP3                (LPTMR_INTERCAPSEL_ACMP3)          /*!< Capture source from Internal event ACMP3 \hideinitializer */
#define LPTMR_CAPTURE_FROM_INTERNAL             (LPTMR_CAPTURE_FROM_ACMP0)                /*!< Capture source from Internal event ACMP0 \hideinitializer */
#define LPTMR_CAPTURE_FROM_EXTERNAL             (0xFFUL)                                  /*!< Capture source from Tx_EXT pin  \hideinitializer          */

/* Event Counter Sourdce Selection */
#define LPTMR_EVENT_COUNTER_SOURCE_TMX          (0UL << LPTMR_EXTCTL_ECNTSSEL_Pos) /*!< Event Counter source from  external TMx (x= 0~3) pin \hideinitializer */
#define LPTMR_EVENT_COUNTER_SOURCE_ACMP0        (2UL << LPTMR_EXTCTL_ECNTSSEL_Pos) /*!< Event Counter source from internal ACMP0 output signal \hideinitializer */
#define LPTMR_EVENT_COUNTER_SOURCE_ACMP1        (3UL << LPTMR_EXTCTL_ECNTSSEL_Pos) /*!< Event Counter source from internal ACMP1 output signal \hideinitializer */
#define LPTMR_EVENT_COUNTER_SOURCE_ACMP2        (4UL << LPTMR_EXTCTL_ECNTSSEL_Pos) /*!< Event Counter source from internal ACMP2 output signal \hideinitializer */
#define LPTMR_EVENT_COUNTER_SOURCE_ACMP3        (5UL << LPTMR_EXTCTL_ECNTSSEL_Pos) /*!< Event Counter source from internal ACMP3 output signal \hideinitializer */

#define LPTMR_TRGSRC_TIMEOUT_EVENT              (0UL << LPTMR_TRGCTL_TRGSSEL_Pos)       /*!< Select internal trigger source from lptmr time-out event \hideinitializer */
#define LPTMR_TRGSRC_CAPTURE_EVENT              (1UL << LPTMR_TRGCTL_TRGSSEL_Pos)       /*!< Select internal trigger source from lptmr capture event \hideinitializer */
#define LPTMR_TRGSEL_TIMEOUT_EVENT              LPTMR_TRGSRC_TIMEOUT_EVENT
#define LPTMR_TRGSEL_CAPTURE_EVENT              LPTMR_TRGSRC_CAPTURE_EVENT

#define LPTMR_TRGEN                             (LPTMR_TRGCTL_TRGEN_Msk)                /*!< Each lptmr event to trigger Low Power IP \hideinitializer */
#define LPTMR_TRG_TO_LPPDMA                     (LPTMR_TRGCTL_TRGLPPDMA_Msk)            /*!< Each lptmr event to trigger PDMA transfer \hideinitializer */

#define LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_1   (0UL)    /*!< Capture noise filter clock is PCLK divide by 1 \hideinitializer */
#define LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_2   (1UL)    /*!< Capture noise filter clock is PCLK divide by 2 \hideinitializer */
#define LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_4   (2UL)    /*!< Capture noise filter clock is PCLK divide by 4 \hideinitializer */
#define LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_8   (3UL)    /*!< Capture noise filter clock is PCLK divide by 8 \hideinitializer */
#define LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_16  (4UL)    /*!< Capture noise filter clock is PCLK divide by 16 \hideinitializer */
#define LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_32  (5UL)    /*!< Capture noise filter clock is PCLK divide by 32 \hideinitializer */
#define LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_64  (6UL)    /*!< Capture noise filter clock is PCLK divide by 64 \hideinitializer */
#define LPTMR_CAPTURE_NOISE_FILTER_PCLK_DIV_128 (7UL)    /*!< Capture noise filter clock is PCLK divide by 128 \hideinitializer */

#define LPTMR_CMP_MAX_VALUE                     (0xFFFFFFUL)                        /*!< Max LPTMR compare value \hideinitializer */

#define LPTMR_OK                                ( 0L)    /*!< LPTMR operation OK \hideinitializer */
#define LPTMR_ERR_FAIL                          (-1L)    /*!< LPTMR operation failed \hideinitializer */
#define LPTMR_ERR_TIMEOUT                       (-2L)    /*!< LPTMR operation abort due to timeout error \hideinitializer */

/** @} end of group LPTMR_EXPORTED_CONSTANTS */


/** @addtogroup LPTMR_EXPORTED_FUNCTIONS LPTMR Exported Functions
  @{
*/

/**
  * @brief      Set LPTMR Compared Value
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  * @param[in]  u32Value    LPTMR compare value. Valid values are between 2 to 0xFFFFFF.
  *
  * @return     None
  *
  * @details    This macro is used to set lptmr compared value to adjust lptmr time-out interval.
  * @note       1. Never write 0x0 or 0x1 in this field, or the core will run into unknown state. \n
  *             2. If update lptmr compared value in continuous counting mode, lptmr counter value will keep counting continuously. \n
  *                But if lptmr is operating at other modes, the lptmr up counter will restart counting and start from 0.
  * \hideinitializer
  */
#define LPTMR_SET_CMP_VALUE(lptmr, u32Value)        ((lptmr)->CMP = (u32Value))

/**
  * @brief      Set LPTMR Prescale Value
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  * @param[in]  u32Value    LPTMR prescale value. Valid values are between 0 to 0xFF.
  *
  * @return     None
  *
  * @details    This macro is used to set lptmr prescale value and lptmr source clock will be divided by (prescale + 1) \n
  *             before it is fed into lptmr.
  * \hideinitializer
  */
#define LPTMR_SET_PRESCALE_VALUE(lptmr, u32Value)   ((lptmr)->CTL = ((lptmr)->CTL & ~LPTMR_CTL_PSC_Msk) | (u32Value))

/**
  * @brief      Check specify LPTMR Status
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @retval     0   LPTMR 24-bit up counter is inactive
  * @retval     1   LPTMR 24-bit up counter is active
  *
  * @details    This macro is used to check if specify LPTMR counter is inactive or active.
  * \hideinitializer
  */
#define LPTMR_IS_ACTIVE(lptmr)                      (((lptmr)->CTL & LPTMR_CTL_ACTSTS_Msk)? 1 : 0)

/**
  * @brief      Select Toggle-output Pin
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  * @param[in]  u32ToutSel  Toggle-output pin selection, valid values are:
  *                         - \ref LPTMR_TOUT_PIN_FROM_TMX
  *                         - \ref LPTMR_TOUT_PIN_FROM_TMX_EXT
  *
  * @return     None
  *
  * @details    This macro is used to select lptmr toggle-output pin is output on TMx or TMx_EXT pin.
  * \hideinitializer
  */
#define LPTMR_SELECT_TOUT_PIN(lptmr, u32ToutSel)    ((lptmr)->CTL = ((lptmr)->CTL & ~LPTMR_CTL_TGLPINSEL_Msk) | (u32ToutSel))

/**
  * @brief      Select LPTMR operating mode
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  * @param[in]  u32OpMode   Operation mode. Possible options are
  *                         - \ref LPTMR_ONESHOT_MODE
  *                         - \ref LPTMR_PERIODIC_MODE
  *                         - \ref LPTMR_TOGGLE_MODE
  *                         - \ref LPTMR_CONTINUOUS_MODE
  *
  * @return     None
  * \hideinitializer
  */
#define LPTMR_SET_OPMODE(lptmr, u32OpMode)   ((lptmr)->CTL = ((lptmr)->CTL & ~LPTMR_CTL_OPMODE_Msk) | (u32OpMode))

/* Declare these inline functions here to avoid MISRA C 2004 rule 8.1 error */
__STATIC_INLINE void LPTMR_Start(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_Stop(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_EnableWakeup(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_DisableWakeup(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_StartCapture(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_StopCapture(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_EnableCaptureDebounce(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_DisableCaptureDebounce(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_EnableEventCounterDebounce(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_DisableEventCounterDebounce(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_EnableInt(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_DisableInt(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_EnableCaptureInt(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_DisableCaptureInt(LPTMR_T *lptmr);
__STATIC_INLINE uint32_t LPTMR_GetIntFlag(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_ClearIntFlag(LPTMR_T *lptmr);
__STATIC_INLINE uint32_t LPTMR_GetCaptureIntFlag(LPTMR_T *lptmr);
__STATIC_INLINE uint32_t LPTMR_GetCaptureIntFlagOV(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_ClearCaptureIntFlag(LPTMR_T *lptmr);
__STATIC_INLINE uint32_t LPTMR_GetWakeupFlag(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_ClearWakeupFlag(LPTMR_T *lptmr);
__STATIC_INLINE uint32_t LPTMR_GetCaptureData(LPTMR_T *lptmr);
__STATIC_INLINE uint32_t LPTMR_GetCounter(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_EnablePDCLK(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_DisablePDCLK(LPTMR_T *lptmr);
__STATIC_INLINE void LPTMR_EventCounterSelect(LPTMR_T *lptmr, uint32_t u32Src);

/**
  * @brief      Start LPTMR Counting
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to start LPTMR counting.
  */
__STATIC_INLINE void LPTMR_Start(LPTMR_T *lptmr)
{
    lptmr->CTL |= LPTMR_CTL_CNTEN_Msk;
}

/**
  * @brief      Stop LPTMR Counting
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to stop/suspend LPTMR counting.
  */
__STATIC_INLINE void LPTMR_Stop(LPTMR_T *lptmr)
{
    lptmr->CTL &= ~LPTMR_CTL_CNTEN_Msk;
}

/**
  * @brief      Enable LPTMR Interrupt Wake-up Function
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to enable the lptmr interrupt wake-up function and interrupt source could be time-out interrupt, \n
  *             counter event interrupt or capture trigger interrupt.
  * @note       To wake the system from Power-down mode, lptmr clock source must be ether LXT or LIRC.
  */
__STATIC_INLINE void LPTMR_EnableWakeup(LPTMR_T *lptmr)
{
    lptmr->CTL |= (LPTMR_CTL_WKEN_Msk | LPTMR_CTL_PDCLKEN_Msk);
}

/**
  * @brief      Disable LPTMR Wake-up Function
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to disable the lptmr interrupt wake-up function.
  */
__STATIC_INLINE void LPTMR_DisableWakeup(LPTMR_T *lptmr)
{
    lptmr->CTL &= ~LPTMR_CTL_WKEN_Msk;
}

/**
  * @brief      Start LPTMR Capture Function
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to start LPTMR capture function.
  */
__STATIC_INLINE void LPTMR_StartCapture(LPTMR_T *lptmr)
{
    lptmr->EXTCTL |= LPTMR_EXTCTL_CAPEN_Msk;
}

/**
  * @brief      Stop LPTMR Capture Function
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to stop LPTMR capture function.
  */
__STATIC_INLINE void LPTMR_StopCapture(LPTMR_T *lptmr)
{
    lptmr->EXTCTL &= ~LPTMR_EXTCTL_CAPEN_Msk;
}

/**
  * @brief      Enable Capture Pin De-bounce
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to enable the detect de-bounce function of capture pin.
  */
__STATIC_INLINE void LPTMR_EnableCaptureDebounce(LPTMR_T *lptmr)
{
    lptmr->EXTCTL |= LPTMR_EXTCTL_CAPDBEN_Msk;
}

/**
  * @brief      Disable Capture Pin De-bounce
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to disable the detect de-bounce function of capture pin.
  */
__STATIC_INLINE void LPTMR_DisableCaptureDebounce(LPTMR_T *lptmr)
{
    lptmr->EXTCTL &= ~LPTMR_EXTCTL_CAPDBEN_Msk;
}

/**
  * @brief      Enable Counter Pin De-bounce
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to enable the detect de-bounce function of counter pin.
  */
__STATIC_INLINE void LPTMR_EnableEventCounterDebounce(LPTMR_T *lptmr)
{
    lptmr->EXTCTL |= LPTMR_EXTCTL_CNTDBEN_Msk;
}

/**
  * @brief      Disable Counter Pin De-bounce
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to disable the detect de-bounce function of counter pin.
  */
__STATIC_INLINE void LPTMR_DisableEventCounterDebounce(LPTMR_T *lptmr)
{
    lptmr->EXTCTL &= ~LPTMR_EXTCTL_CNTDBEN_Msk;
}

/**
  * @brief      Enable LPTMR Time-out Interrupt
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to enable the lptmr time-out interrupt function.
  */
__STATIC_INLINE void LPTMR_EnableInt(LPTMR_T *lptmr)
{
    lptmr->CTL |= LPTMR_CTL_INTEN_Msk;
}

/**
  * @brief      Disable LPTMR Time-out Interrupt
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to disable the lptmr time-out interrupt function.
  */
__STATIC_INLINE void LPTMR_DisableInt(LPTMR_T *lptmr)
{
    lptmr->CTL &= ~LPTMR_CTL_INTEN_Msk;
}

/**
  * @brief      Enable Capture Trigger Interrupt
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to enable the lptmr capture trigger interrupt function.
  */
__STATIC_INLINE void LPTMR_EnableCaptureInt(LPTMR_T *lptmr)
{
    lptmr->EXTCTL |= LPTMR_EXTCTL_CAPIEN_Msk;
}

/**
  * @brief      Disable Capture Trigger Interrupt
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to disable the lptmr capture trigger interrupt function.
  */
__STATIC_INLINE void LPTMR_DisableCaptureInt(LPTMR_T *lptmr)
{
    lptmr->EXTCTL &= ~LPTMR_EXTCTL_CAPIEN_Msk;
}

/**
  * @brief      Get LPTMR Time-out Interrupt Flag
  *
  * @param[in]  lptmr   The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @retval     0   LPTMR time-out interrupt did not occur
  * @retval     1   LPTMR time-out interrupt occurred
  *
  * @details    This function indicates lptmr time-out interrupt occurred or not.
  */
__STATIC_INLINE uint32_t LPTMR_GetIntFlag(LPTMR_T *lptmr)
{
    return ((lptmr->INTSTS & LPTMR_INTSTS_TIF_Msk) ? 1UL : 0UL);
}

/**
  * @brief      Clear LPTMR Time-out Interrupt Flag
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function clears lptmr time-out interrupt flag to 0.
  */
__STATIC_INLINE void LPTMR_ClearIntFlag(LPTMR_T *lptmr)
{
    lptmr->INTSTS = LPTMR_INTSTS_TIF_Msk;
}

/**
  * @brief      Get LPTMR Capture Interrupt Flag
  *
  * @param[in]  lptmr   The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @retval     0   LPTMR capture interrupt did not occur
  * @retval     1   LPTMR capture interrupt occurred
  *
  * @details    This function indicates lptmr capture trigger interrupt occurred or not.
  */
__STATIC_INLINE uint32_t LPTMR_GetCaptureIntFlag(LPTMR_T *lptmr)
{
    return ((lptmr->EINTSTS & LPTMR_EINTSTS_CAPIF_Msk) ? 1UL : 0UL);
}

/**
  * @brief      Get LPTMR Capture Interrupt Flag Overrun Status
  *
  * @param[in]  lptmr   The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @retval     0   LPTMR capture interrupt did not overrun
  * @retval     1   LPTMR capture interrupt overrun
  *
  * @details    This function indicates lptmr capture trigger interrupt overrun or not.
  */
__STATIC_INLINE uint32_t LPTMR_GetCaptureIntFlagOV(LPTMR_T *lptmr)
{
    return ((lptmr->EINTSTS & LPTMR_EINTSTS_CAPIFOV_Msk) ? 1UL : 0UL);
}

/**
  * @brief      Clear LPTMR Capture Interrupt Flag
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function clears lptmr capture trigger interrupt flag to 0.
  */
__STATIC_INLINE void LPTMR_ClearCaptureIntFlag(LPTMR_T *lptmr)
{
    lptmr->EINTSTS = LPTMR_EINTSTS_CAPIF_Msk;
}

/**
  * @brief      Get LPTMR Wake-up Flag
  *
  * @param[in]  lptmr   The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @retval     0   LPTMR does not cause CPU wake-up
  * @retval     1   LPTMR interrupt event cause CPU wake-up
  *
  * @details    This function indicates lptmr interrupt event has waked up system or not.
  */
__STATIC_INLINE uint32_t LPTMR_GetWakeupFlag(LPTMR_T *lptmr)
{
    return (lptmr->INTSTS & LPTMR_INTSTS_TWKF_Msk ? 1UL : 0UL);
}

/**
  * @brief      Clear LPTMR Wake-up Flag
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function clears the lptmr wake-up system flag to 0.
  */
__STATIC_INLINE void LPTMR_ClearWakeupFlag(LPTMR_T *lptmr)
{
    lptmr->INTSTS = LPTMR_INTSTS_TWKF_Msk;
}

/**
  * @brief      Get Capture value
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     24-bit Capture Value
  *
  * @details    This function reports the current 24-bit lptmr capture value.
  */
__STATIC_INLINE uint32_t LPTMR_GetCaptureData(LPTMR_T *lptmr)
{
    return lptmr->CAP;
}

/**
  * @brief      Get Counter value
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  *
  * @return     24-bit Counter Value
  *
  * @details    This function reports the current 24-bit lptmr counter value.
  */
__STATIC_INLINE uint32_t LPTMR_GetCounter(LPTMR_T *lptmr)
{
    return lptmr->CNT;
}

/**
  * @brief      Enable LPTMR Power-down Engine Clock
  *
  * @param[in]  lptmr       The pointer of the specified lptmr module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to enable the lptmr Power-down Engine Clock.
  */
__STATIC_INLINE void LPTMR_EnablePDCLK(LPTMR_T *lptmr)
{
    lptmr->CTL |= LPTMR_CTL_PDCLKEN_Msk;
}

/**
  * @brief      Disable LPTMR Power-down Engine Clock
  *
  * @param[in]  lptmr       The pointer of the specified lptmr module. It could be LPTMR0, LPTMR1.
  *
  * @return     None
  *
  * @details    This function is used to disable the lptmr Power-down Engine Clock.
  */
__STATIC_INLINE void LPTMR_DisablePDCLK(LPTMR_T *lptmr)
{
    lptmr->CTL &= ~LPTMR_CTL_PDCLKEN_Msk;
}

/**
  * @brief      Select LPTMR Event Counter Source
  *
  * @param[in]  lptmr       The pointer of the specified LPTMR module. It could be LPTMR0, LPTMR1.
  * @param[in]  u32Src      LPTMR Event Counter source. Possible values are
  *                         - \ref LPTMR_EVENT_COUNTER_SOURCE_TMX
  *                         - \ref LPTMR_EVENT_COUNTER_SOURCE_ACMP0
  *                         - \ref LPTMR_EVENT_COUNTER_SOURCE_ACMP1
  *                         - \ref LPTMR_EVENT_COUNTER_SOURCE_ACMP2
  *                         - \ref LPTMR_EVENT_COUNTER_SOURCE_ACMP3
  * @return     None
  *
  * @details    This API is used to select lptmr Event Counter source from LPTMx or internal singal.
  */
__STATIC_INLINE void LPTMR_EventCounterSelect(LPTMR_T *lptmr, uint32_t u32Src)
{
    lptmr->EXTCTL = (lptmr->EXTCTL & ~LPTMR_EXTCTL_ECNTSSEL_Msk) | u32Src;
}

uint32_t LPTMR_Open(LPTMR_T *lptmr, uint32_t u32Mode, uint32_t u32Freq);
void LPTMR_Close(LPTMR_T *lptmr);
int32_t LPTMR_Delay(LPTMR_T *lptmr, uint32_t u32Usec);
void LPTMR_EnableCapture(LPTMR_T *lptmr, uint32_t u32CapMode, uint32_t u32Edge);
void LPTMR_CaptureSelect(LPTMR_T *lptmr, uint32_t u32Src);
void LPTMR_DisableCapture(LPTMR_T *lptmr);
void LPTMR_EnableEventCounter(LPTMR_T *lptmr, uint32_t u32Edge);
void LPTMR_DisableEventCounter(LPTMR_T *lptmr);
uint32_t LPTMR_GetModuleClock(LPTMR_T *lptmr);
void LPTMR_SetTriggerSource(LPTMR_T *lptmr, uint32_t u32Src);
void LPTMR_SetTriggerTarget(LPTMR_T *lptmr, uint32_t u32Mask);
int32_t LPTMR_ResetCounter(LPTMR_T *lptmr);
void LPTMR_EnableCaptureInputNoiseFilter(LPTMR_T *lptmr, uint32_t u32FilterCount, uint32_t u32ClkSrcSel);
void LPTMR_DisableCaptureInputNoiseFilter(LPTMR_T *lptmr);


/** @} end of group LPTMR_EXPORTED_FUNCTIONS */
/** @} end of group LPTMR_Driver */
/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __LPTMR_H__ */
