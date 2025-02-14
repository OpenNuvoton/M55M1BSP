/**************************************************************************//**
 * @file     lpadc.c
 * @version  V1.00
 * @brief    M55M1 series LPADC driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LPADC_Driver LPADC Driver
  @{
*/

int32_t g_LPADC_i32ErrCode = 0;   /*!< LPADC global error code */

/** @addtogroup LPADC_EXPORTED_FUNCTIONS LPADC Exported Functions
  @{
*/

/**
  * @brief This API configures LPADC module to be ready for convert the input from selected channel
  * @param[in] lpadc The pointer of the specified LPADC module
  * @param[in] u32InputMode Decides the LPADC analog input mode. Valid values are:
  *                          - \ref LPADC_ADCR_DIFFEN_SINGLE_END      :Single-end input mode
  *                          - \ref LPADC_ADCR_DIFFEN_DIFFERENTIAL    :Differential input mode
  * @param[in] u32OpMode Decides the LPADC operation mode. Valid values are:
  *                       - \ref LPADC_ADCR_ADMD_SINGLE               :Single mode.
  *                       - \ref LPADC_ADCR_ADMD_BURST                :Burst mode.
  *                       - \ref LPADC_ADCR_ADMD_SINGLE_CYCLE         :Single cycle scan mode.
  *                       - \ref LPADC_ADCR_ADMD_CONTINUOUS           :Continuous scan mode.
  * @param[in] u32ChMask Channel enable bit. Each bit corresponds to a input channel. Bit 0 is channel 0, bit 1 is channel 1..., bit 15 is channel 15.
  * @return  None
  * @note LPADC can only convert 1 channel at a time. If more than 1 channels are enabled, only channel
  *       with smallest number will be convert.
  * @note This API does not trigger LPADC conversion.
  */
void LPADC_Open(LPADC_T *lpadc, uint32_t u32InputMode, uint32_t u32OpMode, uint32_t u32ChMask)
{

    /*Start the LPADC calibration function*/
    LPADC_Calibration(lpadc);

    lpadc->ADCR = (lpadc->ADCR & (~(LPADC_ADCR_DIFFEN_Msk | LPADC_ADCR_ADMD_Msk))) | (u32InputMode) | (u32OpMode);

    lpadc->ADCHER = (lpadc->ADCHER & ~LPADC_ADCHER_CHEN_Msk) | (u32ChMask);

    return;
}
/**
  * @brief  Calibration the specified LPADC module
  *
  * @param[in]  lpadc The pointer of the specified LPADC module
  *
  * @return     None
  *
  * @details    To decrease the effect of electrical random noise, the calibration mode performs an offset and mismatch measurement cycles.
  *             Afterwards, in normal operation mode, the calibration engine applies to the capacitor array, so that the offset and mismatch are removed.
  * @note       This API will reset and calibrate LPADC if LPADC never be calibrated after chip power on.
  * @note       If chip power off, calibration function should be executed again.
  * @note       This function sets g_LPADC_i32ErrCode to LPADC_TIMEOUT_ERR if CALIF(LPADC_ADCALSTSR[0]) is not set to 1
  * @note       If you use the calibration function again, you must write 1 to clear CALIF (LPADC_ADCALSTSR[[0]).
  */
void LPADC_Calibration(LPADC_T *lpadc)
{
    uint32_t u32Delay = SystemCoreClock;    /* 1 second */

    g_LPADC_i32ErrCode = 0;
    /*Enable the LPADC Power on*/
    LPADC_POWER_ON(lpadc);

    /*Wait the LPADC Power On Ready  */
    while (!(lpadc->ADSR0 & LPADC_ADSR0_ADPRDY_Msk))
    {
        if (--u32Delay == 0)
        {
            g_LPADC_i32ErrCode = LPADC_TIMEOUT_ERR;
            break;
        }
    }

    /* Do calibration for LPADC to decrease the effect of electrical random noise. */
    if ((lpadc->ADCALSTS & LPADC_ADCALSTS_CALIF_Msk) == 0)
    {
        /* Must reset LPADC before LPADC calibration */
        lpadc->ADCR |= LPADC_ADCR_RESET_Msk;

        while ((lpadc->ADCR & LPADC_ADCR_RESET_Msk) == LPADC_ADCR_RESET_Msk)
        {
            if (--u32Delay == 0)
            {
                g_LPADC_i32ErrCode = LPADC_TIMEOUT_ERR;
                break;
            }
        }

        lpadc->ADCALSTS |= LPADC_ADCALSTS_CALIF_Msk;      /* Clear Calibration Finish Interrupt Flag */
        lpadc->ADCAL |= LPADC_ADCAL_CALEN_Msk;            /* Enable Calibration function */
        LPADC_START_CONV(lpadc);                          /* Start to calibration */
        u32Delay = SystemCoreClock;

        while ((lpadc->ADCALSTS & LPADC_ADCALSTS_CALIF_Msk) != LPADC_ADCALSTS_CALIF_Msk)   /* Wait calibration finish */
        {
            if (--u32Delay == 0)
            {
                g_LPADC_i32ErrCode = LPADC_TIMEOUT_ERR;
                break;
            }
        }

    }

}

/**
  * @brief Disable LPADC module
  *
  * @param[in] lpadc The pointer of the specified LPADC module
  *
  * @return None
  */
void LPADC_Close(LPADC_T *lpadc)
{

    /*Disable the LPADC Power on*/
    lpadc->ADCR &= ~LPADC_ADCR_ADEN_Msk;

    /* Unlock protected registers */
    SYS_UnlockReg();
    SYS_ResetModule(SYS_LPADC0RST);
    /* Lock protected registers */
    SYS_LockReg();
    return;
}

/**
  * @brief Configure the hardware trigger condition and enable hardware trigger
  * @param[in] lpadc The pointer of the specified LPADC module
  * @param[in] u32Source Decides the hardware trigger source. Valid values are:
  *                       - \ref LPADC_STADC_TRIGGER          : A/D conversion is started by external STADC pin.
  *                       - \ref LPADC_BPWM_TRIGGER           : A/D conversion is triggered by BPWM.
  *                       - \ref LPADC_EPWM_TRIGGER           : A/D conversion is triggered by EPWM.
  *                       - \ref LPADC_ACMP0_TRIGGER          : A/D conversion is triggered by ACMP0.
  *                       - \ref LPADC_ACMP1_TRIGGER          : A/D conversion is triggered by ACMP1.
  *                       - \ref LPADC_ACMP2_TRIGGER          : A/D conversion is triggered by ACMP2.
  *                       - \ref LPADC_ACMP3_TRIGGER          : A/D conversion is triggered by ACMP3.
  * @param[in] u32Param While LPADC trigger by external pin, this parameter is used to set trigger condition.
  *                     Valid values are:
  *                      - \ref LPADC_ADCR_TRGCOND_LOW_LEVEL     :STADC Low level active
  *                      - \ref LPADC_ADCR_TRGCOND_HIGH_LEVEL    :STADC High level active
  *                      - \ref LPADC_ADCR_TRGCOND_FALLING_EDGE  :STADC Falling edge active
  *                      - \ref LPADC_ADCR_TRGCOND_RISING_EDGE   :STADC Rising edge active
  *                     While LPADC trigger by other source, this parameter is unused.
  * @return None
  * @note Software should disable TRGEN (ADCR[8]) and ADST (ADCR[11]) before change TRGS(ADCR[5:4]).
  */
void LPADC_EnableHWTrigger(LPADC_T *lpadc, uint32_t u32Source, uint32_t u32Param)
{
    /* Software should clear TRGEN bit and ADST bit before changing TRGS bits. */
    lpadc->ADCR &= ~(LPADC_ADCR_TRGEN_Msk | LPADC_ADCR_ADST_Msk);

    if (u32Source == LPADC_STADC_TRIGGER)
    {
        lpadc->ADCR = (lpadc->ADCR & ~(LPADC_ADCR_TRGS_Msk | LPADC_ADCR_TRGCOND_Msk | LPADC_ADCR_TRGEN_Msk)) |
                      ((u32Source) | (u32Param) | LPADC_ADCR_TRGEN_Msk);
    }
    else
    {
        lpadc->ADCR = (lpadc->ADCR & ~(LPADC_ADCR_TRGS_Msk | LPADC_ADCR_TRGCOND_Msk | LPADC_ADCR_TRGEN_Msk)) |
                      ((u32Source) | LPADC_ADCR_TRGEN_Msk);
    }

}

/**
  * @brief Disable hardware trigger LPADC function.
  * @param[in] lpadc The pointer of the specified LPADC module
  * @return None
  */
void LPADC_DisableHWTrigger(LPADC_T *lpadc)
{
    /* Software should clear TRGEN bit and ADST bit before changing TRGS bits. */
    lpadc->ADCR &= ~(LPADC_ADCR_TRGEN_Msk | LPADC_ADCR_ADST_Msk);
    lpadc->ADCR &= ~(LPADC_ADCR_TRGS_Msk | LPADC_ADCR_TRGCOND_Msk);

}

/**
  * @brief Enable the interrupt(s) selected by u32Mask parameter.
  * @param[in] lpadc The pointer of the specified LPADC module
  * @param[in] u32Mask The combination of interrupt status bits listed below. Each bit
  *                    corresponds to a interrupt status. This parameter decides which
  *                    interrupts will be enabled.
  *                     - \ref LPADC_ADF_INT    :LPADC convert complete interrupt
  *                     - \ref LPADC_CMP0_INT   :LPADC comparator 0 interrupt
  *                     - \ref LPADC_CMP1_INT   :LPADC comparator 1 interrupt
  * @return None
  */
void LPADC_EnableInt(LPADC_T *lpadc, uint32_t u32Mask)
{
    if ((u32Mask) & LPADC_ADF_INT)
        lpadc->ADCR |= LPADC_ADCR_ADIE_Msk;

    if ((u32Mask) & LPADC_CMP0_INT)
        lpadc->ADCMPR[0] |= LPADC_ADCMPR_CMPIE_Msk;

    if ((u32Mask) & LPADC_CMP1_INT)
        lpadc->ADCMPR[1] |= LPADC_ADCMPR_CMPIE_Msk;

    return;
}

/**
  * @brief Disable the interrupt(s) selected by u32Mask parameter.
  * @param[in] lpadc The pointer of the specified LPADC module
  * @param[in] u32Mask The combination of interrupt status bits listed below. Each bit
  *                    corresponds to a interrupt status. This parameter decides which
  *                    interrupts will be disabled.
  *                     - \ref LPADC_ADF_INT     :LPADC convert complete interrupt
  *                     - \ref LPADC_CMP0_INT    :LPADC comparator 0 interrupt
  *                     - \ref LPADC_CMP1_INT    :LPADC comparator 1 interrupt
  * @return None
  */
void LPADC_DisableInt(LPADC_T *lpadc, uint32_t u32Mask)
{
    if ((u32Mask) & LPADC_ADF_INT)
        lpadc->ADCR &= ~LPADC_ADCR_ADIE_Msk;

    if ((u32Mask) & LPADC_CMP0_INT)
        lpadc->ADCMPR[0] &= ~LPADC_ADCMPR_CMPIE_Msk;

    if ((u32Mask) & LPADC_CMP1_INT)
        lpadc->ADCMPR[1] &= ~LPADC_ADCMPR_CMPIE_Msk;

    return;
}

/**
  * @brief Set LPADC extend sample time.
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @param[in] u32ModuleNum Decides the sample module number, valid value are 0.
  * @param[in] u32ExtendSampleTime Decides the extend sampling time, the range is from 0~16383 LPADC clock. Valid value are from 0 to 0x3FFF.
  * @return None
  * @details When A/D converting at high conversion rate, the sampling time of analog input voltage may not enough if input channel loading is heavy,
  *         user can extend A/D sampling time after trigger source is coming to get enough sampling time.
  */
void LPADC_SetExtendSampleTime(LPADC_T *lpadc, uint32_t u32ModuleNum, uint32_t u32ExtendSampleTime)
{
    lpadc->ESMPCTL = (lpadc->ESMPCTL & ~LPADC_ESMPCTL_EXTSMPT_Msk) |
                     (u32ExtendSampleTime << LPADC_ESMPCTL_EXTSMPT_Pos);
}

/**
  * @brief  Select and configure Automatic Operation function
  * @param[in] lpadc The pointer of the specified LPADC module
  * @param[in] u32TrigSel  The LPADC Automatic Operation Trigger Source:
  *                       - \ref LPADC_AUTOCTL_TRIGSEL_SOFTWARE : Auto-operation Trigger Source from Software .
  *                       - \ref LPADC_AUTOCTL_TRIGSEL_LPTMR0   : Auto-operation Trigger Source from LPTMR0.
  *                       - \ref LPADC_AUTOCTL_TRIGSEL_LPTMR1   : Auto-operation Trigger Source from LPTMR1.
  *                       - \ref LPADC_AUTOCTL_TRIGSEL_TTMR0    : Auto-operation Trigger Source from TTMR0.
  *                       - \ref LPADC_AUTOCTL_TRIGSEL_TTMR1    : Auto-operation Trigger Source from TTMR1.
  *                       - \ref LPADC_AUTOCTL_TRIGSEL_WKIOA0   : Auto-operation Trigger Source from WKIOA0.
  *                       - \ref LPADC_AUTOCTL_TRIGSEL_WKIOB0   : Auto-operation Trigger Source from WKIOB0.
  *                       - \ref LPADC_AUTOCTL_TRIGSEL_WKIOC0   : Auto-operation Trigger Source from WKIOC0.
  *                       - \ref LPADC_AUTOCTL_TRIGSEL_WKIOD0   : Auto-operation Trigger Source from WKIOD0.
  *                       - \ref LPADC_AUTOCTL_TRIGSEL_ACMP0    : Auto-operation Trigger Source from ACMP0.
  *                       - \ref LPADC_AUTOCTL_TRIGSEL_ACMP1    : Auto-operation Trigger Source from ACMP1.
  *                       - \ref LPADC_AUTOCTL_TRIGSEL_ACMP2    : Auto-operation Trigger Source from ACMP2.
  *                       - \ref LPADC_AUTOCTL_TRIGSEL_ACMP3    : Auto-operation Trigger Source from ACMP3.
  * @return  None
  * @details The function is used to set Automatic Operation relative setting.
  */
void LPADC_SelectAutoOperationMode(LPADC_T *lpadc, uint32_t u32TrigSel)
{
    /* Automatic Operation Mode Enable */
    lpadc->AUTOCTL |= LPADC_AUTOCTL_AUTOEN_Msk;

    lpadc->AUTOCTL &= ~(LPADC_AUTOCTL_TRIGSEL_Msk | LPADC_AUTOCTL_TRIGEN_Msk);

    lpadc->AUTOCTL |= u32TrigSel;
}

/** @} end of group LPADC_EXPORTED_FUNCTIONS */

/** @} end of group LPADC_Driver */

/** @} end of group Standard_Driver */

