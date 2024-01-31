/**************************************************************************//**
 * @file     lpadc.h
 * @version  V1.00
 * @brief    M55M1 series LPADC driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __LPADC_H__
#define __LPADC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup LPADC_Driver LPADC Driver
  @{
*/

/** @addtogroup LPADC_EXPORTED_CONSTANTS LPADC Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  ADCR Constant Definitions                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define LPADC_ADCR_ADEN_CONVERTER_DISABLE (0UL<<LPADC_ADCR_ADEN_Pos)     /*!< LPADC converter disable \hideinitializer */
#define LPADC_ADCR_ADEN_CONVERTER_ENABLE  (1UL<<LPADC_ADCR_ADEN_Pos)     /*!< LPADC converter enable  \hideinitializer */

#define LPADC_ADCR_ADMD_SINGLE            (0UL<<LPADC_ADCR_ADMD_Pos)     /*!< Single mode           \hideinitializer */
#define LPADC_ADCR_ADMD_BURST             (1UL<<LPADC_ADCR_ADMD_Pos)     /*!< Burst mode            \hideinitializer */
#define LPADC_ADCR_ADMD_SINGLE_CYCLE      (2UL<<LPADC_ADCR_ADMD_Pos)     /*!< Single cycle scan mode    \hideinitializer */
#define LPADC_ADCR_ADMD_CONTINUOUS        (3UL<<LPADC_ADCR_ADMD_Pos)     /*!< Continuous scan mode      \hideinitializer */

#define LPADC_ADCR_DIFFEN_SINGLE_END      (0UL<<LPADC_ADCR_DIFFEN_Pos)   /*!< Single-end input mode     \hideinitializer */
#define LPADC_ADCR_DIFFEN_DIFFERENTIAL    (1UL<<LPADC_ADCR_DIFFEN_Pos)   /*!< Differential input mode   \hideinitializer */

#define LPADC_ADCR_DMOF_UNSIGNED_OUTPUT   (0UL<<LPADC_ADCR_DMOF_Pos)     /*!< Select the straight binary format as the output format of the conversion result   \hideinitializer */
#define LPADC_ADCR_DMOF_TWOS_COMPLEMENT   (1UL<<LPADC_ADCR_DMOF_Pos)     /*!< Select the 2's complement format as the output format of the conversion result    \hideinitializer */

#define LPADC_ADCR_TRGCOND_LOW_LEVEL      (0UL<<LPADC_ADCR_TRGCOND_Pos)  /*!< STADC Low level active    \hideinitializer */
#define LPADC_ADCR_TRGCOND_HIGH_LEVEL     (1UL<<LPADC_ADCR_TRGCOND_Pos)  /*!< STADC High level active   \hideinitializer */
#define LPADC_ADCR_TRGCOND_FALLING_EDGE   (2UL<<LPADC_ADCR_TRGCOND_Pos)  /*!< STADC Falling edge active \hideinitializer */
#define LPADC_ADCR_TRGCOND_RISING_EDGE    (3UL<<LPADC_ADCR_TRGCOND_Pos)  /*!< STADC Rising edge active  \hideinitializer */

#define LPADC_LOW_LEVEL_TRIGGER           ((0UL<<LPADC_ADCR_TRGS_Pos)|LPADC_ADCR_TRGCOND_LOW_LEVEL)     /*!< A/D conversion is triggered by external STADC pin low level  \hideinitializer */
#define LPADC_HIGH_LEVEL_TRIGGER          ((0UL<<LPADC_ADCR_TRGS_Pos)|LPADC_ADCR_TRGCOND_HIGH_LEVEL)    /*!< A/D conversion is triggered by external STADC pin high level  \hideinitializer */
#define LPADC_FALLING_EDGE_TRIGGER        ((0UL<<LPADC_ADCR_TRGS_Pos)|LPADC_ADCR_TRGCOND_FALLING_EDGE)  /*!< A/D conversion is triggered by external STADC pin falling edge \hideinitializer */
#define LPADC_RISING_EDGE_TRIGGER         ((0UL<<LPADC_ADCR_TRGS_Pos)|LPADC_ADCR_TRGCOND_RISING_EDGE)   /*!< A/D conversion is triggered by external STADC pin rising edge  \hideinitializer */

#define LPADC_STADC_TRIGGER               (0UL<<LPADC_ADCR_TRGS_Pos)     /*!< A/D conversion is triggered by external STADC pin   \hideinitializer */
#define LPADC_BPWM_TRIGGER                (2UL<<LPADC_ADCR_TRGS_Pos)     /*!< A/D conversion is triggered by BPWM     \hideinitializer */
#define LPADC_EPWM_TRIGGER                (4UL<<LPADC_ADCR_TRGS_Pos)     /*!< A/D conversion is triggered by EPWM     \hideinitializer */
#define LPADC_ACMP0_TRIGGER               (8UL<<LPADC_ADCR_TRGS_Pos)     /*!< A/D conversion is triggered by ACMP0    \hideinitializer */
#define LPADC_ACMP1_TRIGGER               (9UL<<LPADC_ADCR_TRGS_Pos)     /*!< A/D conversion is triggered by ACMP1    \hideinitializer */
#define LPADC_ACMP2_TRIGGER               (10UL<<LPADC_ADCR_TRGS_Pos)    /*!< A/D conversion is triggered by ACMP2    \hideinitializer */
#define LPADC_ACMP3_TRIGGER               (11UL<<LPADC_ADCR_TRGS_Pos)    /*!< A/D conversion is triggered by ACMP3    \hideinitializer */

#define LPADC_ADCR_RESSEL_12BITS          (0UL<<LPADC_ADCR_RESSEL_Pos)   /*!< Resolution Select 12 Bits  \hideinitializer */
#define LPADC_ADCR_RESSEL_10BITS          (1UL<<LPADC_ADCR_RESSEL_Pos)   /*!< Resolution Select 10 Bits  \hideinitializer */
#define LPADC_ADCR_RESSEL_8BITS           (2UL<<LPADC_ADCR_RESSEL_Pos)   /*!< Resolution Select 8 Bits  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* ADCMPR Constant Definitions                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define LPADC_ADCMPR_CMPD(x)                    ((x) << LPADC_ADCMPR_CMPD_Pos)          /*!< Compare value for compare function     \hideinitializer */
#define LPADC_ADCMPR_CMPMATCNT(x)               (((x)-1) << LPADC_ADCMPR_CMPMATCNT_Pos) /*!< Match count for compare function       \hideinitializer */
#define LPADC_ADCMPR_CMPCH(x)                   ((x) << LPADC_ADCMPR_CMPCH_Pos)         /*!< Compare channel for compare function   \hideinitializer */
#define LPADC_ADCMPR_CMPCOND_LESS_THAN          (0<<LPADC_ADCMPR_CMPCOND_Pos)           /*!< The compare condition is "less than"   \hideinitializer */
#define LPADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL   (1<<LPADC_ADCMPR_CMPCOND_Pos)           /*!< The compare condition is "greater than or equal to"    \hideinitializer */
#define LPADC_ADCMPR_CMPIE_INTERRUPT_ENABLE     (LPADC_ADCMPR_CMPIE_Msk)                /*!< The compare function interrupt enable  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* ADC Interrupt Constant Definitions                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define LPADC_ADF_INT         (LPADC_ADSR0_ADF_Msk)          /*!< LPADC convert complete interrupt    \hideinitializer */
#define LPADC_CMP0_INT        (LPADC_ADSR0_CMPF0_Msk)        /*!< LPADC comparator 0 interrupt        \hideinitializer */
#define LPADC_CMP1_INT        (LPADC_ADSR0_CMPF1_Msk)        /*!< LPADC comparator 1 interrupt        \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* LPADC Operation Mode Constant Definitions                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define LPADC_SINGLE_MODE         0   /*!< LPADC single mode            \hideinitializer */
#define LPADC_BURST_MODE          1   /*!< LPADC burst mode             \hideinitializer */
#define LPADC_SINGLE_CYCLE_MODE   2   /*!< LPADC single-cycle scan mode \hideinitializer */
#define LPADC_CONTINUOUS_MODE     3   /*!< LPADC continuous scan mode   \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* LPADC Trigger Condition Constant Definitions                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define LPADC_LOW_LEVEL      0   /*!< LPADC external trigger condition is low level trigger     \hideinitializer */
#define LPADC_HIGH_LEVEL     1   /*!< LPADC external trigger condition is high level trigger    \hideinitializer */
#define LPADC_FALLING_EDGE   2   /*!< LPADC external trigger condition is falling edge trigger  \hideinitializer */
#define LPADC_RISING_EDGE    3   /*!< LPADC external trigger condition is rising edge trigger   \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* LPADC Compare Condition Constant Definitions                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define LPADC_LESS_THAN          0   /*!< LPADC compare condition is "less than the compare value"                  \hideinitializer */
#define LPADC_GREATER_OR_EQUAL   1   /*!< LPADC compare condition is "greater than or equal to the compare value"   \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  AUTOCTL Constant Definitions                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#define LPADC_AUTOCTL_TRIGSEL_SOFTWARE  (0UL)                                                          /*!< LPADC Automatic Operation Trigger Source Select is Software  \hideinitializer */
#define LPADC_AUTOCTL_TRIGSEL_LPTMR0    (0UL<<LPADC_AUTOCTL_TRIGSEL_Pos | LPADC_AUTOCTL_TRIGEN_Msk)    /*!< LPADC Automatic Operation Trigger Source Select is LPTMR0  \hideinitializer */
#define LPADC_AUTOCTL_TRIGSEL_LPTMR1    (1UL<<LPADC_AUTOCTL_TRIGSEL_Pos | LPADC_AUTOCTL_TRIGEN_Msk)    /*!< LPADC Automatic Operation Trigger Source Select is LPTMR1  \hideinitializer */
#define LPADC_AUTOCTL_TRIGSEL_TTMR0     (2UL<<LPADC_AUTOCTL_TRIGSEL_Pos | LPADC_AUTOCTL_TRIGEN_Msk)    /*!< LPADC Automatic Operation Trigger Source Select is TTMR0  \hideinitializer */
#define LPADC_AUTOCTL_TRIGSEL_TTMR1     (3UL<<LPADC_AUTOCTL_TRIGSEL_Pos | LPADC_AUTOCTL_TRIGEN_Msk)    /*!< LPADC Automatic Operation Trigger Source Select is TTMR1  \hideinitializer */
#define LPADC_AUTOCTL_TRIGSEL_WKIOA0    (4UL<<LPADC_AUTOCTL_TRIGSEL_Pos | LPADC_AUTOCTL_TRIGEN_Msk)    /*!< LPADC Automatic Operation Trigger Source Select is WKIOA0  \hideinitializer */
#define LPADC_AUTOCTL_TRIGSEL_WKIOB0    (5UL<<LPADC_AUTOCTL_TRIGSEL_Pos | LPADC_AUTOCTL_TRIGEN_Msk)    /*!< LPADC Automatic Operation Trigger Source Select is WKIOB0  \hideinitializer */
#define LPADC_AUTOCTL_TRIGSEL_WKIOC0    (6UL<<LPADC_AUTOCTL_TRIGSEL_Pos | LPADC_AUTOCTL_TRIGEN_Msk)    /*!< LPADC Automatic Operation Trigger Source Select is WKIOC0  \hideinitializer */
#define LPADC_AUTOCTL_TRIGSEL_WKIOD0    (7UL<<LPADC_AUTOCTL_TRIGSEL_Pos | LPADC_AUTOCTL_TRIGEN_Msk)    /*!< LPADC Automatic Operation Trigger Source Select is WKIOD0  \hideinitializer */
#define LPADC_AUTOCTL_TRIGSEL_ACMP0     (8UL<<LPADC_AUTOCTL_TRIGSEL_Pos | LPADC_AUTOCTL_TRIGEN_Msk)    /*!< LPADC Automatic Operation Trigger Source Select is ACMP0  \hideinitializer */
#define LPADC_AUTOCTL_TRIGSEL_ACMP1     (9UL<<LPADC_AUTOCTL_TRIGSEL_Pos | LPADC_AUTOCTL_TRIGEN_Msk)    /*!< LPADC Automatic Operation Trigger Source Select is ACMP1  \hideinitializer */
#define LPADC_AUTOCTL_TRIGSEL_ACMP2     (10UL<<LPADC_AUTOCTL_TRIGSEL_Pos | LPADC_AUTOCTL_TRIGEN_Msk)   /*!< LPADC Automatic Operation Trigger Source Select is ACMP2  \hideinitializer */
#define LPADC_AUTOCTL_TRIGSEL_ACMP3     (11UL<<LPADC_AUTOCTL_TRIGSEL_Pos | LPADC_AUTOCTL_TRIGEN_Msk)   /*!< LPADC Automatic Operation Trigger Source Select is ACMP3  \hideinitializer */


/*--------------------------------------------------------------------------------------------------*/
/* Define Error Code                                                                                */
/*--------------------------------------------------------------------------------------------------*/
#define LPADC_TIMEOUT_ERR     (-1)    /*!< LPADC operation abort due to timeout error \hideinitializer */

/** @} end of group LPADC_EXPORTED_CONSTANTS */

extern int32_t g_LPADC_i32ErrCode;

/** @addtogroup LPADC_EXPORTED_FUNCTIONS LPADC Exported Functions
  @{
*/


/**
  * @brief Get conversion data of specified channel.
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @param[in] u32ChNum LPADC Channel, valid value are from 0 to 23 and 28,29,30,31.
  * @return  16-bit data.
  * @details Read RSLT bit field to get conversion data.
  * \hideinitializer
  */
#define LPADC_GET_CONVERSION_DATA(lpadc, u32ChNum) ((lpadc)->ADDR[(u32ChNum)] & LPADC_ADDR_RSLT_Msk)

/**
  * @brief Return the user-specified interrupt flags.
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @param[in] u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *                    Valid values are:
  *                     - \ref LPADC_ADF_INT          :Convert complete interrupt flag.
  *                     - \ref LPADC_CMP0_INT         :Comparator 0 interrupt flag.
  *                     - \ref LPADC_CMP1_INT         :Comparator 1 interrupt flag.
  * @return  User specified interrupt flags.
  * @details Get the status of the LPADC interrupt flag.
  * \hideinitializer
  */
#define LPADC_GET_INT_FLAG(lpadc, u32Mask) ((lpadc)->ADSR0 & (u32Mask))

/**
  * @brief This macro clear the selected interrupt status bits.
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @param[in] u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *                    Valid values are:
  *                     - \ref LPADC_ADF_INT          :Convert complete interrupt flag.
  *                     - \ref LPADC_CMP0_INT         :Comparator 0 interrupt flag.
  *                     - \ref LPADC_CMP1_INT         :Comparator 1 interrupt flag.
  * @return  None
  * @details ADF (ADSR0[0])/CMPF0 (ADSR0[1])/CMPF1 (ADSR0[2]) can be cleared by writing 1 to itself.
  * \hideinitializer
  */
#define LPADC_CLR_INT_FLAG(lpadc, u32Mask) ((lpadc)->ADSR0 = (u32Mask))

/**
  * @brief Get the busy state of LPADC.
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @retval 0 LPADC is not busy.
  * @retval 1 LPADC is busy.
  * @details ADSR0[7] (BUSY) is a mirror of ADCR[11] (ADST).
  * \hideinitializer
  */
#define LPADC_IS_BUSY(lpadc) ((lpadc)->ADSR0 & LPADC_ADSR0_BUSY_Msk ? 1 : 0)

/**
  * @brief Check if the LPADC conversion data is over written or not.
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @param[in] u32ChNum LPADC Channel, valid value are from 0 to 23 and 28,29,30,31.
  * @retval 0 LPADC data is not overrun.
  * @retval 1 LPADC data is overrun.
  * @details ADSR2[31:0] (OVERRUN) is the mirror of ADDR0~31[16] OVERRUN bits.
  * \hideinitializer
  */
#define LPADC_IS_DATA_OVERRUN(lpadc, u32ChNum) (((lpadc)->ADSR2 & (1<<(u32ChNum))) ? 1 : 0)

/**
  * @brief Check if the LPADC conversion data is valid or not.
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @param[in] u32ChNum LPADC Channel, valid value are from 0 to 23 and 28,29,30,31.
  * @retval 0 LPADC data is not valid.
  * @retval 1 LPADC data is valid.
  * @details VALID (ADDR0~31[17]) is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADDR register is read.
  * \hideinitializer
  */
#define LPADC_IS_DATA_VALID(lpadc, u32ChNum) ((lpadc)->ADSR1 & (0x1<<(u32ChNum)) ? 1 : 0)

/**
  * @brief Power down LPADC module.
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @return None
  * @details Disable A/D converter analog circuit for saving power consumption.
  * \hideinitializer
  */
#define LPADC_POWER_DOWN(lpadc) ((lpadc)->ADCR &= ~LPADC_ADCR_ADEN_Msk)

/**
  * @brief Power on LPADC module.
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @return None
  * @details Before starting A/D conversion function, ADEN bit (ADCR[0]) should be set to 1.
  * \hideinitializer
  */
#define LPADC_POWER_ON(lpadc) ((lpadc)->ADCR |= LPADC_ADCR_ADEN_Msk)

/**
  * @brief Configure the comparator 0 and enable it.
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @param[in] u32ChNum  Specifies the source channel, valid value are from 0 to 23 and 28,29,30,31.
  * @param[in] u32Condition Specifies the compare condition. Valid values are:
  *                          - \ref LPADC_ADCMPR_CMPCOND_LESS_THAN            :The compare condition is "less than the compare value".
  *                          - \ref LPADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL     :The compare condition is "greater than or equal to the compare value".
  * @param[in] u32Data Specifies the compare value, valid value are between 0 ~ 0xFFF.
  * @param[in] u32MatchCount Specifies the match count setting, valid values are between 1~16.
  * @return None
  * @details For example, LPADC_ENABLE_CMP0(ADC, 5, LPADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL, 0x800, 10);
  *          means LPADC will assert comparator 0 flag if channel 5 conversion result is greater than or
  *          equal to 0x800 for 10 times continuously.
  * \hideinitializer
  */
#define LPADC_ENABLE_CMP0(lpadc, \
                          u32ChNum, \
                          u32Condition, \
                          u32Data, \
                          u32MatchCount) ((lpadc)->ADCMPR[0] = ((u32ChNum) << LPADC_ADCMPR_CMPCH_Pos) | \
                                                               (u32Condition) | \
                                                               ((u32Data) << LPADC_ADCMPR_CMPD_Pos) | \
                                                               (((u32MatchCount) - 1) << LPADC_ADCMPR_CMPMATCNT_Pos) |\
                                                               LPADC_ADCMPR_CMPEN_Msk)

/**
  * @brief Disable comparator 0
  * @param[in] lpadc The pointer of the specified LPADC module
  * @return None
  * @details Set CMPEN (ADCMPR0[0]) to 0 and reset comparator 0 configurations to disable LPADC compare function.
  * \hideinitializer
  */
#define LPADC_DISABLE_CMP0(lpadc) ((lpadc)->ADCMPR[0] = 0)

/**
  * @brief Configure the comparator 1 and enable it.
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @param[in] u32ChNum  Specifies the source channel, valid value are from 0 to 23 and 28,29,30,31.
  * @param[in] u32Condition Specifies the compare condition. Valid values are:
  *                          - \ref LPADC_ADCMPR_CMPCOND_LESS_THAN            :The compare condition is "less than the compare value".
  *                          - \ref LPADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL     :The compare condition is "greater than or equal to the compare value".
  * @param[in] u32Data Specifies the compare value, valid value are between 0 ~ 0xFFF.
  * @param[in] u32MatchCount Specifies the match count setting, valid values are between 1~16.
  * @return None
  * @details For example, LPADC_ENABLE_CMP1(ADC, 5, LPADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL, 0x800, 10);
  *          means LPADC will assert comparator 1 flag if channel 5 conversion result is greater than or
  *          equal to 0x800 for 10 times continuously.
  * \hideinitializer
  */
#define LPADC_ENABLE_CMP1(lpadc, \
                          u32ChNum, \
                          u32Condition, \
                          u32Data, \
                          u32MatchCount) ((lpadc)->ADCMPR[1] = ((u32ChNum) << LPADC_ADCMPR_CMPCH_Pos) | \
                                                               (u32Condition) | \
                                                               ((u32Data) << LPADC_ADCMPR_CMPD_Pos) | \
                                                               (((u32MatchCount) - 1) << LPADC_ADCMPR_CMPMATCNT_Pos) |\
                                                               LPADC_ADCMPR_CMPEN_Msk)

/**
  * @brief Disable comparator 1.
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @return None
  * @details Set CMPEN (ADCMPR1[0]) to 0 and reset comparator 1 configurations to disable LPADC compare function.
  * \hideinitializer
  */
#define LPADC_DISABLE_CMP1(lpadc) ((lpadc)->ADCMPR[1] = 0)

/**
  * @brief Enable the compare window mode.
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @param[in] u32CMP Specifies the compare register, valid value are 0.
  * @return None
  * @details CMPF0 (ADSR0[1]) will be set when both LPADC_CMP0 and LPADC_CMP1 compared condition matched.
  * \hideinitializer
  */
#define LPADC_ENABLE_CMP_WINDOW_MODE(lpadc, u32CMP) ((lpadc)->ADCMPR[(u32CMP)] |= LPADC_ADCMPR_CMPWEN_Msk)

/**
  * @brief Disable the compare window mode.
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @param[in] u32CMP Specifies the compare register, valid value are 0.
  * @return None
  * @details Disable the compare window mode for specified LPADC module.
  * \hideinitializer
  */
#define LPADC_DISABLE_CMP_WINDOW_MODE(lpadc, u32CMP) ((lpadc)->ADCMPR[(u32CMP)] &= ~LPADC_ADCMPR_CMPWEN_Msk)

/**
  * @brief Set LPADC input channel.
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @param[in] u32Mask  Channel enable bit. Each bit corresponds to a input channel. Bit 0 is channel 0, bit 1 is channel 1..., bit 23 is channel 23.
  * @return None
  * @details Enabled channel will be converted while LPADC starts.
  * @note In single mode, LPADC can only convert 1 channel. If more than 1 channel are enabled, only the channel with smallest number will be converted.
  * \hideinitializer
  */
#define LPADC_SET_INPUT_CHANNEL(lpadc, u32Mask) ((lpadc)->ADCHER = ((lpadc)->ADCHER & ~LPADC_ADCHER_CHEN_Msk) | (u32Mask))

/**
  * @brief Set the output format mode.
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @param[in] u32Format Decides the output format. Valid values are:
  *                      - \ref LPADC_ADCR_DMOF_UNSIGNED_OUTPUT      : Select the straight binary format as the output format of the conversion result.
  *                      - \ref LPADC_ADCR_DMOF_TWOS_COMPLEMENT      : Select the 2's complement format as the output format of the conversion result.
  * @return None
  * @details  The macro is used to set the output format of LPADC differential input mode.
  * @note LPADC compare function can not support 2's complement output format, u32Format should be set to LPADC_ADCR_DMOF_UNSIGNED_OUTPUT.
  * \hideinitializer
  */
#define LPADC_SET_DMOF(lpadc, u32Format) ((lpadc)->ADCR = ((lpadc)->ADCR & ~LPADC_ADCR_DMOF_Msk) | (u32Format))

/**
  * @brief Start the A/D conversion.
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @return None
  * @details Set ADST bit to 1 to start the A/D conversion.
  * \hideinitializer
  */
#define LPADC_START_CONV(lpadc) ((lpadc)->ADCR |= LPADC_ADCR_ADST_Msk)

/**
  * @brief Stop the A/D conversion.
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @return None
  * @details ADST (ADCR[11]) will be cleared to 0 by hardware automatically at the ends of single mode and single-cycle scan mode.
  *          In continuous scan mode and burst mode, A/D conversion is continuously performed until software writes 0 to this bit.
  * @note When the ADST bit is cleared to 0, the ADST bit must be kept at 0 at least one LPADC peripheral clock period
  *       before setting it to 1 again, otherwise the A/D converter may not work.
  *       If ADST bit is cleared to 0 when LPADC is in converting, the BUSY bit will be cleared to 0 immediately,
  *       LPADC will terminate the current conversion and enter idle state directly.
  * \hideinitializer
  */
#define LPADC_STOP_CONV(lpadc) ((lpadc)->ADCR &= ~LPADC_ADCR_ADST_Msk)

/**
  * @brief Enable LPPDMA transfer.
  * @param[in] lpadc The pointer of the specified LPADC module
  * @return None
  * @details Enable LPPDMA to transfer the conversion data.
  * @note While enable PDMA transfer, software must set ADIE = 0 to disable interrupt.
  * \hideinitializer
  */
#define LPADC_ENABLE_LPPDMA(lpadc) ((lpadc)->ADCR |= LPADC_ADCR_PTEN_Msk)

/**
  * @brief Disable LPPDMA transfer.
  * @param[in] lpadc The pointer of the specified LPADC module
  * @return None
  * @details Disable LPPDMA to transfer the conversion data.
  * \hideinitializer
  */
#define LPADC_DISABLE_LPPDMA(lpadc) ((lpadc)->ADCR &= ~LPADC_ADCR_PTEN_Msk)

/**
  * @brief Get PDMA current transfer data
  * @param[in] lpadc The pointer of the specified LPADC module.
  * @return  PDMA current transfer data
  * \hideinitializer
  */
#define LPADC_GET_LPPDMA_DATA(lpadc) ((lpadc)->ADPDMA & LPADC_ADPDMA_CURDAT_Msk)

/**
 *    @brief        Enable specified LPADC Automatic Operation function
 *
 *    @param[in]    lpadc        The pointer of the specified LPADC module
 *
 *    @return       None
 *
 *    @details      This macro enable specified LPADC Automatic Operation function.
 */
#define LPADC_AUTO_OP_ENABLE(lpadc)    ((lpadc)->AUTOCTL |= LPADC_AUTOCTL_AUTOEN_Msk)

/**
 *    @brief        Disable specified LPADC Automatic Operation function
 *
 *    @param[in]    lpadc        The pointer of the specified LPADC module
 *
 *    @return       None
 *
 *    @details      This macro disable specified LPADC Automatic Operation function.
 */
#define LPADC_AUTO_OP_DISABLE(lpadc)    ((lpadc)->AUTOCTL &= ~LPADC_AUTOCTL_AUTOEN_Msk)

/**
 *    @brief        Enable specified LPADC Automatic Operation Software Trigger function
 *
 *    @param[in]    lpadc        The pointer of the specified LPADC module
 *
 *    @return       None
 *
 *    @details      This macro enable specified LPADC Automatic Operation Software Trigger function.
 */
#define LPADC_AUTO_OP_SW_TRIGGER_ENABLE(lpadc)    ((lpadc)->AUTOSTRG |= LPADC_AUTOSTRG_SWTRIG_Msk)

/**
 *    @brief        Enable specified LPADC Automatic Operation Mode Conversion End Wake-up function
 *
 *    @param[in]    lpadc        The pointer of the specified LPADC module
 *
 *    @return       None
 *
 *    \hideinitializer
 */

#define LPADC_CONVERSION_END_WAKEUP_ENABLE(lpadc)    ((lpadc)->AUTOCTL |= LPADC_AUTOCTL_ADWKEN_Msk)
/**
 *    @brief        Disable specified  LPADC Automatic Operation Mode Conversion End Wake-up  function
 *
 *    @param[in]    lpadc        The pointer of the specified LPADC module
 *
 *    @return       None
 *
 *    \hideinitializer
 */
#define LPUART_CONVERSION_END_WAKEUP_DISABLE(lpadc)    ((lpadc)->AUTOCTL &= ~LPADC_AUTOCTL_ADWKEN_Msk)

/**
 *    @brief        Enable specified LPADC Automatic Operation Mode Comparator 0 Wake-up function
 *
 *    @param[in]    lpadc        The pointer of the specified LPADC module
 *
 *    @return       None
 *
 *    \hideinitializer
 */

#define LPADC_COMPARATOR_0_MATCH_WAKEUP_ENABLE(lpadc)    ((lpadc)->AUTOCTL |= LPADC_AUTOCTL_CMP0WKEN_Msk)
/**
 *    @brief        Disable specified LPADC Automatic Operation Mode Comparator 0 Wake-up function
 *
 *    @param[in]    lpadc        The pointer of the specified LPADC module
 *
 *    @return       None
 *
 *    \hideinitializer
 */
#define  LPADC_COMPARATOR_0_MATCH_WAKEUP_DISABLE(lpadc)    ((lpadc)->AUTOCTL &= ~LPADC_AUTOCTL_CMP0WKEN_Msk)


/**
 *    @brief        Enable specified LPADC Automatic Operation Mode Comparator 1 Wake-up function
 *
 *    @param[in]    lpadc    The pointer of the specified LPADC module
 *
 *    @return       None
 *
 *    \hideinitializer
 */

#define  LPADC_COMPARATOR_1_MATCH_WAKEUP_ENABLE(lpadc)    ((lpadc)->AUTOCTL |= LPADC_AUTOCTL_CMP1WKEN_Msk)
/**
 *    @brief        Disable specified LPADC Automatic Operation Mode Comparator 1 Wake-up  function
 *
 *    @param[in]    lpadc   The pointer of the specified LPADC module
 *
 *    @return       None
 *
 *    \hideinitializer
 */
#define  LPADC_COMPARATOR_1_MATCH_WAKEUP_DISABLE(lpadc)    ((lpadc)->AUTOCTL &= ~LPADC_AUTOCTL_CMP1WKEN_Msk)

/**
 *    @brief        Automatic Operation Conversion End Wake-up Status register value
 *
 *    @param[in]    lpadc    The pointer of the specified LPADC module
 *
 *    @retval       0 There is no wakeup for Conversion End.
 *    @retval       1 Chip wake-up from power-down state by Conversion End Wake-up
 *
 *    @details      This macro get Conversion End Wake-up Status register value.
 *    \hideinitializer
 */
#define LPADC_GET_CONVERSION_END_WAKEUP_FLAG(lpadc) (((lpadc)->AUTOSTS & LPADC_AUTOSTS_ADWKF_Msk )>> LPADC_AUTOSTS_ADWKF_Pos)
/**
 *    @brief        Clear Conversion End Wake-up Flag
 *
 *    @param[in]    lpadc    The pointer of the specified LPADC module
 *
 *    @return       None
 *
 *    @details      This macro clear Conversion End Wake-up flag.
 */
#define LPADC_CLEAR_CONVERSION_END_WAKEUP_FLAG(lpadc)    ((lpadc)->AUTOSTS = LPUART_AUTOSTS_AOTOWKF_Msk)

/**
 *    @brief        Automatic Operation  Compare 0 Wake-up Status register value
 *
 *    @param[in]    lpadc    The pointer of the specified LPADC module
 *
 *    @retval       0 There is no wakeup for Compare 0 matching.
 *    @retval       1 Chip wake-up from power-down state by Compare 0 Wake-up
 *
 *    @details      This macro get Compare 0 Wake-up Status register value.
 *    \hideinitializer
 */
#define LPADC_GET_Compare_0_WAKEUP_FLAG(lpadc) (((lpadc)->AUTOSTS & LPADC_AUTOSTS_CMP0WKF_Msk )>> LPADC_AUTOSTS_CMP0WKF_Pos)
/**
 *    @brief        Clear  Compare 0 Wake-up Flag
 *
 *    @param[in]    lpadc    The pointer of the specified LPADC module
 *
 *    @return       None
 *
 *    @details      This macro clear Compare 0 Wake-up flag.
 */
#define LPADC_CLEAR_Compare_0_WAKEUP_FLAG(lpadc)    ((lpadc)->AUTOSTS = LPADC_AUTOSTS_CMP0WKF_Msk)

/**
 *    @brief        Automatic Operation Compare 1 Wake-up Status register value
 *
 *    @param[in]    lpadc    The pointer of the specified LPADC module
 *
 *    @retval       0 There is no wakeup for Compare 1 matching.
 *    @retval       1 Chip wake-up from power-down state by Compare 1 Wake-up
 *
 *    @details      This macro get Compare 1 Wake-up Status register value.
 *    \hideinitializer
 */
#define LPADC_GET_Compare_1_WAKEUP_FLAG(lpadc) (((lpadc)->AUTOSTS & LPADC_AUTOSTS_CMP1WKF_Msk )>> LPADC_AUTOSTS_CMP1WKF_Pos)
/**
 *    @brief        Clear Compare 1 Wake-up Flag
 *
 *    @param[in]    lpadc    The pointer of the specified LPADC module
 *
 *    @return       None
 *
 *    @details      This macro clear Compare 1 Wake-up flag.
 */
#define LPADC_CLEAR_Compare_1_WAKEUP_FLAG(lpadc)    ((lpadc)->AUTOSTS = LPADC_AUTOSTS_CMP1WKF_Msk)

void LPADC_Open(LPADC_T *lpadc, uint32_t u32InputMode, uint32_t u32OpMode, uint32_t u32ChMask);
void LPADC_Calibration(LPADC_T *lpadc);
void LPADC_Close(LPADC_T *lpadc);
void LPADC_EnableHWTrigger(LPADC_T *lpadc, uint32_t u32Source, uint32_t u32Param);
void LPADC_DisableHWTrigger(LPADC_T *lpadc);
void LPADC_EnableInt(LPADC_T *lpadc, uint32_t u32Mask);
void LPADC_DisableInt(LPADC_T *lpadc, uint32_t u32Mask);
void LPADC_SetExtendSampleTime(LPADC_T *lpadc, uint32_t u32ModuleNum, uint32_t u32ExtendSampleTime);
void LPADC_SelectAutoOperationMode(LPADC_T *lpadc, uint32_t u32TrigSel);

/** @} end of group LPADC_EXPORTED_FUNCTIONS */

/** @} end of group LPADC_Driver */

/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif //__LPADC_H__

