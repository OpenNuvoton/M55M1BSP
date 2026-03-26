/******************************************************************************
 * @file     adc_converter.h
 * @version  V1.00
 * @brief    NAU7802 ADC converter header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __ADC_CONVERTER_H__
#define __ADC_CONVERTER_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void NAU7802_Polling(void);
void NAU7802_Time_Delay(void);
void NAU7802_Conversion_Result(void);
int32_t NAU7802_Read_Ch(void);
void NAU7802_Write_Ch(uint8_t ch);
void NAU7802_Switch_Ch(void);
void NAU7802_Switch_CS(uint8_t ch);
int32_t NAU7802_Check_CR(void);

#ifdef __cplusplus
}
#endif

#endif  // __ADC_CONVERTER_H__

