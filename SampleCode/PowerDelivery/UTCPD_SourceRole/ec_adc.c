/**************************************************************************//**
 * @file     ec_adc.c
 * @version  V0.10
 * @brief    The driver use timer to trigger EADC periodicly to generate VBUS and VCONN Voltage.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "utcpdlib.h"

volatile uint32_t g_u32AdcIntFlag0;
volatile uint32_t g_u32AdcIntFlag1;
/*---------------------------------------------------------------------------------------------------------*/
/*                            ISR to handle EADC00 interrupt event                                         */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void EADC00_IRQHandler(void)
{
    g_u32AdcIntFlag0 = 1;
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}
/*---------------------------------------------------------------------------------------------------------*/
/*                            ISR to handle EADC01 interrupt event                                         */
/*---------------------------------------------------------------------------------------------------------*/
NVT_ITCM void EADC01_IRQHandler(void)
{
    g_u32AdcIntFlag1 = 1;
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF1_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}

void EADC_ConfigPins(void)
{
    /* Set PB.2 - PB.3 to input mode For EADC pin to measure VBUS and VCONN */
    GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);    //For VBUS Voltage
    GPIO_SetMode(PB, BIT3, GPIO_MODE_INPUT);    //For VCONN Voltage


    /* Configure the PB.2 - PB.3 ADC analog input pins. */
    SET_EADC0_CH2_PB2();
    SET_EADC0_CH3_PB3();
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2 | BIT3);
}
void EADC_Init(void)
{
    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC0, EADC_CTL_DIFFEN_SINGLE_END);

    /* Configure the sample module 0 for analog input channel 2 and timer0 trigger source.*/
    EADC_ConfigSampleModule(EADC0, 0, EADC_TIMER1_TRIGGER, 2);
    EADC_SetExtendSampleTime(EADC0, 0, 0xff);

    /* Configure the sample module 1 for analog input channel 3 and timer0 trigger source.*/
    EADC_ConfigSampleModule(EADC0, 1, EADC_TIMER1_TRIGGER, 3);
    EADC_SetExtendSampleTime(EADC0, 1, 0xff);

    /* Enable the sample module 0 interrupt.  */
    EADC_ENABLE_INT(EADC0, BIT0);                    //Enable sample module 0 ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT0);   //Enable sample module 0 interrupt.
    NVIC_EnableIRQ(EADC00_IRQn);

    /* Enable the sample module 1 interrupt.  */
    EADC_ENABLE_INT(EADC0, BIT1);                    //Enable sample module 1 ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 1, BIT1);   //Enable sample module 1 interrupt.
    NVIC_EnableIRQ(EADC01_IRQn);

    /* Enable the sample module 2 interrupt.  */
    EADC_ENABLE_INT(EADC0, BIT2);                    //Enable sample module 2 ADINT0 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 2, BIT2);   //Enable sample module 2 interrupt.
    NVIC_EnableIRQ(EADC02_IRQn);

    GPIO_DISABLE_DIGITAL_PATH(PA, BIT12);
    UTCPD->MUXSEL = (UTCPD->MUXSEL & ~(UTCPD_MUXSEL_ADCSELVB_Msk | UTCPD_MUXSEL_ADCSELVC_Msk)) | ((2 << UTCPD_MUXSEL_ADCSELVB_Pos) | (3 << UTCPD_MUXSEL_ADCSELVC_Pos));
    UTCPD->VBVOL = (UTCPD->VBVOL & ~UTCPD_VBVOL_VBSCALE_Msk) | (2 << UTCPD_VBVOL_VBSCALE_Pos);
}