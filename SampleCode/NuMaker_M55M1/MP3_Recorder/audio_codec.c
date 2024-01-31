/**************************************************************************//**
 * @file     audio_codec.c
 * @version  V3.00
 * @brief    Audio codec setting.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "config.h"

/*---------------------------------------------------------------------------*/
/* Functions                                                                 */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of NAU8822 with I2C                                         */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_WriteNAU8822(uint8_t u8Addr, uint16_t u16Data)
{
    I2C_START(I2C_PORT);
    I2C_WAIT_READY(I2C_PORT);

    I2C_SET_DATA(I2C_PORT, 0x1A << 1);
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    I2C_SET_DATA(I2C_PORT, (uint8_t)((u8Addr << 1) | (u16Data >> 8)));
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    I2C_SET_DATA(I2C_PORT, (uint8_t)(u16Data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
    I2C_WAIT_READY(I2C_PORT);

    I2C_STOP(I2C_PORT);
}

/* Config play sampling rate */
void NAU8822_ConfigSampleRate(uint32_t u32SampleRate)
{
    printf("[NAU8822] Configure Sampling Rate to %d\n\n", u32SampleRate);

    switch (u32SampleRate)
    {
        case 8000:
            I2C_WriteNAU8822(6, 0x1ED);    /* Divide by 12, 8K */
            I2C_WriteNAU8822(7, 0x00A);    /* 16K for internal filter coefficients */
            break;

        case 16000:
            I2C_WriteNAU8822(6, 0x1AD);    /* Divide by 6, 16K */
            I2C_WriteNAU8822(7, 0x006);    /* 16K for internal filter coefficients */
            break;

        case 32000:
            I2C_WriteNAU8822(6, 0x16D);    /* Divide by 3, 32K */
            I2C_WriteNAU8822(7, 0x002);    /* 32K for internal filter coefficients */
            break;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  NAU8822 Settings with I2C interface                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void NAU8822_Setup(void)
{
    printf("\nConfigure NAU8822 ...");

    I2C_WriteNAU8822(0,  0x000);   /* Reset all registers */
    CLK_SysTickDelay(10000);

    I2C_WriteNAU8822(1,  0x03F);
    I2C_WriteNAU8822(2,  0x1BF);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x011);   /* 16-bit word length, I2S format, Mono */
    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteNAU8822(6,  0x1AD);   /* Divide by 6, 16K */
    I2C_WriteNAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1EF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1EF);   /* ADC right digital volume control */

    I2C_WriteNAU8822(44, 0x003);   /* LMICN/LMICP is connected to PGA */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */

    printf("[OK]\n");
}
