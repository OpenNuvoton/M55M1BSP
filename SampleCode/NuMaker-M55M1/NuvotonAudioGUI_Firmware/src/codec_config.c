/******************************************************************************
 * @file     codec_config.c
 * @version  V1.00
 * @brief    NAU88C22 configuration source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

/*!<Includes */
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "codec_config.h"
#include "user_config.h"

volatile S_MIC_I2CCTRL s_MIC_I2CCtrl;
volatile uint8_t g_u8CodecInit = 0;
volatile uint16_t g_u16CodecTimeOutCnt = 0;

// Command for 88C22(transfer via I2C)
#if 1
S_MIC_I2CCMD asMIC_Cmd_88C22_CodecMst[] =
{
    { {0x00, 0x00}, {0x00, 0x00}, },
    { {0x00, 0x01}, {0x01, 0xFF}, },
    { {0x00, 0x02}, {0x01, 0xBF}, },
    { {0x00, 0x03}, {0x00, 0x6F}, },
    { {0x00, 0x04}, {0x00, 0x10}, },
    { {0x00, 0x05}, {0x00, 0x00}, },
    { {0x00, 0x06}, {0x01, 0x49}, },
    { {0x00, 0x07}, {0x00, 0x00}, },
    { {0x00, 0x08}, {0x00, 0x00}, },
    { {0x00, 0x09}, {0x00, 0x00}, },
    { {0x00, 0x0A}, {0x00, 0x00}, },
    { {0x00, 0x0B}, {0x00, 0xFF}, },
    { {0x00, 0x0C}, {0x00, 0xFF}, },
    { {0x00, 0x0D}, {0x00, 0x00}, },
    { {0x00, 0x0E}, {0x01, 0x00}, },
    { {0x00, 0x0F}, {0x00, 0xFF}, },
    { {0x00, 0x10}, {0x00, 0xFF}, },
    { {0x00, 0x11}, {0x00, 0x06}, },
    { {0x00, 0x12}, {0x01, 0x2C}, },
    { {0x00, 0x13}, {0x00, 0x2C}, },
    { {0x00, 0x14}, {0x00, 0x2C}, },
    { {0x00, 0x15}, {0x00, 0x2C}, },
    { {0x00, 0x16}, {0x00, 0x2C}, },
    { {0x00, 0x17}, {0x00, 0x00}, },
    { {0x00, 0x18}, {0x00, 0x32}, },
    { {0x00, 0x19}, {0x00, 0x00}, },
    { {0x00, 0x1A}, {0x00, 0x00}, },
    { {0x00, 0x1B}, {0x00, 0x00}, },
    { {0x00, 0x1C}, {0x00, 0x00}, },
    { {0x00, 0x1D}, {0x00, 0x00}, },
    { {0x00, 0x1E}, {0x00, 0x00}, },
    { {0x00, 0x1F}, {0x00, 0x00}, },
    { {0x00, 0x20}, {0x00, 0x38}, },
    { {0x00, 0x21}, {0x00, 0x0B}, },
    { {0x00, 0x22}, {0x00, 0x32}, },
    { {0x00, 0x23}, {0x00, 0x00}, },
    { {0x00, 0x24}, {0x00, 0x08}, },
    { {0x00, 0x25}, {0x00, 0x0C}, },
    { {0x00, 0x26}, {0x00, 0x93}, },
    { {0x00, 0x27}, {0x00, 0xE9}, },
    { {0x00, 0x28}, {0x00, 0x00}, },
    { {0x00, 0x29}, {0x00, 0x00}, },
    { {0x00, 0x2A}, {0x00, 0x00}, },
    { {0x00, 0x2B}, {0x00, 0x01}, },
    { {0x00, 0x2C}, {0x00, 0x33}, },
    { {0x00, 0x2D}, {0x00, 0x10}, },
    { {0x00, 0x2E}, {0x00, 0x10}, },
    { {0x00, 0x2F}, {0x01, 0x05}, },
    { {0x00, 0x30}, {0x01, 0x05}, },
    { {0x00, 0x31}, {0x00, 0x02}, },
    { {0x00, 0x32}, {0x01, 0x01}, },
    { {0x00, 0x33}, {0x00, 0x01}, },
    { {0x00, 0x34}, {0x00, 0x39}, },
    { {0x00, 0x35}, {0x00, 0x39}, },
    { {0x00, 0x36}, {0x00, 0x39}, },
    { {0x00, 0x37}, {0x00, 0x39}, },
    { {0x00, 0x38}, {0x00, 0x01}, },
    { {0x00, 0x39}, {0x00, 0x01}, },
    { {0x00, 0x3A}, {0x00, 0x00}, },
    { {0x00, 0x3B}, {0x00, 0x00}, },
    { {0x00, 0x3C}, {0x00, 0x20}, },
    { {0x00, 0x3D}, {0x00, 0x20}, },
    { {0x00, 0x3E}, {0x00, 0x7F}, },
    { {0x00, 0x3F}, {0x00, 0x1A}, },
    { {0x00, 0x46}, {0x00, 0x10}, },
    { {0x00, 0x47}, {0x00, 0x10}, },
    { {0x00, 0x48}, {0x00, 0x08}, },
    { {0x00, 0x49}, {0x00, 0x00}, },
    { {0x00, 0x4A}, {0x00, 0x00}, },
    { {0x00, 0x4C}, {0x00, 0x03}, },
    { {0x00, 0x4D}, {0x00, 0x00}, },
    { {0x00, 0x4E}, {0x00, 0x01}, },
    { {0x00, 0x4F}, {0x00, 0x00}, },
    { {0x00, 0x51}, {0x00, 0x00}, },
};
#else
S_MIC_I2CCMD asMIC_Cmd_88C22_CodecMst[] =
{
    { {0x00, 0x00}, {0x00, 0x00}, },
    { {0x00, 0x31}, {0x00, 0x00}, },
    { {0x00, 0x01}, {0x01, 0x24}, },
    { {0x00, 0x01}, {0x01, 0x2D}, },
    { {0x00, 0x34}, {0x00, 0xC0}, },
    { {0x00, 0x35}, {0x00, 0xC0}, },
    { {0x00, 0x36}, {0x00, 0xC0}, },
    { {0x00, 0x37}, {0x00, 0xC0}, },
    { {0x00, 0x04}, {0x00, 0x10}, },
    { {0x00, 0x05}, {0x00, 0x00}, },
    { {0x00, 0x06}, {0x01, 0x4D}, },
    { {0x00, 0x07}, {0x00, 0x00}, },
    { {0x00, 0x0A}, {0x00, 0x09}, },
    { {0x00, 0x0E}, {0x01, 0x08}, },
    { {0x00, 0x0F}, {0x01, 0xEF}, },
    { {0x00, 0x10}, {0x01, 0xEF}, },
    { {0x00, 0x2C}, {0x00, 0x00}, },
    { {0x00, 0x02}, {0x01, 0xB3}, },
    { {0x00, 0x03}, {0x00, 0x7F}, },
    { {0x00, 0x32}, {0x01, 0x01}, },
    { {0x00, 0x33}, {0x01, 0x01}, },
    { {0x00, 0x34}, {0x00, 0x90}, },
    { {0x00, 0x35}, {0x01, 0x90}, },
    { {0x00, 0x36}, {0x00, 0x90}, },
    { {0x00, 0x37}, {0x01, 0x90}, },
    { {0x00, 0x0A}, {0x00, 0x08}, },
};
#endif

/**
 * @brief       Codec_InitCallback
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     The function is I2C callback function. It will be called in I2C IRQ handler.
 */
void Codec_InitCallback(void)
{
    if (I2C_GET_TIMEOUT_FLAG(I2C_PORT))
    {
        I2C_ClearTimeoutFlag(I2C_PORT);
    }
    else
    {
        uint8_t u8Temp;

        switch (I2C_GET_STATUS(I2C_PORT))
        {
            /* START has been transmitted and Write SLA+W to Register I2CDAT. */
            case 0x08:
                I2C_SET_DATA(I2C_PORT, s_MIC_I2CCtrl.u8DeviceAddr << 1);
                I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
                break;

            /* SLA+W has been transmitted and ACK has been received. */
            case 0x18:
                u8Temp = s_MIC_I2CCtrl.u16Counter++;
                u8Temp = s_MIC_I2CCtrl.pau8Cmd[u8Temp];
                I2C_SET_DATA(I2C_PORT, u8Temp);
                I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
                break;

            /* SLA+W has been transmitted and NACK has been received. */
            case 0x20:
                I2C_STOP(I2C_PORT);
                I2C_START(I2C_PORT);
                s_MIC_I2CCtrl.u16MaxCount = 0;
                break;

            /* DATA has been transmitted and ACK has been received. */
            case 0x28:
                u8Temp = s_MIC_I2CCtrl.u16MaxCount;

                if (s_MIC_I2CCtrl.u16Counter < u8Temp)
                {
                    u8Temp = s_MIC_I2CCtrl.u16Counter++;
                    I2C_SET_DATA(I2C_PORT, s_MIC_I2CCtrl.pau8Cmd[u8Temp]);
                    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_SI);
                }
                else
                {
                    I2C_SET_CONTROL_REG(I2C_PORT, I2C_CTL_STO_SI);
                    s_MIC_I2CCtrl.u16MaxCount = 0;
                }

                break;
        }
    }
}

/**
 * @brief       NAU88C22_CodecMst_CallBackInit
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to initialize NAU88C22 via I2C
 */
void NAU88C22_CodecMst_CallBackInit(void)
{
    uint16_t u16i;
    s_MIC_I2CCtrl.u8DeviceAddr = 0x1A;

    g_u8CodecInit = 1;
    g_u16CodecTimeOutCnt = 0;

    for (u16i = 0; u16i < sizeof(asMIC_Cmd_88C22_CodecMst) / sizeof(S_MIC_I2CCMD); u16i++)
    {
        s_MIC_I2CCtrl.pau8Cmd = (uint8_t *)&asMIC_Cmd_88C22_CodecMst[u16i];
        s_MIC_I2CCtrl.u16Counter = 0;
        s_MIC_I2CCtrl.u16MaxCount = sizeof(S_MIC_I2CCMD);
        I2C_START(I2C_PORT);

        /* Wait for I2C transmit completed. */
        while ((s_MIC_I2CCtrl.u16MaxCount > 0) && (g_u16CodecTimeOutCnt < 100)) {} // 100ms timeout

        if (g_u16CodecTimeOutCnt >= 100)
        {
            I2C_STOP(I2C_PORT);
            break;
        }
    }

    g_u8CodecInit = 0;
    g_u16CodecTimeOutCnt = 0;
}

/**
 * @brief       I2C_WriteNAU88C22
 *
 * @param[in]   u8Addr   Register address
 * @param[in]   u16Data  Register data
 *
 * @return      None
 *
 * @details     This function is used to write NAU88C22 register via I2C
 */
void I2C_WriteNAU88C22(uint8_t u8Addr, uint16_t u16Data)
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

/**
 * @brief       NAU88C22_CodecMst_Init
 *
 * @param[in]   u32SampleRate  Sampling rate
 *
 * @return      None
 *
 * @details     This function is used to initialize NAU88C22 as master mode via I2C
 */
void NAU88C22_CodecMst_Init(uint32_t u32SampleRate)
{
    printf("\nConfigure NAU88C22 ...");

    I2C_Open(I2C_PORT, 400000);

    // 0) Full reset
    I2C_WriteNAU88C22(0,  0x000);                 // R0: Software Reset
    CLK_SysTickDelay(10000);                      // >100us to ensure internal POR completes

    // 1) Depending on VDDSPK, disable boost if VDDSPK ≤ 3.6V
    I2C_WriteNAU88C22(49, 0x000);                 // R49: SPKBST/AUX1BST/AUX2BST = 0 (low voltage mode)

    // 2) Enable DC tie-off/IO buffer to slowly charge coupling capacitors
    I2C_WriteNAU88C22(1,  0x124);                 // R1: IOBUFEN=1 (DC/IO buffer enable)
    I2C_WriteNAU88C22(1,  0x1FF);                 // R1: REFIMP=80kΩ, ABIASEN=1, slow charge reference
    CLK_SysTickDelay(250000);                     // Wait ~250ms (longer depending on external caps)

    // 3) Set all analog outputs to "mute + minimum gain + ZC" (no update yet)
    //    Format: Bit8=Update, Bit7=ZC, Bit6=Mute, Bits5:0=Gain(-57dB→0x00)
    //    Value = (0<<8) | (1<<7) | (1<<6) | 0x00 = 0x0C0
    I2C_WriteNAU88C22(52, 0x0C0);                 // R52 LHP: ZC=1, Mute=1, Gain=-57dB
    I2C_WriteNAU88C22(53, 0x0C0);                 // R53 RHP
    I2C_WriteNAU88C22(54, 0x0C0);                 // R54 LSPK
    I2C_WriteNAU88C22(55, 0x0C0);                 // R55 RSPK

    // 4) Other digital/interface settings (keep DAC soft-mute ON until last step)
    I2C_WriteNAU88C22(4,  0x010);                 // R4: I2S, 24bit
    I2C_WriteNAU88C22(5,  0x000);                 // R5: no companding
    I2C_WriteNAU88C22(6,  0x149);                 // R6: 48k
    I2C_WriteNAU88C22(7,  0x000);                 // R7: filter coeffs for 48k
    // Recommend: R10 enable DAC soft-mute here
    I2C_WriteNAU88C22(10, 0x00C);                 // R10: soft-mute ON
    // ADC settings
    I2C_WriteNAU88C22(14, 0x100);
    I2C_WriteNAU88C22(15, 0x1FF);
    I2C_WriteNAU88C22(16, 0x1FF);

    // 5) Do not connect LIN directly to ADC MIX if using DAC playback (avoid multi-path transients)
    I2C_WriteNAU88C22(44, 0x000);                 // R44: disconnect PGA source
    I2C_WriteNAU88C22(47, 0x100);                 // R47: LPGA boost
    I2C_WriteNAU88C22(48, 0x100);                 // R48: RPGA boost
    I2C_WriteNAU88C22(49, 0x002);

    CLK_SysTickDelay(250000);                    // Wait ~250ms (longer depending on external caps)
    // 6) Power on but keep outputs muted
    I2C_WriteNAU88C22(2,  0x1BF);                 // R2: enable L/R HP, ADC Mix/Boost, ADC
    I2C_WriteNAU88C22(3,  0x06F);                 // R3: enable L/R main mixer, DAC (DAC still soft-muted)
    I2C_WriteNAU88C22(43, 0x001);                 // R43: RSPK submixer enabled
    I2C_WriteNAU88C22(50, 0x101);                 // R50: LMIX connect to DAC
    I2C_WriteNAU88C22(51, 0x001);                 // R51: RMIX connect to DAC

    // *** Important: after enabling HP/SPK and MCLK/sample rate ready, wait ≥250ms before sending I²S ***
    CLK_SysTickDelay(250000);

    // 7) Use "Update" to simultaneously bring gain from -57dB to target (e.g. 0dB), and unmute
    //    Target 0dB → Gain_code=0x39; Value = (1<<8)|(1<<7)|(0<<6)|0x39 = 0x1B9
    I2C_WriteNAU88C22(52, 0x039);                 // LHP: Update=1, ZC=1, Mute=0, 0dB
    I2C_WriteNAU88C22(53, 0x039);                 // RHP
    I2C_WriteNAU88C22(54, 0x039);                 // LSPK
    I2C_WriteNAU88C22(55, 0x039);                 // RSPK

    I2C_WriteNAU88C22(44, 0x033);                 // R44: connect Microphone to PGA
    I2C_WriteNAU88C22(72, 0x008);

    // 8) Finally disable DAC soft-mute to start normal playback
    I2C_WriteNAU88C22(10, 0x008);                 // R10: soft-mute OFF

    // Configure sample rate
    if ((u32SampleRate % 8) == 0)
    {
        I2C_WriteNAU88C22(36, 0x008);    //12.288Mhz
        I2C_WriteNAU88C22(37, 0x00C);
        I2C_WriteNAU88C22(38, 0x093);
        I2C_WriteNAU88C22(39, 0x0E9);
    }
    else
    {
        I2C_WriteNAU88C22(36, 0x007);    //11.2896Mhz
        I2C_WriteNAU88C22(37, 0x021);
        I2C_WriteNAU88C22(38, 0x161);
        I2C_WriteNAU88C22(39, 0x026);
    }

    switch (u32SampleRate)
    {
        case 16000:
            I2C_WriteNAU88C22(6, 0x1A9);    /* Divide by 6, 16K */
            I2C_WriteNAU88C22(7, 0x006);    /* 16K for internal filter coefficients */
            break;

        case 44100:
            I2C_WriteNAU88C22(6, 0x149);    /* Divide by 2, 48K */
            I2C_WriteNAU88C22(7, 0x000);    /* 48K for internal filter coefficients */
            break;

        case 48000:
            I2C_WriteNAU88C22(6, 0x149);    /* Divide by 2, 48K */
            I2C_WriteNAU88C22(7, 0x000);    /* 48K for internal filter coefficients */
            break;
    }

    I2C_Close(I2C_PORT);

    printf("[OK]\n");
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
