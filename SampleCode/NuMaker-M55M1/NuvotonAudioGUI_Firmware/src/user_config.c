/******************************************************************************
 * @file     user_config.c
 * @version  V1.00
 * @brief    User configuration source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "NuMicro.h"
#include "user_config.h"
#include "audio_class.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global Variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
extern uint8_t g_au8ToI2CDataBuf[];
UserConfig_T sUserConfig;
volatile uint8_t g_u8ApplyFlag = 0;
volatile uint8_t g_u8ResetFlag = 0;

/**
  * @brief      Delay time
  *
  * @param[in]  us              Delay time, unit is us
  *
  * @return     None
  *
  * @details    This function is used to delay some time.
  */
void NuvotonDelayUs(uint32_t us)
{
    uint32_t u32RegLockLevel = SYS_IsRegLocked();

    if (u32RegLockLevel)
        SYS_UnlockReg();

    //CLK->CLKSEL0 &= ~CLK_CLKSEL0_STCLKSEL_Msk;
    //CLK->CLKSEL0 |= CLK_CLKSEL0_STCLKSEL_HIRC_DIV2;
    CLK_SetSysTickClockSrc(CLK_STSEL_ST0SEL_HIRC_DIV2);

    // SysTick->LOAD = us * CyclesPerUs;
    SysTick->LOAD = us * 24;
    SysTick->VAL = (0x00);
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) {}

    /* Disable SysTick counter */
    SysTick->CTRL = 0;

    if (u32RegLockLevel)
        SYS_LockReg();
}

/**
 * @brief       SetConfigToDefault
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     Set user configuration to default value.
 */
void SetConfigToDefault(void)
{
    sUserConfig.u8USBDin_Function = Enable;
    sUserConfig.u8USBDout_Function = Enable;
    sUserConfig.u8I2S0DInOut_OnOff = Enable;
    sUserConfig.u8I2S1DInOut_OnOff = Enable;

    sUserConfig.u8I2SDSync_DataType = I2S_Stereo;
    sUserConfig.u8I2SDSync_SerialMode = I2S_Slave;

    sUserConfig.u8I2S0DInOut_Format = I2S_Standard;
    sUserConfig.u8I2S0DInOut_WordSize = I2S_24bit; // old:I2S_16bit
    sUserConfig.u8I2S0DInOut_FrameSize = I2S_64Fs; // old:I2S_32Fs

    sUserConfig.u8I2S1DInOut_Format = I2S_Standard;
    sUserConfig.u8I2S1DInOut_WordSize = I2S_16bit;
    sUserConfig.u8I2S1DInOut_FrameSize = I2S_32Fs;

    sUserConfig.u8InputMultiplexer_DSP = In_Mux_USB;
    sUserConfig.u8InputMultiplexer_Bypass = In_Mux_I2S0;

    sUserConfig.u8OutputMultiplexer_USB = Out_Mux_ByPass;
    sUserConfig.u8OutputMultiplexer_I2S0 = Out_Mux_DPS_Stereo;
    sUserConfig.u8OutputMultiplexer_I2S1 = Out_Mux_DPS_Stereo;

    sUserConfig.u16InputGain_DSP = 48; // 0dB
    sUserConfig.u16InputGain_Bypass = 48;
    sUserConfig.u16OutputGain_USB = 48;
    sUserConfig.u16OutputGain_I2S0 = 48;
    sUserConfig.u16OutputGain_I2S1 = 48;

    sUserConfig.u8SampleRate = SR_48kHz;
    sUserConfig.u8I2SChannelNum = I2S_2CHANNELS;
    sUserConfig.u8I2C_ClockRate = I2C_Fast;
    sUserConfig.u8I2C_Address = I2C_Addr_0x40;
    sUserConfig.u8InputClock_Crystal = Crystal_12_288;

    sUserConfig.u8USBDin_WordSize = USB_16bit;
    sUserConfig.u8USBDout_WordSize = USB_16bit;

    sUserConfig.u8Default_BongioviConfig = 0;
    sUserConfig.u8Default_BoardNum = 0;
}

/**
 * @brief       NuvotonCmdAnalyze
 *
 * @param[in]   None
 *
 * @return      None
 *
 * @details     This function is used to analyze the command from tool and record the setting to user configuration structure.
 */
void NuvotonCmdAnalyze(void)
{
    switch (g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 1])
    {
        case Empty_Function:
            // No data
            break;

        case Firmware_Version:
            // No need to record
            break;

        case Reset_Function:
            g_u8ResetFlag = 1;
            break;

        case Apply_Function:
            g_u8ApplyFlag = 1;
            break;

        case USBDIn_Function:
            sUserConfig.u8USBDin_Function = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case USBDOut_Function:
            sUserConfig.u8USBDout_Function = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case I2S0DInOut_OnOff:
            sUserConfig.u8I2S0DInOut_OnOff = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case I2S1DInOut_OnOff:
            sUserConfig.u8I2S1DInOut_OnOff = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case I2SDSync_DataType:
            sUserConfig.u8I2SDSync_DataType = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case I2SDSync_SerialMode:
            sUserConfig.u8I2SDSync_SerialMode = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case I2S0DInOut_Format:
            sUserConfig.u8I2S0DInOut_Format = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case I2S0DInOut_WordSize:
            sUserConfig.u8I2S0DInOut_WordSize = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case I2S0DInOut_FrameSize:
            sUserConfig.u8I2S0DInOut_FrameSize = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case I2S1DInOut_Format:
            sUserConfig.u8I2S1DInOut_Format = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case I2S1DInOut_WordSize:
            sUserConfig.u8I2S1DInOut_WordSize = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case I2S1DInOut_FrameSize:
            sUserConfig.u8I2S1DInOut_FrameSize = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case InputMultiplexer_DSP:
            sUserConfig.u8InputMultiplexer_DSP = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case InputMultiplexer_Bypass:
            sUserConfig.u8InputMultiplexer_Bypass = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case OutputMultiplexer_USB:
            sUserConfig.u8OutputMultiplexer_USB = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case OutputMultiplexer_I2S0:
            sUserConfig.u8OutputMultiplexer_I2S0 = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case OutputMultiplexer_I2S1:
            sUserConfig.u8OutputMultiplexer_I2S1 = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case InputGain_DSP:
            sUserConfig.u16InputGain_DSP = (g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3] | (g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 4] << 8));
            break;

        case InputGain_Bypass:
            sUserConfig.u16InputGain_Bypass = (g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3] | (g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 4] << 8));
            break;

        case OutputGain_USB:
            sUserConfig.u16OutputGain_USB = (g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3] | (g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 4] << 8));
            break;

        case OutputGain_I2S0:
            sUserConfig.u16OutputGain_I2S0 = (g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3] | (g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 4] << 8));
            break;

        case OutputGain_I2S1:
            sUserConfig.u16OutputGain_I2S1 = (g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3] | (g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 4] << 8));
            break;

        case SampleRate:
            sUserConfig.u8SampleRate = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case I2C_ClockRate:
            sUserConfig.u8I2C_ClockRate = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case I2C_Address:
            sUserConfig.u8I2C_Address = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case InputClock_Crystal:
            sUserConfig.u8InputClock_Crystal = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case USBDin_WordSize:
            sUserConfig.u8USBDin_WordSize = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case USBDout_WordSize:
            sUserConfig.u8USBDout_WordSize = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case I2S0_MCLK:
            sUserConfig.u8I2S0_MCLK = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;

        case I2S1_MCLK:
            sUserConfig.u8I2S1_MCLK = g_au8ToI2CDataBuf[I2C_PREFIX_LEN + 3];
            break;
    }
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/
