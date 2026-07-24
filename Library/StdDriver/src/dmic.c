/**************************************************************************//**
 * @file     dmic.c
 * @version  V1.00
 * @brief    DMIC driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup DMIC_Driver DMIC Driver
  @{
*/

/** @addtogroup DMIC_EXPORTED_FUNCTIONS DMIC Exported Functions
  @{
*/

/**
  * @brief      Enable DMIC's channel
  * @param[in]  dmic The base address of DMIC module
  * @param[in]  u32ChMsk Enable channel Msk.
  *             - \ref DMIC_CTL_CHEN0_Msk
  *             - \ref DMIC_CTL_CHEN1_Msk
  *             - \ref DMIC_CTL_CHEN2_Msk
  *             - \ref DMIC_CTL_CHEN3_Msk
  * @return     None
  * @details    Enable channel to start input data.
  */
void DMIC_EnableChMsk(DMIC_T *dmic, uint32_t u32ChMsk)
{
    dmic->DIV |= DMIC_DIV_HPF_CUT_F;
    dmic->CTL |= (DMIC_CTL_CH01HPFEN_Msk | DMIC_CTL_CH23HPFEN_Msk);
    dmic->CTL |= u32ChMsk;
}

/**
  * @brief      Disable DMIC's channel
  * @param[in]  dmic The base address of DMIC module
  * @param[in]  u32ChMsk Disable channel Msk.
  *             - \ref DMIC_CTL_CHEN0_Msk
  *             - \ref DMIC_CTL_CHEN1_Msk
  *             - \ref DMIC_CTL_CHEN2_Msk
  *             - \ref DMIC_CTL_CHEN3_Msk
  * @return     None
  * @details    Disable channel to input data.
  */
void DMIC_DisableChMsk(DMIC_T *dmic, uint32_t u32ChMsk)
{
    dmic->CTL &= ~(u32ChMsk);
}

/**
  * @brief      Start DMIC module
  * @param[in]  dmic The base address of DMIC module.
  * @return     None.
  */
void DMIC_Open(DMIC_T *dmic)
{
    uint32_t u32Delay;
    uint32_t u32RamIdx;
    uint32_t u32IsRegLocked;
    u32IsRegLocked = SYS_IsRegLocked();

    if (u32IsRegLocked)
    {
        SYS_UnlockReg();
    }

    (void)PMC_SetDMIC_SRAMPowerMode(PMC_SRAM_NORMAL);

    if (u32IsRegLocked)
    {
        SYS_LockReg();
    }

    dmic->DIV |= DMIC_DIV_FCLR_Msk;
    u32Delay = SystemCoreClock >> 3;

    while ((((dmic->DIV & DMIC_DIV_FCLR_Msk) == DMIC_DIV_FCLR_Msk) && ((--u32Delay) > 0UL)))
    {
    }

    dmic->CTL |= DMIC_CTL_SWRST_Msk;
    dmic->CTL |= (DMIC_CTL_DSPMEMT_Msk);

    for (u32RamIdx = 0UL; u32RamIdx < 128UL; u32RamIdx++)
    {
        if ((u32RamIdx == DMIC_RAM_LGAIN_ADDR) || (u32RamIdx == DMIC_RAM_RGAIN_ADDR))
        {
            //Set gain volume 0db((-128-gain)*4096=F80000)
            outp32(DMIC_DSP0_RAMDATA, 0xF80000UL);
            outp32(DMIC_DSP1_RAMDATA, 0xF80000UL);
        }
        else if ((u32RamIdx == DMIC_RAM_LINITSAMPLE_ADDR) || (u32RamIdx == DMIC_RAM_RINITSAMPLE_ADDR))
        {
            //Set initial sample = 1024(0x400)
            outp32(DMIC_DSP0_RAMDATA, 0x400UL);
            outp32(DMIC_DSP1_RAMDATA, 0x400UL);
        }
        else
        {
            outp32(DMIC_DSP0_RAMDATA, 0x0UL);
            outp32(DMIC_DSP1_RAMDATA, 0x0UL);
        }
    }

    dmic->CTL &= (~DMIC_CTL_DSPMEMT_Msk);
    dmic->DIV |= DMIC_DIV_HPF_CUT_F;
    dmic->CTL |= (DMIC_CTL_CH01HPFEN_Msk | DMIC_CTL_CH23HPFEN_Msk);
}

/**
  * @brief      Stop DMIC module
  * @param[in]  dmic The base address of DMIC module.
  * @return     None.
  */
void DMIC_Close(DMIC_T *dmic)
{
    DMIC_DisableChMsk(dmic, DMIC_CTL_CHEN0_Msk | DMIC_CTL_CHEN1_Msk | DMIC_CTL_CHEN2_Msk | DMIC_CTL_CHEN3_Msk);
}

/**
  * @brief      Set DMIC DSP Gain Volume.
  * @param[in]  dmic The base address of DMIC module
  * @param[in]  u32ChMsk Select channel.
  *             - \ref DMIC_CTL_CHEN0_Msk
  *             - \ref DMIC_CTL_CHEN1_Msk
  *             - \ref DMIC_CTL_CHEN2_Msk
  *             - \ref DMIC_CTL_CHEN3_Msk
  * @param[in]  i16ChVolume Gain Volume. CHVOL= -128-(Real_Gain)(dB),9-bit signed 2s complement number.
  * @return     None
  * @details    Set DMIC DSP volume/gain,if the desired gain is 0dB, the program value will be -128 (0xC000).
    *             if the desired gain is -20dB, then the programmed value will be -108(0xCA00)
  */
void DMIC_SetDSPGainVolume(DMIC_T *dmic, uint32_t u32ChMsk, int16_t i16ChVolume)
{
    int32_t i32TmpChVolume = -128L - (int32_t)i16ChVolume;
    int32_t i32SetChVolume;
    uint32_t u32SetChVolume;

    if (i32TmpChVolume > 0L)
    {
        i32SetChVolume = i32TmpChVolume * 128L;
    }
    else
    {
        i32SetChVolume = (i32TmpChVolume * 128L) | (int32_t)BIT15;
    }

    u32SetChVolume = (uint32_t)((uint16_t)i32SetChVolume);

    if ((u32ChMsk & DMIC_CTL_CHEN0_Msk) != 0UL)
    {
        (dmic)->GAINCTL0 = (dmic->GAINCTL0 & ~(DMIC_GAINCTL0_CHyyLVOL_Msk)) | (u32SetChVolume & DMIC_GAINCTL0_CHyyLVOL_Msk);
    }

    if ((u32ChMsk & DMIC_CTL_CHEN1_Msk) != 0UL)
    {
        (dmic)->GAINCTL0 = (dmic->GAINCTL0 & ~(DMIC_GAINCTL0_CHxxRVOL_Msk)) | (u32SetChVolume << DMIC_GAINCTL0_CHxxRVOL_Pos);
    }

    if ((u32ChMsk & DMIC_CTL_CHEN2_Msk) != 0UL)
    {
        (dmic)->GAINCTL1 = (dmic->GAINCTL1 & ~(DMIC_GAINCTL1_CHyyLVOL_Msk)) | (u32SetChVolume & DMIC_GAINCTL1_CHyyLVOL_Msk);
    }

    if ((u32ChMsk & DMIC_CTL_CHEN3_Msk) != 0UL)
    {
        (dmic)->GAINCTL1 = (dmic->GAINCTL1 & ~(DMIC_GAINCTL1_CHxxRVOL_Msk)) | (u32SetChVolume << DMIC_GAINCTL1_CHxxRVOL_Pos);
    }
}

/**
  * @brief      Clear the FIFO
  * @param[in]  dmic The base address of DMIC module
  *
  * @details    To clear the FIFO, need to write FCLR to 11b,
    *             and can read the EMPTY bit to make sure that the FIFO has been cleared.
  */
void DMIC_ClearFIFO(DMIC_T *dmic)
{
    uint32_t u32Delay;
    (dmic)->DIV |= DMIC_DIV_FCLR_Msk;
    u32Delay = SystemCoreClock >> 3;

    while (((!DMIC_IS_FIFOEMPTY(dmic)) && ((--u32Delay) > 0UL)))
    {
    }
}

/**
  * @brief      Get the sample Rate of DMIC
  * @param[in]  dmic The base address of DMIC module
  *
  * @return     Real sample rate. 0 is DMIC clock source error.
  */
uint32_t DMIC_GetSampleRate(const DMIC_T *dmic)
{
    uint16_t const au16OSRTable[] = {64, 128, 256, 100, 64, 64, 64, 50};
    uint32_t u32SourceClock;
    uint32_t u32OSR;
    uint32_t u32MDiv;

    // Get DMIC clock source.
    switch (CLK->DMICSEL & CLK_DMICSEL_DMIC0SEL_Msk)
    {
        case CLK_DMICSEL_DMIC0SEL_HXT:
            u32SourceClock = CLK_GetHXTFreq();
            break;

        case CLK_DMICSEL_DMIC0SEL_APLL1_DIV2:
            u32SourceClock = (CLK_GetAPLL1ClockFreq() / 2UL);
            break;

        case CLK_DMICSEL_DMIC0SEL_MIRC:
            u32SourceClock = CLK_GetMIRCFreq();
            break;

        case CLK_DMICSEL_DMIC0SEL_HIRC:
            u32SourceClock = __HIRC;
            break;

        case CLK_DMICSEL_DMIC0SEL_HIRC48M:
            u32SourceClock = __HIRC48M;
            break;

        case CLK_DMICSEL_DMIC0SEL_PCLK4:
            u32SourceClock = CLK_GetPCLK4Freq();
            break;

        default:
            return 0UL;
    }

    switch (dmic->DIV & DMIC_DIV_OSR_Msk)
    {
        case DMIC_DOWNSAMPLE_50:
        case DMIC_DOWNSAMPLE_100:
        case DMIC_DOWNSAMPLE_64:
        case DMIC_DOWNSAMPLE_128:
        case DMIC_DOWNSAMPLE_256:
            u32OSR = au16OSRTable[(dmic->DIV & DMIC_DIV_OSR_Msk) >> DMIC_DIV_OSR_Pos];
            break;

        default:
            u32OSR = 64UL;
            break;
    }

    u32MDiv = (((dmic->DIV & DMIC_DIV_DMCLKDIV_Msk) >> DMIC_DIV_DMCLKDIV_Pos) + 1UL);
    return ((u32SourceClock / (CLK->DMICDIV + 1UL)) / u32MDiv) / u32OSR;
}

/**
  * @brief      Set the sample Rate of data
  * @param[in]  dmic The base address of DMIC module
  * @param      u32SampleRate is sample Rate of data.
  * @return     Real sample rate. 0 is DMIC clock source error.
  * @details    This API maybe modify OSR setting for sample Rate
  */
uint32_t DMIC_SetSampleRate(DMIC_T *dmic, uint32_t u32SampleRate)
{
    uint16_t const au16OSRTable[] = {64, 128, 256, 100, 64, 64, 64, 50};
    uint32_t u32SourceClock;
    uint32_t u32BusClock;
    uint32_t u32MainClock;
    uint32_t u32OSR;
    uint32_t u32MDiv;
    uint32_t u32SDiv = 1UL;

    // Get DMIC clock source.
    switch (CLK->DMICSEL & CLK_DMICSEL_DMIC0SEL_Msk)
    {
        case CLK_DMICSEL_DMIC0SEL_HXT:
            u32SourceClock = CLK_GetHXTFreq();
            break;

        case CLK_DMICSEL_DMIC0SEL_APLL1_DIV2:
            u32SourceClock = (CLK_GetAPLL1ClockFreq() / 2UL);
            break;

        case CLK_DMICSEL_DMIC0SEL_MIRC:
            u32SourceClock = CLK_GetMIRCFreq();
            break;

        case CLK_DMICSEL_DMIC0SEL_HIRC:
            u32SourceClock = __HIRC;
            break;

        case CLK_DMICSEL_DMIC0SEL_HIRC48M:
            u32SourceClock = __HIRC48M;
            break;

        case CLK_DMICSEL_DMIC0SEL_PCLK4:
            u32SourceClock = CLK_GetPCLK4Freq();
            break;

        default:
            return 0UL;
    }

    // Get OSR config and cal BusClock.
    // F_DMIC_MCLK = Fs * K
    // F_DMIC_CLK = Fs * OSR
    // F_DMIC_CLK = (F_DMIC_MCLK)/(1 + MCLKDIV)
    // => Fs * OSR = (Fs * K)/(1 + MCLKDIV)
    // => K = OSR * (1 + MCLKDIV)
    // K should be divisible by OSR
    switch (dmic->DIV & DMIC_DIV_OSR_Msk)
    {
        case DMIC_DOWNSAMPLE_50:
        case DMIC_DOWNSAMPLE_100:
        case DMIC_DOWNSAMPLE_64:
        case DMIC_DOWNSAMPLE_128:
        case DMIC_DOWNSAMPLE_256:
            u32OSR = au16OSRTable[(dmic->DIV & DMIC_DIV_OSR_Msk) >> DMIC_DIV_OSR_Pos];
            break;

        //        case DMIC_DOWNSAMPLE_50:
        //        case DMIC_DOWNSAMPLE_100:
        //                    u32OSR = (u32SampleRate>=32500)?50:100;
        //                  break;
        default:
            u32OSR = 64UL;
            break;
    }

    // Cal BusClock.
    u32BusClock = u32SampleRate * u32OSR;

    if (u32BusClock > u32SourceClock)
    {
        if ((u32SampleRate * 50UL) > u32SourceClock)
        {
            dmic->CTL = 0UL;
            dmic->DIV = 0UL;
            return 0UL;
        }
        else
        {
            DMIC_SET_DOWNSAMPLE(DMIC0, DMIC_DOWNSAMPLE_50);
            u32OSR = 50UL;
        }

        u32BusClock = u32SampleRate * u32OSR;
    }

    u32MDiv = u32SourceClock / u32BusClock;

    if (u32MDiv > 64UL)
    {
        u32SDiv = (u32MDiv / 64UL) + 1UL;
        u32MDiv = u32SourceClock / u32SDiv / u32BusClock;
    }

    if (((u32SourceClock / u32SDiv) % u32BusClock) == 0UL)
    {
        u32MainClock = (u32SourceClock / u32SDiv);
    }
    else
    {
        u32MainClock = u32BusClock * u32MDiv;
    }

    CLK->DMICDIV = (u32SourceClock / u32MainClock) - 1UL;
    u32SDiv = (CLK->DMICDIV + 1UL);

    if (((u32SourceClock / u32SDiv) / u32BusClock) != 0UL)
    {
        dmic->DIV = (dmic->DIV & ~(DMIC_DIV_DMCLKDIV_Msk)) | ((((u32SourceClock / u32SDiv) / u32BusClock) - 1UL) << DMIC_DIV_DMCLKDIV_Pos);
    }
    else
    {
        dmic->DIV = (dmic->DIV & ~(DMIC_DIV_DMCLKDIV_Msk)) | (0UL << DMIC_DIV_DMCLKDIV_Pos);
    }

    return DMIC_GetSampleRate(dmic);
}


/**
  * @brief      Get the detect voice's sample Rate
  * @param[in]  vad The base address of VAD module
  *
  * @return     Real detect sample rate. 0 is VAD clock source error.
  */
uint32_t DMIC_VAD_GetSampleRate(const VAD_T *vad)
{
    uint16_t const au16OSRTable[3] = {48, 64, 96};
    uint32_t u32SourceClock;
    uint32_t u32OSR;
    uint32_t u32SincOsrSel;

    // Get VAD clock source.
    switch (CLK->DMICSEL & CLK_DMICSEL_VAD0SEL_Msk)
    {
        case CLK_DMICSEL_VAD0SEL_PCLK4:
            u32SourceClock = CLK_GetPCLK4Freq();
            break;

        case CLK_DMICSEL_VAD0SEL_MIRC:
            u32SourceClock = CLK_GetMIRCFreq();
            break;

        case CLK_DMICSEL_VAD0SEL_HIRC:
            u32SourceClock = __HIRC;
            break;

        default:
            return 0UL;
    }

    // Get OSR config and cal BusClock.
    u32SincOsrSel = (vad->SINCCTL & VAD_SINCCTL_SINCOSR_Msk) >> VAD_SINCCTL_SINCOSR_Pos;

    if (u32SincOsrSel >= 3UL)
    {
        u32OSR = 48UL;
    }
    else
    {
        u32OSR = au16OSRTable[u32SincOsrSel];
    }

    return ((u32SourceClock / (((vad->SINCCTL & VAD_SINCCTL_VADMCLKDIV_Msk) >> VAD_SINCCTL_VADMCLKDIV_Pos) + 1UL)) / 4UL / u32OSR);
}

/**
  * @brief      Set the detect voice's sample Rate
  * @param[in]  vad The base address of VAD module
  * @param[in]  u32SampleRate Sample Rate of input voice data.
  * @return     Real detect sample rate. 0 is VAD clock source error.
  */
uint32_t DMIC_VAD_SetSampleRate(VAD_T *vad, uint32_t u32SampleRate)
{
    uint16_t const au16OSRTable[3] = {48, 64, 96};
    uint32_t u32SourceClock;
    uint32_t u32BusClock;
    uint32_t u32MainClock;
    uint32_t u32OSR;
    uint32_t u32SincOsrSel;

    // Get VAD clock source.
    switch (CLK->DMICSEL & CLK_DMICSEL_VAD0SEL_Msk)
    {
        case CLK_DMICSEL_VAD0SEL_PCLK4:
            u32SourceClock = CLK_GetPCLK4Freq();
            break;

        case CLK_DMICSEL_VAD0SEL_MIRC:
            u32SourceClock = CLK_GetMIRCFreq();
            break;

        case CLK_DMICSEL_VAD0SEL_HIRC:
            u32SourceClock = __HIRC;
            break;

        default:
            return 0UL;
    }

    // Get OSR config and cal BusClock.
    u32SincOsrSel = (vad->SINCCTL & VAD_SINCCTL_SINCOSR_Msk) >> VAD_SINCCTL_SINCOSR_Pos;

    if (u32SincOsrSel >= 3UL)
    {
        u32OSR = 48UL;
    }
    else
    {
        u32OSR = au16OSRTable[u32SincOsrSel];
    }

    // Cal BusClock.
    u32BusClock = u32SampleRate * u32OSR;
    // Cal main working clock
    u32MainClock = u32BusClock * 4UL;

    if ((u32SourceClock / u32MainClock) != 0UL)
    {
        vad->SINCCTL = (vad->SINCCTL & ~(VAD_SINCCTL_VADMCLKDIV_Msk)) | (((u32SourceClock / u32MainClock) - 1UL) << VAD_SINCCTL_VADMCLKDIV_Pos);
    }
    else
    {
        vad->SINCCTL = (vad->SINCCTL & ~(VAD_SINCCTL_VADMCLKDIV_Msk)) | (0UL << VAD_SINCCTL_VADMCLKDIV_Pos);
    }

    return DMIC_VAD_GetSampleRate(vad);
}

/** @} end of group DMIC_EXPORTED_FUNCTIONS */

/** @} end of group DMIC_Driver */

/** @} end of group Standard_Driver */
