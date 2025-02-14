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
    uint32_t u32Delay, u32RamIdx;
    uint32_t u32IsRegLocked;
    u32IsRegLocked = SYS_IsRegLocked();

    if (u32IsRegLocked)
    {
        SYS_UnlockReg();
    }

    PMC_SetDMIC_SRAMPowerMode(PMC_SRAM_NORMAL);

    if (u32IsRegLocked)
    {
        SYS_LockReg();
    }

    dmic->DIV |= DMIC_DIV_FCLR_Msk;
    u32Delay = SystemCoreClock >> 3;

    while (((dmic->DIV & DMIC_DIV_FCLR_Msk) == DMIC_DIV_FCLR_Msk) && (--u32Delay))
    {
        __NOP();
    }

    dmic->CTL |= DMIC_CTL_SWRST_Msk;
    dmic->CTL |= (DMIC_CTL_DSPMEMT_Msk);

    for (u32RamIdx = 0 ; u32RamIdx < 128 ; u32RamIdx++)
    {
        if (u32RamIdx == DMIC_RAM_LGAIN_ADDR || u32RamIdx == DMIC_RAM_RGAIN_ADDR)
        {
            //Set gain volume 0db((-128-gain)*4096=F80000)
            outp32(DMIC_DSP0_RAMDATA, 0xF80000);
            outp32(DMIC_DSP1_RAMDATA, 0xF80000);
        }
        else if (u32RamIdx == DMIC_RAM_LINITSAMPLE_ADDR || u32RamIdx == DMIC_RAM_RINITSAMPLE_ADDR)
        {
            //Set initial sample = 1024(0x400)
            outp32(DMIC_DSP0_RAMDATA, 0x400);
            outp32(DMIC_DSP1_RAMDATA, 0x400);
        }
        else
        {
            outp32(DMIC_DSP0_RAMDATA, 0x0);
            outp32(DMIC_DSP1_RAMDATA, 0x0);
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
    int16_t i16TmpChVolume = -128 - (i16ChVolume);
    int16_t i16SetChVolume;

    if (i16TmpChVolume > 0)
        i16SetChVolume = i16TmpChVolume << 7;
    else
        i16SetChVolume = ((i16TmpChVolume << 7) | BIT15);

    if (u32ChMsk & DMIC_CTL_CHEN0_Msk)
    {
        (dmic)->GAINCTL0 = (dmic->GAINCTL0 & ~(DMIC_GAINCTL0_CHyyLVOL_Msk)) | (i16SetChVolume & DMIC_GAINCTL0_CHyyLVOL_Msk);
    }

    if (u32ChMsk & DMIC_CTL_CHEN1_Msk)
    {
        (dmic)->GAINCTL0 = (dmic->GAINCTL0 & ~(DMIC_GAINCTL0_CHxxRVOL_Msk)) | (i16SetChVolume << DMIC_GAINCTL0_CHxxRVOL_Pos);
    }

    if (u32ChMsk & DMIC_CTL_CHEN2_Msk)
    {
        (dmic)->GAINCTL1 = (dmic->GAINCTL1 & ~(DMIC_GAINCTL1_CHyyLVOL_Msk)) | (i16SetChVolume & DMIC_GAINCTL1_CHyyLVOL_Msk);
    }

    if (u32ChMsk & DMIC_CTL_CHEN3_Msk)
    {
        (dmic)->GAINCTL1 = (dmic->GAINCTL1 & ~(DMIC_GAINCTL1_CHxxRVOL_Msk)) | (i16SetChVolume << DMIC_GAINCTL1_CHxxRVOL_Pos);
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

    while ((!DMIC_IS_FIFOEMPTY(dmic)) && (--u32Delay))
    {
        __NOP();
    }
}

/**
  * @brief      Get the sample Rate of DMIC
  * @param[in]  dmic The base address of DMIC module
  *
  * @return     Real sample rate. 0 is DMIC clock source error.
  */
uint32_t DMIC_GetSampleRate(DMIC_T *dmic)
{
    uint16_t const au16OSRTable[] = {64, 128, 256, 100, 64, 64, 64, 50};
    uint32_t u32SourceClock, u32OSR, u32MDiv;

    // Get DMIC clock source.
    switch (CLK->DMICSEL & CLK_DMICSEL_DMIC0SEL_Msk)
    {
        case CLK_DMICSEL_DMIC0SEL_HXT:
            u32SourceClock = CLK_GetHXTFreq();
            break;

        case CLK_DMICSEL_DMIC0SEL_APLL1_DIV2:
            u32SourceClock = (CLK_GetAPLL1ClockFreq() / 2);
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
            return 0;
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
            u32OSR = 64;
            break;
    }

    u32MDiv = (((dmic->DIV & DMIC_DIV_DMCLKDIV_Msk) >> DMIC_DIV_DMCLKDIV_Pos) + 1);
    return ((u32SourceClock / (CLK->DMICDIV + 1)) / u32MDiv) / u32OSR;
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
    uint32_t u32SourceClock, u32BusClock, u32MainClock, u32OSR, u32MDiv, u32SDiv = 1;

    // Get DMIC clock source.
    switch (CLK->DMICSEL & CLK_DMICSEL_DMIC0SEL_Msk)
    {
        case CLK_DMICSEL_DMIC0SEL_HXT:
            u32SourceClock = CLK_GetHXTFreq();
            break;

        case CLK_DMICSEL_DMIC0SEL_APLL1_DIV2:
            u32SourceClock = (CLK_GetAPLL1ClockFreq() / 2);
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
            return 0;
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
            u32OSR = 64;
            break;
    }

    // Cal BusClock.
    u32BusClock = u32SampleRate * u32OSR;

    if (u32BusClock > u32SourceClock)
    {
        if (u32SampleRate * 50 > u32SourceClock)
        {
            dmic->CTL = 0;
            dmic->DIV = 0;
            return 0;
        }
        else
        {
            DMIC_SET_DOWNSAMPLE(DMIC0, DMIC_DOWNSAMPLE_50);
            u32OSR = 50;
            printf("DMIC change u32OSR %d!\n", u32OSR);
        }

        u32BusClock = u32SampleRate * u32OSR;
    }

    u32MDiv = u32SourceClock / u32BusClock;

    if (u32MDiv > 64)
    {
        u32SDiv = (u32MDiv / 64) + 1;
        u32MDiv = u32SourceClock / u32SDiv / u32BusClock;
    }

    if ((u32SourceClock / u32SDiv) % u32BusClock == 0)
    {
        u32MainClock = (u32SourceClock / u32SDiv);
    }
    else
    {
        u32MainClock = u32BusClock * u32MDiv;
    }

    CLK->DMICDIV = (u32SourceClock / u32MainClock - 1);
    u32SDiv = (CLK->DMICDIV + 1);

    if (u32SourceClock / u32SDiv / u32BusClock)
        dmic->DIV = (dmic->DIV & ~(DMIC_DIV_DMCLKDIV_Msk)) | ((u32SourceClock / u32SDiv / u32BusClock - 1) << DMIC_DIV_DMCLKDIV_Pos);
    else
        dmic->DIV = (dmic->DIV & ~(DMIC_DIV_DMCLKDIV_Msk)) | (0UL << DMIC_DIV_DMCLKDIV_Pos);

    return DMIC_GetSampleRate(dmic);
}


/**
  * @brief      Get the detect voice's sample Rate
  * @param[in]  vad The base address of VAD module
  *
  * @return     Real detect sample rate. 0 is VAD clock source error.
  */
uint32_t DMIC_VAD_GetSampleRate(VAD_T *vad)
{
    uint16_t const au16OSRTable[3] = {48, 64, 96};
    uint32_t u32SourceClock, u32OSR;

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
            return 0;
    }

    // Get OSR config and cal BusClock.
    u32OSR = (((vad->SINCCTL & VAD_SINCCTL_SINCOSR_Msk) >> VAD_SINCCTL_SINCOSR_Pos) >= 3) ? 48 : (au16OSRTable[(vad->SINCCTL & VAD_SINCCTL_SINCOSR_Msk) >> VAD_SINCCTL_SINCOSR_Pos]);
    return ((u32SourceClock / (((vad->SINCCTL & VAD_SINCCTL_VADMCLKDIV_Msk) >> VAD_SINCCTL_VADMCLKDIV_Pos) + 1)) / 4 / u32OSR);
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
    uint32_t u32SourceClock, u32BusClock, u32MainClock, u32OSR;

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
            return 0;
    }

    // Get OSR config and cal BusClock.
    u32OSR = (((vad->SINCCTL & VAD_SINCCTL_SINCOSR_Msk) >> VAD_SINCCTL_SINCOSR_Pos) >= 3) ? 48 : (au16OSRTable[(vad->SINCCTL & VAD_SINCCTL_SINCOSR_Msk) >> VAD_SINCCTL_SINCOSR_Pos]);
    // Cal BusClock.
    u32BusClock = u32SampleRate * u32OSR;
    // Cal main working clock
    u32MainClock = u32BusClock * 4;

    if (u32SourceClock / u32MainClock)
        vad->SINCCTL = (vad->SINCCTL & ~(VAD_SINCCTL_VADMCLKDIV_Msk)) | ((u32SourceClock / u32MainClock - 1) << VAD_SINCCTL_VADMCLKDIV_Pos);
    else
        vad->SINCCTL = (vad->SINCCTL & ~(VAD_SINCCTL_VADMCLKDIV_Msk)) | (0UL << VAD_SINCCTL_VADMCLKDIV_Pos);

    return DMIC_VAD_GetSampleRate(vad);
}

/** @} end of group DMIC_EXPORTED_FUNCTIONS */

/** @} end of group DMIC_Driver */

/** @} end of group Standard_Driver */
