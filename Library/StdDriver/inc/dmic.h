/******************************************************************************
 * @file     DMIC.h
 * @version  V1.00
 * @brief    DMIC driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __DMIC_H__
#define __DMIC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup DMIC_Driver DMIC Driver
  @{
*/

/** @addtogroup DMIC_EXPORTED_CONSTANTS DMIC Exported Constants
  @{
*/

/*-----------------------------------------------------------------
  | APLL1                     | Sample-Rate Hz      | Down-Sample |
  |---------------------------|---------------------|-------------|
  | FREQ_192MHZ               | 8000/16000/48000 Hz | 50/100      |
  |---------------------------|---------------------|-------------|
  | DMIC_APLL1_FREQ_196608KHZ | 8000/16000/48000 Hz | 64/128/256  |
  |---------------------------|---------------------|-------------|
  | DMIC_APLL1_FREQ_194040KHZ | 11025/22050/44100 Hz| 50/100      |
  |---------------------------|---------------------|-------------|
  | DMIC_APLL1_FREQ_180634KHZ | 11025/22050/44100 Hz| 64/128/256  |
  ---------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/* DMIC clock source APLL1 Frequency Definitions                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define DMIC_APLL1_FREQ_196608KHZ  196608000UL  /*!< 196608 KHz for DMIC 8000/16000/48000 Hz sample-rate with 64/128/256 down-sample \hideinitializer */
#define DMIC_APLL1_FREQ_194040KHZ  194040000UL  /*!< 194040 KHz for DMIC 11025/22050/44100 Hz sample-rate with 50/100 down-sample \hideinitializer */
#define DMIC_APLL1_FREQ_180634KHZ  180634000UL  /*!< 180634 KHz for DMIC 11025/22050/44100 Hz sample-rate with 64/128/256 down-sample \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* DMIC CTL Constant Definitions                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define DMIC_FIFOWIDTH_24         (DMIC_CTL_FIFOWIDTH_Msk)
#define DMIC_FIFOWIDTH_16         (0UL<<DMIC_CTL_FIFOWIDTH_Pos)

#define DMIC_GAINSTEP_1_2         (0UL<<DMIC_CTL_GAINSTEP_Pos)
#define DMIC_GAINSTEP_1_4         (1UL<<DMIC_CTL_GAINSTEP_Pos)
#define DMIC_GAINSTEP_1_8         (2UL<<DMIC_CTL_GAINSTEP_Pos)
#define DMIC_GAINSTEP_1_16        (3UL<<DMIC_CTL_GAINSTEP_Pos)

#define DMIC_DOWNSAMPLE_64        (0x0UL<<DMIC_DIV_OSR_Pos)          /*!< DMIC Down Sample Rate 32 */
#define DMIC_DOWNSAMPLE_128       (0x1UL<<DMIC_DIV_OSR_Pos)          /*!< DMIC Down Sample Rate 64 */
#define DMIC_DOWNSAMPLE_256       (0x2UL<<DMIC_DIV_OSR_Pos)          /*!< DMIC Down Sample Rate 128 */
#define DMIC_DOWNSAMPLE_100       (0x3UL<<DMIC_DIV_OSR_Pos)          /*!< DMIC Down Sample Rate 50 */
#define DMIC_DOWNSAMPLE_50        (0x7UL<<DMIC_DIV_OSR_Pos)          /*!< DMIC Down Sample Rate 100 */

#define DMIC_LATCHDATA_CH01FR     (0x0UL<<DMIC_CTL_LCHEDGE01_Pos)    /*!< DMIC Data Latch Channel0 Falling Edge, Channel1 Rising Edge*/
#define DMIC_LATCHDATA_CH01RF     (0x1UL<<DMIC_CTL_LCHEDGE01_Pos)    /*!< DMIC Data Latch Channel0 Rising Edge Channel1 Falling Edge*/
#define DMIC_LATCHDATA_CH23FR     (0x0UL<<DMIC_CTL_LCHEDGE23_Pos)    /*!< DMIC Data Latch Channel2 Falling Edge Channel3 Rising Edge*/
#define DMIC_LATCHDATA_CH23RF     (0x1UL<<DMIC_CTL_LCHEDGE23_Pos)    /*!< DMIC Data Latch Channel2 Rising Edge Channel3 Falling Edge*/

#define DMIC_LATCHDATA_CH01F      (DMIC_LATCHDATA_CH01FR)    /*!< DMIC Data Latch Channel0 Falling Edge, Channel1 Rising Edge */
#define DMIC_LATCHDATA_CH01R      (DMIC_LATCHDATA_CH01RF)    /*!< DMIC Data Latch Channel0 Rising Edge Channel1 Falling Edge  */
#define DMIC_LATCHDATA_CH23F      (DMIC_LATCHDATA_CH23FR)    /*!< DMIC Data Latch Channel2 Falling Edge Channel3 Rising Edge */
#define DMIC_LATCHDATA_CH23R      (DMIC_LATCHDATA_CH23RF)    /*!< DMIC Data Latch Channel2 Rising Edge Channel3 Falling Edge  */

#define DMIC_DIV_HPF_CUT_F        (0x70000000UL)             /*!< DMIC High pass filter -3B cut-off frequency Setting 7 = 1.029%. */
#define DMIC_CTL_DSPMEMT_Pos      (26)                               /*!< DMIC Enable the MCU accessing of DSP RAM Position */
#define DMIC_CTL_DSPMEMT_Msk      (0x1ul << DMIC_CTL_DSPMEMT_Pos)    /*!< DMIC Enable the MCU accessing of DSP RAM Mask */
#define DMIC_DSP0_RAMDATA         (DMIC0_BASE+0x40)          /*!< DMIC DSP0 RAM Test Data Register. */
#define DMIC_DSP1_RAMDATA         (DMIC0_BASE+0x44)          /*!< DMIC DSP1 RAM Test Data Register. */
#define DMIC_RAM_LGAIN_ADDR       (58)                       /*!< DMIC RAM LGAIN_ADDR. */
#define DMIC_RAM_RGAIN_ADDR       (122)                      /*!< DMIC RAM RGAIN_ADDR. */
#define DMIC_RAM_LINITSAMPLE_ADDR (62)                       /*!< DMIC RAM LINITSAMPLE_ADDR. */
#define DMIC_RAM_RINITSAMPLE_ADDR (126)                      /*!< DMIC RAM RINITSAMPLE_ADDR. */
/*---------------------------------------------------------------------------------------------------------*/
/* VAD SINCCTL Constant Definitions                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define DMIC_VAD_DOWNSAMPLE_48    (0x0UL<<VAD_SINCCTL_SINCOSR_Pos)          /*!< VAD SINC Filter Down Sample Rate 48 */
#define DMIC_VAD_DOWNSAMPLE_64    (0x1UL<<VAD_SINCCTL_SINCOSR_Pos)          /*!< VAD SINC Filter Down Sample Rate 64 */
#define DMIC_VAD_DOWNSAMPLE_96    (0x2UL<<VAD_SINCCTL_SINCOSR_Pos)          /*!< VAD SINC Filter Down Sample Rate 96 */

/*---------------------------------------------------------------------------------------------------------*/
/* VAD CTL Constant Definitions                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define DMIC_VAD_STAT_16MS            (0x99UL<<VAD_CTL0_STAT_Pos)                /*!< VAD Short Term Attack Time 16 ms */
#define DMIC_VAD_STAT_8MS             (0xaaUL<<VAD_CTL0_STAT_Pos)                /*!< VAD Short Term Attack Time 8 ms */
#define DMIC_VAD_STAT_4MS             (0xbbUL<<VAD_CTL0_STAT_Pos)                /*!< VAD Short Term Attack Time 4 ms */
#define DMIC_VAD_STAT_2MS             (0xccUL<<VAD_CTL0_STAT_Pos)                /*!< VAD Short Term Attack Time 2 ms */

#define DMIC_VAD_LTAT_512MS           (0x4UL<<VAD_CTL0_LTAT_Pos)                 /*!< VAD Long Term Attack Time 512 ms */
#define DMIC_VAD_LTAT_256MS           (0x5UL<<VAD_CTL0_LTAT_Pos)                 /*!< VAD Long Term Attack Time 256 ms */
#define DMIC_VAD_LTAT_128MS           (0x6UL<<VAD_CTL0_LTAT_Pos)                 /*!< VAD Long Term Attack Time 128 ms */
#define DMIC_VAD_LTAT_64MS            (0x7UL<<VAD_CTL0_LTAT_Pos)                 /*!< VAD Long Term Attack Time 64 ms */

/*---------------------------------------------------------------------------------------------------------*/
/* VAD Power Threshold Definitions                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define DMIC_VAD_POWERTHRE_M90DB            (0x0001UL)                                 /*!< VAD power threshold -90DB      */
#define DMIC_VAD_POWERTHRE_M80DB            (0x0003UL)                                 /*!< VAD power threshold -80DB      */
#define DMIC_VAD_POWERTHRE_M70DB            (0x000AUL)                                 /*!< VAD power threshold -70DB      */
#define DMIC_VAD_POWERTHRE_M60DB            (0x0020UL)                                 /*!< VAD power threshold -60DB      */
#define DMIC_VAD_POWERTHRE_M50DB            (0x0067UL)                                 /*!< VAD power threshold -50DB      */
#define DMIC_VAD_POWERTHRE_M40DB            (0x0147UL)                                 /*!< VAD power threshold -40DB      */
#define DMIC_VAD_POWERTHRE_M30DB            (0x040CUL)                                 /*!< VAD power threshold -30DB      */
#define DMIC_VAD_POWERTHRE_M20DB            (0x0CCCUL)                                 /*!< VAD power threshold -20DB      */
#define DMIC_VAD_POWERTHRE_M10DB            (0x2879UL)                                 /*!< VAD power threshold -10DB      */
#define DMIC_VAD_POWERTHRE_0DB              (0x7FFFUL)                                 /*!< VAD power threshold   0DB      */

/** @} end of group DMIC_EXPORTED_CONSTANTS */


/* VAD Biquad Filter Coefficient Struct */
typedef struct
{
    uint16_t            u16BIQCoeffA1;         /*!< 0xc715,Biquad Filter Coefficient a1, in 3 intergers + 13 fractional bits */
    uint16_t            u16BIQCoeffA2;         /*!< 0x19a0,Biquad Filter Coefficient a2, in 3 intergers + 13 fractional bits */
    uint16_t            u16BIQCoeffB0;         /*!< 0x1ca3,Biquad Filter Coefficient b0, in 3 intergers + 13 fractional bits */
    uint16_t            u16BIQCoeffB1;         /*!< 0xc6bb,Biquad Filter Coefficient b1, in 3 intergers + 13 fractional bits */
    uint16_t            u16BIQCoeffB2;         /*!< 0x1ca3,Biquad Filter Coefficient b2, in 3 intergers + 13 fractional bits */
} DMIC_VAD_BIQ_T;

/** @addtogroup DMIC_EXPORTED_FUNCTIONS DMIC Exported Functions
  @{
*/

/**
  * @brief      Set DMIC channel 01 data latch edge.
  * @param[in]  dmic The base address of DMIC module
  * @param[in]  u32Value Config channel 01 edge state(falliing or rising)
  *             - \ref DMIC_LATCHDATA_CH01FR
  *             - \ref DMIC_LATCHDATA_CH01RF
  * @return     None
  * @details    Channel 01 latched on rising or falling edge of DMIC_CLK.
  */
#define DMIC_SET_LATCHEDGE_CH01(dmic,u32Value)             ((dmic)->CTL = ((dmic)->CTL & ~(DMIC_CTL_LCHEDGE01_Msk))|(u32Value))

/**
  * @brief      Set DMIC channel 23 data latch edge.
  * @param[in]  dmic The base address of DMIC module
  * @param[in]  u32Value Config channel 23 edge state(falliing or rising)
  *             - \ref DMIC_LATCHDATA_CH23FR
  *             - \ref DMIC_LATCHDATA_CH23RF
  * @return     None
  * @details    Channel 23 latched on rising or falling edge of DMIC_CLK.
  */
#define DMIC_SET_LATCHEDGE_CH23(dmic,u32Value)             ((dmic)->CTL = ((dmic)->CTL & ~(DMIC_CTL_LCHEDGE23_Msk))|(u32Value))

/**
  * @brief      Enable DMIC FIFO threshold interrupt.
  * @param[in]  dmic The base address of DMIC module
  * @param[in]  u8Value: 1~31 FIFO buffer threshold count
  * @return     None
  * @details    DMIC FIFO threshold interrupt Enabled.
  */
#define DMIC_ENABLE_FIFOTH_INT(dmic,u8Value)    ((dmic)->DIV = (((dmic)->DIV&~DMIC_DIV_DMTH_Msk)|((((uint32_t)u8Value)<<DMIC_DIV_DMTH_Pos)&DMIC_DIV_DMTH_Msk))|DMIC_DIV_DMTHIE_Msk)

/**
  * @brief      Disable DMIC FIFO threshold interrupt.
  * @param[in]  dmic The base address of DMIC module
  * @return     None
  * @details    DMIC FIFO threshold interrupt Disabled.
  */
#define DMIC_DISABLE_FIFOTH_INT(dmic)           ((dmic)->DIV &= (~DMIC_DIV_DMTHIE_Msk))

/**
  * @brief      Enable DMIC down sample rate.
  * @param[in]  dmic The base address of DMIC module
  * @param[in]  u32Value Down sample rate value.
  *             - \ref DMIC_DOWNSAMPLE_64
  *             - \ref DMIC_DOWNSAMPLE_128
  *             - \ref DMIC_DOWNSAMPLE_256
  *             - \ref DMIC_DOWNSAMPLE_100
  *             - \ref DMIC_DOWNSAMPLE_50
  * @return     None
  * @details    Enable DMIC down sample rate funciton and set down sample value.
  */
#define DMIC_SET_DOWNSAMPLE(dmic,u32Value)         ((dmic)->DIV = (((dmic)->DIV) & (~DMIC_DIV_OSR_Msk))|(u32Value))

/**
  * @brief      Enable DMIC LPPDMA function.
  * @param[in]  dmic The base address of DMIC module
  * @return     None.
  * @details    DMIC will request data to LPPDMA controller whenever there is space in FIFO.
  */
#define DMIC_ENABLE_LPPDMA(dmic)                        ((dmic)->LPPDMACTL |= DMIC_LPPDMACTL_LPPDMAEN_Msk)

/**
  * @brief      Disable DMIC LPPDMA function.
  * @param[in]  dmic The base address of DMIC module
  * @return     None.
  */
#define DMIC_DISABLE_LPPDMA(dmic)                       ((dmic)->LPPDMACTL &= (~DMIC_LPPDMACTL_LPPDMAEN_Msk))

/**
  * @brief      Check DMIC FIFO empty or not
  * @param[in]  dmic The base address of DMIC module
  * @return     0 = FIFO is not empty
  *             1 = FIFO is empty
  */
#define DMIC_IS_FIFOEMPTY(dmic)                       (((dmic)->STATUS) & DMIC_STATUS_EMPTY_Msk)

/**
  * @brief      Check DPWM FIFO full or not
  * @param[in]  dmic The base address of DMIC module
  * @return     0 = FIFO is not full
  *             1 = FIFO is full
  */
#define DMIC_IS_FIFOFULL(dmic)                        (((dmic)->STATUS) & DMIC_STATUS_FULL_Msk)

/**
  * @brief      Check DPWM FIFO  is more than or equal to the TH
  * @param[in]  dmic The base address of DMIC module
  * @return     0 = FIFO is not less than TH
  *             1 = FIFO is more than or equal to the TH
  */
#define DMIC_IS_FIFODMTH(dmic)                        (((dmic)->STATUS) & DMIC_STATUS_DMTHIF_Msk)

/**
  * @brief      Read DMIC FIFO Audio Data Input.
  * @param[in]  dmic The base address of DMIC module
  * @return     None.
  * @details    A read function to this register pop data from the DMIC FIFO.
  */
#define DMIC_READ_DATA(dmic)                          ((dmic)->FIFO)

/**
  * @brief      Enable VAD down sample rate.
  * @param[in]  vad The base address of VAD module
  * @param[in]  u32Value Down sample rate value.
  *             - \ref DMIC_VAD_DOWNSAMPLE_48
  *             - \ref DMIC_VAD_DOWNSAMPLE_64
  *             - \ref DMIC_VAD_DOWNSAMPLE_96
  * @return     None
  * @details    Enable VAD down sample rate funciton and set down sample value.
  */
#define DMIC_VAD_SET_DOWNSAMPLE(vad,u32Value)         ((vad)->SINCCTL = ((vad)->SINCCTL & ~VAD_SINCCTL_SINCOSR_Msk)|(u32Value))

/**
  * @brief      Enable VAD function.
  * @param[in]  vad The base address of VAD module
  * @return     None
  * @details    Start to detect voice from DMIC0
  */
#define DMIC_VAD_ENABLE(vad)                             ((vad)->SINCCTL |= VAD_SINCCTL_VADEN_Msk)

/**
  * @brief      Disable VAD function.
  * @param[in]  vad The base address of VAD module
  * @return     None
  * @details    Stop to detect voice.
  */
#define DMIC_VAD_DISABLE(vad)                            ((vad)->SINCCTL &= ~VAD_SINCCTL_VADEN_Msk)

/**
  * @brief      Set Short Term Attack Time
  * @param[in]  vad The base address of VAD module
  * @param[in]  u32Time Short Term Attack Time.
  *             - \ref DMIC_VAD_STAT_16MS
  *             - \ref DMIC_VAD_STAT_8MS
  *             - \ref DMIC_VAD_STAT_4MS
  *             - \ref DMIC_VAD_STAT_2MS
  * @return     None
  */
#define DMIC_VAD_SET_STAT(vad,u32Time)                   ((vad)->CTL0 = ((vad)->CTL0 & ~VAD_CTL0_STAT_Msk)|(u32Time))

/**
  * @brief      Set Long Term Attack Time
  * @param[in]  vad The base address of VAD module
  * @param[in]  u32Time Long Term Attack Time.
  *             - \ref DMIC_VAD_LTAT_512MS
  *             - \ref DMIC_VAD_LTAT_256MS
  *             - \ref DMIC_VAD_LTAT_128MS
  *             - \ref DMIC_VAD_LTAT_64MS
  * @return     None
  */
#define DMIC_VAD_SET_LTAT(vad,u32Time)                   ((vad)->CTL0 = ((vad)->CTL0 & ~VAD_CTL0_LTAT_Msk)|(u32Time))

/**
  * @brief      Set Short Term Power Threshold.
  * @param[in]  vad The base address of VAD module
  * @param[in]  u32Upper Short Term Upper Limit Threshold
  * @return     None
  */
#define DMIC_VAD_SET_STTHRE(vad,u32Upper)       ((vad)->CTL1 = ((vad)->CTL1&~VAD_CTL1_STTHREHWM_Msk)|((u32Upper)&VAD_CTL1_STTHREHWM_Msk))

/**
  * @brief      Get short term signal power value.
  * @param[in]  vad The base address of VAD module
  *
  * @return     short term signal power value
  * @details    When VADEN(VAD_SINCCTL =1), STP cannot be read by CPU
  */
#define DMIC_VAD_GET_STP(vad)                         ((vad)->STATUS0 & VAD_STATUS0_STP_Msk)

/**
  * @brief      Set Long Term Power Threshold.
  * @param[in]  vad The base address of VAD module
  * @param[in]  u32Value Long Term Power Threshold
  * @return     None
  */
#define DMIC_VAD_SET_LTTHRE(vad,u32Value)       ((vad)->CTL2 = ((vad)->CTL2&~VAD_CTL2_LTTHRE_Msk)|(((u32Value)<<VAD_CTL2_LTTHRE_Pos)&VAD_CTL2_LTTHRE_Msk))

/**
  * @brief      Get long term signal power value.
  * @param[in]  vad The base address of VAD module
  *
  * @return     long term signal power value
  * @details    When VADEN(VAD_SINCCTL =1), LTP cannot be read by CPU
  */
#define DMIC_VAD_GET_LTP(vad)                         (((vad)->STATUS1 & VAD_STATUS1_LTP_Msk)>>VAD_STATUS1_LTP_Pos)

/**
  * @brief      Set Deviation Threshold
  * @param[in]  vad The base address of VAD module
  * @param[in]  u32Value Deviation Threshold
  * @return     None
  */
#define DMIC_VAD_SET_DEVTHRE(vad,u32Value)                ((vad)->CTL3 = ((vad)->CTL3&~VAD_CTL3_DEVTHRE_Msk)|((u32Value)&VAD_CTL3_DEVTHRE_Msk))

/**
  * @brief      Get deviation of the Long Term and Short Term Power value
  * @param[in]  vad The base address of VAD module
  *
  * @return     deviation of the Long Term Signal Power and Short Term Signal Power value
  * @details    When VADEN(VAD_SINCCTL =1), DEV cannot be read by CPU
  */
#define DMIC_VAD_GET_DEV(vad)                         ((vad)->STATUS1 & VAD_STATUS1_DEV_Msk)

/**
  * @brief      Enable VAD's BIQ filter.
  * @param[in]  vad The base address of VAD module
  * @return     None.
  * @details    Before you enable this function, please make sure your BIQ coefficient correct.
  */
#define DMIC_VAD_ENABLE_BIQ(vad)                         ((vad)->BIQCTL2 |= VAD_BIQCTL2_BIQEN_Msk)

/**
  * @brief      Disable VAD's BIQ filter.
  * @param[in]  vad The base address of VAD module
  * @return     None.
  */
#define DMIC_VAD_DISABLE_BIQ(vad)                        ((vad)->BIQCTL2 &= ~VAD_BIQCTL2_BIQEN_Msk)

/**
  * @brief      Check the voice active event occurs.
  * @param[in]  vad The base address of VAD module
  * @return     0 = the voice active event not occurs.
  *             1 = the voice active event occurs.
  */
#define DMIC_VAD_IS_ACTIVE(vad)                        ((vad)->STATUS0 & VAD_STATUS0_ACTIVE_Msk)

/**
  * @brief      Clear VAD Active Flag.
  * @param[in]  vad The base address of VAD module
  * @return     None.
  * @details    Clear VAD Active Flag, ACTIVE, STP, LTP, DEV are cleared
  */
#define DMIC_VAD_CLR_ACTIVE(vad)                        ((vad)->SINCCTL |= VAD_SINCCTL_ACTCL_Msk)

/* Declare these inline functions here to avoid MISRA C 2004 rule 8.1 error */
__STATIC_INLINE void DMIC_SetFIFOWidth(DMIC_T *dmic, uint32_t u32Width);
__STATIC_INLINE void DMIC_SetGainStep(DMIC_T *dmic, uint32_t u32Volume);
__STATIC_INLINE void DMIC_ResetDSP(DMIC_T *dmic);
__STATIC_INLINE void DMIC_EnableMute(DMIC_T *dmic, uint32_t u32ChMute);
__STATIC_INLINE void DMIC_DisableMute(DMIC_T *dmic, uint32_t u32ChMute);
__STATIC_INLINE uint32_t DMIC_GetFIFOPTR(DMIC_T *dmic);
__STATIC_INLINE void DMIC_VAD_SetBIQCoeff(VAD_T *vad, DMIC_VAD_BIQ_T *psBIQCoeff);

/**
  * @brief      Set DMIC Data Effective Bit in FIFO.
  * @param[in]  dmic The base address of DMIC module
  * @param[in]  u32Width Config Data Effective Bit in FIFO
  *             - \ref DMIC_FIFOWIDTH_24
  *             - \ref DMIC_FIFOWIDTH_16
  * @return     None
  * @details    Set Data Effective 24/16 Bit in FIFO.
  */
__STATIC_INLINE void DMIC_SetFIFOWidth(DMIC_T *dmic, uint32_t u32Width)
{
    dmic->CTL = (dmic->CTL & ~(DMIC_CTL_FIFOWIDTH_Msk)) | (u32Width);
}

/**
  * @brief      Set DMIC Volume Control Gain Adjust Step.
  * @param[in]  dmic The base address of DMIC module
  * @param[in]  u32Volume  Config Volume Control Gain Adjust Step
  *             - \ref DMIC_GAINSTEP_1_2
  *             - \ref DMIC_GAINSTEP_1_4
  *             - \ref DMIC_GAINSTEP_1_8
  *             - \ref DMIC_GAINSTEP_1_16
  * @return     None
  * @details    Volume Control Gain Adjust Step is 1/2, 1/4, 1/8 or 1/16.
  */
__STATIC_INLINE void DMIC_SetGainStep(DMIC_T *dmic, uint32_t u32Volume)
{
    dmic->CTL = (dmic->CTL & ~(DMIC_CTL_GAINSTEP_Msk)) | (u32Volume);
}

/**
  * @brief      Enable DMIC Internal DSP State Software Reset.
  * @param[in]  dmic The base address of DMIC module
  * @return     None
  * @details    Enable DMIC Internal State Software Reset DMIC state machine, but all DMIC registers are kept unchanged.
  */
__STATIC_INLINE void DMIC_ResetDSP(DMIC_T *dmic)
{
    uint32_t u32Delay;
    dmic->CTL |= DMIC_CTL_SWRST_Msk;
    u32Delay = SystemCoreClock >> 3;

    while ((dmic->CTL & DMIC_CTL_SWRST_Msk) && (--u32Delay))
    {
        __NOP();
    }
}

/**
  * @brief      Enable DMIC's channel mute
  * @param[in]  dmic The base address of DMIC module
  * @param[in]  u32ChMute Enable channel Mute.
  *             - \ref DMIC_CTL_CH0MUTE_Msk
  *             - \ref DMIC_CTL_CH1MUTE_Msk
  *             - \ref DMIC_CTL_CH2MUTE_Msk
  *             - \ref DMIC_CTL_CH3MUTE_Msk
  * @return     None
  * @details    Enable channel to mute.
  */
__STATIC_INLINE void DMIC_EnableMute(DMIC_T *dmic, uint32_t u32ChMute)
{
    dmic->CTL |= (u32ChMute);
}

/**
  * @brief      Disable DMIC's channel mute
  * @param[in]  dmic The base address of DMIC module
  * @param[in]  u32ChMute Disable channel Mute.
  *             - \ref DMIC_CTL_CH0MUTE_Msk
  *             - \ref DMIC_CTL_CH1MUTE_Msk
  *             - \ref DMIC_CTL_CH2MUTE_Msk
  *             - \ref DMIC_CTL_CH3MUTE_Msk
  * @return     None
  * @details    Disable channel to mute.
  */
__STATIC_INLINE void DMIC_DisableMute(DMIC_T *dmic, uint32_t u32ChMute)
{
    dmic->CTL &= ~(u32ChMute);
}

/**
  * @brief      Get DMIC FIFO buffer Pointer
  *
  * @param[in]  dmic The base address of DMIC module.
  *
  * @retval     0~31   DMIC FIFO buffer Pointer
  *
  * @details    This function indicates the field that the valid data count within the DMIC FIFO buffer.
  */
__STATIC_INLINE uint32_t DMIC_GetFIFOPTR(DMIC_T *dmic)
{
    return ((dmic->STATUS & DMIC_STATUS_FIFOPTR_Msk) >> DMIC_STATUS_FIFOPTR_Pos);
}

/**
  * @brief      Write BIQ's coefficient value.
  * @param[in]  vad The base address of VAD module
  * @param[in]  psBIQCoeff: Biquad Filter Coefficient Struct
  * @return     None.
  */
__STATIC_INLINE void DMIC_VAD_SetBIQCoeff(VAD_T *vad, DMIC_VAD_BIQ_T *psBIQCoeff)
{
    vad->BIQCTL0 = (vad->BIQCTL0 & ~VAD_BIQCTL0_BIQA1_Msk) | ((psBIQCoeff->u16BIQCoeffA1 << VAD_BIQCTL0_BIQA1_Pos)&VAD_BIQCTL0_BIQA1_Msk);
    vad->BIQCTL0 = (vad->BIQCTL0 & ~VAD_BIQCTL0_BIQA2_Msk) | ((psBIQCoeff->u16BIQCoeffA2 << VAD_BIQCTL0_BIQA2_Pos)&VAD_BIQCTL0_BIQA2_Msk);
    vad->BIQCTL1 = (vad->BIQCTL1 & ~VAD_BIQCTL1_BIQB0_Msk) | ((psBIQCoeff->u16BIQCoeffB0 << VAD_BIQCTL1_BIQB0_Pos)&VAD_BIQCTL1_BIQB0_Msk);
    vad->BIQCTL1 = (vad->BIQCTL1 & ~VAD_BIQCTL1_BIQB1_Msk) | ((psBIQCoeff->u16BIQCoeffB1 << VAD_BIQCTL1_BIQB1_Pos)&VAD_BIQCTL1_BIQB1_Msk);
    vad->BIQCTL2 = (vad->BIQCTL2 & ~VAD_BIQCTL2_BIQB2_Msk) | ((psBIQCoeff->u16BIQCoeffB2 << VAD_BIQCTL2_BIQB2_Pos)&VAD_BIQCTL2_BIQB2_Msk);
}

void DMIC_EnableChMsk(DMIC_T *dmic, uint32_t u32ChMsk);
void DMIC_DisableChMsk(DMIC_T *dmic, uint32_t u32ChMsk);
void DMIC_Open(DMIC_T *dmic);
void DMIC_Close(DMIC_T *dmic);
void DMIC_SetDSPGainVolume(DMIC_T *dmic, uint32_t u32ChMsk, int16_t i16ChVolume);
void DMIC_ClearFIFO(DMIC_T *dmic);
uint32_t DMIC_SetSampleRate(DMIC_T *dmic, uint32_t u32SampleRate);
uint32_t DMIC_GetSampleRate(DMIC_T *dmic);

uint32_t DMIC_VAD_SetSampleRate(VAD_T *vad, uint32_t u32SampleRate);
uint32_t DMIC_VAD_GetSampleRate(VAD_T *vad);
/** @} end of group DMIC_EXPORTED_FUNCTIONS */

/** @} end of group DMIC_Driver */

/** @} end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif //__DMIC_H__
