/******************************************************************************
 * @file     user_config.h
 * @version  V1.00
 * @brief    User configuration file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define I2C_PORT                                I2C3

#define I2S_CLKSRC                              CLK_I2SSEL_I2S0SEL_HIRC
#define I2S_RST                                 SYS_I2S0RST
#define I2S_MODULE                              I2S0_MODULE
#define I2S_PORT                                I2S0

#define PDMA_MODULE                             PDMA0_MODULE
#define PDMA_RST                                SYS_PDMA0RST
#define PDMA_IRQn                               PDMA0_IRQn
#define PDMA_PORT                               PDMA0

#define DATA_FLASH_USERCONFIG_BASE              0x7F000

/*---------------------------------------------------------------------------------------------------------*/
/* Macro define                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define USER_CONFIG_MASK                        0x01

// PDMA
#define PDMA_CH1_MASK                           (BIT1)                              /*!<DMA Channel1 Mask Bit  \hideinitializer */
#define PDMA_CH2_MASK                           (BIT2)                              /*!<DMA Channel2 Mask Bit  \hideinitializer */

// I2S
#define I2S_FIFO_24BIT_MSB                      (0)
#define I2S_FIFO_24BIT_LSB                      (1)

#define I2S_ORDER_EVENHIGH                      (0 << I2S_CTL0_ORDER_Pos)           /*!< I2S Even channel data at high byte in 8-bit/16-bit data width. */
#define I2S_ORDER_EVENLOW                       (1 << I2S_CTL0_ORDER_Pos)           /*!< I2S Even channel data at low byte in 8-bit/16-bit data width. */

#define I2S_TDMCHNUM_2CH                        (0 << I2S_CTL0_TDMCHNUM_Pos)        /*!< I2S TDM Channel Number is 2-Channel */
#define I2S_TDMCHNUM_4CH                        (1 << I2S_CTL0_TDMCHNUM_Pos)        /*!< I2S TDM Channel Number is 2-Channel */

#define I2S_CHWIDTH_16                          (1 << I2S_CTL0_CHWIDTH_Pos)         /*!< I2S Channel Width is 16-bit */
#define I2S_CHWIDTH_32                          (3 << I2S_CTL0_CHWIDTH_Pos)         /*!< I2S Channel Width is 32-bit */

#define I2S_PB16ORD_LOW                         (0 << I2S_CTL1_PB16ORD_Pos)         /*!< I2S Low 16-bit read/write access first. */
#define I2S_PB16ORD_HIGH                        (1 << I2S_CTL1_PB16ORD_Pos)         /*!< I2S High 16-bit read/write access first. */

#define I2S_PBWIDTH_32                          (0 << I2S_CTL1_PBWIDTH_Pos)         /*!< I2S Peripheral Bus 32 bits data width. */
#define I2S_PBWIDTH_16                          (1 << I2S_CTL1_PBWIDTH_Pos)         /*!< I2S Peripheral Bus 16 bits data width. */

#define I2S_SET_STEREOORDER(i2s, u32Order)      ((i2s)->CTL0 = ((i2s)->CTL0 & (~I2S_CTL0_ORDER_Msk)) | u32Order)
#define I2S_SET_PB16ORD(i2s, u32Order)          ((i2s)->CTL1 = ((i2s)->CTL1 & (~I2S_CTL1_PB16ORD_Msk)) | u32Order)
#define I2S_SET_PBWIDTH(i2s, u32Width)          ((i2s)->CTL1 = ((i2s)->CTL1 & (~I2S_CTL1_PBWIDTH_Msk)) | u32Width)
#define I2S_SET_CHWIDTH(i2s, u32Width)          ((i2s)->CTL0 = ((i2s)->CTL0 & (~I2S_CTL0_CHWIDTH_Msk)) | u32Width)
#define I2S_SET_TDMCHNUM(i2s, u32ChNum)         ((i2s)->CTL0 = ((i2s)->CTL0 & (~I2S_CTL0_TDMCHNUM_Msk)) | u32ChNum)

// HIRC Auto Trim
#define SYS_IS_TRIMHIRC_DONE()                  (SYS->TISTS48M & SYS_TISTS48M_FREQLOCK_Msk) //(SYS->HIRCTISTS & SYS_HIRCTISTS_FREQLOCK_Msk)
#define SYS_CLEAR_TRIMHIRC_INT_FLAG(u32Mask)    (SYS->TISTS48M |= (u32Mask)) //(SYS->HIRCTISTS |= (u32Mask))
#define SYS_SET_TRIMHIRC_LOOPSEL(u32LoopSel)    (SYS->TCTL48M = (SYS->TCTL48M & ~SYS_TCTL48M_LOOPSEL_Msk) | (u32LoopSel))  //(SYS->HIRCTCTL = (SYS->HIRCTCTL&~SYS_HIRCTCTL_LOOPSEL_Msk) | (u32LoopSel))
#define SYS_SET_TRIMHIRC_RETRYCNT(u32RetryCnt)  (SYS->TCTL48M = (SYS->TCTL48M & ~SYS_TCTL48M_RETRYCNT_Msk) | (u32RetryCnt)) //(SYS->HIRCTCTL = (SYS->HIRCTCTL&~SYS_HIRCTCTL_RETRYCNT_Msk) | (u32RetryCnt))
#define SYS_ENABLE_TRIMHIRC_CLKERRSTOP()        (SYS->TCTL48M |= SYS_TCTL48M_CESTOPEN_Msk) //(SYS->HIRCTCTL |= SYS_HIRCTCTL_CESTOPEN_Msk)
#define SYS_DISABLE_TRIMHIRC_CLKERRSTOP()       (SYS->TCTL48M &= ~SYS_TCTL48M_CESTOPEN_Msk) //(SYS->HIRCTCTL &= ~SYS_HIRCTCTL_CESTOPEN_Msk)

#define SYS_HIRCTCTL_REFCLK_LXT                 (0x0UL << SYS_TCTL48M_REFCKSEL_Pos) //(0x0UL << SYS_HIRCTCTL_REFCKSEL_Pos)    /*!<HIRC trim reference clock is from LXT (32.768 kHz). \hideinitializer */
#define SYS_HIRCTCTL_LOOPSEL_16                 (0x02UL << SYS_TCTL48M_LOOPSEL_Pos) //(0x2UL << SYS_HIRCTCTL_LOOPSEL_Pos)     /*!<Trim value calculation is based on average difference in 16 clocks of reference clock.  \hideinitializer */
#define SYS_HIRCTCTL_RETRYCNT_64                (0x0UL << SYS_TCTL48M_RETRYCNT_Pos) //(0x0UL << SYS_HIRCTCTL_RETRYCNT_Pos)    /*!<Trim value calculation is based on average difference in 4 clocks of reference clock.  \hideinitializer */
#define SYS_HIRCTISTS_TRIMFAIL_INT_FLAG         (SYS_TISTS48M_TFAILIF_Msk)  //(SYS_HIRCTISTS_TFAILIF_Msk)             /*!< Trim Fail Interrupt Flag.  \hideinitializer */
#define SYS_HIRCTISTS_CLKERROR_INT_FLAG         (SYS_TISTS48M_CLKERRIF_Msk) //(SYS_HIRCTISTS_CLKERRIF_Msk)            /*!< Clock Error Interrupt Flag.  \hideinitializer */
#define SYS_HIRCTCTL_FREQSEL_48M                (0x01UL << SYS_TCTL48M_FREQSEL_Pos) //(0x1UL << SYS_HIRCTCTL_FREQSEL_Pos)     /*!<Enable HIRC auto trim function and trim HIRC to 48 MHz. \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* Enum define                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
typedef enum
{
    Empty_Function = 0,

    Firmware_Version,

    Reset_Function,
    Apply_Function,

    USBDIn_Function,
    USBDOut_Function,

    I2S0DInOut_OnOff,
    I2S1DInOut_OnOff,

    I2SDSync_DataType,
    I2SDSync_SerialMode,

    I2S0DInOut_Format,
    I2S0DInOut_WordSize,
    I2S0DInOut_FrameSize,

    I2S1DInOut_Format,
    I2S1DInOut_WordSize,
    I2S1DInOut_FrameSize,

    InputMultiplexer_DSP,
    InputMultiplexer_Bypass,

    OutputMultiplexer_USB,
    OutputMultiplexer_I2S0,
    OutputMultiplexer_I2S1,

    InputGain_DSP,
    InputGain_Bypass,

    OutputGain_USB,
    OutputGain_I2S0,
    OutputGain_I2S1,

    SampleRate,

    I2C_ClockRate,
    I2C_Address,

    InputClock_Crystal,

    USBDin_WordSize,
    USBDout_WordSize,

    I2S0_MCLK,
    I2S1_MCLK,

    Default_BongioviConfig,
    Default_BoardNum,

    USB_DeviceVID,
    USB_DevicePID,
    USB_Manufacture,
    USB_Product,
    USB_SerialNumber,

    SystemConfig_Max = 127
} eUserConfig;

typedef enum
{
    Disable,
    Enable
} eEnable;

typedef enum
{
    I2S_Stereo,
    I2S_Mono
} eI2SDataType;

typedef enum
{
    I2S_Slave,
    I2S_Master
} eI2SSerialMode;


typedef enum
{
    I2S_2CHANNELS,
    I2S_4CHANNELS
} eI2SChannelNum;

typedef enum
{
    I2S_Standard,
    I2S_Left,
    PCM_Standard,
    PCM_Left,
} eI2SFormat;

typedef enum
{
    I2S_16bit,
    I2S_24bit,
    I2S_32bit
} eI2SWordSize;

typedef enum
{
    USB_16bit,
    USB_24bit,
    USB_32bit
} eUSBWordSize;

typedef enum
{
    I2S_32Fs,
    I2S_48Fs,
    I2S_64Fs
} eI2SFrameSize;

typedef enum
{
    In_Mux_None,
    In_Mux_I2S0,
    In_Mux_I2S1,
    In_Mux_USB
} eInputMux;

typedef enum
{
    Out_Mux_None,
    Out_Mux_DPS_Stereo,
    Out_Mux_ByPass,
    Out_Mux_DPS_Sub
} eOutputMux;

typedef enum
{
    SR_96kHz,
    SR_48kHz,
    SR_32kHz,
    SR_16kHz,
    SR_8kHz
} eSampleRate;

typedef enum
{
    I2C_Standard,
    I2C_Fast,
    I2C_Fast_Plus
} eI2CCloclRate;

typedef enum
{
    I2C_Addr_0x40,
    I2C_addr_0x42,
    I2C_Addr_0x44,
    I2C_Addr_0x46
} eI2SAddress;

typedef enum
{
    Crystal_12_288,
    Crystal_11_2896
} eCystal;

typedef enum
{
    Board_Normal,
    Board_Debug,
    Board_Special
} eBoardMode;

/*---------------------------------------------------------------------------------------------------------*/
/* Structure define                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
// total 64 byte
typedef __PACKED_STRUCT
{
    uint8_t u8Flag;
    uint8_t u8Empty_Function;

    uint8_t u8Firmware_Version;

    uint8_t u8Reset_Function;
    uint8_t u8Apply_Function;

    uint8_t u8USBDin_Function;
    uint8_t u8USBDout_Function;

    uint8_t u8I2S0DInOut_OnOff;
    uint8_t u8I2S1DInOut_OnOff;

    uint8_t u8I2SDSync_DataType;
    uint8_t u8I2SDSync_SerialMode;

    uint8_t u8I2S0DInOut_Format;
    uint8_t u8I2S0DInOut_WordSize;
    uint8_t u8I2S0DInOut_FrameSize;

    uint8_t u8I2S1DInOut_Format;
    uint8_t u8I2S1DInOut_WordSize;
    uint8_t u8I2S1DInOut_FrameSize;

    uint8_t u8InputMultiplexer_DSP;
    uint8_t u8InputMultiplexer_Bypass;
    uint8_t u8OutputMultiplexer_USB;
    uint8_t u8OutputMultiplexer_I2S0;
    uint8_t u8OutputMultiplexer_I2S1;

    uint16_t u16InputGain_DSP;
    uint16_t u16InputGain_Bypass;
    uint16_t u16OutputGain_USB;
    uint16_t u16OutputGain_I2S0;
    uint16_t u16OutputGain_I2S1;

    uint8_t u8SampleRate;
    uint8_t u8I2SChannelNum;

    uint8_t u8I2C_ClockRate;
    uint8_t u8I2C_Address;

    uint8_t u8InputClock_Crystal;

    uint8_t u8USBDin_WordSize;
    uint8_t u8USBDout_WordSize;

    uint8_t u8I2S0_MCLK;
    uint8_t u8I2S1_MCLK;

    uint8_t u8Default_BongioviConfig;
    uint8_t u8Default_BoardNum;

    uint8_t u8Reserved[93];
} UserConfig_T;

/*---------------------------------------------------------------------------------------------------------*/
/* Global Variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
extern UserConfig_T sUserConfig;
extern volatile uint8_t g_u8ApplyFlag;
extern volatile uint8_t g_u8ResetFlag;

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
extern void NuvotonCmdAnalyze(void);
extern void SetConfigToDefault(void);
void NuvotonDelayUs(uint32_t us);

#define DelayUs NuvotonDelayUs

#ifdef __cplusplus
}
#endif

#endif  // __USER_CONFIG_H__
