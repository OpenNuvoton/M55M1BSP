/******************************************************************************
 * @file     audio_class.h
 * @version  V1.00
 * @brief    Audio Class header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __AUDIO_CLASS_H__
#define __AUDIO_CLASS_H__

/*!<Includes */
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Macro define                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
/* Define the vendor id and product id */
#define USBD_VID                            0x0416
#define USBD_PID                            0x120B /* For MCU/NuMaker connecting to NuVotonAudioGUI */

/*!<Define Audio Class Current State */
#define UAC_STOP_AUDIO_RECORD               0
#define UAC_START_AUDIO_RECORD              1
#define UAC_PROCESSING_AUDIO_RECORD         2
#define UAC_READY_AUDIO_RECORD              3
#define UAC_BUSY_AUDIO_RECORD               4

#define UAC_STOP_AUDIO_SPEAK                0
#define UAC_START_AUDIO_SPEAK               1
#define UAC_PROCESS1_AUDIO_SPEAK              2
#define UAC_PROCESS2_AUDIO_SPEAK              3
#define UAC_BUSY_AUDIO_SPEAK                4

/*!<Define Audio Class Specific Request */
#define UAC_REQUEST_CODE_UNDEFINED          0x00
#define UAC_SET_CUR                         0x01
#define UAC_GET_CUR                         0x81
#define UAC_SET_MIN                         0x02
#define UAC_GET_MIN                         0x82
#define UAC_SET_MAX                         0x03
#define UAC_GET_MAX                         0x83
#define UAC_SET_RES                         0x04
#define UAC_GET_RES                         0x84
#define UAC_SET_MEM                         0x05
#define UAC_GET_MEM                         0x85
#define UAC_GET_STAT                        0xFF

/* Define Endpoint maximum packet size */
#define CEP_MAX_PKT_SIZE                    64
#define EPC_MAX_PKT_SIZE                    ((REC_RATE_96K * REC_CHANNELS * 2 / 1000) + 50)
#define EPD_MAX_PKT_SIZE                    ((PLAY_RATE_96K * PLAY_CHANNELS * 2 / 1000) + 50)
#define EPE_MAX_PKT_SIZE                    4
#define EPF_MAX_PKT_SIZE                    16
#define EPG_MAX_PKT_SIZE                    64
#define EPH_MAX_PKT_SIZE                    64

#define SETUP_BUF_BASE                      0
#define SETUP_BUF_LEN                       8
#define CEP_BUF_BASE                        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define CEP_BUF_LEN                         (CEP_MAX_PKT_SIZE)
#define EPC_BUF_BASE                        (CEP_BUF_BASE + CEP_BUF_LEN)
#define EPC_BUF_LEN                         (EPC_MAX_PKT_SIZE)
#define EPD_BUF_BASE                        (EPC_BUF_BASE + EPC_BUF_LEN)
#define EPD_BUF_LEN                         (EPD_MAX_PKT_SIZE)
#define EPE_BUF_BASE                        (EPD_BUF_BASE + EPD_BUF_LEN)
#define EPE_BUF_LEN                         (EPE_MAX_PKT_SIZE)
#define EPF_BUF_BASE                        (EPE_BUF_BASE + EPE_BUF_LEN)
#define EPF_BUF_LEN                         (EPF_MAX_PKT_SIZE)
#define EPG_BUF_BASE                        (EPF_BUF_BASE + EPF_BUF_LEN)
#define EPG_BUF_LEN                         (EPG_MAX_PKT_SIZE)
#define EPH_BUF_BASE                        (EPG_BUF_BASE + EPG_BUF_LEN)
#define EPH_BUF_LEN                         (EPH_MAX_PKT_SIZE)

/* For usb data size  = 24bit use */
#define EPC_MAX_PKT_SIZE_24bit              64
#define EPD_MAX_PKT_SIZE_24bit              ((PLAY_RATE_96K * PLAY_CHANNELS * 3 / 1000) + 50)
#define EPC_BUF_LEN_24bit                   (EPC_MAX_PKT_SIZE_24bit)
#define EPD_BUF_BASE_24bit                  (EPC_BUF_BASE + EPC_BUF_LEN_24bit)
#define EPD_BUF_LEN_24bit                   (EPD_MAX_PKT_SIZE_24bit)
#define EPE_BUF_BASE_24bit                  (EPD_BUF_BASE_24bit + EPD_BUF_LEN_24bit)
#define EPF_BUF_BASE_24bit                  (EPE_BUF_BASE_24bit + EPE_BUF_LEN)
#define EPG_BUF_BASE_24bit                  (EPF_BUF_BASE_24bit + EPF_BUF_LEN)
#define EPH_BUF_BASE_24bit                  (EPG_BUF_BASE_24bit + EPG_BUF_LEN)

/* Define Descriptor information */
#define USBD_SELF_POWERED                   0
#define USBD_REMOTE_WAKEUP                  0
#define USBD_MAX_POWER                      100     /* The unit is in 2mA. ex: 50 * 2mA = 100mA */

#define LEN_CONFIG_AND_SUBORDINATE  (LEN_CONFIG+LEN_INTERFACE+LEN_ENDPOINT*2)

/*!<Define AUDIO Class Specific Request */
#define GET_CUR_VOL                           0x81
#define GET_MIN_VOL                           0x82
#define GET_MAX_VOL                           0x83
#define GET_RES_VOL                           0x84

/* Endpoint Control Selectors */
#define EP_CONTROL_UNDEFINED                0x0
#define SAMPLING_FREQ_CONTROL               0x1

#define HID_PREFIX_LEN                      0x04
#define I2C_PREFIX_LEN                      0x04
#define HID_HOST_PREFIX_LEN                 0x03
#define I2C_BUFF_SIZE                       4096

/*!<Define Audio information */
#define PLAY_RATE_96K                       96000
#define PLAY_RATE_48K                       48000           /* The audo play sampling rate. */
#define PLAY_RATE_32K                       32000
#define PLAY_RATE_22_05K                    22050
#define PLAY_RATE_16K                       16000
#define PLAY_RATE_11_025K                   11025
#define PLAY_RATE_8K                        8000
#define PLAY_CHANNELS                       2               /* Number of channels. */
#define PLAY_4CHANNELS                      (4)

#define USB_RATE                            PLAY_RATE_96K   /* for buffer calculation, record and play rate must set the same as this */

#define AMIC_BIT_RES                        16
#define AMIC_DATASIZE_BC                    (AMIC_BIT_RES >> 3)

/* Ring buffer definition */
#define RING_BUFF_LEVEL                     8

/* Microphone */
#define REC_CHANNELS                        (2)             /**[LT]**/
#define REC_4CHANNELS                       (4)

/* The record sampling rate. Must be the same with PLAY_RATE */
#define REC_RATE_96K                        96000
#define REC_RATE_48K                        48000
#define REC_RATE_32K                        32000
#define REC_RATE_22_05K                     22050
#define REC_RATE_16K                        16000
#define REC_RATE_11_025K                    11025
#define REC_RATE_8K                         8000

/* Unit ID */
#define PLAY_FEATURE_UNITID                 0x06
#define REC_FEATURE_UNITID                  0x05

/* Define Descriptor information */
#if (PLAY_CHANNELS == 1)
#define PLAY_CH_CFG                         1
#endif
#if (PLAY_CHANNELS == 2)
#define PLAY_CH_CFG                         3
#endif

#if (REC_CHANNELS == 1)
#define REC_CH_CFG                          0x1
#endif
#if (REC_CHANNELS == 2)
#define REC_CH_CFG                          0x3
#endif
#if (REC_CHANNELS == 4)
#define REC_CH_CFG                          0xF
#endif

/* NAU7802 ADC parameter */
#define HIDTRANS_CMD_7802_CONVERTER         0x31
#define ADC_Single                          0x00
#define ADC_Dual                            0x01
#define ADC_Polling                         0x00
#define ADC_Time_Delay                      0x01

/* HID Transfer structure */
typedef __PACKED_STRUCT
{
    uint8_t u8Header;
    uint8_t u8DataLen;
    uint8_t u8FrameNum;
    uint8_t u8CommandType;
} HIDTRANS_CMD_T;

typedef __PACKED_UNION
{
    __PACKED_STRUCT
    {
        uint8_t Recipient : 5;
        uint8_t Type      : 2;
        uint8_t Dir       : 1;
    } BM;

    uint8_t B;
} REQUEST_TYPE;

typedef __PACKED_UNION
{
    __PACKED_STRUCT
    {
        uint8_t L;
        uint8_t H;
    } WB;

    uint16_t W;
} DB16;

typedef __PACKED_STRUCT
{
    REQUEST_TYPE    bmRequestType;
    uint8_t         bRequest;
    DB16            wValue;
    DB16            wIndex;
    DB16            wLength;
} SetupPkt_t;

typedef enum
{
    NOT_PROCESSING,
    PROCESSING,
    WAITING_MORE_DATA,
    NO_DATA,
    FINISHED
} eProcessStep;

typedef enum
{
    E_RS_NONE,          // no re-sampling
    E_RS_UP,            // up sampling
    E_RS_DOWN           // down sampling
} RESAMPLE_STATE_T;

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
/* UAC */
void UAC_Init(void);
void UAC_ClassRequest(void);
void UAC_SetInterface(void);

/* Endpoint handlers */
void EPC_Handler(void);
void EPD_Handler(void);
void EPE_Handler(void);
void EPF_Handler(void);

/* Playback and Record buffer process */
void PlaybackPingpongBuffClear(void);
void RecordPingpongBuffClear(void);
void PlaybackBuffInit(void);
void RecordBuffInit(void);

/* Async USB data transfer */
void Adjust_PLL(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Extern Functions                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
extern void PDMA_Init(void);
extern void I2S_TX_Start(void);
extern void I2S_TX_Stop(void);
extern void I2S_RX_Start(void);
extern void I2S_RX_Stop(void);
extern void Switch_I2S_Sample_Rate(void);
extern void HIRC_AutoTrim_Enable(void);
extern void HIRC_AutoTrim_Reset(void);
extern void Flash_Memory(void);

#ifdef __cplusplus
}
#endif

#endif  // __AUDIO_CLASS_H__
