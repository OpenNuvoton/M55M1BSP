/***************************************************************************//**
 * @file     UVC.h
 * @brief    USB driver header file
 * @version  2.0.0
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __HSUSBD_UVC_H__
#define __HSUSBD_UVC_H__

#include "NuMicro.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/* Define the vendor id and product id */
#define HSUSBD_VID          0x0416
#define HSUSBD_PID          0x1104

#define USBD_MAX_DMA_LEN    1022 //0x10000

/*-------------------------------------------------------------*/
/* Define EP maximum packet size */
#define CEP_MAX_PKT_SIZE        64
#define CEP_OTHER_MAX_PKT_SIZE  64
#define EPA_MAX_PKT_SIZE        1024
#define EPA_OTHER_MAX_PKT_SIZE  64
#define EPC_MAX_PKT_SIZE        8
#define EPC_OTHER_MAX_PKT_SIZE  8

#define CEP_BUF_BASE    0
#define CEP_BUF_LEN     CEP_MAX_PKT_SIZE
#define EPA_BUF_BASE    0x200
#define EPA_BUF_LEN     (0x800)
#define EPC_BUF_BASE    0xE00
#define EPC_BUF_LEN     (EPC_MAX_PKT_SIZE)

/* Define the interrupt In EP number */
#define ISO_IN_EP_NUM      0x01
#define INT_IN_EP_NUM      0x03

/* Define Descriptor information */
#define USBD_SELF_POWERED               0
#define USBD_REMOTE_WAKEUP              0
#define USBD_MAX_POWER                  50  /* The unit is in 2mA. ex: 50 * 2mA = 100mA */

/*-------------------------------------------------------------*/
#define UVC_CS_INTERFACE    0x0001
#define UVC_CT_CONTROL      0x0100
#define UVC_PU_CONTROL      0x0500
#define UVC_INTERFACE_CTRL  0x0000

#define PLAY_FEATURE_UNITID 0x06

/*!<Define Video Class Specific Request */
#define UVC_SET_CUR     0x01
#define UVC_GET_CUR     0x81
#define UVC_GET_MIN     0x82
#define UVC_GET_MAX     0x83
#define UVC_GET_RES     0x84
#define UVC_GET_LEN     0x85
#define UVC_GET_INFO    0x86
#define UVC_GET_DEF     0x87

/* Payload Header */
#define UVC_PH_EndOfFrame       0x02
#define UVC_PH_Preview          0x00
#define UVC_PH_Snapshot         0x20

/* Control Selector Codes */
/* VideoControl Interface Control Selectors */
#define VC_CONTROL_UNDEFINED                0x0000
#define VC_VIDEO_POWER_MODE_CONTROL         0x0100
#define VC_REQUEST_ERROR_CODE_CONTROL       0X0200
#define Reserved                            0x0300

/* Terminal Control Selectors */
#define TE_CONTROL_UNDEFINED                0x00

/* Selector Unit Control Selectors */
#define SU_CONTROL_UNDEFINED                0x00
#define SU_INPUT_SELECT_CONTROL             0x01

/* Camera Terminal Control Selectors */
#define CT_CONTROL_UNDEFINED                0x00
#define CT_SCANNING_MODE_CONTROL            0x01
#define CT_AE_MODE_CONTROL                  0x02
#define CT_AE_PRIORITY_CONTROL              0x03
#define CT_EXPOSURE_TIME_ABSOLUTE_CONTROL   0x04
#define CT_EXPOSURE_TIME_RELATIVE_CONTROL   0x05
#define CT_FOCUS_ABSOLUTE_CONTROL           0x06
#define CT_FOCUS_RELATIVE_CONTROL           0x07
#define CT_FOCUS_AUTP_CONTROL               0x08
#define CT_IRIS_ABSOLUTE_CONTROL            0x09
#define CT_IRIS_RELATIVE_CONTROL            0x0A
#define CT_ZOOM_ABSOLUTE_CONTROL            0x0B
#define CT_ZOOM_RELATIVE_CONTROL            0x0C
#define CT_PANTILT_ABSOLUTE_CONTROL         0x0D
#define CT_PANTILT_RELATIVE_CONTROL         0x0E
#define CT_ROLL_ABSOLUTE_CONTROL            0x0F
#define CT_ROLL_RELATIVE_CONTROL            0x10
#define CT_PRIVACY_CONTROL                  0x11

/* Processing Unit Control Selectors */
#define PU_CONTROL_UNDEFINED                        0x00
#define PU_BACKLIGHT_COMPENSATION_CONTROL           0x0100
#define PU_BRIGHTNESS_CONTROL                       0x0200
#define PU_CONTRAST_CONTROL                         0x0300
#define PU_GAIN_CONTROL                             0x0400
#define PU_POWER_LINE_FREQUENCY_CONTROL             0x0500
#define PU_HUE_CONTROL                              0x0600
#define PU_SATURATION_CONTROL                       0x0700
#define PU_SHARPNESS_CONTROL                        0x0800
#define PU_GAMMA_CONTROL                            0x0900
#define PU_WHITE_BALANCE_TEMPERATURE_CONTROL        0x0A00
#define PU_WHITE_BALANCE_TEMPERATURE_AUTO_CONTROL   0x0B00
#define PU_WHITE_BALANCE_COMPONENT_CONTROL          0x0C00
#define PU_WHITE_BALANCE_COMPONENT_AUTO_CONTROL     0x0D00
#define PU_DIGITAL_MULTIPLIER_CONTROL               0x0E00
#define PU_DIGITAL_MULTIPLIER_LIMIT_CONTROL         0x0F00
#define PU_HUE_AUTO_CONTROL                         0x1000
#define PU_ANALOG_VIDEO_STANDARD_CONTROL            0x1100
#define PU_ANALOG_LOCK_STATUS_CONTROL               0x1200

// define audio for interface 2
#define PU_MUTE_CONTROL                             0x0102
#define PU_VOLUME_CONTROL                           0x0202
/* Extension Unit Control Selectors */
#define XU_CONTROL_UNDEFINED                        0x00

/* VideoStreaming Interface Control Selectors */
#define VS_CONTROL_UNDEFINED                        0x0000
#define VS_PROBE_CONTROL                            0x0100
#define VS_COMMIT_CONTROL                           0x0200
#define VS_STILL_PROBE_CONTROL                      0x0300
#define VS_STILL_COMMIT_CONTROL                     0x0400
#define VS_STILL_IMAGE_TRIGGER_CONTROL              0x0500
#define VS_STREAM_ERROR_CODE_CONTROL                0x0600
#define VS_GENERATE_KEY_FRAME_CONTROL               0x0700
#define VS_UPDATE_FRAME_SEGMENT_CONTROL             0x0800
#define VS_SYNCH_DELAY_CONTROL                      0x0900

/* Request Error Code */
#define EC_No_Error         0x00
#define EC_Not_Ready        0x01
#define EC_Wrong_State      0x02
#define EC_Power            0x03
#define EC_Out_Of_Range     0x04
#define EC_Invalid_Uint     0x05
#define EC_Invalid_Control  0x06
#define EC_Invalid_Request  0x07

/* Preview Image Frame Index */
//#define UVC_VGA         1
//#define UVC_QVGA        2
//#define UVC_QQVGA       3
#define UVC_VGA         3
#define UVC_QVGA        1
#define UVC_QQVGA       2


/* Still Image Frame Index */
//#define UVC_STILL_VGA       1
//#define UVC_STILL_QVGA      2
//#define UVC_STILL_QQVGA     3

#define UVC_STILL_VGA       3
#define UVC_STILL_QVGA      1
#define UVC_STILL_QQVGA     2

/* Image Frame Size */
#define UVC_SIZE_VGA        0x00096000  /* 640*480 */
#define UVC_SIZE_QVGA       0x00025800  /* 320*240 */
#define UVC_SIZE_QQVGA      0x00009600  /* 160*120 */

/* Preview and Snapshot Format Index */
#define UVC_Format_YUY2     1
#define UVC_Foramt_MJPEG    2
#define UVC_Format_RGB565   3

#define UVC_Color_Format    UVC_Format_RGB565

/*-----------------------------------------*/
#ifdef  __cplusplus
extern "C"
{
#endif

    typedef struct _VIDEOSTREAMCMDDATA
    {
        uint16_t  bmHint;
        uint8_t   bFormatIndex;
        uint8_t   bFrameIndex;
        uint32_t  dwFrameInterval;
        uint16_t  dwKeyFrameRate;
        uint16_t  wPFrameRate;
        uint16_t  wCompQuality;
        uint16_t  wCompWindowSize;
        uint16_t  wDelay;
        uint32_t  dwMaxVideoFrameSize;
        uint32_t  dwMaxPayloadTransferSize;
    } __attribute__((packed))  VIDEOSTREAMCMDDATA;

    typedef struct _VIDEOSTREAMSTILLCMDDATA
    {
        uint8_t     bFormatIndex;
        uint8_t     bFrameIndex;
        uint8_t       bCompressionIndex;
        uint32_t    dwMaxVideoFrameSize;
        uint32_t    dwMaxPayloadTranferSize;
    } __attribute__((packed))  VIDEOSTREAMSTILLCMDDATA;

    typedef struct _CaptureFilter
    {
        int16_t volatile Brightness;
        int16_t volatile Contrast;
        int16_t volatile Hue;
        uint8_t volatile POWER_LINE_FREQUENCY;
        int16_t volatile Saturation;
        int16_t volatile Sharpness;
        int16_t volatile Gamma;
        int16_t volatile Backlight;
    } __attribute__((packed)) CaptureFilter;

#ifdef  __cplusplus
}
#endif

typedef struct
{
    VIDEOSTREAMCMDDATA          VSCmdCtlData;
    uint16_t u16Dummy;
    VIDEOSTREAMSTILLCMDDATA     VSStillCmdCtlData;
    CaptureFilter               CapFilter;
    uint32_t  u32FrameIndx;       /* Current Frame index */
    uint32_t  u32FormatIndx;      /* Current Format index */
    uint32_t  IsoMaxPacketSize[5];
    uint8_t   u8ErrCode;          /* UVC Error Code */
    char    bChangeData;        /* Format or Frame index Change glag */
} UVC_INFO_T;

typedef struct
{
    int32_t   PU_BACKLIGHT_COMPENSATION_MIN;
    int32_t   PU_BACKLIGHT_COMPENSATION_MAX;
    int32_t   PU_BACKLIGHT_COMPENSATION_DEF;
    int32_t   PU_BRIGHTNESS_MIN;
    int32_t   PU_BRIGHTNESS_MAX;
    int32_t   PU_BRIGHTNESS_DEF;
    int32_t   PU_CONTRAST_MIN;
    int32_t   PU_CONTRAST_MAX;
    int32_t   PU_CONTRAST_DEF;
    int32_t   PU_HUE_MIN;
    int32_t   PU_HUE_MAX;
    int32_t   PU_HUE_DEF;
    int32_t   PU_SATURATION_MIN;
    int32_t   PU_SATURATION_MAX;
    int32_t   PU_SATURATION_DEF;
    int32_t   PU_SHARPNESS_MIN;
    int32_t   PU_SHARPNESS_MAX;
    int32_t   PU_SHARPNESS_DEF;
    int32_t   PU_GAMMA_MIN;
    int32_t   PU_GAMMA_MAX;
    int32_t   PU_GAMMA_DEF;
    int32_t   PU_POWER_LINE_FREQUENCY_MIN;
    int32_t   PU_POWER_LINE_FREQUENCY_MAX;
    int32_t   PU_POWER_LINE_FREQUENCY_DEF;
} UVC_PU_INFO_T;

typedef struct
{
    uint32_t  MaxVideoFrameSize;
    uint32_t  snapshotMaxVideoFrameSize;
    uint32_t  FormatIndex;
    uint32_t  FrameIndex;
    uint32_t  snapshotFormatIndex;
    uint32_t  snapshotFrameIndex;
    uint32_t  StillImage;
    char    bReady;
} UVC_STATUS_T;

/*-------------------------------------------------------------*/
#define HEAD_WORD0      (HSUSBD_BASE+0x248) /* first head data */
#define HEAD_WORD1      (HSUSBD_BASE+0x24C) /* second head data */
#define HEAD_WORD2      (HSUSBD_BASE+0x250) /* third head data */

#define EPA_HEAD_CNT    (HSUSBD_BASE+0x254) /* EPA head count */
#define EPB_HEAD_CNT    (HSUSBD_BASE+0x258) /* EPB head count */
#define EPC_HEAD_CNT    (HSUSBD_BASE+0x25C) /* EPC head count */

/*-----------------------------------------*/
static __inline uint32_t get_be32(uint8_t *buf)
{
    return ((uint32_t) buf[0] << 24) | ((uint32_t) buf[1] << 16) |
           ((uint32_t) buf[2] << 8) | ((uint32_t) buf[3]);
}


/*-------------------------------------------------------------*/
extern volatile UVC_STATUS_T uvcStatus;

/*-------------------------------------------------------------*/
void UVC_Init(void);
void UVC_InitForHighSpeed(void);
void UVC_InitForFullSpeed(void);
void UVC_ClassRequest(void);
void UVC_SetInterface(uint32_t u32AltInterface);
uint32_t UVC_PU_Info(uint32_t Req, uint32_t Unit);
void UVC_PU_Control(int16_t Value);
int UVC_SendImage(uint32_t u32Addr, uint32_t u32transferSize, uint32_t bStillImage);
void ChangeFrame(char bChangeSize, uint32_t u32Address, uint16_t u16Width, uint16_t u16Height);
bool UVC_IsConnect(void);

#ifdef  __cplusplus
}
#endif

#endif  /* __HSUSBD_UVC_H__ */

/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
