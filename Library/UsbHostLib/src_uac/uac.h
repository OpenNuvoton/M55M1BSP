/**************************************************************************//**
 * @file     uac.h
 * @version  V1.00
 * @brief    USB Host audio class header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __INCLUDED_UAC_H__
#define __INCLUDED_UAC_H__

/// @cond HIDDEN_SYMBOLS

//#define UAC_DEBUG

/*
 * Debug message
 */
#define UAC_ERRMSG     (void)usbh_printf
#ifdef UAC_DEBUG
    #define UAC_DBGMSG      (void)usbh_printf
#else
    #define UAC_DBGMSG(...)
#endif

typedef enum
{
    UAC_STATE_CONNECTING,
    UAC_STATE_READY,
    UAC_STATE_RUNNING,
    UAC_STATE_DISCONNECTING,
}  UAC_STATE_E;

/* UAC Version Constants                              */
#define UAC_VERSION_1                 0x0100U /* UAC 1.0 */
#define UAC_VERSION_2                 0x0200U /* UAC 2.0 */

/* Audio Interface Subclass Codes (A.2)               */
#define SUBCLS_UNDEFINED              0x00
#define SUBCLS_AUDIOCONTROL           0x01
#define SUBCLS_AUDIOSTREAMING         0x02
#define SUBCLS_MIDISTREAMING          0x03

/* Audio Interface Protocol Code (A.3)                */
#define PR_PROTOCOL_UNDEFINED         0x00
#define PR_PROTOCOL_IP_VERSION_02_00  0x20  /* UAC 2.0 IP version */

/* Audio Class-specific descritpor types (A.4)        */
#define CS_UNDEFINED                  0x20
#define CS_DEVICE                     0x21
#define CS_CONFIGURATION              0x22
#define CS_STRING                     0x23
#define CS_INTERFACE                  0x24
#define CS_ENDPOINT                   0x25

/* Audio Class-Specific AC Interface Descriptor Subtypes (A.5) */
#define AC_DESCRIPTOR_UNDEFINED       0x00
#define HEADER                        0x01
#define INPUT_TERMINAL                0x02
#define OUTPUT_TERMINAL               0x03
#define MIXER_UNIT                    0x04
#define SELECTOR_UNIT                 0x05
#define FEATURE_UNIT                  0x06
#define PROCESSING_UNIT               0x07
#define EXTENSION_UNIT                0x08

/* UAC 2.0 Audio Class-Specific AC Interface Descriptor Subtypes (A.9, UAC2) */
#define UAC2_EFFECT_UNIT              0x07
#define UAC2_PROCESSING_UNIT          0x08
#define UAC2_EXTENSION_UNIT           0x09
#define UAC2_CLOCK_SOURCE             0x0A
#define UAC2_CLOCK_SELECTOR           0x0B
#define UAC2_CLOCK_MULTIPLIER         0x0C
#define UAC2_SAMPLE_RATE_CONVERTER    0x0D

/* Audio Class-Specific AS Interface Descriptor Subtypes (A.6) */
#define AS_DESCRIPTOR_UNDEFINED       0x00
#define AS_GENERAL                    0x01
#define FORMAT_TYPE                   0x02
#define FORMAT_SPECIFIC               0x03

/* Processing Unit Process Types (A.7)                */
#define PROCESS_UNDEFINED             0x00
#define UP_DOWNMIX_PROCESS            0x01
#define DOLBY_PROLOGIC_PROCESS        0x02
#define _3D_STEREO_EXTENDER_PROCESS   0x03
#define REVERBERATION_PROCESS         0x04
#define CHORUS_PROCESS                0x05
#define DYN_RANGE_COMP_PROCESS        0x06

/* Audio Class-Specific Endpoint Descriptor Subtypes (A.8) */
#define DESCRIPTOR_UNDEFINED          0x00
#define EP_GENERAL                    0x01

/* Audio Class-Specific Request Codes (A.9)           */
#define REQUEST_CODE_UNDEFINED        0x00
#define SET_CUR                       0x01
#define GET_CUR                       0x81
#define SET_MIN                       0x02
#define GET_MIN                       0x82
#define SET_MAX                       0x03
#define GET_MAX                       0x83
#define SET_RES                       0x04
#define GET_RES                       0x84
#define SET_MEM                       0x05
#define GET_MEM                       0x85
#define GET_STAT                      0xFF

/* Terminal Control Selectors (A.10.1)                */
#define TE_CONTROL_UNDEFINED          0x00
#define COPY_PROTECT_CONTROL          0x01

/* Feature Unit Control Selectors (A.10.2)            */
#define FU_CONTROL_UNDEFINED          0x00
#define MUTE_CONTROL                  0x01
#define VOLUME_CONTROL                0x02
#define BASS_CONTROL                  0x03
#define MID_CONTROL                   0x04
#define TREBLE_CONTROL                0x05
#define GRAPHIC_EQUALIZER_CONTROL     0x06
#define AUTOMATIC_GAIN_CONTROL        0x07
#define DELAY_CONTROL                 0x08
#define BASS_BOOST_CONTROL            0x09
#define LOUDNESS_CONTROL              0x0A

/* Up/Down-mix Processing Unit Control Selectors (A.10.3.1) */
#define UD_CONTROL_UNDEFINED          0x00
#define UD_ENABLE_CONTROL             0x01
#define UD_MODE_SELECT_CONTROL        0x02

/* Dolby Prologic Processing Unit Control Selectors (A.10.3.2) */
#define DP_CONTROL_UNDEFINED          0x00
#define DP_ENABLE_CONTROL             0x01
#define DP_MODE_SELECT_CONTROL        0x02

/* 3D Stereo Extender Processing Unit Control Selectors (A.10.3.3) */
#define _3D_CONTROL_UNDEFINED         0x00
#define _3D_ENABLE_CONTROL            0x01
#define SPACIOUSNESS_CONTROL          0x03

/* Reverberation Processing Unit Control Selectors (A.10.3.4) */
#define RV_CONTROL_UNDEFINED          0x00
#define RV_ENABLE_CONTROL             0x01
#define REVERB_LEVEL_CONTROL          0x02
#define REVERB_TIME_CONTROL           0x03
#define REVERB_FEEDBACK_CONTROL       0x04

/* Chorus Processing Unit Control Selectors (A.10.3.5) */
#define CH_CONTROL_UNDEFINED          0x00
#define CH_ENABLE_CONTROL             0x01
#define CHORUS_LEVEL_CONTROL          0x02
#define CHORUS_RATE_CONTROL           0x03
#define CHORUS_DEPTH_CONTROL          0x04

/* Dynamic Range Compressor Processing Unit Control Selectors (A.10.3.6) */
#define DR_CONTROL_UNDEFINED          0x00
#define DR_ENABLE_CONTROL             0x01
#define COMPRESSION_RATE_CONTROL      0x02
#define MAXAMPL_CONTROL               0x03
#define THRESHOLD_CONTROL             0x04
#define ATTACK_TIME                   0x05
#define RELEASE_TIME                  0x06

/* Extension Unit Control Selectors (A.10.4)  */
#define XU_CONTROL_UNDEFINED          0x00
#define XU_ENABLE_CONTROL             0x01

/* Endpoint Control Selectors  (A.10.5) */
#define EP_CONTROL_UNDEFINED          0x00
#define SAMPLING_FREQ_CONTROL         0x01
#define PITCH_CONTROL                 0x02

/*====================================================================================*/
/*  UAC 2.0 Specific Definitions                                                      */
/*====================================================================================*/

/* UAC 2.0 Audio Class-Specific Request Codes (A.14, UAC2)  */
#define UAC2_CUR                      0x01  /* Current setting attribute                  */
#define UAC2_RANGE                    0x02  /* Range attribute                            */
#define UAC2_MEM                      0x03  /* Memory attribute                           */

/* UAC 2.0 Clock Source Control Selectors (A.17.1, UAC2) */
#define UAC2_CS_CONTROL_UNDEFINED     0x00
#define UAC2_CS_SAM_FREQ_CONTROL      0x01  /* Sampling Frequency Control                 */
#define UAC2_CS_CLOCK_VALID_CONTROL   0x02  /* Clock Validity Control                     */

/* UAC 2.0 Clock Selector Control Selectors (A.17.2, UAC2) */
#define UAC2_CX_CONTROL_UNDEFINED     0x00
#define UAC2_CX_CLOCK_SELECTOR_CONTROL 0x01 /* Clock Selector Control                    */

/* UAC 2.0 Clock Multiplier Control Selectors (A.17.3, UAC2) */
#define UAC2_CM_CONTROL_UNDEFINED     0x00
#define UAC2_CM_NUMERATOR_CONTROL     0x01  /* Numerator Control                          */
#define UAC2_CM_DENOMINATOR_CONTROL   0x02  /* Denominator Control                        */

/* UAC 2.0 Feature Unit Control Selectors (A.17.7, UAC2) */
#define UAC2_FU_CONTROL_UNDEFINED     0x00
#define UAC2_FU_MUTE_CONTROL          0x01
#define UAC2_FU_VOLUME_CONTROL        0x02
#define UAC2_FU_BASS_CONTROL          0x03
#define UAC2_FU_MID_CONTROL           0x04
#define UAC2_FU_TREBLE_CONTROL        0x05
#define UAC2_FU_GRAPHIC_EQ_CONTROL    0x06
#define UAC2_FU_AGC_CONTROL           0x07
#define UAC2_FU_DELAY_CONTROL         0x08
#define UAC2_FU_BASS_BOOST_CONTROL    0x09
#define UAC2_FU_LOUDNESS_CONTROL      0x0A
#define UAC2_FU_INPUT_GAIN_CONTROL    0x0B
#define UAC2_FU_INPUT_GAIN_PAD_CONTROL 0x0C
#define UAC2_FU_PHASE_INVERTER_CONTROL 0x0D
#define UAC2_FU_UNDERFLOW_CONTROL     0x0E
#define UAC2_FU_OVERFLOW_CONTROL      0x0F
#define UAC2_FU_LATENCY_CONTROL       0x10

/* Format Type Codes of Format Type Descriptor bFormatType field */
#define FORMAT_TYPE_UNDEFINED         0x00
#define FORMAT_TYPE_I                 0x01
#define FORMAT_TYPE_II                0x02
#define FORMAT_TYPE_III               0x03

/*-----------------------------------------------------------------------------------
 *  Audio Class Control Interface Descriptor header
 */
#ifdef __ICCARM__
typedef struct cs_hdr_t                     /*! Audio Class-Specific AC Interface Header Descriptor   */
{
    __packed uint8_t  bLength;              /*!< Size of this descriptor in bytes                     */
    __packed uint8_t  bDescriptorType;      /*!< CS_INTERFACE descriptor type 0x24                    */
    __packed uint8_t  bDescriptorSubtype;   /*!< HEADER descriptor subtype                            */
} CS_HDR_T;                                 /*! Audio Class-Specific AC Interface Header Descriptor   */
#else
typedef struct __attribute__((__packed__)) cs_hdr_t       /*! Audio Class-Specific AC Interface Header Descriptor   */
{
    uint8_t  bLength;                       /*!< Size of this descriptor in bytes                     */
    uint8_t  bDescriptorType;               /*!< CS_INTERFACE descriptor type 0x24                    */
    uint8_t  bDescriptorSubtype;            /*!< HEADER descriptor subtype                            */
} CS_HDR_T;                                 /*! Audio Class-Specific AC Interface Header Descriptor   */
#endif

/*-----------------------------------------------------------------------------------
 *  Class-Specific AC Interface Header Descriptor (4.3.2)
 */
#ifdef __ICCARM__
typedef struct ac_if_header                 /*! Audio Class-Specific AC Interface Header Descriptor   */
{
    __packed uint8_t  bLength;              /*!< Size of this descriptor, in bytes: 8+n               */
    __packed uint8_t  bDescriptorType;      /*!< CS_INTERFACE descriptor type; 0x24                   */
    __packed uint8_t  bDescriptorSubtype;   /*!< HEADER descriptor subtype; 0x1                       */
    __packed uint16_t bcdADC;               /*!< Audio Device Class Specification Release Number in Binary-Coded Decimal \hideinitializer  */
    __packed uint16_t wTotalLength;         /*!< Total number of bytes returned for the class-specific AudioControl interface
                                                 descriptor. Includes the combined length of this descriptor header and all Unit and
                                                 Terminal descriptors.                                */
    __packed uint8_t  bInCollection;        /*!< The number of AudioStreaming and MIDIStreaming interfaces in the Audio
                                                Interface Collection to which this AudioControl interface belongs.  */
} AC_IF_HDR_T;                              /*! Audio Class-Specific AC Interface Header Descriptor   */
#else
typedef struct __attribute__((__packed__)) ac_if_header    /*! Audio Class-Specific AC Interface Header Descriptor   */
{
    uint8_t  bLength;                       /*!< Size of this descriptor, in bytes: 8+n               */
    uint8_t  bDescriptorType;               /*!< CS_INTERFACE descriptor type; 0x24                   */
    uint8_t  bDescriptorSubtype;            /*!< HEADER descriptor subtype; 0x1                       */
    uint16_t bcdADC;                        /*!< Audio Device Class Specification Release Number in Binary-Coded Decimal \hideinitializer  */
    uint16_t wTotalLength;                  /*!< Total number of bytes returned for the class-specific AudioControl interface
                                                 descriptor. Includes the combined length of this descriptor header and all Unit and
                                                 Terminal descriptors.                                */
    uint8_t  bInCollection;                 /*!< The number of AudioStreaming and MIDIStreaming interfaces in the Audio
                                                 Interface Collection to which this AudioControl interface belongs.  */
} AC_IF_HDR_T;                              /*! Audio Class-Specific AC Interface Header Descriptor   */
#endif

/*-----------------------------------------------------------------------------------
 *  UAC Input Terminal Descriptor
 */
#ifdef __ICCARM__
typedef struct ac_itd_t                     /*! Audio Class-Specific Input Terminal Descriptor        */
{
    __packed uint8_t  bLength;              /*!< Size of this descriptor, in bytes: 12                */
    __packed uint8_t  bDescriptorType;      /*!< CS_INTERFACE descriptor type; 0x24                   */
    __packed uint8_t  bDescriptorSubtype;   /*!< INPUT_TERMINAL descriptor subtype; 0x2               */
    __packed uint8_t  bTerminalID;
    __packed uint16_t wTerminalType;
    __packed uint8_t  bAssocTerminal;
    __packed uint8_t  bNrChannels;
    __packed uint16_t wChannelConfig;
    __packed uint8_t  iChannelNames;
    __packed uint8_t  iTerminal;
} AC_IT_T;                                  /*! Audio Class-Specific Input Terminal Descriptor        */
#else
typedef struct __attribute__((__packed__)) ac_itd_t     /*! Audio Class-Specific Input Terminal Descriptor        */
{
    uint8_t  bLength;                       /*!< Size of this descriptor, in bytes: 12                */
    uint8_t  bDescriptorType;               /*!< CS_INTERFACE descriptor type; 0x24                   */
    uint8_t  bDescriptorSubtype;            /*!< INPUT_TERMINAL descriptor subtype; 0x2               */
    uint8_t  bTerminalID;
    uint16_t wTerminalType;
    uint8_t  bAssocTerminal;
    uint8_t  bNrChannels;
    uint16_t wChannelConfig;
    uint8_t  iChannelNames;
    uint8_t  iTerminal;
} AC_IT_T;                                  /*! Audio Class-Specific Input Terminal Descriptor        */
#endif

/*-----------------------------------------------------------------------------------
 *  UAC Output Terminal Descriptor
 */
#ifdef __ICCARM__
typedef struct ac_otd_t
{
    __packed uint8_t  bLength;
    __packed uint8_t  bDescriptorType;
    __packed uint8_t  bDescriptorSubtype;
    __packed uint8_t  bTerminalID;
    __packed uint16_t wTerminalType;
    __packed uint8_t  bAssocTerminal;
    __packed uint8_t  bSourceID;
    __packed uint8_t  iTerminal;
} AC_OT_T;
#else
typedef struct __attribute__((__packed__)) ac_otd_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bTerminalID;
    uint16_t wTerminalType;
    uint8_t  bAssocTerminal;
    uint8_t  bSourceID;
    uint8_t  iTerminal;
} AC_OT_T;
#endif

/*---------------------------------*/
/*  Terminal Types                 */
/*---------------------------------*/
/* USB Terminal Types     */
#define UAC_TT_USB_UNDEFINED        0x0100  /* USB Terminal, undefined Type.              */
#define UAC_TT_USB_STREAMING        0x0101  /* A Terminal dealing with a signal carried
                                               over an endpoint in an AudioStreaming
                                               interface. The AudioStreaming interface.   */
#define UAC_TT_USB_VENDOR           0x01FF  /* A Terminal dealing with a signal carried
                                               over a vendor-specific interface.          */
/* Input Terminal Types   */
#define UAC_TT_INPUT_UNDEFINED      0x0200  /* Input Terminal, undefined Type.            */
#define UAC_TT_MICROPHONE           0x0201  /* A generic microphone that does not fit
                                               under any of the other classifications.    */
#define UAC_TT_DESKTOP_MICROPHONE   0x0202  /* A microphone normally placed on the desktop
                                               or integrated into the monitor.            */
#define UAC_TT_PERSONAL_MICROPHONE  0x0203  /* A head-mounted or clip-on microphone.      */
#define UAC_TT_OMNI_MICROPHONE      0x0204  /* A microphone designed to pick up voice from
                                               more than one speaker at relatively long
                                               ranges.                                    */
#define UAC_TT_MICROPHONE_ARRAY     0x0205  /* An array of microphones designed for
                                               directional processing using host-based
                                               signal processing algorithms.              */
/* Output Terminal Types  */
#define UAC_TT_OUTPUT_UNDEFINED     0x0300  /* Output Terminal, undefined Type.           */
#define UAC_TT_SPEAKER              0x0301  /* A generic speaker or set of speakers that
                                               doe not fit under any of the other
                                               classifications.                           */
#define UAC_TT_HEADPHONES           0x0302  /* A head-mounted audio output device.        */
#define UAC_TT_HEAD_MOUNTED         0x0303  /* The audio part of a VR head mounted display.
                                               The Associated Interfaces descriptor can
                                               be used to reference the HID interface used
                                               to report the position and orientation of
                                               the HMD.                                   */
#define UAC_TT_DESKTOP_SPEAKER      0x0304  /* Relatively small speaker or set of speakers
                                               normally placed on the desktop or
                                               integrated into the monitor. These speakers
                                               are close to the user and have limited
                                               stereo separation.                         */
#define UAC_TT_ROOM_SPEAKER         0x0305  /* Larger speaker or set of speakers that are
                                               heard well anywhere in the room.           */
#define UAC_TT_COMM_SPEAKER         0x0306  /* Speaker or set of speakers designed for
                                               voice communication.                       */
#define UAC_TT_LFE_SPEAKER          0x0307  /* Speaker designed for low frequencies
                                               (subwoofer). Not capable of reproducing
                                               speech or music.                           */

/*----------------------------------------------------------------------------------------*/
/*  UAC Mixer Unit Descriptor                                                             */
/*----------------------------------------------------------------------------------------*/
#ifdef __ICCARM__
typedef struct ac_mxr_t
{
    __packed uint8_t  bLength;
    __packed uint8_t  bDescriptorType;
    __packed uint8_t  bDescriptorSubtype;
    __packed uint8_t  bUnitID;              /* Constant uniquely identifying the Unit within the audio function. */
    __packed uint8_t  bNrInPins;            /* Number of Input Pins of this Unit: p       */
} AC_MXR_T;
#else
typedef struct __attribute__((__packed__)) ac_mxr_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bUnitID;                       /* Constant uniquely identifying the Unit within the audio function. */
    uint8_t  bNrInPins;                     /* Number of Input Pins of this Unit: p       */
} AC_MXR_T;
#endif

/*-----------------------------------------------------------------------------------
 *  UAC Selector Unit Descriptor
 */
#ifdef __ICCARM__
typedef struct ac_su_t
{
    __packed uint8_t  bLength;
    __packed uint8_t  bDescriptorType;
    __packed uint8_t  bDescriptorSubtype;
    __packed uint8_t  bUnitID;              /* Constant uniquely identifying the Unit within the audio function. */
    __packed uint8_t  bNrInPins;            /* Number of Input Pins of this Unit: p       */
} AC_SU_T;
#else
typedef struct __attribute__((__packed__)) ac_su_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bUnitID;                       /* Constant uniquely identifying the Unit within the audio function. */
    uint8_t  bNrInPins;                     /* Number of Input Pins of this Unit: p       */
} AC_SU_T;
#endif

/*-----------------------------------------------------------------------------------
 *  UAC Feature Unit Descriptor
 */
#ifdef __ICCARM__
typedef struct ac_fu_t
{
    __packed uint8_t  bLength;
    __packed uint8_t  bDescriptorType;
    __packed uint8_t  bDescriptorSubtype;
    __packed uint8_t  bUnitID;              /* Constant uniquely identifying the Unit within the audio function. */
    __packed uint8_t  bSourceID;            /* ID of the Unit or Terminal to which this Feature Unit is connected. */
    __packed uint8_t  bControlSize;         /* Size in bytes of an element of the bmaControls() array: n */
} AC_FU_T;
#else
typedef struct __attribute__((__packed__)) ac_fu_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bUnitID;                       /* Constant uniquely identifying the Unit within the audio function. */
    uint8_t  bSourceID;                     /* ID of the Unit or Terminal to which this Feature Unit is connected. */
    uint8_t  bControlSize;                  /* Size in bytes of an element of the bmaControls() array: n */
} AC_FU_T;
#endif

/* Feature Unit Control Selectors */
#define FU_CONTROL_UNDEFINED          0x00
#define MUTE_CONTROL                  0x01  /* Feature Unit Descriptor bmaControls bit 0  */
#define VOLUME_CONTROL                0x02  /* Feature Unit Descriptor bmaControls bit 1  */
#define BASS_CONTROL                  0x03  /* Feature Unit Descriptor bmaControls bit 2  */
#define MID_CONTROL                   0x04
#define TREBLE_CONTROL                0x05
#define GRAPHIC_EQUALIZER_CONTROL     0x06
#define AUTOMATIC_GAIN_CONTROL        0x07
#define DELAY_CONTROL                 0x08
#define BASS_BOOST_CONTROL            0x09
#define LOUDNESS_CONTROL              0x0A

/*-----------------------------------------------------------------------------------
 *  UAC AS Isochronous Audio Data Endpoint Descriptor
 */
#ifdef __ICCARM__
typedef struct as_gen_t
{
    __packed uint8_t  bLength;
    __packed uint8_t  bDescriptorType;
    __packed uint8_t  bDescriptorSubtype;
    __packed uint8_t  bTerminalLink;
    __packed uint8_t  bDelay;
    __packed uint16_t wFormatTag;
} AS_GEN_T;
#else
typedef struct __attribute__((__packed__)) as_gen_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bTerminalLink;
    uint8_t  bDelay;
    uint16_t wFormatTag;
} AS_GEN_T;
#endif

/*-----------------------------------------------------------------------------------
 *  UAC Processing Unit Descriptor
 */
#ifdef __ICCARM__
typedef struct ac_pu_t
{
    __packed uint8_t  bLength;
    __packed uint8_t  bDescriptorType;
    __packed uint8_t  bDescriptorSubtype;
    __packed uint8_t  bUnitID;              /* Constant uniquely identifying the Unit within the audio function. */
    __packed uint16_t wProcessType;         /* Constant identifying the type of processing this Unit is performing. */
    __packed uint8_t  bNrInPins;            /* Number of Input Pins of this Unit: p */
} AC_PU_T;
#else
typedef struct __attribute__((__packed__)) ac_pu_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bUnitID;                       /* Constant uniquely identifying the Unit within the audio function. */
    uint16_t wProcessType;                  /* Constant identifying the type of processing this Unit is performing. */
    uint8_t  bNrInPins;                     /* Number of Input Pins of this Unit: p */
} AC_PU_T;
#endif

/*-----------------------------------------------------------------------------------
 *  Class-Specific AS Isochronous Audio Data Endpoint Descriptor
 */
#ifdef __ICCARM__
typedef struct cs_ep_t
{
    __packed uint8_t  bLength;
    __packed uint8_t  bDescriptorType;
    __packed uint8_t  bDescriptorSubtype;
    __packed uint8_t  bmAttributes;
    __packed uint8_t  bLockDelayUnits;
    __packed uint16_t wLockDelay;
} CS_EP_T;
#else
typedef struct __attribute__((__packed__)) cs_ep_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bmAttributes;
    uint8_t  bLockDelayUnits;
    uint16_t wLockDelay;
} CS_EP_T;
#endif

/*-----------------------------------------------------------------------------------
 *  UAC Type I Format Type Descriptor
 */
#ifdef __ICCARM__
typedef struct ac_ft1_t
{
    __packed uint8_t  bLength;
    __packed uint8_t  bDescriptorType;
    __packed uint8_t  bDescriptorSubtype;
    __packed uint8_t  bFormatType;
    __packed uint8_t  bNrChannels;
    __packed uint8_t  bSubframeSize;
    __packed uint8_t  bBitResolution;
    __packed uint8_t  bSamFreqType;
    __packed uint8_t  tSamFreq[16][3];
} AS_FT1_T;
#else
typedef struct __attribute__((__packed__)) ac_ft1_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bFormatType;
    uint8_t  bNrChannels;
    uint8_t  bSubframeSize;
    uint8_t  bBitResolution;
    uint8_t  bSamFreqType;
    uint8_t  tSamFreq[16][3];
} AS_FT1_T;
#endif

/*-----------------------------------------------------------------------------------
 *  UAC Type 2 Format Type Descriptor
 */
#ifdef __ICCARM__
typedef struct ac_ft2_t
{
    __packed uint8_t  bLength;
    __packed uint8_t  bDescriptorType;
    __packed uint8_t  bDescriptorSubtype;
    __packed uint8_t  bFormatType;
    __packed uint16_t wMaxBitRate;
    __packed uint16_t wSamplesPerFrame;
    __packed uint8_t  bSamFreqType;
} AS_FT2_T;
#else
typedef struct __attribute__((__packed__)) ac_ft2_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bFormatType;
    uint16_t wMaxBitRate;
    uint16_t wSamplesPerFrame;
    uint8_t  bSamFreqType;
} AS_FT2_T;
#endif

/*-----------------------------------------------------------------------------------
 *  UAC Type 3 Format Type Descriptor
 */
#ifdef __ICCARM__
typedef struct ac_ft3_t
{
    __packed uint8_t  bLength;
    __packed uint8_t  bDescriptorType;
    __packed uint8_t  bDescriptorSubtype;
    __packed uint8_t  bFormatType;
    __packed uint8_t  bNrChannels;
    __packed uint8_t  bSubframeSize;
    __packed uint8_t  bBitResolution;
    __packed uint8_t  bSamFreqType;
    __packed uint8_t  tLowerSamFreq[3];
    __packed uint8_t  tUpperSamFreq[3];
} AS_FT3_T;
#else
typedef struct __attribute__((__packed__)) ac_ft3_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bFormatType;
    uint8_t  bNrChannels;
    uint8_t  bSubframeSize;
    uint8_t  bBitResolution;
    uint8_t  bSamFreqType;
    uint8_t  tLowerSamFreq[3];
    uint8_t  tUpperSamFreq[3];
} AS_FT3_T;
#endif

/*====================================================================================*/
/*  UAC 2.0 Descriptor Structures                                                     */
/*====================================================================================*/

/*-----------------------------------------------------------------------------------
 *  UAC 2.0 Class-Specific AC Interface Header Descriptor (4.7.2, UAC2)
 */
#ifdef __ICCARM__
typedef struct ac2_if_header
{
    __packed uint8_t  bLength;
    __packed uint8_t  bDescriptorType;      /* CS_INTERFACE                               */
    __packed uint8_t  bDescriptorSubtype;   /* HEADER                                     */
    __packed uint16_t bcdADC;               /* 0x0200 for UAC 2.0                         */
    __packed uint8_t  bCategory;            /* Primary use of the audio function           */
    __packed uint16_t wTotalLength;         /* Total number of bytes                       */
    __packed uint8_t  bmControls;           /* D1..0: Latency Control                     */
} AC2_IF_HDR_T;
#else
typedef struct __attribute__((__packed__)) ac2_if_header
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint16_t bcdADC;
    uint8_t  bCategory;
    uint16_t wTotalLength;
    uint8_t  bmControls;
} AC2_IF_HDR_T;
#endif

/*-----------------------------------------------------------------------------------
 *  UAC 2.0 Clock Source Descriptor (4.7.2.1, UAC2)
 */
#ifdef __ICCARM__
typedef struct ac2_clk_src_t
{
    __packed uint8_t  bLength;
    __packed uint8_t  bDescriptorType;      /* CS_INTERFACE                               */
    __packed uint8_t  bDescriptorSubtype;   /* CLOCK_SOURCE (0x0A)                        */
    __packed uint8_t  bClockID;             /* Unique Clock Source ID                      */
    __packed uint8_t  bmAttributes;         /* D1..0: Clock type
                                               00: External Clock
                                               01: Internal fixed Clock
                                               10: Internal variable Clock
                                               11: Internal programmable Clock
                                               D2: Clock synchronized to SOF             */
    __packed uint8_t  bmControls;           /* D1..0: Clock Frequency Control
                                               D3..2: Clock Validity Control             */
    __packed uint8_t  bAssocTerminal;       /* Terminal ID of associated Terminal          */
    __packed uint8_t  iClockSource;         /* String descriptor index                     */
} AC2_CLK_SRC_T;
#else
typedef struct __attribute__((__packed__)) ac2_clk_src_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bClockID;
    uint8_t  bmAttributes;
    uint8_t  bmControls;
    uint8_t  bAssocTerminal;
    uint8_t  iClockSource;
} AC2_CLK_SRC_T;
#endif

/*-----------------------------------------------------------------------------------
 *  UAC 2.0 Clock Selector Descriptor (4.7.2.2, UAC2)
 */
#ifdef __ICCARM__
typedef struct ac2_clk_sel_t
{
    __packed uint8_t  bLength;
    __packed uint8_t  bDescriptorType;      /* CS_INTERFACE                               */
    __packed uint8_t  bDescriptorSubtype;   /* CLOCK_SELECTOR (0x0B)                      */
    __packed uint8_t  bClockID;             /* Unique Clock Selector ID                    */
    __packed uint8_t  bNrInPins;            /* Number of Input Pins                        */
    __packed uint8_t  baCSourceID[1];       /* Variable length: ID of Clock Entities       */
} AC2_CLK_SEL_T;
#else
typedef struct __attribute__((__packed__)) ac2_clk_sel_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bClockID;
    uint8_t  bNrInPins;
    uint8_t  baCSourceID[1];
} AC2_CLK_SEL_T;
#endif

/*-----------------------------------------------------------------------------------
 *  UAC 2.0 Clock Multiplier Descriptor (4.7.2.3, UAC2)
 */
#ifdef __ICCARM__
typedef struct ac2_clk_mul_t
{
    __packed uint8_t  bLength;
    __packed uint8_t  bDescriptorType;      /* CS_INTERFACE                               */
    __packed uint8_t  bDescriptorSubtype;   /* CLOCK_MULTIPLIER (0x0C)                    */
    __packed uint8_t  bClockID;             /* Unique Clock Multiplier ID                  */
    __packed uint8_t  bCSourceID;           /* ID of Clock Entity                          */
    __packed uint8_t  bmControls;           /* D1..0: Clock Numerator Control
                                               D3..2: Clock Denominator Control           */
    __packed uint8_t  iClockMultiplier;     /* String descriptor index                     */
} AC2_CLK_MUL_T;
#else
typedef struct __attribute__((__packed__)) ac2_clk_mul_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bClockID;
    uint8_t  bCSourceID;
    uint8_t  bmControls;
    uint8_t  iClockMultiplier;
} AC2_CLK_MUL_T;
#endif

/*-----------------------------------------------------------------------------------
 *  UAC 2.0 Input Terminal Descriptor (4.7.2.4, UAC2)
 */
#ifdef __ICCARM__
typedef struct ac2_itd_t
{
    __packed uint8_t  bLength;
    __packed uint8_t  bDescriptorType;      /* CS_INTERFACE                               */
    __packed uint8_t  bDescriptorSubtype;   /* INPUT_TERMINAL                             */
    __packed uint8_t  bTerminalID;
    __packed uint16_t wTerminalType;
    __packed uint8_t  bAssocTerminal;
    __packed uint8_t  bCSourceID;           /* ID of the Clock Entity                      */
    __packed uint8_t  bNrChannels;
    __packed uint32_t bmChannelConfig;      /* Spatial location of logical channels        */
    __packed uint8_t  iChannelNames;
    __packed uint16_t bmControls;           /* D1..0: Copy Protect Control
                                               D3..2: Connector Control
                                               D5..4: Overload Control
                                               D7..6: Cluster Control
                                               D9..8: Underflow Control
                                               D11..10: Overflow Control                 */
    __packed uint8_t  iTerminal;
} AC2_IT_T;
#else
typedef struct __attribute__((__packed__)) ac2_itd_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bTerminalID;
    uint16_t wTerminalType;
    uint8_t  bAssocTerminal;
    uint8_t  bCSourceID;
    uint8_t  bNrChannels;
    uint32_t bmChannelConfig;
    uint8_t  iChannelNames;
    uint16_t bmControls;
    uint8_t  iTerminal;
} AC2_IT_T;
#endif

/*-----------------------------------------------------------------------------------
 *  UAC 2.0 Output Terminal Descriptor (4.7.2.5, UAC2)
 */
#ifdef __ICCARM__
typedef struct ac2_otd_t
{
    __packed uint8_t  bLength;
    __packed uint8_t  bDescriptorType;      /* CS_INTERFACE                               */
    __packed uint8_t  bDescriptorSubtype;   /* OUTPUT_TERMINAL                            */
    __packed uint8_t  bTerminalID;
    __packed uint16_t wTerminalType;
    __packed uint8_t  bAssocTerminal;
    __packed uint8_t  bSourceID;
    __packed uint8_t  bCSourceID;           /* ID of the Clock Entity                      */
    __packed uint16_t bmControls;           /* D1..0: Copy Protect Control
                                               D3..2: Connector Control
                                               D5..4: Overload Control
                                               D7..6: Underflow Control
                                               D9..8: Overflow Control                   */
    __packed uint8_t  iTerminal;
} AC2_OT_T;
#else
typedef struct __attribute__((__packed__)) ac2_otd_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bTerminalID;
    uint16_t wTerminalType;
    uint8_t  bAssocTerminal;
    uint8_t  bSourceID;
    uint8_t  bCSourceID;
    uint16_t bmControls;
    uint8_t  iTerminal;
} AC2_OT_T;
#endif

/*-----------------------------------------------------------------------------------
 *  UAC 2.0 Feature Unit Descriptor (4.7.2.8, UAC2)
 *  Note: bmaControls is 4 bytes per channel in UAC 2.0 (vs variable in UAC 1.0)
 */
#ifdef __ICCARM__
typedef struct ac2_fu_t
{
    __packed uint8_t  bLength;
    __packed uint8_t  bDescriptorType;      /* CS_INTERFACE                               */
    __packed uint8_t  bDescriptorSubtype;   /* FEATURE_UNIT                               */
    __packed uint8_t  bUnitID;
    __packed uint8_t  bSourceID;
    __packed uint32_t bmaControls[1];       /* Variable length: 4 bytes per channel
                                               bmaControls[0] = master channel
                                               D1..0: Mute Control
                                               D3..2: Volume Control
                                               D5..4: Bass Control
                                               ...                                       */
} AC2_FU_T;
#else
typedef struct __attribute__((__packed__)) ac2_fu_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bUnitID;
    uint8_t  bSourceID;
    uint32_t bmaControls[1];
} AC2_FU_T;
#endif

/*-----------------------------------------------------------------------------------
 *  UAC 2.0 Class-Specific AS Interface Descriptor (4.9.2, UAC2)
 */
#ifdef __ICCARM__
typedef struct as2_gen_t
{
    __packed uint8_t  bLength;
    __packed uint8_t  bDescriptorType;      /* CS_INTERFACE                               */
    __packed uint8_t  bDescriptorSubtype;   /* AS_GENERAL                                 */
    __packed uint8_t  bTerminalLink;        /* Terminal ID of associated Terminal           */
    __packed uint8_t  bmControls;           /* D1..0: Active Alternate Setting Control
                                               D3..2: Valid Alternate Settings Control    */
    __packed uint8_t  bFormatType;          /* Format Type (from Format Type Descriptor)   */
    __packed uint32_t bmFormats;            /* Audio data formats supported                */
    __packed uint8_t  bNrChannels;          /* Number of physical channels in the cluster  */
    __packed uint32_t bmChannelConfig;      /* Spatial location of logical channels        */
    __packed uint8_t  iChannelNames;        /* String descriptor index                     */
} AS2_GEN_T;
#else
typedef struct __attribute__((__packed__)) as2_gen_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bTerminalLink;
    uint8_t  bmControls;
    uint8_t  bFormatType;
    uint32_t bmFormats;
    uint8_t  bNrChannels;
    uint32_t bmChannelConfig;
    uint8_t  iChannelNames;
} AS2_GEN_T;
#endif

/*-----------------------------------------------------------------------------------
 *  UAC 2.0 Type I Format Type Descriptor (2.3.1.6, Format Type)
 *  Note: UAC 2.0 does not embed sampling rates; they are managed by Clock Source.
 */
#ifdef __ICCARM__
typedef struct ac2_ft1_t
{
    __packed uint8_t  bLength;              /* 6                                           */
    __packed uint8_t  bDescriptorType;      /* CS_INTERFACE                               */
    __packed uint8_t  bDescriptorSubtype;   /* FORMAT_TYPE                                */
    __packed uint8_t  bFormatType;          /* FORMAT_TYPE_I                              */
    __packed uint8_t  bSubslotSize;         /* Bytes per subslot (1, 2, 3, or 4)          */
    __packed uint8_t  bBitResolution;       /* Effectively used bits in the subslot        */
} AS2_FT1_T;
#else
typedef struct __attribute__((__packed__)) ac2_ft1_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bFormatType;
    uint8_t  bSubslotSize;
    uint8_t  bBitResolution;
} AS2_FT1_T;
#endif

/*-----------------------------------------------------------------------------------
 *  UAC 2.0 Class-Specific AS Isochronous Audio Data Endpoint Descriptor (4.10.1.2, UAC2)
 */
#ifdef __ICCARM__
typedef struct cs2_ep_t
{
    __packed uint8_t  bLength;              /* 8                                           */
    __packed uint8_t  bDescriptorType;      /* CS_ENDPOINT                                */
    __packed uint8_t  bDescriptorSubtype;   /* EP_GENERAL                                 */
    __packed uint8_t  bmAttributes;         /* D0: Sampling Frequency
                                               D1: Pitch
                                               D7: MaxPacketsOnly                         */
    __packed uint8_t  bmControls;           /* D1..0: Pitch Control
                                               D3..2: Data Overrun Control
                                               D5..4: Data Underrun Control               */
    __packed uint8_t  bLockDelayUnits;      /* 0: Undefined, 1: Milliseconds, 2: PCM samples */
    __packed uint16_t wLockDelay;
} CS2_EP_T;
#else
typedef struct __attribute__((__packed__)) cs2_ep_t
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;
    uint8_t  bmAttributes;
    uint8_t  bmControls;
    uint8_t  bLockDelayUnits;
    uint16_t wLockDelay;
} CS2_EP_T;
#endif

/*-----------------------------------------------------------------------------------
 *  UAC 2.0 Frequency Range layout (for RANGE request response)
 */
#ifdef __ICCARM__
typedef struct uac2_freq_subrange_t
{
    __packed uint32_t dMIN;                 /* Minimum sampling frequency                  */
    __packed uint32_t dMAX;                 /* Maximum sampling frequency                  */
    __packed uint32_t dRES;                 /* Sampling frequency resolution               */
} UAC2_FREQ_SUBRANGE_T;
#else
typedef struct __attribute__((__packed__)) uac2_freq_subrange_t
{
    uint32_t dMIN;
    uint32_t dMAX;
    uint32_t dRES;
} UAC2_FREQ_SUBRANGE_T;
#endif

/// @endcond HIDDEN_SYMBOLS

#endif /* __INCLUDED_UAC_H__ */
