/**************************************************************************//**
 * @file     descriptors.c
 * @version  V3.00
 * @brief    HSUSBD descriptor.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*!<Includes */
#include "NuMicro.h"
#include "usbd_audio.h"

/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
uint8_t gu8DeviceDescriptor[] =
{
    LEN_DEVICE,     /* bLength */
    DESC_DEVICE,    /* bDescriptorType */
    0x00, 0x02,     /* bcdUSB */
    0x00,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    CEP_MAX_PKT_SIZE,   /* bMaxPacketSize0 */
    /* idVendor */
    USBD_VID & 0x00FF,
    ((USBD_VID & 0xFF00) >> 8),
    /* idProduct */
    USBD_PID & 0x00FF,
    ((USBD_PID & 0xFF00) >> 8),
    0x00, 0x00,     /* bcdDevice */
    0x01,           /* iManufacture */
    0x02,           /* iProduct */
    0x00,           /* iSerialNumber - no serial */
    0x01            /* bNumConfigurations */
};

/*!<USB Qualifier Descriptor */
uint8_t gu8QualifierDescriptor[] =
{
    LEN_QUALIFIER,  /* bLength */
    DESC_QUALIFIER, /* bDescriptorType */
    0x00, 0x02,     /* bcdUSB */
    0x00,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    CEP_MAX_PKT_SIZE, /* bMaxPacketSize0 */
    0x01,           /* bNumConfigurations */
    0x00
};

/*!<USB Configure Descriptor */
uint8_t gu8ConfigDescriptor[] =
{
    /* Configuration Descriptor */
    0x09,               /* Config Descriptor Length */
    0x02,               /* DescriptorType: CONFIG */
    0xFE, 0x00,         /* wTotalLength
                           Configuration Descriptor                      (0x09)
                           Standard Interface Association Descriptor     (0x08)
                           Standard AC Interface Descriptor              (0x09)
                           Class-Specific AC Interface Header Descriptor (0x09)
                           Clock Source Descriptor                       (0x08)
                           Microphone - Audio Control
                             Input Terminal Descriptor                   (0x11)
                             Feature Unit Descriptor                     (0x12)
                             Output Terminal Descriptor                  (0x0C)
                           Speaker - Audio Control
                             Input Terminal Descriptor                   (0x11)
                             Feature Unit Descriptor                     (0x12)
                             Output Terminal Descriptor                  (0x0C)
                           Microphone - Interface alternate 0
                             Standard AS interface                       (0x09)
                           Microphone - Interface alternate 1~2
                             Standard AC Interface Descriptor                             (0x09,0x09)
                             Class-Specific AS Interface Descriptor                       (0x10,0x10)
                             Audio Format Type Descriptor                                 (0x06,0x06)
                             Standard AS Isochronous Feedback Endpoint Descriptor         (0x07,0x07)
                             Class-Specific AS Isochronous Audio Data Endpoint Descriptor (0x08,0x08)
                             *Each Interface alternate Summary                            (0x2E,0x2E)
                           Speaker - Interface alternate 0
                             Standard AS interface                       (0x09)
                           Speaker - Interface alternate 1~2
                             Standard AC Interface Descriptor                             (0x09,0x09)
                             Class-Specific AS Interface Descriptor                       (0x10,0x10)
                             Audio Format Type Descriptor                                 (0x06,0x06)
                             Standard AS Isochronous Feedback Endpoint Descriptor         (0x07,0x07)
                             Class-Specific AS Isochronous Audio Data Endpoint Descriptor (0x08,0x08)
                             AS Isochronous Feedback Endpoint Descriptor                  (0x07,0x07)
                             *Each Interface alternate Summary                            (0x35,0x35)

                           0x09 + 0x08 + 0x9 + 0x09 + 0x08 + (0x11 + 0x12 + 0x0C) + (0x11 + 0x12 + 0x0C) +
                           0x09 + 2 * 0x2E +
                           0x09 + 2 * 0x35 = 0x161
                        */
    0x03,               /* bNumInterfaces - Interface 0, Interface 1 (Speaker), Interface 2 (Microphone) */
    0x01,               /* bConfigurationValue */
    0x00,               /* iConfiguration */
    0x80,               /* bmAttributes */
    0x32,               /* bMaxPower */

    /* Standard Interface Association Descriptor */
    0x08,               /* bLength(0x08) */
    0x0B,               /* bDescriptorType(0x0B) */
    0x00,               /* bFirstInterface(0x00) */
    0x03,               /* bInterfaceCount(0x03) */
    0x01,               /* bFunctionClass(0x01): AUDIO */
    0x00,               /* bFunctionSubClass(0x00) */
    0x20,               /* bFunctionProtocol(0x2000): 2.0 AF_VERSION_02_00 */
    0x00,               /* iFunction(0x00) */

    /* Standard AC Interface Descriptor */
    0x09,               /* bLength */
    0x04,               /* bDescriptorType */
    0x00,               /* bInterfaceNumber */
    0x00,               /* bAlternateSetting */
    0x00,               /* bNumEndpoints */
    0x01,               /* bInterfaceClass:AUDIO */
    0x01,               /* bInterfaceSubClass:AUDIOCONTROL */
    0x20,               /* bInterfaceProtocol */
    0x02,               /* iInterface */

    /* Class-Specific AC Interface Header Descriptor */
    0x09,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x01,               /* bDescriptorSubType:HEADER */
    0x00, 0x02,         /* bcdADC:2.0 */
    0x08,               /* UAC_FUNCTION_IO_BOX */
    0x6F, 0x00,         /* wTotalLength
                           Class-Specific AC Interface Header Descriptor (0x09)
                           Clock Source Descriptor                       (0x08)
                           Speaker - Audio Control
                             Input Terminal Descriptor                   (0x11)
                             Feature Unit Descriptor                     (0x12)
                             Output Terminal Descriptor                  (0x0C)
                           Microphone - Audio Control
                             Input Terminal Descriptor                   (0x11)
                             Feature Unit Descriptor                     (0x12)
                             Output Terminal Descriptor                  (0x0C)

                           0x09 + 0x08 + (0x11 + 0x12 + 0x0C) + (0x11 + 0x12 + 0x0C) = 0x6F
                        */
    0x00,               /* bmControls(0b00000000) - D1..0: Latency Control */

    /* Clock Source Descriptor (bClockID 0x10) */
    0x08,               /* bLength(0x08) */
    0x24,               /* bDescriptorType(0x24): CS_INTERFACE */
    0x0A,               /* bDescriptorSubType(0x0A): CLOCK_SOURCE */
    CLOCK_SOURCE_ID,    /* bClockID(0x10): CLOCK_SOURCE_ID */
    0x03,               /* bmAttributes */
    0x07,               /* bmControls(0x07):
                           clock frequency control: 0b11 - host programmable;
                           clock validity control: 0b01 - host read only */
    0x00,               /* bAssocTerminal(0x00) */
    0x00,               /* iClockSource */

    /* Input Terminal Descriptor (Terminal ID 0x04 - Source ID 0x10) */
    0x11,                                   /* bLength*/
    0x24,                                   /* bDescriptorType:CS_INTERFACE*/
    0x02,                                   /* bDescriptorSubType:INPUT_TERMINAL*/
    0x04,                                   /* bTerminalID*/
    0x02, 0x04,                             /* wTerminalType: HEADSET */
    0x00,                                   /* bAssocTerminal*/
    0x10,                                   /* bSourceID*/
    REC_CHANNELS,                           /* bNrChannels*/
    REC_CH_CFG, 0x00, 0x00, 0x00,           /* wChannelConfig
                                                 Bit 0: Front Left - FL
                                                 Bit 1: Front Right - FR
                                                 Bit 2: Front Center - FC
                                                 Bit 3: Low Frequency Effects - LFE
                                                 Bit 4: Back Left - BL
                                                 Bit 5: Back Right - BR
                                                 Bit 6: Front Left of Center - FLC
                                                 Bit 7: FRONT RIght of Center - FRC
                                                 Bit 8: Back Center - BC
                                                 Bit 9: Side Left - SL
                                                 Bit 10: Side Right - SR
                                                 Bit 11: Top Center - TC
                                                 Bit 12: Top Front Left - TFL
                                                 Bit 13: Top Front Center - TFC
                                                 Bit 14: Top Front Right - TFR
                                                 Bit 15: Top Back Left - TBL
                                                 Bit 16: Top Back Center - TBC
                                                 Bit 17: Top Back Right - TBR
                                                 Bit 18: Top Front Left of Center - TFLC
                                                 Bit 19: Top Front Right of Center - TFRC
                                                 Bit 20: Left Low Frequency Effects - LLFE
                                                 Bit 21: Right Low Frequency Effects - RLFE
                                                 Bit 22: Top Side Left - TSL
                                                 Bit 23: Top Side Right - TSR
                                                 Bit 24: Bottom Center - BC
                                                 Bit 25: Back Left of Center - BLC
                                                 Bit 26: Back Right of Center - BRC
                                                 Bit 31: Raw Data - RD; Mutually exclusive with all other spatial locations
                                            */
    0x00,                                   /* iChannelNames */
    0x00, 0x00,                             /* bmcontrols
                                                 D1..0: Copy Protect Control
                                                 D3..2: Connector Control
                                                 D5..4: Overload Control
                                                 D7..6: Cluster Control
                                                 D9..8: Underflow Control
                                                 D11..10: Overflow Control
                                                 D15..12: Reserved, should set to 0
                                            */
    0x00,                                   /* iTerminal */

    /* Feature Unit Descriptor (Unit ID 0x05 - Source ID 0x4) */
    0x12,                                   /* bLength */
    0x24,                                   /* bDescriptorType */
    0x06,                                   /* bDescriptorSubType */
    REC_FEATURE_UNITID,                     /* bUnitID */
    0x04,                                   /* bSourceID */
    0x00, 0x00, 0x00, 0x00,                 /* bmaControls(0)
                                               Master control
                                                D1..0: Mute Control
                                                D3..2: Volume Control
                                                D5..4: Bass Control
                                                D7..6: Mid Control
                                                D9..8: Treble Control
                                                D11..10: Graphic Equalizer Control
                                                D13..12: Automatic Gain Control
                                                D15..14: Delay Control
                                                D17..16: Bass Control
                                                D19..18: Loudness Control
                                                D21..20: Input Gain Control
                                                D23..22: Input Gain Pad Control
                                                D25..24: Phase Inverter Control
                                                D27..26: Underflow Control
                                                D29..28: Overflow Control
                                                D31..30: Reserved, should set to 0
                                            */
    0x00, 0x00, 0x00, 0x00,                 /* bmaControls(1)
                                               Left volume control
                                                D1..0: Mute Control
                                                D3..2: Volume Control
                                                D5..4: Bass Control
                                                D7..6: Mid Control
                                                D9..8: Treble Control
                                                D11..10: Graphic Equalizer Control
                                                D13..12: Automatic Gain Control
                                                D15..14: Delay Control
                                                D17..16: Bass Control
                                                D19..18: Loudness Control
                                                D21..20: Input Gain Control
                                                D23..22: Input Gain Pad Control
                                                D25..24: Phase Inverter Control
                                                D27..26: Underflow Control
                                                D29..28: Overflow Control
                                                D31..30: Reserved, should set to 0
                                            */
    0x00, 0x00, 0x00, 0x00,                 /* bmaControls(2)
                                               Right volume control
                                                D1..0: Mute Control
                                                D3..2: Volume Control
                                                D5..4: Bass Control
                                                D7..6: Mid Control
                                                D9..8: Treble Control
                                                D11..10: Graphic Equalizer Control
                                                D13..12: Automatic Gain Control
                                                D15..14: Delay Control
                                                D17..16: Bass Control
                                                D19..18: Loudness Control
                                                D21..20: Input Gain Control
                                                D23..22: Input Gain Pad Control
                                                D25..24: Phase Inverter Control
                                                D27..26: Underflow Control
                                                D29..28: Overflow Control
                                                D31..30: Reserved, should set to 0
                                            */
    0x00,                                   /* iFeature */

    /* Output Terminal Descriptor (Terminal ID 0x02 - Source ID 0x5 - Clock Source ID 0x10) */
    0x0C,               /* bLength*/
    0x24,               /* bDescriptorType:CS_INTERFACE*/
    0x03,               /* bDescriptorSubType:OUTPUT_TERMINAL*/
    0x02,               /* bTerminalID*/
    0x01, 0x01,         /* wTerminalType: 0x0101 */
    0x00,               /* bAssocTerminal*/
    REC_FEATURE_UNITID, /* bSourceID*/
    0x10,               /* bCSourceID*/
    0x00, 0x00,         /* bmControls
                             D1..0: Copy Protect Control
                             D3..2: Connector Control
                             D5..4: Overload Control
                             D7..6: Cluster Control
                             D9..8: Underflow Control
                             D11..10: Overflow Control
                             D15..12: Reserved, should set to 0
                        */
    0x00,               /* iTerminal*/
    /* Input Terminal Descriptor (Terminal ID 1 - Source ID 0x10) */
    0x11,                                   /* bLength */
    0x24,                                   /* bDescriptorType:CS_INTERFACE */
    0x02,                                   /* bDescriptorSubType:INPUT_TERMINAL */
    0x01,                                   /* bTerminalID */
    0x01, 0x01,                             /* wTerminalType: 0x0101 usb streaming */
    0x00,                                   /* bAssocTerminal */
    CLOCK_SOURCE_ID,                        /* bCSourceID(0x10): CLOCK_SOURCE_ID */
    PLAY_CHANNELS,                          /* bNrChannels - */
    PLAY_CH_CFG, 0x00, 0x00, 0x00,          /* bmChannelConfig
                                                 Bit 0: Front Left - FL
                                                 Bit 1: Front Right - FR
                                                 Bit 2: Front Center - FC
                                                 Bit 3: Low Frequency Effects - LFE
                                                 Bit 4: Back Left - BL
                                                 Bit 5: Back Right - BR
                                                 Bit 6: Front Left of Center - FLC
                                                 Bit 7: FRONT RIght of Center - FRC
                                                 Bit 8: Back Center - BC
                                                 Bit 9: Side Left - SL
                                                 Bit 10: Side Right - SR
                                                 Bit 11: Top Center - TC
                                                 Bit 12: Top Front Left - TFL
                                                 Bit 13: Top Front Center - TFC
                                                 Bit 14: Top Front Right - TFR
                                                 Bit 15: Top Back Left - TBL
                                                 Bit 16: Top Back Center - TBC
                                                 Bit 17: Top Back Right - TBR
                                                 Bit 18: Top Front Left of Center - TFLC
                                                 Bit 19: Top Front Right of Center - TFRC
                                                 Bit 20: Left Low Frequency Effects - LLFE
                                                 Bit 21: Right Low Frequency Effects - RLFE
                                                 Bit 22: Top Side Left - TSL
                                                 Bit 23: Top Side Right - TSR
                                                 Bit 24: Bottom Center - BC
                                                 Bit 25: Back Left of Center - BLC
                                                 Bit 26: Back Right of Center - BRC
                                                 Bit 31: Raw Data - RD; Mutually exclusive with all other spatial locations
                                            */
    0x00,                                   /* iChannelNames */
    0x00, 0x00,                             /* bmcontrols
                                                 D1..0: Copy Protect Control
                                                 D3..2: Connector Control
                                                 D5..4: Overload Control
                                                 D7..6: Cluster Control
                                                 D9..8: Underflow Control
                                                 D11..10: Overflow Control
                                                 D15..12: Reserved, should set to 0
                                            */
    0x00,                                   /* iTerminal */

    /* Feature Unit Descriptor (Unit ID 0x06 - Source ID 0x1) */
    0x12,                                   /* bLength */
    0x24,                                   /* bDescriptorType */
    0x06,                                   /* bDescriptorSubType */
    PLAY_FEATURE_UNITID,                    /* bUnitID */
    0x01,                                   /* bSourceID */
    0x00, 0x00, 0x00, 0x00,                 /* bmaControls(0)
                                               Master control
                                                D1..0: Mute Control
                                                D3..2: Volume Control
                                                D5..4: Bass Control
                                                D7..6: Mid Control
                                                D9..8: Treble Control
                                                D11..10: Graphic Equalizer Control
                                                D13..12: Automatic Gain Control
                                                D15..14: Delay Control
                                                D17..16: Bass Control
                                                D19..18: Loudness Control
                                                D21..20: Input Gain Control
                                                D23..22: Input Gain Pad Control
                                                D25..24: Phase Inverter Control
                                                D27..26: Underflow Control
                                                D29..28: Overflow Control
                                                D31..30: Reserved, should set to 0
                                            */
    0x00, 0x00, 0x00, 0x00,                 /* bmaControls(1)
                                               Left volume control
                                                D1..0: Mute Control
                                                D3..2: Volume Control
                                                D5..4: Bass Control
                                                D7..6: Mid Control
                                                D9..8: Treble Control
                                                D11..10: Graphic Equalizer Control
                                                D13..12: Automatic Gain Control
                                                D15..14: Delay Control
                                                D17..16: Bass Control
                                                D19..18: Loudness Control
                                                D21..20: Input Gain Control
                                                D23..22: Input Gain Pad Control
                                                D25..24: Phase Inverter Control
                                                D27..26: Underflow Control
                                                D29..28: Overflow Control
                                                D31..30: Reserved, should set to 0
                                            */
    0x00, 0x00, 0x00, 0x00,                 /* bmaControls(2)
                                               Right volume control
                                                D1..0: Mute Control
                                                D3..2: Volume Control
                                                D5..4: Bass Control
                                                D7..6: Mid Control
                                                D9..8: Treble Control
                                                D11..10: Graphic Equalizer Control
                                                D13..12: Automatic Gain Control
                                                D15..14: Delay Control
                                                D17..16: Bass Control
                                                D19..18: Loudness Control
                                                D21..20: Input Gain Control
                                                D23..22: Input Gain Pad Control
                                                D25..24: Phase Inverter Control
                                                D27..26: Underflow Control
                                                D29..28: Overflow Control
                                                D31..30: Reserved, should set to 0
                                            */
    0x00,                                   /* iFeature */

    /* Output Terminal Descriptor (Terminal ID 0x03 - Source ID 0x6 - Clock Source ID 0x10) */
    0x0C,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x03,               /* bDescriptorSubType:OUTPUT_TERMINAL */
    0x03,               /* bTerminalID */
    0x02, 0x04,         /* wTerminalType: HEADSET */
    0x00,               /* bAssocTerminal */
    PLAY_FEATURE_UNITID,/* bSourceID */
    0x10,               /* bCSourceID */
    0x00, 0x00,         /* bmControls
                             D1..0: Copy Protect Control
                             D3..2: Connector Control
                             D5..4: Overload Control
                             D7..6: Cluster Control
                             D9..8: Underflow Control
                             D11..10: Overflow Control
                             D15..12: Reserved, should set to 0
                        */
    0x00,               /* iTerminal */
    /* Standard AC Interface Descriptor - Interface 1, alternate 0 */
    0x09,               /* bLength */
    0x04,               /* bDescriptorType */
    0x01,               /* bInterfaceNumber */
    0x00,               /* bAlternateSetting */
    0x00,               /* bNumEndpoints */
    0x01,               /* bInterfaceClass:AUDIO */
    0x02,               /* bInterfaceSubClass:AUDIOSTREAMING */
    0x20,               /* bInterfaceProtocol */
    0x00,               /* iInterface */

    /* Standard AC Interface Descriptor - Interface 1, alternate 1 */
    0x09,               /* bLength */
    0x04,               /* bDescriptorType */
    0x01,               /* bInterfaceNumber */
    0x01,               /* bAlternateSetting */
    0x01,               /* bNumEndpoints */
    0x01,               /* bInterfaceClass:AUDIO */
    0x02,               /* bInterfaceSubClass:AUDIOSTREAMING */
    0x20,               /* bInterfaceProtocol */
    0x00,               /* iInterface */

    /* Class-Specific AS Interface Descriptor (this interface's endpoint connect to Terminal ID 0x02 - Microphone) */
    0x10,                                   /* bLength(16) */
    0x24,                                   /* bDescriptorType(0x024): CS_INTERFACE */
    0x01,                                   /* bDescriptorSubType(0x01): AS_GENERAL */
    0x02,                                   /* bTerminalLink(0x02): INPUT_TERMINAL_ID */
    0x00,                                   /* bmControls(0x00) */
    0x01,                                   /* bFormatType(0x01): FORMAT_TYPE_I */
    0x01, 0x00, 0x00, 0x00,                 /* bmFormats(0x00000001): PCM
                                                 Bit 0: IEC61937_AC-3
                                                 Bit 1: IEC61937_MPEG-1_Layer1
                                                 Bit 2: IEC61937_MPEG-1_Layer2/3 or IEC61937__MPEG-2_NOEXT
                                                 Bit 3: IEC61937_MPEG-2_EXT
                                                 Bit 4: IEC61937_MPEG-2_AAC_ADTS
                                                 Bit 5: IEC61937_MPEG-2_Layer1_LS
                                                 Bit 6: IEC61937_MPEG-2_Layer2/3_LS
                                                 Bit 7: IEC61937_DTS-I
                                                 Bit 8: IEC61937_DTS-II
                                                 Bit 9: IEC61937_DTS-III
                                                 Bit 10: IEC61937_ATRAC
                                                 Bit 11: IEC61937_ATRAC2/3
                                                 Bit 12: TYPE_III_WMA
                                            */
    0x02,                                   /* bNrChannels(0x02): NB_CHANNELS */
    0x00, 0x00, 0x00, 0x00,                 /* bmChannelCOnfig(0x00000003)
                                                 Bit 0: Front Left - FL
                                                 Bit 1: Front Right - FR
                                                 Bit 2: Front Center - FC
                                                 Bit 3: Low Frequency Effects - LFE
                                                 Bit 4: Back Left - BL
                                                 Bit 5: Back Right - BR
                                                 Bit 6: Front Left of Center - FLC
                                                 Bit 7: FRONT RIght of Center - FRC
                                                 Bit 8: Back Center - BC
                                                 Bit 9: Side Left - SL
                                                 Bit 10: Side Right - SR
                                                 Bit 11: Top Center - TC
                                                 Bit 12: Top Front Left - TFL
                                                 Bit 13: Top Front Center - TFC
                                                 Bit 14: Top Front Right - TFR
                                                 Bit 15: Top Back Left - TBL
                                                 Bit 16: Top Back Center - TBC
                                                 Bit 17: Top Back Right - TBR
                                                 Bit 18: Top Front Left of Center - TFLC
                                                 Bit 19: Top Front Right of Center - TFRC
                                                 Bit 20: Left Low Frequency Effects - LLFE
                                                 Bit 21: Right Low Frequency Effects - RLFE
                                                 Bit 22: Top Side Left - TSL
                                                 Bit 23: Top Side Right - TSR
                                                 Bit 24: Bottom Center - BC
                                                 Bit 25: Back Left of Center - BLC
                                                 Bit 26: Back Right of Center - BRC
                                                 Bit 31: Raw Data - RD; Mutually exclusive with all other spatial locations
                                            */
    0x00,                                   /* iChannelNames(0x00): None */

    /* Audio Format Type Descriptor */
    0x06,               /* bLength(6) */
    0x24,               /* bDescriptorType(0x24): CS_INTERFACE */
    0x02,               /* bDescriptorSubtype(0x02): FORMAT_TYPE */
    0x01,               /* bFormatType(0x01): FORMAT_TYPE_I */
    0x02,               /* bSubSlotSize(0x02)  :  2 bytes per sample */
    0x10,               /* bBitResolution(0x10): 16  bits per sample */

    /* Standard AS Isochronous Feedback Endpoint Descriptor */
    0x07,                                   /* bLength */
    0x05,                                   /* bDescriptorType */
    ISO_IN_EP_NUM | EP_INPUT,               /* bEndpointAddress */
    0x25,                                   /* bmAttributes */
    (144 + 24) & 0xff, ((144 + 24) >> 8) & 0xff, /* wMaxPacketSize note */
    0x01,                                   /* bInterval - Must be 1 for compliance */

    /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor */
    0x08,               /* bLength */
    0x25,               /* bDescriptorType:CS_ENDPOINT */
    0x01,               /* bDescriptorSubType:EP_GENERAL */
    0x00,               /* bmAttributes */
    0x00,               /* bmControls */
    0x00,               /* bLockDelayUnits */
    0x00, 0x00,         /* wLockDelay */

    /* Standard AC Interface Descriptor - Interface 1/2, alternate 0
        Interface 2 for Speaker & Microphone
        Interface 1 for Speaker Only
    */
    0x09,               /* bLength */
    0x04,               /* bDescriptorType */
    0x02,               /* bInterfaceNumber */
    0x00,               /* bAlternateSetting */
    0x00,               /* bNumEndpoints */
    0x01,               /* bInterfaceClass:AUDIO */
    0x02,               /* bInterfaceSubClass:AUDIOSTREAMING */
    0x20,               /* bInterfaceProtocol */
    0x00,               /* iInterface */

    /* Standard AC Interface Descriptor - Interface 1/2, alternate 1 */
    0x09,               /* bLength */
    0x04,               /* bDescriptorType */
    0x02,               /* bInterfaceNumber */
    0x01,               /* bAlternateSetting */
    0x02,               /* bNumEndpoints */
    0x01,               /* bInterfaceClass:AUDIO */
    0x02,               /* bInterfaceSubClass:AUDIOSTREAMING */
    0x20,               /* bInterfaceProtocol */
    0x00,               /* iInterface */

    /* Class-Specific AS Interface Descriptor (this interface's endpoint connect to Terminal ID 0x01 - Speaker)*/
    0x10,                                   /* bLength(16) */
    0x24,                                   /* bDescriptorType(0x024): CS_INTERFACE */
    0x01,                                   /* bDescriptorSubType(0x01): AS_GENERAL */
    0x01,                                   /* bTerminalLink(0x01): INPUT_TERMINAL_ID */
    0x00,                                   /* bmControls(0x00) */
    0x01,                                   /* bFormatType(0x01): FORMAT_TYPE_I */
    0x01, 0x00, 0x00, 0x00,                 /* bmFormats(0x00000001): PCM
                                                 Bit 0: IEC61937_AC-3
                                                 Bit 1: IEC61937_MPEG-1_Layer1
                                                 Bit 2: IEC61937_MPEG-1_Layer2/3 or IEC61937__MPEG-2_NOEXT
                                                 Bit 3: IEC61937_MPEG-2_EXT
                                                 Bit 4: IEC61937_MPEG-2_AAC_ADTS
                                                 Bit 5: IEC61937_MPEG-2_Layer1_LS
                                                 Bit 6: IEC61937_MPEG-2_Layer2/3_LS
                                                 Bit 7: IEC61937_DTS-I
                                                 Bit 8: IEC61937_DTS-II
                                                 Bit 9: IEC61937_DTS-III
                                                 Bit 10: IEC61937_ATRAC
                                                 Bit 11: IEC61937_ATRAC2/3
                                                 Bit 12: TYPE_III_WMA
                                            */
    0x02,                                   /* bNrChannels(0x02): NB_CHANNELS */
    0x03, 0x00, 0x00, 0x00,                 /* bmChannelCOnfig(0x00000003)
                                                 Bit 0: Front Left - FL
                                                 Bit 1: Front Right - FR
                                                 Bit 2: Front Center - FC
                                                 Bit 3: Low Frequency Effects - LFE
                                                 Bit 4: Back Left - BL
                                                 Bit 5: Back Right - BR
                                                 Bit 6: Front Left of Center - FLC
                                                 Bit 7: FRONT RIght of Center - FRC
                                                 Bit 8: Back Center - BC
                                                 Bit 9: Side Left - SL
                                                 Bit 10: Side Right - SR
                                                 Bit 11: Top Center - TC
                                                 Bit 12: Top Front Left - TFL
                                                 Bit 13: Top Front Center - TFC
                                                 Bit 14: Top Front Right - TFR
                                                 Bit 15: Top Back Left - TBL
                                                 Bit 16: Top Back Center - TBC
                                                 Bit 17: Top Back Right - TBR
                                                 Bit 18: Top Front Left of Center - TFLC
                                                 Bit 19: Top Front Right of Center - TFRC
                                                 Bit 20: Left Low Frequency Effects - LLFE
                                                 Bit 21: Right Low Frequency Effects - RLFE
                                                 Bit 22: Top Side Left - TSL
                                                 Bit 23: Top Side Right - TSR
                                                 Bit 24: Bottom Center - BC
                                                 Bit 25: Back Left of Center - BLC
                                                 Bit 26: Back Right of Center - BRC
                                                 Bit 31: Raw Data - RD; Mutually exclusive with all other spatial locations
                                            */
    0x00,                                   /* iChannelNames(0x00): None */

    /* Audio Format Type Descriptor */
    0x06,               /* bLength(6) */
    0x24,               /* bDescriptorType(0x24): CS_INTERFACE */
    0x02,               /* bDescriptorSubtype(0x02): FORMAT_TYPE */
    0x01,               /* bFormatType(0x01): FORMAT_TYPE_I */
    0x02,               /* bSubSlotSize(0x02)  :  2 bytes per sample */
    0x10,               /* bBitResolution(0x10): 16  bits per sample */

    /* Standard AS Isochronous Feedback Endpoint Descriptor */
    0x07,                                   /* bLength */
    0x05,                                   /* bDescriptorType */
    ISO_OUT_EP_NUM | EP_OUTPUT,             /* bEndpointAddress */
    0x05,                                   /* bmAttributes */
    (144 + 0) & 0xff, ((144 + 0) >> 8) & 0xff, /* wMaxPacketSize note */
    0x01,                                   /* bInterval - Must be 1 for compliance */

    /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor */
    0x08,               /* bLength */
    0x25,               /* bDescriptorType:CS_ENDPOINT */
    0x01,               /* bDescriptorSubType:EP_GENERAL */
    0x00,               /* bmAttributes */
    0x00,               /* bmControls */
    0x00,               /* bLockDelayUnits */
    0x00, 0x00,         /* wLockDelay */

    /* AS Isochronous Feedback Endpoint Descriptor */
    0x07,                                   /* bLength */
    0x05,                                   /* bDescriptorType: ENDPOINT */
    ISO_FEEDBACK_ENDPOINT | EP_INPUT,       /* bEndpointAddress */
    0x11,                                   /* bmAttributes (bitmap)  */
    0x04, 0x0,                              /* wMaxPacketSize */
    0x04                                    /* bInterval - Must be 1 for compliance */
};

/*!<USB Other Speed Configure Descriptor */
uint8_t gu8OtherConfigDescriptor[] =
{
    /* Configuration Descriptor */
    0x09,               /* Config Descriptor Length */
    0x07,               /* DescriptorType: OTHERSPEED */
    0xFE, 0x00,         /* wTotalLength
                           Configuration Descriptor                      (0x09)
                           Standard Interface Association Descriptor     (0x08)
                           Standard AC Interface Descriptor              (0x09)
                           Class-Specific AC Interface Header Descriptor (0x09)
                           Clock Source Descriptor                       (0x08)
                           Microphone - Audio Control
                             Input Terminal Descriptor                   (0x11)
                             Feature Unit Descriptor                     (0x12)
                             Output Terminal Descriptor                  (0x0C)
                           Speaker - Audio Control
                             Input Terminal Descriptor                   (0x11)
                             Feature Unit Descriptor                     (0x12)
                             Output Terminal Descriptor                  (0x0C)
                           Microphone - Interface alternate 0
                             Standard AS interface                       (0x09)
                           Microphone - Interface alternate 1~2
                             Standard AC Interface Descriptor                             (0x09,0x09)
                             Class-Specific AS Interface Descriptor                       (0x10,0x10)
                             Audio Format Type Descriptor                                 (0x06,0x06)
                             Standard AS Isochronous Feedback Endpoint Descriptor         (0x07,0x07)
                             Class-Specific AS Isochronous Audio Data Endpoint Descriptor (0x08,0x08)
                             *Each Interface alternate Summary                            (0x2E,0x2E)
                           Speaker - Interface alternate 0
                             Standard AS interface                       (0x09)
                           Speaker - Interface alternate 1~2
                             Standard AC Interface Descriptor                             (0x09,0x09)
                             Class-Specific AS Interface Descriptor                       (0x10,0x10)
                             Audio Format Type Descriptor                                 (0x06,0x06)
                             Standard AS Isochronous Feedback Endpoint Descriptor         (0x07,0x07)
                             Class-Specific AS Isochronous Audio Data Endpoint Descriptor (0x08,0x08)
                             AS Isochronous Feedback Endpoint Descriptor                  (0x07,0x07)
                             *Each Interface alternate Summary                            (0x35,0x35)

                           0x09 + 0x08 + 0x9 + 0x09 + 0x08 + (0x11 + 0x12 + 0x0C) + (0x11 + 0x12 + 0x0C) +
                           0x09 + 2 * 0x2E +
                           0x09 + 2 * 0x35 = 0x161
                        */
    0x03,               /* bNumInterfaces - Interface 0, Interface 1 (Speaker), Interface 2 (Microphone) */
    0x01,               /* bConfigurationValue */
    0x00,               /* iConfiguration */
    0x80,               /* bmAttributes */
    0x32,               /* bMaxPower */

    /* Standard Interface Association Descriptor */
    0x08,               /* bLength(0x08) */
    0x0B,               /* bDescriptorType(0x0B) */
    0x00,               /* bFirstInterface(0x00) */
    0x03,               /* bInterfaceCount(0x03) */
    0x01,               /* bFunctionClass(0x01): AUDIO */
    0x00,               /* bFunctionSubClass(0x00) */
    0x20,               /* bFunctionProtocol(0x2000): 2.0 AF_VERSION_02_00 */
    0x00,               /* iFunction(0x00) */

    /* Standard AC Interface Descriptor */
    0x09,               /* bLength */
    0x04,               /* bDescriptorType */
    0x00,               /* bInterfaceNumber */
    0x00,               /* bAlternateSetting */
    0x00,               /* bNumEndpoints */
    0x01,               /* bInterfaceClass:AUDIO */
    0x01,               /* bInterfaceSubClass:AUDIOCONTROL */
    0x20,               /* bInterfaceProtocol */
    0x02,               /* iInterface */

    /* Class-Specific AC Interface Header Descriptor */
    0x09,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x01,               /* bDescriptorSubType:HEADER */
    0x00, 0x02,         /* bcdADC:2.0 */
    0x08,               /* UAC_FUNCTION_IO_BOX */
    0x6F, 0x00,         /* wTotalLength
                           Class-Specific AC Interface Header Descriptor (0x09)
                           Clock Source Descriptor                       (0x08)
                           Speaker - Audio Control
                             Input Terminal Descriptor                   (0x11)
                             Feature Unit Descriptor                     (0x12)
                             Output Terminal Descriptor                  (0x0C)
                           Microphone - Audio Control
                             Input Terminal Descriptor                   (0x11)
                             Feature Unit Descriptor                     (0x12)
                             Output Terminal Descriptor                  (0x0C)

                           0x09 + 0x08 + (0x11 + 0x12 + 0x0C) + (0x11 + 0x12 + 0x0C) = 0x6F
                        */
    0x00,               /* bmControls(0b00000000) - D1..0: Latency Control */

    /* Clock Source Descriptor (bClockID 0x10) */
    0x08,               /* bLength(0x08) */
    0x24,               /* bDescriptorType(0x24): CS_INTERFACE */
    0x0A,               /* bDescriptorSubType(0x0A): CLOCK_SOURCE */
    CLOCK_SOURCE_ID,    /* bClockID(0x10): CLOCK_SOURCE_ID */
    0x03,               /* bmAttributes */
    0x07,               /* bmControls(0x07):
                           clock frequency control: 0b11 - host programmable;
                           clock validity control: 0b01 - host read only */
    0x00,               /* bAssocTerminal(0x00) */
    0x00,               /* iClockSource */

    /* Input Terminal Descriptor (Terminal ID 0x04 - Source ID 0x10) */
    0x11,                                   /* bLength*/
    0x24,                                   /* bDescriptorType:CS_INTERFACE*/
    0x02,                                   /* bDescriptorSubType:INPUT_TERMINAL*/
    0x04,                                   /* bTerminalID*/
    0x02, 0x04,                             /* wTerminalType: HEADSET */
    0x00,                                   /* bAssocTerminal*/
    0x10,                                   /* bSourceID*/
    REC_CHANNELS,                           /* bNrChannels*/
    REC_CH_CFG, 0x00, 0x00, 0x00,           /* wChannelConfig
                                                 Bit 0: Front Left - FL
                                                 Bit 1: Front Right - FR
                                                 Bit 2: Front Center - FC
                                                 Bit 3: Low Frequency Effects - LFE
                                                 Bit 4: Back Left - BL
                                                 Bit 5: Back Right - BR
                                                 Bit 6: Front Left of Center - FLC
                                                 Bit 7: FRONT RIght of Center - FRC
                                                 Bit 8: Back Center - BC
                                                 Bit 9: Side Left - SL
                                                 Bit 10: Side Right - SR
                                                 Bit 11: Top Center - TC
                                                 Bit 12: Top Front Left - TFL
                                                 Bit 13: Top Front Center - TFC
                                                 Bit 14: Top Front Right - TFR
                                                 Bit 15: Top Back Left - TBL
                                                 Bit 16: Top Back Center - TBC
                                                 Bit 17: Top Back Right - TBR
                                                 Bit 18: Top Front Left of Center - TFLC
                                                 Bit 19: Top Front Right of Center - TFRC
                                                 Bit 20: Left Low Frequency Effects - LLFE
                                                 Bit 21: Right Low Frequency Effects - RLFE
                                                 Bit 22: Top Side Left - TSL
                                                 Bit 23: Top Side Right - TSR
                                                 Bit 24: Bottom Center - BC
                                                 Bit 25: Back Left of Center - BLC
                                                 Bit 26: Back Right of Center - BRC
                                                 Bit 31: Raw Data - RD; Mutually exclusive with all other spatial locations
                                            */
    0x00,                                   /* iChannelNames */
    0x00, 0x00,                             /* bmcontrols
                                                 D1..0: Copy Protect Control
                                                 D3..2: Connector Control
                                                 D5..4: Overload Control
                                                 D7..6: Cluster Control
                                                 D9..8: Underflow Control
                                                 D11..10: Overflow Control
                                                 D15..12: Reserved, should set to 0
                                            */
    0x00,                                   /* iTerminal */

    /* Feature Unit Descriptor (Unit ID 0x05 - Source ID 0x4) */
    0x12,                                   /* bLength */
    0x24,                                   /* bDescriptorType */
    0x06,                                   /* bDescriptorSubType */
    REC_FEATURE_UNITID,                     /* bUnitID */
    0x04,                                   /* bSourceID */
    0x00, 0x00, 0x00, 0x00,                 /* bmaControls(0)
                                               Master control
                                                D1..0: Mute Control
                                                D3..2: Volume Control
                                                D5..4: Bass Control
                                                D7..6: Mid Control
                                                D9..8: Treble Control
                                                D11..10: Graphic Equalizer Control
                                                D13..12: Automatic Gain Control
                                                D15..14: Delay Control
                                                D17..16: Bass Control
                                                D19..18: Loudness Control
                                                D21..20: Input Gain Control
                                                D23..22: Input Gain Pad Control
                                                D25..24: Phase Inverter Control
                                                D27..26: Underflow Control
                                                D29..28: Overflow Control
                                                D31..30: Reserved, should set to 0
                                            */
    0x00, 0x00, 0x00, 0x00,                 /* bmaControls(1)
                                               Left volume control
                                                D1..0: Mute Control
                                                D3..2: Volume Control
                                                D5..4: Bass Control
                                                D7..6: Mid Control
                                                D9..8: Treble Control
                                                D11..10: Graphic Equalizer Control
                                                D13..12: Automatic Gain Control
                                                D15..14: Delay Control
                                                D17..16: Bass Control
                                                D19..18: Loudness Control
                                                D21..20: Input Gain Control
                                                D23..22: Input Gain Pad Control
                                                D25..24: Phase Inverter Control
                                                D27..26: Underflow Control
                                                D29..28: Overflow Control
                                                D31..30: Reserved, should set to 0
                                            */
    0x00, 0x00, 0x00, 0x00,                 /* bmaControls(2)
                                               Right volume control
                                                D1..0: Mute Control
                                                D3..2: Volume Control
                                                D5..4: Bass Control
                                                D7..6: Mid Control
                                                D9..8: Treble Control
                                                D11..10: Graphic Equalizer Control
                                                D13..12: Automatic Gain Control
                                                D15..14: Delay Control
                                                D17..16: Bass Control
                                                D19..18: Loudness Control
                                                D21..20: Input Gain Control
                                                D23..22: Input Gain Pad Control
                                                D25..24: Phase Inverter Control
                                                D27..26: Underflow Control
                                                D29..28: Overflow Control
                                                D31..30: Reserved, should set to 0
                                            */
    0x00,                                   /* iFeature */

    /* Output Terminal Descriptor (Terminal ID 0x02 - Source ID 0x5 - Clock Source ID 0x10) */
    0x0C,               /* bLength*/
    0x24,               /* bDescriptorType:CS_INTERFACE*/
    0x03,               /* bDescriptorSubType:OUTPUT_TERMINAL*/
    0x02,               /* bTerminalID*/
    0x01, 0x01,         /* wTerminalType: 0x0101 */
    0x00,               /* bAssocTerminal*/
    REC_FEATURE_UNITID, /* bSourceID*/
    0x10,               /* bCSourceID*/
    0x00, 0x00,         /* bmControls
                             D1..0: Copy Protect Control
                             D3..2: Connector Control
                             D5..4: Overload Control
                             D7..6: Cluster Control
                             D9..8: Underflow Control
                             D11..10: Overflow Control
                             D15..12: Reserved, should set to 0
                        */
    0x00,               /* iTerminal*/
    /* Input Terminal Descriptor (Terminal ID 1 - Source ID 0x10) */
    0x11,                                   /* bLength */
    0x24,                                   /* bDescriptorType:CS_INTERFACE */
    0x02,                                   /* bDescriptorSubType:INPUT_TERMINAL */
    0x01,                                   /* bTerminalID */
    0x01, 0x01,                             /* wTerminalType: 0x0101 usb streaming */
    0x00,                                   /* bAssocTerminal */
    CLOCK_SOURCE_ID,                        /* bCSourceID(0x10): CLOCK_SOURCE_ID */
    PLAY_CHANNELS,                          /* bNrChannels - */
    PLAY_CH_CFG, 0x00, 0x00, 0x00,          /* bmChannelConfig
                                                 Bit 0: Front Left - FL
                                                 Bit 1: Front Right - FR
                                                 Bit 2: Front Center - FC
                                                 Bit 3: Low Frequency Effects - LFE
                                                 Bit 4: Back Left - BL
                                                 Bit 5: Back Right - BR
                                                 Bit 6: Front Left of Center - FLC
                                                 Bit 7: FRONT RIght of Center - FRC
                                                 Bit 8: Back Center - BC
                                                 Bit 9: Side Left - SL
                                                 Bit 10: Side Right - SR
                                                 Bit 11: Top Center - TC
                                                 Bit 12: Top Front Left - TFL
                                                 Bit 13: Top Front Center - TFC
                                                 Bit 14: Top Front Right - TFR
                                                 Bit 15: Top Back Left - TBL
                                                 Bit 16: Top Back Center - TBC
                                                 Bit 17: Top Back Right - TBR
                                                 Bit 18: Top Front Left of Center - TFLC
                                                 Bit 19: Top Front Right of Center - TFRC
                                                 Bit 20: Left Low Frequency Effects - LLFE
                                                 Bit 21: Right Low Frequency Effects - RLFE
                                                 Bit 22: Top Side Left - TSL
                                                 Bit 23: Top Side Right - TSR
                                                 Bit 24: Bottom Center - BC
                                                 Bit 25: Back Left of Center - BLC
                                                 Bit 26: Back Right of Center - BRC
                                                 Bit 31: Raw Data - RD; Mutually exclusive with all other spatial locations
                                            */
    0x00,                                   /* iChannelNames */
    0x00, 0x00,                             /* bmcontrols
                                                 D1..0: Copy Protect Control
                                                 D3..2: Connector Control
                                                 D5..4: Overload Control
                                                 D7..6: Cluster Control
                                                 D9..8: Underflow Control
                                                 D11..10: Overflow Control
                                                 D15..12: Reserved, should set to 0
                                            */
    0x00,                                   /* iTerminal */

    /* Feature Unit Descriptor (Unit ID 0x06 - Source ID 0x1) */
    0x12,                                   /* bLength */
    0x24,                                   /* bDescriptorType */
    0x06,                                   /* bDescriptorSubType */
    PLAY_FEATURE_UNITID,                    /* bUnitID */
    0x01,                                   /* bSourceID */
    0x00, 0x00, 0x00, 0x00,                 /* bmaControls(0)
                                               Master control
                                                D1..0: Mute Control
                                                D3..2: Volume Control
                                                D5..4: Bass Control
                                                D7..6: Mid Control
                                                D9..8: Treble Control
                                                D11..10: Graphic Equalizer Control
                                                D13..12: Automatic Gain Control
                                                D15..14: Delay Control
                                                D17..16: Bass Control
                                                D19..18: Loudness Control
                                                D21..20: Input Gain Control
                                                D23..22: Input Gain Pad Control
                                                D25..24: Phase Inverter Control
                                                D27..26: Underflow Control
                                                D29..28: Overflow Control
                                                D31..30: Reserved, should set to 0
                                            */
    0x00, 0x00, 0x00, 0x00,                 /* bmaControls(1)
                                               Left volume control
                                                D1..0: Mute Control
                                                D3..2: Volume Control
                                                D5..4: Bass Control
                                                D7..6: Mid Control
                                                D9..8: Treble Control
                                                D11..10: Graphic Equalizer Control
                                                D13..12: Automatic Gain Control
                                                D15..14: Delay Control
                                                D17..16: Bass Control
                                                D19..18: Loudness Control
                                                D21..20: Input Gain Control
                                                D23..22: Input Gain Pad Control
                                                D25..24: Phase Inverter Control
                                                D27..26: Underflow Control
                                                D29..28: Overflow Control
                                                D31..30: Reserved, should set to 0
                                            */
    0x00, 0x00, 0x00, 0x00,                 /* bmaControls(2)
                                               Right volume control
                                                D1..0: Mute Control
                                                D3..2: Volume Control
                                                D5..4: Bass Control
                                                D7..6: Mid Control
                                                D9..8: Treble Control
                                                D11..10: Graphic Equalizer Control
                                                D13..12: Automatic Gain Control
                                                D15..14: Delay Control
                                                D17..16: Bass Control
                                                D19..18: Loudness Control
                                                D21..20: Input Gain Control
                                                D23..22: Input Gain Pad Control
                                                D25..24: Phase Inverter Control
                                                D27..26: Underflow Control
                                                D29..28: Overflow Control
                                                D31..30: Reserved, should set to 0
                                            */
    0x00,                                   /* iFeature */

    /* Output Terminal Descriptor (Terminal ID 0x03 - Source ID 0x6 - Clock Source ID 0x10) */
    0x0C,               /* bLength */
    0x24,               /* bDescriptorType:CS_INTERFACE */
    0x03,               /* bDescriptorSubType:OUTPUT_TERMINAL */
    0x03,               /* bTerminalID */
    0x02, 0x04,         /* wTerminalType: HEADSET */
    0x00,               /* bAssocTerminal */
    PLAY_FEATURE_UNITID,/* bSourceID */
    0x10,               /* bCSourceID */
    0x00, 0x00,         /* bmControls
                             D1..0: Copy Protect Control
                             D3..2: Connector Control
                             D5..4: Overload Control
                             D7..6: Cluster Control
                             D9..8: Underflow Control
                             D11..10: Overflow Control
                             D15..12: Reserved, should set to 0
                        */
    0x00,               /* iTerminal */
    /* Standard AC Interface Descriptor - Interface 1, alternate 0 */
    0x09,               /* bLength */
    0x04,               /* bDescriptorType */
    0x01,               /* bInterfaceNumber */
    0x00,               /* bAlternateSetting */
    0x00,               /* bNumEndpoints */
    0x01,               /* bInterfaceClass:AUDIO */
    0x02,               /* bInterfaceSubClass:AUDIOSTREAMING */
    0x20,               /* bInterfaceProtocol */
    0x00,               /* iInterface */

    /* Standard AC Interface Descriptor - Interface 1, alternate 1 */
    0x09,               /* bLength */
    0x04,               /* bDescriptorType */
    0x01,               /* bInterfaceNumber */
    0x01,               /* bAlternateSetting */
    0x01,               /* bNumEndpoints */
    0x01,               /* bInterfaceClass:AUDIO */
    0x02,               /* bInterfaceSubClass:AUDIOSTREAMING */
    0x20,               /* bInterfaceProtocol */
    0x00,               /* iInterface */

    /* Class-Specific AS Interface Descriptor (this interface's endpoint connect to Terminal ID 0x02 - Microphone) */
    0x10,                                   /* bLength(16) */
    0x24,                                   /* bDescriptorType(0x024): CS_INTERFACE */
    0x01,                                   /* bDescriptorSubType(0x01): AS_GENERAL */
    0x02,                                   /* bTerminalLink(0x02): INPUT_TERMINAL_ID */
    0x00,                                   /* bmControls(0x00) */
    0x01,                                   /* bFormatType(0x01): FORMAT_TYPE_I */
    0x01, 0x00, 0x00, 0x00,                 /* bmFormats(0x00000001): PCM
                                                 Bit 0: IEC61937_AC-3
                                                 Bit 1: IEC61937_MPEG-1_Layer1
                                                 Bit 2: IEC61937_MPEG-1_Layer2/3 or IEC61937__MPEG-2_NOEXT
                                                 Bit 3: IEC61937_MPEG-2_EXT
                                                 Bit 4: IEC61937_MPEG-2_AAC_ADTS
                                                 Bit 5: IEC61937_MPEG-2_Layer1_LS
                                                 Bit 6: IEC61937_MPEG-2_Layer2/3_LS
                                                 Bit 7: IEC61937_DTS-I
                                                 Bit 8: IEC61937_DTS-II
                                                 Bit 9: IEC61937_DTS-III
                                                 Bit 10: IEC61937_ATRAC
                                                 Bit 11: IEC61937_ATRAC2/3
                                                 Bit 12: TYPE_III_WMA
                                            */
    0x02,                                   /* bNrChannels(0x02): NB_CHANNELS */
    0x00, 0x00, 0x00, 0x00,                 /* bmChannelCOnfig(0x00000003)
                                                 Bit 0: Front Left - FL
                                                 Bit 1: Front Right - FR
                                                 Bit 2: Front Center - FC
                                                 Bit 3: Low Frequency Effects - LFE
                                                 Bit 4: Back Left - BL
                                                 Bit 5: Back Right - BR
                                                 Bit 6: Front Left of Center - FLC
                                                 Bit 7: FRONT RIght of Center - FRC
                                                 Bit 8: Back Center - BC
                                                 Bit 9: Side Left - SL
                                                 Bit 10: Side Right - SR
                                                 Bit 11: Top Center - TC
                                                 Bit 12: Top Front Left - TFL
                                                 Bit 13: Top Front Center - TFC
                                                 Bit 14: Top Front Right - TFR
                                                 Bit 15: Top Back Left - TBL
                                                 Bit 16: Top Back Center - TBC
                                                 Bit 17: Top Back Right - TBR
                                                 Bit 18: Top Front Left of Center - TFLC
                                                 Bit 19: Top Front Right of Center - TFRC
                                                 Bit 20: Left Low Frequency Effects - LLFE
                                                 Bit 21: Right Low Frequency Effects - RLFE
                                                 Bit 22: Top Side Left - TSL
                                                 Bit 23: Top Side Right - TSR
                                                 Bit 24: Bottom Center - BC
                                                 Bit 25: Back Left of Center - BLC
                                                 Bit 26: Back Right of Center - BRC
                                                 Bit 31: Raw Data - RD; Mutually exclusive with all other spatial locations
                                            */
    0x00,                                   /* iChannelNames(0x00): None */

    /* Audio Format Type Descriptor */
    0x06,               /* bLength(6) */
    0x24,               /* bDescriptorType(0x24): CS_INTERFACE */
    0x02,               /* bDescriptorSubtype(0x02): FORMAT_TYPE */
    0x01,               /* bFormatType(0x01): FORMAT_TYPE_I */
    0x02,               /* bSubSlotSize(0x02)  :  2 bytes per sample */
    0x10,               /* bBitResolution(0x10): 16  bits per sample */

    /* Standard AS Isochronous Feedback Endpoint Descriptor */
    0x07,                                   /* bLength */
    0x05,                                   /* bDescriptorType */
    ISO_IN_EP_NUM | EP_INPUT,               /* bEndpointAddress */
    0x25,                                   /* bmAttributes */
    (144 + 24) & 0xff, ((144 + 24) >> 8) & 0xff, /* wMaxPacketSize note */
    0x01,                                   /* bInterval - Must be 1 for compliance */

    /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor */
    0x08,               /* bLength */
    0x25,               /* bDescriptorType:CS_ENDPOINT */
    0x01,               /* bDescriptorSubType:EP_GENERAL */
    0x00,               /* bmAttributes */
    0x00,               /* bmControls */
    0x00,               /* bLockDelayUnits */
    0x00, 0x00,         /* wLockDelay */

    /* Standard AC Interface Descriptor - Interface 1/2, alternate 0
        Interface 2 for Speaker & Microphone
        Interface 1 for Speaker Only
    */
    0x09,               /* bLength */
    0x04,               /* bDescriptorType */
    0x02,               /* bInterfaceNumber */
    0x00,               /* bAlternateSetting */
    0x00,               /* bNumEndpoints */
    0x01,               /* bInterfaceClass:AUDIO */
    0x02,               /* bInterfaceSubClass:AUDIOSTREAMING */
    0x20,               /* bInterfaceProtocol */
    0x00,               /* iInterface */

    /* Standard AC Interface Descriptor - Interface 1/2, alternate 1 */
    0x09,               /* bLength */
    0x04,               /* bDescriptorType */
    0x02,               /* bInterfaceNumber */
    0x01,               /* bAlternateSetting */
    0x02,               /* bNumEndpoints */
    0x01,               /* bInterfaceClass:AUDIO */
    0x02,               /* bInterfaceSubClass:AUDIOSTREAMING */
    0x20,               /* bInterfaceProtocol */
    0x00,               /* iInterface */

    /* Class-Specific AS Interface Descriptor (this interface's endpoint connect to Terminal ID 0x01 - Speaker)*/
    0x10,                                   /* bLength(16) */
    0x24,                                   /* bDescriptorType(0x024): CS_INTERFACE */
    0x01,                                   /* bDescriptorSubType(0x01): AS_GENERAL */
    0x01,                                   /* bTerminalLink(0x01): INPUT_TERMINAL_ID */
    0x00,                                   /* bmControls(0x00) */
    0x01,                                   /* bFormatType(0x01): FORMAT_TYPE_I */
    0x01, 0x00, 0x00, 0x00,                 /* bmFormats(0x00000001): PCM
                                                 Bit 0: IEC61937_AC-3
                                                 Bit 1: IEC61937_MPEG-1_Layer1
                                                 Bit 2: IEC61937_MPEG-1_Layer2/3 or IEC61937__MPEG-2_NOEXT
                                                 Bit 3: IEC61937_MPEG-2_EXT
                                                 Bit 4: IEC61937_MPEG-2_AAC_ADTS
                                                 Bit 5: IEC61937_MPEG-2_Layer1_LS
                                                 Bit 6: IEC61937_MPEG-2_Layer2/3_LS
                                                 Bit 7: IEC61937_DTS-I
                                                 Bit 8: IEC61937_DTS-II
                                                 Bit 9: IEC61937_DTS-III
                                                 Bit 10: IEC61937_ATRAC
                                                 Bit 11: IEC61937_ATRAC2/3
                                                 Bit 12: TYPE_III_WMA
                                            */
    0x02,                                   /* bNrChannels(0x02): NB_CHANNELS */
    0x03, 0x00, 0x00, 0x00,                 /* bmChannelCOnfig(0x00000003)
                                                 Bit 0: Front Left - FL
                                                 Bit 1: Front Right - FR
                                                 Bit 2: Front Center - FC
                                                 Bit 3: Low Frequency Effects - LFE
                                                 Bit 4: Back Left - BL
                                                 Bit 5: Back Right - BR
                                                 Bit 6: Front Left of Center - FLC
                                                 Bit 7: FRONT RIght of Center - FRC
                                                 Bit 8: Back Center - BC
                                                 Bit 9: Side Left - SL
                                                 Bit 10: Side Right - SR
                                                 Bit 11: Top Center - TC
                                                 Bit 12: Top Front Left - TFL
                                                 Bit 13: Top Front Center - TFC
                                                 Bit 14: Top Front Right - TFR
                                                 Bit 15: Top Back Left - TBL
                                                 Bit 16: Top Back Center - TBC
                                                 Bit 17: Top Back Right - TBR
                                                 Bit 18: Top Front Left of Center - TFLC
                                                 Bit 19: Top Front Right of Center - TFRC
                                                 Bit 20: Left Low Frequency Effects - LLFE
                                                 Bit 21: Right Low Frequency Effects - RLFE
                                                 Bit 22: Top Side Left - TSL
                                                 Bit 23: Top Side Right - TSR
                                                 Bit 24: Bottom Center - BC
                                                 Bit 25: Back Left of Center - BLC
                                                 Bit 26: Back Right of Center - BRC
                                                 Bit 31: Raw Data - RD; Mutually exclusive with all other spatial locations
                                            */
    0x00,                                   /* iChannelNames(0x00): None */

    /* Audio Format Type Descriptor */
    0x06,               /* bLength(6) */
    0x24,               /* bDescriptorType(0x24): CS_INTERFACE */
    0x02,               /* bDescriptorSubtype(0x02): FORMAT_TYPE */
    0x01,               /* bFormatType(0x01): FORMAT_TYPE_I */
    0x02,               /* bSubSlotSize(0x02)  :  2 bytes per sample */
    0x10,               /* bBitResolution(0x10): 16  bits per sample */

    /* Standard AS Isochronous Feedback Endpoint Descriptor */
    0x07,                                   /* bLength */
    0x05,                                   /* bDescriptorType */
    ISO_OUT_EP_NUM | EP_OUTPUT,             /* bEndpointAddress */
    0x05,                                   /* bmAttributes */
    (144 + 0) & 0xff, ((144 + 0) >> 8) & 0xff, /* wMaxPacketSize note */
    0x01,                                   /* bInterval - Must be 1 for compliance */

    /* Class-Specific AS Isochronous Audio Data Endpoint Descriptor */
    0x08,               /* bLength */
    0x25,               /* bDescriptorType:CS_ENDPOINT */
    0x01,               /* bDescriptorSubType:EP_GENERAL */
    0x00,               /* bmAttributes */
    0x00,               /* bmControls */
    0x00,               /* bLockDelayUnits */
    0x00, 0x00,         /* wLockDelay */

    /* AS Isochronous Feedback Endpoint Descriptor */
    0x07,                                   /* bLength */
    0x05,                                   /* bDescriptorType: ENDPOINT */
    ISO_FEEDBACK_ENDPOINT | EP_INPUT,       /* bEndpointAddress */
    0x11,                                   /* bmAttributes (bitmap)  */
    0x04, 0x0,                              /* wMaxPacketSize */
    0x04                                    /* bInterval - Must be 1 for compliance */
};

/*!<USB Language String Descriptor */
uint8_t gu8StringLang[4] =
{
    4,              /* bLength */
    DESC_STRING,    /* bDescriptorType */
    0x09, 0x04
};

/*!<USB Vendor String Descriptor */
uint8_t gu8VendorStringDesc[] =
{
    16,
    DESC_STRING,
    'N', 0, 'u', 0, 'v', 0, 'o', 0, 't', 0, 'o', 0, 'n', 0
};

/*!<USB Product String Descriptor */
uint8_t gu8ProductStringDesc[] =
{
    20,
    DESC_STRING,
    'U', 0, 'S', 0, 'B', 0, ' ', 0, 'A', 0, 'u', 0, 'd', 0, 'i', 0, 'o', 0
};

uint8_t *gpu8UsbString[4] =
{
    gu8StringLang,
    gu8VendorStringDesc,
    gu8ProductStringDesc,
    NULL
};

uint8_t *gu8UsbHidReport[3] =
{
    NULL,
    NULL,
    NULL
};

uint32_t gu32UsbHidReportLen[3] =
{
    0,
    0,
    0
};

uint32_t gu32ConfigHidDescIdx[3] =
{
    0,
    0,
    0
};

S_HSUSBD_INFO_T gsHSInfo =
{
    gu8DeviceDescriptor,
    gu8ConfigDescriptor,
    gpu8UsbString,
    gu8QualifierDescriptor,
    gu8ConfigDescriptor,
    gu8OtherConfigDescriptor,
    gu8OtherConfigDescriptor,
    NULL,
    gu8UsbHidReport,
    gu32UsbHidReportLen,
    gu32ConfigHidDescIdx
};
