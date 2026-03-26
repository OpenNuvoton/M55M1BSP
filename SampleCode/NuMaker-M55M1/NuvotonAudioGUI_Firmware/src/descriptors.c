/**************************************************************************//**
 * @file     descriptors.c
 * @version  V3.00
 * @brief    HSUSBD descriptor.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>

#include "NuMicro.h"
#include "usbd_audio.h"

#define WBVAL(x) (x & 0xFF), ((x >> 8) & 0xFF)
#define B3VAL(x) (x & 0xFF), ((x >> 8) & 0xFF), ((x >> 16) & 0xFF)

const uint8_t gu8HidReportDesc[] =
{
    0x05, 0x01,                     // USAGE_PAGE (Generic Desktop)
    0x09, 0x00,                     // USAGE (0)
    0xA1, 0x01,                     // COLLECTION (Application)
    0x15, 0x00,                     // LOGICAL_MINIMUM (0)
    0x25, 0xFF,                     // LOGICAL_MAXIMUM (255)
    0x19, 0x01,                     // USAGE_MINIMUM (1)
    0x29, 0x08,                     // USAGE_MAXIMUM (8)
    0x95, 0x40,                     // REPORT_COUNT (64)
    0x75, 0x08,                     // REPORT_SIZE (8)
    0x81, 0x02,                     // INPUT (Data,Var,Abs)
    0x19, 0x01,                     // USAGE_MINIMUM (1)
    0x29, 0x08,                     // USAGE_MAXIMUM (8)
    0x91, 0x02,                     // OUTPUT (Data,Var,Abs)
    0xC0                            // END_COLLECTION
};

/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
uint8_t gu8DeviceDescriptor[] =
{
    LEN_DEVICE,                     /* bLength */
    DESC_DEVICE,                    /* bDescriptorType */
    WBVAL(0x0200),                  /* bcdUSB */
    0x00,                           /* bDeviceClass */
    0x00,                           /* bDeviceSubClass */
    0x00,                           /* bDeviceProtocol */
    CEP_MAX_PKT_SIZE,               /* bMaxPacketSize0 */
    WBVAL(USBD_VID),                /* idVendor */
    WBVAL(USBD_PID),                /* idProduct */
    0x00, 0x00,                     /* bcdDevice */
    0x01,                           /* iManufacture */
    0x02,                           /* iProduct */
    0x03,                           /* iSerialNumber
                                       NOTE: The serial number must be different between each MassStorage device. */
    0x01                            /* bNumConfigurations */
};

/*!<USB Configure Descriptor */
uint8_t gu8ConfigDescriptor[] =
{
    LEN_CONFIG,                     /* bLength */
    DESC_CONFIG,                    /* bDescriptorType */
#if (REC_CHANNELS == 1)
    WBVAL((261)),                   /* wTotalLength */
#elif (REC_CHANNELS == 2)
    WBVAL((226)),                   /* wTotalLength */
#elif (REC_CHANNELS == 4)
    WBVAL((246)),                   /* wTotalLength */
#endif
    0x04,                           /* bNumInterfaces */
    0x01,                           /* bConfigurationValue */
    0x00,                           /* iConfiguration */
    USB_CONFIG_BUS_POWERED,         /* bmAttributes */
    USB_CONFIG_POWER_MA(100),       /* Max power */

    /* Standard AC inteface */
    LEN_INTERFACE,                  /* bLength */
    DESC_INTERFACE,                 /* bDescriptorType */
    0x00,                           /* bInterfaceNumber */
    0x00,                           /* bAlternateSetting */
    0x00,                           /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,         /* bInterfaceClass:AUDIO */
    AUDIO_SUBCLASS_AUDIOCONTROL,    /* bInterfaceSubClass:AUDIOCONTROL */
    0x00,                           /* bInterfaceProtocol */
    0x00,                           /* iInterface */

    /* Class-spec AC interface descriptor */
    0x0A,                           /* bLength */
    0x24,                           /* bDescriptorType:CS_INTERFACE */
    0x01,                           /* bDescriptorSubType:HEADER */
    0x00, 0x01,                     /* bcdADC:1.0 */
#if (REC_CHANNELS == 1)
    WBVAL(0x47),                    /* wTotalLength */
#elif (REC_CHANNELS == 2)
    WBVAL(0x48),                    /* wTotalLength */
#elif (REC_CHANNELS == 4)
    WBVAL(0x4A),                    /* wTotalLength */
#endif
    0x02,                           /* bInCollection */
    0x01,                           /* baInterfaceNr(1) */
    0x02,                           /* baInterfaceNr(2) */

    /* Playback device input terminal for usb streaming */
    0x0C,                           /* bLength */
    0x24,                           /* bDescriptorType:CS_INTERFACE */
    0x02,                           /* bDescriptorSubType:INPUT_TERMINAL */
    0x01,                           /* bTerminalID */
    WBVAL(0x0101),                  /* wTerminalType */
    0x00,                           /* bAssocTerminal */
    PLAY_CHANNELS,                  /* bNrChannels */
    PLAY_CH_CFG, 0x00,              /* wChannelConfig */
    0x00,                           /* iChannelNames */
    0x00,                           /* iTerminal */

    /* Feature Unit */
    0x0A,                           /* bLength */
    0x24,                           /* bDescriptorType */
    0x06,                           /* bDescriptorSubType */
    PLAY_FEATURE_UNITID,            /* bUnitID */
    0x01,                           /* bSourceID */
    0x01,                           /* bControlSize */
    0x01,                           /* bmaControls(0) */
    0x00,                           /* bmaControls(1) */
    0x00,                           /* bmaControls(2) */
    0x00,                           /* iFeature */

    /* Playback device output terminal for speaker */
    0x09,                           /* bLength*/
    0x24,                           /* bDescriptorType:CS_INTERFACE*/
    0x03,                           /* bDescriptorSubType:OUTPUT_TERMINAL*/
    0x03,                           /* bTerminalID*/
    WBVAL(0x0301),                  /* wTerminalType: 0x0301 speaker*/
    0x00,                           /* bAssocTerminal*/
    PLAY_FEATURE_UNITID,            /* bSourceID*/
    0x00,                           /* iTerminal*/

    /* Record device input terminal for microphone */
    0x0C,                           /* bLength */
    0x24,                           /* bDescriptorType:CS_INTERFACE */
    0x02,                           /* bDescriptorSubType:INPUT_TERMINAL*/
    0x04,                           /* bTerminalID*/
    0x01, 0x02,                     /* wTerminalType: 0x0201 microphone*/
    0x00,                           /* bAssocTerminal*/
    REC_CHANNELS,                   /* bNrChannels*/
    REC_CH_CFG, 0x00,               /* wChannelConfig*/
    0x00,                           /* iChannelNames*/
    0x00,                           /* iTerminal*/

    /* Feature Unit */
#if (REC_CHANNELS == 1)
    0x09,                           /* bLength */
#elif (REC_CHANNELS == 2)
    0x0A,                           /* bLength */
#elif (REC_CHANNELS == 4)
    0x0C,                           /* bLength */
#endif
    0x24,                           /* bDescriptorType */
    0x06,                           /* bDescriptorSubType */
    REC_FEATURE_UNITID,             /* bUnitID */
    0x04,                           /* bSourceID */
    0x01,                           /* bControlSize */
#if (REC_CHANNELS == 1)
    0x03,                           /* bmaControls(0) */
    0x00,                           /* bmaControls(1) */
#elif (REC_CHANNELS == 2)
    0x01,                           /* bmaControls(0) */
    0x02,                           /* bmaControls(1) */
    0x02,                           /* bmaControls(2) */
#elif (REC_CHANNELS == 4)
    0x01,                           /* bmaControls(0) */
    0x02,                           /* bmaControls(1) */
    0x02,                           /* bmaControls(2) */
    0x02,                           /* bmaControls(3) */
    0x02,                           /* bmaControls(4) */
#endif
    0x00,                           /* iFeature */

    /* Record device output terminal for usb streaming */
    0x09,                           /* bLength */
    0x24,                           /* bDescriptorType:CS_INTERFACE */
    0x03,                           /* bDescriptorSubType:OUTPUT_TERMINAL */
    0x02,                           /* bTerminalID */
    WBVAL(0x0101),                  /* wTerminalType */
    0x00,                           /* bAssocTerminal */
    REC_FEATURE_UNITID,             /* bSourceID */
    0x00,                           /* iTerminal */

    /* Standard AS interface 1, alternate 0 */
    LEN_INTERFACE,                  /* bLength */
    DESC_INTERFACE,                 /* bDescriptorType */
    0x01,                           /* bInterfaceNumber */
    0x00,                           /* bAlternateSetting */
    0x00,                           /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,         /* bInterfaceClass:AUDIO */
    AUDIO_SUBCLASS_AUDIOSTREAMING,  /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,                           /* bInterfaceProtocol */
    0x00,                           /* iInterface */

    // ---------- Standard AS interface 1, alternate 1 Start ----------
    LEN_INTERFACE,                  /* bLength */
    DESC_INTERFACE,                 /* bDescriptorType */
    0x01,                           /* bInterfaceNumber */
    0x01,                           /* bAlternateSetting */
    0x01,                           /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,         /* bInterfaceClass:AUDIO */
    AUDIO_SUBCLASS_AUDIOSTREAMING,  /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,                           /* bInterfaceProtocol */
    0x00,                           /* iInterface */

    /* Class-spec AS interface */
    0x07,                           /* bLength */
    0x24,                           /* bDescriptorType:CS_INTERFACE */
    0x01,                           /* bDescriptorSubType:AS_GENERAL */
    0x01,                           /* bTernimalLink */
    0x01,                           /* bDelay */
    WBVAL(0x0001),                  /* wFormatTag */

    /* Type I format type Descriptor */
    0x0B,                           /* bLength */
    0x24,                           /* bDescriptorType:CS_INTERFACE */
    0x02,                           /* bDescriptorSubType:FORMAT_TYPE */
    0x01,                           /* bFormatType:FORMAT_TYPE_I */
    PLAY_CHANNELS,                  /* bNrChannels */
    0x02,                           /* bSubFrameSize */
    0x10,                           /* bBitResolution */
    0x01,                           /* bSamFreqType : 0 continuous; 1 discrete */
    B3VAL(PLAY_RATE_48K),           /* Sample Frequency */

    /* Standard AS ISO Audio Data Endpoint */
    0x09,                           /* bLength */
    0x05,                           /* bDescriptorType */
    (EPD | EP_OUTPUT),              /* bEndpointAddress */
    0x09,                           /* bmAttributes */
    WBVAL(EPD_MAX_PKT_SIZE),        /* wMaxPacketSize */
    0x01,                           /* bInterval */
    0x00,                           /* bRefresh */
    0x00,

    /* Class-spec AS ISO Audio Data endpoint Descriptor */
    0x07,                           /* bLength */
    0x25,                           /* bDescriptorType:CS_ENDPOINT */
    0x01,                           /* bDescriptorSubType:EP_GENERAL */
    0x01,                           /* bmAttributes */
    0x00,                           /* bLockDelayUnits */
    0x00, 0x00,                     /* wLockDelay */

    // ---------- Standard AS interface 1, alternate 1 End ----------

    //-------------------------------------------------------
    /* Standard AS interface 2, alternate 0 */
    LEN_INTERFACE,                  /* bLength */
    DESC_INTERFACE,                 /* bDescriptorType */
    0x02,                           /* bInterfaceNumber */
    0x00,                           /* bAlternateSetting */
    0x00,                           /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,         /* bInterfaceClass:AUDIO */
    AUDIO_SUBCLASS_AUDIOSTREAMING,  /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,                           /* bInterfaceProtocol */
    0x00,                           /* iInterface */

    //---------- Standard AS interface 2, alternate 1 Start----------
    LEN_INTERFACE,                  /* bLength */
    DESC_INTERFACE,                 /* bDescriptorType */
    0x02,                           /* bInterfaceNumber */
    0x01,                           /* bAlternateSetting */
    0x01,                           /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,         /* bInterfaceClass:AUDIO */
    AUDIO_SUBCLASS_AUDIOSTREAMING,  /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,                           /* bInterfaceProtocol */
    0x00,                           /* iInterface */

    /* Class-spec AS interface */
    0x07,                           /* bLength */
    0x24,                           /* bDescriptorType:CS_INTERFACE */
    0x01,                           /* bDescriptorSubType:AS_GENERAL */
    0x02,                           /* bTernimalLink */
    0x01,                           /* bDelay */
    WBVAL(0x0001),                  /* wFormatTag */

    /* Type I format type Descriptor */
    0x0B,                           /* bLength */
    0x24,                           /* bDescriptorType:CS_INTERFACE */
    0x02,                           /* bDescriptorSubType:FORMAT_TYPE */
    0x01,                           /* bFormatType:FORMAT_TYPE_I */
#if (REC_CHANNELS == 1)
    0x01,                           /* bNrChannels */
#elif (REC_CHANNELS == 2)
    0x02,                           /* bNrChannels */
#elif (REC_CHANNELS == 4)
    0x04,                           /* bNrChannels */
#endif
    0x02,                           /* bSubFrameSize */
    0x10,                           /* bBitResolution */
    0x01,                           /* bSamFreqType : 0 continuous; 1 discrete */
    B3VAL(REC_RATE_48K),            /* Sample Frequency */

    /* Standard AS ISO Audio Data Endpoint */
    0x09,                           /* bLength */
    0x05,                           /* bDescriptorType */
    (EPC | EP_INPUT),               /* bEndpointAddress */
    0x0d,                           /* bmAttributes */
    WBVAL(EPC_MAX_PKT_SIZE),        /* wMaxPacketSize */
    0x01,                           /* bInterval*/
    0x00,                           /* bRefresh*/
    0x00,                           /* bSynchAddress*/

    /* Class-spec AS ISO Audio Data endpoint Descriptor */
    0x07,                           /* bLength */
    0x25,                           /* bDescriptorType:CS_ENDPOINT */
    0x01,                           /* bDescriptorSubType:EP_GENERAL */
    0x00,                           /* bmAttributes */
    0x00,                           /* bLockDelayUnits */
    0x00, 0x00,                     /* wLockDelay */
    //---------- Standard AS interface 2, alternate 1 End----------

    //---------- AudioStreaming Interface Descriptors and Endpoint Descriptors END----------

    // Standard Interface Descriptor for HID
    LEN_INTERFACE,                  /* bLength */
    DESC_INTERFACE,                 /* bDescriptorType */
    0x03,                           /* bInterfaceNumber */
    0x00,                           /* bAlternateSetting */
    0x02,                           /* bNumEndpoints */
    0x03,                           /* bInterfaceClass */
    0x00,                           /* bInterfaceSubClass */
    0x00,                           /* bInterfaceProtocol */
    0x00,                           /* iInterface */

    // HID Descriptor
    LEN_HID,                        /* Size of this descriptor in UINT8s. */
    DESC_HID,                       /* HID descriptor type. */
    0x10, 0x01,                     /* HID Class Spec. release number. */
    0x00,                           /* H/W target country. */
    0x01,                           /* Number of HID class descriptors to follow. */
    DESC_HID_RPT,                   /* Descriptor type. */
    /* Total length of report descriptor. */
    sizeof(gu8HidReportDesc) & 0x00FF,
    (sizeof(gu8HidReportDesc) & 0xFF00) >> 8,

                                        // EP Descriptor: interrupt in.
                                        LEN_ENDPOINT,                   /* bLength */
                                        DESC_ENDPOINT,                  /* bDescriptorType */
                                        (EPG | EP_INPUT),               /* bEndpointAddress */
    EP_INT,                         /* bmAttributes */
    /* wMaxPacketSize */
    EPG_MAX_PKT_SIZE & 0x00FF,
    (EPG_MAX_PKT_SIZE & 0xFF00) >> 8,
                                10,                             /* bInterval */

                                // EP Descriptor: interrupt out.
                                LEN_ENDPOINT,      /* bLength */
                                DESC_ENDPOINT,     /* bDescriptorType */
                                (EPH | EP_OUTPUT), /* bEndpointAddress */
    EP_INT,            /* bmAttributes */
    /* wMaxPacketSize */
    EPH_MAX_PKT_SIZE & 0x00FF,
    (EPH_MAX_PKT_SIZE & 0xFF00) >> 8,
                                10 /* bInterval */

};

uint8_t gu8ConfigDescriptor_Normal_mode_Rec_4ch[] =
{
    LEN_CONFIG,                     /* bLength */
    DESC_CONFIG,                    /* bDescriptorType */
    WBVAL((228)),                   /* wTotalLength */
    0x04,                           /* bNumInterfaces */
    0x01,                           /* bConfigurationValue */
    0x00,                           /* iConfiguration */
    USB_CONFIG_BUS_POWERED,         /* bmAttributes */
    USB_CONFIG_POWER_MA(100),       /* Max power */

    /* Standard AC inteface */
    LEN_INTERFACE,                  /* bLength */
    DESC_INTERFACE,                 /* bDescriptorType */
    0x00,                           /* bInterfaceNumber */
    0x00,                           /* bAlternateSetting */
    0x00,                           /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,         /* bInterfaceClass:AUDIO */
    AUDIO_SUBCLASS_AUDIOCONTROL,    /* bInterfaceSubClass:AUDIOCONTROL */
    0x00,                           /* bInterfaceProtocol */
    0x00,                           /* iInterface */

    /* Class-spec AC interface descriptor */
    0x0A,                           /* bLength */
    0x24,                           /* bDescriptorType:CS_INTERFACE */
    0x01,                           /* bDescriptorSubType:HEADER */
    0x00, 0x01,                     /* bcdADC:1.0 */
    WBVAL(0x4A),                    /* wTotalLength */
    0x02,                           /* bInCollection */
    0x01,                           /* baInterfaceNr(1) */
    0x02,                           /* baInterfaceNr(2) */

    /* Playback device input terminal for usb streaming */
    0x0C,                           /* bLength */
    0x24,                           /* bDescriptorType:CS_INTERFACE */
    0x02,                           /* bDescriptorSubType:INPUT_TERMINAL */
    0x01,                           /* bTerminalID */
    WBVAL(0x0101),                  /* wTerminalType */
    0x00,                           /* bAssocTerminal */
    PLAY_CHANNELS,                  /* bNrChannels */
    PLAY_CH_CFG, 0x00,              /* wChannelConfig */
    0x00,                           /* iChannelNames */
    0x00,                           /* iTerminal */

    /* Feature Unit */
    0x0A,                           /* bLength */
    0x24,                           /* bDescriptorType */
    0x06,                           /* bDescriptorSubType */
    PLAY_FEATURE_UNITID,            /* bUnitID */
    0x01,                           /* bSourceID */
    0x01,                           /* bControlSize */
    0x01,                           /* bmaControls(0) */
    0x00,                           /* bmaControls(1) */
    0x00,                           /* bmaControls(2) */
    0x00,                           /* iFeature */

    /* Playback device output terminal for speaker */
    0x09,                           /* bLength*/
    0x24,                           /* bDescriptorType:CS_INTERFACE*/
    0x03,                           /* bDescriptorSubType:OUTPUT_TERMINAL*/
    0x03,                           /* bTerminalID*/
    WBVAL(0x0301),                  /* wTerminalType: 0x0301 speaker*/
    0x00,                           /* bAssocTerminal*/
    PLAY_FEATURE_UNITID,            /* bSourceID*/
    0x00,                           /* iTerminal*/

    /* Record device input terminal for microphone */
    0x0C,                           /* bLength */
    0x24,                           /* bDescriptorType:CS_INTERFACE */
    0x02,                           /* bDescriptorSubType:INPUT_TERMINAL*/
    0x04,                           /* bTerminalID*/
    0x01, 0x02,                     /* wTerminalType: 0x0201 microphone*/
    0x00,                           /* bAssocTerminal*/
    0x04,                           /* bNrChannels*/
    0x0F, 0x00,                     /* wChannelConfig*/
    0x00,                           /* iChannelNames*/
    0x00,                           /* iTerminal*/

    /* Feature Unit */
    0x0C,                           /* bLength */
    0x24,                           /* bDescriptorType */
    0x06,                           /* bDescriptorSubType */
    REC_FEATURE_UNITID,             /* bUnitID */
    0x04,                           /* bSourceID */
    0x01,                           /* bControlSize */
    0x01,                           /* bmaControls(0) */
    0x02,                           /* bmaControls(1) */
    0x02,                           /* bmaControls(2) */
    0x02,                           /* bmaControls(3) */
    0x02,                           /* bmaControls(4) */
    0x00,                           /* iFeature */

    /* Record device output terminal for usb streaming */
    0x09,                           /* bLength */
    0x24,                           /* bDescriptorType:CS_INTERFACE */
    0x03,                           /* bDescriptorSubType:OUTPUT_TERMINAL */
    0x02,                           /* bTerminalID */
    WBVAL(0x0101),                  /* wTerminalType */
    0x00,                           /* bAssocTerminal */
    REC_FEATURE_UNITID,             /* bSourceID */
    0x00,                           /* iTerminal */

    /* Standard AS interface 1, alternate 0 */
    LEN_INTERFACE,                  /* bLength */
    DESC_INTERFACE,                 /* bDescriptorType */
    0x01,                           /* bInterfaceNumber */
    0x00,                           /* bAlternateSetting */
    0x00,                           /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,         /* bInterfaceClass:AUDIO */
    AUDIO_SUBCLASS_AUDIOSTREAMING,  /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,                           /* bInterfaceProtocol */
    0x00,                           /* iInterface */

    // ---------- Standard AS interface 1, alternate 1 Start ----------
    LEN_INTERFACE,                  /* bLength */
    DESC_INTERFACE,                 /* bDescriptorType */
    0x01,                           /* bInterfaceNumber */
    0x01,                           /* bAlternateSetting */
    0x01,                           /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,         /* bInterfaceClass:AUDIO */
    AUDIO_SUBCLASS_AUDIOSTREAMING,  /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,                           /* bInterfaceProtocol */
    0x00,                           /* iInterface */

    /* Class-spec AS interface */
    0x07,                           /* bLength */
    0x24,                           /* bDescriptorType:CS_INTERFACE */
    0x01,                           /* bDescriptorSubType:AS_GENERAL */
    0x01,                           /* bTernimalLink */
    0x01,                           /* bDelay */
    WBVAL(0x0001),                  /* wFormatTag */

    /* Type I format type Descriptor */
    0x0B,                           /* bLength */
    0x24,                           /* bDescriptorType:CS_INTERFACE */
    0x02,                           /* bDescriptorSubType:FORMAT_TYPE */
    0x01,                           /* bFormatType:FORMAT_TYPE_I */
    PLAY_CHANNELS,                  /* bNrChannels */
    0x02,                           /* bSubFrameSize */
    0x10,                           /* bBitResolution */
    0x01,                           /* bSamFreqType : 0 continuous; 1 discrete */
    B3VAL(PLAY_RATE_48K),           /* Sample Frequency */

    /* Standard AS ISO Audio Data Endpoint */
    0x09,                           /* bLength */
    0x05,                           /* bDescriptorType */
    (EPD | EP_OUTPUT),              /* bEndpointAddress */
    0x09,                           /* bmAttributes */
    WBVAL(EPD_MAX_PKT_SIZE),        /* wMaxPacketSize */
    0x01,                           /* bInterval */
    0x00,                           /* bRefresh */
    0x00,

    /* Class-spec AS ISO Audio Data endpoint Descriptor */
    0x07,                           /* bLength */
    0x25,                           /* bDescriptorType:CS_ENDPOINT */
    0x01,                           /* bDescriptorSubType:EP_GENERAL */
    0x01,                           /* bmAttributes */
    0x00,                           /* bLockDelayUnits */
    0x00, 0x00,                     /* wLockDelay */

    // ---------- Standard AS interface 1, alternate 1 End ----------

    //-------------------------------------------------------
    /* Standard AS interface 2, alternate 0 */
    LEN_INTERFACE,                  /* bLength */
    DESC_INTERFACE,                 /* bDescriptorType */
    0x02,                           /* bInterfaceNumber */
    0x00,                           /* bAlternateSetting */
    0x00,                           /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,         /* bInterfaceClass:AUDIO */
    AUDIO_SUBCLASS_AUDIOSTREAMING,  /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,                           /* bInterfaceProtocol */
    0x00,                           /* iInterface */

    //---------- Standard AS interface 2, alternate 1 Start----------
    LEN_INTERFACE,                  /* bLength */
    DESC_INTERFACE,                 /* bDescriptorType */
    0x02,                           /* bInterfaceNumber */
    0x01,                           /* bAlternateSetting */
    0x01,                           /* bNumEndpoints */
    USB_DEVICE_CLASS_AUDIO,         /* bInterfaceClass:AUDIO */
    AUDIO_SUBCLASS_AUDIOSTREAMING,  /* bInterfaceSubClass:AUDIOSTREAMING */
    0x00,                           /* bInterfaceProtocol */
    0x00,                           /* iInterface */

    /* Class-spec AS interface */
    0x07,                           /* bLength */
    0x24,                           /* bDescriptorType:CS_INTERFACE */
    0x01,                           /* bDescriptorSubType:AS_GENERAL */
    0x02,                           /* bTernimalLink */
    0x01,                           /* bDelay */
    WBVAL(0x0001),                  /* wFormatTag */

    /* Type I format type Descriptor */
    0x0B,                           /* bLength */
    0x24,                           /* bDescriptorType:CS_INTERFACE */
    0x02,                           /* bDescriptorSubType:FORMAT_TYPE */
    0x01,                           /* bFormatType:FORMAT_TYPE_I */
    0x04,                           /* bNrChannels */
    0x02,                           /* bSubFrameSize */
    0x10,                           /* bBitResolution */
    0x01,                           /* bSamFreqType : 0 continuous; 1 discrete */
    B3VAL(REC_RATE_48K),            /* Sample Frequency */

    /* Standard AS ISO Audio Data Endpoint */
    0x09,                           /* bLength */
    0x05,                           /* bDescriptorType */
    (EPC | EP_INPUT),               /* bEndpointAddress */
    0x0d,                           /* bmAttributes */
    WBVAL(EPC_MAX_PKT_SIZE),        /* wMaxPacketSize */
    0x01,                           /* bInterval*/
    0x00,                           /* bRefresh*/
    0x00,                           /* bSynchAddress*/

    /* Class-spec AS ISO Audio Data endpoint Descriptor */
    0x07,                           /* bLength */
    0x25,                           /* bDescriptorType:CS_ENDPOINT */
    0x01,                           /* bDescriptorSubType:EP_GENERAL */
    0x00,                           /* bmAttributes */
    0x00,                           /* bLockDelayUnits */
    0x00, 0x00,                     /* wLockDelay */
    //---------- Standard AS interface 2, alternate 1 End----------

    //---------- AudioStreaming Interface Descriptors and Endpoint Descriptors END----------

    // Standard Interface Descriptor for HID
    LEN_INTERFACE,                  /* bLength */
    DESC_INTERFACE,                 /* bDescriptorType */
    0x03,                           /* bInterfaceNumber */
    0x00,                           /* bAlternateSetting */
    0x02,                           /* bNumEndpoints */
    0x03,                           /* bInterfaceClass */
    0x00,                           /* bInterfaceSubClass */
    0x00,                           /* bInterfaceProtocol */
    0x00,                           /* iInterface */

    // HID Descriptor
    LEN_HID,                        /* Size of this descriptor in UINT8s. */
    DESC_HID,                       /* HID descriptor type. */
    0x10, 0x01,                     /* HID Class Spec. release number. */
    0x00,                           /* H/W target country. */
    0x01,                           /* Number of HID class descriptors to follow. */
    DESC_HID_RPT,                   /* Descriptor type. */
    /* Total length of report descriptor. */
    sizeof(gu8HidReportDesc) & 0x00FF,
    (sizeof(gu8HidReportDesc) & 0xFF00) >> 8,

                                        // EP Descriptor: interrupt in.
                                        LEN_ENDPOINT,                   /* bLength */
                                        DESC_ENDPOINT,                  /* bDescriptorType */
                                        (EPG | EP_INPUT),               /* bEndpointAddress */
    EP_INT,                         /* bmAttributes */
    /* wMaxPacketSize */
    EPG_MAX_PKT_SIZE & 0x00FF,
    (EPG_MAX_PKT_SIZE & 0xFF00) >> 8,
                                10,                             /* bInterval */

                                // EP Descriptor: interrupt out.
                                LEN_ENDPOINT,                   /* bLength */
                                DESC_ENDPOINT,                  /* bDescriptorType */
                                (EPH | EP_OUTPUT),              /* bEndpointAddress */
    EP_INT,                         /* bmAttributes */
    /* wMaxPacketSize */
    EPH_MAX_PKT_SIZE & 0x00FF,
    (EPH_MAX_PKT_SIZE & 0xFF00) >> 8,
                                10                              /* bInterval */
};

/*!<USB Language String Descriptor */
uint8_t gu8StringLang[4] =
{
    4,                              /* bLength */
    DESC_STRING,                    /* bDescriptorType */
    0x09, 0x04
};

/*!<USB Vendor String Descriptor */
uint8_t gu8VendorStringDesc[16] =
{
    16,
    DESC_STRING,
    'N', 0,
    'u', 0,
    'v', 0,
    'o', 0,
    't', 0,
    'o', 0,
    'n', 0

};

/*!<USB Product String Descriptor */
uint8_t gu8ProductStringDesc[] =
{
    46,                             /* bLength          */
    DESC_STRING,                    /* bDescriptorType  */
    'N', 0,
    'u', 0,
    'v', 0,
    'o', 0,
    't', 0,
    'o', 0,
    'n', 0,
    ' ', 0,
    'U', 0,
    'A', 0,
    'C', 0,
    '+', 0,
    'H', 0,
    'I', 0,
    'D', 0,
    ' ', 0,
    'D', 0,
    'e', 0,
    'v', 0,
    'i', 0,
    'c', 0,
    'e', 0

};

uint8_t gu8StringSerial[26] =
{
    26,                             /* bLength */
    DESC_STRING,                    /* bDescriptorType */
    'A', 0, '0', 0, '0', 0, '0', 0, '0', 0, '8', 0, '0', 0, '4', 0, '0', 0, '1', 0, '1', 0, '5', 0

};

/*!<USB BOS Descriptor */
uint8_t gu8BOSDescriptor[] =
{
    LEN_BOS,                        /* bLength */
    DESC_BOS,                       /* bDescriptorType */
    /* wTotalLength */
    0x0C & 0x00FF,
    (0x0C & 0xFF00) >> 8,
                    0x01,                           /* bNumDeviceCaps */

                    /* Device Capability */
                    0x7,                            /* bLength */
                    DESC_CAPABILITY,                /* bDescriptorType */
                    CAP_USB20_EXT,                  /* bDevCapabilityType */
                    0x02, 0x00, 0x00, 0x00          /* bmAttributes */
};

uint8_t *gpu8UsbString[4] =
{
    gu8StringLang,
    gu8VendorStringDesc,
    gu8ProductStringDesc,
    gu8StringSerial
};

const uint8_t *gu8UsbHidReport[6] =
{
    NULL,
    NULL,
    NULL,
    gu8HidReportDesc,
    NULL,
    NULL

};

const uint32_t gu32UsbHidReportLen[6] =
{
    0,
    0,
    0,
    sizeof(gu8HidReportDesc),
    0,
    0

};

uint32_t gu32ConfigHidDescIdx[4] =
{
    0,
    0,
    0,
    //(sizeof(gu8ConfigDescriptor) - LEN_ENDPOINT - LEN_HID)
    (sizeof(gu8ConfigDescriptor) - (2 * LEN_ENDPOINT) - LEN_HID),
};

S_HSUSBD_INFO_T gsHSInfo =
{
    (uint8_t *)gu8DeviceDescriptor,
    (uint8_t *)NULL,
    (uint8_t **)gpu8UsbString,

    (uint8_t *)NULL,
    (uint8_t *)gu8ConfigDescriptor,
    (uint8_t *)NULL,
    (uint8_t *)NULL,

    (uint8_t *)gu8BOSDescriptor,
    (uint8_t **)gu8UsbHidReport,
    (uint32_t *)gu32UsbHidReportLen,
    (uint32_t *)gu32ConfigHidDescIdx

};

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/