/**************************************************************************//**
 * @file     descriptors.c
 *
 * @brief    HSUSBD descriptor
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*!<Includes */
#include "NuMicro.h"
#include "UVC.h"

/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
uint8_t gu8DeviceDescriptor[] __attribute__((aligned(DCACHE_LINE_SIZE))) =
{
    LEN_DEVICE,     /* bLength */
    DESC_DEVICE,    /* bDescriptorType */
    0x00, 0x02,     /* bcdUSB */
    0xEF,           /* bDeviceClass */
    0x02,           /* bDeviceSubClass */
    0x01,           /* bDeviceProtocol */
    CEP_MAX_PKT_SIZE,   /* bMaxPacketSize0 */
    /* idVendor */
    HSUSBD_VID & 0x00FF,
    ((HSUSBD_VID & 0xFF00) >> 8),
    /* idProduct */
    HSUSBD_PID & 0x00FF,
    ((HSUSBD_PID & 0xFF00) >> 8),
    0x00, 0x01,     /* bcdDevice */
    0x01,           /* iManufacture */
    0x02,           /* iProduct */
    0x00,           /* iSerialNumber - no serial */
    0x01            /* bNumConfigurations */
};

/*!<USB Qualifier Descriptor */
uint8_t gu8QualifierDescriptor[] __attribute__((aligned(DCACHE_LINE_SIZE))) =
{
    LEN_QUALIFIER,  /* bLength */
    DESC_QUALIFIER, /* bDescriptorType */
    0x00, 0x02,     /* bcdUSB */
    0xEF,           /* bDeviceClass */
    0x02,           /* bDeviceSubClass */
    0x01,           /* bDeviceProtocol */
    CEP_OTHER_MAX_PKT_SIZE, /* bMaxPacketSize0 */
    0x01,           /* bNumConfigurations */
    0x00
};

/*!<USB Configure Descriptor */
uint8_t gu8ConfigDescriptor[] __attribute__((aligned(DCACHE_LINE_SIZE))) =
{
    LEN_CONFIG,     /* bLength              */
    DESC_CONFIG,    /* bDescriptorType      */
    0x58, 0x01,
//    0x26, 0x01,

    0x02,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x00,           /* iConfiguration */
    0xC0,           /* bmAttributes */
    0x00,           /* MaxPower */

    /*  Standard Video Interface Collection IAD(interface Association Descriptor) */
    0x08,   /* bLength */
    0x0B,   /* bDescriptorType */
    0x00,   /* bFirstInterface */
    0x02,   /* bInterfaceCount */
    0x0E,   /* bFunctionClass */
    0x03,   /* bFunctionSubClass */
    0x00,   /* bFunctionProtocol */
    0x02,   /* iFunction */

    /* Standard VideoControl Interface Descriptor */
    0x09,   /* bLength */
    0x04,   /* bDescriptorType */
    0x00,   /* bInterfceNumber */
    0x00,   /* bAlternateSetting */
    0x01,   /* bNumEndpoints */
    0x0E,   /* bInterfaceClass */
    0x01,   /* bInterfaceSubClass */
    0x00,   /* bInterfaceProtocol */
    0x02,   /* iInterface */

    /* Class-specific VideoControl Interface Descriptor */
    0x0D,                   /* bLength */
    0x24,                   /* bDescriptorType */
    0x01,                   /* bDescriptorSubType */
    0x00, 0x01,             /* bcdUAVC */
    0x32, 0x00,             /* wTotalLength */
    0x80, 0x8D, 0x5B, 0x00, /* dwClockFrequency */
    0x01,                   /* bInCollection */
    0x01,                   /* baInterfaceNr */

    /* Output Terminal Descriptor */
    0x09,   /* bLength */
    0x24,   /* bDescriptorType */
    0x03,   /* bDescriptorSubType */
    0x03,   /* bTerminalID */
    0x01, 0x01,   /* wTerminalType */
    0x00,   /* bAssocTerminal */
    0x05,   /* bSourceID */
    0x00,   /* iTerminal */

    /* Input Terminal Descriptor (Camera) */
    0x11,        /* bLength */
    0x24,        /* bDescriptorType */
    0x02,        /* bDescriptorSubType */
    0x01,        /* bTerminalID */
    0x01, 0x02,  /* wTerminalType */
    0x00,        /* bAssocTerminal */
    0x00,        /* iTerminal */
    0x00, 0x00,  /* wObjectiveFocalLengthMin */
    0x00, 0x00,  /* wObjectiveFocalLengthMax */
    0x00, 0x00,  /* wOcularFocalLength */
    0x02,        /* bControlSize */
    0x00, 0x00,  /* bmControls */

    /*  Processing Uint Descriptor  */
    0x0B,       /* bLength */
    0x24,       /* bDescriptorType */
    0x05,       /* bDescriptorSubType */
    0x05,       /* bUnitID */
    0x01,       /* bSourceID */
    0x00, 0x00, /* wMaxMultiplier */
    0x02,       /* bControlSize */
    0x3F, 0x05, /* bmControls */
    0x00,       /* iProcessing */

    /*  Standard Interrupt Endpoint Descriptor */
    0x07,       /* bLength */
    0x05,       /* bDescriptorType */
    (INT_IN_EP_NUM | EP_INPUT),    /* bEndpointAddress */
    EP_INT,   /* bmAttributes */
    EPC_MAX_PKT_SIZE & 0x00FF,
    (EPC_MAX_PKT_SIZE & 0xFF00) >> 8,
    0x20,       /* bInterval */

    /*  Class-specific Interrupt Endpoint Descriptor */
    0x05,   /* bLength */
    0x25,   /* bDescriptorType */
    0x03,   /* bDescriptorSubType */
    0x08, 0x00,

    /*  Standard VideoStreaming Interface Descriptor */
    0x09,   /* bLength */
    0x04,   /* bDescriptorType */
    0x01,   /* bInterfceNumber */
    0x00,   /* bAlternateSetting */
    0x00,   /* bNumEndpoints */
    0x0E,   /* bInterfaceClass */
    0x02,   /* bInterfaceSubClass */
    0x00,   /* bInterfaceProtocol */
    0x00,   /* iInterface */

    /*  Class-specific VideoStreaming Header Descriptor */
    0x0E,        /* bLength */
    0x24,        /* bDescriptorType */
    0x01,        /* bDescriptorSubType */
    0x01,        /* bNumFormats, MJPEG and still format */
    0xD7, 0x00,
    0x81,        /* bEndpointAddress */
    0x00,        /* bmInfo */
    0x03,        /* bTerminalLink */
    0x02,        /* bStillCaptureMethod, method 2 */
    0x01,        /* bTriggerSupport */
    0x00,        /* bTriggerUsage */
    0x01,        /* bControlSize */
    0x00,        /* bmaControls(1) */

    /* Class-specific VideoStreaming Format Descriptor */
    /* Uncompressed Video YUV422 */
    0x1B,   /* bLength */
    0x24,   /* bDescriptorType */
    0x04,   /* bDescriptorSubType */
    0x01,   /* bFormatIndex */
    0x02,   /* bNumFrameDescriptors */
#if (UVC_Color_Format == UVC_Format_YUY2)
//YUYV GUID: MEDIASUBTYPE_YUY2
	0x59, 0x55, 0x59, 0x32, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71,
#endif
#if (UVC_Color_Format == UVC_Format_RGB565)
//RGB565 GUID: MEDIASUBTYPE_RGB565
	0x7b, 0xeb, 0x36, 0xe4, 0x4f, 0x52, 0xce, 0x11, 0x9f, 0x53, 0x00, 0x20, 0xaf, 0x0b, 0xa7, 0x70,
#endif
	0x10,
    0x01,   /* bDefaultFrameIndex */
    0x00,   /* bAspectRatioX */
    0x00,   /* bAspectRatioY */
    0x00,   /* bmInterlaceFlags */
    0x00,   /* bCopyProtect */

    /* Class-specific VideoStreaming Frame 1 Descriptor */
    0x32,           //bLength
    0x24,           //bDescriptorType
    0x05,           //bDescriptorSubType
    0x01,           //bFrameIndex           // Modified Here
    0x00,           //bmCapabilities
    0x40, 0x01,         //wWidth->320
    0xF0, 0x00,         //wHeight->240
    0x00, 0x58, 0x02, 0x00,     //dwMinBitRate
    0x00, 0x50, 0x46, 0x00,     //dwMaxBitRate
    0x00, 0x58, 0x02, 0x00,     //dwMaxVideoFrameBufSize
    0x15, 0x16, 0x05, 0x00,     //dwDefaultFrameInterval
    0x06,           //bFrameIntervalType
    0x15, 0x16, 0x05, 0x00,     //dwMinFrameInterval
    0x20, 0xA1, 0x07, 0x00,     //dwMinFrameInterval
    0x2A, 0x2C, 0x0A, 0x00,     //dwMinFrameInterval
    0x40, 0x42, 0x0F, 0x00,     //dwMinFrameInterval
    0x80, 0x84, 0x1E, 0x00,     //dwMinFrameInterval
    0x80, 0x96, 0x98, 0x00,      //dwMinFrameInterval

    /* Class-specific VideoStreaming Frame 2 Descriptor */
    0x32,           //bLength
    0x24,           //bDescriptorType
    0x05,           //bDescriptorSubType
    0x02,           //bFrameIndex           // Modified Here
    0x00,           //bmCapabilities
    0xA0, 0x00,          //wWidth->160
    0x78, 0x00,          //wHeight->120
    0x00, 0x96, 0x00, 0x00,     //dwMinBitRate
    0x00, 0x94, 0x11, 0x00,     //dwMaxBitRate
    0x00, 0x96, 0x00, 0x00,     //dwMaxVideoFrameBufSize
    0x15, 0x16, 0x05, 0x00,     //dwDefaultFrameInterval
    0x06,           //bFrameIntervalType
    0x15, 0x16, 0x05, 0x00,     //dwMinFrameInterval
    0x20, 0xA1, 0x07, 0x00,     //dwMinFrameInterval
    0x2A, 0x2C, 0x0A, 0x00,     //dwMinFrameInterval
    0x40, 0x42, 0x0F, 0x00,     //dwMinFrameInterval
    0x80, 0x84, 0x1E, 0x00,     //dwMinFrameInterval
    0x80, 0x96, 0x98, 0x00,      //dwMinFrameInterval

#if 1
    /* Class-specific VideoStreaming Frame 3 Descriptor */
    0x32,           /* bLength */
    0x24,           /* bDescriptorType */
    0x05,           /* bDescriptorSubType */
    0x03,           /* bFrameIndex */
    0x00,           /* bmCapabilities */

    0x80, 0x02,         //wWidth->640
    0xE0, 0x01,         //wHeight->480
    0x00, 0x60, 0x09, 0x00,     //dwMinBitRate
    0x00, 0x40, 0x19, 0x01,     //dwMaxBitRate
    0x00, 0x60, 0x09, 0x00,     //dwMaxVideoFrameBufSize
    0x15, 0x16, 0x05, 0x00,     //dwDefaultFrameInterval
    0x06,           //bFrameIntervalType
    0x15, 0x16, 0x05, 0x00,     //dwMinFrameInterval
    0x20, 0xA1, 0x07, 0x00,     //dwMinFrameInterval
    0x2A, 0x2C, 0x0A, 0x00,     //dwMinFrameInterval
    0x40, 0x42, 0x0F, 0x00,     //dwMinFrameInterval
    0x80, 0x84, 0x1E, 0x00,     //dwMinFrameInterval
    0x80, 0x96, 0x98, 0x00,      //dwMinFrameInterval
#endif

    /* Class-specific Still Image Frame Descriptor */
    0x12,           //bLength
    0x24,           //bDescriptorType
    0x03,           //bDescriptorSubType
    0x00,           //bEndpointAddress
    0x03,           //bNumImageSizePatterns
    0x80, 0x02,         //wWidth
    0xE0, 0x01,         //wHeight
    0x40, 0x01,         //wWidth
    0xF0, 0x00,         //wHeight
    0xA0, 0x00,         //wWidth
    0x78, 0x00,         //wHeight
    0x00,            //bNumCompressionPtn

    /* Color Matching Descriptor */
    0x06,           //bLength
    0x24,           //bDescriptorType
    0x0D,           //bDescriptorSubType
    0x01,           //bColorPrimaries
    0x01,           //bTransferCharacteristics
    0x04,            //bMatrixCoefficients

    /* Standard VideoStreaming Interface Descriptor  (Num 1, Alt 3) */
    0x09,   //bLength
    0x04,   //bDescriptorType
    0x01,   //bInterfceNumber
    0x01,   //bAlternateSetting
    0x01,   //bNumEndpoints
    0x0E,   //bInterfaceClass
    0x02,   //bInterfaceSubClass
    0x00,   //bInterfaceProtocol
    0x00,   //iInterface

    /* Standard VideoStreaming Iso Video Data Endpoint Descriptor */
    0x07,   /* bLength */
    0x05,   /* bDescriptorType */
    (ISO_IN_EP_NUM | EP_INPUT),    /* bEndpointAddress */
    0x05,   /* bmAttributes */
    //0x90, 0x12,
    EPA_MAX_PKT_SIZE & 0x00FF,
    (EPA_MAX_PKT_SIZE & 0xFF00) >> 8,
    0x01,   /* bInterval */

    /* Standard VideoStreaming Interface Descriptor  (Num 1, Alt 4) */
    0x09,   //bLength
    0x04,   //bDescriptorType
    0x01,   //bInterfceNumber
    0x02,   //bAlternateSetting
    0x01,   //bNumEndpoints
    0x0E,   //bInterfaceClass
    0x02,   //bInterfaceSubClass
    0x00,   //bInterfaceProtocol
    0x00,    //iInterface

    /* Standard VideoStreaming Iso Video Data Endpoint Descriptor */
    0x07,   /* bLength */
    0x05,   /* bDescriptorType */
    (ISO_IN_EP_NUM | EP_INPUT),    /* bEndpointAddress */
    0x05,   /* bmAttributes */
    //0x90, 0x12,
    EPA_MAX_PKT_SIZE & 0x00FF,
    (EPA_MAX_PKT_SIZE & 0xFF00) >> 8,
    0x01,   /* bInterval */

};

uint8_t gu8OtherConfigDescriptorHS[] __attribute__((aligned(DCACHE_LINE_SIZE))) =
{
    LEN_CONFIG,     /* bLength              */
    DESC_OTHERSPEED,/* bDescriptorType      */
    0x58, 0x01,

    0x02,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x00,           /* iConfiguration */
    0xC0,           /* bmAttributes */
    0x00,           /* MaxPower */

    /*  Standard Video Interface Collection IAD(interface Association Descriptor) */
    0x08,   /* bLength */
    0x0B,   /* bDescriptorType */
    0x00,   /* bFirstInterface */
    0x02,   /* bInterfaceCount */
    0x0E,   /* bFunctionClass */
    0x03,   /* bFunctionSubClass */
    0x00,   /* bFunctionProtocol */
    0x02,   /* iFunction */

    /* Standard VideoControl Interface Descriptor */
    0x09,   /* bLength */
    0x04,   /* bDescriptorType */
    0x00,   /* bInterfceNumber */
    0x00,   /* bAlternateSetting */
    0x01,   /* bNumEndpoints */
    0x0E,   /* bInterfaceClass */
    0x01,   /* bInterfaceSubClass */
    0x00,   /* bInterfaceProtocol */
    0x02,   /* iInterface */

    /* Class-specific VideoControl Interface Descriptor */
    0x0D,                   /* bLength */
    0x24,                   /* bDescriptorType */
    0x01,                   /* bDescriptorSubType */
    0x00, 0x01,             /* bcdUAVC */
    0x32, 0x00,             /* wTotalLength */
    0x80, 0x8D, 0x5B, 0x00, /* dwClockFrequency */
    0x01,                   /* bInCollection */
    0x01,                   /* baInterfaceNr */

    /* Output Terminal Descriptor */
    0x09,   /* bLength */
    0x24,   /* bDescriptorType */
    0x03,   /* bDescriptorSubType */
    0x03,   /* bTerminalID */
    0x01, 0x01,   /* wTerminalType */
    0x00,   /* bAssocTerminal */
    0x05,   /* bSourceID */
    0x00,   /* iTerminal */

    /* Input Terminal Descriptor (Camera) */
    0x11,        /* bLength */
    0x24,        /* bDescriptorType */
    0x02,        /* bDescriptorSubType */
    0x01,        /* bTerminalID */
    0x01, 0x02,  /* wTerminalType */
    0x00,        /* bAssocTerminal */
    0x00,        /* iTerminal */
    0x00, 0x00,  /* wObjectiveFocalLengthMin */
    0x00, 0x00,  /* wObjectiveFocalLengthMax */
    0x00, 0x00,  /* wOcularFocalLength */
    0x02,        /* bControlSize */
    0x00, 0x00,  /* bmControls */

    /*  Processing Uint Descriptor  */
    0x0B,       /* bLength */
    0x24,       /* bDescriptorType */
    0x05,       /* bDescriptorSubType */
    0x05,       /* bUnitID */
    0x01,       /* bSourceID */
    0x00, 0x00, /* wMaxMultiplier */
    0x02,       /* bControlSize */
    0x3F, 0x05, /* bmControls */
    0x00,       /* iProcessing */

    /*  Standard Interrupt Endpoint Descriptor */
    0x07,       /* bLength */
    0x05,       /* bDescriptorType */
    (INT_IN_EP_NUM | EP_INPUT),    /* bEndpointAddress */
    EP_INT,   /* bmAttributes */
    EPC_MAX_PKT_SIZE & 0x00FF,
    (EPC_MAX_PKT_SIZE & 0xFF00) >> 8,
    0x20,       /* bInterval */

    /*  Class-specific Interrupt Endpoint Descriptor */
    0x05,   /* bLength */
    0x25,   /* bDescriptorType */
    0x03,   /* bDescriptorSubType */
    0x08, 0x00,

    /*  Standard VideoStreaming Interface Descriptor */
    0x09,   /* bLength */
    0x04,   /* bDescriptorType */
    0x01,   /* bInterfceNumber */
    0x00,   /* bAlternateSetting */
    0x00,   /* bNumEndpoints */
    0x0E,   /* bInterfaceClass */
    0x02,   /* bInterfaceSubClass */
    0x00,   /* bInterfaceProtocol */
    0x00,   /* iInterface */

    /*  Class-specific VideoStreaming Header Descriptor */
    0x0E,        /* bLength */
    0x24,        /* bDescriptorType */
    0x01,        /* bDescriptorSubType */
    0x01,        /* bNumFormats, MJPEG and still format */
    0xD7, 0x00,
    0x81,        /* bEndpointAddress */
    0x00,        /* bmInfo */
    0x03,        /* bTerminalLink */
    0x02,        /* bStillCaptureMethod, method 2 */
    0x01,        /* bTriggerSupport */
    0x00,        /* bTriggerUsage */
    0x01,        /* bControlSize */
    0x00,        /* bmaControls(1) */

    /* Class-specific VideoStreaming Format Descriptor */
    /* Uncompressed Video YUV422 */
    0x1B,   /* bLength */
    0x24,   /* bDescriptorType */
    0x04,   /* bDescriptorSubType */
    0x01,   /* bFormatIndex */
    0x02,   /* bNumFrameDescriptors */
#if (UVC_Color_Format == UVC_Format_YUY2)
	//YUYV
	0x59, 0x55, 0x59, 0x32, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71,
#endif
#if (UVC_Color_Format == UVC_Format_RGB565)
	//RGB565
	0x7b, 0xeb, 0x36, 0xe4, 0x4f, 0x52, 0xce, 0x11, 0x9f, 0x53, 0x00, 0x20, 0xaf, 0x0b, 0xa7, 0x70,
#endif
	0x10,
    0x02,   /* bDefaultFrameIndex */
    0x00,   /* bAspectRatioX */
    0x00,   /* bAspectRatioY */
    0x00,   /* bmInterlaceFlags */
    0x00,   /* bCopyProtect */

    /* Class-specific VideoStreaming Frame 1 Descriptor */
    0x32,           //bLength
    0x24,           //bDescriptorType
    0x05,           //bDescriptorSubType
    0x01,           //bFrameIndex           // Modified Here
    0x00,           //bmCapabilities
    0x40, 0x01,         //wWidth->320
    0xF0, 0x00,         //wHeight->240
    0x00, 0x58, 0x02, 0x00,     //dwMinBitRate
    0x00, 0x50, 0x46, 0x00,     //dwMaxBitRate
    0x00, 0x58, 0x02, 0x00,     //dwMaxVideoFrameBufSize
    0x15, 0x16, 0x05, 0x00,     //dwDefaultFrameInterval
    0x06,           //bFrameIntervalType
    0x15, 0x16, 0x05, 0x00,     //dwMinFrameInterval
    0x20, 0xA1, 0x07, 0x00,     //dwMinFrameInterval
    0x2A, 0x2C, 0x0A, 0x00,     //dwMinFrameInterval
    0x40, 0x42, 0x0F, 0x00,     //dwMinFrameInterval
    0x80, 0x84, 0x1E, 0x00,     //dwMinFrameInterval
    0x80, 0x96, 0x98, 0x00,      //dwMinFrameInterval

    /* Class-specific VideoStreaming Frame 2 Descriptor */
    0x32,           //bLength
    0x24,           //bDescriptorType
    0x05,           //bDescriptorSubType
    0x02,           //bFrameIndex           // Modified Here
    0x00,           //bmCapabilities
    0xA0, 0x00,          //wWidth->160
    0x78, 0x00,          //wHeight->120
    0x00, 0x96, 0x00, 0x00,     //dwMinBitRate
    0x00, 0x94, 0x11, 0x00,     //dwMaxBitRate
    0x00, 0x96, 0x00, 0x00,     //dwMaxVideoFrameBufSize
    0x15, 0x16, 0x05, 0x00,     //dwDefaultFrameInterval
    0x06,           //bFrameIntervalType
    0x15, 0x16, 0x05, 0x00,     //dwMinFrameInterval
    0x20, 0xA1, 0x07, 0x00,     //dwMinFrameInterval
    0x2A, 0x2C, 0x0A, 0x00,     //dwMinFrameInterval
    0x40, 0x42, 0x0F, 0x00,     //dwMinFrameInterval
    0x80, 0x84, 0x1E, 0x00,     //dwMinFrameInterval
    0x80, 0x96, 0x98, 0x00,      //dwMinFrameInterval

    /* Class-specific VideoStreaming Frame 3 Descriptor */
    0x32,           /* bLength */
    0x24,           /* bDescriptorType */
    0x05,           /* bDescriptorSubType */
    0x03,           /* bFrameIndex */
    0x00,           /* bmCapabilities */

    0x80, 0x02,         //wWidth->640
    0xE0, 0x01,         //wHeight->480
    0x00, 0x60, 0x09, 0x00,     //dwMinBitRate
    0x00, 0x40, 0x19, 0x01,     //dwMaxBitRate
    0x00, 0x60, 0x09, 0x00,     //dwMaxVideoFrameBufSize
    0x15, 0x16, 0x05, 0x00,     //dwDefaultFrameInterval
    0x06,           //bFrameIntervalType
    0x15, 0x16, 0x05, 0x00,     //dwMinFrameInterval
    0x20, 0xA1, 0x07, 0x00,     //dwMinFrameInterval
    0x2A, 0x2C, 0x0A, 0x00,     //dwMinFrameInterval
    0x40, 0x42, 0x0F, 0x00,     //dwMinFrameInterval
    0x80, 0x84, 0x1E, 0x00,     //dwMinFrameInterval
    0x80, 0x96, 0x98, 0x00,      //dwMinFrameInterval

	/* Class-specific Still Image Frame Descriptor */
    0x12,           //bLength
    0x24,           //bDescriptorType
    0x03,           //bDescriptorSubType
    0x00,           //bEndpointAddress
    0x03,           //bNumImageSizePatterns
    0x80, 0x02,         //wWidth
    0xE0, 0x01,         //wHeight
    0x40, 0x01,         //wWidth
    0xF0, 0x00,         //wHeight
    0xA0, 0x00,         //wWidth
    0x78, 0x00,         //wHeight
    0x00,            //bNumCompressionPtn

    /* Color Matching Descriptor */
    0x06,           //bLength
    0x24,           //bDescriptorType
    0x0D,           //bDescriptorSubType
    0x01,           //bColorPrimaries
    0x01,           //bTransferCharacteristics
    0x04,            //bMatrixCoefficients

    /* Standard VideoStreaming Interface Descriptor  (Num 1, Alt 3) */
    0x09,   //bLength
    0x04,   //bDescriptorType
    0x01,   //bInterfceNumber
    0x01,   //bAlternateSetting
    0x01,   //bNumEndpoints
    0x0E,   //bInterfaceClass
    0x02,   //bInterfaceSubClass
    0x00,   //bInterfaceProtocol
    0x00,   //iInterface

    /* Standard VideoStreaming Iso Video Data Endpoint Descriptor */
    0x07,   /* bLength */
    0x05,   /* bDescriptorType */
    (ISO_IN_EP_NUM | EP_INPUT),    /* bEndpointAddress */
    0x05,   /* bmAttributes */
    //0x90, 0x12,
    EPA_MAX_PKT_SIZE & 0x00FF,
    (EPA_MAX_PKT_SIZE & 0xFF00) >> 8,
    0x01,   /* bInterval */

    /* Standard VideoStreaming Interface Descriptor  (Num 1, Alt 4) */
    0x09,   //bLength
    0x04,   //bDescriptorType
    0x01,   //bInterfceNumber
    0x02,   //bAlternateSetting
    0x01,   //bNumEndpoints
    0x0E,   //bInterfaceClass
    0x02,   //bInterfaceSubClass
    0x00,   //bInterfaceProtocol
    0x00,    //iInterface

    /* Standard VideoStreaming Iso Video Data Endpoint Descriptor */
    0x07,   /* bLength */
    0x05,   /* bDescriptorType */
    (ISO_IN_EP_NUM | EP_INPUT),    /* bEndpointAddress */
    0x05,   /* bmAttributes */
    //0x90, 0x12,
    EPA_MAX_PKT_SIZE & 0x00FF,
    (EPA_MAX_PKT_SIZE & 0xFF00) >> 8,
    0x01,   /* bInterval */

};


/*!<USB Other Speed Configure Descriptor */
uint8_t gu8ConfigDescriptorFS[] __attribute__((aligned(DCACHE_LINE_SIZE))) =
{
    LEN_CONFIG,     /* bLength              */
    DESC_CONFIG,    /* bDescriptorType      */
    0x58, 0x01,

    0x02,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x00,           /* iConfiguration */
    0xC0,           /* bmAttributes */
    0x00,           /* MaxPower */

    /*  Standard Video Interface Collection IAD(interface Association Descriptor) */
    0x08,   /* bLength */
    0x0B,   /* bDescriptorType */
    0x00,   /* bFirstInterface */
    0x02,   /* bInterfaceCount */
    0x0E,   /* bFunctionClass */
    0x03,   /* bFunctionSubClass */
    0x00,   /* bFunctionProtocol */
    0x02,   /* iFunction */

    /* Standard VideoControl Interface Descriptor */
    0x09,   /* bLength */
    0x04,   /* bDescriptorType */
    0x00,   /* bInterfceNumber */
    0x00,   /* bAlternateSetting */
    0x01,   /* bNumEndpoints */
    0x0E,   /* bInterfaceClass */
    0x01,   /* bInterfaceSubClass */
    0x00,   /* bInterfaceProtocol */
    0x02,   /* iInterface */

    /* Class-specific VideoControl Interface Descriptor */
    0x0D,                   /* bLength */
    0x24,                   /* bDescriptorType */
    0x01,                   /* bDescriptorSubType */
    0x00, 0x01,             /* bcdUAVC */
    0x32, 0x00,             /* wTotalLength */
    0x80, 0x8D, 0x5B, 0x00, /* dwClockFrequency */
    0x01,                   /* bInCollection */
    0x01,                   /* baInterfaceNr */

    /* Output Terminal Descriptor */
    0x09,   /* bLength */
    0x24,   /* bDescriptorType */
    0x03,   /* bDescriptorSubType */
    0x03,   /* bTerminalID */
    0x01, 0x01,   /* wTerminalType */
    0x00,   /* bAssocTerminal */
    0x05,   /* bSourceID */
    0x00,   /* iTerminal */

    /* Input Terminal Descriptor (Camera) */
    0x11,        /* bLength */
    0x24,        /* bDescriptorType */
    0x02,        /* bDescriptorSubType */
    0x01,        /* bTerminalID */
    0x01, 0x02,  /* wTerminalType */
    0x00,        /* bAssocTerminal */
    0x00,        /* iTerminal */
    0x00, 0x00,  /* wObjectiveFocalLengthMin */
    0x00, 0x00,  /* wObjectiveFocalLengthMax */
    0x00, 0x00,  /* wOcularFocalLength */
    0x02,        /* bControlSize */
    0x00, 0x00,  /* bmControls */

    /*  Processing Uint Descriptor  */
    0x0B,       /* bLength */
    0x24,       /* bDescriptorType */
    0x05,       /* bDescriptorSubType */
    0x05,       /* bUnitID */
    0x01,       /* bSourceID */
    0x00, 0x00, /* wMaxMultiplier */
    0x02,       /* bControlSize */
    0x3F, 0x05, /* bmControls */
    0x00,       /* iProcessing */

    /*  Standard Interrupt Endpoint Descriptor */
    0x07,       /* bLength */
    0x05,       /* bDescriptorType */
    (INT_IN_EP_NUM | EP_INPUT),    /* bEndpointAddress */
    EP_INT,   /* bmAttributes */
    EPC_MAX_PKT_SIZE & 0x00FF,
    (EPC_MAX_PKT_SIZE & 0xFF00) >> 8,
    0x20,       /* bInterval */

    /*  Class-specific Interrupt Endpoint Descriptor */
    0x05,   /* bLength */
    0x25,   /* bDescriptorType */
    0x03,   /* bDescriptorSubType */
    0x08, 0x00,

    /*  Standard VideoStreaming Interface Descriptor */
    0x09,   /* bLength */
    0x04,   /* bDescriptorType */
    0x01,   /* bInterfceNumber */
    0x00,   /* bAlternateSetting */
    0x00,   /* bNumEndpoints */
    0x0E,   /* bInterfaceClass */
    0x02,   /* bInterfaceSubClass */
    0x00,   /* bInterfaceProtocol */
    0x00,   /* iInterface */

    /*  Class-specific VideoStreaming Header Descriptor */
    0x0E,        /* bLength */
    0x24,        /* bDescriptorType */
    0x01,        /* bDescriptorSubType */
    0x01,        /* bNumFormats, MJPEG and still format */
    0xD7, 0x00,
    0x81,        /* bEndpointAddress */
    0x00,        /* bmInfo */
    0x03,        /* bTerminalLink */
    0x02,        /* bStillCaptureMethod, method 2 */
    0x01,        /* bTriggerSupport */
    0x00,        /* bTriggerUsage */
    0x01,        /* bControlSize */
    0x00,        /* bmaControls(1) */

    /* Class-specific VideoStreaming Format Descriptor */
    /* Uncompressed Video YUV422 */
    0x1B,   /* bLength */
    0x24,   /* bDescriptorType */
    0x04,   /* bDescriptorSubType */
    0x01,   /* bFormatIndex */
    0x02,   /* bNumFrameDescriptors */
#if (UVC_Color_Format == UVC_Format_YUY2)
	//YUYV
	0x59, 0x55, 0x59, 0x32, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71,
#endif
#if (UVC_Color_Format == UVC_Format_RGB565)
	//RGB565
	0x7b, 0xeb, 0x36, 0xe4, 0x4f, 0x52, 0xce, 0x11, 0x9f, 0x53, 0x00, 0x20, 0xaf, 0x0b, 0xa7, 0x70,
#endif
	0x10,
    0x02,   /* bDefaultFrameIndex */
    0x00,   /* bAspectRatioX */
    0x00,   /* bAspectRatioY */
    0x00,   /* bmInterlaceFlags */
    0x00,   /* bCopyProtect */

    /* Class-specific VideoStreaming Frame 1 Descriptor */
    0x32,           //bLength
    0x24,           //bDescriptorType
    0x05,           //bDescriptorSubType
    0x01,           //bFrameIndex           // Modified Here
    0x00,           //bmCapabilities
    0x40, 0x01,         //wWidth->320
    0xF0, 0x00,         //wHeight->240
    0x00, 0x58, 0x02, 0x00,     //dwMinBitRate
    0x00, 0x50, 0x46, 0x00,     //dwMaxBitRate
    0x00, 0x58, 0x02, 0x00,     //dwMaxVideoFrameBufSize
    0x15, 0x16, 0x05, 0x00,     //dwDefaultFrameInterval
    0x06,           //bFrameIntervalType
    0x15, 0x16, 0x05, 0x00,     //dwMinFrameInterval
    0x20, 0xA1, 0x07, 0x00,     //dwMinFrameInterval
    0x2A, 0x2C, 0x0A, 0x00,     //dwMinFrameInterval
    0x40, 0x42, 0x0F, 0x00,     //dwMinFrameInterval
    0x80, 0x84, 0x1E, 0x00,     //dwMinFrameInterval
    0x80, 0x96, 0x98, 0x00,      //dwMinFrameInterval

    /* Class-specific VideoStreaming Frame 2 Descriptor */
    0x32,           //bLength
    0x24,           //bDescriptorType
    0x05,           //bDescriptorSubType
    0x02,           //bFrameIndex           // Modified Here
    0x00,           //bmCapabilities
    0xA0, 0x00,          //wWidth->160
    0x78, 0x00,          //wHeight->120
    0x00, 0x96, 0x00, 0x00,     //dwMinBitRate
    0x00, 0x94, 0x11, 0x00,     //dwMaxBitRate
    0x00, 0x96, 0x00, 0x00,     //dwMaxVideoFrameBufSize
    0x15, 0x16, 0x05, 0x00,     //dwDefaultFrameInterval
    0x06,           //bFrameIntervalType
    0x15, 0x16, 0x05, 0x00,     //dwMinFrameInterval
    0x20, 0xA1, 0x07, 0x00,     //dwMinFrameInterval
    0x2A, 0x2C, 0x0A, 0x00,     //dwMinFrameInterval
    0x40, 0x42, 0x0F, 0x00,     //dwMinFrameInterval
    0x80, 0x84, 0x1E, 0x00,     //dwMinFrameInterval
    0x80, 0x96, 0x98, 0x00,      //dwMinFrameInterval

    /* Class-specific VideoStreaming Frame 3 Descriptor */
    0x32,           /* bLength */
    0x24,           /* bDescriptorType */
    0x05,           /* bDescriptorSubType */
    0x03,           /* bFrameIndex */
    0x00,           /* bmCapabilities */

    0x80, 0x02,         //wWidth->640
    0xE0, 0x01,         //wHeight->480
    0x00, 0x60, 0x09, 0x00,     //dwMinBitRate
    0x00, 0x40, 0x19, 0x01,     //dwMaxBitRate
    0x00, 0x60, 0x09, 0x00,     //dwMaxVideoFrameBufSize
    0x15, 0x16, 0x05, 0x00,     //dwDefaultFrameInterval
    0x06,           //bFrameIntervalType
    0x15, 0x16, 0x05, 0x00,     //dwMinFrameInterval
    0x20, 0xA1, 0x07, 0x00,     //dwMinFrameInterval
    0x2A, 0x2C, 0x0A, 0x00,     //dwMinFrameInterval
    0x40, 0x42, 0x0F, 0x00,     //dwMinFrameInterval
    0x80, 0x84, 0x1E, 0x00,     //dwMinFrameInterval
    0x80, 0x96, 0x98, 0x00,      //dwMinFrameInterval

	/* Class-specific Still Image Frame Descriptor */
    0x12,           //bLength
    0x24,           //bDescriptorType
    0x03,           //bDescriptorSubType
    0x00,           //bEndpointAddress
    0x03,           //bNumImageSizePatterns
    0x80, 0x02,         //wWidth
    0xE0, 0x01,         //wHeight
    0x40, 0x01,         //wWidth
    0xF0, 0x00,         //wHeight
    0xA0, 0x00,         //wWidth
    0x78, 0x00,         //wHeight
    0x00,            //bNumCompressionPtn

    /* Color Matching Descriptor */
    0x06,           //bLength
    0x24,           //bDescriptorType
    0x0D,           //bDescriptorSubType
    0x01,           //bColorPrimaries
    0x01,           //bTransferCharacteristics
    0x04,            //bMatrixCoefficients

    /* Standard VideoStreaming Interface Descriptor  (Num 1, Alt 3) */
    0x09,   //bLength
    0x04,   //bDescriptorType
    0x01,   //bInterfceNumber
    0x01,   //bAlternateSetting
    0x01,   //bNumEndpoints
    0x0E,   //bInterfaceClass
    0x02,   //bInterfaceSubClass
    0x00,   //bInterfaceProtocol
    0x00,   //iInterface

    /* Standard VideoStreaming Iso Video Data Endpoint Descriptor */
    0x07,   /* bLength */
    0x05,   /* bDescriptorType */
    (ISO_IN_EP_NUM | EP_INPUT),    /* bEndpointAddress */
    0x05,   /* bmAttributes */
    //0x90, 0x12,
    EPA_MAX_PKT_SIZE & 0x00FF,
    (EPA_MAX_PKT_SIZE & 0xFF00) >> 8,
    0x01,   /* bInterval */

    /* Standard VideoStreaming Interface Descriptor  (Num 1, Alt 4) */
    0x09,   //bLength
    0x04,   //bDescriptorType
    0x01,   //bInterfceNumber
    0x02,   //bAlternateSetting
    0x01,   //bNumEndpoints
    0x0E,   //bInterfaceClass
    0x02,   //bInterfaceSubClass
    0x00,   //bInterfaceProtocol
    0x00,    //iInterface

    /* Standard VideoStreaming Iso Video Data Endpoint Descriptor */
    0x07,   /* bLength */
    0x05,   /* bDescriptorType */
    (ISO_IN_EP_NUM | EP_INPUT),    /* bEndpointAddress */
    0x05,   /* bmAttributes */
    //0x90, 0x12,
    EPA_MAX_PKT_SIZE & 0x00FF,
    (EPA_MAX_PKT_SIZE & 0xFF00) >> 8,
    0x01,   /* bInterval */

};

uint8_t gu8OtherConfigDescriptorFS[] __attribute__((aligned(DCACHE_LINE_SIZE))) =
{
    LEN_CONFIG,     /* bLength              */
    DESC_OTHERSPEED,/* bDescriptorType      */
    0x58, 0x01,

    0x02,           /* bNumInterfaces */
    0x01,           /* bConfigurationValue */
    0x00,           /* iConfiguration */
    0xC0,           /* bmAttributes */
    0x00,           /* MaxPower */

    /*  Standard Video Interface Collection IAD(interface Association Descriptor) */
    0x08,   /* bLength */
    0x0B,   /* bDescriptorType */
    0x00,   /* bFirstInterface */
    0x02,   /* bInterfaceCount */
    0x0E,   /* bFunctionClass */
    0x03,   /* bFunctionSubClass */
    0x00,   /* bFunctionProtocol */
    0x02,   /* iFunction */

    /* Standard VideoControl Interface Descriptor */
    0x09,   /* bLength */
    0x04,   /* bDescriptorType */
    0x00,   /* bInterfceNumber */
    0x00,   /* bAlternateSetting */
    0x01,   /* bNumEndpoints */
    0x0E,   /* bInterfaceClass */
    0x01,   /* bInterfaceSubClass */
    0x00,   /* bInterfaceProtocol */
    0x02,   /* iInterface */

    /* Class-specific VideoControl Interface Descriptor */
    0x0D,                   /* bLength */
    0x24,                   /* bDescriptorType */
    0x01,                   /* bDescriptorSubType */
    0x00, 0x01,             /* bcdUAVC */
    0x32, 0x00,             /* wTotalLength */
    0x80, 0x8D, 0x5B, 0x00, /* dwClockFrequency */
    0x01,                   /* bInCollection */
    0x01,                   /* baInterfaceNr */

    /* Output Terminal Descriptor */
    0x09,   /* bLength */
    0x24,   /* bDescriptorType */
    0x03,   /* bDescriptorSubType */
    0x03,   /* bTerminalID */
    0x01, 0x01,   /* wTerminalType */
    0x00,   /* bAssocTerminal */
    0x05,   /* bSourceID */
    0x00,   /* iTerminal */

    /* Input Terminal Descriptor (Camera) */
    0x11,        /* bLength */
    0x24,        /* bDescriptorType */
    0x02,        /* bDescriptorSubType */
    0x01,        /* bTerminalID */
    0x01, 0x02,  /* wTerminalType */
    0x00,        /* bAssocTerminal */
    0x00,        /* iTerminal */
    0x00, 0x00,  /* wObjectiveFocalLengthMin */
    0x00, 0x00,  /* wObjectiveFocalLengthMax */
    0x00, 0x00,  /* wOcularFocalLength */
    0x02,        /* bControlSize */
    0x00, 0x00,  /* bmControls */

    /*  Processing Uint Descriptor  */
    0x0B,       /* bLength */
    0x24,       /* bDescriptorType */
    0x05,       /* bDescriptorSubType */
    0x05,       /* bUnitID */
    0x01,       /* bSourceID */
    0x00, 0x00, /* wMaxMultiplier */
    0x02,       /* bControlSize */
    0x3F, 0x05, /* bmControls */
    0x00,       /* iProcessing */

    /*  Standard Interrupt Endpoint Descriptor */
    0x07,       /* bLength */
    0x05,       /* bDescriptorType */
    (INT_IN_EP_NUM | EP_INPUT),    /* bEndpointAddress */
    EP_INT,   /* bmAttributes */
    EPC_MAX_PKT_SIZE & 0x00FF,
    (EPC_MAX_PKT_SIZE & 0xFF00) >> 8,
    0x20,       /* bInterval */

    /*  Class-specific Interrupt Endpoint Descriptor */
    0x05,   /* bLength */
    0x25,   /* bDescriptorType */
    0x03,   /* bDescriptorSubType */
    0x08, 0x00,

    /*  Standard VideoStreaming Interface Descriptor */
    0x09,   /* bLength */
    0x04,   /* bDescriptorType */
    0x01,   /* bInterfceNumber */
    0x00,   /* bAlternateSetting */
    0x00,   /* bNumEndpoints */
    0x0E,   /* bInterfaceClass */
    0x02,   /* bInterfaceSubClass */
    0x00,   /* bInterfaceProtocol */
    0x00,   /* iInterface */

    /*  Class-specific VideoStreaming Header Descriptor */
    0x0E,        /* bLength */
    0x24,        /* bDescriptorType */
    0x01,        /* bDescriptorSubType */
    0x01,        /* bNumFormats, MJPEG and still format */
    0xD7, 0x00,
    0x81,        /* bEndpointAddress */
    0x00,        /* bmInfo */
    0x03,        /* bTerminalLink */
    0x02,        /* bStillCaptureMethod, method 2 */
    0x01,        /* bTriggerSupport */
    0x00,        /* bTriggerUsage */
    0x01,        /* bControlSize */
    0x00,        /* bmaControls(1) */

    /* Class-specific VideoStreaming Format Descriptor */
    /* Uncompressed Video YUV422 */
    0x1B,   /* bLength */
    0x24,   /* bDescriptorType */
    0x04,   /* bDescriptorSubType */
    0x01,   /* bFormatIndex */
    0x02,   /* bNumFrameDescriptors */
#if (UVC_Color_Format == UVC_Format_YUY2)
	//YUYV
    0x59, 0x55, 0x59, 0x32, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71,
#endif
#if (UVC_Color_Format == UVC_Format_RGB565)
	//RGB565
	0x7b, 0xeb, 0x36, 0xe4, 0x4f, 0x52, 0xce, 0x11, 0x9f, 0x53, 0x00, 0x20, 0xaf, 0x0b, 0xa7, 0x70,
#endif
	0x10,
    0x02,   /* bDefaultFrameIndex */
    0x00,   /* bAspectRatioX */
    0x00,   /* bAspectRatioY */
    0x00,   /* bmInterlaceFlags */
    0x00,   /* bCopyProtect */

    /* Class-specific VideoStreaming Frame 1 Descriptor */
    0x32,           //bLength
    0x24,           //bDescriptorType
    0x05,           //bDescriptorSubType
    0x01,           //bFrameIndex           // Modified Here
    0x00,           //bmCapabilities
    0x40, 0x01,         //wWidth->320
    0xF0, 0x00,         //wHeight->240
    0x00, 0x58, 0x02, 0x00,     //dwMinBitRate
    0x00, 0x50, 0x46, 0x00,     //dwMaxBitRate
    0x00, 0x58, 0x02, 0x00,     //dwMaxVideoFrameBufSize
    0x15, 0x16, 0x05, 0x00,     //dwDefaultFrameInterval
    0x06,           //bFrameIntervalType
    0x15, 0x16, 0x05, 0x00,     //dwMinFrameInterval
    0x20, 0xA1, 0x07, 0x00,     //dwMinFrameInterval
    0x2A, 0x2C, 0x0A, 0x00,     //dwMinFrameInterval
    0x40, 0x42, 0x0F, 0x00,     //dwMinFrameInterval
    0x80, 0x84, 0x1E, 0x00,     //dwMinFrameInterval
    0x80, 0x96, 0x98, 0x00,      //dwMinFrameInterval

    /* Class-specific VideoStreaming Frame 2 Descriptor */
    0x32,           //bLength
    0x24,           //bDescriptorType
    0x05,           //bDescriptorSubType
    0x02,           //bFrameIndex           // Modified Here
    0x00,           //bmCapabilities
    0xA0, 0x00,          //wWidth->160
    0x78, 0x00,          //wHeight->120
    0x00, 0x96, 0x00, 0x00,     //dwMinBitRate
    0x00, 0x94, 0x11, 0x00,     //dwMaxBitRate
    0x00, 0x96, 0x00, 0x00,     //dwMaxVideoFrameBufSize
    0x15, 0x16, 0x05, 0x00,     //dwDefaultFrameInterval
    0x06,           //bFrameIntervalType
    0x15, 0x16, 0x05, 0x00,     //dwMinFrameInterval
    0x20, 0xA1, 0x07, 0x00,     //dwMinFrameInterval
    0x2A, 0x2C, 0x0A, 0x00,     //dwMinFrameInterval
    0x40, 0x42, 0x0F, 0x00,     //dwMinFrameInterval
    0x80, 0x84, 0x1E, 0x00,     //dwMinFrameInterval
    0x80, 0x96, 0x98, 0x00,      //dwMinFrameInterval

    /* Class-specific VideoStreaming Frame 3 Descriptor */
    0x32,           /* bLength */
    0x24,           /* bDescriptorType */
    0x05,           /* bDescriptorSubType */
    0x03,           /* bFrameIndex */
    0x00,           /* bmCapabilities */

    0x80, 0x02,         //wWidth->640
    0xE0, 0x01,         //wHeight->480
    0x00, 0x60, 0x09, 0x00,     //dwMinBitRate
    0x00, 0x40, 0x19, 0x01,     //dwMaxBitRate
    0x00, 0x60, 0x09, 0x00,     //dwMaxVideoFrameBufSize
    0x15, 0x16, 0x05, 0x00,     //dwDefaultFrameInterval
    0x06,           //bFrameIntervalType
    0x15, 0x16, 0x05, 0x00,     //dwMinFrameInterval
    0x20, 0xA1, 0x07, 0x00,     //dwMinFrameInterval
    0x2A, 0x2C, 0x0A, 0x00,     //dwMinFrameInterval
    0x40, 0x42, 0x0F, 0x00,     //dwMinFrameInterval
    0x80, 0x84, 0x1E, 0x00,     //dwMinFrameInterval
    0x80, 0x96, 0x98, 0x00,      //dwMinFrameInterval

	/* Class-specific Still Image Frame Descriptor */
    0x12,           //bLength
    0x24,           //bDescriptorType
    0x03,           //bDescriptorSubType
    0x00,           //bEndpointAddress
    0x03,           //bNumImageSizePatterns
    0x80, 0x02,         //wWidth
    0xE0, 0x01,         //wHeight
    0x40, 0x01,         //wWidth
    0xF0, 0x00,         //wHeight
    0xA0, 0x00,         //wWidth
    0x78, 0x00,         //wHeight
    0x00,            //bNumCompressionPtn

    /* Color Matching Descriptor */
    0x06,           //bLength
    0x24,           //bDescriptorType
    0x0D,           //bDescriptorSubType
    0x01,           //bColorPrimaries
    0x01,           //bTransferCharacteristics
    0x04,            //bMatrixCoefficients

    /* Standard VideoStreaming Interface Descriptor  (Num 1, Alt 3) */
    0x09,   //bLength
    0x04,   //bDescriptorType
    0x01,   //bInterfceNumber
    0x01,   //bAlternateSetting
    0x01,   //bNumEndpoints
    0x0E,   //bInterfaceClass
    0x02,   //bInterfaceSubClass
    0x00,   //bInterfaceProtocol
    0x00,   //iInterface

    /* Standard VideoStreaming Iso Video Data Endpoint Descriptor */
    0x07,   /* bLength */
    0x05,   /* bDescriptorType */
    (ISO_IN_EP_NUM | EP_INPUT),    /* bEndpointAddress */
    0x05,   /* bmAttributes */
    //0x90, 0x12,
    EPA_MAX_PKT_SIZE & 0x00FF,
    (EPA_MAX_PKT_SIZE & 0xFF00) >> 8,
    0x01,   /* bInterval */

    /* Standard VideoStreaming Interface Descriptor  (Num 1, Alt 4) */
    0x09,   //bLength
    0x04,   //bDescriptorType
    0x01,   //bInterfceNumber
    0x02,   //bAlternateSetting
    0x01,   //bNumEndpoints
    0x0E,   //bInterfaceClass
    0x02,   //bInterfaceSubClass
    0x00,   //bInterfaceProtocol
    0x00,    //iInterface

    /* Standard VideoStreaming Iso Video Data Endpoint Descriptor */
    0x07,   /* bLength */
    0x05,   /* bDescriptorType */
    (ISO_IN_EP_NUM | EP_INPUT),    /* bEndpointAddress */
    0x05,   /* bmAttributes */
    //0x90, 0x12,
    EPA_MAX_PKT_SIZE & 0x00FF,
    (EPA_MAX_PKT_SIZE & 0xFF00) >> 8,
    0x01,   /* bInterval */

};


/*!<USB Language String Descriptor */
uint8_t gu8StringLang[4] __attribute__((aligned(DCACHE_LINE_SIZE))) =
{
    4,              /* bLength */
    DESC_STRING,    /* bDescriptorType */
    0x09, 0x04
};

/*!<USB Vendor String Descriptor */
uint8_t gu8VendorStringDesc[] __attribute__((aligned(DCACHE_LINE_SIZE))) =
{
    16,
    DESC_STRING,
    'N', 0, 'u', 0, 'v', 0, 'o', 0, 't', 0, 'o', 0, 'n', 0
};

/*!<USB Product String Descriptor */
uint8_t gu8ProductStringDesc[] __attribute__((aligned(DCACHE_LINE_SIZE))) =
{
    22,             /* bLength          */
    DESC_STRING,    /* bDescriptorType  */
    'U', 0, 'S', 0, 'B', 0, ' ', 0, 'C', 0, 'a', 0, 'm', 0, 'e', 0, 'r', 0, 'a', 0
};

uint8_t *gpu8UsbString[4] =
{
    gu8StringLang,
    gu8VendorStringDesc,
    gu8ProductStringDesc,
    NULL,
};

uint8_t *gu8UsbHidReport[3] =
{
    NULL,
    NULL,
    NULL,
};

uint32_t gu32UsbHidReportLen[3] =
{
    0,
    0,
    0,
};

S_HSUSBD_INFO_T gsHSInfo =
{
    gu8DeviceDescriptor,
    gu8ConfigDescriptor,
    gpu8UsbString,
    gu8QualifierDescriptor,
    gu8ConfigDescriptorFS,
    gu8OtherConfigDescriptorHS,
    gu8OtherConfigDescriptorFS,
    NULL,
    gu8UsbHidReport,
    gu32UsbHidReportLen,
    NULL
};


