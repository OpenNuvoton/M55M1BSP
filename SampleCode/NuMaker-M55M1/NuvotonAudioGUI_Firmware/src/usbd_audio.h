/******************************************************************************
 * @file     usbd_audio.h
 * @version  V1.00
 * @brief    USBD audio codec header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_UAC_H__
#define __USBD_UAC_H__

/*!<Includes */
#include "audio_class.h"
#include "hid_transfer.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Macro define                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
/*!<Define HID Class Specific Request */
#define HID_SET_REPORT                      0x09
#define HID_SET_IDLE                        0x0A
#define HID_SET_PROTOCOL                    0x0B

#define MUTE_CONTROL                        0x01
#define VOLUME_CONTROL                      0x02

/* USB Device Classes */
#define USB_DEVICE_CLASS_RESERVED           0x00
#define USB_DEVICE_CLASS_AUDIO              0x01
#define USB_DEVICE_CLASS_COMMUNICATIONS     0x02

/* Audio Interface Subclass Codes */
#define AUDIO_SUBCLASS_UNDEFINED            0x00
#define AUDIO_SUBCLASS_AUDIOCONTROL         0x01
#define AUDIO_SUBCLASS_AUDIOSTREAMING       0x02
#define AUDIO_SUBCLASS_MIDISTREAMING        0x03

/* Audio Interface Protocol Codes */
#define AUDIO_PROTOCOL_UNDEFINED            0x00

/* bmAttributes in Configuration Descriptor */
#define USB_CONFIG_POWERED_MASK             0x40
#define USB_CONFIG_BUS_POWERED              0x80
#define USB_CONFIG_SELF_POWERED             0xC0
#define USB_CONFIG_REMOTE_WAKEUP            0x20

/* bMaxPower in Configuration Descriptor */
#define USB_CONFIG_POWER_MA(mA)             ((mA)/2)

extern S_HSUSBD_INFO_T gsHSInfo;

#ifdef __cplusplus
}
#endif

#endif  /* __USBD_UAC_H_ */
