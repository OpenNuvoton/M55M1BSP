/**************************************************************************//**
 * @file     RTE_Device_USBD.h
 * @version  V1.00
 * @brief    RTE Device Configuration for Nuvoton M55M1 USB device
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

/*
  CMSIS Driver Instance | Hardware Resource
  :---------------------|:-----------------------
  Driver_USBD0          | USBD0
  Driver_USBD1          | HSUSBD0
*/

#ifndef __RTE_DEVICE_USBD_H
#define __RTE_DEVICE_USBD_H

//   <o0.0> USBD0 (Full Speed Universal Serial Bus Device)
//   <i> Configuration settings for Driver_USBD0 in component ::CMSIS Driver:USB Device
#define RTE_USBD0                       1
//   <o0.0> USBD1 (High Speed Universal Serial Bus Device)
//   <i> Configuration settings for Driver_USBD1 in component ::CMSIS Driver:USB Device
#define RTE_USBD1                       1
#endif
