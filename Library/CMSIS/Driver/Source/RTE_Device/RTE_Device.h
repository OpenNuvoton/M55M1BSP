
/**************************************************************************//**
 * @file     RTE_Device.h
 * @version  V1.00
 * @brief    RTE Device Configuration for Nuvoton M55M1
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef __RTE_DEVICE_H
#define __RTE_DEVICE_H


#define GPIO_PORT(num) \
    ((num == 0) ? GPIOA : \
     (num == 1) ? GPIOB : \
     (num == 2) ? GPIOC : \
     (num == 3) ? GPIOD : \
     (num == 4) ? GPIOE : \
     (num == 5) ? GPIOF : \
     (num == 6) ? GPIOG : \
     (num == 7) ? GPIOH : \
     (num == 8) ? GPIOI : \
     NULL)

#if __has_include("RTE_Device_GPIO.h")
    #include "RTE_Device_GPIO.h"
#endif
#if __has_include("RTE_Device_GPIO_LPGPIO.h")
    #include "RTE_Device_GPIO_LPGPIO.h"
#endif
#if __has_include("RTE_Device_USART.h")
    #include "RTE_Device_USART.h"
#endif
#if __has_include("RTE_Device_I2C.h")
    #include "RTE_Device_I2C.h"
#endif
#if __has_include("RTE_Device_SAI.h")
    #include "RTE_Device_SAI.h"
#endif
#if __has_include("RTE_Device_SPI.h")
    #include "RTE_Device_SPI.h"
#endif
#if __has_include("RTE_Device_MCI.h")
    #include "RTE_Device_MCI.h"
#endif
#if __has_include("RTE_Device_CAN.h")
    #include "RTE_Device_CAN.h"
#endif
#if __has_include("RTE_Device_ETH_MAC.h")
    #include "RTE_Device_ETH_MAC.h"
#endif
#if __has_include("RTE_Device_USBD.h")
    #include "RTE_Device_USBD.h"
#endif
#if __has_include("RTE_Device_USBH.h")
    #include "RTE_Device_USBH.h"
#endif
#if __has_include("RTE_Device_Flash.h")
    #include "RTE_Device_Flash.h"
#endif


#endif  /* __RTE_DEVICE_H */
