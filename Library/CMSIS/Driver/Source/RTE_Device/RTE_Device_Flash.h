/**************************************************************************//**
 * @file     RTE_Device_Flash.h
 * @version  V1.00
 * @brief    RTE Device Configuration for Nuvoton CMSIS Flash driver
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __RTE_DEVICE_FLASH_H
#define __RTE_DEVICE_FLASH_H

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

//  <h> Flash Region in APROM
//  <i> Specify the valid Flash Region that Flash Driver can read, write and erase.
//  <i> This region should not overlap with the application and Start Offset + Byte Size must <= APROM size.
//      <o> Start Offset <0x0-0x200000:0x2000>
//      <i> Configure start offset of Flash Region from 0x0 ~ 0x200000 and must align with 0x2000.
#define RTE_FLASH_START_OFFSET      0x00100000

//      <o> Byte Size <0x2000-0x200000:0x2000>
//      <i> Configure total byte size of Flash Region from 0x2000 ~ 0x200000 and must align with 0x2000.
#define RTE_FLASH_BYTE_SIZE         0x00100000
//  </h>

//  <e> Enable PDMA for Read Operation
#define RTE_FLASH_READ_PDMA         0
//    <e> PDMA Port and Channel
//      <i> Uncheck to allocate a free PDMA channel dynamically during runtime.
//      <i> Check to specify a fixed port and channel.
#define RTE_FLASH_READ_PDMA_FIXED   0
//      <o> Number <0=>0 <1=>1
//        <i>  Select a PDMA Port.
#define RTE_FLASH_PDMA_PORT         1
//      <o> Channel (0~15) <0-15>
//        <i>  Select a PDMA Channel.
#define RTE_FLASH_PDMA_CHANNEL      0
//    </e>
//  </e>
//-------- <<< End of Configuration Wizard in Context Menu >>> --------------------

#endif  // __RTE_DEVICE_FLASH_H
