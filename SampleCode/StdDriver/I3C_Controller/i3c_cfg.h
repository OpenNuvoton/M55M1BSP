/**************************************************************************//**
 * @file     i3c_cfg.c
 * @version  V3.00
 * @brief    i3c_cfg header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/
#define DA_ASSIGNED_MODE        (g_DA_ASSIGNED_MODE)

// <o> Select Device Role
//      <0=> Device as Target Role
//      <1=> Device as Controller Role
// <i> Select the default Device Role.
#define DEVICE_CONTROLLER_ROLE  (1)


// <o> SDR SCL Frequency
//      <1000000=>  1M
//      <2000000=>  2M
//      <4000000=>  4M
//      <6000000=>  6M
//      <8000000=>  8M
//      <10000000=> 10M
//      <12500000=> 12.5M
// <i> Select the SDR SCL frequency
#define DEVICE_SDR_FREQ         (10000000)


// <o> Set DMA Function ON/OFF
//      <0=> DMA Function OFF
//      <1=> DMA Function ON
// <i> Select DMA function for I3C wrtie and read opeartion.
#define DEVICE_DMA_ENABLED      (1)


// <c1> Enable Interrupt Debug Log
// <i> Show detail interrupt status on UART.
//#define DGBINT          printf
//</c>
#ifndef DGBINT
    #define DGBINT(...)
#endif

// *** <<< end of configuration section >>>    ***


#define I3C0_CTR_DA         (0x4A)
#define I3C0_CTR_MID        (0x5A00UL)
#define I3C0_CTR_PID        (0xA13579BDUL)

#define I3C0_TGT_SA         (0x68)
#define I3C0_TGT_MID        (0xA501UL)
#define I3C0_TGT_PID        (0xB2468ACEUL)

extern const uint8_t        g_TgtDA[];
extern const uint8_t        g_TgtSA[];
extern volatile uint32_t    g_I3CDevRx[I3C_DEVICE_RX_BUF_CNT], g_I3CDevTx[I3C_DEVICE_TX_BUF_CNT];
extern volatile uint32_t    g_DA_ASSIGNED_MODE;
