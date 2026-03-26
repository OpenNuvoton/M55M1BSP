/******************************************************************************
 * @file     i2c_process.h
 * @version  V1.00
 * @brief    NAU88C22 I2C process header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __I2C_PROCESS_H__
#define __I2C_PROCESS_H__

#include "NuMicro.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Extern Global Variables                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern uint8_t g_u8SlaveAddr;
extern volatile uint16_t g_u16I2CRxDataLen;
extern volatile uint16_t g_u16I2CRxDataCnt;
extern volatile uint16_t g_u16I2CTxDataLen;
extern volatile uint16_t g_u16I2CTxDataCnt;
extern volatile uint16_t g_u16I2CTimeOutCnt;
extern volatile uint8_t g_u8I2CTimeOutTestflag;
extern volatile uint8_t g_u8I2CTimeOut;
extern volatile uint8_t g_u8I2CRead;
extern volatile uint8_t g_u8CheckConnection;
extern volatile uint8_t g_u8NackCnt;

extern void (*pI2CProtocolCallback)(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_Callback(void);
void Codec_I2CCallback(void);

#ifdef __cplusplus
}
#endif

#endif  // __I2C_PROCESS_H__
