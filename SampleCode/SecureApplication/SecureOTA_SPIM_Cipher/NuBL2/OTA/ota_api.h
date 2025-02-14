/**************************************************************************//**
 * @file     ota_api.h
 * @version  V1.00
 * @brief    OTA porting API header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __OTA_API_H__
#define __OTA_API_H__

#include "NuMicro.h"

#define BUF_SIZE                128
#define ALIGN_BUFF_SIZE         64

#ifdef __cplusplus
extern "C"
{
#endif

extern uint8_t g_au8SendBuf[BUF_SIZE];

/**
  * @brief      OTA task routine
  * @param      None
  * @retval     0             Success
  * @retval     others        Failed
  * @details    OTA task routine
  */
int8_t OTA_API_TaskProcess(void);

/**
  * @brief       Disconnect transfer connection
  * @param       None
  * @return      None
  * @details     The function is used to disconnect transfer connection.
  */
void OTA_API_TransferConnClose(void);

/**
  * @brief       Send frame data
  * @param[in]   pu8TxBuf        The buffer to send the data
  * @param[in]   u32Len          The data lengths
  * @return      None
  * @details     The function is to write frame data into send buffer to transmit data.
  */
void OTA_API_SendFrame(uint8_t *pu8Buff, uint32_t u32Len);

/**
  * @brief      OTA commands handler
  * @param[in]  pu8Buff        The pointer of packet buffer
  * @param[in]  u32Len         Valind data length
  * @param[in]  u32StartIdx    Valind data index in packet buffer
  * @param[in]  u32ValidLen    Valind data length
  * @return     None
  * @details    OTA commands handler
  */
int8_t OTA_API_RecvCallBack(uint8_t *pu8Buff, uint32_t u32Len, uint32_t u32StartIdx, uint32_t u32ValidLen);

/**
  * @brief      Init function for any hardware requirements(optional)
  * @param[in]  u32HSI        PLL Output Clock Frequency
  * @return     None
  * @details    This funcfion is to init function for any hardware requirements.
  */
void OTA_API_Init(uint32_t u32HSI);

/**
  * @brief      Get page size of flash
  * @param      None
  * @retval     0             Success
  * @retval     others        Failed
  * @details    This function is to get page size of flash.
  */
uint32_t OTA_API_GetFlashPageSize(void);

/**
  * @brief      Erase flash region
  * @param[in]  u32FlashAddr  Flash address
  * @retval     0             Success
  * @retval     others        Failed
  * @details    This function is to erase flash region.
  */
uint8_t OTA_API_EraseFlash(uint32_t u32FlashAddr);

/**
  * @brief      Write flash data
  * @param[in]  u32FlashAddr  Flash address
  * @param[in]  u32Data       data
  * @retval     0             Success
  * @retval     others        Failed
  * @details    This function is to write flash data.
  */
uint8_t OTA_API_WriteFlash(uint32_t u32FlashAddr, uint32_t u32Data);

/**
  * @brief      Get flag of firmware upgrade done
  * @param      None
  * @return     Flag of firmware upgrade done
  * @details    This function is to get flag of firmware upgrade done.
  */
uint8_t OTA_API_GetFwUpgradeDone(void);

/**
  * @brief      System tick handler for transfer task
  * @param[in]  u32Ticks  System ticks
  * @retval     0         success
  * @retval     Other     fail
  * @details    The function is the system tick handler for transfer task.
  */
uint8_t OTA_API_SysTickProcess(uint32_t u32Ticks);

/**
  * @brief        Read received transfer data
  * @param        None
  * @return       None
  * @details      The function is used to read Rx data from Wi-Fi module.
  */
void OTA_API_WiFiProcess(void);

/**
  * @brief        Set Reset flag for transfer task
  * @param        None
  * @return       None
  * @details      The function is used to set reset flag for transfer task.
  */
void OTA_API_SetResetFlag(void);


#ifdef __cplusplus
}
#endif

#endif /* __OTA_API_H__ */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
