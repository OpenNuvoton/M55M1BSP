/**************************************************************************//**
 * @file     ota_api.c
 * @version  V1.00
 * @brief    OTA porting API demo code
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "ota.h"
#include "ota_transfer.h"
#include "ff.h"

//#define printf(...)

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
extern uint32_t CyclesPerUs;
extern volatile uint8_t g_u8SendbytesFlag;
extern volatile uint8_t g_u8ResetFlag;
extern volatile uint8_t g_u8DisconnFlag;
extern uint8_t g_au8SendBuf[BUF_SIZE];
extern volatile uint32_t g_u32SendbytesLen;

/* Init NuBL2 global variables for library */
/**
  * @brief      Init function for any hardware requirements(optional)
  * @param[in]  u32HSI        PLL Output Clock Frequency
  * @return     None
  * @details    This funcfion is to init function for any hardware requirements.
  */
void OTA_SysValueInit(uint32_t u32HSI)
{
    uint8_t au8SendBuf[] = "CONNECT0\r\n";

    CyclesPerUs = (u32HSI / 1000000UL);
    g_u8SendbytesFlag = 1;
    g_u8ResetFlag = 0;
    g_u8DisconnFlag = 0;
    memcpy(g_au8SendBuf, au8SendBuf, sizeof(au8SendBuf));
    g_u32SendbytesLen = sizeof(au8SendBuf);
}

/**
  * @brief        Set Reset flag for transfer task
  * @param        None
  * @return       None
  * @details      The function is used to set reset flag for transfer task.
  */
void OTA_API_SetResetFlag(void)
{
    Transfer_SetResetFlag();
}

/**
  * @brief      OTA task routine
  * @param      None
  * @retval     0             Success
  * @retval     others        Failed
  * @details    Transfer task routine
  */
int8_t OTA_API_TaskProcess(void)
{
    /* Process transfer task */
    Transfer_Process();

    return 0;
}

/**
  * @brief        Disconnect transfer connection
  * @param        None
  * @return       None
  * @details      The function is used to disconnect transfer connection.
  */
void OTA_API_TransferConnClose(void)
{
    /* Set disconnect flag for transfer task */
    Transfer_SetDisconnFlag();
}

/**
  * @brief        Send frame data
  * @param[in]    pu8TxBuf        The buffer to send the data
  * @param[in]    u32Len          The data lengths
  * @return       None
  * @details      The function is to write frame data into send buffer to transmit data.
  */
void OTA_API_SendFrame(uint8_t *pu8Buff, uint32_t u32Len)
{
    /* Write data to send buffer */
    Transfer_SendBytes(pu8Buff, u32Len);
}

/**
  * @brief      Get flag of firmware upgrade done
  * @param      None
  * @return     Flag of firmware upgrade done
  * @details    This function is to get flag of firmware upgrade done.
  */
uint8_t OTA_API_GetFwUpgradeDone(void)
{
    return OTA_GetFwUpgradeDone();
}

/**
  * @brief      OTA commands handler
  * @param[in]  pu8Buff        The pointer of packet buffer
  * @param[in]  u32Len         Valind data length
  * @param[in]  u32StartIdx    Valind data index in packet buffer
  * @param[in]  u32ValidLen    Valind data length
  * @return     None
  * @details    OTA commands handler
  */
int8_t OTA_API_RecvCallBack(uint8_t *pu8Buff, uint32_t u32Len, uint32_t u32StartIdx, uint32_t u32ValidLen)
{
    OTA_CallBackHandler(pu8Buff, u32Len, u32StartIdx, u32ValidLen);

    return 0;
}

/**
  * @brief      Init function for any hardware requirements(optional)
  * @param[in]  u32HSI        PLL Output Clock Frequency
  * @return     None
  * @details    This funcfion is to init function for any hardware requirements.
  */
void OTA_API_Init(uint32_t u32HSI)
{
    /* Init some global variables of NuBL2 when now is running NuBL32 firmware. */
    OTA_SysValueInit(u32HSI);

    /* Init hardware for transfer task */
    Transfer_Init();
}

/**
  * @brief      Get page size of flash
  * @param      None
  * @retval     0             Success
  * @retval     others        Failed
  * @details    This function is to get page size of flash.
  */
uint32_t OTA_API_GetFlashPageSize()
{
    return (uint32_t)FMC_FLASH_PAGE_SIZE;
}

/**
  * @brief      Erase flash region
  * @param[in]  u32FlashAddr  Flash address
  * @retval     0             Success
  * @retval     others        Failed
  * @details    This function is to erase flash region
  */
uint8_t OTA_API_EraseFlash(uint32_t u32FlashAddr)
{
    uint32_t u8Status;

    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    DEBUG_MSG("FMC_Erase(0x%08X)\n", u32FlashAddr);

    if (FMC_Erase(u32FlashAddr))
        u8Status = STATUS_FAILED;
    else
        u8Status = STATUS_SUCCESS;

    return u8Status;
}

/**
  * @brief      Write flash data
  * @param[in]  u32FlashAddr  Flash address
  * @param[in]  u32Data       data
  * @retval     0             Success
  * @retval     others        Failed
  * @details    This function is to write flash data
  */
uint8_t OTA_API_WriteFlash(uint32_t u32FlashAddr, uint32_t u32Data)
{
    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    DEBUG_MSG("FMC_Write(0x%08X, 0x%08X)\n", u32FlashAddr, u32Data);

    if (FMC_Write(u32FlashAddr, u32Data) != 0)
        return STATUS_FAILED;

    return STATUS_SUCCESS;
}

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

DWORD get_fattime(void)
{
    DWORD tmr;

    tmr = 0x00000;

    return tmr;
}

/**
  * @brief      System tick handler for transfer task
  * @param[in]  u32Ticks  System ticks
  * @retval     0         success
  * @retval     Other     fail
  * @details    The function is the system tick handler for transfer task.
  */
uint8_t OTA_API_SysTickProcess(uint32_t u32Ticks)
{
    return Transfer_SysTickProcess(u32Ticks);
}

/**
  * @brief        Read received transfer data
  * @param        None
  * @return       None
  * @details      The function is used to read Rx data from Wi-Fi module.
  */
void OTA_API_WiFiProcess(void)
{
    Transfer_WiFiProcess();
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
