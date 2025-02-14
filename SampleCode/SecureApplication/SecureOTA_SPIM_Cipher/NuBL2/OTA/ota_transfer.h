/**************************************************************************//**
 * @file     ota_transfer.h
 * @version  V1.00
 * @brief    OTA transfer header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __OTA_TRANSFER_H__
#define __OTA_TRANSFER_H__

/*****************************************/
/* Set GPIO and UART pins for boards     */
/*****************************************/
#define ESP_WIFI_UART_IRQn          UART6_IRQn
#define ESP_WIFI_UART_IRQHandler    UART6_IRQHandler
#define ESP_WIFI_UART_MODULE        UART6_MODULE
#define ESP_WIFI_UART_PORT          UART6       // UART port connected to Wi-Fi module
//#define ESP_USART_TX_PORT           PC
//#define ESP_USART_TX_PIN            12
//#define ESP_USART_RX_PORT           PC
//#define ESP_USART_RX_PIN            11
#define ESP_WIFI_RESET_PIN          PE12
#define LED_OFF                     PG15        // LED
//#define FW_UPDATE_OFF               PD12

#define IOCTL_INIT      { \
        CLK_EnableModuleClock(GPIOE_MODULE); \
        CLK_EnableModuleClock(GPIOG_MODULE); \
        PE->MODE = (PE->MODE & ~(0x3 << (12 * 2))) | (GPIO_MODE_OUTPUT << (12 * 2)); \
        PG->MODE = (PG->MODE & ~(0x3 << (15 * 2))) | (GPIO_MODE_OUTPUT << (15 * 2)); \
    }

#define INPUT_WIFI_PROMPT    0   /* Enable interface of manual input Wi-Fi configure */
#define PERIODIC_CHK_NEW_VER 0   /* Enable checking new firmware version periodically */
/* Wi-Fi network settings, replace with your settings */
#define WIFINAME            "eagle"
#define WIFIPASS            "12345678"
#define OTA_SRV_IP          "192.168.50.224"
#define OTA_SRV_PORT        (1111U)

#ifdef __cplusplus
extern "C"
{
#endif

extern volatile uint32_t g_u32SendbytesLen;
extern volatile uint8_t  g_u8SendbytesFlag;
extern volatile uint8_t  g_u8ResetFlag;
extern volatile uint8_t  g_u8DisconnFlag;

/**
  * @brief      Init hardware for transfer task
  * @param      None
  * @retval     None
  * @details    This API is used for init hardware configure of transfer task.
  */
void Transfer_Init(void);

/**
  * @brief Transfer task process
  * @param      None
  * @retval     0             Success
  * @retval     others        Failed
  * @details    Transfer task process.
  */
int8_t Transfer_Process(void);

/**
  * @brief        Send transfer data
  * @param[in]    pu8TxBuf        The buffer to send the data
  * @param[in]    u32Len          The data lengths
  * @return       None
  * @details      The function is to write data into send buffer to transmit data.
  */
void Transfer_SendBytes(uint8_t pu8TxBuf[], uint32_t u32Len);

/**
  * @brief        Read received transfer data
  * @param        None
  * @return       None
  * @details      The function is used to read Rx data from Wi-Fi module.
  */
void Transfer_WiFiProcess(void);

/**
  * @brief      System tick handler for transfer task
  * @param[in]  u32Ticks  System ticks
  * @retval     0         success
  * @retval     Other     fail
  * @details    The function is the system tick handler for transfer task.
  */
uint8_t Transfer_SysTickProcess(uint32_t u32Ticks);

/**
  * @brief        Disconnect transfer connection
  * @param        None
  * @return       None
  * @details      The function is used to disconnect transfer connection.
  */
void Transfer_ConnClose(void);

/**
  * @brief        Set Reset flag for transfer task
  * @param        None
  * @return       None
  * @details      The function is used to set reset flag for transfer task.
  */
void Transfer_SetResetFlag(void);

/**
  * @brief        Set disconnect flag for transfer task
  * @param        None
  * @return       None
  * @details      The function is used to set disconnect flag for transfer task.
  */
void Transfer_SetDisconnFlag(void);

#ifdef __cplusplus
}
#endif

#endif /* __OTA_TRANSFER_H__ */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
