/******************************************************************************
 * @file     hid_transfer.h
 * @version  V1.00
 * @brief    HID transfer header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __HID_TRANSFER_H__
#define __HID_TRANSFER_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Macro define                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
/*!<Define HID Class Specific Request */
#define GET_REPORT                          0x01
#define GET_IDLE                            0x02
#define GET_PROTOCOL                        0x03
#define SET_REPORT                          0x09
#define SET_IDLE                            0x0A
#define SET_PROTOCOL                        0x0B

/*!<USB HID Interface Class protocol */
#define HID_NONE                            0x00
#define HID_KEYBOARD                        0x01
#define HID_MOUSE                           0x02

/*!<USB HID Class Report Type */
#define HID_RPT_TYPE_INPUT                  0x01
#define HID_RPT_TYPE_OUTPUT                 0x02
#define HID_RPT_TYPE_FEATURE                0x03

/* HID Transfer definition */
#define HIDTRANS_READ                       0xFC
#define HIDTRANS_WRITE                      0xF3
#define HIDTRANS_RESPOND                    0xF5
#define HIDTRANS_ACK                        0xF0
#define HIDTRANS_CHECK                      0xFA
#define HIDTRANS_STATUS                     0xF7

#define HIDTRANS_CMD_BONGIOVI               0x13
#define HIDTRANS_CMD_NUVOTON                0x1C
#define HIDTRANS_CMD_DPS_SETTING            0x1F
#define HIDTRANS_CMD_DPS_STRING             0x2F
#define HIDTRANS_CMD_DATA_CONFIG            0x1A
#define HIDTRANS_CMD_DATA_PRESET            0x1B

#define HIDTRANS_CMD_I2C                    0x1E
#define HIDTRANS_CMD_FW_VERSION             0x30
#define HIDTRANS_CMD_Monitor_data           0x32
#define HIDTRANS_CMD_BOARD_STATE            0x33
#define HIDTRANS_CMD_BOARD_NUM              0x34
#define HIDTRANS_CMD_I2C_CLK_SET            0x35

#define HIDTRANS_CMD_CODEC_I2S_SETTING      0x10
#define HIDTRANS_CMD_CODEC_DATA_9BIT        0x1D
#define HIDTRANS_CMD_CODEC_DATA_8BIT        0x11
#define HIDTRANS_CMD_CODEC_DATA_16BIT       0x12
#define HIDTRANS_CMD_CODEC_BURST_DATA_9BIT  0x20
#define HIDTRANS_CMD_CODEC_BURST_DATA_8BIT  0x21
#define HIDTRANS_CMD_CODEC_BURST_DATA_16BIT 0x22

/*-------------------------------------------------------------*/
/* Define the EP number */
#define INT_IN_EP_NUM                       0x06
#define INT_OUT_EP_NUM                      0x07

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
/* Endpoint handlers */
void EPG_Handler(void);
void EPH_Handler(void);

extern void HIDTrans_ProcessCommand(uint8_t *pu8Buffer, uint32_t u32BufferLen);

#ifdef __cplusplus
}
#endif

#endif  // __HID_TRANSFER_H__
