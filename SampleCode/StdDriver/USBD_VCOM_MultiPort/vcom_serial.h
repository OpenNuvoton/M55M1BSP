/******************************************************************************
 * @file     vcom_serial.h
 * @version  V3.00
 * @brief    USBD virtual COM header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_CDC_H__
#define __USBD_CDC_H__

/* Define the vendor id and product id */
#define USBD_VID        0x0416
#define USBD_PID        0x50A1

/*!<Define CDC Class Specific Request */
#define SET_LINE_CODE           0x20
#define GET_LINE_CODE           0x21
#define SET_CONTROL_LINE_STATE  0x22

/*-------------------------------------------------------------*/
/* Define EP maximum packet size */
#define EP0_MAX_PKT_SIZE    64
#define EP1_MAX_PKT_SIZE    EP0_MAX_PKT_SIZE // 64

#define EP2_MAX_PKT_SIZE    64
#define EP3_MAX_PKT_SIZE    64
#define EP4_MAX_PKT_SIZE    8

#define EP5_MAX_PKT_SIZE    8
#define EP6_MAX_PKT_SIZE    64
#define EP7_MAX_PKT_SIZE    64
//---------------------
#define EP8_MAX_PKT_SIZE    8
#define EP9_MAX_PKT_SIZE    64
#define EP10_MAX_PKT_SIZE   64

#define EP11_MAX_PKT_SIZE   8
#define EP12_MAX_PKT_SIZE   64
#define EP13_MAX_PKT_SIZE   64

#define EP14_MAX_PKT_SIZE   8
#define EP15_MAX_PKT_SIZE   64
#define EP16_MAX_PKT_SIZE   64

#define EP17_MAX_PKT_SIZE   8
#define EP18_MAX_PKT_SIZE   64
#define EP19_MAX_PKT_SIZE   64

#define EP20_MAX_PKT_SIZE   8
#define EP21_MAX_PKT_SIZE   64
#define EP22_MAX_PKT_SIZE   64
//---------------------

#define SETUP_BUF_BASE      0
#define SETUP_BUF_LEN       8
#define EP0_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP0_BUF_LEN         EP0_MAX_PKT_SIZE //64
#define EP1_BUF_BASE        (SETUP_BUF_BASE + SETUP_BUF_LEN)
#define EP1_BUF_LEN         EP1_MAX_PKT_SIZE //64

#define EP2_BUF_BASE        (EP1_BUF_BASE + EP1_BUF_LEN)
#define EP2_BUF_LEN         EP2_MAX_PKT_SIZE
#define EP3_BUF_BASE        (EP2_BUF_BASE + EP2_BUF_LEN)
#define EP3_BUF_LEN         EP3_MAX_PKT_SIZE
#define EP4_BUF_BASE        (EP3_BUF_BASE + EP3_BUF_LEN)
#define EP4_BUF_LEN         EP4_MAX_PKT_SIZE
#define EP5_BUF_BASE        (EP4_BUF_BASE + EP4_BUF_LEN)
#define EP5_BUF_LEN         EP5_MAX_PKT_SIZE
#define EP6_BUF_BASE        (EP5_BUF_BASE + EP5_BUF_LEN)
#define EP6_BUF_LEN         EP6_MAX_PKT_SIZE
#define EP7_BUF_BASE        (EP6_BUF_BASE + EP6_BUF_LEN)
#define EP7_BUF_LEN         EP7_MAX_PKT_SIZE

#define EP8_BUF_BASE        (EP7_BUF_BASE + EP7_BUF_LEN)
#define EP8_BUF_LEN         EP8_MAX_PKT_SIZE
#define EP9_BUF_BASE        (EP8_BUF_BASE + EP8_BUF_LEN)
#define EP9_BUF_LEN         EP9_MAX_PKT_SIZE
#define EP10_BUF_BASE        (EP9_BUF_BASE + EP9_BUF_LEN)
#define EP10_BUF_LEN         EP10_MAX_PKT_SIZE

#define EP11_BUF_BASE        (EP10_BUF_BASE + EP10_BUF_LEN)
#define EP11_BUF_LEN         EP11_MAX_PKT_SIZE
#define EP12_BUF_BASE        (EP11_BUF_BASE + EP11_BUF_LEN)
#define EP12_BUF_LEN         EP12_MAX_PKT_SIZE
#define EP13_BUF_BASE        (EP12_BUF_BASE + EP12_BUF_LEN)
#define EP13_BUF_LEN         EP13_MAX_PKT_SIZE

#define EP14_BUF_BASE        (EP13_BUF_BASE + EP13_BUF_LEN)
#define EP14_BUF_LEN         EP14_MAX_PKT_SIZE
#define EP15_BUF_BASE        (EP14_BUF_BASE + EP14_BUF_LEN)
#define EP15_BUF_LEN         EP15_MAX_PKT_SIZE
#define EP16_BUF_BASE        (EP15_BUF_BASE + EP15_BUF_LEN)
#define EP16_BUF_LEN         EP16_MAX_PKT_SIZE

#define EP17_BUF_BASE        (EP16_BUF_BASE + EP16_BUF_LEN)
#define EP17_BUF_LEN         EP17_MAX_PKT_SIZE
#define EP18_BUF_BASE        (EP17_BUF_BASE + EP17_BUF_LEN)
#define EP18_BUF_LEN         EP18_MAX_PKT_SIZE
#define EP19_BUF_BASE        (EP18_BUF_BASE + EP18_BUF_LEN)
#define EP19_BUF_LEN         EP19_MAX_PKT_SIZE

#define EP20_BUF_BASE        (EP19_BUF_BASE + EP19_BUF_LEN)
#define EP20_BUF_LEN         EP20_MAX_PKT_SIZE
#define EP21_BUF_BASE        (EP20_BUF_BASE + EP20_BUF_LEN)
#define EP21_BUF_LEN         EP21_MAX_PKT_SIZE
#define EP22_BUF_BASE        (EP21_BUF_BASE + EP21_BUF_LEN)
#define EP22_BUF_LEN         EP22_MAX_PKT_SIZE

/* Define the interrupt In EP number */
#define BULK_IN_EP_NUM      0x02
#define BULK_OUT_EP_NUM     0x02
#define INT_IN_EP_NUM       0x01

#define BULK_IN_EP_NUM_1    0x04
#define BULK_OUT_EP_NUM_1   0x04
#define INT_IN_EP_NUM_1     0x03

#define BULK_IN_EP_NUM_2    0x06
#define BULK_OUT_EP_NUM_2   0x06
#define INT_IN_EP_NUM_2     0x05

#define BULK_IN_EP_NUM_3    0x08
#define BULK_OUT_EP_NUM_3   0x08
#define INT_IN_EP_NUM_3     0x07

#define BULK_IN_EP_NUM_4    0x0A
#define BULK_OUT_EP_NUM_4   0x0A
#define INT_IN_EP_NUM_4     0x09

#define BULK_IN_EP_NUM_5    0x0C
#define BULK_OUT_EP_NUM_5   0x0C
#define INT_IN_EP_NUM_5     0x0B

#define BULK_IN_EP_NUM_6    0x0E
#define BULK_OUT_EP_NUM_6   0x0E
#define INT_IN_EP_NUM_6     0x0D

/* Define Descriptor information */
#define USBD_SELF_POWERED               0
#define USBD_REMOTE_WAKEUP              0
#define USBD_MAX_POWER                  50  /* The unit is in 2mA. ex: 50 * 2mA = 100mA */

/************************************************/
/* for CDC class */
/* Line coding structure
  0-3 dwDTERate    Data terminal rate (baudrate), in bits per second
  4   bCharFormat  Stop bits: 0 - 1 Stop bit, 1 - 1.5 Stop bits, 2 - 2 Stop bits
  5   bParityType  Parity:    0 - None, 1 - Odd, 2 - Even, 3 - Mark, 4 - Space
  6   bDataBits    Data bits: 5, 6, 7, 8, 16  */

#pragma pack(push)
#pragma pack(1)
typedef struct
{
    uint32_t  u32DTERate;     /* Baud rate    */
    uint8_t   u8CharFormat;   /* stop bit     */
    uint8_t   u8ParityType;   /* parity       */
    uint8_t   u8DataBits;     /* data bits    */
} STR_VCOM_LINE_CODING;
#pragma pack(pop)

#define VCOM_CNT    7


/*-------------------------------------------------------------*/
extern volatile int8_t g_i8BulkOutReady0;
extern STR_VCOM_LINE_CODING g_LineCoding0;
extern uint16_t g_u16CtrlSignal0;
extern volatile uint16_t g_u16ComRbytes0;
extern volatile uint16_t g_u16ComRhead0;
extern volatile uint16_t g_u16ComRtail0;
extern volatile uint16_t g_u16ComTbytes0;
extern volatile uint16_t g_u16ComThead0;
extern volatile uint16_t g_u16ComTtail0;
extern uint8_t *g_pu8RxBuf0;
extern uint32_t g_u32RxSize0;
extern uint32_t g_u32TxSize0;

extern volatile int8_t g_i8BulkOutReady1;
extern STR_VCOM_LINE_CODING g_LineCoding1;
extern uint16_t g_u16CtrlSignal1;
extern volatile uint16_t g_u16ComRbytes1;
extern volatile uint16_t g_u16ComRhead1;
extern volatile uint16_t g_u16ComRtail1;
extern volatile uint16_t g_u16ComTbytes1;
extern volatile uint16_t g_u16ComThead1;
extern volatile uint16_t g_u16ComTtail1;
extern uint8_t *g_pu8RxBuf1;
extern uint32_t g_u32RxSize1;
extern uint32_t g_u32TxSize1;

extern volatile int8_t g_i8BulkOutReady2;
extern STR_VCOM_LINE_CODING g_LineCoding2;
extern uint16_t g_u16CtrlSignal2;
extern volatile uint16_t g_u16ComRbytes2;
extern volatile uint16_t g_u16ComRhead2;
extern volatile uint16_t g_u16ComRtail2;
extern volatile uint16_t g_u16ComTbytes2;
extern volatile uint16_t g_u16ComThead2;
extern volatile uint16_t g_u16ComTtail2;
extern uint8_t *g_pu8RxBuf2;
extern uint32_t g_u32RxSize2;
extern uint32_t g_u32TxSize2;

extern volatile int8_t g_i8BulkOutReady3;
extern STR_VCOM_LINE_CODING g_LineCoding3;
extern uint16_t g_u16CtrlSignal3;
extern volatile uint16_t g_u16ComRbytes3;
extern volatile uint16_t g_u16ComRhead3;
extern volatile uint16_t g_u16ComRtail3;
extern volatile uint16_t g_u16ComTbytes3;
extern volatile uint16_t g_u16ComThead3;
extern volatile uint16_t g_u16ComTtail3;
extern uint8_t *g_pu8RxBuf3;
extern uint32_t g_u32RxSize3;
extern uint32_t g_u32TxSize3;

extern volatile int8_t g_i8BulkOutReady4;
extern STR_VCOM_LINE_CODING g_LineCoding4;
extern uint16_t g_u16CtrlSignal4;
extern volatile uint16_t g_u16ComRbytes4;
extern volatile uint16_t g_u16ComRhead4;
extern volatile uint16_t g_u16ComRtail4;
extern volatile uint16_t g_u16ComTbytes4;
extern volatile uint16_t g_u16ComThead4;
extern volatile uint16_t g_u16ComTtail4;
extern uint8_t *g_pu8RxBuf4;
extern uint32_t g_u32RxSize4;
extern uint32_t g_u32TxSize4;

extern volatile int8_t g_i8BulkOutReady5;
extern STR_VCOM_LINE_CODING g_LineCoding5;
extern uint16_t g_u16CtrlSignal5;
extern volatile uint16_t g_u16ComRbytes5;
extern volatile uint16_t g_u16ComRhead5;
extern volatile uint16_t g_u16ComRtail5;
extern volatile uint16_t g_u16ComTbytes5;
extern volatile uint16_t g_u16ComThead5;
extern volatile uint16_t g_u16ComTtail5;
extern uint8_t *g_pu8RxBuf5;
extern uint32_t g_u32RxSize5;
extern uint32_t g_u32TxSize5;

extern volatile int8_t g_i8BulkOutReady6;
extern STR_VCOM_LINE_CODING g_LineCoding6;
extern uint16_t g_u16CtrlSignal6;
extern volatile uint16_t g_u16ComRbytes6;
extern volatile uint16_t g_u16ComRhead6;
extern volatile uint16_t g_u16ComRtail6;
extern volatile uint16_t g_u16ComTbytes6;
extern volatile uint16_t g_u16ComThead6;
extern volatile uint16_t g_u16ComTtail6;
extern uint8_t *g_pu8RxBuf6;
extern uint32_t g_u32RxSize6;
extern uint32_t g_u32TxSize6;

extern uint8_t volatile g_u8Suspend;

/*-------------------------------------------------------------*/
void VCOM_Init(void);
void VCOM_ClassRequest(void);

void EP2_Handler(void);
void EP3_Handler(void);
void EP7_Handler(void);
void EP6_Handler(void);

void EP10_Handler(void);
void EP9_Handler(void);

void EP13_Handler(void);
void EP12_Handler(void);

void EP16_Handler(void);
void EP15_Handler(void);

void EP19_Handler(void);
void EP18_Handler(void);

void EP22_Handler(void);
void EP21_Handler(void);

void VCOM_LineCoding(uint8_t u8Port);
void VCOM_TransferData(void);

#endif  /* __USBD_CDC_H_ */
